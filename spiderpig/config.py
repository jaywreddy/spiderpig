"""The build: what to make and how, as one validated, hashable record.

:class:`BuildConfig` is the key every cache uses (side designs, baked
``.glb`` files, the MuJoCo model) and the argument every stage takes. It
validates itself on construction: the linkage exists (and walks, when a
robot is asked for), the module is one of the linkage's, the leg phases
(radians, one per leg) fit the module and are ``None`` when they are the
module's own, the proportions name the linkage's parameters, lengths are
positive, and only the overrides that differ from the defaults are kept,
in the linkage's order. So a design has exactly one config however it was
asked for, and a bad one fails where it is named (:class:`ParamError`, a
``ValueError``).

The CLIs share ``--linkage`` / ``--phases`` (degrees) / ``--proportion
NAME=VALUE`` and the build options (:func:`add_design_args`,
:func:`add_build_args`, :func:`config_from_args`); the server takes
``linkage=``, ``module=``, ``phases=`` and ``p.NAME=``
(:func:`design_from_query`).
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping
from dataclasses import dataclass, field

from spiderpig import construction, linkage, servos
from spiderpig.construction.base import Params
from spiderpig.hardware.catalog import sheet_thickness


class ParamError(ValueError):
    """Bad design parameters (unknown linkage, module or proportion, wrong phase count, ...)."""


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class BuildConfig:
    """What to build and how. Construction keys refer to :mod:`construction` registries."""

    linkage: str = "klann"            # see linkage.available()
    module: str = "quad"              # legs per side: one of the linkage's modules
    robot: bool = True                # two mirrored sides, servos back to back in one frame
    phases: tuple[float, ...] | None = None          # crank phase per leg (rad); None = module's
    proportions: tuple[tuple[str, float], ...] = ()  # overrides of the linkage's params
    sheet: str = "acrylic_3mm"        # catalog item for the sheet stock (sets the layer pitch)
    thickness: float | None = None    # override the sheet's nominal thickness
    servo: str = servos.DEFAULT
    pillar: str = "printed"           # frame pivots
    pin: str = "printed"              # pivots between links
    crank: str = "printed"
    params: Params = field(default_factory=Params)

    def __post_init__(self) -> None:
        try:
            lk = linkage.get(self.linkage)
        except KeyError as e:
            raise ParamError(e.args[0]) from None
        if self.robot and lk.kind != "walker":
            raise ParamError(f"{lk.key} is a mechanism, not a walker: it has no feet to walk on, "
                             f"so it builds one side (robot=False; --side-only on the command "
                             f"line) (walkers: {', '.join(linkage.available('walker'))})")
        if self.module not in lk.leg_modules:
            raise ParamError(f"unknown module {self.module!r}; have {list(lk.leg_modules)}")
        if self.servo not in servos.available():
            raise ParamError(f"unknown servo {self.servo!r}; have {servos.available()}")
        object.__setattr__(self, "phases", self._phases(lk))
        object.__setattr__(self, "proportions", self._proportions(lk))

    def _phases(self, lk: linkage.Linkage) -> tuple[float, ...] | None:
        """Radians per leg; ``None`` for the module's own (mod 2 pi), however given."""
        if self.phases is None:
            return None
        legs = lk.leg_modules[self.module]
        try:
            values = tuple(float(p) for p in self.phases)
        except (TypeError, ValueError):
            raise ParamError(f"phases must be numbers, got {list(self.phases)!r}") from None
        if len(values) != len(legs):
            raise ParamError(f"{self.module} has {len(legs)} legs per side, got {len(values)} "
                             "phases")
        if not all(math.isfinite(p) for p in values):
            raise ParamError(f"phases must be finite numbers, got {list(values)}")
        if all(abs(_wrap(a - ph)) < 1e-9 for a, (_, ph) in zip(values, legs, strict=True)):
            return None
        return values

    def _proportions(self, lk: linkage.Linkage) -> tuple[tuple[str, float], ...]:
        """Overrides that differ from the defaults, validated, in the linkage's order."""
        overrides = dict(self.proportions)
        unknown = sorted(set(overrides) - set(lk.params))
        if unknown:
            raise ParamError(f"unknown {lk.key} proportions {unknown}; have {list(lk.params)}")
        out = []
        for name, default in lk.params.items():
            if name not in overrides:
                continue
            try:
                v = float(overrides[name])
            except (TypeError, ValueError):
                raise ParamError(f"proportion {name} must be a number, "
                                 f"got {overrides[name]!r}") from None
            if not math.isfinite(v):
                raise ParamError(f"proportion {name} must be a finite number, "
                                 f"got {overrides[name]!r}")
            if name not in lk.signed and v <= 0:       # angles and coordinates may be <= 0
                raise ParamError(f"proportion {name} is a length and must be > 0, got {v:g}")
            if abs(v - float(default)) > 1e-12:
                out.append((name, v))
        return tuple(out)

    # -- what the stages read ---------------------------------------------------

    @property
    def lk(self) -> linkage.Linkage:
        return linkage.get(self.linkage)

    @property
    def legs(self) -> linkage.LegList:
        """``(orientation, crank phase in rad)`` per leg of one side."""
        legs = self.lk.leg_modules[self.module]
        if self.phases is None:
            return legs
        return tuple((o, ph) for (o, _), ph in zip(legs, self.phases, strict=True))

    @property
    def pitch(self) -> float:
        """The layer pitch: the sheet's thickness."""
        return sheet_thickness(self.sheet, self.thickness)

    @property
    def values(self) -> tuple[float, ...]:
        """Every parameter of the linkage, overrides applied."""
        return self.lk.values(dict(self.proportions))

    @property
    def is_default(self) -> bool:
        """The linkage's default design (the module's phases, its proportions), built the
        default way."""
        return self == BuildConfig(linkage=self.linkage, module=self.module, robot=self.robot)

    @property
    def key(self) -> str:
        """A file-name stem: ``klann_quad_robot``, plus a hash for anything but a default."""
        stem = f"{self.linkage}_{self.module}_{'robot' if self.robot else 'side'}"
        if self.is_default:
            return stem
        return f"{stem}_{hashlib.sha1(repr(self).encode()).hexdigest()[:12]}"

    def design_json(self) -> dict:
        """The design parameters as JSON: linkage, module, phases (deg) and all proportions."""
        return {
            "linkage": self.linkage,
            "module": self.module,
            "phases_deg": [round(math.degrees(ph), 6) for _, ph in self.legs],
            "proportions": dict(zip(self.lk.params, self.values, strict=True)),
        }


# ---------------------------------------------------------------------------
# From the command line and the query string
# ---------------------------------------------------------------------------


def parse_phases(text: str) -> tuple[float, ...]:
    """``"0,180,90,270"`` -> radians (degrees in; the count is checked by the config)."""
    try:
        values = [float(s) for s in str(text).split(",")]
    except ValueError:
        raise ParamError(f"phases must be comma-separated numbers (degrees), got {text!r}") \
            from None
    if not all(math.isfinite(v) for v in values):
        raise ParamError(f"phases must be finite numbers, got {text!r}")
    return tuple(math.radians(v) for v in values)


def parse_proportion(item: str) -> tuple[str, float]:
    """``"DF=2.6"`` -> ``("DF", 2.6)`` (the config checks the name)."""
    name, sep, value = str(item).partition("=")
    name = name.strip()
    if not sep or not name:
        raise ParamError(f"expected NAME=VALUE, got {item!r}")
    try:
        return name, float(value)
    except ValueError:
        raise ParamError(f"proportion {name} must be a number, got {value!r}") from None


DEFAULT_MODULE = "quad"           # a walker's, when none is asked for


def default_module(key: str) -> str:
    """The module a design gets when none is asked for: a walker's ``quad`` (its first
    module if it has no quad), a mechanism's one module (:class:`ParamError` for an
    unknown linkage)."""
    try:
        lk = linkage.get(key)
    except KeyError as e:
        raise ParamError(e.args[0]) from None
    if DEFAULT_MODULE in lk.leg_modules:
        return DEFAULT_MODULE
    return next(iter(lk.leg_modules))


def default_robot(key: str) -> bool:
    """Whether a design builds the two-sided robot when nothing says: a walker does, a
    mechanism is one side (it has no feet to walk on)."""
    try:
        return linkage.get(key).kind == "walker"
    except KeyError as e:
        raise ParamError(e.args[0]) from None


def add_design_args(p) -> None:
    """``--linkage``, ``--module``, ``--phases`` and ``--proportion`` on an argparse parser.
    ``--module`` defaults to ``None``: :func:`config_from_args` fills in the linkage's
    (:func:`default_module`), so a mechanism needs no ``--module single``."""
    import argparse

    def arg(fn):
        def convert(text):
            try:
                return fn(text)
            except ParamError as e:
                raise argparse.ArgumentTypeError(str(e)) from None
        convert.__name__ = fn.__name__
        return convert

    p.add_argument("--linkage", choices=linkage.available(), default=linkage.DEFAULT,
                   help=f"the linkage (default {linkage.DEFAULT})")
    p.add_argument("--module", default=None,
                   help="legs per side (the linkage's modules): single, double (mirrored "
                   "pair), decker (two legs on one crankshaft), quad (two mirrored deckers). "
                   f"Default: a walker's {DEFAULT_MODULE}, a mechanism's single")
    p.add_argument("--phases", type=arg(parse_phases), default=None, metavar="DEG,...",
                   help="crank phase of every leg of a side, in degrees (default: the "
                   "module's, e.g. quad 0,180,90,270)")
    names = "; ".join(f"{k}: {', '.join(linkage.get(k).params)}" for k in linkage.available())
    p.add_argument("--proportion", type=arg(parse_proportion), action="append", default=None,
                   metavar="NAME=VALUE",
                   help=f"override one of the linkage's parameters (repeatable; lengths in mm "
                   f"or the linkage's unit, angles in degrees): {names}")


def add_build_args(p) -> None:
    """The build options: ``--servo``, the constructions, the sheet."""
    d = BuildConfig()
    axles, cranks = sorted(construction.AXLES), sorted(construction.CRANKS)
    p.add_argument("--servo", default=d.servo, choices=servos.available(),
                   help=f"servo model (default {d.servo})")
    p.add_argument("--pillar", default=d.pillar, choices=axles,
                   help=f"construction of the frame pivots (default {d.pillar})")
    p.add_argument("--pin", default=d.pin, choices=axles,
                   help=f"construction of the pivots between links (default {d.pin})")
    p.add_argument("--crank", default=d.crank, choices=cranks,
                   help=f"crank construction (default {d.crank})")
    p.add_argument("--sheet", default=d.sheet, help=f"sheet stock catalog item (default {d.sheet})")
    p.add_argument("--thickness", type=float, default=None,
                   help="measured sheet thickness in mm (default: the sheet's nominal)")


def config_from_args(args, **fixed) -> BuildConfig:
    """The config the parsed arguments ask for (:class:`ParamError` if they don't fit).

    ``fixed`` overrides fields (``robot=False``); build options the parser lacks keep
    their defaults. A module not given (``None``) is the linkage's
    (:func:`default_module`); ``robot`` not given, or given as ``None``, is the
    linkage's kind's (:func:`default_robot`: a mechanism builds one side).
    """
    d = BuildConfig()
    fields = {k: getattr(args, k, getattr(d, k)) for k in ("linkage", "sheet", "thickness",
                                                            "servo", "pillar", "pin", "crank")}
    fields.update(phases=getattr(args, "phases", None),
                  proportions=tuple(getattr(args, "proportion", None) or ()))
    fields.update(fixed)
    fields["module"] = (fixed.get("module") or getattr(args, "module", None)
                        or default_module(fields["linkage"]))
    if fields.get("robot") is None:
        fields["robot"] = default_robot(fields["linkage"])
    return BuildConfig(**fields)


def design_from_query(query: Mapping, **fixed) -> BuildConfig:
    """The config a query string asks for: ``linkage``, ``module`` (default: the linkage's,
    :func:`default_module`), ``phases`` (degrees, comma-separated) and ``p.<NAME>=<value>``;
    other keys are ignored."""
    items = query.multi_items() if hasattr(query, "multi_items") else query.items()
    proportions = [parse_proportion(f"{k[2:]}={v}") for k, v in items if k.startswith("p.")]
    key = query.get("linkage") or linkage.DEFAULT
    kw = {"linkage": key,
          "module": query.get("module") or default_module(key),
          "phases": parse_phases(query["phases"]) if query.get("phases") else None,
          "proportions": tuple(proportions)}
    return BuildConfig(**{**kw, **fixed})
