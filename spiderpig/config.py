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


DEFAULT_CRANKS = {"walker": "bolt", "mechanism": "bolt"}
"""The crank a design gets when it names none, per kind (:func:`default_crank`): the bolt
crank for both (the crank study of 2026-10-03; mechanisms since 2026-10-04), which is the
single-plate web crank with hex-standoff crankpins in hex pockets (the user's decision of
2026-10-04 (1), ``BoltCrank.pin="hex"``; jam SF >= 2.25 on 0.100 in 6061-T6). The round
friction-clamped standoff it replaced is ``bolt_round``. Which construction a design gets is
data here, not code."""

LINKAGE_CRANKS: dict[str, str] = {
    # Empty since the hex-standoff crank merged (2026-10-04): hoecken_pantograph and
    # dwell_rocker, which kept the keyed crank (jam SF 0.64) because the round crankpin's top
    # screw head over the hub plate needed 2.5 mm of the 1.83 mm horn spacer, plan with it
    # (9 layers each): DriveGroup.spacer adds a layer for that head (BoltCrank.hub_head_need)
    # or the spacer caps a pin wholly under it (hub_capped).
    #
    # TrotBot's heel and toe: b7 sweeps across the crank at O and passes crankpin J1 at 10.2
    # mm; the hex crankpin's 8.5 mm printed sleeve needs 11.2 mm there (static stage stops).
    # The round 6 mm standoff of bolt_round clears it and plans (14 layers, single module;
    # its friction clamp rates jam SF 3.57 at factor 1, UNVERIFIED coefficients). The
    # alternative, the hex crank at unit 12 (x1.14), changes the linkage's size.
    "trotbot_heel": "bolt_round",
    "trotbot_toe": "bolt_round",
    # klann_lego: its b1 (6061, the user's decision 3) carries the crank bore 8.5 mm from
    # pin C's hole; the hex sleeve's 8.8 mm bore leaves 2.02 mm between them, under 1 x the
    # 3.175 mm sheet (a cut-rule error). The round standoff's 6.3 mm bore leaves 3.27 mm
    # (its end grown round the bore, plates.rider_bosses, keeps the edge too).
    "klann_lego": "bolt_round",
}
"""Per linkage: the crank it gets when it names none, where :data:`DEFAULT_CRANKS`'
doesn't plan (each with why)."""


MODULE_CRANKS: dict[tuple[str, str], str] = {
    # The Strider's decker and quad find no plan with the hex-standoff crank (the merge of
    # 2026-10-04: none in 600 CPU s on ao-server; "no crank route passes", the 8.5 mm hex
    # sleeve's post blocking the links that pass the crankpins, "no way past the layer of
    # b8"). The round standoff plans them (decker 17 layers in 11 s, quad 25 in 50 s, on the
    # 0.100 in 6061 crank sheet), rated as its friction clamp (UNVERIFIED coefficients).
    ("strider", "decker"): "bolt_round",
    ("strider", "quad"): "bolt_round",
}
"""Per (linkage, module): the crank it gets when it names none, ahead of
:data:`LINKAGE_CRANKS` (each with why)."""


def default_crank(lk: linkage.Linkage, module: str = "") -> str:
    """The crank construction ``lk`` (in ``module``) gets when the config names none: its
    module's (:data:`MODULE_CRANKS`), else its own (:data:`LINKAGE_CRANKS`), else its kind's
    (:data:`DEFAULT_CRANKS`)."""
    return MODULE_CRANKS.get((lk.key, module),
                             LINKAGE_CRANKS.get(lk.key, DEFAULT_CRANKS[lk.kind]))


@dataclass(frozen=True)
class BuildConfig:
    """What to build and how. Construction keys refer to :mod:`construction` registries."""

    linkage: str = linkage.DEFAULT    # see linkage.available()
    module: str = ""                  # legs per side: one of the linkage's modules ("": its
    #                                   default, config.default_module: Strider's double)
    robot: bool = True                # two mirrored sides, servos back to back in one frame
    phases: tuple[float, ...] | None = None          # crank phase per leg (rad); None = module's
    proportions: tuple[tuple[str, float], ...] = ()  # overrides of the linkage's params
    sheet: str = "acrylic_3mm"        # the default sheet (the links, rings, deck): it sets
    #                                   the layer pitch (spiderpig.materials)
    thickness: float | None = None    # override the sheet's nominal thickness
    frame_sheet: str = "al5052_2mm"     # the frame and centre plates (aluminium: acrylic
    #                                     can't take their load; the user's call 2026-10-04):
    #                                     0.080 in 5052, the thinnest stock that passes
    #                                     (materials.thinnest_sheet("frame"); tests pin it)
    crank_sheet: str = "al6061_2p5mm"   # the crank's laser-cut plates: 0.100 in 6061-T6, the
    #                                     thinnest whose hex pockets hold the jam twist at SF 2
    #                                     (materials.thinnest_sheet("crank"), 2026-10-04)
    heads: str = "best"               # fasteners' heads: "sink" into the layer beside their
    #                                   link, "gap" (a thin clearance gap where a link passes),
    #                                   "best" (sunk, else in gaps; stack.StackSpec)
    link_sheets: tuple[tuple[str, str], ...] | None = None   # link class -> sheet; None: the
    #                                   linkage's (materials.default_link_sheets: a Klann
    #                                   variant's foot links in 6061)
    servo: str = servos.DEFAULT
    pillar: str = "standoff"          # frame pivots: 6 mm round aluminium standoffs, spliced at
    #                                   plate rings (construction.pivots.standoff; "printed":
    #                                   the printed stepped pillar, the default before 2026-10-03)
    pin: str = "chicago"              # pivots between links: an M3 Chicago screw (4 mm barrel),
    #                                   rings, PTFE washer and shims (construction.pivots.chicago;
    #                                   "rod": 3 mm rod and push-on clips; "printed": snap pins)
    crank: str = ""                   # the crankshaft ("": default_crank: the linkage's own,
    #                                   LINKAGE_CRANKS, else its kind's, DEFAULT_CRANKS: "bolt",
    #                                   construction.crank.BoltCrank, single aluminium web plates
    #                                   on the crank sheet, hex-standoff crankpins in hex
    #                                   pockets; "bolt_round": the round friction-clamped
    #                                   standoff; "keyed", printed segments
    #                                   keyed by brass hex standoffs, the default before
    #                                   2026-10-03; "printed": clamp friction only)
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
        if not self.module:
            object.__setattr__(self, "module", default_module(lk.key))
        if not self.crank:
            object.__setattr__(self, "crank", default_crank(lk, self.module))
        if self.module not in lk.leg_modules:
            raise ParamError(f"unknown module {self.module!r}; have {list(lk.leg_modules)}")
        if self.servo not in servos.available():
            raise ParamError(f"unknown servo {self.servo!r}; have {servos.available()}")
        object.__setattr__(self, "phases", self._phases(lk))
        object.__setattr__(self, "proportions", self._proportions(lk))
        object.__setattr__(self, "link_sheets", self._link_sheets(lk))
        if self.heads not in ("best", "sink", "gap"):
            raise ParamError(f"heads must be best, sink or gap, got {self.heads!r}")
        from spiderpig.hardware.catalog import get

        for what in ("sheet", "frame_sheet", "crank_sheet"):
            key = getattr(self, what)
            try:
                if get(key).category != "sheet":
                    raise KeyError(key)
            except KeyError:
                raise ParamError(f"{what} {key!r} is not a sheet in the catalog") from None

    def _link_sheets(self, lk: linkage.Linkage) -> tuple[tuple[str, str], ...] | None:
        """Link class -> sheet, validated; ``None`` when it is the linkage's own."""
        if self.link_sheets is None:
            return None
        from spiderpig.hardware.catalog import get
        from spiderpig.materials import default_link_sheets

        out = {}
        for link, key in dict(self.link_sheets).items():
            if link not in lk.links:
                raise ParamError(f"link_sheets: {lk.key} has no link {link!r} "
                                 f"(have {list(lk.links)})")
            try:
                ok = get(key).category == "sheet"
            except KeyError:
                ok = False
            if not ok:
                raise ParamError(f"link_sheets: {key!r} is not a sheet in the catalog")
            out[link] = key
        if out == default_link_sheets(lk):
            return None
        return tuple(sorted(out.items()))

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
        """A file-name stem: ``strider_double_robot``, plus a hash for anything but a default."""
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


DEFAULT_MODULE = "quad"           # a walker's, when neither it nor the linkage says


def default_module(key: str) -> str:
    """The module a design gets when none is asked for: the linkage's own
    (``Linkage.default_module``: Strider's ``double``), else a walker's ``quad`` (its
    first module if it has no quad), a mechanism's one module (:class:`ParamError` for
    an unknown linkage)."""
    try:
        lk = linkage.get(key)
    except KeyError as e:
        raise ParamError(e.args[0]) from None
    if lk.default_module:
        return lk.default_module
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
                   f"Default: the linkage's ({linkage.DEFAULT}'s "
                   f"{default_module(linkage.DEFAULT)}; a walker's {DEFAULT_MODULE} otherwise, "
                   "a mechanism's single)")
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
    p.add_argument("--crank", default=None, choices=cranks,
                   help="crank construction (default: "
                        + ", ".join(f"{v} for a {k}" for k, v in DEFAULT_CRANKS.items())
                        + "; " + ", ".join(f"{v} for {k}" for k, v in LINKAGE_CRANKS.items())
                        + "; " + ", ".join(f"{v} for the {k[0]} {k[1]}"
                                           for k, v in MODULE_CRANKS.items())
                        + ")")
    p.add_argument("--sheet", default=d.sheet, help=f"sheet stock catalog item (default {d.sheet})")
    p.add_argument("--thickness", type=float, default=None,
                   help="measured sheet thickness in mm (default: the sheet's nominal)")
    p.add_argument("--frame-sheet", dest="frame_sheet", default=d.frame_sheet,
                   help=f"sheet of the frame and centre plates (default {d.frame_sheet})")
    p.add_argument("--crank-sheet", dest="crank_sheet", default=d.crank_sheet,
                   help=f"sheet of the crank's plates (default {d.crank_sheet})")
    p.add_argument("--heads", default=d.heads, choices=("best", "sink", "gap"),
                   help="fasteners' heads: sunk into a layer, in thin clearance gaps, or "
                        f"the lower of both plans (default {d.heads})")
    p.add_argument("--link-sheet", dest="link_sheet", action="append", default=None,
                   metavar="LINK=SHEET",
                   help="cut a link class from another sheet, e.g. b4=al6061_3p2mm "
                        "(repeatable; default: the linkage's, a Klann variant's foot links "
                        "in 6061 aluminium)")


def config_from_args(args, **fixed) -> BuildConfig:
    """The config the parsed arguments ask for (:class:`ParamError` if they don't fit).

    ``fixed`` overrides fields (``robot=False``); build options the parser lacks keep
    their defaults. A module not given (``None``) is the linkage's
    (:func:`default_module`); ``robot`` not given, or given as ``None``, is the
    linkage's kind's (:func:`default_robot`: a mechanism builds one side).
    """
    d = BuildConfig()
    fields = {k: getattr(args, k, getattr(d, k)) for k in ("linkage", "sheet", "thickness",
                                                            "servo", "pillar", "pin",
                                                            "frame_sheet", "crank_sheet",
                                                            "heads")}
    if getattr(args, "link_sheet", None):
        pairs = []
        for item in args.link_sheet:
            link, sep, key = str(item).partition("=")
            if not sep:
                raise ParamError(f"--link-sheet wants LINK=SHEET, got {item!r}")
            pairs.append((link.strip(), key.strip()))
        fields["link_sheets"] = tuple(pairs)
    fields["crank"] = getattr(args, "crank", None) or ""        # the kind's default
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
