"""Spec v1: what an agent asks the engine for, as a validated, versioned, JSON-able record.

A :class:`Spec` names a linkage (a registered key and overrides of its
parameters), a leg module (a named preset of the linkage), what to build it
from (materials, constructions, fit) and what the design must meet: every
metric under ``motion``, ``size`` and ``budget`` is a :class:`Target`
(``min`` / ``max`` / ``value`` with a weight, hard or soft). Only fields the
engine can verify today exist; anything else is an error, never ignored:
:func:`validate` returns every :class:`SpecError` (path, message, allowed
values, nearest key) and :meth:`Spec.from_dict` raises them as one
:class:`SpecErrors`.

Hard targets fail :func:`spiderpig.verify`; soft ones lower its score. The
defaults (:data:`TARGET_FIELDS`): physical limits (size, budget, the stack,
ground clearance) are hard, gait quality (stride, lift, speed, bob, slip,
tipping) soft; ``hard: true|false`` on a target overrides. What each metric
means is pinned once, in the same table (robot metrics come from the
quasi-static walk model, :mod:`walk`; ``lift_mm`` from one leg's foot path;
``speed_mm_s`` is the stride at the servo's no-load rpm, so its tier is
``estimated``).

:func:`spec_schema` is the JSON Schema of a spec document.
"""

from __future__ import annotations

import difflib
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, fields

from spiderpig import construction, linkage, servos
from spiderpig.config import DEFAULT_CRANKS, BuildConfig
from spiderpig.config import default_module as config_default_module
from spiderpig.construction.base import Params
from spiderpig.hardware.catalog import CATALOG
from spiderpig.hardware.catalog import _load as _load_catalog
from spiderpig.layout import DEFAULT_KERF

SPEC_VERSION = "1"
KINDS = ("walker", "mechanism")
OUTPUTS = ("step", "stl", "print", "dxf", "bom", "glb", "mjcf")
DEFAULT_OUTPUTS = ("step", "stl", "print", "dxf", "bom")
WILDCARDS = frozenset({"any", "*", "?", ""})
FIT_FIELDS = tuple(f.name for f in fields(Params))
SECTIONS = ("motion", "size", "budget")


# ---------------------------------------------------------------------------
# Targets and the metrics they may name
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Target:
    """A requirement on one metric: ``min`` and/or ``max``, or ``value`` (met within
    ``tol``; default 5 % of the value). ``hard``: a miss fails verification (``None``:
    the field's default, :data:`TARGET_FIELDS`); soft misses lower the score, weighted by
    ``weight``."""

    min: float | None = None
    max: float | None = None
    value: float | None = None
    tol: float | None = None
    weight: float = 1.0
    hard: bool | None = None

    def check(self, x: float) -> tuple[bool, float]:
        """``(met, miss)``: whether ``x`` meets the target and by how much it misses
        (0 when met, in the metric's unit)."""
        miss = 0.0
        if self.min is not None and x < self.min:
            miss = self.min - x
        if self.max is not None and x > self.max:
            miss = max(miss, x - self.max)
        if self.value is not None:
            tol = self.tol if self.tol is not None else max(abs(self.value) * 0.05, 1e-9)
            if abs(x - self.value) > tol:
                miss = max(miss, abs(x - self.value) - tol)
        return miss == 0.0, miss

    @property
    def scale(self) -> float:
        """The magnitude a miss is scored against (the bound it misses; 1 for a zero bound)."""
        for v in (self.value, self.max, self.min):
            if v is not None and v != 0:
                return abs(v)
        return 1.0

    def describe(self) -> str:
        parts = []
        if self.value is not None:
            tol = self.tol if self.tol is not None else abs(self.value) * 0.05
            parts.append(f"= {self.value:g} ± {tol:g}")
        if self.min is not None and self.max is not None:
            parts.append(f"{self.min:g}..{self.max:g}")
        elif self.min is not None:
            parts.append(f">= {self.min:g}")
        elif self.max is not None:
            parts.append(f"<= {self.max:g}")
        return ", ".join(parts)

    def to_dict(self, hard: bool | None = None) -> dict:
        out = {k: v for k, v in (("min", self.min), ("max", self.max), ("value", self.value),
                                 ("tol", self.tol)) if v is not None}
        if self.weight != 1.0:
            out["weight"] = self.weight
        h = self.hard if hard is None else hard
        if h is not None:
            out["hard"] = h
        return out


@dataclass(frozen=True)
class TargetField:
    """One metric a spec may target: its section, which kinds it applies to, unit, whether
    it is hard by default, where :mod:`spiderpig.verify` measures it, the evidence tier of
    that measurement, and what it means."""

    name: str
    section: str
    kinds: tuple[str, ...]
    unit: str
    hard: bool
    source: str
    tier: str
    doc: str

    @property
    def path(self) -> str:
        return f"{self.section}.{self.name}"


def _fields(*rows: tuple) -> dict[str, TargetField]:
    return {r[0]: TargetField(*r) for r in rows}


BOTH = ("walker", "mechanism")
WALKER, MECHANISM = ("walker",), ("mechanism",)

TARGET_FIELDS: dict[str, dict[str, TargetField]] = {
    "motion": _fields(
        ("stride_mm", "motion", WALKER, "mm/rev", False, "walk", "measured",
         "forward travel of the body per crank revolution, both sides at the same rate "
         "(walk.straight_walk_metrics stride_mm)"),
        ("lift_mm", "motion", WALKER, "mm", False, "foot_path", "measured",
         "one foot's vertical travel over a revolution (kinematic; the same for every leg)"),
        ("speed_mm_s", "motion", WALKER, "mm/s", False, "walk", "estimated",
         "stride x the servo's no-load rpm / 60 (no load: an estimate until a sim confirms it)"),
        ("bob_mm", "motion", WALKER, "mm", False, "walk", "measured",
         "range of the body's height over a revolution (walk model)"),
        ("slip_mm_per_rev", "motion", WALKER, "mm/rev", False, "walk", "measured",
         "RMS foot slip per crank revolution (walk model slip_rms_mm_per_rev)"),
        ("tipping_fraction", "motion", WALKER, "fraction", False, "walk", "measured",
         "fraction of the cycle the centre of mass falls outside the feet's support"),
        ("ground_clearance_mm", "motion", WALKER, "mm", True, "static", "measured",
         "the body's lowest point above the lowest foot point (SideDesign.ground_clearance_mm)"),
        ("transmission_angle_deg", "motion", BOTH, "deg", False, "check", "measured",
         "the least transmission angle over every loop closure of the program, folded about "
         "90° (a 140° angle is as poor as 40°; under about 40° a joint binds; "
         "StepCheck.angle_deg)"),
        ("stroke_mm", "motion", MECHANISM, "mm", False, "output", "measured",
         "the output point's travel along its fitted line (OutputCheck.stroke_mm)"),
        ("straightness_mm", "motion", MECHANISM, "mm", False, "output", "measured",
         "band across the line over the straight stretch (OutputCheck.straightness_mm)"),
        ("on_line_fraction", "motion", MECHANISM, "fraction", False, "output", "measured",
         "longest part of the turn within ON_LINE_MM of the line (OutputCheck.on_line)"),
        ("rotation_deg", "motion", MECHANISM, "deg", False, "output", "measured",
         "a translating platform's rotation over the cycle (OutputCheck.rotation_deg)"),
        ("swing_deg", "motion", MECHANISM, "deg", False, "output", "measured",
         "a rocker's swing (OutputCheck.swing_deg)"),
        ("dwell_deg", "motion", MECHANISM, "deg", False, "output", "measured",
         "crank degrees the output stands still within its tolerance (OutputCheck.dwell_deg)"),
    ),
    "size": _fields(
        ("stack_mm", "size", BOTH, "mm", True, "plan", "proven",
         "one side's stack, both frame plates included (StackPlan.height)"),
        ("mass_g", "size", BOTH, "g", True, "build", "measured",
         "mass of every part (volumes x densities, the servo's catalogued mass); the walk "
         "model's nominal mass before a build (estimated)"),
        ("envelope_x_mm", "size", BOTH, "mm", True, "build", "measured",
         "extent of the built machine along the walking axis x at the build's crank angle; "
         "the joints' sweep plus the plates before a build (estimated)"),
        ("envelope_y_mm", "size", BOTH, "mm", True, "build", "measured",
         "extent along y (up) at the build's crank angle; estimated before a build"),
        ("envelope_z_mm", "size", BOTH, "mm", True, "build", "measured",
         "extent along z (across the sides, the stack axis); estimated before a build"),
    ),
    "budget": _fields(
        ("cost_usd", "budget", BOTH, "USD", True, "bom", "estimated",
         "the BOM's purchase total at the catalog's preferred offers (unverified prices)"),
        ("print_g", "budget", BOTH, "g", True, "bom", "measured",
         "filament for the printed parts at 100 % infill"),
        ("sheets", "budget", BOTH, "sheets", True, "layout", "measured",
         "sheets of the stock the laser-cut parts pack onto"),
    ),
}


def target_field(section: str, name: str) -> TargetField:
    return TARGET_FIELDS[section][name]


OUTPUT_TARGETS = ("stroke_mm", "straightness_mm", "on_line_fraction", "rotation_deg",
                  "swing_deg", "dwell_deg")
_OUTPUT_METRICS: dict[str, tuple[str, ...]] = {}
# the budget section's one plain number: what to allow, in all, for the items the catalog
# doesn't price (they are otherwise left out of the total, which then can't confirm a max)
ALLOWANCE = "allowance_usd"


def output_metrics(lk) -> tuple[str, ...]:
    """The output metrics a mechanism measures (the others are ``None`` on its
    :class:`linkage.checks.OutputCheck`: a line has no dwell, a rocker no straightness),
    from its output check at the defaults, cached per linkage."""
    if lk.key not in _OUTPUT_METRICS:
        try:
            c = lk.output_check()
        except ValueError:          # a loop that can't close at the defaults: refuse nothing
            _OUTPUT_METRICS[lk.key] = OUTPUT_TARGETS
        else:
            attr = {"on_line_fraction": "on_line"}
            _OUTPUT_METRICS[lk.key] = tuple(n for n in OUTPUT_TARGETS
                                            if getattr(c, attr.get(n, n)) is not None)
    return _OUTPUT_METRICS[lk.key]


def effective_hard(t: Target, f: TargetField) -> bool:
    return f.hard if t.hard is None else t.hard


# ---------------------------------------------------------------------------
# The spec
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LinkageSpec:
    """A registered linkage (:func:`linkage.available`) and overrides of its parameters."""

    key: str
    params: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class LegsSpec:
    """A named leg module of the linkage (``single``, ``double``, ``decker``, ``quad`` or its
    own), the crank phase of each leg in degrees, and how many sides (2: the robot; 1: one
    side). ``None`` is inferred: a walker's ``quad`` on 2 sides, a mechanism's ``single``."""

    module: str | None = None
    phases_deg: tuple[float, ...] | None = None
    sides: int | None = None


@dataclass(frozen=True)
class MaterialsSpec:
    """Sheet stock (a catalog ``sheet`` item; it sets the layer pitch), a measured thickness
    overriding its nominal, and the servo (:func:`servos.available`)."""

    sheet: str | None = None
    thickness_mm: float | None = None
    servo: str | None = None
    frame_sheet: str | None = None      # the frame plates' (aluminium by default)
    crank_sheet: str | None = None      # the crank's plates'


@dataclass(frozen=True)
class ConstructionsSpec:
    """Constructions of the frame pivots, the link pins (:data:`construction.AXLES`) and
    the crank (:data:`construction.CRANKS`)."""

    pillar: str | None = None
    pin: str | None = None
    crank: str | None = None
    heads: str | None = None     # fasteners' heads: "sink", "gap" or "best" (stack.StackSpec)


@dataclass(frozen=True)
class FitSpec:
    """Part sizes and fits (:class:`construction.base.Params`, mm) plus the export's laser
    kerf and usable sheet size. ``None`` keeps the engine's default."""

    margin: float | None = None
    link_radius: float | None = None
    frame_radius: float | None = None
    min_wall: float | None = None
    running_fit: float | None = None
    glue_fit: float | None = None
    print_fit: float | None = None
    axle_d: float | None = None
    spacer_d: float | None = None
    neck_d: float | None = None
    head_d: float | None = None
    crankpin_d: float | None = None
    web_radius: float | None = None
    journal_d: float | None = None
    stub_d: float | None = None
    hub_thickness: float | None = None
    kerf_mm: float | None = None
    sheet_size_mm: tuple[float, float] | None = None

    def params(self) -> Params:
        given = {k: getattr(self, k) for k in FIT_FIELDS if getattr(self, k) is not None}
        return Params(**given)


assert set(FIT_FIELDS) <= {f.name for f in fields(FitSpec)}, "FitSpec must cover Params"


@dataclass(frozen=True)
class Spec:
    """What to design (see the module docstring). Build one with :meth:`from_dict`, which
    validates; :meth:`to_dict` gives it back as written (unset fields omitted)."""

    kind: str
    linkage: LinkageSpec
    version: str = SPEC_VERSION
    legs: LegsSpec = field(default_factory=LegsSpec)
    motion: dict[str, Target] = field(default_factory=dict)
    size: dict[str, Target] = field(default_factory=dict)
    materials: MaterialsSpec = field(default_factory=MaterialsSpec)
    constructions: ConstructionsSpec = field(default_factory=ConstructionsSpec)
    fit: FitSpec = field(default_factory=FitSpec)
    budget: dict[str, Target] = field(default_factory=dict)
    outputs: tuple[str, ...] = DEFAULT_OUTPUTS
    allowance_usd: float | None = None      # ``budget.allowance_usd``: for the unpriced items

    @classmethod
    def from_dict(cls, data: Mapping) -> Spec:
        """A validated spec; :class:`SpecErrors` lists everything wrong with ``data``."""
        errors = validate(data)
        if errors:
            raise SpecErrors(errors)
        return _build(data)

    def to_dict(self) -> dict:
        out: dict = {"version": self.version, "kind": self.kind,
                     "linkage": {"key": self.linkage.key}}
        if self.linkage.params:
            out["linkage"]["params"] = dict(self.linkage.params)
        legs = _drop_none({"module": self.legs.module, "sides": self.legs.sides,
                           "phases_deg": (list(self.legs.phases_deg)
                                          if self.legs.phases_deg is not None else None)})
        if legs:
            out["legs"] = legs
        for section in SECTIONS:
            targets = getattr(self, section)
            if targets:
                out[section] = {k: t.to_dict() for k, t in targets.items()}
        if self.allowance_usd is not None:
            out.setdefault("budget", {})[ALLOWANCE] = self.allowance_usd
        for name in ("materials", "constructions", "fit"):
            sub = _drop_none({f.name: getattr(getattr(self, name), f.name)
                              for f in fields(getattr(self, name))})
            if "sheet_size_mm" in sub:
                sub["sheet_size_mm"] = list(sub["sheet_size_mm"])
            if sub:
                out[name] = sub
        if tuple(self.outputs) != DEFAULT_OUTPUTS:
            out["outputs"] = list(self.outputs)
        return out

    def targets(self) -> list[tuple[TargetField, Target]]:
        """Every target of the spec with its field, hard/soft resolved."""
        out = []
        for section in SECTIONS:
            for name, t in getattr(self, section).items():
                f = target_field(section, name)
                out.append((f, Target(t.min, t.max, t.value, t.tol, t.weight,
                                      effective_hard(t, f))))
        return out


def _drop_none(d: dict) -> dict:
    return {k: v for k, v in d.items() if v is not None}


def _build(data: Mapping) -> Spec:
    """``data`` as a :class:`Spec` (validated already)."""
    lk = data["linkage"]
    legs = data.get("legs") or {}
    fit = dict(data.get("fit") or {})
    if fit.get("sheet_size_mm") is not None:
        fit["sheet_size_mm"] = tuple(float(v) for v in fit["sheet_size_mm"])
    return Spec(
        kind=data["kind"],
        linkage=LinkageSpec(lk["key"], {k: float(v) for k, v in (lk.get("params") or {}).items()}),
        version=str(data.get("version", SPEC_VERSION)),
        legs=LegsSpec(legs.get("module"),
                      None if legs.get("phases_deg") is None
                      else tuple(float(v) for v in legs["phases_deg"]),
                      None if legs.get("sides") is None else int(legs["sides"])),
        motion={k: _target(v) for k, v in (data.get("motion") or {}).items()},
        size={k: _target(v) for k, v in (data.get("size") or {}).items()},
        materials=MaterialsSpec(**(data.get("materials") or {})),
        constructions=ConstructionsSpec(**(data.get("constructions") or {})),
        fit=FitSpec(**fit),
        budget={k: _target(v) for k, v in (data.get("budget") or {}).items()
                if k != ALLOWANCE},
        outputs=tuple(data.get("outputs") or DEFAULT_OUTPUTS),
        allowance_usd=_opt((data.get("budget") or {}).get(ALLOWANCE)),
    )


def _target(d: Mapping) -> Target:
    return Target(min=_opt(d.get("min")), max=_opt(d.get("max")), value=_opt(d.get("value")),
                  tol=_opt(d.get("tol")), weight=float(d.get("weight", 1.0)),
                  hard=d.get("hard"))


def _opt(v):
    return None if v is None else float(v)


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SpecError:
    """One thing wrong with a spec: where (a dotted ``path``), what, the values allowed
    there (when they are a closed set) and the nearest allowed key or value."""

    path: str
    message: str
    allowed: tuple | None = None
    nearest: str | None = None

    def describe(self) -> str:
        out = f"{self.path}: {self.message}"
        if self.nearest:
            out += f" (did you mean {self.nearest!r}?)"
        if self.allowed:
            shown = ", ".join(str(a) for a in self.allowed[:12])
            more = "" if len(self.allowed) <= 12 else f", ... ({len(self.allowed)} in all)"
            out += f"; allowed: {shown}{more}"
        return out

    def to_dict(self) -> dict:
        return {"path": self.path, "message": self.message,
                "allowed": list(self.allowed) if self.allowed is not None else None,
                "nearest": self.nearest}


class SpecErrors(ValueError):
    """A spec that doesn't validate: ``errors`` lists every :class:`SpecError`."""

    def __init__(self, errors: Iterable[SpecError]):
        self.errors = list(errors)
        super().__init__("\n".join(e.describe() for e in self.errors))


def nearest(key: str, options: Iterable[str]) -> str | None:
    """The allowed key closest to ``key`` (a case-insensitive match first), or ``None``."""
    options = [str(o) for o in options]
    lower = {o.lower(): o for o in options}
    if str(key).lower() in lower:
        return lower[str(key).lower()]
    close = difflib.get_close_matches(str(key), options, n=1, cutoff=0.5)
    return close[0] if close else None


def sheet_keys() -> list[str]:
    _load_catalog()
    return sorted(k for k, it in CATALOG.items() if it.category == "sheet")


def module_keys() -> list[str]:
    """Every leg module any linkage offers (the union; :func:`validate` checks the exact
    ones)."""
    out: list[str] = []
    for key in linkage.available():
        for m in linkage.get(key).leg_modules:
            if m not in out:
                out.append(m)
    return out


class _Validator:
    def __init__(self) -> None:
        self.errors: list[SpecError] = []

    def err(self, path: str, message: str, allowed=None, nearest_key=None) -> None:
        self.errors.append(SpecError(path, message, tuple(allowed) if allowed else None,
                                     nearest_key))

    # -- primitives ----------------------------------------------------------

    def obj(self, v, path: str, keys: Iterable[str]) -> Mapping | None:
        if v is None:
            return None
        keys = list(keys)
        if not isinstance(v, Mapping):
            self.err(path, f"must be an object with keys {sorted(keys)}, got {_kind(v)}", keys)
            return None
        for k in v:
            if k not in keys:
                self.err(f"{path}.{k}" if path else str(k), "unknown field", keys,
                         nearest(k, keys))
        return v

    def string(self, v, path: str, allowed: Iterable[str] | None = None) -> str | None:
        if v is None:
            return None
        allowed = None if allowed is None else list(allowed)
        if not isinstance(v, str):
            self.err(path, f"must be a string, got {_kind(v)}", allowed)
            return None
        if v.strip().lower() in WILDCARDS:
            self.err(path, f"{v!r} is a wildcard: name one value (v1 compiles one design; "
                           "search is a separate step)", allowed)
            return None
        if allowed is not None and v not in allowed:
            self.err(path, f"unknown value {v!r}", allowed, nearest(v, allowed))
            return None
        return v

    def number(self, v, path: str, *, positive: bool = False, nonneg: bool = False,
               integer: bool = False) -> float | None:
        if v is None:
            return None
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            self.err(path, f"must be a number, got {_kind(v)}")
            return None
        if not math.isfinite(v):
            self.err(path, f"must be finite, got {v!r}")
            return None
        if integer and int(v) != v:
            self.err(path, f"must be an integer, got {v!r}")
            return None
        if positive and v <= 0:
            self.err(path, f"must be > 0, got {v:g}")
            return None
        if nonneg and v < 0:
            self.err(path, f"must be >= 0, got {v:g}")
            return None
        return float(v)

    def target(self, v, path: str) -> None:
        keys = ("min", "max", "value", "tol", "weight", "hard")
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            self.err(path, f"a target is an object {{min, max, value, tol, weight, hard}}, "
                           f"not a bare number ({v!r}: use {{\"max\": {v!r}}}, "
                           f"{{\"min\": {v!r}}} or {{\"value\": {v!r}}})", keys)
            return
        d = self.obj(v, path, keys)
        if d is None:
            return
        lo, hi = self.number(d.get("min"), f"{path}.min"), self.number(d.get("max"), f"{path}.max")
        value = self.number(d.get("value"), f"{path}.value")
        tol = self.number(d.get("tol"), f"{path}.tol", nonneg=True)
        self.number(d.get("weight", 1.0), f"{path}.weight", nonneg=True)
        if "hard" in d and not isinstance(d["hard"], bool):
            self.err(f"{path}.hard", f"must be true or false, got {_kind(d['hard'])}")
        if not any(k in d for k in ("min", "max", "value")):
            self.err(path, "a target needs min, max or value")
        if lo is not None and hi is not None and lo > hi:
            self.err(path, f"min {lo:g} is above max {hi:g}")
        if tol is not None and value is None:
            self.err(f"{path}.tol", "tol goes with value")

    def targets(self, v, section: str, kind: str | None, lk=None) -> None:
        table = TARGET_FIELDS[section]
        allowed = [n for n, f in table.items() if kind is None or kind in f.kinds]
        if not isinstance(v, Mapping):
            self.obj(v, section, allowed + ([ALLOWANCE] if section == "budget" else []))
            return
        # a mechanism's output measures only some of the output metrics (a line has no
        # dwell, a rocker no straightness): a target on one it lacks is refused here
        has = output_metrics(lk) if lk is not None and lk.output is not None else None
        for name, t in v.items():
            if section == "budget" and name == ALLOWANCE:
                self.number(t, f"{section}.{name}", nonneg=True)
                continue
            if name in allowed:
                if (has is not None and section == "motion" and name in OUTPUT_TARGETS
                        and name not in has):
                    others = [n for n in allowed if n not in OUTPUT_TARGETS]
                    its = (f"its metrics: {', '.join(has)}" if has else
                           f"an {lk.output.motion} output is measured by its extent alone "
                           f"(extent_mm on the card), which is no target")
                    self.err(f"{section}.{name}",
                             f"{lk.key}'s {lk.output.motion} output has no {name}; {its}",
                             (*has, *others))
                    continue
                self.target(t, f"{section}.{name}")
            elif name in table:      # the other kind's metric: say so, not "unknown"
                self.err(f"{section}.{name}", f"{name} is a metric of a "
                         f"{'/'.join(table[name].kinds)}, not a {kind}", allowed)
            elif (home := next((s for s, t in TARGET_FIELDS.items() if name in t), None)):
                self.err(f"{section}.{name}", f"{name} is a {home} metric: put it under "
                                              f"{home}", allowed)
            else:
                # a misspelling is nearest by letters; a metric of another name (the
                # transmission angle asked for as a "rotation") is not
                near = nearest(name, allowed)
                if near is not None and not name.split("_")[0].startswith(near.split("_")[0][:3]):
                    near = None
                self.err(f"{section}.{name}", "unknown metric"
                         + ("" if section != "budget" else f" (a plain number under budget is "
                                                           f"{ALLOWANCE} only)"), allowed, near)


def _kind(v) -> str:
    return {dict: "an object", list: "a list", str: "a string", bool: "a boolean",
            int: "a number", float: "a number", type(None): "null"}.get(type(v),
                                                                         type(v).__name__)


def validate(data: Mapping) -> list[SpecError]:
    """Everything wrong with a spec document (empty: it is a valid v1 spec)."""
    v = _Validator()
    if not isinstance(data, Mapping):
        return [SpecError("", f"a spec is an object, got {_kind(data)}")]
    top = v.obj(data, "", ("version", "kind", "linkage", "legs", "motion", "size", "materials",
                           "constructions", "fit", "budget", "outputs"))
    version = top.get("version", SPEC_VERSION)
    if str(version) != SPEC_VERSION:
        v.err("version", f"unknown spec version {version!r}; this engine reads version "
                         f"{SPEC_VERSION!r}", (SPEC_VERSION,))
    kind = None
    if "kind" not in top:
        v.err("kind", "required: walker or mechanism", KINDS)
    else:
        kind = v.string(top["kind"], "kind", KINDS)

    # linkage
    lk = None
    if "linkage" not in top:
        v.err("linkage", "required: {key, params}")
    else:
        d = v.obj(top["linkage"], "linkage", ("key", "params"))
        if d is not None:
            if "key" not in d:
                v.err("linkage.key", "required: a registered linkage", linkage.available(kind))
            else:
                key = d["key"]
                if (isinstance(key, str) and key not in linkage.available() and kind is not None
                        and key.strip().lower() not in WILDCARDS):
                    # unknown: the kind's keys are what is allowed; the nearest key of the
                    # other kind is still named, with its kind
                    near = nearest(key, linkage.available())
                    of = linkage.get(near).kind if near else None
                    v.err("linkage.key", f"unknown value {key!r}"
                          + (f"; {near!r} is a {of}, not a {kind}" if of and of != kind else ""),
                          linkage.available(kind), near)
                    key = None
                else:
                    key = v.string(key, "linkage.key", linkage.available())
                if key is not None:
                    lk = linkage.get(key)
                    if kind is not None and lk.kind != kind:
                        v.err("linkage.key", f"{key} is a {lk.kind}, not a {kind}",
                              linkage.available(kind))
            params = v.obj(d.get("params"), "linkage.params",
                           list(lk.params) if lk is not None else ())
            if params is not None and lk is not None:
                for name, val in params.items():
                    if name in lk.params:
                        # a length must be > 0; an angle or a coordinate (a fixed pivot's
                        # x or y, whose default is not positive) may be anything finite
                        v.number(val, f"linkage.params.{name}", positive=name not in lk.signed)

    # legs
    legs = v.obj(top.get("legs"), "legs", ("module", "phases_deg", "sides"))
    if legs is not None:
        modules = list(lk.leg_modules) if lk is not None else module_keys()
        mod = legs.get("module")
        if (isinstance(mod, str) and mod not in modules and lk is not None
                and mod.strip().lower() not in WILDCARDS):
            if lk.kind == "mechanism":      # one side, no legs: the walker's words don't apply
                v.err("legs.module", f"unknown value {mod!r}; {lk.key} is a mechanism: its one "
                                     f"module is {', '.join(modules)} (one side, no legs; the leg "
                                     f"modules double, decker and quad are a walker's)",
                      modules, nearest(mod, modules))
            else:
                legs_of = ", ".join(f"{m} {len(lk.leg_modules[m])} a side "
                                    f"({2 * len(lk.leg_modules[m])} on the robot)"
                                    for m in modules)
                v.err("legs.module", f"unknown value {mod!r}; a module is the legs per side, and "
                                     f"the robot has two sides: {legs_of}; no linkage has a "
                                     f"three-leg module (which modules walk is on the linkage's "
                                     f"card, api.describe)", modules, nearest(mod, modules))
            module = None
        else:
            module = v.string(mod, "legs.module", modules)
        phases = legs.get("phases_deg")
        if phases is not None:
            if not isinstance(phases, (list, tuple)):
                v.err("legs.phases_deg", f"must be a list of degrees, one per leg, got "
                                         f"{_kind(phases)}")
            else:
                for i, p in enumerate(phases):
                    v.number(p, f"legs.phases_deg[{i}]")
                if lk is not None:
                    m = module or default_module(lk)
                    n = len(lk.leg_modules[m])
                    if len(phases) != n:
                        v.err("legs.phases_deg", f"{m} has {n} legs per side, got "
                                                 f"{len(phases)} phases")
        sides = v.number(legs.get("sides"), "legs.sides", integer=True)
        if sides is not None and int(sides) not in (1, 2):
            v.err("legs.sides", f"must be 1 (one side) or 2 (the robot), got {sides:g}", (1, 2))
        elif sides == 2 and lk is not None and lk.kind == "mechanism":
            v.err("legs.sides", f"{lk.key} is a mechanism: one side only (it has no feet to "
                                "walk on)", (1,))

    # targets
    for section in SECTIONS:
        if section in top:
            v.targets(top[section], section, kind, lk)

    # materials, constructions, fit
    m = v.obj(top.get("materials"), "materials", ("sheet", "thickness_mm", "servo",
                                                  "frame_sheet", "crank_sheet"))
    if m is not None:
        v.string(m.get("sheet"), "materials.sheet", sheet_keys())
        v.string(m.get("frame_sheet"), "materials.frame_sheet", sheet_keys())
        v.string(m.get("crank_sheet"), "materials.crank_sheet", sheet_keys())
        v.number(m.get("thickness_mm"), "materials.thickness_mm", positive=True)
        v.string(m.get("servo"), "materials.servo", servos.available())
    c = v.obj(top.get("constructions"), "constructions", ("pillar", "pin", "crank", "heads"))
    if c is not None:
        v.string(c.get("heads"), "constructions.heads", ["best", "gap", "sink"])
        v.string(c.get("pillar"), "constructions.pillar", sorted(construction.AXLES))
        v.string(c.get("pin"), "constructions.pin", sorted(construction.AXLES))
        v.string(c.get("crank"), "constructions.crank", sorted(construction.CRANKS))
    f = v.obj(top.get("fit"), "fit", (*FIT_FIELDS, "kerf_mm", "sheet_size_mm"))
    if f is not None:
        for name in FIT_FIELDS:
            v.number(f.get(name), f"fit.{name}", positive=True)
        v.number(f.get("kerf_mm"), "fit.kerf_mm", nonneg=True)
        size = f.get("sheet_size_mm")
        if size is not None:
            if not isinstance(size, (list, tuple)) or len(size) != 2:
                v.err("fit.sheet_size_mm", "must be [width, height] in mm")
            else:
                for i, s in enumerate(size):
                    v.number(s, f"fit.sheet_size_mm[{i}]", positive=True)

    # outputs
    outs = top.get("outputs")
    if outs is not None:
        if not isinstance(outs, (list, tuple)):
            v.err("outputs", f"must be a list of formats, got {_kind(outs)}", OUTPUTS)
        else:
            for i, o in enumerate(outs):
                v.string(o, f"outputs[{i}]", OUTPUTS)
    return v.errors


def default_module(lk: linkage.Linkage) -> str:
    """The module inferred when a spec names none: the linkage's own (Strider's ``double``),
    else a walker's ``quad``, else ``single`` (:func:`spiderpig.config.default_module`)."""
    return config_default_module(lk.key)


# ---------------------------------------------------------------------------
# JSON Schema
# ---------------------------------------------------------------------------


def _target_schema(f: TargetField) -> dict:
    return {
        "type": "object", "additionalProperties": False,
        "description": f"{f.doc} [{f.unit}; {'hard' if f.hard else 'soft'} by default; "
                       f"measured by {f.source}, tier {f.tier}]",
        "properties": {
            "min": {"type": "number"}, "max": {"type": "number"}, "value": {"type": "number"},
            "tol": {"type": "number", "minimum": 0,
                    "description": "with value: met within tol (default 5 % of the value)"},
            "weight": {"type": "number", "minimum": 0, "default": 1.0},
            "hard": {"type": "boolean", "default": f.hard,
                     "description": "a miss fails verify (true) or lowers the score (false)"},
        },
    }


def spec_schema() -> dict:
    """The JSON Schema (draft 2020-12) of a v1 spec document, vocabularies from the live
    registries. A linkage's parameter names and its modules depend on its key (see
    :func:`spiderpig.api.describe`); the validator checks those exactly."""
    d = Params()
    fit_props = {name: {"type": "number", "exclusiveMinimum": 0, "default": getattr(d, name)}
                 for name in FIT_FIELDS}
    fit_props["kerf_mm"] = {"type": "number", "minimum": 0, "default": DEFAULT_KERF,
                            "description": "laser kerf compensation"}
    fit_props["sheet_size_mm"] = {"type": "array", "items": {"type": "number",
                                                             "exclusiveMinimum": 0},
                                  "minItems": 2, "maxItems": 2,
                                  "description": "usable sheet size (default: the stock's)"}
    sections = {
        s: {"type": "object", "additionalProperties": False,
            "properties": {n: _target_schema(f) for n, f in table.items()}}
        for s, table in TARGET_FIELDS.items()
    }
    sections["budget"]["properties"][ALLOWANCE] = {
        "type": "number", "minimum": 0,
        "description": "USD allowed, in all, for the items the catalog doesn't price (they are "
                       "otherwise left out of the total, which then can't confirm a max): the "
                       "cost row adds it and names them",
    }
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://spiderpig/schema/spec-v1.json",
        "title": "spiderpig Spec v1",
        "description": __doc__.split("\n\n")[0],
        "type": "object", "additionalProperties": False,
        "required": ["kind", "linkage"],
        "properties": {
            "version": {"const": SPEC_VERSION, "default": SPEC_VERSION},
            "kind": {"enum": list(KINDS)},
            "linkage": {
                "type": "object", "additionalProperties": False, "required": ["key"],
                "properties": {
                    "key": {"enum": linkage.available(),
                            "description": "a registered linkage (no wildcard)"},
                    "params": {"type": "object", "additionalProperties": {"type": "number"},
                               "description": "overrides of the linkage's parameters (its "
                                              "names; lengths > 0, angles in degrees, a "
                                              "coordinate such as a fixed pivot's x or y may "
                                              "be negative: the card marks them `signed`)"},
                },
            },
            "legs": {
                "type": "object", "additionalProperties": False,
                "properties": {
                    "module": {"enum": module_keys(),
                               "description": "one of the linkage's leg modules (default: "
                                              "quad for a walker, single for a mechanism)"},
                    "phases_deg": {"type": "array", "items": {"type": "number"},
                                   "description": "crank phase per leg (default: the "
                                                  "module's)"},
                    "sides": {"enum": [1, 2], "description": "2: the robot (default for a "
                                                             "walker); 1: one side"},
                },
            },
            "motion": sections["motion"],
            "size": sections["size"],
            "materials": {
                "type": "object", "additionalProperties": False,
                "properties": {
                    "sheet": {"enum": sheet_keys(), "default": "acrylic_3mm"},
                    "thickness_mm": {"type": "number", "exclusiveMinimum": 0,
                                     "description": "measured sheet thickness (default: "
                                                    "the stock's nominal)"},
                    "servo": {"enum": servos.available(), "default": servos.DEFAULT},
                    "frame_sheet": {"enum": sheet_keys(), "default": BuildConfig.frame_sheet,
                                    "description": "the frame and centre plates' sheet"},
                    "crank_sheet": {"enum": sheet_keys(), "default": BuildConfig.crank_sheet,
                                    "description": "the crank's plates' sheet"},
                },
            },
            "constructions": {
                "type": "object", "additionalProperties": False,
                "properties": {
                    # the engine's defaults (what ``resolve`` fills in): BuildConfig's
                    "pillar": {"enum": sorted(construction.AXLES), "default": BuildConfig.pillar},
                    "pin": {"enum": sorted(construction.AXLES), "default": BuildConfig.pin},
                    "crank": {"enum": sorted(construction.CRANKS),
                              "default": DEFAULT_CRANKS["walker"],
                              "description": "default: " + ", ".join(
                                  f"{v} for a {k}" for k, v in DEFAULT_CRANKS.items())},
                    "heads": {"enum": ["best", "gap", "sink"], "default": BuildConfig.heads,
                              "description": "fasteners' heads: sunk into a layer, in thin "
                                             "clearance gaps, or the lower plan of both"},
                },
            },
            "fit": {"type": "object", "additionalProperties": False, "properties": fit_props},
            "budget": sections["budget"],
            "outputs": {"type": "array", "items": {"enum": list(OUTPUTS)}, "uniqueItems": True,
                        "default": list(DEFAULT_OUTPUTS)},
        },
    }
