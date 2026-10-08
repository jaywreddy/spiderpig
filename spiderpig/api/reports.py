"""The operations' reports (:class:`Report` and one per operation)."""


from __future__ import annotations

from dataclasses import Field, dataclass, field, fields
from typing import TYPE_CHECKING, Any, ClassVar, Self

from spiderpig.failure import Failure, Recommendation

# ---------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------


class Report:
    """A stage report: a dataclass whose JSON form (:func:`spiderpig.design.jsonable`) a
    store writes and :meth:`from_dict` reads back (unknown keys ignored)."""

    if TYPE_CHECKING:       # every subclass is a dataclass
        __dataclass_fields__: ClassVar[dict[str, Field[Any]]]

    @classmethod
    def from_dict(cls, d: dict) -> Self:
        kw = {}
        for f in fields(cls):
            if f.name not in d:
                continue
            v = d[f.name]
            if f.name == "failures":
                v = [Failure.from_dict(x) for x in v]
            elif f.name == "rows":
                from spiderpig.verify import Row

                v = [Row.from_dict(x) for x in v]
            elif f.name == "envelope_mm" and v is not None:
                v = tuple(v)
            kw[f.name] = v
        return cls(**kw)


@dataclass
class CheckReport(Report):
    """The stages before any layer: the program's loop closures (``steps``), a mechanism's
    ``output`` check, one leg's ``foot_path`` numbers (a walker), the ``drive``, the static
    ``clearances`` (links that can never share a keep-out's layers), the crank's
    ``crank_facts`` (links that need O free and the crank points that clear them, the body's
    underside) and the ``ground_clearance_mm``."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    steps: list[dict] = field(default_factory=list)
    output: dict | None = None
    foot_path: dict | None = None
    drive: dict = field(default_factory=dict)
    clearances: list[dict] = field(default_factory=list)
    crank_facts: dict | None = None
    ground_clearance_mm: float | None = None
    lowest_body_part: str = ""      # which body shape sets the ground clearance
    warnings: list[str] = field(default_factory=list)   # the constructions', at the static stage
    seconds: float = 0.0


@dataclass
class PlanReport(Report):
    """The layer plan of one side: every link's layer, the stack (``n_layers`` of
    ``pitch_mm``: ``height_mm``), the crank's ``route`` (runs along its posts, whether it
    keeps its bottom bearing), whether it is proven the thinnest (``optimal``, ``proof``),
    and the plan's table (``table``). A failure carries the blockers and recommendations.
    ``reused`` names the stored plan this one was re-made and verified from (``"store"``:
    the design's own; another design's id: the same spec on another engine version)."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    layers: dict[str, int] = field(default_factory=dict)
    top: int | None = None
    n_layers: int | None = None
    height_mm: float | None = None
    pitch_mm: float | None = None
    route: dict | None = None
    optimal: bool | None = None
    proof: str = ""
    cost: int | None = None
    ground_clearance_mm: float | None = None
    table: str = ""
    reused: str | None = None
    warnings: list[str] = field(default_factory=list)   # the constructions' (a strained snap)
    seconds: float = 0.0
    heads: str | None = None       # fasteners' heads sunk into layers, or in clearance gaps
    gaps_mm: dict[str, float] = field(default_factory=dict)   # layer -> the gap over it


@dataclass
class WalkReport(Report):
    """The quasi-static walk model's metrics (:func:`walk.api_payload`) with each motion
    target's verdict (``rows``); ``skipped`` for a mechanism. Feet sit at their planned
    layers when the side is planned (``feet_z_planned``), the mass is the model's nominal
    one (``mass_nominal``) unless a build gave it."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    skipped: str | None = None
    metrics: dict | None = None
    mass_g: float | None = None
    mass_nominal: bool = True
    feet_z_planned: bool = False
    servo: dict = field(default_factory=dict)
    rows: list = field(default_factory=list)
    notes: list[str] = field(default_factory=list)      # why a stride reads zero
    seconds: float = 0.0


@dataclass
class BuildReport(Report):
    """Every part built at crank angle ``t`` (the manifest: ``parts``), how many of each
    fabrication, the total mass, the envelope (x, y, z extents) and the mechanism's meta."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    t: float | None = None
    n_parts: int = 0
    counts: dict = field(default_factory=dict)
    mass_g: float | None = None
    envelope_mm: tuple[float, float, float] | None = None
    meta: dict = field(default_factory=dict)
    parts: list[dict] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)   # the constructions' while building
    cut_rules: dict = field(default_factory=dict)  # manufacture.summary + its issues
    seconds: float = 0.0


@dataclass
class RecheckReport(Report):
    """The contract and clash checks over the parts as they now are: which parts were
    ``edited``, which the contract ``checked`` (inside their group's claims), the
    violations, the pairwise ``clashes`` and the ``bad_solids``."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    edited: list[str] = field(default_factory=list)
    checked: list[str] = field(default_factory=list)
    contract: list[dict] = field(default_factory=list)
    clashes: list[dict] = field(default_factory=list)
    bad_solids: list[dict] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)      # an edit that left the solid as built
    seconds: float = 0.0


@dataclass
class ExportReport(Report):
    """What :func:`export` wrote: the ``files`` and the ``manifest`` (also written as
    ``manifest.json``)."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    out_dir: str = ""
    formats: list[str] = field(default_factory=list)
    files: list[str] = field(default_factory=list)
    manifest: dict = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    seconds: float = 0.0


@dataclass
class AdviceReport(Report):
    """What would move the design: ``stage`` is the failing stage the advice is for
    (``construction``, ``static``, ``plan``), ``target`` when every stage passes but a
    target is missed, or ``None`` when nothing is missed; ``recommendations`` are checked
    (each with its spec ``patch``); ``notes`` say what can't help, what wasn't checked,
    and which missed target has no lever the engine can compute."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    stage: str | None = None
    recommendations: list[Recommendation] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    seconds: float = 0.0
