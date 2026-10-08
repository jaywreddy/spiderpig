"""The stage reports' base (:class:`Report`) and the first stages' reports
(:class:`CheckReport`, :class:`PlanReport`); the later operations' are
:mod:`spiderpig.api.reports`'."""


from __future__ import annotations

from collections.abc import Callable
from dataclasses import Field, dataclass, field, fields
from typing import TYPE_CHECKING, Any, ClassVar, Self

from spiderpig.failure import Failure


class Report:
    """A stage report: a dataclass whose JSON form (:func:`spiderpig.design.jsonable`) a
    store writes and :meth:`from_dict` reads back (unknown keys ignored)."""

    if TYPE_CHECKING:       # every subclass is a dataclass
        __dataclass_fields__: ClassVar[dict[str, Field[Any]]]

    readers: ClassVar[dict[str, Callable[[Any], Any]]] = {}
    """How a field's JSON form reads back, per field name, beside ``failures`` (every
    report's): a subclass's own (:class:`spiderpig.api.reports.WalkReport`'s ``rows``)."""

    @classmethod
    def from_dict(cls, d: dict) -> Self:
        kw = {}
        for f in fields(cls):
            if f.name not in d:
                continue
            v = d[f.name]
            if f.name == "failures":
                v = [Failure.from_dict(x) for x in v]
            elif f.name in cls.readers:
                v = cls.readers[f.name](v)
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
