"""The operations' reports (:class:`Report` and one per operation)."""


from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, ClassVar

from spiderpig.failure import Failure, Recommendation
from spiderpig.stages.reports import CheckReport, PlanReport, Report

__all__ = ["AdviceReport", "BuildReport", "CheckReport", "ExportReport", "PlanReport",
           "RecheckReport", "Report", "WalkReport"]


def _rows(v: list) -> list:
    from spiderpig.verify import Row

    return [Row.from_dict(x) for x in v]


# ---------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------


@dataclass
class WalkReport(Report):
    """The quasi-static walk model's metrics (:func:`walk.api_payload`) with each motion
    target's verdict (``rows``); ``skipped`` for a mechanism. Feet sit at their planned
    layers when the side is planned (``feet_z_planned``), the mass is the model's nominal
    one (``mass_nominal``) unless a build gave it."""

    readers: ClassVar[dict[str, Any]] = {"rows": _rows}

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

    readers: ClassVar[dict[str, Any]] = {
        "envelope_mm": lambda v: None if v is None else tuple(v)}

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
