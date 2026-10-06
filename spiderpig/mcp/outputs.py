"""The typed structured outputs of the MCP tools.

Every tool returns one of these ``TypedDict``s: the SDK publishes each as the tool's
``outputSchema`` and validates every result against it (a key it doesn't name is
dropped, so a report's field must be listed here to reach the wire). Each carries the
envelope of :class:`Result`, ``ok`` and ``failures`` (:class:`FailureOut`: the JSON form
of :class:`spiderpig.failure.Failure`), so a stage that fails is an ordinary result with
``ok = false``. Only a misuse of a tool (an unknown design, a malformed id, ``gc``
without arguments) comes back with ``isError`` set, and even then the payload is this
same envelope, never an exception's text.

Numbers are floats in mm, g, degrees or USD as the field says; a part is its manifest
entry with the ``path`` of its STEP file inside the store; a report's ``rows`` are
:class:`spiderpig.verify.Row` documents (``pass``, ``tier``, ``hard``).
"""

from __future__ import annotations

from typing import Any, NotRequired, TypedDict

JSON = dict[str, Any]


class RecommendationOut(TypedDict):
    """A checked change that clears a failure: ``patch`` is the spec merge patch that
    applies it (hand it to ``derive``)."""

    changes: list[JSON]
    why: str
    effects: str
    verified: str
    patch: JSON
    notes: list[str]


class FailureOut(TypedDict):
    """What a stage said went wrong, as data (:class:`spiderpig.failure.Failure`)."""

    stage: str
    code: str
    message: str
    culprits: list[JSON]
    numbers: JSON
    recommendations: list[RecommendationOut]
    notes: list[str]
    blockers: list[JSON]


class SpecErrorOut(TypedDict):
    """One thing wrong with a spec: where, what, the values allowed there, the nearest."""

    path: str
    message: str
    allowed: list[Any] | None
    nearest: str | None


class JobOut(TypedDict):
    """A long operation's record: ``state`` is ``queued``, ``running``, ``done`` or
    ``failed``; ``result`` (done) is the operation's own output, ``error`` (failed) a
    failure."""

    job: str
    op: str
    design: str
    args: JSON
    state: str
    started_at: str
    finished_at: str | None
    seconds: float | None
    result: NotRequired[JSON]
    error: NotRequired[FailureOut]


class Result(TypedDict):
    """The envelope of every tool result."""

    ok: bool
    failures: list[FailureOut]


class LinkagesOut(Result):
    linkages: list[JSON]


class DescribeOut(Result):
    card: JSON


class CatalogOut(Result):
    servos: NotRequired[list[JSON]]
    sheets: NotRequired[list[JSON]]
    constructions: NotRequired[JSON]


class DesignOut(Result):
    """A resolved design (``resolve``, ``derive``): its id and every inferred value; or,
    for a spec that doesn't validate, the ``errors``."""

    design: NotRequired[str]
    kind: NotRequired[str]
    linkage: NotRequired[str]
    module: NotRequired[str]
    sides: NotRequired[int]
    engine_version: NotRequired[str]
    resolved: NotRequired[JSON]
    warnings: NotRequired[list[str]]
    derived_from: NotRequired[str | None]
    patch: NotRequired[JSON | None]
    created_at: NotRequired[str]
    store: NotRequired[str]
    errors: NotRequired[list[SpecErrorOut]]


class CheckOut(Result):
    design: str
    steps: list[JSON]
    output: JSON | None
    foot_path: JSON | None
    drive: JSON
    clearances: list[JSON]
    crank_facts: JSON | None
    ground_clearance_mm: float | None
    lowest_body_part: str
    warnings: list[str]
    seconds: float


class PlanOut(Result):
    design: str
    layers: dict[str, int]
    top: int | None
    n_layers: int | None
    height_mm: float | None
    pitch_mm: float | None
    route: JSON | None
    optimal: bool | None
    proof: str
    cost: int | None
    ground_clearance_mm: float | None
    table: str
    reused: str | None
    warnings: list[str]
    heads: str | None           # fasteners' heads sunk into layers, or in clearance gaps
    gaps_mm: dict[str, float]   # layer -> the clearance gap over it (height_mm counts them)
    seconds: float


RowOut = TypedDict("RowOut", {
    "requirement": str, "source": str, "value": Any, "target": str | None, "pass": bool,
    "tier": str, "hard": bool, "detail": str, "score": float | None, "weight": float,
    "unit": str,
})


class WalkOut(Result):
    design: str
    skipped: str | None
    metrics: JSON | None
    mass_g: float | None
    mass_nominal: bool
    feet_z_planned: bool
    servo: JSON
    rows: list[RowOut]
    notes: list[str]
    seconds: float


class ExplainOut(Result):
    design: str
    text: str


class RecommendOut(Result):
    design: str
    stage: str | None
    recommendations: list[RecommendationOut]
    notes: list[str]
    seconds: NotRequired[float]


class BuildOut(Result):
    """The build manifest: every part with its mass, layers and STEP file (``path``,
    inside the store's ``build/parts/``; a right-side part names its left twin in
    ``same_as`` instead). While the job runs only ``job`` is set."""

    design: NotRequired[str]
    t: NotRequired[float | None]
    n_parts: NotRequired[int]
    counts: NotRequired[JSON]
    mass_g: NotRequired[float | None]
    envelope_mm: NotRequired[list[float] | None]
    meta: NotRequired[JSON]
    parts: NotRequired[list[JSON]]
    dir: NotRequired[str]
    files: NotRequired[int]
    warnings: NotRequired[list[str]]
    cut_rules: NotRequired[JSON]
    seconds: NotRequired[float]
    job: NotRequired[JobOut]


class VerifyOut(Result):
    design: NotRequired[str]
    level: NotRequired[str]
    score: NotRequired[float]
    rows: NotRequired[list[RowOut]]
    unverified: NotRequired[list[str]]
    seconds: NotRequired[float]
    job: NotRequired[JobOut]


class ExportOut(Result):
    """The files written and the manifest; ``warnings`` is what the bake and the
    constructions warned about while writing (a purchased model's faces the mesher
    skipped, a printed snap that overstrains). While the job runs only ``job`` is set."""

    design: NotRequired[str]
    out_dir: NotRequired[str]
    formats: NotRequired[list[str]]
    files: NotRequired[list[str]]
    manifest: NotRequired[JSON]
    warnings: NotRequired[list[str]]
    seconds: NotRequired[float]
    job: NotRequired[JobOut]


class JobResult(Result):
    """``get_job`` / ``wait_job``: what the long tool itself returns, so a finished job's
    result (a build's manifest, a verify's rows, an export's files) sits flat beside the
    ``job`` record, and a running job is the record alone."""

    job: JobOut
    design: NotRequired[str]
    seconds: NotRequired[float]
    # build
    t: NotRequired[float | None]
    n_parts: NotRequired[int]
    counts: NotRequired[JSON]
    mass_g: NotRequired[float | None]
    envelope_mm: NotRequired[list[float] | None]
    meta: NotRequired[JSON]
    parts: NotRequired[list[JSON]]
    dir: NotRequired[str]
    files: NotRequired[int | list[str]]
    warnings: NotRequired[list[str]]
    cut_rules: NotRequired[JSON]
    # verify
    level: NotRequired[str]
    score: NotRequired[float]
    rows: NotRequired[list[RowOut]]
    unverified: NotRequired[list[str]]
    # export
    out_dir: NotRequired[str]
    formats: NotRequired[list[str]]
    manifest: NotRequired[JSON]


class CompareOut(Result):
    a: str
    b: str
    spec_patch: JSON
    resolved_patch: JSON
    engine_version: JSON | None
    derived: str | None
    reports: dict[str, JSON]
    only_in: dict[str, list[str]]


class StageOut(Result):
    design: str
    stage: str
    report: JSON


class DesignsOut(Result):
    store: str
    designs: list[JSON]


class GcOut(Result):
    removed: list[str]


class ViewOut(Result):
    """``view``: the viewer's URL for the design (``?design=<id>``), the server's base
    URL, and the mode the page opens in (``robot`` or ``side``)."""

    design: str
    url: str
    server: str
    mode: str


__all__ = [
    "JSON", "BuildOut", "CatalogOut", "CheckOut", "CompareOut", "DescribeOut", "DesignOut",
    "DesignsOut", "ExplainOut", "ExportOut", "FailureOut", "GcOut", "JobOut", "JobResult",
    "LinkagesOut", "PlanOut", "RecommendOut", "RecommendationOut", "Result", "RowOut",
    "SpecErrorOut", "StageOut", "VerifyOut", "ViewOut", "WalkOut",
]
