"""The error budget at the output (:func:`precision`) as a stage: the spec's
``precision.*`` targets measured and judged, for any linkage with an output (a mechanism's
point, an arm's tool)."""


from __future__ import annotations

import math
import time

from spiderpig import precision as precision_model
from spiderpig.api.reports import PrecisionReport
from spiderpig.design import Design
from spiderpig.failure import Failure
from spiderpig.repeatability import PAYLOAD_G
from spiderpig.spec import CAPABILITIES, TARGET_FIELDS
from spiderpig.stages.planning import plan
from spiderpig.stages.records import _cached, _finish

PLANAR_NOTE = ("planar: the links' tilt out of the plane is not counted (it reads a "
               "fabricated side's wobble notes: api.precision, or verify standard)")


def precision(design: Design, force: bool = False) -> PrecisionReport:
    """The design's error budget at its output (:func:`spiderpig.precision.precision`), its
    ``precision.*`` metrics and each target's verdict; stored as ``precision.json``. A
    ranged linkage's budget is complete: its links' tilt and end float from the side's
    wobble notes (the build's when the handle holds one, else the side fabricated at the
    reference pose through the fabrication cache), so the out-of-plane row, the ISO 9283 RP
    and the payload sag are in it. A walker (no output) is skipped."""
    if not force and (rep := _cached(design, "precision", PrecisionReport)) is not None:
        return rep
    t0 = time.time()
    return _finish(design, "precision", measure(design, planar=False), t0)


def measure(design: Design, planar: bool) -> PrecisionReport:
    """The budget as a report, not stored (``planar``: a ranged linkage's links' tilt left
    out, no fabrication: what ``verify`` affords at ``quick``)."""
    rep = PrecisionReport()
    lk, cfg = design.lk, design.config
    if not CAPABILITIES["output"][0](lk):
        rep.skipped = f"{cfg.linkage} {CAPABILITIES['output'][2]}: no placement to budget"
        return rep
    pr = plan(design)
    if not pr.ok:
        rep.failures += pr.failures
        return rep
    rep.payload_g = design.spec.payload_g if design.spec.payload_g is not None else PAYLOAD_G
    wobble = None
    full = lk.ranged and not planar
    if full:
        meta = design.mech.meta if design.mech is not None else {}
        wobble = meta.get("wobble") or precision_model._wobble(cfg)
    try:
        p = precision_model.precision(cfg, design.side, payload_g=rep.payload_g,
                                      wobble=wobble, oop=full)
    except (ValueError, ArithmeticError) as e:     # (a model that can't be evaluated)
        rep.failures.append(Failure.from_exception(e, stage="precision"))
        return rep
    rep.complete = full or not lk.ranged
    rep.metrics = p.metrics()
    if not rep.complete:        # (a bound without the links' rise out of the plane)
        rep.metrics.pop("repeatability_mm", None)
    rep.budget = p.as_dict()
    rep.text = p.lines()
    rep.notes = list(p.notes) + ([] if rep.complete else [PLANAR_NOTE])
    rep.rows = rows_of(design, rep)
    return rep


def rows_of(design: Design, rep: PrecisionReport) -> list:
    """A row per ``precision.*`` metric the budget measured (against its target, else
    informational), and a failing row per target it couldn't (unbounded at a singular
    pose); a ranged linkage's tilt-dependent targets of a planar budget are left for a
    level that fabricates (``verify``'s ``unverified``)."""
    from spiderpig.verify import target_row

    rows = []
    singular = rep.budget.get("singular_poses") or 0
    for name, f in TARGET_FIELDS["precision"].items():
        if not f.applies(design.lk) or name not in rep.metrics:
            continue
        v = rep.metrics[name]
        detail = _detail(name, rep)
        if v is None or not math.isfinite(v):
            detail = (f"unbounded: the joint system is singular at {singular} of the poses "
                      "sampled (a parallelogram flat, or an input not declared as a drive)")
            v = None
        row = target_row(design, f, v, "precision", detail=detail)
        if row is not None:
            rows.append(row)
    return rows


def _detail(name: str, rep: PrecisionReport) -> str:
    b = rep.budget
    if name == "repeatability_mm":
        joints = sorted((b.get("at_worst_mm") or {}).items(), key=lambda kv: -(kv[1] or 0))
        top = ", ".join(f"{j} {v:.3f}" for j, v in joints[:3] if v)
        return (f"worst case (every play the same way); {b.get('repeatability_rss_mm')} mm "
                f"root sum square" + (f"; the worst joints: {top} mm" if top else "")
                + ("" if rep.complete else "; " + PLANAR_NOTE))
    if name == "accuracy_mm":
        return (f"holes off by {b.get('hole_tol_mm')} mm: {b.get('accuracy_links_mm')} mm, "
                f"{b.get('dt_k')} K over {b.get('reach_mm')} mm: {b.get('thermal_mm')} mm, "
                f"the turntable's {b.get('turret_accuracy_mm')} mm, the drives' systematic "
                "errors and a step each; it calibrates out")
    if name in ("rp_mm", "rp_powerup_mm"):
        return (f"ISO 9283, mean + 3 sigma of 30 cycles, worst of 5 poses, a "
                f"{rep.payload_g:g} g payload; from any direction "
                f"{b.get('iso_rp_any_direction_mm')} mm")
    if name == "payload_sag_mm":
        return f"{rep.payload_g:g} g at the tool, in the plane; it calibrates out at one payload"
    return ""
