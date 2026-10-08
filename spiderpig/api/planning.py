"""What would clear a failure (:func:`explain`, :func:`recommend`) and :func:`advise`, over
the check and the plan (:mod:`spiderpig.stages.planning`)."""


from __future__ import annotations

import time

from spiderpig.api.reports import AdviceReport, CheckReport, PlanReport
from spiderpig.config import (
    BuildConfig,
)
from spiderpig.design import (
    Design,
)
from spiderpig.failure import Recommendation
from spiderpig.spec import (
    Target,
    effective_hard,
)
from spiderpig.stages.planning import _output_dict, check, foot_path, plan
from spiderpig.stages.records import _record, _report
from spiderpig.stages.resolve import second_input_note


def explain(design: Design) -> str:
    """Each stage's verdict on one side of the design, in prose (:func:`explain.explain_config`
    with the design's full config: servo, sheet, constructions and fit). The plan is the
    design's own (:func:`plan`: the handle's, the store's re-made, or solved once); a
    recorded failure is printed, not searched for again."""
    from spiderpig import explain as explain_module

    pr = plan(design)
    failure = None if pr.ok else "\n  ".join(f.message for f in pr.failures[:1]) or "failed"
    text = explain_module.explain_config(design.config, side=design.side if pr.ok else None,
                                         plan_failure=failure)
    if not pr.ok or not design.spec.targets():
        return text
    # 4. the spec's targets that check and plan can read, and what would meet a miss
    got = cheap_measures(design)
    lines = ["", "4. targets"]
    for f, t in design.spec.targets():
        v = got.get(f.path)
        if v is None:
            lines.append(f"  {f.path}: {t.describe()}: measured by verify (the walk, the build "
                         "or the BOM)")
            continue
        met, _ = t.check(float(v))
        lines.append(f"  {f.path}: {v:.4g} {f.unit} vs {t.describe()}: "
                     f"{'ok' if met else ('MISSED (hard)' if effective_hard(t, f) else 'missed')}")
    adv = advise(design)
    if adv.recommendations:
        lines += ["  what would meet it:", *(f"  {r.describe()}" for r in adv.recommendations)]
    lines += [f"  {n}" for n in adv.notes]
    return text + "\n".join(lines)


def recommend(design: Design) -> list[Recommendation]:
    """The checked recommendations of :func:`advise`: the failing stage's (a construction
    that can't be built, the static stage's, else the planner's), or, when every stage
    passes, a scale of the linkage that meets a missed target that scales with it; each
    with the spec patch that applies it. Empty when there is nothing to recommend
    (:func:`advise` says why in its ``notes``)."""
    return list(advise(design).recommendations)


SCALED_METRICS = ("motion.stroke_mm", "motion.straightness_mm", "motion.lift_mm")
CHEAP_OUTPUT = ("stroke_mm", "straightness_mm", "on_line_fraction", "rotation_deg",
                "swing_deg", "dwell_deg")


def cheap_measures(design: Design) -> dict[str, float]:
    """The metrics a target can be read against from ``check`` and ``plan`` alone: a
    mechanism's output numbers, a walker's lift and ground clearance, the stack."""
    out: dict[str, float] = {}
    cr, pr = _report(design, "check", CheckReport), _report(design, "plan", PlanReport)
    if cr is not None and cr.ok:
        from spiderpig.verify import least_transmission_angle

        angle, _ = least_transmission_angle([s for s in cr.steps if s["kind"] == "closure"])
        if angle is not None:
            out["motion.transmission_angle_deg"] = angle
        if cr.output:
            out.update({f"motion.{k}": cr.output[k] for k in CHEAP_OUTPUT
                        if cr.output.get(k) is not None})
        if cr.foot_path:
            out["motion.lift_mm"] = cr.foot_path["lift_mm"]
        if cr.ground_clearance_mm is not None:
            out["motion.ground_clearance_mm"] = cr.ground_clearance_mm
    if pr is not None and pr.ok and pr.height_mm is not None:
        out["size.stack_mm"] = pr.height_mm
    return out


def missed_targets(design: Design) -> list[tuple[str, float, Target]]:
    """``(path, value, target)`` for every spec target :func:`cheap_measures` can read that
    the design misses."""
    got = cheap_measures(design)
    out = []
    for f, t in design.spec.targets():
        v = got.get(f.path)
        if v is None:
            continue
        met, _ = t.check(float(v))
        if not met:
            out.append((f.path, float(v), t))
    return out


def measure_config(config: BuildConfig) -> dict[str, float]:
    """The scaled metrics of ``config`` without a design: a mechanism's stroke and
    straightness, a walker's lift (what :func:`recommend.target_scale` re-measures)."""
    lk = config.lk
    params = dict(config.proportions)
    if lk.output is not None:
        c = _output_dict(lk.output_check(params or None))
        return {f"motion.{k}": c[k] for k in ("stroke_mm", "straightness_mm")
                if c.get(k) is not None}
    return {"motion.lift_mm": foot_path(lk, params)["lift_mm"]}


def advise(design: Design) -> AdviceReport:
    """What would move the design, checked (:class:`AdviceReport`). A stage that fails
    (``check``, then ``plan``) answers with its own recommendations and notes. When every
    stage passes, the targets :func:`cheap_measures` can read are compared with the spec:
    a missed stroke, straightness or lift is met by scaling the linkage
    (:func:`recommend.target_scale`: the least practical scale, measured again and
    planned); a missed stack that is proven the thinnest, or a clearance, gets a note
    saying which levers are left."""
    from spiderpig import verify as _verify
    from spiderpig.recommend import target_scale

    t0 = time.time()
    rep = AdviceReport()

    def done(rep: AdviceReport) -> AdviceReport:     # logged, never a stage of the handle
        rep.seconds = round(time.time() - t0, 3)
        _record(design, "advise", rep.seconds, rep.ok)
        return rep

    cr = check(design)
    failing = cr if not cr.ok else None
    pr = None
    if failing is None:
        pr = plan(design)
        failing = pr if not pr.ok else None
    if failing is not None:
        f = failing.failures[0] if failing.failures else None
        rep.stage = f.stage if f is not None else None
        rep.recommendations = list(f.recommendations) if f is not None else []
        rep.notes = list(f.notes) if f is not None else []
        if f is not None and f.code == "second_input_no_drive":
            rep.notes.append("no fix: " + second_input_note(design.lk))
        return done(rep)
    misses = missed_targets(design)
    if not misses:
        rep.notes.append("every stage passes and no target check or plan can read is missed; "
                         "verify measures the rest (the walk, the build, the BOM)")
        return done(rep)
    rep.stage = "target"
    scaled = [m for m in misses if m[0] in SCALED_METRICS]
    if scaled:
        # the scaled targets met now bound the scale too: a fix mustn't break one
        got = cheap_measures(design)
        missed = {m[0] for m in scaled}
        keep = [(f.path, float(got[f.path]), t) for f, t in design.spec.targets()
                if f.path in SCALED_METRICS and f.path not in missed
                and got.get(f.path) is not None]
        rec, note = target_scale(design.config, scaled, measure_config, keep=keep)
        if rec is not None:
            rep.recommendations.append(Recommendation.from_engine(rec, design.lk))
        if note:
            rep.notes.append(note)
    for path, v, t in misses:
        if path in SCALED_METRICS:
            continue
        if path == "size.stack_mm" and pr is not None and pr.optimal:
            rep.notes.append(f"size.stack_mm {v:g} vs {t.describe()}: "
                             + _verify.stack_floor_note(design, pr))
        elif path == "motion.ground_clearance_mm":
            rep.notes.append(f"motion.ground_clearance_mm {v:.1f} vs {t.describe()}: the "
                             f"lowest point is {cr.lowest_body_part or 'the body'}; it grows "
                             f"with the linkage's scale and shrinks with the servo's body and "
                             f"the chassis, none exactly, so no scale is computed: derive on "
                             f"the scale parameter or the servo and read check")
        else:
            rep.notes.append(f"{path} {v:g} vs {t.describe()}: missed; no lever the engine "
                             f"can compute for it")
    return done(rep)
