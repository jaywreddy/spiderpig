"""The check and the plan (:func:`check`, :func:`plan`, :func:`plan_config`), what
would clear a failure (:func:`explain`, :func:`recommend`) and :func:`advise`."""


from __future__ import annotations

import math
import time
from dataclasses import replace
from pathlib import Path

from spiderpig import walk as walk_model
from spiderpig.api.cards import _output_dict, _step_dict, foot_path
from spiderpig.api.reports import AdviceReport, CheckReport, PlanReport
from spiderpig.api.store_ops import (
    _cached,
    _commit,
    _finish,
    _record,
    _template,
    capture_warnings,
    log,
    resolve,
    second_input_note,
    spec_of,
)
from spiderpig.config import (
    BuildConfig,
    default_robot,
)
from spiderpig.construction.base import ConstructionError
from spiderpig.construction.crank import CrankRoute, Run
from spiderpig.design import (
    Design,
    spec_hash,
)
from spiderpig.fabricate import (
    SideDesign,
    design_side,
    ground_clearance,
    remember,
    side_problem,
    static_stage,
)
from spiderpig.fabricate import template_for as _template_for
from spiderpig.failure import Failure, Recommendation
from spiderpig.spec import (
    effective_hard,
)
from spiderpig.stack import ClearanceError, PlanError, verify_plan
from spiderpig.store import PROJECT, Store


def check(design: Design, force: bool = False) -> CheckReport:
    """The stages before any layer (:class:`CheckReport`): every loop closes (else
    ``program``), a mechanism's output keeps its promises (else ``output``), the drive
    (``drive``: one servo turns ``t``), the static facts and the crank's route points
    (else ``static``, with what would clear it), the ground clearance."""
    if not force and (rep := _cached(design, "check", CheckReport)) is not None:
        return rep
    t0 = time.time()
    cfg, lk = design.config, design.lk
    params = dict(cfg.proportions)
    rep = CheckReport()
    steps = lk.check(params)
    rep.steps = [_step_dict(s) for s in steps]
    bad = next((s for s in steps if s.fails_deg is not None), None)
    if bad is not None:
        if bad.invalid:      # a point that isn't a number: the parameters, not a loop
            rep.failures.append(Failure(
                "program", "point_undefined", f"{lk.key}: {bad.describe()}",
                culprits=[{"joint": bad.point, "refs": list(bad.refs)}],
                notes=["the parameters put a length under a square root below zero (or divided "
                       "by zero): change them so the named expression is positive; the card's "
                       "defaults are one such set"]))
            return _finish(design, "check", rep, t0)
        rep.failures.append(Failure(
            "program", "loop_cannot_close", f"{lk.key}: {bad.describe()}",
            culprits=[{"joint": bad.point, "refs": list(bad.refs)}],
            numbers={"margin_mm": bad.margin_mm, "worst_deg": bad.worst_deg,
                     "fails_deg": bad.fails_deg, "fail_fraction": bad.fail_fraction,
                     "radii_mm": bad.radii}))
        return _finish(design, "check", rep, t0)
    if lk.output is not None:
        oc = lk.output_check(params)
        rep.output = _output_dict(oc)
        if oc.broken:
            rep.failures.append(Failure(
                "output", "promise_broken", f"{lk.key}: {oc.broken}",
                culprits=[{"output": oc.output.name, "point": oc.output.point,
                           "body": oc.output.link}],
                numbers={k: v for k, v in rep.output.items()
                         if isinstance(v, (int, float)) and v is not None}))
            return _finish(design, "check", rep, t0)
    else:
        rep.foot_path = foot_path(lk, params)
    rep.drive = {"servo": cfg.servo, "rpm_max": walk_model.servo_info(cfg.servo)["rpm_max"],
                 "inputs": list(lk.inputs)}
    try:
        tmpl = design.template = _template_for(cfg)
        with capture_warnings() as warned:      # the constructions size themselves here
            # hint=False: the static stage needs no leg hint (that is one more plan, the
            # single module's, which only the search uses)
            ctx, _, problem = side_problem(tmpl, replace(cfg, robot=False), hint=False)
    except ValueError as e:      # AssemblyError / OutputError (caught above) / ConstructionError
        fl = Failure.from_exception(e, lk=lk)
        if isinstance(e, ConstructionError) and getattr(e, "changes", ()):
            from spiderpig.recommend import construction_fix

            recs, notes = construction_fix(replace(cfg, robot=False), e)
            fl.recommendations += [Recommendation.from_engine(r, lk) for r in recs]
            fl.notes += notes
        rep.failures.append(fl)
        return _finish(design, "check", rep, t0)
    rep.warnings = list(warned)
    rep.clearances = [{"link": c.link, "keepout": c.keepout.owner, "where": c.keepout.where,
                       "dist_mm": c.dist, "need_mm": c.need, "text": c.describe()}
                      for c in problem.clearances]
    if problem.router is not None:
        f = problem.router.facts
        rep.crank_facts = {
            "o_free": {k: max(v, 0.0) for k, v in f.o_free.items()},
            "hosts": {k: list(v) for k, v in f.hosts.items()},
            "detours": [{"name": d.name, "r_mm": d.r, "angle_deg": d.angle, "sweep_mm": d.sweep}
                        for d in f.detours],
            "underside_lowest_mm": f.envelope.lowest if f.envelope is not None else None,
            "allow_mm": f.allow if math.isfinite(f.allow) else None,
        }
    rep.ground_clearance_mm = ground_clearance(tmpl, ctx)
    under = ctx.interfaces.get("underside")
    # which body shape sets the clearance: a walker's question (a mechanism has no feet)
    rep.lowest_body_part = (under.lowest_part
                            if under is not None and rep.ground_clearance_mm is not None else "")
    try:
        static_stage(tmpl, problem, cfg)
    except ClearanceError as e:
        fl = Failure.from_exception(e, lk=lk)
        fails = problem.router.facts.failures
        fl.culprits = [{"body": f.link, "point": f.pin, "dist_mm": f.dist, "need_mm": f.need,
                        "post_mm": f.post, "link_radius_mm": f.link_r, "margin_mm": f.margin,
                        "detour": f.detour, "allow_mm": f.allow} for f in fails]
        if fails:
            fl.numbers = {"dist_mm": fails[0].dist, "need_mm": fails[0].need}
        rep.failures.append(fl)
    return _finish(design, "check", rep, t0)


def plan(design: Design, force: bool = False) -> PlanReport:
    """The layer plan of one side (:class:`PlanReport`), from :func:`fabricate.design_side`
    (cached by the engine per template and config). A ``plan`` failure carries the
    blockers (count, the two shapes, gap, need) and checked recommendations.

    With a store, a recorded plan is re-made through :meth:`stack.StackProblem.plan` and
    checked with :func:`stack.verify_plan` before use (the design's own, else the same
    spec's on another engine version: ``reused``); one that no longer holds is solved
    again."""
    if not force:
        rep = design.reports.get("plan")
        if rep is not None and (design.side is not None or (not rep.ok and not timed_out(rep))):
            return rep                  # (a failure for want of CPU time is searched again)
        rep = _reuse_plan(design)
        if rep is not None:
            return rep
    t0 = time.time()
    rep = PlanReport()
    cr = check(design, force)
    if not cr.ok:
        rep.failures = list(cr.failures)
        return _finish(design, "plan", rep, t0)
    try:
        with capture_warnings() as warned:
            design.side = design_side(_template(design), design.config)
    except (PlanError, ConstructionError) as e:
        rep.failures.append(Failure.from_exception(e, lk=design.lk))
        return _finish(design, "plan", rep, t0)
    rep = _plan_report(design.side, rep)
    rep.warnings = list(warned)
    return _finish(design, "plan", rep, t0)


def timed_out(rep) -> bool:
    """Did this plan report fail only because the planner's CPU budget ran out
    (``no_plan_in_time``: the machine was busy, the design may well plan)?"""
    return bool(rep.failures) and all(f.code == "no_plan_in_time" for f in rep.failures)


class PlanTimeout(ValueError):
    """:func:`plan_config`'s error when the planner's CPU budget ran out (``no_plan_in_time``):
    not a verdict on the design; a server shouldn't remember it as unbuildable."""


def plan_config(config: BuildConfig, store: Store | str | Path | None = PROJECT) -> SideDesign:
    """The planned side of a build config (a CLI's options), through the store: the config
    resolved as a design (:func:`spec_of`, as ``spiderpig view --linkage ...`` does), its
    plan reused when the store holds one (:func:`plan`: re-made and verified, not searched
    for again), else solved and recorded there. The side is then what
    :func:`fabricate.design_side` answers for that config (:func:`fabricate.remember`), so
    a build that follows plans nothing again. ``ValueError`` with the failing stage's
    message (the engine's own) when the design has no plan; :class:`PlanTimeout` (one)
    when the planner's CPU budget ran out before it could say.

    The plan is one side's whatever ``config.robot`` says, so the design is the one the
    linkage's kind builds (:func:`config.default_robot`: a walker's robot, a mechanism's
    one side), the very design ``spiderpig export`` / ``view`` by the same options make:
    ``explain`` (one side) and ``audit`` / ``export`` / ``view`` (the robot) share it."""
    config = replace(config, robot=default_robot(config.linkage))
    design = resolve(spec_of(config), store)
    rep = plan(design)
    if not rep.ok or design.side is None:
        msg = "\n  ".join(f.message for f in rep.failures[:1]) or "no plan"
        raise PlanTimeout(msg) if timed_out(rep) else ValueError(msg)
    remember(_template(design), design.side)
    return design.side


def _plan_report(d: SideDesign, rep: PlanReport | None = None) -> PlanReport:
    rep = rep or PlanReport()
    p = d.plan
    route = p.choices.get("crank")
    rep.layers = dict(p.layers)
    rep.top, rep.n_layers, rep.height_mm, rep.pitch_mm = p.top, p.top + 1, p.height, p.spec.pitch
    rep.route = (None if route is None else
                 {"runs": [{"at": r.at, "lo": r.lo, "hi": r.hi} for r in route.runs],
                  "bearing": route.bearing})
    rep.optimal, rep.proof, rep.cost = p.optimal, p.proof, p.cost
    rep.heads, rep.gaps_mm = p.heads, {str(k): v for k, v in sorted(p.gaps.items())}
    rep.ground_clearance_mm = d.ground_clearance_mm
    rep.table = p.describe()
    return rep


def _reuse_plan(design: Design) -> PlanReport | None:
    """A stored plan the design can use, or ``None``: its own (a failing one only from
    the running engine; a valid one re-made and verified, then served as cached), else
    the newest plan of the same resolved spec under another engine version (verified on
    fresh sampling, then written as this design's, without the optimality proof)."""
    store = design.store
    if store is None:
        return None
    t0 = time.time()
    own = store.read_report(design.id, "plan")
    if own is not None:
        same = own.get("engine_version") == design.engine_version
        if not own.get("ok"):
            stored = PlanReport.from_dict(own)
            if same and not timed_out(stored):   # (one for want of CPU time: search again)
                return _commit(design, "plan", stored, cached=True)
            return None
        side = _remake_plan(design, own, same)
        if side is not None:
            design.side = side
            rep = _plan_report(side)
            rep.reused, rep.seconds = "store", round(time.time() - t0, 3)
            return _commit(design, "plan", rep, cached=same)
        if same:
            log.warning("%s: the stored plan no longer holds under this engine: re-solving",
                        design.id)
    for other, doc in store.find_plans(spec_hash(design.resolved), exclude=(design.id,))[:3]:
        side = _remake_plan(design, doc, same_engine=False)
        if side is not None:
            design.side = side
            rep = _plan_report(side)
            rep.reused, rep.seconds = other, round(time.time() - t0, 3)
            return _commit(design, "plan", rep)
    return None


def _remake_plan(design: Design, doc: dict, same_engine: bool) -> SideDesign | None:
    """The side of ``design`` with the layout of a stored plan, if every claim still
    clears: the groups and the problem are rebuilt from the template, the plan is re-made
    (:meth:`stack.StackProblem.plan`) and checked (:func:`stack.verify_plan`: on the
    plan's own sampling for the same engine, on fresh sampling for another). ``None``
    when the check fails or the design fails an earlier stage."""
    if not check(design).ok:
        return None
    tmpl, cfg = _template(design), replace(design.config, robot=False)
    try:
        # hint=False: re-making a layout searches nothing, so the leg hint (the single
        # module's own plan) would be one more plan for nothing
        ctx, groups, problem = side_problem(tmpl, cfg, hint=False)
        static_stage(tmpl, problem)
        route = doc.get("route")
        choices = {} if route is None else {
            "crank": CrankRoute(tuple(Run(r["at"], int(r["lo"]), int(r["hi"]))
                                      for r in route["runs"]), bool(route.get("bearing", True)))}
        p = problem.plan({k: int(v) for k, v in doc["layers"].items()}, int(doc["top"]), choices,
                         doc.get("heads") or "gap")
        bad = verify_plan(p) if same_engine else verify_plan(p, tmpl)
    except (ValueError, KeyError, TypeError) as e:
        log.info("%s: stored plan not reusable: %s", design.id, e)
        return None
    if bad:
        log.info("%s: stored plan fails verification: %s", design.id, "; ".join(bad[:3]))
        return None
    if same_engine:
        p.optimal, p.proof = bool(doc.get("optimal")), doc.get("proof", "")
        p.cost = int(doc.get("cost") or 0)
    else:
        p.optimal, p.cost = False, int(doc.get("cost") or 0)
        p.proof = (f"re-verified under engine {design.engine_version} (planned under "
                   f"{doc.get('engine_version')}); not proven the thinnest here")
    return SideDesign(cfg, ctx, groups, p, list(problem.clearances), ground_clearance(tmpl, ctx),
                      problem.router.facts if problem.router is not None else None)


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
    cr, pr = design.reports.get("check"), design.reports.get("plan")
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


def missed_targets(design: Design) -> list[tuple[str, float, object]]:
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
