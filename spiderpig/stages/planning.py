"""The check and the plan of a design (:func:`check`, :func:`plan`), a stored plan re-made
and verified before use, and :func:`plan_config`: a build config's planned side through
the store (what ``spiderpig bake`` / ``build`` / ``explain`` / ``audit`` plan by)."""


from __future__ import annotations

import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from spiderpig import linkage
from spiderpig import walk as walk_model
from spiderpig.config import BuildConfig, default_robot
from spiderpig.construction.base import ConstructionError
from spiderpig.construction.crank import CrankRoute, Run
from spiderpig.design import Design, spec_hash
from spiderpig.fabricate import (
    SideDesign,
    design_side,
    ground_clearance,
    remember,
    router_facts,
    side_problem,
    static_stage,
)
from spiderpig.fabricate import template_for as _template_for
from spiderpig.failure import Failure, Recommendation
from spiderpig.stack import ClearanceError, PlanError, verify_plan
from spiderpig.stages.records import (
    _cached,
    _commit,
    _finish,
    _report,
    _template,
    capture_warnings,
    log,
)
from spiderpig.stages.reports import CheckReport, PlanReport
from spiderpig.stages.resolve import resolve, spec_of
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
    if (f := router_facts(problem)) is not None:
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
        facts = router_facts(problem)
        assert facts is not None    # static_stage raises only on the crank router's facts
        fails = facts.failures
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
        rep = _report(design, "plan", PlanReport)
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
    assert route is None or isinstance(route, CrankRoute)   # the crank router's choice
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
                      router_facts(problem))


def foot_path(lk: linkage.Linkage, params: dict, n: int = 720) -> dict:
    """One foot's path over a revolution (leg 0's first foot, its defaults unless
    ``params``): ``lift_mm`` (vertical travel), the stance stride and fraction within 2 mm
    of the lowest point, the crank radius, the leg's height and width."""
    ts = 2.0 * math.pi * np.arange(n) / n
    pts = lk.solve(params=params or None).evaluate(ts)
    f = pts[lk.feet[0][1]]
    y0 = float(f[:, 1].min())
    stance = f[:, 1] <= y0 + 2.0
    top = max(float(pts[j][:, 1].max()) for j in lk.points)
    xs = np.concatenate([pts[j][:, 0] for j in lk.points])
    return {
        "lift_mm": float(np.ptp(f[:, 1])),
        "stance_stride_mm": float(np.ptp(f[stance, 0])) if stance.any() else 0.0,
        "stance_fraction": float(stance.mean()),
        "crank_radius_mm": float(np.linalg.norm(pts[lk.crank[1]][0])),
        "height_mm": top - y0,
        "width_mm": float(np.ptp(xs)),
    }


def _step_dict(s) -> dict:
    return {"point": s.point, "kind": s.kind, "refs": list(s.refs), "radii_mm": s.radii,
            "margin_mm": s.margin_mm, "worst_deg": s.worst_deg, "fails_deg": s.fails_deg,
            "transmission_deg": s.angle_deg, "fail_fraction": s.fail_fraction,
            "toggles": s.toggles, "invalid": s.invalid, "text": s.describe()}


def _output_dict(c) -> dict:
    return {"name": c.output.name, "motion": c.output.motion, "point": c.output.point,
            "extent_mm": list(c.extent_mm), "stroke_mm": c.stroke_mm,
            "straightness_mm": c.straightness_mm, "on_line_fraction": c.on_line,
            "rotation_deg": c.rotation_deg, "swing_deg": c.swing_deg, "dwell_deg": c.dwell_deg,
            "broken": c.broken, "text": c.describe()}
