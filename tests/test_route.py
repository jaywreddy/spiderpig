"""The crank's route through the stack (:mod:`construction.route`), the body's underside
(:mod:`construction.underside`) and the planner's optimality (:mod:`stack`), checked against
a brute-force reference (:mod:`tests.brute`)."""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.config import BuildConfig
from spiderpig.construction.crank import CrankRoute
from spiderpig.construction.route import crank_facts
from spiderpig.construction.underside import Underside, underside
from spiderpig.fabricate import (
    design_side,
    ground_clearance,
    side_problem,
    static_stage,
    template_for,
)
from spiderpig.stack import ClearanceError, Layout, RouteConflict, RouteView, verify_plan
from tests import brute, cache
from tests.tiers import quick


def _cfg(key: str, module: str = "single", **kw) -> BuildConfig:
    """One side of ``key`` on the default constructions (the bolt crank, the linkage's own:
    ``bolt_round`` for TrotBot's heel) unless ``kw`` names others."""
    return BuildConfig(linkage=key, module=module, robot=False, **kw)


def _design(key: str, module: str = "single", **kw):
    """The template and the side design (planned, or its plan from the test cache)."""
    return cache.cached_design(_cfg(key, module, **kw))


# -- the envelope --------------------------------------------------------------------


def test_the_envelope_is_the_largest_circle_about_o_above_the_profile():
    xs = np.arange(-50.0, 50.25, 0.25)
    crank = np.where(np.abs(xs) <= 34, -np.sqrt(np.maximum(34**2 - xs**2, 0)), np.inf)
    alone = Underside(xs, np.minimum(crank, np.where(np.abs(xs) <= 50, 0.0, np.inf)), (0.0, 0.0))
    assert alone.allows(1.0) == pytest.approx(33.0, abs=0.01)   # 1 mm above the crank's own
    assert alone.allows(0.0) == pytest.approx(34.0, abs=0.01)
    # a deeper body under O lets more through, until its x-extent stops it ...
    deep = Underside(xs, np.full_like(xs, -60.0), (0.0, 0.0))
    assert deep.allows(1.0) == pytest.approx(50.0, abs=0.01)    # the body ends 50 mm out
    # ... or where it rises level with O: a circle wider than that would hang below it
    step = Underside(xs, np.where(np.abs(xs) <= 40, -60.0, 0.0), (0.0, 0.0))
    assert step.allows(1.0) == pytest.approx(40.25, abs=0.01)


def test_a_detour_outside_the_envelope_is_refused_and_one_inside_is_used():
    """trotbot_heel at its drawing's 7 mm unit (its own crank, ``bolt_round``: a 6 mm round
    standoff, 3 mm post radius, 7.4 mm webs): the heel link b7 clears no crankpin and the
    nearest point off them that clears it sweeps beyond the body's underside."""
    cfg = _cfg("trotbot_heel", proportions=(("unit", 7.0),))
    assert cfg.crank == "bolt_round"
    tmpl = template_for(cfg)
    ctx, groups, problem = side_problem(tmpl, cfg)
    facts = problem.router.facts
    assert [f.link for f in facts.failures] == ["b7"]
    (f,) = facts.failures
    assert f.pin == "J1"
    assert f.need == pytest.approx(10.0)            # 3 post + 6 link half-width + 1 margin
    r, first, last = f.detour
    assert r == 40.0
    assert first <= 180.0 <= last
    assert f.reach == pytest.approx(7.4)            # the round crank's web
    assert r + f.reach > f.allow == pytest.approx(34.4, abs=0.05)
    # without an envelope the same point is a detour the router can use
    crank = next(g for g in groups if g.name == "crank")
    free = crank_facts(ctx, crank.dims(ctx), None, problem.spec.margin)
    assert not free.failures
    assert [(d.r, d.sweep) for d in free.detours] == [(40.0, pytest.approx(47.4))]
    assert free.detours[0].name in free.hosts["b7"]


def test_ground_clearance_is_the_body_above_the_feet():
    tmpl, design = _design("klann")
    ctx = design.ctx
    pts = ctx.topo.geometry.points
    feet = min(float(pts[ctx.topo.point_of[f]][:, 1].min()) for f in linkage.feet_of(tmpl))
    body = underside(ctx, None).lowest     # without the crank circle: the servo is lower still
    assert design.ground_clearance_mm == pytest.approx(ctx.interfaces["underside"].lowest - feet)
    assert design.ground_clearance_mm == pytest.approx(body - feet)
    assert 60 < design.ground_clearance_mm < 75
    assert ground_clearance(tmpl, ctx) == design.ground_clearance_mm


# -- the static stage ------------------------------------------------------------------


def test_the_heel_stops_the_static_stage_with_the_numbers():
    cfg = _cfg("trotbot_heel", proportions=(("unit", 7.0),))
    with pytest.raises(ClearanceError) as e:
        design_side(template_for(cfg), cfg)
    msg = str(e.value)
    assert msg.startswith("trotbot_heel: b7 sweeps right across the crank at O, so its layer "
                          "needs the crank off its axis, and no crank point clears it: it passes "
                          "crankpin J1 at 6.8 mm, under the 10.0 mm a post there needs (3 post "
                          "radius + 6 link half-width + 1 margin); the nearest point off the "
                          "crankpins that clears it is 40 mm from O")
    assert ("where a run would sweep 47 mm about O, below the body's underside, which lets the "
            "crank sweep 34.4 mm (a run at most 27.0 mm out)") in msg


def test_the_hex_cranks_sleeve_stops_the_heel_at_its_default_scale():
    """The hex crank's 8.5 mm printed sleeve (``bolt``) needs 11.2 mm from b7, which passes
    J1 at 10.2 at the heel's default scale: the static stage says so, and the heel's own
    crank (``bolt_round``, config.LINKAGE_CRANKS: the 6 mm round standoff) plans."""
    cfg = _cfg("trotbot_heel", crank="bolt")
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg, hint=False)
    with pytest.raises(ClearanceError) as e:
        static_stage(tmpl, problem)
    msg = str(e.value)
    assert ("it passes crankpin J1 at 10.2 mm, under the 11.2 mm a post there needs (4.25 post "
            "radius + 6 link half-width + 1 margin)") in msg
    assert _design("trotbot_heel")[1].plan.top == 12       # the round standoff's plan


# -- routes ------------------------------------------------------------------------------


@pytest.mark.parametrize(("key", "pin", "through"), [("trotbot", "J1", "b6"),
                                                      ("sixbar", "M", "b4"),
                                                      ("sixbar_v1", "M", "b4")])
def test_the_crank_runs_along_its_pin_through_the_link_that_sweeps_o(key, pin, through):
    tmpl, design = _design(key)
    plan = design.plan
    route = plan.choices["crank"]
    assert isinstance(route, CrankRoute)
    assert route.bearing
    k = plan.layers[through]
    assert any(r.at == pin and r.lo <= k <= r.hi for r in route.runs)
    assert not any(p.layer == k and p.shape.core == ("pt", "O") for p in plan.shapes("crank"))
    assert verify_plan(plan, tmpl) == []


@pytest.mark.parametrize(("module", "layers"), [("single", 10), ("double", 10),
                                                ("decker", 12), ("quad", 13)])
def test_klann_plans_keep_their_layers_and_are_proven_thinnest(module, layers):
    """The demo Klann's modules on the default constructions (the bolt crank's single
    plates, its heads in clearance gaps; standoff pillars, Chicago pins), each proven the
    thinnest: no plan in fewer layers, the crank's own rules having forced it there."""
    _, design = _design("klann", module)
    plan = design.plan
    assert plan.top + 1 == layers
    assert plan.height == pytest.approx(sum(plan.t(k) for k in range(plan.top + 1))
                                        + sum(plan.gaps.values()))
    assert plan.optimal, plan.proof
    assert plan.proof.startswith(f"no plan in {plan.top} layers or fewer")


def test_the_routers_rules_are_the_bolt_cranks():
    """The bolt crank's rules (``BoltCrank.joint_rules``): no plate between two runs of one
    point (``inner_webs``), no journal plate: two chains share a plate or a journal
    standoff joins them, and the last ends in the hub plate (``j_last``); the screw heads
    beyond its plates claim their clearance gaps (``gap_head``); a chain's span is one a
    stock standoff fits, the window the longest of those."""
    cfg = _cfg("klann")
    ctx, groups, problem = side_problem(template_for(cfg), cfg)
    router = problem.router
    rules = router.rules
    crank = next(g for g in groups if g.name == "crank").construction
    assert not rules.inner_webs
    assert not router.inner_webs
    assert rules.j_last
    assert router.j_last
    assert rules.gap_head > 0
    assert rules.gap_head == pytest.approx(crank.head_r())
    assert router.gap_pieces
    assert router.washer_bit == len(router.gap_pieces)
    pitch, t = ctx.pitch, ctx.sheet_t("crank")
    feasible = sorted(n for n, m in rules.spans.items() if m)
    assert feasible == [n for n in range(3, 64) if crank._web_span_ok(n - 2, 0, pitch, t)]
    assert router.window == max(feasible) - 3
    assert rules.bottom_layers == crank.stub_layers_web(ctx.sheet_t("frame"), pitch, t)


# -- optimality against the brute force ---------------------------------------------------


@pytest.mark.parametrize("key", quick(["klann", "trotbot", "dwell_rocker",
                                       "hoecken_pantograph"], ["klann"]))
def test_the_planner_matches_the_brute_force_optimum(key):
    """Every layering and every route up to the planner's stack size: none thinner, and none
    in it with fewer added crank features. (The brute force knows the ``heads="gap"``
    search's claims: the planner's with its heads in their clearance gaps.)"""
    cfg = _cfg(key, heads="gap")
    tmpl = template_for(cfg)
    plan = design_side(tmpl, cfg).plan
    ctx, _, problem = side_problem(tmpl, cfg)
    for top in range(plan.top - 2, plan.top):
        assert brute.solve(problem, top, ctx) is None, f"a plan in {top + 1} layers"
    best = brute.solve(problem, plan.top, ctx)
    ours = brute.cost(plan.choices["crank"], plan.layers, problem.topo.riders, {})
    assert best is not None
    assert ours == best[0]


@pytest.mark.parametrize(("key", "module", "top", "crank"), [
    ("klann", "single", 9, None), ("klann", "double", 11, None),
    ("klann", "decker", 13, None), ("klann", "quad", 13, None),
    ("trotbot", "single", 10, None), ("strider", "single", 10, None),
    ("jansen", "single", 9, None), ("hoecken_pantograph", "single", 8, None),
    ("klann", "double", 11, "bolt_round"), ("trotbot_heel", "single", 12, None)])
def test_the_routers_joint_rules_are_the_brute_forces_on_every_rider_layering(
        key, module, top, crank):
    """The router's joint rules (``JointRules``, from the bolt crank's standoff fits) against
    the brute force's own model of the crank (:func:`tests.brute.buildable`, as ``realize``
    builds it), on hand-made layerings: every layer of the riders under the hub, nothing
    else in the way. The router's cheapest route builds by the brute force's model and
    costs what the brute force's cheapest does, or neither has one."""
    cfg = _cfg(key, module, heads="gap", **({"crank": crank} if crank else {}))
    ctx, _, problem = side_problem(template_for(cfg), cfg, hint=False)
    router, riders = problem.router, problem.topo.riders
    h0 = router.hub_bottom(top)
    names = sorted(riders)
    points = list(dict.fromkeys(riders.values()))
    seen = routed = 0
    for ks in itertools.product(range(2, h0), repeat=len(names)):
        layers = dict(zip(names, ks, strict=True))
        if len({(k, riders[n]) for n, k in layers.items()}) > len(set(ks)):
            continue                    # riders of two crankpins in one layer: never a route
        res = router.route(RouteView(Layout(layers, top, problem.spec.pitch), {}))
        need = {k: riders[n] for n, k in layers.items()}
        costs = [brute.cost(r, layers, riders, {}) for r in brute.routes(points, 2, h0 - 1, need)
                 if brute.buildable(r, layers, problem, ctx, h0)]
        if isinstance(res, RouteConflict):
            assert not costs, (layers, res)
        else:
            assert brute.buildable(res.choice, layers, problem, ctx, h0), (layers, res)
            assert brute.cost(res.choice, layers, riders, {}) == min(costs), layers
            routed += 1
        seen += 1
    assert routed       # both outcomes met
    assert routed < seen
