"""The crank's route through the stack (:mod:`construction.route`), the body's underside
(:mod:`construction.underside`) and the planner's optimality (:mod:`stack`), checked against
a brute-force reference (:mod:`tests.brute`)."""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.config import BuildConfig
from spiderpig.construction.base import ConstructionError
from spiderpig.construction.crank import CrankRoute, Run
from spiderpig.construction.route import crank_facts
from spiderpig.construction.underside import Underside, underside
from spiderpig.fabricate import (
    SideDesign,
    design_side,
    fabricate_side,
    ground_clearance,
    side_problem,
    static_stage,
    template_for,
)
from spiderpig.stack import ClearanceError, Layout, RouteConflict, RouteView, verify_plan
from tests import brute
from tests.tiers import quick


def _cfg(key: str, module: str = "single", **kw) -> BuildConfig:
    """The printed crank's routes are this file's subject: printed pillars and the keyed
    crank unless ``kw`` names others (the defaults are the bolt crank and standoffs)."""
    kw = {"pillar": "printed", "crank": "keyed", **kw}
    return BuildConfig(linkage=key, module=module, robot=False, **kw)


# TrotBot's heel cases are the printed crank's numbers (a 3 mm post radius: b7 needs 10 mm
# from J1); the keyed crank's 8.5 mm post asks 11.2 of it, which the heel's default scale
# hasn't (test_the_keyed_cranks_post_stops_the_heel_at_its_default_scale).
HEEL = {"crank": "printed"}


def _design(key: str, module: str = "single", **kw):
    cfg = _cfg(key, module, **kw)
    tmpl = template_for(cfg)
    return tmpl, design_side(tmpl, cfg)


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
    """trotbot_heel at its drawing's 7 mm unit: the heel link b7 clears no crankpin and the
    nearest point off them that clears it sweeps beyond the body's underside."""
    cfg = _cfg("trotbot_heel", proportions=(("unit", 7.0),), **HEEL)
    tmpl = template_for(cfg)
    ctx, groups, problem = side_problem(tmpl, cfg)
    facts = problem.router.facts
    assert [f.link for f in facts.failures] == ["b7"]
    (f,) = facts.failures
    assert f.pin == "J1"
    assert f.need == pytest.approx(10.0)
    r, first, last = f.detour
    assert r == 40.0
    assert first <= 180.0 <= last
    assert r + f.reach > f.allow == pytest.approx(33.0, abs=0.05)
    # without an envelope the same point is a detour the router can use
    crank = next(g for g in groups if g.name == "crank")
    free = crank_facts(ctx, crank.dims(ctx), None, problem.spec.margin)
    assert not free.failures
    assert [(d.r, d.sweep) for d in free.detours] == [(40.0, 46.0)]
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
    with pytest.raises(ClearanceError) as e:
        _design("trotbot_heel", proportions=(("unit", 7.0),), **HEEL)
    msg = str(e.value)
    assert msg.startswith("trotbot_heel: b7 sweeps right across the crank at O, so its layer "
                          "needs the crank off its axis, and no crank point clears it: it passes "
                          "crankpin J1 at 6.8 mm, under the 10.0 mm a post there needs (3 post "
                          "radius + 6 link half-width + 1 margin); the nearest point off the "
                          "crankpins that clears it is 40 mm from O")
    assert ("where a run would sweep 46 mm about O, below the body's underside, which lets the "
            "crank sweep 33.0 mm (a run at most 27.0 mm out)") in msg


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


def test_a_chain_ends_set_back_in_the_hub_only_if_the_horn_screws_still_fit():
    """TrotBot's heel in 12 layers: with b4 right under the hub, the web over it (the hub's
    lowest layer) is set back for b4's end play, and the sts3215's 5.8 mm hub then takes no
    M3x6 horn screw (the xl330's 6 mm hub still does). The route rules refuse that end, the
    brute force agrees, the construction would have caught it, and the planner's own plan
    (b4 elsewhere) builds."""
    # (the scenario on 3 mm acrylic frame plates and full-layer heads, as it was found)
    old = {"frame_sheet": "acrylic_3mm", "heads": "sink"}
    cfg = _cfg("trotbot_heel", **old, **HEEL)
    tmpl = template_for(cfg)
    ctx, groups, problem = side_problem(tmpl, cfg)
    assert not problem.router.hub_play
    assert side_problem(tmpl, _cfg("trotbot_heel", servo="xl330_m288", **old,
                                   **HEEL))[2].router.hub_play
    h0 = problem.router.hub_bottom(11)
    assert h0 == 9
    under = {"b1": 3, "b2": 2, "b3": 3, "b4": 8, "b5": 2, "b6": 4, "b7": 6, "b8": 3}
    route = CrankRoute((Run("J1", 2, 8),))
    assert not brute.buildable(route, under, problem, ctx, h0)
    design = SideDesign(cfg, ctx, groups, problem.plan(under, 11, {"crank": route}))
    with pytest.raises(ConstructionError, match="no screw fits between the crank hub and the "
                                                "sts3215 horn"):
        fabricate_side(design, tmpl.freeze_at(1.0))
    plan = design_side(tmpl, cfg).plan
    assert plan.top == 11
    assert plan.layers["b4"] != 8
    assert brute.buildable(plan.choices["crank"], plan.layers, problem, ctx, h0)
    assert "leaves the hub too short for the horn screws" in plan.proof
    assert verify_plan(plan, tmpl) == []


@pytest.mark.parametrize(("module", "crank", "height"), [
    ("single", "keyed", 24), ("double", "keyed", 27), ("decker", "keyed", 36),
    ("quad", "keyed", 51), ("quad", "printed", 39)])
def test_klann_plans_keep_their_heights_and_are_proven_thinnest(module, crank, height):
    """The keyed crank's two-layer top webs cost the quad four layers (every chain's top web
    sits under the next leg's riders); the single, double and decker had the room. (Heights
    in 3 mm layers: since 2026-10-04 the frame plates and the foot links are aluminium,
    which the plan's z adds; with the 0.080 in frame plates the STS3215 horn's face is
    1.17 mm under the inner plate, so these cranks' hubs sit a layer lower: one more each.)"""
    tmpl, design = _design("klann", module, crank=crank)
    plan = design.plan
    assert plan.top + 1 == height // 3
    assert plan.height == pytest.approx(sum(plan.t(k) for k in range(plan.top + 1))
                                        + sum(plan.gaps.values()))
    assert design.plan.optimal, design.plan.proof
    assert design.plan.proof.startswith(f"no plan in {design.plan.top} layers or fewer")


def test_the_keyed_rules_end_a_chain_one_layer_higher():
    """The keyed crank's rules: the same stock-screw spans as the printed crank's at a 3 mm
    pitch (3-6, 9 or 11 layers between a chain's outer webs, the top web's two layers
    counted), one layer less of window for a point's riders, and the pocket radius of the
    key socket's lead-in."""
    from spiderpig.construction.crank import KeyedCrank, standoff_dims

    keyed, printed = (side_problem(template_for(c), c)[2].router
                      for c in (_cfg("klann"), _cfg("klann", crank="printed")))
    assert keyed.two_layer_top
    assert not printed.two_layer_top
    feasible = {n for n, m in keyed.spans.items() if m}
    assert feasible == {n for n, m in printed.spans.items() if m} == {3, 4, 5, 6, 9, 11}
    assert (keyed.window, printed.window) == (7, 8)
    crank = KeyedCrank()
    af, _ = standoff_dims(crank.standoff_key)
    assert crank.pocket_af() == pytest.approx(af)          # pressed: cut to the key's AF
    assert keyed.rules.nut == pytest.approx(
        max(5.5 + 0.3, af + 2 * crank.pocket_chamfer) / 3 ** 0.5)
    assert printed.rules.nut == pytest.approx((5.5 + 0.3) / 3 ** 0.5)


def test_the_keyed_cranks_post_stops_the_heel_at_its_default_scale():
    """The keyed crank's 8.5 mm post (room for its hex key) needs 11.2 mm from b7, which
    passes J1 at 10.2 at the heel's default scale: the static stage says so. (Its checked
    recommendation, the next scale up, planning in 15 layers, is
    tests/test_recommend.py::test_the_keyed_crank_post_sends_the_heel_up_a_scale: the
    same design, the same recommendation.)"""
    cfg = _cfg("trotbot_heel")
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg, hint=False)
    with pytest.raises(ClearanceError) as e:
        static_stage(tmpl, problem)
    msg = str(e.value)
    assert ("it passes crankpin J1 at 10.2 mm, under the 11.2 mm a post there needs (4.25 post "
            "radius + 6 link half-width + 1 margin)") in msg
    assert _design("trotbot_heel", **HEEL)[1].plan.top == 12       # the printed crank's plan


# -- optimality against the brute force ---------------------------------------------------


@pytest.mark.parametrize("key", quick(["klann", "trotbot"], ["klann"]))
def test_the_planner_matches_the_brute_force_optimum(key):
    """Every layering and every route up to the planner's stack size: none thinner, and none
    in it with fewer added crank features. (The brute force knows full-layer heads only:
    the planner's with its heads sunk.)"""
    cfg = _cfg(key, heads="sink")
    tmpl = template_for(cfg)
    plan = design_side(tmpl, cfg).plan
    ctx, _, problem = side_problem(tmpl, cfg)
    for top in range(plan.top - 2, plan.top):
        assert brute.solve(problem, top, ctx) is None, f"a plan in {top + 1} layers"
    best = brute.solve(problem, plan.top, ctx)
    ours = brute.cost(plan.choices["crank"], plan.layers, problem.topo.riders, {})
    assert best is not None
    assert ours == best[0]


@pytest.mark.parametrize(("key", "crank", "module", "top"), [
    ("klann", "keyed", "single", 8), ("klann", "printed", "double", 9),
    ("klann", "keyed", "decker", 11), ("trotbot", "keyed", "single", 12),
    ("trotbot", "printed", "single", 12), ("strider", "keyed", "single", 10),
    ("jansen", "keyed", "single", 9)])
def test_the_routers_joint_rules_are_the_brute_forces_on_every_rider_layering(
        key, crank, module, top):
    """The router's joint rules (``JointRules``, compiled from the printed crank's joints)
    against the brute force's own model of those joints (:func:`tests.brute.buildable`, as
    ``realize`` builds them), on hand-made layerings: every layer of the riders under the
    hub, nothing else in the way. The router's cheapest route builds by the brute force's
    joints and costs what the brute force's cheapest does, or neither has one."""
    cfg = _cfg(key, module, crank=crank, heads="sink")
    ctx, _, problem = side_problem(template_for(cfg), cfg, hint=False)
    router, riders = problem.router, problem.topo.riders
    h0 = router.hub_bottom(top)
    names = sorted(riders)
    points = list(dict.fromkeys(riders.values()))
    seen = 0
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
        seen += 1
    assert seen
