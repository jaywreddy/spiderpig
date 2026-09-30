"""The crank's route through the stack (:mod:`construction.route`), the body's underside
(:mod:`construction.underside`) and the planner's optimality (:mod:`stack`), checked against
a brute-force reference (:mod:`tests.brute`)."""

from __future__ import annotations

import numpy as np
import pytest

import linkage
from config import BuildConfig
from construction.base import ConstructionError
from construction.crank import CrankRoute, Run
from construction.route import crank_facts
from construction.underside import Underside, underside
from fabricate import (
    SideDesign,
    design_side,
    fabricate_side,
    ground_clearance,
    side_problem,
    template_for,
)
from stack import ClearanceError, verify_plan
from tests import brute


def _cfg(key: str, module: str = "single", **kw) -> BuildConfig:
    return BuildConfig(linkage=key, module=module, robot=False, **kw)


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
    cfg = _cfg("trotbot_heel", proportions=(("unit", 7.0),))
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
        _design("trotbot_heel", proportions=(("unit", 7.0),))
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
    cfg = _cfg("trotbot_heel")
    tmpl = template_for(cfg)
    ctx, groups, problem = side_problem(tmpl, cfg)
    assert not problem.router.hub_play
    assert side_problem(tmpl, _cfg("trotbot_heel", servo="xl330_m288"))[2].router.hub_play
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


@pytest.mark.parametrize(("module", "height"), [("single", 21), ("double", 24), ("decker", 33),
                                                ("quad", 36)])
def test_klann_plans_keep_their_heights_and_are_proven_thinnest(module, height):
    """walk._NOMINAL_LAYERS depends on these."""
    tmpl, design = _design("klann", module)
    assert design.plan.height == height
    assert design.plan.optimal, design.plan.proof
    assert design.plan.proof.startswith(f"no plan in {design.plan.top} layers or fewer")


# -- optimality against the brute force ---------------------------------------------------


@pytest.mark.parametrize("key", ["klann", "trotbot"])
def test_the_planner_matches_the_brute_force_optimum(key):
    """Every layering and every route up to the planner's stack size: none thinner, and none
    in it with fewer added crank features."""
    cfg = _cfg(key)
    tmpl = template_for(cfg)
    plan = design_side(tmpl, cfg).plan
    ctx, _, problem = side_problem(tmpl, cfg)
    for top in range(plan.top - 2, plan.top):
        assert brute.solve(problem, top, ctx) is None, f"a plan in {top + 1} layers"
    best = brute.solve(problem, plan.top, ctx)
    ours = brute.cost(plan.choices["crank"], plan.layers, problem.topo.riders, {})
    assert best is not None
    assert ours == best[0]
