"""Every pipeline stage says what fails, without further investigation."""

from __future__ import annotations

import math
import re

import pytest

from spiderpig import linkage, stack
from spiderpig.config import BuildConfig
from spiderpig.fabricate import design_side, side_clearances, side_problem, template_for
from spiderpig.stack import PlanError


def test_program_stage_names_the_loop_that_cannot_close():
    with pytest.raises(linkage.AssemblyError) as e:
        linkage.build_module_template("single", proportions={"MC": 0.3}, linkage="klann")
    msg = str(e.value)
    assert msg.startswith("klann: joint C can't be placed for")
    assert "bars A-C 54.5 mm and M-C 18.0 mm miss each other by up to" in msg


def test_program_stage_reports_margins_and_toggles():
    steps = {s.point: s for s in linkage.get("jansen").check()}
    c = steps["C"]
    assert c.kind == "closure"
    assert c.fails_deg is None
    assert c.margin_mm == pytest.approx(1.2 * 1.6, abs=0.05)   # (|MA|min - (k - c)) units
    assert c.toggles                                           # 9° transmission angle
    assert steps["D"].kind == "rigid"


def test_static_clearances_state_the_forced_layer_facts():
    # the standoff pillar's narrowest ring, (6 + 0.35) / 2 + a 1.5 mm wall = 4.675 mm, the
    # link's 6 mm radius and the 1 mm margin: the 11.7 mm (Params, StandoffAxle.dims)
    cfg = BuildConfig(linkage="jansen", module="single", robot=False, proportions=(("unit", 1.5),))
    ctx, groups, _ = side_problem(template_for(cfg), cfg)
    facts = [c.describe() for c in side_clearances(ctx, groups)]
    assert ("b2 passes pillar:A at 8.7 mm, under the 11.7 mm its thinnest part needs, so it "
            "can't be in any layer pillar:A spans") in facts


@pytest.mark.slow
@pytest.mark.usefixtures("fresh_plan_memo")
def test_plan_stage_names_what_blocked_it(monkeypatch):
    """The plan stage's error: the tally of what blocked it and the static clearances
    behind it (what would clear it: the test below, from the same clearance).

    Deterministic: the planner's CPU-seconds deadline (``stack.MAX_SECONDS``) is taken out,
    so the search is bounded by its node budgets alone and gets as far on a loaded machine
    as on an idle one (~110 s). On the default constructions (the bolt crank, standoff
    pillars, Chicago screw pins)."""
    monkeypatch.setattr(stack, "MAX_SECONDS", math.inf)
    cfg = BuildConfig(linkage="jansen", module="decker", robot=False, proportions=(("unit", 1.5),))
    assert (cfg.crank, cfg.pillar, cfg.pin) == ("bolt", "standoff", "chicago")
    assert math.isinf(stack.StackSpec().max_seconds)
    with pytest.raises(PlanError) as e:
        design_side(template_for(cfg), cfg, advise=False)
    msg = str(e.value)
    assert "no layer plan found with up to 61 layers" in msg
    assert "the 60000 search-step budget ran out; what blocked it" in msg
    # the crank's routes, the standoff pillar's ring against the link that passes it, and
    # the Chicago screw pins' heads and caps against each other (the tally of a search
    # bounded by its node budget alone: the same on any machine)
    assert "x crank route: no crank route passes" in msg
    assert "x pillar:A_leg0 shoulder vs b2_leg0: -1.3 mm apart in one layer, need 1.0" in msg
    assert "x pin:B_leg1 head vs pin:B_leg0 cap: -8.7 mm apart in one layer, need 1.0" in msg
    assert "static clearances behind it:" in msg
    assert re.search(r"b2_leg\d passes pillar:A_leg\d at 8.7 mm, under the 11.7 mm its "
                     r"thinnest part needs, so it can't be in any layer pillar:A_leg\d spans",
                     msg)


def test_legs_that_must_sit_in_disjoint_blocks_are_found():
    """Strider's legs sweep across each other's pins: one leg's block above the other's,
    and nothing thinner (the search ran to the end: the default constructions' 14 layers
    are proven thinnest; the plan from the fabrication cache, which keeps its proof)."""
    from tests import cache

    cfg = BuildConfig(linkage="strider", module="double", robot=False)
    plan = cache.cached_design(cfg)[1].plan
    leg0 = [k for n, k in plan.layers.items() if n.endswith("_leg0")]
    leg1 = [k for n, k in plan.layers.items() if n.endswith("_leg1")]
    assert max(leg0) < min(leg1)
    assert plan.optimal, plan.proof


def test_the_plan_stages_recommendations_clear_the_clearance(monkeypatch):
    """What would clear a static clearance behind a plan's failure (b2 past pillar A, the
    test above's), each checked by planning it with the clock taken out, and each clearing
    it: the design it makes no longer has that clearance. (The Jansen single plans with the
    clearance, so this checks the recommendations' own claim, not only that they plan.)"""
    from dataclasses import replace

    from spiderpig.recommend import recommend

    monkeypatch.setattr(stack, "MAX_SECONDS", math.inf)
    cfg = BuildConfig(linkage="jansen", module="single", robot=False, proportions=(("unit", 1.5),))
    ctx, groups, problem = side_problem(template_for(cfg), cfg)
    involved = tuple(c for c in problem.clearances if c.keepout.owner == "pillar:A")
    assert {c.link for c in involved} == {"b2"}
    (c,) = involved
    assert (round(c.dist, 1), round(c.need, 1)) == (8.7, 11.7)
    recs, notes = recommend(cfg, clearances=involved, plan=True)
    # no thinner part at this scale clears it (a thinner link would leave the crank's 8.5 mm
    # sleeve too little rider): the scale is the recommendation, the note says why no other
    assert [n.split(" (")[0] for n in notes] == [
        "no part sizes at this scale clear it within the constructions' limits"]
    assert recs
    for r in recs:
        assert r.verified.startswith("checked: the static stage passes, and it plans in")
        props = dict(cfg.proportions)
        params = cfg.params
        for name, _, to in r.changes:
            if hasattr(params, name):
                params = replace(params, **{name: to})
            else:
                props[name] = to
        trial = replace(cfg, proportions=tuple(sorted(props.items())), params=params)
        ctx, groups, _ = side_problem(template_for(trial), trial)
        left = [x.describe() for x in side_clearances(ctx, groups)
                if x.link == "b2" and x.keepout.owner == "pillar:A"]
        assert left == [], (r.changes, left)
