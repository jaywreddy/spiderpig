"""Every pipeline stage says what fails, without further investigation."""

from __future__ import annotations

import re

import pytest

import linkage
from fabricate import BuildConfig, design_side, side_clearances, side_problem, template_for
from stack import PlanError


def test_program_stage_names_the_loop_that_cannot_close():
    with pytest.raises(linkage.AssemblyError) as e:
        linkage.build_module_template("single", proportions={"MC": 0.3})
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
    cfg = BuildConfig(linkage="jansen", module="single", robot=False, proportions=(("unit", 1.5),))
    ctx, groups, _ = side_problem(template_for(cfg), cfg)
    facts = [c.describe() for c in side_clearances(ctx, groups)]
    assert ("b2 passes pillar:A at 8.7 mm, under the 9.0 mm its thinnest part needs, so it "
            "can't be in any layer pillar:A spans") in facts


def test_plan_stage_names_what_blocked_it():
    cfg = BuildConfig(linkage="jansen", module="decker", robot=False, proportions=(("unit", 1.5),))
    with pytest.raises(PlanError) as e:
        design_side(template_for(cfg), cfg)
    msg = str(e.value)
    assert "what blocked it" in msg
    assert "pin:B_leg1 head vs pin:B_leg0 cap: -8.7 mm apart in one layer, need 1.0" in msg
    assert "static clearances behind it:" in msg
    assert re.search(r"b2_leg\d passes pillar:A_leg\d at 8.7 mm, under the 9.0 mm its thinnest "
                     r"part needs, so it can't be in any layer pillar:A_leg\d spans", msg)
    # and what would clear it, checked by planning it: a larger scale, or thinner links
    assert "what would clear it:" in msg
    scaled, thinner = e.value.recommendations
    assert scaled.changes == (("unit", 1.5, 1.6),)
    assert scaled.effects.startswith("crank 22.5 -> 24.0 mm")
    assert thinner.changes == (("link_radius", 6.0, 5.5),)
    for r in (scaled, thinner):
        assert r.verified == "checked: the static stage passes, and it plans in 13 layers (39 mm)"


def test_legs_that_must_sit_in_disjoint_blocks_are_found():
    """Strider's legs sweep across each other's pins: one leg's block above the other's,
    and nothing thinner (the search ran to the end)."""
    cfg = BuildConfig(linkage="strider", module="double", robot=False)
    plan = design_side(template_for(cfg), cfg).plan
    leg0 = [k for n, k in plan.layers.items() if n.endswith("_leg0")]
    leg1 = [k for n, k in plan.layers.items() if n.endswith("_leg1")]
    assert max(leg0) < min(leg1)
    assert plan.optimal, plan.proof
