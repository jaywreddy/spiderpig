"""The planner never hangs: its node budgets and CPU-time deadline (``StackSpec.max_seconds``)
bound every search, a run cut short says so (:mod:`stack`), and the checks of the
recommendations share one more deadline (:mod:`recommend`)."""

from __future__ import annotations

import time
from dataclasses import replace

import pytest

from spiderpig import stack
from spiderpig.config import BuildConfig
from spiderpig.fabricate import design_side, side_problem, template_for
from spiderpig.recommend import recommend
from spiderpig.stack import PlanError, StackSpec, verify_plan


def _problem(key: str, module: str, **spec):
    # the printed crank: these budgets were measured with it, and the keyed crank's 8.5 mm
    # post stops TrotBot's heel before the planner (tests/test_route.py)
    cfg = BuildConfig(linkage=key, module=module, robot=False, crank="printed")
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    problem.spec = replace(problem.spec, **spec)
    return tmpl, problem


def test_a_plan_cut_short_by_its_budget_is_valid_and_unproven():
    """TrotBot's heel plans within 40 nodes with its heads sunk into layers (the gap search
    runs only when that finds none: the user's rule of 2026-10-04, ``StackSpec.heads``
    "best"); with 1 node left for the proof, the plan comes back valid, unproven, and the
    proof says what the budget left open."""
    tmpl, problem = _problem("trotbot_heel", "single", quick_nodes=40, max_nodes=2)
    plan = problem.solve()
    assert plan.heads == "sink"
    assert "against" not in plan.proof                      # no gap search
    assert "with the heads" not in plan.proof
    assert not plan.optimal
    assert "not ruled out (the search stopped at its budget" in plan.proof
    assert verify_plan(plan, tmpl) == []


def test_nothing_found_within_the_node_budget_raises_with_the_tally():
    _, problem = _problem("klann", "double", max_total_nodes=5)
    with pytest.raises(PlanError) as e:
        problem.solve()
    msg = str(e.value)
    # 7 layers are ruled out at once since the standoff pillars (no stock standoff fills
    # pillar A's 9 mm column: goBILDA's shortest is 12 mm)
    assert msg.startswith("klann_double: no layer plan found with up to 8 layers after 6 search "
                          "steps in 0 CPU s; the 5 search-step budget ran out; what blocked it")
    assert e.value.blockers
    assert "no stock standoffs (12-60 mm)" in msg
    assert ("sizes: 3-7 layers ruled out (0-1 nodes each, 0 s in all); 8 layers left open at "
            "their budget (4 nodes, 0 s in all); 9-61 layers not tried") in msg


def test_nothing_found_before_the_deadline_raises_with_the_tally():
    _, problem = _problem("klann", "double", max_seconds=0.0)
    t0 = time.monotonic()
    with pytest.raises(PlanError, match="the 0 CPU s deadline ran out") as e:
        problem.solve()
    assert time.monotonic() - t0 < 5
    assert "sizes: 3-61 layers not tried" in str(e.value)


def test_the_recommendation_checks_share_one_deadline():
    """The heel at its drawing's 7 mm unit stops the static stage; with no time to check
    what would clear it, nothing is recommended and the note says what wasn't checked."""
    cfg = BuildConfig(linkage="trotbot_heel", module="single", robot=False, crank="printed",
                      proportions=(("unit", 7.0),))
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    recs, notes = recommend(cfg, failures=tuple(problem.router.facts.failures), seconds=0.0)
    assert recs == []
    assert notes == ["not checked, the 0 s for checking what would clear it ran out: a scale "
                     "of the linkage from unit 10.5 up and thinner parts"]


@pytest.mark.slow
@pytest.mark.parametrize(("key", "module", "expect"), [
    ("trotbot_heel", "double", "either"), ("trotbot_toe", "double", "either"),
    ("trotbot_toe", "decker", "plan"), ("trotbot_toe", "quad", "either"),
])
def test_the_trotbot_modules_return_inside_the_deadlines(key, module, expect, monkeypatch):
    """These ran for over 40 minutes once. (Since the clearance gaps of 2026-10-04 the doubles
    may plan: with the heads in gaps the chain finds a layering.) A double module puts both
    legs on one crankpin,
    so one chain of the crank must span both riders, and no layering of the heel's links
    lets a stock screw fit it: the search says so, with the tally, once its deadline is
    up (and the checks of what would clear it theirs). The toe's decker plans in seconds;
    its quad (39 layers) takes most of the deadline, so a slow machine may get the tally
    instead. Whatever the machine, each returns within the deadlines. (The "either" cases
    under a 15 s deadline, what the bound is checked against too: the heads sunk, then in
    gaps, then the checks of what would clear it, each up to its deadline, took 4 minutes
    a case at 60 s for the same verdict.)"""
    if expect == "either":
        monkeypatch.setattr(stack, "MAX_SECONDS", 15.0)
    cfg = BuildConfig(linkage=key, module=module, robot=False, crank="printed")
    t0 = time.monotonic()
    try:
        outcome = design_side(template_for(cfg), cfg)
    except PlanError as e:
        outcome = e
    took = time.monotonic() - t0
    if isinstance(outcome, PlanError):
        assert expect != "plan"
        assert "sizes: " in str(outcome)
        assert " ran out" in str(outcome)
    else:
        assert expect != "error"
        assert verify_plan(outcome.plan) == []
    # heads in layers and in gaps: two searches, each within the deadline (StackSpec.heads)
    assert took < 7 * StackSpec().max_seconds


@pytest.mark.slow
def test_a_single_plate_crank_falls_back_to_the_pivots_heads_sunk():
    """TrotBot's heel with the default single-plate crank: in clearance gaps no crank route
    keeps the crankpin's washers clear of the pins' caps, so the gap search gives up after
    ``stack.GIVE_UP`` such layerings (seconds, not its 60 s deadline), and the plan has the
    pivots' heads sunk into layers with the crank's and the drive's screws still in gaps
    (``heads="gap_sink"``): 13 layers (14 before the simplified hardware's printed fills),
    proven, and it verifies."""
    from spiderpig.stack import GIVE_UP

    cfg = BuildConfig(linkage="trotbot_heel", module="single", robot=False)
    t0 = time.monotonic()
    plan = design_side(template_for(cfg), cfg).plan
    assert time.monotonic() - t0 < StackSpec().max_seconds
    assert plan.heads == "sink"
    assert plan.top + 1 == 13
    assert plan.optimal
    assert f"with the heads gap: none (it gave up after {GIVE_UP} layerings" in plan.proof
    crank_heads = [p for p in plan.placed if p.group == "crank" and p.gap and p.height > 0]
    assert crank_heads                          # its screw heads beyond its webs, in gaps
    # most pivot heads sunk into the layer beside their link (the rest keep a gap the plan
    # has anyway, or need more than a layer gives at the plan's z)
    sunk = [k for k in plan.sunk if k[0].startswith(("pin:", "pillar:"))]
    in_gaps = [p for p in plan.placed if p.group.startswith(("pin:", "pillar:"))
               and p.gap and p.toward]
    assert len(sunk) > len(in_gaps)
    assert verify_plan(plan, template_for(cfg)) == []
