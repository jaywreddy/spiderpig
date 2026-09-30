"""The planner never hangs: its node budgets and wall-clock deadline (``StackSpec.max_seconds``)
bound every search, a run cut short says so (:mod:`stack`), and the checks of the
recommendations share one more deadline (:mod:`recommend`)."""

from __future__ import annotations

import time
from dataclasses import replace

import pytest

from spiderpig.config import BuildConfig
from spiderpig.fabricate import design_side, side_problem, template_for
from spiderpig.recommend import recommend
from spiderpig.stack import PlanError, StackSpec, verify_plan


def _problem(key: str, module: str, **spec):
    cfg = BuildConfig(linkage=key, module=module, robot=False)
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    problem.spec = replace(problem.spec, **spec)
    return tmpl, problem


def test_a_plan_cut_short_by_its_budget_is_valid_and_unproven():
    """TrotBot's heel plans in 12 layers within 40 nodes; with 1 node left for the proof, the
    plan comes back valid, unproven, and the proof says what the budget left open."""
    tmpl, problem = _problem("trotbot_heel", "single", quick_nodes=40, max_nodes=2)
    plan = problem.solve()
    assert plan.top == 11
    assert not plan.optimal
    assert "not ruled out (the search stopped at its budget" in plan.proof
    assert verify_plan(plan, tmpl) == []


def test_nothing_found_within_the_node_budget_raises_with_the_tally():
    _, problem = _problem("klann", "double", max_total_nodes=5)
    with pytest.raises(PlanError) as e:
        problem.solve()
    msg = str(e.value)
    assert msg.startswith("klann_double: no layer plan found with up to 7 layers after 6 search "
                          "steps in 0 s; the 5 search-step budget ran out; what blocked it")
    assert e.value.blockers
    assert ("sizes: 3-6 layers ruled out (0-1 nodes each, 0 s in all); 7 layers left open at "
            "their budget (4 nodes, 0 s in all); 8-41 layers not tried") in msg


def test_nothing_found_before_the_deadline_raises_with_the_tally():
    _, problem = _problem("klann", "double", max_seconds=0.0)
    t0 = time.monotonic()
    with pytest.raises(PlanError, match="the 0 s deadline ran out") as e:
        problem.solve()
    assert time.monotonic() - t0 < 5
    assert "sizes: 3-41 layers not tried" in str(e.value)


def test_the_recommendation_checks_share_one_deadline():
    """The heel at its drawing's 7 mm unit stops the static stage; with no time to check
    what would clear it, nothing is recommended and the note says what wasn't checked."""
    cfg = BuildConfig(linkage="trotbot_heel", module="single", robot=False,
                      proportions=(("unit", 7.0),))
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    recs, notes = recommend(cfg, failures=tuple(problem.router.facts.failures), seconds=0.0)
    assert recs == []
    assert notes == ["not checked, the 0 s for checking what would clear it ran out: a scale "
                     "of the linkage from unit 10.5 up and thinner parts"]


@pytest.mark.slow
@pytest.mark.parametrize(("key", "module", "expect"), [
    ("trotbot_heel", "double", "error"), ("trotbot_toe", "double", "error"),
    ("trotbot_toe", "decker", "plan"), ("trotbot_toe", "quad", "either"),
])
def test_the_trotbot_modules_return_inside_the_deadlines(key, module, expect):
    """These ran for over 40 minutes once. A double module puts both legs on one crankpin,
    so one chain of the crank must span both riders, and no layering of the heel's links
    lets a stock screw fit it: the search says so, with the tally, once its deadline is
    up (and the checks of what would clear it theirs). The toe's decker plans in seconds;
    its quad (39 layers) takes most of the deadline, so a slow machine may get the tally
    instead. Whatever the machine, each returns within the deadlines."""
    cfg = BuildConfig(linkage=key, module=module, robot=False)
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
    assert took < 4 * StackSpec().max_seconds
