"""Tests for :mod:`stack`: the layer plan and its crankshaft."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from fabricate import plan_for
from klann import (
    build_double_decker_template,
    build_double_double_decker_template,
    build_double_template,
    build_klann_template,
    build_multi_leg_template,
    create_klann_geometry,
)
from stack import Link, StackProblem, seg_seg, verify_plan

TEMPLATES = {
    "single": lambda: build_klann_template(create_klann_geometry()),
    "double": build_double_template,
    "decker": build_double_decker_template,
    "quad": build_double_double_decker_template,
    "multi3": lambda: build_multi_leg_template(3),
}


def test_seg_seg_is_exact():
    a, b = np.array([[0.0, 0.0]]), np.array([[10.0, 0.0]])
    assert seg_seg(a, b, np.array([[5.0, -1.0]]), np.array([[5.0, 1.0]]))[0] == 0.0  # crossing
    assert seg_seg(a, b, np.array([[0.0, 3.0]]), np.array([[10.0, 3.0]]))[0] == 3.0  # parallel
    assert seg_seg(a, b, np.array([[13.0, 4.0]]), np.array([[20.0, 4.0]]))[0] == 5.0  # end-to-end


def test_links_that_collide_get_different_slots():
    ts = np.linspace(0.0, 1.0, 4)
    bar = (np.stack([ts * 0, ts * 0], 1), np.stack([ts * 0 + 50, ts * 0], 1))
    crossing = (np.stack([ts * 0 + 25, ts * 0 - 20], 1), np.stack([ts * 0 + 25, ts * 0 + 20], 1))
    far = (np.stack([ts * 0, ts * 0 + 90], 1), np.stack([ts * 0 + 50, ts * 0 + 90], 1))
    plan = StackProblem([Link("a", (bar,)), Link("b", (crossing,)), Link("c", (far,))]).solve()
    assert plan.slots["a"] != plan.slots["b"]
    assert plan.slots["c"] in (plan.slots["a"], plan.slots["b"])  # it may share


@pytest.mark.parametrize("mode", sorted(TEMPLATES))
def test_plan_clears_everything_over_the_full_cycle(mode):
    tmpl = TEMPLATES[mode]()
    assert verify_plan(plan_for(tmpl), tmpl) == []


def test_verifier_catches_a_bad_plan():
    """Negative control: b1 and b2 on one slot is the original layering bug."""
    tmpl = TEMPLATES["single"]()
    plan = plan_for(tmpl)
    broken = dataclasses.replace(plan, slots={**plan.slots, "b2": plan.slots["b1"]})
    assert any("b1 x b2" in v or "b2 x b1" in v for v in verify_plan(broken, tmpl))


@pytest.mark.parametrize("mode", sorted(TEMPLATES))
def test_crankshaft_never_crosses_a_b1_on_the_axis(mode):
    """Every b1 sweeps over O, so the crank may enter a b1 slot only along
    that b1's own crankpin, with a web beside it reaching that crankpin."""
    plan = plan_for(TEMPLATES[mode]())
    riders = plan.rider_slots
    assert not set(plan.journal_slots) & set(riders)
    lowest = min(riders)
    for slot, pins in riders.items():
        above = riders.get(slot + 1)
        assert above is None or above == pins  # only a b1 on the same crankpin
        if above is None:
            assert set(pins) <= set(plan.webs[slot + 1])
        if slot > lowest and riders.get(slot - 1) is None:
            assert set(pins) <= set(plan.webs[slot - 1])


def test_quad_crank_alternates_webs_and_b1s():
    plan = plan_for(TEMPLATES["quad"]())
    kinds = []
    for k in range(min(plan.rider_slots), plan.plate):
        kinds.append("b1" if k in plan.rider_slots else "web" if k in plan.webs else "journal")
    assert kinds == ["b1", "web"] * 4
