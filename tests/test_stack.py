"""Tests for :mod:`stack`: the claims-based layer planner."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from stack import (
    Claim,
    Geometry,
    Layout,
    Pill,
    Placed,
    StackProblem,
    Topology,
    seg_seg,
    verify_plan,
)


@pytest.fixture(params=["single", "double", "decker", "quad"])
def planned(request, design):
    """``(side template, SideDesign)`` per module."""
    return design(request.param)


def test_seg_seg_is_exact():
    a, b = np.array([[0.0, 0.0]]), np.array([[10.0, 0.0]])
    assert seg_seg(a, b, np.array([[5.0, -1.0]]), np.array([[5.0, 1.0]]))[0] == 0.0  # crossing
    assert seg_seg(a, b, np.array([[0.0, 3.0]]), np.array([[10.0, 3.0]]))[0] == 3.0  # parallel
    assert seg_seg(a, b, np.array([[13.0, 4.0]]), np.array([[20.0, 4.0]]))[0] == 5.0  # end-to-end


def test_distance_is_a_lower_bound_between_samples():
    """A point on a circle passing a fixed point: coarse samples must not overshoot."""
    fine = np.linspace(0, 2 * np.pi, 20000, endpoint=False)
    coarse = fine[::500]

    def geo(ts):
        return Geometry({"p": np.c_[10 * np.cos(ts), 10 * np.sin(ts)], "q": [10.0, 0.4]})

    truth = geo(fine).dist(("pt", "p"), ("pt", "q"))
    assert geo(coarse).dist(("pt", "p"), ("pt", "q")) <= truth + 1e-9


def _toy(links: dict[str, tuple[np.ndarray, np.ndarray]]):
    points, segs = {}, {}
    for n, (p, q) in links.items():
        points[f"{n}.a"], points[f"{n}.b"] = p, q
        segs[n] = ((f"{n}.a", f"{n}.b"),)
    topo = Topology("toy", Geometry(points), segs, (), {})

    def claim(n):
        return Claim(n, frozenset((n,)),
                     lambda L: [Placed(L.layers[n], Pill(f"{n}.a", f"{n}.b", 3.0), n, n)])

    return topo, [claim(n) for n in links]


def test_links_that_collide_get_different_layers():
    ts = np.zeros((4, 1))
    bar = (np.c_[ts * 0, ts * 0], np.c_[ts * 0 + 50, ts * 0])
    crossing = (np.c_[ts * 0 + 25, ts * 0 - 20], np.c_[ts * 0 + 25, ts * 0 + 20])
    far = (np.c_[ts * 0, ts * 0 + 90], np.c_[ts * 0 + 50, ts * 0 + 90])
    topo, claims = _toy({"a": bar, "b": crossing, "c": far})
    plan = StackProblem(topo, claims).solve()
    assert plan.layers["a"] != plan.layers["b"]
    assert plan.top == 3                                 # two link layers between the plates
    assert all(0 < k < plan.top for k in plan.layers.values())


def test_a_claim_that_cannot_be_built_forces_another_layout():
    ts = np.zeros((2, 1))
    topo, claims = _toy({"a": (np.c_[ts * 0, ts * 0], np.c_[ts * 0 + 9, ts * 0])})
    claims.append(Claim("fussy", frozenset("a"), lambda L: None if L.layers["a"] < 3 else []))
    plan = StackProblem(topo, claims).solve()
    assert plan.layers["a"] >= 3


def test_plan_clears_everything_over_the_full_cycle(planned):
    tmpl, design = planned
    assert verify_plan(design.plan, tmpl) == []


def test_verifier_catches_a_bad_plan(design):
    """Negative control: b1 and b2 in one layer is the original layering bug."""
    tmpl, d = design("single")
    plan = d.plan
    broken = dataclasses.replace(plan, layers={**plan.layers, "b2": plan.layers["b1"]})
    assert any("b1" in v and "b2" in v for v in verify_plan(broken, tmpl))


def test_every_pillar_is_held_by_both_frame_plates(planned):
    _, design = planned
    plan = design.plan
    for ax in plan.topo.axes_of("frame"):
        labels = {p.label for p in plan.shapes(f"pillar:{ax.name}")}
        anchors = [p.layer for p in plan.shapes(f"pillar:{ax.name}") if p.label.endswith("anchor")]
        assert sorted(anchors) == [0, plan.top], (ax.name, labels)


def test_crank_crosses_a_b1_layer_only_along_its_crankpin(planned):
    _, design = planned
    plan = design.plan
    riders = {plan.layers[b] for b in plan.topo.riders}
    for p in plan.shapes("crank"):
        if p.layer in riders:
            assert p.seat, p
            assert p.label.startswith("crankpin"), p


def test_every_link_is_held_on_its_axles(planned):
    """Each link on an axle has a shoulder, head, cap, plate or another link either side."""
    _, design = planned
    plan = design.plan
    for ax in plan.topo.axes:
        if ax.kind not in ("pin", "frame"):
            continue
        group = ("pillar:" if ax.kind == "frame" else "pin:") + ax.name
        held = {p.layer for p in plan.shapes(group)
                if not p.label.endswith("neck")} | {plan.layers[m] for m in ax.members}
        for m in ax.members:
            k = plan.layers[m]
            assert {k - 1, k + 1} <= held, (ax.name, m)


def test_layout_layers_between():
    L = Layout({}, 5, 3.0)
    assert list(L.layers_between(2.9, 6.1)) == [0, 1, 2]
    assert list(L.layers_between(3.0, 6.0)) == [1]
