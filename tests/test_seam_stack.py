"""Seam tests of the layer planner (:mod:`stack`) on hand-built problems: links as segments
the test states (``tests/_ctx.py``), claims made by hand, the planner's thinnest stack
checked against the brute force (``tests/brute.py``: every layering). No linkage, no
fabrication (``no_fabricate``)."""

from __future__ import annotations

import random

import numpy as np
import pytest

from spiderpig.stack import (
    Claim,
    Disc,
    Layout,
    Placed,
    PlanError,
    StackProblem,
    StackSpec,
    Unbuildable,
    made,
    verify_plan,
)
from tests import _ctx, brute

pytestmark = pytest.mark.no_fabricate


def _problem(segs: dict[str, tuple], extra=(), r: float = 3.0, **spec) -> StackProblem:
    """Links as fixed segments ``name -> ((x0, y0), (x1, y1))``, each claiming a pill of
    radius ``r`` in its layer, and ``extra`` claims."""
    points, links = {}, {}
    for n, (a, b) in segs.items():
        points[f"{n}.a"], points[f"{n}.b"] = a, b
        links[n] = ((f"{n}.a", f"{n}.b"),)
    topo = _ctx.topology(points, links, axles=())
    claims = [_ctx.link_claim(n, r, topo) for n in segs]
    return StackProblem(topo, [*claims, *extra], StackSpec(**spec))


def _least_top(problem: StackProblem, most: int = 8) -> int | None:
    for top in range(2, most + 1):
        if brute.solve(problem, top, None) is not None:
            return top
    return None


def test_two_crossing_links_take_two_layers_a_third_far_one_shares():
    """a and b cross: two link layers between the plates (top 3); c, 90 mm off, shares one."""
    p = _problem({"a": ((0, 0), (50, 0)), "b": ((25, -20), (25, 20)),
                  "c": ((0, 90), (50, 90))})
    plan = p.solve()
    assert plan.top == 3 == _least_top(p)
    assert plan.layers["a"] != plan.layers["b"]
    assert plan.optimal
    assert verify_plan(plan) == []


def test_three_mutually_crossing_links_take_three_layers():
    p = _problem({"a": ((0, 0), (50, 0)), "b": ((25, -20), (25, 20)),
                  "c": ((0, -20), (50, 20))})
    plan = p.solve()
    assert plan.top == 4 == _least_top(p)
    assert len(set(plan.layers.values())) == 3


def test_links_a_margin_apart_share_a_layer_and_closer_ones_dont():
    """Pills of radius 3 with the 1 mm margin: centre lines 7 mm apart clear, 6.9 don't."""
    apart = _problem({"a": ((0, 0), (50, 0)), "b": ((0, 7.0), (50, 7.0))})
    close = _problem({"a": ((0, 0), (50, 0)), "b": ((0, 6.9), (50, 6.9))})
    assert apart.solve().top == 2 == _least_top(apart)
    assert close.solve().top == 3 == _least_top(close)


def test_a_fixed_part_in_a_layer_pushes_a_link_out_of_it():
    """A ring round a point on a's line in layers 1 and 2: a goes to layer 3 (top 4)."""
    ring = _ctx.disc_claim("ring", "a.a", 4.0, (1, 2))
    p = _problem({"a": ((0, 0), (50, 0))}, extra=[ring])
    plan = p.solve()
    assert plan.layers["a"] == 3
    assert plan.top == 4 == _least_top(p)


def test_a_claim_that_refuses_low_layers_moves_its_link_up():
    fussy = Claim("fussy", frozenset("a"), lambda L: None if L.layers["a"] < 3 else [])
    p = _problem({"a": ((0, 0), (9, 0))}, extra=[fussy])
    plan = p.solve()
    assert plan.layers["a"] == 3
    assert _least_top(p) == plan.top == 4


@pytest.mark.parametrize("seed", range(6))
def test_the_planners_thinnest_stack_is_the_brute_forces(seed):
    """Random sets of four or five bars in a 60 mm square: the planner's least stack is the
    least any layering has (every layering tried), and its plan verifies."""
    rng = random.Random(seed)
    segs = {f"l{i}": ((rng.uniform(0, 60), rng.uniform(0, 60)),
                      (rng.uniform(0, 60), rng.uniform(0, 60)))
            for i in range(rng.choice((4, 5)))}
    p = _problem(segs)
    plan = p.solve()
    assert plan.top == _least_top(p)
    assert plan.optimal
    assert verify_plan(plan) == []


def test_a_layer_bound_with_no_plan_raises_with_the_sizes_ruled_out():
    p = _problem({"a": ((0, 0), (50, 0)), "b": ((25, -20), (25, 20)),
                  "c": ((0, -20), (50, 20))}, max_top=3)
    with pytest.raises(PlanError) as e:
        p.solve()
    assert not e.value.expired
    assert "sizes: 3-4 layers ruled out" in str(e.value)


def test_moving_links_are_checked_over_the_whole_cycle():
    """b turns round a's middle: it meets a at some angle, so they never share a layer,
    though at the first sample they are 20 mm apart."""
    turning = _ctx.turning(20.0, 90.0, centre=(25.0, 0.0))
    topo = _ctx.topology({"a.a": (0, 0), "a.b": (50, 0), "b.a": turning,
                          "b.b": turning + np.array([0.0, 5.0])},
                         {"a": (("a.a", "a.b"),), "b": (("b.a", "b.b"),)}, axles=())
    p = StackProblem(topo, [_ctx.link_claim(n, 3.0, topo) for n in ("a", "b")])
    assert p.solve().top == 3 == _least_top(p)


# -- the claims' API ---------------------------------------------------------------------------


def test_made_says_why_a_claim_cant_be_built():
    def refuse(L):
        raise Unbuildable("no stock part fits")

    L = Layout({"a": 1}, 4, 3.0)
    assert made(Claim("g", frozenset("a"), refuse), L) == (None, "g: no stock part fits")
    assert made(Claim("g", frozenset("a"), lambda L: None), L) == (
        None, "g can't be built in this layout")
    shape = Placed(1, Disc("O", 2.0), "g", "g")
    assert made(Claim("g", frozenset("a"), lambda L: [shape]), L) == ([shape], "")


def test_the_layouts_z_counts_thicker_layers_and_gaps_below():
    """Layers 3 mm, layer 1 a 2.54 mm plate, a 1.5 mm gap over layer 2: layer 3 starts at
    3 + 2.54 + 3 + 1.5 = 10.04; the stack's height adds the rest."""
    L = _ctx.layout(top=4, thick={1: 2.54}, gaps={2: 1.5})
    assert L.z(3) == pytest.approx((10.04, 13.04))
    assert L.gap_z(2) == pytest.approx((8.54, 10.04))
    assert L.gap_z(3) == pytest.approx((13.04, 13.04))      # no gap: empty
    assert (L.t(1), L.t(2), L.gap(2), L.gap(0)) == (2.54, 3.0, 1.5, 0.0)
    assert L.height() == pytest.approx(16.04)
    assert list(L.layers_between(8.6, 10.0)) == []          # inside the gap only
    assert list(L.layers_between(5.0, 11.0)) == [1, 2, 3]


def test_layers_under_zero_count_down_from_the_outer_plate():
    L = _ctx.layout(top=4, thick={-1: 2.0}, gaps={-1: 0.5})
    assert L.z(-1) == pytest.approx((-2.5, -0.5))


# -- the brute force's routes ---------------------------------------------------------------------


def test_brute_routes_put_every_riders_layer_in_a_run_of_its_point():
    """Layers 2-4, a rider of M in layer 3: every route has a run along M through layer 3
    whose upper web (b + 1) isn't a rider's layer, the runs at least a layer apart."""
    got = {tuple((r.at, r.lo, r.hi) for r in route.runs)
           for route in brute.routes(["M"], 2, 4, {3: "M"})}
    assert got == {(("M", 2, 3),), (("M", 2, 4),), (("M", 3, 3),), (("M", 3, 4),)}


def test_brute_routes_never_run_two_points_through_one_run():
    """Riders of M in 2 and N in 3: a run holds one point, so two runs, apart."""
    got = list(brute.routes(["M", "N"], 2, 6, {2: "M", 4: "N"}))
    assert got
    for route in got:
        assert all(len({r.at}) == 1 for r in route.runs)
        assert any(r.at == "M" and r.lo <= 2 <= r.hi for r in route.runs)
        assert any(r.at == "N" and r.lo <= 4 <= r.hi for r in route.runs)


def test_brute_cost_counts_run_layers_no_rider_needs():
    from spiderpig.construction.crank import CrankRoute, Run

    route = CrankRoute((Run("M", 2, 4),))
    riders = {"b1": "M"}
    assert brute.cost(route, {"b1": 3}, riders, {}) == (2, 0, 0)
    assert brute.cost(CrankRoute((Run("M", 3, 3),), bearing=False), {"b1": 3}, riders,
                      {}) == (0, 0, 1)


# -- the guard ------------------------------------------------------------------------------


def test_the_no_fabricate_guard_refuses_a_fabrication():
    """These seam tests are marked ``no_fabricate``: ``fabricate`` and ``fabricate_side``
    refuse to run (through any name they were imported under)."""
    from spiderpig import fabricate
    from spiderpig.fabricate import fabricate_side

    with pytest.raises(AssertionError, match="no_fabricate"):
        fabricate.fabricate(None)
    with pytest.raises(AssertionError, match="no_fabricate"):
        fabricate_side(None, None)


# -- the opt-in search strategies (StackSpec.symmetry, StackSpec.workers) --------------------


def _two_legs(**spec) -> StackProblem:
    """Two legs half a turn apart: each a bar turning 10 mm round O with a 30 mm tail and a
    second bar across it; leg 1's paths are leg 0's run half a cycle later."""
    pts = {"O": (0.0, 0.0)}
    links = {}
    for leg, phase in ((0, 0.0), (1, 180.0)):
        p = _ctx.turning(10.0, phase)
        pts[f"p{leg}"], pts[f"q{leg}"] = p, p + np.array([30.0, 0.0])
        pts[f"r{leg}"], pts[f"s{leg}"] = p + np.array([15.0, -12.0]), p + np.array([15.0, 12.0])
        links[f"a_leg{leg}"] = ((f"p{leg}", f"q{leg}"),)
        links[f"b_leg{leg}"] = ((f"r{leg}", f"s{leg}"),)
    topo = _ctx.topology(pts, links, axles=())
    return StackProblem(topo, [_ctx.link_claim(n, 3.0, topo) for n in links], StackSpec(**spec))


def test_the_leg_swap_is_found_as_a_symmetry():
    from spiderpig.stack_symmetry import symmetries

    syms = symmetries(_two_legs())
    assert syms
    g, sig = syms[0]
    assert g == {"a_leg0": "a_leg1", "a_leg1": "a_leg0", "b_leg0": "b_leg1", "b_leg1": "b_leg0"}
    assert (sig["p0"], sig["O"]) == ("p1", "O")


def test_no_symmetry_where_the_legs_differ():
    from spiderpig.stack_symmetry import symmetries

    p = _two_legs()
    p.topo.links["b_leg1"] = (("r1", "q1"),)          # leg 1's second bar is another bar
    assert symmetries(p) == []


def test_searching_one_of_each_mirror_pair_finds_the_serial_plan():
    """``symmetry``: the same thinnest stack and route cost as the plain search."""
    plain = _two_legs().solve()
    sym = _two_legs(symmetry=True).solve()
    assert (sym.top, sym.cost, sym.optimal) == (plain.top, plain.cost, plain.optimal)
    assert verify_plan(sym) == []


def test_the_sizes_searched_in_worker_processes_give_the_serial_answer():
    """``workers``: every size's search forked into a worker, the coordinator's answer the
    serial one, its proof included."""
    plain = _two_legs().solve()
    pooled = _two_legs(workers=2).solve()
    assert (pooled.top, pooled.layers, pooled.cost, pooled.optimal, pooled.proof) == (
        plain.top, plain.layers, plain.cost, plain.optimal, plain.proof)
