"""Clearance gaps, layer thicknesses and per-part sheets (stage 1 of the assembly work,
2026-10-04): the planner reserves a thin gap for a fastener's head that a link would sweep
in the layer beside it (:attr:`stack.Placed.gap`), sizes each gap from the heads it holds,
and takes each layer at its plates' own thickness (:func:`stack.finalize`); the parts are
cut from their own sheets (:mod:`spiderpig.materials`), packed and bought per sheet, and
checked against the services' cut rules (:mod:`spiderpig.manufacture`)."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest
from build123d import Box, Cylinder, Location

from spiderpig import materials
from spiderpig.config import BuildConfig, ParamError
from spiderpig.construction.pivots.chicago import ChicagoShaft
from spiderpig.manufacture import part_issues
from spiderpig.mechanism import Body
from spiderpig.stack import (
    Claim,
    Disc,
    Geometry,
    Layout,
    Pill,
    Placed,
    PlanError,
    StackProblem,
    StackSpec,
    Topology,
    bridges,
    verify_plan,
)


def test_layout_z_takes_each_layer_and_gap_at_its_own_thickness():
    L = Layout({}, 4, 3.0, gaps={1: 2.0}, thick={0: 3.175, 4: 3.175}, final=True)
    assert L.z(0) == (0.0, 3.175)
    assert L.z(1) == pytest.approx((3.175, 6.175))
    assert L.gap_z(1) == pytest.approx((6.175, 8.175))
    assert L.z(2) == pytest.approx((8.175, 11.175))
    assert L.z(4)[1] == pytest.approx(3.175 * 2 + 3 * 3.0 + 2.0)
    assert L.height() == pytest.approx(L.z(4)[1])
    assert L.z(-1) == pytest.approx((-3.0, 0.0))
    assert list(L.layers_between(6.5, 9.0)) == [2]          # the gap holds no layer
    assert list(L.layers_between(5.0, 9.0)) == [1, 2]
    plain = Layout({}, 4, 3.0)
    assert plain.z(3) == (9.0, 12.0)
    assert plain.gap_z(3) == (12.0, 12.0)


def _toy(head_h: float = 2.0, heads: str = "gap", b_x: float = 20.0):
    """Link ``a`` with a head on its top face at P; link ``b`` (at x = ``b_x``: right over
    P, by default) must sit in the layer over ``a``; link ``c`` elsewhere."""
    ts = np.zeros((4, 1))
    pts = {"a0": np.c_[ts * 0, ts * 0], "P": np.c_[ts * 0 + 20, ts * 0],
           "b0": np.c_[ts * 0 + b_x, ts * 0 - 20], "b1": np.c_[ts * 0 + b_x, ts * 0 + 20],
           "c0": np.c_[ts * 0, ts * 0 + 90], "c1": np.c_[ts * 0 + 40, ts * 0 + 90]}
    links = {"a": (("a0", "P"),), "b": (("b0", "b1"),), "c": (("c0", "c1"),)}
    topo = Topology("toy", Geometry(pts), links, (), {})
    def link_claim(n: str, a: str, b: str) -> Claim:
        return Claim(n, frozenset((n,)), lambda L: [Placed(L.layers[n], Pill(a, b, 3.0), n, n)])

    claims = [link_claim(n, *segs[0]) for n, segs in links.items()]
    claims.append(Claim("head", frozenset("a"), lambda L: [
        Placed(L.layers["a"], Disc("P", 4.0), "head", "head", gap=True, height=head_h,
               toward=+1)]))
    claims.append(Claim("ab", frozenset("ab"), lambda L: [] if L.layers["b"] ==
                        L.layers["a"] + 1 else None))       # b must sit right over a
    return topo, claims, StackSpec(heads=heads)


def test_a_head_a_link_sweeps_over_gets_a_clearance_gap():
    topo, claims, spec = _toy()
    plan = StackProblem(topo, claims, spec).solve()
    a = plan.layers["a"]
    assert plan.layers["b"] == a + 1
    assert plan.gaps == {a: 2.0}                       # the thinnest gap over its 2 mm
    assert plan.heads == "gap"
    (head,) = plan.shapes("head")
    assert head.gap
    assert plan.slot_z(head) == plan.gap_z(a)
    assert plan.z(a + 1)[0] == pytest.approx(plan.z(a)[1] + 2.0)
    assert verify_plan(plan) == []


def test_with_the_heads_sunk_the_link_cannot_pass_and_no_plan_exists():
    topo, claims, spec = _toy(heads="sink")
    with pytest.raises(PlanError, match="no layer plan found"):
        StackProblem(topo, claims, replace(spec, max_top=6, max_seconds=math.inf)).solve()


def test_a_head_nothing_passes_sinks_into_the_layer_and_makes_no_gap():
    topo, claims, spec = _toy(b_x=60.0)          # b over a, but nowhere near the head
    plan = StackProblem(topo, claims, spec).solve()
    assert plan.gaps == {}
    (head,) = plan.shapes("head")
    assert not head.gap
    assert head.layer == plan.layers["a"] + 1 == plan.layers["b"]
    assert verify_plan(plan) == []


def test_best_keeps_the_lower_stack():
    topo, claims, spec = _toy()
    plan = StackProblem(topo, claims, replace(spec, heads="best")).solve()
    assert plan.heads == "gap"
    assert plan.gaps


def test_a_gap_too_tall_for_any_stock_is_refused():
    topo, claims, spec = _toy(head_h=9.0)
    with pytest.raises(PlanError, match="no layer plan found"):
        StackProblem(topo, claims, replace(spec, max_top=6, max_seconds=math.inf)).solve()


def test_what_runs_through_a_gap_is_in_it():
    shapes = [Placed(2, Disc("O", 5.0), "crank", "body", sheet=3.175),
              Placed(3, Disc("O", 4.0), "crank", "body", sheet=3.175),
              Placed(3, Disc("P", 4.0), "pin", "washer", gap=True)]
    (got,) = bridges(shapes, {2: 1.5})
    assert got.gap
    assert (got.layer, got.shape) == (2, Disc("O", 4.0))
    assert bridges(shapes, {}) == []


def test_layers_take_their_plates_thickness():
    topo, claims, spec = _toy()
    spec = replace(spec, frame_t=3.175, link_t=(("c", 3.175),))
    plan = StackProblem(topo, claims, spec).solve()
    assert plan.t(0) == plan.t(plan.top) == 3.175
    assert plan.t(plan.layers["c"]) == 3.175
    assert plan.t(plan.layers["b"]) == 3.0


def test_washer_stacks_fill_a_gap_to_the_shim():
    items, left = materials.washer_stack(4.0, 2.3)
    assert items[0][0] == "ptfe_washer_4x8x0p5"
    assert sum(t for _, t in items) + left == pytest.approx(2.3)
    assert left < 0.1
    assert materials.washer_od(6.0) == 12.0


def test_chicago_heights_split_the_shims_between_two_gaps():
    s = ChicagoShaft()
    L = Layout({}, 8, 3.0, gaps={1: 2.3, 4: 2.3}, final=True)
    lo, hi = s.end_heights(L, 2, 4)
    base_lo, base_hi = s.base_heights()
    assert lo >= base_lo
    assert hi >= base_hi
    assert abs((lo - base_lo) - (hi - base_hi)) < 0.2 + 0.11        # shims, half each end


def test_sheets_per_part():
    """The thinnest stock sheet per part (2026-10-04): 0.080 in 5052 frame plates, 0.063 in
    crank plates (materials.thinnest_sheet), acrylic links; a Klann variant's foot link in
    6061, and klann_lego's crank rider b1 too (the user's decision 3 of 2026-10-04: its jam
    load is past what acrylic holds, strength.link_rows)."""
    cfg = BuildConfig()
    assert (cfg.sheet, cfg.frame_sheet, cfg.crank_sheet) == ("acrylic_3mm", "al5052_2mm",
                                                              "al5052_1p6mm")
    assert cfg.frame_sheet == materials.thinnest_sheet("frame")
    assert cfg.crank_sheet == materials.thinnest_sheet("crank")
    assert materials.link_sheets(cfg) == {}
    kl = BuildConfig(linkage="klann_lego", module="single")
    assert materials.link_sheets(kl) == {"b4": "al6061_3p2mm", "b1": "al6061_3p2mm"}
    assert materials.sheet_of(kl, "link", "b4_leg1") == "al6061_3p2mm"     # the foot link
    assert materials.sheet_of(kl, "link", "b1_leg1") == "al6061_3p2mm"     # the crank rider
    assert materials.sheet_of(kl, "link", "b2_leg1") == "acrylic_3mm"
    patent = BuildConfig(linkage="klann_patent", module="single")
    assert materials.link_sheets(patent) == {"b4": "al6061_3p2mm"}      # its foot link only
    acrylic = BuildConfig(linkage="klann_lego", module="single", link_sheets=())
    assert materials.link_sheets(acrylic) == {}
    assert BuildConfig(linkage="klann_lego", module="single",
                       link_sheets=(("b4", "al6061_3p2mm"), ("b1", "al6061_3p2mm"))
                       ).link_sheets is None             # the default, dropped
    foot_only = BuildConfig(linkage="klann_lego", module="single",
                            link_sheets=(("b4", "al6061_3p2mm"),))
    assert materials.link_sheets(foot_only) == {"b4": "al6061_3p2mm"}  # b1 back in acrylic
    with pytest.raises(ParamError):
        BuildConfig(frame_sheet="m3_washer")
    with pytest.raises(ParamError):
        BuildConfig(linkage="klann_lego", link_sheets=(("b9", "acrylic_3mm"),))
    assert materials.sheet("al5052_3p2mm").min_edge == pytest.approx(6.35)
    assert materials.sheet("acrylic_3mm").min_edge == 1.0
    assert materials.gap_options()[0] == 1.0


def test_klann_quads_default_to_phases_0_0_180_180():
    for key in ("klann_lego", "klann_patent", "klann_long_legs", "klann_high_step"):
        legs = BuildConfig(linkage=key, module="quad").legs
        assert [round(math.degrees(p)) for _, p in legs] == [0, 0, 180, 180], key
    for key in ("strider", "klann"):        # the generic quad (the demo Klann's too)
        assert [round(math.degrees(p)) for _, p in BuildConfig(linkage=key,
                                                               module="quad").legs] == \
            [0, 180, 90, 270]


def test_the_planner_searches_up_to_60_layers():
    assert StackSpec().max_top == 60


def _plate(w: float, h: float, t: float, holes: list[tuple[float, float, float]]):
    part = Box(w, h, t).moved(Location((w / 2, h / 2, t / 2)))
    for x, y, d in holes:
        part = part - Cylinder(d / 2, 3 * t).moved(Location((x, y, t / 2)))
    return part


def test_the_services_cut_rules():
    al = Body("frame", _plate(40, 20, 3.175, [(10, 10, 2.4), (35, 10, 6.0)]), fab="laser",
              sheet="al5052_3p2mm")
    rules = {i["rule"] for i in part_issues(al, "al5052_3p2mm")}
    assert rules == {"min_hole", "edge"}             # a 2.4 mm hole; 2 mm to the edge
    acrylic = replace(al, part=_plate(40, 20, 3.0, [(10, 10, 2.4), (35, 10, 6.0)]),
                      sheet=None)
    assert part_issues(acrylic, "acrylic_3mm") == []
    tiny = replace(al, part=_plate(5, 8, 3.175, []))
    assert {i["rule"] for i in part_issues(tiny, "al5052_3p2mm")} == {"min_part"}


def test_cut_rule_levels_and_messages():
    """The design review's levels (the user's rule of 2026-10-04): in metal a hole closer
    than 1 x the thickness to an edge is an error, under SendCutSend's 2 x a warning, a hole
    under the minimum an error; every issue says why and how to fix it, and the messages
    (the audit's, verify's, the design card's) name the rule, the worst part and the fix."""
    from spiderpig import manufacture

    t = 3.175
    near = Body("near", _plate(40, 20, t, [(4.0, 10, 4.0)]), fab="laser", sheet="al5052_3p2mm")
    (e,) = part_issues(near, "al5052_3p2mm")          # 2.0 mm of web: under 1 x t
    assert (e["rule"], e["level"]) == ("edge", "error")
    assert e["value"] == pytest.approx(2.0, abs=0.01)
    assert "1 x the thickness" in e["why"] and e["fix"]
    mid = Body("mid", _plate(40, 20, t, [(7.0, 10, 4.0)]), fab="laser", sheet="al5052_3p2mm")
    (w,) = part_issues(mid, "al5052_3p2mm")           # 5.0 mm: over 1 x t, under 2 x t
    assert (w["rule"], w["level"]) == ("edge", "warning")
    assert "2 x the thickness" in w["why"]
    issues = [dict(i, sheet="al5052_3p2mm") for i in (e, w)]
    m = {"parts": 2, "issues": issues, "errors": {"edge": 1},
         "sheets": {"al5052_3p2mm": materials.sheet("al5052_3p2mm").label}}
    (err,) = manufacture.messages(m, "error")
    assert err.startswith("manufacture: 1 part(s) break the hole-to-edge distance rule; "
                          "worst near (al5052_3p2mm)")
    assert "(fix: " in err
    (warn,) = manufacture.messages(m, "warning")
    assert "worst mid" in warn
    s = manufacture.summary(m)
    assert not s["ok"]
    assert (s["errors"], s["warnings"]) == ({"edge": 1}, {"edge": 1})
    assert s["messages"] == [err, warn]                # errors first
