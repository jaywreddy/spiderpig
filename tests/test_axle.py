"""Tests for the printed stepped axle (:mod:`construction.axle`, :mod:`construction.printed`).

Every pillar and pin is printed in segments that snap together: do they stay
inside their claims, go together, thread every link and hold?
"""

from __future__ import annotations

import itertools
import math
from dataclasses import replace

import pytest
from build123d import Box, Location

from construction import ConstructionError
from construction.axle import AxleGroup, PrintedAxle
from construction.base import Build
from construction.contract import check_side
from construction.printed import Snap, plan_segments, segment_solid
from fabricate import BuildConfig, design_side, fabricate_side
from klann import (
    build_double_decker_template,
    build_double_double_decker_template,
    build_double_template,
    build_klann_template,
    create_klann_geometry,
)
from shapes import disc

TEMPLATES = {
    "single": lambda: build_klann_template(create_klann_geometry()),
    "double": build_double_template,
    "decker": build_double_decker_template,
    "quad": build_double_double_decker_template,
}
SIDE = BuildConfig(robot=False)
T = 1.0


def _vol(a, b) -> float:
    inter = a & b
    return 0.0 if inter is None else sum(s.volume for s in inter.solids())


def _above(part, z: float):
    """The part of ``part`` above ``z``."""
    return part & Box(1e3, 1e3, 1e3).moved(Location((0.0, 0.0, z + 500.0)))


@pytest.fixture(scope="module", params=sorted(TEMPLATES))
def side(request):
    tmpl = TEMPLATES[request.param]()
    design = design_side(tmpl, SIDE)
    mech = tmpl.freeze_at(T)
    build = Build(design.ctx, design.plan, mech)
    fab = fabricate_side(design, mech)
    axles = [g for g in design.groups if isinstance(g, AxleGroup)]
    return tmpl, design, build, fab, axles


def _segments(fab, group):
    base = group.name.replace(":", "_")
    bodies = sorted((b for b in fab.bodies if b.name.startswith(base + "_seg")),
                    key=lambda b: int(b.name.rsplit("seg", 1)[1]))
    assert [b.name for b in bodies] == [f"{base}_seg{i}" for i in range(len(bodies))]
    return bodies


# -- the contract ---------------------------------------------------------------


@pytest.mark.parametrize("t", [0.0, 4.38])
def test_axles_stay_inside_their_claims(side, t):
    tmpl, design, *_ = side
    assert check_side(design, tmpl.freeze_at(t)) == []


# -- segments -------------------------------------------------------------------


def test_every_segment_is_one_valid_printed_solid(side):
    _, design, build, fab, axles = side
    frame = design.plan.topo.frame_bodies[0]
    for g in axles:
        segs = _segments(fab, g)
        assert len(segs) >= 2, g.name        # at least one joint: every axle carries a link
        lowest = min(g.axis.members, key=lambda m: (build.layers[m], m))
        for b in segs:
            assert len(b.part.solids()) == 1, b.name
            assert b.part.is_valid, b.name
            assert b.fab == "printed", b.name
            assert b.rigid_with == (frame if g.pillar else lowest), b.name


def test_segments_touch_but_never_overlap(side):
    *_, fab, axles = side
    for g in axles:
        segs = [b.part for b in _segments(fab, g)]
        for a, b in itertools.combinations(segs, 2):
            assert _vol(a, b) < 1e-3, g.name
        for lower, upper in itertools.pairwise(segs):
            assert lower.distance_to(upper) < 1e-6, g.name


def test_every_link_is_threaded_by_exactly_one_bearing(side):
    _, design, build, fab, axles = side
    p = design.ctx.params
    for g in axles:
        segs = _segments(fab, g)
        planned = g.construction.segments(g, build)
        xy = build.xy(g.axis.name)
        for m in g.axis.members:
            hole = disc(xy, p.hole(p.axle_d) / 2, *build.z(build.layers[m]))
            threads = [i for i, b in enumerate(segs) if _vol(hole, b.part) > 1e-3]
            assert len(threads) == 1, (g.name, m, threads)
            assert m in planned[threads[0]].links
            # the bearing fills the hole but for the running fit and (at most) the slot
            r, w = p.axle_d / 2, g.construction.slot_width
            bearing = _vol(hole, segs[threads[0]].part)
            assert bearing > (math.pi * r**2 - 2 * r * w) * design.ctx.pitch, (g.name, m)


def test_snap_pegs_fit_their_sockets_and_hold(side):
    _, design, build, fab, axles = side
    for g in axles:
        snap = g.construction.snap(design.ctx)
        c = snap.clearance
        assert c == pytest.approx(design.ctx.params.print_fit / 2)
        for lower, upper in itertools.pairwise(b.part for b in _segments(fab, g)):
            split, tip = upper.bounding_box().min.Z, lower.bounding_box().max.Z
            assert tip == pytest.approx(split + snap.height)
            peg = _above(lower, split + 1e-3)
            assert peg.distance_to(upper) == pytest.approx(c, abs=1e-3), g.name
            # the barb catches the ledge once pulled more than the clearance apart
            assert _vol(lower, upper.moved(Location((0.0, 0.0, c - 0.02)))) < 1e-3, g.name
            assert _vol(lower, upper.moved(Location((0.0, 0.0, c + 0.05)))) > 1e-3, g.name
        # and the prongs can close far enough to push it on
        assert snap.deflection() < snap.slot / 2
        assert snap.barb > snap.throat > snap.shank


def test_axle_ends(side):
    _, design, build, fab, axles = side
    plan = design.plan
    for g in axles:
        segs = _segments(fab, g)
        labels = {s.layer: s.label.rsplit(" ", 1)[1] for s in build.shapes(g.name)}
        bottom, top = segs[0].part.bounding_box().min.Z, segs[-1].part.bounding_box().max.Z
        assert bottom == pytest.approx(plan.z(min(labels))[0])
        assert top == pytest.approx(plan.z(max(labels))[1])
        if g.pillar and labels[plan.top] == "anchor":
            # flush with the inner plate's top face: the servo stands there
            assert top == pytest.approx(plan.z(plan.top)[1])
        if g.pillar and labels[0] == "anchor":
            assert labels[-1] == "head"          # held under the outer plate too


def test_holes_and_glue(side):
    _, design, build, fab, axles = side
    p = design.ctx.params
    for g in axles:
        got = g.realize(build)
        anchors = [s.layer for s in build.shapes(g.name) if s.label.endswith("anchor")]
        plates = {0: "frame:outer", build.top: "frame:inner"}
        assert set(got.cuts) == set(g.axis.members) | {plates[k] for k in anchors}
        for k in anchors:
            assert [c.d for c in got.cuts[plates[k]]] == [pytest.approx(p.hole(p.axle_d, "glue"))]
        for m in g.axis.members:
            assert [c.d for c in got.cuts[m]] == [pytest.approx(p.hole(p.axle_d))]
        if anchors:
            assert {e.key for e in got.extras} == {"ca_glue"}
        else:
            assert got.extras == []


def test_snap_prongs_are_not_overstrained(side):
    _, design, build, _, axles = side
    for g in axles:
        snap = g.construction.snap(design.ctx)
        for seg in g.construction.segments(g, build):
            if seg.peg is None:
                continue
            strain = seg.strain(snap)
            assert 0 < strain < 0.065, (g.name, seg.index, strain)
            if not g.pillar and len(seg.links) == 2:         # a pin through two links
                assert strain < 0.025, (g.name, seg.index, strain)


def _is_flat_bottom(part) -> bool:
    z0 = part.bounding_box().min.Z
    return any(abs(f.center().Z - z0) < 1e-6 and abs(abs(f.normal_at().Z) - 1) < 1e-6
               and f.area > 10.0 for f in part.faces())


def test_segments_stand_on_a_flat_bottom(side):
    *_, fab, axles = side
    for g in axles:
        for b in _segments(fab, g):
            assert _is_flat_bottom(b.part), b.name


# -- a single side: nothing clashes -----------------------------------------------


@pytest.mark.parametrize("t", [0.0, 2.2])
def test_single_side_bodies_do_not_intersect(t):
    tmpl = TEMPLATES["single"]()
    design = design_side(tmpl, SIDE)
    mech = fabricate_side(design, tmpl.freeze_at(t))
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    assert any(n.startswith("pin_") for n in parts)
    assert any(n.startswith("pillar_") for n in parts)
    clashes = []
    for a, b in itertools.combinations(parts, 2):
        vol = _vol(parts[a], parts[b])
        if vol > 1e-3:
            clashes.append((a, b, round(vol, 4)))
    assert clashes == []


# -- planning in isolation --------------------------------------------------------

PITCH = 3.0


def _z(k: int) -> tuple[float, float]:
    return k * PITCH, (k + 1) * PITCH


def _snap() -> Snap:
    return Snap(barb=3.0, engage=0.25, clearance=0.15, shank_h=1.0, land_h=0.4, flats=1.5,
                slot=1.2)


def _plan(column, links):
    return plan_segments(column, _z, links, axle=3.0, snap=_snap(), play=0.1, bridge=0.8,
                         base=2.0, slot_max=12.0)


def test_a_link_under_the_inner_plate_needs_no_split():
    """The plate retains the link; the bearing carries on through it as the anchor."""
    column = {-1: ("head", 4.25), 0: ("anchor", 3.0), 1: ("neck", 3.0), 2: ("shoulder", 4.25),
              3: ("axle", 3.0), 4: ("shoulder", 4.25), 5: ("axle", 3.0), 6: ("anchor", 3.0)}
    segs = _plan(column, {3: ("lo",), 5: ("hi",)})
    assert [s.links for s in segs] == [("lo",), ("hi",)]
    assert segs[0].peg == segs[1].socket == pytest.approx(4 * PITCH + 0.1)
    assert segs[1].peg is None
    assert segs[1].z1 == pytest.approx(7 * PITCH)
    assert [s.anchors for s in segs] == [(0,), (6,)]
    assert segs[0].slot_root == pytest.approx(PITCH)       # never into the glued anchor
    for s in segs:
        part = segment_solid(s, _snap(), (10.0, -5.0))
        assert part.is_valid
        assert len(part.solids()) == 1


def test_pin_splits_above_each_run_of_links():
    column = {1: ("head", 4.25), 2: ("axle", 3.0), 3: ("axle", 3.0), 4: ("shoulder", 4.25),
              5: ("neck", 2.5), 6: ("shoulder", 4.25), 7: ("axle", 3.0), 8: ("cap", 4.25)}
    segs = _plan(column, {2: ("a",), 3: ("b",), 7: ("c",)})
    assert [s.links for s in segs] == [("a", "b"), ("c",), ()]
    assert [s.peg for s in segs] == [pytest.approx(12.1), pytest.approx(24.1), None]
    # a stiff base under each slot: 2 mm of the head, the whole layer holding a socket
    assert [s.slot_root for s in segs] == [pytest.approx(5.0), pytest.approx(15.0), None]
    # ... and never through a neck too thin to split
    thin = _plan({**column, 5: ("neck", 1.3)}, {2: ("a",), 3: ("b",), 7: ("c",)})
    assert thin[1].slot_root == pytest.approx(18.0)
    # the shoulders stop short of the links they hold
    shoulders = [p for s in segs for p in s.pieces if p.role == "shoulder"]
    assert [(p.z0, p.z1) for p in shoulders] == [pytest.approx((12.1, 15.0)),
                                                 pytest.approx((18.0, 20.9))]


def test_snap_that_cannot_fit_is_refused():
    ctx = design_side(TEMPLATES["single"](), SIDE).ctx
    with pytest.raises(ConstructionError):
        PrintedAxle(slot_width=4.5).dims(ctx, False)          # prongs too thin
    with pytest.raises(ConstructionError):
        PrintedAxle(slot_width=0.5).dims(ctx, False)          # prongs can't close enough
    with pytest.raises(ConstructionError):
        PrintedAxle().dims(replace(ctx, pitch=2.0), True)     # socket taller than a shoulder
    PrintedAxle().dims(ctx, True)
