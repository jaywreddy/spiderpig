"""Tests for the printed crankshaft (:mod:`construction.crank`) with every servo."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

import linkage
import servos
from config import BuildConfig
from construction.base import FRAME_OUTER, Build, ConstructionError, Realized
from construction.contract import check_side, clashes
from construction.crank import (
    BHCS,
    NUT_H,
    CrankRoute,
    PrintedCrank,
    Run,
    default_route,
    route_of,
)
from fabricate import (
    SideDesign,
    design_side,
    fabricate_side,
    side_problem,
    template_for,
)
from hardware.bom import bom_from_mechanism
from servos import cad as cadlib
from servos import model
from shapes import disc
from stack import Axis, Layout, verify_plan

TEMPLATES = {
    "single": lambda: linkage.build_module_template("single"),
    "double": lambda: linkage.build_module_template("double"),
    "decker": lambda: linkage.build_module_template("decker"),
    "quad": lambda: linkage.build_module_template("quad"),
}
SERVOS = servos.available()
OTHERS = [s for s in SERVOS if s != servos.DEFAULT]


@pytest.fixture(scope="module", autouse=True)
def _offline(tmp_path_factory):
    """No downloads in tests: servos are drawn parametrically."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv(cadlib.OFFLINE_ENV, "1")
        mp.setenv(cadlib.CACHE_ENV, str(tmp_path_factory.mktemp("cad")))
        for f in (model.cad_servo, model._servo_part, cadlib._load_cached):
            f.cache_clear()
        yield
        for f in (model.cad_servo, model._servo_part, cadlib._load_cached):
            f.cache_clear()


@pytest.fixture(scope="module")
def templates():
    return {k: f() for k, f in TEMPLATES.items()}


def _design(tmpl, servo=servos.DEFAULT):
    return design_side(tmpl, BuildConfig(robot=False, servo=servo))


def _clashes(mech) -> list[tuple[str, str, float]]:
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in itertools.combinations(parts, 2):
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > 1e-3:
            out.append((a, b, round(vol, 4)))
    return out


def _volume(shape) -> float:
    return 0.0 if shape is None else sum(s.volume for s in shape.solids())


# -- the contract ---------------------------------------------------------------------


@pytest.mark.parametrize("mode", sorted(TEMPLATES))
@pytest.mark.parametrize("t", [0.0, 2.2, 4.38])
def test_crank_stays_inside_its_claims(templates, mode, t):
    tmpl = templates[mode]
    assert check_side(_design(tmpl), tmpl.freeze_at(t)) == []


@pytest.mark.parametrize("mode", sorted(TEMPLATES))
@pytest.mark.parametrize("servo", OTHERS)
def test_every_servo_couples_inside_the_claims(templates, servo, mode):
    tmpl = templates[mode]
    assert check_side(_design(tmpl, servo), tmpl.freeze_at(2.2)) == []


# -- the parts ------------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["single", "double", "quad"])
def test_every_segment_is_one_valid_solid(templates, mode):
    tmpl = templates[mode]
    design = _design(tmpl)
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    segs = [b for b in mech.bodies if b.name.startswith("crank_seg")]
    assert len(segs) == len(_runs(design)) + 1
    for b in mech.bodies:
        if b.name.startswith("crank"):
            assert len(b.part.solids()) == 1, b.name
            assert b.part.is_valid, b.name
            assert b.rigid_with == design.plan.topo.crank_bodies[0]
            assert b.fab == ("printed" if b.name.startswith("crank_seg") else "purchased")
            if b.fab == "purchased":
                assert b.bom_key, b.name


@pytest.mark.parametrize("servo", SERVOS)
@pytest.mark.parametrize("t", [1.0, 4.38])
def test_single_side_parts_do_not_intersect(templates, servo, t):
    """Every body, screws and nuts included."""
    tmpl = templates["single"]
    mech = fabricate_side(_design(tmpl, servo), tmpl.freeze_at(t))
    assert any(b.name.startswith("servo_screw") for b in mech.bodies)
    assert any(b.name.startswith("crank_nut") for b in mech.bodies)
    assert _clashes(mech) == []


@pytest.mark.parametrize("servo", SERVOS)
def test_horn_screws_land_in_the_horn_holes(templates, servo):
    tmpl = templates["single"]
    design = _design(tmpl, servo)
    frozen = tmpl.freeze_at(1.0)
    mech = fabricate_side(design, frozen)
    build = Build(design.ctx, design.plan, frozen)
    spec = servos.get(servo)
    pat = spec.horn.pattern
    iface = design.ctx.interfaces["drive"]
    o = build.xy("O")
    theta = design.drive.horn_angle(build)
    holes = [o + pat.pcd / 2 * np.array([math.cos(theta + 2 * math.pi * k / pat.count),
                                         math.sin(theta + 2 * math.pi * k / pat.count)])
             for k in range(pat.count)]
    plate_top = design.plan.z(design.plan.top)[1]
    horn_face = plate_top - spec.horn_face_depth
    horn = mech.body("servo_horn").part
    screws = [b for b in mech.bodies if b.name.startswith("crank_horn_screw")]
    assert len(screws) == pat.count
    used = set()
    for b in screws:
        bb = b.part.bounding_box()
        c = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2])
        k = min(range(len(holes)), key=lambda i: np.linalg.norm(holes[i] - c))
        assert np.linalg.norm(holes[k] - c) < 1e-3
        used.add(k)
        engage = bb.max.Z - horn_face
        assert engage >= min(pat.thread_depth, 1.5) - 1e-6
        assert engage <= min(pat.reach, spec.horn_face_depth) + 1e-6
        assert plate_top + 1e-6 >= bb.max.Z
        # the shank is in the hole, not in the horn's metal
        hole = disc(tuple(holes[k]), pat.thread_d / 2, horn_face, bb.max.Z)
        assert _volume(b.part & hole) > 0.5 * math.pi * (0.97 * pat.thread_d / 2) ** 2 * engage
        assert _volume(b.part & horn) < 1e-3
        assert b.bom_key.startswith("m" + pat.thread[1:].replace(".", "p") + "_")
    assert used == set(range(pat.count))
    # the hub bolts on at the coupling face and never rises above it
    top = mech.body(f"crank_seg{len(_runs(design))}").part.bounding_box()
    assert pytest.approx(plate_top - iface.horn_face_depth) == top.max.Z


def _runs(design) -> tuple[Run, ...]:
    return route_of(design.plan.layout, design.plan.topo.axes_of("crankpin")).runs


@pytest.mark.parametrize("servo", SERVOS)
def test_the_hub_turns_clear_of_the_inner_plate(templates, servo):
    tmpl = templates["single"]
    design = _design(tmpl, servo)
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    hub = mech.body(f"crank_seg{len(_runs(design))}").part
    plate_bottom = design.plan.z(design.plan.top)[0]
    margin = design.ctx.params.margin
    r = design.ctx.interfaces["drive"].horn_radius
    o = tuple(Build(design.ctx, design.plan, tmpl.freeze_at(1.0)).xy("O"))
    outside = disc(o, 60, plate_bottom - margin + 1e-3, plate_bottom + 3) - disc(
        o, r + 1e-3, plate_bottom - margin, plate_bottom + 4)
    assert _volume(hub & outside) < 1e-6


# -- the crankpin joints ----------------------------------------------------------------


def test_default_route_merges_adjacent_riders_of_one_pin():
    pins = [Axis("M", "crankpin", ("a", "b"), ()), Axis("N", "crankpin", ("c",), ())]
    route = default_route(Layout({"a": 2, "b": 3, "c": 6}, 8, 3.0), pins)
    assert route.runs == (Run("M", 2, 3), Run("N", 6, 6))
    assert route.bearing
    route = default_route(Layout({"a": 2, "b": 5, "c": 3}, 8, 3.0), pins)
    assert route.runs == (Run("M", 2, 2), Run("M", 5, 5), Run("N", 3, 3))


@pytest.mark.parametrize("mode", ["single", "double", "quad"])
def test_crankpins_are_screwed_through(templates, mode):
    tmpl = templates[mode]
    design = _design(tmpl)
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    gaps = _runs(design)
    crank = PrintedCrank()
    for g in gaps:
        screw = mech.body(f"crank_screw_{g.at}")
        nut = mech.body(f"crank_nut_{g.at}")
        assert nut.bom_key == "m3_nut"
        assert screw.bom_key.startswith("m3_bhcs_")      # an ISO 4762 head won't fit 3 mm webs
        sb, nb = screw.part.bounding_box(), nut.part.bounding_box()
        # head under the lower web, tip in the nut, both inside the webs either side
        assert design.plan.z(g.lo - 1)[0] - 1e-6 <= sb.min.Z
        assert design.plan.z(g.hi + 1)[1] + 1e-6 >= sb.max.Z
        assert design.plan.z(g.hi + 1)[1] + 1e-6 >= nb.max.Z
        assert crank.min_nut_engage - 1e-6 <= sb.max.Z - nb.min.Z
        # the post carries b1 with end play
        post_len = (g.hi - g.lo + 1) * design.ctx.pitch + crank.axial_play
        assert post_len > (g.hi - g.lo + 1) * design.ctx.pitch
    if mode == "double":                               # both legs on one post: one joint
        assert len(gaps) == 1
        assert gaps[0].hi == gaps[0].lo + 1
        assert mech.body(f"crank_screw_{gaps[0].at}").bom_key == BHCS["3"].key(10)


def test_b1_has_end_play(templates):
    tmpl = templates["single"]
    design = _design(tmpl)
    frozen = tmpl.freeze_at(1.0)
    mech = fabricate_side(design, frozen)
    (g,) = _runs(design)
    b1_bottom = design.plan.z(g.lo)[0]
    play = PrintedCrank().axial_play
    xy = tuple(Build(design.ctx, design.plan, frozen).xy(g.at))
    lower = mech.body("crank_seg0").part
    slab = disc(xy, 100, b1_bottom - play + 1e-3, b1_bottom) - disc(xy, 3.2, b1_bottom - 1,
                                                                     b1_bottom + 1)
    assert _volume(lower & slab) < 1e-6                # nothing but the post near b1


def test_post_joint_lengths():
    crank = PrintedCrank()
    one = crank.post_joint(3.0, 5.85, 9.0, 12.0)       # one b1 between 3 mm webs
    assert one.screw is BHCS["3"]
    assert one.length == 6
    assert one.engagement >= crank.min_nut_engage
    two = crank.post_joint(3.0, 5.85, 12.0, 15.0)      # two b1s on one post
    assert two.length == 10
    assert two.engagement == pytest.approx(NUT_H)
    thick = crank.post_joint(4.5, 8.85, 13.5, 18.0)    # 4.5 mm sheet: a socket head fits
    assert thick.screw.kind == "shcs"


def test_too_thin_sheet_is_refused(templates):
    with pytest.raises(ConstructionError, match="crankpin joint"):
        design_side(templates["single"], BuildConfig(robot=False, thickness=2.0))


# -- crank routes: hand-made plans of what the planner may choose ---------------------------

KLANN = BuildConfig(robot=False, module="single")
TROTBOT = BuildConfig(robot=False, module="single", linkage="trotbot")
TROT_LAYERS = {"b4": 2, "b3": 3, "b5": 4, "b2": 5, "b1": 6, "b6": 7}
ROUTES = {  # config, link layers, top, route, detour point, screw length per joint
    # b1's run goes on over an empty layer above it (below it)
    "past b1": (KLANN, {"b1": 2, "b3": 4, "b4": 4, "b2": 5}, 7,
                CrankRoute((Run("M", 2, 3),)), None, {"M": 10}),
    "under b1": (KLANN, {"b1": 3, "b3": 4, "b4": 4, "b2": 5}, 7,
                 CrankRoute((Run("M", 2, 3),)), None, {"M": 10}),
    # a detour opposite the crankpin in an empty layer; its lower web shares layer 3 with
    # the upper web of b1's run
    "detour": (KLANN, {"b1": 2, "b3": 3, "b4": 3, "b2": 5}, 7,
               CrankRoute((Run("M", 2, 2), Run("X", 4, 4))), ("X", 14.0, 180.0),
               {"M": 6, "X": 6}),
    "no bearing": (KLANN, {"b1": 2, "b3": 3, "b4": 3, "b2": 4}, 6,
                   CrankRoute((Run("M", 2, 2),), bearing=False), None, {"M": 6}),
    # TrotBot's B8 (b6) sweeps across O: it sits in the crankpin's run beside the post (with
    # pin J8's head and cap, which would sweep across a web), b4 at the run's foot
    "trotbot": (TROTBOT, TROT_LAYERS, 11, CrankRoute((Run("J1", 2, 8),)), None, {"J1": 25}),
    # ... or b4 on a run of its own, its upper web on the next run's lower web: one screw
    "trotbot chain": (TROTBOT, TROT_LAYERS, 11,
                      CrankRoute((Run("J1", 2, 2), Run("J1", 5, 8))), None, {"J1": 25}),
}
_ROUTED: dict[str, tuple] = {}


def _routed(config, layers, top, route, point=None):
    """One side planned by hand along ``route`` (``point``: a detour's (name, r, degrees))."""
    tmpl = template_for(config)
    ctx, groups, problem = side_problem(tmpl, config)
    if point is not None:
        ctx.topo.add_crank_point(*point)
    plan = problem.plan(layers, top, {"crank": route})
    assert verify_plan(plan) == []
    return tmpl, SideDesign(config, ctx, groups, plan)


def _route_case(name: str):
    """(template, design, the side fabricated at t = 4.38) for a case of :data:`ROUTES`."""
    if name not in _ROUTED:
        config, layers, top, route, point, _ = ROUTES[name]
        tmpl, design = _routed(config, layers, top, route, point)
        _ROUTED[name] = tmpl, design, fabricate_side(design, tmpl.freeze_at(4.38))
    return _ROUTED[name]


def _part(mech, name: str):
    return next(b.part for b in mech.bodies if b.name == name)


@pytest.mark.parametrize("case", sorted(ROUTES))
def test_every_route_builds_inside_its_claims(case):
    tmpl, design, mech = _route_case(case)
    assert check_side(design, tmpl.freeze_at(2.2)) == []
    assert clashes(mech) == []
    crank = [b for b in mech.bodies if b.name.startswith("crank")]
    for b in crank:
        assert len(b.part.solids()) == 1, b.name
        assert b.part.is_valid, b.name
    runs, lengths = ROUTES[case][3].runs, ROUTES[case][-1]
    assert len([b for b in crank if b.name.startswith("crank_seg")]) == len(runs) + 1
    # one joint per point (a chain of runs along one point shares its screw)
    screws = {b.name: b.bom_key for b in crank if b.name.startswith("crank_screw")}
    assert screws == {f"crank_screw_{at}": f"m3_bhcs_{n}" for at, n in lengths.items()}
    nuts = {b.name: b.bom_key for b in crank if b.name.startswith("crank_nut")}
    assert nuts == {f"crank_nut_{at}": "m3_nut" for at in lengths}
    bought = {r.key for r in bom_from_mechanism(mech).purchased}
    assert {*screws.values(), *nuts.values()} <= bought


@pytest.mark.parametrize(("case", "rider", "bare"), [("past b1", 2, 3), ("under b1", 3, 2)])
def test_a_run_past_its_rider_is_bare_with_end_play_at_the_rider_only(case, rider, bare):
    tmpl, design, mech = _route_case(case)
    z, play = design.plan.z, PrintedCrank().axial_play
    xy = tuple(Build(design.ctx, design.plan, tmpl.freeze_at(4.38)).xy("M"))
    lower, upper = _part(mech, "crank_seg0"), _part(mech, "crank_seg1")
    # the post runs bare through the layer b1 isn't in
    post = disc(xy, 3.01, *z(bare))
    assert _volume(lower & (disc(xy, 100, *z(bare)) - post)) < 1e-6
    assert _volume(lower & post) > 0.9 * math.pi * (3**2 - 1.7**2) * design.ctx.pitch
    # end play on b1's side only: the web above the run is set back if b1 is under it,
    # the web below if b1 is over it
    assert pytest.approx(z(4)[0] + (play if rider == 3 else 0)) == upper.bounding_box().min.Z
    under = disc(xy, 100, z(2)[0] - play + 1e-3, z(2)[0] - 1e-3) - disc(xy, 3.01, 0, 100)
    assert (_volume(lower & under) > 1.0) == (rider == 3)


def test_a_crank_point_turns_with_the_crank():
    tmpl, design, _ = _route_case("detour")
    for t in (0.0, 1.0, 4.38):
        build = Build(design.ctx, design.plan, tmpl.freeze_at(t))
        x, m = build.xy("X") - build.xy("O"), build.xy("M") - build.xy("O")
        assert np.linalg.norm(x) == pytest.approx(14.0)
        assert np.dot(x, m) / np.linalg.norm(m) == pytest.approx(-14.0)   # opposite M


def test_without_the_bearing_the_crank_ends_at_its_lowest_web():
    tmpl, design, mech = _route_case("no bearing")
    bottom = _part(mech, "crank_seg0").bounding_box().min.Z
    assert pytest.approx(design.plan.z(1)[0]) == bottom   # the web's face, no journal stub
    crank = next(g for g in design.groups if g.name == "crank")
    got = crank.realize(Build(design.ctx, design.plan, tmpl.freeze_at(4.38)), Realized())
    assert FRAME_OUTER not in got.cuts                 # no journal hole in the outer plate


def test_a_joint_no_stock_screw_fits_is_reported():
    """TrotBot's shortest run (b4, J8's head, b1, b6, J8's cap): webs 21 mm apart."""
    layers = {"b4": 2, "b2": 3, "b1": 4, "b5": 4, "b6": 5, "b3": 5}
    tmpl, design = _routed(TROTBOT, layers, 9, CrankRoute((Run("J1", 2, 6),)))
    with pytest.raises(ConstructionError, match=r"no stock screw fits .* J1: .* 21\.00 mm"):
        fabricate_side(design, tmpl.freeze_at(1.0))


def test_pockets_that_meet_are_reported():
    """A detour 4 mm from the crankpin: X's head pocket would cut M's nut out of layer 3."""
    config, layers, top, route, _, _ = ROUTES["detour"]
    tmpl, design = _routed(config, layers, top, route, ("X", 22.0, 8.0))
    with pytest.raises(ConstructionError, match="pockets for M and X"):
        fabricate_side(design, tmpl.freeze_at(1.0))
