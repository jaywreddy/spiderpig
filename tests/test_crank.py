"""Tests for the printed crankshaft (:mod:`construction.crank`) with every servo."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

import servos
from construction.base import Build, ConstructionError
from construction.contract import check_side
from construction.crank import BHCS, NUT_H, Gap, PrintedCrank, _gaps
from fabricate import BuildConfig, design_side, fabricate_side
from klann import (
    build_double_decker_template,
    build_double_double_decker_template,
    build_double_template,
    build_klann_template,
    create_klann_geometry,
)
from servos import cad as cadlib
from servos import model
from shapes import disc

TEMPLATES = {
    "single": lambda: build_klann_template(create_klann_geometry()),
    "double": build_double_template,
    "decker": build_double_decker_template,
    "quad": build_double_double_decker_template,
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
    riders = {design.plan.layers[b] for b in design.plan.topo.riders}
    assert len(segs) == len(_gaps({k: "" for k in riders})) + 1
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
    top = mech.body(f"crank_seg{len(_gaps(_riders(design)))}").part.bounding_box()
    assert pytest.approx(plate_top - iface.horn_face_depth) == top.max.Z


def _riders(design) -> dict[int, str]:
    return {design.plan.layers[b]: p for b, p in design.plan.topo.riders.items()}


@pytest.mark.parametrize("servo", SERVOS)
def test_the_hub_turns_clear_of_the_inner_plate(templates, servo):
    tmpl = templates["single"]
    design = _design(tmpl, servo)
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    hub = mech.body(f"crank_seg{len(_gaps(_riders(design)))}").part
    plate_bottom = design.plan.z(design.plan.top)[0]
    margin = design.ctx.params.margin
    r = design.ctx.interfaces["drive"].horn_radius
    o = tuple(Build(design.ctx, design.plan, tmpl.freeze_at(1.0)).xy("O"))
    outside = disc(o, 60, plate_bottom - margin + 1e-3, plate_bottom + 3) - disc(
        o, r + 1e-3, plate_bottom - margin, plate_bottom + 4)
    assert _volume(hub & outside) < 1e-6


# -- the crankpin joints ----------------------------------------------------------------


def test_gaps_merge_adjacent_riders_of_one_pin():
    assert _gaps({2: "M", 3: "M", 6: "N"}) == [Gap(2, 3, "M"), Gap(6, 6, "N")]
    assert _gaps({2: "M", 4: "N"}) == [Gap(2, 2, "M"), Gap(4, 4, "N")]


@pytest.mark.parametrize("mode", ["single", "double", "quad"])
def test_crankpins_are_screwed_through(templates, mode):
    tmpl = templates[mode]
    design = _design(tmpl)
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    gaps = _gaps(_riders(design))
    crank = PrintedCrank()
    for g in gaps:
        screw = mech.body(f"crank_screw_{g.pin}")
        nut = mech.body(f"crank_nut_{g.pin}")
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
        assert mech.body(f"crank_screw_{gaps[0].pin}").bom_key == BHCS["3"].key(10)


def test_b1_has_end_play(templates):
    tmpl = templates["single"]
    design = _design(tmpl)
    frozen = tmpl.freeze_at(1.0)
    mech = fabricate_side(design, frozen)
    (g,) = _gaps(_riders(design))
    b1_bottom = design.plan.z(g.lo)[0]
    play = PrintedCrank().axial_play
    xy = tuple(Build(design.ctx, design.plan, frozen).xy(g.pin))
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
