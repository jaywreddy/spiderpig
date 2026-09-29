"""Tests for the printed crankshaft (:mod:`construction.crank`) with every servo.

(Inside its claims and clash-free for every module and servo: ``test_contract.py``.)
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import servos
from construction.base import Build, ConstructionError
from construction.crank import BHCS, NUT_H, Gap, PrintedCrank, _gaps
from fabricate import BuildConfig, design_side
from shapes import disc

SERVOS = servos.available()


def _volume(shape) -> float:
    return 0.0 if shape is None else sum(s.volume for s in shape.solids())


def _riders(design) -> dict[int, str]:
    return {design.plan.layers[b]: p for b, p in design.plan.topo.riders.items()}


# -- the parts ------------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["single", "double", "quad"])
def test_every_segment_is_one_valid_solid(design, side, mode):
    _, d = design(mode)
    mech = side(mode, 1.0)
    segs = [b for b in mech.bodies if b.name.startswith("crank_seg")]
    riders = {d.plan.layers[b] for b in d.plan.topo.riders}
    assert len(segs) == len(_gaps({k: "" for k in riders})) + 1
    for b in mech.bodies:
        if b.name.startswith("crank"):
            assert len(b.part.solids()) == 1, b.name
            assert b.part.is_valid, b.name
            assert b.rigid_with == d.plan.topo.crank_bodies[0]
            assert b.fab == ("printed" if b.name.startswith("crank_seg") else "purchased")
            if b.fab == "purchased":
                assert b.bom_key, b.name


@pytest.mark.parametrize("servo", SERVOS)
def test_horn_screws_land_in_the_horn_holes(design, side, servo):
    tmpl, d = design("single", servo)
    frozen = tmpl.freeze_at(1.0)
    mech = side("single", 1.0, servo)
    build = Build(d.ctx, d.plan, frozen)
    spec = servos.get(servo)
    pat = spec.horn.pattern
    iface = d.ctx.interfaces["drive"]
    o = build.xy("O")
    theta = d.drive.horn_angle(build)
    holes = [o + pat.pcd / 2 * np.array([math.cos(theta + 2 * math.pi * k / pat.count),
                                         math.sin(theta + 2 * math.pi * k / pat.count)])
             for k in range(pat.count)]
    plate_top = d.plan.z(d.plan.top)[1]
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
    top = mech.body(f"crank_seg{len(_gaps(_riders(d)))}").part.bounding_box()
    assert pytest.approx(plate_top - iface.horn_face_depth) == top.max.Z


@pytest.mark.parametrize("servo", SERVOS)
def test_the_hub_turns_clear_of_the_inner_plate(design, side, servo):
    tmpl, d = design("single", servo)
    mech = side("single", 1.0, servo)
    hub = mech.body(f"crank_seg{len(_gaps(_riders(d)))}").part
    plate_bottom = d.plan.z(d.plan.top)[0]
    margin = d.ctx.params.margin
    r = d.ctx.interfaces["drive"].horn_radius
    o = tuple(Build(d.ctx, d.plan, tmpl.freeze_at(1.0)).xy("O"))
    outside = disc(o, 60, plate_bottom - margin + 1e-3, plate_bottom + 3) - disc(
        o, r + 1e-3, plate_bottom - margin, plate_bottom + 4)
    assert _volume(hub & outside) < 1e-6


# -- the crankpin joints ----------------------------------------------------------------


def test_gaps_merge_adjacent_riders_of_one_pin():
    assert _gaps({2: "M", 3: "M", 6: "N"}) == [Gap(2, 3, "M"), Gap(6, 6, "N")]
    assert _gaps({2: "M", 4: "N"}) == [Gap(2, 2, "M"), Gap(4, 4, "N")]


@pytest.mark.parametrize("mode", ["single", "double", "quad"])
def test_crankpins_are_screwed_through(design, side, mode):
    _, d = design(mode)
    mech = side(mode, 1.0)
    gaps = _gaps(_riders(d))
    crank = PrintedCrank()
    for g in gaps:
        screw = mech.body(f"crank_screw_{g.pin}")
        nut = mech.body(f"crank_nut_{g.pin}")
        assert nut.bom_key == "m3_nut"
        assert screw.bom_key.startswith("m3_bhcs_")      # an ISO 4762 head won't fit 3 mm webs
        sb, nb = screw.part.bounding_box(), nut.part.bounding_box()
        # head under the lower web, tip in the nut, both inside the webs either side
        assert d.plan.z(g.lo - 1)[0] - 1e-6 <= sb.min.Z
        assert d.plan.z(g.hi + 1)[1] + 1e-6 >= sb.max.Z
        assert d.plan.z(g.hi + 1)[1] + 1e-6 >= nb.max.Z
        assert crank.min_nut_engage - 1e-6 <= sb.max.Z - nb.min.Z
        # the post carries b1 with end play
        post_len = (g.hi - g.lo + 1) * d.ctx.pitch + crank.axial_play
        assert post_len > (g.hi - g.lo + 1) * d.ctx.pitch
    if mode == "double":                               # both legs on one post: one joint
        assert len(gaps) == 1
        assert gaps[0].hi == gaps[0].lo + 1
        assert mech.body(f"crank_screw_{gaps[0].pin}").bom_key == BHCS["3"].key(10)


def test_b1_has_end_play(design, side):
    tmpl, d = design("single")
    mech = side("single", 1.0)
    (g,) = _gaps(_riders(d))
    b1_bottom = d.plan.z(g.lo)[0]
    play = PrintedCrank().axial_play
    xy = tuple(Build(d.ctx, d.plan, tmpl.freeze_at(1.0)).xy(g.pin))
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


def test_too_thin_sheet_is_refused(design):
    with pytest.raises(ConstructionError, match="crankpin joint"):
        design_side(design("single")[0], BuildConfig(robot=False, thickness=2.0))
