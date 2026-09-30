"""Tests for the robot assembly (:mod:`construction.robot`) and the hardware catalog."""

from __future__ import annotations

import math

import pytest
from build123d import Location

from spiderpig import servos
from spiderpig.construction.base import FRAME_INNER, Build, Realized
from spiderpig.construction.chassis import (
    MIN_ENGAGE,
    ServoFrame,
    centre_plates,
    servo_frame,
    tie_dims,
)
from spiderpig.construction.contract import bad_solids, clashes
from spiderpig.construction.robot import FrameTies
from spiderpig.hardware import fasteners
from spiderpig.hardware.catalog import CATALOG, _load, get
from spiderpig.servos.model import UNKNOWN_HOLE_DEPTH
from spiderpig.shapes import disc

TS = (1.0, 4.38)


def _placed(mech):
    return {b.name: b.placed_part() for b in mech.bodies if b.part is not None}


def _volume(shape) -> float:
    return 0.0 if shape is None else sum(s.volume for s in shape.solids())


@pytest.mark.parametrize("t", TS)
def test_no_two_parts_of_the_robot_intersect(robot, t):
    assert clashes(robot("single", t)) == []


def test_rear_screws_engage_their_pilots_without_bottoming_out(robot):
    """The servo model's pilots are drilled at screw size, so a screw that fits its
    hole doesn't intersect the case; it must reach in far enough and no further."""
    mech = robot("single", TS[0])
    parts_ = _placed(mech)
    spec = servos.get(mech.meta["servo"])
    depth = min(h.depth if h.depth is not None else UNKNOWN_HOLE_DEPTH for h in spec.rear_mount)
    engage = mech.meta["rear_engagement_mm"]
    assert MIN_ENGAGE <= engage <= depth
    screws = [(s, h) for s, h in mech.meta["fastened"] if "rear_screw" in s]
    assert screws
    for screw, host in screws:
        assert _volume(parts_[screw] & parts_[host]) < 1e-3, screw


def test_every_robot_part_is_one_valid_solid(robot):
    assert bad_solids(robot("single", TS[0])) == []


def _frames(design, mech) -> dict[str, ServoFrame]:
    build = Build(design.ctx, design.plan, mech)
    left = servo_frame(build, design.drive)
    return {"L": left, "R": ServoFrame(left.o, left.u, hand=-1)}


def test_rear_screws_sit_on_the_servo_pilots(design, robot):
    tmpl, d = design("single")
    mech = robot("single", TS[0])
    spec = d.ctx.servo
    frames = _frames(d, tmpl.freeze_at(TS[0]))
    pilots = {(h.x, h.y) for h in spec.rear_mount}
    n = centre_plates(spec, d.ctx.pitch, d.ctx.params.margin)
    half = n * d.ctx.pitch / 2
    engage = mech.meta["rear_engagement_mm"]
    assert 3.0 <= engage <= 5.0
    screws = [b for b in mech.bodies if ".rear_screw" in b.name]
    assert len(screws) == 2 * mech.meta["rear_screws_per_servo"] >= 4
    world = {"L": set(), "R": set()}
    for b in screws:
        side = b.name[0]
        bb = b.part.bounding_box()
        c = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2)
        x, y = frames[side].local(c)
        assert any(math.hypot(x - px, y - py) < 1e-6 for px, py in pilots), (b.name, x, y)
        assert y > 0            # each servo uses the holes on its own +y side
        world[side].add((round(c[0], 3), round(c[1], 3)))
        # the servos' rear hole faces sit on the centre plates at z = -half / +half
        # (their bumps reach into the plates' cut-outs, so not the bounding box)
        z0, z1 = bb.min.Z, bb.max.Z
        if side == "L":         # tip `engage` into the left servo, head inside the stack
            assert z0 == pytest.approx(-half - engage)
            assert -half < z1 < half
        else:
            assert z1 == pytest.approx(half + engage)
            assert -half < z0 < half
    assert not world["L"] & world["R"]      # the two screw sets never share a hole position


def test_robot_is_mirror_symmetric(design, robot):
    tmpl, d = design("single")
    mech = robot("single", TS[0])
    lefts = [b for b in mech.bodies if b.name.startswith("L.") and b.part is not None]
    assert lefts
    for b in lefts:
        name = b.name[2:]
        if name.startswith(("tie_", "rear_screw")):
            continue            # chassis hardware; checked below
        left = b.part.bounding_box()
        right = mech.body(f"R.{name}").part.bounding_box()
        z_left = (left.min.Z, left.max.Z)
        xy_left = (left.min.X, left.min.Y, left.max.X, left.max.Y)
        assert z_left == pytest.approx((-right.max.Z, -right.min.Z)), name
        assert xy_left == pytest.approx((right.min.X, right.min.Y, right.max.X, right.max.Y)), name
        assert b.fab == mech.body(f"R.{name}").fab
    # rear screws: the right one is the left one mirrored in z, on the right servo's frame
    frames = _frames(d, tmpl.freeze_at(TS[0]))
    for i in range(mech.meta["rear_screws_per_servo"]):
        lb = mech.body(f"L.rear_screw{i}").part.bounding_box()
        rb = mech.body(f"R.rear_screw{i}").part.bounding_box()
        z_left = (lb.min.Z, lb.max.Z)
        assert z_left == pytest.approx((-rb.max.Z, -rb.min.Z))
        lc = frames["L"].local(((lb.min.X + lb.max.X) / 2, (lb.min.Y + lb.max.Y) / 2))
        rc = frames["R"].local(((rb.min.X + rb.max.X) / 2, (rb.min.Y + rb.max.Y) / 2))
        assert lc == pytest.approx(rc)


def test_ties_join_the_inner_plates_above_them(design, robot):
    _, d = design("single")
    mech = robot("single", TS[0])
    td = tie_dims(d.ctx)
    halves = [b for b in mech.bodies if "tie_screw_half" in b.name or "tie_insert_half" in b.name]
    assert len(halves) == 2 * mech.meta["ties"] == 8
    plates = [b for b in mech.bodies if b.name.startswith("centre_plate")]
    for b in halves:
        side = b.name[0]
        plate = mech.body(f"{side}.torso").part
        pb, tb = plate.bounding_box(), b.part.bounding_box()
        if side == "L":        # nothing below the inner plate's leg-side face
            assert tb.min.Z > pb.min.Z
            face = pb.max.Z
        else:
            assert tb.max.Z < pb.max.Z
            face = pb.min.Z
        # the column bears on the plate's top face all round its spigot
        xy = ((tb.min.X + tb.max.X) / 2, (tb.min.Y + tb.max.Y) / 2)
        ring = (disc(xy, td.column - 0.05, face - 0.5, face + 0.5)
                - disc(xy, td.spigot_d / 2 + 0.3, face - 1, face + 1))
        under = ring.moved(Location((0, 0, -0.5 if side == "L" else 0.5)))
        assert _volume(under & plate) == pytest.approx(_volume(under), rel=1e-3)
        # and on the centre plates at the other end
        stack = [p.part for p in plates]
        end = tb.max.Z if side == "L" else tb.min.Z
        cap = (disc(xy, td.column - 0.05, end - 0.25, end + 0.25)
               - disc(xy, td.clearance_d / 2 + 0.3, end - 1, end + 1))
        cap = cap.moved(Location((0, 0, 0.25 if side == "L" else -0.25)))
        got = sum(_volume(cap & s) for s in stack)
        assert got == pytest.approx(_volume(cap), rel=1e-3)


def test_frame_ties_only_touch_the_inner_plate(design):
    """The ties are the robot's: a side's design has none, and what they add to the side
    (their spigot holes and pads) is all in the inner plate."""
    tmpl, d = design("single")
    assert not any(isinstance(g, FrameTies) for g in d.groups)
    ties = FrameTies(d.drive)
    assert ties.claims(d.ctx) == []
    got = ties.realize(Build(d.ctx, d.plan, tmpl.freeze_at(1.0)), Realized())
    assert got.bodies == []
    assert set(got.cuts) == {FRAME_INNER}
    assert set(got.pads) == {FRAME_INNER}
    assert len(got.cuts[FRAME_INNER]) == 4


def test_ties_keep_clear_of_the_servo(design, robot):
    tmpl, d = design("single")
    mech = robot("single", TS[0])
    spec, p = d.ctx.servo, d.ctx.params
    frame = _frames(d, tmpl.freeze_at(TS[0]))["L"]
    L, W, _ = spec.body
    x0, x1 = spec.axis_offset - L / 2, spec.axis_offset + L / 2
    td = tie_dims(d.ctx)
    for b in mech.bodies:
        if "tie_screw_half" in b.name:
            bb = b.part.bounding_box()
            x, y = frame.local(((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2))
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(abs(y) - W / 2, 0.0)
            assert math.hypot(dx, dy) >= td.column + p.margin - 1e-6


# ---------------------------------------------------------------------------
# Catalog
# ---------------------------------------------------------------------------

REQUIRED = (
    "m3_nut", "m3_heat_set_insert", "m2_self_tap_6", "acrylic_cement", "wood_glue", "ca_glue",
    "pla_filament", "petg_filament", "acrylic_3mm", "plywood_3mm", "servo_sts3215",
)


def test_every_catalog_key_the_project_uses_is_registered(design, robot):
    _load()
    keys = set(REQUIRED)
    for sk in fasteners.SCREWS.values():
        keys |= {sk.key(L) for L in sk.lengths}
    _, d = design("single")
    mech = robot("single", TS[0])
    keys |= {b.bom_key for b in mech.bodies if b.bom_key}
    keys |= {line.key for line in mech.bom_extras}
    keys |= {h.screw for h in d.ctx.servo.mount + d.ctx.servo.rear_mount if h.screw}
    missing = sorted(k for k in keys if k not in CATALOG)
    assert missing == []
    for key in keys:
        item = get(key)
        assert item.offers, key
        for o in item.offers:
            assert o.url.startswith("https://"), key
            assert o.pack_qty >= 1, key


def test_catalog_data_the_code_reads():
    assert get("acrylic_3mm").dims["thickness"] == 3.0
    assert get("plywood_3mm").dims["thickness"] == 3.0
    assert get("pla_filament").dims["density"] == 1.24
    assert get("m3_heat_set_insert").dims["hole_d"] == 4.0
    for sk in fasteners.SCREWS.values():
        item = get(sk.key(sk.lengths[0]))
        assert (item.dims["d"], item.dims["head_d"], item.dims["head_h"]) == (
            sk.d, sk.head_d, sk.head_h)
    assert fasteners.shcs("3", 12) == "m3_shcs_12"
    assert fasteners.parse("m2p5_shcs_12") == (fasteners.screw("shcs", "2p5"), 12.0)
    assert fasteners.parse("m3_standoff_ff_20") is None
