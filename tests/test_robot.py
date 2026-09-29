"""Tests for the robot assembly (:mod:`construction.robot`) and the hardware catalog."""

from __future__ import annotations

import itertools
import math

import pytest
from build123d import Location

from construction.base import FRAME_INNER, Build
from construction.robot import (
    FrameTies,
    ServoFrame,
    centre_plates,
    servo_frame,
    tie_dims,
)
from fabricate import BuildConfig, design_side, fabricate, side_groups
from hardware import parts
from hardware.catalog import CATALOG, _load, get
from klann import build_klann_template, create_klann_geometry
from shapes import disc

CONFIG = BuildConfig(module="single")
TS = (1.0, 4.38)


@pytest.fixture(scope="module")
def single():
    tmpl = build_klann_template(create_klann_geometry())
    design = design_side(tmpl, CONFIG)
    robots = {t: fabricate(tmpl, CONFIG, t) for t in TS}
    return tmpl, design, robots


def _placed(mech):
    return {b.name: b.placed_part() for b in mech.bodies if b.part is not None}


def _boxes_meet(a, b) -> bool:
    return all(max(getattr(a.min, c), getattr(b.min, c))
               < min(getattr(a.max, c), getattr(b.max, c)) - 1e-6 for c in "XYZ")


def _volume(shape) -> float:
    return 0.0 if shape is None else sum(s.volume for s in shape.solids())


@pytest.mark.parametrize("t", TS)
def test_no_two_parts_of_the_robot_intersect(single, t):
    _, _, robots = single
    mech = robots[t]
    fastened = {frozenset(p) for p in mech.meta["fastened"]}
    parts_ = _placed(mech)
    boxes = {n: p.bounding_box() for n, p in parts_.items()}
    clashes = []
    for a, b in itertools.combinations(parts_, 2):
        if frozenset((a, b)) in fastened or not _boxes_meet(boxes[a], boxes[b]):
            continue
        vol = _volume(parts_[a] & parts_[b])
        if vol > 1e-3:
            clashes.append((a, b, round(vol, 3)))
    assert clashes == []


def test_fastened_screws_really_engage(single):
    _, _, robots = single
    mech = robots[TS[0]]
    parts_ = _placed(mech)
    engage = mech.meta["rear_engagement_mm"]
    for screw, host in mech.meta["fastened"]:
        if "rear_screw" in screw:
            d = get(mech.body(screw).bom_key).dims["d"]
            expected = math.pi * (d / 2) ** 2 * engage
            assert _volume(parts_[screw] & parts_[host]) == pytest.approx(expected, rel=1e-3)


def test_every_robot_part_is_one_valid_solid(single):
    _, _, robots = single
    for b in robots[TS[0]].bodies:
        if b.part is not None:
            assert len(b.part.solids()) == 1, b.name
            assert b.part.is_valid, b.name


def _frames(design, mech) -> dict[str, ServoFrame]:
    build = Build(design.ctx, design.plan, mech)
    left = servo_frame(build, design.drive)
    return {"L": left, "R": ServoFrame(left.o, left.u, hand=-1)}


def test_rear_screws_sit_on_the_servo_pilots(single):
    tmpl, design, robots = single
    mech = robots[TS[0]]
    spec = design.ctx.servo
    frames = _frames(design, tmpl.freeze_at(TS[0]))
    pilots = {(h.x, h.y) for h in spec.rear_mount}
    n = centre_plates(spec, design.ctx.pitch, design.ctx.params.margin)
    half = n * design.ctx.pitch / 2
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
        servo = mech.body(f"{side}.servo").part.bounding_box()
        z0, z1 = bb.min.Z, bb.max.Z
        if side == "L":         # tip `engage` into the left servo, head inside the stack
            rear = servo.max.Z
            assert z0 == pytest.approx(-half - engage)
            assert rear == pytest.approx(-half)
            assert -half < z1 < half
        else:
            rear = servo.min.Z
            assert z1 == pytest.approx(half + engage)
            assert rear == pytest.approx(half)
            assert -half < z0 < half
    assert not world["L"] & world["R"]      # the two screw sets never share a hole position


def test_robot_is_mirror_symmetric(single):
    tmpl, _, robots = single
    mech = robots[TS[0]]
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
    _, design, _ = single
    frames = _frames(design, tmpl.freeze_at(TS[0]))
    for i in range(mech.meta["rear_screws_per_servo"]):
        lb = mech.body(f"L.rear_screw{i}").part.bounding_box()
        rb = mech.body(f"R.rear_screw{i}").part.bounding_box()
        z_left = (lb.min.Z, lb.max.Z)
        assert z_left == pytest.approx((-rb.max.Z, -rb.min.Z))
        lc = frames["L"].local(((lb.min.X + lb.max.X) / 2, (lb.min.Y + lb.max.Y) / 2))
        rc = frames["R"].local(((rb.min.X + rb.max.X) / 2, (rb.min.Y + rb.max.Y) / 2))
        assert lc == pytest.approx(rc)


def test_ties_join_the_inner_plates_above_them(single):
    tmpl, design, robots = single
    mech = robots[TS[0]]
    d = tie_dims(design.ctx)
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
        ring = (disc(xy, d.column - 0.05, face - 0.5, face + 0.5)
                - disc(xy, d.spigot_d / 2 + 0.3, face - 1, face + 1))
        under = ring.moved(Location((0, 0, -0.5 if side == "L" else 0.5)))
        assert _volume(under & plate) == pytest.approx(_volume(under), rel=1e-3)
        # and on the centre plates at the other end
        stack = [p.part for p in plates]
        end = tb.max.Z if side == "L" else tb.min.Z
        cap = (disc(xy, d.column - 0.05, end - 0.25, end + 0.25)
               - disc(xy, d.clearance_d / 2 + 0.3, end - 1, end + 1))
        cap = cap.moved(Location((0, 0, 0.25 if side == "L" else -0.25)))
        got = sum(_volume(cap & s) for s in stack)
        assert got == pytest.approx(_volume(cap), rel=1e-3)


def test_frame_ties_only_touch_the_inner_plate(single):
    tmpl, design, _ = single
    ties = [g for g in design.groups if isinstance(g, FrameTies)]
    assert len(ties) == 1
    assert ties[0].claims(design.ctx) == []
    got = ties[0].realize(Build(design.ctx, design.plan, tmpl.freeze_at(1.0)))
    assert got.bodies == []
    assert set(got.cuts) == {FRAME_INNER}
    assert set(got.pads) == {FRAME_INNER}
    assert len(got.cuts[FRAME_INNER]) == 4
    side_only = side_groups(design.ctx, BuildConfig(module="single", robot=False))
    assert not any(isinstance(g, FrameTies) for g in side_only)


def test_ties_keep_clear_of_the_servo(single):
    tmpl, design, robots = single
    mech = robots[TS[0]]
    spec, p = design.ctx.servo, design.ctx.params
    frame = _frames(design, tmpl.freeze_at(TS[0]))["L"]
    L, W, _ = spec.body
    x0, x1 = spec.axis_offset - L / 2, spec.axis_offset + L / 2
    d = tie_dims(design.ctx)
    for b in mech.bodies:
        if "tie_screw_half" in b.name:
            bb = b.part.bounding_box()
            x, y = frame.local(((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2))
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(abs(y) - W / 2, 0.0)
            assert math.hypot(dx, dy) >= d.column + p.margin - 1e-6


# ---------------------------------------------------------------------------
# Catalog
# ---------------------------------------------------------------------------

REQUIRED = (
    "m3_nut", "m3_nylock", "m3_washer", "m3_heat_set_insert", "m2_self_tap_6",
    "acrylic_cement", "wood_glue", "ca_glue", "pla_filament", "petg_filament",
    "acrylic_3mm", "plywood_3mm", "servo_sts3215",
)


def test_every_catalog_key_the_project_uses_is_registered(single):
    _load()
    keys = set(REQUIRED)
    for size, lengths in parts.SHCS_LENGTHS.items():
        keys |= {parts.shcs(size, L) for L in lengths}
    for size, lengths in parts.SELF_TAP_LENGTHS.items():
        keys |= {parts.self_tap(size, L) for L in lengths}
    keys |= {parts.standoff_ff(L) for L in parts.STANDOFF_FF_LENGTHS}
    _, design, robots = single
    mech = robots[TS[0]]
    keys |= {b.bom_key for b in mech.bodies if b.bom_key}
    keys |= {line.key for line in mech.bom_extras}
    keys |= {h.screw for h in design.ctx.servo.mount + design.ctx.servo.rear_mount if h.screw}
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
    for size, (dk, k) in parts.SHCS_HEAD.items():
        item = get(parts.shcs(size, parts.SHCS_LENGTHS[size][0]))
        assert (item.dims["head_d"], item.dims["head_h"]) == (dk, k)
