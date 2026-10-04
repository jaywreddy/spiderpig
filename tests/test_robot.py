"""Tests for the robot assembly (:mod:`construction.robot`) and the hardware catalog."""

from __future__ import annotations

import math

import pytest

from spiderpig import servos
from spiderpig.construction.base import FRAME_INNER, Build, Realized
from spiderpig.construction.chassis import (
    MIN_ENGAGE,
    ServoFrame,
    centre_plates,
    centre_t,
    servo_frame,
    tie_dims,
)
from spiderpig.construction.contract import bad_solids, clashes
from spiderpig.construction.robot import FrameTies
from spiderpig.hardware import fasteners
from spiderpig.hardware.catalog import CATALOG, _load, get
from spiderpig.servos.model import UNKNOWN_HOLE_DEPTH
from tests.tiers import quick

TS = (1.0, 4.38)


def _placed(mech):
    return {b.name: b.placed_part() for b in mech.bodies if b.part is not None}


def _volume(shape) -> float:
    return 0.0 if shape is None else sum(s.volume for s in shape.solids())


@pytest.mark.parametrize("t", quick(TS, [TS[0]]))
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
    t = centre_t(d.ctx)                 # the centre plates: the frame's aluminium (2026-10-04)
    n = centre_plates(spec, t, d.ctx.params.margin)
    half = n * t / 2
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
    """Each tie: a standoff chain per side from the inner plate's servo-side face to the
    centre plates, an M4 screw up through the inner plate from the leg side (its head in
    the gap under the plate), a stud through the centre plates (2026-10-04: no glue)."""
    _, d = design("single")
    mech = robot("single", TS[0])
    chains = [b for b in mech.bodies if ".tie_standoff" in b.name]
    assert mech.meta["ties"] == 4
    assert {b.name[0] for b in chains} == {"L", "R"}
    plates = [b for b in mech.bodies if b.name.startswith("centre_plate")]
    lo = min(b.part.bounding_box().min.Z for b in plates)
    for side in ("L", "R"):
        plate = mech.body(f"{side}.torso").part.bounding_box()
        mine = [b.part.bounding_box() for b in chains if b.name[0] == side]
        if side == "L":
            assert min(bb.min.Z for bb in mine) >= plate.max.Z - 1e-3
            assert max(bb.max.Z for bb in mine) == pytest.approx(lo, abs=0.01)
        else:
            assert max(bb.max.Z for bb in mine) <= plate.min.Z + 1e-3
        for i in range(mech.meta["ties"]):
            screw = mech.body(f"{side}.tie_screw{i}").part.bounding_box()
            # its head on the leg side of the inner plate
            assert (screw.min.Z < plate.min.Z) if side == "L" else (screw.max.Z > plate.max.Z)
    studs = [b for b in mech.bodies if b.name.startswith("tie_stud")]
    assert len(studs) == mech.meta["ties"]
    assert not [line for line in mech.bom_extras if line.key == "ca_glue"
                and "tie" in line.where]


def test_frame_ties_only_touch_the_inner_plate(design):
    """The ties are the robot's: a side's design has none, and what they add to the side
    (their screw holes and pads, and the electronics deck rails' two screw holes) is all in
    the inner plate; the screws' heads under it are the drive group's claims."""
    from spiderpig.construction.deck import RAIL_HOLE, spigot_points

    tmpl, d = design("single")
    assert not any(isinstance(g, FrameTies) for g in d.groups)
    ties = FrameTies(d.drive)
    assert ties.claims(d.ctx) == []
    got = ties.realize(Build(d.ctx, d.plan, tmpl.freeze_at(1.0)), Realized())
    assert got.bodies == []
    assert set(got.cuts) == {FRAME_INNER}
    assert set(got.pads) == {FRAME_INNER}
    build = Build(d.ctx, d.plan, tmpl.freeze_at(1.0))
    deck = spigot_points(build, d.drive)
    assert len(deck) == 2                                    # the deck fits the Strider
    holes = sorted(c.d for c in got.cuts[FRAME_INNER])
    assert len(holes) == 4 + len(deck)
    assert holes.count(RAIL_HOLE) == len(deck)
    heads = {s.label for s in d.plan.shapes("drive")}
    assert {"frame tie screw head", "deck rail screw head"} <= heads


def test_ties_keep_clear_of_the_servo(design, robot):
    tmpl, d = design("single")
    mech = robot("single", TS[0])
    spec, p = d.ctx.servo, d.ctx.params
    frame = _frames(d, tmpl.freeze_at(TS[0]))["L"]
    L, W, _ = spec.body
    x0, x1 = spec.axis_offset - L / 2, spec.axis_offset + L / 2
    td = tie_dims(d.ctx)
    for b in mech.bodies:
        if ".tie_standoff" in b.name:
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


# ---------------------------------------------------------------------------
# Test drive, round 4 (docs/agentlib/TESTDRIVE.md): a robot's tie spigots take a few drops
# of CA glue each, not a bottle, so a robot buys one bottle
# ---------------------------------------------------------------------------


def test_the_robots_glue_is_a_few_drops_on_the_battery_cradle(robot):
    """No glue in the structure since 2026-10-04 (ties, pillars, deck rails, centre plates
    are screwed): CA only for the battery cradle, epoxy for the Chicago barrels."""
    from spiderpig.hardware.bom import bom_from_mechanism

    mech = robot("single", 1.0)
    glue = [line for line in mech.bom_extras if line.key == "ca_glue"]
    assert all("cradle" in line.where for line in glue)
    assert 0 < sum(line.qty for line in glue) < 1
    assert any(line.key == "epoxy_2part" for line in mech.bom_extras)
    bom = bom_from_mechanism(mech, group=False)
    row = next(r for r in bom.purchased if r.key == "ca_glue")
    assert row.packs == 1
