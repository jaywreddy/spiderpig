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
    # both per servo since 2026-10-08 (one 2026-10-04..08, the far one lost to the pad)
    assert len(screws) == 2 * mech.meta["rear_screws_per_servo"] == 4
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
    _, _d = design("single")
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
    assert holes.count(RAIL_HOLE) == len(deck) + 4           # the rails' and the 4 M3 ties'
    heads = {s.label for s in d.plan.shapes("drive")}
    assert {"frame tie screw head", "deck rail screw head"} <= heads


def test_tie_holes_keep_two_thicknesses_off_the_screw_holes(design):
    """A tie's hole shares the inner plate with the servo's front screw holes and the
    centre plates with both servos' rear screw holes and head recesses: each tie is moved
    along the servo until it is the service's two thicknesses off all of them."""
    from spiderpig.construction.chassis import tie_locals, tie_neighbours

    _, d = design("single")
    r = tie_dims(d.ctx).hole_d / 2
    near = tie_neighbours(d.ctx)
    assert near
    for x, y in tie_locals(d.ctx):
        for hx, hy, hr, web in near:
            assert math.hypot(x - hx, y - hy) - r - hr >= web, (x, y, hx, hy)


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
# Test drive, round 4 (docs/history/TESTDRIVE.md): a robot's tie spigots take a few drops
# of CA glue each, not a bottle, so a robot buys one bottle
# ---------------------------------------------------------------------------


def test_the_robot_buys_no_ca_glue(robot):
    """No glue in the structure since 2026-10-04 (ties, pillars, deck rails, centre plates
    are screwed), and since 2026-10-05 the battery cradle is screwed to the deck too: the
    only adhesive is the Chicago barrels' epoxy (threadlocker on metal threads aside)."""
    from spiderpig.hardware.bom import bom_from_mechanism

    mech = robot("single", 1.0)
    assert not [line for line in mech.bom_extras if line.key == "ca_glue"]
    assert any(line.key == "epoxy_2part" for line in mech.bom_extras)
    bom = bom_from_mechanism(mech, group=False)
    assert not [r for r in bom.purchased if r.key == "ca_glue"]


def _sts_frames():
    from dataclasses import replace as _replace

    left = ServoFrame((0.0, 0.0), (1.0, 0.0))
    return (left, _replace(left, hand=-1))


def test_the_sts3215_bus_window_and_channel():
    """The research of 2026-10-08: the STS3215's bus sockets are top-entry headers in a
    trench in the rear face (x 11.55..16.55, |y| <= 10.1); a plug, 3.9 along x and 9.9
    across, stands 3.5 beyond the face and its wires 2.5 more, turned toward +x. Each servo
    uses the socket on its own +y side (the user's decision of 2026-10-08): the plates get a
    window round that plug, inside the research's x 11.2..16.9, |y| <= 10.5, and a channel
    |y| <= 4.5 from it to the far edge, through the plates within 6 mm of each rear face;
    the stack holds one plug, not two."""
    from spiderpig.construction import chassis as ch
    from spiderpig.materials import sheet

    spec = servos.get("sts3215")
    ports = spec.bus_ports
    assert ports is not None
    assert (ports.opening, ports.used, ports.exit) == ("face", "own", "+x")
    assert (ports.x0, ports.x1, ports.y0, ports.y1) == (11.55, 16.55, -10.1, 10.1)
    window, channel = ports.slot()
    assert window == pytest.approx((11.6, 16.5, -0.4, 10.5))
    wx0, wx1, wy0, wy1 = window
    assert wx0 >= 11.2                                              # the research's window
    assert wx1 <= 16.9
    assert wy1 <= 10.5
    cx = (ports.x0 + ports.x1) / 2                                  # the plug, centred
    assert wx0 <= cx - ports.plug_t / 2 - 0.5 + 1e-9
    assert wx1 >= cx + ports.plug_t / 2 + 0.5 - 1e-9
    assert wy0 <= 0.1                                               # the +y socket's plug
    assert wy1 >= 10.0
    assert channel == pytest.approx((cx, math.inf, -4.5, 4.5))
    assert ports.height == pytest.approx(6.0)
    assert not ports.opposed
    # one plug and its wires over the other servo's raised pad (1.9), not two plugs
    assert ch.centre_stack(spec, 1.0) == pytest.approx(6.0 + 1.9)
    assert ch.centre_stack(spec, 1.0) < 2 * ports.height + 1.0
    t = sheet("al5052_1p6mm").thickness
    n = centre_plates(spec, t, 1.0)
    half = n * t / 2
    slots = ch._port_slots(spec, _sts_frames(), half)
    assert [s[1] for s in slots] == [pytest.approx(window), pytest.approx(channel)] * 2
    assert all(isinstance(s[1], ch.PortCut) for s in slots)
    assert [z for _, _, z in slots] == [pytest.approx((-half, -half + 6.0))] * 2 + [
        pytest.approx((half - 6.0, half))] * 2
    # the two servos' windows sit on opposite sides (the right servo's +y is the left's -y)
    left, right = _sts_frames()
    lw = [left.local(right.xy(x, y))[1] for x in window[:2] for y in window[2:]]
    assert max(lw) <= 0.4 + 1e-9


def test_the_sts3215_keeps_both_rear_screws():
    """The user's decision of 2026-10-08: both rear screws per servo. The near hole (8.3,
    10.25) keeps 1 x t of web to the window (``chassis.BUS_WEB_T``), its head recess opening
    into it; the far one (32.75, 10.25) keeps 1.62 mm to the raised pad's relief (its
    measured outline grown 0.2 mm, the pocket's 1 mm corners): over 0.063 in, under 0.080 in,
    so the centre plates are 0.063 in (thinner than the frame's 0.080 in only because they
    seat more screws: ``chassis.centre_sheet``)."""
    from spiderpig.construction import chassis as ch
    from spiderpig.materials import sheet

    spec = servos.get("sts3215")
    frames = _sts_frames()
    assert ch._centre_sheet(spec, "al5052_2mm", 1.0) == "al5052_1p6mm"
    kept = {}
    for key in ("al5052_1p6mm", "al5052_2mm", "al5052_2p3mm"):
        t = sheet(key).thickness
        n = centre_plates(spec, t, 1.0)
        half = n * t / 2
        rel = ch._relief_volumes(spec, frames, half)
        slots = ch._port_slots(spec, frames, half)
        rs = ch.rear_screws(spec, n, t, 1)
        kept[key] = [(h.x, h.y) for h in ch._clear_holes(rs, frames, rel, half, t, slots,
                                                        sheet(key).min_hole)]
    assert kept == {"al5052_1p6mm": [(8.3, 10.25), (32.75, 10.25)],
                    "al5052_2mm": [(8.3, 10.25)], "al5052_2p3mm": []}
    pad = next(r for r in spec.rear_reliefs if r.label == "raised pad")
    assert (pad.x1, pad.y1, pad.grow) == (29.7, 9.2, 0.2)
    rect = next(r for _, r, _ in ch._relief_volumes(spec, frames, 4.0) if r[1] > 29)
    assert ch._cut_distance((32.75, 10.25), rect) - 1.2 == pytest.approx(1.617, abs=1e-3)
    window = ch.PortCut(spec.bus_ports.slot()[0])
    assert ch._cut_distance((8.3, 10.25), window) - 1.2 == pytest.approx(2.165, abs=1e-3)


@pytest.mark.parametrize("model", ["xl430_w250", "xl330_m288"])
def test_the_bus_change_leaves_the_xl_servos_alone(model):
    """The XL servos have no bus ports modelled: their centre sheet and stack are what they
    were before 2026-10-08."""
    from spiderpig.construction import chassis as ch
    from spiderpig.materials import sheet

    spec = servos.get(model)
    want = {"xl430_w250": ("al5052_2p3mm", 4), "xl330_m288": ("al5052_2p5mm", 3)}[model]
    key = ch._centre_sheet(spec, "al5052_2mm", 1.0)
    assert (key, centre_plates(spec, sheet(key).thickness, 1.0)) == want


def test_the_robot_buys_one_bus_y_cable(robot):
    """One socket per servo (2026-10-08): one Y cable from the driver board feeds both
    servos; no ready-made one was found, so the line is unpriced with a search note."""
    from spiderpig.construction.chassis import BUS_Y_CABLE

    mech = robot("single", 1.0)
    assert [(line.key, line.qty) for line in mech.bom_extras
            if line.key == BUS_Y_CABLE] == [(BUS_Y_CABLE, 1)]
    assert mech.meta["bus_sockets_used"] == "own"
    item = get(BUS_Y_CABLE)
    assert item.offer is not None
    assert item.offer.price_usd is None
    assert "search" in item.offer.note


@pytest.mark.parametrize("model", ["xl430_w250", "xl330_m288"])
def test_a_servo_without_bus_ports_has_no_plug_slots(model):
    """``chassis._port_slots`` reads the plugs' height only for a servo whose
    ``ServoSpec.bus_ports`` are modelled: the XL servos have none, and get no slot (pyright's
    possibly-``None`` ``ports.height``, 2026-10-08: unreachable, ``rect`` is ``None`` then)."""
    from dataclasses import replace as _replace

    from spiderpig.construction import chassis as ch

    spec = servos.get(model)
    assert spec.bus_ports is None
    left = ServoFrame((0.0, 0.0), (1.0, 0.0))
    assert ch._port_slots(spec, (left, _replace(left, hand=-1)), 3.0) == []

