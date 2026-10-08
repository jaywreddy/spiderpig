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
    # the stock M2 x 6 through two own 0.063 in plates: 2.8 mm (since 2026-10-08)
    assert 2.0 <= engage <= 5.0
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
    "m3_nut", "m2_self_tap_6", "acrylic_cement", "wood_glue", "ca_glue",
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
    window round that plug, inside the research's x 11.2..16.9, |y| <= 10.5, and its own
    channel, centred on its plug (y 0.55..9.55), from it to the far edge; the stack holds
    one plug, not two."""
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
    # each servo's own channel, centred on its plug (review of 2026-10-08), 9 wide: the
    # wires (6.3 across where they leave the plug) inside it
    assert ports.plug_y == pytest.approx(5.05)
    assert channel == pytest.approx((cx, math.inf, 0.55, 9.55))
    assert channel[2] <= ports.plug_y - ports.wire_w / 2 - 0.5
    assert channel[3] >= ports.plug_y + ports.wire_w / 2 + 0.5
    assert ports.height == pytest.approx(6.0)
    assert not ports.opposed
    # one plug and its wires, 1 mm of margin (BUS_WIRE_MARGIN), under the other servo's
    # raised pad (1.9): not two plugs; the SO model's header pins (model_only) don't count
    assert ch.bus_reserve(spec) == pytest.approx(7.0)
    assert ch.centre_stack(spec, 1.0) == pytest.approx(7.0 + 1.9)
    assert ch.centre_stack(spec, 1.0) < 2 * ports.height + 1.0
    t = sheet("al5052_1p6mm").thickness
    n = centre_plates(spec, t, 1.0)
    assert (n, round(n * t, 2)) == (6, 9.6)
    half = n * t / 2
    slots = ch._port_slots(spec, _sts_frames(), half)
    assert [s[1] for s in slots] == [pytest.approx(window), pytest.approx(channel)] * 2
    assert all(isinstance(s[1], ch.PortCut) for s in slots)
    # the window from each rear face to the reserve, the channel from the plug's top
    assert [z for _, _, z in slots] == [
        pytest.approx((-half, -half + 7.0)), pytest.approx((-half + 3.5, -half + 7.0)),
        pytest.approx((half - 7.0, half)), pytest.approx((half - 7.0, half - 3.5))]
    # the two servos' windows and channels sit on opposite sides (the right servo's +y is
    # the left's -y)
    left, right = _sts_frames()
    for rect, most in ((window, 0.4), (channel, -0.55)):
        lw = [left.local(right.xy(x, y))[1] for x in (rect[0], 50.0) for y in rect[2:]]
        assert max(lw) <= most + 1e-9


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
        rs = ch.rear_screws(spec, n, t, 2 if key == "al5052_1p6mm" else 1)
        kept[key] = [(h.x, h.y) for h in ch._clear_holes(rs, frames, rel, half, t, slots,
                                                        sheet(key).min_hole)]
        if key == "al5052_1p6mm":
            # two own plates: the M2 x 6 the servo ships with (MountHole.stock), 2.8 mm into
            # the pilot (one plate: an M2 x 5, another SKU; 4.4 with an M2 x 6, past the
            # unknown pilot's modelled depth)
            assert (rs.key, rs.engage) == ("m2_self_tap_6", pytest.approx(2.8))
    assert kept == {"al5052_1p6mm": [(8.3, 10.25), (32.75, 10.25)],
                    "al5052_2mm": [(8.3, 10.25)], "al5052_2p3mm": []}
    pad = next(r for r in spec.rear_reliefs if r.label == "raised pad")
    assert (pad.x1, pad.y1, pad.grow) == (29.7, 9.2, 0.2)
    rect = next(r for _, r, _ in ch._relief_volumes(spec, frames, 4.0) if r[1] > 29)
    assert ch._cut_distance((32.75, 10.25), rect) - 1.2 == pytest.approx(1.617, abs=1e-3)
    window = ch.PortCut(spec.bus_ports.slot()[0])
    assert ch._cut_distance((8.3, 10.25), window) - 1.2 == pytest.approx(2.165, abs=1e-3)


def _envelope(frame, x0, x1, y0, y1, z0, z1):
    from spiderpig.construction import chassis as ch

    return ch._rounded_rect(frame, x0, x1, y0, y1, 0.05, z0, z1)


@pytest.mark.slow       # ~20 s of OCCT booleans: two envelopes against every plate and screw
def test_the_bus_plugs_and_wires_meet_no_centre_plate_and_no_screw(design, robot):
    """The review of 2026-10-08: each servo's plug, and its wires swept from the plug's top
    along their channel past the plates' +x edge, at the research's upper height (6.5 mm
    over the rear face: 0.5 more than the 6.0 taken), meet no centre plate and no rear
    screw; the other servo's real bumps (its raised pad) stand clear of the wires too. Not
    vacuous: the same envelopes moved off their cuts meet the plates by tens of mm^3 (the
    wires 3 mm toward +y, or 3 mm toward -y a plate lower where no channel is cut; the plug
    1 mm toward -x). (The far rear screw's head clears the wires by ~0.05 mm in y with the
    1.3 mm wire taken: DECISIONS.md, verify on the first article.)"""
    from spiderpig.construction import chassis as ch

    tmpl, d = design("single")
    mech = robot("single", TS[0])
    spec = d.ctx.servo
    ports = spec.bus_ports
    frames = _frames(d, tmpl.freeze_at(TS[0]))
    n, t = mech.meta["centre_plates"], centre_t(d.ctx)
    half = n * t / 2
    top = ports.height + 0.5                       # the research's upper estimate
    cx, cy = (ports.x0 + ports.x1) / 2, ports.plug_y
    plates = [b for b in mech.bodies if b.name.startswith("centre_plate")]
    screws = [b for b in mech.bodies if ".rear_screw" in b.name]
    assert len(plates) == n
    for side, face, sign in (("L", -half, 1.0), ("R", half, -1.0)):
        f = frames[side]
        plug = _envelope(f, cx - ports.plug_t / 2, cx + ports.plug_t / 2,
                         cy - ports.plug_w / 2, cy + ports.plug_w / 2,
                         *sorted((face, face + sign * ports.plug_h)))
        wires = _envelope(f, cx - ports.plug_t / 2, 120.0,
                          cy - ports.wire_w / 2, cy + ports.wire_w / 2,
                          *sorted((face + sign * ports.plug_h, face + sign * top)))
        for env, what in ((plug, "plug"), (wires, "wires")):
            for b in plates + screws:
                assert _met(env, b.part) < 1e-6, (side, what, b.name)
        if side == "L":                     # the negative checks, on one side
            off = {"wires +y 3": _envelope(f, cx - ports.plug_t / 2, 120.0,
                                           cy + 3 - ports.wire_w / 2, cy + 3 + ports.wire_w / 2,
                                           face + ports.plug_h, face + top),
                   "wires -y 3, a plate lower": _envelope(
                       f, cx - ports.plug_t / 2, 120.0, cy - 3 - ports.wire_w / 2,
                       cy - 3 + ports.wire_w / 2, face + ports.plug_h - t, face + top - t),
                   "plug -x 1": _envelope(f, cx - 1 - ports.plug_t / 2,
                                          cx - 1 + ports.plug_t / 2, cy - ports.plug_w / 2,
                                          cy + ports.plug_w / 2, face, face + ports.plug_h)}
            for what, env in off.items():
                assert sum(_met(env, b.part) for b in plates) > 10.0, what
    # the other servo's pad (mirrored in y) over the wires' run: below them, not in them
    pad = next(r for r in spec.rear_reliefs if r.label == "raised pad")
    assert -half + top <= half - pad.height
    assert ch.centre_stack(spec, d.ctx.params.margin) <= n * t


def _met(a, b) -> float:
    """The volume two shapes share (0 when they don't meet)."""
    met = a & b
    return 0.0 if met is None else sum(x.volume for x in met.solids())


@pytest.mark.slow       # ~10 s of OCCT booleans
def test_the_centre_plates_are_mirror_twins(design, robot):
    """The review of 2026-10-08: centre plate k and plate n-1-k are one part, the second
    turned a half turn about the servo's x axis (the two servos are mirror images through
    the stack), however the cuts are listed (``chassis._merge_close`` stretches the cut that
    grows least, not the first)."""
    from build123d import Axis, Pos

    tmpl, d = design("single")
    mech = robot("single", TS[0])
    f = _frames(d, tmpl.freeze_at(TS[0]))["L"]
    plates = {b.name: b.part for b in mech.bodies if b.name.startswith("centre_plate")}
    n = len(plates)
    axis = Axis((f.o[0], f.o[1], 0), (f.u[0], f.u[1], 0))
    for k in range(n // 2):
        a, b = plates[f"centre_plate{k}"], plates[f"centre_plate{n - 1 - k}"]
        turned = b.rotate(axis, 180)
        turned = turned.moved(Pos(0, 0, a.bounding_box().min.Z - turned.bounding_box().min.Z))
        assert a.volume == pytest.approx(b.volume, rel=1e-9)
        assert sum(x.volume for x in (a - turned).solids()) < 1e-6, k
        assert sum(x.volume for x in (turned - a).solids()) < 1e-6, k


def test_the_centre_plates_hold_the_jammed_servo(robot):
    """The review of 2026-10-08: the 0.063 in centre plates, cut by the bus window and
    channel, at the servo's jam torque (``strength.centre_plate_row``): the screws' tear-out
    across their least web (1.62 mm, the far hole to the pad's relief) governs, far over the
    jam warning (2)."""
    from spiderpig import strength
    from spiderpig.config import BuildConfig

    mech = robot("single", TS[0])
    row = strength.centre_plate_row(mech.meta, BuildConfig(linkage="klann", module="single"),
                                    {"torque_limit_nm": 0.85, "walk_torque_nm": 0.18})
    assert row is not None
    assert (row["sheet"], row["own_plates"]) == ("al5052_1p6mm", 2)
    assert row["least_web_mm"] == pytest.approx(1.617, abs=1e-3)
    jam = row["jam"]
    assert jam["governs"] == "screw tear-out"
    assert jam["load_n"] == pytest.approx(850 / 24.45, rel=1e-3)        # the couple
    assert jam["safety"] > 2 * strength.JAM_WARN
    assert set(jam["stress_mpa"]) >= {"net section at the window", "net section at the pad",
                                      "tie bearing", "screw bearing"}
    assert strength.level(row) is None


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


def test_each_servo_has_its_own_bus_cable_on_hand(robot):
    """One socket per servo (2026-10-08): each servo's own cable (in its box) to one of the
    driver board's two bus ports, no Y cable: two on-hand lines, in no cart or total."""
    from spiderpig.construction.chassis import BUS_CABLE
    from spiderpig.hardware.bom import ON_HAND, bought

    mech = robot("single", 1.0)
    assert [(line.key, line.qty) for line in mech.bom_extras
            if line.key == BUS_CABLE] == [(BUS_CABLE, 2)]
    assert mech.meta["bus_sockets_used"] == "own"
    assert BUS_CABLE in ON_HAND
    assert not bought(BUS_CABLE)
    item = get(BUS_CABLE)
    assert item.offer is not None
    assert item.offer.price_usd is not None       # the spares are priced


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

