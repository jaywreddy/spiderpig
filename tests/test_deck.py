"""The electronics deck (construction/deck.py): it fits between the inner frame plates
over the servos, clears every moving part over the whole crank cycle, keeps its ports
reachable, and its mass, centre of mass, BOM rows and DXF plate are what the robot says."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction import deck as deck_mod
from spiderpig.construction.contract import CLASH_MM3
from spiderpig.hardware.bom import bom_from_mechanism
from spiderpig.hardware.catalog import get
from spiderpig.layout import pack
from spiderpig.linkage import build_module_template

ELECTRONICS = {"esp32_servo_driver": "deck_board", "lipo_2s_450": "deck_battery",
               "ip2326_charger": "deck_charger", "bms_hx_2s_jh20": "deck_bms",
               "toggle_mts102": "deck_switch"}


@pytest.fixture(scope="module")
def strider():
    """The default design's robot (the Strider double), built once for this module."""
    from spiderpig.fabricate import fabricate

    cfg = BuildConfig()
    return cfg, fabricate(build_module_template(cfg.module, linkage=cfg.linkage), cfg, 1.0)


@pytest.fixture(scope="module", params=["strider", "klann"])
def built(request, strider, robot):
    if request.param == "strider":
        return strider
    return BuildConfig(linkage="klann", module="quad"), robot("quad", 1.0)


def _deck(mech):
    return [b for b in mech.bodies if b.part is not None and "deck" in b.name]


def _inner_face(mech) -> float:
    return min(abs(b.part.bounding_box().max.Z) for b in mech.bodies
               if b.name == "L.torso")


def test_the_deck_fits_between_the_inner_plates_over_the_chassis(built):
    _, mech = built
    info = mech.meta["deck"]
    assert info["fitted"], info
    assert info["plate_mm"] == [2 * deck_mod.HALF_LEN, pytest.approx(
        2 * _inner_face(mech) - 2 * deck_mod.DECK_GAP, abs=0.01)]
    assert info["rail_y"] - info["chassis_top_y"] >= deck_mod.FLOOR_MARGIN - 1e-6
    face = _inner_face(mech)
    for b in _deck(mech):
        bb = b.part.bounding_box()
        if b.name.endswith("deck_rail"):          # on the inner plate's face (screwed to it)
            assert max(abs(bb.min.Z), abs(bb.max.Z)) <= face + 1e-6
        elif "deck_rail_screw" in b.name:         # up through the plate from the leg side
            torso = next(t for t in mech.bodies if t.name == "L.torso").part.bounding_box()
            plate = torso.max.Z - torso.min.Z
            assert max(abs(bb.min.Z), abs(bb.max.Z)) <= (face + plate
                                                         + deck_mod.RAIL_SCREW.head_h + 1e-6)
        else:
            assert max(abs(bb.min.Z), abs(bb.max.Z)) <= face - deck_mod.DECK_GAP + 1e-6, b.name
    # under the plates' top edges: walled in on both sides
    plate_top = max(b.part.bounding_box().max.Y for b in mech.bodies if b.name == "L.torso")
    assert info["top_y"] < plate_top


def test_the_deck_clears_every_moving_part_over_the_cycle(built):
    _, mech = built
    c = deck_mod.deck_clearance(mech)
    assert c["ok"], c
    assert c["z_gap_mm"] >= 0.2       # (the rail screws' heads are the planner's claims)
    assert c["sweep_gap_mm"] > 1.0


def test_no_deck_part_clashes_at_the_built_angle(built):
    _, mech = built
    allowed = {frozenset(p) for p in mech.meta["fastened"]}
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    names = [b.name for b in _deck(mech)]
    for a in names:
        ba = parts[a].bounding_box()
        for b, pb in parts.items():
            if b == a or frozenset((a, b)) in allowed:
                continue
            bb = pb.bounding_box()
            if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y
                    or bb.min.Y > ba.max.Y or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
                continue
            inter = parts[a] & pb
            vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
            assert vol <= CLASH_MM3, (a, b, vol)


def test_the_ports_and_the_switch_are_reachable(strider):
    _, mech = strider
    by = {b.name: b.part.bounding_box() for b in mech.bodies if b.part is not None}
    plate = by["deck_plate"]
    face = _inner_face(mech)
    # the board's and the charger's USB-C ends face the open front of the bay (the charger
    # stepped back from the edge only as far as a pillar's head in its way down needs)
    assert pytest.approx(
        mech.meta["deck"]["charger_setback_mm"], abs=0.01) == plate.max.X - by["deck_charger"].max.X
    assert plate.max.X - by["deck_charger"].max.X <= 3.0
    assert plate.max.X - by["deck_board"].max.X <= 3.0
    for port in ("deck_board", "deck_charger"):
        p = by[port]
        for name, bb in by.items():
            if name == port or abs(bb.min.Z) > face and abs(bb.max.Z) > face \
                    and bb.min.Z * bb.max.Z > 0:
                continue                           # outboard of the inner plates
            in_front = bb.min.X >= p.max.X - 1e-6
            shares = not (bb.min.Y > p.max.Y or bb.max.Y < p.min.Y
                          or bb.min.Z > p.max.Z or bb.max.Z < p.min.Z)
            assert not (in_front and shares), (port, name)
    # the switch's lever stands above the deck with nothing over it
    sw = by["deck_switch"]
    assert sw.max.Y > plate.max.Y + 10
    for name, bb in by.items():
        if name == "deck_switch":
            continue
        over = (bb.min.Y > sw.max.Y - 1e-6 and not (bb.min.X > sw.max.X or bb.max.X < sw.min.X
                                                     or bb.min.Z > sw.max.Z or bb.max.Z < sw.min.Z))
        assert not over, name


def test_the_deck_mass_is_the_catalogued_electronics_at_their_place(built):
    from spiderpig import walk

    cfg, mech = built
    masses = walk.body_masses(mech, cfg)
    for key, name in ELECTRONICS.items():
        assert masses[name][0] == pytest.approx(get(key).dims["mass_g"])
    deck = _deck(mech)
    grams = sum(masses[b.name][0] for b in deck)
    com = sum(masses[b.name][0] * masses[b.name][1] for b in deck) / grams
    assert 100.0 < grams < 125.0
    assert com[1] > mech.meta["deck"]["chassis_top_y"]          # it raises the robot's COM
    assert abs(com[2]) < 0.5                                     # balanced across
    # the walking model's nominal deck (no parts) is the built one
    legs = walk.side_legs(cfg)
    b = walk.nominal_mass_breakdown(cfg, legs)
    assert b["deck"] == pytest.approx(grams, abs=1.0)
    from spiderpig import servos

    spec = servos.get(cfg.servo)
    sb = next(b for b in mech.bodies if b.name == "L.servo").part.bounding_box()
    centre = np.array([(sb.min.X + sb.max.X) / 2, (sb.min.Y + sb.max.Y) / 2])
    u = centre / np.linalg.norm(centre)          # the servo's +x: O to its body's centre
    _, c = walk.nominal_deck(spec, u)
    assert np.allclose(c, com[:2], atol=1.5), (c, com)


def test_the_bom_lists_the_deck(built):
    _, mech = built
    bom = bom_from_mechanism(mech, group=False)
    rows = {r.key: r for r in bom.purchased}
    for key in ELECTRONICS:
        assert rows[key].qty == 1
    for key in ("resistor_100k", "resistor_33k", "lipo_strap_10mm", "xt30_pigtail_pair",
                "dc_plug_5521_pigtail", "foam_tape"):
        assert rows[key].qty == 1, key
    for key in ("m25_nylon_standoff_mf_6", "m25_nylon_nut", "m25_nylon_screw_5"):
        assert rows[key].qty == 4
    assert all(rows[k].sku for k in ("m25_nylon_standoff_mf_6", "m25_nylon_nut",
                                     "m25_nylon_screw_5"))   # distributor parts, not a kit
    screw = mech.meta["deck"]["screw"]
    assert sum("deck_screw" in w for w in rows[screw].where) == 4
    assert sum("deck_insert" in w for w in rows["m3_heat_set_insert"].where) == 4
    for key in (*ELECTRONICS, "resistor_100k", "resistor_33k"):
        assert rows[key].url.startswith("https://")
        # the generic IP2326 module's listing shows no price to a fetch (hardware.sources)
        assert rows[key].pack_price_usd or key == "ip2326_charger"


def test_the_deck_plate_is_on_the_dxf_sheets(strider):
    _, mech = strider
    sheets = pack(mech, (300.0, 300.0))
    placed = {name: sketch for sheet in sheets for name, sketch, _ in sheet}
    assert "deck_plate" in placed
    bb = placed["deck_plate"].bounding_box()
    dims = sorted((bb.max.X - bb.min.X, bb.max.Y - bb.min.Y))
    assert dims == [pytest.approx(mech.meta["deck"]["plate_mm"][1], abs=0.01),
                    pytest.approx(136.0, abs=0.01)]
    # its holes came through: screws, standoffs, switch (circles) and slots
    # (and the four cable-tie slots beside the wire slots, 2026-10-04; the battery cradle's
    # two screw holes, 2026-10-05; the path notches are in the outline)
    assert len(placed["deck_plate"].wires()) == 1 + 4 + 4 + 1 + 2 + 2 + 4 + 2


def test_every_deck_part_is_one_valid_solid(built):
    from spiderpig.construction.contract import bad_solids

    _, mech = built
    assert [b for b in bad_solids(mech) if "deck" in b["part"]] == []


def test_the_deck_lowers_straight_down_onto_its_rails(built):
    """The assembly's last step: the deck, electronics on, goes down between the inner plates
    past the pillars' inner M4 heads (3 mm into the bay from each plate) onto the rails.
    Checked on the parts' geometry: each lowered part swept straight up meets nothing."""
    _, mech = built
    assert deck_mod.deck_path(mech) == []
    assert deck_mod.deck_clearance(mech)["blocked"] == []


def test_the_deck_is_notched_round_the_pillar_heads_in_its_way(strider):
    """On the Strider double the plate's corners pass by J2's and J6's inner heads and
    washers: each one the unnotched plate would meet on its way down has a notch round it,
    0.5 mm clear."""
    _, mech = strider
    info = mech.meta["deck"]
    plate = next(b for b in mech.bodies if b.name == "deck_plate").part.bounding_box()
    hw = (plate.max.Z - plate.min.Z) / 2
    face = _inner_face(mech)
    heads = [b.part.bounding_box() for b in mech.bodies
             if b.name.startswith(("L.pillar_", "R.pillar_"))
             and ("_screw" in b.name or "_washer" in b.name)]
    in_way = [bb for bb in heads
              if bb.max.Y > plate.min.Y and bb.min.X < plate.max.X and bb.max.X > plate.min.X
              and min(abs(bb.min.Z), abs(bb.max.Z)) < hw and max(abs(bb.min.Z),
                                                                abs(bb.max.Z)) >= face - 1e-6]
    # (since 2026-10-05 the double's pillars are one-piece M3 standoffs: their 5.7 mm button
    # heads stand clear of the plate's corners and only the washers' 0.5 mm clearance is
    # notched, so nothing may overlap the plate itself; with the M4 heads before, the corners
    # met the heads)
    assert in_way or info["notches"]                # the check has something to do here
    notches = info["notches"]
    c = deck_mod.NOTCH_CLEAR - 0.01
    for bb in in_way:
        # what of it the plate's outline covers, and its clearance, is inside a notch
        lo_x, hi_x = max(bb.min.X - c, plate.min.X), min(bb.max.X + c, plate.max.X)
        lo_z, hi_z = max(bb.min.Z - c, plate.min.Z), min(bb.max.Z + c, plate.max.Z)
        assert any(x0 <= lo_x and x1 >= hi_x and z0 <= lo_z and z1 >= hi_z
                   for x0, x1, z0, z1 in notches), bb
    # small corner notches only: the plate keeps its rails' screw holes' webs
    for x0, x1, z0, z1 in notches:
        assert min(x1, plate.max.X) - max(x0, plate.min.X) < 5.0
        assert min(z1, hw) - max(z0, -hw) < 4.0


def test_deck_path_sees_a_part_in_the_way_and_a_notch_clears_it():
    """:func:`deck.deck_path` on a toy: a plate under a head standing over its edge is
    blocked; the same plate notched round the head is not."""
    from types import SimpleNamespace

    from build123d import Box, Cylinder, Pos

    from spiderpig.mechanism import Body

    plate = Box(40, 3, 20).move(Pos(0, 1.5, 0))
    head = Cylinder(3.8, 2.2).move(Pos(18.0, 10.0, 9.0))     # over the corner, z 7.9..10.1
    notched = plate - Box(10, 10, 5).move(Pos(18.0, 1.5, 9.5))

    def mech(p):
        return SimpleNamespace(bodies=[Body(name="deck_plate", part=p),
                                       Body(name="L.pillar_A_screw9", part=head)])

    got = deck_mod.deck_path(mech(plate))
    assert [(a, b) for a, b, _ in got] == [("deck_plate", "L.pillar_A_screw9")]
    assert deck_mod.deck_path(mech(notched)) == []
    # the deck's own screws go in after it, the rails are on the plates first
    assert not deck_mod.lowered("deck_screw0")
    assert not deck_mod.lowered("L.deck_rail")
    assert not deck_mod.lowered("R.deck_insert1")
    assert deck_mod.lowered("deck_cradle_screw0")
    assert deck_mod.lowered("deck_board")


def test_a_screw_hole_a_notch_crowds_moves_into_the_bay(strider):
    """:func:`deck.insert_z`: ``klann_lego``'s B pillars' inner heads stand at the inserts'
    x, so their path notches came 0.30 mm from the deck screws' holes (under Ponoko's 1 mm);
    the inserts then sit 1.5 mm further into the bay. The Strider's corner notches leave
    them centred in the rails."""
    lay = deck_mod.DeckLayout(x_c=0.0, rail_y0=13.7, deck_y=21.7, pitch=3.0, z_in=-36.57,
                              z_leg=-38.6, spigot_x=12.0)
    near = [(19.7, 28.3, -36.57, -33.07)]                  # klann_lego's, round a head
    assert deck_mod.insert_z(lay, near, 1.0) == 7.0
    assert deck_mod.insert_z(lay, [], 1.0) == deck_mod.RAIL_T / 2
    far = [(55.0, 69.0, -36.57, -33.07)]                   # a corner's
    assert deck_mod.insert_z(lay, far, 1.0) == deck_mod.RAIL_T / 2
    r = deck_mod.CLEARANCE["3"] / 2
    moved = deck_mod.replace(lay, insert_z=7.0)
    assert min(deck_mod._rect_dist(p, *near[0]) for p in moved.screws()) - r >= 1.0
    _, mech = strider
    assert mech.meta["deck"]["insert_z"] == deck_mod.RAIL_T / 2


def test_the_battery_cradle_is_screwed_to_the_deck(strider):
    """The user's decision of 2026-10-05: two M3 screws and nuts, no glue."""
    _, mech = strider
    assert not [x for x in mech.bom_extras if x.key == "ca_glue"]
    screws = [b for b in mech.bodies if b.name.startswith("deck_cradle_screw")]
    nuts = [b for b in mech.bodies if b.name.startswith("deck_cradle_nut")]
    assert len(screws) == len(nuts) == 2
    assert {b.bom_key for b in nuts} == {"m3_nut"}
    assert all(b.bom_key.startswith("m3_bhcs_") for b in screws)
    pairs = {frozenset(p) for p in mech.meta["fastened"]}
    for i in range(2):
        assert frozenset((f"deck_cradle_screw{i}", f"deck_cradle_nut{i}")) in pairs
    by = {b.name: b.part.bounding_box() for b in mech.bodies if b.part is not None}
    plate = by["deck_plate"]
    for i in range(2):
        s, n = by[f"deck_cradle_screw{i}"], by[f"deck_cradle_nut{i}"]
        assert s.max.Y > plate.max.Y                 # head on the ear, over the deck
        assert pytest.approx(plate.min.Y) == n.max.Y      # nut under it
        assert s.min.Y < n.min.Y                     # through the nut
    # one each side of the battery, diagonal
    zs = sorted((by[f"deck_cradle_screw{i}"].min.Z + by[f"deck_cradle_screw{i}"].max.Z) / 2
                for i in range(2))
    assert zs[0] < 0 < zs[1]


def test_no_deck_carries_its_mass_as_a_payload():
    from types import SimpleNamespace

    from spiderpig.sim.mjcf import DECK_FALLBACK_G, SimParams, payload_g

    p = SimParams()
    assert p.payload_g == 0.0
    assert payload_g(SimpleNamespace(meta={"deck": {"fitted": True}}), p) == 0.0
    assert payload_g(SimpleNamespace(meta={"deck": {"fitted": False}}), p) == DECK_FALLBACK_G
    assert math.isclose(payload_g(SimpleNamespace(meta={}), p), DECK_FALLBACK_G)
