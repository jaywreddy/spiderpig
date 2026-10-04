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
    assert c["z_gap_mm"] >= 0.2       # the rail screws' heads in the gap under the plate
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
    # the board's and the charger's USB-C ends face the open front of the bay
    assert plate.max.X - by["deck_charger"].max.X < 0.01
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
    assert rows["m25_nylon_nut"].same_pack_as      # one kit
    screw = mech.meta["deck"]["screw"]
    assert sum("deck_screw" in w for w in rows[screw].where) == 4
    assert sum("deck_insert" in w for w in rows["m3_heat_set_insert"].where) == 4
    for key in (*ELECTRONICS, "resistor_100k", "resistor_33k"):
        assert rows[key].url.startswith("https://")
        assert rows[key].pack_price_usd


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
    # (and the four cable-tie slots beside the wire slots, 2026-10-04)
    assert len(placed["deck_plate"].wires()) == 1 + 4 + 4 + 1 + 2 + 2 + 4


def test_no_deck_carries_its_mass_as_a_payload():
    from types import SimpleNamespace

    from spiderpig.sim.mjcf import DECK_FALLBACK_G, SimParams, payload_g

    p = SimParams()
    assert p.payload_g == 0.0
    assert payload_g(SimpleNamespace(meta={"deck": {"fitted": True}}), p) == 0.0
    assert payload_g(SimpleNamespace(meta={"deck": {"fitted": False}}), p) == DECK_FALLBACK_G
    assert math.isclose(payload_g(SimpleNamespace(meta={}), p), DECK_FALLBACK_G)
