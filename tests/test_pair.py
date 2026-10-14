"""The preloaded ball pairs (:mod:`spiderpig.construction.pivots.pair`, 2026-10-12): on the
palletizer, a hub beside a link where its spacer layer has room, a pair in it, the column
preloaded (no float), and the budget's tilt down by an order of magnitude."""

from __future__ import annotations

import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction.pivots.pair import PAIR_TILT_DEG, PairSeat, land_gaps
from tests import cache


def _pair(**kw) -> BuildConfig:
    return BuildConfig(linkage="palletizer", robot=False, pin="ball_pair",
                       pillar="ball_pair_pillar", **kw)


def test_a_seat_pairs_in_a_thick_link_and_counts_a_single_bearings_play():
    seat = PairSeat("bearing_mr84zz", 0.05, hub_bearing="bearing_mr84")
    assert seat.pair(6.35)
    assert not seat.pair(3.0)
    assert seat.play(2) == pytest.approx(0.05)                  # the bore on the shaft
    assert seat.play(1) == pytest.approx(0.063)                 # and the balls' clearance
    assert seat.tilt_deg(2) == PAIR_TILT_DEG < seat.tilt_deg(1)


def test_neighbours_face_to_face_keep_a_gap_for_a_land():
    # the host (3) under a rider (4); a hub (5) of the rider in 6, bonded to it; a plate (0)
    # two layers under the host
    els = [(3, 3), (4, 4), (5, 6), (6, 6), (0, -1)]
    assert land_gaps(els, 0.5) == {3: 0.5, 4: 0.5}


@pytest.mark.slow     # (plans and fabricates the palletizer's side: ~30 s)
def test_the_palletizer_pairs_its_links_inside_its_claims():
    from spiderpig.construction.contract import check_side
    from spiderpig.hardware.bom import bom_from_mechanism

    cfg = _pair()
    tmpl, design = cache.cached_design(cfg)
    assert check_side(design, tmpl.freeze_at(2.0)) == []
    mech = cache.cached_side(cfg, 1.0)
    names = {b.name for b in mech.bodies}
    hubs = [n for n in names if "_hub_" in n]
    assert len(hubs) >= 4
    for h in hubs:                       # each hub holds its open bearing
        stem, link = h.split("_hub_")
        assert f"{stem}_bearing_{link}_hub" in names
    wobble = mech.meta["wobble"]
    assert all(note["play_mm"] == 0.0 for note in wobble.values())
    pairs = [e for note in wobble.values() for e in note["links"] if e.get("bearings") == 2]
    assert pairs
    assert all(e["tilt_deg"] == PAIR_TILT_DEG for e in pairs)
    bom = {r.key: r.qty for r in bom_from_mechanism(mech).purchased}
    assert bom["bearing_mr84"] + bom.get("bearing_mr106", 0) == len(hubs)
    assert bom["disc_spring_din2093_c8"] >= 1
    # the top spacer bears on the top rider's inner ring through a 4 x 6 land, never on
    # its outer ring (the review of 2026-10-13)
    by = {b.name: b for b in mech.bodies if b.part is not None}
    tops = [n for n in by if n.startswith("pin_") and n.endswith("_spacer_hi")]
    assert tops
    for n in tops:
        land = by[n.replace("_spacer_hi", "_land_hi")].part.bounding_box()
        spacer = by[n].part.bounding_box()
        assert abs(land.size.X - 6.0) < 0.01       # inside the inner ring's land
        assert abs(spacer.min.Z - land.max.Z) < 1e-6
    assert bom["shim_4x6x0p5"] == len(tops)
