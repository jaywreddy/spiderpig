"""Seam tests of the bolt crank's fits (:class:`construction.crank.BoltCrank`) on numbers
the test states: no linkage, no plan search, no fabrication (``no_fabricate``).

The crank sheet is the default 0.100 in 6061-T6 (2.54 mm); the hex standoff crankpins are
M3 x 5.5 AF in stock lengths 5, 6, 8, 10, 12, 15, 16, 18, 20, 22, 25, 30, ... mm; their
screws ISO 7380 M3 button heads (6, 8, 10 ... mm, head 1.65 mm) over a DIN 9021 washer
(0.8 mm) and DIN 125 ones (0.5 mm).
"""

from __future__ import annotations

import math

import pytest

from spiderpig.construction.crank import (
    BoltCrank,
    CrankRoute,
    HexJoint,
    Run,
    hex_bearing_nm,
    shim_stack,
)
from spiderpig.stack import GAP_MAX
from tests import _ctx

pytestmark = pytest.mark.no_fabricate

HEX = BoltCrank().for_sheet("al6061_2p5mm")
ROUND = BoltCrank(pin="round").for_sheet("al6061_2p5mm")
T = 2.54


# -- the screws into a hex standoff's ends ------------------------------------------------


@pytest.mark.parametrize("length", [5.0, 6.0, 8.0])
def test_a_hex_standoff_under_ten_mm_takes_no_pair_of_screws(length):
    """Two M3 screws into a standoff tapped through need their tips 0.3 mm apart and 2.5 mm
    of thread each: (L - 0.3) / 2 >= 2.5, and the shortest screw (6 mm) over its 0.8 mm wide
    washer and up to two 0.5 mm ones leaves at least 4.2 mm: nothing under 10 mm takes it."""
    assert HEX.hex_screws(length) is None


def test_a_ten_mm_standoff_takes_a_washer_under_each_head():
    """(10 - 0.3) / 2 = 4.85 mm of thread at most: a 6 mm screw over the 0.8 mm washer
    reaches 5.2, so one DIN 125 washer more (4.7 mm), the fewest that fit."""
    assert HEX.hex_screws(10.0) == (6.0, 1, 4.7)


@pytest.mark.parametrize("length", [12.0, 15.0, 30.0, 60.0])
def test_a_long_standoff_takes_the_six_mm_screw_bare(length):
    """The most thread up to 6 mm: a 6 mm screw over the wide washer, 5.2 mm (an 8 mm one
    would reach 7.2, past ``hex_engage_max``)."""
    assert HEX.hex_screws(length) == (6.0, 0, 5.2)


# -- fit_hex: the stock length for a span --------------------------------------------------


def test_a_span_a_stock_length_matches_is_flush_with_no_collars():
    j = HEX.fit_hex(15.0, T, T)
    assert (j.standoff, j.length, j.out_lo, j.out_hi) == ("hex_standoff_m3_15", 15.0, 0.0, 0.0)
    assert (j.collar_lo, j.collar_hi) == (0.0, 0.0)
    assert j.sleeve == pytest.approx(15.0 - 2 * T - HEX.sleeve_play)      # 9.72
    assert j.stack_lo == j.stack_hi == pytest.approx(0.8 + 1.65)         # washer + head


def test_what_stands_past_is_split_evenly_into_printed_collars():
    """A 13.08 mm span on the 15 mm standoff: 1.92 mm past the plates, 0.96 each end (the
    lower end first would put 1.2 under and 0.72 over: a taller lower gap), each taken up by
    a printed collar of that thickness."""
    j = HEX.fit_hex(13.08, T, T)
    assert j.length == 15.0
    assert (j.out_lo, j.out_hi) == (0.96, 0.96)
    assert (j.collar_lo, j.collar_hi) == (0.96, 0.96)
    assert j.stack_lo == pytest.approx(0.96 + 0.8 + 1.65)


def test_standing_past_is_preferred_to_a_recess():
    """16.5 mm: the 16 mm standoff would sit 0.25 mm inside each pocket (within
    ``recess_max``), the 18 mm one stands 0.75 mm past each plate: flush or past first."""
    j = HEX.fit_hex(16.5, T, T)
    assert (j.length, j.out_lo, j.out_hi) == (18.0, 0.75, 0.75)


def test_a_protrusion_under_the_collar_minimum_floats_without_a_collar():
    """29.9 mm on the 30 mm standoff: 0.05 mm each end, under the 0.4 mm printable collar:
    none (the plate floats that much)."""
    j = HEX.fit_hex(29.9, T, T)
    assert (j.length, j.out_lo, j.out_hi, j.collar_lo, j.collar_hi) == (30.0, 0.05, 0.05, 0, 0)


@pytest.mark.parametrize("span", [26.0, 26.5, 27.0, 27.5])
def test_no_stock_length_fits_between_25_and_30_mm(span):
    """The series steps 5 mm past 25: a 25 mm standoff recessed at most 0.3 each end covers
    25.6, a 30 mm one standing 1.2 past each end 27.6: in between nothing fits (27.5: the
    30 mm one would need 1.25 at one end)."""
    if span >= 27.6 - 1e-9:
        pytest.fail("not in the hole")
    assert HEX.fit_hex(span, T, T) is None


def test_the_upper_end_uses_the_air_over_its_plate_first():
    """``air_hi``: 0.46 mm of air over the upper plate in its own layer takes the whole
    0.46 mm the 15 mm standoff stands past a 14.54 mm span, so the lower end needs no gap."""
    j = HEX.fit_hex(14.54, T, T, air_hi=0.46)
    assert (j.out_lo, j.out_hi) == (0.0, 0.46)
    plain = HEX.fit_hex(14.54, T, T)
    assert (plain.out_lo, plain.out_hi) == (0.23, 0.23)


def test_no_room_over_the_upper_plate_leaves_a_recess_or_nothing():
    """``out_hi_max`` 0 (the hub plate's pocket, the horn spacer over it): 20.6 mm takes the
    20 mm standoff 0.3 mm inside both pockets (the 22 one would stand 1.4 past the lower
    plate alone, over ``protrude_max``), its hex 2.24 mm in each plate."""
    j = HEX.fit_hex(20.6, T, T, out_hi_max=0.0)
    assert (j.length, j.out_lo, j.out_hi) == (20.0, -0.3, -0.3)
    assert (j.engaged_lo, j.engaged_hi) == (pytest.approx(2.24), pytest.approx(2.24))
    assert HEX.fit_hex(20.75, T, T, out_hi_max=0.0) is None


def test_a_capped_chain_has_no_upper_screw_and_loses_the_thrust_play():
    j = HEX.fit_hex(29.9, T, T, capped=True)
    assert (j.screw_hi, j.engage_hi, j.washers_hi, j.stack_hi) == ("", 0.0, 0, 0.0)
    assert j.screw_lo == "m3_bhcs_6"
    assert j.engaged_hi == pytest.approx(T - HEX.thrust_play)


def test_a_journal_has_no_sleeve():
    assert HEX.fit_hex(15.0, T, T, sleeve=False).sleeve == 0.0


def test_fit_hex_answers_the_same_for_the_same_numbers():
    """Remembered per rounded arguments: the planner asks it at every node."""
    assert HEX.fit_hex(13.08, T, T) is HEX.fit_hex(13.080000001, T, T)


# -- hex_gap_fit: opening the gaps a stock length needs ------------------------------------


def test_the_least_gap_opening_that_fits_goes_in_the_first_slot():
    """26.0 mm fits nothing; 1.6 mm more (27.6) fits the 30 mm standoff, 1.2 past each end:
    the whole 1.6 in the first slot (the lowest web's gap)."""
    j = HEX.hex_gap_fit(26.0, [(3, 0.0), (4, 0.0)], T, T)
    assert isinstance(j, HexJoint)
    assert (j.length, j.span) == (30.0, 27.6)
    assert (j.gap, j.gaps) == (1.6, ((3, 1.6),))


def test_an_opening_past_a_full_gap_spills_into_the_next_slot():
    """The first slot has 3.0 of its 4.0 mm: 1.0 there, the 0.6 left in the next."""
    j = HEX.hex_gap_fit(26.0, [(3, 3.0), (4, 0.0)], T, T)
    assert (j.gap, j.gaps) == (GAP_MAX, ((3, 4.0), (4, 0.6)))


def test_no_room_in_the_gaps_means_no_fit():
    assert HEX.hex_gap_fit(26.0, [(3, GAP_MAX)], T, T) is None


# -- the round standoff: fit_web ------------------------------------------------------------


def test_a_round_standoff_fills_its_span_exactly_with_no_shims():
    j = ROUND.fit_web(12.0, 0.0, T, T)
    assert (j.standoff, j.length, j.shims, j.gap) == ("gobilda_1501_12", 12.0, 0.0, 0.0)
    assert (j.screw_lo, j.engage_lo) == ("m4_bhcs_8", pytest.approx(8 - T))


def test_a_round_standoff_longer_than_its_span_takes_the_gap_and_shims_fill_the_rest():
    """11.6 mm free and a 1.0 mm gap: the 12 mm standoff needs 0.4 of the gap, 0.6 mm of
    DIN 988 shims under its end fill the rest (and the lower screw passes them)."""
    j = ROUND.fit_web(11.6, 1.0, T, T)
    assert (j.length, j.gap, j.shims) == (12.0, 0.4, 0.6)
    assert j.engage_lo == pytest.approx(8 - T - 0.6)


def test_a_round_standoff_past_every_stock_length_is_refused():
    assert ROUND.fit_web(500.0, 0.0, T, T) is None


# -- a chain on a hand-made layout ------------------------------------------------------------


def test_a_chain_spans_its_plates_outer_faces_at_the_layouts_z():
    """Runs in layers 3-5 of 3 mm layers: webs in 2 and 6, each 2.54 mm plate on its layer's
    floor, so the span is 6 + 3 x 3 + 3 + 2.54 - 6 = 14.54 mm and the 0.46 mm of air over
    the upper web takes what the 15 mm standoff stands past."""
    L = _ctx.layout(top=10)
    j = HEX.chain_fit_web(L, 3, 5, t=T)
    assert (j.span, j.length, j.out_lo, j.out_hi) == (14.54, 15.0, 0.0, 0.46)


def test_the_hub_plate_sits_at_its_layers_top():
    """The hub plate (layer 6 here) against the horn spacer: no air over it, the span 15.0
    exactly, and capped (no screw over the hub)."""
    L = _ctx.layout(top=10)
    assert HEX.plate_z(L, 6, T, hub=6) == pytest.approx((18.46, 21.0))
    assert HEX.air_over(L, 6, T, hub=6) == 0.0
    assert HEX.air_over(L, 6, T) == pytest.approx(0.46)
    j = HEX.chain_fit_web(L, 3, 5, t=T, hub=6, capped=True)
    assert (j.span, j.length, j.out_hi, j.screw_hi) == (15.0, 15.0, 0.0, "")


def test_a_gap_over_the_lowest_web_lengthens_the_span():
    L = _ctx.layout(top=10, gaps={2: 1.5})
    assert HEX.chain_fit_web(L, 3, 5, t=T).span == pytest.approx(14.54 + 1.5)


# -- the horn screws ---------------------------------------------------------------------------


def test_the_sts3215_horn_screw_takes_whole_1mm_shims_before_tenths():
    """Its M3 horn: at most 2.9 mm of thread (the tip 0.3 under the inner plate's top face),
    2.5 wanted. Through a 1.6 mm hub and a 3.032 spacer the 6 mm screw is short (1.37) and
    the 8 mm one long (3.37): one whole 1 mm shim (a DIN 433 pair) leaves 2.37."""
    sk, length, e, shim = HEX.horn_fit_web(_ctx.context(servo="sts3215"), 1.6, 3.032)
    assert (sk.kind, sk.size, length, shim) == ("bhcs", "3", 8, 1.0)
    assert e == pytest.approx(8 - 1.6 - 3.032 - 1.0)


def test_the_sts3215_horn_screw_fits_the_default_hub_bare():
    _, length, e, shim = HEX.horn_fit_web(_ctx.context(servo="sts3215"), T, 3.032)
    assert (length, shim) == (8, 0.0)
    assert e == pytest.approx(2.428)


def test_the_xl430_horn_screw_takes_tenths_where_a_whole_shim_leaves_too_little():
    """The XL430's M2 horn holds 1.7 mm of thread at most: an 8 mm screw through 3.175 +
    3.032 reaches 1.793, a whole 1 mm shim would leave 0.79 (under 1.5): 0.1 mm."""
    sk, length, e, shim = HEX.horn_fit_web(_ctx.context(servo="xl430_w250"), 3.175, 3.032)
    assert (sk.kind, length, shim) == ("shcs", 8, 0.1)
    assert e == pytest.approx(1.693)


def test_the_xl330_horn_takes_self_tapping_screws():
    sk, *_ = HEX.horn_fit_web(_ctx.context(servo="xl330_m288"), T, 3.032)
    assert (sk.kind, sk.size) == ("self_tap", "2")


def test_no_horn_screw_reaches_through_a_40mm_stack():
    assert HEX.horn_fit_web(_ctx.context(servo="sts3215"), 40.0, 3.032) is None


# -- small pieces ---------------------------------------------------------------------------------


def test_the_stub_is_the_longest_stock_standoff_that_seats_in_the_outer_plate():
    """10 mm from the lowest web's underside to the outer plate's bottom face (a 2.032 mm
    plate): the 12 mm standoff ends 2 mm under the plate (``stub_below`` 2.5), its 6 mm
    screw 3.46 into it; 3 mm reaches nothing, nor does 100."""
    assert HEX.stub_z(10.0, 2.032, T) == ("m3_round_standoff_ff_12", 12, -2.0, 6)
    assert HEX.stub_z(3.0, 2.032, T) is None
    assert HEX.stub_z(100.0, 2.032, T) is None


def test_two_webs_in_adjacent_layers_take_no_standoff():
    """A run of 0 layers: the screws into a 5-6 mm standoff's ends would meet."""
    assert not HEX._web_span_ok(0, 0, 3.0, T)
    assert all(HEX._web_span_ok(n, 0, 3.0, T) for n in range(1, 10))


def test_the_crank_reads_the_sheet_and_sizes_its_pocket():
    assert (HEX.web_t, HEX.web_yield) == (T, 276.0)
    assert HEX.hex_pocket_af() == pytest.approx(5.6)
    assert HEX.hex_reach() == pytest.approx(5.6 / math.sqrt(3) + 1.6)
    assert (HEX.rider_d(), ROUND.rider_d()) == (8.5, 6.0)
    assert HEX.head_r() == pytest.approx(9.0 / 2 + 0.3)      # the wide washer, not the head


def test_the_hex_pocket_is_one_solid_with_six_reliefs_round_its_centre():
    cut = HEX.hex_cut((10.0, 5.0), 0.0, 2.0, 0.3)
    assert len(cut.solids()) == 1
    bb = cut.bounding_box()
    z0, z1 = bb.min.Z, bb.max.Z
    assert (z0, z1) == (pytest.approx(0.0), pytest.approx(2.0))
    centre = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2)
    assert centre == (pytest.approx(10.0, abs=0.01), pytest.approx(5.0, abs=0.01))
    hexagon = math.sqrt(3) / 2 * 5.6 ** 2 * 2.0
    assert hexagon < cut.volume < hexagon + 6 * math.pi * 0.8 ** 2 * 2.0


def test_shim_stacks_are_thickest_first():
    assert shim_stack(0.7) == [0.5, 0.2]
    assert shim_stack(1.35) == [1.0, 0.3]
    assert shim_stack(0.0) == []


def test_the_hex_bearing_model():
    """0.75 p a^2 L, a the flat (AF / sqrt 3) less the relief: an M3 nut's 5.5 AF in 2.4 mm
    of printed plastic at 50 MPa carries 0.91 N·m; no engagement or no flat, nothing."""
    assert hex_bearing_nm(5.5, 2.4) == pytest.approx(0.75 * 50 * (5.5 / math.sqrt(3)) ** 2
                                                     * 2.4 / 1e3)
    assert hex_bearing_nm(5.5, 2.4) == pytest.approx(0.9075)
    assert hex_bearing_nm(5.5, 0.0) == 0.0
    assert hex_bearing_nm(5.5, 2.0, relief=10.0) == 0.0


def test_the_thrust_sleeve_only_with_a_capped_chain_into_the_hub():
    """Whichever chain's top web is the hub plate (layer 5 or 9 here) is capped, so the
    crank body needs the thrust sleeve; a hub no chain reaches (7), none."""
    route = CrankRoute((Run("M", 2, 4), Run("N", 6, 8)))
    for hub in (5, 9):
        assert HEX.stub_thrust_r(None, route, hub) == pytest.approx(HEX.thrust_od / 2)
    assert HEX.stub_thrust_r(None, route, 7) == 0.0
