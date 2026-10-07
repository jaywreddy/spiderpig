"""Seam tests of the default pivots' planner rules and fits on numbers the test states: the
Chicago screw pin (:class:`construction.pivots.chicago.ChicagoShaft`) and the standoff
pillar (:class:`construction.pivots.standoff.StandoffAxle`). No linkage, no plan search,
no fabrication (``no_fabricate``).

The Chicago screw: an M3 x 4 mm barrel in stock lengths 4-16 mm in 1 mm steps, then 18, 20,
22, 23, 25, 28 ... 80; its barrel head 1.9 mm, its screw head 1.4 mm over a 0.5 mm washer;
shims in 0.1-1.0 mm steps; 0.05 mm least play. The standoff: goBILDA 1501 M4 lengths 12-60
mm (where one fills the column: 0.1 mm under to 0.8 mm over its gap), else one steel shaft
made to length (0.1 mm steps, M3 ends).
"""

from __future__ import annotations

import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction.base import ConstructionError
from spiderpig.construction.pivots.chicago import MAX_BARREL, ChicagoAxle, ChicagoShaft, Fit
from spiderpig.construction.pivots.standoff import COLUMN_TOL, SHIM_STEP, StandoffAxle
from spiderpig.stack import Unbuildable
from tests import _ctx

pytestmark = pytest.mark.no_fabricate

PIN = ChicagoShaft()
PILLAR = StandoffAxle()
SHAFT = PILLAR.one_piece()


# -- ChicagoShaft.column: the planner's rule ------------------------------------------------


@pytest.mark.parametrize("links", [[3], [3, 4], [2, 6], [1, 9]])
def test_a_stock_barrel_spans_the_links_while_searching(links):
    PIN.column(False, links, 12, (True, True), 3.0)


def test_no_barrel_spans_a_stack_past_the_longest():
    """Links in layers 1 and 30: 90 mm of stack and the washer, past the 80 mm barrel."""
    with pytest.raises(Unbuildable, match="no stock Chicago screw spans its 90 mm stack "
                       r"\(longest 80 mm\)"):
        PIN.column(False, [1, 30], 31, (True, True), 3.0)


def test_a_pillar_or_a_pin_with_no_links_is_not_the_pins_business():
    PIN.column(True, [1, 30], 31, (True, True), 3.0)
    PIN.column(False, [], 31, (True, True), 3.0)


def test_the_longest_barrel_a_linkage_allows_is_a_planner_rule():
    """Strider pins take at most 23 mm (:data:`MAX_BARREL`): layers 2-8 (21 mm + the 0.55 mm
    washer and play) fit, layers 2-9 (24 mm) don't."""
    cap = ChicagoShaft(max_length=MAX_BARREL["strider"])
    cap.column(False, [2, 8], 12, (True, True), 3.0)
    with pytest.raises(Unbuildable, match="its 24 mm stack needs a barrel over the 23 mm"):
        cap.column(False, [2, 9], 12, (True, True), 3.0)


def test_the_strider_resolves_its_cap_and_the_klann_keeps_the_long_barrels():
    axle = ChicagoAxle()
    assert axle.resolve(_ctx.context()).shaft.max_length == 23.0            # the Strider
    klann = _ctx.context(config=BuildConfig(linkage="klann", module="single", robot=False))
    assert axle.resolve(klann) is axle
    assert axle.hole() == pytest.approx(4.2)                     # the barrel's running fit


def test_the_shims_rule_waits_for_the_plans_z():
    """Links in layers 3-20 with a 2 mm clearance gap over layer 10: 56 mm at the plan's z,
    so the 60 mm barrel leaves 3.45 mm to take up, more than two end slots (2.2 mm) and one
    1 mm shim hold. While searching (no final z) the rule isn't checked: the stack only
    grows at the plan's z, and an estimate could rule out what fits. Layers 3-21 (59 mm)
    fit the 60 mm barrel."""
    L = _ctx.layout(top=30, gaps={10: 2.0})
    with pytest.raises(Unbuildable, match="no stock Chicago screw fits its 56 mm stack: a 60 "
                       r"mm barrel leaves 3\.5 mm of shims"):
        PIN.column(False, [3, 20], 30, (True, True), 3.0, layout=L)
    PIN.column(False, [3, 20], 30, (True, True), 3.0)
    PIN.column(False, [3, 21], 30, (True, True), 3.0, layout=L)


def test_the_column_reads_the_air_its_own_parts_close():
    """``air``: what the plan's z leaves free round the pin's own parts (its barrel closes
    it): 3 mm less stack takes the 3-20 column back under the 60 mm barrel's shims."""
    L = _ctx.layout(top=30, gaps={10: 2.0})
    PIN.column(False, [3, 20], 30, (True, True), 3.0, layout=L, air=lambda a, b: 3.0)


# -- ChicagoShaft.fit: the barrel and its shims -------------------------------------------------


def test_one_layer_takes_the_4mm_barrel_its_shims_under_the_screws_head():
    """3 mm of link, the 0.5 mm washer, 0.05 play: the 4 mm barrel, 0.45 over: 0.4 mm of
    shims (whole 0.1 steps) under the screw's head, 0.1 mm of play."""
    assert PIN.fit(3.0, 3.0) == Fit(4.0, 0.0, 0.4, 0.1)


def test_a_stack_that_matches_a_barrel_takes_no_shims():
    assert PIN.fit(8.4, 3.0) == Fit(9.0, 0.0, 0.0, 0.1)


def test_shims_go_under_the_barrels_head_when_the_screws_slot_is_full():
    """A 2.0 mm gap over the screw end holds its head and washer only: the 0.4 mm go to the
    barrel's end."""
    assert PIN.fit(3.0, 3.0, slot_hi=2.0) == Fit(4.0, 0.4, 0.0, 0.1)


def test_more_shims_than_both_end_slots_hold_is_refused():
    """40 mm: the 43 mm barrel leaves 2.4 mm, 1.0 under the screw's head and 1.4 under the
    barrel's, whose slot holds 1.1."""
    with pytest.raises(ConstructionError, match="a 43 mm barrel over a 40.0 mm stack leaves "
                       "2.4 mm of shims"):
        PIN.fit(40.0, 3.0)


def test_shim_rings_are_counted_thickest_first():
    assert PIN.shim_count(0.7) == 2              # 0.5 + 0.2
    assert PIN.shim_count(1.35) == 2             # 1.0 + 0.3 (the 0.05 left is play)
    assert PIN.shim_count(0.0) == 0


def test_the_pin_needs_its_head_and_its_shims_in_two_end_layers():
    """The screw's head, washer and play take 1.95 mm of its layer (1.9 mm layers: refused);
    the 2 mm steps between barrel lengths up to 22 must fit as shims in what the two end
    layers leave (2 p - 1.95 - 1.9 + one 0.1 mm shim): 2.9 mm layers hold them, 2.8 don't.
    A Chicago screw is never a pillar."""
    PIN.check(_ctx.context(pitch=2.9), False)
    with pytest.raises(ConstructionError, match="a 2 mm step between barrel lengths needs "
                       "more shims than two 2.8 mm end layers hold"):
        PIN.check(_ctx.context(pitch=2.8), False)
    with pytest.raises(ConstructionError, match="don't fit a 1.9 mm layer"):
        PIN.check(_ctx.context(pitch=1.9), False)
    with pytest.raises(ConstructionError, match="link pin only"):
        PIN.check(_ctx.context(), True)


# -- the standoff pillar ---------------------------------------------------------------------------


@pytest.mark.parametrize(("gap", "stock"), [
    (11.0, None),      # under the shortest (12) by more than it may be long
    (11.9, 12.0),      # 0.1 over its gap: within max_long 0.8
    (12.0, 12.0),
    (12.5, None),      # 0.5 short of 12.5 (max_short 0.1), 14 is 1.5 over
    (60.5, None),      # past the longest
])
def test_a_gobilda_segment_is_the_nearest_stock_length_within_its_tolerance(gap, stock):
    assert PILLAR.segment(gap) == stock


def test_end_shims_take_up_to_2mm_where_the_upper_end_is_a_spacer():
    assert PILLAR.segment(12.5, shims=True) == 12.0
    assert PILLAR.segment(14.1, shims=True) == 14.0
    assert PILLAR.segment(62.4, shims=True) is None


@pytest.mark.parametrize("gap", [12.5, 33.0, 62.4, 128.1])
def test_the_one_piece_shaft_is_made_to_its_columns_length(gap):
    """0.1 mm steps from 8 to 300 mm (MISUMI NETRF6): any column is its own length."""
    assert SHAFT.segment(gap) == gap


def test_the_one_piece_shaft_is_m3_steel_with_its_own_screws_and_shims():
    assert (SHAFT.stock, SHAFT.size, SHAFT.screw_d, SHAFT.thread_max) == ("shaft", "M3", 3.0, 6.0)
    assert (SHAFT.washer_key, SHAFT.shim_key) == ("m3_washer_9021", "shim_din988_3x6")
    assert SHAFT.end_screw(2.032)[:2] == ("m3_bhcs_8", 8)
    assert PILLAR.end_screw(2.032)[:2] == ("m4_bhcs_8", 8)
    assert PILLAR.end_screw(3.0, plate=False)[:2] == ("m4_bhcs_6", 6)     # a free end


def test_a_column_one_stock_standoff_fills_stays_gobilda():
    """Faces 0 and 5 in 3 mm layers: a 12 mm column, the 12 mm goBILDA standoff."""
    assert PILLAR.column_axle({3}, 5, 3.0) == PILLAR      # (remembered: an equal axle)


def test_a_column_no_stock_length_fills_but_a_splice_would_becomes_one_shaft():
    """A 33 mm column (32 is 1 short, 34 one over) whose free layers would take a splice
    (12 + 18 or so, the removed spliced pillar's rule): one shaft made to 33 mm."""
    assert PILLAR._spliceable({11}, 12, 3.0, 0, None)
    got = PILLAR.column_axle({11}, 12, 3.0)
    assert (got.stock, got.size) == ("shaft", "M3")


def test_a_short_column_with_no_free_layer_for_a_splice_is_refused():
    """Faces 0 and 6, links in every layer between: 15 mm, which no goBILDA length fills
    (14 short, 16 long; no shims under a link), no splice could, and the planner always
    refused (one goBILDA standoff's length, so not 'too long for stock')."""
    every = set(range(1, 6))
    assert not PILLAR._spliceable(every, 6, 3.0, 0, None)
    assert PILLAR.column_axle(every, 6, 3.0) is None
    with pytest.raises(Unbuildable, match="no stock standoff \\(12-60 mm\\) nor a shaft made to "
                       "length fills its 15 mm column"):
        PILLAR.column(True, sorted(every), 6, (True, True), 3.0)


def test_a_column_longer_than_any_stock_length_is_one_shaft():
    """Faces 0 and 25: 72 mm, past the 60 mm goBILDA: a shaft, splice or not."""
    got = PILLAR.column_axle({5}, 25, 3.0)
    assert got.stock == "shaft"


def test_the_column_is_measured_at_the_plans_z():
    """A 1.5 mm gap over layer 2 makes the 12 mm column 13.5: no goBILDA length (12 short
    by 1.5, 14 long by 0.5 is in max_long: 14)."""
    L = _ctx.layout(top=5, gaps={2: 1.5})
    zs = PILLAR.column_axle({3}, 5, 3.0, 0, L, air=lambda a, b: 0.0)
    assert zs == PILLAR
    assert PILLAR.segment(13.5) == 14.0


def test_the_columns_faces_are_the_plates_it_reaches():
    assert StandoffAxle.faces([3, 5], 12, (True, True)) == (0, 12)
    assert StandoffAxle.faces([3, 5], 12, (True, False)) == (0, 6)      # over its last link
    assert StandoffAxle.faces([3, 5], 12, (False, True)) == (2, 12)


def test_take_up_shims_are_whole_mm_then_half_steps():
    """Rounded to the 0.5 mm step (DIN 433 washers): 1.7 -> 1.5 = 1 + 0.5; 0.24 -> none;
    0.26 -> 0.5."""
    assert SHIM_STEP == 0.5
    assert COLUMN_TOL == 0.25
    assert PILLAR.shims(1.7) == [1.0, 0.5]
    assert PILLAR.shims(0.24) == []
    assert PILLAR.shims(0.26) == [0.5]
    assert SHAFT.shims(2.0) == [1.0, 1.0]


def test_the_shaft_is_its_gaps_length_up_to_a_tenth_over():
    """An M3 shaft: the least residual after whole 0.5 mm shims, then the nearest: a 13.3 mm
    gap takes the 13.3 mm shaft (12.0 with 1.3 of shims would leave 0.2 over the 1.5 of
    shims, inside the 0.25 tolerance, but farther); at most 0.1 over its gap: an 11.9 gap
    takes the 12 mm minimum, an 11.8 one nothing."""
    assert SHAFT.segment(13.3, shims=True) == 13.3
    assert SHAFT.segment(11.9) == 12.0
    assert SHAFT.segment(11.8) is None


def test_a_standoff_is_a_pillar_only():
    with pytest.raises(ConstructionError, match="a standoff is a pillar only"):
        PILLAR.dims(_ctx.context(), False)
