"""Seam tests of the chassis' frame ties (:func:`construction.chassis.tie_locals`) and the
deck's path notches (:func:`construction.deck.path_notches`) on hand-made inputs: a
synthetic context (``tests/_ctx.py``) or a deck layout and obstacle boxes the test states.
No plan, no fabrication (``no_fabricate``)."""

from __future__ import annotations

import math

import pytest

from spiderpig.construction import chassis
from spiderpig.construction.deck import HALF_LEN, NOTCH_CLEAR, DeckLayout, path_notches
from tests import _ctx

pytestmark = pytest.mark.no_fabricate


def _clear(xy, near, r):
    x, y = xy
    return all(math.hypot(x - hx, y - hy) >= r + hr + web + 0.05 for hx, hy, hr, web in near)


# -- frame ties ---------------------------------------------------------------------------------


def test_the_sts3215s_ties_sit_beside_its_long_sides_near_its_ends():
    """Its body -10.11..35.11 x +-12.36 in the servo frame: each tie 3.8 mm in from an end
    and 1 + 3.8 mm off a side (17.16); the rear pair moved 3.75 mm in along the side (from
    31.31), the first place two thicknesses of the plates off the servo's rear screw holes
    and recesses."""
    ctx = _ctx.context(servo="sts3215")
    assert chassis._footprint(ctx.servo) == (-10.11, 35.11, -12.36, 12.36)
    got = chassis.tie_locals(ctx)
    assert got == [(-6.31, 17.16), (-6.31, -17.16), (27.56, 17.16), (27.56, -17.16)]


@pytest.mark.parametrize("servo", ["sts3215", "xl330_m288", "xl430_w250"])
def test_every_tie_keeps_the_services_web_to_every_hole_it_shares_a_plate_with(servo):
    ctx = _ctx.context(servo=servo)
    near = chassis.tie_neighbours(ctx)
    r = chassis.tie_dims(ctx).hole_d / 2
    for xy in chassis.tie_locals(ctx):
        assert _clear(xy, near, r), (servo, xy)


@pytest.mark.parametrize("servo", ["sts3215", "xl330_m288", "xl430_w250"])
def test_a_tie_moves_the_least_step_that_clears(servo):
    """Each tie's shift along the side is the smallest in the order tried (outward first,
    then inward, in 0.25 mm steps): no smaller shift would clear."""
    ctx = _ctx.context(servo=servo)
    near = chassis.tie_neighbours(ctx)
    r = chassis.tie_dims(ctx).hole_d / 2
    x0, x1, _, _ = chassis._footprint(ctx.servo)
    c = max(3.0, chassis.tie_dims(ctx).head_r, chassis.TIE_PLACE_R)
    for (x, y), home in zip(chassis.tie_locals(ctx), (x0 + c, x0 + c, x1 - c, x1 - c),
                            strict=True):
        shift = round(x - home, 6)
        smaller = [k * chassis.TIE_SHIFT_STEP for k in range(round(abs(shift) / 0.25))]
        for d in smaller:
            for s in (1, -1):
                assert not _clear((home + s * d, y), near, r), (servo, home, s * d)


def test_with_nothing_in_the_way_the_ties_stay_at_their_places(monkeypatch):
    ctx = _ctx.context(servo="xl330_m288")
    monkeypatch.setattr(chassis, "tie_neighbours", lambda ctx: [])
    x0, x1, _, y1 = chassis._footprint(ctx.servo)
    c, yt = chassis.TIE_PLACE_R, y1 + ctx.params.margin + chassis.TIE_PLACE_R
    assert chassis.tie_locals(ctx) == [(x0 + c, yt), (x0 + c, -yt), (x1 - c, yt), (x1 - c, -yt)]


def test_a_tie_no_shift_clears_stays_unmoved_for_the_audit_to_warn(monkeypatch):
    """A hole wider than the whole shift range at every place: none clears within
    ``TIE_SHIFT_MAX``, so each tie keeps its place."""
    ctx = _ctx.context(servo="xl330_m288")
    x0, x1, _, y1 = chassis._footprint(ctx.servo)
    c, yt = chassis.TIE_PLACE_R, y1 + ctx.params.margin + chassis.TIE_PLACE_R
    big = [(x, s * yt, chassis.TIE_SHIFT_MAX + 2, 0.0) for x in (x0 + c, x1 - c) for s in (1, -1)]
    monkeypatch.setattr(chassis, "tie_neighbours", lambda ctx: big)
    assert chassis.tie_locals(ctx) == [(x0 + c, yt), (x0 + c, -yt), (x1 - c, yt), (x1 - c, -yt)]


def test_the_neighbours_are_the_front_holes_and_both_servos_rear_ones():
    """The STS3215: its four front screw holes in the inner plate (0.080 in: a 2 x 2.032 mm
    web), and its two rear holes' head recesses mirrored for the other servo in the centre
    plates (0.090 in: 2 x 2.286)."""
    ctx = _ctx.context(servo="sts3215")
    near = chassis.tie_neighbours(ctx)
    assert len(near) == 8
    assert {round(w, 3) for *_, w in near[:4]} == {4.064}
    assert {round(w, 3) for *_, w in near[4:]} == {4.572}
    assert sorted(y for _, y, _, _ in near[4:]) == [-10.25, -10.25, 10.25, 10.25]


@pytest.mark.parametrize(("servo", "sheet"), [("sts3215", "al5052_2p3mm"),
                                              ("xl330_m288", "al5052_2p5mm")])
def test_the_centre_plates_take_the_thinnest_sheet_that_seats_the_most_rear_screws(servo,
                                                                                    sheet):
    assert chassis.centre_sheet(_ctx.context(servo=servo)) == sheet


# -- the deck's path notches ----------------------------------------------------------------------


LAY = DeckLayout(x_c=0.0, rail_y0=10.0, deck_y=20.0, pitch=3.0, z_in=-40.0, z_leg=-42.0,
                 spigot_x=0.0)
"""A deck plate x -68..68, z -39..39 (1 mm inside each inner plate), its underside at y 20."""


def test_the_deck_plate_is_its_half_length_and_inside_the_inner_plates():
    assert LAY.half_w == pytest.approx(39.0)
    assert (HALF_LEN, NOTCH_CLEAR) == (68.0, 0.5)


def test_an_obstacle_beside_the_plate_or_under_it_needs_no_notch():
    """Beyond the plate's x (-80..-75), and wholly under its underside (y1 18 <= 20)."""
    assert path_notches(LAY, [(-80, -75, 15, 25, -38, -30), (0, 5, 5, 18, 0, 5)]) == []


def test_a_notch_opens_to_the_nearer_side_edge():
    """Grown by the 0.5 mm clearance; on the -z side it runs out past -39 (to -40), on the
    +z side to +40."""
    assert path_notches(LAY, [(-30, -25, 15, 25, -38, -30)]) == [(-30.5, -24.5, -40.0, -29.5)]
    assert path_notches(LAY, [(0, 5, 15, 25, 30, 35)]) == [(-0.5, 5.5, 29.5, 40.0)]


def test_a_centred_obstacle_opens_to_the_plus_z_side():
    assert path_notches(LAY, [(0, 5, 15, 25, -2, 2)]) == [(-0.5, 5.5, -2.5, 40.0)]


def test_a_notch_within_a_thickness_of_an_end_opens_through_it():
    """Within the plate's thickness (3 mm) of an end, no sliver is left: the notch runs out
    past that end (x1 + 1, x0 - 1)."""
    assert path_notches(LAY, [(66, 70, 15, 25, -38, -30)]) == [(65.5, 69.0, -40.0, -29.5)]
    assert path_notches(LAY, [(-66, -62, 15, 25, -38, -30)]) == [(-69.0, -61.5, -40.0, -29.5)]


def test_one_notch_per_obstacle_in_order():
    obs = [(-30, -25, 15, 25, -38, -30), (0, 5, 15, 25, 30, 35)]
    assert len(path_notches(LAY, obs)) == 2
    assert path_notches(LAY, obs)[1][0] == -0.5
