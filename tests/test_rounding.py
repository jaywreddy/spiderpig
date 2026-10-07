"""Tests for :mod:`spiderpig.rounding`: report numbers don't flip on a rounding tie."""

from __future__ import annotations

import math

import pytest

from spiderpig.rounding import fixed, rounded


@pytest.mark.parametrize("x", [3.925, 3.9249999999999, 3.9250000000001,
                               math.nextafter(3.925, 0), math.nextafter(3.925, 9)])
def test_a_tie_rounds_the_same_from_either_side(x):
    """R.b4_leg0's hole (klann quad) is 3.925 mm from its edge: BRepExtrema gives a hair
    under or over it with OCCT's thread count; the report says 3.93 either way."""
    assert rounded(x, 2) == 3.93
    assert fixed(x, 2) == "3.93"


def test_ties_go_half_away_from_zero():
    assert rounded(2.675, 2) == 2.68            # (round() says 2.67: 2.67499999...)
    assert rounded(-2.675, 2) == -2.68
    assert rounded(0.125, 2) == 0.13
    assert rounded(1.5, 0) == 2.0


def test_off_a_tie_it_is_round():
    for x in (3.92, 3.931, 3.9249, 0.0, -1.234567, 12345.678):
        for digits in (0, 1, 2, 3):
            assert rounded(x, digits) == round(x, digits)
    assert rounded(3.123456789, 7) == round(3.123456789, 7)
    assert rounded(4.6e-6, 5) == 0.0            # (settled past the digits kept, not at 1e-6)
    assert rounded(1.4e-5, 5) == 1e-5
    assert fixed(1.0, 3) == "1.000"
    assert math.isnan(rounded(float("nan"), 2))
