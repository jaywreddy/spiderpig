"""Reference-value tests for :mod:`klann`.

These values were captured once from the pre-port numerical output. If the
symbolic chain or constants change, expect these to drift and update
deliberately.
"""

from __future__ import annotations

import math

import pytest

from klann import create_klann_geometry, custom_intersection


EXPECTED_FX_PHASE_1 = 224.87767188289433
EXPECTED_FY_PHASE_1 = -93.54456977054052


def _xy(pt):
    return float(pt.x.evalf()), float(pt.y.evalf())


def test_foot_tip_reference_value_phase_1():
    geo = create_klann_geometry(orientation=1, phase=1)
    fx, fy = _xy(geo[6])
    assert fx == pytest.approx(EXPECTED_FX_PHASE_1, abs=1e-9)
    assert fy == pytest.approx(EXPECTED_FY_PHASE_1, abs=1e-9)


def test_foot_path_closure_over_full_cycle():
    fx0, fy0 = _xy(create_klann_geometry(orientation=1, phase=0.0)[6])
    fx2, fy2 = _xy(create_klann_geometry(orientation=1, phase=2 * math.pi)[6])
    assert fx0 == pytest.approx(fx2, abs=1e-9)
    assert fy0 == pytest.approx(fy2, abs=1e-9)


def test_fixed_pivot_O_is_origin():
    geo = create_klann_geometry(orientation=1, phase=0.7)
    ox, oy = _xy(geo[0])
    assert ox == pytest.approx(0.0, abs=1e-12)
    assert oy == pytest.approx(0.0, abs=1e-12)


def test_orientation_flips_foot_x_sign():
    geo_r = create_klann_geometry(orientation=1, phase=1.0)
    geo_l = create_klann_geometry(orientation=-1, phase=1.0)
    fx_r, _ = _xy(geo_r[6])
    fx_l, _ = _xy(geo_l[6])
    assert fx_r > 0
    assert fx_l < 0


def test_custom_intersection_two_unit_circles():
    from sympy.geometry import Circle, Point

    c1 = Circle(Point(0, 0), 1)
    c2 = Circle(Point(1, 0), 1)
    pts = custom_intersection(c1, c2)
    assert len(pts) == 2
    xs = sorted(float(p.x.evalf()) for p in pts)
    ys = sorted(float(p.y.evalf()) for p in pts)
    # intersections at (1/2, +/- sqrt(3)/2)
    assert xs[0] == pytest.approx(0.5, abs=1e-12)
    assert xs[1] == pytest.approx(0.5, abs=1e-12)
    assert ys[0] == pytest.approx(-math.sqrt(3) / 2, abs=1e-12)
    assert ys[1] == pytest.approx(math.sqrt(3) / 2, abs=1e-12)
