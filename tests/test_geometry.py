"""Reference-value tests for :mod:`klann`.

These values were captured once from the pre-port numerical output. If the
symbolic chain or constants change, expect these to drift and update
deliberately.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import sympy as sp

from klann import STEPS, circle_x_circle, create_klann_geometry

EXPECTED_FX_PHASE_1 = 224.87767188289433
EXPECTED_FY_PHASE_1 = -93.54456977054052


def test_foot_tip_reference_value_phase_1():
    sol = create_klann_geometry(orientation=1, phase=0.0)
    fx, fy = sol.joints_at(1.0)["F"]
    assert fx == pytest.approx(EXPECTED_FX_PHASE_1, abs=1e-9)
    assert fy == pytest.approx(EXPECTED_FY_PHASE_1, abs=1e-9)


def test_foot_path_closure_over_full_cycle():
    sol = create_klann_geometry(orientation=1, phase=0.0)
    fx0, fy0 = sol.joints_at(0.0)["F"]
    fx2, fy2 = sol.joints_at(2 * math.pi)["F"]
    assert fx0 == pytest.approx(fx2, abs=1e-9)
    assert fy0 == pytest.approx(fy2, abs=1e-9)


def test_fixed_pivot_O_is_origin():
    sol = create_klann_geometry(orientation=1, phase=0.7)
    ox, oy = sol.joints_at(0.0)["O"]
    assert ox == pytest.approx(0.0, abs=1e-12)
    assert oy == pytest.approx(0.0, abs=1e-12)


def test_orientation_flips_foot_x_sign():
    sol_r = create_klann_geometry(orientation=1, phase=0.0)
    sol_l = create_klann_geometry(orientation=-1, phase=0.0)
    fx_r, _ = sol_r.joints_at(1.0)["F"]
    fx_l, _ = sol_l.joints_at(1.0)["F"]
    assert fx_r > 0
    assert fx_l < 0


def test_circle_x_circle_two_unit_circles_is_exact():
    c1, c2 = sp.Matrix([0, 0]), sp.Matrix([1, 0])
    up = circle_x_circle(c1, 1, c2, 1, +1)
    down = circle_x_circle(c1, 1, c2, 1, -1)
    assert list(up) == [sp.Rational(1, 2), sp.sqrt(3) / 2]
    assert list(down) == [sp.Rational(1, 2), -sp.sqrt(3) / 2]


def test_program_steps_stay_small():
    """Each step is a short expression over earlier points' symbols, not a
    substituted tree (the old chain grew F to ~34k ops)."""
    for name, expr in STEPS:
        ops = sum(sp.count_ops(c) for c in expr)
        assert ops < 200, f"step {name} has {ops} ops"


def test_phase_is_a_time_shift():
    base = create_klann_geometry(orientation=-1, phase=0.0)
    shifted = create_klann_geometry(orientation=-1, phase=0.9)
    ts = np.linspace(0.0, 2.0 * math.pi, 50)
    np.testing.assert_allclose(
        shifted.evaluate(ts)["F"], base.evaluate(ts + 0.9)["F"], atol=1e-12
    )


def test_link_lengths_are_rigid_over_the_cycle():
    sol = create_klann_geometry(orientation=1, phase=0.0)
    xy = sol.evaluate(np.linspace(0.0, 2.0 * math.pi, 720))
    for a, b in [("O", "M"), ("M", "C"), ("M", "D"), ("A", "C"), ("B", "E"),
                 ("E", "D"), ("E", "F")]:
        d = np.linalg.norm(xy[a] - xy[b], axis=-1)
        assert np.ptp(d) < 1e-9, f"|{a}{b}| drifts by {np.ptp(d)}"
