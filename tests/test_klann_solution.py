"""Tests for the symbolic KlannSolution + multi-leg builder."""

from __future__ import annotations

import math

import numpy as np
import pytest

from klann import (
    KlannSolution,
    build_multi_leg_mechanism,
    create_klann_geometry,
)


def test_solution_time_driven():
    """Evaluating the solution at t=1.0 reproduces the reference foot tip."""
    sol = create_klann_geometry(orientation=1, phase=0.0)
    assert isinstance(sol, KlannSolution)
    fx, fy = sol.joints_at(1.0)["F"]
    assert fx == pytest.approx(224.87767188289433, abs=1e-9)
    assert fy == pytest.approx(-93.54456977054052, abs=1e-9)


def test_solution_points_and_segments_present():
    sol = create_klann_geometry(orientation=1, phase=0.0)
    assert set(sol.points) == {"O", "A", "B", "C", "D", "E", "F", "M"}
    assert set(sol.segments) == {"b1", "b2", "b3", "b4", "conn"}


@pytest.mark.parametrize("t_value", [0.0, 0.7, 1.5, math.pi, 2.0 * math.pi - 0.1])
def test_solution_phase_offset_cancels_time_shift(t_value: float):
    """The same crank angle must land M at the same spot regardless of whether
    the offset is baked into ``phase`` or added to ``t``."""
    dphi = 1.23
    sol_a = create_klann_geometry(orientation=1, phase=0.0)
    sol_b = create_klann_geometry(orientation=1, phase=dphi)

    a = sol_a.joints_at(t_value + dphi)["M"]
    b = sol_b.joints_at(t_value)["M"]
    assert a[0] == pytest.approx(b[0], abs=1e-9)
    assert a[1] == pytest.approx(b[1], abs=1e-9)


def test_multi_leg_build_bodies_and_z_stride():
    n_legs = 8
    thickness = 3.0
    mech = build_multi_leg_mechanism(
        n_legs=n_legs, t=1.0, thickness=thickness, with_parts=False
    )
    # 7 bodies per leg: torso, coupler, conn, b1, b2, b3, b4.
    assert len(mech.bodies) == n_legs * 7

    # Connections: 8 per leg.
    assert len(mech.connections) == n_legs * 8

    solved = mech.solved()
    z_stride = 3.0 * thickness
    for k in range(n_legs):
        z = solved.body(f"torso_leg{k}").pose.matrix[2, 3]
        assert z == pytest.approx(k * z_stride, abs=1e-9)


def test_multi_leg_phase_spacing():
    """Leg k's foot must match leg 0's foot at a time shifted by -2π·k/n_legs."""
    n_legs = 4
    sols = [
        create_klann_geometry(orientation=1, phase=2.0 * math.pi * k / n_legs)
        for k in range(n_legs)
    ]
    t_world = 0.5
    for k in range(n_legs):
        f_leg_k = sols[k].joints_at(t_world)["F"]
        f_leg_0_shifted = sols[0].joints_at(t_world + 2.0 * math.pi * k / n_legs)["F"]
        np.testing.assert_allclose(f_leg_k, f_leg_0_shifted, atol=1e-9)


def test_multi_leg_joint_snap_holds_per_leg():
    """Each leg's connections must close in world space after solve()."""
    mech = build_multi_leg_mechanism(
        n_legs=3, t=0.7, thickness=3.0, with_parts=False
    )
    solved = mech.solved()
    for (_, a_name, a_joint), (_, b_name, b_joint) in solved.connections:
        a = solved.body(a_name)
        b = solved.body(b_name)
        pa = (a.pose @ a.joint(a_joint).pose).matrix
        pb = (b.pose @ b.joint(b_joint).pose).matrix
        np.testing.assert_allclose(
            pa[:3, 3], pb[:3, 3], atol=1e-6,
            err_msg=f"{a_name}.{a_joint} ≠ {b_name}.{b_joint}",
        )
