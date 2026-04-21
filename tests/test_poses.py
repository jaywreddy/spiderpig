"""Tests for viewer.poses (per-phase bake via Mechanism.solved())."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

# viewer/ is a sibling of the package modules; make it importable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))

from poses import bake_frames  # noqa: E402

from klann import solve_klann_at_phase  # noqa: E402
from shapes import THICKNESS  # noqa: E402


def _pose_from_flat(flat: list[float]) -> np.ndarray:
    """Unpack a column-major 16-vector back into a 4×4 row-major matrix."""
    col_major = np.array(flat, dtype=float).reshape(4, 4)
    return col_major.T


def test_bake_frames_shape():
    data = bake_frames(n_frames=8, fps=30)
    assert data["fps"] == 30
    assert data["n_frames"] == 8
    assert len(data["frames"]) == 8
    assert len(data["foot_path"]) == 8
    assert set(data["body_names"]) == {"torso", "coupler", "conn", "b1", "b2", "b3", "b4"}
    for f in data["frames"]:
        for name in data["body_names"]:
            assert len(f["poses"][name]) == 16


def test_bake_frames_z_layered():
    """b1 and b4 must sit at distinct Z (one THICKNESS apart) — layering preserved."""
    data = bake_frames(n_frames=4, fps=30)
    m_b1 = _pose_from_flat(data["frames"][0]["poses"]["b1"])
    m_b4 = _pose_from_flat(data["frames"][0]["poses"]["b4"])
    z_b1 = m_b1[2, 3]
    z_b4 = m_b4[2, 3]
    assert abs(z_b1 - z_b4) > 1e-6, "b1 and b4 ended up at the same Z"
    assert abs(abs(z_b1 - z_b4) - THICKNESS) < 1e-6


def test_bake_frames_joint_snap():
    """Shared joint C coincides in 3D between b1 (owns neg_C) and b3 (owns pos_C).

    b1 and b3 live at different body-world Z (one THICKNESS apart) precisely
    because their local copies of joint C sit on opposite layers. Composing
    them must land the pin at the same world point.
    """
    data = bake_frames(n_frames=4, fps=30)
    mech = solve_klann_at_phase(float(data["frames"][0]["phase"]), with_parts=False).solved()

    b1 = mech.body("b1")
    b3 = mech.body("b3")
    c_local_b1 = b1.joint("C").pose.matrix[:, 3]
    c_local_b3 = b3.joint("C").pose.matrix[:, 3]

    m_b1 = _pose_from_flat(data["frames"][0]["poses"]["b1"])
    m_b3 = _pose_from_flat(data["frames"][0]["poses"]["b3"])
    c_world_b1 = m_b1 @ c_local_b1
    c_world_b3 = m_b3 @ c_local_b3

    np.testing.assert_allclose(c_world_b1, c_world_b3, atol=1e-9)
    # Sanity: body Zs really do differ by one layer (the reason local Cs differ).
    assert abs(abs(m_b1[2, 3] - m_b3[2, 3]) - THICKNESS) < 1e-6


def test_bake_frames_matches_direct_solve():
    """Every baked pose equals a fresh ``solve_klann_at_phase(p).solved()`` call."""
    data = bake_frames(n_frames=6, fps=30)
    for frame in data["frames"]:
        mech = solve_klann_at_phase(float(frame["phase"]), with_parts=False).solved()
        for name in data["body_names"]:
            baked = _pose_from_flat(frame["poses"][name])
            np.testing.assert_allclose(
                baked, mech.body(name).pose.matrix, atol=1e-9, err_msg=name
            )
