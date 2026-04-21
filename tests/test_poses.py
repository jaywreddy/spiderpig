"""Tests for viewer.poses (Kabsch + per-body pose computation)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

# viewer/ is a sibling of the package modules; make it importable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))

from poses import body_pose_for_phase, kabsch_2d  # noqa: E402


def _apply_se2(r: np.ndarray, t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ r.T + t


def test_kabsch_recovers_known_transform():
    rng = np.random.default_rng(0)
    ref = rng.standard_normal((4, 2))
    theta = 0.6
    r_true = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    t_true = np.array([3.5, -1.2])
    new = _apply_se2(r_true, t_true, ref)

    r, t = kabsch_2d(ref, new)
    np.testing.assert_allclose(r, r_true, atol=1e-12)
    np.testing.assert_allclose(t, t_true, atol=1e-12)


def test_kabsch_proper_rotation_no_reflection():
    # Same point set in both frames -> identity, never a reflection.
    ref = np.array([[0.0, 0.0], [1.0, 0.0]])
    new = ref.copy()
    r, t = kabsch_2d(ref, new)
    assert np.linalg.det(r) > 0.999
    np.testing.assert_allclose(r, np.eye(2), atol=1e-12)
    np.testing.assert_allclose(t, [0.0, 0.0], atol=1e-12)


def test_torso_pose_is_identity_at_every_phase():
    ref = {"A": (1.0, 2.0), "O": (0.0, 0.0), "B": (5.0, 5.0)}
    for p in [0.0, 0.5, 1.7, 2 * math.pi]:
        # Even with totally different "current" positions, torso should pin.
        new = {"A": (10.0, 20.0), "O": (3.0, 3.0), "B": (50.0, 50.0)}
        m = body_pose_for_phase("torso", ref, new)
        np.testing.assert_allclose(m, np.eye(4), atol=1e-12)


def test_body_pose_reproduces_new_joint_positions():
    """Applying the returned pose to ref joints must land on new joints."""
    # Use b2 (joints B, E) with a known rotation+translation.
    ref = {"B": (10.0, 0.0), "E": (40.0, 0.0)}
    theta = 0.4
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    rot = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    trans = np.array([5.0, -3.0])

    def _xform(pt):
        v = np.array(pt)
        return tuple((rot @ v + trans).tolist())

    new = {"B": _xform(ref["B"]), "E": _xform(ref["E"])}

    m = body_pose_for_phase("b2", ref, new)
    for j in ("B", "E"):
        ref_v = np.array([*ref[j], 0.0, 1.0])
        out = m @ ref_v
        np.testing.assert_allclose(out[:2], new[j], atol=1e-9)


def test_real_klann_b1_pose_closes_joints():
    """End-to-end: solve klann at two phases, recover b1 pose, joints align."""
    from poses import REFERENCE_PHASE, _solve_joint_positions

    ref_joints, _ = _solve_joint_positions(REFERENCE_PHASE)
    new_joints, _ = _solve_joint_positions(0.7)

    m = body_pose_for_phase("b1", ref_joints, new_joints)
    for j in ("M", "C", "D"):
        ref_v = np.array([*ref_joints[j], 0.0, 1.0])
        out = m @ ref_v
        np.testing.assert_allclose(out[:2], new_joints[j], atol=1e-9)
