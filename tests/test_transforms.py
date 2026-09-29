"""Tests for :class:`mechanism.Pose`."""

from __future__ import annotations

import numpy as np
import pytest

from mechanism import Pose


def _rz(theta: float, xyz=(0.0, 0.0, 0.0)) -> Pose:
    c, s = np.cos(theta), np.sin(theta)
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[:3, 3] = xyz
    return Pose(m)


def test_identity_is_4x4_eye():
    p = Pose.identity()
    assert p.matrix.shape == (4, 4)
    np.testing.assert_allclose(p.matrix, np.eye(4))


def test_from_translation_sets_xyz_column():
    p = Pose.from_translation([1.0, 2.0, 3.0])
    np.testing.assert_allclose(p.matrix[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(p.matrix[:3, :3], np.eye(3))


def test_compose_identity_is_noop():
    p = Pose.from_translation([4.0, -2.0, 7.0])
    np.testing.assert_allclose((p @ Pose.identity()).matrix, p.matrix)
    np.testing.assert_allclose((Pose.identity() @ p).matrix, p.matrix)


def test_compose_is_matmul_of_matrices():
    a = _rz(0.3, [1.0, 0.0, 0.0])
    b = Pose.from_translation([0.0, 2.0, 0.0])
    c = a @ b
    # a @ b means: first apply b, then a (convention: T_ac = T_ab @ T_bc)
    np.testing.assert_allclose(c.matrix, a.matrix @ b.matrix)
    np.testing.assert_allclose(c.matrix[:3, 3], [1.0 - 2.0 * np.sin(0.3), 2.0 * np.cos(0.3), 0.0])


def test_inverse_cancels_self():
    p = _rz(0.6, [1.0, 2.0, 3.0])
    np.testing.assert_allclose((p @ p.inverse()).matrix, np.eye(4), atol=1e-12)
    np.testing.assert_allclose(p.inverse().matrix, np.linalg.inv(p.matrix), atol=1e-12)


def test_rejects_wrong_shape():
    with pytest.raises(ValueError, match="4x4"):
        Pose(np.eye(3))
