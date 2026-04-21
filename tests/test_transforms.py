"""Round-trip tests for :class:`mechanism.Pose`."""

from __future__ import annotations

import numpy as np
import pytest

from mechanism import Pose


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
    a = Pose.from_translation([1.0, 0.0, 0.0])
    b = Pose.from_translation([0.0, 2.0, 0.0])
    c = a @ b
    # a @ b means: first apply b, then a (convention: T_ac = T_ab @ T_bc)
    np.testing.assert_allclose(c.matrix, a.matrix @ b.matrix)
    np.testing.assert_allclose(c.matrix[:3, 3], [1.0, 2.0, 0.0])


def test_inverse_cancels_self():
    p = Pose.from_xyz_quat_xyzw([1.0, 2.0, 3.0], [0.0, 0.0, np.sin(0.3), np.cos(0.3)])
    ident = p @ p.inverse()
    np.testing.assert_allclose(ident.matrix, np.eye(4), atol=1e-12)


def test_xyz_quat_xyzw_identity_quat():
    # xyzw = (0,0,0,1) is identity rotation.
    p = Pose.from_xyz_quat_xyzw([5.0, 6.0, 7.0], [0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(p.matrix[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(p.matrix[:3, 3], [5.0, 6.0, 7.0])


def test_xyz_quat_xyzw_z_180():
    # digifab Z_JOINT_POSE uses (0,0,1,0) in xyzw, i.e. 180 deg about Z.
    p = Pose.from_xyz_quat_xyzw([0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0])
    expected = np.diag([-1.0, -1.0, 1.0, 1.0])
    np.testing.assert_allclose(p.matrix, expected, atol=1e-12)


def test_rejects_wrong_shape():
    with pytest.raises(ValueError):
        Pose(np.eye(3))
