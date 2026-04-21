"""Solver tests for :class:`mechanism.Mechanism`."""

from __future__ import annotations

import numpy as np

from mechanism import Body, Joint, Mechanism, Pose


def _toy_two_body() -> Mechanism:
    """Two bodies sharing a joint; root has a tip-joint at +x=3, child has
    a base-joint at +x=-1. With joints aligned, the child's origin should sit
    at world +x=4."""
    root = Body(
        name="root",
        joints=[
            Joint("tip", Pose.from_translation([3.0, 0.0, 0.0])),
        ],
    )
    child = Body(
        name="child",
        joints=[
            Joint("base", Pose.from_translation([-1.0, 0.0, 0.0])),
        ],
    )
    return Mechanism(
        name="toy",
        bodies=[root, child],
        connections=[((0, "root", "tip"), (1, "child", "base"))],
    )


def test_solved_translates_child_to_align_joint():
    solved = _toy_two_body().solved()
    root = solved.body("root")
    child = solved.body("child")

    np.testing.assert_allclose(root.pose.matrix, np.eye(4))

    # child origin should be at +x=4.
    np.testing.assert_allclose(child.pose.matrix[:3, 3], [4.0, 0.0, 0.0])

    # joint coincidence: world position of root.tip == world position of child.base
    root_tip_world = root.pose @ root.joint("tip").pose
    child_base_world = child.pose @ child.joint("base").pose
    np.testing.assert_allclose(
        root_tip_world.matrix,
        child_base_world.matrix,
        atol=1e-9,
    )


def test_solved_is_pure_does_not_mutate_input():
    mech = _toy_two_body()
    mech.solved()
    # original child body still at identity
    np.testing.assert_allclose(mech.body("child").pose.matrix, np.eye(4))


def test_solved_handles_chain_of_three():
    a = Body("a", joints=[Joint("t", Pose.from_translation([1.0, 0.0, 0.0]))])
    b = Body(
        "b",
        joints=[
            Joint("b_in", Pose.from_translation([0.0, 0.0, 0.0])),
            Joint("b_out", Pose.from_translation([2.0, 0.0, 0.0])),
        ],
    )
    c = Body("c", joints=[Joint("c_in", Pose.from_translation([0.0, 0.0, 0.0]))])
    mech = Mechanism(
        name="chain",
        bodies=[a, b, c],
        connections=[
            ((0, "a", "t"), (1, "b", "b_in")),
            ((1, "b", "b_out"), (2, "c", "c_in")),
        ],
    )
    solved = mech.solved()
    # a at origin, b.origin at +x=1, c.origin at +x=1+2=3
    np.testing.assert_allclose(solved.body("b").pose.matrix[:3, 3], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(solved.body("c").pose.matrix[:3, 3], [3.0, 0.0, 0.0])
