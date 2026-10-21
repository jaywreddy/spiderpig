"""The robot's kinematic tree beyond its planar side: spatial joints (:mod:`.tree`).

The planar side moves in its own plane (:mod:`spiderpig.mechanism`, :mod:`spiderpig.linkage`);
a robot layout (:data:`construction.robot.LAYOUTS`) may stand that side on joints whose axes
are anywhere (an arm's turntable about the vertical, its wrist's roll). This package holds
those joints as data, the poses they make over time and the rotations' algebra, below
everything that builds or writes them (the bake, the URDF).
"""

from spiderpig.kinematics.tree import (
    RobotTree,
    SpatialJoint,
    planar_matrix,
    quat_from_matrix,
    rigid_about,
    rotation,
)

__all__ = ["RobotTree", "SpatialJoint", "planar_matrix", "quat_from_matrix", "rigid_about",
           "rotation"]
