"""build123d part factories for the Klann linkage.

Pure geometry: no mechanism wiring, no I/O. Each function returns either a
build123d ``Part`` (single rigid solid) or a :class:`mechanism.Body`
(part + joints + label + colour ready to be placed by :class:`Mechanism`).

The OpenSCAD port honours one important semantics flip: build123d's
``Cylinder`` / ``Box`` are centred on all three axes by default, while the
original SolidPython code used ``cylinder(center=False)`` (bottom at z=0) as
its norm. Rather than sprinkle ``align=`` keyword arguments everywhere, the
private ``_base_cylinder`` helper below places the bottom face at z=0 so the
ported geometry keeps the same z-origin conventions.
"""

from __future__ import annotations

import math

import numpy as np
from build123d import (
    Align,
    Axis,
    Box,
    Color,
    Compound,
    Cylinder,
    Part,
    Pos,
)

from mechanism import Body, Joint, Pose

THICKNESS = 3.0
BUFF = 6.0
HOLE_R = 2.0
CLEVIS_HOLE_H = 20.0


def _centered_cylinder(radius: float, height: float) -> Part:
    """Solid cylinder centred on all three axes (OpenSCAD ``center=True``)."""
    return Cylinder(
        radius=radius,
        height=height,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )


def _base_cylinder(radius: float, height: float) -> Part:
    """Solid cylinder whose bottom face lies on z=0 (OpenSCAD default)."""
    return Cylinder(radius=radius, height=height).move(Pos(0, 0, height / 2))


def _base_box(dx: float, dy: float, dz: float) -> Part:
    """Box centred on x,y with its bottom face on z=0."""
    return Box(dx, dy, dz).move(Pos(0, 0, dz / 2))


def joint_from_point(p, name: str, layer: int, thickness: float = THICKNESS):
    """Evaluate a sympy point into a positive / negative joint pair.

    Returns ``(pos, neg)`` :class:`Joint` objects offset by one layer in z.
    ``pos`` sits at ``z = thickness * layer``, ``neg`` one layer below. Both
    share the base ``name`` (matching the original digifab convention) —
    each body carries only one of the pair, so connection lookups remain
    unambiguous.
    """
    x = float(p.x.evalf())
    y = float(p.y.evalf())
    pos = Joint(name=name, pose=Pose.from_translation([x, y, thickness * layer]))
    neg = Joint(name=name, pose=Pose.from_translation([x, y, thickness * (layer - 1)]))
    return pos, neg


def clevis_neg(
    part: Part,
    joint: Joint,
    hole_r: float = HOLE_R,
    height: float = CLEVIS_HOLE_H,
) -> Part:
    """Subtract a pin-hole at ``joint``'s local pose from ``part``."""
    hole = _centered_cylinder(hole_r, height).moved(joint.pose.to_location())
    return part - hole


def _servo_mount_cutout() -> Part:
    """Tall 4x4 mm cube centred at the origin used to cut the servo slot."""
    return Box(4, 4, 50, align=(Align.CENTER, Align.CENTER, Align.CENTER))


def pill_link(p1, p2, thickness: float = THICKNESS, buff: float = BUFF) -> Part:
    """Constant-thickness pill between two 2D points, extruded in +z.

    The pill is the Minkowski sum of the line segment with a disk of radius
    ``buff``: it's the 2D convex hull of two equal-radius circles. Here we
    build it as a rectangle plus two end caps (a stadium) rather than hitting
    build123d's hull algorithm, which avoids tangent-edge edge cases and
    produces identical geometry for equal-radius endpoints.
    """
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    length = float(np.hypot(dx, dy))
    theta_deg = math.degrees(math.atan2(dy, dx))
    mid = (p1[:2] + p2[:2]) / 2.0

    body = _base_box(length, 2 * buff, thickness).rotate(Axis.Z, theta_deg).move(
        Pos(float(mid[0]), float(mid[1]), 0)
    )
    cap1 = _base_cylinder(buff, thickness).move(Pos(float(p1[0]), float(p1[1]), 0))
    cap2 = _base_cylinder(buff, thickness).move(Pos(float(p2[0]), float(p2[1]), 0))
    return body + cap1 + cap2


def rationalize_segment(seg, joints, name: str, color: str = "green") -> Body:
    """Turn a sympy ``Segment`` + named joints into a laser-cut Body.

    Every joint in ``joints`` becomes a pin-hole in the link. If ``name``
    contains ``"conn"``, a servo-mount square is also cut (the conn body
    carries the motor-shaft keyway).
    """
    p1 = seg.p1
    p2 = seg.p2
    p1xy = (float(p1.x.evalf()), float(p1.y.evalf()))
    p2xy = (float(p2.x.evalf()), float(p2.y.evalf()))

    link = pill_link(p1xy, p2xy)
    for j in joints:
        link = clevis_neg(link, j)
    if "conn" in name:
        link = link - _servo_mount_cutout()

    return Body(name=name, part=link, joints=list(joints), color=color)


def create_shaft_connector() -> Part:
    """Servo-shaft coupler: 10 mm-radius disc with a 4x4x20 mm square key."""
    mount_plate = _base_cylinder(10, 3)
    shaft = Box(4, 4, 20, align=(Align.MIN, Align.MIN, Align.MIN)).move(Pos(-2, -2, 0))
    return mount_plate + shaft


def create_support_bar(joint: Joint, offset: float) -> Part:
    """Cylindrical standoff from the xy plane down to a joint, ending in a clevis pin.

    Positioned so the bar's top sits at z=0 and the pivot pin protrudes below
    by ``offset + 2.5 mm``. Used by the torso to reach down to the A and B
    body-frame pivots.
    """
    pos = joint.pose.matrix[:3, 3]
    # bar: bottom at z = -offset, top at z = 0
    bar = _base_cylinder(4, offset).move(Pos(0, 0, -offset))
    # clevis pin: bottom at z = -(offset + 2.5), top at z = offset + 2.5 + 5.5 - offset = 3
    pin = _base_cylinder(1.9, 5.5).move(Pos(0, 0, -(offset + 2.5)))
    return (bar + pin).move(Pos(float(pos[0]), float(pos[1]), float(pos[2])))


def create_servo_mount() -> Part:
    """Two-rail servo mount sized for the Jacobs Institute model.

    Left and right bars run parallel in y, connected by a transverse bar at
    ``y = -length/2``. Screw holes are drilled through both rails. The whole
    mount sits below z=0.
    """
    width = 6.5
    length = 20.0
    depth = 2.3
    voffset = 18.5
    left_spread = -30.0
    right_spread = 17.0

    def _rail(x_shift: float) -> Part:
        bar = Box(width, length, depth, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        hole = _centered_cylinder(2, 10)
        bar = bar - hole.move(Pos(0, 4, 0))
        bar = bar - hole.move(Pos(0, -4, 0))
        return bar.move(Pos(x_shift, 0, -(depth / 2 + voffset)))

    left_bar = _rail(left_spread)
    right_bar = _rail(right_spread)
    connector_bar = (
        Box(
            (right_spread - left_spread) + width,
            width,
            depth,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        .move(
            Pos(
                (left_spread + right_spread) / 2,
                -(length / 2 + width / 2),
                -(depth / 2 + voffset),
            )
        )
    )
    return left_bar + connector_bar + right_bar


def create_torso(joints: list[Joint], servo: bool = True) -> Part:
    """Torso plate: servo mount + vertical support bars down to each joint.

    Simpler than the SolidPython original (no 3D hull welding the bars to
    the servo mount). The STEP export still shows an unambiguously placed
    assembly with the correct joints, which is what downstream consumers
    need.
    """
    parts: list[Part] = []
    if servo:
        parts.append(create_servo_mount())

    for j in joints:
        # offset chosen so the support bar top sits at z=0 and reaches down
        # to the joint's z location plus a 2.5mm clearance.
        z = float(j.pose.matrix[2, 3])
        offset = max(-z + 2.5, 3.0)
        parts.append(create_support_bar(j, offset))

    return Compound(parts)


def connector(index: int = 0, color: str = "blue") -> Body:
    """3D-printed pin + cap clevis joiner. ``index`` disambiguates the name."""
    base = _base_cylinder(4, 4)
    conn_cyl = _base_cylinder(1.9, 8)
    bottom = base + conn_cyl.move(Pos(0, 0, 4))
    cap = base.move(Pos(0, 0, 4 + 6.4))
    dimple_cap = cap - bottom
    part = bottom + dimple_cap

    pin_joint = Joint("pin", Pose.from_translation([0, 0, 5.2]))
    return Body(
        name=f"coupler{index}",
        part=part,
        joints=[pin_joint],
        color=color,
    )


def standoff(thickness: float, index: int = 0, color: str = "yellow") -> Body:
    """Stackable 3D-printable standoff. ``index`` disambiguates the name."""
    peg_height = 5.5
    clearance = 3.2
    base = _base_cylinder(4, thickness)
    conn_cyl = _base_cylinder(1.9, peg_height)
    b_placed = base.move(Pos(0, 0, -thickness))
    unit = b_placed + conn_cyl - conn_cyl.move(Pos(0, 0, -(clearance + thickness)))

    anchor = Joint("A", Pose.from_translation([0, 0, -100]))
    return Body(
        name=f"standoff{index}",
        part=unit,
        joints=[anchor],
        color=color,
    )
