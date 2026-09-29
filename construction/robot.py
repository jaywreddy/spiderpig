"""The robot: two mirror-image sides with their servos back to back in one frame.

In side coordinates (:mod:`fabricate`) the outer frame plate starts at
``z = 0``, the inner frame plate is on top and the servo stands on it. The
robot's mid-plane is where the two servos' rear faces meet a stack of
**centre plates** that the servos screw into. The left side ("L.") is its
side moved so that mid-plane is at ``z = 0``; the right side ("R.") is the
mirror image (``z -> -z``). The mechanism is planar, so both sides move
identically in XY; only their parts are mirrored. The servos therefore turn
in opposite senses about their own axes, which drives both sides forward.

Chassis parts (centre plates, their screws) belong to the robot, not a side.
"""

from __future__ import annotations

import math

import numpy as np
from build123d import Location, Plane

from construction.base import Build
from hardware.bom import BomLine
from mechanism import Body, Mechanism, MechanismTemplate
from shapes import Rect, plate

SIDES = ("L", "R")


def prefixed(name: str | None, side: str) -> str | None:
    return None if name is None else f"{side}.{name}"


def robot_template(tmpl: MechanismTemplate) -> MechanismTemplate:
    """Both sides' kinematics: the side template twice, prefixed ``L.`` and ``R.``."""
    from dataclasses import replace

    bodies, connections = [], []
    for side in SIDES:
        bodies += [replace(b, name=prefixed(b.name, side)) for b in tmpl.bodies]
        connections += [((i, prefixed(pb, side), pj), (k, prefixed(cb, side), cj))
                        for (i, pb, pj), (k, cb, cj) in tmpl.connections]
    return MechanismTemplate(name=f"{tmpl.name}_robot", bodies=bodies, connections=connections)


def centre_plates(spec, pitch: float, margin: float) -> int:
    """Centre plates needed so the two servos' rear bumps clear each other."""
    proud = max((r.height for r in spec.rear_reliefs), default=0.0)
    return max(1, math.ceil((2 * proud + margin) / pitch - 1e-9))


def mid_plane(design) -> float:
    """Side-coordinate z of the robot's mid-plane."""
    spec, plan = design.ctx.servo, design.plan
    rear = spec.rear_face_z if spec.rear_face_z is not None else spec.mount_face_z - spec.body[2]
    n = centre_plates(spec, design.ctx.pitch, design.ctx.params.margin)
    return plan.z(plan.top)[1] + (spec.mount_face_z - rear) + n * design.ctx.pitch / 2


def _moved(part, dz: float, mirror: bool):
    if part is None:
        return None
    if mirror:
        part = part.mirror(Plane.XY)
        dz = -dz
    return part.moved(Location((0.0, 0.0, dz)))


def assemble_robot(side: Mechanism, design) -> Mechanism:
    """Both sides of the robot and the frame between their servos."""
    z_mid = mid_plane(design)
    bodies: list[Body] = []
    connections = []
    for s in SIDES:
        mirror = s == "R"
        for b in side.bodies:
            bodies.append(Body(
                name=prefixed(b.name, s), part=_moved(b.part, -z_mid, mirror), joints=b.joints,
                color=b.color, pose=b.pose, outline=b.outline,
                rigid_with=prefixed(b.rigid_with, s), fab=b.fab, bom_key=b.bom_key,
            ))
        connections += [((i, prefixed(pb, s), pj), (k, prefixed(cb, s), cj))
                        for (i, pb, pj), (k, cb, cj) in side.connections]
    chassis, extras = _chassis(side, design)
    bodies += chassis
    meta = dict(side.meta, robot=True, mid_plane=z_mid)
    return Mechanism(
        name=f"{side.name}_robot", bodies=bodies, connections=connections, meta=meta,
        bom_extras=list(side.bom_extras) * 2 + extras,
    )


def _chassis(side: Mechanism, design) -> tuple[list[Body], list[BomLine]]:
    """Centre plates between the servos' rear faces (first pass: plates and reliefs only)."""
    spec, ctx = design.ctx.servo, design.ctx
    build = Build(ctx, design.plan, side)
    o = build.xy("O")
    u = design.drive.direction(build)
    v = np.array([u[1], -u[0]])
    ang = math.atan2(u[1], u[0])
    L, W, _ = spec.body

    def world(x, y):
        return tuple(o + x * u + y * v)

    n = centre_plates(spec, ctx.pitch, ctx.params.margin)
    x0, x1 = spec.axis_offset - L / 2, spec.axis_offset + L / 2
    half = W / 2
    cuts = [Rect(world((r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2),
                 (r.x1 - r.x0 + 1.0, r.y1 - r.y0 + 1.0), ang) for r in spec.rear_reliefs]
    host = prefixed(side.body(design.plan.topo.frame_bodies[0]).name, "L")
    bodies = []
    for k in range(n):
        z0 = (k - n / 2) * ctx.pitch
        part = plate([(world(x0 + half, 0), world(x1 - half, 0), half)], z0, z0 + ctx.pitch, cuts)
        bodies.append(Body(name=f"centre_plate{k}", part=part, rigid_with=host, fab="laser",
                           color="#eb6834"))
    return bodies, []


__all__ = ["SIDES", "assemble_robot", "centre_plates", "mid_plane", "prefixed", "robot_template"]
