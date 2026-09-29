"""The drive group: a servo on the inner frame plate, and what the crank couples to.

The servo sits on the inner frame plate's top face (the face away from the
legs), output face down, output axis on the crank axis O, its long side
along a direction ``u`` in the plate. Its horn turns in a clearance hole in
the plate and couples to the crank below (:mod:`construction.crank`).

Coordinates: see :mod:`servos.spec` for the servo frame. :func:`servo_to_world`
maps it to side coordinates (servo +z = world -z).
"""

from __future__ import annotations

import math

import numpy as np

from construction.base import FRAME_INNER, Build, Context, DriveInterface, Realized, hardware
from hardware.bom import BomLine
from servos.spec import ServoSpec
from shapes import Cut, Rect, box, disc
from stack import Claim

SCREW_HEAD_D = {"M2": 3.8, "M2.5": 4.5, "M3": 5.5}   # socket head diameters


def servo_to_world(center, u, face_z: float) -> np.ndarray:
    """4x4 transform from the servo frame to side coordinates.

    ``center``: world XY of the output axis; ``u``: world XY direction of
    servo +x; ``face_z``: world Z of the servo frame's origin (z = 0 face).
    Servo +z points down (world -z), so servo +y is ``-(Z x u)``.
    """
    ux, uy = np.asarray(u, dtype=float) / np.linalg.norm(u)
    m = np.eye(4)
    m[:3, 0] = (ux, uy, 0.0)
    m[:3, 1] = (uy, -ux, 0.0)
    m[:3, 2] = (0.0, 0.0, -1.0)
    m[:3, 3] = (center[0], center[1], face_z)
    return m


class DriveGroup:
    """One servo on the inner frame plate."""

    name = "drive"

    def __init__(self, spec: ServoSpec):
        self.spec = spec

    def interface(self, ctx: Context) -> DriveInterface:
        s, h = self.spec, self.spec.horn
        thread = h.pattern.thread or "M3"
        return DriveInterface(
            horn_face_depth=s.horn_face_depth,
            horn_radius=h.diameter / 2,
            horn_thickness=h.thickness,
            screw_pcd=h.pattern.pcd,
            screw_count=h.pattern.count,
            screw_clearance_d=h.pattern.hole_d,
            screw_head_d=SCREW_HEAD_D.get(thread, 5.5) + 0.5,
            screw_key=h.pattern.screw,
            center_head_d=h.center_screw_head_d,
            center_head_h=3.0,
            pattern_angle=math.radians(h.pattern.angle_deg),
        )

    def claims(self, ctx: Context) -> list[Claim]:
        return []   # the servo sits outside the stack, above the inner plate

    # -- build ------------------------------------------------------------------

    def direction(self, build: Build) -> np.ndarray:
        """Servo +x in the plate: away from the frame pillars."""
        topo = build.plan.topo
        o = build.xy("O")
        away = -sum((build.xy(a.name) - o for a in topo.axes_of("frame")), np.zeros(2))
        n = np.linalg.norm(away)
        return away / n if n > 1e-9 else np.array([1.0, 0.0])

    def realize(self, build: Build) -> Realized:
        s = self.spec
        out = Realized()
        o = build.xy("O")
        u = self.direction(build)
        v = np.array([u[1], -u[0]])          # servo +y in the world (see servo_to_world)
        ang = math.atan2(u[1], u[0])
        plate_top = build.z(build.top)[1]

        def world(x, y):
            return tuple(o + x * u + y * v)

        L, W, H = s.body
        # plate: horn clearance, screw holes, reliefs; a pad under the whole footprint
        out.cut(FRAME_INNER, Cut(tuple(o), s.horn.diameter + 2 * build.ctx.params.margin))
        for mh in s.mount:
            out.cut(FRAME_INNER, Cut(world(mh.x, mh.y), mh.d))
        for r in s.front_reliefs:
            out.cut(FRAME_INNER, Rect(world((r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2),
                                      (r.x1 - r.x0 + 0.5, r.y1 - r.y0 + 0.5), ang))
        x0, x1 = s.axis_offset - L / 2, s.axis_offset + L / 2
        half = W / 2 + build.ctx.params.min_wall
        out.pad(FRAME_INNER, world(x0 + half, 0), world(x1 - half, 0), half * math.sqrt(2))
        # servo body (parametric placeholder) and its horn
        crank_host = next(iter(build.plan.topo.crank_bodies), None)
        frame_host = build.plan.topo.frame_bodies[0]
        body = box(world(s.axis_offset, 0), (L, W, H), plate_top, ang)
        if s.mount_face_z > 0:     # the face around the shaft is recessed; the horn sits in it
            body = body - disc(tuple(o), s.horn.diameter / 2 + 0.5, plate_top - 1,
                               plate_top + s.mount_face_z)
        out.bodies.append(hardware("servo", body, frame_host, fab="purchased",
                                   bom_key=s.bom_key, color="#1f1f1f"))
        face = plate_top - s.horn_face_depth
        out.bodies.append(hardware(
            "servo_horn", disc(tuple(o), s.horn.diameter / 2, face, face + s.horn.thickness),
            crank_host or frame_host, fab="purchased", color="#c0c0c0"))
        screws = [mh.screw for mh in s.mount if mh.screw]
        for key in sorted(set(screws)):
            out.extras.append(BomLine(key, screws.count(key), "servo to inner frame plate"))
        return out


__all__ = ["DriveGroup", "servo_to_world"]
