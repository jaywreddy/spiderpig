"""The drive group: a servo on the inner frame plate, and what the crank couples to.

The servo sits on the inner frame plate's top face (the face away from the
legs), output face down, output axis on the crank axis O, its long side
along a direction ``u`` in the plate. Its horn turns in a clearance hole in
the plate and couples to the crank below (:mod:`construction.crank`).

Where the horn's outer face ends up inside the plate (a horn recessed into
its case, like the XL430's), a printed **horn spacer** under the horn brings
the coupling face down to the plate's bottom face; the crank's horn screws
pass through it. The horn is turned on its spline so its holes sit between
the crank webs (:meth:`DriveGroup.pattern_angle`).

The plate gets the horn clearance hole, the front mounting-screw holes and a
cut-out under every raised region of the servo's front face; the frame plate
grows a pad under the servo's footprint.

**Mounting screws** go up through the plate into the servo's front face, so
their heads sit under the plate, inside the leg stack. Only the front holes
whose heads clear the crank hub are used (:meth:`DriveGroup.front_screws`;
in the robot the rear face is screwed to the centre plates as well), and the
drive group claims their heads: fixed points ``servo.screw<i>`` are added to
the side's geometry, with a disc per head in the layer(s) under the plate
and a seat through the plate.

Coordinates: see :mod:`servos.spec` for the servo frame. :func:`servo_to_world`
maps it to side coordinates (servo +z = world -z).
"""

from __future__ import annotations

import math
from functools import lru_cache

import numpy as np
from build123d import Box, Cylinder, Location
from OCP.gp import gp_Trsf

from construction.base import FRAME_INNER, Build, Context, DriveInterface, Realized, hardware
from construction.crank import ScrewKind, screw_body, screw_from_key
from servos.model import cut_each, horn_part, servo_part
from servos.spec import MountHole, ServoSpec
from shapes import Cut, Rect, disc
from stack import Claim, Disc, Layout, Placed

SCREW_HEAD_D = {"M2": 3.8, "M2.5": 4.5, "M3": 5.5}   # ISO 4762 socket head diameters
MIN_SPACER = 1.0          # thinnest printed horn spacer worth making (mm)
SERVO_COLOR = "#1f1f1f"
HORN_COLOR = "#c0c0c0"
SPACER_COLOR = "#6a4fc7"
STEEL = "#4a4a4a"
GROUP = "drive"


def servo_to_world(center, u, face_z: float) -> np.ndarray:
    """4x4 transform from the servo frame to side coordinates.

    ``center``: world XY of the output axis; ``u``: world XY direction of
    servo +x; ``face_z``: world Z of the servo frame's origin (z = 0 face).
    Servo +z points down (world -z), so servo +y is ``-(Z x u)``. A proper
    rotation (no mirror).
    """
    ux, uy = np.asarray(u, dtype=float) / np.linalg.norm(u)
    m = np.eye(4)
    m[:3, 0] = (ux, uy, 0.0)
    m[:3, 1] = (uy, -ux, 0.0)
    m[:3, 2] = (0.0, 0.0, -1.0)
    m[:3, 3] = (center[0], center[1], face_z)
    return m


def to_location(m: np.ndarray) -> Location:
    """A build123d location for a rigid 4x4 transform."""
    t = gp_Trsf()
    t.SetValues(*(float(v) for v in m[:3, :4].ravel()))
    return Location(t)


def away_from_pillars(o, pillars) -> np.ndarray:
    """Unit XY direction from O away from the frame pillars (servo +x)."""
    o = np.asarray(o, dtype=float)
    away = -sum((np.asarray(p, dtype=float) - o for p in pillars), np.zeros(2))
    n = np.linalg.norm(away)
    return away / n if n > 1e-9 else np.array([1.0, 0.0])


def _key(xy, u, z) -> tuple:
    return (round(float(xy[0]), 6), round(float(xy[1]), 6), round(float(u[0]), 9),
            round(float(u[1]), 9), round(float(z), 6))


@lru_cache(maxsize=64)
def _placed_servo(spec: ServoSpec, key: tuple):
    """The servo in the world, cut back to what stands on the plate (see ``DriveGroup.realize``)."""
    ox, oy, ux, uy, plate_top = key
    part = servo_part(spec).moved(to_location(servo_to_world((ox, oy), (ux, uy),
                                                             plate_top + spec.mount_face_z)))
    bb = part.bounding_box()
    if plate_top - bb.min.Z > 1e-6:
        depth = plate_top - bb.min.Z + 1.0
        r = spec.horn.diameter / 2
        below = Box(1e3, 1e3, depth).moved(Location((ox, oy, plate_top - depth / 2)))
        keep = Cylinder(r, depth).moved(Location((ox, oy, plate_top - depth / 2)))
        part = cut_each(part, below - keep)
    return part


class DriveGroup:
    """One servo on the inner frame plate."""

    name = GROUP

    def __init__(self, spec: ServoSpec):
        self.spec = spec

    # -- interface ----------------------------------------------------------------

    def spacer(self, ctx: Context) -> float:
        """Thickness of the printed horn spacer (0: none needed).

        The crank bolts on at or below the inner plate's bottom face (it turns
        against the plate otherwise).
        """
        t = ctx.pitch - self.spec.horn_face_depth
        return 0.0 if t <= 1e-6 else max(t, MIN_SPACER)

    def pattern_angle(self, ctx: Context) -> float:
        """Horn-hole angle (from the first crankpin) farthest from every crankpin.

        The crank's horn screws go in from below, past the webs; between the
        webs they don't weaken them. The horn goes on the spline in whatever
        position gets closest (it doesn't matter to the servo: it turns
        continuously).
        """
        pat = self.spec.horn.pattern
        topo = ctx.topo
        pins = topo.axes_of("crankpin")
        if topo.center is None or not pins:
            return math.radians(pat.angle_deg)
        pts = topo.geometry.points
        o = pts["O"][0]

        def ang(name):
            d = pts[name][0] - o
            return math.atan2(d[1], d[0])

        rel = [ang(p.name) - ang(pins[0].name) for p in pins]
        step = 2 * math.pi / pat.count
        best, best_clear = 0.0, -1.0
        for k in range(360):
            phi = step * k / 360
            clear = min(abs(math.remainder(phi + step * j - r, 2 * math.pi))
                        for j in range(pat.count) for r in rel)
            if clear > best_clear + 1e-9:
                best, best_clear = phi, clear
        return best

    def interface(self, ctx: Context) -> DriveInterface:
        s, h = self.spec, self.spec.horn
        t = self.spacer(ctx)
        thread = h.pattern.thread or "M3"
        return DriveInterface(
            horn_face_depth=s.horn_face_depth + t,
            horn_radius=h.diameter / 2,
            horn_thickness=h.thickness + t,
            screw_pcd=h.pattern.pcd,
            screw_count=h.pattern.count,
            screw_clearance_d=h.pattern.hole_d,
            screw_head_d=SCREW_HEAD_D.get(thread, 5.5) + 0.5,
            screw_key=h.pattern.screw,
            center_head_d=h.center_screw_head_d,
            center_head_h=max(0.0, h.center_screw_head_h - t),
            pattern_angle=self.pattern_angle(ctx),
        )

    # -- mounting screws ------------------------------------------------------------

    def front_screws(self, ctx: Context) -> list[tuple[str, MountHole, ScrewKind, float]]:
        """Front mounting holes used: ``(point name, hole, screw kind, length)``.

        A hole is used when its screw head, under the plate, clears the widest
        crank hub (the horn, or the horn screws' counterbores plus a wall, as
        :class:`construction.crank.PrintedCrank` builds it) by the margin.
        """
        iface = self.interface(ctx)
        p = ctx.params
        hub = max(iface.horn_radius, iface.screw_pcd / 2 + iface.screw_head_d / 2 + p.min_wall)
        out = []
        for i, mh in enumerate(self.spec.mount):
            parsed = screw_from_key(mh.screw)
            if parsed is None:
                continue
            sk, length = parsed
            if math.hypot(mh.x, mh.y) - sk.head_d / 2 < hub + p.margin:
                continue
            out.append((f"servo.screw{i}", mh, sk, length))
        return out

    def _frame(self, ctx: Context) -> tuple[np.ndarray, np.ndarray]:
        """(O, servo +x) from the side's geometry (both fixed: any sample will do)."""
        pts = ctx.topo.geometry.points
        o = np.asarray(pts["O"][0], dtype=float)
        u = away_from_pillars(o, [pts[a.name][0] for a in ctx.topo.axes_of("frame")])
        return o, u

    def claims(self, ctx: Context) -> list[Claim]:
        """The mounting screws' heads under the inner plate (and their shanks through it).

        The screws are fixed, so they become fixed points of the side's
        geometry (``servo.screw<i>``).
        """
        if ctx.topo.center is None:
            return []
        screws = self.front_screws(ctx)
        if not screws:
            return []
        geo = ctx.topo.geometry
        o, u = self._frame(ctx)
        v = np.array([u[1], -u[0]])
        for name, mh, _, _ in screws:
            xy = o + mh.x * u + mh.y * v
            geo.points[name] = np.broadcast_to(xy, (geo.samples, 2))
        geo.__dict__.pop("step", None)      # cached per point; recompute with the new ones

        def make(L: Layout):
            plate_bottom = L.z(L.top)[0]
            out = []
            for name, mh, sk, _ in screws:
                out.append(Placed(L.top, Disc(name, mh.d / 2), GROUP, "servo screw", seat=True))
                out += [Placed(k, Disc(name, sk.head_d / 2), GROUP, "servo screw head")
                        for k in L.layers_between(plate_bottom - sk.head_h, plate_bottom)]
            return out

        return [Claim("servo screws", frozenset(), make)]

    # -- build ------------------------------------------------------------------

    def direction(self, build: Build) -> np.ndarray:
        """Servo +x in the plate: away from the frame pillars."""
        topo = build.plan.topo
        return away_from_pillars(build.xy("O"), [build.xy(a.name) for a in topo.axes_of("frame")])

    def horn_angle(self, build: Build) -> float:
        """World angle of the horn's first screw hole at this crank angle."""
        iface: DriveInterface = build.ctx.interfaces[self.name]
        pins = build.plan.topo.axes_of("crankpin")
        base = build.angle("O", pins[0].name) if pins else 0.0
        return base + iface.pattern_angle

    def realize(self, build: Build) -> Realized:
        """The servo, its horn (and spacer), mounting screws, the plate's holes and pad.

        The servo body is cut back to the plate's top face except over the
        horn hole: what pokes into the plate's relief cut-outs (the STS3215's
        raised front panel) isn't drawn, since the construction contract only
        lets drive parts below that face sit in the horn's claim and the
        drive's own (screw) claims.
        """
        s = self.spec
        ctx = build.ctx
        p = ctx.params
        out = Realized()
        o = build.xy("O")
        u = self.direction(build)
        v = np.array([u[1], -u[0]])          # servo +y in the world (see servo_to_world)
        ang = math.atan2(u[1], u[0])
        plate_bottom, plate_top = build.z(build.top)

        def world(x, y):
            return tuple(o + x * u + y * v)

        L, W, _ = s.body
        # plate: horn clearance, screw holes, reliefs; a pad under the whole footprint
        out.cut(FRAME_INNER, Cut(tuple(o), s.horn.diameter + 2 * p.margin))
        for r in s.front_reliefs:
            out.cut(FRAME_INNER, Rect(world((r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2),
                                      (r.x1 - r.x0 + 0.5, r.y1 - r.y0 + 0.5), ang))
        x0, x1 = s.axis_offset - L / 2, s.axis_offset + L / 2
        half = W / 2 + p.min_wall
        out.pad(FRAME_INNER, world(x0 + half, 0), world(x1 - half, 0), half * math.sqrt(2))

        crank_host = next(iter(build.plan.topo.crank_bodies), None)
        frame_host = build.plan.topo.frame_bodies[0]
        body = _placed_servo(s, _key(o, u, plate_top))
        out.bodies.append(hardware("servo", body, frame_host, fab="purchased",
                                   bom_key=s.bom_key, color=SERVO_COLOR))
        for i, (_, mh, sk, length) in enumerate(self.front_screws(ctx)):
            xy = world(mh.x, mh.y)
            out.cut(FRAME_INNER, Cut(xy, mh.d))
            out.bodies.append(hardware(f"servo_screw{i}", screw_body(xy, sk, plate_bottom, length),
                                       frame_host, fab="purchased", bom_key=mh.screw, color=STEEL))

        # the horn turns with the crank: its first hole at the interface's pattern angle
        theta = self.horn_angle(build)
        horn_frame = servo_to_world(o, (math.cos(theta), math.sin(theta)),
                                    plate_top + s.mount_face_z)
        horn = horn_part(s).moved(to_location(horn_frame))
        out.bodies.append(hardware("servo_horn", horn, crank_host or frame_host,
                                   fab="purchased", bom_key=s.horn.bom_key, color=HORN_COLOR))
        t = self.spacer(ctx)
        if t > 0:
            face = plate_top - s.horn_face_depth
            spacer = disc(tuple(o), s.horn.diameter / 2, face - t, face)
            pat = s.horn.pattern
            holes = [disc(tuple(o + pat.pcd / 2 * np.array([math.cos(a), math.sin(a)])),
                          pat.hole_d / 2, face - t - 1, face + 1)
                     for a in (theta + 2 * math.pi * k / pat.count for k in range(pat.count))]
            if s.horn.center_screw_head_d > 0:
                holes.append(disc(tuple(o), (s.horn.center_screw_head_d + p.print_fit) / 2,
                                  face - t - 1, face + 1))
            for hole in holes:
                spacer = spacer - hole
            out.bodies.append(hardware("servo_horn_spacer", spacer, crank_host or frame_host,
                                       fab="printed", color=SPACER_COLOR))
        return out


__all__ = ["DriveGroup", "away_from_pillars", "servo_to_world", "to_location"]
