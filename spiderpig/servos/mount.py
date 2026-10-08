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
from typing import cast

import numpy as np
from build123d import Box, Cylinder, Location
from OCP.gp import gp_Trsf

from spiderpig import linkage
from spiderpig.construction.base import (
    FRAME_INNER,
    Build,
    ConstructionError,
    Context,
    DriveInterface,
    Group,
    Realized,
    hardware,
)
from spiderpig.hardware.fasteners import SIZES, Screw, parse, screw, screw_solid
from spiderpig.servos.model import cad_state, cut_each, horn_part, servo_part
from spiderpig.servos.spec import MountHole, ServoSpec
from spiderpig.shapes import Cut, Rect, Shape3D, disc, moved
from spiderpig.stack import Claim, Disc, Layout, Placed

MIN_SPACER = 1.0          # thinnest printed horn spacer worth making (mm)
RELIEF_CUT_GROW = 0.0     # a front relief's cut-out past its rectangle, per side (mm): none
#                           since the assembly audit of 2026-10-04 (0.25 before): the STS3215's
#                           panel rectangle already holds both models ([SO] |y| <= 7.0,
#                           [WS3] 6.7), and 0.25 left its far front holes 1.80 mm of web in
#                           0.080 in, under 1 x t (2.05 now: a warning)
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
def _placed_servo(spec: ServoSpec, key: tuple, state: tuple = ()):
    """The servo in the world, cut back to what stands on the plate (see ``DriveGroup.realize``).
    ``state``: the model in play (:func:`servos.model.cad_state`), so a switch or a
    download mid-process isn't served the earlier one."""
    ox, oy, ux, uy, plate_top = key
    part = moved(servo_part(spec, state=state), to_location(servo_to_world((ox, oy), (ux, uy),
                                                             plate_top + spec.mount_face_z)))
    bb = part.bounding_box()
    if plate_top - bb.min.Z > 1e-6:
        depth = plate_top - bb.min.Z + 1.0
        r = spec.horn.diameter / 2
        below = Box(1e3, 1e3, depth).moved(Location((ox, oy, plate_top - depth / 2)))
        keep = Cylinder(r, depth).moved(Location((ox, oy, plate_top - depth / 2)))
        part = cut_each(part, below - keep)
    return part


class DriveGroup(Group):
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
        plate = ctx.sheet_t("frame")            # the inner frame plate
        # the crank's laser plates need the face on a layer boundary: the plate's bottom
        # face, or a layer's under it (the horn's layers hold no plate of the crank's: the
        # default sheet's thickness)
        layer = ctx.pitch
        n = max(0, math.ceil((self.spec.horn_face_depth - plate) / layer - 1e-9))
        t = plate + n * layer - self.spec.horn_face_depth
        if 1e-6 < t < MIN_SPACER:
            t += layer
        # a crankpin's screw head over the hub plate stands in a pocket of the spacer (a
        # short crank: the Hoecken pantograph's): the spacer at least that thick
        need = self._hub_head_need(ctx)
        while need > 0 and t < need - 1e-6:
            t += layer
        return 0.0 if t <= 1e-6 else t

    def _hub_head_need(self, ctx: Context) -> float:
        """What the side's crank needs of the horn spacer over its hub plate (0: nothing):
        :meth:`construction.crank.BoltCrank.hub_head_need`."""
        key = getattr(ctx.config, "crank", None)
        if key is None or ctx.topo.center is None:
            return 0.0
        from spiderpig.construction import CRANKS

        crank = CRANKS.get(key)
        if crank is None or not hasattr(crank, "hub_head_need"):
            return 0.0
        crank = crank.resolve(ctx)
        h = self.spec.horn
        return crank.hub_head_need(ctx, h.diameter / 2)

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
        inputs = linkage.get(ctx.config.linkage).inputs
        if len(inputs) > 1:
            others = [k for k in linkage.available("mechanism")
                      if len(linkage.get(k).inputs) == 1]
            raise ConstructionError(
                f"{ctx.topo.name}: the drive turns one input, the crank at O (one servo); "
                f"{ctx.config.linkage} has {len(inputs)} ({', '.join(inputs)}), and a drive "
                f"for {', '.join(inputs[1:])} isn't built: a limit of v1 (one servo per "
                f"machine), not of this spec, so no change to it helps; a one-input mechanism "
                f"builds ({', '.join(others)})")
        s, h = self.spec, self.spec.horn
        t = self.spacer(ctx)
        head = screw("shcs", SIZES.get(h.pattern.thread, "3"))    # an ISO 4762 head, or larger
        return DriveInterface(
            horn_face_depth=s.horn_face_depth + t,
            horn_radius=h.diameter / 2,
            horn_thickness=h.thickness + t,
            screw_pcd=h.pattern.pcd,
            screw_count=h.pattern.count,
            screw_clearance_d=h.pattern.hole_d,
            screw_head_d=head.head_d + 0.5,
            screw_key=h.pattern.screw,
            center_head_d=h.center_screw_head_d,
            center_head_h=max(0.0, h.center_screw_head_h - t),
            pattern_angle=self.pattern_angle(ctx),
            plate_t=ctx.sheet_t("frame"),
            horn_layers=round((s.horn_face_depth + t - ctx.sheet_t("frame")) / ctx.pitch),
            spacer_t=t,
        )

    # -- mounting screws ------------------------------------------------------------

    def front_screws(self, ctx: Context) -> list[tuple[str, MountHole, Screw, float]]:
        """Front mounting holes used: ``(point name, hole, screw family, length)``.

        A hole is used when its screw head, under the plate, clears the widest
        crank hub (the horn, or the horn screws' counterbores plus a wall) by the
        margin.
        """
        iface = self.interface(ctx)
        p = ctx.params
        hub = max(iface.horn_radius, iface.screw_pcd / 2 + iface.screw_head_d / 2 + p.min_wall)
        if iface.horn_layers >= 1:
            # the hub steps down a layer under the horn (a crank of whole plates): the heads
            # under the plate meet only the horn and its spacer, so all four front screws
            hub = iface.horn_radius
        out = []
        for i, mh in enumerate(self.spec.mount):
            parsed = parse(mh.screw)
            if parsed is None:
                continue
            sk, length = parsed
            if math.hypot(mh.x, mh.y) - sk.head_d / 2 < hub + p.margin:
                continue
            if self.screw_web(ctx, mh) < p.servo_screw_web_t * self._plate_t(ctx) - 1e-6:
                continue
            out.append((f"servo.screw{i}", mh, sk, length))
        return out

    @staticmethod
    def _plate_t(ctx: Context) -> float:
        return ctx.sheet_t("frame") if ctx.sheet("frame") is not None else ctx.pitch

    def hole_d(self, ctx: Context, mh: MountHole) -> float:
        """A front screw's hole in the inner plate: the servo's, at least the service's
        smallest (SendCutSend: the sheet's thickness)."""
        from spiderpig.materials import sheet

        key = ctx.sheet("frame")
        min_hole = sheet(key).min_hole if key else 0.0
        return max(mh.d, min_hole + 0.025)

    def screw_web(self, ctx: Context, mh: MountHole) -> float:
        """The inner plate's web round a front screw's hole: to the horn's clearance hole and
        to the front reliefs' cut-outs (mm). The STS3215's near holes (r 13.19) leave 1.01
        mm to the horn's (r 10.975), its far ones 2.05 to the raised panel's relief."""
        r = self.hole_d(ctx, mh) / 2
        s = self.spec
        web = math.hypot(mh.x, mh.y) - r - (s.horn.diameter / 2 + ctx.params.margin)
        for f in s.front_reliefs:
            dx = max(f.x0 - RELIEF_CUT_GROW - mh.x, 0.0, mh.x - f.x1 - RELIEF_CUT_GROW)
            dy = max(f.y0 - RELIEF_CUT_GROW - mh.y, 0.0, mh.y - f.y1 - RELIEF_CUT_GROW)
            web = min(web, math.hypot(dx, dy) - r)
        return web

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
        geo = ctx.topo.geometry
        o, u = self._frame(ctx)
        v = np.array([u[1], -u[0]])
        heads: list[tuple[str, float, float, float, str]] = []   # name, hole r, head r, h
        for name, mh, sk, _ in screws:
            xy = o + mh.x * u + mh.y * v
            geo.points[name] = np.broadcast_to(xy, (geo.samples, 2))
            heads.append((name, mh.d / 2, sk.head_d / 2, sk.head_h, "servo screw"))
        # the frame ties' and the deck rails' screws come up through the inner plate from the
        # leg side too (construction.chassis, construction.deck): their heads are claimed here
        for name, xy, hole, r, h, what in self.chassis_screws(ctx):
            geo.points[name] = np.broadcast_to(np.asarray(xy, dtype=float), (geo.samples, 2))
            heads.append((name, hole, r, h, what))
        geo.__dict__.pop("step", None)      # cached per point; recompute with the new ones
        if not heads:
            return []

        from spiderpig.construction.pivots.common import HEAD_CLEARANCE

        def make(L: Layout):
            out = []
            for name, hole, r, h, what in heads:
                out.append(Placed(L.top, Disc(name, hole), GROUP, what, seat=True))
                # the head under the plate: in the clearance gap there, or the layer under
                # it where nothing else is (its height declared, the gap sized to it)
                out.append(Placed(L.top - 1, Disc(name, r), GROUP, f"{what} head", gap=True,
                                  height=h + HEAD_CLEARANCE, toward=-1))
            return out

        return [Claim("servo screws", frozenset(), make)]

    def chassis_screws(self, ctx: Context) -> list[tuple[str, tuple, float, float, float, str]]:
        """The screws the robot's chassis puts up through this side's inner plate from the
        leg side: ``(point, xy, hole radius, head radius, head height, what)``; none for a
        servo with no chassis (a frame tie or deck that doesn't fit: the robot says why)."""
        from spiderpig.construction.chassis import tie_dims, tie_points_ctx
        from spiderpig.construction.deck import (
            RAIL_HOLE,
            RAIL_SCREW,
            RAIL_SCREW_R,
            rail_screw_points,
        )

        out = []
        lk = getattr(ctx.config, "lk", None)
        if lk is not None and lk.kind != "walker":
            return out          # a mechanism is one side: no chassis
        try:
            d = tie_dims(ctx)
            for i, xy in enumerate(tie_points_ctx(ctx)):
                out.append((f"frame.tie{i}", xy, d.hole_d / 2, d.head_r + 0.3, d.head_h,
                            "frame tie screw"))
        except (ConstructionError, ValueError):
            pass
        try:
            for i, xy in enumerate(rail_screw_points(ctx)):
                out.append((f"frame.rail{i}", xy, RAIL_HOLE / 2, RAIL_SCREW_R,
                            RAIL_SCREW.head_h, "deck rail screw"))
        except (ConstructionError, ValueError):
            pass
        return out

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

    def realize(self, build: Build, done: Realized) -> Realized:
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
            g = 2 * RELIEF_CUT_GROW
            out.cut(FRAME_INNER, Rect(world((r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2),
                                      (r.x1 - r.x0 + g, r.y1 - r.y0 + g), ang))
        x0, x1 = s.axis_offset - L / 2, s.axis_offset + L / 2
        half = W / 2 + p.min_wall
        out.pad(FRAME_INNER, world(x0 + half, 0), world(x1 - half, 0), half * math.sqrt(2))

        crank_host = next(iter(build.plan.topo.crank_bodies), None)
        frame_host = build.plan.topo.frame_bodies[0]
        body = _placed_servo(s, _key(o, u, plate_top), cad_state(s))
        out.bodies.append(hardware("servo", body, frame_host, fab="purchased",
                                   bom_key=s.bom_key, color=SERVO_COLOR))
        for i, (_, mh, sk, length) in enumerate(self.front_screws(ctx)):
            xy = world(mh.x, mh.y)
            # a hole the service cuts (SendCutSend: at least the sheet's thickness); a pan
            # head still bears on the ring round it
            out.cut(FRAME_INNER, Cut(xy, self.hole_d(ctx, mh)))
            out.bodies.append(hardware(f"servo_screw{i}", screw_solid(xy, sk, plate_bottom, length),
                                       frame_host, fab="purchased", bom_key=mh.screw, color=STEEL))

        # the horn turns with the crank: its first hole at the interface's pattern angle
        theta = self.horn_angle(build)
        horn_frame = servo_to_world(o, (math.cos(theta), math.sin(theta)),
                                    plate_top + s.mount_face_z)
        horn = moved(horn_part(s), to_location(horn_frame))
        out.bodies.append(hardware("servo_horn", horn, crank_host or frame_host,
                                   fab="purchased", bom_key=s.horn.bom_key, color=HORN_COLOR))
        t = self.spacer(ctx)
        iface: DriveInterface = ctx.interfaces[self.name]
        if iface.horn_layers:       # the hub's top face, at the plan's z
            hub_top = build.z(build.top - iface.horn_layers - 1)[1]   # over a gap too
            t = plate_top - s.horn_face_depth - hub_top
            t = 0.0 if t <= 1e-6 else t
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
            # a crankpin's screw head over the crank's hub plate (a short crank): a pocket
            hub = build.top - iface.horn_layers - 1 if iface.horn_layers else None
            for sh in build.plan.shapes("crank"):
                if (hub is not None and sh.label.startswith("crankpin screw")
                        and sh.toward > 0
                        and sh.layer == (hub if sh.gap else hub + 1)):    # in its gap, or sunk
                    head = cast("Disc", sh.shape)  # a crankpin screw head's shape is a Disc
                    xy = tuple(build.xy(head.at))
                    if math.dist(xy, tuple(o)) - head.r < s.horn.diameter / 2:
                        holes.append(disc(xy, head.r, face - t - 1, face + 1))
            for hole in holes:
                spacer = cast("Shape3D", spacer - hole)  # a hole leaves the spacer whole
            out.bodies.append(hardware("servo_horn_spacer", spacer, crank_host or frame_host,
                                       fab="printed", color=SPACER_COLOR))
        return out
