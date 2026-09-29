"""The robot: two mirror-image sides with their servos back to back in one frame.

In side coordinates (:mod:`fabricate`) the outer frame plate starts at
``z = 0``, the inner frame plate is on top and the servo stands on it. The
robot's mid-plane is where the two servos' rear faces meet a stack of
**centre plates** that the servos screw into. The left side ("L.") is its
side moved so that mid-plane is at ``z = 0``; the right side ("R.") is the
mirror image (``z -> -z``). The mechanism is planar, so both sides move
identically in XY; only their parts are mirrored. The servos therefore turn
in opposite senses about their own axes, which drives both sides forward.

The chassis (the frame the two servos live in) belongs to the robot, not a side:

**Centre plates** (laser-cut, laminated with the sheet's adhesive). There are
:func:`centre_plates` of them, enough that the two servos' rear bumps clear
each other; each plate has a relief cut-out only where a bump reaches it.
Both servos are real servos (the right one is not the left one's mirror
image), so the right servo's servo-frame ``+y`` points the other way in the
world. Each servo uses its rear holes on its own ``+y`` side, so the two
screw sets never share a hole position. The first ``n // 2`` plates on a
servo's side are its own:
its screws pass through them into the pilot holes (``REAR_ENGAGE`` of thread
in the case), and their heads bear on the last own plate, recessed in the
plates beyond it (for three plates: the middle one). Assembly: screw each
servo to its own plate(s), then laminate the stack with the middle plate(s)
between them, over the heads.

**Frame ties** (printed): four columns between the two inner frame plates,
beside the servo's long sides, make the frame rigid in bending and torsion.
Nothing may go below an inner plate (the leg stack is there, and the
planner doesn't know about the ties), so a column can't be clamped to a plate
by a screw head on the leg side. Instead each column is two halves that meet
the centre plates: a *screw half* on the left and an *insert half* on the
right, each seated in its inner plate with a spigot (flush with the plate's
leg-side face, bonded with CA glue) and bearing on the plate's top face. One
M3 socket head screw per tie runs down the screw half's bore, through the
centre plates, into an M3 heat-set insert in the insert half: it clamps both
halves to the centre plates and ties the two inner plates together through
them. Heat-set inserts rather than nuts: they are fused into the printed
half, so the screw pulls the half itself (a nut in an end pocket would only
bear on the centre plates), they need no side nut slot that weakens a
column, and the thread is reusable. The screw goes in from the leg side of
the left inner plate, through the spigot, before the legs are fitted.

:class:`FrameTies` is the side-level part of the ties: the spigot holes and
pads it adds to the inner frame plate (no claims: nothing it adds is below
the plate's top face). :func:`assemble_robot` places the ties themselves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from build123d import Location, Plane

from construction.base import FRAME_INNER, Build, ConstructionError, Context, Realized
from hardware.bom import BomLine
from hardware.catalog import adhesive, get, pick_length
from hardware.fasteners import CLEARANCE, Screw, parse, screw
from mechanism import Body, Mechanism, MechanismTemplate
from servos.model import UNKNOWN_HOLE_DEPTH
from shapes import Cut, Rect, box, cut_holes, disc, union

SIDES = ("L", "R")
TIE_SCREW = screw("shcs", "3")   # the frame ties' M3 socket head screws
REAR_ENGAGE = 4.0        # target thread engagement of a rear screw in its servo's pilot (mm)
MIN_ENGAGE = 2.0         # least thread engagement that still holds
HEAD_CLEARANCE = 0.3     # radial clearance around a screw head in a laser-cut recess (mm)
RELIEF_GROW = 0.5        # a relief cut-out is this much bigger than the bump, per side (mm)
SPIGOT_RECESS = 0.2      # a tie spigot stops this short of the inner plate's leg-side face
INSERT_ENGAGE = 5.0      # target thread engagement of a tie screw in its insert (mm)
MODEL_GAP = 0.01         # radial gap between modelled parts that touch in reality (mm)
CHASSIS_COLOR = "#eb6834"
TIE_COLOR = "#2a7ab0"
STEEL = "#8a8d91"
BRASS = "#c9a227"


def prefixed(name: str | None, side: str) -> str | None:
    return None if name is None else f"{side}.{name}"


def robot_template(tmpl: MechanismTemplate) -> MechanismTemplate:
    """Both sides' kinematics: the side template twice, prefixed ``L.`` and ``R.``."""
    bodies, connections = [], []
    for side in SIDES:
        bodies += [replace(b, name=prefixed(b.name, side)) for b in tmpl.bodies]
        connections += [((i, prefixed(pb, side), pj), (k, prefixed(cb, side), cj))
                        for (i, pb, pj), (k, cb, cj) in tmpl.connections]
    return MechanismTemplate(name=f"{tmpl.name}_robot", bodies=bodies, connections=connections,
                             meta=dict(tmpl.meta))


def centre_plates(spec, pitch: float, margin: float) -> int:
    """Centre plates needed so the two servos' rear bumps clear each other."""
    proud = max((r.height for r in spec.rear_reliefs), default=0.0)
    return max(1, math.ceil((2 * proud + margin) / pitch - 1e-9))


def _rear_face(spec) -> float:
    return spec.rear_face_z if spec.rear_face_z is not None else spec.mount_face_z - spec.body[2]


def mid_plane_z(spec, top: int, pitch: float, margin: float) -> float:
    """Side-coordinate z of the robot's mid-plane for a stack of ``top + 1`` layers."""
    n = centre_plates(spec, pitch, margin)
    return (top + 1) * pitch + (spec.mount_face_z - _rear_face(spec)) + n * pitch / 2


def mid_plane(design) -> float:
    """Side-coordinate z of the robot's mid-plane."""
    return mid_plane_z(design.ctx.servo, design.plan.top, design.ctx.pitch,
                       design.ctx.params.margin)


# ---------------------------------------------------------------------------
# Servo frames in the plate plane
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ServoFrame:
    """World XY of a servo's frame (see :mod:`servos.spec`).

    ``u`` is servo +x, ``v`` the left servo's +y (:func:`servos.mount.servo_to_world`).
    The right servo is a real servo facing the other way, so its +y is ``-v``
    (``hand = -1``).
    """

    o: tuple[float, float]
    u: tuple[float, float]
    hand: int = 1

    @property
    def v(self) -> np.ndarray:
        return np.array([self.u[1], -self.u[0]])

    @property
    def angle(self) -> float:
        return math.atan2(self.u[1], self.u[0])

    def xy(self, x: float, y: float) -> tuple[float, float]:
        p = np.asarray(self.o) + x * np.asarray(self.u) + self.hand * y * self.v
        return float(p[0]), float(p[1])

    def local(self, p) -> tuple[float, float]:
        d = np.asarray(p, dtype=float)[:2] - np.asarray(self.o)
        return float(d @ np.asarray(self.u)), float(self.hand * (d @ self.v))


def servo_frame(build: Build, drive) -> ServoFrame:
    """The left servo's frame (the side's own servo) for ``drive`` (a :class:`DriveGroup`)."""
    u = np.asarray(drive.direction(build), dtype=float)
    u = u / np.linalg.norm(u)
    o = build.xy("O")
    return ServoFrame((float(o[0]), float(o[1])), (float(u[0]), float(u[1])))


def _rect_distance(p: tuple[float, float], x0, x1, y0, y1) -> float:
    """Distance from a point to an axis-aligned rectangle (0 inside)."""
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


def _footprint(spec) -> tuple[float, float, float, float]:
    """The servo body's rectangle in the servo frame."""
    L, W, _ = spec.body
    return spec.axis_offset - L / 2, spec.axis_offset + L / 2, -W / 2, W / 2


# ---------------------------------------------------------------------------
# Frame ties
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TieDims:
    """A tie column (mm). Radii except where named ``_d``."""

    column: float        # radius of a column
    spigot_d: float      # spigot seated in the inner plate
    bore_d: float        # screw half: bore the screw head passes down
    floor: float         # screw half: thickness of the floor the head bears on
    clearance_d: float   # screw clearance hole (floor, centre plates)
    insert_d: float      # insert half: hole for the heat-set insert
    insert_len: float
    screw_d: float
    head_d: float
    head_h: float


def tie_dims(ctx: Context) -> TieDims:
    p = ctx.params
    ins = get("m3_heat_set_insert").dims
    sk = TIE_SCREW
    bore = sk.head_d + 2 * p.print_fit
    spigot = round(bore + 2 * max(p.min_wall, 1.2), 1)
    column = spigot / 2 + 2.5
    dims = TieDims(column=column, spigot_d=spigot, bore_d=bore, floor=3.0,
                   clearance_d=CLEARANCE[sk.size], insert_d=ins["hole_d"],
                   insert_len=ins["length"], screw_d=sk.d, head_d=sk.head_d, head_h=sk.head_h)
    if column - dims.insert_d / 2 < ins["min_wall"]:
        raise ConstructionError("a tie column is too thin for its heat-set insert")
    return dims


def tie_points(build: Build, drive) -> list[tuple[float, float]]:
    """World XY of the frame ties: beside the servo's long sides, near its ends.

    A tie is dropped if its spigot hole would cut into anything seated in the
    inner plate (a pillar's anchor, the servo horn's clearance hole).
    """
    ctx = build.ctx
    spec, p, d = ctx.servo, ctx.params, tie_dims(ctx)
    frame = servo_frame(build, drive)
    x0, x1, y0, y1 = _footprint(spec)
    yt = y1 + p.margin + d.column
    xs = (x0 + d.column, x1 - d.column) if x1 - x0 > 2 * d.column else ((x0 + x1) / 2,)
    hole_r = p.hole(d.spigot_d, "glue") / 2
    keep_out = [(build.xy("O"), spec.horn.diameter / 2 + p.margin)]   # the horn's clearance hole
    for s in build.plan.shapes(layer=build.top):
        if s.seat and hasattr(s.shape, "at"):
            keep_out.append((build.xy(s.shape.at), s.shape.r))
    points = []
    for x in xs:
        for y in (yt, -yt):
            if _rect_distance((x, y), x0, x1, y0, y1) < d.column + p.margin - 1e-9:
                raise ConstructionError("a frame tie would touch the servo")
            xy = frame.xy(x, y)
            if all(math.dist(xy, c) >= hole_r + r + p.min_wall for c, r in keep_out):
                points.append(xy)
    if len(points) < 2:
        raise ConstructionError("fewer than two frame ties fit beside the servo")
    return points


class FrameTies:
    """Side-level part of the frame ties: spigot holes and pads in the inner frame plate."""

    name = "frame ties"

    def __init__(self, drive):
        self.drive = drive

    def claims(self, ctx: Context) -> list:
        return []   # nothing below the inner plate's top face

    def realize(self, build: Build) -> Realized:
        out = Realized()
        p, d = build.ctx.params, tie_dims(build.ctx)
        frame = servo_frame(build, self.drive)
        for xy in tie_points(build, self.drive):
            x, _ = frame.local(xy)
            out.cut(FRAME_INNER, Cut(xy, p.hole(d.spigot_d, "glue")))
            out.pad(FRAME_INNER, xy, frame.xy(x, 0.0), d.column + p.min_wall)
        return out


# ---------------------------------------------------------------------------
# Rear screws
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RearScrews:
    """How the servos screw to the centre plates (one set per servo, same servo-frame holes)."""

    holes: tuple               # servo.spec.MountHole, in the servo frame
    key: str | None            # catalog item
    own: int                   # centre plates on each servo's side that its screws clamp
    length: float
    engage: float              # thread in the pilot
    d: float
    head_d: float
    head_h: float


def _screw_choice(hole_key: str | None, grip: float, depth: float) -> tuple[Screw, float]:
    """The screw for a rear hole: the hole's screw family, resized.

    The screw reaches ``REAR_ENGAGE`` into the pilot if the hole is that deep,
    never past its bottom (``depth``), and at least ``MIN_ENGAGE``.
    """
    parsed = parse(hole_key or "m2_self_tap_6")
    if parsed is None:
        raise ConstructionError(f"no screws modelled for the rear hole's {hole_key!r}")
    sk, _ = parsed
    fits = [L for L in sk.lengths if MIN_ENGAGE <= L - grip <= depth]
    if not fits:
        raise ConstructionError(f"no {sk.key(0).rsplit('_', 1)[0]} length engages "
                                f"{MIN_ENGAGE:g}..{depth:g} mm through {grip:g} mm of plate")
    target = min(REAR_ENGAGE, depth)
    return sk, float(min(fits, key=lambda L: (abs(L - grip - target), -L)))


def rear_screws(spec, n: int, pitch: float) -> RearScrews | None:
    """The rear screw set, or ``None`` if the servo can't be screwed to the centre plates."""
    own = n // 2
    holes = tuple(h for h in spec.rear_mount if h.y > 1e-6)
    if own < 1 or not holes:
        return None
    grip = own * pitch
    depth = min(h.depth if h.depth is not None else UNKNOWN_HOLE_DEPTH for h in holes)
    sk, length = _screw_choice(holes[0].screw, grip, depth)
    rs = RearScrews(holes=holes, key=sk.key(length), own=own, length=length,
                    engage=length - grip, d=sk.d, head_d=sk.head_d, head_h=sk.head_h)
    if rs.head_h >= (n - own) * pitch:      # from its seat to the other servo's rear face
        raise ConstructionError("rear screw heads don't fit between the servos")
    return rs


def _clear_holes(rs: RearScrews, frames, reliefs, half: float, pitch: float) -> tuple:
    """The holes whose heads (on either servo) stay out of every rear bump's space."""
    head_r = rs.head_d / 2 + HEAD_CLEARANCE
    seat = -half + rs.own * pitch                     # the left heads bear here
    heads = ((frames[0], (seat, seat + rs.head_h)), (frames[1], (-seat - rs.head_h, -seat)))
    keep = []
    for h in rs.holes:
        if all(not (_overlaps(hz, zr) and _rect_distance(rf.local(f.xy(h.x, h.y)), *rect) < head_r)
               for f, hz in heads for rf, rect, zr in reliefs):
            keep.append(h)
    return tuple(keep)


def _relief_volumes(spec, frames, half: float):
    """Every rear bump as (frame, rect in its servo frame, z range)."""
    out = []
    for frame, face, sign in ((frames[0], -half, 1.0), (frames[1], half, -1.0)):
        for r in spec.rear_reliefs:
            z = sorted((face, face + sign * r.height))
            rect = (r.x0 - RELIEF_GROW, r.x1 + RELIEF_GROW, r.y0 - RELIEF_GROW, r.y1 + RELIEF_GROW)
            out.append((frame, rect, tuple(z)))
    return out


def _overlaps(a: tuple[float, float], b: tuple[float, float]) -> bool:
    return max(a[0], b[0]) < min(a[1], b[1]) - 1e-9


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------


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
    chassis, extras, info = _chassis(side, design, z_mid)
    bodies += chassis
    meta = dict(side.meta, robot=True, mid_plane=z_mid, filament="pla_filament", **info)
    return Mechanism(
        name=f"{side.name}_robot", bodies=bodies, connections=connections, meta=meta,
        bom_extras=list(side.bom_extras) * 2 + extras,
    )


def _rounded_rect(frame: ServoFrame, x0, x1, y0, y1, r: float, z0: float, z1: float):
    """A plate outline: rectangle ``x0..x1`` by ``y0..y1`` in ``frame``, corners of radius r."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    c, ang, h = frame.xy(cx, cy), frame.angle, z1 - z0
    parts = [box(c, (x1 - x0, y1 - y0 - 2 * r, h), z0, ang),
             box(c, (x1 - x0 - 2 * r, y1 - y0, h), z0, ang)]
    parts += [disc(frame.xy(x, y), r, z0, z1) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)]
    return union(parts)


def _chassis(side: Mechanism, design, z_mid: float) -> tuple[list[Body], list[BomLine], dict]:
    """Centre plates, the servos' rear screws and the frame ties (world coordinates)."""
    ctx, plan = design.ctx, design.plan
    spec, p, pitch = ctx.servo, ctx.params, ctx.pitch
    build = Build(ctx, plan, side)
    left = servo_frame(build, design.drive)
    frames = (left, replace(left, hand=-1))
    n = centre_plates(spec, pitch, p.margin)
    half = n * pitch / 2
    frame_body = plan.topo.frame_bodies[0]
    host = {s: prefixed(frame_body, s) for s in SIDES}
    x0, x1, y0, y1 = _footprint(spec)
    bodies: list[Body] = []
    fastened: list[tuple[str, str]] = []
    info: dict = {"centre_plates": n}

    # -- rear screws: (side, xy, head z, shank z) ---------------------------------
    screws: list[tuple[str, tuple[float, float], tuple, tuple, object]] = []
    rs = rear_screws(spec, n, pitch)
    reliefs = _relief_volumes(spec, frames, half)
    if rs is not None:
        usable = _clear_holes(rs, frames, reliefs, half, pitch)
        rs = replace(rs, holes=usable) if usable else None
    if rs is not None:
        for i, h in enumerate(rs.holes):
            for s, frame, sign in (("L", frames[0], 1.0), ("R", frames[1], -1.0)):
                xy = frame.xy(h.x, h.y)
                seat = sign * (-half + rs.own * pitch)
                head = tuple(sorted((seat, seat + sign * rs.head_h)))
                shank = tuple(sorted((seat, seat - sign * rs.length)))
                part = union([disc(xy, rs.head_d / 2, *head), disc(xy, rs.d / 2, *shank)])
                name = f"{s}.rear_screw{i}"
                bodies.append(Body(name=name, part=part, rigid_with=host[s], fab="purchased",
                                   bom_key=rs.key, color=STEEL))
                fastened.append((name, f"{s}.servo"))
                screws.append((s, xy, head, (sign * -half, seat), h))
        info.update(rear_screw=rs.key, rear_screws_per_servo=len(rs.holes),
                    rear_engagement_mm=rs.engage)
    else:
        info.update(rear_screw=None, rear_screws_per_servo=0)

    # -- frame ties -----------------------------------------------------------------
    tie_xy = tie_points(build, design.drive)
    extras: list[BomLine] = []
    if tie_xy:
        d = tie_dims(ctx)
        z_top = plan.z(plan.top)[1] - z_mid           # inner plate's top face (left side)
        z_spigot = z_top - (pitch - SPIGOT_RECESS)
        need = d.floor + 2 * half + min(INSERT_ENGAGE, d.insert_len)
        length = pick_length(need, TIE_SCREW.lengths)
        key = TIE_SCREW.key(length)
        engage = length - d.floor - 2 * half
        pocket = max(d.insert_len, engage) + 1.0
        if -half - z_top < d.floor + d.head_h + 1.0 or pocket > -half - z_top - 1.0:
            raise ConstructionError("the servo is too short for the frame ties")
        for i, xy in enumerate(tie_xy):
            z_floor = -half - d.floor
            screw_half = union([disc(xy, d.column, z_top, -half),
                                disc(xy, d.spigot_d / 2, z_spigot, z_top)])
            screw_half = cut_holes(screw_half, [Cut(xy, d.clearance_d)], z_floor, -half)
            screw_half = screw_half - disc(xy, d.bore_d / 2, z_spigot - 1.0, z_floor)
            insert_half = union([disc(xy, d.column, half, -z_top),
                                 disc(xy, d.spigot_d / 2, -z_top, -z_spigot)])
            insert_half = insert_half - disc(xy, d.insert_d / 2, half - 1.0, half + pocket)
            screw = union([disc(xy, d.head_d / 2, z_floor - d.head_h, z_floor),
                           disc(xy, d.screw_d / 2 - 0.05, z_floor, z_floor + length)])
            insert = (disc(xy, d.insert_d / 2 - MODEL_GAP, half, half + d.insert_len)
                      - disc(xy, d.screw_d / 2, half - 1.0, half + d.insert_len + 1.0))
            bodies += [
                Body(name=f"L.tie_screw_half{i}", part=screw_half, rigid_with=host["L"],
                     fab="printed", color=TIE_COLOR),
                Body(name=f"R.tie_insert_half{i}", part=insert_half, rigid_with=host["R"],
                     fab="printed", color=TIE_COLOR),
                Body(name=f"L.tie_screw{i}", part=screw, rigid_with=host["L"],
                     fab="purchased", bom_key=key, color=STEEL),
                Body(name=f"R.tie_insert{i}", part=insert, rigid_with=host["R"],
                     fab="purchased", bom_key="m3_heat_set_insert", color=BRASS),
            ]
            fastened.append((f"L.tie_screw{i}", f"R.tie_insert{i}"))
        extras.append(BomLine("ca_glue", 1, "tie spigots into the inner frame plates"))
        info.update(ties=len(tie_xy), tie_screw=key, tie_engagement_mm=engage)
    else:
        info.update(ties=0)

    # -- centre plates ----------------------------------------------------------------
    rr = (rs.head_d / 2 + HEAD_CLEARANCE if rs else 0.0) + p.min_wall   # wall round a recess
    xs, ys = [x0, x1], [y0, y1]
    for _, xy, _, _, _ in screws:
        lx, ly = left.local(xy)
        xs += [lx - rr, lx + rr]
        ys += [ly - rr, ly + rr]
    corner = 3.0
    if tie_xy:
        tr = tie_dims(ctx).column + 1.0
        corner = tr
        for xy in tie_xy:
            lx, ly = left.local(xy)
            xs += [lx - tr, lx + tr]
            ys += [ly - tr, ly + tr]
    tie_d = tie_dims(ctx).clearance_d if tie_xy else 0.0
    for k in range(n):
        z0 = -half + k * pitch
        z1 = z0 + pitch
        cuts: list = []
        for rf, rect, zr in reliefs:
            if _overlaps((z0, z1), zr):
                rx0, rx1, ry0, ry1 = rect
                cuts.append(Rect(rf.xy((rx0 + rx1) / 2, (ry0 + ry1) / 2),
                                 (rx1 - rx0, ry1 - ry0), rf.angle))
        for _, xy, head, shank, h in screws:
            if _overlaps((z0, z1), tuple(sorted(shank))):
                cuts.append(Cut(xy, h.d))
            if _overlaps((z0, z1), head):
                cuts.append(Cut(xy, rs.head_d + 2 * HEAD_CLEARANCE))
        cuts += [Cut(xy, tie_d) for xy in tie_xy]
        part = _rounded_rect(left, min(xs), max(xs), min(ys), max(ys), corner, z0, z1)
        part = cut_holes(part, cuts, z0, z1)
        # A servo's face is flat round its own mounting holes (the relief rectangles are
        # bounding boxes of round features): keep a seat for the screw head there.
        seats = [disc(xy, rs.head_d / 2 + HEAD_CLEARANCE, z0, z1) - disc(xy, h.d / 2, z0, z1)
                 for _, xy, _, shank, h in screws if _overlaps((z0, z1), tuple(sorted(shank)))]
        if seats:
            part = union([part, *seats])
        bodies.append(Body(name=f"centre_plate{k}", part=part, rigid_with=host["L"],
                           fab="laser", color=CHASSIS_COLOR))
    if n > 1:
        extras.append(BomLine(adhesive(ctx.config.sheet), 1, "laminate the centre plates"))
    info["fastened"] = fastened
    return bodies, extras, info
