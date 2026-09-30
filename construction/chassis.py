"""The chassis between the two servos: the servo frames in the plate plane, the centre
plates the servos screw to, their rear screws and the frame ties' columns, in world
coordinates (:mod:`construction.robot` says how the robot is put together and why).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from construction.base import Build, ConstructionError, Context
from hardware.bom import BomLine
from hardware.catalog import adhesive, get, pick_length
from hardware.fasteners import CLEARANCE, Screw, parse, screw
from mechanism import Body, Mechanism
from servos.model import UNKNOWN_HOLE_DEPTH
from shapes import Cut, Rect, box, cut_holes, disc, union

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


def centre_plates(spec, pitch: float, margin: float) -> int:
    """Centre plates needed so the two servos' rear bumps clear each other."""
    proud = max((r.height for r in spec.rear_reliefs), default=0.0)
    return max(1, math.ceil((2 * proud + margin) / pitch - 1e-9))


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
# The chassis parts
# ---------------------------------------------------------------------------


def _rounded_rect(frame: ServoFrame, x0, x1, y0, y1, r: float, z0: float, z1: float):
    """A plate outline: rectangle ``x0..x1`` by ``y0..y1`` in ``frame``, corners of radius r."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    c, ang, h = frame.xy(cx, cy), frame.angle, z1 - z0
    parts = [box(c, (x1 - x0, y1 - y0 - 2 * r, h), z0, ang),
             box(c, (x1 - x0 - 2 * r, y1 - y0, h), z0, ang)]
    parts += [disc(frame.xy(x, y), r, z0, z1) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)]
    return union(parts)


def chassis(side: Mechanism, design, z_mid: float, host: dict[str, str],
            ) -> tuple[list[Body], list[BomLine], dict]:
    """Centre plates, the servos' rear screws and the frame ties (world coordinates), the
    unmodelled purchases and what ``mech.meta`` records about them. ``host`` is the frame
    body every part of a side rides (``{"L": "L.torso", "R": "R.torso"}``)."""
    ctx, plan = design.ctx, design.plan
    spec, p, pitch = ctx.servo, ctx.params, ctx.pitch
    build = Build(ctx, plan, side)
    left = servo_frame(build, design.drive)
    frames = (left, replace(left, hand=-1))
    n = centre_plates(spec, pitch, p.margin)
    half = n * pitch / 2
    fastened: list[tuple[str, str]] = []
    info: dict = {"centre_plates": n}
    reliefs = _relief_volumes(spec, frames, half)
    rs, screws, bodies = _rear_screw_parts(spec, frames, reliefs, n, half, pitch, host, info,
                                           fastened)
    tie_xy = tie_points(build, design.drive)
    extras: list[BomLine] = []
    bodies += _tie_parts(ctx, plan, tie_xy, z_mid, half, host, info, fastened, extras)
    bodies += _centre_plate_parts(ctx, left, reliefs, rs, screws, tie_xy, n, half, pitch,
                                  host, extras)
    info["fastened"] = fastened
    return bodies, extras, info


def _rear_screw_parts(spec, frames, reliefs, n: int, half: float, pitch: float, host, info,
                      fastened) -> tuple:
    """The servos' rear screws: ``(the screw set, (side, xy, head z, shank z, hole) per
    screw, bodies)``."""
    bodies: list[Body] = []
    screws: list[tuple[str, tuple[float, float], tuple, tuple, object]] = []
    rs = rear_screws(spec, n, pitch)
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
    return rs, screws, bodies


def _tie_parts(ctx, plan, tie_xy, z_mid: float, half: float, host, info, fastened,
               extras) -> list[Body]:
    """The frame ties' columns (a screw half on the left, an insert half on the right),
    their screws and inserts."""
    pitch = ctx.pitch
    bodies: list[Body] = []
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
    return bodies


def _centre_plate_parts(ctx, left: ServoFrame, reliefs, rs, screws, tie_xy, n: int,
                        half: float, pitch: float, host, extras) -> list[Body]:
    """The centre plates: the servo footprint grown round every screw recess and tie,
    relieved where a rear bump reaches a plate, with the screws' holes and seats."""
    p = ctx.params
    bodies: list[Body] = []
    x0, x1, y0, y1 = _footprint(ctx.servo)
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
    return bodies
