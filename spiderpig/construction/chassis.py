"""The chassis between the two servos: the servo frames in the plate plane, the centre
plates the servos screw to, their rear screws and the frame ties' columns, in world
coordinates (:mod:`construction.robot` says how the robot is put together and why).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, replace

import numpy as np

from spiderpig.construction.base import Build, ConstructionError, Context
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.hardware.fasteners import Screw, parse
from spiderpig.mechanism import Body, Mechanism
from spiderpig.servos.model import UNKNOWN_HOLE_DEPTH
from spiderpig.shapes import Cut, Rect, box, cut_holes, disc, ring, union

REAR_ENGAGE = 4.0        # target thread engagement of a rear screw in its servo's pilot (mm)
MIN_ENGAGE = 2.0         # least thread engagement that still holds
HEAD_CLEARANCE = 0.3     # radial clearance around a screw head in a laser-cut recess (mm)
RELIEF_GROW = 0.5        # a relief cut-out is this much bigger than the bump, per side (mm)
MODEL_GAP = 0.01         # radial gap between modelled parts that touch in reality (mm)
EPS_CH = 1e-6
CHASSIS_COLOR = "#eb6834"
TIE_COLOR = "#2a7ab0"
STEEL = "#8a8d91"
BRASS = "#c9a227"


def centre_t(ctx: Context) -> float:
    """The centre plates' thickness: the frame's sheet (5052 aluminium, clamped by the ties;
    the joinery plan of 2026-10-03)."""
    return ctx.sheet_t("frame")


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
    """A frame tie (mm): a chain of goBILDA 1501 round standoffs (6 mm OD, M4) from each
    inner plate to the centre plates, an M4 button head through each inner plate from the
    leg side (its head in the clearance gap under the plate, which the drive group claims),
    an M4 set screw through the centre plates joining the two chains."""

    column: float        # radius of a column (the standoff)
    head_r: float        # the end screw's head
    head_h: float
    hole_d: float        # its hole in the inner and the centre plates (ISO 273 medium)
    screw_d: float


def tie_dims(ctx: Context) -> TieDims:
    from spiderpig.hardware.crank_catalog import m4_bhcs

    hd = get(m4_bhcs(8)).dims
    return TieDims(column=3.0, head_r=float(hd["head_d"]) / 2, head_h=float(hd["head_h"]),
                   hole_d=4.5, screw_d=4.0)


def servo_frame_ctx(ctx: Context) -> ServoFrame:
    """The left servo's frame from the side's geometry alone (before any plan: the claims
    of the screws the drive group puts under the inner plate need it)."""
    from spiderpig.servos.mount import away_from_pillars

    pts = ctx.topo.geometry.points
    o = np.asarray(pts["O"][0], dtype=float)
    u = away_from_pillars(o, [pts[a.name][0] for a in ctx.topo.axes_of("frame")])
    u = u / np.linalg.norm(u)
    return ServoFrame((float(o[0]), float(o[1])), (float(u[0]), float(u[1])))


def seat_keepouts(ctx: Context) -> list[tuple[tuple[float, float], float]]:
    """What the inner plate holds that a screw through it must keep clear of, known before
    a plan: the horn's clearance hole and every pillar's end (its M4 head and washer)."""
    p = ctx.params
    pts = ctx.topo.geometry.points
    o = pts["O"][0]
    out = [((float(o[0]), float(o[1])), ctx.servo.horn.diameter / 2 + p.margin)]
    out += [((float(pts[a.name][0][0]), float(pts[a.name][0][1])), 4.5 + p.min_wall)
            for a in ctx.topo.axes_of("frame")]
    return out


def tie_points_ctx(ctx: Context) -> list[tuple[float, float]]:
    """World XY of the frame ties: beside the servo's long sides, near its ends; a tie whose
    head would meet the horn's hole or a pillar's end in the inner plate is dropped."""
    spec, p, d = ctx.servo, ctx.params, tie_dims(ctx)
    frame = servo_frame_ctx(ctx)
    x0, x1, y0, y1 = _footprint(spec)
    c = max(d.column, d.head_r)
    yt = y1 + p.margin + c
    xs = (x0 + c, x1 - c) if x1 - x0 > 2 * c else ((x0 + x1) / 2,)
    keep_out = seat_keepouts(ctx)
    points = []
    for x in xs:
        for y in (yt, -yt):
            xy = frame.xy(x, y)
            if all(math.dist(xy, q) >= d.head_r + r + p.min_wall for q, r in keep_out):
                points.append(xy)
    if len(points) < 2:
        raise ConstructionError("fewer than two frame ties fit beside the servo")
    return points


def tie_points(build: Build, drive=None) -> list[tuple[float, float]]:
    """:func:`tie_points_ctx` for a build."""
    return tie_points_ctx(build.ctx)


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
    spec, p = ctx.servo, ctx.params
    pitch = centre_t(ctx)               # the centre plates' sheet (the frame's: aluminium)
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
    info["centre_plate_sheet"] = ctx.sheet("frame")
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


def _chain(D: float) -> tuple[list[float], float] | None:
    """Stock goBILDA 1501 lengths (at most 60 mm each, joined by M4 set screws) that fill
    ``D`` mm with less than 2 mm left over (taken up by DIN 988 shims): the fewest
    segments, then the least left over."""
    from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS

    lengths = sorted(GOBILDA_LENGTHS)
    for n in range(1, 5):
        best = None
        for combo in itertools.combinations_with_replacement(lengths, n):
            if n > 1 and min(combo) < 12:
                continue                     # a joined segment needs thread both ends
            left = round(D - sum(combo), 3)
            if 0 <= left < 2.0 and (best is None or left < best[1]):
                best = (list(combo), left)
        if best is not None:
            return best
    return None


def _shims(t: float) -> list[float]:
    steps = sorted((float(x) for x in get("shim_din988_4x8").dims["t"]), reverse=True)
    out, left = [], round(t, 3)
    for s in steps:
        k = int(left / s + 1e-6)
        out += [s] * k
        left = round(left - k * s, 3)
    return out


def _tie_parts(ctx, plan, tie_xy, z_mid: float, half: float, host, info, fastened,
               extras) -> list[Body]:
    """The frame ties: per tie and side a chain of round standoffs from the inner plate's
    servo-side face to the centre plates, shims at the plate, an M4 button head through
    the inner plate from the leg side, and an M4 set screw through the centre plates into
    both chains, which clamps them (no glue, no tapped plate, no insert)."""
    from spiderpig.hardware.crank_catalog import (
        M4_BHCS_LENGTHS,
        M4_SET_LENGTHS,
        gobilda_1501,
        m4_bhcs,
        m4_set_screw,
    )

    bodies: list[Body] = []
    if not tie_xy:
        info.update(ties=0)
        return bodies
    d = tie_dims(ctx)
    z_top = plan.z(plan.top)[1] - z_mid           # inner plate's servo-side face (left side)
    t_in = plan.t(plan.top)
    D = -half - z_top
    got = _chain(D)
    if got is None:
        raise ConstructionError(f"no stock standoffs fill the {D:.1f} mm from an inner plate "
                                "to the centre plates")
    segs, left = got
    shims = _shims(left)
    t_sh = round(sum(shims), 3)
    depth = min(8.0, min(segs) / 2)
    end = next(((L, L - t_in - t_sh) for L in M4_BHCS_LENGTHS
                if 4.0 - EPS_CH <= L - t_in - t_sh <= depth + EPS_CH), None)
    stud = next(((L, (L - 2 * half) / 2) for L in M4_SET_LENGTHS
                 if 3.0 - EPS_CH <= (L - 2 * half) / 2 <= depth + EPS_CH), None)
    if end is None or stud is None:
        raise ConstructionError("no stock M4 screw closes a frame tie's chain")
    for i, xy in enumerate(tie_xy):
        for side, sign in (("L", 1.0), ("R", -1.0)):
            face = sign * z_top                          # inner plate's servo-side face
            z = face
            if shims:
                zs = sorted((z, z + sign * t_sh))
                bodies.append(Body(name=f"{side}.tie_shims{i}", part=ring(xy, 8.0, 4.1, *zs),
                                   rigid_with=host[side], fab="purchased",
                                   bom_key="shim_din988_4x8", color=STEEL))
                if len(shims) > 1:
                    extras.append(BomLine("shim_din988_4x8", len(shims) - 1,
                                          f"frame tie {i}, {side}"))
                z += sign * t_sh
            for k, L in enumerate(segs):
                zs = sorted((z, z + sign * L))
                part = disc(xy, d.column - 0.01, *zs) - disc(xy, 2.0, zs[0] - 1, zs[1] + 1)
                bodies.append(Body(name=f"{side}.tie_standoff{i}_{k}", part=part,
                                   rigid_with=host[side], fab="purchased",
                                   bom_key=gobilda_1501(L), color=TIE_COLOR))
                z += sign * L
                if k + 1 < len(segs):
                    bodies.append(Body(name=f"{side}.tie_joint{i}_{k}",
                                       part=disc(xy, 1.95, *sorted((z - 6, z + 6))),
                                       rigid_with=host[side], fab="purchased",
                                       bom_key=m4_set_screw(12), color=STEEL))
            # the end screw from the leg side: its head under the inner plate
            leg = face - sign * t_in
            L, _ = end
            screw = union([disc(xy, d.head_r, *sorted((leg, leg - sign * d.head_h))),
                           disc(xy, 1.95, *sorted((leg, leg + sign * L)))])
            bodies.append(Body(name=f"{side}.tie_screw{i}", part=screw, rigid_with=host[side],
                               fab="purchased", bom_key=m4_bhcs(L), color=STEEL))
        L, _ = stud
        bodies.append(Body(name=f"tie_stud{i}", part=disc(xy, 1.95, -L / 2, L / 2),
                           rigid_with=host["L"], fab="purchased", bom_key=m4_set_screw(L),
                           color=STEEL))
        fastened += [(f"L.tie_screw{i}", f"L.tie_standoff{i}_0"),
                     (f"R.tie_screw{i}", f"R.tie_standoff{i}_0"),
                     (f"tie_stud{i}", f"L.tie_standoff{i}_{len(segs) - 1}"),
                     (f"tie_stud{i}", f"R.tie_standoff{i}_{len(segs) - 1}")]
        fastened += [(f"{s_}.tie_joint{i}_{k}", f"{s_}.tie_standoff{i}_{k + kk}")
                     for s_ in ("L", "R") for k in range(len(segs) - 1) for kk in (0, 1)]
    extras.append(BomLine("threadlocker_243", 0.01 * len(tie_xy),
                          "frame tie screws and studs"))
    info.update(ties=len(tie_xy), tie_screw=m4_bhcs(end[0]), tie_standoffs=segs,
                tie_shims_mm=t_sh, tie_stud=m4_set_screw(stud[0]),
                tie_engagement_mm=round(end[1], 2), tie_stud_engagement_mm=round(stud[1], 2))
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
    tie_d = tie_dims(ctx).hole_d if tie_xy else 0.0
    from spiderpig.materials import sheet

    sheet_key = ctx.sheet("frame")
    min_hole = sheet(sheet_key).min_hole if sheet_key else 0.0
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
                cuts.append(Cut(xy, max(h.d, min_hole)))
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
                           fab="laser", color=CHASSIS_COLOR, sheet=sheet_key))
    # (no adhesive: the ties' studs clamp the stack, and the rear screws hold each servo's
    # own plates)
    return bodies
