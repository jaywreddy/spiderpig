"""The chassis between the two servos: the servo frames in the plate plane, the centre
plates the servos screw to, their rear screws and the frame ties' columns, in world
coordinates (:mod:`construction.robot` says how the robot is put together and why).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, replace
from functools import lru_cache

import numpy as np

from spiderpig.construction.base import Build, ConstructionError, Context
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.hardware.fasteners import Screw, parse
from spiderpig.mechanism import Body, Mechanism
from spiderpig.servos.model import UNKNOWN_HOLE_DEPTH
from spiderpig.shapes import Cut, box, cut_holes, disc, ring, union

REAR_ENGAGE = 4.0        # target thread engagement of a rear screw in its servo's pilot (mm)
MIN_ENGAGE = 2.0         # least thread engagement that still holds
HEAD_CLEARANCE = 0.3     # radial clearance around a screw head in a laser-cut recess (mm)
RELIEF_GROW = 0.5        # a relief cut-out is this much bigger than the bump, per side (mm)
RELIEF_CORNER = 1.0      # its inside corners' radius (SendCutSend cuts 0.8 mm in aluminium)
RELIEF_ROUND = RELIEF_CORNER * (1 - math.sqrt(0.5)) + 0.05   # a pocket grown round its
#                          rectangle so the rounded corners still hold it (each side)
MODEL_GAP = 0.01         # radial gap between modelled parts that touch in reality (mm)
EPS_CH = 1e-6
CHASSIS_COLOR = "#eb6834"
TIE_COLOR = "#2a7ab0"
STEEL = "#8a8d91"
BRASS = "#c9a227"


def centre_sheet(ctx: Context) -> str | None:
    """The centre plates' sheet: the thinnest aluminium of the frame's alloy that seats the
    most rear screws (the plates' count and thickness set where the screws' heads sit
    against the two servos' rear bumps; 0.080 in leaves one hole per servo on the STS3215,
    0.125 in two); the frame's sheet when nothing is better (or no chassis)."""
    key = ctx.sheet("frame")
    try:
        return _centre_sheet(ctx.servo, key, ctx.params.margin)
    except Exception:         # noqa: BLE001 - a servo with no rear holes: the frame's sheet
        return key


@lru_cache(maxsize=64)
def _centre_sheet(spec, frame_key: str | None, margin: float) -> str | None:
    # pure (the servo's spec, the catalogued sheets): asked 15-40 times per design and robot
    from spiderpig.materials import aluminium_sheets, sheet

    if frame_key is None or not sheet(frame_key).metal:
        return frame_key
    alloy = str(get(frame_key).dims.get("alloy", "5052"))[:4]
    left = ServoFrame((0.0, 0.0), (1.0, 0.0))
    frames = (left, replace(left, hand=-1))
    best = None
    for key in aluminium_sheets(alloy):
        t = sheet(key).thickness
        if t < sheet(frame_key).thickness - 1e-9:
            continue
        n = centre_plates(spec, t, margin)
        half = n * t / 2
        reliefs = _relief_volumes(spec, frames, half)
        slots = _port_slots(spec, frames, half)
        most = 0
        for own in range(1, max(2, n)):
            try:
                rs = rear_screws(spec, n, t, own)
            except ConstructionError:
                continue
            if rs is not None:
                most = max(most, len(_clear_holes(rs, frames, reliefs, half, t, slots,
                                                  sheet(key).min_hole)))
        # the most screws, then the thinnest stack (the bus plugs can need a fifth thin
        # plate where four of the next sheet do), then the thinnest sheet
        rank = (-most, round(n * t, 3))
        if best is None or rank < best[0]:
            best = (rank, key)
    return frame_key if best is None or best[0][0] == 0 else best[1]


def centre_t(ctx: Context) -> float:
    """The centre plates' thickness (:func:`centre_sheet`)."""
    key = centre_sheet(ctx)
    if key is None or key == getattr(ctx.config, "sheet", None):
        return ctx.pitch
    from spiderpig.materials import thickness

    return thickness(ctx.config, key)


def centre_plates(spec, pitch: float, margin: float) -> int:
    """Centre plates needed so the two servos' rear bumps, and the bus plugs in their
    sockets (``ServoSpec.bus_ports``: the two servos' plugs sit at the same place, from
    either side), clear each other."""
    proud = max((r.height for r in spec.rear_reliefs), default=0.0)
    if spec.bus_ports is not None:
        proud = max(proud, spec.bus_ports.height)
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


class RoundRelief(tuple):
    """A relief's rectangle ``(x0, x1, y0, y1)`` (servo frame) that stands for the circle
    inscribed in it (``servos.spec.Relief.round``: an idler boss)."""

    @property
    def centre(self) -> tuple[float, float]:
        return (self[0] + self[1]) / 2, (self[2] + self[3]) / 2

    @property
    def r(self) -> float:
        return (self[1] - self[0]) / 2


def _rect_distance(p: tuple[float, float], x0, x1, y0, y1) -> float:
    """Distance from a point to an axis-aligned rectangle (0 inside)."""
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


def _relief_distance(p: tuple[float, float], rect) -> float:
    """Distance from a point to a relief's cut-out (a :class:`RoundRelief`'s circle, else
    the rectangle; 0 inside)."""
    if isinstance(rect, RoundRelief):
        c = rect.centre
        return max(0.0, math.hypot(p[0] - c[0], p[1] - c[1]) - rect.r)
    return _rect_distance(p, *rect)


def _merge_close(rects: list[tuple], web: float) -> list[tuple]:
    """A plate's rectangular cut-outs (``(frame, (x0, x1, y0, y1))``, servo frame) with every
    two of one frame that face each other across less than ``web`` (the service's edge
    distance: a thinner web distorts, under a kerf it doesn't come back at all) merged into
    their bounding rectangle: one cut (the assembly audit of 2026-10-04: the STS3215's
    "pins" relief stood 0.31 mm off the bus plugs' slot). Round reliefs stay apart."""
    out = list(rects)
    merged = True
    while merged:
        merged = False
        for i, j in itertools.combinations(range(len(out)), 2):
            (fa, a), (fb, b) = out[i], out[j]
            if fa is not fb or isinstance(a, RoundRelief) or isinstance(b, RoundRelief):
                continue
            gx = max(a[0] - b[1], b[0] - a[1])
            gy = max(a[2] - b[3], b[2] - a[3])
            # facing across a gap in one direction, overlapping in the other
            if (gx < web and gy < 0) or (gy < web and gx < 0):
                box_ = (min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3]))
                out = [r for k, r in enumerate(out) if k not in (i, j)] + [(fa, box_)]
                merged = True
                break
    return out


def _footprint(spec) -> tuple[float, float, float, float]:
    """The servo body's rectangle in the servo frame."""
    L, W, _ = spec.body
    return spec.axis_offset - L / 2, spec.axis_offset + L / 2, -W / 2, W / 2


# ---------------------------------------------------------------------------
# Frame ties
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TieDims:
    """A frame tie (mm): a chain of uxcell 6 mm round M3 standoffs from each inner plate to
    the centre plates, an M3 button head through each inner plate from the leg side (its
    head in the clearance gap under the plate, which the drive group claims), an M3 set
    screw through the centre plates joining the two chains (:func:`_tie_parts`)."""

    column: float        # radius of a column (the standoff)
    head_r: float        # the end screw's head
    head_h: float
    hole_d: float        # its hole in the inner and the centre plates (ISO 273 medium)
    screw_d: float


TIE_SHIM_KEY = "shim_din988_3x6"   # a tie's clamped 1 mm shims (bought as DIN 433 pairs)
TIE_PLACE_R = 3.8                  # the ties keep the places the M4 ties had (their
#                                    M4 head's radius; the ties are M3 since 2026-10-05)


def tie_dims(ctx: Context) -> TieDims:
    """The M3 tie: uxcell 6 mm round M3 standoffs, M3 button heads and set screws."""
    from spiderpig.hardware.fasteners import SCREWS

    sk = SCREWS["bhcs", "3"]
    return TieDims(column=3.0, head_r=sk.head_d / 2, head_h=sk.head_h, hole_d=3.4,
                   screw_d=3.0)


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
    a plan: the horn's clearance hole and every pillar's end (its button head and 9 mm
    washer)."""
    p = ctx.params
    pts = ctx.topo.geometry.points
    o = pts["O"][0]
    out = [((float(o[0]), float(o[1])), ctx.servo.horn.diameter / 2 + p.margin)]
    out += [((float(pts[a.name][0][0]), float(pts[a.name][0][1])), 4.5 + p.min_wall)
            for a in ctx.topo.axes_of("frame")]
    return out


def tie_pad_r(ctx: Context) -> float:
    """The centre plates' outline round a tie: its hole and two thicknesses of the frame's
    sheet to the edge (SendCutSend's hole-to-edge rule; under one thickness is an error)."""
    d = tie_dims(ctx)
    return max(d.column + 1.0, d.hole_d / 2 + 2 * centre_t(ctx) + 0.1)


TIE_SHIFT_STEP = 0.25    # a tie moves along the servo's long side in these steps (mm) ...
TIE_SHIFT_MAX = 6.0      # ... at most this far, to keep its hole 2 x t off its neighbours'


def _min_edge(key: str | None) -> float:
    """The service's least hole-to-edge distance in sheet ``key`` (2 x t in metal)."""
    if key is None:
        return 0.0
    from spiderpig.materials import sheet

    return sheet(key).min_edge


def tie_neighbours(ctx: Context) -> list[tuple[float, float, float, float]]:
    """The holes a tie's hole shares a plate with, in the left servo's frame: ``(x, y,
    radius, least web)``: the servo's front screw holes in the inner plate, and both
    servos' rear screw holes and head recesses in the centre plates (each servo's on its
    own +y side, the right one's mirrored), each with the service's hole-to-edge rule for
    its plate (2 x t, :attr:`materials.Sheet.min_edge`)."""
    from spiderpig.materials import sheet

    spec = ctx.servo
    frame_key = ctx.sheet("frame")
    centre_key = centre_sheet(ctx)
    min_hole = sheet(frame_key).min_hole if frame_key else 0.0
    out = [(h.x, h.y, max(h.d, min_hole + 0.025) / 2, _min_edge(frame_key))
           for h in spec.mount]
    for h in spec.rear_mount:
        if h.y <= 1e-6:
            continue
        parsed = parse(h.screw or "m2_self_tap_6")
        r = max(h.d, (parsed[0].head_d + 2 * HEAD_CLEARANCE) if parsed else h.d) / 2
        out += [(h.x, h.y, r, _min_edge(centre_key)), (h.x, -h.y, r, _min_edge(centre_key))]
    return out


def tie_locals(ctx: Context) -> list[tuple[float, float]]:
    """The frame ties' places in the left servo's frame (before the inner plate's keep-outs:
    :func:`tie_points_ctx`): beside the servo's long sides near its ends, each moved along
    the side (outward first, at most :data:`TIE_SHIFT_MAX`) until its hole is the service's
    two thicknesses off every hole it shares a plate with (:func:`tie_neighbours`; the
    design review's warning level); where none is, the unmoved place (the audit warns)."""
    spec, p, d = ctx.servo, ctx.params, tie_dims(ctx)
    x0, x1, y0, y1 = _footprint(spec)
    c = max(d.column, d.head_r, TIE_PLACE_R)
    yt = y1 + p.margin + c
    xs = (x0 + c, x1 - c) if x1 - x0 > 2 * c else ((x0 + x1) / 2,)
    near = tie_neighbours(ctx)
    r = d.hole_d / 2
    n = int(round(TIE_SHIFT_MAX / TIE_SHIFT_STEP))
    # the bus plugs' slot through the centre plates (:func:`_port_slots`): a tie beside it
    # moves out across the servo until its hole is two thicknesses off the slot's side
    slot = spec.bus_ports.slot() if spec.bus_ports is not None else None
    slot_web = _min_edge(centre_sheet(ctx))
    out = []
    for x in xs:
        outward = -1.0 if x < (x0 + x1) / 2 else 1.0
        steps = [0.0] + [s * k * TIE_SHIFT_STEP for k in range(1, n + 1)
                         for s in (outward, -outward)]
        for y in (yt, -yt):
            best = next((x + dx for dx in steps
                         if all(math.hypot(x + dx - hx, y - hy) >= r + hr + web + 0.05
                                for hx, hy, hr, web in near)), x)
            if (slot is not None
                    and slot[0] - RELIEF_ROUND - r - slot_web < best < slot[1] + r + slot_web):
                side = (slot[3] if y > 0 else -slot[2]) + RELIEF_ROUND
                y = math.copysign(max(abs(y), side + r + slot_web + 0.05), y)
            out.append((best, y))
    return out


def tie_points_ctx(ctx: Context) -> list[tuple[float, float]]:
    """World XY of the frame ties (:func:`tie_locals`); a tie whose head would meet the
    horn's hole or a pillar's end in the inner plate is dropped."""
    p, d = ctx.params, tie_dims(ctx)
    frame = servo_frame_ctx(ctx)
    keep_out = seat_keepouts(ctx)
    points = []
    for x, y in tie_locals(ctx):
        xy = frame.xy(x, y)
        if all(math.dist(xy, q) >= d.head_r + r + p.min_wall for q, r in keep_out):
            points.append(xy)
    if len(points) < 2:
        raise ConstructionError("fewer than two frame ties fit beside the servo")
    return points


def recess_wall(ctx: Context, head_d: float) -> float:
    """The centre plates' outline round a rear screw's head recess: the recess and two
    thicknesses of the centre plates' sheet (the service's hole-to-edge rule), at least
    the old wall (``min_wall``)."""
    rec = head_d / 2 + HEAD_CLEARANCE
    return max(rec + ctx.params.min_wall, rec + _min_edge(centre_sheet(ctx)) + 0.1)


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


def rear_screws(spec, n: int, pitch: float, own: int | None = None) -> RearScrews | None:
    """The rear screw set, or ``None`` if the servo can't be screwed to the centre plates;
    ``own``: the centre plates each servo's screws clamp (default half of them)."""
    own = n // 2 if own is None else own
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


def _clear_holes(rs: RearScrews, frames, reliefs, half: float, pitch: float,
                 slots=(), min_hole: float = 0.0) -> tuple:
    """The holes whose heads (on either servo) stay out of every rear bump's space, whose
    head recesses stay two plate thicknesses (the cut rules' warning level) off the bus
    plugs' slots (:func:`_port_slots`), which run through every plate there, and whose
    shank holes, as cut (at least ``min_hole``, the service's), keep one plate thickness
    of web (the cut rules' error level) to the reliefs in the plates they pass."""
    head_r = rs.head_d / 2 + HEAD_CLEARANCE
    seat = -half + rs.own * pitch                     # the left heads bear here
    heads = ((frames[0], (seat, seat + rs.head_h)), (frames[1], (-seat - rs.head_h, -seat)))
    shanks = ((frames[0], (-half, seat)), (frames[1], (-seat, half)))
    keep = []
    for h in rs.holes:
        hole_r = max(h.d, min_hole) / 2
        if (all(not (_overlaps(hz, zr)
                     and _relief_distance(rf.local(f.xy(h.x, h.y)), rect) < head_r)
                for f, hz in heads for rf, rect, zr in reliefs)
                and all(not (_overlaps(sz, zr)
                             and _relief_distance(rf.local(f.xy(h.x, h.y)), rect)
                             < hole_r + RELIEF_ROUND + pitch)
                        for f, sz in shanks for rf, rect, zr in reliefs)
                and all(_rect_distance(rf.local(f.xy(h.x, h.y)), *rect)
                        >= head_r + 2 * pitch + RELIEF_ROUND
                        for f, _ in heads for rf, rect, _ in slots)):
            keep.append(h)
    return tuple(keep)


def _relief_volumes(spec, frames, half: float):
    """Every rear bump as (frame, rect in its servo frame, z range)."""
    out = []
    for frame, face, sign in ((frames[0], -half, 1.0), (frames[1], half, -1.0)):
        for r in spec.rear_reliefs:
            z = sorted((face, face + sign * r.height))
            rect = (r.x0 - RELIEF_GROW, r.x1 + RELIEF_GROW, r.y0 - RELIEF_GROW, r.y1 + RELIEF_GROW)
            out.append((frame, RoundRelief(rect) if r.round else rect, tuple(z)))
    return out


def _port_slots(spec, frames, half: float):
    """The bus plugs' way to their sockets (``ServoSpec.bus_ports``) as (frame, rect in its
    servo frame, z range), like :func:`_relief_volumes`: per servo an open slot from its
    socket bump to the centre plates' far edge through every plate the plugs stand in
    (from its rear face out to the plug's thickness). The rear face is on the plates, so
    a closed relief the size of the bump leaves no way in for a plug (assembly audit,
    2026-10-04); which way the sockets open is UNVERIFIED (:data:`servos.catalog`)."""
    ports = spec.bus_ports
    rect = ports.slot() if ports is not None else None
    if rect is None:
        return []
    out = []
    for frame, face, sign in ((frames[0], -half, 1.0), (frames[1], half, -1.0)):
        z = sorted((face, face + sign * ports.height))
        out.append((frame, rect, tuple(z)))
    return out


def _recess_bridges(xy, r: float, rects, web: float, z0: float, z1: float) -> list:
    """Cuts joining a round head recess at ``xy`` (radius ``r``) to each rectangular relief
    of ``rects`` (``(frame, rect)``) it stands closer to than ``web``: a band as wide as
    the recess from it into the relief, its corners rounded (one cut-out, no thin web)."""
    out = []
    for rf, rect in rects:
        if isinstance(rect, RoundRelief):
            continue
        lx, ly = rf.local(xy)
        rx0, rx1, ry0, ry1 = rect
        if _rect_distance((lx, ly), rx0, rx1, ry0, ry1) - r >= web:
            continue
        if ry0 <= ly <= ry1 or not (rx0 <= lx <= rx1):
            # beside it along x (or diagonal: along x first)
            bx0, bx1 = (lx, rx0 + 1.0) if lx < rx0 else (rx1 - 1.0, lx)
            out.append(_rounded_rect(rf, bx0, bx1, ly - r, ly + r, min(RELIEF_CORNER, r),
                                     z0 - 1.0, z1 + 1.0))
            if not ry0 <= ly <= ry1:
                by0, by1 = (ly, ry0 + 1.0) if ly < ry0 else (ry1 - 1.0, ly)
                xin = min(max(lx, rx0 + r), rx1 - r)
                out.append(_rounded_rect(rf, xin - r, xin + r, by0, by1,
                                         min(RELIEF_CORNER, r), z0 - 1.0, z1 + 1.0))
        else:
            by0, by1 = (ly, ry0 + 1.0) if ly < ry0 else (ry1 - 1.0, ly)
            out.append(_rounded_rect(rf, lx - r, lx + r, by0, by1, min(RELIEF_CORNER, r),
                                     z0 - 1.0, z1 + 1.0))
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
    slots = _port_slots(spec, frames, half)
    from spiderpig.materials import sheet as sheet_spec

    centre_key = centre_sheet(ctx)
    rs, screws, bodies = _rear_screw_parts(spec, frames, reliefs, n, half, pitch, host, info,
                                           fastened, slots,
                                           sheet_spec(centre_key).min_hole if centre_key
                                           else 0.0)
    tie_xy = tie_points(build, design.drive)
    extras: list[BomLine] = []
    bodies += _tie_parts(ctx, plan, tie_xy, z_mid, half, host, info, fastened, extras)
    info["centre_plate_sheet"] = centre_sheet(ctx)
    bodies += _centre_plate_parts(ctx, left, reliefs + slots, rs, screws, tie_xy, n, half,
                                  pitch, host, extras)
    if spec.bus_ports is not None:
        info["bus_ports"] = spec.bus_ports.opening
    info["fastened"] = fastened
    return bodies, extras, info


def _rear_screw_parts(spec, frames, reliefs, n: int, half: float, pitch: float, host, info,
                      fastened, slots=(), min_hole: float = 0.0) -> tuple:
    """The servos' rear screws: ``(the screw set, (side, xy, head z, shank z, hole) per
    screw, bodies)``."""
    bodies: list[Body] = []
    screws: list[tuple[str, tuple[float, float], tuple, tuple, object]] = []
    # the own plates that leave the most holes whose heads clear the other servo's bumps
    # (thinner aluminium centre plates: more of them, so the heads' z is a choice)
    best = None
    for own in range(1, max(2, n)):
        try:
            cand = rear_screws(spec, n, pitch, own)
        except ConstructionError:
            continue
        if cand is None:
            continue
        usable = _clear_holes(cand, frames, reliefs, half, pitch, slots, min_hole)
        if usable and (best is None or len(usable) > len(best.holes)):
            best = replace(cand, holes=usable)
    rs = best
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


TIE_TAKE_UP = (2.0, 3.0)   # the most a tie's shims take up: 2 mm, else (no chain fits) 3 mm
TIE_TOL = 0.1              # a chain within this of its span after the shims (the plates' fit)


def _chain(D: float) -> tuple[list[float], float] | None:
    """Stock uxcell 6 mm round M3 standoff lengths (joined by M3 set screws) and the shim
    stack (a multiple of :data:`construction.pivots.standoff.SHIM_STEP`, bought as DIN 433
    washers) that fill ``D`` mm within :data:`TIE_TOL`: ``(segments, shims mm)``, the fewest
    segments, then the thinnest stack; the stack up to 2 mm, else 3 mm (the XL330's 23 mm:
    no M3 pair fills it), ``None`` when none does."""
    from spiderpig.construction.pivots.standoff import SHIM_STEP
    from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS

    lengths = sorted(M3_ROUND_STANDOFF_LENGTHS)
    for most in TIE_TAKE_UP:
        for n in range(1, 5):
            best = None
            for combo in itertools.combinations_with_replacement(lengths, n):
                if n > 1 and min(combo) < 12:
                    continue                 # a joined segment needs thread both ends
                left = round(D - sum(combo), 3)
                take = round(left / SHIM_STEP) * SHIM_STEP
                if abs(left - take) > TIE_TOL + 1e-6:
                    continue
                if 0 <= take <= most + 1e-6 and (best is None or take < best[1]):
                    best = (list(combo), round(take, 3))
            if best is not None:
                return best
    return None


def _shims(t: float) -> list[float]:
    """The shims stacking to ``t`` (a multiple of the step): whole 1 mm and SHIM_STEP, the
    steps the BOM orders (:func:`hardware.bom.stack_steps`)."""
    from spiderpig.hardware.bom import shim_breakdown, stack_steps

    return shim_breakdown(t, stack_steps(TIE_SHIM_KEY))


def _tie_parts(ctx, plan, tie_xy, z_mid: float, half: float, host, info, fastened,
               extras) -> list[Body]:
    """The frame ties: per tie and side a chain of uxcell 6 mm round M3 standoffs from the
    inner plate's servo-side face to the centre plates, shims at the plate, an M3 button
    head through the inner plate from the leg side, and an M3 set screw through the centre
    plates into both chains, which clamps them (no glue, no tapped plate, no insert)."""
    from spiderpig.hardware.crank_catalog import (
        M3_SET_LENGTHS,
        m3_round_standoff,
        m3_set_screw,
    )
    from spiderpig.hardware.fasteners import SCREWS

    bhcs = SCREWS["bhcs", "3"]
    thread_max, engage_min = 6.0, 3.0
    shim_key = TIE_SHIM_KEY
    shim_od, shim_id = 6.0, 3.1
    r_screw = 1.45

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
    depth = min(thread_max, min(segs) / 2)
    end = next(((L, L - t_in - t_sh) for L in bhcs.lengths
                if engage_min - EPS_CH <= L - t_in - t_sh <= depth + EPS_CH), None)
    stud = next(((L, (L - 2 * half) / 2) for L in M3_SET_LENGTHS
                 if 3.0 - EPS_CH <= (L - 2 * half) / 2 <= depth + EPS_CH), None)
    if end is None or stud is None:
        raise ConstructionError("no stock M3 screw closes a frame tie's chain")
    for i, xy in enumerate(tie_xy):
        for side, sign in (("L", 1.0), ("R", -1.0)):
            face = sign * z_top                          # inner plate's servo-side face
            z = face
            if shims:
                zs = sorted((z, z + sign * t_sh))
                bodies.append(Body(name=f"{side}.tie_shims{i}",
                                   part=ring(xy, shim_od, shim_id, *zs),
                                   rigid_with=host[side], fab="purchased",
                                   bom_key=shim_key, color=STEEL))
                if len(shims) > 1:
                    extras.append(BomLine(shim_key, len(shims) - 1,
                                          f"frame tie {i}, {side}"))
                z += sign * t_sh
            for k, L in enumerate(segs):
                zs = sorted((z, z + sign * L))
                part = disc(xy, d.column - 0.01, *zs) - disc(xy, 2.0, zs[0] - 1, zs[1] + 1)
                bodies.append(Body(name=f"{side}.tie_standoff{i}_{k}", part=part,
                                   rigid_with=host[side], fab="purchased",
                                   bom_key=m3_round_standoff(L), color=TIE_COLOR))
                z += sign * L
                if k + 1 < len(segs):
                    bodies.append(Body(name=f"{side}.tie_joint{i}_{k}",
                                       part=disc(xy, r_screw, *sorted((z - 6, z + 6))),
                                       rigid_with=host[side], fab="purchased",
                                       bom_key=m3_set_screw(12), color=STEEL))
            # the end screw from the leg side: its head under the inner plate
            leg = face - sign * t_in
            L, _ = end
            screw = union([disc(xy, d.head_r, *sorted((leg, leg - sign * d.head_h))),
                           disc(xy, r_screw, *sorted((leg, leg + sign * L)))])
            bodies.append(Body(name=f"{side}.tie_screw{i}", part=screw, rigid_with=host[side],
                               fab="purchased", bom_key=bhcs.key(L), color=STEEL))
        L, _ = stud
        bodies.append(Body(name=f"tie_stud{i}", part=disc(xy, r_screw, -L / 2, L / 2),
                           rigid_with=host["L"], fab="purchased", bom_key=m3_set_screw(L),
                           color=STEEL))
        fastened += [(f"L.tie_screw{i}", f"L.tie_standoff{i}_0"),
                     (f"R.tie_screw{i}", f"R.tie_standoff{i}_0"),
                     (f"tie_stud{i}", f"L.tie_standoff{i}_{len(segs) - 1}"),
                     (f"tie_stud{i}", f"R.tie_standoff{i}_{len(segs) - 1}")]
        fastened += [(f"{s_}.tie_joint{i}_{k}", f"{s_}.tie_standoff{i}_{k + kk}")
                     for s_ in ("L", "R") for k in range(len(segs) - 1) for kk in (0, 1)]
    extras.append(BomLine("threadlocker_243", 0.01 * len(tie_xy),
                          "frame tie screws and studs"))
    info.update(ties=len(tie_xy), tie_screw=bhcs.key(end[0]), tie_standoffs=segs,
                tie_shims_mm=t_sh, tie_stud=m3_set_screw(stud[0]),
                tie_engagement_mm=round(end[1], 2), tie_stud_engagement_mm=round(stud[1], 2))
    return bodies


def _centre_plate_parts(ctx, left: ServoFrame, reliefs, rs, screws, tie_xy, n: int,
                        half: float, pitch: float, host, extras) -> list[Body]:
    """The centre plates: the servo footprint grown round every screw recess and tie,
    relieved where a rear bump reaches a plate and slotted to the far edge where the bus
    plugs pass (``reliefs`` holds both), with the screws' holes and seats."""
    p = ctx.params
    bodies: list[Body] = []
    x0, x1, y0, y1 = _footprint(ctx.servo)
    rr = recess_wall(ctx, rs.head_d) if rs else p.min_wall   # 2 x t round a recess
    xs, ys = [x0, x1], [y0, y1]
    for _, xy, _, _, _ in screws:
        lx, ly = left.local(xy)
        xs += [lx - rr, lx + rr]
        ys += [ly - rr, ly + rr]
    corner = 3.0
    if tie_xy:
        tr = tie_pad_r(ctx)
        corner = tr
        for xy in tie_xy:
            lx, ly = left.local(xy)
            xs += [lx - tr, lx + tr]
            ys += [ly - tr, ly + tr]
    tie_d = tie_dims(ctx).hole_d if tie_xy else 0.0
    from spiderpig.materials import sheet

    sheet_key = centre_sheet(ctx)
    min_hole = sheet(sheet_key).min_hole if sheet_key else 0.0
    web = sheet(sheet_key).min_edge if sheet_key else 0.0
    for k in range(n):
        z0 = -half + k * pitch
        z1 = z0 + pitch
        cuts: list = []
        pockets = []
        here = [(rf, rect) for rf, rect, zr in reliefs if _overlaps((z0, z1), zr)]
        # reliefs closer than the service's web: one cut (the pins relief into the slot)
        here = _merge_close(here, web + 2 * RELIEF_ROUND)
        for rf, rect in here:
            if isinstance(rect, RoundRelief):
                pockets.append(disc(rf.xy(*rect.centre), rect.r, z0 - 1.0, z1 + 1.0))
                continue
            # its corners rounded past the service's inside radius, grown so the rounded
            # pocket still holds the bump's rectangle
            rx0, rx1, ry0, ry1 = rect
            rx1 = min(rx1, max(xs) + 10.0)           # an open slot: past the far edge
            g = RELIEF_ROUND
            # in the plate's own frame (the frames share their origin and x axis; the
            # right servo's is mirrored in y): a pocket closer than one plate thickness
            # to the outline (the cut rules' error level) opens through it instead
            lx = (rx0 - g, rx1 + g)
            ly = tuple(sorted(left.local(rf.xy(0.0, y))[1] for y in (ry0 - g, ry1 + g)))
            lx0 = min(xs) - 10.0 if lx[0] - min(xs) < pitch + 1e-6 else lx[0]
            lx1 = max(xs) + 10.0 if max(xs) - lx[1] < pitch + 1e-6 else lx[1]
            ly0 = min(ys) - 10.0 if ly[0] - min(ys) < pitch + 1e-6 else ly[0]
            ly1 = max(ys) + 10.0 if max(ys) - ly[1] < pitch + 1e-6 else ly[1]
            pockets.append(_rounded_rect(left, lx0, lx1, ly0, ly1,
                                         RELIEF_CORNER, z0 - 1.0, z1 + 1.0))
        for _, xy, head, shank, h in screws:
            if _overlaps((z0, z1), tuple(sorted(shank))):
                cuts.append(Cut(xy, max(h.d, min_hole)))
            if _overlaps((z0, z1), head):
                cuts.append(Cut(xy, rs.head_d + 2 * HEAD_CLEARANCE))
                # a head recess within the web of a relief opens into it: one cut (the
                # recess only clears the head, which bears on the plate under it)
                pockets += _recess_bridges(xy, rs.head_d / 2 + HEAD_CLEARANCE, here,
                                           web + RELIEF_ROUND, z0, z1)
        cuts += [Cut(xy, tie_d) for xy in tie_xy]
        part = _rounded_rect(left, min(xs), max(xs), min(ys), max(ys), corner, z0, z1)
        part = cut_holes(part, cuts, z0, z1)
        if pockets:
            part = part - union(pockets)
        # A servo's face is flat round its own mounting holes (the relief rectangles are
        # bounding boxes of round features): keep a seat for the screw head there.
        seats = [disc(xy, rs.head_d / 2 + HEAD_CLEARANCE, z0, z1)
                 - disc(xy, max(h.d, min_hole) / 2, z0, z1)       # (the hole as cut above)
                 for _, xy, _, shank, h in screws if _overlaps((z0, z1), tuple(sorted(shank)))]
        if seats:
            part = union([part, *seats])
        bodies.append(Body(name=f"centre_plate{k}", part=part, rigid_with=host["L"],
                           fab="laser", color=CHASSIS_COLOR, sheet=sheet_key))
    # (no adhesive: the ties' studs clamp the stack, and the rear screws hold each servo's
    # own plates)
    return bodies
