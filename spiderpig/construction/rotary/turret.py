"""The turret: an arm's side standing on a turntable about the vertical (2026-10-12).

The layout :data:`construction.robot.LAYOUTS` ``"turret"``. The side keeps its plane (the
robot's coordinates are the side's: x forward, y up, z across the stack); the yaw axis is
the vertical through the shoulder ``O`` at the stack's middle. Under it the base of
:mod:`.base_pair` (its plates, columns, bearings, shaft, drive, turntable and hard stop:
:func:`.base_pair.turret_parts`); on the turntable the side's two frame plates, each grown a
**foot** (:class:`TurretFoot`) to its top face and held by two stock steel angle brackets
outside it (an M3 through the plate from inside, an M3 up through the turntable), placed
clear of the drives' motors and sliders outside the plates. The cables loop loosely round
the base (a spiral wrap).

**Its plan** (:func:`turret_plan`): the yaw motor behind the axis on the shortest stock GT2
loop that keeps it clear of the bearings and collars; the base's heights
(:func:`.base_pair.heights`); its keep-out round the axis (:func:`.base_pair.keep_radius`:
the turntable, the hard stop, the base's round front; behind, its tail round the motor).
The turntable's top is under every link over that keep-out (its pins' heads, the drives'
hubs, the belts at the motors) by what stands on it (the shaft's clamp, the brackets'
screws), and under the motors and sliders outside the plates; the rest of the side passes
lower, so the tool reaches the bench, and under the tool wherever its flange passes over
the base. A static part of the side below the turntable over the base at assembly raises
:class:`ConstructionError` (the estimate missed it).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    Group,
    Realized,
)
from spiderpig.construction.plane import point_rect, poly_rect, seg_rect
from spiderpig.construction.rotary import RotaryPreset
from spiderpig.construction.rotary import base_pair as bp
from spiderpig.hardware.fasteners import screw
from spiderpig.shapes import Cut, moved

FOOT_R = 6.0            # the frame plates' foot: a stadium with a flat underside
MOTOR_GAP = 2.0         # the yaw motor's end over the base's bottom plate
SHAFT_HOLE = 8.1
SCREW_HOLE = 3.4
NUT_H = 2.4
GLUE = 0.003            # the epoxy kit's share per bearing bonded


@dataclass(frozen=True)
class TurretPlan:
    """Where everything of the turret goes (module docstring). Robot coordinates for the
    axis and the turntable's top (``y_t``); the rest in the turret's own frame (``local``:
    ``(a, b, h)`` -> robot ``(x_a + a, y_ground + h, z_a - b)``: a forward, h up)."""

    preset: RotaryPreset
    x_a: float
    z_a: float
    y_t: float
    y_ground: float
    h: dict[str, float]
    r_base: float
    r_table: float
    c: float                       # the motor from the axis
    loop: int                      # the belt's stock loop (mm)
    travel_deg: float              # what the joint may turn, its range's width
    range_deg: tuple[float, float]
    brackets: tuple[tuple[str, float], ...]     # (plate: "outer" | "inner", x)
    reach: float                   # the brackets' screws through the turntable, out from
    #                                their plate's face
    faces: dict[str, float]        # each plate's outside face z
    columns: tuple[float, ...]     # the base's columns' standoffs (mm, stock)
    sensor_r: float                # the hard stop's radius (:func:`.base_pair.stop_radius`)
    spacing: float                 # the support's bearings' centres apart
    keep_r: float                  # the arm's keep-out round the axis
    foot_l: float                  # the frame plates' feet's half length
    notes: tuple[str, ...] = field(default=())

    def robot(self, a: float, b: float, h: float) -> tuple[float, float, float]:
        return (self.x_a + a, self.y_ground + h, self.z_a - b)

    def local_ab(self, x: float, z: float) -> tuple[float, float]:
        return (x - self.x_a, self.z_a - z)

    def place(self, part):
        """A part drawn in the turret's frame, moved to the robot's."""
        from build123d import Location, Plane

        return moved(part, Location(Plane((self.x_a, self.y_ground, self.z_a),
                                          x_dir=(1, 0, 0), z_dir=(0, 1, 0))))

    @property
    def bracket(self) -> dict[str, float]:
        """The feet's stock steel angle brackets (:func:`.base_pair.stock_bracket`)."""
        return bp.stock_bracket()

    def drive_r(self) -> float:
        """The pitch radius the yaw is turned at (the large pulley's)."""
        _, tr, _, _ = self.preset.parts
        return float(tr.big["pitch_d"]) / 2

    def drive_terms(self):
        _, tr, _, _ = self.preset.parts
        return tr.terms(self.c, self.loop)

    def error_terms(self):
        return self.preset.error_terms(self.spacing, self.drive_terms())


# -- the plan ------------------------------------------------------------------------------


def _lows(design, keep: tuple[float, float, float]) -> tuple[float, float, list]:
    """The lowest y inside the stack (links, pins, the drives' hubs and belts) and outside
    the plates (motors, sliders, pillar heads), and the outside footprints per plate:
    ``[(plate, kind, data)]``. ``keep``: ``(x_a, front, back)``: only the links over the
    keep-out round the axis (``front`` before it, ``back`` behind) count; the rest may pass
    lower, past the turntable (the tool reaches the table)."""
    from spiderpig import construction
    from spiderpig.construction.transmissions import JointDrive

    ctx: Context = design.ctx
    pts = ctx.topo.geometry.points
    lat = ctx.topo.geometry.lateral
    p = ctx.params
    pin_head = construction.axle(ctx.config.pin).dims(ctx, False).head
    pillar_head = construction.axle(ctx.config.pillar).dims(ctx, True).head
    r_link = max(p.link_radius, pin_head) + p.margin
    inside = math.inf
    for segs in ctx.topo.links.values():
        for a_, b_ in segs:
            pa = np.asarray(pts[a_], dtype=float).reshape(-1, 2)
            pb = np.asarray(pts[b_], dtype=float).reshape(-1, 2)
            for f_ in np.linspace(0.0, 1.0, 9):
                q = pa + (pb - pa) * f_
                d_ = q[:, 0] - keep[0]
                q = q[(d_ < keep[1] + r_link) & (d_ > -(keep[2] + r_link))]
                if q.size:
                    lat_ = max(lat.get(a_, 0.0), lat.get(b_, 0.0))
                    inside = min(inside, float(q[:, 1].min()) - lat_ - r_link)
    outside = math.inf
    prints: list = []
    for ax in ctx.topo.axes_of("frame"):
        xy = np.asarray(pts[ax.name], dtype=float).reshape(-1, 2)[0]
        outside = min(outside, float(xy[1]) - pillar_head)
        prints += [(plate, "circle", (xy, pillar_head)) for plate in ("outer", "inner")]
    for g in design.groups:
        if isinstance(g, JointDrive):
            i, o = g.lowest(ctx)
            inside, outside = min(inside, i), min(outside, o)
            down, corners, slider = g.footprint(ctx)
            plate = "outer" if down else "inner"
            prints.append((plate, "poly", corners))
            if slider is not None:
                prints.append((plate, "capsule", slider))
    return inside, outside, prints


def _clear(rect: tuple[float, float, float, float], prints: list, plate: str) -> bool:
    for pl, kind, data in prints:
        if pl != plate:
            continue
        if kind == "poly" and poly_rect(data, rect) < 1.0:
            return False
        if kind == "capsule" and seg_rect(data[0], data[1], rect) < data[2] + 1.0:
            return False
        if kind == "circle" and point_rect(data[0], rect) < data[1] + 1.0:
            return False
    return True


def turret_plan(design, preset: RotaryPreset, tool_plan=None) -> TurretPlan:
    """The turret under ``design``'s side (module docstring); ``tool_plan``: the tool's
    (:func:`construction.tools.tool_plan`), or ``None``."""
    from spiderpig.hardware.catalog import get

    ctx: Context = design.ctx
    plan = design.plan
    support, tr, motor, _sensor = preset.parts
    try:
        preset.check()                  # its homing: a stall on the stop built here
    except ValueError as e:
        raise ConstructionError(str(e)) from None
    pts = ctx.topo.geometry.points
    o = np.asarray(pts["O"] if "O" in pts else pts[ctx.topo.axes_of("frame")[0].name],
                   dtype=float).reshape(-1, 2)[0]
    x_a = float(o[0])
    faces = {"outer": float(plan.z(0)[0]), "inner": float(plan.z(plan.top)[1])}
    z_a = (faces["outer"] + faces["inner"]) / 2
    notes: list[str] = []

    # the drive: the motor square to the arm behind the axis, its flat face clear of the
    # bearings' bodies and the collars, on the shortest stock loop that keeps it so
    lo, hi = preset.range_deg
    clear_r = max(float(get(support.bearing).dims["od"]),
                  float(get("collar_8mm").dims["od"])) / 2
    need = max(clear_r + motor.frame / 2 + 1.0,
               (float(tr.big["flange_d"]) + float(tr.small["flange_d"])) / 2 + 2.0)
    fits = [(c, L) for L in _loops()
            if (c := tr.centre_distance(L)) is not None and c >= need
            and tr.teeth_in_mesh(c) >= bp.YAW_MIN_MESH - 1e-9]
    if not fits:
        raise ConstructionError("turret: no stock GT2 loop keeps the motor clear of the "
                                "shaft's bearings")
    c, loop = fits[0]

    # the base: its heights, its round core (the keep-out) and the lobe behind
    h, columns, spacing, protrude = bp.heights(ctx, support, tr, motor)
    foot_l = bp.BASE_FOOT_L
    front, r_stop = bp.keep_radius(tr, c, faces, preset.range_deg, foot_l)
    h["front"] = front
    r_base = bp.base_radius(front, c, motor)
    keep_r = front + ctx.params.margin
    half_t = motor.frame / 2 + bp.BASE_RIM
    turn = max(abs(v) for v in preset.range_deg)
    if r_base > front and turn + math.degrees(math.asin(min(1.0, half_t / keep_r))) > 175.0:
        raise ConstructionError(f"turret: a {turn:.0f} deg yaw swings the arm's low links "
                                "over the base's lobe")

    # how high the turntable stands: under the side over the keep-out, by what stands on it
    inside, outside, prints = _lows(design, (x_a, keep_r, r_base))
    bk = bp.stock_bracket()
    head = bk["hole"] + screw("shcs", "3").head_d / 2
    y_t = min(inside - max(head, protrude) - 1.0, outside - 1.0)

    # the brackets, clear of what stands outside each plate: a symmetric pair about the
    # axis, if any fits
    brackets: list[tuple[str, float]] = []
    for plate in ("outer", "inner"):
        xs = [x_a + v for v in np.arange(-(foot_l - bk["w"] / 2 - 1),
                                          foot_l - bk["w"] / 2 - 1 + 1e-9, 1.0)]
        ok = [float(x) for x in xs
              if _clear((float(x) - bk["w"] / 2 - 1, float(y_t),
                         float(x) + bk["w"] / 2 + 1, float(y_t) + bk["h"] + 1),
                        prints, plate)]
        if len(ok) < 2 or max(ok) - min(ok) < bk["w"] + 8:
            raise ConstructionError(f"turret: no room for two brackets outside the {plate} "
                                    f"frame plate's foot (the motors and sliders there "
                                    f"reach down to it)")
        pair = (min(ok), max(ok))
        have = {round(x - x_a) for x in ok}
        sym = [dd for dd in sorted(have, reverse=True)
               if dd > 0 and -dd in have and 2 * dd >= bk["w"] + 8]
        if sym:
            pair = (x_a - sym[0], x_a + sym[0])
        brackets += [(plate, float(pair[0])), (plate, float(pair[1]))]
    reach = bk["hole"]
    corners = []
    for plate, x in brackets:
        z0 = faces[plate]
        sgn = -1.0 if plate == "outer" else 1.0
        corners += [math.hypot(xx - x_a, zz - z_a)
                    for xx in (x - bk["w"] / 2, x + bk["w"] / 2)
                    for zz in (z0, z0 + sgn * bk["d"])]
    corners += [math.hypot(foot_l + FOOT_R, faces[pl] - z_a) for pl in faces]
    # round the brackets and the hard stop, flush with the base's round core
    r_table = max(max(corners) + 3.0, r_stop + 4.0, front)

    # the tool, wherever the flange passes over the turntable in front or the base's tail
    if tool_plan is not None:
        from spiderpig.construction.tools import keepout

        f = np.asarray(pts[tool_plan.point], dtype=float)
        d = f[:, 0] - x_a
        over = ((d < max(keep_r, r_table) + keepout(tool_plan.tool))
                & (d > -(r_base + keepout(tool_plan.tool))))
        if over.any():
            low = float(f[over, 1].min()) - ctx.topo.geometry.lateral.get(
                tool_plan.point, 0.0) - tool_plan.tool.drop()
            if low - protrude - 1.0 < y_t:        # (over the clamp and nuts it stands)
                y_t = low - protrude - 1.0
                notes.append("the turntable stands lower for the tool passing over it")
    if r_table > front + 1e-6:     # (as wide as it: flush)
        raise ConstructionError(f"turret: the turntable's {r_table:.1f} mm passes the base's "
                                f"front of {front:.1f} mm")
    y_ground = y_t - h["table"]
    if tool_plan is not None:      # where the tool reaches the bench
        note, h["reach_lo"], h["reach_hi"] = bp.table_reach(design, tool_plan, x_a, y_ground,
                                                            keep_r)
        notes.append(note)
    return TurretPlan(preset, x_a, z_a, y_t, y_ground, h, r_base, r_table, c, loop, hi - lo,
                      (lo, hi), tuple(brackets), reach, faces, columns, r_stop, spacing,
                      keep_r, foot_l, tuple(notes))


def _loops():
    from spiderpig.hardware.motion_catalog import GT2_LOOPS

    return GT2_LOOPS


# -- the side's part: its feet -------------------------------------------------------------


class TurretFoot(Group):
    """The frame plates grown down to the turntable's top (a stadium with a flat underside,
    joined to each frame pivot) and drilled for the brackets' screws. No claims: nothing it
    adds is inside the stack (the brackets' screws' heads are under every link over the
    base: :func:`turret_plan`)."""

    name = "turret foot"

    def __init__(self, tp: TurretPlan):
        self.tp = tp

    def claims(self, ctx: Context) -> list:
        return []

    def realize(self, build: Build, done: Realized) -> Realized:
        tp = self.tp
        out = Realized()
        r = build.ctx.params.frame_radius
        ends = ((tp.x_a - tp.foot_l, tp.y_t + FOOT_R), (tp.x_a + tp.foot_l, tp.y_t + FOOT_R))
        for key, plate in ((FRAME_OUTER, "outer"), (FRAME_INNER, "inner")):
            out.pad(key, ends[0], ends[1], FOOT_R)
            for ax in build.ctx.topo.axes_of("frame"):
                f = build.xy(ax.name)
                x = min(max(float(f[0]), ends[0][0]), ends[1][0])
                out.pad(key, tuple(f), (x, tp.y_t + max(r, FOOT_R)), r)
            for pl, x in tp.brackets:
                if pl == plate:
                    out.cut(key, Cut((x, tp.y_t + tp.bracket["hole"]), SCREW_HOLE))
        return out


def _along_y(part):
    """Drawn along z (``disc((a, b), r, h0, h1)``), stood along the robot's y."""
    from build123d import Axis, Plane

    return part.rotate(Axis.X, -90).mirror(Plane.XY)
