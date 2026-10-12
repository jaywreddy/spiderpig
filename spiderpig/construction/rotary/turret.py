"""The turret: an arm's side standing on a turntable about the vertical (2026-10-12).

The layout :data:`construction.robot.LAYOUTS` ``"turret"``. The side keeps its plane (the
robot's coordinates are the side's: x forward, y up, z across the stack); the yaw axis is
the vertical through the shoulder ``O`` at the stack's middle. Bottom up:

* **the base**: a bottom plate and a top plate (the frame's aluminium) on four columns of
  round M3 standoffs, as tall as the yaw motor hanging under the top plate; under the top
  plate a **boss** of :data:`BOSS_N` discs bolted to it, through which the fixed 8 mm
  **shaft** is clamped by clamping collars above the plate and under the boss (their
  faces square it; a hardened shaft takes no set screw);
* **the drive** between the base and the hub: the belt's 80-tooth plate pulley (its bore
  drilled out to clear the collar under the hub, four 3.4 mm holes drilled on the hub's
  screws' circle, :data:`TIE_PCD`, on a drill press centred on its own bore) under the hub,
  the 16-tooth on the motor's uncut shaft; a stock loop with at least
  :data:`~construction.transmissions.belt.MIN_MESH` teeth in mesh; the motor's screws in
  slots along the belt, its tension set from above. Or the capstan's drum of laser-cut
  plates (its bottom flange bored out over the boss's screw heads), the cord on a sleeve
  over the motor's shaft, its travel what the shaft's length leaves the wraps' walk, its
  ends pinched by two anchor screws past the travel either side of where it leaves for the
  motor (:meth:`TurretPlan.anchor_angles`), each in a notch from the rim;
* **the hub**: two stacks of :data:`seat plates <seat_n>` with the 608s bonded in (on the
  shaft, which centres them), a shoulder plate each against the bearings' inner faces, a
  printed spacer between them (the bearings' centres :data:`~construction.rotary.SPACING`
  apart), all clamped with the turntable by four M3 screws from under the pulley (or sunk in
  the drum's bottom flange), nuts on the turntable. The lower inner ring sits on a shim on a
  collar; the upper is held by a collar over a shim (``deep_groove_pair``) or a wave washer
  (the preload);
* **the turntable**: a disc the side's two frame plates stand on, each grown a **foot**
  (:class:`TurretFoot`) to its top face and held by two printed **brackets** outside it
  (an M3 through the plate from inside, nut on the bracket; an M3 up through the turntable,
  nut on the bracket), placed clear of the drives' motors and sliders outside the plates;
* **the sensor**: a slotted optical endstop on standoffs on the base's top plate in front,
  a printed (opaque) half-circle flag at the turntable's rim whose edge is in the beam at
  yaw 0: the larger its radius, the less the switch's repeatability turns the yaw.

The cables loop loosely round the base (a spiral wrap): the shaft is solid.

**Its height** (:func:`turret_plan`): the turntable's top is under everything of the side
over the inputs' box: under every link (its pins' heads, the drives' hubs, the belts at the
motors) by what stands on the turntable (the shaft's collar, the hub's nuts, the brackets'
screws inside the stack), and under the motors and sliders outside the plates; and under
the tool wherever the flange passes over the base. A static part of the side below it at
assembly raises :class:`ConstructionError` (the estimate missed it).
"""

from __future__ import annotations

import itertools
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
from spiderpig.construction.rotary import SPACING, RotaryPreset
from spiderpig.construction.transmissions.belt import MIN_MESH
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.mechanism import Body
from spiderpig.shapes import (
    Cut,
    Rect,
    box,
    cut_holes,
    difference,
    disc,
    intersection,
    moved,
    ring,
    union,
)

BOSS_N = 3              # discs under the base's top plate the shaft is clamped through
BOSS_R = 18.5
BOSS_SCREW_R = 12.5     # the boss's screws' heads clear the collar clamping the shaft over it
FOOT_R = 6.0            # the frame plates' foot: a stadium with a flat underside
FOOT_L = 40.0           # its half length either side of the yaw axis
BRACKET = {"w": 12.0, "h": 16.0, "d": 16.0, "t": 5.0, "hole": 10.5}
MOTOR_GAP = 2.0         # the yaw motor's end over the base's bottom plate
BASE_EDGE = 5.0         # the base's plates past the motor's corner
COL_INSET = 6.0         # the columns in from the base's edge
SHOULDER_D = 18.5       # the shoulder plates' bore: on a 608's outer ring, clear of its inner
SEAT_FIT = 0.2          # a seat's bore over the bearing (bonded: :mod:`construction.pivots.ball`)
HOUSING_STOCK = "plate_6061_9p525"   # a milled housing's plate: 3/8 in 6061, past a 608
SLIP_FIT = 0.025        # a milled seat over its bearing, the slip fit's middle (mm)
SHAFT_HOLE = 8.1
COLLAR_HOLE = 20.0      # what turns round a collar: the pulley's bore drilled out, the drum's
ANCHOR_IN = 6.0         # the capstan's anchor screws in from the drum's rim
SCREW_HOLE = 3.4
NUT_H = 2.4
FLAG = {"t": 2.0, "flange": 3.0, "w": 8.0}   # printed opaque (black): IR passes thin PLA
FLAG_CLEAR = 10.0       # the flag's radius past the drive's (pulley's flange, drum's)
TIE_PCD = {"belt": 34.0, "capstan": 44.0}
"""The hub's clamping screws' circle per drive: 2 x the plate's thickness of web from the
bearing seat (SendCutSend's hole-to-edge rule); the 80-tooth pulley is drilled to it."""
GLUE = 0.003            # the epoxy kit's share per bearing bonded
PLATE_COLOR = "#b9bec5"
PRINT_COLOR = "#e8a33d"
STEEL = "#4a4a4a"


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
    loop: int | None               # the belt's stock loop (mm), the capstan's None
    travel_deg: float              # what the joint may turn, its range's width
    range_deg: tuple[float, float]
    brackets: tuple[tuple[str, float], ...]     # (plate: "outer" | "inner", x)
    reach: float                   # the brackets' screws through the turntable, out from
    #                                their plate's face (clear of the flag under it)
    faces: dict[str, float]        # each plate's outside face z
    columns: tuple[float, ...]     # the base's columns' standoffs (mm, stock)
    hub_r: float
    sensor_r: float
    notes: tuple[str, ...] = field(default=())
    mill: str = ""                 # the user's mill (milling.MILLS): the hub's housings milled

    def housing(self):
        """The milled bearing housing (two in the hub, the lower one turned over about x:
        its holes' pattern is symmetric about it): its stock, its seat pocketed to the
        bearing from the outer face (drawn at a slip fit's middle for retaining compound),
        the shoulder through, the hub's screws' holes drilled, the outline cut free
        (:class:`milling.MillPart`); ``None`` without a mill."""
        if not self.mill:
            return None
        from spiderpig.hardware.catalog import get
        from spiderpig.milling import MillPart, Op

        support = self.preset.parts[0]
        brg = get(support.bearing).dims
        pcd = TIE_PCD[self.preset.transmission] / 2
        od = float(brg["od"])
        ops = [Op("pocket", od + SLIP_FIT, depth=float(brg["w"]), purpose="the bearing's seat",
                  fit=f"{od:g} +0.01/+0.04 (a slip fit: try the bearing)"),
               Op("pocket", SHOULDER_D, purpose="the shoulder's bore (through)")]
        ops += [Op("drill", SCREW_HOLE, pcd * math.cos(math.radians(45 + 90 * k)),
                   pcd * math.sin(math.radians(45 + 90 * k)), purpose="the hub's screws")
                for k in range(4)]
        ops.append(Op("profile", 2 * self.hub_r, purpose="the outline"))
        return MillPart("hub_housing", HOUSING_STOCK, float(get(HOUSING_STOCK).dims["thickness"]),
                        2 * self.hub_r, tuple(ops), role="the turntable's bearing housing")

    @property
    def phi_m(self) -> float:
        return math.pi       # the motor behind the axis

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
    def spacing(self) -> float:
        return SPACING

    def anchor_angles(self) -> list[float]:
        """A capstan drum's two cord anchors (rad, the turret's frame at yaw 0): past the
        travel's ends either side of where the cord leaves toward the motor
        (:data:`transmissions.capstan.ANCHOR_MARGIN`), so it never leaves its groove."""
        from spiderpig.construction.transmissions.capstan import ANCHOR_MARGIN

        half = math.radians(self.travel_deg / 2 + ANCHOR_MARGIN)
        return [self.phi_m + half, self.phi_m - half]

    def drive_r(self) -> float:
        """The pitch radius the yaw is turned at (the large pulley's, the drum's cord)."""
        from spiderpig.construction.transmissions.capstan import CORD_D

        _, tr, _, _ = self.preset.parts
        if self.loop is not None:
            return float(tr.big["pitch_d"]) / 2
        return tr.drum_r + CORD_D / 2

    def drive_terms(self):
        _, tr, _, _ = self.preset.parts
        if self.loop is not None:
            return tr.terms(self.c, self.loop)
        return tr.terms(self.travel_deg)

    def error_terms(self):
        return self.preset.error_terms(self.spacing, self.drive_terms())


# -- the plan ------------------------------------------------------------------------------


def seat_n(t: float, w: float) -> int:
    """How many plates of ``t`` seat a bearing ``w`` wide (it stands back from the outer
    face by what's over)."""
    return math.ceil(w / t - 1e-9)


def _stock_column(need: float) -> tuple[float, ...]:
    """The fewest stock round standoffs (joined ones 12 mm or longer) reaching ``need``,
    the shortest such."""
    from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS

    lengths = sorted(M3_ROUND_STANDOFF_LENGTHS)
    for n in (1, 2, 3):
        combos = [c for c in itertools.combinations_with_replacement(lengths, n)
                  if (n == 1 or min(c) >= 12) and sum(c) >= need - 1e-9]
        if combos:
            return min(combos, key=lambda c: (sum(c), c))
    raise ConstructionError(f"turret: no stock standoff column reaches {need:.1f} mm")


def _lows(design, tool_plan) -> tuple[float, float, list]:
    """The lowest y inside the stack (links, pins, the drives' hubs and belts) and outside
    the plates (motors, sliders, pillar heads), and the outside footprints per plate:
    ``[(plate, kind, data)]``."""
    from spiderpig import construction
    from spiderpig.construction.transmissions import DriveGroup

    ctx: Context = design.ctx
    pts = ctx.topo.geometry.points
    lat = ctx.topo.geometry.lateral
    p = ctx.params
    pin_head = construction.axle(ctx.config.pin).dims(ctx, False).head
    pillar_head = construction.axle(ctx.config.pillar).dims(ctx, True).head
    r_link = max(p.link_radius, pin_head) + p.margin
    inside = math.inf
    for segs in ctx.topo.links.values():
        for n in {q for s in segs for q in s}:
            y = np.asarray(pts[n], dtype=float).reshape(-1, 2)[:, 1]
            inside = min(inside, float(y.min()) - lat.get(n, 0.0) - r_link)
    outside = math.inf
    prints: list = []
    for ax in ctx.topo.axes_of("frame"):
        xy = np.asarray(pts[ax.name], dtype=float).reshape(-1, 2)[0]
        outside = min(outside, float(xy[1]) - pillar_head)
        prints += [(plate, "circle", (xy, pillar_head)) for plate in ("outer", "inner")]
    for g in design.groups:
        if isinstance(g, DriveGroup):
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


def turret_plan(design, preset: RotaryPreset, tool_plan=None, mill: str = "") -> TurretPlan:
    """The turret under ``design``'s side (module docstring); ``mill``: the user's mill's
    key (``""``: none), which mills the hub's housings."""
    from spiderpig.construction.transmissions.capstan import CORD_D, FLANGE

    ctx: Context = design.ctx
    plan = design.plan
    support, tr, motor, _sensor = preset.parts
    t = ctx.sheet_t("frame")
    pitch = ctx.pitch
    pts = ctx.topo.geometry.points
    o = np.asarray(pts["O"] if "O" in pts else pts[ctx.topo.axes_of("frame")[0].name],
                   dtype=float).reshape(-1, 2)[0]
    x_a = float(o[0])
    faces = {"outer": float(plan.z(0)[0]), "inner": float(plan.z(plan.top)[1])}
    z_a = (faces["outer"] + faces["inner"]) / 2
    from spiderpig.hardware.catalog import get

    brg = get(support.bearing).dims
    w_b, _od_b = float(brg["w"]), float(brg["od"])
    collar = get("collar_8mm").dims
    collar_w = float(collar["w"])
    ns = seat_n(t, w_b)
    recess = ns * t - w_b
    h_sp = SPACING - w_b - 2 * t
    hub_h = 2 * ns * t + 2 * t + h_sp
    if mill:                       # two milled housings, each bearing flush with its face
        th = float(get(HOUSING_STOCK).dims["thickness"])
        if th < w_b + 1.0:
            raise ConstructionError(f"turret: {HOUSING_STOCK} leaves the bearing no shoulder")
        recess, hub_h = 0.0, SPACING + w_b
        h_sp = hub_h - 2 * th
    if h_sp < 3.0:
        raise ConstructionError(f"turret: bearings {SPACING:g} mm apart leave no spacer")
    spring = float(get(support.spring).dims["work_h"]) if support.preload else 0.5
    notes: list[str] = []

    # the drive: where the motor goes, what it leaves the hub's floor
    half_diag = motor.frame / math.sqrt(2)
    key = preset.transmission
    lo, hi = preset.range_deg
    travel = hi - lo
    loop = None
    if key == "belt":
        need = max(BOSS_R + half_diag + 1.0,
                   (float(tr.big["flange_d"]) + float(tr.small["flange_d"])) / 2 + 2.0)
        fits = [(c, L) for L in _loops()
                if (c := tr.centre_distance(L)) is not None and c >= need
                and tr.teeth_in_mesh(c) >= MIN_MESH]
        if not fits:
            raise ConstructionError("turret: no stock GT2 loop keeps the motor clear of the "
                                    "shaft's boss")
        c, loop = fits[0]
        drive_r = float(tr.big["flange_d"]) / 2
    else:
        c = tr.drum_r + tr.capstan_d / 2 + CORD_D
        drive_r = tr.drum_r + FLANGE
    r_base = c + half_diag + BASE_EDGE
    columns = _stock_column(motor.length + MOTOR_GAP)
    h: dict[str, float] = {"bottom": 0.0}
    h["bt_bot"] = t + sum(columns)
    h["bt_top"] = h["bt_bot"] + t
    if key == "belt":
        w = tr.width()
        hub_l = float(tr.small["hub_l"])
        h["drive_bot"] = h["bt_top"] + 0.5 + hub_l      # the pulleys' teeth from here
        h["hub_bot"] = h["drive_bot"] + w
    else:
        floor = h["bt_top"] + screw("bhcs", "3").head_h + 0.5   # over the motor's screws
        core_bot = floor + pitch
        reach = h["bt_bot"] + motor.shaft_len - 0.5     # the sleeve stops at the shaft's end
        n_core = math.floor((reach - core_bot) / pitch + 1e-9)
        from spiderpig.construction.transmissions.capstan import (
            ANCHOR_MARGIN,
            CORD_PITCH,
            WRAPS,
        )
        fit = ((n_core * pitch - 1.0) / CORD_PITCH - WRAPS) * 360.0 / tr.ratio()
        fit = min(fit, 360.0 - 2 * ANCHOR_MARGIN)
        if fit <= 0:
            raise ConstructionError("turret: the yaw motor's shaft holds no capstan wraps")
        if fit < travel - 1e-9:
            notes.append(f"the capstan's travel is {fit:.0f} deg (the wraps' walk on the "
                         f"motor's {motor.shaft_len:g} mm shaft), not the preset's "
                         f"{travel:.0f}")
            lo, hi = -fit / 2, fit / 2
            travel = fit
        h["drive_bot"] = floor
        h["core_bot"] = core_bot
        h["n_core"] = float(n_core)
        h["hub_bot"] = core_bot + n_core * pitch + pitch
        h["sleeve_top"] = core_bot + n_core * pitch
        if c - motor.shaft_d / 2 < drive_r + 0.5:      # the shaft past the sleeve, uncut
            raise ConstructionError("turret: the yaw motor's shaft meets the drum's flange")
    # the collar under the lower inner ring clears the one clamping the shaft
    b_top = h["hub_bot"] + recess - 0.5
    lift = h["bt_top"] + collar_w - (b_top - collar_w)
    if lift > 1e-9:
        if key != "belt":
            raise ConstructionError("turret: the drum leaves the hub's collar no room over "
                                    "the shaft's")
        h["hub_bot"] += lift
        h["drive_bot"] += lift
    h["shaft_top"] = h["bt_bot"] + motor.shaft_len      # the stock shaft, uncut
    if key == "belt" and h["hub_bot"] > h["shaft_top"] + 1e-9:
        raise ConstructionError(f"turret: the yaw motor's {motor.shaft_len:g} mm shaft doesn't "
                                "reach its pulley's top")
    h["hub_top"] = h["hub_bot"] + hub_h
    if h["shaft_top"] > h["hub_top"] - 1.0:
        raise ConstructionError("turret: the yaw motor's shaft reaches the turntable")
    h["table"] = h["hub_top"] + t
    d_top = h["hub_top"] - recess + spring + collar_w
    tie = screw("shcs", "3")
    protrude = max(d_top - h["table"], NUT_H + 1.0, tie.head_h)

    # how high the turntable stands: under the side, by what stands on it
    inside, outside, prints = _lows(design, tool_plan)
    head = BRACKET["hole"] + screw("shcs", "3").head_d / 2
    y_t = min(inside - max(head, protrude) - 1.0, outside - 1.0)

    # the brackets, clear of what stands outside each plate
    brackets: list[tuple[str, float]] = []
    for plate in ("outer", "inner"):
        xs = [x_a + v for v in np.arange(-(FOOT_L - BRACKET["w"] / 2 - 1),
                                          FOOT_L - BRACKET["w"] / 2 - 1 + 1e-9, 1.0)]
        ok = [float(x) for x in xs
              if _clear((float(x) - BRACKET["w"] / 2 - 1, float(y_t),
                         float(x) + BRACKET["w"] / 2 + 1, float(y_t) + BRACKET["h"] + 1),
                        prints, plate)]
        if len(ok) < 2 or max(ok) - min(ok) < BRACKET["w"] + 8:
            raise ConstructionError(f"turret: no room for two brackets outside the {plate} "
                                    f"frame plate's foot (the motors and sliders there "
                                    f"reach down to it)")
        brackets += [(plate, float(min(ok))), (plate, float(max(ok)))]
    hub_r = TIE_PCD[key] / 2 + SCREW_HOLE / 2 + 2 * t
    head_r = screw("shcs", "3").head_d / 2
    reach = BRACKET["hole"]
    depth = reach + BRACKET["hole"] - BRACKET["t"]      # as much leg past the hole as under
    corners, screws_r = [], []
    for plate, x in brackets:
        z0 = faces[plate]
        sgn = -1.0 if plate == "outer" else 1.0
        corners += [math.hypot(xx - x_a, zz - z_a)
                    for xx in (x - BRACKET["w"] / 2, x + BRACKET["w"] / 2)
                    for zz in (z0, z0 + sgn * depth)]
        screws_r.append(math.hypot(x - x_a, z0 + sgn * reach - z_a))
    corners += [math.hypot(FOOT_L + FOOT_R, faces[pl] - z_a) for pl in faces]
    r_table = max(max(corners) + 3.0, hub_r + 5.0)
    # the homing flag at the turntable's rim (its edge's place over the most radius), past
    # the drive and outside the brackets' screws under the turntable
    rw = FLAG["w"] / 2
    sensor_r = max(r_table - rw - 2.0, drive_r + FLAG_CLEAR,
                   max(screws_r) + head_r + 1.0 + rw)
    r_table = max(r_table, sensor_r + rw + 2.0)

    # the tool, wherever the flange passes over the base or the turntable
    if tool_plan is not None:
        from spiderpig.construction.tools import keepout

        f = np.asarray(pts[tool_plan.point], dtype=float)
        over = np.abs(f[:, 0] - x_a) < max(r_base, r_table) + keepout(tool_plan.tool)
        if over.any():
            low = float(f[over, 1].min()) - ctx.topo.geometry.lateral.get(
                tool_plan.point, 0.0) - tool_plan.tool.drop()
            if low - protrude - 1.0 < y_t:        # (over the collar and nuts it stands)
                y_t = low - protrude - 1.0
                notes.append("the turntable stands lower for the tool passing over it")
    y_ground = y_t - h["table"]
    return TurretPlan(preset, x_a, z_a, y_t, y_ground, h, r_base, r_table, c, loop, travel,
                      (lo, hi), tuple(brackets), reach, faces, columns, hub_r, sensor_r,
                      tuple(notes), mill)


def _loops():
    from spiderpig.hardware.motion_catalog import GT2_LOOPS

    return GT2_LOOPS


# -- the side's part: its feet -------------------------------------------------------------


class TurretFoot(Group):
    """The frame plates grown down to the turntable's top (a stadium with a flat underside,
    joined to each frame pivot) and drilled for the brackets' screws. No claims: nothing it
    adds is inside the stack (the brackets' screws' heads are under every link:
    :func:`turret_plan`)."""

    name = "turret foot"

    def __init__(self, tp: TurretPlan):
        self.tp = tp

    def claims(self, ctx: Context) -> list:
        return []

    def realize(self, build: Build, done: Realized) -> Realized:
        tp = self.tp
        out = Realized()
        r = build.ctx.params.frame_radius
        ends = ((tp.x_a - FOOT_L, tp.y_t + FOOT_R), (tp.x_a + FOOT_L, tp.y_t + FOOT_R))
        for key, plate in ((FRAME_OUTER, "outer"), (FRAME_INNER, "inner")):
            out.pad(key, ends[0], ends[1], FOOT_R)
            for ax in build.ctx.topo.axes_of("frame"):
                f = build.xy(ax.name)
                x = min(max(float(f[0]), ends[0][0]), ends[1][0])
                out.pad(key, tuple(f), (x, tp.y_t + max(r, FOOT_R)), r)
            for pl, x in tp.brackets:
                if pl == plate:
                    out.cut(key, Cut((x, tp.y_t + BRACKET["hole"]), SCREW_HOLE))
        return out


# -- the robot's parts ---------------------------------------------------------------------


def _hw(name, part, *, fab, bom_key=None, color=None, sheet=None) -> Body:
    return Body(name=name, part=part, color=color, fab=fab, bom_key=bom_key, sheet=sheet)


def _along_y(part):
    """Drawn along z (``disc((a, b), r, h0, h1)``), stood along the robot's y."""
    from build123d import Axis, Plane

    return part.rotate(Axis.X, -90).mirror(Plane.XY)


def turret_parts(tp: TurretPlan, ctx: Context) -> tuple[list[Body], list[str], list[BomLine],
                                                        dict]:
    """The turret's bodies (robot coordinates), the names of those that turn with the
    turntable, its purchases not drawn, and what the mechanism's meta says of it."""
    from spiderpig.construction.pivots.ball import bearing_solid
    from spiderpig.hardware.catalog import get
    from spiderpig.hardware.crank_catalog import m3_round_standoff

    support, tr, motor, sensor = tp.preset.parts
    key = tp.preset.transmission
    frame_sheet = ctx.sheet("frame")
    t = ctx.sheet_t("frame")
    pitch = ctx.pitch
    h = tp.h
    bodies: list[Body] = []
    turning: list[str] = []
    extras: list[BomLine] = []
    place = tp.place
    o = (0.0, 0.0)
    sk = screw("shcs", "3")
    bh = screw("bhcs", "3")

    def add(name, part, turns=False, **kw):
        bodies.append(_hw(name, place(part), **kw))
        if turns:
            turning.append(name)

    def length_for(need: float) -> float:
        try:
            return next(L for L in sk.lengths if need - 1e-6 <= L)
        except StopIteration:
            raise ConstructionError(f"turret: no stock M3 screw reaches {need:.1f} mm") from None

    m = np.array([tp.c * math.cos(tp.phi_m), tp.c * math.sin(tp.phi_m)])
    u = m / np.linalg.norm(m)
    v = np.array([-u[1], u[0]])
    hs = motor.hole_square / 2
    mscrews = [tuple(m + sx * hs * u + sy * hs * v) for sx in (-1, 1) for sy in (-1, 1)]
    col_r = tp.r_base - COL_INSET
    cols = [(col_r * math.cos(tp.phi_m + math.radians(45 + 90 * k)),
             col_r * math.sin(tp.phi_m + math.radians(45 + 90 * k))) for k in range(4)]
    corners = [m + sx * motor.frame / 2 * u + sy * motor.frame / 2 * v
               for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    from spiderpig.construction.plane import inside_poly, point_seg

    for cxy in cols:
        if inside_poly(cxy, corners) or min(point_seg(cxy, corners[i], corners[(i + 1) % 4])
                                            for i in range(4)) < 3.0 + 1.0:
            raise ConstructionError("turret: a base column meets the yaw motor")
    boss_screws = [(BOSS_SCREW_R * math.cos(math.radians(45 + 90 * k)),
                    BOSS_SCREW_R * math.sin(math.radians(45 + 90 * k))) for k in range(4)]
    board = get("optical_endstop").dims
    pitch_e = float(board["hole_pitch"])
    e_x = tp.sensor_r + 2.0               # the board outward of the slot, over no pulley
    e_holes = [(e_x, -pitch_e / 2), (e_x, pitch_e / 2)]

    # the base
    add("base_bottom", cut_holes(disc(o, tp.r_base, 0.0, t), [Cut(c, SCREW_HOLE) for c in cols],
                                 0.0, t), fab="laser", color=PLATE_COLOR, sheet=frame_sheet)
    belt = key == "belt"
    slot = 2 * 2.0 if belt else 0.0           # the belt's tension: the motor's travel
    top_cuts: list[Cut | Rect] = [Cut(o, SHAFT_HOLE)]
    top_cuts += [Cut(c, SCREW_HOLE) for c in cols + boss_screws]
    top_cuts += [Cut(e, SCREW_HOLE) for e in e_holes]
    ang = math.atan2(u[1], u[0])
    top_cuts += [Rect(xy, (SCREW_HOLE + slot, SCREW_HOLE), ang) for xy in mscrews]
    top_cuts.append(Rect(tuple(m), (motor.boss_d + 0.4 + slot, motor.boss_d + 0.4), ang)
                    if belt else Cut(tuple(m), motor.boss_d + 0.4))
    add("base_top", cut_holes(disc(o, tp.r_base, h["bt_bot"], h["bt_top"]), top_cuts,
                              h["bt_bot"], h["bt_top"]), fab="laser", color=PLATE_COLOR,
        sheet=frame_sheet)
    for i in range(BOSS_N):
        z1 = h["bt_bot"] - i * t
        add(f"base_boss{i}", cut_holes(disc(o, BOSS_R, z1 - t, z1),
                                       [Cut(o, SHAFT_HOLE)] + [Cut(c, SCREW_HOLE)
                                                               for c in boss_screws],
                                       z1 - t, z1), fab="laser", color=PLATE_COLOR,
            sheet=frame_sheet)
    blen = length_for((BOSS_N + 1) * t + NUT_H)
    for i, xy in enumerate(boss_screws):
        add(f"base_boss_screw{i}", screw_solid(xy, sk, h["bt_top"], blen, up=False),
            fab="purchased", bom_key=sk.key(blen), color=STEEL)
    extras.append(BomLine("m3_nut", len(boss_screws), "turret: the boss's screws"))
    z = t
    for i, xy in enumerate(cols):
        for j, L in enumerate(tp.columns):
            add(f"base_col{i}_{j}", ring(xy, 6.0, 2.5, z, z + L),
                fab="purchased", bom_key=m3_round_standoff(L), color="#3a3a3a")
            z += L
        z = t
    clen = length_for(t + 4.0)
    for i, xy in enumerate(cols):
        add(f"base_col_screw{i}a", screw_solid(xy, sk, 0.0, clen, up=True), fab="purchased",
            bom_key=sk.key(clen), color=STEEL)
        add(f"base_col_screw{i}b", screw_solid(xy, sk, h["bt_top"], clen, up=False),
            fab="purchased", bom_key=sk.key(clen), color=STEEL)
    if len(tp.columns) > 1:
        from spiderpig.hardware.crank_catalog import m3_set_screw

        extras.append(BomLine(m3_set_screw(8), 4 * (len(tp.columns) - 1),
                              "turret: the columns' joints"))

    # the yaw motor under the base's top plate, its shaft up
    shaft_len = round(h["shaft_top"] - h["bt_bot"], 1)
    mbody = union([box(tuple(m), (motor.frame, motor.frame, motor.length),
                       h["bt_bot"] - motor.length, ang),
                   disc(tuple(m), motor.boss_d / 2 - 0.2, h["bt_bot"], h["bt_bot"] + motor.boss_h),
                   disc(tuple(m), motor.shaft_d / 2 - 0.05, h["bt_bot"], h["bt_bot"] + shaft_len)])
    add("yaw_motor", mbody, fab="purchased", bom_key=motor.bom_key, color="#2b2b2b")
    mlen = next(L for L in bh.lengths if t + motor.hole_depth * 0.8 - 1e-6 <= L)
    for i, xy in enumerate(mscrews):
        add(f"yaw_motor_screw{i}", screw_solid(xy, bh, h["bt_top"], mlen, up=False),
            fab="purchased", bom_key=bh.key(mlen), color=STEEL)

    # the shaft and its collars
    cw = float(get("collar_8mm").dims["w"])
    cod = float(get("collar_8mm").dims["od"])
    brg = get(support.bearing).dims
    w_b, od_b = float(brg["w"]), float(brg["od"])
    ns = seat_n(t, w_b)
    recess = 0.0 if tp.mill else ns * t - w_b      # (a milled seat: flush with its face)
    lower = h["hub_bot"] + recess
    upper = h["hub_top"] - recess - w_b
    spring = float(get(support.spring).dims["work_h"]) if support.preload else 0.5
    c_bot = h["bt_bot"] - BOSS_N * t - cw
    d_top = h["hub_top"] - recess + spring + cw
    shaft_l = d_top - c_bot
    if shaft_l > float(get("shaft_8mm_h6_100").dims["length"]) + 1e-9:
        raise ConstructionError(f"turret: the shaft would be {shaft_l:.0f} mm, over the stock "
                                "100 mm")
    add("yaw_shaft", disc(o, 4.0, c_bot, d_top), fab="purchased", bom_key="shaft_8mm_h6_100",
        color="#9aa0a6")
    for name, z0 in (("yaw_collar_a", h["bt_top"]), ("yaw_collar_c", c_bot),
                     ("yaw_collar_b", lower - 0.5 - cw), ("yaw_collar_d", d_top - cw)):
        add(name, ring(o, cod, 8.0, z0, z0 + cw), fab="purchased", bom_key="collar_8mm",
            color=STEEL)
    add("yaw_shim_lower", ring(o, 12.0, 8.2, lower - 0.5, lower), fab="purchased",
        bom_key="shim_8x12x0p5", color=STEEL)
    top_key = support.spring if support.preload else "shim_8x12x0p5"
    add("yaw_shim_upper", ring(o, 12.0, 8.4, upper + w_b, upper + w_b + spring),
        fab="purchased", bom_key=top_key, color=STEEL)
    for name, z0 in (("yaw_bearing_lower", lower), ("yaw_bearing_upper", upper)):
        add(name, bearing_solid(support.bearing, o, z0), fab="purchased",
            bom_key=support.bearing, color="#d0d0d0")
        extras.append(BomLine("retaining_compound_641", 0.05, f"turret: {name} in its milled "
                              "seat") if tp.mill else
                      BomLine("epoxy_2part", GLUE, f"turret: {name} into its seat"))

    # the hub and the turntable
    pcd = TIE_PCD[key] / 2
    ties = [(pcd * math.cos(math.radians(45 + 90 * k)), pcd * math.sin(math.radians(45 + 90 * k)))
            for k in range(4)]
    tie_cuts = [Cut(xy, SCREW_HOLE) for xy in ties]
    z = h["hub_bot"]
    seat = od_b + SEAT_FIT
    housing = tp.housing()
    milled: dict[str, dict] = {}
    if housing is not None:        # two milled housings, the printed spacer between
        from build123d import Axis, Location

        th = housing.thickness
        solid = housing.solid()    # its seat bored from its top face
        lower_h = solid.rotate(Axis.X, 180).moved(Location((0, 0, z + th)))   # seat down
        upper_h = solid.moved(Location((0, 0, h["hub_top"] - th)))
        for name, part in (("hub_housing_lower", lower_h), ("hub_housing_upper", upper_h)):
            add(name, part, turns=True, fab="milled", color=PLATE_COLOR, sheet=housing.stock)
            milled[name] = housing.as_dict()
        sp0, sp1 = z + th, h["hub_top"] - th
    else:                          # laser-cut seats and shoulders, the spacer between
        stacks = (("hub_seat_lower", z, seat, ns), ("hub_shoulder_lower", z + ns * t,
                                                    SHOULDER_D, 1),
                  ("hub_shoulder_upper", upper - t, SHOULDER_D, 1),
                  ("hub_seat_upper", upper, seat, ns))
        for stem, z0, bore, n in stacks:
            for i in range(n):
                zz = z0 + i * t
                add(f"{stem}{i}" if n > 1 or "seat" in stem else stem,
                    cut_holes(disc(o, tp.hub_r, zz, zz + t), [Cut(o, bore), *tie_cuts], zz,
                              zz + t), turns=True, fab="laser", color=PLATE_COLOR,
                    sheet=frame_sheet)
        sp0, sp1 = z + (ns + 1) * t, upper - t
    add("hub_spacer", cut_holes(disc(o, tp.hub_r, sp0, sp1),
                                [Cut(o, SHOULDER_D + 2.0), *tie_cuts], sp0, sp1),
        turns=True, fab="printed", color=PRINT_COLOR)
    table_cuts: list[Cut | Rect] = [Cut(o, COLLAR_HOLE), *tie_cuts]
    # the flag's screws: their heads on the turntable between the frame plates' feet
    head_r = sk.head_d / 2
    room = min(tp.z_a - tp.faces["outer"], tp.faces["inner"] - tp.z_a) - t - head_r - 1.0
    fa = math.asin(min(1.0, max(0.0, room) / tp.sensor_r))
    flag_screws = [(tp.sensor_r * math.cos(a), tp.sensor_r * math.sin(a))
                   for a in (fa / 2, math.pi - fa / 2)]
    table_cuts += [Cut(xy, SCREW_HOLE) for xy in flag_screws]
    for plate, x in tp.brackets:
        zc = tp.faces[plate] + (-1 if plate == "outer" else 1) * tp.reach
        table_cuts.append(Cut(tp.local_ab(x, zc), SCREW_HOLE))
    add("turntable", cut_holes(disc(o, tp.r_table, h["hub_top"], h["table"]), table_cuts,
                               h["hub_top"], h["table"]), turns=True, fab="laser",
        color=PLATE_COLOR, sheet=frame_sheet)

    # the drive under the hub
    if belt:
        big, small = tr.big, tr.small
        w = tr.width()
        ft = float(big["flange_t"])
        z0 = h["drive_bot"]
        large = union([ring(o, float(big["flange_d"]), COLLAR_HOLE, z0, z0 + ft),
                       ring(o, float(big["od"]), COLLAR_HOLE, z0 + ft, z0 + w - ft),
                       ring(o, float(big["flange_d"]), COLLAR_HOLE, z0 + w - ft, z0 + w)])
        from spiderpig.shapes import drill

        large = drill(large, [(xy, SCREW_HOLE / 2) for xy in ties], z0, z0 + w)
        add("yaw_pulley_large", large, turns=True, fab="purchased", bom_key=tr.large_key,
            color="#c9cdd2")
        sb = float(small["bore"])
        sft = float(small["flange_t"])
        small_part = union([
            ring(tuple(m), float(small["hub_d"]), sb, z0 - float(small["hub_l"]), z0),
            ring(tuple(m), float(small["flange_d"]), sb, z0, z0 + sft),
            ring(tuple(m), float(small["od"]), sb, z0 + sft, z0 + w - sft),
            ring(tuple(m), float(small["flange_d"]), sb, z0 + w - sft, z0 + w)])
        add("yaw_pulley_small", small_part, fab="purchased", bom_key=tr.small_key,
            color="#c9cdd2")
        from spiderpig.construction.transmissions.belt import _belt_loop
        from spiderpig.hardware.motion_catalog import gt2_loop

        zm = z0 + w / 2
        add("yaw_belt", _belt_loop(o, tuple(m), float(big["od"]) / 2, float(small["od"]) / 2,
                                   zm - 3.0, zm + 3.0), fab="purchased",
            bom_key=gt2_loop(int(tp.loop or 0)), color="#202020")
        head_z = z0
        tlen = length_for(h["table"] - head_z + NUT_H)
    else:
        from spiderpig.construction.transmissions.capstan import FLANGE, NOTCH_W, WRAPS

        n_core = int(h["n_core"])
        layers = [h["drive_bot"] + i * pitch for i in range(n_core + 2)]
        r_a = tr.drum_r - ANCHOR_IN
        anchors = [(r_a * math.cos(a), r_a * math.sin(a)) for a in tp.anchor_angles()]
        notches = [Rect(((r_a + tr.drum_r) / 2 * math.cos(a), (r_a + tr.drum_r) / 2 * math.sin(a)),
                        (ANCHOR_IN + 2.0, NOTCH_W), a) for a in tp.anchor_angles()]
        for i, z0 in enumerate(layers):
            flange = i in (0, n_core + 1)
            r = tr.drum_r + (FLANGE if flange else 0.0)
            bore = 32.0 if i == 0 else COLLAR_HOLE   # (the bottom one over the boss's heads)
            holes: list[Cut | Rect] = [Cut(xy, 6.2 if i == 0 else SCREW_HOLE)
                                       for xy in ties + anchors]
            if not flange:
                holes += notches
            add(f"yaw_drum{i}", cut_holes(ring(o, 2 * r, bore, z0, z0 + pitch), holes, z0,
                                          z0 + pitch), turns=True, fab="laser",
                color="#7fb3e0", sheet=ctx.sheet("link"))
        alen = length_for(h["hub_bot"] - (h["drive_bot"] + pitch) + NUT_H)
        for i, xy in enumerate(anchors):          # each pinches the cord's end between plates
            add(f"yaw_anchor{i}", screw_solid(xy, sk, h["drive_bot"] + pitch, alen, up=True),
                turns=True, fab="purchased", bom_key=sk.key(alen), color=STEEL)
        extras.append(BomLine("m3_nut", len(anchors), "turret: the capstan's anchor screws"))
        add("yaw_capstan", ring(tuple(m), tr.capstan_d, motor.shaft_d + 0.1, h["core_bot"],
                                h["sleeve_top"]), fab="purchased", bom_key=tr.sleeve_key,
            color="#c9cdd2")
        arc = 2 * tr.drum_r * math.radians(tp.travel_deg + 80.0)
        extras += [BomLine("dyneema_1mm", round((arc + WRAPS * math.pi * tr.capstan_d + 200.0)
                                                / 1000.0, 3), "turret: the capstan's cord"),
                   BomLine("epoxy_2part", 0.003, "turret: the sleeve on the motor's shaft")]
        head_z = h["drive_bot"] + pitch
        tlen = length_for(h["table"] - head_z + NUT_H)
    for i, xy in enumerate(ties):
        add(f"hub_screw{i}", screw_solid(xy, sk, head_z, tlen, up=True), turns=True,
            fab="purchased", bom_key=sk.key(tlen), color=STEEL)
    extras.append(BomLine("m3_nut", len(ties), "turret: the hub's screws, on the turntable"))

    # the sensor: an endstop on standoffs, a half-circle flag under the turntable
    if sensor is not None:
        from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS

        bw, bd, bt = (float(v) for v in board["board"])
        body_h = float(board["body"][2])
        slot_w = float(board["slot_w"])
        avail = h["hub_top"] - FLAG["flange"] - 6.0 - (bt + body_h - 4.0) - h["bt_top"]
        L = max((x for x in M3_ROUND_STANDOFF_LENGTHS if x <= avail), default=None)
        if L is None:
            raise ConstructionError("turret: no room for the endstop under the turntable")
        b_bot = h["bt_top"] + L
        fin_bot = b_bot + bt + body_h - 4.0
        for i, xy in enumerate(e_holes):
            add(f"yaw_endstop_post{i}", ring(xy, 6.0, 2.5, h["bt_top"], b_bot),
                fab="purchased", bom_key=m3_round_standoff(L), color="#3a3a3a")
        sensor_body = union([box((e_x, 0.0), (bd, bw, bt), b_bot),
                             box((tp.sensor_r - slot_w / 2 - 2.0, 0.0),
                                 (4.0, float(board["body"][0]), body_h), b_bot + bt),
                             box((tp.sensor_r + slot_w / 2 + 2.0, 0.0),
                                 (4.0, float(board["body"][0]), body_h), b_bot + bt)])
        sensor_body = cut_holes(sensor_body, [Cut(xy, SCREW_HOLE) for xy in e_holes], b_bot,
                                b_bot + bt)
        add("yaw_endstop", sensor_body, fab="purchased", bom_key=sensor.bom_key,
            color="#1f5f9f")
        fl0 = h["hub_top"] - FLAG["flange"]
        half_plane = box((0.0, tp.sensor_r + 10.0), (2 * tp.sensor_r + 40, 2 * tp.sensor_r + 20,
                                                   h["hub_top"] - fin_bot + 2), fin_bot - 1)
        rw = FLAG["w"] / 2
        flag = union([ring(o, 2 * (tp.sensor_r + rw), 2 * (tp.sensor_r - rw), fl0,
                           h["hub_top"]),
                      ring(o, 2 * (tp.sensor_r + FLAG["t"] / 2), 2 * (tp.sensor_r - FLAG["t"] / 2),
                           fin_bot, fl0)])
        flag = intersection(flag, half_plane)
        flag = cut_holes(flag, [Cut(xy, SCREW_HOLE) for xy in flag_screws], fin_bot,
                         h["hub_top"])
        add("yaw_flag", flag, turns=True, fab="printed", color=PRINT_COLOR)
        slen = length_for(t + 4.0)
        for i, xy in enumerate(e_holes):
            add(f"yaw_endstop_screw{i}a", screw_solid(xy, sk, b_bot + bt, slen, up=False),
                fab="purchased", bom_key=sk.key(slen), color=STEEL)
            add(f"yaw_endstop_screw{i}b", screw_solid(xy, sk, h["bt_bot"], slen, up=True),
                fab="purchased", bom_key=sk.key(slen), color=STEEL)
        flen = length_for(t + FLAG["flange"] + NUT_H)
        for i, xy in enumerate(flag_screws):
            add(f"yaw_flag_screw{i}", screw_solid(xy, sk, h["table"], flen, up=False),
                turns=True, fab="purchased", bom_key=sk.key(flen), color=STEEL)
        extras.append(BomLine("m3_nut", len(flag_screws), "turret: the flag's screws"))

    # the brackets (robot coordinates: they're at the plates)
    bk = BRACKET
    for i, (plate, x) in enumerate(tp.brackets):
        face = tp.faces[plate]
        sgn = -1.0 if plate == "outer" else 1.0
        zv = tuple(sorted((face, face + sgn * bk["t"])))
        zh = tuple(sorted((face, face + sgn * (tp.reach + bk["hole"] - bk["t"]))))
        part = union([box((x, tp.y_t + bk["h"] / 2), (bk["w"], bk["h"], zv[1] - zv[0]), zv[0]),
                      box((x, tp.y_t + bk["t"] / 2), (bk["w"], bk["t"], zh[1] - zh[0]), zh[0])])
        zc = face + sgn * tp.reach
        part = cut_holes(part, [Cut((x, tp.y_t + bk["hole"]), SCREW_HOLE)], zv[0], zv[1])
        part = difference(part, _along_y(disc((x, zc), SCREW_HOLE / 2, tp.y_t - 1,
                                              tp.y_t + bk["t"] + 1)))
        bodies.append(_hw(f"foot_bracket{i}", part, fab="printed", color=PRINT_COLOR))
        turning.append(f"foot_bracket{i}")
        inner_face = face + (ctx.sheet_t("frame") if plate == "outer"
                             else -ctx.sheet_t("frame"))
        L1 = length_for(ctx.sheet_t("frame") + bk["t"] + NUT_H)
        s1 = screw_solid((x, tp.y_t + bk["hole"]), sk, inner_face, L1, up=plate == "inner")
        bodies.append(_hw(f"foot_screw{i}a", s1, fab="purchased", bom_key=sk.key(L1),
                          color=STEEL))
        L2 = length_for(t + bk["t"] + NUT_H)
        s2 = _along_y(screw_solid((x, zc), sk, tp.y_t - t, L2, up=True))
        bodies.append(_hw(f"foot_screw{i}b", s2, fab="purchased", bom_key=sk.key(L2),
                          color=STEEL))
        turning += [f"foot_screw{i}a", f"foot_screw{i}b"]
    extras.append(BomLine("m3_nut", 2 * len(tp.brackets), "turret: the brackets' screws"))
    extras.append(BomLine("spiral_wrap_6mm", 1.0, "turret: the cables' loop round the base"))
    preload = (f"set the top collar so the wave washer stands at its working "
               f"{float(get(support.spring).dims['work_h']):g} mm (a feeler gauge under the "
               "collar's face), " if support.preload else "")
    drive = ("drill the 80-tooth pulley's bore out to "
             f"{COLLAR_HOLE:g} mm and its four 3.4 mm holes on a {TIE_PCD[key]:g} mm circle "
             "on a drill press, centred on its own bore; tension the belt by sliding the "
             "motor in its slots before tightening its screws"
             if belt else "lead the cord in through one anchor's notch round its screw, round "
             "the groove to the sleeve, its turns round it, back round the groove and in "
             "through the other notch; pull it taut and tighten both anchors")
    assembly = [
        "Bolt the boss under the base's top plate, slide the shaft through and clamp it with "
        "the two collars, square to the plate (a square against it as they close).",
        ("Press the bearings into the milled housings' seats with retaining compound on "
         "their outer rings (a slip fit: by hand; on aluminium the compound wants its "
         "primer, Loctite 7649, or a day's cure), then, before it sets, fit the hub on the "
         "shaft with both bearings and tighten its screws there: the shaft lines the two "
         "seats up (the screws' holes alone leave them 0.4 mm apart)." if tp.mill else
         "Bond the bearings into the hub's seats with the hub's plates stacked on the shaft "
         "itself (the shaft centres them while the epoxy cures)."),
        f"For the drive: {drive}.",
        "Centre the pulley or drum on the hub with a sleeve on the shaft before the hub's "
        "screws are tightened (its eccentricity is the yaw's once-a-turn error).",
        f"Lower the hub onto the lower collar and its shim; {preload}clamp the top collar.",
        "Print the homing flag in black (opaque): the endstop's infrared passes thin pale "
        "plastic.",
        "Loop the cables loosely round the base in the spiral wrap: the yaw turns them."]
    fastened = [(b.name, "yaw_motor") for b in bodies if b.name.startswith("yaw_motor_screw")]
    names = {b.name for b in bodies}
    for b in bodies:
        if b.name.startswith("base_col_screw"):          # into its column's end standoff
            i, end = b.name[len("base_col_screw"):-1], b.name[-1]
            j = 0 if end == "a" else len(tp.columns) - 1
            fastened.append((b.name, f"base_col{i}_{j}"))
        if b.name.startswith("yaw_endstop_screw"):       # into its post
            post = f"yaw_endstop_post{b.name[len('yaw_endstop_screw'):-1]}"
            if post in names:
                fastened.append((b.name, post))
    info = {"preset": tp.preset.key, "support": tp.preset.support,
            "transmission": key, "motor": motor.key,
            "sensor": tp.preset.sensor, "ratio": round(tr.ratio(None), 4),
            "range_deg": [round(v, 2) for v in tp.range_deg],
            "axis": [tp.x_a, round(tp.y_ground, 3), tp.z_a],
            "turntable_y": round(tp.y_t, 3), "height_mm": round(tp.h["table"], 3),
            "base_r": round(tp.r_base, 2), "table_r": round(tp.r_table, 2),
            "centre_mm": round(tp.c, 3), "belt_mm": tp.loop,
            "shaft_mm": round(shaft_l, 1), "motor_shaft_mm": shaft_len,
            "columns": list(tp.columns), "notes": list(tp.notes), "fastened": fastened,
            "parts": sorted(b.name for b in bodies), "assembly": assembly,
            "mill": tp.mill, "milled": milled}
    return bodies, turning, extras, info
