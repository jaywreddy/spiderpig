"""The MT4's turret base: the bearings in the base, the shaft turning (2026-10-12, round 7
of the arm's review, the turret's variant C; the support ``base_pair``).

Stock parts only but for four cut plates (the user's: "only stock parts except for the
plates"). Bottom up, in the turret's frame (:class:`.turret.TurretPlan`):

* **the bottom plate**, 1/8 in 5052, screwed to the bench through four :data:`FOOT` mm
  spacers (the lower bearing and the columns' screw heads stand under it; on rubber feet
  a base this light rocks and walks under the arm's moment: round 7's review). The slides
  and the reference sheet lie on the bench in front (:func:`table_reach`):
  a work plate in one piece with it, as round 7's review asked, would be some 400 mm long,
  past the 300 mm blank the sheets are nested on, and the camera's homing re-references
  the sheet at every power-up anyway (:mod:`control.vision`);
* **four columns** of 7 mm hex M4 standoffs (:data:`~spiderpig.hardware.robot_catalog.
  M4_HEX_STANDOFF_LENGTHS`, the shortest past the yaw motor), two in front, two behind the
  motor, an M4 button head through each plate into each end;
* **the top plate**, 1/8 in 5052, the yaw motor hanging under it on slots along the belt
  (its tension), its shaft up through it;
* **the bearings**: an F608ZZ bonded into each plate, its flange on the plate's top face,
  the bearings' centres the column's length and a plate apart (about four times the
  classic hub's: their play tilts the axis a quarter as much); the 8 mm h6 **shaft** turns
  in them, slid through both while the retaining compound cures (the jig that lines them
  up; the columns tightened first, their holes' play past the compound's gap). The upper
  inner ring is clamped between two collars (on 0.5 mm shims clear of the seals): it holds
  the shaft both ways. The lower one floats on the shaft, a wave washer under a collar
  pressing it down: the pair preloaded, the washer's force against the upper's;
* **the drive** over the top plate: the 80-tooth plate pulley on the shaft over the upper
  collar (its own bore centres it, four 3.4 mm holes drilled on its 30 mm circle), the
  16-tooth on the motor's uncut shaft, a stock loop;
* **the turntable**, 1/8 in 5052, over a spacer disc (the frame sheet) that lifts it past
  the motor's shaft and the small pulley's flange; four M3 screws through the turntable,
  the spacer and the pulley, nuts under the pulley; an M4 screw and washer in the shaft's
  tapped end clamp the stack down on the over collar (a collar pushed down by hand clamps
  too lightly to hold the arm's moment); the spacer runs out to the small pulley under the
  turntable, which overhangs it less;
* **the homing flag**: an M2 screw hanging from the turntable outside the belt and the
  small pulley, its tip about 1 mm over the floor of a slotted optical endstop on posts on
  the top plate (in slots: the board centred on the flag at assembly), placed where
  the flag is just past the yaw's lower end: homing turns down until the beam breaks, as
  the MT4 homes to a limit switch (the camera then fixes the zero: :mod:`control.vision`);
* **the feet**: the frame plates' feet held to the turntable by stock steel angle brackets
  (:data:`STOCK_BRACKET`) in place of the printed ones.

The base stays inside a keep-out round the axis (:func:`keep_radius`: the turntable, the
switch, the base's round front; behind, its tail round the motor): only the side's links
over it must clear the turntable (:func:`.turret._lows`), the rest pass lower, so the tool
reaches the table.
"""

from __future__ import annotations

import math

import numpy as np

from spiderpig.construction.base import ConstructionError, Context
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.mechanism import Body
from spiderpig.shapes import Cut, Rect, box, cut_holes, disc, ring, union

BASE_SHEET = "al5052_3p2mm"     # the base's two plates and the turntable: 5052 as stiff as
#                                 6061 (its strength unused) at half SendCutSend's rate
STOCK_BRACKET = {"w": 16.0, "h": 20.0, "d": 20.0, "t": 2.0, "hole": 10.0}
"""The stock steel angle bracket (``angle_bracket_20``): its width, its legs (``h`` up the
frame plate, ``d`` out along the turntable), its thickness and each leg's hole from the
corner."""
SHIM = 0.5                      # a shim under a collar on an inner ring (clear of the seal)
FOOT = 6.0                      # the bench spacers under the bottom plate (m4_spacer_6)
SHAFT_KEY = "shaft_8mm_h6_tapped"
HEX_R = 7.0 / math.sqrt(3)      # a 7 mm hex standoff's corner radius
COL_GAP = 4.0                   # a column clear of the motor's body
TAIL_EDGE = 4.0                 # the plates past a back column's corner
FLAG_GAP = 2.0                  # the flag past the small pulley's flange
PLATE_COLOR = "#b9bec5"
STEEL = "#4a4a4a"
E_ALU = 69e3                    # N/mm^2


def _sheet_t(key: str) -> float:
    from spiderpig.hardware.catalog import sheet_thickness

    return sheet_thickness(key)


def heights(ctx: Context, support, tr, motor) -> tuple[dict[str, float], tuple[float, ...],
                                                       float, float]:
    """``(h, columns, spacing, protrude)``: the turret's heights from the bottom plate's
    underside (module docstring), its columns' standoffs, the bearings' centres apart, and
    how far over the turntable its top collar stands."""
    from spiderpig.hardware.catalog import get
    from spiderpig.hardware.robot_catalog import M4_HEX_STANDOFF_LENGTHS

    from .turret import MOTOR_GAP

    tb = _sheet_t(BASE_SHEET)
    ts = ctx.sheet_t("frame")
    brg = get(support.bearing).dims
    w_b, ft = float(brg["w"]), float(brg["flange_t"])
    cw = float(get("collar_8mm").dims["w"])
    spring = float(get(support.spring).dims["work_h"])
    need = motor.length + MOTOR_GAP
    L = next((float(x) for x in sorted(M4_HEX_STANDOFF_LENGTHS) if x >= need - 1e-9), None)
    if L is None:
        raise ConstructionError(f"turret: no stock M4 hex standoff reaches {need:.1f} mm")
    b0 = FOOT                                                    # the bench at 0
    h: dict[str, float] = {"bottom": 0.0, "bp_bot": b0, "bp_top": b0 + tb,
                           "bt_bot": b0 + tb + L, "bt_top": b0 + 2 * tb + L}
    h["lb_top"] = b0 + tb + ft
    h["lb_bot"] = h["lb_top"] - w_b
    h["ub_top"] = h["bt_top"] + ft
    h["ub_bot"] = h["ub_top"] - w_b
    h["hub_bot"], h["hub_top"] = h["lb_bot"], h["ub_top"]     # the bearings' span
    h["mid_bot"] = h["lb_top"] + spring                          # over the wave washer
    h["ul_bot"] = h["ub_bot"] - SHIM - cw                        # under the upper inner ring
    if h["ul_bot"] < h["mid_bot"] + cw + 1.0:
        raise ConstructionError("turret: the columns leave no room for the shaft's collars")
    h["lo_bot"] = h["ub_top"] + SHIM
    h["drive_bot"] = h["lo_bot"] + cw
    h["drive_top"] = h["drive_bot"] + tr.width()
    h["shaft_top"] = h["bt_bot"] + motor.shaft_len               # the motor's, uncut
    hub_floor = h["bt_top"] + screw("bhcs", "3").head_h + 0.5
    if h["drive_bot"] - float(tr.small["hub_l"]) < hub_floor:
        raise ConstructionError("turret: the small pulley's hub meets the motor's screws")
    if h["drive_top"] > h["shaft_top"] + 1e-9:
        raise ConstructionError(f"turret: the yaw motor's {motor.shaft_len:g} mm shaft doesn't "
                                "reach its pulley's top")
    lift = max(h["shaft_top"] + 1.0, h["drive_top"] + 0.5) - h["drive_top"]
    n_sp = max(1, math.ceil(lift / ts - 1e-9))
    h["n_spacer"] = float(n_sp)
    h["tt_bot"] = h["drive_top"] + n_sp * ts
    h["table"] = h["tt_bot"] + tb
    h["shaft_end"] = h["table"] - 0.5           # under the clamp screw's washer
    if h["lb_bot"] < 1.0:
        raise ConstructionError("turret: the lower bearing reaches the bench")
    spacing = (h["ub_bot"] + h["ub_top"]) / 2 - (h["lb_bot"] + h["lb_top"]) / 2
    washer = float(get("m4_washer").dims["t"])
    protrude = max(2.2 + washer, screw("shcs", "3").head_h)    # the clamp's M4 head, the ties'
    return h, (L,), spacing, protrude


def flag_radius(tr, c: float, screws_r: float) -> float:
    """The homing flag's radius: outside the small pulley's flange (the belt inside it) and
    the brackets' screws' heads under the turntable (``screws_r`` their farthest), the
    endstop's inner jaw clear of them."""
    from spiderpig.hardware.catalog import get

    from .turret import SCREW_HOLE

    board = get("optical_endstop").dims
    jaw = float(board["slot_w"]) / 2 + 4.0          # the slot's half and a jaw
    head_r = screw("shcs", "3").head_d / 2
    return max(c + float(tr.small["flange_d"]) / 2 + FLAG_GAP + 1.0,
               screws_r + head_r + 1.0 + jaw, float(tr.big["flange_d"]) / 2 + jaw + 1.0,
               SCREW_HOLE)


def board_outer(r_flag: float) -> float:
    """How far out the endstop's board reaches (its middle 2 mm outside the flag, as the
    classic turret's)."""
    from spiderpig.hardware.catalog import get

    return r_flag + 2.0 + float(get("optical_endstop").dims["board"][1]) / 2


def keep_radius(ctx: Context, tr, motor, c: float, faces: dict[str, float]) -> tuple[float,
                                                                                    float]:
    """``(front, r_flag)``: the base's round front's radius (the turntable under the feet's
    brackets, the flag, the endstop's board), which the side's links over it must clear, and
    the flag's radius (:func:`flag_radius`, with the brackets' screws' farthest bound)."""
    from .turret import FOOT_L, FOOT_R

    bk = STOCK_BRACKET
    half = (faces["inner"] - faces["outer"]) / 2
    screws_r = math.hypot(FOOT_L - bk["w"] / 2 - 1.0, half + bk["hole"])
    r_flag = flag_radius(tr, c, screws_r)
    feet = math.hypot(FOOT_L + FOOT_R, half + bk["d"]) + 3.0
    table = max(feet, r_flag + 4.0)
    return max(table, board_outer(r_flag)) + 1.0, r_flag


def tail(motor, c: float) -> tuple[float, float]:
    """The base's tail behind the axis round the yaw motor and the back columns: ``(depth
    behind the axis, half width)``."""
    back = motor.frame / 2 + COL_GAP + HEX_R
    return c + back + HEX_R + TAIL_EDGE, back + HEX_R + TAIL_EDGE


def columns_xy(tp, motor) -> list[tuple[float, float]]:
    """The four columns (the turret's frame): two in the base's round front at 45 deg, two
    behind the motor."""
    r = tp.h["front"] - HEX_R - TAIL_EDGE
    back = motor.frame / 2 + COL_GAP + HEX_R
    front = [(r * math.cos(a), r * math.sin(a)) for a in (math.radians(45), math.radians(-45))]
    return front + [(-(tp.c + back), s * back) for s in (1, -1)]


def table_reach(design, tool_plan, x_a: float, y_ground: float, keep_r: float
                ) -> tuple[str, float, float]:
    """Where the tool's tip gets down to the bench the base is screwed to (the slides and
    the reference sheet on it) over the arm's inputs' box: a note and the radial span
    ``(lo, hi)`` from the axis (``nan`` if it never gets down to it)."""
    from spiderpig import linkage
    from spiderpig.linkage.constraints import layout

    cfg = design.ctx.config
    lk = cfg.lk
    pts = linkage.LegSolution(1, 0.0, cfg.values, lk.key).at(*lk.box(41))
    f = np.asarray(pts[layout(lk).output_point], dtype=float).reshape(-1, 2)
    tip = f[:, 1] - design.ctx.topo.geometry.lateral.get(tool_plan.point, 0.0) \
        - tool_plan.tool.drop()
    top = y_ground                                  # the bench the base is screwed to
    r = f[:, 0] - x_a
    down = (tip <= top + 1e-6) & (r > keep_r)
    if not down.any():
        gap = float(tip[r > keep_r].min() - top) if (r > keep_r).any() else math.inf
        return (f"the tool's tip stops {gap:.1f} mm over the bench (a riser under the "
                "slides)", math.nan, math.nan)
    lo, hi = float(r[down].min()), float(r[down].max())
    return (f"the tool gets down to the bench from {lo:.0f} to {hi:.0f} mm out from the yaw "
            f"axis (past the base's {keep_r:.0f} mm front): a {hi - lo:.0f} mm deep band "
            f"round it", lo, hi)


def racking(tp) -> tuple[float, str]:
    """The base's columns racking under a tipping moment: the axis's tilt per N·m (rad) and
    a note. Each column a fixed-fixed hex beam between the plates (``12 E I / L^3``, the
    screws' joints stiffer than they are: a lower bound on the tilt); the moment a couple
    over the bearings' spacing, the plates' shear between them."""
    a = 7.0 / math.sqrt(3)
    i_hex = 5 * math.sqrt(3) / 16 * a ** 4 - math.pi * 3.3 ** 4 / 64
    L = sum(tp.columns)
    k = 4 * 12 * E_ALU * i_hex / L ** 3                  # N/mm
    per_nm = 1000.0 / tp.spacing / k / tp.spacing       # rad per N·m
    return per_nm, (f"the base's four {L:g} mm hex columns rack {per_nm * 1e6:.1f} urad per "
                    f"N·m of the arm's tipping moment (fixed-fixed beams, {k:.0f} N/mm: a lower "
                    "bound: thin plates and screwed ends rack some 2-4x that); it changes "
                    "with the pose as the moment does (accuracy, not repeatability)")


def _hw(name, part, *, fab, bom_key=None, color=None, sheet=None) -> Body:
    return Body(name=name, part=part, color=color, fab=fab, bom_key=bom_key, sheet=sheet)


def _m4(xy, z: float, length: float, up: bool):
    """An M4 button head bearing at ``z`` (ISO 7380: 7.6 x 2.2)."""
    s = 1.0 if up else -1.0
    return union([disc(xy, 3.8, *sorted((z - s * 2.2, z))),
                  disc(xy, 2.0, *sorted((z, z + s * length)))])


def parts(tp, ctx: Context) -> tuple[list[Body], list[str], list[BomLine], dict]:
    """The base_pair turret's bodies (robot coordinates), those that turn, its purchases not
    drawn and its meta (as :func:`.turret.turret_parts`)."""
    from spiderpig.construction.pivots.ball import bearing_solid
    from spiderpig.construction.plane import inside_poly, point_seg
    from spiderpig.construction.transmissions.belt import _belt_loop
    from spiderpig.hardware.catalog import get
    from spiderpig.hardware.crank_catalog import (
        M3_ROUND_STANDOFF_LENGTHS,
        M4_BHCS_LENGTHS,
        m3_round_standoff,
        m4_bhcs,
    )
    from spiderpig.hardware.motion_catalog import gt2_loop

    from .turret import GLUE, NUT_H, SCREW_HOLE, SHAFT_HOLE, _along_y

    support, tr, motor, sensor = tp.preset.parts
    h = tp.h
    tb = _sheet_t(BASE_SHEET)
    ts = ctx.sheet_t("frame")
    bodies: list[Body] = []
    turning: list[str] = []
    extras: list[BomLine] = []
    o = (0.0, 0.0)
    sk = screw("shcs", "3")
    bh = screw("bhcs", "3")

    def add(name, part, turns=False, **kw):
        bodies.append(_hw(name, tp.place(part), **kw))
        if turns:
            turning.append(name)

    def length_for(need: float, s=sk) -> float:
        try:
            return next(L for L in s.lengths if need - 1e-6 <= L)
        except StopIteration:
            raise ConstructionError(f"turret: no stock screw reaches {need:.1f} mm") from None

    # the motor behind the axis, its screws in slots along the belt
    m = np.array([-tp.c, 0.0])
    hs = motor.hole_square / 2
    mscrews = [(m[0] + sx * hs, sy * hs) for sx in (-1, 1) for sy in (-1, 1)]
    corners = [(m[0] + sx * motor.frame / 2, sy * motor.frame / 2)
               for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    cols = columns_xy(tp, motor)
    for cxy in cols:
        if inside_poly(cxy, corners) or min(point_seg(cxy, corners[i], corners[(i + 1) % 4])
                                            for i in range(4)) < HEX_R + 1.0:
            raise ConstructionError("turret: a base column meets the yaw motor")
    depth, half_w = tail(motor, tp.c)
    front = h["front"]

    # the endstop and its flag
    board = get("optical_endstop").dims
    bw, bd, bt = (float(v) for v in board["board"])
    body_h = float(board["body"][2])
    slot_w, slot_h = float(board["slot_w"]), float(board["slot_h"])
    nut2 = get("m2_nut").dims
    lo_deg = tp.range_deg[0]
    phi_s = math.radians(lo_deg - 3.0)             # the flag there just past the range's end
    u = np.array([math.cos(phi_s), math.sin(phi_s)])
    v = np.array([-u[1], u[0]])
    e_c = u * (tp.sensor_r + 2.0)
    pitch_e = float(board["hole_pitch"])
    e_holes = [tuple(e_c + s * pitch_e / 2 * v) for s in (-1, 1)]
    body_top_max = h["tt_bot"] - float(nut2["h"]) - 0.5
    avail = body_top_max - body_h - bt - h["bt_top"]
    post = max((x for x in M3_ROUND_STANDOFF_LENGTHS if x <= avail), default=None)
    if post is None:
        raise ConstructionError("turret: no room for the endstop under the turntable")
    b_bot = h["bt_top"] + post
    body_top = b_bot + bt + body_h
    s2 = screw("shcs", "2")
    floor = body_top - slot_h + 0.8                 # its tip about 1 mm over the slot's floor
    flen = max((L for L in s2.lengths if h["table"] - L >= floor - 1e-6), default=None)
    if flen is None or h["table"] - flen > body_top - 2.0:
        raise ConstructionError("turret: no stock M2 screw hangs into the endstop's slot")

    # the base
    tail_box = ((-depth / 2, 0.0), (depth, 2 * half_w))

    def outline(z0, z1):
        return union([disc(o, front, z0, z1), box(tail_box[0], (*tail_box[1], z1 - z0), z0)])

    lo_r, hi_r = h.get("reach_lo", math.nan), h.get("reach_hi", math.nan)
    col_cuts = [Cut(c, 4.4) for c in cols]
    b0, b1 = h["bp_bot"], h["bp_top"]
    r_b = front - 6.0
    bench = [(r_b, 0.0), (r_b * math.cos(math.radians(110)), r_b * math.sin(math.radians(110))),
             (r_b * math.cos(math.radians(110)), -r_b * math.sin(math.radians(110))),
             (-depth + 6.0, 0.0)]
    add("base_bottom", cut_holes(outline(b0, b1), [Cut(o, 22.0 + 0.2), *col_cuts,
                                                   *(Cut(xy, 4.4) for xy in bench)], b0, b1),
        fab="laser", color=PLATE_COLOR, sheet=BASE_SHEET)
    for i, xy in enumerate(bench):                  # screwed to the bench on spacers
        add(f"base_spacer{i}", ring(xy, 8.0, 4.2, 0.0, b0), fab="purchased",
            bom_key="m4_spacer_6", color="#3a3a3a")
    slot = 4.0                                      # the belt's tension: the motor's travel
    top_cuts: list[Cut | Rect] = [Cut(o, 22.0 + 0.2), *col_cuts]
    top_cuts += [Rect(e, (SCREW_HOLE + 2.0, SCREW_HOLE), phi_s) for e in e_holes]   # slots:
    #                                                 the board centred on the flag at assembly
    top_cuts += [Rect(xy, (SCREW_HOLE + slot, SCREW_HOLE), 0.0) for xy in mscrews]
    top_cuts.append(Rect(tuple(m), (motor.boss_d + 0.4 + slot, motor.boss_d + 0.4), 0.0))
    add("base_top", cut_holes(outline(h["bt_bot"], h["bt_top"]), top_cuts, h["bt_bot"],
                              h["bt_top"]), fab="laser", color=PLATE_COLOR, sheet=BASE_SHEET)
    L = sum(tp.columns)
    for i, xy in enumerate(cols):
        hexa = box(xy, (7.0, 7.0 / math.sqrt(3) * 2, L), b1)
        add(f"base_col{i}_0", cut_holes(hexa, [Cut(xy, 3.3)], b1, b1 + L), fab="purchased",
            bom_key=f"m4_hex_standoff_{L:g}", color="#3a3a3a")
    clen = next((x for x in M4_BHCS_LENGTHS if x >= tb + 4.0 - 1e-6), M4_BHCS_LENGTHS[-1])
    for i, xy in enumerate(cols):
        add(f"base_col_screw{i}a", _m4(xy, b0, clen, True), fab="purchased",
            bom_key=m4_bhcs(clen), color=STEEL)
        add(f"base_col_screw{i}b", _m4(xy, h["bt_top"], clen, False), fab="purchased",
            bom_key=m4_bhcs(clen), color=STEEL)

    # the yaw motor
    shaft_len = round(h["shaft_top"] - h["bt_bot"], 1)
    mbody = union([box(tuple(m), (motor.frame, motor.frame, motor.length),
                       h["bt_bot"] - motor.length),
                   disc(tuple(m), motor.boss_d / 2 - 0.2, h["bt_bot"], h["bt_bot"] + motor.boss_h),
                   disc(tuple(m), motor.shaft_d / 2 - 0.05, h["bt_bot"], h["bt_bot"] + shaft_len)])
    add("yaw_motor", mbody, fab="purchased", bom_key=motor.bom_key, color="#2b2b2b")
    mlen = next(x for x in bh.lengths if tb + motor.hole_depth * 0.8 - 1e-6 <= x)
    for i, xy in enumerate(mscrews):
        add(f"yaw_motor_screw{i}", screw_solid(xy, bh, h["bt_top"], mlen, up=False),
            fab="purchased", bom_key=bh.key(mlen), color=STEEL)

    # the bearings, the shaft and its collars
    cw = float(get("collar_8mm").dims["w"])
    cod = float(get("collar_8mm").dims["od"])
    spring = float(get(support.spring).dims["work_h"])
    shaft_l = h["shaft_end"] - h["lb_bot"]
    if shaft_l > float(get(SHAFT_KEY).dims["length"]) + 1e-9:
        raise ConstructionError(f"turret: the shaft would be {shaft_l:.0f} mm, over the stock "
                                "100 mm")
    add("yaw_shaft", disc(o, 4.0, h["lb_bot"], h["shaft_end"]), turns=True, fab="purchased",
        bom_key=SHAFT_KEY, color="#9aa0a6")
    wt = float(get("m4_washer").dims["t"])          # the stack's clamp in the shaft's end
    add("yaw_clamp_washer", ring(o, float(get("m4_washer").dims["od"]), 4.3, h["table"],
                                 h["table"] + wt), turns=True, fab="purchased",
        bom_key="m4_washer", color=STEEL)
    add("yaw_clamp_screw", _m4(o, h["table"] + wt, 8.0, False), turns=True, fab="purchased",
        bom_key=m4_bhcs(8), color=STEEL)
    for name, z0 in (("yaw_bearing_lower", h["lb_bot"]), ("yaw_bearing_upper", h["ub_bot"])):
        add(name, bearing_solid(support.bearing, o, z0), fab="purchased",
            bom_key=support.bearing, color="#d0d0d0")
        extras.append(BomLine("retaining_compound_641", GLUE, f"turret: {name} into its "
                              "plate"))
    for name, z0 in (("yaw_collar_mid", h["mid_bot"]), ("yaw_collar_under", h["ul_bot"]),
                     ("yaw_collar_over", h["lo_bot"])):
        add(name, ring(o, cod, 8.0, z0, z0 + cw), turns=True, fab="purchased",
            bom_key="collar_8mm", color=STEEL)
    add("yaw_spring", ring(o, 12.0, 8.4, h["lb_top"], h["lb_top"] + spring), turns=True,
        fab="purchased", bom_key=support.spring, color=STEEL)
    for name, z0 in (("yaw_shim_under", h["ub_bot"] - SHIM), ("yaw_shim_over", h["ub_top"])):
        add(name, ring(o, 12.0, 8.2, z0, z0 + SHIM), turns=True, fab="purchased",
            bom_key="shim_8x12x0p5", color=STEEL)

    # the drive, the spacer and the turntable
    big, small = tr.big, tr.small
    w = tr.width()
    z0 = h["drive_bot"]
    pcd = float(big["bolt_pcd"]) / 2
    ties = [(pcd * math.cos(math.radians(45 + 90 * k)), pcd * math.sin(math.radians(45 + 90 * k)))
            for k in range(4)]
    from spiderpig.shapes import drill

    ft = float(big["flange_t"])
    large = union([ring(o, float(big["flange_d"]), float(big["bore"]), z0, z0 + ft),
                   ring(o, float(big["od"]), float(big["bore"]), z0 + ft, z0 + w - ft),
                   ring(o, float(big["flange_d"]), float(big["bore"]), z0 + w - ft, z0 + w)])
    large = drill(large, [(xy, SCREW_HOLE / 2) for xy in ties], z0, z0 + w)
    add("yaw_pulley_large", large, turns=True, fab="purchased", bom_key=tr.large_key,
        color="#c9cdd2")
    sb, sft = float(small["bore"]), float(small["flange_t"])
    add("yaw_pulley_small", union([
        ring(tuple(m), float(small["hub_d"]), sb, z0 - float(small["hub_l"]), z0),
        ring(tuple(m), float(small["flange_d"]), sb, z0, z0 + sft),
        ring(tuple(m), float(small["od"]), sb, z0 + sft, z0 + w - sft),
        ring(tuple(m), float(small["flange_d"]), sb, z0 + w - sft, z0 + w)]),
        fab="purchased", bom_key=tr.small_key, color="#c9cdd2")
    zm = z0 + w / 2
    add("yaw_belt", _belt_loop(o, tuple(m), float(big["od"]) / 2, float(small["od"]) / 2,
                               zm - 3.0, zm + 3.0), fab="purchased",
        bom_key=gt2_loop(int(tp.loop or 0)), color="#202020")
    sp_r = tp.c - float(small["flange_d"]) / 2 - 1.5     # under the turntable to the small
    #                                                       pulley (round 7's review)
    if sp_r > tp.c - float(small["flange_d"]) / 2 - 1.0:
        raise ConstructionError("turret: the turntable's spacer meets the small pulley")
    tie_cuts = [Cut(xy, SCREW_HOLE) for xy in ties]
    n_sp = int(h["n_spacer"])
    foot_cuts = [Cut(xy, SCREW_HOLE) for plate, x in tp.brackets
                 if math.hypot(*(xy := tp.local_ab(x, tp.faces[plate] + (-1 if plate == "outer"
                                                                         else 1) * tp.reach)))
                 < sp_r + 3.0]
    for i in range(n_sp):
        zz = h["drive_top"] + i * ts
        add(f"turntable_spacer{i}", cut_holes(disc(o, sp_r, zz, zz + ts),
                                              [Cut(o, SHAFT_HOLE), *tie_cuts, *foot_cuts], zz,
                                              zz + ts),
            turns=True, fab="laser", color=PLATE_COLOR, sheet=ctx.sheet("frame"))
    flag_xy = (tp.sensor_r, 0.0)
    table_cuts: list[Cut | Rect] = [Cut(o, SHAFT_HOLE), *tie_cuts, Cut(flag_xy, 2.4)]
    for plate, x in tp.brackets:
        zc = tp.faces[plate] + (-1 if plate == "outer" else 1) * tp.reach
        table_cuts.append(Cut(tp.local_ab(x, zc), SCREW_HOLE))
    add("turntable", cut_holes(disc(o, tp.r_table, h["tt_bot"], h["table"]), table_cuts,
                               h["tt_bot"], h["table"]), turns=True, fab="laser",
        color=PLATE_COLOR, sheet=BASE_SHEET)
    tlen = length_for(h["table"] - z0 + NUT_H)
    for i, xy in enumerate(ties):
        add(f"hub_screw{i}", screw_solid(xy, sk, h["table"], tlen, up=False), turns=True,
            fab="purchased", bom_key=sk.key(tlen), color=STEEL)
    extras.append(BomLine("m3_nut", len(ties), "turret: the turntable's screws, under the "
                          "pulley"))

    # the endstop on posts, the M2 flag hanging from the turntable
    if sensor is not None:
        for i, xy in enumerate(e_holes):
            add(f"yaw_endstop_post{i}", ring(xy, 6.0, 2.5, h["bt_top"], b_bot),
                fab="purchased", bom_key=m3_round_standoff(post), color="#3a3a3a")
        ang = phi_s + math.pi / 2                   # the board's long side along v
        jaws = [box(tuple(u * (tp.sensor_r + s * (slot_w / 2 + 2.0))), (4.0,
                    float(board["body"][0]), body_h), b_bot + bt, phi_s) for s in (-1, 1)]
        sensor_body = union([box(tuple(e_c), (bw, bd, bt), b_bot, ang), *jaws])
        sensor_body = cut_holes(sensor_body, [Cut(xy, SCREW_HOLE) for xy in e_holes], b_bot,
                                b_bot + bt)
        add("yaw_endstop", sensor_body, fab="purchased", bom_key=sensor.bom_key,
            color="#1f5f9f")
        slen = length_for(bt + post + tb + NUT_H)     # through the board, post and plate
        for i, xy in enumerate(e_holes):
            add(f"yaw_endstop_screw{i}a", screw_solid(xy, sk, b_bot + bt, slen, up=False),
                fab="purchased", bom_key=sk.key(slen), color=STEEL)
        extras.append(BomLine("m3_nut", len(e_holes), "turret: the endstop's screws, under "
                              "the top plate"))
        add("yaw_flag", screw_solid(flag_xy, s2, h["table"], flen, up=False), turns=True,
            fab="purchased", bom_key=s2.key(flen), color=STEEL)
        add("yaw_flag_nut", ring(flag_xy, float(nut2["af"]) * 1.1, 2.0,
                                 h["tt_bot"] - float(nut2["h"]), h["tt_bot"]), turns=True,
            fab="purchased", bom_key="m2_nut", color=STEEL)

    # the stock angle brackets on the frame plates' feet (robot coordinates)
    bk = STOCK_BRACKET
    for i, (plate, x) in enumerate(tp.brackets):
        face = tp.faces[plate]
        sgn = -1.0 if plate == "outer" else 1.0
        zv = tuple(sorted((face, face + sgn * bk["t"])))
        zh = tuple(sorted((face, face + sgn * bk["d"])))
        part = union([box((x, tp.y_t + bk["h"] / 2), (bk["w"], bk["h"], zv[1] - zv[0]), zv[0]),
                      box((x, tp.y_t + bk["t"] / 2), (bk["w"], bk["t"], zh[1] - zh[0]), zh[0])])
        zc = face + sgn * tp.reach
        part = cut_holes(part, [Cut((x, tp.y_t + bk["hole"]), SCREW_HOLE)], zv[0], zv[1])
        from spiderpig.shapes import difference

        part = difference(part, _along_y(disc((x, zc), SCREW_HOLE / 2, tp.y_t - 1,
                                              tp.y_t + bk["t"] + 1)))
        bodies.append(_hw(f"foot_bracket{i}", part, fab="purchased",
                          bom_key="angle_bracket_20", color="#9a9a9a"))
        turning.append(f"foot_bracket{i}")
        inner_face = face + (ctx.sheet_t("frame") if plate == "outer"
                             else -ctx.sheet_t("frame"))
        L1 = length_for(ctx.sheet_t("frame") + bk["t"] + NUT_H)
        s1 = screw_solid((x, tp.y_t + bk["hole"]), sk, inner_face, L1, up=plate == "inner")
        bodies.append(_hw(f"foot_screw{i}a", s1, fab="purchased", bom_key=sk.key(L1),
                          color=STEEL))
        under = n_sp * ts if math.hypot(*tp.local_ab(x, zc)) < sp_r + 3.0 else 0.0
        L2 = length_for(tb + under + bk["t"] + NUT_H)      # (through the spacer under it)
        s2b = _along_y(screw_solid((x, zc), sk, tp.y_t - tb - under, L2, up=True))
        bodies.append(_hw(f"foot_screw{i}b", s2b, fab="purchased", bom_key=sk.key(L2),
                          color=STEEL))
        turning += [f"foot_screw{i}a", f"foot_screw{i}b"]
    extras.append(BomLine("m3_nut", 2 * len(tp.brackets), "turret: the brackets' screws"))
    extras.append(BomLine("spiral_wrap_6mm", 1.0, "turret: the cables' loop round the base"))

    assembly = [
        "Screw the four hex columns between the bottom plate and the top plate, M4 button "
        "heads through each plate; tighten them before the bearings go in.",
        "Retain an F608 in each plate with Loctite 641 (off the shields), flange on the "
        "plate's top face, the shaft slid through both (the mid collar, the wave washer, the "
        "under collar and its shim on it in order) while it cures: the shaft lines the two "
        "seats up. The lower bearing must slide on the shaft (pick one that does).",
        "Clamp the under collar on its shim up against the upper inner ring, then the over "
        "collar on its shim down on it: the shaft held both ways. Push the mid collar down "
        f"until the wave washer stands at its working {spring:g} mm (a feeler gauge) and "
        "clamp it: the pair preloaded.",
        "Drill the 80-tooth pulley's four 3.4 mm holes on its 30 mm circle (a drill press, "
        "its own bore on a pin), slide it on the shaft onto the over collar, the spacer and "
        "the turntable over it; screw the four M3s through, nuts under the pulley; screw the "
        "M4 and its washer into the shaft's tapped end, clamping the stack.",
        "Hang the motor under the top plate, the small pulley on its shaft's flat in line "
        "with the large one, the belt on; slide the motor along its slots to tension it.",
        f"Screw the endstop's posts to the top plate at {lo_deg - 3:.0f} deg; hang the M2 "
        "flag from the turntable with its nut under it: at the yaw's lower end it passes "
        "through the slot (centre the board on it in its slots). Homing turns down until the "
        "beam breaks, at most the range and 10 deg (start beam-broken: turn up first).",
        f"Screw the base to the bench through the four {FOOT:g} mm spacers (M4 or #8 wood "
        "screws, not drawn): on rubber it rocks under the arm.",
        "Stand the side on the turntable, its feet held by the angle brackets.",
        "Loop the cables loosely round the base in the spiral wrap: the yaw turns them."]
    fastened = [(b.name, "yaw_motor") for b in bodies if b.name.startswith("yaw_motor_screw")]
    fastened.append(("yaw_clamp_screw", "yaw_shaft"))     # in its tapped end
    names = {b.name for b in bodies}
    for b in bodies:
        if b.name.startswith("base_col_screw"):
            fastened.append((b.name, f"base_col{b.name[len('base_col_screw'):-1]}_0"))
        if b.name.startswith("yaw_endstop_screw"):
            post_name = f"yaw_endstop_post{b.name[len('yaw_endstop_screw'):-1]}"
            if post_name in names:
                fastened.append((b.name, post_name))
    info = {"preset": tp.preset.key, "support": tp.preset.support,
            "transmission": tp.preset.transmission, "motor": motor.key,
            "sensor": tp.preset.sensor, "ratio": round(tr.ratio(None), 4),
            "range_deg": [round(x, 2) for x in tp.range_deg],
            "axis": [tp.x_a, round(tp.y_ground, 3), tp.z_a],
            "turntable_y": round(tp.y_t, 3), "height_mm": round(h["table"], 3),
            "base_r": round(tp.r_base, 2), "table_r": round(tp.r_table, 2),
            "keep_r": round(tp.keep_r, 2), "spacing_mm": round(tp.spacing, 2),
            "centre_mm": round(tp.c, 3), "belt_mm": tp.loop,
            "shaft_mm": round(shaft_l, 1), "motor_shaft_mm": shaft_len,
            "columns": list(tp.columns),
            "reach_mm": [round(lo_r, 1), round(hi_r, 1)] if math.isfinite(hi_r) else None,
            "notes": list(tp.notes), "fastened": fastened,
            "parts": sorted(b.name for b in bodies), "assembly": assembly,
            "mill": "", "milled": {}}
    return bodies, turning, extras, info
