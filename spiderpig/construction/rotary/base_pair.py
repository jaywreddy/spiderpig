"""The turret's base: the bearings in the base, the shaft turning (2026-10-12, round 7 of
the arm's review, the turret's variant C; the support ``base_pair``, the preset
``mt4_compact``'s).

Stock parts only but for three cut plates (the user's: "only stock parts except for the
plates"). Bottom up, in the turret's frame (:class:`.turret.TurretPlan`):

* **the bottom plate**, 3 mm cast acrylic (:data:`BOTTOM_SHEET`), screwed to the bench
  through three :data:`FOOT` mm spacers (the lower bearing and the columns' screw heads
  stand under it; on rubber feet a base this light rocks and walks under the arm's
  moment: round 7's review). The slides and the reference sheet lie on the bench in front
  (:func:`table_reach`):
  a work plate in one piece with it, as round 7's review asked, would be some 400 mm long,
  past the 300 mm blank the sheets are nested on, and the camera's homing re-references
  the sheet at every power-up anyway (:mod:`control.vision`);
* **seven columns** of 7 mm hex M4 standoffs (:data:`~spiderpig.hardware.robot_catalog.
  M4_HEX_STANDOFF_LENGTHS`, the shortest past the yaw motor), two in front, two behind the
  motor, three round the bearings (:data:`CENTRE_R`: the acrylic plate held flat where the
  preload bears), an M4 button head through each plate into each end (a washer under it on
  the acrylic);
* **the top plate**, 1/8 in 5052 (:data:`TOP_SHEET`: the columns stand off it as
  cantilevers), the yaw motor hanging under it on slots along the belt (its tension), its
  shaft up through it;
* **the bearings**: an F608ZZ bonded into each plate, its flange on the plate's top face,
  the bearings' centres the column's length and a plate apart (their play tilts the axis
  little over that spacing); the 8 mm h6 **shaft** turns in them, slid through both while
  the epoxy cures (the jig that lines them
  up; the columns tightened first, their holes' play past the compound's gap). The upper
  inner ring is clamped between two collars (on 0.5 mm shims clear of the seals): it holds
  the shaft both ways. The lower one floats on the shaft, a wave washer under a collar
  pressing it down: the pair preloaded, the washer's force against the upper's;
* **the drive** over the top plate: the large plate pulley on the shaft over the upper
  collar (its own bore centres it, four 3.4 mm holes drilled on its 30 mm circle), the
  16-tooth on the motor's uncut shaft, a stock loop;
* **the turntable**, 1/8 in 5052 (:data:`TABLE_SHEET`: it bends under the arm's moment),
  lifted past the motor's shaft and the small pulley's flange by a stack of M3 washers at
  each of its four ties (:data:`LIFT_WASHER`); four M3 screws through the turntable, the
  washers and the pulley, nuts under the pulley; an M4 screw and washer in the shaft's
  tapped end clamp the stack down on the over collar (a collar pushed down by hand clamps
  too lightly to hold the arm's moment);
* **the hard stop** (the sensor ``hard_stop``): an M3 screw hanging from the turntable
  stalls the yaw's driver on a post on the top plate just past the yaw's lower end
  (:func:`stop_radius`): homing turns down until it stalls (the camera then fixes the
  zero: :mod:`control.vision`);
* **the feet**: the frame plates' feet held to the turntable by stock steel angle brackets
  (:func:`stock_bracket`).

The base stays inside a keep-out round the axis (:func:`keep_radius`: the turntable, the
hard stop, the base's round front; behind, its tail round the motor): only the side's
links over it must clear the turntable (:func:`.turret._lows`), the rest pass lower, so the
tool reaches the table.
"""

from __future__ import annotations

import math

import numpy as np

from spiderpig.construction.base import ConstructionError, Context, stock_length
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.mechanism import Body
from spiderpig.shapes import Cut, Rect, box, cut_holes, disc, hull, ring, union

BOTTOM_SHEET = "acrylic_3mm"    # the base's bottom plate: loaded in its plane, the columns
#                                 pinned in it (round 2 of the cost loop: half 5052's cut)
TOP_SHEET = "al5052_3p2mm"      # the top plate: the columns stand off it as cantilevers (an
#                                 acrylic one let the base rack 6-25x more, round 2's
#                                 review), the motor's heat and the belt's pull on it
TABLE_SHEET = "al5052_3p2mm"    # the turntable: the arm's moment bends it; 5052 as stiff as
#                                 6061 (its strength unused) at a cheaper rate
BRACKET_KEY = "angle_bracket_16"   # the 16 mm ones in place of the 20 mm: the turntable,
#                                    round the brackets' far corners, the smaller (the
#                                    user's "an elegant machine", 2026-10-14)


def stock_bracket() -> dict[str, float]:
    """The stock steel angle bracket (:data:`BRACKET_KEY`, its catalog item's dims): its
    width, its legs (``h`` up the frame plate, ``d`` out along the turntable), its
    thickness and each leg's hole from the corner."""
    from spiderpig.hardware.catalog import get

    d = get(BRACKET_KEY).dims
    leg = float(d["leg"])
    return {"w": float(d["w"]), "h": leg, "d": leg, "t": float(d["t"]),
            "hole": float(d["hole_at"])}
SHIM = 0.5                      # a shim under a collar on an inner ring (clear of the seal)
FOOT = 6.0                      # the bench spacers under the bottom plate (m4_spacer_6)
SHAFT_KEY = "shaft_8mm_h6_tapped"
LIFT_WASHER = "shim_din988_3x6_t0p5"   # the stacks under the turntable at its four ties
#                                 (round 2 of the cost loop: in place of a cut spacer disc;
#                                 shims from the kit already bought, not DIN 125 washers whose
#                                 0.45-0.55 mm would tilt it: round 2's review)
CENTRE_SHIM = "shim_8x12x0p5"   # and a stack on the shaft under its centre, where the M4
#                                 clamp pulls it down
BASE_FOOT_L = 19.0              # the side's feet's half length on the turntable (the
#                                 classic's 40): the shortest that holds the brackets'
#                                 symmetric pair (a smaller turntable, the user's "make the
#                                 plate smaller... an elegant machine", 2026-10-14)
YAW_MIN_MESH = 5.0              # teeth in mesh on the yaw's 16-tooth pulley: 5 (Gates' 6 is
#                                 for full rated load; the turntable's is light): the 200 mm
#                                 loop, the motor 17 mm nearer the axis
BASE_RIM = 4.0                  # the base's lobe past the yaw motor's far corners
LOBE_R = 8.0                    # the lobe's back corners' radius
STOP_POST_R = 3.0               # the hard stop's post (a 6 mm round standoff), its screw's
#                                 head under the top plate inside it
CENTRE_R = 18.0                 # three more columns round the bearings: they close the
#                                 preload's loop by the axis, the acrylic plates' bending
#                                 out of it (round 2's review)
HEX_R = 7.0 / math.sqrt(3)      # a 7 mm hex standoff's corner radius
COL_GAP = 4.0                   # a column clear of the motor's body
TAIL_EDGE = 4.0                 # the plates past a back column's corner
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

    tb, tp_ = _sheet_t(BOTTOM_SHEET), _sheet_t(TOP_SHEET)
    tt = _sheet_t(TABLE_SHEET)
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
                           "bt_bot": b0 + tb + L, "bt_top": b0 + tb + L + tp_}
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
    wt = float(get(LIFT_WASHER).dims["t"])      # stacks of stock washers at the ties lift it
    n_w = max(1, math.ceil(lift / wt - 1e-9))
    h["n_washer"] = float(n_w)
    h["tt_bot"] = h["drive_top"] + n_w * wt
    h["table"] = h["tt_bot"] + tt
    h["shaft_end"] = h["table"] - 0.5           # under the clamp screw's washer
    if h["lb_bot"] < 1.0:
        raise ConstructionError("turret: the lower bearing reaches the bench")
    spacing = (h["ub_bot"] + h["ub_top"]) / 2 - (h["lb_bot"] + h["lb_top"]) / 2
    washer = float(get("m4_washer").dims["t"])
    protrude = max(2.2 + washer, screw("shcs", "3").head_h)    # the clamp's M4 head, the ties'
    return h, (L,), spacing, protrude


def stop_radius(tr, c: float, range_deg: tuple[float, float]) -> float:
    """A hard stop's radius (the sensor ``hard_stop``): past the large pulley's flange, and
    where both the post (fixed past the range's lower end) and the stop screw's sweep
    (under the turntable, over the range) stay clear of the belt's runs to the motor behind
    (straight from the large pulley's pitch circle to the small one's)."""
    big, small = float(tr.big["pitch_d"]) / 2, float(tr.small["pitch_d"]) / 2
    post_r, screw_r, gap = 3.0, 2.75, 2.0            # a 6 mm post; the M3 nut's corner
    reach = max(abs(v) for v in range_deg)
    r = float(tr.big["flange_d"]) / 2 + screw_r + gap
    while r < c - small:
        # the belt's run at radius r: from (0, big) to (-c, small), the point that far out
        ts = np.linspace(0.0, 1.0, 401)
        px, py = -c * ts, big + (small - big) * ts
        k = int(np.argmin(np.abs(np.hypot(px, py) - r)))
        belt = math.degrees(math.atan2(py[k], px[k]))        # from the front, 90..180
        post = reach + 3.0 + math.degrees((post_r + gap) / r)
        sweep = reach + math.degrees((screw_r + gap) / r)
        if belt - post > 0 and belt - sweep > 0:
            return r
        r += 0.5
    raise ConstructionError("turret: no hard stop's radius clears the belt")


def keep_radius(tr, c: float, faces: dict[str, float], range_deg: tuple[float, float],
                foot_l: float = BASE_FOOT_L) -> tuple[float, float]:
    """``(front, r_stop)``: the base's round front's radius (the turntable under the feet's
    brackets and the hard stop), which the side's links over it must clear, and the hard
    stop's radius (:func:`stop_radius`)."""
    from .turret import FOOT_R

    bk = stock_bracket()
    half = (faces["inner"] - faces["outer"]) / 2
    feet = max(math.hypot(foot_l - 1.0, half + bk["d"]),     # a bracket's far corner, on
               math.hypot(foot_l + FOOT_R, half)) + 3.0       # the foot; the foot's end
    r_stop = stop_radius(tr, c, range_deg)
    return max(feet, r_stop + 4.0) + 1.0, r_stop


def lobe(c: float, motor) -> list[tuple[tuple[float, float], float]]:
    """The base's lobe behind the axis round the yaw motor (and its slots' travel): its two
    back corners' circles, the plates :data:`BASE_RIM` past the motor's far corners."""
    x = -(c + motor.frame / 2 + 2.0 + BASE_RIM - LOBE_R)
    y = motor.frame / 2 + BASE_RIM - LOBE_R
    return [((x, y), LOBE_R), ((x, -y), LOBE_R)]


def base_radius(front: float, c: float, motor) -> float:
    """How far the base reaches behind the axis: its round core (``front``, the turntable's
    keep-out, the turntable as wide over it) or the lobe round the yaw motor out the back
    (the user's, 2026-10-14: a tight circle for what turns, the motor out the back)."""
    return max(front, c + motor.frame / 2 + 2.0 + BASE_RIM)


OUTER_COLUMNS = (45.0, -45.0, 135.0, -135.0)     # deg: square to the arm, clear of the motor


def columns_xy(tp, motor) -> list[tuple[float, float]]:
    """The seven columns (the turret's frame): four on a circle near the round core's rim
    (:data:`OUTER_COLUMNS`), three round the bearings at :data:`CENTRE_R` (the acrylic plate
    held flat by the axis, where the wave washer's preload bears); each turned in 5 deg
    steps past the yaw motor and the hard stop's post, where it would meet them."""
    r = tp.h["front"] - HEX_R - TAIL_EDGE
    post = (tp.sensor_r * math.cos(math.radians(tp.range_deg[0] - 3.0)),
            tp.sensor_r * math.sin(math.radians(tp.range_deg[0] - 3.0)))

    def clear(xy) -> bool:          # of the motor's body and the hard stop's post
        dx = max(abs(xy[0] + tp.c) - motor.frame / 2, 0.0)
        dy = max(abs(xy[1]) - motor.frame / 2, 0.0)
        if math.hypot(dx, dy) < HEX_R + 1.0:
            return False
        return math.dist(xy, post) >= HEX_R + STOP_POST_R + 1.0

    def at(radius: float, a: float) -> tuple[float, float]:
        for d in (0.0, *(s * k for k in range(5, 35, 5) for s in (1.0, -1.0))):
            xy = (radius * math.cos(math.radians(a + d)), radius * math.sin(math.radians(a + d)))
            if clear(xy):           # turned past them, if need be (the motor's check: parts)
                return xy
        return radius * math.cos(math.radians(a)), radius * math.sin(math.radians(a))

    return ([at(r, a) for a in OUTER_COLUMNS] + [at(CENTRE_R, a) for a in (0.0, 120.0, 240.0)])


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
    from spiderpig import materials

    n = len(columns_xy(tp, tp.preset.parts[2]))
    fixed = 2 if materials.sheet(BOTTOM_SHEET).metal else 1     # ends held: 12 or 3 E I / L^3
    k = n * (12 if fixed == 2 else 3) * E_ALU * i_hex / L ** 3   # N/mm
    per_nm = 1000.0 / tp.spacing / k / tp.spacing       # rad per N·m
    ends = ("fixed-fixed" if fixed == 2 else "cantilevers off the aluminium top plate, pinned "
            "in the acrylic bottom one")
    return per_nm, (f"the base's {n} {L:g} mm hex columns rack {per_nm * 1e6:.1f} urad per "
                    f"N·m of the arm's tipping moment ({ends}, {k:.0f} N/mm: a lower bound: "
                    "screwed ends rack more); it changes with the pose as the moment does "
                    "(accuracy, not repeatability)")


def _hw(name, part, *, fab, bom_key=None, color=None, sheet=None) -> Body:
    return Body(name=name, part=part, color=color, fab=fab, bom_key=bom_key, sheet=sheet)


def _m4(xy, z: float, length: float, up: bool):
    """An M4 button head bearing at ``z`` (ISO 7380: 7.6 x 2.2)."""
    s = 1.0 if up else -1.0
    return union([disc(xy, 3.8, *sorted((z - s * 2.2, z))),
                  disc(xy, 2.0, *sorted((z, z + s * length)))])


def turret_parts(tp, ctx: Context) -> tuple[list[Body], list[str], list[BomLine], dict]:
    """The turret's bodies (robot coordinates, :class:`.turret.TurretPlan` ``tp``), the
    names of those that turn with the turntable, its purchases not drawn, and what the
    mechanism's meta says of it (module docstring)."""
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

    support, tr, motor, _sensor = tp.preset.parts
    h = tp.h
    tb, tp_ = _sheet_t(BOTTOM_SHEET), _sheet_t(TOP_SHEET)
    tt = _sheet_t(TABLE_SHEET)
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
        return stock_length(s, need, "turret")

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
    front = h["front"]

    # the hard stop: a post just past the range's lower end, its top under the brackets'
    # screws' heads (which pass over it); an M3 hanging from the turntable to 2 mm under it
    lo_deg = tp.range_deg[0]
    phi_s = math.radians(lo_deg - 3.0)
    u = np.array([math.cos(phi_s), math.sin(phi_s)])
    avail = h["tt_bot"] - sk.head_h - 1.0 - h["bt_top"]
    post = max((x for x in M3_ROUND_STANDOFF_LENGTHS if x <= avail), default=None)
    if post is None:
        raise ConstructionError("turret: no room for the hard stop's post under the turntable")
    b_bot = h["bt_top"] + post
    flen = length_for(h["table"] - b_bot + 2.0)    # its tip 2 mm under the post's top
    if h["table"] - flen < h["bt_top"] + sk.head_h + 1.0:
        raise ConstructionError("turret: no stock M3 screw reaches the hard stop's post")

    # the base: a round core under the turntable, as wide, a lobe out the back round the motor
    def outline(z0, z1):
        return hull([(o, front), *lobe(tp.c, motor)], z0, z1)

    lo_r, hi_r = h.get("reach_lo", math.nan), h.get("reach_hi", math.nan)
    col_cuts = [Cut(c, 4.4) for c in cols]
    b0, b1 = h["bp_bot"], h["bp_top"]
    r_b = front - 6.0                               # three: a kinematic mount on the bench
    bench = [(r_b * math.cos(math.radians(a)), r_b * math.sin(math.radians(a)))
             for a in (0.0, 90.0, -90.0)]
    add("base_bottom", cut_holes(outline(b0, b1), [Cut(o, 22.0 + 0.2), *col_cuts,
                                                   *(Cut(xy, 4.4) for xy in bench)], b0, b1),
        fab="laser", color=PLATE_COLOR, sheet=BOTTOM_SHEET)
    for i, xy in enumerate(bench):                  # screwed to the bench on spacers
        add(f"base_spacer{i}", ring(xy, 8.0, 4.2, 0.0, b0), fab="purchased",
            bom_key="m4_spacer_6", color="#3a3a3a")
    slot = 4.0                                      # the belt's tension: the motor's travel
    top_cuts: list[Cut | Rect] = [Cut(o, 22.0 + 0.2), *col_cuts]
    top_cuts.append(Cut(tuple(u * tp.sensor_r), SCREW_HOLE))     # the hard stop's post
    top_cuts += [Rect(xy, (SCREW_HOLE + slot, SCREW_HOLE), 0.0) for xy in mscrews]
    boss = motor.boss_d + 0.4                       # its round boss, slid along the slots:
    top_cuts += [Cut((m[0] - slot / 2, 0.0), boss), Cut((m[0] + slot / 2, 0.0), boss),
                 Rect(tuple(m), (slot, boss), 0.0)]  # a stadium (a rectangle left 2.6 mm
    #                                                 of web to the screws' slots, 2026-10-14)
    add("base_top", cut_holes(outline(h["bt_bot"], h["bt_top"]), top_cuts, h["bt_bot"],
                              h["bt_top"]), fab="laser", color=PLATE_COLOR, sheet=TOP_SHEET)
    L = sum(tp.columns)
    for i, xy in enumerate(cols):
        hexa = box(xy, (7.0, 7.0 / math.sqrt(3) * 2, L), b1)
        add(f"base_col{i}_0", cut_holes(hexa, [Cut(xy, 3.3)], b1, b1 + L), fab="purchased",
            bom_key=f"m4_hex_standoff_{L:g}", color="#3a3a3a")
    wd4 = get("m4_washer").dims             # under each head on the acrylic (its creep)
    w4 = float(wd4["t"])

    def m4_len(t: float) -> float:
        return next((x for x in M4_BHCS_LENGTHS if x >= t + 4.0 - 1e-6), M4_BHCS_LENGTHS[-1])

    clen_a, clen_b = m4_len(tb + w4), m4_len(tp_)
    for i, xy in enumerate(cols):
        add(f"base_col_washer{i}", ring(xy, float(wd4["od"]), float(wd4["id"]), b0 - w4, b0),
            fab="purchased", bom_key="m4_washer", color=STEEL)
        add(f"base_col_screw{i}a", _m4(xy, b0 - w4, clen_a, True), fab="purchased",
            bom_key=m4_bhcs(clen_a), color=STEEL)
        add(f"base_col_screw{i}b", _m4(xy, h["bt_top"], clen_b, False), fab="purchased",
            bom_key=m4_bhcs(clen_b), color=STEEL)

    # the yaw motor
    shaft_len = round(h["shaft_top"] - h["bt_bot"], 1)
    mbody = union([box(tuple(m), (motor.frame, motor.frame, motor.length),
                       h["bt_bot"] - motor.length),
                   disc(tuple(m), motor.boss_d / 2 - 0.2, h["bt_bot"], h["bt_bot"] + motor.boss_h),
                   disc(tuple(m), motor.shaft_d / 2 - 0.05, h["bt_bot"], h["bt_bot"] + shaft_len)])
    add("yaw_motor", mbody, fab="purchased", bom_key=motor.bom_key, color="#2b2b2b")
    mlen = length_for(tp_ + motor.hole_depth * 0.8, bh)
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
        extras.append(BomLine("epoxy_2part", GLUE, f"turret: {name} into its plate "
                              "(abrade both)"))
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
    tie_cuts = [Cut(xy, SCREW_HOLE) for xy in ties]
    n_w = int(h["n_washer"])
    wd = get(LIFT_WASHER).dims
    for i, xy in enumerate(ties):                   # the washer stacks lifting the turntable
        add(f"turntable_lift{i}", ring(xy, float(wd["od"]), float(wd["id"]), h["drive_top"],
                                       h["tt_bot"]), turns=True, fab="purchased",
            color=STEEL)
    extras.append(BomLine(LIFT_WASHER, len(ties) * n_w, f"turret: {n_w} under the turntable "
                          "at each of its four ties"))
    cd_ = get(CENTRE_SHIM).dims
    add("turntable_centre_shims", ring(o, float(cd_["od"]), float(cd_["id"]), h["drive_top"],
                                       h["tt_bot"]), turns=True, fab="purchased", color=STEEL)
    extras.append(BomLine(CENTRE_SHIM, n_w, f"turret: {n_w} on the shaft under the turntable's "
                          "centre (the clamp's seat)"))
    flag_xy = (tp.sensor_r, 0.0)
    table_cuts: list[Cut | Rect] = [Cut(o, SHAFT_HOLE), *tie_cuts, Cut(flag_xy, SCREW_HOLE)]
    for plate, x in tp.brackets:
        zc = tp.faces[plate] + (-1 if plate == "outer" else 1) * tp.reach
        table_cuts.append(Cut(tp.local_ab(x, zc), SCREW_HOLE))
    add("turntable", cut_holes(disc(o, tp.r_table, h["tt_bot"], h["table"]), table_cuts,
                               h["tt_bot"], h["table"]), turns=True, fab="laser",
        color=PLATE_COLOR, sheet=TABLE_SHEET)
    tlen = length_for(h["table"] - z0 + NUT_H)
    for i, xy in enumerate(ties):
        add(f"hub_screw{i}", screw_solid(xy, sk, h["table"], tlen, up=False), turns=True,
            fab="purchased", bom_key=sk.key(tlen), color=STEEL)
    extras.append(BomLine("m3_nut", len(ties), "turret: the turntable's screws, under the "
                          "pulley"))

    # the hard stop: a post on the top plate past the range's lower end, an M3 hanging from
    # the turntable stalls on it (the driver's stall detection homes the yaw coarsely)
    p_xy = tuple(u * tp.sensor_r)
    add("yaw_stop_post", ring(p_xy, 6.0, 2.5, h["bt_top"], b_bot), fab="purchased",
        bom_key=m3_round_standoff(post), color="#3a3a3a")
    plen = length_for(tp_ + 4.0)
    add("yaw_stop_post_screw", screw_solid(p_xy, bh, h["bt_bot"], plen, up=True),
        fab="purchased", bom_key=bh.key(plen), color=STEEL)
    add("yaw_stop", screw_solid(flag_xy, sk, h["table"], flen, up=False), turns=True,
        fab="purchased", bom_key=sk.key(flen), color=STEEL)
    add("yaw_stop_nut", ring(flag_xy, 6.0, 3.0, h["tt_bot"] - NUT_H, h["tt_bot"]),
        turns=True, fab="purchased", bom_key="m3_nut", color=STEEL)

    # the stock angle brackets on the frame plates' feet (robot coordinates)
    bk = stock_bracket()
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
                          bom_key=BRACKET_KEY, color="#2b2b2b"))
        turning.append(f"foot_bracket{i}")
        inner_face = face + (ctx.sheet_t("frame") if plate == "outer"
                             else -ctx.sheet_t("frame"))
        L1 = length_for(ctx.sheet_t("frame") + bk["t"] + NUT_H)
        s1 = screw_solid((x, tp.y_t + bk["hole"]), sk, inner_face, L1, up=plate == "inner")
        bodies.append(_hw(f"foot_screw{i}a", s1, fab="purchased", bom_key=sk.key(L1),
                          color=STEEL))
        L2 = length_for(tt + bk["t"] + NUT_H)
        s2b = _along_y(screw_solid((x, zc), sk, tp.y_t - tt, L2, up=True))
        bodies.append(_hw(f"foot_screw{i}b", s2b, fab="purchased", bom_key=sk.key(L2),
                          color=STEEL))
        turning += [f"foot_screw{i}a", f"foot_screw{i}b"]
    extras.append(BomLine("m3_nut", 2 * len(tp.brackets), "turret: the brackets' screws"))
    extras.append(BomLine("spiral_wrap_6mm", 1.0, "turret: the cables' loop round the base"))

    steps = [   # (the hook's tag, its sentence): :func:`assembly` turns them into ops
        ("columns", "Screw the seven hex columns between the bottom plate and the top plate, "
                    "M4 button heads through each plate (a washer under each on the "
                    "acrylic); tighten them before the bearings go in."),
        ("bearings", "Bond an F608 into each plate with epoxy (both faces abraded, none on "
                     "the shields), flange on the plate's top face, the shaft slid through "
                     "both (the mid collar, the wave washer, the under collar and its shim on "
                     "it in order) while it cures: the shaft lines the two seats up. The "
                     "lower bearing must slide on the shaft (pick one that does)."),
        ("preload", "Clamp the under collar on its shim up against the upper inner ring, "
                    "then the over collar on its shim down on it: the shaft held both ways. "
                    f"Push the mid collar down until the wave washer stands at its working "
                    f"{spring:g} mm (a feeler gauge) and clamp it: the pair preloaded."),
        ("turntable", f"Drill the {big['teeth']}-tooth pulley's four 3.4 mm holes on its "
                      f"{float(big['bolt_pcd']):g} mm circle (a drill press, its own bore on "
                      "a pin), slide it on the shaft onto the over collar, a washer stack on "
                      "each of its holes and the turntable over them; screw the four M3s "
                      "through, nuts under the pulley; screw the M4 and its washer into the "
                      "shaft's tapped end, clamping the stack."),
        ("motor", "Hang the motor under the top plate, the small pulley on its shaft's flat in "
                  "line with the large one, the belt on; slide the motor along its slots to "
                  "tension it."),
        ("stop", f"Screw the hard stop's post to the top plate at {lo_deg - 3:.0f} deg; hang "
                 "its M3 from the turntable with its nut under it: past the yaw's lower end it "
                 "meets the post. Homing turns down at a reduced current until the driver "
                 "stalls on it (the camera then sets the fine zero)."),
        ("bench", f"Screw the base to the bench through the {len(bench)} {FOOT:g} mm spacers "
                  "(M4 or #8 wood screws, not drawn): on rubber it rocks under the arm."),
        ("mount", "Stand the side on the turntable, its feet held by the angle brackets."),
        ("cables", "Loop the cables loosely round the base in the spiral wrap: the yaw turns "
                   "them.")]
    fastened = [(b.name, "yaw_motor") for b in bodies if b.name.startswith("yaw_motor_screw")]
    fastened.append(("yaw_clamp_screw", "yaw_shaft"))     # in its tapped end
    fastened.append(("yaw_stop_post_screw", "yaw_stop_post"))   # into its female end
    fastened += [(b.name, f"base_col{b.name[len('base_col_screw'):-1]}_0") for b in bodies
                 if b.name.startswith("base_col_screw")]
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
            "parts": sorted(b.name for b in bodies), "assembly": steps}
    return bodies, turning, extras, info


ASSEMBLY: tuple[tuple[str, str, str, str], ...] = (
    # (tag, the stage, the step's title, the parts: a pattern over the bodies' names)
    ("columns", "turret", "Base: plates and columns",
     r"base_(bottom|top|col\d+_0|col_washer\d+|col_screw\d+[ab])"),
    ("bearings", "turret", "Base: bearings bonded round the shaft",
     r"yaw_(bearing_\w+|shaft|collar_mid|spring|collar_under|shim_under)"),
    ("preload", "turret", "Shaft clamped and preloaded", r"yaw_(collar_over|shim_over)"),
    ("turntable", "turret", "Pulley and turntable",
     r"yaw_pulley_large|turntable\w*|hub_screw\d+|yaw_clamp_\w+"),
    ("motor", "turret", "Yaw motor and belt", r"yaw_motor\w*|yaw_pulley_small|yaw_belt"),
    ("stop", "turret", "Hard stop", r"yaw_stop\w*"),
    ("bench", "turret", "Base onto the bench", r"base_spacer\d+"),
    ("mount", "mount", "Side onto the turntable", r"foot_bracket\d+|foot_screw\d+[ab]"),
    ("cables", "wiring", "", ""),
)
"""How the turret goes on, step by step: each of :func:`turret_parts`' sentences (by its
tag), the stage it goes on in (:mod:`construction.assembly`) and the parts it puts on."""

EXTRAS = {"turntable": (LIFT_WASHER, CENTRE_SHIM, "m3_nut"), "mount": ("m3_nut",),
          "cables": ("spiral_wrap_6mm",)}
"""The bought lines that are no body's (:func:`turret_parts`' extras), by the step they're
for."""


def assembly(view) -> list:
    """How the turret goes on (:mod:`construction.assembly`): the base built on the bench
    (its plates and columns, the bearings bonded round the shaft, the shaft clamped and
    preloaded, the pulley and the turntable, the yaw motor and its belt, the hard stop, the
    base screwed to the bench), then the side stood on the turntable by its brackets, and
    the cables looped round the base with the wiring. The sentences are the meta's
    (``turret.assembly``: :func:`turret_parts`'), so its numbers are the plan's. ``view``: a
    ``RobotView``."""
    from spiderpig.construction.assembly import Op, whole

    text = dict((view.meta.get("turret") or {}).get("assembly") or ())
    ops = []
    for i, (tag, stage, title, pattern) in enumerate(ASSEMBLY):
        parts = view.named(pattern) if pattern else []
        if pattern and not parts:
            continue
        ops.append(Op(stage, (i,), whole(*parts), title, text.get(tag, ""), tag,
                      extras=EXTRAS.get(tag, ())))
    return ops
