"""An arm's tool on its flange (2026-10-12, phase 4): the role ``tool``.

The tool hangs under the output point (the palletizer's flange ``F`` on its end plate
``b8``, which never turns: :mod:`linkages.palletizer`), held by a printed holder bolted to
the end plate by two M3 screws along the plate's arm toward ``W`` (the nuts on the
holder's far face). The holder stands beside the end plate, on the side toward the stack's
middle, in layers no other link sweeps near the flange (:func:`tool_plan` checks every
link and frame pivot in them over the inputs' box, in the end plate's frame).

* ``suction_cup``: a 10 mm vacuum cup (Festo VAS-10-M5-NBR) screwed into the holder's
  underside (a tapped M5 hole), its vacuum through the holder to a push-in fitting on its
  front; the pump and the release valve sit on the bench (in the BOM, not on the arm).
* ``suction_wrist``: the same cup turned about the vertical by a NEMA 11 stepper, the arm's
  fourth axis (``q4``, :class:`kinematics.SpatialJoint`): the motor stands on the holder's
  shelf, shaft down through it, the cup on a printed coupler on the shaft.

What the tool weighs is the payload the drives' sag counts on top of the work's
(:mod:`spiderpig.precision`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig.construction.base import Build, ConstructionError, Context, Group, Realized, hardware
from spiderpig.construction.plane import Rect, point_rect, seg_rect_many
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.shapes import Cut, box, cut_holes, difference, disc, pill, ring, union

TAB_R = 6.0          # the holder's tab along the end plate's arm: its half-width (mm)
TAB_L = 15.0         # how far the tab runs from the flange toward the arm's other end
HOLES_AT = (5.0, 12.0)   # the two screws along the tab from the flange
BLOCK = 8.0          # the holder's block under the flange (the cup's thread in it)
WALL = 5.0           # the wrist holder's tab, cap, wall and shelf thickness
CAP = 6.5            # the wrist's cap stands this far under the flange (the plate's end over)
COUPLER = 10.0       # the wrist's printed coupler on the motor's shaft
NUT_H = 2.4
SCREW_HOLE = 3.4
HOLDER_COLOR = "#e8a33d"
CUP_COLOR = "#303030"
MARGIN = 1.5         # the tool's clearance to whatever moves past it


@dataclass(frozen=True)
class Tool:
    """A tool for the flange (module docstring). ``wrist``: the stepper turning it
    (:data:`spiderpig.actuators.STEPPERS`; ``""``: a fixed tool)."""

    key: str
    label: str
    cup: str = "suction_cup_m5_10"
    wrist: str = ""

    def cup_dims(self) -> dict:
        from spiderpig.hardware.catalog import get

        return get(self.cup).dims

    def width(self) -> float:
        """The holder's tab across the plane, from the end plate's face (mm)."""
        return WALL if self.wrist else 12.0

    def motor(self):
        from spiderpig import actuators

        return actuators.get(self.wrist)

    def drop(self) -> float:
        """How far its lowest point hangs under the flange (mm)."""
        h = float(self.cup_dims()["h"])
        if not self.wrist:
            return BLOCK + h
        return CAP + WALL + self.motor().length + WALL + 1.0 + COUPLER + h


TOOLS: dict[str, Tool] = {t.key: t for t in (
    Tool("suction_cup", "a 10 mm vacuum cup under the flange, on a printed holder"),
    Tool("suction_wrist", "the vacuum cup turned by a NEMA 11 stepper (the fourth axis)",
         wrist="nema11_11hs20"),
)}
DEFAULT = "suction_cup"


@dataclass(frozen=True)
class ToolPlan:
    """Where the tool goes: the output body and point, the arm's other end ``toward``, the
    side of the end plate its tab bears on (``s``: +1 up the stack), and what it fills:
    ``regions``, each an outline in the end plate's frame relative to the flange (a box)
    over a z span."""

    tool: Tool
    body: str
    point: str
    toward: str
    s: int
    regions: tuple[tuple[Rect, tuple[float, float]], ...]


def _toward(ctx: Context, body: str, point: str) -> str:
    for a, b in ctx.topo.links.get(body, ()):
        if point in (a, b):
            return b if a == point else a
    raise ConstructionError(f"tool: {body} has no arm at its output point {point}")


def wrist_z(tool: Tool, lo: float, hi: float, floor: float, ceil: float) -> float:
    """The wrist's axis across the plane: under the end plate (``lo..hi``), moved toward the
    stack's middle as far as its yoke needs to stay between the frame plates."""
    hz = tool.motor().frame / 2 + 0.5 + WALL
    return min(max((lo + hi) / 2, floor + hz), ceil - hz)


def _regions(tool: Tool, u: np.ndarray, s: int, lo: float, hi: float, floor: float,
             ceil: float) -> list[tuple[Rect, tuple[float, float]]]:
    """What the tool fills beside an end plate in ``lo..hi``, its tab on side ``s``: the tab
    (and, for the plain cup, the block and cup under it) over the tab's z, the screws'
    heads on the plate's other face and their nuts; the wrist's yoke, motor and cup under
    the flange, centred on the plate."""
    tip = TAB_L * u
    head = screw("shcs", "3").head_h
    face, other = (hi, lo) if s > 0 else (lo, hi)
    ends = (other - s * head, face + s * (tool.width() + NUT_H + 0.5))
    tab = (min(ends), max(ends))
    xs = (min(-TAB_R, tip[0] - TAB_R), max(TAB_R, tip[0] + TAB_R))
    top = max(TAB_R, tip[1] + TAB_R)
    if not tool.wrist:
        return [((xs[0], -tool.drop(), xs[1], top), tab)]
    mo = tool.motor()
    half = mo.frame / 2 + WALL
    zb = wrist_z(tool, lo, hi, floor, ceil)
    hz = mo.frame / 2 + 0.5 + WALL
    return [((xs[0], -(CAP + WALL), xs[1], top), tab),
            ((-half, -tool.drop(), half, -CAP), (zb - hz, zb + hz))]


def tool_plan(ctx: Context, plan, tool: Tool, groups=()) -> ToolPlan:
    """The tool's place (module docstring); raises :class:`ConstructionError` when neither
    side of the end plate has room near the flange over the box."""
    lk = ctx.config.lk
    if lk.output is None or lk.output.kind != "point" or not lk.output.body:
        raise ConstructionError(f"tool: {lk.key} has no output point on a body to hold one")
    body = lk.output.body
    point = ctx.topo.point_of.get((body, lk.output.point), lk.output.point)
    toward = _toward(ctx, body, point)
    pts = ctx.topo.geometry.points
    f, w = np.asarray(pts[point], dtype=float), np.asarray(pts[toward], dtype=float)
    d = w - f
    ang = np.arctan2(d[:, 1], d[:, 0])
    u0 = d[0] / np.linalg.norm(d[0])
    k = plan.layers[body]
    lo, hi = plan.z(k)
    floor, ceil = plan.z(0)[1], plan.z(plan.top)[0]
    hubs = drive_hubs(ctx, groups)
    tried = []
    for s in ((1, -1) if k <= plan.top / 2 else (-1, 1)):
        side = "up" if s > 0 else "down"
        regions = _regions(tool, u0, s, lo, hi, floor, ceil)
        if any(z[0] < floor - 1e-9 or z[1] > ceil + 1e-9 for _, z in regions):
            tried.append(f"{side} the stack it reaches a frame plate")
            continue
        hit = next((h for rect, z in regions
                    if (h := _hits(ctx, plan, body, f, ang, rect, z, hubs)) is not None), None)
        if hit is None:
            return ToolPlan(tool, body, point, toward, s, tuple(regions))
        tried.append(f"{side} the stack it meets {hit}")
    raise ConstructionError(f"tool {tool.key}: no room beside {body} at {point}: "
                            + "; ".join(tried))


def _hits(ctx: Context, plan, body: str, f: np.ndarray, ang: np.ndarray, rect: Rect,
          span: tuple[float, float], hubs: dict[str, float]) -> str | None:
    """What meets the tool's outline ``rect`` over the box (in ``body``'s frame, its turn
    from the first sample undone): a link in a layer ``span`` reaches, a frame pivot's
    pillar or drive hub; ``None``: nothing."""
    from spiderpig import construction

    pts = ctx.topo.geometry.points
    lat = ctx.topo.geometry.lateral
    p = ctx.params
    pin_head = construction.axle(ctx.config.pin).dims(ctx, False).head
    r_link = max(p.link_radius, pin_head) + MARGIN
    turn = ang - ang[0]
    c, s_ = np.cos(-turn), np.sin(-turn)

    def local(xy: np.ndarray) -> np.ndarray:
        q = np.broadcast_to(xy, f.shape) - f
        return np.stack([c * q[:, 0] - s_ * q[:, 1], s_ * q[:, 0] + c * q[:, 1]], axis=1)

    for link, segs in ctx.topo.links.items():
        if link == body:
            continue
        z0, z1 = plan.z(plan.layers[link])
        if z1 <= span[0] + 1e-9 or z0 >= span[1] - 1e-9:
            continue
        for a, b in segs:
            la = local(np.asarray(pts[a], dtype=float))
            lb = local(np.asarray(pts[b], dtype=float))
            slack = lat.get(a, 0.0) + lat.get(b, 0.0)
            if (seg_rect_many(la, lb, rect) < r_link + slack).any():
                return f"{link} (layer {plan.layers[link]})"
    heads = construction.axle(ctx.config.pillar).dims(ctx, True).head
    for ax in ctx.topo.axes_of("frame"):
        r = max(heads, hubs.get(ax.name, 0.0), p.frame_radius) + MARGIN
        lp = local(np.asarray(pts[ax.name], dtype=float).reshape(-1, 2)[0])
        if min(point_rect(q, rect) for q in lp) < r:
            return f"the frame pivot {ax.name}"
    return None


def drive_hubs(ctx: Context, groups) -> dict[str, float]:
    """Per frame pivot, the largest drive hub turning round it (its pulley or drum)."""
    from spiderpig.construction.transmissions import DriveGroup

    out: dict[str, float] = {}
    for g in groups:
        if isinstance(g, DriveGroup):
            axis = g.hub(ctx).axis
            out[axis] = max(out.get(axis, 0.0), g.transmission.hub_r(ctx))
    return out


class ToolMount(Group):
    """The tool, its holder and its screws, riding the output body (module docstring)."""

    name = "tool"

    def __init__(self, plan: ToolPlan):
        self.plan = plan

    def claims(self, ctx: Context) -> list:
        return []        # the robot's: checked against the side's links by tool_plan

    def realize(self, build: Build, done: Realized) -> Realized:
        tp, tool = self.plan, self.plan.tool
        out = Realized()
        f = np.asarray(build.xy(tp.point), dtype=float)
        w = np.asarray(build.xy(tp.toward), dtype=float)
        u = (w - f) / np.linalg.norm(w - f)
        k = build.layers[tp.body]
        bz0, bz1 = build.z(k)
        face = bz1 if tp.s > 0 else bz0                   # the end plate's face it bears on
        tab_far = face + tp.s * tool.width()
        tz = tuple(sorted((face, tab_far)))
        cup = tool.cup_dims()
        sk = screw("shcs", "3")
        holes = [f + a * u for a in HOLES_AT]
        rotating: list[str] = []
        zm = ((tz[0] + tz[1]) / 2 if not tool.wrist
              else wrist_z(tool, bz0, bz1, build.z(0)[1], build.z(build.top)[0]))
        drop = BLOCK if not tool.wrist else CAP + WALL
        parts = [pill(tuple(f), tuple(f + TAB_L * u), TAB_R, *tz),
                 box((f[0], f[1] - drop / 2), (2 * TAB_R, drop, tz[1] - tz[0]), tz[0])]
        if tool.wrist:
            mo = tool.motor()
            half = mo.frame / 2 + WALL
            hz = mo.frame / 2 + 0.5 + WALL
            cap0 = f[1] - CAP - WALL                       # the yoke's cap under the flange
            face_y = cap0 - mo.length                      # the motor's face, down
            shelf0 = face_y - WALL
            wall = tuple(sorted((zm + tp.s * (mo.frame / 2 + 0.5), zm + tp.s * hz)))
            parts += [box((f[0], cap0 + WALL / 2), (2 * half, WALL, 2 * hz), zm - hz),
                      box((f[0], (shelf0 + cap0) / 2), (2 * half, cap0 - shelf0,
                                                         wall[1] - wall[0]), wall[0]),
                      box((f[0], face_y - WALL / 2), (2 * half, WALL, 2 * hz), zm - hz)]
            motor = union([box((f[0], face_y + mo.length / 2), (mo.frame, mo.length, mo.frame),
                               zm - mo.frame / 2),
                           _from_xz(disc((f[0], zm), mo.shaft_d / 2 - 0.05,
                                         face_y - WALL - 1.0 - COUPLER + 0.5, face_y))])
            out.bodies.append(hardware("wrist_motor", motor, tp.body, fab="purchased",
                                       bom_key=mo.bom_key, color="#2b2b2b"))
            msk = screw("shcs", "2p5")
            mlen = next(L for L in msk.lengths if WALL + mo.hole_depth * 0.8 - 1e-6 <= L)
            hs = mo.hole_square / 2
            spots = [(f[0] + dx * hs, zm + dz * hs) for dx, dz in ((-1, -1), (1, -1), (1, 1),
                                                                   (-1, 1))]
            for i, xz in enumerate(spots):
                sc = screw_solid(xz, msk, shelf0, mlen, up=True)
                out.bodies.append(hardware(f"wrist_screw{i}", _from_xz(sc), tp.body,
                                           fab="purchased", bom_key=msk.key(mlen),
                                           color="#4a4a4a"))
            cuts = [_from_xz(disc(xz, 1.45, shelf0 - 1, face_y + 1)) for xz in spots]
            cuts.append(_from_xz(disc((f[0], zm), mo.boss_d / 2 + 0.3, shelf0 - 1, face_y + 1)))
            cy1 = shelf0 - 1.0
            coupler = _from_xz(ring((f[0], zm), 12.0, mo.shaft_d + 0.1, cy1 - COUPLER, cy1))
            out.bodies.append(hardware("tool_coupler", coupler, tp.body, fab="printed",
                                       color=HOLDER_COLOR))
            rotating.append("tool_coupler")
            cup_top = cy1 - COUPLER
            out.extras.append(BomLine("m3_set_screw_6", 1, "tool: the coupler on the "
                                                           "wrist's shaft flat"))
        else:
            cuts = []
            cup_top = f[1] - BLOCK
        holder = union(parts)
        holder = cut_holes(holder, [Cut(tuple(xy), SCREW_HOLE) for xy in holes], *tz)
        if cuts:
            holder = difference(holder, *cuts)
        out.bodies.append(hardware("tool_holder", holder, tp.body, fab="printed",
                                   color=HOLDER_COLOR))
        cup_part = _from_xz(disc((f[0], zm), float(cup["d"]) / 2, cup_top - float(cup["h"]),
                                 cup_top))
        out.bodies.append(hardware("tool_cup", cup_part, tp.body, fab="purchased",
                                   bom_key=tool.cup, color=CUP_COLOR))
        if tool.wrist:
            rotating.append("tool_cup")
            out.notes["fastened"] = {f"wrist_screw{i}|wrist_motor": [f"wrist_screw{i}",
                                                                   "wrist_motor"]
                                     for i in range(4)}
        # the holder's screws: heads on the end plate's other face, nuts on the tab's far face
        link_t = bz1 - bz0
        length = next(L for L in sk.lengths
                      if link_t + abs(tab_far - face) + NUT_H - 1e-6 <= L)
        head_z = bz0 if tp.s > 0 else bz1
        for i, xy in enumerate(holes):
            out.bodies.append(hardware(f"tool_screw{i}", screw_solid(
                tuple(xy), sk, head_z, length, up=tp.s > 0), tp.body, fab="purchased",
                bom_key=sk.key(length), color="#4a4a4a"))
            out.cut(tp.body, Cut(tuple(xy), SCREW_HOLE))
        out.extras += [BomLine("m3_nut", len(holes), "tool: the holder's screws"),
                       BomLine("barb_m5_4mm", 1, "tool: the vacuum port on the holder"),
                       BomLine("pu_tube_4mm", 1.0, "tool: holder to the valve"),
                       BomLine("vacuum_pump_12v", 1, "tool: on the bench"),
                       BomLine("solenoid_valve_12v_3way", 1, "tool: releases the cup")]
        out.notes["tool"] = {
            "key": tool.key, "body": tp.body, "point": tp.point, "side": tp.s,
            "drop_mm": round(tool.drop(), 3), "wrist": tool.wrist,
            "axis": [float(f[0]), float(f[1]), float(zm)], "rotating": rotating}
        return out


def _from_xz(part):
    """A part drawn with its axis along z (``disc((x, z), r, y0, y1)``) stood along y, at
    ``(x, y0..y1, z)``: ``(X, Y, Z) -> (X, Z, Y)``, a quarter turn about x then a mirror
    in the xy plane."""
    from build123d import Axis, Plane

    return part.rotate(Axis.X, -90).mirror(Plane.XY)


def keepout(tool: Tool) -> float:
    """The tool's radius in the plane round its axis (for the robot's floor)."""
    return max(TAB_R, float(tool.cup_dims()["d"]) / 2,
               (0.0 if not tool.wrist else math.hypot(1, 1) * 15.0))
