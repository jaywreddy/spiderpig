"""An arm's tool on its flange (2026-10-12, phase 4): the role ``tool``.

The tool hangs under the output point (the palletizer's flange ``F`` on its end plate
``b8``, which never turns: :mod:`linkages.palletizer`), held beside the end plate, on the
side toward the stack's middle, in layers no other link sweeps near the flange
(:func:`tool_plan` checks every link and frame pivot in them over the inputs' box, in the
end plate's frame).

* ``suction_cup``: a 10 mm vacuum cup on two laser-cut aluminium plates (2026-10-13: no
  printed part; :data:`TOOL_SHEET`). The **hanger** stands against the end plate's face,
  bolted to it by two M3 screws along the plate's arm toward ``W`` (the nuts on its far
  face), and runs down to :data:`BLOCK` under the flange. The **shelf** lies level under
  it, held up against the hanger's bottom edge by an M3 screw in a T-slot (an M3 nut in
  its crossbar) behind the flange, where the nut, wider than the plate, passes under the
  end plate, and located by a tab of the hanger in a slot of the shelf under the flange.
  The cup's column stands outboard of the hanger: a push-in fitting's M5 thread down
  through the shelf into an M5 coupling nut under it (clamping the shelf), the cup screwed
  into the nut's lower end; the vacuum runs through the three. The pump and the release
  valve sit on the bench (in the BOM, not on the arm). When the homing reference has a
  camera (:data:`actuators.sensors.REFERENCES`), it hangs under the shelf ahead of the cup
  (away from the arm) on four M2 screws, two nuts a spacer each, looking straight down;
  the shelf's holes fix its offset from the cup (``notes["tool"]["camera"]``).
* ``suction_wrist``: the same cup turned about the vertical by a NEMA 11 stepper, the arm's
  fourth axis (``q4``, :class:`kinematics.SpatialJoint`): the motor stands on a printed
  holder's shelf, shaft down through it, the cup on a printed coupler on the shaft.

What the tool weighs is the payload the drives' sag counts on top of the work's
(:mod:`spiderpig.precision`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from spiderpig.construction.base import (
    Build,
    ConstructionError,
    Context,
    Group,
    Realized,
    hardware,
    stock_length,
)
from spiderpig.construction.plane import Rect, point_rect, seg_rect, seg_rect_many
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.shapes import Cut, box, cut_holes, difference, disc, pill, ring, union

TAB_R = 6.0          # the holder's tab along the end plate's arm: its half-width (mm)
TAB_L = 15.0         # how far the tab runs from the flange toward the arm's other end
HOLES_AT = (5.0, 12.5)   # the two screws along the tab from the flange (2 x the hanger's
#                          thickness of web between them)
BLOCK = 8.0          # the plain cup's shelf top under the flange (the end plate's round
#                      over it: link_radius + BLOCK_GAP at most)
BLOCK_GAP = 1.0
WALL = 5.0           # the wrist holder's tab, cap, wall and shelf thickness
CAP = 7.5            # the wrist's cap stands this far under the flange: the plate's end
#                      over (an arm's 6.6 mm links, config.KIND_FIT) and CAP_GAP
COUPLER = 10.0       # the wrist's printed coupler on the motor's shaft
NUT_H = 2.4
SCREW_HOLE = 3.4
HOLDER_COLOR = "#e8a33d"
PLATE_COLOR = "#b8bcc2"
CUP_COLOR = "#303030"
MARGIN = 1.5         # the tool's clearance to whatever moves past it
CAP_GAP = 0.5        # the end plate's end to the wrist's cap, the least
TOOL_SHEET = "al5052_2mm"      # the hanger and the shelf (SendCutSend, the frame's sheet)
SLOT_NUT = "m3_nut"            # in each T-slot's crossbar (standing out of the 2 mm plate)
SLOT_CLEAR = 0.5     # a T-slot's nut and screw, wider than the hanger, to the end plate
COLUMN = "m5_coupling_nut_15"  # the cup's column under the shelf
FITTING = "barb_m5_4mm"        # its push-in fitting on the shelf
CUP_HOLE = 5.5       # M5 clearance (ISO 273 medium)
CAM_GAP = 1.0        # the camera's board edge to the cup's column
CAM_NUTS = 2         # M2 nuts stacked under the shelf at each camera screw (its spacer)
CAM_HOLE = 2.4       # M2 clearance
TAB = (-2.5, 1.5)    # the hanger's tab into the shelf, under the flange (a from it)
TAB_FIT = 0.1        # the shelf's slot round it


@dataclass(frozen=True)
class Tool:
    """A tool for the flange (module docstring). ``wrist``: the stepper turning it
    (:data:`spiderpig.actuators.STEPPERS`; ``""``: a fixed tool); ``camera``: the homing
    camera it carries (a catalog key; :func:`tool_plan` sets it from the reference)."""

    key: str
    label: str
    cup: str = "suction_cup_m5_10"
    wrist: str = ""
    camera: str = ""

    def cup_dims(self) -> dict:
        from spiderpig.hardware.catalog import get

        return get(self.cup).dims

    def width(self) -> float:
        """The holder's tab across the plane, from the end plate's face (mm): the wrist's
        printed wall, the plain cup's hanger plate."""
        return WALL if self.wrist else sheet_t()

    def cup_w(self) -> float:
        """The plain cup's axis out from the end plate's face (mm): past the hanger and its
        screws' nuts, by the fitting's radius."""
        return sheet_t() + NUT_H + 0.5 + _dim(FITTING, "od") / 2

    def motor(self):
        from spiderpig import actuators

        return actuators.get(self.wrist)

    def drop(self) -> float:
        """How far its lowest point (the cup's lip) hangs under the flange (mm)."""
        h = float(self.cup_dims()["h"])
        if not self.wrist:
            return BLOCK + sheet_t() + _dim(COLUMN, "length") + h
        return CAP + WALL + self.motor().length + WALL + 1.0 + COUPLER + h


def sheet_t() -> float:
    from spiderpig.materials import sheet

    return sheet(TOOL_SHEET).thickness


def _dim(key: str, name: str) -> float:
    from spiderpig.hardware.catalog import get

    return float(get(key).dims[name])


def _edge() -> float:
    from spiderpig.materials import sheet

    return sheet(TOOL_SHEET).min_edge


def camera_of(bom) -> str:
    """The camera among a homing reference's parts (``(key, quantity)``), else ``""``."""
    from spiderpig.hardware.catalog import get

    return next((k for k, _ in bom if k.startswith("camera_") and "board" in get(k).dims), "")


TOOLS: dict[str, Tool] = {t.key: t for t in (
    Tool("suction_cup", "a 10 mm vacuum cup under the flange, on two laser-cut plates"),
    Tool("suction_wrist", "the vacuum cup turned by a NEMA 11 stepper (the fourth axis)",
         wrist="nema11_11hs20"),
)}
DEFAULT = "suction_cup"


@dataclass(frozen=True)
class PlateLayout:
    """The plain cup's plates, in the end plate's frame from the flange: ``a`` forward
    (away from the arm's other end), ``y`` up, ``w`` out from the end plate's face the
    hanger bears on (mm). ``slots``: the T-slot's ``a``; ``tab``: the hanger's locating
    tab ``(a0, a1)`` into the shelf; ``hanger``: its lower body
    ``(a0, a1, y0, y1)`` (the tab along the arm above it); ``shelf``: its outline, rectangles
    ``(a0, a1, w0, w1)``; ``cam``: the camera's centre ``(a, w)`` or ``None``."""

    t: float
    slots: tuple[float, ...]
    tab: tuple[float, float]
    slot_screw: float
    cross: tuple[float, float]
    slot_top: float
    hanger: tuple[float, float, float, float]
    shelf: tuple[tuple[float, float, float, float], ...]
    cup_w: float
    cam: tuple[float, float] | None

    @property
    def shelf_y(self) -> tuple[float, float]:
        return (-BLOCK - self.t, -BLOCK)


def _cross_w() -> float:
    """A T-slot's crossbar along the edge: the nut's corners and the cut's round corners."""
    return _dim(SLOT_NUT, "af") / math.cos(math.radians(30)) + 0.8


def _slot_parts(t: float) -> tuple[float, tuple[float, float], float]:
    """The T-slot screw's length, its crossbar's ``y`` span and the slot's top (from the
    flange): the crossbar a web of the sheet's least edge above the hanger's bottom edge,
    the screw through the shelf and the nut, its tip short of the slot's end."""
    nut_h = _dim(SLOT_NUT, "h")
    cb0 = -BLOCK + _edge()
    sk = screw("shcs", "3")
    length = stock_length(sk, t + _edge() + nut_h + 0.3, "tool")
    return length, (cb0, cb0 + nut_h + 0.3), -BLOCK - t + length + 0.5


def slot_boxes(a: float, t: float) -> list[tuple[float, float, float, float]]:
    """What of a T-slot at ``a`` stands out of the hanger's faces: its nut and its screw's
    shank, ``(a0, y0, a1, y1)``."""
    length, cross, _ = _slot_parts(t)
    nut = _dim(SLOT_NUT, "af") / math.cos(math.radians(30)) / 2
    shank = screw("shcs", "3").shank_d / 2
    return [(a - nut, cross[0], a + nut, cross[0] + _dim(SLOT_NUT, "h")),
            (a - shank, -BLOCK, a + shank, -BLOCK - t + length)]


def layout(tool: Tool, slot: float) -> PlateLayout:
    """The plain cup's plates, the T-slot ``slot`` behind the flange (:func:`tool_plan`)."""
    t, e = sheet_t(), _edge()
    cw = _cross_w()
    slots = (-slot,)
    length, cross, top = _slot_parts(t)
    back = min(slots) - max(cw / 2, SCREW_HOLE / 2) - e
    hanger = (back, TAB_R, -BLOCK, max(top + e, TAB_R))   # (its top meets the tab's side)
    wc = tool.cup_w()
    hole = SCREW_HOLE / 2 + e
    cup = CUP_HOLE / 2 + e
    cam = None
    front, w0, w1 = cup, min(t / 2 - hole, wc - cup), wc + cup
    if tool.camera:
        bd = _cam(tool.camera)
        col = max(float(tool.cup_dims()["d"]) / 2,
                  _dim(COLUMN, "af") / math.cos(math.radians(30)) / 2)
        # centred halfway from the end plate's face to the cup: its board, wider than the
        # tool, reaches under the end plate (over it only the shelf) as far as past the cup
        cam = (col + CAM_GAP + bd["board"][0] / 2, wc / 2)
        px, pw = bd["pitch"]
        front = cam[0] + px / 2 + CAM_HOLE / 2 + e
        reach = pw / 2 + CAM_HOLE / 2 + e
        w0, w1 = min(w0, cam[1] - reach), max(w1, cam[1] + reach)
    shelf = ((back, -cup, t / 2 - hole, t / 2 + hole), (-cup, front, w0, w1))
    return PlateLayout(t, slots, TAB, length, cross, top, hanger, shelf, wc, cam)


def _cam(key: str) -> dict:
    """A camera's board ``(along a, along w, height with its lens)``, hole pitch, hole and
    lens diameters (the catalog's; a board without them: the Raspberry Pi camera's
    21 x 12.5 mm M2 pattern round the lens)."""
    from spiderpig.hardware.catalog import get

    d = get(key).dims
    return {"board": tuple(float(v) for v in d["board"]),
            "pitch": tuple(float(v) for v in d.get("hole_pitch", (21.0, 12.5))),
            "hole": float(d.get("hole_d", 2.2)), "lens": float(d.get("lens_d", 14.0)),
            "board_t": float(d.get("board_t", 1.0))}


def cam_spacer() -> float:
    return CAM_NUTS * _dim("m2_nut", "h")


@dataclass(frozen=True)
class ToolPlan:
    """Where the tool goes: the output body and point, the arm's other end ``toward``, the
    side of the end plate its tab bears on (``s``: +1 up the stack), and what it fills:
    ``regions``, each an outline in the end plate's frame relative to the flange (a box)
    over a z span. The plain cup's: ``fwd`` the sign of ``x`` away from ``toward`` and
    ``slot`` its first T-slot behind the flange (:func:`layout`)."""

    tool: Tool
    body: str
    point: str
    toward: str
    s: int
    regions: tuple[tuple[Rect, tuple[float, float]], ...]
    fwd: int = 1
    slot: float = 0.0

    def layout(self) -> PlateLayout:
        return layout(self.tool, self.slot)


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


def axis_z(plan: ToolPlan, lo: float, hi: float, floor: float, ceil: float) -> float:
    """The tool's axis across the plane (its cup's centre): the plain cup's column outboard
    of the hanger beside the end plate (``lo..hi``), the wrist's under it
    (:func:`wrist_z`)."""
    if plan.tool.wrist:
        return wrist_z(plan.tool, lo, hi, floor, ceil)
    face = hi if plan.s > 0 else lo
    return face + plan.s * plan.tool.cup_w()


def _span(face: float, s: int, w0: float, w1: float) -> tuple[float, float]:
    a, b = face + s * w0, face + s * w1
    return (min(a, b), max(a, b))


def _rect(fwd: int, a0: float, a1: float, y0: float, y1: float) -> Rect:
    x0, x1 = sorted((fwd * a0, fwd * a1))
    return (x0, y0, x1, y1)


def _regions(tool: Tool, u: np.ndarray, s: int, lo: float, hi: float, floor: float,
             ceil: float, fwd: int = 1, slot: float = 0.0
             ) -> list[tuple[Rect, tuple[float, float]]]:
    """What the tool fills beside an end plate in ``lo..hi``, its tab on side ``s``: the
    plain cup's hanger with its tab, its screws' heads on the plate's other face, their
    nuts and the T-slot's nut (over the tab's z); the fitting on the shelf; the shelf, the
    column, the cup and the camera under it. The wrist's tab, then its yoke, motor and cup
    under the flange, centred on the plate."""
    tip = TAB_L * u
    head = screw("shcs", "3").head_h
    face, other = (hi, lo) if s > 0 else (lo, hi)
    top = max(TAB_R, tip[1] + TAB_R)
    xs = (min(-TAB_R, tip[0] - TAB_R), max(TAB_R, tip[0] + TAB_R))
    if not tool.wrist:
        lay = layout(tool, slot)
        ends = (other - s * head, face + s * (lay.t + NUT_H + 0.5))
        tab = (min(ends), max(ends))
        a0, a1, _, y1 = lay.hanger
        hangers = [((xs[0], -BLOCK, xs[1], top), tab), (_rect(fwd, a0, a1, -BLOCK, y1), tab)]
        r_fit = _dim(FITTING, "od") / 2
        fitting = (_rect(fwd, -r_fit, r_fit, -BLOCK, -BLOCK + _dim(FITTING, "h")),
                   _span(face, s, lay.cup_w - r_fit, lay.cup_w + r_fit))
        sa = [r[0] for r in lay.shelf] + [r[1] for r in lay.shelf]
        sw = [r[2] for r in lay.shelf] + [r[3] for r in lay.shelf]
        y_top = -BLOCK + (screw("shcs", "2").head_h if lay.cam else 0.0)
        if lay.cam:
            bd = _cam(tool.camera)
            sa += [lay.cam[0] - bd["board"][0] / 2, lay.cam[0] + bd["board"][0] / 2]
            sw += [lay.cam[1] - bd["board"][1] / 2, lay.cam[1] + bd["board"][1] / 2]
        under = (_rect(fwd, min(sa), max(sa), -tool.drop(), y_top),
                 _span(face, s, min(sw), max(sw)))
        return [*hangers, fitting, under]
    head_end = other - s * head
    ends = (head_end, face + s * (tool.width() + NUT_H + 0.5))
    tab = (min(ends), max(ends))
    mo = tool.motor()
    half = mo.frame / 2 + WALL
    zb = wrist_z(tool, lo, hi, floor, ceil)
    hz = mo.frame / 2 + 0.5 + WALL
    return [((xs[0], -(CAP + WALL), xs[1], top), tab),
            ((-half, -tool.drop(), half, -CAP), (zb - hz, zb + hz))]


def _first_slot(ctx: Context, body: str, f0: np.ndarray, fwd: int) -> float:
    """How far behind the flange the first T-slot goes: the least that keeps its nut and
    screw, standing out of the hanger under the end plate, ``SLOT_CLEAR`` off the plate's
    outline (its arms, ``link_radius`` round each)."""
    pts = ctx.topo.geometry.points
    t = sheet_t()
    segs = [(np.asarray(pts[a], dtype=float).reshape(-1, 2)[0] - f0,
             np.asarray(pts[b], dtype=float).reshape(-1, 2)[0] - f0)
            for a, b in ctx.topo.links[body]]
    need = ctx.params.link_radius + SLOT_CLEAR
    for slot in np.arange(_cross_w() / 2, 80.0, 0.5):
        boxes = slot_boxes(-float(slot), t)
        if all(seg_rect(p, q, _rect(fwd, b[0], b[2], b[1], b[3])) >= need
               for p, q in segs for b in boxes):
            return float(slot)
    raise ConstructionError(f"tool: no place for the shelf's T-slot behind {body}'s "
                            "flange clear of the plate")


def tool_plan(ctx: Context, plan, tool: Tool, groups=(), camera: str = "") -> ToolPlan:
    """The tool's place (module docstring); raises :class:`ConstructionError` when neither
    side of the end plate has room near the flange over the box. ``camera``: the homing
    camera the plain cup carries (:func:`camera_of`)."""
    lk = ctx.config.lk
    if lk.output is None or lk.output.kind != "point" or not lk.output.body:
        raise ConstructionError(f"tool: {lk.key} has no output point on a body to hold one")
    if tool.wrist and ctx.params.link_radius + CAP_GAP > CAP + 1e-9:
        raise ConstructionError(f"tool {tool.key}: its cap {CAP:g} mm under the flange meets "
                                f"the end plate's {ctx.params.link_radius:g} mm-radius end "
                                f"(link_radius {CAP - CAP_GAP:g} at most)")
    if not tool.wrist and ctx.params.link_radius + BLOCK_GAP > BLOCK + 1e-9:
        raise ConstructionError(f"tool {tool.key}: its shelf {BLOCK:g} mm under the flange "
                                f"meets the end plate's {ctx.params.link_radius:g} mm-radius "
                                f"end (link_radius {BLOCK - BLOCK_GAP:g} at most)")
    if camera and not tool.wrist:
        tool = replace(tool, camera=camera)
    body = lk.output.body
    point = ctx.topo.point_of.get((body, lk.output.point), lk.output.point)
    toward = _toward(ctx, body, point)
    pts = ctx.topo.geometry.points
    f, w = np.asarray(pts[point], dtype=float), np.asarray(pts[toward], dtype=float)
    d = w - f
    ang = np.arctan2(d[:, 1], d[:, 0])
    u0 = d[0] / np.linalg.norm(d[0])
    fwd = -1 if u0[0] > 0 else 1
    slot = 0.0 if tool.wrist else _first_slot(ctx, body, f[0], fwd)
    k = plan.layers[body]
    lo, hi = plan.z(k)
    floor, ceil = plan.z(0)[1], plan.z(plan.top)[0]
    hubs = drive_hubs(ctx, groups)
    if not tool.wrist:
        _own_pins(ctx, body, point, f[0], fwd, layout(tool, slot))
    tried = []
    for s in ((1, -1) if k <= plan.top / 2 else (-1, 1)):
        side = "up" if s > 0 else "down"
        regions = _regions(tool, u0, s, lo, hi, floor, ceil, fwd, slot)
        if any(z[0] < floor - 1e-9 or z[1] > ceil + 1e-9 for _, z in regions):
            tried.append(f"{side} the stack it reaches a frame plate")
            continue
        hit = next((h for rect, z in regions
                    if (h := _hits(ctx, plan, body, f, ang, rect, z, hubs, groups)) is not None),
                   None)
        if hit is None:
            return ToolPlan(tool, body, point, toward, s, tuple(regions), fwd, slot)
        tried.append(f"{side} the stack it meets {hit}")
    raise ConstructionError(f"tool {tool.key}: no room beside {body} at {point}: "
                            + "; ".join(tried))


def _own_pins(ctx: Context, body: str, point: str, f0: np.ndarray, fwd: int,
              lay: PlateLayout) -> None:
    """The hanger's lower body clear of the end plate's own pins (their heads on its face:
    :func:`_hits` passes the plate's own axles)."""
    from spiderpig import construction

    head = construction.axle(ctx.config.pin).dims(ctx, False).head
    a0, a1, y0, y1 = lay.hanger
    r = _rect(fwd, a0, a1, y0, y1)
    for ax in ctx.topo.axes:
        if body not in ax.members or ax.name == point:
            continue
        q = np.asarray(ctx.topo.geometry.points[ax.name], dtype=float).reshape(-1, 2)[0] - f0
        if point_rect(q, r) < head + MARGIN:
            raise ConstructionError(f"tool: the hanger meets {body}'s pin at {ax.name}")


def _hits(ctx: Context, plan, body: str, f: np.ndarray, ang: np.ndarray, rect: Rect,
          span: tuple[float, float], hubs: dict[str, float], groups=()) -> str | None:
    """What meets the tool's outline ``rect`` over the box (in ``body``'s frame, its turn
    from the first sample undone): a link in a layer ``span`` reaches, a pin whose axle
    crosses those layers, a drive's belt or cord there, a frame pivot's pillar or drive
    hub; ``None``: nothing."""
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
    def crosses(layers) -> bool:
        z0, z1 = plan.z(min(layers))[0], plan.z(max(layers))[1]
        return z1 > span[0] + 1e-9 and z0 < span[1] - 1e-9

    for ax in ctx.topo.axes:                    # an axle crossing the tool's layers
        if ax.kind == "frame" or not ax.members:
            continue
        layers = [plan.layers[m] for m in ax.members if m in plan.layers]
        if not layers or body in ax.members or not crosses(layers):
            continue
        lp = local(np.asarray(pts[ax.name], dtype=float))
        if (seg_rect_many(lp, lp, rect) < pin_head + MARGIN + lat.get(ax.name, 0.0)).any():
            return f"the axle at {ax.name}"
    from spiderpig.construction.transmissions import DriveGroup

    for g in groups:                            # a drive's belt or cord: static, its layers
        if not isinstance(g, DriveGroup):
            continue
        hub = g.hub(ctx)
        if not crosses(hub.layers(plan.layers[hub.link])):
            continue
        hull = g.transmission.hull(ctx, hub) or [g.transmission.hub_r(ctx),
                                                 g.transmission.motor_side_r(ctx)]
        o = np.asarray(pts[hub.axis], dtype=float).reshape(-1, 2)[0]
        m = np.asarray(hub.motor, dtype=float)
        # the belt's half-width grows linearly between the pulleys (BeltDrive.hull): a
        # capsule per short span, its larger end's radius
        fr = np.linspace(0.0, 1.0, 21)
        radii = np.interp(fr, np.linspace(0.0, 1.0, len(hull)), hull)
        along = [local(o + f_ * (m - o)) for f_ in fr]
        for i in range(len(fr) - 1):
            r = max(radii[i], radii[i + 1]) + MARGIN
            if (seg_rect_many(along[i], along[i + 1], rect) < r).any():
                return f"{g.name}'s drive"
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

    def __init__(self, plan: ToolPlan, extras: tuple[tuple[str, float], ...] = ()):
        self.plan = plan
        self.extras = extras        # what else rides the tool: a homing camera

    def claims(self, ctx: Context) -> list:
        return []        # the robot's: checked against the side's links by tool_plan

    def realize(self, build: Build, done: Realized) -> Realized:
        out = self._realize_wrist(build) if self.plan.tool.wrist else self._realize_plates(build)
        out.extras += [BomLine(FITTING, 1, "tool: the vacuum port on the cup's column"),
                       BomLine("pu_tube_4mm", 1.0, "tool: the cup to the valve"),
                       BomLine("vacuum_pump_12v", 1, "tool: on the bench"),
                       BomLine("solenoid_valve_12v_3way", 1, "tool: releases the cup"),
                       *(BomLine(k, q, "tool: the homing reference's")
                         for k, q in self.extras if k != self.plan.tool.camera)]
        if camera_of(self.extras):
            out.extras.append(BomLine("csi_cable_pi_zero_30", 1, "tool: the camera to the "
                                      "Pi Zero 2 W's 22-pin CSI port"))
        return out

    def _realize_plates(self, build: Build) -> Realized:
        """The plain cup (module docstring): the hanger, the shelf, the column and the cup,
        the camera; every part placed from :func:`layout`."""
        tp, tool = self.plan, self.plan.tool
        out = Realized()
        lay = tp.layout()
        f = np.asarray(build.xy(tp.point), dtype=float)
        w_ = np.asarray(build.xy(tp.toward), dtype=float)
        u = (w_ - f) / np.linalg.norm(w_ - f)
        bz0, bz1 = build.z(build.layers[tp.body])
        face = bz1 if tp.s > 0 else bz0                   # the end plate's face it bears on
        t, s, fw = lay.t, tp.s, tp.fwd

        def X(a: float) -> float:
            return float(f[0] + fw * a)

        def Y(y: float) -> float:
            return float(f[1] + y)

        def Z(w: float) -> float:
            return face + s * w

        def plate_part(name: str, part):
            return hardware(name, part, tp.body, fab="laser", color=PLATE_COLOR,
                            sheet=TOOL_SHEET)

        # the hanger: the tab along the arm, its lower body down to the shelf, the T-slot
        hz = tuple(sorted((Z(0.0), Z(t))))
        a0, a1, y0, y1 = lay.hanger
        ta0, ta1 = lay.tab
        hanger = union([pill(tuple(f), tuple(f + TAB_L * u), TAB_R, *hz),
                        box(((X(a0) + X(a1)) / 2, Y((y0 + y1) / 2)), (a1 - a0, y1 - y0,
                                                                     hz[1] - hz[0]), hz[0]),
                        box(((X(ta0) + X(ta1)) / 2, Y(-BLOCK - t / 2)),
                            (ta1 - ta0, t + 0.01, hz[1] - hz[0]), hz[0])])
        holes = [f + a * u for a in HOLES_AT]
        hanger = cut_holes(hanger, [Cut(tuple(xy), SCREW_HOLE) for xy in holes], *hz)
        cw = _cross_w()
        slots = []
        for a in lay.slots:
            slots.append(box((X(a), Y((-BLOCK - 1.0 + lay.slot_top) / 2)),
                             (SCREW_HOLE, lay.slot_top + BLOCK + 1.0, hz[1] - hz[0] + 2),
                             hz[0] - 1))
            slots.append(box((X(a), Y(sum(lay.cross) / 2)),
                             (cw, lay.cross[1] - lay.cross[0], hz[1] - hz[0] + 2), hz[0] - 1))
        out.bodies.append(plate_part("tool_hanger", difference(hanger, *slots)))

        # the shelf, level under the hanger (drawn in x-z, stood along y: _from_xz)
        sy0, sy1 = (Y(v) for v in lay.shelf_y)
        rects = []
        for b0, b1, w0, w1 in lay.shelf:
            xs, zs = sorted((X(b0), X(b1))), sorted((Z(w0), Z(w1)))
            rects.append(box(((xs[0] + xs[1]) / 2, (zs[0] + zs[1]) / 2),
                             (xs[1] - xs[0], zs[1] - zs[0], sy1 - sy0), sy0))
        cup_xz = (X(0.0), Z(lay.cup_w))
        cuts: list = [Cut(cup_xz, CUP_HOLE)] + [Cut((X(a), Z(t / 2)), SCREW_HOLE)
                                                for a in lay.slots]
        cam_xz: list[tuple[float, float]] = []
        if lay.cam:
            bd = _cam(tool.camera)
            px, pw = bd["pitch"]
            cam_xz = [(X(lay.cam[0] + i * px / 2), Z(lay.cam[1] + j * pw / 2))
                      for i in (-1, 1) for j in (-1, 1)]
            cuts += [Cut(xz, CAM_HOLE) for xz in cam_xz]
        shelf = cut_holes(union(rects), cuts, sy0, sy1)
        r_tab = t / 2 + TAB_FIT        # the tab's slot, round-ended past the tab's corners
        shelf = difference(shelf, pill((X(ta0 - TAB_FIT), Z(t / 2)),
                                       (X(ta1 + TAB_FIT), Z(t / 2)), r_tab, sy0 - 1.0,
                                       sy1 + 1.0))
        out.bodies.append(plate_part("tool_shelf", _from_xz(shelf)))

        # the T-slot's screw, up through the shelf into the nut in the hanger's crossbar
        sk = screw("shcs", "3")
        for i, a in enumerate(lay.slots):
            out.bodies.append(hardware(f"tool_slot_screw{i}", _from_xz(screw_solid(
                (X(a), Z(t / 2)), sk, sy0, lay.slot_screw, up=True)), tp.body,
                fab="purchased", bom_key=sk.key(lay.slot_screw), color="#4a4a4a"))
        out.extras.append(BomLine(SLOT_NUT, len(lay.slots), "tool: in the hanger's T-slot"))

        # the column under the shelf (an M5 coupling nut), the cup on its lower end
        corner = _dim(COLUMN, "af") / math.cos(math.radians(30)) / 2
        col0 = sy0 - _dim(COLUMN, "length")
        out.bodies.append(hardware("tool_column", _from_xz(disc(cup_xz, corner, col0, sy0)),
                                   tp.body, fab="purchased", bom_key=COLUMN, color="#c9a227"))
        cup = tool.cup_dims()
        out.bodies.append(hardware("tool_cup", _from_xz(disc(
            cup_xz, float(cup["d"]) / 2, col0 - float(cup["h"]), col0)), tp.body,
            fab="purchased", bom_key=tool.cup, color=CUP_COLOR))

        # the camera under the shelf ahead of the cup, on its four M2 screws
        info: dict = {}
        if lay.cam:
            bd = _cam(tool.camera)
            top = sy0 - cam_spacer()
            bt = bd["board_t"]
            c = (X(lay.cam[0]), Z(lay.cam[1]))
            board = cut_holes(box(c, (bd["board"][0], bd["board"][1], bt), top - bt),
                              [Cut(xz, bd["hole"]) for xz in cam_xz], top - bt, top)
            lens = disc(c, bd["lens"] / 2, top - bd["board"][2], top - bt)
            out.bodies.append(hardware("tool_camera", _from_xz(union([board, lens])), tp.body,
                                       fab="purchased", bom_key=tool.camera, color="#2f6b3a"))
            m2 = screw("shcs", "2")
            nut_h = _dim("m2_nut", "h")
            mlen = stock_length(m2, t + cam_spacer() + bt + nut_h, "tool: the camera")
            for i, xz in enumerate(cam_xz):
                out.bodies.append(hardware(f"tool_camera_screw{i}", _from_xz(screw_solid(
                    xz, m2, sy1, mlen, up=False)), tp.body, fab="purchased",
                    bom_key=m2.key(mlen), color="#4a4a4a"))
            out.extras.append(BomLine("m2_nut", (CAM_NUTS + 1) * len(cam_xz),
                                      f"tool: the camera's screws, {CAM_NUTS} a spacer and "
                                      "one under the board"))
            info["camera"] = [round(fw * lay.cam[0], 3), round(top - bd["board"][2] - (
                col0 - float(cup["h"])), 3), round(s * (lay.cam[1] - lay.cup_w), 3)]
            info["camera_key"] = tool.camera

        # the hanger's screws: heads on the end plate's other face, nuts on its far face
        link_t = bz1 - bz0
        length = stock_length(sk, link_t + t + NUT_H, "tool")
        head_z = bz0 if tp.s > 0 else bz1
        for i, xy in enumerate(holes):
            out.bodies.append(hardware(f"tool_screw{i}", screw_solid(
                tuple(xy), sk, head_z, length, up=tp.s > 0), tp.body, fab="purchased",
                bom_key=sk.key(length), color="#4a4a4a"))
            out.cut(tp.body, Cut(tuple(xy), SCREW_HOLE))
        out.extras.append(BomLine("m3_nut", len(holes), "tool: the hanger's screws"))
        out.notes["tool"] = {
            "key": tool.key, "body": tp.body, "point": tp.point, "side": tp.s,
            "drop_mm": round(tool.drop(), 3), "wrist": "",
            "axis": [float(f[0]), float(f[1]), float(Z(lay.cup_w))], "rotating": [], **info}
        return out

    def _realize_wrist(self, build: Build) -> Realized:
        """The wrist (module docstring): its printed holder and coupler."""
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
        zm = axis_z(tp, bz0, bz1, build.z(0)[1], build.z(build.top)[0])
        drop = CAP + WALL
        parts = [pill(tuple(f), tuple(f + TAB_L * u), TAB_R, *tz),
                 box((f[0], f[1] - drop / 2), (2 * TAB_R, drop, tz[1] - tz[0]), tz[0])]
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
        mlen = stock_length(msk, WALL + mo.hole_depth * 0.8, "tool: the wrist's motor")
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
        holder = union(parts)
        holder = cut_holes(holder, [Cut(tuple(xy), SCREW_HOLE) for xy in holes], *tz)
        holder = difference(holder, *cuts)
        out.bodies.append(hardware("tool_holder", holder, tp.body, fab="printed",
                                   color=HOLDER_COLOR))
        cup_part = _from_xz(disc((f[0], zm), float(cup["d"]) / 2, cup_top - float(cup["h"]),
                                 cup_top))
        out.bodies.append(hardware("tool_cup", cup_part, tp.body, fab="purchased",
                                   bom_key=tool.cup, color=CUP_COLOR))
        rotating.append("tool_cup")
        out.notes["fastened"] = {f"wrist_screw{i}|wrist_motor": [f"wrist_screw{i}",
                                                               "wrist_motor"]
                                 for i in range(4)}
        # the holder's screws: heads on the end plate's other face, nuts on the tab's far face
        link_t = bz1 - bz0
        length = stock_length(sk, link_t + abs(tab_far - face) + NUT_H, "tool")
        head_z = bz0 if tp.s > 0 else bz1
        for i, xy in enumerate(holes):
            out.bodies.append(hardware(f"tool_screw{i}", screw_solid(
                tuple(xy), sk, head_z, length, up=tp.s > 0), tp.body, fab="purchased",
                bom_key=sk.key(length), color="#4a4a4a"))
            out.cut(tp.body, Cut(tuple(xy), SCREW_HOLE))
        out.extras.append(BomLine("m3_nut", len(holes), "tool: the holder's screws"))
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
    """The tool's reach in the plane round its axis at about its tip's depth, toward the
    arm's other end (for the robot's floor and the turret, which the flange approaches
    from ahead): the cup and its column; the wrist's yoke. The plain cup's shelf and
    hanger stand some 25 mm over the cup's lip; its camera rides ahead of the cup, away
    from the arm (:func:`camera_reach`), so with the flange ahead of the yaw axis it comes
    over the turret only after the cup."""
    r = max(TAB_R, float(tool.cup_dims()["d"]) / 2)
    if tool.wrist:
        return max(r, math.hypot(1, 1) * 15.0)
    return max(r, _dim(COLUMN, "af") / math.cos(math.radians(30)) / 2)


def camera_reach(tool: Tool) -> float:
    """How far ahead of the cup's axis the plain cup's camera reaches (mm; 0 without)."""
    if tool.wrist or not tool.camera:
        return 0.0
    lay = layout(tool, 0.0)
    assert lay.cam is not None
    return lay.cam[0] + _cam(tool.camera)["board"][0] / 2
