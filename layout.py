"""2D sheet packing + DXF emission for every laser-cut part.

Given a fabricated :class:`mechanism.Mechanism`, we:

1. Select the laser-cut bodies (``fab == "laser"``): links, frame plate,
   crankshaft plates, spacer rings, brackets.
2. Slice each part through the middle of its own slot to get its 2D profile.
   Links are turned so their long axis runs along X; any part too long for
   the sheet is laid along the diagonal if that fits.
3. Compensate for the laser's kerf: outlines grow and holes shrink by half
   the kerf, so cut parts come out at their nominal size.
4. Pack the profiles' bounding boxes onto fixed-size sheets with rectpack. A
   part that fits no sheet is an error, never a silent drop.
5. Emit one DXF per sheet: outer contours as LWPOLYLINE, round holes as
   CIRCLE, all on layer ``CUT``, units = mm.
"""

from __future__ import annotations

import math
from pathlib import Path

import ezdxf
import numpy as np
from build123d import Axis, GeomType, Plane, section
from rectpack import newPacker

_CUT_LAYER = "CUT"
_DEFAULT_SHEET = (200.0, 200.0)
_MARGIN = 3.0
_WIRE_SAMPLES = 96  # N-gon resolution for non-circle curves
DEFAULT_KERF = 0.15


def laser_bodies(mech) -> list:
    return [b for b in mech.bodies if b.fab == "laser" and b.part is not None]


def _wire_points(wire, n: int = _WIRE_SAMPLES):
    """Sample a wire into a closed 2D polyline ``[(x, y), ...]``."""
    return [(v.X, v.Y) for v in (wire.position_at(u) for u in np.linspace(0, 1, n, endpoint=False))]


def _wire_is_circle(wire):
    edges = wire.edges()
    if len(edges) != 1 or edges[0].geom_type != GeomType.CIRCLE:
        return None
    c = edges[0].arc_center
    return (c.X, c.Y), float(edges[0].radius)


def _bbox_2d(shape):
    bb = shape.bounding_box()
    return bb.min.X, bb.min.Y, bb.max.X, bb.max.Y


def _long_axis_degrees(body) -> float:
    """Direction of a link's outline (first segment), in degrees; 0 for other parts."""
    if not body.outline or not body.joints:
        return 0.0
    p, q = body.outline[0]
    a = (body.pose @ body.joint(p).pose).matrix[:2, 3]
    b = (body.pose @ body.joint(q).pose).matrix[:2, 3]
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def _profile(body, sheet: tuple[float, float], margin: float):
    """The body's mid-slot section, turned to lie flat (or along the sheet diagonal)."""
    bb = body.part.bounding_box()
    sketch = section(body.part, Plane.XY.offset((bb.min.Z + bb.max.Z) / 2))
    sketch = sketch.rotate(Axis.Z, -_long_axis_degrees(body))
    usable = (sheet[0] - 2 * margin, sheet[1] - 2 * margin)
    for extra in (0.0, 90.0, math.degrees(math.atan2(usable[1], usable[0]))):
        turned = sketch.rotate(Axis.Z, extra) if extra else sketch
        x0, y0, x1, y1 = _bbox_2d(turned)
        if x1 - x0 <= usable[0] and y1 - y0 <= usable[1]:
            return turned
    raise ValueError(
        f"{body.name} ({x1 - x0:.0f} x {y1 - y0:.0f} mm) does not fit a "
        f"{sheet[0]:.0f} x {sheet[1]:.0f} mm sheet with {margin:.0f} mm margins"
    )


def _outer(wires):
    def area(w):
        x0, y0, x1, y1 = _bbox_2d(w)
        return (x1 - x0) * (y1 - y0)

    return max(wires, key=area)


def _emit(msp, wire, offset_xy, grow: float):
    """Emit a wire shifted by ``offset_xy``, offset by ``grow`` (kerf compensation)."""
    ox, oy = offset_xy
    circle = _wire_is_circle(wire)
    if circle is not None:
        (cx, cy), r = circle
        msp.add_circle((cx + ox, cy + oy), r + grow, dxfattribs={"layer": _CUT_LAYER})
        return
    if abs(grow) > 1e-9:
        wire = wire.offset_2d(grow)
    msp.add_lwpolyline(
        [(x + ox, y + oy) for x, y in _wire_points(wire)],
        close=True,
        dxfattribs={"layer": _CUT_LAYER},
    )


def save_sheets(
    mech,
    prefix,
    sheet_size: tuple[float, float] = _DEFAULT_SHEET,
    margin: float = _MARGIN,
    kerf: float = DEFAULT_KERF,
) -> list[Path]:
    """Pack the mechanism's laser-cut parts onto sheets and write DXFs.

    Returns the DXF paths written. Raises if any part can't be placed.
    """
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    items = []
    for body in laser_bodies(mech):
        sketch = _profile(body, sheet_size, margin)
        x0, y0, x1, y1 = _bbox_2d(sketch)
        items.append((body.name, sketch, x0, y0, (x1 - x0) + 2 * margin, (y1 - y0) + 2 * margin))
    if not items:
        return []

    packer = newPacker(rotation=False)
    for rid, it in enumerate(items):
        packer.add_rect(math.ceil(it[4]), math.ceil(it[5]), rid=rid)
    for _ in items:  # plenty of bins; rectpack only uses what it fills
        packer.add_bin(*sheet_size)
    packer.pack()
    packed = {rect.rid for abin in packer for rect in abin}
    missing = [items[i][0] for i in range(len(items)) if i not in packed]
    if missing:
        raise ValueError(f"sheet packing dropped {missing}")

    written: list[Path] = []
    for sheet_idx, abin in enumerate(packer):
        doc = ezdxf.new(dxfversion="R2010")
        doc.units = ezdxf.units.MM
        if _CUT_LAYER not in doc.layers:
            doc.layers.add(name=_CUT_LAYER)
        msp = doc.modelspace()
        for rect in abin:
            _, sketch, x0, y0, _, _ = items[rect.rid]
            off = (rect.x + margin - x0, rect.y + margin - y0)
            wires = list(sketch.wires())
            outer = _outer(wires)
            for wire in wires:
                _emit(msp, wire, off, kerf / 2 if wire is outer else -kerf / 2)
        path = Path(f"{prefix}_{sheet_idx}.dxf")
        doc.saveas(str(path))
        written.append(path)
    return written
