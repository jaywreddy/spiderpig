"""2D sheet packing + DXF emission for laser-cut bodies.

Given a solved :class:`mechanism.Mechanism`, we:

1. Select bodies whose name looks like a laser-cut link (b1..b4, conn).
2. Slice each body's part at z = ``THICKNESS / 2`` to pull its 2D profile.
3. Translate each profile so its AABB is anchored at (0, 0) and pack the
   resulting rectangles onto fixed-size sheets with rectpack.
4. Emit one DXF per sheet with every outer contour as an LWPOLYLINE on
   layer ``CUT`` and every circular hole as a CIRCLE on layer ``CUT``.

The DXF header units are set to millimetres explicitly (ezdxf does not
default to mm).
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import ezdxf
from build123d import GeomType, Plane, section
from rectpack import newPacker

from shapes import THICKNESS

_LAYOUT_SKIP = re.compile(r"^(coupler|torso|pin|standoff)")
_CUT_LAYER = "CUT"
_DEFAULT_SHEET = (200.0, 200.0)
_MARGIN = 5.0
_WIRE_SAMPLES = 72  # N-gon resolution for non-circle curves


def _profile_wires(part, z: float = THICKNESS / 2):
    """Return the outer and inner closed wires of ``part`` at plane z=z."""
    sketch = section(part, Plane.XY.offset(z))
    return list(sketch.wires())


def _wire_to_polyline_points(wire, n: int = _WIRE_SAMPLES):
    """Sample a wire into a closed 2D polyline ``[(x, y), ...]``."""
    import numpy as np

    pts = []
    for u in np.linspace(0.0, 1.0, n, endpoint=False):
        v = wire.position_at(u)
        pts.append((v.X, v.Y))
    return pts


def _wire_is_circle(wire):
    edges = wire.edges()
    if len(edges) != 1:
        return None
    e = edges[0]
    if e.geom_type != GeomType.CIRCLE:
        return None
    c = e.arc_center
    return (c.X, c.Y), float(e.radius)


def _wire_bbox_2d(wire):
    bb = wire.bounding_box()
    return bb.min.X, bb.min.Y, bb.max.X, bb.max.Y


def _body_profile(body):
    """Return ``(outer_wire, inner_wires)`` at the mid-plane for a body."""
    wires = _profile_wires(body.part)
    if not wires:
        return None, []
    # Use the wire with the largest 2D AABB as the outer contour.
    def _aabb_area(w):
        x0, y0, x1, y1 = _wire_bbox_2d(w)
        return (x1 - x0) * (y1 - y0)

    outer = max(wires, key=_aabb_area)
    inner = [w for w in wires if w is not outer]
    return outer, inner


def _emit_wire_to_dxf(msp, wire, offset_xy, as_polyline_points=True):
    """Emit a wire (offset by ``offset_xy``) to a DXF modelspace.

    Circles are emitted as DXF ``CIRCLE`` for exactness; everything else is
    approximated as a closed ``LWPOLYLINE``.
    """
    ox, oy = offset_xy
    circle_info = _wire_is_circle(wire)
    if circle_info is not None:
        (cx, cy), r = circle_info
        msp.add_circle((cx + ox, cy + oy), r, dxfattribs={"layer": _CUT_LAYER})
        return

    pts = _wire_to_polyline_points(wire)
    shifted = [(x + ox, y + oy) for x, y in pts]
    msp.add_lwpolyline(
        shifted,
        dxfattribs={"layer": _CUT_LAYER, "closed": True},
    )


def save_sheets(
    mech,
    prefix,
    sheet_size: tuple[float, float] = _DEFAULT_SHEET,
    margin: float = _MARGIN,
) -> list[Path]:
    """Pack the mechanism's laser-cut bodies onto sheets and write DXFs.

    Returns the list of DXF paths actually written.
    """
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    # Collect eligible bodies + their 2D profiles.
    items = []
    for body in mech.bodies:
        if _LAYOUT_SKIP.match(body.name):
            continue
        if body.part is None:
            continue
        outer, inner = _body_profile(body)
        if outer is None:
            continue
        x0, y0, x1, y1 = _wire_bbox_2d(outer)
        w = (x1 - x0) + 2 * margin
        h = (y1 - y0) + 2 * margin
        items.append((body.name, outer, inner, x0, y0, w, h))

    if not items:
        return []

    packer = newPacker(rotation=False)
    for rid, it in enumerate(items):
        packer.add_rect(math.ceil(it[5]), math.ceil(it[6]), rid=rid)
    # Provide plenty of bins; rectpack adds only as many as it fills.
    for _ in range(len(items)):
        packer.add_bin(sheet_size[0], sheet_size[1])
    packer.pack()

    written: list[Path] = []
    for sheet_idx, abin in enumerate(packer):
        doc = ezdxf.new(dxfversion="R2010")
        doc.units = ezdxf.units.MM
        if _CUT_LAYER not in doc.layers:
            doc.layers.add(name=_CUT_LAYER)
        msp = doc.modelspace()

        for rect in abin:
            name, outer, inner, x0, y0, w_r, h_r = items[rect.rid]
            # translate profile's AABB-min to the rect's (x, y) + margin
            off = (rect.x + margin - x0, rect.y + margin - y0)
            _emit_wire_to_dxf(msp, outer, off)
            for iw in inner:
                _emit_wire_to_dxf(msp, iw, off)

        path = Path(f"{prefix}_{sheet_idx}.dxf")
        doc.saveas(str(path))
        written.append(path)

    return written
