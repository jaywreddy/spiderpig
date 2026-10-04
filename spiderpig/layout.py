"""2D sheet packing + DXF emission for every laser-cut part.

Given a fabricated :class:`mechanism.Mechanism` (one side or the whole
robot), we:

1. Select the laser-cut bodies (``fab == "laser"``): links, frame plates,
   centre plates, anything else a construction cuts from the sheet.
2. Slice each part through the middle of its own layer to get its 2D profile.
   Links are turned so their long axis runs along X, other plates along their
   principal axis; any part too long for the sheet is laid along the diagonal
   if that fits.
3. Compensate for the laser's kerf: outlines grow and holes shrink by half
   the kerf, so cut parts come out at their nominal size.
4. Pack the profiles' bounding boxes onto fixed-size sheets with rectpack
   (a box may be turned 90°). A part that fits no sheet is an error, never a
   silent drop. Every physical part is packed (a left and a right plate of
   the same shape are two parts on the sheet).
5. Emit one DXF per sheet: outer contours as LWPOLYLINE, round holes as
   CIRCLE, all on layer ``CUT``, units = mm; and ``<prefix>_parts.csv``
   saying which part is where.
"""

from __future__ import annotations

import csv
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
    """Direction of a link's outline (first segment), else of the part's principal axis."""
    if body.outline and body.joints:
        p, q = body.outline[0]
        a = (body.pose @ body.joint(p).pose).matrix[:2, 3]
        b = (body.pose @ body.joint(q).pose).matrix[:2, 3]
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    axes = sorted(body.part.principal_properties, key=lambda am: am[1])
    for axis, _ in axes:           # smallest moment: the long in-plane direction
        if abs(axis.Z) < 0.5:
            return math.degrees(math.atan2(axis.Y, axis.X))
    return 0.0


def _lay_flat(part):
    """The part turned so its thinnest bounding-box axis is Z: every plate of the stack
    already is; the electronics deck (:mod:`construction.deck`) lies in the XZ plane."""
    bb = part.bounding_box()
    size = (bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z)
    thin = min((2, 1, 0), key=lambda i: size[i])     # Z on a tie
    if size[thin] > size[2] - 1e-6:
        return part
    if thin == 1:
        return part.rotate(Axis.X, 90)
    if thin == 0:
        return part.rotate(Axis.Y, 90)
    return part


def _profile(body, sheet: tuple[float, float], margin: float):
    """The body's mid-slot section, turned to lie flat (or along the sheet diagonal)."""
    part = _lay_flat(body.part)
    bb = part.bounding_box()
    sketch = section(part, Plane.XY.offset((bb.min.Z + bb.max.Z) / 2))
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


def pack(mech, sheet_size: tuple[float, float] = _DEFAULT_SHEET, margin: float = _MARGIN):
    """Lay out every laser-cut part: ``[[(name, sketch, (dx, dy)), ...] per sheet]``.

    ``sketch`` is the part's profile as laid out, ``(dx, dy)`` moves it to its
    place on the sheet. Raises if any part can't be placed.
    """
    items = []
    for body in laser_bodies(mech):
        sketch = _profile(body, sheet_size, margin)
        x0, y0, x1, y1 = _bbox_2d(sketch)
        items.append((body.name, sketch, (x1 - x0) + 2 * margin, (y1 - y0) + 2 * margin))
    if not items:
        return []

    packer = newPacker(rotation=True)
    for rid, it in enumerate(items):
        packer.add_rect(math.ceil(it[2]), math.ceil(it[3]), rid=rid)
    for _ in items:  # plenty of bins; rectpack only uses what it fills
        packer.add_bin(*sheet_size)
    packer.pack()
    packed = {rect.rid for abin in packer for rect in abin}
    missing = [items[i][0] for i in range(len(items)) if i not in packed]
    if missing:
        raise ValueError(f"sheet packing dropped {missing}")

    sheets = []
    for abin in packer:
        placed = []
        for rect in abin:
            name, sketch, w, _ = items[rect.rid]
            if rect.width != math.ceil(w) and rect.height == math.ceil(w):   # turned 90°
                sketch = sketch.rotate(Axis.Z, 90.0)
            x0, y0, _, _ = _bbox_2d(sketch)
            placed.append((name, sketch, (rect.x + margin - x0, rect.y + margin - y0)))
        sheets.append(placed)
    return sheets


def sheet_key(body, default: str) -> str:
    """The sheet a laser-cut body is cut from (``Body.sheet``, else ``default``)."""
    return getattr(body, "sheet", None) or default


def by_sheet(mech, default: str) -> dict[str, list]:
    """The laser-cut bodies per sheet (material and thickness), the default sheet first."""
    out: dict[str, list] = {}
    for b in sorted(laser_bodies(mech), key=lambda b: sheet_key(b, default) != default):
        out.setdefault(sheet_key(b, default), []).append(b)
    return out


def pack_sheets(mech, default: str, sheet_size: tuple[float, float] | None = None,
                margin: float = _MARGIN) -> dict[str, list]:
    """:func:`pack` per sheet: ``{sheet key: [sheet, ...]}``, each on its own blank size
    (``sheet_size`` for all, when given)."""
    from types import SimpleNamespace

    from spiderpig.hardware.catalog import sheet_size as blank

    return {key: pack(SimpleNamespace(bodies=bodies), sheet_size or blank(key), margin)
            for key, bodies in by_sheet(mech, default).items()}


def sheet_lines(mech, default: str, sheet_size: tuple[float, float] | None = None) -> list:
    """The BOM's sheet lines: how many blanks of each sheet the laser-cut parts take."""
    from spiderpig.hardware.bom import BomLine

    return [BomLine(key, len(sheets), "laser-cut parts")
            for key, sheets in pack_sheets(mech, default, sheet_size).items() if sheets]


def save_sheets(
    mech,
    prefix,
    sheet_size: tuple[float, float] | None = None,
    margin: float = _MARGIN,
    kerf: float = DEFAULT_KERF,
    default: str | None = None,
) -> list[Path]:
    """Pack the mechanism's laser-cut parts onto sheets, one set per sheet (material and
    thickness: one order per service), and write DXFs.

    Returns the DXF paths written (``<prefix>_<sheet key>_<i>.dxf``); also writes
    ``<prefix>_parts.csv`` (sheet, part, position). Raises if any part can't
    be placed. ``default``: the sheet of a body that names none (the build's
    ``config.sheet``; ``mech.meta["sheet"]`` when not given).
    """
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    default = default or (mech.meta.get("sheet") if hasattr(mech, "meta") else None) \
        or "acrylic_3mm"
    written: list[Path] = []
    rows = []
    for key, sheets in pack_sheets(mech, default, sheet_size, margin).items():
        written += _write_sheets(sheets, Path(f"{prefix}_{key}"), kerf, key, rows)
    if rows:
        with open(f"{prefix}_parts.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["material", "sheet", "part", "x_mm", "y_mm", "width_mm", "height_mm"])
            w.writerows(rows)
    return written


def _write_sheets(sheets, prefix: Path, kerf: float, key: str, rows: list) -> list[Path]:
    written: list[Path] = []
    for sheet_idx, placed in enumerate(sheets):
        doc = ezdxf.new(dxfversion="R2010")
        doc.units = ezdxf.units.MM
        if _CUT_LAYER not in doc.layers:
            doc.layers.add(name=_CUT_LAYER)
        msp = doc.modelspace()
        for name, sketch, off in placed:
            wires = list(sketch.wires())
            outer = _outer(wires)
            for wire in wires:
                _emit(msp, wire, off, kerf / 2 if wire is outer else -kerf / 2)
            x0, y0, x1, y1 = _bbox_2d(sketch)
            rows.append((key, sheet_idx, name, round(x0 + off[0], 1), round(y0 + off[1], 1),
                         round(x1 - x0, 1), round(y1 - y0, 1)))
        path = Path(f"{prefix}_{sheet_idx}.dxf")
        doc.saveas(str(path))
        written.append(path)
    return written
