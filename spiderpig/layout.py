"""2D sheet packing + DXF emission for every laser-cut part.

Given a fabricated :class:`mechanism.Mechanism` (one side or the whole
robot), we:

1. Select the laser-cut bodies (``fab == "laser"``): links, frame plates,
   centre plates, anything else a construction cuts from the sheet.
2. Slice each part through the middle of its own layer to get its 2D profile.
   Links are turned so their long axis runs along X, other plates along their
   principal axis; any part too long for the sheet is laid along the diagonal
   if that fits.
3. Compensate for the laser's kerf where the service doesn't: outlines grow and holes
   shrink by half the sheet's kerf (:func:`sheet_kerf`: the sheet item's ``kerf_mm``,
   0 at SendCutSend, which compensates for its kerf itself; Ponoko's 0.2 mm in acrylic,
   whose laser follows the line), so cut parts come out at their nominal size. A
   ``kerf`` given to :func:`save_sheets` overrides every sheet's.
4. Pack the profiles' bounding boxes onto fixed-size sheets with rectpack
   (a box may be turned 90°). A part that fits no sheet is an error, never a
   silent drop. Every physical part is packed (a left and a right plate of
   the same shape are two parts on the sheet).
5. Emit one DXF per sheet, a set per service and sheet stock
   (``<prefix>_<service>_<sheet>_<i>.dxf``: one order per service): round holes as
   CIRCLE, every other contour one closed LWPOLYLINE whose lines and arcs are exact (an
   arc is a vertex's bulge; every edge's endpoints are vertices); any other curve (none
   of the plates has one) is flattened to chords within :data:`CHORD_TOL` through its
   endpoints. All on layer ``CUT``, units = mm; and ``<prefix>_parts.csv`` saying which
   part is where (service, sheet, the kerf compensated).

:func:`save_parts` writes the other form a service may want: one DXF per different part
(``<dir>/<service>_<sheet>/<part>_x<qty>.dxf``, the part at the origin, the same contours
and kerf), and ``order.csv`` with each file's material, thickness and quantity: SendCutSend
quotes and nests per part and takes one part per file.

:func:`fidelity` reads a part's emitted contours back and measures them against the
solid's section (area and outline deviation); the cut-rule review
(:mod:`spiderpig.manufacture`, the audit's ``manufacture``) reports it per part.
"""

from __future__ import annotations

import csv
import math
import re
import threading
from pathlib import Path

import ezdxf
import numpy as np
from build123d import Axis, GeomType, Plane, section
from rectpack import newPacker

_CUT_LAYER = "CUT"
CUT_COLOR = 5        # blue (ACI 5): Ponoko's convention for a cut line, mapped at upload
_DEFAULT_SHEET = (200.0, 200.0)
_MARGIN = 3.0
CHORD_TOL = 0.02     # mm: a curve that is neither a line nor an arc, flattened within this
DEFAULT_KERF = 0.15  # mm: the kerf of a sheet whose item names none (no service's figure)


def laser_bodies(mech) -> list:
    return [b for b in mech.bodies if b.fab == "laser" and b.part is not None]


def sheet_kerf(key: str) -> float:
    """The kerf (mm) to compensate for on sheet ``key``: the sheet item's ``kerf_mm``
    (:mod:`hardware.sheet_catalog`: 0 at SendCutSend, which compensates for its kerf
    itself, so a compensated file would come back off size; Ponoko's 0.2 mm in acrylic),
    else :data:`DEFAULT_KERF`."""
    from spiderpig.hardware.catalog import get

    try:
        k = get(key).dims.get("kerf_mm")
    except KeyError:
        k = None
    return DEFAULT_KERF if k is None else float(k)


def sheet_service(key: str) -> str:
    """The service that cuts sheet ``key`` (the item's ``service``; ``""``: none named)."""
    from spiderpig.hardware.catalog import get

    try:
        return str(get(key).dims.get("service") or "")
    except KeyError:
        return ""


# ---------------------------------------------------------------------------
# Exact contours
# ---------------------------------------------------------------------------


def _ordered_edges(wire) -> list:
    """The wire's edges in order round it, each ``(adaptor, u_start, u_end)`` in the
    direction the wire runs (an edge reversed in the wire runs from its last parameter)."""
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.TopAbs import TopAbs_REVERSED

    out = []
    ex = BRepTools_WireExplorer(wire.wrapped)
    while ex.More():
        e = ex.Current()
        c = BRepAdaptor_Curve(e)
        a, b = c.FirstParameter(), c.LastParameter()
        out.append((c, b, a) if e.Orientation() == TopAbs_REVERSED else (c, a, b))
        ex.Next()
    return out


def _xy(c, u: float) -> tuple[float, float]:
    p = c.Value(u)
    return (p.X(), p.Y())


def _bulge(c, u0: float, u1: float) -> list[tuple[float, float, float]]:
    """An arc from ``u0`` to ``u1`` as LWPOLYLINE vertices ``(x, y, bulge)``: its start,
    bulge ``tan(sweep / 4)`` (positive counter-clockwise); split in two past a half turn."""
    sweep = abs(u1 - u0)
    if sweep > math.pi + 1e-9:
        um = (u0 + u1) / 2
        return _bulge(c, u0, um) + _bulge(c, um, u1)
    (x0, y0), (xm, ym), (x1, y1) = _xy(c, u0), _xy(c, (u0 + u1) / 2), _xy(c, u1)
    turn = (xm - x0) * (y1 - ym) - (ym - y0) * (x1 - xm)      # > 0: counter-clockwise
    return [(x0, y0, math.copysign(math.tan(sweep / 4), turn))]


def _params(c, u0: float, u1: float, tol: float) -> list[float]:
    """Parameters from ``u0`` to ``u1`` (both kept) whose chords stay within ``tol``."""
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection

    lo, hi = min(u0, u1), max(u0, u1)
    d = GCPnts_QuasiUniformDeflection(c, tol, lo, hi)
    us = [d.Parameter(i) for i in range(1, d.NbPoints() + 1)] if d.IsDone() else []
    us = sorted({lo, hi, *(u for u in us if lo < u < hi)})
    return us if u0 <= u1 else us[::-1]


def wire_vertices(wire, tol: float = CHORD_TOL) -> list[tuple[float, float, float]]:
    """A closed wire as LWPOLYLINE vertices ``(x, y, bulge)``: a line is the vertex at its
    start, an arc its start with a bulge (both exact), anything else chords within ``tol``
    through its endpoints. Every edge's start is a vertex, so no endpoint moves."""
    from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Line

    out: list[tuple[float, float, float]] = []
    for c, u0, u1 in _ordered_edges(wire):
        kind = c.GetType()
        if kind == GeomAbs_Line:
            out.append((*_xy(c, u0), 0.0))
        elif kind == GeomAbs_Circle:
            out += _bulge(c, u0, u1)
        else:
            out += [(*_xy(c, u), 0.0) for u in _params(c, u0, u1, tol)[:-1]]
    return out


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


def section_of(body):
    """The body's section through the middle of its own layer, laid flat."""
    part = _lay_flat(body.part)
    bb = part.bounding_box()
    return section(part, Plane.XY.offset((bb.min.Z + bb.max.Z) / 2))


def _profile(body, sheet: tuple[float, float], margin: float):
    """The body's mid-slot section, turned to lie flat (or along the sheet diagonal)."""
    sketch = section_of(body).rotate(Axis.Z, -_long_axis_degrees(body))
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
    """Emit a wire shifted by ``offset_xy``, offset by ``grow`` (kerf compensation): a
    CIRCLE for a round hole, else one closed LWPOLYLINE of exact lines and arcs."""
    ox, oy = offset_xy
    circle = _wire_is_circle(wire)
    if circle is not None:
        (cx, cy), r = circle
        return msp.add_circle((cx + ox, cy + oy), r + grow, dxfattribs={"layer": _CUT_LAYER})
    if abs(grow) > 1e-9:
        wire = wire.offset_2d(grow)     # kind "arc": an offset line or arc stays one
    return msp.add_lwpolyline(
        [(x + ox, y + oy, b) for x, y, b in wire_vertices(wire)], format="xyb",
        close=True, dxfattribs={"layer": _CUT_LAYER},
    )


# ---------------------------------------------------------------------------
# The DXF read back against the solid
# ---------------------------------------------------------------------------


def _ring_area(pts: np.ndarray) -> float:
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _loop_points(entity, tol: float) -> np.ndarray:
    """A closed LWPOLYLINE as drawn, flattened within ``tol``: each bulge read back as its
    true arc (ezdxf's own paths turn a bulge into Bézier curves, about 2.7e-4 x the radius
    off a quarter circle, which would hide a real error that small)."""
    verts = [tuple(v) for v in entity.get_points("xyb")]
    out: list[tuple[float, float]] = []
    for i, (x0, y0, b) in enumerate(verts):
        x1, y1, _ = verts[(i + 1) % len(verts)]
        out.append((x0, y0))
        if abs(b) < 1e-12:
            continue
        theta = 4.0 * math.atan(b)                       # signed sweep, + counter-clockwise
        chord = math.hypot(x1 - x0, y1 - y0)
        r = chord / (2.0 * abs(math.sin(theta / 2)))
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        h = r * math.cos(theta / 2)                      # centre's distance from the chord
        ux, uy = (x1 - x0) / chord, (y1 - y0) / chord
        cx, cy = mx - math.copysign(h, b) * uy, my + math.copysign(h, b) * ux
        a0 = math.atan2(y0 - cy, x0 - cx)
        step = 2.0 * math.acos(max(-1.0, 1.0 - tol / r)) if tol < r else math.pi / 2
        n = max(2, math.ceil(abs(theta) / step))
        out += [(cx + r * math.cos(a0 + theta * k / n), cy + r * math.sin(a0 + theta * k / n))
                for k in range(1, n)]
    return np.array(out)


def _wire_samples(wire, tol: float, step: float = 2.0) -> np.ndarray:
    """Points round the wire's exact edges: chords within ``tol``, no two ``step`` apart."""
    pts: list[tuple[float, float]] = []
    for c, u0, u1 in _ordered_edges(wire):
        us = _params(c, u0, u1, tol)
        for a, b in zip(us, us[1:], strict=False):
            n = max(1, math.ceil(c.Value(a).Distance(c.Value(b)) / step))
            pts += [_xy(c, a + (b - a) * i / n) for i in range(n)]
    return np.array(pts)


def _to_loops(p: np.ndarray, loops: list[np.ndarray]) -> np.ndarray:
    """Each point's distance to the nearest closed polyline of ``loops``."""
    best = np.full(len(p), np.inf)
    for q in loops:
        a = q
        ab = np.roll(q, -1, axis=0) - a
        l2 = np.maximum((ab ** 2).sum(1), 1e-18)
        for i in range(0, len(p), 256):
            pp = p[i:i + 256, None, :]
            t = np.clip(((pp - a) * ab).sum(2) / l2, 0.0, 1.0)
            d = np.sqrt((((a + t[..., None] * ab) - pp) ** 2).sum(2)).min(1)
            best[i:i + 256] = np.minimum(best[i:i + 256], d)
    return best


_SCRATCH = threading.local()


def _scratch():
    """An empty modelspace to emit into and read back (one document per thread)."""
    msp = getattr(_SCRATCH, "msp", None)
    if msp is None:
        msp = _SCRATCH.msp = ezdxf.new(dxfversion="R2010").modelspace()
    msp.delete_all_entities()
    return msp


def fidelity(sketch, tol: float = 2e-3) -> dict:
    """A part's section ``sketch`` emitted as :func:`save_sheets` emits it (no kerf), read
    back and measured against the solid: ``{"area_mm2", "dxf_area_mm2", "area_rel",
    "deviation_mm", "contours", "entities"}``. The areas are the section's (exact) and the
    emitted loops' (the outer less the rest, flattened within ``tol``); the deviation is
    the larger one-sided distance between the emitted contours and the section's edges
    (both sampled within ``tol``): about 0 for exact lines and arcs, up to
    :data:`CHORD_TOL` where a curve was flattened."""
    wires = list(sketch.wires())
    if not wires:
        return {"area_mm2": 0.0, "dxf_area_mm2": 0.0, "area_rel": 0.0, "deviation_mm": 0.0,
                "contours": 0, "entities": {}}
    msp = _scratch()
    outer = _outer(wires)
    ents = [_emit(msp, w, (0.0, 0.0), 0.0) for w in wires]
    areas, dev = [], 0.0
    for w, e in zip(wires, ents, strict=True):     # each contour against its own entity
        if e.dxftype() == "CIRCLE":                 # read back as drawn: centre and radius
            (cx, cy), r = _wire_is_circle(w)
            c, re_ = e.dxf.center, e.dxf.radius
            areas.append(math.pi * re_ ** 2)
            dev = max(dev, math.hypot(c.x - cx, c.y - cy) + abs(re_ - r))
            continue
        q = _loop_points(e, tol)
        areas.append(_ring_area(q))
        s = _wire_samples(w, tol)
        dev = max(dev, float(_to_loops(s, [q]).max()), float(_to_loops(q, [s]).max()))
    k = next(i for i, w in enumerate(wires) if w is outer)
    dxf_area = areas[k] - sum(a for i, a in enumerate(areas) if i != k)
    area = float(sum(f.area for f in sketch.faces()))
    kinds: dict[str, int] = {}
    for e in ents:
        kinds[e.dxftype()] = kinds.get(e.dxftype(), 0) + 1
    return {"area_mm2": round(area, 4), "dxf_area_mm2": round(dxf_area, 4),
            "area_rel": abs(dxf_area - area) / max(area, 1e-9),
            "deviation_mm": dev, "contours": len(wires), "entities": kinds}


# ---------------------------------------------------------------------------
# Packing and writing
# ---------------------------------------------------------------------------


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


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "", text) or "any"


def save_sheets(
    mech,
    prefix,
    sheet_size: tuple[float, float] | None = None,
    margin: float = _MARGIN,
    kerf: float | None = None,
    default: str | None = None,
) -> list[Path]:
    """Pack the mechanism's laser-cut parts onto sheets, one set per service and sheet
    (material and thickness: one order per service), and write DXFs.

    Returns the DXF paths written (``<prefix>_<service>_<sheet key>_<i>.dxf``, the
    service's name as letters and digits, ``any`` when the sheet names none); also writes
    ``<prefix>_parts.csv`` (service, sheet, kerf, part, position). Raises if any part
    can't be placed. ``kerf``: ``None`` compensates each sheet for its own service's kerf
    (:func:`sheet_kerf`), a number for that kerf on every sheet. ``default``: the sheet of
    a body that names none (the build's ``config.sheet``; ``mech.meta["sheet"]`` when not
    given).
    """
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    default = default or (mech.meta.get("sheet") if hasattr(mech, "meta") else None) \
        or "acrylic_3mm"
    written: list[Path] = []
    rows = []
    packed = pack_sheets(mech, default, sheet_size, margin)
    for key in sorted(packed, key=lambda k: (sheet_service(k), k != default, k)):
        service = sheet_service(key)
        k = sheet_kerf(key) if kerf is None else kerf
        written += _write_sheets(packed[key], Path(f"{prefix}_{_slug(service)}_{key}"), k,
                                 key, service, rows)
    if rows:
        with open(f"{prefix}_parts.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["service", "material", "kerf_mm", "sheet", "part", "x_mm", "y_mm",
                        "width_mm", "height_mm"])
            w.writerows(rows)
    return written


def _write_sheets(sheets, prefix: Path, kerf: float, key: str, service: str,
                  rows: list) -> list[Path]:
    written: list[Path] = []
    for sheet_idx, placed in enumerate(sheets):
        doc = ezdxf.new(dxfversion="R2010")
        doc.units = ezdxf.units.MM
        if _CUT_LAYER not in doc.layers:
            doc.layers.add(name=_CUT_LAYER, color=CUT_COLOR)
        msp = doc.modelspace()
        for name, sketch, off in placed:
            wires = list(sketch.wires())
            outer = _outer(wires)
            for wire in wires:
                _emit(msp, wire, off, kerf / 2 if wire is outer else -kerf / 2)
            x0, y0, x1, y1 = _bbox_2d(sketch)
            rows.append((service, key, kerf, sheet_idx, name, round(x0 + off[0], 1),
                         round(y0 + off[1], 1), round(x1 - x0, 1), round(y1 - y0, 1)))
        path = Path(f"{prefix}_{sheet_idx}.dxf")
        doc.saveas(str(path))
        written.append(path)
    return written


def _part_slug(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "-", name).strip("-") or "part"


def save_parts(groups, out_dir, default: str, kerf: float | None = None,
               margin: float = _MARGIN) -> list[dict]:
    """One DXF per different laser-cut part (``groups``: :func:`hardware.bom.group_made`'s
    laser groups; a part and its mirror image are one cut, flipped), for a service that
    quotes and nests per part: ``<out_dir>/<service>_<sheet>/<part>_x<qty>.dxf``, the part
    at the origin on layer ``CUT`` in mm, kerf-compensated as :func:`save_sheets` does
    (``kerf``: ``None`` for each sheet's service's). Writes ``<out_dir>/order.csv`` (a row
    per file: service, sheet, material, thickness, quantity, size, the parts it makes) and
    returns those rows."""
    from spiderpig.hardware.catalog import sheet_name, sheet_thickness
    from spiderpig.hardware.catalog import sheet_size as blank

    out_dir = Path(out_dir)
    rows: list[dict] = []
    for g in sorted(groups, key=lambda g: (sheet_service(sheet_key(g.ref, default)),
                                           sheet_key(g.ref, default), g.ref.name)):
        key = sheet_key(g.ref, default)
        service = sheet_service(key)
        k = sheet_kerf(key) if kerf is None else kerf
        sketch = _profile(g.ref, blank(key), margin)
        x0, y0, x1, y1 = _bbox_2d(sketch)
        doc = ezdxf.new(dxfversion="R2007")     # Ponoko's most compatible; SendCutSend's too
        doc.units = ezdxf.units.MM
        doc.layers.add(name=_CUT_LAYER, color=CUT_COLOR)
        msp = doc.modelspace()
        wires = list(sketch.wires())
        outer = _outer(wires)
        for wire in wires:
            _emit(msp, wire, (-x0, -y0), k / 2 if wire is outer else -k / 2)
        folder = out_dir / f"{_slug(service)}_{key}"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{_part_slug(g.ref.name)}_x{g.qty}.dxf"
        doc.saveas(str(path))
        rows.append({"service": service or "any", "sheet": key, "material": sheet_name(key),
                     "thickness_mm": round(sheet_thickness(key), 3), "kerf_mm": k,
                     "file": str(path.relative_to(out_dir)), "qty": g.qty,
                     "size_mm": f"{x1 - x0:.1f} x {y1 - y0:.1f}",
                     "parts": " ".join(g.names)})
    if rows:
        out_dir.mkdir(parents=True, exist_ok=True)
        with open(out_dir / "order.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    return rows
