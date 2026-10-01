"""Solid models of a servo and its stock horn, in the servo frame (:mod:`servos.spec`).

:func:`servo_part` is the servo without its output horn: the manufacturer's
model when one can be had (:mod:`servos.cad`), else a parametric model built
from the spec. :func:`horn_part` is the stock output horn, drawn from the spec
(so the screws that bolt to it have real holes to go into), always separate
from the servo: the horn turns with the crank, the servo doesn't.

The parametric model: the main case between the two faces that carry the
mounting holes, recessed around the output axis (:class:`servos.spec.Recess`),
the raised panels and bumps (solid :class:`servos.spec.Relief`), the spline,
whatever stands on the axis through the horn (a raised centre ring), and the
idler boss (and idler horn, when it comes with the servo). It is a single
solid.
"""

from __future__ import annotations

import logging
import math
from functools import lru_cache

from build123d import Box, Compound, Cylinder, Location, Part

from spiderpig.hardware.fasteners import parse
from spiderpig.servos import cad as cadlib
from spiderpig.servos.spec import CadRef, HolePattern, ServoSpec

log = logging.getLogger("spiderpig.servos")

BBOX_TOL = 0.1          # mm: how closely a solid must match a CadRef.strip box
SPLINE_FIT = 0.1        # diametral clearance of the horn's splined bore in the model
UNKNOWN_HOLE_DEPTH = 4.0  # modelled depth of a mounting hole whose depth isn't published


def _cyl(r: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> Part:
    return Cylinder(r, z1 - z0).moved(Location((x, y, (z0 + z1) / 2)))


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Part:
    return Box(x1 - x0, y1 - y0, z1 - z0).moved(
        Location(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


def _one(shape):
    """A lone solid instead of a compound of one."""
    solids = shape.solids()
    return solids[0] if len(solids) == 1 else shape


def _fuse(parts):
    parts = [p for p in parts if p is not None]
    out = parts[0].fuse(*parts[1:]) if len(parts) > 1 else parts[0]
    if isinstance(out, list):
        out = Compound(children=list(out))
    return _one(out)


def cut_each(shape, tool):
    """``shape - tool``, one solid at a time.

    CAD assemblies are compounds of solids that may overlap each other; a
    single boolean on such a compound can leave material behind.
    """
    solids = shape.solids()
    if len(solids) == 1:
        return _one(solids[0] - tool)
    out = []
    for s in solids:
        rest = s - tool
        out += rest.solids() if rest is not None else []
    return Compound(children=out)


# -- horn -------------------------------------------------------------------------


def _pattern_holes(pat: HolePattern, z0: float, z1: float) -> list[Part]:
    """The pattern's holes, drawn at the thread's nominal size (a tapped hole)."""
    d = pat.thread_d or pat.hole_d
    r = pat.pcd / 2
    out = []
    for k in range(pat.count):
        a = math.radians(pat.angle_deg) + 2 * math.pi * k / pat.count
        out.append(_cyl(d / 2, z0, z1, r * math.cos(a), r * math.sin(a)))
    return out


def center_boss_on_servo(spec: ServoSpec) -> bool:
    """Is what stands on the axis above the horn part of the servo (it passes through the horn)?

    Otherwise it is the horn's centre screw head, drawn with the horn.
    """
    boss = spec.horn.center_boss
    return boss is not None and spec.horn.center_hole_d >= boss[0]


@lru_cache(maxsize=32)
def horn_part(spec: ServoSpec) -> Part:
    """The stock output horn (servo frame): flange, hub, screw holes, centre screw head."""
    h = spec.horn
    z1 = spec.horn_bottom
    flange = h.flange_thickness or h.thickness
    parts = [_cyl(h.diameter / 2, z1 - flange, z1)]
    if h.hub_d > 0:
        hub_bottom = h.hub_bottom if h.hub_bottom is not None else spec.seat_height
        parts.append(_cyl(h.hub_d / 2, hub_bottom, z1))
    else:
        parts.append(_cyl(h.diameter / 2, spec.seat_height, z1))
    if h.center_boss is not None and not center_boss_on_servo(spec):
        d, height = h.center_boss                 # the centre screw head
        parts.append(_cyl(d / 2, z1, z1 + height))
    horn = _fuse(parts)
    lo = min(spec.seat_height, h.hub_bottom if h.hub_bottom is not None else spec.seat_height)
    holes = _pattern_holes(h.pattern, lo - 1, z1 + 0.01)
    for extra in h.extra_holes:
        holes += _pattern_holes(extra, lo - 1, z1 + 0.01)
    if h.center_hole_d > 0:
        holes.append(_cyl(h.center_hole_d / 2, lo - 1, z1 + 0.01))
    if spec.spline_od > 0 and spec.spline_top > 0:
        holes.append(_cyl((spec.spline_od + SPLINE_FIT) / 2, lo - 1, spec.spline_top + 0.1))
    return _one(horn - _fuse(holes))


# -- parametric servo -------------------------------------------------------------


def _recess_cutter(spec: ServoSpec, rec, z0: float, z1: float) -> Part:
    L, W, _ = spec.body
    near = spec.axis_offset - L / 2
    parts = [_cyl(rec.r, z0, z1)]
    if rec.x_end > near:
        parts.append(_box(near - 1, rec.x_end, -W / 2 - 1, W / 2 + 1, z0, z1))
    return _fuse(parts)


@lru_cache(maxsize=32)
def parametric_servo(spec: ServoSpec) -> Part:
    """A servo from its spec alone (servo frame, one solid, no output horn)."""
    L, W, H = spec.body
    zf = spec.mount_face_z
    zr = spec.rear_z
    x0, x1 = spec.axis_offset - L / 2, spec.axis_offset + L / 2
    h = spec.horn
    body = _box(x0, x1, -W / 2, W / 2, zr, zf)
    cuts = []
    if spec.front_recess is not None:
        rec = spec.front_recess
        cuts.append(_recess_cutter(spec, rec, zf - rec.depth, zf + 1))
    if spec.rear_recess is not None:
        rec = spec.rear_recess
        cuts.append(_recess_cutter(spec, rec, zr - 1, zr + rec.depth))
    # the pocket the horn (and its hub) turns in
    cuts.append(_cyl(h.diameter / 2 + 0.25, spec.seat_height, zf + 10))
    if h.hub_bottom is not None and h.hub_d > 0:
        cuts.append(_cyl(h.hub_d / 2 + 0.1, h.hub_bottom, zf + 10))
    if cuts:
        body = body - _fuse(cuts)
    adds = [body]
    front_cut = (_cyl(spec.front_recess.r, zf - 1, zf + 10)
                 if spec.front_recess is not None else None)
    for r in spec.front_reliefs:
        if not r.solid:
            continue
        panel = _box(r.x0, r.x1, r.y0, r.y1, zf - 0.01, zf + r.height)
        panel = panel & _box(x0, x1, -W / 2, W / 2, zf - 1, zf + 10)
        if front_cut is not None:
            panel = panel - front_cut
        adds.append(panel)
    for r in spec.rear_reliefs:
        if r.solid:
            bump = _box(r.x0, r.x1, r.y0, r.y1, zr - r.height, zr + 0.01)
            adds.append(bump & _box(x0, x1, -W / 2, W / 2, zr - 10, zr + 1))
    if spec.spline_od > 0 and spec.spline_top > 0:
        base = zf - spec.front_recess.depth if spec.front_recess is not None else zf
        adds.append(_cyl(spec.spline_od / 2, base - 0.01, spec.spline_top))
    if center_boss_on_servo(spec):
        d, height = h.center_boss                 # e.g. a centre ring on the output
        adds.append(_cyl(d / 2, spec.seat_height - 0.5, spec.horn_bottom + height))
    idl = spec.idler
    if idl is not None and idl.included:
        base = idl.base_z if idl.base_z is not None else zr
        adds.append(_cyl(idl.boss_d / 2, base - idl.boss_h, base + 0.01))
        if idl.horn_d > 0 and idl.horn_thickness > 0 and idl.horn_face_z is not None:
            horn = _cyl(idl.horn_d / 2, idl.horn_face_z, idl.horn_face_z + idl.horn_thickness)
            adds.append(horn - _fuse(_pattern_holes(idl.pattern, idl.horn_face_z - 1,
                                                    idl.horn_face_z + idl.horn_thickness + 1)))
    return _fuse(adds)


# -- manufacturer's model ---------------------------------------------------------


def _matches(solid, box: tuple[float, ...]) -> bool:
    bb = solid.bounding_box()
    have = (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)
    return all(abs(a - b) <= BBOX_TOL for a, b in zip(have, box, strict=True))


def strip_indices(shape, ref: CadRef) -> list[int]:
    """Which of ``shape``'s solids (by index) survive ``ref.strip``: those whose bounding
    box matches none of its boxes. A detailed model's boxes take seconds, so
    :func:`cad_servo` records the answer beside the download."""
    return [i for i, s in enumerate(shape.solids())
            if not any(_matches(s, b) for b in ref.strip)]


def strip_horn(shape, ref: CadRef, keep: list[int] | None = None):
    """Drop the solids ``ref.strip`` names (``keep``: the indices :func:`strip_indices`
    found, else found now) and cut ``ref.strip_cut`` away."""
    solids = shape.solids()
    if keep is None:
        keep = strip_indices(shape, ref)
    if len(keep) != len(solids) - len(ref.strip):
        log.warning("%s: expected to strip %d solids, stripped %d", ref.filename,
                    len(ref.strip), len(solids) - len(keep))
    kept = [solids[i] for i in keep]
    out = kept[0] if len(kept) == 1 else Compound(children=kept)
    if ref.strip_cut is not None:
        r, z0, z1 = ref.strip_cut
        out = cut_each(out, _cyl(r, z0, z1))
    return _one(out)


def _strip_key(ref: CadRef) -> str:
    """What the strip's indices depend on besides the file: its placement and the boxes."""
    return repr((tuple(ref.transform), ref.scale, ref.strip, BBOX_TOL))


def _cached_indices(doc: dict | None, n: int) -> list[int] | None:
    """The recorded indices, when the record is of a model with ``n`` solids and sound."""
    if not doc or doc.get("solids") != n:
        return None
    keep = doc.get("keep")
    if not isinstance(keep, list) or not all(isinstance(i, int) and 0 <= i < n for i in keep):
        return None
    return keep


@lru_cache(maxsize=32)
def cad_servo(spec: ServoSpec):
    """The manufacturer's model without its output horn (servo frame), or ``None``.

    Which solids the strip drops is decided by a bounding box of every solid of the
    imported model (seconds for a detailed one); the answer is recorded beside the
    download (:func:`servos.cad.prepared_path`) and read back by every later process, so
    the shape is built from the same import exactly as the first time, without the boxes.
    """
    for ref in spec.cads:
        shape = cadlib.load(ref)
        if shape is None:
            continue
        n = len(shape.solids())
        path = cadlib.prepared_path(ref, _strip_key(ref))
        keep = _cached_indices(cadlib.read_prepared(path), n)
        try:
            if keep is None:
                keep = strip_indices(shape, ref)
                cadlib.write_prepared(path, {"solids": n, "keep": keep})
            return strip_horn(shape, ref, keep)
        except Exception as e:
            log.warning("%s: couldn't strip the horn from %s: %s", spec.key, ref.filename, e)
    return None


def drill_mounts(spec: ServoSpec, part):
    """The mounting holes, drawn at the screws' nominal size (a tapped or tapping hole)."""
    cutters = []
    for holes, face, sense in ((spec.mount, spec.mount_face_z, -1.0),
                               (spec.rear_mount, spec.rear_z, 1.0)):
        for mh in holes:
            screw = parse(mh.screw)
            d = screw[0].d if screw else mh.d
            depth = mh.depth if mh.depth is not None else UNKNOWN_HOLE_DEPTH
            z0, z1 = sorted((face - sense, face + sense * depth))
            cutters.append(_cyl(d / 2, z0, z1, mh.x, mh.y))
    return cut_each(part, _fuse(cutters)) if cutters else part


@lru_cache(maxsize=32)
def _servo_part(spec: ServoSpec, use_cad: bool):
    part = cad_servo(spec) if use_cad else None
    return drill_mounts(spec, part if part is not None else parametric_servo(spec))


def servo_part(spec: ServoSpec, *, cad: bool = True):
    """The servo without its output horn, in the servo frame, mounting holes drilled.

    The manufacturer's model when ``cad`` (and ``SPIDERPIG_SERVO_CAD`` isn't
    ``0``) and one can be had, else :func:`parametric_servo`.
    """
    return _servo_part(spec, bool(cad and cadlib.cad_enabled()))
