"""build123d part primitives for the walker.

Pure geometry: no mechanism wiring, no planning. Every function takes world
XY positions and a Z range and returns a build123d ``Part``; the
constructions (:mod:`construction`, :mod:`servos`) decide what goes where.
"""

from __future__ import annotations

import copy
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

import numpy as np
from build123d import (
    Align,
    Axis,
    Box,
    Compound,
    Cylinder,
    Edge,
    Face,
    Part,
    Pos,
    Solid,
    Vector,
    Wire,
)

XY = Sequence[float]


def share(shape):
    """A wrapper of its own round ``shape``'s B-rep: the same TShape (no copy), under a
    TopoDS_Shape of its own with its own location, so moving either in place (build123d's
    ``move`` / ``locate``) leaves the other where it was. Reads ``shape`` only (unlike
    :func:`moved`, it never touches its ``wrapped``); its label and colour are kept."""
    from build123d.topology import downcast
    from OCP.TopLoc import TopLoc_Location

    # its topology class (a Box's is Part: an object's own constructor takes dimensions)
    cls = next(c for c in type(shape).__mro__ if c.__module__.startswith("build123d.topology"))
    out = cls(downcast(shape.wrapped.Moved(TopLoc_Location())))
    out.label, out.color = shape.label, shape.color
    return out


def moved(shape, loc):
    """``shape.moved(loc)``, without the B-rep copy build123d makes and throws away.

    build123d's ``Shape.moved`` deep-copies the shape (``BRepBuilderAPI_Copy`` of every
    face) and then replaces the copy's B-rep with ``wrapped.Moved(loc)``, which shares the
    original's: the result is that, its Python attributes (label, colour, children)
    deep-copied as before. Same shape, same attributes; only the discarded copy is gone.
    """
    from build123d.topology import downcast

    w = shape.wrapped
    if w is None:
        return shape.moved(loc)          # build123d's own error
    shape.wrapped = None                 # deepcopy copies the B-rep only when set
    try:
        out = copy.deepcopy(shape)
    finally:
        shape.wrapped = w
    out.wrapped = downcast(w.Moved(loc.wrapped))
    return out


@dataclass(frozen=True)
class Cut:
    """A through-hole: diameter ``d``; ``flat`` > 0 makes it a D-hole whose flat
    is ``flat`` deep and faces direction ``angle`` (radians, world XY)."""

    xy: tuple[float, float]
    d: float
    flat: float = 0.0
    angle: float = 0.0


@dataclass(frozen=True)
class Rect:
    """A rectangular cut-out: ``size`` (along, across) centred on ``xy``, its
    long side turned to ``angle`` (radians, world XY)."""

    xy: tuple[float, float]
    size: tuple[float, float]
    angle: float = 0.0


def disc(xy: XY, radius: float, z0: float, z1: float) -> Part:
    """Solid cylinder at ``xy`` spanning ``z0..z1``."""
    return Cylinder(radius=radius, height=z1 - z0).move(
        Pos(float(xy[0]), float(xy[1]), (z0 + z1) / 2)
    )


def pill(p: XY, q: XY, radius: float, z0: float, z1: float) -> Part:
    """Stadium between two points (segment ⊕ disc), extruded ``z0..z1``: one prism of two
    lines and two arcs (no boolean)."""
    p = np.asarray(p, dtype=float)[:2]
    q = np.asarray(q, dtype=float)[:2]
    d = q - p
    length = float(np.hypot(*d))
    if length <= 1e-9:
        return disc(p, radius, z0, z1)
    u = d / length
    n = np.array([-u[1], u[0]])
    r = radius

    def v(xy):
        return Vector(float(xy[0]), float(xy[1]), z0)

    wire = Wire([Edge.make_line(v(p - r * n), v(q - r * n)),
                 Edge.make_three_point_arc(v(q - r * n), v(q + r * u), v(q + r * n)),
                 Edge.make_line(v(q + r * n), v(p + r * n)),
                 Edge.make_three_point_arc(v(p + r * n), v(p - r * u), v(p - r * n))])
    return Solid.extrude(Face(wire), Vector(0, 0, z1 - z0))


def union(parts: Iterable[Part]) -> Part:
    """Fuse parts in one boolean (``a + b`` on two disjoint Solids returns a list)."""
    parts = [p for p in parts if p is not None]
    out = parts[0].fuse(*parts[1:]) if len(parts) > 1 else parts[0]
    if isinstance(out, list):
        out = Compound(children=list(out))
    return _unwrap(out)


def _unwrap(shape):
    # A lone solid can come back wrapped in an untyped Compound, which STEP
    # export can't colour; unwrap it.
    solids = shape.solids()
    return solids[0] if len(solids) == 1 else shape


def _cutter(cut: Cut | Rect, z0: float, z1: float) -> Part:
    if isinstance(cut, Rect):
        return box(cut.xy, (cut.size[0], cut.size[1], z1 - z0), z0, cut.angle)
    body = disc(cut.xy, cut.d / 2, z0, z1)
    if cut.flat <= 0:
        return body
    # Keep the part of the circle behind the flat: remove a slab beyond it.
    r = cut.d / 2
    keep = r - cut.flat                         # distance from centre to the flat
    ux, uy = math.cos(cut.angle), math.sin(cut.angle)
    slab = Box(2 * r, 4 * r, z1 - z0 + 2).rotate(Axis.Z, math.degrees(cut.angle))
    cx = cut.xy[0] + ux * (keep + r)
    cy = cut.xy[1] + uy * (keep + r)
    return body - slab.move(Pos(cx, cy, (z0 + z1) / 2))


def cut_holes(part: Part, cuts: Iterable[Cut | Rect], z0: float, z1: float) -> Part:
    """Cut through-holes (round, D or rectangular) spanning ``z0..z1`` (with overshoot)."""
    cutters = [_cutter(c, z0 - 1.0, z1 + 1.0) for c in cuts]
    if not cutters:
        return part
    return _unwrap(part - union(cutters))


def plate(
    pills: Iterable[tuple[XY, XY, float]],
    z0: float,
    z1: float,
    cuts: Iterable[Cut | Rect] = (),
    discs: Iterable[tuple[XY, float]] = (),
) -> Part:
    """A laser-cut plate: union of pills ``(p, q, r)`` and discs ``(xy, r)``, minus cuts."""
    shapes = [pill(p, q, r, z0, z1) for p, q, r in pills]
    shapes += [disc(xy, r, z0, z1) for xy, r in discs]
    return cut_holes(union(shapes), cuts, z0, z1)


def box(center: XY, size: tuple[float, float, float], z0: float, angle: float = 0.0) -> Part:
    """A box standing on ``z0``, centred on ``center`` in XY, turned by ``angle`` (rad)."""
    b = Box(*size, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return b.rotate(Axis.Z, math.degrees(angle)).move(Pos(float(center[0]), float(center[1]), z0))


# The metal-shaft pivots' spacer rings and washers (construction/pivots).
def ring(xy: XY, od: float, id_: float, z0: float, z1: float) -> Part:
    """A spacer ring or washer: a disc with a bore."""
    return drill(disc(xy, od / 2, z0, z1), [(xy, id_ / 2)], z0, z1)


def drill(part: Part, holes: Iterable[tuple[XY, float]], z0: float, z1: float) -> Part:
    """Cut round holes given as ``(xy, radius)``."""
    return cut_holes(part, [Cut((float(xy[0]), float(xy[1])), 2 * r) for xy, r in holes], z0, z1)
