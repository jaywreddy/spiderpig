"""build123d part primitives for the Klann walker.

Pure geometry: no mechanism wiring, no planning. Every function takes world
XY positions and a Z range and returns a build123d ``Part``; :mod:`fabricate`
decides what goes where, :mod:`joinery` and :mod:`servos` model hardware.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

import numpy as np
from build123d import Align, Axis, Box, Compound, Cylinder, Part, Pos

THICKNESS = 3.0   # default laser-cut sheet = one stack slot
BUFF = 6.0        # link half-width (pill end radius)
HOLE_R = 2.0      # running-fit hole for a 3.8 mm printed pin
PIN_R = 1.9       # printed pin shaft; also the press-fit bore in printed caps
FLANGE_R = 4.0    # printed pin head / cap / sleeve

XY = Sequence[float]


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
    """Stadium between two points (segment ⊕ disc), extruded ``z0..z1``."""
    p = np.asarray(p, dtype=float)[:2]
    q = np.asarray(q, dtype=float)[:2]
    shape = disc(p, radius, z0, z1)
    length = float(np.hypot(*(q - p)))
    if length > 1e-9:
        theta = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))
        mid = (p + q) / 2
        bar = Box(length, 2 * radius, z1 - z0).rotate(Axis.Z, theta)
        shape = shape + bar.move(Pos(float(mid[0]), float(mid[1]), (z0 + z1) / 2)) + disc(
            q, radius, z0, z1
        )
    return shape


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


def drill(part: Part, holes: Iterable[tuple[XY, float]], z0: float, z1: float) -> Part:
    """Cut round holes given as ``(xy, radius)``."""
    return cut_holes(part, [Cut((float(xy[0]), float(xy[1])), 2 * r) for xy, r in holes], z0, z1)


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


def link_plate(
    segments: Iterable[tuple[XY, XY]],
    z0: float,
    z1: float,
    holes: Iterable[XY | Cut] = (),
    radius: float = BUFF,
) -> Part:
    """A laser-cut link: pills over ``segments``, cut at ``holes`` (a bare XY
    gets the printed-pin running fit)."""
    cuts = [h if isinstance(h, (Cut, Rect)) else Cut((float(h[0]), float(h[1])), 2 * HOLE_R)
            for h in holes]
    return plate([(p, q, radius) for p, q in segments], z0, z1, cuts)


def ring(xy: XY, od: float, id_: float, z0: float, z1: float) -> Part:
    """A spacer ring (laser-cut washer)."""
    return drill(disc(xy, od / 2, z0, z1), [(xy, id_ / 2)], z0, z1)


def box(center: XY, size: tuple[float, float, float], z0: float, angle: float = 0.0) -> Part:
    """A box standing on ``z0``, centred on ``center`` in XY, turned by ``angle`` (rad)."""
    b = Box(*size, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return b.rotate(Axis.Z, math.degrees(angle)).move(Pos(float(center[0]), float(center[1]), z0))


# -- printed pin kit (the no-purchase joinery option) --------------------------


def pin(xy: XY, head: tuple[float, float], top: float, *, shaft_r: float = PIN_R,
        head_r: float = FLANGE_R) -> Part:
    """Printed pin: a head flange over ``head`` (z0, z1) and a shaft up to ``top``."""
    return union([disc(xy, head_r, *head), disc(xy, shaft_r, head[0], top)])


def cap(xy: XY, z0: float, z1: float, *, bore_r: float = PIN_R, r: float = FLANGE_R) -> Part:
    """Press-on cap: a flange with a bore that grips the pin shaft."""
    return drill(disc(xy, r, z0, z1), [(xy, bore_r)], z0, z1)


def sleeve(xy: XY, z0: float, z1: float, *, bore_r: float = HOLE_R,
           r: float = FLANGE_R) -> Part:
    """Spacer between two links on one pin (running fit on the shaft)."""
    return drill(disc(xy, r, z0, z1), [(xy, bore_r)], z0, z1)
