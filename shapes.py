"""build123d part primitives for the Klann walker.

Pure geometry: no mechanism wiring, no planning. Every function takes world
XY positions and a Z range and returns a build123d ``Part``; :mod:`fabricate`
decides what goes where.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

import numpy as np
from build123d import Align, Axis, Box, Cylinder, Part, Pos

THICKNESS = 3.0   # laser-cut sheet = one stack slot
BUFF = 6.0        # link half-width (pill end radius)
HOLE_R = 2.0      # running-fit hole for a pin in a link
PIN_R = 1.9       # printed pin shaft; also the press-fit bore in caps and crank webs
FLANGE_R = 4.0    # pin head / cap / sleeve / frame post
JOURNAL_R = 4.0   # crank journal on the axis O
HUB_R = 3.0       # crank stub through the frame plate to the servo
KEY = 4.0         # square key above the plate that the servo horn grips

XY = Sequence[float]


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


def drill(part: Part, holes: Iterable[tuple[XY, float]], z0: float, z1: float) -> Part:
    """Cut vertical round holes ``(xy, radius)`` through ``z0..z1`` (with overshoot)."""
    cutters = [disc(xy, r, z0 - 1.0, z1 + 1.0) for xy, r in holes]
    if not cutters:
        return part
    cut = cutters[0]
    for c in cutters[1:]:
        cut = cut + c
    out = part - cut
    solids = out.solids()
    # A lone solid can come back wrapped in an untyped Compound, which STEP
    # export can't colour; unwrap it.
    return solids[0] if len(solids) == 1 else out


def link_plate(
    segments: Iterable[tuple[XY, XY]],
    z0: float,
    z1: float,
    holes: Iterable[XY] = (),
    radius: float = BUFF,
) -> Part:
    """A laser-cut link: union of pills over ``segments`` with pin holes."""
    segs = list(segments)
    shape = pill(*segs[0], radius, z0, z1)
    for p, q in segs[1:]:
        shape = shape + pill(p, q, radius, z0, z1)
    return drill(shape, [(h, HOLE_R) for h in holes], z0, z1)


def square_key(xy: XY, side: float, z0: float, z1: float) -> Part:
    return Box(side, side, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).move(
        Pos(float(xy[0]), float(xy[1]), z0)
    )


def pin(xy: XY, head: tuple[float, float], top: float) -> Part:
    """Printed pin: a head flange over ``head`` (z0, z1) and a shaft up to ``top``."""
    return disc(xy, FLANGE_R, *head) + disc(xy, PIN_R, head[0], top)


def cap(xy: XY, z0: float, z1: float) -> Part:
    """Press-on cap: a flange with a bore that grips the pin shaft."""
    return drill(disc(xy, FLANGE_R, z0, z1), [(xy, PIN_R)], z0, z1)


def sleeve(xy: XY, z0: float, z1: float) -> Part:
    """Spacer between two links on one pin (running fit on the shaft)."""
    return drill(disc(xy, FLANGE_R, z0, z1), [(xy, HOLE_R)], z0, z1)
