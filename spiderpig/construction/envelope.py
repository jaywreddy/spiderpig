"""Solids for claimed shapes at one crank angle.

Used two ways: :func:`claimed_solid` is the envelope a group promised to stay
inside (:mod:`construction.contract` checks parts against it), and
:func:`shape_solid` is a convenient way for a construction to build a part
that *is* its claim (a spacer, a web).
"""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np
from build123d import Edge, Face, Solid, Vector, Wire

from spiderpig.construction.base import Build
from spiderpig.shapes import disc, pill, union
from spiderpig.stack import Disc, Placed


def shape_solid(build: Build, p: Placed, grow: float = 0.0, z: tuple[float, float] | None = None):
    """The solid of one placed shape (its whole layer, or its clearance gap, unless ``z``
    narrows it)."""
    z0, z1 = z if z is not None else build.plan.slot_z(p)
    s = p.shape
    if isinstance(s, Disc):
        return disc(build.xy(s.at), s.r + grow, z0, z1)
    return pill(build.xy(s.a), build.xy(s.b), s.r + grow, z0, z1)


def _stadium(p, q, r: float, z0: float, z1: float):
    """The same solid as :func:`shapes.pill` (a stadium ``p``-``q`` of radius ``r``,
    extruded ``z0..z1``) built as one prism of two lines and two arcs, no boolean: the
    envelopes' shapes (never a part, so the parts' B-reps are untouched)."""
    p = np.asarray(p, dtype=float)[:2]
    q = np.asarray(q, dtype=float)[:2]
    d = q - p
    length = float(np.hypot(*d))
    if length <= 1e-9:
        return disc(p, r, z0, z1)
    u = d / length
    n = np.array([-u[1], u[0]])

    def v(xy):
        return Vector(float(xy[0]), float(xy[1]), z0)

    wire = Wire([Edge.make_line(v(p - r * n), v(q - r * n)),
                 Edge.make_three_point_arc(v(q - r * n), v(q + r * u), v(q + r * n)),
                 Edge.make_line(v(q + r * n), v(p + r * n)),
                 Edge.make_three_point_arc(v(p + r * n), v(p - r * u), v(p - r * n))])
    return Solid.extrude(Face(wire), Vector(0, 0, z1 - z0))


def claim_tools(build: Build, shapes: Iterable[Placed], grow: float = 0.0) -> list:
    """The given shapes' solids, each grown by ``grow``, not fused."""
    out = []
    for p in shapes:
        z0, z1 = build.plan.slot_z(p)
        s = p.shape
        out.append(disc(build.xy(s.at), s.r + grow, z0, z1) if isinstance(s, Disc)
                   else _stadium(build.xy(s.a), build.xy(s.b), s.r + grow, z0, z1))
    return out


def claimed_solid(build: Build, shapes: Iterable[Placed], grow: float = 0.0):
    """Union of the given shapes' solids, each grown by ``grow`` (a tolerance); a pill is
    one prism (:func:`_stadium`), not a fused disc, box and disc."""
    solids = claim_tools(build, shapes, grow)
    return union(solids) if solids else None
