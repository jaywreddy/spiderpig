"""Solids for claimed shapes at one crank angle.

Used two ways: :func:`claimed_solid` is the envelope a group promised to stay
inside (:mod:`construction.contract` checks parts against it), and
:func:`shape_solid` is a convenient way for a construction to build a part
that *is* its claim (a spacer, a web).
"""

from __future__ import annotations

from collections.abc import Iterable

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


def claimed_solid(build: Build, shapes: Iterable[Placed], grow: float = 0.0):
    """Union of the given shapes' solids, each grown by ``grow`` (a tolerance)."""
    solids = [shape_solid(build, p, grow) for p in shapes]
    return union(solids) if solids else None
