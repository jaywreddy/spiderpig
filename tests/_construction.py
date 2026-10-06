"""Helpers of the construction tier (``mise run test-construction``, PLAN P3): realize one
group of a side alone instead of fabricating the whole side, and a clash check that only
intersects pairs whose boxes meet.

``realize_groups(design, t, names)`` is the seam for a test that checks one construction:
those groups' parts at crank angle ``t`` exactly as :func:`spiderpig.fabricate.fabricate_side`
builds them (a non-cutting group reads nothing of the others; a plate group gets every
other group's cuts first, as there), in a fraction of the time. A test that needs the whole
side takes it from :mod:`tests.cache` instead.
"""

from __future__ import annotations

import itertools

from spiderpig.construction.base import Build, Realized
from spiderpig.fabricate import template_for


def realize_groups(design, t: float, names, mech=None) -> Realized:
    """The :class:`Realized` of just the groups ``names`` of ``design`` at crank angle
    ``t`` (``mech``: the side's template frozen at ``t``, when the caller has it). A plate
    group (``cuts``) is realized last, after every non-cutting group, with their cuts, as
    :func:`spiderpig.fabricate.fabricate_side` does; only the named groups' bodies, cuts and
    extras come back."""
    names = set(names)
    by_name = {g.name: g for g in design.groups}
    unknown = names - set(by_name)
    if unknown:
        raise ValueError(f"no such group: {sorted(unknown)} (the side's: {sorted(by_name)})")
    if mech is None:
        mech = template_for(design.config).freeze_at(t)
    build = Build(design.ctx, design.plan, mech)
    plates = [g for g in design.groups if g.cuts]
    needed = ([g for g in design.groups if not g.cuts] + plates
              if any(by_name[n].cuts for n in names)
              else [g for g in design.groups if g.name in names])
    done, out = Realized(), Realized()
    for g in needed:
        got = g.realize(build, done)
        done.merge(got)
        if g.name in names:
            out.merge(got)
    return out


def overlapping_pairs(parts: dict):
    """The pairs of ``parts`` (name -> placed part) whose bounding boxes overlap: any other
    pair's intersection has no volume, so a clash check need not intersect it."""
    boxes = {n: p.bounding_box() for n, p in parts.items()}
    for a, b in itertools.combinations(parts, 2):
        ba, bb = boxes[a], boxes[b]
        if all(max(getattr(ba.min, c), getattr(bb.min, c)) < min(getattr(ba.max, c),
                                                                  getattr(bb.max, c))
               for c in "XYZ"):
            yield a, b
