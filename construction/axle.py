"""Axle constructions: pillars (frame pivots) and pins (pivots between links).

A **pillar** is held by both frame plates: it runs from the outer plate
(layer 0) to the inner plate (layer ``top``) through every layer between,
and the links on it turn on it. A **pin** joins links only: it runs from
just below its lowest link to just above its highest and must be retained at
both ends.

Something must keep each link from sliding along its axle: a shoulder right
beside it (a built-in spacer). Between shoulders the axle necks down to its
thinnest, so other parts can pass close by; where even the neck can't pass,
a pillar stops short of that plate and is held by the other one (see
:meth:`AxleGroup.claims`).

:class:`PrintedAxle`: a printed stepped axle. The diameter changes along its
length: ``axle_d`` where a plate turns on it or is glued to it and in the
necks, up to ``spacer_d`` in the shoulders beside each link, a head outside
the outer plate (pillars) or below the lowest link and a cap above the
highest (pins). It is printed in segments that snap together, split at every link it
carries, so the links can be threaded on.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    Params,
    Realized,
    hardware,
)
from construction.envelope import claimed_solid
from shapes import Cut
from stack import Axis, Claim, Disc, Layout, Placed

STOP_OVERLAP = 0.8   # how far a spacer must overlap a link's hole to hold it (radial, mm)


@dataclass(frozen=True)
class AxleDims:
    axle: float      # radius plates turn on (or are glued to)
    spacer: float    # radius of the shoulder beside a link
    head: float      # radius of a head / cap
    neck: float      # thinnest the axle may get where another link passes


class AxleGroup:
    """One axle (``kind`` "frame" = pillar, "pin" = link pin), built by ``construction``."""

    def __init__(self, axis: Axis, construction):
        self.axis = axis
        self.construction = construction
        self.name = ("pillar:" if axis.kind == "frame" else "pin:") + axis.name

    @property
    def pillar(self) -> bool:
        return self.axis.kind == "frame"

    def dims(self, ctx: Context) -> AxleDims:
        return self.construction.dims(ctx, self.pillar)

    def claims(self, ctx: Context) -> list[Claim]:
        """One claim; it depends on the axle's links and on every link that passes close.

        Right beside each link (or run of links) the axle has a **shoulder**
        that keeps it in its layer: as wide as ``spacer`` where the links in
        that layer allow, and at least wide enough to overlap the link's
        hole. Everywhere else it crosses it is a **neck**, ``axle`` wide or
        thinner (down to ``neck``) where another link passes close. If even
        that can't pass a layer, the axle can't cross it. A pillar
        is anchored in both frame plates when it can reach them, else in the
        one it can reach, with a cap at its free end. A pin ends in a head
        below its lowest link and a cap above its highest.
        """
        d = self.dims(ctx)
        p = ctx.params
        geo = ctx.topo.geometry
        ax, g, members = self.axis.name, self.name, self.axis.members
        stop = p.hole(2 * d.axle) / 2 + STOP_OVERLAP     # a shoulder this wide holds a link
        room: dict[str, float] = {}                      # link -> radius free around the axle
        for n, segs in ctx.topo.links.items():
            if n in members:
                continue
            d_min = min(geo.dist(("pt", ax), ("seg", a, b)) for a, b in segs)
            free = d_min - p.link_radius - p.margin
            if free < max(d.spacer, d.head):
                room[n] = free

        def make(L: Layout):
            ms = sorted(L.layers[m] for m in members)
            mset = set(ms)
            free: dict[int, float] = {}
            for n, r in room.items():
                free[L.layers[n]] = min(free.get(L.layers[n], np.inf), r)

            def passes(k0: int, k1: int) -> bool:
                return all(free.get(k, np.inf) >= d.neck
                           for k in range(k0, k1 + 1) if k not in mset)

            lo, hi = ms[0], ms[-1]
            if not passes(lo, hi):
                return None
            if self.pillar:
                down, up = passes(1, lo - 1), passes(hi + 1, L.top - 1)
                if not (down or up):
                    return None
                k0 = 0 if down else lo - 1          # outer anchor, or a head below
                k1 = L.top if up else hi + 1        # inner anchor, or a cap above
            else:
                k0, k1 = lo - 1, hi + 1
            out = [Placed(k, Disc(ax, d.axle), g, f"{g} axle", seat=True) for k in ms]
            for k in (k0, k1):
                if self.pillar and k in (0, L.top):  # anchored in a frame plate
                    out.append(Placed(k, Disc(ax, d.axle), g, f"{g} anchor", seat=True))
                else:
                    end = "head" if k == k0 else "cap"
                    out.append(Placed(k, Disc(ax, d.head), g, f"{g} {end}"))
            if self.pillar and k0 == 0:
                out.append(Placed(-1, Disc(ax, d.head), g, f"{g} head"))
            beside = {k for m in ms for k in (m - 1, m + 1)} - mset - {k0, k1}
            for k in range(k0 + 1, k1):
                if k in mset:
                    continue
                if k in beside:
                    r = min(d.spacer, free.get(k, np.inf))
                    if r < stop:
                        return None                 # nothing could hold the link here
                    out.append(Placed(k, Disc(ax, r), g, f"{g} shoulder"))
                else:
                    r = min(d.axle, free.get(k, np.inf))   # necks down where a link passes
                    out.append(Placed(k, Disc(ax, r), g, f"{g} neck"))
            return out

        return [Claim(g, frozenset(members) | frozenset(room), make)]

    def realize(self, build: Build) -> Realized:
        return self.construction.realize(self, build)


@dataclass(frozen=True)
class PrintedAxle:
    """Printed stepped axle with built-in spacers."""

    key: str = "printed"
    label: str = "printed stepped axle (built-in spacers, snap-together segments)"

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        p: Params = ctx.params
        if p.hole(p.axle_d) / 2 + p.min_wall > p.link_radius:
            raise ConstructionError(
                f"a {p.axle_d} mm axle leaves less than {p.min_wall} mm of link around it "
                f"(link radius {p.link_radius})")
        if pillar and p.hole(p.axle_d, "glue") / 2 + p.min_wall > p.frame_radius:
            raise ConstructionError(f"a {p.axle_d} mm pillar doesn't fit the frame plate arms")
        if p.spacer_d <= p.hole(p.axle_d) or p.head_d <= p.hole(p.axle_d):
            raise ConstructionError("spacers and heads must be wider than the holes they retain")
        if not 0 < p.neck_d <= p.axle_d:
            raise ConstructionError("the neck can't be wider than the axle")
        return AxleDims(axle=p.axle_d / 2, spacer=p.spacer_d / 2, head=p.head_d / 2,
                        neck=p.neck_d / 2)

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        """First pass: one part that is exactly the claim (not yet split for assembly)."""
        out = Realized()
        ax = group.axis
        host = build.plan.topo.frame_bodies[0] if group.pillar else ax.members[0]
        solid = claimed_solid(build, build.shapes(group.name))
        out.bodies.append(hardware(group.name.replace(":", "_"), solid, host, fab="printed",
                                   color="#1baf7a"))
        p = build.ctx.params
        xy = tuple(build.xy(ax.name))
        for m in ax.members:
            out.cut(m, Cut(xy, p.hole(p.axle_d)))
        if group.pillar:
            for plate in (FRAME_INNER, FRAME_OUTER):
                out.cut(plate, Cut(xy, p.hole(p.axle_d, "glue")))
        return out


__all__ = ["AxleDims", "AxleGroup", "PrintedAxle"]
