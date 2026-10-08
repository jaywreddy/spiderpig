"""Axle constructions: pillars (frame pivots) and pins (pivots between links).

A **pillar** is held by both frame plates: it runs from the outer plate
(layer 0) to the inner plate (layer ``top``) through every layer between,
and the links on it turn on it. A **pin** joins links only: it runs from
just below its lowest link to just above its highest and must be retained at
both ends.

Something must keep each link from sliding along its axle: a shoulder right
beside it, and a loose spacer (a printed ring) in every other layer between the
ends (a metal shaft can't neck down); where even a ring can't pass, a pillar stops
short of that plate and is held by the other one (see :meth:`AxleGroup.claims`).

The constructions (the ``standoff`` pillar, the ``chicago`` pin) live in
:mod:`construction.pivots`; they state their radii through :class:`AxleDims` and their
retainers through an optional ``ends`` method (else :func:`default_ends`) and
``end_heights``. (The printed stepped axle, its snap segments and the flanged pivots'
rules were removed on 2026-10-07: :data:`config.REMOVED_CONSTRUCTIONS`.)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from spiderpig.construction.base import Build, Context, Group, Realized
from spiderpig.stack import Axis, Claim, Disc, Keepout, Layout, Placed, Unbuildable

STOP_OVERLAP = 0.8   # how far a spacer must overlap a link's hole to hold it (radial, mm)


@dataclass(frozen=True)
class AxleDims:
    axle: float      # radius plates turn on (or are glued to)
    spacer: float    # radius of the shoulder beside a link
    head: float      # radius of a head / cap
    neck: float      # the narrowest ring: what a passing link must leave the axle
    #                  (the least it is in the search's early claims, before what passes it
    #                  is known)
    # The height each end's retainer (a head, a nut, a clip, with its washers and a
    # clearance) needs beyond the retained stack, below and above: one that needs any
    # (> 0) sits in the thin clearance gap beside its link (:attr:`stack.Placed.gap`), or
    # the layer beyond when nothing there is in its way; 0: a full layer, as a printed cap.
    # Only a construction with a single retainer per end and no flange sets it.
    end_h: tuple[float, float] = (0.0, 0.0)
    # the radius of the washers the axle carries through a clearance gap it crosses
    # (0: ``spacer``)
    washer: float = 0.0


End = tuple[str, float]     # (label, radius) of one layer claimed beyond an axle's end


def default_ends(d: AxleDims, pillar: bool, anchored: tuple[bool, bool]) -> tuple[
        tuple[End, ...], tuple[End, ...]]:
    """What an axle without an ``ends`` hook claims beyond its retained stack: a head below
    (under the outer plate, or under its lowest link), a cap above unless the inner plate
    holds it (the Chicago pin: the barrel's head and the screw's)."""
    below: tuple[End, ...] = (("head", d.head),)
    above: tuple[End, ...] = () if pillar and anchored[1] else (("cap", d.head),)
    return below, above


class AxleGroup(Group):
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

    def keepouts(self, ctx: Context) -> list[Keepout]:
        """At its thinnest (the neck) the axle still fills every layer it spans."""
        return [Keepout(self.name, ("pt", self.axis.name), self.dims(ctx).neck,
                        f"{self.name} spans", frozenset(self.axis.members), span=True,
                        anchored=self.pillar)]

    def claims(self, ctx: Context) -> list[Claim]:
        """One claim; it depends on the axle's links and on every link that passes close.

        Right beside each link (or run of links) the axle has a **shoulder**
        that keeps it in its layer: as wide as ``spacer`` where the links in
        that layer allow, and at least wide enough to overlap the link's
        hole. Everywhere else it crosses it is a loose **spacer** as wide as
        the shoulder where the links allow, at least ``neck``. If even that
        can't pass a layer, the axle can't cross it. A pillar is anchored in
        both frame plates when it can reach them, else in the one it can reach,
        with a cap at its free end.
        A pin ends in a head below its lowest link and a cap above its
        highest. A construction with a ``column`` method may refuse a column
        (its links' layers, the stack, which plates it reaches) with
        :class:`Unbuildable` (the standoff pillar: no stock standoff fits;
        the Chicago pin: no stock barrel). A construction with an ``ends``
        method claims its own retainers beyond each end instead of
        :func:`default_ends`.
        """
        d = self.dims(ctx)
        p = ctx.params
        geo = ctx.topo.geometry
        ax, g, members = self.axis.name, self.name, self.axis.members
        seat = d.axle
        stop = p.hole(2 * seat) / 2 + STOP_OVERLAP       # a shoulder this wide holds a link
        room: dict[str, float] = {}                      # link -> radius free around the axle
        for n, segs in ctx.topo.links.items():
            if n in members:
                continue
            d_min = min(geo.dist(("pt", ax), ("seg", a, b)) for a, b in segs)
            free = d_min - p.link_radius - p.margin
            if free < max(d.spacer, d.head):
                room[n] = free
        ends = getattr(self.construction, "ends", None)
        link_t = {m: ctx.sheet_t("link", m) for m in members}

        def make(L: Layout):
            ms = sorted(L.layers[m] for m in members)
            mset = set(ms)
            own: dict[int, float] = {}
            for m in members:
                own[L.layers[m]] = max(own.get(L.layers[m], 0.0), link_t[m])

            def air(a: int, b: int) -> float:
                """What the plan's z leaves free in layers ``a``..``b`` around this axle's
                own parts (its links and its default-sheet rings, in layers an aluminium
                plate elsewhere made thicker): its stack closes it up."""
                if not L.final:
                    return 0.0
                return sum(max(0.0, L.t(k) - own.get(k, ctx.pitch))
                           for k in range(a, b + 1) if 0 < k < L.top)
            free: dict[int, float] = {}
            who: dict[int, str] = {}
            for n, r in room.items():
                if r < free.get(L.layers[n], np.inf):
                    free[L.layers[n]], who[L.layers[n]] = r, n

            def blocker(k0: int, k1: int) -> int | None:
                """The first layer in k0..k1 the axle can't neck through."""
                return next((k for k in range(k0, k1 + 1)
                             if k not in mset and free.get(k, np.inf) < d.neck), None)

            def crossing(k: int) -> str:
                gap = free[k] + p.link_radius + p.margin
                if gap <= 0:
                    return f"{who[k]} in layer {k} sweeps right across it"
                return (f"{who[k]} in layer {k} passes {gap:.1f} mm from its centre, leaving "
                        f"{free[k]:.1f} mm, under its {d.neck:g} mm thinnest neck radius")

            lo, hi = ms[0], ms[-1]
            if (k := blocker(lo, hi)) is not None:
                raise Unbuildable(f"can't run between its links (layers {lo}..{hi}): "
                                  + crossing(k))
            down = up = False
            if self.pillar:
                kd, ku = blocker(1, lo - 1), blocker(hi + 1, L.top - 1)
                down, up = kd is None, ku is None
                if not (down or up):
                    raise Unbuildable("can't reach either frame plate: " + crossing(kd)
                                      + " below its links and " + crossing(ku) + " above")
            column = getattr(self.construction, "column", None)
            if column is not None:          # a construction's own rule over the whole column
                column(self.pillar, ms, L.top, (down, up), L.pitch, layout=L, air=air)
            k0 = 0 if down else lo              # the retained stack: outer anchor or lowest link
            k1 = L.top if up else hi            # ... to inner anchor or highest link
            out = [Placed(k, Disc(ax, seat), g, f"{g} axle", seat=True) for k in ms]
            for k, anchored in ((0, down), (L.top, up)):
                if anchored:
                    out.append(Placed(k, Disc(ax, d.axle), g, f"{g} anchor", seat=True))
            if ends is None:
                below, above = default_ends(d, self.pillar, (down, up))
            else:
                below, above = ends(d, self.pillar, (down, up), k1 - k0 + 1, L.pitch,
                                    span=L.z(k1)[1] - L.z(k0)[0] if L.final else None)
            h_lo, h_hi = d.end_h
            heights = getattr(self.construction, "end_heights", None)
            if heights is not None:                 # what the retainers need at this z
                h_lo, h_hi = heights(d, L, k0, k1, air=air(k0, k1))
            w0, w1 = k0, k1         # the clearance gaps its column crosses: range(w0, w1)
            for i, (label, r) in enumerate(below):
                if i == 0 and h_lo > 0 and k0 >= 1:     # in the clearance gap under k0
                    out.append(Placed(k0 - 1, Disc(ax, r), g, f"{g} {label}", gap=True,
                                      height=h_lo, toward=-1))
                else:
                    out.append(Placed(k0 - 1 - i, Disc(ax, r), g, f"{g} {label}"))
                    if i == 0 and k0 >= 1:  # an end retainer in the layer beyond its stack
                        w0 = k0 - 1
            for i, (label, r) in enumerate(above):
                if i == 0 and h_hi > 0 and k1 <= L.top - 1:   # in the gap over k1
                    out.append(Placed(k1, Disc(ax, r), g, f"{g} {label}", gap=True,
                                      height=h_hi, toward=+1))
                else:
                    out.append(Placed(k1 + 1 + i, Disc(ax, r), g, f"{g} {label}"))
                    if i == 0 and k1 <= L.top - 1:
                        w1 = k1 + 1
            # the washers it carries through every clearance gap of its column (only a gap
            # the plan has is built; the planner keeps other groups' heads off them),
            # including, at any end whose first retainer takes the layer beyond the retained
            # stack rather than its clearance gap (today only a standoff pillar's free end,
            # a cantilever's), the gap between that end layer and the retainer: filled, so
            # the end's link can't slide
            wr = d.washer or d.spacer
            out += [Placed(k, Disc(ax, wr), g, f"{g} washer", gap=True) for k in range(w0, w1)]
            beside = {k for m in ms for k in (m - 1, m + 1)} - mset
            for k in range(k0 + 1, k1):
                if k in mset:
                    continue
                if k in beside:
                    r = min(d.spacer, free.get(k, np.inf))
                    if r < stop:                    # nothing could hold the link here
                        raise Unbuildable(f"no room for a shoulder in layer {k} beside its link: "
                                          f"{who[k]} leaves {r:.1f} mm, a shoulder needs "
                                          f"{stop:.1f}")
                    out.append(Placed(k, Disc(ax, r), g, f"{g} shoulder"))
                else:
                    r = min(d.spacer, free.get(k, np.inf))   # a loose spacer, as wide as it can
                    out.append(Placed(k, Disc(ax, r), g, f"{g} spacer"))
            return out

        def early(L: Layout):
            """The least the axle is once its own links have layers, whatever passes it: a
            pin's head and cap, shoulders at their narrowest beside its links, necks at
            their thinnest between them (a pillar's ends may still become anchors)."""
            ms = sorted(L.layers[m] for m in members)
            mset = set(ms)
            lo, hi = ms[0], ms[-1]
            beside = {k for m in ms for k in (m - 1, m + 1)}
            out = [Placed(k, Disc(ax, d.axle), g, f"{g} axle", seat=True) for k in ms]
            h_lo, h_hi = d.end_h
            for k in range(lo - 1, hi + 2):
                if k in mset or (self.pillar and k in (0, L.top)):
                    continue
                if not self.pillar and k == lo - 1 and h_lo > 0:
                    out.append(Placed(k, Disc(ax, d.head), g, f"{g} head", gap=True,
                                      height=h_lo, toward=-1))
                elif not self.pillar and k == hi + 1 and h_hi > 0:
                    out.append(Placed(hi, Disc(ax, d.head), g, f"{g} cap", gap=True,
                                      height=h_hi, toward=+1))
                elif k in beside:
                    out.append(Placed(k, Disc(ax, stop), g, f"{g} shoulder"))
                else:
                    out.append(Placed(k, Disc(ax, d.neck), g, f"{g} neck"))
            return out

        return [Claim(g, frozenset(members) | frozenset(room), make, early=early,
                      early_deps=frozenset(members))]

    def realize(self, build: Build, done: Realized) -> Realized:
        return self.construction.realize(self, build)
