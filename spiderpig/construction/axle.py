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
highest (pins). It is printed in segments that snap together, split above
every run of links it carries, so the links can be threaded on
(:mod:`construction.printed`).

Assembly, bottom up: glue each pillar's first segment into the outer plate
(head underneath); thread the links on; snap the next segment of every
pillar and pin on; repeat per deck; glue the inner plate over the pillars'
top ends, which finish flush with its top face.

The constructions on purchased metal shafts (``rod``, ``bolt``, ``bearing``,
``bushing``) live in :mod:`construction.pivots`; they state how they differ
through the extra fields of :class:`AxleDims` and an optional ``ends``
method (see :meth:`AxleGroup.claims`).
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass

import numpy as np

from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    Group,
    Params,
    Realized,
    hardware,
)
from spiderpig.construction.printed import Segment, Snap, plan_segments, segment_solid
from spiderpig.hardware.bom import BomLine
from spiderpig.shapes import Cut
from spiderpig.stack import Axis, Claim, Disc, Keepout, Layout, Placed, Unbuildable

STOP_OVERLAP = 0.8   # how far a spacer must overlap a link's hole to hold it (radial, mm)


@dataclass(frozen=True)
class AxleDims:
    axle: float      # radius plates turn on (or are glued to)
    spacer: float    # radius of the shoulder beside a link
    head: float      # radius of a head / cap
    neck: float      # thinnest the axle may get where another link passes
    # How a construction departs from a printed stepped axle (the defaults: it doesn't).
    fill: bool = False      # a loose spacer fills every layer between the ends (a rod can't
    #                         neck down, so a passing link must leave room for the spacer)
    flange: float = 0.0     # radius of a flange each link carries on one face (a bearing's);
    #                         it needs a free layer beside the link at least this wide
    seat: float | None = None   # radius seated in a link's hole when not ``axle`` (a bearing)


End = tuple[str, float]     # (label, radius) of one layer claimed beyond an axle's end


def default_ends(d: AxleDims, pillar: bool, anchored: tuple[bool, bool]) -> tuple[
        tuple[End, ...], tuple[End, ...]]:
    """What a printed axle claims beyond its retained stack: a head below (under the outer
    plate, or under its lowest link), a cap above unless the inner plate holds it."""
    below: tuple[End, ...] = (("head", d.head),)
    above: tuple[End, ...] = () if pillar and anchored[1] else (("cap", d.head),)
    return below, above


def flange_sides(layers: list[int], room: Callable[[int], float], flange: float,
                 names: Mapping[int, str] | None = None) -> dict[int, int]:
    """Which face of each link (by layer) its flange goes on: ``+1`` up, ``-1`` down.

    ``room(k)`` is the radius free for a flange in layer ``k`` (0 where a link,
    a frame plate or a narrow neighbour leaves none). Two links in adjacent
    layers turn their flanges outwards; a lone link puts its flange on the
    roomier side (down on a tie). Raises :class:`Unbuildable` when a flange
    has nowhere to go.
    """
    names = names or {}
    out: dict[int, int] = {}
    runs: list[list[int]] = []
    for k in sorted(layers):
        if runs and runs[-1][-1] == k - 1:
            runs[-1].append(k)
        else:
            runs.append([k])
    for run in runs:
        if len(run) > 2:
            raise Unbuildable(f"{len(run)} of its links sit in adjacent layers {run[0]}..{run[-1]}:"
                              f" the flange of {names.get(run[1], 'the middle one')} has no room")
        if len(run) == 2:
            wants = ((run[0], -1), (run[1], +1))
        else:
            k = run[0]
            wants = ((k, -1 if room(k - 1) >= room(k + 1) else +1),)
        for k, side in wants:
            if room(k + side) < flange - 1e-9:
                raise Unbuildable(f"no room for the {2 * flange:g} mm flange of "
                                  f"{names.get(k, f'the link in layer {k}')} beside layer {k}: "
                                  f"{room(k + side):.1f} mm free")
            out[k] = side
    return out


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
        hole. Everywhere else it crosses it is a **neck**, ``axle`` wide or
        thinner (down to ``neck``) where another link passes close (or, for
        a construction that ``fill``\\ s with loose spacers, a **spacer** as
        wide as the shoulder). If even that can't pass a layer, the axle
        can't cross it. A pillar is anchored in both frame plates when it can
        reach them, else in the one it can reach, with a cap at its free end.
        A pin ends in a head below its lowest link and a cap above its
        highest. A construction with an ``ends`` method claims its own
        retainers beyond each end instead (:func:`default_ends` is the
        printed axle's); it may raise :class:`Unbuildable` for a stack it
        can't span.
        """
        d = self.dims(ctx)
        p = ctx.params
        geo = ctx.topo.geometry
        ax, g, members = self.axis.name, self.name, self.axis.members
        seat = d.axle if d.seat is None else d.seat
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

        def make(L: Layout):
            ms = sorted(L.layers[m] for m in members)
            mset = set(ms)
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
            k0 = 0 if down else lo              # the retained stack: outer anchor or lowest link
            k1 = L.top if up else hi            # ... to inner anchor or highest link
            out = [Placed(k, Disc(ax, seat), g, f"{g} axle", seat=True) for k in ms]
            for k, anchored in ((0, down), (L.top, up)):
                if anchored:
                    out.append(Placed(k, Disc(ax, d.axle), g, f"{g} anchor", seat=True))
            if ends is None:
                below, above = default_ends(d, self.pillar, (down, up))
            else:
                below, above = ends(d, self.pillar, (down, up), k1 - k0 + 1, L.pitch)
            out += [Placed(k0 - 1 - i, Disc(ax, r), g, f"{g} {label}")
                    for i, (label, r) in enumerate(below)]
            out += [Placed(k1 + 1 + i, Disc(ax, r), g, f"{g} {label}")
                    for i, (label, r) in enumerate(above)]
            beside = {k for m in ms for k in (m - 1, m + 1)} - mset
            claimed: dict[int, float] = {}
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
                elif d.fill:
                    r = min(d.spacer, free.get(k, np.inf))   # a loose spacer, as wide as it can
                    out.append(Placed(k, Disc(ax, r), g, f"{g} spacer"))
                else:
                    r = min(d.axle, free.get(k, np.inf))   # necks down where a link passes
                    out.append(Placed(k, Disc(ax, r), g, f"{g} neck"))
                claimed[k] = r
            if d.flange > 0:
                for i, (_, r) in enumerate(below):
                    claimed[k0 - 1 - i] = r
                for i, (_, r) in enumerate(above):
                    claimed[k1 + 1 + i] = r
                flange_sides(ms, lambda k: claimed.get(k, 0.0), d.flange,
                             {L.layers[m]: m for m in members})
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
            for k in range(lo - 1, hi + 2):
                if k in mset or (self.pillar and k in (0, L.top)):
                    continue
                if not self.pillar and k in (lo - 1, hi + 1):
                    out.append(Placed(k, Disc(ax, d.head), g, f"{g} {'head' if k < lo else 'cap'}"))
                elif k in beside:
                    out.append(Placed(k, Disc(ax, stop), g, f"{g} shoulder"))
                else:
                    out.append(Placed(k, Disc(ax, d.neck), g, f"{g} neck"))
            return out

        return [Claim(g, frozenset(members) | frozenset(room), make, early=early,
                      early_deps=frozenset(members))]

    def realize(self, build: Build, done: Realized) -> Realized:
        return self.construction.realize(self, build)


@dataclass(frozen=True)
class PrintedAxle:
    """Printed stepped axle with built-in spacers, in segments that snap together.

    The profile, the segments and the snap joint are described in
    :mod:`construction.printed`. Pillars are glued into the frame plates they
    reach (``ca_glue``); every other joint snaps. The knobs are the fields
    below (mm unless noted); the fits come from :class:`Params`.
    """

    key: str = "printed"
    label: str = "printed stepped axle (built-in spacers, snap-together segments)"
    axial_play: float = 0.1      # gap between a link and each shoulder beside it
    snap_engage: float = 0.25    # radial overlap of the barb over the socket's ledge
    snap_wall: float = 0.8       # thinnest wall around a socket (in a shoulder or cap)
    snap_shank: float = 1.0      # peg shank height (the ledge is this less the clearance)
    snap_land: float = 0.4       # height of the barb's cylindrical land
    snap_flats: float = 3.0      # width across the flats trimmed on the barb
    slot_width: float = 1.2      # slot splitting the peg (and the bearing below it)
    slot_max: float = 12.0       # deepest the slot runs, measured down from the peg's tip
    max_strain: float = 0.04     # peak prong strain while snapping (PETG); more is logged
    bridge: float = 0.8          # least solid between a socket and the slot above it
    base: float = 2.0            # least solid under a slot's root at a segment's bottom
    min_prong: float = 0.8       # thinnest a prong of the peg's shank may be
    glue_per_anchor: float = 0.02  # CA glue per anchor, as a fraction of a bottle

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
        self.snap(ctx)          # the snap joint must fit too
        return AxleDims(axle=p.axle_d / 2, spacer=p.spacer_d / 2, head=p.head_d / 2,
                        neck=p.neck_d / 2)

    def snap(self, ctx: Context) -> Snap:
        """The snap joint between segments, sized for the narrowest socket a plan can ask for.

        A socket sits in a shoulder (at least as wide as a link's hole plus
        ``STOP_OVERLAP``) or a cap (``head_d``). The barb is as wide as that
        leaves room for, but no wider than the bearing.
        """
        p: Params = ctx.params
        c = p.print_fit / 2
        stop = p.hole(p.axle_d) / 2 + STOP_OVERLAP
        room = min(stop, p.spacer_d / 2, p.head_d / 2)
        barb = min(p.axle_d / 2, room - self.snap_wall - c)
        snap = Snap(barb=barb, engage=self.snap_engage, clearance=c,
                    shank_h=self.snap_shank, land_h=self.snap_land,
                    flats=self.snap_flats / 2, slot=self.slot_width)
        if snap.shank - snap.slot / 2 < self.min_prong:
            raise ConstructionError(
                f"no room for a snap peg: a {2 * snap.shank:.2f} mm shank split by a "
                f"{snap.slot} mm slot (wider shoulders or heads, or a narrower slot)")
        if snap.deflection() > snap.slot / 2 - 0.1:
            raise ConstructionError(
                f"the snap peg's prongs would have to close by {snap.deflection():.2f} mm "
                f"each; the {snap.slot} mm slot lets them close {snap.slot / 2 - 0.1:.2f}")
        if self.snap_shank <= c:
            raise ConstructionError("the snap peg's shank must be taller than the clearance")
        room_z = ctx.pitch - 2 * self.axial_play
        if snap.depth + self.bridge > room_z + 1e-9:
            raise ConstructionError(
                f"a {snap.depth:.2f} mm snap socket and a {self.bridge} mm bridge don't fit "
                f"a {room_z:.2f} mm shoulder ({ctx.pitch} mm sheet)")
        return snap

    def segments(self, group: AxleGroup, build: Build) -> list[Segment]:
        """The axle's printed segments for the solved plan (see :func:`plan_segments`)."""
        column = {s.layer: (s.label.rsplit(" ", 1)[1], s.shape.r)
                  for s in build.shapes(group.name)}
        links: dict[int, tuple[str, ...]] = {}
        for m in group.axis.members:
            links[build.layers[m]] = links.get(build.layers[m], ()) + (m,)
        return plan_segments(
            column, build.z, links, axle=group.dims(build.ctx).axle, snap=self.snap(build.ctx),
            play=self.axial_play, bridge=self.bridge, base=self.base, slot_max=self.slot_max,
            min_prong=self.min_prong, max_strain=self.max_strain, name=group.name)

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        """One printed body per segment; holes in its links and in the plates it's glued into."""
        out = Realized()
        ax = group.axis
        p = build.ctx.params
        if group.pillar:
            host = build.plan.topo.frame_bodies[0]
        else:
            host = min(ax.members, key=lambda m: (build.layers[m], m))
        snap = self.snap(build.ctx)
        xy = build.xy(ax.name)
        stem = group.name.replace(":", "_")
        anchors: list[int] = []
        for seg in self.segments(group, build):
            part = segment_solid(seg, snap, xy)
            out.bodies.append(hardware(f"{stem}_seg{seg.index}", part, host, fab="printed",
                                       color="#1baf7a"))
            anchors += seg.anchors
        xy = (float(xy[0]), float(xy[1]))
        for m in ax.members:
            out.cut(m, Cut(xy, p.hole(p.axle_d)))
        plates = {0: FRAME_OUTER, build.top: FRAME_INNER}
        for k in anchors:
            out.cut(plates[k], Cut(xy, p.hole(p.axle_d, "glue")))
        if anchors:
            out.extras.append(BomLine("ca_glue", self.glue_per_anchor * len(anchors),
                                      f"{group.name} anchors"))
        return out
