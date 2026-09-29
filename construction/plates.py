"""Laser-cut plates: the leg links and the two frame plates.

These groups run last: they cut the holes every other group asked for
(:attr:`construction.base.Realized.cuts`) and grow the frame plates by the
pads other groups need (the servo footprint, chassis tabs).
"""

from __future__ import annotations

from construction.base import FRAME_INNER, FRAME_OUTER, Build, Context, Realized, hardware
from shapes import link_plate, plate
from stack import Claim, Layout, Pill, Placed


class LinkPlates:
    """Every leg link, cut from the sheet, in the layer the plan gives it."""

    name = "links"

    def claims(self, ctx: Context) -> list[Claim]:
        r = ctx.params.link_radius

        def make(link: str, segs):
            def f(L: Layout):
                return [Placed(L.layers[link], Pill(a, b, r), link, link) for a, b in segs]
            return f

        return [Claim(n, frozenset((n,)), make(n, segs)) for n, segs in ctx.topo.links.items()]

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        r = build.ctx.params.link_radius
        for name, segs in build.plan.topo.links.items():
            z0, z1 = build.z(build.layers[name])
            part = link_plate([(build.xy(a), build.xy(b)) for a, b in segs], z0, z1,
                              holes=done.cuts.get(name, []), radius=r)
            out.bodies.append(hardware(name, part, name, fab="laser"))
        return out


class FramePlates:
    """The inner (servo) and outer frame plates: arms from O out to every pillar."""

    name = "frame"

    def claims(self, ctx: Context) -> list[Claim]:
        return []   # layers 0 and top are reserved for these plates by the planner

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        p = build.ctx.params
        topo = build.plan.topo
        o = tuple(build.xy("O"))
        arms = [(o, tuple(build.xy(a.name)), p.frame_radius) for a in topo.axes_of("frame")]
        frame = topo.frame_bodies[0]
        for key, layer, name in ((FRAME_INNER, build.top, frame),
                                 (FRAME_OUTER, 0, "frame_outer")):
            z0, z1 = build.z(layer)
            pills = arms + done.pads.get(key, [])
            part = plate(pills, z0, z1, done.cuts.get(key, []), discs=[(o, p.frame_radius)])
            out.bodies.append(hardware(name, part, frame, fab="laser", color="#eb6834"))
        return out


__all__ = ["FramePlates", "LinkPlates"]
