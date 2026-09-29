"""Crank constructions.

The crank turns about O, driven by the servo horn, and carries one crankpin
per leg (point M), which b1 turns on. Every b1 sweeps over O, and the crank
turns fully relative to b1, so the crank can cross a b1's layer only along
that b1's own crankpin: it is a **built-up crankshaft**, with a web (an arm
from O out to the crankpin) in the layers either side of each b1, a body on
O through the other layers, a hub under the servo horn, and a journal stub
turning in the outer frame plate. That shape (:meth:`CrankGroup.claims`) is
the same for every construction; a construction decides radii and how the
pieces are made and joined.

:class:`PrintedCrank`: printed segments, split at every b1 layer (b1 has to
be threaded onto its crankpin). Each crankpin is a printed post on one
segment that passes through b1 into the next. The hub bolts to the servo
horn.
"""

from __future__ import annotations

from dataclasses import dataclass

from construction.base import (
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    DriveInterface,
    Params,
    Realized,
    hardware,
)
from construction.envelope import claimed_solid
from shapes import Cut, box
from stack import Claim, Disc, Layout, Pill, Placed

GROUP = "crank"


@dataclass(frozen=True)
class CrankDims:
    """The radii a construction builds the crank with (mm)."""

    web: float        # half-width of a web (pill O -> crankpin)
    journal: float    # body on O between webs
    stub: float       # journal stub through the outer frame plate
    post: float       # crankpin post, inside b1's hole
    hub: float        # coupling disc under the horn
    hub_thickness: float


def hub_layers(layout: Layout, drive: DriveInterface, hub_thickness: float) -> tuple[range, range]:
    """(horn layers, hub layers) below the inner frame plate's top face."""
    plate_top = layout.z(layout.top)[1]
    face = plate_top - drive.horn_face_depth
    horn = layout.layers_between(face, face + drive.horn_thickness)
    hub = layout.layers_between(face - hub_thickness, face)
    return horn, hub


class CrankGroup:
    """The crankshaft of one side, built by ``construction``."""

    name = GROUP

    def __init__(self, construction):
        self.construction = construction

    def dims(self, ctx: Context) -> CrankDims:
        return self.construction.dims(ctx)

    def claims(self, ctx: Context) -> list[Claim]:
        topo = ctx.topo
        if topo.center is None:
            return []
        drive: DriveInterface = ctx.interfaces["drive"]
        d = self.dims(ctx)
        pins = topo.axes_of("crankpin")
        riders = topo.riders

        def hub(L: Layout):
            horn, hub = hub_layers(L, drive, d.hub_thickness)
            out = [Placed(k, Disc("O", drive.horn_radius), GROUP, "servo horn", seat=k >= L.top)
                   for k in horn if k <= L.top]
            out += [Placed(k, Disc("O", d.hub), GROUP, "crank hub") for k in hub]
            return out

        def webs(pin: str, members: tuple[str, ...]):
            def make(L: Layout):
                rs = {L.layers[b] for b in members}
                out = []
                for s in sorted(rs):
                    out.append(Placed(s, Disc(pin, d.post), GROUP, f"crankpin {pin}", seat=True))
                    for k in (s - 1, s + 1):
                        if k not in rs:
                            out.append(Placed(k, Pill("O", pin, d.web), GROUP, f"web {pin}"))
                return out
            return make

        def body(L: Layout):
            rider_layers = {L.layers[b] for b in riders}
            web_layers = {k for s in rider_layers for k in (s - 1, s + 1)} - rider_layers
            _, hub = hub_layers(L, drive, d.hub_thickness)
            lo, hi = min(web_layers), min(hub)
            if lo > hi:
                return None
            out = [Placed(k, Disc("O", d.journal), GROUP, "crank body")
                   for k in range(lo, hi) if k not in rider_layers]
            out += [Placed(k, Disc("O", d.stub), GROUP, "journal stub") for k in range(1, lo)]
            out.append(Placed(0, Disc("O", d.stub), GROUP, "journal stub", seat=True))
            return out

        claims = [Claim("crank hub", frozenset(), hub)]
        claims += [Claim(f"crank webs {p.name}", frozenset(p.members), webs(p.name, p.members))
                   for p in pins]
        claims.append(Claim("crank body", frozenset(riders), body))
        return claims

    def realize(self, build: Build) -> Realized:
        if build.plan.topo.center is None:
            return Realized()
        return self.construction.realize(self, build)


@dataclass(frozen=True)
class PrintedCrank:
    """Printed crankshaft segments, bolted to the servo horn."""

    key: str = "printed"
    label: str = "printed crankshaft (segments joined through each b1, bolted to the horn)"

    def dims(self, ctx: Context) -> CrankDims:
        p: Params = ctx.params
        drive: DriveInterface = ctx.interfaces["drive"]
        hub = max(drive.horn_radius, drive.screw_pcd / 2 + drive.screw_head_d / 2 + p.min_wall)
        dims = CrankDims(web=p.web_radius, journal=p.journal_d / 2, stub=p.stub_d / 2,
                         post=p.crankpin_d / 2, hub=hub, hub_thickness=p.hub_thickness)
        if p.hole(p.crankpin_d) / 2 + p.min_wall > p.link_radius:
            raise ConstructionError(
                f"a {p.crankpin_d} mm crankpin leaves less than {p.min_wall} mm of b1 around it "
                f"(link radius {p.link_radius})")
        if dims.journal > dims.web:
            raise ConstructionError("the crank body on O must not be wider than a web")
        return dims

    def realize(self, group: CrankGroup, build: Build) -> Realized:
        """First pass: each segment is exactly its claimed shape (no fasteners yet)."""
        out = Realized()
        d = group.dims(build.ctx)
        topo = build.plan.topo
        host = topo.crank_bodies[0]
        shapes = [p for p in build.shapes(GROUP) if p.label != "servo horn"]
        rider_layers = sorted({build.layers[b] for b in topo.riders})
        drive = build.ctx.interfaces["drive"]
        face = build.z(build.top)[1] - drive.horn_face_depth
        above_face = box((0.0, 0.0), (1e4, 1e4, 1e3), face)
        # split into segments at rider layers; a crankpin post joins the segment above
        bounds = [-1, *rider_layers, build.top]
        for i in range(len(bounds) - 1):
            lo, hi = bounds[i], bounds[i + 1]
            seg = [p for p in shapes if lo < p.layer < hi or (p.seat and p.layer == lo)]
            seg = [p for p in seg if not (p.seat and p.layer == lo and lo == -1)]
            solid = claimed_solid(build, [p for p in seg if not p.label.startswith("crankpin")
                                          or p.layer == lo])
            if solid is not None:
                solid = solid - above_face          # the hub stops at the horn's face
                out.bodies.append(hardware(f"crank_seg{i}", solid, host, fab="printed",
                                           color="#6a4fc7"))
        params = build.ctx.params
        for pin in topo.axes_of("crankpin"):
            for b in pin.members:
                out.cut(b, Cut(tuple(build.xy(pin.name)), params.hole(2 * d.post)))
        out.cut(FRAME_OUTER, Cut(tuple(build.xy("O")), params.hole(2 * d.stub)))
        return out


__all__ = ["CrankDims", "CrankGroup", "PrintedCrank", "hub_layers"]
