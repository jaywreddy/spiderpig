"""The crank's shape: its routes, claims and group (:class:`CrankGroup`)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import cast

import numpy as np
from build123d import Axis, Box, Location

from spiderpig.construction.base import (
    RIDES_HOST,
    Build,
    Context,
    DriveInterface,
    Group,
    Motion,
    Realized,
)
from spiderpig.servos.mount import MIN_SPACER
from spiderpig.shapes import moved
from spiderpig.stack import Claim, Disc, Keepout, Layout, Pill, Placed, Unbuildable

GROUP = "crank"
SEGMENT_COLOR = "#6a4fc7"
STEEL = "#4a4a4a"
BOLT_COLOR = "#8fb3e0"           # the crank's plates (the crank study's blue)
EPS = 1e-9


PRESS_DRAWN = 0.02      # a pressed printed bore drawn this much over its steel (no clash)


def hex_play(af: float, pocket_af: float) -> float:
    """Rotation (deg, either way) of a hex ``af`` across flats in a hex pocket
    ``pocket_af`` across flats before its corners (``af / sqrt 3`` out) meet the pocket's
    flats: ``R cos(30 deg - play) = pocket_af / 2``. 0 when the pocket is no wider."""
    if pocket_af <= af:
        return 0.0
    r = af / math.sqrt(3)
    return 30.0 - math.degrees(math.acos(min(1.0, pocket_af / 2 / r)))


# ---------------------------------------------------------------------------
# Claims
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CrankDims:
    """The radii a construction builds the crank with (mm)."""

    web: float        # half-width of a web (pill O -> crankpin)
    journal: float    # body on O between webs
    stub: float       # journal stub through the outer frame plate
    post: float       # crankpin post, inside b1's hole
    hub: float        # coupling disc under the horn
    hub_thickness: float


@dataclass(frozen=True)
class Run:
    """The crankshaft runs along the post at ``at`` over layers ``lo``..``hi``.

    ``at`` is a crankpin, or a detour point fixed to the crank (a point of the
    plan's geometry that turns with it, :meth:`stack.Topology.add_crank_point`).
    Webs from O lead in at ``lo - 1`` and
    out at ``hi + 1``; the centre O is free in the run's layers.
    """

    at: str
    lo: int
    hi: int


@dataclass(frozen=True)
class CrankRoute:
    """The crankshaft's shape through the stack: its runs off the centre, and whether it
    keeps its journal stub in the outer frame plate (the bottom bearing)."""

    runs: tuple[Run, ...]
    bearing: bool = True


def default_route(layout: Layout, pins) -> CrankRoute:
    """Each crankpin's riders, grouped into runs of adjacent layers (the crank as it was)."""
    runs = []
    for p in pins:
        for k in sorted({layout.layers[b] for b in p.members if b in layout.layers}):
            if runs and runs[-1].at == p.name and runs[-1].hi == k - 1:
                runs[-1] = Run(p.name, runs[-1].lo, k)
            else:
                runs.append(Run(p.name, k, k))
    return CrankRoute(tuple(runs))


def route_of(layout: Layout, pins) -> CrankRoute:
    """The route the planner chose (``layout.choices["crank"]``), else :func:`default_route`."""
    chosen = layout.choices.get(GROUP)
    # the crank's choice is its router's CrankRoute (Route.choice)
    return cast("CrankRoute", chosen) if chosen is not None else default_route(layout, pins)


def chains_of(runs) -> list[list[Run]]:
    """Runs along one point whose webs meet (in one layer, or in two adjacent ones) are one
    **chain**: one screw through all their posts, since a nut and a head can't share that
    column. In order of the runs' lowest layers."""
    chains: list[list[Run]] = []
    for r in sorted(runs, key=lambda r: (r.lo, r.at)):
        c = next((c for c in chains if c[-1].at == r.at and r.lo - c[-1].hi <= 3), None)
        if c is None:
            chains.append([r])
        else:
            c.append(r)
    return chains


def hub_layers(layout: Layout, drive: DriveInterface, hub_thickness: float) -> tuple[range, range]:
    """(horn layers, hub layers) below the inner frame plate's top face. A search's layout
    (not :attr:`stack.Layout.final`) takes the inner plate at its sheet's thickness
    (``drive.plate_t``), as the plan will have it."""
    if drive.horn_layers:
        # whole plates: the hub under the horn's layers, whatever their thickness (the
        # printed horn spacer takes it up)
        top, n = layout.top, drive.horn_layers
        h = max(1, round(hub_thickness / layout.pitch))
        return range(top - n, top + 1), range(top - n - h, top - n)
    if not layout.final and drive.plate_t and abs(drive.plate_t - layout.pitch) > 1e-9:
        layout = Layout({}, layout.top, layout.pitch, thick={layout.top: drive.plate_t})
    plate_top = layout.z(layout.top)[1]
    face = plate_top - drive.horn_face_depth
    horn = layout.layers_between(face, face + drive.horn_thickness)
    hub = layout.layers_between(face - hub_thickness, face)
    return horn, hub


class CrankGroup(Group):
    """The crankshaft of one side, built by ``construction``."""

    name = GROUP

    def __init__(self, construction):
        self.construction = construction

    def dims(self, ctx: Context) -> CrankDims:
        return self.construction.dims(ctx)

    def keepouts(self, ctx: Context) -> list[Keepout]:
        """The journal on O, wherever the crank runs on its axis (a link sweeping across it
        needs a layer where the crank runs along a post instead)."""
        if ctx.topo.center is None:
            return []
        return [Keepout(GROUP, ("pt", "O"), self.dims(ctx).journal,
                        "where the crank runs on its axis", frozenset(ctx.topo.riders))]

    def reach(self, ctx: Context) -> float:
        """The radius the crank sweeps about O: its farthest crankpin plus a web."""
        g = ctx.topo.geometry.points
        return max(float(np.linalg.norm(g[p.name][0] - g["O"][0]))
                   for p in ctx.topo.axes_of("crankpin")) + self.dims(ctx).web

    def router(self, ctx: Context, envelope, margin: float, drop_bearing: bool = False):
        """The planner's router for this crank (:mod:`construction.route`): the static facts
        (which links need O free, detour points inside ``envelope``) and the route search."""
        from spiderpig.construction.route import CrankRouter, crank_facts, joint_rules

        d = self.dims(ctx)
        return CrankRouter(ctx, d, crank_facts(ctx, d, envelope, margin), drop_bearing,
                           joint_rules(self.construction, ctx, d))

    def claims(self, ctx: Context) -> list[Claim]:
        """The hub under the servo horn (fixed), and the rest from the planner's route
        (:func:`route_of`): each run's post (inside a rider's hole where one rides it) and
        its two webs, the journal on O from the lowest web up to the hub, and the stub from
        there down into the outer frame plate when the route keeps the bearing."""
        topo = ctx.topo
        if topo.center is None:
            return []
        drive: DriveInterface = ctx.interfaces["drive"]
        d = self.dims(ctx)
        pins = topo.axes_of("crankpin")
        riders = topo.riders

        c = self.construction
        plate_t = ctx.sheet_t("crank")
        horn_pts = c.horn_points(ctx)

        def hub(L: Layout):
            horn, hub = hub_layers(L, drive, d.hub_thickness)
            if L.final and drive.horn_layers and hub:
                # the printed horn spacer takes up what the plan's z leaves over the hub
                t = (L.z(L.top)[1] - L.z(max(hub))[1]
                     - (drive.horn_face_depth - drive.spacer_t))
                if t < -1e-6 or 1e-6 < t < MIN_SPACER:
                    raise Unbuildable(f"at the plan's z the hub's top face leaves a {t:.2f} mm "
                                      "horn spacer")
            out = [Placed(k, Disc("O", drive.horn_radius), GROUP, "servo horn", seat=k >= L.top)
                   for k in horn if k <= L.top]
            # the horn and its spacer through any clearance gap among its layers, and over
            # the hub
            ks = [k for k in horn if k <= L.top]
            if ks:
                out += [Placed(k, Disc("O", drive.horn_radius), GROUP, "servo horn", gap=True)
                        for k in range(min(ks) - 1, L.top)]
            out += [Placed(k, Disc("O", d.hub), GROUP, "crank hub", sheet=plate_t) for k in hub]
            return out

        def shaft(L: Layout):
            route = route_of(L, pins)
            ridden: dict[str, set[int]] = {}
            for b, pin in riders.items():
                ridden.setdefault(pin, set()).add(L.layers[b])
            for pin, ks in ridden.items():
                if off := sorted(k for k in ks if not any(r.at == pin and r.lo <= k <= r.hi
                                                          for r in route.runs)):
                    raise Unbuildable(f"a link riding {pin} sits in layer {off[0]}, off every "
                                      f"run of the crankshaft along {pin}")
            out = []
            for r in route.runs:
                out += [Placed(k, Disc(r.at, d.post), GROUP, f"crankpin {r.at}",
                               seat=k in ridden.get(r.at, ()))
                        for k in range(r.lo, r.hi + 1)]
                out += [Placed(k, Pill("O", r.at, d.web), GROUP, f"web {r.at}", sheet=plate_t)
                        for k in (r.lo - 1, r.hi + 1)]
                # the washers on the crankpin through a gap along its run (the jam nut
                # in the gap over the lowest web)
                first = r.lo if not any(q.at == r.at and q.hi < r.lo
                                        for q in route.runs) else r.lo - 1
                out += [Placed(k, Disc(r.at, c.washer_r), GROUP, f"crankpin {r.at} washer",
                               gap=True) for k in range(first, r.hi + 1)]
            run_layers = {k for r in route.runs for k in range(r.lo, r.hi + 1)}
            web_layers = {k for r in route.runs for k in (r.lo - 1, r.hi + 1)} - run_layers
            _, hub = hub_layers(L, drive, d.hub_thickness)
            if not len(hub):
                raise Unbuildable("the hub under the servo horn has no layer (a clearance gap "
                                  "where it would be)")
            lo, hi = min(web_layers), min(hub)
            if lo > hi:
                raise Unbuildable(f"its lowest web (layer {lo}) would sit above the hub under "
                                  f"the servo horn (layer {hi}): the riders are too high")
            top_web = max(r.hi + 1 for r in route.runs)
            out += [Placed(k, Disc("O", d.journal), GROUP, "crank body", sheet=plate_t)
                    for k in web_layers if k <= top_web]
            out += c.web_claims(ctx, L, route, ridden, d, plate_t, drive, hi, horn_pts)
            if route.bearing:     # the stub standoff under the lowest web
                out += [Placed(k, Disc("O", d.stub), GROUP, "journal stub")
                        for k in range(1, lo)]
                out.append(Placed(-1, Disc("O", d.stub), GROUP, "journal stub"))
                out.append(Placed(0, Disc("O", d.stub), GROUP, "journal stub", seat=True))
                tr = c.stub_thrust_r(ctx, route, hi)
                if tr > 0:        # its thrust sleeve, through the layers and gaps there
                    out += [Placed(k, Disc("O", tr), GROUP, "stub thrust sleeve")
                            for k in range(1, lo)]
                    out += [Placed(k, Disc("O", tr), GROUP, "stub thrust sleeve",
                                   gap=True) for k in range(lo)]
            if L.final:
                c.check_route(L, route, ridden, {p.layer for p in out if p.sheet > 0},
                              drive=drive)
            return out

        return [Claim("crank hub", frozenset(), hub),
                Claim("crank route", frozenset(riders), shaft, choice=GROUP)]

    def realize(self, build: Build, done: Realized) -> Realized:
        if build.plan.topo.center is None:
            return Realized()
        return self.construction.realize(self, build)

    def motion(self, got: Realized) -> Motion | None:
        """Every part turns with the crank about O (the webs, the crankpins' hex pockets and
        standoffs at the crank's angle, the horn screws at the horn's), and every hole it
        asks of a link is round at a crankpin; but a hex journal on O is drawn at the
        world's angle 0 whatever the crank's, its pockets in the webs too: such a crank
        changes shape with the angle (``None``)."""
        note = got.notes.get("crank_bolt", {})
        if note.get("journals") and getattr(self.construction, "hex", True):
            return None
        return RIDES_HOST


def _hex(xy, af: float, z0: float, z1: float, angle: float):
    """A hexagonal prism, ``af`` across flats, one pair of flats facing ``angle``."""
    ang = math.degrees(angle)
    boxes = [Box(af, 4 * af, z1 - z0).rotate(Axis.Z, ang + a) for a in (0.0, 60.0, 120.0)]
    prism = boxes[0] & boxes[1] & boxes[2]
    return moved(prism, Location((float(xy[0]), float(xy[1]), (z0 + z1) / 2)))
