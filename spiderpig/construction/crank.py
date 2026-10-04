"""Crank constructions.

The crank turns about O, driven by the servo horn, and carries one crankpin
per leg (point M), which the leg's riders (Klann's b1) turn on. Every rider
sweeps over O, and the crank turns fully relative to it, so the crank can
cross a rider's layer only along that rider's own crankpin: it is a
**built-up crankshaft**. Its shape is a :class:`CrankRoute`, which the
planner may choose (else :func:`default_route`): **runs**, where the shaft
leaves O along a post (at a crankpin, or at a detour point fixed to the
crank) over some layers, each between two **webs** (arms from O out to the
post) in the layers either side; a body on O through the other layers; a hub
under the servo horn; and, with the bottom bearing, a journal stub turning in
the outer frame plate (without it the crank hangs from the servo side). That
shape (:meth:`CrankGroup.claims`) is the same for every construction; a
construction decides radii and how the pieces are made and joined.

Two constructions build it, both printed segments split at every run (a
rider has to be threaded onto its post): :class:`KeyedCrank` (``keyed``, the
default) keys every post to the web above it with a brass hex standoff and
clamps each chain with a screw and nut through a two-layer top web;
:class:`PrintedCrank` (``printed``) is the same stack held by the screw's
clamp friction alone, kept for comparison (its joints are free to twist about
the post once the clamp creeps: 0.17-0.67 N·m of friction against 0.36 N·m
walking on the Strider, 1.3 N·m on the Klann). Going up from the outer frame
plate, :class:`PrintedCrank`:

* **segments**: the layers between runs, each the union of its claimed
  shapes. The bottom one carries the journal stub, or ends at the lowest web
  without the bearing; the top one is the hub. A crank face against a rider
  is set back by ``axial_play``, the rider's end play: the face below a run's
  riders when they fill the run, else every face a rider touches. A face
  toward a run layer no rider sits in isn't: nothing turns against it;
* **posts**: a printed post (``Params.crankpin_d``) on the segment below each
  run. It runs through the run's layers, bare where no rider rides it (a
  link may pass it there), and butts against the web of the segment above;
* **joints**: an axial M3 screw runs through each post. Its head is recessed
  into the lower web from below and it screws into a hex nut trapped in the
  upper web (pocket open away from the post). An ISO 4762 head is 3 mm tall
  and doesn't fit a 3 mm web with any floor, so at that pitch the screw is
  an ISO 7380 button head (:data:`POST_SCREWS` is the order of preference).
  Runs along one point whose webs meet (in one layer, or in two adjacent
  ones) are one joint: one screw through all their posts, since a nut and a
  head can't share that column. Pockets reach their web through a tunnel the
  height of the segment, so two joints' (or a joint's and the horn screws')
  pockets in one segment must not meet;
* **detours** are split and screwed like crankpins. O is free in a detour's
  layers, so in one piece its upper web would print hanging off the post over
  nothing (support trapped between the webs), and the post, which carries
  the drive torque past the detour, would be loaded across its print layers
  with no screw clamping it. Split, each segment prints standing on the flat
  face of its lowest web (or its stub), posts upright;
* **hub**: bolts to the horn with the horn's screws from below, through the
  hub into the horn's holes (through the drive's horn spacer, if any). The
  heads sit in counterbores in the hub, reached through tunnels in the rest
  of the top segment; a pocket clears the horn's centre screw. The hub's top
  face is the horn's outer face, and within ``Params.margin`` of the inner
  frame plate the hub is no wider than the horn (it turns inside the plate's
  horn hole). Nothing of the crank rises past the horn's outer face. A
  chain's highest web may share the hub's lowest layer; set back for its
  rider's end play it shortens the hub, which the route rules allow only
  when the horn screws still fit (``JointRules.hub_play`` in
  :mod:`construction.route`).

Assembly: servo on the inner plate; the top segment bolted to the horn
(nut in its trap first); then, going down, each rider onto its post and the
next segment up to it (every segment of a joint, then its screw from below);
the outer frame plate last, over the journal stub.

:class:`KeyedCrank` changes the joints and the top web, nothing else:

* **keys**: a brass M3 female-female hex standoff (:data:`STANDOFF_KEY`; its
  across-flats and length come from the catalog item, never from a constant
  here, the AF overridden by ``key_af`` for a measured kit) sits at every
  post-to-web interface, half in a hex cavity in the top of the post and half
  in a hex socket in the underside of the web above, with a lead-in step at
  both mouths. It carries the twist about the post (the drive torque times
  chord / crank radius, 2.0 on the Strider and the Klann, which the printed
  post's clamp friction cannot) and nothing axial, so its cavity and socket
  leave it ``key_float`` of room along the screw (a threaded key is locked to
  the screw's thread, so the thread pitch plus the length tolerance must
  fit). Across flats it is **pressed** (``key_fit="press"``, the default):
  both pockets are cut to the key's AF (``press_fit`` 0.0; FDM holes print
  0.05-0.15 mm small, a light press). A sliding fit (``"float"``,
  ``standoff_fit`` 0.15, ``--crank keyed_float``) lets the key turn
  :func:`hex_play` = 3.13 deg in each pocket, 6.25 deg either way between a
  post and its web; the screw then carries the slip about its own axis each
  time the twist beats the clamp's friction (0.17-0.67 N·m against 0.36 N·m
  walking on the Strider, 1.33 on the Klann), working the head and nut loose.
  Pressed, the play is 0 (2.0 deg should a pocket print 0.05 mm over), and
  each chain screw gets a drop of low-strength threadlocker (``lock_key``) so
  the clamp keeps its preload. Two keys side by side would lock the joint
  without the hex fit but need a 13.9 mm post (two 5.0 AF keys, 11.6 for M2)
  where the rider's hole leaves room for 8.8, and a pin across the interface
  has nowhere to go (the joint face is level, the screw on the axis). The post
  is 8.5 mm (``KeyedCrank.post_d``, unless ``Params.crankpin_d`` is wider)
  for 1.3 mm of wall round the cavity's corners, which costs TrotBot's heel
  its plan at the default scale (b7 passes J1 at 10.2 mm; a 4.25 mm post
  needs 11.2);
* **the clamp** is :class:`PrintedCrank`'s: one button-head screw per chain
  from a counterbore in its lowest web through every key (they float on it)
  into a stock hex nut on a floor in its highest web, so every rider
  interface is clamped to ``axial_play``. That web is **two layers thick** at
  the crankpin: the hex socket from below, ``min_key_floor`` of floor, the
  nut trap from above (one layer holds no socket under a nut), which the
  planner's route rules know (:class:`construction.route.JointRules`,
  ``two_layer_top``) and pay for: the Strider double plans at 18 layers (54
  mm a side) instead of 16, the Klann quad at 16 instead of 12. A chain of
  several runs has a key in every post, so the one-layer web between two
  runs holds a socket from below and a cavity's overflow from above with
  ``min_key_floor`` between them, which a 4 mm key leaves and a 5 mm one
  does not at a 3 mm pitch (:meth:`KeyedCrank.dims` refuses it).

Assembly with keys: the same order, plus one step per interface: press a key
into the post's cavity before threading the rider on (flats square, a flat
block or a vise jaw on top until it bottoms), then lower the next segment so
its socket meets the key (turn it until the hex indexes in the lead-in; a 60°
mis-index shows at once, the webs point the wrong way), nut in the top trap,
a drop of threadlocker on the screw's tip, and the screw from below through
the keys into the nut: tightening it draws the web down over the key. To take
it apart, unscrew it and lift the web off the key. A pocket printed loose
(the key turns in it by hand) takes a drop of CA; one too tight to press,
``press_fit`` 0.05. The outer frame plate last.

Screw lengths are stock lengths chosen for the most thread engagement that
keeps every head, nut and tip inside the crank's claims; a joint no stock
length fits raises :class:`ConstructionError` (at a 3 mm pitch: 5, 6, 8 or
more than 9 layers between a printed crank's webs; a keyed chain's webs 4, 6
or 9 layers apart, both layers of the top web counted, take one).
"""

from __future__ import annotations

import functools
import itertools
import math
import re
from dataclasses import dataclass, replace

import numpy as np
from build123d import Axis, Box, Location

from spiderpig.construction.base import (
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    DriveInterface,
    Group,
    Params,
    Realized,
    hardware,
)
from spiderpig.construction.envelope import shape_solid
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import adhesive
from spiderpig.hardware.parts import SHCS_LENGTHS, shcs
from spiderpig.servos.mount import MIN_SPACER
from spiderpig.shapes import Cut, disc, moved, union
from spiderpig.stack import Claim, Disc, Keepout, Layout, Pill, Placed, Unbuildable

GROUP = "crank"
SEGMENT_COLOR = "#6a4fc7"
STEEL = "#4a4a4a"
EPS = 1e-9


# ---------------------------------------------------------------------------
# Fasteners (mm)
# ---------------------------------------------------------------------------

SIZES = {"M2": "2", "M2.5": "2p5", "M3": "3"}
NOMINAL = {"2": 2.0, "2p5": 2.5, "3": 3.0}


@dataclass(frozen=True)
class ScrewKind:
    """A screw type: head size, stock lengths, and its catalog key per length."""

    kind: str                   # "shcs" | "bhcs" | "self_tap"
    size: str                   # "2", "2p5", "3"
    head_d: float
    head_h: float
    lengths: tuple[float, ...]

    @property
    def d(self) -> float:
        return NOMINAL[self.size]

    @property
    def shank_d(self) -> float:
        """Modelled shank: about the thread's major diameter (ISO 965 6g: M3 2.874-2.98)."""
        return 0.97 * self.d

    def key(self, length: float) -> str:
        if self.kind == "shcs":
            return shcs(self.size, length)
        return f"m{self.size}_{self.kind}_{length:g}"


# ISO 4762 heads (docs research: joinery.json, m3_shcs / m2_m2p5_shcs)
SHCS = {size: ScrewKind("shcs", size, hd, hh, SHCS_LENGTHS[size])
        for size, hd, hh in (("2", 3.8, 2.0), ("2p5", 4.5, 2.5), ("3", 5.5, 3.0))}
# ISO 7380-1 button head M3: dk 5.7, k 1.65 (from the standard's table; not in the
# project's research data). Preferred lengths.
BHCS = {"3": ScrewKind("bhcs", "3", 5.7, 1.65, (6, 8, 10, 12, 16, 20, 25, 30))}
# M2 pan-head tapping screws for pilot holes (PA2.0 / "PHS M2 TAP"): head about
# 4.0 x 1.6 (ISO 7049 ST2.2; UNVERIFIED for the servo makers' screws).
SELF_TAP = {"2": ScrewKind("self_tap", "2", 4.0, 1.6, (6, 8, 10, 12))}
POST_SCREWS = (SHCS["3"], BHCS["3"])       # order of preference for the crankpin joints

NUT_KEY = "m3_nut"
NUT_AF, NUT_H, NUT_BORE = 5.5, 2.4, 3.0    # ISO 4032 M3 (docs research: m3_hex_nut)
# The keyed crank's key: a brass M3 female-female hex standoff (hardware/parts.py). Its
# across-flats size the sockets and its length the pockets' depths (:func:`standoff_dims`).
STANDOFF_KEY = "m3_hex_standoff_ff_4"
SOCKET_MAX = 2.4                           # deepest hex socket in the web above a post
BRASS = "#b08d3c"


KEY_FITS = ("press", "float")              # KeyedCrank.key_fit


def hex_play(af: float, pocket_af: float) -> float:
    """Rotation (deg, either way) of a hex ``af`` across flats in a hex pocket
    ``pocket_af`` across flats before its corners (``af / sqrt 3`` out) meet the pocket's
    flats: ``R cos(30 deg - play) = pocket_af / 2``. 0 when the pocket is no wider."""
    if pocket_af <= af:
        return 0.0
    r = af / math.sqrt(3)
    return 30.0 - math.degrees(math.acos(min(1.0, pocket_af / 2 / r)))


def standoff_dims(key: str = STANDOFF_KEY) -> tuple[float, float]:
    """(across flats, length) in mm of the catalog's hex standoff ``key``."""
    from spiderpig.hardware.catalog import get

    dims = get(key).dims
    return float(dims["af"]), float(dims["length"])


_SCREW_KEY = re.compile(r"^m(\d+(?:p\d+)?)_(shcs|bhcs|self_tap)_(\d+(?:\.\d+)?)$")


def screw_from_key(key: str | None) -> tuple[ScrewKind, float] | None:
    """``"m2_self_tap_6"`` -> (the M2 tapping screw kind, 6.0); ``None`` if not modelled."""
    m = _SCREW_KEY.match(key or "")
    if m is None:
        return None
    size, kind, length = m.groups()
    sk = {"shcs": SHCS, "bhcs": BHCS, "self_tap": SELF_TAP}[kind].get(size)
    return None if sk is None else (sk, float(length))


def screw_body(xy, sk: ScrewKind, bearing_z: float, length: float, up: bool = True):
    """A screw whose head bears at ``bearing_z``, shank pointing up (or down)."""
    s = 1.0 if up else -1.0
    head = disc(xy, sk.head_d / 2, *sorted((bearing_z - s * sk.head_h, bearing_z)))
    shank = disc(xy, sk.shank_d / 2, *sorted((bearing_z, bearing_z + s * length)))
    return union([head, shank])


def _horn_screw_kind(drive_thread: str, tapping: bool) -> ScrewKind:
    size = SIZES.get(drive_thread)
    if size is None:
        raise ConstructionError(f"no screws modelled for a {drive_thread or 'unknown'} horn thread")
    if tapping:
        if size not in SELF_TAP:
            raise ConstructionError(f"no tapping screws modelled for {drive_thread}")
        return SELF_TAP[size]
    return SHCS[size]


# ---------------------------------------------------------------------------
# Claims (the same for every construction)
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
    tip: float = 0.0  # a crankpin's tip under its chain's lowest web (the bolt crank's thread)


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
    return chosen if chosen is not None else default_route(layout, pins)


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

        plate_t = ctx.sheet_t("crank") if getattr(self.construction, "plates", False) else 0.0
        washer_r = getattr(self.construction, "washer_r", 0.0)
        face_on_layer = getattr(self.construction, "face_on_layer", False)
        single = getattr(self.construction, "single", False)
        horn_pts = self.construction.horn_points(ctx) if single else []

        def hub(L: Layout):
            horn, hub = hub_layers(L, drive, d.hub_thickness)
            if L.final and face_on_layer and drive.horn_layers and hub:
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
                if washer_r > 0:    # washers on the crankpin through a gap along its run
                    # (single webs: the jam nut is in the gap over the lowest web)
                    first = r.lo if single and not any(
                        q.at == r.at and q.hi < r.lo for q in route.runs) else r.lo - 1
                    out += [Placed(k, Disc(r.at, washer_r), GROUP, f"crankpin {r.at} washer",
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
            if single:
                top_web = max(r.hi + 1 for r in route.runs)
                out += [Placed(k, Disc("O", d.journal), GROUP, "crank body", sheet=plate_t)
                        for k in web_layers if k <= top_web]
                out += self.construction.web_claims(ctx, L, route, ridden, d, plate_t, drive,
                                                    hi, horn_pts)
                if route.bearing:     # the stub standoff under the lowest web
                    out += [Placed(k, Disc("O", d.stub), GROUP, "journal stub")
                            for k in range(1, lo)]
                    out.append(Placed(-1, Disc("O", d.stub), GROUP, "journal stub"))
                    out.append(Placed(0, Disc("O", d.stub), GROUP, "journal stub", seat=True))
                check = getattr(self.construction, "check_route", None)
                if L.final and check is not None:
                    check(L, route, ridden, {p.layer for p in out if p.sheet > 0}, drive=drive)
                return out
            out += [Placed(k, Disc("O", d.journal), GROUP, "crank body", sheet=plate_t)
                    for k in range(lo, hi) if k not in run_layers]
            if getattr(self.construction, "two_layer_bottom", False):
                # a bolt chain's lowest web is two layers thick: the layer under it as well,
                # and the crankpin's tip (the bolt's thread past its nut) the layer under that
                for c in chains_of(route.runs):
                    k = c[0].lo - 2
                    if k in run_layers or k < 1:
                        raise Unbuildable(
                            f"the two-layer lowest web of {c[0].at} needs layer {k}, "
                            + ("where another run needs O free" if k in run_layers
                               else "in the outer frame plate"))
                    out.append(Placed(k, Pill("O", c[0].at, d.web), GROUP, f"web {c[0].at}",
                                      sheet=plate_t))
                    if d.tip > 0:
                        if k - 1 < 1:
                            raise Unbuildable(f"the tip of crankpin {c[0].at} needs layer "
                                              f"{k - 1}, in the outer frame plate")
                        out.append(Placed(k - 1, Disc(c[0].at, d.tip), GROUP,
                                          f"crankpin tip {c[0].at}"))
                lo = min([lo] + [c[0].lo - 2 for c in chains_of(route.runs)])
                out += [Placed(k, Disc("O", d.journal), GROUP, "crank body", sheet=plate_t)
                        for k in range(lo, hi) if k not in run_layers
                        and not any(q.layer == k and q.label == "crank body" for q in out)]
            if getattr(self.construction, "two_layer_top", False):
                # a keyed chain's highest web is two layers thick: the layer over it as well
                for c in chains_of(route.runs):
                    k = c[-1].hi + 2
                    if k in run_layers or k > hi:
                        raise Unbuildable(
                            f"the two-layer top web of {c[0].at} needs layer {k}, "
                            + ("where another run needs O free" if k in run_layers
                               else "above the hub's lowest layer"))
                    out.append(Placed(k, Pill("O", c[0].at, d.web), GROUP, f"web {c[0].at}",
                                      sheet=plate_t))
            if route.bearing:
                out += [Placed(k, Disc("O", d.stub), GROUP, "journal stub") for k in range(1, lo)]
                out.append(Placed(0, Disc("O", d.stub), GROUP, "journal stub", seat=True))
            check = getattr(self.construction, "check_route", None)
            if L.final and check is not None:
                check(L, route, ridden, {p.layer for p in out if p.sheet > 0}, drive=drive)
            return out

        return [Claim("crank hub", frozenset(), hub),
                Claim("crank route", frozenset(riders), shaft, choice=GROUP)]

    def realize(self, build: Build, done: Realized) -> Realized:
        if build.plan.topo.center is None:
            return Realized()
        return self.construction.realize(self, build)


# ---------------------------------------------------------------------------
# The printed crankshaft
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PostJoint:
    """How one crankpin joint is screwed together (z in side coordinates)."""

    screw: ScrewKind
    length: float
    head_depth: float     # head counterbore, from the lower web's bottom face
    nut_depth: float      # nut trap, from the upper web's top face
    engagement: float     # thread in the nut


@dataclass(frozen=True)
class HornJoint:
    """How the hub bolts to the horn."""

    screw: ScrewKind
    length: float
    floor: float          # hub material between a screw head and the hub's top face
    engagement: float     # thread in the horn (from its outer face)
    spacer: float         # drive's horn spacer the screw passes first


def _hex(xy, af: float, z0: float, z1: float, angle: float):
    """A hexagonal prism, ``af`` across flats, one pair of flats facing ``angle``."""
    ang = math.degrees(angle)
    boxes = [Box(af, 4 * af, z1 - z0).rotate(Axis.Z, ang + a) for a in (0.0, 60.0, 120.0)]
    prism = boxes[0] & boxes[1] & boxes[2]
    return moved(prism, Location((float(xy[0]), float(xy[1]), (z0 + z1) / 2)))


@dataclass(frozen=True)
class PrintedCrank:
    """Printed crankshaft segments joined through each crankpin, bolted to the servo horn."""

    key: str = "printed"
    label: str = ("printed crankshaft (segments screwed together through each crankpin, "
                  "bolted to the horn)")
    axial_play: float = 0.15     # b1 end play: the web below each b1 is this much thinner
    post_bore: float = 3.4       # through each post, for its M3 screw (ISO 273 medium)
    screw_fit: float = 0.4       # counterbore diameter over a screw head
    nut_fit: float = 0.3         # nut trap across-flats over the nut
    head_recess: float = 0.05    # heads and nuts sit at least this far below a face
    tip_recess: float = 0.05     # screw tips stay this far inside the upper web
    min_web_floor: float = 0.8   # printed floor between a head counterbore and the post
    min_nut_floor: float = 0.3   # printed floor under a nut trap (loaded in compression)
    min_nut_engage: float = 1.5  # thread in the nut (3 pitches of M3)
    min_hub_floor: float = 1.5   # printed floor between a horn-screw head and the horn
    min_horn_engage: float = 1.5  # thread in the horn

    # -- dimensions and checks ----------------------------------------------------

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
        if (p.crankpin_d - self.post_bore) / 2 < 0.8:
            raise ConstructionError(f"a {p.crankpin_d} mm crankpin is too thin for an M3 screw")
        if self.tightest_joint(ctx.pitch) is None:
            raise self._pitch_error(ctx, ctx.pitch)
        if self.hub_joint(ctx, p.hub_thickness) is None:
            spec = ctx.servo
            raise ConstructionError(
                f"no {spec.horn.pattern.thread} screw fits between the crank hub and the "
                f"{spec.key} horn")
        return dims

    def tightest_joint(self, pitch: float) -> PostJoint | None:
        """The tightest crankpin joint at this layer pitch: one rider between two one-layer
        webs, both next to other riders (set back for their end play)."""
        return self.post_joint(0.0, pitch - self.axial_play, 2 * pitch,
                               3 * pitch - self.axial_play)

    def least_pitch(self, pitch: float, limit: float = 8.0) -> float | None:
        """The least layer pitch (from ``pitch`` up, in 0.05 mm steps) at which the tightest
        crankpin joint (:meth:`tightest_joint`) takes a stock screw and nut; ``None`` when
        none does up to ``limit``."""
        p = round(math.ceil(pitch / 0.05 - EPS) * 0.05, 6)
        while p <= limit + EPS:
            if self.tightest_joint(p) is not None:
                return round(p, 2)
            p = round(p + 0.05, 6)
        return None

    def _pitch_why(self, pitch: float) -> tuple[str, str]:
        """Why no crankpin joint fits in ``pitch`` mm layers, and what the joints need (one
        clause, for the recommendation's lever)."""
        lo_web = pitch - self.axial_play
        why = (f"no M3 screw and nut fit a crankpin joint in {pitch:g} mm layers: each web of "
               f"the printed crank is one layer thick ({lo_web:g} mm beside a link, after "
               f"{self.axial_play:g} mm of end play) and must hold a screw head under "
               f"{self.min_web_floor:g} mm of floor and a nut under {self.min_nut_floor:g} mm "
               f"(the flattest head is {min(sk.head_h for sk in POST_SCREWS):g} mm, the nut "
               f"{NUT_H:g} mm)")
        return why, "a stock M3 screw head and nut in one-layer webs"

    def _pitch_error(self, ctx: Context, pitch: float) -> ConstructionError:
        """Why no crankpin joint fits in ``pitch`` mm layers, and the thickness that would:
        a web is one layer thick and must hold a screw head (or a nut) under a printed
        floor, so the layer pitch, which is the sheet's thickness, has a least value."""
        from spiderpig.hardware.catalog import sheet_thickness

        least = self.least_pitch(pitch)
        why, need = self._pitch_why(pitch)
        if least is None:
            return ConstructionError(why + "; no layer pitch up to 8 mm fits one",
                                     numbers={"pitch_mm": pitch})
        nominal = sheet_thickness(ctx.config.sheet) if ctx.config is not None else None
        after = nominal if nominal is not None and nominal >= least else least
        lever = (f"; the least layer pitch that fits is {least:g} mm, and the layer pitch is "
                 f"the sheet's thickness: materials.thickness_mm {pitch:g} -> {after:g}"
                 + (f" (the {ctx.config.sheet} sheet's nominal)"
                    if after == nominal and nominal != least else "")
                 + ", or a thicker sheet")
        return ConstructionError(
            why + lever, changes=(("thickness_mm", pitch, after),),
            lever=(f"the {self.key} crank's crankpin joints need layers of at least {least:g} mm "
                   f"({need}), and the layer pitch is the sheet's thickness"),
            numbers={"pitch_mm": pitch, "least_pitch_mm": least})

    def post_joint(self, zl0: float, zl1: float, zu0: float, zu1: float) -> PostJoint | None:
        """The screw for a joint: lower web ``zl0..zl1``, upper web ``zu0..zu1``.

        The head sits in a counterbore in the lower web's bottom face, the nut
        in a trap in the upper web's top face. Most thread in the nut wins,
        then the shorter screw; the first screw kind that fits is used.
        """
        screw = self._screw(zl0, zu1, (zl1 - zl0) - self.min_web_floor,
                            NUT_H + self.head_recess, (zu1 - zu0) - self.min_nut_floor)
        return None if screw is None else PostJoint(*screw)

    def _screw(self, zl0: float, zu1: float, hp_hi: float, np_lo: float, np_hi: float
               ) -> tuple[ScrewKind, float, float, float, float] | None:
        """The stock screw with the most thread in the nut: its head's counterbore at most
        ``hp_hi`` deep from the lower web's bottom face ``zl0``, the nut trap ``np_lo..np_hi``
        deep from the upper web's top face ``zu1``, the tip inside the nut's web; the first
        kind of :data:`POST_SCREWS` that fits. ``(kind, length, head depth, nut depth,
        engagement)``, :class:`PostJoint`'s fields."""
        for sk in POST_SCREWS:
            hp_lo = sk.head_h + self.head_recess
            if hp_lo > hp_hi + EPS or np_lo > np_hi + EPS:
                continue
            best = None
            for length in sk.lengths:
                hp = min(hp_hi, zu1 - self.tip_recess - zl0 - length)
                if hp < hp_lo - EPS:
                    continue
                tip = zl0 + hp + length
                nd = min(np_hi, max(np_lo, zu1 - tip + NUT_H))
                engage = min(NUT_H, tip - (zu1 - nd))
                if engage < self.min_nut_engage - EPS:
                    continue
                if best is None or engage > best[4] + EPS:
                    best = (sk, length, hp, nd, engage)
            if best is not None:
                return best
        return None

    def horn_joint(self, drive: DriveInterface, spec, hub_height: float) -> HornJoint | None:
        """The horn screws: most thread in the horn (up to its thread depth), then shortest.

        The tip may pass the thread (up to the pattern's ``max_depth``) but not
        the inner plate's top face (the horn's claim ends there).
        """
        pat = spec.horn.pattern
        sk = _horn_screw_kind(pat.thread, pat.tapping)
        spacer = drive.horn_face_depth - spec.horn_face_depth
        reach = pat.reach if pat.reach is not None else spec.horn.thickness
        e_max = min(reach, spec.horn_face_depth)
        e_want = min(pat.thread_depth if pat.thread_depth is not None else e_max, e_max)
        e_min = min(e_want, self.min_horn_engage)
        best: HornJoint | None = None
        for length in sk.lengths:
            f_lo = max(self.min_hub_floor, length - spacer - e_max)
            f_hi = min(hub_height - sk.head_h, length - spacer - e_min)
            if f_lo > f_hi + EPS:
                continue
            floor = min(max(length - spacer - e_want, f_lo), f_hi)
            engage = length - spacer - floor
            if best is None or min(engage, e_want) > min(best.engagement, e_want) + EPS:
                best = HornJoint(sk, length, floor, engage, spacer)
        return best

    def hub_joint(self, ctx: Context, hub_thickness: float,
                  set_back: float = 0.0) -> HornJoint | None:
        """The horn joint of the hub claiming ``hub_thickness`` under the horn (whole
        layers: its height depends on the pitch alone, not the stack), its bottom face
        ``set_back`` (a rider's end play under a web sharing its lowest layer)."""
        drive: DriveInterface = ctx.interfaces["drive"]
        top, pitch = 100, ctx.pitch
        face = (top + 1) * pitch - drive.horn_face_depth
        hub_bottom = Layout({}, top, pitch).layers_between(face - hub_thickness, face).start
        return self.horn_joint(drive, ctx.servo, face - hub_bottom * pitch - set_back)

    # -- parts ----------------------------------------------------------------------

    def realize(self, group: CrankGroup, build: Build) -> Realized:
        stack = _Stack(group, build, self.axial_play)
        self.joints(stack)
        self.hub(stack)
        return stack.finish()

    def chain_joint(self, c: list[Run], span) -> PostJoint | None:
        """A chain's joint, from ``span(layer)`` (the crank's faces in that layer)."""
        return self.post_joint(*span(c[0].lo - 1), *span(c[-1].hi + 1))

    def check_route(self, L: Layout, route: CrankRoute, ridden: dict[str, set[int]],
                    plates: set[int], drive: DriveInterface | None = None) -> None:
        """The plan's own z (:attr:`stack.Layout.final`): every chain's stock screw (and key)
        as :meth:`realize` will pick it, the faces set back as :class:`_Stack` sets them
        (:class:`stack.Unbuildable` if none fits)."""
        face = L.z(L.top)[1] - (drive.horn_face_depth if drive is not None else 0.0)
        below, above = set(), set()
        for r in route.runs:
            ks = ridden.get(r.at, set()) & set(range(r.lo, r.hi + 1))
            if r.lo in ks:
                below.add(r.lo - 1)
            if r.hi in ks and len(ks) <= r.hi - r.lo:
                above.add(r.hi + 1)

        def span(k: int) -> tuple[float, float]:
            z0, z1 = L.z(k)
            top = min(z1 - self.axial_play * (k in below), face if drive is not None else z1)
            return z0 + self.axial_play * (k in above), top

        for c in chains_of(sorted(route.runs, key=lambda r: (r.lo, r.at))):
            if self.chain_joint(c, span) is None:
                raise Unbuildable(f"at the plan's z no stock screw fits the crankpin joint at "
                                  f"{c[0].at} (layers {c[0].lo - 1}..{c[-1].hi + 1})")

    def joints(self, s: _Stack) -> None:
        """Runs along one point whose webs meet are one joint (:func:`chains_of`): a post on
        the segment below each run, one screw from the lowest web's head counterbore up
        through every post into a nut trapped in the highest web."""
        for c in s.chains:
            at, tag, xy, ang = c[0].at, s.tag(c), s.xy(c[0].at), s.angle(c[0].at)
            (zl0, zl1), (zu0, zu1) = s.span(c[0].lo - 1), s.span(c[-1].hi + 1)
            lo_seg, hi_seg = s.seg_of[c[0].lo - 1], s.seg_of[c[-1].hi + 1]
            joint = self.post_joint(zl0, zl1, zu0, zu1)
            if joint is None:
                raise ConstructionError(
                    f"no stock screw fits the crankpin joint at {at}: its webs' outer faces are "
                    f"{zu1 - zl0:.2f} mm apart (layers {c[0].lo - 1}..{c[-1].hi + 1}), and no "
                    f"length of {', '.join(f'M3 {sk.kind} {sk.lengths}' for sk in POST_SCREWS)} "
                    f"keeps its head, nut and tip inside those webs with {self.min_nut_engage} "
                    "mm in the nut")
            for r in c:
                s.post(r, xy, tag)
            self.clamp(s, joint, tag, xy, ang, zl0, zu1, lo_seg, hi_seg)

    def clamp(self, s: _Stack, joint: PostJoint, tag: str, xy, ang: float, zl0: float,
              zu1: float, lo_seg: int, hi_seg: int) -> None:
        """A chain's screw and nut: the bore through every segment of the chain, the head's
        counterbore in the lowest web's bottom face, the nut trap open to the top of the
        highest web's segment (a tunnel the height of the segment), screw and nut bought."""
        bore = disc(xy, self.post_bore / 2, zl0 - 1, zu1 + 1)
        head_r = (joint.screw.head_d + self.screw_fit) / 2
        nut_z = zu1 - joint.nut_depth
        for i in range(lo_seg, hi_seg + 1):
            s.cuts[i].append(bore)
        s.cuts[lo_seg].append(disc(xy, head_r, s.zspan[lo_seg][0] - 1, zl0 + joint.head_depth))
        s.cuts[hi_seg].append(_hex(xy, NUT_AF + self.nut_fit, nut_z, s.zspan[hi_seg][1] + 1, ang))
        s.holes += [(lo_seg, xy, head_r, tag, True),
                    (hi_seg, xy, (NUT_AF + self.nut_fit) / math.sqrt(3), tag, True)]
        s.buy(f"crank_screw_{tag}", screw_body(xy, joint.screw, zl0 + joint.head_depth,
                                               joint.length), joint.screw.key(joint.length))
        nut = _hex(xy, NUT_AF, nut_z, nut_z + NUT_H, ang) - disc(xy, NUT_BORE / 2, nut_z - 1,
                                                                nut_z + NUT_H + 1)
        s.buy(f"crank_nut_{tag}", nut, NUT_KEY)

    def hub(self, s: _Stack) -> None:
        """The hub: horn screws from below, the centre pocket, the step inside the plate."""
        top_seg, drive, face = s.top_seg, s.drive, s.face
        hub_bottom = min((s.span(p.layer)[0] for p in s.claimed if p.label == "crank hub"),
                         default=face)
        joint = self.horn_joint(drive, s.spec, face - hub_bottom)
        if joint is None:
            raise ConstructionError(f"no screw fits between the crank hub and the {s.spec.key} "
                                    "horn")
        sk = joint.screw
        o = s.build.xy("O")
        theta = s.angle(s.pins[0].name) + drive.pattern_angle
        bearing = face - joint.floor
        for k in range(drive.screw_count):
            a = theta + 2 * math.pi * k / drive.screw_count
            xy = tuple(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]))
            s.cuts[top_seg] += [disc(xy, drive.screw_clearance_d / 2, bearing - 1, face + 1),
                                disc(xy, drive.screw_head_d / 2, s.zspan[top_seg][0] - 1, bearing)]
            s.holes.append((top_seg, xy, drive.screw_head_d / 2, "the servo horn", True))
            s.buy(f"crank_horn_screw{k}", screw_body(xy, sk, bearing, joint.length),
                  sk.key(joint.length))
        if drive.center_head_d > 0:
            r = (drive.center_head_d + s.params.print_fit) / 2
            s.cuts[top_seg].append(disc(tuple(o), r, face - drive.center_head_h - 0.2, face + 1))
            s.holes.append((top_seg, tuple(o), r, "the servo horn", True))
        step = s.plate_bottom - s.params.margin
        if face > step + EPS:
            ring = disc(tuple(o), 2 * s.d.hub + 10, step, face + 1) - disc(
                tuple(o), drive.horn_radius, step - 1, face + 2)
            s.cuts[top_seg].append(ring)


class _Stack:
    """The printed segments of one crank under construction (:meth:`PrintedCrank.realize`):
    the layers between runs, each the union of its claimed shapes, which the joints and
    the hub cut pockets into and add posts and purchased parts to."""

    def __init__(self, group: CrankGroup, build: Build, play: float):
        self.build = build
        ctx = build.ctx
        self.params = ctx.params
        self.d = group.dims(ctx)
        topo = build.plan.topo
        self.host = topo.crank_bodies[0]
        self.drive: DriveInterface = ctx.interfaces["drive"]
        self.spec = ctx.servo
        self.plate_bottom, plate_top = build.z(build.top)
        self.face = plate_top - self.drive.horn_face_depth
        self.play = play
        self.pins = topo.axes_of("crankpin")
        self.route = route_of(build.plan.layout, self.pins)
        self.runs = sorted(self.route.runs, key=lambda r: (r.lo, r.at))
        in_run = {k for r in self.runs for k in range(r.lo, r.hi + 1)}
        for r in self.runs:
            webs = {r.lo - 1, r.hi + 1}
            if k := sorted(webs & in_run) or sorted(k for k in webs if not 0 < k < build.top):
                raise ConstructionError(f"the crank's route puts a web of {r.at} in layer {k[0]}: "
                                        "in a frame plate, or where another run needs O free")
        riders = {p.name: {build.layers[b] for b in p.members} for p in self.pins}
        self.claimed = [p for p in build.shapes(GROUP)
                        if p.label != "servo horn" and not p.label.startswith("crankpin")]

        # end play: the crank's face against a rider is set back by ``play``, once per
        # stack of riders filling a run, else on each side a rider touches
        self.below, self.above = set(), set()   # layers whose top / bottom face is set back
        for r in self.runs:
            ridden = riders.get(r.at, set()) & set(range(r.lo, r.hi + 1))
            if r.lo in ridden:
                self.below.add(r.lo - 1)
            if r.hi in ridden and len(ridden) <= r.hi - r.lo:
                self.above.add(r.hi + 1)

        # segments: the layers between runs, each printed standing on its bottom face
        ranges: list[list[int]] = []
        for k in range(build.top):
            if k not in in_run:
                if ranges and ranges[-1][1] == k - 1:
                    ranges[-1][1] = k
                else:
                    ranges.append([k, k])
        self.seg_of = {k: i for i, (a, b) in enumerate(ranges) for k in range(a, b + 1)}
        self.segs: list[list] = [[] for _ in ranges]
        self.zspan: list[list[float]] = [[math.inf, -math.inf] for _ in ranges]
        for p in self.claimed:
            if p.gap:       # a clearance gap inside a segment: printed through it
                i = self.seg_of.get(p.layer)
                if i is None or self.seg_of.get(p.layer + 1) != i or p.label.endswith("washer"):
                    continue
                z = build.plan.gap_z(p.layer)
            else:
                i, z = self.seg_of.get(p.layer), self.span(p.layer)
            if i is None or z[1] <= z[0] + EPS:
                continue
            self.segs[i].append(shape_solid(build, p, z=z))
            self.zspan[i] = [min(self.zspan[i][0], z[0]), max(self.zspan[i][1], z[1])]
        self.cuts: list[list] = [[] for _ in ranges]
        self.purchased: list = []
        self.extras: list[BomLine] = []
        self.notes: dict[str, dict] = {}
        # pockets per segment: (segment, xy, radius, owner, a tunnel the segment's height)
        self.holes: list[tuple[int, tuple, float, str, bool]] = []
        self.chains = chains_of(self.runs)

    @property
    def top_seg(self) -> int:
        return len(self.segs) - 1

    def span(self, k: int) -> tuple[float, float]:
        """The crank's faces in layer ``k``; nothing rises past the horn's outer face."""
        z0, z1 = self.build.z(k)
        return z0 + self.play * (k in self.above), min(z1 - self.play * (k in self.below),
                                                       self.face)

    def xy(self, point: str) -> tuple:
        return tuple(self.build.xy(point))

    def angle(self, point: str) -> float:
        return self.build.angle("O", point)

    def tag(self, chain: list[Run]) -> str:
        """A chain's name in part names: its point, with its lowest layer when the point
        carries more than one chain."""
        at = chain[0].at
        return at if sum(c[0].at == at for c in self.chains) == 1 else f"{at}_{chain[0].lo}"

    def post(self, r: Run, xy, tag: str) -> tuple[int, float]:
        """The post of run ``r`` on the segment below it, up to the underside of the web
        above (its riders turn on it, bare where none does): (that segment, the post's top)."""
        seg, top = self.seg_of[r.lo - 1], self.span(r.hi + 1)[0]
        self.segs[seg].append(disc(xy, self.d.post, self.span(r.lo - 1)[1] - 0.5, top))
        self.holes.append((seg, xy, self.d.post, tag, False))
        return seg, top

    def buy(self, name: str, part, bom_key: str, color: str = STEEL) -> None:
        self.purchased.append(hardware(name, part, self.host, fab="purchased", bom_key=bom_key,
                                       color=color))

    def finish(self) -> Realized:
        # a tunnel runs the height of its segment: it mustn't cut another joint's
        for (i, p, rp, a, tp), (j, q, rq, b, tq) in itertools.combinations(self.holes, 2):
            if i == j and a != b and (tp or tq) and math.dist(p, q) < rp + rq:
                raise ConstructionError(f"the crank's pockets for {a} and {b} are "
                                        f"{math.dist(p, q):.1f} mm apart in segment {i}: they "
                                        "would cut into each other's screw, nut or post")
        out = Realized()
        for i, parts in enumerate(self.segs):
            if not parts:
                continue
            solid = union(parts)
            if self.cuts[i]:
                solid = solid - union(self.cuts[i])
            solids = solid.solids()
            solid = solids[0] if len(solids) == 1 else solid
            out.bodies.append(hardware(f"crank_seg{i}", solid, self.host, fab="printed",
                                       color=SEGMENT_COLOR))
        out.bodies += self.purchased
        out.extras += self.extras
        out.notes.update(self.notes)
        for pin in self.pins:
            for b in pin.members:
                out.cut(b, Cut(self.xy(pin.name), self.params.hole(2 * self.d.post)))
        if self.route.bearing:
            out.cut(FRAME_OUTER, Cut(self.xy("O"), self.params.hole(2 * self.d.stub)))
        return out


# ---------------------------------------------------------------------------
# The keyed crankshaft (the default)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class KeyedJoint(PostJoint):
    """A keyed chain: a hex key floating between each post and the web above it, the
    chain's top web two layers thick (hex socket from below, nut trap from above)."""

    socket: float = 0.0       # hex socket, from a web's underside (the post's top)
    cavity: float = 0.0       # hex cavity, from a post's top
    key_floor: float = 0.0    # top web between the socket's ceiling and the nut's floor
    head_floor: float = 0.0   # lowest web between the head counterbore and the first cavity


@dataclass(frozen=True)
class KeyedCrank(PrintedCrank):
    """Printed crankshaft whose segments are keyed through each crankpin by brass hex
    standoffs and clamped by a screw and nut through a two-layer top web."""

    key: str = "keyed"
    label: str = ("printed crankshaft, its segments keyed through each crankpin by brass hex "
                  "standoffs pressed into both halves and clamped by a threadlocked screw and "
                  "nut in a two-layer web, bolted to the horn")
    standoff_key: str = STANDOFF_KEY   # the key (hardware/parts.py): its AF and length rule
    key_af: float | None = None  # the key's measured across-flats (None: the catalog's); the
    #                              pockets are cut to it, so measure a kit before printing
    key_fit: str = "press"       # "press": pockets cut to the key's AF + ``press_fit`` (no
    #                              rotational play); "float": + ``standoff_fit`` (slides in)
    press_fit: float = 0.0       # hex pockets over the key when pressed: FDM holes print
    #                              0.05-0.15 small, so 0.0 is a light press (see KEY_LOCK)
    standoff_fit: float = 0.15   # hex pockets over the key when floating
    lock_key: str | None = "threadlocker_222"   # on each chain screw's thread (None: dry)
    lock_per_screw: float = 0.01  # of a bottle, per chain screw
    key_float: float = 0.8       # axial room for a key threaded on the screw: the thread
    #                              pitch (0.5) + length tolerance + print error (0.3 for an
    #                              unthreaded spacer)
    min_socket: float = 1.5      # least hex engagement either side, at either end of the float
    min_key_floor: float = 1.0   # printed floor over a socket's ceiling (under the nut, or
    #                              the next post's cavity)
    pocket_chamfer: float = 0.4  # lead-in at a pocket's mouth (a step this deep and wide)
    post_d: float = 8.5          # the post, unless Params.crankpin_d is wider: 1.3 mm of wall
    #                              round the cavity's corners (8.0 leaves 1.0; the rider's ring
    #                              is 1.58 mm at link_radius 6)
    min_key_wall: float = 1.0    # least post wall at the cavity's corners
    two_layer_top: bool = True   # the route rules claim a second layer over a chain's top web

    def key_dims(self) -> tuple[float, float]:
        """(across flats, length) of the key: the catalog's, the AF overridden by ``key_af``."""
        af, length = standoff_dims(self.standoff_key)
        return (af if self.key_af is None else self.key_af), length

    def pocket_af(self) -> float:
        """Across flats of the hex pockets (the post's cavity and the web's socket)."""
        if self.key_fit not in KEY_FITS:
            raise ConstructionError(f"key_fit {self.key_fit!r}: one of {', '.join(KEY_FITS)}")
        fit = self.press_fit if self.key_fit == "press" else self.standoff_fit
        return self.key_dims()[0] + fit

    def key_play(self, print_error: float = 0.0) -> float:
        """Rotational play (deg, either way) between a post and the web above it: the key
        turns in each of its two pockets until its corners meet their flats
        (:func:`hex_play`), the pockets printed ``print_error`` over their model."""
        af = self.key_dims()[0]
        return 2 * hex_play(af, self.pocket_af() + print_error)

    def dims(self, ctx: Context) -> CrankDims:
        p: Params = ctx.params
        af = self.key_dims()[0]
        post_d = max(p.crankpin_d, self.post_d)
        corners = self.pocket_af() / math.cos(math.pi / 6)   # the cavity across corners
        if (post_d - corners) / 2 < self.min_key_wall - EPS:
            raise ConstructionError(
                f"a {post_d:g} mm crankpin leaves {(post_d - corners) / 2:.2f} mm of wall round "
                f"the {af:g} mm AF key cavity's corners ({corners:.2f} mm across), under "
                f"{self.min_key_wall:g} mm", numbers={"crankpin_d": post_d, "wall_mm":
                                                     (post_d - corners) / 2})
        with_post = replace(p, crankpin_d=post_d)
        dims = super().dims(replace(ctx, params=with_post))
        # a chain of runs one web apart: that web holds a socket from below and the next
        # post's cavity from above, with a floor between (the post is one layer + end play)
        joint = self.tightest_joint(ctx.pitch)
        floor = 2 * ctx.pitch - self.axial_play - joint.cavity - joint.socket
        if floor < self.min_key_floor - EPS:
            af, length = self.key_dims()
            raise ConstructionError(
                f"a {length:g} mm hex key leaves {floor:.2f} mm of web between one run's key "
                f"socket ({joint.socket:g} mm) and the next run's key cavity ({joint.cavity:g} "
                f"mm) in {ctx.pitch:g} mm layers, under the {self.min_key_floor:g} mm floor it "
                "needs: a shorter standoff, or less key float",
                numbers={"pitch_mm": ctx.pitch, "key_length_mm": length, "floor_mm": floor})
        return dims

    def tightest_joint(self, pitch: float) -> KeyedJoint | None:
        """One rider between a one-layer web and a two-layer top web, both set back."""
        return self.post_joint(0.0, pitch - self.axial_play, 2 * pitch,
                               4 * pitch - self.axial_play, first_post_top=2 * pitch)

    def _pitch_why(self, pitch: float) -> tuple[str, str]:
        af, length = self.key_dims()
        why = (f"no M3 screw, nut and {length:g} mm hex key fit a keyed crankpin joint in "
               f"{pitch:g} mm layers: a chain's lowest web is one layer thick "
               f"({pitch - self.axial_play:g} mm beside a link, after {self.axial_play:g} mm of "
               "end play) and must hold a "
               f"screw head under {self.min_web_floor:g} mm of floor, and its top web, two "
               f"layers, a {self.min_socket:g} mm key socket, {self.min_key_floor:g} mm of "
               f"floor and a nut (the flattest head is {min(sk.head_h for sk in POST_SCREWS):g} "
               f"mm, the nut {NUT_H:g} mm)")
        return why, ("a stock M3 screw head in a one-layer web, a key socket and nut in a "
                     "two-layer one")

    def post_joint(self, zl0: float, zl1: float, zu0: float, zu1: float,
                   first_post_top: float | None = None) -> KeyedJoint | None:
        """The screw and the keys of a chain: lowest web ``zl0..zl1`` (head from below), top
        web ``zu0..zu1``, two layers (key socket from below, nut trap from above).
        ``first_post_top`` is the top of the chain's first post (the underside of the web
        above its first run): that post's cavity must leave the head counterbore its floor;
        ``zu0`` when not given (a one-run chain).

        The socket is as deep as the top web leaves under the nut and the key floor (at
        most :data:`SOCKET_MAX`), the cavity takes the rest of the key plus its float, and
        the key must engage ``min_socket`` either side at both ends of the float.
        """
        if first_post_top is None:
            first_post_top = zu0
        _, length = self.key_dims()
        sd = min(SOCKET_MAX, (zu1 - zu0) - self.min_key_floor - NUT_H - self.head_recess)
        cd = length - sd + self.key_float
        if min(sd, length - cd, length - sd) < self.min_socket - EPS:
            return None
        hp_hi = min((zl1 - zl0) - self.min_web_floor,
                    first_post_top - cd - self.min_web_floor - zl0)
        screw = self._screw(zl0, zu1, hp_hi, NUT_H + self.head_recess,
                            (zu1 - zu0) - sd - self.min_key_floor)
        if screw is None:
            return None
        sk, L, hp, nd, engage = screw
        return KeyedJoint(sk, L, hp, nd, engage, sd, cd, (zu1 - nd) - (zu0 + sd),
                          (first_post_top - cd) - (zl0 + hp))

    def chain_joint(self, c: list[Run], span) -> KeyedJoint | None:
        return self.post_joint(*span(c[0].lo - 1), span(c[-1].hi + 1)[0],
                               span(c[-1].hi + 2)[1], span(c[0].hi + 1)[0])

    def joints(self, s: _Stack) -> None:
        """A key in every post, its socket in the web above; the chain's screw and nut as the
        printed crank's, the nut in the two-layer top web."""
        af, length = self.key_dims()
        af_fit, lead = self.pocket_af(), self.pocket_chamfer
        depths: list[tuple[float, float]] = []
        for c in s.chains:
            at, tag, xy, ang = c[0].at, s.tag(c), s.xy(c[0].at), s.angle(c[0].at)
            zl0, zl1 = s.span(c[0].lo - 1)
            zu0, zu1 = s.span(c[-1].hi + 1)[0], s.span(c[-1].hi + 2)[1]
            lo_seg, hi_seg = s.seg_of[c[0].lo - 1], s.seg_of[c[-1].hi + 1]
            if s.seg_of.get(c[-1].hi + 2) != hi_seg:
                raise ConstructionError(f"the two-layer top web of {at} needs layer "
                                        f"{c[-1].hi + 2}, where another run needs O free")
            joint = self.post_joint(zl0, zl1, zu0, zu1, s.span(c[0].hi + 1)[0])
            if joint is None:
                raise ConstructionError(
                    f"no stock screw, nut and {length:g} mm hex key fit the keyed crankpin "
                    f"joint at {at}: its webs' outer faces are {zu1 - zl0:.2f} mm apart (layers "
                    f"{c[0].lo - 1}..{c[-1].hi + 2}, the top web two layers thick), and no "
                    f"length of {', '.join(f'M3 {sk.kind} {sk.lengths}' for sk in POST_SCREWS)} "
                    f"keeps its head, nut and tip inside those webs with {self.min_nut_engage} "
                    f"mm in the nut over a {self.min_socket:g} mm key socket")
            sd, cd = joint.socket, joint.cavity
            depths.append((sd, cd))
            tops: list[tuple[Run, float]] = []
            for r in c:
                post_seg, post_top = s.post(r, xy, tag)
                web_seg = s.seg_of[r.hi + 1]
                # the hex cavity in the post's top and the blind hex socket in the web's
                # underside, each with a lead-in step at its mouth; the key bottomed in the
                # cavity, ``key_float`` short of the socket's ceiling
                s.cuts[post_seg] += [_hex(xy, af_fit, post_top - cd, post_top + 1, ang),
                                     _hex(xy, af_fit + 2 * lead, post_top - lead, post_top + 1,
                                          ang)]
                s.cuts[web_seg] += [_hex(xy, af_fit, post_top - 1, post_top + sd, ang),
                                    _hex(xy, af_fit + 2 * lead, post_top - 1, post_top + lead,
                                         ang)]
                s.holes.append((web_seg, xy, (af_fit + 2 * lead) / math.sqrt(3), tag, False))
                key = _hex(xy, af, post_top - cd, post_top - cd + length, ang) - disc(
                    xy, NUT_BORE / 2, post_top - cd - 1, post_top - cd + length + 1)
                s.buy(f"crank_key_{tag}_{r.lo}", key, self.standoff_key, BRASS)
                tops.append((r, post_top))
            for (ra, a), (rb, b) in itertools.pairwise(tops):
                floor = (b - cd) - (a + sd)
                if floor < self.min_key_floor - EPS:
                    raise ConstructionError(
                        f"the web of {at} between its runs at layers {ra.hi} and {rb.lo} leaves "
                        f"{floor:.2f} mm between the key socket below and the key cavity above "
                        f"(at least {self.min_key_floor:g} mm)")
            self.clamp(s, joint, tag, xy, ang, zl0, zu1, lo_seg, hi_seg)
            if self.lock_key is not None:
                s.extras.append(BomLine(self.lock_key, self.lock_per_screw,
                                        f"crank screw {tag}"))
        s.notes["crank_key"] = {
            "fit": self.key_fit, "key_af_mm": af, "pocket_af_mm": round(af_fit, 3),
            "keys": sum(len(c) for c in s.chains), "play_deg": round(self.key_play(), 2),
            "play_deg_if_0p05_big": round(self.key_play(0.05), 2),
            "threadlocker": self.lock_key is not None, "key_length_mm": length,
            "chamfer_mm": lead,
            # the hex engaged at worst (the key floats between its cavity's floor and its
            # socket's ceiling): in the web's socket, in the post's cavity
            "socket_engaged_mm": round(min((length - c for _, c in depths), default=0.0), 3),
            "cavity_engaged_mm": round(min((length - d for d, _ in depths), default=0.0), 3)}


KEYED_FLOAT = KeyedCrank(
    key="keyed_float", key_fit="float", lock_key=None,
    label=("printed crankshaft keyed through each crankpin by floating brass hex standoffs "
           "(a sliding fit, 6.25 deg of play per interface) and clamped by a dry screw and nut "
           "in a two-layer web, bolted to the horn"))
"""The keyed crank as it was before the keys were pressed in (``--crank keyed_float``)."""


# ---------------------------------------------------------------------------
# The bolt crankshaft: laser-cut plate stacks on M6 hex-bolt crankpins
# ---------------------------------------------------------------------------

BOLT_COLOR = "#8fb3e0"           # the crank's acrylic plates (the study's blue)
HEX_RELIEF_D = 0.5               # corner relief circles of a laser-cut hex pocket (mm)


def hex_pocket(xy, af: float, z0: float, z1: float, angle: float, relief: float = HEX_RELIEF_D):
    """A laser-cut hex pocket ``af`` across flats with a ``relief`` circle at each corner (a
    laser can't cut a sharp inside corner, and a sharp corner in acrylic starts a crack)."""
    cut = _hex(xy, af, z0, z1, angle)
    r = af / math.sqrt(3)
    for i in range(6):
        a = angle + math.pi / 6 + i * math.pi / 3
        c = (float(xy[0]) + r * math.cos(a), float(xy[1]) + r * math.sin(a))
        cut = cut + disc(c, relief / 2, z0, z1)
    return cut


def hex_bearing_nm(af: float, engaged: float, p: float = 50.0, relief: float = 0.0) -> float:
    """The twist (N·m) a hex ``af`` across flats carries in a socket of a plastic (printed PLA,
    acrylic) over ``engaged`` mm when the pressure on each flat's loaded half reaches ``p``
    everywhere (fully plastic): per flat ``p L (a/2)^2 / 2`` about the flat's midpoint, ``a =
    af / sqrt 3`` its length less ``relief`` (the corner reliefs take that off a flat), six
    flats: ``0.75 p a^2 L``. An M3 nut (5.5 AF) in a 2.4 mm printed trap: 0.9 N·m, about
    where printed nut traps are reported to spin out. The one bearing model for every
    hex-in-socket joint (:mod:`spiderpig.strength`)."""
    a = max(af / math.sqrt(3) - relief, 0.0)
    return 0.75 * p * a * a * max(engaged, 0.0) / 1e3


@dataclass(frozen=True)
class BoltJoint:
    """One chain's bolt: z measured up from the bottom face of its lowest web stack."""

    bolt: str            # catalog key (the stock length)
    length: float        # stock length under the head
    cut: float           # length after cutting the tip (== length: not cut)
    plain: float         # nominal plain shank, ``lg = L - b``
    sink: float          # the nut's bottom below the stack's bottom face
    head_engaged: float  # hex of the head in its pocket
    nut_engaged: float   # hex of the nut in its pocket
    tip_out: float       # thread past the nut's bottom face (into the tip layer)


@functools.cache
def _pin_lengths() -> tuple[tuple[float, ...], ...]:
    """A crankpin's standoffs, shortest first: one stock goBILDA 1501 length, or two (each
    at least 12 mm: thread for the stud and a screw) joined by an M4 set screw."""
    from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS

    one = [(float(a),) for a in GOBILDA_LENGTHS]
    two = [(float(a), float(b)) for a in GOBILDA_LENGTHS for b in GOBILDA_LENGTHS
           if a >= b >= 12 and a + b > max(GOBILDA_LENGTHS)]
    return tuple(sorted(one + two, key=lambda c: (sum(c), len(c))))


@dataclass(frozen=True)
class WebJoint:
    """One crankpin between single-plate webs (:meth:`BoltCrank.fit_web`): a round standoff
    clamped between its two webs by an M4 screw into each end."""

    standoff: str        # catalog key (a stock length)
    length: float
    shims: float         # DIN 988 shims under its lower end (mm)
    gap: float           # the clearance gap over the lowest web its length needs (mm)
    screw_lo: str        # the M4 screw up through the lowest web (and the shims)
    engage_lo: float
    screw_hi: str        # the M4 screw down through the top web
    engage_hi: float
    segments: tuple[float, ...] = ()   # stock lengths, bottom up (two: joined by a stud)


@dataclass(frozen=True)
class BoltCrank:
    """Laser-cut acrylic crank: every web a stack of two plates, each crankpin an M6 hex bolt.

    The construction the crank study of 2026-10-03 recommends ("C+"), with what the layer
    model made of it:

    * **plates**: every crank layer that isn't a run (the webs, the journal on O, the hub)
      is one laser-cut plate from the link sheet, its outline the crank's claimed shapes in
      that layer (:mod:`layout` puts them on the DXF sheets). A chain's lowest and highest
      webs are two plates each (``two_layer_bottom`` / ``two_layer_top``, which the
      router's rules know); a chain's lowest stack may share a plate with the chain below's
      top stack (the **mid stack**: that plate holds one pin's head pocket and the next
      pin's nut pocket). The plates between two runs (a segment) are solvent-welded into
      one stack (acrylic cement, which the BOM lists); a plate's interface with the next
      is the bond the strength check counts.
    * **crankpin**: an ISO 4014 M6 hex bolt (:mod:`hardware.crank_catalog`), its head (10
      AF x 4 mm) in a hex pocket through the chain's top stack, the riders on its plain
      shank, a DIN 985 M6 nylock (10 AF x 6 mm) in a hex pocket through the lowest stack,
      flush with its bottom face, threadlocked (``lock_key``, medium strength). The thread
      past the nut stands in the layer under the stack (the **tip layer**, claimed at the
      pin: ``tip``). Pockets are cut ``pocket_fit`` over the AF with ``relief`` circles at
      the corners. The head and the nut key the stacks to the bolt (the twist goes head ->
      shank -> thread -> nut); the bolt carries no preload: its head bears on the top
      rider's face, the nut sits where the plan puts its stack (the nylock and the
      threadlocker hold it on the thread; the pockets hold it against turning), and the
      plates and the riders keep their layers between the frame plates as every link does.
    * **plain shank**: the riders must turn on the bolt's full-diameter shank, which ends
      ``runout`` (ISO 3508: 2.5 pitches) above the nominal thread start ``L - 18``, and the
      nut must sit on complete thread, so a rider can't sit right on the nut's stack: at
      least one layer of the run at its bottom is bare (no rider of its pin), and how many
      a bolt needs depends on the stock length (``JointRules.low_count``; :meth:`fit`).
      Stock lengths come in 5 mm steps against 3 mm layers, so a chain's span is one some
      stock length fits (cut to length at its tip when longer).
    * **journal stub**: a round M3 F-F standoff (6 mm OD) screwed to the lowest stack by
      an M3 button head recessed in its upper plate, turning in the outer frame plate's
      hole; stock lengths, so the lowest stack's layers are a route rule
      (``JointRules.bottom_layers``). Without the bearing (``drop_bearing``): none.
    * **hub**: the top two plates (``hub_thickness``) on the horn's screw pattern: the
      horn screws' heads in the lower plate, their shanks through the upper plate and the
      drive's horn spacer into the horn. A laser plate is a whole layer, so the horn's face
      must sit on a layer boundary: the drive's printed horn spacer is sized for it
      (``face_on_layer``, :meth:`servos.mount.DriveGroup.spacer`).

    What it doesn't build, against the study: no PTFE thrust washers (a 0.5 mm washer
    between a plate and a rider has no room where plates touch, 3 mm sheet in 3 mm layers;
    the riders turn against the acrylic and the head's and nut's faces, as links on a pin
    turn against their neighbours), and no Chicago screws locking the stacks (their 8 x
    1.5 mm heads would stand into a rider's layer, which sweeps them): the stacks are
    solvent-welded, and the head and nut pockets run through both plates of their stack,
    so the twist enters each plate directly.

    Assembly (the servo side up, as the printed crank): cement each segment's plates
    (align them on the bolts' pockets and a 6 mm rod through the pin holes; acrylic cement
    by capillary, 24 h); screw the stub standoff to the lowest stack (button head from
    above, threadlocker); the hub stack to the horn (the horn spacer between, screws from
    the hub's underside); then going down, per chain: a bolt through the top stack's
    pocket (head in its hex), the riders onto the shank (and the bare layers' links past
    it, the plan's order), a drop of medium threadlocker on the thread, the nylock from
    below with a 10 mm spanner to its height (the lowest stack held against the riders as a
    gauge: the nut's bottom face flush with the stack's), then turned to the nearest flat
    (1/6 turn is 0.17 mm) and the stack pushed up over it, its pocket on the nut's hex; the
    riders must still turn. A bolt cut to length: cut before assembly, chase the thread
    with a nut. The outer frame plate last, over the stub. Apart: heat the nut
    (threadlocker), unscrew it, the stack off first.
    """

    key: str = "bolt"
    label: str = ("laser-cut acrylic crank: two-plate web stacks keyed on M6 hex-bolt "
                  "crankpins (head and nylock in hex pockets), a round standoff stub, bolted "
                  "to the horn")
    axial_play: float = 0.0       # plates touch (laser sheet in its layer)
    bolt_d: float = 6.0
    shank_model: float = 5.9      # the modelled shank (inside its claim, clear of a 6.4 bore)
    bore: float = 6.4             # a plate's hole for the shank or thread (ISO 273 medium)
    pocket_fit: float = 0.1       # hex pockets over the head's and nut's AF (a slide fit)
    relief: float = HEX_RELIEF_D  # corner relief circles
    runout: float = 2.5           # incomplete thread above the nominal thread start (ISO 3508)
    min_tip: float = 1.5          # thread past the nylock's bottom (the insert needs it)
    tip_recess: float = 0.3       # the tip stays this far inside the tip layer
    max_sink: float = 1.0         # how far the nut may stand below its stack, into the tip
    #                               layer (its hex then engages 5 of its 6 mm)
    head_gap: float = 0.05        # the head's underside above its stack's bottom face
    nut_key: str = "m6_nylock"
    lock_key: str | None = "threadlocker_243"
    lock_per_bolt: float = 0.01
    cement_per_plate: float = 0.01   # acrylic cement per plate, a fraction of the bottle
    stub_screw_engage: float = 2.5   # least thread of the stub screw in the standoff (5
    #                                  turns of M3; 3.0 before the 3.175 mm aluminium plates)
    stub_seat: float = 1.5           # least of the stub in the outer frame plate
    bond_mpa: float = 5.0            # solvent-welded acrylic in shear (UNVERIFIED, low end)
    hex_mpa: float = 50.0            # acrylic crushed by a hex's flats
    face_on_layer: bool = True       # the drive's horn face on a layer boundary
    two_layer_top: bool = True
    two_layer_bottom: bool = True
    plates: bool = True              # laser-cut plates (the crank's sheet, config.crank_sheet)
    washer_r: float = 6.0            # PTFE washers / shims (6 x 12) on a crankpin through a gap
    web_t: float = 3.175             # (single webs) the crank sheet's thickness, set by resolve
    stub_below: float = 2.5          # (single webs) the stub may stand this far out under the
    #                                  outer frame plate (a thin plate leaves a stock length
    #                                  too little room to end inside it); claimed in layer -1
    # -- single-plate webs (:meth:`resolve`: a metal crank sheet, the joinery plan) --------
    single: bool = False             # one plate per web (resolved from the crank's sheet)
    webs: str = "auto"               # "auto": single on a metal sheet; "stack": two plates
    pin_od: float = 6.0              # the single webs' crankpin: a goBILDA 1501 round standoff
    pin_hole: float = 4.5            # a web's hole for its M4 screw
    pin_min_engage: float = 2.8      # least M4 thread in a standoff's end (4 turns)
    pin_preload_n: float = 2200.0    # an M4 button head at about 2 N·m into the standoff
    pin_mu: float = 0.3              # the standoff's end face on a web (anodised on bare
    #                                  aluminium, dry; UNVERIFIED: the test-build checklist)
    head_mu: float = 0.2             # the screw head on the web
    pin_lock_key: str = "threadlocker_243"
    web_edge_t: float = 1.0          # a web's wall round a hole, in sheet thicknesses
    head_clear: float = 0.3          # z clearance over a head in its gap

    # -- the sheet decides the webs -----------------------------------------------------

    def resolve(self, ctx: Context) -> BoltCrank:
        """This crank for ``ctx``'s crank sheet (:meth:`for_sheet`)."""
        c = self.for_sheet(ctx.sheet("crank"))
        return replace(c, web_t=ctx.sheet_t("crank")) if c.single else c

    def for_sheet(self, key: str | None) -> BoltCrank:
        """On a metal sheet (the joinery plan's 0.125 in 5052) every web is **one plate** and
        every crankpin a goBILDA 1501 round standoff (6 mm OD, a stock length within 1-2 mm)
        clamped between its two webs by an M4 button head into each end, its heads in the
        clearance gaps over and under the chain (:class:`stack.Placed`). The M6 bolt the
        joinery plan names doesn't fit single webs in stock lengths: its 18 mm thread
        (ISO 4014 ``b``) would leave the bottom 4-5 layers of every run without a rider
        unless cut, and nothing here is cut. Two acrylic plates per web otherwise
        (``webs="stack"``, or an acrylic crank sheet: the designs before 2026-10-04)."""
        if self.single or self.webs == "stack" or key is None:
            return self
        from spiderpig.materials import sheet

        if self.webs != "single" and not sheet(key).metal:
            return self
        return replace(
            self, single=True, two_layer_top=False, two_layer_bottom=False,
            lock_key=self.pin_lock_key, cement_per_plate=0.0,
            label=("laser-cut aluminium crank: one plate per web, each crankpin a round "
                   "standoff clamped between its webs by M4 screws, a round standoff journal, "
                   "bolted to the horn through its top plates"))

    # -- parts ------------------------------------------------------------------------

    def bolt_lengths(self) -> tuple[float, ...]:
        from spiderpig.hardware.crank_catalog import M6_BOLT_LENGTHS

        return M6_BOLT_LENGTHS

    def head(self) -> tuple[float, float]:
        """(across flats, height) of the bolt's head."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import m6_bolt

        d = get(m6_bolt(self.bolt_lengths()[0])).dims
        return float(d["head_af"]), float(d["head_h"])

    def thread_b(self) -> float:
        from spiderpig.hardware.crank_catalog import M6_THREAD_B

        return M6_THREAD_B

    def nut(self) -> tuple[float, float]:
        from spiderpig.hardware.catalog import get

        d = get(self.nut_key).dims
        return float(d["af"]), float(d["h"])

    def pocket_af(self) -> float:
        return max(self.head()[0], self.nut()[0]) + self.pocket_fit

    def pocket_radius(self) -> float:
        """The pocket's reach from the pin: a corner plus its relief."""
        return self.pocket_af() / math.sqrt(3) + self.relief / 2

    # -- dimensions and rules -----------------------------------------------------------

    def dims(self, ctx: Context) -> CrankDims:
        if self.single:
            return self._web_dims(ctx)
        p: Params = ctx.params
        drive: DriveInterface = ctx.interfaces["drive"]
        pitch = ctx.pitch
        web = max(p.web_radius, math.ceil((self.pocket_radius() + p.min_wall) * 10) / 10)
        hub = max(drive.horn_radius, drive.screw_pcd / 2 + drive.screw_head_d / 2 + p.min_wall)
        dims = CrankDims(web=web, journal=p.journal_d / 2, stub=self.stub_od() / 2,
                         post=self.bolt_d / 2, hub=hub, hub_thickness=2 * pitch - 1e-3,
                         tip=(self.nut()[0] / math.sqrt(3) if self.max_sink > 0
                              else self.bolt_d / 2))
        if p.hole(self.bolt_d) / 2 + p.min_wall > p.link_radius + EPS:
            raise ConstructionError(
                f"an M6 crankpin's hole leaves less than {p.min_wall} mm of the rider around it "
                f"(link radius {p.link_radius})")
        if dims.journal > dims.web:
            raise ConstructionError("the crank body on O must not be wider than a web")
        if not self._pitch_ok(pitch):
            raise PrintedCrank._pitch_error(self, ctx, pitch)
        face = (drive.horn_face_depth - ctx.sheet_t("frame")) / pitch
        if abs(face - round(face)) > 1e-6:
            raise ConstructionError(
                f"the bolt crank's hub plates need the horn's face on a layer boundary; it is "
                f"{drive.horn_face_depth:g} mm under the inner plate's top face in {pitch:g} "
                "mm layers (the drive's horn spacer sizes for it: DriveGroup.spacer)")
        if self.horn_joint(ctx) is None:
            raise ConstructionError(f"no screw fits between the bolt crank's hub plates and the "
                                    f"{ctx.servo.key} horn")
        if not self.stub_layers(pitch):
            raise ConstructionError(f"no stock stub standoff fits {pitch:g} mm layers")
        return dims

    def fit(self, run_layers: int, low: int, pitch: float) -> BoltJoint | None:
        """The bolt of a chain whose stacks are ``run_layers`` apart (the runs and inner webs
        between them), ``low`` layers at the bottom of its run bare (no rider; 3 means at
        least 3): the stock length whose shank's full diameter covers every rider, whose
        complete thread carries the whole nut (flush in its stack, or sunk at most
        ``max_sink``) and whose tip passes the nut by ``min_tip`` inside the tip layer (cut
        to length if the stock is longer). Uncut first, then the nut highest, then shortest."""
        if run_layers < 1:
            return None
        return self.fit_z((2 + run_layers) * pitch, (2 + low) * pitch if run_layers > low
                          else None, 2 * pitch, pitch)

    def fit_z(self, z_h: float, z_rb: float | None, nut_stack: float,
              tip_room: float) -> BoltJoint | None:
        """:meth:`fit` at a plan's own z, measured up from the lowest stack's bottom face:
        ``z_h`` the top stack's bottom face (the head's underside, less ``head_gap``),
        ``z_rb`` the lowest rider's bottom face (``None``: no rider), ``nut_stack`` the
        lowest stack's height and ``tip_room`` how far the layer under it (and the gap
        between, if any) reaches below that face."""
        head_af, head_h = self.head()
        _, nut_h = self.nut()
        b = self.thread_b()
        z_h = z_h + self.head_gap                             # the head's underside
        best: BoltJoint | None = None
        for L in self.bolt_lengths():
            lg = L - b
            if z_rb is not None and lg - self.runout < z_h - z_rb - EPS:
                continue
            sink = max(0.0, nut_h - (z_h - lg), nut_h - nut_stack)
            if sink > self.max_sink + EPS:
                continue
            lo, hi = z_h + sink + self.min_tip, z_h + tip_room - self.tip_recess
            if lo - EPS > L or lo > hi + EPS:
                continue
            cut = min(L, hi)
            j = BoltJoint(self._key(L), L, cut, lg, sink, head_h,
                          min(nut_h - sink, nut_stack), cut - z_h - sink)
            rank = (j.cut < j.length, j.sink, j.length)
            if best is None or rank < (best.cut < best.length, best.sink, best.length):
                best = j
        return best

    def chain_fit(self, L: Layout, lo: int, hi: int, low: int) -> BoltJoint | None:
        """The bolt of a chain over runs ``lo``..``hi`` (``low`` bare layers at the bottom)
        at the layout's z (:meth:`fit_z`)."""
        zb = L.z(lo - 2)[0]
        n_run = hi - lo + 1
        return self.fit_z(L.z(hi + 1)[0] - zb, L.z(lo + low)[0] - zb if low < n_run else None,
                          L.z(lo - 1)[1] - zb, zb - L.z(lo - 3)[0])

    def check_route(self, L: Layout, route: CrankRoute, ridden: dict[str, set[int]],
                    plates: set[int], drive: DriveInterface | None = None) -> None:
        """The plan's own z (:attr:`stack.Layout.final`): every chain's bolt, the stub and
        the horn's face on a layer boundary, as the parts will be built
        (:class:`stack.Unbuildable` if not)."""
        if self.single:
            return self._web_check(L, route, plates, drive)
        for ch in chains_of(route.runs):
            lo, hi = ch[0].lo, ch[-1].hi
            ks = ridden.get(ch[0].at, set()) & set(range(lo, hi + 1))
            n_run = hi - lo + 1
            low = min(3, next((i for i in range(n_run) if lo + i in ks), n_run))
            if self.chain_fit(L, lo, hi, low) is None:
                raise Unbuildable(f"at the plan's z no stock M6 bolt fits the crankpin chain "
                                  f"at {ch[0].at} (layers {lo - 2}..{hi + 2})")
        if route.bearing and plates:
            lowest = min(plates)
            if self.stub_z(L.z(lowest)[0] - L.z(0)[0], L.t(0), L.t(lowest)) is None:
                raise Unbuildable(f"at the plan's z no stock stub standoff reaches the outer "
                                  f"frame plate from the crank's lowest stack (layer {lowest})")

    def _key(self, length: float) -> str:
        from spiderpig.hardware.crank_catalog import m6_bolt

        return m6_bolt(length)

    def stub_od(self) -> float:
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS, m3_round_standoff

        return float(get(m3_round_standoff(M3_ROUND_STANDOFF_LENGTHS[0])).dims["od"])

    def stub(self, lowest: int, pitch: float) -> tuple[str, float, float, float] | None:
        """The stub standoff under a lowest stack starting in layer ``lowest``: (key, length,
        its bottom end's z, the screw's length); the longest stock length that ends inside
        the outer frame plate at least ``stub_seat`` deep, and a button head that engages it
        ``stub_screw_engage`` (its head recessed in the stack's upper plate)."""
        return self.stub_z(lowest * pitch, pitch, pitch)

    def stub_z(self, top: float, plate: float, upper: float
               ) -> tuple[str, float, float, float] | None:
        """:meth:`stub` at a plan's own z: ``top`` the lowest stack's bottom face over the
        outer frame plate's bottom face, ``plate`` that plate's thickness, ``upper`` the
        lowest stack's plate the screw passes."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS, m3_round_standoff

        below = self.stub_below if self.single else 0.0
        for S in sorted(M3_ROUND_STANDOFF_LENGTHS, reverse=True):
            z0 = top - S
            if not -below - EPS <= z0 <= plate - self.stub_seat + EPS:
                continue
            depth = float(get(m3_round_standoff(S)).dims["thread_depth"])
            for L in BHCS["3"].lengths:
                e = L - upper
                if self.stub_screw_engage - EPS <= e <= depth + EPS:
                    return m3_round_standoff(S), S, z0, L
        return None

    def stub_layers(self, pitch: float, most: int = 64) -> frozenset[int]:
        """The layers a chain's lowest stack may start in, with the bottom bearing."""
        return frozenset(k for k in range(2, most) if self.stub(k, pitch) is not None)

    def horn_joint(self, ctx: Context) -> tuple[ScrewKind, float, float] | None:
        """The horn screws: (kind, length, thread in the horn). The head bears on the hub's
        upper plate (its floor, one layer), in a hole through the lower plate; the shank
        passes the floor and the drive's horn spacer into the horn. Most thread (up to the
        pattern's depth), then shortest; a button head first (it sits lower in its hole)."""
        drive: DriveInterface = ctx.interfaces["drive"]
        spec = ctx.servo
        pat = spec.horn.pattern
        from spiderpig.hardware.fasteners import SCREWS

        size = SIZES.get(pat.thread)
        order = ("self_tap",) if pat.tapping else ("bhcs", "shcs")
        kinds = [SCREWS[(k, size)] for k in order if (k, size) in SCREWS]   # every stock length
        if not kinds:
            return None
        spacer = drive.horn_face_depth - spec.horn_face_depth
        reach = pat.reach if pat.reach is not None else spec.horn.thickness
        e_max = min(reach, spec.horn_face_depth)
        e_want = min(pat.thread_depth if pat.thread_depth is not None else e_max, e_max)
        e_min = min(e_want, 1.5)
        for sk in kinds:
            if sk.head_h > ctx.pitch + EPS:
                continue
            best = None
            for L in sk.lengths:
                e = L - ctx.pitch - spacer
                if not e_min - EPS <= e <= e_max + EPS:
                    continue
                if best is None or min(e, e_want) > min(best[2], e_want) + EPS:
                    best = (sk, L, e)
            if best is not None:
                return best
        return None

    def joint_rules(self, ctx: Context, dims: CrankDims):
        """The router's rules (:class:`construction.route.JointRules`): every span a stock
        bolt fits, by how many layers at the bottom of the run are bare; the pockets; the
        stub's layers."""
        from spiderpig.construction.route import JointRules, horn_pockets

        if self.single:
            return self._web_rules(ctx, dims)
        p = ctx.pitch
        spans: dict[int, int] = {}
        for n in range(5, 64):
            m = 0
            for low in range(4):
                if self.fit(n - 4, low, p) is not None:
                    for a in (0, 1):
                        for c in (0, 1):
                            for d in (0, 1):
                                m |= 1 << (16 * a + 4 * low + 2 * c + d)
            spans[n] = m
        r = self.pocket_radius()
        return JointRules(spans, head=r, nut=r, post=dims.post, horn=horn_pockets(ctx),
                          hub_play=True, two_layer_top=True, two_layer_bottom=True, tip=True,
                          low_count=True, inner_webs=False, share_stack=True,
                          bottom_layers=self.stub_layers(p))

    def _pitch_ok(self, pitch: float) -> bool:
        """The head and the nylock fit their two-plate stacks, and some stock bolt a chain."""
        _, head_h = self.head()
        _, nut_h = self.nut()
        if self.single:
            return any(self._web_span_ok(r, low, pitch, pitch) for r in range(2, 16)
                       for low in range(4))
        return (head_h + self.head_gap <= 2 * pitch + EPS
                and nut_h <= 2 * pitch + self.max_sink + EPS
                and any(self.fit(r, low, pitch) for r in range(2, 12) for low in range(4)))

    def _pitch_why(self, pitch: float) -> tuple[str, str]:
        _, head_h = self.head()
        _, nut_h = self.nut()
        why = (f"no M6 hex bolt and nylock fit a bolt crank joint in {pitch:g} mm layers: the "
               f"head ({head_h:g} mm) and the nylock ({nut_h:g} mm) sit in hex pockets through "
               "two-plate stacks")
        return why, "an M6 hex head and nylock in two-plate stacks"

    def least_pitch(self, pitch: float, limit: float = 8.0) -> float | None:
        p = round(math.ceil(pitch / 0.05 - EPS) * 0.05, 6)
        while p <= limit + EPS:
            if self._pitch_ok(p):
                return round(p, 2)
            p = round(p + 0.05, 6)
        return None

    # -- the strength check's capacities ------------------------------------------------

    def capacity(self, joint: BoltJoint | None = None) -> dict[str, float]:
        """What one crankpin joint holds (N·m), per element, for :mod:`spiderpig.strength`:
        the head's and the nut's hex bearing in the acrylic (:func:`hex_bearing_nm` at
        ``hex_mpa``, the flats shortened by the corner reliefs), the thread's torsional
        yield (stress diameter), and the nut's lock on its thread (the nylock's prevailing
        torque plus the threadlocker's breakaway, from M10's by thread area x radius,
        halved for plated steel). ``joint``: the built one (its engagement), else a flush
        nut."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import m6_bolt

        if self.single:
            return self._web_capacity(joint)
        bolt = get(joint.bolt if joint else m6_bolt(self.bolt_lengths()[0])).dims
        nut = get(self.nut_key).dims
        head_af, head_h = self.head()
        nut_af, nut_h = self.nut()
        head_eng = joint.head_engaged if joint else head_h
        nut_eng = joint.nut_engaged if joint else nut_h
        ds, sy = float(bolt["stress_d"]), float(bolt["yield_mpa"])
        torsion = sy / math.sqrt(3) * math.pi * ds ** 3 / 16 / 1e3
        lock = float(nut.get("prevailing_nm", 0.0))
        lock_note = "nylock prevailing"
        if self.lock_key is not None:
            m10 = float(get(self.lock_key).dims.get("breakaway_m10_nm", 0.0))
            metal = float(nut.get("metal_h", nut_h))
            lock += 0.5 * m10 * (self.bolt_d / 10.0) ** 2 * (metal / 8.4)
            lock_note = "nylock prevailing + threadlocker breakaway (half: plated steel)"
        return {
            f"head pocket, {head_eng:g} mm of 10 AF in acrylic": round(
                hex_bearing_nm(head_af, head_eng, self.hex_mpa, self.relief), 3),
            f"nut pocket, {nut_eng:g} mm of 10 AF in acrylic": round(
                hex_bearing_nm(nut_af, nut_eng, self.hex_mpa, self.relief), 3),
            f"M6 thread torsion ({sy:g} MPa)": round(torsion, 3),
            f"nut lock ({lock_note})": round(lock, 3),
        }

    # -- single-plate webs ----------------------------------------------------------------

    def pin_screw(self) -> tuple[float, float]:
        """(head diameter, head height) of the crankpins' M4 button heads."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M4_BHCS_LENGTHS, m4_bhcs

        d = get(m4_bhcs(M4_BHCS_LENGTHS[0])).dims
        return float(d["head_d"]), float(d["head_h"])

    def head_r(self) -> float:
        """A crankpin screw head's clearance shape."""
        return self.pin_screw()[0] / 2 + 0.3

    def _web_dims(self, ctx: Context) -> CrankDims:
        p: Params = ctx.params
        drive: DriveInterface = ctx.interfaces["drive"]
        t = ctx.sheet_t("crank")
        wall = max(p.min_wall, self.web_edge_t * t)
        web = max(p.web_radius, math.ceil((self.pin_hole / 2 + wall) * 10) / 10,
                  self.head_r())
        hub = max(drive.horn_radius, drive.screw_pcd / 2 + drive.screw_head_d / 2 + p.min_wall)
        dims = CrankDims(web=web, journal=self.pin_od / 2, stub=self.stub_od() / 2,
                         post=self.pin_od / 2, hub=hub, hub_thickness=t - 1e-3,
                         tip=self.head_r())
        if p.hole(self.pin_od) / 2 + p.min_wall > p.link_radius + EPS:
            raise ConstructionError(
                f"a {self.pin_od:g} mm crankpin's hole leaves less than {p.min_wall} mm of the "
                f"rider around it (link radius {p.link_radius})")
        if dims.journal > dims.web:
            raise ConstructionError("the crank body on O must not be wider than a web")
        face = (drive.horn_face_depth - ctx.sheet_t("frame")) / ctx.pitch
        if abs(face - round(face)) > 1e-6:
            raise ConstructionError(
                f"the bolt crank's hub plate needs the horn's face on a layer boundary; it is "
                f"{drive.horn_face_depth:g} mm under the inner plate's top face in "
                f"{ctx.pitch:g} mm layers (the drive's horn spacer sizes for it: "
                "DriveGroup.spacer)")
        if self.horn_joint_web(ctx, t, drive.spacer_t) is None:
            raise ConstructionError(f"no screw fits between the bolt crank's hub plate and the "
                                    f"{ctx.servo.key} horn")
        if not self.stub_layers_web(ctx.sheet_t("frame"), ctx.pitch, t):
            raise ConstructionError(f"no stock stub standoff fits {ctx.pitch:g} mm layers")
        return dims

    def _pin_screw(self, through: float, depth: float) -> tuple[str, float] | None:
        """The M4 button head through ``through`` mm (a web, and shims) into a standoff's
        end tapped ``depth`` deep: the most thread, (key, engagement)."""
        from spiderpig.hardware.crank_catalog import M4_BHCS_LENGTHS, m4_bhcs

        best = None
        for L in M4_BHCS_LENGTHS:
            e = L - through
            if self.pin_min_engage - EPS <= e <= depth + EPS and (best is None or e > best[1]):
                best = (m4_bhcs(L), round(e, 3))
        return best

    def fit_web(self, free: float, gap: float, t_lo: float, t_top: float) -> WebJoint | None:
        """The crankpin of a chain of single-plate webs: ``free`` mm between the lowest web's
        top face and the top web's bottom face, not counting the clearance gap over the
        lowest web (``gap``). The shortest stock standoff at least ``free`` long; the gap
        over the lowest web it needs (its length past ``free``), the rest of that gap taken
        up by DIN 988 shims under its end (0.1 mm steps); an M4 button head into each end
        with at least ``pin_min_engage`` of thread."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import gobilda_1501
        from spiderpig.stack import GAP_MAX

        for segs in _pin_lengths():
            S = sum(segs)
            if free - EPS > S:
                continue
            need = max(0.0, S - free)
            if need > GAP_MAX + EPS:
                return None
            shims = round(max(0.0, free + gap - S), 1)
            d_lo = float(get(gobilda_1501(segs[0])).dims["thread_depth"])
            d_hi = float(get(gobilda_1501(segs[-1])).dims["thread_depth"])
            lo = self._pin_screw(t_lo + shims, d_lo)
            hi = self._pin_screw(t_top, d_hi)
            if lo is None or hi is None:
                continue
            return WebJoint(gobilda_1501(segs[0]), S, shims, round(need, 3), lo[0], lo[1],
                            hi[0], hi[1], segs)
        return None

    @staticmethod
    def plate_z(L: Layout, k: int, t: float, hub: int | None = None) -> tuple[float, float]:
        """A single crank plate's z in layer ``k``: on the layer's floor, the hub plate at its
        top (against the horn spacer). A plate thinner than its layer leaves air over (or
        under) it, which the crankpins' shims and the screws' heads use."""
        z0, z1 = L.z(k)
        return (z1 - t, z1) if k == hub else (z0, z0 + t)

    def chain_fit_web(self, L: Layout, lo: int, hi: int, low: int = 0, t: float | None = None,
                      hub: int | None = None) -> WebJoint | None:
        """:meth:`fit_web` for a chain over runs ``lo``..``hi`` at the layout's z: its webs
        in ``lo - 1`` and ``hi + 1``, plates ``t`` thick (default: the layers'). Its
        ``gap`` is what the standoff needs of the clearance gap over the lowest web once the
        air over that plate is used."""
        w0, w1 = lo - 1, hi + 1
        t0 = L.t(w0) if t is None else t
        t1 = L.t(w1) if t is None else t
        top_face = self.plate_z(L, w0, t0, hub)[1]
        bottom = self.plate_z(L, w1, t1, hub)[0]
        floor = L.z(lo)[0]
        j = self.fit_web(bottom - floor, floor - top_face, t0, t1)
        if j is None:
            return None
        air = L.z(w0)[1] - top_face
        return replace(j, gap=round(max(0.0, j.gap - air), 3))

    def _web_span_ok(self, run: int, low: int, pitch: float, t: float) -> bool:
        """Whether a chain whose webs are ``run`` layers apart takes a stock standoff at the
        nominal z or up to an aluminium plate's thickness more (the exact z decides)."""
        air = max(0.0, pitch - t)
        for dz in (0.0, run * 0.175, run * 0.175 + 1.0, air):
            if self.fit_web(run * pitch + dz, air, t, t) is not None:
                return True
        return False

    def _web_rules(self, ctx: Context, dims: CrankDims):
        from spiderpig.construction.route import JointRules, horn_pockets

        p, t = ctx.pitch, ctx.sheet_t("crank")
        spans: dict[int, int] = {}
        for n in range(3, 64):
            m = 0
            if self._web_span_ok(n - 2, 0, p, t):
                for a in (0, 1):
                    for low in range(4):
                        for c in (0, 1):
                            for d in (0, 1):
                                m |= 1 << (16 * a + 4 * low + 2 * c + d)
            spans[n] = m
        r = self.head_r()
        hr = self.horn_kind(ctx).head_d / 2 + 0.3
        return JointRules(spans, head=r, nut=r, post=dims.post, horn=horn_pockets(ctx),
                          hub_play=True, two_layer_top=False, two_layer_bottom=False, tip=False,
                          low_count=False, inner_webs=False, share_stack=False,
                          bottom_layers=self.stub_layers_web(ctx.sheet_t("frame"), p, t),
                          gap_head=r, horn_heads=tuple((h, hr) for h in self.horn_points(ctx)),
                          j_spans=tuple((n, self._web_span_ok(n, 0, p, t)) for n in range(64)),
                          j_last=True, gap_washer=self.washer_r)

    def stub_layers_web(self, t0: float, pitch: float, t: float, most: int = 64
                        ) -> frozenset[int]:
        """The layers the first chain's lowest web may sit in: the stub standoff from its
        underside down into the outer frame plate (screwed to it from above)."""
        return frozenset(a for a in range(2, most)
                         if self.stub_z(t0 + (a - 1) * pitch, t0, t) is not None)

    def horn_joint_web(self, ctx: Context, seg: float, spacer: float
                       ) -> tuple[ScrewKind, float, float] | None:
        """The horn screws up through the crank's top plates (``seg`` mm: the hub and what
        is under it, their heads under the lowest) and the horn spacer (``spacer``) into
        the horn: (kind, length, thread in the horn), most thread then shortest."""
        from spiderpig.hardware.fasteners import SCREWS

        spec = ctx.servo
        pat = spec.horn.pattern
        size = SIZES.get(pat.thread)
        order = ("self_tap",) if pat.tapping else ("bhcs", "shcs")
        kinds = [SCREWS[(k, size)] for k in order if (k, size) in SCREWS]
        reach = pat.reach if pat.reach is not None else spec.horn.thickness
        e_max = min(reach, spec.horn_face_depth)
        e_want = min(pat.thread_depth if pat.thread_depth is not None else e_max, e_max)
        e_min = min(e_want, 1.5)
        for sk in kinds:
            best = None
            for L in sk.lengths:
                e = L - seg - spacer
                if not e_min - EPS <= e <= e_max + EPS:
                    continue
                if best is None or min(e, e_want) > min(best[2], e_want) + EPS:
                    best = (sk, L, e)
            if best is not None:
                return best
        return None

    def horn_kind(self, ctx: Context) -> ScrewKind:
        """The horn screws' kind (its head is what hangs under the crank's top plates)."""
        from spiderpig.hardware.fasteners import SCREWS

        pat = ctx.servo.horn.pattern
        size = SIZES.get(pat.thread)
        order = ("self_tap",) if pat.tapping else ("bhcs", "shcs")
        kinds = [SCREWS[(k, size)] for k in order if (k, size) in SCREWS]
        if not kinds:
            raise ConstructionError(f"no screw for the {ctx.servo.key} horn's {pat.thread}")
        return kinds[0]

    def horn_points(self, ctx: Context) -> list[str]:
        """The horn screws as points fixed to the crank (added to the side's geometry)."""
        drive: DriveInterface = ctx.interfaces["drive"]
        topo = ctx.topo
        out = []
        for i in range(drive.screw_count):
            name = f"crank.horn{i}"
            if name not in topo.crank_points:
                a = math.degrees(drive.pattern_angle + 2 * math.pi * i / drive.screw_count)
                topo.add_crank_point(name, drive.screw_pcd / 2, a)
            out.append(name)
        return out

    @staticmethod
    def top_segment(plates: set[int], hub: int) -> int:
        """The lowest layer of the crank plates stacked under (and with) the hub's."""
        k = hub
        while k - 1 in plates:
            k -= 1
        return k

    def horn_spacer(self, L: Layout, drive: DriveInterface, hub: int) -> float:
        """The printed horn spacer at the plan's z: over the hub plate up to the horn's face."""
        return L.z(L.top)[1] - L.z(hub)[1] - (drive.horn_face_depth - drive.spacer_t)

    def web_claims(self, ctx: Context, L: Layout, route: CrankRoute, ridden, d: CrankDims,
                   plate_t: float, drive: DriveInterface, hub: int, horn_pts: list[str]
                   ) -> list[Placed]:
        """What single-plate webs add to the route's shapes: per chain the tip under its
        lowest web, the jam nut over it and the head over its top web (clearance shapes:
        each in the gap beside its plate, or sunk into the layer there when nothing is in
        its way); the stub plate under the first chain; the plates between the last chain
        and the hub as wide as the hub (the horn screws pass them); the horn screws' heads
        under the lowest of those."""
        out: list[Placed] = []
        hr = self.head_r()
        chains = chains_of(route.runs)
        clear = self.head_clear
        head = self.pin_screw()[1] + clear
        t = plate_t
        for ch in chains:
            at, lo, hi = ch[0].at, ch[0].lo, ch[-1].hi
            j = self.chain_fit_web(L, lo, hi, t=t, hub=hub)
            if j is None and L.final:
                raise Unbuildable(f"at the plan's z no stock standoff fits the crankpin at {at} "
                                  f"(webs in layers {lo - 1} and {hi + 1})")
            if lo - 2 < 0:
                raise Unbuildable(f"the screw under crankpin {at} needs layer {lo - 2}")
            out.append(Placed(lo - 2, Disc(at, hr), GROUP, f"crankpin screw {at}", gap=True,
                              height=head, toward=-1))
            out.append(Placed(hi + 1, Disc(at, hr), GROUP, f"crankpin screw {at}", gap=True,
                              height=head, toward=1))
            # the standoff and its shims through the gap over the lowest web (the height:
            # its length past the layers and the air over the plate)
            out.append(Placed(lo - 1, Disc(at, self.pin_od / 2 + 1.0), GROUP,
                              f"crankpin {at} spacer", gap=True,
                              height=j.gap if j is not None else 0.0))
            g = ctx.topo.geometry.points
            R = float(np.linalg.norm(g[at][0] - g["O"][0]))
            if hi + 1 == hub and drive.horn_radius + hr + ctx.params.margin > R:
                # its head over the hub plate stands in a pocket of the printed horn spacer
                # (servos.mount.DriveGroup.realize cuts it), which must be that thick
                spacer = (self.horn_spacer(L, drive, hub) if L.final else drive.spacer_t)
                if spacer < head - 1e-6:
                    raise Unbuildable(f"the screw head of crankpin {at} over the hub plate "
                                      f"needs {head:.2f} mm of the horn spacer, which is "
                                      f"{spacer:.2f} mm")
                if hr + (drive.center_head_d + ctx.params.print_fit) / 2 + 0.5 > R:
                    raise Unbuildable(f"the screw head of crankpin {at} would meet the horn's "
                                      "centre screw")
        # journal standoffs on O between chains that don't share a web
        order = sorted(chains, key=lambda ch: ch[0].lo)
        for c0, c1 in itertools.pairwise(order):
            e, f = c0[-1].hi + 1, c1[0].lo - 1
            if f <= e:
                continue
            j = self.chain_fit_web(L, e + 1, f - 1, t=t, hub=hub)
            if j is None and L.final:
                raise Unbuildable(f"at the plan's z no stock standoff joins the crank's webs "
                                  f"in layers {e} and {f} on O")
            out += [Placed(k, Disc("O", d.journal), GROUP, "crank journal")
                    for k in range(e + 1, f)]
            out.append(Placed(e - 1, Disc("O", hr), GROUP, "journal screw", gap=True,
                              height=head, toward=-1))
            out.append(Placed(f, Disc("O", hr), GROUP, "journal screw", gap=True,
                              height=head, toward=1))
            out.append(Placed(e, Disc("O", self.pin_od / 2 + 1.0), GROUP,
                              "crank journal spacer", gap=True,
                              height=j.gap if j is not None else 0.0))
        first = min(chains, key=lambda ch: ch[0].lo)
        a = first[0].lo - 1
        if route.bearing:
            # the stub standoff's screw from above the lowest web: its head in the air over
            # that plate, the rest in the gap there
            sk = BHCS["3"]
            air = L.t(a) - t
            h = sk.head_h + clear - air
            out.append(Placed(a, Disc("O", sk.head_d / 2 + 0.3), GROUP, "stub screw head",
                              gap=True, height=round(max(h, 0.0), 3)))
        top_web = max(ch[-1].hi + 1 for ch in chains)
        out += [Placed(k, Disc("O", d.hub), GROUP, "crank body", sheet=plate_t)
                for k in range(top_web + 1, hub)]
        seg = top_web
        sk = self.horn_kind(ctx)
        for name in horn_pts:
            out.append(Placed(seg - 1, Disc(name, sk.head_d / 2 + 0.3), GROUP,
                              "horn screw head", gap=True, height=sk.head_h + clear,
                              toward=-1))
        if L.final and self.horn_joint_web(ctx, t, self.horn_spacer(L, drive, hub)) is None:
            raise Unbuildable(f"at the plan's z no stock horn screw fits the crank's top "
                              f"plates (layers {seg}..{hub}) and the horn spacer")
        return out

    def _web_check(self, L: Layout, route: CrankRoute, plates: set[int],
                   drive: DriveInterface | None) -> None:
        """The stub and the horn screws at the plan's z (the chains' bolts: their claims)."""
        if route.bearing and plates:
            a = min(ch[0].lo for ch in chains_of(route.runs)) - 1
            if self.stub_z(L.z(a)[0] - L.z(0)[0], L.t(0), self.web_t) is None:
                raise Unbuildable(f"at the plan's z no stock stub standoff reaches the outer "
                                  f"frame plate from the lowest web (layer {a})")

    def _web_capacity(self, joint: WebJoint | None = None) -> dict[str, float]:
        """What one crankpin of single aluminium webs holds (N·m), per web (the weaker
        counts; both are alike): the standoff's end face clamped on the web by the M4
        screw's preload, and beside it the screw's head on the web's other face, which
        reaches the standoff only through the screw's thread (its friction under that
        preload plus the threadlocker's breakaway, half for plated steel). Friction joints:
        the preload and both coefficients are estimates, UNVERIFIED until the test build."""
        from spiderpig.hardware.catalog import get

        F = self.pin_preload_n
        hd, _ = self.pin_screw()

        def r_eff(ro: float, ri: float) -> float:      # an annulus's friction radius (mm)
            return 2 / 3 * (ro ** 3 - ri ** 3) / (ro ** 2 - ri ** 2)

        face = self.pin_mu * F * r_eff(self.pin_od / 2, 2.0) / 1e3
        head = self.head_mu * F * r_eff(hd / 2, self.pin_hole / 2) / 1e3
        thread = 0.15 * F * (3.545 / 2) / math.cos(math.radians(30)) / 1e3
        e = min(joint.engage_lo, joint.engage_hi) if joint else self.pin_min_engage
        lock = 0.0
        if self.pin_lock_key is not None:
            m10 = float(get(self.pin_lock_key).dims.get("breakaway_m10_nm", 0.0))
            lock = 0.5 * m10 * (4.0 / 10.0) ** 2 * (e / 8.4)
        screw = min(head, thread + lock)
        return {
            f"web clamped on the standoff's end ({F:g} N, mu {self.pin_mu:g}) + the screw "
            f"head (mu {self.head_mu:g}, through the M4 thread)": round(face + screw, 3),
        }

    # -- parts ----------------------------------------------------------------------------

    def realize(self, group: CrankGroup, build: Build) -> Realized:
        if self.single:
            return _WebPlates(self, group, build).make()
        return _BoltPlates(self, group, build).make()


class _BoltPlates:
    """The bolt crank of one side under construction (:meth:`BoltCrank.realize`)."""

    def __init__(self, c: BoltCrank, group: CrankGroup, build: Build):
        self.c, self.build = c, build
        ctx = build.ctx
        self.p = ctx.params
        self.pitch = ctx.pitch
        self.d = group.dims(ctx)
        topo = build.plan.topo
        self.host = topo.crank_bodies[0]
        self.drive: DriveInterface = ctx.interfaces["drive"]
        self.pins = topo.axes_of("crankpin")
        self.route = route_of(build.plan.layout, self.pins)
        self.runs = sorted(self.route.runs, key=lambda r: (r.lo, r.at))
        self.chains = chains_of(self.runs)
        self.run_layers = {k for r in self.runs for k in range(r.lo, r.hi + 1)}
        self.plates: dict[int, list] = {}
        self.fillers: dict[int, list] = {}      # gap layer -> the crank's shapes through it
        self.washers: list = []                 # crankpin washers in a gap
        for pl in build.shapes(GROUP):
            if pl.gap:
                if pl.label.endswith(" washer"):
                    self.washers.append(pl)
                elif pl.sheet > 0:          # a stack of its plates through the gap
                    self.fillers.setdefault(pl.layer, []).append(pl)
                continue
            if pl.label in ("crank body", "crank hub") or pl.label.startswith("web "):
                self.plates.setdefault(pl.layer, []).append(pl)
        self.sheet = ctx.sheet("crank")
        self.sheet_t = ctx.sheet_t("crank")
        self.cuts: dict[int, list] = {k: [] for k in self.plates}
        self.out = Realized()
        self.notes: list[dict] = []

    def xy(self, point: str) -> tuple:
        return tuple(self.build.xy(point))

    def buy(self, name: str, part, key: str, color: str = STEEL) -> None:
        self.out.bodies.append(hardware(name, part, self.host, fab="purchased", bom_key=key,
                                        color=color))

    def cut(self, k: int, solid) -> None:
        if k in self.cuts:
            self.cuts[k].append(solid)

    def make(self) -> Realized:
        c, b = self.c, self.build
        riders = {p.name: {b.layers[m] for m in p.members} for p in self.pins}
        af = c.pocket_af()
        head_af, head_h = c.head()
        nut_af, nut_h = c.nut()
        for ch in self.chains:
            at, xy, ang = ch[0].at, self.xy(ch[0].at), b.angle("O", ch[0].at)
            tag = at if sum(x[0].at == at for x in self.chains) == 1 else f"{at}_{ch[0].lo}"
            lo, hi = ch[0].lo, ch[-1].hi
            if len(ch) > 1:
                raise ConstructionError(
                    f"the crank's route returns to O between runs of {at} (layers "
                    f"{ch[0].hi + 1}..{ch[1].lo - 1}): a bolt crank's plate there would turn "
                    "loose on the bolt's round shank (JointRules.inner_webs)")
            n_run = hi - lo + 1
            ridden = riders.get(at, set()) & set(range(lo, hi + 1))
            low = min(3, next((i for i in range(n_run) if lo + i in ridden), n_run))
            joint = c.chain_fit(b.plan.layout, lo, hi, low)
            if joint is None:
                raise ConstructionError(
                    f"no stock M6 bolt fits the crankpin chain at {at}: its stacks are "
                    f"{n_run} layers apart (layers {lo}..{hi}), its lowest rider {low} layers "
                    "up, and no length keeps every rider on the plain shank, the nylock on "
                    "complete thread and the tip inside the layer under its stack")
            z_l = b.z(lo - 2)[0]
            z_h = b.z(hi + 1)[0] + c.head_gap
            for k in (lo - 2, lo - 1, hi + 1, hi + 2):
                self.cut(k, hex_pocket(xy, af, b.z(k)[0] - 3, b.z(k)[1] + 3, ang, c.relief))
            self.cut(lo - 3, disc(xy, c.bore / 2, b.z(lo - 3)[0] - 3, b.z(lo - 3)[1] + 3))
            bolt = union([_hex(xy, head_af, z_h, z_h + head_h, ang),
                          disc(xy, c.shank_model / 2, z_h - joint.cut, z_h)])
            self.buy(f"crank_bolt_{tag}", bolt, joint.bolt)
            z_n = z_l - joint.sink
            nut = _hex(xy, nut_af, z_n, z_n + nut_h, ang) - disc(xy, c.bore / 2, z_n - 1,
                                                                 z_n + nut_h + 1)
            self.buy(f"crank_nut_{tag}", nut, c.nut_key)
            if c.lock_key is not None:
                self.out.extras.append(BomLine(c.lock_key, c.lock_per_bolt, f"crank bolt {tag}"))
            self.notes.append({
                "at": tag, "bolt": joint.bolt, "length_mm": joint.length,
                "cut_to_mm": None if joint.cut >= joint.length else round(joint.cut, 2),
                "plain_mm": joint.plain, "bare_layers": low, "run_layers": n_run,
                "head_engaged_mm": joint.head_engaged, "nut_engaged_mm": joint.nut_engaged,
                "tip_mm": round(joint.tip_out, 2), "layers": [lo - 3, hi + 2],
                "capacity_nm": c.capacity(joint)})
        self.stub()
        self.hub()
        return self.finish()

    def stub(self) -> None:
        c, b = self.c, self.build
        o = self.xy("O")
        if not self.route.bearing or not self.plates:
            return
        lowest = min(self.plates)
        got = c.stub_z(b.z(lowest)[0] - b.z(0)[0], b.plan.t(0), b.plan.t(lowest))
        if got is None:
            raise ConstructionError(f"no stock stub standoff reaches the outer frame plate from "
                                    f"the crank's lowest stack (layer {lowest})")
        key, S, z0, L = got
        z0 += b.z(0)[0]
        top = b.z(lowest)[0]
        od = c.stub_od()
        st = disc(o, od / 2 - 0.01, z0, top) - disc(o, 1.5, z0 - 1, top + 1)   # M3 thread
        self.buy("crank_stub", st, key, "#c0c0c0")
        sk = BHCS["3"]
        bearing = b.z(lowest)[1]
        self.buy("crank_stub_screw", screw_body(o, sk, bearing, L, up=False), sk.key(L))
        self.cut(lowest, disc(o, 3.4 / 2, b.z(lowest)[0] - 3, bearing + 1))
        self.cut(lowest + 1, disc(o, (sk.head_d + 0.5) / 2, bearing - 1, b.z(lowest + 1)[1] + 3))
        self.out.cut(FRAME_OUTER, Cut(o, self.p.hole(od)))

    def hub(self) -> None:
        c, b, drive = self.c, self.build, self.drive
        hub = sorted(pl.layer for pl in b.shapes(GROUP) if pl.label == "crank hub")
        if not hub:
            return
        got = c.horn_joint(b.ctx)
        if got is None:
            raise ConstructionError(f"no screw fits the bolt crank's hub and the "
                                    f"{b.ctx.servo.key} horn")
        sk, L, _ = got
        upper = hub[-1]
        o = np.asarray(self.xy("O"))
        theta = b.angle("O", self.pins[0].name) + drive.pattern_angle
        bearing = b.z(upper)[0]
        for i in range(drive.screw_count):
            a = theta + 2 * math.pi * i / drive.screw_count
            xy = tuple(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]))
            self.cut(upper, disc(xy, drive.screw_clearance_d / 2, bearing - 1,
                                 b.z(upper)[1] + 1))
            for k in hub[:-1]:
                self.cut(k, disc(xy, drive.screw_head_d / 2, b.z(k)[0] - 3, b.z(k)[1] + 3))
            self.buy(f"crank_horn_screw{i}", screw_body(xy, sk, bearing, L), sk.key(L))
        if drive.center_head_d > 0 and drive.center_head_h > EPS:
            r = (drive.center_head_d + self.p.print_fit) / 2
            face = b.z(upper)[1]
            for k in hub:
                if b.z(k)[1] > face - drive.center_head_h - 0.2:
                    self.cut(k, disc(tuple(o), r, b.z(k)[0] - 3, b.z(k)[1] + 3))

    def gap_parts(self, out: Realized) -> None:
        """What fills a clearance gap the crank runs through: a filler plate cut from the
        thin sheet of the gap's thickness where a stack of plates goes on through it (its
        neighbours' pockets and bores cut through it too), and a stack of washers on a
        crankpin between its riders."""
        from spiderpig.materials import filler_sheet, sheet, washer_stack

        b = self.build
        for k, shapes in sorted(self.fillers.items()):
            g = b.plan.gaps.get(k, 0.0)
            if g <= 0:
                continue
            z = b.plan.gap_z(k)
            solid = union([shape_solid(b, pl, z=z) for pl in shapes])
            cuts = self.cuts.get(k, []) + self.cuts.get(k + 1, [])
            if cuts:
                solid = solid - union(cuts)
            key = filler_sheet(sheet(self.sheet).material, g)
            out.bodies.append(hardware(f"crank_filler{k}", solid, self.host, fab="laser",
                                       color=BOLT_COLOR, sheet=key))
        horn = [pl for pl in b.shapes(GROUP) if pl.label == "horn screw head" and pl.gap]
        for pl in self.washers:
            g = b.plan.gaps.get(pl.layer, 0.0)
            if g <= 0:
                continue
            if any(h.layer == pl.layer and math.dist(self.xy(h.shape.at), self.xy(pl.shape.at))
                   < h.shape.r + pl.shape.r for h in horn):
                continue        # a horn screw's head is there: the rider turns on air
            z0, _ = b.plan.gap_z(pl.layer)
            items, _ = washer_stack(self.c.bolt_d, g)
            if not items:
                continue
            t = sum(x[1] for x in items)
            xy = self.xy(pl.shape.at)
            od = 2 * pl.shape.r
            part = disc(xy, od / 2, z0, z0 + t) - disc(xy, self.c.bore / 2, z0 - 1, z0 + t + 1)
            tag = f"{pl.shape.at}_{pl.layer}"
            self.buy(f"crank_washers_{tag}", part, items[0][0], "#f2f2f2")
            for key, _ in items[1:]:
                out.extras.append(BomLine(key, 1, f"crankpin {tag}: gap washers"))

    def finish(self) -> Realized:
        from build123d import Plane, section

        c, b = self.c, self.build
        out = self.out
        solids: dict[int, object] = {}
        for k, shapes in sorted(self.plates.items()):
            z0 = b.z(k)[0]
            z = (z0, min(b.z(k)[1], z0 + self.sheet_t))      # its own sheet, on the layer's floor
            solid = union([shape_solid(b, pl, z=z) for pl in shapes])
            if self.cuts[k]:
                solid = solid - union(self.cuts[k])
            parts = solid.solids()
            solid = parts[0] if len(parts) == 1 else solid
            solids[k] = solid
            out.bodies.append(hardware(f"crank_plate{k}", solid, self.host, fab="laser",
                                       color=BOLT_COLOR, sheet=self.sheet))
        self.gap_parts(out)
        # segments: the plates between runs, each cemented into one stack
        segs: list[list[int]] = []
        for k in sorted(solids):
            if segs and segs[-1][-1] == k - 1:
                segs[-1].append(k)
            else:
                segs.append([k])
        bonds = []
        for seg in segs:
            for k0, k1 in itertools.pairwise(seg):
                m0, m1 = sum(b.z(k0)) / 2, sum(b.z(k1)) / 2
                try:
                    f0 = section(solids[k0], Plane.XY.offset(m0))
                    f1 = section(solids[k1], Plane.XY.offset(m1))
                    common = f0 & f1.moved(Location((0, 0, m0 - m1)))
                    area = float(common.area)
                except Exception:      # noqa: BLE001 - a section that fails: no bond counted
                    continue
                r_eq = 2.0 / 3.0 * math.sqrt(area / math.pi)
                bonds.append({"layers": [k0, k1], "area_mm2": round(area, 1),
                              "capacity_nm": round(c.bond_mpa * area * r_eq / 1e3, 3)})
        n_plates = len(solids)
        if n_plates:
            out.extras.append(BomLine(adhesive(self.sheet), c.cement_per_plate * n_plates,
                                      "crank plate stacks"))
        for pin in self.pins:
            for m in pin.members:
                out.cut(m, Cut(self.xy(pin.name), self.p.hole(self.c.bolt_d)))
        weakest_bond = min(bonds, key=lambda x: x["capacity_nm"], default=None)
        out.notes["crank_bolt"] = {
            "pocket_af_mm": round(c.pocket_af(), 3), "relief_mm": c.relief,
            "play_deg": round(2 * hex_play(c.head()[0], c.pocket_af()), 2),
            "play_deg_worst": round(2 * hex_play(9.78, c.pocket_af()), 2),
            "chains": self.notes, "plates": n_plates, "segments": [s for s in segs],
            "bond_mpa": c.bond_mpa, "weakest_bond": weakest_bond,
            "threadlocker": c.lock_key}
        return out


class _WebPlates(_BoltPlates):
    """The single-plate bolt crank of one side (:meth:`BoltCrank.for_sheet`): every crank
    layer one aluminium plate (on its layer's floor; the hub plate at its layer's top,
    against the horn spacer), every crankpin and journal a round standoff clamped between
    two webs by M4 screws, the stub standoff screwed to the lowest web from above, the horn
    screws up through the hub plate."""

    def __init__(self, c: BoltCrank, group: CrankGroup, build: Build):
        super().__init__(c, group, build)
        hub = [pl.layer for pl in build.shapes(GROUP) if pl.label == "crank hub"]
        self.hub_layer = max(hub) if hub else None
        self.t = self.sheet_t

    def pz(self, k: int) -> tuple[float, float]:
        return self.c.plate_z(self.build.plan.layout, k, self.t, self.hub_layer)

    def layer_cut(self, k: int, xy, r: float, z0: float | None = None,
                  z1: float | None = None) -> None:
        b = self.build
        if k in self.cuts:
            lo, hi = b.z(k)
            self.cuts[k].append(disc(xy, r, lo - 3 if z0 is None else z0,
                                     hi + 3 if z1 is None else z1))

    def make(self) -> Realized:
        self.journals: list[dict] = []
        for ch in self.chains:
            at, xy = ch[0].at, self.xy(ch[0].at)
            tag = at if sum(x[0].at == at for x in self.chains) == 1 else f"{at}_{ch[0].lo}"
            lo, hi = ch[0].lo, ch[-1].hi
            if len(ch) > 1:
                raise ConstructionError(
                    f"the crank's route returns to O between runs of {at} (layers "
                    f"{ch[0].hi + 1}..{ch[1].lo - 1}): a plate there would turn loose on the "
                    "standoff (JointRules.inner_webs)")
            self.notes.append(self.pin(at, xy, tag, lo, hi))
        order = sorted(self.chains, key=lambda ch: ch[0].lo)
        for c0, c1 in itertools.pairwise(order):
            e, f = c0[-1].hi + 1, c1[0].lo - 1
            if f > e:
                note = self.pin("O", self.xy("O"), f"journal{e}", e + 1, f - 1)
                self.journals.append(note)
        self.stub()
        self.hub()
        return self.finish()

    def pin(self, at: str, xy, tag: str, lo: int, hi: int) -> dict:
        """A round standoff between the webs in layers ``lo - 1`` and ``hi + 1`` at ``xy``,
        an M4 button head into each end, shims under its lower end; its note."""
        from spiderpig.hardware.catalog import get
        from spiderpig.materials import washer_stack

        c, b = self.c, self.build
        hd, hh = c.pin_screw()
        j = c.chain_fit_web(b.plan.layout, lo, hi, t=self.t, hub=self.hub_layer)
        if j is None:
            raise ConstructionError(f"no stock standoff fits the crankpin at {at} (webs in "
                                    f"layers {lo - 1} and {hi + 1})")
        w0, w1 = lo - 1, hi + 1
        z_lo, z_hi = self.pz(w0)[1], self.pz(w1)[0]
        for k in (w0, w1):
            self.layer_cut(k, xy, c.pin_hole / 2)
        # a crank plate under or over the chain at the pin (the stub plate, a hub plate)
        self.layer_cut(w0 - 1, xy, c.head_r())
        self.layer_cut(w1 + 1, xy, c.head_r())
        from spiderpig.hardware.crank_catalog import gobilda_1501, m4_set_screw

        z0 = z_hi - j.length
        za = z0
        for i, seg in enumerate(j.segments or (j.length,)):
            st = disc(xy, c.pin_od / 2 - 0.01, za, za + seg) - disc(xy, 2.0, za - 1,
                                                                    za + seg + 1)
            self.buy(f"crank_pin_{tag}" + (f"_{i}" if i else ""), st, gobilda_1501(seg),
                     "#c8ccd0")
            za += seg
            if i + 1 < len(j.segments):
                self.buy(f"crank_pin_stud_{tag}", disc(xy, 1.95, za - 6, za + 6),
                         m4_set_screw(12))
        if j.shims > 0:
            items, _ = washer_stack(4.0, j.shims)
            items = [x for x in items if not x[0].startswith("ptfe")] or items
            t = sum(x[1] for x in items)
            shim = disc(xy, 3.95, z_lo, z_lo + t) - disc(xy, 2.05, z_lo - 1, z_lo + t + 1)
            self.buy(f"crank_pin_shims_{tag}", shim, items[0][0], "#9a9a9a")
            for key, _ in items[1:]:
                self.out.extras.append(BomLine(key, 1, f"crankpin {tag}: shims"))
        sk_lo = get(j.screw_lo).dims
        sk_hi = get(j.screw_hi).dims
        for name, key, d, bearing, up in (
                (f"crank_pin_screw_lo_{tag}", j.screw_lo, sk_lo, self.pz(w0)[0], True),
                (f"crank_pin_screw_hi_{tag}", j.screw_hi, sk_hi, self.pz(w1)[1], False)):
            s_ = 1.0 if up else -1.0
            body = union([disc(xy, hd / 2, *sorted((bearing - s_ * hh, bearing))),
                          disc(xy, 1.95, *sorted((bearing, bearing + s_ * d["length"])))])
            self.buy(name, body, key)
        if c.lock_key is not None:
            self.out.extras.append(BomLine(c.lock_key, 2 * c.lock_per_bolt,
                                           f"crankpin {tag} screws"))
        return {
            "at": tag, "standoff": j.standoff, "length_mm": j.length,
            "segments_mm": list(j.segments),
            "shims_mm": j.shims, "gap_mm": j.gap, "screws": [j.screw_lo, j.screw_hi],
            "engage_mm": [j.engage_lo, j.engage_hi], "run_layers": hi - lo + 1,
            "layers": [lo - 2, hi + 1], "capacity_nm": c.capacity(j)}

    def stub(self) -> None:
        c, b = self.c, self.build
        o = self.xy("O")
        if not self.route.bearing or not self.chains:
            return
        k = min(ch[0].lo for ch in self.chains) - 1           # the lowest web
        got = c.stub_z(b.z(k)[0] - b.z(0)[0], b.plan.t(0), self.t)
        if got is None:
            raise ConstructionError(f"no stock stub standoff reaches the outer frame plate from "
                                    f"the lowest web (layer {k})")
        key, S, z0, L = got
        z0 += b.z(0)[0]
        top = self.pz(k)[0]
        od = c.stub_od()
        st = disc(o, od / 2 - 0.01, z0, top) - disc(o, 1.5, z0 - 1, top + 1)   # M3 thread
        self.buy("crank_stub", st, key, "#c0c0c0")
        sk = BHCS["3"]
        bearing = self.pz(k)[1]          # the head on the lowest web, from above
        self.buy("crank_stub_screw", screw_body(o, sk, bearing, L, up=False), sk.key(L))
        self.layer_cut(k, o, 3.4 / 2)
        self.out.cut(FRAME_OUTER, Cut(o, od + 0.6))      # +/-0.3 mm: the journal's clearance

    def hub(self) -> None:
        c, b, drive = self.c, self.build, self.drive
        hub = sorted(pl.layer for pl in b.shapes(GROUP) if pl.label == "crank hub")
        if not hub:
            return
        top = hub[-1]
        seg = top
        spacer = c.horn_spacer(b.plan.layout, drive, top)
        got = c.horn_joint_web(b.ctx, self.t, spacer)
        if got is None:
            raise ConstructionError(f"no screw fits the bolt crank's top plates and the "
                                    f"{b.ctx.servo.key} horn")
        sk, L, _ = got
        o = np.asarray(self.xy("O"))
        theta = b.angle("O", self.pins[0].name) + drive.pattern_angle
        bearing = self.pz(top)[0]
        for i in range(drive.screw_count):
            a = theta + 2 * math.pi * i / drive.screw_count
            xy = tuple(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]))
            for k in range(seg, top + 1):
                self.layer_cut(k, xy, drive.screw_clearance_d / 2)
            self.buy(f"crank_horn_screw{i}", screw_body(xy, sk, bearing, L), sk.key(L))
        if drive.center_head_d > 0 and drive.center_head_h > EPS:
            r = (drive.center_head_d + self.p.print_fit) / 2
            face = b.z(top)[1] + spacer
            for k in range(seg, top + 1):
                if b.z(k)[1] > face - drive.center_head_h - 0.2:
                    self.layer_cut(k, tuple(o), r)

    def finish(self) -> Realized:
        c, b = self.c, self.build
        out = self.out
        n_plates = 0
        for k, shapes in sorted(self.plates.items()):
            z = self.pz(k)
            solid = union([shape_solid(b, pl, z=z) for pl in shapes])
            if self.cuts[k]:
                solid = solid - union(self.cuts[k])
            parts = solid.solids()
            solid = parts[0] if len(parts) == 1 else solid
            n_plates += 1
            out.bodies.append(hardware(f"crank_plate{k}", solid, self.host, fab="laser",
                                       color=BOLT_COLOR, sheet=self.sheet))
        self.gap_parts(out)
        for pin in self.pins:
            for m in pin.members:
                out.cut(m, Cut(self.xy(pin.name), self.p.hole(self.c.pin_od)))
        out.notes["crank_bolt"] = {
            "webs": "single", "crankpin": "round standoff clamped by M4 screws",
            "chains": self.notes, "journals": self.journals, "plates": n_plates,
            "segments": [],
            "weakest_bond": None, "threadlocker": c.lock_key}
        return out
