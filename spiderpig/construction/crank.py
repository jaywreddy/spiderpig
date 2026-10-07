"""The crank: the bolt crank's single aluminium web plates on stock standoff crankpins.

The crank turns about O, driven by the servo horn, and carries one crankpin
per leg (point M), which the leg's riders (Klann's b1) turn on. Every rider
sweeps over O, and the crank turns fully relative to it, so the crank can
cross a rider's layer only along that rider's own crankpin: it is a
**built-up crankshaft**. Its shape is a :class:`CrankRoute`, which the
planner chooses (else :func:`default_route`): **runs**, where the shaft
leaves O along a post (at a crankpin, or at a detour point fixed to the
crank) over some layers, each between two **webs** (arms from O out to the
post) in the layers either side; the hub under the servo horn; and, with the
bottom bearing, a journal stub turning in the outer frame plate (without it
the crank hangs from the servo side). That shape is :meth:`CrankGroup.claims`;
the construction, :class:`BoltCrank`, decides radii and how the pieces are made
and joined (its docstring; the planner's rules for it are in
:mod:`construction.route`).

``bolt`` (the default) builds every web as one laser-cut aluminium plate and every
crankpin and journal as a stock steel hex standoff keyed in hex pockets of its two webs;
``bolt_round`` (TrotBot's heel and toe, :data:`config.LINKAGE_CRANKS`) as a round goBILDA
standoff clamped between them by friction. The printed cranks, the acrylic two-plate
M6-bolt crank and the hex crank's variants were removed on 2026-10-07
(:data:`config.REMOVED_CONSTRUCTIONS`).
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

PRESS_DRAWN = 0.02      # a pressed printed bore drawn this much over its steel (no clash)


def hex_play(af: float, pocket_af: float) -> float:
    """Rotation (deg, either way) of a hex ``af`` across flats in a hex pocket
    ``pocket_af`` across flats before its corners (``af / sqrt 3`` out) meet the pocket's
    flats: ``R cos(30 deg - play) = pocket_af / 2``. 0 when the pocket is no wider."""
    if pocket_af <= af:
        return 0.0
    r = af / math.sqrt(3)
    return 30.0 - math.degrees(math.acos(min(1.0, pocket_af / 2 / r)))


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

        c = self.construction
        plate_t = ctx.sheet_t("crank")
        horn_pts = c.horn_points(ctx)

        def hub(L: Layout):
            horn, hub = hub_layers(L, drive, d.hub_thickness)
            if L.final and c.face_on_layer and drive.horn_layers and hub:
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
                                   gap=True) for k in range(0, lo)]
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


def _hex(xy, af: float, z0: float, z1: float, angle: float):
    """A hexagonal prism, ``af`` across flats, one pair of flats facing ``angle``."""
    ang = math.degrees(angle)
    boxes = [Box(af, 4 * af, z1 - z0).rotate(Axis.Z, ang + a) for a in (0.0, 60.0, 120.0)]
    prism = boxes[0] & boxes[1] & boxes[2]
    return moved(prism, Location((float(xy[0]), float(xy[1]), (z0 + z1) / 2)))


# ---------------------------------------------------------------------------
# The bolt crankshaft: single aluminium web plates on stock standoff crankpins
# ---------------------------------------------------------------------------

BOLT_COLOR = "#8fb3e0"           # the crank's plates (the crank study's blue)
SHIM_KEY = "shim_din988_3x6"     # under a horn screw's head (M2 and M3: the head bears on it)
HORN_TIP_CLEAR = 0.3             # a horn screw's tip under the inner plate's top face (mm)


def shim_stack(t: float) -> list[float]:
    """DIN 988 3 x 6 shims making up ``t`` mm (0.1 mm steps), thickest first."""
    from spiderpig.hardware.catalog import get

    sizes = sorted((float(v) for v in get(SHIM_KEY).dims["t"]), reverse=True)
    left, out = round(t, 3), []
    for s in sizes:
        while left >= s - 1e-6:
            out.append(s)
            left = round(left - s, 3)
    return out


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
class HexJoint:
    """One crankpin (or journal) of single-plate webs on a hex standoff
    (:meth:`BoltCrank.fit_hex`): its hex ends in hex pockets through its two webs, an M3
    button head and a wide washer screwed into each end. ``span`` is the distance between
    the two plates' outer faces; the standoff is at least that long, and what it stands past
    a plate's outer face (``out_lo`` / ``out_hi``, into the clearance gap where that end's
    screw head is) carries a printed hex collar of that thickness (``collar_*``, none under
    ``BoltCrank.collar_min``: the plate floats that much), so the washer bears on the
    standoff's end and the collar (or the plate) together."""

    standoff: str        # catalog key (a stock length)
    length: float
    span: float          # the plates' outer faces apart (mm)
    out_lo: float        # the standoff past the lower plate's outer face (mm)
    out_hi: float        # ... past the upper plate's
    collar_lo: float     # printed hex collars taking those up (0: none)
    collar_hi: float
    screw_lo: str        # the M3 button head into the lower end
    engage_lo: float
    washers_lo: int      # DIN 125 washers under the head besides the DIN 9021 one
    screw_hi: str
    engage_hi: float
    washers_hi: int
    engaged_lo: float    # the hex in each plate's pocket (mm)
    engaged_hi: float
    sleeve: float        # the printed sleeve the riders turn on (0: a journal, none)
    gap: float = 0.0     # the clearance gap over the lowest web its length needs (mm; 0:
    #                      a stock length fits as the plan stands, BoltCrank.hex_gap_fit)
    stack_lo: float = 0.0   # what hangs under the lower plate: standoff end, washers, head
    stack_hi: float = 0.0   # ... over the upper plate
    segments: tuple[float, ...] = ()
    gaps: tuple[tuple[int, float], ...] = ()   # (gap slot, mm) the length needs in all


@dataclass(frozen=True)
class BoltCrank:
    """The laser-cut crank (``bolt``, the default; ``bolt_round``): single aluminium web plates.

    Every web is **one plate** cut from the crank's sheet (the default 0.100 in 6061-T6,
    :data:`config.CRANK_SHEET`; :meth:`resolve` reads its thickness and yield), every
    crankpin and journal a stock **hex standoff** (``pin="hex"``, ``bolt``): an M3 x 5.5 AF
    steel standoff whose ends sit in hex pockets of its two webs, an M3 button head and DIN
    9021 washer into each end, the riders turning on a printed sleeve over the hex; or
    (``pin="round"``, ``bolt_round``) a goBILDA 1501 round standoff clamped between its webs
    by an M4 button head into each end (friction). :class:`_WebPlates` builds it; the
    planner's rules for it are in :mod:`construction.route` (:meth:`joint_rules`).

    Assembly: bottom up with the legs, the hub chain capped by the hub plate, which comes
    on with the horn, the servo and the inner plate as one unit
    (:data:`construction.robot.ASSEMBLY`, steps 2 to 4). Axially (the assembly audit of
    2026-10-04): the capped standoff is carried by its sleeve, a light press on the hex
    caught between its two plates (``capped_press``), and the crank body (stub, webs,
    standoffs) stops toward the outer plate on the stub's printed thrust sleeve and toward
    the hub on the capped sleeve (``sleeve_play``): it floats 0.1 + 0.2 mm, and the capped hex
    is rated in the hub's depth less the 0.1. The round standoff's clamp needs a screw over
    the hub plate as well, so a ``bolt_round`` chain that ends in the hub plate has no
    assembly order (:meth:`_WebPlates.assembly_issue`, an audit error).
    """

    key: str = "bolt"
    label: str = ("laser-cut aluminium crank: one plate per web, each crankpin a steel "
                  "hex standoff keyed in hex pockets of its webs (screws and wide washers "
                  "retain them), the riders on a printed sleeve, a round standoff journal "
                  "stub, bolted to the horn through its top plate")
    bolt_d: float = 6.0           # (round) the standoff's washers in a gap: their size
    bore: float = 6.4             # (round) those washers' bore
    lock_key: str | None = "threadlocker_243"
    lock_per_bolt: float = 0.01
    stub_screw_engage: float = 2.5   # least thread of the stub screw in the standoff (5
    #                                  turns of M3; 3.0 before the 3.175 mm aluminium plates)
    stub_seat: float = 1.5           # least of the stub in the outer frame plate
    face_on_layer: bool = True       # the drive's horn face on a layer boundary
    washer_r: float = 6.0            # PTFE washers / shims (6 x 12) on a crankpin through a gap
    web_t: float = 3.175             # the crank sheet's thickness, set by resolve
    stub_below: float = 2.5          # the stub may stand this far out under the
    #                                  outer frame plate (a thin plate leaves a stock length
    #                                  too little room to end inside it); claimed in layer -1
    pin_od: float = 6.0              # the single webs' crankpin: a goBILDA 1501 round standoff
    pin_hole: float = 4.5            # a web's hole for its M4 screw
    pin_min_engage: float = 2.8      # least M4 thread in a standoff's end (4 turns)
    pin_preload_n: float = 2200.0    # an M4 button head at about 2 N·m into the standoff
    pin_mu: float = 0.3              # the standoff's end face on a web (anodised on bare
    #                                  aluminium, dry; UNVERIFIED: the test-build checklist)
    head_mu: float = 0.2             # the screw head on the web
    web_edge_t: float = 1.0          # a web's wall round a hole, in sheet thicknesses
    head_clear: float = 0.3          # z clearance over a head in its gap
    # -- the crankpin: a hex standoff (the default) or the round one ---------
    pin: str = "hex"                 # "hex": a steel hex standoff keyed in hex pockets (the
    #                                  user's decision of 2026-10-04); "round": the goBILDA
    #                                  round standoff clamped by friction (``bolt_round``)
    hex_af: float = 5.5              # the M3 hex standoff's across-flats (crank_catalog)
    hex_fit: float = 0.1             # a hex pocket over the AF (laser-cut, a slide fit)
    dogbone_r: float = 0.8           # the pocket's corner reliefs: at least the service's
    #                                  inside radius (SendCutSend 0.8 mm), each a circle
    #                                  through the hex's corner, centred out along its bisector,
    #                                  so the flats keep their whole length
    hex_corner_loss: float = 0.3     # off each flat for the standoff's rounded corners (mm)
    hex_yield: float = 300.0         # the standoff's steel in bearing (MPa; brass 250)
    web_yield: float = 276.0         # the crank sheet's (6061-T6; set by for_sheet)
    sleeve_od: float = 8.5           # the printed sleeve the riders turn on (its wall 0.9 mm
    #                                  round the bore's corners; a rider's hole + wall within
    #                                  link_radius 6)
    sleeve_fit: float = 0.3          # its hex bore over the standoff's AF (printed)
    sleeve_play: float = 0.2         # its length short of the plates' inner faces
    protrude_max: float = 1.2        # a standoff end past its plate's outer face, at most
    collar_min: float = 0.4          # a printed collar's thinnest (under it the plate floats)
    recess_max: float = 0.3          # a standoff end inside its pocket, at most (the hex in
    #                                  2.24 of 0.100 in 6061-T6: 3.83 N·m, SF 2.25 at 1.7 N·m;
    #                                  materials.ROLES["crank"] counts it)
    hex_min_engage: float = 2.5      # least M3 thread of a screw in the standoff (5 turns)
    hex_tip_gap: float = 0.3         # between the two screws' tips inside the standoff
    hex_engage_max: float = 6.0      # the screws' thread in the standoff, at most
    capped_press: float = 0.1        # (hex) the capped chain's printed sleeve: its hex bore
    #                                  printed this much under the standoff's AF, a light
    #                                  press (the assembly audit of 2026-10-04): the sleeve,
    #                                  captured between its two plates, then carries the
    #                                  standoff, which nothing else holds axially (no screw
    #                                  over the hub plate)
    thrust_od: float = 8.5           # its outside (it bears on the plate round the 6.6 hole)
    thrust_play: float = 0.1         # its end short of the outer plate's inner face

    # -- the sheet ------------------------------------------------------------------------

    def resolve(self, ctx: Context) -> BoltCrank:
        """This crank for ``ctx``'s crank sheet (:meth:`for_sheet`, its thickness as the
        design measures it)."""
        return replace(self.for_sheet(ctx.sheet("crank")), web_t=ctx.sheet_t("crank"))

    @property
    def hex(self) -> bool:
        """Hex standoff crankpins (the default; ``bolt_round``: the round standoff)."""
        return self.pin == "hex"

    def for_sheet(self, key: str | None) -> BoltCrank:
        """This crank cut from sheet ``key``: its plates' thickness and yield (``None``: as
        it stands). A non-metal sheet raises :class:`ConstructionError`: the single plates
        are aluminium (an acrylic crank sheet built the two-plate M6 stack crank, removed on
        2026-10-07; :class:`config.BuildConfig` refuses it first)."""
        if key is None:
            return self
        from spiderpig.materials import sheet

        sh = sheet(key)
        if not sh.metal:
            raise ConstructionError(
                f"the bolt crank's single web plates need a metal crank sheet, not {key!r} "
                "(the acrylic two-plate crank was removed on 2026-10-07)")
        return replace(self, web_t=sh.thickness, web_yield=sh.yield_mpa)

    # -- dimensions and rules -----------------------------------------------------------

    def dims(self, ctx: Context) -> CrankDims:
        p: Params = ctx.params
        drive: DriveInterface = ctx.interfaces["drive"]
        t = ctx.sheet_t("crank")
        wall = max(p.min_wall, self.web_edge_t * t)
        hub = max(drive.horn_radius, drive.screw_pcd / 2 + drive.screw_head_d / 2 + p.min_wall)
        sh = ctx.sheet("crank")
        if sh is not None:
            from spiderpig.materials import sheet

            # the horn screws' holes at the service's edge distance from the hub's rim
            hub = max(hub, drive.screw_pcd / 2 + self.horn_hole(ctx) / 2 + sheet(sh).min_edge)
        if self.hex:
            web = max(p.web_radius, math.ceil((self.hex_reach() + wall) * 10) / 10,
                      self.head_r(),
                      # the stub screw's hole at O, the service's edge distance from the rim
                      math.ceil((3.4 / 2 + (sheet(sh).min_edge if sh else wall)) * 10) / 10)
            journal = math.ceil((self.hex_pocket_af() / math.sqrt(3) + 0.2) * 10) / 10
        else:
            # its screw holes the service's edge distance (2 t) from the web's rim
            edge = max(wall, sheet(sh).min_edge) if sh is not None else wall
            web = max(p.web_radius, math.ceil((self.pin_hole / 2 + edge) * 10) / 10,
                      self.head_r())
            journal = self.pin_od / 2
        rd = self.rider_d()
        dims = CrankDims(web=web, journal=journal, stub=self.stub_od() / 2,
                         post=rd / 2, hub=hub, hub_thickness=t - 1e-3)
        if p.hole(rd) / 2 + p.min_wall > p.link_radius + EPS:
            raise ConstructionError(
                f"a {rd:g} mm crankpin's hole leaves less than {p.min_wall} mm of the "
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

    def check_route(self, L: Layout, route: CrankRoute, ridden: dict[str, set[int]],
                    plates: set[int], drive: DriveInterface | None = None) -> None:
        """The plan's own z (:attr:`stack.Layout.final`): the stub standoff reaches the
        outer frame plate (the chains' standoffs and the horn screws: :meth:`web_claims`;
        :class:`stack.Unbuildable` if not)."""
        if route.bearing and plates:
            a = min(ch[0].lo for ch in chains_of(route.runs)) - 1
            if self.stub_z(L.z(a)[0] - L.z(0)[0], L.t(0), self.web_t) is None:
                raise Unbuildable(f"at the plan's z no stock stub standoff reaches the outer "
                                  f"frame plate from the lowest web (layer {a})")

    def joint_rules(self, ctx: Context, dims: CrankDims):
        """The router's rules (:class:`construction.route.JointRules`): every span a stock
        standoff fits, the screw heads' and washers' gap pieces, the stub's layers."""
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
        hr = self.horn_head_r(ctx)
        return JointRules(spans, head=r, nut=r, post=dims.post, horn=horn_pockets(ctx),
                          inner_webs=False,
                          bottom_layers=self.stub_layers_web(ctx.sheet_t("frame"), p, t),
                          gap_head=r, horn_heads=tuple((h, hr) for h in self.horn_points(ctx)),
                          j_spans=tuple((n, self._web_span_ok(n, 0, p, t)) for n in range(64)),
                          j_last=True, gap_washer=self.washer_r)

    # -- the stub and the strength check's capacities -------------------------------

    def stub_od(self) -> float:
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS, m3_round_standoff

        return float(get(m3_round_standoff(M3_ROUND_STANDOFF_LENGTHS[0])).dims["od"])

    def stub_z(self, top: float, plate: float, upper: float
               ) -> tuple[str, float, float, float] | None:
        """The stub standoff under the lowest web, at a plan's own z: ``top`` that web's
        bottom face over the outer frame plate's bottom face, ``plate`` that plate's
        thickness, ``upper`` the web the screw passes. (key, length, its bottom end's z, the
        screw's length): the longest stock length that ends at most ``stub_below`` under the
        outer plate and at least ``stub_seat`` inside it, and a button head that engages it
        ``stub_screw_engage`` (its head on the lowest web, from above)."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS, m3_round_standoff

        below = self.stub_below
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

    def capacity(self, joint: WebJoint | HexJoint | None = None) -> dict[str, float]:
        """What one crankpin of single aluminium webs holds (N·m), per web (the weaker
        counts; both are alike). The hex pin: :meth:`hex_capacity`. The round one: the
        standoff's end face clamped on the web by the M4
        screw's preload, and beside it the screw's head on the web's other face, which
        reaches the standoff only through the screw's thread (its friction under that
        preload plus the threadlocker's breakaway, half for plated steel). Friction joints:
        the preload and both coefficients are estimates, UNVERIFIED until the test build."""
        from spiderpig.hardware.catalog import get

        if self.hex:
            return self.hex_capacity(joint if isinstance(joint, HexJoint) else None)

        F = self.pin_preload_n
        hd, _ = self.pin_screw()

        def r_eff(ro: float, ri: float) -> float:      # an annulus's friction radius (mm)
            return 2 / 3 * (ro ** 3 - ri ** 3) / (ro ** 2 - ri ** 2)

        face = self.pin_mu * F * r_eff(self.pin_od / 2, 2.0) / 1e3
        head = self.head_mu * F * r_eff(hd / 2, self.pin_hole / 2) / 1e3
        thread = 0.15 * F * (3.545 / 2) / math.cos(math.radians(30)) / 1e3
        e = min(joint.engage_lo, joint.engage_hi) if joint else self.pin_min_engage
        lock = 0.0
        if self.lock_key is not None:
            m10 = float(get(self.lock_key).dims.get("breakaway_m10_nm", 0.0))
            lock = 0.5 * m10 * (4.0 / 10.0) ** 2 * (e / 8.4)
        screw = min(head, thread + lock)
        return {
            f"web clamped on the standoff's end ({F:g} N, mu {self.pin_mu:g}) + the screw "
            f"head (mu {self.head_mu:g}, through the M4 thread)": round(face + screw, 3),
        }

    # -- the crankpins -------------------------------------------------------------------

    def pin_screw(self) -> tuple[float, float]:
        """(head diameter, head height) of the crankpins' M4 button heads."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M4_BHCS_LENGTHS, m4_bhcs

        d = get(m4_bhcs(M4_BHCS_LENGTHS[0])).dims
        return float(d["head_d"]), float(d["head_h"])

    def head_r(self) -> float:
        """A crankpin screw head's clearance shape (the hex pin's: its wide washer)."""
        if self.hex:
            return max(self.hex_screw()[0], self.hex_washer()[1]) / 2 + 0.3
        return self.pin_screw()[0] / 2 + 0.3

    def rider_d(self, params=None) -> float:
        """What the riders turn on: the hex pin's printed sleeve, else the round standoff
        (``params`` unused: the crank's own)."""
        return self.sleeve_od if self.hex else self.pin_od

    # -- the hex standoff crankpin --------------------------------------------------------

    def hex_lengths(self) -> tuple[float, ...]:
        from spiderpig.hardware.crank_catalog import HEX_M3_LENGTHS

        return tuple(float(x) for x in HEX_M3_LENGTHS)

    @staticmethod
    def hex_key(length: float) -> str:
        from spiderpig.hardware.crank_catalog import hex_standoff_m3

        return hex_standoff_m3(length)

    @staticmethod
    def hex_screw() -> tuple[float, float]:
        """(head diameter, head height) of the hex pins' M3 button heads (ISO 7380)."""
        return BHCS["3"].head_d, BHCS["3"].head_h

    @staticmethod
    def hex_washer() -> tuple[float, float, float]:
        """(id, od, t) of the wide washer under each hex pin screw (DIN 9021 M3)."""
        from spiderpig.hardware.catalog import get

        d = get("m3_washer_9021").dims
        return float(d["id"]), float(d["od"]), float(d["t"])

    @staticmethod
    def extra_washer_t() -> float:
        from spiderpig.hardware.catalog import get

        return float(get("m3_washer").dims["t"])

    def hex_pocket_af(self) -> float:
        return self.hex_af + self.hex_fit

    def hex_reach(self) -> float:
        """A hex pocket's reach from its centre: a corner plus its dog-bone relief."""
        return self.hex_pocket_af() / math.sqrt(3) + 2 * self.dogbone_r

    def hex_cut(self, xy, z0: float, z1: float, angle: float):
        """A hex pocket with a dog-bone relief at each corner: a ``dogbone_r`` circle through
        the corner, centred out along its bisector (the flats keep their whole length; the
        service's inside radius is the circle's)."""
        af = self.hex_pocket_af()
        cut = _hex(xy, af, z0, z1, angle)
        rc = af / math.sqrt(3)
        for i in range(6):
            a = angle + math.pi / 6 + i * math.pi / 3
            # 0.05 mm over the corner, so the pocket is one outline (a circle through the
            # corner only touches it): each flat 0.09 mm shorter at each end, within the
            # rating's corner loss
            r = rc + self.dogbone_r - 0.05
            cut = cut + disc((float(xy[0]) + r * math.cos(a), float(xy[1]) + r * math.sin(a)),
                             self.dogbone_r, z0, z1)
        return cut

    def hex_screws(self, length: float) -> tuple[float, int, float] | None:
        """The screw into each end of a ``length`` mm standoff (tapped through): (its stock
        length, DIN 125 washers under its head besides the wide one, thread in the
        standoff): the most thread up to ``hex_engage_max`` that leaves ``hex_tip_gap``
        between the two tips, fewest washers first."""
        _, _, wt = self.hex_washer()
        et = self.extra_washer_t()
        cap = min(self.hex_engage_max, (length - self.hex_tip_gap) / 2)
        best = None
        for k in (0, 1, 2):
            for L in BHCS["3"].lengths:
                e = L - wt - k * et
                if (self.hex_min_engage - EPS <= e <= cap + EPS
                        and (best is None or e > best[2] + EPS)):
                    best = (float(L), k, round(e, 3))
            if best is not None:
                return best
        return None

    def hex_stack(self, out: float, washers: int) -> float:
        """What a hex pin's end holds past its plate's outer face: the standoff's end (or its
        collar), the washers and the screw head."""
        return out + self.hex_washer()[2] + washers * self.extra_washer_t() + self.hex_screw()[1]

    def fit_hex(self, span: float, t_lo: float, t_hi: float, sleeve: bool = True,
                out_hi_max: float | None = None, capped: bool = False,
                air_hi: float = 0.0) -> HexJoint | None:
        """:meth:`_fit_hex`, remembered (the planner asks it at every node's layouts)."""
        return _fit_hex(self, round(span, 6), round(t_lo, 6), round(t_hi, 6), bool(sleeve),
                        out_hi_max, bool(capped), round(max(air_hi, 0.0), 6))

    def _fit_hex(self, span: float, t_lo: float, t_hi: float, sleeve: bool = True,
                 out_hi_max: float | None = None, capped: bool = False,
                 air_hi: float = 0.0) -> HexJoint | None:
        """The hex standoff between two single-plate webs whose outer faces are ``span``
        apart (plates ``t_lo`` and ``t_hi`` thick): the stock length nearest ``span`` that
        either stands past the plates by what the two ends' stacks take (``protrude_max``
        each, the lower end first, ``out_hi_max`` over the upper plate; each stack within a
        clearance gap: the washer then bears on the standoff's end and a printed collar)
        or ends inside both pockets by at most ``recess_max`` (the washers then clamp the
        plates on the printed sleeve, as long as the inner faces are apart, and the hex is
        in that much less of each pocket). Flush or standing past first, then shortest.
        What stands past is split between the ends so the taller of the two stacks' gaps is
        the least (``air_hi``: the air over the upper plate in its own layer, which its stack
        uses first; since the debug of 2026-10-05, before: the lower end first, which made
        every crankpin gap under a chain 4 mm and pushed the Strider decker's Chicago pins
        off their stock barrels)."""
        from spiderpig.stack import GAP_MAX

        room = GAP_MAX - self.head_clear
        hi_max = self.protrude_max if out_hi_max is None else out_hi_max
        best: tuple | None = None
        for S in self.hex_lengths():
            extra = S - span
            if extra < -2 * self.recess_max - EPS:
                continue
            if extra > self.protrude_max + hi_max + EPS:
                break
            scr = self.hex_screws(S)
            if scr is None:
                continue
            L, k, e = scr
            if extra >= -EPS:
                x = max(extra, 0.0)
                lo = min(x, self.protrude_max)
                splits = [(lo, x - lo)]
                if hi_max > 0:
                    # the upper end as far as evens the two gaps' needs (its stack has the
                    # air over its plate first), else shared
                    up = min(max((x + air_hi) / 2, 0.0), x, hi_max)
                    splits += [(x - up, up), (x / 2, x / 2)]
            else:
                splits = [(extra / 2, extra / 2)]       # recessed in both pockets
            ok = [(max(o_lo, o_hi - air_hi), i, o_lo, o_hi)
                  for i, (o_lo, o_hi) in enumerate(splits)
                  if not (o_lo > self.protrude_max + EPS or o_hi > hi_max + EPS
                          or self.hex_stack(max(o_lo, 0.0), k) > room + EPS
                          or self.hex_stack(max(o_hi, 0.0), k) > room + EPS)]
            if ok:
                _, _, out_lo, out_hi = min(ok)
                rank = (extra < -EPS, abs(extra))
                if best is None or rank < best[0]:
                    best = (rank, S, out_lo, out_hi, L, k, e)
        if best is None:
            return None
        _, S, out_lo, out_hi, L, k, e = best

        def collar(o: float) -> float:
            # to 0.01 mm, never past the standoff's end (rounded up, it stood into the
            # washer on that end: klann_lego double's 0.165 mm^3 clash, r4 2026-10-05)
            return math.floor(o * 100 + 1e-6) / 100 if o >= self.collar_min - EPS else 0.0

        inner = span - t_lo - t_hi
        # standing past: the plates captured between the washers and the sleeve, which is
        # short of them by its play; recessed: the washers clamp the plates on the sleeve
        sl = inner - (self.sleeve_play if out_lo >= -EPS else 0.0)
        key = BHCS["3"].key(L)
        return HexJoint(
            self.hex_key(S), S, round(span, 3), round(out_lo, 3), round(out_hi, 3),
            collar(out_lo), collar(out_hi), key, e, k,
            "" if capped else key, 0.0 if capped else e, 0 if capped else k,
            round(t_lo + min(out_lo, 0.0), 3),
            # capped: the crank body's float toward the outer plate (the thrust sleeve's
            # play) draws the hex that much out of the hub plate's pocket
            round(t_hi + min(out_hi, 0.0) - (self.thrust_play if capped else 0.0), 3),
            round(sl, 3) if sleeve else 0.0,
            stack_lo=round(self.hex_stack(max(out_lo, 0.0), k), 3),
            stack_hi=0.0 if capped else round(self.hex_stack(max(out_hi, 0.0), k), 3),
            segments=(S,))

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

    def hub_capped(self, ctx: Context, at: str) -> bool:
        """Whether a crankpin whose chain ends in the hub plate takes no screw over the
        hub plate, only the one from below: every hex one (the hub plate, held to the horn
        by the horn screws, caps its upper end; the hex keeps its whole depth in the hub
        plate\'s pocket); the round standoff\'s clamp needs both screws."""
        return self.hex

    def stub_thrust_r(self, ctx: Context, route: CrankRoute, hub: int) -> float:
        """The radius of the stub's thrust sleeve  when the route has
        a chain capped in the hub plate (0: none): with no screw over the hub plate, the hex
        only slides in its pocket, and nothing else stops the crank body (stub, webs,
        standoffs) moving toward the outer plate (the assembly audit of 2026-10-04: the hex
        would leave the hub plate's pocket after 2.54 mm, the screw heads meet the links
        first)."""
        if not (self.hex and route.bearing):
            return 0.0
        for ch in chains_of(route.runs):
            if ch[-1].hi + 1 == hub and self.hub_capped(ctx, ch[0].at):
                return self.thrust_od / 2
        return 0.0

    def hub_head_need(self, ctx: Context, horn_radius: float, center_d: float) -> float:
        """How thick the printed horn spacer must be for a crankpin\'s screw head over the hub
        plate (it stands in a pocket of the spacer, under the horn): a round standoff\'s
        crankpin within a head\'s reach of the horn\'s rim (a hex one is capped by the hub
        plate, :meth:`hub_capped`); 0 when none is (:meth:`servos.mount.DriveGroup.spacer`
        adds a layer for it)."""
        if self.hex:
            return 0.0              # capped by the hub plate: no head over it
        hr = self.head_r()
        g = ctx.topo.geometry.points
        head = self.pin_screw()[1] + self.head_clear
        need = 0.0
        for a in ctx.topo.axes_of("crankpin"):
            R = float(np.linalg.norm(g[a.name][0] - g["O"][0]))
            if horn_radius + hr + ctx.params.margin <= R:
                continue
            need = max(need, head)
        return need

    def chain_fit_web(self, L: Layout, lo: int, hi: int, low: int = 0, t: float | None = None,
                      hub: int | None = None, sleeve: bool = True, capped: bool = False
                      ) -> WebJoint | HexJoint | None:
        """:meth:`fit_web` for a chain over runs ``lo``..``hi`` at the layout's z: its webs
        in ``lo - 1`` and ``hi + 1``, plates ``t`` thick (default: the layers'). Its
        ``gap`` is what the standoff needs of the clearance gap over the lowest web once the
        air over that plate is used."""
        w0, w1 = lo - 1, hi + 1
        t0 = L.t(w0) if t is None else t
        t1 = L.t(w1) if t is None else t
        if self.hex:
            span = self.plate_z(L, w1, t1, hub)[1] - self.plate_z(L, w0, t0, hub)[0]
            # over the hub plate the upper end stands in the horn spacer's pocket, or (capped)
            # ends in the hub plate under the spacer
            hi_max = 0.0 if capped or w1 == hub else None
            air = self.air_over(L, w1, t1, hub)
            j = self.fit_hex(span, t0, t1, sleeve=sleeve, out_hi_max=hi_max, capped=capped,
                             air_hi=air)
            if j is not None:
                return j
            # the gaps over the lowest web and along the run (a journal: over its lowest
            # web only), those the plan has first
            ks = [w0] + (list(range(lo, hi + 1)) if sleeve else [])
            ks.sort(key=lambda k: (L.gap(k) <= 0, k))
            return self.hex_gap_fit(span, [(k, L.gap(k)) for k in ks], t0, t1, sleeve,
                                    hi_max, capped, air)
        top_face = self.plate_z(L, w0, t0, hub)[1]
        bottom = self.plate_z(L, w1, t1, hub)[0]
        floor = L.z(lo)[0]
        j = self.fit_web(bottom - floor, floor - top_face, t0, t1)
        if j is None:
            return None
        air = L.z(w0)[1] - top_face
        return replace(j, gap=round(max(0.0, j.gap - air), 3))

    def hex_gap_fit(self, span: float, slots: list[tuple[int, float]], t_lo: float,
                    t_hi: float, sleeve: bool = True, out_hi_max: float | None = None,
                    capped: bool = False, air_hi: float = 0.0) -> HexJoint | None:
        """:meth:`_hex_gap_fit`, remembered."""
        return _hex_gap_fit(self, round(span, 6), tuple((k, round(h, 6)) for k, h in slots),
                            round(t_lo, 6), round(t_hi, 6), bool(sleeve), out_hi_max,
                            bool(capped), round(air_hi, 6))

    def _hex_gap_fit(self, span: float, slots: tuple[tuple[int, float], ...], t_lo: float,
                     t_hi: float, sleeve: bool = True, out_hi_max: float | None = None,
                     capped: bool = False, air_hi: float = 0.0) -> HexJoint | None:
        """A hex standoff for a chain no stock length fits at ``span``: the clearance gaps
        ``slots`` (``(slot, mm now)``, in the order to use them: the chain's own gap slots,
        from its lowest web's up) opened or thickened, in 0.1 mm steps each up to
        :data:`stack.GAP_MAX`, by the least that one fits (its ``gaps``: the heights it
        claims there; printed rings on the sleeve fill them, :meth:`web_claims`; ``gap``:
        the one over the lowest web). The round standoff's rule (:meth:`fit_web`'s ``gap``)
        for the hex, whose stock lengths are 5 mm apart past 25 mm: no length fits a span
        in 25.7-27.6 mm (25.7-28.8 capped in the hub plate), which kept the Strider decker
        and quad off the hex crank (the debug of 2026-10-05)."""
        from spiderpig.stack import GAP_MAX, GAP_STEP

        room = sum(max(0.0, GAP_MAX - h) for _, h in slots)
        for i in range(1, int(room / GAP_STEP + 1e-6) + 1):
            dz = i * GAP_STEP
            j = self.fit_hex(span + dz, t_lo, t_hi, sleeve=sleeve, out_hi_max=out_hi_max,
                             capped=capped, air_hi=air_hi)
            if j is None:
                continue
            left, gaps = dz, []
            for k, h in slots:
                take = min(left, max(0.0, GAP_MAX - h))
                if take > EPS:
                    gaps.append((k, round(h + take, 3)))
                    left -= take
            w0 = slots[0][0]
            return replace(j, gap=dict(gaps).get(w0, 0.0), gaps=tuple(sorted(gaps)))
        return None

    def hex_gap_needed(self, L: Layout, lo: int, hi: int, t: float, hub: int | None,
                       capped: bool) -> bool:
        """Whether the clearance gap the plan has over a hex chain's lowest web (layer
        ``lo - 1``) is one its standoff needs: no stock length fits the span without it
        (:meth:`hex_gap_fit` opened it)."""
        w0, w1 = lo - 1, hi + 1
        g = L.gap(w0)
        if not self.hex or g <= 0:
            return False
        span = self.plate_z(L, w1, t, hub)[1] - self.plate_z(L, w0, t, hub)[0] - g
        hi_max = 0.0 if capped or w1 == hub else None
        return self.fit_hex(span, t, t, out_hi_max=hi_max, capped=capped,
                            air_hi=self.air_over(L, w1, t, hub)) is None

    def air_over(self, L: Layout, k: int, t: float, hub: int | None = None) -> float:
        """The air over the crank plate in layer ``k`` inside its own layer (a plate thinner
        than its layer sits on the floor; the hub plate at the top: none): a crankpin's
        stack over that plate uses it before the clearance gap above."""
        return max(0.0, L.z(k)[1] - self.plate_z(L, k, t, hub)[1])

    def _web_span_ok(self, run: int, low: int, pitch: float, t: float) -> bool:
        """Whether a chain whose webs are ``run`` layers apart takes a stock standoff at the
        nominal z or up to an aluminium plate's thickness more (the exact z decides)."""
        air = max(0.0, pitch - t)
        if self.hex:
            # outer faces: the lower web on its layer's floor, the upper's top face, the run's
            # layers between (and up to two clearance gaps the plan may put among them)
            span = max(pitch, t) + run * pitch + t
            # (two webs in adjacent layers: no gap between them, nothing short enough joins
            # them, as the screws into a 5-6 mm standoff's ends would meet)
            return any(self.fit_hex(span + dz, t, t) is not None
                       for dz in ((0.0, 1.0, 1.6, 2.0, 2.6, 3.2) if run else (0.0,)))
        for dz in (0.0, run * 0.175, run * 0.175 + 1.0, air):
            if self.fit_web(run * pitch + dz, air, t, t) is not None:
                return True
        return False

    def stub_layers_web(self, t0: float, pitch: float, t: float, most: int = 64
                        ) -> frozenset[int]:
        """The layers the first chain's lowest web may sit in: the stub standoff from its
        underside down into the outer frame plate (screwed to it from above)."""
        return frozenset(a for a in range(2, most)
                         if self.stub_z(t0 + (a - 1) * pitch, t0, t) is not None)

    horn_shim_max: float = 2.0       # DIN 988 shims under a horn screw's head, at most (mm)

    def horn_joint_web(self, ctx: Context, seg: float, spacer: float
                       ) -> tuple[ScrewKind, float, float] | None:
        """:meth:`horn_fit_web` without its shims: (kind, length, thread in the horn)."""
        got = self.horn_fit_web(ctx, seg, spacer)
        return None if got is None else got[:3]

    def horn_fit_web(self, ctx: Context, seg: float, spacer: float
                     ) -> tuple[ScrewKind, float, float, float] | None:
        """The horn screws up through the crank's top plates (``seg`` mm: the hub and what
        is under it, their heads under the lowest) and the horn spacer (``spacer``) into
        the horn: (kind, length, thread in the horn, DIN 988 shims under the head). Most
        thread, then no shims, then shortest. A stock length too long for the plates (a
        horn whose thread window is short, the XL430's 1.5-2.0 mm, against a thin hub
        plate) takes 0.1 mm steps of shims under its head (up to ``horn_shim_max``, in the
        gap under the hub where its head hangs)."""
        from spiderpig.hardware.fasteners import SCREWS
        from spiderpig.stack import GAP_MAX

        spec = ctx.servo
        pat = spec.horn.pattern
        size = SIZES.get(pat.thread)
        order = ("self_tap",) if pat.tapping else ("bhcs", "shcs")
        kinds = [SCREWS[(k, size)] for k in order if (k, size) in SCREWS]
        reach = pat.reach if pat.reach is not None else spec.horn.thickness
        # the tip stays HORN_TIP_CLEAR under the inner plate's top face (inside the horn)
        e_max = min(reach, spec.horn_face_depth - HORN_TIP_CLEAR)
        e_want = min(pat.thread_depth if pat.thread_depth is not None else e_max, e_max)
        e_min = min(e_want, 1.5)
        for sk in kinds:
            best = None
            for L in sk.lengths:
                e = L - seg - spacer
                shim = 0.0
                if e > e_max + EPS:
                    # shims to the wanted thread, else to the most the horn takes; no more
                    # than the gap under the hub holds over the head
                    most = min(self.horn_shim_max,
                               GAP_MAX - sk.head_h - self.head_clear)
                    shim = math.ceil((e - e_want) * 10 - 1e-6) / 10
                    whole = float(math.ceil(e - e_max - 1e-6))
                    if e - whole >= e_min - EPS:
                        shim = whole       # whole 1 mm shims (DIN 433 pairs), thread to e_max
                    if shim > most + EPS:
                        shim = math.ceil((e - e_max) * 10 - 1e-6) / 10
                    if shim > most + EPS:
                        continue
                    e -= shim
                if not e_min - EPS <= e <= e_max + EPS:
                    continue
                rank = (-min(e, e_want), shim, L)
                if best is None or rank < best[0]:
                    best = (rank, (sk, L, round(e, 3), round(shim, 2)))
            if best is not None:
                return best[1]
        return None

    @staticmethod
    def horn_hole(ctx: Context) -> float:
        """A horn screw's hole in the hub plate: its clearance hole, or the least hole the
        service cuts in the crank's sheet (SendCutSend: the thickness; an M2 head still
        bears on the ring round it)."""
        drive: DriveInterface = ctx.interfaces["drive"]
        key = ctx.sheet("crank")
        if key is None:
            return drive.screw_clearance_d
        from spiderpig.materials import sheet

        return max(drive.screw_clearance_d, sheet(key).min_hole + 0.025)

    def horn_head_r(self, ctx: Context, shim: float | None = None) -> float:
        """A horn screw head's clearance shape under the hub: the head, or with ``shim`` mm
        of DIN 988 shims under it (6 mm OD) the wider of them; ``None``: whether the plan's
        nominal z needs shims."""
        from spiderpig.hardware.catalog import get

        if shim is None:
            drive: DriveInterface = ctx.interfaces["drive"]
            got = self.horn_fit_web(ctx, ctx.sheet_t("crank"), drive.spacer_t)
            shim = got[3] if got is not None else 0.0
        d = self.horn_kind(ctx).head_d
        if shim > 0:
            d = max(d, float(get(SHIM_KEY).dims["od"]))
        return d / 2 + 0.3

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
        if self.hex:
            head = self.hex_stack(0.0, 0) + clear
        t = plate_t
        spacer_r = (self.rider_d() / 2 + 0.5) if self.hex else self.pin_od / 2 + 1.0

        def heights(j, w1: int) -> tuple[float, float]:
            """What hangs under the lowest web and over the top web (in layer ``w1``; with
            clearance): the hex pin's upper stack in the gap over that web's layer less the
            air over the plate in it."""
            if isinstance(j, HexJoint):
                up = max(0.0, j.stack_hi + clear - self.air_over(L, w1, t, hub))
                return round(j.stack_lo + clear, 3), round(up, 3)
            return head, head

        for ch in chains:
            at, lo, hi = ch[0].at, ch[0].lo, ch[-1].hi
            capped = hi + 1 == hub and self.hub_capped(ctx, at)
            j = self.chain_fit_web(L, lo, hi, t=t, hub=hub, capped=capped)
            if j is None and L.final:
                raise Unbuildable(f"at the plan's z no stock standoff fits the crankpin at {at} "
                                  f"(webs in layers {lo - 1} and {hi + 1})")
            if lo - 2 < 0:
                raise Unbuildable(f"the screw under crankpin {at} needs layer {lo - 2}")
            h_lo, h_hi = heights(j, hi + 1)
            out.append(Placed(lo - 2, Disc(at, hr), GROUP, f"crankpin screw {at}", gap=True,
                              height=h_lo, toward=-1))
            if self.hex and j is not None and (j.gap > 0 or self.hex_gap_needed(
                    L, lo, hi, t, hub, capped)):
                # the gap over the lowest web the standoff's length opened (hex_gap_fit): a
                # printed ring on the sleeve fills it, so the lowest rider keeps its layer
                # (_WebPlates.gap_parts prints it, as the run's rings)
                out.append(Placed(lo - 1, Disc(at, self.washer_r), GROUP,
                                  f"crankpin {at} washer", gap=True, height=j.gap))
            if isinstance(j, HexJoint):
                # the gaps along the run it needs (their rings: the run's washer claims)
                out += [Placed(k, Disc(at, spacer_r), GROUP, f"crankpin {at} gap", gap=True,
                               height=h) for k, h in j.gaps if k != lo - 1]
            if capped:
                # under the horn spacer, which caps its upper end: no screw there
                out.append(Placed(lo - 1, Disc(at, spacer_r), GROUP,
                                  f"crankpin {at} spacer", gap=True, height=0.0))
                continue
            out.append(Placed(hi + 1, Disc(at, hr), GROUP, f"crankpin screw {at}", gap=True,
                              height=h_hi, toward=1))
            # the standoff and its shims through the gap over the lowest web (the height:
            # its length past the layers and the air over the plate); the hex pin's sleeve
            out.append(Placed(lo - 1, Disc(at, spacer_r), GROUP,
                              f"crankpin {at} spacer", gap=True,
                              height=j.gap if j is not None else 0.0))
            g = ctx.topo.geometry.points
            R = float(np.linalg.norm(g[at][0] - g["O"][0]))
            if hi + 1 == hub and drive.horn_radius + hr + ctx.params.margin > R:
                # its head over the hub plate stands in a pocket of the printed horn spacer
                # (servos.mount.DriveGroup.realize cuts it), which must be that thick
                spacer = (self.horn_spacer(L, drive, hub) if L.final else drive.spacer_t)
                if spacer < h_hi - 1e-6:
                    raise Unbuildable(f"the screw head of crankpin {at} over the hub plate "
                                      f"needs {h_hi:.2f} mm of the horn spacer, which is "
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
            j = self.chain_fit_web(L, e + 1, f - 1, t=t, hub=hub, sleeve=False)
            if j is None and L.final:
                raise Unbuildable(f"at the plan's z no stock standoff joins the crank's webs "
                                  f"in layers {e} and {f} on O")
            h_lo, h_hi = heights(j, f)
            out += [Placed(k, Disc("O", d.journal), GROUP, "crank journal")
                    for k in range(e + 1, f)]
            out.append(Placed(e - 1, Disc("O", hr), GROUP, "journal screw", gap=True,
                              height=h_lo, toward=-1))
            out.append(Placed(f, Disc("O", hr), GROUP, "journal screw", gap=True,
                              height=h_hi, toward=1))
            out.append(Placed(e, Disc("O", d.journal if self.hex else self.pin_od / 2 + 1.0),
                              GROUP,
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
        got = self.horn_fit_web(ctx, t, self.horn_spacer(L, drive, hub) if L.final
                                else drive.spacer_t)
        if L.final and got is None:
            raise Unbuildable(f"at the plan's z no stock horn screw fits the crank's top "
                              f"plates (layers {seg}..{hub}) and the horn spacer")
        shim = got[3] if got is not None else 0.0
        if got is not None and got[0].head_h > sk.head_h:
            sk = got[0]
        for name in horn_pts:
            out.append(Placed(seg - 1, Disc(name, self.horn_head_r(ctx, shim)), GROUP,
                              "horn screw head", gap=True, height=sk.head_h + shim + clear,
                              toward=-1))
        return out

    def hex_capacity(self, joint: HexJoint | None = None) -> dict[str, float]:
        """What one hex crankpin holds (N·m): the hex in each web's pocket (the shallower
        counts), one bearing model (:func:`hex_bearing_nm`) at the weaker of the plate's
        and the standoff's yield (``web_yield`` 193 MPa on 5052-H32, 276 on 6061-T6;
        ``hex_yield`` 300 MPa steel), each flat short by ``hex_corner_loss`` for the
        standoff's rounded corners (the dog-bone reliefs leave the pocket's flats whole);
        and the standoff's own torsion (the tube in its flats round the M3 thread)."""
        eng = min(joint.engaged_lo, joint.engaged_hi) if joint else self.web_t
        p = min(self.web_yield, self.hex_yield)
        bearing = hex_bearing_nm(self.hex_af, eng, p, self.hex_corner_loss)
        zp = math.pi * (self.hex_af ** 4 - 3.0 ** 4) / (16 * self.hex_af)
        torsion = self.hex_yield / math.sqrt(3) * zp / 1e3
        return {
            f"hex {self.hex_af:g} AF in its plate's pocket, {eng:g} mm at {p:g} MPa": round(
                bearing, 3),
            f"hex standoff torsion ({self.hex_yield:g} MPa)": round(torsion, 3),
        }

    # -- parts ----------------------------------------------------------------------------

    def realize(self, group: CrankGroup, build: Build) -> Realized:
        return _WebPlates(self, group, build).make()


class _WebPlates:
    """The bolt crank of one side (:meth:`BoltCrank.realize`): every crank layer one
    aluminium plate (on its layer's floor; the hub plate at its layer's top, against the horn
    spacer), every crankpin and journal a hex standoff in hex pockets of its two webs (or a
    round standoff clamped between them by M4 screws), the stub standoff screwed to the
    lowest web from above, the horn screws up through the hub plate."""

    def __init__(self, c: BoltCrank, group: CrankGroup, build: Build):
        self.c, self.build = c, build
        ctx = build.ctx
        self.p = ctx.params
        self.d = group.dims(ctx)
        topo = build.plan.topo
        self.host = topo.crank_bodies[0]
        self.drive: DriveInterface = ctx.interfaces["drive"]
        self.pins = topo.axes_of("crankpin")
        self.route = route_of(build.plan.layout, self.pins)
        self.chains = chains_of(sorted(self.route.runs, key=lambda r: (r.lo, r.at)))
        self.plates: dict[int, list] = {}
        self.washers: list = []                 # crankpin washers in a gap
        for pl in build.shapes(GROUP):
            if pl.gap:
                if pl.label.endswith(" washer"):
                    self.washers.append(pl)
                continue
            if pl.label in ("crank body", "crank hub") or pl.label.startswith("web "):
                self.plates.setdefault(pl.layer, []).append(pl)
        self.sheet = ctx.sheet("crank")
        self.t = ctx.sheet_t("crank")
        self.cuts: dict[int, list] = {k: [] for k in self.plates}
        self.out = Realized()
        self.notes: list[dict] = []
        hub = [pl.layer for pl in build.shapes(GROUP) if pl.label == "crank hub"]
        self.hub_layer = max(hub) if hub else None

    def xy(self, point: str) -> tuple:
        return tuple(self.build.xy(point))

    def buy(self, name: str, part, key: str, color: str = STEEL) -> None:
        self.out.bodies.append(hardware(name, part, self.host, fab="purchased", bom_key=key,
                                        color=color))

    def gap_parts(self, out: Realized) -> None:
        """What fills a clearance gap a crankpin's run crosses between its riders: a printed
        ring on the hex pin's sleeve, a stack of washers on the round standoff (none where a
        horn screw's head is in that gap: the rider turns on air)."""
        from spiderpig.materials import washer_stack

        b = self.build
        horn = [pl for pl in b.shapes(GROUP) if pl.label == "horn screw head" and pl.gap]
        for pl in self.washers:
            g = b.plan.gaps.get(pl.layer, 0.0)
            if g <= 0:
                continue
            if any(h.layer == pl.layer and math.dist(self.xy(h.shape.at), self.xy(pl.shape.at))
                   < h.shape.r + pl.shape.r for h in horn):
                continue        # a horn screw's head is there: the rider turns on air
            z0, _ = b.plan.gap_z(pl.layer)
            if self.c.hex:
                # a printed ring on the riders' sleeve (no stock washer fits its 8.5 mm)
                xy = self.xy(pl.shape.at)
                ring = (disc(xy, pl.shape.r - 0.05, z0 + 0.05, z0 + g - 0.05)
                        - disc(xy, (self.c.sleeve_od + self.p.print_fit) / 2, z0 - 1,
                               z0 + g + 1))
                out.bodies.append(hardware(f"crank_ring_{pl.shape.at}_{pl.layer}", ring,
                                           self.host, fab="printed", color=SEGMENT_COLOR))
                continue
            items, _ = washer_stack(self.c.bolt_d, g)
            if not items:
                continue
            t = sum(x[1] for x in items)
            xy = self.xy(pl.shape.at)
            od = 2 * pl.shape.r
            part = disc(xy, od / 2, z0, z0 + t) - disc(xy, self.c.bore / 2, z0 - 1, z0 + t + 1)
            tag = f"{pl.shape.at}_{pl.layer}"
            self.buy(f"crank_washers_{tag}", part, items[0][0], "#f2f2f2")
            for key, tt in items[1:]:     # each shim by its thickness (the BOM orders them so)
                out.extras.append(BomLine(key, 1, f"crankpin {tag}: {tt:g} mm in the gap"))

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

        c, b = self.c, self.build
        if c.hex:
            return self.hex_pin(at, xy, tag, lo, hi)
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
            # DIN 988 4 x 8 shims to the clamp's take-up, thickest first (the BOM orders
            # them per thickness, hardware.bom.split_shims); no PTFE washer in a clamp
            from spiderpig.hardware.bom import shim_breakdown

            key = "shim_din988_4x8"
            items = [(key, t) for t in shim_breakdown(j.shims, get(key).dims["t"])]
            t = sum(x[1] for x in items)
            if 0 < z_lo + t - z0 <= 0.1:
                # the stack's rounding (stock shim steps and the fit's tolerance) leaves the
                # standoff a hair onto the shims: they are what gives, drawn up to its end
                t = z0 - z_lo
            shim = disc(xy, 3.95, z_lo, z_lo + t) - disc(xy, 2.05, z_lo - 1, z_lo + t + 1)
            self.buy(f"crank_pin_shims_{tag}", shim, items[0][0], "#9a9a9a")
            # the stack as bought (the body may be drawn a hair short of it, above): the BOM
            # orders these thicknesses, not a breakdown of the drawn height
            self.out.notes.setdefault("shim_stacks", {})[f"crank_pin_shims_{tag}"] = [
                x[1] for x in items]
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

    def hex_pin(self, at: str, xy, tag: str, lo: int, hi: int) -> dict:
        """A hex standoff between the webs in layers ``lo - 1`` and ``hi + 1`` at ``xy``
        (``at`` "O": a journal, no riders): its hex ends in the webs' hex pockets, an M3
        button head and a DIN 9021 washer into each end, printed hex collars where it stands
        past a plate, the riders' printed sleeve between the plates; its note."""
        c, b = self.c, self.build
        journal = at == "O"
        capped = hi + 1 == self.hub_layer and not journal and c.hub_capped(b.ctx, at)
        j = c.chain_fit_web(b.plan.layout, lo, hi, t=self.t, hub=self.hub_layer,
                            sleeve=not journal, capped=capped)
        if j is None:
            raise ConstructionError(f"no stock hex standoff fits the crankpin at {at} (webs in "
                                    f"layers {lo - 1} and {hi + 1})")
        w0, w1 = lo - 1, hi + 1
        ang = 0.0 if journal else b.angle("O", at)
        z_lo, z_hi = self.pz(w0)[0], self.pz(w1)[1]          # the plates' outer faces
        for k in (w0, w1):
            if k in self.cuts:
                z0, z1 = self.pz(k)
                self.cuts[k].append(c.hex_cut(xy, z0 - 1, z1 + 1, ang))
        # a crank plate under or over the chain at the pin (the stub plate, a hub plate)
        self.layer_cut(w0 - 1, xy, c.head_r())
        self.layer_cut(w1 + 1, xy, c.head_r())
        e0, e1 = z_lo - j.out_lo, z_hi + j.out_hi            # the standoff's ends
        st = _hex(xy, c.hex_af, e0, e1, ang) - disc(xy, 1.5, e0 - 1, e1 + 1)   # M3, through
        self.buy(f"crank_pin_{tag}", st, j.standoff, "#b9b9b9")
        hd, hh = c.hex_screw()
        _, wod, wt = c.hex_washer()
        et = c.extra_washer_t()
        pocket = c.hex_pocket_af() + c.sleeve_fit - c.hex_fit
        for side, end, collar, key, k, s_ in (("lo", e0, j.collar_lo, j.screw_lo, j.washers_lo,
                                                -1.0),
                                               ("hi", e1, j.collar_hi, j.screw_hi, j.washers_hi,
                                                1.0)):
            face = z_lo if side == "lo" else z_hi
            if not key:
                continue                    # capped by the horn spacer: no screw
            # the washer on the standoff's end, or on the plate when the end is recessed
            end = min(end, face) if side == "lo" else max(end, face)
            if collar > 0:
                zc = sorted((face, face + s_ * collar))
                ring = disc(xy, wod / 2 - 0.05, *zc) - _hex(xy, pocket, zc[0] - 1, zc[1] + 1,
                                                            ang)
                self.out.bodies.append(hardware(f"crank_pin_collar_{side}_{tag}", ring,
                                                self.host, fab="printed", color=SEGMENT_COLOR))
            zw = sorted((end, end + s_ * wt))
            washer = disc(xy, wod / 2, *zw) - disc(xy, 1.6, zw[0] - 1, zw[1] + 1)
            self.buy(f"crank_pin_washer_{side}_{tag}", washer, "m3_washer_9021", "#d0d0d0")
            bearing = end + s_ * wt
            if k:
                zx = sorted((bearing, bearing + s_ * k * et))
                extra = disc(xy, 3.5, *zx) - disc(xy, 1.6, zx[0] - 1, zx[1] + 1)
                self.buy(f"crank_pin_washers_{side}_{tag}", extra, "m3_washer", "#d0d0d0")
                if k > 1:
                    self.out.extras.append(BomLine("m3_washer", k - 1, f"crankpin {tag}"))
                bearing += s_ * k * et
            sk, length = screw_from_key(key)
            self.buy(f"crank_pin_screw_{side}_{tag}",
                     screw_body(xy, sk, bearing, length, up=side == "lo"), key)
        press = capped
        if j.sleeve > 0:
            z0 = self.pz(w0)[1] + (self.pz(w1)[0] - self.pz(w0)[1] - j.sleeve) / 2
            bore = pocket
            if press:
                # pressed on the hex resting on the lower plate (the washer under that plate
                # drawn up against it), so the play is all over it; drawn a hair over the AF
                # (the print is ``capped_press`` under it)
                z0, bore = self.pz(w0)[1], c.hex_af + PRESS_DRAWN
            sleeve = (disc(xy, c.sleeve_od / 2, z0, z0 + j.sleeve)
                      - _hex(xy, bore, z0 - 1, z0 + j.sleeve + 1, ang))
            self.out.bodies.append(hardware(f"crank_pin_sleeve_{tag}", sleeve, self.host,
                                            fab="printed", color=SEGMENT_COLOR))
        if c.lock_key is not None:
            self.out.extras.append(BomLine(c.lock_key, 2 * c.lock_per_bolt,
                                           f"crankpin {tag} screws"))
        return {
            "at": tag, "standoff": j.standoff, "length_mm": j.length, "span_mm": j.span,
            "capped": capped,
            "segments_mm": [j.length], "out_mm": [j.out_lo, j.out_hi],
            "collars_mm": [j.collar_lo, j.collar_hi],
            "screws": [x for x in (j.screw_lo, j.screw_hi) if x],
            "engage_mm": [j.engage_lo, j.engage_hi], "washers": [j.washers_lo, j.washers_hi],
            "hex_engaged_mm": [j.engaged_lo, j.engaged_hi], "sleeve_mm": j.sleeve,
            "sleeve_press_mm": c.capped_press if press and j.sleeve > 0 else 0.0,
            "run_layers": hi - lo + 1, "layers": [lo - 2, hi + 1],
            "capacity_nm": c.capacity(j)}

    def assembly_issue(self) -> str | None:
        """Why this crank can't be put together, or ``None``: a chain that ends in the hub
        plate with a screw over it (the round standoff's clamp, ``bolt_round``).
        The horn screws come up through the hub plate from below, so the hub plate, the horn,
        the servo and the inner plate go on as one unit (:data:`construction.robot.ASSEMBLY`)
        once the stack under the hub plate is closed, which buries that screw's head under
        the horn spacer; and built the other way round (the hub plate screwed to the chain
        first), the horn screws' heads are inside the closed stack. The assembly audit of
        2026-10-04 (the Strider quad's J1_leg3, ``klann_lego``'s M_leg2)."""
        for n in self.notes:
            if (self.hub_layer is not None and n["layers"][1] == self.hub_layer
                    and not n.get("capped") and len(n["screws"]) > 1):
                return (f"crankpin {n['at']}'s chain ends in the hub plate (layer "
                        f"{self.hub_layer}) with a screw over it ({n['screws'][-1]}), and the "
                        "horn screws come up through the hub plate from below: no order "
                        "drives both (the hub plate, horn, servo and inner plate go on as one "
                        "unit once the stack under it is closed; construction.robot.ASSEMBLY)"
                        "; a crank that caps that chain (the hex crank, --crank bolt) has an "
                        "order, the round standoff's friction clamp needs both screws")
        return None

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
        if c.hex and any(n.get("capped") for n in self.notes):
            # the crank body's stop toward the outer plate: a printed sleeve round the stub
            # from the lowest web's underside to just over the outer plate's inner face,
            # bearing on the plate round the journal's hole
            zb = b.z(0)[1] + c.thrust_play
            ring = (disc(o, c.thrust_od / 2, zb, top)
                    - disc(o, (od + self.p.print_fit) / 2, zb - 1, top + 1))
            self.out.bodies.append(hardware("crank_stub_thrust", ring, self.host,
                                            fab="printed", color=SEGMENT_COLOR))
            self.thrust = {"od_mm": c.thrust_od, "length_mm": round(top - zb, 3),
                           "play_mm": c.thrust_play}

    def hub(self) -> None:
        c, b, drive = self.c, self.build, self.drive
        hub = sorted(pl.layer for pl in b.shapes(GROUP) if pl.label == "crank hub")
        if not hub:
            return
        top = hub[-1]
        seg = top
        spacer = c.horn_spacer(b.plan.layout, drive, top)
        got = c.horn_fit_web(b.ctx, self.t, spacer)
        if got is None:
            raise ConstructionError(f"no screw fits the bolt crank's top plates and the "
                                    f"{b.ctx.servo.key} horn")
        sk, L, _, shim = got
        o = np.asarray(self.xy("O"))
        theta = b.angle("O", self.pins[0].name) + drive.pattern_angle
        face = self.pz(top)[0]
        bearing = face - shim
        shims = shim_stack(shim)
        for i in range(drive.screw_count):
            a = theta + 2 * math.pi * i / drive.screw_count
            xy = tuple(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]))
            for k in range(seg, top + 1):
                self.layer_cut(k, xy, c.horn_hole(b.ctx) / 2)
            self.buy(f"crank_horn_screw{i}", screw_body(xy, sk, bearing, L), sk.key(L))
            if shims:
                # DIN 988 shims under the head: a stock length too long for the hub plate
                ring = disc(xy, 2.95, bearing, face) - disc(xy, 1.55, bearing - 1, face + 1)
                self.buy(f"crank_horn_shims{i}", ring, SHIM_KEY, "#9a9a9a")
                if len(shims) > 1:
                    self.out.extras.append(BomLine(SHIM_KEY, len(shims) - 1,
                                                   f"horn screw {i}: shims"))
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
                out.cut(m, Cut(self.xy(pin.name), self.p.hole(self.c.rider_d())))
        out.notes["crank_bolt"] = {
            "webs": "single",
            "crankpin": ("hex standoff in hex pockets, the riders on a printed sleeve"
                         if c.hex else "round standoff clamped by M4 screws"),
            "chains": self.notes, "journals": self.journals, "plates": n_plates,
            "stub_thrust": getattr(self, "thrust", None),
            "assembly": self.assembly_issue(),
            # (the acrylic stacks' keys, kept empty so stored audits compare)
            "segments": [], "sheet_mm": self.t,
            "weakest_bond": None, "threadlocker": c.lock_key}
        if c.hex:
            out.notes["crank_bolt"].update({
                "pocket_af_mm": round(c.hex_pocket_af(), 3), "dogbone_r_mm": c.dogbone_r,
                "play_deg": round(2 * hex_play(c.hex_af, c.hex_pocket_af()), 2)})
        return out


@functools.lru_cache(maxsize=1 << 16)
def _fit_hex(c: BoltCrank, *args) -> HexJoint | None:
    return c._fit_hex(*args)


@functools.lru_cache(maxsize=1 << 14)
def _hex_gap_fit(c: BoltCrank, *args) -> HexJoint | None:
    return c._hex_gap_fit(*args)


BOLT_ROUND = BoltCrank(
    key="bolt_round", pin="round",
    label=("laser-cut crank, its crankpins round goBILDA standoffs clamped between single "
           "aluminium webs by M4 screws (friction; the default before the hex standoff of "
           "2026-10-04)"))
"""The friction-clamped round-standoff crankpin (``--crank bolt_round``): TrotBot's heel and
toe (:data:`config.LINKAGE_CRANKS`), where the hex crankpin's 8.5 mm sleeve doesn't clear b7."""
