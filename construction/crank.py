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
be threaded onto its crankpin); b1s of one crankpin in adjacent layers (two
legs on one pin) share a split. Going up from the outer frame plate:

* **segments**: the bottom one carries the journal stub; the top one is the
  hub. Each is the union of its claimed shapes, except that a web's face
  toward the b1 above it is set back by ``axial_play``, which is the b1's
  end play;
* **crankpins**: a printed post (``Params.crankpin_d``) on the segment below
  each split. It runs through the b1 layer(s) and butts against the web of
  the segment above;
* **joints**: an axial M3 screw runs through each post. Its head is recessed
  into the lower web from below and it screws into a hex nut trapped in the
  upper web (pocket open away from the b1). An ISO 4762 head is 3 mm tall
  and doesn't fit a 3 mm web with any floor, so at that pitch the screw is
  an ISO 7380 button head (:data:`POST_SCREWS` is the order of preference);
* **hub**: bolts to the horn with the horn's screws from below, through the
  hub into the horn's holes (through the drive's horn spacer, if any). The
  heads sit in counterbores in the hub, reached through tunnels in the rest
  of the top segment; a pocket clears the horn's centre screw. The hub's top
  face is the horn's outer face, and within ``Params.margin`` of the inner
  frame plate the hub is no wider than the horn (it turns inside the plate's
  horn hole).

Assembly: servo on the inner plate; the top segment bolted to the horn
(nut in its trap first); then, going down, each b1 onto its post and the
next segment up to it, screwed from below; the outer frame plate last, over
the journal stub.

Screw lengths are standard lengths (:func:`hardware.catalog.pick_length`
style) chosen for the most thread engagement that keeps every head, nut and
tip inside the crank's claims.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

import numpy as np
from build123d import Axis, Box, Location

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
from construction.envelope import shape_solid
from hardware.parts import SHCS_LENGTHS, shcs
from shapes import Cut, disc, union
from stack import Claim, Disc, Keepout, Layout, Pill, Placed, Unbuildable

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


@dataclass(frozen=True)
class Run:
    """The crankshaft runs along the post at ``at`` over layers ``lo``..``hi``.

    ``at`` is a crankpin, or a detour point fixed to the crank (a point of the
    plan's geometry that turns with it). Webs from O lead in at ``lo - 1`` and
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


def add_crank_point(topo, name: str, r: float, angle_deg: float) -> str:
    """Add a point fixed to the crank to ``topo``'s geometry: ``r`` from O, ``angle_deg``
    counter-clockwise from the first crankpin. Returns its name (a detour run's ``at``)."""
    return topo.add_crank_point(name, r, angle_deg)


def route_of(layout: Layout, pins) -> CrankRoute:
    """The route the planner chose (``layout.choices["crank"]``), else :func:`default_route`."""
    chosen = layout.choices.get(GROUP)
    return chosen if chosen is not None else default_route(layout, pins)


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
        from construction.route import CrankRouter, crank_facts

        d = self.dims(ctx)
        return CrankRouter(ctx, d, crank_facts(ctx, d, envelope, margin), drop_bearing)

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

        def hub(L: Layout):
            horn, hub = hub_layers(L, drive, d.hub_thickness)
            out = [Placed(k, Disc("O", drive.horn_radius), GROUP, "servo horn", seat=k >= L.top)
                   for k in horn if k <= L.top]
            out += [Placed(k, Disc("O", d.hub), GROUP, "crank hub") for k in hub]
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
                out += [Placed(k, Pill("O", r.at, d.web), GROUP, f"web {r.at}")
                        for k in (r.lo - 1, r.hi + 1)]
            run_layers = {k for r in route.runs for k in range(r.lo, r.hi + 1)}
            web_layers = {k for r in route.runs for k in (r.lo - 1, r.hi + 1)} - run_layers
            _, hub = hub_layers(L, drive, d.hub_thickness)
            lo, hi = min(web_layers), min(hub)
            if lo > hi:
                raise Unbuildable(f"its lowest web (layer {lo}) would sit above the hub under "
                                  f"the servo horn (layer {hi}): the riders are too high")
            out += [Placed(k, Disc("O", d.journal), GROUP, "crank body")
                    for k in range(lo, hi) if k not in run_layers]
            if route.bearing:
                out += [Placed(k, Disc("O", d.stub), GROUP, "journal stub") for k in range(1, lo)]
                out.append(Placed(0, Disc("O", d.stub), GROUP, "journal stub", seat=True))
            return out

        return [Claim("crank hub", frozenset(), hub),
                Claim("crank route", frozenset(riders), shaft, choice=GROUP)]

    def realize(self, build: Build) -> Realized:
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


@dataclass(frozen=True)
class Gap:
    """A run of adjacent b1 layers on one crankpin: where the crank is split."""

    lo: int
    hi: int
    pin: str


def _gaps(rider_layers: dict[int, str]) -> list[Gap]:
    out: list[Gap] = []
    for k in sorted(rider_layers):
        pin = rider_layers[k]
        if out and out[-1].hi == k - 1 and out[-1].pin == pin:
            out[-1] = Gap(out[-1].lo, k, pin)
        else:
            out.append(Gap(k, k, pin))
    return out


def _hex(xy, af: float, z0: float, z1: float, angle: float):
    """A hexagonal prism, ``af`` across flats, one pair of flats facing ``angle``."""
    ang = math.degrees(angle)
    boxes = [Box(af, 4 * af, z1 - z0).rotate(Axis.Z, ang + a) for a in (0.0, 60.0, 120.0)]
    prism = boxes[0] & boxes[1] & boxes[2]
    return prism.moved(Location((float(xy[0]), float(xy[1]), (z0 + z1) / 2)))


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
        # the tightest crankpin joint: one b1 between two webs, both next to other b1s
        pitch = ctx.pitch
        if self.post_joint(0.0, pitch - self.axial_play, 2 * pitch,
                           3 * pitch - self.axial_play) is None:
            raise ConstructionError(
                f"no M3 screw and nut fit a crankpin joint in {pitch} mm layers")
        spec = ctx.servo
        top = 100                       # any stack: the hub's height only depends on the pitch
        plate_top = (top + 1) * pitch
        face = plate_top - drive.horn_face_depth
        hub_bottom = Layout({}, top, pitch).layers_between(face - p.hub_thickness, face).start
        if self.horn_joint(drive, spec, face - hub_bottom * pitch) is None:
            raise ConstructionError(
                f"no {spec.horn.pattern.thread} screw fits between the crank hub and the "
                f"{spec.key} horn")
        return dims

    def post_joint(self, zl0: float, zl1: float, zu0: float, zu1: float) -> PostJoint | None:
        """The screw for a joint: lower web ``zl0..zl1``, upper web ``zu0..zu1``.

        The head sits in a counterbore in the lower web's bottom face, the nut
        in a trap in the upper web's top face. Most thread in the nut wins,
        then the shorter screw; the first screw kind that fits is used.
        """
        for sk in POST_SCREWS:
            hp_lo = sk.head_h + self.head_recess
            hp_hi = (zl1 - zl0) - self.min_web_floor
            np_lo = NUT_H + self.head_recess
            np_hi = (zu1 - zu0) - self.min_nut_floor
            if hp_lo > hp_hi + EPS or np_lo > np_hi + EPS:
                continue
            best: PostJoint | None = None
            for length in sk.lengths:
                hp = min(hp_hi, zu1 - self.tip_recess - zl0 - length)
                if hp < hp_lo - EPS:
                    continue
                tip = zl0 + hp + length
                nd = min(np_hi, max(np_lo, zu1 - tip + NUT_H))
                engage = min(NUT_H, tip - (zu1 - nd))
                if engage < self.min_nut_engage - EPS:
                    continue
                if best is None or engage > best.engagement + EPS:
                    best = PostJoint(sk, length, hp, nd, engage)
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

    # -- parts ----------------------------------------------------------------------

    def realize(self, group: CrankGroup, build: Build) -> Realized:
        out = Realized()
        ctx = build.ctx
        params = ctx.params
        d = group.dims(ctx)
        topo = build.plan.topo
        host = topo.crank_bodies[0]
        drive: DriveInterface = ctx.interfaces["drive"]
        spec = ctx.servo
        plate_bottom, plate_top = build.z(build.top)
        face = plate_top - drive.horn_face_depth
        play = self.axial_play
        route = route_of(build.plan.layout, topo.axes_of("crankpin"))
        rider_layers = {k: r.at for r in route.runs for k in range(r.lo, r.hi + 1)}
        gaps = [Gap(r.lo, r.hi, r.at) for r in sorted(route.runs, key=lambda r: r.lo)]
        claimed = [p for p in build.shapes(GROUP)
                   if p.label != "servo horn" and not p.label.startswith("crankpin")]

        def span(p: Placed) -> tuple[float, float]:
            z0, z1 = build.z(p.layer)
            if p.layer + 1 in rider_layers:
                z1 -= play                 # b1's end play
            if p.label == "crank hub":
                z1 = min(z1, face)         # the hub stops at the horn's outer face
            return z0, z1

        # segments: the layers between splits
        ranges = []
        lo = 0
        for g in gaps:
            ranges.append((lo, g.lo - 1))
            lo = g.hi + 1
        ranges.append((lo, build.top - 1))
        segs: list[list] = [[] for _ in ranges]
        zspan: list[list[float]] = [[math.inf, -math.inf] for _ in ranges]
        for p in claimed:
            i = next((i for i, (a, b) in enumerate(ranges) if a <= p.layer <= b), None)
            if i is None:
                continue
            z = span(p)
            segs[i].append(shape_solid(build, p, z=z))
            zspan[i] = [min(zspan[i][0], z[0]), max(zspan[i][1], z[1])]
        cuts: list[list] = [[] for _ in ranges]
        purchased: list = []

        # crankpin joints: a post on the segment below, screwed into a nut above
        for i, g in enumerate(gaps):
            xy = tuple(build.xy(g.pin))
            ang = build.angle("O", g.pin)
            zl0, zl1 = build.z(g.lo - 1)[0], build.z(g.lo)[0] - play
            zu0 = build.z(g.hi + 1)[0]
            zu1 = build.z(g.hi + 1)[1] - (play if g.hi + 2 in rider_layers else 0.0)
            joint = self.post_joint(zl0, zl1, zu0, zu1)
            if joint is None:
                raise ConstructionError(f"no screw fits the crankpin joint at {g.pin}")
            segs[i].append(disc(xy, d.post, zl1 - 0.5, zu0))
            bore = disc(xy, self.post_bore / 2, zl0 - 1, zu1 + 1)
            cuts[i] += [bore, disc(xy, (joint.screw.head_d + self.screw_fit) / 2,
                                   zspan[i][0] - 1, zl0 + joint.head_depth)]
            nut_z = zu1 - joint.nut_depth
            cuts[i + 1] += [bore, _hex(xy, NUT_AF + self.nut_fit, nut_z, zspan[i + 1][1] + 1, ang)]
            purchased.append(hardware(
                f"crank_screw_{g.pin}", screw_body(xy, joint.screw, zl0 + joint.head_depth,
                                                    joint.length),
                host, fab="purchased", bom_key=joint.screw.key(joint.length), color=STEEL))
            nut = _hex(xy, NUT_AF, nut_z, nut_z + NUT_H, ang) - disc(xy, NUT_BORE / 2, nut_z - 1,
                                                                    nut_z + NUT_H + 1)
            purchased.append(hardware(f"crank_nut_{g.pin}", nut, host, fab="purchased",
                                      bom_key=NUT_KEY, color=STEEL))

        # the hub: horn screws from below, the centre pocket, the step inside the plate
        top_seg = len(ranges) - 1
        hub_bottom = min((span(p)[0] for p in claimed if p.label == "crank hub"), default=face)
        joint = self.horn_joint(drive, spec, face - hub_bottom)
        if joint is None:
            raise ConstructionError(f"no screw fits between the crank hub and the {spec.key} horn")
        sk = joint.screw
        o = build.xy("O")
        theta = build.angle("O", topo.axes_of("crankpin")[0].name) + drive.pattern_angle
        bearing = face - joint.floor
        for k in range(drive.screw_count):
            a = theta + 2 * math.pi * k / drive.screw_count
            xy = tuple(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]))
            cuts[top_seg] += [disc(xy, drive.screw_clearance_d / 2, bearing - 1, face + 1),
                              disc(xy, drive.screw_head_d / 2, zspan[top_seg][0] - 1, bearing)]
            purchased.append(hardware(
                f"crank_horn_screw{k}", screw_body(xy, sk, bearing, joint.length), host,
                fab="purchased", bom_key=sk.key(joint.length), color=STEEL))
        if drive.center_head_d > 0:
            cuts[top_seg].append(disc(tuple(o), (drive.center_head_d + params.print_fit) / 2,
                                      face - drive.center_head_h - 0.2, face + 1))
        step = plate_bottom - params.margin
        if face > step + EPS:
            ring = disc(tuple(o), 2 * d.hub + 10, step, face + 1) - disc(
                tuple(o), drive.horn_radius, step - 1, face + 2)
            cuts[top_seg].append(ring)

        for i, parts in enumerate(segs):
            if not parts:
                continue
            solid = union(parts)
            if cuts[i]:
                solid = solid - union(cuts[i])
            solids = solid.solids()
            solid = solids[0] if len(solids) == 1 else solid
            out.bodies.append(hardware(f"crank_seg{i}", solid, host, fab="printed",
                                       color=SEGMENT_COLOR))
        out.bodies += purchased

        for pin in topo.axes_of("crankpin"):
            for b in pin.members:
                out.cut(b, Cut(tuple(build.xy(pin.name)), params.hole(2 * d.post)))
        if route.bearing:
            out.cut(FRAME_OUTER, Cut(tuple(build.xy("O")), params.hole(2 * d.stub)))
        return out


__all__ = [
    "BHCS", "CrankDims", "CrankGroup", "CrankRoute", "Gap", "HornJoint", "POST_SCREWS",
    "PostJoint", "PrintedCrank", "Run", "SELF_TAP", "SHCS", "ScrewKind", "add_crank_point",
    "default_route",
    "hub_layers", "route_of", "screw_body", "screw_from_key",
]
