"""``ball_pair`` (pins) and ``ball_pair_pillar`` (pillars): the ball pivots with each link's
bearing paired, in a hub beside the link (or in the link where it is thick enough for two),
on ground shafts, and a disc spring that clamps the column through the bearings' inner
rings (2026-10-12, the arm's precision review).

A link on one 3 mm deep-groove bearing (``ball``, ``ball_pillar``) tips out of the plane as
far as the bearing's bore rocks on the shaft plus the balls' own free misalignment, bounded
by the faces either side of it once the column's end float is used up: about 1.4 deg, which
at an arm's 150 mm forearm is millimetres at the tool (:func:`precision.out_of_plane`). The
remedy every precise arm uses is a pair of bearings spaced apart and preloaded against each
other (the WLKATA MT4's forearm: two flanged F686ZZ in a 15 mm aluminium bore, spacers on
the inner rings; SCARA joints: angular-contact pairs or crossed rollers).

**The pair.** Each rider link holds its bearing as on ``ball`` (an MR84ZZ / MR106ZZ bonded
by its outer ring, flush with the link's faces). Its second bearing is either

* in the link itself, where the link is at least two bearings and :data:`PAIR_GAP` thick
  (one flush with each face); or
* in a **hub** beside it: a ring laser-cut from the link's own sheet (3 mm acrylic), its
  bore the open bearing's seat (an MR84 / MR106, 2 / 2.5 mm wide, flush with the hub's far
  face: :data:`PAIR_GAP` and more between the two bearings), bonded to the link's face. The
  hub takes the place of the column's spacer ring in the layer beside the link, so every
  spacer layer claims the hub's radius (the seat and ``min_wall``: ``AxleDims.spacer``),
  and the planner keeps a free spacer layer beside every rider (:func:`hub_layers`, the
  axle's ``column`` rule). As built, a ring that doesn't touch the link's face (a
  clearance gap between) or isn't a full layer takes no hub.

A link with neither keeps one bearing (its inner ring clamped, its balls unloaded). As the
WLKATA MT4 does at its forearm, thickness at the joint, not a thicker link: a thick link
thickens its whole layer, and with it every column and the drives' shafts across it (the
review of 2026-10-12).

**The preload.** The column's rings and gap rings bear on the inner rings only (a land
under the inner ring's shoulder, :data:`INNER_LAND` across: stepped rings, the shields
clear). A disc spring under the pin's screw head (DIN 2093 series C, in place of the PTFE
washer; on a pillar, in the column's end shims against the inner frame plate) presses the
stack: spring, inner ring, balls, outer ring, link, outer ring, balls, inner ring, the next
ring, ... the bonded host (the pin) or the outer plate (the pillar). Each pair is so loaded
back to back (an O arrangement: its moment stiffness from the spread of its contact
lines), its radial and axial clearance taken up. A single bearing is not: the rings either
side bear on its inner ring only, so the spring's load passes inner ring to inner ring and
its balls see none (its clamped inner ring no longer rocks on the shaft; its balls' play
stays). The shims under the spring set its
compression (:data:`Preload.n`, about ``0.005-0.01 kN/mm`` of bore: SKF's spring
preload for deep-groove pairs).

**The shaft.** A pin is a ground 4 mm g6 shaft (:class:`GroundShaft`, cut to length in
0.5 mm steps, an M2 button head (ISO 7380, 1.3 mm) and a 2.2 x 5 washer at each end (the
washer's 5 mm on the disc spring's 4.2 mm bore), the host link bonded to
it as to a Chicago barrel): against the bearing's bore (+0 / -8 um) a 12 um gap at worst,
where the Chicago barrel, tube and not ground, is taken at 50 um. Every precise arm runs its
bearings on solid ground shafts (the MT4: 6 mm shafts with M4-tapped ends through F686ZZ
pairs; the review of 2026-10-12, round 2). A pillar is the one-piece 6 mm steel shaft
(:meth:`standoff.StandoffAxle.one_piece`), ordered ground (g6, tapped M3: the same 12 um).
The disc spring sits under the top washer (its slot: the spring, a 0.1 mm shim and its
travel, :data:`hardware.fastener_catalog.DISC_SPRINGS`).

**The pin's ends.** The host, the lowest link, is bonded to the pin: the spacer under it turns
with it, nothing to clear. The top link always rides, its top bearing flush with its face:
the top spacer (the 8 mm DIN 433 washer and any 4 x 8 shims, :func:`stock_spacers`) would
cover its outer ring too, so the pair would not be preloaded through its inner rings and the
still washer would rub the turning ring (the review of 2026-10-13). A 4 x 6 x 0.5 land
(``shim_4x6x0p5``, :attr:`GroundShaft.land_key`) sits on that inner ring under the washer
(steel, 0.5 mm: it carries the disc spring's 8 mm rim over the land, clear of the outer ring
by the land's thickness); the pin is 0.5 mm longer for it (the column rule and the end
heights count it: ``ChicagoShaft.end_t``; with the button heads the end still sinks into a
3 mm layer). Between two neighbours, the column's tubes (:func:`stock_tubes`) bear on the
inner rings; two neighbours in consecutive layers (the host and a rider, two riders, a hub
and the next link, a pillar's link and its frame plate) would meet face to face, an outer
ring on the other's face, so the planner keeps a 0.5 mm clearance gap between them
(:func:`land_gaps`, the column's ``lands`` rule) and a land washer fills it.

**Error terms.** The column has no end float. A pair tilts by its elastic compliance and
the preload's hysteresis only (:data:`PAIR_TILT_DEG`, an estimate: a Hertz estimate of the
review's gives 0.016 deg for two MR84 3.5 mm apart at 30 N under 0.05 N·m); a single
bearing by its balls' free misalignment (:func:`ball.internal_tilt`; its clamped inner ring
no longer rocks on the shaft). The radial play is the bore's gap on the shaft
(the Chicago barrel's, the standoff's: assumed, unmeasured), plus the balls' clearance for a
single bearing: the construction's terms are a single bearing's, the wobble note has each
link's (``bearings``, ``radial_mm``: what :func:`precision.joint_clearances` reads). The
clamp's friction (about 0.15 x the preload per face) holds the inner rings against a pin
load under it, which the budget doesn't count. No spacer may touch both of a pair's inner
rings: the spring's load would bypass the balls.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field, replace
from typing import Any, ClassVar, cast

from spiderpig.construction.axle import AxleDims, AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Context, Realized, hardware
from spiderpig.construction.joints import ErrorTerms
from spiderpig.construction.pivots.ball import (
    BEARING_COLOR,
    GLUE_PER_BEARING,
    BallAxle,
    BallPillar,
    BearingSeat,
    bearing_solid,
    internal_clearance,
    internal_tilt,
)
from spiderpig.construction.pivots.chicago import ChicagoShaft, Fit
from spiderpig.construction.pivots.common import (
    EPS,
    Column,
    Stock,
    host_of,
    link_z,
    stem_of,
    xy_of,
)
from spiderpig.construction.pivots.standoff import StandoffAxle
from spiderpig.construction.wobble import Section
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.hardware.fastener_catalog import GROUND_LENGTHS, ground_pin
from spiderpig.hardware.fasteners import SCREWS
from spiderpig.shapes import Cut, ring
from spiderpig.stack import Unbuildable

PAIR_GAP = 0.2          # the least gap between a pair's outer rings (mm)
PAIR_TILT_DEG = 0.02    # a preloaded pair's tilt under a reversing load: its contacts'
#                         elastic compliance at the pins' couples and the preload's
#                         hysteresis (an estimate, unmeasured)
HUB_GLUE = 0.003        # of the epoxy kit, per hub bonded to its link
PIN_LAND = "shim_4x6x0p5"      # an MR84's inner-ring land washer
PILLAR_LAND = "shim_6x8x0p5"   # an MR106's
INNER_LAND = 5.2        # the rings' land on an MR84ZZ's inner ring (mm across; the
#                         shields' bore clears it: unverified at the vendor's drawing)


@dataclass(frozen=True)
class Preload:
    """The column's spring: ``spring_key`` (a disc spring), ``n`` the preload it is
    compressed to (N)."""

    spring_key: str
    n: float


def _width(key: str) -> float:
    from spiderpig.construction.pivots.ball import bearing_dims

    return float(bearing_dims(key)["w"])


@dataclass(frozen=True)
class PairSeat(BearingSeat):
    """A bearing seat that pairs each link's bearing: in the link where it is thick enough,
    else in a hub beside it (``hub_bearing``, the open bearing of the same bore and OD)."""

    hub_bearing: str = ""

    def in_link(self, thickness: float) -> tuple[str, str] | None:
        """The two bearings a link this thick seats inside itself, one flush with each face
        (the shielded one and the hub's open one, else two open ones), with
        :data:`PAIR_GAP` between them; ``None``: it's too thin for two."""
        for a, b in ((self.bearing_key, self.hub_bearing),
                     (self.hub_bearing, self.hub_bearing)):
            if not a or not b:
                continue
            if thickness + EPS >= _width(a) + _width(b) + PAIR_GAP:
                return a, b
        return None

    def pair(self, thickness: float) -> bool:
        return self.in_link(thickness) is not None

    def hub_r(self, ctx: Context) -> float:
        """The hub's radius: the hub bearing's seat and ``min_wall``."""
        from spiderpig.construction.pivots.ball import bearing_dims

        od = float(bearing_dims(self.hub_bearing)["od"])
        return round((od + self.seat_fit) / 2 + ctx.params.min_wall, 3)

    def hub_width(self) -> float:
        from spiderpig.construction.pivots.ball import bearing_dims

        return float(bearing_dims(self.hub_bearing)["w"])

    def tilt_deg(self, bearings: int) -> float:
        return PAIR_TILT_DEG if bearings > 1 else round(
            math.degrees(internal_tilt(self.bearing_key)), 3)

    def play(self, bearings: int) -> float:
        """The diametral play of a link on ``bearings``: the bore on the shaft, and the
        balls' clearance where a single bearing seats."""
        own = 0.0 if bearings > 1 else internal_clearance(self.bearing_key)
        return round(self.shaft_fit + own, 4)

    def seat_bearings(self, build: Build, group: AxleGroup, out: Realized, riders,
                      stem: str) -> None:
        """One bearing per rider link, or two flush with its faces where it is thick enough
        (:meth:`pair`); the bore cut."""
        xy = xy_of(build, group)
        for m in sorted(riders):
            t = build.ctx.sheet_t("link", m)
            if self.width > t + EPS:
                raise ConstructionError(f"{group.name}: {m} is {t:g} mm thick, under the "
                                        f"{self.width:g} mm bearing it seats")
            mid = sum(link_z(build, m)) / 2         # the link's middle, on its layer's floor
            two = self.in_link(t)
            seats = ([(two[0], mid - t / 2), (two[1], mid + t / 2 - _width(two[1]))] if two
                     else [(self.bearing_key, mid - self.width / 2)])
            zs = [zb for _, zb in seats]
            for i, (key, zb) in enumerate(seats):
                tag = f"{stem}_bearing_{m}" + (f"_{i}" if len(seats) > 1 else "")
                out.bodies.append(hardware(tag, bearing_solid(key, xy, zb), m,
                                           fab="purchased", bom_key=key,
                                           color=BEARING_COLOR))
            out.cut(m, Cut(xy, self.seat_d()))
            out.extras.append(BomLine("epoxy_2part", GLUE_PER_BEARING * len(zs),
                                      f"{group.name}: bearing{'s' if len(zs) > 1 else ''} "
                                      f"into {m} (slow epoxy on the outer ring"
                                      + (f"; a {PAIR_GAP:g} mm feeler between the pair's "
                                         "outer rings as it cures)" if len(zs) > 1 else ")")))

    def hubs(self, build: Build, group: AxleGroup, out: Realized, riders,
             stem: str) -> dict[str, int]:
        """Each rider's bearings: two in a thick link; else a hub in the spacer layer beside
        it (below first, the lowest rider first: as :func:`hub_layers` planned it) where
        the column's ring there is the hub's width, touches the link's face and is a full
        layer (the ring replaced by the hub, its bearing at the far face), one hub per
        ring; else one."""
        ctx = build.ctx
        col = Column.of(build, group)
        xy = xy_of(build, group)
        hub_r, wa = self.hub_r(ctx), self.hub_width()
        taken: set[int] = set()
        count: dict[str, int] = {}
        for m in sorted(riders, key=lambda m: build.layers[m]):    # as hub_layers: up
            k = build.layers[m]
            t = ctx.sheet_t("link", m)
            count[m] = 2 if self.pair(t) else 1
            if count[m] == 2:
                continue
            for kn, face in zip((k - 1, k + 1), link_z(build, m), strict=True):
                if kn in taken or kn not in col.between or col.roles[kn][1] + EPS < hub_r:
                    continue
                name = f"{stem}_ring{kn}"
                idx = next((i for i, b in enumerate(out.bodies) if b.name == name), None)
                if idx is None:
                    continue
                part: Any = out.bodies[idx].part             # (build123d: no stubs)
                bb = part.bounding_box()
                h = bb.max.Z - bb.min.Z
                near = bb.min.Z if kn > k else bb.max.Z
                if abs(near - face) > EPS or abs(h - ctx.pitch) > EPS or h < wa + PAIR_GAP:
                    continue
                r = col.roles[kn][1]
                out.bodies[idx] = hardware(f"{stem}_hub_{m}", ring(xy, 2 * r, self.seat_d(),
                                                                   bb.min.Z, bb.max.Z),
                                           m, fab="laser")
                za = bb.max.Z - wa if kn > k else bb.min.Z
                out.bodies.append(hardware(f"{stem}_bearing_{m}_hub", bearing_solid(
                    self.hub_bearing, xy, za), m, fab="purchased", bom_key=self.hub_bearing,
                    color=BEARING_COLOR))
                out.extras += [BomLine("epoxy_2part", HUB_GLUE + GLUE_PER_BEARING,
                                       f"{group.name}: {m}'s hub onto its face, its bearing "
                                       "into it (slow epoxy)")]
                taken.add(kn)
                count[m] = 2
                break
        return count


def preloaded(note: dict, seat: PairSeat, preload: Preload, count: dict[str, int]) -> dict:
    """A column's wobble note with the preload's: no end float, each rider's tilt and
    radial play its bearings' (``count``: :meth:`PairSeat.hubs`), the host (no tilt)
    kept."""
    links = []
    for e in note["links"]:
        if e["link"] not in count:
            links.append(e)                       # the bonded host
            continue
        n = count[e["link"]]
        tilt = seat.tilt_deg(n)
        links.append({**e, "play_mm": 0.0, "free_deg": tilt, "supported_deg": tilt,
                      "tilt_deg": tilt, "bearings": n, "radial_mm": seat.play(n)})
    return {**note, "links": links, "play_mm": 0.0,
            "play_basis": f"preloaded by a disc spring ({preload.spring_key}, about "
                          f"{preload.n:g} N, set by the shims under it)",
            "worst_deg": max((e["tilt_deg"] for e in links), default=0.0),
            "worst_free_deg": max((e["free_deg"] for e in links), default=0.0)}


def _terms(seat: PairSeat, preload: Preload, base: ErrorTerms, key: str) -> ErrorTerms:
    """A single preloaded bearing's terms (the worst link's: a paired one is in the wobble
    note)."""
    return replace(base, radial_play_mm=seat.play(1), axial_play_mm=0.0,
                   tilt_deg=seat.tilt_deg(1),
                   source=f"{key}: preloaded {seat.bearing_key} on the shaft, paired where "
                          "there is room",
                   notes=(f"{seat.shaft_fit:g} mm bore-on-shaft gap (assumed, unmeasured); "
                          f"about {preload.n:g} N of spring preload; a single bearing "
                          f"+{internal_clearance(seat.bearing_key) * 1000:g} um and its balls' "
                          f"tilt, a pair {PAIR_TILT_DEG:g} deg (estimate)",))


def hub_layers(riders: list[int], occupied: set[int], lo: int, hi: int,
               room: dict[int, float], spacer: float) -> dict[int, int]:
    """Each rider's layer -> the layer beside it its hub takes: one not ``occupied`` (the
    axle's links, a drive's hub), strictly between ``lo`` and ``hi`` (a spacer layer of the
    column), with ``spacer`` of ``room`` (no link passing closer), one hub a layer; the
    lower side first (on a line, that greedy matching is the largest). :class:`Unbuildable`
    names the first rider with none: the planner's rule (:meth:`BallPairAxle.column`)."""
    taken: dict[int, int] = {}
    used: set[int] = set()
    for k in sorted(riders):
        for kn in (k - 1, k + 1):
            if (lo < kn < hi and kn not in occupied and kn not in used
                    and room.get(kn, math.inf) + EPS >= spacer):
                taken[k] = kn
                used.add(kn)
                break
        else:
            raise Unbuildable(f"its link in layer {k} has no free layer beside it for its "
                              "bearing pair's hub")
    return taken


def land_gaps(elements: list[tuple[int, int]], height: float) -> dict[int, float]:
    """The clearance gaps a column keeps for its inner-ring lands: over every element's
    layer (``(layer, owner)``: a link, a hub with its link's layer as owner, a frame plate)
    whose neighbour in the next layer up is another owner's, ``height`` (a land washer):
    face to face, a bearing's outer ring would bear on its neighbour's face and the
    preload bypass its balls (pin T of the arm, 2026-10-13)."""
    els = sorted(elements)
    return {a: height for (a, oa), (b, ob) in itertools.pairwise(els)
            if b == a + 1 and oa != ob}


def spring_slot(out: Realized, stem: str, slot: float, spring_key: str) -> None:
    """The top spacer (``<stem>_spacer_hi``: the washer's slot and the take-up shims, one
    printed piece from :meth:`ChicagoShaft.realize`) parted: the take-up shims (bought as a
    DIN 988 stack, :func:`stock_spacers`) on the land, and the disc spring in its slot over
    them, drawn (bought once: the preload's BOM line). Before the review of 2026-10-13 the
    whole spacer, the spring's slot included, was bought as rigid shims: the screw then
    flattened the spring, the preload set by its torque, not the spring's."""
    from dataclasses import replace as _replace

    for i, b in enumerate(out.bodies):
        if b.name != f"{stem}_spacer_hi" or b.part is None:
            continue
        bb = cast(Any, b.part).bounding_box()
        xy = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2)
        z0, z1 = bb.min.Z, bb.max.Z
        shims = z1 - z0 - slot
        d = get(spring_key).dims
        spring = hardware(f"{stem}_spring", ring(xy, float(d["od"]), float(d["id"]),
                                                 z1 - slot, z1 - slot + float(d["h0"])),
                          b.rigid_with or "", fab="purchased", color="#aa3333")
        if shims > EPS:
            out.bodies[i] = _replace(b, part=ring(xy, float(d["od"]), 4.2, z0, z0 + shims))
            out.bodies.append(spring)
        else:
            out.bodies[i] = spring
        return


def _hubless(seat: PairSeat, ctx: Context) -> frozenset[str]:
    """The design's links thick enough to seat their pair inside (no hub beside them)."""
    return frozenset(m for m in ctx.topo.links if seat.pair(ctx.sheet_t("link", m)))


def _inside_layers(hubless: frozenset[str], layout) -> set[int]:
    """The layers of ``hubless`` links placed so far."""
    placed = getattr(layout, "layers", {}) or {}
    return {placed[m] for m in hubless if m in placed}


def _dims(seat: PairSeat, d: AxleDims, ctx: Context, tube_key: str = "") -> AxleDims:
    """Every spacer layer the hub's width (it may take a ring's place); in a drive hub's
    bore, the column's tube (:func:`stock_tubes`)."""
    from spiderpig.hardware.catalog import get as _get

    nested = float(_get(tube_key).dims["od"]) / 2 if tube_key else 0.0
    return replace(d, spacer=max(d.spacer, seat.hub_r(ctx)), nested=max(d.nested, nested))


def _assembly(seat: PairSeat, preload: Preload, group: AxleGroup, view) -> list:
    from spiderpig.construction.assembly import STACK, Op, whole

    stem = stem_of(group)
    by_layer: dict[int, list[str]] = {}
    for n in view.z:
        if n.startswith(f"{stem}_bearing_"):
            by_layer.setdefault(view.slot(view.z[n][0]), []).append(n)
    return [Op(STACK, (k, 0, 0), whole(*sorted(names)), f"Bearings for layer {k}",
               "Bond the bearings into each of these links' bores by their outer rings (slow "
               "epoxy, none on the shields or the bore): in a thick link one flush with each "
               f"face, a {PAIR_GAP:g} mm feeler between their outer rings and both centred on "
               "a pin while it cures; a hub's open bearing flush with its far face, the hub "
               "bonded to its link's face on the same pin. The rings beside a link bear on "
               "its inner rings "
               f"only (their {INNER_LAND:g} mm land; on a pin's top link, a 4 x 6 land washer "
               "under the top washer). Once the column is assembled, shim the "
               f"disc spring ({preload.spring_key}) to about {preload.n:g} N: flat, then back "
               "a quarter of its travel.", "layer", sub=True)
            for k, names in sorted(by_layer.items())]


END_SCREW_L = 4         # the ground pin's M2 button-head end screws (mm)
GROUND_FIT = 0.012      # a g6 ground shaft in a bearing's bore (+0 / -8 um): the worst gap
GROUND_YIELD = 350.0    # MPa: the ground shaft's steel, taken low (S45C-class, unhardened)


@dataclass(frozen=True)
class GroundShaft(ChicagoShaft):
    """A ground 4 mm g6 pin cut to length, M2 screws and washers at its ends, the disc spring
    under the top one, a 4 x 6 land on the top rider's inner ring under the top spacer (module
    docstring): a Chicago screw's part in the column."""

    washer_key: str = "disc_spring_din2093_c8"
    land_key: str | None = PIN_LAND   # on the top rider's inner ring (module docstring)
    lengths: tuple[float, ...] = GROUND_LENGTHS

    def item(self, length: float = GROUND_LENGTHS[0]) -> dict:
        return get(self.key(length)).dims

    def key(self, length: float) -> str:
        return ground_pin(length)

    def section(self) -> Section:
        return Section.rod(self.d, GROUND_YIELD, name="ground shaft 4 mm g6")

    def check(self, ctx, pillar: bool, extra: float = 0.0) -> None:
        """Its end (the button head and washer, the disc spring's slot and the land) fits a
        layer, and its length steps the end slots' shims (its own steps: 0.5 mm, cut to
        order)."""
        if pillar:
            raise ConstructionError("a ground pin is a link pin only")
        it = self.item()
        top = float(it["screw_head_h"]) + self.end_t + self.min_play + extra
        if top > ctx.pitch + EPS:
            raise ConstructionError(f"a ground pin's {it['screw_head_h']:g} mm end, its disc "
                                    f"spring and play don't fit a {ctx.pitch:g} mm layer")
        step = max(b - a for a, b in zip(self.lengths, self.lengths[1:], strict=False))
        room = (ctx.pitch - top) + (ctx.pitch - float(it["head_h"]))
        if step > room + min(self.shim_steps) + EPS:
            raise ConstructionError(f"a {step:g} mm step between pin lengths needs more shims "
                                    f"than two {ctx.pitch:g} mm end layers hold")

    def realize(self, build: Build, group: AxleGroup, col, out: Realized, *,
                faces: dict[str, float] | None = None) -> Fit:
        f = super().realize(build, group, col, out, faces=faces)
        out.extras += [BomLine(SCREWS["button", "2"].key(END_SCREW_L), 2,
                               f"{group.name}: the pin's end screws"),
                       BomLine("m2_washer_din125", 2, f"{group.name}: under the end screws")]
        return f


@dataclass(frozen=True)
class BallPairAxle(BallAxle):
    """Preloaded MR84ZZ pairs (one in a thin link) on a Chicago-screw pin (pins only)."""

    key: str = "ball_pair"
    spacers: ClassVar[Stock | None] = Stock("collar_5p5x4p1", "shim_din988_4x8", land=PIN_LAND)
    label: str = ("MR84ZZ ball bearings, each paired in a hub beside its link where there is "
                  "room (or in a thick link), on a ground 4 mm g6 pin with M2 ends; "
                  "inner-ring rings; a disc spring under the top end; host link bonded "
                  "(pins only)")
    seat: BearingSeat = field(default_factory=lambda: PairSeat("bearing_mr84zz", GROUND_FIT,
                                                               hub_bearing="bearing_mr84"))
    shaft: ChicagoShaft = field(default_factory=GroundShaft)
    hubless: frozenset[str] = frozenset()     # the links thick enough for a pair inside

    def resolve(self, ctx: Context) -> BallPairAxle:
        """As the Chicago pin's, and the links that seat their pair inside (no hub)."""
        own = cast(BallPairAxle, super().resolve(ctx))
        assert isinstance(self.seat, PairSeat)
        return replace(own, hubless=_hubless(self.seat, ctx))
    preload: Preload = field(default_factory=lambda: Preload("disc_spring_din2093_c8", 35.0))
    cad_keys: ClassVar[tuple[str, ...]] = ("bearing_mr84zz", "bearing_mr84")

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        assert isinstance(self.seat, PairSeat)
        return _dims(self.seat, super().dims(ctx, pillar), ctx)

    def column(self, pillar: bool, links, top: int, anchored, pitch: float, layout=None,
               air=None, *, members=None, room=None, spacer=0.0) -> None:
        """The ground pin's rule (its length), and a hub's layer beside every rider (all but
        the bonded host, the lowest): :func:`hub_layers`."""
        super().column(pillar, links, top, anchored, pitch, layout, air)
        if pillar or not members:
            return
        ms = sorted(members)
        inside = _inside_layers(self.hubless, layout)
        hub_layers([k for k in ms[1:] if k not in inside], set(links), ms[0], ms[-1],
                   room or {}, spacer)

    def lands(self, pillar: bool, links, top: int, anchored, *, members=None, layout=None,
              room=None, spacer=0.0) -> dict[int, float]:
        """The planner's clearance gaps for the inner-ring lands (:func:`land_gaps`): between
        every two neighbouring members, the host included, or a hub and the next member
        (the hubs :meth:`column` placed)."""
        if pillar or not members:
            return {}
        ms = sorted(members)
        inside = _inside_layers(self.hubless, layout)
        hubs = hub_layers([k for k in ms[1:] if k not in inside], set(links), ms[0], ms[-1],
                          room or {}, spacer)
        return land_gaps([*((k, k) for k in ms), *((kn, k) for k, kn in hubs.items())],
                         float(get(PIN_LAND).dims["t"]))

    def error_terms(self, ctx: Context) -> ErrorTerms:
        assert isinstance(self.seat, PairSeat)
        return _terms(self.seat, self.preload, super().error_terms(ctx), self.key)

    def assembly(self, group: AxleGroup, view) -> list:
        assert isinstance(self.seat, PairSeat)
        rest = [op for op in BallAxle.assembly(self, group, view)
                if not op.title.startswith("Bearings for layer")]
        return [*_assembly(self.seat, self.preload, group, view), *rest]

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        assert isinstance(self.seat, PairSeat)
        out = super().realize(group, build)
        host = host_of(build, group)
        riders = [m for m in group.axis.members if m != host]
        count = self.seat.hubs(build, group, out, riders, stem_of(group))
        out.notes["wobble"][group.name] = preloaded(out.notes["wobble"][group.name],
                                                   self.seat, self.preload, count)
        out.extras.append(BomLine(self.preload.spring_key, 1,
                                  f"{group.name}: the preload, under the screw's head"))
        spring_slot(out, stem_of(group), self.shaft.washer_t, self.preload.spring_key)
        return out          # (its tubes, lands and shims: ``spacers``, the group applies)


@dataclass(frozen=True)
class BallPairPillar(BallPillar):
    """Preloaded MR106ZZ pairs (one in a thin link) on a 6 mm standoff pillar (pillars
    only)."""

    key: str = "ball_pair_pillar"
    spacers: ClassVar[Stock | None] = Stock("collar_7x6p1", "shim_din988_6x12",
                                          land=PILLAR_LAND, plates=True)
    label: str = ("MR106ZZ ball bearings, each paired in a hub beside its link where there "
                  "is room (or in a thick link), on a one-piece 6 mm steel shaft ordered "
                  "ground; inner-ring rings; a disc spring in the column's end shims (pillars "
                  "only; link_radius 6.6 mm)")
    seat: BearingSeat = field(default_factory=lambda: PairSeat("bearing_mr106zz", GROUND_FIT,
                                                               hub_bearing="bearing_mr106"))
    hubless: frozenset[str] = frozenset()     # the links thick enough for a pair inside
    preload: Preload = field(default_factory=lambda: Preload("disc_spring_din2093_c12p5",
                                                             50.0))
    cad_keys: ClassVar[tuple[str, ...]] = ("bearing_mr106zz", "bearing_mr106")

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        assert isinstance(self.seat, PairSeat)
        return _dims(self.seat, super().dims(ctx, pillar), ctx, "collar_7x6p1")

    def column_axle(self, links, top: int, pitch: float, lo: int = 0, layout=None,
                    air=None):
        """The one-piece steel shaft always (ordered ground), never a goBILDA standoff:
        aluminium, not ground; ``None`` where no ground shaft's length fills the column
        (a goBILDA length that would is no use: the planner's rule and the build agree)."""
        ground = replace(self.one_piece(), ground=True)
        return StandoffAxle.column_axle(ground, links, top, pitch, lo, layout, air)

    def resolve(self, ctx: Context) -> BallPairPillar:
        assert isinstance(self.seat, PairSeat)
        return replace(self, hubless=_hubless(self.seat, ctx))

    def column(self, pillar: bool, links, top: int, anchored, pitch: float, layout=None,
               air=None, *, members=None, room=None, spacer=0.0) -> None:
        """The standoff's rule, and a hub's layer beside every link on it that seats one
        bearing, between the frame plates: :func:`hub_layers`."""
        super().column(pillar, links, top, anchored, pitch, layout, air)
        if members:
            inside = _inside_layers(self.hubless, layout)
            hub_layers([k for k in sorted(members) if k not in inside], set(links), 0, top,
                       room or {}, spacer)

    def lands(self, pillar: bool, links, top: int, anchored, *, members=None, layout=None,
              room=None, spacer=0.0) -> dict[int, float]:
        """As the pin's (:func:`land_gaps`): between neighbouring links and hubs, and a link
        and the frame plate it's next to."""
        if not members:
            return {}
        ms = sorted(members)
        inside = _inside_layers(self.hubless, layout)
        hubs = hub_layers([k for k in ms if k not in inside], set(links), 0, top,
                          room or {}, spacer)
        plates = [(k, -1 - k) for k, on in ((0, anchored[0]), (top, anchored[1])) if on]
        return land_gaps([*((k, k) for k in ms), *((kn, k) for k, kn in hubs.items()),
                          *plates], float(get(PILLAR_LAND).dims["t"]))

    def error_terms(self, ctx: Context) -> ErrorTerms:
        assert isinstance(self.seat, PairSeat)
        return _terms(self.seat, self.preload, super().error_terms(ctx), self.key)

    def assembly(self, group: AxleGroup, view) -> list:
        assert isinstance(self.seat, PairSeat)
        rest = [op for op in BallPillar.assembly(self, group, view)
                if not op.title.startswith("Bearings for layer")]
        return [*_assembly(self.seat, self.preload, group, view), *rest]

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        assert isinstance(self.seat, PairSeat)
        out = super().realize(group, build)
        count = self.seat.hubs(build, group, out, list(group.axis.members), stem_of(group))
        out.notes["wobble"][group.name] = preloaded(out.notes["wobble"][group.name],
                                                   self.seat, self.preload, count)
        out.extras.append(BomLine(self.preload.spring_key, 1,
                                  f"{group.name}: the preload, in the column's end shims"))
        return out          # (its tubes, lands and shims: ``spacers``, the group applies)


__all__ = ["BallPairAxle", "BallPairPillar", "PairSeat", "Preload"]
