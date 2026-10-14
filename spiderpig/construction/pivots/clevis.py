"""``clevis8``: an 8 mm clevis pin, its head inside the stack and a hairpin cotter (an R-clip)
through its cross hole from the bay: a decanting cradle's quick-release link pin (B's X and Y,
``config.PRODUCT_PIN_JOINTS``, since 2026-10-13). Pull the two clips on the passive side and
its panel slides off its cradle link, the pins with it (:mod:`spiderpig.cradle_pack`).

**The part.** McMaster's 18-8 clevis pin, 8 mm, head 14 x 3, a 2 mm cotter hole at its usable
length (the head's underside to the hole's near edge), the end 3 mm past the hole
(:data:`hardware.cradle_catalog.CLEVIS8`: 15 mm usable, 20 long, USD 5.47), and a hairpin
cotter (2 mm wire, 50 mm long, for a 6-11 mm pin). The links turn on the pin in the M6
Chicago barrel's 8.2 mm holes, as on a pivot pin (:mod:`construction.pivots.pivot_pin`).

**In the stack.** The pin's head is on its lowest link's outer face, over a printed take-up
spacer: the stock usable length less the stack and the least play (to the 0.1 mm), so the
pin keeps :attr:`ClevisPinShaft.min_play` to under a step more. The head and the spacer
take the two layers outside the lowest link (``ends``: full layers, no clearance gap); on
B the arms b2 and b3 never come within 70 mm of X or Y there. The clip lies in the
clearance gap over the highest link (the cradle link), its whole reach (:data:`CLIP_REACH`:
it turns with the pin) claimed there; the pin's end goes on into the next layer, the inner
plate's on B, in the plate's open as a tie rod does (``tips``: a seat, and
:class:`construction.cradle.CradlePlates` keeps the plate off its path).

**Strength** (:mod:`spiderpig.strength`): a solid 8 mm rod at
:data:`hardware.cradle_catalog.CLEVIS8_YIELD` (205 MPa, annealed 304's minimum): 1.39 x the
M6 Chicago barrel it replaces (an 8 x 6 tube at 215). The cross hole lies past the cradle
link, where nothing bends the pin.

**Assembly** (:meth:`ClevisPinAxle.assembly`): on the bench, the printed spacer and the pin up
through the lowest link from its outer face (its head under the spacer), the link onto the
stack with the pin standing in it; once the cradle link is over it, the clip through the
pin's hole from the bay side by hand, its bent leg over the pin.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from build123d import Cylinder, Location

from spiderpig.construction.axle import AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Realized, hardware
from spiderpig.construction.pivots.chicago import ChicagoAxle, ChicagoShaft, Fit
from spiderpig.construction.pivots.common import (
    EPS,
    HEAD_CLEARANCE,
    PRINT_MIN,
    PRINT_TOL,
    SLEEVE_COLOR,
    STEEL,
    column_air,
    host_of,
    stem_of,
    xy_of,
)
from spiderpig.construction.wobble import Section
from spiderpig.shapes import difference, disc, ring, union
from spiderpig.stack import Disc, Placed, Unbuildable

CLIP = "hairpin_cotter_2x50"
CLIP_REACH = 44.0       # mm: the clip's reach round the pin's axis (its eye's far side, 50 mm
#                         of clip with the pin near its straight leg's end): it turns with the pin
CLIP_COLOR = "#b8b8b8"
SPACER_STEP = 0.1       # the printed take-up spacer's steps


def _lengths() -> tuple[float, ...]:
    from spiderpig.hardware.cradle_catalog import CLEVIS8_LENGTHS

    return CLEVIS8_LENGTHS


@dataclass(frozen=True)
class ClevisPinShaft(ChicagoShaft):
    """The clevis pin as a Chicago screw's part in the column (``lengths``: the stock usable
    lengths): its head outside the lowest link over a printed take-up spacer, its cross hole
    and clip past the highest; nothing threaded, no thrust washer."""

    family: str = "clevis8"
    lengths: tuple[float, ...] = field(default_factory=_lengths)
    bonded: bool = False
    washer_key: str = "ptfe_washer_8x14x0p5"   # (its OD only: the head's claims)
    shim_key: str = "shim_din988_8x14"
    min_play: float = 0.25
    lock_key: str = ""

    def key_of(self, length: float) -> str:
        from spiderpig.hardware.cradle_catalog import clevis_pin8

        return clevis_pin8(length)

    @property
    def washer_t(self) -> float:
        return 0.0

    def clip_h(self) -> float:
        """What the clip needs past the highest link: the play at most (the spacer's step
        and print tolerance over the least), the hole it lies in, the clearance."""
        it = self.item()
        return (self.min_play + SPACER_STEP + PRINT_TOL + float(it["hole_d"])
                + HEAD_CLEARANCE)

    def tip(self) -> float:
        """The pin past its hole's far edge."""
        it = self.item()
        return float(it["overall"]) - float(it["length"]) - float(it["hole_d"])

    def check(self, ctx, pillar: bool, extra: float = 0.0) -> None:
        from spiderpig.stack import GAP_MAX

        if pillar:
            raise ConstructionError("a clevis pin is a link pin only: its head and clip would "
                                    "stand outside the frame plates")
        if self.clip_h() > GAP_MAX + EPS:
            raise ConstructionError(f"a clevis pin's clip and its play need {self.clip_h():.2f} "
                                    f"mm, more than a {GAP_MAX:g} mm clearance gap")

    def base_heights(self) -> tuple[float, float]:
        """The head takes whole layers (0); the clip a clearance gap."""
        return 0.0, self.clip_h()

    def end_heights(self, L, k0: int, k1: int, extra: tuple[float, float] = (0.0, 0.0),
                    air: float = 0.0) -> tuple[float, float]:
        return self.base_heights()

    def take_up(self, stack: float) -> Fit:
        """The stock pin for a ``stack`` mm stack: the shortest usable length over the stack
        and the least play, its printed spacer (``shims_lo``: the rest less the least play,
        to the step; none under :data:`common.PRINT_MIN`) and the play left."""
        need = stack + self.min_play
        stock = [u for u in self.lengths if self.max_length is None
                 or u <= self.max_length + EPS]
        usable = next((u for u in stock if u >= need - EPS), None)
        if usable is None:
            raise ConstructionError(f"no stock clevis pin spans a {stack:.2f} mm stack "
                                    f"(the longest it may take is {max(stock, default=0):g} "
                                    "mm usable: a long pin is a long span)")
        spacer = math.floor((usable - need) / SPACER_STEP + 1e-6) * SPACER_STEP
        if spacer < PRINT_MIN - EPS:
            spacer = 0.0
        return Fit(float(usable), round(spacer, 3), 0.0, round(usable - stack - spacer, 3))

    def outer(self, stack: float) -> float:
        """How far the head stands out past the lowest link's outer face at most (its
        spacer, the head and the play)."""
        f = self.take_up(stack)
        return f.length - stack + float(self.item()["head_h"])

    def column(self, pillar: bool, links, top: int, anchored: tuple[bool, bool],
               pitch: float, layout=None, air=None, hi=None) -> None:
        """The planner's rule: a stock pin spans the links (at the plan's z: its head and
        spacer within the two layers outside them). A search's layout is a lower bound of the
        stack: no pin long enough there is none in any plan."""
        if pillar or not links:
            return
        k0, k1 = min(links), max(links)
        stack = (k1 - k0 + 1) * pitch
        if layout is not None:
            stack = layout.z(k1)[1] - layout.z(k0)[0]
            if air is not None:
                stack -= air(k0, k1)
        try:
            out = self.outer(stack)
        except ConstructionError as e:
            raise Unbuildable(str(e)) from None
        if layout is not None and layout.final and k0 >= 2:
            room = layout.z(k0)[0] - layout.z(k0 - 2)[0]
            if out > room + EPS:
                raise Unbuildable(f"a clevis pin's head and spacer stand {out:.2f} mm out, "
                                  f"more than the two layers outside its links ({room:.2f})")

    def section(self) -> Section:
        from spiderpig.hardware.cradle_catalog import CLEVIS8_YIELD

        return Section.rod(self.d, CLEVIS8_YIELD,
                           name=f"{self.d:g} mm 18-8 clevis pin (solid, {CLEVIS8_YIELD:g} MPa)")

    def realize(self, build: Build, group: AxleGroup, col, out: Realized, *,
                faces: dict[str, float] | None = None) -> Fit:
        """The pin (one body), its printed take-up spacer and its clip: the spacer and the
        head under the lowest link, the pin drawn pushed home (its play at the clip); the
        clip's straight leg through the hole along x, its bent leg over the pin, its eye."""
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        z_lo, z_hi = build.z(col.k0)[0], build.z(col.k1)[1]
        air = column_air(build, group, col.k0, col.k1)
        f = self.take_up(z_hi - z_lo - air)
        it = self.item(f.length)
        head_h, head_d = float(it["head_h"]), float(it["head_d"])
        hole, overall = float(it["hole_d"]), float(it["overall"])
        zh = z_lo - f.shims_lo                      # the head's underside
        zc = zh + f.length + air + hole / 2         # the hole's (and the clip's) centre
        lo_room = build.z(col.k0 - 2)[0] if col.k0 >= 2 else -math.inf
        if zh - head_h - (f.play) < lo_room - EPS:
            raise ConstructionError(f"{group.name}: the clevis pin's head and spacer leave the "
                                    "two layers outside its links")
        hi_slot = col.end_z(build, "hi")
        if hi_slot is None or zc + hole / 2 > hi_slot[1] + EPS:
            raise ConstructionError(f"{group.name}: the clevis pin's clip leaves the gap over "
                                    "its highest link")
        r = (self.d - self.model_gap) / 2
        cross = Cylinder(hole / 2, self.d + 2.0, rotation=(0, 90, 0)).moved(
            Location((xy[0], xy[1], zc)))
        pin = difference(union([disc(xy, head_d / 2, zh - head_h, zh),
                                disc(xy, r, zh, zh + overall + air)]), cross)
        out.bodies.append(hardware(f"{stem}_pin", pin, host, fab="purchased",
                                   bom_key=self.key_of(f.length), color=STEEL))
        if f.shims_lo > EPS:
            out.bodies.append(hardware(f"{stem}_spacer_lo",
                                       ring(xy, head_d, self.d + 0.3, zh, z_lo), host,
                                       fab="printed", color=SLEEVE_COLOR))
        out.bodies.append(hardware(f"{stem}_clip", _clip(xy, zc, self.d), host,
                                   fab="purchased", bom_key=CLIP, color=CLIP_COLOR))
        out.notes.setdefault("clevis_pin", {})[group.name] = {
            "usable_mm": f.length, "stack_mm": round(z_hi - z_lo - air, 3),
            "spacer_mm": f.shims_lo, "play_mm": f.play, "item": self.key_of(f.length),
            "clip": CLIP, "past_link_mm": round(f.play + hole + self.tip(), 3)}
        return f


def _clip(xy, zc: float, pin_d: float, wire: float = 1.9):
    """A hairpin cotter through a pin's cross hole at ``zc`` (world, in the plane square to
    the pin): its straight leg through the pin along +x, its bent leg over the pin's side, its
    eye joining them."""
    x, y = float(xy[0]), float(xy[1])
    r = pin_d / 2
    reach = CLIP_REACH - 6.0                         # the eye's centre
    along = (0, 90, 0)
    straight = Cylinder(wire / 2, reach + r + 6.0, rotation=along).moved(
        Location((x + (reach - r - 6.0) / 2, y, zc)))
    off = r + wire / 2 + 0.2                         # the bent leg rides the pin's side
    bent = Cylinder(wire / 2, reach, rotation=along).moved(Location((x + reach / 2, y + off, zc)))
    eye = ring((x + reach, y + off / 2), off + wire + 1.0, off - wire + 1.0, zc - wire / 2,
               zc + wire / 2)
    return union([straight, bent, eye])


@dataclass(frozen=True)
class ClevisPinAxle(ChicagoAxle):
    """An 8 mm clevis pin through the stack, its head over a printed spacer outside the lowest
    link, a hairpin cotter past the highest; printed rings between (pins only)."""

    key: str = "clevis8"
    label: str = ("8 mm 18-8 clevis pin, its head inside the stack over a printed spacer, a "
                  "hairpin cotter (R-clip) through its hole from the bay: a decanting cradle's "
                  "quick-release pin (pins only)")
    shaft: ChicagoShaft = field(default_factory=ClevisPinShaft)

    def resolve(self, ctx) -> ClevisPinAxle:
        """This construction for the design ``ctx`` builds: its linkage's longest pin
        (:data:`chicago.MAX_BARREL`, here the usable length), as a Chicago screw's."""
        out = super().resolve(ctx)
        assert isinstance(out, ClevisPinAxle)
        return out

    def ends(self, d, pillar: bool, anchored: tuple[bool, bool], n: int, pitch: float,
             span: float | None = None):
        """The head and its spacer in the two layers outside the lowest link; the clip's
        whole reach in the clearance gap over the highest."""
        head = d.head + 0.3
        return (("head", head), ("head", head)), (("clip", CLIP_REACH),)

    def tips(self, d, L, k0: int, k1: int, ax: str, g: str) -> list[Placed]:
        """The pin's end past its clip, in the layer over the gap: in the inner plate's open
        when that is the frame (a seat, as a tie rod; :class:`construction.cradle.CradlePlates`
        keeps the plate off its path)."""
        k = k1 + 1
        if k > L.top:
            return []
        return [Placed(k, Disc(ax, self.shaft.d / 2 + 0.3), g, f"{g} tip", seat=k == L.top)]

    def assembly(self, group: AxleGroup, view) -> list:
        """How a pin goes on (:mod:`construction.assembly`): the printed spacer and the pin up
        through its lowest link on the bench (the link's step), the clip once its highest
        link is on."""
        from spiderpig.construction.assembly import STACK, Op, whole

        stem = stem_of(group)
        pin = f"{stem}_pin"
        if pin not in view.z:
            return []
        host = view.hosts.get(pin)
        k_lo = view.slot(view.z[host][0]) if host in view.z else view.slot(view.z[pin][0])
        clip = f"{stem}_clip"
        k_hi = view.slot(view.z[clip][0]) if clip in view.z else k_lo
        spacer = view.named(rf"{stem}_spacer_lo")
        on = ("Each clevis pin: its printed spacer over the pin, then the pin up through its "
              "lowest link from the link's outer face (the head and the spacer under the "
              "link), on the bench; the link goes on with the pin standing in it.")
        clip_on = ("Each clevis pin's hairpin cotter (R-clip) through the pin's hole from the "
                   "bay side, by hand, its bent leg over the pin: the links must turn freely "
                   "and lift their play (about 0.3 mm). To take the side off, pull the clip; "
                   "it goes back in the pin's hole.")
        ops = [Op(STACK, (k_lo, 0, 1), whole(pin, *spacer), "", on, "layer")]
        if clip in view.z:
            ops.append(Op(STACK, (k_hi, 2), whole(clip), "", clip_on, "layer"))
        return ops


CLEVIS8 = ClevisPinAxle()
"""The 8 mm clevis pin with a hairpin cotter (McMaster 90378A701 and 98350A131): a decanting
cradle's quick-release pins (``config.PRODUCT_PIN_JOINTS``: B's X and Y)."""
