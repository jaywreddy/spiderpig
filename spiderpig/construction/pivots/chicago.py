"""``chicago`` / ``chicago_bushing``: an M3 Chicago screw (binding barrel and screw) as the pin.

A Chicago screw is a **barrel** (a 4 mm tube with a flat 8.5 mm head, threaded M3
inside) and a **screw** with the same head that threads into it (the parts bought,
Harfington's 18-8 set: barrel head 1.9 mm tall, screw head 1.4; :mod:`hardware.sources`).
The barrel runs
the pin's whole stack, so every link on the pin bears on the 4 mm barrel; the
screw's head bottoms on the barrel's end, so the head-to-head distance is the
barrel length whatever the screw is tightened to, and the links turn between the
heads. Barrels come in fixed lengths (the catalog's ``CHICAGO_LENGTHS``: 1 mm steps
from 4 to 16 mm, then 18, 20, 22, 23, 25, 28 and on to 80; on the Strider at most 23 mm,
:data:`MAX_BARREL`) against a stack of 3 mm layers, so the construction picks the
shortest barrel that clears the stack plus the top spacer's 0.5 mm plus ``min_play`` and
takes up the rest with **one printed head spacer per end** (unclamped: the screw bottoms
on the barrel, so a spacer only sets the column's axial play): above the top link up to
what that end slot holds, the rest under the barrel's head. What is left is the column's
axial play, between ``min_play`` and ``min_play`` plus 0.1 mm, set by the barrel length
rather than by feel, plus the spacers' print tolerance (``PRINT_TOL``, counted as play).
(Until 2026-10-05 a PTFE washer and DIN 988 shim rings did this; the shim catalog item
still sets the spacers' 8 mm OD and 0.1 mm steps.)

Pins only (``--pin chicago``): a pillar's head would stand outside a frame plate.

**The default link pin** (``BuildConfig.pin == "chicago"``), from the pivot review of
2026-10-03 (with the printed pillars then; standoff pillars since): ``rod``, ``chicago``,
``chicago_bushing`` and ``bushing`` built and audited as robots on the Strider double (S)
and **the demo Klann** (``--linkage klann``, K) quad at phases 0,0,180,180 (``spiderpig
audit``; wobble from :mod:`construction.wobble`). The K columns are the demo's, not
``klann_lego``'s (the second test design: below the table). The jam safety factors are the
strength check's of 2026-10-03 (``docs/audit/STRENGTH.md``): each design's own MuJoCo loads,
jammed at the servo's 0.85 N·m torque limit, a two-link pin bending by ``F s / 2`` (the
review's first figures, 2.27 / 1.57 for K, were at a family-wide 155 N and ``F s / 4``, half
the bending; a first run with a soft foot pin, K 2.57 chicago / 1.62 rod, under-read the
jam); the demo Klann's 9 mm-span pin E fails jammed with either pin (245 N), which is the
demo's problem, not the pin's; the bushed rows were not re-run (the same barrel / rod, so
the same SF):

==================  =======  =========================  =============  ===========  ======
pin                 layers   pin tilt worst / mean       free tilt      jam SF       parts
                    S / K    deg (S; K the same worst)   deg            S / K        S / K
==================  =======  =========================  =============  ===========  ======
rod                 18 / 12  0.68 / 0.61 (assumed 0.1)  3.8            2.23 / 0.53  232/222
chicago             18 / 12  0.72 / 0.39 (0.05-0.15)    3.8 (0 host)   3.55 / 0.80  246/246
chicago_bushing     18 / 12  1.53 / 0.61 (sleeve gaps)  1.5            as chicago   223/215
bushing (rod)       18 / 12  1.27 / 0.88                1.3            as rod       253/239
==================  =======  =========================  =============  ===========  ======

(Part counts: the rod and chicago rows from the 2026-10-03 strength audits, which carry
the electronics deck; the bushed rows from the review, before it.)

**On ``klann_lego``** quad at 0,0,180,180 (``spiderpig audit --linkage klann_lego
--modules quad --phases 0,0,180,180 [--pin rod]``) every pin spans 3 mm: chicago against
rod, 13 layers each, jam SF 2.62 against 2.02 at its own loads (up to 174 N jammed, 12 N
walking; the family-wide 155 N it was first checked at is the demo Klann's), walking
55 against 43, pin tilt 0.72 / 0.36 against 0.59 / 0.59 deg worst / mean, 242 against
218 parts, $206.09 against $227.58.

No clash, contract or plan problem in any of the eight. What decided it: the same stacks
as the rod (one end layer each side, the head 8 x 1.5 mm with its washer against the
Starlock's 9.7 x 1.3 mm plus 0.5 mm of rod); the column's axial play set by the barrel
length and the shims (0.05-0.15 mm, checked with a feeler gauge) where the rod's clip is
set by feel (0.1 mm assumed; left 0.3 mm proud the rod's links tip 1.8 deg); the lowest
link bonded to the barrel, so it doesn't tilt at all and the mean tilt drops from 0.61
to 0.39 deg; a PTFE thrust face under the turning head; the 4 mm barrel's lower bending
stress, 0.6 times the rod's at any span (12.9 against 17.2 MPa bearing on the acrylic),
which matters where a pin spans more than a layer: the demo Klann's 9 mm-span pin E jams
at SF 0.80 against the rod's 0.53 (both fail there: 245 N jammed), the Strider's worst
pin, J7, at 3.55 against 2.23 (on ``klann_lego``'s 3 mm spans, 2.62 against 2.02,
above); nothing to cut or deburr; and a joint that comes apart (a Starlock is
single-use). Its costs: +14 parts on the Strider double (washers, shims), a glue step
per pin, and parts that are less documented than the rod (the head height is not from a
page). The bushed variant has the lowest free tilt but, with its printed sleeves kept
``flange_play`` off the flanges, more play, and $2.30 a bushing (+$91 on the Strider);
the rod stays selectable (``--pin rod``). The rod in a PTFE tube liner (3 x 4 mm), first
ruled out at the family-wide Klann loads (13-17 MPa on its 3 x 3 mm bore), is built
since the per-design loads (``--pin ptfe``, :mod:`.ptfe`): fine walking, but jammed a
warning on the Strider double (SF 1.79) and an error on ``klann_lego`` (0.52), where
this screw holds 3.55 / 2.62, so it stays an option. The thrust face under the head (a
PTFE washer at the review, a printed spacer since 2026-10-05) carries only axial load:
even a 155 N jam on a 36 mm^2 face is 4.3 MPa, and the walking axial load is a small
fraction of that.

* ``chicago``: plain 4.2 mm running holes in the links, printed spacer rings
  between (a 4.2 mm bore; an 8.5 mm laser-cut ring is under both services' smallest
  part), a **printed head spacer** under the screw's head (and under the barrel's where
  the take-up needs it): the head turns against it.
* ``chicago_bushing``: an igus GFM-0405-03 flange bushing (4 x 5.5 x 3, flange
  9.5 x 0.75) pressed into every link but the lowest, printed sleeves between
  (:mod:`.insert`); the flanges are the thrust faces, so the top spacer has no washer's
  0.5 mm in it, but the claim's spacers grow to the 9.5 mm flange.

**The lowest link is bonded to the barrel** (slow two-part epoxy in a 4.15 mm glue-fit
hole: CA crazes acrylic; about 0.75 N·m, well over the screw's tightening torque), in
both: assembly is bottom up, so once the lowest link is on nothing reaches the
barrel's head, and a barrel that isn't held spins with the screw. Bonded, the
barrel is held by holding that link, which is always within reach (it is the
pin's host, the link its other parts ride with), and the joint comes apart by
unscrewing the screw against it. The lowest link then turns with the barrel
and every other link turns on it.

Assembly, bottom up: glue each pin's barrel into its lowest link with its head
underneath (its lower printed spacer, where it has one, under the head as a gluing jig);
thread the rings (or sleeves) and links on in layer order as the layer plan says; at the
pin's top link put on its upper printed spacer, check with a feeler gauge that the barrel
stands 0.05-0.15 mm proud of it (a sheet thicker than nominal: sand the spacer down or
reprint it thinner), put a drop of low-strength
threadlocker on the screw and drive it from above (Phillips or slot, the only
tool) holding the lowest link until it bottoms. The links must turn freely. The
tool comes in from above, the side assembly is open on, so no part placed later
is in its way.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import ClassVar

from spiderpig.construction.axle import AxleDims, AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Context, Realized, hardware
from spiderpig.construction.pivots.common import (
    EPS,
    SLEEVE_COLOR,
    STEEL,
    Column,
    bored,
    column_air,
    gap_washers,
    host_of,
    ring_z,
    stem_of,
    xy_of,
)
from spiderpig.construction.pivots.insert import InsertAxle
from spiderpig.construction.wobble import Section, column_wobble
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.hardware.fastener_catalog import CHICAGO_LENGTHS, chicago
from spiderpig.materials import washer_od
from spiderpig.shapes import Cut, disc, ring, union
from spiderpig.stack import Unbuildable

PTFE_COLOR = "#f2f2f2"


def _hw_tol() -> float:
    """The printed head spacers' height tolerance, counted as play."""
    from spiderpig.construction.pivots.common import PRINT_TOL

    return PRINT_TOL


@dataclass(frozen=True)
class Fit:
    """A Chicago screw's barrel length and how the stack takes it up."""

    length: float
    shims_lo: float
    shims_hi: float
    play: float

    @property
    def shims(self) -> float:
        return self.shims_lo + self.shims_hi


@dataclass(frozen=True)
class ChicagoShaft:
    """The Chicago screw as a pin's shaft: the same interface as
    :class:`construction.pivots.common.RodShaft` (``d``, ``check``, ``clip``, ``realize``)."""

    roles: ClassVar[tuple[str, ...]] = ("pin",)       # a pillar's head would leave the plates
    washer_key: str | None = "ptfe_washer_4x8x0p5"   # the thrust face under the screw's head:
    #                                  its 0.5 mm is part of the printed top spacer (None: a
    #                                  flange is the face)
    shim_key: str = "shim_din988_4x8"   # the take-up's steps and the spacers' OD (printed)
    lock_key: str = "threadlocker_222"
    min_play: float = 0.05          # least axial play left in the column
    max_length: float | None = None  # the longest barrel a pin may take (None: the longest
    #                                  stock): a planner rule, a long barrel being a long span;
    #                                  per linkage, MAX_BARREL (ChicagoAxle.resolve)
    model_gap: float = 0.01
    glue_fit: float = 0.15          # the lowest link's hole over the barrel (bonded)
    glue_per_pin: float = 0.005
    lock_per_pin: float = 0.01

    def item(self, length: float = CHICAGO_LENGTHS[0]) -> dict:
        return get(chicago(length)).dims

    @property
    def d(self) -> float:
        return float(self.item()["barrel_d"])

    @property
    def shim_steps(self) -> tuple[float, ...]:
        return tuple(float(t) for t in get(self.shim_key).dims["t"])

    @property
    def washer_t(self) -> float:
        return float(get(self.washer_key).dims["t"]) if self.washer_key else 0.0

    def clip(self) -> tuple[float, float]:
        """(outside diameter, height) of an end: the head (and the washer under the top one)."""
        it = self.item()
        return float(it["head_d"]), float(it["head_h"]) + self.washer_t

    def check(self, ctx, pillar: bool, extra: float = 0.0) -> None:
        if pillar:
            raise ConstructionError("a Chicago screw is a link pin only: a pillar's head would "
                                    "stand outside the frame plate")
        it = self.item()
        top = float(it["screw_head_h"]) + self.washer_t + self.min_play + extra
        if top > ctx.pitch + EPS:
            raise ConstructionError(f"a {it['screw_head_h']:g} mm Chicago screw head, its washer "
                                    f"and play don't fit a {ctx.pitch:g} mm layer")
        steps = [b - a for a, b in zip(CHICAGO_LENGTHS, CHICAGO_LENGTHS[1:], strict=False)
                 if b <= 22]      # (longer stacks: fit() says if the shims fit)
        room = (ctx.pitch - top) + (ctx.pitch - float(it["head_h"]))
        if max(steps, default=0.0) > room + min(self.shim_steps) + EPS and not \
                __import__("os").environ.get("SPIDERPIG_BARRELS"):
            raise ConstructionError(f"a {max(steps):g} mm step between barrel lengths needs "
                                    f"more shims than two {ctx.pitch:g} mm end layers hold")

    def column(self, pillar: bool, links, top: int, anchored: tuple[bool, bool],
               pitch: float, layout=None, air=None) -> None:
        """The planner's rule: a stock barrel must span the pin's links (their layers, the
        washer and the least play; at the plan's own z, its gaps and thicker plates
        included): :class:`stack.Unbuildable` otherwise."""
        if pillar or not links:
            return
        stack = (max(links) - min(links) + 1) * pitch
        if layout is not None and layout.final:
            stack = layout.z(max(links))[1] - layout.z(min(links))[0]
            if air is not None:         # its own parts' stack: the barrel closes the air
                stack -= air(min(links), max(links))
        need = stack + self.washer_t + self.min_play
        if need > max(CHICAGO_LENGTHS) + EPS:
            raise Unbuildable(f"no stock Chicago screw spans its {stack:g} mm stack (longest "
                              f"{max(CHICAGO_LENGTHS):g} mm)")
        if self.max_length is not None and need > self.max_length + EPS:
            raise Unbuildable(f"its {stack:g} mm stack needs a barrel over the "
                              f"{self.max_length:g} mm a pin may take (its bending)")
        length = next(L for L in CHICAGO_LENGTHS if need - EPS <= L)
        it = self.item()
        room = 2 * pitch - float(it["head_h"]) - float(it["screw_head_h"]) - self.washer_t
        if length - need > max(room, 0.0) + max(self.shim_steps) + EPS:
            # the longer barrels come in 5 mm steps: more shims than its end slots hold
            raise Unbuildable(f"no stock Chicago screw fits its {stack:g} mm stack: a "
                              f"{length:g} mm barrel leaves {length - need:.1f} mm of shims")

    def fit(self, stack: float, pitch: float, extra_hi: float = 0.0,
            extra_lo: float = 0.0, *, slot_hi: float | None = None,
            slot_lo: float | None = None, clear_hi: float = 0.0,
            clear_lo: float = 0.0) -> Fit:
        """The barrel for a ``stack`` mm stack (link faces, flanges included), its shims and
        the play left; ``extra_hi`` / ``extra_lo``: a flange already in the end slot. An
        end's slot is a layer (``pitch``) or the clearance gap it sits in (``slot_hi`` /
        ``slot_lo``, its height, with ``clear_*`` kept free under the next layer); the
        shims go under the screw's head as far as its slot holds them, the rest under the
        barrel's."""
        it = self.item()
        lo, hi, length, play = self._split(stack, pitch if slot_hi is None else slot_hi,
                                           extra_hi + clear_hi)
        room_lo = (pitch if slot_lo is None else slot_lo) - float(it["head_h"]) - extra_lo \
            - clear_lo
        if lo > room_lo + EPS:
            raise ConstructionError(f"a {length:g} mm barrel over a {stack:.1f} mm stack leaves "
                                    f"{lo + hi:.1f} mm of shims, more than the end slots hold")
        return Fit(float(length), round(lo, 3), round(hi, 3), round(play, 3))

    def _split(self, stack: float, slot_hi: float, extra_hi: float
               ) -> tuple[float, float, float, float]:
        """(shims under the barrel's head, under the screw's, the barrel length, the play)."""
        it = self.item()
        need = stack + self.washer_t + self.min_play
        length = next((L for L in CHICAGO_LENGTHS if need - EPS <= L), None)
        if length is None:
            raise ConstructionError(f"no stock Chicago screw spans a {stack:.1f} mm stack "
                                    f"(longest {max(CHICAGO_LENGTHS):g} mm)")
        step = min(self.shim_steps)
        excess = length - need
        shims = math.floor(excess / step + 1e-6) * step
        play = self.min_play + excess - shims
        room_hi = slot_hi - float(it["screw_head_h"]) - self.washer_t - play - extra_hi
        hi = min(shims, math.floor(max(room_hi, 0.0) / step + 1e-6) * step)
        return round(shims - hi, 3), round(hi, 3), float(length), round(play, 3)

    def base_heights(self) -> tuple[float, float]:
        """What each end needs in a clearance gap before any shims: the barrel's head, the
        screw's head and its washer, each with the clearance under the next layer."""
        from spiderpig.construction.pivots.common import HEAD_CLEARANCE

        it = self.item()
        return (float(it["head_h"]) + HEAD_CLEARANCE,
                float(it["screw_head_h"]) + self.washer_t + HEAD_CLEARANCE)

    def end_heights(self, L, k0: int, k1: int, extra: tuple[float, float] = (0.0, 0.0),
                    air: float = 0.0) -> tuple[float, float]:
        """What each end needs at the plan's z: its head (and washer) and the shims the
        barrel's length leaves, under the screw's head as far as its slot (the gap the
        plan gave it, else the layer it sank into) holds them, the rest under the
        barrel's; with the clearance in a gap."""
        from spiderpig.construction.pivots.common import HEAD_CLEARANCE

        it = self.item()
        stack = L.z(k1)[1] - L.z(k0)[0] + extra[0] + extra[1] - air
        gap_hi, gap_lo = L.gap(k1), L.gap(k0 - 1)
        try:
            if not gap_hi:          # the screw's head sank into the layer over: shims there
                lo, hi, _, play = self._split(stack, L.t(k1 + 1), extra[1])
            else:                   # else half each end (the barrel's head's in a layer: all)
                lo, hi, _, play = self._split(stack, 0.0, extra[1])
                if gap_lo:
                    step = min(self.shim_steps)
                    hi = math.floor((lo + hi) / 2 / step + 1e-6) * step
                    lo = round(lo - hi, 3)
        except ConstructionError:
            lo = hi = play = 0.0
        h_hi = float(it["screw_head_h"]) + self.washer_t + hi + play + extra[1]
        h_lo = float(it["head_h"]) + lo + extra[0]
        # a head in a layer bears on the plate over it, as it always has; in a gap, it
        # keeps a clearance under the next layer
        return (h_lo + (HEAD_CLEARANCE if gap_lo else 0.0),
                h_hi + (HEAD_CLEARANCE if gap_hi else 0.0))

    def shim_count(self, total: float) -> int:
        """Shim rings for ``total`` mm, thickest first."""
        n, left = 0, round(total, 3)
        for t in sorted(self.shim_steps, reverse=True):
            k = int(math.floor(left / t + 1e-6))
            n, left = n + k, round(left - k * t, 3)
        return n

    def host_hole(self) -> float:
        return self.d + self.glue_fit

    def realize(self, build: Build, group: AxleGroup, col: Column, out: Realized, *,
                faces: dict[str, float] | None = None) -> Fit:
        """The Chicago screw, its printed head spacers, and the bonded lowest link's hole. An
        end in a clearance gap keeps :data:`common.HEAD_CLEARANCE` under the next layer."""
        from spiderpig.construction.pivots.common import HEAD_CLEARANCE

        faces = faces or {}
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        z_lo = build.z(col.k0)[0] - faces.get("lo", 0.0)
        z_hi = build.z(col.k1)[1] + faces.get("hi", 0.0)
        lo_slot, hi_slot = col.end_z(build, "lo"), col.end_z(build, "hi")
        if lo_slot is None or hi_slot is None:
            raise ConstructionError(f"{group.name}: a Chicago screw needs a slot at both ends")
        air = column_air(build, group, col.k0, col.k1)
        f = self.fit(z_hi - z_lo - air, build.ctx.pitch, faces.get("hi", 0.0),
                     faces.get("lo", 0.0),
                     slot_hi=hi_slot[1] - hi_slot[0], slot_lo=lo_slot[1] - lo_slot[0],
                     clear_hi=HEAD_CLEARANCE if col.hi_gap else 0.0,
                     clear_lo=HEAD_CLEARANCE if col.lo_gap else 0.0)
        it = self.item(f.length)
        d, head_d = self.d, float(it["head_d"])
        zb = z_lo - f.shims_lo                      # barrel head's top face
        zt = zb + f.length + air                    # the screw head's underside (the model's
        #                                             stack keeps the air the barrel closes)
        if zb - float(it["head_h"]) < lo_slot[0] - EPS or \
                zt + float(it["screw_head_h"]) > hi_slot[1] + EPS:
            raise ConstructionError(f"{group.name}: a {f.length:g} mm Chicago screw leaves its "
                                    "end slots")
        screw = union([disc(xy, head_d / 2, zb - float(it["head_h"]), zb),
                       disc(xy, (d - self.model_gap) / 2, zb, zt),
                       disc(xy, head_d / 2, zt, zt + float(it["screw_head_h"]))])
        out.bodies.append(hardware(f"{stem}_screw", screw, host, fab="purchased",
                                   bom_key=chicago(f.length), color=STEEL))
        z = z_hi
        # the PTFE washer and the take-up shims as one printed spacer per end (unclamped:
        # the screw bottoms on the barrel, so they only set the column's axial play)
        for tag, z0, t in (("hi", z, self.washer_t + f.shims_hi), ("lo", zb, f.shims_lo)):
            if t <= EPS:
                continue
            sp = bored(disc(xy, float(get(self.shim_key).dims["od"]) / 2, z0, z0 + t), xy,
                       d + 0.2, z0, z0 + t)
            out.bodies.append(hardware(f"{stem}_spacer_{tag}", sp, host, fab="printed",
                                       color=SLEEVE_COLOR))
        out.extras.append(BomLine("epoxy_2part", self.glue_per_pin,
                                  f"{group.name}: barrel into {host} (slow epoxy: CA "
                                  "crazes acrylic)"))
        out.extras.append(BomLine(self.lock_key, self.lock_per_pin, group.name))
        out.cut(host, Cut(xy, self.host_hole()))
        out.notes.setdefault("chicago", {})[group.name] = {
            "length_mm": f.length, "stack_mm": round(z_hi - z_lo, 3),
            "spacer_lo_mm": f.shims_lo, "spacer_hi_mm": round(self.washer_t + f.shims_hi, 3),
            "play_mm": f.play, "item": chicago(f.length), "printed": True}
        return f


def chicago_section(shaft: ChicagoShaft) -> Section:
    """The barrel as a 304 tube, 4 mm over the M3 thread's 3 mm major diameter (the screw's
    core inside it ignored: conservative)."""
    return Section.tube(shaft.d, 3.0, 215.0, name="chicago barrel 4 x 3 tube")


MAX_BARREL: dict[str, float] = {"strider": 23.0}
"""The longest barrel a linkage's pins may take (a planner rule, :meth:`ChicagoAxle.resolve`):
a long barrel is a long span, and a pin bends as its span. The Strider (2026-10-05): the quad's
J7 on a 30 mm barrel, its links 26 mm apart at the plan's z, was jam SF 1.8; capped at 23 mm
the same 24 layers put it on 23 mm (SF 2.6), and the quad buys 8 barrel lengths, not 10 (the
double and single plan as before). Not the Klann: the demo quad needs its longer barrels
(capped at 23 it finds no plan in 60 s)."""


@dataclass(frozen=True)
class ChicagoAxle:
    """M3 Chicago screw, printed spacer rings and head spacers (pins only)."""

    key: str = "chicago"
    label: str = ("M3 Chicago screw (4 mm barrel) through the stack, printed rings and head "
                  "spacers; lowest link bonded (pins only)")
    running_fit: float = 0.2      # a link's and a ring's hole over the 4 mm barrel
    shaft: ChicagoShaft = field(default_factory=ChicagoShaft)

    @property
    def roles(self) -> tuple[str, ...]:
        return self.shaft.roles

    def hole(self) -> float:
        return self.shaft.d + self.running_fit

    def column(self, *args, **kw) -> None:
        self.shaft.column(*args, **kw)

    def resolve(self, ctx: Context) -> ChicagoAxle:
        """This construction for the design ``ctx`` builds: its linkage's longest barrel
        (:data:`MAX_BARREL`)."""
        cap = MAX_BARREL.get(getattr(ctx.config, "linkage", None))
        if cap is None or self.shaft.max_length is not None:
            return self
        return replace(self, shaft=replace(self.shaft, max_length=cap))

    def end_heights(self, d: AxleDims, L, k0: int, k1: int, air: float = 0.0
                    ) -> tuple[float, float]:
        return self.shaft.end_heights(L, k0, k1, air=air)

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        p = ctx.params
        self.shaft.check(ctx, pillar)
        if self.hole() / 2 + p.min_wall > p.link_radius:
            raise ConstructionError(f"a {self.shaft.d:g} mm barrel's hole leaves less than "
                                    f"{p.min_wall} mm of link around it")
        ring_min = self.hole() / 2 + p.min_wall
        if ring_min > p.spacer_d / 2:
            raise ConstructionError(f"a {p.spacer_d} mm spacer ring leaves less than {p.min_wall} "
                                    f"mm around a {self.hole():.2f} mm hole")
        head_d, _ = self.shaft.clip()
        w_od = float(get(self.shaft.washer_key).dims["od"]) if self.shaft.washer_key else 0.0
        return AxleDims(axle=self.shaft.d / 2, spacer=p.spacer_d / 2,
                        head=max(head_d, w_od) / 2, neck=ring_min, fill=True,
                        end_h=self.shaft.base_heights(), washer=washer_od(self.shaft.d) / 2)

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        out = Realized()
        col = Column.of(build, group)
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        for k in col.between:
            role, r = col.roles[k]
            if role == "neck":
                raise ConstructionError(f"{group.name}: a barrel can't neck down (layer {k})")
            z0, z1 = ring_z(build, k)
            out.bodies.append(hardware(f"{stem}_ring{k}", ring(xy, 2 * r, self.hole(), z0, z1),
                                       host, fab="printed", color=SLEEVE_COLOR))
        f = self.shaft.realize(build, group, col, out)
        gap_washers(build, group, col, out, self.shaft.d, host, stem)
        for m in group.axis.members:
            if m != host:
                out.cut(m, Cut(xy, self.hole()))
        out.notes["wobble"] = {group.name: column_wobble(
            build, group, col,
            clearance=lambda m: 0.0 if m == host else self.running_fit,
            length=build.ctx.pitch, play=f.play + _hw_tol(),
            play_basis=f"barrel length less stack, washer and shims ({f.length:g} mm barrel)",
            section=chicago_section(self.shaft))}
        return out


CHICAGO_BUSHING = InsertAxle(
    key="chicago_bushing",
    label=("igus GFM-0405-03 flange bushing pressed in each link but the lowest, M3 Chicago "
           "screw (4 mm barrel), printed sleeves, shims (pins only)"),
    insert="bushing_gfm0405_03", seat_fit=0.02, glued=False,
    shaft=ChicagoShaft(washer_key=None), spacer_d=10.5,
    bore_clearance=0.06,
)
"""E10 after pressing (4.020-4.068 mm, igus) on the barrel (4 mm nominal, its tolerance not
published): about 0.06 mm diametral at mid tolerances."""
