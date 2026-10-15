"""``pivot8``: an 8 mm hardened steel pivot pin, retained by an E-ring at each end, as the link
pin: a decanting cradle's (``config.PRODUCT_PINS``, since 2026-10-13: a 6 L bottle's jam bent
the M6 Chicago screw's 8 x 6 stainless barrel, B's rod pin P at jam SF 0.74).

**The part.** MISUMI's precision pivot pin, straight, retaining rings (CCGH8-L-NC2.4:
:func:`hardware.cradle_catalog.pivot_pin8`): a solid 8 mm g6 pin of 1045 steel hardened to
40-45 HRC, black oxide, the length L between its two ring grooves made to order in 0.1 mm
steps (10-100 mm, +0.1/-0), the pin 2.4 mm past each (the alteration NC2.4, the least; 3 mm
standard); two E-type rings No. 7 (14 mm across, 0.8 mm thick) come with it. The links turn
on the pin between the rings, as on a Chicago screw's barrel between its heads (the M6's
8.2 mm holes, so the links don't change): it rides on
:class:`construction.pivots.chicago.ChicagoShaft`'s column, its "heads" each ring and the
pin past it. L made to the stack (its stack plus :attr:`ChicagoShaft.min_play`, to the
0.1 mm) leaves no take-up: the column's play is the least plus under 0.1 mm, plus the +0.1
of L's tolerance.

**Its ends.** 2.4 mm past each link face (the ring, the pin's end), the play over the
highest link: they sink into the layer beside their links as the M6 Chicago screw's heads
did, or stand in the clearance gap there (with 0.25 mm of clearance: the cradle link's, under
the inner plate). The standard 3 mm end, with the play, was a hair over a 3.175 mm layer, and
every end of B's pins took a gap: 94.5 mm of stack against 77. :meth:`PivotPinShaft.check`
allows an end up to :data:`stack.GAP_MAX`, not the pitch.

**Why not the shoulder screw** the user picked on 2026-10-13 (ISO 7379, 8 mm, McMaster
92981A, M6; built and planned that day): its 6 mm head over the highest link and the 11 mm
of thread, washer and nut under the lowest took whole layers, so B's links spread over its
stack (88-91 mm against 77, pin X's span 40 mm) and its rod rose off the crank block (the
crankpin at jam SF 0.32); the pivot pin keeps the stack as the Chicago screws had it. Its
cost: about USD 21-24 a pin against 3 (``docs/agentlib/DECISIONS.md``).

**Strength** (:mod:`spiderpig.strength`): a solid 8 mm rod (the ring grooves lie past the
outermost links, where nothing bends it) at :data:`hardware.cradle_catalog.PIVOT8_YIELD`,
class 8.8's minimum yield (640 MPa) for a pin hardened to 40-45 HRC: 1.46 x the M6 barrel's
section and 3 x its 215 MPa, 4.3 x in all.

**Assembly** (:meth:`PivotPinAxle.assembly`): on the bench, a ring into the pin's lower
groove and the pin up through its lowest link; that link onto the stack, the links and rings
over it in layer order; once the highest link is on, the other ring into the upper groove
(E-ring pliers, or pushed on sideways). Every link must turn freely and lift its play.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from spiderpig.construction.axle import AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Realized, hardware
from spiderpig.construction.pivots.chicago import ChicagoAxle, ChicagoShaft, Fit
from spiderpig.construction.pivots.common import (
    EPS,
    HEAD_CLEARANCE,
    column_air,
    host_of,
    stem_of,
    xy_of,
)
from spiderpig.construction.wobble import Section
from spiderpig.hardware.catalog import get
from spiderpig.shapes import disc, ring, union
from spiderpig.stack import Unbuildable

BLACK_OXIDE = "#2b2b2b"


def _lengths() -> tuple[float, ...]:
    from spiderpig.hardware.cradle_catalog import PIVOT8_LENGTHS

    return PIVOT8_LENGTHS


@dataclass(frozen=True)
class PivotPinShaft(ChicagoShaft):
    """The pivot pin as a Chicago screw's part in the column: the barrel is the pin between
    its rings, each head a ring and the pin's end past it (``head_h``, ``screw_head_h``); no
    thrust face (``washer_t`` 0: the ring bears on the link), nothing threaded."""

    family: str = "pivot8"
    lengths: tuple[float, ...] = field(default_factory=_lengths)
    bonded: bool = False
    washer_key: str = "ptfe_washer_8x14x0p5"   # (its OD only: the rings' claims; no thrust
    #                                           face, washer_t 0)
    shim_key: str = "shim_din988_8x14"   # (the printed rings' OD; no take-up)
    min_play: float = 0.25          # least axial play (as the M6 Chicago's)
    lock_key: str = ""              # (nothing threaded: no threadlocker)

    def key_of(self, length: float) -> str:
        from spiderpig.hardware.cradle_catalog import pivot_pin8

        return pivot_pin8(length)

    @property
    def washer_t(self) -> float:
        return 0.0

    def check(self, ctx, pillar: bool, extra: float = 0.0) -> None:
        """A pin only, and each end (its ring and the pin past it, the play) fits a
        clearance gap (:data:`stack.GAP_MAX`): taller than a layer, it stands in the gap
        beside its link, or sinks into a layer thick enough."""
        from spiderpig.stack import GAP_MAX

        if pillar:
            raise ConstructionError("a pivot pin is a link pin only: its rings would stand "
                                    "outside the frame plates")
        it = self.item()
        end = float(it["screw_head_h"]) + self.min_play + extra + HEAD_CLEARANCE
        if end > GAP_MAX + EPS:
            raise ConstructionError(f"a pivot pin's {it['screw_head_h']:g} mm end and its play "
                                    f"don't fit a {GAP_MAX:g} mm clearance gap")

    def column(self, pillar: bool, links, top: int, anchored: tuple[bool, bool],
               pitch: float, layout=None, air=None, hi=None) -> None:
        """The Chicago screw's rule (no pin longer than the longest), and the pin's own: made
        to its stack, it takes no shims, so the stack and the least play must reach the
        shortest length (:meth:`realize` refuses the rest). A minimum: a search's layout
        refuses only when even its upper bound of the stack falls short (``hi``, the most
        the axle's stack can be in any plan); the plan's own z decides."""
        super().column(pillar, links, top, anchored, pitch, layout=layout, air=air, hi=hi)
        if pillar or not links or layout is None:
            return
        a, b = min(links), max(links)
        if layout.final:
            stack = layout.span(a, b) - (air(a, b) if air is not None else 0.0)
        elif hi is not None:
            stack = hi(a, b)
        else:
            return
        need = stack + self.washer_t + self.min_play
        length = next((L for L in self.lengths if need - EPS <= L), None)
        step = min(self.shim_steps)
        if length is not None and length - need >= step - EPS:
            raise Unbuildable(f"a pivot pin is made to its stack, and its stack "
                              f"{'is' if layout.final else 'can reach at most'} {stack:.2f} mm: "
                              f"{length - need:.2f} mm under the shortest pin ({length:g} mm) "
                              "with its play")

    def section(self) -> Section:
        from spiderpig.hardware.cradle_catalog import PIVOT8_YIELD

        return Section.rod(self.d, PIVOT8_YIELD,
                           name=f"{self.d:g} mm hardened steel pivot pin (solid, "
                                f"{PIVOT8_YIELD:g} MPa)")

    def realize(self, build: Build, group: AxleGroup, col, out: Realized, *,
                faces: dict[str, float] | None = None) -> Fit:
        """The pin and its two rings (one purchase, one body), between its end slots: the
        lower ring under the lowest link, the upper one over the highest with the play
        under it (no take-up: L is made to the stack)."""
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        z_lo, z_hi = build.z(col.k0)[0], build.z(col.k1)[1]
        lo_slot, hi_slot = col.end_z(build, "lo"), col.end_z(build, "hi")
        if lo_slot is None or hi_slot is None:
            raise ConstructionError(f"{group.name}: a pivot pin needs a slot at both ends")
        air = column_air(build, group, col.k0, col.k1)
        f = self.fit(z_hi - z_lo - air, build.ctx.pitch,
                     slot_hi=hi_slot[1] - hi_slot[0], slot_lo=lo_slot[1] - lo_slot[0],
                     clear_hi=HEAD_CLEARANCE if col.hi_gap else 0.0,
                     clear_lo=HEAD_CLEARANCE if col.lo_gap else 0.0)
        if f.shims > EPS:
            raise ConstructionError(f"{group.name}: a pivot pin is made to its stack; "
                                    f"{f.shims:.1f} mm left to take up")
        it = self.item(f.length)
        end, ring_t, groove = (float(it["screw_head_h"]), float(it["ring_t"]),
                               float(it["groove_w"]))
        ring_od, groove_d = float(it["head_d"]), float(it["groove_d"])
        zb = z_lo                                   # the lower groove's inner edge
        zt = zb + f.length + air                    # the upper one's (the model keeps its air)
        if zb - end < lo_slot[0] + (HEAD_CLEARANCE if col.lo_gap else 0.0) - EPS or \
                zt + end > hi_slot[1] - (HEAD_CLEARANCE if col.hi_gap else 0.0) + EPS:
            raise ConstructionError(f"{group.name}: an {f.length:g} mm pivot pin's ends leave "
                                    "their slots")
        pin = union([disc(xy, (self.d - self.model_gap) / 2, zb - end, zt + end),
                     ring(xy, ring_od, groove_d, zb - groove + (groove - ring_t) / 2,
                          zb - (groove - ring_t) / 2),
                     ring(xy, ring_od, groove_d, zt + (groove - ring_t) / 2,
                          zt + groove - (groove - ring_t) / 2)])
        out.bodies.append(hardware(f"{stem}_pin", pin, host, fab="purchased",
                                   bom_key=self.key_of(f.length), color=BLACK_OXIDE))
        out.notes.setdefault("pivot_pin", {})[group.name] = {
            "length_mm": f.length, "stack_mm": round(z_hi - z_lo - air, 3),
            "play_mm": round(f.play, 3), "item": self.key_of(f.length)}
        return f


@dataclass(frozen=True)
class PivotPinAxle(ChicagoAxle):
    """An 8 mm hardened pivot pin through the stack, an E-ring at each end; printed rings
    between the links (pins only)."""

    key: str = "pivot8"
    label: str = ("8 mm hardened steel pivot pin (MISUMI CCGH8, made to length) through the "
                  "stack, an E-ring at each end; printed rings (pins only): a decanting "
                  "cradle's")
    shaft: ChicagoShaft = field(default_factory=PivotPinShaft)

    def assembly(self, group: AxleGroup, view) -> list:
        """How a pin goes on (:mod:`construction.assembly`): the pin with its lower ring up
        through its lowest link on the bench (the link's step), the upper ring once its
        highest link is on. The pin is one body (pin and rings, one purchase): its two
        pieces split under the upper ring."""
        from spiderpig.construction.assembly import STACK, Op, Piece

        stem = stem_of(group)
        pin = f"{stem}_pin"
        if pin not in view.z:
            return []
        z0, z1 = view.z[pin]
        key = view.keys.get(pin)
        end = float(get(key).dims.get("screw_head_h", 3.0)) if key else 3.0
        cut = z1 - end - 0.05
        host = view.hosts.get(pin)
        k_lo = view.slot(view.z[host][0]) if host in view.z else view.slot(z0)
        k_hi = view.slot(cut)
        on = ("Each pivot pin: an E-ring into its lower groove, then the pin up through its "
              "lowest link from under it (the ring under the link), on the bench; the link "
              "goes on with the pin standing in it.")
        ring_on = ("Each pivot pin's other E-ring into its upper groove once its highest link "
                   "is on (E-ring pliers, or pushed on sideways with a flat screwdriver): the "
                   "links must turn freely and lift their play (0.25-0.35 mm; the worm can't "
                   "be turned from the links: turn the gearbox's input shaft).")
        return [Op(STACK, (k_lo, 0, 1), (Piece(pin, "pin", (z0, cut)),), "", on, "layer"),
                Op(STACK, (k_hi, 2), (Piece(pin, "ring", (cut, z1)),), "", ring_on, "layer",
                   count=False)]


PIVOT8 = PivotPinAxle()
"""The 8 mm hardened pivot pin (MISUMI CCGH8): a decanting cradle's pins
(``config.PRODUCT_PINS``)."""

