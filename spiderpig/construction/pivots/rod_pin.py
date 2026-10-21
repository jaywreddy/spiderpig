"""``rod516``, ``rod38``, ``rodclip516``: a decanting cradle's link pins cut from 6061-T6
cold-finished round rod (simplification round 1, 2026-10-21: the user's materials rule, no
steel in a structural part; they replace the hardened MISUMI CCGH8 pivot pins and the 18-8
McMaster clevis pins: :mod:`construction.pivots.pivot_pin`, :mod:`construction.pivots.clevis`,
still there).

**The parts.** A pin is a length of 6061-T6 round rod (:data:`ROD_D`: 5/16 in, or 3/8 in where
the 5/16 in doesn't hold the clutch's cap: B's rod pin P, A's E), cold-finished (ASTM B211,
its diameter within :data:`ROD_TOL`), cut to its stack plus the least play (to 0.1 mm,
+0.1/-0: faced on the Carvera in a printed fixture, or filed to a caliper), and

* ``rod516`` / ``rod38``: each end drilled and tapped M3 (:data:`END_TAP`), an M3 x 6 ISO 7380
  button head (A2) through a wide washer (:data:`END_WASHER`, DIN 9021 M4: 12 mm across over
  the link's hole) clamped on the rod's end face, threadlocker 222 on it: the links turn on
  the rod between the washers, as on the pivot pin between its E-rings (made to its stack,
  :class:`RodPinShaft`, on the pivot pin's column);
* ``rodclip516``: B's X and Y (the cradle box comes off): one end so (its "head" over a
  printed take-up spacer outside the lowest link, as the clevis pin's head), a 2 mm cross hole
  drilled at its usable length from that end (:data:`CLIP_STEP` steps; a V-block on the
  Carvera) for the hairpin cotter past the highest link, as the clevis pin's
  (:class:`RodClipShaft`, on the clevis pin's column).

The holes stay the 8 mm pins' 8.2 mm for the 5/16 in rod (:data:`ROD_FIT`), 9.79 mm for the
3/8 in.

**Strength** (:mod:`spiderpig.strength`): a solid rod at :data:`ROD_YIELD`, 6061-T6's minimum
(the sheets' own, ``sheet_catalog.MIN_YIELD``); the tapped ends lie past the outermost links,
where nothing bends the pin. The 18-8 clevis pin was rated at 205 MPa, the hardened pivot pin
at 640: at P and E the 3/8 in rod's section (1.67 x the 8 mm's) takes some of that back.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from spiderpig.construction.axle import AxleDims, AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Realized, hardware
from spiderpig.construction.pivots.chicago import Fit
from spiderpig.construction.pivots.clevis import ClevisPinAxle, ClevisPinShaft, _clip
from spiderpig.construction.pivots.common import (
    EPS,
    HEAD_CLEARANCE,
    SLEEVE_COLOR,
    column_air,
    host_of,
    stem_of,
    xy_of,
)
from spiderpig.construction.pivots.pivot_pin import PivotPinAxle, PivotPinShaft
from spiderpig.construction.wobble import Section
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.shapes import difference, disc, ring, union

IN = 25.4
ROD_D: dict[str, float] = {"516": 5 / 16 * IN, "38": 3 / 8 * IN}
"""The pins' rod (mm): 5/16 in, 3/8 in."""
ROD_KEY: dict[str, str] = {"516": "rod_6061_516", "38": "rod_6061_38"}
"""Each rod's catalog key: a 12 in length, cut into pins (its pack)."""
ROD_YIELD = 241.3
"""MPa: 6061-T6's minimum yield, 35 ksi (the sheets' ``sheet_catalog.MIN_YIELD``; ASTM B211's
cold-finished rod and B221's extruded the same)."""
ROD_TOL = 0.0381
"""A cold-finished rod's diameter tolerance either way (mm): +-0.0015 in at 0.125-0.500 in
(Online Metals' aluminum tolerance table for 6061-T6 cold finish, ASTM B211, archived
2025-08-08; Alcobra Metals' table for its B211 6061 rod from 0.375 in, fetched 2026-10-21).
An extruded rod (ASTM B221, ANSI H35.2's Table 11.2) is +-0.008 in at 0.250-0.499: a laser's
8.2 mm hole could jam on it."""
ROD_FIT = 0.2625
"""A link's (and a printed ring's) hole over the rod: 8.2 mm for 5/16 in, the 8 mm pins' holes
unchanged (diametral play 0.26 nominal, 0.10-0.43 with the laser's +-0.127 and the rod's
:data:`ROD_TOL`, against the g6 pin's 0.08-0.34)."""
END_SCREW = (3.0, 6.0, "m3_bhcs_6")
"""Each tapped end's screw: M3 x 6 ISO 7380 A2 (thread, length, catalog key); through the
washer, 5 mm into the rod."""
END_HEAD = (5.7, 1.65)          # its head: across, high (ISO 7380-1)
END_WASHER = "m4_washer_9021"
"""The wide washer under each end screw's head, clamped on the rod's end face: DIN 9021 M4,
4.3 x 12 x 1.0 (its 12 mm over the link's 8.2 or 9.79 mm hole; the button head's 5.7 mm on its
4.3 mm bore)."""
END_TAP = (2.5, 8.0)            # the M3 tap drill and its depth in each end
TAP_R = 0.97 * END_SCREW[0] / 2 + 0.05   # the tapped hole as drawn: round the screw's modelled
#                                          shank (the thread is the model's overlap otherwise)
LOCK = "threadlocker_222"       # on each end screw (a turning link's washer would back it out)
CLIP_STEP = 0.5                 # a clip pin's usable lengths: its cross hole drilled to the 0.5 mm
ALU = "#c4c8cc"


def _end() -> tuple[float, float, float]:
    """(the washer's OD, its thickness, the end's height: washer and head)."""
    w = get(END_WASHER).dims
    od, t = float(w["od"]), float(w["t"])
    return od, t, t + END_HEAD[1]


def _pin_lengths() -> tuple[float, ...]:
    return tuple(L / 10 for L in range(80, 1001))


def _clip_lengths() -> tuple[float, ...]:
    return tuple(k * CLIP_STEP for k in range(round(8.0 / CLIP_STEP), round(40.0 / CLIP_STEP) + 1))


def _section(d: float, what: str) -> Section:
    return Section.rod(d, ROD_YIELD, name=f"{d:.2f} mm 6061-T6 rod pin ({what}, cold-finished, "
                                          f"solid, {ROD_YIELD:g} MPa)")


@dataclass(frozen=True)
class RodPinShaft(PivotPinShaft):
    """The rod pin as the pivot pin's part in the column: the rod between its washers (made to
    its stack plus the least play, no take-up), each "head" a washer and an M3 button head on
    the rod's tapped end (``head_h``, ``screw_head_h``)."""

    family: str = "rod516"
    rod: str = "516"
    lengths: tuple[float, ...] = field(default_factory=_pin_lengths)
    washer_key: str = "ptfe_washer_8x14x0p5"   # (its OD only: the ends' claims)
    lock_key: str = LOCK

    def key_of(self, length: float) -> str:
        return ROD_KEY[self.rod]

    def item(self, length: float | None = None) -> dict:
        od, _, end = _end()
        return {"barrel_d": ROD_D[self.rod], "head_d": od, "head_h": end,
                "screw_head_h": end, "thread": 0.0,
                "length": self.lengths[0] if length is None else float(length),
                "yield_mpa": ROD_YIELD}

    def check(self, ctx, pillar: bool, extra: float = 0.0) -> None:
        from spiderpig.stack import GAP_MAX

        if pillar:
            raise ConstructionError("a rod pin is a link pin only: its ends would stand outside "
                                    "the frame plates")
        end = float(self.item()["screw_head_h"]) + self.min_play + extra + HEAD_CLEARANCE
        if end > GAP_MAX + EPS:
            raise ConstructionError(f"a rod pin's {end:.2f} mm end and its play don't fit a "
                                    f"{GAP_MAX:g} mm clearance gap")

    def section(self) -> Section:
        return _section(self.d, f"{self.rod[0]}/{self.rod[1:]} in")

    def realize(self, build: Build, group: AxleGroup, col, out: Realized, *,
                faces: dict[str, float] | None = None) -> Fit:
        """The rod (made to its stack: no take-up), a washer and a button head on each end:
        the lower washer under the lowest link, the upper over the highest with the play
        under it."""
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        z_lo, z_hi = build.z(col.k0)[0], build.z(col.k1)[1]
        lo_slot, hi_slot = col.end_z(build, "lo"), col.end_z(build, "hi")
        if lo_slot is None or hi_slot is None:
            raise ConstructionError(f"{group.name}: a rod pin needs a slot at both ends")
        air = column_air(build, group, col.k0, col.k1)
        f = self.fit(z_hi - z_lo - air, build.ctx.pitch,
                     slot_hi=hi_slot[1] - hi_slot[0], slot_lo=lo_slot[1] - lo_slot[0],
                     clear_hi=HEAD_CLEARANCE if col.hi_gap else 0.0,
                     clear_lo=HEAD_CLEARANCE if col.lo_gap else 0.0)
        if f.shims > EPS:
            raise ConstructionError(f"{group.name}: a rod pin is made to its stack; "
                                    f"{f.shims:.1f} mm left to take up")
        w_od, w_t, end = _end()
        zb = z_lo                                   # the rod's lower end face
        zt = zb + f.length + air                    # its upper one (the model keeps its air)
        if zb - end < lo_slot[0] + (HEAD_CLEARANCE if col.lo_gap else 0.0) - EPS or \
                zt + end > hi_slot[1] - (HEAD_CLEARANCE if col.hi_gap else 0.0) + EPS:
            raise ConstructionError(f"{group.name}: an {f.length:g} mm rod pin's ends leave "
                                    "their slots")
        d = self.d
        tap = disc(xy, TAP_R, zb - 1.0, zb + END_TAP[1])
        tap_hi = disc(xy, TAP_R, zt - END_TAP[1], zt + 1.0)
        rod = difference(disc(xy, (d - self.model_gap) / 2, zb, zt), tap, tap_hi)
        out.bodies.append(hardware(f"{stem}_pin", rod, host, fab="purchased",
                                   bom_key=self.key_of(f.length), color=ALU))
        s_d, s_len, s_key = END_SCREW
        for tag, face, s in (("lo", zb, -1.0), ("hi", zt, 1.0)):
            w0, w1 = sorted((face, face + s * w_t))
            out.bodies.append(hardware(f"{stem}_washer_{tag}", ring(xy, w_od, 4.3, w0, w1),
                                       host, fab="purchased", bom_key=END_WASHER,
                                       color="#b8b8b8"))
            bear = face + s * w_t                   # the head's bearing face
            h0, h1 = sorted((bear, bear + s * END_HEAD[1]))
            k0, k1 = sorted((bear, bear - s * s_len))
            screw = union([disc(xy, END_HEAD[0] / 2, h0, h1),
                           disc(xy, 0.97 * s_d / 2, k0, k1)])
            out.bodies.append(hardware(f"{stem}_screw_{tag}", screw, host, fab="purchased",
                                       bom_key=s_key, color="#b8b8b8"))
        out.extras.append(BomLine(self.lock_key, 2 * self.lock_per_pin, group.name))
        out.notes.setdefault("rod_pin", {})[group.name] = {
            "length_mm": f.length, "stack_mm": round(z_hi - z_lo - air, 3),
            "play_mm": round(f.play, 3), "d_mm": round(d, 4), "item": self.key_of(f.length)}
        return f


@dataclass(frozen=True)
class RodPinAxle(PivotPinAxle):
    """A 6061 rod pin through the stack, a washer and an M3 button head on each tapped end;
    printed rings between (pins only)."""

    key: str = "rod516"
    label: str = ("5/16 in 6061-T6 rod pin (cold-finished, cut to its stack) through the stack, "
                  "an M3 button head and a wide washer on each tapped end; printed rings (pins "
                  "only): a decanting cradle's")
    running_fit: float = ROD_FIT
    shaft: PivotPinShaft = field(default_factory=RodPinShaft)

    def dims(self, ctx, pillar: bool) -> AxleDims:
        """As the pivot pin's, its printed rings at least a wall round its hole (the 3/8 in
        rod's 9.79 mm hole is over the cradle's 12.5 mm rings' wall)."""
        from spiderpig.materials import washer_od

        p = ctx.params
        if self.hole() / 2 + p.min_wall <= p.spacer_d / 2:
            return super().dims(ctx, pillar)
        self.shaft.check(ctx, pillar)
        ring_min = self.hole() / 2 + p.min_wall
        if ring_min > p.link_radius:
            raise ConstructionError(f"a {self.shaft.d:g} mm pin's hole leaves less than "
                                    f"{p.min_wall} mm of link around it")

        head_d, _ = self.shaft.clip()
        w_od = float(get(self.shaft.washer_key).dims["od"])
        return AxleDims(axle=self.shaft.d / 2, spacer=ring_min, head=max(head_d, w_od) / 2,
                        neck=ring_min, end_h=self.shaft.base_heights(),
                        washer=washer_od(self.shaft.d) / 2)

    def assembly(self, group: AxleGroup, view) -> list:
        """How a pin goes on (:mod:`construction.assembly`): on the bench its lower washer and
        screw on (threadlocker 222), then the pin up through its lowest link; the upper
        washer and screw once its highest link is on."""
        from spiderpig.construction.assembly import STACK, Op, whole

        stem = stem_of(group)
        pin = f"{stem}_pin"
        if pin not in view.z:
            return []
        host = view.hosts.get(pin)
        z0 = view.z[pin][0]
        k_lo = view.slot(view.z[host][0]) if host in view.z else view.slot(z0)
        hi = [n for n in (f"{stem}_washer_hi", f"{stem}_screw_hi") if n in view.z]
        k_hi = view.slot(view.z[hi[0]][0]) if hi else k_lo
        on = ("Each rod pin, on the bench: its lower washer and M3 button head into one end "
              "(a drop of threadlocker 222), snug with a 2 mm hex key; the pin up through its "
              "lowest link from under it (the washer under the link), greased (the food-grade "
              "grease: aluminium on aluminium); the link goes on with the pin standing in it.")
        top = ("Each rod pin's upper washer and M3 button head once its highest link is on "
               "(threadlocker 222), snug: the links must turn freely and lift their play (a few "
               "tenths of a mm; the worm can't be turned from the links: turn the gearbox's "
               "input shaft). If they bind, the pin is short of its stack (a thick sheet: the "
               "calipered figures): sand the printed ring between its links by the excess.")
        lo = [n for n in (pin, f"{stem}_washer_lo", f"{stem}_screw_lo") if n in view.z]
        return [Op(STACK, (k_lo, 0, 1), whole(*lo), "", on, "layer"),
                Op(STACK, (k_hi, 2), whole(*hi), "", top, "layer", count=False)]


@dataclass(frozen=True)
class RodClipShaft(ClevisPinShaft):
    """The clip pin as the clevis pin's part in the column (``lengths``: its usable lengths,
    the head's face to the cross hole's near edge, :data:`CLIP_STEP` apart): its head, a
    washer and an M3 button head on its tapped end, outside the lowest link over a printed
    take-up spacer; its cross hole and clip past the highest."""

    family: str = "rodclip516"
    rod: str = "516"
    lengths: tuple[float, ...] = field(default_factory=_clip_lengths)

    def key_of(self, length: float) -> str:
        return ROD_KEY[self.rod]

    def item(self, length: float | None = None) -> dict:
        od, _, end = _end()
        u = self.lengths[0] if length is None else float(length)
        return {"barrel_d": ROD_D[self.rod], "head_d": od, "head_h": end, "screw_head_h": 0.0,
                "thread": 0.0, "length": u, "overall": u + 2.0 + 3.0, "hole_d": 2.0,
                "yield_mpa": ROD_YIELD}

    def section(self) -> Section:
        return _section(self.d, f"{self.rod[0]}/{self.rod[1:]} in, a clip's")

    def realize(self, build: Build, group: AxleGroup, col, out: Realized, *,
                faces: dict[str, float] | None = None) -> Fit:
        """The rod (one body, its tapped end and cross hole), its washer and button head, its
        printed take-up spacer and its clip, as the clevis pin's."""
        import math

        from build123d import Cylinder, Location

        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        z_lo, z_hi = build.z(col.k0)[0], build.z(col.k1)[1]
        air = column_air(build, group, col.k0, col.k1)
        f = self.take_up(z_hi - z_lo - air)
        it = self.item(f.length)
        hole, overall = float(it["hole_d"]), float(it["overall"])
        w_od, w_t, end = _end()
        zh = z_lo - f.shims_lo                      # the rod's head end (its washer's top)
        zc = zh + f.length + air + hole / 2         # the cross hole's (and the clip's) centre
        lo_room = build.z(col.k0 - 2)[0] if col.k0 >= 2 else -math.inf
        if zh - end - f.play < lo_room - EPS:
            raise ConstructionError(f"{group.name}: the clip pin's head and spacer leave the "
                                    "two layers outside its links")
        hi_slot = col.end_z(build, "hi")
        if hi_slot is None or zc + hole / 2 > hi_slot[1] + EPS:
            raise ConstructionError(f"{group.name}: the clip pin's clip leaves the gap over "
                                    "its highest link")
        d = self.d
        cross = Cylinder(hole / 2, d + 2.0, rotation=(0, 90, 0)).moved(
            Location((xy[0], xy[1], zc)))
        rod = difference(disc(xy, (d - self.model_gap) / 2, zh, zh + overall + air), cross,
                         disc(xy, TAP_R, zh - 1.0, zh + END_TAP[1]))
        out.bodies.append(hardware(f"{stem}_pin", rod, host, fab="purchased",
                                   bom_key=self.key_of(f.length), color=ALU))
        out.bodies.append(hardware(f"{stem}_washer_lo", ring(xy, w_od, 4.3, zh - w_t, zh),
                                   host, fab="purchased", bom_key=END_WASHER, color="#b8b8b8"))
        s_d, s_len, s_key = END_SCREW
        bear = zh - w_t
        out.bodies.append(hardware(f"{stem}_screw_lo", union([
            disc(xy, END_HEAD[0] / 2, bear - END_HEAD[1], bear),
            disc(xy, 0.97 * s_d / 2, bear, bear + s_len)]), host, fab="purchased",
            bom_key=s_key, color="#b8b8b8"))
        if f.shims_lo > EPS:
            out.bodies.append(hardware(f"{stem}_spacer_lo", ring(xy, w_od, d + 0.3, zh, z_lo),
                                       host, fab="printed", color=SLEEVE_COLOR))
        out.bodies.append(hardware(f"{stem}_clip", _clip(xy, zc, d), host,
                                   fab="purchased", bom_key="hairpin_cotter_2x50",
                                   color="#b8b8b8"))
        out.extras.append(BomLine(LOCK, self.lock_per_pin, group.name))
        out.notes.setdefault("rod_clip_pin", {})[group.name] = {
            "usable_mm": f.length, "stack_mm": round(z_hi - z_lo - air, 3),
            "spacer_mm": f.shims_lo, "play_mm": f.play, "d_mm": round(d, 4),
            "overall_mm": overall, "item": self.key_of(f.length), "clip": "hairpin_cotter_2x50",
            "past_link_mm": round(f.play + hole + self.tip(), 3)}
        return f


@dataclass(frozen=True)
class RodClipAxle(ClevisPinAxle):
    """A 5/16 in 6061 clip pin through the stack: its head (a washer and an M3 button head)
    over a printed spacer outside the lowest link, a hairpin cotter past the highest."""

    key: str = "rodclip516"
    label: str = ("5/16 in 6061-T6 rod pin (cold-finished), its head an M3 button head and a "
                  "wide washer on its tapped end inside the stack over a printed spacer, a "
                  "hairpin cotter (R-clip) through its drilled hole from the bay: a decanting "
                  "cradle's quick-release pin (pins only)")
    running_fit: float = ROD_FIT
    shaft: ClevisPinShaft = field(default_factory=RodClipShaft)

    def assembly(self, group: AxleGroup, view) -> list:
        """As the clevis pin's: the head end's washer and screw on first, on the bench."""
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
        lo = [n for n in (pin, f"{stem}_washer_lo", f"{stem}_screw_lo") if n in view.z]
        on = ("Each clip pin, on the bench: its washer and M3 button head into its tapped end (a "
              "drop of threadlocker 222), snug with a 2 mm hex key; its printed spacer over "
              "the pin, then the pin up through its lowest link from the link's outer face "
              "(the head and the spacer under the link), greased; the link goes on with the "
              "pin standing in it.")
        clip_on = ("Each clip pin's hairpin cotter (R-clip) through the pin's hole from the "
                   "bay side, by hand, its bent leg over the pin: the links must turn freely "
                   "and lift their play (about 0.3 mm). If the hole stays covered, the stack "
                   "is over the pin (a thick sheet: the calipered figures): sand the printed "
                   "spacer under its head by the excess. Where the pin holds the cradle link, "
                   "pull its clip to take the side off; it goes back in the pin's hole.")
        ops = [Op(STACK, (k_lo, 0, 1), whole(*lo, *spacer), "", on, "layer")]
        if clip in view.z:
            ops.append(Op(STACK, (k_hi, 2), whole(clip), "", clip_on, "layer"))
        return ops


ROD516 = RodPinAxle()
ROD38 = RodPinAxle(key="rod38",
                   label=("3/8 in 6061-T6 rod pin (cold-finished, cut to its stack) through the "
                          "stack, an M3 button head and a wide washer on each tapped end; "
                          "printed rings (pins only): a decanting cradle's rod pin"),
                   shaft=RodPinShaft(family="rod38", rod="38",
                                     washer_key="ptfe_washer_10x16x0p5"))
RODCLIP516 = RodClipAxle()
"""The 6061 rod pins: a decanting cradle's (``config.PRODUCT_PINS``,
``config.PRODUCT_PIN_JOINTS``)."""
