"""``bolt``: an M3 socket head cap screw as the axle, laser-cut rings, a nylock nut.

What most hobby walkers use. A **pin** is a screw with its head under the
lowest link and a flat washer plus nylock nut on the highest, the rings
between (as for ``rod``). A **pillar** is turned over: its head sits on the
inner frame plate's top face and the washer and nut under the outer plate,
so the plates are clamped in the joint (nothing to glue), and a pillar that
reaches only one plate keeps the same orientation. The nut is 4 mm tall, so
the nut end claims two layers (three for a pillar, where the thread's tip
may stand further proud); the screw is the shortest standard length that
passes the nut by ``min_tip`` and keeps its tip inside those layers, and a
stack no standard length fits is unbuildable for the planner.

The thread rides in the holes (an M3 is 2.87-2.98 mm across its threads):
holes are ISO 273 fine (3.2 mm) in the links, medium (3.4 mm) in rings and
plates. Tighten the nylock only until the joint still turns freely: the
links are clamped in series with the rings.

Not the default pin (that is :mod:`.chicago`; before it :mod:`.rod`), for two reasons the pin review
found (2026-10):

* **Access.** Assembly is bottom up (:mod:`construction.axle`), and a pin's
  head sits under its lowest link, so once that link is on there is no axial
  access for the 2.5 mm key; a nylock's prevailing torque then spins a free
  screw. Pre-joining a leg's links off the frame doesn't work either: the
  planner interleaves the two legs of a side in the same layers (Klann's
  leg0 and leg1 both occupy layers 2-4, Strider's likewise), so a pre-joined
  unit can't be lowered past the other leg.
* **Stack.** The nut end claims two layers (``pin_nut_layers``) against links
  that sweep the pin, so the Strider double doesn't plan within the default
  budget (3-16 layers ruled out, 17+ open at 60 CPU-s) and needs 19 layers
  (57 mm per side, +19 %) at ten times the budget, where rod and printed
  pins plan at 16 in seconds; the Klann quad takes 13 layers to their 12.

Strength isn't the issue: an A2-70 M3's 2.39 mm core at 450 MPa is about as strong
in bending as the 3 mm 304 rod (1.34 against 2.65 mm^3, 450 against 215 MPa), and the
strength check (``docs/audit/STRENGTH.md``) lists ``--pin bolt`` with its recomputed SF
among a weak pin's fixes, so the stainless offers in the catalog are fine. A
``bolt_captive`` variant would fix both points if a strong bolted pin is ever wanted: a
plain DIN 934 nut (2.4 mm, under a 3 mm sheet) keyed in a hex pocket of the top link with
threadlocker, the head driven from above; it claims no extra layer and needs
no access below.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from spiderpig.construction.axle import AxleDims, AxleGroup, End
from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    Realized,
    hardware,
)
from spiderpig.construction.pivots.common import (
    EPS,
    SLEEVE_COLOR,
    STEEL,
    Column,
    bored,
    gap_washers,
    hex_prism,
    host_of,
    ring_z,
    stem_of,
    xy_of,
)
from spiderpig.construction.wobble import Section, column_wobble
from spiderpig.hardware.catalog import get
from spiderpig.hardware.fastener_catalog import BOLT_LENGTHS
from spiderpig.hardware.fasteners import CLEARANCE, shcs
from spiderpig.shapes import Cut, disc, ring, union
from spiderpig.stack import Unbuildable

SHANK = 0.97      # modelled shank over the nominal diameter (ISO 965 6g major: 2.874-2.98)
SHANK_MID = 0.976  # the mid major diameter (2.927 mm) over nominal: the thread rides the hole
MINOR_D = 2.387    # M3 minor diameter (ISO 724 d3): the bending core


@dataclass(frozen=True)
class BoltAxle:
    """M3 SHCS axle, laser-cut spacer rings, flat washer and nylock nut."""

    key: str = "bolt"
    label: str = "M3 socket head cap screw as the axle, laser-cut spacer rings, nylock nut"
    running_fit: float = 0.2       # link hole over M3 (3.2 mm: ISO 273 fine)
    clearance_fit: float = 0.4     # ring and plate holes over M3 (3.4 mm: ISO 273 medium)
    min_tip: float = 0.5           # thread standing proud of the nut
    pin_nut_layers: int = 2
    pillar_nut_layers: int = 3
    nut_key: str = "m3_nylock"
    snug_play: float = 0.05        # axial play a nylock tightened "snug, still turning" leaves
    washer_key: str = "m3_washer"

    d: float = 3.0

    def head(self) -> tuple[float, float]:
        """(diameter, height) of the screw head (ISO 4762 M3)."""
        h = get(shcs("3", BOLT_LENGTHS[0])).dims
        return float(h["head_d"]), float(h["head_h"])

    def nut(self) -> tuple[float, float, float]:
        """(across flats, height, corner radius) of the nut."""
        n = get(self.nut_key).dims
        return float(n["af"]), float(n["h"]), float(n["af"]) / math.sqrt(3.0)

    def washer(self) -> tuple[float, float, float]:
        w = get(self.washer_key).dims
        return float(w["od"]), float(w["id"]), float(w["t"])

    def length(self, stack: float, room: float) -> float | None:
        """The shortest standard screw that clamps ``stack`` and ends within ``room`` past it."""
        _, nut_h, _ = self.nut()
        need = stack + self.washer()[2] + nut_h + self.min_tip
        return next((L for L in BOLT_LENGTHS if need - EPS <= L <= stack + room + EPS), None)

    def max_stack(self, pitch: float) -> float:
        """The tallest stack any stock screw clamps (mm): the longest length less the
        washer, the nut and the thread's tip. A pillar clamps both frame plates, so this
        bounds the stack the planner may search (:meth:`construction.base.Group.max_top`)."""
        _, nut_h, _ = self.nut()
        return max(BOLT_LENGTHS) - self.washer()[2] - nut_h - self.min_tip

    def stock_note(self) -> str:
        return f"the longest stock M{self.d:g} screw ({max(BOLT_LENGTHS):g} mm)"

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        p = ctx.params
        hole = self.d + self.running_fit
        if hole / 2 + p.min_wall > p.link_radius:
            raise ConstructionError(f"an M{self.d:g} screw's hole leaves less than {p.min_wall} "
                                    f"mm of link around it (link radius {p.link_radius})")
        ring_min = (self.d + self.clearance_fit) / 2 + p.min_wall
        if ring_min > p.spacer_d / 2:
            raise ConstructionError(f"a {p.spacer_d} mm spacer ring leaves less than {p.min_wall} "
                                    f"mm around a {self.d + self.clearance_fit:.2f} mm hole")
        head_d, head_h = self.head()
        _, nut_h, nut_r = self.nut()
        w_od, _, w_t = self.washer()
        if head_h > ctx.pitch + EPS:
            raise ConstructionError(f"a {head_h:g} mm screw head is taller than a {ctx.pitch:g} mm "
                                    "layer")
        n = self.pillar_nut_layers if pillar else self.pin_nut_layers
        if w_t + nut_h + self.min_tip > n * ctx.pitch + EPS:
            raise ConstructionError(f"a washer, a {nut_h:g} mm nut and the screw's tip don't fit "
                                    f"{n} layers of {ctx.pitch:g} mm")
        if pillar and (self.d + self.clearance_fit) / 2 + p.min_wall > p.frame_radius:
            raise ConstructionError("an M3 pillar doesn't fit the frame plate arms")
        return AxleDims(axle=self.d / 2, spacer=p.spacer_d / 2,
                        head=max(head_d / 2, nut_r, w_od / 2), neck=ring_min, fill=True)

    def ends(self, d: AxleDims, pillar: bool, anchored: tuple[bool, bool], n_layers: int,
             pitch: float, span: float | None = None
             ) -> tuple[tuple[End, ...], tuple[End, ...]]:
        """Head and nut ends (see the module docstring); unbuildable without a standard length
        (``span``: the stack at the plan's own z, else ``n_layers`` pitches)."""
        head_d, _ = self.head()
        _, nut_h, nut_r = self.nut()
        w_od, _, w_t = self.washer()
        n = self.pillar_nut_layers if pillar else self.pin_nut_layers
        stack = n_layers * pitch if span is None else span
        if self.length(stack, n * pitch) is None:
            need = stack + w_t + nut_h + self.min_tip
            raise Unbuildable(f"no standard M3 screw for its {stack:g} mm stack (needs "
                              f"{need:.1f} to {stack + n * pitch:g} mm)")
        head: End = ("head", head_d / 2)
        nut: End = ("nut", max(nut_r, w_od / 2))
        if pillar:
            return (nut,) * n, (head,)
        return (head,), (nut,) * n

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        out = Realized()
        col = Column.of(build, group)
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        pitch = build.ctx.pitch
        ring_d = self.d + self.clearance_fit
        for k in col.between:
            role, r = col.roles[k]
            if role == "neck":
                raise ConstructionError(f"{group.name}: a screw can't neck down (layer {k})")
            z0, z1 = ring_z(build, k)
            out.bodies.append(hardware(f"{stem}_ring{k}", ring(xy, 2 * r, ring_d, z0, z1), host,
                                       fab="printed", color=SLEEVE_COLOR))
        gap_washers(build, group, col, out, self.d, host, stem)
        z_lo, z_hi = build.z(col.k0)[0], build.z(col.k1)[1]
        n_nut = len(col.below) if group.pillar else len(col.above)
        length = self.length(z_hi - z_lo, n_nut * pitch)
        if length is None:
            raise ConstructionError(f"{group.name}: no standard M3 screw for a "
                                    f"{z_hi - z_lo:g} mm stack")
        head_d, head_h = self.head()
        af, nut_h, _ = self.nut()
        w_od, w_id, w_t = self.washer()
        shank = SHANK * self.d
        if group.pillar:        # head on the inner plate (or the highest link), nut below
            screw = union([disc(xy, head_d / 2, z_hi, z_hi + head_h),
                           disc(xy, shank / 2, z_hi - length, z_hi)])
            washer = bored(disc(xy, w_od / 2, z_lo - w_t, z_lo), xy, w_id, z_lo - w_t, z_lo)
            nut = bored(hex_prism(xy, af, z_lo - w_t - nut_h, z_lo - w_t), xy, self.d,
                        z_lo - w_t - nut_h, z_lo - w_t)
        else:                   # head under the lowest link, nut on the highest
            screw = union([disc(xy, head_d / 2, z_lo - head_h, z_lo),
                           disc(xy, shank / 2, z_lo, z_lo + length)])
            washer = bored(disc(xy, w_od / 2, z_hi, z_hi + w_t), xy, w_id, z_hi, z_hi + w_t)
            nut = bored(hex_prism(xy, af, z_hi + w_t, z_hi + w_t + nut_h), xy, self.d,
                        z_hi + w_t, z_hi + w_t + nut_h)
        out.bodies += [
            hardware(f"{stem}_screw", screw, host, fab="purchased", bom_key=shcs("3", length),
                     color=STEEL),
            hardware(f"{stem}_washer", washer, host, fab="purchased", bom_key=self.washer_key,
                     color=STEEL),
            hardware(f"{stem}_nut", nut, host, fab="purchased", bom_key=self.nut_key, color=STEEL),
        ]
        for m in group.axis.members:
            out.cut(m, Cut(xy, self.d + self.running_fit))
        plates = {0: FRAME_OUTER, build.top: FRAME_INNER}
        for k in col.anchors:
            out.cut(plates[k], Cut(xy, CLEARANCE["3"]))
        out.notes["wobble"] = {group.name: column_wobble(
            build, group, col, clearance=self.running_fit + self.d * (1 - SHANK_MID),
            length=pitch, play=self.snug_play,
            play_basis=f"nylock snug ({self.snug_play:g} mm assumed)",
            section=Section.rod(MINOR_D, 450.0, name="M3 A2-70 core"))}
        return out
