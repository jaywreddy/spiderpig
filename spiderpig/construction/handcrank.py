"""The hand-cranked worm drive of a decanting cradle (``config.crank == "hand"``).

A decanting cradle (:mod:`linkages.decanters`, ``docs/agentlib/DECANTER.md`` has the why) is
turned by hand, slowly, and must stay where it is let go, so its crank is driven through a
self-locking worm gear reducer instead of a servo:

* **The gearbox** (:class:`WormBox`, an NRV050 at 30:1: :data:`NRV050`, since design review
  round 2's drive) stands outboard of the drive side's *outer* frame plate, its output axis on
  O, its output face toward the plate. Its static efficiency at 30:1 is 0.44, under the 0.5
  the maker's catalogue gives for static irreversibility: a load on the cradle can't turn the
  worm. Its 14 mm input shaft points back (``input_angle_deg``) and carries the printed hand
  crank.
* **The riser** (:func:`riser_for`, design review round 3's M2): a clamped stack on the
  gearbox's machined output face. Four class 8.8 M8 studs threaded into the face
  (:data:`RISER_STUD`); a ring of the crank sheet on the face, centred on its raised spigot
  ring (:data:`RISER_RING`); a steel sleeve on each stud (:data:`RISER_SLEEVE`, 12 mm across:
  1.5 mm off the 70 mm spigot on the 85 mm PCD); the go/no-go's shims over the sleeves
  (:data:`RISER_SHIM`); the outer plate; a thin nut over it on each stud. Clamped, friction
  carries the gearbox's side load and turn, the sleeves the columns' bending, the studs
  their tension (``cradle_drive.riser_rating``; round 2's studs had carried it all in
  bending over 18.7 mm of printed setting sleeve). The shims take up the clone's unmeasured
  hub. The gearbox's stock output shaft (25 mm, its collar on the hub's end, keyed 8 x 7
  ending 5 short of its end, an M10 end tap; 53.5 past the hub's end) reaches into the
  stack: the crank block's keyed plates on its key's straight flanks.
* **The crank block** (the arm): laser-cut 0.125 in 6061-T6 plates (the crank sheet), each
  the arm's whole outline, :data:`BLOCK_PLATES` of them stacked on the shaft: the lowest
  :data:`KEYED_PLATES` keyed over the key (cut :data:`KEY_SLOT` under its width and filed
  together on it to a slide fit), the next bored round the shaft (slotted clear of the key's
  end), the *seat* on the shaft's steel end face (the riser's nuts set the end under it: the
  guide's go/no-go; round 1's shim rings went with the fixed spacers), an M10 button head
  down through it into the shaft's end tap at :data:`END_SCREW_NM` (steel and aluminium only
  in that clamp), and over
  the seat the plates that ring the end screw's head (the block's top stays where round 1 had
  it, so the rod rides where it did). An M6 button head through the whole block at the arm's
  middle (the *tie*) into a thin nut under its lowest plate makes the plates one rigid block.
* **The crankpin**: a NETRF12 standoff (12 mm steel round) standing in the block from its
  lowest plate (an M6 button head up through that plate into it) to the rod and on: the
  rod turns on it (greased) over a printed spacer on the block, an M6 button head and a
  wide washer on its end over the rod. The rod rides just over the block, so the crankpin
  carries the rod's force a few millimetres off its socket (the 2026-10-10 review round 2:
  the arm had been on the key, the rod 21 mm over it, the crankpin a cantilever that long).
* **The hand crank**: printed in PETG in two parts joined by a slip clutch (:func:`clutch`):
  its hub is keyed on the input shaft (an M3 thumb screw into a heat-set insert, down onto
  the key: it comes off by hand, :data:`HANDLE_THUMB`) and runs on as a tube out along the
  shaft to a flange and a journal; a laser-cut 6061 seat ring (:data:`SEAT_RING`) sits keyed
  on the flange's hex; the arm turns on the journal, three steel ball-nose spring plungers
  (:data:`PLUNGER`) in its boss pressed onto the ring's seats (holes, each ball on a hole's
  edge: a 30° seat), so the arm slips round the tube past the slip it is set to: 1.3 x the
  start the user measures with the heaviest bottle in the built cradle, under a ceiling
  (:data:`CLUTCH_SET`, :func:`clutch_ceiling`), checked on a printed stub
  (:func:`check_stub`); the plungers start from a felt stop (:data:`SET_BACK`). The arm's
  throw, grip and reach are the linkage's (:func:`handle_spec`): B a light-load crank, A a
  heavy-load one (MIL-STD-1472H Figure 13), its grip a printed sleeve spinning on an M8 bolt
  through the arm (a nyloc nut).

The rod sweeps across the axis O once a turn (the crank-rocker's folded toggle), so nothing
of the drive may stand at O in the rod's layer: the planner puts the rod over the block and
its end screw's head.
The drive side alone carries all of this; the other side of the cradle has no crank
(:mod:`construction.cradle`).
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, replace

import numpy as np
from build123d import Axis, Box, Cylinder, Location

from spiderpig.construction.base import (
    FRAME_OUTER,
    RIDES_HOST,
    Build,
    ConstructionError,
    Context,
    Group,
    Motion,
    Realized,
    hardware,
)
from spiderpig.construction.plates import arm_fills, arm_plate
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.hardware.cradle_catalog import NETRF12_THREAD, netrf12
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.servos.mount import to_location
from spiderpig.shapes import (
    Cut,
    Rect,
    Shape3D,
    difference,
    disc,
    intersection,
    moved,
    ring,
    union,
)
from spiderpig.stack import Claim, Disc, Layout, Pill, Placed, Unbuildable

GROUP = "crank"
STEEL = "#4a4a4a"
GEARBOX_COLOR = "#2f5d8a"       # an NRV's RAL 5010 blue
PRINT_COLOR = "#6a4fc7"

HAND_RPM = 100.0
"""The fastest a hand turns the crank in its design (input rpm): MIL-STD-1472H Figure 13's
"Rate Below 100 RPM" column, which both the light-load (B) and heavy-load (A) cranks are
sized in. The clutch's cap takes the worm's efficiency here (:meth:`WormBox.eta_at`): a
cradle blocked while the handle turns stops the worm at the speed it was turning."""
HAND_FAST_RPM = 200.0
"""A hand's fast spin (input rpm): the empty cradle cranked back to rest at about 4 N on the
grip (MIL-STD-1472H Figure 13's light-load "Rate Above 100 RPM" row). The worm is more
efficient there, so the same slip puts more on the crank: the report gives the cap there
(``cradle_drive``'s ``fast_cap_nm``, design review round 3's M1), which the rating doesn't
take (the rated cap is at :data:`HAND_RPM`)."""
RUNIN_LOSS = 0.12
"""How much lower a new worm gear's efficiency is than its catalogue's run-in figure: SEW-
EURODRIVE's run-in table, "1-start approx. 12%" (MOVIONE project planning, "Run-in phase",
download.sew-eurodrive.com/download/html/33348626/en-EN/55361646091.html, rendered
2026-10-14: "the nominal efficiency of the gear unit is reduced by the respective value",
over a run-in of "usually 48 hours"; its row is for ratios 50-280, and the effect "intensifies
with increasing gear ratio", so 30:1 single-start sits within it). A hand crank never runs a
box in that long: the cradles' figures are rated at both ends (the M-5 finding of design
review round 2)."""


def agma_mu(v: float) -> float:
    """AGMA's worm-gear friction coefficient at a sliding speed ``v`` (m/s, at the worm's mean
    diameter): 0.15 standing, 0.124 exp(-2.233 v^0.645) to 0.0508 m/s, 0.103 exp(-1.185
    v^0.45) over it (engineersedge.com, "AGMA WormGear Equations For Rating Factors", rendered
    2026-10-14; Shigley's form adds 0.012 to the last, which lowers the efficiency: left out,
    the side that raises a clutch's cap). Only its shape is used (:meth:`WormBox.eta_at`)."""
    if v <= 0.0:
        return 0.15
    if v <= 0.0508:
        return 0.124 * math.exp(-2.233 * v ** 0.645)
    return 0.103 * math.exp(-1.185 * v ** 0.45)


@dataclass(frozen=True)
class WormBox:
    """A worm gear reducer with a hollow output and a plug-in output shaft (mm, N·m, kg).

    The defaults are the NRV030 (Motovario's VSF catalogue rev 0 2017; RS's NMRV030
    datasheet 4001782; Phidgets' TRM4400_0 and TRM4004_0 drawings, read 2026-10-10): the
    output face 4 x M6 x 11 on a 65 mm PCD at 45°, a 55 mm h8 spigot, the faces 29 mm each
    side of the worm's plane (58 across), the output hub 63 long (2.5 past each face), the
    worm 30 off the output axis, the 9 j6 input shaft (3 mm key) 20 long past its shoulder
    45 from the output axis. The single output shaft (14 h6 x 102, M6 x 15 centre tap) goes
    in from the plate's side: a 20 x 2.5 collar against the hub's end, then 30 more (B1 =
    32.5 past the hub's end), keyed 5 x 5 x 20 ending 5 short of its end. Mesh data at 30:1:
    lead 4°52', static efficiency 0.39, 0.66 at 1400 rpm. :data:`NRV050` is the size the
    cradles carry a 6 L bottle on (round 2's drive)."""

    key: str = "nrv030_30"
    ratio: int = 30
    eta_static: float = 0.39        # the catalogue's static efficiency (run in, at 25 °C)
    eta_ref: tuple[float, float] = (500.0, 0.0)   # (input rpm, efficiency): its slowest
    #                                 performance row's Mn2 x n2 / Pn1 (0: not computed)
    worm_d: float = 0.0             # the worm's mean diameter (from the mesh table's module)
    lead_deg: float = 4.87          # the worm's lead angle (the mesh table's)
    rated_nm: float = 20.0          # the seller's maximum output torque
    input_fr: tuple[float, float, float] = (210.0, 86.0, 76.0)  # the input's radial load: Fr1
    #                                 max (N, mid-shaft) and a, b of Frx = Fr1 a / (b + x), x
    #                                 from its shoulder (the VSF catalogue, 2.11.2, p. 43)
    output_fr: tuple[float, float, float] = (1830.0, 65.0, 50.0)  # the output's: Fr2 max and
    #                                 a, b, x from the hub's end (2.11.3, p. 44)
    pcd: float = 65.0
    face_thread: float = 6.0
    face_depth: float = 11.0        # the face holes' thread depth (KE)
    face_d: float = 75.0            # the machined face's boss (P)
    spigot_d: float = 55.0
    spigot_h: float = 2.5           # its raised spigot ring (assumed: measure it)
    width: float = 58.0             # face to face, along the output axis
    hub_d: float = 25.0             # the output hub's end (assumed: not dimensioned)
    hub_out: float = 2.5
    shaft_d: float = 14.0
    shaft_out: float = 32.5         # the shaft's end past the hub's end
    collar: tuple[float, float] = (20.0, 2.5)   # its collar against the hub's end: d, length
    shaft_key: tuple[float, float, float] = (5.0, 5.0, 20.0)
    key_short: float = 5.0          # the key ends this far short of the shaft's end
    key_proud: float = 2.0          # and stands this far out of it
    end_tap: tuple[float, float] = (6.0, 15.0)   # the shaft's end tap: thread, depth
    worm_offset: float = 30.0
    input_d: float = 9.0
    input_len: float = 20.0
    input_key: tuple[float, float] = (3.0, 1.4)   # its key (DIN 6885) and how deep the hand
    #                                               crank's hub's keyway is over the shaft
    input_face: float = 45.0        # output axis to the input shaft's shoulder
    cap_face: float = 51.0          # output axis to the worm's far end (the VS's shoulder)
    below: float = 40.0             # output axis to the face away from the worm
    pass_d: float = 24.0            # what turns through the outer plate's bore (2 mm wider) and
    #                                 the layer over it: the output shaft and its key, with room
    crank_hub_r: float = 12.0       # the printed hand crank's hub on its input shaft
    mass_kg: float = 1.2

    def irreversible(self) -> bool:
        """Static irreversibility, the maker's criterion: static efficiency under 0.5."""
        return self.eta_static < 0.5

    def eta_new(self) -> float:
        """The static efficiency before run-in (:data:`RUNIN_LOSS` under the catalogue's)."""
        return round(self.eta_static * (1.0 - RUNIN_LOSS), 3)

    def _eta_mesh(self, rpm: float) -> float:
        lead = math.radians(self.lead_deg)
        v = math.pi * self.worm_d * rpm / 60000.0 / math.cos(lead)
        phi = math.atan(agma_mu(v) / math.cos(math.radians(20.0)))
        return math.tan(lead) / math.tan(lead + phi)

    def eta_at(self, rpm: float) -> float:
        """The forward efficiency with the input turning at ``rpm``: the catalogue's at its
        slowest row (``eta_ref``) scaled down by AGMA's friction against sliding speed
        (:func:`agma_mu`), never under the static efficiency. The catalogues list nothing
        under 500 rpm; this is that row's efficiency carried down by the mesh's own friction
        curve, an estimate (the catalogue's 1400 rpm 0.73 is above its 0.657 at 500, as the
        curve has it)."""
        rpm_ref, eta_ref = self.eta_ref
        if not eta_ref or not self.worm_d:
            return self.eta_static
        return round(max(self.eta_static,
                         eta_ref * self._eta_mesh(rpm) / self._eta_mesh(rpm_ref)), 3)

    def input_allowed(self, x: float) -> float:
        """The input shaft's allowed radial load (N) ``x`` mm past its shoulder."""
        fr, a, b = self.input_fr
        return fr * a / (b + x)

    def output_allowed(self, x: float) -> float:
        """The output shaft's allowed radial load (N) ``x`` mm past the hub's end (the
        catalogue's "static ... limited operations" maximum, Fr2 a / (b + x))."""
        fr, a, b = self.output_fr
        return fr * a / (b + x)


NRV030 = WormBox()
NRV040 = WormBox(
    key="nrv040_30", eta_static=0.45, eta_ref=(500.0, round(62.0 * (500.0 / 30.0) * math.pi
                                                            / 30.0 / 170.0, 3)),
    worm_d=round(80.0 - 30.0 * 2.06, 2), lead_deg=6.483, rated_nm=45.0,
    input_fr=(350.0, 106.0, 95.0), output_fr=(3490.0, 84.0, 64.0), pcd=75.0, face_d=87.0,
    spigot_d=60.0, width=73.0, hub_d=30.0, shaft_d=19.0, shaft_out=43.0, collar=(24.0, 3.0),
    shaft_key=(6.0, 6.0, 30.0), key_proud=2.5, worm_offset=40.0, input_d=11.0,
    input_len=23.0, input_key=(4.0, 1.8), input_face=53.0, cap_face=60.0, below=50.0,
    pass_d=28.0, crank_hub_r=14.0, mass_kg=2.1)
"""The NRV040 at 30:1 the cradles were built on from 2026-10-13 to design review round 2's
drive (Phidgets' TRM4401_0, "Maximum Strength of Gears 45 N·m"): kept for the record, its
data as round 1 had it. Round 2 found the clutch's cap over its 45 N·m, and no clutch that
still starts a full 6 L under it (the user's call of 2026-10-14: :data:`NRV050`)."""
NRV050 = WormBox(
    key="nrv050_30", eta_static=0.44,
    eta_ref=(500.0, round(113.0 * (500.0 / 30.0) * math.pi / 30.0 / 300.0, 3)),
    worm_d=round(100.0 - 30.0 * 2.56, 2), lead_deg=6.0 + 19.0 / 60.0, rated_nm=85.0,
    input_fr=(490.0, 129.0, 114.0), output_fr=(4840.0, 101.0, 76.0), pcd=85.0,
    face_thread=8.0, face_depth=10.0, face_d=100.0, spigot_d=70.0, width=87.0, hub_d=40.0,
    shaft_d=25.0, shaft_out=53.5, collar=(32.0, 3.5), shaft_key=(8.0, 7.0, 40.0),
    key_proud=3.0, end_tap=(10.0, 16.0), worm_offset=50.0, input_d=14.0, input_len=30.0,
    input_key=(5.0, 2.3), input_face=64.0, cap_face=74.0, below=60.0, pass_d=36.0,
    crank_hub_r=17.0, mass_kg=3.45)
"""The NRV050 at 30:1 the cradles are built on since design review round 2's drive (the
user's call of 2026-10-14): Phidgets' TRM4402_0, "85Nm 50 Series Worm Gearbox 30:1",
USD 92.50 (phidgets.com/?prodid=870, its specification tab rendered 2026-10-14: "Maximum
Strength of Gears 85 N·m", "Shaft Maximum Radial Load 4.8 kN", a 14 mm input with a 5 mm
keyway, a 25 mm bore with an 8 mm keyway, backlash 1/2°, made in China, HTS 8483.40.50.10).

* Phidgets' drawing (its "Mechanical Drawings" link, productfiles/TRM4400/TRM4400_0/
  Documentation/TRM4400_0Mechanical.pdf, an NRV dimension page, fetched 2026-10-14), the 050
  column: the input D1 14 j6, B 30 long, key b1 5, t1 16 (the key 2 mm proud), an M6 end tap
  (f1); its shoulder G3 64 from the output axis, the far end G2 74; the worm I 50 off the
  output axis. Its low-speed shaft: 25 h6, B1 53.5 past the hub, G1 92 (the hub), M10 end
  tap, key 8, t1 28 (3 mm proud).
* Phidgets' single output shaft TRM4008_0 (25 mm, 8 mm keyway, USD 6.85; its drawing
  TRM4004_0Mechanical.pdf, the 050 row, fetched 2026-10-14): a 32 x 3.5 collar (D, L2), 50
  past it (L3), the keyway 40 long (E) ending 5 short of the end, 4 deep (S 21), an M10 x 16
  end tap, C45 quenched and tempered HB 220-260.
* Where Phidgets is silent, Motovario's (VSF IEC catalogue, EN rev 0 2017, my.motovario.com,
  downloaded 2026-10-14; Phidgets sells a clone of this pattern, so its figures are the
  pattern's, not measured on the clone): the face 4 x M8 x 10 (KE) on M 85 at 45° (W), the
  spigot N 70 h8, the face's boss P 100, N1 43.5 each side of the worm's plane (87 across),
  H 60 to the face away from the worm (p. 54); 3.3 kg dry (p. 68; about 3.45 with its oil);
  the mesh at 30:1, module 2.56, lead 6°19', ηd 0.73 at 1400 rpm, ηs 0.44, "only valid after
  the unit has been run in ... at 25°C" (pp. 25-27); Mn2 113 N·m at 500 rpm input with Pn1
  0.30 kW and n2 17 rpm (p. 211: efficiency 0.657 there), 88 at 1400 (p. 181), and the ATEX
  catalogue's 84 at every speed from 1750 down; Fr1 490 N (a 129, b 114) and Fr2 max 4840 N
  (a 101, b 76), "static conditions and/or ... limited operations" (pp. 43-44).
* ``rated_nm`` is Phidgets' 85 N·m, a maximum (not a nominal to scale or double: round 2's
  M-1); the clutch's cap is held under it with margin (``cradle_drive``)."""
BOXES = {b.key: b for b in (NRV030, NRV040, NRV050)}


CRANKPIN_D = 12.0       # the crankpin: a NETRF12 standoff (12 mm steel round, M6 x 12 deep each
#                         end, made to length: hardware.cradle_catalog); the 6 L's jam bends a
#                         10 mm one (SF 1.13, cradle_report)
SOCKET_FIT = 0.3        # the block's holes over it (the standoff is 0/-0.1 mm; a laser's hole
#                         +-0.127, SendCutSend's cut tolerance: seven of them must pass it)
CRANKPIN_PLAY = 0.3     # the rod's axial play on the crankpin, under its washer, at most (the
#                         standoff's length rounds down: 0.2-0.3): the washer holds it off the
#                         rear tie's screw head under the cradle link (0.26 mm nominal where it
#                         crosses, under the chain's +-0.3: the guide's paper check, filing
#                         the crankpin's top if it drags). The block hangs from its seat plate
#                         on the shaft's end and the crankpin stands on its lowest plate, so
#                         the five plates between lie in that loop, each +0.152/-0.127 mm
#                         (PLATE_UNDER): thin ones lift the rod up to 0.64 mm. A printed
#                         spacer under the rod, sanded on the bench, keeps its play (the rod
#                         only goes down)
TIE_NUT = (10.0, 3.2, "m6_thin_nut")    # the tie's thin nut under the block (DIN 439: across
#                                         flats, height, catalog key), in the head layer, the
#                                         bolt's end past it 2 mm or more over the outer plate
HEADS_OVER = 3.3 + 1.5  # over the block: the tie's M6 head (the end screw's sits lower, in
#                         the block's top plates) and 1.5 mm under the rod (the stack's
#                         tolerance: round 3 review)


PIN_WASHER = "m6_washer_wide"   # over the crankpin's end, holding the rod on
PIN_WASHER_T = 1.6
M5 = screw("bhcs", "5")         # ISO 7380 M5 button heads (the stand's)
M6 = screw("bhcs", "6")         # ISO 7380 M6 button heads: the crankpin's, the tie
SCREW_NM = 4.0                  # the drive's M6 A2-70 ISO 7380 button heads into steel (the tie,
#                                 the crankpin): TR Fastenings' maximum for ISO 7380 in A2-70,
#                                 4 N·m at M6 (its socket screws' torque table, rendered
#                                 2026-10-20: ISO 7380-1's reduced loadability; design review
#                                 round 3's m1: 5 N·m had been over it)
HEAD_LAYER_T = 7.0              # the layer over the outer plate: the riser's thin nuts and studs'
#                                 ends, the crankpin's screw head and the tie's nut under the
#                                 block, with room to run
HEAD_LAYER = 1
ARM_LAYER = 2
BLOCK_PLATES = 8                # the crank block's plates: its top where round 1 had it, so the
#                                 rod rides where it did (the planner's plan unchanged)
KEYED_PLATES = 3                # the lowest plates, on the key's straight flanks: three crank-
#                                 sheet plates, filed together, hold 4.2 / 4.0 x the walking
#                                 peak and 2.37 / 2.26 x the clutch's cap (B / A, on the sheet's
#                                 thinnest: cradle_report's keyway rows); the fourth would hold
#                                 the seat over the end screw's M10 head. The plates over them
#                                 up to the seat are slotted clear of the key (KEY_CLEAR)
KEY_SLOT = -0.25                # the keyed plates' slot under the key's width: cut at 7.75 mm (a
#                                 laser's +-0.13 stays under the key's 8) and filed together on
#                                 the key to a slide fit, so every one bears (round 2's m4: a
#                                 slot cut at the key's width that comes out wide can't be made
#                                 to bear)
KEY_CLEAR = 0.4                 # the other plates round the shaft: their slot clear of the key's
#                                 end (only the keyed plates drive)
PLATE_UNDER = 0.127             # a 0.125 in 6061 plate's thinnest under its nominal: SendCutSend's
#                                 table, +0.006 / -0.005 in (its spec table, fetched 2026-10-15);
#                                 the keyway's bearing is rated on it
ARM_BORE_FIT = 0.25             # the arm plates' bore over the h6 shaft: over a laser's
#                                 +-0.13 mm (the key drives; the seat plate's end screw and the
#                                 shaft's end hold the block along the axis)
RISER_THIN = (13.0, 4.0, "m8_thin_nut")
"""The riser's thin nut over the outer plate on each stud (DIN 439 M8, in the head layer):
the plate, the shims, the sleeve and the ring clamped between it and the gearbox's face, a 13
mm wrench on it."""
RISER_NUT_NM = 9.0              # the thin nut's torque: under TR Fastenings' A2-70 M8 14.5 N·m
#                                 at a friction of 0.1, a thin nut's two-thirds (its stainless
#                                 table, rendered 2026-10-14); 10 until the simplification round
#                                 1 (2026-10-21): on A2-70 studs and 6061 sleeves 9 keeps the
#                                 stud 2 x walking and the sleeve's crush 1.3 x at A's cap
#                                 (cradle_drive.riser_rating)
RISER_NUT_K = (0.15, 0.2)       # the nut factor's band (threadlocker 243 wet on zinc and 18-8):
#                                 the clamp T / (K d) at its two ends (the friction rated on the
#                                 low clamp, the stud on the high)
RISER_STUD = (8.0, 45.0, "m8_stud_a270_45")
"""The riser's studs (design review round 3's M2: the 18-8 set screws had carried the
gearbox in bending, and a set screw's class is for a headless screw not loaded in tension, TR
Fastenings' note under its socket screws' torque table): M8 x 1.25 A2-70 studs (a stated
class; the simplification round 1, 2026-10-21, the materials rule: class 8.8 steel cut from a
DIN 976 rod before), each sawn 45 mm long from an M8 x 60 full-thread A2-70 hex screw, its head
off (Fastenal M82580060A20000), each
threaded into one of the face's tapped holes until it stops (threadlocker 243); its end filed
to at most :data:`STUD_PAST_MAX` over its thin nut (thread, length, catalog key)."""
STUD_PAST_MAX = 0.8             # a stud's end over its thin nut at most (the guide files it): the
#                                 crank block's lowest plate passes over the four ends (design
#                                 review round 3's m9: 1.45 mm drawn, under the stack's tolerances)
STUD_PROOF_MPA = 450.0          # A2-70's stress at 0.2 % (ISO 3506-1): the studs' (class 8.8's
#                                 580 until the simplification round 1)
RISER_RING = 0.3
"""The riser's ring (design review round 3's M2): one plate of the crank sheet (0.125 in
6061) on the gearbox's machined face, its bore this much over the face's spigot ring (it
centres on it: the gearbox's side load goes into the ring through the spigot, not through the
studs), out to the face's boss, a hole on each stud (:data:`RISER_RING_HOLE` over it); the
sleeves stand on it, so the face takes their clamp over the whole ring."""
RISER_RING_HOLE = 0.3           # its holes over the studs' thread (8.3 mm: the web to its bore
#                                 stays over a thickness, SendCutSend's least; a stud that won't
#                                 pass, the hole filed)
RISER_SLEEVE = (12.7, 8.0, 20.0, "m8_sleeve_6061_12p7x20")
"""The riser's sleeve on each stud, from the ring up to the shims under the plate (OD, bore,
length, catalog key): 6061-T6511, a 1 in piece of the posts' 1/2 in round faced to 20 mm and
drilled 8.0 mm, a snug bore on the M8 stud (the simplification round 1, 2026-10-21, the
materials rule: Harfington's 304 stainless 8 x 12 x 20 mm spacers before, whose annealed 205 MPa
the clamp took past at the clutch's cap; 6061's 241 MPa on the wider ring holds it at 1.35,
``cradle_drive.riser_rating``). 12.7 mm across keeps 1.15 mm off the 70 mm spigot ring on the
85 mm PCD and 1.15 mm inside the face's 100 mm boss. Clamped, the plate, the shims, the sleeves
and the ring are one stack on the face: friction carries the gearbox's side load and its turn,
the sleeve the column's bending, the stud only its tension (round 2's printed sleeves had
carried nothing, the studs all of it in bending over 18.7 mm)."""
RISER_SHIM = (14.0, 8.0, 0.5, "shim_8x14x0p5", 20)
"""The go/no-go's shims over each sleeve, under the plate (OD, bore, thickness, catalog key,
bought): 0.5 mm 304 flat shims, the drawn stack the riser's height less the ring and the
sleeve; four DIN 125 M8 washers (1.6 mm, :data:`RISER_COARSE`) as coarse ones."""
RISER_COARSE = ("m8_washer", 4)
RISER_SHIMS_RANGE = (1.0, 3.0)  # the drawn shim stack's least and most (mm): the go/no-go moves
#                                 the shaft's end a millimetre either way with the shims bought
RISER_HEAD_R = RISER_THIN[0] / math.sqrt(3)   # its corners' radius
RISER_CLEAR = 3.0               # the riser holds the plate this far over the face's raised spigot
#                                 ring and the turning hub's end at least (neither measured)
STUD_IN = 1.0                   # a stud's least engagement in the face, in diameters
SEAT_LOW = 0.7                  # how far under the seat plate's drawn place the shaft's end may
#                                 stand (the block on it sits that much low: the head layer has
#                                 the room); the go/no-go's window, the riser's shims set it
PLATE_OVER = 0.152              # a 0.125 in 6061 plate's thickest over its nominal (SendCutSend's
#                                 table, +0.006 in): the plates under the seat lower the block
END_SCREW_NM = 15.0             # the shaft's end screw: an M10 A2-70 ISO 7380 button head through
#                                 the seat plate into the shaft's M10 end tap, threadlocker
#                                 (243): TR Fastenings' maximum for ISO 7380 in A2-70 at M10
#                                 (its socket screws' torque table, rendered 2026-10-20; design
#                                 review round 3's m1: 20 N·m had been over it); it only holds
#                                 the block on the shaft's end
LIGHT_LOAD_N = 22.0
"""The grip force bar (N): MIL-STD-1472H's crank (§5.1.4.1.2.4.7, Figure 13 "Cranks", p. 76:
"Light Loads <22 N (5 lb); Wrist & Finger Movement", a 38 to 125 mm turning radius below 100
rpm, a grip 25 to 75 mm long and 10 to 16 mm across; "Heavy Loads >=22 N (5 lb); Arm
Movement", 190 to 510 mm, a grip 75 mm long or more and 25 to 38 mm across). A pour is about
15 handle turns (30:1 over half a crank turn), the many-turn task the standard puts cranks to
(§5.1.4.1.2.4.1.1), turned well under 100 rpm. A sealed full 6 L's worst starting turn stays
under it on either crank (``tests/test_cradle.py``)."""


@dataclass(frozen=True)
class HandleSpec:
    """A hand crank's arm: its throw (mm), its grip (d, length), how far its arm's outer face
    stands past the input shaft's end (``reach``, the tube), its grip bolt (d, length, thread,
    head height, catalog key) and MIL-STD-1472H Figure 13's row it is sized in (``load``)."""

    throw: float
    grip: tuple[float, float]
    reach: float
    bolt: tuple[float, float, float, float, str]
    load: str


HANDLES: dict[str, HandleSpec] = {
    "": HandleSpec(125.0, (16.0, 60.0), 100.0, (8.0, 90.0, 22.0, 5.3, "m8_hex_bolt_90"),
                   "light"),
    "decanter_hinge": HandleSpec(130.0, (16.0, 60.0), 230.0,
                                 (8.0, 90.0, 22.0, 5.3, "m8_hex_bolt_90"), "light"),
}
"""Per linkage, the hand crank's arm (``""``: the default, B's). B's is MIL-STD-1472H's
longest light-load crank, 125 mm, a 60 x 16 grip (a full 6 L's worst start 21.7 N run in).
A's (design review round 2's M-3) is 130 mm on a 230 mm reach: the reach puts its arm and a
hand on its grip behind A's cradle link b2 and its rear tie Tr2 over the whole turn (round
1's 85 mm crank on a 100 mm reach had the hand through them for 64.5° of crank) and in
front of the base stop's rails behind the cradle (``cradle_report.hand_sweep``). A full 6
L's worst start is 21.5 N at 130 mm run in, under the 22 N bar (21.97, at it, from design
review round 2's 12 mm steel tie rods to the simplification round 1's 6061 ones); at Figure
13's light-load maximum, 125 mm, it is 22.8 N, and a heavy-load crank (190 mm or more) meets
A's bottle and the base stop's rails at every reach to 300 mm: so 5 mm over the light row's radius
(:data:`HANDLE_OVER_RADIUS`). Its tube is long, so its hub prints in two pieces
(:data:`HUB_PRINT_MAX`)."""
HANDLE_OVER_RADIUS: dict[str, float] = {"decanter_hinge": 5.0}
"""Per linkage, how far its light-load crank's throw is over MIL-STD-1472H Figure 13's 125 mm
(A's: :data:`HANDLES`), which the tests allow it."""
HUB_PRINT_MAX = 200.0           # a hand crank's hub and tube longer than this print in two
#                                 pieces (a 210 mm tall bed): a hex spigot into a socket, an M3
#                                 button head through the socket's wall into an insert
HUB_SPLIT = (110.0, 22.0, 15.0) # the hub piece's length to the joint, the spigot's across
#                                 flats and length


def handle_spec(config) -> HandleSpec:
    """The hand crank's arm for ``config`` (its linkage's :data:`HANDLES` entry, else the
    default; a config-less context: the default; a linkage's key: its own)."""
    key = config if isinstance(config, str) else getattr(config, "linkage", "")
    return HANDLES.get(key, HANDLES[""])


def handle_r(config) -> float:
    """The hand crank's throw for ``config`` (a config or a linkage's key:
    :func:`handle_spec`)."""
    return handle_spec(config).throw


HANDLE_R = HANDLES[""].throw
ARM_T, ARM_W = 14.0, 18.0       # the printed crank arm's bar: along the shaft, across (round 2's
#                                 m8: 10 deep put an out-of-plane 100 N at the grip at about 42
#                                 MPa in it, printed strength; 14 at 21 on B's, 34 on A's)
HUB_LEN = 28.0                  # the printed crank's hub on the input shaft (30 long), printed
#                                 solid; its radius the box's crank_hub_r
HUB_BORE = 14.1                 # its bore, printed and reamed with a 14 mm drill
HANDLE_THUMB = (9.0, 8.0, 5.0)  # the knurled-head M3 thumb screw down onto that key (McMaster
#                                 92545A115, 18-8): under its head, its head d and h; through a
#                                 heat-set insert in the hub to the key, its head 2 mm off the hub
INSERT = (4.0, 5.7, "m3_heat_set_insert")   # an M3 heat-set insert: its hole's d, its length
#                                             and catalog key (the hub's, the journal's end's)
TOGGLE_TURNS = 2.5
"""The handle turns from either end of the swing within which some part of a cradle holds
the clutch's cap at a jam SF under 1.3 (``cradle_report``'s ``jam_capped``,
``holds_1p3_turns``, which a test holds under this): the toggle zones the guide marks on the
gearbox's far face and the use limits keep everything off the cradle in (design review round
2's M-4)."""
CLUTCH_SET = 1.3
"""The slip clutch is set to slip at this times the start the user measures (the user's call
of 2026-10-14, design review round 2's B-1): the heaviest bottle they have, at its hardest
start, a luggage scale on the grip; the slip at the grip then 1.3 x that reading, but never
over the cradle's ceiling (:func:`clutch_ceiling`: design review round 3's M1). It depends on
neither efficiency guess (the scale reads the worm as it is)."""
CLUTCH_CHECK = 0.08
"""The check's window: three pulls in a row within this of the target (a luggage scale's
tenth of a kilogram is 3.5 % of B's 28 N), none over the ceiling. The check (on its printed
stub in the riser locator's pocket: no vise) is the last build step, part of every set-up and
follows any greasing, a hot car, and a month or ten pours standing set up (the guide): between
checks the cap holds at the ceiling. Unchecked, the friction can carry the slip far further
(:func:`clutch_drift`)."""
SET_BACK = 0.5
"""The plungers' starting setting (design review round 3's B1: "flush with its jam nut" had
pushed each 1.98 mm, its nose onto the seat ring with its ball still in its hole: no clutch):
with the arm at a click, each plunger turned in until it stops (its nose on the ring, wherever
its ball and its nuts are), then out this many turns. Drawn so."""
NEAR_STOP = 0.125
"""The nearest a plunger is ever set to that stop (turns): its ball still climbs out of its
hole with the code's 0.2 mm to spare (:func:`plunger_setting`)."""
PLUNGER = {"thread": 12.7, "length": 26.67, "ball_d": 7.14, "travel": 1.83,
           "force": (26.7, 53.4), "key": "spring_plunger_half13", "pitch": 25.4 / 13.0}
"""The clutch's ball-nose spring plungers (McMaster 5901A17, the 1/2"-13 listing rendered
2026-10-14): a 440C stainless 0.281 in (7.14 mm) ball standing 0.072 in (1.83 mm) out of a
black-oxide steel body 1.05 in long, hex drive at the back, 6 lbf extended to 12 lbf pushed in
(26.7 to 53.4 N, 14.6 N/mm). Round 2's reviewer named 3408A81 (3/8"-16, a 4.8 mm ball, 6 to
21 lbf): a 30° seat on a 4.8 mm ball is a 2.4 mm hole, under SendCutSend's least hole in
0.125 in 6061 (its thickness), and its 55 N/mm spring makes an eighth of a turn 20 % of the
slip; the 7.14 mm ball's 30° seat is a 3.57 mm hole, and an eighth of a turn of one of three
plungers moves the slip about 3 %."""
CLUTCH = {"plungers": 3, "seats": 6, "r": 40.0, "seat_d": 3.57, "ring_t": 3.175,
          "hex_af": 30.0, "flange_t": 6.0, "boss_t": 17.5, "journal_r": 8.0, "gap": 0.6}
"""The clutch's geometry (mm): the plungers on a circle ``r`` round the axis in the arm's boss
(``boss_t`` long on the hub's journal, ``journal_r``), their balls on ``seats`` holes
``seat_d`` across (a 30° seat: the ball on the hole's edge) in the seat ring (:data:`SEAT_RING`,
``ring_t``), keyed on the flange's hex (``hex_af``), ``gap`` under the boss. ``r`` puts the
plunger's force for every set-up (B or A, run in or new, the greased friction's either end)
inside its 26.7 N and its 53.4 N less the climb."""
SEAT_RING = "clutch seat ring"  # its part's label: cut from the crank sheet (6061)
DRIVE_LOCK = "threadlocker_243"  # the drive's threadlocker (the guide's steps, the BOM's line)
PETG_PARTS = ("handle", "handle_tube", "handle_arm", "handle_cap", "handle_grip")
"""The hand crank's prints in PETG (``bom.part_filament``): the plungers push on the hub, the
arm and the cap always, and PLA creeps out of a load in a hot car (round 2's m7)."""
THRUST = (17.0, 30.0, 3.0, "m16_washer")
"""The steel washer over the journal between the arm's boss and the journal's cap (DIN 125
M16: bore, OD, thickness, catalog key), greased: the plungers' push on the cap runs on it, its
friction a few per cent of the slip (:func:`clutch_slip`)."""
JAM_NUT = (19.05, 7.94, "half13_jam_nut")   # each plunger's jam nut over the boss (AF, h, key)
NUT_HALF = (19.05, 11.11, "half13_nut")     # its hex nut pressed into the boss (AF, h, key)
NUT_POCKET = 0.15               # that nut's hex pocket over its across-flats: pressed in from the
#                                 boss's inner face, its floor toward the boss's outer face takes
#                                 the spring's push and the jam nut's clamp (design review round
#                                 3's m2: 0.3 mm over, open toward the push, it had held by
#                                 friction alone)
GREASED_MU = (0.09, 0.19)
"""The clutch's contacts' friction, greased (both the ball on the 6061 seat's edge and the
ball in its steel body): RoyMech's "Steel (Mild) / Steel (Mild)" greasy sliding 0.09-0.19
(roymech.org, Useful_Tables/Tribology/co_of_frict, rendered 2026-10-14; it and Engineering
ToolBox list no greasy steel-on-aluminium figure, so the steel pair's stands in; ToolBox's
greasy static steel 0.16 is inside it). The seats and the thrust washer are greased at
assembly and after the yearly check."""
DRY_MU = ((0.47, 0.61), (0.42, 0.78))
"""Dry: the ball on 6061 (RoyMech's "Aluminum / Mild Steel" 0.47 sliding, 0.61 static) and in
its steel body (0.42 sliding; static 0.74 mild, 0.78 hard): where the grease has gone."""
SEAT_WEAR = 0.1                 # the seat holes' edge rounded this far (mm) before the yearly
#                                 look replaces the ring (the guide: any bright land you can
#                                 see round a hole's edge; design review round 3's m4: "wider
#                                 than a millimetre" had been ten times this). The balls ride
#                                 the ring's laser entry face, its dross face on the flange
JOURNAL_MU = (0.1, 0.3)         # the arm's PETG boss on the hub's PETG journal: greased (the
#                                 guide), and dry: the grip's couple on it (journal_share)
GRIP_NUT = (13.0, 8.0, "m8_nyloc_nut")      # the grip bolt's nyloc on the arm's inner face
GRIP_WASHER = "m8_washer"                   # a DIN 125 washer each side of the grip and the arm


def clutch_slip(f_seat: float, mu1: float, mu2: float, wear: float = 0.0,
                mu_thrust: float | None = None) -> float:
    """The clutch's slip torque (N·m at the input) with each plunger pushing ``f_seat`` N on
    its ball seated (as set, the seat unworn): the most the arm must exert over the balls'
    climb out of their seats (each ball pivots on its hole's edge, its contact angle from the
    plunger's axis falling from 30° to 0 as it climbs, the spring pushing harder as it does)
    with friction ``mu1`` at the edge and ``mu2`` between the ball and its body's bore (the
    force on the ball P = F k / (1 - mu2 k), k = tan(angle + atan mu1); design review round
    2's B-1: the contact on the seat's wall, not its top), plus the plungers' push on the
    journal's cap turning on its thrust washer (``mu_thrust``, else ``mu1``). ``wear``: the
    holes' edge rounded that far (mm), the ball seated deeper and steeper, its spring the
    sink less pressed. ``inf``: it locks (mu2 k >= 1)."""
    n, r_c = CLUTCH["plungers"], CLUTCH["r"]
    big_r = PLUNGER["ball_d"] / 2
    rate = (PLUNGER["force"][1] - PLUNGER["force"][0]) / PLUNGER["travel"]
    a0 = CLUTCH["seat_d"] / 2
    a = a0 + wear
    h_set = math.sqrt(big_r ** 2 - a0 ** 2)
    s = np.linspace(0.0, a, 801)
    h = np.sqrt(big_r ** 2 - (a - s) ** 2)
    k = np.tan(np.arcsin((a - s) / big_r) + math.atan(mu1))
    den = 1.0 - mu2 * k
    if (den <= 1e-9).any():
        return math.inf
    force = f_seat + rate * (h - h_set)
    p = float((force * k / den).max())
    t_bore, t_od = THRUST[0] / 2, THRUST[1] / 2
    r_t = 2.0 / 3.0 * (t_od ** 3 - t_bore ** 3) / (t_od ** 2 - t_bore ** 2)
    mu3 = mu1 if mu_thrust is None else mu_thrust
    return n * (r_c * p + mu3 * f_seat * r_t) / 1000.0


def clutch_drift(wear: float = SEAT_WEAR) -> dict:
    """How far the clutch's slip moves from its setting with nothing touched (each a ratio to
    the slip as set): set greased with its friction anywhere in :data:`GREASED_MU`, then the
    friction anywhere in it again (``greased``), and with the seats worn ``wear`` too
    (``greased_worn``); dry (:data:`DRY_MU`): the least friction pair at which it locks, the
    plungers' own bores at their lowest (``locks_dry``: a seat friction from which the
    ball can't climb at all)."""
    f = 35.0
    mus = np.linspace(*GREASED_MU, 6)
    fresh = [clutch_slip(f, m1, m2, 0.0, m3) for m1 in mus for m2 in mus for m3 in mus]
    worn = [clutch_slip(f, m1, m2, wear, m3) for m1 in mus for m2 in mus for m3 in mus]
    lo, hi = min(fresh), max(fresh)
    seat_dry, bore_dry = DRY_MU
    dry = clutch_slip(f, seat_dry[0], bore_dry[0])
    lock = next((round(m, 2) for m in np.arange(0.05, 1.2, 0.01)
                 if math.isinf(clutch_slip(f, m, m))), None)
    return {"greased": [round(float(lo / hi), 3), round(float(hi / lo), 3)],
            "greased_worn": [round(float(min(lo, *worn) / hi), 3),
                             round(float(max(worn) / lo), 3)],
            "wear_mm": wear, "dry_x_greased_mean": round(float(dry / ((lo + hi) / 2)), 2)
            if math.isfinite(dry) else None,
            "locks_at_mu": None if lock is None else float(lock),
            "locks_dry": math.isinf(clutch_slip(f, seat_dry[1], bore_dry[1]))}


def plunger_force(slip_nm: float, mu: float) -> float:
    """Each plunger's seated force (N) for a slip of ``slip_nm`` at the input, its contacts'
    friction ``mu``: :func:`clutch_slip` solved for it (it rises with the force)."""
    lo, hi = 0.0, 500.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if clutch_slip(mid, mu, mu) < slip_nm else (lo, mid)
    return (lo + hi) / 2


def plunger_setting() -> dict:
    """The plungers' settings (mm of push, each ball's spring pressed in from its free length,
    and N): the felt stop (the nose on the ring, the ball seated: push ``travel - climb``,
    where the ball can't climb out of its hole at all: the lock), the start the guide sets
    (:data:`SET_BACK` turns out from it, drawn), the nearest the guide allows
    (:data:`NEAR_STOP` turns out), each one's force, and the room left to climb at each
    (``travel - push - climb``: the code's least 0.2 mm). Whatever the nuts' heights: the stop
    is felt, not measured."""
    big_r, a = PLUNGER["ball_d"] / 2, CLUTCH["seat_d"] / 2
    climb = big_r - math.sqrt(big_r ** 2 - a ** 2)
    travel, pitch = PLUNGER["travel"], PLUNGER["pitch"]
    lo, hi = PLUNGER["force"]
    rate = (hi - lo) / travel
    stop = travel - climb
    out = {"climb_mm": round(climb, 3), "pitch_mm": round(pitch, 3),
           "stop_push_mm": round(stop, 3), "rate_n_mm": round(rate, 2)}
    for tag, turns in (("start", SET_BACK), ("limit", NEAR_STOP)):
        push = stop - turns * pitch
        out[f"{tag}_push_mm"] = round(push, 3)
        out[f"{tag}_n"] = round(lo + rate * push, 1)
        out[f"{tag}_room_mm"] = round(travel - push - climb, 3)
    return out


def plunger_reach(slip_nm: float) -> dict:
    """Whether a plunger set between the guide's start and its limit (:func:`plunger_setting`)
    reaches ``slip_nm`` (N·m at the input) at the greased friction's either end with its
    spring 10 % weak, nominal or 10 % strong (no maker states a ball plunger's force
    tolerance: design review round 3's m3): the force needed and the force at the limit, per
    case; ``under``: the cases it can't reach (the slip then stays under the target, the safe
    side: the guide says to leave it there if it still clears 1.1 x the start)."""
    s = plunger_setting()
    need = {f"{mu:g}": round(plunger_force(slip_nm, mu), 1) for mu in GREASED_MU}
    at = {f"{k:g}": round(k * s["limit_n"], 1) for k in (0.9, 1.0, 1.1)}
    under = [[m, k] for m, f in need.items() for k, g in at.items() if f > g + 1e-9]
    return {"need_n": need, "limit_n": at, "under": under}


def clutch_ceiling(linkage: str) -> float:
    """Cradle ``linkage``'s ceiling (N·m at the input): the most any of the check's pulls may
    read, whatever the start measured: the drawn 6 L's window top (:data:`CLUTCH_SET` x
    (1 + :data:`CLUTCH_CHECK`) x its run-in start, :data:`DESIGN_START_NM`). A new box's start
    reads up to :data:`RUNIN_LOSS` higher and its user can't know it; set on it, 1.3 x had let
    the box run in under a slip 14 % over the rated (design review round 3's M1): under the
    ceiling the cap holds whatever the box's run-in state or the reading."""
    return CLUTCH_SET * (1 + CLUTCH_CHECK) * DESIGN_START_NM[linkage]


def clutch(start_nm: float, grip_r: float, box: WormBox,
           ceiling_nm: float | None = None) -> dict:
    """The hand crank's slip clutch for a cradle whose worst starting turn takes
    ``start_nm`` N·m at the input (a sealed full 6 L at the catalogue's static efficiency; the
    user measures their own): set to :data:`CLUTCH_SET` x it, but its window's top never over
    ``ceiling_nm`` (:func:`clutch_ceiling`; ``None``: no ceiling), held within
    :data:`CLUTCH_CHECK` by the check; the grip's forces at ``grip_r`` mm; the most it puts on
    the crank (``crank_cap_nm``, ``config.PRODUCT_CLUTCH_NM``): the check's top through the
    worm at the hand's fastest (:data:`HAND_RPM`, :meth:`WormBox.eta_at`), and at a fast spin
    (``fast_cap_nm``, :data:`HAND_FAST_RPM`). The plungers' force where it slips, set, for the
    greased friction's either end (``plunger_n``, within :data:`PLUNGER`'s)."""
    set_nm = CLUTCH_SET * start_nm
    if ceiling_nm is not None:
        set_nm = min(set_nm, ceiling_nm / (1 + CLUTCH_CHECK))
    lo, hi = set_nm * (1 - CLUTCH_CHECK), set_nm * (1 + CLUTCH_CHECK)
    eta = box.eta_at(HAND_RPM)
    cap = hi * box.ratio * eta
    # each plunger's seated force for the set slip at the friction's two ends
    f_at = [round(plunger_force(set_nm, mu), 1) for mu in GREASED_MU]
    return {"set_nm": round(set_nm, 3), "lo_nm": round(lo, 3), "hi_nm": round(hi, 3),
            "set_grip_n": round(set_nm * 1000.0 / grip_r, 1),
            "start_grip_n": round(start_nm * 1000.0 / grip_r, 1),
            "hi_grip_n": round(hi * 1000.0 / grip_r, 1),
            "margin": round(lo / max(start_nm, 1e-9), 2),
            "eta_hand": eta, "crank_cap_nm": round(cap, 1),
            "fast_cap_nm": round(hi * box.ratio * box.eta_at(HAND_FAST_RPM), 1),
            "plunger_n": sorted(f_at)}


def journal_share(spec: HandleSpec) -> dict:
    """What the grip's offset adds to the slip (an estimate: design review round 3's m6): the
    hand's pull at the grip, ``e`` mm out along the axis from the middle of the arm's boss,
    tilts the arm on the hub's journal (:data:`CLUTCH`'s ``boss_t`` long, ``journal_r``); the
    journal's two ends take the couple, ``2 F e / L`` together, its friction (:data:`JOURNAL_MU`)
    at the journal's radius resists the turn. As a fraction of the slip at the grip's middle
    (where the check pulls), its inner end and its outer end, greased and dry; and the outer
    end over the middle (what a hand there adds over the checked slip)."""
    cl = CLUTCH
    grip_len = spec.grip[1]
    boss_mid = spec.reach - cl["boss_t"] / 2
    w_t = 1.6                                       # (the grip's washer: _handle's)
    mid = spec.reach + w_t + 0.2 + grip_len / 2 - boss_mid
    out = {"e_mid_mm": round(mid, 1)}
    for mu in JOURNAL_MU:
        k = 2 * mu * cl["journal_r"] / (cl["boss_t"] * spec.throw)
        at = {p: k * max(e, cl["boss_t"] / 2) for p, e in
              (("inner", mid - grip_len / 2), ("mid", mid), ("outer", mid + grip_len / 2))}
        out[f"{mu:g}"] = {p: round(v, 3) for p, v in at.items()}
        out[f"{mu:g}"]["outer_over_mid"] = round((1 + at["outer"]) / (1 + at["mid"]), 3)
    return out


DESIGN_START_NM: dict[str, float] = {"decanter_rcm": 2.66, "decanter_hinge": 2.794}
"""Per cradle, a sealed full 6 L's worst starting turn at the input (N·m) at the catalogue's
static efficiency (:data:`NRV050`): ``cradle_report``'s ``handle_torque_nm``, which a test
holds these to. The clutch is set on the user's own measurement; these draw it (the guide's
expected reading, ``config.PRODUCT_CLUTCH_NM``)."""


def drawn_clutch(linkage: str) -> dict:
    """:func:`clutch` for cradle ``linkage`` as drawn (its 6 L's start, its crank, the box,
    under its ceiling)."""
    spec = HANDLES.get(linkage, HANDLES[""])
    return clutch(DESIGN_START_NM[linkage], spec.throw, NRV050, clutch_ceiling(linkage))


CHECK_STUB = (100.0, 30.0, 10.0, 32.0)
"""The clutch's check stub (design review round 3's stand and use m12: the check had needed a
bench vise): its bar's length, width and thickness, its post's height (mm)."""


def check_stub(box: WormBox = NRV050) -> Shape3D:
    """The clutch's check stub, printed in PETG (a tool: ``build`` writes its STL and the
    BOM lists it): a bar that drops into the riser locator's 101 mm square pocket (100 x 30
    mm: it turns under 2° before its corners meet the pocket's walls), a post standing up
    from it the size of the gearbox's input shaft with its key printed on it, for the hand
    crank's hub and thumb screw. The check pulls the grip with the hub held: at the ceiling's
    4 N·m the bar's ends push about 40 N on the walls (about 4 MPa in the bar at the post,
    10 MPa of bearing on the key), the board held down by the cradle on it."""
    length, width, thick, tall = CHECK_STUB
    key_w, key_hub = box.input_key
    r = box.input_d / 2 - 0.05                      # (the hub's bore is reamed 14.1)
    top = box.input_d / 2 + key_hub - 0.2           # the key under the hub's keyway's top
    bar = Box(length, width, thick).moved(Location((0.0, 0.0, thick / 2)))
    post = Cylinder(r, tall).moved(Location((0.0, 0.0, thick + tall / 2)))
    ridge = Box(key_w - 0.2, top, tall).moved(Location((0.0, top / 2, thick + tall / 2)))
    return union([bar, post, ridge])


def _turns(turns: float) -> str:
    """A part of a plunger's turn in words (the guide's)."""
    return {0.5: "half a turn", 0.25: "a quarter of a turn",
            0.125: "an eighth of a turn"}.get(turns, f"{turns:g} of a turn")


def _drawn_start(view, spec: HandleSpec) -> float:
    """The guide's expected start reading (N at the grip) for a full 6 L on ``view``'s
    cradle (0: not a cradle we draw)."""
    key = getattr(getattr(view.ctx, "config", None), "linkage", "")
    start = DESIGN_START_NM.get(key)
    return round(start * 1000.0 / spec.throw) if start else 0.0


@dataclass(frozen=True)
class Riser:
    """The riser: how far the gearbox's machined output face stands under the outer plate's
    outer face (``height``) and where it and its output shaft then are (side z, the drawn
    housing's): the hub's end, the key's span and the shaft's end; the stack on the face (the
    ring ``ring_t`` thick, the sleeve, the drawn shims ``shims`` thick, the plate); the studs'
    cut length, how far each goes into the face, how far its end stands over its thin nut
    once filed (``past``) and its drawn length; the crank block's seat plate's underside (its
    index ``seat_k``) on that end, and how far under it the end may stand (``low``: the keyed
    plates still on the key's straight flanks, the block that much low)."""

    height: float
    face: float
    hub_end: float
    key: tuple[float, float]
    shaft_end: float
    stud: float
    engage: float
    past: float
    ring_t: float
    shims: float
    seat: float
    seat_k: int
    low: float
    over: float = 0.0           # the plate's outer face to its thin nut's top

    @property
    def stud_drawn(self) -> float:
        """The stud's drawn length: in the face, up through the stack and its thin nut, and
        its end :attr:`past` over the nut (filed so)."""
        return self.engage + self.height + self.over + self.past


def end_screw(box: WormBox):
    """The shaft's end screw's family: an ISO 7380 button head on its end tap."""
    return screw("bhcs", f"{box.end_tap[0]:g}")


def riser_for(z0: float, a0: float, t: float, box: WormBox = NRV050,
              t_plate: float = 4.75, t_ring: float = 3.175) -> Riser:
    """The riser that puts the shaft's end under the crank block's seat plate (the block's
    plates ``t`` thick from ``a0``, side z; ``z0`` the outer plate's outer face, ``t_plate``
    its thickness; ``t_ring`` the ring's, the crank sheet): the seat the highest plate whose
    end screw's head stands no higher over the block's top than the tie's M6 head, with the
    key's straight flanks (its keyseat is end-milled: its ends are round, half its width
    each) still over the keyed plates' whole height with the shaft's end :data:`SEAT_LOW`
    under it; the shaft's end drawn at the seat; the face under it by the shaft's and the
    hub's lengths; on the face the ring, the sleeve and the shims up to the plate (the
    shims set it: a hub prouder or lower than drawn is shimmed out, :data:`RISER_SHIMS_RANGE`
    either way); each stud threaded :data:`STUD_IN` diameters or more into the face, short of
    its tapped depth, up through the stack and its thin nut, its end filed to at most
    :data:`STUD_PAST_MAX` over the nut, so the crank block's lowest plate clears it with the
    go/no-go's window and the plates under the seat at their thickest (design review round
    3's m9). :class:`ConstructionError` if any of it doesn't hold."""
    key_w, _, key_len = box.shaft_key
    a1 = a0 + KEYED_PLATES * t
    lo = a1 - 0.2 + box.key_short + key_w / 2
    top = a0 + BLOCK_PLATES * t
    head = end_screw(box).head_h
    seat_k = next((k for k in range(BLOCK_PLATES - 1, KEYED_PLATES, -1)
                   if a0 + (k + 1) * t + head <= top + M6.head_h + 1e-9
                   and a0 + k * t >= lo + SEAT_LOW - 1e-9), None)
    if seat_k is None:
        raise ConstructionError(
            f"no plate of the {BLOCK_PLATES}-plate crank block seats on the shaft's end with "
            f"its M{box.end_tap[0]:g} end screw's head under the block's top and the key's "
            f"straight flanks over the {KEYED_PLATES} keyed plates (from z {lo:.2f})")
    seat = a0 + seat_k * t
    end = seat
    hub = end - box.shaft_out
    face = hub - box.hub_out
    height = z0 - face
    if height - max(box.spigot_h, box.hub_out) < RISER_CLEAR:
        raise ConstructionError(f"the gearbox's face {height:.1f} mm under the plate leaves "
                                f"under {RISER_CLEAR:g} mm over its spigot ring")
    shims = height - t_ring - RISER_SLEEVE[2]
    if not RISER_SHIMS_RANGE[0] - 1e-9 <= shims <= RISER_SHIMS_RANGE[1] + 1e-9:
        raise ConstructionError(f"the riser's {height:.2f} mm leaves {shims:.2f} mm of shims "
                                f"over its ring and {RISER_SLEEVE[2]:g} mm sleeve, not "
                                f"{RISER_SHIMS_RANGE[0]:g}-{RISER_SHIMS_RANGE[1]:g}")
    d, length, _ = RISER_STUD
    thin = RISER_THIN[1]
    over = t_plate + thin                               # the plate's outer face to the nut's top
    engage = box.face_depth - 0.5
    spare = length - engage - height - over             # its end over the nut, as cut
    if engage < STUD_IN * d - 1e-9 or spare < -1e-9:
        raise ConstructionError(f"a {length:g} mm M{d:g} stud {engage:g} mm into the face "
                                f"doesn't reach its thin nut's top over the {height:.1f} mm "
                                "riser and the plate")
    past = min(STUD_PAST_MAX, spare)
    room = (HEAD_LAYER_T - thin - STUD_PAST_MAX - SEAT_LOW - seat_k * PLATE_OVER)
    if room < 0.5 - 1e-9:
        raise ConstructionError(f"the crank block's lowest plate passes {room:.2f} mm over the "
                                "riser's studs' ends at the stack's worst, under 0.5")
    return Riser(round(height, 3), face, hub, (end - box.key_short - key_len,
                                               end - box.key_short), end, float(length),
                 round(engage, 2), round(past, 3), t_ring, round(shims, 3), seat, seat_k,
                 round(min(SEAT_LOW, seat - lo), 3), over)


def arm_z(z0: float, t_frame: float, t_arm: float) -> tuple[float, float]:
    """The keyed plates' z (side): on the layer over the riser's nuts (fixed by the frame
    plate and the head layer alone, so the search's and the plan's layouts agree)."""
    a0 = z0 + t_frame + HEAD_LAYER_T
    return a0, a0 + t_arm


class HandCrank(Group):
    """The decanting cradle's crank: the worm gearbox on its riser's studs, the crank block
    keyed on its shaft and seated on its end, the crankpin and the hand crank with its slip
    clutch (this module's docstring)."""

    name = GROUP
    stack_only = True           # the contract checks only what is inside the stack

    def __init__(self, box: WormBox = NRV050, input_angle_deg: float = 0.0):
        self.box = box
        self.input_angle_deg = input_angle_deg    # the input shaft's world direction

    # -- geometry ---------------------------------------------------------------------

    @staticmethod
    def pin(ctx: Context) -> str:
        pins = ctx.topo.axes_of("crankpin")
        if len(pins) != 1:
            raise ConstructionError(f"{ctx.topo.name}: the hand crank drives one crankpin, "
                                    f"the side has {len(pins)}")
        return pins[0].name

    @staticmethod
    def rider(ctx: Context) -> str:
        pins = ctx.topo.axes_of("crankpin")
        members = pins[0].members if pins else ()
        if len(members) != 1:
            raise ConstructionError(f"{ctx.topo.name}: the hand crank's pin carries one rod, "
                                    f"not {list(members)}")
        return members[0]

    @staticmethod
    def throw(ctx: Context) -> float:
        g = ctx.topo.geometry.points
        return float(np.linalg.norm(g[HandCrank.pin(ctx)][0] - g["O"][0]))

    def hub_r(self, ctx: Context) -> float:
        """The arm's hub round O: the key's notch (the key stands 2 mm proud of the shaft)
        and two thicknesses of plate past it (the cut rules' web)."""
        return self.box.shaft_d / 2 + self.box.key_proud + 0.3 + 2 * ctx.sheet_t("crank")

    def web_r(self, ctx: Context) -> float:
        """The arm's half-width at the crankpin: the standoff's hole and two thicknesses."""
        return (CRANKPIN_D + SOCKET_FIT) / 2 + 2 * ctx.sheet_t("crank")

    def tie_r(self) -> float:
        """How far out along the arm the crank block's tie stands: its thin nut under the
        block (its corners) clear of the riser's thin nuts' corners on the PCD."""
        return math.ceil(self.box.pcd / 2 + RISER_HEAD_R + TIE_NUT[0] / math.sqrt(3) + 0.5)

    def screw_points(self, ctx_or_build) -> list[tuple[float, float]]:
        """The gearbox's four face studs, world XY: on the PCD at 45° to the input shaft."""
        g = (ctx_or_build.ctx if isinstance(ctx_or_build, Build) else ctx_or_build)
        o = np.asarray(g.topo.geometry.points["O"][0], dtype=float)
        a0 = math.radians(self.input_angle_deg)
        return [tuple(o + self.box.pcd / 2 * np.array([math.cos(a), math.sin(a)]))
                for a in (a0 + math.pi / 4 + k * math.pi / 2 for k in range(4))]

    def layout_z(self, ctx: Context, L: Layout) -> tuple[float, float, Riser, float]:
        """(the keyed plates' bottom and top, the riser, the block's top: whole plates from
        the keyed ones up to its seat plate on the shaft's end), side z."""
        z0 = L.z(0)[0]
        t = ctx.sheet_t("crank")
        a0, a1 = arm_z(z0, ctx.sheet_t("frame"), KEYED_PLATES * t)
        riser = riser_for(z0, a0, t, self.box, ctx.sheet_t("frame"), t)
        return a0, a1, riser, a0 + BLOCK_PLATES * t

    # -- claims -----------------------------------------------------------------------

    def claims(self, ctx: Context) -> list[Claim]:
        pin, rider = self.pin(ctx), self.rider(ctx)
        geo = ctx.topo.geometry
        names = []
        for i, xy in enumerate(self.screw_points(ctx)):
            name = f"crank.screw{i}"
            geo.points[name] = np.broadcast_to(np.asarray(xy, dtype=float), (geo.samples, 2))
            names.append(name)
        geo.__dict__.pop("step", None)      # cached per point; recompute with the new ones
        m = ctx.params.margin
        hub, web = self.hub_r(ctx), self.web_r(ctx)
        fills = arm_fills(ctx, hub, web, pin, self.throw(ctx))     # (its neck's R10)
        t = ctx.sheet_t("crank")
        head_r = RISER_HEAD_R + 0.3                         # the riser's thin nuts
        end_r = end_screw(self.box).head_d / 2 + 0.3        # the shaft's end screw's head
        at_o = self.box.pass_d / 2 + m
        over_r = get(PIN_WASHER).dims["od"] / 2 + 0.3   # their washers over it

        def both(layer, shape, label, **kw):
            return [Placed(layer, shape, GROUP, label, **kw),
                    Placed(layer, shape, GROUP, label, gap=True)]

        def z_lo(L: Layout, k: int) -> float:
            """Layer ``k``'s bottom (over the arm): the plan's own z, or (a search's layout,
            :class:`stack.Layout`) the least it can be: the arm's top (fixed by the frame
            plate and the head layer alone, :func:`arm_z`) and each layer between at least
            its bound, no gap."""
            if L.final:
                return L.z(k)[0]
            a1 = arm_z(L.z(0)[0], ctx.sheet_t("frame"), KEYED_PLATES * t)[1]
            return a1 + sum(L.t(j) for j in range(ARM_LAYER + 1, k))

        def z_up(L: Layout, k: int) -> float:
            """The most layer ``k``'s bottom can be in any plan of ``L`` (each layer between
            the arm and it at its upper bound, a gap as thick as one can be over each)."""
            if L.final:
                return L.z(k)[0]
            a1 = arm_z(L.z(0)[0], ctx.sheet_t("frame"), KEYED_PLATES * t)[1]
            return (a1 + sum(L.t_hi(j) for j in range(ARM_LAYER + 1, k))
                    + (k - ARM_LAYER) * L.gap_hi)

        def rise_up(L: Layout, a: int, b: int) -> float:
            """The most layer ``b``'s bottom can be over layer ``a``'s."""
            if L.final:
                return L.z(b)[0] - L.z(a)[0]
            return sum(L.t_hi(j) + L.gap_hi for j in range(a, b))

        def block(L: Layout) -> tuple[list[int], list[int], list[int], int, float]:
            """The layers the block crosses (the keyed plates' and up to its top), the ones
            the heads over it cross (the end screw's, the tie's), the layers one of those
            surely takes without its being known which (a search's layout), the highest
            layer either may take, and the heads' top. At the plan's own z: exact (the
            third empty). A search's layout claims only what every plan of it has: a layer
            the block crosses whatever its z (its bottom under the block's top even at its
            highest), and so on; a layer the bounds leave open takes what both would have
            there (:func:`stack.Layout`)."""
            top = self.layout_z(ctx, L)[3]
            heads = top + HEADS_OVER
            ks = [ARM_LAYER]
            while z_up(L, ks[-1] + 1) < top - 1e-6:
                ks.append(ks[-1] + 1)
                if ks[-1] >= L.top - 2:
                    raise Unbuildable("the crank block reaches the inner plate")
            hs = [ks[-1]] if z_lo(L, ks[-1] + 1) > top + 1e-6 else []
            either: list[int] = []
            k = ks[-1] + 1
            while k < L.top and z_lo(L, k) < heads - 1e-6:
                if z_up(L, k) >= heads - 1e-6:
                    break               # (it may be over the heads: nothing sure from here)
                if z_lo(L, k) >= top - 1e-6:
                    hs.append(k)        # over the block's top in every plan: the heads'
                else:
                    either.append(k)    # the block's or the heads'
                k += 1
            most = k                    # the highest layer the block or its heads may take
            while most < L.top and z_lo(L, most) < heads - 1e-6:
                most += 1
            return ks, hs, either, most - 1, heads

        def fixed(L: Layout):
            if L.top <= ARM_LAYER + 2:
                raise Unbuildable("the stack has no room for the crank block and the rod over it")
            a0 = self.layout_z(ctx, L)[0]
            if L.final and abs(L.z(ARM_LAYER)[0] - a0) > 1e-6:
                # (a link or a gap under the block: the riser is sized for its z)
                raise Unbuildable(f"the crank block's layer {ARM_LAYER} is at z "
                                  f"{L.z(ARM_LAYER)[0]:.2f}, not the {a0:.2f} over the plate "
                                  f"and the screw heads' layer")
            ks, hs, either, _, _ = block(L)
            out = [Placed(0, Disc("O", self.box.pass_d / 2 + 1.0), GROUP, "output shaft",
                          seat=True)]
            out += [Placed(0, Disc(n, (self.box.face_thread + 0.6) / 2), GROUP,
                           "riser stud", seat=True) for n in names]
            out.append(Placed(0, Disc("O", at_o), GROUP, "output shaft", gap=True))
            out += [Placed(HEAD_LAYER, Disc(n, head_r), GROUP, "riser nut",
                           sheet=HEAD_LAYER_T) for n in names]
            out += both(HEAD_LAYER, Disc("O", at_o), "output shaft")
            # under the block: the crankpin's screw head, the tie's nut and its bolt's end
            out += both(HEAD_LAYER, Pill("O", pin, over_r), "block screw heads")
            for k in ks:                            # (the keyed plates: one thick layer)
                out.append(Placed(k, Disc("O", hub), GROUP, "crank block hub",
                                  sheet=KEYED_PLATES * t if k == ARM_LAYER else 0.0))
                out.append(Placed(k, Disc("O", hub), GROUP, "crank block hub", gap=True))
                out += both(k, Pill("O", pin, web), "crank block")
                out += [c for p, q, w in fills for c in both(k, Pill(p, q, w), "crank block")]
            # over it: the end screw's head at O, the tie bolt's head, the crankpin's spacer
            # (the rod rides over these)
            for k in hs:
                out += both(k, Disc("O", end_r), "end screw head")
                out += both(k, Pill("O", pin, CRANKPIN_D / 2 + 2.0),
                            "tie bolt and crankpin spacer")
            for k in either:                        # (a search's: what both have there)
                out += both(k, Disc("O", min(hub, end_r)), "crank block or its heads")
                out += both(k, Pill("O", pin, min(web, CRANKPIN_D / 2 + 2.0)),
                            "crank block or its heads")
            return out

        def crankpin(L: Layout):
            a0, _, _, _ = self.layout_z(ctx, L)
            ks, hs, either, most, heads = block(L)
            r = L.layers[rider]
            sure = max(ks + hs + either)
            # (a search's layout: only what holds in every plan of it, its upper bounds)
            if r <= sure or z_up(L, r) < heads + 0.3 - 1e-6:
                raise Unbuildable(f"the rod {rider} must ride over the crank block and its "
                                  f"screws' heads (z {heads + 0.3:.1f} mm), not in layer {r}")
            rod_top = z_up(L, r) + ctx.sheet_t("link", rider)
            length = rod_top + CRANKPIN_PLAY - (a0 + t)
            if length < 2 * NETRF12_THREAD:
                raise Unbuildable(f"the crankpin standoff would be {length:.1f} mm, under the "
                                  f"{2 * NETRF12_THREAD:g} its two M6 threads need")
            out = []
            spacer = CRANKPIN_D / 2 + 2.0
            for layer in range(sure + 1, r):     # its printed spacer up to the rod
                if layer > most:
                    out += both(layer, Disc(pin, spacer), "crankpin spacer")
                else:       # (a search's: the block's, its heads' or the spacer, all round pin)
                    out += both(layer, Disc(pin, min(spacer, web)), "crankpin spacer")
            out.append(Placed(r, Disc(pin, CRANKPIN_D / 2), GROUP, "crankpin", seat=True))
            over = (ctx.sheet_t("link", rider) + CRANKPIN_PLAY + PIN_WASHER_T + M6.head_h
                    + 0.3)                  # the washer and screw over the rod's bottom face
            hi = r + 1
            while hi < L.top and rise_up(L, r, hi) < over - 1e-6:
                out += both(hi, Disc(pin, over_r), "crankpin washer and screw")
                hi += 1
            if L.final and L.z(hi)[0] - L.z(r)[0] < over - 1e-6 and hi >= L.top:
                # (only at the plan's own z: a clearance gap under the plate may take it, and
                # a search's bound has none, so it would be stricter than the plan)
                raise Unbuildable(f"the crankpin's screw over {rider} reaches the inner plate")
            out.append(Placed(r, Disc(pin, over_r), GROUP, "crankpin washer and screw",
                              gap=True))
            return out

        return [Claim("hand crank", frozenset(), fixed),
                Claim("hand crankpin", frozenset((rider,)), crankpin)]

    # -- what the build needs to know ---------------------------------------------------


    def handle_axis(self, build: Build, riser: Riser) -> tuple[np.ndarray, np.ndarray, float]:
        """(the input shaft's end, world XY; its direction; its world z)."""
        o = build.xy("O")
        a = math.radians(self.input_angle_deg)
        d = np.array([math.cos(a), math.sin(a)])
        n = np.array([-d[1], d[0]])      # the worm's side (_frame_of's e2)
        end = o + d * (self.box.input_face + self.box.input_len) + n * self.box.worm_offset
        return end, d, riser.face - self.box.width / 2

    # -- build ------------------------------------------------------------------------

    def realize(self, build: Build, done: Realized) -> Realized:
        ctx = build.ctx
        out = Realized()
        box = self.box
        topo = build.plan.topo
        crank = next(iter(topo.crank_bodies))
        frame = topo.frame_bodies[0]
        pin, rider = self.pin(ctx), self.rider(ctx)
        o, mxy = build.xy("O"), build.xy(pin)
        p = ctx.params
        z0, z1 = build.z(0)                          # the outer frame plate
        a0, a1, riser, block_top = self.layout_z(ctx, build.plan.layout)
        if abs(build.z(ARM_LAYER)[0] - a0) > 1e-6:
            raise ConstructionError(f"the arm's layer is at z {build.z(ARM_LAYER)[0]:.2f}, "
                                    f"not the {a0:.2f} its riser is sized for")
        u = (mxy - o) / max(float(np.linalg.norm(mxy - o)), 1e-9)
        arm_ang = math.atan2(u[1], u[0])
        face = riser.face                                # the gearbox's machined output face

        # the outer plate: the output shaft's hole, the studs' holes, solid round them (no
        # guard over the crank: the user's 2026-10-10 call, a minimal mount; the crank turns
        # about 2 rpm behind the truss, and the use limit keeps fingers out of the drive side)
        screws = self.screw_points(build)
        hole = box.face_thread + 0.6
        out.cut(FRAME_OUTER, Cut(tuple(o), box.pass_d + 2.0))
        for xy in screws:
            out.cut(FRAME_OUTER, Cut(xy, hole))
        pad = box.pcd / 2 + hole / 2 + 2 * ctx.sheet_t("frame") + 0.5
        out.pad(FRAME_OUTER, tuple(o), tuple(o), pad)

        # the riser, a clamped stack on the gearbox's face (design review round 3's M2): the
        # ring centred on the spigot, a steel sleeve on each stud, the go/no-go's shims, the
        # plate, a thin nut over it on each stud; the gearbox (outboard: the frame's)
        sl_od, sl_bore, sl_len, sl_key = RISER_SLEEVE
        if box.pcd / 2 - sl_od / 2 < box.spigot_d / 2 + 1.0 - 1e-9 or \
                box.pcd / 2 + sl_od / 2 > box.face_d / 2 - 1.0 + 1e-9:
            raise ConstructionError(f"a {sl_od:g} mm sleeve doesn't sit on the face between "
                                    f"its {box.spigot_d:g} mm spigot ring and its "
                                    f"{box.face_d:g} mm boss's edge")
        thin_af, thin_h, thin_key = RISER_THIN
        d_stud, _, stud_key = RISER_STUD
        ring_bore = box.spigot_d + RISER_RING
        ring_hole = box.face_thread + RISER_RING_HOLE
        t_ring = riser.ring_t
        web = box.pcd / 2 - ring_hole / 2 - ring_bore / 2
        if web < t_ring - 1e-9 or box.face_d / 2 - box.pcd / 2 - ring_hole / 2 < t_ring - 1e-9:
            raise ConstructionError(f"the riser's ring leaves {web:.2f} mm round its studs' "
                                    f"holes, under its {t_ring:g} mm thickness")
        shim_od, shim_bore = RISER_SHIM[0], RISER_SHIM[1]

        def hexnut(xy, af, z_lo, z_hi):
            from build123d import RegularPolygon, extrude

            prism = extrude(RegularPolygon(af / math.sqrt(3), 6), amount=z_hi - z_lo)
            prism = prism.moved(Location((float(xy[0]), float(xy[1]), z_lo)))
            return difference(prism, disc(xy, d_stud / 2, z_lo - 1, z_hi + 1))

        out.bodies.append(hardware("riser_ring", difference(
            ring(tuple(o), box.face_d, ring_bore, face, face + t_ring),
            *[disc(xy, ring_hole / 2, face - 1, face + t_ring + 1) for xy in screws]),
            frame, fab="laser", sheet=ctx.sheet("crank"), color="#9aa3ad"))
        s0 = face + t_ring + sl_len                     # the sleeve's top, the shims'
        for i, xy in enumerate(screws):
            out.bodies.append(hardware(f"riser_stud{i}", disc(xy, 0.97 * d_stud / 2,
                                                              face - riser.engage,
                                                              face - riser.engage
                                                              + riser.stud_drawn),
                                       frame, fab="purchased", bom_key=stud_key, color=STEEL))
            out.bodies.append(hardware(f"riser_sleeve{i}", ring(xy, sl_od, sl_bore,
                                                                face + t_ring, s0),
                                       frame, fab="purchased", bom_key=sl_key, color=STEEL))
            out.bodies.append(hardware(f"riser_shims{i}", ring(xy, shim_od, shim_bore + 0.2,
                                                               s0, z0),
                                       frame, fab="purchased", color=STEEL))
            out.bodies.append(hardware(f"riser_thin_nut{i}", hexnut(xy, thin_af, z1,
                                                                    z1 + thin_h),
                                       frame, fab="purchased", bom_key=thin_key, color=STEEL))
        # the shims and the coarse washers, bought by the pack (a stack is drawn as one)
        out.extras.append(BomLine(RISER_SHIM[3], RISER_SHIM[4],
                                  "the drive: the riser's go/no-go shims (0.5 mm), over its "
                                  "sleeves"))
        out.extras.append(BomLine(RISER_COARSE[0], RISER_COARSE[1],
                                  "the drive: the riser's coarse shims (1.6 mm), over its "
                                  "sleeves"))
        out.bodies.append(hardware("gearbox", self._gearbox(o, face, screws), frame,
                                   fab="purchased", bom_key=box.key, color=GEARBOX_COLOR))

        # the output shaft (in from the plate's side: its collar on the hub's end, its key
        # out of its seat)
        key_ang = arm_ang + math.pi / 2
        kc = np.array([math.cos(key_ang), math.sin(key_ang)])
        k_w = box.shaft_key[0]
        col_d, col_t = box.collar
        key = Box(box.key_proud + 3.0, k_w, riser.key[1] - riser.key[0]).moved(Location(
            (float(o[0] + kc[0] * (box.shaft_d / 2 + (box.key_proud - 3.0) / 2)),
             float(o[1] + kc[1] * (box.shaft_d / 2 + (box.key_proud - 3.0) / 2)),
             (riser.key[0] + riser.key[1]) / 2), (0, 0, math.degrees(key_ang))))
        tap_d, tap_depth = box.end_tap
        shaft = difference(union([
            disc(tuple(o), box.shaft_d / 2, riser.hub_end - 5.0, riser.shaft_end),
            disc(tuple(o), col_d / 2, riser.hub_end, riser.hub_end + col_t), key]),
            disc(tuple(o), tap_d / 2, riser.shaft_end - tap_depth, riser.shaft_end + 1.0))
        out.bodies.append(hardware("crank_shaft", shaft, crank, fab="purchased",
                                   bom_key=HandCrankConstruction.output_shaft, color=STEEL))

        # the crank block: BLOCK_PLATES plates from the keyed plates', each the arm's outline;
        # holes round the shaft (the keyed ones' slot filed to the key, the others' clear of
        # its end), the seat's for the end screw, those over it ring the end screw's head
        t = ctx.sheet_t("crank")
        zs = [a0 + i * t for i in range(BLOCK_PLATES)]
        if abs(zs[riser.seat_k] - riser.seat) > 1e-6 or riser.seat < riser.shaft_end - 1e-6:
            raise ConstructionError("the crank block's seat plate doesn't sit on the shaft's "
                                    "end")
        throw = float(np.linalg.norm(mxy - o))
        tie_r = self.tie_r()
        tie = o + (mxy - o) * (tie_r / throw)
        if tie_r - TIE_NUT[0] / math.sqrt(3) - (box.pcd / 2 + RISER_HEAD_R) < 0.5 or \
                throw - tie_r < CRANKPIN_D + SOCKET_FIT + 2 * t:
            raise ConstructionError(f"the crank block's tie at {tie_r:g} mm meets the riser's "
                                    f"nuts or the crankpin ({throw:.1f} mm out)")
        es = end_screw(box)
        slot_at = tuple(o + kc * (box.shaft_d / 2 + box.key_proud / 2))
        for i, z_lo in enumerate(zs):
            # the crankpin stands on the lowest plate; the tie bolt runs through them all
            pins = [Cut(tuple(mxy), M6.shank_d + 0.5 if i == 0 else CRANKPIN_D + SOCKET_FIT),
                    Cut(tuple(tie), M6.d + 0.6)]
            if i < riser.seat_k:                    # round the shaft: keyed or clear of it
                fit = KEY_SLOT if i < KEYED_PLATES else KEY_CLEAR
                at_o = [Cut(tuple(o), box.shaft_d + ARM_BORE_FIT),
                        Rect(slot_at, (box.key_proud + 2.6, k_w + fit), key_ang)]
            elif i == riser.seat_k:                 # the seat: the end screw through
                at_o = [Cut(tuple(o), es.d + 0.6)]
            else:                                   # round the end screw's head
                at_o = [Cut(tuple(o), es.head_d + 1.5)]
            part = arm_plate(self.hub_r(ctx), self.web_r(ctx), self.throw(ctx), o, mxy, z_lo,
                             z_lo + t, [*at_o, *pins])
            name = crank if i == 0 else f"{crank}_plate{i}"
            out.bodies.append(hardware(name, part, crank, fab="laser", sheet=ctx.sheet("crank"),
                                       color="#9aa3ad"))

        def add(name: str, part, key: str):
            out.bodies.append(hardware(name, part, crank, fab="purchased", bom_key=key,
                                       color=STEEL))

        # the seat on the shaft's steel end face, the end screw through it
        seat_top = riser.seat + t
        reach = seat_top - riser.shaft_end
        end_len = next((L for L in es.lengths
                        if es.d <= L - reach <= tap_depth - 0.5), None)
        if end_len is None:
            raise ConstructionError(f"no stock M{es.d:g} screw reaches the shaft's end tap "
                                    f"{reach:.1f} mm under the crank block's seat plate's top")
        add("crank_end_screw", screw_solid(tuple(o), es, seat_top, end_len, up=False),
            es.key(end_len))

        # the tie: an M6 button head down through the whole block into a thin nut under its
        # lowest plate (threadlocker): the plates one rigid block, filed and fitted on the
        # bench before it goes on the shaft
        nut_af, nut_h, nut_key = TIE_NUT
        tie_len = next((L for L in M6.lengths if L - (block_top - a0 + nut_h) >= 1.0), None)
        if tie_len is None or block_top - tie_len < z1 + 2.0:
            raise ConstructionError(f"no stock M6 ties the crank block ({block_top - a0:.1f} "
                                    f"mm and a {nut_h:g} mm nut) clear of the outer plate")
        add("crank_tie", screw_solid(tuple(tie), M6, block_top, tie_len, up=False),
            M6.key(tie_len))
        add("crank_tie_nut", difference(disc(tuple(tie), nut_af / 2, a0 - nut_h, a0),
                                        disc(tuple(tie), M6.d / 2, a0 - nut_h - 1, a0 + 1)),
            nut_key)

        # the crankpin from the lowest plate (an M6 up through it) through the rod (a wide
        # washer and an M6 over the rod)
        pin_w = get(PIN_WASHER).dims
        pw_t, pw_od = float(pin_w["t"]), float(pin_w["od"])

        def standoff(xy, z_lo: float, z_hi: float) -> tuple[Shape3D, str]:
            """A NETRF12 from ``z_lo`` to at most ``z_hi``, its length in 0.1 mm steps."""
            length = math.floor((z_hi - z_lo) * 10 + 1e-6) / 10
            bore = M6.shank_d / 2 + 0.05            # (its tapped ends, the screws in them)
            part = difference(disc(xy, CRANKPIN_D / 2, z_lo, z_lo + length),
                              disc(xy, bore, z_lo - 1, z_lo + NETRF12_THREAD),
                              disc(xy, bore, z_lo + length - NETRF12_THREAD, z_lo + length + 1))
            return part, netrf12(length)

        def m6(grip: float) -> float:
            """The shortest stock M6 through ``grip`` mm with 6 mm to a NETRF12's 12 mm
            thread."""
            return next(L for L in M6.lengths if 6.0 <= L - grip <= NETRF12_THREAD)

        r0 = build.z(build.layers[rider])[0]
        rod_top = r0 + ctx.sheet_t("link", rider)
        pin_part, pin_key = standoff(tuple(mxy), a0 + t, rod_top + CRANKPIN_PLAY)
        out.bodies.append(hardware("crankpin", pin_part, crank, fab="purchased",
                                   bom_key=pin_key, color=STEEL))
        pin_top = a0 + t + _len_of(pin_key)
        lo = m6(t)
        add("crankpin_screw_lo", screw_solid(tuple(mxy), M6, a0, lo, up=True), M6.key(lo))
        add("crankpin_washer", ring(tuple(mxy), pw_od, M6.shank_d + 0.3, pin_top,
                                    pin_top + pw_t), PIN_WASHER)
        hi = m6(pw_t)
        add("crankpin_screw_hi", screw_solid(tuple(mxy), M6, pin_top + pw_t, hi, up=False),
            M6.key(hi))
        if r0 - block_top > 0.3:
            out.bodies.append(hardware("crankpin_spacer",
                                       ring(tuple(mxy), CRANKPIN_D + 4.0, CRANKPIN_D + 0.3,
                                            block_top, r0 - 0.1),
                                       crank, fab="printed", color=PRINT_COLOR))
        out.cut(rider, Cut(tuple(mxy), p.hole(CRANKPIN_D)))
        # the drive's threadlocker (243: the guide's steps): the riser's studs, the shaft's
        # end screw, the block's tie, the crankpin's two screws, the input's key
        locked = len(screws) + 1 + 1 + 2 + 1
        out.extras.append(BomLine(DRIVE_LOCK, 0.01 * locked,
                                  "the drive: the riser's studs, the end screw, the tie, the "
                                  "crankpin's screws, the input's key"))

        # the hand crank on the input shaft, its arm behind every moving part of the stack
        end, d, zh = self.handle_axis(build, riser)
        spec = handle_spec(ctx.config)
        bodies, sweep = self._handle(end, d, zh, frame, spec, ctx.sheet("crank"))
        out.bodies += bodies
        out.notes["hand_crank"] = {
            "gearbox": box.key, "ratio": box.ratio, "riser_height_mm": riser.height,
            "riser_stud_mm": riser.stud, "riser_engage_mm": riser.engage,
            "riser_stud_past_mm": riser.past, "riser_ring_mm": riser.ring_t,
            "riser_sleeve_mm": RISER_SLEEVE[2], "riser_shims_mm": riser.shims,
            "face_z_mm": round(riser.face, 3),
            "plate_inner_z_mm": round(z1, 3),
            "arm_z_mm": [round(a0, 3), round(block_top, 3)],
            "keyed_z_mm": [round(a0, 3), round(a1, 3)],
            "block_plates": len(zs), "seat_plate": riser.seat_k, "end_screw_mm": end_len,
            "end_screw": es.key(end_len),
            "key_z_mm": [round(riser.key[0], 3), round(riser.key[1], 3)],
            "hub_end_z_mm": round(riser.hub_end, 3),
            "shaft_end_z_mm": round(riser.shaft_end, 3),
            "seat_z_mm": round(riser.seat, 3), "seat_low_mm": riser.low,
            # the guide's go/no-go: the shaft's end over the outer plate's inner face, the
            # gearbox on its riser; and the seat plate's underside there
            "shaft_over_plate_mm": round(riser.shaft_end - z1, 2),
            "seat_over_plate_mm": round(riser.seat - z1, 2),
            "crankpin_mm": _len_of(pin_key), "crankpin_top_z_mm": round(pin_top, 3),
            "tie_bolt_mm": tie_len,
            "crankpin_lever_mm": round(r0 + ctx.sheet_t("link", rider) / 2 - block_top, 3),
            "rod_mid_z_mm": round(r0 + ctx.sheet_t("link", rider) / 2, 3),
            "handle": {"end_xy": [round(float(v), 3) for v in end],
                       "dir": [round(float(v), 6) for v in d], "axis_z": round(zh, 3),
                       "reach_mm": spec.reach, "grip_r_mm": spec.throw,
                       "grip_mm": list(spec.grip), "load": spec.load,
                       "sweep": [[round(v, 3) for v in s] for s in sweep]},
            "throw_mm": round(float(np.linalg.norm(mxy - o)), 3),
            "irreversible": box.irreversible(),
            "petg": list(PETG_PARTS),
            # the keyed plates' slots cut under the key, filed to it at assembly (round 2's m4)
            "filed": [["crank_shaft", crank if i == 0 else f"{crank}_plate{i}"]
                      for i in range(KEYED_PLATES)],
        }
        return out

    def assembly(self, view) -> list:
        """How the drive goes on (:mod:`construction.assembly`, a cradle's
        ``CRADLE_ORDER``): on the bench the shaft through the gearbox from the face's side,
        the studs in the face, the ring, the sleeves and the shims on them; the outer plate
        down onto the shims and the thin nuts over it (their heads end up inside the stack, so
        first), and the go/no-go measurement that sets the shims; the crank block built and
        fitted on the bench (its plates, the tie, the crankpin), then down over the shaft onto
        the shaft's end and the end screw; the crankpin's spacer before the rod, its washer and
        screw once the rod is on; the hand crank, its plungers at their felt start; and, the
        cradle built, the clutch set on a measured start, the worm's hold and the toggle
        zones' marks (:meth:`first_pour`: design review round 3's stand and use M2, M4)."""
        from spiderpig.construction.assembly import FINISH, STACK, Op, whole

        if "gearbox" not in view.z:
            return []
        box = self.box
        topo = getattr(view.ctx, "topo", None)
        arm = next(iter(topo.crank_bodies)) if topo is not None else "conn"
        rider = HandCrank.rider(view.ctx) if view.ctx is not None else None
        k_rod = view.slot(view.z[rider][0]) if rider in view.z else ARM_LAYER + 1
        studs = view.named(r"riser_stud\d+")
        sleeves = view.named(r"riser_sleeve\d+")
        shims = view.named(r"riser_shims\d+")
        thins = view.named(r"riser_thin_nut\d+")
        ring_ = [n for n in ("riser_ring",) if n in view.z]
        plates = view.named(rf"{re.escape(arm)}_plate\d+")
        k = box.shaft_key
        spec = handle_spec(getattr(view.ctx, "config", None))
        es = end_screw(box)
        # the go/no-go: the shaft's end over the outer plate's inner face, and the seat
        # plate's underside there (the block on the end, at most SEAT_LOW low)
        inner = view.layers[0][1]
        shaft = view.z["crank_shaft"][1] - inner if "crank_shaft" in view.z else 0.0
        over = [view.z[p][0] for p in [arm, *plates] if p in view.z
                and view.z[p][0] >= view.z.get("crank_shaft", (0.0, -1e9))[1] - 1e-6]
        seat = (min(over) if over else 0.0) - inner
        n_clear = len([p for p in [arm, *plates] if p in view.z]) - len(over) - KEYED_PLATES
        a1 = view.layers[ARM_LAYER][0] + KEYED_PLATES * HAND_CRANK.plate_t - inner
        lo = seat - min(SEAT_LOW, seat - (a1 - 0.2 + box.key_short + k[0] / 2))
        d_stud, cut, _ = RISER_STUD
        stand = cut - (box.face_depth - 0.5)            # as cut, in the face until it stops
        shim_t = RISER_SHIM[2]
        n_shims = round((view.z[shims[0]][1] - view.z[shims[0]][0]) / shim_t) if shims else 0
        sl_od, sl_bore, sl_len, _ = RISER_SLEEVE
        ops = [
            Op(STACK, (-1, -1, "gearbox"), whole("gearbox", "crank_shaft", *studs, *ring_,
                                                 *sleeves, *shims),
               "Gearbox on its riser",
               "Wipe the gearbox: a new worm box can weep oil at its seals. Run it in now, "
               "before it goes on the cradle (a drill on the built cradle's input would turn "
               f"it past the hand crank's clutch): a few hundred turns of its {box.input_d:g} "
               "mm input shaft (a drill, slowly), then wipe it again; it stands over the base "
               "board beside the wine. The output shaft into the gearbox's hollow output from "
               f"the face's side, collar first against the hub, its {k[0]:g} x {k[1]:g} key in "
               "its seat (the snap ring and washer on its far end); the gearbox on its back, "
               "face up, on a block with a hole for the shaft's far end. The riser's four "
               f"studs: saw the head off each of the four M{d_stud:g} x 60 A2-70 hex screws "
               f"and cut it to {cut:g} mm (a hacksaw; file each cut end square and run a nut "
               "on and off it to clean its thread). Each into one of the face's four tapped "
               "holes, a drop of threadlocker (243) on its thread, two nuts locked together on "
               "it to turn it, "
               f"until it stops: it stands about {stand:.1f} mm over the face (a caliper's "
               "depth rod; if it stands more than a millimetre taller, the hole's thread is "
               "short: cut the stud down to it). The riser's ring (cut with the crank plates) "
               "down over the studs onto the machined face, its bore round the raised spigot "
               "ring: it must lie flat (a stud that won't pass its hole: file the hole, never "
               f"force it). An aluminium sleeve ({sl_bore:g} x {sl_od:g} x {sl_len:g} mm) "
               "down "
               f"each stud onto the ring, and {n_shims} of the {shim_t:g} mm shims on each "
               "sleeve: the go/no-go may change how many.", "gearbox", sub=True),
            Op(STACK, (-1, 2), whole(*thins), "",
               "The outer plate over the studs onto the shims, its outside down, the shaft up "
               "through its bore; an M8 thin nut on each stud over the plate, finger tight. The "
               "gearbox now stands the plate up off the bench. Go/no-go: measure the shaft's "
               "end over the plate's inner face (a caliper's depth rod beside the shaft): "
               f"{shaft:.2f} mm drawn, the crank block's seat plate sitting on it. Between "
               f"{lo:.2f} and {seat:.2f} it is right (the block sits on the shaft's end, at "
               f"most {seat - lo:.1f} mm lower than drawn). Over {seat:.2f} (the hub prouder "
               "than drawn): the plate off, a shim more on every sleeve (a 1.6 mm washer for "
               f"three); under {lo:.2f} (lower: the key's straight flanks would fall short of "
               "the keyed plates): a shim fewer on every sleeve; each shim moves it "
               f"{shim_t:.2f} mm; and measure again. Then each thin nut, a drop of 243 on its "
               f"thread, {RISER_NUT_NM:g} N·m (a torque wrench), the four in a cross: the "
               "plate, the shims, the sleeves and the ring one clamped stack on the face. Each "
               f"stud's end must stand no more than {STUD_PAST_MAX:g} mm over its thin nut "
               "(the crank block's lowest plate passes over the four): file it down, masking "
               "tape on the plate round it.", "gearbox_studs"),
            Op(STACK, (ARM_LAYER, 0), whole(arm, *plates, "crank_tie", "crank_tie_nut",
                                            "crankpin", "crankpin_screw_lo"),
               "Crank block",
               "On the bench, as one block: the lowest plate (keyed, a small hole at its end), "
               "the thick steel standoff (the crankpin) stood on that small hole, an M6 button "
               f"head up through the plate into it (threadlocker 243), {SCREW_NM:g} N·m; the "
               "other plates down over the crankpin in their order (the "
               f"{KEYED_PLATES - 1} more keyed ones with the narrow slot, the "
               f"{n_clear} with a wider slot, the seat with the "
               f"{es.d + 0.6:.1f} mm hole, the one with the {es.head_d + 1.5:g} mm hole on "
               "top), the tie's M6 button head down through every plate's middle hole into its "
               "thin nut under the lowest plate (threadlocker 243; a 10 mm wrench on the nut), "
               f"{SCREW_NM:g} N·m. Fit the keyways: the keyed plates' slots are cut "
               f"{-KEY_SLOT:g} mm under the shaft's {k[0]:g} mm key, so each can be filed to "
               "it; push the key, out of its seat, up into the block's slots from under it "
               "and file the narrow slots together (a flat file, the block tied) until it "
               f"slides through all {KEYED_PLATES} with no play: until then the "
               f"tightest plate carries the crank alone. The {n_clear} wider slots "
               f"over them are cut {KEY_CLEAR:g} mm over the key, clear of it: no file. The "
               "key back in its seat.", "layer"),
            Op(STACK, (ARM_LAYER, 1), whole("crank_end_screw"), "",
               "The block down over the shaft, its keyways on the key, its seat plate onto the "
               f"shaft's end; the M{es.d:g} button head down through it into the shaft's end "
               f"tap, threadlocker (243), {END_SCREW_NM:g} N·m (a torque wrench: steel and "
               "aluminium only in this clamp, so it holds its preload). The plate must sit on "
               "the shaft's end, not on the screw: no shake along the shaft. Turn the "
               "gearbox's "
               f"{box.input_d:g} mm input shaft by hand until the block has gone a whole turn "
               "(30 turns: the worm can't be turned from the block): nothing may touch the "
               "outer plate, the riser's thin nuts or its studs' ends.", "layer"),
            Op(STACK, (ARM_LAYER, 2), whole("crankpin_spacer"), "",
               "The crankpin's printed spacer down over the crankpin onto the block; grease "
               "the crankpin where the rod turns on it (aluminium on bare steel: any lithium "
               "or PTFE grease).", "layer"),
            Op(STACK, (k_rod, 2), whole("crankpin_washer", "crankpin_screw_hi"), "",
               "With the rod on the crankpin: the wide washer and the M6 button head into "
               f"the crankpin's top (threadlocker 243), {SCREW_NM:g} N·m. The rod must turn "
               "freely under the washer with a little end play (about 0.3 mm): if it binds, "
               "sand its printed spacer down (never shim it up: the washer keeps it off the "
               "screw heads over it).", "layer"),
            Op(FINISH, (2,), whole(*view.named(r"handle(_\w+)?")),
               "Hand crank",
               "Print the hub (with its tube, flange and journal), the arm and the cap in "
               "PETG (a hot car softens PLA, and the plungers push on these all the time), "
               "the hub solid, 100 % infill: it carries the input's key; ream its bore with a "
               f"{box.input_d:g} mm drill by hand. M3 heat-set inserts, a soldering iron "
               "at about 230 °C: one down into the hub's hole over the keyway, one into the "
               "journal's end"
               + ("" if "handle_tube" not in view.z else
                  ", one into the hub's hex spigot (a long tube prints in two pieces: the "
                  "spigot into the tube's socket, an M3 button head through the socket's wall "
                  "into that insert, threadlocker 243)")
               + ". The clutch's seat ring (cut with the crank plates) onto the flange, its "
               "hex bore on the flange's hex, the face the laser entered up (the clean face "
               "marked on arrival, its six seats broken alike: the balls ride it; the face "
               "with the dross down on the flange); grease its face, its six holes and the "
               "hub's journal (the food-grade grease). A 1/2\"-13 "
               "hex nut pressed into each of the three hex pockets in the arm's boss from its "
               "inner face, the face toward the ring (a vise): the pocket's floor takes the "
               "spring's push. A spring plunger threaded through each from outside, ball "
               "first, until its ball just stands out of the inner face, a jam nut on each, "
               "loose. The arm onto the journal, the balls toward the ring; the steel washer "
               "(greased) and the cap on the journal's flat, its M3 thumb screw into the "
               f"insert, snug. The grip: an M{spec.bolt[0]:g} washer, the printed grip, a "
               f"washer and the arm's end over the M{spec.bolt[0]:g} bolt from the outside, a "
               "washer and the nyloc nut on the arm's inner face, tight against the arm: the "
               "grip spins on the bolt's plain shank. A drop of threadlocker (243) in the "
               f"gearbox's input keyseat and its {box.input_key[0]:g} x {box.input_key[0]:g} "
               "key pressed in (it comes with the gearbox; else a DIN 6885 key that size), so "
               "it stays when the crank comes off.",
               "handle", lines=(
                   "The plungers' starting setting: turn the arm on the journal until it drops "
                   "into a click (each ball in a hole). Turn each plunger in by its hex until "
                   f"it stops hard (its nose on the ring), then out {_turns(SET_BACK)}, and "
                   "snug its jam nut holding its back still: the clutch now slips about where "
                   "it will be set. Never turn a plunger nearer that stop than "
                   f"{_turns(NEAR_STOP)}: at the stop its ball can't climb out of its hole, "
                   "and the clutch is locked solid.",
                   "The crank's hub onto the input shaft, its keyway over the key, and the "
                   "hub's M3 thumb screw down onto the key by hand (snug: it only keeps the hub "
                   "on; it comes off the same way). Stand at the drive side's rear corner to "
                   "turn it, the neck in view past the stand. The clutch is set on a measured "
                   "start once the cradle is built (the last step, \"Before the first pour\").",
                   "Then the rod's check: from rest (the cradle neck up) turn the handle "
                   "4 turns one way and, if the rear tie's screw head under the drive side's "
                   "cradle link never passed over the rod, back to rest and 4 turns the other "
                   "way. Where the head is over the rod, press the rod up against its washer "
                   "and slide a sheet of paper between them: it must pass. If it drags, take "
                   "the crankpin's top screw and washer off, file 0.1 mm off the crankpin's "
                   "top end, sand the rod's spacer to keep its play, and check again.")),
            self.first_pour(view, spec),
        ]
        ops = [replace(op, pieces=tuple(p for p in op.pieces if p.name in view.z))
               for op in ops]
        return [op for op in ops if op.pieces or op.tag == "clutch"]

    @staticmethod
    def first_pour(view, spec: HandleSpec):
        """The drive's last step, the cradle built (design review round 3's stand and use
        M2: the hand crank's step had set the clutch "with the heaviest full bottle in the
        cradle" before the cradle box existed): the start measured, the clutch set on it under
        the ceiling (:func:`clutch_ceiling`, its M1) and checked on its printed stub in the
        riser locator's pocket (its m12: no vise), the worm's hold, and the toggle zones found
        by the crank and the rod and painted on the gearbox's far face (its M4)."""
        from spiderpig.construction.assembly import FINISH, Op

        key = getattr(getattr(view.ctx, "config", None), "linkage", "")
        start = _drawn_start(view, spec)
        new = round(start * NRV050.eta_static / NRV050.eta_new()) if start else 0
        drawn = drawn_clutch(key) if key in DESIGN_START_NM else None
        cap_set = drawn["set_grip_n"] if drawn else 0.0
        ceiling = drawn["hi_grip_n"] if drawn else 0.0
        tz = f"{TOGGLE_TURNS:g}"
        reading = (f" (a full 6 L reads about {start:g} N on this crank, up to {new:g} N "
                   "while the gearbox is new)" if start else "")
        top = (f", but at most {cap_set:g} N ({cap_set / 9.81:.1f} kg): no pull may ever read "
               f"over {ceiling:g} N ({ceiling / 9.81:.1f} kg), whatever the start (a new "
               "gearbox's start reads high, and the cradle's parts are rated at that ceiling)"
               if drawn else "")
        return Op(FINISH, (3,), (), "Before the first pour",
                  "The cradle built, and at every set-up after it: the clutch set on a "
                  "measured start, the worm's hold, the toggle zones' marks.", "clutch",
                  count=False, lines=(
                      "The start: the heaviest full bottle you'll pour (a 6 L at most) in "
                      "the cradle, set for it; turn the handle slowly from rest and stop "
                      "where it is heaviest (about level); let go. The luggage scale on the "
                      "grip's middle, pulled square to the arm, slowly: the reading where the "
                      "handle starts to turn is the start" + reading + ".",
                      f"The target: {CLUTCH_SET:g} times the start" + top + ".",
                      "The check, on its stub (no vise): the bottle out, the riser out of "
                      "its locator and the printed check stub's bar down into the locator's "
                      "pocket, its post up; the hand crank off the gearbox (its thumb screw) "
                      "and its hub down over the post, its keyway on the post's key, its "
                      "thumb screw snug; the arm pointing out over the board's notch. The "
                      "luggage scale on the grip's middle, pulled square to the arm, slowly: "
                      "the reading where the arm clicks round is the slip.",
                      "Over the target: turn one plunger out an eighth of a turn (a hex key "
                      "in its back, about 3 % of the slip; the three in turn); under: in an "
                      "eighth; each time hold its back and snug its jam nut, and pull again. "
                      f"It is set when three pulls in a row read within "
                      f"{CLUTCH_CHECK * 100:g} % of the target and none over the ceiling.",
                      f"Never nearer the stop than {_turns(NEAR_STOP)}: if it still reads "
                      "under the target with all three there, leave it (under is the safe "
                      "side) while it reads 1.1 times your start or more; less, and a "
                      "plunger is weak (no maker states their tolerance): replace it.",
                      "Check it so at every set-up, after greasing it, after a trip in a hot "
                      "car, and every month or ten pours while it stands set up: the grease "
                      "under the balls ages and the slip drifts with it, and only the check "
                      "keeps it under the ceiling. Once a year grease the ring's holes, the "
                      "washer and the journal again and look at the holes: any bright land "
                      "you can see round a hole's edge, a new ring.",
                      "The worm's hold: the bottle back in, turn to level, let go, tap the "
                      "table and watch a pencil line across the crank's arm and the outer "
                      "plate for 10 minutes: it must not move. If it creeps back toward rest, "
                      "the worm holds less than drawn: an O-ring (about 15 x 2 mm) between "
                      "the hub's face and the gearbox's input seal adds a little drag (1 to 2 "
                      "N at the grip) and doubles the margin; a bottle that creeps never tips "
                      "further, it settles back neck up.",
                      f"The toggle zones (the use limits' rule 3): the {tz} handle turns "
                      "either side of rest and of full tilt, where no clutch bounds the force. "
                      "Find each toggle by the crank and the rod, not by the bottle (it barely "
                      "moves there): at rest the crank's arm lies folded under the rod, the two "
                      "one straight line, the rod's end over the arm's; at full tilt the arm "
                      "and the rod make one straight line, end to end.",
                      "On the gearbox's far face the output hub turns with the crank. At rest "
                      "draw an arrow on the hub's end with the paint pen, from its middle to "
                      "its rim, and a mark on the housing at its tip; turn the handle "
                      f"{tz} turns one way and mark the housing at the tip, then back through "
                      f"rest {tz} turns the other way and mark it; paint the housing's rim red "
                      "between those two outer marks. Likewise at full tilt. While the arrow "
                      "points into a red arc: hands out, and nothing added under the cradle "
                      "box."))

    def motion(self, got: Realized) -> Motion:
        """Each part rides its host: the block's plates, the shaft, the tie and
        the end screw, the crankpin and its spacer turn with the crank; the gearbox, its
        riser's studs, sleeves and nuts and the handle stand with the frame (the handle is
        drawn at rest)."""
        return RIDES_HOST

    # -- parts ------------------------------------------------------------------------
    def _frame_of(self, o, z_face) -> np.ndarray:
        """4x4: the gearbox's frame in the world: e1 the input shaft's direction, e2 its
        left (the worm's side: up for an input pointing between -90° and 90°), e3 = +z, the
        output axis toward the plate; origin O on its output face."""
        a = math.radians(self.input_angle_deg)
        e1 = np.array([math.cos(a), math.sin(a), 0.0])
        e2 = np.array([-e1[1], e1[0], 0.0])
        e3 = np.cross(e1, e2)
        m = np.eye(4)
        m[:3, 0], m[:3, 1], m[:3, 2] = e1, e2, e3
        m[:3, 3] = (o[0], o[1], z_face)
        return m

    def _gearbox(self, o, z_face, holes=()) -> Shape3D:
        """The gearbox drawn from its dimensions (enough for clearances and the look; the
        housing's square and the worm's barrel scaled from the NRV030's by the worm's
        offset), its face's tapped holes at ``holes`` (world XY) and its hub bored for the
        shaft."""
        b = self.box
        k = b.worm_offset / NRV030.worm_offset
        half = min(40.0 * k, b.input_face - 2.0, b.cap_face - 2.0)   # (inside the shoulders)
        body = Box(2 * half, b.below + 45.0 * k, b.width).moved(
            Location((0.0, (45.0 * k - b.below) / 2, -b.width / 2)))
        worm = Cylinder(22.0 * k, b.input_face + b.cap_face, rotation=(0, 90, 0)).moved(
            Location(((b.input_face - b.cap_face) / 2, b.worm_offset, -b.width / 2)))
        shaft = Cylinder(b.input_d / 2, b.input_len, rotation=(0, 90, 0)).moved(
            Location((b.input_face + b.input_len / 2, b.worm_offset, -b.width / 2)))
        spigot = Cylinder(b.spigot_d / 2, b.spigot_h).moved(Location((0.0, 0.0, b.spigot_h / 2)))
        hub = Cylinder(b.hub_d / 2, b.width + 2 * b.hub_out).moved(
            Location((0.0, 0.0, -b.width / 2)))
        local = union([body, worm, shaft, spigot, hub])
        part = moved(local, to_location(self._frame_of(o, z_face)))
        cut = [disc(tuple(o), b.shaft_d / 2 + 0.1, z_face - b.width - 10, z_face + 10)]
        cut += [disc(xy, b.face_thread / 2, z_face - b.face_depth, z_face + 3) for xy in holes]
        return difference(part, *cut)

    def _handle(self, end, d, z, host, spec: HandleSpec = HANDLES[""],
                sheet: str | None = None) -> tuple[list, list[tuple[float, float, float]]]:
        """The printed hand crank on the input shaft, drawn at rest (the arm down), and what
        its turning sweeps (``(r, x0, x1)``: cylinders round the shaft's axis, ``x`` along it
        from the shaft's end): built along +x from the shaft's end, then turned to ``d`` and
        moved to ``end`` (side z ``z``); ``spec`` its arm (:data:`HANDLES`), ``sheet`` the
        seat ring's (the crank sheet).

        * The hub (printed solid, PETG) on the shaft, keyed over its key, an M3 thumb screw
          down onto the key through a heat-set insert (:data:`HANDLE_THUMB`), runs on as a
          tube to the clutch's flange (a cone under it, so it prints hub down), a hex boss on
          the flange's face and a journal through the arm's boss.
        * The seat ring (laser-cut 6061: :data:`SEAT_RING`) on the flange's face, its hex
          bore on the hex boss, its six holes the seats.
        * The arm (PETG) turns on the journal: :data:`CLUTCH`'s plungers threaded through
          hex nuts pressed into its boss, each locked by a jam nut, their balls on the ring's
          holes' edges; it slips round the tube past its setting (:func:`clutch`). A steel
          washer and the cap (keyed on the journal's flat, an M3 thumb screw into an insert in
          the journal's end) hold it on against the plungers' push.
        * The grip, a printed sleeve, spins on an M8 bolt's plain shank through the arm's
          end, a washer either side of it and a nyloc nut on the arm's inner face."""
        from build123d import Cone, RegularPolygon, Sphere, extrude

        along_x = (0.0, 90.0, 0.0)                      # a cylinder's axis along +x
        a = math.degrees(math.atan2(d[1], d[0]))
        cl = CLUTCH
        throw, reach = spec.throw, spec.reach

        def place(part):
            return part.rotate(Axis.Z, a).moved(Location((float(end[0]), float(end[1]), z)))

        def rod(r: float, x0: float, x1: float, y: float = 0.0, zz: float = 0.0):
            return Cylinder(r, x1 - x0, rotation=along_x).moved(
                Location(((x0 + x1) / 2, y, zz)))

        def cone(r0: float, r1: float, x0: float, x1: float, y: float = 0.0, zz: float = 0.0):
            """A cone along +x, ``r0`` at ``x0``, ``r1`` at ``x1``."""
            return Cone(r0, r1, x1 - x0, rotation=along_x).moved(
                Location(((x0 + x1) / 2, y, zz)))

        def ring_x(ro: float, ri: float, x0: float, x1: float, y: float = 0.0,
                   zz: float = 0.0):
            return rod(ro, x0, x1, y, zz) - rod(ri, x0 - 1, x1 + 1, y, zz)

        def hex_x(af: float, x0: float, x1: float, y: float = 0.0, zz: float = 0.0):
            """A hexagonal prism across flats ``af`` along +x."""
            prism = extrude(RegularPolygon(af / math.sqrt(3), 6), amount=x1 - x0)
            return prism.rotate(Axis.Y, 90).moved(Location((x0, y, zz)))

        hub_r = self.box.crank_hub_r
        t_ring, gap, boss_t, jr = cl["ring_t"], cl["gap"], cl["boss_t"], cl["journal_r"]
        r_c, hex_af = cl["r"], cl["hex_af"]
        t_sheet = t_ring
        ring_r = math.ceil(2 * (r_c + cl["seat_d"] / 2 + 2 * t_sheet)) / 2
        fl_r = ring_r + 0.5
        a_in = reach - boss_t                           # the arm's boss's inner face
        r1 = a_in - gap                                 # the seat ring's face (the balls')
        f1 = r1 - t_ring                                # the flange's face
        f0 = f1 - cl["flange_t"]
        c0 = f0 - (fl_r - hub_r)                        # a 45° cone under the flange
        if c0 < 2.0:
            raise ConstructionError(f"a {reach:g} mm reach leaves no tube between the hub and "
                                    "the clutch's flange")
        t_washer = THRUST[2]
        cap0 = reach + t_washer                         # the cap's underside, on the washer
        j1 = cap0 + 2.0                                 # the journal's end, inside the cap
        cap1 = j1 + 1.5
        ins_d, ins_len, ins_key = INSERT

        # -- the hub: on the shaft, keyed, the thumb screw in its insert over the key
        key_w, key_hub = self.box.input_key
        key_top = self.box.input_d / 2 + key_hub      # the keyway's top, over the axis
        t_len, t_d, t_h = HANDLE_THUMB
        if key_top + t_len <= hub_r + 0.5:
            raise ConstructionError(f"an M3 x {t_len:g} thumb screw doesn't reach the input "
                                    f"key through the {hub_r - key_top:.1f} mm hub")
        xt = -HUB_LEN / 2                               # the thumb screw's place along it
        ins0 = hub_r - ins_len                          # its insert, down from the hub's face

        def radial(r: float, z0: float, z1: float):
            return Cylinder(r, z1 - z0).moved(Location((xt, 0.0, (z0 + z1) / 2)))

        flat = jr - 1.5                                 # the journal's end's flat (the cap's key)
        journal = rod(jr, f1 - 1.0, j1) - Box(j1 - reach + 1.0, 2 * jr, 2 * jr).moved(
            Location(((reach + j1) / 2 + 1.0, 0.0, flat + jr)))
        hub = union([rod(hub_r, -HUB_LEN, c0 + 0.5), cone(hub_r, fl_r, c0, f0 + 0.01),
                     rod(fl_r, f0, f1), hex_x(hex_af - 0.3, f1 - 0.01, r1 - 0.2), journal])
        bore = rod(HUB_BORE / 2, -HUB_LEN - 0.5, 0.5)
        keyway = Box(HUB_LEN + 2.0, key_w + 0.2, key_top + 0.2).moved(
            Location((-HUB_LEN / 2, 0.0, (key_top + 0.2) / 2)))
        hole = union([radial(ins_d / 2, ins0, hub_r + 1.0), radial(1.6, key_top - 0.5,
                                                                    ins0 + 0.5)])
        j_hole = union([rod(ins_d / 2, j1 - ins_len, j1 + 1.0), rod(1.6, j1 - ins_len - 4.0,
                                                                     j1 - ins_len + 0.5)])
        hub = difference(hub, bore, keyway, hole, j_hole)
        split = []                              # a long hub and tube: two printed pieces
        if j1 + HUB_LEN > HUB_PRINT_MAX:
            x_j = -HUB_LEN + HUB_SPLIT[0]
            af, sp_len = HUB_SPLIT[1], HUB_SPLIT[2]
            big = 4 * (fl_r + reach)
            beyond = Box(big, big, big).moved(Location((x_j + big / 2, 0.0, 0.0)))
            head = union([intersection(hub, beyond), rod(hub_r, x_j, x_j + 1.0)])
            x_s = x_j + sp_len / 2                  # the screw's place along the joint
            pocket = union([Cylinder(ins_d / 2, ins_len + 3.0).moved(
                Location((x_s, 0.0, af / 2 - ins_len / 2 + 1.5))),
                Cylinder(1.6, af / 2).moved(Location((x_s, 0.0, af / 4)))])
            spigot = difference(hex_x(af, x_j - 0.01, x_j + sp_len), pocket)
            hub = union([difference(hub, beyond), spigot])
            through = Cylinder(1.7, hub_r).moved(Location((x_s, 0.0, hub_r / 2 + af / 2 - 1)))
            head = difference(head, hex_x(af + 0.3, x_j - 1.0, x_j + sp_len + 0.2), through)
            sk = screw("bhcs", "3")
            s_top = hub_r                           # its head on the tube's crest
            s_len = next(L for L in sk.lengths      # 4 mm or more into the insert
                         if af / 2 - ins_len + 0.5 <= s_top - L <= af / 2 - 4.0 + 1e-6)
            t_ins = (Cylinder(ins_d / 2, ins_len) - Cylinder(1.5, ins_len + 2)).moved(
                Location((x_s, 0.0, af / 2 - ins_len / 2)))
            t_screw = union([Cylinder(sk.shank_d / 2, s_len).moved(
                Location((x_s, 0.0, s_top - s_len / 2))),
                Cylinder(sk.head_d / 2, sk.head_h).moved(
                    Location((x_s, 0.0, s_top + sk.head_h / 2)))])
            split = [("handle_tube", head), ("handle_tube_insert", t_ins, ins_key),
                     ("handle_tube_screw", t_screw, sk.key(s_len))]
        insert = (Cylinder(ins_d / 2, ins_len) - Cylinder(1.5, ins_len + 2)).moved(
            Location((xt, 0.0, ins0 + ins_len / 2)))
        t_top = key_top + 0.05 + t_len               # the thumb screw's head's underside
        thumb = union([Cylinder(1.45, t_len).moved(Location((xt, 0.0, t_top - t_len / 2))),
                       Cylinder(t_d / 2, t_h).moved(Location((xt, 0.0, t_top + t_h / 2)))])

        # -- the seat ring: on the flange's face, its hex bore on the hex boss, six seats
        seats_at = [(r_c * math.cos(phi), r_c * math.sin(phi))
                    for phi in (math.radians(-90.0 + 360.0 / cl["seats"] * i)
                                for i in range(cl["seats"]))]
        seat_ring = difference(rod(ring_r, f1, r1), hex_x(hex_af + 0.3, f1 - 1, r1 + 1),
                               *[rod(cl["seat_d"] / 2, f1 - 1, r1 + 1, y, zz)
                                 for y, zz in seats_at])

        # -- the arm on the journal: its boss with the plungers; the bar out to the grip
        big_r = PLUNGER["ball_d"] / 2
        a_seat = cl["seat_d"] / 2
        xb = r1 + math.sqrt(big_r ** 2 - a_seat ** 2)  # a ball's centre, seated on its edge
        setting = plunger_setting()                     # drawn at the guide's start: the
        pushed = setting["start_push_mm"]               # felt stop, SET_BACK turns out
        x_front = xb - big_r + PLUNGER["travel"] - pushed   # the body's nose face
        if min(setting["start_room_mm"], setting["limit_room_mm"]) < 0.2 - 1e-9 or \
                x_front < a_in - 1e-6:
            raise ConstructionError("the clutch's plungers can't climb out of their seats")
        x_back = x_front + PLUNGER["length"]
        nut_af, nut_h, nut_key = NUT_HALF
        jam_af, jam_h, jam_key = JAM_NUT
        if x_back < reach + jam_h + 0.5:
            raise ConstructionError(f"a plunger's back doesn't clear its jam nut over the "
                                    f"{boss_t:g} mm boss")
        boss_r = r_c + nut_af / math.sqrt(3) + 2.5
        balls = [(r_c * math.cos(phi), r_c * math.sin(phi))
                 for phi in (math.radians(-90.0 + 360.0 / cl["plungers"] * i)
                             for i in range(cl["plungers"]))]
        arm_len = throw + ARM_W / 2
        boss = rod(boss_r, a_in, reach)
        bar = Box(ARM_T, arm_len, ARM_W).moved(Location((reach - ARM_T / 2, -arm_len / 2, 0.0)))
        holes = []
        for y, zz in balls:     # each nut's pocket from the boss's inner face: its floor
            #                     takes the spring's push (design review round 3's m2)
            holes += [rod(PLUNGER["thread"] / 2 + 0.2, a_in - 1, reach + 1, y, zz),
                      hex_x(nut_af + NUT_POCKET, a_in - 1, a_in + nut_h, y, zz)]
        down = Location((0.0, -throw, 0.0))            # the arm's end
        g_d, bolt_len, _, head_h, bolt_key = spec.bolt
        arm = difference(union([boss, bar]), rod(jr + 0.2, a_in - 1, reach + 1), *holes,
                         rod(g_d / 2 + 0.3, reach - ARM_T - 1, reach + 1).moved(down))
        washer = ring_x(THRUST[1] / 2, THRUST[0] / 2, reach, cap0)
        cap_r = THRUST[1] / 2 + 1.5                     # over the thrust washer
        cap = difference(rod(cap_r, cap0, cap1),
                         rod(jr + 0.2, cap0 - 1, j1) - Box(j1 - cap0 + 2, 2 * jr + 1, 2 * jr)
                         .moved(Location(((cap0 + j1) / 2, 0.0, flat + 0.2 + jr))),
                         rod(1.7, j1 - 1, cap1 + 1))
        c_ins = (rod(ins_d / 2, j1 - ins_len, j1) - rod(1.5, j1 - ins_len - 1, j1 + 1))
        c_screw = union([rod(1.45, cap1 - t_len, cap1), rod(t_d / 2, cap1, cap1 + t_h)])

        # -- the grip on its bolt: washer, grip, washer, the head outside; washer and nyloc
        # on the arm's inner face
        grip_d, grip_len = spec.grip
        g_nut_af, g_nut_h, g_nut_key = GRIP_NUT
        gw = get(GRIP_WASHER).dims
        w_od, w_t, w_id = float(gw["od"]), float(gw["t"]), float(gw["id"])
        g0 = reach + w_t + 0.2
        g1 = g0 + grip_len
        bear = g1 + 0.2 + w_t                           # the bolt head's bearing face
        inner = reach - ARM_T
        nut0 = inner - w_t - g_nut_h
        if bear - bolt_len > nut0 - 1.0 or bear - bolt_len < nut0 - 6.0:
            raise ConstructionError(f"an M{g_d:g} x {bolt_len:g} grip bolt doesn't end 1 to 6 "
                                    "mm past its nut")
        shank = 0.97 * g_d / 2                          # (as fasteners.Screw.shank_d)
        grip = rod(grip_d / 2, g0, g1) - rod(g_d / 2 + 0.25, g0 - 1, g1 + 1)
        bolt = union([rod(shank, bear - bolt_len, bear), rod(g_nut_af / 2, bear, bear + head_h)])
        washers = [ring_x(w_od / 2, w_id / 2, x, x + w_t) for x in (reach, g1 + 0.2,
                                                                     inner - w_t)]
        nut = ring_x(g_nut_af / 2, g_d / 2, nut0, nut0 + g_nut_h)

        def hw(name, part, key=None, fab="purchased", color=STEEL):
            return hardware(name, place(part), host, fab=fab, bom_key=key, color=color)

        bodies = [hardware("handle", place(hub), host, fab="printed", color=PRINT_COLOR),
                  hardware("handle_arm", place(arm), host, fab="printed", color=PRINT_COLOR),
                  hardware("handle_cap", place(cap), host, fab="printed", color=PRINT_COLOR),
                  hardware("handle_grip", place(grip.moved(down)), host, fab="printed",
                           color="#2b2b2b"),
                  hardware("handle_seat_ring", place(seat_ring), host, fab="laser",
                           sheet=sheet, color="#9aa3ad"),
                  hw("handle_insert", insert, ins_key, color="#b08d57"),
                  hw("handle_thumb_screw", thumb, f"m3_thumb_screw_{t_len:g}"),
                  hw("handle_cap_insert", c_ins, ins_key, color="#b08d57"),
                  hw("handle_cap_screw", c_screw, f"m3_thumb_screw_{t_len:g}"),
                  hw("handle_thrust_washer", washer, THRUST[3]),
                  hw("handle_grip_pin", bolt.moved(down), bolt_key),
                  hw("handle_grip_nut", nut.moved(down), g_nut_key)]
        bodies += [hw(f"handle_grip_washer{i}", w.moved(down), GRIP_WASHER)
                   for i, w in enumerate(washers)]
        if split:
            bodies += [hardware(split[0][0], place(split[0][1]), host, fab="printed",
                                color=PRINT_COLOR),
                       hw(split[1][0], split[1][1], split[1][2], color="#b08d57"),
                       hw(split[2][0], split[2][1], split[2][2])]
        for i, (y, zz) in enumerate(balls):
            plunger = union([rod(PLUNGER["thread"] / 2 * 0.97, x_front, x_back, y, zz),
                             Sphere(big_r).moved(Location((xb, y, zz)))])
            bodies += [hw(f"handle_plunger{i}", plunger, PLUNGER["key"]),
                       hw(f"handle_plunger_nut{i}",
                          difference(hex_x(nut_af, a_in, a_in + nut_h, y, zz),
                                     rod(PLUNGER["thread"] / 2, a_in - 1, a_in + nut_h + 1,
                                         y, zz)), nut_key),
                       hw(f"handle_plunger_jam{i}",
                          difference(hex_x(jam_af, reach, reach + jam_h, y, zz),
                                     rod(PLUNGER["thread"] / 2, reach - 1, reach + jam_h + 1,
                                         y, zz)), jam_key)]
        thumb_r = math.hypot(t_top + t_h, t_d / 2)
        sweep = [(hub_r, -HUB_LEN, c0), (thumb_r, xt - t_d / 2, xt + t_d / 2),
                 (max(fl_r, boss_r), c0, reach),
                 (r_c + jam_af / math.sqrt(3) + 0.5, reach, x_back),
                 (throw + grip_d / 2 + 1.0, bear - bolt_len, bear + head_h),
                 (THRUST[1] / 2 + 1.5, reach, cap1 + t_h)]
        return bodies, sweep


def _len_of(key: str) -> float:
    """A NETRF12 standoff's length, from its catalog key."""
    return float(get(key).dims["length"])


@dataclass(frozen=True)
class HandCrankConstruction:
    """The ``"hand"`` entry of :data:`construction.CRANKS`: what the rest of the code asks of
    a crank construction by key (the rider's bore, what the crank holds); the group is
    :class:`HandCrank`."""

    key: str = "hand"
    label: str = ("hand crank: an NRV050 worm box, a crank block of 0.125 in plates keyed "
                  "on its shaft, a steel standoff crankpin and a printed offset hand crank "
                  "with a spring-plunger slip clutch")
    gearbox: str = NRV050.key
    output_shaft: str = "nrv050_output_shaft_25"
    lock_key: str | None = DRIVE_LOCK     # the studs', the block's, the end screw's
    crankpin: str = "pillar_shaft_12_m6"     # a NETRF12 made to length (the build's)
    plate_t: float = 3.175      # one of the block's crank-sheet plates (6061-T6)
    yield_mpa: float = 276.0    # 6061-T6
    bearing_mpa: float = 386.0  # its bearing yield (1.4 x yield, ASM)
    throw: float = 80.0         # the nominal crank (the build's own: notes["hand_crank"])

    @property
    def sheet_t(self) -> float:
        """The keyed plates' thickness: what bears on the key (and, a lower bound, what
        bends at the hub: the block over them is stiffer still)."""
        return KEYED_PLATES * self.plate_t

    def resolve(self, ctx) -> HandCrankConstruction:
        return self

    def for_sheet(self, key: str | None) -> HandCrankConstruction:
        """This crank's arm cut from sheet ``key`` (``None``: as it stands): its plates'
        thickness and yield (bearing yield 1.4 x)."""
        if key is None:
            return self
        from spiderpig.materials import sheet

        sh = sheet(key)
        return replace(self, plate_t=sh.thickness, yield_mpa=sh.yield_mpa,
                       bearing_mpa=1.4 * sh.yield_mpa)

    def rider_d(self, params) -> float:
        """The rod's bore: the crankpin standoff's."""
        return CRANKPIN_D

    def capacity(self, throw: float | None = None) -> dict[str, float]:
        """The crank torque (N·m) each element holds at yield, ``throw`` the crank's (the
        built one's, else the nominal): the arm bent at its hub by the crankpin's force
        across it (the torque over the throw; its width the arm's at the crankpin,
        :meth:`HandCrank.web_r`), and the arm's keyway bearing on the shaft's key (what the
        key stands proud of the shaft, the keyed plates' thickness, at the key's mean
        radius: the :data:`KEYED_PLATES` bearing together, their slots filed together on the
        key)."""
        box = BOXES[self.gearbox]
        r = throw or self.throw
        t = self.sheet_t
        w = 2 * ((CRANKPIN_D + SOCKET_FIT) / 2 + 2 * self.plate_t)
        z = t * w * w / 6
        hub = box.shaft_d / 2 + box.key_proud + 0.3 + 2 * self.plate_t
        arm = self.yield_mpa * z * r / max(r - hub, 1.0) / 1000.0
        return {"arm": round(arm, 2), "arm keyway": round(KEYED_PLATES * self.keyway_one(), 2)}

    def keyway_one(self) -> float:
        """The crank torque (N·m) one keyed plate's keyway holds at its bearing yield, on
        the sheet's thinnest (:data:`PLATE_UNDER` under its nominal): all the block holds
        until the keyed plates are filed together on the key (a laser's slot is +-0.127 mm in
        width and place: the tightest bears first; round 1's drive review; cut under the
        key's width since round 2, so each can be filed to it). Only the
        :data:`KEYED_PLATES` bear: the plates over them up to the seat are slotted
        :data:`KEY_CLEAR` wider than the key."""
        box = BOXES[self.gearbox]
        proud = box.key_proud
        t = max(self.plate_t - PLATE_UNDER, 0.0)
        return self.bearing_mpa * proud * t * (box.shaft_d / 2 + proud / 2) / 1000.0


HAND_CRANK = HandCrankConstruction()
