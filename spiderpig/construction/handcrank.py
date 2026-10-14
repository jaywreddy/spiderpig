"""The hand-cranked worm drive of a decanting cradle (``config.crank == "hand"``).

A decanting cradle (:mod:`linkages.decanters`, ``docs/agentlib/DECANTER.md`` has the why) is
turned by hand, slowly, and must stay where it is let go, so its crank is driven through a
self-locking worm gear reducer instead of a servo:

* **The gearbox** (:class:`WormBox`, an NRV040 at 30:1: :data:`NRV040`) stands outboard of
  the drive side's *outer* frame plate, its output axis on O, its output face toward the
  plate. Its static efficiency at 30:1 is 0.45, under the 0.5 the maker's catalogue gives
  for static irreversibility: a load on the cradle can't turn the worm. Its 11 mm input
  shaft points back (``input_angle_deg``) and carries the printed hand crank.
* **The riser** (:func:`riser_for`): four round M6 spacers (:data:`SPACER`) on the face
  screws, bearing on the gearbox's machined output face outside its raised spigot ring; four
  M6 button heads come up through the plate and the spacers into the face's tapped holes.
  The spigot ring and the turning output hub's end stand :data:`RISER_CLEAR` or more under
  the plate whatever their heights (neither is dimensioned by any maker): only the machined
  face is in the stack-up (design review round 1, 2026-10-14: discs bored to the hub's
  drawn diameter had two assumed housing dimensions meeting at zero margin). The spacers
  set how far the gearbox's stock output shaft (19 mm, its collar on the hub's end, keyed
  6 x 6 ending 5 short of its end, an M6 end tap; 43 past the hub's end) reaches into the
  stack: the crank block's keyed plates on its key's straight flanks.
* **The crank block** (the arm): laser-cut 0.125 in 6061-T6 plates (the crank sheet: the
  key's 6.0 mm slot is legal in it), each the arm's whole outline, stacked on the shaft from
  its key to past its end: the lowest :data:`KEYED_PLATES` keyed over the key (cut at the
  key's width and filed together on it to a slide fit), the next bored round the shaft, the
  top one (the *seat*) on the shaft's steel end face over DIN 988 shim rings (:data:`SHIM`:
  the guide's go/no-go measurement sets them), an M6 button head down through it and the
  shims into the shaft's end tap at :data:`END_SCREW_NM`: steel and aluminium only in that
  clamp (round 1: a printed sleeve under the block had crept out of its preload). An M6
  button head through the whole block at the arm's middle (the *tie*) into a thin nut under
  its lowest plate makes the plates one rigid block.
* **The crankpin**: a NETRF12 standoff (12 mm steel round) standing in the block from its
  lowest plate (an M6 button head up through that plate into it) to the rod and on: the
  rod turns on it (greased) over a printed spacer on the block, an M6 button head and a
  wide washer on its end over the rod. The rod rides just over the block, so the crankpin
  carries the rod's force a few millimetres off its socket (the 2026-10-10 review round 2:
  the arm had been on the key, the rod 21 mm over it, the crankpin a cantilever that long).
* **The hand crank**: printed in two parts joined by a ball-detent slip clutch
  (:func:`clutch`). Its hub is keyed on the input shaft (an M3 thumb screw into a heat-set
  insert, down onto the key: it comes off by hand, :data:`HANDLE_THUMB`) and runs on as a
  tube :data:`REACH` out along the shaft to a flange and a journal; the arm turns on the
  journal, three steel balls on springs in it pressed into conical seats in the flange, so
  the arm slips round the tube past :data:`CLUTCH_SLIP_N` at the grip: no push at the grip
  puts more than ``config.PRODUCT_CLUTCH_NM`` on the crank (round 1's drive review). The
  arm, :data:`HANDLE_R` throw, turns behind every moving part of the stack; its grip a
  printed sleeve spinning on an M8 bolt through the arm (a nyloc nut).

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
from spiderpig.hardware.catalog import get
from spiderpig.hardware.cradle_catalog import NETRF12_THREAD, netrf12
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.servos.mount import to_location
from spiderpig.shapes import Cut, Rect, Shape3D, difference, disc, moved, plate, ring, union
from spiderpig.stack import Claim, Disc, Layout, Pill, Placed, Unbuildable

GROUP = "crank"
STEEL = "#4a4a4a"
GEARBOX_COLOR = "#2f5d8a"       # an NRV's RAL 5010 blue
PRINT_COLOR = "#6a4fc7"


@dataclass(frozen=True)
class WormBox:
    """A worm gear reducer with a hollow output and a plug-in output shaft (mm, N·m).

    The defaults are the NRV030 (Motovario's VSF catalogue rev 0 2017; RS's NMRV030
    datasheet 4001782; Phidgets' TRM4400_0 and TRM4004_0 drawings, read 2026-10-10): the
    output face 4 x M6 x 11 on a 65 mm PCD at 45°, a 55 mm h8 spigot, the faces 29 mm each
    side of the worm's plane (58 across), the output hub 63 long (2.5 past each face), the
    worm 30 off the output axis, the 9 j6 input shaft (3 mm key) 20 long past its shoulder
    45 from the output axis. The single output shaft (14 h6 x 102, M6 x 15 centre tap) goes
    in from the plate's side: a 20 x 2.5 collar against the hub's end, then 30 more (B1 =
    32.5 past the hub's end, the catalogue's "low speed shafts" row), keyed 5 x 5 x 20
    ending 5 short of its end (the shaft drawing). Mesh data at 30:1: lead 4°52', static
    efficiency 0.39 (statically irreversible under 0.5), 0.66 at 1400 rpm; Mn2 21 N·m at
    1140 rpm input (Phidgets rates it 20 N·m, 1/2° backlash). :data:`NRV040` is the size up
    the cradles carry a 6 L bottle on."""

    key: str = "nrv030_30"
    ratio: int = 30
    eta_static: float = 0.39
    eta_dynamic: float = 0.66       # the catalogue's, at 1400 rpm: a bound on the hand's
    eta_hand: float = 0.5           # at a hand's speed (about 60 rpm): between the two
    rated_nm: float = 20.0
    pcd: float = 65.0
    face_thread: float = 6.0
    spigot_d: float = 55.0
    spigot_h: float = 2.5           # its raised spigot ring (assumed: measure it)
    width: float = 58.0             # face to face, along the output axis
    hub_d: float = 25.0             # the output hub's end (assumed: not dimensioned)
    hub_out: float = 2.5
    shaft_d: float = 14.0
    shaft_out: float = 32.5         # the shaft's end past the hub's end (35 past the face)
    collar: tuple[float, float] = (20.0, 2.5)   # its collar against the hub's end: d, length
    shaft_key: tuple[float, float, float] = (5.0, 5.0, 20.0)
    key_short: float = 5.0          # the key ends this far short of the shaft's end
    key_proud: float = 2.0          # and stands this far out of it (a 3 mm keyseat)
    worm_offset: float = 30.0
    input_d: float = 9.0
    input_len: float = 20.0
    input_key: tuple[float, float] = (3.0, 1.4)   # its key (DIN 6885) and how deep the hand
    #                                               crank's hub's keyway is over the shaft (the
    #                                               key stands t1 - d proud: 1.2, NRV040 1.5)
    input_face: float = 45.0        # output axis to the input shaft's shoulder
    cap_face: float = 51.0          # output axis to the worm's far end (the VS's shoulder)
    below: float = 40.0             # output axis to the face away from the worm
    pass_d: float = 24.0            # what turns through the outer plate's bore (2 mm wider) and
    #                                 the layer over it: the output shaft and its key, with room
    crank_hub_r: float = 12.0       # the printed hand crank's hub on its input shaft

    def irreversible(self) -> bool:
        """Static irreversibility, the maker's criterion: static efficiency under 0.5."""
        return self.eta_static < 0.5

    def tie_r(self) -> float:
        """How far out along the arm the crank block's tie stands: its thin nut under the
        block (its corners) clear of the riser screws' heads on the PCD."""
        return math.ceil(self.pcd / 2 + M6.head_d / 2 + TIE_NUT[0] / math.sqrt(3) + 0.5)


NRV030 = WormBox()
NRV040 = WormBox(
    key="nrv040_30", eta_static=0.45, eta_dynamic=0.71, eta_hand=0.55, rated_nm=45.0,
    pcd=75.0, spigot_d=60.0, width=73.0, hub_d=30.0, shaft_d=19.0, shaft_out=43.0,
    collar=(25.0, 3.0), shaft_key=(6.0, 6.0, 30.0), key_proud=2.5, worm_offset=40.0,
    input_d=11.0, input_len=23.0, input_key=(4.0, 1.8), input_face=53.0, cap_face=60.0,
    below=50.0, pass_d=28.0, crank_hub_r=14.0)
"""The NRV040 at 30:1 the cradles are built on since they carry a 6 L bottle (2026-10-13):
Phidgets' TRM4401_0 (an NMRV040 pattern, 45 N·m, 11 mm input with a 4 mm key, 19 mm bore
with a 6 mm key, 1/2° backlash, phidgets.com/?prodid=869). Motovario's NRV040 (RS
datasheet 4002371, the VSF US catalogue rev 0 2017): the output face 4 x M6 x 11 on a 75 mm
PCD at 45°, a 60 mm h8 spigot, 73 mm face to face (36.5 each side of the worm's plane), the
hub 78 long (G1), the worm 40 off the output axis (I), the 11 j6 input 23 long past its
shoulder 53 from the output axis (G3), the far side's shoulder 60 (G2), 50 from the output
axis to the face away from the worm (H); 2.0 kg dry. Mesh data at 30:1: lead 6°29', static
efficiency 0.45 (statically irreversible, under 0.5), 0.71 at 1400 rpm; Mn2 469 in-lb
(53 N·m) at 1750 rpm (Phidgets rates it 45). The low-speed shaft (Phidgets' TRM4006_0, 19
mm): Motovario's 18 mm one's lengths (B1 43 past the hub's end, B 40 keyed), its collar
and key's length assumed (25 x 3, 6 x 6 x 30)."""
BOXES = {b.key: b for b in (NRV030, NRV040)}


CRANKPIN_D = 12.0       # the crankpin: a NETRF12 standoff (12 mm steel round, M6 x 12 deep each
#                         end, made to length: hardware.cradle_catalog); the 6 L's jam bends a
#                         10 mm one (SF 1.13, cradle_report)
SOCKET_FIT = 0.3        # the block's holes over it (the standoff is 0/-0.1 mm; a laser's hole
#                         +-0.13: seven of them must pass it)
CRANKPIN_PLAY = 0.3     # the rod's axial play on the crankpin, under its washer, at most (the
#                         standoff's length rounds down: 0.2-0.3): the washer holds it off the
#                         rear tie's screw head under the cradle link (0.26 mm nominal where it
#                         crosses, under the chain's +-0.3: the guide's paper check); seven
#                         plates' +-0.11 mm lie in that loop: a printed spacer under the rod,
#                         sanded on the bench, sets it (the rod only goes down)
TIE_NUT = (10.0, 3.2, "m6_thin_nut")    # the tie's thin nut under the block (DIN 439: across
#                                         flats, height, catalog key), in the head layer, the
#                                         bolt's end past it 2 mm or more over the outer plate
HEADS_OVER = 3.3 + 1.5  # over the block: the end screw's and the tie's M6 heads and 1.5 mm
#                         under the rod (the stack's tolerance: round 3 review)


PIN_WASHER = "m6_washer_wide"   # over the crankpin's end, holding the rod on
PIN_WASHER_T = 1.6
M5 = screw("bhcs", "5")         # ISO 7380 M5 button heads (the stand's)
M6 = screw("bhcs", "6")         # ISO 7380 M6 button heads: the riser's, the shaft's end, the tie
FACE_DEPTH = 11.0               # the gearbox's face holes' thread depth
HEAD_LAYER_T = 7.0              # the layer over the outer plate: the riser screws' heads and
#                                 the crankpin's screw head and the tie's nut under the block,
#                                 with room to run (it also sets the keyed plates on the key)
HEAD_LAYER = 1
ARM_LAYER = 2
KEYED_PLATES = 4                # the block's keyed plates: four crank-sheet plates (0.125 in
#                                 6061 each: the key's 6.0 mm slot is legal in it), all on the
#                                 key's straight flanks (a 6 L's jam: three held 1.5)
KEY_SLOT = 0.0                  # the keyed plates' slot over the key's width: cut at its 6.0 and
#                                 filed together on the key, the block tied, to a slide fit (round
#                                 1: a laser's +-0.13 mm in width and place let the tightest plate
#                                 bear alone, 33 N·m at yield: cradle_report's keyway rows)
ARM_BORE_FIT = 0.25             # the arm plates' bore over the h6 shaft: over a laser's
#                                 +-0.13 mm (the key drives; the seat plate's end screw and the
#                                 shaft's end hold the block along the axis)
SPACER = (13.0, 6.3, {11.0: "m6_spacer_al_11", 12.0: "m6_spacer_al_12",
                      13.0: "m6_spacer_al_13"})
"""The riser's round spacers (:func:`riser_for`, McMaster's 94669A aluminium spacers for M6):
their outside diameter (13 at most keeps 1 mm off the 60 mm spigot ring on the 75 mm PCD),
their bore, and each stock length's catalog key (``hardware.cradle_catalog``). The guide's
go/no-go swaps the drawn length for its neighbour, 1 mm either way."""
RISER_CLEAR = 3.0               # the spacers hold the plate this far over the face's raised
#                                 spigot ring and the turning hub's end at least (each drawn 2.5
#                                 mm, neither measured: a hub standing proud of a disc's face
#                                 would have been braked by the four screws' preload)
SHIM = (0.2, 8, "shim_din988_6x12_0p2")
"""The DIN 988 6 x 12 shim rings bought for the seat: their thickness, how many (one line, up
to 1.6 mm in 0.2 mm steps) and catalog key. On the shaft's end under the crank block's top
plate: as many as the guide's measurement says."""
SHIM_D = (6.0, 12.0)
END_SCREW_NM = 8.0              # the shaft's end screw: an M6 button head through the seat plate
#                                 and the shims into the shaft's end tap, threadlocker (243)
HANDLE_R = 100.0                # the hand crank's throw (mm): a full 6 L's worst turn about 25 N
#                                 at the grip (cradle_report: the drive)
HANDLE_RS: dict[str, float] = {"decanter_hinge": 105.0}
"""Per cradle, its handle's throw where it isn't :data:`HANDLE_R`: A's worst turn takes more."""
REACH = 100.0                   # its printed tube from the input shaft's end to its arm's outer
#                                 face: the arm turns behind every moving part of the stack (it
#                                 crosses the stack's side there, where nothing moves), clear of
#                                 the outer plate (2026-10-10 review: a crank on the input shaft
#                                 itself swept through the crank arm)
ARM_T, ARM_W = 10.0, 18.0       # the printed crank arm's bar: along the shaft, across
GRIP = (24.0, 60.0)             # its printed grip: d, length (round 1: a 25 mm knob was a pinch
#                                 grip; a hand wants 60 or more)
HUB_LEN = 21.0                  # the printed crank's hub on the input shaft (23 long), printed
#                                 solid; its radius the box's crank_hub_r
HUB_BORE = 11.1                 # its bore, printed and reamed with an 11 mm drill (round 1: 11.3
#                                 let the grip tilt +-2 mm)
HANDLE_THUMB = (9.0, 8.0, 5.0)  # the knurled-head M3 thumb screw down onto that key (McMaster
#                                 92545A115, 18-8): under its head, its head d and h; through a
#                                 heat-set insert in the hub to the key, its head 2 mm off the hub
#                                 (2026-10-13: the hand crank comes off with no tool, as a
#                                 cradle packs; round 1: brass threads, not the print's, as it is
#                                 undone at every pack)
INSERT = (4.0, 5.7, "m3_heat_set_insert")   # an M3 heat-set insert: its hole's d, its length
#                                             and catalog key (the hub's, the journal's end's)
CLUTCH_SLIP_N = 31.5
"""The grip force (N, at the grip's middle) the hand crank's slip clutch is set to slip at on
the bench (:func:`clutch`): 1.1 x a sealed full 6 L's worst starting turn (25.4 N on either
cradle, 2026-10-14) at the low end of its :data:`CLUTCH_SCATTER`."""
CLUTCH_SCATTER = 0.10           # how far its slip drifts either way once set (a luggage scale's
#                                 reading, the springs' set, the seats' wear; dry, never greased)
CLUTCH_MARGIN = 1.1             # its least slip over the worst starting turn's
CLUTCH = {"balls": 3, "seats": 6, "ball_d": 6.0, "r": 20.0, "flange_r": 26.0, "flange_t": 6.0,
          "boss_t": 32.0, "journal_r": 8.0, "gap": 0.3, "seat_d": 5.0}
"""The clutch's geometry (mm): its balls in the arm's boss on a circle ``r`` round the axis,
pressed by springs (:data:`CLUTCH_SPRING`) into ``seats`` 90° conical seats ``seat_d`` across
in the tube's flange (``flange_r``, ``flange_t``), the boss ``boss_t`` long on the tube's
journal (``journal_r``), ``gap`` off the flange."""
CLUTCH_SPRING = (6.1, 20.65, 10.5, 14.07, 1.07, "compression_spring_clutch")
"""A clutch ball's spring (McMaster 9434K49, spring steel, closed and ground): outside d, free
length, rate (N/mm), its length at its 66.7 N maximum load, wire d, catalog key."""
SET_SCREW = (8.0, 8.0, "m8_set_screw_8")    # each spring's M8 cup-point set screw (d, length,
#                                             key): it self-taps into a 6.9 mm pilot, sets the slip
GRIP_BOLT = (8.0, 85.0, 22.0, 5.3, "m8_hex_bolt_85")   # the grip's pin: an M8 partially threaded
#                                                        hex bolt (d, length, thread, head, key)
GRIP_NUT = (13.0, 8.0, "m8_nyloc_nut")      # its nyloc on the arm's inner face (AF, h, key)
GRIP_WASHER = "m8_washer"                   # a DIN 125 washer each side of the grip and the arm


def handle_r(config) -> float:
    """The hand crank's throw for ``config`` (its linkage's :data:`HANDLE_RS` entry, else
    :data:`HANDLE_R`; a config-less context: the default)."""
    return HANDLE_RS.get(getattr(config, "linkage", ""), HANDLE_R)


@dataclass(frozen=True)
class Riser:
    """The riser: its spacers' length, and where the gearbox's machined output face and its
    output shaft then are (side z, the drawn housing's): its hub's end, its key's span and
    its end; the crank block's seat plate's underside over that end, the shims between them
    as drawn (``shim``) and the most the keyed plates allow (``shim_max``: the key's
    straight flanks)."""

    spacer: float
    face: float
    hub_end: float
    key: tuple[float, float]
    shaft_end: float
    screw: float
    seat: float
    shim: float
    shim_max: float


def riser_for(z0: float, a0: float, a1: float, t: float, box: WormBox = NRV040,
              t_plate: float = 3.175) -> Riser:
    """The riser that puts the shaft's key under the keyed plates (``a0``..``a1``, side z,
    plates ``t`` thick; ``z0`` the outer plate's outer face): a stock spacer length
    (:data:`SPACER`) that holds the plate :data:`RISER_CLEAR` over the spigot ring and the
    hub's end, a stock M6 button head through the plate and a spacer into the face's tapped
    holes, the block's seat plate (the first plate boundary at or over the shaft's end) on
    the shaft's end over shims, and the keyed plates on the key's straight flanks (the
    shaft's keyseat is end-milled, so the key's ends are round: half its width each) with as
    much shim under the seat as the guide allows: the length whose drawn shim leaves the
    most room either way (a hub standing higher or lower than drawn). :class:`
    ConstructionError` if none does."""
    best = None
    key_w, _, key_len = box.shaft_key
    why: list[str] = []
    for length in sorted(SPACER[2]):
        if length - max(box.spigot_h, box.hub_out) < RISER_CLEAR:
            why.append(f"{length:g}: under {RISER_CLEAR:g} mm over the spigot ring")
            continue
        face = z0 - length
        hub = face + box.hub_out
        end = hub + box.shaft_out
        key = (end - box.key_short - key_len, end - box.key_short)
        k = max(KEYED_PLATES + 1, math.ceil((end - a0) / t - 1e-6))
        seat = a0 + k * t
        shim = seat - end
        # the shaft as far under its seat as the shims reach: its key's straight flanks
        # still the keyed plates' whole height
        shim_max = min(SHIM[0] * SHIM[1], seat - box.key_short - key_w / 2 - (a1 - 0.2))
        if key[0] + key_w / 2 > a0 + 0.2 or shim > shim_max - 0.3:
            why.append(f"{length:g}: the key's straight flanks don't stay under the keyed "
                       f"plates (a drawn shim of {shim:.2f} mm, at most {shim_max:.2f})")
            continue
        # the screws through the plate and a spacer: at least 1.2 d into the face's tapped
        # holes and short of their bottom
        through = t_plate + length
        screw = next((L for L in M6.lengths
                      if 1.2 * box.face_thread <= L - through <= FACE_DEPTH - 0.5), None)
        if screw is None:
            why.append(f"{length:g}: no stock M6 through {through:.1f} mm engages "
                       f"{1.2 * box.face_thread:.1f} mm (1.2 d) short of the tapped holes' "
                       f"{FACE_DEPTH - 0.5:g} mm")
            continue
        cand = (-min(shim, shim_max - shim), Riser(length, face, hub, key, end, screw, seat,
                                                    round(shim, 3), round(shim_max, 3)))
        if best is None or cand[0] < best[0]:
            best = cand
    if best is None:
        raise ConstructionError(
            f"no stock spacer puts the gearbox's shaft key's straight flanks under keyed "
            f"plates at z {a0:.1f}-{a1:.1f} mm with a stock M6 screw into the face "
            f"({'; '.join(why)}): the frame and crank sheets set those heights (the cradle's "
            "defaults build)")
    return best[1]


def arm_z(z0: float, t_frame: float, t_arm: float) -> tuple[float, float]:
    """The keyed plates' z (side): on the layer over the riser screws' heads (fixed by the
    frame plate and the head layer alone, so the search's and the plan's layouts agree)."""
    a0 = z0 + t_frame + HEAD_LAYER_T
    return a0, a0 + t_arm


def shims_for(gap: float) -> tuple[float, ...]:
    """The :data:`SHIM` rings that fill ``gap`` to the ring below (the block then sits up to
    a ring's thickness low: the rod away from the screw head over it)."""
    t, most, _ = SHIM
    n = math.floor(gap / t + 1e-6)
    if n > most:
        raise ConstructionError(f"{gap:.2f} mm takes more than the {most} x {t:g} mm shims")
    return (t,) * max(n, 0)


def clutch(grip_r: float, ratio: int = 30, eta: float = 0.55) -> dict[str, float]:
    """The hand crank's slip clutch at a grip ``grip_r`` mm out: its slip torque at the
    input (N·m) as set (:data:`CLUTCH_SLIP_N` at the grip) and at either end of its
    :data:`CLUTCH_SCATTER`, the most it puts on the crank through the worm (``ratio``, at a
    hand's speed ``eta``: the crank torque no push at the grip exceeds,
    ``config.PRODUCT_CLUTCH_NM``), and each ball's spring force where it slips (at the top of
    its climb out of its 90° seat, a friction angle of 6°: ``spring_n``) and seated
    (``spring_set_n``, its spring that much less compressed: what the set screws are drawn
    at; the scale sets it in the end)."""
    set_nm = CLUTCH_SLIP_N * grip_r / 1000.0
    lo, hi = set_nm * (1 - CLUTCH_SCATTER), set_nm * (1 + CLUTCH_SCATTER)
    ramp = math.tan(math.radians(45.0 + 6.0))
    peak = set_nm * 1000.0 / (CLUTCH["balls"] * CLUTCH["r"] * ramp)
    return {"slip_nm": round(set_nm, 3), "slip_lo_nm": round(lo, 3), "slip_hi_nm": round(hi, 3),
            "crank_cap_nm": round(hi * ratio * eta, 1), "spring_n": round(peak, 1),
            "spring_set_n": round(peak - CLUTCH_SPRING[2] * clutch_climb(), 1)}


def clutch_climb() -> float:
    """How far a clutch ball rides into its 90° seat past the flange's face (mm): what it
    climbs, its spring compressed that much more, to slip."""
    return CLUTCH["seat_d"] / 2 - CLUTCH["ball_d"] / 2 * (math.sqrt(2) - 1)


class HandCrank(Group):
    """The decanting cradle's crank: the worm gearbox on its riser's spacers, the crank block
    keyed on its shaft and seated on its end, the crankpin and the hand crank with its slip
    clutch (this module's docstring)."""

    name = GROUP
    stack_only = True           # the contract checks only what is inside the stack

    def __init__(self, box: WormBox = NRV040, input_angle_deg: float = 0.0):
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

    def screw_points(self, ctx_or_build) -> list[tuple[float, float]]:
        """The gearbox's four face screws, world XY: on the PCD at 45° to the input shaft."""
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
        riser = riser_for(z0, a0, a1, t, self.box, ctx.sheet_t("frame"))
        return a0, a1, riser, riser.seat + t

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
        t = ctx.sheet_t("crank")
        head_r = M6.head_d / 2 + 0.3
        at_o = self.box.pass_d / 2 + m
        over = get(PIN_WASHER).dims["od"] / 2 + 0.3   # their washers over it

        def both(layer, shape, label, **kw):
            return [Placed(layer, shape, GROUP, label, **kw),
                    Placed(layer, shape, GROUP, label, gap=True)]

        def z_of(L: Layout, k: int) -> float:
            """Layer ``k``'s bottom: the plan's own z, or (while searching, every layer the
            pitch) the drive's thick layers under it and the pitch a layer over them: a lower
            bound of the plan's z (a thicker link layer or a clearance gap raises it), so
            the search may claim more than the plan would; the final layout's claims decide
            (a plan the search proves thinnest is thinnest among the layouts this estimate
            admits)."""
            if L.final or L.thick or L.gaps:
                return L.z(k)[0]
            a1 = arm_z(L.z(0)[0], ctx.sheet_t("frame"), KEYED_PLATES * t)[1]
            return a1 + (k - ARM_LAYER - 1) * ctx.pitch

        def block(L: Layout) -> tuple[list[int], list[int], float]:
            """The layers the block crosses (the keyed plates' and up to its top), the ones
            the heads over it cross (the end screw's, the tie's), and the heads' top."""
            top = self.layout_z(ctx, L)[3]
            heads = top + HEADS_OVER
            ks = [ARM_LAYER]
            while z_of(L, ks[-1] + 1) < top - 1e-6:
                ks.append(ks[-1] + 1)
                if ks[-1] >= L.top - 2:
                    raise Unbuildable("the crank block reaches the inner plate")
            hs = [ks[-1]] if z_of(L, ks[-1] + 1) > top + 1e-6 else []
            k = ks[-1] + 1
            while z_of(L, k) < heads - 1e-6 and k < L.top:
                hs.append(k)
                k += 1
            return ks, hs, heads

        def fixed(L: Layout):
            if L.top <= ARM_LAYER + 2:
                raise Unbuildable("the stack has no room for the crank block and the rod over it")
            a0 = self.layout_z(ctx, L)[0]
            if L.final and abs(L.z(ARM_LAYER)[0] - a0) > 1e-6:
                # (a link or a gap under the block: the riser is sized for its z)
                raise Unbuildable(f"the crank block's layer {ARM_LAYER} is at z "
                                  f"{L.z(ARM_LAYER)[0]:.2f}, not the {a0:.2f} over the plate "
                                  f"and the screw heads' layer")
            ks, hs, _ = block(L)
            out = [Placed(0, Disc("O", self.box.pass_d / 2 + 1.0), GROUP, "output shaft",
                          seat=True)]
            out += [Placed(0, Disc(n, (self.box.face_thread + 0.6) / 2), GROUP,
                           "riser screw", seat=True) for n in names]
            out.append(Placed(0, Disc("O", at_o), GROUP, "output shaft", gap=True))
            out += [Placed(HEAD_LAYER, Disc(n, head_r), GROUP, "riser screw head",
                           sheet=HEAD_LAYER_T) for n in names]
            out += both(HEAD_LAYER, Disc("O", at_o), "output shaft")
            # under the block: the crankpin's screw head, the tie's nut and its bolt's end
            out += both(HEAD_LAYER, Pill("O", pin, over), "block screw heads")
            for k in ks:                            # (the keyed plates: one thick layer)
                out.append(Placed(k, Disc("O", hub), GROUP, "crank block hub",
                                  sheet=KEYED_PLATES * t if k == ARM_LAYER else 0.0))
                out.append(Placed(k, Disc("O", hub), GROUP, "crank block hub", gap=True))
                out += both(k, Pill("O", pin, web), "crank block")
            # over it: the end screw's head at O, the tie bolt's head, the crankpin's spacer
            # (the rod rides over these)
            for k in hs:
                out += both(k, Disc("O", head_r), "end screw head")
                out += both(k, Pill("O", pin, CRANKPIN_D / 2 + 2.0),
                            "tie bolt and crankpin spacer")
            return out

        def crankpin(L: Layout):
            a0, _, _, _ = self.layout_z(ctx, L)
            ks, hs, heads = block(L)
            r = L.layers[rider]
            if r <= max(ks + hs) or z_of(L, r) < heads + 0.3 - 1e-6:
                raise Unbuildable(f"the rod {rider} must ride over the crank block and its "
                                  f"screws' heads (z {heads + 0.3:.1f} mm), not in layer {r}")
            rod_top = z_of(L, r) + ctx.sheet_t("link", rider)
            length = rod_top + CRANKPIN_PLAY - (a0 + t)
            if length < 2 * NETRF12_THREAD:
                raise Unbuildable(f"the crankpin standoff would be {length:.1f} mm, under the "
                                  f"{2 * NETRF12_THREAD:g} its two M6 threads need")
            out = []
            for layer in range(max(ks + hs) + 1, r):     # its printed spacer up to the rod
                out += both(layer, Disc(pin, CRANKPIN_D / 2 + 2.0), "crankpin spacer")
            out.append(Placed(r, Disc(pin, CRANKPIN_D / 2), GROUP, "crankpin", seat=True))
            head_top = rod_top + CRANKPIN_PLAY + PIN_WASHER_T + M6.head_h + 0.3
            hi = r + 1
            while hi < L.top and z_of(L, hi) < head_top - 1e-6:
                out += both(hi, Disc(pin, over), "crankpin washer and screw")
                hi += 1
            if L.final and z_of(L, hi) < head_top - 1e-6 and hi >= L.top:
                # (only at the plan's own z: a clearance gap under the plate may take it, and
                # the search's estimate has none, so it would be stricter than the plan)
                raise Unbuildable(f"the crankpin's screw over {rider} reaches the inner plate")
            out.append(Placed(r, Disc(pin, over), GROUP, "crankpin washer and screw",
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
        a0, a1, riser, _ = self.layout_z(ctx, build.plan.layout)
        if abs(build.z(ARM_LAYER)[0] - a0) > 1e-6:
            raise ConstructionError(f"the arm's layer is at z {build.z(ARM_LAYER)[0]:.2f}, "
                                    f"not the {a0:.2f} its riser is sized for")
        u = (mxy - o) / max(float(np.linalg.norm(mxy - o)), 1e-9)
        arm_ang = math.atan2(u[1], u[0])
        face = riser.face                                # the gearbox's machined output face

        # the outer plate: the output shaft's hole, the screws' holes, solid round them (no
        # guard over the crank: the user's 2026-10-10 call, a minimal mount; the crank turns
        # about 2 rpm behind the truss, and the use limit keeps fingers out of the drive side)
        screws = self.screw_points(build)
        out.cut(FRAME_OUTER, Cut(tuple(o), box.pass_d + 2.0))
        for xy in screws:
            out.cut(FRAME_OUTER, Cut(xy, box.face_thread + 0.6))
        pad = box.pcd / 2 + (box.face_thread + 0.6) / 2 + 2 * ctx.sheet_t("frame") + 0.5
        out.pad(FRAME_OUTER, tuple(o), tuple(o), pad)

        # the riser's spacers on the face screws, the gearbox, its screws (outboard: the
        # frame's); the spacers outside the spigot ring
        sp_od, sp_bore, sp_keys = SPACER
        if box.pcd / 2 - sp_od / 2 < box.spigot_d / 2 + 1.0 - 1e-9:
            raise ConstructionError(f"a {sp_od:g} mm riser spacer meets the {box.spigot_d:g} mm "
                                    "spigot ring")
        for i, xy in enumerate(screws):
            out.bodies.append(hardware(f"riser_spacer{i}",
                                       ring(xy, sp_od, sp_bore, face, z0), frame,
                                       fab="purchased", bom_key=sp_keys[riser.spacer],
                                       color=STEEL))
        out.bodies.append(hardware("gearbox", self._gearbox(o, face, screws), frame,
                                   fab="purchased", bom_key=box.key, color=GEARBOX_COLOR))
        for i, xy in enumerate(screws):
            out.bodies.append(hardware(f"gearbox_screw{i}", screw_solid(xy, M6, z1, riser.screw,
                                                                         up=False), frame,
                                       fab="purchased", bom_key=M6.key(riser.screw),
                                       color=STEEL))

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
        shaft = difference(union([
            disc(tuple(o), box.shaft_d / 2, riser.hub_end - 5.0, riser.shaft_end),
            disc(tuple(o), col_d / 2, riser.hub_end, riser.hub_end + col_t), key]),
            disc(tuple(o), 3.0, riser.shaft_end - 15.0, riser.shaft_end + 1.0))   # M6 x 15 tap
        out.bodies.append(hardware("crank_shaft", shaft, crank, fab="purchased",
                                   bom_key=HandCrankConstruction.output_shaft, color=STEEL))

        # the crank block: one plate a layer from the keyed plates' to its seat on the
        # shaft's end, each the arm's outline; holes round the shaft (keyed over the key),
        # then the seat plate's for the end screw
        t = ctx.sheet_t("crank")
        a0, a1, riser, block_top = self.layout_z(ctx, build.plan.layout)
        zs = [a0 + i * t for i in range(round((block_top - a0) / t))]
        if abs(zs[-1] - riser.seat) > 1e-6 or riser.seat < riser.shaft_end - 1e-6:
            raise ConstructionError("the crank block's top plate doesn't sit on the shaft's "
                                    "end")
        throw = float(np.linalg.norm(mxy - o))
        tie_r = box.tie_r()
        tie = o + (mxy - o) * (tie_r / throw)
        if tie_r - TIE_NUT[0] / math.sqrt(3) - (box.pcd / 2 + M6.head_d / 2) < 0.5 or \
                throw - tie_r < CRANKPIN_D + SOCKET_FIT + 2 * t:
            raise ConstructionError(f"the crank block's tie at {tie_r:g} mm meets the riser "
                                    f"screws' heads or the crankpin ({throw:.1f} mm out)")
        key_slot = Rect(tuple(o + kc * (box.shaft_d / 2 + box.key_proud / 2)),
                        (box.key_proud + 2.6, k_w + KEY_SLOT), key_ang)  # (1.3 under the key)
        for i, z_lo in enumerate(zs):
            # the crankpin stands on the lowest plate; the tie bolt runs through them all
            pins = [Cut(tuple(mxy), M6.shank_d + 0.5 if i == 0 else CRANKPIN_D + SOCKET_FIT),
                    Cut(tuple(tie), M6.d + 0.6)]
            if z_lo < riser.shaft_end - 1e-6:       # round the shaft: keyed (the key's
                at_o = [Cut(tuple(o), box.shaft_d + ARM_BORE_FIT), key_slot]   # end in some)
            else:                                   # the seat: the end screw through
                at_o = [Cut(tuple(o), M6.d + 0.6)]
            part = plate([(tuple(o), tuple(mxy), self.web_r(ctx))], z_lo, z_lo + t,
                         [*at_o, *pins], discs=[(tuple(o), self.hub_r(ctx))])
            name = crank if i == 0 else f"{crank}_plate{i}"
            out.bodies.append(hardware(name, part, crank, fab="laser", sheet=ctx.sheet("crank"),
                                       color="#9aa3ad"))

        def add(name: str, part, key: str):
            out.bodies.append(hardware(name, part, crank, fab="purchased", bom_key=key,
                                       color=STEEL))

        # the seat: shims on the shaft's steel end face under the top plate (drawn as the
        # drawn housing's gap; the guide's measurement sets them), the end screw through both
        if riser.shim > 0.05:
            add("crank_shims", ring(tuple(o), SHIM_D[1], SHIM_D[0], riser.shaft_end,
                                    riser.seat), SHIM[2])
        reach = block_top - riser.shaft_end
        end_len = next((L for L in M6.lengths
                        if FACE_DEPTH - 2.0 <= L - reach <= 15.0 - 0.5), None)
        if end_len is None:
            raise ConstructionError(f"no stock M6 screw reaches the shaft's end tap {reach:.1f} "
                                    "mm under the crank block's top plate")
        add("crank_end_screw", screw_solid(tuple(o), M6, block_top, end_len, up=False),
            M6.key(end_len))

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

        # the hand crank on the input shaft, its arm behind every moving part of the stack
        end, d, zh = self.handle_axis(build, riser)
        grip_r = handle_r(ctx.config)
        bodies, sweep = self._handle(end, d, zh, frame, grip_r)
        out.bodies += bodies
        out.notes["hand_crank"] = {
            "gearbox": box.key, "ratio": box.ratio, "riser_spacer_mm": riser.spacer,
            "riser_screw_mm": riser.screw, "face_z_mm": round(riser.face, 3),
            "plate_inner_z_mm": round(z1, 3),
            "arm_z_mm": [round(a0, 3), round(block_top, 3)],
            "keyed_z_mm": [round(a0, 3), round(a1, 3)],
            "block_plates": len(zs), "end_screw_mm": end_len,
            "key_z_mm": [round(riser.key[0], 3), round(riser.key[1], 3)],
            "shaft_end_z_mm": round(riser.shaft_end, 3),
            "seat_z_mm": round(riser.seat, 3), "shim_mm": riser.shim,
            "shim_max_mm": riser.shim_max, "shims_mm": list(shims_for(riser.shim)),
            # the guide's go/no-go: the shaft's end over the outer plate's inner face, the
            # gearbox on its spacers; and the seat plate's underside there
            "shaft_over_plate_mm": round(riser.shaft_end - z1, 2),
            "seat_over_plate_mm": round(riser.seat - z1, 2),
            "crankpin_mm": _len_of(pin_key), "crankpin_top_z_mm": round(pin_top, 3),
            "tie_bolt_mm": tie_len,
            "crankpin_lever_mm": round(r0 + ctx.sheet_t("link", rider) / 2 - block_top, 3),
            "handle": {"end_xy": [round(float(v), 3) for v in end],
                       "dir": [round(float(v), 6) for v in d], "axis_z": round(zh, 3),
                       "reach_mm": REACH, "grip_r_mm": grip_r,
                       "sweep": [[round(v, 3) for v in s] for s in sweep],
                       "clutch": clutch(grip_r, box.ratio, box.eta_hand)},
            "throw_mm": round(float(np.linalg.norm(mxy - o)), 3),
            "irreversible": box.irreversible(),
        }
        return out

    def assembly(self, view) -> list:
        """How the drive goes on (:mod:`construction.assembly`, a cradle's
        ``CRADLE_ORDER``): on the bench the shaft through the gearbox from the face's side
        and the riser's spacers on the face; the outer plate over them and the four screws
        down into the face (their heads end up inside the stack, so first), and the go/no-go
        measurement that sets the shims; the crank block built and fitted on the bench (its
        plates, the tie, the crankpin), then down over the shaft onto the shims and the end
        screw; the crankpin's spacer before the rod, its washer and screw once the rod is on;
        the hand crank last, its clutch set on the bench."""
        from spiderpig.construction.assembly import FINISH, STACK, Op, whole

        if "gearbox" not in view.z:
            return []
        topo = getattr(view.ctx, "topo", None)
        arm = next(iter(topo.crank_bodies)) if topo is not None else "conn"
        rider = HandCrank.rider(view.ctx) if view.ctx is not None else None
        k_rod = view.slot(view.z[rider][0]) if rider in view.z else ARM_LAYER + 1
        spacers = view.named(r"riser_spacer\d+")
        screws = view.named(r"gearbox_screw\d+")
        plates = view.named(rf"{re.escape(arm)}_plate\d+")
        k = self.box.shaft_key
        # the go/no-go: the shaft's end over the outer plate's inner face, and the seat
        # plate's underside there (the shims between, to the ring below)
        inner = view.layers[0][1]
        shaft = view.z["crank_shaft"][1] - inner if "crank_shaft" in view.z else 0.0
        seat = (view.z[plates[-1]][0] if plates else 0.0) - inner
        a1 = view.layers[ARM_LAYER][1] - inner
        t_shim, n_shim, _ = SHIM
        lo = seat - min(t_shim * n_shim, seat - self.box.key_short
                        - self.box.shaft_key[0] / 2 - (a1 - 0.2))
        shims = shims_for(seat - shaft) if seat > shaft else ()
        lengths = SPACER[2]
        length = next(iter(lengths))
        if spacers and view.keys.get(spacers[0]):
            length = next((L for L, key in lengths.items() if key == view.keys[spacers[0]]),
                          length)
        shorter = max((L for L in lengths if length > L), default=None)
        longer = min((L for L in lengths if length < L), default=None)
        ops = [
            Op(STACK, (-1, -1, "gearbox"), whole("gearbox", "crank_shaft", *spacers),
               "Gearbox on its riser",
               "Wipe the gearbox: a new worm box can weep oil at its seals. Run it in: a few "
               f"hundred turns of its {self.box.input_d:g} mm input shaft (a drill, slowly), "
               "then wipe it again; it stands over the base board beside the wine. The output "
               "shaft into the gearbox's hollow output from the face's side, collar first "
               f"against the hub, its {k[0]:g} x {k[1]:g} key in its seat (the snap ring and "
               "washer on its far end); the gearbox on its back, face up, on a block with a "
               "hole for the shaft's far end (it stands about 6 mm proud). A round spacer "
               f"({length:g} mm, aluminium) on each of the face's {len(spacers)} tapped holes, "
               "outside its raised spigot ring: they bear on the machined face, so the ring and "
               "the turning hub's end stand clear of the plate whatever their height.",
               "gearbox", sub=True),
            Op(STACK, (-1, 2), whole(*screws), "",
               "The outer plate over the spacers, its outside down, the shaft up through its "
               f"bore; the {len(screws)} M6 button heads down through the plate and the "
               "spacers into the gearbox's face (threadlocker), tight. The gearbox now stands "
               "the plate up off the bench. Go/no-go: measure the shaft's end over the plate's "
               f"inner face (a caliper's depth rod beside the shaft): {shaft:.2f} mm drawn. "
               f"The crank block's top plate will sit {seat:.2f} mm over that face on "
               f"{t_shim:g} mm shim rings (DIN 988, 6 x 12) on the shaft's end: {seat:.2f} "
               f"less your measurement, over {t_shim:g} and rounded down, is how many rings "
               f"to stack ({seat - shaft:.2f} drawn: {len(shims)}). Over {seat:.2f}: the "
               "shaft stands too high (its hub prouder than drawn)"
               + (f": {longer:g} mm spacers instead (the same family) and measure again"
                  if longer else ": an M6 washer under each spacer and measure again")
               + f". Under {lo:.2f}: the key's straight flanks would fall short of the keyed "
               "plates"
               + (f": {shorter:g} mm spacers instead (the same family) and measure again."
                  if shorter else ": file each spacer down the difference and measure again."),
               "gearbox_screws"),
            Op(STACK, (ARM_LAYER, 0), whole(arm, *plates, "crank_tie", "crank_tie_nut",
                                            "crankpin", "crankpin_screw_lo"),
               "Crank block",
               "On the bench, as one block: the lowest plate (keyed, a small hole at its end), "
               "the thick steel standoff (the crankpin) stood on that small hole, an M6 button "
               "head up through the plate into it (threadlocker), tight; the other plates down "
               "over the crankpin in their order (the keyed ones, then the round-bored ones, "
               "the one with a 6.6 mm hole at the shaft on top), the tie's M6 button head down "
               "through every plate's middle hole into its thin nut under the lowest plate "
               "(threadlocker; a 10 mm wrench on the nut), tight. Fit the keyways together: "
               f"the shaft's {k[0]:g} mm key, out of its seat, pushed through the four keyed "
               "plates' slots; file them together until it slides through the block with no "
               "play (a laser's slot is 0.13 mm out: until they are filed together the "
               "tightest plate carries the crank alone). The key back in its seat.", "layer"),
            Op(STACK, (ARM_LAYER, 1), whole("crank_shims", "crank_end_screw"), "",
               "The shim rings the go/no-go gave on the shaft's end, round its tapped hole (a "
               "dab of grease holds them); the block down over the shaft, its keyways on the "
               "key, its top plate onto the shims; the M6 button head down through it and the "
               f"shims into the shaft's end tap, threadlocker, {END_SCREW_NM:g} N·m (a "
               "torque wrench: steel and aluminium only in this clamp, so it holds its "
               "preload). The plate must sit on the shims, not on the screw: no shake along "
               f"the shaft. Turn the gearbox's {self.box.input_d:g} mm input shaft by hand "
               "until the block has gone a whole turn (30 turns: the worm can't be turned from "
               "the block): nothing may touch the outer plate.", "layer"),
            Op(STACK, (ARM_LAYER, 2), whole("crankpin_spacer"), "",
               "The crankpin's printed spacer down over the crankpin onto the block; grease "
               "the crankpin where the rod turns on it (aluminium on bare steel: any lithium "
               "or PTFE grease).", "layer"),
            Op(STACK, (k_rod, 2), whole("crankpin_washer", "crankpin_screw_hi"), "",
               "With the rod on the crankpin: the wide washer and the M6 button head into "
               "the crankpin's top (threadlocker), tight. The rod must turn freely under the "
               "washer with a little end play (about 0.3 mm): if it binds, sand "
               "its printed spacer down (never shim it up: the washer keeps it off the screw "
               "heads over it).", "layer"),
            Op(FINISH, (2,), whole(*view.named(r"handle(_\w+)?")),
               "Hand crank",
               "Print the hub (with its tube, flange and journal) solid, 100 % infill: it "
               "carries the input's key; ream its bore with an 11 mm drill by hand. Two M3 "
               "heat-set inserts, a soldering iron at about 220 °C: one down into the hub's "
               "hole over the keyway, one into the journal's end. The arm: a steel ball, its "
               "spring and an M8 set screw into each of its three bores from its outer face "
               "(dry: never grease the clutch), the set screws just started; the arm onto the "
               "journal, the balls toward the flange; the printed cap on the journal's end and "
               "an M3 thumb screw through it into the insert, tight. Set the clutch: the hub's "
               "tube in a vise (a rag round it), a luggage scale on the grip's middle pulling "
               "square to the arm, slowly: it must click round at "
               f"{CLUTCH_SLIP_N:g} N ({CLUTCH_SLIP_N / 9.81:.1f} kg on the scale). Under: "
               "turn each set screw a quarter turn in, the same each; over: out; pull again, "
               f"three times in a row within {CLUTCH_SLIP_N * CLUTCH_SCATTER:.0f} N of it. It "
               "is what keeps a hard push at the grip from breaking the cradle (it slips "
               "instead), so check it again yearly and after it has slipped hard. The grip: "
               f"an M{GRIP_BOLT[0]:g} washer, the printed grip, a washer and the arm's end "
               f"over the M{GRIP_BOLT[0]:g} bolt from the outside, a washer and the nyloc nut "
               "on the arm's inner face, tight against the arm: the grip spins on the bolt's "
               "plain shank. A drop "
               "of threadlocker or super glue in the gearbox's input keyseat and its 4 x 4 "
               "key pressed in (it comes with the gearbox; else a 4 x 4 x 16 DIN 6885 key), "
               "so it stays when the crank comes off. The crank's hub onto the input shaft, "
               "its keyway over the key, and the hub's M3 thumb screw down onto the key by "
               "hand (snug: it only keeps the hub on; it comes off the same way). Stand at the "
               "drive side's rear corner to turn it, the neck in view past the stand. "
               "Then the rod's check: from rest (the cradle neck up) turn the handle "
               "4 turns one way and, if the rear tie's screw head under the drive side's "
               "cradle link never passed over the rod, back to rest and 4 turns the other "
               "way. Where the head is over the rod, press the rod up against its washer and "
               "slide a sheet of paper between them: it must pass. If it drags, take the "
               "crankpin's top screw and washer off, file 0.1 mm off the crankpin's top end, "
               "sand the rod's spacer to keep its play, and check again. Then the worm's "
               "hold: with the heaviest full bottle you have (a 6 L at best) in the cradle, "
               "turn it to level, let go, tap the table and watch a pencil line across the "
               "crank's arm and the outer plate for 10 minutes: it must not move. If it creeps "
               "back toward rest, the worm holds less than drawn: an O-ring (about 12 x 2 mm) "
               "between the hub's face and the gearbox's input seal adds a little drag (1 to "
               "2 N at the grip) and doubles the margin; a bottle that creeps never tips "
               "further, it settles back neck up.", "handle"),
        ]
        ops = [replace(op, pieces=tuple(p for p in op.pieces if p.name in view.z))
               for op in ops]               # (a block that needs no spacer has none)
        return [op for op in ops if op.pieces]

    def motion(self, got: Realized) -> Motion:
        """Each part rides its host: the block's plates, the shaft, its shims, the tie and
        the end screw, the crankpin and its spacer turn with the crank; the gearbox, its
        spacers and screws and the handle stand with the frame (the handle is drawn at
        rest)."""
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
        body = Box(80.0 * k, b.below + 45.0 * k, b.width).moved(
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
        cut += [disc(xy, b.face_thread / 2, z_face - FACE_DEPTH, z_face + 3) for xy in holes]
        return difference(part, *cut)

    def _handle(self, end, d, z, host, throw: float = HANDLE_R
                ) -> tuple[list, list[tuple[float, float, float]]]:
        """The printed hand crank on the input shaft, drawn at rest (the arm down), and what
        its turning sweeps (``(r, x0, x1)``: cylinders round the shaft's axis, ``x`` along it
        from the shaft's end): built along +x from the shaft's end, then turned to ``d`` and
        moved to ``end`` (side z ``z``).

        * The hub (printed solid) on the shaft, keyed over its 4 x 4 key, an M3 thumb
          screw down onto the key through a heat-set insert (:data:`HANDLE_THUMB`), runs on
          as a tube :data:`REACH` out to the clutch's flange (a cone under it, so it prints
          hub down) and a journal through the arm's boss.
        * The arm turns on the journal, held by a printed cap and an M3 thumb screw into an
          insert in the journal's end; :data:`CLUTCH`'s balls in its boss, each on a spring
          and an M8 set screw, press into conical seats in the flange's face: it slips round
          the tube past :data:`CLUTCH_SLIP_N` at the grip.
        * The grip, a printed sleeve, spins on an M8 bolt's plain shank through the arm's
          end, a washer either side of it and a nyloc nut on the arm's inner face."""
        from build123d import Cone, Sphere

        along_x = (0.0, 90.0, 0.0)                      # a cylinder's axis along +x
        a = math.degrees(math.atan2(d[1], d[0]))
        cl = CLUTCH

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

        hub_r = self.box.crank_hub_r
        fl_r, fl_t, boss_t, jr, gap = (cl["flange_r"], cl["flange_t"], cl["boss_t"],
                                       cl["journal_r"], cl["gap"])
        a_in = REACH - boss_t                           # the arm's boss's inner face
        f1 = a_in - gap                                 # the flange's face
        f0 = f1 - fl_t
        c0 = f0 - (fl_r - hub_r)                        # a 45° cone under the flange
        j1 = REACH + 0.1                                # the journal's end, a hair proud
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

        hub = union([rod(hub_r, -HUB_LEN, c0 + 0.5), cone(hub_r, fl_r, c0, f0 + 0.01),
                     rod(fl_r, f0, f1), rod(jr, f1 - 1.0, j1)])
        bore = rod(HUB_BORE / 2, -HUB_LEN - 0.5, 0.5)
        keyway = Box(HUB_LEN + 2.0, key_w + 0.2, key_top + 0.2).moved(
            Location((-HUB_LEN / 2, 0.0, (key_top + 0.2) / 2)))
        hole = union([radial(ins_d / 2, ins0, hub_r + 1.0), radial(1.6, key_top - 0.5,
                                                                    ins0 + 0.5)])
        # the seats: 90° cones in the flange's face, the balls' circle; the journal's insert
        seats = [cone(0.0, cl["seat_d"] / 2 + 0.5, f1 - cl["seat_d"] / 2, f1 + 0.5,
                      cl["r"] * math.cos(phi), cl["r"] * math.sin(phi))
                 for phi in (math.radians(-90.0 + 360.0 / cl["seats"] * i)
                             for i in range(cl["seats"]))]
        j_hole = union([rod(ins_d / 2, j1 - ins_len, j1 + 1.0), rod(1.6, j1 - ins_len - 4.0,
                                                                     j1 - ins_len + 0.5)])
        hub = difference(hub, bore, keyway, hole, j_hole, *seats)
        insert = (Cylinder(ins_d / 2, ins_len) - Cylinder(1.5, ins_len + 2)).moved(
            Location((xt, 0.0, ins0 + ins_len / 2)))
        t_top = key_top + 0.05 + t_len               # the thumb screw's head's underside
        thumb = union([Cylinder(1.45, t_len).moved(Location((xt, 0.0, t_top - t_len / 2))),
                       Cylinder(t_d / 2, t_h).moved(Location((xt, 0.0, t_top + t_h / 2)))])

        # -- the arm on the journal: its boss with the balls, springs and set screws; the
        # bar out to the grip
        arm_len = throw + ARM_W / 2
        boss = rod(fl_r, a_in, REACH)
        bar = Box(ARM_T, arm_len, ARM_W).moved(Location((REACH - ARM_T / 2, -arm_len / 2, 0.0)))
        ball_r = cl["ball_d"] / 2
        xb = f1 - cl["seat_d"] / 2 + ball_r * math.sqrt(2) + 0.02   # a ball's centre, seated
        sp_od, sp_free, sp_k, sp_max_len, sp_wire, sp_key = CLUTCH_SPRING
        sc_d, sc_len, sc_key = SET_SCREW
        got = clutch(throw)
        f_s = got["spring_set_n"]
        s0 = xb + ball_r + (sp_free - f_s / sp_k)       # the set screw's inner end
        if s0 + sc_len > REACH - 0.5 or sp_free - got["spring_n"] / sp_k < sp_max_len + 0.3:
            raise ConstructionError(f"the clutch's springs ({f_s:.0f} N set) and set screws "
                                    f"don't fit the arm's {boss_t:g} mm boss")
        balls = [(cl["r"] * math.cos(phi), cl["r"] * math.sin(phi))
                 for phi in (math.radians(-90.0 + 360.0 / cl["balls"] * i)
                             for i in range(cl["balls"]))]
        lip = ball_r - 0.15                             # retains a ball off its seat
        bore_r, pilot_r = (sp_od + 0.3) / 2, (sc_d - 1.1) / 2   # (M8 self-taps a 6.9 pilot)
        bores = []
        for y, zz in balls:
            bores += [cone(lip, lip + 0.6, a_in - 0.01, a_in + 0.6, y, zz),
                      rod(bore_r, a_in + 0.6, s0, y, zz),
                      rod(pilot_r, s0 - 0.5, REACH + 1, y, zz)]
        down = Location((0.0, -throw, 0.0))            # the arm's end
        g_d, bolt_len, _, head_h, bolt_key = GRIP_BOLT
        arm = difference(union([boss, bar]), rod(jr + 0.2, a_in - 1, REACH + 1), *bores,
                         rod(g_d / 2 + 0.3, REACH - ARM_T - 1, REACH + 1).moved(down))
        cap = rod(jr + 4.0, j1, j1 + 3.0) - rod(1.7, j1 - 1, j1 + 4)
        c_ins = (rod(ins_d / 2, j1 - ins_len, j1) - rod(1.5, j1 - ins_len - 1, j1 + 1))
        c_screw = union([rod(1.45, j1 + 3.0 - t_len, j1 + 3.0), rod(t_d / 2, j1 + 3.0,
                                                                    j1 + 3.0 + t_h)])

        # -- the grip on its bolt: washer, grip, washer, the head outside; washer and nyloc
        # on the arm's inner face
        nut_af, nut_h, nut_key = GRIP_NUT
        washer = get(GRIP_WASHER).dims
        w_od, w_t, w_id = float(washer["od"]), float(washer["t"]), float(washer["id"])
        g0 = REACH + w_t + 0.2
        g1 = g0 + GRIP[1]
        bear = g1 + 0.2 + w_t                           # the bolt head's bearing face
        inner = REACH - ARM_T
        nut0 = inner - w_t - nut_h
        if bear - bolt_len > nut0 - 1.0 or bear - bolt_len < nut0 - 6.0:
            raise ConstructionError(f"an M{g_d:g} x {bolt_len:g} grip bolt doesn't end 1 to 6 "
                                    "mm past its nut")
        shank = 0.97 * g_d / 2                          # (as fasteners.Screw.shank_d)
        grip = rod(GRIP[0] / 2, g0, g1) - rod(g_d / 2 + 0.25, g0 - 1, g1 + 1)
        bolt = union([rod(shank, bear - bolt_len, bear), rod(nut_af / 2, bear, bear + head_h)])
        washers = [ring_x(w_od / 2, w_id / 2, x, x + w_t) for x in (REACH, g1 + 0.2,
                                                                     inner - w_t)]
        nut = ring_x(nut_af / 2, g_d / 2, nut0, nut0 + nut_h)

        def hw(name, part, key=None, fab="purchased", color=STEEL):
            return hardware(name, place(part), host, fab=fab, bom_key=key, color=color)

        bodies = [hardware("handle", place(hub), host, fab="printed", color=PRINT_COLOR),
                  hardware("handle_arm", place(arm), host, fab="printed", color=PRINT_COLOR),
                  hardware("handle_cap", place(cap), host, fab="printed", color=PRINT_COLOR),
                  hardware("handle_grip", place(grip.moved(down)), host, fab="printed",
                           color="#2b2b2b"),
                  hw("handle_insert", insert, ins_key, color="#b08d57"),
                  hw("handle_thumb_screw", thumb, f"m3_thumb_screw_{t_len:g}"),
                  hw("handle_cap_insert", c_ins, ins_key, color="#b08d57"),
                  hw("handle_cap_screw", c_screw, f"m3_thumb_screw_{t_len:g}"),
                  hw("handle_grip_pin", bolt.moved(down), bolt_key),
                  hw("handle_grip_nut", nut.moved(down), nut_key)]
        bodies += [hw(f"handle_grip_washer{i}", w.moved(down), GRIP_WASHER)
                   for i, w in enumerate(washers)]
        for i, (y, zz) in enumerate(balls):
            bodies += [hw(f"handle_ball{i}", Sphere(ball_r).moved(Location((xb, y, zz))),
                          "steel_ball_6"),
                       hw(f"handle_spring{i}", ring_x(sp_od / 2, sp_od / 2 - sp_wire,
                                                      xb + ball_r + 0.05, s0 - 0.05, y, zz),
                          sp_key),
                       hw(f"handle_set_screw{i}", rod(pilot_r - 0.05, s0, s0 + sc_len, y, zz),
                          sc_key)]
        thumb_r = math.hypot(t_top + t_h, t_d / 2)
        sweep = [(hub_r, -HUB_LEN, c0), (thumb_r, xt - t_d / 2, xt + t_d / 2),
                 (fl_r, c0, REACH), (throw + GRIP[0] / 2 + 1.0, bear - bolt_len, bear + head_h),
                 (jr + 4.0, REACH, j1 + 3.0 + t_h)]
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
    label: str = ("hand crank: an NRV040 worm box, a crank block of 0.125 in plates keyed "
                  "on its shaft, a steel standoff crankpin and a printed offset hand crank")
    gearbox: str = NRV040.key
    output_shaft: str = "nrv040_output_shaft_19"
    lock_key: str | None = "threadlocker_243"     # the block's, the end screw's, the grip's
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
        radius: the four bearing together, their slots filed together on the key)."""
        box = BOXES[self.gearbox]
        r = throw or self.throw
        t = self.sheet_t
        w = 2 * ((CRANKPIN_D + SOCKET_FIT) / 2 + 2 * self.plate_t)
        z = t * w * w / 6
        hub = box.shaft_d / 2 + box.key_proud + 0.3 + 2 * self.plate_t
        arm = self.yield_mpa * z * r / max(r - hub, 1.0) / 1000.0
        return {"arm": round(arm, 2), "arm keyway": round(KEYED_PLATES * self.keyway_one(), 2)}

    def keyway_one(self) -> float:
        """The crank torque (N·m) one keyed plate's keyway holds at its bearing yield: all
        the block holds until the four are filed together on the key (a laser's slot is
        +-0.13 mm in width and place: the tightest bears first; round 1's drive review)."""
        box = BOXES[self.gearbox]
        proud = box.key_proud
        return self.bearing_mpa * proud * self.plate_t * (box.shaft_d / 2 + proud / 2) / 1000.0


HAND_CRANK = HandCrankConstruction()
