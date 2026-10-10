"""The hand-cranked worm drive of a decanting cradle (``config.crank == "hand"``).

A decanting cradle (:mod:`linkages.decanters`, ``docs/agentlib/DECANTER.md`` has the why) is
turned by hand, slowly, and must stay where it is let go, so its crank is driven through a
self-locking worm gear reducer instead of a servo:

* **The gearbox** (:class:`WormBox`, an NRV030 at 30:1) stands outboard of the drive side's
  *outer* frame plate, its output axis on O, its output face toward the plate. Its static
  efficiency at 30:1 is 0.39, under the 0.5 the maker's catalogue gives for static
  irreversibility: a load on the cradle can't turn the worm. Its 9 mm input shaft points
  back (``input_angle_deg``) and carries the printed hand crank.
* **The riser** (:func:`riser_for`): laser-cut 0.125 in discs between the gearbox's face and
  the plate, the face's raised 55 mm spigot ring bearing on the first; four M6 button heads
  come up through the plate and the discs into the face's tapped holes. The discs set how
  far the gearbox's stock output shaft (14 mm, its collar on the hub's end, keyed 5 x 5 x
  20 ending 5 short of its end, an M6 end tap; 32.5 past the hub's end) reaches into the
  stack: the crank block's keyed plates on its key's straight flanks.
* **The crank block** (the arm): laser-cut 0.125 in 6061-T6 plates (the crank sheet: a 5.1 mm
  keyway is legal in it, not in 0.250 in), each the arm's whole outline, stacked on the shaft
  from its key to past its end: the lowest :data:`KEYED_PLATES` keyed over the key, the
  next bored round the shaft, the last two over its end, an M6 button head down through the
  one under the top plate into the shaft's end tap, its head in the top plate's hole. A
  printed sleeve under the block down to the shaft's collar. A NETRF10 standoff through
  the block at the arm's middle (the *tie*), screwed at both ends, makes the plates one
  rigid block and clamps them.
* **The crankpin**: a NETRF10 standoff (10 mm steel round) standing in the block from its
  lowest plate (an M5 button head up through that plate into it) to the rod and on: the
  rod turns on it over a printed spacer on the block, an M5 button head and a wide washer
  on its end over the rod. The rod rides just over the block, so the crankpin carries the
  rod's force a few millimetres off its socket (the 2026-10-10 review round 2: the arm had
  been on the key, the rod 21 mm over it, the crankpin a cantilever that long).
* **The hand crank**: printed, on the input shaft (keyed, an M3 set screw on the key), a
  tube :data:`REACH` out along the shaft to its arm, :data:`HANDLE_R` throw, so the arm
  turns behind every moving part of the stack; its grip a printed sleeve spinning on a
  NETRF10 standoff screwed into the arm.

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
from spiderpig.hardware.cradle_catalog import NETRF10_THREAD, netrf10
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.materials import sheet as sheet_of
from spiderpig.servos.mount import to_location
from spiderpig.shapes import Cut, Rect, Shape3D, difference, disc, moved, plate, ring, union
from spiderpig.stack import Claim, Disc, Layout, Pill, Placed, Unbuildable

GROUP = "crank"
STEEL = "#4a4a4a"
GEARBOX_COLOR = "#2f5d8a"       # an NRV030's RAL 5010 blue
PRINT_COLOR = "#6a4fc7"


@dataclass(frozen=True)
class WormBox:
    """A worm gear reducer with a hollow output and a plug-in output shaft (mm, N·m).

    The NRV030 (Motovario's VSF catalogue rev 0 2017; RS's NMRV030 datasheet 4001782;
    Phidgets' TRM4400_0 and TRM4004_0 drawings, read 2026-10-10): the output face 4 x M6 x 11
    on a 65 mm PCD at 45°, a 55 mm h8 spigot, the faces 29 mm each side of the worm's plane
    (58 across), the output hub 63 long (2.5 past each face), the worm 30 off the output
    axis, the 9 j6 input shaft (3 mm key) 20 long past its shoulder 45 from the output axis.
    The single output shaft (14 h6 x 102, M6 x 15 centre tap) goes in from the plate's side:
    a 20 x 2.5 collar against the hub's end, then 30 more (B1 = 32.5 past the hub's end, the
    catalogue's "low speed shafts" row), keyed 5 x 5 x 20 ending 5 short of its end (the
    shaft drawing). Mesh data at 30:1: lead 4°52', static efficiency 0.39 (statically
    irreversible under 0.5), 0.66 at 1400 rpm; Mn2 21 N·m at 1140 rpm input (Phidgets rates
    it 20 N·m, 1/2° backlash)."""

    key: str = "nrv030_30"
    ratio: int = 30
    eta_static: float = 0.39
    eta_dynamic: float = 0.66       # the catalogue's, at 1400 rpm: a bound on the hand's
    rated_nm: float = 20.0
    pcd: float = 65.0
    face_thread: float = 6.0
    spigot_d: float = 55.0
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
    input_face: float = 45.0        # output axis to the input shaft's shoulder
    cap_face: float = 51.0          # output axis to the worm's far end (the VS's shoulder)
    below: float = 40.0             # output axis to the face away from the worm

    def irreversible(self) -> bool:
        """Static irreversibility, the maker's criterion: static efficiency under 0.5."""
        return self.eta_static < 0.5


NRV030 = WormBox()



CRANKPIN_D = 10.0       # the crankpin and the block's tie: NETRF10 standoffs (10 mm steel round,
#                         M5 x 10 deep each end, made to length: hardware.cradle_catalog)
SOCKET_FIT = 0.3        # the block's holes over them (the standoff is 0/-0.1 mm; a laser's hole
#                         +-0.13: thirteen of them must pass it)
CRANKPIN_PLAY = 0.3     # the rod's axial play on the crankpin, under its washer, at most (the
#                         standoff's length rounds down: 0.2-0.3): the washer holds it off the
#                         rear tie's screw head under the cradle link (0.26 mm nominal where it
#                         crosses, under the chain's +-0.3: the guide's paper check); seven
#                         plates' +-0.11 mm lie in that loop: a printed spacer under the rod,
#                         sanded on the bench, sets it (the rod only goes down)
TIE_SHORT = 0.85        # the tie under the top plate: shorter than its six plates at their
#                         thinnest (+-0.11 mm each), so its screws always clamp them
TIE_R = 47.0            # the block's tie standoff, this far out along the arm: its screw head
#                         and wide washer under the block clear the riser screws' heads (on
#                         the gearbox's 65 mm PCD) and its hole the crankpin's
HEADS_OVER = 3.3 + 1.5  # over the block: the end screw's M6 head (the tie's M5 is lower) and
#                         1.5 mm under the rod (the stack's tolerance: round 3 review)
GUARD_PAST = 20.0       # the outer plate is solid this far past the crank's sweep (a guard)


GRIP_SOCKET = 3.0               # the grip's standoff stands this deep in the hand crank's arm
GRIP_HEAD_SINK = 3.0            # the M5 under it: its head this deep in the arm's inner face
M5 = screw("bhcs", "5")         # ISO 7380 M5 button heads: the standoffs' ends
M6 = screw("bhcs", "6")         # ISO 7380 M6 button heads: the riser's, the shaft's end
SPIGOT_H = 2.5                  # the gearbox's raised 55 mm spigot ring (assumed: measure it)
FACE_DEPTH = 11.0               # the gearbox's face holes' thread depth
HEAD_LAYER_T = 7.0              # the layer over the outer plate: the riser screws' heads and
#                                 the block's standoffs' screw heads under it, with room to
#                                 run (it also sets the keyed plates on the key's flanks)
HEAD_LAYER = 1
ARM_LAYER = 2
KEYED_PLATES = 3                # the block's keyed plates: three crank-sheet plates (0.125 in
#                                 6061 each: the key's 5.1 mm slot is legal in it, not in a
#                                 0.250 in plate), all on the key's straight flanks
SLEEVE_D = 24.0                 # the printed sleeve and spacer on the shaft (14.2 bore)
ARM_BORE_FIT = 0.25             # the arm plates' bore over the 14 h6 shaft: over a laser's
#                                 +-0.13 mm (the key drives; the end cap centres the arm)
RISER_SHEET = ("al5052_3p2mm", 3.175)    # the riser's discs: the frame's sheet
HANDLE_R = 60.0                 # the hand crank's throw (mm)
REACH = 100.0                   # its printed tube from the input shaft's end to its arm: the
#                                 arm turns behind every moving part of the stack (it crosses
#                                 the stack's side there, where nothing moves), clear of the
#                                 outer plate (2026-10-10 review: a crank on the input shaft
#                                 itself swept through the crank arm)
ARM_T, ARM_W = 10.0, 18.0       # the printed crank arm: along the shaft, across
GRIP = (20.0, 25.0)             # its printed grip: d, length (on a 10 mm NETRF10)
HUB_R, HUB_LEN = 12.0, 18.0     # the printed crank's hub on the 9 mm input shaft
INPUT_KEY, INPUT_KEY_HUB = 3.0, 1.4   # the input shaft's key (3 x 3, DIN 6885) and how far
#                                       it stands into the hub (its keyway's depth, t2)
HANDLE_SET = 6.0                # the M3 cup-point set screw on that key


@dataclass(frozen=True)
class Riser:
    """The riser: the discs' sheets (gearbox side first), their height, and where the
    output shaft then is (side z): its hub's end, its key's span and its end."""

    discs: tuple[str, ...]
    height: float
    hub_end: float
    key: tuple[float, float]
    shaft_end: float
    screw: float


def riser_for(z0: float, a0: float, a1: float, box: WormBox = NRV030,
              t_plate: float = 3.175) -> Riser:
    """The riser that puts the shaft's key under the arm (``a0``..``a1``, side z; ``z0`` the
    outer plate's outer face): the whole arm on the key's straight flanks (the shaft's
    keyseat is end-milled, so the key's ends are round: half its width each), the shaft's
    end within 1.5 mm under the arm's top or above it, a stock M6 button head through the
    plate and the discs into the face's tapped holes, the stack over the arm least (its end
    as low as those allow), then the fewest discs. :class:`ConstructionError` if no stack
    of up to four discs does."""
    best = None
    t_arm = a1 - a0
    key_w, _, key_len = box.shaft_key
    why: list[str] = []
    for n in range(1, 5):
        height = n * RISER_SHEET[1]
        face = z0 - height - SPIGOT_H
        hub = face + box.hub_out
        end = hub + box.shaft_out
        key = (end - box.key_short - key_len, end - box.key_short)
        flat = min(a1, key[1] - key_w / 2) - max(a0, key[0] + key_w / 2)
        if flat < t_arm - 0.2 or end < a1 - 1.5:
            why.append(f"{n}: the key's flanks under {max(flat, 0.0):.1f} of the arm's "
                       f"{t_arm:.1f} mm" if flat < t_arm - 0.2 else
                       f"{n}: the shaft's end {a1 - end:.1f} mm under the arm's top")
            continue
        # the screws through the plate, the discs and the spigot's gap: at least 1.2 d into
        # the face's tapped holes and short of their bottom
        through = t_plate + height + SPIGOT_H
        screw = next((L for L in M6.lengths
                      if 1.2 * box.face_thread <= L - through <= FACE_DEPTH - 0.5), None)
        if screw is None:
            why.append(f"{n}: no stock M6 through {through:.1f} mm engages "
                       f"{1.2 * box.face_thread:.1f} mm (1.2 d) short of the tapped holes' "
                       f"{FACE_DEPTH - 0.5:g} mm")
            continue
        cand = (max(end, a1), n, Riser((RISER_SHEET[0],) * n, round(height, 3), hub, key, end,
                                       screw))
        if best is None or cand[:2] < best[:2]:
            best = cand
    if best is None:
        raise ConstructionError(
            f"no riser of up to four 0.125 in discs puts the gearbox's shaft key's straight "
            f"flanks under an arm at z {a0:.1f}-{a1:.1f} mm with a stock M6 screw into the "
            f"face (discs {'; '.join(why)}): the frame and crank sheets set those heights "
            "(the cradle's defaults build)")
    return best[2]


def arm_z(z0: float, t_frame: float, t_arm: float) -> tuple[float, float]:
    """The keyed plates' z (side): on the layer over the riser screws' heads (fixed by the
    frame plate and the head layer alone, so the search's and the plan's layouts agree)."""
    a0 = z0 + t_frame + HEAD_LAYER_T
    return a0, a0 + t_arm


def block_need(riser: Riser, t: float) -> float:
    """How high the crank block must reach (side z): past the shaft's end with room, and a
    plate (``t`` thick) over it for the end screw's head to bear on."""
    return riser.shaft_end + 0.5 + t


class HandCrank(Group):
    """The decanting cradle's crank: the worm gearbox on its riser, the keyed arm and its
    spacers and end cap, the crankpin and the hand crank (this module's docstring)."""

    name = GROUP
    stack_only = True           # the contract checks only what is inside the stack

    def __init__(self, box: WormBox = NRV030, input_angle_deg: float = 0.0):
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
        return self.box.shaft_d / 2 + 2.3 + 2 * ctx.sheet_t("crank")

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
        the keyed ones up to the z it must reach), side z."""
        z0 = L.z(0)[0]
        t = ctx.sheet_t("crank")
        a0, a1 = arm_z(z0, ctx.sheet_t("frame"), KEYED_PLATES * t)
        riser = riser_for(z0, a0, a1, self.box, ctx.sheet_t("frame"))
        n = max(2, math.ceil((block_need(riser, t) - a1) / t - 1e-6))
        return a0, a1, riser, a1 + n * t

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
        at_o = SLEEVE_D / 2 + m
        over = get("m5_washer_wide").dims["od"] / 2 + 0.3   # their washers over it

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
            out = [Placed(0, Disc("O", SLEEVE_D / 2 + 1.0), GROUP, "sleeve", seat=True)]
            out += [Placed(0, Disc(n, (self.box.face_thread + 0.6) / 2), GROUP,
                           "riser screw", seat=True) for n in names]
            out.append(Placed(0, Disc("O", at_o), GROUP, "sleeve", gap=True))
            out += [Placed(HEAD_LAYER, Disc(n, head_r), GROUP, "riser screw head",
                           sheet=HEAD_LAYER_T) for n in names]
            out += both(HEAD_LAYER, Disc("O", at_o), "sleeve")
            # the block's standoffs' screw heads under it (the tie's on a wide washer)
            out += both(HEAD_LAYER, Pill("O", pin, over), "block screw heads")
            for k in ks:                            # (the keyed plates: one thick layer)
                out.append(Placed(k, Disc("O", hub), GROUP, "crank block hub",
                                  sheet=KEYED_PLATES * t if k == ARM_LAYER else 0.0))
                out.append(Placed(k, Disc("O", hub), GROUP, "crank block hub", gap=True))
                out += both(k, Pill("O", pin, web), "crank block")
            # over it: the end screw's head at O, the tie's head, the crankpin's spacer (the
            # rod rides over these)
            for k in hs:
                out += both(k, Disc("O", head_r), "end screw head")
                out += both(k, Pill("O", pin, CRANKPIN_D / 2 + 2.0),
                            "tie screw and crankpin spacer")
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
            if length < 2 * NETRF10_THREAD:
                raise Unbuildable(f"the crankpin standoff would be {length:.1f} mm, under the "
                                  f"{2 * NETRF10_THREAD:g} its two M5 threads need")
            out = []
            for layer in range(max(ks + hs) + 1, r):     # its printed spacer up to the rod
                out += both(layer, Disc(pin, CRANKPIN_D / 2 + 2.0), "crankpin spacer")
            out.append(Placed(r, Disc(pin, CRANKPIN_D / 2), GROUP, "crankpin", seat=True))
            head_top = rod_top + CRANKPIN_PLAY + 1.2 + M5.head_h + 0.3
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
        face = build.z(0)[0] - riser.height - SPIGOT_H
        return end, d, face - self.box.width / 2

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
        face = z0 - riser.height - SPIGOT_H          # the gearbox's output face

        # the outer plate: the sleeve's hole, the screws' holes, solid round them and over the
        # whole crank (a guard: no finger reaches the block or the rod from outside)
        screws = self.screw_points(build)
        out.cut(FRAME_OUTER, Cut(tuple(o), SLEEVE_D + 2.0))
        for xy in screws:
            out.cut(FRAME_OUTER, Cut(xy, box.face_thread + 0.6))
        pad = box.pcd / 2 + (box.face_thread + 0.6) / 2 + 2 * ctx.sheet_t("frame") + 0.5
        reach = float(np.linalg.norm(mxy - o)) + self.web_r(ctx) + GUARD_PAST
        out.pad(FRAME_OUTER, tuple(o), tuple(o), max(pad, reach))

        # the riser's discs, the gearbox, its screws (outboard: the frame's)
        z = z0
        for i, key in enumerate(reversed(riser.discs)):         # from the plate outward
            t = sheet_of(key).thickness
            r_out = box.pcd / 2 + (box.face_thread + 0.6) / 2 + 2 * t + 0.5
            part = plate([], z - t, z, [Cut(tuple(o), SLEEVE_D + 2.0),
                                        *[Cut(xy, box.face_thread + 0.6) for xy in screws]],
                         discs=[(tuple(o), r_out)])
            out.bodies.append(hardware(f"riser_disc{i}", part, frame, fab="laser", sheet=key,
                                       color="#9aa3ad"))
            z -= t
        out.bodies.append(hardware("gearbox", self._gearbox(o, face, screws), frame,
                                   fab="purchased", bom_key=box.key, color=GEARBOX_COLOR))
        for i, xy in enumerate(screws):
            out.bodies.append(hardware(f"gearbox_screw{i}", screw_solid(xy, M6, z1, riser.screw,
                                                                         up=False), frame,
                                       fab="purchased", bom_key=M6.key(riser.screw),
                                       color=STEEL))

        # the output shaft (in from the plate's side: its collar on the hub's end, its key
        # out of its seat), the printed sleeve under the block, slotted for the key
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
        bore = box.shaft_d + 0.2
        r_slot = box.shaft_d / 2 + box.key_proud + 0.5
        slot = Box(r_slot, k_w + 0.3, a0 - riser.hub_end - col_t + 2.0).moved(Location(
            (float(o[0] + kc[0] * r_slot / 2), float(o[1] + kc[1] * r_slot / 2),
             (riser.hub_end + col_t + a0) / 2), (0, 0, math.degrees(key_ang))))
        out.bodies.append(hardware("crank_sleeve", difference(
            ring(tuple(o), SLEEVE_D, bore, riser.hub_end + col_t, a0), slot), crank,
            fab="printed", color=PRINT_COLOR))

        # the crank block: one plate a layer from the keyed plates' to its need, each the
        # arm's outline; holes round the shaft (keyed over the key), then for the end screw
        t = ctx.sheet_t("crank")
        a0, a1, riser, block_top = self.layout_z(ctx, build.plan.layout)
        zs = [a0 + i * t for i in range(round((block_top - a0) / t))]
        if zs[-1] < riser.shaft_end + 0.3:
            raise ConstructionError("the crank block's end screw has no plate over the "
                                    "shaft's end to bear on")
        throw = float(np.linalg.norm(mxy - o))
        tie = o + (mxy - o) * (TIE_R / throw)
        w_r = get("m5_washer_wide").dims["od"] / 2
        if TIE_R - w_r - (box.pcd / 2 + M6.head_d / 2) < 0.5 or throw - TIE_R < CRANKPIN_D + \
                SOCKET_FIT + 2 * t:
            raise ConstructionError(f"the crank block's tie at {TIE_R:g} mm meets the riser "
                                    f"screws' heads or the crankpin ({throw:.1f} mm out)")
        key_slot = Rect(tuple(o + kc * (box.shaft_d / 2 + 1.0)), (4.6, k_w + 0.1), key_ang)
        socket, through = CRANKPIN_D + SOCKET_FIT, M5.shank_d + 0.5
        for i, z_lo in enumerate(zs):
            # the crankpin stands on the lowest plate, the tie runs through all but the top
            pins = [Cut(tuple(mxy), through if i == 0 else socket),
                    Cut(tuple(tie), through if i == len(zs) - 1 else socket)]
            if z_lo < riser.shaft_end - 1e-6:       # round the shaft: keyed (the key's
                at_o = [Cut(tuple(o), box.shaft_d + ARM_BORE_FIT), key_slot]   # end in some)
            else:                                   # over its end: the end screw through
                at_o = [Cut(tuple(o), M6.d + 0.6)]
            part = plate([(tuple(o), tuple(mxy), self.web_r(ctx))], z_lo, z_lo + t,
                         [*at_o, *pins], discs=[(tuple(o), self.hub_r(ctx))])
            name = crank if i == 0 else f"{crank}_plate{i}"
            out.bodies.append(hardware(name, part, crank, fab="laser", sheet=ctx.sheet("crank"),
                                       color="#9aa3ad"))
        seat = block_top                    # the end screw's head on the top plate
        reach = seat - riser.shaft_end
        end_len = next((L for L in M6.lengths
                        if FACE_DEPTH - 2.0 <= L - reach <= 15.0 - 0.5), None)
        if end_len is None:
            raise ConstructionError(f"no stock M6 screw reaches the shaft's end tap {reach:.1f} "
                                    "mm under the crank block's top plate")
        out.bodies.append(hardware("crank_end_screw", screw_solid(tuple(o), M6, seat, end_len,
                                                                   up=False),
                                   crank, fab="purchased", bom_key=M6.key(end_len),
                                   color=STEEL))

        # the standoffs: the tie through the block from under its lowest plate to under its
        # top one (an M5 up through a wide washer into its foot, one down through the top
        # plate into its head: they clamp the block), the crankpin from the lowest plate (an
        # M5 up through it) through the rod (a wide washer and an M5 over the rod)
        washer = get("m5_washer_wide").dims
        w_t, w_od = float(washer["t"]), float(washer["od"])

        def standoff(xy, z_lo: float, z_hi: float, up: bool) -> tuple[Shape3D, str]:
            """A NETRF10 from ``z_lo`` to ``z_hi``, its length in 0.1 mm steps (``up``: at
            least that long, else at most)."""
            tenths = (z_hi - z_lo) * 10
            length = (math.ceil(tenths - 1e-6) if up else math.floor(tenths + 1e-6)) / 10
            bore = M5.shank_d / 2 + 0.05            # (its tapped ends, the screws in them)
            part = difference(disc(xy, CRANKPIN_D / 2, z_lo, z_lo + length),
                              disc(xy, bore, z_lo - 1, z_lo + NETRF10_THREAD),
                              disc(xy, bore, z_lo + length - NETRF10_THREAD, z_lo + length + 1))
            return part, netrf10(length)

        def m5(grip: float) -> float:
            """The shortest stock M5 of 10 mm or more (one length to buy) through ``grip`` mm
            with 5 mm to a standoff's 10 mm thread."""
            return next(L for L in M5.lengths
                        if L >= 10.0 and 5.0 <= L - grip <= NETRF10_THREAD)

        def add(name: str, part, key: str):
            out.bodies.append(hardware(name, part, crank, fab="purchased", bom_key=key,
                                       color=STEEL))

        tie_lo, tie_hi = a0, zs[-1] - TIE_SHORT
        tie_part, tie_key = standoff(tuple(tie), tie_lo, tie_hi, up=False)
        add("crank_tie", tie_part, tie_key)
        add("crank_tie_washer", ring(tuple(tie), w_od, M5.shank_d + 0.3, a0 - w_t, a0),
            "m5_washer_wide")
        lo = m5(w_t + 0.1)
        add("crank_tie_screw_lo", screw_solid(tuple(tie), M5, a0 - w_t, lo, up=True),
            M5.key(lo))
        hi = m5(block_top - (tie_lo + _len_of(tie_key)))   # the top plate and the gap
        add("crank_tie_screw_hi", screw_solid(tuple(tie), M5, block_top, hi, up=False),
            M5.key(hi))

        r0 = build.z(build.layers[rider])[0]
        rod_top = r0 + ctx.sheet_t("link", rider)
        pin_part, pin_key = standoff(tuple(mxy), a0 + t, rod_top + CRANKPIN_PLAY, up=False)
        out.bodies.append(hardware("crankpin", pin_part, crank, fab="purchased",
                                   bom_key=pin_key, color=STEEL))
        pin_top = a0 + t + _len_of(pin_key)
        lo = m5(t)
        add("crankpin_screw_lo", screw_solid(tuple(mxy), M5, a0, lo, up=True), M5.key(lo))
        add("crankpin_washer", ring(tuple(mxy), w_od, M5.shank_d + 0.3, pin_top, pin_top + w_t),
            "m5_washer_wide")
        hi = m5(w_t)
        add("crankpin_screw_hi", screw_solid(tuple(mxy), M5, pin_top + w_t, hi, up=False),
            M5.key(hi))
        if r0 - block_top > 0.3:
            out.bodies.append(hardware("crankpin_spacer",
                                       ring(tuple(mxy), CRANKPIN_D + 4.0, CRANKPIN_D + 0.3,
                                            block_top, r0 - 0.1),
                                       crank, fab="printed", color=PRINT_COLOR))
        out.cut(rider, Cut(tuple(mxy), p.hole(CRANKPIN_D)))

        # the hand crank on the input shaft, its arm behind every moving part of the stack
        end, d, zh = self.handle_axis(build, riser)
        out.bodies += self._handle(end, d, zh, frame)
        out.notes["hand_crank"] = {
            "gearbox": box.key, "ratio": box.ratio, "riser_discs": list(riser.discs),
            "riser_mm": riser.height, "riser_screw_mm": riser.screw,
            "arm_z_mm": [round(a0, 3), round(block_top, 3)],
            "keyed_z_mm": [round(a0, 3), round(a1, 3)],
            "block_plates": len(zs), "end_screw_mm": end_len,
            "key_z_mm": [round(riser.key[0], 3), round(riser.key[1], 3)],
            "shaft_end_z_mm": round(riser.shaft_end, 3),
            "sleeve_mm": round(a0 - riser.hub_end, 2),
            "crankpin_mm": _len_of(pin_key), "crankpin_top_z_mm": round(pin_top, 3),
            "tie_mm": _len_of(tie_key),
            "crankpin_lever_mm": round(r0 + ctx.sheet_t("link", rider) / 2 - block_top, 3),
            "handle": {"end_xy": [round(float(v), 3) for v in end],
                       "dir": [round(float(v), 6) for v in d], "axis_z": round(zh, 3),
                       "reach_mm": REACH, "arm_t_mm": ARM_T, "hub_r_mm": HUB_R,
                       "hub_len_mm": HUB_LEN,
                       "r_mm": HANDLE_R + GRIP[0] / 2 + 1.0, "grip_r_mm": HANDLE_R,
                       "grip_len_mm": GRIP[1] + 1.2 + M5.head_h},
            "throw_mm": round(float(np.linalg.norm(mxy - o)), 3),
            "irreversible": box.irreversible(),
        }
        return out

    def assembly(self, view) -> list:
        """How the drive goes on (:mod:`construction.assembly`, a cradle's
        ``CRADLE_ORDER``): on the bench the shaft through the gearbox from the face's side
        and the riser discs on the face; the outer plate over them and the four screws down
        into the face (their heads end up inside the stack, so first); the slotted sleeve;
        the crank block from its lowest plate (the two standoffs screwed to it on the bench)
        up, the tie's top screw and the end screw; the crankpin's spacer before the rod, its
        washer and screw once the rod is on; the hand crank last."""
        from spiderpig.construction.assembly import FINISH, STACK, Op, whole

        if "gearbox" not in view.z:
            return []
        topo = getattr(view.ctx, "topo", None)
        arm = next(iter(topo.crank_bodies)) if topo is not None else "conn"
        rider = HandCrank.rider(view.ctx) if view.ctx is not None else None
        k_rod = view.slot(view.z[rider][0]) if rider in view.z else ARM_LAYER + 1
        discs = view.named(r"riser_disc\d+")
        screws = view.named(r"gearbox_screw\d+")
        plates = view.named(rf"{re.escape(arm)}_plate\d+")
        k = self.box.shaft_key
        ops = [
            Op(STACK, (-1, -1, "gearbox"), whole("gearbox", "crank_shaft", *discs),
               "Gearbox on its riser",
               "Measure the gearbox first: its output face's raised spigot ring (2.5 mm drawn: "
               "under 2 mm, the riser screws reach too deep: an M6 washer under each head, "
               "from a hardware store) and where the output shaft's key sits. The output "
               "shaft into the gearbox's hollow output "
               f"from the face's side, collar first against the hub, its {k[0]:g} x {k[1]:g} "
               "key in its seat (the snap ring and washer on its far end); the gearbox on its "
               "back, face up, on a block with a hole for the shaft's far end (it stands about "
               f"6 mm proud); the riser disc{'s' if len(discs) > 1 else ''} on the face's "
               "raised spigot ring, the holes over the face's tapped holes.", "gearbox",
               sub=True),
            Op(STACK, (-1, 2), whole(*screws), "",
               "The outer plate over the discs, its outside down, the shaft up through its "
               f"bore; the {len(screws)} M6 button heads down through the plate and the "
               "discs into the gearbox's face (threadlocker), tight. The gearbox now "
               "stands the plate up off the bench.", "gearbox_screws"),
            Op(STACK, (HEAD_LAYER, 0), whole("crank_sleeve"), "",
               "The printed sleeve (printed solid: the block clamps on it) down over the "
               "shaft, its slot over the key, onto the shaft's collar.", "layer"),
            Op(STACK, (ARM_LAYER, 0), whole(arm, "crank_tie", "crank_tie_washer",
                                            "crank_tie_screw_lo", "crankpin",
                                            "crankpin_screw_lo"),
               "Crank block",
               "On the bench, the crank's lowest plate (keyed, one small hole at its end): "
               "the long steel standoff (the crankpin) stood on its small hole, an M5 button "
               "head up through the plate into it (threadlocker), tight; the short one (the "
               "tie) down through the plate's middle hole to its wide washer, an M5 button "
               "head up through the washer into it (threadlocker), snug: the washer, not the "
               "plate, holds the tie, so tape the washer to the plate until the top plate's "
               "screw is in. That plate down over "
               "the shaft, its keyway over the key, the standoffs up, onto the sleeve (file a "
               "keyway that binds: the key must slide).", "layer"),
            Op(STACK, (ARM_LAYER, 1), whole(*plates, "crank_tie_screw_hi", "crank_end_screw"),
               "",
               "The other crank plates down over the shaft and both standoffs: the keyed ones, "
               "the one with a 6.6 mm hole at the shaft on top (its small hole over the tie); "
               "an M5 button head down through the top plate into the tie (threadlocker), "
               "tight: it clamps the block; the M6 button head down into the shaft's end tap, "
               "dry, to 3 N·m (it clamps the block on the printed sleeve, which settles: after "
               "the first few pours take it out, threadlocker on it, and back to 3 N·m). Turn "
               "the gearbox's 9 mm input shaft by hand until the block has gone a whole turn "
               "(30 turns: the worm can't be turned from the block): nothing may touch the "
               "outer plate.", "layer"),
            Op(STACK, (ARM_LAYER, 2), whole("crankpin_spacer"), "",
               "The crankpin's printed spacer down over the crankpin onto the block.",
               "layer"),
            Op(STACK, (k_rod, 2), whole("crankpin_washer", "crankpin_screw_hi"), "",
               "With the rod on the crankpin: the wide washer and the M5 button head into "
               "the crankpin's top (threadlocker), tight. The rod must turn freely under the "
               "washer with a little end play (about 0.3 mm): if it binds, sand "
               "its printed spacer down (never shim it up: the washer keeps it off the screw "
               "heads over it).", "layer"),
            Op(FINISH, (2,), whole(*view.named(r"handle(_\w+)?")),
               "Hand crank",
               "The grip's steel standoff into the socket in the printed crank's arm, an M5 "
               "button head up through the arm's inner face into it (threadlocker); the "
               "printed grip over the standoff, the wide washer and an M5 button head on its "
               "end (threadlocker): the grip spins on the standoff. The crank's hub onto the "
               "gearbox's input shaft, its keyway over the shaft's 3 mm key (it comes with "
               "the gearbox: else a 3 x 3 x 16 DIN 6885 key), and the M3 set screw down onto "
               "the key. Then the rod's check: from rest (the cradle neck up) turn the handle "
               "4 turns one way and, if the rear tie's screw head under the drive side's "
               "cradle link never passed over the rod, back to rest and 4 turns the other "
               "way. Where the head is over the rod, press the rod up against its washer and "
               "slide a sheet of paper between them: it must pass. If it drags, take the "
               "crankpin's top screw and washer off, file 0.1 mm off the crankpin's top end, "
               "sand the rod's spacer to keep its play, and check again.", "handle"),
        ]
        ops = [replace(op, pieces=tuple(p for p in op.pieces if p.name in view.z))
               for op in ops]               # (a block that needs no spacer has none)
        return [op for op in ops if op.pieces]

    def motion(self, got: Realized) -> Motion:
        """Each part rides its host: the arm, the shaft, its sleeve, spacer and cap and the
        crankpin turn with the crank; the gearbox, its riser and screws and the handle stand
        with the frame (the handle is drawn at rest)."""
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
        """The NRV030 drawn from its dimensions (enough for clearances and the look), its
        face's tapped holes at ``holes`` (world XY) and its hub bored for the shaft."""
        b = self.box
        body = Box(80.0, b.below + 45.0, b.width).moved(
            Location((0.0, (45.0 - b.below) / 2, -b.width / 2)))
        worm = Cylinder(22.0, b.input_face + b.cap_face, rotation=(0, 90, 0)).moved(
            Location(((b.input_face - b.cap_face) / 2, b.worm_offset, -b.width / 2)))
        shaft = Cylinder(b.input_d / 2, b.input_len, rotation=(0, 90, 0)).moved(
            Location((b.input_face + b.input_len / 2, b.worm_offset, -b.width / 2)))
        spigot = Cylinder(b.spigot_d / 2, SPIGOT_H).moved(Location((0.0, 0.0, SPIGOT_H / 2)))
        hub = Cylinder(b.hub_d / 2, b.width + 2 * b.hub_out).moved(
            Location((0.0, 0.0, -b.width / 2)))
        local = union([body, worm, shaft, spigot, hub])
        part = moved(local, to_location(self._frame_of(o, z_face)))
        cut = [disc(tuple(o), b.shaft_d / 2 + 0.1, z_face - b.width - 10, z_face + 10)]
        cut += [disc(xy, b.face_thread / 2, z_face - FACE_DEPTH, z_face + 3) for xy in holes]
        return difference(part, *cut)

    def _handle(self, end, d, z, host) -> list:
        """The printed hand crank on the input shaft: its hub keyed over the shaft's 3 mm key
        (an M3 set screw down onto the key in a printed pilot), a tube :data:`REACH` on
        along the shaft and the arm there, :data:`HANDLE_R` out; its grip a printed sleeve
        spinning on a NETRF10 (the drive's other standoffs' stock: no shoulder screw to buy
        elsewhere) in a socket in the arm. Drawn at rest, the arm down: built along +x from
        the shaft's end, then turned to ``d`` and moved to ``end`` (side z ``z``)."""
        along_x = (0.0, 90.0, 0.0)                      # a cylinder's axis along +x
        a = math.degrees(math.atan2(d[1], d[0]))

        def place(part):
            return part.rotate(Axis.Z, a).moved(Location((float(end[0]), float(end[1]), z)))

        def rod(r: float, x0: float, x1: float):
            return Cylinder(r, x1 - x0, rotation=along_x).moved(Location(((x0 + x1) / 2, 0, 0)))

        hub = rod(HUB_R, -HUB_LEN, REACH)
        t_arm = REACH - ARM_T / 2                       # the arm's middle, along the shaft
        arm_len = HANDLE_R + ARM_W / 2
        arm = Box(ARM_T, arm_len, ARM_W).moved(Location((t_arm, -arm_len / 2, 0.0)))
        bore = rod(self.box.input_d / 2 + 0.15, -HUB_LEN - 0.5, 0.5)
        key_top = self.box.input_d / 2 + INPUT_KEY_HUB      # the key's top, over the axis
        keyway = Box(HUB_LEN + 2.0, INPUT_KEY + 0.2, key_top + 0.2).moved(
            Location((-HUB_LEN / 2, 0.0, (key_top + 0.2) / 2)))
        pilot = Cylinder(1.3, HUB_R - key_top + 2.0).moved(
            Location((-HUB_LEN / 2, 0.0, (HUB_R + key_top + 2.0) / 2)))
        set_screw = Cylinder(1.25, HANDLE_SET).moved(
            Location((-HUB_LEN / 2, 0.0, key_top + 0.05 + HANDLE_SET / 2)))
        # the grip at the arm's end, out along the shaft: a printed sleeve spinning on a
        # NETRF10 standing in a socket in the arm's outer face, an M5 up through the arm into
        # it (its head in a counterbore in the arm's inner face), a wide washer and an M5 on
        # its other end
        down = Location((0.0, -HANDLE_R, 0.0))         # the arm's end
        g0 = REACH + 0.2                              # the grip from the arm's outer face
        grip = rod(GRIP[0] / 2, g0, g0 + GRIP[1] - 0.4) - rod(CRANKPIN_D / 2 + 0.25, g0 - 1,
                                                               g0 + GRIP[1] + 1)
        inner = REACH - ARM_T                         # the arm's inner face
        s0 = REACH - GRIP_SOCKET                      # the standoff's foot, in its socket
        s_len = math.ceil((g0 + GRIP[1] - s0) * 10 - 1e-6) / 10     # 0.2 of play past the grip
        washer = get("m5_washer_wide").dims
        w_t, w_od = float(washer["t"]), float(washer["od"])
        bore_m5 = M5.shank_d / 2 + 0.05
        standoff = difference(rod(CRANKPIN_D / 2, s0, s0 + s_len),
                              rod(bore_m5, s0 - 1, s0 + NETRF10_THREAD),
                              rod(bore_m5, s0 + s_len - NETRF10_THREAD, s0 + s_len + 1))
        seat = inner + GRIP_HEAD_SINK                 # the inner screw's head's seat
        lo_len = next(L for L in M5.lengths if L - (s0 - seat) >= 5.0)
        hi_len = next(L for L in M5.lengths if L >= 10.0 and L - w_t >= 5.0)
        screw_lo = union([rod(M5.head_d / 2, seat - M5.head_h, seat),
                          rod(M5.shank_d / 2, seat, seat + lo_len)])
        top = s0 + s_len
        ring_w = rod(w_od / 2, top, top + w_t) - rod(M5.shank_d / 2 + 0.3, top - 1, top + w_t + 1)
        screw_hi = union([rod(M5.head_d / 2, top + w_t, top + w_t + M5.head_h),
                          rod(M5.shank_d / 2, top + w_t - hi_len, top + w_t)])
        hole = rod(M5.shank_d / 2 + 0.3, inner - 1, REACH + 1)
        socket = rod(CRANKPIN_D / 2 + 0.1, s0, REACH + 1)
        sink = rod(M5.head_d / 2 + 0.5, inner - 1, seat)
        handle = difference(union([hub, arm]), bore, keyway, pilot, hole.moved(down),
                            socket.moved(down), sink.moved(down))
        return [hardware("handle", place(handle), host, fab="printed", color=PRINT_COLOR),
                hardware("handle_set_screw", place(set_screw), host, fab="purchased",
                         bom_key=f"m3_set_screw_{HANDLE_SET:g}", color=STEEL),
                hardware("handle_grip", place(grip.moved(down)), host, fab="printed",
                         color="#2b2b2b"),
                hardware("handle_grip_pin", place(standoff.moved(down)), host,
                         fab="purchased", bom_key=netrf10(s_len), color=STEEL),
                hardware("handle_grip_screw", place(screw_lo.moved(down)), host,
                         fab="purchased", bom_key=M5.key(lo_len), color=STEEL),
                hardware("handle_grip_washer", place(ring_w.moved(down)), host,
                         fab="purchased", bom_key="m5_washer_wide", color=STEEL),
                hardware("handle_grip_cap", place(screw_hi.moved(down)), host,
                         fab="purchased", bom_key=M5.key(hi_len), color=STEEL)]


def _len_of(key: str) -> float:
    """A NETRF10 standoff's length, from its catalog key."""
    return float(get(key).dims["length"])


@dataclass(frozen=True)
class HandCrankConstruction:
    """The ``"hand"`` entry of :data:`construction.CRANKS`: what the rest of the code asks of
    a crank construction by key (the rider's bore, what the crank holds); the group is
    :class:`HandCrank`."""

    key: str = "hand"
    label: str = ("hand crank: an NRV030 worm box, a crank block of 0.125 in plates keyed "
                  "on its shaft, a steel standoff crankpin and a printed offset hand crank")
    gearbox: str = NRV030.key
    output_shaft: str = "nrv030_output_shaft_14"
    lock_key: str | None = "threadlocker_243"     # the block's, the end screw's, the grip's
    crankpin: str = "pillar_shaft_10_m5"     # a NETRF10 made to length (the build's)
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
        :meth:`HandCrank.web_r`), and the arm's keyway bearing on the shaft's key (the key's
        2 mm proud of the shaft, the arm's thickness, at the key's mean radius)."""
        r = throw or self.throw
        t = self.sheet_t
        w = 2 * ((CRANKPIN_D + SOCKET_FIT) / 2 + 2 * self.plate_t)
        z = t * w * w / 6
        hub = NRV030.shaft_d / 2 + 2.3 + 2 * self.plate_t
        arm = self.yield_mpa * z * r / max(r - hub, 1.0) / 1000.0
        proud = 2.0
        key = self.bearing_mpa * proud * t * (NRV030.shaft_d / 2 + proud / 2) / 1000.0
        return {"arm": round(arm, 2), "arm keyway": round(key, 2)}


HAND_CRANK = HandCrankConstruction()
