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
  stack: the arm on its key, the end just over it.
* **The arm**: two laser-cut 0.125 in 6061-T6 plates (the crank sheet: a 5.1 mm keyway is
  legal in it, not in 0.250 in) on the shaft's key, a printed sleeve under it down to the
  shaft's collar and a printed spacer over it up to the shaft's end, both slotted for the
  key, all clamped by a laser-cut end cap and an M6 button head in the shaft's end tap.
* **The crankpin**: an ISO 7379 shoulder screw (:data:`CRANKPIN`: 10 mm shoulder, M8) down
  through the rod into the arm, an M8 jam nut under the arm in the head layer. The rod
  turns on the shoulder, a printed spacer under it, a printed washer over it where the
  stock shoulder is longer, the head over that.
* **The hand crank**: printed, on the input shaft (keyed, an M3 set screw on the key), a
  tube :data:`REACH` out along the shaft to its arm, :data:`HANDLE_R` throw, so the arm
  turns behind every moving part of the stack; its grip a printed sleeve on a shoulder
  screw.

The rod sweeps across the axis O once a turn (the crank-rocker's folded toggle), so nothing
of the drive may stand at O in the rod's layer: the planner puts the rod over the end cap.
The drive side alone carries all of this; the other side of the cradle has no crank
(:mod:`construction.cradle`).
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, replace

import numpy as np
from build123d import Box, Cylinder, Location

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



@dataclass(frozen=True)
class ShoulderScrew:
    """ISO 7379 shoulder screw: shoulder ``d`` x ``length``, thread ``thread`` x
    ``thread_len``, socket head ``head_d`` x ``head_h`` (McMaster 92981A's)."""

    d: float = 8.0
    thread: float = 6.0
    thread_len: float = 11.0
    head_d: float = 13.0
    head_h: float = 6.0
    lengths: tuple[float, ...] = (6, 8, 10, 12, 16, 20, 25)

    family: str = "m6x8"

    def key(self, length: float) -> str:
        return f"shoulder_{self.family}_{length:g}"


SHOULDER = ShoulderScrew()
"""The hand crank's grip pin: an 8 mm shoulder, M6."""
CRANKPIN = ShoulderScrew(d=10.0, thread=8.0, thread_len=13.0, head_d=16.0, head_h=7.0,
                         lengths=(16, 20, 25, 30), family="m8x10")
"""The crankpin: a 10 mm shoulder, M8 x 1.25 (McMaster 92981A302-305): the rod's force
bends it over the arm (a cantilever from the arm's top to the rod's middle), and an M6's
thread yields under the jam (the 2026-10-10 review: SF 0.7); the M8's root holds 2.5 x."""


def shoulder_length(need: float, screw: ShoulderScrew = SHOULDER) -> float | None:
    """The shortest stock shoulder at least ``need`` plus 0.15 mm of play (None: none)."""
    return next((v for v in screw.lengths if v >= need + 0.15 - 1e-9), None)


THIN_NUT = (10.0, 3.2)          # DIN 439 M6 thin nut: across flats, height (the grip's)
CRANKPIN_NUT = (13.0, 4.0)      # the crankpin's M8 jam nut (Bolt Depot 13573: 13 x 4)
M6 = screw("bhcs", "6")         # ISO 7380 M6 button heads: the riser's, the shaft's end
SPIGOT_H = 2.5                  # the gearbox's raised 55 mm spigot ring (assumed: measure it)
FACE_DEPTH = 11.0               # the gearbox's face holes' thread depth
HEAD_LAYER_T = 7.0              # the layer over the outer plate: the riser screws' heads, the
#                                 crankpin's jam nut under the arm and its thread's end (its
#                                 13 mm through the arm and the nut), each with room to run
EST_T = 3.175                   # a layer's thickness over the arm, as the search estimates it
HEAD_LAYER = 1
ARM_LAYER = 2
ARM_PLATES = 2                  # the arm: two crank-sheet plates (0.125 in 6061 each: the
#                                 key's 5.1 mm slot is legal in it, not in a 0.250 in plate)
SLEEVE_D = 24.0                 # the printed sleeve and spacer on the shaft (14.2 bore)
CAP_D = 24.0                    # the laser-cut end cap over the spacer (an M6 hole)
RISER_SHEET = ("al5052_3p2mm", 3.175)    # the riser's discs: the frame's sheet
HANDLE_R = 60.0                 # the hand crank's throw (mm)
REACH = 100.0                   # its printed tube from the input shaft's end to its arm: the
#                                 arm turns behind every moving part of the stack (it crosses
#                                 the stack's side there, where nothing moves), clear of the
#                                 outer plate (2026-10-10 review: a crank on the input shaft
#                                 itself swept through the crank arm)
ARM_T, ARM_W = 10.0, 18.0       # the printed crank arm: along the shaft, across
GRIP = (20.0, 25.0)             # its printed grip: d, length (on an 8 x 25 shoulder screw)
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
    outer plate's outer face): at least 80 % of the arm on the key, the shaft's end within
    1.5 mm under the arm's top or above it, a stock M6 button head through the plate and the
    discs into the face's tapped holes, the stack over the arm least (its end as low as
    those allow), then the fewest discs. :class:`ConstructionError` if no stack of up to
    four discs does."""
    best = None
    t_arm = a1 - a0
    key_len = box.shaft_key[2]
    for n in range(1, 5):
        height = n * RISER_SHEET[1]
        face = z0 - height - SPIGOT_H
        hub = face + box.hub_out
        end = hub + box.shaft_out
        key = (end - box.key_short - key_len, end - box.key_short)
        on_key = min(a1, key[1]) - max(a0, key[0])
        if on_key < 0.8 * t_arm or end < a1 - 1.5:
            continue
        # the screws through the plate, the discs and the spigot's gap: at least 1.2 d into
        # the face's tapped holes and short of their bottom
        through = t_plate + height + SPIGOT_H
        screw = next((L for L in M6.lengths
                      if 1.2 * box.face_thread <= L - through <= FACE_DEPTH - 0.5), None)
        if screw is None:
            continue
        cand = (max(end, a1), n, Riser((RISER_SHEET[0],) * n, round(height, 3), hub, key, end,
                                       screw))
        if best is None or cand[:2] < best[:2]:
            best = cand
    if best is None:
        raise ConstructionError(
            f"no riser of up to four 0.125 in discs puts the gearbox's shaft key under an arm "
            f"at z {a0:.1f}-{a1:.1f} mm with a stock M6 screw into the face")
    return best[2]


def arm_z(z0: float, t_frame: float, t_arm: float) -> tuple[float, float]:
    """The arm's z (side): on the layer over the riser screws' heads (fixed by the frame
    plate and the head layer alone, so the search's and the plan's layouts agree)."""
    a0 = z0 + t_frame + HEAD_LAYER_T
    return a0, a0 + t_arm


def top_at_o(riser: Riser, a1: float, t_cap: float) -> float:
    """The highest point of the drive at O: the end cap (``t_cap``, the frame's sheet) over
    the spacer (or the arm) and the end screw's head, with a clearance."""
    return max(riser.shaft_end + 0.5, a1) + t_cap + M6.head_h + 0.3


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
        """The arm's half-width at the crankpin: the shoulder's hole and two thicknesses."""
        return ctx.params.hole(CRANKPIN.d) / 2 + 2 * ctx.sheet_t("crank")

    def screw_points(self, ctx_or_build) -> list[tuple[float, float]]:
        """The gearbox's four face screws, world XY: on the PCD at 45° to the input shaft."""
        g = (ctx_or_build.ctx if isinstance(ctx_or_build, Build) else ctx_or_build)
        o = np.asarray(g.topo.geometry.points["O"][0], dtype=float)
        a0 = math.radians(self.input_angle_deg)
        return [tuple(o + self.box.pcd / 2 * np.array([math.cos(a), math.sin(a)]))
                for a in (a0 + math.pi / 4 + k * math.pi / 2 for k in range(4))]

    def layout_z(self, ctx: Context, L: Layout) -> tuple[float, float, Riser, float]:
        """(the arm's bottom and top, its riser, the drive's top at O), side z."""
        z0 = L.z(0)[0]
        a0, a1 = arm_z(z0, ctx.sheet_t("frame"), ARM_PLATES * ctx.sheet_t("crank"))
        riser = riser_for(z0, a0, a1, self.box, ctx.sheet_t("frame"))
        return a0, a1, riser, top_at_o(riser, a1, ctx.sheet_t("frame"))

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
        t_arm = ARM_PLATES * ctx.sheet_t("crank")
        head_r = M6.head_d / 2 + 0.3
        at_o = max(SLEEVE_D, CAP_D) / 2 + m

        def both(layer, shape, label, **kw):
            return [Placed(layer, shape, GROUP, label, **kw),
                    Placed(layer, shape, GROUP, label, gap=True)]

        def fixed(L: Layout):
            if L.top <= ARM_LAYER + 2:
                raise Unbuildable("the stack has no room for the arm and the rod over it")
            a0, _, _, top = self.layout_z(ctx, L)
            if L.final and abs(L.z(ARM_LAYER)[0] - a0) > 1e-6:
                # (a link or a gap under the arm: the riser is sized for the arm's z)
                raise Unbuildable(f"the arm's layer {ARM_LAYER} is at z "
                                  f"{L.z(ARM_LAYER)[0]:.2f}, not the {a0:.2f} over the plate "
                                  f"and the screw heads' layer")
            out = [Placed(0, Disc("O", SLEEVE_D / 2 + 1.0), GROUP, "sleeve", seat=True)]
            out += [Placed(0, Disc(n, (self.box.face_thread + 0.6) / 2), GROUP,
                           "riser screw", seat=True) for n in names]
            out.append(Placed(0, Disc("O", at_o), GROUP, "sleeve", gap=True))
            out += [Placed(HEAD_LAYER, Disc(n, head_r), GROUP, "riser screw head",
                           sheet=HEAD_LAYER_T) for n in names]
            out += both(HEAD_LAYER, Disc("O", at_o), "sleeve")
            out += both(ARM_LAYER, Disc("O", hub), "arm hub")
            out += both(ARM_LAYER, Pill("O", pin, web), "arm")
            out[-4] = Placed(ARM_LAYER, Disc("O", hub), GROUP, "arm hub", sheet=t_arm)
            k = ARM_LAYER + 1
            while k < L.top and z_of(L, k) < top - 1e-6:
                out += both(k, Disc("O", at_o), "end cap")
                k += 1
            return out

        def z_of(L: Layout, k: int) -> float:
            """Layer ``k``'s bottom: the plan's own z, or (while searching, every layer the
            pitch) the drive's thick layers under it and :data:`EST_T` a layer over them."""
            if L.final or L.thick or L.gaps:
                return L.z(k)[0]
            a1 = arm_z(L.z(0)[0], ctx.sheet_t("frame"), t_arm)[1]
            return a1 + (k - ARM_LAYER - 1) * EST_T

        def crankpin(L: Layout):
            _, a1, _, top = self.layout_z(ctx, L)
            r = L.layers[rider]
            r0 = z_of(L, r)
            if r0 < top - 1e-6 or r <= ARM_LAYER:
                raise Unbuildable(f"the rod {rider} must ride over the arm's end cap (z "
                                  f"{top:.1f} mm), not in layer {r}")
            rod_top = r0 + ctx.sheet_t("link", rider)
            length = shoulder_length(rod_top - a1 + (0.0 if L.final else 1.0), CRANKPIN)
            if length is None:
                raise Unbuildable(f"no stock shoulder spans the arm to {rider} "
                                  f"({rod_top - a1:.1f} mm)")
            nut_r = CRANKPIN_NUT[0] / math.sqrt(3) + 0.3
            out = both(HEAD_LAYER, Disc(pin, nut_r), "crankpin nut")
            out.append(Placed(ARM_LAYER, Disc(pin, CRANKPIN.thread / 2 + 0.3), GROUP,
                              "crankpin", seat=True))
            for layer in range(ARM_LAYER + 1, r):     # the shoulder in its printed spacer
                out += both(layer, Disc(pin, CRANKPIN.d / 2 + 2.0), "crankpin spacer")
            out.append(Placed(ARM_LAYER, Disc(pin, CRANKPIN.d / 2 + 2.0), GROUP,
                              "crankpin spacer", gap=True))
            out.append(Placed(r, Disc(pin, CRANKPIN.d / 2), GROUP, "crankpin", seat=True))
            head_top = a1 + length + CRANKPIN.head_h + 0.3
            hi = r + 1
            while hi < L.top and z_of(L, hi) < head_top - 1e-6:
                out += both(hi, Disc(pin, CRANKPIN.head_d / 2 + 0.3), "crankpin head")
                hi += 1
            if z_of(L, hi) < head_top - 1e-6 and hi >= L.top:
                raise Unbuildable(f"the crankpin's head over {rider} reaches the inner plate")
            out.append(Placed(r, Disc(pin, CRANKPIN.head_d / 2 + 0.3), GROUP,
                              "crankpin head", gap=True))
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

        # the outer plate: the sleeve's hole, the screws' holes, a pad holding them
        screws = self.screw_points(build)
        out.cut(FRAME_OUTER, Cut(tuple(o), SLEEVE_D + 2.0))
        for xy in screws:
            out.cut(FRAME_OUTER, Cut(xy, box.face_thread + 0.6))
        pad = box.pcd / 2 + (box.face_thread + 0.6) / 2 + 2 * ctx.sheet_t("frame") + 0.5
        out.pad(FRAME_OUTER, tuple(o), tuple(o), pad)

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
        # out of its seat), the printed sleeve under the arm and the spacer over it, both
        # slotted for the key, the cap
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
                                   bom_key="nrv030_output_shaft_14", color=STEEL))
        bore = box.shaft_d + 0.2
        r_slot = box.shaft_d / 2 + box.key_proud + 0.5

        def slotted(z_lo: float, z_hi: float) -> Shape3D:
            """A printed ring on the shaft, slotted for its key."""
            slot = Box(r_slot, k_w + 0.3, z_hi - z_lo + 2.0).moved(Location(
                (float(o[0] + kc[0] * r_slot / 2), float(o[1] + kc[1] * r_slot / 2),
                 (z_lo + z_hi) / 2), (0, 0, math.degrees(key_ang))))
            return difference(ring(tuple(o), SLEEVE_D, bore, z_lo, z_hi), slot)

        out.bodies.append(hardware("crank_sleeve", slotted(riser.hub_end + col_t, a0), crank,
                                   fab="printed", color=PRINT_COLOR))
        cap0 = max(riser.shaft_end + 0.5, a1)
        if cap0 - a1 > 0.3:
            out.bodies.append(hardware("crank_spacer", slotted(a1, cap0), crank,
                                       fab="printed", color=PRINT_COLOR))
        cap = ring(tuple(o), CAP_D, M6.d + 0.6, cap0, cap0 + ctx.sheet_t("frame"))
        out.bodies.append(hardware("crank_cap", cap, crank, fab="laser",
                                   sheet=ctx.sheet("frame"), color="#9aa3ad"))
        cap1 = cap0 + ctx.sheet_t("frame")
        end_len = next(L for L in M6.lengths if L - (cap1 - riser.shaft_end) >= 9.0)
        out.bodies.append(hardware("crank_end_screw", screw_solid(tuple(o), M6, cap1, end_len,
                                                                   up=False),
                                   crank, fab="purchased", bom_key=M6.key(end_len),
                                   color=STEEL))

        # the arm: two keyed plates on the shaft's key, the crankpin's hole
        holes = [Cut(tuple(o), box.shaft_d + 0.1),
                 Rect(tuple(o + kc * (box.shaft_d / 2 + 1.0)), (4.6, k_w + 0.1), key_ang),
                 Cut(tuple(mxy), CRANKPIN.thread + 0.4)]
        t_sheet = ctx.sheet_t("crank")
        for i in range(ARM_PLATES):
            z_lo = a0 + i * t_sheet
            part = plate([(tuple(o), tuple(mxy), self.web_r(ctx))], z_lo, z_lo + t_sheet, holes,
                         discs=[(tuple(o), self.hub_r(ctx))])
            name = crank if i == 0 else f"{crank}_plate{i}"
            out.bodies.append(hardware(name, part, crank, fab="laser", sheet=ctx.sheet("crank"),
                                       color="#9aa3ad"))

        # the crankpin: the shoulder screw through the rod into the arm, its nut under the arm
        r0 = build.z(build.layers[rider])[0]
        rod_top = r0 + ctx.sheet_t("link", rider)
        length = shoulder_length(rod_top - a1, CRANKPIN)
        if length is None:
            raise ConstructionError(f"the crankpin's shoulder would be over {rod_top - a1:.1f} "
                                    f"mm, past the longest stock {CRANKPIN.lengths[-1]:g} mm")
        top_pin = a1 + length
        screw = union([disc(tuple(mxy), CRANKPIN.d / 2, a1, top_pin),
                       disc(tuple(mxy), CRANKPIN.head_d / 2, top_pin,
                            top_pin + CRANKPIN.head_h),
                       disc(tuple(mxy), CRANKPIN.thread * 0.97 / 2,
                            a1 - CRANKPIN.thread_len, a1)])      # its whole thread
        out.bodies.append(hardware("crankpin", screw, crank, fab="purchased",
                                   bom_key=CRANKPIN.key(length), color=STEEL))
        if r0 - a1 > 0.3:
            out.bodies.append(hardware("crankpin_spacer",
                                       ring(tuple(mxy), CRANKPIN.d + 4.0, CRANKPIN.d + 0.3,
                                            a1, r0 - 0.1),
                                       crank, fab="printed", color=PRINT_COLOR))
        if top_pin - rod_top > 0.4:     # the stock shoulder past the rod: a printed washer
            out.bodies.append(hardware("crankpin_washer",
                                       ring(tuple(mxy), CRANKPIN.d + 4.0, CRANKPIN.d + 0.3,
                                            rod_top + 0.1, top_pin - 0.15),
                                       crank, fab="printed", color=PRINT_COLOR))
        nut = _hex(tuple(mxy), CRANKPIN_NUT[0], a0 - CRANKPIN_NUT[1], a0,
                   bore=CRANKPIN.thread)
        out.bodies.append(hardware("crankpin_nut", nut, crank, fab="purchased",
                                   bom_key="m8_jam_nut", color=STEEL))
        out.cut(rider, Cut(tuple(mxy), p.hole(CRANKPIN.d)))

        # the hand crank on the input shaft, its arm behind every moving part of the stack
        end, d, zh = self.handle_axis(build, riser)
        out.bodies += self._handle(end, d, zh, frame)
        out.notes["hand_crank"] = {
            "gearbox": box.key, "ratio": box.ratio, "riser_discs": list(riser.discs),
            "riser_mm": riser.height, "riser_screw_mm": riser.screw,
            "arm_z_mm": [round(a0, 3), round(a1, 3)],
            "key_z_mm": [round(riser.key[0], 3), round(riser.key[1], 3)],
            "shaft_end_z_mm": round(riser.shaft_end, 3),
            "sleeve_mm": round(a0 - riser.hub_end, 2),
            "spacer_mm": round(max(cap0 - a1, 0.0), 2),
            "crankpin_shoulder_mm": length,
            "handle": {"end_xy": [round(float(v), 3) for v in end],
                       "dir": [round(float(v), 6) for v in d], "axis_z": round(zh, 3),
                       "reach_mm": REACH, "arm_t_mm": ARM_T, "hub_r_mm": HUB_R,
                       "r_mm": HANDLE_R + GRIP[0] / 2 + 1.0, "grip_r_mm": HANDLE_R,
                       "grip_len_mm": GRIP[1] + SHOULDER.head_h},
            "throw_mm": round(float(np.linalg.norm(mxy - o)), 3),
            "irreversible": box.irreversible(),
        }
        return out

    def assembly(self, view) -> list:
        """How the drive goes on (:mod:`construction.assembly`, a cradle's
        ``CRADLE_ORDER``): on the bench the shaft through the gearbox from the face's side
        and the riser discs on the face; the outer plate over them and the four screws down
        into the face (their heads end up inside the stack, so first); the slotted sleeve,
        the two arm plates on the key, the spacer, cap and end screw; the crankpin's spacer
        before the rod, its screw and nut once the rod is on; the hand crank last."""
        from spiderpig.construction.assembly import FINISH, STACK, Op, whole

        if "gearbox" not in view.z:
            return []
        topo = getattr(view.ctx, "topo", None)
        arm = next(iter(topo.crank_bodies)) if topo is not None else "conn"
        rider = HandCrank.rider(view.ctx) if view.ctx is not None else None
        k_rod = view.slot(view.z[rider][0]) if rider in view.z else ARM_LAYER + 1
        discs = view.named(r"riser_disc\d+")
        screws = view.named(r"gearbox_screw\d+")
        plates = [arm, *view.named(rf"{re.escape(arm)}_plate\d+")]
        k = self.box.shaft_key
        ops = [
            Op(STACK, (-1, -1, "gearbox"), whole("gearbox", "crank_shaft", *discs),
               "Gearbox on its riser",
               "Measure the gearbox first: its output face's raised spigot (2.5 mm drawn: "
               "under 2 mm, put an M6 washer under each riser screw's head) and where the "
               "output shaft's key sits. The output shaft into the gearbox's hollow output "
               f"from the face's side, collar first against the hub, its {k[0]:g} x {k[1]:g} "
               "key in its seat (the snap ring and washer on its far end); the gearbox on its "
               "back, face up; the riser discs on the face, the spigot in the first one's "
               "bore, every disc's holes over the face's tapped holes.", "gearbox", sub=True),
            Op(STACK, (-1, 2), whole(*screws), "",
               "The outer plate over the discs, its outside down, the shaft up through its "
               f"bore; the {len(screws)} M6 button heads down through the plate and the "
               "discs into the gearbox's face (threadlocker), tight. The gearbox now "
               "stands the plate up off the bench.", "gearbox_screws"),
            Op(STACK, (HEAD_LAYER, 0), whole("crank_sleeve"), "",
               "The printed sleeve down over the shaft, its slot over the key, onto the "
               "shaft's collar.", "layer"),
            Op(STACK, (ARM_LAYER, 0), whole(*plates), "Crank arm",
               "The two arm plates down onto the shaft, their keyways over the key, onto "
               "the sleeve (file a keyway that binds: the key must slide).", "layer"),
            Op(STACK, (ARM_LAYER, 1), whole("crank_spacer", "crank_cap", "crank_end_screw"),
               "", "The printed spacer over the shaft's end, slot over the key, the cap on "
               "it and the M6 button head into the shaft's end tap (threadlocker), snug: it "
               "clamps the arm down on the sleeve.", "layer"),
            Op(STACK, (ARM_LAYER, 2), whole("crankpin_spacer"), "",
               "The crankpin's printed spacer on the arm over its hole.", "layer"),
            Op(STACK, (k_rod, 2), whole("crankpin_washer", "crankpin", "crankpin_nut"), "",
               "With the rod on: the shoulder screw down through the printed washer, the "
               "rod's bore and the spacer into the arm; under the arm, its M8 jam nut held "
               "on a thin 13 mm open-end wrench slid into the 7 mm gap over the outer plate "
               "(threadlocker), tight; the rod turns on the screw's shoulder.", "layer"),
            Op(FINISH, (2,), whole(*view.named(r"handle(_grip|_grip_screw|_nut|_set_screw)?")),
               "Hand crank",
               "The grip's shoulder screw through the printed grip and the crank's arm, its "
               "thin nut behind (threadlocker): the grip spins on the shoulder. The crank's "
               "hub onto the gearbox's input shaft, its keyway over the shaft's key, and the "
               "M3 set screw down onto the key.", "handle"),
        ]
        ops = [replace(op, pieces=tuple(p for p in op.pieces if p.name in view.z))
               for op in ops]               # (a stock shoulder that fits needs no washer)
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
        spigot = Cylinder(b.spigot_d / 2, 2.0).moved(Location((0.0, 0.0, 1.0)))
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
        spinning on an 8 mm shoulder screw through the arm, a thin nut behind. Drawn at
        rest, the arm down."""
        a = math.atan2(d[1], d[0])
        rot = (0.0, 90.0, math.degrees(a))
        e = np.asarray(end, dtype=float)

        def along(t: float) -> tuple[float, float]:
            return float(e[0] + d[0] * t), float(e[1] + d[1] * t)

        hub = Cylinder(HUB_R, HUB_LEN + REACH, rotation=rot).moved(
            Location((*along((REACH - HUB_LEN) / 2), z)))
        t_arm = REACH - ARM_T / 2                       # the arm's middle, along the shaft
        n = np.array([d[1], -d[0]])                     # the arm's way at rest (d +x: down)
        arm_len = HANDLE_R + ARM_W / 2
        ac = np.asarray(along(t_arm)) + n * arm_len / 2
        arm = Box(ARM_T, arm_len, ARM_W).moved(
            Location((float(ac[0]), float(ac[1]), z), (0, 0, math.degrees(a))))
        bore = Cylinder(self.box.input_d / 2 + 0.15, HUB_LEN + 1.0, rotation=rot).moved(
            Location((*along(-HUB_LEN / 2 + 0.5), z)))
        key_top = self.box.input_d / 2 + INPUT_KEY_HUB      # the key's top, over the axis
        keyway = Box(HUB_LEN + 2.0, INPUT_KEY + 0.2, key_top + 0.2).moved(
            Location((*along(-HUB_LEN / 2), z + (key_top + 0.2) / 2), (0, 0, math.degrees(a))))
        mid = along(-HUB_LEN / 2)
        pilot = Cylinder(1.3, HUB_R - key_top + 2.0).moved(
            Location((*mid, z + (HUB_R + key_top + 2.0) / 2)))
        set_screw = Cylinder(1.25, HANDLE_SET).moved(
            Location((*mid, z + key_top + 0.05 + HANDLE_SET / 2)))
        # the grip at the arm's end, out along the shaft; its screw through the arm, the nut
        # on the arm's inner face
        g0 = REACH + 0.2                              # the grip from the arm's outer face
        grip = Cylinder(GRIP[0] / 2, GRIP[1] - 0.4, rotation=rot).moved(
            Location((*along(g0 + (GRIP[1] - 0.4) / 2), z)))
        grip = grip - Cylinder(SHOULDER.d / 2 + 0.25, GRIP[1] + 2, rotation=rot).moved(
            Location((*along(g0 + GRIP[1] / 2), z)))
        hole = Cylinder(SHOULDER.thread / 2 + 0.2, ARM_T + 2, rotation=rot).moved(
            Location((*along(t_arm), z)))
        g_len = shoulder_length(GRIP[1]) or GRIP[1]
        screw = union([
            Cylinder(SHOULDER.d / 2, g_len, rotation=rot).moved(
                Location((*along(REACH + g_len / 2), z))),
            Cylinder(SHOULDER.head_d / 2, SHOULDER.head_h, rotation=rot).moved(
                Location((*along(REACH + g_len + SHOULDER.head_h / 2), z))),
            Cylinder(SHOULDER.thread * 0.97 / 2, SHOULDER.thread_len, rotation=rot).moved(
                Location((*along(REACH - SHOULDER.thread_len / 2), z)))])
        nut = (Cylinder(THIN_NUT[0] / 2, THIN_NUT[1], rotation=rot)
               - Cylinder(SHOULDER.thread / 2, THIN_NUT[1] + 2, rotation=rot)).moved(
            Location((*along(REACH - ARM_T - THIN_NUT[1] / 2), z)))
        down = Location((float(n[0] * HANDLE_R), float(n[1] * HANDLE_R), 0.0))   # arm's end
        return [hardware("handle", difference(union([hub, arm]), bore, keyway, pilot,
                                              hole.moved(down)),
                         host, fab="printed", color=PRINT_COLOR),
                hardware("handle_set_screw", set_screw, host, fab="purchased",
                         bom_key=f"m3_set_screw_{HANDLE_SET:g}", color=STEEL),
                hardware("handle_grip", grip.moved(down), host, fab="printed",
                         color="#2b2b2b"),
                hardware("handle_grip_screw", screw.moved(down), host, fab="purchased",
                         bom_key=SHOULDER.key(g_len), color=STEEL),
                hardware("handle_nut", nut.moved(down), host, fab="purchased",
                         bom_key="m6_thin_nut", color=STEEL)]


def _hex(xy, af: float, z0: float, z1: float, bore: float = 0.0) -> Shape3D:
    """A hex prism (a nut, a standoff), ``af`` across flats, optionally bored."""
    from build123d import RegularPolygon, extrude

    face = RegularPolygon(af / 2, 6, major_radius=False)
    part = extrude(face, z1 - z0)
    part = moved(part, Location((float(xy[0]), float(xy[1]), z0)))
    if bore > 0:
        part = part - disc(xy, bore / 2, z0 - 1, z1 + 1)
    return part


@dataclass(frozen=True)
class HandCrankConstruction:
    """The ``"hand"`` entry of :data:`construction.CRANKS`: what the rest of the code asks of
    a crank construction by key (the rider's bore, what the crank holds); the group is
    :class:`HandCrank`."""

    key: str = "hand"
    label: str = ("hand crank: an NRV030 worm box, a keyed arm of two 0.125 in plates and a "
                  "printed offset hand crank")
    gearbox: str = NRV030.key
    crankpin: str = CRANKPIN.key(25)
    sheet_t: float = ARM_PLATES * 3.175    # the arm (two crank-sheet plates, 6061-T6)
    yield_mpa: float = 276.0    # 6061-T6
    bearing_mpa: float = 386.0  # its bearing yield (1.4 x yield, ASM)
    throw: float = 80.0         # the nominal crank (the build's own: notes["hand_crank"])

    def resolve(self, ctx) -> HandCrankConstruction:
        return self

    def for_sheet(self, key: str) -> HandCrankConstruction:
        return self

    def rider_d(self, params) -> float:
        """The rod's bore: the crankpin's shoulder."""
        return CRANKPIN.d

    def capacity(self, throw: float | None = None) -> dict[str, float]:
        """The crank torque (N·m) each element holds at yield: the arm bent at its hub by
        the crankpin's force across it (the torque over the throw), and the arm's keyway
        bearing on the shaft's key (the key's 2 mm proud of the shaft, the arm's thickness,
        at the key's mean radius)."""
        r = throw or self.throw
        t = self.sheet_t
        w = 2 * ((CRANKPIN.thread + 0.4) / 2 + 2 * t)
        z = t * w * w / 6
        hub = NRV030.shaft_d / 2 + 2.3 + 2 * t
        arm = self.yield_mpa * z * r / max(r - hub, 1.0) / 1000.0
        proud = 2.0
        key = self.bearing_mpa * proud * t * (NRV030.shaft_d / 2 + proud / 2) / 1000.0
        return {"arm": round(arm, 2), "arm keyway": round(key, 2)}


HAND_CRANK = HandCrankConstruction()
