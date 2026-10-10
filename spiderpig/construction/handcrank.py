"""The hand-cranked worm drive of a decanting cradle (``config.crank == "hand"``).

A decanting cradle (:mod:`linkages.decanters`, ``docs/agentlib/DECANTER.md`` has the why) is
turned by hand, slowly, and must stay where it is let go, so its crank is driven through a
self-locking worm gear reducer instead of a servo:

* **The gearbox** (:class:`WormBox`, an NRV030 at 30:1) stands outboard of the drive side's
  *outer* frame plate, its output axis on O, its output face toward the plate. Its static
  efficiency at 30:1 is 0.39, under the 0.5 the maker's catalogue gives for static
  irreversibility: a load on the cradle can't turn the worm. Its 9 mm input shaft points
  back (``input_angle_deg``) and carries a printed crank arm with a revolving handle.
* **The riser** (:func:`riser_for`): laser-cut aluminium discs between the gearbox's face
  and the plate, the face's raised 55 mm spigot ring bearing on the first; four M6 button
  heads come up through the plate and the discs into the face's tapped holes. The discs set
  how far the gearbox's stock output shaft (14 mm, keyed 5 x 5 x 20, an M6 end tap; 32.5 mm
  past the face) reaches into the stack: just past the arm, so the rod can sweep over it.
* **The arm**: one laser-cut 0.250 in 6061-T6 plate (the crank sheet) on the shaft's key, a
  printed sleeve under it down to the gearbox's hub, a printed spacer over it up to the
  shaft's end, all clamped by a laser-cut end cap and an M6 button head in the shaft's end
  tap. The sleeve and spacer are printed to the shaft as measured (the key's place on it is
  not dimensioned).
* **The crankpin**: an ISO 7379 shoulder screw (8 mm shoulder, M6 thread) down through the
  rod into the arm, a thin M6 nut under the arm. The rod turns on the shoulder, a printed
  spacer under it, a printed washer over it where the stock shoulder is longer, the head
  over that.

The rod sweeps across the axis O once a turn (the crank-rocker's folded toggle), so nothing
of the drive may stand at O in the rod's layer: the planner puts the rod over the end cap.
The drive side alone carries all of this; the other side of the cradle has no crank
(:mod:`construction.cradle`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

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
    The single output shaft (14 h6 x 102, keyed 5 x 5 x 20 outboard, M6 x 15 centre tap)
    stands 32.5 past the face. Mesh data at 30:1: lead 4°52', static efficiency 0.39
    (statically irreversible under 0.5), 0.66 at 1400 rpm; Mn2 21 N·m at 1140 rpm input
    (Phidgets rates it 20 N·m, 1.8 kN radial, 1/2° backlash)."""

    key: str = "nrv030_30"
    ratio: int = 30
    eta_static: float = 0.39
    eta_dynamic: float = 0.66       # the catalogue's, at 1400 rpm: a bound on the hand's
    rated_nm: float = 20.0
    radial_n: float = 1162.0        # Fr2 at 1140 rpm, i = 30 (Phidgets says 1.8 kN)
    pcd: float = 65.0
    face_thread: float = 6.0
    spigot_d: float = 55.0
    width: float = 58.0             # face to face, along the output axis
    hub_d: float = 25.0             # the output hub's end (assumed: not dimensioned)
    hub_out: float = 2.5
    shaft_d: float = 14.0
    shaft_out: float = 30.0         # the shaft's end past the hub's end (32.5 past the face)
    shaft_key: tuple[float, float, float] = (5.0, 5.0, 20.0)
    worm_offset: float = 30.0
    input_d: float = 9.0
    input_len: float = 20.0
    input_face: float = 45.0        # output axis to the input shaft's shoulder
    cap_face: float = 51.0          # output axis to the worm's far end (the VS's shoulder)
    below: float = 40.0             # output axis to the face away from the worm
    above: float = 57.0             # output axis to the face over the worm
    mass_g: float = 1200.0

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

    def key(self, length: float) -> str:
        return f"shoulder_m6x8_{length:g}"


SHOULDER = ShoulderScrew()


def shoulder_length(need: float) -> float | None:
    """The shortest stock shoulder at least ``need`` plus 0.15 mm of play (None: none)."""
    return next((v for v in SHOULDER.lengths if v >= need + 0.15 - 1e-9), None)


SHIM_MAX = 3.0                  # the crankpin's DIN 988 shims under the rod, at most


def crankpin_fit(reach: float) -> tuple[float, float] | None:
    """(the stock shoulder, the shims under the rod) for a rod whose bottom is ``reach`` mm
    over the arm's bottom: the longest shoulder the 0.15 mm of play leaves room for, the
    rest in 0.1 mm shims (at most :data:`SHIM_MAX`, and the thread's 11 mm through them, the
    rod and the nut). ``None``: none fits."""
    for v in sorted(SHOULDER.lengths, reverse=True):
        shims = round(reach - 0.15 - v, 1)
        if -1e-9 <= shims <= SHIM_MAX + 1e-9:
            return v, max(shims, 0.0)
    return None


THIN_NUT = (10.0, 3.2)          # DIN 439 M6 thin nut: across flats, height
BUTTON_M6 = (10.5, 3.3)         # ISO 7380 M6 button head: diameter, height
SPIGOT_H = 2.5                  # the gearbox's raised 55 mm spigot ring (assumed: measure it)
KEY_START = 4.0                 # the shaft's key starts this far past the hub's end (assumed)
FACE_DEPTH = 11.0               # the gearbox's face holes' thread depth
HEAD_LAYER_T = 3.6              # the layer over the outer plate: the riser screws' heads and
#                                 the crankpin's thin nut under the arm
EST_T = 3.175                   # a layer's thickness over the arm, as the search estimates it
HEAD_LAYER = 1
ARM_LAYER = 2
SLEEVE_D = 24.0                 # the printed sleeve and spacer on the shaft (14.2 bore)
CAP_D = 24.0                    # the laser-cut end cap over the spacer (an M6 hole)
RISER_SHEETS = (("al5052_6p35mm", 6.35), ("al5052_3p2mm", 3.175))
RISER_SCREWS = (16.0, 20.0, 25.0, 30.0, 35.0, 40.0)   # ISO 7380 M6 button heads (stock)
HANDLE_R = 60.0                 # the hand crank's throw (mm)
GRIP = (24.0, 74.0)             # the revolving handle's grip (GN 798-KT-24-M8): d, length


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


def riser_for(z0: float, a0: float, a1: float, box: WormBox = NRV030) -> Riser:
    """The riser that puts the shaft's key under the arm (``a0``..``a1``, side z; ``z0`` the
    outer plate's outer face): at least 80 % of the arm on the key, the shaft's end within
    1.5 mm under the arm's top or above it, the stack over the arm least (its end as low as
    those allow), then the fewest discs. :class:`ConstructionError` if no stack of up to
    four discs does."""
    best = None
    t_arm = a1 - a0
    for n6 in range(4):
        for n3 in range(3):
            if n6 + n3 == 0 or n6 + n3 > 4:
                continue
            height = n6 * RISER_SHEETS[0][1] + n3 * RISER_SHEETS[1][1]
            face = z0 - height - SPIGOT_H
            hub = face + box.hub_out
            key = (hub + KEY_START, hub + KEY_START + box.shaft_key[2])
            end = face + box.hub_out + box.shaft_out
            on_key = min(a1, key[1]) - max(a0, key[0])
            if on_key < 0.8 * t_arm or end < a1 - 1.5:
                continue
            # the screws through the plate (0.125 in), the discs and the spigot's gap: at least
            # 1.2 d into the face's tapped holes and short of their bottom
            through = 3.175 + height + SPIGOT_H
            screw = next((L for L in RISER_SCREWS
                          if 1.2 * box.face_thread <= L - through <= FACE_DEPTH - 0.5), None)
            if screw is None:
                continue
            cand = (max(end, a1), n6 + n3, Riser(
                (RISER_SHEETS[0][0],) * n6 + (RISER_SHEETS[1][0],) * n3, round(height, 3),
                hub, key, end, screw))
            if best is None or cand[:2] < best[:2]:
                best = cand
    if best is None:
        raise ConstructionError(
            f"no riser of up to four 0.250/0.125 in discs puts the gearbox's shaft key under "
            f"an arm at z {a0:.1f}-{a1:.1f} mm")
    return best[2]


def arm_z(z0: float, t_frame: float, t_arm: float) -> tuple[float, float]:
    """The arm's z (side): on the layer over the riser screws' heads (fixed by the frame
    plate and the head layer alone, so the search's and the plan's layouts agree)."""
    a0 = z0 + t_frame + HEAD_LAYER_T
    return a0, a0 + t_arm


def top_at_o(riser: Riser, a1: float) -> float:
    """The highest point of the drive at O: the end cap over the spacer (or the arm) and
    the end screw's head, with a clearance."""
    return max(riser.shaft_end + 0.5, a1) + 3.175 + BUTTON_M6[1] + 0.3


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
        return ctx.params.hole(SHOULDER.d) / 2 + 2 * ctx.sheet_t("crank")

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
        a0, a1 = arm_z(z0, ctx.sheet_t("frame"), ctx.sheet_t("crank"))
        riser = riser_for(z0, a0, a1, self.box)
        return a0, a1, riser, top_at_o(riser, a1)

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
        t_arm = ctx.sheet_t("crank")
        head_r = BUTTON_M6[0] / 2 + 0.3
        at_o = max(SLEEVE_D, CAP_D) / 2 + m

        def both(layer, shape, label, **kw):
            return [Placed(layer, shape, GROUP, label, **kw),
                    Placed(layer, shape, GROUP, label, gap=True)]

        def fixed(L: Layout):
            if L.top <= ARM_LAYER + 2:
                raise Unbuildable("the stack has no room for the arm and the rod over it")
            _, _, _, top = self.layout_z(ctx, L)
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
            length = shoulder_length(rod_top - a1 + (0.0 if L.final else 1.0))
            if length is None:
                raise Unbuildable(f"no stock shoulder spans the arm to {rider} "
                                  f"({rod_top - a1:.1f} mm)")
            nut_r = THIN_NUT[0] / math.sqrt(3) + 0.3
            out = both(HEAD_LAYER, Disc(pin, nut_r), "crankpin nut")
            out.append(Placed(ARM_LAYER, Disc(pin, SHOULDER.thread / 2 + 0.3), GROUP,
                              "crankpin", seat=True))
            for layer in range(ARM_LAYER + 1, r):     # the shoulder in its printed spacer
                out += both(layer, Disc(pin, SHOULDER.d / 2 + 2.0), "crankpin spacer")
            out.append(Placed(ARM_LAYER, Disc(pin, SHOULDER.d / 2 + 2.0), GROUP,
                              "crankpin spacer", gap=True))
            out.append(Placed(r, Disc(pin, SHOULDER.d / 2), GROUP, "crankpin", seat=True))
            head_top = a1 + length + SHOULDER.head_h + 0.3
            hi = r + 1
            while hi < L.top and z_of(L, hi) < head_top - 1e-6:
                out += both(hi, Disc(pin, SHOULDER.head_d / 2 + 0.3), "crankpin head")
                hi += 1
            if z_of(L, hi) < head_top - 1e-6 and hi >= L.top:
                raise Unbuildable(f"the crankpin's head over {rider} reaches the inner plate")
            out.append(Placed(r, Disc(pin, SHOULDER.head_d / 2 + 0.3), GROUP,
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
            head = disc(xy, BUTTON_M6[0] / 2, z1, z1 + BUTTON_M6[1])
            shank = disc(xy, box.face_thread * 0.97 / 2, z1 - riser.screw, z1)
            out.bodies.append(hardware(f"gearbox_screw{i}", union([head, shank]), frame,
                                       fab="purchased", bom_key=f"m6_bhcs_{riser.screw:g}",
                                       color=STEEL))

        # the output shaft, the printed sleeve under the arm, the spacer over it, the cap
        shaft = disc(tuple(o), box.shaft_d / 2, riser.hub_end - 5.0, riser.shaft_end) - disc(
            tuple(o), 3.0, riser.shaft_end - 15.0, riser.shaft_end + 1.0)    # its M6 x 15 tap
        out.bodies.append(hardware("crank_shaft", shaft, crank, fab="purchased",
                                   bom_key="nrv030_output_shaft_14", color=STEEL))
        bore = box.shaft_d + 0.2
        out.bodies.append(hardware("crank_sleeve", ring(tuple(o), SLEEVE_D, bore,
                                                        riser.hub_end, a0), crank,
                                   fab="printed", color=PRINT_COLOR))
        cap0 = max(riser.shaft_end + 0.5, a1)
        if cap0 - a1 > 0.3:
            out.bodies.append(hardware("crank_spacer", ring(tuple(o), SLEEVE_D, bore, a1, cap0),
                                       crank, fab="printed", color=PRINT_COLOR))
        cap = ring(tuple(o), CAP_D, 6.6, cap0, cap0 + 3.175)
        out.bodies.append(hardware("crank_cap", cap, crank, fab="laser",
                                   sheet=ctx.sheet("frame"), color="#9aa3ad"))
        end_screw = union([disc(tuple(o), BUTTON_M6[0] / 2, cap0 + 3.175,
                                cap0 + 3.175 + BUTTON_M6[1]),
                           disc(tuple(o), 6.0 * 0.97 / 2, cap0 + 3.175 - 16.0, cap0 + 3.175)])
        out.bodies.append(hardware("crank_end_screw", end_screw, crank, fab="purchased",
                                   bom_key="m6_bhcs_16", color=STEEL))

        # the arm: one keyed plate on the shaft's key, the crankpin's hole
        key_ang = arm_ang + math.pi / 2
        kc = np.array([math.cos(key_ang), math.sin(key_ang)])
        holes = [Cut(tuple(o), box.shaft_d + 0.1),
                 Rect(tuple(o + kc * (box.shaft_d / 2 + 1.0)), (4.6, box.shaft_key[0] + 0.1),
                      key_ang),
                 Cut(tuple(mxy), SHOULDER.thread + 0.4)]
        part = plate([(tuple(o), tuple(mxy), self.web_r(ctx))], a0, a1, holes,
                     discs=[(tuple(o), self.hub_r(ctx))])
        out.bodies.append(hardware(crank, part, crank, fab="laser", sheet=ctx.sheet("crank"),
                                   color="#9aa3ad"))

        # the crankpin: the shoulder screw through the rod into the arm, its nut under the arm
        r0 = build.z(build.layers[rider])[0]
        rod_top = r0 + ctx.sheet_t("link", rider)
        length = shoulder_length(rod_top - a1)
        if length is None:
            raise ConstructionError(f"the crankpin's shoulder would be over {rod_top - a1:.1f} "
                                    f"mm, past the longest stock {SHOULDER.lengths[-1]:g} mm")
        top_pin = a1 + length
        screw = union([disc(tuple(mxy), SHOULDER.d / 2, a1, top_pin),
                       disc(tuple(mxy), SHOULDER.head_d / 2, top_pin,
                            top_pin + SHOULDER.head_h),
                       disc(tuple(mxy), SHOULDER.thread * 0.97 / 2,
                            max(a1 - SHOULDER.thread_len, a0 - THIN_NUT[1]), a1)])
        out.bodies.append(hardware("crankpin", screw, crank, fab="purchased",
                                   bom_key=SHOULDER.key(length), color=STEEL))
        if r0 - a1 > 0.3:
            out.bodies.append(hardware("crankpin_spacer",
                                       ring(tuple(mxy), SHOULDER.d + 4.0, SHOULDER.d + 0.3,
                                            a1, r0 - 0.1),
                                       crank, fab="printed", color=PRINT_COLOR))
        if top_pin - rod_top > 0.4:     # the stock shoulder past the rod: a printed washer
            out.bodies.append(hardware("crankpin_washer",
                                       ring(tuple(mxy), SHOULDER.d + 4.0, SHOULDER.d + 0.3,
                                            rod_top + 0.1, top_pin - 0.15),
                                       crank, fab="printed", color=PRINT_COLOR))
        nut = _hex(tuple(mxy), THIN_NUT[0], a0 - THIN_NUT[1], a0, bore=SHOULDER.thread)
        out.bodies.append(hardware("crankpin_nut", nut, crank, fab="purchased",
                                   bom_key="m6_thin_nut", color=STEEL))
        out.cut(rider, Cut(tuple(mxy), p.hole(SHOULDER.d)))

        # the hand crank on the input shaft (printed arm, revolving handle on an M8 stud)
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
            "throw_mm": round(float(np.linalg.norm(mxy - o)), 3),
            "irreversible": box.irreversible(),
        }
        return out

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
        """The printed crank arm on the input shaft (keyed, an M3 grub screw on the key) and
        the revolving handle bolted through it (its M8 stud, a nut behind): drawn at rest,
        the grip out from the gearbox."""
        a = math.atan2(d[1], d[0])
        t = 10.0
        rot = (0.0, 90.0, math.degrees(a))
        arm = Box(t, 16.0, HANDLE_R + 16.0).moved(
            Location((float(end[0] - d[0] * t / 2), float(end[1] - d[1] * t / 2),
                      z - HANDLE_R / 2), (0, 0, math.degrees(a))))
        hub = Cylinder(10.0, 18.0, rotation=rot).moved(
            Location((float(end[0] - d[0] * 9.0), float(end[1] - d[1] * 9.0), z)))
        bore = Cylinder(self.box.input_d / 2 + 0.15, 40.0, rotation=rot).moved(
            Location((float(end[0] - d[0] * 20.0), float(end[1] - d[1] * 20.0), z)))
        grip_c = np.asarray(end, dtype=float) + d * (GRIP[1] / 2)
        grip = Cylinder(GRIP[0] / 2, GRIP[1], rotation=rot).moved(
            Location((float(grip_c[0]), float(grip_c[1]), z - HANDLE_R)))
        return [hardware("handle", union([arm, hub]) - bore, host, fab="printed",
                         color=PRINT_COLOR),
                hardware("handle_grip", grip, host, fab="purchased",
                         bom_key="revolving_handle_m8", color="#2b2b2b"),
                hardware("handle_nut", Cylinder(7.5, 6.5, rotation=rot).moved(
                    Location((float(end[0] - d[0] * (t + 3.25)), float(end[1] - d[1] *
                              (t + 3.25)), z - HANDLE_R))), host, fab="purchased",
                         bom_key="m8_nut", color=STEEL)]


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
    label: str = "hand crank: an NRV030 worm box, a keyed 0.25 in arm and a revolving handle"
    gearbox: str = NRV030.key
    handle: str = "revolving_handle_m8"
    sheet_t: float = 6.35       # the arm (the crank sheet, 0.250 in 6061-T6)
    yield_mpa: float = 276.0    # 6061-T6
    bearing_mpa: float = 386.0  # its bearing yield (1.4 x yield, ASM)
    throw: float = 80.0         # the nominal crank (the build's own: notes["hand_crank"])

    def resolve(self, ctx) -> HandCrankConstruction:
        return self

    def for_sheet(self, key: str) -> HandCrankConstruction:
        return self

    def rider_d(self, params) -> float:
        """The rod's bore: the crankpin's shoulder."""
        return SHOULDER.d

    def capacity(self, throw: float | None = None) -> dict[str, float]:
        """The crank torque (N·m) each element holds at yield: the arm bent at its hub by
        the crankpin's force across it (the torque over the throw), and the arm's keyway
        bearing on the shaft's key (the key's 2 mm proud of the shaft, the arm's thickness,
        at the key's mean radius)."""
        r = throw or self.throw
        t = self.sheet_t
        w = 2 * ((SHOULDER.thread + 0.4) / 2 + 2 * t)
        z = t * w * w / 6
        hub = NRV030.shaft_d / 2 + 2.3 + 2 * t
        arm = self.yield_mpa * z * r / max(r - hub, 1.0) / 1000.0
        proud = 2.0
        key = self.bearing_mpa * proud * t * (NRV030.shaft_d / 2 + proud / 2) / 1000.0
        return {"arm": round(arm, 2), "arm keyway": round(key, 2)}


HAND_CRANK = HandCrankConstruction()
