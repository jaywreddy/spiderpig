"""``capstan``: a Dyneema cord wrapped on a sleeve on the motor's shaft, anchored on a drum
bolted to the link.

The drum is a stack of laser-cut plates beside the driven link (a flange, ``core`` layers
of the drum's radius, a flange: the cord runs in the groove they make), bolted through the
link's pad with four M3 screws (their heads on the drum's far face, the nuts past the
link). The capstan is a 6 mm aluminium tube
bonded over the motor's D-flat shaft (the cord wraps a round surface: no ripple once a
turn, no fraying on the flat's edges), touching the drum through the cord: the motor sits
at ``R + r + d`` from the pivot. The flanges stand only ``FLANGE`` over the core: the
capstan comes within the cord's diameter of the core, so a taller flange would meet it.

**The cord**: from one anchor round the groove to the capstan, ``WRAPS`` turns round it
(friction grows as ``e^{mu theta}``: four turns hold with a small pretension), back round
the groove to the other anchor. Each anchor is one of the drum's bolts: the cord goes in
through a notch cut in the core plates from the rim to the bolt, round its shank, and the
bolt pinches it between the plates. The anchors sit ``ANCHOR_MARGIN`` past the travel's
ends either side of where the cord leaves the drum (the tangent toward the motor), in the
link's frame, so the cord never leaves its groove over the input's travel. The capstan's
wraps walk along it as the drum turns (``travel x ratio`` turns), so the core is as tall
as the wraps and that walk need (:meth:`CapstanDrive.layers`).

The ratio is ``(R + d / 2) / (r + d / 2)``: 8.7:1 at the default 30 mm drum on the 6 mm
sleeve, with no backlash (the cord never leaves the capstan). (The drum's radius is
bounded by the frame: the palletizer's levelling pillar stands 40 mm from the shoulder,
and its spacers must pass the drum's flange.)

**Its error terms.** Two strands pretensioned, each ``EA / l``: ``l`` the cord from the
capstan round the groove to its anchor, the longest at the travel's end
(``R (travel + margin) + notch``); on the drum's radius, ``2 (EA / l) R²`` at the joint;
no backlash.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig.construction.base import Build, ConstructionError, Context, Realized, hardware
from spiderpig.construction.joints import ErrorTerms
from spiderpig.construction.transmissions.base import ALU, DriveGroup, DriveHub, Transmission
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.motion_catalog import CORD_D, CORD_EA_N
from spiderpig.shapes import Cut, Rect, cut_holes, ring

WRAPS = 4
FLANGE = 0.6          # the drum's flanges stand this far over its core (mm): under the cord
CORD_PITCH = 1.2      # the wraps' pitch along the capstan (mm)
ANCHOR_MARGIN = 40.0  # the anchors past the travel's ends from the cord's tangent (deg)
NOTCH_W = 1.6         # the notch the cord takes from the rim to its anchor bolt (mm)
DRUM_COLOR = "#7fb3e0"


@dataclass(frozen=True)
class CapstanDrive(Transmission):
    """A cord capstan: drum radius ``drum_r``, capstan diameter ``capstan_d`` (module
    docstring)."""

    key: str = "capstan"
    label: str = ("Dyneema cord capstan, 8.7:1: a laser-cut drum bolted to the link, the cord "
                  "wrapped on a sleeve on the motor's shaft, no backlash")
    drum_r: float = 30.0
    capstan_d: float = 6.0   # the sleeve on the shaft (tube_al_6x5)
    sleeve_key: str = "tube_al_6x5"
    bolts: int = 4
    bolt_pcd: float = 30.0
    nuts: bool = True

    def ratio(self, detail=None) -> float:
        return (self.drum_r + CORD_D / 2) / (self.capstan_d / 2 + CORD_D / 2)

    def core(self, ctx: Context, travel_deg: float) -> int:
        """The core's layers: the wraps and their walk along the capstan, and a millimetre."""
        walk = travel_deg / 360.0 * self.ratio()
        return max(1, math.ceil(((WRAPS + walk) * CORD_PITCH + 1.0) / ctx.pitch - 1e-9))

    def layers(self, ctx: Context, travel_deg: float = 360.0) -> int:
        return self.core(ctx, travel_deg) + 2

    def hub_r(self, ctx: Context) -> float:
        return self.drum_r + FLANGE

    def motor_side_r(self, ctx: Context) -> float:
        return self.capstan_d / 2

    def distances(self, ctx: Context, motor) -> list[tuple[float, None]]:
        return [(self.drum_r + self.capstan_d / 2 + CORD_D, None)]

    def check(self, ctx: Context, hub: DriveHub) -> None:
        """The flanges clear the sleeve (and the bare shaft beside them)."""
        c = math.dist(hub.motor, ctx.topo.geometry.points[hub.axis][0])
        if self.drum_r + FLANGE + self.capstan_d / 2 + 0.2 > c + 1e-9:
            raise ConstructionError(f"capstan: a {FLANGE:g} mm flange over the {self.drum_r:g} "
                                    f"mm drum meets the {self.capstan_d:g} mm capstan")

    def bolt_angles(self, hub: DriveHub) -> list[float]:
        """Two anchors either side of the cord's tangent, past the travel's ends (in the
        link's frame at mid travel), and a bolt opposite each."""
        half = math.radians(hub.travel_deg / 2 + ANCHOR_MARGIN)
        anchors = [hub.angle + half - hub.mid_dir, hub.angle - half - hub.mid_dir]
        return [*anchors, *(a + math.pi for a in anchors)]

    def realize(self, build: Build, group: DriveGroup, hub: DriveHub, out: Realized,
                layers: list[int]) -> None:
        ctx = build.ctx
        o = tuple(build.xy(hub.axis))
        stem = group.name.replace(":", "_")
        bore = self._bore(ctx)
        sheet = ctx.sheet("link")
        other = build.xy(group._other(ctx))
        bolts = [tuple(p[0]) for p in group.bolt_points(np.asarray(o), np.asarray(other),
                                                        hub)]
        link_dir = math.atan2(other[1] - o[1], other[0] - o[0])
        rb = self.bolt_pcd / 2
        notches: list[Cut | Rect] = []
        for a in self.bolt_angles(hub)[:2]:          # rim to anchor bolt, turning with it
            w = link_dir + a
            mid = (rb + self.drum_r) / 2
            notches.append(Rect((o[0] + mid * math.cos(w), o[1] + mid * math.sin(w)),
                                (self.drum_r - rb + 2.0, NOTCH_W), w))
        holes: list[Cut | Rect] = [Cut(b, 3.4) for b in bolts]
        for i, k in enumerate(layers):
            z0, z1 = build.z(k)
            flange = i in (0, len(layers) - 1)
            r = self.drum_r + (FLANGE if flange else 0.0)
            part = cut_holes(ring(o, 2 * r, bore, z0, z1),
                             holes + ([] if flange else notches), z0, z1)
            out.bodies.append(hardware(f"{stem}_drum{i}", part, group.link, fab="laser",
                                       color=DRUM_COLOR, sheet=sheet))
        z0, z1 = build.z(layers[1])[0], build.z(layers[-2])[1]
        frame = build.plan.topo.frame_bodies[0]
        cap = ring(hub.motor, self.capstan_d, group.motor.shaft_d + 0.1, z0, z1)
        out.bodies.append(hardware(f"{stem}_capstan", cap, frame, fab="purchased",
                                   bom_key=self.sleeve_key, color=ALU))
        arc = 2 * self.drum_r * math.radians(hub.travel_deg + 2 * ANCHOR_MARGIN)
        cord_m = (arc + WRAPS * math.pi * self.capstan_d + 200.0) / 1000.0
        out.extras.append(BomLine("dyneema_1mm", round(cord_m, 3),
                                  f"{group.name}: the capstan's cord"))
        out.extras.append(BomLine("epoxy_2part", 0.003,
                                  f"{group.name}: the sleeve on the shaft"))

    def _bore(self, ctx: Context) -> float:
        """The drum's bore: clear of the pivot's axle (its radius and the running fit)."""
        from spiderpig import construction

        a = construction.axle(ctx.config.pillar).dims(ctx, True)
        return 2 * a.axle + ctx.params.running_fit

    def error_terms(self, ctx: Context, hub: DriveHub) -> ErrorTerms:
        free = (self.drum_r * math.radians(hub.travel_deg + ANCHOR_MARGIN)
                + (self.drum_r - self.bolt_pcd / 2))                  # the worst strand
        stiff = 2 * (CORD_EA_N / free) * self.drum_r ** 2 / 1000.0     # N·m/rad
        return ErrorTerms(stiffness_nm_per_rad=stiff,
                          source=f"capstan: {self.drum_r:g} mm drum, {self.capstan_d:g} mm "
                                 "capstan, 1 mm Dyneema",
                          notes=(f"two strands up to {free:.0f} mm at EA {CORD_EA_N:g} N "
                                 "(unverified)", "no backlash: the cord never leaves the "
                                 "capstan"))

    def assembly_text(self, hub: DriveHub) -> str:
        side = "under" if hub.down else "over"
        return (f"Bond the 6 mm tube over the motor's shaft (epoxy), level with the drum's "
                f"core. Stack the drum's plates {side} its link's pad and bolt them through "
                "it, the two anchor bolts loose (four M3 screws from the drum's far face, nuts "
                "on the link); lead the cord in through one notch round its bolt, round the groove "
                f"to the sleeve, {WRAPS} turns round it, back round the groove and in through "
                "the other notch round its bolt; pull it taut and tighten both anchor bolts "
                "to pinch it.")
