"""``capstan``: a Dyneema cord wrapped on the motor's capstan, anchored on a drum bolted to
the link.

The drum is a stack of laser-cut plates in the layers under the driven link (a flange,
``core`` layers of the drum's radius, a flange: the cord runs in the groove they make),
bolted through the link with four M3 screws and nuts under it. The capstan is the motor's
own 5 mm shaft by default (a larger sleeve on it, ``capstan_d``, lowers the ratio; printed
here, a turned aluminium one is better), touching the drum through the cord: the motor sits
at ``R + r + d`` from the pivot. The cord leaves each
end of the drum's groove, wraps the capstan ``WRAPS`` turns (friction grows as
``e^{mu theta}``: five turns hold with a small pretension) and is tied off at an anchor
screw near each end of the drum's arc; one anchor tensions it.

The ratio is ``(R + d / 2) / (r + d / 2)``: 10:1 at the default 30 mm drum on the bare
5 mm shaft, with no backlash (the cord never leaves the capstan). (The drum's radius is
bounded by the frame: the palletizer's levelling pillar stands 40 mm from the shoulder,
and its spacers must pass the drum's flange.)

**Its error terms.** Two strands pretensioned, each ``EA / l`` (``l`` the free cord from
the capstan round the drum to its anchor: half the arc the input sweeps, the worst), on
the drum's radius: ``2 (EA / l) R²`` at the joint; no backlash.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from spiderpig.construction.base import Build, Context, Realized, hardware
from spiderpig.construction.joints import ErrorTerms
from spiderpig.construction.transmissions.base import DriveGroup, DriveHub, Transmission
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.motion_catalog import CORD_D, CORD_EA_N
from spiderpig.shapes import drill, ring

WRAPS = 5
FLANGE = 2.0          # the drum's flanges stand this far over its core (mm)
DRUM_COLOR = "#7fb3e0"
CAPSTAN_COLOR = "#1baf7a"


@dataclass(frozen=True)
class CapstanDrive(Transmission):
    """A cord capstan: drum radius ``drum_r``, capstan diameter ``capstan_d`` (module
    docstring)."""

    key: str = "capstan"
    label: str = ("Dyneema cord capstan, 10:1: a laser-cut drum bolted to the link, the cord "
                  "wrapped on the motor's shaft, no backlash")
    drum_r: float = 30.0
    capstan_d: float = 5.0   # the motor's shaft itself (a sleeve: larger, a lower ratio)
    core: int = 2            # core layers (the cord's wraps across them)
    bolts: int = 4
    bolt_pcd: float = 30.0
    nuts: bool = True

    def layers(self, ctx: Context) -> int:
        return self.core + 2

    def hub_r(self, ctx: Context) -> float:
        return self.drum_r + FLANGE

    def motor_side_r(self, ctx: Context) -> float:
        return self.capstan_d / 2 + 0.5

    def distances(self, ctx: Context, motor) -> list[tuple[float, None]]:
        return [(self.drum_r + self.capstan_d / 2 + CORD_D, None)]

    def ratio(self, detail) -> float:
        return (self.drum_r + CORD_D / 2) / (self.capstan_d / 2 + CORD_D / 2)

    def _bore(self, ctx: Context) -> float:
        """The drum's bore: clear of the pivot's axle (its seat radius and the running
        fit)."""
        from spiderpig.construction import axle as axle_of

        a = axle_of(ctx.config.pillar).dims(ctx, True)
        return 2 * (a.seat or a.axle) + ctx.params.running_fit

    def realize(self, build: Build, group: DriveGroup, hub: DriveHub, out: Realized,
                layers: list[int]) -> None:
        ctx = build.ctx
        o = tuple(build.xy(hub.axis))
        stem = group.name.replace(":", "_")
        bore = self._bore(ctx)
        sheet = ctx.sheet("link")
        holes = [tuple(p[0]) for p in group.bolt_points(build.xy(hub.axis),
                                                        build.xy(group._other(ctx)))]
        for i, k in enumerate(layers):
            z0, z1 = build.z(k)
            flange = i in (0, len(layers) - 1)
            r = self.drum_r + (FLANGE if flange else 0.0)
            drilled = [(h, 3.4 / 2) for h in holes]
            plate_side = 0 if hub.down else len(layers) - 1
            if flange and i == plate_side:    # the anchors in the flange nearer the plate
                for a in (hub.angle + math.pi * 0.75, hub.angle - math.pi * 0.75):
                    xy = (o[0] + (self.drum_r - 4) * math.cos(a),
                          o[1] + (self.drum_r - 4) * math.sin(a))
                    drilled.append((xy, 3.4 / 2))
            part = drill(ring(o, 2 * r, bore, z0, z1), drilled, z0, z1)
            out.bodies.append(hardware(f"{stem}_drum{i}", part, group.link, fab="laser",
                                       color=DRUM_COLOR, sheet=sheet))
        z0, z1 = build.z(layers[1])[0], build.z(layers[-2])[1]
        frame = build.plan.topo.frame_bodies[0]
        if self.capstan_d > group.motor.shaft_d + 1e-9:     # a sleeve on the shaft
            cap = ring(hub.motor, self.capstan_d, group.motor.shaft_d + 0.1, z0, z1)
            out.bodies.append(hardware(f"{stem}_capstan", cap, frame, fab="printed",
                                       color=CAPSTAN_COLOR))
        arc = 2 * math.pi * self.drum_r * 0.75          # the groove the cord lies in
        cord_m = (arc + WRAPS * math.pi * self.capstan_d + 200.0) / 1000.0
        out.extras.append(BomLine("dyneema_1mm", round(cord_m, 3),
                                  f"{group.name}: the capstan's cord"))
        out.extras.append(BomLine("m3_nut", 2, f"{group.name}: the cord's anchors"))

    def error_terms(self, ctx: Context, hub: DriveHub) -> ErrorTerms:
        lk = ctx.config.lk
        name = next(k for k, (link, _) in lk.drives.items() if link == hub.link)
        lo, hi = lk.ranges[name]
        free = self.drum_r * math.radians(hi - lo) / 2 + 10.0      # the worst strand
        stiff = 2 * (CORD_EA_N / free) * self.drum_r ** 2 / 1000.0  # N·m/rad
        return ErrorTerms(stiffness_nm_per_rad=stiff,
                          source=f"capstan: {self.drum_r:g} mm drum, {self.capstan_d:g} mm "
                                 "capstan, 1 mm Dyneema",
                          notes=(f"two strands up to {free:.0f} mm at EA {CORD_EA_N:g} N "
                                 "(unverified)", "no backlash: the cord never leaves the "
                                 "capstan"))

    def assembly_text(self, hub: DriveHub) -> str:
        side = "under" if hub.down else "over"
        return (f"Stack the drum's plates {side} its link and bolt them through it (four M3 "
                f"screws, nuts on the drum's far face); tie "
                f"the cord to one anchor, lay it round the groove to the motor's shaft, wrap it "
                f"{WRAPS} turns, back round the groove to the other anchor and tie it off "
                "taut (turn the anchor screw to tension it).")
