"""``belt``: a GT2 timing belt from the motor's pulley to a plate pulley bolted to the link.

The small pulley (16 teeth, a 16 mm hub with set screws, one on the shaft's flat) on the
motor, its hub toward the link (away from the plate, where the motor's boss is); the large
one (80 teeth: 5:1) bolted to the driven link with four M3 screws into its tapped holes; a
stock closed-loop belt (:data:`hardware.motion_catalog.GT2_LOOPS`):
the shortest whose centre distance keeps the motor clear of the pivot's pillar heads and
the pulleys apart, the motor's slots in the outer plate tensioning it (±``TENSION``).

A loop of length ``L`` over pitch diameters ``D`` and ``d`` sits at the centre distance
``C`` that solves ``L = 2 C + pi (D + d) / 2 + (D - d)² / (4 C)``.

**Its error terms.** Both spans pretensioned, each a spring ``c / l`` (``c`` the belt's
specific stiffness, ``l`` the free span, the tangent length ``sqrt(C² - ((D - d)/2)²)``),
on the large pulley's pitch radius: ``2 (c / l) (D / 2)²`` at the joint. A tensioned GT2
belt's backlash is its teeth's clearance in the grooves: ``BACKLASH_MM`` at the small
pulley's pitch line (an assumption: measure it), divided by the ratio at the joint.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig.construction.base import Build, Context, Realized, hardware
from spiderpig.construction.joints import ErrorTerms
from spiderpig.construction.transmissions.base import ALU, DriveGroup, DriveHub, Transmission
from spiderpig.hardware.catalog import get
from spiderpig.hardware.motion_catalog import (
    GT2_LOOPS,
    GT2_SPECIFIC_STIFFNESS_N,
    gt2_loop,
)
from spiderpig.shapes import ring, union

BACKLASH_MM = 0.02
"""A tensioned GT2 belt's lost motion at the small pulley's pitch line (mm): its teeth's
clearance in the grooves. An assumption (Gates rates 2MGT 'low backlash'); measure it."""
BELT_COLOR = "#202020"


@dataclass(frozen=True)
class BeltDrive(Transmission):
    """A GT2 belt, ``small`` to ``large`` teeth (module docstring)."""

    key: str = "belt"
    label: str = ("GT2 6 mm timing belt, 16 to 80 teeth (5:1): plate pulley bolted to the "
                  "link, stock loop, motor slots to tension")
    small_key: str = "pulley_gt2_16t_5mm"
    large_key: str = "pulley_gt2_80t_8mm"
    bolts: int = 4
    bolt_pcd: float = 30.0
    nuts: bool = False

    def _p(self, key: str) -> dict:
        return get(key).dims

    @property
    def big(self) -> dict:
        return self._p(self.large_key)

    @property
    def small(self) -> dict:
        return self._p(self.small_key)

    def width(self) -> float:
        return float(self.big["toothed_w"]) + 2 * float(self.big["flange_t"])

    def layers(self, ctx: Context) -> int:
        return math.ceil(self.width() / ctx.pitch - 1e-9)

    def hub_r(self, ctx: Context) -> float:
        return float(self.big["flange_d"]) / 2

    def motor_side_r(self, ctx: Context) -> float:
        return float(self.small["flange_d"]) / 2

    def motor_hub(self, ctx: Context) -> tuple[int, float]:
        """The small pulley's hub: the layers it takes past the teeth, and its radius."""
        hub_l = float(self.small["hub_l"])
        return math.ceil(hub_l / ctx.pitch - 1e-9), float(self.small["hub_d"]) / 2

    def centre_distance(self, length: float) -> float | None:
        """The centre distance a loop ``length`` long sits at (``None``: too short)."""
        big, small = float(self.big["pitch_d"]), float(self.small["pitch_d"])
        a = length - math.pi * (big + small) / 2
        disc_ = a * a - 2 * (big - small) ** 2
        if a <= 0 or disc_ < 0:
            return None
        return (a + math.sqrt(disc_)) / 4

    def distances(self, ctx: Context, motor) -> list[tuple[float, int]]:
        """Every stock loop's centre distance that keeps the motor's body clear of the
        pivot's pillar head and the pulleys apart, shortest first."""
        need = max(motor.frame / 2 + 5.0 + 1.0,
                   (float(self.big["flange_d"]) + float(self.small["flange_d"])) / 2
                   + ctx.params.margin)
        out = []
        for length in GT2_LOOPS:
            c = self.centre_distance(length)
            if c is not None and c >= need:
                out.append((c, length))
        return out

    def ratio(self, detail) -> float:
        return float(self.big["teeth"]) / float(self.small["teeth"])

    def hull(self, ctx: Context, hub: DriveHub) -> list[float]:
        """The belt's envelope along the line pivot-motor (five radii): the tangent lines'
        distance from it grows linearly between the pulleys, ``1 / cos`` of their angle
        over the radii's (the flanges' radii at the ends)."""
        big, small = float(self.big["pitch_d"]) / 2, float(self.small["pitch_d"]) / 2
        c = math.dist(hub.motor, ctx.topo.geometry.points[hub.axis][0])
        sin_a = (big - small) / c
        k = 1.0 / math.sqrt(max(1e-9, 1 - sin_a * sin_a))
        t = 1.4                                       # the belt's thickness
        rs = [k * (big + t + f * (small - big)) for f in np.linspace(0.0, 1.0, 5)]
        rs[0] = max(rs[0], self.hub_r(ctx))
        rs[-1] = max(rs[-1], self.motor_side_r(ctx))
        return [float(r) for r in rs]

    def realize(self, build: Build, group: DriveGroup, hub: DriveHub, out: Realized,
                layers: list[int]) -> None:
        z0, z1 = build.z(layers[0])[0], build.z(layers[-1])[1]
        zm = (z0 + z1) / 2
        w = self.width()
        o = tuple(build.xy(hub.axis))
        m = hub.motor
        big, small = self.big, self.small
        frame = build.plan.topo.frame_bodies[0]
        stem = group.name.replace(":", "_")
        bore = float(big["bore"])
        ft = float(big["flange_t"])
        large = union([ring(o, float(big["flange_d"]), bore, zm - w / 2, zm - w / 2 + ft),
                       ring(o, float(big["od"]), bore, zm - w / 2 + ft, zm + w / 2 - ft),
                       ring(o, float(big["flange_d"]), bore, zm + w / 2 - ft, zm + w / 2)])
        out.bodies.append(hardware(f"{stem}_pulley_large", large, group.link,
                                   fab="purchased", bom_key=self.large_key, color=ALU))
        sft = float(small["flange_t"])
        sb = float(small["bore"])
        hub_l = float(small["hub_l"])
        hub_z = ((zm + w / 2, zm + w / 2 + hub_l) if hub.down       # its hub toward the link
                 else (zm - w / 2 - hub_l, zm - w / 2))
        small_part = union([
            ring(m, float(small["hub_d"]), sb, *hub_z),
            ring(m, float(small["flange_d"]), sb, zm - w / 2, zm - w / 2 + sft),
            ring(m, float(small["od"]), sb, zm - w / 2 + sft, zm + w / 2 - sft),
            ring(m, float(small["flange_d"]), sb, zm + w / 2 - sft, zm + w / 2)])
        out.bodies.append(hardware(f"{stem}_pulley_small", small_part, frame,
                                   fab="purchased", bom_key=self.small_key, color=ALU))
        length = hub.detail
        belt = _belt_loop(o, m, float(big["pitch_d"]) / 2, float(small["pitch_d"]) / 2,
                          zm - 3.0, zm + 3.0)
        out.bodies.append(hardware(f"{stem}_belt", belt, frame, fab="purchased",
                                   bom_key=gt2_loop(int(length)), color=BELT_COLOR))

    def error_terms(self, ctx: Context, hub: DriveHub) -> ErrorTerms:
        big, small = float(self.big["pitch_d"]) / 2, float(self.small["pitch_d"]) / 2
        c = math.dist(hub.motor, ctx.topo.geometry.points[hub.axis][0])
        span = math.sqrt(max(c * c - (big - small) ** 2, 1e-9))
        k_span = GT2_SPECIFIC_STIFFNESS_N / span                     # N/mm
        stiff = 2 * k_span * big * big / 1000.0                      # N·m/rad
        back = math.degrees(BACKLASH_MM / small) / hub.ratio
        return ErrorTerms(backlash_deg=back, stiffness_nm_per_rad=stiff,
                          source=f"belt: GT2 {int(hub.detail)} mm loop, "
                                 f"{int(self.small['teeth'])}:{int(self.big['teeth'])}",
                          notes=(f"two {span:.1f} mm spans at {GT2_SPECIFIC_STIFFNESS_N:g} N "
                                 "per unit strain (unverified)",
                                 f"{BACKLASH_MM:g} mm at the small pulley (assumed)"))

    def assembly_text(self, hub: DriveHub) -> str:
        side = "under" if hub.down else "over"
        return (f"Bolt the 80-tooth pulley {side} its link (four M3 screws through the link "
                "into the pulley); put the 16-tooth pulley on the motor's shaft, its hub away "
                f"from the plate, a set screw on the flat, level with the large one; loop the "
                f"{int(hub.detail)} mm belt "
                "over both and slide the motor out along its slots until the belt is taut "
                "(it should twist no more than a quarter turn mid-span), then tighten them.")


def _belt_loop(o, m, r_big: float, r_small: float, z0: float, z1: float):
    """The belt: the band round both pulleys' pitch circles (a hull ring 1.4 mm thick)."""
    from build123d import Location, Polyline, extrude, make_face

    def hull_pts(rb: float, rs: float) -> list[tuple[float, float]]:
        pts = [(c[0] + r * math.cos(a), c[1] + r * math.sin(a))
               for c, r in ((o, rb), (m, rs))
               for a in np.linspace(0, 2 * math.pi, 72, endpoint=False)]
        return _convex(pts)

    def face(pts):
        return make_face(Polyline(*pts, close=True).edges())

    outer = face(hull_pts(r_big + 0.7, r_small + 0.7))
    inner = face(hull_pts(r_big - 0.7, r_small - 0.7))
    band = extrude(outer.moved(Location((0, 0, z0))), amount=z1 - z0)
    hole = extrude(inner.moved(Location((0, 0, z0 - 1))), amount=z1 - z0 + 2)
    return band - hole


def _convex(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The convex hull (Andrew's monotone chain), counter-clockwise."""
    pts = sorted(set(pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]
