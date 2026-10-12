"""``belt``: a GT2 timing belt from the motor's pulley to a plate pulley bolted to the link.

The small pulley (16 teeth, a 16 mm hub with set screws, one on the shaft's flat) on the
motor, its hub toward the plate (a layer clear of it: the motor's boss); the large
one (80 teeth: 5:1) bolted to the driven link's pad with four M3 screws into holes drilled
and tapped on a 30 mm circle; a stock closed-loop belt
(:data:`hardware.motion_catalog.GT2_LOOPS`): the shortest whose centre distance keeps the
pulleys apart and the motor clear of the pillar heads. The motor stands on a slider plate
outside its frame plate (``tensioned``: :meth:`DriveGroup._slider`), clamped by two screws
from outside: the belt is tensioned (±``TENSION``) any time, the stack built.

A loop of length ``L`` over pitch diameters ``D`` and ``d`` sits at the centre distance
``C`` that solves ``L = 2 C + pi (D + d) / 2 + (D - d)² / (4 C)``.

**Its error terms.** Both spans pretensioned, each a spring ``c / l`` (``c`` the belt's
specific stiffness, ``l`` the free span, the tangent length ``sqrt(C² - ((D - d)/2)²)``),
on the large pulley's pitch radius: ``2 (c / l) (D / 2)²`` at the joint. A tensioned GT2
belt's backlash is its teeth's clearance in the grooves, ``BACKLASH_MM`` at each mesh's
pitch line (an assumption: measure it): the two meshes' at the joint, ``2 b / (D / 2)``.
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
from spiderpig.shapes import drill, ring, union

BACKLASH_MM = 0.02
"""A tensioned GT2 belt's lost motion at the small pulley's pitch line (mm): its teeth's
clearance in the grooves. An assumption (Gates rates 2MGT 'low backlash'); measure it."""
BELT_COLOR = "#202020"


@dataclass(frozen=True)
class BeltDrive(Transmission):
    """A GT2 belt, ``small`` to ``large`` teeth (module docstring)."""

    key: str = "belt"
    label: str = ("GT2 6 mm timing belt, 16 to 80 teeth (5:1): plate pulley bolted to the "
                  "link, stock loop, the motor on a slider tensioned from outside")
    small_key: str = "pulley_gt2_16t_5mm"
    large_key: str = "pulley_gt2_80t_8mm"
    bolts: int = 4
    bolt_pcd: float = 30.0
    nuts: bool = False
    tensioned: bool = True

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

    def layers(self, ctx: Context, travel_deg: float = 360.0) -> int:
        return math.ceil(self.width() / ctx.pitch - 1e-9)

    def check(self, ctx: Context, hub: DriveHub) -> None:
        """The large pulley's bore clears the pivot's axle; the pulleys clear each other."""
        from spiderpig import construction
        from spiderpig.construction.base import ConstructionError

        a = construction.axle(ctx.config.pillar).dims(ctx, True)
        if float(self.big["bore"]) < 2 * a.axle + ctx.params.running_fit - 1e-9:
            raise ConstructionError(f"belt: the {self.big['bore']:g} mm pulley bore doesn't "
                                    f"clear the pivot's {2 * a.axle:g} mm axle")
        c = math.dist(hub.motor, ctx.topo.geometry.points[hub.axis][0])
        if (float(self.big["flange_d"]) + float(self.small["flange_d"])) / 2 + 0.5 > c:
            raise ConstructionError("belt: the pulleys' flanges meet")

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
        need = ((float(self.big["flange_d"]) + float(self.small["flange_d"])) / 2
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
        holes = [(tuple(p[0]), 2.5 / 2) for p in group.bolt_points(
            np.asarray(o), np.asarray(build.xy(group._other(build.ctx))), hub)]
        large = drill(large, holes, zm - w / 2, zm + w / 2)     # drilled and tapped M3
        out.bodies.append(hardware(f"{stem}_pulley_large", large, group.link,
                                   fab="purchased", bom_key=self.large_key, color=ALU))
        sft = float(small["flange_t"])
        sb = float(small["bore"])
        hub_l = float(small["hub_l"])
        hub_z = ((zm - w / 2 - hub_l, zm - w / 2) if hub.down       # its hub toward the plate
                 else (zm + w / 2, zm + w / 2 + hub_l))
        small_part = union([
            ring(m, float(small["hub_d"]), sb, *hub_z),
            ring(m, float(small["flange_d"]), sb, zm - w / 2, zm - w / 2 + sft),
            ring(m, float(small["od"]), sb, zm - w / 2 + sft, zm + w / 2 - sft),
            ring(m, float(small["flange_d"]), sb, zm + w / 2 - sft, zm + w / 2)])
        out.bodies.append(hardware(f"{stem}_pulley_small", small_part, frame,
                                   fab="purchased", bom_key=self.small_key, color=ALU))
        length = hub.detail
        belt = _belt_loop(o, m, float(big["od"]) / 2, float(small["od"]) / 2,
                          zm - 3.0, zm + 3.0)
        out.bodies.append(hardware(f"{stem}_belt", belt, frame, fab="purchased",
                                   bom_key=gt2_loop(int(length)), color=BELT_COLOR))

    def error_terms(self, ctx: Context, hub: DriveHub) -> ErrorTerms:
        c = math.dist(hub.motor, ctx.topo.geometry.points[hub.axis][0])
        return self.terms(c, int(hub.detail))

    def terms(self, c: float, length: int) -> ErrorTerms:
        """The terms of a ``length`` mm loop at centre distance ``c`` (module docstring):
        the planar drives' and the turntable's (:mod:`construction.rotary`)."""
        big, small = float(self.big["pitch_d"]) / 2, float(self.small["pitch_d"]) / 2
        span = math.sqrt(max(c * c - (big - small) ** 2, 1e-9))
        k_span = GT2_SPECIFIC_STIFFNESS_N / span                     # N/mm
        stiff = 2 * k_span * big * big / 1000.0                      # N·m/rad
        back = math.degrees(2 * BACKLASH_MM / big)       # both meshes, seen at the joint
        return ErrorTerms(backlash_deg=back, stiffness_nm_per_rad=stiff,
                          source=f"belt: GT2 {length} mm loop, "
                                 f"{int(self.small['teeth'])}:{int(self.big['teeth'])}",
                          notes=(f"two {span:.1f} mm spans at {GT2_SPECIFIC_STIFFNESS_N:g} N "
                                 "per unit strain (unverified)",
                                 f"{BACKLASH_MM:g} mm at each mesh (assumed)"))

    def assembly_text(self, hub: DriveHub) -> str:
        side = "under" if hub.down else "over"
        return (f"Drill and tap the 80-tooth pulley's four M3 holes on a 30 mm circle, and "
                f"bolt it {side} its link's pad (screws through the pad into it); put the "
                "16-tooth pulley on the motor's shaft, its hub toward the plate, a set "
                "screw on the flat, level with the large one; loop the "
                f"{int(hub.detail)} mm belt over both. Once the stack is built, slide the "
                "motor's slider out (its clamp screws from outside) until the belt is taut (a "
                "quarter turn's twist at mid-span, no more), then tighten them.")


def _belt_loop(o, m, r_big: float, r_small: float, z0: float, z1: float):
    """The belt: the band round both pulleys' teeth (``r_*`` their outside radii), 1.38 mm
    thick (its teeth in the grooves left out)."""
    from build123d import Location, Polyline, extrude, make_face

    def hull_pts(rb: float, rs: float) -> list[tuple[float, float]]:
        pts = [(c[0] + r * math.cos(a), c[1] + r * math.sin(a))
               for c, r in ((o, rb), (m, rs))
               for a in np.linspace(0, 2 * math.pi, 72, endpoint=False)]
        return _convex(pts)

    def face(pts):
        return make_face(Polyline(*pts, close=True).edges())

    k = 1.0 / math.cos(math.pi / 72)          # a 72-gon's chords stay outside its circle
    outer = face(hull_pts(r_big + 1.39, r_small + 1.39))
    inner = face(hull_pts(k * (r_big + 0.01), k * (r_small + 0.01)))
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
