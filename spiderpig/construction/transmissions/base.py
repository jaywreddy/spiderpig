"""The drive group and what its transmissions share (:mod:`construction.transmissions`)."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from spiderpig.actuators import sensors
from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    Group,
    Realized,
    hardware,
    stock_length,
)
from spiderpig.construction.joints import ErrorTerms
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.shapes import Cut, Rect, box, cut_holes, disc, pill, union
from spiderpig.stack import Claim, Disc, Layout, Pill, Placed, Unbuildable

STEEL = "#4a4a4a"
MOTOR_COLOR = "#2b2b2b"
ALU = "#c9cdd2"
SLIDER_COLOR = "#d8d8d8"
MOTOR_ANGLES = (-30.0, -150.0, -60.0, -120.0, -10.0, -170.0, -90.0, 15.0, 165.0)
"""Where a drive tries its motor, in degrees from +x about its pivot, in order: under the
shoulder first (an arm's base is below it), then round."""
HEAD_CLEAR = 1.0     # a motor body's clearance to the frame's pillar heads and to another motor
SCREW_HOLE = 3.4     # an M3 clearance hole (ISO 273 medium)
TAP_HOLE = 2.5       # an M3 tap drill: the frame plate's holes the slider's screws thread into
TENSION = 2.0        # a tensioned motor's slider travel either way along the belt (mm)
HARD_STOP = (("m3_shcs_10", 1.0), ("m3_nut", 1.0))
"""What a drive buys for its hard stop at the end of its travel when its homing sensor
homes on one (:attr:`actuators.sensors.HomingSensor.needs_stop`: a stall): an M3 screw and
nut (the turntable's stop is the turret's own, :mod:`construction.rotary.base_pair`)."""
ENGAGE = 1.2         # the least thread a motor screw takes in the motor, in diameters
PAD_EDGE = 2.0       # a pad's web round the bolt holes it carries (mm)


@dataclass(frozen=True)
class DriveHub:
    """What a drive publishes (``ctx.interfaces["drive:<input>"]``): the pivot's axis, the
    driven link, how many layers the hub's parts take beside it and on which side
    (``side`` -1: under it, the motor under the outer plate; +1: over it, the motor on the
    inner plate; the axle only seats inside them), the motor's place and the radius of
    what stands outside the plate there, the reduction, the transmission's own record
    (``detail``), the input's travel and its link's direction at mid travel, and the pad
    the driven link grows round the pivot for the hub's bolts
    (:class:`construction.plates.LinkPlates`)."""

    group: str
    axis: str
    link: str
    below: int
    motor: tuple[float, float]
    motor_r: float
    angle: float                 # the motor's direction from the pivot (rad)
    ratio: float
    detail: Any = None
    side: int = -1
    travel_deg: float = 360.0
    mid_dir: float = 0.0         # the link's direction at mid travel (rad)
    pad_r: float = 0.0

    @property
    def down(self) -> bool:
        return self.side < 0

    def layers(self, k: int) -> list[int]:
        """The hub's layers beside the link's layer ``k``, from the link out."""
        return [k + self.side * i for i in range(1, self.below + 1)]


class Transmission:
    """The contract a transmission implements (the drive group does the rest).

    ``layers(ctx, travel_deg)``: the hub layers its parts take beside the link.
    ``hub_r(ctx)`` / ``motor_side_r(ctx)``: the radii it claims in them at the pivot and at
    the motor. ``distances(ctx, motor)``: candidate centre distances (pivot to motor shaft),
    each with its own record, in order of preference. ``ratio(detail)``.
    ``hull(ctx, hub)``: the radii along the line pivot-motor in the hub layers (a belt's
    spans), or ``None``. ``motor_hub(ctx)``: the layers and radius the motor-side part takes
    past the hub layers, toward the plate.
    ``bolt_angles(hub)``: the hub's bolts' angles in the link's frame (``None``: evenly
    spaced). ``check(ctx, hub)``: its own parts' clearances to each other (the planner
    never checks a group's shapes against each other): :class:`ConstructionError`.
    ``realize(build, group, hub, out, layers)``; ``error_terms(ctx, hub)``;
    ``assembly_text(hub)``. ``tensioned``: the motor on a slider plate outside its frame
    plate, clamped by screws from outside (a belt: tensioned after the stack is built),
    else bolted straight to the plate from inside. ``bolts`` / ``bolt_pcd``: the screws
    that hold the hub to the link and their circle; ``nuts``: nuts beyond the hub (else
    they thread into it).
    """

    key: str = ""
    label: str = ""
    bolts: int = 4
    bolt_pcd: float = 30.0
    nuts: bool = False
    tensioned: bool = False

    def layers(self, ctx: Context, travel_deg: float) -> int:
        raise NotImplementedError

    def hub_r(self, ctx: Context) -> float:
        raise NotImplementedError

    def motor_side_r(self, ctx: Context) -> float:
        raise NotImplementedError

    def distances(self, ctx: Context, motor) -> list[tuple[float, Any]]:
        raise NotImplementedError

    def ratio(self, detail) -> float:
        raise NotImplementedError

    def hull(self, ctx: Context, hub: DriveHub) -> list[float] | None:
        return None

    def motor_hub(self, ctx: Context) -> tuple[int, float]:
        return 0, 0.0

    def bolt_angles(self, hub: DriveHub) -> list[float] | None:
        return None

    def check(self, ctx: Context, hub: DriveHub) -> None:
        return None

    def realize(self, build: Build, group: JointDrive, hub: DriveHub, out: Realized,
                layers: list[int]) -> None:
        raise NotImplementedError

    def error_terms(self, ctx: Context, hub: DriveHub) -> ErrorTerms:
        raise NotImplementedError

    def assembly_text(self, hub: DriveHub) -> str:
        return ""


def half_diagonal(motor) -> float:
    return motor.frame / math.sqrt(2.0)


def motor_ends(m: np.ndarray, angle: float, motor) -> tuple[np.ndarray, np.ndarray]:
    """The motor's square as a pill: its ends, half a side either way across the drive
    (with the radius half a side it covers the square, faces square to the drive)."""
    v = np.array([-math.sin(angle), math.cos(angle)])
    h = motor.frame / 2
    return m - h * v, m + h * v


def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _segs_dist(a: tuple, b: tuple) -> float:
    return min(_seg_dist(a[0], *b), _seg_dist(a[1], *b), _seg_dist(b[0], *a),
               _seg_dist(b[1], *a))


def place_motor(o: np.ndarray, distances: list[tuple[float, Any]], motor,
                heads: list[tuple[np.ndarray, float]], motors: list[tuple[tuple, float]],
                angles=MOTOR_ANGLES, reach: float | None = None
                ) -> tuple[np.ndarray, float, float, Any]:
    """The first (centre distance, angle) whose motor body (a pill across the drive, radius
    ``reach``, else half its side: :func:`motor_ends`) clears every disc in ``heads`` (the
    frame's pillar heads) and every pill in ``motors`` (the other drives' on that plate) by
    :data:`HEAD_CLEAR`: ``(motor xy, angle, distance, record)``. The links' sweeps are the
    planner's: a motor's parts stand in layers it gives them."""
    r = motor.frame / 2 if reach is None else reach
    for dist, detail in distances:
        for a in angles:
            th = math.radians(a)
            m = o + dist * np.array([math.cos(th), math.sin(th)])
            ends = motor_ends(m, th, motor)
            if all(_seg_dist(p, *ends) >= r + hr + HEAD_CLEAR for p, hr in heads) and \
                    all(_segs_dist(ends, e) >= r + mr + HEAD_CLEAR for e, mr in motors):
                return m, th, dist, detail
    raise ConstructionError("no place round the pivot for the drive's motor: it meets the "
                            "frame's pillar heads or another drive's motor everywhere")


def screw_length(sk, grip: float, depth: float) -> float:
    """A motor screw's stock length through ``grip`` mm of plate into a ``depth`` mm blind
    hole: at least :data:`ENGAGE` diameters of thread, never bottoming (0.3 mm short)."""
    lo, hi = grip + ENGAGE * sk.d, grip + depth - 0.3
    for length in sk.lengths:
        if lo - 1e-6 <= length <= hi + 1e-6:
            return float(length)
    raise ConstructionError(f"no stock M{sk.d:g} screw goes through {grip:g} mm into the "
                            f"motor's {depth:g} mm holes with {ENGAGE:g} d of thread and "
                            f"without bottoming (lengths {list(sk.lengths)})")


@dataclass
class JointDrive(Group):
    """One arm input's drive: a stepper on a frame plate and its transmission to the driven
    link's hub (module docstring). The drives on one pivot alternate sides: the first's
    motor under the outer plate (its hub under its link), the next's on the inner plate
    (its hub over its link), so neither's hub stands in the other's way and each motor's
    shaft reaches its hub from its own plate."""

    input: str
    link: str
    about: str
    transmission: Transmission
    motor: Any                         # actuators.StepperSpec
    cut_shaft: bool = True             # the stock shaft cut to the hub (else its tip claimed)
    name: str = field(init=False)

    def __post_init__(self) -> None:
        self.name = f"drive:{self.input}"

    # -- where ---------------------------------------------------------------------

    def _axis(self, ctx: Context) -> str:
        return ctx.topo.point_of[(self.link, self.about)]

    def _other(self, ctx: Context) -> str:
        """The driven link's other joint's point (its direction from the pivot)."""
        joints = ctx.config.lk.links[self.link][0]
        j = next(j for j in joints if j != self.about)
        return ctx.topo.point_of[(self.link, j)]

    def _travel(self, ctx: Context) -> tuple[float, float]:
        """The input's travel (deg) and its link's direction at mid travel (rad)."""
        lk = ctx.config.lk
        lo, hi = lk.ranges.get(self.input, (0.0, 360.0))
        pts = lk.solve(params=dict(zip(lk.params, ctx.config.values, strict=True))).at(
            *[np.array([math.radians((a + b) / 2)]) for a, b in
              (lk.ranges.get(k, (0.0, 360.0)) for k in lk.inputs)])    # (a crank's: a turn)
        joints = lk.links[self.link][0]
        other = next(j for j in joints if j != self.about)
        d = pts[other][0] - pts[self.about][0]
        return hi - lo, math.atan2(d[1], d[0])

    def slider_t(self, ctx: Context) -> float:
        return ctx.sheet_t("frame") if self.transmission.tensioned else 0.0

    def interface(self, ctx: Context) -> DriveHub:
        from spiderpig import construction

        pts = ctx.topo.geometry.points
        axis = self._axis(ctx)
        if axis not in {a.name for a in ctx.topo.axes_of("frame")}:
            raise ConstructionError(f"{self.name}: {self.link} turns about {axis}, not a frame "
                                    "pivot: a drive's motor stands on the frame")
        o = np.asarray(pts[axis][0], dtype=float)
        before = [h for h in ctx.interfaces.values() if isinstance(h, DriveHub)]
        side = -1 if sum(h.axis == axis for h in before) % 2 == 0 else 1
        head_r = construction.axle(ctx.config.pillar).dims(ctx, True).head
        heads = [(np.asarray(pts[a.name][0], dtype=float), head_r)
                 for a in ctx.topo.axes_of("frame")]
        reach = self._outside_r()
        motors = [(motor_ends(np.asarray(h.motor), h.angle, self.motor), h.motor_r)
                  for h in before if h.side == side]
        m, th, _, detail = place_motor(o, self.transmission.distances(ctx, self.motor),
                                       self.motor, heads, motors, reach=reach)
        travel, mid = self._travel(ctx)
        sk = screw("bhcs", "3")
        pad = self.transmission.bolt_pcd / 2 + max(SCREW_HOLE / 2 + PAD_EDGE, sk.head_d / 2)
        hub = DriveHub(self.name, axis, self.link, self.transmission.layers(ctx, travel),
                       (float(m[0]), float(m[1])), reach, th,
                       self.transmission.ratio(detail), detail, side, travel, mid,
                       max(pad, ctx.params.link_radius))
        self.transmission.check(ctx, hub)
        return hub

    def _outside_r(self) -> float:
        """The radius of what stands outside the plate at the motor (a pill across the
        drive): the motor's square, or the slider plate's stadium round it (and its
        travel)."""
        if self.transmission.tensioned:
            return self.motor.frame / 2 + TENSION + 2.0
        return self.motor.frame / 2

    def hub(self, ctx: Context) -> DriveHub:
        h = ctx.interfaces.get(self.name)
        return h if isinstance(h, DriveHub) else self.interface(ctx)

    def _uv(self, hub: DriveHub) -> tuple[np.ndarray, np.ndarray]:
        u = np.array([math.cos(hub.angle), math.sin(hub.angle)])
        return u, np.array([-u[1], u[0]])

    def screw_points(self, hub: DriveHub) -> list[tuple[float, float]]:
        """The motor's four mounting screws (its square turned square to the drive)."""
        m = np.asarray(hub.motor)
        u, v = self._uv(hub)
        h = self.motor.hole_square / 2
        return [tuple(m + sx * h * u + sy * h * v) for sx in (-1, 1) for sy in (-1, 1)]

    def window_half(self) -> float:
        """Half the window in the frame plate a slider's motor screws' heads stand in."""
        return self.motor.hole_square / 2 + screw("bhcs", "3").head_d / 2 + 1.0

    def clamp_offset(self) -> float:
        """A slider's clamp screws from the motor's axis, across the drive: their heads
        clear the motor's body, their tapped holes the window's edge."""
        head = screw("bhcs", "3").head_d / 2
        return max(self.window_half() + TAP_HOLE / 2 + PAD_EDGE,
                   self.motor.frame / 2 + head + 0.5)

    def clamp_points(self, hub: DriveHub) -> list[tuple[float, float]]:
        """A slider's two clamp screws, either side of the motor across the drive."""
        m = np.asarray(hub.motor)
        _, v = self._uv(hub)
        c = self.clamp_offset()
        return [tuple(m + s * c * v) for s in (-1, 1)]

    def footprint(self, ctx: Context) -> tuple[bool, list[np.ndarray],
                                                tuple[np.ndarray, np.ndarray, float] | None]:
        """What this drive stands outside its frame plate, in the plane: ``(on the outer
        plate, the motor's square's corners, the slider's stadium (ends and radius) or
        None)`` (a robot layout keeps its parts clear of them: :mod:`construction.rotary`)."""
        hub = self.hub(ctx)
        m = np.asarray(hub.motor, dtype=float)
        u, v = self._uv(hub)
        h = self.motor.frame / 2
        corners = [m + sx * h * u + sy * h * v for sx, sy in ((-1, -1), (1, -1), (1, 1),
                                                                (-1, 1))]
        slider = None
        if self.transmission.tensioned:
            a = self.motor.frame / 2
            slider = (m - a * v, m + a * v, self._outside_r() - TENSION)
        return hub.down, corners, slider

    def lowest(self, ctx: Context) -> tuple[float, float]:
        """The lowest y of what this drive puts inside the stack (its hub round the pivot,
        turning in place; the belt at the motor) and outside its frame plate
        (the motor, its slider): a robot layout's floor stays under both."""
        hub = self.hub(ctx)
        tr = self.transmission
        o = np.asarray(ctx.topo.geometry.points[hub.axis][0], dtype=float)
        m = np.asarray(hub.motor, dtype=float)
        hull = tr.hull(ctx, hub)
        r_o = max(tr.hub_r(ctx), hull[0] if hull else 0.0)
        r_m = max(tr.motor_side_r(ctx), hull[-1] if hull else 0.0)
        inside = min(o[1] - r_o, m[1] - r_m)
        _, corners, slider = self.footprint(ctx)
        outside = min(float(c[1]) for c in corners)
        if slider is not None:
            outside = min(outside, min(float(slider[0][1]), float(slider[1][1])) - slider[2])
        return float(inside), outside

    def _bolt_offsets(self, hub: DriveHub | None = None) -> list[float]:
        given = self.transmission.bolt_angles(hub) if hub is not None else None
        if given is not None:
            return list(given)
        n = max(self.transmission.bolts, 1)
        return [math.pi / n + 2 * math.pi * i / n for i in range(n)]

    def bolt_points(self, o: np.ndarray, other: np.ndarray,
                    hub: DriveHub | None = None) -> list[np.ndarray]:
        """The hub's bolts through the link, ``(T, 2)`` each, turning with it."""
        d = np.atleast_2d(other) - o
        u = d / np.linalg.norm(d, axis=-1, keepdims=True)
        v = np.stack([-u[:, 1], u[:, 0]], axis=-1)
        rb = self.transmission.bolt_pcd / 2
        return [o + rb * (math.cos(a) * u + math.sin(a) * v)
                for a in self._bolt_offsets(hub)]

    # -- claims --------------------------------------------------------------------

    def claims(self, ctx: Context) -> list[Claim]:
        hub = self.hub(ctx)
        geo = ctx.topo.geometry
        n = geo.samples
        o = np.asarray(geo.points[hub.axis][0], dtype=float)
        m = np.asarray(hub.motor)
        g = self.name
        mname = f"{g}.motor"
        geo.points[mname] = np.broadcast_to(m, (n, 2))
        for tag, e in zip("ab", motor_ends(m, hub.angle, self.motor), strict=True):
            geo.points[f"{g}.body_{tag}"] = np.broadcast_to(e, (n, 2))
        tensioned = self.transmission.tensioned
        inside_pts = self.clamp_points(hub) if tensioned else self.screw_points(hub)
        inside = []
        for i, xy in enumerate(inside_pts):
            geo.points[f"{g}.screw{i}"] = np.broadcast_to(np.asarray(xy), (n, 2))
            inside.append(f"{g}.screw{i}")
        heads = []                        # a slider's motor screws' heads, in the window
        if tensioned:
            for i, xy in enumerate(self.screw_points(hub)):
                geo.points[f"{g}.head{i}"] = np.broadcast_to(np.asarray(xy), (n, 2))
                heads.append(f"{g}.head{i}")
        hull = self.transmission.hull(ctx, hub)
        hull_pts: list[tuple[str, float]] = []
        if hull:
            for i, r in enumerate(hull):
                f = i / (len(hull) - 1)
                name = f"{g}.span{i}"
                geo.points[name] = np.broadcast_to(o + f * (m - o), (n, 2))
                hull_pts.append((name, r))
        # the hub's bolts turn with the link: moving points, with the link's lateral bound
        other = self._other(ctx)
        rb = self.transmission.bolt_pcd / 2
        reach = float(np.linalg.norm(geo.points[other][0] - o))
        lat = geo.lateral.get(other, 0.0) * rb / reach if reach > 0 else 0.0
        bolts = [ctx.topo.add_link_point(f"{g}.bolt{i}", hub.axis, other,
                                         (rb * math.cos(a), rb * math.sin(a)), lat)
                 for i, a in enumerate(self._bolt_offsets(hub))]
        geo = ctx.topo.geometry
        geo.__dict__.pop("step", None)
        sk = screw("bhcs", "3")
        hub_r, side_r = self.transmission.hub_r(ctx), self.transmission.motor_side_r(ctx)
        shaft_r = self.motor.shaft_d / 2 + 0.5
        body_layers = math.ceil((self.motor.length + self.slider_t(ctx)) / ctx.pitch)
        nuts = self.transmission.nuts
        plate_t = ctx.sheet_t("frame")
        # the shaft from the motor's face: through the slider and the plate into the stack
        reach_z = self.motor.shaft_len - self.slider_t(ctx) - plate_t
        n_hub, hub_r_m = self.transmission.motor_hub(ctx)
        s = hub.side

        link_t = {m: ctx.sheet_t("link", m) for m in ctx.topo.links}

        def nominal(L: Layout) -> list[float]:
            """Each layer's thickness while searching: the pitch, or a thicker link placed
            in it so far (a lower bound of the plan's: a thick link's layer is as thick as
            it is, so a shaft that can't reach past it is ruled out here, not at the
            plan's z, where the search would learn nothing from it)."""
            out = [L.pitch] * (L.top + 1)
            for m, k in L.layers.items():
                if 0 < k < L.top and link_t.get(m, L.pitch) > out[k]:
                    out[k] = link_t[m]
            return out

        def lay_z(L: Layout, j: int) -> tuple[float, float]:
            """Layer ``j``'s z (at the plan's z when final, else :func:`nominal`'s)."""
            if L.final:
                return L.z(j)
            t = nominal(L)
            z0 = plate_t + sum(t[1:j])
            return (z0, z0 + t[j])

        def plate_face(L: Layout) -> float:
            """The plate's face toward the stack (the shaft's start inside)."""
            if hub.down:
                return L.z(0)[1] if L.final else plate_t
            return L.z(L.top)[0] if L.final else plate_t + sum(nominal(L)[1:L.top])

        def make(L: Layout) -> list[Placed]:
            k = L.layers[self.link]
            hub_layers = hub.layers(k)
            far = hub_layers[-1]                         # the hub's layer nearest the plate
            plate = 0 if hub.down else L.top
            screw_layer = plate - s                      # the layer over (under) the plate
            room = (far - plate) * -s - 1        # the layers between the hub and the plate
            if room < max(1, n_hub):             # (the thrust ring and any bolt heads: one)
                raise Unbuildable(f"{g}: its hub's {hub.below} layers "
                                  f"{'under' if hub.down else 'over'} {self.link} (layer {k}) "
                                  f"leave {room} before the plate; it needs "
                                  f"{max(1, n_hub)} (the axle's thrust ring, the motor "
                                  "pulley's hub)")
            near = hub_layers[0]                         # the shaft's far end: the hub's top
            end = lay_z(L, near)[1] if hub.down else lay_z(L, near)[0]
            if abs(end - plate_face(L)) > reach_z + 1e-6:
                raise Unbuildable(f"{g}: the motor's {self.motor.shaft_len:g} mm shaft doesn't "
                                  f"reach the hub beside {self.link} in layer {k}")
            out: list[Placed] = []
            for j in hub_layers:
                out.append(Placed(j, Disc(hub.axis, hub_r), g, f"{g} hub"))
                out.append(Placed(j, Disc(mname, side_r), g, f"{g} motor side"))
                out += [Placed(j, Pill(a, b, max(ra, rb_)), g, f"{g} span")
                        for (a, ra), (b, rb_) in itertools.pairwise(hull_pts)]
            for j in range(far + s, plate, s):           # the shaft, in its pulley's hub first
                r = hub_r_m if (j - far) * s <= n_hub else shaft_r
                out.append(Placed(j, Disc(mname, max(r, shaft_r)), g, f"{g} shaft"))
            tip = plate_face(L) + (-s) * reach_z         # an uncut shaft's tip, its layers
            j = hub_layers[0] - s
            while not self.cut_shaft and 0 < j < L.top and (
                    lay_z(L, j)[0] < tip - 1e-6 if hub.down else lay_z(L, j)[1] > tip + 1e-6):
                out.append(Placed(j, Disc(mname, shaft_r), g, f"{g} shaft tip"))
                j -= s
            if tensioned:   # the clamp screws' tips through the plate; the window's heads
                out += [Placed(screw_layer, Disc(c, 1.6), g, f"{g} clamp tip")
                        for c in inside]
                out += [Placed(plate, Disc(c, SCREW_HOLE / 2), g, f"{g} clamp", seat=True)
                        for c in inside]
                out += [Placed(plate, Disc(h, sk.head_d / 2), g, f"{g} screw head", seat=True)
                        for h in heads]
            else:
                out += [Placed(screw_layer, Disc(sc, sk.head_d / 2), g, f"{g} motor screw")
                        for sc in inside]
                out += [Placed(plate, Disc(sc, SCREW_HOLE / 2), g, f"{g} screw shank",
                               seat=True) for sc in inside]
            out.append(Placed(plate, Disc(mname, self.motor.boss_d / 2), g, f"{g} boss",
                              seat=True))
            out += [Placed(plate + s * (1 + i), Pill(f"{g}.body_a", f"{g}.body_b", hub.motor_r),
                           g, f"{g} motor") for i in range(body_layers)]
            out += [Placed(k, Disc(b, SCREW_HOLE / 2), g, f"{g} bolt", seat=True)
                    for b in bolts]
            # ... and through the clearance gap between the link and its hub where the plan
            # puts one (2026-10-21: the lowest-stack planner's palletizer has gaps there)
            out += [Placed(min(k, near), Disc(b, SCREW_HOLE / 2), g, f"{g} bolt", gap=True)
                    for b in bolts]
            if nuts:            # heads on the hub's far face; nuts and tips past the link
                out += [Placed(far + s, Disc(b, sk.head_d / 2), g, f"{g} bolt head")
                        for b in bolts]
                past = self._bolt_past(ctx, L.pitch)
                beyond = [k - s * i for i in range(1, past + 1)]
                if not all(0 < j < L.top for j in beyond):
                    raise Unbuildable(f"{g}: the nuts on its hub's bolts and their tips need "
                                      f"{past} layers past {self.link}")
                out += [Placed(j, Disc(b, 3.2), g, f"{g} nut") for b in bolts for j in beyond]
            else:               # heads on the link's far face, threaded into the hub: the
                # layer beyond it, and the clearance gap between them where the plan puts
                # one (the head bears on the link: it stands in that gap first; before
                # 2026-10-14 it was outside its claims there)
                out += [Placed(k - s, Disc(b, sk.head_d / 2), g, f"{g} bolt head")
                        for b in bolts]
                out += [Placed(min(k, k - s), Disc(b, sk.head_d / 2), g, f"{g} bolt head",
                               gap=True) for b in bolts]
            return out

        return [Claim(g, frozenset({self.link}), make)]

    def _bolt_length(self, ctx: Context, link_t: float, hub_t: float) -> float:
        """The hub's bolts' stock length: through the link and the hub, and a nut beyond
        it; or 6 mm into the hub's tapped holes."""
        sk = screw("bhcs", "3")
        need = link_t + (hub_t + 2.4 if self.transmission.nuts else 6.0)
        return sk.length_for(need, longest=True)

    def _bolt_past(self, ctx: Context, pitch: float) -> int:
        """The layers a nutted bolt (its nut, its tip) takes past the hub."""
        hub_t = self.transmission.layers(ctx, self.hub(ctx).travel_deg) * pitch
        blen = self._bolt_length(ctx, pitch, hub_t)
        return max(1, math.ceil((blen - pitch - hub_t) / pitch - 1e-9))

    # -- parts ---------------------------------------------------------------------

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        ctx = build.ctx
        hub = self.hub(ctx)
        k = build.layers[self.link]
        layers = sorted(hub.layers(k))
        frame = build.plan.topo.frame_bodies[0]
        m = np.asarray(hub.motor)
        mo = self.motor
        plate = 0 if hub.down else build.top
        plate_key = FRAME_OUTER if hub.down else FRAME_INNER
        p0, p1 = build.z(plate)
        st = self.slider_t(ctx)
        outer_face = p0 if hub.down else p1           # the plate's face away from the stack
        inner_face = p1 if hub.down else p0
        sgn = -1.0 if hub.down else 1.0               # outward, away from the stack
        face = outer_face + sgn * st                  # the motor's face
        shaft = self.shaft_length(build, hub, layers)
        body = union([
            box(tuple(m), (mo.frame, mo.frame, mo.length),
                face - mo.length if hub.down else face, hub.angle),
            disc(tuple(m), mo.boss_d / 2 - 0.2, *sorted((face, face - sgn * mo.boss_h))),
            disc(tuple(m), mo.shaft_d / 2 - 0.05, *sorted((face, face - sgn * shaft)))])
        stem = self.name.replace(":", "_")
        out.bodies.append(hardware(f"{stem}_motor", body, frame, fab="purchased",
                                   bom_key=mo.bom_key, color=MOTOR_COLOR))
        sk = screw("bhcs", "3")
        o = build.xy(hub.axis)
        if self.transmission.tensioned:
            self._slider(build, hub, out, frame, plate_key, outer_face, sgn, st, stem)
        else:
            length = screw_length(sk, ctx.sheet_t("frame"), mo.hole_depth)
            for i, xy in enumerate(self.screw_points(hub)):
                out.bodies.append(hardware(f"{stem}_motor_screw{i}", screw_solid(
                    xy, sk, inner_face, length, up=not hub.down), frame, fab="purchased",
                    bom_key=sk.key(length), color=STEEL))
                out.cut(plate_key, Cut(xy, SCREW_HOLE))
            out.cut(plate_key, Cut(tuple(m), mo.boss_d + 0.4))
            out.pad(plate_key, tuple(o), tuple(m), self.motor.hole_square / math.sqrt(2)
                    + SCREW_HOLE / 2 + PAD_EDGE + 1.0)
        # the hub's bolts, at this pose, through the link's pad (heads on its face away from
        # the hub), into the hub's tapped holes or through it to nuts
        other = build.xy(self._other(ctx))
        zl0, zl1 = build.z(k)
        hub_t = build.z(layers[-1])[1] - build.z(layers[0])[0]
        blen = self._bolt_length(ctx, zl1 - zl0, hub_t)
        if self.transmission.nuts:     # from the hub's far face, through it and the link
            head_z = build.z(min(layers))[0] if hub.down else build.z(max(layers))[1]
            up = hub.down
        else:                          # from the link's far face into the hub
            head_z, up = (zl1 if hub.down else zl0), not hub.down
        for i, p in enumerate(self.bolt_points(np.asarray(o), np.asarray(other), hub)):
            xy = tuple(p[0])
            out.bodies.append(hardware(f"{stem}_bolt{i}", screw_solid(
                xy, sk, head_z, blen, up=up), self.link, fab="purchased",
                bom_key=sk.key(blen), color=STEEL))
            out.cut(self.link, Cut(xy, SCREW_HOLE))
        if self.transmission.nuts:
            out.extras.append(BomLine("m3_nut", self.transmission.bolts,
                                      f"{self.name}: the hub's bolts"))
        self.transmission.realize(build, self, hub, out, layers)
        # the screws threaded into what they hold (not clashes: contract.clashes)
        plate_body = frame if plate_key == FRAME_INNER else "frame_outer"
        pairs = []
        for b in out.bodies:
            if "_motor_screw" in b.name:
                pairs.append((b.name, f"{stem}_motor"))
            elif "_clamp" in b.name:
                pairs.append((b.name, plate_body))
            elif "_bolt" in b.name and not self.transmission.nuts:
                pairs += [(b.name, n.name) for n in out.bodies if n.name.endswith("_pulley_large")]
        fastened = out.notes.setdefault("fastened", {})
        fastened.update({f"{a}|{b}": [a, b] for a, b in pairs})
        cfg = ctx.config
        sensor = (sensors.get(cfg.role("angle_sensor", self.input)) if cfg is not None
                  else None)
        if sensor is not None and sensor.bom_key:
            out.extras.append(BomLine(sensor.bom_key, 1, f"{self.name}: {self.link}'s angle "
                                      "(on the link, its x along it)"))
        for key, qty in (HARD_STOP if sensor is not None and sensor.needs_stop else ()):
            out.extras.append(BomLine(key, qty, f"{self.name}: {self.link}'s hard stop at the "
                                      "end of its travel (sensorless homing)"))
        out.notes.setdefault("drive", {})[self.name] = {
            "input": self.input, "link": self.link, "transmission": self.transmission.key,
            "motor": mo.key, "ratio": round(hub.ratio, 4), "layers": layers,
            "side": "outer" if hub.down else "inner",
            "motor_xy": [round(v, 3) for v in hub.motor],
            "angle_deg": round(math.degrees(hub.angle), 2), "shaft_mm": shaft,
            "tensioned": self.transmission.tensioned}
        return out

    def _slider(self, build: Build, hub: DriveHub, out: Realized, frame: str, plate_key: str,
                outer_face: float, sgn: float, st: float, stem: str) -> None:
        """The motor on a slider plate outside its frame plate: the motor's screws through
        the slider (their heads in a window cut in the frame plate), the slider clamped to
        the plate by two screws from outside into tapped holes, slotted along the drive
        (the belt's tension, set any time from outside)."""
        ctx = build.ctx
        mo = self.motor
        sk = screw("bhcs", "3")
        m = np.asarray(hub.motor)
        _, v = self._uv(hub)
        a = mo.frame / 2                                # the stadium's half length across
        r = self._outside_r() - TENSION
        z0, z1 = sorted((outer_face, outer_face + sgn * st))
        holes: list[Cut | Rect] = [Cut(tuple(m), mo.boss_d + 0.4)]
        holes += [Cut(xy, SCREW_HOLE) for xy in self.screw_points(hub)]
        holes += [Rect(c, (SCREW_HOLE + 2 * TENSION, SCREW_HOLE), hub.angle)
                  for c in self.clamp_points(hub)]
        slider = cut_holes(pill(tuple(m - a * v), tuple(m + a * v), r, z0, z1), holes, z0, z1)
        out.bodies.append(hardware(f"{stem}_slider", slider, frame, fab="laser",
                                   color=SLIDER_COLOR, sheet=ctx.sheet("frame")))
        length = screw_length(sk, st, mo.hole_depth)
        for i, xy in enumerate(self.screw_points(hub)):  # heads in the plate's window
            out.bodies.append(hardware(f"{stem}_motor_screw{i}", screw_solid(
                xy, sk, outer_face, length, up=not hub.down), frame, fab="purchased",
                bom_key=sk.key(length), color=STEEL))
        clamp = stock_length(sk, st + ctx.sheet_t("frame"), "drive: the slider's clamp")
        for i, xy in enumerate(self.clamp_points(hub)):
            out.bodies.append(hardware(f"{stem}_clamp{i}", screw_solid(
                xy, sk, outer_face + sgn * st, clamp, up=hub.down), frame, fab="purchased",
                bom_key=sk.key(clamp), color=STEEL))
            out.cut(plate_key, Cut(xy, TAP_HOLE))
        w = 2 * self.window_half()
        out.cut(plate_key, Rect(tuple(m), (w + 2 * TENSION, w), hub.angle))
        # the pad, run past the motor so its round end clears the window's far corners by
        # PAD_EDGE and holds the clamp screws' tapped holes
        r = self.clamp_offset() + TAP_HOLE / 2 + PAD_EDGE + 1.0
        half = w / 2 + TENSION                       # the window's half length along u
        e = max(0.0, half - math.sqrt(max((r - PAD_EDGE) ** 2 - (w / 2) ** 2, 0.0)))
        u, _ = self._uv(hub)
        out.pad(plate_key, tuple(build.xy(hub.axis)), tuple(m + (e + 1.0) * u), r)

    def shaft_length(self, build: Build, hub: DriveHub, layers: list[int]) -> float:
        """The motor's shaft from its face: cut to the motor pulley's far end less 0.5 mm
        (``cut_shaft``), else the stock length."""
        if not self.cut_shaft:
            return self.motor.shaft_len
        st = self.slider_t(build.ctx)
        if hub.down:
            need = build.z(max(layers))[1] - (build.z(0)[0] - st)
        else:
            need = (build.z(build.top)[1] + st) - build.z(min(layers))[0]
        return min(self.motor.shaft_len, round(need - 0.5, 1))

    def error_terms(self, ctx: Context, got: Realized | None = None) -> ErrorTerms:
        """The transmission's terms in series with the motor's through the reduction."""
        hub = self.hub(ctx)
        return self.transmission.error_terms(ctx, hub) + self.motor.error_terms(hub.ratio)

    def assembly(self, view) -> list:
        """The motor on its plate (on a slider outside it, a tensioned drive's), the hub's
        parts with the driven link, the transmission's own step."""
        from spiderpig.construction.assembly import STACK, UNIT, Op, whole

        stem = self.name.replace(":", "_")
        motor = [n for n in view.z if n.startswith((f"{stem}_motor", f"{stem}_slider",
                                                    f"{stem}_clamp"))]
        hub_parts = [n for n in view.z if n.startswith(f"{stem}_") and n not in motor]
        if not motor:
            return []
        hub = self.hub(view.ctx)
        where = "under the outer plate" if hub.down else "on the inner plate"
        cut = ""
        if self.cut_shaft and view.z.get(f"{stem}_motor"):
            z0, z1 = view.z[f"{stem}_motor"]
            shaft = round((z1 - z0) - self.motor.length, 1)
            if shaft < self.motor.shaft_len - 0.05:
                cut = (f" Cut its shaft to {shaft:g} mm from its face (from "
                       f"{self.motor.shaft_len:g}): it must not pass its pulley's hub; file "
                       "the flat back to the cut.")
        if self.transmission.tensioned:
            text = (f"Screw the {self.motor.name} to its slider plate (four screws from the "
                    f"slider's plain face), then clamp the slider {where} with its two "
                    "screws from outside into the plate's tapped holes (tap them M3 first), "
                    "loose: the belt is tensioned from outside once it is on." + cut)
        else:
            text = (f"Bolt the {self.motor.name} {where} (its screws from the leg side)."
                    + cut)
        ops = [Op(STACK, (-1, 1), whole(*sorted(motor)), f"Motor for {self.input}", text,
                  "motor") if hub.down else
               Op(UNIT, (1,), whole(*sorted(motor)), f"Motor for {self.input}", text, "motor")]
        if hub_parts:
            k = min(view.slot(view.z[n][0]) for n in hub_parts)
            ops.append(Op(STACK, (k, 0, 1), whole(*sorted(hub_parts)),
                          f"Drive for {self.input}",
                          self.transmission.assembly_text(hub), "drive"))
        return ops
