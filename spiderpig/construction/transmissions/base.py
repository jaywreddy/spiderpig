"""The drive group and what its transmissions share (:mod:`construction.transmissions`)."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    Group,
    Realized,
    hardware,
)
from spiderpig.construction.joints import ErrorTerms
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import screw, screw_solid
from spiderpig.shapes import Cut, Rect, box, disc, union
from spiderpig.stack import Claim, Disc, Layout, Pill, Placed, Unbuildable

STEEL = "#4a4a4a"
MOTOR_COLOR = "#2b2b2b"
ALU = "#c9cdd2"
MOTOR_ANGLES = (-30.0, -150.0, -60.0, -120.0, -10.0, -170.0, -90.0, 15.0, 165.0)
"""Where a drive tries its motor, in degrees from +x about its pivot, in order: under the
shoulder first (an arm's base is below it), then round."""
HEAD_CLEAR = 1.0     # a motor body's clearance to the frame's pillar heads and to another motor
SCREW_HOLE = 3.4     # an M3 clearance hole (ISO 273 medium)
TENSION = 2.0        # a belt motor's slots: its travel either way along the belt (mm)


@dataclass(frozen=True)
class DriveHub:
    """What a drive publishes (``ctx.interfaces["drive:<input>"]``): the pivot's axis, the
    driven link, how many layers the hub's parts take beside it and on which side
    (``side`` -1: under it, the motor under the outer plate; +1: over it, the motor on the
    inner plate; the axle only seats inside them), the motor's place and body radius
    (outside the stack), the reduction, and the transmission's own record (``detail``)."""

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

    @property
    def down(self) -> bool:
        return self.side < 0

    def layers(self, k: int) -> list[int]:
        """The hub's layers beside the link's layer ``k``, from the link out."""
        return [k + self.side * i for i in range(1, self.below + 1)]


class Transmission:
    """The contract a transmission implements (the drive group does the rest). Its
    ``realize`` reads the hub's side (``hub.down``) for which way its motor-side parts
    face (the motor pulley's hub toward the plate).

    ``layers(ctx)``: the hub layers its parts take under the link. ``hub_r(ctx)`` /
    ``motor_side_r(ctx)``: the radii it claims in them at the pivot and at the motor.
    ``distances(ctx, motor)``: candidate centre distances (pivot to motor shaft), each with
    its own record, in order of preference. ``ratio(detail)``. ``hull(ctx, place)``: the
    radii along the line pivot-motor in the hub layers (a belt's spans), or ``None``.
    ``motor_layer_free(ctx, place)``: whether the motor's screw heads (in the layer over
    the outer plate) clear the hub's parts (else that layer is the screws' alone).
    ``realize(build, group, hub, out, layers)``; ``error_terms(ctx, hub)``; ``bolts`` and
    ``bolt_pcd``: how many screws hold the hub to the link and on what circle; ``nuts``:
    whether they take nuts under the hub (else they thread into it).
    """

    key: str = ""
    label: str = ""
    bolts: int = 4
    bolt_pcd: float = 30.0
    nuts: bool = False

    def layers(self, ctx: Context) -> int:
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

    def realize(self, build: Build, group: DriveGroup, hub: DriveHub, out: Realized,
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
                angles=MOTOR_ANGLES) -> tuple[np.ndarray, float, float, Any]:
    """The first (centre distance, angle) whose motor body (its square as a pill:
    :func:`motor_ends`) clears every disc in ``heads`` (the frame's pillar heads) and every
    pill in ``motors`` (the other drives' on that plate) by :data:`HEAD_CLEAR`:
    ``(motor xy, angle, distance, record)``."""
    r = motor.frame / 2
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


@dataclass
class DriveGroup(Group):
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

    def interface(self, ctx: Context) -> DriveHub:
        pts = ctx.topo.geometry.points
        axis = self._axis(ctx)
        o = np.asarray(pts[axis][0], dtype=float)
        before = [h for h in ctx.interfaces.values() if isinstance(h, DriveHub)]
        side = -1 if sum(h.axis == axis for h in before) % 2 == 0 else 1
        heads = [(np.asarray(pts[a.name][0], dtype=float), 5.0)
                 for a in ctx.topo.axes_of("frame")]
        motors = [(motor_ends(np.asarray(h.motor), h.angle, self.motor), h.motor_r)
                  for h in before if h.side == side]
        r = self.motor.frame / 2
        m, th, _, detail = place_motor(o, self.transmission.distances(ctx, self.motor),
                                       self.motor, heads, motors)
        return DriveHub(self.name, axis, self.link, self.transmission.layers(ctx),
                        (float(m[0]), float(m[1])), r, th, self.transmission.ratio(detail),
                        detail, side)

    def hub(self, ctx: Context) -> DriveHub:
        h = ctx.interfaces.get(self.name)
        return h if isinstance(h, DriveHub) else self.interface(ctx)

    def screw_points(self, hub: DriveHub) -> list[tuple[float, float]]:
        """The motor's four mounting screws (its square turned square to the belt)."""
        m = np.asarray(hub.motor)
        u = np.array([math.cos(hub.angle), math.sin(hub.angle)])
        v = np.array([-u[1], u[0]])
        h = self.motor.hole_square / 2
        return [tuple(m + sx * h * u + sy * h * v) for sx in (-1, 1) for sy in (-1, 1)]

    def _bolt_offsets(self) -> list[float]:
        n = max(self.transmission.bolts, 1)
        return [math.pi / n + 2 * math.pi * i / n for i in range(n)]

    def bolt_points(self, o: np.ndarray, other: np.ndarray) -> list[np.ndarray]:
        """The hub's bolts through the link, ``(T, 2)`` each, turning with it."""
        d = np.atleast_2d(other) - o
        u = d / np.linalg.norm(d, axis=-1, keepdims=True)
        v = np.stack([-u[:, 1], u[:, 0]], axis=-1)
        rb = self.transmission.bolt_pcd / 2
        return [o + rb * (math.cos(a) * u + math.sin(a) * v) for a in self._bolt_offsets()]

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
        ends = motor_ends(m, hub.angle, self.motor)
        for tag, e in zip("ab", ends, strict=True):
            geo.points[f"{g}.body_{tag}"] = np.broadcast_to(e, (n, 2))
        screws = []
        for i, xy in enumerate(self.screw_points(hub)):
            geo.points[f"{g}.screw{i}"] = np.broadcast_to(np.asarray(xy), (n, 2))
            screws.append(f"{g}.screw{i}")
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
                 for i, a in enumerate(self._bolt_offsets())]
        geo = ctx.topo.geometry
        sk = screw("bhcs", "3")
        hub_r, side_r = self.transmission.hub_r(ctx), self.transmission.motor_side_r(ctx)
        shaft_r = self.motor.shaft_d / 2 + 0.5
        body_layers = math.ceil(self.motor.length / ctx.pitch)
        min_screw = min(float(np.linalg.norm(np.asarray(p) - o)) for p in
                        self.screw_points(hub))
        screws_alone = min_screw < hub_r + sk.head_d / 2 + ctx.params.margin
        nuts = self.transmission.nuts
        reach_z = self.motor.shaft_len - self.motor.boss_h   # the shaft past the plate's face
        plate_t = ctx.sheet_t("frame")
        s = hub.side

        def lay_z(L: Layout, j: int) -> tuple[float, float]:
            """Layer ``j``'s z (at the plan's z when final, else at the pitch)."""
            if L.final:
                return L.z(j)
            z0 = plate_t + (j - 1) * L.pitch
            return (z0, z0 + L.pitch)

        def plate_face(L: Layout) -> float:
            """The motor's face: the outer plate's bottom, or the inner plate's top."""
            if hub.down:
                return L.z(0)[0] if L.final else 0.0
            return L.z(L.top)[1] if L.final else plate_t + (L.top - 1) * L.pitch + plate_t

        def make(L: Layout) -> list[Placed]:
            k = L.layers[self.link]
            hub_layers = hub.layers(k)
            far = hub_layers[-1]                         # the hub's layer nearest the plate
            plate = 0 if hub.down else L.top
            screw_layer = plate - s                      # the layer over (under) the plate
            free = (far - screw_layer) * -s              # layers between hub and screws
            if free < (1 if screws_alone else 0):
                raise Unbuildable(f"{g}: its hub's {hub.below} layers "
                                  f"{'under' if hub.down else 'over'} {self.link} (layer {k}) "
                                  "don't fit before the plate"
                                  + (" and the motor's screw layer" if screws_alone else ""))
            n_hub = self._motor_hub(ctx)[0]
            near = hub_layers[0] - s * n_hub             # the shaft's far end: the hub's top
            if not 0 < near < L.top:
                raise Unbuildable(f"{g}: its motor pulley's hub has no room past the hub")
            if L.final:
                span = (L.z(near)[1] - L.z(0)[1] if hub.down
                        else L.z(L.top)[0] - L.z(near)[0])
            else:
                span = abs(plate - near) * L.pitch
            if span > reach_z + 1e-6:
                raise Unbuildable(f"{g}: the motor's {self.motor.shaft_len:g} mm shaft doesn't "
                                  f"reach the hub beside {self.link} in layer {k}")
            out: list[Placed] = []
            for j in hub_layers:
                out.append(Placed(j, Disc(hub.axis, hub_r), g, f"{g} hub"))
                out.append(Placed(j, Disc(mname, side_r), g, f"{g} motor side"))
                out += [Placed(j, Pill(a, b, max(ra, rb_)), g, f"{g} span")
                        for (a, ra), (b, rb_) in itertools.pairwise(hull_pts)]
            out += [Placed(j, Disc(mname, shaft_r), g, f"{g} shaft")     # the motor's shaft
                    for j in range(far + s, plate, s)]
            n_hub, hub_r_m = self._motor_hub(ctx)        # its pulley's hub, toward the link
            for i in range(1, n_hub + 1):
                j = hub_layers[0] - s * i
                if not 0 < j < L.top:
                    raise Unbuildable(f"{g}: its motor pulley's hub has no room past the hub")
                out.append(Placed(j, Disc(mname, hub_r_m), g, f"{g} pulley hub"))
            # its tip past the hub (a stock shaft is longer than the stack needs): the
            # layers it pokes into, up to the shaft's end
            tip = plate_face(L) - s * self.motor.shaft_len
            j = hub_layers[0] - s
            while not self.cut_shaft and 0 < j < L.top and (
                    lay_z(L, j)[0] < tip - 1e-6 if hub.down else lay_z(L, j)[1] > tip + 1e-6):
                out.append(Placed(j, Disc(mname, shaft_r), g, f"{g} shaft tip"))
                j -= s
            out += [Placed(plate, Disc(sc, SCREW_HOLE / 2), g, f"{g} screw shank", seat=True)
                    for sc in screws]
            out += [Placed(k, Disc(b, SCREW_HOLE / 2), g, f"{g} bolt", seat=True)
                    for b in bolts]
            out += [Placed(screw_layer, Disc(sc, sk.head_d / 2), g, f"{g} motor screw")
                    for sc in screws]
            out.append(Placed(plate, Disc(mname, self.motor.boss_d / 2), g, f"{g} boss",
                              seat=True))
            out += [Placed(plate + s * (1 + i), Pill(f"{g}.body_a", f"{g}.body_b", hub.motor_r),
                           g, f"{g} motor") for i in range(body_layers)]
            out += [Placed(k - s, Disc(b, sk.head_d / 2), g, f"{g} bolt head") for b in bolts]
            if nuts:            # the nut, and the bolt's tip past it (a stock length)
                if not 0 < far + 2 * s < L.top:
                    raise Unbuildable(f"{g}: the nuts on its hub's bolts and their tips need two "
                                      f"layers beyond the hub, before the plate")
                out += [Placed(j, Disc(b, 3.2), g, f"{g} nut") for b in bolts
                        for j in (far + s, far + 2 * s)]
            return out

        return [Claim(g, frozenset({self.link}), make)]

    def _motor_hub(self, ctx: Context) -> tuple[int, float]:
        """The layers the motor's pulley's hub takes past the hub's layers, toward the link
        (away from the plate: the motor's boss is in the plate), and its radius."""
        fn = getattr(self.transmission, "motor_hub", None)
        return fn(ctx) if fn is not None else (0, 0.0)

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
        plate_key = FRAME_OUTER if hub.down else FRAME_INNER
        shaft = self.shaft_length(build, hub, layers)
        if hub.down:
            face = build.z(0)[0]                   # the motor's face against the plate
            body = union([
                box(tuple(m), (mo.frame, mo.frame, mo.length), face - mo.length, hub.angle),
                disc(tuple(m), mo.boss_d / 2 - 0.2, face, face + mo.boss_h),
                disc(tuple(m), mo.shaft_d / 2 - 0.05, face, face + shaft)])
            inside = build.z(0)[1]                 # the screws' heads on the plate's top
        else:
            face = build.z(build.top)[1]
            body = union([
                box(tuple(m), (mo.frame, mo.frame, mo.length), face, hub.angle),
                disc(tuple(m), mo.boss_d / 2 - 0.2, face - mo.boss_h, face),
                disc(tuple(m), mo.shaft_d / 2 - 0.05, face - shaft, face)])
            inside = build.z(build.top)[0]
        stem = self.name.replace(":", "_")
        out.bodies.append(hardware(f"{stem}_motor", body, frame, fab="purchased",
                                   bom_key=mo.bom_key, color=MOTOR_COLOR))
        sk = screw("bhcs", "3")
        length = _screw_length(sk, build.ctx.sheet_t("frame") + mo.hole_depth - 0.5)
        slot = self.transmission.key == "belt"
        for i, xy in enumerate(self.screw_points(hub)):
            out.bodies.append(hardware(f"{stem}_motor_screw{i}", screw_solid(
                xy, sk, inside, length, up=not hub.down), frame, fab="purchased",
                bom_key=sk.key(length), color=STEEL))
            out.cut(plate_key, Rect(xy, (SCREW_HOLE + 2 * TENSION, SCREW_HOLE), hub.angle)
                    if slot else Cut(xy, SCREW_HOLE))
        boss = mo.boss_d + 0.4
        out.cut(plate_key, Rect(tuple(m), (boss + 2 * TENSION, boss), hub.angle)
                if slot else Cut(tuple(m), boss))
        o = build.xy(hub.axis)
        out.pad(plate_key, tuple(o), tuple(m), mo.frame / 2 + 2.0)
        # the hub's bolts, at this pose, through the link (heads on its face away from the
        # hub), into the hub's tapped holes or through it to nuts
        other = build.xy(self._other(ctx))
        zl0, zl1 = build.z(k)
        head_z = zl1 if hub.down else zl0
        hub_t = build.z(layers[-1])[1] - build.z(layers[0])[0]
        blen = _screw_length(sk, (zl1 - zl0) + (hub_t + 2.0 if self.transmission.nuts
                                                else 6.0))
        for i, p in enumerate(self.bolt_points(np.asarray(o), np.asarray(other))):
            xy = tuple(p[0])
            out.bodies.append(hardware(f"{stem}_bolt{i}", screw_solid(
                xy, sk, head_z, blen, up=not hub.down), self.link, fab="purchased",
                bom_key=sk.key(blen), color=STEEL))
            out.cut(self.link, Cut(xy, SCREW_HOLE))
        if self.transmission.nuts:
            out.extras.append(BomLine("m3_nut", self.transmission.bolts,
                                      f"{self.name}: the hub's bolts"))
        self.transmission.realize(build, self, hub, out, layers)
        out.notes.setdefault("drive", {})[self.name] = {
            "input": self.input, "link": self.link, "transmission": self.transmission.key,
            "motor": mo.key, "ratio": round(hub.ratio, 4), "layers": layers,
            "side": "outer" if hub.down else "inner",
            "motor_xy": [round(v, 3) for v in hub.motor],
            "angle_deg": round(math.degrees(hub.angle), 2)}
        return out

    def shaft_length(self, build: Build, hub: DriveHub, layers: list[int]) -> float:
        """The motor's shaft from its face: cut to the hub's far face less 0.5 mm
        (``cut_shaft``), else the stock length."""
        if not self.cut_shaft:
            return self.motor.shaft_len
        n_hub, _ = self._motor_hub(build.ctx)
        if hub.down:
            need = build.z(max(layers) + n_hub)[1] - build.z(0)[0]
        else:
            need = build.z(build.top)[1] - build.z(min(layers) - n_hub)[0]
        return min(self.motor.shaft_len, round(need - 0.5, 1))

    def error_terms(self, ctx: Context, got: Realized | None = None) -> ErrorTerms:
        """The transmission's terms in series with the motor's through the reduction."""
        hub = self.hub(ctx)
        return self.transmission.error_terms(ctx, hub) + self.motor.error_terms(hub.ratio)

    def assembly(self, view) -> list:
        """The motor on its plate (its screws from the leg side, its pulley or capstan on
        its shaft), the hub's parts with the driven link, the transmission's own step."""
        from spiderpig.construction.assembly import STACK, UNIT, Op, whole

        stem = self.name.replace(":", "_")
        motor = [n for n in view.z if n.startswith(f"{stem}_motor")]
        hub_parts = [n for n in view.z if n.startswith(f"{stem}_") and n not in motor]
        if not motor:
            return []
        hub = self.hub(view.ctx)
        where = "under the outer plate" if hub.down else "on the inner plate"
        cut = ""
        if self.cut_shaft and view.z.get(f"{stem}_motor"):
            z0, z1 = view.z[f"{stem}_motor"]
            shaft = round((z1 - z0) - self.motor.length, 1)   # the body's span less the box
            if shaft < self.motor.shaft_len - 0.05:
                cut = (f" Cut its shaft to {shaft:g} mm from the face (from "
                       f"{self.motor.shaft_len:g}): it must not pass its pulley's hub; file "
                       "the flat back to the cut.")
        text = (f"Bolt the {self.motor.name} {where} (its screws from the leg side"
                + ("; slots: leave them loose to tension the belt" if
                   self.transmission.key == "belt" else "") + ")." + cut)
        ops = [Op(STACK, (-1, 1), whole(*sorted(motor)), f"Motor for {self.input}", text,
                  "motor") if hub.down else
               Op(UNIT, (1,), whole(*sorted(motor)), f"Motor for {self.input}", text, "motor")]
        if hub_parts:
            k = min(view.slot(view.z[n][0]) for n in hub_parts)
            ops.append(Op(STACK, (k, 0, 1), whole(*sorted(hub_parts)),
                          f"Drive for {self.input}",
                          self.transmission.assembly_text(hub), "drive"))
        return ops


def _screw_length(sk, need: float) -> float:
    return next((L for L in sk.lengths if need - 1e-6 <= L), sk.lengths[-1])
