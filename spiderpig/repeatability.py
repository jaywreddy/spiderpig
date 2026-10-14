"""Repeatability as an arm maker quotes it: ISO 9283's pose repeatability by Monte Carlo
(2026-10-12, round 5 of the arm's precision review).

:mod:`spiderpig.precision`'s total is a worst-case bound: every clearance at its extreme,
the same way, at the worst pose. A maker's 0.1 mm is ISO 9283's **RP**: the arm sent 30
times to the same pose from the same direction, the mean distance of the 30 positions from
their barycentre plus 3 standard deviations, at five poses (the centre and four near the
corners of the workspace: here the inputs' box at 10 / 90 %). What changes between those
cycles is what this counts (:func:`iso_rp`):

* **gravity seats the pins.** The arm's weight and the payload load every interface; at
  rest at a pose that load has one direction, so its clearance is taken up the same way
  every time (an offset: accuracy). An interface whose load at the pose is under
  :data:`SEAT_N` re-seats at random (uniform in its clearance's disc). A preloaded pair's
  bore gap on its shaft is held by the clamp's friction instead (``clamped``): it slips
  only where the load, with the moves' accelerations (:data:`ACCEL_FACTOR`), passes
  :data:`SLIP_N` (half of ``4 mu F_p``: mu 0.1 oiled ground steel, ~25 N of preload
  relaxed: the review's), and then it lands against the shaft the way the last load above
  that pushed it: the same every cycle of a same-direction approach (ISO's), so not in RP
  (in a multi-directional figure it would be). A power-off drop (the motors released: the
  arm sags against their detent) re-seats the gaps that never slip anywhere, uniform in
  their disc, across power-ups. The loads are a static solve of the
  linearised constraints (:func:`gravity_loads`: the links' masses from their sheet, the
  joints' hardware, the drums, the tool and the payload);
* **the drives**: backlash counts only where the drive's gravity torque at the pose is
  under :data:`SEAT_NM` (else it rests against one flank), and only for a controller
  that doesn't end its moves one-sidedly (:func:`control.host.one_sided`: the host
  does, so RP leaves backlash out; :attr:`Rp.any_direction_mm` counts it, the arm sent
  to a pose from anywhere without that final approach); a friction dead band
  (:data:`DEAD_DEG` at the joint, uniform, through its lever) only where the gravity torque
  is under the joint's friction (:data:`FRICTION_NM`), and always on the turntable (no
  gravity about a vertical axis);
* **the pairs' tilt** is elastic under a load of one direction: only its hysteresis
  (:data:`TILT_HYSTERESIS` of the out-of-plane worst case) varies, across the plane;
* **the turntable** has no gravity torque: its backlash and dead band count, uniform;
* **across power-ups** (:attr:`Rp.powerup_mm`) every cycle also re-homes: each axis's
  homing error (uniform) and, where homing can't snap to the driver's phase, half a
  microstep and the microsteps' nonlinearity (``nonlin_deg``: a microstep's place, +-5 %
  of a full step, is systematic only while the motor's phase keeps its place against the
  joint).

The links' bending under a fixed payload repeats at a pose, and isn't counted; a payload
that changes (a slide picked up) is the precision report's own row.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig import linkage
from spiderpig.linkage.constraints import _matrix, gains, layout, scale_of

SEAT_N = 0.2            # a ball's clearance under this load at rest may re-seat (N)
SLIP_N = 5.0            # a preloaded pair's inner rings slip on the shaft past this (N)
ACCEL_FACTOR = 1.5      # the moves' accelerations on the static load (about 0.5 g)
SEAT_NM = 0.01          # a drive under this gravity torque may rest anywhere in its backlash
FRICTION_NM = 0.01      # a joint's friction torque (bearings, cord): dead band below it
DEAD_DEG = 0.003        # a joint's friction dead band: where it stops, either side (deg)
TILT_HYSTERESIS = 0.1   # the share of the pairs' elastic tilt that varies between cycles
#                         (metal seats; the review's, 0.2 on acrylic)
HARDWARE_G = 3.0        # each joint's hardware on a link (a bearing pair, hub, shaft share)
DRUM_G = 25.0           # a drive's driven drum or pulley on its link (g)
TOOL_G = 15.0           # the tool (a cup and its holder) at the output (g)
PAYLOAD_G = 50.0        # the load at the output a design assumes unless its spec says
#                         (``precision.payload_g``): a few glass slides
CYCLES = 30             # ISO 9283's
TRIALS = 400
POSES = ((0.5, 0.5), (0.1, 0.1), (0.9, 0.1), (0.9, 0.9), (0.1, 0.9))   # the box's share
G = 9.81


@dataclass(frozen=True)
class GravityLoads:
    """At each ISO pose: each interface's gravity load (N) and each drive's torque (N·m)."""

    force_n: dict[tuple[str, str], np.ndarray]     # (poses,)
    torque_nm: dict[str, np.ndarray]                # driven body -> (poses,)


def iso_inputs(lk: linkage.Linkage) -> tuple[np.ndarray, ...]:
    """The five ISO poses' inputs (radians): the box's centre and its 10 / 90 % corners
    (within its band: :meth:`Linkage.inputs_at`)."""
    return lk.inputs_at(*(np.array([f[i % 2] for f in POSES]) for i in range(len(lk.inputs))))


def gravity_loads(lk: linkage.Linkage, values, *, link_g_per_mm: float | dict[str, float],
                  payload_g: float, ins=None) -> GravityLoads:
    """The static gravity solve (module docstring): ``link_g_per_mm`` a link's mass per mm
    of its length (its sheet's density x thickness x width; per link, a mapping)."""
    per = link_g_per_mm
    lay = layout(lk)
    ins = iso_inputs(lk) if ins is None else ins
    with np.errstate(all="ignore"):
        pts = linkage.LegSolution(1, 0.0, values, lk.key).at(*ins)
    scale = scale_of(pts)
    names = (*lay.bodies, *(f"axle:{j}" for j in lay.axles))
    col = {n: 3 * i for i, n in enumerate(names)}
    n = next(iter(pts.values())).reshape(-1, 2).shape[0]
    force = {i: np.zeros(n) for i in lay.interfaces}
    torque = {b: np.zeros(n) for b in lay.driven}
    drums = {lk.drives[k][0] for k in lk.inputs} if lk.ranged else set()
    for s in range(n):
        a, row_of = _matrix(lay, pts, s, scale)
        f = np.zeros(a.shape[1])

        def weight(body: str, grams: float, p: np.ndarray, f=f) -> None:
            k = col[body]
            fy = -grams * G / 1000
            f[k + 1] += fy
            f[k + 2] += fy * p[0] / scale

        for b in lay.bodies:
            if b == "torso":
                continue
            ps = [np.asarray(pts[j]).reshape(-1, 2)[s] for j in lay.joints[b]]
            for p in ps[1:]:
                length = float(np.linalg.norm(p - ps[0]))
                g_mm = per.get(b, 0.0) if isinstance(per, dict) else per
                weight(b, g_mm * length, (p + ps[0]) / 2)
            for p in ps:
                weight(b, HARDWARE_G, p)
            if b in drums:
                weight(b, DRUM_G, ps[0])
        f_pt = np.asarray(pts[lay.output_point]).reshape(-1, 2)[s]
        weight(lay.output_body, payload_g + TOOL_G, f_pt)
        lam = -np.linalg.solve(a.T, f)
        for i, r in row_of.items():
            force[i][s] = float(np.hypot(lam[r], lam[r + 1]))
        base = 2 * len(row_of) + len(lay.axles) + 3
        for k, b in enumerate(lay.driven):
            torque[b][s] = float(lam[base + k] * scale / 1000)
    return GravityLoads(force, torque)


BOX_FRACTIONS = (0.1, 0.3, 0.5, 0.7, 0.9)   # the strength check's poses: a grid over the box
SINGULAR_COND = 1e6     # a pose whose drives can't place the output's force (near a lock)


def box_inputs(lk: linkage.Linkage) -> tuple[np.ndarray, ...]:
    """A grid of poses over the inputs' box at :data:`BOX_FRACTIONS` (the ISO poses' 10 / 90 %
    corners and between: away from the box's edges, where a toggle has no limit)."""
    f = np.array(BOX_FRACTIONS)
    if len(lk.inputs) == 1:
        return lk.inputs_at(f)
    a, b = np.meshgrid(f, f, indexing="ij")
    return lk.inputs_at(a.ravel(), b.ravel())


@dataclass(frozen=True)
class StaticLoads:
    """A ranged linkage's pin loads (N) per interface, worst over the box (:func:`static_loads`):
    ``operating`` its weight and the payload's at rest x :data:`ACCEL_FACTOR` (the moves'
    accelerations); ``gravity`` and ``crash`` the two terms of a crash (the output blocked,
    every drive at its capacity, each sign: the joint's ``jam`` load is their sum, worst
    pose); ``drive_nm`` each input's largest gravity torque; ``crash_force_n`` the largest
    force the drives push the output with; ``singular`` the poses left out (the drives
    can't set the output's force there)."""

    operating: dict[tuple[str, str], float]
    gravity: dict[tuple[str, str], float]
    crash: dict[tuple[str, str], float]
    drive_nm: dict[str, float]
    crash_force_n: float
    singular: int = 0


def static_loads(lk: linkage.Linkage, values, *, link_g_per_mm: float | dict[str, float],
                 payload_g: float, capacity_nm: dict[str, float]) -> StaticLoads:
    """The strength check's loads on a ranged linkage (no floor, no sim): over
    :func:`box_inputs`, the static gravity solve (:func:`gravity_loads`) and the crash, the
    output held while each drive turns at its ``capacity_nm`` (its motor's holding torque x
    its ratio, N·m at the joint), every combination of their senses. The crash is linear in
    the output's reaction: a unit force along x and along y at the output point gives each
    interface's force and each drive's torque per newton; the drives' torques fix the
    reaction (least squares for one input: the force along its lever)."""
    ins = box_inputs(lk)
    grav = gravity_loads(lk, values, link_g_per_mm=link_g_per_mm, payload_g=payload_g, ins=ins)
    lay = layout(lk)
    with np.errstate(all="ignore"):
        pts = linkage.LegSolution(1, 0.0, values, lk.key).at(*ins)
    scale = scale_of(pts)
    names = (*lay.bodies, *(f"axle:{j}" for j in lay.axles))
    col = {n: 3 * i for i, n in enumerate(names)}
    n = next(iter(pts.values())).reshape(-1, 2).shape[0]
    caps = np.array([capacity_nm.get(k, 0.0) for k in lk.inputs])
    crash = {i: np.zeros(n) for i in lay.interfaces}
    force, singular = 0.0, 0
    signs = np.array(np.meshgrid(*[[-1.0, 1.0]] * len(caps))).reshape(len(caps), -1).T
    for s in range(n):
        a, row_of = _matrix(lay, pts, s, scale)
        p = np.asarray(pts[lay.output_point]).reshape(-1, 2)[s]
        k = col[lay.output_body]
        unit = np.zeros((a.shape[1], 2))
        unit[k, 0], unit[k + 2, 0] = 1.0, -p[1] / scale          # a newton along x at p
        unit[k + 1, 1], unit[k + 2, 1] = 1.0, p[0] / scale        # and along y
        try:
            lam = -np.linalg.solve(a.T, unit)                    # (rows, 2)
        except np.linalg.LinAlgError:
            singular += 1
            continue
        base = 2 * len(row_of) + len(lay.axles) + 3
        per_n = lam[base:base + len(lay.driven)] * scale / 1000  # N·m per N, each drive
        if not np.all(np.isfinite(per_n)) or np.linalg.cond(per_n) > SINGULAR_COND:
            singular += 1
            continue
        for sg in signs:
            f = np.linalg.lstsq(per_n, sg * caps, rcond=None)[0]   # the output's reaction
            force = max(force, float(np.hypot(*f)))
            for i, r in row_of.items():
                v = lam[r:r + 2] @ f
                crash[i][s] = max(crash[i][s], float(np.hypot(*v)))
    worst = {i: float(np.max(grav.force_n[i] + crash[i])) for i in lay.interfaces}
    return StaticLoads(
        {i: float(np.max(v)) * ACCEL_FACTOR for i, v in grav.force_n.items()},
        {i: float(np.max(grav.force_n[i])) for i in lay.interfaces},
        {i: worst[i] - float(np.max(grav.force_n[i])) for i in lay.interfaces},
        {k: float(np.max(np.abs(grav.torque_nm[b]))) for k, b in zip(lk.inputs, lay.driven,
                                                                    strict=True)},
        force, singular)


@dataclass(frozen=True)
class Rp:
    """ISO 9283 RP (mm), worst of the five poses: within a session and across power-ups;
    ``unseated`` the interfaces and drives counted at random somewhere."""

    session_mm: float
    powerup_mm: float
    unseated: tuple[str, ...]
    any_direction_mm: float = 0.0    # within a session, without a one-sided approach


def _rp(points: np.ndarray) -> float:
    """``points`` (trials, cycles, 3) -> the mean RP over the trials."""
    bary = points.mean(axis=1, keepdims=True)
    d = np.linalg.norm(points - bary, axis=2)
    return float((d.mean(axis=1) + 3 * d.std(axis=1, ddof=1)).mean())


def iso_rp(lk: linkage.Linkage, values, clearances: dict, loads: GravityLoads,
           backlash_deg: dict[str, float], homing_deg: dict[str, float],
           step_deg: dict[str, float], snaps: dict[str, bool], oop_mm: float,
           yaw_lever_mm: float = 0.0, seed: int = 0,
           clamped: frozenset = frozenset(), slips: frozenset = frozenset(),
           nonlin_deg: dict[str, float] | None = None, one_sided: bool = True) -> Rp:
    """:class:`Rp` (module docstring): ``clearances`` interface -> diametral play (mm);
    ``backlash_deg`` / ``homing_deg`` / ``step_deg`` (a microstep) per input at its joint
    (``q1`` the turntable's: its error across the plane through ``yaw_lever_mm``); ``snaps``
    whether an input's homing snaps to its driver's phase; ``clamped`` the interfaces whose
    play is a preloaded pair's bore gap (held by friction), ``slips`` those of them whose
    load passes the slip somewhere over the box (the rest re-seat at a power-off drop);
    ``nonlin_deg`` each input's microstep nonlinearity at its joint; ``one_sided``: the
    controller ends its moves one-sidedly (backlash out of RP, in ``any_direction_mm``)."""
    nonlin_deg = nonlin_deg or {}
    rng = np.random.default_rng(seed)
    lay = layout(lk)
    ins = iso_inputs(lk)
    g = gains(lk, values, ins)
    out_pt = lay.output_point
    h = 1e-6
    jac = {}
    for i, name in enumerate(lk.inputs):        # the output per radian of each input
        up = [a + (h if k == i else 0.0) for k, a in enumerate(ins)]
        dn = [a - (h if k == i else 0.0) for k, a in enumerate(ins)]
        with np.errstate(all="ignore"):
            jac[name] = ((linkage.LegSolution(1, 0.0, values, lk.key).at(*up)[out_pt]
                          - linkage.LegSolution(1, 0.0, values, lk.key).at(*dn)[out_pt])
                         / (2 * h)).reshape(-1, 2)
    driven = {lk.drives[k][0]: k for k in lk.inputs} if lk.ranged else {}
    unseated: set[str] = set()
    session, powerup, anywhere = 0.0, 0.0, 0.0
    shape = (TRIALS, CYCLES)

    def disc(radius: float) -> np.ndarray:
        r = radius * np.sqrt(rng.random(shape))
        a = rng.random(shape) * 2 * math.pi
        return np.stack([r * np.cos(a), r * np.sin(a)], axis=-1)

    def uni(half: float) -> np.ndarray:
        return (rng.random(shape) * 2 - 1) * half

    for s in range(len(ins[0])):
        xy = np.zeros((*shape, 2))
        dropped = np.zeros((*shape, 2))         # what a power-off drop re-seats
        for iface, c in clearances.items():
            if c <= 0 or iface not in g.per:
                continue
            load = loads.force_n.get(iface, np.full(len(ins[0]), math.inf))[s]
            if iface in clamped:                # frozen, or slipped the same way each time
                if iface not in slips:
                    dropped += np.einsum("ij,tcj->tci", g.per[iface][s], disc(c / 2))
                continue
            if load >= SEAT_N:
                continue                        # seated by gravity
            unseated.add(f"{iface[0]}/{iface[1]}")
            xy += np.einsum("ij,tcj->tci", g.per[iface][s], disc(c / 2))
        lash = np.zeros((*shape, 3))            # backlash: only without a one-sided approach
        for name in lk.inputs:
            body = next((b for b, k in driven.items() if k == name), None)
            tau = abs(loads.torque_nm[body][s]) if body is not None else 0.0
            dq = uni(math.radians(DEAD_DEG)) if tau < FRICTION_NM else np.zeros(shape)
            if body is not None and tau < SEAT_NM:
                unseated.add(f"{name} backlash")
                lash[..., :2] += (uni(math.radians(backlash_deg.get(name, 0.0)) / 2)[..., None]
                                  * jac[name][s])
            xy += dq[..., None] * jac[name][s]
        z = uni(TILT_HYSTERESIS * oop_mm)
        if "q1" in backlash_deg:                # no gravity about the vertical
            z = z + uni(math.radians(DEAD_DEG)) * yaw_lever_mm
            lash[..., 2] += uni(math.radians(backlash_deg["q1"]) / 2) * yaw_lever_mm
        pts = np.concatenate([xy, z[..., None]], axis=-1)
        anywhere = max(anywhere, _rp(pts + lash))
        if not one_sided:
            pts = pts + lash
        session = max(session, _rp(pts))
        # across power-ups: each cycle re-homed
        up = pts.copy()
        up[..., :2] += dropped
        for name, hd in homing_deg.items():
            dq = uni(math.radians(hd))
            if not snaps.get(name, False):
                dq = dq + uni(math.radians(step_deg.get(name, 0.0)) / 2) \
                    + uni(math.radians(nonlin_deg.get(name, 0.0)))
            if name == "q1":
                up[..., 2] += dq * yaw_lever_mm
            elif name in jac:
                up[..., :2] += dq[..., None] * jac[name][s]
        powerup = max(powerup, _rp(up))
    return Rp(session, powerup, tuple(sorted(unseated)), anywhere)
