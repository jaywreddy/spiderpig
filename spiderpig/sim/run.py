"""Step the MuJoCo model of the walker and measure how it walks.

:func:`simulate` runs the model (:mod:`sim.mjcf`) under crank-speed commands
and records a :class:`SimResult` time series; :func:`walk_metrics` reduces it
to numbers (speed, stride, bob, attitude, drive torque against the servo's
limits, foot slip, loop-closure error, whether it fell). :func:`kinematic_gait`
is what the kinematics alone promise (no slip, rigid ground), for comparison;
:func:`kinematic_qpos` poses the model exactly on the template at a crank
angle (kinematic playback, and a check that the model is the template).

Controls are crank speeds in rad/s per drive, ``(left, right)``, positive =
forward: a constant pair, a schedule ``[(t0, left, right), ...]`` (piecewise
constant from each ``t0``), or a callable ``f(t) -> (left, right)``.

The commands go through a :class:`PhaseLock` (the controller the real servo
bus needs, see its docstring) unless ``SimParams.phase_lock_kp`` is 0: two
servos commanded the same speed open-loop drift apart (the catalog's
``servo_mismatch``) and the Klann quad rolls over within seconds once its
sides are more than ~90° out of phase (measured, :data:`PHASE_NOTE`).
"""

from __future__ import annotations

import copy
import itertools
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

import numpy as np

from spiderpig import linkage
from spiderpig.config import BuildConfig, torque_limit_nm
from spiderpig.fabricate import template_for
from spiderpig.linkage import feet_of
from spiderpig.sim.mjcf import (
    T_REF,
    SimParams,
    crank_sign,
    load_model,
    motor_line,
)

Controls = Callable[[float], Sequence[float]] | Sequence

FELL_TILT = math.radians(45.0)      # the base tilted further than this: it fell over
G = 9.81                            # m/s², for the peak vertical acceleration in g

PHASE_NOTE = ("the sides are phase-locked: the Klann quad stands only while its two cranks "
              "keep their phase (open-loop, a 2 / 5 / 10 % servo mismatch rolls it over at "
              "17 / 8 / 3 s; a held offset of 90° or more within a second); the real "
              "STS3215 bus needs position feedback and this controller, not wheel mode")


def _control_fn(controls: Controls | None, vmax: float) -> Callable[[float], np.ndarray]:
    if controls is None:
        controls = (vmax, vmax)
    if callable(controls):
        return lambda t: np.asarray(controls(t), dtype=float)
    items = list(controls)
    if len(items) == 2 and all(np.isscalar(x) for x in items):
        const = np.asarray(items, dtype=float)
        return lambda t: const
    # sorted by time alone (a tie would compare the arrays): the later entry at a tie wins
    sched = sorted(((float(t0), np.array([a, b], dtype=float)) for t0, a, b in items),
                   key=lambda e: e[0])

    def f(t: float) -> np.ndarray:
        out = np.zeros(2)
        for t0, v in sched:
            if t + 1e-12 >= t0:
                out = v
        return out

    return f


class PhaseLock:
    """The host-side controller that keeps the two cranks in step: what the real servo bus
    needs (the STS3215 reports its position; wheel mode alone is open-loop) and what every
    straight-walk figure of the sim assumes (:data:`PHASE_NOTE`).

    ``ctrl(cmd, phi, dt)`` turns the commanded crank speeds ``cmd`` (rad/s, left, right)
    into the drives' ``ctrl``: a PI term on the *side phase error* ``(phi_L - phi_R) -
    (ref_L - ref_R)``, where ``ref`` integrates the command, is split between the drives
    (the left slowed by half of it, the right sped up by half: at a full command the right
    alone has no headroom), so the sides are held at the commanded difference (a spin or
    a turn is a growing one: the lock fights drift, not steering). ``mismatch`` models the
    plant: the right servo turns that fraction slower than told (servo-to-servo gain
    error, battery sag per side), which a lock-less pair cannot survive. Measured on the
    Klann quad with 3 % (kp 1, ki 4): the sides stay within 0.7°; the heading still drifts
    1.3°/s at a full command (the servo on its torque line has little speed authority,
    so the weaker side pushes less) and 0.2°/s at 80 %; open loop, 5 % rolls it over at
    8 s.

    ``max_offset`` (rad) bounds a steering excursion while walking: when both drives are
    commanded the same way, the reference difference is clipped to it, and when the
    commands agree again (|L - R| under ``AGREE`` of ``vmax``) it relaxes back to the
    nearest whole revolution at ``relock_rate``, so a steering key makes a bounded phase
    excursion and re-locks (:func:`steering_check` measures the safe one, ``step_deg``).
    A spin (drives opposed) is never bounded.
    """

    AGREE = 1e-3            # |L - R| under this fraction of vmax: the commands agree
    CLIP = 0.3              # the correction's bound, fraction of vmax

    def __init__(self, vmax: float, kp: float = 1.0, ki: float = 4.0, mismatch: float = 0.0,
                 max_offset: float = math.inf, relock_rate: float | None = None) -> None:
        self.vmax, self.kp, self.ki, self.mismatch = vmax, kp, ki, mismatch
        self.max_offset = max_offset
        self.relock_rate = 0.2 * vmax if relock_rate is None else relock_rate
        self.ref = np.zeros(2)          # the commanded crank travel, rad
        self.integral = 0.0
        self.error = 0.0                # the last side phase error, rad
        self.home = 0.0                 # the whole revolution the sides are locked at, rad

    @property
    def on(self) -> bool:
        return self.kp > 0.0 or self.ki > 0.0

    def reset(self) -> None:
        self.ref[:] = 0.0
        self.integral = self.error = self.home = 0.0

    def ctrl(self, cmd: np.ndarray, phi: np.ndarray, dt: float) -> np.ndarray:
        """The drives' ``ctrl`` for the commanded speeds ``cmd`` at crank travel ``phi``
        (``actuator_length`` of the two drives), the step being ``dt``."""
        u = np.asarray(cmd, dtype=float)
        if not self.on:
            return np.array([u[0], u[1] * (1.0 - self.mismatch)])
        self.ref += u * dt
        d = self.ref[0] - self.ref[1]
        if math.isfinite(self.max_offset):
            if abs(u[0] - u[1]) <= self.AGREE * self.vmax:      # agreeing: re-lock
                home = self.home = round(d / (2 * math.pi)) * 2 * math.pi
                d = home + math.copysign(min(abs(d - home), self.relock_rate * dt), d - home) \
                    if abs(d - home) > self.relock_rate * dt else home
            elif u[0] * u[1] >= 0.0:                          # a differential while walking
                # about the whole revolution the sides were locked at (after a spin, not
                # zero: clipping to zero would unwind every revolution the spin made; held,
                # not re-rounded, so an offset up to half a turn and more stays one)
                d = self.home + max(-self.max_offset, min(self.max_offset, d - self.home))
            else:                                             # a spin: free, and re-homed
                self.home = round(d / (2 * math.pi)) * 2 * math.pi
            self.ref[1] = self.ref[0] - d
        self.error = err = float((phi[0] - phi[1]) - d)
        self.integral = max(-1.0, min(1.0, self.integral + err * dt))
        corr = self.kp * err + self.ki * self.integral
        corr = max(-self.CLIP, min(self.CLIP, corr)) * self.vmax
        return np.array([u[0] - 0.5 * corr, (u[1] + 0.5 * corr) * (1.0 - self.mismatch)])


def phase_lock(params, vmax: float, max_offset: float = math.inf) -> PhaseLock:
    """The :class:`PhaseLock` ``params`` (:class:`sim.mjcf.SimParams`) asks for."""
    return PhaseLock(vmax, kp=params.phase_lock_kp, ki=params.phase_lock_ki,
                     mismatch=params.servo_mismatch, max_offset=max_offset)


@dataclass
class SimResult:
    """Time series of one simulation (world frame, SI; one row per recorded step)."""

    t: np.ndarray                   # (N,) s
    base_pos: np.ndarray            # (N, 3) m, the base frame's origin (O on the mid-plane)
    base_quat: np.ndarray           # (N, 4) wxyz
    attitude: np.ndarray            # (N, 3) yaw, pitch (nose up +), roll (left up +), rad
    tilt: np.ndarray                # (N,) angle between the base's up axis and world z, rad
    crank: np.ndarray               # (N, S) crank angle travelled, rad (positive = forward)
    crank_speed: np.ndarray         # (N, S) rad/s
    ctrl: np.ndarray                # (N, S) commanded speed, rad/s
    torque: np.ndarray              # (N, S) drive torque, N·m
    foot_contact: np.ndarray        # (N, F) bool
    foot_force: np.ndarray          # (N, F) normal force, N
    foot_slip: np.ndarray           # (N, F) tangential speed at the contact, m/s (nan: no contact)
    loop_error: np.ndarray          # (N,) worst loop-closure gap, m
    penetration: np.ndarray         # (N,) deepest floor contact, m
    body_contact: np.ndarray        # (N,) bool: a body that carries no foot touches the floor
    loop_force: np.ndarray          # (N, E) in-plane force each loop (pin) carries, N
    base_acc: np.ndarray            # (N, 3) the base's linear acceleration, world m/s²
    drives: list[str] = field(default_factory=list)
    feet: list[str] = field(default_factory=list)
    loops: list[str] = field(default_factory=list)
    speed_max: float = 0.0          # rad/s, the servo's no-load speed
    torque_max: float = 0.0         # N·m, the servo's stall torque
    torque_rated: float | None = None   # N·m, its rated (continuous) torque, if published
    mass: float = 0.0               # kg
    config: BuildConfig | None = None
    meta: dict = field(default_factory=dict)


def _attitude(xmat: np.ndarray) -> tuple[float, float, float, float]:
    """(yaw, pitch, roll, tilt) of the base from its rotation matrix (base = mech axes)."""
    r = xmat.reshape(3, 3)
    fwd, up, left = r[:, 0], r[:, 1], -r[:, 2]
    yaw = math.atan2(fwd[1], fwd[0])
    pitch = math.asin(max(-1.0, min(1.0, fwd[2])))
    roll = math.atan2(left[2], up[2])
    tilt = math.acos(max(-1.0, min(1.0, up[2])))
    return yaw, pitch, roll, tilt


def simulate(
    config: BuildConfig | None = None,
    controls: Controls | None = None,
    seconds: float = 2.0,
    *,
    params: SimParams | None = None,
    record_every: int = 1,
    model_xml: str | None = None,
    model_meta: dict | None = None,
    motor: bool = True,
    lock: PhaseLock | bool | None = None,
    observe: Callable[[object, object], None] | None = None,
) -> SimResult:
    """Run the robot for ``seconds`` under ``controls`` (default: both drives full forward).
    ``model_xml`` with ``model_meta`` runs an exported MJCF (``export(design, ["mjcf"])``,
    ``spiderpig sim --xml``) instead of building the model from ``config``. ``motor``
    holds each drive to the servo's speed-torque line every step (:func:`sim.mjcf.motor_line`;
    off: the stiff velocity loop alone, up to the stall torque at any speed). ``lock``:
    the :class:`PhaseLock` the commands go through (``None``: the one ``params`` asks for,
    with its servo mismatch; ``False``: none, two open-loop servos, no mismatch).
    ``observe(model, data)`` is called at every recorded step, after it (the constraint
    forces are the step's own: :mod:`sim.loads` reads the pin forces there)."""
    import mujoco

    config = config or BuildConfig()
    params = params or SimParams()
    if model_xml is not None:
        if model_meta is None:
            raise ValueError("model_xml needs model_meta (the .json written beside the MJCF)")
        model, meta = mujoco.MjModel.from_xml_string(model_xml), model_meta
    else:
        model, meta = load_model(config, params)
        model = copy.copy(model)            # the cached model is shared; motor_line edits it
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    drives = list(meta["actuators"])
    act = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, d) for d in drives]
    vmax = meta["actuators"][drives[0]]["ctrlrange"][1]
    tau = meta["actuators"][drives[0]]["forcerange"][1]
    control = _control_fn(controls, vmax)
    feet = list(meta["feet"])
    foot_geom = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f): k
                 for k, f in enumerate(feet)}
    # a foot-bearing link's other geoms (its capsule) touching the floor aren't a fall:
    # classify floor contacts by body (every link pinned on a foot point)
    foot_bodies = {model.geom_bodyid[g] for g in foot_geom} | {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b)
        for info in meta["feet"].values() for b in info.get("links", ())}
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    base_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base")]
    eq_ids = np.flatnonzero(model.eq_type == mujoco.mjtEq.mjEQ_CONNECT)
    loops = eq_ids.size > 0
    loop_names = [lp["name"] for lp in meta.get("loops", [])] or [f"eq{i}" for i in eq_ids]
    gaps = _LoopGaps(model)
    if lock is None:
        lock = phase_lock(params, vmax)
    elif lock is False:
        lock = None
    dt = model.opt.timestep

    n_steps = int(round(seconds / dt))
    n = (n_steps + record_every - 1) // record_every
    s, nf = len(drives), len(feet)
    out = {
        "t": np.zeros(n), "base_pos": np.zeros((n, 3)), "base_quat": np.zeros((n, 4)),
        "attitude": np.zeros((n, 3)), "tilt": np.zeros(n), "crank": np.zeros((n, s)),
        "crank_speed": np.zeros((n, s)), "ctrl": np.zeros((n, s)), "torque": np.zeros((n, s)),
        "foot_contact": np.zeros((n, nf), dtype=bool), "foot_force": np.zeros((n, nf)),
        "foot_slip": np.full((n, nf), np.nan), "loop_error": np.zeros(n),
        "penetration": np.zeros(n),
        "body_contact": np.zeros(n, dtype=bool),
        "loop_force": np.zeros((n, eq_ids.size)), "base_acc": np.zeros((n, 3)),
    }
    f6, v6 = np.zeros(6), np.zeros(6)
    row = 0
    for step in range(n_steps):
        t0 = data.time
        u = np.clip(control(t0), -vmax, vmax)
        if lock is not None:
            u = np.clip(lock.ctrl(u, data.actuator_length[act], dt), -vmax, vmax)
        data.ctrl[act] = u
        if motor:
            motor_line(model, data, act, vmax, tau)
        mujoco.mj_step(model, data)
        if step % record_every:
            continue
        # everything below is the state at t0 (mj_step's forward pass), not after it; the
        # constraint forces and the acceleration are the step's own
        out["t"][row] = t0
        out["base_pos"][row] = data.xpos[base]
        out["base_quat"][row] = data.xquat[base]
        *att, tilt = _attitude(data.xmat[base])
        out["attitude"][row], out["tilt"][row] = att, tilt
        out["crank"][row] = data.actuator_length[act]
        out["crank_speed"][row] = data.actuator_velocity[act]
        out["ctrl"][row] = u
        out["torque"][row] = data.actuator_force[act]
        out["base_acc"][row] = data.qacc[base_dof:base_dof + 3]
        if loops:
            out["loop_error"][row] = gaps(model, data).max()
            out["loop_force"][row] = loop_forces(data, eq_ids)
        con = data.contact                  # the floor's contacts, from the arrays
        g1, g2 = con.geom1[:data.ncon], con.geom2[:data.ncon]
        for i in np.flatnonzero((g1 == floor) | (g2 == floor)).tolist():
            g = int(g2[i] if g1[i] == floor else g1[i])
            out["penetration"][row] = max(out["penetration"][row], -con.dist[i])
            k = foot_geom.get(g)
            if k is None:
                if model.geom_bodyid[g] not in foot_bodies:
                    out["body_contact"][row] = True
                continue
            mujoco.mj_contactForce(model, data, i, f6)
            b = model.geom_bodyid[g]
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, b, v6, 0)
            v = v6[3:] + _cross(v6[:3], con.pos[i] - data.xpos[b])
            nrm = con.frame[i, :3]
            slip = float(np.linalg.norm(v - (v @ nrm) * nrm))
            out["foot_contact"][row, k] = True
            out["foot_force"][row, k] += f6[0]
            prev = out["foot_slip"][row, k]
            out["foot_slip"][row, k] = slip if np.isnan(prev) else max(prev, slip)
        if observe is not None:
            observe(model, data)
        row += 1

    return SimResult(**{k: v[:row] for k, v in out.items()}, drives=drives, feet=feet,
                     loops=loop_names, speed_max=vmax, torque_max=tau,
                     torque_rated=meta["actuators"][drives[0]].get("rated_torque"),
                     mass=meta["mass"]["total"], config=config, meta=meta)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------


def walk_metrics(result: SimResult, skip: float = 0.5) -> dict:
    """How the robot walked, from ``skip`` seconds on (lengths in mm, angles in degrees).

    * ``speed``: mean speed along the initial heading (mm/s); ``lateral``: drift
      across it (mm); ``yaw_rate`` (deg/s), ``heading_drift`` (deg) and
      ``turn_radius`` (mm, the path's; ``inf`` when straight);
    * ``revolutions`` of the cranks (mean of the drives, signed), ``revolutions_abs``
      (mean of their magnitudes: a spin's rate), ``drives_oppose`` (one drive turned
      forward and the other back) and ``stride``, the forward travel per revolution
      (mm); ``yaw_per_rev`` (deg);
    * ``bob``: base height range per revolution (mm); ``pitch_range``, ``roll_range``;
    * per drive (``torque``): peak and mean |torque| (N·m), the stall torque
      (``limit``) and the rated one (``rated``, ``mean_over_rated``),
      ``saturated`` (fraction of samples at the stall torque), ``at_envelope``
      (fraction of the motoring samples held on the servo's speed-torque
      line, ``stall * (1 - |speed| / no_load)``: the servo giving all it has at
      that speed), ``speed_droop`` (how far the mean speed fell short of the
      command, as a fraction), ``speed_under_load`` (rad/s), what a DC-motor
      servo turns at under the mean load at full voltage, and mean mechanical
      ``power`` (W);
    * ``slip``: mean / max tangential speed of feet in contact (mm/s);
      ``feet_down``: mean number of feet on the floor; ``feet_down_hist``: the
      fraction of samples with 0, 1, 2, ... feet down; ``airborne``: the fraction
      with none; ``side_support_low``: the fraction with fewer than two feet
      down on one side (a tripod gait's weak moments);
    * ``loop_error``: worst loop-closure gap (mm); ``penetration``: deepest
      floor contact (mm); ``fell``: tilted past 45°; ``body_contact``: fraction
      of time a body that carries no foot is down;
    * ``side_phase``: the sides' crank difference L − R at the end (deg, wrapped to
      ±180) and ``side_phase_max``, the largest |L − R| seen (deg, unwrapped: how far
      the lock let them drift or a steering excursion went);
    * the loads that break parts: ``airborne_per_rev`` (the airborne fraction of each
      revolution), ``accel_z_peak_g`` (the base's peak vertical acceleration, in g),
      ``foot_force_peak`` (N, one foot's normal force), ``loop_force`` per loop (the
      in-plane force its pin carries: ``peak`` and ``p999``, the 99.9th percentile, N),
      ``loop_force_peak`` and ``loop_force_p999`` over every loop; the crank's joints:
      ``joint_moment_peak`` (N·m, the twist a crankpin joint of the built-up crankshaft
      carries at the torque peak: ``joint_moment_factor``, chord / crank radius from
      :func:`crank_joint_factor`, times it), ``crank_capacity_nm`` (what the weakest
      element of a crankpin joint of the design's crank holds, nominal:
      :func:`spiderpig.strength.crank_capacity`; ``crank_weakest`` names it),
      ``torque_limit_recommended`` (N·m, the servo's firmware limit that keeps a jam
      under it, :func:`config.torque_limit_nm`) and
      ``joint_moment_at_limit`` (the joint's moment in a jam at that limit);
    * ``walks``: it covered ground (|stride| ≥ :data:`MIN_STRIDE_MM` per revolution)
      with its body off the floor (``body_contact`` under :data:`BODY_DOWN`).

    ``loop_error``, ``penetration`` and ``fell`` cover the whole run.
    """
    r = result
    m = r.t >= r.t[0] + skip
    if m.sum() < 2:
        raise ValueError("not enough samples after skip")
    t = r.t[m]
    dur = t[-1] - t[0]
    pos = r.base_pos[m]
    yaw = np.unwrap(r.attitude[m, 0])
    h0 = yaw[0]
    fwd = np.array([math.cos(h0), math.sin(h0)])
    left = np.array([-fwd[1], fwd[0]])
    disp = (pos[-1] - pos[0])[:2]
    forward, lateral = float(disp @ fwd), float(disp @ left)
    crank = r.crank[m]
    revs_each = (crank[-1] - crank[0]) / (2 * math.pi)
    revs = float(np.mean(revs_each))

    # bob (and the airborne fraction) per revolution, by the mean crank angle travelled
    travel = np.abs(np.mean(crank - crank[0], axis=1))
    z = pos[:, 2]
    k = np.floor(travel / (2 * math.pi)).astype(int)
    revs_done = range(k.max()) if k.max() >= 1 else [0]
    windows = [k == i for i in revs_done]
    bob = float(np.mean([z[w].max() - z[w].min() for w in windows if w.any()]))
    none_down = r.foot_contact[m].sum(axis=1) == 0
    airborne_per_rev = [float(none_down[w].mean()) for w in windows if w.any()]
    side_phase = np.degrees(crank[:, 0] - crank[:, -1])
    loop_force = {}
    if r.loop_force.size:
        lf = r.loop_force[m]
        loop_force = {name: {"peak": float(lf[:, i].max()),
                             "p999": float(np.percentile(lf[:, i], 99.9))}
                      for i, name in enumerate(r.loops)}

    torque = {}
    for i, d in enumerate(r.drives):
        tq, w, u = r.torque[m, i], r.crank_speed[m, i], r.ctrl[m, i]
        avail = r.torque_max * np.clip(1.0 - np.abs(w) / r.speed_max, 0.0, None)
        motoring = tq * w > 0
        at_line = np.abs(tq[motoring]) >= 0.98 * avail[motoring]
        driven = np.abs(u) > 1e-9
        mean_abs = float(np.abs(tq).mean())
        torque[d] = {
            "peak": float(np.abs(tq).max()), "mean": mean_abs,
            "rms": float(np.sqrt(np.mean(tq**2))), "limit": r.torque_max,
            "rated": r.torque_rated,
            "mean_over_rated": mean_abs / r.torque_rated if r.torque_rated else None,
            "peak_fraction": float(np.abs(tq).max() / r.torque_max),
            "saturated": float(np.mean(np.abs(tq) >= 0.98 * r.torque_max)),
            "at_envelope": float(at_line.mean()) if at_line.size else 0.0,
            "speed_droop": float(1.0 - np.abs(w[driven]).mean() / np.abs(u[driven]).mean())
            if driven.any() else 0.0,
            "power": float(np.mean(tq * w)),
            "speed": float(np.mean(w)),
            # what a DC-motor servo would turn at under this mean load
            "speed_under_load": float(r.speed_max * (1.0 - mean_abs / r.torque_max)),
        }
    factor = crank_joint_factor(r.config) if r.config is not None else 1.0
    limit = torque_limit_nm(r.config) if r.config is not None else None
    from spiderpig.strength import crank_capacity

    caps = (crank_capacity({}, r.config) if r.config is not None else None) or {}
    weakest = min(caps, key=caps.get, default=None)
    slip = r.foot_slip[m][r.foot_contact[m]]
    down = r.foot_contact[m]
    n_down = down.sum(axis=1)
    hist = np.bincount(n_down, minlength=len(r.feet) + 1) / len(n_down)
    sides = {f[0] for f in r.feet}
    per_side = np.stack([down[:, [f.startswith(s) for f in r.feet]].sum(axis=1) for s in sides])
    turned = yaw[-1] - yaw[0]
    path = float(np.sum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)))
    return {
        "duration": float(dur),
        "speed": forward / dur * 1e3,
        "forward": forward * 1e3,
        "lateral": lateral * 1e3,
        "heading_drift": math.degrees(turned),
        "yaw_rate": math.degrees(turned) / dur,
        "yaw_per_rev": math.degrees(turned) / revs if abs(revs) > 0.25 else float("nan"),
        "turn_radius": path / abs(turned) * 1e3 if abs(turned) > 1e-6 else float("inf"),
        "revolutions": revs,
        "revolutions_abs": float(np.mean(np.abs(revs_each))),
        "drives_oppose": bool(revs_each.min() < -0.05 < 0.05 < revs_each.max()),
        "stride": forward * 1e3 / revs if abs(revs) > 0.25 else float("nan"),
        "bob": bob * 1e3,
        "height": float(z.mean() * 1e3),
        "pitch_range": math.degrees(float(np.ptp(r.attitude[m, 1]))),
        "roll_range": math.degrees(float(np.ptp(r.attitude[m, 2]))),
        "max_tilt": math.degrees(float(r.tilt[m].max())),
        "torque": torque,
        "torque_peak": max(v["peak"] for v in torque.values()),
        "torque_limit": r.torque_max,
        "joint_moment_factor": factor,
        "joint_moment_peak": factor * max(v["peak"] for v in torque.values()),
        "crank_capacity_nm": caps.get(weakest) if weakest else None,
        "crank_weakest": weakest,
        "torque_limit_recommended": limit,
        "joint_moment_at_limit": None if limit is None else factor * limit,
        "saturates": any(v["saturated"] > 0.0 for v in torque.values()),
        "slip": float(slip.mean() * 1e3) if slip.size else 0.0,
        "slip_max": float(slip.max() * 1e3) if slip.size else 0.0,
        "feet_down": float(n_down.mean()),
        "feet_down_hist": hist.tolist(),
        "airborne": float(hist[0]),
        "side_support_low": float((per_side < 2).any(axis=0).mean()),
        "loop_error": float(r.loop_error.max() * 1e3),
        "penetration": float(r.penetration.max() * 1e3),
        "fell": bool(r.tilt.max() > FELL_TILT),
        "body_contact": float(r.body_contact[m].mean()),
        "side_phase": float((side_phase[-1] + 180.0) % 360.0 - 180.0),
        "side_phase_max": float(np.abs(side_phase).max()),
        "airborne_per_rev": airborne_per_rev,
        "accel_z_peak_g": float(np.abs(r.base_acc[m, 2]).max() / G) if r.base_acc.size else 0.0,
        "foot_force_peak": float(r.foot_force[m].max()) if r.foot_force.size else 0.0,
        "loop_force": loop_force,
        "loop_force_peak": max((v["peak"] for v in loop_force.values()), default=0.0),
        "loop_force_p999": max((v["p999"] for v in loop_force.values()), default=0.0),
        "walks": bool(abs(forward * 1e3 / revs) >= MIN_STRIDE_MM if abs(revs) > 0.25 else False)
        and float(r.body_contact[m].mean()) < BODY_DOWN,
        "mass": r.mass,
        **_fall(r, skip),
    }


SPEED_DISAGREEMENT = 0.25   # the two models' speeds further apart than this is flagged
SUPPORT_LOW = 0.10          # a side on fewer than two feet more often than this is flagged
MIN_STRIDE_MM = 5.0         # under this much ground per revolution the robot doesn't walk
BODY_DOWN = 0.10            # a body on the floor more often than this: it isn't walking
CONTACT_NOTE = ("the support metrics (feet down, airborne, side_support_low) and the impact "
                "torque peaks follow the unvalidated contact softness (SimParams.contact_solref, "
                "10 ms; 5-20 ms moves the Klann quad's feet down 2.6-3.6 and airborne 9-0 %); "
                "speed, stride and mean torque don't")
QUASI_STATIC_NOTE = ("the quasi-static model assumes no slip on its lowest feet; MuJoCo's feet "
                     "slip and the Klann quad strides ~1.7x further per revolution even at a "
                     "crawl, so a speed ratio near 2 there is the model's kinematics, not the "
                     "sim's dynamics")
TORQUE_RANGE_NOTE = ("the torque peak is a range, not a number: the Klann quad's full-speed "
                     "peak reads 0.33-0.60 N·m over the unmeasured servo loop stiffness "
                     "(SimParams.stall_error) and 0.16-0.89 N·m over the reflected rotor "
                     "inertia 5e-4..2e-2 kg·m² (SimParams.crank_armature, UNVERIFIED); the "
                     "mean torque and the speed hardly move")


def compare_with_walk(metrics: dict, config: BuildConfig | None = None) -> dict:
    """MuJoCo's :func:`walk_metrics` beside the quasi-static model's
    (:func:`walk.straight_walk_metrics`) for ``config``: both speeds (mm/s, MuJoCo's
    scaled to a full-speed command by the drives' mean |crank rate|; ``nan`` when the
    drives opposed each other, a spin has no straight-walk speed), strides and bobs,
    the kinematic no-slip stride and the fraction of it the feet slip away
    (``slip_fraction``: the stride is the kinematic one minus the skating), the speed
    ratio, ``flags``: ``speeds_disagree`` (ratio off 1 by more than
    :data:`SPEED_DISAGREEMENT`; never for a spin), ``support_low`` (one side on fewer than
    two feet more than :data:`SUPPORT_LOW` of the time), ``does_not_walk`` (either model:
    MuJoCo's ``walks`` is false, or the quasi-static stride is under
    :data:`MIN_STRIDE_MM`) and ``fell``, and ``notes``: what the numbers hinge on
    (:data:`CONTACT_NOTE`, :data:`QUASI_STATIC_NOTE`, :data:`TORQUE_RANGE_NOTE`,
    :data:`PHASE_NOTE`)."""
    from spiderpig import walk

    config = config or BuildConfig()
    qs = walk.straight_walk_metrics(walk.walker(config))
    kin = kinematic_gait(config)
    dur = metrics["duration"]
    revs_per_s = metrics.get("revolutions_abs", abs(metrics["revolutions"])) / dur if dur else 0.0
    opposed = bool(metrics.get("drives_oppose", False))
    full = qs["rpm_max"] / 60.0
    at_full = (metrics["speed"] * (full / revs_per_s)
               if revs_per_s > 1e-9 and not opposed else float("nan"))
    ratio = abs(at_full) / qs["speed_mm_s"] if qs["speed_mm_s"] > 1e-9 else float("nan")
    stride = metrics["stride"]
    slip_fraction = (1.0 - stride / kin["stride"]
                     if math.isfinite(stride) and abs(kin["stride"]) > 1e-9 else float("nan"))
    flags = []
    if not opposed and (not math.isfinite(ratio) or abs(ratio - 1.0) > SPEED_DISAGREEMENT):
        flags.append("speeds_disagree")
    if metrics["side_support_low"] > SUPPORT_LOW:
        flags.append("support_low")
    if not opposed and (not metrics.get("walks", True) or not qs.get("walks", True)):
        flags.append("does_not_walk")
    if metrics["fell"]:
        flags.append("fell")
    return {
        "mujoco": {"speed_mm_s": at_full, "stride_mm": stride, "bob_mm": metrics["bob"],
                   "stride_kinematic_mm": kin["stride"], "slip_fraction": slip_fraction,
                   "feet_down": metrics["feet_down"], "airborne": metrics["airborne"],
                   "side_support_low": metrics["side_support_low"], "fell": metrics["fell"],
                   "fell_at_s": metrics["fell_at_s"], "yaw_rate_deg_s": metrics["yaw_rate"],
                   "walks": metrics.get("walks"), "body_contact": metrics["body_contact"]},
        "quasi_static": {"speed_mm_s": qs["speed_mm_s"], "stride_mm": qs["stride_mm"],
                         "bob_mm": qs["bob_mm"], "mean_contacts": qs["mean_contacts"],
                         "min_margin_mm": qs["min_margin_mm"], "walks": qs.get("walks")},
        "speed_ratio": ratio,
        "flags": flags,
        "notes": [CONTACT_NOTE, QUASI_STATIC_NOTE, TORQUE_RANGE_NOTE, PHASE_NOTE],
    }


# The steering check behind the viewer's turn authority: a straight reference run, a
# differential while walking ((1, 0.6): |L - R| = 0.4, the sides drifting apart without
# bound) and a turn in place ((0.5, -0.5)), each driven this long after half a second at
# rest; then a bounded excursion: the right side held back until the sides are STEER_STEPS
# degrees apart, re-locked there and walked on (the largest safe one is ``step_deg``). The
# Klann quad rolls over at 3.0 s into the differential (measured), so a shorter run would
# pass it; it walks a 90° offset at 19° of tilt and rolls over at 180° (measured).
STEER_TESTS = (("forward", 1.0, 1.0), ("walk_turn", 1.0, 0.6), ("spin", 0.5, -0.5))
STEER_STEPS = (90.0, 45.0)
STEER_SECONDS = 4.0
STEER_TILT = 20.0           # degrees: tilted further than this counts as unsafe too


def steering_check(config: BuildConfig | None = None, params: SimParams | None = None, *,
                   xml: str | None = None, meta: dict | None = None) -> dict:
    """Can this design walk and skid-steer in MuJoCo? Runs :data:`STEER_TESTS` and the
    :data:`STEER_STEPS` excursions (``xml`` with ``meta`` runs that model, as a worker
    process that just built it does) and returns ``turn``: the proven-safe ``|L - R|``
    while walking (0.4, or 0 when the robot fell or tilted past :data:`STEER_TILT`),
    ``spin``: the proven-safe speed of a turn in place (0.5 or 0), ``step_deg``: the
    largest side offset it walked with (90, 45 or 0: what :class:`PhaseLock` bounds a
    steering excursion to), ``forward``: the straight run's verdict (``fell``,
    ``max_tilt``, ``side_support_low``, ``speed``, ``walks``, ``body_contact``: the
    server's gate for the physics drive) and ``tests``: each run's command, ``fell``,
    ``fell_at_s``, ``max_tilt``, ``yaw_rate`` and ``side_support_low``. The viewer caps
    its commands to these (``drive/index.ts`` ``physicsCommand``) and says why."""
    config = config or BuildConfig()
    params = params or SimParams()
    if meta is None:
        _, meta = load_model(config, params)
    vmax = meta["actuators"][next(iter(meta["actuators"]))]["ctrlrange"][1]

    def run(controls, seconds, max_offset=math.inf):
        res = simulate(config, controls, seconds, params=params, record_every=4,
                       model_xml=xml, model_meta=meta,
                       lock=phase_lock(params, vmax, max_offset))
        return walk_metrics(res, skip=1.0)

    def verdict(cmd, m):
        return {"cmd": list(cmd), "fell": m["fell"], "fell_at_s": m["fell_at_s"],
                "fell_axis": m["fell_axis"], "max_tilt": m["max_tilt"],
                "yaw_rate": m["yaw_rate"], "side_support_low": m["side_support_low"],
                "side_phase_max": m["side_phase_max"]}

    tests, forward = {}, {}
    for name, left, right in STEER_TESTS:
        m = run([(0.0, 0.0, 0.0), (0.5, left * vmax, right * vmax)], 0.5 + STEER_SECONDS)
        tests[name] = verdict((left, right), m)
        if name == "forward":
            forward = {"fell": m["fell"], "fell_at_s": m["fell_at_s"], "max_tilt": m["max_tilt"],
                       "side_support_low": m["side_support_low"], "speed": m["speed"],
                       "walks": m["walks"], "body_contact": m["body_contact"],
                       "stride": m["stride"]}
    safe = {k: not v["fell"] and v["max_tilt"] < STEER_TILT for k, v in tests.items()}
    step_deg = 0.0
    for deg in STEER_STEPS:
        hold = math.radians(deg) / (0.4 * vmax)     # (1, 0.6) this long opens the offset
        m = run([(0.0, 0.0, 0.0), (0.5, vmax, 0.6 * vmax), (0.5 + hold, vmax, vmax)],
                0.5 + hold + STEER_SECONDS, max_offset=math.radians(deg))
        tests[f"step_{deg:.0f}"] = verdict((1.0, 0.6), m)
        if not m["fell"] and m["max_tilt"] < STEER_TILT:
            step_deg = deg
            break
    return {"turn": abs(STEER_TESTS[1][1] - STEER_TESTS[1][2]) if safe["walk_turn"] else 0.0,
            "spin": abs(STEER_TESTS[2][1]) if safe["spin"] else 0.0,
            "step_deg": step_deg, "forward": forward,
            "seconds": STEER_SECONDS, "tests": tests}


def _fall(r: SimResult, skip: float) -> dict:
    """When and how the robot fell (``fell_at_s``: seconds into the run, so before ``skip``
    means while it was still settling or the drives were just starting; ``fell_axis``:
    ``rolling`` or ``pitching`` by the larger attitude at that moment), or nothing."""
    over = np.flatnonzero(r.tilt > FELL_TILT)
    if over.size == 0:
        return {"fell_at_s": None, "fell_axis": None}
    k = int(over[0])
    pitch, roll = float(r.attitude[k, 1]), float(r.attitude[k, 2])
    return {"fell_at_s": float(r.t[k] - r.t[0]),
            "fell_axis": "rolling" if abs(roll) >= abs(pitch) else "pitching"}


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------


def crank_joint_factor(config: BuildConfig | None = None) -> float:
    """The moment a crankpin joint of the built-up crankshaft carries per N·m of drive
    torque: the chord between the crank's two farthest crankpins over the crank radius, at
    least 1.0. The segments above and below a rider meet only inside the rider's hole, so
    the torque crosses each post-to-web joint at the post; between two posts it is a couple
    over their chord. 2.0 on the Strider (pins 180° apart) and the Klann (0/90/180/270),
    1.0 for a single crankpin (the torque itself)."""
    config = config or BuildConfig()
    lk = linkage.get(config.linkage)
    joints = template_for(config).sample(np.array([T_REF])).joint_world
    pins, o = [], None
    for by_joint in joints.values():
        for name, p in by_joint.items():
            stem = name.split("_leg")[0]
            if stem in lk.crank[1:]:
                pins.append(np.asarray(p[0, :2], dtype=float))
            elif stem == lk.crank[0] and o is None:
                o = np.asarray(p[0, :2], dtype=float)
    if not pins:
        return 1.0
    o = np.zeros(2) if o is None else o
    r = float(np.mean([np.linalg.norm(p - o) for p in pins]))
    chord = max((float(np.linalg.norm(a - b)) for a, b in itertools.combinations(pins, 2)),
                default=0.0)
    return max(1.0, chord / r) if r > 1e-9 else 1.0


def kinematic_gait(config: BuildConfig | None = None, samples: int = 1440) -> dict:
    """What the kinematics alone promise, on rigid ground without slip (mm per revolution).

    One side's feet (the linkage's) over a crank revolution: the body rests
    on the lowest foot and moves as that foot moves back (``stride``);
    ``bob`` is the range of the lowest foot's height; ``stance_length`` is
    one foot's travel over the lowest 10 mm of its path (the classic step
    length).
    """
    config = config or BuildConfig()
    tmpl = template_for(config)
    ts = T_REF + np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    joint_world = tmpl.sample(ts).joint_world
    feet = np.stack([joint_world[b][j][:, :2] for b, j in feet_of(tmpl)])    # (F, T, 2)
    low = feet[:, :, 1].argmin(axis=0)
    idx = np.arange(samples)
    x_now, x_next = feet[low, idx, 0], feet[low, (idx + 1) % samples, 0]
    # the body moves against the stance foot; the drives turn the crank the way that walks
    # forward (crank_sign), so the stride reads positive for the sim's forward
    stride = float(-(x_next - x_now).sum()) * crank_sign(config)
    lowest = feet[low, idx, 1]
    f0 = feet[0]
    stance = f0[:, 1] <= f0[:, 1].min() + 10.0
    return {
        "stride": stride,
        "bob": float(np.ptp(lowest)),
        "stance_length": float(np.ptp(f0[stance, 0])),
        "foot_lift": float(np.ptp(f0[:, 1])),
        "legs_per_side": len(tmpl.meta["phases"]),
        "feet_per_side": int(feet.shape[0]),
    }


def kinematic_qpos(config: BuildConfig | None, t: float, params: SimParams | None = None):
    """The model's ``qpos`` with every link on the template at crank angle ``t``.

    The base stays at its reference pose; the cranks turn by ``t - t_ref``.
    """
    import mujoco

    config = config or BuildConfig()
    model, meta = load_model(config, params)
    bodies = meta["bodies"]         # the model's bodies, parents first (its kinematic tree)
    tmpl = template_for(config)
    jw = tmpl.sample(np.array([T_REF, float(t)])).joint_world

    def angle(mj: str) -> float:
        if mj == "base":
            return 0.0
        mb = bodies[mj]
        if mb["kind"] == "crank":
            return float(t) - T_REF
        kin = mb["kinematic"][0].split(".", 1)[1]   # "L.b1_leg0" -> "b1_leg0" (side template)
        j = [b for b in tmpl.bodies if b.name == kin][0].joints
        d = jw[kin][j[1].name] - jw[kin][j[0].name]
        a = np.arctan2(d[:, 1], d[:, 0])
        return float(a[1] - a[0])

    qpos = model.qpos0.copy()
    for name, mb in bodies.items():
        if name == "base":
            continue
        joint = f"{mb['side']}.crank" if mb["kind"] == "crank" else name
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        rel = angle(name) - angle(mb["parent"])
        if mb["kind"] == "crank":
            qpos[model.jnt_qposadr[jid]] = rel * meta["crank_sign"]
        else:
            qpos[model.jnt_qposadr[jid]] = math.remainder(rel, 2 * math.pi)
    return qpos


class MotionIndex:
    """What :func:`motions` needs once per model: the bodies' ids (``meta["bodies"]``
    order) and their reference positions in the base frame."""

    def __init__(self, model, meta: dict) -> None:
        import mujoco

        self.names = list(meta["bodies"])
        self.base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                             for n in self.names])
        self.ref = np.array([meta["bodies"][n]["ref_pos"] for n in self.names])    # (B, 3) m


def motions(data, index: MotionIndex) -> np.ndarray:
    """Every body's ``D`` (see :func:`body_motions`) as one ``(B, 3, 4)`` array (the top
    three rows, mech mm), in ``index.names`` order; vectorised over the bodies."""
    rb, pb = data.xmat[index.base].reshape(3, 3), data.xpos[index.base]
    rot = np.einsum("ji,bjk->bik", rb, data.xmat[index.ids].reshape(-1, 3, 3))  # rb.T @ R_b
    pos = (data.xpos[index.ids] - pb) @ rb                                      # rb.T @ dp
    out = np.empty((len(index.ids), 3, 4))
    out[:, :, :3] = rot
    out[:, :, 3] = (pos - np.einsum("bik,bk->bi", rot, index.ref)) * 1e3
    return out


def body_motions(model, data, meta: dict) -> dict[str, np.ndarray]:
    """Per MuJoCo body, its motion ``D`` (4x4, mech mm) relative to the base since ``qpos0``.

    ``D = X_b(now) · X_b(qpos0)⁻¹`` with ``X_b`` the body's pose in the base
    frame; a glb node of body ``b`` is drawn at ``D[b] @ (its frame-0 matrix)``
    under a root that carries the base's world pose (see :mod:`sim.mjcf`).
    Needs ``data`` after kinematics (``mj_forward`` / ``mj_step``). A session
    that asks every tick keeps a :class:`MotionIndex` and calls :func:`motions`.
    """
    index = MotionIndex(model, meta)
    out = {}
    for name, top in zip(index.names, motions(data, index), strict=True):
        d = np.eye(4)
        d[:3] = top
        out[name] = d
    return out


def loop_forces(data, eq_ids: np.ndarray) -> np.ndarray:
    """The in-plane force (N) each ``connect`` equality in ``eq_ids`` carried in the last
    step: the load on that loop's pin (the mechanism is planar: world ``x`` and ``z``; the
    lateral row is the redundant one). Zero for a loop the solver has no rows for."""
    import mujoco

    n = data.nefc
    rows = np.flatnonzero(data.efc_type[:n] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    ids, force = data.efc_id[rows], data.efc_force[rows]
    out = np.zeros(eq_ids.size)
    if not ids.size:
        return out
    # each equality's rows in solver order (a stable sort by id), found by bisection: what
    # ``force[ids == e]`` gave, without a mask per equality per step
    order = np.argsort(ids, kind="stable")
    ids, force = ids[order], force[order]
    lo = np.searchsorted(ids, eq_ids, side="left").tolist()
    hi = np.searchsorted(ids, eq_ids, side="right").tolist()
    f = force.tolist()
    for k, (a, b) in enumerate(zip(lo, hi, strict=True)):
        if b - a >= 3:
            out[k] = math.hypot(f[a], f[a + 2])
        elif b > a:
            out[k] = float(np.linalg.norm(force[a:b]))
    return out


def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """``np.cross`` of two 3-vectors, the same products and differences in the same order
    (bit for bit), without its axis bookkeeping (a third of a recorded step's time)."""
    a0, a1, a2 = a.tolist()
    b0, b1, b2 = b.tolist()
    return np.array([a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0])


class _LoopGaps:
    """:func:`loop_errors` of one model, its equalities looked up once."""

    def __init__(self, model) -> None:
        import mujoco

        self.eq = np.flatnonzero(model.eq_type == mujoco.mjtEq.mjEQ_CONNECT)
        self.b1, self.b2 = model.eq_obj1id[self.eq], model.eq_obj2id[self.eq]

    def __call__(self, model, data) -> np.ndarray:
        eq, b1, b2 = self.eq, self.b1, self.b2
        p1 = data.xpos[b1] + np.einsum("nij,nj->ni", data.xmat[b1].reshape(-1, 3, 3),
                                       model.eq_data[eq, 0:3])
        p2 = data.xpos[b2] + np.einsum("nij,nj->ni", data.xmat[b2].reshape(-1, 3, 3),
                                       model.eq_data[eq, 3:6])
        return np.linalg.norm(p1 - p2, axis=1)


def loop_errors(model, data) -> np.ndarray:
    """Gap (m) of every ``connect`` equality in the current ``data`` (after kinematics)."""
    return _LoopGaps(model)(model, data)
