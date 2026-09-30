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
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

import numpy as np

from spiderpig.config import BuildConfig
from spiderpig.fabricate import template_for
from spiderpig.linkage import feet_of
from spiderpig.sim.mjcf import T_REF, SimParams, load_model, robot_model

Controls = Callable[[float], Sequence[float]] | Sequence

FELL_TILT = math.radians(45.0)      # the base tilted further than this: it fell over


def _control_fn(controls: Controls | None, vmax: float) -> Callable[[float], np.ndarray]:
    if controls is None:
        controls = (vmax, vmax)
    if callable(controls):
        return lambda t: np.asarray(controls(t), dtype=float)
    items = list(controls)
    if len(items) == 2 and all(np.isscalar(x) for x in items):
        const = np.asarray(items, dtype=float)
        return lambda t: const
    sched = sorted((float(t0), np.array([a, b], dtype=float)) for t0, a, b in items)

    def f(t: float) -> np.ndarray:
        out = np.zeros(2)
        for t0, v in sched:
            if t + 1e-12 >= t0:
                out = v
        return out

    return f


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
    body_contact: np.ndarray        # (N,) bool: something other than a foot touches the floor
    drives: list[str] = field(default_factory=list)
    feet: list[str] = field(default_factory=list)
    speed_max: float = 0.0          # rad/s, the servo's no-load speed
    torque_max: float = 0.0         # N·m, the servo's stall torque
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
) -> SimResult:
    """Run the robot for ``seconds`` under ``controls`` (default: both drives full forward)."""
    import mujoco

    config = config or BuildConfig()
    params = params or SimParams()
    model, meta = load_model(config, params)
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
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    loops = bool(np.any(model.eq_type == mujoco.mjtEq.mjEQ_CONNECT))

    n_steps = int(round(seconds / model.opt.timestep))
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
    }
    f6, v6 = np.zeros(6), np.zeros(6)
    row = 0
    for step in range(n_steps):
        t0 = data.time
        u = np.clip(control(t0), -vmax, vmax)
        data.ctrl[act] = u
        mujoco.mj_step(model, data)
        if step % record_every:
            continue
        # everything below is the state at t0 (mj_step's forward pass), not after it
        out["t"][row] = t0
        out["base_pos"][row] = data.xpos[base]
        out["base_quat"][row] = data.xquat[base]
        *att, tilt = _attitude(data.xmat[base])
        out["attitude"][row], out["tilt"][row] = att, tilt
        out["crank"][row] = data.actuator_length[act]
        out["crank_speed"][row] = data.actuator_velocity[act]
        out["ctrl"][row] = u
        out["torque"][row] = data.actuator_force[act]
        if loops:
            out["loop_error"][row] = loop_errors(model, data).max()
        for i in range(data.ncon):
            c = data.contact[i]
            if floor not in (c.geom1, c.geom2):
                continue
            g = c.geom2 if c.geom1 == floor else c.geom1
            out["penetration"][row] = max(out["penetration"][row], -c.dist)
            k = foot_geom.get(g)
            if k is None:
                out["body_contact"][row] = True
                continue
            mujoco.mj_contactForce(model, data, i, f6)
            b = model.geom_bodyid[g]
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, b, v6, 0)
            v = v6[3:] + np.cross(v6[:3], c.pos - data.xpos[b])
            nrm = c.frame[:3]
            slip = float(np.linalg.norm(v - (v @ nrm) * nrm))
            out["foot_contact"][row, k] = True
            out["foot_force"][row, k] += f6[0]
            prev = out["foot_slip"][row, k]
            out["foot_slip"][row, k] = slip if np.isnan(prev) else max(prev, slip)
        row += 1

    return SimResult(**{k: v[:row] for k, v in out.items()}, drives=drives, feet=feet,
                     speed_max=vmax, torque_max=tau, mass=meta["mass"]["total"],
                     config=config, meta=meta)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------


def walk_metrics(result: SimResult, skip: float = 0.5) -> dict:
    """How the robot walked, from ``skip`` seconds on (lengths in mm, angles in degrees).

    * ``speed``: mean speed along the initial heading (mm/s); ``lateral``: drift
      across it (mm); ``yaw_rate`` (deg/s) and ``heading_drift`` (deg);
    * ``revolutions`` of the cranks (mean of the drives, signed) and ``stride``,
      the forward travel per revolution (mm);
    * ``bob``: base height range per revolution (mm); ``pitch_range``, ``roll_range``;
    * per drive (``torque``): peak and mean |torque| (N·m), the stall torque,
      ``saturated`` (fraction of samples at the torque limit) and ``envelope``,
      the peak ratio of torque to what a DC-motor servo gives at that speed,
      ``stall * (1 - |speed| / no_load)`` (> 1: the real servo would slow down),
      ``speed_under_load`` (rad/s), what such a servo turns at under the mean
      load at full voltage, and mean mechanical ``power`` (W);
    * ``slip``: mean / max tangential speed of feet in contact (mm/s);
      ``feet_down``: mean number of feet on the floor;
    * ``loop_error``: worst loop-closure gap (mm); ``penetration``: deepest
      floor contact (mm); ``fell``: tilted past 45°; ``body_contact``: fraction
      of time something other than a foot is down.

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
    revs = float(np.mean(crank[-1] - crank[0]) / (2 * math.pi))

    # bob per revolution (by the mean crank angle travelled)
    travel = np.abs(np.mean(crank - crank[0], axis=1))
    z = pos[:, 2]
    k = np.floor(travel / (2 * math.pi)).astype(int)
    windows = [z[k == i] for i in range(k.max())] if k.max() >= 1 else [z]
    bob = float(np.mean([w.max() - w.min() for w in windows if len(w)]))

    torque = {}
    for i, d in enumerate(r.drives):
        tq, w = r.torque[m, i], r.crank_speed[m, i]
        avail = r.torque_max * np.clip(1.0 - np.abs(w) / r.speed_max, 0.0, None)
        motoring = tq * w > 0
        ratio = np.abs(tq[motoring]) / np.maximum(avail[motoring], 1e-9)
        torque[d] = {
            "peak": float(np.abs(tq).max()), "mean": float(np.abs(tq).mean()),
            "rms": float(np.sqrt(np.mean(tq**2))), "limit": r.torque_max,
            "peak_fraction": float(np.abs(tq).max() / r.torque_max),
            "saturated": float(np.mean(np.abs(tq) >= 0.98 * r.torque_max)),
            "envelope": float(ratio.max()) if ratio.size else 0.0,
            "envelope_p95": float(np.percentile(ratio, 95)) if ratio.size else 0.0,
            "power": float(np.mean(tq * w)),
            "speed": float(np.mean(w)),
            # what a DC-motor servo would turn at under this mean load
            "speed_under_load": float(r.speed_max * (1.0 - np.abs(tq).mean() / r.torque_max)),
        }
    slip = r.foot_slip[m][r.foot_contact[m]]
    return {
        "duration": float(dur),
        "speed": forward / dur * 1e3,
        "forward": forward * 1e3,
        "lateral": lateral * 1e3,
        "heading_drift": math.degrees(yaw[-1] - yaw[0]),
        "yaw_rate": math.degrees(yaw[-1] - yaw[0]) / dur,
        "revolutions": revs,
        "stride": forward * 1e3 / revs if abs(revs) > 0.25 else float("nan"),
        "bob": bob * 1e3,
        "height": float(z.mean() * 1e3),
        "pitch_range": math.degrees(float(np.ptp(r.attitude[m, 1]))),
        "roll_range": math.degrees(float(np.ptp(r.attitude[m, 2]))),
        "max_tilt": math.degrees(float(r.tilt[m].max())),
        "torque": torque,
        "torque_peak": max(v["peak"] for v in torque.values()),
        "torque_limit": r.torque_max,
        "saturates": any(v["saturated"] > 0.0 for v in torque.values()),
        "slip": float(slip.mean() * 1e3) if slip.size else 0.0,
        "slip_max": float(slip.max() * 1e3) if slip.size else 0.0,
        "feet_down": float(r.foot_contact[m].sum(axis=1).mean()),
        "loop_error": float(r.loop_error.max() * 1e3),
        "penetration": float(r.penetration.max() * 1e3),
        "fell": bool(r.tilt.max() > FELL_TILT),
        "body_contact": float(r.body_contact[m].mean()),
        "mass": r.mass,
    }


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------


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
    stride = float(-(x_next - x_now).sum())
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
    rm = robot_model(config, (params or SimParams()).printed_fill,
                     (params or SimParams()).hull_tolerance)
    tmpl = template_for(config)
    jw = tmpl.sample(np.array([T_REF, float(t)])).joint_world

    def angle(mj: str) -> float:
        if mj == "base":
            return 0.0
        mb = rm.bodies[mj]
        if mb.kind == "crank":
            return float(t) - T_REF
        kin = mb.kinematic[0].split(".", 1)[1]      # "L.b1_leg0" -> "b1_leg0" (side template)
        j = [b for b in tmpl.bodies if b.name == kin][0].joints
        d = jw[kin][j[1].name] - jw[kin][j[0].name]
        a = np.arctan2(d[:, 1], d[:, 0])
        return float(a[1] - a[0])

    qpos = model.qpos0.copy()
    for name, mb in rm.bodies.items():
        if name == "base":
            continue
        joint = f"{mb.side}.crank" if mb.kind == "crank" else name
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        rel = angle(name) - angle(mb.parent)
        if mb.kind == "crank":
            qpos[model.jnt_qposadr[jid]] = rel * meta["crank_sign"]
        else:
            qpos[model.jnt_qposadr[jid]] = math.remainder(rel, 2 * math.pi)
    return qpos


def body_motions(model, data, meta: dict) -> dict[str, np.ndarray]:
    """Per MuJoCo body, its motion ``D`` (4x4, mech mm) relative to the base since ``qpos0``.

    ``D = X_b(now) · X_b(qpos0)⁻¹`` with ``X_b`` the body's pose in the base
    frame; a glb node of body ``b`` is drawn at ``D[b] @ (its frame-0 matrix)``
    under a root that carries the base's world pose (see :mod:`sim.mjcf`).
    Needs ``data`` after kinematics (``mj_forward`` / ``mj_step``).
    """
    base = data.body("base")
    rb, pb = base.xmat.reshape(3, 3), base.xpos
    out = {}
    for name, info in meta["bodies"].items():
        b = data.body(name)
        rot = rb.T @ b.xmat.reshape(3, 3)
        pos = rb.T @ (b.xpos - pb)
        d = np.eye(4)
        d[:3, :3] = rot
        d[:3, 3] = (pos - rot @ np.asarray(info["ref_pos"])) * 1e3
        out[name] = d
    return out


def loop_errors(model, data) -> np.ndarray:
    """Gap (m) of every ``connect`` equality in the current ``data`` (after kinematics)."""
    import mujoco

    eq = np.flatnonzero(model.eq_type == mujoco.mjtEq.mjEQ_CONNECT)
    b1, b2 = model.eq_obj1id[eq], model.eq_obj2id[eq]
    p1 = data.xpos[b1] + np.einsum("nij,nj->ni", data.xmat[b1].reshape(-1, 3, 3),
                                   model.eq_data[eq, 0:3])
    p2 = data.xpos[b2] + np.einsum("nij,nj->ni", data.xmat[b2].reshape(-1, 3, 3),
                                   model.eq_data[eq, 3:6])
    return np.linalg.norm(p1 - p2, axis=1)
