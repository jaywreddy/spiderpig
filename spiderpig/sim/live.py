"""A MuJoCo session the viewer drives live (``/ws/sim``): commands in, poses out, for any design.

One session for every kind, over the one emitter's model (:func:`sim.mjcf.load_model`): what
it takes and what it streams follow what the model declares (its actuators, its free base,
its feet, its sites), never the kind's name:

* **commands** (:meth:`LiveSim.set_command`, :meth:`LiveSim.set_joints`): each crank's speed
  (a velocity actuator, a servo: ``cmd``, one fraction of its no-load speed per crank in
  ``drives`` order: a walker's two sides ``[left, right]``, a mechanism's one ``[speed]``)
  and each stepped joint's angle (a position actuator, ``joints``: degrees, the input's
  design angle or a spatial joint's from its zero, as ``/api/pose``'s ``j.<name>``);
* **state**: every body's pose (the node contract of :mod:`sim.mjcf`: the ``walker`` root at
  the base's world pose, every glb node at ``D_body @ M0``), each crank's travel and torque,
  each joint's angle and torque, the feet and the floor where it walks, the output and tool
  points where it has them (sites ``output`` / ``tool0``);
* **the base**: free on the floor where it walks (feet), fixed to the world where it doesn't
  (the model's, :func:`sim.mjcf.model_config`).

A frame is little-endian float32: ``header`` values named by the hello's ``columns``, then
12 per body in ``LiveSim.bodies`` order (the top three rows of ``D_body``, row-major, mech
mm). The columns, in this order, each where the design has it:

    t (s), base x y z (world mm), base quat w x y z,
    <drive>.angle per crank (rad travelled, the hinge's qpos: multiply by hello's
    ``crank_sign`` for the design's crank angle),
    feet_down (count; with feet),
    <drive>.torque per crank (N·m, the actuator force: compare with hello's
    ``torque_rated`` and ``torque_stall``),
    body_down (1 when a body that carries no foot touches the floor; with feet),
    side_phase (rad, crank L − crank R: what the phase lock holds; with two cranks),
    az (the base's vertical acceleration, m/s²; with a free base),
    pin_load (N, the largest in-plane force a loop carries this step),
    <joint>.angle per stepped joint (rad, its actuator's ``ctrl`` units), then
    <joint>.torque per stepped joint (N·m),
    <site>.x <site>.y <site>.z per site (world mm: the output point, the tool's tip)

A walker's is the 17 columns of :data:`HEADER` (unchanged since the session was a walker's
alone). The crank commands go through the :class:`sim.run.PhaseLock` the params ask for
when there are two cranks, a walker's sides (the right servo ``servo_mismatch`` slower than
told; a steering excursion bounded to the hello's ``steering.step_deg``), as
:func:`sim.run.simulate` does; a single crank is the servo in position mode tracking its
commanded travel (:data:`sim.run.HOLD_GAIN` over its speed loop, as :func:`sim.run.hold`
holds it: a zero speed holds the crank where it is instead of letting it creep). A stepped
joint's position actuator is told its target every step.

Threads: the server steps (:meth:`LiveSim.advance`) and packs
(:meth:`LiveSim.frame`) on a worker thread while its websocket reader takes
commands on the event loop. ``MjData`` is not thread-safe (a reset under a
running ``mj_step`` has crashed the process), so the reader only *asks*: a
reset is :meth:`request_reset`, applied by the next ``advance``; a command is a
small array the step copies into ``ctrl``. :meth:`reset` itself (a direct
caller's) and the stepping take one lock.
"""

from __future__ import annotations

import copy
import math
import threading
from collections.abc import Mapping

import numpy as np

from spiderpig.config import BuildConfig
from spiderpig.sim.mjcf import SimParams, load_model, motor_line
from spiderpig.sim.run import (
    HOLD_GAIN,
    MotionIndex,
    kinematic_gait,
    loop_forces,
    motions,
    phase_lock,
    steering_check,
)

HEADER = 17
"""A walker's header columns (two cranks, feet, a free base): :attr:`LiveSim.header`."""
BASE_COLUMNS = ("t", "base.x", "base.y", "base.z", "base.qw", "base.qx", "base.qy", "base.qz")
# s of physics per tick at most (three 60 Hz ticks): a stalled server runs in slow motion
# instead of bursting 100 ms of physics into one frame
MAX_CATCH_UP = 0.05


class LiveSim:
    """One design in MuJoCo, stepped on demand under its drives' commands.

    ``model`` and ``meta`` already built (:func:`sim.mjcf.load_model`'s) skip the build,
    for a server that built them elsewhere; the model is copied (the drives'
    ``forcerange`` follows the servo's speed-torque line every step,
    :func:`sim.mjcf.motor_line`). ``config`` is the design asked for (the hello's
    ``design``); the model is of :func:`sim.mjcf.model_config` of it."""

    def __init__(self, config: BuildConfig, params: SimParams | None = None, *,
                 model=None, meta: dict | None = None) -> None:
        import mujoco

        self._mj = mujoco
        self.params = params or SimParams()
        if model is None or meta is None:
            model, meta = load_model(config, params)
        self.config = config
        self.model, self.meta = copy.copy(model), meta
        self.data = mujoco.MjData(self.model)
        acts = self.meta["actuators"]
        # what drives it: a servo's speed on each crank, a stepper's angle on each ranged joint
        self.drives = [n for n, a in acts.items() if a.get("type") == "velocity"]
        self.joints = [n for n, a in acts.items() if a.get("type") == "position"]
        self._act = self._actuators(self.drives)
        self._pos = self._actuators(self.joints)
        first = acts[self.drives[0]] if self.drives else {}
        self.vmax = float(first["ctrlrange"][1]) if first else 0.0
        self.tau = float(first["forcerange"][1]) if first else 0.0
        self.rated = first.get("rated_torque")
        self.walks = bool(self.meta.get("freejoint"))     # free on a floor (else fixed)
        self.bodies = list(self.meta["bodies"])
        self._index = MotionIndex(self.model, self.meta)
        feet = self.meta.get("feet") or {}
        self._floor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self._feet = np.array(sorted(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f["geom"])
            for f in feet.values()), dtype=int)
        # bodies whose floor contacts aren't a fall (a foot's link and the links pinned there)
        self._foot_bodies = np.array(sorted(
            {int(self.model.geom_bodyid[g]) for g in self._feet}
            | {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b)
               for f in feet.values() for b in f.get("links", ())}), dtype=int)
        self._eq = np.flatnonzero(self.model.eq_type == mujoco.mjtEq.mjEQ_CONNECT)
        self._base_dof = (int(self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base")])
            if self.walks else None)
        self.sites = list(self.meta.get("sites") or {})      # its output point, its tool's tip
        self._sites = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, s)
                                for s in self.sites], dtype=int)
        self.command = np.zeros(len(self._act))
        # two cranks are a walker's sides, held in phase; a lone one is a servo in position
        # mode tracking its commanded travel
        self.lock = phase_lock(self.params, self.vmax) if len(self._act) == 2 else None
        if self.lock is not None and "steering" in self.meta:
            self.lock.max_offset = math.radians(self.meta["steering"].get("step_deg", 0.0))
        self._travel = np.zeros(len(self._act))     # a lone crank's commanded travel, rad
        jnt = self.model.actuator_trnid[self._pos, 0]
        self.home = self.model.qpos0[self.model.jnt_qposadr[jnt]].copy()   # rad, at qpos0
        self.ranges = self.model.actuator_ctrlrange[self._pos].copy()
        self.targets = self.home.copy()             # the stepped joints' angles, rad
        self.columns = self._columns()
        self.header = len(self.columns)
        self._residual = 0.0        # s of a tick not yet stepped (the step is whole)
        self._lock = threading.Lock()
        self._reset_asked = False
        self.reset()

    def _actuators(self, names: list[str]) -> np.ndarray:
        return np.array([self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_ACTUATOR, n)
                         for n in names], dtype=int)

    def _columns(self) -> list[str]:
        """The frame's header columns, in the module docstring's order (the hello's)."""
        cols = list(BASE_COLUMNS)
        cols += [f"{d}.angle" for d in self.drives]
        if self._feet.size:
            cols.append("feet_down")
        cols += [f"{d}.torque" for d in self.drives]
        if self._feet.size:
            cols.append("body_down")
        if self.lock is not None:
            cols.append("side_phase")
        if self._base_dof is not None:
            cols.append("az")
        cols.append("pin_load")
        cols += [f"{q}.angle" for q in self.joints]
        cols += [f"{q}.torque" for q in self.joints]
        cols += [f"{s}.{a}" for s in self.sites for a in "xyz"]
        return cols

    def reset(self) -> None:
        """Back to ``qpos0`` now (the stepping thread's own call, or a test's). The commands
        stand: a walker commanded ahead walks on, an arm goes back to its joints' targets."""
        with self._lock:
            self._reset()

    def request_reset(self) -> None:
        """Reset at the start of the next :meth:`advance` (another thread's call)."""
        self._reset_asked = True

    def _reset(self) -> None:
        self._reset_asked = False
        self._mj.mj_resetData(self.model, self.data)
        self.data.ctrl[self._pos] = self.targets
        self._mj.mj_forward(self.model, self.data)
        self._residual = 0.0
        self._travel = self.data.actuator_length[self._act].copy()
        if self.lock is not None:
            self.lock.reset()

    def set_command(self, *speeds: float) -> None:
        """Crank speeds as fractions of the servo's no-load speed (+ = forward), one per
        crank in :attr:`drives` order (a walker's ``left, right``); anything but that many
        finite numbers is a ``ValueError`` (a NaN would freeze the solver). Safe from any
        thread: the array is read into ``ctrl`` by the step."""
        n = len(self.drives)
        want = (f"{n} finite number{'s' if n != 1 else ''}, one per crank {self.drives}"
                if n else "absent: this design has no crank (its joints "
                f"{self.joints} take {{\"joints\": {{name: degrees}}}})")
        try:
            cmd = np.array(speeds, dtype=float)
        except (TypeError, ValueError):
            raise ValueError(f"cmd must be {want}") from None
        if not n or cmd.shape != (n,) or not np.isfinite(cmd).all():
            raise ValueError(f"cmd must be {want}")
        self.command = np.clip(cmd, -1.0, 1.0) * self.vmax      # a new array: one swap

    def set_joints(self, degrees: Mapping[str, float]) -> None:
        """Stepped joints' target angles in degrees (an input's design angle, a spatial
        joint's from its zero: ``/api/pose``'s ``j.<name>``), any subset of :attr:`joints`,
        each clipped to its range; the rest keep theirs. A ``ValueError`` for an unknown
        joint or a value that isn't a finite number. Safe from any thread (one swap)."""
        if not isinstance(degrees, Mapping) or not degrees:
            raise ValueError(f"joints must be an object of degrees by joint, of {self.joints}")
        unknown = sorted(set(degrees) - set(self.joints))
        if unknown:
            raise ValueError(f"unknown joints {unknown}: this design's are {self.joints}"
                             if self.joints else
                             f"unknown joints {unknown}: this design has no stepped joint "
                             f"(its cranks {self.drives} take {{\"cmd\": [...]}})")
        targets = self.targets.copy()
        for name, v in degrees.items():
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"joints.{name} must be a finite number of degrees; got {v!r}")
            k = self.joints.index(name)
            targets[k] = min(max(math.radians(v), self.ranges[k, 0]), self.ranges[k, 1])
        self.targets = targets

    def advance(self, seconds: float) -> None:
        """Step the physics by ``seconds`` (whole timesteps; the remainder carries into the
        next call so the sim clock keeps pace with the wall clock; at most
        :data:`MAX_CATCH_UP` per call). A requested reset happens first."""
        with self._lock:
            if self._reset_asked:
                self._reset()
            dt = self.model.opt.timestep
            self._residual += min(max(seconds, 0.0), MAX_CATCH_UP)
            n = int(self._residual // dt)
            self._residual -= n * dt
            cmd, targets, act, d = self.command, self.targets, self._act, self.data
            # a lone crank's lag behind its commanded travel: at most what saturates its
            # position loop (more would only wind up while it stalls)
            lag = self.vmax / HOLD_GAIN
            for _ in range(n):
                if act.size:
                    phi = d.actuator_length[act]
                    if self.lock is not None:
                        u = self.lock.ctrl(cmd, phi, dt)
                    else:
                        self._travel = np.clip(self._travel + cmd * dt, phi - lag, phi + lag)
                        u = cmd + HOLD_GAIN * (self._travel - phi)
                    d.ctrl[act] = np.clip(u, -self.vmax, self.vmax)
                    motor_line(self.model, d, act, self.vmax, self.tau)
                d.ctrl[self._pos] = targets
                self._mj.mj_step(self.model, d)

    def steering(self) -> dict:
        """:func:`sim.run.steering_check` of this model, computed once per model (a server
        does it when it builds the model; this fills in for a model loaded here). Its
        ``step_deg`` bounds this session's steering excursions (:class:`sim.run.PhaseLock`).
        A walker's: a fixed design has none (``ValueError``)."""
        if not self.walks:
            raise ValueError(f"{self.config.linkage} doesn't walk: it has no steering")
        if "steering" not in self.meta:
            self.meta["steering"] = steering_check(self.config, self.params)
        if self.lock is not None:
            self.lock.max_offset = math.radians(self.meta["steering"].get("step_deg", 0.0))
        return self.meta["steering"]

    def kinematic(self) -> dict:
        """:func:`sim.run.kinematic_gait` of this design (the no-slip stride the HUD shows
        beside the measured one), computed once per model. A walker's."""
        if "kinematic" not in self.meta:
            self.meta["kinematic"] = kinematic_gait(self.config)
        return self.meta["kinematic"]

    def hello(self) -> dict:
        """What the viewer needs before the first frame: the bodies and nodes, the frame's
        ``columns``, what drives the design (``drives``: its cranks; ``joints``: its stepped
        joints, each one's range and start in degrees and its holding torque) and what it
        has (``walks``, ``feet``, ``sites``); a walker's steering check, gait and lock."""
        cfg = self.config
        acts = self.meta["actuators"]
        out = {"bodies": self.bodies, "nodes": self.meta["nodes"], "header": self.header,
               "columns": self.columns, "walks": self.walks,
               "vmax": self.vmax, "drives": self.drives, "feet": len(self._feet),
               "joints": {q: {"lo": math.degrees(self.ranges[k, 0]),
                              "hi": math.degrees(self.ranges[k, 1]),
                              "value": math.degrees(self.home[k]),
                              "effort": float(acts[q]["forcerange"][1])}
                          for k, q in enumerate(self.joints)},
               "sites": self.sites,
               "torque_stall": self.tau, "torque_rated": self.rated,
               "mass": self.meta["mass"]["total"], "payload_g": self.params.payload_g,
               "timestep": self.model.opt.timestep, "crank_sign": self.meta["crank_sign"],
               "rest_height": self.meta["rest_height"], "robot": cfg.robot,
               "design": {"linkage": cfg.linkage, "module": cfg.module, "key": cfg.key}}
        if self.walks:
            steer = self.steering()
            out.update({"steering": steer, "forward": steer.get("forward", {}),
                        "kinematic_stride_mm": self.kinematic()["stride"]})
        if self.lock is not None:
            out["phase_lock"] = {"kp": self.lock.kp, "ki": self.lock.ki,
                                 "servo_mismatch": self.lock.mismatch,
                                 "step_deg": math.degrees(self.lock.max_offset)
                                 if math.isfinite(self.lock.max_offset) else None}
        return out

    def _floor_contacts(self) -> np.ndarray:
        """The geoms on the floor now."""
        d = self.data
        g1, g2 = d.contact.geom1[:d.ncon], d.contact.geom2[:d.ncon]
        on_floor = (g1 == self._floor) | (g2 == self._floor)
        return np.where(g1 == self._floor, g2, g1)[on_floor]

    def feet_down(self) -> int:
        """How many feet touch the floor now."""
        if not self._feet.size:
            return 0
        return int(np.count_nonzero(np.isin(self._floor_contacts(), self._feet)))

    def body_down(self) -> bool:
        """Does a body that carries no foot touch the floor now (the frame, a link's
        capsule away from the feet)? Never, with no floor."""
        if self._floor < 0:
            return False
        other = self._floor_contacts()
        return bool(np.any(~np.isin(self.model.geom_bodyid[other], self._foot_bodies)))

    def frame(self) -> bytes:
        """The binary frame now: :attr:`columns`' values, then every body's motion."""
        with self._lock:
            d = self.data
            base, act, pos = self._index.base, self._act, self._pos
            head: list = [(d.time,), d.xpos[base] * 1e3, d.xquat[base], d.actuator_length[act]]
            if self._feet.size:
                head.append((self.feet_down(),))
            head.append(d.actuator_force[act])
            if self._feet.size:
                head.append((self.body_down(),))
            if self.lock is not None:
                head.append((d.actuator_length[act[0]] - d.actuator_length[act[-1]],))
            if self._base_dof is not None:
                head.append((d.qacc[self._base_dof + 2],))
            head.append((loop_forces(d, self._eq).max() if self._eq.size else 0.0,))
            head += [d.actuator_length[pos], d.actuator_force[pos],
                     (d.site_xpos[self._sites] * 1e3).ravel()]
            values = np.concatenate([np.asarray(h, dtype=float).ravel() for h in head])
            if values.size != self.header:      # (the columns and the values disagree)
                raise AssertionError(f"frame header {values.size} != columns {self.header}")
            return np.concatenate([values, motions(d, self._index).ravel()]) \
                .astype("<f4").tobytes()

    def upright(self) -> bool:
        """Is the base's up axis within 45 degrees of vertical?"""
        r = self.data.xmat[self._index.base].reshape(3, 3)
        return r[2, 1] > math.cos(math.pi / 4)
