"""A MuJoCo session the viewer drives live (``/ws/sim``): crank commands in, poses out.

The browser sends drive commands (``left``, ``right`` in [-1, 1] of the servo's
no-load speed); the server steps the model in real time and answers with one
binary frame per tick, which the viewer draws with the node contract of
:mod:`sim.mjcf` (the ``walker`` root at the base's world pose, every glb node
at ``D_body @ M0``).

Frame layout (little-endian float32), ``HEADER`` values then 12 per body in
``LiveSim.bodies`` order (the top three rows of ``D_body``, row-major, mech
mm)::

    t (s), base x y z (world mm), base quat w x y z,
    crank L, crank R (rad travelled, the hinge's qpos: multiply by hello's
    ``crank_sign`` for the design's crank angle), feet down (count),
    torque L, torque R (N·m on the crank, the actuator force: compare with
    hello's ``torque_rated`` and ``torque_stall``),
    body down (1 when a body that carries no foot touches the floor),
    side phase (rad, crank L − crank R: what the phase lock holds),
    az (the base's vertical acceleration, m/s²), pin load (N, the largest
    in-plane force a loop carries this step)

The commands go through the :class:`sim.run.PhaseLock` the params ask for
(the right servo ``servo_mismatch`` slower than told; a steering excursion
bounded to the hello's ``steering.step_deg``), as :func:`sim.run.simulate` does.

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

import numpy as np

from spiderpig.config import BuildConfig
from spiderpig.sim.mjcf import SimParams, load_model, motor_line
from spiderpig.sim.run import (
    MotionIndex,
    kinematic_gait,
    loop_forces,
    motions,
    phase_lock,
    steering_check,
)

HEADER = 17
# s of physics per tick at most (three 60 Hz ticks): a stalled server runs in slow motion
# instead of bursting 100 ms of physics into one frame
MAX_CATCH_UP = 0.05


class LiveSim:
    """One robot in MuJoCo, stepped on demand under crank-speed commands.

    ``model`` and ``meta`` already built (:func:`sim.mjcf.load_model`'s) skip the build,
    for a server that built them elsewhere; the model is copied (the drives'
    ``forcerange`` follows the servo's speed-torque line every step,
    :func:`sim.mjcf.motor_line`)."""

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
        self.drives = list(self.meta["actuators"])
        self._act = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, d)
                              for d in self.drives])
        first = self.meta["actuators"][self.drives[0]]
        self.vmax = float(first["ctrlrange"][1])
        self.tau = float(first["forcerange"][1])
        self.rated = first.get("rated_torque")
        self.bodies = list(self.meta["bodies"])
        self._index = MotionIndex(self.model, self.meta)
        self._floor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self._feet = np.array(sorted(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f["geom"])
            for f in self.meta["feet"].values()))
        # bodies whose floor contacts aren't a fall (a foot's link and the links pinned there)
        self._foot_bodies = np.array(sorted(
            {self.model.geom_bodyid[g] for g in self._feet}
            | {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b)
               for f in self.meta["feet"].values() for b in f.get("links", ())}))
        self._eq = np.flatnonzero(self.model.eq_type == mujoco.mjtEq.mjEQ_CONNECT)
        self._base_dof = self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base")]
        self.command = np.zeros(len(self._act))
        self.lock = phase_lock(self.params, self.vmax)
        if "steering" in self.meta:
            self.lock.max_offset = math.radians(self.meta["steering"].get("step_deg", 0.0))
        self._residual = 0.0        # s of a tick not yet stepped (the step is whole)
        self._lock = threading.Lock()
        self._reset_asked = False
        self.reset()

    def reset(self) -> None:
        """Back to ``qpos0`` now (the stepping thread's own call, or a test's)."""
        with self._lock:
            self._reset()

    def request_reset(self) -> None:
        """Reset at the start of the next :meth:`advance` (another thread's call)."""
        self._reset_asked = True

    def _reset(self) -> None:
        self._reset_asked = False
        self._mj.mj_resetData(self.model, self.data)
        self._mj.mj_forward(self.model, self.data)
        self._residual = 0.0
        self.lock.reset()

    def set_command(self, left: float, right: float) -> None:
        """Crank speeds as fractions of the servo's no-load speed (+ = forward); anything
        but two finite numbers is a ``ValueError`` (a NaN would freeze the solver). Safe
        from any thread: the array is read into ``ctrl`` by the step."""
        try:
            cmd = np.array([left, right], dtype=float)
        except (TypeError, ValueError):
            raise ValueError("cmd must be two numbers [left, right]") from None
        if cmd.shape != (2,) or not np.isfinite(cmd).all():
            raise ValueError("cmd must be two finite numbers [left, right]")
        self.command = np.clip(cmd, -1.0, 1.0) * self.vmax      # a new array: one swap

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
            cmd = self.command
            for _ in range(n):
                self.data.ctrl[self._act] = np.clip(
                    self.lock.ctrl(cmd, self.data.actuator_length[self._act], dt),
                    -self.vmax, self.vmax)
                motor_line(self.model, self.data, self._act, self.vmax, self.tau)
                self._mj.mj_step(self.model, self.data)

    def steering(self) -> dict:
        """:func:`sim.run.steering_check` of this model, computed once per model (a server
        does it when it builds the model; this fills in for a model loaded here). Its
        ``step_deg`` bounds this session's steering excursions (:class:`sim.run.PhaseLock`)."""
        if "steering" not in self.meta:
            self.meta["steering"] = steering_check(self.config, self.params)
        self.lock.max_offset = math.radians(self.meta["steering"].get("step_deg", 0.0))
        return self.meta["steering"]

    def kinematic(self) -> dict:
        """:func:`sim.run.kinematic_gait` of this design (the no-slip stride the HUD shows
        beside the measured one), computed once per model."""
        if "kinematic" not in self.meta:
            self.meta["kinematic"] = kinematic_gait(self.config)
        return self.meta["kinematic"]

    def hello(self) -> dict:
        """What the viewer needs before the first frame."""
        cfg = self.config
        return {"bodies": self.bodies, "nodes": self.meta["nodes"], "header": HEADER,
                "vmax": self.vmax, "drives": self.drives, "feet": len(self._feet),
                "torque_stall": self.tau, "torque_rated": self.rated,
                "steering": self.steering(),
                "forward": self.steering().get("forward", {}),
                "kinematic_stride_mm": self.kinematic()["stride"],
                "mass": self.meta["mass"]["total"], "payload_g": self.params.payload_g,
                "phase_lock": {"kp": self.lock.kp, "ki": self.lock.ki,
                               "servo_mismatch": self.lock.mismatch,
                               "step_deg": math.degrees(self.lock.max_offset)
                               if math.isfinite(self.lock.max_offset) else None},
                "timestep": self.model.opt.timestep, "crank_sign": self.meta["crank_sign"],
                "rest_height": self.meta["rest_height"],
                "design": {"linkage": cfg.linkage, "module": cfg.module, "key": cfg.key}}

    def _floor_contacts(self) -> np.ndarray:
        """The geoms on the floor now."""
        d = self.data
        g1, g2 = d.contact.geom1[:d.ncon], d.contact.geom2[:d.ncon]
        on_floor = (g1 == self._floor) | (g2 == self._floor)
        return np.where(g1 == self._floor, g2, g1)[on_floor]

    def feet_down(self) -> int:
        """How many feet touch the floor now."""
        return int(np.count_nonzero(np.isin(self._floor_contacts(), self._feet)))

    def body_down(self) -> bool:
        """Does a body that carries no foot touch the floor now (the frame, a link's
        capsule away from the feet)?"""
        other = self._floor_contacts()
        return bool(np.any(~np.isin(self.model.geom_bodyid[other], self._foot_bodies)))

    def frame(self) -> bytes:
        with self._lock:
            d = self.data
            base = self._index.base
            out = np.empty(HEADER + 12 * len(self.bodies), dtype="<f4")
            out[0] = d.time
            out[1:4] = d.xpos[base] * 1e3
            out[4:8] = d.xquat[base]
            out[8:10] = d.actuator_length[self._act]
            out[10] = self.feet_down()
            out[11:13] = d.actuator_force[self._act]
            out[13] = self.body_down()
            out[14] = d.actuator_length[self._act[0]] - d.actuator_length[self._act[-1]]
            out[15] = d.qacc[self._base_dof + 2]
            out[16] = loop_forces(d, self._eq).max() if self._eq.size else 0.0
            out[HEADER:] = motions(d, self._index).ravel()
            return out.tobytes()

    def upright(self) -> bool:
        """Is the base's up axis within 45 degrees of vertical?"""
        r = self.data.xmat[self._index.base].reshape(3, 3)
        return r[2, 1] > math.cos(math.pi / 4)
