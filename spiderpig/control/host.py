"""A host driver for an arm on Klipper (:mod:`.klipper`): straight moves as synchronised
joint segments, homing, the vacuum, streamed to Klipper through Moonraker.

**A straight move** (:func:`line`) from one tip position to another (the base frame, mm) is
cut into segments of at most ``seg_mm``; each segment's end is solved
(:meth:`.arm.ArmModel.ik`) and every joint moves there over the same time (each its own
speed, ``SYNC=0`` so they run together, the slowest's ``SYNC=1`` so the next waits): the
tip follows the line to within the segments' chord. A joint's position in Klipper is its
motor's, in joint degrees from step 0 (:class:`.mapping.Axis`: its direction applied).

Each segment moves at constant speed (``ACCEL=0``: Klipper's manual steppers have no
lookahead, so each move would stop at its end; at constant speed each joint takes exactly
the segment's time and the joints stay together); the tip's speed is kept low
(:data:`MAX_TIP_MM_S`), where a stepper starts and stops without ramps. (Klipper's newer
``MANUAL_STEPPER ... GCODE_AXIS`` gives coordinated moves with lookahead, where installed.)

**A one-sided final approach** (:func:`one_sided`, ``move_to(..., approach=True)``): a
joint whose last motion into the target was negative goes on past it by
:data:`APPROACH_DEG` and comes back positive, so every stop takes up its drive's backlash
(a belt's teeth, the turntable's, which no gravity holds) the same way: what the MT4 and
every belt-driven arm rely on, and what ISO 9283's same-direction cycle measures. Pick and
place approach the hover pose above the slide one-sidedly and then descend straight down
(the turntable still: its backlash stays taken up; the arm's joints seated by gravity).

**Homing** (:func:`home_gcode`) follows each joint's sensor (:func:`.homing.method`). A
stall joint (the turntable's ``hard_stop``, a planar input's ``stall``) lowers its
driver's current (:data:`.klipper.HOME_CURRENT`), turns toward its range's lower end no
farther than the whole range and the stop past it, at half a motor turn a second
(Klipper's sensorless homing guide: StallGuard wants speed), until the driver stalls on
the hard stop (its ``virtual_endstop``), sets that as the stop's angle
(:func:`stop_angle`), restores the run current, and backs off to the range's end.

:class:`Moonraker` posts G-code to Moonraker's ``/printer/gcode/script`` (the standard
library only); nothing here talks to hardware until it is called with a URL.
"""

from __future__ import annotations

import json
import math
import urllib.request
from dataclasses import dataclass

import numpy as np

from spiderpig.control import homing
from spiderpig.control.arm import ArmModel
from spiderpig.control.mapping import Axis

MAX_TIP_MM_S = 25.0     # the tip's speed a straight move is held to (constant-speed segments)
APPROACH_DEG = 0.5      # past the target before a one-sided final approach (joint degrees),
#                         at least: well over a GT2 5:1 drive's backlash (0.1 deg at the joint)
STALL_REV_S = 0.5       # a stall's homing speed: motor turns a second (Klipper's guide: half
#                         its rotation_distance a second)
STALL_SETTLE_MS = 2000  # a wait after a stall for StallGuard to clear (Klipper's guide)
APPROACH_X = 1.5        # ... and at least this many times the axis's own backlash (a cheap
#                         planetary's: 25' to 3 deg)


def position_deg(axis: Axis, q: float) -> float:
    """The joint's Klipper position (its motor's, in joint degrees from step 0)."""
    return axis.direction * math.degrees(q - axis.zero)


def joints_gcode(axes: dict[str, Axis], start: dict[str, float], end: dict[str, float],
                 seconds: float) -> list[str]:
    """One synchronised segment: every joint from ``start`` to ``end`` (rad) in
    ``seconds``, the slowest last with ``SYNC=1``."""
    moves = []
    for n, ax in axes.items():
        if n not in end:
            continue
        d = abs(position_deg(ax, end[n]) - position_deg(ax, start.get(n, end[n])))
        moves.append((d, n, ax))
    if not moves:
        return []
    moves.sort(key=lambda m: m[0])
    out = []
    for i, (d, n, ax) in enumerate(moves):
        speed = max(d / max(seconds, 1e-6), 1e-3)
        sync = 1 if i == len(moves) - 1 else 0
        out.append(f"MANUAL_STEPPER STEPPER={n} MOVE={position_deg(ax, end[n]):.5f} "
                   f"SPEED={speed:.5f} ACCEL=0 SYNC={sync}")
    return out


def line(model: ArmModel, axes: dict[str, Axis], start_q: dict[str, float], b, *,
         seg_mm: float = 5.0, speed_mm_s: float = 20.0) -> tuple[list[str], dict[str, float]]:
    """The G-code of a straight move of the tip from where ``start_q`` puts it to ``b``
    (base frame, mm), and the joints at its end (module docstring)."""
    speed_mm_s = min(speed_mm_s, MAX_TIP_MM_S)
    a = model.fk(start_q)
    b = np.asarray(b, dtype=float)
    dist = float(np.linalg.norm(b - a))
    n = max(1, math.ceil(dist / seg_mm))
    out: list[str] = []
    q = dict(start_q)
    for k in range(1, n + 1):
        nxt = model.ik(a + (b - a) * k / n)
        out += joints_gcode(axes, q, nxt, dist / n / speed_mm_s)
        q = nxt
    return out, q


def one_sided(axes: dict[str, Axis], before: dict[str, float], end: dict[str, float],
              seconds: float = 0.5) -> list[str]:
    """The G-code that ends a move one-sidedly (module docstring): each joint whose motion
    from ``before`` (rad: where its last segment started) to ``end`` was negative goes past
    ``end`` (:data:`APPROACH_DEG`, or :data:`APPROACH_X` times its backlash if more), then
    back to it, positive."""
    past = {}
    for n, q in end.items():
        if n in axes and q < before.get(n, q) - 1e-12:
            over = max(APPROACH_DEG, APPROACH_X * axes[n].backlash_deg)
            past[n] = q - math.radians(over)
    if not past:
        return []
    back = {n: end[n] for n in past}           # (only the joints that go past move)
    return joints_gcode(axes, end, past, seconds) + joints_gcode(axes, past, back, seconds)


def stop_angle(axis: Axis) -> float:
    """A stall joint's hard stop (rad): :data:`homing.STALL_PAST_DEG` past its range's
    lower end (nominal: calibration finds it)."""
    return axis.limits[0] - math.radians(homing.STALL_PAST_DEG)


def home_gcode(axes: dict[str, Axis], sensors: dict[str, str] | None = None) -> list[str]:
    """The homing moves (module docstring), the turntable first: ``sensors`` each joint's
    (:func:`homing.sensors_of`; by default the turntable's hard stop alone). A joint with
    no homing move here (a gauge, an inclinometer, a planar switch: :mod:`.homing`) has
    none."""
    sensors = {"q1": homing.YAW_SENSOR} if sensors is None else sensors
    out: list[str] = []
    for n in sorted(axes, key=lambda n: n != "q1"):
        how = homing.method(sensors.get(n, ""))
        if how == "stall":
            out += _stall_gcode(n, axes[n])
    return out


def homed(axes: dict[str, Axis], sensors: dict[str, str] | None = None) -> dict[str, float]:
    """Where :func:`home_gcode` leaves the joints it homes (rad)."""
    sensors = {"q1": homing.YAW_SENSOR} if sensors is None else sensors
    out = {}
    for n, ax in axes.items():
        how = homing.method(sensors.get(n, ""))
        if how == "stall":
            out[n] = ax.limits[0]
    return out


def _stall_gcode(n: str, ax: Axis) -> list[str]:
    """Joint ``n`` onto its hard stop by its driver's stall (module docstring)."""
    from spiderpig.control.klipper import currents

    amps = currents(ax)
    if amps is None:
        raise ValueError(f"{n}: a stall homing needs a catalogued motor (its current), "
                         f"not {ax.motor!r}")
    run, _, home = amps
    lo, hi = ax.limits
    stop = stop_angle(ax)
    speed = STALL_REV_S * ax.deg_per_motor_rev                 # joint deg/s
    seek = stop - math.radians(5.0)          # wherever it is, the stop is under hi - seek
    return [f"MANUAL_STEPPER STEPPER={n} ENABLE=1",
            f"SET_TMC_CURRENT STEPPER={n} CURRENT={home:.2f}",
            f"MANUAL_STEPPER STEPPER={n} SET_POSITION={position_deg(ax, hi):.4f}",
            f"MANUAL_STEPPER STEPPER={n} MOVE={position_deg(ax, seek):.4f} SPEED={speed:.3f} "
            "STOP_ON_ENDSTOP=1",
            f"MANUAL_STEPPER STEPPER={n} SET_POSITION={position_deg(ax, stop):.4f}",
            f"G4 P{STALL_SETTLE_MS}",
            f"SET_TMC_CURRENT STEPPER={n} CURRENT={run:.2f}",
            f"MANUAL_STEPPER STEPPER={n} MOVE={position_deg(ax, lo):.4f} SPEED={speed:.3f}"]


@dataclass
class Moonraker:
    """Moonraker's HTTP API at ``url`` (``http://host:7125``)."""

    url: str
    timeout: float = 60.0

    def script(self, lines: list[str]) -> dict:
        body = json.dumps({"script": "\n".join(lines)}).encode()
        req = urllib.request.Request(f"{self.url.rstrip('/')}/printer/gcode/script",
                                     data=body, method="POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read().decode() or "{}")


@dataclass
class ArmHost:
    """The arm driven through Moonraker: its model, its axes, where its joints are."""

    model: ArmModel
    axes: dict[str, Axis]
    client: Moonraker
    q: dict[str, float]
    sensors: dict[str, str] | None = None
    """Each joint's homing sensor (:func:`homing.sensors_of`; ``None``: the turntable's
    hard stop alone)."""

    def move_to(self, xyz, *, approach: bool = False, **kw) -> None:
        """A straight move to ``xyz``; ``approach``: ending one-sidedly (module docstring:
        the hover pose before a pick or a place)."""
        lines, q = line(self.model, self.axes, self.q, xyz, **kw)
        if approach:
            before = _last_start(self.model, self.q, xyz, kw.get("seg_mm", 5.0))
            lines += one_sided(self.axes, before, q)
        self.client.script(lines)
        self.q = q                 # only once Klipper has taken the move

    def vacuum(self, on: bool) -> None:
        self.client.script([f"SET_PIN PIN=vacuum_pump VALUE={1 if on else 0}",
                            f"SET_PIN PIN=vacuum_release VALUE={0 if on else 1}"])

    def home(self) -> None:
        self.client.script(home_gcode(self.axes, self.sensors))
        self.q.update(homed(self.axes, self.sensors))


def _last_start(model: ArmModel, start_q: dict[str, float], b, seg_mm: float) -> dict[str, float]:
    """The joints at the start of a straight move's last segment (as :func:`line` cuts
    it)."""
    a = model.fk(start_q)
    b = np.asarray(b, dtype=float)
    n = max(1, math.ceil(float(np.linalg.norm(b - a)) / seg_mm))
    return start_q if n == 1 else model.ik(a + (b - a) * (n - 1) / n)
