"""A host driver for an arm on Klipper (:mod:`.klipper`): straight moves as synchronised
joint segments, homing, the vacuum, streamed to Klipper through Moonraker.

**A straight move** (:func:`line`) from one tip position to another (the base frame, mm) is
cut into segments of at most ``seg_mm``; each segment's end is solved
(:meth:`.arm.ArmModel.ik`) and every joint moves there over the same time (each its own
speed, ``SYNC=0`` so they run together, the slowest's ``SYNC=1`` so the next waits): the
tip follows the line to within the segments' chord. A joint's position in Klipper is its
motor's, in joint degrees from step 0 (:class:`.mapping.Axis`: its direction applied).

**Homing** (:func:`home_gcode`) runs the turntable onto its flag's edge from the positive
side (:mod:`.homing`), sets that as 0, then each joint's motor is snapped to its nearest
full step by its driver's microstep counter (a TMC2209's ``MSCNT``, 0..1023: a full step
where one coil carries the full current, every 256: :func:`mscnt_snap`; the counter is read
with ``DUMP_TMC``).

:class:`Moonraker` posts G-code to Moonraker's ``/printer/gcode/script`` (the standard
library only); nothing here talks to hardware until it is called with a URL.
"""

from __future__ import annotations

import json
import math
import urllib.parse
import urllib.request
from dataclasses import dataclass

import numpy as np

from spiderpig.control.arm import ArmModel
from spiderpig.control.mapping import Axis

MSCNT_STEP = 256        # a TMC2209's microstep counter per full step (1024 per electrical turn)


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
                   f"SPEED={speed:.5f} SYNC={sync}")
    return out


def line(model: ArmModel, axes: dict[str, Axis], start_q: dict[str, float], b, *,
         seg_mm: float = 2.0, speed_mm_s: float = 20.0) -> tuple[list[str], dict[str, float]]:
    """The G-code of a straight move of the tip from where ``start_q`` puts it to ``b``
    (base frame, mm), and the joints at its end (module docstring)."""
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


def mscnt_snap(mscnt: int, microsteps: int) -> int:
    """The microsteps to move from a driver's counter ``mscnt`` to the nearest full step
    (one coil at full current: a multiple of :data:`MSCNT_STEP`), positive raising it."""
    per = MSCNT_STEP / microsteps          # the counter's steps per microstep
    near = round(mscnt / MSCNT_STEP) * MSCNT_STEP
    return round((near - mscnt) / per)


def home_gcode(axes: dict[str, Axis]) -> list[str]:
    """The turntable's homing (module docstring): onto the flag's edge from the positive
    side, then 0 there. The planar joints' zeros are the angle gauge's (:mod:`.homing`)."""
    if "q1" not in axes:
        return []
    ax = axes["q1"]
    span = math.degrees(ax.limits[1] - ax.limits[0])   # never past the range (the cables)
    step = 2.0
    slow = ax.deg_per_motor_rev * 300.0 / 60.0 / 10.0
    return [
        "MANUAL_STEPPER STEPPER=q1 ENABLE=1",
        "MANUAL_STEPPER STEPPER=q1 SET_POSITION=0",
        # blocked: off the flag toward positive; then toward negative onto its edge
        f"MANUAL_STEPPER STEPPER=q1 MOVE={ax.direction * span:.1f} STOP_ON_ENDSTOP=-1",
        "MANUAL_STEPPER STEPPER=q1 SET_POSITION=0",
        f"MANUAL_STEPPER STEPPER=q1 MOVE={-ax.direction * span:.1f} STOP_ON_ENDSTOP=1",
        "MANUAL_STEPPER STEPPER=q1 SET_POSITION=0",
        f"MANUAL_STEPPER STEPPER=q1 MOVE={ax.direction * step:.1f}",
        f"MANUAL_STEPPER STEPPER=q1 MOVE={-ax.direction * 2 * step:.1f} SPEED={slow:.3f} "
        "STOP_ON_ENDSTOP=1",
        "MANUAL_STEPPER STEPPER=q1 SET_POSITION=0",
        "DUMP_TMC STEPPER='manual_stepper q1' REGISTER=MSCNT",
    ]


@dataclass
class Moonraker:
    """Moonraker's HTTP API at ``url`` (``http://host:7125``)."""

    url: str
    timeout: float = 60.0

    def script(self, lines: list[str]) -> dict:
        body = urllib.parse.urlencode({"script": "\n".join(lines)}).encode()
        req = urllib.request.Request(f"{self.url.rstrip('/')}/printer/gcode/script",
                                     data=body, method="POST")
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read().decode() or "{}")


@dataclass
class ArmHost:
    """The arm driven through Moonraker: its model, its axes, where its joints are."""

    model: ArmModel
    axes: dict[str, Axis]
    client: Moonraker
    q: dict[str, float]

    def move_to(self, xyz, **kw) -> None:
        lines, self.q = line(self.model, self.axes, self.q, xyz, **kw)
        self.client.script(lines)

    def vacuum(self, on: bool) -> None:
        self.client.script([f"SET_PIN PIN=vacuum_pump VALUE={1 if on else 0}",
                            f"SET_PIN PIN=vacuum_release VALUE={0 if on else 1}"])

    def home(self) -> None:
        self.client.script(home_gcode(self.axes))
        self.q["q1"] = 0.0
