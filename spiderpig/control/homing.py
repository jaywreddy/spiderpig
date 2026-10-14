"""How each joint finds its zero, and the phase that makes homing repeatable to the step.

* **The turntable** (``q1``) by its preset's sensor (:data:`construction.rotary.SENSORS`):
  an optical endstop and half-circle flag (:mod:`construction.rotary.turret`): the flag's
  edge is in the beam at yaw 0 and the flag covers the negative half, so the beam's state
  says which way home is (blocked: turn positive until it clears; clear: turn negative
  until it blocks), then back off and come in again slowly from the same side: the edge,
  from one side only, every time. Or ``hard_stop``: no sensor, the turntable's screw
  stalls the driver on a post past the range's lower end (Klipper's sensorless homing:
  the TMC2209's StallGuard on its DIAG pin, at a reduced current), coarse (two full steps):
  the camera's fiducials (``homing_reference`` ``vision_fiducial``) set the fine zero.
* **A stall** (``angle_sensor`` ``stall`` on a planar input, the turntable's
  ``hard_stop``: :func:`method`): the joint is driven at a reduced current toward its
  range's lower end until its driver stalls on the hard stop there; the stop's angle is
  nominally :data:`STALL_PAST_DEG` past the range's end (its place found once by
  calibration), then it backs off into the range at its run current. No phase snap: the
  stall wanders by more than half a step.
* **The planar inputs** (an arm's ``q2``, ``q3``: the palletizer's upper arm's and
  forearm's angles from horizontal, :mod:`linkages.palletizer`) by an angle gauge: with no
  switch on them, the motors are energised (a rotor jumps up to two full steps to its
  driver's phase when they are: the reading must come after), the arm is jogged to a pose
  with the upper arm under 90 deg (a gauge reads 0..90), the gauge zeroed on the base reads
  the upper arm (``q2``) and the forearm (``q3``: it keeps that direction), and those
  readings set the joints' zeros at the motors' counts there (:func:`set_from_gauge`). A
  gauge's 0.05-0.1 deg is the zero's error until :mod:`.calibrate` refines it; an
  inclinometer on each link (:mod:`.incline`, the role ``angle_sensor``) reads the same
  angles by itself, to about 0.02 deg.
* **The phase** (:func:`phase_move`): a switch's trigger wanders by a fraction of a step
  from homing to homing; the motor's driver counts its microstep phase (a TMC2209's
  ``MSCNT``, 1024 a turn of four full steps). The first homing records the phase at the
  trigger (the reference); every later one moves from the trigger to the nearest place with
  that phase and calls that home, so homing repeats to the step while the trigger wanders
  under half a step (Klipper's ``endstop_phase`` for its own axes, the same idea).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from spiderpig.control.mapping import Axis

MSCNT_STEP = 256        # a TMC2209's microstep counter per full step (1024 an electrical turn)
STALL_PAST_DEG = 3.0    # a hard stop past its range's lower end (nominal: the turret's post,
#                         construction.rotary.base_pair.stop_radius; calibration finds it)
YAW_SENSOR = "optical_flag"     # the turntable's sensor when none is named (the classic's)
ENDSTOPS = ("optical_flag", "ee_sx67")   # the turntable's switches (construction.rotary)
STALLS = ("stall", "hard_stop")          # a planar input's, the turntable's: no sensor


def method(sensor: str) -> str:
    """How a joint with sensor ``sensor`` (an ``angle_sensor``, :mod:`actuators.sensors`,
    or the turntable preset's, :data:`construction.rotary.SENSORS`) homes in Klipper:
    ``"endstop"`` (a switch on an endstop input), ``"stall"`` (its driver's StallGuard on a
    hard stop) or ``""`` (no homing move: a gauge, an inclinometer, none)."""
    if sensor in STALLS:
        return "stall"
    if sensor in ENDSTOPS or sensor.startswith("endstop"):
        return "endstop"
    return ""


def sensors_of(config, axes) -> dict[str, str]:
    """Each joint's homing sensor in the robot's ``config``: the turntable's its preset's
    (role ``turntable``), each planar input's its ``angle_sensor`` (the wrist's none)."""
    out = {n: config.role("angle_sensor", n) for n in axes if n not in ("q1", "q4")}
    if "q1" in axes:
        from spiderpig.construction.rotary import PRESETS

        out["q1"] = PRESETS[config.role("turntable")].sensor
    return out


@dataclass(frozen=True)
class HomingStep:
    """One thing the operator or the host does."""

    axis: str
    action: str


def plan(axes: dict[str, Axis], sensors: dict[str, str] | None = None,
         phase: bool = True) -> list[HomingStep]:
    """The homing sequence of ``axes`` (module docstring). ``sensors``: each planar
    input's angle sensor (:mod:`actuators.sensors`; ``gauge`` when not named) and the
    turntable's (``q1``: its preset's, :data:`YAW_SENSOR` when not named; :func:`sensors_of`
    reads a config's); ``phase``:
    the turntable's homing snaps to its driver's phase (a toothed drive: a belt's; not a
    capstan's, whose cord may slip against the motor, which the snap would hide)."""
    sensors = sensors or {}
    out: list[HomingStep] = []
    yaw = method(sensors.get("q1", YAW_SENSOR))
    if "q1" in axes and yaw == "stall":
        out += _stall_steps("q1", "the turntable's screw on the post")
        out.append(HomingStep("q1", "the camera's fiducials set the fine yaw zero "
                                    "(homing_reference vision_fiducial)"))
    elif "q1" in axes and yaw == "endstop":
        out += [HomingStep("q1", "read the endstop: blocked, turn positive until it clears; "
                                 "clear, turn negative until it blocks"),
                HomingStep("q1", "back off 2 deg, then approach again at a tenth of the speed "
                                 "from the same side until the beam changes"),
                *([HomingStep("q1", "read the driver's phase (DUMP_TMC STEPPER=q1 "
                                    "REGISTER=MSCNT): the first time, record it as the "
                                    "reference; after, move to the nearest place with the "
                                    "reference phase")] if phase else []),
                HomingStep("q1", "that is yaw 0")]
    planar = [n for n in axes if n not in ("q1", "q4")]
    gauged = [n for n in planar if sensors.get(n, "gauge") == "gauge"]
    switched = [n for n in planar if method(sensors.get(n, "")) == "endstop"]
    stalled = [n for n in planar if method(sensors.get(n, "")) == "stall"]
    sensed = [n for n in planar if n not in gauged + switched + stalled]
    for n in stalled:
        out += _stall_steps(n, "the link on its hard stop")
    for n in switched:      # one joint at a time, each to its switch (the MT4's, GRBL's)
        out += [HomingStep(n, "turn toward the end of travel until the flag breaks the beam"),
                HomingStep(n, "back off 3 deg, then approach again at a tenth of the speed "
                              "from the same side until the beam breaks"),
                HomingStep(n, "that is the joint's stored switch angle (its offset, found "
                              "once by calibration)")]
    if gauged:
        out.append(HomingStep(",".join(gauged), "energise the motors (they hold), jog the arm "
                              "to a pose with the upper arm under 90 deg; zero a digital "
                              "angle gauge on the base, read it on the upper arm (q2) and on "
                              "the forearm (q3); enter each with the motor's count there"))
    if sensed:
        out.append(HomingStep(",".join(sensed), "energise the motors (they hold), let the arm "
                              "settle; read each link's inclinometer for half a second "
                              "(spiderpig.control.incline, its assembly offset taken off) and "
                              "set the joint's zero at the motor's count there"))
    if "q4" in axes:
        out.append(HomingStep("q4", "jog the cup to its mark: that is roll 0"))
    return out


def _stall_steps(n: str, what: str) -> list[HomingStep]:
    return [HomingStep(n, f"lower the driver's current, turn negative toward the range's end "
                          f"until the driver stalls ({what}: Klipper's "
                          f"tmc2209_{n}:virtual_endstop)"),
            HomingStep(n, f"that is the stop's angle ({STALL_PAST_DEG:g} deg past the range's "
                          "end, nominally: found once by calibration); restore the run "
                          "current and back off to the range's end")]


def phase_move(mscnt: int, reference: int, microsteps: int, sign: int = 1) -> int:
    """The microsteps to move from where the trigger left the motor (its driver's counter
    ``mscnt``) to the nearest place whose phase is the ``reference``'s (module docstring).
    ``sign``: +1 when a positive move raises the counter (a jog of one full step shows which:
    flipping a driver's DIR pin flips it)."""
    per = MSCNT_STEP / microsteps                # the counter's counts per microstep
    d = (mscnt - reference + MSCNT_STEP / 2) % MSCNT_STEP - MSCNT_STEP / 2
    return -sign * round(d / per)


def set_from_gauge(axis: Axis, reading_deg: float, steps: int = 0) -> Axis:
    """``axis`` with its zero set so that its motor's count ``steps`` is the joint's angle
    the gauge reads (degrees: the link's angle from horizontal)."""
    q = math.radians(reading_deg)
    return replace(axis, zero=q - axis.direction * steps / axis.per_rad)
