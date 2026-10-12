"""How each joint finds its zero, and the snap to its motor's nearest full step.

* **The turntable** (``q1``) by its optical endstop and half-circle flag
  (:mod:`construction.rotary.turret`): the flag's edge is in the beam at yaw 0 and the flag
  covers the negative half, so the beam's state says which way home is (blocked: turn
  positive until it clears; clear: turn negative until it blocks), then back off and come
  in again slowly from the same side: the edge, from one side only, every time.
* **The planar inputs** (an arm's ``q2``, ``q3``: the palletizer's upper arm's and
  forearm's angles from horizontal, :mod:`linkages.palletizer`) by an angle gauge: with no
  switch on them, the arm is set to a pose by hand, a digital angle gauge on each driven
  link reads its angle, and those readings are the joints' angles there
  (:func:`set_from_gauge`). A gauge's 0.05-0.1 deg is the zero's error until
  :mod:`.calibrate` refines it from touched points.
* **The snap** (:func:`snap`): wherever a joint stops homing, it then moves to its motor's
  nearest full step and calls that its zero (a full step's place is exact, a microstep's
  off by up to :attr:`actuators.StepperSpec.microstep_accuracy` of a step), and the
  joint's angle there is known to the step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from spiderpig.control.mapping import Axis


@dataclass(frozen=True)
class HomingStep:
    """One thing the operator or the host does."""

    axis: str
    action: str


def plan(axes: dict[str, Axis]) -> list[HomingStep]:
    """The homing sequence of ``axes`` (module docstring): the turntable first (it needs
    the arm clear of nothing), then the gauge on the planar joints, then the snaps."""
    out: list[HomingStep] = []
    if "q1" in axes:
        out += [HomingStep("q1", "read the endstop: blocked, turn positive until it clears; "
                                 "clear, turn negative until it blocks"),
                HomingStep("q1", "back off 2 deg, then approach again at a tenth of the speed "
                                 "from the same side until the beam changes"),
                HomingStep("q1", "that is yaw 0")]
    planar = [n for n in axes if n not in ("q1", "q4")]
    if planar:
        out.append(HomingStep(",".join(planar), "set the arm to a comfortable pose by hand "
                              "(motors off), put a digital angle gauge on each driven link, "
                              "enter each reading as that joint's angle"))
    if "q4" in axes:
        out.append(HomingStep("q4", "turn the cup to its mark by hand: that is roll 0"))
    out.append(HomingStep(",".join(axes), "energise the motors and snap each to its nearest "
                                          "full step (the zero becomes that step)"))
    return out


def snap(axis: Axis, steps: int) -> tuple[int, float]:
    """Where a joint stopped homing (``steps``, the motor's count) to its nearest full step:
    ``(the move in microsteps, the joint's angle there in rad)``."""
    target = axis.full_step(steps)
    return target - steps, axis.angle(target)


def set_from_gauge(axis: Axis, reading_deg: float, steps: int = 0) -> Axis:
    """``axis`` with its zero set so that its motor's count ``steps`` is the joint's angle
    the gauge reads (degrees: the link's angle from horizontal)."""
    from dataclasses import replace

    q = math.radians(reading_deg)
    return replace(axis, zero=q - axis.direction * steps / axis.per_rad)
