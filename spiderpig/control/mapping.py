"""Each joint's angle to its motor's steps: through its transmission's exact ratio.

A joint turned ``dq`` (rad) turns its motor ``dq x ratio``; the motor counts
``steps_per_rev x microsteps`` per turn (:class:`Axis`). Its sense (``direction``) is the
design's: a motor's positive turn is counter-clockwise seen from its shaft's end; a joint's
positive turn is about its axis (+z across the side's stack; +z up, the turntable's and the
wrist's). So a motor whose shaft points along the joint's axis turns with it (+1), one
pointing against it the other way (-1): the planar drives on the outer frame plate point
their shafts into the stack (+z), those on the inner plate out of it (-z); the turntable's
points up, the wrist's down. A belt keeps the sense, a capstan's cord reverses it (the drum
turns against the capstan). Check each on the machine once (a jog), and flip a driver's DIR
where it disagrees: a board's pins decide the rest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Axis:
    """One motor-driven joint (module docstring). ``zero``: the joint's design angle (rad)
    at the motor's step 0 (where homing leaves it)."""

    name: str
    ratio: float
    step_deg: float
    microsteps: int
    direction: int = 1
    zero: float = 0.0
    motor: str = ""
    limits: tuple[float, float] = (-math.pi, math.pi)     # the joint's (rad, design angles)
    backlash_deg: float = 0.0      # its drive's at the joint (a one-sided approach's overshoot)

    @property
    def steps_per_rev(self) -> int:
        return round(360.0 / self.step_deg)

    @property
    def per_rad(self) -> float:
        """Microsteps per radian of the joint."""
        return self.ratio * self.steps_per_rev * self.microsteps / (2 * math.pi)

    @property
    def deg_per_motor_rev(self) -> float:
        """The joint's degrees per turn of its motor (Klipper's ``rotation_distance``)."""
        return 360.0 / self.ratio

    def steps(self, q: float) -> int:
        """The motor's microstep count at the joint's angle ``q`` (rad)."""
        if not self.limits[0] - 1e-9 <= q <= self.limits[1] + 1e-9:
            raise ValueError(f"{self.name} at {math.degrees(q):.2f} deg is outside "
                             f"{math.degrees(self.limits[0]):.1f}.."
                             f"{math.degrees(self.limits[1]):.1f}")
        return self.direction * round((q - self.zero) * self.per_rad)

    def angle(self, steps: int) -> float:
        """The joint's angle (rad) at the motor's microstep count."""
        return self.zero + self.direction * steps / self.per_rad


def axes(config, design=None) -> dict[str, Axis]:
    """Every motor-driven joint of ``config``'s arm: its planar drives (their ratio, motor
    and plate: :mod:`construction.transmissions`), the turntable's and the wrist's when it
    builds the robot. ``zero``: each joint's reference (0 for the turntable and wrist; the
    planar inputs' design angles are absolute, the homing sets them)."""
    from spiderpig import actuators
    from spiderpig.construction.transmissions import DriveGroup
    from spiderpig.fabricate import design_side, template_for

    side = config.side_config()
    if design is None:
        design = design_side(template_for(side), side)
    lk = config.lk
    out: dict[str, Axis] = {}
    for g in design.groups:
        if not isinstance(g, DriveGroup):
            continue
        hub = g.hub(design.ctx)
        sense = (1 if hub.down else -1) * (-1 if g.transmission.reverses else 1)
        lo, hi = lk.ranges[g.input]
        out[g.input] = Axis(g.input, hub.ratio, g.motor.step_deg, g.motor.microsteps, sense,
                            0.0, g.motor.key, (math.radians(lo), math.radians(hi)),
                            g.error_terms(design.ctx).backlash_deg)
    if config.layout == "turret":
        from spiderpig.construction.robot import Turret

        tp, tool = Turret().plans(design, config)
        _, tr, motor, _ = tp.preset.parts
        sense = -1 if tr.reverses else 1
        lo, hi = (math.radians(v) for v in tp.range_deg)
        out["q1"] = Axis("q1", tr.ratio(None), motor.step_deg, motor.microsteps, sense, 0.0,
                         motor.key, (lo, hi), tp.error_terms().backlash_deg)
        if tool is not None and tool.tool.wrist:
            m = actuators.get(tool.tool.wrist)
            out["q4"] = Axis("q4", 1.0, m.step_deg, m.microsteps, -1, 0.0, m.key)
    return out
