"""Homing sensors: how every joint of an arm finds its zero, the turntable's included
(2026-10-12, round 3 of the arm's precision review; one registry for every joint since
2026-10-14).

One registry, :data:`SENSORS`, for every joint's homing: a planar input's through the role
``angle_sensor`` (one per drive, :mod:`spiderpig.roles`), the turntable's (``q1``) through
its preset (:attr:`construction.rotary.RotaryPreset.sensor`). Each sensor declares how it
homes (:attr:`HomingSensor.method`, what :mod:`control.homing`, the Klipper config and the
host follow), whether it reads its angle against gravity (:attr:`HomingSensor.gravity`: no
use about a vertical axis) and what it buys; each joint's construction builds the hardware
the method needs at that joint (a stall's hard stop: a planar drive's screw and nut,
:data:`construction.transmissions.base.HARD_STOP`; the turret's post and screw,
:mod:`construction.rotary.base_pair`).

* ``gauge``: a digital angle gauge laid on each link by hand, zeroed on the base: 0.05 deg
  on a good one (:attr:`HomingSensor.repeat_deg`), the zero's error until a calibration
  refines it, lost again at every power-up;
* ``endstop``: what the WLKATA MT4 does (its STEP names an optical mounting plate on the
  base and a "J2 detection flag" on the upper arm; Thor, PAROL6, the AR4 and the Mirobot
  home every joint to a switch too): a slotted optical endstop on the frame, a printed flag
  on the link :data:`FLAG_R` from the pivot at the end of its travel; homed by a fast seek,
  a back-off and a slow approach from the same side, every time. A slot switch repeats to
  about 20 um at its flag's edge (:data:`SWITCH_REPEAT_MM`): that over the flag's radius is
  the joint's (a belt-driven joint could then snap to its driver's phase, Klipper's
  ``endstop_phase``, step-exact; not counted);
* ``stall``: no sensor: the joint's driver (a TMC2209) is run at a reduced current against
  a hard stop past its range's end until StallGuard trips (Klipper's sensorless homing).
  Where it trips is set at the motor, so its repeatability is a number of the motor's full
  steps (:data:`STALL_FULL_STEPS`) through the joint's own reduction
  (:meth:`HomingSensor.at_joint`): 2 x 1.8 / 5 = 0.72 deg on a belt drive's 5:1, 0.96 deg on
  the turntable's 3.75:1;
* ``scl3300``: a Murata SCL3300-D01 MEMS inclinometer on the link (the upper arm reads
  ``q2``; the forearm, which keeps ``q3``'s direction, ``q3``), its in-plane tilt
  ``atan2`` of its two axes in the link's plane, averaged half a second at rest
  (0.001 deg/sqrt(Hz) noise), its offset set once at assembly against the gauge and
  drifting 0.005 deg/K: :attr:`HomingSensor.repeat_deg` over a few kelvin. Read at rest
  before each final approach it also takes out what the drive's sag leaves
  (:mod:`control.incline`). The Dobot Magician reads its arms' angles
  from accelerometers in the links the same way.

:meth:`HomingSensor.at_joint` is what homing leaves at the joint each time (a
repeatability term: the budget's ``homing_mm``, :func:`precision.drive_budgets` and
:func:`precision.turret_budget`). A camera homing (:data:`REFERENCES` ``vision_fiducial``)
sets every zero from a reference sheet instead: the sensors then only find the joints
coarsely, and their repeatability leaves the budget.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

SWITCH_REPEAT_MM = 0.02    # a slotted optical switch's repeatability at its flag's edge (a
#                            typical figure, unverified)
FLAG_R = 40.0              # the flag's edge from the joint's pivot, on the link (mm)
PRECISE_REPEAT_MM = 0.005  # an Omron EE-SX67's, budgeted: its datasheet's 2 um over 20
#                            tries at 25 C and 20-40 um of drift over 25 C +-30 K
PRECISE_FLAG_R = 45.0      # a tab on the link over the frame plate (the review's: q2 45, q3 55)
STALL_FULL_STEPS = 2.0
"""Where a stall homing trips, either way, in the motor's full steps: the rotor lags its
field by up to a full step (90 electrical degrees, the pull-out angle) before it slips, and
a TMC2209 updates its StallGuard result once a full step (its datasheet's ``SG_RESULT``),
so Klipper's sensorless homing stops within about two full steps of the same place (an
estimate, unmeasured). The planar inputs' fixed 0.72 deg assumed it at 5:1; the
turntable's 0.75 mm at its 26 mm stop (1.65 deg at the joint) assumed nothing of its own
ratio, and its 3.75:1 makes 0.96 deg."""
METHODS = ("endstop", "stall", "")
"""How a sensor homes in Klipper (:attr:`HomingSensor.method`): a switch on an endstop
input, the driver's stall on a hard stop, or no homing move (read by hand or by the host)."""


@dataclass(frozen=True)
class HomingSensor:
    """How a joint's zero is found (module docstring): ``method`` how it homes
    (:data:`METHODS`); ``repeat_deg`` what each homing leaves at the joint, or for a stall
    ``full_steps`` at the motor (:meth:`at_joint`); ``gravity``: it reads the link's angle
    against gravity (no use about a vertical axis); ``bom_key`` the part bought per joint
    (``""``: none); ``source``: where the numbers come from."""

    key: str
    label: str
    method: str = ""
    repeat_deg: float = 0.0
    full_steps: float = 0.0
    gravity: bool = False
    bom_key: str = ""
    source: str = ""

    def __post_init__(self) -> None:
        if self.method not in METHODS:
            raise ValueError(f"sensor {self.key!r}: unknown homing method {self.method!r}")
        if (self.full_steps > 0) == (self.repeat_deg > 0):
            raise ValueError(f"sensor {self.key!r}: give its repeatability at the joint "
                             "(repeat_deg) or at the motor (full_steps), one of them")

    @property
    def needs_stop(self) -> bool:
        """It homes on a hard stop (a stall): the joint's construction builds one."""
        return self.method == "stall"

    def at_joint(self, motor: Any = None, ratio: float = 1.0) -> float:
        """What each homing leaves at the joint (deg): ``repeat_deg``, or ``full_steps`` of
        ``motor``'s (:class:`actuators.StepperSpec`) through the joint's reduction
        ``ratio``."""
        if not self.full_steps:
            return self.repeat_deg
        if motor is None:
            raise ValueError(f"sensor {self.key!r}: its repeatability is the motor's steps; "
                             "give the joint's motor and ratio")
        return self.full_steps * motor.step_deg / ratio


SENSORS: dict[str, HomingSensor] = {s.key: s for s in (
    HomingSensor("gauge", "a digital angle gauge on each link, by hand", repeat_deg=0.05,
                 gravity=True,
                 source="a 0.05 deg digital angle gauge (0.05-0.1 deg in use: the better)"),
    HomingSensor("endstop", "a slotted optical endstop on the frame, a flag on the link",
                 "endstop", round(math.degrees(SWITCH_REPEAT_MM / FLAG_R), 4),
                 bom_key="optical_endstop",
                 source=f"{SWITCH_REPEAT_MM * 1000:g} um at a flag {FLAG_R:g} mm out "
                        "(unverified); the MT4's way"),
    HomingSensor("endstop_ee_sx67", "an Omron EE-SX67 slot sensor on the frame, an "
                 "aluminium flag on the link", "endstop",
                 round(math.degrees(PRECISE_REPEAT_MM / PRECISE_FLAG_R), 4),
                 bom_key="omron_ee_sx670",
                 source=f"{PRECISE_REPEAT_MM * 1000:g} um (its datasheet's 2 um over 20 "
                        f"tries at 25 C, and drift) at a tab {PRECISE_FLAG_R:g} mm out (q2's "
                        "on b1; q3's on the crank b2 about 55)"),
    HomingSensor("stall", "no sensor: the driver's stall detection (TMC2209 StallGuard) "
                 "against a hard stop past the end of the joint's travel", "stall",
                 full_steps=STALL_FULL_STEPS,
                 source=f"{STALL_FULL_STEPS:g} full steps at the motor (Klipper's sensorless "
                        "homing, at reduced current) through the joint's reduction; only "
                        "coarse: a camera homing (vision_fiducial) sets every zero, so it "
                        "leaves the budget (the cost loop's, 2026-10-13: the endstops "
                        "dropped)"),
    HomingSensor("scl3300", "a Murata SCL3300-D01 inclinometer on each link", repeat_deg=0.02,
                 gravity=True, bom_key="inclinometer_scl3300",
                 source="Murata SCL3300-D01: 0.0055 deg/LSB, 0.001 deg/sqrt(Hz) noise, "
                        "offset drift 0.005 deg/K (a few K: 0.02 deg); estimate"),
)}


def get(key: str) -> HomingSensor:
    try:
        return SENSORS[key]
    except KeyError:
        raise KeyError(f"unknown homing sensor {key!r}; have {sorted(SENSORS)}") from None


@dataclass(frozen=True)
class Reference:
    """What the joints' fine zero is taken from (the role ``homing_reference``): ``tool_mm``
    the tip's place each homing leaves (``None``: each axis's own sensor); ``bom``: what it
    buys, ``(catalog key, quantity)``."""

    key: str
    label: str
    tool_mm: float | None = None
    bom: tuple[tuple[str, float], ...] = ()
    source: str = ""


REFERENCES: dict[str, Reference] = {r.key: r for r in (
    Reference("switches", "each joint's own switch or gauge (angle_sensor)"),
    Reference("vision_fiducial", "a camera on the tool sighting dots on a reference board: "
              "every zero from the board, the switches only coarse", 0.01,
              (("camera_rpi_module3_wide", 1.0), ("fiducial_pcb", 1.0)),
              "a Camera Module 3 Wide about 50 mm over an FR-4 board of copper dots, its "
              "lens locked; a dot's centroid to a tenth of a pixel; 10 um budgeted: "
              "control.vision (an estimate)"),
    Reference("vision_fiducial_ov5647", "vision_fiducial with the cheaper OV5647 camera "
              "(manual focus, 5 MP)", 0.015,
              (("camera_ov5647_m12", 1.0), ("fiducial_pcb", 1.0)),
              "an OV5647 on an M12 lens about 50 mm over the board, 39 um a pixel (the "
              "Wide's 27): 15 um budgeted (an estimate, scaled from vision_fiducial's)"),
)}
"""The user's (2026-10-12): "vision based homing to a fiducial target with a reference
sheet, we don't need super accurate switch based homing". With ``vision_fiducial`` every
input's homing is the camera's ``tool_mm`` at the tip (:func:`precision.precision`), and
the switches only find the joints coarsely (any slot switch: ``endstop``)."""
