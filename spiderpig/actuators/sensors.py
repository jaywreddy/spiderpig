"""Joint angle sensors: how an arm's planar inputs find their zero (2026-10-12, round 3 of
the arm's precision review).

An arm's turntable homes on its endstop (:mod:`construction.rotary`); the role
``angle_sensor`` (one per drive, :mod:`spiderpig.roles`) says how its planar inputs (the
palletizer's ``q2``, ``q3``) find their zero at homing (:mod:`control.homing`):

* ``gauge``: a digital angle gauge laid on each link by hand, zeroed on the base: 0.05 deg
  on a good one (:attr:`AngleSensor.repeat_deg`), the zero's error until a calibration
  refines it, lost again at every power-up;
* ``endstop``: what the WLKATA MT4 does (its STEP names an optical mounting plate on the
  base and a "J2 detection flag" on the upper arm; Thor, PAROL6, the AR4 and the Mirobot
  home every joint to a switch too): a slotted optical endstop on the frame, a printed flag
  on the link :data:`FLAG_R` from the pivot at the end of its travel; homed by a fast seek,
  a back-off and a slow approach from the same side, every time. A slot switch repeats to
  about 20 um at its flag's edge (:data:`SWITCH_REPEAT_MM`): that over the flag's radius is
  the joint's (a belt-driven joint could then snap to its driver's phase, Klipper's
  ``endstop_phase``, step-exact; not counted);
* ``scl3300``: a Murata SCL3300-D01 MEMS inclinometer on the link (the upper arm reads
  ``q2``; the forearm, which keeps ``q3``'s direction, ``q3``), its in-plane tilt
  ``atan2`` of its two axes in the link's plane, averaged half a second at rest
  (0.001 deg/sqrt(Hz) noise), its offset set once at assembly against the gauge and
  drifting 0.005 deg/K: :attr:`AngleSensor.repeat_deg` over a few kelvin. Read at rest
  before each final approach it also takes out what a capstan's creep or slip and the
  drive's sag leave (:mod:`control.incline`). The Dobot Magician reads its arms' angles
  from accelerometers in the links the same way.

``repeat_deg`` is what homing leaves at the joint each time (a repeatability term: the
budget's ``homing_mm``, :func:`precision.drive_budgets`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

SWITCH_REPEAT_MM = 0.02    # a slotted optical switch's repeatability at its flag's edge (as
#                            the turntable's, construction.rotary; a typical figure,
#                            unverified)
FLAG_R = 40.0              # the flag's edge from the joint's pivot, on the link (mm)


@dataclass(frozen=True)
class AngleSensor:
    """How a joint's zero is read (module docstring): ``repeat_deg`` what each homing
    leaves; ``bom_key`` the part bought per joint (``""``: none); ``source``: where the
    numbers come from."""

    key: str
    label: str
    repeat_deg: float
    bom_key: str = ""
    source: str = ""


ANGLE_SENSORS: dict[str, AngleSensor] = {s.key: s for s in (
    AngleSensor("gauge", "a digital angle gauge on each link, by hand", 0.05,
                source="a 0.05 deg digital angle gauge (0.05-0.1 deg in use: the better)"),
    AngleSensor("endstop", "a slotted optical endstop on the frame, a flag on the link",
                round(math.degrees(SWITCH_REPEAT_MM / FLAG_R), 4), bom_key="optical_endstop",
                source=f"{SWITCH_REPEAT_MM * 1000:g} um at a flag {FLAG_R:g} mm out "
                       "(unverified); the MT4's way"),
    AngleSensor("scl3300", "a Murata SCL3300-D01 inclinometer on each link", 0.02,
                bom_key="inclinometer_scl3300",
                source="Murata SCL3300-D01: 0.0055 deg/LSB, 0.001 deg/sqrt(Hz) noise, "
                       "offset drift 0.005 deg/K (a few K: 0.02 deg); estimate"),
)}


def get(key: str) -> AngleSensor:
    try:
        return ANGLE_SENSORS[key]
    except KeyError:
        raise KeyError(f"unknown angle sensor {key!r}; have {sorted(ANGLE_SENSORS)}") from None
