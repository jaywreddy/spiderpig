"""Transmissions: what turns an arm's input (a link about a frame pivot) from a motor.

Each of an arm's inputs (:attr:`linkage.Linkage.drives`) gets a :class:`DriveGroup` (named
``drive:<input>``): a stepper (:mod:`spiderpig.actuators`, the role ``actuator``) bolted
under the outer frame plate, and a transmission (the role ``transmission``, one per
drive: ``transmission@q2``) from its shaft to the driven link's hub at the frame pivot:

=============  ===========================================================================
key            construction
=============  ===========================================================================
``belt``       a GT2 timing belt, 16 to 80 teeth (5:1): the large pulley bolted to the
               link, a stock closed loop, the motor on slots in the plate to tension it
               (:mod:`construction.transmissions.belt`)
``capstan``    a Dyneema cord wrapped on the motor's shaft and anchored on a
               laser-cut drum bolted to the link (10:1 on the bare shaft, no backlash)
               (:mod:`construction.transmissions.capstan`)
=============  ===========================================================================

**Where it goes.** The hub's parts (the large pulley, the drum's plates) stand in the
layers right under the driven link, coaxial with its pivot; the motor's (its pulley, its
capstan) in the same layers at the motor, whose shaft comes up through the outer plate
from the motor under it. The drive publishes those layers (:class:`DriveHub`); the
pivot's axle then claims only a seat inside the hub's bore there (``nested``, no spacer
ring: :class:`construction.axle.AxleGroup`). The motor's place is chosen before planning,
around the pivot below the arm, clear of the frame's pillar heads and the other drives'
motors (:func:`place_motor`): a fixed point of the side's geometry.

**Its error terms** (:meth:`DriveGroup.error_terms`): the transmission's own (a belt's
spans' stretch and its teeth's backlash, a cord's stretch) in series with the motor's seen
through the reduction (its microstep, its holding stiffness times the ratio squared).
"""

from __future__ import annotations

from spiderpig.construction.transmissions.base import (
    DriveGroup,
    DriveHub,
    Transmission,
    place_motor,
)
from spiderpig.construction.transmissions.belt import BeltDrive
from spiderpig.construction.transmissions.capstan import CapstanDrive

TRANSMISSIONS: dict[str, Transmission] = {c.key: c for c in (BeltDrive(), CapstanDrive())}

__all__ = ["TRANSMISSIONS", "BeltDrive", "CapstanDrive", "DriveGroup", "DriveHub",
           "Transmission", "place_motor"]
