"""Transmissions: what turns an arm's input (a link about a frame pivot) from a motor.

Each of an arm's inputs (:attr:`linkage.Linkage.drives`) gets a :class:`JointDrive` (named
``drive:<input>``): a stepper (:mod:`spiderpig.actuators`, the role ``actuator``) on a
frame plate, and a transmission (the role ``transmission``, one per drive:
``transmission@q2``) from its shaft to the driven link's hub at the frame pivot:

=============  ===========================================================================
key            construction
=============  ===========================================================================
``belt``       a GT2 timing belt, 16 to 80 teeth (5:1): the large pulley bolted to the
               link's pad, a stock closed loop, the motor on a slider plate outside its
               frame plate, tensioned from outside (:mod:`construction.transmissions.belt`)
=============  ===========================================================================

**Where it goes.** The hub's parts (the large pulley) stand in the
layers beside the driven link, coaxial with its pivot, and a layer short of the plate (the
axle's thrust ring there); the motor's (its pulley) in the same layers at the
motor, whose shaft comes in through its frame plate. The drives on one pivot alternate
plates: the first's motor under the outer plate (its hub under its link), the next's on
the inner plate (its hub over it). The drive publishes its hub (:class:`DriveHub`): the
pivot's axle then claims only a seat inside it (``nested``, no spacer ring:
:class:`construction.axle.AxleGroup`), and the driven link grows a pad round its pivot for
the hub's bolts (:func:`construction.plates.hub_pads`). The motor's place is chosen before
planning, round the pivot, clear of the frame's pillar heads and the other drives' motors
on its plate (:func:`place_motor`): a fixed point of the side's geometry. A transmission
checks its own parts against each other (:meth:`Transmission.check`): the planner never
compares a group's shapes with each other.

**Its error terms** (:meth:`JointDrive.error_terms`): the transmission's own (a belt's
spans' stretch and its teeth's backlash) in series with the motor's seen
through the reduction (its microstep, its microstep's systematic error, its holding
stiffness times the ratio squared).
"""

from __future__ import annotations

from spiderpig.construction.transmissions.base import (
    DriveHub,
    JointDrive,
    Transmission,
    place_motor,
)
from spiderpig.construction.transmissions.belt import BeltDrive

TRANSMISSIONS: dict[str, Transmission] = {c.key: c for c in (BeltDrive(),)}

__all__ = [
    "TRANSMISSIONS",
    "BeltDrive",
    "DriveHub",
    "JointDrive",
    "Transmission",
    "place_motor",
]
