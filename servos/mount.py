"""Mounting a servo on the frame plate and coupling it to the crank.

Contract used by :mod:`fabricate` (implementations live in this package):

The servo sits on top of the frame plate, output face down, output axis on
the crank centre O, its body's long side (servo +x) along the plate
direction ``u``. Its horn hangs below the plate (the plate has a clearance
hole the horn can turn in). Directly below the horn the crank's top plate is
the **horn adapter**: horn screws pass up through it into the horn, and
their heads sink into the crank plate below it (so nothing sticks out into
the stack). If the servo has a rear idler, an **idler bracket** (a laser-cut
plate over the servo's back, on standoffs from the frame plate) supports it
from the other side.

Z is quantized: the horn's outer face must land on a slot boundary, so the
servo may be lifted off the plate by ``lift`` mm (shims/spacers under its
mounting points).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from hardware.bom import BomLine
from mechanism import Body
from servos.spec import ServoSpec

XY = tuple[float, float]


@dataclass(frozen=True)
class Hub:
    """What the stack planner must reserve under the frame plate for this servo."""

    radius: float        # horn + anything else under the plate near O (screw heads)
    slots: int           # slots between the plate's underside and the horn's outer face
    lift: float          # servo raised off the plate top by this much (mm)
    adapter_radius: float  # radius of the adapter + head-recess crank plates


@dataclass
class ServoMount:
    """Everything the servo contributes, in world coordinates at build time."""

    plate_holes: list[tuple[XY, float]] = field(default_factory=list)   # (xy, diameter)
    plate_pads: list[tuple[XY, XY, float]] = field(default_factory=list)  # pills (p, q, r) the
    #                                         frame plate outline must include (servo footprint)
    adapter_holes: list[tuple[XY, float]] = field(default_factory=list)  # horn screws, centre
    recess_holes: list[tuple[XY, float]] = field(default_factory=list)   # screw heads, centre
    bodies: list[Body] = field(default_factory=list)   # servo, horn, idler bracket, standoffs
    extras: list[BomLine] = field(default_factory=list)  # screws, shims (unmodelled)


def hub(spec: ServoSpec, pitch: float) -> Hub:
    """Hub reservation for ``spec`` on a stack of ``pitch``-thick plates."""
    raise NotImplementedError


def mount(
    spec: ServoSpec,
    *,
    center: XY,
    u: XY,
    plate_top: float,
    pitch: float,
    crank_angle: float,
    frame_host: str,
    crank_host: str,
    idler_bracket: bool = True,
) -> ServoMount:
    """Place ``spec`` on the frame plate. ``crank_angle`` (radians) is the angle of
    the crank's first crankpin at build time; the horn and its hole pattern turn
    with the crank, so their angle is ``crank_angle + horn.pattern.angle_deg``.
    """
    raise NotImplementedError


def servo_to_world(center: XY, u: XY, face_z: float) -> np.ndarray:
    """4x4 transform from the servo frame to world (output face at ``face_z``, facing down)."""
    raise NotImplementedError
