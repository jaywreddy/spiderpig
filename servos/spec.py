"""Contract for servo models.

Servo frame (every dimension in a :class:`ServoSpec` uses it):

* origin on the **output axis**, at the **output face** of the case (the face
  the spline sticks out of);
* **+z points out of the output face**, toward the horn;
* **+x points along the body's long side, toward the body centre** (for a
  servo whose output is off-centre, the body centre sits at ``x =
  axis_offset``, the body spans ``x in [axis_offset - L/2, axis_offset + L/2]``);
* +y completes a right-handed frame.

The integration code mounts the servo on top of the frame plate with the
output face down (servo +z = world -z), the output axis on the crank centre
O, and servo +x along a chosen direction ``u`` in the plate.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class HolePattern:
    """``count`` holes equally spaced on a circle (pitch-circle diameter ``pcd``).

    ``hole_d`` is the finished clearance hole a mating plate needs;
    ``screw`` names the catalog item that goes through it (e.g.
    ``"m2_shcs_6"``); ``angle_deg`` is the first hole's angle from +x.
    """

    count: int
    pcd: float
    hole_d: float
    screw: str | None = None
    thread: str = ""                # thread the holes take in the horn, e.g. "M2.5"
    angle_deg: float = 0.0


@dataclass(frozen=True)
class Horn:
    """The output horn that bolts to the adapter (stock unless ``bom_key`` given)."""

    name: str
    diameter: float
    thickness: float
    pattern: HolePattern
    center_screw_head_d: float      # clearance around the centre (spline) screw head
    bom_key: str | None = None      # catalog item if bought separately


@dataclass(frozen=True)
class MountHole:
    """A hole in the frame plate that fixes the servo.

    ``x, y`` in the servo frame; ``d`` the finished hole diameter;
    ``screw`` catalog key; ``z`` the height in the servo frame where the
    screw engages the servo (0 = output face, negative = further up the
    body toward the back face; used to size standoffs for ear-mounted
    servos); ``standoff`` whether a spacer is needed between plate and servo.
    """

    x: float
    y: float
    d: float
    screw: str | None = None
    z: float = 0.0
    standoff: bool = False


@dataclass(frozen=True)
class Idler:
    """Coaxial support on the back face (opposite the output), if the servo has one.

    ``boss_d``/``boss_h`` describe the idler boss/bearing seat (at
    ``z = -H``, pointing -z); ``pattern`` is how an idler horn/bracket
    bolts to it.
    """

    boss_d: float
    boss_h: float
    pattern: HolePattern
    horn_bom_key: str | None = None  # idler horn if sold separately


@dataclass(frozen=True)
class CadRef:
    """A downloadable model of the servo, aligned to the servo frame by ``transform``.

    ``transform`` is a 4x4 row-major matrix (as a tuple of 16 floats) taking
    the file's coordinates (after scaling by ``scale`` to mm) into the servo
    frame. ``sha256`` pins the exact file; a mismatch means "don't use it".
    """

    url: str
    sha256: str
    filename: str
    format: str = "step"           # "step" | "stl"
    scale: float = 1.0
    transform: tuple[float, ...] = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
    license: str = ""
    source: str = ""               # human-readable provenance


@dataclass(frozen=True)
class ServoSpec:
    key: str                        # registry key, e.g. "sts3215"
    name: str
    bom_key: str                    # catalog item for the servo itself
    body: tuple[float, float, float]  # L (along x), W (along y), H (output face to back face)
    axis_offset: float              # body centre x in the servo frame
    spline_od: float
    seat_height: float              # output face to horn seat (the horn's back face)
    horn: Horn
    mount: tuple[MountHole, ...]
    ears: tuple[float, float, float] | None = None  # (z_bottom, thickness, overall length) if eared
    idler: Idler | None = None
    cad: CadRef | None = None
    torque_kgcm: float | None = None
    voltage: tuple[float, float] | None = None
    interface: str = ""
    notes: str = ""
    sources: tuple[str, ...] = field(default_factory=tuple)

    @property
    def horn_bottom(self) -> float:
        """z (servo frame) of the horn's outer face."""
        return self.seat_height + self.horn.thickness
