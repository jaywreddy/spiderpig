"""Contract for servo models.

Servo frame (every dimension in a :class:`ServoSpec` uses it):

* origin on the **output axis**, at the **output face** of the case (the face
  the spline sticks out of; where that face is recessed around the shaft, the
  recessed face);
* **+z points out of the output face**, toward the horn;
* **+x points along the body's long side, toward the body centre** (for a
  servo whose output is off-centre, the body centre sits at ``x =
  axis_offset``, the body spans ``x in [axis_offset - L/2, axis_offset + L/2]``);
* +y completes a right-handed frame.

The integration code mounts the servo on top of the frame plate with the
output face down (servo +z = world -z), the output axis on the crank centre
O, and servo +x along a chosen direction ``u`` in the plate.

Everything but the fields documented as optional is required data;
unverified values are marked where they are set (:mod:`servos.catalog`).
"""

from __future__ import annotations

from dataclasses import dataclass, field

THREAD_D = {"M2": 2.0, "M2.5": 2.5, "M2.6": 2.6, "M3": 3.0}   # nominal diameters (mm)


@dataclass(frozen=True)
class HolePattern:
    """``count`` holes equally spaced on a circle (pitch-circle diameter ``pcd``).

    ``hole_d`` is the finished clearance hole a mating plate needs;
    ``screw`` names the catalog item that goes through it (e.g.
    ``"m2_shcs_6"``) when the length is fixed; ``angle_deg`` is the first
    hole's angle from +x.

    Optional (for parts that bolt on and pick their own screw length):
    ``tapping`` marks pilot holes for thread-forming (self-tapping) screws
    rather than machine threads; ``thread_depth`` is how deep the thread
    goes from the face the screws enter; ``max_depth`` is how far a screw may
    reach from that face before it hits something (``None``: same as
    ``thread_depth``).
    """

    count: int
    pcd: float
    hole_d: float
    screw: str | None = None
    thread: str = ""                # thread the holes take in the horn, e.g. "M2.5"
    angle_deg: float = 0.0
    tapping: bool = False
    thread_depth: float | None = None
    max_depth: float | None = None

    @property
    def thread_d(self) -> float:
        """Nominal diameter of the thread (0 if unknown)."""
        return THREAD_D.get(self.thread, 0.0)

    @property
    def reach(self) -> float | None:
        """How far a screw may go into the holes from the face (None if unknown)."""
        return self.max_depth if self.max_depth is not None else self.thread_depth


@dataclass(frozen=True)
class Horn:
    """The output horn that bolts to the adapter (stock unless ``bom_key`` given).

    Shape (for modelling): a disc of ``diameter``, ``flange_thickness`` thick at
    the outer face, on a hub of ``hub_d`` that runs from the outer face down to
    ``hub_bottom`` (servo z; default: the horn seat). ``center_screw_head_d``
    and ``center_screw_head_h`` are the pocket a mating part needs on the axis
    (the centre screw head, or whatever else stands proud of the horn's outer
    face there); ``center_boss`` ``(d, h)`` is what is drawn there, when known.
    A boss narrower than ``center_hole_d`` passes through the horn: it belongs
    to the servo's output (a centre ring), not the horn.
    """

    name: str
    diameter: float
    thickness: float
    pattern: HolePattern
    center_screw_head_d: float      # clearance around the centre (spline) screw head
    bom_key: str | None = None      # catalog item if bought separately
    center_screw_head_h: float = 0.0
    center_boss: tuple[float, float] | None = None
    center_hole_d: float = 0.0      # hole through the horn's outer face on the axis
    flange_thickness: float | None = None
    hub_d: float = 0.0
    hub_bottom: float | None = None
    extra_holes: tuple[HolePattern, ...] = ()   # other holes in the horn face (not used)


@dataclass(frozen=True)
class MountHole:
    """A hole in the frame plate that fixes the servo.

    ``x, y`` in the servo frame; ``d`` the finished hole diameter;
    ``screw`` catalog key; ``z`` the height in the servo frame where the
    screw engages the servo (0 = output face, negative = further up the
    body toward the back face; used to size standoffs for ear-mounted
    servos); ``standoff`` whether a spacer is needed between plate and servo.
    Optional: ``depth`` how deep the hole goes into the servo from that face
    (``None``: not published).
    """

    x: float
    y: float
    d: float
    screw: str | None = None
    z: float = 0.0
    standoff: bool = False
    depth: float | None = None


@dataclass(frozen=True)
class Idler:
    """Coaxial support on the back face (opposite the output), if the servo has one.

    ``boss_d``/``boss_h`` describe the idler boss/bearing seat (standing on
    the face at ``base_z``, default ``z = -H``, pointing -z); ``pattern`` is
    how an idler horn/bracket bolts to it.

    Optional: ``horn_d``/``horn_thickness``/``horn_face_z`` the idler horn
    (its outer face, servo z); ``included`` whether the idler parts come with
    the servo (else ``horn_bom_key`` names what to buy).
    """

    boss_d: float
    boss_h: float
    pattern: HolePattern
    horn_bom_key: str | None = None  # idler horn if sold separately
    base_z: float | None = None
    horn_d: float = 0.0
    horn_thickness: float = 0.0
    horn_face_z: float | None = None
    included: bool = True


@dataclass(frozen=True)
class Relief:
    """A raised region of a case face that a flat plate must clear (a cut-out).

    Rectangle ``x0..x1`` by ``y0..y1`` in the servo frame, standing
    ``height`` mm proud of the face that rests on the plate.

    ``solid``: the relief is part of the case (a parametric model draws it);
    ``False`` marks clearance for something else (an idler horn, pins that
    only one CAD model shows). ``label`` says what it is.
    """

    x0: float
    x1: float
    y0: float
    y1: float
    height: float
    solid: bool = True
    label: str = ""


@dataclass(frozen=True)
class Recess:
    """Where a case face is recessed around the output axis (servo frame).

    The recess covers the disc of radius ``r`` about the axis and the whole
    width of the case from its near end up to ``x = x_end``; it is ``depth``
    mm below the face that carries the mounting holes.
    """

    r: float
    x_end: float
    depth: float


@dataclass(frozen=True)
class CadRef:
    """A downloadable model of the servo, aligned to the servo frame by ``transform``.

    ``transform`` is a 4x4 row-major matrix (as a tuple of 16 floats) taking
    the file's coordinates (after scaling by ``scale`` to mm) into the servo
    frame. ``sha256`` pins the exact file; a mismatch means "don't use it".

    Optional: ``member`` names the model inside a zip archive (``url`` is
    then the archive, pinned by ``archive_sha256``; ``sha256`` pins the
    member). ``strip`` lists the servo-frame bounding boxes ``(x0, y0, z0,
    x1, y1, z1)`` of solids to drop (the stock output horn and its screw,
    which :mod:`servos.model` draws separately); ``strip_cut`` a cylinder
    ``(r, z0, z1)`` about the output axis cut away where a model fuses the
    horn into the case. Files are downloaded and cached at build time
    (:mod:`servos.cad`), never checked into the repo.
    """

    url: str
    sha256: str
    filename: str
    format: str = "step"           # "step" | "stl"
    scale: float = 1.0
    transform: tuple[float, ...] = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
    license: str = ""
    source: str = ""               # human-readable provenance
    member: str = ""
    archive_sha256: str = ""
    strip: tuple[tuple[float, ...], ...] = ()
    strip_cut: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class ServoSpec:
    key: str                        # registry key, e.g. "sts3215"
    name: str
    bom_key: str                    # catalog item for the servo itself
    # L (along x), W (along y), H: the main case between the front and rear faces
    # that carry the mounting holes (mount_face_z - H is the rear hole face)
    body: tuple[float, float, float]
    axis_offset: float              # body centre x in the servo frame
    spline_od: float                # 0 if not published
    seat_height: float              # output face to horn seat (the horn's back face)
    horn: Horn
    mount: tuple[MountHole, ...]
    mount_face_z: float = 0.0       # z of the front face that rests on a plate (its hole face)
    front_reliefs: tuple[Relief, ...] = ()
    rear_face_z: float | None = None  # z of the rear face that rests on a plate, if it has holes
    rear_mount: tuple[MountHole, ...] = ()
    rear_reliefs: tuple[Relief, ...] = ()   # heights measured beyond the rear face (-z)
    continuous: bool = False        # can turn a crank (full rotation, speed mode)
    ears: tuple[float, float, float] | None = None  # (z_bottom, thickness, overall length) if eared
    idler: Idler | None = None
    cad: CadRef | None = None
    torque_kgcm: float | None = None
    voltage: tuple[float, float] | None = None
    interface: str = ""
    notes: str = ""
    sources: tuple[str, ...] = field(default_factory=tuple)
    # optional detail
    spline_top: float = 0.0         # z of the spline's end (0: not modelled)
    front_recess: Recess | None = None
    rear_recess: Recess | None = None
    cad_alternates: tuple[CadRef, ...] = ()   # tried in order when ``cad`` is unavailable
    weight_g: float | None = None
    speed_rpm: float | None = None  # no-load output speed at the upper voltage

    @property
    def horn_bottom(self) -> float:
        """z (servo frame) of the horn's outer face."""
        return self.seat_height + self.horn.thickness

    @property
    def horn_face_depth(self) -> float:
        """How far the horn's outer face sits beyond the plate the servo rests on."""
        return self.horn_bottom - self.mount_face_z

    @property
    def rear_z(self) -> float:
        """z of the rear hole face (the back of the main case)."""
        if self.rear_face_z is not None:
            return self.rear_face_z
        return self.mount_face_z - self.body[2]

    @property
    def cads(self) -> tuple[CadRef, ...]:
        """Every model reference, preferred first."""
        return ((self.cad,) if self.cad is not None else ()) + self.cad_alternates
