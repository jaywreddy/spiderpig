"""MuJoCo model (MJCF) of the fabricated walker.

:func:`build_mjcf` turns a :class:`fabricate.BuildConfig` into one
self-contained MJCF document (no external files) plus a JSON-able metadata
dict. The XML runs in Python (``mujoco``); the browser's physics drive
streams the server's session of it (:mod:`sim.live`, ``/ws/sim``), and the
metadata tells the viewer how to put the simulated poses on the glb nodes of
:mod:`viewer.bake_gltf`.

Units and frames
----------------
* **SI** throughout the model (m, kg, s, rad, N, N·m). The design is in mm;
  every length is scaled by ``1e-3`` on the way in.
* **mech** frame (the design's): ``x`` along the linkage plane (the walking
  axis), ``y`` up (feet at low ``y``), ``z`` lateral, the layer stack (left
  side ``z < 0``, right side ``z > 0``); origin on the crank axis ``O`` at the
  robot's mid-plane.
* **world** frame (MuJoCo's usual z-up): ``x`` forward (positive drive walks
  towards ``+x``), ``y`` left, ``z`` up; gravity ``-z``; the floor is the plane
  ``z = 0``. ``world = Rx(+90°) · mech``: mech ``+y`` -> world ``+z``, mech
  ``+z`` -> world ``-y`` (so the ``L.`` side is at world ``+y``, on the left).
  This is the same rotation as the glb's ``walker`` root node.
* The free-floating **base** body's frame *is* the mech frame (in metres):
  its origin is ``O`` on the mid-plane and its axes are mech ``x, y, z``.
  Every other body is a descendant with an unrotated frame at the reference
  configuration, so positions inside the model are mech coordinates and every
  hinge axis is the stack axis ``(0, 0, ±1)``. At ``qpos0`` the base sits at
  ``(0, 0, h)`` with quaternion ``Rx(+90°) · Rz(pitch)``: standing on its feet
  at ``t_ref`` (:func:`rest_pose`: the pair of feet under the centre of mass
  level, :attr:`SimParams.clearance` above the floor). The metadata has the
  resulting ``mech_to_world_ref`` and ``rest_height``. That height is the
  base's origin over the floor with the lowest foot sphere's bottom
  ``clearance`` above it; the quasi-static walking model (:mod:`walk`) rests
  the body on the feet's *centres*, so its height reads ``link_radius`` (+ the
  clearance) lower than the base's here (Klann quad: 88.8 vs 94.4 mm).

Kinematic tree
--------------
* ``base`` (freejoint ``base``): the robot's rigid frame, i.e. both sides'
  torso (inner frame plate) and everything whose ``rigid_with`` chain ends
  there: outer frame plates, pillars, servos and their screws, the centre
  plates and the frame ties.
* per side ``S``: ``S.conn``, the crankshaft (hinge ``S.crank`` at ``O``):
  every crank body of the template (``conn``, ``conn_upper``, the shaft
  ``coupler``: one printed crankshaft in the robot) with the crank segments,
  their screws and nuts and the servo horn.
* every link ``b<k>`` of every leg, on a hinge at one of its joints. The tree
  is the linkage's connection graph (bodies pinned at a shared joint) grown
  from the base: every body pinned to a body already in the tree hangs from
  it, the base's neighbours first (the cranks before the links on frame
  pivots), then each new body's in turn. For Klann: ``b1`` at ``M`` on the
  crank, ``b4`` at ``D`` on ``b1``, ``b3`` at ``A`` and ``b2`` at ``B`` on
  the base. Every other connection closes a loop with a ``<connect>``
  equality named ``S.<joint><suffix>`` (Klann: ``S.C`` between ``b1`` and
  ``b3``, ``S.E`` between ``b4`` and ``b2``). Their anchors are placed at
  the reference configuration, so they hold exactly at ``qpos0``. The
  mechanism is planar: the z-rows of those constraints are redundant
  (MuJoCo's soft constraints handle that).
* Link pins follow the link they're glued to (``rigid_with``).

Body frames sit on their pivot (Klann: ``O``, ``M``, ``D``, ``A``, ``B``) in
the plane and at the middle of their own layer(s) in ``z``; the reference
crank angle is ``t_ref = 0`` (the glb's frame 0), and the crank angle is
``t = t_ref + crank_sign * qpos[S.crank]``.

Metadata (for the viewer)
-------------------------
``nodes`` maps every fabricated body, i.e. every glb node by name, to its
MuJoCo body (hardware follows its ``rigid_with`` host; ``L.torso`` and
``R.torso`` are ``base``; ``conn``, ``conn_upper`` and ``coupler`` are
``S.conn``). A glb node's frame-0 matrix ``M0`` places its part at ``t_ref``
in mech mm. To draw a simulated state, the glb's ``walker`` root takes the
base's world pose (``base.xpos`` × 1000, ``base.xquat``) instead of its fixed
rotation and ground offset, and each node becomes ``D_b @ M0`` with
``D_b = X_b(now) · X_b(qpos0)⁻¹``, ``X_b`` the pose of its MuJoCo body in the
base frame (translations × 1000 for mm). :func:`sim.run.body_motions` is the
reference implementation. The metadata also has the frames, each body's
``ref_pos`` (its origin in the base frame at ``qpos0``), the actuators and
their limits, the feet (site / geom names), the loops and the masses.

Mass and inertia
----------------
Exact, from the fabricated parts at ``t_ref`` (OCCT B-rep volume, centroid
and inertia tensor per part, combined per body with the parallel-axis
theorem) and written as each body's ``<inertial>`` (``inertiafromgeom`` is
off: geoms carry no mass). Materials and densities are
:func:`hardware.mass.material_of`'s (cast acrylic 1.19 g/cm³, PLA 1.24,
steel 7.85, brass 8.5, an aluminium horn 2.70); printed parts are scaled by
:attr:`SimParams.printed_fill` (1.0 = the BOM's 100 % infill, an upper
bound); the servo is its datasheet mass (``ServoSpec.weight_g``, 55 g for
the STS3215) spread uniformly over its modelled case. The battery and the
boards ride the base as parts (the electronics deck, :mod:`construction.deck`: its
plate, rails, boards, battery and switch are parts of the robot like any other, the
electronics at their catalogued masses, ``mass_g``), plus :attr:`SimParams.payload_g`
(0 by default; a point mass at :attr:`SimParams.payload_pos` for anything else carried;
the wiring, ~5-10 g, is not modelled). A robot whose frame has no room for the deck
(``meta["deck"]["fitted"]`` false) carries :data:`DECK_FALLBACK_G` there instead, the
deck's measured mass on the default designs. (Before the deck, a 100 g ESTIMATE at the
crank axis stood for the electronics: +100 / +200 g on the Klann quad cost 1 / 3 % of
speed and raised the mean torque from 0.05 to 0.055 / 0.065 N·m.) ``mass.total``
counts all of it.

Collision geometry
------------------
Simple shapes; only the robot and the floor collide (robot geoms
``contype=1 conaffinity=0``, floor ``contype=0 conaffinity=1``: the layer
planner already guarantees robot parts never meet each other).

* feet: a sphere of the link radius at each of the linkage's foot tips (the
  rounded end of the foot link, Klann's ``b4.F``; the plate's 3 mm width is
  ignored), geom and site ``S.foot<suffix>`` (``S.foot_<joint><suffix>``
  when a leg has several feet);
* links: capsules along their outline at their layer, link radius; every
  capsule that ends on a foot joint (the foot link's and any link pinned
  there: Strider's ``b4`` and ``b8``) stops two radii short of it so the foot
  sphere alone makes the foot contact (a near-horizontal link would
  otherwise touch the floor with its end cap beside the foot); the metadata
  names those links per foot (``feet[..]["links"]``) and
  :func:`sim.run.simulate` doesn't count their floor contacts as a fall;
* base: the convex hull of each frame plate and servo (so a fall or tip
  shows).

Friction is one coefficient for everything on the floor
(:attr:`SimParams.friction`, 0.5: acrylic or PLA on a hard floor, dry). The
feet use ``condim=4`` so :attr:`SimParams.torsional_friction` acts (a 6 mm
sphere standing in for a 3 mm plate edge spinning on the floor, second-order
for skid steering); every other geom uses ``condim=3``, where MuJoCo ignores
the torsional and rolling coefficients. Contacts use elliptic
cones, and a moderately soft contact (``solref 0.01 1``, MuJoCo's default
``solimp 0.9 0.95 0.001``): about 0.4 mm of static give and 2.2 mm at
impacts (Klann quad at full speed; 1.2 mm with ``solref`` 5 ms, 4.2 mm with
20 ms), standing in for the leg's compliance and the play of its running fits
(an acrylic foot on a hard floor alone is stiffer). The softness is
UNVALIDATED (one leg's deflection under 5 N on the real build would set it).
Speed, stride and mean drive torque don't depend on it (Klann quad: 161-164
mm/s across 5-20 ms); the impact torque peaks, foot slip and every support
metric do: feet down 2.6 / 3.1 / 3.6, airborne 9 / 4 / 0 %, a side on fewer
than two feet 66 / 47 / 26 % of the time at 5 / 10 / 20 ms
(:func:`sim.run.compare_with_walk` says so in its ``notes``).

Drives and joints
-----------------
One ``velocity`` actuator per side on its crank hinge (``L.drive``,
``R.drive``): ``ctrl`` is the crank speed in rad/s (positive = forward),
``ctrlrange`` ± the servo's no-load speed (STS3215: 52 rpm = 5.45 rad/s),
``forcerange`` ± its stall torque (19.5 kg·cm = 1.91 N·m), and ``kv`` such
that the stall torque is reached at a speed error of
:attr:`SimParams.stall_error` (10 %) of the no-load speed, a stiff speed
loop like the servo's own. The actuator force is then the torque on the
output shaft, directly comparable with the servo's ratings. The MJCF alone
lets that loop deliver the stall torque at any speed; :func:`motor_line`
(applied every step by :func:`sim.run.simulate` and :class:`sim.live.LiveSim`)
narrows ``forcerange`` to a DC motor's speed-torque line while motoring,
``stall · (1 - |speed| / no_load)``, so the drives can't push harder than the
servo at the speed they turn (braking keeps the full stall torque). The crank
hinge carries the gear train's reflected rotor inertia as ``armature`` and no
extra damping: the servo's own losses are inside its speed and torque
ratings. Passive pins carry a tiny armature and damping (numerical
regularization) and a Coulomb ``frictionloss`` (:attr:`SimParams.pin_frictionloss`,
an estimate of mu · load · r for a steel pin in an acrylic running fit).
The armature is an UNVERIFIED estimate (a coast-down
measurement of the servo would settle it); between 5e-4 and 2e-2 kg·m² the
walking speed and loop closure don't change (measured) while the torque
peaks do (0.89 / 0.43 / 0.16 N·m), so present peaks as a range; below
:data:`MIN_CRANK_ARMATURE` the model is refused.

What the model assumes (and the hardware must deliver)
-------------------------------------------------------
* **Phase-locked sides.** The two cranks are driven by two servos, and every
  straight-walk figure here comes from cranks that keep their phase: the Klann
  quad stands only while they do (open-loop, a 2 / 5 / 10 % speed mismatch
  rolls it over at 17 / 8 / 3 s; a held offset of 90° or more within a second;
  measured). The commands therefore go through :class:`sim.run.PhaseLock`, a
  PI lock on the sides' crank difference with the right servo
  :attr:`SimParams.servo_mismatch` slower than told. The real STS3215 bus
  needs position feedback and that controller, not open-loop wheel mode.
* **A rigid crankshaft.** Each side's crank is one body (``S.conn``): the
  printed crankshaft, its segments, screws and horn flex not at all. The pin
  loads (``walk_metrics`` ``loop_force``) are what that rigid chain carries.
* **Contact softness** stands in for the legs' compliance and the play of the
  running fits and is UNVALIDATED (above); the impulsive gait (airborne a few
  per cent of the time at full speed) and its torque and acceleration peaks
  follow it.

Solver
------
``timestep`` 1 ms, ``implicitfast`` (the velocity servos are integrated
implicitly), Newton with elliptic cones, 100 iterations (50 line search).
The loop equalities are as stiff as the step allows: ``solref 0.002 1``
(MuJoCo keeps the time constant at least two steps) and ``solimp 0.99 0.999
0.0001``, which keeps the closure error under about 0.1 mm while walking.
Halving the step leaves the walking metrics unchanged.
"""

from __future__ import annotations

import itertools
import math
import re
import threading
import xml.etree.ElementTree as ET
from collections import OrderedDict
from dataclasses import asdict, dataclass, field, replace
from functools import lru_cache

import numpy as np
from scipy.spatial import ConvexHull

from spiderpig import servos
from spiderpig.config import BuildConfig
from spiderpig.fabricate import fabricate, template_for
from spiderpig.hardware.mass import material_of, part_props
from spiderpig.linkage import feet_of
from spiderpig.linkage import get as get_linkage
from spiderpig.stack import body_class, is_crank, is_frame, is_link

T_REF = 0.0                     # crank angle the model's qpos0 is at (the glb's frame 0)
MIN_CRANK_ARMATURE = 5e-4       # kg·m²; see SimParams.crank_armature
MM = 1e-3                       # m per mm
KGF_CM = 9.80665e-2             # N·m per kgf·cm
RPM = 2.0 * math.pi / 60.0      # rad/s per rpm


@dataclass(frozen=True)
class SimParams:
    """Physical and numerical parameters of the model (SI unless named otherwise)."""

    friction: float = 0.65          # sliding friction robot / floor: the feet's TPU 95A
    #                                 socks on a hard floor, dry (0.6-0.7, the joinery plan;
    #                                 0.5 for bare acrylic or PLA before 2026-10-04)
    torsional_friction: float = 0.005
    rolling_friction: float = 0.0001
    timestep: float = 1e-3
    iterations: int = 100
    ls_iterations: int = 50
    stall_error: float = 0.1        # speed error (fraction of no-load speed) at stall torque
    # Reflected rotor inertia of the geared servo, UNVERIFIED estimate: a ~4e-8 kg·m²
    # micro DC motor rotor through the STS3215's 1:345 gear train (J · N²).
    crank_armature: float = 5e-3    # kg·m²
    crank_damping: float = 0.0      # N·m·s/rad (the servo's losses are inside its ratings)
    pin_armature: float = 1e-6      # kg·m², passive pins (regularization)
    # Below MIN_CRANK_ARMATURE the model is ill-posed (measured at 0: loop error 2.5 mm,
    # roll 8 deg, the robot shoots backwards): build_mjcf refuses it.
    pin_damping: float = 1e-4       # N·m·s/rad, passive pins
    # Coulomb friction of a passive pin, N·m: mu · load · r for a 3 mm steel pin in an
    # acrylic running fit (mu ~0.2) under the ~5 N a loaded leg's pin carries while
    # walking (r = 1.5 mm): 1.5e-3. An ESTIMATE (the walking pin loads peak at ~110 N,
    # the jammed ones at 155 N, so the real friction is spiky, not constant).
    pin_frictionloss: float = 1.5e-3
    # What rides on the frame besides the parts, as a point mass at ``payload_pos`` (mech
    # mm on the base): nothing by default, the electronics deck is modelled
    # (construction.deck); a design without the deck gets DECK_FALLBACK_G there instead.
    payload_g: float = 0.0
    payload_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # The controller (:class:`sim.run.PhaseLock`), not the model: the right servo turns
    # ``servo_mismatch`` slower than told (servo-to-servo variation, battery sag per side)
    # and the host holds the cranks' phase with a PI lock of these gains (fraction of the
    # no-load speed per rad, per rad·s; 0 and 0: open loop).
    servo_mismatch: float = 0.03
    phase_lock_kp: float = 1.0
    phase_lock_ki: float = 4.0
    eq_solref: tuple[float, float] = (0.002, 1.0)
    eq_solimp: tuple[float, ...] = (0.99, 0.999, 0.0001, 0.5, 2.0)
    contact_solref: tuple[float, float] = (0.01, 1.0)
    contact_solimp: tuple[float, ...] = (0.9, 0.95, 0.001, 0.5, 2.0)   # MuJoCo's default
    printed_fill: float = 1.0       # printed density / solid density (1.0 = 100 % infill)
    clearance: float = 0.5          # mm between the lowest foot and the floor at qpos0
    hull_tolerance: float = 0.5     # mm, tessellation of the base's collision hulls


# ---------------------------------------------------------------------------
# The fabricated robot, reduced to MuJoCo bodies
# ---------------------------------------------------------------------------


@dataclass
class MjBody:
    """One MuJoCo body: the robot bodies it stands for and their mass properties (mech, mm)."""

    name: str
    kind: str                           # "base" | "crank" | a link's class ("b1", "b2", ...)
    side: str | None
    kinematic: list[str] = field(default_factory=list)   # template bodies (L.torso, L.conn, ...)
    members: list[str] = field(default_factory=list)     # every robot body riding it
    mass: float = 0.0                   # kg
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))       # mm (mech)
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))  # kg·mm², about com
    z_range: tuple[float, float] = (math.inf, -math.inf)
    layer_z: float | None = None        # mm, mid-plane of the body's own plate (links)
    origin: np.ndarray = field(default_factory=lambda: np.zeros(3))    # frame origin, mm (mech)
    parent: str | None = None
    pivot: str | None = None            # the hinge's point name (Klann: O, M, D, A, B)
    mass_by: dict[str, float] = field(default_factory=dict)            # kg per material

    @property
    def z(self) -> float:
        """Mid-plane of its own plate (links), else of everything riding it."""
        return self.layer_z if self.layer_z is not None else 0.5 * sum(self.z_range)


@dataclass
class RobotModel:
    """Everything :func:`build_mjcf` needs from the fabricated robot (cached per config)."""

    config: BuildConfig
    bodies: dict[str, MjBody]           # MuJoCo body name -> body (tree order: parents first)
    host: dict[str, str]                # every robot body -> its MuJoCo body
    joints: dict[str, dict[str, np.ndarray]]   # kinematic body -> joint -> xy (mm) at t_ref
    loops: list[tuple[str, str, str, np.ndarray]]   # (name, body1, body2, xy mm)
    outlines: dict[str, list[tuple[np.ndarray, np.ndarray]]]   # link -> capsule segments (mm)
    feet: dict[str, tuple[str, str, np.ndarray]]   # foot -> (foot link, joint, xy mm)
    foot_links: dict[str, list[str]]    # foot -> every link with a joint on that foot point
    hulls: dict[str, tuple[str, np.ndarray]]  # robot body -> (MuJoCo body, hull points mm)
    link_radius: float                  # mm
    crank_sign: int
    meta: dict                          # the fabricated robot's meta
    servo: object                       # servos.spec.ServoSpec


def _kind(name: str) -> str:
    cls = body_class(name)
    if is_frame(name):
        return "base"
    if is_crank(name) or cls == "coupler":
        return "crank"
    if is_link(name):
        return cls
    raise ValueError(f"no MuJoCo role for kinematic body {name!r}")


def _side(name: str) -> str | None:
    m = re.match(r"^([LR])\.", name)
    return m.group(1) if m else None


def _mj_name(kin: str) -> str:
    """MuJoCo body of a kinematic (template) body."""
    kind = _kind(kin)
    if kind == "base":
        return "base"
    if kind == "crank":
        return f"{_side(kin)}.conn"
    return kin


def _root(body, by_name) -> str:
    """The kinematic body at the end of ``body``'s ``rigid_with`` chain."""
    seen = set()
    while body.rigid_with is not None:
        if body.name in seen:
            raise ValueError(f"rigid_with cycle at {body.name!r}")
        seen.add(body.name)
        body = by_name[body.rigid_with]
    return body.name


def _hulls(parts, tolerance: float) -> list[np.ndarray]:
    """:func:`_hull` of each part, the meshes read out in one pass."""
    from spiderpig.mesh import tessellate_many

    return [_hull_of(verts) for verts, _, _ in tessellate_many(parts, tolerance)]


def _hull(part, tolerance: float) -> np.ndarray:
    """The convex hull's vertices of a part's mesh: :func:`spiderpig.mesh.tessellate`, face
    by face, so a purchased model with a face the mesher can't triangulate (the XL330's)
    gives its hull from the faces it has instead of failing the whole model."""
    from spiderpig.mesh import tessellate

    return _hull_of(tessellate(part, tolerance)[0])


def _hull_of(verts) -> np.ndarray:
    pts = np.unique(np.round(np.asarray(verts, dtype=float), 4), axis=0)
    return pts[ConvexHull(pts).vertices]


def crank_sign(config: BuildConfig) -> int:
    """+1 if a growing crank angle walks the robot towards mech ``+x`` (the stance foot
    moves ``-x``), else -1. Judged on leg 0's first foot over the lowest tenth of its path."""
    tmpl = template_for(config)
    ts = np.linspace(0.0, 2.0 * math.pi, 720, endpoint=False)
    body, joint = feet_of(tmpl)[0]
    f = tmpl.sample(ts).joint_world[body][joint][:, :2]
    low = f[:, 1] <= np.quantile(f[:, 1], 0.1)
    vx = (np.roll(f[:, 0], -1) - np.roll(f[:, 0], 1))[low].mean()
    return 1 if vx < 0 else -1


CACHE_SIZE = 8      # designs kept per cache (a design's model is ~37 MB of RSS)

# config -> fabricated robot; config -> (robot, its parts' props). Newest last, the
# oldest dropped past CACHE_SIZE.
_FABRICATED: OrderedDict[BuildConfig, object] = OrderedDict()
_PROPS: OrderedDict[BuildConfig, tuple[object, dict]] = OrderedDict()
_MODELS: OrderedDict[tuple[BuildConfig, SimParams], tuple] = OrderedDict()  # compiled
_BUILD_LOCKS: dict[BuildConfig, threading.Lock] = {}    # single-flight builds per config
_LOCKS_LOCK = threading.Lock()


def _remember(cache: OrderedDict, key, value) -> None:
    cache[key] = value
    cache.move_to_end(key)
    while len(cache) > CACHE_SIZE:
        cache.popitem(last=False)


def build_lock(config: BuildConfig) -> threading.Lock:
    """The lock two threads building the same design's model share (so the second waits
    for the first's cached result instead of fabricating it again)."""
    with _LOCKS_LOCK:
        return _BUILD_LOCKS.setdefault(replace(config, robot=True), threading.Lock())


def clear_caches() -> None:
    """Forget every fabricated robot, reduced model, MJCF and compiled model (the sources
    changed: the server's watcher calls this beside its re-bake)."""
    _FABRICATED.clear()
    _PROPS.clear()
    _MODELS.clear()
    robot_model.cache_clear()
    _build_mjcf.cache_clear()


def fabricated(config: BuildConfig):
    """The fabricated robot (both sides) at ``t_ref`` (cached per config; one fabricated
    elsewhere at that angle comes in through :func:`set_fabricated`)."""
    config = replace(config, robot=True)
    if config not in _FABRICATED:
        _remember(_FABRICATED, config, fabricate(template_for(config), config, T_REF))
    return _FABRICATED[config]


def set_fabricated(config: BuildConfig, robot, props: dict | None = None) -> None:
    """Adopt ``robot``, the robot of ``config`` fabricated at :data:`T_REF` (both sides), as
    what :func:`fabricated` returns for it: an export that bakes the glb from the same
    fabrication doesn't fabricate twice. ``props``: its parts' mass properties already
    measured (body -> :class:`hardware.mass.PartProps`: the bake's), not measured again."""
    key = replace(config, robot=True)
    _remember(_FABRICATED, key, robot)
    if props:
        _remember(_PROPS, key, (robot, props))


@lru_cache(maxsize=CACHE_SIZE)
def robot_model(config: BuildConfig, printed_fill: float = 1.0,
                hull_tolerance: float = 0.5) -> RobotModel:
    """The fabricated robot at ``t_ref`` reduced to MuJoCo bodies (cached per config)."""
    config = replace(config, robot=True)
    robot = fabricated(config)
    servo = servos.get(config.servo)
    known = _PROPS.get(config, (None, {}))
    known = known[1] if known[0] is robot else {}
    by_name = {b.name: b for b in robot.bodies}

    # kinematic bodies and where every robot body goes
    kinematic = [b for b in robot.bodies if b.rigid_with is None]
    joints = {
        b.name: {j.name: (b.pose @ j.pose).matrix[:2, 3].copy() for j in b.joints}
        for b in kinematic
    }
    bodies: dict[str, MjBody] = {}
    for b in kinematic:
        name = _mj_name(b.name)
        mb = bodies.setdefault(name, MjBody(name, _kind(b.name), _side(b.name) if name != "base"
                                            else None))
        mb.kinematic.append(b.name)
    host = {b.name: _mj_name(_root(b, by_name)) for b in robot.bodies}

    # mass properties
    acc: dict[str, list] = {n: [] for n in bodies}
    for b in robot.bodies:
        mb = bodies[host[b.name]]
        mb.members.append(b.name)
        if b.part is None:
            continue
        material, density, fixed_g = material_of(b, config.sheet, robot.meta.get("filament"),
                                                 servo)
        if material == "printed":
            density *= printed_fill
        props = known.get(b.name) or part_props(b.part)
        vol, com, inertia = props.volume, props.com, props.inertia
        if vol <= 0:
            raise ValueError(f"part of {b.name!r} has no volume")
        rho = (fixed_g / vol) if fixed_g is not None else density * 1e-3   # g/mm³
        acc[mb.name].append((rho * vol * 1e-3, com, rho * inertia * 1e-3))  # kg, mm, kg·mm²
        mb.mass_by[material] = mb.mass_by.get(material, 0.0) + rho * vol * 1e-3
        z0, z1 = props.z_range          # the part's (optimal) bounding box's z extent
        mb.z_range = (min(mb.z_range[0], z0), max(mb.z_range[1], z1))
        if b.name in mb.kinematic:
            mb.layer_z = 0.5 * (z0 + z1)
    for name, mb in bodies.items():
        parts = acc[name]
        if not parts:
            raise ValueError(f"MuJoCo body {name!r} has no parts (no mass)")
        m = sum(p[0] for p in parts)
        c = sum(p[0] * p[1] for p in parts) / m
        inertia = np.zeros((3, 3))
        for mi, ci, ii in parts:
            d = ci - c
            inertia += ii + mi * (d @ d * np.eye(3) - np.outer(d, d))
        mb.mass, mb.com, mb.inertia = m, c, inertia

    # the tree grown from the base over the template's connections (see the module doc)
    adj: dict[str, list[tuple[int, str, str, np.ndarray]]] = {n: [] for n in bodies}
    for i, ((_, pa, ja), (_, pb, jb)) in enumerate(robot.connections):
        ma, mb_ = _mj_name(pa), _mj_name(pb)
        if ma == mb_:
            continue                    # conn - coupler: one crankshaft
        xy = joints[pa][ja]
        if np.abs(xy - joints[pb][jb]).max() > 1e-6:
            raise ValueError(f"joint {pa}.{ja} and {pb}.{jb} don't coincide at t_ref")
        point = re.sub(r"_leg\d+$", "", ja)
        adj[ma].append((i, mb_, point, xy))
        adj[mb_].append((i, ma, point, xy))
    placed = {"base"}
    loops: list[tuple[str, str, str, np.ndarray]] = []
    seen: set[int] = set()

    def hinge(a: str, b: str) -> np.ndarray | None:
        """Where the tree pins ``a`` and ``b`` together, if it does."""
        for child, parent in ((a, b), (b, a)):
            if bodies[child].parent == parent:
                return bodies[child].origin[:2]
        return None

    def grow(name: str) -> None:
        kids = []
        for i, other, point, xy in sorted(adj[name], key=lambda e: bodies[e[1]].kind != "crank"):
            if i in seen:
                continue
            seen.add(i)
            ob = bodies[other]
            if other not in placed:             # hang it from this body, on this joint
                ob.parent, ob.pivot = name, point
                ob.origin = np.array([xy[0], xy[1], ob.z])
                placed.add(other)
                kids.append(other)
                continue
            h = hinge(name, other)
            if h is not None and np.abs(xy - h).max() <= 1e-6:
                continue                        # another pin of that hinge (two cranks on O)
            m = re.search(r"_leg\d+$", name) or re.search(r"_leg\d+$", other)
            loop = base = f"{bodies[name].side or ob.side}.{point}{m.group() if m else ''}"
            k = 0
            while any(n == loop for n, *_ in loops):    # a third body on that joint
                k += 1
                loop = f"{base}#{k}"
            loops.append((loop, name, other, xy))
        for kid in kids:
            grow(kid)

    grow("base")
    missing = sorted(set(bodies) - placed)
    if missing:
        raise ValueError(f"{missing} aren't connected to the base")
    ordered = {"base": bodies["base"]}          # tree order: parents first, else as found
    while len(ordered) < len(bodies):
        for n, mb in bodies.items():
            if n not in ordered and mb.parent in ordered:
                ordered[n] = mb

    # collision geometry: link capsules (a foot link's stops two radii short of the
    # foot: the foot sphere alone makes the contact) and foot spheres
    n_feet = len(get_linkage(config.linkage).feet)
    r = config.params.link_radius
    feet = {}
    foot_links: dict[str, list[str]] = {}
    # per side, the joint names at the feet (Klann: F_leg0, ...): every link's capsule
    # stops short of them, whichever link ends there
    foot_points: dict[str, set[str]] = {}
    for body, joint in feet_of(robot):
        tag = "" if n_feet == 1 else f"_{joint}"
        foot = re.sub(r"^([LR])\.b\d+", rf"\g<1>.foot{tag}", body)
        feet[foot] = (body, joint, joints[body][joint])
        foot_points.setdefault(_side(body), set()).add(joint)
        foot_links[foot] = sorted(b.name for b in kinematic if is_link(b.name)
                                  and _side(b.name) == _side(body) and joint in joints[b.name])
    outlines = {}
    for b in kinematic:
        if not is_link(b.name):
            continue
        on_foot = foot_points.get(_side(b.name), set())
        segs = []
        for p, q in b.outline:
            a, c = joints[b.name][p], joints[b.name][q]
            u = (c - a) / np.linalg.norm(c - a) * 2.0 * r
            if np.linalg.norm(c - a) <= 2.0 * r * ((p in on_foot) + (q in on_foot)):
                continue                        # a stub shorter than its own trims
            segs.append((a + u if p in on_foot else a, c - u if q in on_foot else c))
        outlines[b.name] = segs
    hulled = [b for b in robot.bodies if b.part is not None and host[b.name] == "base" and (
        (b.fab == "laser" and body_class(b.name) in ("torso", "frame_outer"))
        or body_class(b.name) in ("servo", "deck_plate", "deck_battery", "deck_board"))]
    hulls = {b.name: ("base", pts) for b, pts in zip(
        hulled, _hulls([b.part for b in hulled], hull_tolerance), strict=True)}

    return RobotModel(
        config=config, bodies=ordered, host=host, joints=joints, loops=loops,
        outlines=outlines, feet=feet, foot_links=foot_links, hulls=hulls,
        link_radius=config.params.link_radius,
        crank_sign=crank_sign(config), meta=dict(robot.meta), servo=servo,
    )


# ---------------------------------------------------------------------------
# MJCF
# ---------------------------------------------------------------------------


def _f(x: float) -> str:
    return f"{float(x):.9g}"


def _v(xs) -> str:
    return " ".join(_f(x) for x in xs)


# world = Rx(+90°) · mech
_RX90 = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])

_RGBA = {
    "base": "0.95 0.45 0.15 0.6", "crank": "0.45 0.3 0.9 1", "link": "0.4 0.7 0.95 0.8",
    "foot": "0.1 0.1 0.1 1", "floor": "0.8 0.8 0.8 1",
}


def drive_limits(servo) -> tuple[float, float]:
    """``(max crank speed rad/s, stall torque N·m)`` of a servo."""
    if servo.speed_rpm is None or servo.torque_kgcm is None:
        raise ValueError(f"servo {servo.key!r} lacks speed_rpm / torque_kgcm")
    return servo.speed_rpm * RPM, servo.torque_kgcm * KGF_CM


def rated_torque(servo) -> float | None:
    """The servo's rated (continuous) torque in N·m, if its catalog entry has one."""
    kgcm = getattr(servo, "rated_kgcm", None)
    return None if kgcm is None else kgcm * KGF_CM


def rest_pose(rm: RobotModel, clearance: float) -> tuple[float, float]:
    """The base's pitch (rad, about mech ``z``) and height (m) at ``qpos0``.

    The robot stands on its feet at ``t_ref``: it is pitched so that the edge of
    the feet's lower hull beneath the centre of mass is level, with those feet
    ``clearance`` mm above the floor. With no such edge (every foot on one side
    of the centre of mass, as in the single module) it stays level, on its
    lowest foot.
    """
    pts = np.array([xy for *_, xy in rm.feet.values()], dtype=float)
    total = sum(mb.mass for mb in rm.bodies.values())
    xc = sum(mb.mass * mb.com[0] for mb in rm.bodies.values()) / total
    pitch = 0.0
    for i, j in itertools.permutations(range(len(pts)), 2):
        (xi, yi), (xj, yj) = pts[i], pts[j]
        if not xi < xc < xj:
            continue
        slope = (yj - yi) / (xj - xi)
        if np.all(pts[:, 1] - (yi + slope * (pts[:, 0] - xi)) >= -1e-9):
            pitch = -math.atan(slope)
            break
    heights = pts[:, 0] * math.sin(pitch) + pts[:, 1] * math.cos(pitch)
    return pitch, (clearance - heights.min() + rm.link_radius) * MM


DECK_FALLBACK_G = 113.0
"""Grams standing in for the electronics deck on a robot it doesn't fit (its measured
mass on the Strider double and the Klann quad: plate, rails, hardware, electronics)."""


def payload_g(rm: RobotModel, params: SimParams) -> float:
    """The point mass the base carries: :attr:`SimParams.payload_g`, plus
    :data:`DECK_FALLBACK_G` when the robot has no electronics deck."""
    fitted = rm.meta.get("deck", {}).get("fitted", False)
    return params.payload_g + (0.0 if fitted else DECK_FALLBACK_G)


def _with_payload(mb: MjBody, grams: float, pos) -> tuple[float, np.ndarray, np.ndarray]:
    """``mb``'s mass (kg), centre (mm) and inertia (kg·mm², about the new centre) with a
    point mass of ``grams`` at ``pos`` (mech mm) added: the payload on the base."""
    m2, p = grams * 1e-3, np.asarray(pos, dtype=float)
    mass = mb.mass + m2
    com = (mb.mass * mb.com + m2 * p) / mass
    inertia = mb.inertia.copy()                 # about mb.com; the parallel-axis shifts:
    for mi, ci in ((mb.mass, mb.com), (m2, p)):
        d = ci - com
        inertia += mi * (d @ d * np.eye(3) - np.outer(d, d))
    return mass, com, inertia


def _world_from_mech(pitch: float) -> tuple[np.ndarray, tuple[float, ...]]:
    """Rotation (3x3) and quaternion (wxyz) of the base at rest: ``Rx(90°) · Rz(pitch)``."""
    c, s = math.cos(pitch), math.sin(pitch)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    a, h = math.sqrt(0.5), 0.5 * pitch
    quat = (a * math.cos(h), a * math.cos(h), -a * math.sin(h), a * math.sin(h))
    return _RX90 @ rz, quat


def build_mjcf(config: BuildConfig | None = None,
               params: SimParams | None = None) -> tuple[str, dict]:
    """The MJCF document of the robot for ``config`` and its metadata (see module doc)."""
    return _build_mjcf(config or BuildConfig(), params or SimParams())


@lru_cache(maxsize=CACHE_SIZE)
def _build_mjcf(config: BuildConfig, params: SimParams) -> tuple[str, dict]:
    if params.crank_armature < MIN_CRANK_ARMATURE:
        raise ValueError(f"crank_armature {params.crank_armature:g} kg·m² is under "
                         f"{MIN_CRANK_ARMATURE:g}: the model is ill-posed there (loop closure "
                         "and roll blow up); the STS3215's reflected rotor inertia is estimated "
                         "at 5e-3")
    with build_lock(config):
        rm = robot_model(config, params.printed_fill, params.hull_tolerance)
    vmax, tau = drive_limits(rm.servo)
    kv = tau / (params.stall_error * vmax)
    r = rm.link_radius * MM
    pitch, height = rest_pose(rm, params.clearance)
    rot, quat = _world_from_mech(pitch)
    payload = payload_g(rm, params)

    root = ET.Element("mujoco", model=f"spiderpig_{rm.config.module}")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="false")
    ET.SubElement(root, "option", timestep=_f(params.timestep), integrator="implicitfast",
                  cone="elliptic", solver="Newton", iterations=str(params.iterations),
                  ls_iterations=str(params.ls_iterations), gravity="0 0 -9.81")
    ET.SubElement(root, "size", memory="16M")
    default = ET.SubElement(root, "default")
    friction = _v((params.friction, params.torsional_friction, params.rolling_friction))
    ET.SubElement(default, "geom", contype="1", conaffinity="0", condim="3", friction=friction,
                  solref=_v(params.contact_solref), solimp=_v(params.contact_solimp))
    ET.SubElement(default, "joint", type="hinge", axis="0 0 1", armature=_f(params.pin_armature),
                  damping=_f(params.pin_damping), frictionloss=_f(params.pin_frictionloss))
    ET.SubElement(default, "site", size="0.002", rgba="1 0 0 1")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")

    asset = ET.SubElement(root, "asset")
    for name, (_, pts) in rm.hulls.items():
        ET.SubElement(asset, "mesh", name=name, vertex=_v((pts * MM).ravel()))

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="sun", pos="0 0 2", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", contype="0",
                  conaffinity="1", friction=friction, rgba=_RGBA["floor"])

    elems: dict[str, ET.Element] = {}
    for name, mb in rm.bodies.items():
        if name == "base":
            el = ET.SubElement(world, "body", name="base", pos=_v((0, 0, height)),
                               quat=_v(quat))
            ET.SubElement(el, "freejoint", name="base")
            ET.SubElement(el, "site", name="base", pos="0 0 0")
        else:
            parent = rm.bodies[mb.parent]
            el = ET.SubElement(elems[mb.parent], "body", name=name,
                               pos=_v((mb.origin - parent.origin) * MM))
            if mb.kind == "crank":
                ET.SubElement(el, "joint", name=f"{mb.side}.crank",
                              axis=_v((0, 0, rm.crank_sign)),
                              armature=_f(params.crank_armature),
                              damping=_f(params.crank_damping))
            else:
                ET.SubElement(el, "joint", name=name)
        elems[name] = el
        mass, com, inertia = mb.mass, mb.com, mb.inertia
        if name == "base" and payload > 0:                     # what else rides the frame
            mass, com, inertia = _with_payload(mb, payload, params.payload_pos)
        i = inertia * 1e-6                                      # kg·m²
        ET.SubElement(el, "inertial", pos=_v((com - mb.origin) * MM), mass=_f(mass),
                      fullinertia=_v((i[0, 0], i[1, 1], i[2, 2], i[0, 1], i[0, 2], i[1, 2])))
        # collision geometry, in the body's own layer
        z = mb.z
        for k, (p, q) in enumerate(rm.outlines.get(name, ())):
            a = np.array([p[0], p[1], z]) - mb.origin
            b = np.array([q[0], q[1], z]) - mb.origin
            ET.SubElement(el, "geom", name=f"{name}#{k}", type="capsule", size=_f(r),
                          fromto=_v(np.concatenate([a, b]) * MM), rgba=_RGBA["link"])
        for foot, (link, _, xy) in rm.feet.items():
            if link == name:
                pos = _v((np.array([xy[0], xy[1], z]) - mb.origin) * MM)
                ET.SubElement(el, "geom", name=foot, type="sphere", size=_f(r), pos=pos,
                              condim="4", rgba=_RGBA["foot"])
                ET.SubElement(el, "site", name=foot, pos=pos)
        for part, (host, _) in rm.hulls.items():
            if host == name:
                ET.SubElement(el, "geom", name=part, type="mesh", mesh=part, rgba=_RGBA["base"])

    equality = ET.SubElement(root, "equality")
    for name, b1, b2, xy in rm.loops:
        m1, m2 = rm.bodies[b1], rm.bodies[b2]
        z = 0.5 * (m1.z + m2.z)                                 # between the two layers
        anchor = (np.array([xy[0], xy[1], z]) - m1.origin) * MM
        ET.SubElement(equality, "connect", name=name, body1=b1, body2=b2, anchor=_v(anchor),
                      solref=_v(params.eq_solref), solimp=_v(params.eq_solimp))

    actuator = ET.SubElement(root, "actuator")
    sides = sorted({mb.side for mb in rm.bodies.values() if mb.kind == "crank"})
    for s in sides:
        ET.SubElement(actuator, "velocity", name=f"{s}.drive", joint=f"{s}.crank", kv=_f(kv),
                      ctrlrange=_v((-vmax, vmax)), forcerange=_v((-tau, tau)))

    sensor = ET.SubElement(root, "sensor")
    for s in sides:
        ET.SubElement(sensor, "actuatorfrc", name=f"{s}.torque", actuator=f"{s}.drive")
        ET.SubElement(sensor, "jointvel", name=f"{s}.speed", joint=f"{s}.crank")
    ET.SubElement(sensor, "framepos", name="base.pos", objtype="site", objname="base")
    ET.SubElement(sensor, "framequat", name="base.quat", objtype="site", objname="base")

    ET.indent(root)
    xml = ET.tostring(root, encoding="unicode")
    return xml, _metadata(rm, params, height, pitch, rot, kv, vmax, tau, sides)


def _metadata(rm: RobotModel, params: SimParams, height: float, pitch: float, rot: np.ndarray,
              kv: float, vmax: float, tau: float, sides: list[str]) -> dict:
    mech_to_world = np.eye(4)
    mech_to_world[:3, :3] = rot * MM
    mech_to_world[2, 3] = height
    payload = payload_g(rm, params)
    total = sum(mb.mass for mb in rm.bodies.values()) + payload * 1e-3
    by_material: dict[str, float] = {}
    for mb in rm.bodies.values():
        for k, v in mb.mass_by.items():
            by_material[k] = by_material.get(k, 0.0) + v
    if payload > 0:
        by_material["payload"] = payload * 1e-3
    cfg = rm.config
    return {
        "format": "spiderpig-mjcf/1",
        "config": {"linkage": cfg.linkage, "module": cfg.module, "servo": cfg.servo,
                   "sheet": cfg.sheet,
                   "phases": list(rm.config.phases) if cfg.phases else None,
                   "proportions": dict(cfg.proportions)},
        "units": {"model": "SI (m, kg, s, rad)", "design": "mm"},
        "frames": {
            "world": "z up, x forward (positive drive), y left; floor z = 0, gravity -z",
            "mech": "design frame (mm): x walking axis, y up, z lateral stack "
                    "(left side z < 0); origin on the crank axis O at the mid-plane",
            "base": "the base body's frame is the mech frame in metres",
            # at qpos0 (includes the mm -> m scale); in general
            # p_world = base.xmat @ (p_mech_mm * 1e-3) + base.xpos
            "mech_to_world_ref": mech_to_world.tolist(),
            "rotation_world_from_mech": _RX90.tolist(),     # before the rest pitch
            "rest_pitch": pitch,                            # rad about mech z, nose up +
        },
        "t_ref": T_REF,
        "crank_sign": rm.crank_sign,
        "crank_angle": "t = t_ref + crank_sign * qpos[S.crank]",
        "base": "base",
        "freejoint": "base",
        "bodies": {
            name: {
                "kind": mb.kind, "side": mb.side, "parent": mb.parent, "pivot": mb.pivot,
                "kinematic": list(mb.kinematic),
                "ref_pos": (mb.origin * MM).tolist(),       # in the base frame at qpos0
                "mass": mb.mass,
            }
            for name, mb in rm.bodies.items()
        },
        # every robot / glb node -> MuJoCo body. A node moves by
        # D(t) = X_base_body(t) · X_base_body(qpos0)^-1 (mech frame; x1000 for mm)
        # applied to its frame-0 matrix; the glb root takes the base's world pose.
        "nodes": dict(rm.host),
        "node_motion": "node_matrix(t) = D_body(t) @ node_matrix(frame 0), "
                       "D_body(t) = X_base_body(t) @ inv(X_base_body(qpos0)) in mech mm",
        "actuators": {
            f"{s}.drive": {"joint": f"{s}.crank", "type": "velocity", "kv": kv,
                           "ctrlrange": [-vmax, vmax], "forcerange": [-tau, tau],
                           "ctrl_units": "rad/s crank speed, positive = forward",
                           "servo": rm.servo.key, "speed_rpm": rm.servo.speed_rpm,
                           "stall_torque_kgcm": rm.servo.torque_kgcm,
                           "rated_torque": rated_torque(rm.servo)}
            for s in sides
        },
        "feet": {foot: {"body": link, "joint": joint, "site": foot, "geom": foot,
                        "radius": rm.link_radius * MM, "links": rm.foot_links[foot]}
                 for foot, (link, joint, _) in rm.feet.items()},
        "loops": [{"name": n, "body1": b1, "body2": b2} for n, b1, b2, _ in rm.loops],
        "mass": {"total": total, "by_material": by_material},
        "params": {k: (list(v) if isinstance(v, tuple) else v) for k, v in asdict(params).items()},
        "rest_height": height,
    }


def load_model(config: BuildConfig | None = None, params: SimParams | None = None):
    """``(mujoco.MjModel, metadata)`` for ``config``, compiled once per config and params
    (the newest :data:`CACHE_SIZE` kept). The model is shared: copy it before changing it
    (:func:`motor_line` does, per session)."""
    config, params = config or BuildConfig(), params or SimParams()
    key = (config, params)
    if key not in _MODELS:
        xml, meta = _build_mjcf(config, params)
        adopt_mjcf(config, params, xml, meta)
    _MODELS.move_to_end(key)
    return _MODELS[key]


def cached_model(config: BuildConfig | None = None, params: SimParams | None = None):
    """:func:`load_model`'s answer when it is already compiled, else ``None`` (nothing is
    built)."""
    key = (config or BuildConfig(), params or SimParams())
    if key in _MODELS:
        _MODELS.move_to_end(key)
        return _MODELS[key]
    return None


def adopt_mjcf(config: BuildConfig, params: SimParams, xml: str, meta: dict):
    """Compile ``xml`` (``meta`` beside it: :func:`build_mjcf`'s, built in another process)
    as :func:`load_model`'s answer for ``config`` and ``params``; the ``(model, meta)``."""
    import mujoco

    key = (config, params)
    if key not in _MODELS:
        _remember(_MODELS, key, (mujoco.MjModel.from_xml_string(xml), meta))
    return _MODELS[key]


def motor_line(model, data, act, vmax: float, tau: float) -> None:
    """Narrow each drive's ``forcerange`` to the servo's speed-torque line at its speed
    now: motoring (torque with the motion) at most ``tau · (1 - |w| / vmax)``, braking
    the full ``tau``. Call before every ``mj_step`` on a copy of the shared model."""
    w = data.actuator_velocity[act]
    avail = tau * np.clip(1.0 - np.abs(w) / vmax, 0.0, None)
    lo = np.where(w < 0, -avail, -tau)
    hi = np.where(w > 0, avail, tau)
    model.actuator_forcerange[act, 0] = lo
    model.actuator_forcerange[act, 1] = hi
