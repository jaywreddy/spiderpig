"""MuJoCo model (MJCF) of the fabricated walker.

:func:`build_mjcf` turns a :class:`fabricate.BuildConfig` into one
self-contained MJCF document (no external files) plus a JSON-able metadata
dict. The same XML runs in Python (``mujoco``) and in the browser (the
official WebAssembly bindings); the metadata tells the viewer how to put the
simulated poses on the glb nodes of :mod:`viewer.bake_gltf`.

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
  resulting ``mech_to_world_ref``.

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
off: geoms carry no mass). Densities (g/cm³): laser-cut sheet
:data:`SHEET_DENSITY` (cast acrylic 1.19); printed parts the robot's filament
from the catalog (PLA 1.24) times :attr:`SimParams.printed_fill` (1.0 = the
BOM's 100 % infill, an upper bound); screws and nuts steel 7.85; heat-set
inserts brass 8.5; an aluminium horn 2.70; the servo its datasheet mass
(``ServoSpec.weight_g``, 55 g for the STS3215) spread uniformly over its
modelled case.

Collision geometry
------------------
Simple shapes; only the robot and the floor collide (robot geoms
``contype=1 conaffinity=0``, floor ``contype=0 conaffinity=1``: the layer
planner already guarantees robot parts never meet each other).

* feet: a sphere of the link radius at each of the linkage's foot tips (the
  rounded end of the foot link, Klann's ``b4.F``; the plate's 3 mm width is
  ignored), geom and site ``S.foot<suffix>`` (``S.foot_<joint><suffix>``
  when a leg has several feet);
* links: capsules along their outline at their layer, link radius; a foot
  link's capsule stops one radius short of the foot so the foot sphere alone
  makes the foot contact;
* base: the convex hull of each frame plate and servo (so a fall or tip
  shows).

Friction is one coefficient for everything on the floor
(:attr:`SimParams.friction`, 0.5: acrylic or PLA on a hard floor, dry;
torsional and rolling friction small). Contacts use ``condim=3``, elliptic
cones, and a moderately soft contact (``solref 0.01 1``, MuJoCo's default
``solimp 0.9 0.95 0.001``): about 0.4 mm of static give and 1-1.5 mm at
impacts, standing in for the leg's compliance and the play of its running
fits (an acrylic foot on a hard floor alone is stiffer). Speed, stride and
mean drive torque don't depend on it; the impact torque peaks, foot slip
and the number of feet down do (stiffer: higher peaks, fewer feet down).

Drives and joints
-----------------
One ``velocity`` actuator per side on its crank hinge (``L.drive``,
``R.drive``): ``ctrl`` is the crank speed in rad/s (positive = forward),
``ctrlrange`` ± the servo's no-load speed (STS3215: 52 rpm = 5.45 rad/s),
``forcerange`` ± its stall torque (19.5 kg·cm = 1.91 N·m), and ``kv`` such
that the stall torque is reached at a speed error of
:attr:`SimParams.stall_error` (10 %) of the no-load speed, a stiff speed
loop like the servo's own. The actuator force is then the torque on the
output shaft, directly comparable with the servo's ratings. The crank hinge
carries the gear train's reflected rotor inertia as ``armature`` and no extra
damping: the servo's own losses are inside its speed and torque ratings.
Passive pins carry a tiny armature and damping (numerical regularization).
:func:`sim.run.walk_metrics` also checks the torque against a DC motor's
speed-torque line, which a velocity actuator doesn't enforce.

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
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field, replace
from functools import cache

import numpy as np

from fabricate import BuildConfig, fabricate, template_for
from linkage import feet_of
from linkage import get as get_linkage
from stack import body_class, is_crank, is_frame, is_link

T_REF = 0.0                     # crank angle the model's qpos0 is at (the glb's frame 0)
MM = 1e-3                       # m per mm
KGF_CM = 9.80665e-2             # N·m per kgf·cm
RPM = 2.0 * math.pi / 60.0      # rad/s per rpm

# Densities (g/cm³) of what the catalog doesn't carry.
SHEET_DENSITY = {
    # cast PMMA: 1.19 g/cm³ (ISO 1183; e.g. Röhm PLEXIGLAS GS technical data, 1.19)
    "acrylic_3mm": 1.19,
    # Baltic birch plywood: about 0.68 g/cm³ (typical 650-720 kg/m³ for birch plywood)
    "plywood_3mm": 0.68,
}
STEEL_DENSITY = 7.85            # carbon / alloy steel fasteners (EN 10025 / ISO 898 steels)
BRASS_DENSITY = 8.5             # heat-set inserts (CuZn39Pb3 free-cutting brass: 8.47)
ALUMINIUM_DENSITY = 2.70        # 6061-T6 (ASM handbook: 2.70 g/cm³)
POM_DENSITY = 1.41              # a plastic (acetal) horn, when the horn isn't aluminium



@dataclass(frozen=True)
class SimParams:
    """Physical and numerical parameters of the model (SI unless named otherwise)."""

    friction: float = 0.5           # sliding friction robot / floor (acrylic or PLA, dry)
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
    pin_damping: float = 1e-4       # N·m·s/rad, passive pins
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
    feet: dict[str, tuple[str, np.ndarray]]   # foot name -> (foot link, foot xy mm)
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


def _part_props(part) -> tuple[float, np.ndarray, np.ndarray]:
    """``(volume mm³, centroid mm, inertia about the centroid at unit density mm⁵)``."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(part.wrapped, g)
    c = g.CentreOfMass()
    m = g.MatrixOfInertia()
    inertia = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return g.Mass(), np.array([c.X(), c.Y(), c.Z()]), inertia


def _material(body, config: BuildConfig, meta: dict,
              servo) -> tuple[str, float | None, float | None]:
    """``(material, density g/cm³, fixed mass g)`` of a fabricated body."""
    from hardware.catalog import get

    cls = body_class(body.name)
    if body.fab == "laser":
        if config.sheet not in SHEET_DENSITY:
            raise ValueError(f"no density for sheet {config.sheet!r}; add it to SHEET_DENSITY")
        return "sheet", SHEET_DENSITY[config.sheet], None
    if body.fab == "printed":
        filament = meta.get("filament") or "pla_filament"
        density = float(get(filament).dims["density"])
        return "printed", density, None
    if body.fab == "purchased":
        category = None
        if body.bom_key:
            try:
                category = get(body.bom_key).category
            except KeyError:
                category = None
        if category == "servo" or cls == "servo":
            if servo.weight_g is None:
                raise ValueError(f"servo {servo.key!r} has no weight_g")
            return "servo", None, float(servo.weight_g)
        if cls.startswith("servo_horn") or category == "horn":
            alu = "alumin" in servo.horn.name.lower()
            return ("aluminium", ALUMINIUM_DENSITY, None) if alu else ("plastic", POM_DENSITY, None)
        if body.bom_key and "insert" in body.bom_key:
            return "brass", BRASS_DENSITY, None
        return "steel", STEEL_DENSITY, None
    raise ValueError(f"body {body.name!r} has a part but no known fabrication ({body.fab!r})")


def _hull(part, tolerance: float) -> np.ndarray:
    from scipy.spatial import ConvexHull

    verts, _ = part.tessellate(tolerance)
    pts = np.unique(np.round(np.array([(v.X, v.Y, v.Z) for v in verts]), 4), axis=0)
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


@cache
def fabricated(config: BuildConfig):
    """The fabricated robot (both sides) at ``t_ref`` (cached per config)."""
    config = replace(config, robot=True)
    return fabricate(template_for(config), config, T_REF)


@cache
def robot_model(config: BuildConfig, printed_fill: float = 1.0,
                hull_tolerance: float = 0.5) -> RobotModel:
    """The fabricated robot at ``t_ref`` reduced to MuJoCo bodies (cached per config)."""
    import servos

    config = replace(config, robot=True)
    robot = fabricated(config)
    servo = servos.get(config.servo)
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
        material, density, fixed_g = _material(b, config, robot.meta, servo)
        if material == "printed":
            density *= printed_fill
        vol, com, inertia = _part_props(b.part)
        if vol <= 0:
            raise ValueError(f"part of {b.name!r} has no volume")
        rho = (fixed_g / vol) if fixed_g is not None else density * 1e-3   # g/mm³
        acc[mb.name].append((rho * vol * 1e-3, com, rho * inertia * 1e-3))  # kg, mm, kg·mm²
        mb.mass_by[material] = mb.mass_by.get(material, 0.0) + rho * vol * 1e-3
        bb = b.part.bounding_box()
        mb.z_range = (min(mb.z_range[0], bb.min.Z), max(mb.z_range[1], bb.max.Z))
        if b.name in mb.kinematic:
            mb.layer_z = 0.5 * (bb.min.Z + bb.max.Z)
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

    # collision geometry: link capsules (a foot link's stops a radius short of the
    # foot: the foot sphere alone makes the contact) and foot spheres
    n_feet = len(get_linkage(config.linkage).feet)
    r = config.params.link_radius
    feet = {}
    foot_joints: dict[str, set[str]] = {}
    for body, joint in feet_of(robot):
        tag = "" if n_feet == 1 else f"_{joint}"
        feet[re.sub(r"^([LR])\.b\d+", rf"\g<1>.foot{tag}", body)] = (body, joints[body][joint])
        foot_joints.setdefault(body, set()).add(joint)
    outlines = {}
    for b in kinematic:
        if not is_link(b.name):
            continue
        on_foot = foot_joints.get(b.name, set())
        segs = []
        for p, q in b.outline:
            a, c = joints[b.name][p], joints[b.name][q]
            u = (c - a) / np.linalg.norm(c - a) * r
            segs.append((a + u if p in on_foot else a, c - u if q in on_foot else c))
        outlines[b.name] = segs
    hulls = {}
    for b in robot.bodies:
        if b.part is None or host[b.name] != "base":
            continue
        cls = body_class(b.name)
        if (b.fab == "laser" and cls in ("torso", "frame_outer")) or cls == "servo":
            hulls[b.name] = ("base", _hull(b.part, hull_tolerance))

    return RobotModel(
        config=config, bodies=ordered, host=host, joints=joints, loops=loops,
        outlines=outlines, feet=feet, hulls=hulls, link_radius=config.params.link_radius,
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


def rest_pose(rm: RobotModel, clearance: float) -> tuple[float, float]:
    """The base's pitch (rad, about mech ``z``) and height (m) at ``qpos0``.

    The robot stands on its feet at ``t_ref``: it is pitched so that the edge of
    the feet's lower hull beneath the centre of mass is level, with those feet
    ``clearance`` mm above the floor. With no such edge (every foot on one side
    of the centre of mass, as in the single module) it stays level, on its
    lowest foot.
    """
    pts = np.array([xy for _, xy in rm.feet.values()], dtype=float)
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


@cache
def _build_mjcf(config: BuildConfig, params: SimParams) -> tuple[str, dict]:
    rm = robot_model(config, params.printed_fill, params.hull_tolerance)
    vmax, tau = drive_limits(rm.servo)
    kv = tau / (params.stall_error * vmax)
    r = rm.link_radius * MM
    pitch, height = rest_pose(rm, params.clearance)
    rot, quat = _world_from_mech(pitch)

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
                  damping=_f(params.pin_damping))
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
        i = mb.inertia * 1e-6                                   # kg·m²
        ET.SubElement(el, "inertial", pos=_v((mb.com - mb.origin) * MM), mass=_f(mb.mass),
                      fullinertia=_v((i[0, 0], i[1, 1], i[2, 2], i[0, 1], i[0, 2], i[1, 2])))
        # collision geometry, in the body's own layer
        z = mb.z
        for k, (p, q) in enumerate(rm.outlines.get(name, ())):
            a = np.array([p[0], p[1], z]) - mb.origin
            b = np.array([q[0], q[1], z]) - mb.origin
            ET.SubElement(el, "geom", name=f"{name}#{k}", type="capsule", size=_f(r),
                          fromto=_v(np.concatenate([a, b]) * MM), rgba=_RGBA["link"])
        for foot, (link, xy) in rm.feet.items():
            if link == name:
                pos = _v((np.array([xy[0], xy[1], z]) - mb.origin) * MM)
                ET.SubElement(el, "geom", name=foot, type="sphere", size=_f(r), pos=pos,
                              rgba=_RGBA["foot"])
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
    total = sum(mb.mass for mb in rm.bodies.values())
    by_material: dict[str, float] = {}
    for mb in rm.bodies.values():
        for k, v in mb.mass_by.items():
            by_material[k] = by_material.get(k, 0.0) + v
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
                           "stall_torque_kgcm": rm.servo.torque_kgcm}
            for s in sides
        },
        "feet": {foot: {"body": link, "site": foot, "geom": foot, "radius": rm.link_radius * MM}
                 for foot, (link, _) in rm.feet.items()},
        "loops": [{"name": n, "body1": b1, "body2": b2} for n, b1, b2, _ in rm.loops],
        "mass": {"total": total, "by_material": by_material},
        "params": {k: (list(v) if isinstance(v, tuple) else v) for k, v in asdict(params).items()},
        "rest_height": height,
    }


def load_model(config: BuildConfig | None = None, params: SimParams | None = None):
    """``(mujoco.MjModel, metadata)`` for ``config``."""
    import mujoco

    xml, meta = build_mjcf(config, params)
    return mujoco.MjModel.from_xml_string(xml), meta


__all__ = [
    "MM", "RPM", "SHEET_DENSITY", "T_REF", "MjBody", "RobotModel", "SimParams", "build_mjcf",
    "crank_sign", "drive_limits", "fabricated", "load_model", "rest_pose", "robot_model",
]
