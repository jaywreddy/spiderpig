"""Kinematic-tree data model: rigid bodies, named joints and 4x4 poses (numpy).

A :class:`Mechanism` is a list of rigid :class:`Body` objects connected by
named :class:`Joint` pairs, at one crank angle; a :class:`MechanismTemplate`
is the same assembly with every joint's pose a function of time, sampled in
one batch (:meth:`MechanismTemplate.sample`) and projected to a
:class:`Mechanism` with :meth:`MechanismTemplate.freeze_at`. build123d is
imported lazily inside ``Pose.to_location`` so importing this module does
not drag in OCCT.
"""

from __future__ import annotations

import re
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

# -- body names (the template's: :mod:`linkage.assembly`; the planner reads them) ----------


def body_class(name: str) -> str:
    """``"b1_leg3"`` -> ``"b1"``, ``"R.b1_leg3"`` -> ``"b1"``; ``"conn_upper"`` stays."""
    return re.sub(r"_leg\d+$", "", re.sub(r"^[LR]\.", "", name))


def is_link(name: str) -> bool:
    """A leg link: every linkage names its links ``b<k>`` (see :mod:`linkage`)."""
    return re.fullmatch(r"b\d+", body_class(name)) is not None


def is_crank(name: str) -> bool:
    return body_class(name).startswith("conn")


def is_frame(name: str) -> bool:
    return body_class(name) == "torso"


@dataclass(frozen=True)
class Pose:
    """Rigid 4x4 SE(3) transform."""

    matrix: np.ndarray

    def __post_init__(self) -> None:
        m = np.asarray(self.matrix, dtype=float)
        if m.shape != (4, 4):
            raise ValueError(f"Pose matrix must be 4x4, got {m.shape}")
        object.__setattr__(self, "matrix", m)

    @classmethod
    def identity(cls) -> Pose:
        return cls(np.eye(4))

    @classmethod
    def from_translation(cls, xyz) -> Pose:
        m = np.eye(4)
        m[:3, 3] = np.asarray(xyz, dtype=float)
        return cls(m)

    @classmethod
    def from_matrix(cls, matrix) -> Pose:
        return cls(np.asarray(matrix, dtype=float))

    def inverse(self) -> Pose:
        return Pose(_invert_se3_batch(self.matrix))

    def __matmul__(self, other: Pose) -> Pose:
        return Pose(self.matrix @ other.matrix)

    def to_location(self):
        from build123d import Location, Plane  # lazy import

        origin = tuple(self.matrix[:3, 3])
        x_dir = tuple(self.matrix[:3, 0])
        z_dir = tuple(self.matrix[:3, 2])
        return Location(Plane(origin, x_dir, z_dir))


@dataclass(frozen=True)
class Joint:
    """A named frame expressed in its body's local coordinates."""

    name: str
    pose: Pose = field(default_factory=Pose.identity)


# A link's physical skeleton: the pairs of its own joints its outline spans.
Outline = tuple[tuple[str, str], ...]


@dataclass
class Body:
    """A rigid body carrying a build123d part, joints, and a world pose.

    ``outline`` names the joint pairs the body's laser-cut shape spans (empty
    for bodies that aren't links). ``rigid_with`` marks fabricated hardware
    that moves with another body; such a body has no joints of its own and
    its part is modelled in the same frame as its host's. ``fab`` says how
    the part is made (``"laser"``, ``"printed"`` or ``"purchased"``); a
    purchased body names its catalog item in ``bom_key``.
    """

    name: str
    part: object = None  # build123d Part/Compound or None for abstract bodies
    joints: list[Joint] = field(default_factory=list)
    color: str | None = None
    pose: Pose = field(default_factory=Pose.identity)
    outline: Outline = ()
    rigid_with: str | None = None
    fab: str | None = None
    bom_key: str | None = None
    sheet: str | None = None     # a laser-cut part's sheet (catalog key; None: the default)

    def joint(self, name: str) -> Joint:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"Body {self.name!r} has no joint {name!r}")

    def placed_part(self):
        if self.part is None:
            return None
        # build123d's moved, without the copy it discards
        from spiderpig.shapes import moved

        return moved(self.part, self.pose.to_location())


# ((parent_index, parent_body, parent_joint), (child_index, child_body, child_joint))
Connection = tuple[tuple[int, str, str], tuple[int, str, str]]


@dataclass
class Mechanism:
    """Bodies joined by connections.

    ``meta`` carries build settings for downstream writers (sheet material,
    servo, joinery); ``bom_extras`` holds purchases that aren't modelled as
    bodies (``hardware.bom.BomLine``).
    """

    name: str
    bodies: list[Body] = field(default_factory=list)
    connections: list[Connection] = field(default_factory=list)
    meta: dict = field(default_factory=dict)
    bom_extras: list = field(default_factory=list)

    def body(self, name: str) -> Body:
        for b in self.bodies:
            if b.name == name:
                return b
        raise KeyError(f"Mechanism has no body {name!r}")

    def to_compound(self):
        from build123d import Color, Compound  # lazy import

        parts = []
        for b in self.bodies:
            placed = b.placed_part()
            if placed is None:
                continue
            placed.label = b.name
            if b.color is not None:
                placed.color = Color(b.color)
            parts.append(placed)
        return Compound(children=parts, label=self.name)

    def export_step(self, path) -> None:
        from build123d import export_step  # lazy import

        export_step(self.to_compound(), str(path))

    def export_stl(self, path) -> None:
        from spiderpig.mesh import export_stl  # lazy import

        export_stl(self.to_compound(), path)


# ---------------------------------------------------------------------------
# Template layer: parametric-in-time mechanism.
#
# A :class:`MechanismTemplate` carries the static topology of an assembly
# (body names, colors, connections, build123d parts for tessellation) plus
# *callables* that produce per-joint poses for any batch of sample times ``t``.
# Downstream bakes call :meth:`MechanismTemplate.sample` once over the whole
# ``ts`` array instead of rebuilding the symbolic geometry per frame.
# ---------------------------------------------------------------------------


# ``pose_at`` signature: (ts: (T,) float ndarray) -> (T, 4, 4) SE(3) tensor.
JointPoseFn = Callable[[np.ndarray], np.ndarray]


def _identity_pose_batch(n: int) -> np.ndarray:
    """Return ``(n, 4, 4)`` identity SE(3) tensor."""
    return np.broadcast_to(np.eye(4), (n, 4, 4)).copy()


def _invert_se3_batch(m: np.ndarray) -> np.ndarray:
    """Closed-form inverse of ``(..., 4, 4)`` SE(3) tensors: ``[R | t]^-1 = [R^T | -R^T t]``."""
    r = m[..., :3, :3]
    t = m[..., :3, 3]
    r_t = np.swapaxes(r, -1, -2)
    inv_t = -np.einsum("...ij,...j->...i", r_t, t)
    out = np.zeros_like(m)
    out[..., :3, :3] = r_t
    out[..., :3, 3] = inv_t
    out[..., 3, 3] = 1.0
    return out


def _compose_se3_batch(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return ``a @ b`` elementwise over ``(T, 4, 4)`` batches."""
    return np.einsum("tij,tjk->tik", a, b)


@dataclass(frozen=True)
class JointTemplate:
    """Named joint whose pose is a pure function of sample time ``t``."""

    name: str
    pose_at: JointPoseFn

    def eval(self, ts: np.ndarray) -> np.ndarray:
        """Return ``(T, 4, 4)`` SE(3) tensor of joint poses over ``ts``."""
        out = np.asarray(self.pose_at(ts))
        if out.shape[-2:] != (4, 4):
            raise ValueError(
                f"JointTemplate {self.name!r} pose_at returned shape {out.shape}, "
                f"expected (..., 4, 4)"
            )
        return out


@dataclass
class BodyTemplate:
    """Rigid body: static metadata + list of time-parametric joints.

    ``base_pose`` seeds the BFS root of a connected component so leg-stacked
    assemblies can pin each leg at its own ``z_base``.

    ``part`` is the build123d part (if any) carried through for the one-shot
    reference build and tessellation. Templates used only for the frame loop
    leave this ``None``.
    """

    name: str
    joints: list[JointTemplate] = field(default_factory=list)
    color: str | None = None
    base_pose: Pose = field(default_factory=Pose.identity)
    part: object = None
    outline: Outline = ()

    def joint(self, name: str) -> JointTemplate:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"BodyTemplate {self.name!r} has no joint {name!r}")


@dataclass
class SampledPoses:
    """World-pose tensors produced by :meth:`MechanismTemplate.sample`."""

    body_names: list[str]
    ts: np.ndarray                        # (T,) float
    world: dict[str, np.ndarray]          # name → (T, 4, 4)
    joint_world: dict[str, dict[str, np.ndarray]]
    """Per-body joint world positions. ``joint_world[body][joint]`` has shape
    ``(T, 3)`` and is the world-space XYZ of that joint over every sample.
    Precomputed once during ``sample()`` so the Procrustes pass doesn't
    re-multiply joint poses.
    """


@dataclass
class MechanismTemplate:
    """Parametric-in-time analogue of :class:`Mechanism`."""

    name: str
    bodies: list[BodyTemplate] = field(default_factory=list)
    connections: list[Connection] = field(default_factory=list)
    meta: dict = field(default_factory=dict)   # design parameters (module, phases, proportions)

    def body(self, name: str) -> BodyTemplate:
        for b in self.bodies:
            if b.name == name:
                return b
        raise KeyError(f"MechanismTemplate has no body {name!r}")

    def sample(self, ts: np.ndarray) -> SampledPoses:
        """Vectorized pose propagation over every ``t`` in ``ts``.

        Each joint's ``pose_at`` is called once on the whole ``ts`` array, then
        a BFS over the undirected connection graph composes world poses per
        connected component (seeded from its first body's ``base_pose``) so
        that connected joints coincide:

            T_world_child = T_world_parent @ P_parent_joint @ inv(P_child_joint)

        on ``(T, 4, 4)`` tensors, with batched SE(3) multiply + inverse.
        """
        ts = np.asarray(ts, dtype=float)
        if ts.ndim != 1:
            raise ValueError(f"ts must be 1-D, got shape {ts.shape}")
        n = ts.shape[0]

        if not self.bodies:
            return SampledPoses(
                body_names=[], ts=ts, world={}, joint_world={}
            )

        # Pre-evaluate every joint.pose_at(ts). Shape per joint: (T, 4, 4).
        joint_local: dict[str, dict[str, np.ndarray]] = {
            b.name: {j.name: j.eval(ts) for j in b.joints} for b in self.bodies
        }

        # Adjacency: name -> list of (neighbour_name, joint_here, joint_there).
        adj: dict[str, list[tuple[str, str, str]]] = {b.name: [] for b in self.bodies}
        for (_, a_name, a_joint), (_, b_name, b_joint) in self.connections:
            adj[a_name].append((b_name, a_joint, b_joint))
            adj[b_name].append((a_name, b_joint, a_joint))

        world: dict[str, np.ndarray] = {}
        visited: set[str] = set()
        id4 = _identity_pose_batch(n)
        for root in self.bodies:
            if root.name in visited:
                continue
            # Seed the component with the root's static base pose broadcast
            # over T so BFS composes cleanly.
            seed = id4.copy()
            seed[:] = root.base_pose.matrix
            world[root.name] = seed
            visited.add(root.name)
            q: deque[str] = deque([root.name])
            while q:
                cur = q.popleft()
                cur_world = world[cur]
                for other, j_here, j_there in adj[cur]:
                    if other in visited:
                        continue
                    p_here = joint_local[cur][j_here]
                    p_there = joint_local[other][j_there]
                    world[other] = _compose_se3_batch(
                        _compose_se3_batch(cur_world, p_here),
                        _invert_se3_batch(p_there),
                    )
                    visited.add(other)
                    q.append(other)

        # World-space XYZ of every joint under its owning body's world pose.
        joint_world: dict[str, dict[str, np.ndarray]] = {}
        for b in self.bodies:
            w = world[b.name]                       # (T, 4, 4)
            jw: dict[str, np.ndarray] = {}
            for j in b.joints:
                p = joint_local[b.name][j.name]      # (T, 4, 4)
                # world joint pose = body_world @ joint_local; take translation.
                composed = _compose_se3_batch(w, p)
                jw[j.name] = composed[:, :3, 3]       # (T, 3)
            joint_world[b.name] = jw

        return SampledPoses(
            body_names=[b.name for b in self.bodies],
            ts=ts,
            world=world,
            joint_world=joint_world,
        )

    def freeze_at(self, t: float) -> Mechanism:
        """Single-t projection back to the concrete :class:`Mechanism` type.

        Every single-t consumer (fabrication, the CLI build, tests) works on
        this projection of the template, so the two can't drift apart.
        """
        ts = np.array([float(t)], dtype=float)
        bodies: list[Body] = []
        for b in self.bodies:
            joints = [
                Joint(name=j.name, pose=Pose.from_matrix(j.eval(ts)[0]))
                for j in b.joints
            ]
            bodies.append(
                Body(
                    name=b.name,
                    part=b.part,
                    joints=joints,
                    color=b.color,
                    pose=b.base_pose,
                    outline=b.outline,
                )
            )
        return Mechanism(name=self.name, bodies=bodies, connections=list(self.connections),
                         meta=dict(self.meta))


def translation_pose_at(
    xy_fn: Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]],
    z: float,
) -> JointPoseFn:
    """Build a ``pose_at`` callable from a 2D ``(ts,) -> (xs, ys)`` function.

    ``linkage.LegSolution.callables`` returns numpy callables of this shape
    for each named point. ``z`` is the fixed joint layer offset (every joint
    is planar: bodies rotate only about Z).
    """

    def _fn(ts: np.ndarray) -> np.ndarray:
        ts = np.asarray(ts, dtype=float)
        xs, ys = xy_fn(ts)
        xs = np.broadcast_to(np.asarray(xs, dtype=float), ts.shape)
        ys = np.broadcast_to(np.asarray(ys, dtype=float), ts.shape)
        out = np.broadcast_to(np.eye(4), (ts.shape[0], 4, 4)).copy()
        out[:, 0, 3] = xs
        out[:, 1, 3] = ys
        out[:, 2, 3] = z
        return out

    return _fn
