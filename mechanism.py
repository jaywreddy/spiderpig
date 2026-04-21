"""Small kinematic-tree data model backed by pytransform3d.

A :class:`Mechanism` is a list of rigid :class:`Body` objects connected by
named :class:`Joint` pairs. ``solved()`` walks the connection graph from the
root body and propagates world poses so that connected joints coincide:

    T_world_child = T_world_parent @ P_parent_joint @ inv(P_child_joint)

The :class:`Pose` wrapper stores a 4x4 SE(3) matrix and offers constructors
for the ``(xyz, xyzw)`` tuples the legacy digifab code used. build123d is
imported lazily inside ``Pose.to_location`` so importing this module does not
drag in OCCT.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace

import numpy as np
from pytransform3d.transformations import (
    concat,
    invert_transform,
    transform_from_pq,
)


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
        xyz = np.asarray(xyz, dtype=float)
        m = np.eye(4)
        m[:3, 3] = xyz
        return cls(m)

    @classmethod
    def from_xyz_quat_xyzw(cls, xyz, quat_xyzw) -> Pose:
        """Build a Pose from legacy ``(xyz, (x, y, z, w))`` tuples.

        pytransform3d uses ``(w, x, y, z)``; reorder here.
        """
        xyz = np.asarray(xyz, dtype=float)
        x, y, z, w = quat_xyzw
        pq = np.array([xyz[0], xyz[1], xyz[2], w, x, y, z], dtype=float)
        return cls(transform_from_pq(pq))

    @classmethod
    def from_matrix(cls, matrix) -> Pose:
        return cls(np.asarray(matrix, dtype=float))

    def inverse(self) -> Pose:
        return Pose(invert_transform(self.matrix, check=False))

    def __matmul__(self, other: Pose) -> Pose:
        return Pose(concat(other.matrix, self.matrix))

    def to_location(self):
        from build123d import Location  # lazy import

        return Location(self.matrix.tolist())


@dataclass(frozen=True)
class Joint:
    """A named frame expressed in its body's local coordinates."""

    name: str
    pose: Pose = field(default_factory=Pose.identity)


@dataclass
class Body:
    """A rigid body carrying a build123d part, joints, and a world pose."""

    name: str
    part: object = None  # build123d Part/Compound or None for abstract bodies
    joints: list[Joint] = field(default_factory=list)
    color: str | None = None
    pose: Pose = field(default_factory=Pose.identity)

    def joint(self, name: str) -> Joint:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"Body {self.name!r} has no joint {name!r}")

    def placed_part(self):
        if self.part is None:
            return None
        return self.part.moved(self.pose.to_location())


# ((parent_index, parent_body, parent_joint), (child_index, child_body, child_joint))
Connection = tuple[tuple[int, str, str], tuple[int, str, str]]


@dataclass
class Mechanism:
    name: str
    bodies: list[Body] = field(default_factory=list)
    connections: list[Connection] = field(default_factory=list)

    def body(self, name: str) -> Body:
        for b in self.bodies:
            if b.name == name:
                return b
        raise KeyError(f"Mechanism has no body {name!r}")

    def solved(self) -> Mechanism:
        """Return a copy with world poses propagated from ``bodies[0]``.

        BFS over the undirected connection graph. The first body's current
        pose is taken as the world anchor; every body reached via a connection
        is placed so its joint coincides with the already-placed parent joint.
        """
        if not self.bodies:
            return replace(self)

        placed = [replace(b) for b in self.bodies]
        by_name = {b.name: i for i, b in enumerate(placed)}

        # adjacency: name -> list of (neighbour_name, joint_here, joint_there)
        adj: dict[str, list[tuple[str, str, str]]] = {b.name: [] for b in placed}
        for (_, a_name, a_joint), (_, b_name, b_joint) in self.connections:
            adj[a_name].append((b_name, a_joint, b_joint))
            adj[b_name].append((a_name, b_joint, a_joint))

        visited = {placed[0].name}
        q: deque[str] = deque([placed[0].name])
        while q:
            cur = q.popleft()
            cur_body = placed[by_name[cur]]
            for other, j_here, j_there in adj[cur]:
                if other in visited:
                    continue
                other_body = placed[by_name[other]]
                p_here = cur_body.joint(j_here).pose
                p_there = other_body.joint(j_there).pose
                other_body.pose = cur_body.pose @ p_here @ p_there.inverse()
                visited.add(other)
                q.append(other)

        return Mechanism(name=self.name, bodies=placed, connections=list(self.connections))

    def to_compound(self):
        from build123d import Compound  # lazy import

        parts = []
        for b in self.bodies:
            placed = b.placed_part()
            if placed is None:
                continue
            if b.color is not None:
                placed.label = b.name
                placed.color = b.color
            else:
                placed.label = b.name
            parts.append(placed)
        return Compound(children=parts, label=self.name)

    def export_step(self, path) -> None:
        from build123d import export_step  # lazy import

        export_step(self.to_compound(), str(path))

    def export_stl(self, path) -> None:
        from build123d import export_stl  # lazy import

        export_stl(self.to_compound(), str(path))

    def save_layouts(self, prefix) -> None:
        from layout import save_sheets  # lazy import

        save_sheets(self, prefix)
