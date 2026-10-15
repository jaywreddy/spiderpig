"""Spatial joints over a planar side, and the poses they make.

A **spatial joint** turns what it carries about an axis given at the reference pose (the
pose the robot was fabricated at, every spatial joint at zero), in robot coordinates (the
side's: x forward, y up, z across the stack; mm). It rides its ``parent`` joint (``None``:
the world), after its ``host``'s planar motion (``None``: none): an arm's turntable ``q1``
rides the world, about the vertical through its bearing; its wrist ``q4`` rides ``q1``
after the end plate's planar motion, about the vertical through the tool flange.

A body's pose at a time is then, innermost last::

    T(body) = T(joint) @ P(anchor) @ G          when the joint has no host
    T(body) = T(joint) @ G                       when it has one (its host is the anchor)
    T(joint) = T(parent) @ P(host) @ R(joint)

with ``P`` a body's planar motion from the reference pose (:func:`planar_matrix`), ``R`` the
joint's rotation about its axis (:func:`rigid_about`) and ``G`` whatever places the body's
mesh. :attr:`RobotTree.carried` names each body's innermost joint; a body it doesn't name
moves with its planar anchor alone (the world's, for a static one).

The animation's sweep (:meth:`RobotTree.sweep`) turns each joint over its range,
``mid + half sin(k t)`` over one tour of the planar inputs (``k`` the joint's depth: the
wrist turns twice per tour), so a bake shows every axis move.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class SpatialJoint:
    """A revolute joint about ``axis`` through ``origin`` (robot coordinates at the
    reference pose), over ``range_deg`` from the reference; it rides ``parent`` (a joint's
    name, ``None``: the world) after ``host``'s planar motion (a body's name, ``None``)."""

    name: str
    axis: tuple[float, float, float]
    origin: tuple[float, float, float]
    range_deg: tuple[float, float]
    parent: str | None = None
    host: str | None = None
    label: str = ""

    def __post_init__(self):
        n = math.sqrt(sum(a * a for a in self.axis))
        if n < 1e-12:
            raise ValueError(f"joint {self.name}: a zero axis")
        object.__setattr__(self, "axis", tuple(float(a) / n for a in self.axis))
        object.__setattr__(self, "origin", tuple(float(v) for v in self.origin))
        lo, hi = (float(v) for v in self.range_deg)
        if not lo <= 0.0 <= hi:
            raise ValueError(f"joint {self.name}: its range {lo:g}..{hi:g} deg must hold the "
                             "reference pose (0)")
        object.__setattr__(self, "range_deg", (lo, hi))

    def as_dict(self) -> dict:
        return {"name": self.name, "axis": list(self.axis), "origin": list(self.origin),
                "range_deg": list(self.range_deg), "parent": self.parent, "host": self.host,
                "label": self.label}

    @classmethod
    def from_dict(cls, d: Mapping) -> SpatialJoint:
        return cls(str(d["name"]), tuple(d["axis"]), tuple(d["origin"]),  # type: ignore[arg-type]
                   tuple(d["range_deg"]), d.get("parent"), d.get("host"),  # type: ignore[arg-type]
                   str(d.get("label", "")))


@dataclass(frozen=True)
class RobotTree:
    """A robot's spatial joints (parents before children) and the bodies each carries
    (``carried``: body -> its innermost joint)."""

    joints: tuple[SpatialJoint, ...]
    carried: dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        seen: set[str] = set()
        for j in self.joints:
            if j.parent is not None and j.parent not in seen:
                raise ValueError(f"joint {j.name}: its parent {j.parent!r} must come first")
            if j.name in seen:
                raise ValueError(f"joint {j.name} twice")
            seen.add(j.name)
        bad = sorted({v for v in self.carried.values() if v not in seen})
        if bad:
            raise ValueError(f"bodies carried by unknown joints {bad}")

    def joint(self, name: str) -> SpatialJoint:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"no spatial joint {name!r}; have {[j.name for j in self.joints]}")

    def chain(self, name: str) -> list[SpatialJoint]:
        """The joints from the world to ``name``, outermost first."""
        out = []
        cur: str | None = name
        while cur is not None:
            j = self.joint(cur)
            out.append(j)
            cur = j.parent
        return out[::-1]

    def depth(self, name: str) -> int:
        return len(self.chain(name))

    def sweep(self, ts: np.ndarray) -> dict[str, np.ndarray]:
        """Each joint's angle (rad) over ``ts`` (one tour: 0..2 pi): its range's middle
        plus half its range times ``sin(depth t)``."""
        ts = np.asarray(ts, dtype=float)
        out = {}
        for j in self.joints:
            lo, hi = (math.radians(v) for v in j.range_deg)
            out[j.name] = (lo + hi) / 2 + (hi - lo) / 2 * np.sin(self.depth(j.name) * ts)
        return out

    def joint_transforms(self, angles: Mapping[str, np.ndarray],
                         planar: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        """``T(joint)`` (``(T, 4, 4)``) of every joint, ``angles`` each joint's (rad) and
        ``planar`` each host's planar motion (``(T, 4, 4)``: :func:`planar_matrix`)."""
        out: dict[str, np.ndarray] = {}
        for j in self.joints:
            r = rigid_about(j.axis, j.origin, angles[j.name])
            t = out[j.parent] if j.parent is not None else None
            if j.host is not None:
                r = np.einsum("tij,tjk->tik", planar[j.host], r)
            out[j.name] = r if t is None else np.einsum("tij,tjk->tik", t, r)
        return out

    def body_transform(self, body: str, anchor: str | None,
                       joints: Mapping[str, np.ndarray],
                       planar: Mapping[str, np.ndarray], n: int) -> np.ndarray:
        """``T(body)`` without its mesh's placement (``(n, 4, 4)``): see the module
        docstring. ``anchor``: the body's planar anchor (``None``: static)."""
        p = planar[anchor] if anchor is not None else _eye(n)
        name = self.carried.get(body)
        if name is None:
            return p
        if self.joint(name).host is not None:
            return joints[name]
        return np.einsum("tij,tjk->tik", joints[name], p)

    def to_meta(self) -> dict:
        return {"joints": [j.as_dict() for j in self.joints], "carried": dict(self.carried)}

    @classmethod
    def from_meta(cls, meta: Mapping | None) -> RobotTree | None:
        """The tree a mechanism's ``meta["tree"]`` holds (``None``: a planar robot)."""
        if not meta:
            return None
        return cls(tuple(SpatialJoint.from_dict(d) for d in meta["joints"]),
                   {str(k): str(v) for k, v in dict(meta.get("carried", {})).items()})


def _eye(n: int) -> np.ndarray:
    return np.broadcast_to(np.eye(4), (n, 4, 4)).copy()


def rotation(axis, angles) -> np.ndarray:
    """``(T, 3, 3)`` rotations by ``angles`` (rad) about the unit ``axis`` (Rodrigues)."""
    a = np.atleast_1d(np.asarray(angles, dtype=float))
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    c, s = np.cos(a)[:, None, None], np.sin(a)[:, None, None]
    return np.eye(3) + s * kx + (1 - c) * (kx @ kx)


def rigid_about(axis, origin, angles) -> np.ndarray:
    """``(T, 4, 4)``: rotation by ``angles`` about the line through ``origin`` along
    ``axis``."""
    r = rotation(axis, angles)
    o = np.asarray(origin, dtype=float)
    out = _eye(r.shape[0])
    out[:, :3, :3] = r
    out[:, :3, 3] = o - np.einsum("tij,j->ti", r, o)
    return out


def planar_matrix(theta: np.ndarray, trans: np.ndarray) -> np.ndarray:
    """``(T, 4, 4)`` of a planar motion: rotation ``theta`` about z, then ``trans``
    (``(T, 3)``)."""
    theta = np.asarray(theta, dtype=float)
    out = _eye(theta.shape[0])
    c, s = np.cos(theta), np.sin(theta)
    out[:, 0, 0], out[:, 0, 1], out[:, 1, 0], out[:, 1, 1] = c, -s, s, c
    out[:, :3, 3] = np.asarray(trans, dtype=float)
    return out


def quat_from_matrix(r: np.ndarray) -> np.ndarray:
    """``(T, 4)`` xyzw unit quaternions of ``(T, 3, 3)`` rotations (Shepperd's method:
    the largest of the four squares first, so no near-zero divisor)."""
    r = np.asarray(r, dtype=float)
    tr = np.trace(r, axis1=1, axis2=2)
    diag = np.stack([r[:, 0, 0], r[:, 1, 1], r[:, 2, 2], tr], axis=1)
    pick = np.argmax(diag, axis=1)
    q = np.empty((r.shape[0], 4))
    for i in range(r.shape[0]):
        m = r[i]
        k = pick[i]
        if k == 3:
            s = math.sqrt(1.0 + tr[i]) * 2
            q[i] = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s,
                    0.25 * s]
        elif k == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            q[i] = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                    (m[2, 1] - m[1, 2]) / s]
        elif k == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            q[i] = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                    (m[0, 2] - m[2, 0]) / s]
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            q[i] = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
                    (m[1, 0] - m[0, 1]) / s]
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return np.where(q[:, 3:4] < 0, -q, q)
