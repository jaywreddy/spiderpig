"""How far a linkage's links bend under a load at its output: a linear 3D frame model
(2026-10-12, rounds 4-5 of the arm's precision review).

The error budget (:mod:`spiderpig.precision`) took the links as rigid. They aren't: a 3 mm
acrylic link 13 mm wide is about 125 times softer in its plane than the MT4's 5 x 20 mm
aluminium bar, and every reference arm has stiff metal (or thick printed) links. So each
body becomes beams along its outline's segments (``Linkage.links``: a plate with three
joints an L or a T of bars, not a triangle), of the link's own section (``width`` x
``thickness``, Young's ``modulus_mpa``): in-plane bending ``t w^3 / 12``, out-of-plane
``w t^3 / 12``, torsion about ``w t^3 / 3 (1 - 0.63 t / w)``, axial ``w t``. Each body
stands at its layer's mid-plane (``z``: the plan's), each pin is its shaft (a steel rod)
between its bodies' layers, a frame pivot's anchored in both frame plates: a load that
crosses between layers twists and bends the thin links on their weak axis, which is most
of what these plates give (the review: out-of-plane bending of b2, b1, b5, b4 the most).
A pin joins each body to its shaft in every direction but its turn: rigidly in the plane,
and against tilt by the joint's moment stiffness (``tilt_k``: a preloaded ball pair's, a
Hertz estimate). Each drive holds its link's turn. The frame plates and the turntable are
rigid, the drums bolted to the driven links at O left out (about 10 % softer: the review).

:func:`compliance` gives the output's displacement (and a point ``drop`` below it, the
tool's tip: its rotation counted) under a force there, worst over the poses: the
payload's, in the plane and across it (the payload's weight also pushes the tip across
the plane, through the layers' offsets), and a force across the plane (the turntable's
acceleration, a hose's pull). The payload's sag at a given pose and payload repeats (it
calibrates out for that payload, though it varies with the pose); a change in the payload
(a slide picked up) and a load across the plane move the tip by that much each time.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

from spiderpig.linkage.constraints import layout
from spiderpig.linkage.engine import LegSolution, Linkage

PIN_K = 1e9            # N/mm, N·mm/rad: a joint's coupling springs (rigid against the links)


@dataclass(frozen=True)
class Section:
    """A link's section: ``width`` and ``thickness`` (mm), Young's modulus (MPa)."""

    width: float
    thickness: float
    modulus_mpa: float
    poisson: float = 0.35

    @property
    def props(self) -> tuple[float, float, float, float, float]:
        """``(A, I_out, I_in, J, 0)``: the area, the out-of-plane and in-plane second
        moments, the torsion constant."""
        w, t = self.width, self.thickness
        return (w * t, w * t ** 3 / 12, t * w ** 3 / 12,
                w * t ** 3 / 3 * max(1 - 0.63 * t / w, 0.1), 0.0)


@dataclass(frozen=True)
class Rod:
    """A round shaft's section: diameter (mm), Young's modulus (MPa)."""

    d: float
    modulus_mpa: float = 200_000.0
    poisson: float = 0.3

    @property
    def props(self) -> tuple[float, float, float, float, float]:
        i = math.pi * self.d ** 4 / 64
        return (math.pi * self.d ** 2 / 4, i, i, 2 * i, 0.0)

    @property
    def width(self) -> float:
        return self.d


def _beam(p: np.ndarray, q: np.ndarray, s: Section | Rod) -> np.ndarray:
    """A 3D Euler-Bernoulli beam's 12 x 12 stiffness in global axes: a link in the plane
    (its local z the plane's normal: ``I_out`` about local y, ``I_in`` about local z), or
    a shaft along z (round: either)."""
    d = q - p
    length = float(np.linalg.norm(d))
    e = s.modulus_mpa
    g = e / (2 * (1 + s.poisson))
    a, i_out, i_in, j, _ = s.props
    k = np.zeros((12, 12))
    k[0, 0] = k[6, 6] = e * a / length
    k[0, 6] = k[6, 0] = -e * a / length
    k[3, 3] = k[9, 9] = g * j / length
    k[3, 9] = k[9, 3] = -g * j / length

    def bend(iv: float, v: int, r: int, sign: float) -> None:
        """Bending in the plane of local axis ``v`` (translation index), rotation ``r``."""
        c1, c2, c3, c4 = (12 * e * iv / length ** 3, 6 * e * iv / length ** 2,
                          4 * e * iv / length, 2 * e * iv / length)
        idx = [v, r, v + 6, r + 6]
        m = np.array([[c1, sign * c2, -c1, sign * c2],
                      [sign * c2, c3, -sign * c2, c4],
                      [-c1, -sign * c2, c1, -sign * c2],
                      [sign * c2, c4, -sign * c2, c3]])
        for a_, i in enumerate(idx):
            for b_, jj in enumerate(idx):
                k[i, jj] += m[a_, b_]

    bend(i_in, 1, 5, 1.0)       # local y translation with rotation about local z: in-plane
    bend(i_out, 2, 4, -1.0)     # local z translation with rotation about local y: across
    ex = d / length
    ref = np.array([0.0, 0.0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    ey = np.cross(ref, ex)
    ey /= np.linalg.norm(ey)
    ez = np.cross(ex, ey)
    r3 = np.array([ex, ey, ez])
    t = np.zeros((12, 12))
    for b_ in range(4):
        t[3 * b_:3 * b_ + 3, 3 * b_:3 * b_ + 3] = r3
    return t.T @ k @ t


@dataclass(frozen=True)
class Compliance:
    """The tool's worst displacement over the poses sampled: under the payload (its weight,
    straight down in the plane: y) in the plane (``sag_mm``) and across it
    (``sag_across_mm``), and per newton across the plane (``across_mm_per_n``), each at
    the pose where it is worst."""

    sag_mm: float              # the payload's, in the plane (worst over the poses)
    across_mm_per_n: float     # per newton across the plane
    sag_across_mm: float = 0.0  # the payload's, across the plane (through the layers)


class _Frame:
    """One pose's stiffness matrix (module docstring)."""

    def __init__(self, n: int):
        self.k = np.zeros((6 * n, 6 * n))

    def add(self, i: int, j: int, kk: np.ndarray) -> None:
        ii = [*range(6 * i, 6 * i + 6), *range(6 * j, 6 * j + 6)]
        self.k[np.ix_(ii, ii)] += kk

    def spring(self, i: int, j: int | None, comps, kv: float) -> None:
        """Springs ``kv`` on components ``comps`` of node ``i`` (to node ``j``, else to
        the ground)."""
        k = self.k
        for c in comps:
            a = 6 * i + c
            k[a, a] += kv
            if j is not None:
                b = 6 * j + c
                k[b, b] += kv
                k[a, b] -= kv
                k[b, a] -= kv

    def tip(self, o: int, f: np.ndarray, lever: np.ndarray) -> np.ndarray:
        """The displacement of a point ``lever`` from node ``o`` under a force ``f`` there."""
        load = np.zeros(self.k.shape[0])
        load[6 * o:6 * o + 3] = f
        load[6 * o + 3:6 * o + 6] = np.cross(lever, f)
        u = np.linalg.solve(self.k, load)
        return u[6 * o:6 * o + 3] + np.cross(u[6 * o + 3:6 * o + 6], lever)


def _pose(lay, at: dict[str, np.ndarray], section, z: dict[str, float],
          rods: dict[str, Rod], frame_z, drives: dict[str, str],
          segments: dict[str, tuple] | None = None,
          tilt_k: dict[str, float] | None = None,
          sections: dict | None = None) -> tuple[_Frame, int]:
    """The frame of one pose (``at``: each point's xy) and the output's node. ``segments``:
    each body's beams (default: every pair of its points); ``tilt_k``: each joint's
    moment stiffness against a body's tilt on it (N·mm/rad; default rigid)."""
    tilt_k = tilt_k or {}
    bodies = [b for b in lay.bodies if b != "torso"]
    nodes: dict[tuple[str, str], int] = {}
    for b in bodies:
        for j in lay.joints[b]:
            nodes[(b, j)] = len(nodes)
    out_key = (lay.output_body, lay.output_point)
    if out_key not in nodes:
        nodes[out_key] = len(nodes)
    frame = set(lay.joints["torso"])
    # each joint's shaft: a node at each of its bodies' z (a frame pivot's also at both
    # frame plates, where they are given)
    shaft: dict[str, list[tuple[float, int]]] = {}
    count = len(nodes)
    for j in lay.axles:
        zs = {z.get(b, 0.0) for b in bodies if j in lay.joints[b]}
        if j in frame and frame_z is not None:
            zs |= set(frame_z)
        shaft[j] = [(zz, count + i) for i, zz in enumerate(sorted(zs))]
        count += len(zs)
    fr = _Frame(count)
    for b in bodies:
        ps = list(lay.joints[b])
        if b == lay.output_body and lay.output_point not in ps:
            ps.append(lay.output_point)
        pairs = (segments or {}).get(b) or [(ps[a], ps[c]) for a in range(len(ps))
                                           for c in range(a + 1, len(ps))]
        for u, v in pairs:
            if (b, u) not in nodes or (b, v) not in nodes:
                continue
            p = np.array([*at[u], z.get(b, 0.0)])
            q = np.array([*at[v], z.get(b, 0.0)])
            if np.linalg.norm(q - p) > 1e-6:
                fr.add(nodes[(b, u)], nodes[(b, v)],
                       _beam(p, q, (sections or {}).get(b, section)))
    for j, levels in shaft.items():
        rod = rods.get(j, Rod(4.0))
        x, y = at[j]
        for (za, a), (zb, b) in itertools.pairwise(levels):
            fr.add(a, b, _beam(np.array([x, y, za]), np.array([x, y, zb]), rod))
        at_z = dict(levels)
        for (b, jj), i in nodes.items():
            if jj == j:           # all but its turn: the tilt through the joint's moment
                fr.spring(i, at_z[z.get(b, 0.0)], [0, 1, 2], PIN_K)
                fr.spring(i, at_z[z.get(b, 0.0)], [3, 4], tilt_k.get(j, PIN_K))
        for zz, i in levels:
            fr.spring(i, None, [5], 1e-6)                         # (its own turn: nothing)
            if j in frame and (frame_z is None or zz in frame_z):
                fr.spring(i, None, range(6), PIN_K)               # in a frame plate: fixed
    for b, j in drives.items():
        if (b, j) in nodes:
            fr.spring(nodes[(b, j)], None, [5], PIN_K)            # the drive holds its turn
    return fr, nodes[out_key]


def compliance(lk: Linkage, values: tuple[float, ...], section: Section, *,
               payload_n: float, drop: float = 0.0, ins=None,
               z: dict[str, float] | None = None, rods: dict[str, Rod] | None = None,
               frame_z: tuple[float, float] | None = None,
               tilt_k: dict[str, float] | None = None,
               sections: dict[str, Section] | None = None) -> Compliance:
    """:class:`Compliance` of ``lk``'s side (module docstring) at the inputs ``ins`` (radians,
    one array per input; default: the box, 5 a side). With ``z`` (each body's mid-plane
    across the stack, the plan's) the bodies stand at their layers and each pin is its
    shaft (``rods``: per joint, default a 4 mm steel rod) between them, a frame pivot's
    anchored in both frame plates (``frame_z``): the stack's own depth, a space frame;
    without, every body in one plane (softer across it). ``sections``: a body's own section
    where its link is cut from another sheet (default ``section``)."""
    lay = layout(lk)
    if ins is None:
        ins = lk.box(5)
    with np.errstate(all="ignore"):
        pts = LegSolution(1, 0.0, values, lk.key).at(*ins)
    n = next(iter(pts.values())).reshape(-1, 2).shape[0]
    drives = {lk.drives[k][0]: lk.drives[k][1] for k in lk.inputs} if lk.ranged else {}
    lever = np.array([0.0, -drop, 0.0])
    segments = {b: segs for b, (_, segs) in lk.links.items()}
    worst = np.zeros(3)
    for s in range(n):
        at = {k: np.asarray(v).reshape(-1, 2)[s] for k, v in pts.items()}
        if not all(np.isfinite(v).all() for v in at.values()):
            continue
        fr, o = _pose(lay, at, section, z or {}, rods or {}, frame_z, drives, segments,
                      tilt_k, sections)
        try:
            u = fr.tip(o, np.array([0.0, -payload_n, 0.0]), lever)
            across = float(abs(fr.tip(o, np.array([0.0, 0.0, 1.0]), lever)[2]))
        except np.linalg.LinAlgError:
            return Compliance(math.inf, math.inf, math.inf)
        worst = np.maximum(worst, [float(np.hypot(u[0], u[1])), across, abs(float(u[2]))])
    return Compliance(*(float(v) for v in worst))
