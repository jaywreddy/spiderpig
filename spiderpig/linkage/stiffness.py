"""How far a linkage's links bend under a load where it meets the world: a linear 3D frame
model (2026-10-12, rounds 4-5 of the arm's precision review; a walker's feet 2026-10-14).

The error budget (:mod:`spiderpig.precision`) took the links as rigid. They aren't: a 3 mm
acrylic link 13 mm wide is about 125 times softer in its plane than the MT4's 5 x 20 mm
aluminium bar, and every reference arm has stiff metal (or thick printed) links. So each
body becomes beams along its outline's segments (``Linkage.links``: a plate with three
joints an L or a T of bars, not a triangle; a segment split at a joint partway along it,
Strider's J1 on b2), of the link's own section (``width`` x
``thickness``, Young's ``modulus_mpa``): in-plane bending ``t w^3 / 12``, out-of-plane
``w t^3 / 12``, torsion about ``w t^3 / 3 (1 - 0.63 t / w)``, axial ``w t``. Each body
stands at its layer's mid-plane (``z``: the plan's), each pin is its shaft (a steel rod)
between its bodies' layers, a frame pivot's anchored in both frame plates: a load that
crosses between layers twists and bends the thin links on their weak axis, which is most
of what these plates give (the review: out-of-plane bending of b2, b1, b5, b4 the most).
A pin joins each body to its shaft in every direction but its turn: rigidly in the plane,
and against tilt by the joint's moment stiffness (``tilt_k``: a preloaded ball pair's, a
Hertz estimate). Each drive holds its link's turn (:func:`drives`: a ranged linkage's
inputs, a crank's at its centre). The frame plates and the turntable are rigid, the drums
bolted to the driven links at O left out (about 10 % softer: the review).

Where the load comes in is the linkage's own (:func:`outputs`, by what it can do, never by
its kind's name): a mechanism's or a ranged linkage's output (the arm's tool point), a
walker's feet.

:func:`compliance` gives the output's displacement (and a point ``drop`` below it, the
tool's tip: its rotation counted) under a force there, worst over the poses: the
payload's, in the plane and across it (the payload's weight also pushes the tip across
the plane, through the layers' offsets), and a force across the plane (the turntable's
acceleration, a hose's pull). The payload's sag at a given pose and payload repeats (it
calibrates out for that payload, though it varies with the pose); a change in the payload
(a slide picked up) and a load across the plane move the tip by that much each time.

:func:`across` gives each output's displacement across the plane per newton across it
(a walker's foot pushed sideways: a turn, a slip), worst over the poses, and what the
links do under it: the most one tilts out of the plane and the most two on one joint
splay (their tilts' difference: the pin between them bending), per newton. The play in
the joints (:mod:`construction.wobble`'s tilt) comes on top: this is the elastic part.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np
from scipy.sparse import csc_matrix
from scipy.sparse.linalg import splu

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

    @classmethod
    def bending(cls, inertia: float, modulus_mpa: float) -> Rod:
        """The round rod as stiff in bending as a shaft of second moment ``inertia``
        (mm^4: a tube's, :class:`construction.wobble.Section`) and modulus ``modulus_mpa``."""
        return cls((64 * inertia / math.pi) ** 0.25, modulus_mpa)


def outputs(lk: Linkage) -> tuple[tuple[str, str], ...]:
    """Where a linkage meets the world, ``(body, point)`` each: its output where it has one
    (a mechanism's or a ranged linkage's tool point), else every foot of its leg (a
    walker's, :attr:`Linkage.feet`)."""
    if lk.output is not None:
        return ((lk.output.link, lk.output.point),)
    return tuple((b, p) for b, p in lk.feet)


def drives(lk: Linkage) -> dict[str, str]:
    """Each driven body and the joint its drive holds its turn at: a ranged linkage's inputs
    (:attr:`Linkage.drives`), else its crank (``conn``, the layout's) at the crank's
    centre."""
    if lk.ranged:
        return {lk.drives[k][0]: lk.drives[k][1] for k in lk.inputs}
    return {"conn": lk.crank[0]} if lk.crank else {}


ON_SEGMENT = 1e-6      # a joint this near a segment (over its length) lies on it


def _beams(ps: list[str], segs, at: dict[str, np.ndarray]) -> list[tuple[str, str]]:
    """A body's beams: its outline's segments (without one, every pair of its points), each
    split at the body's other points that lie on it (a pin partway along a bar, Strider's
    J1 on b2), and a point on no segment joined to the ends of the nearest one."""
    if not segs:
        return [(ps[a], ps[c]) for a in range(len(ps)) for c in range(a + 1, len(ps))]
    out: list[tuple[str, str]] = []
    placed = {p for s in segs for p in s}

    def along(x: str, u: str, v: str) -> tuple[float, float]:
        """``x``'s distance from segment ``u v`` (over its length) and where along it."""
        p, d = at[u], at[v] - at[u]
        ll = float(d @ d)
        if ll <= 0:
            return math.inf, 0.0
        t = float((at[x] - p) @ d) / ll
        return float(np.linalg.norm(p + min(max(t, 0.0), 1.0) * d - at[x])) / math.sqrt(ll), t

    for u, v in segs:
        on = sorted((t, x) for x in ps if x not in (u, v)
                    for dist, t in (along(x, u, v),) if dist <= ON_SEGMENT and 0 < t < 1)
        placed |= {x for _, x in on}
        out += list(itertools.pairwise([u, *(x for _, x in on), v]))
    for x in ps:
        if x not in placed:
            u, v = min(segs, key=lambda s: along(x, *s)[0])
            out += [(u, x), (x, v)]
    return out


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
        self.nodes: dict[tuple[str, str], int] = {}     # (body, point) -> its node

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
          sections: dict | None = None,
          outs: tuple[tuple[str, str], ...] | None = None) -> tuple[_Frame, int]:
    """The frame of one pose (``at``: each point's xy) and the first output's node (every
    body point's in the frame's ``nodes``). ``segments``: each body's outline (default:
    every pair of its points; :func:`_beams`); ``tilt_k``: each joint's moment stiffness
    against a body's tilt on it (N·mm/rad; default rigid); ``outs``: the points loads come
    in at (default the layout's output)."""
    tilt_k = tilt_k or {}
    bodies = [b for b in lay.bodies if b != "torso"]
    nodes: dict[tuple[str, str], int] = {}
    for b in bodies:
        for j in lay.joints[b]:
            nodes[(b, j)] = len(nodes)
    outs = outs or ((lay.output_body, lay.output_point),)
    for out_key in outs:
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
    fr.nodes = nodes
    for b in bodies:
        ps = list(lay.joints[b])
        ps += [p for ob, p in outs if ob == b and p not in ps]
        for u, v in _beams(ps, (segments or {}).get(b), at):
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
    return fr, nodes[outs[0]]


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
        ins = poses(lk, 5)
    with np.errstate(all="ignore"):
        pts = LegSolution(1, 0.0, values, lk.key).at(*ins)
    n = next(iter(pts.values())).reshape(-1, 2).shape[0]
    held = drives(lk)
    lever = np.array([0.0, -drop, 0.0])
    segments = {b: segs for b, (_, segs) in lk.links.items()}
    worst = np.zeros(3)
    for s in range(n):
        at = {k: np.asarray(v).reshape(-1, 2)[s] for k, v in pts.items()}
        if not all(np.isfinite(v).all() for v in at.values()):
            continue
        fr, o = _pose(lay, at, section, z or {}, rods or {}, frame_z, held, segments,
                      tilt_k, sections)
        try:
            u = fr.tip(o, np.array([0.0, -payload_n, 0.0]), lever)
            side = float(abs(fr.tip(o, np.array([0.0, 0.0, 1.0]), lever)[2]))
        except np.linalg.LinAlgError:
            return Compliance(math.inf, math.inf, math.inf)
        worst = np.maximum(worst, [float(np.hypot(u[0], u[1])), side, abs(float(u[2]))])
    return Compliance(*(float(v) for v in worst))


def poses(lk: Linkage, n: int) -> tuple[np.ndarray, ...]:
    """The inputs a linkage's stiffness is sampled at: a ranged linkage's box (``n`` a
    side), a crank's ``n`` angles over its revolution (radians)."""
    if lk.ranged:
        return lk.box(n)
    return (np.linspace(0.0, 2 * math.pi, n, endpoint=False),)


@dataclass(frozen=True)
class Across:
    """One output's stiffness across the plane (:func:`across`), each figure the worst over
    the poses sampled: the output's displacement across the plane per newton across it
    there (``mm_per_n``, at the first input's ``at`` radians), the most a link tilts out
    of the plane per newton (``tilt_deg_per_n``, link ``tilt_link``) and the most two links
    on one joint splay (their tilts' difference, ``splay_deg_per_n`` at ``splay_joint``)."""

    body: str
    point: str
    mm_per_n: float
    at: float
    tilt_deg_per_n: float
    tilt_link: str
    splay_deg_per_n: float
    splay_joint: str


def across(lk: Linkage, values: tuple[float, ...], section: Section, *, ins=None,
           z: dict[str, float] | None = None, rods: dict[str, Rod] | None = None,
           frame_z: tuple[float, float] | None = None,
           tilt_k: dict[str, float] | None = None,
           sections: dict[str, Section] | None = None) -> list[Across]:
    """:class:`Across` of each of ``lk``'s outputs (:func:`outputs`: an arm's tool point, a
    walker's feet) at the inputs ``ins`` (default :func:`poses`, 24), the frame as
    :func:`compliance` builds it: a unit force across the plane at the output, each link's
    tilt the largest out-of-plane rotation of its points, a joint's splay the largest
    difference between its links' rotations there. A singular frame (a free motion across
    the plane) reads ``inf``."""
    lay = layout(lk)
    if ins is None:
        ins = poses(lk, 24)
    with np.errstate(all="ignore"):
        pts = LegSolution(1, 0.0, values, lk.key).at(*ins)
    n = next(iter(pts.values())).reshape(-1, 2).shape[0]
    first = np.broadcast_to(np.asarray(ins[0], dtype=float).ravel(), (n,))
    outs = outputs(lk)
    held = drives(lk)
    segments = {b: segs for b, (_, segs) in lk.links.items()}
    links = [b for b in lay.bodies if b in lk.links]
    worst = {o: [0.0, 0.0, 0.0, "", 0.0, ""] for o in outs}
    for s in range(n):
        at = {k: np.asarray(v).reshape(-1, 2)[s] for k, v in pts.items()}
        if not all(np.isfinite(v).all() for v in at.values()):
            continue
        fr, _ = _pose(lay, at, section, z or {}, rods or {}, frame_z, held, segments,
                      tilt_k, sections, outs)
        load = np.zeros((fr.k.shape[0], len(outs)))
        for c, o in enumerate(outs):
            load[6 * fr.nodes[o] + 2, c] = 1.0                    # 1 N across the plane
        try:        # sparse (SuperLU, one thread): a dense solve's threads crawl on a busy box
            u = splu(csc_matrix(fr.k)).solve(load)
        except RuntimeError:                                      # exactly singular
            return [Across(*o, math.inf, math.nan, math.inf, "", math.inf, "") for o in outs]
        for c, o in enumerate(outs):
            w = worst[o]
            side = abs(float(u[6 * fr.nodes[o] + 2, c]))
            if side > w[0]:
                w[0], w[1] = side, float(first[s])
            rot = {key: u[6 * i + 3:6 * i + 5, c] for key, i in fr.nodes.items()}
            for b in links:
                tilt = max(float(np.hypot(*rot[(b, j)])) for j in lay.joints[b])
                if tilt > w[2]:
                    w[2], w[3] = tilt, b
            for j in lay.axles:
                on = [rot[(b, j)] for b in links if (b, j) in rot]
                for a, b in itertools.combinations(on, 2):
                    splay = float(np.hypot(*(a - b)))
                    if splay > w[4]:
                        w[4], w[5] = splay, j
    return [Across(*o, worst[o][0], worst[o][1], math.degrees(worst[o][2]), str(worst[o][3]),
                   math.degrees(worst[o][4]), str(worst[o][5])) for o in outs]
