"""A decanting cradle's structure at its statics' force vectors: what the generic joint
rules of :mod:`spiderpig.strength` can't see (design review round 1, 2026-10-14).

:func:`spiderpig.cradle_report.statics` solves every joint's force *vector* per crank angle.
The generic rules use only the largest magnitude: a link of three or more pins as a beam
between its farthest two (``F L / 4``), a pillar as a beam between both frame plates. Those
miss a bell crank's lever (B's b2 read 2.61 jammed where it is about 1.0), a cradle link's
arms, and a pillar whose far plate can't hold it (B's inner bar O1-O2 held O2 only along
itself: about 0.39 jammed, not 1.33). Here, from the vectors:

* **Links in bending** (:func:`link_bending`): each link a plane frame of its chords
  (24 mm strips of its sheet, rigid at the joints), loaded at its pins by the statics'
  forces; what else holds it in equilibrium (its own weight; on the cradle link the bottle's
  and the box's, and the jam's couple or block) spread over its load nodes as a rigid box
  would (least squares: the cradle link's, its tie rods). Each chord's axial force and
  moment at its ends, rated at the end's hole (the net section: ``(w - d) t`` and
  ``t (w^3 - d^3) / 6 w``) and, in compression, its Euler load out of the plane (pinned).
  A tree of chords is statically determinate: its moments are ``|r x F|`` about each
  joint, whatever the stiffness.
* **Posts on the box** (:func:`posts`): the pillars and the stand's columns between a
  side's two frame plates, solved together as one frame. Each post is a steel beam, its
  foot fixed in the outer plate's plane and its end moving with the inner plate, which is
  rigid in its plane and held there by nothing but the posts. Each end is held against
  turning only as stiffly as its plate's members there bend and twist out of their plane
  (:func:`plate_spring`: 24 mm strips, so a post's ends are far from clamped). A pillar's
  link loads are shared through the inner plate by compatibility; whatever net force the
  pillars take goes back to the outer plate through every post's bending. So a plate on
  two posts in a line (B's bar O1-O2, before round 1) turns about one of them and holds a
  pillar across the line hardly at all: there the pillar is a cantilever off the outer
  plate. Three or more posts not in a line hold it both ways.
* **The frame plates' members** (:func:`members`): each plate a plane frame of its members
  (the stand's pills, split where a post stands on one), loaded by the posts' feet (the
  outer plate: and by the gearbox at the crank, its bearing force and its torque) or their
  ends (the inner plate); the outer plate pinned at its feet. Each member's in-plane stress
  and, in compression, its Euler load out of the plane between its nodes (pinned ends, the
  nodes held: :data:`MEMBER_K`).

Everything takes the cases' force vectors, so a caller rates any crank torque (a clutch's
cap) or any angles (the toggles the jam case leaves out: :func:`cradle_report.structure`).
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

E_ALU = 69000.0         # MPa: 5052, 6061 (7075's 71.7 taken as this: its frames' shares)
E_STEEL = 200000.0      # MPa: the steel standoffs
MEMBER_K = 1.0          # a frame member's effective length over its length, out of its plane:
#                         pinned at both nodes, the nodes held by the box (its posts and the
#                         gearbox's block)


# -- a plane frame ---------------------------------------------------------------------


def _local_k(L: float, EA: float, EI: float) -> np.ndarray:
    a, b, c, d = EA / L, 12 * EI / L ** 3, 6 * EI / L ** 2, EI / L
    return np.array([[a, 0, 0, -a, 0, 0], [0, b, c, 0, -b, c], [0, c, 4 * d, 0, -c, 2 * d],
                     [-a, 0, 0, a, 0, 0], [0, -b, -c, 0, b, -c], [0, c, 2 * d, 0, -c, 4 * d]])


def _rot(c: float, s: float) -> np.ndarray:
    r = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    t = np.zeros((6, 6))
    t[:3, :3] = r
    t[3:, 3:] = r
    return t


def frame2d(nodes: np.ndarray, elems: Sequence[tuple[int, int, float, float]],
            loads: np.ndarray, fixed: Sequence[int]) -> np.ndarray:
    """A plane frame (Euler-Bernoulli beams, rigid joints): ``nodes`` (n, 2) mm, ``elems``
    ``(i, j, EA, EI)`` (N, N·mm²), ``loads`` (cases, n, 2) nodal forces (N) or (cases, n, 3)
    with a moment (N·mm), ``fixed`` the held degrees of freedom (``3 node + {0: x, 1: y, 2:
    turn}``). Returns (cases, elems, 3): each element's axial force (N, tension positive)
    and its end moments at ``i`` and ``j`` (N·mm)."""
    n = len(nodes)
    K = np.zeros((3 * n, 3 * n))
    mats = []
    for i, j, EA, EI in elems:
        d = nodes[j] - nodes[i]
        L = float(np.hypot(*d))
        k = _local_k(L, EA, EI)
        t = _rot(d[0] / L, d[1] / L)
        dofs = [3 * i, 3 * i + 1, 3 * i + 2, 3 * j, 3 * j + 1, 3 * j + 2]
        K[np.ix_(dofs, dofs)] += t.T @ k @ t
        mats.append((dofs, k @ t))
    loads = np.asarray(loads, float)
    F = np.zeros((loads.shape[0], 3 * n))
    F[:, 0::3] = loads[..., 0]
    F[:, 1::3] = loads[..., 1]
    if loads.shape[-1] > 2:
        F[:, 2::3] = loads[..., 2]
    free = [d for d in range(3 * n) if d not in set(fixed)]
    U = np.zeros_like(F)
    U[:, free] = np.linalg.solve(K[np.ix_(free, free)], F[:, free].T).T
    out = np.zeros((loads.shape[0], len(elems), 3))
    for e, (dofs, kt) in enumerate(mats):
        f = U[:, dofs] @ kt.T
        out[:, e] = np.stack([f[:, 3], f[:, 2], f[:, 5]], axis=-1)
    return out


def _cross(r: np.ndarray, f: np.ndarray) -> np.ndarray:
    return r[..., 0] * f[..., 1] - r[..., 1] * f[..., 0]


def spread(at: np.ndarray, force: np.ndarray, moment: np.ndarray) -> np.ndarray:
    """Forces at the points ``at`` (k, 2) whose sum is ``force`` (cases, 2) and whose moment
    about the origin is ``moment`` (cases,), the least in sum of squares: a rigid body's
    share (a lone point takes the force, its moment unbalanced). (cases, k, 2)."""
    k = len(at)
    A = np.zeros((3, 2 * k))
    A[0, 0::2] = 1.0
    A[1, 1::2] = 1.0
    A[2, 0::2] = -at[:, 1]
    A[2, 1::2] = at[:, 0]
    b = np.concatenate([force, moment[:, None]], axis=1)
    return (b @ np.linalg.pinv(A).T).reshape(len(force), k, 2)


def _split(names: list[str], xy: Mapping[str, np.ndarray], segs, tol: float = 0.5
           ) -> list[tuple[str, str]]:
    """``segs`` split at every point of ``names`` lying inside one (within ``tol`` mm)."""
    out = []
    for a, b in segs:
        pa, pb = xy[a], xy[b]
        ab = pb - pa
        L = float(np.hypot(*ab))
        on = []
        for p in names:
            if p in (a, b):
                continue
            u = float((xy[p] - pa) @ ab) / (L * L)
            if 1e-6 < u < 1 - 1e-6 and float(np.hypot(*(pa + u * ab - xy[p]))) < tol:
                on.append((u, p))
        chain = [a, *(p for _, p in sorted(on)), b]
        out += list(itertools.pairwise(chain))
    return out


def _section_sf(N: np.ndarray, M: np.ndarray, w: float, t: float, d: float, yield_mpa: float
                ) -> tuple[np.ndarray, np.ndarray]:
    """A strip ``w`` x ``t`` with a hole ``d`` across it: the stress (MPa) of its axial force
    ``N`` and in-plane moment ``M`` (N·mm) on the net section, and the safety factor."""
    area = (w - d) * t
    z = t * (w ** 3 - d ** 3) / (6 * w)
    sig = np.abs(N) / area + np.abs(M) / z
    return sig, yield_mpa / np.maximum(sig, 1e-9)


# -- links in bending ------------------------------------------------------------------


@dataclass(frozen=True)
class LinkSection:
    """A link's strip: width and thickness (mm), its sheet's yield (MPa)."""

    w: float
    t: float
    yield_mpa: float
    sheet: str = ""
    e: float = E_ALU


def link_bending(lk, pts: Mapping[str, np.ndarray], joints, vecs: np.ndarray,
                 sections: Mapping[str, LinkSection], holes: Mapping[str, float],
                 load_nodes: Mapping[str, Sequence[str]] | None = None,
                 skip: np.ndarray | None = None,
                 width: Callable[[str, str, str], float] | None = None) -> dict[str, dict]:
    """Every link of ``lk`` (``lk.links``) as a plane frame of its chords under one case's
    joint force vectors ``vecs`` (``cradle_report.statics_vectors``: ``joints`` its
    ``(point, body, other)``, ``pts`` the joints' world paths): per link, its worst chord end
    over the angles not in ``skip`` (the stress on the net section at the end's hole,
    ``holes`` by joint, and the safety factor), its axial force and moment there, and its
    least Euler factor of a chord in compression (out of the plane, pinned at its ends:
    the whole chord, a joint on it no brace). What holds the link besides its pins is spread
    over ``load_nodes[link]`` (default: all its joints). ``width(link, a, b)``: a chord's
    width where it isn't its section's."""
    load_nodes = load_nodes or {}
    keep = np.ones(len(vecs), bool) if skip is None else ~np.asarray(skip, bool)
    out: dict[str, dict] = {}
    if not keep.any():
        return out
    k0 = int(np.argmax(keep))
    for name, (js, edges) in lk.links.items():
        sec = sections.get(name)
        if sec is None or not edges:
            continue
        ends = sorted({p for e in edges for p in e})
        at0 = {p: pts[p][k0] for p in js if p in pts}
        segs, parent = [], {}
        for e in edges:
            for s_ in _split(list(at0), at0, [tuple(e)]):
                segs.append(s_)
                parent[s_] = tuple(e)
        wide = {s_: (width(name, *parent[s_]) if width else sec.w) for s_ in segs}
        names = sorted({p for s in segs for p in s}, key=lambda p: (p not in ends, p))
        idx = {p: i for i, p in enumerate(names)}
        # the link's turn per angle, from its first chord
        a, b = edges[0]
        d = pts[b] - pts[a]
        turn = np.arctan2(d[:, 1], d[:, 0])
        turn = turn - turn[k0]
        c, s = np.cos(-turn), np.sin(-turn)

        def ref(v, c=c, s=s):       # a world vector at each angle, in the link's pose at k0
            return np.stack([c * v[..., 0] - s * v[..., 1], s * v[..., 0] + c * v[..., 1]], -1)

        xy = np.array([pts[p][k0] for p in names])
        loads = np.zeros((len(vecs), len(names), 2))
        for jk, (j, b1, b2) in enumerate(joints):
            if j not in idx or name not in (b1, b2):
                continue
            f = vecs[:, jk] if b1 == name else -vecs[:, jk]
            loads[:, idx[j]] += ref(f)
        # what else holds it (gravity, the jam), spread over its load nodes as a rigid body
        total = loads.sum(axis=1)
        mom = _cross(xy[None, :, :], loads).sum(axis=1)
        on = [p for p in load_nodes.get(name, names) if p in idx] or names
        share = spread(xy[[idx[p] for p in on]], -total, -mom)
        for i, p in enumerate(on):
            loads[:, idx[p]] += share[:, i]
        loads = loads[keep]
        elems = [(idx[p], idx[q], sec.e * wide[(p, q)] * sec.t,
                  sec.e * sec.t * wide[(p, q)] ** 3 / 12) for p, q in segs]
        res = frame2d(xy, elems, loads, fixed=[0, 1, 2])
        worst = {"safety": math.inf}
        comp: dict[tuple[str, str], float] = {}
        for e, (p, q) in enumerate(segs):
            N = res[:, e, 0]
            w = wide[(p, q)]
            for end, M in ((p, res[:, e, 1]), (q, res[:, e, 2])):
                sig, sf = _section_sf(N, M, w, sec.t, holes.get(end, 0.0), sec.yield_mpa)
                i = int(np.argmin(sf))
                if sf[i] < worst["safety"]:
                    worst = {"safety": float(sf[i]), "chord": f"{p}-{q}", "at": end,
                             "width_mm": w, "moment_nm": round(float(abs(M[i])) / 1000.0, 2),
                             "axial_n": round(float(N[i]), 1),
                             "stress_mpa": round(float(sig[i]), 1),
                             "hole_mm": holes.get(end, 0.0)}
            c_ = parent[(p, q)]
            comp[c_] = max(comp.get(c_, 0.0), float(max(-N.min(), 0.0)))
        buckle = {"safety": math.inf}
        for (p, q), f in comp.items():
            L = float(np.hypot(*(at0[q] - at0[p])))
            w = width(name, p, q) if width else sec.w
            pcr = math.pi ** 2 * sec.e * w * sec.t ** 3 / 12 / L ** 2
            if f > 0 and pcr / f < buckle["safety"]:
                buckle = {"safety": pcr / f, "chord": f"{p}-{q}", "compression_n": round(f, 1),
                          "euler_n": round(pcr, 1)}
        worst["safety"] = round(worst["safety"], 2)
        out[name] = {"section": [sec.w, sec.t], "sheet": sec.sheet,
                     "allowable_mpa": sec.yield_mpa, "chords": [f"{p}-{q}" for p, q in segs],
                     "worst": worst,
                     "buckling": ({**buckle, "safety": round(buckle["safety"], 2)}
                                  if math.isfinite(buckle["safety"]) else None)}
    return out


# -- posts on the box ------------------------------------------------------------------

G_ALU = 26000.0         # MPa: aluminium's shear modulus


def plate_spring(at: np.ndarray, others: Sequence[tuple[np.ndarray, float]], t: float
                 ) -> np.ndarray:
    """How a frame plate holds a post's end against turning (2 x 2, N·mm/rad, on the
    turn's vector in the plate's plane): each of the plate's members at ``at`` (its far node
    and its width, ``t`` thick), held at its far node, bends across its weak axis for a
    turn square to it (``4 E I / L``) and twists for one along it (``G J / L``)."""
    k = np.zeros((2, 2))
    for far, w in others:
        d = np.asarray(far, float) - np.asarray(at, float)
        L = float(np.hypot(*d))
        if L < 1e-6:
            continue
        m = d / L
        mm = np.outer(m, m)
        bend = 4 * E_ALU * w * t ** 3 / 12 / L
        twist = G_ALU * w * t ** 3 / 3 / L
        k += twist * mm + bend * (np.eye(2) - mm)
    return k


@dataclass
class Post:
    """A post between a side's frame plates: a pillar (its links' loads ``loads``: joint
    index into the statics' ``joints``, the sign that makes it the force on the post, its
    height over the outer plate's face) or a stand column (none). ``ei`` its bending
    stiffness (N·mm²); ``foot_k`` and ``end_k`` how the outer and the inner plate hold its
    ends against turning (:func:`plate_spring`); ``thread`` how deep its ends are tapped
    (its moments there are rated on the tube the tap leaves, ``ends_nm``; between them on
    the solid bar, ``mid_nm``)."""

    name: str
    xy: np.ndarray
    ei: float
    loads: list[tuple[int, float, float]] = field(default_factory=list)
    foot_k: np.ndarray = field(default_factory=lambda: np.zeros((2, 2)))
    end_k: np.ndarray = field(default_factory=lambda: np.zeros((2, 2)))
    thread: float = 0.0


def _beam_k(ei: float, h: float) -> np.ndarray:
    """A beam element's stiffness in one plane, ``[u_a, s_a, u_b, s_b]`` (slope s = du/dz)."""
    hh = h * h
    return ei / h ** 3 * np.array([[12, 6 * h, -12, 6 * h], [6 * h, 4 * hh, -6 * h, 2 * hh],
                                   [-12, -6 * h, 12, -6 * h], [6 * h, 2 * hh, -6 * h, 4 * hh]])


_TURN = np.array([[0.0, -1.0], [1.0, 0.0]])   # a post's end's slopes (dx/dz, dy/dz) -> its turn


def posts(side: Sequence[Post], length: float, vecs: np.ndarray,
          skip: np.ndarray | None = None) -> dict[str, dict]:
    """The posts of one side on the box (the module doc) under one case's joint force
    vectors ``vecs``, over the angles not in ``skip``: one plane-frame solve of every post
    (a beam, its foot fixed in the outer plate's plane and its end moving with the inner
    plate, a rigid body in its plane), each end held against turning by its plate
    (``foot_k``, ``end_k``). Per post: its worst bending moment and shear, its foot's moment
    (into the outer plate) and the force its end puts on the inner plate; and per angle its
    force on the outer plate (``forces["box"]["foot"]``) and the inner plate's on its end
    (``["end"]``), for :func:`members`. ``length``: between the plates' faces (mm)."""
    keep = np.ones(len(vecs), bool) if skip is None else ~np.asarray(skip, bool)
    v = vecs[keep]
    m = len(v)
    L = length
    xy = np.array([p.xy for p in side], float)
    rho = xy - xy.mean(axis=0)
    # the global unknowns: the inner plate's (tx, ty, turn), then each post's own
    n_glob = 3
    plans = []
    for p in side:
        zs = sorted({0.0, L, *(a for _, _, a in p.loads if 0.0 < a < L)})
        nn = len(zs)
        # each node: ux, uy, sx, sy; the foot's u fixed, the end's u the plate's
        free = {}
        for k in range(nn):
            for d in range(4):
                if (k == 0 and d < 2) or (k == nn - 1 and d < 2):
                    continue
                free[(k, d)] = n_glob
                n_glob += 1
        plans.append((zs, free))
    K = np.zeros((n_glob, n_glob))
    F = np.zeros((m, n_glob))
    maps = []
    for i, (p, (zs, free)) in enumerate(zip(side, plans, strict=True)):
        nn = len(zs)
        A = np.zeros((4 * nn, n_glob))           # the post's local dofs from the global ones
        for (k, d), g in free.items():
            A[4 * k + d, g] = 1.0
        r = rho[i]
        A[4 * (nn - 1), :3] = [1.0, 0.0, -r[1]]
        A[4 * (nn - 1) + 1, :3] = [0.0, 1.0, r[0]]
        Kl = np.zeros((4 * nn, 4 * nn))
        for e in range(nn - 1):
            ke = _beam_k(p.ei, zs[e + 1] - zs[e])
            for plane in (0, 1):              # x: (ux, sx), y: (uy, sy)
                idx = [4 * e + plane, 4 * e + 2 + plane, 4 * (e + 1) + plane,
                       4 * (e + 1) + 2 + plane]
                Kl[np.ix_(idx, idx)] += ke
        for k, spring in ((0, p.foot_k), (nn - 1, p.end_k)):
            ks = _TURN.T @ spring @ _TURN
            idx = [4 * k + 2, 4 * k + 3]
            Kl[np.ix_(idx, idx)] += ks
        fl = np.zeros((m, 4 * nn))
        for jk, sign, a in p.loads:
            k = int(np.argmin([abs(z - a) for z in zs]))
            fl[:, 4 * k] += v[:, jk, 0] * sign
            fl[:, 4 * k + 1] += v[:, jk, 1] * sign
        K += A.T @ Kl @ A
        F += fl @ A
        maps.append((A, Kl, fl))
    U = np.linalg.solve(K + np.eye(n_glob) * 1e-9 * np.abs(K).max(), F.T).T
    out: dict[str, dict] = {}
    for p, (zs, _), (A, Kl, fl) in zip(side, plans, maps, strict=True):
        nn = len(zs)
        ul = U @ A.T
        M = np.zeros(m)
        ends = np.zeros(m)          # within its tapped ends (``thread`` of either end)
        mid = np.zeros(m)
        V = np.zeros(m)
        for e in range(nn - 1):
            za, zb = zs[e], zs[e + 1]
            ke = _beam_k(p.ei, zb - za)
            m0, m1, sh = [], [], []
            for plane in (0, 1):
                idx = [4 * e + plane, 4 * e + 2 + plane, 4 * (e + 1) + plane,
                       4 * (e + 1) + 2 + plane]
                f = ul[:, idx] @ ke.T
                m0.append(-f[:, 1])               # the bending moment at za and at zb
                m1.append(f[:, 3])
                sh.append(f[:, 0])
            m0_, m1_ = np.stack(m0, -1), np.stack(m1, -1)

            def at(z, m0_=m0_, m1_=m1_, za=za, zb=zb):
                u = (z - za) / (zb - za)
                return np.hypot(*((1 - u) * m0_ + u * m1_).T)

            M = np.maximum(M, np.maximum(at(za), at(zb)))
            for lo, hi, into in ((0.0, p.thread, "ends"), (L - p.thread, L, "ends"),
                                 (p.thread, L - p.thread, "mid")):
                a, b = max(za, lo), min(zb, hi)
                if b < a - 1e-9:
                    continue
                got = np.maximum(at(a), at(b))
                if into == "ends":
                    ends = np.maximum(ends, got)
                else:
                    mid = np.maximum(mid, got)
            V = np.maximum(V, np.hypot(sh[0], sh[1]))
        # the plates' forces on the post's ends: what the post's own equations leave over
        res = ul @ Kl.T - fl
        foot_f = res[:, 0:2]                     # the outer plate's on its foot
        end_f = res[:, 4 * (nn - 1):4 * (nn - 1) + 2]
        foot_m = ul[:, 2:4] @ (_TURN.T @ p.foot_k @ _TURN).T     # its springs' moments
        tip_m = ul[:, 4 * (nn - 1) + 2:4 * nn] @ (_TURN.T @ p.end_k @ _TURN).T
        out[p.name] = {"moment_nm": round(float(M.max()) / 1000.0, 2),
                       "ends_nm": round(float(ends.max()) / 1000.0, 2),
                       "mid_nm": round(float(mid.max()) / 1000.0, 2),
                       "shear_n": round(float(V.max()), 1),
                       "foot_nm": round(float(np.hypot(*foot_m.T).max()) / 1000.0, 2),
                       "tip_nm": round(float(np.hypot(*tip_m.T).max()) / 1000.0, 2),
                       "end_n": round(float(np.hypot(*end_f.T).max()), 1),
                       "forces": {"box": {"foot": -foot_f, "end": end_f}}}
    return out


# -- the frame plates' members ---------------------------------------------------------


def members(xy: Mapping[str, np.ndarray], pairs: Sequence[tuple[str, str]],
            widths: Mapping[tuple[str, str], float], t: float, yield_mpa: float,
            loads: Mapping[str, np.ndarray], held: Sequence[str],
            moment_at: tuple[str, np.ndarray] | None = None,
            tied: Sequence[str] = ()) -> dict:
    """One frame plate's members (``pairs`` of node names at ``xy``, each ``widths[pair]``
    wide, ``t`` thick) as a plane frame under ``loads`` (node -> (cases, 2) N; ``moment_at``
    a node and its (cases,) moment, N·mm), standing on ``held`` (its feet: the first
    pinned, the others held up only; else, a plate floating on its posts in equilibrium,
    held at its first node): each member's worst in-plane stress
    and safety factor, and its least factor on buckling out of the plane in compression:
    Euler between its nodes (:data:`MEMBER_K`) where both are ``tied`` (held across the
    plate: a post's, a foot's), else the lesser of that and its tilt on what holds an untied
    node across, the other members there as cantilevers from their far nodes (``k L``; two
    untied nodes, their springs in series)."""
    names = list(xy)
    segs = _split(names, xy, pairs, tol=1.0)
    used = sorted({p for s in segs for p in s})
    idx = {p: i for i, p in enumerate(used)}
    nodes = np.array([xy[p] for p in used], float)
    m = len(next(iter(loads.values()))) if loads else 1
    F = np.zeros((m, len(used), 3))
    for p, f in loads.items():
        if p not in idx:
            raise ValueError(f"a load at {p}, no member's node")
        F[:, idx[p], :2] += f
    if moment_at is not None:
        F[:, idx[moment_at[0]], 2] += moment_at[1]
    width = {}
    for a, b in pairs:
        for s in _split(names, xy, [(a, b)], tol=1.0):
            width[s] = widths.get((a, b), widths.get((b, a), 24.0))
    elems = [(idx[a], idx[b], E_ALU * width[(a, b)] * t,
              E_ALU * t * width[(a, b)] ** 3 / 12) for a, b in segs]
    if held:    # the first foot pinned, the others on the board only (the plate, not the
        #         printed brackets, carries what its loads do among themselves)
        fixed = [3 * idx[held[0]], 3 * idx[held[0]] + 1, *(3 * idx[p] + 1 for p in held[1:])]
    else:
        fixed = [0, 1, 2]
    res = frame2d(nodes, elems, F, fixed)

    def length(a: str, b: str) -> float:
        return float(np.hypot(*(nodes[idx[b]] - nodes[idx[a]])))

    def across(n: str, but: tuple[str, str]) -> float:
        """What holds node ``n`` across the plate (N/mm): its other members, each a
        cantilever (weak axis) from its far node."""
        return sum(3 * E_ALU * width[s] * t ** 3 / 12 / length(*s) ** 3 for s in segs
                   if n in s and s != but)

    rows = []
    for e, (a, b) in enumerate(segs):
        w = width[(a, b)]
        N = res[:, e, 0]
        M = np.maximum(np.abs(res[:, e, 1]), np.abs(res[:, e, 2]))
        sig = np.abs(N) / (w * t) + M / (t * w * w / 6)
        L = length(a, b)
        pcr = math.pi ** 2 * E_ALU * w * t ** 3 / 12 / (MEMBER_K * L) ** 2
        free = [n for n in (a, b) if n not in tied]
        sway = None
        if free:
            ks = [across(n, (a, b)) for n in free]
            k = ks[0] if len(ks) == 1 else (ks[0] * ks[1] / max(ks[0] + ks[1], 1e-12))
            sway = k * L
            pcr = min(pcr, sway)
        comp = float(max(-N.min(), 0.0))
        rows.append({"member": f"{a}-{b}", "length_mm": round(L, 1), "width_mm": w,
                     "stress_mpa": round(float(sig.max()), 1),
                     "safety": round(yield_mpa / max(float(sig.max()), 1e-9), 2),
                     "compression_n": round(comp, 1), "euler_n": round(pcr, 1),
                     "untied": free, "sway_n": round(sway, 1) if sway is not None else None,
                     "buckling": round(pcr / comp, 2) if comp > 1e-6 else None})
    return {"members": rows}
