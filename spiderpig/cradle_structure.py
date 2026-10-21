"""A decanting cradle's structure at its statics' force vectors: what the generic joint
rules of :mod:`spiderpig.strength` can't see (design review round 1, 2026-10-14).

:func:`spiderpig.cradle_report.statics_vectors` solves every joint's force *vector* per
crank angle. The generic rules use only the largest magnitude: a link of three or more pins
as a beam between its farthest two (``F L / 4``), a pillar as a beam between both frame
plates. Those miss a bell crank's lever (B's b2 as an L read 2.61 jammed: it held 0.80), a
cradle link's arms (B's b6 had only been rated in tension), and a pillar whose far plate
can't hold it (B's inner bar O1-O2 held O2 only along itself: about 0.4 jammed, not 1.33).
Here, from the vectors:

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
* **A side as one frame** (:func:`side_frame`, design review round 2, 2026-10-15): the
  posts and both frame plates together, each plate a grillage out of its plane
  (:class:`Grillage`: its members' weak-axis bending and twist), the outer one on its feet,
  the inner one on nothing but the posts, the posts stiff along their axes and turning
  with their plates at their ends. What the springs above (:func:`plate_spring`, each
  member clamped at its far node) can't hold: each plate's own equilibrium out of its
  plane, the inner plate twisting as a whole about a line of posts, and the gearbox's
  turn of the outer plate at the crank (``turns``). Each member end then rated out of the
  plane on the plate as cut (:func:`plate_rows`). (The round's review had shared each
  post's moment among the members at its node with the moment vector 90° off: a post bent
  along x turns its foot about y, ``test_a_posts_foot_turns_square_to_its_load``.)
* **The frame plates' members** (:func:`members`): each plate a plane frame of its members
  (the stand's pills, split where a post stands on one), loaded by the posts' feet (the
  outer plate: and by the gearbox at the crank, its bearing force and its torque) or their
  ends (the inner plate); the outer plate standing on its feet. Each member's in-plane
  stress and, in compression, its Euler load out of the plane between its nodes (pinned
  ends, the nodes held across by the box: :data:`MEMBER_K`), or its tilt where a node isn't
  (the gearbox's O: the side's own stiffness across there, or the other members there as
  cantilevers, ``k L``).

* **Round 3** (design review round 3, 2026-10-20): the side's solve exact (:func:`_solve`:
  Jacobi-scaled, no ridge; a frame not held raises :class:`SingularFrame`), the plates
  optionally at their mid-planes (:class:`SidePlates`' ``mid``), the plates' members'
  in-plane forces and the posts' axial ones as a geometric stiffness (:func:`_geo4`,
  :func:`side_frame_solve`): the side's buckling factor per angle and its second-order
  solve; each plate section's in-plane and out-of-plane stresses combined
  (:func:`combined_sf`, :func:`grillage_axial`); a link's chord past its boss.

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
#                         pinned at both nodes, the nodes held across by the box (its posts,
#                         its feet on the board)


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
                 width: Callable[[str, str, str], float] | None = None,
                 bosses: Mapping[str, Mapping[str, float]] | None = None) -> dict[str, dict]:
    """Every link of ``lk`` (``lk.links``) as a plane frame of its chords under one case's
    joint force vectors ``vecs`` (``cradle_report.statics_vectors``: ``joints`` its
    ``(point, body, other)``, ``pts`` the joints' world paths): per link, its worst chord end
    over the angles not in ``skip`` (the stress on the net section at the end's hole,
    ``holes`` by joint, and the safety factor), its axial force and moment there, and its
    least Euler factor of a chord in compression (out of the plane, pinned at its ends:
    the whole chord, a joint on it no brace). What holds the link besides its pins is spread
    over ``load_nodes[link]`` (default: all its joints). ``width(link, a, b)``: a chord's
    width where it isn't its section's. ``bosses`` (per link, a joint's boss radius,
    ``config.PRODUCT_BOSS_R``; a wider chord's end disc there counts as one): a chord end at
    a boss rated through its hole at the boss's width and, past it, where the boss meets the
    chord's edges, on the chord without a hole at the moment there."""
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
        own = dict((bosses or {}).get(name, {}))
        for p_, q_ in segs:     # a wider chord's end disc round a joint is a boss there too
            for j_ in (p_, q_):
                k_ = j_.rsplit(".", 1)[-1]
                own[k_] = max(own.get(k_, 0.0), wide[(p_, q_)] / 2)
        for e, (p, q) in enumerate(segs):
            N = res[:, e, 0]
            w = wide[(p, q)]
            L_pq = float(np.hypot(*(xy[idx[q]] - xy[idx[p]])))
            for end, M in ((p, res[:, e, 1]), (q, res[:, e, 2])):
                r_b = float(own.get(end.rsplit(".", 1)[-1], 0.0))
                cuts = [(M, w, holes.get(end, 0.0), 0.0)]
                if 2 * r_b > w:         # through its hole at the boss, and past the boss
                    s_b = min(math.sqrt(r_b * r_b - w * w / 4), L_pq)
                    x = s_b if end == p else L_pq - s_b
                    m_b = -res[:, e, 1] + (res[:, e, 1] + res[:, e, 2]) * x / L_pq
                    cuts = [(M, 2 * r_b, holes.get(end, 0.0), 0.0), (m_b, w, 0.0, s_b)]
                for M_, w_, d_, s_ in cuts:
                    sig, sf = _section_sf(N, M_, w_, sec.t, d_, sec.yield_mpa)
                    i = int(np.argmin(sf))
                    if sf[i] < worst["safety"]:
                        worst = {"safety": float(sf[i]), "chord": f"{p}-{q}", "at": end,
                                 "width_mm": w_, "moment_nm": round(float(abs(M_[i])) / 1000.0, 2),
                                 "axial_n": round(float(N[i]), 1),
                                 "stress_mpa": round(float(sig[i]), 1),
                                 "hole_mm": d_, "from_mm": round(s_, 1)}
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
    ea: float = 0.0             # its stiffness along its axis (N; 0: a solid round bar's of
    #                             its ``ei``), plate to plate (:class:`SidePlates`)


def _beam_k(ei: float, h: float) -> np.ndarray:
    """A beam element's stiffness in one plane, ``[u_a, s_a, u_b, s_b]`` (slope s = du/dz)."""
    hh = h * h
    return ei / h ** 3 * np.array([[12, 6 * h, -12, 6 * h], [6 * h, 4 * hh, -6 * h, 2 * hh],
                                   [-12, -6 * h, 12, -6 * h], [6 * h, 2 * hh, -6 * h, 4 * hh]])


_TURN = np.array([[0.0, -1.0], [1.0, 0.0]])   # a post's end's slopes (dx/dz, dy/dz) -> its turn


@dataclass
class SidePlates:
    """A side's two frame plates out of their plane, for :func:`posts` (design review round
    2's B1, 2026-10-15): each plate's members split at its nodes (``((a, b), width)``,
    nodes at ``xy``), ``t`` thick; the outer plate's ``feet`` on the base board, held across
    and against turning by their cleats; each post stiff along its axis (``Post.ea``). With
    them a post's ends are held by its plates as they are, and every member's share of
    every post's end moment is its stiffness's (:class:`Grillage`)."""

    xy: Mapping[str, np.ndarray]
    outer: Sequence[tuple[tuple[str, str], float]]
    inner: Sequence[tuple[tuple[str, str], float]]
    t: float
    feet: Sequence[str] = ()
    mid: bool = False           # the plates bending about their mid-planes, each post's end a
    #                             rigid t / 2 off it at the plate's face, so its shear bends the
    #                             plate too (design review round 3's minor 2); False: the
    #                             plates at their faces. A caller rates both (cradle_report)


def _post_node(p: Post) -> str:
    return p.name.split(":", 1)[-1]


class SingularFrame(ValueError):
    """A side frame that isn't held: its stiffness singular (a mechanism) or so near it that
    its solve can't be trusted (design review round 3's M4: a ridge on the diagonal had
    propped A's frame, its sway 1.11 mm where it is 1.91)."""


COND_MAX = 1e12         # the side frame's stiffness, Jacobi-scaled, at most this conditioned
RESIDUAL_MAX = 1e-6     # its solve's equilibrium residual over its loads, at most (the ridge
#                         it replaced had left A's inner plate 1.7 % out of equilibrium)


def _scale(K: np.ndarray) -> np.ndarray:
    """The Jacobi scaling of a stiffness (``1 / sqrt(diag)``); a dof with nothing holding it
    raises :class:`SingularFrame`."""
    d = np.abs(np.diag(K))
    if (d <= 0.0).any():
        raise SingularFrame(f"{int((d <= 0.0).sum())} of the side frame's degrees of freedom "
                            "have no stiffness")
    return 1.0 / np.sqrt(d)


def _solve(K: np.ndarray, F: np.ndarray, what: str = "the side frame") -> np.ndarray:
    """``K x = F`` exactly (``F`` (n, cases)), Jacobi-scaled, no ridge: a singular or
    ill-conditioned ``K`` raises :class:`SingularFrame`, and so does an equilibrium
    residual over :data:`RESIDUAL_MAX`."""
    s = _scale(K)
    Ks = K * s[:, None] * s[None, :]
    c = float(np.linalg.cond(Ks))
    if not math.isfinite(c) or c > COND_MAX:
        raise SingularFrame(f"{what} is not held (its stiffness, scaled, conditioned {c:.3g}): "
                            "a mechanism")
    x = s[:, None] * np.linalg.solve(Ks, s[:, None] * F)
    x += s[:, None] * np.linalg.solve(Ks, s[:, None] * (F - K @ x))     # (one refinement)
    _residual(K, x, F, what)
    return x


def _residual(K: np.ndarray, x: np.ndarray, F: np.ndarray, what: str) -> float:
    """``K x - F`` over the loads (the largest, per case: the frame's equilibrium), raising
    :class:`SingularFrame` over :data:`RESIDUAL_MAX`."""
    # (K one stiffness, or one per case: (cases, n, n), x and F (n, cases))
    r = K @ x - F if K.ndim == 2 else (K @ x.T[:, :, None])[:, :, 0].T - F
    scale = np.maximum(np.abs(F).max(axis=0), 1e-300)
    worst = float((np.abs(r).max(axis=0) / scale).max()) if F.size else 0.0
    if worst > RESIDUAL_MAX:
        raise SingularFrame(f"{what} out of equilibrium by {worst:.3g} of its loads")
    return worst


def _geo4(L: float) -> np.ndarray:
    """A beam's consistent geometric stiffness per newton of tension, ``[u_a, s_a, u_b,
    s_b]`` as :func:`_beam_k` (its axial force times this: P-Delta, and buckling)."""
    return np.array([[36.0, 3 * L, -36.0, 3 * L], [3 * L, 4 * L * L, -3 * L, -L * L],
                     [-36.0, -3 * L, 36.0, -3 * L], [3 * L, -L * L, -3 * L, 4 * L * L]]) / (30 * L)


@dataclass
class SideFrame:
    """:func:`side_frame_solve`'s result: the posts' rows (:func:`_posts`' ``out``), each
    plate's grillage, per case the side's buckling factor (``buckling``: the least positive
    ``lambda`` of ``(K + lambda K_G) phi = 0``, the plates' members' in-plane forces and the
    posts' axial ones times it; ``inf`` with nothing in compression; ``None`` without the
    members' forces), whether the solve was second order (P-Delta: its stiffness with the
    forces' ``K_G``), and its equilibrium residual over its loads."""

    posts: dict[str, dict]
    grillages: dict[str, Grillage]
    buckling: np.ndarray | None = None
    second_order: bool = False
    residual: float = 0.0


def _posts(side: Sequence[Post], length: float, vecs: np.ndarray,
           skip: np.ndarray | None = None, plates: SidePlates | None = None,
           push: Mapping[str, float] | None = None,
           turns: Mapping[str, np.ndarray] | None = None,
           axial: Mapping[str, Mapping[tuple[str, str], np.ndarray]] | None = None,
           second_order: bool = False) -> SideFrame:
    """The posts of one side on the box (the module doc) under one case's joint force
    vectors ``vecs``, over the angles not in ``skip``: one frame solve of every post (a
    beam, its foot fixed in the outer plate's plane and its end moving with the inner plate,
    a rigid body in its plane), each end held against turning by its plate: by
    ``foot_k``/``end_k`` (:func:`plate_spring`: each member there held at its far node), or
    with ``plates`` by the two plates themselves out of their plane (:class:`SidePlates`:
    each a grillage, the outer standing on its feet, the inner held by nothing but the
    posts, which are stiff along their axes; ``push``: a force across the plates at a
    post's node, N, the side's stiffness across). Per post: its worst bending moment and
    shear, its foot's moment (into the outer plate) and the force its end puts on the inner
    plate; and per angle its force on the outer plate (``forces["box"]["foot"]``) and the
    inner plate's on its end (``["end"]``), for :func:`members`; with ``plates``, each
    plate's grillage (its members' end forces, :class:`Grillage`).
    ``length``: between the plates' faces (mm).

    Solved exactly (:func:`_solve`: a frame that isn't held raises :class:`SingularFrame`).
    With ``axial`` (per plate, each grillage member's in-plane axial force per kept angle, N,
    tension positive, or its forces ``(cases, 3)``: :func:`grillage_axial`), the side's
    buckling factor per angle
    (:class:`SideFrame`), the posts' own axial forces (the first-order solve's) in it too;
    ``second_order`` then solves with that geometric stiffness (P-Delta) per angle."""
    keep = np.ones(len(vecs), bool) if skip is None else ~np.asarray(skip, bool)
    v = vecs[keep]
    m = len(v)
    L = length
    xy = np.array([p.xy for p in side], float)
    rho = xy - xy.mean(axis=0)
    # the global unknowns: the inner plate's (tx, ty, turn), with plates each plate's
    # grillage nodes' (w, rx, ry), then each post's own
    n_glob = 3
    gnodes: dict[str, dict[str, int]] = {}
    if plates is not None:
        for plate, segs in (("outer", plates.outer), ("inner", plates.inner)):
            gnodes[plate] = {}
            for p in sorted({q for (a, b), _ in segs for q in (a, b)}):
                gnodes[plate][p] = n_glob
                n_glob += 3
        for p in side:
            for plate in ("outer", "inner"):
                if _post_node(p) not in gnodes[plate]:
                    raise ValueError(f"post {p.name}: no node of the {plate} plate's members")
    plans = []
    for p in side:
        zs = sorted({0.0, L, *(a for _, _, a in p.loads if 0.0 < a < L)})
        nn = len(zs)
        # each node: ux, uy, sx, sy; the foot's u fixed, the end's u the plate's (with
        # plates, the ends' slopes are the plates' turns there)
        free = {}
        for k in range(nn):
            for d in range(4):
                if (k == 0 and d < 2) or (k == nn - 1 and d < 2):
                    continue
                if plates is not None and k in (0, nn - 1):
                    continue
                free[(k, d)] = n_glob
                n_glob += 1
        plans.append((zs, free))
    K = np.zeros((n_glob, n_glob))
    F = np.zeros((m, n_glob))
    els: dict[str, list] = {}
    fixed: list[int] = []
    post_ea: list[tuple[float, int, int]] = []
    if plates is not None:
        for plate, segs in (("outer", plates.outer), ("inner", plates.inner)):
            g0 = gnodes[plate]
            els[plate] = []
            for (a, b), w in segs:
                kg, kl, T, Le = _grillage_k(np.asarray(plates.xy[a], float),
                                            np.asarray(plates.xy[b], float), w, plates.t)
                dofs = [g0[a], g0[a] + 1, g0[a] + 2, g0[b], g0[b] + 1, g0[b] + 2]
                K[np.ix_(dofs, dofs)] += kg
                els[plate].append(((a, b), w, Le, dofs, kl, T))
        for f in plates.feet:           # on the board, its cleats either side of the rail
            g = gnodes["outer"].get(f)
            if g is not None:
                fixed += [g, g + 1, g + 2]
        for p in side:                  # each post along its axis, plate to plate
            ea = p.ea or math.sqrt(4 * math.pi * E_STEEL * p.ei)
            wo, wi = gnodes["outer"][_post_node(p)], gnodes["inner"][_post_node(p)]
            K[np.ix_([wo, wi], [wo, wi])] += ea / L * np.array([[1.0, -1.0], [-1.0, 1.0]])
            post_ea.append((ea, wo, wi))
        for name, f in (push or {}).items():
            F[:, gnodes["outer"][name]] += f
        for name, mv in (turns or {}).items():     # (all angles: kept here)
            g = gnodes["outer"][name]
            F[:, g + 1:g + 3] += np.asarray(mv, float)[keep]
    off = plates.t / 2 if plates is not None and plates.mid else 0.0
    maps = []
    for i, (p, (zs, free)) in enumerate(zip(side, plans, strict=True)):
        nn = len(zs)
        A = np.zeros((4 * nn, n_glob))           # the post's local dofs from the global ones
        for (k, d), g in free.items():
            A[4 * k + d, g] = 1.0
        r = rho[i]
        A[4 * (nn - 1), :3] = [1.0, 0.0, -r[1]]
        A[4 * (nn - 1) + 1, :3] = [0.0, 1.0, r[0]]
        if plates is not None:          # the slopes (sx, sy) = (ry, -rx) of its plate's turn
            for k, plate, sgn in ((0, "outer", 1.0), (nn - 1, "inner", -1.0)):
                g = gnodes[plate][_post_node(p)]
                A[4 * k + 2, g + 2] = 1.0
                A[4 * k + 3, g + 1] = -1.0
                if off:                 # its end's face t / 2 off the plate's mid-plane:
                    A[4 * k, g + 2] += sgn * off        # the plate's turn moves it across
                    A[4 * k + 1, g + 1] -= sgn * off
        Kl = np.zeros((4 * nn, 4 * nn))
        for e in range(nn - 1):
            ke = _beam_k(p.ei, zs[e + 1] - zs[e])
            for plane in (0, 1):              # x: (ux, sx), y: (uy, sy)
                idx = [4 * e + plane, 4 * e + 2 + plane, 4 * (e + 1) + plane,
                       4 * (e + 1) + 2 + plane]
                Kl[np.ix_(idx, idx)] += ke
        if plates is None:
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
    # a mode nothing holds and nothing loads (a lone post's inner plate turning about it):
    # held exactly, as it is indeterminate and moves nothing; one a load moves is a mechanism
    # (:func:`_solve` raises)
    fixed += [g for g in range(n_glob) if g not in set(fixed) and not K[g].any()
              and not F[:, g].any()]
    fixed_set = set(fixed)
    free_g = [g for g in range(n_glob) if g not in fixed_set]
    Kf = K[np.ix_(free_g, free_g)]
    U = np.zeros((m, n_glob))
    U[:, free_g] = _solve(Kf, F[:, free_g].T).T
    lam = None
    second = False
    if axial is not None and plates is not None and m:
        # the geometric stiffness per angle: each plate member's in-plane force on its own
        # bending out of the plate, each post's axial force on its bending across
        units, forces = [], []
        for plate in ("outer", "inner"):
            got = axial.get(plate) or {}
            for (a, b), _w, Le, dofs, _kl, T in els[plate]:
                n_ab = got.get((a, b))
                if n_ab is None:
                    continue
                g6 = np.zeros((6, 6))
                g6[np.ix_([0, 2, 3, 5], [0, 2, 3, 5])] = _geo4(Le)
                gg = np.zeros((n_glob, n_glob))
                gg[np.ix_(dofs, dofs)] = T.T @ g6 @ T
                units.append(gg)
                n_ab = np.asarray(n_ab, float)
                forces.append((n_ab[:, 0] if n_ab.ndim == 2 else n_ab).reshape(m))
        for (ea, wo, wi), (zs, _), (A, _, _) in zip(post_ea, plans, maps, strict=True):
            nn = len(zs)
            gl = np.zeros((4 * nn, 4 * nn))
            for e in range(nn - 1):
                g4 = _geo4(zs[e + 1] - zs[e])
                for plane in (0, 1):
                    idx = [4 * e + plane, 4 * e + 2 + plane, 4 * (e + 1) + plane,
                           4 * (e + 1) + 2 + plane]
                    gl[np.ix_(idx, idx)] += g4
            units.append(A.T @ gl @ A)
            forces.append(ea / L * (U[:, wi] - U[:, wo]))     # tension positive
        if units:
            G = np.stack([u_[np.ix_(free_g, free_g)] for u_ in units])
            N = np.stack(forces, axis=1)                        # (cases, elements)
            s = _scale(Kf)
            Ks = Kf * s[:, None] * s[None, :]
            Li = np.linalg.inv(np.linalg.cholesky(Ks))
            # (K + lambda K_G) phi = 0 as the symmetric L^-1 (-S K_G S) L^-T y = y / lambda,
            # K_G linear in the forces: each element's matrix once, then summed per case
            Be = -(Li[None, :, :] @ (G * s[None, :, None] * s[None, None, :])
                   @ Li.T[None, :, :])
            mu = np.linalg.eigvalsh(np.tensordot(N, Be, axes=(1, 0))).max(axis=1)
            lam_c = np.where(mu > 1e-12, 1.0 / np.maximum(mu, 1e-300), np.inf)
            lam = lam_c
            # P-Delta where the side stands (an angle at or past its buckling keeps the
            # first-order solve: its buckling factor, under 1, is its rating there)
            up = lam_c > 1.0 + 1e-6
            rows_up = np.nonzero(up)[0]
            if second_order and len(rows_up):
                Kc = Kf[None, :, :] + np.tensordot(N[up], G, axes=(1, 0))
                Uc = np.linalg.solve(Kc * s[None, :, None] * s[None, None, :],
                                     (s[:, None] * F[up][:, free_g].T).T[:, :, None])[:, :, 0]
                fg = np.asarray(free_g, dtype=np.intp)
                U[np.ix_(rows_up, fg)] = Uc * s[None, :]
                _residual(Kc, U[np.ix_(rows_up, fg)].T, F[up][:, free_g].T,
                          "the side frame (P-Delta)")
                second = True
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
        # the moments its ends put on its plates, as moment vectors in the plates' plane
        # (the turn's space: the slopes' generalized moments turned by _TURN), N·mm per angle
        if plates is None:
            foot_m = ul[:, 2:4] @ (p.foot_k @ _TURN).T
            tip_m = ul[:, 4 * (nn - 1) + 2:4 * nn] @ (p.end_k @ _TURN).T
        else:
            foot_m = -res[:, 2:4] @ _TURN.T
            tip_m = -res[:, 4 * (nn - 1) + 2:4 * nn] @ _TURN.T
        row = {"moment_nm": round(float(M.max(initial=0.0)) / 1000.0, 2),
               "ends_nm": round(float(ends.max(initial=0.0)) / 1000.0, 2),
               "mid_nm": round(float(mid.max(initial=0.0)) / 1000.0, 2),
               "shear_n": round(float(V.max(initial=0.0)), 1),
               "foot_nm": round(float(np.hypot(*foot_m.T).max(initial=0.0)) / 1000.0, 2),
               "tip_nm": round(float(np.hypot(*tip_m.T).max(initial=0.0)) / 1000.0, 2),
               "end_n": round(float(np.hypot(*end_f.T).max(initial=0.0)), 1),
               "forces": {"box": {"foot": -foot_f, "end": end_f},
                          "turn": {"foot": foot_m, "end": tip_m}}}
        # its links' nodes against its end (the inner plate there), in the plates' plane:
        # how far what rides on it moves past the inner plate (a tie rod passing its bar)
        drift = np.zeros(m)
        for k in range(1, nn - 1):
            drift = np.maximum(drift, np.hypot(ul[:, 4 * k] - ul[:, 4 * (nn - 1)],
                                               ul[:, 4 * k + 1] - ul[:, 4 * (nn - 1) + 1]))
        row["drift_mm"] = round(float(drift.max(initial=0.0)), 3)
        if plates is not None:                  # its axial force (tension positive)
            wo, wi = gnodes["outer"][_post_node(p)], gnodes["inner"][_post_node(p)]
            ea = p.ea or math.sqrt(4 * math.pi * E_STEEL * p.ei)
            row["forces"]["axial"] = ea / L * (U[:, wi] - U[:, wo])
        out[p.name] = row
    grills: dict[str, Grillage] = {}
    if plates is not None:
        for plate in ("outer", "inner"):
            members = {}
            for (a, b), w, Le, dofs, kl, T in els[plate]:
                members[(a, b)] = (w, Le, (U[:, dofs] @ T.T) @ kl.T)
            grills[plate] = Grillage(members, {q: U[:, g:g + 3]
                                               for q, g in gnodes[plate].items()})
    resid = 0.0
    if m and not second:
        resid = _residual(Kf, U[:, free_g].T, F[:, free_g].T, "the side frame")
    return SideFrame(out, grills, lam, second, resid)


def posts(side: Sequence[Post], length: float, vecs: np.ndarray,
          skip: np.ndarray | None = None) -> dict[str, dict]:
    """The posts of one side on the box, each end held by its plate's springs
    (``Post.foot_k``, ``end_k``: :func:`plate_spring`); :func:`side_frame` holds them by the
    plates themselves."""
    return _posts(side, length, vecs, skip).posts


def side_frame(side: Sequence[Post], length: float, vecs: np.ndarray, plates: SidePlates,
               skip: np.ndarray | None = None, push: Mapping[str, float] | None = None,
               turns: Mapping[str, np.ndarray] | None = None
               ) -> tuple[dict[str, dict], dict[str, Grillage]]:
    """A side's posts on its two plates as one frame (:func:`_posts` with ``plates``): the
    posts' rows and each plate's grillage (``"outer"``, ``"inner"``). ``turns``: moment
    vectors in the outer plate's plane at its nodes, per angle (N·mm, every angle: ``skip``
    applies), what else turns it: the gearbox at the crank, its shaft carrying the rod's
    force at the rod's height."""
    got = _posts(side, length, vecs, skip, plates, push, turns)
    return got.posts, got.grillages


def side_frame_solve(side: Sequence[Post], length: float, vecs: np.ndarray,
                     plates: SidePlates, skip: np.ndarray | None = None,
                     turns: Mapping[str, np.ndarray] | None = None,
                     axial: Mapping[str, Mapping[tuple[str, str], np.ndarray]] | None = None,
                     second_order: bool = True) -> SideFrame:
    """:func:`side_frame` with the plates' members' in-plane forces (``axial``: per plate,
    per grillage member, N per kept angle, :func:`grillage_axial`): the side's buckling
    factor per angle (design review round 3's M2) and, ``second_order``, the solve with
    their geometric stiffness (P-Delta: its minor 1)."""
    return _posts(side, length, vecs, skip, plates, None, turns, axial, second_order)


# -- a frame plate out of its plane ------------------------------------------------------


def _grillage_k(pa: np.ndarray, pb: np.ndarray, w: float, t: float
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """A plate member as a grillage element (out of the plate's plane), dofs ``(w, rx, ry)``
    at each end (``w`` across the plate, ``r`` the turn's vector in the plane): its weak
    axis bending (``E w t^3 / 12``) and its twist (``G w t^3 / 3``). Returns the global
    stiffness, the local one and the transform (local ``(w, twist, slope)`` per end), and
    the length."""
    d = pb - pa
    L = float(np.hypot(*d))
    c, s = d / L
    EI = E_ALU * w * t ** 3 / 12
    GJ = G_ALU * w * t ** 3 / 3
    kl = np.zeros((6, 6))
    kl[np.ix_([0, 2, 3, 5], [0, 2, 3, 5])] = _beam_k(EI, L)
    kl[np.ix_([1, 4], [1, 4])] = GJ / L * np.array([[1.0, -1.0], [-1.0, 1.0]])
    T = np.zeros((6, 6))
    for o in (0, 3):            # twist = r . e; the slope dw/dx along e = r . (s, -c)
        T[o, o] = 1.0
        T[o + 1, o + 1:o + 3] = [c, s]
        T[o + 2, o + 1:o + 3] = [s, -c]
    return T.T @ kl @ T, kl, T, L


@dataclass
class Grillage:
    """A frame plate's members as a grillage (:func:`grillage`): per member ``(a, b)``, its
    width, length, and per case its end forces in its own frame, ``(cases, 6)``: the shear
    ``V``, the twist ``T`` and the bending moment ``M`` at ``a`` then at ``b`` (N, N·mm,
    the element's end forces: the moment along it is ``-M_a + V_a s``); per node, its
    displacement across the plate and its turn, ``(cases, 3)``."""

    members: dict[tuple[str, str], tuple[float, float, np.ndarray]]
    nodes: dict[str, np.ndarray]

    def moment_at(self, a: str, b: str, s: float) -> np.ndarray:
        """The bending moment's size across member ``a``-``b`` ``s`` mm from ``a``, per
        case (N·mm)."""
        if (a, b) in self.members:
            _, _, f = self.members[(a, b)]
            return np.abs(-f[:, 2] + f[:, 0] * s)
        _, L, f = self.members[(b, a)]
        return np.abs(-f[:, 2] + f[:, 0] * (L - s))

    def twist(self, a: str, b: str) -> np.ndarray:
        """The member's torque per case (N·mm; the same along it)."""
        _, _, f = self.members[(a, b)] if (a, b) in self.members else self.members[(b, a)]
        return np.abs(f[:, 1])


def out_of_plane_sf(M: np.ndarray, T: np.ndarray, w: float, t: float, hole: float,
                    yield_mpa: float) -> tuple[np.ndarray, np.ndarray]:
    """A plate strip ``w`` wide with a hole ``hole`` across it, bent across its weak axis
    by ``M`` and twisted by ``T`` (N·mm, per case): the von Mises stress on the net section
    (``6 M / (w_n t^2)``, ``3 T / (w_n t^2)``) and the safety factor."""
    wn = max(w - hole, 1e-9)
    sig = 6.0 * M / (wn * t * t)
    tau = 3.0 * T / (wn * t * t)
    vm = np.sqrt(sig ** 2 + 3.0 * tau ** 2)
    return vm, yield_mpa / np.maximum(vm, 1e-9)


def strip_root(n: str, far: str, w: float, others: Sequence[tuple[str, float]],
               xy: Mapping[str, np.ndarray], boss: float = 0.0) -> float:
    """How far from node ``n`` member ``n``-``far`` (``w`` wide) leaves the plate's material
    round ``n``: where both its edges come out of the other members there (``others``: each
    member's far node and width, a pill round its axis) and of the boss (``boss`` its
    radius), so its own strip is all that carries it on (mm)."""
    p0 = np.asarray(xy[n], float)
    d = np.asarray(xy[far], float) - p0
    L = float(np.hypot(*d))
    e = d / L
    nrm = np.array([-e[1], e[0]])
    h = w / 2

    def inside(q: np.ndarray) -> bool:
        if float(np.hypot(*(q - p0))) < boss - 1e-9:
            return True
        for o, wo in others:
            b = np.asarray(xy[o], float) - p0
            u = float(np.clip(((q - p0) @ b) / max(float(b @ b), 1e-18), 0.0, 1.0))
            if float(np.hypot(*(q - p0 - u * b))) < wo / 2 - 1e-9:
                return True
        return False

    s_out = 0.0
    for s in np.linspace(0.0, L, 401):
        if any(inside(p0 + s * e + sign * h * nrm) for sign in (-1.0, 1.0)):
            s_out = float(s)
    return min(s_out, L)


_ROOTS: dict = {}       # strip_root by its inputs (the plate's shape: the same every call)


def combined_sf(N: np.ndarray, M_in: np.ndarray, M_out: np.ndarray, T: np.ndarray,
                w: float, t: float, hole: float, yield_mpa: float
                ) -> tuple[np.ndarray, np.ndarray]:
    """A plate strip ``w`` x ``t`` with a hole ``hole`` across it under its in-plane axial
    force ``N`` and moment ``M_in`` and its moment ``M_out`` across its weak axis and twist
    ``T`` out of the plane (N, N·mm, per case), on the net section (design review round 3's
    minor 1: they had been rated apart): at its edge, where the twist's shear is nothing,
    ``|N| / A + |M_in| / Z_in + |M_out| / Z_out``; at its face's middle, where the in-plane
    moment is nothing, the von Mises of ``|N| / A + |M_out| / Z_out`` and the twist's
    ``3 T / (w_n t^2)``; the larger, and the safety factor."""
    wn = max(w - hole, 1e-9)
    s_n = np.abs(N) / (wn * t)
    s_out = np.abs(M_out) / (wn * t * t / 6.0)
    s_in = np.abs(M_in) / (t * (w ** 3 - hole ** 3) / (6.0 * w))
    tau = 3.0 * np.abs(T) / (wn * t * t)
    vm = np.maximum(s_n + s_in + s_out, np.sqrt((s_n + s_out) ** 2 + 3.0 * tau ** 2))
    return vm, yield_mpa / np.maximum(vm, 1e-9)


def plate_rows(g: Grillage, segs: Sequence[tuple[tuple[str, str], float]],
               xy: Mapping[str, np.ndarray], t: float, yield_mpa: float,
               holes: Mapping[str, float], web: float,
               inplane: Mapping[tuple[str, str], np.ndarray] | None = None) -> dict[str, dict]:
    """Each member end of a frame plate rated out of its plane from its grillage ``g``
    (:func:`side_frame`): its bending across its weak axis and its twist (von Mises,
    :func:`out_of_plane_sf`) where it is weakest near the end, on the cut plate: at a node on
    a straight run (a column's on a leg, a foot's screw) the net section through the hole
    (``holes`` by node; the boss round it, ``web`` past the hole, where wider than the
    strip); at a node where members meet, at its strip's root (:func:`strip_root`: where it
    leaves the others and the boss), on the strip, at the moment there. With ``inplane``
    (per grillage member ``(a, b)``, its in-plane axial force and end moments per case,
    ``(cases, 3)``: :func:`grillage_axial` of :func:`members`' ``forces``), the member's
    in-plane stress at that section with it (:func:`combined_sf`). Per member end
    (``"a-b at n"``): the worst over the cases, its stress, safety factor, moment and twist
    (N·m) and the section."""
    out: dict[str, dict] = {}
    for (a, b), (w, L, _) in g.members.items():
        for n, far in ((a, b), (b, a)):
            others = [((q if o == n else o), wo) for (o, q), wo in segs
                      if n in (o, q) and {o, q} != {a, b}]
            hole = float(holes.get(n, 0.0))
            boss = hole / 2 + web if hole else 0.0
            e = (np.asarray(xy[far], float) - xy[n]) / L
            straight = all(abs(abs(float(e @ ((np.asarray(xy[o], float) - xy[n])
                                              / float(np.hypot(*(np.asarray(xy[o], float)
                                                                 - xy[n])))))) - 1.0) < 1e-3
                           for o, _ in others)
            T = g.twist(n, far)
            if straight:
                s, ww, h = 0.0, max(w, 2 * boss), hole
                where = f"the net section through its {hole:g} mm hole" if hole else "the strip"
            else:
                key = (n, far, w, boss, tuple(others),
                       *(tuple(np.round(np.asarray(xy[q], float), 6))
                         for q in (n, far, *(o for o, _ in others))))
                if key not in _ROOTS:
                    if len(_ROOTS) > 4096:
                        _ROOTS.clear()
                    _ROOTS[key] = strip_root(n, far, w, others, xy, boss)
                s, ww, h = _ROOTS[key], w, 0.0
                where = "its strip's root"
            M = g.moment_at(n, far, s)
            f = (inplane or {}).get((a, b))
            N_ = M_in = None
            if f is None:
                vm, sf = out_of_plane_sf(M, T, ww, t, h, yield_mpa)
            else:
                x = s if n == a else L - s          # (the beam's moment from a: -m_a + ...)
                N_ = f[:, 0]
                M_in = -f[:, 1] + (f[:, 1] + f[:, 2]) * x / L
                vm, sf = combined_sf(N_, M_in, M, T, ww, t, h, yield_mpa)
            i = int(np.argmin(sf))
            row = {"member": f"{a}-{b} at {n} (out of the plate"
                             + (", with its in-plane stress)" if f is not None else ")"),
                   "width_mm": round(ww, 1), "at_mm": round(s, 1), "section": where,
                   "moment_nm": round(float(M[i]) / 1000, 2),
                   "twist_nm": round(float(T[i]) / 1000, 2),
                   "stress_mpa": round(float(vm[i]), 1),
                   "safety": round(float(sf[i]), 2), "compression_n": 0.0, "buckling": None}
            if N_ is not None and M_in is not None:
                row.update(axial_n=round(float(N_[i]), 1),
                           inplane_nm=round(float(abs(M_in[i])) / 1000, 2))
            out[f"{a}-{b} at {n}"] = row
    return out


def grillage_axial(segs: Sequence[tuple[tuple[str, str], float]],
                   xy: Mapping[str, np.ndarray],
                   forces: Mapping[tuple[str, str], np.ndarray], tol: float = 1.0
                   ) -> dict[tuple[str, str], np.ndarray]:
    """Each grillage member's in-plane forces (``segs``: a plate's, :class:`SidePlates`) from
    :func:`members`' (``forces``: its own segments' axial force and end moments per case,
    ``(cases, 3)``, which may be split elsewhere): per member ``(a, b)``, ``(cases, 3)``,
    its axial force (the same along a segment) and its end moments at ``a`` and ``b`` (the
    segment holding it, its moment linear between its ends; a member on none is left
    out)."""
    out: dict[tuple[str, str], np.ndarray] = {}
    for (a, b), _w in segs:
        pa, pb = np.asarray(xy[a], float), np.asarray(xy[b], float)
        for (p, q), f in forces.items():
            pp, pq = np.asarray(xy[p], float), np.asarray(xy[q], float)
            d = pq - pp
            Lpq = float(np.hypot(*d))
            u = []
            for c in (pa, pb):
                t_ = float((c - pp) @ d) / (Lpq * Lpq)
                if not -1e-6 <= t_ <= 1 + 1e-6 or float(np.hypot(*(pp + t_ * d - c))) > tol:
                    break
                u.append(t_)
            if len(u) < 2:
                continue
            # the segment's beam moment along it, -m_p + (m_p + m_q) x, at a and at b, back
            # to the end moments of a member a-b (m_a = -M(a), m_b = M(b)); flipped (and the
            # moments' sense with it) when the member runs q to p
            Ma = -f[:, 1] + (f[:, 1] + f[:, 2]) * u[0]
            Mb = -f[:, 1] + (f[:, 1] + f[:, 2]) * u[1]
            out[(a, b)] = (np.stack([f[:, 0], -Ma, Mb], axis=-1) if u[1] >= u[0] else
                           np.stack([f[:, 0], Ma, -Mb], axis=-1))
            break
    return out


# -- the frame plates' members ---------------------------------------------------------


def members(xy: Mapping[str, np.ndarray], pairs: Sequence[tuple[str, str]],
            widths: Mapping[tuple[str, str], float], t: float, yield_mpa: float,
            loads: Mapping[str, np.ndarray], held: Sequence[str],
            moment_at: tuple[str, np.ndarray] | None = None,
            tied: Sequence[str] = (), across_k: Mapping[str, float] | None = None) -> dict:
    """One frame plate's members (``pairs`` of node names at ``xy``, each ``widths[pair]``
    wide, ``t`` thick) as a plane frame under ``loads`` (node -> (cases, 2) N; ``moment_at``
    a node and its (cases,) moment, N·mm), standing on ``held`` (its feet: the first
    pinned, the others held up only; else, a plate floating on its posts in equilibrium,
    held at its first node): each member's worst in-plane stress
    and safety factor, and its least factor on buckling out of the plane in compression:
    Euler between its nodes (:data:`MEMBER_K`) where both are ``tied`` (held across the
    plate: a post's, a foot's), else the lesser of that and its tilt on what holds an untied
    node across, the other members there as cantilevers from their far nodes (``k L``; two
    untied nodes, their springs in series), or the stiffness across ``across_k`` gives a node
    (N/mm: the whole side's, :func:`side_frame` pushed there; design review round 2's minor 3:
    the cantilevers' 2.9 N/mm at B's O had ranked O-Fr's sway first, the side holds O at 11)."""
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
            ks = [(across_k or {}).get(n) or across(n, (a, b)) for n in free]
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
    return {"members": rows, "forces": {s_: res[:, e] for e, s_ in enumerate(segs)}}
