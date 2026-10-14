"""Packing real shapes in a box, on a voxel grid: what :mod:`spiderpig.cradle_pack` stacks
its pieces with and ``spiderpig pack-inspect`` (:mod:`spiderpig.tools.pack_inspect`)
judges a packing by. numpy and scipy only (no engine import): a piece is its parts'
triangle meshes, its mass and its centre of mass.

**The voxels** (:func:`voxelize`): a part's tessellation, filled solid, on a grid of
:data:`CELL` mm cells: every cell the surface passes through (points sampled on each
triangle at most half a cell apart, its edges too) and every cell whose centre is inside
(a ray up each column of cell centres, its crossings paired: a part is a closed mesh; a
column with an odd count keeps its surface cells only and is counted). A cell any part
touches is full, so two pieces whose cells are apart are apart. Many parts at once: each
one's inside is its own (a screw in a plate's hole fills the hole, not the plate).

**Turning** (:class:`Vox`, :data:`ROTATIONS`): a piece is voxelized once in its own frame
(a levelled piece after its turn about z); each of the cube's 24 rotations of that grid is
the array transposed and flipped, exactly.

**The drop** (:class:`Bin`): the box is a heightmap (``top``, cells, per column of the
footprint) of every placed piece grown by one cell all round (:data:`CLEAR`: the foam
between two pieces). A piece in an orientation is its bottom and top per column
(:class:`Shape`: ``lo`` and ``hi``); dropped straight down at a column offset it comes to
rest at the least height where its bottom clears ``top`` in every column it covers (a max-
plus correlation, :func:`rest_heights`). Pieces go in one after another, each dropped from
above, so the order they are dropped in is an order they can be put in: none is both over
and under another. A piece's open parts take what drops into them (a truss's triangles, a
panel's bay between its links), and a piece dropped over smaller ones rests on them.

**Where** (:meth:`Packer.drop`, :meth:`Bin.candidates`): of the orientations and offsets a
piece may take (all, or what its plan's step allows), the one that grows the box least,
then its centre of mass lowest, then nearest the corner it is told (or any), where it rests
(:func:`support`): its centre of mass over the convex hull of the columns where it touches
what is under it, or the floor (within :data:`SETTLE`: dropped straight, a piece meets one
point first and tips onto the next ones), else over foam pads filling gaps up to
:data:`PAD_MAX` under it (named: where, how tall), else not at all (flagged; it would tip).

**Plans and search** (:class:`Plan`, :func:`pack`, :func:`search`): a plan says per piece
its orientation (a named face down, a turn about the vertical, or any), where it goes (a
corner, on or into another piece, a region) and what goes in first; what it leaves free
the packer picks greedily (:class:`Packer`). :func:`search` anneals the order, the
orientations and the corners against :func:`score` (the box's volume, the pieces that tip
or need pads, the mass centre's height, the box's height) within a time budget, in worker
processes.

**Metrics** (:func:`metrics`): any layout, its pieces voxelized afresh on one grid over the
box (:func:`occupancy`), whoever packed it: the box, the true fill (the parts' own volumes
over the box's), the air per layer, how each piece rests (:func:`support`), the real-shape
overlaps (cells two pieces share, then measured on :data:`FINE` cells whose centres are
inside both), the order's faults (a piece over another that goes in after it), the mass
centre's height, the order.

Not MuJoCo: settling the layout under gravity would need each piece's collision shape, and
convex hulls of the parts fill a truss's triangles and a panel's bay (every nested piece
would start inside another's hull); boxes over the voxels' columns would be honest but
thousands of geoms a module. :func:`support` answers whether a piece floats without it.
"""

from __future__ import annotations

import copy
import itertools
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from itertools import permutations

import numpy as np

CELL = 3.0          # mm: the packing grid
CLEAR = 3.0         # mm between two pieces (foam, cardboard): one cell's growth
SETTLE = 6.0        # mm a piece may sink at a contact as it settles (tips onto its next one)
BIG = 1 << 20       # a profile's "nothing here"
RAY_JITTER = (0.000123456789, 0.000234567891)   # the column rays off the cells' centres


# -- rotations -------------------------------------------------------------------------------


def _rotations() -> list[np.ndarray]:
    """The cube's 24 proper rotations (signed permutation matrices of determinant +1),
    the identity first, then by their matrices' entries: a fixed order."""
    out = []
    for perm in permutations(range(3)):
        for signs in np.ndindex(2, 2, 2):
            m = np.zeros((3, 3))
            for i, j in enumerate(perm):
                m[i, j] = 1.0 if signs[i] == 0 else -1.0
            if np.linalg.det(m) > 0:
                out.append(m)
    out.sort(key=lambda m: (not np.allclose(m, np.eye(3)), tuple(-m.ravel())))
    return out


ROTATIONS: list[np.ndarray] = _rotations()


def rot_about(axis: str, deg: float) -> np.ndarray:
    """The rotation by ``deg`` about ``axis`` (x, y or z), right-handed."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    c, s = round(c, 15), round(s, 15)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def describe(ri: int, faces: dict[str, np.ndarray] | None = None) -> tuple[str, int]:
    """Rotation ``ri`` (:data:`ROTATIONS`) as a plan's words: the face (a named one of
    ``faces``, else its own axis) that it turns down, and the spin from there (degrees)."""
    r = ROTATIONS[ri]
    ax = np.rint(r.T @ np.array([0.0, -1.0, 0.0]))
    name = next((k for k, v in (faces or {}).items() if np.allclose(np.rint(v), ax)), None)
    if name is None:
        i = int(np.flatnonzero(ax)[0])
        name = ("+" if ax[i] > 0 else "-") + "xyz"[i]
    m = r @ down_rotation(ax).T
    return name, round(math.degrees(math.atan2(m[0, 2], m[0, 0]))) % 360


def down_rotation(down: np.ndarray, spin: float = 0.0) -> np.ndarray:
    """The rotation (one of :data:`ROTATIONS`) that turns axis vector ``down`` (a unit
    axis of the piece's frame, ±x, ±y or ±z) to -y, then ``spin`` degrees about y (a
    quarter turn): the first such in :data:`ROTATIONS`' order for spin 0."""
    want = np.array([0.0, -1.0, 0.0])
    cands = [r for r in ROTATIONS if np.allclose(r @ down, want)]
    base = cands[0]
    return np.rint(rot_about("y", spin)) @ base


# -- meshes and voxels -----------------------------------------------------------------------

Mesh = tuple[np.ndarray, np.ndarray]        # positions (n, 3), triangles (m, 3) int


def box_mesh(lo, hi) -> Mesh:
    """A closed box's 12 triangles (outward)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    p = np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1], (lo, hi)[(i >> 2) & 1][2]]
                  for i in range(8)])
    t = np.array([[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4],
                  [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]])
    return p, t


def mesh_volume(pos: np.ndarray, tri: np.ndarray) -> tuple[float, np.ndarray]:
    """A closed mesh's volume (mm^3) and centroid (the divergence theorem; its sign: the
    triangles' winding outward)."""
    a, b, c = (pos[tri[:, i]] for i in range(3))
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = float(v6.sum()) / 6.0
    if abs(vol) < 1e-12:
        return 0.0, pos.mean(axis=0) if len(pos) else np.zeros(3)
    cen = ((a + b + c) * v6[:, None]).sum(axis=0) / (4.0 * v6.sum())
    return abs(vol), cen


@dataclass
class Vox:
    """A voxel grid: ``occ`` (bool, x y z), its cell ``(0, 0, 0)``'s lower corner ``org``
    (mm), ``cell`` across."""

    occ: np.ndarray
    org: np.ndarray
    cell: float = CELL

    @property
    def hi(self) -> np.ndarray:
        return self.org + np.array(self.occ.shape) * self.cell

    def turned(self, rot: np.ndarray) -> Vox:
        """The grid turned by ``rot`` (one of :data:`ROTATIONS`): transposed and flipped,
        its lower corner the turned grid's."""
        r = np.rint(rot).astype(int)
        perm = [int(np.flatnonzero(r[k])[0]) for k in range(3)]
        a = np.transpose(self.occ, perm)
        flips = tuple(k for k in range(3) if r[k, perm[k]] < 0)
        if flips:
            a = np.flip(a, flips)
        corners = np.array([[(self.org, self.hi)[i & 1][0], (self.org, self.hi)[(i >> 1) & 1][1],
                             (self.org, self.hi)[(i >> 2) & 1][2]] for i in range(8)])
        return Vox(np.ascontiguousarray(a), (corners @ r.T).min(axis=0), self.cell)


def _expand(counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``(owner, local)``: each index ``i`` repeated ``counts[i]`` times, and 0..counts-1."""
    owner = np.repeat(np.arange(len(counts)), counts)
    starts = np.cumsum(counts) - counts
    local = np.arange(int(counts.sum())) - np.repeat(starts, counts)
    return owner, local


def _chunks(counts: np.ndarray, budget: int = 1 << 21):
    """Index ranges of ``counts`` summing to about ``budget`` each."""
    cum = np.cumsum(counts)
    s = 0
    while s < len(counts):
        base = int(cum[s - 1]) if s else 0
        e = max(int(np.searchsorted(cum, base + budget, side="right")), s + 1)
        yield s, e
        s = e


def surface_points(pos: np.ndarray, tri: np.ndarray, step: float, inset: float = 0.0):
    """Points on every triangle at most about ``step`` apart (a grid from the vertex
    opposite its longest edge, and its three edges), in chunks; each moved ``inset`` mm
    against its triangle's normal and as far toward its centroid (into the part: a face or
    an edge on a cell's boundary marks the cell it bounds, not the next one)."""
    a, b, c = (pos[tri[:, i]] for i in range(3))
    nrm = np.cross(b - a, c - a)
    nrm *= -inset / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-300)
    cen = (a + b + c) / 3.0

    # toward the centroid: a vertex by ``inset`` (or half its way), the rest in proportion
    far = np.max([np.linalg.norm(v - cen, axis=1) for v in (a, b, c)], axis=0)
    shrink = (np.minimum(inset, 0.5 * far) / np.maximum(far, 1e-300))[:, None]

    def moved(p: np.ndarray, t: np.ndarray) -> np.ndarray:
        if not inset:
            return p
        return p + nrm[t] + (cen[t] - p) * shrink[t]

    lens = np.stack([np.linalg.norm(b - c, axis=1), np.linalg.norm(c - a, axis=1),
                     np.linalg.norm(a - b, axis=1)], axis=1)
    k = np.argmax(lens, axis=1)
    verts = np.stack([a, b, c], axis=1)
    m = np.arange(len(tri))
    o, p, q = verts[m, k], verts[m, (k + 1) % 3], verts[m, (k + 2) % 3]
    e1, e2 = p - o, q - o
    ku = np.maximum(1, np.ceil(np.linalg.norm(e1, axis=1) / step)).astype(np.int64)
    kv = np.maximum(1, np.ceil(np.linalg.norm(e2, axis=1) / step)).astype(np.int64)
    n = (ku + 1) * (kv + 1)
    for s, e in _chunks(n):
        t, loc = _expand(n[s:e])
        t += s
        i, j = loc // (kv[t] + 1), loc % (kv[t] + 1)
        u, v = i / ku[t], j / kv[t]
        keep = u + v <= 1.0 + 1e-9
        t, u, v = t[keep], u[keep], v[keep]
        yield moved(o[t] + u[:, None] * e1[t] + v[:, None] * e2[t], t)
    # the edges (the grid's hypotenuse is sampled only where it lands on it)
    for p0, p1 in ((a, b), (b, c), (c, a)):
        ne = np.maximum(1, np.ceil(np.linalg.norm(p1 - p0, axis=1) / step)).astype(np.int64) + 1
        for s, e in _chunks(ne):
            t, loc = _expand(ne[s:e])
            t += s
            f = loc / (ne[t] - 1)
            yield moved(p0[t] + f[:, None] * (p1[t] - p0[t]), t)


def _column_crossings(pos, tri, owner, org, cell, nx, nz):
    """Where each column ray (up y, through the jittered cell centres) crosses each
    triangle: ``(key, y)`` with ``key = (owner * nx + i) * nz + k``."""
    jx, jz = RAY_JITTER[0] * cell, RAY_JITTER[1] * cell
    X, Y, Z = pos[tri, 0], pos[tri, 1], pos[tri, 2]          # (m, 3) each
    det = (X[:, 1] - X[:, 0]) * (Z[:, 2] - Z[:, 0]) - (X[:, 2] - X[:, 0]) * (Z[:, 1] - Z[:, 0])
    ok = np.abs(det) > 1e-12
    i0 = np.maximum(np.ceil((X.min(1) - org[0] - jx) / cell - 0.5), 0).astype(np.int64)
    i1 = np.minimum(np.floor((X.max(1) - org[0] - jx) / cell - 0.5), nx - 1).astype(np.int64)
    k0 = np.maximum(np.ceil((Z.min(1) - org[2] - jz) / cell - 0.5), 0).astype(np.int64)
    k1 = np.minimum(np.floor((Z.max(1) - org[2] - jz) / cell - 0.5), nz - 1).astype(np.int64)
    ni, nk = np.maximum(i1 - i0 + 1, 0), np.maximum(k1 - k0 + 1, 0)
    cnt = np.where(ok, ni * nk, 0)
    keys, ys = [], []
    for s, e in _chunks(cnt):
        t, loc = _expand(cnt[s:e])
        t += s
        ci = i0[t] + loc // nk[t]
        ck = k0[t] + loc % nk[t]
        px = org[0] + (ci + 0.5) * cell + jx
        pz = org[2] + (ck + 0.5) * cell + jz
        x0, z0 = X[t, 0], Z[t, 0]
        ux, uz = X[t, 1] - x0, Z[t, 1] - z0
        vx, vz = X[t, 2] - x0, Z[t, 2] - z0
        d = det[t]
        wx, wz = px - x0, pz - z0
        bu = (wx * vz - vx * wz) / d
        bv = (ux * wz - wx * uz) / d
        inside = (bu >= 0) & (bv >= 0) & (bu + bv <= 1)
        if not inside.any():
            continue
        t, bu, bv = t[inside], bu[inside], bv[inside]
        y = Y[t, 0] + bu * (Y[t, 1] - Y[t, 0]) + bv * (Y[t, 2] - Y[t, 0])
        keys.append((owner[t] * nx + ci[inside]) * nz + ck[inside])
        ys.append(y)
    if not keys:
        return np.zeros(0, np.int64), np.zeros(0)
    return np.concatenate(keys), np.concatenate(ys)


@dataclass
class VoxStats:
    """What :func:`voxelize` met: columns whose crossings didn't pair (an open mesh)."""

    odd_columns: int = 0


def voxelize(meshes: Sequence[Mesh], cell: float = CELL, rot: np.ndarray | None = None,
             org: np.ndarray | None = None, shape: tuple[int, int, int] | None = None,
             surface: bool = True, stats: VoxStats | None = None) -> Vox:
    """The meshes (each a closed part), turned by ``rot``, filled solid on a grid of
    ``cell``: its lower corner ``org`` and ``shape`` (default: the meshes' bounds). With
    ``surface`` every cell the surface touches too (a conservative fill: what the packer
    uses); without, only the cells whose centres are inside (a volume estimate: the
    overlap check's)."""
    ps = [np.asarray(p, float) @ (rot.T if rot is not None else np.eye(3)) for p, _ in meshes]
    ts = [np.asarray(t, np.int64).reshape(-1, 3) for _, t in meshes]
    have = [i for i, t in enumerate(ts) if len(t)]
    if org is None or shape is None:
        if not have:
            return Vox(np.zeros((1, 1, 1), bool), np.zeros(3), cell)
        allp = np.concatenate([ps[i] for i in have])
        lo, hi = allp.min(axis=0), allp.max(axis=0)
        org = lo if org is None else np.asarray(org, float)
        if shape is None:
            n = np.maximum(1, np.ceil((hi - org) / cell - 1e-9)).astype(int)
            shape = (int(n[0]), int(n[1]), int(n[2]))
    org = np.asarray(org, float)
    nx, ny, nz = shape
    occ = np.zeros(shape, bool)
    if not have:
        return Vox(occ, org, cell)
    # inside: per part, each column's crossings paired
    base = np.cumsum([0] + [len(ps[i]) for i in have])
    pos = np.concatenate([ps[i] for i in have])
    tri = np.concatenate([ts[i] + base[n] for n, i in enumerate(have)])
    owner = np.concatenate([np.full(len(ts[i]), n, np.int64) for n, i in enumerate(have)])
    key, y = _column_crossings(pos, tri, owner, org, cell, nx, nz)
    if len(key):
        o = np.lexsort((y, key))
        key, y = key[o], y[o]
        _, cnt = np.unique(key, return_counts=True)
        odd = cnt % 2 == 1
        if stats is not None:
            stats.odd_columns += int(odd.sum())
        good = np.repeat(~odd, cnt)
        key, y = key[good], y[good]
        ya, yb, kk = y[0::2], y[1::2], key[0::2]
        j0 = np.maximum(np.ceil((ya - org[1]) / cell - 0.5), 0).astype(np.int64)
        j1 = np.minimum(np.floor((yb - org[1]) / cell - 0.5), ny - 1).astype(np.int64)
        fine = j1 >= j0
        col = kk[fine] % (nx * nz)
        diff = np.zeros((nx * nz, ny + 1), np.int32)
        np.add.at(diff, (col, j0[fine]), 1)
        np.add.at(diff, (col, j1[fine] + 1), -1)
        filled = np.cumsum(diff[:, :ny], axis=1) > 0
        occ |= filled.reshape(nx, nz, ny).transpose(0, 2, 1)
    if surface:
        step = cell / 2.0
        n = np.array(shape)
        for pts in surface_points(pos, tri, step, inset=1e-4 * cell):
            idx = np.floor((pts - org) / cell).astype(np.int64)
            inb = ((idx >= 0) & (idx < n)).all(axis=1)
            idx = idx[inb]
            occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    return Vox(occ, org, cell)


# -- a piece in its orientations -------------------------------------------------------------


@dataclass
class Item:
    """A piece to pack: ``key``, ``label``, ``role`` (``"base"``: the floor, as it is;
    ``"panel"``: lies flat, its own frame as given; ``"fill"``: turned first by ``pre``),
    its parts' meshes (the assembled pose), their mass (kg), centre of mass and volume (mm,
    mm^3: the parts' own), ``pre`` (its own frame's turn from the assembled pose) and its
    named ``faces`` (a name -> the axis, in the turned frame, that faces that way: a side
    panel's ``outer``, ``bay``)."""

    key: str
    label: str
    role: str
    meshes: list[Mesh]
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    volume: float = 0.0
    pre: np.ndarray = field(default_factory=lambda: np.eye(3))
    faces: dict[str, np.ndarray] = field(default_factory=dict)


@dataclass
class Shape:
    """A piece in one orientation (``ri``: :data:`ROTATIONS`' index; ``rot`` the whole turn
    from the assembled pose): its voxels, each column's bottom (``lo``) and top (``hi``,
    one past) in cells (``BIG`` / ``-BIG`` where it has none), the top of it grown by one
    cell all round (``ghi``, one column more each side), its centre of mass in cells from
    its grid's corner, and ``rects``: its bottom as rectangles of one height (i0, i1, k0,
    k1, lo) for :func:`rest_heights`."""

    key: str
    ri: int
    rot: np.ndarray
    vox: Vox
    lo: np.ndarray
    hi: np.ndarray
    ghi: np.ndarray
    com: np.ndarray
    rects: np.ndarray
    _cols: tuple[np.ndarray, np.ndarray, np.ndarray] | None = field(
        default=None, repr=False, compare=False)

    @property
    def span(self) -> tuple[int, int, int]:
        a, b, c = self.vox.occ.shape
        return a, b, c

    @property
    def height(self) -> int:
        return int(self.hi[self.hi > -BIG].max()) if (self.hi > -BIG).any() else 0

    @classmethod
    def of(cls, key: str, base: Vox, pre: np.ndarray, com_pre: np.ndarray, ri: int) -> Shape:
        r = ROTATIONS[ri]
        vox = base.turned(r)
        occ = vox.occ
        has = occ.any(axis=1)
        ny = occ.shape[1]
        lo = np.where(has, occ.argmax(axis=1), BIG).astype(np.int32)
        hi = np.where(has, ny - occ[:, ::-1, :].argmax(axis=1), -BIG).astype(np.int32)
        from scipy.ndimage import maximum_filter

        pad = np.pad(hi, 1, constant_values=-BIG)
        g = maximum_filter(pad, size=3, mode="constant", cval=-BIG)
        ghi = np.where(g > -BIG, g + 1, -BIG).astype(np.int32)
        com = ((r @ com_pre) - vox.org) / vox.cell
        return cls(key, ri, r @ pre, vox, lo, hi, ghi, com, _rects(lo))

    @property
    def cols(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Its columns (x, z indices) and each one's bottom."""
        if self._cols is None:
            ii, kk = np.nonzero(self.lo < BIG)
            self._cols = (ii, kk, self.lo[ii, kk])
        return self._cols

    def signature(self) -> bytes:
        """Equal for two orientations with the same voxels (a symmetric piece's)."""
        return self.vox.occ.tobytes() + bytes(str(self.vox.occ.shape), "ascii")


def _rects(lo: np.ndarray) -> np.ndarray:
    """``lo``'s columns (those < ``BIG``) as rectangles of one value: each row's runs of
    one value, merged down the rows while the run repeats. ``(n, 5)``: i0, i1, k0, k1, lo."""
    a, c = lo.shape
    open_: dict[tuple[int, int, int], int] = {}        # (k0, k1, value) -> its first row
    out = []
    for i in range(a + 1):
        runs = set()
        if i < a:
            row = lo[i]
            edges = np.flatnonzero(np.diff(row)) + 1
            starts = np.concatenate([[0], edges])
            ends = np.concatenate([edges, [c]])
            for k0, k1 in zip(starts.tolist(), ends.tolist(), strict=True):
                v = int(row[k0])
                if v < BIG:
                    runs.add((k0, k1, v))
        out += [(open_.pop(r), i, r[0], r[1], r[2]) for r in list(open_) if r not in runs]
        for r in runs:
            open_.setdefault(r, i)
    return np.array(out, np.int64).reshape(-1, 5)


class _MaxPyramid:
    """Sliding-window maxima of a 2-D array, any window, in O(size) each once the
    power-of-two levels along each axis are made (cached per window)."""

    def __init__(self, top: np.ndarray):
        self.top = top
        self._x: list[np.ndarray] = [top]
        self._zx: dict[int, list[np.ndarray]] = {}
        self._win: dict[tuple[int, int], np.ndarray] = {}

    @staticmethod
    def _levels(levels: list[np.ndarray], n: int, axis: int) -> None:
        while (1 << (len(levels) - 1)) * 2 <= n:
            prev = levels[-1]
            s = 1 << (len(levels) - 1)
            if axis == 0:
                levels.append(np.maximum(prev[:-s], prev[s:]))
            else:
                levels.append(np.maximum(prev[:, :-s], prev[:, s:]))

    @staticmethod
    def _window(levels: list[np.ndarray], n: int, axis: int) -> np.ndarray:
        k = n.bit_length() - 1
        p = levels[k]
        m = p.shape[axis] - (n - (1 << k))
        if axis == 0:
            return np.maximum(p[:m], p[n - (1 << k):n - (1 << k) + m])
        return np.maximum(p[:, :m], p[:, n - (1 << k):n - (1 << k) + m])

    def window(self, h: int, w: int) -> np.ndarray:
        """``out[u, v] = top[u:u+h, v:v+w].max()``."""
        got = self._win.get((h, w))
        if got is not None:
            return got
        zx = self._zx.get(h)
        if zx is None:
            self._levels(self._x, h, 0)
            zx = self._zx[h] = [self._window(self._x, h, 0)]
        self._levels(zx, w, 1)
        got = self._win[(h, w)] = self._window(zx, w, 1)
        return got


def rest_heights(top: np.ndarray | _MaxPyramid, shape: Shape) -> np.ndarray | None:
    """Where ``shape`` comes to rest dropped on heightmap ``top`` at each column offset:
    ``W[u, v] = max over its columns (i, k) of top[u+i, v+k] - lo[i, k]`` (cells, at least
    0), ``(nu, nv)``; None when it doesn't fit the footprint."""
    pyr = top if isinstance(top, _MaxPyramid) else _MaxPyramid(top)
    a, _, c = shape.span
    nx, nz = pyr.top.shape
    nu, nv = nx - a + 1, nz - c + 1
    if nu <= 0 or nv <= 0:
        return None
    w = np.zeros((nu, nv), np.int32)
    for i0, i1, k0, k1, v in shape.rects.tolist():
        m = pyr.window(i1 - i0, k1 - k0)
        np.maximum(w, m[i0:i0 + nu, k0:k0 + nv] - v, out=w)
    return w


class Body:
    """An :class:`Item` voxelized once in its own frame (``item.pre``), and its
    orientations (:class:`Shape`), made as asked and kept."""

    def __init__(self, item: Item, cell: float = CELL, stats: VoxStats | None = None):
        self.item = item
        self.cell = cell
        self.vox = voxelize(item.meshes, cell, rot=item.pre, stats=stats)
        self.com_pre = item.pre @ np.asarray(item.com, float)
        self._shapes: dict[int, Shape] = {}

    @property
    def key(self) -> str:
        return self.item.key

    def shape(self, ri: int) -> Shape:
        got = self._shapes.get(ri)
        if got is None:
            got = self._shapes[ri] = Shape.of(self.item.key, self.vox, self.item.pre,
                                              self.com_pre, ri)
        return got

    def distinct(self, ris: Sequence[int]) -> list[int]:
        """``ris`` less those whose voxels repeat an earlier one's."""
        seen: set[bytes] = set()
        out = []
        for ri in ris:
            sig = self.shape(ri).signature()
            if sig not in seen:
                seen.add(sig)
                out.append(ri)
        return out

    def axis(self, name: str) -> np.ndarray:
        """A face's axis in the turned frame: a named one (``faces``) or ``+x`` .. ``-z``."""
        if name in self.item.faces:
            return np.asarray(self.item.faces[name], float)
        m = {"x": 0, "y": 1, "z": 2}
        if len(name) == 2 and name[0] in "+-" and name[1] in m:
            v = np.zeros(3)
            v[m[name[1]]] = 1.0 if name[0] == "+" else -1.0
            return v
        raise ValueError(f"{self.key}: no face {name!r} (have {sorted(self.item.faces)}, "
                         "+x .. -z)")

    def orientations(self, down: str | None = None, spin: int | None = None) -> list[int]:
        """The rotations (indices into :data:`ROTATIONS`) a plan's ``down`` and ``spin``
        allow, distinct, in order: ``down`` a face (its axis turned to -y), ``"flat"``
        (the least tall ways, within a cell), or None (any); ``spin`` a quarter turn about
        the vertical from the face's first way down, or None (any)."""
        if down in (None, "any"):
            return self.distinct(range(len(ROTATIONS)))
        if down == "flat":
            ris = self.distinct(range(len(ROTATIONS)))
            hs = {ri: self.shape(ri).height for ri in ris}
            least = min(hs.values())
            return [ri for ri in ris if hs[ri] <= least + 1]
        base = down_rotation(self.axis(down))
        spins = [spin] if spin is not None else [0, 90, 180, 270]
        out = []
        for s in spins:
            r = np.rint(rot_about("y", float(s))) @ base
            out.append(next(i for i, q in enumerate(ROTATIONS) if np.allclose(q, r)))
        return self.distinct(out)


@dataclass
class Placed:
    """A piece in the box: its orientation (:class:`Shape`), its grid's corner in the box's
    cells ``at`` (x, y, z), its turn and shift (packed = rot @ assembled + shift, mm), how
    it rests (:class:`Support`)."""

    key: str
    shape: Shape
    at: tuple[int, int, int]
    rot: np.ndarray
    shift: np.ndarray
    rest: Support = field(default_factory=lambda: Support("direct", ("floor",)))

    @property
    def supported(self) -> bool:
        return self.rest.ok

    def matrix(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3] = self.rot
        m[:3, 3] = self.shift
        return m


CORNERS = {"-x-z": (-1, -1), "+x-z": (1, -1), "-x+z": (-1, 1), "+x+z": (1, 1)}


class Bin:
    """The box being packed: ``nx`` x ``nz`` columns of :data:`CELL` from ``org`` (mm, the
    floor's corner), each one's grown top (``top``), real top (``real``) and whose that is
    (``who``, an index into ``placed``), all in cells."""

    def __init__(self, nx: int, nz: int, org, cell: float = CELL):
        self.nx, self.nz, self.cell = nx, nz, cell
        self.org = np.asarray(org, float)
        self.top = np.zeros((nx, nz), np.int32)
        self.real = np.zeros((nx, nz), np.int32)
        self.who = np.full((nx, nz), -1, np.int32)
        self.placed: list[Placed] = []
        self.height = 0
        self.xr = (nx, 0)           # the real cells' column range so far
        self.zr = (nz, 0)
        self._pyr: _MaxPyramid | None = None

    def copy(self) -> Bin:
        b = copy.copy(self)
        b.top, b.real, b.who = self.top.copy(), self.real.copy(), self.who.copy()
        b.placed = list(self.placed)
        b._pyr = None               # (this one's own top from now on)
        return b

    def index(self, key: str) -> int:
        return next(i for i, p in enumerate(self.placed) if p.key == key)

    def pyramid(self) -> _MaxPyramid:
        if self._pyr is None:
            self._pyr = _MaxPyramid(self.top)
        return self._pyr

    def put(self, sh: Shape, u: int, w: int, v: int, rest: Support | None = None) -> Placed:
        """Place ``sh`` with its grid's corner at cell ``(u, w, v)``, resting as ``rest``
        says (default: as :meth:`support` finds)."""
        a, _, c = sh.span
        shift = self.org + np.array([u, w, v]) * self.cell - sh.vox.org
        rest = rest or self.support(sh, u, w, v)
        p = Placed(sh.key, sh, (u, w, v), sh.rot, shift, rest)
        n = len(self.placed)
        self.placed.append(p)
        has = sh.hi > -BIG
        reg = (slice(u, u + a), slice(v, v + c))
        rtop = np.where(has, w + sh.hi, 0)
        mine = rtop > self.real[reg]
        self.real[reg] = np.where(mine, rtop, self.real[reg])
        self.who[reg] = np.where(mine, n, self.who[reg])
        g0u, g0v = u - 1, v - 1
        gu0, gv0 = max(g0u, 0), max(g0v, 0)
        gu1, gv1 = min(g0u + a + 2, self.nx), min(g0v + c + 2, self.nz)
        gt = sh.ghi[gu0 - g0u:gu1 - g0u, gv0 - g0v:gv1 - g0v]
        sub = self.top[gu0:gu1, gv0:gv1]
        np.maximum(sub, np.where(gt > -BIG, w + gt, 0), out=sub)
        self.height = max(self.height, w + sh.height)
        xs, zs = np.flatnonzero(has.any(axis=1)), np.flatnonzero(has.any(axis=0))
        if len(xs):
            self.xr = (min(self.xr[0], u + int(xs[0])), max(self.xr[1], u + int(xs[-1]) + 1))
            self.zr = (min(self.zr[0], v + int(zs[0])), max(self.zr[1], v + int(zs[-1]) + 1))
        self._pyr = None
        return p

    def support(self, sh: Shape, u: int, w: int, v: int) -> Support:
        """How ``sh`` at ``(u, w, v)`` rests on what is under it (:func:`support`): each of
        its columns' gap over the real cell under it (a cell of foam between) or the
        floor."""
        ii, kk, lo = sh.cols
        bottom = w + lo
        real = self.real[u + ii, v + kk]
        who = self.who[u + ii, v + kk]
        gap = np.where(real > 0, bottom - (real + 1), bottom)
        names = [p.key for p in self.placed]
        owner = np.where(real > 0, who, -1)
        xz = np.stack([u + ii, v + kk], axis=1)
        return support(xz, gap, owner, names, sh.com[[0, 2]] + [u, v], self.cell,
                       self.org[[0, 2]])

    def own_top(self, i: int) -> np.ndarray:
        """Placed piece ``i``'s own grown top (a heightmap of it alone)."""
        p = self.placed[i]
        sh = p.shape
        u, w, v = p.at
        a, _, c = sh.span
        out = np.zeros((self.nx, self.nz), np.int32)
        g0u, g0v = u - 1, v - 1
        gu0, gv0 = max(g0u, 0), max(g0v, 0)
        gu1, gv1 = min(g0u + a + 2, self.nx), min(g0v + c + 2, self.nz)
        gt = sh.ghi[gu0 - g0u:gu1 - g0u, gv0 - g0v:gv1 - g0v]
        out[gu0:gu1, gv0:gv1] = np.where(gt > -BIG, w + gt, 0)
        return out

    def candidates(self, sh: Shape, region: np.ndarray | None = None,
                   corners: Sequence[str | None] = (None,), keep: int = 400,
                   on: int | None = None) -> dict[str | None, tuple[np.ndarray, ...]]:
        """Every offset ``sh`` may drop at, best first for each of ``corners`` (the module's
        docstring): ``(u, v, w, box volume after)`` arrays, by the box's volume after, then
        the centre of
        mass's height, then the gap to the corner (None: to the nearest); none when it fits
        nowhere. ``region``: a mask over the box's columns its footprint's centre must be
        in; ``on``: a placed piece's index it must come down on."""
        w = rest_heights(self.pyramid(), sh)
        if w is None:
            return {}
        bind = None
        if on is not None:          # the offsets where piece ``on`` is what it comes down on
            wo = rest_heights(self.own_top(on), sh)
            assert wo is not None
            bind = ((wo >= w - round(SETTLE / self.cell)) & (wo > 0)).ravel()
        nu, nv = w.shape
        u, v = np.meshgrid(np.arange(nu), np.arange(nv), indexing="ij")
        u, v, w = u.ravel(), v.ravel(), w.ravel()
        a, _, c = sh.span
        if region is not None or bind is not None:
            ok = np.ones(len(u), bool) if bind is None else bind
            if region is not None:
                cu, cv = u + a // 2, v + c // 2
                ok &= region[np.clip(cu, 0, self.nx - 1), np.clip(cv, 0, self.nz - 1)]
            u, v, w = u[ok], v[ok], w[ok]
            if not len(u):
                return {}
        h = np.maximum(self.height, w + sh.height)
        x0, x1 = np.minimum(self.xr[0], u), np.maximum(self.xr[1], u + a)
        z0, z1 = np.minimum(self.zr[0], v), np.maximum(self.zr[1], v + c)
        vol = (x1 - x0).astype(np.int64) * (z1 - z0) * h
        com = np.round(w + sh.com[1], 3)
        if len(u) > 4 * keep:          # (only the first few are ever looked at)
            sel = vol <= np.partition(vol, keep)[keep]
            if sel.sum() > 4 * keep:
                cs = com[sel]
                sel[sel] = cs <= np.partition(cs, keep)[keep]
            u, v, w, vol, com = u[sel], v[sel], w[sel], vol[sel], com[sel]
        out = {}
        for corner in corners:
            if corner in CORNERS:
                sx, sz = CORNERS[corner]
                gap = (u if sx < 0 else nu - 1 - u) + (v if sz < 0 else nv - 1 - v)
            else:
                gap = np.minimum(u, nu - 1 - u) + np.minimum(v, nv - 1 - v)
            order = np.lexsort((gap, com, vol))
            out[corner] = (u[order], v[order], w[order], vol[order])
        return out


def over_hull(cols: np.ndarray, point) -> bool:
    """Whether ``point`` (x z, cells) is over the convex hull of the cells ``cols`` ((n, 2):
    each one's x z index)."""
    if not len(cols):
        return False
    px, pz = float(point[0]), float(point[1])
    lo, hi = cols.min(axis=0), cols.max(axis=0) + 1
    if not (lo[0] <= px <= hi[0] and lo[1] <= pz <= hi[1]):
        return False
    from scipy.spatial import ConvexHull, QhullError

    # each row's ends are all the hull can use
    x = cols[:, 0] - lo[0]
    n = int(hi[0] - lo[0])
    zmin = np.full(n, BIG)
    zmax = np.full(n, -BIG)
    np.minimum.at(zmin, x, cols[:, 1])
    np.maximum.at(zmax, x, cols[:, 1])
    rows = np.flatnonzero(zmin < BIG)
    ends = np.concatenate([np.stack([rows + lo[0], zmin[rows]], axis=1),
                           np.stack([rows + lo[0], zmax[rows]], axis=1)])
    pts = np.concatenate([ends + d for d in ((0, 0), (0, 1), (1, 0), (1, 1))]).astype(float)
    try:
        hull = ConvexHull(pts)
    except QhullError:
        return True             # (all on a line: within its box, above)
    return bool((hull.equations[:, :2] @ [px, pz] + hull.equations[:, 2] <= 1e-9).all())


PAD_MAX = 60.0
"""mm: the deepest gap under a piece a foam pad fills (a block cut to fit)."""


@dataclass(frozen=True)
class Support:
    """How a piece rests: ``how`` (``"direct"``: its centre of mass over the convex hull of
    its contacts; ``"padded"``: over it once foam pads fill gaps of up to :data:`PAD_MAX`
    under it; ``"none"``: neither, it would tip), on what (piece keys, ``"floor"``) and the
    ``pads`` (x, z, height: mm, the box's frame)."""

    how: str
    on: tuple[str, ...] = ()
    pads: tuple[tuple[float, float, float], ...] = ()

    @property
    def ok(self) -> bool:
        return self.how != "none"


def support(xz: np.ndarray, gap: np.ndarray, owner: np.ndarray, names: Sequence[str],
            com, cell: float, org=(0.0, 0.0)) -> Support:
    """How a piece rests (:class:`Support`): its columns ``xz`` (cells), each one's ``gap``
    (cells) down to what is under it, whose that is (``owner``: an index into ``names``,
    -1 the floor), its centre of mass (x z, cells). A column within :data:`SETTLE` touches
    (dropped straight a piece meets one point, then tips onto the next ones as it settles);
    one within :data:`PAD_MAX` takes a pad. The pads are the padded columns at the corners of
    the support's hull, those within 8 cells of each other one pad, as tall as the deepest."""
    settle = round(SETTLE / cell)
    padmax = PAD_MAX / cell

    def on(sel: np.ndarray) -> tuple[str, ...]:
        return tuple(sorted({"floor" if o < 0 else names[int(o)]
                             for o in np.unique(owner[sel])}))

    direct = gap <= settle
    if over_hull(xz[direct], com):
        return Support("direct", on(direct))
    padable = (gap > settle) & (gap <= padmax)
    both = direct | padable
    if not over_hull(xz[both], com):
        return Support("none", on(direct))
    from scipy.spatial import ConvexHull, QhullError

    idx = np.flatnonzero(both)
    try:
        corners = idx[ConvexHull(xz[idx].astype(float)).vertices]
    except QhullError:
        corners = idx
    corners = corners[padable[corners]]
    pads: list[list] = []
    for i in corners:
        x, z = xz[i]
        for p in pads:
            if abs(p[0] - x) <= 8 and abs(p[1] - z) <= 8:
                p[2] = max(p[2], gap[i])
                break
        else:
            pads.append([x, z, gap[i]])
    used = direct.copy()
    used[corners] = True
    return Support("padded", on(used),
                   tuple((round(float(org[0] + (x + 0.5) * cell), 1),
                          round(float(org[1] + (z + 0.5) * cell), 1),
                          round(float(g * cell), 1)) for x, z, g in pads))


# -- plans -----------------------------------------------------------------------------------


@dataclass(frozen=True)
class Step:
    """A plan's word on one piece (``key``). Its orientation: ``down``, the face that goes
    down (a named face of the piece, ``"outer"``, ``"bay"``; an axis of its own frame,
    ``"+x"`` .. ``"-z"``; ``"flat"``: its least tall ways; None: any), ``spin`` a quarter
    turn about the vertical from there (None: any), or ``rot``, an explicit turn (one of
    :data:`ROTATIONS`, its rows). Where: ``corner`` (``"-x-z"`` ..: the
    slot nearest it; None: the nearest any), ``on`` (it rests on that piece), ``into`` (its
    bottom below that piece's top, inside its footprint: nested in it), ``region`` (x0, z0,
    x1, z1: fractions of the box's footprint its own footprint's centre is in). ``after``:
    the pieces that go in before it (``on`` and ``into`` add theirs). ``why``: the human
    packer's reason."""

    key: str
    down: str | None = None
    spin: int | None = None
    corner: str | None = None
    on: str | None = None
    into: str | None = None
    region: tuple[float, float, float, float] | None = None
    after: tuple[str, ...] = ()
    why: str = ""
    rot: tuple[tuple[float, float, float], ...] | None = None

    def before(self) -> set[str]:
        return set(self.after) | {k for k in (self.on, self.into) if k}


@dataclass(frozen=True)
class Plan:
    """A packing's intent: its :class:`Step` per piece, in the order they go in (a piece it
    doesn't name goes in after them, automatically: :func:`pack`), and the box's footprint
    (x, z mm; None: as large as the largest piece lying flat and the base)."""

    name: str
    steps: tuple[Step, ...] = ()
    footprint: tuple[float, float] | None = None
    note: str = ""

    def step(self, key: str) -> Step | None:
        return next((s for s in self.steps if s.key == key), None)

    def to_json(self) -> dict:
        return {"name": self.name, "note": self.note,
                "footprint": list(self.footprint) if self.footprint else None,
                "steps": [{k: (list(v) if isinstance(v, tuple) else v)
                           for k, v in s.__dict__.items() if v not in (None, (), "")}
                          for s in self.steps]}

    @classmethod
    def from_json(cls, d: dict) -> Plan:
        steps = []
        for s in d.get("steps", []):
            s = dict(s)
            for k in ("region", "after"):
                if s.get(k) is not None:
                    s[k] = tuple(s[k])
            if s.get("rot") is not None:
                s["rot"] = tuple(tuple(float(v) for v in row) for row in s["rot"])
            steps.append(Step(**s))
        fp = d.get("footprint")
        return cls(d.get("name", "plan"), tuple(steps), tuple(fp) if fp else None,
                   d.get("note", ""))


@dataclass(frozen=True)
class Choice:
    """A piece's fixed choice in a sequence (the search's): its orientation (``ri``) and
    the corner its slot is nearest (None: any)."""

    key: str
    ri: int
    corner: str | None = None


@dataclass
class Layout:
    """:func:`pack`'s answer: ``placed`` in the order they go in, the :class:`Bin` they are
    in, the choices that made it (``seq``) and its :func:`score`."""

    placed: list[Placed]
    bin: Bin
    seq: list[Choice]
    score: float = math.inf
    warnings: list[str] = field(default_factory=list)

    def by_key(self) -> dict[str, Placed]:
        return {p.key: p for p in self.placed}


W_UNSUPPORTED = 25.0    # litres a piece whose centre of mass overhangs what it rests on costs
W_PADDED = 2.0          # litres a piece resting on foam pads costs (they're cut to fit)
W_HIGH = 10.0           # litres for the mass centre at the box's top (0 at its floor)
W_TALL = 10.0           # litres a metre of box height costs (a tall box tips, stacks badly)
WALL = 5.0              # the box's corrugated wall, mm


def score(bin_: Bin, masses: dict[str, float] | None = None) -> float:
    """A layout's cost: its box's outside volume (litres, from the grid's cells, the walls
    on), :data:`W_UNSUPPORTED` per piece that tips, :data:`W_PADDED` per piece on pads,
    :data:`W_HIGH` times the mass centre's height over the box's, :data:`W_TALL` per metre
    of height. ``masses``: each piece's (kg)."""
    c = bin_.cell
    dx = max(bin_.xr[1] - bin_.xr[0], 0) * c + 2 * (CLEAR + WALL)
    dz = max(bin_.zr[1] - bin_.zr[0], 0) * c + 2 * (CLEAR + WALL)
    dy = bin_.height * c + 2 * (CLEAR + WALL)
    s = dx * dy * dz / 1e6
    s += W_UNSUPPORTED * sum(not p.supported for p in bin_.placed)
    s += W_PADDED * sum(p.rest.how == "padded" for p in bin_.placed)
    if masses and bin_.height:
        m = sum(masses[p.key] for p in bin_.placed if p.key in masses)
        if m > 0:
            y = sum(masses[p.key] * (p.at[1] + p.shape.com[1]) for p in bin_.placed
                    if p.key in masses) / m
            s += W_HIGH * y / bin_.height
    s += W_TALL * dy / 1000.0
    return float(s)


# -- packing ---------------------------------------------------------------------------------


class Packer:
    """A set of pieces made ready to pack (each voxelized once) and the box's footprint:
    :meth:`run` packs a sequence of choices, :meth:`greedy` picks them as it goes."""

    def __init__(self, items: Sequence[Item], plan: Plan | None = None, cell: float = CELL,
                 footprint: tuple[float, float] | None = None):
        self.cell = cell
        self.plan = plan or Plan("auto")
        self.stats = VoxStats()
        self.bodies = {it.key: Body(it, cell, self.stats) for it in items}
        self.items = {it.key: it for it in items}
        bases = [k for k, it in self.items.items() if it.role == "base"]
        self.base = bases[0] if bases else None
        fp = footprint or self.plan.footprint or self._footprint()
        nx, nz = (max(1, math.ceil(f / cell - 1e-6)) for f in fp)
        if self.base is not None:
            bv = self.bodies[self.base].vox
            a, _, c = bv.occ.shape
            nx, nz = max(nx, a), max(nz, c)
            self.base_at = ((nx - a) // 2, (nz - c) // 2)
            org = bv.org - np.array([self.base_at[0], 0, self.base_at[1]]) * cell
        else:
            self.base_at = (0, 0)
            org = np.zeros(3)
        self.nx, self.nz, self.org = nx, nz, org
        self.masses = {k: it.mass for k, it in self.items.items()}

    def _footprint(self) -> tuple[float, float]:
        """As large as the base and each piece lying flat (its least tall way), the long
        sides together."""
        sizes = []
        for k, b in self.bodies.items():
            if k == self.base:
                sh = b.shape(0)
            else:
                ris = b.orientations("flat")
                sh = min((b.shape(ri) for ri in ris),
                         key=lambda s: (s.span[0] * s.span[2], s.ri))
            a, _, c = sh.span
            sizes.append(sorted((a, c), reverse=True))
        base = None
        if self.base is not None:
            a, _, c = self.bodies[self.base].shape(0).span
            base = (a, c)
        lx = max(s[0] for s in sizes)
        lz = max(s[1] for s in sizes)
        if base is not None:
            if base[0] >= base[1]:
                lx, lz = max(lx, base[0]), max(lz, base[1])
            else:
                lx, lz = max(lz, base[0]), max(lx, base[1])
        return lx * self.cell, lz * self.cell

    def fresh(self) -> Bin:
        b = Bin(self.nx, self.nz, self.org, self.cell)
        if self.base is not None:
            b.put(self.bodies[self.base].shape(0), self.base_at[0], 0, self.base_at[1],
                  Support("direct", ("floor",)))
        return b

    def order(self) -> list[str]:
        """The plan's pieces in its order (each after what it names), then the rest: the
        panels, then the others, the largest first."""
        named = [s.key for s in self.plan.steps if s.key in self.bodies and s.key != self.base]
        rest = [k for k in self.bodies if k not in named and k != self.base]

        def size(k: str) -> tuple:
            occ = self.bodies[k].vox.occ
            return (self.items[k].role != "panel", -int(np.prod(occ.shape)), k)

        return _topo(named + sorted(rest, key=size), self._before())

    def _before(self) -> dict[str, set[str]]:
        out: dict[str, set[str]] = {}
        for s in self.plan.steps:
            out[s.key] = {k for k in s.before() if k in self.bodies and k != self.base}
        return out

    def allowed(self, key: str) -> list[int]:
        """The orientations piece ``key`` may take: its step's, else a panel either named
        face down (or flat), anything else any way."""
        b = self.bodies[key]
        st = self.plan.step(key)
        if st is not None and st.rot is not None:
            r = np.asarray(st.rot, float)
            ri = next((i for i, q in enumerate(ROTATIONS) if np.allclose(q, r)), None)
            if ri is None:
                raise ValueError(f"{key}: rot {st.rot} is not one of the cube's 24 turns")
            return [ri]
        if st is not None and (st.down is not None or st.spin is not None):
            return b.orientations(st.down, st.spin)
        if self.items[key].role == "panel":
            faces = [f for f in ("outer", "bay") if f in self.items[key].faces]
            if faces:
                return b.distinct([ri for f in faces for ri in b.orientations(f)])
            return b.orientations("flat")
        return b.orientations(None)

    def corners(self, key: str) -> list[str | None]:
        """The corners piece ``key``'s slot may be nearest: its step's, else the nearest
        any (the search tries each)."""
        st = self.plan.step(key)
        if st is not None and st.corner is not None:
            return [st.corner]
        return [None]

    def _region(self, st: Step | None) -> np.ndarray | None:
        if st is None or st.region is None:
            return None
        x0, z0, x1, z1 = st.region
        xs = (np.arange(self.nx) + 0.5) / self.nx
        zs = (np.arange(self.nz) + 0.5) / self.nz
        return ((xs[:, None] >= x0) & (xs[:, None] <= x1)
                & (zs[None, :] >= z0) & (zs[None, :] <= z1))

    def _fits(self, b: Bin, st: Step | None, sh: Shape, u: int, w: int, v: int,
              on: set[str]) -> bool:
        """The step's ``on`` and ``into``."""
        if st is None:
            return True
        if st.on and st.on in self.bodies and st.on not in on:
            return False
        if st.into and st.into in self.bodies:
            i = b.index(st.into)
            a, _, c = sh.span
            who = b.who[u:u + a, v:v + c]
            mine = sh.lo < BIG
            over = mine & (who == i)
            if not over.any():
                return False
            other = b.placed[i]
            if w + int(sh.lo[mine].min()) >= other.at[1] + other.shape.height:
                return False
        return True

    def drop(self, b: Bin, key: str, ris: Sequence[int], corners: Sequence[str | None],
             limit: int = 80) -> tuple[tuple, Placed | None]:
        """The best place for ``key`` among its orientations ``ris`` and ``corners`` (by
        :meth:`Bin.candidates`' order, of its first ``limit`` the first supported that
        meets its step's ``on`` and ``into``, else the first that meets them), put in
        ``b``; its sort key and choice."""
        st = self.plan.step(key)
        region = self._region(st)
        best = None
        for ri in ris:
            sh = self.bodies[key].shape(ri)
            on = b.index(st.on) if st is not None and st.on in self.bodies else None
            if st is not None and st.into in self.bodies:
                limit = max(limit, 2000)
            for corner, (u, v, w, vol) in b.candidates(sh, region, corners, on=on).items():
                pick = None
                for n in range(min(limit, len(u))):
                    if pick is not None and pick[1].ok and (vol[n] > vol[pick[0]]
                                                            or n > pick[0] + 16):
                        break           # (a padded one, and the rest raise the box more)
                    uu, ww, vv = int(u[n]), int(w[n]), int(v[n])
                    rest = b.support(sh, uu, ww, vv)
                    if not self._fits(b, st, sh, uu, ww, vv, set(rest.on)):
                        continue
                    if rest.how == "direct":
                        pick = (n, rest)
                        break
                    if pick is None or (rest.ok and not pick[1].ok):
                        pick = (n, rest)
                if pick is None:
                    continue
                n, rest = pick
                uu, vv, ww = int(u[n]), int(v[n]), int(w[n])
                a, _, c = sh.span
                h = max(b.height, ww + sh.height)
                x0, x1 = min(b.xr[0], uu), max(b.xr[1], uu + a)
                z0, z1 = min(b.zr[0], vv), max(b.zr[1], vv + c)
                k = (not rest.ok, (x1 - x0) * (z1 - z0) * h, rest.how != "direct",
                     round(ww + float(sh.com[1]), 3), n, ri)
                if best is None or k < best[0]:
                    best = (k, sh, uu, ww, vv, rest, corner)
        if best is None:
            return (), None
        k, sh, u, w, v, rest, corner = best
        p = b.put(sh, u, w, v, rest)
        return (k, Choice(key, sh.ri, corner)), p

    def run(self, seq: Sequence[Choice], start: Bin | None = None,
            prefix: list[Bin] | None = None) -> Layout:
        """Pack ``seq`` (each piece in its orientation, toward its corner) after ``start``
        (default: the base alone); ``prefix``, when given, gets the box after each one."""
        b = (start or self.fresh()).copy()
        done = []
        warn = []
        for ch in seq:
            _, p = self.drop(b, ch.key, [ch.ri], [ch.corner])
            if p is None:
                warn.append(f"{ch.key}: no room")
                return Layout(b.placed, b, list(seq), math.inf, warn)
            done.append(ch)
            if prefix is not None:
                prefix.append(b.copy())
        return Layout(b.placed, b, list(seq), score(b, self.masses), warn)

    def greedy(self, order: Sequence[str] | None = None) -> Layout:
        """Each piece in turn (``order``, default :meth:`order`), its best orientation and
        corner (:meth:`drop`)."""
        b = self.fresh()
        seq = []
        warn = []
        for key in order or self.order():
            got, p = self.drop(b, key, self.allowed(key), self.corners(key))
            if p is None:
                warn.append(f"{key}: no room")
                return Layout(b.placed, b, seq, math.inf, warn)
            seq.append(got[1])
        return Layout(b.placed, b, seq, score(b, self.masses), warn)

    def best_greedy(self) -> Layout:
        """:meth:`greedy` from each order of the panels the plan leaves free (the rest after
        them as :meth:`order` has it), the least :func:`score` kept."""
        order = self.order()
        named = {s.key for s in self.plan.steps}
        free_panels = [k for k in order if self.items[k].role == "panel" and k not in named]
        best = None
        tried = set()
        for perm in permutations(free_panels):
            o = [k for k in order if k not in free_panels or k in perm]
            it = iter(perm)
            o = [next(it) if k in free_panels else k for k in o]
            o = _topo(o, self._before())
            if tuple(o) in tried:
                continue
            tried.add(tuple(o))
            lay = self.greedy(o)
            if best is None or lay.score < best.score - 1e-9:
                best = lay
        assert best is not None
        return best


def _topo(order: Sequence[str], before: dict[str, set[str]]) -> list[str]:
    """``order`` with each key after those ``before`` names (stable: otherwise as given)."""
    out: list[str] = []
    left = list(order)
    while left:
        for i, k in enumerate(left):
            if not (before.get(k, set()) & set(left[:i] + left[i + 1:])):
                out.append(left.pop(i))
                break
        else:
            raise ValueError(f"the plan's order has a cycle among {left}")
    return out


def pack(items: Sequence[Item], plan: Plan | None = None, cell: float = CELL,
         seq: Sequence[Choice] | None = None) -> tuple[Layout, Packer]:
    """Pack ``items`` (their roles as :class:`Item`'s) by ``plan`` (or none: every piece
    automatic), or by an explicit sequence of choices ``seq`` (the search's); the layout
    and the packer it was made with."""
    pk = Packer(items, plan, cell)
    if seq is not None:
        return pk.run(seq), pk
    return pk.best_greedy(), pk


# -- search ----------------------------------------------------------------------------------


def _valid(seq: Sequence[Choice], before: dict[str, set[str]]) -> bool:
    seen: set[str] = set()
    for ch in seq:
        if before.get(ch.key, set()) - seen:
            return False
        seen.add(ch.key)
    return True


def anneal(items: Sequence[Item], plan: Plan | None, seq0: Sequence[Choice], seconds: float,
           seed: int, cell: float = CELL) -> tuple[float, list[Choice], int]:
    """Simulated annealing from the choices ``seq0`` for ``seconds``: a move swaps two
    pieces' places in the order (each still after what its plan step names), turns one
    piece to another orientation its step allows, or sends it toward another corner (one
    its step leaves free); each sequence packed (:meth:`Packer.run`, again from the first
    piece it changes) and kept by Metropolis on :func:`score`, the temperature falling
    from 3 % of the start's score to 0.1 % over the time. Its best ``(score, choices,
    sequences tried)``. A package function (:func:`spiderpig.workers.submit` runs one per
    process)."""
    rng = np.random.default_rng(seed)
    pk = Packer(items, plan, cell)
    before = pk._before()
    allowed = {ch.key: pk.allowed(ch.key) for ch in seq0}
    corners: dict[str, list[str | None]] = {}
    for ch in seq0:
        st = pk.plan.step(ch.key)
        corners[ch.key] = [st.corner] if st is not None and st.corner else [None, *CORNERS]
    cur = list(seq0)
    bins: list[Bin] = [pk.fresh()]
    lay = pk.run(cur, bins[0], bins)
    cur_s = lay.score
    best = (cur_s, list(cur))
    t0 = time.perf_counter()
    t_hi, t_lo = 0.03 * cur_s, 0.001 * cur_s
    tried = 1
    n = len(cur)
    while n and time.perf_counter() - t0 < seconds:
        frac = (time.perf_counter() - t0) / max(seconds, 1e-9)
        temp = t_hi * (t_lo / t_hi) ** frac
        new = list(cur)
        kind = rng.integers(3)
        if kind == 0 and n > 1:
            i, j = sorted(rng.choice(n, 2, replace=False).tolist())
            new[i], new[j] = new[j], new[i]
            first = i
            if not _valid(new, before):
                continue
        else:
            first = int(rng.integers(n))
            ch = new[first]
            if kind == 1 and len(allowed[ch.key]) > 1:
                ri = int(rng.choice([r for r in allowed[ch.key] if r != ch.ri]))
                new[first] = Choice(ch.key, ri, ch.corner)
            elif len(corners[ch.key]) > 1:
                c = [k for k in corners[ch.key] if k != ch.corner]
                new[first] = Choice(ch.key, ch.ri, c[int(rng.integers(len(c)))])
            else:
                continue
        prefix = bins[:first + 1]
        lay = pk.run(new[first:], prefix[-1], prefix)
        tried += 1
        s = lay.score
        if s <= cur_s or rng.random() < math.exp(-(s - cur_s) / max(temp, 1e-9)):
            cur, cur_s, bins = new, s, prefix
            if s < best[0] - 1e-9:
                best = (s, list(new))
    return best[0], best[1], tried


def search(items: Sequence[Item], plan: Plan | None = None, seconds: float = 60.0,
           processes: int = 4, seed: int = 0, cell: float = CELL,
           log: Callable[[str], None] | None = None) -> Layout:
    """The best packing :func:`anneal` finds round ``plan`` (from its greedy packing,
    :meth:`Packer.best_greedy`) in ``processes`` worker processes at once (at most 6; each
    its own seed), each for ``seconds``; packed again here. ``SPIDERPIG_WORKERS=0``: one,
    in this process."""
    from spiderpig import workers

    processes = max(1, min(6, processes))
    pk = Packer(items, plan, cell)
    start = pk.best_greedy()
    if log:
        log(f"greedy: score {start.score:.1f}")
    seq0 = list(start.seq)
    jobs = []
    if workers.enabled() and processes > 1:
        jobs = [workers.submit(anneal, list(items), plan, seq0, seconds, seed + i, cell)
                for i in range(processes)]
        got = [j.result() for j in jobs]
    else:
        got = [anneal(items, plan, seq0, seconds, seed, cell)]
    got.append((start.score, seq0, 1))
    _, seq, _ = min(got, key=lambda g: g[0])
    if log:
        log("anneal: " + ", ".join(f"{g[0]:.1f} ({g[2]} tried)" for g in got[:-1]))
    return pk.run(seq)


# -- judging a packing -----------------------------------------------------------------------


@dataclass
class Packed:
    """A piece where a packing put it, whoever packed it: its parts' meshes in the box's
    frame, its mass (kg), centre of mass (mm, that frame) and volume (mm^3, its parts')."""

    key: str
    label: str
    meshes: list[Mesh]
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    volume: float = 0.0


FINE = 1.0          # mm: the overlap check's cells


@dataclass
class Grid:
    """Packed pieces voxelized in the box (:func:`occupancy`): each piece's cells, how many
    pieces claim each cell, the box's inside (``lo``, ``hi``, mm) and the grid's corner."""

    occ: list[np.ndarray]
    count: np.ndarray
    lo: np.ndarray
    hi: np.ndarray
    cell: float

    def owner(self) -> np.ndarray:
        """Whose each cell is (-1: empty; the last piece that claims it)."""
        out = np.full(self.count.shape, -1, np.int16)
        for i, o in enumerate(self.occ):
            out[o] = i
        return out


def box_of(pieces: Sequence[Packed]) -> tuple[np.ndarray, np.ndarray]:
    """The box's inside round ``pieces``: their meshes' bounds, :data:`CLEAR` more."""
    allp = np.concatenate([np.asarray(p, float) for pc in pieces for p, _ in pc.meshes])
    return allp.min(axis=0) - CLEAR, allp.max(axis=0) + CLEAR


def occupancy(pieces: Sequence[Packed], inside: tuple[np.ndarray, np.ndarray] | None = None,
              cell: float = CELL) -> Grid:
    """Every piece voxelized (:func:`voxelize`) on one grid over the box's ``inside``."""
    lo, hi = inside if inside is not None else box_of(pieces)
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    n = np.maximum(1, np.ceil((hi - lo) / cell - 1e-6)).astype(int)
    shape = (int(n[0]), int(n[1]), int(n[2]))
    occ = [voxelize(pc.meshes, cell, org=lo, shape=shape).occ for pc in pieces]
    count = np.zeros(shape, np.uint8)
    for o in occ:
        count += o
    return Grid(occ, count, lo, hi, cell)


def _fine_overlap(a: Packed, b: Packed, fine: float) -> float:
    """The volume (mm^3) two pieces share, by the cells of ``fine`` whose centres are
    inside both, over their meshes' common bounds."""
    def bounds(pc: Packed) -> tuple[np.ndarray, np.ndarray]:
        p = np.concatenate([np.asarray(q, float) for q, _ in pc.meshes])
        return p.min(axis=0), p.max(axis=0)

    (alo, ahi), (blo, bhi) = bounds(a), bounds(b)
    lo, hi = np.maximum(alo, blo), np.minimum(ahi, bhi)
    if (hi <= lo).any():
        return 0.0
    n = np.maximum(1, np.ceil((hi - lo) / fine)).astype(int)
    shape = (int(n[0]), int(n[1]), int(n[2]))
    if math.prod(shape) > 60_000_000:           # (too fine for that much: coarser)
        f = fine * (math.prod(shape) / 60_000_000) ** (1 / 3)
        return _fine_overlap(a, b, f)
    va = voxelize(a.meshes, fine, org=lo, shape=shape, surface=False).occ
    vb = voxelize(b.meshes, fine, org=lo, shape=shape, surface=False).occ
    return float((va & vb).sum()) * fine ** 3


def column_ranges(occ: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Each column's lowest cell and one past its highest (``BIG`` / ``-BIG``: none)."""
    has = occ.any(axis=1)
    ny = occ.shape[1]
    lo = np.where(has, occ.argmax(axis=1), BIG)
    hi = np.where(has, ny - occ[:, ::-1, :].argmax(axis=1), -BIG)
    return lo, hi


def metrics(pieces: Sequence[Packed], inside: tuple[np.ndarray, np.ndarray] | None = None,
            cell: float = CELL, fine: float = FINE, bands: int = 12,
            grid: Grid | None = None) -> dict:
    """What a packing is worth, from the pieces where it put them (in the order they go
    in): the box (``inside``, default :func:`box_of`; its outside with :data:`WALL`), the
    true fill (the parts' own volumes over the box's inside), its cells' fill, the air per
    layer (``bands`` of the inside's height, bottom up), each piece's mass, centre of mass
    height, how it rests (:func:`support`: on what, directly or on foam pads, or not: each
    of its columns' gap down to another piece's nearest cell or the floor, a cell of foam
    off it), the real-shape overlaps (pieces that share cells, then the volume
    they share at ``fine`` mm), the order's faults (a piece over another that goes in after
    it), the mass centre's height."""
    g = grid or occupancy(pieces, inside, cell)
    lo, hi = g.lo, g.hi
    size = hi - lo
    ny = g.count.shape[1]
    owner = g.owner()
    out_size = size + 2 * WALL           # (a floor and a lid)
    vol = sum(p.volume for p in pieces)
    mass = sum(p.mass for p in pieces)
    com_y = (sum(p.mass * float(p.com[1]) for p in pieces) / mass) if mass > 0 else float("nan")
    # the air per layer
    filled = g.count > 0
    per_layer = 1.0 - filled.mean(axis=(0, 2))
    edges = np.linspace(0, ny, bands + 1).round().astype(int)
    air = [{"y0_mm": round(float(a * cell), 1), "y1_mm": round(float(b * cell), 1),
            "air": round(float(per_layer[a:b].mean()), 4)}
           for a, b in itertools.pairwise(edges) if b > a]
    # each piece: its columns' bottoms and tops, what is under its bottoms (another
    # piece's nearest cell, or the floor: one cell of foam taken off the gap)
    ranges = [column_ranges(o) for o in g.occ]
    depth = math.ceil((PAD_MAX + 2 * CLEAR) / g.cell) + 2
    names = [p.key for p in pieces]
    rows = []
    for i, pc in enumerate(pieces):
        blo, _ = ranges[i]
        ii, kk = np.nonzero(blo < BIG)
        y0 = blo[ii, kk]
        gap = np.maximum(y0 - 1, 0)              # down to the floor
        who = np.full(len(ii), -1)
        found = np.zeros(len(ii), bool)
        others = g.count - g.occ[i]
        for d in range(1, depth + 1):
            y = y0 - d
            look = ~found & (y >= 0)
            hit = np.zeros(len(ii), bool)
            hit[look] = others[ii[look], y[look], kk[look]] > 0
            if hit.any():
                gap[hit] = np.maximum(d - 2, 0)
                w = owner[ii[hit], y[hit], kk[hit]]
                who[hit] = np.where(w == i, -1, w)
                found |= hit
        com_c = (np.asarray(pc.com, float) - lo) / cell
        rest = support(np.stack([ii, kk], axis=1), gap, who, names, com_c[[0, 2]], cell,
                       lo[[0, 2]])
        rows.append({"key": pc.key, "label": pc.label, "order": i + 1,
                     "mass_kg": round(pc.mass, 3), "volume_l": round(pc.volume / 1e6, 3),
                     "com_height_mm": round(float(pc.com[1] - lo[1]), 1),
                     "supported": rest.ok, "rests": rest.how, "on": list(rest.on),
                     "pads": [list(p) for p in rest.pads], "cells": int(g.occ[i].sum())})
    # overlaps: cells two pieces claim, then measured finer
    overlaps = []
    if (g.count > 1).any():
        for i in range(len(pieces)):
            for j in range(i + 1, len(pieces)):
                shared = int((g.occ[i] & g.occ[j]).sum())
                if shared:
                    v = _fine_overlap(pieces[i], pieces[j], fine)
                    overlaps.append({"a": pieces[i].key, "b": pieces[j].key, "cells": shared,
                                     "volume_mm3": round(v, 1)})
    # the order: a piece over another that goes in after it
    faults = []
    for i in range(len(pieces)):
        lo_i, hi_i = ranges[i]
        for j in range(i + 1, len(pieces)):
            lo_j, _ = ranges[j]
            both = (lo_i < BIG) & (lo_j < BIG)
            if (both & (hi_i - 1 > lo_j)).any():     # a cell of i over one of j
                faults.append({"over": pieces[i].key, "under": pieces[j].key})
    return {
        "box": {"inside_mm": [round(float(v), 1) for v in size],
                "outside_mm": [round(float(v), 1) for v in out_size],
                "floor": [round(float(v), 2) for v in lo],
                "volume_l": round(float(np.prod(out_size)) / 1e6, 1),
                "inside_l": round(float(np.prod(size)) / 1e6, 1)},
        "count": len(pieces),
        "order": [p.key for p in pieces],
        "fill": round(vol / float(np.prod(size)), 4),
        "fill_outside": round(vol / float(np.prod(out_size)), 4),
        "cell_fill": round(float(filled.mean()), 4),
        "air_per_layer": air,
        "mass_kg": round(mass, 3),
        "mass_centre_mm": round(com_y - float(lo[1]), 1),
        "mass_centre_frac": round((com_y - float(lo[1])) / float(size[1]), 3),
        "unsupported": [r["key"] for r in rows if not r["supported"]],
        "padded": [r["key"] for r in rows if r["rests"] == "padded"],
        "overlaps": [o for o in overlaps if o["volume_mm3"] > 0],
        "touching": [o for o in overlaps if o["volume_mm3"] <= 0],
        "order_faults": faults,
        "pieces": rows,
        "cell_mm": cell,
    }

