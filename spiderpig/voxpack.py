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
point first and tips onto the next ones), else over pads (blocks cut or printed to fit)
filling gaps up to :data:`PAD_MAX` under it (:class:`Pad`: where, how large, how tall),
else not at all (flagged; it would tip).

**Plans and search** (:class:`Plan`, :func:`pack`, :func:`search`): a plan says per piece
its orientation (a named face down, a turn about the vertical, or any), where it goes (a
corner, on or into another piece, a region) and what goes in first; what it leaves free
the packer picks greedily (:class:`Packer`). :func:`search` anneals the order, the
orientations and the corners against :func:`score` (the box's volume, the pieces that tip,
the pads: each, their volume and the tallest, the mass centre's height, the box's height)
within a time budget, from as many seeds as asked, in worker processes (:func:`spread`:
how far the seeds' answers lie apart).

**Metrics** (:func:`metrics`): any layout, its pieces voxelized afresh on one grid over the
box (:func:`occupancy`), whoever packed it: the box, the true fill (the parts' own volumes
over the box's), the air per layer, how each piece rests (:func:`support`), its pads
measured on the real meshes (:func:`measure_pads`: a cut list), how far it slides before it
meets a neighbour or the wall (:func:`free_travel`), the real-shape overlaps (cells two
pieces share, then measured on :data:`FINE` cells whose centres are inside both), the
order's faults (a piece over another that goes in after it), the mass centre's height, the
order.

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
    mm^3: the parts' own), ``pre`` (its own frame's turn from the assembled pose), its
    named ``faces`` (a name -> the axis, in the turned frame, that faces that way: a side
    panel's ``outer``, ``bay``) and its ``parts``' names (one per mesh, when known)."""

    key: str
    label: str
    role: str
    meshes: list[Mesh]
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    volume: float = 0.0
    pre: np.ndarray = field(default_factory=lambda: np.eye(3))
    faces: dict[str, np.ndarray] = field(default_factory=dict)
    parts: list[str] = field(default_factory=list)


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
"""mm: the deepest gap under a piece a pad fills (a block cut or printed to fit)."""
PAD_SIDE = 15.0
"""mm: a pad's footprint, square, before it is cut to the piece over it (:func:`measure_pads`)."""
PAD_MERGE = 8
"""cells: the support's hull corners this near the first of a group share its pad."""


@dataclass(frozen=True)
class Pad:
    """A pad under a piece, a block cut or printed to fit the gap at a corner of what the
    piece rests on: its footprint ``x0`` .. ``x1`` by ``z0`` .. ``z1`` (mm, the box's frame:
    :data:`PAD_SIDE` square), its ``height`` (mm: the least gap of the piece's padded columns
    over the footprint, on the voxels; :func:`measure_pads` measures the real one) and what
    it stands on (``on``: a piece's key, or ``"floor"``)."""

    x0: float
    z0: float
    x1: float
    z1: float
    height: float
    on: str = "floor"

    @property
    def volume(self) -> float:
        """mm^3."""
        return (self.x1 - self.x0) * (self.z1 - self.z0) * self.height


@dataclass(frozen=True)
class Support:
    """How a piece rests: ``how`` (``"direct"``: its centre of mass over the convex hull of
    its contacts; ``"padded"``: over it once pads fill gaps of up to :data:`PAD_MAX` under
    it; ``"none"``: neither, it would tip), on what (piece keys, ``"floor"``) and the
    ``pads`` (:class:`Pad`)."""

    how: str
    on: tuple[str, ...] = ()
    pads: tuple[Pad, ...] = ()

    @property
    def ok(self) -> bool:
        return self.how != "none"


def support(xz: np.ndarray, gap: np.ndarray, owner: np.ndarray, names: Sequence[str],
            com, cell: float, org=(0.0, 0.0)) -> Support:
    """How a piece rests (:class:`Support`): its columns ``xz`` (cells), each one's ``gap``
    (cells) down to what is under it, whose that is (``owner``: an index into ``names``,
    -1 the floor), its centre of mass (x z, cells). A column within :data:`SETTLE` touches
    (dropped straight a piece meets one point, then tips onto the next ones as it settles);
    one within :data:`PAD_MAX` takes a pad. A pad goes at each group of the support hull's
    padded corners (those within :data:`PAD_MERGE` cells of the group's first): a
    :data:`PAD_SIDE` square centred on them, as tall as the least gap of the padded columns
    under it (a block fits under the lowest), standing on what is under the tightest."""
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
    groups: list[list[int]] = []
    for i in corners.tolist():
        first = next((g for g in groups if abs(xz[g[0], 0] - xz[i, 0]) <= PAD_MERGE
                      and abs(xz[g[0], 1] - xz[i, 1]) <= PAD_MERGE), None)
        if first is None:
            groups.append([i])
        else:
            first.append(i)
    half = PAD_SIDE / 2.0
    mid = np.stack([org[0] + (xz[:, 0] + 0.5) * cell, org[1] + (xz[:, 1] + 0.5) * cell], 1)
    pads = []
    for g in groups:
        c = np.array([org[0] + (xz[g, 0].mean() + 0.5) * cell,
                      org[1] + (xz[g, 1].mean() + 0.5) * cell])
        under = padable & (np.abs(mid - c) <= half).all(axis=1)
        under[g] = True
        tight = np.flatnonzero(under)[int(np.argmin(gap[under]))]
        pads.append(Pad(round(float(c[0] - half), 1), round(float(c[1] - half), 1),
                        round(float(c[0] + half), 1), round(float(c[1] + half), 1),
                        round(float(gap[tight] * cell), 1),
                        "floor" if owner[tight] < 0 else names[int(owner[tight])]))
    used = direct.copy()
    used[corners] = True
    return Support("padded", on(used), tuple(pads))


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
    in, the choices that made it (``seq``) and its :func:`score`. A piece with no room (its
    plan's step can't be met) stops it: what is placed so far, an infinite score, the
    ``warnings`` and the pieces not placed (``missing``: it, and those after it). ``runs``:
    :func:`search`'s, one per seed (and its greedy start)."""

    placed: list[Placed]
    bin: Bin
    seq: list[Choice]
    score: float = math.inf
    warnings: list[str] = field(default_factory=list)
    missing: list[str] = field(default_factory=list)
    runs: list[dict] = field(default_factory=list)

    def by_key(self) -> dict[str, Placed]:
        return {p.key: p for p in self.placed}


W_UNSUPPORTED = 25.0    # litres a piece whose centre of mass overhangs what it rests on costs
W_PAD = 0.4             # litres a pad costs (one more block cut or printed, then set blind):
#                         a piece on five, the old mean, costs the 2 L a padded piece once did
W_PAD_VOLUME = 10.0     # litres a litre of pads costs (their material)
W_PAD_TALL = 5.0        # litres a metre of the tallest pad costs (a tall pad is a column)
W_HIGH = 10.0           # litres for the mass centre at the box's top (0 at its floor)
W_TALL = 10.0           # litres a metre of box height costs (a tall box tips, stacks badly)
WALL = 5.0              # the box's corrugated wall, mm


def box_litres(bin_: Bin) -> float:
    """The box's outside volume (litres) round what is in ``bin_``: its cells' extent,
    :data:`CLEAR` and :data:`WALL` each side."""
    c = bin_.cell
    dx = max(bin_.xr[1] - bin_.xr[0], 0) * c + 2 * (CLEAR + WALL)
    dz = max(bin_.zr[1] - bin_.zr[0], 0) * c + 2 * (CLEAR + WALL)
    dy = bin_.height * c + 2 * (CLEAR + WALL)
    return dx * dy * dz / 1e6


def outside_litres(items: Sequence[Item], layout: Layout) -> float:
    """The box's outside volume (litres) round ``layout``'s pieces' real meshes, not their
    cells: their bounds, :data:`CLEAR` and :data:`WALL` each side (the box
    ``cradle_pack.layout_payload`` and :func:`metrics` give it)."""
    by = {it.key: it for it in items}
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for p in layout.placed:
        for pos, tri in by[p.key].meshes:
            if len(tri):
                q = np.asarray(pos, float) @ p.rot.T + p.shift
                lo, hi = np.minimum(lo, q.min(axis=0)), np.maximum(hi, q.max(axis=0))
    if not np.isfinite(lo).all():
        return 0.0
    return float(np.prod(hi - lo + 2 * (CLEAR + WALL))) / 1e6


def pad_cost(pads: Sequence[Pad]) -> float:
    """What ``pads`` cost a layout (litres of box): :data:`W_PAD` each,
    :data:`W_PAD_VOLUME` per litre of them, :data:`W_PAD_TALL` per metre of the tallest:
    of two layouts in one box, the one on fewer and smaller pads wins."""
    if not pads:
        return 0.0
    return (W_PAD * len(pads) + W_PAD_VOLUME * sum(p.volume for p in pads) / 1e6
            + W_PAD_TALL * max(p.height for p in pads) / 1000.0)


def score(bin_: Bin, masses: dict[str, float] | None = None) -> float:
    """A layout's cost: its box's outside volume (:func:`box_litres`),
    :data:`W_UNSUPPORTED` per piece that tips, its pads' :func:`pad_cost`, :data:`W_HIGH`
    times the mass centre's height over the box's, :data:`W_TALL` per metre of height.
    ``masses``: each piece's (kg)."""
    c = bin_.cell
    dy = bin_.height * c + 2 * (CLEAR + WALL)
    s = box_litres(bin_)
    s += W_UNSUPPORTED * sum(not p.supported for p in bin_.placed)
    s += pad_cost([q for p in bin_.placed for q in p.rest.pads])
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
        for n, ch in enumerate(seq):
            _, p = self.drop(b, ch.key, [ch.ri], [ch.corner])
            if p is None:
                return Layout(b.placed, b, list(seq), math.inf, [self._no_room(ch.key)],
                              [c.key for c in seq[n:]])
            if prefix is not None:
                prefix.append(b.copy())
        return Layout(b.placed, b, list(seq), score(b, self.masses))

    def greedy(self, order: Sequence[str] | None = None) -> Layout:
        """Each piece in turn (``order``, default :meth:`order`), its best orientation and
        corner (:meth:`drop`)."""
        b = self.fresh()
        seq = []
        keys = list(order or self.order())
        for n, key in enumerate(keys):
            got, p = self.drop(b, key, self.allowed(key), self.corners(key))
            if p is None:
                return Layout(b.placed, b, seq, math.inf, [self._no_room(key)], keys[n:])
            seq.append(got[1])
        return Layout(b.placed, b, seq, score(b, self.masses))

    def _no_room(self, key: str) -> str:
        """Why ``key`` stopped a packing: no room where its step puts it."""
        st = self.plan.step(key)
        where = [f"{k} {v}" for k, v in (("on", st.on), ("into", st.into), ("region", st.region),
                                          ("down", st.down)) if v is not None] if st else []
        return f"{key}: no room" + (f" ({', '.join(where)})" if where else "")

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
           log: Callable[[str], None] | None = None, seeds: int | None = None) -> Layout:
    """The best packing :func:`anneal` finds round ``plan`` (from its greedy packing,
    :meth:`Packer.best_greedy`) from ``seeds`` seeds (``seed``, ``seed + 1``, ..; default
    one per process), each for ``seconds``, ``processes`` worker processes at once (at most
    6; ``SPIDERPIG_WORKERS=0``: one after another, in this process); packed again here. Its
    ``runs``: each seed's best (``score``, its box's ``litres``: :func:`outside_litres`, its
    ``pads``, the sequences ``tried``) and the greedy start's (seed None), so a caller can
    say how far another seed or more time might move it (:func:`spread`)."""
    from spiderpig import workers

    processes = max(1, min(6, processes))
    n = max(1, seeds if seeds is not None else processes)
    pk = Packer(items, plan, cell)
    start = pk.best_greedy()
    if log:
        log(f"greedy: score {start.score:.1f}")
    seq0 = list(start.seq)
    got: list[tuple[int | None, tuple[float, list[Choice], int]]] = []
    if workers.enabled() and processes > 1:
        for wave in range(0, n, processes):         # (``processes`` at a time)
            jobs = [(seed + i, workers.submit(anneal, list(items), plan, seq0, seconds,
                                              seed + i, cell))
                    for i in range(wave, min(n, wave + processes))]
            got += [(s, j.result()) for s, j in jobs]
    else:
        got = [(seed + i, anneal(items, plan, seq0, seconds, seed + i, cell)) for i in range(n)]
    got.append((None, (start.score, seq0, 1)))
    runs = []
    for s, (sc, seq, tried) in got:
        lay = pk.run(seq)
        runs.append({"seed": s, "score": round(sc, 2) if math.isfinite(sc) else None,
                     "litres": round(outside_litres(items, lay), 1) if not lay.missing
                     else None,
                     "pads": sum(len(p.rest.pads) for p in lay.placed), "tried": tried})
    _, (_, seq, _) = min(got, key=lambda g: g[1][0])
    if log:
        log("anneal: " + ", ".join(f"seed {r['seed']} {r['score']} ({r['tried']} tried)"
                                   for r in runs[:-1]))
    out = pk.run(seq)
    out.runs = runs
    return out


def spread(runs: Sequence[dict], near: float = 0.5) -> dict:
    """How robust a :func:`search` is, from its ``runs`` (the seeds', not its greedy
    start): how many seeds, the best, the median and the worst of their scores and box
    litres, the litres' spread (worst less best) and how many seeds came within ``near``
    litres of the best box."""
    seeds = [r for r in runs if r["seed"] is not None]
    out: dict = {"seeds": len(seeds)}
    for k in ("score", "litres"):
        v = sorted(r[k] for r in seeds if r[k] is not None)
        if v:
            out[k] = {"best": v[0], "median": round(float(np.median(v)), 2), "worst": v[-1]}
    if "litres" in out:
        lit = out["litres"]
        lit["spread"] = round(lit["worst"] - lit["best"], 1)
        out["at_best"] = sum(r["litres"] is not None and r["litres"] <= lit["best"] + near
                             for r in seeds)
    return out


# -- judging a packing -----------------------------------------------------------------------


@dataclass
class Packed:
    """A piece where a packing put it, whoever packed it: its parts' meshes in the box's
    frame, its mass (kg), centre of mass (mm, that frame), volume (mm^3, its parts') and
    its ``parts``' names (one per mesh, when known)."""

    key: str
    label: str
    meshes: list[Mesh]
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    volume: float = 0.0
    parts: list[str] = field(default_factory=list)


class Rays:
    """Vertical rays through packed pieces' real meshes (no voxels): every part's
    triangles at once, each one's part and piece (:func:`measure_pads`)."""

    def __init__(self, pieces: Sequence[Packed]):
        pos, tri, part = [], [], []
        self.names: list[str] = []          # each part's
        self.piece: list[int] = []          # each part's piece (an index into ``pieces``)
        base = 0
        for i, pc in enumerate(pieces):
            for m, (p, t) in enumerate(pc.meshes):
                t = np.asarray(t, np.int64).reshape(-1, 3)
                if not len(t):
                    continue
                part.append(np.full(len(t), len(self.names), np.int64))
                self.names.append(pc.parts[m] if m < len(pc.parts) else f"{pc.key}#{m}")
                self.piece.append(i)
                pos.append(np.asarray(p, float))
                tri.append(t + base)
                base += len(p)
        self.pos = np.concatenate(pos) if pos else np.zeros((0, 3))
        self.tri = np.concatenate(tri) if tri else np.zeros((0, 3), np.int64)
        self.part = np.concatenate(part) if part else np.zeros(0, np.int64)
        xs, zs = self.pos[self.tri, 0], self.pos[self.tri, 2]
        self.lo = np.stack([xs.min(axis=1), zs.min(axis=1)], 1)     # (each triangle's x z)
        self.hi = np.stack([xs.max(axis=1), zs.max(axis=1)], 1)

    def cast(self, x0: float, z0: float, side: float, n: int
             ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Where ``n`` x ``n`` rays up through the square ``side`` mm from ``(x0, z0)``
        (each through its cell's centre) cross the parts: ``(ray, part, y)``, ray ``i * n +
        k``."""
        sel = np.flatnonzero((self.hi[:, 0] >= x0) & (self.lo[:, 0] <= x0 + side)
                             & (self.hi[:, 1] >= z0) & (self.lo[:, 1] <= z0 + side))
        if not len(sel):
            return np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0)
        key, y = _column_crossings(self.pos, self.tri[sel], self.part[sel],
                                   np.array([x0, 0.0, z0]), side / n, n, n)
        return key % (n * n), key // (n * n), y


PAD_TRIES = (9.0, 3.0)
"""mm: the smaller squares :func:`measure_pads` tries where a pad's block doesn't fit."""


def measure_pads(pieces: Sequence[Packed], i: int, pads: Sequence[Pad], floor: float,
                 rays: Rays | None = None, step: float = 1.0) -> list[dict]:
    """Piece ``i``'s ``pads`` (:class:`Pad`, from the voxels) measured on the real meshes,
    each a rectangular block that fits where it stands: rays every ``step`` mm through its
    square; its footprint the rays' that meet the piece (the square cut to the piece over
    it), its top the piece's lowest underside over that footprint (the part it bears up
    against), its base the highest top of what is under it there (another piece's part, or
    the box's ``floor``: y, mm). So its height is the real gap, not rounded to the voxels,
    and the least over its footprint (a block, not a shaped one). A square whose block
    meets something (what is under it reaches past the piece's lowest underside, or a part
    of another piece runs through the block) is tried smaller (:data:`PAD_TRIES`); where
    none fits, the smallest that meets the piece, ``fits`` false and its height 0. Each a
    dict: ``footprint`` (x0, z0, x1, z1: mm, the box's frame), ``size_mm``, ``height_mm``
    (the real gap), ``voxel_mm`` (the voxels'), ``base_mm`` (the y it stands at), ``under``
    (the piece's part over it), ``on`` (the piece under it, or ``"floor"``), ``on_part``
    and ``fits`` (None, and the voxels' footprint and height, where no ray meets the piece:
    unmeasured)."""
    rays = rays or Rays(pieces)
    own = np.flatnonzero(np.asarray(rays.piece) == i)
    nparts = max(len(rays.names), 1)
    out = []
    for pad in pads:
        cx, cz = (pad.x0 + pad.x1) / 2, (pad.z0 + pad.z1) / 2
        row = None
        sides = [pad.x1 - pad.x0] + [s for s in PAD_TRIES if s < pad.x1 - pad.x0]
        for side in sides:
            n = max(1, round(side / step))
            c = side / n
            ray, part, y = rays.cast(cx - side / 2, cz - side / 2, side, n)
            mine = np.isin(part, own)
            bottom = np.full(n * n, np.inf)
            np.minimum.at(bottom, ray[mine], y[mine])
            hit = np.isfinite(bottom)
            if not hit.any():
                continue
            ii, kk = np.divmod(np.flatnonzero(hit), n)
            ri, rk = np.divmod(ray, n)
            inside = ((ri >= ii.min()) & (ri <= ii.max()) & (rk >= kk.min())
                      & (rk <= kk.max()))          # (the footprint's rays, the piece's or not)
            low = float(bottom[hit].min())
            below = inside & ~mine & (y <= low + 1e-6)
            top = float(y[below].max()) if below.any() else floor
            # a part across the block's top: in below it, out above (on a closed mesh, an odd
            # count of its crossings below; an open one's, any crossing either side)
            pair = ray * nparts + part
            sel = inside & ~mine
            keys, total = np.unique(pair[sel], return_counts=True)
            under_n = np.zeros(len(keys), np.int64)
            if below.any():
                k_below, n_below = np.unique(pair[below], return_counts=True)
                under_n[np.searchsorted(keys, k_below)] = n_below
            across = np.where(total % 2 == 0, under_n % 2 == 1,
                              (under_n > 0) & (under_n < total))
            fits = top < low - 0.05 and not across.any()
            k = int(np.argmin(np.where(mine, y, np.inf)))
            on, on_part = "floor", "floor"
            if below.any():
                q = int(part[below][np.argmax(y[below])])
                on, on_part = pieces[rays.piece[q]].key, rays.names[q]
            x0, z0 = cx - side / 2, cz - side / 2
            fp = (x0 + ii.min() * c, z0 + kk.min() * c,
                  x0 + (ii.max() + 1) * c, z0 + (kk.max() + 1) * c)
            row = {"footprint": [round(float(v), 1) for v in fp],
                   "size_mm": [round(float(fp[2] - fp[0]), 1), round(float(fp[3] - fp[1]), 1)],
                   "height_mm": round(low - top, 1) if fits else 0.0,
                   "voxel_mm": pad.height, "base_mm": round(top, 1),
                   "under": rays.names[int(part[k])], "on": on, "on_part": on_part,
                   "fits": bool(fits)}
            if fits:
                break
        if row is None:                     # (no ray met the piece: the voxels' word)
            row = {"footprint": [pad.x0, pad.z0, pad.x1, pad.z1],
                   "size_mm": [round(pad.x1 - pad.x0, 1), round(pad.z1 - pad.z0, 1)],
                   "height_mm": pad.height, "voxel_mm": pad.height, "base_mm": None,
                   "under": None, "on": pad.on, "on_part": None, "fits": None}
        out.append(row)
    return out


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


TRAVEL = {"+x": (0, 1), "-x": (0, -1), "+z": (2, 1), "-z": (2, -1), "+y": (1, 1)}
"""The ways :func:`free_travel` slides a piece: along the box's floor (x, z) and up."""


def free_travel(g: Grid, i: int, owner: np.ndarray | None = None
                ) -> dict[str, tuple[float, int]]:
    """How far (mm) piece ``i`` of grid ``g`` slides each way of :data:`TRAVEL`, rigid,
    before a cell of it meets another piece's or the box's inside wall (``-1``): the empty
    cells between, a cell each. The cells are the parts' conservative fill (every cell
    their surface touches), so the real travel is that or up to two cells more. Each way:
    (mm, whose cell it meets: an index into the grid's pieces, -1 the wall)."""
    owner = g.owner() if owner is None else owner
    mine = g.occ[i]
    if not mine.any():
        return {}
    idx = np.nonzero(mine)
    lo_b = [int(v.min()) for v in idx]
    hi_b = [int(v.max()) + 1 for v in idx]
    out = {}
    for way, (a, sgn) in TRAVEL.items():
        # (only the rows through the piece: its bounds across ``a``, the whole box along it)
        sl = tuple(slice(None) if k == a else slice(lo_b[k], hi_b[k]) for k in range(3))
        others = (g.count[sl] - mine[sl]) > 0
        m = mine[sl]
        n = others.shape[a]
        shape = [1, 1, 1]
        shape[a] = n
        at = np.arange(n).reshape(shape)
        if sgn > 0:      # the nearest other cell at or past each one (n: the wall)
            nxt = np.flip(np.minimum.accumulate(np.flip(np.where(others, at, n), a), axis=a),
                          a)
            gap = nxt - at - 1
        else:
            nxt = np.maximum.accumulate(np.where(others, at, -1), axis=a)
            gap = at - nxt - 1
        t = np.where(m, gap, np.iinfo(np.int64).max)
        j = np.unravel_index(int(np.argmin(t)), t.shape)
        cells = max(int(t[j]), 0)
        hit = list(j)
        hit[a] = j[a] + sgn * (int(t[j]) + 1)
        who = -1
        if 0 <= hit[a] < n:
            hit_g = tuple(hit[k] + (0 if k == a else lo_b[k]) for k in range(3))
            who = int(owner[hit_g])
        out[way] = (cells * g.cell, who)
    return out


# -- spacers: printed blocks where the pads were ---------------------------------------------

SPACER_GROW = 4     # cells: a spacer's footprint past the pads it takes the place of, each way
SPACER_LIP = 3      # cells: how far it rises past the underside it carries (sinks past the
#                     top it stands on) beside them, where nothing is over (under) it: its
#                     lips, which hold the piece over it (and it) from sliding
SPACER_JOIN = 30.0  # mm: one piece's pads this near each other share a spacer


@dataclass
class Spacer:
    """A printed spacer (:func:`spacers`), the block a support's pads become: ``key``
    (``spacer_4a``, the first pad it takes the place of), the piece it carries (``under``)
    and what it stands on (``on``: pieces' keys, ``"floor"``), the pads it replaces
    (``pads``: their ids), the parts over and under it there (``bears``) and its cells on
    the judged grid (``org``, ``cell``), one run per column (``cols``: i, k, j0, j1, the run
    j0 .. j1 - 1), each from what is under it to what it carries, or to its lips."""

    key: str
    under: str
    on: tuple[str, ...]
    pads: tuple[str, ...]
    cols: np.ndarray
    org: np.ndarray
    cell: float
    bears: tuple[str, ...] = ()
    rank: float = 0.0
    """Where it goes in the order: after the pieces (their indices) under it, before those
    over it (a support just before what it carries; a restraint as soon as what is under
    it is in)."""
    why: str = "support"
    """``"support"`` (in place of pads) or ``"restraint +x"`` .. (beside a piece, the way it
    would slide)."""

    @property
    def volume(self) -> float:
        """mm^3."""
        return float((self.cols[:, 3] - self.cols[:, 2]).sum()) * self.cell ** 3

    def vox(self) -> Vox:
        """Its cells as a grid of their own (its bounds)."""
        i, k, j0, j1 = (self.cols[:, n] for n in range(4))
        lo = np.array([i.min(), j0.min(), k.min()])
        occ = np.zeros((int(i.max()) + 1 - lo[0], int(j1.max()) - lo[1],
                        int(k.max()) + 1 - lo[2]), bool)
        ys = np.arange(occ.shape[1]) + lo[1]
        col = (ys[None, :] >= j0[:, None]) & (ys[None, :] < j1[:, None])
        occ[i - lo[0], :, k - lo[2]] = col
        return Vox(occ, np.asarray(self.org, float) + lo * self.cell, self.cell)

    def mesh(self) -> Mesh:
        """Its closed surface (:func:`vox_mesh`), the box's frame."""
        return vox_mesh(self.vox())

    def size_mm(self) -> list[float]:
        v = self.vox()
        return [round(float(n * self.cell), 1) for n in v.occ.shape]

    def to_json(self) -> dict:
        return {"key": self.key, "under": self.under, "on": list(self.on),
                "pads": list(self.pads), "bears": list(self.bears),
                "org": [float(v) for v in self.org], "cell": self.cell,
                "cols": self.cols.astype(int).tolist(),
                "volume_cm3": round(self.volume / 1000.0, 1), "size_mm": self.size_mm(),
                "rank": self.rank, "why": self.why}

    @classmethod
    def from_json(cls, d: dict) -> Spacer:
        return cls(d["key"], d["under"], tuple(d["on"]), tuple(d["pads"]),
                   np.asarray(d["cols"], np.int64).reshape(-1, 4),
                   np.asarray(d["org"], float), float(d["cell"]), tuple(d.get("bears", ())),
                   float(d.get("rank", 0.0)), d.get("why", "support"))


def vox_mesh(v: Vox) -> Mesh:
    """A voxel grid's closed surface: each face between a full cell and an empty one (or the
    grid's edge), two triangles, outward."""
    pad = np.pad(v.occ, 1)
    verts, tris = [], []
    base = 0
    for ax in range(3):
        u, w = (a for a in range(3) if a != ax)
        for sgn in (1, -1):
            face = pad & ~np.roll(pad, -sgn, axis=ax)
            idx = np.argwhere(face[1:-1, 1:-1, 1:-1]).astype(float)
            if not len(idx):
                continue
            if sgn > 0:
                idx[:, ax] += 1.0
            q = np.repeat(idx[:, None, :], 4, axis=1)
            q[:, 1, u] += 1.0
            q[:, 2, u] += 1.0
            q[:, 2, w] += 1.0
            q[:, 3, w] += 1.0
            m = len(idx)
            b = base + 4 * np.arange(m)
            # (u, w) in increasing order: their cross product is +ax but for ax = y
            if (sgn > 0) == (ax != 1):
                t = np.concatenate([np.stack([b, b + 1, b + 2], 1), np.stack([b, b + 2, b + 3], 1)])
            else:
                t = np.concatenate([np.stack([b, b + 2, b + 1], 1), np.stack([b, b + 3, b + 2], 1)])
            verts.append(q.reshape(-1, 3) * v.cell + v.org)
            tris.append(t)
            base += 4 * m
    if not verts:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    return np.concatenate(verts), np.concatenate(tris)


def _pad_clusters(pads: Sequence[dict], join: float) -> list[list[dict]]:
    """``pads`` (measured: :func:`measure_pads`) grouped: two whose footprints come within
    ``join`` mm share a group (single linkage)."""
    n = len(pads)
    parent = list(range(n))

    def root(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b in itertools.combinations(range(n), 2):
        fa, fb = pads[a]["footprint"], pads[b]["footprint"]
        gap = max(fa[0] - fb[2], fb[0] - fa[2], fa[1] - fb[3], fb[1] - fa[3], 0.0)
        if gap <= join:
            parent[root(a)] = root(b)
    groups: dict[int, list[dict]] = {}
    for i in range(n):
        groups.setdefault(root(i), []).append(pads[i])
    return list(groups.values())


SPACER_STEP = 3     # cells: a loose spacer's footprint grows this far a step
SPACER_SPAN = 60    # cells: the most a spacer's footprint spans either way (180 mm: a print bed)
SPACER_REACH = 20   # cells: how far a restraint reaches down to stand on something
SPACER_HOLD = 1     # cells: the most a piece or a spacer may slide (one cell: up to 3 mm)
SPACER_MAX = 400.0  # cm^3: a spacer grows no bigger (its solid)
WAYS = {"+x": (0, 1), "-x": (0, -1), "+z": (2, 1), "-z": (2, -1)}
"""The ways a spacer is grown for and a piece restrained: along the box's floor."""


@dataclass
class _Spec:
    """A spacer being made: its footprint (rectangles of columns ``i0 i1 k0 k1``, ends one
    past), each column's seed (``seed``: its cell, the lowest and the one past the highest it
    may fill, -1 none), its rank in the order (it goes in after the pieces ranked under it
    and before those over), the piece it carries or holds, the pads it replaces, the parts
    it bears on, how it grows when it slides (``"away"``: its far side, to wrap what it
    carries; ``"toward"``: its near side, to meet what would stop it) and its pads' seeds."""

    rects: list[tuple[int, int, int, int]]
    seed: np.ndarray
    rank: float | None
    under: str
    pads: tuple[str, ...]
    bears: tuple[str, ...]
    grow: str = "away"
    pad_seeds: list[tuple[float, float, int, int, int]] = field(default_factory=list)
    why: str = "support"


class _Spacers:
    """The judged grid ``g`` with every spacer made so far in it (:func:`spacers`)."""

    def __init__(self, g: Grid, keys: Sequence[str]):
        self.g = g
        self.occ = g.count > 0
        self.owner = g.owner().astype(np.int32)
        self.keys = list(keys)
        self.ranks = [float(i) for i in range(len(keys))]      # by owner (-2 - n: spacers)
        self.names = list(keys)
        self.made: list[Spacer] = []
        self.shape = self.occ.shape
        # the cells a spacer may take: inside the pieces' bounds (the box's clearance, a
        # cell all round, stays the box's), each axis's past the last
        self.end = tuple(math.floor((float(g.hi[a]) - CLEAR - float(g.lo[a])) / g.cell
                                    + 1e-6) for a in range(3))

    def ids(self, name: str) -> list[int]:
        """The grid owners named ``name``: a piece's index, or a spacer's (-2 - n)."""
        return ([o for o, k in enumerate(self.keys) if k == name]
                + [-2 - n for n, sp in enumerate(self.made) if sp.key == name])

    def rank(self, o: int) -> float:
        return self.ranks[o] if o >= 0 else self.ranks[len(self.keys) - 2 - o]

    def name(self, o: int) -> str:
        return self.names[o] if o >= 0 else self.names[len(self.keys) - 2 - o]

    def pad_seed(self, seeds: Sequence[tuple[float, float, int, int, int]]) -> np.ndarray:
        """Each column's seed from pads' seeds (x, z mm; cell, lowest, one past highest):
        the nearest pad's."""
        nx, _, nz = self.shape
        g = self.g
        cx = g.lo[0] + (np.arange(nx) + 0.5) * g.cell
        cz = g.lo[2] + (np.arange(nz) + 0.5) * g.cell
        s = np.asarray(seeds, float)
        d = ((cx[:, None, None] - s[None, None, :, 0]) ** 2
             + (cz[None, :, None] - s[None, None, :, 1]) ** 2)
        near = np.argmin(d, axis=2)
        return s[near][..., 2:].astype(np.int64)

    def ranks_of(self, own: np.ndarray) -> np.ndarray:
        """The order's rank of each grid owner in ``own`` (pieces' indices, spacers' -2 - n;
        an empty cell's -1: nan)."""
        m = len(self.made)
        lut = np.full(m + 1 + len(self.keys), np.nan)
        lut[m + 1:] = self.ranks[:len(self.keys)]
        if m:
            lut[:m] = self.ranks[len(self.keys):][::-1]     # (-2 - n at m - 1 - n)
        return lut[own + m + 1]

    def columns(self, sp: _Spec) -> np.ndarray:
        """The spacer's runs (i, k, j0, j1): each footprint column's empty run round its
        seed (a restraint's: its seed or the first empty cell over it), within its seed's
        bounds; not a column under a piece that goes in before it, nor over one that goes in
        after it (a restraint goes in as soon as what is under it is); the 6-connected
        parts of them that touch what it carries or holds, or hold one of its pads' seeds."""
        from scipy import ndimage

        nx, ny, nz = self.shape
        cols = np.zeros((nx, nz), bool)
        for i0, i1, k0, k1 in sp.rects:
            cols[max(i0, 1):min(i1, self.end[0]), max(k0, 1):min(k1, self.end[2])] = True
        ii, kk = np.nonzero(cols)
        empty = np.zeros((0, 4), np.int64)
        if not len(ii):
            return empty
        j, cap0, cap1 = (sp.seed[ii, kk, n].astype(np.int64) for n in range(3))
        prof = self.occ[ii, :, kk]
        at = np.arange(ny)[None, :]
        if sp.rank is None:             # (a restraint: the first empty cell from its seed)
            free = ~prof & (at >= np.maximum(j, 1)[:, None]) & (at < np.minimum(cap1, ny - 1)
                                                                  [:, None])
            j = np.where(free.any(axis=1), free.argmax(axis=1), -1)
        jj = np.clip(j, 0, ny - 1)
        ok = (j >= 1) & (j < ny - 1) & ~prof[np.arange(len(ii)), jj]
        a = np.where(prof & (at < jj[:, None]), at, -1).max(axis=1) + 1
        b = np.where(prof & (at >= jj[:, None]), at, ny).min(axis=1)
        j0 = np.maximum(np.maximum(a, cap0), 1)
        j1 = np.minimum(np.minimum(b, cap1), self.end[1])
        ok &= j1 > j0
        rk = self.ranks_of(self.owner[ii, :, kk])
        over = np.where(prof & (at >= j1[:, None]), rk, np.inf).min(axis=1)
        under = np.where(prof & (at < j0[:, None]), rk, -1.0).max(axis=1)
        rank = sp.rank
        if rank is None:                # in as soon as what is under it is
            rank = float(under[ok].max(initial=-1.0)) + 0.25
        ok &= (over > rank) & (under < rank)
        if not ok.any():                # (it couldn't go in under the one, or over the other)
            return empty
        runs = np.stack([ii[ok], kk[ok], j0[ok], j1[ok]], axis=1)
        lo = np.array([runs[:, 0].min() - 1, runs[:, 2].min() - 1, runs[:, 1].min() - 1])
        hi = np.array([runs[:, 0].max() + 2, runs[:, 3].max() + 1, runs[:, 1].max() + 2])
        lo, hi = np.maximum(lo, 0), np.minimum(hi, self.shape)
        box = tuple(slice(int(u), int(v)) for u, v in zip(lo, hi, strict=True))
        mask = np.zeros(tuple(int(v) for v in hi - lo), bool)
        ys = np.arange(lo[1], hi[1])[None, :]
        mask[runs[:, 0] - lo[0], :, runs[:, 1] - lo[2]] = ((ys >= runs[:, 2:3])
                                                          & (ys < runs[:, 3:4]))
        lab, _ = ndimage.label(mask)
        carried = np.isin(self.owner[box], self.ids(sp.under))
        holds = set(np.unique(lab[ndimage.binary_dilation(carried) & mask]).tolist())
        for x, z, js, _, _ in sp.pad_seeds:
            i = int((x - self.g.lo[0]) / self.g.cell) - lo[0]
            k = int((z - self.g.lo[2]) / self.g.cell) - lo[2]
            q = js - lo[1]
            if 0 <= i < mask.shape[0] and 0 <= k < mask.shape[2] and 0 <= q < mask.shape[1]:
                holds.add(int(lab[i, q, k]))
        holds.discard(0)
        col = lab[runs[:, 0] - lo[0], runs[:, 2] - lo[1], runs[:, 1] - lo[2]]
        return runs[np.isin(col, sorted(holds))]

    def mask(self, cols: np.ndarray) -> np.ndarray:
        """Runs (i, k, j0, j1) as the grid's cells."""
        m = np.zeros(self.shape, bool)
        if len(cols):
            ys = np.arange(self.shape[1])[None, :]
            m[cols[:, 0], :, cols[:, 1]] = (ys >= cols[:, 2:3]) & (ys < cols[:, 3:4])
        return m

    def slide_runs(self, cols: np.ndarray) -> dict[str, int]:
        """:meth:`slide` of a spacer's runs (i, k, j0, j1), on the slabs through them."""
        if not len(cols):
            return dict.fromkeys(WAYS, 0)
        lo = [int(cols[:, 0].min()), int(cols[:, 2].min()), int(cols[:, 1].min())]
        hi = [int(cols[:, 0].max()) + 1, int(cols[:, 3].max()), int(cols[:, 1].max()) + 1]
        return self.slide(cols, (lo, hi))

    def slide(self, mine, bounds=None) -> dict[str, int]:
        """How many cells the cells ``mine`` slide each way of :data:`WAYS` before one meets
        a full cell (another's: the grid's and the spacers' made so far) or the box's
        wall. ``mine``: a mask of the grid, or runs (i, k, j0, j1) with their ``bounds``."""
        if bounds is None:
            idx = np.nonzero(mine)
            if not len(idx[0]):
                return dict.fromkeys(WAYS, 0)
            lo_b = [int(v.min()) for v in idx]
            hi_b = [int(v.max()) + 1 for v in idx]
        else:
            lo_b, hi_b = bounds
        out = {}
        for way, (a, sgn) in WAYS.items():
            sl = tuple(slice(None) if k == a else slice(lo_b[k], hi_b[k]) for k in range(3))
            if bounds is None:
                m = mine[sl]
            else:                       # (the runs' cells in the slab)
                m = np.zeros(self.occ[sl].shape, bool)
                ys = np.arange(lo_b[1], hi_b[1])[None, :]
                ci = mine[:, 0] - (0 if a == 0 else lo_b[0])
                ck = mine[:, 1] - (0 if a == 2 else lo_b[2])
                m[ci, :, ck] = (ys >= mine[:, 2:3]) & (ys < mine[:, 3:4])
            o = self.occ[sl] & ~m
            n = o.shape[a]
            at = np.arange(n).reshape([n if k == a else 1 for k in range(3)])
            if sgn > 0:
                nxt = np.flip(np.minimum.accumulate(np.flip(np.where(o, at, n), a), axis=a), a)
                gap = nxt - at - 1
            else:
                nxt = np.maximum.accumulate(np.where(o, at, -1), axis=a)
                gap = at - nxt - 1
            out[way] = int(np.where(m, gap, np.iinfo(np.int64).max).min())
        return out

    def commit(self, sp: _Spec, cols: np.ndarray, key: str) -> Spacer:
        """The spacer made: its cells full in the grid from now on (a piece for the next).
        A restraint's rank is then fixed: just after the most of what is under it. A key
        already made gets ``_2``, ``_3`` .."""
        idx = -2 - len(self.made)
        taken = {sp.key for sp in self.made}
        n, base = 2, key
        while key in taken:
            key, n = f"{base}_{n}", n + 1
        if sp.rank is None:
            sp.rank = max((self.rank(int(self.owner[i, j0 - 1, k]))
                           for i, k, j0, _ in cols.tolist()
                           if j0 >= 1 and self.occ[i, j0 - 1, k]), default=-1.0) + 0.25
        under = {int(self.owner[i, j0 - 1, k]) for i, k, j0, _ in cols.tolist()
                 if j0 >= 1 and self.occ[i, j0 - 1, k]}
        floor = any(j0 <= 1 for _, _, j0, _ in cols.tolist())
        on = sorted({self.name(o) for o in under}) + (["floor"] if floor else [])
        out = Spacer(key, sp.under, tuple(on), sp.pads, cols, self.g.lo.copy(), self.g.cell,
                     sp.bears, float(sp.rank), sp.why)
        for i, k, j0, j1 in cols:
            self.occ[i, j0:j1, k] = True
            self.owner[i, j0:j1, k] = idx
        self.ranks.append(sp.rank)
        self.names.append(key)
        self.made.append(out)
        return out

    def merge(self, n: int, cols: np.ndarray) -> None:
        """Spacer ``n`` (made) with the cells ``cols`` too: one print (a restraint of it)."""
        sp = self.made[n]
        both = _runs(self.mask(sp.cols) | self.mask(cols))
        for i, k, j0, j1 in cols:
            self.occ[i, j0:j1, k] = True
            self.owner[i, j0:j1, k] = -2 - n
        sp.cols = both

    def grow(self, sp: _Spec) -> np.ndarray:
        """The spacer's runs, its footprint grown :data:`SPACER_STEP` cells a step for each
        way it slides more than :data:`SPACER_HOLD` (its far side, to wrap round what it
        carries, or its near side, to meet what would stop it: :class:`_Spec`'s ``grow``;
        the other when a step frees nothing), until it is held or it can't grow
        (:data:`SPACER_SPAN`, :data:`SPACER_MAX`); the best of its steps (the least it
        slides past :data:`SPACER_HOLD`, then the fewest cells) kept, ``sp.rects`` its."""
        def excess(free: dict[str, int]) -> int:
            return sum(max(0, d - SPACER_HOLD) for d in free.values())

        cols = self.columns(sp)
        if not len(cols):
            return cols
        free = self.slide_runs(cols)
        best = (excess(free), len(cols), list(sp.rects), cols)
        side = dict.fromkeys(WAYS, sp.grow)
        for _ in range(40):
            loose = [w for w, d in free.items() if d > SPACER_HOLD and side[w]]
            vol = float((cols[:, 3] - cols[:, 2]).sum()) * self.g.cell ** 3 / 1000.0
            if not loose or vol > SPACER_MAX:
                break
            rects = list(sp.rects)
            for w in loose:
                a, sgn = WAYS[w]
                far = (side[w] == "away") == (sgn > 0)       # (the side at -way: low)
                lo = min(r[0 if a == 0 else 2] for r in rects)
                hi = max(r[1 if a == 0 else 3] for r in rects)
                if hi - lo + SPACER_STEP > SPACER_SPAN:
                    side[w] = ""
                    continue
                new = []
                for r in rects:
                    i0, i1, k0, k1 = r
                    if a == 0 and far and i0 == lo:
                        r = (i0 - SPACER_STEP, i1, k0, k1)
                    elif a == 0 and not far and i1 == hi:
                        r = (i0, i1 + SPACER_STEP, k0, k1)
                    elif a == 2 and far and k0 == lo:
                        r = (i0, i1, k0 - SPACER_STEP, k1)
                    elif a == 2 and not far and k1 == hi:
                        r = (i0, i1, k0, k1 + SPACER_STEP)
                    new.append(r)
                rects = new
            if rects == sp.rects:
                break
            sp.rects = rects
            more = self.columns(sp)
            if not len(more):
                break
            now = self.slide_runs(more)
            for w in loose:             # a step that freed nothing: the other side, then none
                if now[w] >= free[w]:
                    side[w] = {"away": "toward", "toward": ""}.get(side[w], "")
            cols, free = more, now
            if (excess(free), len(cols)) < best[:2]:
                best = (excess(free), len(cols), list(sp.rects), cols)
        sp.rects = best[2]
        return best[3]


def spacers(g: Grid, rows: Sequence[dict]) -> list[Spacer]:
    """The printed spacers of a judged packing (:func:`metrics`' ``pieces`` rows, in the order
    the pieces go in, on its grid ``g``): each a block of the grid's empty cells, one run a
    column, that goes in at its place in the order, just before what it carries.

    *Supports*, where the pads were: each piece's pads within :data:`SPACER_JOIN` of one
    another make one spacer, its footprint their footprints grown :data:`SPACER_GROW` cells
    each way; in each column the empty run round the nearest pad's gap, from what is under
    it (another piece's top, the floor) to the piece's underside, but no more than
    :data:`SPACER_LIP` cells past the pad's own ends: beside a part it carries (or stands
    on) it rises (sinks) past it, a lip round it. A column's run another piece closes over
    it that goes in before it (it couldn't go in under it), or one under it that goes in
    after, isn't the spacer's. A spacer that slides more than :data:`SPACER_HOLD` cells a
    way grows (:meth:`_Spacers.grow`): its far side wraps round what it carries; two of one
    piece that meet become one.

    *Cradles*: a piece that still slides more than :data:`SPACER_HOLD` cells a way (but the
    base) gets one round its foot: its footprint grown :data:`SPACER_GROW` cells, in each
    column from what is under it up to the piece's underside (round it, :data:`SPACER_LIP`
    cells past its lowest), grown toward where it slides until something stops it.

    The grid's cells are the parts' conservative fill, so a spacer meets each part with up
    to a cell's gap: a piece it holds slides up to 3 mm (:data:`CELL`)."""
    keys = [r["key"] for r in rows]
    st = _Spacers(g, keys)
    for u, r in enumerate(rows):
        pads = [p for p in r["pads"] if p.get("fits") and p.get("base_mm") is not None
                and p["height_mm"] > 0]
        specs = []
        for group in _pad_clusters(pads, SPACER_JOIN):
            rects, seeds = [], []
            for p in group:
                x0, z0, x1, z1 = p["footprint"]
                rects.append((math.floor((x0 - g.lo[0]) / g.cell) - SPACER_GROW,
                              math.ceil((x1 - g.lo[0]) / g.cell) + SPACER_GROW,
                              math.floor((z0 - g.lo[2]) / g.cell) - SPACER_GROW,
                              math.ceil((z1 - g.lo[2]) / g.cell) + SPACER_GROW))
                b = (p["base_mm"] - g.lo[1]) / g.cell
                t = (p["base_mm"] + p["height_mm"] - g.lo[1]) / g.cell
                seeds.append(((x0 + x1) / 2, (z0 + z1) / 2, int((b + t) / 2),
                              math.floor(b) - SPACER_LIP, math.ceil(t) + SPACER_LIP))
            bears = sorted({p["under"] for p in group if p.get("under")}
                           | {p["on_part"] for p in group if p.get("on_part")})
            specs.append(_Spec(rects, st.pad_seed(seeds), u - 0.5, r["key"],
                               tuple(p["id"] for p in group), tuple(bears),
                               pad_seeds=seeds))
        for sp, cols in _grow_merged(st, specs):
            parts = _split(st, sp, cols)
            if len(parts) > 1:              # (apart: each grown again on its own)
                parts = [q for sub, c in _grow_merged(st, [s for s, _ in parts])
                         for q in _split(st, sub, c)]
            for sub, c in parts:
                if len(c) and not _steady(st, c):       # (it would tip: down to more)
                    deeper = _Spec(list(sub.rects), sub.seed.copy(), sub.rank, sub.under,
                                   sub.pads, sub.bears, sub.grow, sub.pad_seeds)
                    deeper.seed[..., 1] -= SPACER_REACH
                    more = _split(st, deeper, st.grow(deeper))
                    if more and _steady(st, more[0][1]):
                        sub, c = more[0]
                    else:
                        c = _trimmed(st, c)
                if len(c) and _stands(st, c):
                    st.commit(sub, c, f"spacer_{sub.pads[0]}" if sub.pads
                              else f"spacer_{r['key']}")
    for _ in range(6):                      # restraints for what still slides
        more = False
        owners = [p for p in range(len(keys)) if p or len(keys) == 1]   # (not the base)
        for p in owners + [-2 - n for n in range(len(st.made))]:
            key = st.name(p)
            mine = st.owner == p
            if not mine.any():
                continue
            free = st.slide(mine)
            for way in sorted(WAYS, key=lambda w: -free[w]):
                if free[way] <= SPACER_HOLD:
                    break
                for sp in _bar(st, p, mine, way):
                    cols = st.grow(sp)
                    if len(cols):
                        cols = _split(st, sp, cols)[0][1]
                    if len(cols) and not _steady(st, cols):
                        cols = _trimmed(st, cols)
                    if not len(cols) or not _steady(st, cols):
                        continue
                    if p < 0:               # (a spacer's: one print with it)
                        st.merge(-2 - p, cols)
                    else:
                        st.commit(sp, cols, f"spacer_{key}_{'pm'[WAYS[way][1] < 0]}"
                                            f"{'xyz'[WAYS[way][0]]}")
                    more = True
                    mine = st.owner == p
                    free = st.slide(mine)
                    break
        if not more:
            break
    return st.made


def _split(st: _Spacers, sp: _Spec, cols: np.ndarray) -> list[tuple[_Spec, np.ndarray]]:
    """A spacer's runs by its 6-connected parts (largest first), each with the pads whose
    seeds it holds and its own footprint (its bounds)."""
    from scipy import ndimage

    if not len(cols):
        return []
    lab, n = ndimage.label(st.mask(cols))
    if n <= 1:
        return [(sp, cols)]
    out = []
    for m in sorted(range(1, n + 1), key=lambda m: -int((lab == m).sum())):
        part = lab == m
        mine = _runs(part)
        held = [(pid, q) for pid, q in zip(sp.pads, sp.pad_seeds, strict=False)
                if _holds_seed(st, part, q)]
        seeds = [q for _, q in held] or sp.pad_seeds
        rect = (int(mine[:, 0].min()), int(mine[:, 0].max()) + 1,
                int(mine[:, 1].min()), int(mine[:, 1].max()) + 1)
        sub = _Spec([rect], st.pad_seed(seeds) if seeds else sp.seed, sp.rank, sp.under,
                    tuple(pid for pid, _ in held), sp.bears, sp.grow, seeds)
        out.append((sub, mine))
    return out


def _next_full(full: np.ndarray, a: int, sgn: int) -> np.ndarray:
    """For each cell, the index along axis ``a`` of the next full cell past it going ``sgn``
    (the grid's size, or -1, past its edge)."""
    n = full.shape[a]
    at = np.arange(n).reshape([n if k == a else 1 for k in range(3)])
    edge = [1 if k == a else s for k, s in enumerate(full.shape)]
    if sgn > 0:
        sm = np.flip(np.minimum.accumulate(np.flip(np.where(full, at, n), a), axis=a), a)
        return np.concatenate([np.take(sm, np.arange(1, n), axis=a), np.full(edge, n)], axis=a)
    pm = np.maximum.accumulate(np.where(full, at, -1), axis=a)
    return np.concatenate([np.full(edge, -1), np.take(pm, np.arange(n - 1), axis=a)], axis=a)


def _least_gap(mine: np.ndarray, others: np.ndarray, a: int, sgn: int
               ) -> tuple[int, np.ndarray, int] | None:
    """Where cells ``mine`` come nearest a full cell of ``others`` (or the grid's edge) going
    ``sgn`` along axis ``a``, not across one of its own: (the gap in cells, its cell, the
    index along ``a`` where it meets: the grid's size, or -1, past its edge); None when it
    meets itself first everywhere."""
    n = mine.shape[a]
    nxt = _next_full(others | mine, a, sgn)
    at = np.arange(n).reshape([n if k == a else 1 for k in range(3)])
    gap = np.abs(nxt - at) - 1
    inside = (nxt >= 0) & (nxt < n)
    own = np.take_along_axis(mine, np.clip(nxt, 0, n - 1), axis=a) & inside
    sel = mine & ~own & (gap > 0)
    if not sel.any():
        return None
    g = np.where(sel, gap, np.iinfo(np.int64).max)
    c = np.unravel_index(int(np.argmin(g)), g.shape)
    return int(g[c]), np.array(c), int(nxt[c])


def _least_gaps(mine: np.ndarray, others: np.ndarray, a: int, sgn: int, k: int = 4
                ) -> list[tuple[int, np.ndarray, int]]:
    """:func:`_least_gap`'s ``k`` least, each at least two footprints (:data:`SPACER_GROW`)
    across from those before it."""
    out = []
    left = mine.copy()
    other = 2 if a == 0 else 0
    for _ in range(k):
        got = _least_gap(left, others, a, sgn)
        if got is None:
            break
        out.append(got)
        c = got[1]
        sl = [slice(None)] * 3
        sl[other] = slice(max(int(c[other]) - 4 * SPACER_GROW, 0), int(c[other]) + 4 * SPACER_GROW)
        left[tuple(sl)] = False
    return out


def _bar(st: _Spacers, p: int, mine: np.ndarray, way: str) -> list[_Spec]:
    """A restraint for piece ``p`` sliding ``way`` (:func:`spacers`): a block in the gap beside
    its foot that way, where the gap is least, from the piece (its end's sides too, a lip
    round it) to what it would meet (or the box's wall), standing on what is under it; in
    after both."""
    a, sgn = WAYS[way]
    js = np.flatnonzero(mine.any(axis=(0, 2)))
    foot = mine.copy()
    foot[:, int(js[0]) + 4:, :] = False         # (its lowest four layers, else all of it)
    n_a = st.shape[a]
    other = 2 if a == 0 else 0
    out = []
    for cells in (foot, mine):
        for _, c, end in _least_gaps(cells, st.occ & ~mine, a, sgn):
            last = (end - sgn if 0 <= end < n_a else st.end[a] - 1 if sgn > 0 else 1)
            lo_a, hi_a = sorted((int(c[a]) - sgn * SPACER_LIP, last))
            if hi_a - lo_a + 1 > SPACER_SPAN:
                continue                # (too long a print: nothing near enough)
            w0, w1 = int(c[other]) - SPACER_GROW, int(c[other]) + SPACER_GROW + 1
            rect = (lo_a, hi_a + 1, w0, w1) if a == 0 else (w0, w1, lo_a, hi_a + 1)
            j = int(c[1])
            seed = np.empty((st.shape[0], st.shape[2], 3), np.int64)
            seed[...] = (j, j - SPACER_REACH, j + SPACER_LIP)
            out.append(_Spec([rect], seed, st.rank(p) if p < 0 else None, st.name(p), (),
                             (st.name(p),), "toward", why=f"restraint {way}"))
    return out          # (a spacer's restraint goes in with it: its rank)


def _grow_merged(st: _Spacers, specs: list[_Spec]) -> list[tuple[_Spec, np.ndarray]]:
    """Each spec grown (:meth:`_Spacers.grow`); two of one piece whose cells meet merged and
    grown again, until none meet."""
    while True:
        got = [(sp, st.grow(sp)) for sp in specs]
        pair = next(((a, b) for a, b in itertools.combinations(range(len(got)), 2)
                     if _meet(st, got[a][1], got[b][1])), None)
        if pair is None:
            return got
        a, b = pair
        sa, sb = specs[a], specs[b]
        seeds = sa.pad_seeds + sb.pad_seeds
        merged = _Spec(sa.rects + sb.rects, st.pad_seed(seeds), sa.rank, sa.under,
                       sa.pads + sb.pads, tuple(sorted(set(sa.bears) | set(sb.bears))),
                       sa.grow, seeds)
        specs = [s for n, s in enumerate(specs) if n not in pair] + [merged]


def _runs(mask: np.ndarray) -> np.ndarray:
    """A mask's cells as one run a column (i, k, j0, j1): its lowest to past its highest."""
    ii, kk = np.nonzero(mask.any(axis=1))
    out = []
    for i, k in zip(ii.tolist(), kk.tolist(), strict=True):
        js = np.flatnonzero(mask[i, :, k])
        out.append((i, k, int(js[0]), int(js[-1]) + 1))
    return np.asarray(out, np.int64).reshape(-1, 4)


def _holds_seed(st: _Spacers, mask: np.ndarray, q: tuple) -> bool:
    x, z, j = q[0], q[1], int(q[2])
    i, k = int((x - st.g.lo[0]) / st.g.cell), int((z - st.g.lo[2]) / st.g.cell)
    nx, ny, nz = st.shape
    return 0 <= i < nx and 0 <= k < nz and 0 <= j < ny and bool(mask[i, j, k])


def _steady(st: _Spacers, cols: np.ndarray) -> bool:
    """Whether a spacer stands by itself before what it carries is on it (:func:`support`):
    its cells' centre over the columns where it stands on something (or the floor)."""
    on = [(i, k) for i, k, j0, _ in cols.tolist() if j0 <= 1 or st.occ[i, j0 - 1, k]]
    if not on:
        return False
    n = (cols[:, 3] - cols[:, 2]).astype(float)
    com = ((cols[:, 0] + 0.5) * n).sum() / n.sum(), ((cols[:, 1] + 0.5) * n).sum() / n.sum()
    return over_hull(np.asarray(on, np.int64), com)


def _trimmed(st: _Spacers, cols: np.ndarray) -> np.ndarray:
    """A spacer that would tip with its overhang off: the columns it doesn't stand on,
    farthest from those it does first, until it stands (:func:`_steady`), but as it was if
    it can't, or if what is left falls apart."""
    from scipy import ndimage

    on = np.array([j0 <= 1 or st.occ[i, j0 - 1, k] for i, k, j0, _ in cols.tolist()])
    if not on.any():
        return cols
    mid = cols[on][:, :2].mean(axis=0)
    far = np.argsort(-np.hypot(*(cols[:, :2] - mid).T))
    keep = np.ones(len(cols), bool)
    for n in far:
        if on[n]:
            continue
        keep[n] = False
        if _steady(st, cols[keep]):
            if ndimage.label(st.mask(cols[keep]))[1] == 1:
                return cols[keep]
            break
    return cols


def _meet(st: _Spacers, a: np.ndarray, b: np.ndarray) -> bool:
    """Whether two spacers' runs share a face (or a cell)."""
    from scipy import ndimage

    if not len(a) or not len(b):
        return False
    lo = np.minimum(a[:, [0, 2, 1]].min(axis=0), b[:, [0, 2, 1]].min(axis=0)) - 1
    hi = np.maximum(a[:, [0, 3, 1]].max(axis=0), b[:, [0, 3, 1]].max(axis=0)) + 2
    for x, y in ((a, b), (b, a)):           # (their bounds, a cell apart at most: else no)
        if (x[:, [0, 2, 1]].min(axis=0) > y[:, [0, 3, 1]].max(axis=0) + 1).any():
            return False
    lo = np.maximum(lo, 0)

    def local(c: np.ndarray) -> np.ndarray:
        m = np.zeros(tuple(int(v) for v in hi - lo), bool)
        ys = np.arange(lo[1], hi[1])[None, :]
        m[c[:, 0] - lo[0], :, c[:, 1] - lo[2]] = (ys >= c[:, 2:3]) & (ys < c[:, 3:4])
        return m

    return bool((ndimage.binary_dilation(local(a)) & local(b)).any())


def _stands(st: _Spacers, cols: np.ndarray) -> bool:
    """Whether a spacer's runs stand on something: a full cell under one, or the floor."""
    return any(j0 <= 1 or st.occ[i, j0 - 1, k] for i, k, j0, _ in cols.tolist())


def pad_id(order: int, n: int) -> str:
    """Pad ``n`` (from 0) of the piece that goes in ``order``-th (from 1): ``4a``, ``4b``
    .. ``4z``, ``4aa`` .."""
    letters = ""
    n += 1
    while n:
        n, r = divmod(n - 1, 26)
        letters = "abcdefghijklmnopqrstuvwxyz"[r] + letters
    return f"{order}{letters}"


def metrics(pieces: Sequence[Packed], inside: tuple[np.ndarray, np.ndarray] | None = None,
            cell: float = CELL, fine: float = FINE, bands: int = 12,
            grid: Grid | None = None) -> dict:
    """What a packing is worth, from the pieces where it put them (in the order they go
    in): the box (``inside``, default :func:`box_of`; its outside with :data:`WALL`), the
    true fill (the parts' own volumes over the box's inside), its cells' fill, the air per
    layer (``bands`` of the inside's height, bottom up), each piece's mass, centre of mass
    height, how it rests (:func:`support`: on what, directly or on pads, or not: each of
    its columns' gap down to another piece's nearest cell or the floor, a cell of foam off
    it), its pads measured on the real meshes (:func:`measure_pads`; ``pads``: their count,
    volume, the tallest and the cut list, each named by its piece's number and a letter),
    how far it slides before it meets a neighbour or the wall (:func:`free_travel`:
    ``free_mm`` each way, ``slide_mm`` the most along the floor; ``restraint``: the pieces
    that slide more than :data:`CLEAR`, the foam's width, and the most any does), the
    real-shape overlaps (pieces that share cells, then the volume they share at ``fine``
    mm), the order's faults (a piece over another that goes in after it), the mass centre's
    height."""
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
    rays: Rays | None = None
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
        pads = []
        if rest.pads:
            rays = rays or Rays(pieces)
            pads = measure_pads(pieces, i, rest.pads, float(lo[1]), rays)
            for n, p in enumerate(pads):
                p["id"] = pad_id(i + 1, n)
                p["piece"] = pc.key
                p["centre_mm"] = [round((p["footprint"][0] + p["footprint"][2]) / 2
                                        - float(lo[0]), 1),
                                  round((p["footprint"][1] + p["footprint"][3]) / 2
                                        - float(lo[2]), 1)]
        free = free_travel(g, i, owner)
        rows.append({"key": pc.key, "label": pc.label, "order": i + 1,
                     "mass_kg": round(pc.mass, 3), "volume_l": round(pc.volume / 1e6, 3),
                     "com_height_mm": round(float(pc.com[1] - lo[1]), 1),
                     "supported": rest.ok, "rests": rest.how, "on": list(rest.on),
                     "pads": pads, "cells": int(g.occ[i].sum()),
                     "free_mm": {k: round(v[0], 1) for k, v in free.items()},
                     "meets": {k: "wall" if v[1] < 0 else names[v[1]]
                               for k, v in free.items()},
                     "slide_mm": max((v[0] for k, v in free.items() if k != "+y"),
                                     default=0.0)})
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
    cut = [p for r in rows for p in r["pads"]]
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
        "pads": {"count": len(cut), "volume_cm3": round(sum(
                     p["size_mm"][0] * p["size_mm"][1] * p["height_mm"] for p in cut) / 1e3, 1),
                 "tallest_mm": max((p["height_mm"] for p in cut), default=0.0),
                 "cut_list": cut},
        "restraint": {"held_mm": CLEAR,
                      "max_slide_mm": max((r["slide_mm"] for r in rows), default=0.0),
                      "loose": [r["key"] for r in rows if r["slide_mm"] > CLEAR + 1e-6]},
        "overlaps": [o for o in overlaps if o["volume_mm3"] > 0],
        "touching": [o for o in overlaps if o["volume_mm3"] <= 0],
        "order_faults": faults,
        "pieces": rows,
        "cell_mm": cell,
    }

