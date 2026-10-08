"""The assembly guide's pictures: a small deterministic software renderer.

Flat, banded (toon) shading of the triangles in an orthographic view, resolved in a
z-buffer written with numpy alone, ink lines found in image space (where the part, the
depth or the face normal jumps), all drawn ``SS`` times larger and box-filtered down
(anti-aliasing). No GPU, no display, nothing beyond numpy and Pillow, and the same bytes
on every run on one machine: every reduction is an integer ``maximum.at``.

A picture is a list of :class:`Item` (a body's triangles in robot coordinates, its style)
and a :class:`View`; :func:`render` gives the image and, for each marked item, where its
visible pixels are (:class:`Mark`), so the label bubbles are drawn afterwards
(:func:`bubbles`), clear of each other, once the labels are known. :func:`render_jobs` is
a worker's share of a guide (:func:`spiderpig.workers.submit`: it imports this module,
numpy and Pillow, not the engine).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import numpy as np
from PIL import Image, ImageDraw, ImageFont

SS = 2                      # supersampling factor
BG = (255, 255, 255)
INK = (24, 24, 28)
CONTEXT_FILL = (226, 226, 230)
HIGHLIGHT = (245, 140, 40)
PLACED = (250, 196, 140)    # a sub-assembly put on in this step: a lighter highlight
BOUGHT_FILL = (105, 165, 225)   # a bought part the step adds: blue, the made ones orange

VIEWS = {
    # the camera of each kind of step: the bench views keep a side's stack upright (its
    # z axis up the page), the robot's look from its left front
    "bench_L": {"eye": (0.75, 0.55, 0.85), "up": (0, 0, 1), "light": (0.3, 0.5, 1.0)},
    "bench_L-": {"eye": (0.75, 0.55, -0.85), "up": (0, 0, -1), "light": (0.3, 0.5, -1.0)},
    "bench_R": {"eye": (0.75, 0.55, -0.85), "up": (0, 0, -1), "light": (0.3, 0.5, -1.0)},
    "bench_R-": {"eye": (0.75, 0.55, 0.85), "up": (0, 0, 1), "light": (0.3, 0.5, 1.0)},
    "bench_": {"eye": (0.75, 0.55, 0.85), "up": (0, 0, 1), "light": (0.3, 0.5, 1.0)},
    "bench_-": {"eye": (0.75, 0.55, -0.85), "up": (0, 0, -1), "light": (0.3, 0.5, -1.0)},
    "robot": {"eye": (1.0, 0.8, -1.2), "up": (0, 1, 0), "light": (0.4, 1.0, -0.6)},
    "robot-": {"eye": (1.0, 0.8, 1.2), "up": (0, 1, 0), "light": (0.4, 1.0, 0.6)},
    "robot^": {"eye": (0.7, 1.4, -0.5), "up": (0, 1, 0), "light": (0.4, 1.0, -0.6)},
    "part": {"eye": (0.6, 0.8, 1.0), "up": (0, 1, 0), "light": (0.3, 1.0, 0.8)},
}


@dataclass
class Item:
    """One body's triangles (``pos`` (n, 3) float, ``tri`` (m, 3) int) and its style;
    ``mark``: say where it shows (:class:`Mark`)."""

    name: str
    pos: np.ndarray
    tri: np.ndarray
    fill: tuple[int, int, int] = CONTEXT_FILL
    ink: tuple[int, int, int] = INK
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)    # an exploded part's shift
    mark: bool = False


@dataclass
class View:
    eye: tuple[float, float, float] = (1.0, 0.9, -1.4)   # from the target toward the eye
    up: tuple[float, float, float] = (0.0, 1.0, 0.0)
    size: tuple[int, int] = (1200, 900)
    margin: float = 0.06
    light: tuple[float, float, float] = (0.4, 1.0, -0.6)
    arrows: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list)


@dataclass(frozen=True)
class Mark:
    """Where a marked item shows in the picture: the centre of its visible pixels, and
    how many (picture pixels)."""

    x: float
    y: float
    n: int


def _basis(view: View) -> np.ndarray:
    d = np.asarray(view.eye, float)
    d /= np.linalg.norm(d)
    right = np.cross(np.asarray(view.up, float), d)
    right /= np.linalg.norm(right)
    up = np.cross(d, right)
    return np.stack([right, up, d])     # rows: screen x, screen y, depth (toward the eye)


def _raster(px, py, pz, tris, w, h, chunk=1 << 21):
    """Every triangle's covered pixel centres, scanline by scanline (vectorized over
    triangles, then rows, then pixels): a packed (depth, triangle) key per pixel, the
    nearest kept by ``maximum.at`` (an integer reduction: order-independent)."""
    key = np.full(w * h, -1, np.int64)
    zlo, zhi = float(pz.min()), float(pz.max())
    zq = (pz - zlo) / max(zhi - zlo, 1e-9) * ((1 << 30) - 1)    # depth, 30 bits
    X, Y, Z = px[tris], py[tris], zq[tris]                       # (m, 3) each
    # the depth plane z = A x + B y + C of each triangle
    ux, uy, uz = X[:, 1] - X[:, 0], Y[:, 1] - Y[:, 0], Z[:, 1] - Z[:, 0]
    vx, vy, vz = X[:, 2] - X[:, 0], Y[:, 2] - Y[:, 0], Z[:, 2] - Z[:, 0]
    det = ux * vy - uy * vx
    ok = np.abs(det) > 1e-12
    det = np.where(ok, det, 1.0)
    A = (uz * vy - uy * vz) / det
    B = (ux * vz - uz * vx) / det
    C = Z[:, 0] - A * X[:, 0] - B * Y[:, 0]
    r0 = np.maximum(np.ceil(Y.min(1) - 0.5), 0).astype(np.int64)
    r1 = np.minimum(np.ceil(Y.max(1) - 0.5) - 1, h - 1).astype(np.int64)
    nrow = np.where(ok, np.maximum(r1 - r0 + 1, 0), 0)
    tri_ids = np.nonzero(nrow)[0]
    # chunks of triangles whose rows fit the budget
    cum = np.cumsum(nrow[tri_ids])
    starts = np.searchsorted(cum, np.arange(0, cum[-1] if len(cum) else 0, chunk // 8))
    bounds = [*starts.tolist(), len(tri_ids)]
    for s, e in itertools.pairwise(bounds):
        t = tri_ids[s:e]
        if not len(t):
            continue
        nr = nrow[t]
        tt = np.repeat(t, nr)                                   # one entry per (tri, row)
        row = r0[tt] + (np.arange(len(tt)) - np.repeat(np.cumsum(nr) - nr, nr))
        yc = row + 0.5
        xl = np.full(len(tt), np.inf)
        xr = np.full(len(tt), -np.inf)
        for i, j in ((0, 1), (1, 2), (2, 0)):
            ya, yb, xa, xb = Y[tt, i], Y[tt, j], X[tt, i], X[tt, j]
            lo, hi = np.minimum(ya, yb), np.maximum(ya, yb)
            cross = (yc >= lo) & (yc < hi)
            x = xa + (yc - ya) * (xb - xa) / np.where(hi > lo, yb - ya, 1.0)
            xl = np.where(cross, np.minimum(xl, x), xl)
            xr = np.where(cross, np.maximum(xr, x), xr)
        c0 = np.maximum(np.ceil(xl - 0.5), 0)
        c1 = np.minimum(np.ceil(xr - 0.5) - 1, w - 1)
        good = np.isfinite(c0) & np.isfinite(c1) & (c1 >= c0)
        tt, row, c0, c1 = tt[good], row[good], c0[good].astype(np.int64), c1[good].astype(
            np.int64)
        npx = c1 - c0 + 1
        for a_, b_ in _slices(npx, chunk):
            n_ = npx[a_:b_]
            ft = np.repeat(tt[a_:b_], n_)
            fr = np.repeat(row[a_:b_], n_)
            fc = np.repeat(c0[a_:b_], n_) + (np.arange(int(n_.sum()))
                                             - np.repeat(np.cumsum(n_) - n_, n_))
            z = A[ft] * (fc + 0.5) + B[ft] * (fr + 0.5) + C[ft]
            k_ = (np.clip(z, 0, (1 << 30) - 1).astype(np.int64) << 32) | ft
            np.maximum.at(key, fr * w + fc, k_)
    return key


def _slices(counts, budget):
    """Consecutive index ranges of ``counts`` summing to about ``budget`` each."""
    cum = np.cumsum(counts)
    out, s = [], 0
    while s < len(counts):
        base = cum[s - 1] if s else 0
        e = int(np.searchsorted(cum, base + budget, side="right"))
        e = max(e, s + 1)
        out.append((s, e))
        s = e
    return out


def _edges(owner, tri, depth, fn, w, h, crease_cos=0.80):
    """Ink where the owning item changes or the depth jumps (``line``), and a lighter one
    at a crease, where neighbouring triangles of one item turn by more than
    ``acos(crease_cos)`` (``soft``)."""
    o = owner.reshape(h, w)
    tr = tri.reshape(h, w)
    z = depth.reshape(h, w)
    line = np.zeros((h, w), bool)
    soft = np.zeros((h, w), bool)
    step = 0.004 * max(float(np.ptp(z[o >= 0])) if (o >= 0).any() else 1.0, 1.0)
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        sl_b = (slice(dy, h), slice(max(0, dx), w + min(0, dx)))
        sl_a = (slice(0, h - dy), slice(max(0, -dx), w - max(0, dx)))
        oa, ob = o[sl_a], o[sl_b]
        diff = oa != ob
        both = (oa >= 0) & ~diff
        jump = both & (np.abs(z[sl_a] - z[sl_b]) > 8 * step)
        hard = diff | jump
        line[sl_a] |= hard
        line[sl_b] |= hard
        ta, tb = tr[sl_a], tr[sl_b]
        cand = both & ~jump & (ta != tb)
        r, c = np.nonzero(cand)
        if len(r):
            cos = (fn[ta[r, c]] * fn[tb[r, c]]).sum(-1)
            sa = soft[sl_a]
            sa[r[cos < crease_cos], c[cos < crease_cos]] = True
    return line, soft


def _dilate(m, r):
    out = m.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r + r:
                continue
            out |= np.roll(np.roll(m, dy, 0), dx, 1)
    return out


def render(items: list[Item], view: View, ss: int = SS
           ) -> tuple[Image.Image, dict[str, Mark]]:
    """The picture of ``items`` from ``view`` (an RGB image of ``view.size``), and where
    each marked item shows."""
    basis = _basis(view)
    W, H = view.size[0] * ss, view.size[1] * ss
    pos = [(it.pos + np.asarray(it.offset)) @ basis.T for it in items]
    allp = np.concatenate(pos)
    lo, hi = allp[:, :2].min(0), allp[:, :2].max(0)
    for a, b in view.arrows:            # the arrows' ends are framed too
        for q in (a, b):
            s = basis @ np.asarray(q, float)
            lo, hi = np.minimum(lo, s[:2]), np.maximum(hi, s[:2])
    usable = np.array([W, H]) * (1 - 2 * view.margin)
    scale = float(min(usable / np.maximum(hi - lo, 1e-6)))
    centre = (lo + hi) / 2

    def to_px(p):
        x = (p[..., 0] - centre[0]) * scale + W / 2
        y = H / 2 - (p[..., 1] - centre[1]) * scale
        return x, y

    tris, owner_of_tri, base = [], [], 0
    for i, (it, p) in enumerate(zip(items, pos, strict=True)):
        tris.append(it.tri.reshape(-1, 3).astype(np.int64) + base)
        owner_of_tri.append(np.full(len(tris[-1]), i, np.int64))
        base += len(p)
    v = np.concatenate(pos)
    t = np.concatenate(tris)
    owner_of_tri = np.concatenate(owner_of_tri)
    px, py = to_px(v)
    key = _raster(px, py, v[:, 2], t, W, H)
    hit = key >= 0
    tri_id = np.where(hit, key & 0xFFFFFFFF, 0)
    depth = np.where(hit, (key >> 32).astype(np.float32), np.float32(0))
    owner = np.where(hit, owner_of_tri[tri_id], -1)
    # flat normals in view space, turned toward the eye; one tone per triangle
    e1 = v[t[:, 1]] - v[t[:, 0]]
    e2 = v[t[:, 2]] - v[t[:, 0]]
    fn = np.cross(e1, e2)
    fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)
    fn *= np.where(fn[:, 2:3] < 0, -1.0, 1.0)
    light = basis @ np.asarray(view.light, float)
    light /= np.linalg.norm(light)
    shade = 0.72 + 0.28 * np.clip(fn @ light, 0, 1)      # soft, poster-like
    shade = np.round(shade * 10) / 10                      # banded: flat, toon-like tones
    fills = np.array([it.fill for it in items], np.float32)
    inks = np.array([it.ink for it in items] + [INK], np.float32)
    tone = np.minimum(fills[owner_of_tri] * shade[:, None].astype(np.float32), 255)
    img = np.where(hit[:, None], tone[tri_id], np.asarray(BG, np.float32))
    line, soft = _edges(owner, tri_id, depth, fn, W, H)
    lw = max(1, ss // 2 + 1)
    line = _dilate(line, lw - 1) if lw > 1 else line
    img = img.reshape(H, W, 3)
    # a line takes the ink of the part under it, or a neighbour's over the background
    o2 = owner.reshape(H, W)
    r, c = np.nonzero(line)
    ol = o2[r, c]
    for dy, dx in ((0, 1), (1, 0), (0, -1), (-1, 0)):
        nb = o2[np.clip(r + dy, 0, H - 1), np.clip(c + dx, 0, W - 1)]
        ol = np.where(ol < 0, nb, ol)
    img[r, c] = inks[np.where(ol < 0, len(items), ol)]
    r, c = np.nonzero(soft & ~line)
    img[r, c] = inks[o2[r, c]] * 0.55 + img[r, c] * 0.45
    img = img.reshape(view.size[1], ss, view.size[0], ss, 3).mean((1, 3))
    out = Image.fromarray(np.clip(np.round(img), 0, 255).astype(np.uint8), "RGB")
    if view.arrows:
        d = ImageDraw.Draw(out)
        for a, b in view.arrows:
            (ax, ay), (bx, by) = [(float(x) / ss, float(y) / ss)
                                  for x, y in (to_px(basis @ np.asarray(q, float))
                                               for q in (a, b))]
            _arrow(d, (ax, ay), (bx, by), color=(90, 90, 100))
    marks: dict[str, Mark] = {}
    marked = [i for i, it in enumerate(items) if it.mark]
    if marked:
        rows, cols = np.nonzero(o2 >= 0)
        who = o2[rows, cols]
        for i in marked:
            sel = who == i
            n = int(sel.sum())
            if n:
                # the visible pixel nearest the centre of them all: on the part itself
                ry, rx = rows[sel], cols[sel]
                cy, cx = ry.mean(), rx.mean()
                k = int(np.argmin((ry - cy) ** 2 + (rx - cx) ** 2))
                marks[items[i].name] = Mark(round(float(rx[k]) / ss, 1),
                                            round(float(ry[k]) / ss, 1), n // (ss * ss))
    return out, marks


def _arrow(d: ImageDraw.ImageDraw, a, b, color=INK):
    """A dashed shaft from ``a`` to ``b`` and a filled head at ``b``."""
    ax, ay = a
    bx, by = b
    L = max(((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5, 1e-6)
    ux, uy = (bx - ax) / L, (by - ay) / L
    dash = 9.0
    s = 0.0
    while s < L - 14:
        e = min(s + dash, L - 14)
        d.line((ax + ux * s, ay + uy * s, ax + ux * e, ay + uy * e), fill=color, width=3)
        s += 2 * dash
    hx, hy = bx - ux * 16, by - uy * 16
    d.polygon([(bx, by), (hx - uy * 8, hy + ux * 8), (hx + uy * 8, hy - ux * 8)], fill=color)


def font(size: int, bold: bool = True):
    """DejaVu Sans (Pillow's own font when it isn't installed: still deterministic)."""
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for path in (name, f"/usr/share/fonts/truetype/dejavu/{name}"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default(size)


# ---------------------------------------------------------------------------
# label bubbles
# ---------------------------------------------------------------------------

R_BUBBLE = 21


def bubbles(img: Image.Image, marks: list[tuple[str, Mark]]
            ) -> tuple[Image.Image, dict[str, tuple[float, float]]]:
    """``img`` with a bubble per ``(label, mark)`` on a short leader to its part, and where
    each went: each put where it overlaps no other bubble and covers least of the drawing
    (tried round the part at three distances), the biggest parts' first; one that finds no
    room is left out. A copy; deterministic."""
    out = img.copy()
    d = ImageDraw.Draw(out)
    w, h = out.size
    ink = np.asarray(img.convert("L")) < 245          # what a bubble shouldn't hide
    taken: list[tuple[float, float]] = []
    placed: dict[str, tuple[float, float]] = {}
    anchors = [(m.x, m.y) for _, m in marks]
    f = font(17)
    r = R_BUBBLE
    for label, m in sorted(marks, key=lambda lm: (-lm[1].n, lm[0])):
        best = None
        for dist, ang in itertools.product((48, 80, 120), range(-45, 315, 30)):
            bx = m.x + dist * math.cos(math.radians(ang))
            by = m.y - dist * math.sin(math.radians(ang))
            if not (r + 2 <= bx <= w - r - 2 and r + 2 <= by <= h - r - 2):
                continue
            if any((bx - x) ** 2 + (by - y) ** 2 < (2 * r + 6) ** 2 for x, y in taken):
                continue
            if any((bx - x) ** 2 + (by - y) ** 2 < (r + 4) ** 2 for x, y in anchors):
                continue
            x0, x1 = int(bx - r), int(bx + r)
            y0, y1 = int(by - r), int(by + r)
            cover = float(ink[y0:y1, x0:x1].mean())
            score = cover + dist / 400.0
            if best is None or score < best[0] - 1e-9:
                best = (score, bx, by)
        if best is None:
            continue
        _, bx, by = best
        taken.append((bx, by))
        placed[label] = (round(bx, 1), round(by, 1))
        d.line((m.x, m.y, bx, by), fill=INK, width=2)
        d.ellipse((m.x - 3, m.y - 3, m.x + 3, m.y + 3), fill=INK)
        d.ellipse((bx - r, by - r, bx + r, by + r), fill=(255, 255, 255), outline=INK,
                  width=2)
        d.text((bx, by), label, fill=INK, font=f, anchor="mm")
    return out, placed


# ---------------------------------------------------------------------------
# a worker's share
# ---------------------------------------------------------------------------


def _clipped(pos: np.ndarray, tri: np.ndarray, z: tuple[float, float] | None):
    """The triangles of a body whose centre's z is in ``z`` (a piece of it)."""
    if z is None:
        return pos, tri
    t = tri.reshape(-1, 3)
    cz = pos[t, 2].mean(1)
    return pos, t[(cz >= z[0]) & (cz <= z[1])]


def items_of(mesh, rows: list) -> list[Item]:
    """Items from rows ``(piece id, fill, offset, mark, clip)`` over ``mesh`` (body name ->
    ``(pos, tri)``)."""
    out = []
    for pid, fill, off, mark, clip in rows:
        pos, tri = mesh[pid.split("#", 1)[0]]
        pos, tri = _clipped(pos, tri, None if clip is None else tuple(clip))
        out.append(Item(pid, pos, tri, fill=tuple(fill), offset=tuple(off), mark=mark))
    return out


def choose(items: list[Item], options: list[str], want: set[str]) -> str:
    """The camera among ``options`` (:data:`VIEWS`) that shows most of ``want`` (first on
    a tie): a small render of each."""
    if len(options) == 1:
        return options[0]
    scores = []
    for o in options:
        small = [Item(it.name, it.pos, it.tri, offset=it.offset, mark=it.name in want)
                 for it in items]
        _, marks = render(small, View(**VIEWS[o], size=(200, 150), margin=0.05), ss=1)
        scores.append(sum(m.n for m in marks.values()))
    return options[scores.index(max(scores))]


def render_jobs(mesh_path: str, jobs: list[dict]) -> list[dict]:
    """Draw ``jobs`` (each: ``out`` its PNG, ``rows`` its items as :func:`items_of`
    takes, ``views`` the cameras to choose among, ``size``, ``explode`` the lift along
    the camera's up of the marked items with an arrow, ``margin``): what each chose and
    where its marked items show."""
    data = np.load(mesh_path)
    names = sorted({k.rsplit("|", 1)[0] for k in data.files})
    mesh = {n: (data[n + "|p"], data[n + "|i"]) for n in names}
    out = []
    for job in jobs:
        items = items_of(mesh, job["rows"])
        want = {it.name for it in items if it.mark}
        cam = choose(items, job["views"], want)
        v = VIEWS[cam]
        arrows = []
        lift = float(job.get("explode") or 0.0)
        if lift and any(not it.mark for it in items) and want:
            up = np.asarray(v["up"], float)
            for it in items:
                if it.mark:
                    it.offset = tuple(np.asarray(it.offset) + up * lift)
            moved = np.concatenate([it.pos for it in items if it.mark])
            c = moved.mean(0)
            arrows = [(c + up * lift * 0.9, c + up * 1.0)]
        img, marks = render(items, View(**v, size=tuple(job["size"]),
                                         margin=job.get("margin", 0.06), arrows=arrows))
        img.save(job["out"], compress_level=6)
        out.append({"out": job["out"], "view": cam,
                    "marks": {k: [m.x, m.y, m.n] for k, m in marks.items()}})
    return out
