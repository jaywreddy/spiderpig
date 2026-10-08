"""A small deterministic software renderer for the assembly guide's step pictures.

Flat-shaded triangles in an orthographic view, resolved in a z-buffer written with numpy
alone, then ink lines found in image space (where the part, the depth or the face normal
jumps), the whole thing drawn ``SS`` times larger and box-filtered down (anti-aliasing).
No GPU, no display, no new dependency (numpy and Pillow are in the lock already), and the
same bytes on every run on one machine: every reduction is an integer ``maximum.at``.

A picture is a list of :class:`Item` (a body's triangles in robot coordinates, its style)
and a :class:`View` (the direction the eye looks from, the up vector, what to frame).
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

SS = 2                      # supersampling factor
BG = (255, 255, 255)
INK = (24, 24, 28)
CONTEXT_FILL = (226, 226, 230)
CONTEXT_INK = (150, 150, 158)
HIGHLIGHT = (245, 140, 40)


@dataclass
class Item:
    """One body's triangles (``pos`` (n, 3) float, ``tri`` (m, 3) int) and its style."""

    name: str
    pos: np.ndarray
    tri: np.ndarray
    fill: tuple[int, int, int] = CONTEXT_FILL
    ink: tuple[int, int, int] = INK
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)    # an exploded part's shift
    label: str | None = None


@dataclass
class View:
    eye: tuple[float, float, float] = (1.0, 0.9, -1.4)   # from the target toward the eye
    up: tuple[float, float, float] = (0.0, 1.0, 0.0)
    size: tuple[int, int] = (1200, 900)
    margin: float = 0.06
    light: tuple[float, float, float] = (0.4, 1.0, -0.6)
    frame: list[str] | None = None     # the items to frame (None: every item)
    arrows: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list)


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


def render(items: list[Item], view: View, ss: int = SS) -> Image.Image:
    """The picture of ``items`` from ``view`` (an RGB image of ``view.size``)."""
    basis = _basis(view)
    W, H = view.size[0] * ss, view.size[1] * ss
    pos = [(it.pos + np.asarray(it.offset)) @ basis.T for it in items]
    frame = [p for it, p in zip(items, pos, strict=True)
             if view.frame is None or it.name in view.frame] or pos
    allp = np.concatenate(frame)
    lo, hi = allp[:, :2].min(0), allp[:, :2].max(0)
    # the arrows' ends are framed too
    for a, b in view.arrows:
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

    verts, tris, owner_of_tri = [], [], []
    base = 0
    for i, (it, p) in enumerate(zip(items, pos, strict=True)):
        verts.append(p)
        tris.append(it.tri.reshape(-1, 3).astype(np.int64) + base)
        owner_of_tri.append(np.full(len(tris[-1]), i, np.int64))
        base += len(p)
    v = np.concatenate(verts)
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
    labels = [(it, p) for it, p in zip(items, pos, strict=True) if it.label]
    if labels:
        # a bubble per label, up and to the right of its part on a short leader
        d = ImageDraw.Draw(out)
        font = _font(20)
        r = 19
        for it, p in labels:
            x, y = (float(q) / ss for q in to_px(p.mean(0)))
            bx, by = x + 34, y - 34
            d.line((x, y, bx, by), fill=INK, width=2)
            d.ellipse((x - 3, y - 3, x + 3, y + 3), fill=INK)
            d.ellipse((bx - r, by - r, bx + r, by + r), fill=(255, 255, 255), outline=INK,
                      width=2)
            d.text((bx, by), str(it.label), fill=INK, font=font, anchor="mm")
    return out


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


def _font(size: int):
    for name in ("DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default(size)


def visible(items: list[Item], view: View, names: set[str], size=(240, 180)) -> int:
    """How many pixels of a small render the items named ``names`` show (a camera
    chooser's score: the parts a step adds should be seen)."""
    basis = _basis(view)
    W, H = size
    pos = [(it.pos + np.asarray(it.offset)) @ basis.T for it in items]
    allp = np.concatenate(pos)
    lo, hi = allp[:, :2].min(0), allp[:, :2].max(0)
    scale = float(min(np.array([W, H]) * 0.9 / np.maximum(hi - lo, 1e-6)))
    c = (lo + hi) / 2
    tris, owner, base = [], [], 0
    for i, (it, p) in enumerate(zip(items, pos, strict=True)):
        tris.append(it.tri.reshape(-1, 3).astype(np.int64) + base)
        owner.append(np.full(len(tris[-1]), i, np.int64))
        base += len(p)
    v = np.concatenate(pos)
    key = _raster((v[:, 0] - c[0]) * scale + W / 2, H / 2 - (v[:, 1] - c[1]) * scale,
                  v[:, 2], np.concatenate(tris), W, H)
    hit = key >= 0
    who = np.concatenate(owner)[(key[hit] & 0xFFFFFFFF)]
    wanted = np.array([it.name in names for it in items])
    return int(wanted[who].sum())


def render_batch(mesh: dict, jobs: list[tuple], folder: str) -> list[str]:
    """Draw ``jobs`` (``(number, spec)``) into ``folder``: a worker's share
    (:func:`workers.submit`; it imports numpy and Pillow, not the engine)."""
    out = []
    for number, (rows, view) in jobs:
        items = [Item(n, *mesh[n], fill=fill, offset=off, label=lab)
                 for n, fill, off, lab in rows]
        path = Path(folder) / f"step_{number:03d}.png"
        render(items, view).save(path, compress_level=6)
        out.append(str(path))
    return out
