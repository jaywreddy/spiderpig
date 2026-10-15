"""Distances in the side's plane between segments, rectangles and convex polygons: what the
robot-level parts (:mod:`construction.rotary.turret`, :mod:`construction.tools`) check
their keep-outs with. Rectangles are axis-aligned ``(x0, y0, x1, y1)``."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

Rect = tuple[float, float, float, float]


def point_seg(p, a, b) -> float:
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    d = b - a
    L = float(d @ d)
    t = 0.0 if L < 1e-18 else min(1.0, max(0.0, float((p - a) @ d) / L))
    return float(np.linalg.norm(p - (a + t * d)))


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def segs_cross(a, b, c, d) -> bool:
    """Whether segments ``ab`` and ``cd`` intersect (touching counts)."""
    d1, d2 = _cross(c, d, a), _cross(c, d, b)
    d3, d4 = _cross(a, b, c), _cross(a, b, d)
    if ((d1 > 0) != (d2 > 0) or d1 == 0 or d2 == 0) and \
            ((d3 > 0) != (d4 > 0) or d3 == 0 or d4 == 0):
        return (min(a[0], b[0]) <= max(c[0], d[0]) + 1e-12
                and min(c[0], d[0]) <= max(a[0], b[0]) + 1e-12
                and min(a[1], b[1]) <= max(c[1], d[1]) + 1e-12
                and min(c[1], d[1]) <= max(a[1], b[1]) + 1e-12)
    return False


def seg_seg(a, b, c, d) -> float:
    if segs_cross(a, b, c, d):
        return 0.0
    return min(point_seg(a, c, d), point_seg(b, c, d), point_seg(c, a, b), point_seg(d, a, b))


def rect_corners(r: Rect) -> list[tuple[float, float]]:
    x0, y0, x1, y1 = r
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def inside_rect(p, r: Rect) -> bool:
    return r[0] <= p[0] <= r[2] and r[1] <= p[1] <= r[3]


def seg_rect(a, b, r: Rect) -> float:
    """The least distance between segment ``ab`` and rectangle ``r`` (0: they meet)."""
    if inside_rect(a, r) or inside_rect(b, r):
        return 0.0
    c = rect_corners(r)
    return min(seg_seg(a, b, c[i], c[(i + 1) % 4]) for i in range(4))


def point_rect(p, r: Rect) -> float:
    dx = max(r[0] - p[0], 0.0, p[0] - r[2])
    dy = max(r[1] - p[1], 0.0, p[1] - r[3])
    return math.hypot(dx, dy)


def inside_poly(p, poly: Sequence) -> bool:
    """Inside a convex polygon (either winding)."""
    s = [_cross(poly[i], poly[(i + 1) % len(poly)], p) for i in range(len(poly))]
    return all(v >= -1e-12 for v in s) or all(v <= 1e-12 for v in s)


def poly_rect(poly: Sequence, r: Rect) -> float:
    """The least distance between a convex polygon and a rectangle (0: they meet)."""
    if any(inside_rect(p, r) for p in poly) or any(inside_poly(c, poly)
                                                    for c in rect_corners(r)):
        return 0.0
    n = len(poly)
    return min(seg_rect(poly[i], poly[(i + 1) % n], r) for i in range(n))


def seg_rect_many(a: np.ndarray, b: np.ndarray, r: Rect) -> np.ndarray:
    """:func:`seg_rect` of segments ``a[i] b[i]`` (``(N, 2)`` each), vectorized: 0 where a
    segment meets the rectangle (Liang-Barsky clipping), else the least of its ends' and
    the rectangle's corners' distances."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    x0, y0, x1, y1 = r

    def to_rect(p):
        dx = np.maximum.reduce([x0 - p[:, 0], np.zeros(len(p)), p[:, 0] - x1])
        dy = np.maximum.reduce([y0 - p[:, 1], np.zeros(len(p)), p[:, 1] - y1])
        return np.hypot(dx, dy)

    d = b - a
    best = np.minimum(to_rect(a), to_rect(b))
    ll = (d * d).sum(1)
    for cx, cy in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        c = np.array([cx, cy])
        t = np.where(ll > 1e-18, ((c - a) * d).sum(1) / np.where(ll > 1e-18, ll, 1.0), 0.0)
        q = a + np.clip(t, 0.0, 1.0)[:, None] * d
        best = np.minimum(best, np.hypot(*(c - q).T))
    lo, hi = np.zeros(len(a)), np.ones(len(a))
    ok = np.ones(len(a), dtype=bool)
    for p, q in ((-d[:, 0], a[:, 0] - x0), (d[:, 0], x1 - a[:, 0]),
                 (-d[:, 1], a[:, 1] - y0), (d[:, 1], y1 - a[:, 1])):
        par = np.abs(p) < 1e-18
        ok &= ~(par & (q < 0))
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(par, 0.0, q / np.where(par, 1.0, p))
        lo = np.where(~par & (p < 0), np.maximum(lo, t), lo)
        hi = np.where(~par & (p > 0), np.minimum(hi, t), hi)
    return np.where(ok & (lo <= hi), 0.0, best)
