"""A linkage's linearised rigid-body constraints: how small shifts at its joints move it.

The side is planar rigid bodies (the frame, the links, a crank) joined at pins. Each pin
is modelled as an **axle** of its own, and every body on it as one interface to that axle
(a link's hole on the shaft, the frame plate's hole on a pillar): a shift ``e`` across an
interface is that body's clearance on the axle taken up, or its hole off the drawing. With
every input held (an arm's driven links don't turn against the frame; a crank doesn't),
the small motions of the bodies and axles answer the shifts through a square linear system
for a linkage with as many degrees of freedom as declared inputs:

    for each interface (j, body):  motion of body at p_j - motion of axle j at p_j = e
    each axle:                      it doesn't spin (its spin moves nothing at p_j)
    the frame:                      fixed
    each input:                     its link's turn = the frame's

``gains`` solves it at each pose for the output point's motion per unit shift at each
interface (a 2 x 2 matrix per interface). Rotations are scaled by the linkage's size so
the system's condition number is unit-free: it grows without bound where a parallelogram
flattens or a loop locks, which the arm checks refuse (:func:`check_conditioning`), and
where the system is short of equations (an input not declared as a drive) it is singular.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig.linkage.engine import LegSolution, Linkage


@dataclass(frozen=True)
class Layout:
    """The bodies (the frame first), the axles (one per joint shared by two or more bodies)
    and the interfaces ``(joint, body)``; the driven bodies; the output body and point."""

    bodies: tuple[str, ...]
    joints: dict[str, tuple[str, ...]]
    axles: tuple[str, ...]
    interfaces: tuple[tuple[str, str], ...]
    driven: tuple[str, ...]
    output_body: str
    output_point: str


def layout(lk: Linkage) -> Layout:
    joints = {"torso": tuple(lk.frame), **{b: js for b, (js, _) in lk.links.items()}}
    if lk.crank:
        joints["conn"] = tuple(lk.crank)
    bodies = tuple(joints)
    every = dict.fromkeys(j for js in joints.values() for j in js)
    on = {j: [b for b in bodies if j in joints[b]] for j in every}
    axles = tuple(sorted(j for j, bs in on.items() if len(bs) > 1))
    interfaces = tuple((j, b) for j in axles for b in on[j])
    driven = (tuple(lk.drives[k][0] for k in lk.inputs) if lk.kind == "arm" else ("conn",))
    if lk.output is not None:
        out_body, out_pt = lk.output.link, lk.output.point
    else:
        out_body, out_pt = lk.feet[0]
    return Layout(bodies, joints, axles, interfaces, driven, out_body, out_pt)


def _matrix(lay: Layout, pts: dict[str, np.ndarray], s: int, scale: float
            ) -> tuple[np.ndarray, dict[tuple[str, str], int]]:
    """The scaled system at pose ``s``: unknowns ``(dx, dy, L dtheta)`` per body then per
    axle (``L = scale``); rows in mm. Returns the matrix and each interface's first row."""
    names = (*lay.bodies, *(f"axle:{j}" for j in lay.axles))
    col = {n: 3 * i for i, n in enumerate(names)}
    n_rows = 2 * len(lay.interfaces) + len(lay.axles) + 3 + len(lay.driven)
    a = np.zeros((n_rows, 3 * len(names)))
    row_of: dict[tuple[str, str], int] = {}

    def point(r: int, who: str, p: np.ndarray, sign: float) -> None:
        k = col[who]
        a[r, k] += sign
        a[r, k + 2] += -sign * p[1] / scale
        a[r + 1, k + 1] += sign
        a[r + 1, k + 2] += sign * p[0] / scale

    r = 0
    for j, b in lay.interfaces:
        p = pts[j][s]
        point(r, b, p, 1.0)
        point(r, f"axle:{j}", p, -1.0)
        row_of[(j, b)] = r
        r += 2
    for j in lay.axles:                      # an axle doesn't spin
        a[r, col[f"axle:{j}"] + 2] = 1.0
        r += 1
    t = col["torso"]
    a[r, t], a[r + 1, t + 1], a[r + 2, t + 2] = 1.0, 1.0, 1.0      # the frame: fixed
    r += 3
    for b in lay.driven:                     # each input holds its link's turn
        a[r, col[b] + 2], a[r, t + 2] = 1.0, -1.0
        r += 1
    return a, row_of


def scale_of(pts: dict[str, np.ndarray]) -> float:
    """The linkage's size (mm): what makes its rotations comparable to its shifts."""
    flat = np.concatenate([v.reshape(-1) for v in pts.values()])
    return max(float(np.nanmax(np.abs(flat[np.isfinite(flat)]))) if np.isfinite(flat).any()
               else 1.0, 1.0)


@dataclass(frozen=True)
class Gains:
    """The output's motion per unit shift at each interface, at every pose sampled:
    ``per[(joint, body)]`` an array ``(poses, 2, 2)`` (NaN where the system is singular);
    ``cond`` each pose's condition number (inf: singular, or short of equations)."""

    per: dict[tuple[str, str], np.ndarray]
    cond: np.ndarray

    @property
    def singular(self) -> int:
        return int((~np.isfinite(self.cond)).sum())


SINGULAR = 1e-12


def gains(lk: Linkage, values: tuple[float, ...], ins: tuple[np.ndarray, ...]) -> Gains:
    """:class:`Gains` at the explicit inputs ``ins`` (radians, one array per input)."""
    lay = layout(lk)
    with np.errstate(all="ignore"):
        pts = LegSolution(1, 0.0, values, lk.key).at(*ins)
    scale = scale_of(pts)
    n = next(iter(pts.values())).shape[0]
    names = (*lay.bodies, *(f"axle:{j}" for j in lay.axles))
    k_out = 3 * names.index(lay.output_body)
    per = {i: np.full((n, 2, 2), np.nan) for i in lay.interfaces}
    cond = np.full(n, math.inf)
    for s in range(n):
        a, row_of = _matrix(lay, pts, s, scale)
        if not np.isfinite(a).all() or a.shape[0] < a.shape[1]:
            continue
        try:
            sv = np.linalg.svd(a, compute_uv=False)
        except np.linalg.LinAlgError:       # (no convergence: as singular as it gets)
            continue
        if sv[-1] <= SINGULAR * sv[0]:
            continue
        cond[s] = float(sv[0] / sv[-1])
        f = pts[lay.output_point][s]
        sel = np.zeros((2, a.shape[1]))
        sel[0, k_out], sel[0, k_out + 2] = 1.0, -f[1] / scale
        sel[1, k_out + 1], sel[1, k_out + 2] = 1.0, f[0] / scale
        g = sel @ np.linalg.pinv(a)
        for i, r in row_of.items():
            per[i][s] = g[:, r:r + 2]
    return Gains(per, cond)


def sample_inputs(lk: Linkage, n: int) -> tuple[np.ndarray, ...]:
    """An arm's box ``n`` a side; a cranked linkage's ``4 n`` crank angles (the others at
    0, as its template holds them)."""
    if lk.kind == "arm":
        return lk.box(n)
    t = np.linspace(0.0, 2 * math.pi, 4 * n, endpoint=False)
    return (t, *[np.zeros_like(t)] * (len(lk.inputs) - 1))


LOCK_COND = 2000.0
"""The worst condition number an arm may have anywhere in its box: the default palletizer
is under 150 (its tightest parallelogram 20 deg), and one flattened to a fraction of a
degree is in the tens of thousands (:func:`check_conditioning`)."""


def check_conditioning(lk: Linkage, values: tuple[float, ...], n: int = 25
                       ) -> tuple[float, tuple[float, ...] | None]:
    """The worst condition number over an arm's box, and the inputs (deg) where it is."""
    ins = sample_inputs(lk, n)
    g = gains(lk, values, ins)
    k = int(np.argmax(np.where(np.isfinite(g.cond), g.cond, np.inf)))
    return float(g.cond[k]), tuple(float(np.degrees(a[k])) for a in ins)
