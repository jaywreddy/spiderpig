"""The link lengths and the joints' zeros fitted to measured poses.

A built arm differs from its drawing: its holes are off by up to the laser's tolerance
(:mod:`spiderpig.precision`'s accuracy rows), every homing's zero by the gauge's error.
What the linkage's parameters describe calibrates out: command the arm to a spread of
poses, measure where its output point is each time (in the side's plane: a dial
indicator's touches against known points, a camera, a coordinate table), and :func:`fit`
finds the parameters named and the inputs' zero offsets that best explain them (bounded
least squares on the planar output point, the parameters within ``span`` of the drawing).
Fit the measuring frame too where it isn't the design's (the palletizer's flange offset
``fx``, ``fy`` absorbs a shifted frame). What they don't describe doesn't: a parallelogram
whose rod or crank differs from its opposite side turns the forearm by a pose-dependent
angle the program has no parameter for (the rods' lengths are implied), so its residual
stays. The fitted values replace the drawing's in the arm's model
(:class:`.arm.ArmModel`, ``values``); the offsets move the joints' zeros
(:func:`apply_offsets`).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from spiderpig import linkage


@dataclass(frozen=True)
class Fit:
    """What :func:`fit` found: every parameter's value (the linkage's order), each input's
    zero offset (rad: the joint's true angle is its commanded one plus it), and the
    residuals (mm)."""

    values: tuple[float, ...]
    offsets: dict[str, float]
    rms_mm: float
    worst_mm: float


def fit(lk_key: str, values: tuple[float, ...], q: np.ndarray, measured: np.ndarray,
        params: tuple[str, ...], output: str, offsets: bool = True,
        span: float = 0.05) -> Fit:
    """Fit ``params`` (and the inputs' zero offsets) to poses: ``q`` the commanded inputs
    ``(N, n_inputs)`` (rad), ``measured`` the output point there ``(N, 2)`` (mm)."""
    from scipy.optimize import least_squares

    lk = linkage.get(lk_key)
    names = list(lk.params)
    idx = [names.index(p) for p in params]
    base = np.asarray(values, dtype=float)
    n_in = len(lk.inputs)
    q = np.asarray(q, dtype=float)
    measured = np.asarray(measured, dtype=float)

    def model(v):
        vals = base.copy()
        vals[idx] = v[:len(idx)]
        off = v[len(idx):] if offsets else np.zeros(n_in)
        sol = linkage.LegSolution(1, 0.0, tuple(vals), lk_key)
        with np.errstate(all="ignore"):
            pts = sol.at(*(q[:, i] + off[i] for i in range(n_in)))[output]
        return np.asarray(pts, dtype=float).reshape(-1, 2)

    def resid(v):
        return np.nan_to_num((model(v) - measured).ravel(), nan=1e3)

    x0 = np.concatenate([base[idx], np.zeros(n_in if offsets else 0)])
    scale = np.abs(base[idx]) * span + 1e-9
    lo = np.concatenate([base[idx] - scale, np.full(n_in if offsets else 0, -0.2)])
    hi = np.concatenate([base[idx] + scale, np.full(n_in if offsets else 0, 0.2)])
    sol = least_squares(resid, x0, bounds=(lo, hi), xtol=1e-12, ftol=1e-12, gtol=1e-12)
    vals = base.copy()
    vals[idx] = sol.x[:len(idx)]
    r = np.linalg.norm(model(sol.x) - measured, axis=1)
    off = {n: float(sol.x[len(idx) + i]) if offsets else 0.0 for i, n in enumerate(lk.inputs)}
    return Fit(tuple(float(v) for v in vals), off, float(np.sqrt((r * r).mean())),
               float(r.max()))


def apply_offsets(fit_: Fit, axes: dict) -> dict:
    """``axes`` (:class:`.mapping.Axis` by joint) with each fitted offset folded into its
    zero: the true angle is the commanded one plus the offset, so the angle at a count is
    the zero plus it."""
    from dataclasses import replace

    return {n: replace(a, zero=a.zero + fit_.offsets.get(n, 0.0)) for n, a in axes.items()}
