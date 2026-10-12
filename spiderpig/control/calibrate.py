"""The link lengths and the joints' zeros fitted to measured poses.

A built arm differs from its drawing: every hole is off by up to the laser's tolerance
(:mod:`spiderpig.precision`'s accuracy rows), every homing's zero by the gauge's error. Both
are systematic, so they calibrate out: command the arm to a spread of poses, measure where
its output point is each time (a dial indicator's touches against known points, a camera, a
coordinate table), and :func:`fit` finds the linkage's parameters and the inputs' zero
offsets that best explain them (bounded least squares on the planar output point, the
parameters within ``span`` of the drawing). The fitted values replace the drawing's in the
arm's model (:class:`.arm.ArmModel`, ``values``).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from spiderpig import linkage


@dataclass(frozen=True)
class Fit:
    """What :func:`fit` found: every parameter's value (the linkage's order), each input's
    zero offset (rad, added to its commanded angle), and the residuals (mm)."""

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
