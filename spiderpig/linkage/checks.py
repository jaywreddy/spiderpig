"""Stage checks: does every loop of a linkage close, and does a mechanism's output do its
job? :meth:`linkage.Linkage.check` / :meth:`linkage.Linkage.output_check` call these (the
results are cached per linkage and parameter values).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import cache
from typing import Any

import numpy as np

from spiderpig.linkage.engine import LegSolution, Linkage, Output, get

# ---------------------------------------------------------------------------
# Stage check: does every loop close, and how well?
# ---------------------------------------------------------------------------

TOGGLE_DEG = 15.0     # a transmission angle this close to 0° or 180° is flagged


@dataclass(frozen=True)
class StepCheck:
    """One step of a program over a revolution (leg as defined, phase 0).

    ``kind``: ``fixed`` / ``crank`` (no earlier points), ``closure`` (placed
    at given distances from two earlier points that move relative to each
    other: a loop closes here), ``rigid`` (fixed relative to two earlier
    points) or ``derived``. For a closure: the two bar lengths, the margin
    (how far the loop is from failing to close, worst over the cycle; < 0 is
    a failure), the crank-angle range where it fails and the transmission
    angle range at the new joint.
    """

    point: str
    kind: str
    refs: tuple[str, ...]
    radii: tuple[float, float] | None = None
    margin_mm: float | None = None
    worst_deg: float | None = None
    fails_deg: tuple[float, float] | None = None
    angle_deg: tuple[float, float] | None = None
    fail_fraction: float = 0.0
    worst_t2_deg: float | None = None     # a second input's angle at the worst sample

    @property
    def toggles(self) -> bool:
        return self.angle_deg is not None and min(self.angle_deg[0],
                                                  180 - self.angle_deg[1]) < TOGGLE_DEG

    def describe(self) -> str:
        if self.kind != "closure":
            return f"{self.point}: {self.kind} ({', '.join(self.refs) or 'no earlier points'})"
        a, b = self.refs
        if self.radii is None:
            return (f"joint {self.point} can't be placed at any crank angle: "
                    f"its bars from {a} and {b} never meet")
        r1, r2 = self.radii
        where = f"bars {a}-{self.point} {r1:.1f} mm and {b}-{self.point} {r2:.1f} mm"
        at = f"{self.worst_deg:.0f}°" + ("" if self.worst_t2_deg is None
                                         else f", t2 {self.worst_t2_deg:.0f}°")
        if self.fails_deg is not None:
            lo, hi = self.fails_deg
            return (f"joint {self.point} can't be placed for {self.fail_fraction:.0%} of the cycle "
                    f"(crank angles {lo:.0f}°..{hi:.0f}°): {where} miss each other by up to "
                    f"{-self.margin_mm:.2f} mm (worst at {at})")
        lo, hi = self.angle_deg
        flag = "; near toggle" if self.toggles else ""
        return (f"{self.point}: {where} close with {self.margin_mm:.2f} mm to spare "
                f"(worst at {at}), transmission angle {lo:.0f}°..{hi:.0f}°{flag}")


def _inputs(lk: Linkage, n: int = 720) -> list[np.ndarray]:
    """What a stage check samples: ``n`` crank angles; with two inputs, a grid of
    ``(n/4)²`` over their torus."""
    if len(lk.inputs) == 1:
        return [2.0 * math.pi * np.arange(n) / n]
    g = 2.0 * math.pi * np.arange(n // 4) / (n // 4)
    return [a.ravel() for a in np.meshgrid(*[g] * len(lk.inputs), indexing="ij")]


@cache
def check_steps(key: str, values: tuple[float, ...], n: int = 720) -> tuple[StepCheck, ...]:
    lk = get(key)
    ins = _inputs(lk, n)
    ts = ins[0]
    with np.errstate(all="ignore"):
        pts = LegSolution(1, 0.0, values, key).evaluate(*ins)
    out = []
    for name, expr in lk.steps:
        syms = {s.name for s in expr.free_symbols}
        refs = tuple(p for p in lk.points if p != name and {f"{p}x", f"{p}y"} & syms)
        if len(refs) != 2:
            kind = "derived" if refs else ("crank" if "t" in syms else "fixed")
            out.append(StepCheck(name, kind, refs))
            continue
        z, a, b = pts[name], pts[refs[0]], pts[refs[1]]
        u, v = a - z, b - z
        ang = np.degrees(np.arctan2(np.abs(u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]),
                                    (u * v).sum(-1)))
        ok = np.isfinite(ang)
        if not (np.isfinite(a).all() and np.isfinite(b).all()):
            out.append(StepCheck(name, "derived", refs))    # an earlier step already failed
            continue
        if not ok.any():
            out.append(StepCheck(name, "closure", refs, None, -math.inf, 0.0, (0.0, 360.0),
                                 fail_fraction=1.0))
            continue
        if np.ptp(ang[ok]) < 1e-6:
            out.append(StepCheck(name, "rigid", refs))
            continue
        r1 = float(np.median(np.linalg.norm(u[ok], axis=-1)))
        r2 = float(np.median(np.linalg.norm(v[ok], axis=-1)))
        d = np.linalg.norm(a - b, axis=-1)
        margin = np.minimum(r1 + r2 - d, d - abs(r1 - r2))
        k = int(np.argmin(margin))
        fails = np.degrees(ts[margin < 0])
        out.append(StepCheck(
            name, "closure", refs, (r1, r2), float(margin[k]), math.degrees(ts[k]),
            (float(fails.min()), float(fails.max())) if fails.size else None,
            (float(ang[ok].min()), float(ang[ok].max())), fails.size / ts.size,
            math.degrees(ins[1][k]) if len(ins) > 1 else None,
        ))
    return tuple(out)


# ---------------------------------------------------------------------------
# Stage check: does a mechanism's output do its job?
# ---------------------------------------------------------------------------

ON_LINE_MM = 0.05     # a point this close to its fitted line is on it
STILL_DEG = 1e-6      # a translating platform turns no more than this


@dataclass(frozen=True)
class OutputCheck:
    """A mechanism's output over the cycle (or the torus of its inputs), in mm and degrees.

    ``extent_mm``: the output point's path, x and y. Over its straight
    stretch (``Output.straight``; a platform's whole turn): ``stroke_mm``
    along the fitted line; if it promises one, ``straightness_mm`` (the band
    across the line) and ``on_line`` (the longest part of the turn within
    :data:`ON_LINE_MM` of it). A platform's ``rotation_deg``; a rotation's
    ``swing_deg`` and, if it promises one, its ``dwell_deg`` (crank degrees
    within its tolerance). ``broken``: the promise it breaks, if any.
    """

    key: str
    output: Output
    extent_mm: tuple[float, float]
    stroke_mm: float | None = None
    straightness_mm: float | None = None
    on_line: float | None = None
    rotation_deg: float | None = None
    swing_deg: float | None = None
    dwell_deg: float | None = None
    broken: str | None = None

    def describe(self) -> str:
        o = self.output
        out = [f"{o.name} ({o.motion}) covers {self.extent_mm[0]:.2f} x {self.extent_mm[1]:.2f} mm"]
        if self.stroke_mm is not None:
            out.append(f"stroke {self.stroke_mm:.2f} mm")
        if self.straightness_mm is not None:
            lo, hi, _ = o.straight
            out.append(f"straight to {self.straightness_mm:.3g} mm over crank {lo:g}°..{hi:g}°, "
                       f"on the line for {self.on_line:.1%} of the turn")
        if self.rotation_deg is not None:
            out.append(f"turns {self.rotation_deg:.3g}°")
        if self.swing_deg is not None:
            out.append(f"swings {self.swing_deg:.2f}° about {o.frame[0]}")
        if self.dwell_deg is not None:
            out.append(f"stands still (±{o.dwell[0]:g}°) for {self.dwell_deg:.1f}° of the turn")
        if self.broken:
            out.append(f"BROKEN: {self.broken}")
        return f"{self.key}: " + ", ".join(out)


def _longest_run(mask: np.ndarray) -> int:
    """Longest cyclic run of True."""
    if mask.all():
        return mask.size
    edges = np.diff(np.concatenate([[0], mask, mask, [0]]).astype(int))
    return int((np.flatnonzero(edges < 0) - np.flatnonzero(edges > 0)).max(initial=0))


def _dwell(psi: np.ndarray, tol: float) -> int:
    """Most consecutive samples (cyclic) of ``psi`` within a band ``2 tol`` wide."""
    lo, hi, k = psi, psi, 0
    while k < psi.size and (hi - lo <= 2 * tol).any():
        k += 1
        nxt = np.roll(psi, -k)
        lo, hi = np.minimum(lo, nxt), np.maximum(hi, nxt)
    return k


@cache
def check_output(key: str, values: tuple[float, ...], n: int = 720) -> OutputCheck:
    lk = get(key)
    o = lk.output
    ins = _inputs(lk, n)
    deg = np.degrees(ins[0])
    pts = LegSolution(1, 0.0, values, key).evaluate(*ins)
    p, a, b = pts[o.point], pts[o.frame[0]], pts[o.frame[1]]
    ang = np.degrees(np.unwrap(np.arctan2(b[:, 1] - a[:, 1], b[:, 0] - a[:, 0])))
    got: dict[str, Any] = {"extent_mm": tuple(float(v) for v in np.ptp(p, axis=0))}
    broken = []
    if o.straight or o.motion == "translation_platform":
        lo, hi, tol = o.straight or (0.0, 360.0, math.inf)
        win = (deg >= lo) & (deg <= hi) if lo <= hi else (deg >= lo) | (deg <= hi)
        c = p[win].mean(0)
        along, across = np.linalg.svd(p[win] - c, full_matrices=False)[2]
        off = (p - c) @ across
        got["stroke_mm"] = float(np.ptp((p[win] - c) @ along))
        if o.straight:
            band = float(np.ptp(off[win]))
            got.update(straightness_mm=band,
                       on_line=_longest_run(np.abs(off) <= ON_LINE_MM) / off.size)
            if band > tol:
                broken.append(f"output {o.name} strays across a {band:.3g} mm band about its line "
                              f"over crank {lo:g}°..{hi:g}°, wider than the {tol:g} mm it promises")
    if o.motion == "translation_platform":
        got["rotation_deg"] = rot = float(np.ptp(ang))
        if rot > STILL_DEG:
            k = int(np.argmax(np.abs(ang - ang[0])))
            broken.append(f"platform {o.name} turns {rot:.3g}° over the cycle (most at crank "
                          f"{deg[k]:.0f}°): a translating platform must not rotate")
    if o.motion == "rotation":
        got["swing_deg"] = float(np.ptp(ang))
        if o.dwell:
            tol, need = o.dwell
            got["dwell_deg"] = dwell = 360.0 * _dwell(ang, tol) / ang.size
            if dwell < need:
                broken.append(f"output {o.name} stands still (±{tol:g}°) for only {dwell:.1f}° of "
                              f"the turn, less than the {need:g}° it promises")
    return OutputCheck(key, o, **got, broken="; ".join(broken) or None)
