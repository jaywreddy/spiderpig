"""Sketch: Klann kinematics as a small symbolic straight-line program.

Reference for the refactor proposed in ``docs/audit/AUDIT.md``. Not wired into
the pipeline; run it to see parity with ``klann.py`` and the timings::

    uv run python docs/audit/klann_core_sketch.py

Design goals (from the audit):

* **Readable sympy.** Each construction step is one tiny expression over
  *symbols* of earlier steps — never a substituted mega-expression. ``F``
  in ``klann.py`` is ~34k ops; here every step prints legibly (the circle
  intersections are the largest, ~130 ops).
* **Design parameters are symbols** (exact rationals / degrees), not
  baked-in floats, so one compiled program serves every proportion set,
  both chiralities, and every phase.
* **Phase is a time shift**, not a new symbolic solve: leg k evaluates the
  same program at ``t + phase_k``.
* **Links are rigid bodies with fixed local geometry.** Each link gets a
  closed-form SE(2) pose ``(x, y, theta)(t)``; parts are built once in the
  link frame. No Procrustes, no witness joints, no per-t part rebuilds.
* **Fast path = compile the program once** (lambdify with CSE) and evaluate
  over ``(T,)`` or ``(T, P)`` numpy batches.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import cache

import numpy as np
import sympy as sp

t = sp.Symbol("t", real=True)
s = sp.Symbol("s", real=True)  # chirality (+1 / -1)

# Klann's published proportions, in units of the crank-to-A distance.
PARAMS: dict[str, sp.Expr] = {
    "OA": sp.Integer(60),
    "angA": sp.Rational(6628, 100),   # degrees
    "OB": sp.Rational(1121, 1000),    # × OA
    "angB": sp.Rational(-4176, 100),  # degrees
    "OM": sp.Rational(412, 1000),
    "MC": sp.Rational(1143, 1000),
    "AC": sp.Rational(909, 1000),
    "CD": sp.Rational(726, 1000),
    "DE": sp.Rational(93, 100),
    "BE": sp.Rational(8, 10),
    "DF": sp.Rational(2577, 1000),
}
p = {k: sp.Symbol(k, positive=True) if k not in ("angA", "angB") else sp.Symbol(k, real=True)
     for k in PARAMS}


def V(name: str) -> sp.Matrix:
    return sp.Matrix(sp.symbols(f"{name}x {name}y", real=True))


def rot(v: sp.Matrix, deg: sp.Expr) -> sp.Matrix:
    a = deg * sp.pi / 180
    return sp.Matrix([[sp.cos(a), -sp.sin(a)], [sp.sin(a), sp.cos(a)]]) * v


def circle_x_circle(c1: sp.Matrix, r1, c2: sp.Matrix, r2, branch) -> sp.Matrix:
    """Two-circle intersection; ``branch`` = +1/-1 picks the side of c1→c2."""
    d = c2 - c1
    L2 = d.dot(d)
    a = (r1**2 - r2**2 + L2) / (2 * L2)
    h = sp.sqrt(r1**2 / L2 - a**2)
    perp = sp.Matrix([-d[1], d[0]])
    return c1 + a * d + branch * h * perp


def extend(frm: sp.Matrix, through: sp.Matrix, length) -> sp.Matrix:
    """Point ``length`` beyond ``through`` on the ray frm→through."""
    u = through - frm
    return through + u * length / sp.sqrt(u.dot(u))


# ---- the program: (name, expression over earlier symbols) -----------------
OA = p["OA"]
STEPS: list[tuple[str, sp.Matrix]] = [
    ("O", sp.Matrix([0, 0])),
    ("A", rot(sp.Matrix([0, -OA]), s * p["angA"])),
    ("B", rot(sp.Matrix([0, p["OB"] * OA]), s * p["angB"])),
    ("M", p["OM"] * OA * sp.Matrix([sp.cos(t), sp.sin(t)])),
    ("C", circle_x_circle(V("M"), p["MC"] * OA, V("A"), p["AC"] * OA, s)),
    ("D", extend(V("M"), V("C"), p["CD"] * OA)),
    ("E", circle_x_circle(V("B"), p["BE"] * OA, V("D"), p["DE"] * OA, s)),
    ("F", extend(V("E"), V("D"), p["DF"] * OA)),
]

# Links: name -> ordered joints along the bar (first = frame origin, second
# fixes the x-axis). Local joint coords are constants of the design.
LINKS: dict[str, tuple[str, ...]] = {
    "conn": ("O", "M"), "b1": ("M", "C", "D"), "b2": ("B", "E"),
    "b3": ("A", "C"), "b4": ("E", "D", "F"),
}


def closed_form() -> dict[str, sp.Matrix]:
    """Fully substituted expressions (for inspection / diff, not for speed)."""
    env: dict[sp.Symbol, sp.Expr] = {}
    out = {}
    for name, expr in STEPS:
        e = expr.xreplace(env)
        out[name] = e
        vx, vy = V(name)
        env[vx], env[vy] = e[0], e[1]
    return out


@dataclass(frozen=True)
class Compiled:
    fn: object          # (t, s, *params) -> flat tuple of point coords
    names: list[str]

    def points(self, ts, chirality=1, **overrides) -> dict[str, np.ndarray]:
        vals = {k: float(v) for k, v in PARAMS.items()} | overrides
        flat = self.fn(np.asarray(ts, float), float(chirality), *(vals[k] for k in PARAMS))
        ts = np.asarray(ts, float)
        return {n: np.stack(np.broadcast_arrays(flat[2 * i], flat[2 * i + 1], ts)[:2], -1)
                for i, n in enumerate(self.names)}

    def link_poses(self, ts, chirality=1, **kw) -> dict[str, np.ndarray]:
        """SE(2) pose per link as (T, 3) = (x, y, theta). Closed form, no fitting."""
        P = self.points(ts, chirality, **kw)
        out = {}
        for link, (j0, j1, *_) in LINKS.items():
            d = P[j1] - P[j0]
            out[link] = np.column_stack([P[j0], np.arctan2(d[..., 1], d[..., 0])])
        return out


@cache
def compile_program() -> Compiled:
    """Chain the steps symbolically, CSE, and lambdify once (the fast path)."""
    cf = closed_form()
    names = [n for n, _ in STEPS]
    exprs = [c for n in names for c in cf[n]]
    fn = sp.lambdify([t, s, *p.values()], exprs, modules="numpy", cse=True)
    return Compiled(fn=fn, names=names)


if __name__ == "__main__":
    import sys
    import time
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

    print("each step is small and legible:")
    for name, e in STEPS:
        print(f"  {name}: ops={sum(sp.count_ops(c) for c in e):3d}")
    print("  C =", sp.simplify(STEPS[4][1][0]))

    t0 = time.perf_counter()
    prog = compile_program()
    print(f"compile once (all params symbolic, both chiralities): "
          f"{time.perf_counter() - t0:.3f}s")

    ts = np.linspace(0, 2 * np.pi, 100_000)
    t0 = time.perf_counter()
    prog.points(ts)
    print(f"eval 1e5 samples: {(time.perf_counter() - t0) * 1e3:.1f} ms")

    # parity with the current klann.py (both chiralities, a few phases)
    import klann
    for chir in (1, -1):
        for ph in (0.0, np.pi / 2):
            ref = klann.create_klann_geometry(chir, ph).callables["F"](ts[:720])
            got = prog.points(ts[:720] + ph, chir)["F"]
            err = np.abs(np.stack(ref, -1) - got).max()
            print(f"  parity F vs klann.py chirality={chir:+d} phase={ph:.2f}: max|err|={err:.2e}")

    # rigidity + closed-form link poses
    L = prog.link_poses(ts)
    print("link pose array shapes:", {k: v.shape for k, v in L.items()})

    # parameter sweep without re-running sympy: 200 designs x 360 samples
    t0 = time.perf_counter()
    stride = []
    for k in np.linspace(2.4, 2.8, 200):
        F = prog.points(np.linspace(0, 2 * np.pi, 360), DF=k)["F"]
        stride.append(np.ptp(F[:, 0]))
    t1 = time.perf_counter()
    print(f"200-design DF sweep (no sympy rebuild): {(t1 - t0) * 1e3:.0f} ms, "
          f"stride {min(stride):.1f}..{max(stride):.1f} mm")
