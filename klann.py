"""Symbolic Klann walking-linkage geometry and mechanism assembly.

The Klann linkage is a 6-bar planar mechanism that converts continuous
rotation of a crank into a foot path well suited to walking robots.

The geometry is a short symbolic *straight-line program*: each step
(:data:`STEPS`) is one small sympy expression over the symbols of earlier
points, the crank angle ``t``, the chirality ``s`` (+1 right-hand, -1
mirrored) and the design proportions (:data:`PROPORTIONS`, exact rationals).
Nothing is substituted until :func:`compile_program`, which chains the steps,
runs CSE and lambdifies once per process. Every leg of every assembly then
evaluates that one compiled program: a leg's ``phase`` is a time shift,
``t + phase``, never a new symbolic solve.

Downstream code (shapes.py, layout.py, the glTF bake) evaluates a
:class:`KlannSolution` at float ``t`` via :meth:`KlannSolution.joints_at`, or
over a batch of times via :meth:`KlannSolution.evaluate`.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from contextlib import contextmanager
from dataclasses import dataclass
from functools import cache, cached_property
from typing import Any

import numpy as np
import sympy as sp

from mechanism import (
    BodyTemplate,
    JointTemplate,
    Mechanism,
    MechanismTemplate,
    translation_pose_at,
)
from stack import StackSpec

# Optional profiler hook. ``viewer/bake_gltf.py`` sets this to its
# ``_Profiler`` instance at the start of a bake so the symbolic work
# (create_klann_geometry, lambdify, joints_at, body wiring) shows up as
# labelled rows in the bake profile summary. Leave ``None`` in normal use.
_BAKE_PROFILER: Any = None


@contextmanager
def _maybe_timed(label: str):
    prof = _BAKE_PROFILER
    if prof is None:
        yield
        return
    with prof.timed(label):
        yield


# ---------------------------------------------------------------------------
# Symbolic core
# ---------------------------------------------------------------------------

t = sp.Symbol("t", real=True)   # crank angle
s = sp.Symbol("s", real=True)   # chirality: +1 right-hand, -1 mirrored

# Joseph Klann's published proportions. Lengths are multiples of ``OA`` (the
# crank-centre-to-A distance, in mm); angles are in degrees.
PROPORTIONS: dict[str, sp.Expr] = {
    "OA": sp.Integer(60),
    "angA": sp.Rational(6628, 100),
    "OB": sp.Rational(1121, 1000),
    "angB": sp.Rational(-4176, 100),
    "OM": sp.Rational(412, 1000),
    "MC": sp.Rational(1143, 1000),
    "AC": sp.Rational(909, 1000),
    "CD": sp.Rational(726, 1000),
    "DE": sp.Rational(93, 100),
    "BE": sp.Rational(8, 10),
    "DF": sp.Rational(2577, 1000),
}
_ANGLES = {"angA", "angB"}
PARAMS: dict[str, sp.Symbol] = {
    k: sp.Symbol(k, real=True) if k in _ANGLES else sp.Symbol(k, positive=True)
    for k in PROPORTIONS
}


def _pt(name: str) -> sp.Matrix:
    """Symbolic stand-in for an earlier step's point."""
    return sp.Matrix(sp.symbols(f"{name}x {name}y", real=True))


def rotate(v: sp.Matrix, degrees: sp.Expr) -> sp.Matrix:
    a = degrees * sp.pi / 180
    return sp.Matrix([[sp.cos(a), -sp.sin(a)], [sp.sin(a), sp.cos(a)]]) * v


def circle_x_circle(c1: sp.Matrix, r1, c2: sp.Matrix, r2, branch) -> sp.Matrix:
    """Intersection of two circles; ``branch`` = +1 / -1 picks the side of c1->c2."""
    d = c2 - c1
    dd = d.dot(d)
    a = (r1**2 - r2**2 + dd) / (2 * dd)
    h = sp.sqrt(r1**2 / dd - a**2)
    return c1 + a * d + branch * h * sp.Matrix([-d[1], d[0]])


def extend(frm: sp.Matrix, through: sp.Matrix, length) -> sp.Matrix:
    """The point ``length`` beyond ``through`` on the ray frm -> through."""
    u = through - frm
    return through + u * length / sp.sqrt(u.dot(u))


_OA = PARAMS["OA"]
STEPS: list[tuple[str, sp.Matrix]] = [
    ("O", sp.Matrix([0, 0])),
    ("A", rotate(sp.Matrix([0, -_OA]), s * PARAMS["angA"])),
    ("B", rotate(sp.Matrix([0, PARAMS["OB"] * _OA]), s * PARAMS["angB"])),
    ("M", PARAMS["OM"] * _OA * sp.Matrix([sp.cos(t), sp.sin(t)])),
    ("C", circle_x_circle(_pt("M"), PARAMS["MC"] * _OA, _pt("A"), PARAMS["AC"] * _OA, s)),
    ("D", extend(_pt("M"), _pt("C"), PARAMS["CD"] * _OA)),
    ("E", circle_x_circle(_pt("B"), PARAMS["BE"] * _OA, _pt("D"), PARAMS["DE"] * _OA, s)),
    ("F", extend(_pt("E"), _pt("D"), PARAMS["DF"] * _OA)),
]
POINTS: tuple[str, ...] = tuple(name for name, _ in STEPS)

# Link topology: joints along each bar. The first two fix the link's frame.
LINKS: dict[str, tuple[str, ...]] = {
    "conn": ("O", "M"),
    "b1": ("M", "C", "D"),
    "b2": ("B", "E"),
    "b3": ("A", "C"),
    "b4": ("E", "D", "F"),
}


def closed_form() -> dict[str, sp.Matrix]:
    """Fully substituted point expressions (for inspection and differentiation)."""
    env: dict[sp.Symbol, sp.Expr] = {}
    out: dict[str, sp.Matrix] = {}
    for name, expr in STEPS:
        e = expr.xreplace(env)
        out[name] = e
        x, y = _pt(name)
        env[x], env[y] = e[0], e[1]
    return out


@cache
def compile_program() -> Callable[..., list]:
    """Lambdify the whole program once: ``(t, s, *params) -> [Ox, Oy, Ax, ...]``."""
    with _maybe_timed("4.1b_klann.lambdify"):
        cf = closed_form()
        flat = [c for name in POINTS for c in cf[name]]
        return sp.lambdify([t, s, *PARAMS.values()], flat, modules="numpy", cse=True)


_DEFAULT_VALUES = tuple(float(v) for v in PROPORTIONS.values())


@dataclass(frozen=True)
class KlannSolution:
    """One leg's trajectory: the shared program at a chirality and phase.

    ``orientation=+1`` is a right-hand leg, ``-1`` its mirror. The crank
    angle of this leg is ``t + phase``.
    """

    orientation: int = 1
    phase: float = 0.0

    @cached_property
    def points(self) -> dict[str, sp.Matrix]:
        """Closed-form ``(x, y)`` per named point, in ``t`` (slow; for inspection)."""
        subs = {s: self.orientation, t: t + self.phase}
        subs |= {PARAMS[k]: v for k, v in PROPORTIONS.items()}
        return {name: e.xreplace(subs) for name, e in closed_form().items()}

    @property
    def segments(self) -> dict[str, tuple[str, ...]]:
        """Link name -> the named joints along it."""
        return LINKS

    def evaluate(self, ts) -> dict[str, np.ndarray]:
        """Every named point over ``ts``: ``name -> (..., 2)`` array."""
        ts = np.asarray(ts, dtype=float)
        flat = compile_program()(ts + self.phase, float(self.orientation), *_DEFAULT_VALUES)
        return {
            name: np.stack(np.broadcast_arrays(flat[2 * i], flat[2 * i + 1], ts)[:2], axis=-1)
            for i, name in enumerate(POINTS)
        }

    @cached_property
    def callables(self) -> dict[str, Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]]]:
        """One numpy callable ``(ts,) -> (xs, ys)`` per named point."""

        def _fn(name: str):
            def f(ts):
                xy = self.evaluate(ts)[name]
                return xy[..., 0], xy[..., 1]
            return f

        return {name: _fn(name) for name in POINTS}

    def joints_at(self, t_value: float) -> dict[str, tuple[float, float]]:
        """Evaluate every named point at ``t_value`` and return float (x, y) pairs."""
        with _maybe_timed("4.1c_klann.joints_at_eval"):
            xy = self.evaluate(float(t_value))
            return {name: (float(v[0]), float(v[1])) for name, v in xy.items()}


def create_klann_geometry(orientation: int = 1, phase: float = 0.0) -> KlannSolution:
    """The Klann leg of the given chirality, with crank angle ``t + phase``.

    Cheap: the symbolic program is compiled once per process
    (:func:`compile_program`); this only records the chirality and phase.
    """
    with _maybe_timed("4.1a_klann.create_geometry"):
        compile_program()
        return KlannSolution(orientation=int(orientation), phase=float(phase))


# ---------------------------------------------------------------------------
# One leg as a kinematic template
# ---------------------------------------------------------------------------
#
# Every joint sits at z = 0 in its body's frame: the mechanism is planar, and
# which Z slot each part occupies is a fabrication decision made by
# :mod:`stack` (see :mod:`fabricate`). Joint XY comes straight from the
# compiled symbolic program.

# body -> (joints, outline, colour). ``outline`` is the joint pair(s) the
# laser-cut shape spans; F is b4's foot tip (an outline point, not a pivot).
_LEG_BODIES: dict[str, tuple[tuple[str, ...], tuple[tuple[str, str], ...], str]] = {
    "coupler": (("O",), (), "blue"),
    "b1": (("M", "C", "D"), (("M", "D"),), "green"),
    "b2": (("B", "E"), (("B", "E"),), "green"),
    "b3": (("A", "C"), (("A", "C"),), "green"),
    "b4": (("E", "D", "F"), (("E", "F"),), "green"),
    "conn": (("O", "M"), (("O", "M"),), "orange"),
    "torso": (("A", "O", "B"), (), "yellow"),
}

# Per-leg connections. Body names pick up a ``name_suffix`` so legs can merge.
_CONN_TEMPLATE: list[tuple[tuple[int, str, str], tuple[int, str, str]]] = [
    ((0, "torso", "A"), (0, "b3", "A")),
    ((0, "torso", "B"), (0, "b2", "B")),
    ((0, "torso", "O"), (0, "conn", "O")),
    ((0, "conn", "M"), (0, "b1", "M")),
    ((0, "b1", "C"), (0, "b3", "C")),
    ((0, "b1", "D"), (0, "b4", "D")),
    ((0, "b2", "E"), (0, "b4", "E")),
    ((0, "conn", "O"), (0, "coupler", "O")),
]


def build_klann_template(solution: KlannSolution, *, name_suffix: str = "") -> MechanismTemplate:
    """One Klann leg whose joint poses are closures over the compiled program."""
    with _maybe_timed("4.1d_klann.assemble_leg"):
        callables = solution.callables
        bodies = [
            BodyTemplate(
                name=f"{name}{name_suffix}",
                joints=[
                    JointTemplate(name=j, pose_at=translation_pose_at(callables[j], z=0.0))
                    for j in joints
                ],
                outline=outline,
                color=color,
            )
            for name, (joints, outline, color) in _LEG_BODIES.items()
        ]
        connections = [
            ((pi, f"{pn}{name_suffix}", pj), (ci, f"{cn}{name_suffix}", cj))
            for (pi, pn, pj), (ci, cn, cj) in _CONN_TEMPLATE
        ]
        return MechanismTemplate(name=f"klann{name_suffix}", bodies=bodies, connections=connections)


# ---------------------------------------------------------------------------
# Composition primitives for multi-leg assemblies
# ---------------------------------------------------------------------------
#
# The 2016 project stacked 2–4 Klann legs into walker sculptures by sharing
# rigid bodies across legs (one crank driving a mirrored pair, one torso
# carrying every leg's pivots, one shaft coupler). Each pattern is a small,
# pure template rewrite: drop some bodies and redirect the edges that
# targeted them.


def _merge(name: str, tmpls: list[MechanismTemplate]) -> MechanismTemplate:
    return MechanismTemplate(
        name=name,
        bodies=[b for t in tmpls for b in t.bodies],
        connections=[c for t in tmpls for c in t.connections],
    )


def _rewrite_connections(connections, rewriter):
    """Map each endpoint via ``rewriter(idx, body, joint)``."""
    return [(rewriter(*a), rewriter(*b)) for a, b in connections]


def _fuse(
    tmpl: MechanismTemplate,
    olds: list[str],
    fused: BodyTemplate,
    rename: dict[tuple[str, str], str],
) -> MechanismTemplate:
    """Replace bodies ``olds`` by ``fused`` (at the first one's position).

    ``rename`` maps ``(old_body, old_joint)`` to the fused body's joint name.
    """
    drop = set(olds)
    bodies: list[BodyTemplate] = []
    for b in tmpl.bodies:
        if b.name == olds[0]:
            bodies.append(fused)
        elif b.name not in drop:
            bodies.append(b)

    def rewrite(idx, body, joint):
        if body in drop:
            return (idx, fused.name, rename[(body, joint)])
        return (idx, body, joint)

    return MechanismTemplate(
        name=tmpl.name, bodies=bodies,
        connections=_rewrite_connections(tmpl.connections, rewrite),
    )


def _suffixed_union(tmpl, olds, suffixes, keep_shared=()) -> tuple[list, dict]:
    """Joints of ``olds`` renamed ``{joint}{suffix}``; ``keep_shared`` joints appear once."""
    joints: list[JointTemplate] = []
    rename: dict[tuple[str, str], str] = {}
    for old, suffix in zip(olds, suffixes, strict=True):
        for j in tmpl.body(old).joints:
            name = j.name if j.name in keep_shared else f"{j.name}{suffix}"
            rename[(old, j.name)] = name
            if all(x.name != name for x in joints):
                joints.append(JointTemplate(name=name, pose_at=j.pose_at))
    return joints, rename


def combine_connectors(
    tmpl: MechanismTemplate, suffix_a: str, suffix_b: str, *, new_name: str = "conn",
) -> MechanismTemplate:
    """Fuse two legs' crank links into one rigid crank (``Project/main.py:494``).

    The cranks share the pivot O, so one rigid body driven by one shaft
    co-rotates both legs.
    """
    olds = [f"conn{suffix_a}", f"conn{suffix_b}"]
    joints, rename = _suffixed_union(tmpl, olds, [suffix_a, suffix_b])
    outline = tuple(
        (f"{p}{sfx}", f"{q}{sfx}")
        for old, sfx in zip(olds, (suffix_a, suffix_b), strict=True)
        for p, q in tmpl.body(old).outline
    )
    fused = BodyTemplate(
        name=new_name, joints=joints, outline=outline, color=tmpl.body(olds[0]).color,
    )
    return _fuse(tmpl, olds, fused, rename)


def fuse_couplers(
    tmpl: MechanismTemplate, suffixes: list[str], *, new_name: str = "coupler",
) -> MechanismTemplate:
    """Collapse per-leg shaft couplers into one body with an ``O{suffix}`` joint per leg."""
    olds = [f"coupler{s}" for s in suffixes]
    joints, rename = _suffixed_union(tmpl, olds, suffixes)
    primary = tmpl.body(olds[0])
    fused = BodyTemplate(
        name=new_name, joints=joints, color=primary.color, base_pose=primary.base_pose,
    )
    return _fuse(tmpl, olds, fused, rename)


def fuse_torsos(
    tmpl: MechanismTemplate, suffixes: list[str], *, new_name: str = "torso",
) -> MechanismTemplate:
    """One frame carrying every listed leg's A/B pivots and the shared crank centre O.

    2016 analogue: the single hulled ``torso`` of ``DoubleKlannLinkage``
    (``Project/main.py:587``).
    """
    olds = [f"torso{s}" for s in suffixes]
    joints, rename = _suffixed_union(tmpl, olds, suffixes, keep_shared=("O",))
    fused = BodyTemplate(name=new_name, joints=joints, color=tmpl.body(olds[0]).color)
    return _fuse(tmpl, olds, fused, rename)


# ---------------------------------------------------------------------------
# Assemblies
# ---------------------------------------------------------------------------


def _legs(name: str, legs: list[tuple[int, float]]) -> MechanismTemplate:
    return _merge(name, [
        build_klann_template(create_klann_geometry(o, ph), name_suffix=f"_leg{k}")
        for k, (o, ph) in enumerate(legs)
    ])


def build_multi_leg_template(n_legs: int) -> MechanismTemplate:
    """``n_legs`` same-chirality legs at phases ``2πk/n``, kept as separate bodies.

    Fabrication puts them on one crankshaft (their crank centres coincide).
    """
    if n_legs < 1:
        raise ValueError(f"n_legs must be >= 1, got {n_legs}")
    return _legs("klann_multi", [(1, 2.0 * math.pi * k / n_legs) for k in range(n_legs)])


def build_double_template() -> MechanismTemplate:
    """Mirrored pair sharing one crank and one torso (``DoubleKlannLinkage``)."""
    tmpl = _legs("klann_double", [(+1, 0.0), (-1, 0.0)])
    tmpl = combine_connectors(tmpl, "_leg0", "_leg1")
    tmpl = fuse_couplers(tmpl, ["_leg0", "_leg1"])
    return fuse_torsos(tmpl, ["_leg0", "_leg1"])


def build_double_decker_template() -> MechanismTemplate:
    """Two same-chirality legs 90° apart on one crankshaft (``DoubleDeckerKlannLinkage``)."""
    tmpl = _legs("klann_decker", [(+1, 0.0), (+1, math.pi / 2)])
    tmpl = fuse_couplers(tmpl, ["_leg0", "_leg1"])
    return fuse_torsos(tmpl, ["_leg0", "_leg1"])


def build_double_double_decker_template() -> MechanismTemplate:
    """Four-leg walker: two mirrored pairs (phases 0/π and π/2 / 3π/2) on one crankshaft.

    2016 analogue: ``DoubleDoubleDeckerKlannLinkage`` (``Project/main.py:835``).
    """
    tmpl = _legs("klann_quad", [
        (+1, 0.0), (-1, math.pi), (+1, math.pi / 2), (-1, 3 * math.pi / 2),
    ])
    tmpl = combine_connectors(tmpl, "_leg0", "_leg1", new_name="conn")
    tmpl = combine_connectors(tmpl, "_leg2", "_leg3", new_name="conn_upper")
    suffixes = ["_leg0", "_leg1", "_leg2", "_leg3"]
    tmpl = fuse_couplers(tmpl, suffixes)
    return fuse_torsos(tmpl, suffixes)


# ---------------------------------------------------------------------------
# Single-t mechanisms: a template frozen at t, optionally fabricated
# ---------------------------------------------------------------------------


def _realize(
    tmpl: MechanismTemplate,
    t: float,
    *,
    thickness: float | None,
    with_parts: bool,
    with_joinery: bool = True,
) -> Mechanism:
    mech = tmpl.freeze_at(t)
    if not with_parts:
        return mech
    from fabricate import fabricate, plan_for  # lazy: pulls in build123d

    spec = StackSpec() if thickness is None else StackSpec(pitch=thickness)
    return fabricate(mech, plan_for(tmpl, spec), joinery=with_joinery)


def build_klann_mechanism(
    solution: KlannSolution,
    t: float,
    *,
    thickness: float | None = None,
    name_suffix: str = "",
    with_parts: bool = True,
    with_joinery: bool = True,
) -> Mechanism:
    """One Klann leg at crank angle ``t``; with parts, fabricated as a standalone walker."""
    tmpl = build_klann_template(solution, name_suffix=name_suffix)
    return _realize(tmpl, t, thickness=thickness, with_parts=with_parts, with_joinery=with_joinery)


def build_multi_leg_mechanism(n_legs: int, t: float, **kw) -> Mechanism:
    return _realize(build_multi_leg_template(n_legs), t, **_kw(kw))


def build_double_klann(t: float, **kw) -> Mechanism:
    return _realize(build_double_template(), t, **_kw(kw))


def build_double_decker_klann(t: float, **kw) -> Mechanism:
    return _realize(build_double_decker_template(), t, **_kw(kw))


def build_double_double_decker_klann(t: float, **kw) -> Mechanism:
    return _realize(build_double_double_decker_template(), t, **_kw(kw))


def _kw(kw: dict[str, Any]) -> dict[str, Any]:
    return {
        "thickness": kw.get("thickness"),
        "with_parts": kw.get("with_parts", True),
        "with_joinery": kw.get("with_joinery", True),
    }


def KlannLinkage(name: str = "klann") -> Mechanism:
    """A single fabricated Klann leg at the reference time ``t=1.0``."""
    mech = build_klann_mechanism(create_klann_geometry(), t=1.0)
    mech.name = name
    return mech
