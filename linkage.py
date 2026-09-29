"""Planar walking linkages as symbolic straight-line programs.

A :class:`Linkage` is one leg written as a short program: each step names a
point and gives it as a small sympy expression over earlier points' symbols
(:func:`P`), the crank angle :data:`t` and the linkage's parameters (exact
rationals: lengths in mm or in a drawing unit, angles in degrees). The
geometry helpers below (:func:`crank`, :func:`circle_x_circle`,
:func:`extend`, :func:`offset`, ...) keep each step to one construction a
draughtsman would make with compass and ruler.

Nothing is substituted until :meth:`Linkage.compiled`, which chains the
steps, runs CSE and lambdifies once per process. Every leg of every assembly
evaluates that one compiled program:

* a leg's ``phase`` is a time shift, ``t + phase``;
* its mirror image (``orientation=-1``) is the reflection ``x -> -x`` of the
  leg at crank angle ``pi - (t + phase)``. The crank centre ``O`` is the
  origin and every crankpin turns with ``t``, so the mirrored crankpin at
  ``t`` is the original's: mirrored legs ride the same crank.

Conventions every definition follows: ``O`` is the crank centre at the
origin, ``y`` points up (feet are the lowest points), the crank turns
counter-clockwise with ``t``. Links are bodies named ``b<k>`` (:func:`is_link`
in :mod:`stack`); ``torso`` carries the fixed pivots, ``conn`` the crank.

Definitions live in :mod:`linkages` and register themselves; see
:func:`get` / :func:`available`.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, field
from functools import cache, cached_property
from typing import Any

import numpy as np
import sympy as sp

from mechanism import BodyTemplate, JointTemplate, MechanismTemplate, translation_pose_at

# Optional profiler hook. ``viewer/bake_gltf.py`` sets this to its
# ``_Profiler`` instance at the start of a bake so the symbolic work shows up
# as labelled rows in the bake profile summary. The labels predate the other
# linkages and stay ``*_klann.*`` so downstream parsers keep working.
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
# Compass-and-ruler constructions
# ---------------------------------------------------------------------------

t = sp.Symbol("t", real=True)   # crank angle (radians)


def P(name: str) -> sp.Matrix:
    """Symbolic stand-in for an earlier step's point."""
    return sp.Matrix(sp.symbols(f"{name}x {name}y", real=True))


def xy(x, y) -> sp.Matrix:
    return sp.Matrix([x, y])


def rotate(v: sp.Matrix, degrees) -> sp.Matrix:
    a = degrees * sp.pi / 180
    return sp.Matrix([[sp.cos(a), -sp.sin(a)], [sp.sin(a), sp.cos(a)]]) * v


def polar(r, degrees) -> sp.Matrix:
    """``r`` at ``degrees`` counter-clockwise from +x."""
    a = degrees * sp.pi / 180
    return r * sp.Matrix([sp.cos(a), sp.sin(a)])


def crank(r) -> sp.Matrix:
    """The crankpin: radius ``r`` about ``O``, at the crank angle ``t``."""
    return r * sp.Matrix([sp.cos(t), sp.sin(t)])


def circle_x_circle(c1: sp.Matrix, r1, c2: sp.Matrix, r2, branch) -> sp.Matrix:
    """Intersection of two circles; ``branch`` = +1 / -1 picks the side of c1->c2.

    +1 is the left of the ray c1 -> c2 (counter-clockwise), -1 the right.
    """
    d = c2 - c1
    dd = d.dot(d)
    a = (r1**2 - r2**2 + dd) / (2 * dd)
    h = sp.sqrt(r1**2 / dd - a**2)
    return c1 + a * d + branch * h * sp.Matrix([-d[1], d[0]])


def extend(frm: sp.Matrix, through: sp.Matrix, length) -> sp.Matrix:
    """The point ``length`` beyond ``through`` on the ray frm -> through."""
    u = through - frm
    return through + u * length / sp.sqrt(u.dot(u))


def offset(frm: sp.Matrix, to: sp.Matrix, along, across) -> sp.Matrix:
    """A point rigid with the bar frm -> to: ``along`` it from ``frm``, ``across`` to its left."""
    u = to - frm
    u = u / sp.sqrt(u.dot(u))
    return frm + along * u + across * sp.Matrix([-u[1], u[0]])


def rigid(p: sp.Matrix, q: sp.Matrix, r_p, r_q, branch) -> sp.Matrix:
    """The third corner of a rigid triangle on p, q (``circle_x_circle``, named for intent)."""
    return circle_x_circle(p, r_p, q, r_q, branch)


# ---------------------------------------------------------------------------
# A linkage definition
# ---------------------------------------------------------------------------

Steps = list[tuple[str, sp.Matrix]]
LinkSpec = tuple[tuple[str, ...], tuple[tuple[str, str], ...]]   # joints, outline segments
LegList = tuple[tuple[int, float], ...]                         # (orientation, phase) per leg

# Leg modules (one side of the robot): each leg's chirality and default crank
# phase (radians). 2016 analogues: KlannLinkage, DoubleKlannLinkage,
# DoubleDeckerKlannLinkage, DoubleDoubleDeckerKlannLinkage.
MODULE_LEGS: dict[str, LegList] = {
    "single": ((+1, 0.0),),
    "double": ((+1, 0.0), (-1, 0.0)),
    "decker": ((+1, 0.0), (+1, math.pi / 2)),
    "quad": ((+1, 0.0), (-1, math.pi), (+1, math.pi / 2), (-1, 3 * math.pi / 2)),
}


@dataclass(eq=False)
class Linkage:
    """One walking leg: its straight-line program, bodies and parameters.

    ``program(p)`` returns the steps given the parameter symbols ``p`` (a
    dict name -> Symbol). ``links`` maps each link body ``b<k>`` to its joints
    and the outline segments its laser-cut shape spans. ``frame`` lists the
    torso's joints (fixed pivots and ``O``), ``crank`` the crank's (``O``
    first, then the crankpin(s)). ``feet`` are ``(link, point)`` pairs.
    ``modules`` replaces :data:`MODULE_LEGS` entries for linkages whose natural
    unit differs (e.g. one that already carries a mirrored pair).
    """

    key: str
    name: str
    params: Mapping[str, sp.Expr]
    program: Callable[[Mapping[str, sp.Symbol]], Steps]
    links: Mapping[str, LinkSpec]
    frame: tuple[str, ...]
    feet: tuple[tuple[str, str], ...]
    crank: tuple[str, ...] = ("O", "M")
    angles: frozenset[str] = frozenset()
    labels: Mapping[str, str] = field(default_factory=dict)
    modules: Mapping[str, LegList] = field(default_factory=dict)
    family: str = ""
    source: str = ""
    notes: str = ""

    def __post_init__(self):
        bad = [b for b in self.links if not (b[0] == "b" and b[1:].isdigit())]
        if bad:
            raise ValueError(f"{self.key}: link bodies must be named b<k>, got {bad}")
        if self.frame.count("O") != 1 or self.crank[0] != "O":
            raise ValueError(f"{self.key}: the torso and the crank both carry O (crank first)")
        joints = {j for js, _ in self.links.values() for j in js}
        missing = [(b, j) for b, j in self.feet if b not in self.links or j not in self.links[b][0]]
        if missing:
            raise ValueError(f"{self.key}: feet {missing} are not joints of their links")
        loose = [j for j in (*self.frame, *self.crank[1:]) if j != "O" and j not in joints]
        if loose:
            raise ValueError(f"{self.key}: frame/crank joints {loose} carry no link")

    # -- symbols and program ---------------------------------------------

    @cached_property
    def symbols(self) -> dict[str, sp.Symbol]:
        return {
            k: sp.Symbol(k, real=True) if k in self.angles else sp.Symbol(k, positive=True)
            for k in self.params
        }

    @cached_property
    def steps(self) -> Steps:
        return self.program(self.symbols)

    @property
    def points(self) -> tuple[str, ...]:
        return tuple(name for name, _ in self.steps)

    @property
    def defaults(self) -> tuple[float, ...]:
        return tuple(float(v) for v in self.params.values())

    @property
    def leg_modules(self) -> dict[str, LegList]:
        return {**MODULE_LEGS, **self.modules}

    def closed_form(self) -> dict[str, sp.Matrix]:
        """Fully substituted point expressions (for inspection and differentiation)."""
        env: dict[sp.Symbol, sp.Expr] = {}
        out: dict[str, sp.Matrix] = {}
        for name, expr in self.steps:
            e = expr.xreplace(env)
            out[name] = e
            x, y = P(name)
            env[x], env[y] = e[0], e[1]
        return out

    @cached_property
    def compiled(self) -> Callable[..., list]:
        """The program, compiled once: ``(t, *params) -> [Ox, Oy, Ax, ...]``.

        Each step is lambdified on its own, over ``t``, the params and the
        points before it, and run in order: never substituted, so compiling
        costs the same at any depth.
        """
        with _maybe_timed("4.1b_klann.lambdify"):
            params, fns, before = list(self.symbols.values()), [], []
            for name, expr in self.steps:
                fns.append(sp.lambdify([t, *params, *before], list(expr), modules="numpy",
                                       cse=True))
                before += list(P(name))

        def run(tt, *values):
            flat: list = []
            for fn in fns:
                flat += fn(tt, *values, *flat)
            return flat

        return run

    def values(self, overrides: Mapping[str, float] | None = None) -> tuple[float, ...]:
        """Parameter values in ``params`` order, the defaults unless overridden."""
        overrides = dict(overrides or {})
        unknown = set(overrides) - set(self.params)
        if unknown:
            raise KeyError(f"unknown {self.key} parameters {sorted(unknown)}; "
                           f"have {list(self.params)}")
        return tuple(float(overrides.get(k, v)) for k, v in self.params.items())

    def solve(self, orientation: int = 1, phase: float = 0.0,
              params: Mapping[str, float] | None = None) -> LegSolution:
        """One leg of this linkage (cheap: compiles once, then records parameters)."""
        with _maybe_timed("4.1a_klann.create_geometry"):
            self.compiled  # noqa: B018 - compile (and time it) on first use
            values = self.values(params) if params else ()
            return LegSolution(int(orientation), float(phase), values, self.key)

    def check(self, params: Mapping[str, float] | None = None) -> list[StepCheck]:
        """Every step of the program over one revolution (see :class:`StepCheck`)."""
        return list(_check(self.key, self.values(params)))

    def assert_assembles(self, params: Mapping[str, float] | None = None) -> list[StepCheck]:
        """:meth:`check`, raising :class:`AssemblyError` at the first step that can't close."""
        steps = self.check(params)
        bad = next((s for s in steps if s.fails_deg is not None), None)
        if bad is not None:
            raise AssemblyError(f"{self.key}: {bad.describe()}")
        return steps

    def variant(self, key: str, name: str, *, notes: str = "", source: str = "",
                **params) -> Linkage:
        """The same program with other default parameters (a published variant)."""
        self.values(params)                          # validate names
        return Linkage(
            key=key, name=name, params={**self.params, **params}, program=self.program,
            links=self.links, frame=self.frame, feet=self.feet, crank=self.crank,
            angles=self.angles, labels=self.labels, modules=self.modules,
            family=self.family or self.key, source=source or self.source, notes=notes,
        )


# ---------------------------------------------------------------------------
# Stage check: does every loop close, and how well?
# ---------------------------------------------------------------------------

TOGGLE_DEG = 15.0     # a transmission angle this close to 0° or 180° is flagged


class AssemblyError(ValueError):
    """A loop of the linkage can't close at some crank angle."""


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

    @property
    def toggles(self) -> bool:
        return self.angle_deg is not None and min(self.angle_deg[0],
                                                  180 - self.angle_deg[1]) < TOGGLE_DEG

    def describe(self) -> str:
        if self.kind != "closure":
            return f"{self.point}: {self.kind} ({', '.join(self.refs) or 'no earlier points'})"
        a, b = self.refs
        if self.radii is None:
            return (f"{self.point} can't be placed at any crank angle: "
                    f"its bars from {a} and {b} never meet")
        r1, r2 = self.radii
        where = f"bars {a}-{self.point} {r1:.1f} mm and {b}-{self.point} {r2:.1f} mm"
        if self.fails_deg is not None:
            lo, hi = self.fails_deg
            return (f"{self.point} can't be placed for {self.fail_fraction:.0%} of the cycle "
                    f"(crank angles {lo:.0f}°..{hi:.0f}°): {where} miss each other by up to "
                    f"{-self.margin_mm:.2f} mm (worst at {self.worst_deg:.0f}°)")
        lo, hi = self.angle_deg
        flag = "; near toggle" if self.toggles else ""
        return (f"{self.point}: {where} close with {self.margin_mm:.2f} mm to spare "
                f"(worst at {self.worst_deg:.0f}°), transmission angle {lo:.0f}°..{hi:.0f}°{flag}")


@cache
def _check(key: str, values: tuple[float, ...], n: int = 720) -> tuple[StepCheck, ...]:
    lk = get(key)
    ts = 2.0 * math.pi * np.arange(n) / n
    with np.errstate(all="ignore"):
        pts = LegSolution(1, 0.0, values, key).evaluate(ts)
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
            (float(ang[ok].min()), float(ang[ok].max())), fails.size / n,
        ))
    return tuple(out)


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

REGISTRY: dict[str, Linkage] = {}
DEFAULT = "klann"


def register(linkage: Linkage) -> Linkage:
    if linkage.key in REGISTRY:
        raise ValueError(f"linkage {linkage.key!r} registered twice")
    REGISTRY[linkage.key] = linkage
    return linkage


def _load() -> None:
    import linkages  # noqa: F401 - definitions register on import


def get(key: str) -> Linkage:
    _load()
    try:
        return REGISTRY[key]
    except KeyError:
        raise KeyError(f"unknown linkage {key!r}; have {sorted(REGISTRY)}") from None


def available() -> list[str]:
    _load()
    return list(REGISTRY)


# ---------------------------------------------------------------------------
# One leg's trajectory
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LegSolution:
    """One leg: a linkage's compiled program at a chirality, phase and parameters.

    ``orientation=+1`` is the leg as defined, ``-1`` its mirror image. The
    crank angle of this leg is ``t + phase``. ``values`` are the parameters in
    the linkage's ``params`` order; ``()`` means its defaults.
    """

    orientation: int = 1
    phase: float = 0.0
    values: tuple[float, ...] = ()
    linkage_key: str = DEFAULT

    @property
    def linkage(self) -> Linkage:
        return get(self.linkage_key)

    @property
    def proportions(self) -> dict[str, float]:
        lk = self.linkage
        return dict(zip(lk.params, self.values or lk.defaults, strict=True))

    @cached_property
    def points(self) -> dict[str, sp.Matrix]:
        """Closed-form ``(x, y)`` per named point, in ``t`` (slow; for inspection)."""
        lk = self.linkage
        tt = t + self.phase if self.orientation > 0 else sp.pi - (t + self.phase)
        subs = {t: tt} | {lk.symbols[k]: v for k, v in self.proportions.items()}
        sign = 1 if self.orientation > 0 else -1
        return {name: sp.Matrix([sign * e[0], e[1]]).xreplace(subs)
                for name, e in lk.closed_form().items()}

    @property
    def segments(self) -> dict[str, tuple[str, ...]]:
        """Body name -> the named joints along it (the crank is ``conn``)."""
        lk = self.linkage
        return {**{b: js for b, (js, _) in lk.links.items()}, "conn": lk.crank}

    def evaluate(self, ts) -> dict[str, np.ndarray]:
        """Every named point over ``ts``: ``name -> (..., 2)`` array."""
        lk = self.linkage
        ts = np.asarray(ts, dtype=float)
        tt = ts + self.phase
        sign = 1.0
        if self.orientation < 0:
            tt, sign = np.pi - tt, -1.0
        flat = lk.compiled(tt, *(self.values or lk.defaults))
        cols = [np.broadcast_arrays(sign * flat[2 * i], flat[2 * i + 1], ts)[:2]
                for i in range(len(lk.points))]
        return {name: np.stack(c, axis=-1) for name, c in zip(lk.points, cols, strict=True)}

    @cached_property
    def callables(self) -> dict[str, Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]]]:
        """One numpy callable ``(ts,) -> (xs, ys)`` per named point."""

        def _fn(name: str):
            def f(ts):
                xy_ = self.evaluate(ts)[name]
                return xy_[..., 0], xy_[..., 1]
            return f

        return {name: _fn(name) for name in self.linkage.points}

    def joints_at(self, t_value: float) -> dict[str, tuple[float, float]]:
        """Evaluate every named point at ``t_value`` and return float (x, y) pairs."""
        with _maybe_timed("4.1c_klann.joints_at_eval"):
            pts = self.evaluate(float(t_value))
            return {name: (float(v[0]), float(v[1])) for name, v in pts.items()}


# ---------------------------------------------------------------------------
# One leg as a kinematic template
# ---------------------------------------------------------------------------
#
# Every joint sits at z = 0 in its body's frame: the mechanism is planar, and
# which Z slot each part occupies is a fabrication decision made by
# :mod:`stack` (see :mod:`fabricate`). Joint XY comes straight from the
# compiled symbolic program.


def leg_bodies(lk: Linkage) -> list[tuple[str, tuple[str, ...], tuple[tuple[str, str], ...], str]]:
    """``(body, joints, outline, colour)`` in template order: coupler, links, crank, torso."""
    crank_outline = tuple(("O", pin) for pin in lk.crank[1:])
    return [
        ("coupler", ("O",), (), "blue"),
        *((b, js, outline, "green") for b, (js, outline) in lk.links.items()),
        ("conn", lk.crank, crank_outline, "orange"),
        ("torso", lk.frame, (), "yellow"),
    ]


def leg_connections(lk: Linkage) -> list[tuple[str, str, str]]:
    """``(body_a, body_b, joint)``: bodies sharing a joint name are pinned there.

    Each joint's bodies are chained in template order; the shaft coupler hangs
    off the crank at ``O``.
    """
    order = [(b, js) for b, js, _, _ in leg_bodies(lk) if b != "coupler"]
    out: list[tuple[str, str, str]] = []
    for j in dict.fromkeys(j for _, js in order for j in js):
        on = [b for b, js in order if j in js]
        out += [(a, b, j) for a, b in zip(on, on[1:], strict=False)]
    out.append(("conn", "coupler", "O"))
    return out


def build_leg_template(solution: LegSolution, *, name_suffix: str = "") -> MechanismTemplate:
    """One leg whose joint poses are closures over the compiled program."""
    with _maybe_timed("4.1d_klann.assemble_leg"):
        lk = solution.linkage
        callables = solution.callables
        bodies = [
            BodyTemplate(
                name=f"{name}{name_suffix}",
                joints=[JointTemplate(name=j, pose_at=translation_pose_at(callables[j], z=0.0))
                        for j in joints],
                outline=outline,
                color=color,
            )
            for name, joints, outline, color in leg_bodies(lk)
        ]
        connections = [
            ((0, f"{a}{name_suffix}", j), (0, f"{b}{name_suffix}", j))
            for a, b, j in leg_connections(lk)
        ]
        return MechanismTemplate(name=f"{lk.key}{name_suffix}", bodies=bodies,
                                 connections=connections)


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
        bodies=[b for t_ in tmpls for b in t_.bodies],
        connections=[c for t_ in tmpls for c in t_.connections],
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
    """One frame carrying every listed leg's fixed pivots and the shared crank centre O.

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


def legs_template(name: str, legs: Sequence[tuple[int, float]],
                  params: Mapping[str, float] | None = None,
                  linkage: str = DEFAULT) -> MechanismTemplate:
    """Legs kept as separate bodies, suffixed ``_leg<k>``."""
    lk = get(linkage)
    return _merge(name, [
        build_leg_template(lk.solve(o, ph, params), name_suffix=f"_leg{k}")
        for k, (o, ph) in enumerate(legs)
    ])


def module_legs(module: str, linkage: str = DEFAULT) -> LegList:
    mods = get(linkage).leg_modules
    if module not in mods:
        raise ValueError(f"unknown module {module!r}; have {sorted(mods)}")
    return mods[module]


def build_module_template(
    module: str = "quad",
    phases: Sequence[float] | None = None,
    proportions: Mapping[str, float] | None = None,
    linkage: str = DEFAULT,
) -> MechanismTemplate:
    """One side's legs on one crankshaft and one frame.

    ``phases`` (radians, one per leg) replaces the module's default crank
    phases; ``proportions`` overrides some of the linkage's parameters. The
    template's ``meta`` records all of it, so caches keyed on it stay honest.
    """
    lk = get(linkage)
    lk.assert_assembles(proportions)
    legs = module_legs(module, linkage)
    if phases is not None:
        if len(phases) != len(legs):
            raise ValueError(f"{module} has {len(legs)} legs, got {len(phases)} phases")
        legs = tuple((o, float(ph)) for (o, _), ph in zip(legs, phases, strict=True))
    if len(legs) == 1:
        (o, ph), = legs
        tmpl = build_leg_template(lk.solve(o, ph, proportions))
    else:
        tmpl = legs_template(f"{lk.key}_{module}", legs, proportions, linkage)
        suffixes = [f"_leg{k}" for k in range(len(legs))]
        if module == "double":
            tmpl = combine_connectors(tmpl, "_leg0", "_leg1")
        elif module == "quad":
            tmpl = combine_connectors(tmpl, "_leg0", "_leg1", new_name="conn")
            tmpl = combine_connectors(tmpl, "_leg2", "_leg3", new_name="conn_upper")
        tmpl = fuse_couplers(tmpl, suffixes)
        tmpl = fuse_torsos(tmpl, suffixes)
    tmpl.meta = {
        "linkage": lk.key,
        "module": module,
        "phases": tuple(ph for _, ph in legs),
        "proportions": lk.values(proportions),
    }
    return tmpl


def feet_of(tmpl_or_mech) -> list[tuple[str, str]]:
    """``(body, joint)`` of every foot in a side template or a fabricated robot.

    Reads the linkage from ``meta`` and matches foot links by class (so
    ``L.b4_leg2`` is a Klann foot).
    """
    from stack import body_class  # lazy: stack imports numpy-heavy bits only

    lk = get(tmpl_or_mech.meta.get("linkage", DEFAULT))
    return [(b.name, j) for b in tmpl_or_mech.bodies for cls, j in lk.feet
            if body_class(b.name) == cls]
