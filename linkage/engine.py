"""The engine: compass-and-ruler steps, :class:`Linkage`, the registry and one leg's
trajectory (:class:`LegSolution`). See the package docstring (:mod:`linkage`).
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
import sympy as sp

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


def crank_at(center: sp.Matrix, r, input: str = "t2", phase_deg=0) -> sp.Matrix:
    """A crankpin of radius ``r`` about ``center``, at the input angle ``input`` + ``phase_deg``."""
    a = sp.Symbol(input, real=True) + phase_deg * sp.pi / 180
    return center + r * sp.Matrix([sp.cos(a), sp.sin(a)])


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


MOTIONS = ("line", "path", "rotation", "translation_platform", "xy")


@dataclass(frozen=True)
class Output:
    """What a mechanism delivers, and what it promises.

    ``kind`` "point": the point ``name``, a joint of the link ``body``;
    "body": the link ``name`` itself. ``motion`` is one of :data:`MOTIONS`.
    ``frame`` names two joints of the output body, its origin then its x axis:
    where a later stage's frame would mount (a rotation's pivot comes first).

    Promises, enforced at the template stage (:meth:`Linkage.assert_output`):
    a ``translation_platform`` never rotates; ``straight = (from_deg, to_deg,
    tol_mm)``: over that crank range the output point stays within a band
    ``tol_mm`` wide about one straight line (a ``line`` must promise it);
    ``dwell = (tol_deg, crank_deg)``: a rotation stands still to ``±tol_deg``
    for at least ``crank_deg`` of the turn.
    """

    kind: str
    name: str
    motion: str
    frame: tuple[str, str]
    body: str = ""
    straight: tuple[float, float, float] | None = None
    dwell: tuple[float, float] | None = None

    def __post_init__(self):
        if (self.kind, bool(self.body)) not in (("point", True), ("body", False)):
            raise ValueError(f"output {self.name}: a point names its link (body=), a body doesn't")
        if self.motion not in MOTIONS or (self.motion == "line" and not self.straight):
            raise ValueError(f"output {self.name}: motion is one of {MOTIONS}; "
                             f"a line promises how straight")

    @property
    def link(self) -> str:
        return self.name if self.kind == "body" else self.body

    @property
    def point(self) -> str:
        """The point whose path shows the motion: the output point, a body's origin (a
        rotation's pin)."""
        if self.kind == "point":
            return self.name
        return self.frame[1] if self.motion == "rotation" else self.frame[0]


@dataclass(eq=False)
class Linkage:
    """One walking leg or one mechanism: its straight-line program, bodies and parameters.

    ``program(p)`` returns the steps given the parameter symbols ``p`` (a
    dict name -> Symbol). ``links`` maps each link body ``b<k>`` to its joints
    and the outline segments its laser-cut shape spans. ``frame`` lists the
    torso's joints (fixed pivots and ``O``), ``crank`` the crank's (``O``
    first, then the crankpin(s)). A walker's ``feet`` are ``(link, point)``
    pairs; a mechanism has none and declares its ``output``. ``inputs`` are
    the program's input angles, ``t`` first (a second one is placed with
    :func:`crank_at`). ``modules`` replaces :data:`MODULE_LEGS` entries for
    linkages whose natural unit differs (e.g. one that already carries a
    mirrored pair); a mechanism is one unit (``single``).
    """

    key: str
    name: str
    params: Mapping[str, sp.Expr]
    program: Callable[[Mapping[str, sp.Symbol]], Steps]
    links: Mapping[str, LinkSpec]
    frame: tuple[str, ...]
    feet: tuple[tuple[str, str], ...] = ()
    crank: tuple[str, ...] = ("O", "M")
    output: Output | None = None
    inputs: tuple[str, ...] = ("t",)
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
        if bool(self.feet) == (self.output is not None):
            raise ValueError(f"{self.key}: a walker has feet, a mechanism an output: one of them")
        if self.inputs not in (("t",), ("t", "t2")):
            raise ValueError(f"{self.key}: inputs are the crank angle t, and maybe t2")
        o = self.output
        if o is not None and not {o.point, *o.frame} <= set(self.links.get(o.link, ((),))[0]):
            raise ValueError(f"{self.key}: output {o.name} and its frame {o.frame} must be joints "
                             f"of its link {o.link!r}")

    @property
    def kind(self) -> str:
        """``walker`` (it has feet) or ``mechanism`` (it has an output)."""
        return "walker" if self.feet else "mechanism"

    # -- symbols and program ---------------------------------------------

    @cached_property
    def symbols(self) -> dict[str, sp.Symbol]:
        """Angles and non-positive defaults (coordinates) are real symbols, the rest positive."""
        return {k: sp.Symbol(k, real=True) if k in self.angles or v <= 0
                else sp.Symbol(k, positive=True) for k, v in self.params.items()}

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
        base = MODULE_LEGS if self.feet else {"single": MODULE_LEGS["single"]}
        return {**base, **self.modules}

    @cached_property
    def compiled(self) -> Callable[..., list]:
        """The program, compiled once: ``(t, [t2,] *params) -> [Ox, Oy, Ax, ...]``.

        Each step is lambdified on its own, over the inputs, the params and
        the points before it, and run in order: never substituted, so
        compiling costs the same at any depth.
        """
        head = [sp.Symbol(i, real=True) for i in self.inputs] + list(self.symbols.values())
        fns, before = [], []
        for name, expr in self.steps:
            fns.append(sp.lambdify([*head, *before], list(expr), modules="numpy", cse=True))
            before += list(P(name))

        def run(*args):
            flat: list = []
            for fn in fns:
                flat += fn(*args, *flat)
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
        values = self.values(params) if params else ()
        return LegSolution(int(orientation), float(phase), values, self.key)

    def check(self, params: Mapping[str, float] | None = None) -> list:
        """Every step of the program over one revolution, or over the torus of two
        inputs (see :class:`linkage.checks.StepCheck`)."""
        from linkage.checks import check_steps

        return list(check_steps(self.key, self.values(params)))

    def assert_assembles(self, params: Mapping[str, float] | None = None) -> list:
        """:meth:`check`, raising :class:`AssemblyError` at the first step that can't close."""
        steps = self.check(params)
        bad = next((s for s in steps if s.fails_deg is not None), None)
        if bad is not None:
            raise AssemblyError(f"{self.key}: {bad.describe()}")
        return steps

    def output_check(self, params: Mapping[str, float] | None = None):
        """How well a mechanism's output does its job over the cycle (see
        :class:`linkage.checks.OutputCheck`); :class:`AssemblyError` first if a loop can't
        close."""
        from linkage.checks import check_output

        if self.output is None:
            raise ValueError(f"{self.key} is a walker: it has feet, not an output")
        self.assert_assembles(params)
        return check_output(self.key, self.values(params))

    def assert_output(self, params: Mapping[str, float] | None = None):
        """:meth:`output_check`, raising :class:`OutputError` if the output breaks a promise."""
        if self.output is None:
            return None
        c = self.output_check(params)
        if c.broken:
            raise OutputError(f"{self.key}: {c.broken}")
        return c

    def variant(self, key: str, name: str, *, notes: str = "", source: str = "",
                **params) -> Linkage:
        """The same program with other default parameters (a published variant)."""
        self.values(params)                          # validate names
        return Linkage(
            key=key, name=name, params={**self.params, **params}, program=self.program,
            links=self.links, frame=self.frame, feet=self.feet, crank=self.crank,
            output=self.output, inputs=self.inputs, angles=self.angles, labels=self.labels,
            modules=self.modules,
            family=self.family or self.key, source=source or self.source, notes=notes,
        )


# ---------------------------------------------------------------------------
# Errors the template stage raises
# ---------------------------------------------------------------------------


class AssemblyError(ValueError):
    """A loop of the linkage can't close at some crank angle."""


class OutputError(AssemblyError):
    """A mechanism's output breaks what it promises (see :class:`Output`)."""


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


def available(kind: str | None = None) -> list[str]:
    """Registered keys, Klann first; ``kind`` (``walker`` / ``mechanism``) keeps those."""
    _load()
    return [k for k, lk in REGISTRY.items() if kind in (None, lk.kind)]


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

    @property
    def segments(self) -> dict[str, tuple[str, ...]]:
        """Body name -> the named joints along it (the crank is ``conn``)."""
        lk = self.linkage
        return {**{b: js for b, (js, _) in lk.links.items()}, "conn": lk.crank}

    def evaluate(self, ts, *others) -> dict[str, np.ndarray]:
        """Every named point over ``ts``: ``name -> (..., 2)`` array.

        ``others``: the other inputs' angles (``t2``), 0 when not given: a
        template, which has one time, holds them there.
        """
        lk = self.linkage
        ts = np.asarray(ts, dtype=float)
        tt = ts + self.phase
        sign = 1.0
        if self.orientation < 0:
            tt, sign = np.pi - tt, -1.0
        others = others or (0.0,) * (len(lk.inputs) - 1)
        if len(others) != len(lk.inputs) - 1:
            raise ValueError(f"{lk.key} takes inputs {lk.inputs}, got {1 + len(others)}")
        flat = lk.compiled(tt, *others, *(self.values or lk.defaults))
        cols = [np.broadcast_arrays(sign * flat[2 * i], flat[2 * i + 1], ts, *others)[:2]
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
        pts = self.evaluate(float(t_value))
        return {name: (float(v[0]), float(v[1])) for name, v in pts.items()}


# ---------------------------------------------------------------------------
# Parameters that only scale the linkage
# ---------------------------------------------------------------------------


def scale_params(lk: Linkage) -> tuple[str, ...]:
    """The parameters that only scale the linkage (every point doubles with them, like
    Klann's ``OA``): tuning them resizes the robot, its gait's shape stays."""
    ts = np.linspace(0.0, 2.0 * math.pi, 7)
    base = lk.solve().evaluate(ts)
    out = []
    for k, v in lk.params.items():
        with np.errstate(all="ignore"):
            pts = lk.solve(params={k: 2.0 * float(v)}).evaluate(ts)
        if k not in lk.angles and all(np.allclose(pts[p], 2.0 * base[p]) for p in lk.points):
            out.append(k)
    return tuple(out)
