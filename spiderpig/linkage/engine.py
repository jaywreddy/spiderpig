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


@dataclass(frozen=True)
class Module:
    """A leg module: one side's legs and which of them share one rigid crank.

    ``legs``: each leg's chirality and default crank phase (radians).
    ``cranks``: groups of leg indices whose crank links are one body (a
    mirrored pair rides one crankpin; two legs out of phase share a
    crankshaft with a crankpin each). A leg in no group keeps its own crank
    body; the crank construction joins the crank bodies of a side.
    """

    legs: LegList
    cranks: tuple[tuple[int, ...], ...] = ()

    def __post_init__(self):
        seen = [i for g in self.cranks for i in g]
        if len(set(seen)) != len(seen) or any(i >= len(self.legs) for i in seen):
            raise ValueError(f"crank groups {self.cranks} must name distinct legs of "
                             f"{len(self.legs)}")


# Leg modules (one side of the robot). 2016 analogues: KlannLinkage,
# DoubleKlannLinkage, DoubleDeckerKlannLinkage, DoubleDoubleDeckerKlannLinkage.
MODULES: dict[str, Module] = {
    "single": Module(((+1, 0.0),)),
    "double": Module(((+1, 0.0), (-1, 0.0)), cranks=((0, 1),)),
    "decker": Module(((+1, 0.0), (+1, math.pi / 2))),
    "quad": Module(((+1, 0.0), (-1, math.pi), (+1, math.pi / 2), (-1, 3 * math.pi / 2)),
                   cranks=((0, 1), (2, 3))),
}
MODULE_LEGS: dict[str, LegList] = {k: m.legs for k, m in MODULES.items()}

KLANN_QUAD = Module(((+1, 0.0), (-1, 0.0), (+1, math.pi), (-1, math.pi)), cranks=((0, 1), (2, 3)))
"""The Klann variants' quad (the user's call, 2026-10-04; the diywalkers variants of
:mod:`linkages.klann_variants`, not the demo ``klann``, whose quad finds no plan this way
with standoff pillars): crank phases 0, 0, 180, 180, each
mirrored pair on one crankpin, the two pairs half a turn apart (two crankpins, not the
generic quad's four: a thinner crank, and the phasing the Klann audits used)."""


MOTIONS = ("line", "path", "rotation", "translation_platform", "xy")

TOUR_ROWS = 64
"""The rows of an arm's tour (:meth:`Linkage.tour`): its first input swept across the box
this many times, the second stepped between (even: the tour ends on the edge it starts)."""


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
    :func:`crank_at`). ``modules`` replaces :data:`MODULES` entries for
    linkages whose natural unit differs (e.g. one that already carries a
    mirrored pair): a :class:`Module`, or just its legs (no shared crank); a
    mechanism is one unit (``single``).
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
    # an arm's inputs turn over a range, not full turns: input -> (lo, hi) degrees; each
    # drives a link about a frame joint, ``drives`` input -> (link, joint). An arm has no
    # crank (``crank=()``): its template's one time walks a closed tour over the inputs' box
    # (:meth:`tour`), so every stage that samples a cycle samples the workspace
    ranges: Mapping[str, tuple[float, float]] = field(default_factory=dict)
    # an arm's relative limit: (lo, hi) degrees of its first input less its second (the
    # MT4's elbow stops, a parallelogram kept off its dead points); None: the box is the
    # ranges' rectangle (:meth:`inputs_at`)
    band: tuple[float, float] | None = None
    drives: Mapping[str, tuple[str, str]] = field(default_factory=dict)
    angles: frozenset[str] = frozenset()
    labels: Mapping[str, str] = field(default_factory=dict)
    modules: Mapping[str, Module | LegList] = field(default_factory=dict)
    default_module: str = ""        # the module a design gets when none is asked for ("": quad)
    family: str = ""
    source: str = ""
    notes: str = ""

    def __post_init__(self):
        bad = [b for b in self.links if not (b[0] == "b" and b[1:].isdigit())]
        if bad:
            raise ValueError(f"{self.key}: link bodies must be named b<k>, got {bad}")
        if self.frame.count("O") != 1 or (self.crank and self.crank[0] != "O"):
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
        if self.default_module and self.default_module not in self.modules_of:
            raise ValueError(f"{self.key}: default module {self.default_module!r} is not one of "
                             f"its modules {list(self.modules_of)}")
        self._check_inputs(joints)
        o = self.output
        if o is not None and not {o.point, *o.frame} <= set(self.links.get(o.link, ((),))[0]):
            raise ValueError(f"{self.key}: output {o.name} and its frame {o.frame} must be joints "
                             f"of its link {o.link!r}")

    def _check_inputs(self, joints: set[str]) -> None:
        """A cranked linkage's inputs are ``t`` and maybe ``t2`` (full turns); an arm's
        (no crank) each turn over a range and drive a link about a frame joint."""
        if self.crank:
            if self.inputs not in (("t",), ("t", "t2")) or self.ranges or self.drives:
                raise ValueError(f"{self.key}: inputs are the crank angle t, and maybe t2")
            return
        names = list(self.inputs)
        if not names or len(set(names)) != len(names) or \
                not all(isinstance(n, str) and n.isidentifier() for n in names):
            raise ValueError(f"{self.key}: an arm names its inputs, each once")
        if not 1 <= len(names) <= 2:
            raise ValueError(f"{self.key}: an arm's tour covers one or two inputs, got "
                             f"{len(names)}")
        if set(self.ranges) != set(names) or set(self.drives) != set(names):
            raise ValueError(f"{self.key}: every input of an arm has a range and a drive "
                             f"(inputs {names}, ranges {sorted(self.ranges)}, drives "
                             f"{sorted(self.drives)})")
        for n, (lo, hi) in self.ranges.items():
            if not lo < hi or hi - lo > 360:
                raise ValueError(f"{self.key}: input {n}'s range {lo}..{hi} deg")
        for n, (link, about) in self.drives.items():
            if link not in self.links or about not in self.links[link][0] or \
                    about not in self.frame:
                raise ValueError(f"{self.key}: input {n} drives {link!r} about {about!r}: a "
                                 "link turning on a frame joint")
        if self.output is None or self.feet:
            raise ValueError(f"{self.key}: an arm has an output, no feet")

    @property
    def kind(self) -> str:
        """``walker`` (it has feet), ``arm`` (no crank: its inputs turn over ranges) or
        ``mechanism`` (a crank, and an output)."""
        if self.feet:
            return "walker"
        return "mechanism" if self.crank else "arm"

    @property
    def ranged(self) -> bool:
        """Its inputs turn over ranges, not full circles: no crank and no phases, a
        :meth:`tour` over the inputs' box for a cycle, each input a drive turning a link
        about a frame joint (an arm). What a stage tests to treat the inputs as drives;
        :attr:`kind` is only the label. (A ranged linkage has no feet: its validation.)"""
        return not self.crank

    @property
    def walks(self) -> bool:
        """It has feet on a floor: a gait, the walk model, the sim's ground loads and
        MuJoCo's contacts apply (a walker). What a stage tests for the floor; :attr:`kind`
        is only the label."""
        return bool(self.feet)

    # -- an arm's tour --------------------------------------------------------

    def tour(self, ts) -> tuple[np.ndarray, ...]:
        """The inputs (radians) at each of ``ts`` (radians, one turn) of an arm's tour: a
        closed serpentine over the inputs' box, :data:`TOUR_ROWS` rows of the first input
        swept across the second's range and back along its edge, at constant speed in the
        box normalised to unit sides (one input: there and back). Every stage that samples
        a crank cycle samples the workspace by it; the rows' spacing is
        :meth:`tour_lateral`'s."""
        if not self.ranged:
            raise ValueError(f"{self.key} is a {self.kind}: its inputs turn full circles")
        u = np.mod(np.asarray(ts, dtype=float), 2 * math.pi) / (2 * math.pi)
        names = self.inputs
        if len(names) == 1:
            a = 1.0 - np.abs(2.0 * u - 1.0)                         # 0 -> 1 -> 0
            return (self._denorm(names[0], a),)
        rows = TOUR_ROWS
        step = 1.0 / (rows - 1)
        length = rows * 1.0 + (rows - 1) * step + 1.0               # rows, steps, return
        s = u * length
        a = np.zeros_like(s)
        b = np.zeros_like(s)
        pos = 0.0
        for r in range(rows):
            on = (s >= pos) & (s < pos + 1.0)
            f = s[on] - pos
            a[on] = f if r % 2 == 0 else 1.0 - f
            b[on] = r * step
            pos += 1.0
            if r < rows - 1:
                on = (s >= pos) & (s < pos + step)
                a[on] = 1.0 if r % 2 == 0 else 0.0
                b[on] = r * step + (s[on] - pos)
                pos += step
        on = s >= pos                                               # back along a = 0
        a[on] = 0.0
        b[on] = 1.0 - (s[on] - pos)
        return self.inputs_at(a, b)

    def inputs_at(self, *u) -> tuple[np.ndarray, ...]:
        """The inputs (radians) at fractions ``u`` (0..1, one array each) of the box: each
        input across its range; with a :attr:`band`, the second across its range and the
        first across what the band leaves of its range at that second's angle."""
        if self.band is None or len(self.inputs) != 2:
            return tuple(self._denorm(n, x) for n, x in zip(self.inputs, u, strict=True))
        (la, ha), (lb, hb) = (self.ranges[n] for n in self.inputs)
        b = lb + (hb - lb) * np.clip(np.asarray(u[1], dtype=float), 0.0, 1.0)
        lo = np.maximum(la, b + self.band[0])
        hi = np.minimum(ha, b + self.band[1])
        a = lo + (hi - lo) * np.clip(np.asarray(u[0], dtype=float), 0.0, 1.0)
        return np.radians(a), np.radians(b)

    def in_band(self, *q) -> bool:
        """Whether the inputs ``q`` (radians) are inside the ranges and the band."""
        for n, v in zip(self.inputs, q, strict=True):
            lo, hi = self.ranges[n]
            if not math.radians(lo) - 1e-9 <= v <= math.radians(hi) + 1e-9:
                return False
        if self.band is None or len(q) != 2:
            return True
        d = math.degrees(q[0] - q[1])
        return self.band[0] - 1e-6 <= d <= self.band[1] + 1e-6

    def _denorm(self, name: str, u: np.ndarray) -> np.ndarray:
        lo, hi = self.ranges[name]
        return np.radians(lo + (hi - lo) * np.clip(u, 0.0, 1.0))

    def box(self, n: int = 61) -> tuple[np.ndarray, ...]:
        """An ``n``-per-side grid over an arm's inputs' box (radians, raveled; within its
        :attr:`band`, if any: :meth:`inputs_at`)."""
        u = np.linspace(0.0, 1.0, n)
        grid = np.meshgrid(*([u] * len(self.inputs)), indexing="ij")
        return self.inputs_at(*(g.ravel() for g in grid))

    def tour_lateral(self, values: tuple[float, ...] = ()) -> dict[str, float]:
        """How far each point moves between two neighbouring rows of the tour (mm): what a
        stage that samples the tour adds to its bound between samples, so its guarantee
        covers every pose in the box, not only the tour's (0 with one input)."""
        if not self.ranged or len(self.inputs) == 1:
            return dict.fromkeys(self.points, 0.0)
        n = 4 * TOUR_ROWS + 1
        ua = np.linspace(0.0, 1.0, n)
        ub = np.linspace(0.0, 1.0, 4 * (TOUR_ROWS - 1) + 1)
        UA, UB = (g.ravel() for g in np.meshgrid(ua, ub, indexing="ij"))
        sol = LegSolution(1, 0.0, values, self.key)
        p0 = sol.at(*self.inputs_at(UA, UB))
        p1 = sol.at(*self.inputs_at(UA, np.minimum(UB + 1.0 / (TOUR_ROWS - 1), 1.0)))
        return {k: float(np.linalg.norm(p1[k] - p0[k], axis=-1).max()) for k in self.points}

    # -- symbols and program ---------------------------------------------

    @property
    def signed(self) -> frozenset[str]:
        """The parameters that may be zero or negative: the angles and the coordinates (a
        parameter whose default is not positive: a fixed pivot's x or y, an offset). Their
        symbols are real; every other parameter is a length, its symbol positive
        (:attr:`symbols`), and a value of 0 or less for it is an error wherever a design is
        made (:mod:`spiderpig.config`, :mod:`spiderpig.spec`)."""
        return frozenset(k for k, v in self.params.items() if k in self.angles or v <= 0)

    @cached_property
    def symbols(self) -> dict[str, sp.Symbol]:
        """Angles and non-positive defaults (coordinates) are real symbols, the rest positive."""
        signed = self.signed
        return {k: sp.Symbol(k, real=True) if k in signed else sp.Symbol(k, positive=True)
                for k in self.params}

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
        """Module name -> its legs."""
        return {k: m.legs for k, m in self.modules_of.items()}

    @property
    def modules_of(self) -> dict[str, Module]:
        """Module name -> :class:`Module` (the defaults, a mechanism's ``single`` only,
        and this linkage's own)."""
        base = MODULES if self.feet else {"single": MODULES["single"]}
        own = {k: m if isinstance(m, Module) else Module(tuple(m))
               for k, m in self.modules.items()}
        return {**base, **own}

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
            coords = list(expr)  # pyright: ignore[reportArgumentType]  # sympy: Matrix untyped iter
            fns.append(sp.lambdify([*head, *before], coords, modules="numpy", cse=True))
            before += list(P(name))  # pyright: ignore[reportArgumentType]  # same Matrix iter

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
        from spiderpig.linkage.checks import check_steps

        return list(check_steps(self.key, self.values(params)))

    def assert_assembles(self, params: Mapping[str, float] | None = None) -> list:
        """:meth:`check`, raising :class:`AssemblyError` at the first step that can't close."""
        steps = self.check(params)
        bad = next((s for s in steps if s.fails_deg is not None), None)
        if bad is not None:
            raise AssemblyError(f"{self.key}: {bad.describe()}")
        if self.ranged:
            from spiderpig.linkage.checks import check_drives, check_rigid
            from spiderpig.linkage.constraints import LOCK_COND, check_conditioning

            values = self.values(params)
            soft = check_rigid(self.key, values)
            if soft:
                link, a, b, var = soft[0]
                raise AssemblyError(f"{self.key}: {link}'s joints {a} and {b} drift "
                                    f"{var:.3g} mm apart over the inputs' range: not rigid")
            slip = check_drives(self.key, values)
            if slip:
                name, link, var = slip[0]
                raise AssemblyError(f"{self.key}: input {name} doesn't turn {link} with it "
                                    f"(off by up to {var:.3g} deg over its range)")
            cond, at = check_conditioning(self, values)
            if not cond <= LOCK_COND:
                where = ", ".join(f"{k} {v:.0f} deg" for k, v in
                                  zip(self.inputs, at or (), strict=False))
                raise AssemblyError(f"{self.key}: the linkage nearly locks at {where}: a "
                                    f"parallelogram or loop flattens (condition {cond:.3g}, "
                                    f"limit {LOCK_COND:g}); move a pivot away from the line")
        return steps

    def output_check(self, params: Mapping[str, float] | None = None):
        """How well a mechanism's output does its job over the cycle (see
        :class:`linkage.checks.OutputCheck`); :class:`AssemblyError` first if a loop can't
        close."""
        from spiderpig.linkage.checks import check_output

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
            output=self.output, inputs=self.inputs, ranges=self.ranges, drives=self.drives,
            angles=self.angles, labels=self.labels, modules=self.modules,
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
DEFAULT = "strider"            # the project's walker; Klann stays the wobbly demo


def register(linkage: Linkage) -> Linkage:
    if linkage.key in REGISTRY:
        raise ValueError(f"linkage {linkage.key!r} registered twice")
    REGISTRY[linkage.key] = linkage
    return linkage


def _load() -> None:
    from spiderpig import linkages  # noqa: F401 - definitions register on import


def get(key: str) -> Linkage:
    _load()
    try:
        return REGISTRY[key]
    except KeyError:
        raise KeyError(f"unknown linkage {key!r}; have {sorted(REGISTRY)}") from None


def available(kind: str | None = None) -> list[str]:
    """Registered keys, the default first; ``kind`` (``walker`` / ``mechanism`` / ``arm``)
    keeps those."""
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
        """Body name -> the named joints along it (the crank is ``conn``; an arm has none)."""
        lk = self.linkage
        crank = {"conn": lk.crank} if lk.crank else {}
        return {**{b: js for b, (js, _) in lk.links.items()}, **crank}

    def evaluate(self, ts, *others) -> dict[str, np.ndarray]:
        """Every named point over ``ts``: ``name -> (..., 2)`` array.

        ``others``: the other inputs' angles (``t2``), 0 when not given: a
        template, which has one time, holds them there.
        """
        lk = self.linkage
        ts = np.asarray(ts, dtype=float)
        if lk.ranged and not others:     # an arm's one time: its tour (one unit)
            ins = lk.tour(ts)
            ts_, others = ins[0], ins[1:]
            flat = lk.compiled(ts_, *others, *(self.values or lk.defaults))
            cols = [np.broadcast_arrays(flat[2 * i], flat[2 * i + 1], ts)[:2]
                    for i in range(len(lk.points))]
            return {name: np.stack(c, axis=-1) for name, c in zip(lk.points, cols, strict=True)}
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

    def at(self, *inputs) -> dict[str, np.ndarray]:
        """Every named point at the given input angles (radians, broadcast), one per input:
        an arm's inputs themselves, never its tour (:meth:`evaluate` with one array is the
        tour's time); a cranked linkage's, as :meth:`evaluate`."""
        lk = self.linkage
        if not lk.ranged:
            return self.evaluate(*inputs)
        if len(inputs) != len(lk.inputs):
            raise ValueError(f"{lk.key} takes inputs {lk.inputs}, got {len(inputs)}")
        ins = [np.asarray(a, dtype=float) for a in inputs]
        flat = lk.compiled(*ins, *(self.values or lk.defaults))
        cols = [np.broadcast_arrays(flat[2 * i], flat[2 * i + 1], *ins)[:2]
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
