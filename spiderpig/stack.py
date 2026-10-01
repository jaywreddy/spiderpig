"""Layer planning: which layer every plate sits in, so that nothing ever collides.

The Klann mechanism is planar; Z only matters for fabrication, and this module
decides it. One side of the walker is a stack of equal-height **layers**
(``pitch`` = sheet thickness):

* layer ``0`` holds the **outer frame plate**, layer ``top`` the **inner frame
  plate** (the one the servo bolts to); nothing else may sit in them except
  parts seated in their holes;
* every leg link (b1..b4) sits in one layer in between;
* layers below 0 and above ``top`` are outside the stack (pillar heads, the
  servo).

Everything that isn't a link plate (axles and their built-in spacers, pin
heads, the crank, the servo horn) is described to the planner by the
construction groups as **claims** (:class:`Claim`): the shapes a group will
occupy in each layer, stated relative to the layers of the links it depends
on. A shape is a disc around a point or a pill (capsule) between two points,
and points move over the crank cycle. The planner does not know how anything
is built. It guarantees that no two shapes of different groups in one layer
ever come closer than ``margin`` over the **whole** cycle: distances are
lower bounds that also cover the motion between samples.

One group, the crank, has a shape to choose: which layers its shaft runs
along a crankpin (or a detour point), where its webs go, whether it keeps
its bottom bearing. It is a :class:`Router`: for any layering it finds the
cheapest route whose pieces clear everything else in their layers.

:meth:`StackProblem.solve` finds the thinnest stack, trying stack sizes from
the smallest up. For each it searches link layers (the assembly tree from
the crank outwards, fewest open layers first) with forward checking: every
placed shape removes the layers it rules out for unplaced links. Every node
asks the router for a route through what is placed so far; a dead end, a
collision or a claim that can't be built is explained by the links whose
layers caused it, the search jumps back to the latest of them and remembers
the combination (a nogood). In the thinnest size it keeps searching for a
cheaper route (branch and bound). :attr:`StackPlan.optimal` says whether the
search ran to the end, :attr:`StackPlan.proof` how far it went.
:func:`verify_plan` re-checks a plan exhaustively on a fresh, denser sampling.
"""

from __future__ import annotations

import itertools
import logging
import math
import re
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field, replace
from functools import cached_property
from typing import Literal, Protocol

import numpy as np

log = logging.getLogger("stack")

# ---------------------------------------------------------------------------
# Batched planar distances (exact per sample)
# ---------------------------------------------------------------------------


def _point_seg(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    denom = np.maximum((ab * ab).sum(-1), 1e-18)
    u = np.clip(((p - a) * ab).sum(-1) / denom, 0.0, 1.0)
    return np.linalg.norm(p - (a + ab * u[..., None]), axis=-1)


def _cross(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a[..., 0] - o[..., 0]) * (b[..., 1] - o[..., 1]) - (
        a[..., 1] - o[..., 1]
    ) * (b[..., 0] - o[..., 0])


def seg_seg(p1, q1, p2, q2) -> np.ndarray:
    """Distance between two segments per sample (0 where they cross)."""
    d = np.minimum.reduce([
        _point_seg(p1, p2, q2), _point_seg(q1, p2, q2),
        _point_seg(p2, p1, q1), _point_seg(q2, p1, q1),
    ])
    d1, d2 = _cross(p2, q2, p1), _cross(p2, q2, q1)
    d3, d4 = _cross(p1, q1, p2), _cross(p1, q1, q2)
    crossing = (d1 * d2 < 0) & (d3 * d4 < 0)
    return np.where(crossing, 0.0, d)


Core = tuple  # ("pt", name) | ("seg", name_a, name_b)


class Geometry:
    """XY of named points over one crank cycle, sampled at equal steps.

    ``points[name]`` has shape ``(T, 2)``; a fixed point may be given as
    ``(2,)``. :meth:`dist` is a lower bound on the distance between two cores
    over the continuous cycle: the smallest sampled distance minus half the
    most the two can move between samples.
    """

    def __init__(self, points: Mapping[str, np.ndarray]):
        arrays = {k: np.asarray(v, dtype=float).reshape(-1, 2) for k, v in points.items()}
        n = max((a.shape[0] for a in arrays.values()), default=1)
        self.samples = n
        self.points = {k: np.broadcast_to(a, (n, 2)) for k, a in arrays.items()}
        self._dist: dict[tuple, float] = {}

    @cached_property
    def step(self) -> dict[str, float]:
        """Largest move of each point between consecutive samples (the cycle wraps)."""
        return {
            k: float(np.linalg.norm(np.roll(v, -1, axis=0) - v, axis=1).max())
            for k, v in self.points.items()
        }

    def _step(self, c: Core) -> float:
        return self.step[c[1]] if c[0] == "pt" else max(self.step[c[1]], self.step[c[2]])

    def sampled(self, a: Core, b: Core) -> np.ndarray:
        """Exact distance per sample."""
        P = self.points
        if a[0] == "pt" and b[0] == "pt":
            return np.linalg.norm(P[a[1]] - P[b[1]], axis=-1)
        if a[0] == "pt":
            return _point_seg(P[a[1]], P[b[1]], P[b[2]])
        if b[0] == "pt":
            return _point_seg(P[b[1]], P[a[1]], P[a[2]])
        return seg_seg(P[a[1]], P[a[2]], P[b[1]], P[b[2]])

    def dist(self, a: Core, b: Core) -> float:
        key = (a, b) if a <= b else (b, a)
        d = self._dist.get(key)
        if d is None:
            d = float(self.sampled(a, b).min()) - (self._step(a) + self._step(b)) / 2
            self._dist[key] = d
        return d


# ---------------------------------------------------------------------------
# Shapes and claims
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Disc:
    """A disc of radius ``r`` around point ``at``."""

    at: str
    r: float

    @property
    def core(self) -> Core:
        return ("pt", self.at)


@dataclass(frozen=True)
class Pill:
    """A capsule of radius ``r`` around the segment ``a``-``b``."""

    a: str
    b: str
    r: float

    @property
    def core(self) -> Core:
        return ("seg", self.a, self.b)


Shape = Disc | Pill


@dataclass(frozen=True)
class Placed:
    """A shape in a layer, owned by a rigid ``group``.

    Shapes of one group never need to clear each other (one printed part, one
    link). ``seat`` marks a shape that sits inside a hole of another part (an
    axle through a link or a frame plate, the crankpin through b1): it isn't
    collision-checked (the hole is the clearance) and may sit in a
    frame-plate layer, but it still bounds what the group may build there.
    """

    layer: int
    shape: Shape
    group: str
    label: str = ""
    seat: bool = False


@dataclass
class Layout:
    """What a claim sees: the (possibly partial) link layers, the stack size, and the
    choices the planner made for groups with a shape to choose (``choices[group]``, e.g.
    the crank's route; absent: the group's default)."""

    layers: Mapping[str, int]
    top: int
    pitch: float
    choices: Mapping[str, object] = field(default_factory=dict)

    def z(self, layer: int) -> tuple[float, float]:
        return layer * self.pitch, (layer + 1) * self.pitch

    def layers_between(self, z0: float, z1: float) -> range:
        """Layers whose Z range overlaps the open interval ``(z0, z1)``."""
        eps = 1e-9
        return range(int(np.floor((z0 + eps) / self.pitch)), int(np.ceil((z1 - eps) / self.pitch)))


class Unbuildable(Exception):
    """Raised by a claim's ``make`` instead of returning ``None``, to say why."""


def made(claim: Claim, layout: Layout) -> tuple[list[Placed] | None, str]:
    """``claim.make(layout)`` and, when it can't be built, why."""
    try:
        out = claim.make(layout)
    except Unbuildable as e:
        return None, f"{claim.owner}: {e}"
    if out is None:
        return None, f"{claim.owner} can't be built in this layout"
    return list(out), ""


@dataclass(frozen=True)
class Claim:
    """Space a construction group needs, as a function of link layers.

    ``make`` runs once every link in ``deps`` has a layer (or, if ``final``,
    once every link has one) and returns the shapes, or ``None`` if the group
    can't be built in that layout at all. A claim with ``choice`` is built from
    the planner's choice for that group (``Layout.choices``, a :class:`Router`'s
    route): the router places it during the search, the claim after it.

    ``early`` (optional) runs as soon as the links in ``early_deps`` have
    layers: the least the group will claim whatever the other links do (every
    shape of it lies inside one ``make`` returns later). The search checks it
    early; the plan is built from ``make``.
    """

    owner: str
    deps: frozenset[str]
    make: Callable[[Layout], Iterable[Placed] | None]
    final: bool = False
    choice: str | None = None
    early: Callable[[Layout], Iterable[Placed] | None] | None = None
    early_deps: frozenset[str] = frozenset()


# ---------------------------------------------------------------------------
# Static clearances: what can never share a layer, before any layer is chosen
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Keepout:
    """The thinnest shape a group always has at ``core`` (radius ``r``), in the
    layers ``where`` describes. Links in ``members`` ride on it; with ``span`` it
    fills ``core`` in every layer strictly between its members' layers (an
    axle), and with ``anchored`` also every layer from them to at least one of
    the frame plates (a pillar)."""

    owner: str
    core: Core
    r: float
    where: str
    members: frozenset[str] = frozenset()
    span: bool = False
    anchored: bool = False

    @property
    def label(self) -> str:
        at = self.core[1] if self.core[0] == "pt" else ""
        return self.owner if at in self.owner else f"the {self.owner} at {at}"


@dataclass(frozen=True)
class Clearance:
    """A link that sweeps too close to a keep-out to ever share its layers."""

    link: str
    keepout: Keepout
    dist: float          # closest approach over the cycle, centreline to core (mm)
    need: float          # keep-out radius + link radius + margin

    def describe(self) -> str:
        k = self.keepout
        how = (f"sweeps right across {k.label}" if self.dist <= 0 else
               f"passes {k.label} at {self.dist:.1f} mm, under the {self.need:.1f} mm its "
               "thinnest part needs")
        return f"{self.link} {how}, so it can't be in any layer {k.where}"


def static_clearances(topo: Topology, keepouts: Iterable[Keepout], link_r: float,
                      margin: float) -> list[Clearance]:
    """Every (link, keep-out) pair the layer plan must separate, from geometry alone."""
    out = []
    for k in keepouts:
        need = k.r + link_r + margin
        for link, segs in topo.links.items():
            if link in k.members:
                continue
            d = min(topo.geometry.dist(k.core, ("seg", p, q)) for p, q in segs)
            if d < need:
                out.append(Clearance(link, k, d, need))
    return out


@dataclass(frozen=True)
class Recommendation:
    """A change that clears a failure, checked by re-running the stage with it applied.

    ``changes``: what to set (a linkage parameter such as ``unit``, or a
    ``Params`` field), from, to; ``effects`` what else it changes (the crank
    radius, the torque); ``verified`` what re-running the stage showed.
    """

    changes: tuple[tuple[str, object, object], ...]     # (name, from, to): numbers or keys
    why: str = ""
    effects: str = ""
    verified: str = ""

    def describe(self) -> str:
        def num(v) -> str:
            return f"{v:g}" if isinstance(v, (int, float)) else str(v)

        out = ", ".join(f"{name} {num(a)} -> {num(b)}" for name, a, b in self.changes)
        if self.why:
            out += f": {self.why}"
        if self.effects:
            out += f" ({self.effects})"
        if self.verified:
            out += f"; {self.verified}"
        return out


class PlanError(ValueError):
    """No layer plan: the message says what the search kept running into, and what would
    clear it (:class:`Recommendation`, checked)."""

    def __init__(self, summary: str, blockers: Iterable[str] = (), notes: Iterable[str] = (),
                 recommendations: Iterable[Recommendation] = ()):
        self.summary, self.blockers, self.notes = summary, list(blockers), list(notes)
        self.recommendations = list(recommendations)
        lines = [summary + ("; what blocked it (count x shape vs shape):" if self.blockers
                            else ""), *self.blockers]
        lines += [n for n in self.notes if n]
        if self.recommendations:
            lines += ["what would clear it:", *(r.describe() for r in self.recommendations)]
        super().__init__("\n  ".join(lines))

    def with_notes(self, *notes: str) -> PlanError:
        return type(self)(self.summary, self.blockers, [*self.notes, *notes],
                          self.recommendations)

    def with_recommendations(self, recs: Iterable[Recommendation]) -> PlanError:
        return type(self)(self.summary, self.blockers, self.notes, [*self.recommendations, *recs])


class ClearanceError(PlanError):
    """The planner's static stage: a link no layer can hold, known from the geometry alone
    (before any search)."""


# ---------------------------------------------------------------------------
# Topology: links, axes and the crank, from a kinematic template
# ---------------------------------------------------------------------------

AxisKind = Literal["pin", "frame", "crankpin", "center"]


def body_class(name: str) -> str:
    """``"b1_leg3"`` -> ``"b1"``, ``"R.b1_leg3"`` -> ``"b1"``; ``"conn_upper"`` stays."""
    return re.sub(r"_leg\d+$", "", re.sub(r"^[LR]\.", "", name))


def is_link(name: str) -> bool:
    """A leg link: every linkage names its links ``b<k>`` (see :mod:`linkage`)."""
    return re.fullmatch(r"b\d+", body_class(name)) is not None


def is_crank(name: str) -> bool:
    return body_class(name).startswith("conn")


def is_frame(name: str) -> bool:
    return body_class(name) == "torso"


@dataclass(frozen=True)
class Axis:
    """Coincident joints: one physical axle. Its point in :class:`Geometry` is ``name``.

    ``kind``: ``"pin"`` joins links only; ``"frame"`` joins links to the frame
    (a pillar); ``"crankpin"`` joins b1 links to the crank; ``"center"`` is the
    crank axis O.
    """

    name: str
    kind: AxisKind
    members: tuple[str, ...]                 # leg links on the axle
    joints: tuple[tuple[str, str], ...]      # every (body, joint) on it


@dataclass
class Topology:
    """The planner's view of one side: link outlines and axles as named points."""

    name: str
    geometry: Geometry
    links: dict[str, tuple[tuple[str, str], ...]]   # link -> outline segments (point names)
    axes: tuple[Axis, ...]
    point_of: dict[tuple[str, str], str]            # (body, joint) -> point name
    frame_bodies: tuple[str, ...] = ()
    crank_bodies: tuple[str, ...] = ()
    crank_points: dict[str, tuple[float, float]] = field(default_factory=dict)

    def axes_of(self, kind: AxisKind) -> list[Axis]:
        return [a for a in self.axes if a.kind == kind]

    def add_crank_point(self, name: str, r: float, angle_deg: float) -> str:
        """A point fixed to the crank, ``r`` from O and ``angle_deg`` counter-clockwise from
        the first crankpin, added to the geometry (and remembered, so a re-sampled topology
        gets it too). Returns its name."""
        g = self.geometry.points
        pin = g[self.axes_of("crankpin")[0].name] - g["O"]
        theta = np.arctan2(pin[:, 1], pin[:, 0]) + np.radians(angle_deg)
        xy = g["O"] + r * np.stack([np.cos(theta), np.sin(theta)], axis=-1)
        self.geometry = Geometry({**g, name: xy})
        self.crank_points[name] = (r, angle_deg)
        return name

    @property
    def center(self) -> Axis | None:
        return next(iter(self.axes_of("center")), None)

    @property
    def riders(self) -> dict[str, str]:
        """b1 link -> the crankpin it rides on."""
        return {m: a.name for a in self.axes_of("crankpin") for m in a.members}


def group_axes(
    joint_xy: dict[tuple[str, str], np.ndarray],
    connections: Iterable[tuple[tuple[str, str], tuple[str, str]]],
    tol: float = 1e-6,
) -> list[list[tuple[str, str]]]:
    """Group ``(body, joint)`` nodes into physical axes.

    Nodes joined by a connection are one axis; so are nodes whose XY coincide
    over every sample (e.g. the A pivots of two same-chirality decks).
    """
    nodes = list(joint_xy)
    index = {n: i for i, n in enumerate(nodes)}
    edges = [(index[a], index[b]) for a, b in connections]
    # coincident over every sample: apart at the first sample is apart (the max over the
    # samples is at least that), so only the pairs close there are compared in full
    xy = [np.asarray(joint_xy[n], dtype=float).reshape(-1, 2) for n in nodes]
    first = np.array([a[0] for a in xy])
    near = np.abs(first[:, None, :] - first[None, :, :]).max(axis=-1) < tol
    edges += [(i, j) for i, j in zip(*np.nonzero(np.triu(near, 1)), strict=True)
              if np.abs(xy[i] - xy[j]).max() < tol]
    parent = list(range(len(nodes)))        # union-find: the connected components

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra
    groups: dict[int, list[tuple[str, str]]] = {}
    for i, node in enumerate(nodes):
        groups.setdefault(find(i), []).append(node)
    return list(groups.values())        # in order of each axis's first node


def _axis_name(nodes: list[tuple[str, str]]) -> str:
    """Name an axis after its first link joint: ``("b1_leg0", "C")`` -> ``"C_leg0"``."""
    for body, joint in sorted(nodes):
        if is_link(body):
            m = re.search(r"(_leg\d+)$", body)
            return f"{joint}{m.group(1) if m else ''}"
    return "_".join(j for _, j in sorted(nodes))


def topology_from_template(tmpl, samples: int = 1440) -> Topology:
    """Classify a template's bodies and joints into links and axles.

    Links are the b1..b4 bodies. Coincident joints form axles: the one on the
    crank centre (frame + crank, no link) is O; axles joining the crank to b1s
    are crankpins; axles touching the frame are pillars; the rest are pins.
    """
    ts = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False)
    sampled = tmpl.sample(ts)
    xy = {
        (b.name, j.name): sampled.joint_world[b.name][j.name][:, :2]
        for b in tmpl.bodies for j in b.joints
    }
    edges = [((pn, pj), (cn, cj)) for (_, pn, pj), (_, cn, cj) in tmpl.connections]
    axes: list[Axis] = []
    point_of: dict[tuple[str, str], str] = {}
    points: dict[str, np.ndarray] = {}
    for nodes in group_axes(xy, edges):
        bodies = {b for b, _ in nodes}
        members = tuple(sorted(b for b in bodies if is_link(b)))
        cranked = any(is_crank(b) for b in bodies)
        framed = any(is_frame(b) for b in bodies)
        if cranked and not members:
            axis = Axis("O", "center", (), tuple(sorted(nodes)))
        elif cranked:
            axis = Axis(_axis_name(nodes), "crankpin", members, tuple(sorted(nodes)))
        elif framed and members:
            axis = Axis(_axis_name(nodes), "frame", members, tuple(sorted(nodes)))
        elif len(members) >= 2:
            axis = Axis(_axis_name(nodes), "pin", members, tuple(sorted(nodes)))
        else:
            for n in nodes:  # a lone joint (a foot, a frame corner): its own point
                point_of[n] = f"{n[0]}.{n[1]}"
                points[point_of[n]] = xy[n]
            continue
        if axis.name in points:
            raise ValueError(f"two axles named {axis.name!r}")
        axes.append(axis)
        points[axis.name] = xy[nodes[0]]
        point_of.update({n: axis.name for n in nodes})
    links = {
        b.name: tuple((point_of[(b.name, p)], point_of[(b.name, q)]) for p, q in b.outline)
        for b in tmpl.bodies if is_link(b.name)
    }
    return Topology(
        name=tmpl.name,
        geometry=Geometry(points),
        links=links,
        axes=tuple(sorted(axes, key=lambda a: a.name)),
        point_of=point_of,
        frame_bodies=tuple(b.name for b in tmpl.bodies if is_frame(b.name)),
        crank_bodies=tuple(b.name for b in tmpl.bodies if is_crank(b.name)),
    )


# ---------------------------------------------------------------------------
# Routed groups: a body whose shape the planner chooses for each layering
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Route:
    """A router's choice for one layering, and what it costs (lower is better; 0: nothing
    added to the group's default shape)."""

    choice: object
    cost: int


@dataclass(frozen=True)
class RouteConflict:
    """No route: what happens in layers ``lo..hi`` (their occupants and links) explains it,
    or just the links in ``links`` when the router can name them. ``bound``: routes exist,
    but none cheaper than the bound; ``rules``: routes pass the occupants, but none the
    router's own rules (``why``) allow."""

    lo: int
    hi: int
    why: str = ""
    bound: bool = False
    rules: bool = False
    links: frozenset[str] = frozenset()


@dataclass
class RouteView:
    """What a router sees: the link layers so far, which of its pieces something blocks in
    each layer (``blocked[layer]``, bit ``i`` for piece ``i``), the layers still open to
    each unplaced link (``None`` once every link has one) and the cost a route must beat."""

    layout: Layout
    blocked: Mapping[int, int]
    open: Mapping[str, set[int]] | None = None
    bound: int | None = None


class Router(Protocol):
    """A group whose shape through the stack is chosen per layering (the crank's route).

    ``pieces`` are every shape it may put in a layer. :meth:`check` says whether
    a route can still pass what a partial layering has placed (a relaxation:
    more links only block more) and which of its states each layer still has
    on some route (bit masks); :meth:`states` which states a layer holding a
    link allows, given the pieces the link blocks. :meth:`route` returns the
    cheapest route whose pieces clear everything else, or why there is none.
    Its claims carry ``choice=group`` and are built from the choice.
    """

    group: str
    pieces: tuple[Shape, ...]

    def check(self, view: RouteView) -> RouteConflict | Mapping[int, int]: ...

    def states(self, link: str, blocked: int) -> int: ...

    def route(self, view: RouteView) -> Route | RouteConflict: ...


# ---------------------------------------------------------------------------
# The problem and the plan
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StackSpec:
    """Stack dimensions (mm): ``pitch`` = sheet thickness; ``margin`` = clearance.

    Search effort, in nodes: ``quick_nodes`` per stack size while looking for
    the first plan, ``max_nodes`` per size to rule out a thinner one (or find
    a cheaper route), ``max_total_nodes`` in all; and ``max_seconds`` of
    wall clock for the whole search, the safety net (a node's cost grows
    with the stack size). A plan found is always valid; only the proof that
    it is the thinnest depends on them: when they run out, a plan found is
    returned unproven (``StackPlan.optimal`` false, ``proof`` naming the
    sizes left open) and none found raises :class:`PlanError` with the tally.
    ``drop_bearing`` lets the crank lose its bottom bearing as the last resort.
    """

    pitch: float = 3.0
    margin: float = 1.0
    min_top: int = 2
    max_top: int = 40
    quick_nodes: int = 1500
    max_nodes: int = 20000
    max_total_nodes: int = 60000
    max_seconds: float = 60.0
    drop_bearing: bool = False


class Deadline:
    """A wall-clock deadline ``seconds`` from its creation, shared by nested planner runs
    (a design's plan, the leg hint's, the checks of a recommendation): each takes what is
    left of it. ``math.inf``: none."""

    def __init__(self, seconds: float = math.inf):
        self.seconds = seconds
        self.start = time.monotonic()
        self.at = self.start + seconds

    @property
    def remaining(self) -> float:
        return max(self.at - time.monotonic(), 0.0)

    @property
    def expired(self) -> bool:
        return time.monotonic() >= self.at

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.start


@dataclass
class StackPlan:
    """A solved stack: every link's layer, the stack size and every claimed shape.

    ``optimal``: no thinner stack exists and no cheaper route for the routed
    group in this one (``proof`` says how that was established, or why not).
    """

    spec: StackSpec
    layers: dict[str, int]
    top: int
    topo: Topology
    claims: tuple[Claim, ...]
    placed: tuple[Placed, ...] = ()
    choices: dict[str, object] = field(default_factory=dict)
    optimal: bool = False
    proof: str = ""
    cost: int = 0

    @property
    def layout(self) -> Layout:
        return Layout(dict(self.layers), self.top, self.spec.pitch, dict(self.choices))

    def z(self, layer: int) -> tuple[float, float]:
        return self.layout.z(layer)

    @property
    def height(self) -> float:
        """Total thickness of the stack, both frame plates included (mm)."""
        return (self.top + 1) * self.spec.pitch

    def shapes(self, group: str | None = None, layer: int | None = None) -> list[Placed]:
        return [p for p in self.placed
                if (group is None or p.group == group) and (layer is None or p.layer == layer)]

    def describe(self) -> str:
        lo = min((p.layer for p in self.placed), default=0)
        hi = max((p.layer for p in self.placed), default=self.top)
        rows = []
        for k in range(max(hi, self.top), min(lo, 0) - 1, -1):
            names = sorted(n for n, s in self.layers.items() if s == k)
            groups = sorted({p.label or p.group for p in self.placed
                             if p.layer == k and not p.seat and p.group not in self.layers})
            if k == self.top:
                label = "inner frame plate"
            elif k == 0:
                label = "outer frame plate"
            else:
                label = ", ".join(names + groups) or "·"
            if k < 0 or k > self.top:
                label = "(outside) " + (", ".join(groups) or "·")
            rows.append(f"  layer {k:2d}  z {k * self.spec.pitch:6.1f}  {label}")
        return "\n".join(rows)


class _Budget(Exception):
    """The node budget for one stack size ran out."""


class _Done(Exception):
    """A plan nothing can beat in this stack size."""


class StackProblem:
    """Find the thinnest stack of link layers in which every claim fits, and the cheapest
    route for the routed group (the crank) in it.

    ``clearances`` are the static facts (:func:`static_clearances`); those of an
    axle (``Keepout.span``) keep a link out of the layers between the axle's
    links as soon as those are placed. ``hint``: a layer per link class
    (:func:`body_class`), e.g. from one leg's plan; a second strategy places
    one leg (``_leg<i>``) at a time at those layers, shifted.
    """

    def __init__(self, topo: Topology, claims: Iterable[Claim], spec: StackSpec | None = None,
                 router: Router | None = None, clearances: Iterable[Clearance] = (),
                 hint: Mapping[str, int] | None = None, notes: Iterable[str] = ()):
        self.topo = topo
        self.hint = dict(hint or {})
        self.notes = list(notes)      # facts behind the spec (a stack size some group bounds)
        self.leg = {n: int(m.group(1)) if (m := re.search(r"_leg(\d+)$", n)) else 0
                    for n in topo.links}
        self.spec = spec or StackSpec()
        self.claims = tuple(claims)
        self.router = router
        self.clearances = tuple(clearances)
        self.links = tuple(topo.links)
        self.blocked: dict[tuple[str, str], int] = {}
        self._blocked_by: dict[tuple[str, str], tuple[Placed, Placed | None] | None] = {}
        known = set(self.links)
        for c in self.claims:
            if not c.deps <= known:
                unknown = sorted(c.deps - known)
                raise ValueError(f"claim {c.owner} depends on unknown links {unknown}")
        self.partners: dict[str, set[str]] = {n: set() for n in self.links}
        for ax in topo.axes:
            for a, b in itertools.combinations(ax.members, 2):
                self.partners[a].add(b)
                self.partners[b].add(a)
        # the assembly tree: breadth first from the crank's riders
        self.depth = {n: 1 for n in topo.riders if n in known}
        todo = list(self.depth)
        while todo:
            n = todo.pop(0)
            for m in sorted(self.partners[n]):
                if m not in self.depth:
                    self.depth[m] = self.depth[n] + 1
                    todo.append(m)
        spans: dict[tuple[frozenset[str], bool], set[str]] = {}
        for c in self.clearances:
            k = c.keepout
            if k.span and (len(k.members) > 1 or k.anchored):
                spans.setdefault((k.members, k.anchored), set()).add(c.link)
        # each axle, by its links and by the links that can't pass it
        self.spans: dict[str, list[tuple[tuple[str, ...], tuple[str, ...], bool]]] = {}
        for (members, anchored), links in spans.items():
            fact = (tuple(sorted(members)), tuple(sorted(links)), anchored)
            for n in (*members, *links):
                self.spans.setdefault(n, []).append(fact)

    # -- what blocked it ----------------------------------------------------------

    def _tally(self, p: Placed, q: Placed | None) -> None:
        key = (p.label or p.group, "a frame plate" if q is None else (q.label or q.group))
        self.blocked[key] = self.blocked.get(key, 0) + 1
        self._blocked_by.setdefault(key, (p, q))

    def _tally_why(self, owner: str, why: str) -> None:
        key = (owner, why)
        self.blocked[key] = self.blocked.get(key, 0) + 1
        self._blocked_by.setdefault(key, None)

    def blockers(self, n: int = 6) -> list[str]:
        """The pairs the search ran into most, with how close they come."""
        geo, m = self.topo.geometry, self.spec.margin
        out = []
        for key, count in sorted(self.blocked.items(), key=lambda kv: -kv[1])[:n]:
            if self._blocked_by[key] is None:           # a claim that couldn't be built
                out.append(f"{count:7d} x {key[1]}")
                continue
            p, q = self._blocked_by[key]
            if q is None:
                why = "it would sit in a frame plate's layer"
            else:
                gap = geo.dist(p.shape.core, q.shape.core) - p.shape.r - q.shape.r
                why = f"{gap:.1f} mm apart in one layer, need {m:.1f}"
            out.append(f"{count:7d} x {key[0]} vs {key[1]}: {why}")
        return out

    # -- search ----------------------------------------------------------------------

    def solve(self) -> StackPlan:
        """The thinnest plan, with the cheapest route in it; :class:`PlanError` (saying what
        blocked it, and how far each stack size got) when there is none up to
        ``spec.max_top``, or none was found before the effort ran out.

        First the thinnest stack a short search finds a plan in, then, with the
        full effort, the thinner sizes that short search didn't rule out, then a
        cheaper route in the thinnest size found. The node budgets and the
        deadline (``spec.max_seconds``) bound every part of it: the search
        never runs longer than the deadline, whatever a node costs.
        """
        spec = self.spec
        self.blocked.clear()
        self._blocked_by.clear()
        self.spent = 0
        self.deadline = Deadline(spec.max_seconds)
        tried: dict[int, _Search] = {}
        found = self._first(tried)
        if found is None and not self.exhausted:
            # the quick pass found nothing and effort is left: give the sizes it left
            # open the full effort, thinnest first (a bounded search may have only a few)
            for top in sorted(t for t, s in tried.items() if not s.done):
                if self.exhausted:
                    break
                s = tried[top]
                self._run(s, spec.max_nodes)
                if s.best is None and self.hint:
                    self._run(s, spec.max_nodes, legs=True)
                if s.best is not None:
                    found = s
                    break
        if found is None:
            last = max(tried, default=spec.min_top - 1)
            ran_out = f"; {self.stopped}" if self.stopped else ""
            bounded = " (the most a group allows)" if self.notes and last >= spec.max_top else ""
            raise PlanError(f"{self.topo.name}: no layer plan found with up to {last + 1} layers"
                            f"{bounded} after {self.spent} search steps in "
                            f"{self.deadline.elapsed:.0f} s{ran_out}", self.blockers(),
                            [self.sizes(tried), *self.notes])
        for top in range(found.top - 1, spec.min_top - 1, -1):   # just thinner first
            if self.exhausted:
                break
            s = tried.get(top)
            if s is None:
                s = tried[top] = _Search(self, top)
            self._run(s, spec.max_nodes // 2)
            if s.best is None and self.hint:
                self._run(s, spec.max_nodes // 2, legs=True)
            if s.best is not None:
                found = s
        self._run(found, spec.max_nodes)          # a cheaper route, if not ruled out yet
        plan = found.best
        below = [tried[t] for t in range(spec.min_top, plan.top) if t in tried]
        open_ = [t + 1 for t in range(spec.min_top, plan.top)
                 if t not in tried or not tried[t].done]
        plan.optimal = found.done and not open_
        why = self.stopped or "the search stopped at its budget"
        here = (f"{plan.top + 1} layers searched to the end for the cheapest crank route "
                f"({found.nodes} nodes)" if found.done else
                f"a cheaper crank route in {plan.top + 1} layers not ruled out ({why}, "
                f"{found.nodes} nodes)")
        if open_:
            proof = f"{', '.join(map(str, open_))} layers not ruled out ({why})"
        else:
            proof = (f"no plan in {plan.top} layers or fewer "
                     f"({sum(t.nodes for t in below)} nodes)")
        forced = [(t.top + 1, why, n) for t in below for why, n in t.unbuilt.items()]
        if forced:
            top, why, n = forced[-1]
            proof += (f"; the {self.router.group}'s own rules forced it taller: in {top} "
                      f"layers the search met {n} layouts that fit everything else but {why}")
        plan.proof = f"{proof}; {here}"
        return plan

    def _quick(self, tried: dict[int, _Search], top: int) -> _Search:
        """A short search of one stack size (and one a leg at a time, with a hint)."""
        s = tried[top] = _Search(self, top)
        self._run(s, self.spec.quick_nodes)
        if s.best is None and self.hint:
            self._run(s, self.spec.quick_nodes, legs=True)
        return s

    def _first(self, tried: dict[int, _Search]) -> _Search | None:
        """The thinnest stack size a quick search finds a plan in, or None.

        Every size in turn while they are ruled out; once one exhausts its
        quick budget instead, in doubling steps (a plan is then likely far
        thicker, and every size in between costs its whole budget), then, a
        plan found, back down the skipped sizes while they keep one. Sizes
        skipped stay open for the proof. With none found up to
        ``spec.max_top``, the skipped sizes in turn, while the effort lasts.
        """
        spec = self.spec
        top, jump, found = spec.min_top, 0, None
        while not self.exhausted:
            s = self._quick(tried, top)
            if s.best is not None:
                found = s
                break
            if top >= spec.max_top:
                break
            if s.done:
                top, jump = top + 1, 0
            else:
                top, jump = min(top + (1 << jump), spec.max_top), jump + 1
        if found is not None:
            for top in range(found.top - 1, spec.min_top - 1, -1):
                if top in tried or self.exhausted:
                    break
                s = self._quick(tried, top)
                if s.best is None:
                    break
                found = s
            return found
        for top in range(spec.min_top, spec.max_top + 1):
            if self.exhausted:
                break
            if top not in tried and self._quick(tried, top).best is not None:
                return tried[top]
        return None

    @property
    def exhausted(self) -> bool:
        """The deadline or the total node budget ran out: no search starts or goes on."""
        return self.deadline.expired or self.spent >= self.spec.max_total_nodes

    @property
    def stopped(self) -> str:
        """Which of the two ran out, as a phrase (``""``: neither)."""
        if self.deadline.expired:
            return f"the {self.spec.max_seconds:g} s deadline ran out"
        if self.spent >= self.spec.max_total_nodes:
            return f"the {self.spec.max_total_nodes} search-step budget ran out"
        return ""

    def sizes(self, tried: Mapping[int, _Search]) -> str:
        """How far each stack size got: ruled out, left open at its budget (with nodes and
        seconds), or not tried."""
        runs: list[tuple[str, list[int]]] = []
        for top in range(self.spec.min_top, self.spec.max_top + 1):
            s = tried.get(top)
            state = ("not tried" if s is None else "found" if s.best is not None
                     else "ruled out" if s.done else "left open at their budget")
            if runs and runs[-1][0] == state:
                runs[-1][1].append(top)
            else:
                runs.append((state, [top]))
        out = []
        for state, tops in runs:
            layers = (f"{tops[0] + 1}-{tops[-1] + 1} layers" if len(tops) > 1 else
                      f"{tops[0] + 1} layers")
            if state == "not tried":
                out.append(f"{layers} not tried")
                continue
            nodes = sorted(tried[t].nodes for t in tops)
            secs = sum(tried[t].seconds for t in tops)
            span = f"{nodes[0]}" if nodes[0] == nodes[-1] else f"{nodes[0]}-{nodes[-1]}"
            each = " each" if len(tops) > 1 else ""
            out.append(f"{layers} {state} ({span} nodes{each}, {secs:.0f} s in all)")
        return "sizes: " + "; ".join(out)

    def _run(self, s: _Search, budget: int, legs: bool = False) -> None:
        budget = min(budget, self.spec.max_total_nodes - self.spent)
        if s.done or budget <= 0 or self.deadline.expired:
            return
        before, t0 = s.nodes, time.monotonic()
        s.run(budget, legs, self.deadline)
        s.seconds += time.monotonic() - t0
        self.spent += s.nodes - before
        log.debug("%s: %d layers, %d nodes%s, %s", self.topo.name, s.top + 1, s.nodes,
                  " (a leg at a time)" if legs else "",
                  "found" if s.best else "none" if s.done else "budget")

    def plan(self, layers: Mapping[str, int], top: int,
             choices: Mapping[str, object] | None = None) -> StackPlan:
        layout = Layout(dict(layers), top, self.spec.pitch, dict(choices or {}))
        placed: list[Placed] = []
        for c in self.claims:
            out, why = made(c, layout)
            if out is None:
                raise ValueError(why)
            placed.extend(out)
        return StackPlan(self.spec, dict(layers), top, self.topo, self.claims, tuple(placed),
                         dict(choices or {}))


class _Search:
    """One stack size: forward checking and conflict-directed backjumping over link layers
    (with learned nogoods), the router as a sub-check at every node, and branch and bound
    on the route's cost.

    A conflict is the set of links whose layers explain a failure: the deps of
    the claims whose shapes collide, of a claim that can't be built, or of
    everything in the layers a router's dead end spans.

    :meth:`run` may be called again with more effort or another strategy: what
    it learned (nogoods, the best plan so far and its cost as the bound) stays.
    """

    NOGOOD_MAX = 8       # longest conflict worth remembering

    def __init__(self, prob: StackProblem, top: int):
        self.prob, self.top, self.budget = prob, top, 0
        self.geo, self.margin, self.pitch = prob.topo.geometry, prob.spec.margin, prob.spec.pitch
        self.router = prob.router
        self.links = prob.links
        self.layers: dict[str, int] = {}
        self.trail: list[tuple] = []
        self.nodes = 0
        self.seconds = 0.0                # wall clock spent in this size, over every run
        self.deadline = Deadline()
        self.base: int | None = None      # the trail once the fixed claims are placed
        self.bound: int | None = None
        self.best: StackPlan | None = None
        self.done = False
        self.legs = False
        self.unbuilt: dict[str, int] = {}     # layouts only the router's rules rejected, why
        # learned nogoods, each watched by one of its (link, layer) pairs that doesn't hold
        self.watch: dict[tuple[str, int], list[tuple[tuple[str, int], ...]]] = {}
        self.banned: set[tuple[str, int]] = set()     # nogoods of one link: never again
        self.when: dict[str, int] = {}                # when each link got its layer
        self.claims = [c for c in prob.claims if c.choice is None]
        every = frozenset(self.links)
        self.deps = {id(c): every if c.final else c.deps for c in self.claims}
        self.dep_order = {i: tuple(sorted(d)) for i, d in self.deps.items()}
        self.made: dict[tuple, tuple[list[Placed] | None, str]] = {}    # claim memo
        self.fx: dict[tuple, tuple[tuple[int, ...], tuple[tuple[str, int], ...]]] = {}
        self.pairs: dict[tuple[Shape, Shape], bool] = {}
        self.riders = frozenset(prob.topo.riders)
        self.dirty = True       # something the router sees changed since it last said yes
        self.pending = {id(c): len(self.deps[id(c)]) for c in self.claims}
        self.by_dep = {n: [c for c in self.claims if n in self.deps[id(c)]] for n in self.links}
        firsts = [c for c in self.claims if c.early is not None and c.early_deps < self.deps[id(c)]]
        self.pending.update({-id(c): len(c.early_deps) for c in firsts})
        self.by_early = {n: [c for c in firsts if n in c.early_deps] for n in self.links}
        self.by_layer: dict[int, list[tuple[Placed, frozenset[str]]]] = {}
        self.block: dict[tuple[int, int], list[frozenset[str]]] = {}
        self.bmask: dict[int, int] = {}             # layer -> bit i: router piece i blocked
        self.dom = {n: set(range(1, top)) for n in self.links}
        self.gone: dict[str, dict[int, frozenset[str]]] = {n: {} for n in self.links}
        # what a link brings on its own in each layer (its claims that depend on it alone),
        # by the layer each shape lands in: forward checking against every placed shape;
        # and the router states it allows there
        self.touch: dict[str, dict[int, list[tuple[int, Placed]]]] = {n: {} for n in self.links}
        self.allow: dict[str, dict[int, int]] = {n: {} for n in self.links}
        for n in self.links:
            solo = [(c, c.make) for c in self.claims if self.deps[id(c)] == {n}]
            solo += [(c, c.early) for c in firsts if c.early_deps == {n}]
            for v in range(1, top):
                shapes: list[Placed] = []
                for c, make in solo:
                    out, why = made(replace(c, make=make), Layout({n: v}, top, self.pitch))
                    if out is None:
                        prob._tally_why(c.owner, why)
                        break
                    shapes += out
                else:
                    if not any(not p.seat and p.layer in (0, top) for p in shapes):
                        for p in shapes:
                            if not p.seat:
                                self.touch[n].setdefault(p.layer, []).append((v, p))
                        if self.router is not None:
                            bits = sum(1 << i for i, piece in enumerate(self.router.pieces)
                                       if any(p.layer == v and not p.seat
                                              and self.hit(piece, p.shape) for p in shapes))
                            self.allow[n][v] = self.router.states(n, bits)
                        else:
                            self.allow[n][v] = -1
                        continue
                self.dom[n].discard(v)

    # -- state --------------------------------------------------------------------

    def hit(self, a: Shape, b: Shape) -> bool:
        key = (a, b)
        h = self.pairs.get(key)
        if h is None:
            h = self.pairs[key] = self.geo.dist(a.core, b.core) < a.r + b.r + self.margin
        return h

    def effects(self, p: Placed) -> tuple[tuple[int, ...], tuple[tuple[str, int], ...]]:
        """(the router pieces ``p`` blocks, the (link, layer) choices it rules out)."""
        key = (p.shape, p.layer, p.group)
        fx = self.fx.get(key)
        if fx is None:
            k = p.layer
            pieces: tuple[int, ...] = ()
            if self.router is not None and p.group != self.router.group and 0 < k < self.top:
                pieces = tuple(i for i, piece in enumerate(self.router.pieces)
                               if self.hit(piece, p.shape))
            ruled = tuple((n, v) for n in self.links for v, s in self.touch[n].get(k, ())
                          if s.group != p.group and self.hit(s.shape, p.shape))
            fx = self.fx[key] = (pieces, ruled)
        return fx

    def claim(self, c: Claim, early: bool = False) -> tuple[list[Placed] | None, str]:
        """``made(c, ...)`` for the current layers (its ``early`` part), remembered per layers
        of its deps."""
        i = -id(c) if early else id(c)
        deps = sorted(c.early_deps) if early else self.dep_order[i]
        key = (i, *(self.layers[d] for d in deps))
        out = self.made.get(key)
        if out is None:
            layout = Layout(self.layers, self.top, self.pitch)
            out = self.made[key] = made(replace(c, make=c.early) if early else c, layout)
        return out

    def cut(self, n: str, v: int, why: frozenset[str]) -> None:
        self.dom[n].discard(v)
        self.gone[n][v] = why
        self.trail.append(("cut", n, v))

    def wiped(self, n: str) -> frozenset[str] | None:
        return frozenset().union(*self.gone[n].values()) if not self.dom[n] else None

    def undo(self, mark: int) -> None:
        trail = self.trail
        while len(trail) > mark:
            e = trail.pop()
            kind = e[0]
            if kind == "cut":
                if (e[1], e[2]) not in self.banned:
                    self.dom[e[1]].add(e[2])
                    del self.gone[e[1]][e[2]]
            elif kind == "shape":
                self.by_layer[e[1]].pop()
            elif kind == "block":
                lst = self.block[e[1]]
                lst.pop()
                if not lst:
                    k, i = e[1]
                    self.bmask[k] &= ~(1 << i)
            elif kind == "pending":
                self.pending[e[1]] += 1
            else:
                del self.layers[e[1]]

    def add(self, p: Placed, deps: frozenset[str]) -> frozenset[str] | None:
        """Place a shape: it must clear every other group's shape in its layer; then it
        blocks router pieces and removes the layers it rules out for unplaced links."""
        if p.seat:
            return None
        k, top = p.layer, self.top
        if k in (0, top):
            self.prob._tally(p, None)
            return deps
        for q, qd in self.by_layer.get(k, ()):
            if q.group != p.group and self.hit(p.shape, q.shape):
                self.prob._tally(p, q)
                return deps | qd
        self.by_layer.setdefault(k, []).append((p, deps))
        self.trail.append(("shape", k))
        pieces, ruled = self.effects(p)
        for i in pieces:
            self.block.setdefault((k, i), []).append(deps)
            self.bmask[k] = self.bmask.get(k, 0) | 1 << i
            self.trail.append(("block", (k, i)))
            self.dirty = True
        for n, v in ruled:
            if n not in self.layers and v in self.dom[n]:
                self.cut(n, v, deps)
                if (c := self.wiped(n)) is not None:
                    return c
        return None

    def view(self, partial: bool) -> RouteView:
        """What the router sees now; ``partial``: the layers still open to each unplaced
        link too (the router then answers for the layering as a relaxation, and words a
        dead end of its own rules once per what they see, not per node)."""
        open_ = None
        if partial:
            open_ = {n: self.dom[n] for n in self.links if n not in self.layers}
        return RouteView(Layout(self.layers, self.top, self.pitch), self.bmask, open_, self.bound)

    def explain(self, res: RouteConflict) -> frozenset[str]:
        """The links behind a router's dead end: the ones it names, else everything in the
        layers it spans."""
        if res.bound:
            return frozenset(self.layers)
        self.prob._tally_why(self.router.group, self.describe(res))
        if res.links:
            return res.links
        out: set[str] = {n for n, k in self.layers.items() if res.lo <= k <= res.hi}
        for (k, _), deps in self.block.items():
            if res.lo <= k <= res.hi:
                for d in deps:
                    out |= d
        return frozenset(out)

    def describe(self, res: RouteConflict) -> str:
        if res.why:
            return f"crank route: {res.why}"
        k = res.hi if res.lo == 1 else res.lo
        names = sorted(n for n, v in self.layers.items() if v == k)
        return (f"crank route: no way past the layer of {', '.join(names)}" if names else
                "crank route: no way through")

    # -- search ------------------------------------------------------------------------

    def assign(self, n: str, v: int) -> frozenset[str] | None:
        """``n`` in layer ``v``, and everything that follows; a conflict if it fails."""
        self.layers[n] = v
        self.trail.append(("assign", n))
        self.when[n] = len(self.trail)
        if (watched := self.watch.pop((n, v), None)) is not None:
            for i, ng in enumerate(watched):
                other = next(((x, k) for x, k in ng if self.layers.get(x) != k), None)
                if other is None:           # every pair holds: n goes back at once
                    self.watch[(n, v)] = watched[i:]
                    return frozenset(x for x, _ in ng)
                self.watch.setdefault(other, []).append(ng)
        self.dirty = n in self.riders
        for c in self.by_early[n]:
            i = -id(c)
            self.pending[i] -= 1
            self.trail.append(("pending", i))
            if self.pending[i] or not self.pending[id(c)] - 1:   # (or the claim is due too)
                continue
            out, why = self.claim(c, early=True)
            if out is None:
                self.prob._tally_why(c.owner, why)
                return c.early_deps | {n}
            for p in out:
                if (conf := self.add(p, c.early_deps)) is not None:
                    return conf | {n}
        for c in self.by_dep[n]:
            i = id(c)
            self.pending[i] -= 1
            self.trail.append(("pending", i))
            if self.pending[i]:
                continue
            out, why = self.claim(c)
            if out is None:
                self.prob._tally_why(c.owner, why)
                return self.deps[i]
            for p in out:
                if (conf := self.add(p, self.deps[i])) is not None:
                    return conf | {n}
        if (c := self.spans(n)) is not None:
            return c | {n}
        if self.router is not None and self.dirty:     # else: as the last time it said yes
            res = self.router.check(self.view(partial=True))
            if isinstance(res, RouteConflict):
                if res.rules:
                    self.unbuilt[res.why] = self.unbuilt.get(res.why, 0) + 1
                return self.explain(res) | {n}
            # a layer none of whose states on a route a link's own shapes leave is closed to it
            why = frozenset(self.layers)
            for x in self.links:
                if x in self.layers:
                    continue
                allow = self.allow[x]
                for w in [w for w in self.dom[x] if not res.get(w, -1) & allow[w]]:
                    self.cut(x, w, why)
                if (c := self.wiped(x)) is not None:
                    return c | {n}
        return None

    def spans(self, n: str) -> frozenset[str] | None:
        """An axle runs between its links' layers: a link that can't pass it can't sit between
        them, and once one sits on one side of some of them, the rest can't go to the other.
        A pillar also runs from them to a frame plate: such links can't be on both sides."""
        layers = self.layers
        for members, links, anchored in self.prob.spans.get(n, ()):
            placed = [m for m in members if m in layers]
            if not placed:
                continue
            ks = [layers[m] for m in placed]
            lo, hi = min(ks), max(ks)
            why = frozenset(placed)
            below = above = None
            for x in links:
                w = layers.get(x)
                if w is None:
                    if (c := self.close(x, range(lo + 1, hi), why)) is not None:
                        return c
                    continue
                if lo < w < hi:
                    return why | {x}
                if hi < w:
                    above = x
                    side = range(w, self.top)
                else:
                    below = x
                    side = range(1, w + 1)
                for m in members:
                    if m not in layers and (c := self.close(m, side, why | {x})) is not None:
                        return c
            if anchored and (below or above):
                if below and above:
                    return why | {below, above}
                # the pillar must reach the plate on the other side
                other = range(hi + 1, self.top) if below else range(1, lo)
                for x in links:
                    if x not in layers and (c := self.close(
                            x, other, why | {below or above})) is not None:
                        return c
        return None

    def close(self, x: str, ks: range, why: frozenset[str]) -> frozenset[str] | None:
        """Take layers ``ks`` from unplaced ``x``'s domain; its conflict if none are left."""
        for u in ks:
            if u in self.dom[x]:
                self.cut(x, u, why)
        return self.wiped(x)

    def values(self, n: str) -> list[int]:
        """Layers next to the links it shares an axle with first; a leg at a time: where the
        hint puts it, shifted as the leg's first placed link was."""
        ks = sorted(k for k in self.dom[n] if (n, k) not in self.banned)
        hint, leg = self.prob.hint, self.prob.leg
        if self.legs and body_class(n) in hint:
            placed = [x for x in self.layers if leg[x] == leg[n] and body_class(x) in hint]
            if placed:
                first = min(placed, key=self.when.__getitem__)
                want = hint[body_class(n)] + self.layers[first] - hint[body_class(first)]
                ks.sort(key=lambda k: (abs(k - want), k))
            return ks
        near = [self.layers[m] for m in self.prob.partners[n] if m in self.layers]
        if near:
            ks.sort(key=lambda k: (min(abs(k - j) for j in near), k))
        return ks

    def dfs(self) -> frozenset[str]:
        """Explore below the current layers; the conflict that explains why nothing (better)
        is there. Solutions go to :attr:`best`."""
        self.nodes += 1
        if self.nodes > self.budget or self.deadline.expired:
            raise _Budget
        free = [n for n in self.links if n not in self.layers]
        if not free:
            return self.leaf()
        # fewest open layers first, then the assembly tree from the crank out (riders, the
        # links pinned to them, ...); a leg at a time in the other strategy
        depth, dom, leg = self.prob.depth, self.dom, self.prob.leg
        if self.legs:
            started = {leg[x] for x in self.layers} & {leg[x] for x in free}
            now = min(started) if started else min(leg[x] for x in free)
            free = [x for x in free if leg[x] == now]
        n = min(free, key=lambda x: (len(dom[x]), depth.get(x, 99), x))
        conflict: set[str] = set()
        for v in self.values(n):
            mark = len(self.trail)
            c = self.assign(n, v)
            if c is None:
                c = self.dfs()
                if n not in c:          # n isn't to blame: jump back past it
                    self.undo(mark)
                    return c
            self.undo(mark)
            conflict |= c
        conflict.discard(n)
        for why in self.gone[n].values():
            conflict |= why
        out = frozenset(conflict)
        if len(out) == 1:
            (x,) = out
            self.banned.add((x, self.layers[x]))
            self.dom[x].discard(self.layers[x])
            self.gone[x][self.layers[x]] = frozenset()
        elif 1 < len(out) <= self.NOGOOD_MAX:
            # watched by its latest pair: the one undone first on the way back
            ng = tuple(sorted(((x, self.layers[x]) for x in out), key=lambda p: self.when[p[0]]))
            self.watch.setdefault(ng[-1], []).append(ng)
        return out

    def leaf(self) -> frozenset[str]:
        """Every link has a layer: the final claims, the route, the plan checked."""
        choices, cost = {}, 0
        if self.router is not None:
            res = self.router.route(self.view(partial=False))
            if isinstance(res, RouteConflict):
                if res.rules:
                    self.unbuilt[res.why] = self.unbuilt.get(res.why, 0) + 1
                return self.explain(res)
            choices, cost = {self.router.group: res.choice}, res.cost
        plan = self.prob.plan(self.layers, self.top, choices)
        bad = verify_plan(plan)
        if bad:
            raise AssertionError(f"{self.prob.topo.name}: the planner accepted a plan its "
                                 f"verification rejects: {bad[:3]}")
        plan.cost = cost
        self.best, self.bound = plan, cost
        if cost == 0:
            raise _Done
        return frozenset(self.layers)

    def run(self, budget: int, legs: bool = False, deadline: Deadline | None = None) -> bool:
        """Search this stack size with ``budget`` more nodes, until ``deadline`` (``legs``: a
        leg at a time, at the hint's layers); True once it has been searched to the end."""
        if self.done:
            return True
        self.deadline = deadline or Deadline()
        if self.base is None:                        # the fixed claims, once
            self.base = 0
            for c in self.claims:
                if not self.deps[id(c)]:
                    out, why = made(c, Layout({}, self.top, self.pitch))
                    if out is None:
                        self.prob._tally_why(c.owner, why)
                        self.done = True
                        return True
                    if any(self.add(p, frozenset()) is not None for p in out):
                        self.done = True
                        return True
            self.base = len(self.trail)
            if any(not d for d in self.dom.values()):
                self.done = True
                return True
        self.budget, self.legs = self.nodes + budget, legs
        try:
            self.dfs()
            self.done = True
        except _Done:
            self.done = True
        except _Budget:
            pass
        finally:
            self.undo(self.base)
        return self.done


# ---------------------------------------------------------------------------
# Independent verification
# ---------------------------------------------------------------------------


def verify_plan(plan: StackPlan, tmpl=None, samples: int = 2880, tol: float = 1e-6) -> list[str]:
    """Re-check a plan from scratch: every claim re-evaluated, every pair tested.

    With ``tmpl`` the geometry is re-sampled (denser than the solver's), so
    the check doesn't reuse any solver table. Returns human-readable
    violations (empty when valid).
    """
    topo = topology_from_template(tmpl, samples) if tmpl is not None else plan.topo
    if tmpl is not None:
        # fixed points a group added to the plan's geometry (e.g. the servo's mounting
        # screws) aren't joints of the template: carry them over, and put the points fixed
        # to the crank back on it
        fixed = {k: v[0] for k, v in plan.topo.geometry.points.items()
                 if k not in topo.geometry.points and k not in plan.topo.crank_points
                 and not np.ptp(v, axis=0).any()}
        if fixed:
            topo.geometry = Geometry({**topo.geometry.points, **fixed})
        for name, (r, ang) in plan.topo.crank_points.items():
            topo.add_crank_point(name, r, ang)
    geo, sp, layout = topo.geometry, plan.spec, plan.layout
    bad: list[str] = []
    shapes: list[Placed] = []
    for c in plan.claims:
        out, why = made(c, layout)
        if out is None:
            bad.append(why)
            continue
        shapes.extend(out)
    for n in topo.links:
        if n not in plan.layers:
            bad.append(f"{n} has no layer")
        elif not 0 < plan.layers[n] < plan.top:
            bad.append(f"{n} sits in layer {plan.layers[n]}, outside the frame plates")
    for p in shapes:
        if not p.seat and p.layer in (0, plan.top):
            bad.append(f"{p.label or p.group} sits in frame-plate layer {p.layer}")
    live = [p for p in shapes if not p.seat]
    for a, b in itertools.combinations(live, 2):
        if a.layer != b.layer or a.group == b.group:
            continue
        need = a.shape.r + b.shape.r + sp.margin
        d = geo.dist(a.shape.core, b.shape.core)
        if d < need - tol:
            bad.append(f"layer {a.layer}: {a.label or a.group} x {b.label or b.group} "
                       f"clear {d - a.shape.r - b.shape.r:.2f} mm (need {sp.margin:.2f})")
    return bad
