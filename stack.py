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

:meth:`StackProblem.solve` searches link layers, fewest layers first;
:func:`verify_plan` re-checks a plan exhaustively on a fresh, denser sampling.
"""

from __future__ import annotations

import itertools
import re
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from functools import cached_property
from typing import Literal

import numpy as np

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
    """What a claim sees: the (possibly partial) link layers and the stack size."""

    layers: Mapping[str, int]
    top: int
    pitch: float

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
    can't be built in that layout at all.
    """

    owner: str
    deps: frozenset[str]
    make: Callable[[Layout], Iterable[Placed] | None]
    final: bool = False


# ---------------------------------------------------------------------------
# Static clearances: what can never share a layer, before any layer is chosen
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Keepout:
    """The thinnest shape a group always has at ``core`` (radius ``r``), in the
    layers ``where`` describes. Links in ``members`` ride on it."""

    owner: str
    core: Core
    r: float
    where: str
    members: frozenset[str] = frozenset()
    everywhere: bool = False    # fills ``core`` in every layer but its members'
    hint: str = ""              # what construction would lift it

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


class ClearanceError(ValueError):
    """A link no layer can hold, known from the geometry alone (before any search)."""


def link_gap(topo: Topology, a: str, b: str) -> float:
    """Closest approach of two links' centrelines over the cycle (lower bound)."""
    return min(topo.geometry.dist(("seg", *s), ("seg", *u))
               for s in topo.links[a] for u in topo.links[b])


def impossible(topo: Topology, clearances: Iterable[Clearance], link_r: float,
               margin: float) -> list[str]:
    """Links shut out of every layer: across a keep-out that fills all layers but its
    members', while overlapping each member."""
    out = []
    for c in clearances:
        k = c.keepout
        if k.everywhere and all(link_gap(topo, c.link, m) < 2 * link_r + margin
                                for m in k.members):
            also = f" and it overlaps each of {', '.join(sorted(k.members))}" if k.members else ""
            out.append(f"{c.describe()}{also}, so no layer can hold it"
                       + (f" ({k.hint})" if k.hint else ""))
    return out


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

    def axes_of(self, kind: AxisKind) -> list[Axis]:
        return [a for a in self.axes if a.kind == kind]

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
    parent = {n: n for n in joint_xy}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    for a, b in connections:
        parent[find(a)] = find(b)
    nodes = list(joint_xy)
    for a, b in itertools.combinations(nodes, 2):
        if find(a) != find(b) and np.abs(joint_xy[a] - joint_xy[b]).max() < tol:
            parent[find(a)] = find(b)
    groups: dict[tuple[str, str], list[tuple[str, str]]] = {}
    for n in nodes:
        groups.setdefault(find(n), []).append(n)
    return list(groups.values())


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
# The problem and the plan
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StackSpec:
    """Stack dimensions (mm): ``pitch`` = sheet thickness; ``margin`` = clearance."""

    pitch: float = 3.0
    margin: float = 1.0
    min_top: int = 2
    max_top: int = 40
    max_nodes: int = 4000         # search effort per stack size before trying a bigger one
    max_total_nodes: int = 24000  # search effort over all stack sizes before giving up


@dataclass
class StackPlan:
    """A solved stack: every link's layer, the stack size and every claimed shape."""

    spec: StackSpec
    layers: dict[str, int]
    top: int
    topo: Topology
    claims: tuple[Claim, ...]
    placed: tuple[Placed, ...] = ()

    @property
    def layout(self) -> Layout:
        return Layout(dict(self.layers), self.top, self.spec.pitch)

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


class PlanError(ValueError):
    """No layer plan: the message says what the search kept running into."""

    def __init__(self, summary: str, blockers: list[str], notes: Iterable[str] = ()):
        self.summary, self.blockers, self.notes = summary, list(blockers), list(notes)
        lines = [summary + "; what blocked it (count x shape vs shape):", *self.blockers]
        lines += [n for n in self.notes if n]
        super().__init__("\n  ".join(lines))

    def with_notes(self, *notes: str) -> PlanError:
        return PlanError(self.summary, self.blockers, [*self.notes, *notes])


class StackProblem:
    """Search link layers so that every claim fits."""

    def __init__(self, topo: Topology, claims: Iterable[Claim], spec: StackSpec | None = None):
        self.topo = topo
        self.spec = spec or StackSpec()
        self.claims = tuple(claims)
        self.links = tuple(topo.links)
        self.blocked: dict[tuple[str, str], int] = {}
        self._blocked_by: dict[tuple[str, str], tuple[Placed, Placed | None]] = {}
        known = set(self.links)
        for c in self.claims:
            if not c.deps <= known:
                unknown = sorted(c.deps - known)
                raise ValueError(f"claim {c.owner} depends on unknown links {unknown}")
        self._by_dep: dict[str, list[Claim]] = {n: [] for n in self.links}
        for c in self.claims:
            if not c.final:
                for n in c.deps:
                    self._by_dep[n].append(c)

    # -- checks ----------------------------------------------------------------

    def fits(self, p: Placed, by_layer: Mapping[int, list[Placed]], top: int) -> bool:
        """Does ``p`` clear everything already in its layer (and stay out of the plates)?

        Every refusal is tallied in :attr:`blocked` (what the search ran into).
        """
        if p.seat:
            return True
        if p.layer in (0, top):
            self._block(p, None)
            return False
        geo, m = self.topo.geometry, self.spec.margin
        for q in by_layer.get(p.layer, ()):
            if q.seat or q.group == p.group:
                continue
            if geo.dist(p.shape.core, q.shape.core) < p.shape.r + q.shape.r + m:
                self._block(p, q)
                return False
        return True

    def _make(self, c: Claim, layers: Mapping[str, int], top: int) -> list[Placed] | None:
        out, why = made(c, Layout(layers, top, self.spec.pitch))
        if out is None:
            self.blocked[(c.owner, why)] = self.blocked.get((c.owner, why), 0) + 1
            self._blocked_by.setdefault((c.owner, why), None)
        return out

    def _block(self, p: Placed, q: Placed | None) -> None:
        key = (p.label or p.group, "a frame plate" if q is None else (q.label or q.group))
        self.blocked[key] = self.blocked.get(key, 0) + 1
        self._blocked_by.setdefault(key, (p, q))

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

    # -- search ----------------------------------------------------------------

    def solve(self) -> StackPlan:
        """The thinnest plan the budgeted search finds; :class:`PlanError` (saying what
        blocked it) when it finds none."""
        order = self._order()
        self.blocked.clear()
        self._blocked_by.clear()
        spent = 0
        for top in range(self.spec.min_top, self.spec.max_top + 1):
            found = self._search(order, top)
            if found is not None:
                return self.plan(found, top)
            spent += self._nodes
            if spent >= self.spec.max_total_nodes:
                raise PlanError(f"{self.topo.name}: no layer plan found with up to {top + 1} "
                                f"layers after {spent} search steps", self.blockers())
        raise PlanError(f"{self.topo.name}: no layer plan with up to {self.spec.max_top} "
                        "layers", self.blockers())

    def plan(self, layers: Mapping[str, int], top: int) -> StackPlan:
        layout = Layout(dict(layers), top, self.spec.pitch)
        placed: list[Placed] = []
        for c in self.claims:
            out, why = made(c, layout)
            if out is None:
                raise ValueError(why)
            placed.extend(out)
        return StackPlan(self.spec, dict(layers), top, self.topo, self.claims, tuple(placed))

    def _order(self) -> list[str]:
        """b1s first (they constrain the crank), then the most-conflicted links."""
        riders = self.topo.riders
        solo = Layout({n: 1 for n in self.links}, 10**6, self.spec.pitch)
        own: dict[str, list[Placed]] = {n: [] for n in self.links}
        for c in self.claims:
            if len(c.deps) == 1 and not c.final:
                (n,) = c.deps
                own[n].extend(p for p in (made(c, solo)[0] or ()) if p.layer == 1)
        deg = {n: len(self._by_dep[n]) for n in self.links}
        for a, b in itertools.combinations(self.links, 2):
            by_layer = {1: own[a]}
            if not all(self.fits(p, by_layer, 10**6) for p in own[b]):
                deg[a] += 1
                deg[b] += 1
        return sorted(self.links, key=lambda n: (n not in riders, -deg[n], n))

    def _search(self, order: list[str], top: int) -> dict[str, int] | None:
        """Depth-first search with the most-constrained link first (fewest open layers).

        Gives up on this ``top`` after ``spec.max_nodes`` nodes (a solution
        found is always valid; only minimality is at stake).
        """
        rank = {n: i for i, n in enumerate(order)}
        layers: dict[str, int] = {}
        by_layer: dict[int, list[Placed]] = {}
        unplaced = set(self.links)
        budget = [self.spec.max_nodes]
        partners = {n: set() for n in self.links}
        for ax in self.topo.axes:
            for a, b in itertools.combinations(ax.members, 2):
                partners[a].add(b)
                partners[b].add(a)

        def add(shapes: Iterable[Placed] | None) -> list[Placed] | None:
            if shapes is None:
                return None
            added: list[Placed] = []
            for p in shapes:
                if not self.fits(p, by_layer, top):
                    remove(added)
                    return None
                by_layer.setdefault(p.layer, []).append(p)
                added.append(p)
            return added

        def remove(shapes: list[Placed]) -> None:
            for p in shapes:
                by_layer[p.layer].remove(p)

        self._nodes = 0
        for c in self.claims:
            if not c.deps and not c.final and add(self._make(c, layers, top)) is None:
                return None

        def place(n: str, k: int) -> list[Placed] | None:
            layers[n] = k
            added: list[Placed] = []
            for c in self._by_dep[n]:
                if not c.deps <= layers.keys():
                    continue
                got = add(self._make(c, layers, top))
                if got is None:
                    remove(added)
                    del layers[n]
                    return None
                added.extend(got)
            return added

        def unplace(n: str, added: list[Placed]) -> None:
            remove(added)
            del layers[n]

        def open_layers(n: str) -> list[int]:
            ks = []
            for k in range(1, top):
                added = place(n, k)
                if added is not None:
                    ks.append(k)
                    unplace(n, added)
            return ks

        def finish() -> bool:
            finals: list[Placed] = []
            for c in self.claims:
                if c.final:
                    got = add(self._make(c, layers, top))
                    if got is None:
                        remove(finals)
                        return False
                    finals.extend(got)
            return True

        def rec() -> bool:
            if not unplaced:
                return finish()
            budget[0] -= 1
            if budget[0] < 0:
                return False
            best: tuple[str, list[int]] | None = None
            for n in sorted(unplaced, key=rank.__getitem__):
                ks = open_layers(n)
                if not ks:
                    return False
                if best is None or len(ks) < len(best[1]):
                    best = (n, ks)
                    if len(ks) == 1:
                        break
            n, ks = best
            unplaced.remove(n)
            near = [layers[m] for m in partners[n] if m in layers]
            if near:   # try layers next to the links it shares an axle with first
                ks.sort(key=lambda k: (min(abs(k - j) for j in near), k))
            for k in ks:
                added = place(n, k)
                if added is None:
                    continue
                if rec():
                    return True
                unplace(n, added)
            unplaced.add(n)
            return False

        ok = rec()
        self._nodes = self.spec.max_nodes - budget[0]
        return dict(layers) if ok else None


def plan_problem(topo: Topology, claims: Iterable[Claim],
                 spec: StackSpec | None = None) -> StackPlan:
    return StackProblem(topo, claims, spec).solve()


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
        # screws) aren't joints of the template: carry them over
        fixed = {k: v[0] for k, v in plan.topo.geometry.points.items()
                 if k not in topo.geometry.points and not np.ptp(v, axis=0).any()}
        if fixed:
            topo.geometry = Geometry({**topo.geometry.points, **fixed})
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


__all__ = [
    "Axis", "Claim", "Clearance", "Disc", "Geometry", "Keepout", "Layout", "Pill", "Placed",
    "ClearanceError", "PlanError", "Unbuildable", "impossible", "link_gap", "made",
    "static_clearances",
    "StackPlan", "StackProblem", "StackSpec", "Topology", "body_class", "group_axes", "is_link",
    "is_crank", "is_frame", "plan_problem", "seg_seg", "topology_from_template", "verify_plan",
]
