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

import heapq
import itertools
import logging
import math
import multiprocessing
import os
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

    def __init__(self, points: Mapping[str, np.ndarray], shared: Geometry | None = None):
        # a geometry made from ``shared`` keeps the distances between the points it takes
        # over unchanged (the very same arrays) in that one's table; what a topology's users
        # add (a servo's screws, a crank's detours) is measured into this one's own
        same = ({k for k, v in points.items() if shared.points.get(k) is v}
                if shared is not None else set())
        arrays = {k: v if k in same else np.asarray(v, dtype=float).reshape(-1, 2)
                  for k, v in points.items()}
        n = max((a.shape[0] for a in arrays.values()), default=1)
        if shared is not None and shared.samples != n:
            same = set()
        self.samples = n
        self.points = {k: a if k in same else np.broadcast_to(a, (n, 2))
                       for k, a in arrays.items()}
        self._dist: dict[tuple, float] = {}
        self._shared: dict[tuple, float] | None = None
        self._shared_names: frozenset[str] = frozenset()
        if same:
            if shared._shared is None:
                self._shared, self._shared_names = shared._dist, frozenset(same)
            else:
                self._shared = shared._shared
                self._shared_names = frozenset(same) & shared._shared_names

    def extend(self, points: Mapping[str, np.ndarray]) -> Geometry:
        """A geometry with ``points`` added (or replaced), sharing this one's table for the
        points it keeps unchanged."""
        return Geometry({**self.points, **points}, shared=self)

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
        table = self._dist
        if self._shared is not None:
            names = self._shared_names
            if all(nm in names for nm in a[1:]) and all(nm in names for nm in b[1:]):
                table = self._shared
        d = table.get(key)
        if d is None:
            d = float(self.sampled(a, b).min()) - (self._step(a) + self._step(b)) / 2
            table[key] = d
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
    # A **clearance** shape (``gap``): it sits in the thin clearance gap above ``layer``
    # (between ``layer`` and ``layer + 1``), not in the layer: a fastener's head or nut on a
    # link's face, or the washers an axle carries through such a gap. ``height`` is the z it
    # needs there (a head and its washers, plus clearance): the plan sizes each gap to the
    # tallest it holds, from the stock thin-sheet thicknesses (``StackSpec.gaps``); a gap
    # whose shapes all need 0 (an axle's washers) doesn't make one. ``toward``: the layer a
    # head may sink into instead when nothing there is in its way (the old full-layer head:
    # +1 the layer over the gap, a head on its link's top face; -1 the layer under it, a
    # head hanging under its link; 0 it can't). A head the plan sank keeps its ``height``.
    gap: bool = False
    height: float = 0.0
    toward: int = 0
    # a laser-cut plate of this thickness (mm; 0: none): its layer is at least that thick
    sheet: float = 0.0

    @property
    def slot(self) -> float:
        """Where it sits in the stack: its layer, or ``layer + 0.5`` for the gap above it."""
        return self.layer + 0.5 if self.gap else self.layer


@dataclass
class Layout:
    """What a claim sees: the (possibly partial) link layers, the stack size, and the
    choices the planner made for groups with a shape to choose (``choices[group]``, e.g.
    the crank's route; absent: the group's default).

    Z: layer ``k`` is ``thick[k]`` thick (``pitch`` when absent: the default sheet), and
    the clearance gap above it ``gaps[k]`` (none when absent); layer 0's bottom face is
    z 0. While the planner searches, a layout has neither (every layer ``pitch``, no gaps:
    the nominal stack); the plan's (``final``) has both, sized from what it holds."""

    layers: Mapping[str, int]
    top: int
    pitch: float
    choices: Mapping[str, object] = field(default_factory=dict)
    gaps: Mapping[int, float] = field(default_factory=dict)
    thick: Mapping[int, float] = field(default_factory=dict)
    final: bool = False

    def t(self, layer: int) -> float:
        """Layer ``layer``'s thickness."""
        return self.thick.get(layer, self.pitch)

    def gap(self, layer: int) -> float:
        """The clearance gap above layer ``layer`` (0: none)."""
        return self.gaps.get(layer, 0.0)

    def _z0(self, layer: int) -> float:
        cache = self.__dict__.setdefault("_zcache", {})
        z = cache.get(layer)
        if z is None:
            # ``sum`` of each layer and the gap over it, the same terms in the same order
            # (its float summation is compensated: a running total would differ in the
            # last bits), the terms made once per layout
            if layer >= 0:
                up = self.__dict__.setdefault("_zup", [])         # layers 0, 1, ...
                while len(up) < layer:
                    up.append(self.t(len(up)) + self.gap(len(up)))
                z = sum(up[:layer])
            else:
                down = self.__dict__.setdefault("_zdown", [])     # layers -1, -2, ...
                while len(down) < -layer:
                    down.append(self.t(-1 - len(down)) + self.gap(-1 - len(down)))
                z = -sum(down[-layer - 1::-1])                    # layers layer..-1
            cache[layer] = z
        return z

    def z(self, layer: int) -> tuple[float, float]:
        if not self.thick and not self.gaps:
            return layer * self.pitch, (layer + 1) * self.pitch
        z0 = self._z0(layer)
        return z0, z0 + self.t(layer)

    def gap_z(self, layer: int) -> tuple[float, float]:
        """The clearance gap above layer ``layer`` (empty when there is none)."""
        z1 = self.z(layer)[1]
        return z1, z1 + self.gap(layer)

    def slot_z(self, p: Placed) -> tuple[float, float]:
        """Where a placed shape sits: its layer, or its gap."""
        return self.gap_z(p.layer) if p.gap else self.z(p.layer)

    def layers_between(self, z0: float, z1: float) -> range:
        """Layers whose Z range overlaps the open interval ``(z0, z1)``."""
        eps = 1e-9
        lo = int(np.floor((z0 + eps) / self.pitch))
        hi = int(np.ceil((z1 - eps) / self.pitch))
        if not self.thick and not self.gaps:
            return range(lo, hi)
        # a function of the stack's z alone, asked again and again of equal layouts (the
        # crank's hub under the inner plate, for every route the planner tries)
        key = (self.top, self.pitch, tuple(self.thick.items()), tuple(self.gaps.items()),
               z0, z1)
        got = _BETWEEN.get(key)
        if got is None:
            span = range(min(lo, 0) - 16, max(hi, self.top) + 17)
            ks = [k for k in span if self.z(k)[0] < z1 - eps and self.z(k)[1] > z0 + eps]
            got = range(ks[0], ks[-1] + 1) if ks else range(lo, lo)
            if len(_BETWEEN) >= 1 << 14:
                _BETWEEN.clear()
            _BETWEEN[key] = got
        return got

    def height(self) -> float:
        """Both frame plates' outer faces apart (mm)."""
        return self.z(self.top)[1] - self.z(0)[0]


_BETWEEN: dict[tuple, range] = {}
"""(:meth:`Layout.layers_between`) answers by the stack's z and the interval."""


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
    clear it (:class:`Recommendation`, checked). ``expired``: the search's CPU budget
    (``StackSpec.max_seconds``) ran out before it could rule the sizes out, so this says
    the machine was busy, not that the design has no plan (a caller shouldn't remember
    it as unbuildable)."""

    def __init__(self, summary: str, blockers: Iterable[str] = (), notes: Iterable[str] = (),
                 recommendations: Iterable[Recommendation] = (), expired: bool = False):
        self.summary, self.blockers, self.notes = summary, list(blockers), list(notes)
        self.recommendations = list(recommendations)
        self.expired = expired
        lines = [summary + ("; what blocked it (count x shape vs shape):" if self.blockers
                            else ""), *self.blockers]
        lines += [n for n in self.notes if n]
        if self.recommendations:
            lines += ["what would clear it:", *(r.describe() for r in self.recommendations)]
        super().__init__("\n  ".join(lines))

    def with_notes(self, *notes: str) -> PlanError:
        return type(self)(self.summary, self.blockers, [*self.notes, *notes],
                          self.recommendations, expired=self.expired)

    def with_recommendations(self, recs: Iterable[Recommendation]) -> PlanError:
        return type(self)(self.summary, self.blockers, self.notes, [*self.recommendations, *recs],
                          expired=self.expired)


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
        self.geometry = self.geometry.extend({name: xy})
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


_SAMPLED: dict[tuple, Topology] = {}     # a module template's topology, sampled once
SAMPLED_MAX = 16                          # templates kept (the newest)


def topology_from_template(tmpl, samples: int = 1440) -> Topology:
    """Classify a template's bodies and joints into links and axles.

    Links are the b1..b4 bodies. Coincident joints form axles: the one on the
    crank centre (frame + crank, no link) is O; axles joining the crank to b1s
    are crankpins; axles touching the frame are pillars; the rest are pins.

    A module template (:func:`linkage.build_module_template`: its ``meta`` names the
    linkage, module, phases and proportions that made it) is sampled once per process:
    the designs an agent derives from one another share the linkage and module, and the
    check, the plan and its re-make from the store each ask for the same topology. Every
    call gets its own :class:`Topology` (a plan adds crank points to it) over one shared,
    read-only :class:`Geometry`, whose distance table then serves them all.
    """
    meta = tmpl.meta if isinstance(getattr(tmpl, "meta", None), dict) else {}
    key = None
    if {"linkage", "module", "phases", "proportions"} <= set(meta):
        key = (tmpl.name, tuple(b.name for b in tmpl.bodies), tuple(tmpl.connections),
               tuple(sorted(meta.items())), samples)
        base = _SAMPLED.get(key)
        if base is not None:
            return _copy_of(base)
    topo = _sample_topology(tmpl, samples)
    if key is not None:
        if len(_SAMPLED) >= SAMPLED_MAX:
            del _SAMPLED[next(iter(_SAMPLED))]
        _SAMPLED[key] = topo
        return _copy_of(topo)
    return topo


def _copy_of(base: Topology) -> Topology:
    """A topology of ``base``'s links, axes and points with its own geometry (own points,
    sharing the base's distance table for them) and its own crank points."""
    geo = Geometry(dict(base.geometry.points), shared=base.geometry)
    return replace(base, geometry=geo, crank_points={})


def _sample_topology(tmpl, samples: int) -> Topology:
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


MAX_SECONDS = float(os.environ.get("SPIDERPIG_PLAN_SECONDS", "60"))
"""The planner's default CPU-seconds budget (:attr:`StackSpec.max_seconds`, and the
shared deadline of the recommendation checks, :mod:`recommend`). ``math.inf`` leaves the
node budgets as the only bound, which makes how far a search gets the same on every
machine (``tests/test_stage_checks.py`` does that)."""


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
    ``workers`` (a prototype): search the stack sizes in that many forked processes
    (:mod:`stack_pool`), with the serial search's answer. ``max_seconds`` is CPU time of
    this process (:class:`Deadline`), so how far a search gets doesn't depend on what else
    the machine is doing (a 60 s wall-clock budget made the same design plan at load 3 and
    fail at load 25, measured); a forked pool worker counts its own CPU from zero, so it
    gets about that much on top. ``prove`` (a prototype, on by
    default): with it off the search stops at the thinnest stack its quick pass finds a
    plan in (the plan it would return before the proof), and says what is left unproven.
    ``quick_first`` (a prototype, off by default): a short search stops at its first plan
    instead of spending the rest of its budget on a cheaper route in a size the next,
    thinner one may well beat (the size kept gets the cheaper route's full search).
    ``symmetry`` (a prototype, off by default): a two-leg module whose leg swap is a
    symmetry of the problem (:mod:`stack_symmetry`) searches one of each mirror pair.
    """

    pitch: float = 3.0
    margin: float = 1.0
    min_top: int = 2
    max_top: int = 60
    quick_nodes: int = 1500
    max_nodes: int = 20000
    max_total_nodes: int = 60000
    # of CPU time (Deadline): a loaded machine doesn't shorten it; read from MAX_SECONDS when
    # a spec is made, so a test can take the clock out and bound a search by its nodes alone
    max_seconds: float = field(default_factory=lambda: MAX_SECONDS)
    drop_bearing: bool = False
    workers: int = 1
    prove: bool = True
    quick_first: bool = False
    symmetry: bool = False
    # z of the plan (:meth:`StackProblem.plan`): the frame plates' thickness (``None``: the
    # pitch), each link's sheet thickness (a link not named: the pitch; a layer is as thick
    # as the thickest plate in it), and the thicknesses a clearance gap may have (the thin
    # sheet a filler plate is cut from, and what an axle's washers stack to), thinnest first
    frame_t: float | None = None
    link_t: tuple[tuple[str, float], ...] = ()
    gaps: tuple[float, ...] = (1.0, 1.5, 2.0, 2.29, 2.54)
    # Where a fastener's head (a clearance shape that may sink, :attr:`Placed.toward`) goes:
    # "sink" claims the layer beside its link, as a full-layer head (no gap ever); "gap"
    # puts it in a thin clearance gap unless it fits that layer; "best" plans them sunk, and
    # in gaps only when that fails; "gap_sink" (a router with its own heads in gaps: the
    # single-plate crank) in gaps, and only when that fails the pivots' heads sunk with the
    # router's (and :data:`GAP_GROUPS`') still in their gaps. :attr:`StackPlan.heads` says
    # which ("sink" or "gap").
    heads: str = "best"


def thread_time() -> float:
    """CPU seconds of the calling thread (:data:`time.CLOCK_THREAD_CPUTIME_ID`; the process's
    where a platform lacks it): what the planner's deadline counts, so neither the machine's
    load nor another thread of the same process (a server baking a glb) shortens a search."""
    try:
        return time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
    except (AttributeError, OSError):
        return time.process_time()


class Deadline:
    """A CPU-time deadline ``seconds`` from its creation (``clock``: :func:`thread_time`, the
    calling thread's CPU; a wall clock would make a plan's reach depend on the machine's
    load), shared by nested planner runs (a design's plan, the leg hint's, the checks of a
    recommendation): each takes what is left of it. ``math.inf``: none."""

    def __init__(self, seconds: float = math.inf, clock=thread_time):
        self.seconds = seconds
        self.clock = clock
        self.start = clock()
        self.at = self.start + seconds

    @property
    def remaining(self) -> float:
        return max(self.at - self.clock(), 0.0)

    @property
    def expired(self) -> bool:
        return self.clock() >= self.at

    @property
    def elapsed(self) -> float:
        return self.clock() - self.start


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
    gaps: dict[int, float] = field(default_factory=dict)    # layer -> clearance gap above it
    thick: dict[int, float] = field(default_factory=dict)   # layer -> thickness (not pitch)
    sunk: frozenset = frozenset()       # the heads sunk into a layer (``sunk_key``)
    heads: str = "gap"                  # how the plan placed heads (StackSpec.heads)

    @cached_property
    def layout(self) -> Layout:
        return Layout(dict(self.layers), self.top, self.spec.pitch, dict(self.choices),
                      dict(self.gaps), dict(self.thick), final=True)

    def z(self, layer: int) -> tuple[float, float]:
        return self.layout.z(layer)

    def gap_z(self, layer: int) -> tuple[float, float]:
        return self.layout.gap_z(layer)

    def slot_z(self, p: Placed) -> tuple[float, float]:
        return self.layout.slot_z(p)

    def t(self, layer: int) -> float:
        return self.layout.t(layer)

    @property
    def height(self) -> float:
        """Total thickness of the stack, both frame plates and every gap included (mm)."""
        return self.layout.height()

    def shapes(self, group: str | None = None, layer: int | None = None,
               gaps: bool = True) -> list[Placed]:
        """Placed shapes, of ``group`` and in ``layer`` (the gap above it too, unless
        ``gaps`` is false)."""
        return [p for p in self.placed
                if (group is None or p.group == group) and (layer is None or p.layer == layer)
                and (gaps or not p.gap)]

    def describe(self) -> str:
        lo = min((p.layer for p in self.placed), default=0)
        hi = max((p.layer for p in self.placed), default=self.top)
        rows = []
        for k in range(max(hi, self.top), min(lo, 0) - 1, -1):
            if self.gaps.get(k):
                held = sorted({p.label or p.group for p in self.placed
                               if p.gap and p.layer == k and p.height > 0})
                rows.append(f"  gap      z {self.gap_z(k)[0]:6.1f}  {self.gaps[k]:g} mm "
                            f"clearance: {', '.join(held)}")
            names = sorted(n for n, s in self.layers.items() if s == k)
            groups = sorted({p.label or p.group for p in self.placed
                             if p.layer == k and not p.gap and not p.seat
                             and p.group not in self.layers})
            if k == self.top:
                label = "inner frame plate"
            elif k == 0:
                label = "outer frame plate"
            else:
                label = ", ".join(names + groups) or "·"
            if k < 0 or k > self.top:
                label = "(outside) " + (", ".join(groups) or "·")
            t = self.t(k)
            rows.append(f"  layer {k:2d}  z {self.z(k)[0]:6.1f}  "
                        + (f"{t:g} mm  " if abs(t - self.spec.pitch) > 1e-9 else "") + label)
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
                 hint: Mapping[str, int] | None = None):
        self.topo = topo
        self.hint = dict(hint or {})
        self.give_up = 0              # (solve_heads) stop once this many layerings failed on
        #                               the crank's washers in the plan's gaps with no plan
        #                               found (0: never); ``gave_up`` says so
        self.gave_up = ""
        self.found_any = False
        self.washer_misses = 0
        self.leg = {n: int(m.group(1)) if (m := re.search(r"_leg(\d+)$", n)) else 0
                    for n in topo.links}
        self.spec = spec or StackSpec()
        self.raw_claims = tuple(claims)
        self.heads = "gap" if self.spec.heads in HEADS_ORDER else self.spec.heads
        self.router = router
        # a router whose own heads sit in clearance gaps (gap pieces: the single-plate
        # crank's) keeps them there whatever the other groups' heads do, and so do the
        # fixed screws its gap pieces are checked against (the drive's, under the inner
        # plate): with heads "sink" only the pivots' heads go into layers
        self.gap_groups: frozenset[str] = (
            frozenset((router.group, *GAP_GROUPS)) if router is not None
            and getattr(router, "gap_pieces", ()) else frozenset())
        self.claims = heads_claims(self.raw_claims, self.heads, self.gap_groups)
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
        ``spec.max_top``, or none was found before the effort ran out. With
        ``spec.heads`` "best": sunk, else in gaps; "gap_sink": the other way round
        (:meth:`solve_heads`).

        First the thinnest stack a short search finds a plan in, then, with the
        full effort, the thinner sizes that short search didn't rule out, then a
        cheaper route in the thinnest size found. The node budgets and the
        deadline (``spec.max_seconds``) bound every part of it: the search
        never runs longer than the deadline, whatever a node costs.
        """
        spec = self.spec
        if spec.heads in HEADS_ORDER:
            return self.solve_heads(HEADS_ORDER[spec.heads])
        self.blocked.clear()
        self._blocked_by.clear()
        self.spent = 0
        self.deadline = Deadline(spec.max_seconds)
        self.tried = tried = {}
        self.runs_of: dict[int, list[tuple]] = {}    # each size's runs: (budget, legs, first)
        self.syms = []
        if spec.symmetry:
            from spiderpig.stack_symmetry import symmetries

            self.syms = symmetries(self)
        self.pool = None
        if spec.workers > 1 and "fork" in multiprocessing.get_all_start_methods():
            from spiderpig.stack_pool import Pool

            self.pool = Pool(self, spec.workers)
        try:
            return self._solve(tried)
        finally:
            if self.pool is not None:
                self.pool.close()
                self.pool = None

    def solve_heads(self, order: tuple[str, str] = ("sink", "gap")) -> StackPlan:
        """Plan with the heads sunk into layers (the full-layer heads); in clearance gaps
        only when that finds no plan (2026-10-04: searching both doubled the suite's time).
        ``order`` ("gap", "sink" for a single-plate crank: its gap plans are the thinner and
        the faster nearly everywhere, and with only the pivots' heads sunk it plans what
        gaps don't, TrotBot's heel and toe): then the sunk search runs only when the gap
        search gave up (:data:`GIVE_UP`), and the gap search again in full when the sunk
        one finds none either. Each search has the whole budget."""
        plans, errors = [], []
        gave_up = ""
        searched: list[str] = []
        runs = list(order)
        if order[0] == "gap":
            # a gap search whose layerings keep failing on the crank's washers in the plan's
            # gaps (TrotBot's heel and toe: no route keeps its washers clear of the pins'
            # caps there) gives up early for the sunk one; with none there either it runs
            # again in full (GIVE_UP)
            runs.append("gap")
        for i, heads in enumerate(runs):
            spec = replace(self.spec, heads=heads)
            if plans:
                # the gap search only when the sunk plan failed (the user's rule of
                # 2026-10-04: two searches doubled the suite's time for a few mm)
                break
            if len(runs) > 2 and i >= 1 and not gave_up:
                # the gap search ran in full and found none: the sunk one found none on any
                # design of the sweep of 2026-10-04 either (the Jansen decker and quad), so it
                # isn't searched, and there is nothing to resume
                break
            sub = StackProblem(self.topo, self.raw_claims, spec,
                               self.router, self.clearances, self.hint)
            if i == 0 and len(runs) > 2:
                sub.give_up = GIVE_UP
            searched.append(heads)
            try:
                plans.append(sub.solve())
            except PlanError as e:
                errors.append(e)
            gave_up = gave_up or sub.gave_up
            for key, n in sub.blocked.items():
                self.blocked[key] = self.blocked.get(key, 0) + n
                self._blocked_by.setdefault(key, sub._blocked_by[key])
            self.tried = getattr(sub, "tried", {})
            self.spent = getattr(self, "spent", 0) + getattr(sub, "spent", 0)
        if not plans:
            e = errors[-1]
            raise type(e)(e.summary, self.blockers(), e.notes, e.recommendations,
                          expired=any(x.expired for x in errors))
        best = min(plans, key=lambda p: (round(p.height, 3), len(p.gaps)))
        other = [p for p in plans if p is not best]
        if other:
            o = other[0]
            best.proof += (f"; heads {best.heads}: {best.height:.1f} mm against "
                           f"{o.height:.1f} mm with them {o.heads} ({o.top + 1} layers"
                           + (f", {len(o.gaps)} gaps" if o.gaps else "") + ")")
        elif errors:
            best.proof += (f"; with the heads {'gap' if best.heads == 'sink' else 'sink'}: "
                           + (f"none (it {gave_up})" if gave_up else "none"))
        unsearched = [m for m in dict.fromkeys(order) if m not in searched]
        if unsearched:
            # optimal is the thinnest with the heads as searched: the other placement wasn't
            # (heads "best": sunk first, in gaps only when that finds no plan)
            best.proof += (f"; heads {', '.join(unsearched)}: not searched (heads "
                           f"{best.heads} first; it may be thinner)")
        return best

    def _new(self, top: int):
        """A stack size to search: here, or in a worker already searching it
        (:mod:`stack_pool`)."""
        if self.pool is None or not self.pool.has(top):
            return _Search(self, top)
        from spiderpig.stack_pool import Remote

        return Remote(self, top)

    def _expect(self, ahead: list[tuple[int, str]]) -> None:
        """What the search will likely run next (``(top, phase)``), for the workers to start."""
        if self.pool is not None:
            self.pool.expect(ahead)

    def _solve(self, tried: dict) -> StackPlan:
        spec = self.spec
        found = self._first(tried)
        if found is None and not self.exhausted:
            # the quick pass found nothing and effort is left: give the sizes it left
            # open the full effort, thinnest first (a bounded search may have only a few)
            self._expect([])
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
            raise PlanError(f"{self.topo.name}: no layer plan found with up to {last + 1} layers"
                            f" after {self.spent} search steps in "
                            f"{self.deadline.elapsed:.0f} CPU s{ran_out}", self.blockers(),
                            [self.sizes(tried)], expired=self.deadline.expired)
        if spec.prove:
            self._expect([(found.top, "route")] + [
                (t, "prove") for t in range(found.top - 1, spec.min_top - 1, -1)
                if t not in tried or not tried[t].done])
        for top in range(found.top - 1, spec.min_top - 1, -1):   # just thinner first
            if self.exhausted or not spec.prove:
                break
            s = tried.get(top)
            if s is None:
                s = tried[top] = self._new(top)
            self._run(s, spec.max_nodes // 2)
            if s.best is None and self.hint:
                self._run(s, spec.max_nodes // 2, legs=True)
            if s.best is not None:
                found = s
        found = tried[found.top]                  # (a worker may search it now)
        if spec.prove:
            self._run(found, spec.max_nodes)      # a cheaper route, if not ruled out yet
        plan = found.best
        below = [tried[t] for t in range(spec.min_top, plan.top) if t in tried]
        open_ = [t + 1 for t in range(spec.min_top, plan.top)
                 if t not in tried or not tried[t].done]
        plan.optimal = found.done and not open_
        why = self.stopped or ("the search stopped at its budget" if spec.prove else
                               "the search stopped at the first plan: no proof asked")
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
        s = tried[top] = self._new(top)
        s.first_only = self.spec.quick_first
        self._run(s, self.spec.quick_nodes)
        if s.best is None and self.hint:
            self._run(s, self.spec.quick_nodes, legs=True)
        s.first_only = False
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
            last = tried.get(top - 1)
            if self.pool is not None and last is not None and (last.nodes > 200 or not last.done):
                # the sizes get dear: the next ones in workers, ruled out or not
                nxt = [top + 1, top + 2, min(top + (1 << jump), spec.max_top),
                       min(top + (1 << jump) + (2 << jump), spec.max_top)]
                self._expect([(t, "quick") for t in dict.fromkeys([top, *nxt])
                              if t not in tried and t <= spec.max_top])
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
                self._expect([(t, "quick") for t in range(top, max(top - 4, spec.min_top - 1), -1)
                              if t not in tried])
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
        """The deadline or the total node budget ran out (or the search gave up: ``gave_up``):
        no search starts or goes on."""
        return (self.deadline.expired or self.spent >= self.spec.max_total_nodes
                or bool(self.gave_up))

    @property
    def stopped(self) -> str:
        """Which of the two ran out, as a phrase (``""``: neither)."""
        if self.gave_up:
            return self.gave_up
        if self.deadline.expired:
            return f"the {self.spec.max_seconds:g} CPU s deadline ran out"
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
        if self.pool is not None:
            self.runs_of.setdefault(s.top, []).append((budget, legs, s.first_only))
        s.run(budget, legs, self.deadline)
        s.seconds += (time.monotonic() - t0) if isinstance(s, _Search) else s.last_seconds
        self.spent += s.nodes - before
        log.debug("%s: %d layers, %d nodes%s, %s", self.topo.name, s.top + 1, s.nodes,
                  " (a leg at a time)" if legs else "",
                  "found" if s.best else "none" if s.done else "budget")

    def plan(self, layers: Mapping[str, int], top: int,
             choices: Mapping[str, object] | None = None, heads: str | None = None
             ) -> StackPlan:
        """The plan of a layering (:func:`finalize`): its clearance gaps and layer
        thicknesses, every claim made at the z they give; :class:`PlanReject` (a
        ``ValueError``) when that can't be built. ``heads``: how the plan placed heads
        (:attr:`StackPlan.heads`; default: this problem's)."""
        heads = heads or self.heads
        if heads == "sink" and self.gap_groups:
            # the router's heads in their gaps, every other head sunk (as the search placed
            # them): the claims as they are, every other head forced into its layer, so
            # an axle crossing one of the router's gaps carries its washers there
            plan = finalize(self.topo, self.raw_claims, self.spec, layers, top, choices,
                            sink_all_but=self.gap_groups)
            plan.heads = heads
            return plan
        claims = (self.claims if heads == self.heads
                  else heads_claims(self.raw_claims, heads, self.gap_groups))
        plan = finalize(self.topo, claims, self.spec, layers, top, choices)
        plan.heads = heads
        return plan


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
        self.first_only = False       # stop at the first plan (a short search, quick_first)
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
                    if not any(not p.seat and not p.gap and p.layer in (0, top)
                               for p in shapes):
                        for p in shapes:
                            if not p.seat:
                                self.touch[n].setdefault(p.slot, []).append((v, p))
                        if self.router is not None:
                            bits = sum(1 << i for i, piece in enumerate(self.router.pieces)
                                       if any(p.layer == v and not p.seat and not p.gap
                                              and self.hit(piece, p.shape) for p in shapes))
                            self.allow[n][v] = self.router.states(n, bits)
                        else:
                            self.allow[n][v] = -1
                        continue
                self.dom[n].discard(v)
        self.lex: list[tuple[str, str]] | None = None   # (a, b): layer(a) <= layer(b)
        self.lexed: frozenset[str] = frozenset()

    # -- state --------------------------------------------------------------------

    def hit(self, a: Shape, b: Shape) -> bool:
        key = (a, b)
        h = self.pairs.get(key)
        if h is None:
            h = self.pairs[key] = self.geo.dist(a.core, b.core) < a.r + b.r + self.margin
        return h

    def effects(self, p: Placed) -> tuple[tuple[int, ...], tuple[tuple[str, int], ...]]:
        """(the router pieces ``p`` blocks, the (link, layer) choices it rules out)."""
        key = (p.shape, p.slot, p.group)
        fx = self.fx.get(key)
        if fx is None:
            k = p.slot
            pieces: tuple[int, ...] = ()
            if (self.router is not None and p.group != self.router.group and not p.gap
                    and 0 < k < self.top):
                pieces = tuple(i for i, piece in enumerate(self.router.pieces)
                               if self.hit(piece, p.shape))
            elif (self.router is not None and p.group != self.router.group and p.gap
                  and getattr(self.router, "gap_pieces", ())):
                # a head or washer in a clearance gap blocks the router's pieces there
                pieces = tuple(i for i, piece in enumerate(self.router.gap_pieces)
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
        if len(trail) <= mark:
            return
        dom, gone, banned, pending = self.dom, self.gone, self.banned, self.pending
        while len(trail) > mark:
            e = trail.pop()
            kind = e[0]
            if kind == "cut":
                if (e[1], e[2]) not in banned:
                    dom[e[1]].add(e[2])
                    del gone[e[1]][e[2]]
            elif kind == "shape":
                self.by_layer[e[1]].pop()
            elif kind == "pending":
                pending[e[1]] += 1
            elif kind == "block":
                lst = self.block[e[1]]
                lst.pop()
                if not lst:
                    k, i = e[1]
                    self.bmask[k] &= ~(1 << i)
            else:
                del self.layers[e[1]]

    def add(self, p: Placed, deps: frozenset[str]) -> frozenset[str] | None:
        """Place a shape: it must clear every other group's shape in its layer; then it
        blocks router pieces and removes the layers it rules out for unplaced links."""
        if p.seat:
            return None
        k, top = p.slot, self.top
        if not p.gap and k in (0, top):
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
        for a, b in self.lex if n in self.lexed else ():   # (lexed: empty without lex)
            if n not in (a, b):            # layer(a) <= layer(b): one layering of each orbit
                continue
            other, ks = (b, range(1, v)) if n == a else (a, range(v + 1, self.top))
            if other in self.layers:
                if self.layers[a] > self.layers[b]:
                    return frozenset({a, b})
            elif (c := self.close(other, ks, frozenset({n}))) is not None:
                return c | {n}
        if self.router is not None and self.dirty:     # else: as the last time it said yes
            res = self.router.check(self.view(partial=True))
            if isinstance(res, RouteConflict):
                if res.rules:
                    self.unbuilt[res.why] = self.unbuilt.get(res.why, 0) + 1
                return self.explain(res) | {n}
            # a layer none of whose states on a route a link's own shapes leave is closed to it
            layers, dom, gone, trail, get = self.layers, self.dom, self.gone, self.trail, res.get
            why = None
            for x in self.links:
                if x in layers:
                    continue
                allow = self.allow[x]
                d = dom[x]
                bad = [w for w in d if not get(w, -1) & allow[w]]
                if bad:
                    if why is None:
                        why = frozenset(layers)
                    g = gone[x]
                    for w in bad:
                        d.discard(w)
                        g[w] = why
                        trail.append(("cut", x, w))
                if not d:
                    return frozenset().union(*gone[x].values()) | {n}
        return None

    def spans(self, n: str) -> frozenset[str] | None:
        """An axle runs between its links' layers: a link that can't pass it can't sit between
        them, and once one sits on one side of some of them, the rest can't go to the other.
        A pillar also runs from them to a frame plate: such links can't be on both sides.
        (A domain these layers are already gone from is skipped: most are, an axle closes the
        same layers again at every node; an empty one still answers its conflict.)"""
        layers, dom, close = self.layers, self.dom, self.close
        for members, links, anchored in self.prob.spans.get(n, ()):
            placed = [m for m in members if m in layers]
            if not placed:
                continue
            ks = [layers[m] for m in placed]
            lo, hi = min(ks), max(ks)
            why = frozenset(placed)
            below = above = None
            inside = range(lo + 1, hi)
            for x in links:
                w = layers.get(x)
                if w is None:
                    d = dom[x]
                    if (not d or not d.isdisjoint(inside)) and (
                            c := close(x, inside, why)) is not None:
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
                    if m not in layers:
                        d = dom[m]
                        if (not d or not d.isdisjoint(side)) and (
                                c := close(m, side, why | {x})) is not None:
                            return c
            if anchored and (below or above):
                if below and above:
                    return why | {below, above}
                # the pillar must reach the plate on the other side
                other = range(hi + 1, self.top) if below else range(1, lo)
                for x in links:
                    if x not in layers:
                        d = dom[x]
                        if (not d or not d.isdisjoint(other)) and (c := close(
                                x, other, why | {below or above})) is not None:
                            return c
        return None

    def close(self, x: str, ks: range, why: frozenset[str]) -> frozenset[str] | None:
        """Take layers ``ks`` from unplaced ``x``'s domain; its conflict if none are left.
        (Most calls take nothing: an axle closes the same layers again at every node.)"""
        dom = self.dom[x]
        hit = dom.intersection(ks)
        if hit:
            dom -= hit
            gone, trail = self.gone[x], self.trail
            for u in hit:
                gone[u] = why
                trail.append(("cut", x, u))
        return None if dom else frozenset().union(*self.gone[x].values())

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

    LEAF_REROUTES = 4    # routes a leaf tries past its plan's gaps (see :meth:`leaf`)

    def leaf(self) -> frozenset[str]:
        """Every link has a layer: the final claims, the route, the plan checked.

        A route whose crankpin washers meet another group's shapes in a clearance gap the
        plan has (the router can't know which gaps a plan will have: the search would
        otherwise block every gap a pin's column crosses) is routed again with those gaps
        closed to that crankpin's run (``Router.washer_bit``), a few times; what is left
        fails the layering as before."""
        choices, cost = {}, 0
        extra: dict[float, int] = {}
        for attempt in range(self.LEAF_REROUTES + 1):
            if self.router is not None:
                view = self.view(partial=False)
                if extra:
                    bm = dict(view.blocked)
                    for slot, bits in extra.items():
                        bm[slot] = bm.get(slot, 0) | bits
                    view = replace(view, blocked=bm)
                res = self.router.route(view)
                if isinstance(res, RouteConflict):
                    if extra:           # the gaps closed: the whole layering is to blame
                        prob = self.prob
                        prob._tally_why(self.router.group, "crank route: its crankpin washers "
                                        "meet another group's in the plan's clearance gaps")
                        prob.washer_misses += 1
                        if (prob.give_up and not prob.found_any
                                and prob.washer_misses >= prob.give_up):
                            prob.gave_up = (f"gave up after {prob.washer_misses} layerings whose "
                                            "crank washers met another group's in the plan's "
                                            "gaps, with no plan found")
                            raise _Budget
                        return frozenset(self.layers)
                    if res.rules:
                        self.unbuilt[res.why] = self.unbuilt.get(res.why, 0) + 1
                    return self.explain(res)
                choices, cost = {self.router.group: res.choice}, res.cost
            try:
                plan = self.prob.plan(self.layers, self.top, choices)
            except PlanReject as e:
                # the layering clears at the nominal z, but not at its own (the clearance
                # gaps and the plates' thicknesses moved something a stock part had to fit)
                self.prob._tally_why("the plan at its z", str(e))
                return frozenset(self.layers)
            more = self._washer_blocks(plan, extra) if attempt < self.LEAF_REROUTES else {}
            if not more:
                break
            for slot, bits in more.items():
                extra[slot] = extra.get(slot, 0) | bits
        bad = verify_plan(plan)
        if bad:
            self.prob._tally_why("the plan at its z", bad[0])
            return frozenset(self.layers)
        plan.cost = cost
        self.best, self.bound = plan, cost
        self.prob.found_any = True
        if cost == 0:
            raise _Done
        if self.first_only:
            raise _Budget           # a short search has found what it looks for
        return frozenset(self.layers)

    def _washer_blocks(self, plan: StackPlan, have: Mapping[float, int]) -> dict[float, int]:
        """The router's washer bits (``washer_bit + j``) per gap slot where crankpin j's run
        washers meet another group's shape in a clearance gap ``plan`` has (at the search's
        sampling), beyond those in ``have``."""
        router = self.router
        wb = getattr(router, "washer_bit", -1) if router is not None else -1
        if wb < 0 or not plan.layout.gaps:
            return {}
        made_: list[Placed] = []
        for c in plan.claims:
            out, _ = made(c, plan.layout)
            if out is not None:
                made_.extend(out)
        shapes = [p for p in settle(made_, plan.sunk, plan.layout) if p.gap and not p.seat]
        index = {pt: j for j, pt in enumerate(router.points)}
        mine = [(p, index[p.shape.at]) for p in shapes
                if p.group == router.group and p.label.endswith(" washer")
                and isinstance(p.shape, Disc) and p.shape.at in index]
        out: dict[float, int] = {}
        for p, j in mine:
            bit = 1 << (wb + j)
            if have.get(p.slot, 0) & bit or out.get(p.slot, 0) & bit:
                continue
            if any(q.group != p.group and q.slot == p.slot and self.hit(p.shape, q.shape)
                   for q in shapes):
                out[p.slot] = out.get(p.slot, 0) | bit
        return out

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
            self.first = min(self.links, key=lambda x: (len(self.dom[x]),
                                                         self.prob.depth.get(x, 99), x))
        syms = getattr(self.prob, "syms", ())
        if syms and self.lex is None and (self.nodes or budget > self.prob.spec.quick_nodes):
            # a size worth more than a short search: the first link the search places, and
            # its images (a constraint added later prunes only what it would have: sound)
            from spiderpig.stack_symmetry import claims_commute

            first = self.first
            self.lex = sorted({(first, g[first]) for g, sig in syms if g[first] != first
                               and claims_commute(self.prob, g, sig, self.top)})
            self.lexed = frozenset(x for pair in self.lex for x in pair)
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
    """Re-check a plan from scratch: every claim re-evaluated at the plan's z, the heads it
    sank sunk again, every pair in one layer or one clearance gap tested, every gap and
    sunk head checked for height.

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
    made_: list[Placed] = []
    for c in plan.claims:
        out, why = made(c, layout)
        if out is None:
            bad.append(why)
            continue
        made_.extend(out)
    shapes = settle(made_, plan.sunk, layout)
    for n in topo.links:
        if n not in plan.layers:
            bad.append(f"{n} has no layer")
        elif not 0 < plan.layers[n] < plan.top:
            bad.append(f"{n} sits in layer {plan.layers[n]}, outside the frame plates")
    for p in shapes:
        if not p.seat and not p.gap and p.layer in (0, plan.top):
            bad.append(f"{p.label or p.group} sits in frame-plate layer {p.layer}")
        if p.height > 0:
            room = layout.gap(p.layer) if p.gap else layout.t(p.layer)
            if p.height > room + tol:
                where = f"the {room:g} mm gap over layer {p.layer}" if p.gap else \
                    f"layer {p.layer} ({room:g} mm)"
                bad.append(f"{p.label or p.group} needs {p.height:.2f} mm, more than {where}")
    by_slot: dict[float, list[Placed]] = {}
    for p in shapes:
        if not p.seat:
            by_slot.setdefault(p.slot, []).append(p)
    for slot, live in sorted(by_slot.items()):
        for a, b in itertools.combinations(live, 2):
            if a.group == b.group:
                continue
            need = a.shape.r + b.shape.r + sp.margin
            d = geo.dist(a.shape.core, b.shape.core)
            if d < need - tol:
                where = f"gap over layer {int(slot)}" if slot != int(slot) else f"layer {slot}"
                bad.append(f"{where}: {a.label or a.group} x {b.label or b.group} "
                           f"clear {d - a.shape.r - b.shape.r:.2f} mm (need {sp.margin:.2f})")
    return bad


# ---------------------------------------------------------------------------
# The plan's z: clearance gaps, sunk heads, layer thicknesses
# ---------------------------------------------------------------------------


class PlanReject(ValueError):
    """A layering whose plan can't be built at its own z (:func:`finalize`): a claim a
    clearance gap or a thicker plate moved can't be built, or a head needs a gap no stock
    sheet is thick enough for. The search takes it as a dead end."""


EPS_Z = 1e-6
SunkKey = tuple


def sunk_key(p: Placed) -> SunkKey:
    return (p.group, p.label, p.layer, p.shape.core, p.toward)


def _hit(geo: Geometry, a: Shape, b: Shape, margin: float) -> bool:
    return geo.dist(a.core, b.core) < a.r + b.r + margin - 1e-9


def bridges(shapes: Iterable[Placed], gaps: Mapping[int, float] | None = None) -> list[Placed]:
    """What runs on through a clearance gap: a group's piece (one core) in the layers on
    both sides of it is in the gap too, at the narrower of the two (a crank stack, a
    hub, a post); a group that put shapes of its own at that core in the gap (an axle's
    washers) is left as it said. Only the gaps in ``gaps`` (all, when ``None``)."""
    cores: dict[tuple, dict[int, float]] = {}
    own = set()
    for p in shapes:
        if p.gap:
            own.add((p.group, p.shape.core, p.layer))
            continue
        d = cores.setdefault((p.group, p.shape.core), {})
        r, t = d.get(p.layer, (0.0, math.inf))
        d[p.layer] = (max(r, p.shape.r), min(t, p.sheet))
    out = []
    for (g, core), d in cores.items():
        for k, (r, t) in sorted(d.items()):
            if k + 1 not in d or (g, core, k) in own:
                continue
            if gaps is not None and not gaps.get(k):
                continue
            rr = min(r, d[k + 1][0])
            shape = Disc(core[1], rr) if core[0] == "pt" else Pill(core[1], core[2], rr)
            # a plate stack on both sides (``sheet``: the thinner): a filler plate fills it
            out.append(Placed(k, shape, g, f"{g} through the gap", gap=True,
                              sheet=min(t, d[k + 1][1])))
    return out


def settle(shapes: Iterable[Placed], sunk: Iterable[SunkKey], layout: Layout) -> list[Placed]:
    """The claims' shapes as the plan builds them: the heads in ``sunk`` moved into the layer
    they sink into, the shapes of a gap the plan doesn't have dropped, and what runs on
    through the gaps it has added (:func:`bridges`)."""
    sunk = set(sunk)
    out = []
    for p in shapes:
        if p.gap and p.toward and sunk_key(p) in sunk:
            out.append(replace(p, layer=p.layer + (1 if p.toward > 0 else 0), gap=False))
        elif p.gap and not layout.gap(p.layer):
            continue
        else:
            out.append(p)
    return out + bridges(out, layout.gaps)


def _sinkable(shapes: list[Placed], layout: Layout, geo: Geometry, margin: float
              ) -> set[SunkKey]:
    """The heads that go into the layer beside their gap instead (the full-layer head):
    a gap goes when every head in it fits the layer it would sink into (not a frame
    plate, tall enough, clear of every other group's shape there and of the heads
    already sunk into it), going up the stack."""
    top = layout.top
    plate: dict[int, list[Placed]] = {}
    heads: dict[int, list[Placed]] = {}
    for p in shapes:
        if p.seat:
            continue
        if not p.gap:
            plate.setdefault(p.layer, []).append(p)
        elif p.height > 0:
            heads.setdefault(p.layer, []).append(p)
    sunk: set[SunkKey] = set()
    for b in sorted(heads):
        moves = []
        for h in heads[b]:
            tgt = b + 1 if h.toward > 0 else b if h.toward < 0 else None
            if tgt is None or not 0 < tgt < top or h.height > layout.t(tgt) + EPS_Z:
                break
            if any(q.group != h.group and _hit(geo, h.shape, q.shape, margin)
                   for q in [*plate.get(tgt, ()), *(m for t, m in moves if t == tgt)]):
                break
            moves.append((tgt, h))
        else:
            for tgt, h in moves:
                plate.setdefault(tgt, []).append(h)
                sunk.add(sunk_key(h))
    return sunk


def _thicknesses(layers: Mapping[str, int], shapes: Iterable[Placed], spec: StackSpec,
                 top: int) -> dict[int, float]:
    """Each layer's thickness where it isn't the pitch: the frame plates', the thickest
    link's or plate's in it."""
    t: dict[int, float] = {}
    if spec.frame_t is not None:
        t[0] = t[top] = spec.frame_t
    lt = dict(spec.link_t)
    for n, k in layers.items():
        if n in lt:
            t[k] = max(t.get(k, spec.pitch), lt[n])
    for p in shapes:
        if p.sheet > 0 and not p.gap and 0 < p.layer < top:
            t[p.layer] = max(t.get(p.layer, spec.pitch), p.sheet)
    return {k: v for k, v in t.items() if abs(v - spec.pitch) > EPS_Z}


def _gap_options(k: int, h: float, spec: StackSpec, bridged: set[int]) -> list[float]:
    """The thicknesses gap ``k`` may have for heads ``h`` tall, thinnest first: a thin
    sheet's where plates go on through it (``bridged``), else any :data:`GAP_STEP` up to
    :data:`GAP_MORE` over the need."""
    if k in bridged:
        return [o for o in sorted(spec.gaps) if o >= h - EPS_Z]
    if h > GAP_MAX + EPS_Z:
        return []
    g = math.ceil(h / GAP_STEP - 1e-6) * GAP_STEP
    return [round(g + i * GAP_STEP, 3) for i in range(int(GAP_MORE / GAP_STEP) + 1)
            if g + i * GAP_STEP <= max(GAP_MAX, g) + EPS_Z]


def _gap_sizes(shapes: Iterable[Placed], spec: StackSpec, top: int,
               bridged: set[int] = frozenset()) -> dict[int, float]:
    """Each clearance gap the heads in it need: the thinnest it may be over the tallest
    (:func:`_gap_options`; :class:`PlanReject` when none is tall enough)."""
    need: dict[int, float] = {}
    for p in shapes:
        if p.gap and p.height > 0:
            need[p.layer] = max(need.get(p.layer, 0.0), p.height)
    out = {}
    for k, h in need.items():
        if not 0 <= k < top:
            raise PlanReject(f"a clearance gap over layer {k} is outside the frame plates")
        g = next(iter(_gap_options(k, h, spec, bridged)), None)
        if g is None:
            who = sorted({p.label or p.group for p in shapes
                          if p.gap and p.layer == k and p.height > h - EPS_Z})
            most = max(spec.gaps) if k in bridged else GAP_MAX
            raise PlanReject(f"{', '.join(who)} needs a {h:.2f} mm clearance gap over layer "
                             f"{k}, more than the thickest it may have ({most:g} mm)")
        out[k] = g
    return out


def _make_all(claims: Iterable[Claim], layout: Layout) -> list[Placed]:
    out: list[Placed] = []
    for c in claims:
        got, why = made(c, layout)
        if got is None:
            e = PlanReject(why)
            e.claim = c
            raise e
        out.extend(got)
    return out


def plate_bridged(shapes: Iterable[Placed]) -> set[int]:
    """The gaps a stack of plates runs on through (a group's plates at one core in the
    layers on both sides): a filler plate fills such a gap, so it is a thin sheet's
    thickness; any other gap is a stack of washers and shims, so any 0.1 mm."""
    plates: dict[tuple, set[int]] = {}
    for p in shapes:
        if p.sheet > 0 and not p.gap:
            plates.setdefault((p.group, p.shape.core), set()).add(p.layer)
    return {k for ks in plates.values() for k in ks if k + 1 in ks}


GAP_STEP = 0.1        # a gap only washers and shims fill: any multiple of the thinnest shim
GAP_MAX = 4.0         # ... up to this (a head with its shims; a taller one keeps a layer)
GAP_TRIES = 60        # thicker gaps a plan's z tries for a claim that fails (finalize)
GAP_MORE = 3.0        # the most a plan's z thickens a gap past its heads' need (a layer's worth)


class _ReadGaps(Mapping):
    """A layout's gaps that note every gap a claim read (``read``: layer -> thickness).
    Only values are noted: the keys, and so ``len``, iteration and membership, are the
    same in every try of :func:`_thicker_gaps`."""

    __slots__ = ("_gaps", "read")

    def __init__(self, gaps: Mapping[int, float]):
        self._gaps = gaps
        self.read: dict[int, float] = {}

    def __getitem__(self, k: int) -> float:
        v = self._gaps[k]
        self.read[k] = v
        return v

    def get(self, k, default=None):
        return self[k] if k in self._gaps else default

    def __contains__(self, k) -> bool:
        return k in self._gaps

    def __iter__(self):
        return iter(self._gaps)

    def __len__(self) -> int:
        return len(self._gaps)


def _thicker_gaps(err: PlanReject, spec: StackSpec, layers, top: int, choices,
                  gaps: dict[int, float], thick: dict[int, float],
                  bridged: set[int]) -> dict[int, float]:
    """The gaps thickened the least (one gap at a time, then two) at which the claim that
    failed (``err.claim``) builds, among the :data:`GAP_TRIES` least thickenings; ``err``
    again when none does. Bounded on purpose: the search takes the first layering that
    builds at a size, so a layering let through on a far thicker gap would end it on a
    taller plan than the next layering gives (measured: a 37.6 mm single module went 44.2
    mm when every single thickening was tried)."""
    claim = getattr(err, "claim", None)
    if claim is None:
        raise err
    ks = sorted(gaps)
    more = {k: [o for o in _gap_options(k, gaps[k], spec, bridged) if o > gaps[k] + EPS_Z]
            for k in ks}
    # every thickening (its total, then what it changes), in the order of sorting the gap
    # dicts by (rounded total, sorted items) and keeping the first GAP_TRIES: only those
    # whose total is within rounding (1e-5) of the GAP_TRIES-th least can be among them, and
    # every gap dict has the keys ``ks``, so its items compare as its values in that order
    cands: list[tuple[float, tuple]] = []
    for k in ks:
        cands += [(o - gaps[k], ((k, o),)) for o in more[k]]
    for a, b in itertools.combinations(ks, 2):
        cands += [(oa + ob - gaps[a] - gaps[b], ((a, oa), (b, ob)))
                  for oa in more[a][:8] for ob in more[b][:8]]
    if len(cands) > GAP_TRIES:
        cut = heapq.nsmallest(GAP_TRIES, (c[0] for c in cands))[-1] + 1e-5
        cands = [c for c in cands if c[0] <= cut]
    keyed = []
    for i, (d, how) in enumerate(cands):
        g = {**gaps, **dict(how)}
        keyed.append(((round(d, 6), tuple(g[k] for k in ks), i), d, g))
    keyed.sort(key=lambda t: t[0])
    tries = [(d, g) for _, d, g in keyed[:GAP_TRIES]]
    # A claim's ``make`` is a function of what it reads of its layout: a try that agrees
    # with a failed one on every gap that one read fails too, and is skipped (the gaps
    # read through :class:`_ReadGaps`; every try has the same keys, layers and thicknesses)
    failed: list[dict[int, float]] = []
    seen = _ReadGaps(gaps)
    if made(claim, Layout(layers, top, spec.pitch, choices, seen, dict(thick),
                          final=True))[0] is None:
        failed.append(seen.read)
    for _, g in tries:
        if any(all(g[k] == v for k, v in r.items()) for r in failed):
            continue
        seen = _ReadGaps(g)
        layout = Layout(layers, top, spec.pitch, choices, seen, dict(thick), final=True)
        if made(claim, layout)[0] is not None:
            return g
        failed.append(seen.read)
    most = max((t[0] for t in tries), default=0.0)
    raise PlanReject(f"{err} (nor with its clearance gaps thickened by up to {most:.2g} mm "
                     f"in all: the {len(tries)} least thickenings of one or two gaps)")


HEADS_ORDER = {"best": ("sink", "gap"), "gap_sink": ("gap", "sink")}

GIVE_UP = 200
"""(``gap_sink``) The first gap search gives up for the sunk one after this many layerings
failed on the crank's washers in the plan's gaps with no plan found (TrotBot's heel meets
~65 a CPU second; the designs that plan in gaps meet a handful first)."""
"""The heads searches :meth:`StackProblem.solve_heads` tries in turn, per
:attr:`StackSpec.heads`."""

GAP_GROUPS = ("drive",)
"""Groups whose heads stay in their clearance gaps beside a router's (the servo's, the
frame ties' and the deck rails' screws under the inner plate: the crank's horn screws
share that gap), when the other heads sink (:func:`heads_claims`)."""


def heads_claims(claims: Iterable[Claim], heads: str, keep: frozenset[str] = frozenset()
                 ) -> tuple[Claim, ...]:
    """The claims as the search sees them for ``heads`` (:attr:`StackSpec.heads`): "sink"
    puts every head that may sink into the layer beside its link (and drops what would
    only be in a gap: an axle's washers), "gap" leaves them. ``keep``: groups whose shapes
    stay as they are either way (a single-plate crank's, whose router places its heads in
    gaps: sunk, they would stand in a rider's layer; :data:`GAP_GROUPS`); the plan's z
    gives what crosses their gaps its washers (:meth:`StackProblem.plan`)."""
    claims = tuple(claims)
    if heads != "sink":
        return claims

    def sunk(make):
        if make is None:
            return None

        def f(L: Layout):
            out = make(L)
            if out is None:
                return None
            got = [p if p.group in keep else
                   replace(p, layer=p.layer + (1 if p.toward > 0 else 0), gap=False)
                   if p.gap and p.toward else p
                   for p in out if not p.gap or p.toward or p.group in keep]
            for p in got:
                if p.height > L.t(p.layer) + EPS_Z:
                    raise Unbuildable(f"{p.label or p.group} needs {p.height:.2f} mm, more "
                                      f"than layer {p.layer} ({L.t(p.layer):g} mm)")
            return got
        return f

    return tuple(replace(c, make=sunk(c.make), early=sunk(c.early)) for c in claims)


def finalize(topo: Topology, claims: Iterable[Claim], spec: StackSpec,
             layers: Mapping[str, int], top: int,
             choices: Mapping[str, object] | None = None,
             sink_all_but: frozenset[str] | None = None) -> StackPlan:
    """The plan of a layering: which heads sink into a layer and which keep a clearance
    gap (:func:`_sinkable`), each gap's stock thickness and each layer's (the plates'
    sheets), and every claim made again at the z those give, until they settle (a
    claim's head may need more at its real z: a Chicago screw's shims). Deterministic in
    its arguments, so a stored layering re-makes the same plan. ``sink_all_but``: every
    head but those groups' sinks into its layer (the search placed them there:
    :meth:`StackProblem.plan` with a router's heads in gaps), unless the plan's z or a gap
    kept beside it says otherwise."""
    claims = tuple(claims)
    choices = dict(choices or {})
    layers = dict(layers)
    nominal = Layout(layers, top, spec.pitch, choices)
    shapes = _make_all(claims, nominal)
    sunk = _sinkable(shapes, nominal, topo.geometry, spec.margin)
    if sink_all_but is not None:
        sunk |= {sunk_key(p) for p in shapes
                 if p.gap and p.toward and p.group not in sink_all_but}
    gaps: dict[int, float] = {}
    thick: dict[int, float] = {}
    layout = nominal
    for _ in range(8):
        # a head that needs more at the plan's z than the layer it sank into keeps its gap
        sunk -= {sunk_key(p) for p in shapes if p.gap and p.toward and sunk_key(p) in sunk
                 and p.height > layout.t(p.layer + (1 if p.toward > 0 else 0)) + EPS_Z}
        bridged = plate_bridged(shapes)
        while True:
            kept = [p for p in shapes if not (p.gap and p.toward and sunk_key(p) in sunk)]
            want = _gap_sizes(kept, spec, top, bridged)
            # a head can't sink past a gap the plan keeps there (it hangs off its link's
            # face): every head in a gap the others keep stays in it
            have = set(want) | {k for k, v in gaps.items() if v}
            back = {sunk_key(p) for p in shapes if p.gap and p.toward
                    and sunk_key(p) in sunk and p.layer in have}
            if not back:
                break
            sunk -= back
        new_gaps = {k: max(v, gaps.get(k, 0.0)) for k, v in {**gaps, **want}.items()}
        new_thick = _thicknesses(layers, shapes, spec, top)
        new_thick = {k: max(v, thick.get(k, 0.0)) for k, v in {**thick, **new_thick}.items()}
        if layout is not nominal and new_gaps == gaps and new_thick == thick:
            break
        gaps, thick = new_gaps, new_thick
        layout = Layout(layers, top, spec.pitch, choices, dict(gaps), dict(thick), final=True)
        try:
            shapes = _make_all(claims, layout)
        except PlanReject as e:
            if not gaps:
                raise
            # a stock part (a crank bolt, a standoff) that misses at these gaps may fit at
            # thicker ones: the least thickening that builds the claim, then all of them
            gaps = _thicker_gaps(e, spec, layers, top, choices, gaps, thick, bridged)
            layout = Layout(layers, top, spec.pitch, choices, dict(gaps), dict(thick),
                            final=True)
            shapes = _make_all(claims, layout)
    else:
        raise PlanReject("the clearance gaps and the claims made at their z don't settle")
    placed = settle(shapes, sunk, layout)
    return StackPlan(spec, layers, top, topo, claims, tuple(placed), choices,
                     gaps=dict(gaps), thick=dict(thick), sunk=frozenset(sunk))
