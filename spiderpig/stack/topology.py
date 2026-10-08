"""What the planner knows of a mechanism: links, axes and the crank (from a
kinematic template), static clearances, plan errors, and the routed groups'
protocol (:class:`Router`)."""


from __future__ import annotations

import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Literal, Protocol

import numpy as np

from spiderpig.mechanism import is_crank, is_frame, is_link
from spiderpig.stack.geometry import Geometry

if TYPE_CHECKING:
    from spiderpig.stack.geometry import Core, Layout, Shape


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
        point_of.update(dict.fromkeys(nodes, axis.name))
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
    blocked: Mapping[float, int]        # by slot (a layer; a gap's ``layer + 0.5``)
    open: Mapping[str, set[int]] | None = None
    bound: int | None = None
    open_bits: Mapping[str, int] | None = None     # ``open`` as bits over the layers


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
    # the crank's router's (the search reads them with getattr defaults): the pieces it may
    # put in a clearance gap, its posts' points, and its first washer bit (-1: none)
    gap_pieces: tuple[Shape, ...]
    points: list[str]
    washer_bit: int

    def check(self, view: RouteView) -> RouteConflict | Mapping[int, int]: ...

    def states(self, link: str, blocked: int) -> int: ...

    def route(self, view: RouteView) -> Route | RouteConflict: ...
