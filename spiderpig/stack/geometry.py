"""The planner's geometry: batched planar distances, shapes, claims, the layout."""


from __future__ import annotations

import logging
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from functools import cached_property

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
