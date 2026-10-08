"""The crank's route through the stack: what the planner may choose, and the exact choice.

The crankshaft is one rigid body routed through layers ``1..`` up to the hub
under the servo horn (:mod:`construction.crank`). In every layer it is one
of:

* ``S``: the **stub** on O, below everything, down into the outer frame
  plate (the bottom bearing);
* ``E``: nothing (below everything, when the bearing is dropped);
* ``L``: the **lowest web**, the journal on O plus a web to the run above;
* ``J``: the **journal** on O, with a web to each run next to it;
* ``R(p)``: a **run** along the post at ``p``: a crankpin (its riders turn on
  it) or a detour point fixed to the crank. O is free there.

and the hub's layers end it. A link that sweeps across O ("needs O free")
can only sit in a run layer whose post it clears.

The route must also be buildable (:class:`JointRules`, from the bolt crank's
standoffs, :meth:`construction.crank.BoltCrank.joint_rules`): the runs along one
point form one **chain**, one standoff between the chain's lowest web and its
highest, so the webs' span must take a stock standoff; two chains share a web or a
journal standoff on O joins them, and the last ends in the hub's layer
(``j_last``); the screw heads beyond a chain's webs need their clearance gaps
(``gap_head``); and the pockets of two consecutive chains (or the last chain's and
the horn screws') must not meet.

:meth:`CrankRouter.route` finds, for one layering, the cheapest route whose
pieces clear everything else in their layers: a shortest path over layers
and chains. Cost, in order: added features (a run layer no rider of its point
needs, a detour run), how far detours reach below O (their sweep), a dropped
bearing, then fewer runs. :meth:`CrankRouter.check` is the relaxation a
partial layering gets (states per layer, joints ignored).

:func:`crank_facts` is the static part (before any layer is chosen): which
links need O free, which run points they clear, detour points for those no
crankpin clears, inside the body's underside (:mod:`construction.underside`),
and what stops the pipeline when a link has no crank point at all.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from itertools import repeat

import numpy as np

from spiderpig.construction.base import Context
from spiderpig.construction.crank import (
    CrankDims,
    CrankRoute,
    Run,
    hub_layers,
)
from spiderpig.construction.underside import Underside
from spiderpig.stack import Disc, Layout, Pill, Route, RouteConflict, RouteView

# States are bits: S 1, E 2, L 4, J 8, R(point j) 16 << j. Pieces: the stub (bit 0) and the
# journal (1) on O, a post on each run point (2 + j), a web to each (2 + n + j).
FEATURE = 10**12                 # cost weights, lexicographic
SWEEP = 10**4                    # per 0.01 mm a detour reaches below O
BEARING = 10**3
RUN = 1                          # a tie-break within one layering: not part of Route.cost
INF = math.inf
class _Miss(enum.Enum):
    MISS = 0


_MISS = _Miss.MISS               # a memo miss (a memo may hold None)


def _first(pair: tuple):
    return pair[0]


def _second(pair: tuple):
    return pair[1]


@dataclass(frozen=True)
class Detour:
    """A point fixed to the crank, ``r`` from O and ``angle`` degrees counter-clockwise
    from the first crankpin: a run there sweeps a circle of radius ``sweep`` about O."""

    name: str
    r: float
    angle: float
    sweep: float


@dataclass(frozen=True)
class JointRules:
    """What a buildable crankshaft asks of its route.

    ``spans[n]``: whether a stock standoff fits a chain whose webs span ``n``
    layers (the lowest and highest included), for each set of faces set back
    by the riders' end play, bit ``16a + 4b + 2c + d``: the lowest web's bottom
    face (a rider under it) and top face (a rider on the chain's first run
    layer), the highest web's bottom face (a rider on the last run layer, the
    run not all riders) and top face (a rider over it). Pocket radii (mm): a
    screw head's, a nut's (the bolt crank's: its screw heads, both), a post.
    Horn pockets, fixed on the crank: ``(point on the crank at sample 0, radius)``.
    """

    spans: dict[int, int]
    head: float
    nut: float
    post: float
    horn: tuple[tuple[tuple[float, float], float], ...] = ()
    bottom_layers: frozenset[int] = frozenset()   # the first chain's lowest web may sit in
    #                               these layers only (empty: any; the bolt crank's stub
    #                               standoff comes in stock lengths)
    gap_head: float = 0.0         # a chain's screw heads sit in the clearance gaps beyond
    #                               its outer webs (and its spacer over the lowest): the
    #                               radius they need clear there (0: none)
    horn_heads: tuple[tuple[str, float], ...] = ()   # crank points of the horn screws' heads,
    #                               in the gap under the hub, and their radius
    j_spans: tuple[tuple[int, bool], ...] = ()   # layers a journal standoff between two
    #                               chains may span: whether a stock one fits. (The bolt
    #                               crank's chains: one run each, no plate between two runs of
    #                               a point (it would turn loose on the standoff); two chains
    #                               share a plate or a journal standoff on O joins them; the
    #                               last ends in the hub's layer)
    gap_washer: float = 0.0       # (gap_head) the washers a chain's run carries through every
    #                               clearance gap along it (its first run layer up): the
    #                               radius they need clear there (0: none). Not a gap piece
    #                               the search blocks (every gap a pin's column crosses
    #                               would then close the crank's runs, a gap the plan may
    #                               never have): the planner's leaf sets these bits
    #                               (``CrankRouter.washer_bit``) for the gaps its plan has
    #                               where the run's washers met another group's, and routes
    #                               again (stack._Search.leaf)


def joint_rules(construction, ctx: Context, dims: CrankDims) -> JointRules:
    """The joint rules a crank construction asks of its route (its own ``joint_rules``)."""
    return construction.joint_rules(ctx, dims)


def horn_pockets(ctx: Context) -> tuple[tuple[tuple[float, float], float], ...]:
    """The horn screws' (and the horn's centre screw's) pockets in the hub, fixed on the
    crank: ``(point at sample 0, radius)``."""
    drive = ctx.interfaces["drive"]
    g = ctx.topo.geometry.points
    o = g["O"][0]
    pin = g[ctx.topo.axes_of("crankpin")[0].name][0] - o
    theta = math.atan2(pin[1], pin[0]) + drive.pattern_angle
    horn = [((float(o[0] + drive.screw_pcd / 2 * math.cos(a)),
              float(o[1] + drive.screw_pcd / 2 * math.sin(a))), drive.screw_head_d / 2)
            for a in (theta + 2 * math.pi * k / drive.screw_count
                      for k in range(drive.screw_count))]
    if drive.center_head_d > 0:
        horn.append(((float(o[0]), float(o[1])),
                     (drive.center_head_d + ctx.params.print_fit) / 2))
    return tuple(horn)


@dataclass(frozen=True)
class NoCrankPoint:
    """A link that needs O free and that no crank point clears: the static stage stops.

    ``pin`` is the crankpin it misses by the least, ``dist`` how close it comes
    (a lower bound over the cycle); a post there needs ``post + link_r +
    margin``. ``detour``: the nearest point fixed to the crank that clears it,
    ``(r, first, last)`` (mm from O, degrees from ``pin``), whose run sweeps
    ``r + reach`` about O, more than the envelope's ``allow``.
    """

    link: str
    pin: str
    dist: float
    post: float
    link_r: float
    margin: float
    detour: tuple[float, float, float] | None
    reach: float
    allow: float

    @property
    def need(self) -> float:
        return self.post + self.link_r + self.margin

    def describe(self) -> str:
        out = (f"{self.link} sweeps right across the crank at O, so its layer needs the crank "
               f"off its axis, and no crank point clears it: it passes crankpin {self.pin} at "
               f"{max(self.dist, 0.0):.1f} mm, under the {self.need:.1f} mm a post there needs "
               f"({self.post:g} post radius + {self.link_r:g} link half-width + {self.margin:g} "
               "margin)")
        if self.detour is None:
            return out + "; no point within 150 mm of O clears it"
        r, a0, a1 = self.detour
        where = f"{a0:.0f}°" if a0 == a1 else f"{a0:.0f}°..{a1:.0f}°"
        return out + (f"; the nearest point off the crankpins that clears it is {r:.0f} mm from "
                      f"O ({where} from {self.pin}), where a run would sweep {r + self.reach:.0f} "
                      "mm about O, below the body's underside, which lets the crank sweep "
                      f"{self.allow:.1f} mm (a run at most {self.allow - self.reach:.1f} mm out)")


@dataclass
class CrankFacts:
    """What the geometry alone says about the crank and the links (pass 1)."""

    o_free: dict[str, float] = field(default_factory=dict)   # link -> closest approach to O
    hosts: dict[str, list[str]] = field(default_factory=dict)  # O-free link -> run points it clears
    detours: list[Detour] = field(default_factory=list)
    envelope: Underside | None = None
    allow: float = math.inf         # largest sweep about O the envelope allows
    failures: list[NoCrankPoint] = field(default_factory=list)


def _crank_frame(topo, samples: np.ndarray) -> np.ndarray:
    """Angle of the first crankpin about O at each sample."""
    g = topo.geometry.points
    d = g[topo.axes_of("crankpin")[0].name] - g["O"]
    return np.arctan2(d[:, 1], d[:, 0])[samples]


def point_clearance(topo, link: str, r: float, angles_deg: np.ndarray,
                    stride: int = 1) -> np.ndarray:
    """Lower bound on how close ``link`` comes, over the cycle, to points fixed to the crank
    ``r`` from O at ``angles_deg`` from the first crankpin (every ``stride``-th sample, less
    half the most the link moves between those, in the crank's frame)."""
    g = topo.geometry.points
    idx = np.arange(0, topo.geometry.samples, stride)
    o = g["O"][idx]
    phi = _crank_frame(topo, idx)
    c, s = np.cos(-phi)[:, None], np.sin(-phi)[:, None]

    def local(name):                      # the link's points in the crank's frame
        d = g[name][idx] - o
        return np.concatenate([c * d[:, :1] - s * d[:, 1:], s * d[:, :1] + c * d[:, 1:]], axis=1)

    th = np.radians(angles_deg)
    p = (r * np.stack([np.cos(th), np.sin(th)], -1))[:, None, :]
    best = np.full(len(th), np.inf)
    for a, b in topo.links[link]:
        A, B = local(a), local(b)
        step = max(np.linalg.norm(np.roll(A, -1, 0) - A, axis=1).max(),
                   np.linalg.norm(np.roll(B, -1, 0) - B, axis=1).max())
        ab = B - A
        u = np.clip(((p - A) * ab).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-18), 0.0, 1.0)
        d = np.linalg.norm(p - (A + ab * u[..., None]), axis=-1).min(axis=1) - step / 2
        best = np.minimum(best, d)
    return best


def crank_facts(ctx: Context, dims: CrankDims, envelope: Underside | None,
                margin: float) -> CrankFacts:
    """Links that need O free, the run points each clears, and detours for the rest.

    A detour must clear the link it lets through by post radius + link
    half-width + margin, and its sweep (its distance from O plus the wider of
    its post and its webs) must fit the envelope. When no crankpin and no
    detour inside the envelope clears a link, :attr:`CrankFacts.failures` says
    so with the numbers.
    """
    topo, r = ctx.topo, ctx.params.link_radius
    geo = topo.geometry
    pins = [a.name for a in topo.axes_of("crankpin")]
    riders = topo.riders
    facts = CrankFacts(envelope=envelope,
                       allow=envelope.allows(margin) if envelope is not None else math.inf)
    reach = max(dims.post, dims.web)
    for link, segs in topo.links.items():
        d = min(geo.dist(("pt", "O"), ("seg", a, b)) for a, b in segs)
        if d >= dims.journal + r + margin:
            continue
        facts.o_free[link] = d
        need = dims.post + r + margin
        facts.hosts[link] = [p for p in pins if riders.get(link) == p or min(
            geo.dist(("pt", p), ("seg", a, b)) for a, b in segs) >= need]
    lonely = [n for n, h in facts.hosts.items() if not h]
    angles = np.arange(0.0, 360.0, 2.5)
    need = dims.post + r + margin
    for n in lonely:
        # the nearest ring of points fixed to the crank with one that clears it
        first = None
        for rad in np.arange(1.0, 151.0, 1.0):
            ok = point_clearance(topo, n, rad, angles) >= need
            if not ok.any():
                continue
            first = first or (rad, angles[ok])
            if rad + reach > facts.allow:       # beyond the envelope: no ring further out will do
                break
            # the lowest sweep; on it, the point that also clears most other links needing
            # O free, and that the planner's own distances clear
            others = [point_clearance(topo, m, rad, angles) >= need for m in facts.o_free]
            js = sorted(np.flatnonzero(ok), key=lambda j: -sum(o[j] for o in others))
            j = next((j for j in js if _clears(topo, n, rad, angles[j]) >= need), None)
            if j is None:
                continue
            if not any(d.r == rad and d.angle == angles[j] for d in facts.detours):
                name = topo.add_crank_point(f"crank.detour{len(facts.detours)}", float(rad),
                                            float(angles[j]))
                facts.detours.append(Detour(name, float(rad), float(angles[j]),
                                            float(rad + reach)))
            break
        if not any(d.name in facts.hosts[n] or _clears(topo, n, d.r, d.angle) >= need
                   for d in facts.detours):
            facts.failures.append(_no_point(ctx, dims, n, margin, *(first or (None, angles)),
                                            facts.allow, reach))
    geo = topo.geometry                # with the detours added
    for d in facts.detours:            # which O-free links each detour clears
        for n in facts.o_free:
            if min(geo.dist(("pt", d.name), ("seg", a, b)) for a, b in topo.links[n]) >= need:
                facts.hosts[n].append(d.name)
    return facts


def _clears(topo, link: str, r: float, angle: float) -> float:
    """How close ``link`` comes to a point fixed to the crank, by the planner's own lower
    bound (:meth:`stack.Geometry.dist`)."""
    g = topo.geometry.points
    pin = g[topo.axes_of("crankpin")[0].name] - g["O"]
    theta = np.arctan2(pin[:, 1], pin[:, 0]) + math.radians(angle)
    at = g["O"] + r * np.stack([np.cos(theta), np.sin(theta)], axis=-1)
    names = {q for seg in topo.links[link] for q in seg}
    geo = type(topo.geometry)({**{q: g[q] for q in names}, "_at": at})
    return min(geo.dist(("pt", "_at"), ("seg", a, b)) for a, b in topo.links[link])


def _no_point(ctx, dims, link, margin, rad, clear, allow, reach) -> NoCrankPoint:
    """``rad``: the nearest ring with points that clear ``link`` (None: none within 150 mm),
    ``clear`` their angles from the first crankpin."""
    topo = ctx.topo
    geo = topo.geometry
    miss = {p.name: min(geo.dist(("pt", p.name), ("seg", a, b)) for a, b in topo.links[link])
            for p in topo.axes_of("crankpin")}
    pin = _own_pin(topo, link)
    pin = pin if miss[pin] >= max(miss.values()) - 1e-9 else max(miss, key=miss.__getitem__)
    detour = None
    if rad is not None:
        rel = sorted((clear - _pin_angle(topo, pin)) % 360.0)
        # the clear arc: from after its widest gap round to before it (it may wrap past 0)
        gap = max(range(len(rel)), key=lambda j: (rel[j] - rel[j - 1]) % 360.0)
        detour = (float(rad), float(rel[gap]), float(rel[gap - 1]))
    return NoCrankPoint(link, pin, miss[pin], dims.post, ctx.params.link_radius, margin, detour,
                        reach, allow)


def _ranges(ns: list[int]) -> str:
    """``[3, 4, 5, 6, 9, 11]`` -> ``"3-6, 9 or 11"``."""
    parts: list[list[int]] = []
    for n in ns:
        if parts and parts[-1][1] == n - 1:
            parts[-1][1] = n
        else:
            parts.append([n, n])
    words = [f"{a}-{b}" if b > a + 1 else f"{a}, {b}" if b > a else f"{a}" for a, b in parts]
    return ", ".join(words[:-1]) + " or " + words[-1] if len(words) > 1 else words[0]


def _own_pin(topo, link) -> str:
    """The crankpin of the leg ``link`` belongs to (its suffix), else the first."""
    pins = [a.name for a in topo.axes_of("crankpin")]
    leg = link.rsplit("_", 1)[1] if "_leg" in link else None
    return next((p for p in pins if leg and p.endswith(leg)), pins[0])


def _pin_angle(topo, pin: str) -> float:
    """``pin``'s angle from the first crankpin (degrees, fixed on the crank)."""
    g = topo.geometry.points
    a = [g[p][0] - g["O"][0] for p in (topo.axes_of("crankpin")[0].name, pin)]
    return math.degrees(math.atan2(a[1][1], a[1][0]) - math.atan2(a[0][1], a[0][0])) % 360.0


class CrankRouter:
    """The planner's :class:`stack.Router` for the crank: the exact route for a layering.

    Pieces, in order: the stub and the journal on O, a post on each run point,
    a web to each run point. States are bits: ``S``, ``E``, ``L``, ``J``, then
    ``R(j)`` at bit ``4 + j``.
    """

    group = "crank"
    MEMO_ROUTES = 100_000        # routes remembered before the memo is dropped

    def __init__(self, ctx: Context, dims: CrankDims, facts: CrankFacts, drop_bearing: bool,
                 rules: JointRules):
        topo = ctx.topo
        self.drive = ctx.interfaces["drive"]
        self.pitch = ctx.pitch
        self.dims = dims
        self.pins = [a.name for a in topo.axes_of("crankpin")]
        self.points = self.pins + [d.name for d in facts.detours]
        self.n = n = len(self.points)
        self.full = (1 << n) - 1
        self.sweep = [0] * len(self.pins) + [round(d.sweep * 100) for d in facts.detours]
        index = {p: j for j, p in enumerate(self.points)}
        self.riders = {link: index[p] for link, p in topo.riders.items()}
        self.drop_bearing = drop_bearing
        self.facts = facts
        self.pieces = (Disc("O", dims.stub), Disc("O", dims.journal),
                       *[Disc(p, dims.post) for p in self.points],
                       *[Pill("O", p, dims.web) for p in self.points])
        # the joint rules: which points' chains may follow each other, and end at the hub
        self.rules = rules
        self._why: dict[tuple, str] = {}    # why the joint rules close a layering (memo)
        self._relax = 0                     # which rules _unbuildable has relaxed (memo key)
        self._routes: dict[tuple, tuple | None] = {}    # _solve per what it reads
        at = [topo.geometry.points[p][0] for p in self.points]
        n = self.n
        self.bottom_layers = rules.bottom_layers
        # pieces in the clearance gaps (the planner blocks them with other groups' heads and
        # washers there, keyed by the gap's slot, layer + 0.5)
        self.gap_head = rules.gap_head
        self.gap_pieces = ()
        if self.gap_head > 0:
            self.gap_pieces = (*[Disc(p, self.gap_head) for p in self.points],
                               *[Disc(h, r) for h, r in rules.horn_heads],
                               Disc("O", self.gap_head))
        # (gap_washer) bit washer_bit + j of a gap slot's blocked bits: point j's run washers
        # can't cross that gap (set by the planner's leaf only; -1: no such bits)
        self.washer_bit = (len(self.gap_pieces)
                           if self.gap_head > 0 and rules.gap_washer > 0 else -1)
        # how many layers a journal standoff between two chains may span
        self.j_ok = dict(rules.j_spans)
        self.horn_mask = ((1 << len(rules.horn_heads)) - 1) << self.n if self.gap_head else 0
        self.rider_mask = 0
        for j in set(self.riders.values()):
            self.rider_mask |= 1 << j
        self.spans: dict[int, int] | None = rules.spans     # (None: _unbuildable's relaxing)
        gap = rules.nut + max(rules.head, rules.post)
        self.after = [[i == j or float(np.linalg.norm(at[i] - at[j])) >= gap
                       for j in range(n)] for i in range(n)]
        self.last = [all(float(np.linalg.norm(at[j] - np.asarray(xy))) >= rules.nut + r
                         for xy, r in rules.horn) for j in range(n)]
        # one chain per point, its runs strictly between its outer webs, which a stock
        # standoff must span: two run layers of one point are at most this far apart
        longest = max((k for k, m in rules.spans.items() if m), default=None)
        self.window: int | None = None if longest is None else longest - 3
        self._h0: dict[int, int] = {}       # stack size -> the hub's bottom layer (memo)

    def hub_bottom(self, top: int) -> int:
        h0 = self._h0.get(top)
        if h0 is None:
            _, hub = hub_layers(Layout({}, top, self.pitch), self.drive, self.dims.hub_thickness)
            h0 = self._h0[top] = hub.start if len(hub) else top
        return h0

    # -- per layer ------------------------------------------------------------------

    def _prepare(self, view: RouteView
                 ) -> tuple[int, dict[int, int], dict[int, tuple[int, int]]] | RouteConflict:
        """(the hub's bottom layer, layer -> the point its riders ride, point -> the lowest
        and highest layer of its placed riders), or why not."""
        h0 = self.hub_bottom(view.layout.top)
        if h0 < 2:
            return RouteConflict(1, max(h0, 1), "the hub leaves no layer for the crank below it")
        riding: dict[int, int] = {}
        ends: dict[int, tuple[tuple[int, str], tuple[int, str]]] = {}
        for link, j in self.riders.items():
            k = view.layout.layers.get(link)
            if k is None:
                continue
            if not 2 <= k < h0:
                return RouteConflict(k, k, "a link riding a crankpin sits in the hub's layers"
                                     if k >= h0 else "a link riding a crankpin sits in layer "
                                     "1, with no room for the web below")
            if riding.setdefault(k, j) != j:
                return RouteConflict(k, k, "links riding two crankpins share a layer")
            lo, hi = ends.get(j, ((k, link), (k, link)))
            ends[j] = (min(lo, (k, link)), max(hi, (k, link)))
        if self.window is not None:
            for j, ((lo, a), (hi, b)) in ends.items():
                if hi - lo > self.window:
                    return RouteConflict(
                        lo, hi, f"links riding {self.points[j]} sit {hi - lo} layers apart, "
                        f"more than the {self.window} one chain along it spans (a stock standoff "
                        f"between its webs at most {self.window + 3} layers apart)",
                        rules=True, links=frozenset({a, b}))
        return h0, riding, {j: (lo[0], hi[0]) for j, (lo, hi) in ends.items()}

    def _valid(self, blocked: int, rider: int | None) -> int:
        """The states a layer allows, from what blocks its pieces and who rides in it."""
        if rider is not None:
            return 16 << rider
        v = 2 if self.drop_bearing else 0
        if not blocked & 1:
            v |= 1
        if not blocked & 2:
            v |= 12
        return v | ((~blocked >> 2) & self.full) << 4

    # -- the route --------------------------------------------------------------------

    def states(self, link: str, blocked: int) -> int:
        """The states a layer holding ``link`` allows, given the pieces its shapes block."""
        return self._valid(blocked, self.riders.get(link))

    def check(self, view: RouteView) -> RouteConflict | dict[int, int]:
        """A partial layering: can a buildable route pass everything placed so far (with a
        bound: is the cheapest one cheaper)? If so, the states each layer ``1..`` below the
        hub has on some route (ignoring the joint rules). A dead end is explained by the
        layers reachability dies in, or, when only the joint rules close it, by them
        (``RouteConflict.rules``)."""
        # the cheap relaxation first: a layering no state can get through needs no exact
        # route (the exact DP, below, is the cost of a node); a route found always passes it
        pre = self._prepare(view)
        if isinstance(pre, RouteConflict):
            return pre
        h0, riding, ridden = pre
        fwd = self._forward(view.blocked, riding, h0)
        if isinstance(fwd, int):
            return self._conflict(view, fwd, riding, h0)
        res = self._cheapest(view, pre)
        if isinstance(res, RouteConflict) and (res.bound or res.why):
            return res
        if isinstance(res, RouteConflict):
            # why, worded once per what the rules see (the riders' layers, what blocks
            # each piece where): a partial layering doesn't pay for it again
            key = (h0, tuple(sorted(riding.items())), tuple(sorted(view.blocked.items())))
            why = self._why.get(key)
            if why is None:
                why = self._why[key] = self._unbuildable(view)
            return RouteConflict(1, h0, why, rules=True)
        bwd = self._backward(view.blocked, riding, h0)
        out = {k: fwd[k] & bwd[k] for k in range(1, h0)}
        if self.window is not None:
            # every run along a point lies in its one chain: within the window of its riders
            for j, (lo, hi) in ridden.items():
                for k in out:
                    if k < hi - self.window or k > lo + self.window:
                        out[k] &= ~(16 << j)
        return out

    def _forward(self, blocked, riding, h0) -> list[int] | int:
        """States reachable from the bottom in each layer, or the first layer none reach
        (``h0``: none enters the hub). (:meth:`_valid` inlined.)"""
        out = [0] * h0
        full, n2, base = self.full, 2 + self.n, 2 if self.drop_bearing else 0
        get, rget = blocked.get, riding.get
        b = get(1, 0)
        r = rget(1)
        reach = out[1] = (16 << r if r is not None else
                          base | (0 if b & 1 else 1) | (0 if b & 2 else 12)
                          | ((~b >> 2) & full) << 4) & 7
        if not reach:
            return 1
        web_prev = (~b >> n2) & full
        for k in range(2, h0):
            b = get(k, 0)
            wk = (~b >> n2) & full
            nxt = (reach & 1) * 5 | (reach & 2) * 3            # S -> S, L; E -> E, L
            if reach & 12:
                nxt |= web_prev << 4                            # L, J -> R(j) over a web
            nxt |= reach & 8                                    # J -> J
            rr = reach >> 4
            nxt |= rr << 4                                      # R(j) -> R(j)
            if rr & wk:
                nxt |= 8                                        # R(j) -> J over a web
            r = rget(k)
            reach = out[k] = nxt & (16 << r if r is not None else
                                    base | (0 if b & 1 else 1) | (0 if b & 2 else 12)
                                    | ((~b >> 2) & full) << 4)
            if not reach:
                return k
            web_prev = wk
        wh = (~get(h0, 0) >> n2) & full
        if reach & 8 or (reach >> 4) & wh or (reach & 3 and not riding):
            return out
        return h0

    def _backward(self, blocked, riding, h0) -> list[int]:
        """States in each layer from which the hub can be reached (0 from a dead end down).
        (:meth:`_valid` inlined.)"""
        out = [0] * h0
        full, n2, base = self.full, 2 + self.n, 2 if self.drop_bearing else 0
        get, rget = blocked.get, riding.get
        wn = (~get(h0, 0) >> n2) & full
        alive = 8 | wn << 4 | (0 if riding else 3)
        for k in range(h0 - 1, 0, -1):
            b = get(k, 0)
            wk = (~b >> n2) & full
            if k < h0 - 1:
                here = (1 if alive & 5 else 0) | (2 if alive & 6 else 0)
                if (alive >> 4) & wk:
                    here |= 12                                  # L, J -> R(j) over a web
                here |= alive & 8
                here |= (alive >> 4) << 4
                if alive & 8:
                    here |= wn << 4                             # R(j) -> J over a web
                alive = here
            r = rget(k)
            alive = out[k] = alive & (16 << r if r is not None else
                                      base | (0 if b & 1 else 1) | (0 if b & 2 else 12)
                                      | ((~b >> 2) & full) << 4) & (7 if k == 1 else -1)
            if not alive:
                break
            wn = wk
        return out

    def route(self, view: RouteView, pre=None) -> Route | RouteConflict:
        """The cheapest buildable route for a layering. For a partial one its cost is a lower
        bound: a run layer an unplaced rider may still take costs nothing, and more riders
        only add faces set back for their end play, which only take screw fits away.
        ``pre``: :meth:`_prepare`'s answer for ``view``, when the caller has it.

        The search asks this at every node, and a backtracking search asks for the same
        layering again and again (what it reads: the hub's bottom, the riders' layers, the
        pieces blocked per layer and the run layers unplaced riders may still take), so the
        answer is remembered per that (:meth:`_solve`) and only the bound is applied here."""
        res = self._cheapest(view, pre)
        if not isinstance(res, tuple):
            return res
        cost, runs, bearing = res
        return Route(CrankRoute(tuple(Run(at, lo, hi) for at, lo, hi in runs), bearing),
                     int(cost) // BEARING)

    def _cheapest(self, view: RouteView, pre=None
                  ) -> tuple[int, tuple, bool] | RouteConflict:
        """:meth:`route`'s answer as the DP gives it (cost, runs as triples, bearing), so the
        search's sub-check builds no route it doesn't keep."""
        if pre is None:
            pre = self._prepare(view)
        if isinstance(pre, RouteConflict):
            return pre
        h0, riding, _ = pre
        n = self.n
        may = [0] * n       # per point: the run layers an unplaced rider of it may still take
        if view.open:
            bits = view.open_bits
            for link, j in self.riders.items():
                if bits is not None:
                    may[j] |= bits.get(link, 0)
                    continue
                for k in view.open.get(link, ()):
                    may[j] |= 1 << k
        b = tuple(map(view.blocked.get, range(h0 + 1), repeat(0)))
        gb = (tuple(view.blocked.get(k + 0.5, 0) for k in range(h0 + 1)) if self.gap_pieces
              else ())
        key = (h0, tuple(sorted(riding.items())), b, gb, tuple(may), self._relax)
        found = self._routes.get(key, _MISS)
        if found is _MISS:
            if len(self._routes) >= self.MEMO_ROUTES:
                self._routes.clear()
            found = self._routes[key] = self._solve(h0, riding, b, may, gb)
        if found is None:
            if view.open is not None:
                return RouteConflict(1, h0)
            return RouteConflict(1, h0, self._unbuildable(view), rules=True)
        if view.bound is not None and found[0] // BEARING >= view.bound:
            return RouteConflict(1, h0, bound=True)
        return found

    def _solve(self, h0: int, riding: dict[int, int], b: tuple[int, ...], may: list[int],
               gb: tuple[int, ...] = ()
               ) -> tuple[int, tuple[tuple[str, int, int], ...], bool] | None:
        """The cheapest buildable route below the hub's bottom layer ``h0``, as (cost, runs
        as ``(point, lo, hi)`` triples, bearing), or ``None``: a shortest path over layers
        and chains (module docstring). ``riding``: layer -> the point its rider rides;
        ``b[k]``: the pieces blocked in layer ``k``; ``may[j]``: the layers (bits) an
        unplaced rider of point ``j`` may take. Plain tuples throughout, so what the memo
        keeps costs the garbage collector nothing to walk.

        The search asks this at nearly every node, so it is written for speed: a chain's runs
        are a linked list (``(run, rest)`` pairs, the latest first) until the route is read
        back, and the steps are inlined; the transitions, and the order ties are broken in
        (the first reached among equals), are the same as a plain shortest path's."""
        n, pins = self.n, len(self.pins)
        if gb and gb[h0 - 1] & self.horn_mask:
            return None             # the horn screws' heads under the hub meet something
        on_o = [k not in riding and not b[k] & 2 for k in range(h0 + 1)]    # the journal fits
        on_o[h0] = True                                                     # the hub
        spans = self.spans
        # per point, as bits over the layers: where a post may run (free, or ridden by it),
        # where a web to it may sit (on O, the piece free), and where its riders are
        post_m, web_m, rid_m = [0] * n, [0] * n, [0] * n
        for k in range(h0 + 1):
            r, bk = riding.get(k), b[k]
            for j in range(n):
                if (r == j) if r is not None else not bk >> (2 + j) & 1:
                    post_m[j] |= 1 << k
                if on_o[k] and not bk >> (2 + n + j) & 1:
                    web_m[j] |= 1 << k
                if r == j:
                    rid_m[j] |= 1 << k
        memos: list[dict[int, list]] = [{} for _ in range(n)]      # chains(j, a) per call
        points, sweep = self.points, self.sweep
        washer_bit = self.washer_bit

        def chains(j: int, a: int) -> list[tuple[int, int, int, tuple]]:
            """Every chain along point j from its lowest web in layer a, the cheapest for each
            (highest web's layer, whether a rider on its last run layer sets that web back):
            ``(end, last, cost, runs)``, runs a linked list."""
            out: dict[tuple[int, bool], tuple[int, tuple]] = {}
            enter = RUN + (FEATURE + sweep[j] * SWEEP if j >= pins else 0)
            wm, pm, rm, mm = web_m[j], post_m[j], rid_m[j], may[j]
            point = points[j]
            wb = washer_bit + j if washer_bit >= 0 and gb else -1
            if wm >> a & 1 and a + 1 < h0:
                # the run open in k - 1 ({all ridden so far: (cost, runs before it, its first
                # layer)}); a chain is one run (no plate between two runs of a point)
                prev: dict[bool, tuple[int, tuple, int]] | None = None
                for k in range(a + 1, h0 + 1):
                    if wb >= 0 and gb[k - 1] >> wb & 1 and k > a + 1:
                        # the run's washers can't cross the gap over layer k - 1 (another
                        # group's head or washers there): no run goes on past it
                        prev = None
                    ridden = bool(rm >> k & 1)
                    if prev and wm >> k & 1 and not (gb and gb[k] >> j & 1):
                        # the run open in k - 1 ends: a web in k, the chain's end
                        was = bool(rm >> (k - 1) & 1)
                        for full, (c, rs, lo) in prev.items():
                            key = (k, was and not full)
                            got = out.get(key)
                            if got is None or c < got[0]:
                                out[key] = (c, ((point, lo, k - 1), rs))
                    opts: dict[bool, tuple[int, tuple, int]] | None = None
                    if k < h0 and pm >> k & 1:
                        here = 0 if ridden or mm >> k & 1 else FEATURE
                        opts = {ridden: (enter + here, (), k)} if k == a + 1 else {}
                        if prev:
                            for full, (c, rs, lo) in prev.items():
                                f = full and ridden
                                c += here
                                got = opts.get(f)
                                if got is None or c < got[0]:
                                    opts[f] = (c, rs, lo)
                        if not opts:
                            opts = None
                    if opts is None:
                        break
                    prev = opts
            got = memos[j][a] = [(end, int(last), c, rs) for (end, last), (c, rs) in out.items()]
            return got

        # best[(k, used, last point, pending)]: k is on O and no chain spans it; a chain that
        # ends in k waits in ``pending`` (size, flags) until the next step says if a rider sits
        # right over its highest web. States are kept per layer, in the order first reached.
        best: dict[tuple, tuple[int, tuple, tuple]] = {}
        at: dict[int, dict[tuple, None]] = {}

        def start(used: int, cost: int, prev: tuple, j: int, a: int, under: int) -> None:
            if gb and (gb[a - 1] >> j & 1 or gb[a] >> j & 1):
                return            # its screw head under the lowest web, its spacer over it
            first = rid_m[j] >> (a + 1) & 1
            used |= 1 << j
            ch = memos[j].get(a)
            for end, last, c, runs in (ch if ch is not None else chains(j, a)):
                state = (end, used, j, (end - a + 1, under, first, last))
                c += cost
                got = best.get(state)
                if got is None:
                    at.setdefault(end, {})[state] = None
                    best[state] = (c, prev, runs)
                elif c < got[0]:
                    best[state] = (c, prev, runs)

        stub_ok, empty_ok = True, self.drop_bearing
        allowed = self.bottom_layers
        o_gap = len(self.gap_pieces) - 1
        for a in range(1, h0 - 1):
            # (gap pieces) the stub's screw head over the lowest web, at O, in the gap there
            head_ok = not (gb and gb[a] >> o_gap & 1)
            for bearing, ok in ((True, stub_ok and (not allowed or a in allowed) and head_ok),
                                (False, empty_ok)):
                if ok:
                    for j in range(n):
                        start(0, 0 if bearing else BEARING, ("start", bearing), j, a, 0)
            stub_ok = stub_ok and a not in riding and not b[a] & 1
            empty_ok = empty_ok and a not in riding
            if not (stub_ok or empty_ok):
                break
        after = self.after
        cost_of = _second
        rider_mask, j_ok = self.rider_mask, self.j_ok
        o_bit = len(self.gap_pieces) - 1
        for k in range(1, h0):
            states = at.get(k)
            if not states:
                continue
            here = sorted([(s, best[s][0]) for s in states], key=cost_of)
            through = on_o[k + 1] and k + 1 not in riding
            for state, cost in here:
                _, used, last, pend = state
                jn = -1                     # layers on O since the last chain's web
                if pend[0] < 0:
                    # no pending chain: ``fit`` (read only while one is pending) is unused
                    ok0, under, jn, fit = True, 0, pend[1], 0
                    pend = None
                else:
                    size, a, first, under = pend
                    fit = (spans.get(size, 0) >> (16 * a + 4 * first + 2 * under)
                           if spans is not None else -1)
                    ok0 = bool(fit & 1)
                if (through and ok0 and used & rider_mask != rider_mask
                      and (jn >= 0 or not (gb and (gb[k - 1] | gb[k]) >> o_bit & 1))):
                    # a journal standoff on O to the next chain: screwed to this chain's top
                    # web from under it (its head in the gap there) and to the next one's
                    # lowest web; never past the last chain (the hub's horn screws)
                    nxt = (k + 1, used, last, (-1, jn + 1 if jn >= 0 else 1, 0, 0))
                    got = best.get(nxt)
                    if got is None:
                        at.setdefault(k + 1, {})[nxt] = None
                        best[nxt] = (cost, state, ())
                    elif cost < got[0]:
                        best[nxt] = (cost, state, ())
                if pend is None and not (
                        jn >= 1 and j_ok.get(jn - 1, False)
                        and not (gb and gb[k] >> o_bit & 1)):
                    continue          # no chain starts there
                ok_after = after[last]
                for j in range(n):
                    if used >> j & 1 or not ok_after[j]:
                        continue
                    if pend is not None and not fit >> (rid_m[j] >> (k + 1) & 1) & 1:
                        continue
                    start(used, cost, state, j, k, under)
        last_ok = self.last
        ends = []
        for s in at.get(h0, ()):
            if not last_ok[s[2]]:
                continue
            pend = s[3]
            if pend is not None and pend[0] < 0:
                continue                    # a journal standoff can't end at the hub
            if pend is not None:
                size, a, first, lastf = pend
                if spans is not None and not spans.get(size, 0) >> (16 * a + 4 * first
                                                                      + 2 * lastf) & 1:
                    continue
            ends.append((best[s][0], s))
        if not ends:
            return None
        cost, state = min(ends, key=_first)
        runs: list[tuple[str, int, int]] = []
        while True:
            _, prev, rs = best[state]
            chain = []
            while rs:
                run, rs = rs
                chain.append(run)
            runs[:0] = chain[::-1]
            if prev[0] == "start":
                return int(cost), tuple(runs), prev[1]
            state = prev

    def _unbuildable(self, view: RouteView) -> str:
        """Why no route is buildable although the relaxation lets one through."""
        if self.spans is None:
            return "no crank route passes"
        saved = self.spans, self.after, self.last
        try:
            self.spans, self._relax = None, 1
            if isinstance(self._cheapest(view), tuple):
                sizes = sorted(k for k, m in saved[0].items() if m)
                return ("its crank routes need a joint no stock screw fits (a chain's webs "
                        f"{_ranges(sizes)} layers apart, both counted, take one)")
            self.after = [[True] * self.n for _ in range(self.n)]
            self.last = [True] * self.n
            self._relax = 2
            if isinstance(self._cheapest(view), tuple):
                return "its crank routes put two joints' pockets together"
        finally:
            self.spans, self.after, self.last = saved
            self._relax = 0
        return "no crank route passes"

    def _conflict(self, view: RouteView, dead: int, riding: dict[int, int],
                  h0: int) -> RouteConflict:
        """The layers that explain a dead end: ``1..dead`` seen from below, or from the
        hub down to where no state reaches it, whichever is shorter."""
        bwd = self._backward(view.blocked, riding, h0)
        top = next((k for k in range(h0 - 1, 0, -1) if not bwd[k]), None)
        if top is not None and h0 - top < dead:
            return RouteConflict(top, h0)
        return RouteConflict(1, dead)
