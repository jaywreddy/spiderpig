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

The route must also be buildable (:class:`JointRules`, from the printed
crank's joints): the runs along one point form one **chain**, runs whose webs
meet (in one layer or two adjacent ones), screwed together by one screw from
the chain's lowest web to its highest, so the webs' span must take a stock
screw; and the pockets of two joints in one segment (consecutive chains, or
the last chain and the horn screws) must not meet.

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

import math
from dataclasses import dataclass, field

import numpy as np

from construction.base import Context
from construction.crank import (
    NUT_AF,
    POST_SCREWS,
    CrankDims,
    CrankRoute,
    Run,
    hub_layers,
)
from construction.underside import Underside
from stack import Disc, Layout, Pill, Route, RouteConflict, RouteView

# States are bits: S 1, E 2, L 4, J 8, R(point j) 16 << j. Pieces: the stub (bit 0) and the
# journal (1) on O, a post on each run point (2 + j), a web to each (2 + n + j).
FEATURE = 10**12                 # cost weights, lexicographic
SWEEP = 10**4                    # per 0.01 mm a detour reaches below O
BEARING = 10**3
RUN = 1                          # a tie-break within one layering: not part of Route.cost


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

    ``spans[n]``: whether a stock screw fits a chain whose webs span ``n``
    layers (the lowest and highest included), for each set of faces set back
    by the riders' end play, bit ``8a + 4b + 2c + d``: the lowest web's bottom
    face (a rider under it) and top face (a rider on the chain's first run
    layer), the highest web's bottom face (a rider on the last run layer, the
    run not all riders) and top face (a rider over it). Pocket radii (mm): a
    screw head's counterbore, a nut's trap, a post. Horn pockets, fixed on the
    crank: ``(point on the crank at sample 0, radius)``. ``hub_play``: whether
    the horn screws still fit the hub with its bottom face set back for a
    rider's end play; else no chain may end in the hub's lowest layer with
    that web set back.
    """

    spans: dict[int, int]
    head: float
    nut: float
    post: float
    horn: tuple[tuple[tuple[float, float], float], ...] = ()
    hub_play: bool = True


def joint_rules(construction, ctx: Context, dims: CrankDims) -> JointRules | None:
    """The joint rules of a printed crank (``None`` for a construction without them)."""
    if not hasattr(construction, "post_joint"):
        return None
    p, play = ctx.pitch, construction.axial_play
    spans = {n: sum(1 << (8 * a + 4 * b + 2 * c + d)
                    for a in (0, 1) for b in (0, 1) for c in (0, 1) for d in (0, 1)
                    if construction.post_joint(a * play, p - b * play, (n - 1) * p + c * play,
                                               n * p - d * play) is not None)
             for n in range(3, 64)}
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
    return JointRules(spans,
                      head=max(sk.head_d for sk in POST_SCREWS) / 2 + construction.screw_fit / 2,
                      nut=(NUT_AF + construction.nut_fit) / math.sqrt(3), post=dims.post,
                      horn=tuple(horn),
                      hub_play=construction.hub_joint(ctx, dims.hub_thickness, play) is not None)


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

    def __init__(self, ctx: Context, dims: CrankDims, facts: CrankFacts, drop_bearing: bool,
                 rules: JointRules | None = None):
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
        at = [topo.geometry.points[p][0] for p in self.points]
        n = self.n
        if rules is None:
            self.spans: dict[int, int] | None = None
            self.after = [[True] * n for _ in range(n)]
            self.last = [True] * n
            self.window: int | None = None
        else:
            self.spans = rules.spans
            gap = rules.nut + max(rules.head, rules.post)
            self.after = [[i == j or float(np.linalg.norm(at[i] - at[j])) >= gap
                           for j in range(n)] for i in range(n)]
            self.last = [all(float(np.linalg.norm(at[j] - np.asarray(xy))) >= rules.nut + r
                             for xy, r in rules.horn) for j in range(n)]
            # one chain per point, its runs strictly between its outer webs, which a stock
            # screw must span: two run layers of one point are at most this far apart
            longest = max((k for k, m in rules.spans.items() if m), default=None)
            self.window = None if longest is None else longest - 3
        self.hub_play = rules is None or rules.hub_play

    def hub_bottom(self, top: int) -> int:
        _, hub = hub_layers(Layout({}, top, self.pitch), self.drive, self.dims.hub_thickness)
        return hub.start if len(hub) else top

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
                        f"more than the {self.window} one chain along it spans (a stock screw "
                        f"through its webs at most {self.window + 3} layers apart)",
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

    def _webs(self, blocked: int) -> int:
        return (~blocked >> (2 + self.n)) & self.full

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
        res = self.route(view)
        if isinstance(res, RouteConflict) and (res.bound or res.why):
            return res
        pre = self._prepare(view)
        if isinstance(pre, RouteConflict):
            return pre
        h0, riding, ridden = pre
        fwd = self._forward(view.blocked, riding, h0)
        if isinstance(fwd, int):
            return self._conflict(view, fwd, riding, h0)
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
        (``h0``: none enters the hub)."""
        out = [0] * h0
        reach = out[1] = self._valid(blocked.get(1, 0), riding.get(1)) & 7
        if not reach:
            return 1
        web_prev = self._webs(blocked.get(1, 0))
        for k in range(2, h0):
            b = blocked.get(k, 0)
            wk = self._webs(b)
            nxt = (reach & 1) * 5 | (reach & 2) * 3            # S -> S, L; E -> E, L
            if reach & 12:
                nxt |= web_prev << 4                            # L, J -> R(j) over a web
            nxt |= reach & 8                                    # J -> J
            r = reach >> 4
            nxt |= r << 4                                       # R(j) -> R(j)
            if r & wk:
                nxt |= 8                                        # R(j) -> J over a web
            reach = out[k] = nxt & self._valid(b, riding.get(k))
            if not reach:
                return k
            web_prev = wk
        wh = self._webs(blocked.get(h0, 0))
        if reach & 8 or (reach >> 4) & wh or (reach & 3 and not riding):
            return out
        return h0

    def _backward(self, blocked, riding, h0) -> list[int]:
        """States in each layer from which the hub can be reached (0 from a dead end down)."""
        out = [0] * h0
        alive = 8 | self._webs(blocked.get(h0, 0)) << 4 | (0 if riding else 3)
        for k in range(h0 - 1, 0, -1):
            b = blocked.get(k, 0)
            if k < h0 - 1:
                wk, wn = self._webs(b), self._webs(blocked.get(k + 1, 0))
                here = (1 if alive & 5 else 0) | (2 if alive & 6 else 0)
                if (alive >> 4) & wk:
                    here |= 12                                  # L, J -> R(j) over a web
                here |= alive & 8
                here |= (alive >> 4) << 4
                if alive & 8:
                    here |= wn << 4                             # R(j) -> J over a web
                alive = here
            alive = out[k] = alive & self._valid(b, riding.get(k)) & (7 if k == 1 else -1)
            if not alive:
                break
        return out

    def route(self, view: RouteView) -> Route | RouteConflict:
        """The cheapest buildable route for a layering. For a partial one its cost is a lower
        bound: a run layer an unplaced rider may still take costs nothing, and more riders
        only add faces set back for their end play, which only take screw fits away."""
        pre = self._prepare(view)
        if isinstance(pre, RouteConflict):
            return pre
        h0, riding, _ = pre
        blocked = view.blocked
        maybe: dict[int, set[int]] = {}
        for link, j in self.riders.items():
            for k in (view.open or {}).get(link, ()):
                maybe.setdefault(k, set()).add(j)
        n, pins = self.n, len(self.pins)
        b = [blocked.get(k, 0) for k in range(h0 + 1)]
        on_o = [k not in riding and not b[k] & 2 for k in range(h0 + 1)]    # the journal fits
        on_o[h0] = True                                                     # the hub
        spans = self.spans

        def post(k: int, j: int) -> bool:
            r = riding.get(k)
            return r == j if r is not None else not b[k] >> (2 + j) & 1

        def web(k: int, j: int) -> bool:
            return on_o[k] and not b[k] >> (2 + n + j) & 1

        def fits(chain: tuple[int, int, int, int], d: int) -> bool:
            size, a, first, last = chain
            return spans is None or bool(spans.get(size, 0) >> (8 * a + 4 * first + 2 * last + d)
                                         & 1)

        memo: dict[tuple[int, int], list] = {}

        def chains(j: int, a: int) -> list[tuple[int, int, float, tuple[Run, ...]]]:
            """Every chain along point j from its lowest web in layer a, the cheapest for each
            (highest web's layer, whether a rider on its last run layer sets that web back)."""
            if (j, a) in memo:
                return memo[(j, a)]
            out: dict[tuple[int, int], tuple[float, tuple[Run, ...]]] = {}
            enter = RUN + (FEATURE + self.sweep[j] * SWEEP if j >= pins else 0)
            # open run in layer k: (cost, runs before it, its first layer, all ridden so far)
            run: dict[int, dict[bool, tuple[float, tuple[Run, ...], int]]] = {}
            inner: dict[int, dict[int, tuple[float, tuple[Run, ...]]]] = {}  # after 1, 2 webs
            if web(a, j) and a + 1 < h0:
                for k in range(a + 1, h0 + 1):
                    ridden = riding.get(k) == j
                    # the run open in k - 1 ends: a web in k (the chain's end, or an inner one)
                    ends = [(c, rs + (Run(self.points[j], lo, k - 1),), riding.get(k - 1) == j
                             and not full) for full, (c, rs, lo) in run.get(k - 1, {}).items()]
                    if ends and web(k, j):
                        for c, rs, last in ends:
                            if c < out.get((k, last), (math.inf,))[0]:
                                out[(k, last)] = (c, rs)
                        c, rs, _ = min(ends, key=lambda e: e[0])
                        inner.setdefault(k, {})[1] = (c, rs)
                    if 1 in inner.get(k - 1, {}) and web(k, j):
                        inner.setdefault(k, {})[2] = inner[k - 1][1]
                    if k < h0 and post(k, j):
                        here = 0 if ridden or j in maybe.get(k, ()) else FEATURE
                        opts: dict[bool, tuple[float, tuple[Run, ...], int]] = {}
                        starts = [(enter, (), k)] if k == a + 1 else []
                        starts += [(c + enter, rs, k) for c, rs in inner.get(k - 1, {}).values()]
                        for c, rs, lo in starts:
                            if c + here < opts.get(ridden, (math.inf,))[0]:
                                opts[ridden] = (c + here, rs, lo)
                        for full, (c, rs, lo) in run.get(k - 1, {}).items():
                            f = full and ridden
                            if c + here < opts.get(f, (math.inf,))[0]:
                                opts[f] = (c + here, rs, lo)
                        if opts:
                            run[k] = opts
                    if k not in run and k not in inner:
                        break
            memo[(j, a)] = [(end, last, c, rs) for (end, last), (c, rs) in out.items()]
            return memo[(j, a)]

        INF = math.inf
        # best[(k, used, last point, pending)]: k is on O and no chain spans it; a chain that
        # ends in k waits in ``pending`` (size, flags) until the next step says if a rider sits
        # right over its highest web
        best: dict[tuple, tuple[float, tuple | None, tuple[Run, ...]]] = {}

        def push(state, cost, prev, runs):
            if cost < best.get(state, (INF,))[0]:
                best[state] = (cost, prev, runs)

        def start(state, cost, prev, j: int, a: int, under: int) -> None:
            first = int(riding.get(a + 1) == j)
            for end, last, c, runs in chains(j, a):
                push((end, state[1] | 1 << j, j, (end - a + 1, under, first, int(last))),
                     cost + c, prev, runs)

        stub_ok, empty_ok = True, self.drop_bearing
        for a in range(1, h0 - 1):
            for bearing, ok in ((True, stub_ok), (False, empty_ok)):
                if ok:
                    for j in range(n):
                        start((0, 0), 0 if bearing else BEARING, ("start", bearing), j, a, 0)
            stub_ok = stub_ok and a not in riding and not b[a] & 1
            empty_ok = empty_ok and a not in riding
            if not (stub_ok or empty_ok):
                break
        for k in range(1, h0):
            here = sorted(((s, v[0]) for s, v in best.items() if s[0] == k), key=lambda x: x[1])
            for state, cost in here:
                _, used, last, pend = state
                if on_o[k + 1] and k + 1 not in riding and (pend is None or fits(pend, 0)):
                    push((k + 1, used, last, None), cost, state, ())
                for j in range(n):
                    if used >> j & 1 or not self.after[last][j]:
                        continue
                    if pend is not None and not fits(pend, int(riding.get(k + 1) == j)):
                        continue
                    start(state, cost, state, j, k, pend[3] if pend is not None else 0)
        ends = [(v[0], s) for s, v in best.items()
                if s[0] == h0 and self.last[s[2]]
                and (s[3] is None or fits(s[3], 0) and (self.hub_play or not s[3][3]))]
        if not ends:
            if view.open is not None:
                return RouteConflict(1, h0)
            return RouteConflict(1, h0, self._unbuildable(view), rules=True)
        cost, state = min(ends, key=lambda e: e[0])
        if view.bound is not None and cost // BEARING >= view.bound:
            return RouteConflict(1, h0, bound=True)
        runs: list[Run] = []
        while True:
            _, prev, rs = best[state]
            runs[:0] = rs
            if prev[0] == "start":
                return Route(CrankRoute(tuple(runs), prev[1]), int(cost) // BEARING)
            state = prev

    def _unbuildable(self, view: RouteView) -> str:
        """Why no route is buildable although the relaxation lets one through."""
        if self.spans is None:
            return "no crank route passes"
        saved = self.spans, self.after, self.last, self.hub_play
        try:
            self.spans = None
            if isinstance(self.route(view), Route):
                sizes = sorted(k for k, m in saved[0].items() if m)
                return ("its crank routes need a joint no stock screw fits (a chain's webs "
                        f"{_ranges(sizes)} layers apart, both counted, take one)")
            self.hub_play = True
            if isinstance(self.route(view), Route):
                return ("its crank routes end with a web set back for its rider's end play in "
                        "the hub's lowest layer, which leaves the hub too short for the horn "
                        "screws")
            self.after = [[True] * self.n for _ in range(self.n)]
            self.last = [True] * self.n
            if isinstance(self.route(view), Route):
                return "its crank routes put two joints' pockets together"
        finally:
            self.spans, self.after, self.last, self.hub_play = saved
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
