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
can only sit in a run layer whose post it clears. :meth:`CrankRouter.route`
finds, for one layering, the cheapest route whose pieces clear everything
else in their layers: a shortest path over layers x states. Cost, in order:
added features (a run layer no rider of its point needs, a detour run), how
far detours reach below O (their sweep), a dropped bearing, then fewer runs.

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
from construction.crank import CrankDims, CrankRoute, Run, add_crank_point, hub_layers
from construction.underside import Underside
from stack import Disc, Layout, Pill, Route, RouteConflict, RouteView

S, E, L, J = 0, 1, 2, 3          # states; R(point j) is 4 + j
STUB, JOURNAL = 0, 1             # pieces; post(j) = 2 + 2j, web(j) = 3 + 2j
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


def point_clearance(topo, link: str, radii: np.ndarray, angles_deg: np.ndarray) -> np.ndarray:
    """Lower bound on how close ``link`` comes, over the cycle, to points fixed to the crank
    (``radii`` x ``angles_deg`` from the first crankpin): shape ``(len(radii), len(angles))``."""
    g = topo.geometry.points
    o = g["O"]
    n = topo.geometry.samples
    phi = _crank_frame(topo, np.arange(n))
    c, s = np.cos(-phi)[:, None], np.sin(-phi)[:, None]

    def local(name):                      # the link's points in the crank's frame
        d = g[name] - o
        return np.concatenate([c * d[:, :1] - s * d[:, 1:], s * d[:, :1] + c * d[:, 1:]], axis=1)

    th = np.radians(angles_deg)
    pts = (radii[:, None, None] * np.stack([np.cos(th), np.sin(th)], -1)[None]).reshape(-1, 2)
    best = np.full(len(pts), np.inf)
    for a, b in topo.links[link]:
        A, B = local(a), local(b)
        step = max(np.linalg.norm(np.roll(A, -1, 0) - A, axis=1).max(),
                   np.linalg.norm(np.roll(B, -1, 0) - B, axis=1).max())
        ab = B - A
        den = np.maximum((ab * ab).sum(-1), 1e-18)
        for i in range(0, len(pts), 512):
            p = pts[i:i + 512, None, :]
            u = np.clip(((p - A) * ab).sum(-1) / den, 0.0, 1.0)
            d = np.linalg.norm(p - (A + ab * u[..., None]), axis=-1).min(axis=1) - step / 2
            best[i:i + 512] = np.minimum(best[i:i + 512], d)
    return best.reshape(len(radii), len(angles_deg))


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
    if not lonely:
        return facts
    radii = np.arange(1.0, 151.0, 1.0)
    angles = np.arange(0.0, 360.0, 2.5)
    need = dims.post + r + margin
    clear = {n: point_clearance(topo, n, radii, angles) for n in facts.o_free}
    for n in lonely:
        ok = clear[n] >= need
        inside = ok & (radii[:, None] + reach <= facts.allow)
        if inside.any():
            # the lowest sweep first, then the point that also clears most other O-free links
            i = int(np.argmax(inside.any(axis=1)))
            js = np.flatnonzero(inside[i])
            j = max(js, key=lambda j: sum(clear[m][i, j] >= need for m in facts.o_free))
            name = f"crank.detour{len(facts.detours)}"
            if not any(d.r == radii[i] and d.angle == angles[j] for d in facts.detours):
                add_crank_point(topo, name, float(radii[i]), float(angles[j]))
                facts.detours.append(Detour(name, float(radii[i]), float(angles[j]),
                                            float(radii[i] + reach)))
            continue
        facts.failures.append(_no_point(ctx, dims, n, margin, radii, angles, ok, facts.allow,
                                        reach))
    for d in facts.detours:            # which O-free links each detour clears
        for n in facts.o_free:
            if min(geo.dist(("pt", d.name), ("seg", a, b)) for a, b in topo.links[n]) >= need:
                facts.hosts[n].append(d.name)
    return facts


def _no_point(ctx, dims, link, margin, radii, angles, ok, allow, reach) -> NoCrankPoint:
    topo = ctx.topo
    geo = topo.geometry
    miss = {p.name: min(geo.dist(("pt", p.name), ("seg", a, b)) for a, b in topo.links[link])
            for p in topo.axes_of("crankpin")}
    pin = _own_pin(topo, link)
    pin = pin if miss[pin] >= max(miss.values()) - 1e-9 else max(miss, key=miss.__getitem__)
    detour = None
    if ok.any():
        i = int(np.argmax(ok.any(axis=1)))
        rel = sorted((angles[ok[i]] - _pin_angle(topo, pin)) % 360.0)
        # the clear arc around its middle (it may wrap past 0)
        gap = max(range(len(rel)), key=lambda j: (rel[j] - rel[j - 1]) % 360.0)
        detour = (float(radii[i]), float(rel[gap]), float(rel[gap - 1]))
    return NoCrankPoint(link, pin, miss[pin], dims.post, ctx.params.link_radius, margin, detour,
                        reach, allow)


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

    def __init__(self, ctx: Context, dims: CrankDims, facts: CrankFacts, drop_bearing: bool):
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

    def hub_bottom(self, top: int) -> int:
        _, hub = hub_layers(Layout({}, top, self.pitch), self.drive, self.dims.hub_thickness)
        return hub.start if len(hub) else top

    # -- per layer ------------------------------------------------------------------

    def _prepare(self, view: RouteView) -> tuple[int, dict[int, int]] | RouteConflict:
        """(the hub's bottom layer, layer -> the point its riders ride), or why not."""
        h0 = self.hub_bottom(view.layout.top)
        if h0 < 2:
            return RouteConflict(1, max(h0, 1), "the hub leaves no layer for the crank below it")
        riding: dict[int, int] = {}
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
        return h0, riding

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
        """A partial layering: can a route pass everything placed so far (with a bound: is
        the cheapest one cheaper, a lower bound since unplaced riders may still free a run
        layer)? If so, the states each layer ``1..`` below the hub has on some route."""
        if view.bound is not None:
            res = self.route(view)
            if isinstance(res, RouteConflict):
                return res
        pre = self._prepare(view)
        if isinstance(pre, RouteConflict):
            return pre
        h0, riding = pre
        fwd = self._forward(view.blocked, riding, h0)
        if isinstance(fwd, int):
            return self._conflict(view, fwd, riding, h0)
        bwd = self._backward(view.blocked, riding, h0)
        return {k: fwd[k] & bwd[k] for k in range(1, h0)}

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
        """The cheapest route (a shortest path over layers x states); for a partial layering
        its cost is a lower bound."""
        pre = self._prepare(view)
        if isinstance(pre, RouteConflict):
            return pre
        h0, riding = pre
        blocked = view.blocked
        maybe: dict[int, set[int]] = {}
        if view.open is not None:
            for link, j in self.riders.items():
                for k in view.open.get(link, ()):
                    maybe.setdefault(k, set()).add(j)
        n = self.n
        ns = 4 + n
        INF = math.inf
        enter = [RUN + (FEATURE + self.sweep[j] * SWEEP if j >= len(self.pins) else 0)
                 for j in range(n)]

        def nodes(k: int) -> list[float]:
            va = self._valid(blocked.get(k, 0), riding.get(k))
            out = [0 if va >> s & 1 else INF for s in range(ns)]
            free = maybe.get(k, ())
            for j in range(n):
                if out[4 + j] == 0 and riding.get(k) != j and j not in free:
                    out[4 + j] = FEATURE
            return out

        nc = nodes(1)
        cost = [nc[S], nc[E] + BEARING, nc[L], INF] + [INF] * n
        back: list[list[int]] = []
        web_prev = self._webs(blocked.get(1, 0))
        for k in range(2, h0):
            wk = self._webs(blocked.get(k, 0))
            nc = nodes(k)
            new = [INF] * ns
            ptr = [-1] * ns
            moves = [(a, a) for a in (S, E, J)] + [(L, S), (L, E)]    # (to, from)
            moves += [(4 + j, a) for a in (L, J) for j in range(n) if web_prev >> j & 1]
            moves += [(4 + j, 4 + j) for j in range(n)]
            moves += [(J, 4 + j) for j in range(n) if wk >> j & 1]
            for to, frm in moves:
                c = cost[frm] + nc[to] + (enter[to - 4] if to >= 4 and frm < 4 else 0)
                if c < new[to]:
                    new[to], ptr[to] = c, frm
            if min(new) == INF:
                return self._conflict(view, k, riding, h0)
            cost = new
            back.append(ptr)
            web_prev = wk
        # into the hub's bottom layer: from the journal, or a run's web into the hub
        wh = self._webs(blocked.get(h0, 0))
        ends = [(cost[J], J)] + [(cost[4 + j], 4 + j) for j in range(n) if wh >> j & 1]
        if not riding:
            ends += [(cost[S], S), (cost[E], E)]
        end, frm = min(ends)
        if end == INF:
            return self._conflict(view, h0, riding, h0)
        if view.bound is not None and end // BEARING >= view.bound:
            return RouteConflict(1, h0, bound=True)
        states = [frm]
        for ptr in reversed(back):
            states.append(ptr[states[-1]])
        states.reverse()                   # states[i] is layer i + 1
        runs: list[Run] = []
        for k, s in enumerate(states, start=1):
            if s >= 4:
                p = self.points[s - 4]
                if runs and runs[-1].at == p and runs[-1].hi == k - 1:
                    runs[-1] = Run(p, runs[-1].lo, k)
                else:
                    runs.append(Run(p, k, k))
        return Route(CrankRoute(tuple(runs), bearing=states[0] != E), int(end) // BEARING)

    def _conflict(self, view: RouteView, dead: int, riding: dict[int, int],
                  h0: int) -> RouteConflict:
        """The layers that explain a dead end: ``1..dead`` seen from below, or from the
        hub down to where no state reaches it, whichever is shorter."""
        bwd = self._backward(view.blocked, riding, h0)
        top = next((k for k in range(h0 - 1, 0, -1) if not bwd[k]), None)
        if top is not None and h0 - top < dead:
            return RouteConflict(top, h0)
        return RouteConflict(1, dead)


__all__ = ["CrankFacts", "CrankRouter", "Detour", "NoCrankPoint", "crank_facts",
           "point_clearance"]
