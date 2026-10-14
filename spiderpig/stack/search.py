"""The search for the thinnest stack (:class:`StackProblem`, :class:`_Search`)."""


from __future__ import annotations

import itertools
import re
from collections.abc import Iterable, Mapping
from dataclasses import replace
from typing import TYPE_CHECKING

from spiderpig.mechanism import body_class
from spiderpig.stack.geometry import Disc, Layout, log, made
from spiderpig.stack.plan import Deadline, StackSpec
from spiderpig.stack.plan_z import (
    EPS_Z,
    GAP_GROUPS,
    HEADS_ORDER,
    PlanReject,
    bound_layout,
    finalize,
    heads_claims,
    settle,
)
from spiderpig.stack.topology import RouteConflict, RouteView
from spiderpig.stack.verify import verify_plan

if TYPE_CHECKING:
    from spiderpig.stack.geometry import Claim, Placed, Shape
    from spiderpig.stack.plan import StackPlan
    from spiderpig.stack.topology import Clearance, Router, Topology


HEIGHT_TOL = 1e-4
"""mm: two heights this close are equal (the objective's first term, rounded to it)."""


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
        self.heads_rank = 0           # (stack.drive) this head placement's place among the
        #                               searched: equal plans tie to the first
        self.give_up = 0              # (stack.drive) stop once this many layerings failed on
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
        self._makes: dict = {}        # (finalize) the claims' makes at the plans' z, memo
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
        self._fixed: dict[int, dict[int, float]] = {}
        self.syms: list = []          # (spec.symmetry: stack.drive) the leg swaps it may use

    # -- the search's z -----------------------------------------------------------------

    def fixed_sheets(self, top: int) -> dict[int, float]:
        """The layers a part with no links (a claim with no deps: the hand crank's head layer
        and keyed plates) makes thicker than the pitch in a stack of ``top + 1`` layers: what
        every plan of that size has there."""
        got = self._fixed.get(top)
        if got is None:
            got = {}
            probe = bound_layout(self.spec, {}, top)
            for c in self.raw_claims:
                if c.deps or c.final or c.choice is not None:
                    continue
                out, _ = made(c, probe)
                for p in out or ():
                    if p.sheet > self.spec.pitch + EPS_Z and not p.gap and 0 < p.layer < top:
                        got[p.layer] = max(got.get(p.layer, 0.0), p.sheet)
            self._fixed[top] = got
        return got

    def solo(self, c: Claim, early: bool, n: str, v: int, top: int
             ) -> tuple[list[Placed] | None, str]:
        """``c`` (its ``early`` part) made with link ``n`` alone in layer ``v`` of a stack of
        ``top + 1``: what a link brings on its own (:class:`_Search`), remembered across the
        searches of this problem (each job's search makes them again otherwise)."""
        memo = self.__dict__.setdefault("_solo", {})
        key = (id(c), early, n, v, top)
        got = memo.get(key)
        if got is None:
            got = memo[key] = made(replace(c, make=c.early) if early else c,
                                   self.bound_layout({n: v}, top))
        return got

    def bound_layout(self, layers: Mapping[str, int], top: int,
                     links: Iterable[str] | None = None) -> Layout:
        """The search's layout of ``layers`` (:func:`stack.plan_z.bound_layout`): the least
        z every plan of it can have and the upper bounds; ``links``: the placed links whose
        sheets it counts (all, when ``None``)."""
        return bound_layout(self.spec, layers, top, self.fixed_sheets(top), links)

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
            pair = self._blocked_by[key]
            if pair is None:                            # a claim that couldn't be built
                out.append(f"{count:7d} x {key[1]}")
                continue
            p, q = pair
            if q is None:
                why = "it would sit in a frame plate's layer"
            else:
                gap = geo.dist(p.shape.core, q.shape.core) - p.shape.r - q.shape.r
                why = f"{gap:.1f} mm apart in one layer, need {m:.1f}"
            out.append(f"{count:7d} x {key[0]} vs {key[1]}: {why}")
        return out

    # -- search ----------------------------------------------------------------------

    def solve(self) -> StackPlan:
        """The lowest plan (:meth:`key`: the stack's height, then the crank route's cost,
        then what its stock parts cost, then the parts it prints), with both head placements
        where ``spec.heads`` asks for both ("best", "gap_sink") and every stack size whose
        least height could beat the best found, each searched in a job of its own, here or
        in forked workers (:mod:`stack.drive`, :mod:`stack_pool`): the same answer either
        way. :class:`PlanError` (saying what blocked it, and how far each stack size got)
        when there is none up to ``spec.max_top``, or none was found before the effort ran
        out."""
        from spiderpig.stack.drive import solve

        return solve(self)

    def key(self, plan: StackPlan) -> tuple:
        """What the planner minimises, in order: the stack's height (frame plate to frame
        plate, mm: what packs, what the pillars and the pins span; with what a standoff
        longer than its column still holds it apart, :attr:`StackPlan.hold`), the crank
        route's cost (its added features: :attr:`StackPlan.cost`), what the stock parts the
        plan chooses cost (:attr:`StackPlan.usd`), the parts it prints
        (:attr:`StackPlan.printed`); then,
        so that equal plans tie the same way wherever they were found, the head placement
        (this problem's rank among the searched), the stack size, the layers and the
        route. ``spec.objective`` "layers" (the proxy the planner minimised before
        2026-10-14, kept for designs whose crank router prunes on its cost: the walkers):
        the fewest layers, then the route's cost; plans equal in those rank by the height
        and the rest, but the search stops at the first plan with no route cost (it doesn't
        search such a size on for a lower one: :class:`_Search`)."""
        tie = (self.heads_rank, plan.top, tuple(sorted(plan.layers.items())),
               repr(sorted((k, repr(v)) for k, v in plan.choices.items())))
        height = round(plan.height + plan.hold, 4)
        if self.spec.objective == "layers":
            return (plan.top, plan.cost, height, round(plan.usd, 2), plan.printed, *tie)
        return (height, plan.cost, round(plan.usd, 2), plan.printed, *tie)

    def size_lb(self, top: int) -> float:
        """The least height any plan of ``top + 1`` layers can have: every layer at least its
        bound with no link in it (the frame plates, a fixed part's sheet), and of the links
        thicker than the pitch, a set that pairwise share an axle (so no two share a layer)
        each in a layer of its own at its sheet."""
        h = self.bound_layout({}, top).height()
        return h + self._apart()

    def _apart(self) -> float:
        """(:meth:`size_lb`) The most mm a set of the thick links that pairwise share an axle
        adds over the pitch (greedily, thickest first: a lower bound either way)."""
        got = getattr(self, "_apart_mm", None)
        if got is None:
            pitch = self.spec.pitch
            thick = {n: t for n, t in self.spec.link_t if t > pitch + EPS_Z and n in self.partners}
            chosen: list[str] = []
            for n in sorted(thick, key=lambda x: (-thick[x], x)):
                if all(m in self.partners[n] for m in chosen):
                    chosen.append(n)
            got = self._apart_mm = sum(thick[n] - pitch for n in chosen)
        return got

    def tally(self, plan: StackPlan) -> StackPlan:
        """``plan`` with what it buys and prints (the claims' ``tally`` at its z)."""
        usd, printed = 0.0, 0
        for c in plan.claims:
            if c.tally is not None:
                u, n = c.tally(plan)
                usd, printed = usd + u, printed + n
        plan.usd, plan.printed = round(usd, 2), printed
        return plan

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
                            sink_all_but=self.gap_groups, memo=self._makes,
                            fixed=self.fixed_sheets(top))
            plan.heads = heads
            return self.tally(plan)
        claims = (self.claims if heads == self.heads
                  else heads_claims(self.raw_claims, heads, self.gap_groups))
        plan = finalize(self.topo, claims, self.spec, layers, top, choices,
                        memo=self._makes if claims is self.claims else None,
                        fixed=self.fixed_sheets(top))
        plan.heads = heads
        return self.tally(plan)


class _Search:
    """One stack size: forward checking and conflict-directed backjumping over link layers
    (with learned nogoods), the router as a sub-check at every node, and branch and bound
    on the objective (:meth:`StackProblem.key`): a node whose least height
    (:meth:`lb_height`) is over the best plan's, found here or given (``key_bound``), is
    cut.

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
        self.bound: int | None = None     # (a route's cost to beat: none, height first)
        self.best: StackPlan | None = None
        self.best_key: tuple | None = None
        self.key_bound: tuple | None = None   # a plan to beat from elsewhere (its key)
        self.lb_bad = 0                       # leaves under their own least height (a bug)
        self.height_first = prob.spec.objective == "height"
        self.lt = {n: t for n, t in prob.spec.link_t if t > prob.spec.pitch + EPS_Z}
        base = prob.bound_layout({}, top)
        self.t0 = [base.t(k) for k in range(top + 1)]     # each layer's least, no link placed
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
        # the pending counts an assignment of each link takes one from (early parts', then
        # the claims'), undone by one trail entry
        self.pend_keys = {n: tuple([-id(c) for c in self.by_early[n]]
                                   + [id(c) for c in self.by_dep[n]]) for n in self.links}
        self.early_of = {n: [(-id(c), id(c), c) for c in self.by_early[n]] for n in self.links}
        self.dep_of = {n: [(id(c), c) for c in self.by_dep[n]] for n in self.links}
        # keyed by slot: a layer, or a clearance gap's ``layer + 0.5``
        self.by_layer: dict[float, list[tuple[Placed, frozenset[str]]]] = {}
        self.block: dict[tuple[float, int], list[frozenset[str]]] = {}
        self.bmask: dict[float, int] = {}           # slot -> bit i: router piece i blocked
        self.dom = {n: set(range(1, top)) for n in self.links}
        self.gone: dict[str, dict[int, frozenset[str]]] = {n: {} for n in self.links}
        # what a link brings on its own in each layer (its claims that depend on it alone),
        # by the layer each shape lands in: forward checking against every placed shape;
        # and the router states it allows there
        self.touch: dict[str, dict[float, list[tuple[int, Placed]]]] = {n: {} for n in self.links}
        self.allow: dict[str, dict[int, int]] = {n: {} for n in self.links}
        for n in self.links:
            solo = [(c, False) for c in self.claims if self.deps[id(c)] == {n}]
            solo += [(c, True) for c in firsts if c.early_deps == {n}]
            for v in range(1, top):
                shapes: list[Placed] = []
                for c, early in solo:
                    out, why = prob.solo(c, early, n, v, top)
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
        # each domain again as bits over the layers (kept with ``dom``): what an axle's span
        # closes is one AND
        self.dm = {n: sum(1 << v for v in d) for n, d in self.dom.items()}
        self.lex: list[tuple[str, str]] | None = None   # (a, b): layer(a) <= layer(b)
        self.lexed: frozenset[str] = frozenset()

    # -- state --------------------------------------------------------------------

    def hit(self, a: Shape, b: Shape) -> bool:
        key = (a, b)
        h = self.pairs.get(key)
        if h is None:
            h = self.pairs[key] = self.geo.dist(a.core, b.core) < a.r + b.r + self.margin
        return h

    def stuck(self, p: Placed) -> bool:
        """Whether head ``p`` (in a clearance gap) can't sink into the layer beside it in any
        plan below this node: it may not (``toward`` 0, a frame plate there), or another
        group's shape placed there is in its way (what is placed stays)."""
        if not p.toward:
            return True
        k = p.layer + (1 if p.toward > 0 else 0)
        if not 0 < k < self.top:
            return True
        return any(q.group != p.group and self.hit(p.shape, q.shape)
                   for q, _ in self.by_layer.get(k, ()))

    def lb_height(self) -> float:
        """The least height (frame plate to frame plate, mm) of any plan below this node:
        each layer at least its bound (the frame plates, a fixed part's sheet, its placed
        links' sheets) and each clearance gap at least its tallest head where some head in
        it is :meth:`stuck` (a plan keeps that gap, and every head in it stays: the heads'
        heights here are the least they need, :class:`stack.Layout`)."""
        t = list(self.t0)
        lt = self.lt
        for n, k in self.layers.items():
            v = lt.get(n)
            if v is not None and v > t[k]:
                t[k] = v
        h = sum(t)
        top = self.top
        for slot, lst in self.by_layer.items():
            k = int(slot)
            if slot == k or not 0 <= k < top:
                continue
            heads = [p for p, _ in lst if p.height > 0]
            if heads and any(self.stuck(p) for p in heads):
                h += max(p.height for p in heads)
        return h

    def bound_h(self) -> float | None:
        """The height a plan below this node must not exceed (the best plan's, here or
        given): ``None`` before any."""
        keys = [k for k in (self.key_bound, self.best_key) if k is not None]
        return min(keys)[0] if keys else None

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
            # (the bound of its deps' sheets alone: the memo's key holds all it reads)
            layout = self.prob.bound_layout(self.layers, self.top, deps)
            out = self.made[key] = made(replace(c, make=c.early) if early else c, layout)
        return out

    def cut(self, n: str, v: int, why: frozenset[str]) -> None:
        self.dom[n].discard(v)
        self.dm[n] &= ~(1 << v)
        self.gone[n][v] = why
        self.trail.append(("cut", n, v))

    def undo(self, mark: int) -> None:
        trail = self.trail
        if len(trail) <= mark:
            return
        dom, gone, banned, pending = self.dom, self.gone, self.banned, self.pending
        block, bmask, dm = self.block, self.bmask, self.dm
        pop = trail.pop
        for _ in range(len(trail) - mark):
            e = pop()
            kind = e[0]
            if kind == "cut":
                if not banned or (e[1], e[2]) not in banned:
                    dom[e[1]].add(e[2])
                    dm[e[1]] |= 1 << e[2]
                    del gone[e[1]][e[2]]
            elif kind == "shape":
                k = e[1]
                self.by_layer[k].pop()
                for i in reversed(e[2]):
                    lst = block[(k, i)]
                    lst.pop()
                    if not lst:
                        bmask[k] &= ~(1 << i)
            elif kind == "pending":
                for i in e[1]:
                    pending[i] += 1
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
        pieces, ruled = self.effects(p)
        self.trail.append(("shape", k, pieces))      # (its blocks undone with it)
        if pieces:
            block, bmask = self.block, self.bmask
            m = bmask.get(k, 0)
            for i in pieces:
                block.setdefault((k, i), []).append(deps)
                m |= 1 << i
            bmask[k] = m
            self.dirty = True
        if ruled:
            layers, dom = self.layers, self.dom
            for n, v in ruled:
                if n not in layers and v in dom[n]:
                    self.cut(n, v, deps)
                    if not dom[n]:
                        return frozenset().union(*self.gone[n].values())
        return None

    def view(self, partial: bool) -> RouteView:
        """What the router sees now; ``partial``: the layers still open to each unplaced
        link too (the router then answers for the layering as a relaxation, and words a
        dead end of its own rules once per what they see, not per node)."""
        open_ = bits = None
        if partial:
            layers, dom, dm = self.layers, self.dom, self.dm
            open_ = {n: dom[n] for n in self.links if n not in layers}
            bits = {n: dm[n] for n in open_}
        return RouteView(self.prob.bound_layout(self.layers, self.top), self.bmask, open_,
                         self.bound, bits)

    def explain(self, res: RouteConflict, group: str) -> frozenset[str]:
        """The links behind a router's dead end (the router's ``group``): the ones it names,
        else everything in the layers it spans."""
        if res.bound:
            return frozenset(self.layers)
        self.prob._tally_why(group, self.describe(res))
        if res.links:
            return res.links
        lo, hi = res.lo, res.hi
        out: set[str] = {n for n, k in self.layers.items() if lo <= k <= hi}
        block = self.block
        for k, m in self.bmask.items():         # (the pieces blocked now: bit i set)
            if m and lo <= k <= hi:
                i = 0
                while m:
                    if m & 1:
                        for d in block[(k, i)]:
                            out |= d
                    m >>= 1
                    i += 1
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
        pending = self.pending
        keys = self.pend_keys[n]
        for i in keys:
            pending[i] -= 1
        self.trail.append(("pending", keys))
        for i, j, c in self.early_of[n]:
            if pending[i] or not pending[j]:   # (or the claim is due too)
                continue
            out, why = self.claim(c, early=True)
            if out is None:
                self.prob._tally_why(c.owner, why)
                return c.early_deps | {n}
            for p in out:
                if (conf := self.add(p, c.early_deps)) is not None:
                    return conf | {n}
        for i, c in self.dep_of[n]:
            if pending[i]:
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
        # (lexed: empty without lex)
        for a, b in self.lex if n in self.lexed else ():  # pyright: ignore[reportOptionalIterable]
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
                return self.explain(res, self.router.group) | {n}
            # a layer none of whose states on a route a link's own shapes leave is closed to it
            layers, dom, gone, trail, get = self.layers, self.dom, self.gone, self.trail, res.get
            dm = self.dm
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
                    m = dm[x]
                    for w in bad:
                        d.discard(w)
                        m &= ~(1 << w)
                        g[w] = why
                        trail.append(("cut", x, w))
                    dm[x] = m
                if not d:
                    return frozenset().union(*gone[x].values()) | {n}
        if self.height_first and (h := self.bound_h()) is not None and \
                self.lb_height() > h + HEIGHT_TOL:
            return frozenset(self.layers)      # (no plan below can be as low as the best)
        return None

    def spans(self, n: str) -> frozenset[str] | None:
        """An axle runs between its links' layers: a link that can't pass it can't sit between
        them, and once one sits on one side of some of them, the rest can't go to the other.
        A pillar also runs from them to a frame plate: such links can't be on both sides.
        (A domain these layers are already gone from is skipped: most are, an axle closes the
        same layers again at every node; an empty one still answers its conflict.)"""
        layers, dm, close, top = self.layers, self.dm, self.close, self.top
        for members, links, anchored in self.prob.spans.get(n, ()):
            placed = [m for m in members if m in layers]
            if not placed:
                continue
            ks = [layers[m] for m in placed]
            lo, hi = min(ks), max(ks)
            why = frozenset(placed)
            below = above = None
            inside = ((1 << hi) - 1) & ~((2 << lo) - 1)        # layers lo + 1 .. hi - 1
            for x in links:
                w = layers.get(x)
                if w is None:
                    d = dm[x]
                    if (not d or d & inside) and (c := close(x, inside, why)) is not None:
                        return c
                    continue
                if lo < w < hi:
                    return why | {x}
                if hi < w:
                    above = x
                    side = ((1 << top) - 1) & ~((1 << w) - 1)   # layers w .. top - 1
                else:
                    below = x
                    side = ((2 << w) - 1) & ~1                  # layers 1 .. w
                for m in members:
                    if m not in layers:
                        d = dm[m]
                        if (not d or d & side) and (
                                c := close(m, side, why | {x})) is not None:
                            return c
            end = below or above
            if anchored and end:
                if below and above:
                    return why | {below, above}
                # the pillar must reach the plate on the other side
                other = (((1 << top) - 1) & ~((2 << hi) - 1) if below     # hi + 1 .. top - 1
                         else ((1 << lo) - 1) & ~1)                       # 1 .. lo - 1
                for x in links:
                    if x not in layers:
                        d = dm[x]
                        if (not d or d & other) and (c := close(
                                x, other, why | {end})) is not None:
                            return c
        return None

    def close(self, x: str, ks: range | int, why: frozenset[str]) -> frozenset[str] | None:
        """Take layers ``ks`` (a range, or bits) from unplaced ``x``'s domain; its conflict if
        none are left. (Most calls take nothing: an axle closes the same layers again at
        every node.)"""
        if isinstance(ks, range):
            ks = sum(1 << k for k in ks if k >= 0)
        m = self.dm[x]
        hit = m & ks
        if hit:
            m &= ~hit
            self.dm[x] = m
            dom, gone, trail = self.dom[x], self.gone[x], self.trail
            while hit:
                low = hit & -hit
                u = low.bit_length() - 1
                hit ^= low
                dom.discard(u)
                gone[u] = why
                trail.append(("cut", x, u))
        return None if m else frozenset().union(*self.gone[x].values())

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
            self.dm[x] &= ~(1 << self.layers[x])
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
        attempt = 0
        while True:         # at most LEAF_REROUTES + 1 plans; every pass binds ``plan``
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
                    return self.explain(res, self.router.group)
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
            attempt += 1
        bad = verify_plan(plan)
        if bad:
            self.prob._tally_why("the plan at its z", bad[0])
            return frozenset(self.layers)
        plan.cost = cost
        if plan.height < self.lb_height() - 1e-6:
            # (the least height was over a plan's: a claim placed what its plan doesn't have)
            self.lb_bad += 1
            log.warning("%s: %d layers: a plan %.3f mm under its least height %.3f mm",
                        self.prob.topo.name, self.top + 1, plan.height, self.lb_height())
        key = self.prob.key(plan)
        if self.best_key is None or key < self.best_key:
            self.best, self.best_key = plan, key
        self.prob.found_any = True
        if not self.height_first:
            # (objective "layers": a cheaper route only, the router's bound; none is cheaper
            # than none added)
            self.bound = cost if self.bound is None else min(self.bound, cost)
            if self.bound == 0:
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
        if router is None or wb < 0 or not plan.layout.gaps:
            return {}
        made_: list[Placed] = []
        for c in plan.claims:
            got, _ = made(c, plan.layout)
            if got is not None:
                made_.extend(got)
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

    def prepare(self) -> bool:
        """The fixed claims placed, once, and the link the search places first (``first``);
        False when that already rules the size out (``done``)."""
        if self.base is not None:
            return not self.done
        self.base = 0
        for c in self.claims:
            if not self.deps[id(c)]:
                out, why = made(c, self.prob.bound_layout({}, self.top))
                if out is None:
                    self.prob._tally_why(c.owner, why)
                    self.done = True
                    return False
                if any(self.add(p, frozenset()) is not None for p in out):
                    self.done = True
                    return False
        self.base = len(self.trail)
        if any(not d for d in self.dom.values()):
            self.done = True
            return False
        self.first = min(self.links, key=lambda x: (len(self.dom[x]),
                                                     self.prob.depth.get(x, 99), x))
        return True

    def firsts(self) -> list[int]:
        """The layers the link placed first may take (:meth:`prepare`): a split of this size
        into parts that cover it once each (:meth:`only`)."""
        return sorted(self.dom[self.first]) if self.prepare() else []

    def only(self, layers: Iterable[int]) -> None:
        """Search only the part of this size with its first link in ``layers``."""
        if not self.prepare():
            return
        keep = set(layers)
        for v in sorted(self.dom[self.first]):
            if v not in keep:
                self.cut(self.first, v, frozenset())
        self.base = len(self.trail)
        if not self.dom[self.first]:
            self.done = True

    def run(self, budget: int, legs: bool = False, deadline: Deadline | None = None) -> bool:
        """Search this stack size with ``budget`` more nodes, until ``deadline`` (``legs``: a
        leg at a time, at the hint's layers); True once it has been searched to the end."""
        if self.done:
            return True
        self.deadline = deadline or Deadline()
        if not self.prepare():
            return True
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
            self.undo(self.base or 0)       # (prepared: the fixed claims' mark)
        return self.done
