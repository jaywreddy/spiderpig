"""A reference layer planner for small designs: every layering, every crank route.

Independent of :mod:`stack`'s search and :mod:`construction.route`'s router:
it enumerates link layers (skipping only pairs of links whose own shapes
collide, which no route can change), builds every claim for each, and tries
every crank route: sets of runs (a run point and a layer range, webs in the
layers either side, ``1..`` below the hub) that put each rider in a run of
its own crankpin. A layering and route count when no two shapes of different
groups come too close in one slot (a layer, or the clearance gap over it: the
planner's own distances), nothing but seats sits in a frame plate's layer, the bolt
crank can build the route (:func:`buildable`: its standoffs and plates as
:class:`construction.crank.BoltCrank` fits and ``_WebPlates`` makes them), and the plan
of the layering builds at its own z and verifies (:meth:`stack.StackProblem.plan`,
:func:`stack.verify_plan`: the stage after the search). Cost: the planner's objective,
in order: added crank features (run layers no rider of their point is in, detour runs),
the detours' sweep, a dropped bearing.

The heads are the planner's ``heads="gap"`` search's: every claim as its construction
makes it (a fastener's head in the clearance gap beside its link), so a test compares
the brute force with a design configured ``heads="gap"``. The crank's run washers in a
gap are left to the plan's check (the planner routes again round the gaps a plan has
where they meet another group's: :meth:`stack._Search.leaf`), as are the crank's posts
and webs against the other groups' clearance shapes.
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Iterator

import numpy as np

from spiderpig.construction.crank import CrankRoute, Run
from spiderpig.stack import Layout, Placed, PlanReject, StackProblem, made, verify_plan


def routes(points: list[str], lo: int, hi: int, need: dict[int, str],
           bearing: tuple[bool, ...] = (True,)) -> Iterator[CrankRoute]:
    """Every route whose runs sit in layers ``lo..hi``, one layer apart at least, with the
    layers in ``need`` (layer -> point) inside a run along their point."""

    def rec(k: int) -> Iterator[tuple[Run, ...]]:
        # every layer below k that needs a run is in one; the next run starts at k or later
        if all(j < k for j in need):
            yield ()
        for a in range(k, hi + 1):
            if any(k <= j < a for j in need):
                break                       # a layer that needs a run would be left out
            for b in range(a, hi + 1):
                pts = {need[j] for j in need if a <= j <= b}
                if len(pts) > 1:
                    break
                if b + 1 in need:           # its upper web can't sit in a rider's layer
                    continue
                for p in pts or points:
                    for rest in rec(b + 2):
                        yield (Run(p, a, b), *rest)

    if any(not lo <= j <= hi for j in need):
        return
    for runs in rec(lo):
        if runs:
            for keep in bearing:
                yield CrankRoute(runs, keep)


def cost(route: CrankRoute, layers: dict[str, int], riders: dict[str, str],
         detours: dict[str, float]) -> tuple[int, float, int]:
    ridden = {(k, riders[n]) for n, k in layers.items() if n in riders}
    extra = sum(1 for r in route.runs for k in range(r.lo, r.hi + 1) if (k, r.at) not in ridden)
    extra += sum(1 for r in route.runs if r.at in detours)
    return extra, sum(detours[r.at] for r in route.runs if r.at in detours), int(not route.bearing)


def crank_of(ctx):
    """The side's crank construction, for its sheet (as :mod:`construction` resolves it)."""
    from spiderpig import construction

    return construction.crank(ctx.config.crank).resolve(ctx)


def buildable(route: CrankRoute, layers: dict[str, int], problem: StackProblem, ctx,
              h0: int) -> bool:
    """The bolt crank's single plates and standoffs, as :class:`construction.crank.BoltCrank`
    fits them and ``_WebPlates`` builds them: every run along its own point between two
    plates (a standoff can't take a plate turning on it between two runs, so one run per
    point), each run's span one a stock standoff fits; the runs one above the other, each
    next run's lowest plate the run below's top plate or over it with a journal standoff
    on O between the two plates (one a stock length fits); the last run's top plate the
    hub plate (the hub's lowest layer, ``h0``: the horn screws come up through it); the
    first run's lowest plate in a layer the stub standoff reaches the outer frame plate
    from (with the bearing), with no rider under it (the stub runs through those layers);
    and two consecutive runs' (and the last run's and the horn's) screw heads apart."""
    crank = crank_of(ctx)
    pitch, t = ctx.pitch, ctx.sheet_t("crank")
    runs = sorted(route.runs, key=lambda r: r.lo)
    if len({r.at for r in runs}) < len(runs):
        return False                        # a point carries one run
    if runs[-1].hi + 1 != h0:
        return False                        # the last run ends in the hub plate
    ridden_layers = {layers[n] for n in problem.topo.riders}
    if route.bearing:
        a = runs[0].lo - 1
        if a not in crank.stub_layers_web(ctx.sheet_t("frame"), pitch, t):
            return False
        if any(k in ridden_layers for k in range(1, a)):
            return False                    # the stub passes those layers
    for r in runs:
        if not crank._web_span_ok(r.hi - r.lo + 1, 0, pitch, t):
            return False
    for a, b in itertools.pairwise(runs):
        e, f = a.hi + 1, b.lo - 1
        if f < e:
            return False
        if f > e and not crank._web_span_ok(f - e - 1, 0, pitch, t):
            return False                    # the journal standoff between the two plates
        if any(k in ridden_layers for k in range(e + 1, f)):
            return False                    # the journal on O passes a rider's layer
    geo = problem.topo.geometry
    xy = {p: geo.points[p][0] for p in {r.at for r in runs}}
    head = crank.head_r()
    post = crank.rider_d() / 2
    for x, y in itertools.pairwise(runs):
        if x.at != y.at and math.dist(xy[x.at], xy[y.at]) < head + max(head, post):
            return False
    drive = ctx.interfaces["drive"]
    o, pin = geo.points["O"][0], geo.points[problem.topo.axes_of("crankpin")[0].name][0]
    theta = math.atan2(pin[1] - o[1], pin[0] - o[0]) + drive.pattern_angle
    horn = [(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]), drive.screw_head_d / 2)
            for a in (theta + 2 * math.pi * k / drive.screw_count
                      for k in range(drive.screw_count))]
    if drive.center_head_d > 0:
        horn.append((o, (drive.center_head_d + ctx.params.print_fit) / 2))
    return all(math.dist(xy[runs[-1].at], h) >= head + r for h, r in horn)


def _crank_washer(p: Placed, group: str) -> bool:
    return p.group == group and p.gap and p.label.endswith(" washer")


def solve(problem: StackProblem, top: int, ctx,
          only=None) -> tuple[tuple, dict[str, int], CrankRoute] | None:
    """The cheapest layering and route in ``top + 1`` layers (``ctx``: the side's context, for
    the crank's construction and the drive), or ``None``. ``only(layers, h0)``: the routes
    to try instead of every route (e.g. one run per crankpin over the whole stack)."""
    geo, m, pitch = problem.topo.geometry, problem.spec.margin, problem.spec.pitch
    router = problem.router
    links = list(problem.links)
    riders = problem.topo.riders
    h0 = router.hub_bottom(top)
    detours = {d.name: d.sweep for d in router.facts.detours}
    points = [a.name for a in problem.topo.axes_of("crankpin")] + list(detours)
    fixed = [c for c in problem.claims if c.choice is None]
    routed = [c for c in problem.claims if c.choice is not None]

    dists: dict[tuple, float] = {}

    def clear(a: Placed, b: Placed) -> bool:
        if a.seat or b.seat or a.slot != b.slot or a.group == b.group:
            return True
        key = (a.shape.core, b.shape.core)
        d = dists.get(key)
        if d is None:
            d = dists[key] = geo.dist(*key)
        return d >= a.shape.r + b.shape.r + m

    def in_plate(p: Placed) -> bool:
        return not p.seat and not p.gap and p.layer in (0, top)

    def own(n: str, k: int) -> list[Placed]:
        return [p for c in fixed if c.deps == {n}
                for p in (made(c, Layout({n: k}, top, pitch))[0] or ())]

    solo = {(n, k): own(n, k) for n in links for k in range(1, top)}
    apart = {(a, b, k): all(clear(p, q) for p in solo[(a, k)] for q in solo[(b, k)])
             for a, b in itertools.combinations(links, 2) for k in range(1, top)}
    memo: dict[tuple, list[Placed] | None] = {}

    def built(c, layers) -> list[Placed] | None:
        key = (id(c), *(layers[d] for d in sorted(c.deps)))
        if key not in memo:
            memo[key] = made(c, Layout(layers, top, pitch))[0]
        return memo[key]

    # A fixed claim's shapes depend on its links' layers only (``built``'s key): each is
    # made and checked as soon as its last link has a layer (a claim with no links before
    # the first, a ``final`` one at the end), so a layering that fails is cut off where it
    # first fails. The layerings that pass, in the order they pass, are those of
    # enumerating them all and checking each whole.
    ready: list[list] = [[] for _ in range(len(links) + 1)]    # ready[i]: once links[:i]
    for c in fixed:
        need_ = set(links) if c.final else set(c.deps)
        ready[max((links.index(d) + 1 for d in need_), default=0)].append(c)

    def place(cs, layers, by_slot) -> list[Placed] | None:
        """The shapes of claims ``cs`` when they build and clear ``by_slot`` and each
        other (nothing but seats in a frame plate's layer), else ``None``."""
        new: list[Placed] = []
        for c in cs:
            out = built(c, layers)
            if out is None:
                return None
            new += out
        if any(in_plate(p) for p in new):
            return None
        if not all(clear(a, b) for a, b in itertools.combinations(new, 2)):
            return None
        if not all(clear(a, b) for a in new for b in by_slot.get(a.slot, ())):
            return None
        return new

    def layerings(i: int, layers: dict[str, int], by_slot: dict[float, list[Placed]]
                  ) -> Iterator[tuple[dict[str, int], dict[float, list[Placed]]]]:
        """Every layering whose fixed claims build and clear each other (skipping first
        links whose own shapes collide in a shared layer), with its shapes by slot."""
        if i == len(links):
            yield dict(layers), by_slot
            return
        n = links[i]
        for k in range(1, top):
            if all(layers[mm] != k or apart[(mm, n, k)] for mm in links[:i]):
                layers[n] = k
                new = place(ready[i + 1], layers, by_slot)
                if new is not None:
                    grown = {j: list(ps) for j, ps in by_slot.items()}
                    for p in new:
                        grown.setdefault(p.slot, []).append(p)
                    yield from layerings(i + 1, layers, grown)
                del layers[n]

    best = None
    joints: dict[tuple, bool] = {}
    first = place(ready[0], {}, {})
    if first is None:
        return None
    start: dict[float, list[Placed]] = {}
    for p in first:
        start.setdefault(p.slot, []).append(p)
    group = router.group
    for layers, by_slot in layerings(0, {}, start):
        need = {k: riders[n] for n, k in layers.items() if n in riders}
        keep = (True, False) if problem.spec.drop_bearing else (True,)
        for route in (only(layers, h0) if only else routes(points, 2, h0 - 1, need, keep)):
            c = cost(route, layers, riders, detours)
            if best is not None and c >= best[0]:
                continue
            # (the crank's joints first: they rule out most routes, and cost a fraction of
            # making the crank; ``buildable`` reads the layering only at the riders' layers)
            jkey = (route, *(layers[n] for n in riders))
            ok = joints.get(jkey)
            if ok is None:
                ok = joints[jkey] = buildable(route, layers, problem, ctx, h0)
            if not ok:
                continue
            crank: list[Placed] = []
            for claim in routed:
                out, _ = made(claim, Layout(layers, top, pitch, {group: route}))
                if out is None:
                    break
                crank += out
            else:
                if any(in_plate(p) for p in crank):
                    continue
                if not all(clear(a, b) for a in crank if not _crank_washer(a, group)
                           for b in by_slot.get(a.slot, ())):
                    continue
                try:
                    plan = problem.plan(layers, top, {group: route})
                except PlanReject:
                    continue
                if verify_plan(plan):
                    continue
                best = (c, dict(layers), route)
    return best
