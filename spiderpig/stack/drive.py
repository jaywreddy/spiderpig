"""The planner's driver (:meth:`StackProblem.solve`): the lowest plan, both head placements,
every stack size that could hold it, as independent jobs.

**The objective** (:meth:`StackProblem.key`): the stack's height, frame plate to frame plate
(what packs, and what the pillars and pins span; the heads outside the plates are the same
for every plan of a design), then the crank route's cost, then what the stock parts the
plan chooses cost, then the parts it prints; ties broken by a fixed key (the head
placement's rank, the size, the layers, the route), never by which search finished first.

**The jobs.** A job (:class:`Job`) searches one stack size with one head placement
(:class:`_Search`), with a node budget and, perhaps, a plan to beat (its key: the search
cuts every node whose least height, :meth:`_Search.lb_height`, is over that plan's). A
job's answer depends on nothing but the job, so it is the same whether it runs here or in
a worker (:mod:`stack_pool`, forked: ``StackSpec.workers``, ``SPIDERPIG_PLAN_WORKERS``,
none with ``SPIDERPIG_WORKERS=0``) and whenever it runs.

1. **A first plan**, as the planner always looked for the thinnest: short jobs
   (``quick_nodes``, each stopping at its first plan: ``StackSpec.quick_first``) from
   ``min_top`` up, every head placement side by side (and, with a hint, the plain search
   and a leg at a time side by side), jumping ahead in doubling steps past sizes their
   budget leaves open, then back down the skipped sizes while they keep a plan; an idle
   worker runs the next size up (a job asked twice runs once).
2. **The proof**: the best of those plans bounds every other. Every size and placement the
   objective needs (on the height: each whose least height, :meth:`StackProblem.size_lb`,
   isn't over the plan's; on "layers": the plan's size and every thinner one) and whose
   short job didn't search it to the end gets a full job (``max_nodes``; a placement's
   together no more than what its first step left of ``max_total_nodes``) with the best plan
   so far to beat, split into parts (``Job.at``: they cover the size once each, its budget
   shared), :data:`WAVE` sizes at a time from the thinnest (:meth:`_Drive._prove`); the jobs
   no one needs then are stopped.
3. **The answer**: the least key of every plan any job found, re-made here
   (:meth:`StackProblem.plan`, deterministic). It is optimal when every full job ran to
   the end (and no head placement gave up: :data:`stack.GIVE_UP`).

Budgets are nodes, per job, so the answer is the same on any machine and with any number
of workers; the CPU deadline (``StackSpec.max_seconds``, per process: each worker counts
its own) is the safety net, and a plan found when it runs out says what it left open.
"""

from __future__ import annotations

import multiprocessing
import os
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from spiderpig.stack.geometry import log
from spiderpig.stack.plan import Deadline
from spiderpig.stack.plan_z import GIVE_UP, HEADS_ORDER
from spiderpig.stack.search import HEIGHT_TOL, StackProblem, _Search
from spiderpig.stack.topology import PlanError

if TYPE_CHECKING:
    from spiderpig.stack.plan import StackPlan


ENV_WORKERS = "SPIDERPIG_PLAN_WORKERS"
"""The planner's worker processes (unset: :data:`MAX_WORKERS`, or the cores if fewer; 0 or 1:
none). ``SPIDERPIG_WORKERS=0`` turns them off too."""

WAVE = 4
"""Stack sizes the proof searches at a time, thinnest first (:meth:`_Drive._prove`): a fixed
number, so which sizes it searches doesn't depend on the workers."""

PARTS = 4
"""The most parts a proof job splits into (:meth:`_Drive._split`): each its run of the first
link's layers, its share of the job's budget."""

MAX_WORKERS = 8
"""The most workers a plan forks by default: a module's jobs at once (B, a cradle, has about
ten: five sizes, two head placements; the Strider double about twenty)."""


@dataclass(frozen=True)
class Job:
    """One stack size (``top``) searched with one head placement, ``budget`` nodes, a plan to
    beat (``bound``: its key), whether to stop at the first plan (``first``), the search's
    strategy (``legs``) and the part of the size it covers (``at``)."""

    heads: str
    top: int
    budget: int
    bound: tuple | None = None
    first: bool = False
    legs: bool | None = None    # a leg at a time (True), not (False), else the plain search
    #                             then, where it finds nothing, a leg at a time
    at: tuple[int, ...] | None = None   # only the part with its first link in these layers


def run_job(subs: Mapping[str, StackProblem], job: Job, deadline: Deadline) -> dict:
    """Search ``job`` (its own :class:`_Search`; with ``Job.legs`` unset and a hint, the
    plain search and then, where it found nothing, a leg at a time, half the budget each),
    until its budget or ``deadline``: what it found and what blocked it, as plain data (the
    same here and in a worker)."""
    sub = subs[job.heads]
    sub.blocked, sub._blocked_by = {}, {}
    sub.washer_misses, sub.gave_up, sub.found_any = 0, "", False
    t0 = time.monotonic()
    s = _Search(sub, job.top)
    s.key_bound, s.first_only = job.bound, job.first
    if not s.height_first and job.bound is not None and job.bound[0] == job.top:
        s.bound = job.bound[1]      # (objective "layers": a cheaper route in this size only)
    if job.at is not None:
        s.only(job.at)
    both = job.legs is None and bool(sub.hint)     # (the plain search, then a leg at a time:
    #                                                 half the budget each)
    s.run(job.budget // 2 if both else job.budget, bool(job.legs), deadline)
    if (both and s.best is None and not s.done and not sub.gave_up
            and not deadline.expired):
        s.run(job.budget - s.nodes, True, deadline)
    b = s.best
    return {"best": None if b is None else (dict(b.layers), dict(b.choices), b.cost),
            "key": s.best_key, "done": s.done, "nodes": s.nodes,
            "seconds": time.monotonic() - t0, "blocked": dict(sub.blocked),
            "by": dict(sub._blocked_by), "unbuilt": dict(s.unbuilt), "gave_up": sub.gave_up,
            "expired": deadline.expired and not s.done, "lb_bad": s.lb_bad}


def combine(parts: list[dict], cover: bool) -> dict:
    """One answer of several jobs of one size and placement: ``cover`` (a split, each part a
    layer of the first link): the best of them, searched to the end when all are; else
    alternatives (the plain search and a leg at a time, the first one's plan if it has one):
    to the end when either is."""
    keyed = [r for r in parts if r["key"] is not None]
    if cover:
        best = min(keyed, key=lambda r: r["key"]) if keyed else None
        done = all(r["done"] for r in parts)
    else:
        best = keyed[0] if keyed else None
        done = any(r["done"] for r in parts)
    blocked: dict = {}
    by: dict = {}
    unbuilt: dict = {}
    for r in parts:
        for k, n in r["blocked"].items():
            blocked[k] = blocked.get(k, 0) + n
            by.setdefault(k, r["by"].get(k))
        for k, n in r["unbuilt"].items():
            unbuilt[k] = unbuilt.get(k, 0) + n
    return {"best": None if best is None else best["best"],
            "key": None if best is None else best["key"], "done": done,
            "nodes": sum(r["nodes"] for r in parts),
            "seconds": sum(r["seconds"] for r in parts), "blocked": blocked, "by": by,
            "unbuilt": unbuilt, "gave_up": next((r["gave_up"] for r in parts if r["gave_up"]), ""),
            "expired": any(r["expired"] for r in parts) and not done,
            "lb_bad": sum(r["lb_bad"] for r in parts)}


class Here:
    """Jobs run in this process, one at a time, when their answer is asked for."""

    def __init__(self, subs: Mapping[str, StackProblem], seconds: float):
        self.subs = subs
        self.seconds = seconds
        self.results: dict[Job, dict] = {}

    def submit(self, job: Job, front: bool = False) -> None:
        pass

    def free(self) -> int:
        return 0

    def cancel(self, keep: Iterable[Job]) -> None:
        pass

    def ready(self, job: Job) -> bool:
        return job in self.results

    def result(self, job: Job) -> dict:
        got = self.results.get(job)
        if got is None:
            got = self.results[job] = run_job(self.subs, job, Deadline(self.seconds))
        return got

    def wait_any(self, jobs: list[Job]) -> None:
        if not any(j in self.results for j in jobs):
            self.result(jobs[0])

    def close(self) -> None:
        pass


def workers_for(spec) -> int:
    """How many worker processes a plan may fork: none with ``SPIDERPIG_WORKERS=0``, else
    ``spec.workers``, else :data:`ENV_WORKERS`, else :data:`MAX_WORKERS` (or the cores)."""
    if os.environ.get("SPIDERPIG_WORKERS") == "0":
        return 1
    if spec.workers is not None:
        return max(1, int(spec.workers))
    raw = os.environ.get(ENV_WORKERS)
    if raw not in (None, ""):
        return max(1, int(raw))
    return max(1, min(os.cpu_count() or 1, MAX_WORKERS))


def solve(prob: StackProblem) -> StackPlan:
    """:meth:`StackProblem.solve` (the module's docstring)."""
    spec = prob.spec
    modes = HEADS_ORDER.get(spec.heads, (prob.heads,))
    subs: dict[str, StackProblem] = {}
    for i, m in enumerate(modes):
        sub = prob if len(modes) == 1 else StackProblem(
            prob.topo, prob.raw_claims, replace(spec, heads=m), prob.router, prob.clearances,
            prob.hint)
        sub.heads_rank = i
        if m == "gap" and spec.heads == "gap_sink":
            # (a single-plate crank's gap search: TrotBot's heel and toe meet layering after
            # layering whose crank washers meet a pin's cap in the plan's gaps; the sunk
            # search plans them)
            sub.give_up = GIVE_UP
        if spec.symmetry:
            from spiderpig.stack_symmetry import symmetries

            sub.syms = symmetries(sub)
        subs[m] = sub
    n = workers_for(spec)
    runner: Here
    if n > 1 and "fork" in multiprocessing.get_all_start_methods():
        from spiderpig.stack_pool import Pool

        runner = Pool(subs, n, spec.max_seconds)
    else:
        runner = Here(subs, spec.max_seconds)
    try:
        return _Drive(prob, subs, modes, runner).run()
    finally:
        runner.close()


class _Drive:
    """One :func:`solve`: the first plans, the proof, the answer (the module's docstring)."""

    def __init__(self, prob: StackProblem, subs: dict[str, StackProblem],
                 modes: tuple[str, ...], runner: Here):
        self.prob, self.subs, self.modes, self.runner = prob, subs, modes, runner
        self.spec = prob.spec
        self.tried: dict[str, dict[int, dict]] = {m: {} for m in modes}
        self.order: list[tuple[str, int, dict]] = []      # every job asked for, in order
        self.gave_up: dict[str, str] = {}
        self.t0 = time.monotonic()

    # -- the jobs ---------------------------------------------------------------------

    def _take(self, m: str, top: int, r: dict) -> dict:
        self.tried[m][top] = r
        self.order.append((m, top, r))
        if r["gave_up"]:
            self.gave_up.setdefault(m, r["gave_up"])
        return r

    def _size(self, top: int, up: bool = True) -> dict[str, dict]:
        """Every placement still searched (not given up) at stack size ``top``: a short job
        each, side by side; ``up`` (the search climbing): idle workers run the next size's."""
        spec = self.spec
        modes = [m for m in self.modes if m not in self.gave_up]
        jobs = {m: self._quick(m, top) for m in modes}
        for js in jobs.values():
            for job in js:
                self.runner.submit(job, front=True)
        if up and self.runner.free() >= len(modes) and top < spec.max_top:
            # (the next size up, which the search asks next while sizes are ruled out)
            for m in modes:
                if top + 1 not in self.tried[m]:
                    self.runner.submit(self._quick(m, top + 1)[0])
        out = {}
        for m, js in jobs.items():
            first = self.runner.result(js[0])
            if len(js) > 1 and first["best"] is None and not first["done"] \
                    and not first["gave_up"] and not first["expired"]:
                first = combine([first, self.runner.result(js[1])], cover=False)
            out[m] = self._take(m, top, first)
        return out

    def _quick(self, m: str, top: int) -> list[Job]:
        """Size ``top``'s short jobs with placement ``m``: the plain search, and where the
        problem has a hint, a leg at a time beside it (its answer read only where the plain
        one finds nothing and doesn't end: as one search would run them in turn)."""
        spec = self.spec
        if not self.subs[m].hint:
            return [Job(m, top, spec.quick_nodes, None, spec.quick_first)]
        return [Job(m, top, spec.quick_nodes, None, spec.quick_first, legs=False),
                Job(m, top, spec.quick_nodes, None, spec.quick_first, legs=True)]

    def spent(self, m: str) -> int:
        """The nodes placement ``m``'s jobs asked for so far have taken."""
        return sum(r["nodes"] for mm, _, r in self.order if mm == m)

    def _split(self, job: Job) -> list[Job]:
        """A full job as up to :data:`PARTS` parts, each a run of the layers its first link
        may take (``Job.at``), its budget shared among them: they cover the size once each,
        and run side by side. A size with one layer or none stays one job."""
        s = _Search(self.subs[job.heads], job.top)
        layers = s.firsts()
        if len(layers) < 2:
            return [job]
        n = min(PARTS, len(layers))
        groups = [tuple(layers[i * len(layers) // n:(i + 1) * len(layers) // n])
                  for i in range(n)]
        each = max(1, -(-job.budget // n))      # (its budget, shared)
        return [replace(job, at=g, budget=each) for g in groups]

    def _firsts(self) -> int | None:
        """The thinnest stack size a short job of any placement finds a plan in (``None``:
        none, within ``max_total_nodes`` of each placement's short jobs): every size in
        turn while the short
        jobs rule them out; once one exhausts its budget instead, in doubling steps (a plan
        is then likely far thicker, and every size in between costs its whole budget), then,
        a plan found, back down the skipped sizes while they keep one. The placements go
        size by size together, so which sizes are searched depends on the jobs' answers
        alone."""
        spec = self.spec

        def spent() -> int:
            """The most nodes a placement's short jobs have taken (each has its budget)."""
            return max((self.spent(m) for m in self.modes), default=0)

        def stop(rs: dict[str, dict]) -> bool:
            return (spent() >= spec.max_total_nodes or len(self.gave_up) == len(self.modes)
                    or any(r["expired"] for r in rs.values()))

        top, jump, found = spec.min_top, 0, None
        while True:
            rs = self._size(top)
            if any(r["best"] is not None for r in rs.values()):
                found = top
                break
            if top >= spec.max_top or stop(rs):
                break
            if all(r["done"] for r in rs.values()):
                top, jump = top + 1, 0
            else:
                top, jump = min(top + (1 << jump), spec.max_top), jump + 1
        if found is not None:
            for top in range(found - 1, spec.min_top - 1, -1):
                if any(top in self.tried[m] for m in self.modes) or \
                        spent() >= spec.max_total_nodes:
                    break
                rs = self._size(top, up=False)
                if all(r["best"] is None for r in rs.values()):
                    break
                found = top
            return found
        for top in range(spec.min_top, spec.max_top + 1):
            if (spent() >= spec.max_total_nodes or len(self.gave_up) == len(self.modes)
                    or any(r["expired"] for _, _, r in self.order)):
                break
            if all(top in self.tried[m] for m in self.modes if m not in self.gave_up):
                continue
            rs = self._size(top)
            if any(r["best"] is not None for r in rs.values()):
                return top
            if stop(rs):
                break
        return None

    # -- the answer -------------------------------------------------------------------

    def run(self) -> StackPlan:
        spec, prob = self.spec, self.prob
        found = self._firsts()
        best = self._best(self.order)
        if best is None:
            self._merge_tally()
            last = max((t for m in self.modes for t in self.tried[m]), default=spec.min_top - 1)
            spent = sum(r["nodes"] for _, _, r in self.order)
            stopped = "; ".join(dict.fromkeys(
                [f"heads {m}: {w}" for m, w in self.gave_up.items()]
                + [f"the {spec.max_seconds:g} CPU s deadline ran out"
                   for _, _, r in self.order if r["expired"]][:1]
                + ([f"the {spec.max_total_nodes} search-step budget ran out"]
                   if spent >= spec.max_total_nodes else [])))
            raise PlanError(f"{prob.topo.name}: no layer plan found with up to {last + 1} layers"
                            f" after {spent} search steps"
                            + (f"; {stopped}" if stopped else ""), prob.blockers(),
                            [self.sizes()], expired=any(r["expired"] for _, _, r in self.order))
        full: list[tuple[str, int, dict]] = []
        if spec.prove:
            best = self._prove(best, full)
        self._merge_tally()
        key, m, top, r = best
        layers, choices, cost = r["best"]
        sub = self.subs[m]
        plan = sub.plan(layers, top, choices)
        plan.cost = cost
        if sub.key(plan) != key:
            log.warning("%s: the plan re-made differs from its search's: %s against %s",
                        prob.topo.name, sub.key(plan)[:4], key[:4])
        bad = sum(r["lb_bad"] for _, _, r in self.order)
        if bad:
            log.warning("%s: %d plans came under their least height", prob.topo.name, bad)
        plan.optimal, plan.proof = self._proof(plan, m, full, found)
        return plan

    def _needed(self, best: tuple) -> list[int]:
        """The stack sizes the objective needs searched to prove ``best`` (its key first):
        on "layers" every size up to its own; on the height every size whose least height
        isn't over its height."""
        spec, prob = self.spec, self.prob
        if spec.objective == "layers":
            return list(range(spec.min_top, best[2] + 1))
        out = []
        for top in range(spec.min_top, spec.max_top + 1):
            if prob.size_lb(top) > best[0][0] + HEIGHT_TOL:
                break               # (the least height grows with the size)
            out.append(top)
        return out

    def _prove(self, best: tuple, full: list[tuple[str, int, dict]]) -> tuple:
        """The proof: the sizes the objective needs (:meth:`_needed`) that no short job ran
        to the end, :data:`WAVE` sizes at a time from the thinnest up, each size and
        placement a full job against the best plan so far, its parts side by side
        (:meth:`_split`). After each wave the best may be better: on "layers" a size with a
        plan ends the proof (a taller one is never better), on the height the sizes still
        needed are those under the new best. A placement's jobs share what its first step
        left of ``max_total_nodes`` (its search's nodes in all, as they always were). The
        waves are the same whatever the workers, and so is the answer."""
        spec = self.spec
        live = [m for m in self.modes if m not in self.gave_up]
        todo = [t for t in self._needed(best)
                if any(not (r := self.tried[m].get(t)) or not r["done"] for m in live)]
        while todo:
            wave, todo = todo[:WAVE], todo[WAVE:]
            bound = best[0]
            jobs = []
            for m in live:
                left = [t for t in [*wave, *todo]
                        if not (r := self.tried[m].get(t)) or not r["done"]]
                share = max(1, (spec.max_total_nodes - self.spent(m)) // max(len(left), 1))
                for t in wave:
                    if t not in left:
                        continue
                    if spec.objective == "layers":
                        # (the thinner sizes: any plan; the plan's own: a cheaper route)
                        nominal = spec.max_nodes if t == best[2] else spec.max_nodes // 2
                        job = Job(m, t, min(nominal, share), bound if t == best[2] else None)
                    else:
                        job = Job(m, t, min(spec.proof_nodes or spec.max_nodes, share), bound)
                    jobs.append(job)
            parts = {job: self._split(job) for job in jobs}
            self.runner.cancel([p for ps in parts.values() for p in ps])
            for ps in parts.values():       # (a size guessed next that no one needs now: gone)
                for p in ps:
                    self.runner.submit(p)
            for job, ps in parts.items():
                r = combine([self.runner.result(p) for p in ps], cover=True)
                full.append((job.heads, job.top, r))
                self.order.append((job.heads, job.top, r))
                if r["gave_up"]:
                    self.gave_up.setdefault(job.heads, r["gave_up"])
            got = self._best(self.order)
            assert got is not None          # (the first plans are among them)
            best = got
            if spec.objective == "layers":
                if any(r["key"] is not None for _, _, r in full[-len(jobs):]):
                    break
                todo = [t for t in todo if t <= best[2]]
            else:
                needed = set(self._needed(best))
                todo = [t for t in todo if t in needed]
            live = [m for m in live if m not in self.gave_up]
        return best

    def _best(self, results: Iterable[tuple[str, int, dict]]
              ) -> tuple[tuple, str, int, dict] | None:
        got = [(r["key"], m, t, r) for m, t, r in results if r["key"] is not None]
        return min(got, key=lambda x: x[0]) if got else None

    def _merge_tally(self) -> None:
        """What blocked the searches, every job asked for in order, into the problem's tally
        (:meth:`StackProblem.blockers`)."""
        prob = self.prob
        prob.blocked, prob._blocked_by = {}, {}
        for _, _, r in self.order:
            for key, n in r["blocked"].items():
                prob.blocked[key] = prob.blocked.get(key, 0) + n
                prob._blocked_by.setdefault(key, r["by"].get(key))

    def _proof(self, plan: StackPlan, m: str, full: list[tuple[str, int, dict]],
               found: int | None) -> tuple[bool, str]:
        """(optimal, how it was established), deterministic in the jobs' answers: every
        (placement, size) the objective needs searched (objective "height": each size whose
        least height isn't over the plan's; "layers": each size up to the plan's) has a
        job that ran to the end, and no placement gave up."""
        spec, prob = self.spec, self.prob
        closed = {(mm, t) for mm, t, r in self.order if r["done"]}
        planned = [t for _, t, r in self.order if r["key"] is not None]
        thinnest = min(planned, default=plan.top)
        live = [mm for mm in self.modes if mm not in self.gave_up]
        out = [f"{plan.height:.2f} mm in {plan.top + 1} layers (heads {plan.heads})"]
        if thinnest > spec.min_top and all((mm, t) in closed for mm in live
                                           for t in range(spec.min_top, thinnest)):
            n = sum(r["nodes"] for _, t, r in self.order if t < thinnest)
            out.append(f"no plan in {thinnest} layers or fewer ({n} nodes)")
        if not spec.prove:
            out.append("the search stopped at the first plan: no proof asked")
            return False, "; ".join(out)
        if spec.objective == "layers":
            need = range(spec.min_top, plan.top + 1)
        else:
            need = [t for t in range(spec.min_top, spec.max_top + 1)
                    if prob.size_lb(t) <= plan.height + HEIGHT_TOL]
        open_ = sorted((t, mm) for mm in live for t in need if (mm, t) not in closed)
        why = sorted({("its budget" if not r["expired"] else
                       f"the {spec.max_seconds:g} CPU s deadline")
                      for mm, t, r in self.order if (t, mm) in open_})
        heads = " and ".join(live)
        if open_:
            out.append("not ruled out: " + ", ".join(f"{t + 1} layers (heads {mm})"
                                                     for t, mm in open_)
                       + (f" ({' and '.join(why)} ran out)" if why else ""))
        elif spec.objective == "layers":
            n = sum(r["nodes"] for _, t, r in self.order if t == plan.top)
            out.append(f"{plan.top + 1} layers searched to the end for the cheapest crank "
                       f"route with heads {heads} ({n} nodes)")
        else:
            n = sum(r["nodes"] for _, t, r in full)
            last = max(need, default=plan.top)
            out.append(f"every size up to {last + 1} layers whose least height isn't over it "
                       f"searched to the end with heads {heads} ({n} nodes in the proof)")
        for mm, w in self.gave_up.items():
            out.append(f"heads {mm}: {w}")
        forced = [(t + 1, why_, n) for _, t, r in self.order if t < thinnest
                  for why_, n in r["unbuilt"].items()]
        if forced and prob.router is not None:
            t, why_, n = forced[-1]
            out.append(f"the {prob.router.group}'s own rules forced it taller: in {t} "
                       f"layers the search met {n} layouts that fit everything else but {why_}")
        return not open_ and not self.gave_up, "; ".join(out)

    def sizes(self) -> str:
        """How far each stack size got: a plan found, ruled out, left open at its budget, or
        not tried (for :class:`PlanError`)."""
        spec = self.spec
        runs: list[tuple[str, list[int]]] = []
        for top in range(spec.min_top, spec.max_top + 1):
            rs = [r for m in self.modes if (r := self.tried[m].get(top)) is not None]
            state = ("not tried" if not rs else "found" if any(r["best"] for r in rs)
                     else "ruled out" if all(r["done"] for r in rs)
                     else "left open at their budget")
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
            nodes = sorted(sum(r["nodes"] for m in self.modes
                               if (r := self.tried[m].get(t)) is not None) for t in tops)
            span = f"{nodes[0]}" if nodes[0] == nodes[-1] else f"{nodes[0]}-{nodes[-1]}"
            each = " each" if len(tops) > 1 else ""
            out.append(f"{layers} {state} ({span} nodes{each})")
        return "sizes: " + "; ".join(out)
