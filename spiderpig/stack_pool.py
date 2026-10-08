"""The layer planner's stack sizes searched in parallel, with the serial planner's answer.

A prototype behind ``StackSpec.workers`` (``> 1``; Linux, ``fork``).
:meth:`stack.StackProblem.solve` runs exactly as it does serially: the same sizes, the
same runs of each, in the same order, the same budgets. Only where each size's search
runs changes: every stack size lives in a worker process of its own (forked when the size
is first searched, so the problem, its claims and the router come over as they are;
nothing is pickled but the answers), and while the coordinator waits for the run it
needs, idle workers run the runs it is likely to ask for next (*speculation*): the next
sizes up while sizes are being ruled out, the sizes below a plan while it descends, and,
once a plan is found, the proof of every thinner size and the cheaper route in the size
found, all at once.

Each size's search is deterministic, so a run the coordinator asks for is the run a worker
already made whenever the worker got the same runs before it, with the same budget. Two
cases differ and are handled exactly:

* a run made with more nodes than the coordinator's (the total node budget,
  ``StackSpec.max_total_nodes``, leaves it less; or a size proved where a short search was
  guessed): the worker records, per node, what the run did (plans found, what blocked it,
  the crank's rules), so the run cut at the smaller budget is read off the longer one, and
  a later run of that size replays;
* the coordinator asks a size for something its worker didn't run next (speculation guessed
  wrong): that worker is dropped and a new one replays the size's runs so far, then runs it.

What blocked the search (the tally a :class:`stack.PlanError` lists) is replayed in the serial
order. With a wall-clock deadline (the default 60 s) the workers simply get further than one
process would in the same time, as a faster planner does.
"""

from __future__ import annotations

import contextlib
import multiprocessing
import time
import traceback
from multiprocessing.connection import wait

from spiderpig import stack


class Remote:
    """A stack size whose search runs in a worker: what the coordinator knows of it (the
    attributes :class:`stack.StackProblem` reads of a :class:`stack._Search`)."""

    def __init__(self, prob: stack.StackProblem, top: int):
        self.prob, self.top = prob, top
        self.nodes, self.seconds, self.done = 0, 0.0, False
        self.best: stack.StackPlan | None = None
        self.unbuilt: dict[str, int] = {}
        self.history: list[tuple] = []            # the runs applied: (budget, legs, first)
        self.pos = 0                                   # the worker's next answer to read
        self.last_seconds = 0.0
        self.first_only = False   # (StackSpec.quick_first: the run stops at its first plan)
        self.stale = False      # its worker ran further than the runs applied (a run cut short)

    def run(self, budget: int, legs: bool = False, deadline=None) -> bool:
        self.prob.pool.apply(self, budget, legs)
        return self.done


def _worker(prob: stack.StackProblem, top: int, conn, search=None) -> None:
    """A forked worker: one stack size's search (``search``: the coordinator's, as it was at
    the fork), run by run, with what each run did per node."""
    try:
        tally: list[tuple[int, int]] = []
        keys: dict[tuple, int] = {}
        fresh: list[tuple[int, tuple, object]] = []
        holder: list[stack._Search | None] = [None]

        def node() -> int:
            return holder[0].nodes if holder[0] is not None else 0

        def rec_tally(p, q) -> None:
            key = (p.label or p.group, "a frame plate" if q is None else (q.label or q.group))
            rec(key, (p, q))

        def rec_tally_why(owner: str, why: str) -> None:
            rec((owner, why), None)

        def rec(key: tuple, by) -> None:
            kid = keys.get(key)
            if kid is None:
                kid = keys[key] = len(keys)
                fresh.append((kid, key, by))
            tally.append((node(), kid))

        prob._tally, prob._tally_why = rec_tally, rec_tally_why
        unbuilt: list[tuple[int, str]] = []
        found: list[tuple[int, dict, dict, int]] = []

        class Counted(dict):
            def __setitem__(self, k, v):
                unbuilt.append((node(), k))
                super().__setitem__(k, v)

        s = holder[0] = search if search is not None else stack._Search(prob, top)
        s.unbuilt = Counted(s.unbuilt)
        leaf = s.leaf

        def recorded_leaf():
            before = s.best
            try:
                return leaf()
            finally:
                if s.best is not before:
                    b = s.best
                    found.append((s.nodes, dict(b.layers), dict(b.choices), b.cost))

        s.leaf = recorded_leaf
        while True:
            cmd = conn.recv()
            if cmd is None:
                break
            idx, kind, budget, first = cmd
            if kind == "legs_if" and not (s.best is None and prob.hint):
                conn.send((top, idx, None))
                continue
            before, t0 = s.nodes, time.monotonic()
            s.first_only = first
            s.run(budget, kind in ("legs", "legs_if"), prob.deadline)
            out = {"before": before, "after": s.nodes, "done": s.done,
                   "seconds": time.monotonic() - t0, "tally": tally[:], "keys": fresh[:],
                   "unbuilt": unbuilt[:], "found": found[:]}
            tally.clear(), fresh.clear(), unbuilt.clear(), found.clear()
            conn.send((top, idx, out))
    except BaseException:  # noqa: BLE001 - the coordinator raises it (the traceback, below)
        with contextlib.suppress(Exception):
            conn.send((top, -1, traceback.format_exc()))
    finally:
        conn.close()


class Pool:
    """The workers of one :meth:`stack.StackProblem.solve`, at most ``n`` running at once."""

    def __init__(self, prob: stack.StackProblem, n: int):
        self.prob, self.n = prob, n
        self.ctx = multiprocessing.get_context("fork")
        self.procs: dict[int, tuple] = {}            # top -> (process, connection)
        self.issued: dict[int, list[tuple[str, int, bool]]] = {}  # top -> (kind, budget, ahead)
        self.phases: dict[int, set[str]] = {}       # top -> the phases speculated
        self.answers: dict[tuple[int, int], dict | None] = {}
        self.pending: dict[int, int] = {}            # top -> commands not answered yet
        self.keys: dict[int, dict[int, tuple]] = {}  # top -> key id -> (key, first pair)
        self.ahead: list[tuple[int, str]] = []       # speculation: (top, phase) by priority
        self.stats = {"runs": 0, "speculated": 0, "used": 0, "replays": 0, "cut": 0}

    # -- workers ---------------------------------------------------------------------

    def has(self, top: int) -> bool:
        """Whether a worker searches ``top`` (the coordinator then makes a :class:`Remote`)."""
        return top in self.procs

    def adopt(self, s: stack._Search) -> Remote:
        """Hand the coordinator's own search of a size to a worker (forked with it)."""
        r = Remote(self.prob, s.top)
        r.nodes, r.seconds, r.done, r.best = s.nodes, s.seconds, s.done, s.best
        r.unbuilt = dict(s.unbuilt)
        r.history = list(self.prob.runs_of.get(s.top, ()))
        self._spawn(s.top, s)
        self.prob.tried[s.top] = r
        return r

    def _spawn(self, top: int, search=None) -> None:
        a, b = self.ctx.Pipe()
        p = self.ctx.Process(target=_worker, args=(self.prob, top, b, search), daemon=True)
        p.start()
        b.close()
        self.procs[top] = (p, a)
        self.issued[top] = []
        self.phases[top] = set()
        self.pending[top] = 0
        self.keys[top] = {}

    def _send(self, top: int, kind: str, budget: int, ahead: bool = False,
              first: bool = False) -> None:
        if top not in self.procs:
            self._spawn(top)
        q = self.issued[top]
        q.append((kind, budget, ahead, first))
        self.pending[top] += 1
        self.procs[top][1].send((len(q) - 1, kind, budget, first))
        self.stats["runs"] += 1
        self.stats["speculated"] += ahead

    def _drop(self, top: int) -> None:
        p, conn = self.procs.pop(top)
        with contextlib.suppress(OSError):
            conn.send(None)
        conn.close()
        p.terminate()
        p.join()
        for key in [k for k in self.answers if k[0] == top]:
            del self.answers[key]
        self.pending.pop(top, None)
        self.issued.pop(top, None)
        self.phases.pop(top, None)

    def close(self) -> None:
        for top in list(self.procs):
            self._drop(top)

    def _busy(self) -> int:
        return sum(1 for t, n in self.pending.items() if n)

    def _pump(self) -> None:
        """Receive what any worker answers (blocking until one does)."""
        conns = {c: t for t, (_, c) in self.procs.items() if self.pending.get(t)}
        for c in wait(list(conns)):
            top, idx, out = c.recv()
            if idx < 0:
                raise RuntimeError(f"stack size {top + 1} failed in its worker:\n{out}")
            self.pending[top] -= 1
            self.answers[(top, idx)] = out
            if out is not None:
                self.keys[top].update({kid: (key, by) for kid, key, by in out["keys"]})

    # -- speculation -----------------------------------------------------------------

    def expect(self, ahead: list[tuple[int, str]]) -> None:
        """The runs the coordinator is likely to ask for next, most likely first:
        ``(top, phase)``, phase ``quick`` (a short search, then a leg at a time if it found
        nothing), ``prove`` (the same with the proof's budget) or ``route``."""
        self.ahead = ahead
        self._speculate()

    def _speculate(self) -> None:
        spec = self.prob.spec
        for top, phase in self.ahead:
            if self._busy() >= self.n:
                return
            if self.pending.get(top) or top > spec.max_top or top < spec.min_top:
                continue
            tried = self.prob.tried.get(top)
            if tried is not None and tried.done:
                continue
            if isinstance(tried, stack._Search):
                if phase == "quick":
                    continue
                tried = self.adopt(tried)          # its proof or route goes on in a worker
            if tried is None and phase != "quick" and top in self.procs:
                # a short search guessed for a size the search never asked one of: its proof
                # starts from a fresh worker now, in parallel, not as a replay later
                self.stats["respawned"] = self.stats.get("respawned", 0) + 1
                self._drop(top)
            q = self.issued.get(top, [])
            ran = self.phases.get(top, set())
            if tried is not None and tried.stale:
                continue
            first = phase == "quick" and spec.quick_first
            if phase == "quick" and not q:
                budget = spec.quick_nodes
            elif phase == "prove" and not ran & {"prove", "route"}:
                budget = spec.max_nodes // 2
            elif phase == "route" and "route" not in ran:
                budget = spec.max_nodes
            else:
                continue
            self._send(top, "run" if phase != "route" else "route", budget, True, first)
            if phase != "route":
                self._send(top, "legs_if", budget, True, first)
            self.phases[top].add(phase)

    # -- the coordinator's runs --------------------------------------------------------

    def apply(self, s: Remote, budget: int, legs: bool) -> None:
        """Run ``s`` with ``budget`` (a leg at a time with ``legs``) as the serial planner
        would, from what its worker ran, or by asking it now."""
        top = s.top
        if s.stale:
            self.stats["replays"] += 1
            self._replay(s, budget, legs)
            return
        while True:
            q = self.issued.get(top, [])
            if s.pos >= len(q):
                self._send(top, "run" if not legs else "legs", budget, first=s.first_only)
                q = self.issued[top]
            kind, issued, ahead, first = q[s.pos]
            while (top, s.pos) not in self.answers:
                self._speculate()
                self._pump()
            out = self.answers.pop((top, s.pos))
            s.pos += 1
            if out is None:                        # a leg at a time, not needed: not asked
                continue
            if (kind in ("legs", "legs_if")) != legs or issued < budget or first != s.first_only:
                self.stats["replays"] += 1
                self._replay(s, budget, legs)
                return
            self.stats["used"] += ahead
            self._take(s, out, budget, legs, issued)
            return

    def _replay(self, s: Remote, budget: int, legs: bool) -> None:
        """A new worker for ``s``: its runs so far again, then this one."""
        top = s.top
        self._drop(top)
        self._spawn(top)            # a fresh search of it
        for b, lg, first in s.history:
            self._send(top, "legs" if lg else "run", b, first=first)
        self._send(top, "legs" if legs else "run", budget, first=s.first_only)
        last = len(self.issued[top]) - 1
        while (top, last) not in self.answers:
            self._pump()
        out = self.answers.pop((top, last))
        for i in range(last):
            self.answers.pop((top, i), None)
        s.pos = last + 1
        s.stale = False
        self._take(s, out, budget, legs, budget)

    def _take(self, s: Remote, out: dict, budget: int, legs: bool, issued: int) -> None:
        """Apply a worker's run (made with ``issued`` nodes) to the coordinator: as it ran, or
        cut where a run of ``budget`` nodes stops (the node past it, which does nothing)."""
        prob = self.prob
        cut = out["before"] + budget
        if out["after"] - out["before"] > budget:          # the serial run stops there
            nodes, done = cut + 1, False
            if issued > budget:        # its worker went further: a later run of it replays
                self.stats["cut"] += 1
                s.stale = True
        else:
            nodes, done, cut = out["after"], out["done"], out["after"]
        keys = self.keys[s.top]
        for at, kid in out["tally"]:
            if at <= cut:
                key, by = keys[kid]
                prob.blocked[key] = prob.blocked.get(key, 0) + 1
                prob._blocked_by.setdefault(key, by)
        for at, why in out["unbuilt"]:
            if at <= cut:
                s.unbuilt[why] = s.unbuilt.get(why, 0) + 1
        for at, layers, choices, cost in out["found"]:
            if at <= cut:
                plan = prob.plan(layers, s.top, choices)
                plan.cost = cost
                s.best = plan
        s.nodes, s.done = nodes, done
        s.last_seconds = out["seconds"]
        s.history.append((budget, legs, s.first_only))
