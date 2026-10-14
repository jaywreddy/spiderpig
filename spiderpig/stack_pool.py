"""The layer planner's jobs in forked worker processes (:mod:`stack.drive`).

A job (:class:`stack.drive.Job`: one stack size, one head placement, its node budget and the
plan it must beat) depends on nothing but itself, so where and when it runs can't change its
answer: :class:`Pool` runs jobs in up to ``n`` workers, each forked from the planner once its
problems are built (the claims, the router and the geometry come over as they are; nothing
is pickled but jobs and answers), each running job after job with a CPU deadline of its own
(``StackSpec.max_seconds`` of the worker's own CPU time). The driver asks for the jobs it
needs (:meth:`Pool.result`, first in line) and lets idle workers run the ones it is likely to
need next (:meth:`Pool.submit`); a job asked for twice runs once.

Linux (``fork``) only; elsewhere, and with ``SPIDERPIG_WORKERS=0``, the jobs run in the
planner's own process (:class:`stack.drive.Here`), with the same answers.
"""

from __future__ import annotations

import contextlib
import multiprocessing
import traceback
from multiprocessing.connection import Connection, wait
from multiprocessing.process import BaseProcess
from typing import TYPE_CHECKING, cast

from spiderpig.stack.drive import Here, Job, run_job
from spiderpig.stack.plan import Deadline

if TYPE_CHECKING:
    from collections.abc import Mapping

    from spiderpig.stack.search import StackProblem


def _worker(subs: Mapping[str, StackProblem], conn: Connection, seconds: float) -> None:
    """A forked worker: job after job, each with ``seconds`` of this thread's CPU time, until
    told to stop (``None``)."""
    try:
        while True:
            job = conn.recv()
            if job is None:
                break
            conn.send((job, run_job(subs, job, Deadline(seconds))))
    except BaseException:  # noqa: BLE001 - the coordinator raises it (the traceback, below)
        with contextlib.suppress(Exception):
            conn.send((None, traceback.format_exc()))
    finally:
        conn.close()


class Pool(Here):
    """Up to ``n`` forked workers running :class:`stack.drive.Job` s (the module's docstring)."""

    def __init__(self, subs: Mapping[str, StackProblem], n: int, seconds: float):
        super().__init__(subs, seconds)
        self.n, self.seconds = n, seconds
        self.ctx = multiprocessing.get_context("fork")
        self.procs: list[tuple[BaseProcess, Connection]] = []
        self.busy: dict[Connection, Job] = {}
        self.queue: list[Job] = []
        self.sent: set[Job] = set()

    def _spawn(self) -> Connection:
        a, b = self.ctx.Pipe()
        p = self.ctx.Process(target=_worker, args=(self.subs, b, self.seconds), daemon=True)
        p.start()
        b.close()
        self.procs.append((p, a))
        return a

    def _idle(self) -> Connection | None:
        for _, conn in self.procs:
            if conn not in self.busy:
                return conn
        return self._spawn() if len(self.procs) < self.n else None

    def _dispatch(self) -> None:
        while self.queue:
            conn = self._idle()
            if conn is None:
                return
            job = self.queue.pop(0)
            conn.send(job)
            self.busy[conn] = job
            self.sent.add(job)

    def submit(self, job: Job, front: bool = False) -> None:
        """Run ``job`` when a worker is free (``front``: before the others waiting)."""
        if job in self.results or job in self.sent:
            return
        if job in self.queue:
            if not front:
                return
            self.queue.remove(job)
        self.queue.insert(0, job) if front else self.queue.append(job)
        self._dispatch()

    def free(self) -> int:
        """Workers that would sit idle with what is waiting now."""
        return self.n - len(self.busy) - len(self.queue)

    def _pump(self) -> None:
        """Take the answer of whichever worker answers first (blocking until one does)."""
        for c in cast("list[Connection]", wait(list(self.busy))):
            job, out = c.recv()
            if job is None:
                raise RuntimeError(f"a planner worker failed:\n{out}")
            del self.busy[c]
            self.results[job] = out
        self._dispatch()

    def result(self, job: Job) -> dict:
        self.submit(job, front=True)
        while job not in self.results:
            self._pump()
        return self.results[job]

    def wait_any(self, jobs: list[Job]) -> None:
        for job in reversed(jobs):
            self.submit(job, front=True)
        while not any(j in self.results for j in jobs):
            self._pump()

    def close(self) -> None:
        """Stop every worker: an idle one when told, a busy one (a job no one needs now)
        at once."""
        for p, conn in self.procs:
            if conn in self.busy:
                p.terminate()
            else:
                with contextlib.suppress(OSError):
                    conn.send(None)
        for p, conn in self.procs:
            p.join(timeout=2.0)
            if p.is_alive():
                p.terminate()
                p.join()
            conn.close()
        self.procs.clear()
        self.busy.clear()
