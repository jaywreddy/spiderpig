"""Long operations off the server's event loop: a process pool per store, jobs with ids.

``build``, ``verify`` at ``standard`` or ``full`` and ``export`` take seconds to a
minute. The installed MCP SDK (2.x) carries the wire types of task-augmented execution
but its server doesn't run tools as tasks, so a long tool submits a :class:`Job` here,
waits a grace period, and returns either the finished result or the job's record for
``get_job`` / ``wait_job`` to follow.

A worker process loads the design from the store, runs the operation (which writes
its report, parts or files into the store, exactly as the Python API would) and
returns the report's JSON; nothing but that JSON crosses the process boundary
(decision 3: solids stay in the Python API). Workers are spawned fresh, never forked
from the threaded server (each has its own ``fabricate._DESIGNS`` cache and imports
the engine once), and live as long as the pool. Job records live in the server
process only; what they produced is in the store.
"""

from __future__ import annotations

import concurrent.futures as cf
import contextlib
import logging
import multiprocessing as mp
import os
import secrets
import sys
import threading
import time
from concurrent.futures.process import BrokenProcessPool
from dataclasses import dataclass, field

from spiderpig.design import now_iso
from spiderpig.failure import Failure

log = logging.getLogger("spiderpig.mcp")

LONG_OPS = ("build", "verify", "export")


def run_op(root: str, op: str, design: str, args: dict) -> dict:
    """In a worker process: the operation's report as JSON (a ``build``: the store's
    manifest, each part with the ``path`` of its STEP file)."""
    from spiderpig import api
    from spiderpig.store import Store, report_doc

    store = Store(root)
    d = api.load(design, store)
    if op == "build":
        rep = api.build(d, float(args.get("t", 1.0)))
        doc = store.read_report(design, "build")
        if (doc is None or not rep.ok or doc.get("t") != rep.t or not doc.get("ok")
                or doc.get("engine_version") != d.engine_version):
            # the store's manifest only when it is this build's (its crank angle, this
            # engine, a success): a failed build (one that ran out of planner time isn't
            # stored at all) is its own report, with its failures
            doc = report_doc(rep)
        build_dir = store.dir(design) / "build"
        parts = []
        for e in doc.get("parts", []):
            e = dict(e)
            e["path"] = str(build_dir / e["file"]) if e.get("file") else None
            parts.append(e)
        doc["parts"] = parts
        doc["dir"] = str(build_dir)
        doc["files"] = len([e for e in parts if e.get("path")])
    elif op == "verify":
        doc = report_doc(api.verify(d, args["level"]))
    elif op == "export":
        doc = report_doc(api.export(d, args.get("formats"), args.get("out_dir")))
    else:
        raise ValueError(f"unknown long operation {op!r}; have {LONG_OPS}")
    doc["design"] = design
    return doc


def _init_worker() -> None:
    """A worker's stdout is stderr: the protocol stream belongs to the server alone."""
    with contextlib.suppress(OSError):
        os.dup2(2, 1)
    sys.stdout = sys.stderr


@dataclass
class Job:
    """One submitted operation and its outcome."""

    id: str
    op: str
    design: str
    args: dict
    future: cf.Future
    started_at: str = field(default_factory=now_iso)
    t0: float = field(default_factory=time.time)
    finished_at: str | None = None
    seconds: float | None = None

    def _finish(self, _future) -> None:
        self.finished_at = now_iso()
        self.seconds = round(time.time() - self.t0, 3)

    @property
    def state(self) -> str:
        f = self.future
        if not f.done():
            return "running" if f.running() else "queued"
        return "failed" if f.cancelled() or f.exception() is not None else "done"

    def error(self) -> Failure | None:
        """The failure of a job that didn't finish, else ``None``."""
        f = self.future
        if not f.done():
            return None
        if f.cancelled():
            return Failure("job", "cancelled", f"{self.op} of {self.design} was cancelled")
        exc = f.exception()
        if exc is None:
            return None
        if isinstance(exc, BrokenProcessPool):
            return Failure("job", "worker_died", str(exc) or "the worker process died")
        return Failure.from_exception(exc, stage="job")

    def to_dict(self, result: bool = True) -> dict:
        state = self.state
        if state in ("done", "failed") and self.finished_at is None:
            self._finish(self.future)       # a waiter can wake before the done callback runs
        out = {"job": self.id, "op": self.op, "design": self.design, "args": dict(self.args),
               "state": state, "started_at": self.started_at, "finished_at": self.finished_at,
               "seconds": self.seconds}
        if state == "done" and result:
            out["result"] = self.future.result()
        elif state == "failed":
            out["error"] = self.error().to_dict()
        return out


class Jobs:
    """The jobs of one store: a lazily started pool of spawned workers and the records
    of what ran (``get`` by id; ``wait`` blocks the calling thread, never the loop)."""

    def __init__(self, root: str, workers: int = 2):
        self.root = root
        self.workers = max(1, int(workers))
        self._pool: cf.ProcessPoolExecutor | None = None
        self._jobs: dict[str, Job] = {}
        self._lock = threading.Lock()

    def pool(self) -> cf.ProcessPoolExecutor:
        with self._lock:
            if self._pool is not None and getattr(self._pool, "_broken", False):
                log.warning("the job pool broke (%s): starting a new one", self._pool._broken)
                self._pool.shutdown(wait=False)
                self._pool = None
            if self._pool is None:
                self._pool = cf.ProcessPoolExecutor(
                    self.workers, mp_context=mp.get_context("spawn"), initializer=_init_worker)
            return self._pool

    def submit(self, op: str, design: str, args: dict | None = None) -> Job:
        if op not in LONG_OPS:
            raise ValueError(f"{op!r} is not a long operation; have {LONG_OPS}")
        args = dict(args or {})
        try:
            future = self.pool().submit(run_op, self.root, op, design, args)
        except BrokenProcessPool:
            with self._lock:
                self._pool = None
            future = self.pool().submit(run_op, self.root, op, design, args)
        job = Job(secrets.token_hex(6), op, design, args, future)
        future.add_done_callback(job._finish)
        with self._lock:
            self._jobs[job.id] = job
        return job

    def get(self, id: str) -> Job | None:
        with self._lock:
            return self._jobs.get(id)

    def wait(self, job: Job, seconds: float) -> bool:
        """Block up to ``seconds`` for ``job``; whether it has finished (either way)."""
        try:
            job.future.result(timeout=max(0.0, float(seconds)))
        except cf.TimeoutError:
            return False
        except (cf.CancelledError, Exception):  # noqa: BLE001 - the job's own failure
            return True
        return True

    def list(self) -> list[Job]:
        with self._lock:
            return list(self._jobs.values())

    def shutdown(self) -> None:
        with self._lock:
            pool, self._pool = self._pool, None
        if pool is not None:
            pool.shutdown(wait=False, cancel_futures=True)


__all__ = ["LONG_OPS", "Job", "Jobs", "run_op"]
