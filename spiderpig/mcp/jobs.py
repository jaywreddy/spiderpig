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

A worker holds the design's lock (:meth:`spiderpig.store.Store.lock`) for the whole
operation, so two jobs of one design (a build and an export, or two builds at other
crank angles) run one after the other instead of deleting each other's STEP files, and
``gc`` leaves a design in use alone. :meth:`Jobs.submit` hands back the job already
queued or running for the same ``(op, design, args)`` rather than starting a second.

Finished jobs are evicted past :data:`KEEP_FINISHED` or :data:`KEEP_SECONDS` after they
finished (their results can be large: a build's manifest); an evicted id keeps its op,
design and final state (:meth:`Jobs.evicted`), and its report stays in the store.
"""

from __future__ import annotations

import concurrent.futures as cf
import contextlib
import json
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
KEEP_FINISHED = 64          # finished jobs kept with their results, newest first
KEEP_SECONDS = 3600.0       # a finished job's result is kept at most this long
KEEP_EVICTED = 4096         # evicted ids remembered (op, design, state), newest first


def run_op(root: str, op: str, design: str, args: dict) -> dict:
    """In a worker process: the operation's report as JSON (a ``build``: the store's
    manifest, each part with the ``path`` of its STEP file)."""
    from spiderpig.store import Store

    store = Store(root)
    if not store.has(design):           # not recorded: load's own error, no lock file made
        return _run_op(store, op, design, args)
    with store.lock(design):            # the op's writes and the reads of what it wrote
        return _run_op(store, op, design, args)


def _run_op(store, op: str, design: str, args: dict) -> dict:
    from spiderpig import api
    from spiderpig.store import report_doc

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
    """A worker's stdout is stderr: the protocol stream belongs to the server alone. Each
    worker leads a process group of its own, so killing the group (:meth:`Jobs.kill`) also
    ends the ``python -c`` processes it started (:func:`spiderpig.workers.submit`)."""
    with contextlib.suppress(OSError):
        os.setsid()
    with contextlib.suppress(OSError):
        os.dup2(2, 1)
    sys.stdout = sys.stderr


def _kill_group(pid: int) -> None:
    """SIGKILL the process group ``pid`` leads (a worker and its children), else ``pid``."""
    import signal

    try:
        os.killpg(pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        with contextlib.suppress(OSError):
            os.kill(pid, signal.SIGKILL)


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
    done_at: float | None = None        # time.time() when it finished

    def _finish(self, _future) -> None:
        if self.done_at is not None:
            return
        self.done_at = time.time()
        self.finished_at = now_iso()
        self.seconds = round(self.done_at - self.t0, 3)

    @property
    def key(self) -> tuple[str, str, str]:
        return job_key(self.op, self.design, self.args)

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
            err = self.error()
            assert err is not None      # "failed": the future is done, cancelled or raised
            out["error"] = err.to_dict()
        return out


def job_key(op: str, design: str, args: dict) -> tuple[str, str, str]:
    """What makes two submissions the same job: the op, the design and the arguments."""
    return op, design, json.dumps(args, sort_keys=True, default=str)


class Jobs:
    """The jobs of one store: a lazily started pool of spawned workers and the records
    of what ran (``get`` by id; ``wait`` blocks the calling thread, never the loop)."""

    def __init__(self, root: str, workers: int = 2, keep: int = KEEP_FINISHED,
                 keep_seconds: float = KEEP_SECONDS):
        self.root = root
        self.workers = max(1, int(workers))
        self.keep = int(keep)
        self.keep_seconds = float(keep_seconds)
        self._pool: cf.ProcessPoolExecutor | None = None
        self._jobs: dict[str, Job] = {}
        self._evicted: dict[str, dict] = {}     # id -> {job, op, design, state, finished_at}
        self._lock = threading.Lock()
        self._submitting = threading.Lock()     # a lookup and its submit, as one step

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
        key = job_key(op, design, args)
        with self._submitting:
            with self._lock:
                self._prune()
                for job in self._jobs.values():
                    if job.key == key and not job.future.done():
                        return job          # the same operation is queued or running
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

    def _prune(self) -> None:
        """Evict finished jobs past :attr:`keep` (newest kept) or older than
        :attr:`keep_seconds`; ``self._lock`` held."""
        now = time.time()
        done = [j for j in self._jobs.values() if j.future.done()]
        for j in done:
            j._finish(j.future)
        done.sort(key=lambda j: j.done_at or 0.0, reverse=True)
        for i, j in enumerate(done):
            if i >= self.keep or now - (j.done_at or now) > self.keep_seconds:
                del self._jobs[j.id]
                self._evicted[j.id] = {"job": j.id, "op": j.op, "design": j.design,
                                       "state": j.state, "finished_at": j.finished_at}
        while len(self._evicted) > KEEP_EVICTED:
            self._evicted.pop(next(iter(self._evicted)))

    def get(self, id: str) -> Job | None:
        with self._lock:
            self._prune()
            return self._jobs.get(id)

    def evicted(self, id: str) -> dict | None:
        """An evicted job's record (``{job, op, design, state, finished_at}``), else
        ``None``."""
        with self._lock:
            return self._evicted.get(id)

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
            self._prune()
            return list(self._jobs.values())

    def shutdown(self, kill: bool = False) -> None:
        """Stop the pool (queued jobs cancelled); ``kill``: kill its workers' process groups
        too (:meth:`kill`)."""
        if kill:
            self.kill()
        with self._lock:
            pool, self._pool = self._pool, None
        if pool is not None:
            pool.shutdown(wait=False, cancel_futures=True)

    def kill(self) -> None:
        """SIGKILL every worker's process group (the worker and the processes it started).
        Takes no lock (a signal handler calls it, maybe while this thread holds
        ``_lock``): it reads the pool and its process table as they are."""
        pool = self._pool
        try:
            pids = [p.pid for p in list((getattr(pool, "_processes", None) or {}).values())]
        except RuntimeError:            # the table changed under the copy: once more
            pids = [p.pid for p in list((getattr(pool, "_processes", None) or {}).values())]
        for pid in pids:
            if pid:
                _kill_group(pid)


__all__ = ["KEEP_FINISHED", "KEEP_SECONDS", "LONG_OPS", "Job", "Jobs", "job_key", "run_op"]
