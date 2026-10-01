"""Engine work in a fresh Python process, beside this one.

OCP holds the GIL through every OCCT call (measured: four threads meshing or cutting run
no faster than one), so what runs beside a fabrication, an export's other formats or a
verify's other checks has to be another process. Forking one after OCCT's thread pool has
run deadlocks it (measured), and :mod:`multiprocessing`'s spawn re-imports the caller's
``__main__`` (a script without the ``if __name__ == "__main__"`` guard then starts its
work again in the child, which refuses). :func:`submit` instead runs a module-level
function of the package in ``python -c`` with this process's ``sys.path``: its arguments
and its return value cross as pickles in a temporary folder; its exception is raised
again here; its stdout and stderr are this process's.

A worker imports the engine itself (3-4 s), so it pays off for work that takes longer
than that and needs nothing but what the store holds (a design is loaded by its id).
"""

from __future__ import annotations

import concurrent.futures as cf
import logging
import os
import pickle
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

log = logging.getLogger("spiderpig.workers")

ENV_OFF = "SPIDERPIG_WORKERS"      # "0": every job runs in the calling process instead

_LOCK = threading.Lock()
_THREADS: cf.ThreadPoolExecutor | None = None

_CHILD = ("import pickle, sys; job = pickle.load(open(sys.argv[1], 'rb')); "
          "sys.path[:0] = job[0]; from spiderpig.workers import _child; _child(job, sys.argv[2])")


def enabled() -> bool:
    """Whether :func:`submit` starts processes (``SPIDERPIG_WORKERS=0`` turns them off)."""
    return os.environ.get(ENV_OFF) != "0"


def submit(fn, *args) -> cf.Future:
    """Run ``fn(*args)`` in a fresh Python process (``fn`` a module-level function, the
    arguments and the result picklable); the future of its result."""
    global _THREADS
    job = (list(sys.path), fn.__module__, fn.__qualname__, args)
    with _LOCK:
        if _THREADS is None:
            _THREADS = cf.ThreadPoolExecutor(8, thread_name_prefix="spiderpig-worker")
        return _THREADS.submit(_run, job)


def _run(job) -> object:
    with tempfile.TemporaryDirectory(prefix="spiderpig-worker-") as tmp:
        src, dst = Path(tmp) / "job.pickle", Path(tmp) / "result.pickle"
        src.write_bytes(pickle.dumps(job))
        proc = subprocess.run([sys.executable, "-c", _CHILD, str(src), str(dst)], check=False)
        if not dst.is_file():
            raise RuntimeError(f"worker for {job[1]}.{job[2]} exited with {proc.returncode} "
                               "and no result")
        ok, value, stats = pickle.loads(dst.read_bytes())
    log.debug("worker %s.%s: %.1f s, %.1f CPU-s (the engine's import %.1f s)", job[1], job[2],
              *stats)
    if not ok:
        raise value
    return value


def dump_shape(shape) -> tuple[str, bytes]:
    """A build123d shape as bytes (OCCT's binary BRep: every double as it is, the
    location included) and its class name, for :func:`load_shape` in a worker. Through a
    file: OCP's stream overload fails to read some parts back ("UnExpected
    BRep_PointRepresentation", 13 of round 4's 161, measured)."""
    from OCP.BinTools import BinTools

    with tempfile.TemporaryDirectory(prefix="spiderpig-brep-") as tmp:
        path = os.path.join(tmp, "shape.brep")
        BinTools.Write_s(shape.wrapped, path)
        return type(shape).__name__, Path(path).read_bytes()


def load_shape(dumped: tuple[str, bytes]):
    """The build123d shape :func:`dump_shape` wrote (the same class)."""
    import build123d
    from build123d.topology import downcast
    from OCP.BinTools import BinTools
    from OCP.TopoDS import TopoDS_Shape

    cls, data = dumped
    with tempfile.TemporaryDirectory(prefix="spiderpig-brep-") as tmp:
        path = os.path.join(tmp, "shape.brep")
        Path(path).write_bytes(data)
        shape = TopoDS_Shape()
        BinTools.Read_s(shape, path)
    return getattr(build123d, cls)(downcast(shape))


def _child(job, out: str) -> None:
    import importlib
    import resource
    import time

    _, module, name, args = job
    t0 = time.perf_counter()
    try:
        fn = getattr(importlib.import_module(module), name)
        t_import = time.perf_counter() - t0
        result = (True, fn(*args))
    except BaseException as e:      # noqa: BLE001 - every failure goes back to the caller
        t_import = time.perf_counter() - t0
        result = (False, e)
    r = resource.getrusage(resource.RUSAGE_SELF)
    stats = (time.perf_counter() - t0, r.ru_utime + r.ru_stime, t_import)
    try:
        data = pickle.dumps((*result, stats))
    except Exception as e:          # an unpicklable result or exception
        data = pickle.dumps((False, RuntimeError(f"{module}.{name}: {result[1]!r} ({e})"),
                             stats))
    Path(out).write_bytes(data)
