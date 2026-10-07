"""The store and the MCP's jobs under concurrency (W1 items 2, 3, 6 and 8): a design's lock
around the store's writers and ``gc``, one job per identical long operation, finished jobs
evicted with their ids still answering, one viewer child however many ``view`` calls race,
and design ids matched whole."""

from __future__ import annotations

import concurrent.futures as cf
import contextlib
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from spiderpig.store import Store

ID = "0123456789abcdef"


# ---------------------------------------------------------------------------
# A design's lock: write_build, gc
# ---------------------------------------------------------------------------


def _fake_design(store: Store, names=("a", "b", "c")):
    """A recorded design folder and the handle ``write_build`` reads (no engine)."""
    d = store.dir(ID)
    d.mkdir(parents=True)
    (d / "resolved.json").write_text("{}")
    parts = {n: SimpleNamespace(to_dict=lambda n=n: {"name": n}, pose=[[1.0]], built=n)
             for n in names}
    mech = SimpleNamespace(body=lambda _n: SimpleNamespace(color="#fff"), name="m",
                           meta={}, bom_extras=[])
    design = SimpleNamespace(id=ID, engine_version="e", mech=mech, parts=parts)
    rep = SimpleNamespace(ok=True, to_dict=lambda: {"ok": True, "t": 1.0})
    return design, rep


def _manifest_files(store: Store) -> list[Path]:
    import json

    doc = json.loads((store.dir(ID) / "build" / "manifest.json").read_text())
    return [store.dir(ID) / "build" / e["file"] for e in doc["parts"] if e["file"]]


def test_a_second_build_waits_for_the_first_instead_of_deleting_its_files(tmp_path,
                                                                           monkeypatch):
    """Build A writes its STEP files; build B of the same design starts meanwhile. Before
    the lock, B's ``rmtree`` deleted A's files while A was still writing, so the manifest A
    wrote named files that weren't there. The interleaving is forced (events): A pauses at
    its second file until B has cleared the folder (or 3 s: B waits on the lock), B pauses
    after its rmtree until A is done; the files A's manifest names are looked at as A
    writes it."""
    import build123d

    from spiderpig import store as store_module

    store = Store(tmp_path)
    design, rep = _fake_design(store)
    a_writing, b_cleared, a_done = threading.Event(), threading.Event(), threading.Event()
    owner: dict[int, str] = {}

    def export_step(_part, path):
        me = owner[threading.get_ident()]
        if me == "A" and Path(path).name.startswith(".b"):      # A, at its second file
            a_writing.set()
            b_cleared.wait(3.0)        # before the fix B's rmtree lands here; after, B waits
        if me == "B" and not b_cleared.is_set():                # B, past its rmtree
            b_cleared.set()
            a_done.wait(3.0)           # (A writes its manifest before B's first file)
        Path(path).write_text(me)

    seen: dict[str, list[bool]] = {}
    real_write = store_module._write_json

    def write_json(path, doc, *a, **kw):
        if Path(path).name == "manifest.json":      # what the manifest names, as written
            build = Path(path).parent
            seen[owner[threading.get_ident()]] = [
                (build / e["file"]).is_file() for e in doc["parts"] if e["file"]]
        return real_write(path, doc, *a, **kw)

    monkeypatch.setattr(build123d, "export_step", export_step)
    monkeypatch.setattr(store_module, "_write_json", write_json)
    errors: dict[str, BaseException] = {}

    def build(name: str) -> None:
        owner[threading.get_ident()] = name
        try:
            store.write_build(design, rep)
        except BaseException as e:      # noqa: BLE001 - the test reports it
            errors[name] = e
        finally:
            if name == "A":
                a_done.set()

    a = threading.Thread(target=build, args=("A",))
    a.start()
    assert a_writing.wait(5.0)
    b = threading.Thread(target=build, args=("B",))
    b.start()
    a.join(15.0)
    b.join(15.0)
    assert errors == {}
    assert seen["A"] == [True, True, True], "A's manifest names files B deleted"
    assert seen["B"] == [True, True, True]
    assert all(p.is_file() for p in _manifest_files(store))
    leftovers = [p.name for p in (store.dir(ID) / "build" / "parts").iterdir()
                 if p.name.startswith(".")]
    assert leftovers == []


def test_gc_leaves_a_design_being_written_alone(tmp_path, monkeypatch):
    """``gc`` while a build writes the design: before the lock it removed the folder under
    the writer (whose next file then failed); now it skips the design in use."""
    import build123d

    store = Store(tmp_path)
    design, rep = _fake_design(store)
    writing, gc_done = threading.Event(), threading.Event()
    errors: list[BaseException] = []

    def export_step(_part, path):
        writing.set()
        gc_done.wait(1.0)
        Path(path).write_text("x")

    monkeypatch.setattr(build123d, "export_step", export_step)

    def build() -> None:
        try:
            store.write_build(design, rep)
        except BaseException as e:      # noqa: BLE001 - the test reports it
            errors.append(e)

    t = threading.Thread(target=build)
    t.start()
    assert writing.wait(5.0)
    removed = store.gc(keep=[])
    gc_done.set()
    t.join(10.0)
    assert removed == []
    assert errors == []
    assert store.has(ID)
    assert all(p.is_file() for p in _manifest_files(store))
    assert store.gc(keep=[]) == [ID]                # not in use any more: removed
    assert not store.dir(ID).exists()


def test_the_lock_is_one_holder_across_processes(tmp_path):
    """The design's lock is an ``flock``: another process can't take it while this one
    holds it, and can once it is released."""
    import sys

    store = Store(tmp_path)
    store.dir(ID).mkdir(parents=True)
    probe = ("import fcntl, os, sys\n"
             f"fd = os.open({str(store.lock_path(ID))!r}, os.O_RDWR | os.O_CREAT)\n"
             "try:\n"
             "    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)\n"
             "except BlockingIOError:\n"
             "    sys.exit(3)\n")
    with store.lock(ID), store.lock(ID):            # re-entrant in this thread
        held = subprocess.run([sys.executable, "-c", probe], check=False)
    free = subprocess.run([sys.executable, "-c", probe], check=False)
    assert (held.returncode, free.returncode) == (3, 0)


def _hold_lock(root, id_: str, then: str = "") -> subprocess.Popen:
    """Another process holding design ``id_``'s lock file (an ``flock``, as a pool job's
    ``Store.lock`` does) until its stdin closes, then running ``then`` (``shutil``, ``d``:
    the design's folder) before it lets go; returns once it holds the lock."""
    import sys

    d = Store(root).dir(id_)
    code = ("import fcntl, os, shutil, sys\n"
            f"d = {str(d)!r}\n"
            "fd = os.open(os.path.join(d, '.lock'), os.O_RDWR | os.O_CREAT)\n"
            "fcntl.flock(fd, fcntl.LOCK_EX)\n"
            "print('held', flush=True)\n"
            "sys.stdin.read()\n"
            f"{then or 'pass'}\n")
    proc = subprocess.Popen([sys.executable, "-c", code], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, text=True)
    assert proc.stdout.readline().strip() == "held"
    return proc


def test_a_report_write_never_waits_on_a_build(tmp_path):
    """Review round 1: the MCP's in-process tools (``check``, ``plan``, ``walk``, ``view``)
    run under ``_ENGINE`` and write single reports; a pool job holds the design's lock for
    its whole build. Report writes take no lock (atomic files), so a tool neither waits on
    the build nor holds ``_ENGINE`` while it does."""
    from spiderpig import api
    from spiderpig.mcp import _ENGINE

    store = Store(tmp_path)
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store)
    holder = _hold_lock(tmp_path, d.id)
    try:
        done = threading.Event()

        def tool() -> None:
            with _ENGINE:
                api.check(d)
            done.set()

        t = threading.Thread(target=tool, daemon=True)
        t.start()
        assert done.wait(30.0), "check waited on the build's lock while holding _ENGINE"
        assert _ENGINE.acquire(timeout=5.0)
        _ENGINE.release()
        assert store.read_report(d.id, "check")["ok"]
    finally:
        holder.stdin.close()
        holder.wait(10)


def test_a_store_spelled_through_a_symlink_is_one_lock(tmp_path):
    real = tmp_path / "store"
    (real / "designs" / ID).mkdir(parents=True)
    link = tmp_path / "link"
    link.symlink_to(real)
    done = threading.Event()

    def nested() -> None:
        with Store(real).lock(ID), Store(link).lock(ID):    # one thread: re-entrant
            done.set()

    threading.Thread(target=nested, daemon=True).start()
    assert done.wait(5.0), "a nested lock through the symlink waited on itself"


def test_a_waiter_wakes_to_a_removed_design_and_doesnt_make_it_again(tmp_path):
    """A process holds the design's lock and removes the design (what ``gc`` does) while
    this one waits for it: the waiter must not proceed on the unlinked lock file (an
    inode re-check) nor make the design's folder again (an orphan ``designs/<id>/``)."""
    from spiderpig.store import DesignRemoved

    store = Store(tmp_path)
    store.dir(ID).mkdir(parents=True)
    (store.dir(ID) / "resolved.json").write_text("{}")
    holder = _hold_lock(tmp_path, ID, then="shutil.rmtree(d)")
    got: list[object] = []

    def waiter() -> None:
        try:
            with store.lock(ID):
                got.append("locked")
        except DesignRemoved as e:
            got.append(e)

    t = threading.Thread(target=waiter)
    t.start()
    time.sleep(0.5)                     # the waiter is in flock
    holder.stdin.close()                # the holder removes the design and lets go
    holder.wait(10)
    t.join(10)
    assert len(got) == 1
    assert isinstance(got[0], DesignRemoved), got
    assert not store.dir(ID).exists()
    assert store.list_designs() == []


def test_a_job_runs_under_the_designs_lock(tmp_path, monkeypatch):
    """``run_op`` holds the design's lock across the operation and its reads (a build and
    an export of one design, or a gc, wait for it)."""
    from spiderpig.mcp import jobs as jobs_module

    store = Store(tmp_path)
    store.dir(ID).mkdir(parents=True)
    (store.dir(ID) / "resolved.json").write_text("{}")
    ran = threading.Event()
    monkeypatch.setattr(jobs_module, "_run_op",
                        lambda *_a: ran.set() or {"ok": True})
    with store.lock(ID):
        t = threading.Thread(target=jobs_module.run_op,
                             args=(str(tmp_path), "build", ID, {}))
        t.start()
        assert not ran.wait(0.5), "the job ran while another held the design's lock"
    assert ran.wait(5.0)
    t.join(5)


def test_a_cache_name_is_matched_whole(tmp_path):
    from spiderpig import api

    st = Store(tmp_path)
    assert api._cache_path(st, "walk/abc").name == "abc.json"
    with pytest.raises(ValueError, match="not a cache name"):
        api._cache_path(st, "walk/abc\n")


@pytest.mark.slow
def test_sigterm_ends_the_mcp_server_and_its_viewer(tmp_path):
    """SIGTERM ends ``spiderpig mcp`` at once (the stdio reader thread used to keep it
    alive while its stdin stayed open) and stops the viewer's child process first."""
    import os
    import signal
    import subprocess
    import sys

    pidfile = tmp_path / "viewer.pid"
    code = f"""
import subprocess, sys
from spiderpig import view
from spiderpig import mcp

def start_background(store, **kw):
    proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
    open({str(pidfile)!r}, "w").write(str(proc.pid))
    return view.ViewServer("127.0.0.1", 1, str(store.root), proc)

view.viewer_built = lambda: "dist"
view.start_background = start_background
real = mcp.make_server

def make_server(*a, **kw):
    server = real(*a, **kw)
    server.spiderpig.view_server()
    print("up", file=sys.stderr, flush=True)
    return server

mcp.make_server = make_server
sys.exit(mcp.main(["--store", {str(tmp_path / "store")!r}]))
"""
    proc = subprocess.Popen([sys.executable, "-c", code], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert proc.stderr.readline().strip() == "up"
        time.sleep(1.0)                 # serving: the stdio reader is blocked on stdin
        child = int(pidfile.read_text())
        proc.send_signal(signal.SIGTERM)
        assert proc.wait(timeout=10) == 128 + signal.SIGTERM
    finally:
        if proc.poll() is None:
            proc.kill()
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        try:
            os.kill(child, 0)
        except ProcessLookupError:
            break
        with contextlib.suppress(ChildProcessError):
            os.waitpid(child, os.WNOHANG)
        time.sleep(0.1)
    else:
        os.kill(child, signal.SIGKILL)
        pytest.fail("the viewer's child outlived the server")


# ---------------------------------------------------------------------------
# Jobs: one per identical operation; finished ones evicted
# ---------------------------------------------------------------------------


class _Pool:
    """A pool whose futures the test finishes (no worker processes)."""

    def __init__(self) -> None:
        self.futures: list[cf.Future] = []

    def submit(self, _fn, *args):
        f = cf.Future()
        f.set_running_or_notify_cancel()
        f.args = args
        self.futures.append(f)
        return f


def _jobs(tmp_path, monkeypatch, **kw):
    from spiderpig.mcp.jobs import Jobs

    jobs = Jobs(str(tmp_path), **kw)
    pool = _Pool()
    monkeypatch.setattr(jobs, "pool", lambda: pool)
    return jobs, pool


def test_an_identical_long_operation_is_the_running_job(tmp_path, monkeypatch):
    jobs, pool = _jobs(tmp_path, monkeypatch)
    a = jobs.submit("build", ID, {"t": 1.0})
    b = jobs.submit("build", ID, {"t": 1.0})
    c = jobs.submit("build", ID, {"t": 2.0})            # other arguments: its own job
    assert b is a
    assert c is not a
    assert len(pool.futures) == 2
    pool.futures[0].set_result({"ok": True})
    d = jobs.submit("build", ID, {"t": 1.0})            # the first one finished: a new job
    assert d is not a
    assert len(pool.futures) == 3


def test_finished_jobs_are_evicted_and_their_ids_still_answer(tmp_path, monkeypatch):
    import asyncio

    from mcp import Client

    from spiderpig.mcp import make_server

    jobs, pool = _jobs(tmp_path, monkeypatch, keep=2)
    made = [jobs.submit("export", ID, {"n": i}) for i in range(5)]
    for f in pool.futures:
        f.set_result({"ok": True, "big": "x" * 1000})
    running = jobs.submit("verify", ID, {"level": "full"})
    assert [jobs.get(j.id) is not None for j in made] == [False, False, False, True, True]
    assert jobs.get(running.id) is running              # running jobs are never evicted
    gone = jobs.evicted(made[0].id)
    assert gone == {"job": made[0].id, "op": "export", "design": ID, "state": "done",
                    "finished_at": made[0].finished_at}
    old, _ = _jobs(tmp_path, monkeypatch, keep_seconds=0.0)
    job = old.submit("build", ID, {})
    job.future.set_result({"ok": True})
    time.sleep(0.01)
    assert old.get(job.id) is None
    assert old.evicted(job.id)["state"] == "done"

    server = make_server(tmp_path)
    server.spiderpig.jobs = jobs

    async def get_job(job_id):
        async with Client(server) as client:
            return await client.call_tool("get_job", {"job": job_id})

    result = asyncio.run(get_job(made[0].id))
    assert result.is_error
    (failure,) = result.structured_content["failures"]
    assert failure["code"] == "job_evicted"
    assert "get_design" in failure["notes"][0]
    result = asyncio.run(get_job("feedfeedfeed"))
    assert result.structured_content["failures"][0]["code"] == "no_such_job"


_RACE_HOOK = """
import importlib.machinery, json, os, pathlib, sys, time

D = os.environ.get("W1_RACE_DIR")


def _install(_st):
    import build123d

    _export, _write, _me, _n = build123d.export_step, _st._write_json, [], [0]

    def _role():
        if not _me:
            try:
                os.close(os.open(os.path.join(D, "A"), os.O_CREAT | os.O_EXCL))
                _me.append("A")
            except FileExistsError:
                _me.append("B")
        return _me[0]

    def _flag(name):
        open(os.path.join(D, name), "w").close()

    def _wait(name, seconds):
        end = time.time() + seconds
        while time.time() < end and not os.path.exists(os.path.join(D, name)):
            time.sleep(0.05)

    _write_build = _st.Store.write_build

    def write_build(self, design, rep):
        if _role() == "B":              # B starts writing once A is at its second file
            _wait("a_writing", 30)
        return _write_build(self, design, rep)

    def _b_blocked(parts_dir):
        # is a process waiting on this design's lock? (an flock waiter in /proc/locks)
        try:
            ino = os.stat(os.path.join(os.path.dirname(os.path.dirname(parts_dir)),
                                       ".lock")).st_ino
            locks = open("/proc/locks").read().splitlines()
        except OSError:
            return False                # no lock file: nobody locks (the code before)
        return any("->" in ln and ln.split()[-3].endswith(f":{ino}") for ln in locks)

    def export_step(part, path, *a, **kw):
        _n[0] += 1
        if _role() == "A" and _n[0] == 2:
            # B's rmtree lands here, unless B waits on the design's lock
            _flag("a_writing")
            end = time.time() + 120
            while time.time() < end and not os.path.exists(os.path.join(D, "b_cleared")):
                if _b_blocked(os.path.dirname(path)):
                    _flag("b_blocked")
                    break
                time.sleep(0.05)
        if _role() == "B" and _n[0] == 1:
            _flag("b_cleared")
            _wait("a_done", 30)         # A writes its manifest before B's first file
        return _export(part, path, *a, **kw)

    def write_json(path, doc, *a, **kw):
        out = _write(path, doc, *a, **kw)
        if pathlib.Path(path).name == "manifest.json" and _role() == "A":
            b = pathlib.Path(path).parent
            gone = [e["file"] for e in doc.get("parts", [])
                    if e.get("file") and not (b / e["file"]).is_file()]
            with open(os.path.join(D, "A_missing.json"), "w") as f:
                json.dump(gone, f)
            _flag("a_done")
        return out

    build123d.export_step = export_step
    _st._write_json = write_json
    _st.Store.write_build = write_build


class _Hook:
    # patch spiderpig.store once it is imported: from the worker's own sys.path (the
    # checkout under test), set only after this file runs
    def find_spec(self, name, path, target=None):
        if name != "spiderpig.store":
            return None
        spec = importlib.machinery.PathFinder.find_spec(name, path)
        if spec is None:
            return None
        run = spec.loader.exec_module

        def exec_module(module):
            run(module)
            _install(module)

        spec.loader.exec_module = exec_module
        return spec


if D:
    sys.meta_path.insert(0, _Hook())
"""


def test_finished_jobs_dont_pile_up(tmp_path, monkeypatch):
    """Every finished job used to stay, its result with it (a build's whole manifest):
    after many jobs the server holds at most :data:`KEEP_FINISHED` of them."""
    from spiderpig.mcp import jobs as jobs_module

    jobs = jobs_module.Jobs(str(tmp_path))
    pool = _Pool()
    monkeypatch.setattr(jobs, "pool", lambda: pool)
    for i in range(200):
        jobs.submit("export", ID, {"n": i})
        pool.futures[-1].set_result({"ok": True, "big": "x" * 10_000})
    held = jobs.list()
    assert len(held) <= 64                              # the roadmap's cap
    assert jobs.get(held[-1].id) is held[-1]                # the newest stay


@pytest.mark.slow
def test_two_concurrent_build_jobs_leave_one_consistent_build(tmp_path, monkeypatch):
    """Two builds of one design at once through the real pool (two spawned workers),
    their interleaving forced by a ``sitecustomize`` in the workers (files as events): the
    first to reach ``write_build`` is A; B starts writing once A is at its second STEP file,
    where A pauses until B has cleared the parts folder or is seen waiting on the design's
    lock (an flock waiter on its ``.lock`` in ``/proc/locks``), and B pauses after its
    rmtree until A has written its manifest.
    Before the lock, A's manifest named a file B had deleted; now both finish, A's manifest
    names only files that are there, and the store holds one complete build (every file
    its manifest names, no other, no temporary file)."""
    import json
    import os

    from spiderpig.config import BuildConfig
    from spiderpig.mcp.jobs import Jobs
    from tests import cache

    store = cache.prebuilt_store(BuildConfig(linkage="hoecken", robot=False), tmp_path)
    (design,) = store.ids()
    hooks, race = tmp_path / "hooks", tmp_path / "race"
    hooks.mkdir()
    race.mkdir()
    (hooks / "sitecustomize.py").write_text(_RACE_HOOK)
    monkeypatch.setenv("W1_RACE_DIR", str(race))
    monkeypatch.setenv("PYTHONPATH", os.pathsep.join(
        [str(hooks), *filter(None, [os.environ.get("PYTHONPATH")])]))
    jobs = Jobs(str(store.root), workers=2)
    try:
        a = jobs.submit("build", design, {"t": 2.0})
        b = jobs.submit("build", design, {"t": 3.0})
        assert a is not b
        for j in (a, b):
            assert jobs.wait(j, 600)
            doc = j.to_dict()
            assert doc["state"] == "done", doc
            assert doc["result"]["ok"]
    finally:
        jobs.shutdown()
    assert (race / "b_cleared").exists()             # the hook ran in both workers
    assert json.loads((race / "A_missing.json").read_text()) == [], \
        "A's manifest names files B deleted"
    assert (race / "b_blocked").exists()             # B waited on the lock while A wrote
    build = store.dir(design) / "build"
    manifest = json.loads((build / "manifest.json").read_text())
    assert manifest["t"] in (2.0, 3.0)
    named = {e["file"] for e in manifest["parts"] if e["file"]}
    on_disk = {f"parts/{p.name}" for p in (build / "parts").iterdir()}
    assert named == on_disk


# ---------------------------------------------------------------------------
# view: one child process
# ---------------------------------------------------------------------------


def test_two_view_calls_at_once_start_one_viewer(tmp_path, monkeypatch):
    import atexit

    from spiderpig import view
    from spiderpig.mcp import make_server

    started: list[object] = []
    registered: list[object] = []

    class Child:
        def alive(self) -> bool:
            return True

        def stop(self) -> None:
            started.remove(self)

    def start_background(_store):
        time.sleep(0.3)                     # a child takes a while to come up
        child = Child()
        started.append(child)
        return child

    monkeypatch.setattr(view, "viewer_built", lambda: tmp_path)
    monkeypatch.setattr(view, "start_background", start_background)
    monkeypatch.setattr(atexit, "register", registered.append)
    state = make_server(tmp_path).spiderpig
    with cf.ThreadPoolExecutor(2) as pool:
        got = list(pool.map(lambda _: state.view_server(), range(2)))
    assert len(started) == 1
    assert got[0] is got[1]
    assert registered == [state.stop_viewer]        # stopped when the server process exits
    state.stop_viewer()
    assert started == []


# ---------------------------------------------------------------------------
# Ids
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("bad", [ID + "\n", "0123456789ABCDEF", ID + "0", ID[:-1], "../" + ID])
def test_a_design_id_is_matched_whole(tmp_path, bad):
    store = Store(tmp_path)
    assert store.dir(ID) == tmp_path / "designs" / ID
    with pytest.raises(ValueError, match="not a design id"):
        store.dir(bad)


@pytest.mark.parametrize("op", ["build", "export"])
def test_builds_and_exports_run_under_the_designs_lock(tmp_path, monkeypatch, op):
    """``api.build`` and ``api.export`` (``spiderpig export``, ``spiderpig view``'s glb, a
    pool job) write many files: they hold the design's lock, so a ``gc`` or a job of the
    same design waits for them, and they for it."""
    from spiderpig import api

    store = Store(tmp_path)
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store)
    ran = threading.Event()
    monkeypatch.setattr(api, f"_{op}", lambda *_a, **_k: ran.set())
    holder = _hold_lock(tmp_path, d.id)
    try:
        t = threading.Thread(target=getattr(api, op), args=(d,), daemon=True)
        t.start()
        assert not ran.wait(0.5), f"{op} ran while another process held the design's lock"
    finally:
        holder.stdin.close()
        holder.wait(10)
    assert ran.wait(5.0)
    t.join(5)
