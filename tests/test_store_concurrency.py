"""The store and the MCP's jobs under concurrency (W1 items 2, 3, 6 and 8): a design's lock
around the store's writers and ``gc``, one job per identical long operation, finished jobs
evicted with their ids still answering, one viewer child however many ``view`` calls race,
and design ids matched whole."""

from __future__ import annotations

import concurrent.futures as cf
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
    returned named files that weren't there (what ``mcp.jobs.run_op`` hands the client)."""
    import build123d

    store = Store(tmp_path)
    design, rep = _fake_design(store)
    a_writing, b_cleared, a_done = threading.Event(), threading.Event(), threading.Event()
    owner: dict[int, str] = {}

    def export_step(_part, path):
        me = owner[threading.get_ident()]
        if me == "A" and Path(path).name.startswith(".b"):      # A, at its second file
            a_writing.set()
            b_cleared.wait(1.0)        # before the fix B's rmtree lands here; after, B waits
        if me == "B" and not b_cleared.is_set():                # B, past its rmtree
            b_cleared.set()
            a_done.wait(1.0)           # (A finishes before B writes its first file)
        Path(path).write_text(me)

    monkeypatch.setattr(build123d, "export_step", export_step)
    seen: dict[str, list[bool]] = {}
    errors: dict[str, BaseException] = {}

    def build(name: str) -> None:
        owner[threading.get_ident()] = name
        try:
            store.write_build(design, rep)
            seen[name] = [p.is_file() for p in _manifest_files(store)]
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
    a.join(10.0)
    b.join(10.0)
    assert errors == {}
    assert seen["A"] == [True, True, True], "A's manifest names files B deleted"
    assert seen["B"] == [True, True, True]
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
    import subprocess
    import sys

    store = Store(tmp_path)
    probe = ("import sys; from spiderpig.store import Store\n"
             f"with Store({str(tmp_path)!r}).lock({ID!r}, blocking=False) as got:\n"
             "    sys.exit(0 if got else 3)\n")
    with store.lock(ID), store.lock(ID):            # re-entrant in this thread
        held = subprocess.run([sys.executable, "-c", probe], check=False)
    free = subprocess.run([sys.executable, "-c", probe], check=False)
    assert (held.returncode, free.returncode) == (3, 0)


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


@pytest.mark.slow
def test_two_concurrent_build_jobs_leave_one_consistent_build(tmp_path):
    """Two builds of one design at once, through the real pool (two spawned workers): both
    finish, and the store holds one complete build: its manifest's every file there, no
    file it doesn't name, no temporary file."""
    import json

    from spiderpig.config import BuildConfig
    from spiderpig.mcp.jobs import Jobs
    from tests import cache

    store = cache.prebuilt_store(BuildConfig(linkage="hoecken", robot=False), tmp_path)
    (design,) = store.ids()
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
            assert all(Path(p["path"]).is_file() for p in doc["result"]["parts"] if p["path"])
    finally:
        jobs.shutdown()
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
