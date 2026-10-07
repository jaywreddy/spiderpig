"""``spiderpig build`` into a folder that already holds its outputs does nothing
(:mod:`spiderpig.uptodate`): only when the manifest, the key, the stored plan, the servo
model and every output agree; a partial, older, edited or foreign folder builds."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from spiderpig import store as store_mod
from spiderpig import uptodate

REPO = Path(__file__).resolve().parents[1]


def test_its_store_defaults_are_the_stores():
    assert (uptodate.STORE_ENV, uptodate.DEFAULT_ROOT) == (store_mod.STORE_ENV,
                                                           store_mod.DEFAULT_ROOT)


def test_preparse(tmp_path, monkeypatch):
    monkeypatch.delenv("SPIDERPIG_STORE", raising=False)
    o = uptodate.preparse(["--linkage", "klann", "--out", "a", "--store=s", "--force"])
    assert (o.out, o.store, o.force, o.argv) == (Path("a"), Path("s").resolve(), True,
                                                 ["--linkage", "klann"])
    o = uptodate.preparse(["--profile", "--profile-json", "p.json", "--profile-json=q",
                           "--out=b"])
    assert o.argv == []                     # (the profiler's options shape no output)
    o = uptodate.preparse(["--out=b"])
    assert o.out == Path("b")
    assert o.store == Path(".spiderpig").resolve()
    assert not o.force
    monkeypatch.setenv("SPIDERPIG_STORE", str(tmp_path))
    assert uptodate.preparse([]).store == tmp_path.resolve()
    for argv in (["--list"], ["-h"], ["--o", "x"], ["--for"], ["--st", "x"], ["--out"]):
        assert uptodate.preparse(argv) is None, argv


def test_the_check_imports_nothing_of_the_engine(tmp_path):
    """``spiderpig build`` asks before importing the engine (seconds by itself)."""
    code = ("import sys; from spiderpig import uptodate; "
            f"o = uptodate.preparse(['--out', {str(tmp_path)!r}]); uptodate.check(o); "
            "uptodate.build_key(o); "
            "print([m for m in ('build123d', 'OCP', 'numpy', 'sympy', 'spiderpig.config')"
            " if m in sys.modules])")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         check=True, cwd=REPO).stdout
    assert out.strip() == "[]"


PLAN_ID = "0123456789abcdef"


def _folder(tmp_path: Path, argv=("--linkage", "klann")):
    """A build's folder as :func:`uptodate.record` leaves it, by hand (no engine)."""
    out, store = tmp_path / "out", tmp_path / "store"
    for rel, text in (("laser/parts/order.csv", "a"), ("laser/k_sheet_0.dxf", "dxf"),
                      ("print/p.stl", "solid"), ("print/parts.csv", "p"), ("k.step", "S"),
                      ("k.stl", "T"), ("bom.json", "{}"), ("ORDER.md", "o"),
                      ("notes.txt", "the user's")):
        (out / rel).parent.mkdir(parents=True, exist_ok=True)
        (out / rel).write_text(text)
    plan = uptodate.plan_file(store, PLAN_ID)
    plan.parent.mkdir(parents=True)
    plan.write_text(json.dumps({"layers": {"b1": 1}, "written_at": "now", "seconds": 1.0}))
    opts = uptodate.preparse([*argv, "--out", str(out), "--store", str(store)])
    (out / "manifest.json").write_text(json.dumps({"design": "d",
                                                   "written_by": "spiderpig build"}))
    record = {
        "out": str(out.resolve()), "build_key": uptodate.build_key(opts),
        "plan_design": PLAN_ID, "plan_hash": uptodate.plan_hash(plan),
        "cad_env": uptodate.cad_env(), "servo_files": [],
        "outputs": {str(f.relative_to(out)): uptodate.stamp(f)
                    for f in uptodate.outputs(out)},
        "manifest": uptodate.stamp(out / "manifest.json"),
    }
    _write_record(opts, record)
    return opts, out, store


def _write_record(opts, record: dict) -> None:
    path = uptodate.record_path(opts)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record))


def test_a_current_folder_is_current(tmp_path):
    opts, out, _ = _folder(tmp_path)
    assert "8 files unchanged" in uptodate.check(opts)
    assert not uptodate.skip([*opts.argv, "--out", str(out), "--store", str(opts.store),
                              "--force"])


@pytest.mark.parametrize("change", [
    "edited output", "stray dxf", "missing output", "plan re-made", "plan removed",
    "other options", "other store", "servo models switched", "an export's manifest",
    "manifest rewritten", "no record", "no manifest", "folder moved"])
def test_anything_else_builds(tmp_path, monkeypatch, change):
    opts, out, store = _folder(tmp_path)
    argv = [*opts.argv, "--out", str(out), "--store", str(store)]
    if change == "edited output":
        (out / "k.stl").write_text("U")                     # same size, other bytes
    elif change == "stray dxf":
        (out / "laser" / "old_x9.dxf").write_text("")
    elif change == "missing output":
        (out / "print" / "p.stl").unlink()
    elif change == "plan re-made":
        uptodate.plan_file(store, PLAN_ID).write_text(json.dumps({"layers": {"b1": 2}}))
    elif change == "plan removed":
        uptodate.plan_file(store, PLAN_ID).unlink()
    elif change == "other options":
        argv = ["--linkage", "jansen", *argv[2:]]
    elif change == "other store":
        argv[-1] = str(tmp_path / "another")
    elif change == "servo models switched":
        monkeypatch.setenv("SPIDERPIG_SERVO_CAD", "0")
    elif change == "an export's manifest":
        doc = json.loads((out / "manifest.json").read_text())
        doc["written_by"] = "api.export"
        (out / "manifest.json").write_text(json.dumps(doc))
    elif change == "manifest rewritten":
        (out / "manifest.json").write_text(json.dumps({"design": "e",
                                                       "written_by": "spiderpig build"}))
    elif change == "no record":
        uptodate.forget(opts)
    elif change == "no manifest":
        (out / "manifest.json").unlink()
    elif change == "folder moved":
        out.rename(tmp_path / "moved")
        argv[argv.index(str(out))] = str(tmp_path / "moved")
    assert uptodate.check(uptodate.preparse(argv)) is None
    assert not uptodate.skip(argv)


def test_a_touched_output_with_its_bytes_is_still_current(tmp_path):
    """A file whose stat changed (a copy, a touch) is hashed: the same bytes are current;
    one whose stat didn't is not read at all."""
    opts, out, _ = _folder(tmp_path)
    os.utime(out / "k.stl", ns=(1, 1))
    assert uptodate.check(opts) is not None


def test_the_plans_timestamps_dont_count(tmp_path):
    opts, _, store = _folder(tmp_path)
    plan = uptodate.plan_file(store, PLAN_ID)
    plan.write_text(json.dumps({"layers": {"b1": 1}, "written_at": "later", "seconds": 9.0}))
    assert uptodate.check(opts) is not None


def test_a_servo_model_that_changed_or_could_be_downloaded_builds(tmp_path, monkeypatch):
    model = tmp_path / "cad" / "m.step"
    model.parent.mkdir()
    model.write_text("model")
    st = model.stat()
    missing = tmp_path / "cad" / "other.step"
    opts, out, _ = _folder(tmp_path)
    doc = json.loads(uptodate.record_path(opts).read_text())
    doc["servo_files"] = [[str(model), True, st.st_size, st.st_mtime_ns],
                          [str(missing), False, None, None]]
    doc["cad_env"] = {**uptodate.cad_env(), "SPIDERPIG_OFFLINE": "1"}
    _write_record(opts, doc)
    monkeypatch.setenv("SPIDERPIG_OFFLINE", "1")
    assert uptodate.check(opts) is not None                 # offline: nothing to fetch
    missing.write_text("downloaded since")
    assert uptodate.check(opts) is None
    missing.unlink()
    os.utime(model, ns=(st.st_atime_ns, st.st_mtime_ns + 10**9))
    assert uptodate.check(opts) is None


@pytest.mark.slow
def test_build_twice_does_nothing_the_second_time(tmp_path):
    """The real thing, through the command line (hoecken: seconds): the second build says
    it is up to date without importing the engine, rewrites nothing; ``--force`` builds;
    another folder is served from the fabrication cache and writes the same BOM."""
    env = {**os.environ, "SPIDERPIG_STORE": str(tmp_path / "store"),
           "SPIDERPIG_FAB_CACHE": "on"}
    out = tmp_path / "out"
    argv = ["--linkage", "hoecken_pantograph", "--out", str(out)]

    def run(*args):
        code = ("import sys; from spiderpig import cli; rc = cli.main(sys.argv[1:]); "
                "print('ENGINE', 'build123d' in sys.modules); sys.exit(rc)")
        p = subprocess.run([sys.executable, "-c", code, "build", *args], env=env, cwd=REPO,
                           capture_output=True, text=True)
        assert p.returncode == 0, p.stderr[-2000:]
        return p.stdout

    first = run(*argv)
    assert "up to date" not in first
    assert "ENGINE True" in first
    stamps = {f: f.stat().st_mtime_ns for f in out.rglob("*") if f.is_file()}
    second = run(*argv)
    assert "is up to date" in second
    assert "ENGINE False" in second
    assert {f: f.stat().st_mtime_ns for f in out.rglob("*") if f.is_file()} == stamps
    forced = run(*argv, "--force")
    assert "up to date" not in forced
    entries = list((tmp_path / "store" / "fab").glob("*/*_t1.0_*"))
    assert len([e for e in entries if e.is_dir()]) == 1         # one fabrication, kept
    other = tmp_path / "other"
    run("--linkage", "hoecken_pantograph", "--out", str(other))
    assert (other / "bom.json").read_text() == (out / "bom.json").read_text()
    assert len([e for e in (tmp_path / "store" / "fab").glob("*/*_t1.0_*")
                if e.is_dir()]) == 1


def test_a_strip_record_never_derived_is_no_download(tmp_path, monkeypatch):
    """A second pinned model's strip record is never written when the first one loads:
    missing, it is no reason to build (online too); appearing, it is."""
    monkeypatch.delenv("SPIDERPIG_OFFLINE", raising=False)
    opts, _, _ = _folder(tmp_path)
    rec = tmp_path / "cad" / "x.json"
    doc = json.loads(uptodate.record_path(opts).read_text())
    doc["servo_files"] = [[str(rec), False, None, None, "record"]]
    _write_record(opts, doc)
    assert uptodate.check(opts) is not None
    rec.parent.mkdir()
    rec.write_text("{}")
    assert uptodate.check(opts) is None
