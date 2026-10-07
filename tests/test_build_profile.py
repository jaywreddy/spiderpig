"""The stage profiler (``spiderpig/tools/profiler.py``): the bake's summary unchanged, the
build's ``--profile`` stages (``spiderpig.tools.build_profile.STAGES``) adding up to its wall
time, and none of the measuring tools in the engine's hash."""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from spiderpig.tools.profiler import Profiler, process_age

ROOT = Path(__file__).resolve().parents[1]


def test_profiler_records_stages_counters_and_metrics(caplog):
    prof = Profiler(name="build", total="build_total", logger_name="spiderpig.build")
    with prof.timed("plan"):
        time.sleep(0.01)
    prof.add("import", 0.5)
    prof.add("build_total", 1.0)
    prof.bump("hits", 2)
    prof.set_metric("n_bodies", 3)
    doc = prof.as_dict()
    assert doc["stages"]["import"] == 0.5
    assert doc["stages"]["plan"] >= 0.01
    assert doc["calls"] == {"plan": 1, "import": 1, "build_total": 1}
    assert doc["counters"] == {"hits": 2}
    assert doc["metrics"] == {"n_bodies": 3.0}
    with caplog.at_level(logging.INFO, logger="spiderpig.build"):
        prof.log_summary()
    text = caplog.records[-1].getMessage()
    assert caplog.records[-1].name == "spiderpig.build"
    assert text.startswith("build profile summary:")
    assert "%build" in text
    assert " 50.0" in text                  # import: half of build_total


def test_a_disabled_profiler_records_nothing(caplog):
    prof = Profiler(enabled=False)
    with prof.timed("x"):
        pass
    prof.add("y", 1.0)
    with caplog.at_level(logging.INFO):
        prof.log_summary()
    assert prof.as_dict()["stages"] == {}
    assert not caplog.records


def test_the_bake_keeps_its_summary_and_logger(caplog):
    from spiderpig.bake import _Profiler

    prof = _Profiler()
    with prof.timed("bake_total"), prof.timed("1_reference_build"):
        pass
    with caplog.at_level(logging.INFO, logger="bake_gltf"):
        prof.log_summary()
    rec = caplog.records[-1]
    assert rec.name == "bake_gltf"
    assert rec.getMessage().startswith("bake profile summary:")
    assert "%bake" in rec.getMessage()


@pytest.mark.skipif(not os.path.exists("/proc/self/stat"), reason="no /proc")
def test_process_age_is_the_time_since_start():
    age = process_age()
    assert age is not None
    assert 0 < age < time.time()


@pytest.mark.slow
def test_build_profile_has_every_stage_and_adds_up(tmp_path):
    """``spiderpig build --profile`` of the default design (the Strider double, the roadmap's
    W0 criterion) in its own process: every stage key, their sum within 5 % of the wall time
    measured from outside (which also holds the interpreter's exit, ~1 s, after the profile
    is logged), the outputs written."""
    from spiderpig.tools.build_profile import STAGES

    prof = tmp_path / "profile.json"
    env = {**os.environ, "SPIDERPIG_OFFLINE": "1", "SPIDERPIG_STORE": str(tmp_path / "store")}
    t0 = time.perf_counter()
    p = subprocess.run([sys.executable, "-m", "spiderpig.cli", "build", "--profile",
                        "--profile-json", str(prof),
                        "--out", str(tmp_path / "out"), "--store", str(tmp_path / "store")],
                       cwd=ROOT, env=env, capture_output=True, text=True, check=False)
    outside = time.perf_counter() - t0
    assert p.returncode == 0, p.stderr[-2000:]
    assert "build profile summary:" in p.stderr
    for key in STAGES:
        assert key in p.stderr, key
    doc = json.loads(prof.read_text())
    assert set(STAGES) <= set(doc["stages"])
    summed = sum(doc["stages"][k] for k in STAGES)
    assert abs(summed - doc["wall_s"]) <= 0.05 * doc["wall_s"]
    assert abs(summed - outside) <= 0.05 * outside, (summed, outside)
    assert (tmp_path / "out" / "ORDER.md").exists()


def test_instrumenting_puts_every_call_back():
    from spiderpig.tools import build_profile

    before = [(o, n, o.__dict__[n] if isinstance(o, type) else getattr(o, n))
              for o, n, _ in build_profile._targets()]
    with build_profile.instrumented(Profiler()):
        assert all((o.__dict__[n] if isinstance(o, type) else getattr(o, n)) is not f
                   for o, n, f in before)
    assert all((o.__dict__[n] if isinstance(o, type) else getattr(o, n)) is f
               for o, n, f in before)


TOOLS = ("tools/profiler.py", "tools/build_profile.py", "tools/engine_version.py")


def _engine_version_of(root, monkeypatch):
    """``engine_version()`` itself, run over the package copy at ``root`` (its own file
    selection, no digest cache, nothing remembered)."""
    from spiderpig import design

    monkeypatch.setattr(design, "ROOT", root)
    monkeypatch.setattr(design, "_ENGINE_VERSION", [])
    monkeypatch.setenv(design.DIGEST_CACHE_ENV, "off")
    return design.engine_version()


def test_the_measuring_tools_are_outside_the_engine_hash(tmp_path, monkeypatch):
    """A code change to any measuring tool (the profiler, the build's profile, the
    engine-version printer) leaves ``engine_version()`` as it was, while the same change to
    an engine module changes it: measured with ``engine_version``'s own file selection over
    a copy of the package, so the test fails if a tool module is ever hashed (W0 review: a
    measuring tool must not re-key the stores, the test cache or CI's cache). The scorecard
    and the doc check live in ``tests/``, outside the package."""
    root = _package_copy(tmp_path)
    base = _engine_version_of(root, monkeypatch)
    for tool in TOOLS:                          # every tool changed at once: one more hash
        path = root / tool
        assert path.exists(), tool
        path.write_text(path.read_text() + "\n\ndef _a_change():\n    return 1\n")
    assert _engine_version_of(root, monkeypatch) == base


def test_the_engine_hash_sees_the_same_change_to_an_engine_module(tmp_path, monkeypatch):
    """The control of the test above (split from it: two hashes each, ~2 s): the change it
    makes to the tools, made to ``stack.py``, re-keys the engine."""
    root = _package_copy(tmp_path)
    base = _engine_version_of(root, monkeypatch)
    (root / "stack.py").write_text((root / "stack.py").read_text()
                                   + "\n\ndef _a_change():\n    return 1\n")
    assert _engine_version_of(root, monkeypatch) != base     # the check can see a change


def _package_copy(tmp_path):
    import shutil

    from spiderpig import design

    root = tmp_path / "spiderpig"
    shutil.copytree(design.ROOT, root, ignore=shutil.ignore_patterns(
        "__pycache__", "viewer", "*.pyc"))
    return root


def test_every_stage_is_timed_and_each_wrapped_call_is_the_builds(monkeypatch, tmp_path):
    """The fast twin of the slow drift test: each stage has a wrapped call, each wrapped
    build name is one ``spiderpig.build.main`` really calls, and a run through
    ``build_profile.main`` (the calls stubbed) logs every stage key."""
    import ast
    import inspect

    import spiderpig.build as build_mod
    from spiderpig.tools import build_profile

    targets = build_profile._targets()
    assert {stage for *_, stage in targets} == set(build_profile.STAGES) - {"import"}
    used = {n.id if isinstance(n, ast.Name) else n.attr
            for n in ast.walk(ast.parse(inspect.getsource(build_mod.main)))
            if isinstance(n, ast.Name | ast.Attribute)}
    for _obj, name, _ in targets:
        assert name in used, name               # a renamed call would time nothing

    for obj, name, _ in targets:                # cheap stand-ins, put back by monkeypatch
        monkeypatch.setattr(obj, name, lambda *a, **kw: None)

    def build_main(argv):
        for obj, name, _ in build_profile._targets():
            getattr(obj, name)(None) if isinstance(obj, type) else getattr(obj, name)()
        return 0

    monkeypatch.setattr(build_mod, "main", build_main)
    out = tmp_path / "p.json"
    assert build_profile.main(["--profile-json", str(out)]) == 0
    stages = json.loads(out.read_text())["stages"]
    assert set(build_profile.STAGES) - {"import"} <= set(stages)


def test_build_help_lists_the_profile_options_and_profiles_nothing(capsys, caplog):
    from spiderpig import cli

    with caplog.at_level(logging.INFO, logger="spiderpig.build"), \
            pytest.raises(SystemExit) as done:
        cli.main(["build", "--profile", "--help"])
    assert done.value.code == 0
    out = capsys.readouterr().out
    assert "--side-only" in out                 # the build's own options
    assert "--profile-json FILE" in out
    assert not [r for r in caplog.records if "profile summary" in r.getMessage()]
    with pytest.raises(SystemExit):
        cli.main(["build", "--help"])
    assert "--profile-json FILE" in capsys.readouterr().out
