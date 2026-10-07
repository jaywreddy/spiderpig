"""The stage profiler (``spiderpig/profiler.py``): the bake's summary unchanged, the build's
``--profile`` stages (``spiderpig.build.STAGES``) adding up to its wall time."""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from spiderpig.profiler import Profiler, process_age

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
    from spiderpig.build import STAGES

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
