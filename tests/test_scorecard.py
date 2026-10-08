"""The scorecard's ``--compare`` (``tests/scorecard.py``): exit codes up front, and a warning
when two scorecards' timings aren't comparable."""

from __future__ import annotations

from tests import scorecard


def _card(rc=0, workers=4, load=3.0, wall=100.0):
    return {"meta": {"host": "box"},
            "loadavg": {"start": [load, load, load], "quick": {"before": [load] * 3,
                                                              "after": [load] * 3}},
            "quick": {"rc": rc, "workers": workers, "wall_s": wall, "tests": 10},
            "build": {"cold": {"rc": [0, 0, 0], "wall_s": 40.0}}}


def test_an_exit_code_change_is_shown_first():
    text = scorecard.compare_report(_card(), _card(rc=1))
    assert text.splitlines()[0] == "EXIT CODES (changed, or non-zero):"
    assert "!! quick.rc: 0 -> 1" in text
    assert scorecard.rc_changes(_card(), _card(rc=1)) == [("quick.rc", 0.0, 1.0)]


def test_a_non_zero_exit_code_is_shown_even_unchanged():
    assert scorecard.rc_changes(_card(rc=2), _card(rc=2)) == [("quick.rc", 2.0, 2.0)]
    assert scorecard.rc_changes(_card(), _card()) == []


def test_other_workers_or_load_warn():
    assert scorecard.comparability(_card(), _card()) == []
    warn = scorecard.comparability(_card(), _card(workers=12))
    assert any("quick.workers: 4 vs 12" in w for w in warn)
    assert any("load" in w for w in scorecard.comparability(_card(load=2.0),
                                                           _card(load=5.0)))
    assert not scorecard.comparability(_card(load=2.0), _card(load=3.9))
    assert scorecard.compare_report(_card(), _card(workers=12)).startswith("WARNING:")


def test_a_missing_workers_or_runs_record_warns():
    old = _card()
    del old["quick"]["workers"]
    new = {**_card(), "build": {**_card()["build"], "runs": 3}}
    warn = scorecard.comparability(old, new)
    assert "quick.workers: not recorded in A (timings may not be comparable)" in warn
    assert "build.runs: not recorded in A (timings may not be comparable)" in warn
    assert any("not recorded in B" in w for w in scorecard.comparability(new, old))
    # a section only one card ran is no warning (nothing of it is compared)
    only = {**_card(), "full": {"warm": {"workers": 6, "wall_s": 1.0}}}
    assert scorecard.comparability(_card(), only) == []


def test_the_deltas():
    rows = scorecard.compare(_card(), _card(wall=50.0))
    assert ("quick.wall_s", 100.0, 50.0) in rows
    assert all(not k.startswith("loadavg.") for k, *_ in rows)


def _subprocess_coverage(tmp_path, rcfile) -> float:
    """The coverage of ``spiderpig/tools/profiler.py`` from a ``coverage run`` whose program
    reaches the module only through a child Python (as ``spiderpig.workers`` jobs and the
    CLI runs do), the data combined."""
    import json
    import os
    import subprocess
    import sys

    driver = tmp_path / "driver.py"
    driver.write_text(
        "import subprocess, sys\n"
        "subprocess.run([sys.executable, '-c', 'from spiderpig.tools.profiler import "
        "process_age; process_age()'], check=True)\n")
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("COV_CORE", "COVERAGE"))}       # not the outer run's
    env["COVERAGE_FILE"] = str(tmp_path / ".coverage")
    root = scorecard.ROOT
    subprocess.run([sys.executable, "-m", "coverage", "run", f"--rcfile={rcfile}",
                    str(driver)], cwd=root, env=env, check=True, capture_output=True)
    import coverage

    data = coverage.Coverage(data_file=str(tmp_path / ".coverage"), config_file=str(rcfile))
    data.combine([str(tmp_path)])
    report = tmp_path / "cov.json"
    cwd = os.getcwd()
    os.chdir(root)                          # (paths relative to the repo, as the scorecard's)
    try:
        # the one file read below (every other file of the source parsed for nothing: 4 s)
        data.json_report(outfile=str(report), include=["spiderpig/tools/profiler.py"])
    except coverage.exceptions.NoDataError:    # nothing of spiderpig/ measured at all
        return 0.0
    finally:
        os.chdir(cwd)
    files = json.loads(report.read_text())["files"]
    return files.get("spiderpig/tools/profiler.py", {}).get("summary", {}).get(
        "percent_covered", 0.0)


def test_coverage_follows_subprocesses(tmp_path):
    """pyproject's ``[tool.coverage.run] patch = ["subprocess"]``: a module reached only in
    a child process counts (pytest-cov 7 dropped that); the control, a config without the
    patch, sees none of it."""
    (tmp_path / "with").mkdir()
    (tmp_path / "without").mkdir()
    control = tmp_path / "without" / "rc.toml"
    control.write_text('[tool.coverage.run]\nsource = ["spiderpig"]\n')
    assert _subprocess_coverage(tmp_path / "with", scorecard.ROOT / "pyproject.toml") > 0
    assert _subprocess_coverage(tmp_path / "without", control) == 0
