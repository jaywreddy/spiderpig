"""pyright's ratchet (``tests/pyright_baseline.py``): each area's count (the package's, the
tests') may fall, never rise."""

from __future__ import annotations

import json

from tests import pyright_baseline


def _report(path, rules: list[str], warnings: int = 0, tests: tuple[str, ...] = ()):
    diags = [{"file": "spiderpig/x.py", "severity": "error", "rule": r, "message": "m",
              "range": {"start": {"line": i, "character": 0}}} for i, r in enumerate(rules)]
    diags += [{"file": str(pyright_baseline.ROOT / "tests" / "test_x.py"), "severity": "error",
               "rule": r, "message": "m", "range": {"start": {"line": i, "character": 0}}}
              for i, r in enumerate(tests)]
    diags += [{"file": "spiderpig/x.py", "severity": "warning", "rule": "w", "message": "m",
               "range": {"start": {"line": 0, "character": 0}}}] * warnings
    path.write_text(json.dumps({"generalDiagnostics": diags}))
    return path


def test_the_count_is_per_rule_and_leaves_warnings_out(tmp_path):
    doc = json.loads(_report(tmp_path / "r.json", ["a", "b", "a"], warnings=2).read_text())
    assert pyright_baseline.count(doc) == {"errors": 3, "by_rule": {"a": 2, "b": 1},
                                           "tests": {"errors": 0, "by_rule": {}}}


def test_a_rise_fails_and_a_fall_or_the_same_passes(tmp_path, capsys):
    base = tmp_path / "base.json"
    report = _report(tmp_path / "r.json", ["a", "b"])
    assert pyright_baseline.main(["--report", str(report), "--baseline", str(base),
                                  "--update"]) == 0
    assert json.loads(base.read_text())["errors"] == 2
    check = ["--report", str(report), "--baseline", str(base)]
    assert pyright_baseline.main(check) == 0
    _report(report, ["a"])
    assert pyright_baseline.main(check) == 0
    assert "under the baseline" in capsys.readouterr().out
    _report(report, ["a", "c", "c"])                # one more, in a new rule
    assert pyright_baseline.main(check) == 1
    out = capsys.readouterr().out
    assert "c: 0 -> 2" in out
    assert "the baseline allows 2" in out


def test_the_tests_are_held_on_a_count_of_their_own(tmp_path, capsys):
    """The tests' errors neither hide behind the package's fixes nor count against them: a
    rise in either area fails, whatever the other did."""
    base = tmp_path / "base.json"
    report = _report(tmp_path / "r.json", ["a", "b"], tests=("o", "o", "p"))
    check = ["--report", str(report), "--baseline", str(base)]
    assert pyright_baseline.main([*check, "--update"]) == 0
    doc = json.loads(base.read_text())
    assert (doc["errors"], doc["tests"]["errors"]) == (2, 3)
    _report(report, [], tests=("o", "o", "p", "q"))       # the package fixed, a test worse
    assert pyright_baseline.main(check) == 1
    out = capsys.readouterr().out
    assert "pyright (tests): 4 errors, the baseline allows 3" in out
    assert "pyright (package): 0 errors, under the baseline's 2" in out
    _report(report, ["a", "b", "c"], tests=("o",))         # the reverse
    assert pyright_baseline.main(check) == 1
    assert "pyright (package): 3 errors, the baseline allows 2" in capsys.readouterr().out
    base.write_text(json.dumps({"errors": 2, "by_rule": {"a": 1, "b": 1}}))  # (no tests' entry)
    _report(report, ["a", "b"], tests=("o",))
    assert pyright_baseline.main(check) == 1                # every test error is new to it
