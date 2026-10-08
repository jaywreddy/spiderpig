"""pyright's ratchet (``tests/pyright_baseline.py``): the count may fall, never rise."""

from __future__ import annotations

import json

from tests import pyright_baseline


def _report(path, rules: list[str], warnings: int = 0):
    diags = [{"file": "spiderpig/x.py", "severity": "error", "rule": r, "message": "m",
              "range": {"start": {"line": i, "character": 0}}} for i, r in enumerate(rules)]
    diags += [{"file": "spiderpig/x.py", "severity": "warning", "rule": "w", "message": "m",
               "range": {"start": {"line": 0, "character": 0}}}] * warnings
    path.write_text(json.dumps({"generalDiagnostics": diags}))
    return path


def test_the_count_is_per_rule_and_leaves_warnings_out(tmp_path):
    doc = json.loads(_report(tmp_path / "r.json", ["a", "b", "a"], warnings=2).read_text())
    assert pyright_baseline.count(doc) == {"errors": 3, "by_rule": {"a": 2, "b": 1}}


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
