"""pyright's ratchet (``mise run pyright-check``, CI; ``docs/agentlib/ROADMAP.md`` W6): the
error count of ``[tool.pyright]`` may only go down.

    python -m tests.pyright_baseline              # run pyright, compare with the baseline
    python -m tests.pyright_baseline --update     # write the current count as the baseline
    python -m tests.pyright_baseline --report F   # read pyright's --outputjson F instead

``[tool.pyright]`` covers the package and the tests (since 2026-10-14), each held on its own
count (:data:`AREAS`): the package's ``errors`` / ``by_rule`` at the top of the baseline
(``tests/pyright-baseline.json``), the tests' under ``"tests"`` (hundreds when they were
added: the tests' optional reads of reports and parts, typing noise rather than bugs, left
as they were so the package's own count stays readable). The check fails when an area's
total rises; a rule's count rising while the total falls is printed, not failed (a fix can
move an error from one rule to another). When a total falls the check says so: lower the
baseline with ``--update`` in the same change, so the ratchet holds.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / "tests" / "pyright-baseline.json"
PYRIGHT = ROOT / "viewer" / "node_modules" / ".bin" / "pyright"


def run_pyright() -> dict:
    """pyright's ``--outputjson`` document for the project (exit 1 on errors is expected)."""
    if not PYRIGHT.exists():
        sys.exit(f"{PYRIGHT.relative_to(ROOT)} missing: run `mise run viewer-install`")
    if shutil.which("node") is None:
        sys.exit("no node on PATH: run through `mise run pyright-check`")
    p = subprocess.run([str(PYRIGHT), "--outputjson"], cwd=ROOT, capture_output=True,
                       text=True, check=False, env=dict(os.environ))
    try:
        return json.loads(p.stdout)
    except json.JSONDecodeError:
        sys.exit(f"pyright exited {p.returncode} without a report:\n{p.stderr[-2000:]}")


AREAS = ("package", "tests")
"""Each held on its own count: ``tests`` the files under ``tests/``, ``package`` the rest
(``spiderpig/``)."""


def area(diag: dict) -> str:
    """Which count a diagnostic is in (:data:`AREAS`)."""
    path = Path(diag["file"])
    try:
        rel = (path if path.is_absolute() else ROOT / path).resolve().relative_to(ROOT)
    except ValueError:
        return "package"
    return "tests" if rel.parts[:1] == ("tests",) else "package"


def errors_of(doc: dict, where: str | None = None) -> list[dict]:
    """The report's errors (``where``: one area's)."""
    return [d for d in doc["generalDiagnostics"] if d["severity"] == "error"
            and (where is None or area(d) == where)]


def _tally(errors: list[dict]) -> dict:
    by_rule = Counter(d.get("rule", "?") for d in errors)
    return {"errors": len(errors),
            "by_rule": dict(sorted(by_rule.items(), key=lambda kv: (-kv[1], kv[0])))}


def count(doc: dict) -> dict:
    """The errors of a pyright report: the package's total and per rule (``?`` for a
    rule-less one) at the top, the tests' the same under ``"tests"``."""
    return {**_tally(errors_of(doc, "package")), "tests": _tally(errors_of(doc, "tests"))}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--update", action="store_true", help="write the count as the baseline")
    ap.add_argument("--report", type=Path, help="a pyright --outputjson file to read")
    ap.add_argument("--baseline", type=Path, default=BASELINE, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    baseline: Path = args.baseline
    doc = json.loads(args.report.read_text()) if args.report else run_pyright()
    now = count(doc)
    if args.update:
        baseline.write_text(json.dumps(now, indent=2) + "\n")
        print(f"baseline: {now['errors']} pyright errors in the package, "
              f"{now['tests']['errors']} in the tests -> {baseline}")
        return 0
    base = json.loads(baseline.read_text())
    failed = False
    for where in AREAS:
        have = now if where == "package" else now[where]
        # (a baseline from before the tests were covered holds none: every error is new)
        was_doc = base if where == "package" else base.get(where, {"errors": 0, "by_rule": {}})
        for rule, n in have["by_rule"].items():
            was = was_doc["by_rule"].get(rule, 0)
            if n > was:
                print(f"  {where} {rule}: {was} -> {n}")
        if have["errors"] > was_doc["errors"]:
            for d in errors_of(doc, where):
                line = d["range"]["start"]["line"] + 1
                print(f"{d['file']}:{line}: {d['message']}"
                      f" ({d.get('rule', '?')})")
            print(f"pyright ({where}): {have['errors']} errors, the baseline allows "
                  f"{was_doc['errors']}: fix the new ones (the list above holds them all)")
            failed = True
        elif have["errors"] < was_doc["errors"]:
            print(f"pyright ({where}): {have['errors']} errors, under the baseline's "
                  f"{was_doc['errors']}: lower it with `python -m tests.pyright_baseline "
                  "--update`")
        else:
            print(f"pyright ({where}): {have['errors']} errors, at the baseline")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
