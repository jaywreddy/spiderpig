"""pyright's ratchet (``mise run pyright-check``, CI; ``docs/agentlib/ROADMAP.md`` W6): the
error count of ``[tool.pyright]`` may only go down.

    python -m tests.pyright_baseline              # run pyright, compare with the baseline
    python -m tests.pyright_baseline --update     # write the current count as the baseline
    python -m tests.pyright_baseline --report F   # read pyright's --outputjson F instead

The baseline (``tests/pyright-baseline.json``) holds the total and the count per rule. The
check fails when the total rises; a rule's count rising while the total falls is printed,
not failed (a fix can move an error from one rule to another). When the total falls the
check says so: lower the baseline with ``--update`` in the same change, so the ratchet holds.
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


def count(doc: dict) -> dict:
    """The errors of a pyright report: the total and per rule (``?`` for a rule-less one)."""
    errors = [d for d in doc["generalDiagnostics"] if d["severity"] == "error"]
    by_rule = Counter(d.get("rule", "?") for d in errors)
    return {"errors": len(errors),
            "by_rule": dict(sorted(by_rule.items(), key=lambda kv: (-kv[1], kv[0])))}


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
        print(f"baseline: {now['errors']} pyright errors -> {baseline}")
        return 0
    base = json.loads(baseline.read_text())
    for rule, n in now["by_rule"].items():
        was = base["by_rule"].get(rule, 0)
        if n > was:
            print(f"  {rule}: {was} -> {n}")
    if now["errors"] > base["errors"]:
        for d in doc["generalDiagnostics"]:
            if d["severity"] == "error":
                line = d["range"]["start"]["line"] + 1
                print(f"{d['file']}:{line}: {d['message']}"
                      f" ({d.get('rule', '?')})")
        print(f"pyright: {now['errors']} errors, the baseline allows {base['errors']}: "
              "fix the new ones (the list above holds them all)")
        return 1
    if now["errors"] < base["errors"]:
        print(f"pyright: {now['errors']} errors, under the baseline's {base['errors']}: "
              "lower it with `python -m tests.pyright_baseline --update`")
    else:
        print(f"pyright: {now['errors']} errors, at the baseline")
    return 0


if __name__ == "__main__":
    sys.exit(main())
