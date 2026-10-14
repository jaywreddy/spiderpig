"""A task's linkages, ranked: ``spiderpig candidates TASK.json [--measure N] [--limit K]``.

    spiderpig candidates task.json                       # every linkage that can do it
    spiderpig candidates task.json --measure 2 --limit 5 # the best two planned (minutes cold)
    spiderpig candidates - --json < task.json            # the report as JSON
    spiderpig build 1a2b3c4d5e6f7a8b                     # then a candidate's design ...
    spiderpig export 1a2b3c4d5e6f7a8b --formats guide control order   # ... and the rest

The API's :func:`spiderpig.api.candidates` on the command line: the task is a Task document
(``spiderpig://schema/task``, :mod:`spiderpig.task`: ``do`` walk, place or trace and its
numbers; no linkage named), from a file or ``-`` (stdin). ``--measure N`` plans the best N
that pass the screen into the store (``--store``), so their numbers are the stages';
``--resolve K`` (default 1) resolves the best K's specs into it too (no planning), so each
has the ``design`` id every other command takes (``spiderpig build|guide|control|precision|
export|view|sim DESIGN``).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="spiderpig candidates",
        description="the linkages that can do a task, ranked, each with its spec and its "
                    "design id in the store")
    ap.add_argument("task", metavar="TASK.json",
                    help="the task document (spiderpig://schema/task), or - for stdin")
    ap.add_argument("--measure", type=int, default=0, metavar="N",
                    help="plan the best N that pass the screen (their numbers then the "
                         "stages'; an arm's plan takes minutes cold). Default 0")
    ap.add_argument("--limit", type=int, default=None, metavar="K",
                    help="keep the best K candidates")
    ap.add_argument("--resolve", type=int, default=1, metavar="K",
                    help="resolve the best K candidates' specs into the store (their design "
                         "ids, for build, guide, control, export ...). Default 1")
    ap.add_argument("--store", metavar="PATH",
                    help="the design store (default: $SPIDERPIG_STORE, else ./.spiderpig)")
    ap.add_argument("--json", action="store_true", help="print the report as JSON")
    args = ap.parse_args(argv)
    try:
        text = sys.stdin.read() if args.task == "-" else Path(args.task).read_text()
        doc = json.loads(text)
    except (OSError, ValueError) as e:
        print(f"error: can't read the task {args.task}: {e}", file=sys.stderr)
        return 2
    from spiderpig import api
    from spiderpig.spec import SpecErrors
    from spiderpig.store import Store

    store = Store.of(args.store) if args.store else Store.default()
    try:
        rep = api.candidates(doc, store, measure=args.measure, limit=args.limit)
    except SpecErrors as e:
        for err in e.errors:
            print(f"error: {err.describe()}", file=sys.stderr)
        return 2
    for c in rep.candidates[:max(args.resolve, 0)]:
        if c.design is None:
            c.design = api.resolve(c.spec, store).id
    if args.json:
        print(json.dumps(rep.to_dict(), indent=1))
        return 0
    print(rep.describe())
    for c in rep.candidates:
        if c.design:
            print(f"design {c.design}: {c.rank}. {c.linkage} {c.module}")
    best = next((c for c in rep.candidates if c.design), None)
    if best is not None:
        print(f"\nnext: spiderpig build {best.design} (or export {best.design} --formats ...: "
              "guide, control, order to assemble, wire and buy it)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
