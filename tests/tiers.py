"""The test tiers: which cases of a heavy parametrized test run in ``-m 'not slow'``, and
the runner of a module's fast tier.

``quick(values, keep)`` returns ``values`` for ``pytest.mark.parametrize`` with every value
not in ``keep`` marked ``slow``: the quick tier (``mise run test-quick``) runs the kept
cases, the full tier (``mise run remote-test``) runs them all. Test ids are unchanged.
For a parametrize over several names, ``values`` are tuples and ``keep`` holds tuples.

``python -m tests.tiers <module> [pytest args]`` (``mise run test-<module>``) runs one
module's fast tier: its own files (:func:`module_files`) with ``-m "<module> and not slow
and not e2e"``, on :data:`TIER_WORKERS` xdist workers (``SPIDERPIG_TIER_WORKERS`` changes
it; 0 runs in this process); a module with no fast test yet passes, saying so.
``python -m tests.tiers fixtures`` (``mise run test-fixtures``) runs the recorded
fixtures' currency tests with ``--regen``. ``docs/agentlib/TESTING.md`` has the tiers.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Iterable

import pytest


def quick(values: Iterable, keep: Iterable) -> list:
    keep = list(keep)
    out = []
    for v in values:
        if v in keep:
            out.append(v)
        else:
            out.append(pytest.param(*(v if isinstance(v, tuple) else (v,)),
                                    marks=pytest.mark.slow))
    return out


TIER_WORKERS = {"linkage": 2, "planner": 2, "construction": 4, "hardware": 0, "strength": 0,
                "api": 2, "sim": 2, "server": 0}
"""xdist workers per module's fast tier (0: none, in this process): a worker costs its own
imports and the engine's version hash, which a small tier doesn't win back (the hardware
tier: 8 s in one process against 13 s on 4 workers). ``SPIDERPIG_TIER_WORKERS`` overrides."""


def module_files(module: str) -> list[str]:
    """The test files of ``module``: its files in :data:`tests._modules.MODULE_OF_FILE`, and
    any other file that marks a test ``@pytest.mark.<module>``."""
    from pathlib import Path

    from tests._modules import MODULE_OF_FILE

    here = Path(__file__).resolve().parent
    files = {here / f for f, m in MODULE_OF_FILE.items() if m == module}
    mark = f"pytest.mark.{module}"
    for f in here.rglob("test_*.py"):
        if f not in files and mark in f.read_text():
            files.add(f)
    return sorted(str(f) for f in files if f.exists())


def main(argv: list[str] | None = None) -> int:
    from tests._modules import MODULES

    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in (*MODULES, "fixtures"):
        print(f"usage: python -m tests.tiers {{{','.join(MODULES)},fixtures}} [pytest args]",
              file=sys.stderr)
        return 2
    module, rest = argv[0], argv[1:]
    workers = os.environ.get("SPIDERPIG_TIER_WORKERS", str(TIER_WORKERS.get(module, 4)))
    if module == "fixtures":
        args = ["--regen", "-m", "fixture_regen"]
    else:
        # only the module's own files (and any file that marks a test of it): collecting
        # every file costs each worker its imports (a 12 s floor before, for one test)
        args = [*module_files(module), "-m", f"{module} and not slow and not e2e"]
    xdist = ["-p", "no:xdist"] if workers == "0" else ["-n", workers, "--dist", "worksteal"]
    rc = pytest.main(["-p", "no:warnings", *args, *xdist, *rest])
    if rc == pytest.ExitCode.NO_TESTS_COLLECTED:
        print(f"no {'currency' if module == 'fixtures' else 'fast'} tests in {module!r} yet")
        return 0
    return int(rc)


if __name__ == "__main__":
    raise SystemExit(main())
