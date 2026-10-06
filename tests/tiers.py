"""The test tiers: which cases of a heavy parametrized test run in ``-m 'not slow'``, and
the runner of a module's fast tier.

``quick(values, keep)`` returns ``values`` for ``pytest.mark.parametrize`` with every value
not in ``keep`` marked ``slow``: the quick tier (``mise run test-quick``) runs the kept
cases, the full tier (``mise run remote-test``) runs them all. Test ids are unchanged.
For a parametrize over several names, ``values`` are tuples and ``keep`` holds tuples.

``python -m tests.tiers <module> [pytest args]`` (``mise run test-<module>``) runs one
module's fast tier: ``-m "<module> and not slow and not e2e" -n 4 --dist worksteal``
(``SPIDERPIG_TIER_WORKERS`` changes ``-n``); a module with no fast test yet passes, saying
so. ``python -m tests.tiers fixtures`` (``mise run test-fixtures``) runs the recorded
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


def main(argv: list[str] | None = None) -> int:
    from tests._modules import MODULES

    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in (*MODULES, "fixtures"):
        print(f"usage: python -m tests.tiers {{{','.join(MODULES)},fixtures}} [pytest args]",
              file=sys.stderr)
        return 2
    module, rest = argv[0], argv[1:]
    workers = os.environ.get("SPIDERPIG_TIER_WORKERS", "4")
    if module == "fixtures":
        args = ["--regen", "-m", "fixture_regen"]
    else:
        args = ["-m", f"{module} and not slow and not e2e"]
    rc = pytest.main(["-p", "no:warnings", *args, "-n", workers, "--dist", "worksteal", *rest])
    if rc == pytest.ExitCode.NO_TESTS_COLLECTED:
        print(f"no {'currency' if module == 'fixtures' else 'fast'} tests in {module!r} yet")
        return 0
    return int(rc)


if __name__ == "__main__":
    raise SystemExit(main())
