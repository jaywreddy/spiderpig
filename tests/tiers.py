"""The quick tier: which cases of a heavy parametrized test run in ``-m 'not slow'``.

``quick(values, keep)`` returns ``values`` for ``pytest.mark.parametrize`` with every value
not in ``keep`` marked ``slow``: the quick tier (``mise run test-quick``) runs the kept
cases, the full tier (``mise run remote-test``) runs them all. Test ids are unchanged.
For a parametrize over several names, ``values`` are tuples and ``keep`` holds tuples.
"""

from __future__ import annotations

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
