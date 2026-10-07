"""The viewer's walking model against the Python one, in the Python tiers: the vitest of
``viewer/src/drive/model.test.ts`` (the reference Klann quad and the Strider double,
``tests/fixtures/linkage/walk_reference*.json``) run from pytest, so ``test-quick`` and the
full suite catch a drift between ``drive/model.ts`` and ``spiderpig/walk.py``, not only
``mise run test-viewer``.

It needs Node and the viewer's packages (``mise run viewer-install``); without them it
skips and says why, unless ``SPIDERPIG_REQUIRE_VIEWER_TESTS=1`` (CI), where that is a
failure.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
VITEST = VIEWER / "node_modules" / "vitest" / "vitest.mjs"
TEST_FILE = "src/drive/model.test.ts"
REQUIRE_ENV = "SPIDERPIG_REQUIRE_VIEWER_TESTS"


def _node() -> str | None:
    node = shutil.which("node")
    if node is not None:
        return node
    mise = shutil.which("mise") or str(Path.home() / ".local" / "bin" / "mise")
    if not Path(mise).is_file():
        return None
    got = subprocess.run([mise, "which", "node"], cwd=VIEWER, capture_output=True, text=True,
                         check=False, timeout=60)
    return (got.stdout.strip() or None) if got.returncode == 0 else None


def _missing() -> str | None:
    if not VITEST.is_file():
        return f"{VITEST.relative_to(VIEWER.parent)} is missing: run `mise run viewer-install`"
    if _node() is None:
        return "no `node` on PATH (mise.toml pins Node 20: `mise install`)"
    return None


def test_the_viewers_walk_model_gives_the_python_models_numbers():
    why = _missing()
    if why is not None:
        if os.environ.get(REQUIRE_ENV) == "1":
            pytest.fail(f"{REQUIRE_ENV}=1 but the viewer's test can't run: {why}")
        pytest.skip(why)
    got = subprocess.run([_node(), str(VITEST), "run", TEST_FILE], cwd=VIEWER,
                         capture_output=True, text=True, timeout=180, check=False,
                         env={**os.environ, "CI": "1", "NO_COLOR": "1"})
    out = got.stdout + got.stderr
    assert got.returncode == 0, f"vitest {TEST_FILE} failed:\n{out[-4000:]}"
    assert "model.test.ts" in out, out[-2000:]
    assert "passed" in out, out[-2000:]
