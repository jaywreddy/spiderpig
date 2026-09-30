"""Shared fixtures for the spiderpig test suite.

Fabrication is the slow part, so the session builds each design, side and
robot once (the ``design`` / ``side`` / ``robot`` factories below, cached
per key) and every test reads them: never mutate what they return. Tests
never download servo models (``_offline``): every servo is drawn
parametrically.

Only ``tests/e2e/`` needs Playwright, so we gate the browser-context fixture
behind a ``pytestmark`` in those modules and keep everything else plain-pytest.
"""

from __future__ import annotations

import socket
import subprocess
import sys
import time
from collections.abc import Iterator
from pathlib import Path

import pytest

import servos
from config import BuildConfig
from fabricate import design_side, fabricate, fabricate_side
from linkage import build_module_template
from servos import cad as cadlib
from servos import model

_REPO_ROOT = Path(__file__).resolve().parents[1]


def clear_model_caches() -> None:
    for f in (model.cad_servo, model._servo_part, cadlib._load_cached):
        f.cache_clear()


@pytest.fixture(scope="session", autouse=True)
def _offline(tmp_path_factory):
    """No downloads and an empty model cache: servos are drawn parametrically; the
    spiderpig API's project store (``$SPIDERPIG_STORE``) is a folder of this session, so
    tests never touch the checkout's ``.spiderpig/``."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv(cadlib.OFFLINE_ENV, "1")
        mp.setenv(cadlib.CACHE_ENV, str(tmp_path_factory.mktemp("cad")))
        mp.setenv("SPIDERPIG_STORE", str(tmp_path_factory.mktemp("store")))
        clear_model_caches()
        yield
        clear_model_caches()


@pytest.fixture(scope="session")
def design():
    """``design(module, servo=DEFAULT, linkage="klann") -> (side template, SideDesign)``."""
    cache: dict = {}

    def get(module: str = "single", servo: str = servos.DEFAULT, linkage: str = "klann"):
        key = (module, servo, linkage)
        if key not in cache:
            tmpl = build_module_template(module, linkage=linkage)
            cfg = BuildConfig(module=module, servo=servo, linkage=linkage, robot=False)
            cache[key] = (tmpl, design_side(tmpl, cfg))
        return cache[key]

    return get


@pytest.fixture(scope="session")
def side(design):
    """``side(module, t, servo=DEFAULT)``: the fabricated side (one build per key per session)."""
    cache: dict = {}

    def get(module: str = "single", t: float = 1.0, servo: str = servos.DEFAULT):
        key = (module, t, servo)
        if key not in cache:
            tmpl, d = design(module, servo)
            cache[key] = fabricate_side(d, tmpl.freeze_at(t))
        return cache[key]

    return get


@pytest.fixture(scope="session")
def robot(design):
    """``robot(module, t)``: the fabricated robot (both sides and the chassis)."""
    cache: dict = {}

    def get(module: str = "single", t: float = 1.0):
        if (module, t) not in cache:
            tmpl, _ = design(module)
            cache[module, t] = fabricate(tmpl, BuildConfig(module=module), t)
        return cache[module, t]

    return get


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="session")
def viewer_server() -> Iterator[str]:
    """Start the FastAPI server once per session and yield its base URL.

    E2E tests run against the built viewer bundle (``viewer/dist``) on a
    single port — `mise run test` invokes ``viewer-build`` first via the
    task ``depends`` chain, so the bundle is up to date.
    """
    dist = _REPO_ROOT / "viewer" / "dist"
    if not dist.is_dir():
        raise RuntimeError(
            f"viewer/dist missing — run `mise run viewer-build` before e2e tests "
            f"(or use `mise run test`, which builds it for you). Looked at {dist}"
        )
    port = _free_port()
    proc = subprocess.Popen(  # noqa: S603
        [
            sys.executable, "-m", "uvicorn", "server.app:app",
            "--host", "127.0.0.1", "--port", str(port),
            "--log-level", "warning",
        ],
        cwd=_REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    url = f"http://127.0.0.1:{port}"
    try:
        # Wait for server to accept connections (first-run bake can be slow).
        deadline = time.time() + 180
        while time.time() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f"viewer server exited early (code {proc.returncode})")
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=1):
                    break
            except OSError:
                time.sleep(0.5)
        else:
            raise RuntimeError("viewer server did not start within 180s")
        yield url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
