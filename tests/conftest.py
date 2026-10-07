"""Shared fixtures and hooks for the spiderpig test suite.

Fabrication is the slow part, so each design, side and robot is built once per engine
version, not once per run or per xdist worker: the ``design`` / ``side`` / ``robot``
factories below are thin wrappers over :mod:`tests.cache` (plans seeded from disk,
fabrications as BREP files under ``CACHE_DIR``; one object per key per process), and
every test reads them: never mutate what they return. Tests never download servo models
(``_offline``): every servo is drawn parametrically.

Hooks: ``--regen`` (rewrite the recorded fixtures, ``tests/fixtures/``), ``--no-test-cache``
(the disk cache off: every process builds what it needs once, as before the cache), and
the module markers (every test gets exactly one, its file's default from
``tests/_modules.py`` unless it names its own: ``mise run test-<module>``). See
``docs/agentlib/TESTING.md``.

Only ``tests/e2e/`` needs Playwright, so we gate the browser-context fixture
behind a ``pytestmark`` in those modules and keep everything else plain-pytest.
"""

from __future__ import annotations

import os

# Under pytest-xdist every worker is its own process: one BLAS / OpenMP thread each, or
# N workers x N cores of threads fight over the machine (set before numpy is imported).
if os.environ.get("PYTEST_XDIST_WORKER"):
    for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(_var, "1")
    # OCCT's parallel booleans and meshing share its default pool (every core by default)
    from OCP.OSD import OSD_ThreadPool

    OSD_ThreadPool.DefaultPool_s(int(os.environ.get("SPIDERPIG_TEST_OCCT_THREADS", "1")))

import socket
import subprocess
import sys
import time
from collections.abc import Iterator
from pathlib import Path

import pytest

from spiderpig import servos
from spiderpig.config import BuildConfig
from spiderpig.servos import cad as cadlib
from spiderpig.servos import model
from tests import cache
from tests._modules import MODULE_OF_FILE, MODULES

_REPO_ROOT = Path(__file__).resolve().parents[1]


def pytest_addoption(parser):
    parser.addoption("--regen", action="store_true", default=False,
                     help="rewrite the recorded fixtures (tests/fixtures/) from the engine "
                          "(`mise run test-fixtures`: with -m fixture_regen)")
    parser.addoption("--no-test-cache", action="store_true", default=False,
                     help="don't read or write the fabrication cache (tests/cache.py)")


def pytest_configure(config):
    cache.REGEN = config.getoption("regen")
    if config.getoption("no_test_cache"):
        cache.ENABLED = False
    if not hasattr(config, "workerinput") and cache.ENABLED and cache.enabled():
        cache.prune()       # the controller only: engine folders nobody used for 14 days


def pytest_report_header(config):
    if not cache.ENABLED or not cache.enabled():
        return ["spiderpig test cache: off"]
    lines = [f"spiderpig test cache: {cache.cache_dir()}"]
    stale = cache.stale_fixtures()
    if stale:
        lines.append(f"recorded fixtures from another engine version: {len(stale)} "
                     "(used as they are; `mise run test-fixtures` checks and rewrites them)")
    return lines


@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config, items):
    """Every test gets exactly one module marker before ``-m`` selects: its own, else its
    file's (``tests/_modules.py``)."""
    tests_dir = _REPO_ROOT / "tests"
    unknown, double = set(), []
    for item in items:
        own = {m.name for m in item.iter_markers() if m.name in MODULES}
        if len(own) > 1:
            double.append(f"{item.nodeid}: {sorted(own)}")
        elif not own:
            try:
                rel = Path(item.path).resolve().relative_to(tests_dir).as_posix()
            except ValueError:
                rel = str(item.path)
            module = MODULE_OF_FILE.get(rel)
            if module is None:
                unknown.add(rel)
            else:
                item.add_marker(getattr(pytest.mark, module))
    if unknown or double:
        raise pytest.UsageError(
            "every test needs exactly one module marker (" + ", ".join(MODULES) + "):\n"
            + "".join(f"  {f}: not in tests/_modules.py MODULE_OF_FILE\n"
                      for f in sorted(unknown))
            + "".join(f"  {d}: more than one\n" for d in double[:20]))


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
        # the product's fabrication cache off: a test fabricates or uses tests/cache.py
        # (a `fresh=True` build must be fresh; tests that test the product cache set it)
        mp.setenv("SPIDERPIG_FAB_CACHE", "off")
        clear_model_caches()
        yield
        clear_model_caches()


@pytest.fixture(scope="session")
def design():
    """``design(module, servo=DEFAULT, linkage="klann", **build) -> (side template,
    SideDesign)``, built the default way (Chicago screw pins, standoff pillars, the bolt
    crank) unless ``build`` names a construction (``crank="bolt_round"``: the round standoff
    crankpins, the only other one since 2026-10-07).
    :func:`tests.cache.cached_design`: the plan seeded from the cache."""

    def get(module: str = "single", servo: str = servos.DEFAULT, linkage: str = "klann",
            **build: str):
        cfg = BuildConfig(module=module, servo=servo, linkage=linkage, robot=False, **build)
        return cache.cached_design(cfg)

    return get


@pytest.fixture(scope="session")
def side():
    """``side(module, t, servo=DEFAULT, **build)``: the fabricated side
    (:func:`tests.cache.cached_side`: one build per key and engine version)."""

    def get(module: str = "single", t: float = 1.0, servo: str = servos.DEFAULT, **build: str):
        linkage = build.pop("linkage", "klann")
        cfg = BuildConfig(module=module, servo=servo, linkage=linkage, robot=False, **build)
        return cache.cached_side(cfg, t)

    return get


@pytest.fixture(scope="session")
def robot():
    """``robot(module, t, **build)``: the fabricated Klann robot (both sides and the chassis),
    the default constructions unless ``build`` names others
    (:func:`tests.cache.cached_robot`: one build per key and engine version)."""

    def get(module: str = "single", t: float = 1.0, **build: str):
        return cache.cached_robot(BuildConfig(linkage="klann", module=module, **build), t)

    return get


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="session")
def viewer_server() -> Iterator[str]:
    """Start the FastAPI server once per session and yield its base URL.

    E2E tests run against the built viewer bundle (``spiderpig/viewer/dist``, the
    package data a release ships) on a single port — `mise run test` invokes
    ``viewer-build`` first via the task ``depends`` chain, so the bundle is up to date.
    """
    dist = _REPO_ROOT / "spiderpig" / "viewer" / "dist"
    if not (dist / "index.html").is_file():
        raise RuntimeError(
            f"spiderpig/viewer/dist missing — run `mise run viewer-build` before e2e tests "
            f"(or use `mise run test`, which builds it for you). Looked at {dist}"
        )
    port = _free_port()
    proc = subprocess.Popen(  # noqa: S603
        [
            sys.executable, "-m", "uvicorn", "spiderpig.server.app:app",
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
