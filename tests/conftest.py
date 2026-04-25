"""Shared fixtures for the spiderpig test suite.

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

_REPO_ROOT = Path(__file__).resolve().parents[1]


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
