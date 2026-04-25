"""Spawn FastAPI (uvicorn) and Vite side-by-side for `mise run view`.

FastAPI bakes .glb on source change; Vite serves the viewer with HMR and
proxies /api + /ws to FastAPI. Open http://localhost:5173.
"""

from __future__ import annotations

import contextlib
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER_DIR = REPO_ROOT / "viewer"

VIEWER_URL = "http://localhost:5173"
API_HOST_PORT = ("127.0.0.1", 8000)
WEB_HOST_PORT = ("localhost", 5173)
READY_TIMEOUT_S = 30.0
POLL_INTERVAL_S = 0.25


def _log(msg: str) -> None:
    print(f"[dev] {msg}", flush=True)


def _supports_unicode() -> bool:
    enc = (sys.stdout.encoding or "").lower()
    return enc.startswith("utf")


def _hyperlink(url: str, text: str) -> str:
    if not sys.stdout.isatty():
        return text
    return f"\x1b]8;;{url}\x1b\\{text}\x1b]8;;\x1b\\"


def _port_open(host: str, port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _print_banner() -> None:
    bar_char = "━" if _supports_unicode() else "-"
    arrow = "→" if _supports_unicode() else "->"
    bar = bar_char * 60
    link = _hyperlink(VIEWER_URL, VIEWER_URL)
    print(flush=True)
    print(bar, flush=True)
    print("  spiderpig viewer ready", flush=True)
    print(f"  {arrow} {link}    (click to open)", flush=True)
    print("  api: 127.0.0.1:8000  (proxied via vite)", flush=True)
    print(bar, flush=True)
    print(flush=True)


def _wait_then_announce(stop: threading.Event) -> None:
    deadline = time.monotonic() + READY_TIMEOUT_S
    while time.monotonic() < deadline and not stop.is_set():
        if _port_open(*API_HOST_PORT) and _port_open(*WEB_HOST_PORT):
            _print_banner()
            if os.environ.get("SPIDERPIG_NO_BROWSER") != "1":
                with contextlib.suppress(Exception):
                    webbrowser.open(VIEWER_URL, new=2)
            return
        time.sleep(POLL_INTERVAL_S)
    if not stop.is_set():
        _log(f"servers did not both come up within {READY_TIMEOUT_S:.0f}s; "
             f"open {VIEWER_URL} manually once ready")


def main() -> int:
    api_cmd = [
        sys.executable, "-u", "-m", "uvicorn", "server.app:app",
        "--host", "127.0.0.1", "--port", "8000",
    ]
    if sys.platform == "win32":
        web_cmd = ["npm.cmd", "run", "dev"]
    else:
        web_cmd = ["npm", "run", "dev"]

    # On Windows, mise's task runner inherits handles in a way that can
    # block uvicorn at startup. Detach the api child into a new process
    # group so it has clean stdio. (npm.cmd / Vite need to keep the
    # inherited console, so don't apply this to web.)
    api_kwargs: dict = {}
    if sys.platform == "win32":
        api_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

    _log(f"starting FastAPI: {' '.join(api_cmd)}")
    api = subprocess.Popen(api_cmd, cwd=REPO_ROOT, **api_kwargs)
    _log(f"starting Vite: {' '.join(web_cmd)} (cwd={VIEWER_DIR})")
    web = subprocess.Popen(web_cmd, cwd=VIEWER_DIR)

    _log(f"FastAPI pid={api.pid} :8000  Vite pid={web.pid} :5173")

    ready_stop = threading.Event()
    ready_thread = threading.Thread(
        target=_wait_then_announce, args=(ready_stop,), daemon=True
    )
    ready_thread.start()

    def shutdown(*_: object) -> None:
        ready_stop.set()
        for p in (web, api):
            if p.poll() is None:
                with contextlib.suppress(Exception):
                    p.terminate()

    signal.signal(signal.SIGINT, shutdown)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, shutdown)

    try:
        while True:
            api_rc = api.poll()
            web_rc = web.poll()
            if api_rc is not None:
                _log(f"api exited rc={api_rc}; shutting down web")
                shutdown()
                with contextlib.suppress(subprocess.TimeoutExpired):
                    web.wait(timeout=5)
                return api_rc
            if web_rc is not None:
                _log(f"web exited rc={web_rc}; shutting down api")
                shutdown()
                with contextlib.suppress(subprocess.TimeoutExpired):
                    api.wait(timeout=5)
                return web_rc
            with contextlib.suppress(subprocess.TimeoutExpired):
                api.wait(timeout=0.5)
    except KeyboardInterrupt:
        shutdown()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
