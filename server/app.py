"""FastAPI dev server for the Klann viewer.

Serves the three.js frontend and one self-contained ``klann_<mode>.glb``
per assembly mode from ``viewer/data/``, baked on first request (and again
when a cached file is older than the Python sources). A background watcher
re-runs the glTF bake for every cached mode whenever a source ``.py`` file
changes and pushes a ``reload`` message to every connected browser over
``/ws``.

Start via::

    mise run view          # FastAPI + Vite (dev, recommended)
    uv run uvicorn server.app:app   # FastAPI alone (serves built bundle)

The viewer is served from ``viewer/dist`` (Vite build output). Override
with ``SPIDERPIG_VIEWER_DIST`` if needed; falls back to ``viewer/`` when
no build exists so the API still works during initial setup.
"""

from __future__ import annotations

import os
import sys
import threading
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER_DIR = REPO_ROOT / "viewer"
DATA_DIR = VIEWER_DIR / "data"


def _viewer_static_dir() -> Path:
    override = os.environ.get("SPIDERPIG_VIEWER_DIST")
    if override:
        return Path(override).resolve()
    dist = VIEWER_DIR / "dist"
    return dist if dist.is_dir() else VIEWER_DIR

# User-facing mode id → bake_gltf mode argument. Order is the dropdown order.
# ``robot`` is both sides (quad per side); the rest are one side, under the
# ids old URLs use.
MODES: dict[str, str] = {
    "robot": "robot",
    "klann": "single",
    "double": "double",
    "decker": "decker",
    "double_double": "quad",
}

# Viewer-side helpers live under ``viewer/``; add to sys.path so we can
# import ``bake_gltf`` without it being a proper package.
if str(VIEWER_DIR) not in sys.path:
    sys.path.insert(0, str(VIEWER_DIR))

from bake_gltf import bake_gltf  # noqa: E402

from server.watcher import WatchBroadcaster, is_ignored_dir, is_source  # noqa: E402

_NO_CACHE = {"Cache-Control": "no-store"}
_DEFAULT_MODE = "robot"

# One bake at a time: requests run in a threadpool and the bake isn't reentrant.
_BAKE_LOCK = threading.Lock()


def _glb_path(mode_id: str) -> Path:
    return DATA_DIR / f"klann_{mode_id}.glb"


def _sources_mtime() -> float:
    """Newest mtime of the Python sources a bake depends on (what the watcher watches)."""
    newest = 0.0
    for dirpath, dirnames, filenames in os.walk(REPO_ROOT):
        rel = Path(dirpath).relative_to(REPO_ROOT)
        dirnames[:] = [d for d in dirnames if not is_ignored_dir(d)]
        for name in filenames:
            if is_source(rel / name):
                newest = max(newest, os.stat(os.path.join(dirpath, name)).st_mtime)
    return newest


def _is_fresh(path: Path) -> bool:
    return path.is_file() and path.stat().st_mtime >= _sources_mtime()


def _bake_mode(mode_id: str) -> None:
    bake_mode = MODES[mode_id]
    out = _glb_path(mode_id)
    print(f"[server] baking mode={mode_id} -> {out.name}")
    bake_gltf(out, mode=bake_mode, verbose=True)


def _ensure_baked(mode_id: str) -> Path:
    """The mode's ``.glb``, baked first when missing or older than the sources."""
    path = _glb_path(mode_id)
    with _BAKE_LOCK:
        if not _is_fresh(path):
            _bake_mode(mode_id)
    return path


def _ensure_default_baked() -> None:
    _ensure_baked(_DEFAULT_MODE)


def _rebake_all() -> None:
    """Called by the watcher. Re-bakes every mode that already has a cached
    ``.glb`` — newly-requested modes are baked lazily on first GET."""
    with _BAKE_LOCK:
        for mode_id in MODES:
            if _glb_path(mode_id).exists():
                _bake_mode(mode_id)


broadcaster = WatchBroadcaster(REPO_ROOT, rebake=_rebake_all)


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    _ensure_default_baked()
    await broadcaster.start()
    try:
        yield
    finally:
        await broadcaster.stop()


app = FastAPI(lifespan=_lifespan, title="spiderpig viewer")


@app.get("/api/modes")
def list_modes() -> dict:
    """Expose the mode catalogue so the frontend can build its toggle."""
    return {"default": _DEFAULT_MODE, "modes": list(MODES.keys())}


@app.get("/api/glb/{mode_id}")
def get_glb(mode_id: str) -> Response:
    if mode_id not in MODES:
        raise HTTPException(status_code=404, detail=f"unknown mode {mode_id!r}")
    path = _ensure_baked(mode_id)
    return FileResponse(path, media_type="model/gltf-binary", headers=_NO_CACHE)


@app.websocket("/ws")
async def ws(websocket: WebSocket) -> None:
    await websocket.accept()
    await broadcaster.connect(websocket)
    try:
        while True:
            # Client pings are optional; we just consume them.
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await broadcaster.disconnect(websocket)


# Mount static viewer last so /api and /ws take precedence.
app.mount("/", StaticFiles(directory=_viewer_static_dir(), html=True), name="viewer")
