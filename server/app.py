"""FastAPI dev server for the Klann viewer.

Serves the three.js frontend and a single self-contained ``klann.glb``
from ``viewer/data/``. A background watcher re-runs the glTF bake whenever
a source ``.py`` file changes and pushes a ``reload`` message to every
connected browser over ``/ws``.

Start via::

    uv run python cli.py view
"""

from __future__ import annotations

import sys
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER_DIR = REPO_ROOT / "viewer"
DATA_DIR = VIEWER_DIR / "data"
GLB_PATH = DATA_DIR / "klann.glb"

# Viewer-side helpers live under ``viewer/``; add to sys.path so we can
# import ``bake_gltf`` without it being a proper package.
if str(VIEWER_DIR) not in sys.path:
    sys.path.insert(0, str(VIEWER_DIR))

from bake_gltf import bake_gltf  # noqa: E402

from server.watcher import WatchBroadcaster  # noqa: E402

_NO_CACHE = {"Cache-Control": "no-store"}


def _ensure_baked() -> None:
    if GLB_PATH.exists():
        return
    print("[server] viewer/data/klann.glb missing — running initial bake …")
    bake_gltf(GLB_PATH, verbose=True)


def _rebake() -> None:
    """Called by the watcher."""
    bake_gltf(GLB_PATH, verbose=True)


broadcaster = WatchBroadcaster(REPO_ROOT, rebake=_rebake)


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    _ensure_baked()
    await broadcaster.start()
    try:
        yield
    finally:
        await broadcaster.stop()


app = FastAPI(lifespan=_lifespan, title="spiderpig viewer")


@app.get("/api/klann.glb")
def get_klann_glb() -> Response:
    if not GLB_PATH.is_file():
        return Response(status_code=404)
    return FileResponse(
        GLB_PATH, media_type="model/gltf-binary", headers=_NO_CACHE
    )


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
app.mount("/", StaticFiles(directory=VIEWER_DIR, html=True), name="viewer")
