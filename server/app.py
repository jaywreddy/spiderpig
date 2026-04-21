"""FastAPI dev server for the Klann viewer.

Serves the three.js frontend, the baked STL parts, and the pose JSON from
``viewer/data/``. A background watcher re-runs the bake whenever a source
``.py`` file changes and pushes a ``reload`` message to every connected
browser over ``/ws``.

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
PARTS_DIR = DATA_DIR / "parts"
FRAMES_JSON = DATA_DIR / "frames.json"

# Viewer-side helpers live under ``viewer/``; add to sys.path so we can
# import ``bake`` without it being a proper package.
if str(VIEWER_DIR) not in sys.path:
    sys.path.insert(0, str(VIEWER_DIR))

from bake import bake  # noqa: E402

from server.watcher import WatchBroadcaster  # noqa: E402

_NO_CACHE = {"Cache-Control": "no-store"}


def _ensure_baked() -> None:
    if FRAMES_JSON.exists() and PARTS_DIR.exists() and any(PARTS_DIR.glob("*.stl")):
        return
    print("[server] viewer/data missing — running initial bake …")
    bake(DATA_DIR, verbose=True)


def _rebake(parts: bool = True) -> None:
    """Called by the watcher. ``parts=False`` skips slow STL export."""
    bake(DATA_DIR, parts=parts, verbose=True)


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


@app.get("/api/frames")
def get_frames() -> FileResponse:
    return FileResponse(FRAMES_JSON, media_type="application/json", headers=_NO_CACHE)


@app.get("/api/parts/{name}")
def get_part(name: str) -> Response:
    if "/" in name or "\\" in name or not name.endswith(".stl"):
        return Response(status_code=400)
    path = PARTS_DIR / name
    if not path.is_file():
        return Response(status_code=404)
    return FileResponse(path, media_type="model/stl", headers=_NO_CACHE)


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
