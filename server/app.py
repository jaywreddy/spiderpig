"""FastAPI dev server for the walker viewer.

Serves the three.js frontend and one self-contained ``.glb`` per design
from ``viewer/data/``, baked on first request (and again when a cached file
is older than the Python sources). A background watcher re-runs the glTF
bake for every default design it has baked whenever a source ``.py`` file
changes and pushes a ``reload`` message to every connected browser over
``/ws``.

Design parameters (the walking model's contract, see ``walk.py``):

``GET /api/linkages``
    Every registered linkage (:mod:`linkage`): key, name, family, notes,
    source, its parameters (name, default, whether it's an angle), its
    modules (name -> legs per side), link labels and feet per leg.
``GET /api/walk?linkage=klann&module=quad&phases=0,180,90,270&p.OB=1.121``
    The walking model from the kinematics alone (no parts; fast once the
    linkage's default design is planned): every foot's path, one side's
    joints, the nominal centre of mass and the straight-walk metrics.
    ``linkage`` defaults to Klann; ``phases`` in degrees, one per leg;
    ``p.<NAME>`` overrides one of the linkage's parameters. Bad parameters
    (an unknown linkage too): 422. A linkage that can't be assembled: 200
    with ``valid: false`` and the reason.
``GET /api/glb/{mode}?linkage=...&module=...&phases=...&p.NAME=...``
    The fabricated walker baked with those parameters: ``robot`` (both
    sides, ``module`` legs per side) or ``side`` (one side of ``module``);
    the ids old URLs use (``klann``, ``double``, ``decker``,
    ``double_double``) are one side of that module (:data:`MODES`;
    ``/api/modes`` lists the ids the viewer's dropdown offers, with labels).
    Every design is cached in ``viewer/data/`` under its config's key (the
    newest ``CACHE_SIZE`` non-default ones kept). A design the planner or a
    construction can't build: 422 with the reason. Bakes run one at a time.

Start via::

    mise run view          # FastAPI + Vite (dev, recommended)
    uv run uvicorn server.app:app   # FastAPI alone (serves built bundle)

The viewer is served from ``viewer/dist`` (Vite build output). Override
with ``SPIDERPIG_VIEWER_DIST`` if needed; falls back to ``viewer/`` when
no build exists so the API still works during initial setup.
"""

from __future__ import annotations

import json
import logging
import os
import sys
import threading
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass
from functools import lru_cache
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

log = logging.getLogger("server")
if not logging.root.handlers:       # under uvicorn, which configures only its own loggers
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("build123d").setLevel(logging.WARNING)

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER_DIR = REPO_ROOT / "viewer"
DATA_DIR = VIEWER_DIR / "data"
CACHE_SIZE = 24     # non-default bakes kept on disk (newest first)


def _viewer_static_dir() -> Path:
    override = os.environ.get("SPIDERPIG_VIEWER_DIST")
    if override:
        return Path(override).resolve()
    dist = VIEWER_DIR / "dist"
    return dist if dist.is_dir() else VIEWER_DIR


# Viewer-side helpers live under ``viewer/``; add to sys.path so we can
# import ``bake_gltf`` without it being a proper package.
if str(VIEWER_DIR) not in sys.path:
    sys.path.insert(0, str(VIEWER_DIR))

from bake_gltf import bake_gltf  # noqa: E402

import linkage  # noqa: E402
import walk  # noqa: E402
from config import BuildConfig, design_from_query  # noqa: E402
from server.watcher import WatchBroadcaster, is_ignored_dir, is_source  # noqa: E402

_NO_CACHE = {"Cache-Control": "no-store"}
_DEFAULT_MODE = "robot"


@dataclass(frozen=True)
class Mode:
    """What a ``/api/glb/{mode}`` id builds: ``module`` (``None``: the query's, quad by
    default) as the robot or one side; ``label`` puts it in the viewer's dropdown."""

    module: str | None
    robot: bool
    label: str | None = None


# URL ids, in the dropdown's order; the side-only ids are the ones old URLs use.
MODES: dict[str, Mode] = {
    "robot": Mode(None, True, "robot"),
    "klann": Mode("single", False, "single (one leg)"),
    "double": Mode("double", False, "double"),
    "decker": Mode("decker", False, "decker"),
    "double_double": Mode("quad", False, "double double"),
    "side": Mode(None, False),
}

# One bake at a time: requests run in a threadpool and the bake isn't reentrant.
_BAKE_LOCK = threading.Lock()
# Designs that failed to build: glb path -> (sources mtime, reason).
_FAILED: dict[Path, tuple[float, str]] = {}
# Every design this server has baked, by its file (the watcher re-bakes the defaults).
_BAKED: dict[Path, BuildConfig] = {}


def _glb_path(config: BuildConfig) -> Path:
    return DATA_DIR / f"{config.key}.glb"


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


def _bake(config: BuildConfig) -> Path:
    """Bake ``config`` into its file atomically (a failed bake leaves no file behind)."""
    path = _glb_path(config)
    log.info("baking %s -> %s", config.design_json(), path.name)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.stem}.partial{path.suffix}")
    try:
        bake_gltf(tmp, config)
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)
    return path


def _prune(keep: int = CACHE_SIZE) -> None:
    """Drop all but the ``keep`` newest non-default bakes (a file this server didn't bake
    counts as one)."""
    def is_default(p: Path) -> bool:
        config = _BAKED.get(p)
        return config is not None and config.is_default

    files = sorted((p for p in DATA_DIR.glob("*.glb") if not is_default(p)),
                   key=lambda p: p.stat().st_mtime, reverse=True)
    for old in files[keep:]:
        old.unlink(missing_ok=True)
        _BAKED.pop(old, None)


def _ensure_baked(config: BuildConfig) -> Path:
    """The design's ``.glb``, baked first when missing or older than the sources.

    422 when the linkage can't be assembled (checked first, from the
    kinematics alone), the planner finds no layout or a construction can't
    be built (both ``ValueError``); a failed design is remembered until the
    sources change.
    """
    if config.robot:
        try:
            walk.side_legs(config)
        except walk.LinkageError as e:
            raise HTTPException(status_code=422, detail=f"invalid linkage: {e}") from None
    path = _glb_path(config)
    with _BAKE_LOCK:
        _BAKED[path] = config
        if _is_fresh(path):
            return path
        mtime = _sources_mtime()
        failed = _FAILED.get(path)
        if failed is not None and failed[0] >= mtime:
            raise HTTPException(status_code=422, detail=failed[1])
        try:
            _bake(config)
        except ValueError as e:
            reason = f"can't build this design: {e}"
            _FAILED[path] = (mtime, reason)
            raise HTTPException(status_code=422, detail=reason) from None
        if not config.is_default:
            _prune()
    return path


def _rebake_all() -> None:
    """Called by the watcher. Re-bakes every default design this server has baked; other
    designs are baked again when next requested (they're stale by then)."""
    with _BAKE_LOCK:
        for path, config in list(_BAKED.items()):
            if config.is_default and path.exists():
                _bake(config)


broadcaster = WatchBroadcaster(REPO_ROOT, rebake=_rebake_all)


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    _ensure_baked(BuildConfig())
    await broadcaster.start()
    try:
        yield
    finally:
        await broadcaster.stop()


app = FastAPI(lifespan=_lifespan, title="spiderpig viewer")


def linkage_info(lk: linkage.Linkage) -> dict:
    """What the viewer needs to offer and tune a linkage (``/api/linkages``)."""
    return {
        "key": lk.key, "name": lk.name, "family": lk.family or lk.key, "notes": lk.notes,
        "source": lk.source,
        "params": [{"name": k, "default": float(v), "angle": k in lk.angles}
                   for k, v in lk.params.items()],
        "modules": {m: len(legs) for m, legs in lk.leg_modules.items()},
        "labels": dict(lk.labels),
        "feet": len(lk.feet),
        "kind": lk.kind,
        "output": asdict(lk.output) if lk.output else None,
    }


@lru_cache(maxsize=64)
def _walk_json(config: BuildConfig) -> bytes:
    """``/api/walk``'s body for a (normalized) config, encoded once."""
    return json.dumps(walk.api_payload(config), allow_nan=False,
                      separators=(",", ":")).encode()


@app.get("/api/modes")
def list_modes() -> dict:
    """The ids the viewer's dropdown offers (in order) and their labels."""
    offered = {k: m.label for k, m in MODES.items() if m.label}
    return {"default": _DEFAULT_MODE, "modes": list(offered), "labels": offered}


@app.get("/api/linkages")
def list_linkages() -> dict:
    """The registered linkages (the viewer's walker dropdown and parameter sliders, and its
    mechanism picker: ``kind`` tells them apart)."""
    return {"default": linkage.DEFAULT,
            "linkages": [linkage_info(linkage.get(k)) for k in linkage.available()]}


@app.get("/api/walk")
def get_walk(request: Request) -> Response:
    """The walking model for a design, from the kinematics alone (see the module docstring)."""
    try:
        config = design_from_query(request.query_params, robot=True)
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e)) from None
    return Response(_walk_json(config), media_type="application/json", headers=_NO_CACHE)


@app.get("/api/glb/{mode_id}")
def get_glb(mode_id: str, request: Request) -> Response:
    if mode_id not in MODES:
        raise HTTPException(status_code=404, detail=f"unknown mode {mode_id!r}")
    mode = MODES[mode_id]
    asked = request.query_params.get("module")
    if mode.module is not None and asked not in (None, mode.module):
        raise HTTPException(status_code=422, detail=f"mode {mode_id!r} is one side of its "
                            f"module; module {asked!r} applies to robot or side only")
    try:
        config = design_from_query(request.query_params, robot=mode.robot,
                                   **({"module": mode.module} if mode.module else {}))
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e)) from None
    return FileResponse(_ensure_baked(config), media_type="model/gltf-binary",
                        headers=_NO_CACHE)


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
