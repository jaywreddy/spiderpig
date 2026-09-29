"""FastAPI dev server for the walker viewer.

Serves the three.js frontend and one self-contained ``klann_<mode>.glb``
per assembly mode from ``viewer/data/``, baked on first request (and again
when a cached file is older than the Python sources). A background watcher
re-runs the glTF bake for every cached mode whenever a source ``.py`` file
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
    The fabricated walker baked with those parameters. The default design
    keeps ``viewer/data/klann_<mode>.glb``; other designs (other linkages
    too) are cached in ``viewer/data/params/`` per parameter set (the newest
    ``PARAM_CACHE_SIZE`` kept). A design the planner or a construction
    can't build: 422 with the reason. Bakes run one at a time.

Start via::

    mise run view          # FastAPI + Vite (dev, recommended)
    uv run uvicorn server.app:app   # FastAPI alone (serves built bundle)

The viewer is served from ``viewer/dist`` (Vite build output). Override
with ``SPIDERPIG_VIEWER_DIST`` if needed; falls back to ``viewer/`` when
no build exists so the API still works during initial setup.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
from contextlib import asynccontextmanager
from functools import lru_cache
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER_DIR = REPO_ROOT / "viewer"
DATA_DIR = VIEWER_DIR / "data"
PARAMS_DIR = DATA_DIR / "params"
PARAM_CACHE_SIZE = 24     # parameter bakes kept on disk (newest first)


def _viewer_static_dir() -> Path:
    override = os.environ.get("SPIDERPIG_VIEWER_DIST")
    if override:
        return Path(override).resolve()
    dist = VIEWER_DIR / "dist"
    return dist if dist.is_dir() else VIEWER_DIR

# User-facing mode id → bake_gltf mode argument. Order is the dropdown order.
# ``robot`` is both sides (quad per side unless ``module`` says otherwise); the
# rest are one side, under the ids old URLs use.
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

from bake_gltf import bake_gltf, build_config, is_default, param_glb  # noqa: E402

import linkage  # noqa: E402
import walk  # noqa: E402
from construction import ConstructionError  # noqa: E402
from fabricate import BuildConfig  # noqa: E402
from server.watcher import WatchBroadcaster, is_ignored_dir, is_source  # noqa: E402

_NO_CACHE = {"Cache-Control": "no-store"}
_DEFAULT_MODE = "robot"

# One bake at a time: requests run in a threadpool and the bake isn't reentrant.
_BAKE_LOCK = threading.Lock()
# Parameter sets that failed to build: glb path -> (sources mtime, reason).
_FAILED: dict[Path, tuple[float, str]] = {}


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


def _bake_to(path: Path, bake_mode: str, config: BuildConfig | None = None) -> None:
    """Bake into ``path`` atomically (a failed bake leaves no file behind)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.stem}.partial{path.suffix}")
    try:
        bake_gltf(tmp, mode=bake_mode, config=config, verbose=True)
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def _bake_mode(mode_id: str) -> None:
    bake_mode = MODES[mode_id]
    out = _glb_path(mode_id)
    print(f"[server] baking mode={mode_id} -> {out.name}")
    _bake_to(out, bake_mode)


def _ensure_baked(mode_id: str) -> Path:
    """The mode's ``.glb``, baked first when missing or older than the sources."""
    path = _glb_path(mode_id)
    with _BAKE_LOCK:
        if not _is_fresh(path):
            _bake_mode(mode_id)
    return path


def _prune_params(keep: int = PARAM_CACHE_SIZE) -> None:
    """Drop all but the ``keep`` newest parameter bakes."""
    files = sorted(PARAMS_DIR.glob("*.glb"), key=lambda p: p.stat().st_mtime, reverse=True)
    for old in files[keep:]:
        old.unlink(missing_ok=True)


def _ensure_param_baked(bake_mode: str, config: BuildConfig) -> Path:
    """The ``.glb`` of a non-default design, baked (and cached) on demand.

    422 when the linkage can't be assembled (checked first, from the
    kinematics alone), the planner finds no layout (``ValueError``) or a
    construction can't be built (:class:`construction.ConstructionError`).
    """
    try:
        walk.side_legs(config)
    except walk.LinkageError as e:
        raise HTTPException(status_code=422, detail=f"invalid linkage: {e}") from None
    path = param_glb(bake_mode, config, PARAMS_DIR)
    with _BAKE_LOCK:
        if _is_fresh(path):
            return path
        mtime = _sources_mtime()
        failed = _FAILED.get(path)
        if failed is not None and failed[0] >= mtime:
            raise HTTPException(status_code=422, detail=failed[1])
        print(f"[server] baking mode={bake_mode} params={walk.params_of(config)} "
              f"-> params/{path.name}")
        try:
            _bake_to(path, bake_mode, config)
        except (ValueError, ConstructionError) as e:
            reason = f"can't build this design: {e}"
            _FAILED[path] = (mtime, reason)
            raise HTTPException(status_code=422, detail=reason) from None
        _prune_params()
    return path


def _ensure_default_baked() -> None:
    _ensure_baked(_DEFAULT_MODE)


def _rebake_all() -> None:
    """Called by the watcher. Re-bakes every mode that already has a cached
    ``.glb`` — newly-requested modes are baked lazily on first GET, and
    parameter bakes when next requested (they're stale by then)."""
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


# ---------------------------------------------------------------------------
# Design parameters in the query string
# ---------------------------------------------------------------------------


def design_query(query) -> dict:
    """``linkage``, ``module``, ``phases`` (degrees) and ``proportions`` from a query string.

    ``linkage`` (a registered key, Klann by default), ``module``, ``phases``
    (comma-separated degrees, one per leg) and ``p.<NAME>=<value>``; other
    keys are ignored. Raises :class:`walk.ParamError` for malformed values
    and an unknown linkage (the rest is checked with the config).
    """
    proportions: dict[str, float] = {}
    for key, value in query.multi_items():
        if key.startswith("p."):
            name, v = walk.parse_proportion(f"{key[2:]}={value}")
            proportions[name] = v
    return {
        "linkage": walk.get_linkage(query.get("linkage") or linkage.DEFAULT).key,
        "module": query.get("module") or None,
        "phases": walk.parse_phases(query["phases"]) if query.get("phases") else None,
        "proportions": proportions,
    }


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
    }


@lru_cache(maxsize=64)
def _walk_json(config: BuildConfig) -> bytes:
    """``/api/walk``'s body for a (normalized) config, encoded once."""
    return json.dumps(walk.api_payload(config), allow_nan=False,
                      separators=(",", ":")).encode()


@app.get("/api/modes")
def list_modes() -> dict:
    """Expose the mode catalogue so the frontend can build its toggle."""
    return {"default": _DEFAULT_MODE, "modes": list(MODES.keys())}


@app.get("/api/linkages")
def list_linkages() -> dict:
    """The registered linkages (the viewer's linkage dropdown and parameter sliders)."""
    return {"default": linkage.DEFAULT,
            "linkages": [linkage_info(linkage.get(k)) for k in linkage.available()]}


@app.get("/api/walk")
def get_walk(request: Request) -> Response:
    """The walking model for a design, from the kinematics alone (see the module docstring)."""
    try:
        q = design_query(request.query_params)
        config = walk.make_config(q["module"] or "quad", q["phases"], q["proportions"],
                                  linkage=q["linkage"])
    except walk.ParamError as e:
        raise HTTPException(status_code=422, detail=str(e)) from None
    return Response(_walk_json(config), media_type="application/json", headers=_NO_CACHE)


@app.get("/api/glb/{mode_id}")
def get_glb(mode_id: str, request: Request) -> Response:
    if mode_id not in MODES:
        raise HTTPException(status_code=404, detail=f"unknown mode {mode_id!r}")
    bake_mode = MODES[mode_id]
    try:
        q = design_query(request.query_params)
        config = build_config(
            bake_mode, q["module"], linkage=q["linkage"],
            phases=None if q["phases"] is None else [math.radians(p) for p in q["phases"]],
            proportions=q["proportions"] or None)
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e)) from None
    if is_default(bake_mode, config):
        path = _ensure_baked(mode_id)
    else:
        path = _ensure_param_baked(bake_mode, config)
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
