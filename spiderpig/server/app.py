"""FastAPI server for the walker viewer (the dev server, and what ``spiderpig view`` runs).

Serves the three.js frontend and one self-contained ``.glb`` per design
from the project store's ``bakes/`` (:func:`spiderpig.bake.default_bake_dir`),
baked on first request (and again when a cached file is older than the
package's Python sources). A background watcher re-runs the glTF bake for
every default design it has baked whenever a source ``.py`` file of the
package changes and pushes a ``reload`` message to every connected browser
over ``/ws``.

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
    Every design is cached in the store's ``bakes/`` under its config's key (the
    newest ``CACHE_SIZE`` non-default ones kept). A design the planner or a
    construction can't build: 422 with the reason. Bakes run one at a time.
``?design=<id>`` on ``/api/walk`` and ``/api/glb``
    A design recorded in the store (:func:`configure`; ``spiderpig view``): its
    :class:`BuildConfig` from ``resolved.json`` (servo, sheet, constructions and
    fit included), with the query's ``module``, ``phases`` and ``p.NAME`` applied
    on top (:func:`_config_from_query`). Its own glb is served from the store's
    export when that is there and fresh, else baked (``X-Spiderpig-Glb``).
``GET /api/design/{id}``
    The design's card: linkage, module, sides and ``mode`` (``robot`` or ``side``),
    phases, proportions, materials, constructions, and its glb's URL.

Start via::

    mise run view                              # FastAPI + Vite (dev, recommended)
    uv run uvicorn spiderpig.server.app:app    # FastAPI alone (serves the built bundle)
    spiderpig view <design>                    # a stored design (spiderpig.view)

The viewer is served from ``spiderpig/viewer/dist`` (Vite's build output,
shipped as package data: :data:`VIEWER_DIST`). Override with
``SPIDERPIG_VIEWER_DIST`` if needed; without a build the API still works
and ``/`` says how to build it.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass, replace
from functools import lru_cache
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from spiderpig import api, linkage, walk
from spiderpig.bake import bake_gltf
from spiderpig.config import BuildConfig, design_from_query, parse_phases, parse_proportion
from spiderpig.server.watcher import WatchBroadcaster, is_ignored_dir, is_source
from spiderpig.store import Store, StoreError

log = logging.getLogger("server")
if not logging.root.handlers:       # under uvicorn, which configures only its own loggers
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("build123d").setLevel(logging.WARNING)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]      # the spiderpig package
VIEWER_DIST = PACKAGE_ROOT / "viewer" / "dist"           # the built viewer (package data)
CACHE_SIZE = 24     # non-default bakes kept on disk (newest first)


def viewer_dist_dir() -> Path | None:
    """The built viewer to serve: ``$SPIDERPIG_VIEWER_DIST``, else the package's
    ``viewer/dist``; ``None`` when neither holds an ``index.html``."""
    override = os.environ.get("SPIDERPIG_VIEWER_DIST")
    dist = Path(override).resolve() if override else VIEWER_DIST
    return dist if (dist / "index.html").is_file() else None


# What ``spiderpig view`` (and a test) sets before serving: the store ``?design=<id>``
# reads (``None``: the project store, ``$SPIDERPIG_STORE`` else ``./.spiderpig``) and
# whether the default robot is baked at startup (the dev server's habit).
_STORE: Store | None = None
_PREBAKE_DEFAULT = True


def configure(store: Store | str | Path | None = None,
              prebake_default: bool | None = None) -> None:
    """Point the app at ``store`` (``None``: the project store) and, with
    ``prebake_default``, say whether startup bakes the default robot."""
    global _STORE, _PREBAKE_DEFAULT
    _STORE = None if store is None else Store.of(store)
    if prebake_default is not None:
        _PREBAKE_DEFAULT = prebake_default


def store() -> Store:
    """The store designs are read from and bakes are cached in (``bakes/``)."""
    return _STORE if _STORE is not None else Store.default()


def _data_dir() -> Path:
    return store().root / "bakes"


def _design(design_id: str):
    """The recorded design behind ``?design=<id>`` (:func:`spiderpig.api.load`): 422 for
    a malformed id or a corrupt record, 404 for one the store doesn't hold."""
    try:
        return api.load(design_id, store())
    except ValueError as e:                     # "not a design id: ..."
        raise HTTPException(status_code=422, detail=str(e)) from None
    except StoreError as e:
        raise HTTPException(status_code=422, detail=str(e)) from None
    except KeyError:
        raise HTTPException(status_code=404, detail=f"no design {design_id!r} in "
                            f"{store().root.resolve()}") from None


def _config_from_query(query, **fixed) -> BuildConfig:
    """The config a query asks for (:func:`spiderpig.config.design_from_query`) or, with
    ``design=<id>``, the stored design's config with the query's ``module``, ``phases``
    and ``p.NAME`` applied on top: its servo, sheet, thickness, constructions and fit,
    which no query string expresses, stay. Another ``linkage`` starts from that
    linkage's defaults (its own parameters and phases) and keeps the materials and
    constructions. ``fixed`` (``robot``, a mode's ``module``) wins over both."""
    design_id = query.get("design")
    if not design_id:
        return design_from_query(query, **fixed)
    base = _design(design_id).config
    materials = {"sheet": base.sheet, "thickness": base.thickness, "servo": base.servo,
                 "pillar": base.pillar, "pin": base.pin, "crank": base.crank,
                 "params": base.params}
    if (query.get("linkage") or base.linkage) != base.linkage:
        return design_from_query(query, **materials, **fixed)
    items = query.multi_items() if hasattr(query, "multi_items") else query.items()
    props = dict(base.proportions)
    props.update(parse_proportion(f"{k[2:]}={v}") for k, v in items if k.startswith("p."))
    module = fixed.get("module") or query.get("module") or base.module
    if query.get("phases"):
        phases = parse_phases(query["phases"])
    else:
        phases = base.phases if module == base.module else None     # the module's own
    return replace(base, module=module, phases=phases, proportions=tuple(sorted(props.items())),
                   **{k: v for k, v in fixed.items() if k != "module"})


def _design_glb(design_id: str | None, config: BuildConfig) -> tuple[Path, str]:
    """A design's own glb from its store's export when it is there and fresh
    (``api.export(design, ["glb"])``), else a bake (:func:`_ensure_baked`); which one
    is the response's ``X-Spiderpig-Glb`` header."""
    if design_id:
        d = _design(design_id)
        path = d.store.exports_dir(d.id) / f"{config.linkage}.glb"
        if d.config == config and _is_fresh(path):
            return path, "export"
    return _ensure_baked(config), "bake"

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
    return _data_dir() / f"{config.key}.glb"


def _sources_mtime() -> float:
    """Newest mtime of the Python sources a bake depends on (what the watcher watches:
    the package)."""
    newest = 0.0
    for dirpath, dirnames, filenames in os.walk(PACKAGE_ROOT):
        rel = Path(dirpath).relative_to(PACKAGE_ROOT)
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

    files = sorted((p for p in _data_dir().glob("*.glb") if not is_default(p)),
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


broadcaster = WatchBroadcaster(PACKAGE_ROOT, rebake=_rebake_all)


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    _data_dir().mkdir(parents=True, exist_ok=True)
    if _PREBAKE_DEFAULT:
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


@app.get("/api/design/{design_id}")
def get_design(design_id: str) -> dict:
    """A stored design's card for the page (``?design=<id>``, ``spiderpig view``): what
    the tune panel seeds itself with (linkage, module, phases, proportions), its
    ``mode`` (``robot``, or ``side`` for a one-sided design), its materials and
    constructions, and ``glb``, the URL of its bake."""
    d = _design(design_id)
    cfg = d.config
    mode = "robot" if cfg.robot else "side"
    design = cfg.design_json()
    return {
        "design": d.id, "kind": d.kind, "linkage": design["linkage"], "module": design["module"],
        "sides": 2 if cfg.robot else 1, "mode": mode, "phases_deg": design["phases_deg"],
        "params": design["proportions"], "servo": cfg.servo, "sheet": cfg.sheet,
        "thickness_mm": cfg.thickness,
        "constructions": {"pillar": cfg.pillar, "pin": cfg.pin, "crank": cfg.crank},
        "engine_version": d.engine_version, "store": str(store().root.resolve()),
        "glb": f"/api/glb/{mode}?design={d.id}",
    }


@app.get("/api/walk")
def get_walk(request: Request) -> Response:
    """The walking model for a design, from the kinematics alone (see the module docstring)."""
    try:
        config = _config_from_query(request.query_params, robot=True)
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
        config = _config_from_query(request.query_params, robot=mode.robot,
                                    **({"module": mode.module} if mode.module else {}))
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e)) from None
    path, source = _design_glb(request.query_params.get("design"), config)
    return FileResponse(path, media_type="model/gltf-binary",
                        headers={**_NO_CACHE, "X-Spiderpig-Glb": source})


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


def mount_viewer(app: FastAPI) -> Path | None:
    """Serve the built viewer at ``/`` (mounted last so ``/api`` and ``/ws`` take
    precedence); without a build, ``/`` says how to make one. Returns what is served."""
    dist = viewer_dist_dir()
    if dist is not None:
        app.mount("/", StaticFiles(directory=dist, html=True), name="viewer")
    else:
        log.warning("no built viewer at %s: serving the API only (mise run viewer-build)",
                    VIEWER_DIST)

        @app.get("/")
        def no_viewer() -> Response:
            raise HTTPException(status_code=503, detail=(
                "the viewer isn't built: from a checkout run `mise run viewer-build` "
                "(a release wheel ships it), or point SPIDERPIG_VIEWER_DIST at a build"))
    return dist


mount_viewer(app)
