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
    modules (name -> legs per side) and ``default_module``, link labels and feet per leg.
``GET /api/walk?linkage=klann&module=quad&phases=0,180,90,270&p.OB=1.121``
    The walking model from the kinematics alone (no parts; fast once the
    linkage's default design is planned): every foot's path, one side's
    joints, the nominal centre of mass and the straight-walk metrics.
    ``linkage`` defaults to Strider, ``module`` to the linkage's (Strider's
    ``double``; :func:`spiderpig.config.default_module`); ``phases`` in degrees, one per leg;
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
``WS /ws/sim?linkage=...&module=...``
    The robot of that design driven live in MuJoCo (:mod:`spiderpig.sim.live`): see
    :func:`ws_sim`.

Layer plans go through the store (``api.plan_config``): a design that planned once
is reused, never searched for again; one whose planner budget ran out answers 422
without being remembered as unbuildable.

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

import asyncio
import json
import logging
import math
import os
import threading
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass, replace
from functools import lru_cache
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketState

from spiderpig import api, linkage, walk, workers
from spiderpig.bake import bake_gltf
from spiderpig.config import (
    BuildConfig,
    default_module,
    design_from_query,
)
from spiderpig.server.watcher import WatchBroadcaster, is_ignored_dir, is_source
from spiderpig.store import Store

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
    except ValueError as e:     # "not a design id: ...", or a corrupt record (StoreError)
        raise HTTPException(status_code=422, detail=str(e)) from None
    except KeyError:
        raise HTTPException(status_code=404, detail=f"no design {design_id!r} in "
                            f"{store().root.resolve()}") from None


def _query_design(query, **fixed):
    """``(config, design)`` a query asks for: :func:`spiderpig.config.design_from_query`,
    on top of the stored design ``design=<id>`` names (its servo, sheets, constructions
    and fit stay; ``design`` is its handle, loaded once), else ``(config, None)``.
    ``fixed`` (``robot``, a mode's ``module``) wins over both."""
    design_id = query.get("design")
    d = _design(design_id) if design_id else None
    return design_from_query(query, base=d.config if d else None, **fixed), d


def _config_from_query(query, **fixed) -> BuildConfig:
    """The config alone (:func:`_query_design`)."""
    return _query_design(query, **fixed)[0]


def _design_glb(d, config: BuildConfig, gone=None) -> tuple[Path, str]:
    """A design's own glb from its store's export when it is there and fresh
    (``api.export(design, ["glb"])``; ``d`` the stored design's handle, or ``None``), else
    a bake (:func:`_ensure_baked`); which one is the response's ``X-Spiderpig-Glb``
    header."""
    if d is not None:
        path = d.store.exports_dir(d.id) / f"{config.linkage}.glb"
        if d.config == config and _is_fresh(path):
            return path, "export"
    return _ensure_baked(config, gone=gone), "bake"

_NO_CACHE = {"Cache-Control": "no-store"}
_DEFAULT_MODE = "robot"


@dataclass(frozen=True)
class Mode:
    """What a ``/api/glb/{mode}`` id builds: ``module`` (``None``: the query's, the
    linkage's default when it has none) as the robot or one side; ``label`` puts it in the
    viewer's dropdown."""

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
# Designs that failed to build: glb path -> (sources mtime, reason), oldest first (at most
# CACHE_SIZE: :func:`_prune`).
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


_LOADED_AT = _sources_mtime()
"""The sources as this process imported them: after an edit (the dev server's watcher) its
modules are the old code, so a bake or a walk runs in a fresh worker process instead
(:func:`_stale_code`), which imports the new."""


def _stale_code() -> bool:
    """Have the sources changed since this process imported them (and may a worker
    process run the new code: :func:`spiderpig.workers.enabled`)?"""
    return workers.enabled() and _sources_mtime() > _LOADED_AT


def _is_fresh(path: Path) -> bool:
    return path.is_file() and path.stat().st_mtime >= _sources_mtime()


def _bake_job(config: BuildConfig, store_root: str, out: str) -> tuple[int, bool, str]:
    """:func:`_bake`'s work (the plan through the store, then the bake into ``out``), for a
    worker process: ``(layers, optimal, proof)``."""
    side = api.plan_config(config, Store.of(store_root))
    bake_gltf(Path(out), config)
    return side.plan.top + 1, bool(side.plan.optimal), str(side.plan.proof)


def _bake(config: BuildConfig) -> Path:
    """Bake ``config`` into its file atomically (a failed bake leaves no file behind). The
    layer plan goes through the store first (:func:`spiderpig.api.plan_config`, as the
    bake CLI does): a design that planned once is never searched for again, so a later
    bake can't run out of the planner's budget on it."""
    path = _glb_path(config)
    log.info("baking %s -> %s", config.design_json(), path.name)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.stem}.partial{path.suffix}")
    try:
        if _stale_code():       # the sources changed under this process: bake the new code
            layers, optimal, proof = workers.submit(_bake_job, config, str(store().root),
                                                    str(tmp)).result()
        else:
            layers, optimal, proof = _bake_job(config, str(store().root), str(tmp))
        log.info("plan for %s: %d layers, %s", config.key, layers,
                 "proven optimal" if optimal else f"not proven optimal ({proof})")
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)
    return path


def _prune(keep: int = CACHE_SIZE) -> None:
    """Drop all but the ``keep`` newest non-default bakes (a file this server didn't bake
    counts as one), and all but the ``keep`` newest remembered failures (those older than
    the sources first: they no longer answer). Called under :data:`_BAKE_LOCK`."""
    def is_default(p: Path) -> bool:
        config = _BAKED.get(p)
        return config is not None and config.is_default

    files = sorted((p for p in _data_dir().glob("*.glb") if not is_default(p)),
                   key=lambda p: p.stat().st_mtime, reverse=True)
    for old in files[keep:]:
        old.unlink(missing_ok=True)
        _BAKED.pop(old, None)
    if _FAILED:
        mtime = _sources_mtime()
        for path in [p for p, (at, _) in _FAILED.items() if at < mtime]:
            del _FAILED[path]
        while len(_FAILED) > keep:
            del _FAILED[next(iter(_FAILED))]       # the oldest (insertion order)
        for path in [p for p, c in _BAKED.items() if p not in _FAILED and not p.exists()
                     and not c.is_default]:
            del _BAKED[path]                       # a failed design's entry: no file


class ClientGone(Exception):
    """The client that asked for a bake disconnected while it waited for its turn."""


BAKE_POLL_S = 0.25      # how often a request queued for the bake lock checks its client


def _acquire_bake_lock(gone=None) -> None:
    """Take :data:`_BAKE_LOCK`; with ``gone`` (a callable: has the client disconnected?),
    give up with :class:`ClientGone` when it says so while the request waits its turn."""
    if gone is None:
        _BAKE_LOCK.acquire()
        return
    while not _BAKE_LOCK.acquire(timeout=BAKE_POLL_S):
        if gone():
            raise ClientGone
    if gone():                  # left while the previous bake ran: don't bake for no one
        _BAKE_LOCK.release()
        raise ClientGone


def _ensure_baked(config: BuildConfig, gone=None) -> Path:
    """The design's ``.glb``, baked first when missing or older than the sources.

    422 when the linkage can't be assembled (checked first, from the
    kinematics alone), the planner finds no layout or a construction can't
    be built (both ``ValueError``); a failed design is remembered until the
    sources change, except one whose planner budget ran out
    (:class:`spiderpig.api.PlanTimeout`: a loaded machine, not the design; the
    422 says so and the next request plans again).

    Bakes run one at a time (:data:`_BAKE_LOCK`). ``gone`` (a callable: has the client
    disconnected?) lets a request queued behind another bake give up when its client has
    left (:class:`ClientGone`), so a viewer that asked for one design after another bakes
    only the last; a bake already running finishes (OCCT can't be interrupted cleanly) and
    its file serves the next request. Two requests for one design are one bake: the second
    finds the first's file fresh.
    """
    if config.robot:
        try:
            walk.side_legs(config)
        except walk.LinkageError as e:
            raise HTTPException(status_code=422, detail=f"invalid linkage: {e}") from None
    path = _glb_path(config)
    _acquire_bake_lock(gone)
    try:
        _BAKED[path] = config
        if _is_fresh(path):
            return path
        mtime = _sources_mtime()
        failed = _FAILED.get(path)
        if failed is not None and failed[0] >= mtime:
            raise HTTPException(status_code=422, detail=failed[1])
        try:
            _bake(config)
        except api.PlanTimeout as e:
            raise HTTPException(status_code=422, detail=(
                f"can't build this design yet: {e} (the planner's CPU budget ran out, which "
                "says the machine was busy, not that the design has no plan: not remembered, "
                "ask again)")) from None
        except ValueError as e:
            reason = f"can't build this design: {e}"
            _FAILED.pop(path, None)
            _FAILED[path] = (mtime, reason)             # the newest last
            _prune()
            raise HTTPException(status_code=422, detail=reason) from None
        if not config.is_default:
            _prune()
    finally:
        _BAKE_LOCK.release()
    return path


# The event loop serving the app (set at startup): the live sessions' state
# (:data:`_SIM_BUILDS`, :data:`_SIM_GENERATION`) belongs to it, so a thread changes it
# through ``call_soon_threadsafe`` (:func:`_on_loop`).
_LOOP: asyncio.AbstractEventLoop | None = None


def _on_loop(fn) -> None:
    """Run ``fn`` on the app's event loop: now when called on it (or with no loop
    running), else scheduled there from this thread."""
    loop = _LOOP
    if loop is None or loop.is_closed() or not loop.is_running():
        fn()
        return
    try:
        running = asyncio.get_running_loop()
    except RuntimeError:
        running = None
    if running is loop:
        fn()
    else:
        loop.call_soon_threadsafe(fn)


def _outdate_sims() -> None:
    """Forget the live models being built and move the generation on (every live session
    started before closes as stale). On the event loop only."""
    global _SIM_GENERATION
    _SIM_BUILDS.clear()
    _SIM_GENERATION += 1


def _rebake_all() -> None:
    """Called by the watcher (in a worker thread). Re-bakes every default design this
    server has baked; other designs are baked again when next requested (they're stale by
    then). The MuJoCo models are forgotten too (:func:`spiderpig.sim.mjcf.clear_caches`)
    and every live session is told it is stale (it closes; the viewer reconnects to a fresh
    model): that state is the event loop's, so it changes there (:func:`_on_loop`)."""
    with _BAKE_LOCK:
        from spiderpig.sim import mjcf

        mjcf.clear_caches()
        _walk_json.cache_clear()
        _on_loop(_outdate_sims)
        for path, config in list(_BAKED.items()):
            if config.is_default and path.exists():
                _bake(config)


broadcaster = WatchBroadcaster(PACKAGE_ROOT, rebake=_rebake_all)


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    global _LOOP
    _LOOP = asyncio.get_running_loop()
    _data_dir().mkdir(parents=True, exist_ok=True)
    if _PREBAKE_DEFAULT:
        _ensure_baked(BuildConfig())
    await broadcaster.start()
    try:
        yield
    finally:
        await broadcaster.stop()
        _LOOP = None


# ---------------------------------------------------------------------------
# Who may talk to the server: the Host header (DNS rebinding) and a WebSocket's Origin
# ---------------------------------------------------------------------------

ALLOWED_HOSTS_ENV = "VITE_ALLOWED_HOSTS"
LOOPBACK_HOSTS = ("localhost", "127.0.0.1", "::1")


def _hostname(netloc: str) -> str:
    """The host of a ``Host`` header or an origin's netloc, lower case, without its port
    (``[::1]:8000`` -> ``::1``)."""
    netloc = netloc.strip().lower()
    if netloc.startswith("["):                      # an IPv6 literal, maybe with a port
        return netloc[1:netloc.find("]")] if "]" in netloc else netloc[1:]
    if netloc.count(":") == 1:
        return netloc.partition(":")[0]
    return netloc                                   # a bare IPv6 address, or no port


_SERVED_AS: list[str] = []     # names the server was started under (allow_host)


def allow_host(name: str) -> None:
    """Answer ``name`` too: the host ``spiderpig view --host NAME`` binds and prints."""
    name = _hostname(name)
    if name and name not in _SERVED_AS:
        _SERVED_AS.append(name)


def allowed_hosts() -> list[str]:
    """The host names this server answers: the loopback names, the one it was started
    under (:func:`allow_host`) and every ``$VITE_ALLOWED_HOSTS`` entry (comma-separated, as
    Vite reads it; a leading ``.`` allows the domain and its subdomains, ``.ts.net``
    behind ``tailscale serve``)."""
    extra = [h.strip().lower() for h in os.environ.get(ALLOWED_HOSTS_ENV, "").split(",")]
    return [*LOOPBACK_HOSTS, *_SERVED_AS, *(h for h in extra if h)]


def _is_ip(host: str) -> bool:
    import ipaddress

    try:
        ipaddress.ip_address(host)
    except ValueError:
        return False
    return True


def _named(host: str, allowed: list[str]) -> bool:
    """Is ``host`` one of ``allowed`` (``.example.com``: it or a subdomain;
    ``localhost`` covers ``*.localhost``)?"""
    for pattern in allowed:
        if pattern.startswith("."):
            if host == pattern[1:] or host.endswith(pattern):
                return True
        elif host == pattern or (pattern == "localhost" and host.endswith(".localhost")):
            return True
    return False


def host_allowed(host_header: str) -> bool:
    """May a request with this ``Host`` be served? A loopback or allowed name
    (:func:`allowed_hosts`), or any IP address: a DNS-rebinding page reaches the server
    under its own domain name, never under an address (Vite's rule too)."""
    host = _hostname(host_header)
    return bool(host) and (_is_ip(host) or _named(host, allowed_hosts()))


DEV_ORIGIN_PORT_ENV = "SPIDERPIG_DEV_ORIGIN_PORT"
"""The Vite dev server's port (``spiderpig.tools.dev`` sets it for the API process): a
loopback page on that port is the viewer, whatever ``Host`` its WebSocket arrives with."""


def _port(netloc: str, scheme: str = "") -> str:
    """A netloc's port, the scheme's default when it names none (``""``: no scheme, no
    port)."""
    netloc = netloc.strip().lower()
    tail = netloc.rsplit("]", 1)[-1] if netloc.startswith("[") else netloc
    if tail.count(":") == 1:
        return tail.rpartition(":")[2]
    return {"http": "80", "https": "443"}.get(scheme, "")


def _loopback(host: str) -> bool:
    return host in LOOPBACK_HOSTS or host.endswith(".localhost")


def origin_allowed(origin: str | None, host_header: str, scheme: str = "ws") -> bool:
    """May a WebSocket from this ``Origin`` connect? Browsers don't apply CORS to
    WebSockets, so without this any page could drive ``/ws/sim``. Allowed:

    - no Origin (not a browser);
    - the page's own host *and port*: the ``Host`` it connects to (``spiderpig view``, or
      Vite's ``/ws`` proxy, which passes Host on). A Host naming no port matches the
      default port of the Origin's scheme (``https://box.ts.net`` through a
      TLS-terminating ``tailscale serve``, which speaks ``ws`` to us);
    - the Vite dev page: an Origin on ``$SPIDERPIG_DEV_ORIGIN_PORT`` (set by
      ``spiderpig.tools.dev`` for the API it starts, and stripped by ``spiderpig view``),
      on a loopback or a ``$VITE_ALLOWED_HOSTS`` name;
    - a ``$VITE_ALLOWED_HOSTS`` name (not a loopback one) on the request's own port.

    A page on any other port is refused, a loopback one above all (any local dev
    server's). ``scheme`` (the request's) is kept for callers; the ports compared are
    the Origin's and the Host's."""
    if origin is None:
        return True
    from urllib.parse import urlsplit

    try:
        parts = urlsplit(origin.strip())
    except ValueError:
        return False
    if parts.scheme not in ("http", "https") or not parts.netloc:
        return False                                # "null" (a sandbox, a file), garbage
    host, port = _hostname(parts.netloc), _port(parts.netloc, parts.scheme)
    want = _port(host_header) or _port("", parts.scheme)
    if host == _hostname(host_header) and port == want:
        return True
    extra = [h for h in (x.strip().lower() for x in
                         os.environ.get(ALLOWED_HOSTS_ENV, "").split(",")) if h]
    named = _named(host, extra)
    dev = os.environ.get(DEV_ORIGIN_PORT_ENV, "").strip()
    if dev and port == dev and (_loopback(host) or named):
        return True
    return named and not _loopback(host) and port == want


class HostGuard:
    """ASGI middleware: a request whose ``Host`` isn't allowed gets a 400 (an HTTP one) or
    a refused handshake (a WebSocket, 403), and so does a WebSocket whose ``Origin`` is a
    foreign page's (:func:`host_allowed`, :func:`origin_allowed`).

    Starlette's ``TrustedHostMiddleware`` does the first, but reads ``[::1]:8000`` as host
    ``[`` and answers a WebSocket with an HTTP response; this one parses IPv6 and closes
    the handshake, and reads the allowed hosts per request (``$VITE_ALLOWED_HOSTS``)."""

    def __init__(self, app) -> None:
        self.app = app

    async def __call__(self, scope, receive, send) -> None:
        kind = scope["type"]
        if kind not in ("http", "websocket"):
            await self.app(scope, receive, send)
            return
        headers = {k.decode("latin-1").lower(): v.decode("latin-1")
                   for k, v in scope.get("headers", [])}
        host = headers.get("host", "")
        ok = host_allowed(host)
        why = f"Host {host!r} is not allowed (add it to ${ALLOWED_HOSTS_ENV})"
        if ok and kind == "websocket" and not origin_allowed(
                headers.get("origin"), host, scope.get("scheme", "ws")):
            ok, why = False, f"Origin {headers.get('origin')!r} is not allowed"
        if ok:
            await self.app(scope, receive, send)
            return
        log.warning("refused %s %s: %s", kind, scope.get("path"), why)
        if kind == "http":
            from starlette.responses import PlainTextResponse

            await PlainTextResponse(f"Invalid host header: {why}", status_code=400)(
                scope, receive, send)
        else:
            await receive()                         # websocket.connect
            await send({"type": "websocket.close", "code": 1008})


app = FastAPI(lifespan=_lifespan, title="spiderpig viewer")
app.add_middleware(HostGuard)


def linkage_info(lk: linkage.Linkage) -> dict:
    """What the viewer needs to offer and tune a linkage (``/api/linkages``)."""
    return {
        "key": lk.key, "name": lk.name, "family": lk.family or lk.key, "notes": lk.notes,
        "source": lk.source,
        "params": [{"name": k, "default": float(v), "angle": k in lk.angles}
                   for k, v in lk.params.items()],
        "modules": {m: len(legs) for m, legs in lk.leg_modules.items()},
        "default_module": default_module(lk.key),
        "labels": dict(lk.labels),
        "feet": len(lk.feet),
        "kind": lk.kind,
        "output": asdict(lk.output) if lk.output else None,
    }


@lru_cache(maxsize=64)
def _walk_json(config: BuildConfig) -> bytes:
    """``/api/walk``'s body for a (normalized) config, encoded once (in a worker process
    once the sources changed under this one: :func:`_stale_code`; the cache is cleared on
    every change, :func:`_rebake_all`)."""
    payload = (workers.submit(walk.api_payload, config).result() if _stale_code()
               else walk.api_payload(config))
    return json.dumps(payload, allow_nan=False, separators=(",", ":")).encode()


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
        # the build's cut-rule review (spiderpig.manufacture): ok, errors / warnings per
        # rule, one message each with why and the fix; None until the design is built
        "cut_rules": api.cut_rules(d),
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
        config, d = _query_design(request.query_params, robot=mode.robot,
                                  **({"module": mode.module} if mode.module else {}))
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e)) from None
    try:
        path, source = _design_glb(d, config, gone=_disconnected(request))
    except ClientGone:
        log.info("glb: the client left before its bake of %s started: skipped", config.key)
        return Response(status_code=499)            # nobody reads it (nginx's "client closed")
    return FileResponse(path, media_type="model/gltf-binary",
                        headers={**_NO_CACHE, "X-Spiderpig-Glb": source})


def _disconnected(request: Request):
    """A callable for a sync handler's thread: has ``request``'s client disconnected?
    (``Request.is_disconnected`` run on the event loop; ``False`` when it can't tell.)"""
    import anyio.from_thread

    def gone() -> bool:
        try:
            return bool(anyio.from_thread.run(request.is_disconnected))
        except RuntimeError:        # not in an anyio worker thread (a direct call)
            return False

    return gone


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


SIM_FPS = 60
SIM_BUILD_PING_S = 1.0      # how often a client building a model hears the elapsed time
# Live sessions at once. Each costs a 60 Hz physics thread; measured at load ~10 on an
# 8-core box: 8 sessions ran at 5.3 fps (sim/wall 0.51), 16 at 2.4 fps (0.23), so past
# about ten the sessions are slow motion for everyone. A busy server answers
# ``{"error": "busy: ..."}`` and closes 1013.
SIM_MAX_SESSIONS = 10
_SIM_SESSIONS = 0

# Models being built for live sessions, by config: one build per design however many
# clients ask (single-flight); the watcher clears it with the model caches.
_SIM_BUILDS: dict[BuildConfig, asyncio.Future] = {}
# Bumped by the watcher's re-bake: a session started under an older generation closes.
_SIM_GENERATION = 0


def _build_mjcf_job(config: BuildConfig) -> tuple[str, dict]:
    """The MJCF and metadata of ``config`` (:func:`spiderpig.sim.mjcf.build_mjcf`) with the
    steering check (:func:`spiderpig.sim.run.steering_check`, ``meta["steering"]``: what
    the hello tells the viewer it may send): what a worker process runs, so one client's
    model build (OCCT holds the GIL for seconds; the check's recording loop too) doesn't
    stall every other live session's ticks."""
    from spiderpig.sim.mjcf import build_mjcf
    from spiderpig.sim.run import steering_check

    xml, meta = build_mjcf(config)
    meta["steering"] = steering_check(config, xml=xml, meta=meta)
    return xml, meta


def _build_mjcf_in_process(config: BuildConfig) -> tuple[str, dict]:
    """The same, in this process (workers off): under the bake lock, since fabrication's
    caches aren't thread-safe and the glb bake may be running."""
    with _BAKE_LOCK:
        return _build_mjcf_job(config)


async def _live_model(config: BuildConfig):
    """``(MjModel, meta)`` for ``config``, compiled (in a thread: the compile of a 120 KB
    MJCF takes 25-50 ms, a frame-plus hitch for every other session on the loop) from an
    MJCF built in a worker process (:mod:`spiderpig.workers`; in a thread when workers are
    off). Concurrent requests for one design share the build. A build the watcher
    outdated while it ran (the generation moved: the sources changed) is dropped and the
    design built again, so a stale model is never adopted or served to the next client."""
    from spiderpig import workers
    from spiderpig.sim import mjcf, run

    key = replace(config, robot=True)
    while True:
        have = mjcf.cached_model(key)
        if have is not None:
            return have
        generation = _SIM_GENERATION
        fut = _SIM_BUILDS.get(key)
        if fut is None:
            if workers.enabled():
                fut = asyncio.wrap_future(workers.submit(_build_mjcf_job, key))
            else:
                fut = asyncio.ensure_future(asyncio.to_thread(_build_mjcf_in_process, key))
            fut = _SIM_BUILDS[key] = asyncio.ensure_future(fut)
        try:
            xml, meta = await asyncio.shield(fut)   # a waiter's cancel leaves the build running
        finally:
            # built (adopted below) or failed (retried by the next client): the future is
            # spent; a newer build's future (after a generation change) is left alone
            if fut.done() and _SIM_BUILDS.get(key) is fut:
                _SIM_BUILDS.pop(key)
        if generation != _SIM_GENERATION:
            log.info("sim: the sources changed while %s built: building it again", key.key)
            continue
        model, meta = await asyncio.to_thread(mjcf.adopt_mjcf, key, mjcf.SimParams(), xml, meta)
        if "steering" not in meta:                  # a model this process built for something else
            meta["steering"] = await asyncio.to_thread(
                run.steering_check, key, mjcf.SimParams(), xml=xml, meta=meta)
        steer = meta["steering"]
        log.info("sim: %s: forward %s; turn %s, spin %s, step %s deg", key.key,
                 {k: (round(v, 2) if isinstance(v, float) else v)
                  for k, v in steer.get("forward", {}).items()},
                 steer["turn"], steer["spin"], steer.get("step_deg"))
        return model, meta


def _valid_cmd(msg) -> tuple[float, float]:
    """``[left, right]`` out of a client's message: two finite numbers, else a
    ``ValueError`` with the reason (a NaN or a string would stall the solver)."""
    cmd = msg.get("cmd")
    if not isinstance(cmd, (list, tuple)) or len(cmd) != 2 \
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in cmd):
        raise ValueError(f"cmd must be [left, right], two finite numbers; got {cmd!r}")
    return float(cmd[0]), float(cmd[1])


@app.websocket("/ws/sim")
async def ws_sim(websocket: WebSocket) -> None:
    """Drive the robot in MuJoCo live (:mod:`spiderpig.sim.live`): the design comes from
    the query (as ``/api/glb/robot``'s). The server answers with ``{"status":
    "building", "elapsed": s}`` while the model builds (in a worker process; every
    second), then a JSON ``hello`` (the bodies, the glb node -> body map, the ``design``
    it simulates, the steering check's verdict with its straight-run ``forward``) or
    ``{"error": ...}`` and a close, then one binary frame per tick at
    :data:`SIM_FPS` in real time (deadline-ticked; the physics steps in a thread). The
    client sends ``{"cmd": [left, right]}`` (fractions of the servo's speed) and
    ``{"reset": true}``, from the first message on (one sent while the model builds is
    applied before the hello); a malformed message gets ``{"error": ...}`` back and the
    session goes on. When the sources change the session says ``{"status": "stale"}`` and
    closes (the viewer reconnects to the rebuilt model); a failure inside the physics
    says ``{"error": ...}`` and closes 1011 (this session only). With
    :data:`SIM_MAX_SESSIONS` sessions already streaming, ``{"error": "busy: ..."}`` and
    close 1013."""
    global _SIM_SESSIONS
    from spiderpig.sim.live import LiveSim

    await websocket.accept()
    generation = _SIM_GENERATION
    if _SIM_SESSIONS >= SIM_MAX_SESSIONS:
        log.info("sim: busy (%d sessions)", _SIM_SESSIONS)
        await websocket.send_json({"error": f"busy: {_SIM_SESSIONS} physics sessions are "
                                   f"streaming already (the limit is {SIM_MAX_SESSIONS})"})
        await websocket.close(code=1013)
        return
    _SIM_SESSIONS += 1
    try:
        try:
            config = _config_from_query(websocket.query_params, robot=True)
            sim, early_errors = await _build_for(websocket, config, LiveSim)
        except (ValueError, HTTPException) as e:
            log.info("sim: no model: %s", e)
            await websocket.send_json({"error": f"{type(e).__name__}: {e}"})
            await websocket.close(code=1008)
            return
        except Exception as e:   # any failure is the client's to show
            log.exception("sim: no model")
            await websocket.send_json({"error": f"{type(e).__name__}: {e}"})
            await websocket.close(code=1011)
            return
        if sim is None:         # the client left while the model was building
            return
        await websocket.send_json({"hello": sim.hello()})
        for err in early_errors:    # malformed messages sent while the model built
            await websocket.send_json({"error": err})
        try:
            await _stream(websocket, sim, generation)
        except (WebSocketDisconnect, RuntimeError):
            raise
        except Exception as e:  # the physics failed: this session ends, told why
            log.exception("sim: session failed")
            await websocket.send_json({"error": f"{type(e).__name__}: {e}"})
            await websocket.close(code=1011)
    except WebSocketDisconnect as e:
        log.info("sim: client left (%s)", e.code)
    except RuntimeError as e:   # a send after the socket closed under us
        log.info("sim: %s", e)
    finally:
        _SIM_SESSIONS -= 1


def _apply(sim, msg: dict) -> None:
    """One client message onto ``sim`` (a reset is asked for, a command validated and
    stored); ``ValueError`` for a malformed one."""
    if not isinstance(msg, dict):
        raise ValueError("a message is a JSON object")
    if msg.get("reset"):
        sim.request_reset()
    if "cmd" in msg:
        sim.set_command(*_valid_cmd(msg))


async def _build_for(websocket: WebSocket, config: BuildConfig, make):
    """``(LiveSim of config, errors)`` for ``websocket``, telling it the elapsed seconds
    while the model builds; ``(None, [])`` when it disconnects first (the build goes on
    for the next client; ``WebSocketDisconnect`` later tells the session loop). A command
    or a reset sent meanwhile is kept and applied to the session before the hello; the
    ``errors`` are the malformed ones' answers, for the caller to send after the hello (an
    error before it would read as a failed connect)."""
    loop = asyncio.get_running_loop()
    t0 = loop.time()
    await websocket.send_json({"status": "building", "elapsed": 0})
    build = asyncio.ensure_future(_live_model(config))
    gone = asyncio.ensure_future(websocket.receive())   # a close frame ends the wait
    early: list = []
    try:
        while not build.done():
            done, _ = await asyncio.wait({build, gone}, timeout=SIM_BUILD_PING_S,
                                         return_when=asyncio.FIRST_COMPLETED)
            if gone in done:
                msg = gone.result()
                if msg.get("type") == "websocket.disconnect":
                    build.cancel()  # this waiter's; the shared build is shielded: it finishes
                    log.info("sim: client left while building %s", config.key)
                    return None, []
                if msg.get("text") is not None:
                    early.append(msg["text"])
                gone = asyncio.ensure_future(websocket.receive())
                continue
            if build not in done:
                await websocket.send_json({"status": "building",
                                           "elapsed": round(loop.time() - t0, 1)})
        model, meta = build.result()
        sim = make(config, model=model, meta=meta)
        errors = []
        for text in early:
            try:
                _apply(sim, json.loads(text))
            except ValueError as e:
                errors.append(str(e))
        return sim, errors
    finally:
        gone.cancel()


async def _stream(websocket: WebSocket, sim, generation: int) -> None:
    """The session loop: commands in (validated), frames out on a 1 / :data:`SIM_FPS`
    deadline, the physics stepped in a thread (``mj_step`` releases the GIL). The
    reader never touches ``MjData`` (the step may be running): a reset is asked for
    (:meth:`LiveSim.request_reset`, done by the next tick) and a command is stored."""
    loop = asyncio.get_running_loop()

    async def receive() -> None:
        while True:
            msg = await websocket.receive()
            if msg.get("type") == "websocket.disconnect":
                raise WebSocketDisconnect(msg.get("code", 1000))
            text = msg.get("text")
            if text is None:                # a binary frame: say so, carry on
                await websocket.send_json({"error": "commands are JSON text frames, not "
                                                    "binary ones"})
                continue
            try:
                _apply(sim, json.loads(text))
            except ValueError as e:         # bad JSON, bad shape, NaN: say so, carry on
                await websocket.send_json({"error": str(e)})

    def tick(dt: float) -> bytes:
        sim.advance(dt)
        return sim.frame()

    reader = asyncio.create_task(receive())
    period = 1.0 / SIM_FPS
    last = nxt = loop.time()
    try:
        while not reader.done():
            nxt += period
            await asyncio.sleep(max(0.0, nxt - loop.time()))
            if generation != _SIM_GENERATION:
                await websocket.send_json({"status": "stale"})
                await websocket.close(code=1012)
                return
            now = loop.time()
            frame = await asyncio.to_thread(tick, now - last)
            last = now
            if websocket.client_state != WebSocketState.CONNECTED:
                return
            await websocket.send_bytes(frame)
            if nxt < loop.time() - period:      # we fell behind: don't burst to catch up
                nxt = loop.time()
        reader.result()                         # a disconnect, re-raised
    finally:
        reader.cancel()


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
