"""``spiderpig view <design>``: the viewer for a stored design, with no Node on the machine.

    spiderpig view 1a2b3c4d5e6f7a8b                 # prints the URL, serves until Ctrl-C
    spiderpig view 1a2b3c4d5e6f7a8b --open          # ... and opens it in a browser
    spiderpig view 1a2b3c4d5e6f7a8b --store PATH --port 8123

The store is picked as the MCP server picks it (``--store``, else ``$SPIDERPIG_STORE``,
else ``./.spiderpig``). The design's animated ``.glb`` is exported first
(``api.export(design, ["glb"])``, cached in the store), then the FastAPI app of
:mod:`spiderpig.server.app` serves the built viewer shipped inside the package
(``spiderpig/viewer/dist``) on a free port, with the design's :class:`BuildConfig`
behind ``?design=<id>``: the page's ``/api/glb``, ``/api/walk`` and ``/api/design``
calls answer for that design (its linkage, module, phases, proportions, servo, sheet
and constructions), and the tune panel's edits apply on top of it.

:func:`start_background` runs the same server as a child process for another
program, the MCP ``view`` tool: one per store, reused across calls, serving any
design in it.
"""

from __future__ import annotations

import argparse
import contextlib
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass
from pathlib import Path

DEFAULT_HOST = "127.0.0.1"
STARTUP_TIMEOUT = 90.0     # a cold start imports the engine (~3 s) and may bake


def free_port(host: str = DEFAULT_HOST) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, 0))
        return s.getsockname()[1]


def port_open(host: str, port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def base_url(host: str, port: int) -> str:
    return f"http://{host}:{port}"


def url_for(host: str, port: int, design_id: str | None) -> str:
    """The viewer's URL: ``/?design=<id>`` (the page loads that design's glb and maps
    its linkage, module, phases and proportions onto the tune panel)."""
    return f"{base_url(host, port)}/?design={design_id}" if design_id else base_url(host, port)


def viewer_built() -> Path | None:
    """The built viewer the server would serve, or ``None`` (a checkout before
    ``mise run viewer-build``)."""
    from spiderpig.server.app import viewer_dist_dir

    return viewer_dist_dir()


def load_design(design_id: str, store):
    """The recorded design (:func:`spiderpig.api.load`); ``KeyError`` names the store
    and what it holds."""
    from spiderpig import api

    try:
        return api.load(design_id, store)
    except KeyError:
        ids = [c["id"] for c in api.list_designs(store)]
        have = ", ".join(ids[-8:]) if ids else "nothing"
        raise KeyError(f"no design {design_id!r} in {store.root.resolve()} (has: {have})"
                       ) from None


def export_glb(design) -> Path:
    """The design's animated ``.glb`` in its store (``api.export``: built and baked once,
    cached afterwards); a failing stage raises ``ValueError`` with its failures."""
    from spiderpig import api

    rep = api.export(design, ["glb"])
    if not rep.ok:
        raise ValueError("; ".join(f.describe() for f in rep.failures))
    return Path(next(f for f in rep.files if f.endswith(".glb")))


def run_server(store, host: str, port: int, *, log_level: str = "warning") -> None:
    """Serve the viewer for ``store`` in this process until interrupted (no bake of the
    default robot at startup: designs bake on request, or come from their exports)."""
    import uvicorn

    from spiderpig.server import app as server_app

    server_app.configure(store=store, prebake_default=False)
    server_app.allow_host(host)         # the name its URL is printed under (HostGuard)
    uvicorn.run(server_app.app, host=host, port=port, log_level=log_level)


def _open_when_up(url: str, host: str, port: int) -> None:
    def wait() -> None:
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if port_open(host, port):
                with contextlib.suppress(Exception):
                    webbrowser.open(url, new=2)
                return
            time.sleep(0.25)

    threading.Thread(target=wait, daemon=True).start()


@dataclass
class ViewServer:
    """A ``spiderpig view --serve-only`` child process serving one store."""

    host: str
    port: int
    store_root: str
    proc: subprocess.Popen | None = None

    @property
    def base(self) -> str:
        return base_url(self.host, self.port)

    def url(self, design_id: str) -> str:
        return url_for(self.host, self.port, design_id)

    def alive(self) -> bool:
        return (self.proc is None or self.proc.poll() is None) and port_open(self.host, self.port)

    def stop(self) -> None:
        if self.proc is None or self.proc.poll() is not None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def start_background(store, host: str = DEFAULT_HOST, port: int | None = None,
                     timeout: float = STARTUP_TIMEOUT) -> ViewServer:
    """Start ``python -m spiderpig.view --serve-only`` on ``store`` and return once it
    accepts connections (``RuntimeError`` when it exits or doesn't come up in time)."""
    root = str(Path(store.root).resolve())
    port = port or free_port(host)
    cmd = [sys.executable, "-m", "spiderpig.view", "--serve-only", "--store", root,
           "--host", host, "--port", str(port), "--log-level", "warning"]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"the view server exited with code {proc.returncode}: "
                               f"{' '.join(cmd)}")
        if port_open(host, port):
            return ViewServer(host, port, root, proc)
        time.sleep(0.2)
    proc.terminate()
    raise RuntimeError(f"the view server did not come up on {host}:{port} within {timeout:.0f} s")


DESIGN_OPTIONS = ("linkage", "module", "phases", "proportion", "servo", "pillar", "pin",
                  "crank", "sheet", "thickness", "frame_sheet", "crank_sheet", "heads",
                  "link_sheet", "side_only")


def resolve_args(args, store):
    """The design the build options ask for (``--linkage``, ``--module``, ``--pin``, ...:
    the same options as ``spiderpig build``), resolved into ``store`` through the API
    (:func:`spiderpig.api.spec_of`), so a CLI build can be viewed by its options; or
    ``None`` when none was given."""
    given = {k: getattr(args, k) for k in DESIGN_OPTIONS if getattr(args, k, None) is not None}
    if not given or given == {"side_only": False}:
        return None
    from types import SimpleNamespace

    from spiderpig import api
    from spiderpig.config import BuildConfig, ParamError, config_from_args

    d = BuildConfig()       # what `spiderpig build` makes of the same options
    opts = SimpleNamespace(**{k: getattr(d, k) for k in ("linkage", "sheet", "servo", "pillar",
                                                         "pin", "frame_sheet", "heads")})
    for k, v in given.items():
        setattr(opts, k, v)
    try:            # a mechanism: its one module, one side, with no option saying so
        config = config_from_args(opts, robot=False if given.get("side_only") else None)
    except ParamError as e:
        raise ValueError(str(e)) from None
    return api.resolve(api.spec_of(config), store)


def main(argv: list[str] | None = None) -> int:
    from spiderpig.config import add_build_args, add_design_args

    ap = argparse.ArgumentParser(
        prog="spiderpig view",
        description="serve the viewer for a stored design, or for the design the build "
                    "options describe (prints the URL; Ctrl-C stops)")
    ap.add_argument("design", nargs="?", metavar="DESIGN",
                    help="a design id (16 hex digits, from resolve); spiderpig mcp's "
                         "list_designs or api.list_designs() list the store's. Or give the "
                         "build options below (as for spiderpig build), and the design is "
                         "resolved into the store first")
    add_design_args(ap)
    add_build_args(ap)
    ap.add_argument("--side-only", action="store_true",
                    help="with the build options: one side (no second side, no chassis)")
    ap.set_defaults(linkage=None, module=None, servo=None, pillar=None, pin=None, crank=None,
                    sheet=None, frame_sheet=None, heads=None)   # None: not given
    ap.add_argument("--store", metavar="PATH",
                    help="the design store (default: $SPIDERPIG_STORE, else ./.spiderpig)")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=0, metavar="N",
                    help="TCP port (default: a free one)")
    ap.add_argument("--open", action="store_true", help="open the URL in a browser")
    ap.add_argument("--no-export", action="store_true",
                    help="don't export the design's glb first (it bakes on the first request)")
    ap.add_argument("--serve-only", action="store_true",
                    help="serve the store without a design (what the MCP view tool runs)")
    ap.add_argument("--log-level", default="warning",
                    choices=("critical", "error", "warning", "info", "debug"),
                    help="uvicorn's logging")
    args = ap.parse_args(argv)
    from spiderpig.store import Store

    store = Store.of(args.store) if args.store else Store.default()
    design = None
    if args.design:
        try:
            design = load_design(args.design, store)
        except (KeyError, ValueError) as e:
            print(f"error: {e}", file=sys.stderr)
            return 2
    else:
        try:
            design = resolve_args(args, store)       # the build options, as a design
        except ValueError as e:
            print(f"error: {e}", file=sys.stderr)
            return 2
        if design is None and not args.serve_only:
            ap.error("a design id or the build options (--linkage, --module, --pin, ...) are "
                     "required (or --serve-only)")
        if design is not None:
            print(f"resolved the build options into {store.root.resolve()} as design "
                  f"{design.id} (spiderpig view {design.id} shows it again)", file=sys.stderr)
    # the arguments first (an unknown design, a missing one), then whether there is a
    # viewer to serve them with, before anything is exported
    if viewer_built() is None:
        from spiderpig.server.app import VIEWER_DIST

        print(f"error: the viewer isn't built ({VIEWER_DIST} has no index.html). From a "
              "checkout run `mise run viewer-build`; a release wheel ships it. "
              "SPIDERPIG_VIEWER_DIST=<dir> points at another build.", file=sys.stderr)
        return 2
    if design is not None and not args.no_export:
        print(f"exporting the glb of {design.id} (built and baked once, then cached)...",
              file=sys.stderr, flush=True)
        try:
            export_glb(design)
        except ValueError as e:
            print(f"error: {design.id} can't be built: {e}", file=sys.stderr)
            return 1
    port = args.port or free_port(args.host)
    url = url_for(args.host, port, design.id if design else None)
    print(url, flush=True)
    if args.open:
        _open_when_up(url, args.host, port)
    with contextlib.suppress(KeyboardInterrupt):
        run_server(store, args.host, port, log_level=args.log_level)
    return 0


__all__ = ["ViewServer", "base_url", "export_glb", "free_port", "load_design", "main",
           "port_open", "run_server", "start_background", "url_for", "viewer_built"]

if __name__ == "__main__":
    sys.exit(main())
