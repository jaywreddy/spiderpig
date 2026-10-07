"""The viewer's server (W1 items 1, 4, 5 and 7): a stored design moved to another linkage
or module gets that one's default crank, only allowed Hosts and WebSocket Origins are
served, the failed-bake memo stays bounded, the watcher changes the live sessions' state on
the event loop, and a bake queued for a client that has left is skipped."""

from __future__ import annotations

import asyncio
import socket
import threading
import time

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from spiderpig import api, config
from spiderpig.config import BuildConfig
from spiderpig.server import app as srv

LOCAL = "http://localhost"


@pytest.fixture
def stored(tmp_path):
    """Record configs in a store of the test's own, the server pointed at it."""
    srv.configure(str(tmp_path), prebake_default=False)

    def record(cfg: BuildConfig) -> str:
        return api.resolve(api.spec_of(cfg), str(tmp_path)).id

    yield record
    srv.configure(None, prebake_default=True)


# ---------------------------------------------------------------------------
# 1. A stored design moved to another linkage or module
# ---------------------------------------------------------------------------


def test_another_linkage_gets_its_own_default_crank_and_crank_sheet(stored):
    heel = stored(BuildConfig(linkage="trotbot_heel", module="single"))
    assert api.load(heel, srv.store()).config.crank == "bolt_round"     # TrotBot's own
    got = srv._config_from_query({"design": heel, "linkage": "strider"}, robot=True)
    assert (got.linkage, got.crank) == ("strider", "bolt")

    pant = stored(BuildConfig(linkage="hoecken_pantograph", robot=False))
    assert api.load(pant, srv.store()).config.crank_sheet == "al6061_2mm"
    got = srv._config_from_query({"design": pant, "linkage": "dwell_rocker"}, robot=False)
    assert got.crank_sheet == config.CRANK_SHEET

    # a crank the design chose (not its linkage's default) goes with it
    chose = stored(BuildConfig(crank="bolt_round", crank_sheet="al6061_3p2mm"))
    got = srv._config_from_query({"design": chose, "linkage": "klann"}, robot=True)
    assert (got.crank, got.crank_sheet) == ("bolt_round", "al6061_3p2mm")


def test_another_module_gets_its_own_default_crank(stored, monkeypatch):
    double = stored(BuildConfig())
    monkeypatch.setitem(config.MODULE_CRANKS, ("strider", "quad"), "bolt_round")
    got = srv._config_from_query({"design": double, "module": "quad"}, robot=True)
    assert (got.module, got.crank) == ("quad", "bolt_round")
    got = srv._config_from_query({"design": double, "p.unit": "1.1"}, robot=True)
    assert (got.module, got.crank) == ("double", "bolt")      # its module: its crank
    chose = stored(BuildConfig(crank="bolt_round"))
    monkeypatch.setitem(config.MODULE_CRANKS, ("strider", "decker"), "bolt")
    got = srv._config_from_query({"design": chose, "module": "decker"}, robot=True)
    assert got.crank == "bolt_round"                          # chosen: kept


def test_a_stored_glb_request_loads_the_design_once(stored, monkeypatch):
    pant = stored(BuildConfig(linkage="hoecken_pantograph", robot=False))
    loads = []
    real = srv._design
    monkeypatch.setattr(srv, "_design", lambda i: loads.append(i) or real(i))
    monkeypatch.setattr(srv, "_ensure_baked", lambda cfg, gone=None: (_ for _ in ()).throw(
        srv.HTTPException(status_code=422, detail="not baked in this test")))
    r = TestClient(srv.app, base_url=LOCAL).get(f"/api/glb/side?design={pant}")
    assert r.status_code == 422
    assert loads == [pant]


# ---------------------------------------------------------------------------
# 4. Hosts and Origins
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(("host", "allowed"), [
    ("localhost:8000", True), ("127.0.0.1:8123", True), ("[::1]:8000", True),
    ("192.168.1.20:8000", True), ("app.localhost", True),
    ("evil.example", False), ("localhost.evil.example", False), ("", False),
])
def test_only_allowed_hosts_are_served(host, allowed, monkeypatch):
    monkeypatch.delenv("VITE_ALLOWED_HOSTS", raising=False)
    r = TestClient(srv.app).get("/api/modes", headers={"host": host})
    assert r.status_code == (200 if allowed else 400), (host, r.text)


def test_vite_allowed_hosts_extend_the_hosts(monkeypatch):
    monkeypatch.setenv("VITE_ALLOWED_HOSTS", ".ts.net,box.lan")
    client = TestClient(srv.app)
    for host, code in (("robot.tail1234.ts.net", 200), ("ts.net", 200), ("box.lan:5173", 200),
                       ("ts.net.evil.example", 400), ("evilts.net", 400)):
        assert client.get("/api/modes", headers={"host": host}).status_code == code, host


def test_the_name_spiderpig_view_serves_under_is_allowed(monkeypatch):
    monkeypatch.delenv("VITE_ALLOWED_HOSTS", raising=False)
    monkeypatch.setattr(srv, "_SERVED_AS", [])
    client = TestClient(srv.app)
    assert client.get("/api/modes", headers={"host": "mybox:8765"}).status_code == 400
    srv.allow_host("mybox")
    assert client.get("/api/modes", headers={"host": "mybox:8765"}).status_code == 200


def test_a_foreign_page_cant_open_a_websocket(monkeypatch):
    monkeypatch.delenv("VITE_ALLOWED_HOSTS", raising=False)
    client = TestClient(srv.app, base_url=LOCAL)
    for path in ("ws://localhost/ws", "ws://localhost/ws/sim?linkage=nope"):
        with pytest.raises(WebSocketDisconnect) as refused, client.websocket_connect(
                path, headers={"origin": "https://evil.example"}):
            pass
        assert refused.value.code == 1008
    for origin in (None, "http://localhost", "http://localhost:5173", "http://127.0.0.1:9"):
        headers = {"origin": origin} if origin else {}
        with client.websocket_connect("ws://localhost/ws", headers=headers) as ws:
            ws.send_text("ping")                    # accepted: the socket is open
    with pytest.raises(WebSocketDisconnect), client.websocket_connect(
            "ws://localhost/ws", headers={"origin": "null"}):
        pass


# ---------------------------------------------------------------------------
# 5. The failed-bake memo; the watcher's changes on the loop
# ---------------------------------------------------------------------------


def test_failed_bakes_are_pruned(stored, monkeypatch):
    monkeypatch.setattr(srv, "_FAILED", {})
    monkeypatch.setattr(srv, "_BAKED", {})

    def fail(_cfg):
        raise ValueError("no plan")

    monkeypatch.setattr(srv, "_bake", fail)
    for i in range(1, srv.CACHE_SIZE + 7):      # (unit 16 is the default design)
        cfg = BuildConfig(linkage="hoecken", robot=False, proportions=(("unit", 16 + i / 10),))
        with pytest.raises(srv.HTTPException):
            srv._ensure_baked(cfg)
    assert len(srv._FAILED) == srv.CACHE_SIZE
    assert len(srv._BAKED) <= srv.CACHE_SIZE
    newest = srv._glb_path(cfg)
    assert newest in srv._FAILED                # the newest failures are kept


def test_the_watchers_rebake_changes_the_sessions_state_on_the_loop(monkeypatch):
    class Builds(dict):
        cleared_on: list[int] = []

        def clear(self) -> None:
            self.cleared_on.append(threading.get_ident())
            super().clear()

    builds = Builds()
    monkeypatch.setattr(srv, "_SIM_BUILDS", builds)
    monkeypatch.setattr(srv, "_BAKED", {})
    monkeypatch.setattr(srv, "_SIM_GENERATION", 7)

    async def main() -> int:
        monkeypatch.setattr(srv, "_LOOP", asyncio.get_running_loop(), raising=False)
        await asyncio.to_thread(srv._rebake_all)     # as the watcher runs it
        await asyncio.sleep(0)                        # the loop runs what was scheduled
        await asyncio.sleep(0)
        return threading.get_ident()

    loop_thread = asyncio.run(main())
    assert builds.cleared_on == [loop_thread]
    assert srv._SIM_GENERATION == 8


# ---------------------------------------------------------------------------
# 7. A bake queued for a client that has left
# ---------------------------------------------------------------------------


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_a_queued_bake_whose_client_left_is_skipped(tmp_path, monkeypatch):
    """A viewer asks for one design, then another (the first fetch aborted): the first's
    request waits on the bake lock behind a running bake, its client gone. Before, it baked
    anyway once the lock came free; now it gives up and nothing is baked for it."""
    import uvicorn

    srv.configure(str(tmp_path), prebake_default=False)
    baked: list[str] = []

    def bake(cfg):
        baked.append(cfg.key)
        path = srv._glb_path(cfg)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"glb")
        return path

    monkeypatch.setattr(srv, "_bake", bake)
    monkeypatch.setattr(srv, "BAKE_POLL_S", 0.05, raising=False)
    port = _free_port()
    server = uvicorn.Server(uvicorn.Config(srv.app, host="127.0.0.1", port=port,
                                           lifespan="off", log_level="warning"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 20
        while not server.started:
            assert time.monotonic() < deadline, "the server didn't start"
            time.sleep(0.05)
        srv._BAKE_LOCK.acquire()            # a bake is running
        try:
            with socket.create_connection(("127.0.0.1", port)) as s:
                s.sendall(b"GET /api/glb/side?linkage=hoecken HTTP/1.1\r\n"
                          b"Host: localhost\r\n\r\n")
                time.sleep(0.5)             # the request is queued on the lock
            time.sleep(0.5)                 # the client has gone
        finally:
            srv._BAKE_LOCK.release()
        time.sleep(1.0)                     # time to bake, if it were going to
        assert baked == []
        # a client that stays gets its bake
        import urllib.request

        with urllib.request.urlopen(  # noqa: S310
                f"http://127.0.0.1:{port}/api/glb/side?linkage=hoecken") as r:
            assert r.read() == b"glb"
        assert len(baked) == 1
    finally:
        server.should_exit = True
        thread.join(10)
        srv.configure(None, prebake_default=True)
