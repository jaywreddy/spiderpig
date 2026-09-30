"""``spiderpig view`` (harness v1, step 4): the package carries the built viewer, the
server answers ``/api/design``, ``/api/walk`` and ``/api/glb`` for a stored design
(``?design=<id>``, the tune panel's overrides on top), the CLI serves it on a free port,
and ``import spiderpig`` / ``spiderpig --help`` stay instant.

The tests that need the built viewer (``spiderpig/viewer/dist``: ``mise run
viewer-build``) skip without it; the MCP ``view`` tool is in ``test_spiderpig_mcp.py``.
"""

from __future__ import annotations

import json
import subprocess
import sys
import urllib.request
from pathlib import Path

import pytest

from spiderpig import api, cli, view
from spiderpig.store import Store

ROOT = Path(__file__).resolve().parents[1]
DIST = ROOT / "spiderpig" / "viewer" / "dist"
needs_dist = pytest.mark.skipif(not (DIST / "index.html").is_file(),
                                reason="spiderpig/viewer/dist isn't built (mise run viewer-build)")

SINGLE = {"kind": "walker", "linkage": {"key": "klann"},
          "legs": {"module": "single", "sides": 1}}
QUAD_PHASED = {"kind": "walker", "linkage": {"key": "klann", "params": {"OB": 1.121}},
               "legs": {"module": "quad", "phases_deg": [0, 175, 180, 355]},
               "materials": {"servo": "xl330_m288"}}


def _python(*args: str) -> str:
    return subprocess.run([sys.executable, *args], capture_output=True, text=True, check=True,
                          cwd=ROOT, timeout=120).stdout


# ---------------------------------------------------------------------------
# The package and the CLI
# ---------------------------------------------------------------------------


def test_import_spiderpig_loads_no_engine():
    """``import spiderpig`` imports none of the engine (the names load on first use)."""
    out = _python("-c", "import sys, spiderpig; "
                        "print(sorted(m for m in sys.modules if m.startswith('spiderpig.')))")
    assert out.strip() == "[]"
    out = _python("-c", "from spiderpig import Spec, api; print(Spec.__module__, api.__name__)")
    assert out.split() == ["spiderpig.spec", "spiderpig.api"]


def test_operations_live_in_api_not_the_package():
    """The operations are ``api.*``; several share a name with an engine module
    (``walk``, ``explain``, ``build``, ``verify``, ``recommend``), and the package's
    attribute of that name is the submodule, so the package re-exports no operation."""
    import spiderpig
    import spiderpig.build
    import spiderpig.explain
    import spiderpig.recommend
    import spiderpig.verify
    import spiderpig.walk

    for name in ("walk", "explain", "build", "verify", "recommend", "resolve", "plan"):
        assert name not in spiderpig.__all__
        assert callable(getattr(api, name))
    for name in ("walk", "explain", "build", "verify", "recommend"):
        assert getattr(spiderpig, name).__name__ == f"spiderpig.{name}"
    assert spiderpig.Spec.__module__ == "spiderpig.spec"          # a lazy type export
    with pytest.raises(AttributeError):
        spiderpig.nonsense  # noqa: B018


def test_cli_help_lists_every_command():
    out = _python("-m", "spiderpig.cli", "--help")
    listed = [line.split()[0] for line in out.splitlines()[3:] if line.strip()]
    assert listed == list(cli.COMMANDS)
    assert set(listed) >= {"build", "bake", "audit", "explain", "tune", "sim", "report",
                           "mcp", "view"}
    assert cli.COMMANDS["view"][0] == "spiderpig.view"
    assert "usage: spiderpig <command>" in out
    assert cli.main(["nonsense"]) == 2
    assert cli.main(["--help"]) == 0


def test_view_help_is_instant():
    out = _python("-m", "spiderpig.cli", "view", "--help")
    assert {"--store", "--port", "--open", "--serve-only"} <= set(out.split())


def test_viewer_dist_resolves_inside_the_package():
    from spiderpig.server.app import PACKAGE_ROOT, VIEWER_DIST

    assert PACKAGE_ROOT == ROOT / "spiderpig"
    assert (PACKAGE_ROOT / "__init__.py").is_file()
    assert VIEWER_DIST == PACKAGE_ROOT / "viewer" / "dist"


def test_bakes_live_in_the_store(monkeypatch, tmp_path):
    from spiderpig.bake import default_bake_dir

    monkeypatch.setenv("SPIDERPIG_STORE", str(tmp_path / "s"))
    assert default_bake_dir() == tmp_path / "s" / "bakes"


# ---------------------------------------------------------------------------
# The server answers for a stored design
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def store(tmp_path_factory) -> Store:
    return Store(tmp_path_factory.mktemp("view-store"))


@pytest.fixture(scope="module")
def single(store):
    return api.resolve(SINGLE, store)


@pytest.fixture(scope="module")
def phased(store):
    return api.resolve(QUAD_PHASED, store)


@pytest.fixture(scope="module")
def client(store):
    """The app over ``store``, not entered (no lifespan: no default bake, no watcher)."""
    from fastapi.testclient import TestClient

    from spiderpig.server import app as server_app

    server_app.configure(store=store, prebake_default=False)
    yield TestClient(server_app.app)
    server_app.configure(store=None, prebake_default=True)


def test_design_card(client, single, phased):
    card = client.get(f"/api/design/{single.id}").json()
    assert card["design"] == single.id
    assert (card["kind"], card["linkage"], card["module"], card["sides"], card["mode"]) == \
        ("walker", "klann", "single", 1, "side")
    assert card["phases_deg"] == [0.0]
    assert card["params"] == single.config.design_json()["proportions"]
    assert card["glb"] == f"/api/glb/side?design={single.id}"
    card = client.get(f"/api/design/{phased.id}").json()
    assert (card["mode"], card["module"], card["servo"]) == ("robot", "quad", "xl330_m288")
    assert card["phases_deg"] == [0, 175, 180, 355]
    assert card["params"]["OB"] == pytest.approx(1.121)
    assert card["glb"] == f"/api/glb/robot?design={phased.id}"


def test_design_card_misuse(client):
    assert client.get("/api/design/0000000000000000").status_code == 404
    r = client.get("/api/design/not-an-id")
    assert r.status_code == 422
    assert "design id" in r.json()["detail"]
    r = client.get("/api/walk", params={"design": "0000000000000000"})
    assert r.status_code == 404


def test_walk_answers_for_the_design(client, phased):
    w = client.get("/api/walk", params={"design": phased.id}).json()
    assert w["valid"] is True
    assert (w["linkage"], w["module"]) == ("klann", "quad")
    assert w["phases_deg"] == [0, 175, 180, 355]
    assert w["proportions"]["OB"] == pytest.approx(1.121)
    assert w["servo"]["key"] == "xl330_m288"


def test_tune_edits_apply_on_top_of_the_design(client, phased):
    """What the tune panel sends: the id, then its own module, phases and ``p.NAME``."""
    w = client.get("/api/walk", params={"design": phased.id, "module": "quad",
                                        "phases": "0,180,90,270", "p.DF": "2.7"}).json()
    assert w["phases_deg"] == [0, 180, 90, 270]
    assert w["proportions"]["OB"] == pytest.approx(1.121)      # the design's, kept
    assert w["proportions"]["DF"] == pytest.approx(2.7)        # the panel's
    assert w["servo"]["key"] == "xl330_m288"                   # the design's materials
    w = client.get("/api/walk", params={"design": phased.id, "module": "double"}).json()
    assert (w["module"], len(w["phases_deg"])) == ("double", 2)
    w = client.get("/api/walk", params={"design": phased.id, "linkage": "jansen",
                                        "module": "double"}).json()
    assert (w["linkage"], w["module"]) == ("jansen", "double")   # another linkage: its own
    assert w["servo"]["key"] == "xl330_m288"


@pytest.mark.slow
def test_glb_comes_from_the_export_or_a_bake(client, single):
    r = client.get("/api/glb/side", params={"design": single.id})
    assert r.status_code == 200
    assert r.content[:4] == b"glTF"
    assert r.headers["x-spiderpig-glb"] == "bake"
    path = view.export_glb(single)
    assert path.is_file()
    assert path.parent == single.store.exports_dir(single.id)
    r = client.get("/api/glb/side", params={"design": single.id})
    assert r.headers["x-spiderpig-glb"] == "export"
    assert r.content == path.read_bytes()
    r = client.get("/api/glb/side", params={"design": single.id, "p.OB": "1.2"})
    assert r.status_code == 200
    assert r.headers["x-spiderpig-glb"] == "bake"


# ---------------------------------------------------------------------------
# spiderpig view
# ---------------------------------------------------------------------------


def test_view_refuses_an_unknown_design(store, capsys):
    assert view.main(["0000000000000000", "--store", str(store.root), "--no-export"]) == 2
    assert "no design '0000000000000000'" in capsys.readouterr().err
    assert view.main(["nonsense", "--store", str(store.root), "--no-export"]) == 2
    with pytest.raises(SystemExit):
        view.main(["--store", str(store.root)])        # neither a design nor --serve-only


def test_urls():
    assert view.url_for("127.0.0.1", 8123, "abcdef0123456789") == \
        "http://127.0.0.1:8123/?design=abcdef0123456789"
    assert view.url_for("127.0.0.1", 8123, None) == "http://127.0.0.1:8123"


@needs_dist
@pytest.mark.slow
def test_view_serves_a_design_on_a_free_port(store, single):
    """The server as ``spiderpig view`` runs it (a child process, as the MCP tool starts
    it): the page, the design's card and its glb, on a port nobody chose."""
    view.export_glb(single)
    srv = view.start_background(store)
    try:
        assert srv.alive()
        assert srv.url(single.id) == f"http://127.0.0.1:{srv.port}/?design={single.id}"
        with urllib.request.urlopen(f"{srv.base}/") as r:  # noqa: S310
            html = r.read().decode()
        assert 'id="stage"' in html
        assert "<script" in html
        with urllib.request.urlopen(f"{srv.base}/api/design/{single.id}") as r:  # noqa: S310
            assert json.load(r)["mode"] == "side"
        with urllib.request.urlopen(f"{srv.base}/api/glb/side?design={single.id}") as r:  # noqa: S310
            assert r.read(4) == b"glTF"
            assert r.headers["x-spiderpig-glb"] == "export"
    finally:
        srv.stop()
    assert not srv.alive()


# ---------------------------------------------------------------------------
# Test drive, round 2 (docs/agentlib/TESTDRIVE.md): the CLI's explain takes the build options
# ---------------------------------------------------------------------------


def test_explain_takes_the_build_options(capsys):
    from spiderpig import explain

    assert explain.main(["--linkage", "klann", "--module", "single", "--thickness", "2"]) == 0
    out = capsys.readouterr().out                                          # entry 8
    assert "STOP: no M3 screw and nut fit a crankpin joint in 2 mm layers" in out
    assert "materials.thickness_mm 2 -> 3" in out
    assert explain.main(["--linkage", "klann", "--module", "single", "--pin", "bearing",
                         "--pillar", "bearing", "--servo", "xl330_m288"]) == 0
    out = capsys.readouterr().out
    assert "ground clearance:" in out
    assert "3. plan" in out
    assert "bearing" in out or "sleeve" in out
