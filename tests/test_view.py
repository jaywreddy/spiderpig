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
    # the standoff pillar's end screw is the first construction 2 mm layers stop (before the
    # crank: the pillars are checked first since they became the default, 2026-10-03)
    assert ("STOP: an M4 button head and washer (3 mm) don't fit a 2 mm layer outside the "
            "plate") in out
    assert "materials.thickness_mm 2 -> 3" in out
    # bearing pivots assume full layers, which the default single-plate crank's gaps
    # don't give (it says so and names --crank keyed): the keyed crank here
    assert explain.main(["--linkage", "klann", "--module", "single", "--pin", "bearing",
                         "--pillar", "bearing", "--servo", "xl330_m288",
                         "--crank", "keyed"]) == 0
    out = capsys.readouterr().out
    assert "ground clearance:" in out
    assert "3. plan" in out
    assert "bearing" in out or "sleeve" in out


# ---------------------------------------------------------------------------
# Test drive, round 3 (docs/agentlib/TESTDRIVE.md): a CLI build can be viewed by its
# options, and the CLIs warn about a thickness far from the nominal
# ---------------------------------------------------------------------------


def test_view_takes_the_build_options_and_resolves_them_into_the_store(store, capsys):
    import argparse

    from spiderpig import view

    ns = argparse.Namespace(linkage="klann", module="single", phases=None, proportion=None,
                            servo=None, pillar=None, pin="bolt", crank=None, sheet=None,
                            thickness=None, side_only=False)
    d = view.resolve_args(ns, store)                                          # entry 10
    assert d is not None
    assert d.config.pin == "bolt"
    assert d.config.module == "single"
    assert d.config.robot
    assert api.load(d.id, store).config == d.config
    empty = {**dict.fromkeys(view.DESIGN_OPTIONS), "side_only": False}
    assert view.resolve_args(argparse.Namespace(**empty), store) is None
    mech = argparse.Namespace(**{**empty, "linkage": "hoecken", "module": "single"})
    assert view.resolve_args(mech, store).config.robot is False
    with pytest.raises(SystemExit):                # neither a design id nor an option
        view.main(["--store", str(store.root)])
    assert "build options" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        view.main(["--help"])
    out = capsys.readouterr().out
    assert "--pin" in out
    assert "--linkage" in out
    assert "--thickness" in out


def test_the_clis_warn_about_a_thickness_far_from_the_nominal(capsys):
    from spiderpig import explain

    assert explain.main(["--linkage", "klann", "--module", "single", "--thickness", "5"]) == 0
    err = capsys.readouterr().err                                             # entry 8
    assert "warning: materials.thickness_mm 5 is 67% over acrylic_3mm's nominal 3 mm" in err


# ---------------------------------------------------------------------------
# Test drive, round 4 (docs/agentlib/TESTDRIVE.md): the CLIs take a mechanism as it is
# (its one module, one side), audit and report cover it, the output names its axes
# ---------------------------------------------------------------------------


def test_the_clis_default_a_mechanism_to_its_one_module_and_one_side(store, capsys, tmp_path):
    import argparse

    from spiderpig import build, explain, view
    from spiderpig.config import config_from_args, default_module, default_robot
    from spiderpig.tools import report

    assert default_module("parallelogram_lift") == "single"                  # entry 9
    assert default_module("klann") == "quad"
    assert default_module("strider") == "double"             # the linkage's own default
    assert not default_robot("parallelogram_lift")
    assert default_robot("klann")
    ns = argparse.Namespace(linkage="parallelogram_lift", module=None, phases=None,
                            proportion=None)
    cfg = config_from_args(ns, robot=None)
    assert (cfg.module, cfg.robot) == ("single", False)
    cfg = config_from_args(argparse.Namespace(linkage="klann", module=None, phases=None,
                                              proportion=None))
    assert (cfg.module, cfg.robot) == ("quad", True)
    cfg = config_from_args(argparse.Namespace(linkage="strider", module=None, phases=None,
                                              proportion=None))
    assert (cfg.module, cfg.robot) == ("double", True)
    args = build._parse_args(["--linkage", "parallelogram_lift", "--out", str(tmp_path)])
    assert (args.config.module, args.config.robot) == ("single", False)
    with pytest.raises(SystemExit):        # a robot of a mechanism is still refused, and says
        build._parse_args(["--linkage", "parallelogram_lift", "--module", "quad"])
    assert "unknown module 'quad'; have ['single']" in capsys.readouterr().err
    assert explain.main(["--linkage", "parallelogram_lift"]) == 0
    out = capsys.readouterr().out
    assert "covers 1.62 mm in x by 32.00 mm in y (up), stroke 32.00 mm" in out   # entry 11
    empty = {**dict.fromkeys(view.DESIGN_OPTIONS), "side_only": False}
    d = view.resolve_args(argparse.Namespace(**{**empty, "linkage": "hoecken"}), store)
    assert (d.config.module, d.config.robot) == ("single", False)
    out_file = tmp_path / "report.json"                                       # entry 8
    assert report.main(["--linkages", "parallelogram_lift", "--no-plan", "--out",
                        str(out_file)]) == 0
    (row,) = json.loads(out_file.read_text())
    assert "foot" not in row
    assert row["output"]["stroke_mm"] == pytest.approx(32.0, abs=0.01)
    assert row["output"]["text"].startswith("parallelogram_lift: b4 (translation_platform)")


@pytest.mark.slow
def test_audit_takes_a_mechanism(tmp_path, capsys):
    from spiderpig.tools import audit

    # the bolt crank: a mechanism's default (keyed) crank fails the jam check since its key's
    # printed sockets are rated (SF 0.64 at the 0.85 N·m limit), which fails the audit
    assert audit.main(["--linkage", "parallelogram_lift", "--crank", "bolt", "--ts-contract", "0",
                       "--ts-clash", "1", "--out", str(tmp_path)]) == 0    # entry 10
    out = capsys.readouterr().out
    assert "== single" in out
    assert "OK" in out
    rep = json.loads((tmp_path / "audit.json").read_text())
    assert rep["config"]["linkage"] == "parallelogram_lift"
    assert rep["modules"]["single"]["problems"] == []
    assert rep["modules"]["single"]["parts"] > 20
    assert rep["modules"]["single"]["chassis"] == {}         # one side: no chassis


# ---------------------------------------------------------------------------
# Test drive, round 5 (docs/agentlib/TESTDRIVE.md): ``spiderpig export`` on the command
# line, ``spiderpig sim`` on a stored design or an MJCF, ``report`` over every linkage
# ---------------------------------------------------------------------------


@pytest.mark.slow
def test_r5_the_export_cli_writes_a_design_and_the_sim_cli_takes_a_design_or_an_mjcf(
        store, tmp_path, capsys):
    from spiderpig.tools import export as export_cli
    from spiderpig.tools import sim_walk

    assert "export" in cli.COMMANDS
    assert cli.COMMANDS["export"][0] == "spiderpig.tools.export"
    out = tmp_path / "klann"                                                    # entry 6
    assert export_cli.main(["--linkage", "klann", "--module", "single", "--formats", "bom",
                            "mjcf", "--out", str(out), "--store", str(store.root)]) == 0
    captured = capsys.readouterr()
    assert "resolved the build options into" in captured.err
    assert "wrote" in captured.out
    assert "manifest.json" in captured.out
    for name in ("bom.md", "klann.xml", "klann.json", "manifest.json"):
        assert (out / name).is_file()
    d = api.resolve(api.spec_of(export_cli_config()), store)
    assert export_cli.main([d.id, "--formats", "bom", "--out", str(out), "--store",
                            str(store.root)]) == 0          # a stored design, cached
    with pytest.raises(SystemExit):
        export_cli.main(["--store", str(store.root)])        # neither an id nor options
    assert export_cli.main(["0123456789abcdef", "--store", str(store.root)]) == 2
    assert "error:" in capsys.readouterr().err
    # the sim CLI: a stored design runs its exported MJCF; --mjcf wants the .json beside it
    args = sim_walk._args([d.id, "--store", str(store.root)])
    assert args.config == d.config
    assert args.mjcf == out / "klann.xml"
    assert args.model is not None
    assert args.model[1]["format"] == "spiderpig-mjcf/1"
    args = sim_walk._args(["--mjcf", str(out / "klann.xml"), "--linkage", "klann",
                           "--module", "single"])
    assert args.model[0].startswith("<mujoco")
    with pytest.raises(SystemExit):
        sim_walk._args(["--mjcf", str(tmp_path / "none.xml"), "--linkage", "klann"])
    assert "--mjcf needs" in capsys.readouterr().err
    # a mechanism has nothing to walk: its mjcf is skipped with a warning, not an error
    lift = tmp_path / "lift"
    assert export_cli.main(["--linkage", "hoecken", "--formats", "bom", "mjcf", "--out",
                            str(lift), "--store", str(store.root)]) == 0
    assert "warning: mjcf: hoecken is a mechanism" in capsys.readouterr().err
    assert (lift / "bom.md").is_file()
    assert not (lift / "hoecken.xml").exists()
    rep = api.load(api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}},
                               store).id, store).reports.get("export")
    assert rep is None or any(w.startswith("mjcf: hoecken is a mechanism") for w in rep.warnings)


def export_cli_config():
    from spiderpig.config import BuildConfig

    return BuildConfig(linkage="klann", module="single")


def test_r5_the_report_covers_every_linkage_by_default_and_says_what_it_plans(tmp_path,
                                                                                caplog):
    import logging

    from spiderpig.tools import report

    out_file = tmp_path / "report.json"
    with caplog.at_level(logging.INFO, logger="linkage_report"):                # entry 7
        assert report.main(["--linkages", "hoecken", "klann", "--modules", "single", "--out",
                            str(out_file)]) == 0
    keys = [row["key"] for row in json.loads(out_file.read_text())]
    assert keys == ["hoecken", "klann"]
    assert any("klann single: planning" in r.message for r in caplog.records)
    assert any(r.message.startswith("2 linkages: hoecken, klann") for r in caplog.records)
    assert "every registered one by default" in report.__doc__
    # the default keys are every linkage, mechanisms included (the walkers alone before)
    import inspect

    src = inspect.getsource(report.main)
    assert "args.linkages or linkage.available()" in src
