"""One task flow over every surface (2026-10-14): a design id carries from ``candidates``
through the API's, the MCP's and the CLI's exports to what a maker reads (the assembly
guide, the controller's files, ORDER.md); ``spiderpig build|guide|control|precision`` take
the stored design (its spec's roles kept); ``spiderpig candidates`` is the task's entry."""

from __future__ import annotations

import json
import typing
from types import SimpleNamespace

import pytest

from spiderpig import api
from spiderpig.config import BuildConfig
from spiderpig.store import Store
from tests import _api

KLANN_SINGLE = {"kind": "walker", "linkage": {"key": "klann"},
                "legs": {"module": "single", "sides": 1}}
# a stored design whose roles no build option says: the Strider robot on your own controller
OWN_CONTROLLER = {"kind": "walker", "linkage": {"key": "strider"},
                  "constructions": {"controller": "none"}}


# ---------------------------------------------------------------------------
# the formats
# ---------------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_every_surface_offers_the_same_export_formats(capsys):
    from spiderpig import mcp
    from spiderpig.spec import OUTPUTS
    from spiderpig.tools import export as export_cli

    have = api.formats_all()
    assert have == (*OUTPUTS, *api.MAKE_FORMATS)
    assert set(api.MAKE_FORMATS) == {"guide", "control", "order"}
    assert typing.get_args(mcp.FORMATS) == have          # the MCP's input schema
    with pytest.raises(SystemExit):
        export_cli.main(["--help"])
    assert "urdf, guide, control, order" in capsys.readouterr().out
    with pytest.raises(SystemExit) as e:
        export_cli.main(["0123456789abcdef", "--formats", "pdf"])
    assert e.value.code == 2
    with pytest.raises(ValueError, match="unknown formats"):
        api.export(api.resolve(KLANN_SINGLE, None), ["pdf"])


# ---------------------------------------------------------------------------
# control: one dispatch on what drives the design
# ---------------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_the_control_files_follow_what_drives_the_design(tmp_path, monkeypatch):
    from spiderpig.control import files, mapping

    assert set(files.WRITERS) == set(files.FILES)
    walker = files.write(BuildConfig(), tmp_path / "control")        # the Strider's servos
    assert [p.name for p in walker] == ["servos.json", "HOMING.md"]
    doc = json.loads(walker[0].read_text())
    assert [s["id"] for s in doc["servos"]] == [1, 2]
    homing = walker[1].read_text()
    assert homing.startswith("# Homing: strider double")
    assert "1. **L**:" in homing

    arm = BuildConfig(linkage="palletizer")                          # an arm's steppers
    side = SimpleNamespace(groups=[])     # its planned side (the axes' seam reads its drives)
    seen = []

    def axes(config, design=None):
        seen.append(design)
        return {n: mapping.Axis(n, r, 1.8, 16) for n, r in (("q1", 3.75), ("q2", 5.0),
                                                            ("q3", 5.0))}

    monkeypatch.setattr(mapping, "axes", axes)
    written = files.write(arm, tmp_path / "control", side)
    assert seen == [side]                                   # the caller's side: no re-plan
    assert [p.name for p in written] == ["printer.cfg", "HOMING.md"]
    assert written[0].read_text().count("[manual_stepper ") == 3
    assert "**q1**" in written[1].read_text()
    assert not (tmp_path / "control" / "servos.json").exists()   # not the other design's


@pytest.mark.no_fabricate
def test_the_control_cli_takes_a_stored_design_and_keeps_its_roles(tmp_path, capsys):
    from spiderpig.control import cli

    store = Store(tmp_path / "store")
    d = api.resolve(OWN_CONTROLLER, store)
    assert dict(d.config.roles)["controller"] == "none"
    out = tmp_path / "out"
    assert cli.main(["files", d.id, "--store", str(store.root), "--out", str(out)]) == 0
    doc = json.loads((out / "servos.json").read_text())
    assert doc["controller"] is None                 # the design's own: no controller board
    assert cli.main(["files", "--out", str(tmp_path / "opts")]) == 0
    assert json.loads((tmp_path / "opts" / "servos.json").read_text())["controller"]["key"] \
        == "esp32_bus_servo"                         # the options' default board
    capsys.readouterr()
    with pytest.raises(SystemExit):
        cli.main(["axes", "ffffffffffffffff", "--store", str(store.root)])
    assert "no design 'ffffffffffffffff'" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# the CLIs' DESIGN argument
# ---------------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_build_guide_and_precision_take_a_stored_design(tmp_path, monkeypatch, capsys):
    from spiderpig import build, precision
    from spiderpig.guide import make
    from spiderpig.stages import planning

    store = Store(tmp_path / "store")
    d = api.resolve(OWN_CONTROLLER, store)
    args = build._parse_args([d.id, "--store", str(store.root)])
    assert args.stored.id == d.id
    assert args.config == d.config                    # its roles, not the options' defaults
    assert dict(args.config.roles)["controller"] == "none"
    assert build._parse_args([d.id, "--store", str(store.root), "--side-only"]).config \
        == d.config.side_config()
    assert build._parse_args([]).stored is None

    side = SimpleNamespace(plan=None)
    monkeypatch.setattr(planning, "plan_stored", lambda design: side)   # (its stored plan)
    got: dict = {}

    def guide(config, store_, out, **kw):
        got["guide"] = (config, kw["design"])
        return make.GuideReport(out / "ASSEMBLY.pdf")

    monkeypatch.setattr(make, "build_guide", guide)
    assert make.main([d.id, "--store", str(store.root), "--out", str(tmp_path / "g")]) == 0
    assert got["guide"] == (d.config, side)

    def budget(config, **kw):
        got["precision"] = config
        return SimpleNamespace(lines=lambda: ["budget"])

    monkeypatch.setattr(precision, "precision", budget)
    assert precision.main([d.id, "--store", str(store.root), "--planar"]) == 0
    assert got["precision"].roles == d.config.roles
    assert got["precision"].robot is False            # one side's budget, as by the options
    capsys.readouterr()
    with pytest.raises(SystemExit):
        make.main(["ffffffffffffffff", "--store", str(store.root)])
    assert "no design" in capsys.readouterr().err


@pytest.mark.no_fabricate
def test_the_manifest_of_a_stored_designs_build_names_it(tmp_path):
    from spiderpig import build

    store = Store(tmp_path / "store")
    d = api.resolve(OWN_CONTROLLER, store)
    args = build._parse_args([d.id, "--store", str(store.root)])
    build._write_manifest(tmp_path, args.config, args)
    assert json.loads((tmp_path / "manifest.json").read_text())["design"] == d.id
    args = build._parse_args([d.id, "--store", str(store.root), "--kerf", "0.3"])
    build._write_manifest(tmp_path, args.config, args)        # another kerf: another design
    assert json.loads((tmp_path / "manifest.json").read_text())["design"] != d.id


# ---------------------------------------------------------------------------
# spiderpig candidates
# ---------------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_the_candidates_cli_ranks_a_task_and_resolves_the_best(tmp_path, monkeypatch, capsys):
    from spiderpig.api import tasks
    from spiderpig.tools import candidates

    monkeypatch.setattr(tasks, "scale_params_table", lambda store=None: {})
    task = tmp_path / "task.json"
    task.write_text(json.dumps({"do": "place", "area_mm": [200, 150],
                                "repeatability_mm": 0.1}))
    store = tmp_path / "store"
    assert candidates.main([str(task), "--store", str(store), "--json"]) == 0
    doc = json.loads(capsys.readouterr().out)
    best = doc["candidates"][0]
    assert best["kind"] == "arm"
    assert best["design"]                              # resolved into the store (--resolve 1)
    assert api.load(best["design"], store).config.linkage == best["linkage"]
    assert all(c["design"] is None for c in doc["candidates"][1:])
    assert candidates.main([str(task), "--store", str(store), "--limit", "1"]) == 0
    text = capsys.readouterr().out
    assert f"design {best['design']}" in text
    assert f"next: spiderpig build {best['design']}" in text
    task.write_text(json.dumps({"do": "fly"}))
    assert candidates.main([str(task), "--store", str(store)]) == 2
    assert candidates.main([str(tmp_path / "none.json")]) == 2


@pytest.mark.no_fabricate
def test_the_cli_lists_candidates_and_says_what_each_command_does(capsys):
    from spiderpig import cli

    assert cli.main(["--help"]) == 0
    out = capsys.readouterr().out
    assert "candidates" in out
    lines = {ln.split()[0]: ln for ln in out.splitlines()[3:]}
    assert "walker" not in lines["sim"].split(":")[0]
    assert "urdf" in lines["export"]
    assert "servos.json" in lines["control"]


# ---------------------------------------------------------------------------
# the export: guide, control, order
# ---------------------------------------------------------------------------


@pytest.mark.slow
def test_the_export_writes_the_guide_the_control_files_and_the_order(tmp_path, monkeypatch):
    from spiderpig.guide import make

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)
    seen: dict = {}

    def guide(config, store, out, **kw):          # (the guide's own pipeline: test_guide.py)
        seen.update(config=config, store=store, side=kw.get("design"))
        for name in ("ASSEMBLY.pdf", "ASSEMBLY.md"):
            (out / name).write_text(name)
        return make.GuideReport(out / "ASSEMBLY.pdf")

    monkeypatch.setattr(make, "build_guide", guide)
    store = Store(tmp_path / "store")
    d = api.resolve(KLANN_SINGLE, store)
    _api.seed(d.config)
    out = tmp_path / "out"
    rep = api.export(d, ["guide", "control", "order"], out)
    assert rep.ok, rep.failures
    names = {str(p.relative_to(out)) for p in out.rglob("*") if p.is_file()}
    assert {"ASSEMBLY.pdf", "ASSEMBLY.md", "control/servos.json", "control/HOMING.md",
            "ORDER.md", "laser/parts/order.csv", "print/parts.csv", "manifest.json"} <= names
    assert seen["config"] == d.config
    assert seen["store"] == store
    assert seen["side"] is d.side
    order = (out / "ORDER.md").read_text()
    assert "laser/parts/" in order
    assert "## Buy" in order
    manifest = json.loads((out / "manifest.json").read_text())
    assert manifest["formats"] == ["guide", "control", "order"]
    assert {"control/servos.json", "ORDER.md", "ASSEMBLY.pdf"} <= set(manifest["files"])
    assert manifest["bom"]["items"] > 0
    assert api.export(d, ["control"], out).files == rep.files      # the prior export, reused
