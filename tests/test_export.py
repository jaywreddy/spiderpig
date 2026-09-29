"""End-to-end test of the CLI (``main.py``): STEP / STL / print STLs / DXF / BOM."""

from __future__ import annotations

import csv
import json

import ezdxf
import pytest

import main as cli


@pytest.fixture(scope="module")
def single_out(tmp_path_factory):
    out = tmp_path_factory.mktemp("single")
    assert cli.main(["--module", "single", "--out", str(out), "--name", "walker"]) == 0
    return out


def test_whole_robot_step_and_stl(single_out):
    for name in ("walker.step", "walker.stl"):
        path = single_out / name
        assert path.stat().st_size > 10_000, name


def test_one_stl_per_printed_part_with_quantities(single_out):
    with open(single_out / "print" / "parts.csv") as f:
        rows = list(csv.DictReader(f))
    assert rows
    for r in rows:
        assert (single_out / "print" / r["file"]).stat().st_size > 1000
        if int(r["mirrored"]):
            assert "mirrored" in r["print"]
            assert (single_out / "print" / r["file"].replace(".stl", "_mirrored.stl")).exists()
    files = {r["file"] for r in rows}
    assert {"pin_C.stl", "tie_screw_half0.stl", "tie_insert_half0.stl"} <= files
    by_file = {r["file"]: int(r["qty"]) for r in rows}
    assert by_file["tie_screw_half0.stl"] == by_file["tie_insert_half0.stl"] == 4
    # left and right side parts are one row each: every part is printed twice or more
    assert all(q >= 2 for q in by_file.values())


def test_dxf_sheets_hold_every_laser_part(single_out):
    sheets = sorted((single_out / "laser").glob("walker_sheet_*.dxf"))
    assert sheets
    with open(single_out / "laser" / "walker_sheet_parts.csv") as f:
        placed = [r["part"] for r in csv.DictReader(f)]
    bom = json.loads((single_out / "bom.json").read_text())
    laser = [n for m in bom["made"] if m["method"] == "laser" for n in m["names"]]
    assert sorted(placed) == sorted(laser)
    assert {"L.b1", "R.b1", "L.torso", "R.frame_outer", "centre_plate1"} <= set(placed)
    outlines = circles = 0
    for s in sheets:
        doc = ezdxf.readfile(str(s))
        assert doc.units == ezdxf.units.MM
        for e in doc.modelspace():
            assert e.dxf.layer == "CUT"
            outlines += e.dxftype() == "LWPOLYLINE"
            circles += e.dxftype() == "CIRCLE"
    assert outlines >= len(placed)       # an outline per part, plus rectangular cut-outs
    assert circles > len(placed)


def test_bom_lists_purchases_sheets_and_filament(single_out):
    bom = json.loads((single_out / "bom.json").read_text())
    keys = {r["key"]: r for r in bom["purchased"]}
    assert keys["servo_sts3215"]["qty"] == 2
    assert keys["acrylic_3mm"]["qty"] >= 1            # laser sheets
    assert 0 < keys["pla_filament"]["qty"] < 1        # a fraction of a spool
    assert keys["m3_heat_set_insert"]["qty"] == 4
    assert any(k.startswith("m2_self_tap_") for k in keys)
    assert bom["cost_usd"] > 0
    md = (single_out / "bom.md").read_text()
    assert "## Laser-cut" in md
    assert "## 3D printed" in md
    assert (single_out / "bom.csv").exists()


def test_side_only_without_dxf(tmp_path):
    assert cli.main(["--module", "single", "--side-only", "--no-dxf", "--out", str(tmp_path)]) == 0
    bom = json.loads((tmp_path / "bom.json").read_text())
    keys = {r["key"] for r in bom["purchased"]}
    assert "servo_sts3215" in keys
    assert "m3_heat_set_insert" not in keys
    assert not (tmp_path / "laser").exists()


def test_list(capsys):
    assert cli.main(["--list"]) == 0
    out = capsys.readouterr().out
    for word in ("sts3215", "printed", "acrylic_3mm"):
        assert word in out
