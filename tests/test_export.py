"""End-to-end test of the CLI (``spiderpig build``, ``spiderpig/build.py``): STEP / STL /
print STLs / DXF / BOM."""

from __future__ import annotations

import csv
import json
import math

import ezdxf
import pytest

from spiderpig import build as cli

pytestmark = pytest.mark.slow


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
    assert {"tie_screw_half0.stl", "tie_insert_half0.stl"} <= files
    assert any(f.startswith("pin_C_seg") for f in files)
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
    for word in ("sts3215", "printed", "acrylic_3mm", "jansen", "strider"):
        assert word in out


def test_linkage_flag():
    """``--linkage`` picks the linkage; its own parameters are the ones ``--proportion`` takes."""
    args = cli._parse_args(["--linkage", "jansen", "--module", "double", "--proportion", "m=14"])
    assert (args.config.linkage, args.config.module) == ("jansen", "double")
    assert args.config.proportions == (("m", 14.0),)
    assert args.name == "jansen"
    assert cli._parse_args([]).name == "klann"
    for bad in (["--linkage", "octopus"], ["--linkage", "jansen", "--proportion", "DF=2"]):
        with pytest.raises(SystemExit):
            cli._parse_args(bad)


def test_design_flags():
    """``--phases`` (degrees) and ``--proportion NAME=VALUE`` reach the BuildConfig."""
    args = cli._parse_args(["--phases", "0,175,180,355", "--proportion", "DF=2.4",
                            "--proportion", "OB=1.121"])
    assert args.config.phases == pytest.approx(
        tuple(math.radians(p) for p in (0, 175, 180, 355)))
    assert args.config.proportions == (("DF", 2.4),)          # Klann's OB is no override
    assert cli._parse_args(["--phases", "0,180,90,270"]).config.phases is None
    assert cli._parse_args(["--module", "decker", "--phases", "0,180"]).config.phases == \
        pytest.approx((0.0, math.pi))
    for bad in (["--module", "single", "--phases", "0,90"], ["--phases", "0,x,1,2"],
                ["--proportion", "XX=1"], ["--proportion", "OB"], ["--proportion", "OB=-1"]):
        with pytest.raises(SystemExit):
            cli._parse_args(bad)


def test_unassemblable_design_is_refused(tmp_path, capsys):
    assert cli.main(["--module", "single", "--proportion", "MC=0.3", "--no-dxf",
                     "--out", str(tmp_path)]) == 2
    assert "C can't be placed" in capsys.readouterr().err
