"""Seam tests of the materials' per-role sheet choice (:func:`materials.thinnest_sheet`), the
shim helpers of the BOM (:mod:`hardware.bom`) and the services' cut rules
(:mod:`manufacture`) on hand-made plates: no design, no fabrication (``no_fabricate``)."""

from __future__ import annotations

import math
import subprocess
import sys
from types import SimpleNamespace

import pytest
from build123d import Box, Cylinder, Location

from spiderpig import manufacture, materials
from spiderpig.construction.crank import BoltCrank
from spiderpig.hardware import bom
from spiderpig.hardware.catalog import get
from spiderpig.mechanism import Body

pytestmark = pytest.mark.no_fabricate


# -- the thinnest sheet per role --------------------------------------------------------------


def test_the_frame_takes_the_thinnest_5052_its_pillar_ends_and_servo_holes_allow():
    """0.063 in bends past SF 2 at a pillar's clamped end (155 N over an 84 mm bay); 0.100 in
    and up can't take the servo's 2.4 mm holes (SendCutSend's minimum hole is the
    thickness); 0.080 in passes both."""
    rows = {r["sheet"]: r["why_not"] for r in materials.role_report("frame")}
    assert materials.thinnest_sheet("frame") == "al5052_2mm"
    assert "SF 1.37 under 2" in rows["al5052_1p6mm"]
    assert (rows["al5052_2mm"], rows["al5052_2p3mm"]) == (None, None)
    assert "its 2.4 mm holes under SendCutSend's 2.54 mm minimum" in rows["al5052_2p5mm"]


def test_the_frames_bending_check_is_the_fixed_end_moment_over_the_section():
    """``F L / 8`` over ``b t^2 / 6``: 155 N x 84 mm / 8 on 27 mm of 1.6 mm plate."""
    sigma = 6 * (155 * 84 / 8) / (27 * 1.6 ** 2)
    sh = materials.sheet("al5052_1p6mm")
    why = materials.ROLES["frame"].why_not(sh)
    assert f"{sigma:.0f} MPa" in why
    assert f"SF {sh.yield_mpa / sigma:.2f}" in why


def test_the_crank_takes_6061_where_5052_would_need_a_thicker_plate_than_its_layer():
    """The hex pockets hold 2 x 0.85 N·m at SF 2 in 0.100 in 6061-T6 (3.83 N·m), not in
    0.080 in 6061 (SF 1.74) nor in 0.100 in 5052 (SF 1.58): 5052 needs 0.125 in."""
    assert materials.thinnest_sheet("crank") == "al6061_2p5mm"
    assert materials.thinnest_sheet("crank", "5052") == "al5052_3p2mm"
    check = materials.ROLES["crank"]
    assert check.hex_nm(materials.sheet("al6061_2p5mm")) == pytest.approx(3.834, abs=1e-3)
    assert "SF 1.74 under 2" in check.why_not(materials.sheet("al6061_2mm"))
    assert "SF 1.58 under 2" in check.why_not(materials.sheet("al5052_2p5mm"))


def test_the_roles_hex_rating_is_the_cranks_bearing_model():
    """``RoleCheck.hex_nm`` and the crank's own rating agree: the pocket's depth less the
    recess, each flat less the corner loss, at the plate's yield."""
    sh = materials.sheet("al6061_2p5mm")
    crank = BoltCrank().for_sheet(sh.key)
    a = 5.5 / math.sqrt(3) - crank.hex_corner_loss
    assert materials.ROLES["crank"].hex_nm(sh) == pytest.approx(
        0.75 * 276.0 * a * a * (2.54 - crank.recess_max) / 1e3)


def test_a_role_no_stock_sheet_passes_is_an_error():
    with pytest.raises(ValueError, match="no 7075 sheet passes the frame checks"):
        materials.thinnest_sheet("frame", "7075")


def test_the_aluminium_sheets_come_thinnest_first():
    keys = materials.aluminium_sheets("6061")
    assert keys == ["al6061_1p6mm", "al6061_2mm", "al6061_2p5mm", "al6061_3p2mm"]
    ts = [materials.sheet(k).thickness for k in materials.aluminium_sheets("")]
    assert ts == sorted(ts)


def test_the_thinnest_sheet_doesnt_depend_on_the_catalog_being_loaded_first():
    code = ("from spiderpig import materials; "
            "print(materials.thinnest_sheet('frame'), len(materials.role_report('frame')))")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         timeout=60)
    assert out.stdout.split() == ["al5052_2mm", "6"], out.stderr[-500:]


def test_a_gap_on_a_pin_is_one_ptfe_washer_then_shims_thickest_first():
    """1.85 mm on the 4 mm barrel: the 0.5 mm PTFE washer, 1.0 + 0.3 of shims, 0.05 left as
    play; a gap under the washer's thickness is shims alone."""
    assert materials.washer_stack(4.0, 1.85) == (
        [("ptfe_washer_4x8x0p5", 0.5), ("shim_din988_4x8", 1.0), ("shim_din988_4x8", 0.3)],
        pytest.approx(0.05))
    assert materials.washer_stack(4.0, 0.3) == ([("shim_din988_4x8", 0.3)], 0.0)


# -- the BOM's shims ------------------------------------------------------------------------------


def test_a_shim_stack_breaks_down_thickest_first():
    assert bom.shim_breakdown(0.7, (0.1, 0.2, 0.3, 0.5, 1.0)) == [0.5, 0.2]
    assert bom.shim_breakdown(2.35, (1.0, 0.5)) == [1.0, 1.0]           # 0.35 left: no step
    assert bom.shim_breakdown(3.0, (0.5, 1.0)) == [1.0, 1.0, 1.0]       # sizes in any order
    assert bom.shim_breakdown(0.05, (0.1,)) == []


def test_the_one_shim_loop_returns_what_it_leaves_and_rounds_to_a_step():
    assert bom.stack(2.35, (1.0, 0.5)) == ([1.0, 1.0], 0.35)
    assert bom.stack(1.3, (1.0, 0.5), round_to=0.5) == ([1.0, 0.5], 0.0)    # 1.3 -> 1.5
    assert bom.stack(-0.2, (0.1,)) == ([], 0.0)                             # never negative


def test_the_m3_and_m4_families_stack_in_whole_mm_and_half_steps():
    """The constructions stack the clamped M3 / M4 shims from 1.0 and 0.5 mm (DIN 433
    washers, a pair to the millimetre); the 6 x 12 family from its catalog thicknesses."""
    assert bom.stack_steps("shim_din988_3x6") == (1.0, 0.5)
    assert bom.stack_steps("shim_din988_4x8") == (1.0, 0.5)
    assert bom.stack_steps("shim_din988_6x12") == (0.1, 0.2, 0.3, 0.5, 1.0)


def test_each_shim_thickness_is_its_own_catalog_item():
    assert bom.shim_key("shim_din988_4x8", 0.5) == "shim_din988_4x8_t0p5"
    assert bom.shim_key("shim_din988_4x8", 1.0) == "shim_din988_4x8_t1"
    assert bom.shim_key("shim_din988_3x6", 0.15) == "shim_din988_3x6_t0p15"
    assert get("shim_din988_4x8_t0p5").dims["t"] == 0.5


def test_clamped_shims_are_bought_as_din433_washers():
    """A 1 mm M3 shim is two DIN 433 washers ($0.05 each, not a $5-13 shim); a 0.2 mm one
    stays a DIN 988 shim."""
    assert bom.SHIM_AS["shim_din988_3x6_t1"] == ("m3_washer_433", 2)
    assert bom.SHIM_AS["shim_din988_4x8_t0p5"] == ("m4_washer_433", 1)
    assert bom.shim_as_bought("shim_din988_3x6", 1.0) == f"2 x {get('m3_washer_433').name}"
    assert bom.shim_as_bought("shim_din988_3x6", 0.2) == "a 0.2 mm DIN 988 shim"


# -- the cut rules on hand-made plates ------------------------------------------------------------


def _plate(w: float, h: float, t: float, holes=(), pockets=()):
    part = Box(w, h, t).moved(Location((w / 2, h / 2, t / 2)))
    for x, y, d in holes:
        part = part - Cylinder(d / 2, 3 * t).moved(Location((x, y, t / 2)))
    for cut in pockets:
        part = part - cut
    return part


def _body(part, sheet="al6061_2p5mm", name="plate"):
    return Body(name, part, fab="laser", sheet=sheet)


def test_a_square_pocket_in_metal_breaks_the_corner_rule():
    """A 5.6 mm square pocket: four straight edges, sharp corners, where SendCutSend cuts
    inside corners 0.8 mm round (a warning: a square part won't seat)."""
    t = 2.54
    pocket = Box(5.6, 5.6, 3 * t).moved(Location((15, 15, t / 2)))
    (c,) = [i for i in manufacture.part_issues(_body(_plate(30, 30, t, pockets=[pocket])),
                                               "al6061_2p5mm") if i["rule"] == "corner"]
    assert (c["level"], c["value"], c["limit"]) == ("warning", 0.0, 0.8)
    assert "sharp corners" in c["detail"]


def test_the_crank_s_dog_boned_hex_pocket_passes_the_corner_rule():
    """The hex pocket's reliefs are the service's 0.8 mm inside radius: no corner issue, and
    on a 30 mm plate no web or edge issue either."""
    t = 2.54
    cut = BoltCrank().for_sheet("al6061_2p5mm").hex_cut((15.0, 15.0), -t, 2 * t, 0.0)
    issues = manufacture.part_issues(_body(_plate(30, 30, t, pockets=[cut])), "al6061_2p5mm")
    assert issues == []


def test_a_hole_under_the_minimum_is_an_error_in_metal_and_a_warning_in_acrylic():
    """SendCutSend won't cut a hole under the thickness in metal (the part comes back
    without it); Ponoko's 1 mm minimum in acrylic is a warning."""
    (m,) = manufacture.part_issues(_body(_plate(30, 30, 2.54, [(15, 15, 2.0)])), "al6061_2p5mm")
    assert (m["rule"], m["level"], m["value"], m["limit"]) == ("min_hole", "error", 2.0, 2.54)
    acrylic = _body(_plate(30, 30, 3.0, [(15, 15, 0.8)]), sheet="acrylic_3mm")
    (a,) = manufacture.part_issues(acrylic, "acrylic_3mm")
    assert (a["rule"], a["level"]) == ("min_hole", "warning")


def test_two_holes_too_close_are_measured_hole_to_hole():
    """Two 3 mm holes 5 mm apart on centres: 2.0 mm of web, under 1 x 2.54: an error."""
    plate = _plate(40, 20, 2.54, [(15, 10, 3.0), (20, 10, 3.0)])
    (e,) = manufacture.part_issues(_body(plate), "al6061_2p5mm")
    assert (e["rule"], e["level"]) == ("edge", "error")
    assert e["value"] == pytest.approx(2.0, abs=0.01)
    assert e["limit"] == pytest.approx(2 * 2.54, abs=0.01)


def test_the_dxf_rule_holds_the_contour_to_the_chord_tolerance():
    body = SimpleNamespace(name="p")
    ok = {"deviation_mm": manufacture.DXF_DEVIATION, "area_rel": manufacture.DXF_AREA_REL,
          "dxf_area_mm2": 100.0, "area_mm2": 100.0}
    assert manufacture.dxf_issue(body, ok) is None
    off = dict(ok, deviation_mm=manufacture.DXF_DEVIATION + 0.01)
    issue = manufacture.dxf_issue(body, off)
    assert (issue["rule"], issue["level"], issue["part"]) == ("dxf", "error", "p")
    area = dict(ok, area_rel=2 * manufacture.DXF_AREA_REL, dxf_area_mm2=100.2)
    assert manufacture.dxf_issue(body, area)["value"] == pytest.approx(ok["deviation_mm"],
                                                                       abs=1e-4)


def test_the_check_counts_the_rules_and_errors_over_every_laser_part():
    """Two plates and a printed part: the laser ones checked on their own sheets (or the
    default), worst first per rule; the summary not ok on an error."""
    bad = _body(_plate(30, 30, 2.54, [(15, 15, 2.0)]), name="bad")
    good = _body(_plate(30, 30, 3.0, [(15, 15, 4.0)]), sheet=None, name="good")
    printed = Body("ring", _plate(10, 10, 3.0), fab="print")
    m = manufacture.check(SimpleNamespace(bodies=[bad, good, printed]), "acrylic_3mm",
                          dxf=False)
    assert m["parts"] == 2
    assert (m["by_rule"], m["errors"]) == ({"min_hole": 1}, {"min_hole": 1})
    assert set(m["sheets"]) == {"al6061_2p5mm", "acrylic_3mm"}
    assert m["kerf"]["al6061_2p5mm"] == 0.0              # SendCutSend compensates itself
    s = manufacture.summary(m)
    assert not s["ok"]
    assert s["messages"][0].startswith("manufacture: 1 part(s) break the minimum hole rule; "
                                       "worst bad (al6061_2p5mm)")


# -- the catalog's on-demand families ----------------------------------------------------------


def test_the_netrf6_lengths_are_made_on_demand_not_at_import():
    """``crank_catalog`` registers at most 200 items at import (the NETRF6 pillar shafts'
    2,921 lengths are made when first asked for), and an asked-for length is the item."""
    code = ("from spiderpig.hardware import catalog\n"
            "from spiderpig.hardware import crank_catalog\n"
            "print(len(catalog.CATALOG))\n")
    n = int(subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                           check=True).stdout)
    assert n <= 200
    from spiderpig.hardware.catalog import CATALOG
    from spiderpig.hardware.crank_catalog import pillar_shaft

    item = get("pillar_shaft_6_m3_101.3")
    assert (item.dims["length"], item.offer.sku) == (101.3, "NETRF6-101.3")
    assert pillar_shaft(101.3) == item.key and CATALOG[item.key] is item
    for bad in ("pillar_shaft_6_m3_101.35", "pillar_shaft_6_m3_7.9", "pillar_shaft_6_m3_x"):
        assert bad not in CATALOG and CATALOG.get(bad) is None
        with pytest.raises(KeyError):
            get(bad)
