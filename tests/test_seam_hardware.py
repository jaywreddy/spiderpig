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
from spiderpig.hardware.catalog import finish, get
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
    assert keys == ["al6061_1p6mm", "al6061_2mm", "al6061_2p5mm", "al6061_3p2mm",
                    "al6061_4p8mm", "al6061_6p35mm"]   # the last two a decanting cradle's
    ts = [materials.sheet(k).thickness for k in materials.aluminium_sheets("")]
    assert ts == sorted(ts)


def test_the_thinnest_sheet_doesnt_depend_on_the_catalog_being_loaded_first():
    code = ("from spiderpig import materials; "
            "print(materials.thinnest_sheet('frame'), len(materials.role_report('frame')))")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         timeout=60)
    assert out.stdout.split() == ["al5052_2mm", "8"], out.stderr[-500:]


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


def test_a_bought_parts_finish_is_the_first_its_name_or_offer_names():
    """The surface the viewer draws a bought part with (``catalog.finish``, the bake's node
    extras): the first of ``FINISHES`` its name names, else its offer's note; none for plain
    steel or an unknown key."""
    assert finish("rod6061ha_516_305") == "hard_anodized"       # not bare 6061
    assert finish("rod6061_12p7_m6_68.2") == "aluminium"
    assert finish("spring_plunger_half13") == "black_oxide"     # (its 440C ball's stainless after)
    assert finish("hairpin_cotter_2x50") == "zinc"              # its offer's note
    assert finish("m3_bhcs_6") == "stainless"                   # its offer's note: 18-8
    assert finish("m6_carriage_25") == "zinc"
    assert finish("nrv050_output_shaft_25") is None
    assert finish(None) is None
    assert finish("no_such_part") is None


def test_each_shim_thickness_is_its_own_catalog_item():
    assert bom.shim_key("shim_din988_4x8", 0.5) == "shim_din988_4x8_t0p5"
    assert bom.shim_key("shim_din988_4x8", 1.0) == "shim_din988_4x8_t1"
    assert bom.shim_key("shim_din988_3x6", 0.15) == "shim_din988_3x6_t0p15"
    assert get("shim_din988_4x8_t0p5").dims["t"] == 0.5


def test_clamped_shims_are_bought_as_stock_washers():
    """A 1 mm M3 shim is two DIN 125 washers ($0.05 each at Bolt Depot, not a $5-13 shim),
    an M4 one DIN 433; a 0.2 mm one stays a DIN 988 shim. The M3 stack is modelled and
    claimed at the washers' 7 mm (bom.shim_od), the M4 one at its 8 mm."""
    assert bom.SHIM_AS["shim_din988_3x6_t1"] == ("m3_washer", 2)
    assert bom.SHIM_AS["shim_din988_4x8_t0p5"] == ("m4_washer_433", 1)
    assert bom.shim_as_bought("shim_din988_3x6", 1.0) == f"2 x {get('m3_washer').name}"
    assert bom.shim_as_bought("shim_din988_3x6", 0.2) == "a 0.2 mm DIN 988 shim"
    assert bom.shim_od("shim_din988_3x6") == 7.0
    assert bom.shim_od("shim_din988_4x8") == 8.0
    assert bom.shim_od("shim_din988_6x12") == 12.0


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


def _keyway(t: float, slot: float):
    """A 25.25 mm bore with a key's slot out of it (the cradle's crank plate's), as a
    cutter: the slot 5.6 mm long from 11.2 mm out, ``slot`` wide."""
    bore = Cylinder(25.25 / 2, 3 * t).moved(Location((30, 30, t / 2)))
    return bore + Box(5.6, slot, 3 * t).moved(Location((30 + 14.0, 30, t / 2)))


def test_a_cradle_s_keyway_corners_are_seen_corner_by_corner():
    """A keyway's square end on its round bore: read by its arcs the bore's 12.6 mm passes
    it; read corner by corner (a decanting cradle's, ``kerf_corners``) its two sharp inside
    corners are seen against the laser's published radius (about its kerf, 0.3 mm), a
    warning. Its mouth on the bore (corners of the part pointing in) isn't one."""
    t = 3.175
    plate = _body(_plate(60, 60, t, pockets=[_keyway(t, 7.75)]), sheet="al6061_3p2mm")
    assert not [i for i in manufacture.part_issues(plate, "al6061_3p2mm")
                if i["rule"] == "corner"]
    (c,) = [i for i in manufacture.part_issues(plate, "al6061_3p2mm", kerf_corners=True)
            if i["rule"] == "corner"]
    assert (c["level"], c["value"], c["limit"]) == ("warning", 0.0, 0.3)
    assert "2 inside corners sharp" in c["detail"]
    wires = sorted(manufacture._profile(plate).wires(), key=lambda w: w.length)
    pocket = wires[0]                                   # (the outline the longer)
    assert sorted(manufacture.inside_corners(pocket)) == pytest.approx([0.0, 0.0, 12.625])


def test_corner_by_corner_a_hex_is_six_corners_and_a_dog_bone_none():
    """Corner by corner: a hex pocket's six sharp corners (the clutch's seat ring's, on a
    printed hex whose corners the nozzle rounds: a warning, harmless); the walker's
    dog-boned hex pocket has only its reliefs' 0.8 mm arcs, over the 0.3 mm."""
    from build123d import RegularPolygon, extrude

    t = 2.54
    hexagon = extrude(RegularPolygon(10.0, 6), 3 * t).moved(Location((15, 15, -t)))
    plate = _body(_plate(30, 30, t, pockets=[hexagon]))
    (c,) = [i for i in manufacture.part_issues(plate, "al6061_2p5mm", kerf_corners=True)
            if i["rule"] == "corner"]
    assert "6 inside corners sharp" in c["detail"]
    cut = BoltCrank().for_sheet("al6061_2p5mm").hex_cut((15.0, 15.0), -t, 2 * t, 0.0)
    dog = _body(_plate(30, 30, t, pockets=[cut]))
    assert not [i for i in manufacture.part_issues(dog, "al6061_2p5mm", kerf_corners=True)
                if i["rule"] == "corner"]


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
    printed = Body("ring", _plate(10, 10, 3.0), fab="printed")
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


def test_a_part_under_the_deburring_minimum_arrives_as_cut():
    """SendCutSend deburrs a part only from 0.995 x 3 in (its quoting app's limit, 2026-10-20;
    its spec tables say "1 x 3 in"): a 24 mm
    link on a cradle's 0.187 in 5052 is a warning (deburr it by hand), a 26 mm one passes,
    and a walker's sheet doesn't carry the rule (its audit is the identity gate's)."""
    t = 4.75
    narrow = _body(_plate(120, 24, t, [(12, 12, 8.2), (108, 12, 8.2)]), sheet="al5052_4p8mm")
    (d,) = [i for i in manufacture.part_issues(narrow, "al5052_4p8mm") if i["rule"] == "deburr"]
    assert (d["level"], d["value"], d["limit"]) == ("warning", [24.0, 120.0], [25.27, 76.2])
    assert "arrives as cut" in d["fix"]
    wide = _body(_plate(120, 26, t, [(13, 13, 8.2)]), sheet="al5052_4p8mm")
    assert not [i for i in manufacture.part_issues(wide, "al5052_4p8mm")
                if i["rule"] == "deburr"]
    walker = _body(_plate(120, 24, 2.54, [(12, 12, 3.0)]))
    assert not [i for i in manufacture.part_issues(walker, "al6061_2p5mm")
                if i["rule"] == "deburr"]
    assert manufacture.blank_of(narrow) == pytest.approx((24.0, 120.0), abs=1e-6)


def test_the_thick_sheets_take_their_own_smallest_part():
    """The spec tables' smallest part per thickness (2026-10-15): 0.50 x 0.75 in in 0.187
    and 0.250 in 5052, 0.50 x 0.50 in in 7075 0.190 in; the thin sheets keep 0.25 x 0.375
    in; every SendCutSend sheet carries its published thickness tolerance."""
    assert materials.sheet("al5052_4p8mm").min_part == pytest.approx((12.7, 19.05))
    assert materials.sheet("al5052_6p35mm").min_part == pytest.approx((12.7, 19.05))
    assert materials.sheet("al7075_4p8mm").min_part == pytest.approx((12.7, 12.7))
    assert materials.sheet("al5052_2mm").min_part == (0.25 * 25.4, 0.375 * 25.4)
    assert get("al5052_6p35mm").dims["thickness_tol"] == (0.686, 0.686)
    assert get("al7075_4p8mm").dims["thickness_tol"] == (0.178, 0.178)
    assert get("birch_9mm").dims["thickness_tol"] == (0.406, 0.61)
    small = _body(_plate(18, 12, 4.75), sheet="al5052_4p8mm")
    (m,) = [i for i in manufacture.part_issues(small, "al5052_4p8mm") if i["rule"] == "min_part"]
    assert m["limit"] == pytest.approx([12.7, 19.05])


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
    assert pillar_shaft(101.3) == item.key
    assert CATALOG[item.key] is item
    for bad in ("pillar_shaft_6_m3_101.35", "pillar_shaft_6_m3_7.9", "pillar_shaft_6_m3_x"):
        assert bad not in CATALOG
        assert CATALOG.get(bad) is None
        with pytest.raises(KeyError):
            get(bad)


def test_iterating_the_catalog_while_lengths_are_made_never_breaks():
    """A loop over the catalog (``spec.sheet_keys``, the MCP's cards) that makes a NETRF6
    length, in its own thread or another, walks a snapshot: no "dictionary changed size
    during iteration"."""
    import threading

    from spiderpig.hardware.catalog import CATALOG
    from spiderpig.spec import sheet_keys

    get("m3_washer")                                # (the catalog loaded)
    errors: list[BaseException] = []

    def iterate(first: int):
        try:
            for n in range(first, first + 40):
                sheet_keys()
                for _key in CATALOG:                 # the same loop makes an item too
                    CATALOG.get(f"pillar_shaft_6_m3_{n / 10:g}")
                list(CATALOG.items())
        except BaseException as e:  # noqa: BLE001 - any error fails the test, below
            errors.append(e)

    def make(first: int):
        try:
            for n in range(first, first + 300):
                CATALOG.get(f"pillar_shaft_6_m3_{n / 10:g}")
        except BaseException as e:  # noqa: BLE001 - any error fails the test, below
            errors.append(e)

    threads = [threading.Thread(target=f, args=(n,)) for f, n in
               ((iterate, 2000), (iterate, 2100), (make, 2200), (make, 2600))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert errors == []


def test_a_length_made_on_demand_takes_its_sourced_offer_first(monkeypatch):
    from spiderpig.hardware import sources
    from spiderpig.hardware.catalog import CATALOG, Offer

    key = "pillar_shaft_6_m3_77.7"
    dict.pop(CATALOG, key, None)
    page = Offer("MISUMI", "https://example.com/netrf6-77.7", "NETRF6-77.7", verified=True)
    monkeypatch.setitem(sources.SOURCES, key, (page,))
    try:
        item = get(key)
        assert item.offer == page
        assert len(item.offers) == 2                 # the item's own offer after it
    finally:
        dict.pop(CATALOG, key, None)


def test_a_screw_picks_the_shortest_priced_length_that_reaches():
    """``Screw.length_for`` (the one picker, 2026-10-14): a priced length first (Bolt Depot
    has no M3 x 14 socket head: McMaster's, unpriced, is its first offer), else any listed."""
    from spiderpig.hardware.fasteners import screw

    sk = screw("shcs", "3")
    assert 14 in sk.lengths
    assert not sk.priced(14)
    assert sk.length_for(12.2) == 16
    assert sk.length_for(12.0) == 12
    tap = screw("self_tap", "2")                   # only the 6 mm priced: 8 from the kit
    assert tap.length_for(6.5) == 8
    assert sk.length_for(1e3, longest=True) == max(sk.lengths)
    with pytest.raises(LookupError):
        sk.length_for(1e3)
