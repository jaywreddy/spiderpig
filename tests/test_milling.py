"""Milled parts (2026-10-12, phase 5): the fab method ``milled``, the user's mill
(:mod:`spiderpig.milling`, the role ``mill``), its rules and outputs, and the turntable's
bearing housings milled when a mill is named."""

from __future__ import annotations

import math

import pytest

from spiderpig import fabmethods
from spiderpig import roles as rolemod
from spiderpig.config import BuildConfig
from spiderpig.milling import CARVERA, MILLS, MillPart, Op, check, minutes, setup_sheet
from tests import cache

STOCK = "plate_6061_9p525"


def _part(**kw) -> MillPart:
    base = {"name": "p", "stock": STOCK, "thickness": 9.525, "outline_d": 40.0,
            "ops": (Op("bore", 22.0, depth=7.0, purpose="seat"), Op("bore", 18.5),
                    Op("bore", 3.4, 15.0, 0.0), Op("profile", 40.0))}
    base.update(kw)
    return MillPart(**base)


@pytest.mark.no_fabricate
def test_the_milled_method_and_the_mill_role():
    assert fabmethods.MILLED in fabmethods.made()
    assert "mill" in fabmethods.folders()
    assert fabmethods.stock(fabmethods.MILLED) == "sheet"
    assert rolemod.choices("mill") == [*sorted(MILLS), rolemod.NONE]
    assert BuildConfig(linkage="palletizer").role("mill") == ""           # none by default
    assert rolemod.get("mill").robot                     # the side's plan doesn't hold it
    assert BuildConfig().applied_roles() == {}            # nor do the walkers see it


@pytest.mark.no_fabricate
def test_the_carveras_tools_bore_what_fits_them():
    assert CARVERA.tool_for(22.0, 7.0).d == 3.175
    assert CARVERA.tool_for(2.5, 2.0).d == 2.0
    assert CARVERA.tool_for(1.5, 2.0).d == 1.0
    assert CARVERA.tool_for(0.8, 1.0) is None           # under the smallest tool
    assert CARVERA.tool_for(1.5, 5.0) is None           # deeper than the 1 mm tool's flutes


@pytest.mark.no_fabricate
def test_a_milled_part_is_drawn_from_its_operations():
    p = _part()
    solid = p.solid()
    t = p.thickness
    want = (math.pi * 20.0 ** 2 * t - math.pi * 11.0 ** 2 * 7.0
            - math.pi * 9.25 ** 2 * (t - 7.0) - math.pi * 1.7 ** 2 * t)
    assert solid.volume == pytest.approx(want, rel=1e-6)
    assert MillPart.from_dict(p.as_dict()) == p
    assert check(p, CARVERA) == []
    assert minutes(p, CARVERA) > CARVERA.setup_min
    sheet = "\n".join(setup_sheet([("MH40x9.5", 2, p)], CARVERA))
    for word in ("MH40x9.5 x2", "seat", "em_3p175"):
        assert word in sheet


@pytest.mark.no_fabricate
def test_the_mills_rules_refuse_what_it_cant_make():
    rules = {i["rule"] for i in check(_part(thickness=6.35), CARVERA)}
    assert rules == {"mill_stock"}                         # not its plate's thickness
    big = _part(outline_d=300.0, ops=(Op("profile", 300.0),))
    assert {i["rule"] for i in check(big, CARVERA)} == {"mill_bed"}
    tiny = _part(ops=(Op("bore", 0.5), Op("profile", 40.0)))
    assert {i["rule"] for i in check(tiny, CARVERA)} == {"mill_tool"}
    thin = _part(ops=(Op("bore", 3.4, 18.6, 0.0), Op("bore", 3.4, 14.5, 0.0),
                      Op("profile", 40.0)))
    walls = [i for i in check(thin, CARVERA) if i["rule"] == "mill_wall"]
    assert len(walls) == 2                                # to the edge, and between the two
    assert all(i["level"] == "error" for i in walls)


@pytest.mark.no_fabricate
def test_a_mill_mills_the_turntables_housings():
    from spiderpig.construction.robot import Turret
    from spiderpig.construction.rotary import SPACING

    cfg = BuildConfig(linkage="palletizer", roles=(("mill", "carvera"),))
    _, design = cache.cached_design(cfg)
    tp, _ = Turret().plans(design, cfg)
    housing = tp.housing(design.ctx)
    assert housing is not None
    assert housing.stock == STOCK
    assert check(housing, CARVERA) == []
    assert tp.h["hub_top"] - tp.h["hub_bot"] == pytest.approx(SPACING + 7.0)   # flush seats
    laser, _ = Turret().plans(design, BuildConfig(linkage="palletizer"))
    assert laser.housing(design.ctx) is None


@pytest.mark.slow
def test_the_milled_turret_builds_its_housings_and_their_setup(tmp_path):
    from spiderpig.construction.contract import clashes
    from spiderpig.hardware.bom import bom_from_mechanism, group_made
    from spiderpig.labels import by_body, part_types
    from spiderpig.manufacture import check as cut_check
    from spiderpig.milling import write_outputs

    cfg = BuildConfig(linkage="palletizer", roles=(("mill", "carvera"),))
    mech = cache.cached_robot(cfg, 1.0)
    milled = [b for b in mech.bodies if b.fab == fabmethods.MILLED]
    assert sorted(b.name for b in milled) == ["hub_housing_lower", "hub_housing_upper"]
    assert clashes(mech) == []
    groups = group_made(mech.bodies, fabmethods.MILLED)
    assert [g.qty for g in groups] == [2]                  # one part, one turned over
    bom = bom_from_mechanism(mech)
    rows = {r.key: r.qty for r in bom.purchased}
    assert rows[STOCK] >= 1
    assert "retaining_compound_641" in rows
    assert "epoxy_2part" not in rows or all("seat" not in w for r in bom.purchased
                                            if r.key == "epoxy_2part" for w in r.where)
    assert any("Carvera" in n for n in bom.notes)
    report = cut_check(mech, cfg.sheet, dxf=False)
    assert not [i for i in report["issues"] if i["rule"].startswith("mill_")]
    types = part_types(mech, groups={m: group_made(mech.bodies, m) for m in fabmethods.made()})
    files = write_outputs(mech, groups, by_body(types), tmp_path / "mill")
    assert [f.name for f in files] == ["MH45.5x9.5.step", "parts.csv", "SETUP.md"]
    assert "slip fit" in (tmp_path / "mill" / "SETUP.md").read_text()
