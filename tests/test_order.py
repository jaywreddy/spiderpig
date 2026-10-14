"""ORDER.md (:mod:`spiderpig.hardware.order`): a cart per vendor, uploads per cut service,
shop supplies left out, and an unpriced line estimated from another vendor's price; the
user's three order designs' BOM (recorded, ``tests/fixtures/hardware/``) and order list."""

from __future__ import annotations

import math
import re

import pytest

from spiderpig.hardware.bom import Bom, PurchaseRow
from spiderpig.hardware.order import estimate, order_markdown
from tests import _strength, cache
from tests.tiers import quick


def _row(key: str, name: str, vendor: str, qty: float, pack_qty: int = 1,
         price: float | None = None, url: str = "https://example.com/p") -> PurchaseRow:
    return PurchaseRow(key=key, name=name, category="fastener", qty=qty, where=["here"],
                       vendor=vendor, url=url, sku="SKU", pack_qty=pack_qty, packs=1,
                       pack_price_usd=price, verified=True, alternatives=[])


def test_an_unpriced_line_is_estimated_from_a_priced_alternative():
    nut = _row("m3_nut", "M3 hex nut", "McMaster-Carr", 6, pack_qty=100)   # no price shown
    got = estimate(nut)
    assert got is not None
    usd, vendor = got
    assert vendor == "Bolt Depot"
    assert usd > 0                                                # one pack of 100
    assert estimate(_row("no_such_item", "x", "y", 1)) is None


def test_order_markdown_carts_uploads_and_shop_supplies():
    bom = Bom(purchased=[
        _row("m3_nut", "M3 hex nut", "McMaster-Carr", 6, pack_qty=100),
        _row("m3_washer", "M3 washer", "CNC Kitchen", 4, pack_qty=100, price=10.9),
        _row("pla_filament", "PLA filament", "Prusa", 0.03, price=29.99),
        _row("al5052_2mm", "5052 sheet", "SendCutSend", 1, price=18.0),
    ], made=[])
    laser = [{"service": "SendCutSend", "sheet": "al5052_2mm", "material": "5052 sheet",
              "thickness_mm": 2.032, "file": "SendCutSend_al5052_2mm/L-torso_x2.dxf",
              "qty": 2, "size_mm": "157.0 x 99.9", "parts": "L.torso R.torso"}]
    md = order_markdown(bom, laser, [], title="t", build_dir="b")
    assert "### McMaster-Carr" in md
    assert "### CNC Kitchen" in md
    assert "≈$" in md                                             # the nut's estimate
    assert "(Bolt Depot)" in md
    assert "$10.90" in md
    assert "From the shop" in md
    assert "PLA filament" in md.split("From the shop")[1]
    assert "### SendCutSend" in md
    assert "L-torso_x2.dxf" in md
    assert "### Prusa" not in md                                  # on hand, not a cart
    assert "5052 sheet |" not in md.split("## Cut")[0]            # a cut sheet is its upload


def test_a_sheet_no_service_cuts_is_bought_and_a_shared_pack_names_its_rows():
    ply = _row("plywood_3mm", "3 mm Baltic birch plywood", "Woodpeckers", 2, price=3.1)
    m2_10 = _row("m2_bhcs_10", "M2 x 10 screw", "Amazon", 4, pack_qty=100, price=9.0)
    m2_8 = _row("m2_bhcs_8", "M2 x 8 screw", "Amazon", 12, pack_qty=100, price=9.0)
    m2_8.same_pack_as = m2_10.name
    bom = Bom(purchased=[ply, m2_10, m2_8], made=[])
    laser = [{"service": "any", "sheet": "plywood_3mm", "material": "plywood",
              "thickness_mm": 3.0, "file": "any_plywood_3mm/b1_x2.dxf", "qty": 2,
              "size_mm": "50 x 10", "parts": "b1"}]
    md = order_markdown(bom, laser, [])
    buy = md.split("## Cut")[0]
    assert "3 mm Baltic birch plywood" in buy                      # no service sells it
    assert "also M2 x 8 screw: need 12" in buy                     # in the M2 x 10's pack
    assert "### any\n" not in md


def test_the_bom_total_leaves_out_the_shop_supplies_on_hand():
    bom = Bom(purchased=[_row("m3_nut", "M3 nut", "Bolt Depot", 6, price=2.39),
                         _row("pla_filament", "PLA", "Prusa", 0.3, price=29.99),
                         _row("threadlocker_243", "243", "McMaster-Carr", 0.1)], made=[])
    assert bom.cost_usd == 2.39
    assert [r.key for r in bom.unpriced] == []


def test_the_bom_total_leaves_out_a_sheet_a_service_cuts():
    """A sheet a service cuts (its item's ``service``) is that service's upload: ORDER.md
    buys none, so neither bom.md nor bom.json totals it (the 3 mm acrylic's Inventables
    sheet was in the BOM's total but no cart, 2026-10-08); a sheet no service cuts is
    bought and totalled."""
    from spiderpig.hardware.bom import cut_by

    acrylic = _row("acrylic_3mm", "3 mm acrylic", "Inventables", 1, pack_qty=2, price=10.99)
    al = _row("al5052_2mm", "5052 sheet", "SendCutSend", 1, price=18.0)
    ply = _row("plywood_3mm", "3 mm plywood", "Woodpeckers", 2, price=3.1)
    bom = Bom(purchased=[_row("m3_nut", "M3 nut", "Bolt Depot", 6, price=2.39), acrylic, al,
                         ply], made=[])
    assert cut_by("acrylic_3mm")
    assert cut_by("al5052_2mm") == "SendCutSend"
    assert not cut_by("plywood_3mm")
    assert bom.cost_usd == pytest.approx(2.39 + 3.1)
    assert bom.as_dict()["cost_usd"] == pytest.approx(2.39 + 3.1)
    assert "Estimated total: **$5.49**" in bom.markdown()
    md = order_markdown(bom, [], [])
    assert "Purchases: **$5.49**" in md


# -- the user's three order designs: their BOM (recorded) and its order list ---------------


def _bom_of(doc: dict) -> Bom:
    """A :class:`Bom` again from its ``as_dict()`` (the recorded BOM)."""
    from dataclasses import fields

    from spiderpig.hardware.bom import CutRow, MadeRow

    names = {f.name for f in fields(PurchaseRow)}
    return Bom(purchased=[PurchaseRow(**{k: v for k, v in r.items() if k in names})
                          for r in doc["purchased"]],
               made=[MadeRow(**m) for m in doc["made"]], title=doc["title"], notes=doc["notes"],
               printed_g=doc["printed_g"],
               cutting=[CutRow(**c) for c in doc.get("cutting", [])])


def _purchases(doc: dict) -> dict[str, dict]:
    """The purchase rows by key; a filament's quantity (grams of the printed parts' volume,
    summed in another order without the grouping) to 1e-6 of a spool."""
    return {r["key"]: dict(r, qty=round(r["qty"], 6) if r["category"] != "filament"
                           else pytest.approx(r["qty"], abs=1e-6)) for r in doc["purchased"]}


@pytest.mark.parametrize("name", quick(_strength.ORDER_DESIGNS, ["strider_double"]))
def test_the_order_designs_buy_what_the_robot_needs(name):
    """The robot's purchases (the fabrication cache's robot, ``group=False``: the grouping
    only merges made parts) are the recorded BOM's, row for row: what to buy, how many,
    which packs, where it is used, at what price. (The quads in the full tier: a robot of
    each in the cache.)"""
    from spiderpig.hardware.bom import bom_from_mechanism

    golden = _strength.bom(name)
    live = bom_from_mechanism(cache.cached_robot(_strength.ORDER_DESIGNS[name]), group=False)
    got = live.as_dict()
    assert [r["key"] for r in got["purchased"]] == [r["key"] for r in golden["purchased"]]
    want = _purchases(golden)
    for r in got["purchased"]:
        assert r == want[r["key"]], r["key"]
    assert got["notes"] == golden["notes"]
    assert got["cuts"] == golden["cuts"]
    assert got["cost_usd"] == pytest.approx(golden["cost_usd"], abs=1e-9)
    assert sum(m["qty"] for m in got["made"]) == sum(m["qty"] for m in golden["made"])


@pytest.mark.parametrize("name", list(_strength.ORDER_DESIGNS))
def test_the_order_designs_order_list(name):
    """ORDER.md of the recorded BOM: every line bought is in its vendor's cart, a pack
    shared by two rows is bought once, the shop supplies on hand (filament, threadlocker)
    are listed apart and in no total, and the carts add up to the BOM's total."""
    from spiderpig.hardware.bom import ON_HAND

    doc = _strength.bom(name)
    bom = _bom_of(doc)
    assert bom.cost_usd == pytest.approx(doc["cost_usd"], abs=1e-9)
    md = order_markdown(bom, [], [], title=name)
    buy, _, rest = md.partition("## From the shop")
    carts = {}
    for section in buy.split("\n### ")[1:]:
        vendor = re.match(r"(.*) \(\d+ lines?[,)]", section).group(1)
        carts[vendor] = section
    bought = [r for r in bom.purchased if not r.same_pack_as and r.key not in ON_HAND]
    assert bought
    for r in bought:
        assert f"| {r.name}" in carts[r.vendor or "(no vendor)"], (r.key, r.vendor)
    for r in bom.purchased:
        if r.same_pack_as:                      # in its lead row's pack, not bought again
            assert f"also {r.name}: need {r.qty:g}" in buy
        if r.key in ON_HAND:
            assert f"* {r.name}:" in rest
            assert f"| {r.name} |" not in buy
    packs = [(r.vendor, r.sku) for r in bought if r.sku]
    assert len(packs) == len(set(packs))        # one product bought once
    total = sum(r.cost_usd or 0.0 for r in bought)
    assert total == pytest.approx(bom.purchases_usd, abs=1e-6)
    assert f"Purchases: **${total:.2f}**" in md
    # the cutting, the material included, and the grand total (shipping apart)
    assert bom.cutting
    assert bom.cost_usd == pytest.approx(total + bom.cutting_usd, abs=1e-6)
    assert f"**Total ≈${total + bom.cutting_usd:.2f}**, before shipping" in md
    # the HV LiPo's charging warning on its line and the charger's, and before ordering
    for key in ("lipo_2s_450", "ip2326_charger"):
        r = next(r for r in bom.purchased if r.key == key)
        assert f"| {r.name} — **HV LiPo: charge as 2S, 8.4 V; never the 3S / 12.6 V" in buy
    assert "never its 3S / 12.6 V jumper setting" in md.split("## Before you order")[1]


def test_cutting_is_an_area_and_cut_length_estimate_fitted_to_sendcutsends_quotes():
    """bom.cut_estimate, fitted on SendCutSend's live quotes of two designs: the Strider
    double's 47 parts (2026-10-08, USD 117.14: acrylic 49.44, aluminium 67.70) and the
    palletizer arm's 18 (2026-10-12, USD 163.01), each part's area and cut length (cm)
    measured on the DXF quoted. 5052 pays for its cut length (refit 2026-10-14): the
    Strider's hole-dense aluminium within 3 % (it was 8.9 % low on the area alone), the arm
    within 1 %, 6061 and acrylic within 3 % a sheet, a 5052 sheet within 10 %; 6061 and
    acrylic have no cut term (acrylic's small parts all quoted the 2.00 floor; 6061's eight
    prices don't support one). A part at one off costs at least the sheet's least (3 mm
    acrylic 2.00, 5052 2.70, 6061 3.20), at 4 off SendCutSend's break (1.33 of 2.00: linear
    between its 2 and 10 off prices)."""
    from spiderpig.hardware.bom import cut_estimate as priced
    from spiderpig.hardware.bom import same_files

    def cut_estimate(key, parts) -> float:      # every sheet here is priced
        out = priced(key, parts)
        assert out is not None, key
        return out

    acrylic = cut_estimate("acrylic_3mm", [(4.58, 8), (16.7, 4), (11.0, 4), (8.66, 4),
                                           (16.7, 4), (11.0, 4), (8.65, 4), (93.1, 1)])
    al = (cut_estimate("al5052_2mm", [(51.84, 2, 88.0), (42.56, 2, 61.9)])
          + cut_estimate("al5052_2p3mm", [(16.4, 2, 29.6), (16.17, 2, 30.2)])
          + cut_estimate("al6061_2p5mm", [(7.98, 2, 21.3), (5.09, 2, 15.7), (8.63, 2, 24.5)]))
    assert acrylic == pytest.approx(49.44, rel=0.05)
    assert al == pytest.approx(67.70, rel=0.03)
    # the area alone (no cut length given) prices the hole-dense torso far lower
    assert cut_estimate("al5052_2mm", [(51.84, 1)]) < 0.75 * cut_estimate(
        "al5052_2mm", [(51.84, 1, 88.0)])
    # the arm's parts (their DXFs' net areas and cut lengths), SendCutSend's quote a sheet
    arm = {"al5052_3p2mm": ([(221.6, 1, 91.9), (228.28, 1, 75.5), (194.28, 1, 61.4)], 68.61),
           "al5052_2mm": ([(73.98, 2, 88.9), (98.02, 1, 46.4), (31.61, 2, 38.6)], 50.31),
           "al6061_3p2mm": ([(24.71, 1, 45.0), (32.64, 1, 56.1)], 14.49),
           "acrylic_5p4mm": ([(19.42, 1, 37.0), (17.18, 1, 35.9), (17.87, 1, 34.0),
                              (20.11, 1, 39.3), (10.99, 1, 24.9), (11.52, 1, 26.6)], 25.60),
           "acrylic_3mm": ([(0.46, 1, 6.1), (0.55, 1, 7.3)], 4.00)}
    est = {k: cut_estimate(k, parts) for k, (parts, _) in arm.items()}
    for k, (_, quoted) in arm.items():          # a 5052 sheet: its plates' scatter
        assert est[k] == pytest.approx(quoted, rel=0.03 if "5052" not in k else 0.10), k
    assert est["al5052_3p2mm"] + est["al5052_2mm"] == pytest.approx(68.61 + 50.31, rel=0.02)
    assert sum(est.values()) == pytest.approx(163.01, rel=0.01)
    assert cut_estimate("acrylic_3mm", [(0.0, 1)]) == 2.00
    assert cut_estimate("acrylic_3mm", [(5.0, 4)]) == pytest.approx(4 * 1.33, abs=0.01)
    assert cut_estimate("acrylic_3mm", [(5.0, 4, 80.0)]) == cut_estimate("acrylic_3mm",
                                                                         [(5.0, 4)])
    assert cut_estimate("al6061_2p5mm", [(0.0, 1)]) == 3.20
    assert cut_estimate("al5052_2mm", [(0.0, 1)]) == 2.70
    assert priced("acrylic_3mm_ponoko", [(10.0, 1)]) is None      # no rates there
    # an ungrouped BOM's rows of one shape are one file's quantity (area and cut alike)
    assert same_files([(16.7, 1), (4.58, 1), (16.7001, 1)]) == [(4.58, 1), (16.7, 2)]
    assert same_files([(16.7, 1, 30.0), (16.7, 1, 40.0), (16.7, 2, 30.0)]) == [
        (16.7, 3, 30.0), (16.7, 1, 40.0)]


def test_a_parts_cut_length_is_every_contour_of_its_face():
    """bom.cut_length_cm: a 40 x 20 mm plate 3 mm thick with a 4 mm hole cuts its outline
    and its hole, 120 + 4 pi mm; the BOM's made rows carry it, the cutting rows sum it."""
    from build123d import Box, Cylinder

    from spiderpig.hardware.bom import MadeRow, cut_length_cm, cutting_rows

    plate = Box(40, 20, 3) - Cylinder(2, 3)
    assert cut_length_cm(plate) == pytest.approx((120 + 4 * math.pi) / 10, abs=1e-3)
    rows = [MadeRow("a", "laser", "", "", 40 * 20 * 0.3 / 1000 * 10, qty=2,
                    sheet="al5052_2mm", cut_cm=13.257)]
    cut = cutting_rows(rows)[0]
    assert cut.cut_cm == pytest.approx(26.5, abs=0.06)
    assert cut.usd is not None
    assert cut.usd > 2 * 0.78 * 0.104 * 13.257


@pytest.mark.slow
@pytest.mark.fixture_regen
@pytest.mark.parametrize("name", list(_strength.ORDER_DESIGNS))
def test_the_order_designs_bom_current(name):
    """The BOM of each order design, grouped (the made parts compared shape by shape), is
    the recorded one (``tests/fixtures/hardware/bom_<design>.json``): what the user orders
    from changes only on purpose."""
    _strength.assert_current("hardware", f"bom_{name}", _strength.make_bom(name))
