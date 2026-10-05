"""ORDER.md (:mod:`spiderpig.hardware.order`): a cart per vendor, uploads per cut service,
shop supplies left out, and an unpriced line estimated from another vendor's price."""

from __future__ import annotations

from spiderpig.hardware.bom import Bom, PurchaseRow
from spiderpig.hardware.order import estimate, order_markdown


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
        _row("m3_heat_set_insert", "M3 insert", "CNC Kitchen", 4, pack_qty=100, price=10.9),
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
