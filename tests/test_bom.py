"""Tests for :mod:`hardware.bom` (independent of the real catalog data)."""

from __future__ import annotations

import csv

import pytest
from build123d import Box

from hardware import catalog
from hardware.bom import BomLine, bom_from_mechanism
from hardware.catalog import Item, Offer, pick_length, register
from mechanism import Body, Mechanism


@pytest.fixture(autouse=True)
def _fake_items(monkeypatch):
    monkeypatch.setattr(catalog, "_LOADED", True)  # don't import the real data modules
    monkeypatch.setattr(catalog, "CATALOG", {})
    register(
        Item("test_bolt", "M3 x 12 test bolt", "fastener",
             (Offer("BigVendor", "https://example.com/bolt", "B-12", pack_qty=100,
                    price_usd=9.5, verified=True),)),
        Item("test_glue", "Test glue", "adhesive",
             (Offer("BigVendor", "https://example.com/glue", pack_qty=1, price_usd=6.0),)),
    )


def _mech() -> Mechanism:
    return Mechanism(
        name="m",
        bodies=[
            Body("link", part=Box(10, 20, 3), fab="laser"),
            Body("pin", part=Box(2, 2, 10), fab="printed"),
            *[Body(f"bolt{i}", part=Box(1, 1, 12), fab="purchased", bom_key="test_bolt")
              for i in range(3)],
        ],
        meta={"sheet_name": "3 mm acrylic"},
        bom_extras=[BomLine("test_bolt", 2, "frame"), BomLine("test_glue", 1, "crank")],
    )


def test_purchases_group_by_key_and_round_up_to_packs():
    bom = bom_from_mechanism(_mech())
    rows = {r.key: r for r in bom.purchased}
    assert rows["test_bolt"].qty == 5
    assert rows["test_bolt"].packs == 1                 # 5 bolts fit one pack of 100
    assert rows["test_bolt"].cost_usd == pytest.approx(9.5)
    assert rows["test_glue"].packs == 1
    assert bom.cost_usd == pytest.approx(15.5)
    assert [r.category for r in bom.purchased] == ["fastener", "adhesive"]


def test_made_parts_are_listed_by_method():
    bom = bom_from_mechanism(_mech())
    assert {(m.name, m.method) for m in bom.made} == {("link", "laser"), ("pin", "printed")}
    laser = next(m for m in bom.made if m.method == "laser")
    assert laser.material == "3 mm acrylic"


def test_writers(tmp_path):
    bom = bom_from_mechanism(_mech(), title="demo")
    paths = bom.write(tmp_path)
    assert [p.name for p in paths] == ["bom.csv", "bom.md", "bom.json"]
    with open(paths[0]) as f:
        rows = list(csv.DictReader(f))
    bolt = next(r for r in rows if r["item"] == "M3 x 12 test bolt")
    assert bolt["qty"] == "5"
    assert bolt["url"] == "https://example.com/bolt"
    assert bolt["link_verified"] == "yes"
    md = paths[1].read_text()
    assert "[BigVendor B-12](https://example.com/bolt)" in md
    assert "(unverified link)" in md          # the glue offer isn't verified
    assert "$15.50" in md


def test_unknown_key_is_an_error():
    mech = Mechanism("m", bom_extras=[BomLine("nope", 1)])
    with pytest.raises(KeyError, match="nope"):
        bom_from_mechanism(mech)


def test_pick_length():
    assert pick_length(11.2, (6, 8, 10, 12, 16)) == 12
    assert pick_length(12.0, (6, 8, 10, 12, 16)) == 12
    with pytest.raises(ValueError, match="no standard length"):
        pick_length(40, (6, 8, 10))
