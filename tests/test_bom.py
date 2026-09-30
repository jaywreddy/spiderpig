"""Tests for :mod:`hardware.bom` (independent of the real catalog data)."""

from __future__ import annotations

import csv

import pytest
from build123d import Axis, Box, Plane, Pos

from spiderpig.hardware import catalog
from spiderpig.hardware.bom import BomLine, bom_from_mechanism, congruent, group_made
from spiderpig.hardware.catalog import Item, Offer, pick_length, register
from spiderpig.mechanism import Body, Mechanism


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


def _chiral():
    """A 3D corner with three different arms: not congruent to its mirror image."""
    return (Box(10, 2, 2).moved(Pos(5, 0, 0)) + Box(2, 6, 2).moved(Pos(0, 3, 0))
            + Box(2, 2, 4).moved(Pos(0, 0, 2)))


def test_filament_line_from_printed_volume():
    register(Item("test_pla", "Test PLA, 1 kg", "filament",
                  (Offer("BigVendor", "https://example.com/pla", "PLA-1", price_usd=20.0),),
                  dims={"density": 1.25, "spool_g": 1000.0}))
    mech = _mech()
    mech.meta["filament"] = "test_pla"
    bom = bom_from_mechanism(mech)
    row = next(r for r in bom.purchased if r.key == "test_pla")
    grams = 2 * 2 * 10 / 1000 * 1.25                 # the 2 x 2 x 10 mm printed pin
    assert row.qty == pytest.approx(grams / 1000, abs=1e-3)
    assert row.packs == 1
    assert bom.printed_g == pytest.approx(grams)
    assert [r.category for r in bom.purchased][-2:] == ["filament", "adhesive"]


def test_one_product_covering_several_rows_is_bought_once():
    kit = Offer("BigVendor", "https://example.com/kit", "KIT-1", pack_qty=100, price_usd=12.0)
    register(Item("test_screw_6", "Test screw 6", "fastener", (kit,)),
             Item("test_screw_8", "Test screw 8", "fastener", (kit,)))
    mech = Mechanism("m", bom_extras=[BomLine("test_screw_6", 8), BomLine("test_screw_8", 4)])
    bom = bom_from_mechanism(mech)
    rows = {r.key: r for r in bom.purchased}
    assert rows["test_screw_8"].same_pack_as == "Test screw 6"
    assert rows["test_screw_8"].cost_usd == 0.0
    assert bom.cost_usd == pytest.approx(12.0)


def test_identical_and_mirrored_parts_are_grouped():
    part = _chiral()
    moved = part.rotate(Axis.Z, 70).moved(Pos(40, -3, 9))
    mirrored = part.mirror(Plane.XY).moved(Pos(-30, 0, 0))
    assert congruent(part, moved) == "same"
    assert congruent(part, mirrored) == "mirror"
    assert congruent(part, Box(10, 2, 2)) is None
    bodies = [Body(n, part=p, fab="printed") for n, p in (("a", part), ("b", moved),
                                                            ("c", mirrored))]
    (g,) = group_made(bodies, "printed")
    assert (g.qty, g.mirrored) == (3, ["c"])
    laser = [Body(b.name, part=b.part, fab="laser") for b in bodies]
    (g,) = group_made(laser, "laser")                # a flipped plate is the same cut
    assert (g.qty, g.mirrored) == (3, [])
    mech = Mechanism("m", bodies=bodies)
    bom = bom_from_mechanism(mech)
    (row,) = bom.made
    assert (row.qty, row.mirrored) == (3, 1)
    assert "2 + 1 mirrored" in bom.markdown()
