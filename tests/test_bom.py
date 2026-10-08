"""Tests for :mod:`hardware.bom` (independent of the real catalog data)."""

from __future__ import annotations

import csv
import re

import pytest
from build123d import Axis, Box, Cylinder, Plane, Pos

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


@pytest.mark.parametrize("chiral", [True, False])
def test_a_robots_mirrored_twins_group_as_compared(chiral):
    """A right-side part mirroring its left twin joins the twin's group without a comparison,
    counted as :func:`congruent` would count it: mirrored iff the reference is chiral and
    the twin isn't itself the reference's mirror image."""
    from spiderpig.hardware import bom as bom_mod

    part = _chiral() if chiral else Box(10, 4, 2) - Cylinder(1, 2)
    moved = part.rotate(Axis.Z, 70).moved(Pos(40, -3, 9))
    flipped = part.mirror(Plane.XY).moved(Pos(-30, 0, 0))
    left = {"L.a": part, "L.b": moved, "L.c": flipped}
    right = {"R." + n[2:]: p.mirror(Plane.XY) for n, p in left.items()}
    named = [*left.items(), *right.items()]
    calls = []
    real = bom_mod._proper_fit

    def counted(*args):
        calls.append(1)
        return real(*args)

    for method in ("printed", "laser"):
        twins = [Body(n, part=p, fab=method) for n, p in named]
        plain = [Body(n.replace("R.", "Q."), part=p, fab=method) for n, p in named]
        calls.clear()
        bom_mod._proper_fit = counted
        try:
            (g,) = group_made(twins, method)
            with_twins = len(calls)
            (h,) = group_made(plain, method)
        finally:
            bom_mod._proper_fit = real
        assert g.names == [n.replace("Q.", "R.") for n in h.names]
        assert g.mirrored == [n.replace("Q.", "R.") for n in h.mirrored]
        assert with_twins < len(calls) - 2          # the right side compared nothing
        if method == "printed":
            assert g.mirrored == (["L.c", "R.a", "R.b"] if chiral else [])
        else:
            assert g.mirrored == []                     # a flipped plate is the same cut


# ---------------------------------------------------------------------------
# Test drive, round 3 (docs/history/TESTDRIVE.md): the pivot hardware is priced, and each
# price says where it came from
# ---------------------------------------------------------------------------


def test_the_catalog_prices_the_pivot_hardware_and_says_where_from(monkeypatch):
    import importlib

    from spiderpig.hardware import fastener_catalog, parts

    monkeypatch.setattr(catalog, "CATALOG", {})       # the real data, freshly registered
    importlib.reload(parts)
    importlib.reload(fastener_catalog)
    # (the rod, bolt, bearing and bushing pivots' hardware went with them on 2026-10-07)
    priced = {"m3_shcs_6": 3.83, "m3_shcs_12": 5.18, "m3_bhcs_6": 3.97,
              "m3_bhcs_10": 4.30, "m3_nut": 2.39, "m3_washer": 1.45, "ca_glue": 13.99,
              "wood_glue": 5.49, "plywood_3mm": 3.10}
    # (the Chicago screws' per-length prices are the sourcing's, hardware.sources, which this
    # reload of the item modules leaves out)
    for key, price in priced.items():                                         # entry 9
        offer = catalog.get(key).offer
        assert offer.price_usd == pytest.approx(price), key
        # a fetched page is verified; a price a search quoted says so, with the date
        assert offer.verified or re.search(r"2026-\d\d-\d\d", offer.note), key
        assert offer.url.startswith("https://"), key
    for key in ("m2_self_tap_6", "m3_shcs_18"):
        assert catalog.get(key).offer.price_usd is None, key           # no page priced them
    assert catalog.get("plywood_3mm").offer.pack_qty == 1              # sold per sheet



def _tolerances(shape) -> list[float]:
    from OCP.BRep import BRep_Tool

    return ([BRep_Tool.Tolerance_s(v.wrapped) for v in shape.vertices()]
            + [BRep_Tool.Tolerance_s(e.wrapped) for e in shape.edges()]
            + [BRep_Tool.Tolerance_s(f.wrapped) for f in shape.faces()])


def test_the_rod_pins_get_a_cut_list():
    """Identical lengths grouped longest first, the total, the stock they come from: what
    ``cut X mm`` BOM notes ask for (the removed rod pins' were the only ones; a stand-in rod
    item is registered here), so the buyer knows how many rods to cut them from."""
    register(Item("rod_3mm_100", "3 mm rod, 100 mm", "dowel",
                  (Offer("Amazon", "https://example.com/rod", pack_qty=5, price_usd=5.0),),
                  dims={"d": 3.0, "length": 100.0}),
             Item("ca_glue", "CA glue", "adhesive", (Offer("x", "https://example.com/glue"),)))
    mech = Mechanism("m", bodies=[], connections=[], meta={}, bom_extras=[
        BomLine("rod_3mm_100", 0.18, "pin:J7_leg0: cut 18.0 mm"),
        BomLine("rod_3mm_100", 0.21, "pin:J3_leg0: cut 21.0 mm"),
        BomLine("rod_3mm_100", 0.18, "pin:J7_leg1: cut 18.0 mm"),
        BomLine("ca_glue", 0.02, "pillar:A anchors")])
    bom = bom_from_mechanism(mech, group=False)
    (cut,) = bom.cuts
    assert (cut.key, cut.stock_mm) == ("rod_3mm_100", 100.0)
    assert cut.pieces == ((21.0, 1), (18.0, 2))
    assert (cut.count, cut.total_mm) == (3, pytest.approx(57.0))
    assert cut.describe() == ("3 pieces of 3 mm rod, 100 mm (57 mm in all, from 1 x 100 mm "
                              "stock): 1 x 21.0, 2 x 18.0 mm")
    assert "## Cut to length" in bom.markdown()
    assert bom.as_dict()["cuts"] == [{"key": "rod_3mm_100", "name": "3 mm rod, 100 mm",
                                      "stock_mm": 100.0, "pieces": [[21.0, 1], [18.0, 2]],
                                      "count": 3, "total_mm": 57.0}]
    assert bom.purchased[0].packs == 1                   # one rod (57 mm): one 5-pack
    assert bom.purchased[0].qty == 1                     # whole pieces, packed


def test_a_moved_copy_needs_no_boolean_and_anything_else_gets_one(monkeypatch):
    """:func:`congruent` takes a part's moved copy for the same part without a boolean, its
    boundary where the copy's is (``bom._coincide``); a part a hair different, or the same
    part only up to its own symmetry, goes to the boolean; the answers are those of the
    boolean alone."""
    from spiderpig.hardware import bom as bom_mod

    part = _chiral()
    copy = part.rotate(Axis.Z, 70).rotate(Axis.X, 23).moved(Pos(40, -3, 9))
    nudged = Box(10, 4, 2) - Cylinder(1, 2).moved(Pos(2, 0, 0))
    hair = Box(10, 4, 2) - Cylinder(1, 2).moved(Pos(2.0005, 0, 0))
    plate = Box(10, 4, 2) - Cylinder(1, 2)          # turned half a turn: the same plate
    calls = []
    real = bom_mod._shared_volume
    monkeypatch.setattr(bom_mod, "_shared_volume",
                        lambda a, b: calls.append(1) or real(a, b))
    cases = [(part, copy), (part, part.mirror(Plane.XY).moved(Pos(5, 5, 5))),
             (nudged, hair), (plate, plate.rotate(Axis.Z, 180))]
    got = []
    for a, b in cases:
        calls.clear()
        got.append((congruent(a, b), len(calls)))
    assert got[0] == ("same", 0)
    assert got[1][0] == "mirror"
    assert got[2][0] == "same"                        # within the boolean's tolerance
    assert got[2][1] >= 1
    assert got[3][0] == "same"
    assert got[3][1] >= 1                             # its seams elsewhere: a boolean
    monkeypatch.setattr(bom_mod, "_coincide", lambda a, b: False)
    assert [congruent(a, b) for a, b in cases] == [r for r, _ in got]


def test_grouping_leaves_the_parts_as_they_were():
    """The proof of a fit is a boolean that leaves its arguments alone (non-destructive):
    a part's tolerances are what its construction left however often it was compared, so
    what it is later meshed as (the print STLs) doesn't depend on the grouping. Two cuts
    of a slotted pin by its moved copy widened tolerances on both."""
    pin = (Cylinder(4, 12) - Box(1, 10, 5).moved(Pos(0, 0, 4))) + Cylinder(2, 6).moved(
        Pos(0, 0, 8))
    bodies = [Body(n, part=p, fab="printed") for n, p in (
        ("a", pin), ("b", pin.rotate(Axis.Z, 37).rotate(Axis.X, 11).moved(Pos(13, -7, 3))),
        ("c", _chiral()))]
    before = [_tolerances(b.part) for b in bodies]
    for _ in range(2):
        pins, other = group_made(bodies, "printed")
        assert (pins.names, other.names) == (["a", "b"], ["c"])
    assert [_tolerances(b.part) for b in bodies] == before


def test_a_parts_integrals_are_measured_once_and_shared():
    """``hardware.mass``: a part's volume and surface integrals are measured once and
    shared by every consumer (the BOM's grouping, ``part_props``), to the same numbers as a
    fresh measurement; a part moved in place is measured again, and an entry goes with its
    part."""
    import gc

    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    from spiderpig.hardware import mass

    def fresh(part):
        vol, surf = GProp_GProps(), GProp_GProps()
        BRepGProp.VolumeProperties_s(part.wrapped, vol)
        BRepGProp.SurfaceProperties_s(part.wrapped, surf)
        return vol.Mass(), surf.Mass(), vol.CentreOfMass().X(), vol.MatrixOfInertia().Value(1, 1)

    part = (_chiral() - Cylinder(0.5, 9)).solids()[0]
    first = mass.part_props(part)
    assert mass.volume_props(part) is mass.volume_props(part)          # measured once
    want = fresh(part)
    assert (first.volume, first.area, first.com[0], first.inertia[0, 0]) == want
    assert mass.volume(part) == part.volume                             # build123d's, exactly
    box = Box(2, 3, 4)                                                  # a compound's volume
    assert mass.volume(box) == box.volume
    part.move(Pos(5, 0, 0))                                             # in place: again
    moved = mass.part_props(part)
    assert moved.com[0] == fresh(part)[2] != first.com[0]
    key = id(part)
    assert key in mass._MEASURED
    del part
    gc.collect()
    # its own entry gone (the dict's size isn't the measure: the collection also drops the
    # entries of other tests' dead parts in this worker, flaky in the full suite, W4a)
    assert key not in mass._MEASURED
