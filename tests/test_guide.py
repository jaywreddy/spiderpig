"""The assembly guide (``spiderpig guide``, docs/agentlib/GUIDE.md): the structured
assembly order and its hooks, the part labels, the renderer, the PDF, the docs' copy."""

from __future__ import annotations

import hashlib
import io
import math
import re
from collections import Counter

import numpy as np
import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction.assembly import (
    ROBOT_ORDER,
    SIDE_ORDER,
    SideView,
    assembly_steps,
    body_of,
)
from spiderpig.guide.render import HIGHLIGHT, Item, Mark, View, bubbles, render


def _box(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    p = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = np.array([(a, b, c) for a, b, c, d in q] + [(a, c, d) for a, b, c, d in q])
    return p, t.ravel()


def _png(img) -> bytes:
    buf = io.BytesIO()
    img.save(buf, "PNG")
    return buf.getvalue()


# ---------------------------------------------------------------------------
# quick: no fabrication
# ---------------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_render_draws_the_highlight_marks_it_and_is_byte_stable():
    items = [Item("base", *_box((0, 0, 0), (40, 30, 2))),
             Item("new", *_box((5, 5, 2), (15, 25, 8)), fill=HIGHLIGHT, mark=True)]
    view = View(eye=(1.0, 0.9, 1.4), size=(160, 120))   # from the +z side, over the box
    (a, marks), (b, _) = render(items, view), render(items, view)
    img = np.asarray(a)
    assert img.shape == (120, 160, 3)
    orange = (img[..., 0] > 150) & (img[..., 2] < 90)
    assert orange.sum() > 100                       # the added part shows, highlighted
    assert (img.min(-1) < 60).sum() > 100           # with ink lines
    assert hashlib.sha256(_png(a)).digest() == hashlib.sha256(_png(b)).digest()
    m = marks["new"]                                # where it shows: on the orange part
    assert m.n > 50
    assert orange[int(m.y), int(m.x)]


@pytest.mark.no_fabricate
def test_label_tags_never_overlap_and_fit_their_labels():
    from PIL import Image

    img = Image.new("RGB", (500, 360), (255, 255, 255))
    labels = ["SP8-0.7", "RG9.3-3.0", "SL8.5x23.9-PETG", "M3-BH-8", "CHI-M3-16", "LK75x27",
              "HORN-STS3215", "CW37x29"]
    marks = [(lab, Mark(250 + (i % 3) * 4, 180 + (i // 3) * 4, 100 - i))
             for i, lab in enumerate(labels)]      # eight parts in one small spot
    out, placed = bubbles(img, marks)
    assert len(placed) >= 6
    boxes = list(placed.values())
    for i, (x0, y0, x1, y1) in enumerate(boxes):
        assert 0 <= x0 < x1 <= 500
        assert 0 <= y0 < y1 <= 360
        for u0, v0, u1, v1 in boxes[i + 1:]:
            assert x1 <= u0 or u1 <= x0 or y1 <= v0 or v1 <= y0     # disjoint
    wide = placed.get("SL8.5x23.9-PETG")
    if wide is not None:
        assert wide[2] - wide[0] > placed.get("SP8-0.7", (0, 0, 60, 0))[2] - placed.get(
            "SP8-0.7", (0, 0, 0, 0))[0]
    assert hashlib.sha256(_png(out)).digest() == hashlib.sha256(
        _png(bubbles(img, marks)[0])).digest()


@pytest.mark.no_fabricate
def test_a_gap_goes_with_the_layer_above_it():
    layers = {-1: (-3.0, 0.0), 0: (0.0, 2.0), 1: (4.7, 7.7), 2: (11.0, 14.0), 3: (14.0, 16)}
    v = SideView("L", {}, {}, {}, {}, layers)
    assert v.top == 2
    assert v.slot(-2.5) == -1
    assert v.slot(2.0) == 1        # a spacer on the plate's top: the gap under layer 1
    assert v.slot(7.7) == 2
    assert v.slot(20.0) == 4       # over the stack


@pytest.mark.no_fabricate
def test_the_robot_order_is_structured():
    stages = [(s.stage, s.side) for s in ROBOT_ORDER]
    assert stages.index(("stack", "L")) < stages.index(("unit", "L")) < stages.index(
        ("join", "L")) < stages.index(("chassis", None)) < stages.index(("unit", "R"))
    assert stages.index(("unit", "R")) < stages.index(("stack", "R")) < stages.index(
        ("join", "R")) < stages.index(("wiring", None)) < stages.index(("deck", None))
    right = next(s for s in ROBOT_ORDER if s.stage == "unit" and s.side == "R")
    assert right.where == "robot"          # its chains go onto the studs first
    assert right.tags[0] == ("ties",)
    chassis = next(s for s in ROBOT_ORDER if s.stage == "chassis")
    assert any("R.unit.servo" in t for t in chassis.tags)   # the right servo moves here
    assert [s.stage for s in SIDE_ORDER][:3] == ["stack", "unit", "join"]


def _toy():
    from build123d import Box, Location

    from spiderpig.mechanism import Body, Mechanism

    def body(name, part, fab, key=None):
        return Body(name, part=part, fab=fab, bom_key=key)

    top = Box(8, 8, 0.7)
    return Mechanism("m", [
        body("L.pin_J3_leg0_spacer_hi", top, "printed"),
        body("R.pin_J3_leg0_spacer_hi", top.moved(Location((20, 0, 0))), "printed"),
        body("L.pillar_J2_leg0_ring4", Box(9.3, 9.3, 3), "printed"),
        body("L.pillar_J6_leg0_ring4", Box(9.3, 9.3, 3.04), "printed"),  # rounds the same
        body("L.servo_screw0", Box(2, 2, 6), "purchased", "m3_bhcs_8"),
    ], [], meta={"filament": "pla_filament"})


@pytest.mark.no_fabricate
def test_labels_say_what_the_part_is_whatever_the_order():
    from spiderpig.labels import bought_label, part_types, print_stems

    mech = _toy()
    names = [b.name for b in mech.bodies]
    a, b = part_types(mech, names), part_types(mech, names[::-1])
    assert {t.label: sorted(t.names) for t in a} == {t.label: sorted(t.names) for t in b}
    got = {t.label: t for t in a}
    # two rings 3.0 and 3.04 mm high: one label, told apart by their geometry (volume)
    assert set(got) == {"SP8-0.7", "RG9.3-3.0a", "RG9.3-3.0b", "M3-BH-8"}
    assert got["RG9.3-3.0a"].names == ["L.pillar_J2_leg0_ring4"]
    assert got["SP8-0.7"].qty == 2
    assert got["SP8-0.7"].file == "SP8-0.7_top_spacer.stl"
    assert print_stems(a)["R.pin_J3_leg0_spacer_hi"] == "SP8-0.7_top_spacer"
    assert bought_label("m25_nylon_standoff_mf_6") == "M25-NY-SO-MF-6"
    assert bought_label("m3_heat_set_insert") == "M3-INS"


# ---------------------------------------------------------------------------
# slow: real designs (the test cache's fabrications)
# ---------------------------------------------------------------------------


def _check(mech, st):
    """Every body exactly once (whole, or in pieces that cover it), numbered 1..N, each
    step with text and no sentence twice, nothing added and shown at once."""
    whole, pieces = [], {}
    for s in st:
        assert not set(s.context) & set(s.adds)
        for p in s.adds:
            if "#" in p:
                pieces.setdefault(body_of(p), []).append(p.split("#", 1)[1])
            else:
                whole.append(p)
    assert len(whole) == len(set(whole))
    assert not set(whole) & set(pieces)
    assert all(len(v) == len(set(v)) >= 2 for v in pieces.values())
    assert set(whole) | set(pieces) == {b.name for b in mech.bodies if b.part is not None}
    assert [s.number for s in st] == list(range(1, len(st) + 1))
    for s in st:
        assert s.text
        assert len(s.text) == len(set(s.text)), (s.number, s.text)
        for t in s.text:                # nor one that another of the step repeats
            assert not [u for u in s.text if u != t and u.startswith(t.rstrip("."))], s.text
        if s.sub:                                    # drawn alone, put on in the next step
            assert not s.context
            assert set(s.adds) <= set(st[s.number].places)


def _quantities(mech, design):
    """Each label's quantity in the steps' parts lists against its type's (the BOM's and
    the parts table's): every part is listed, once."""
    from spiderpig.labels import assembly_order, by_body, part_types

    st = assembly_steps(mech, design)
    types = part_types(mech, assembly_order(mech, design))
    of = by_body(types)
    listed = Counter(of[n].label for s in st for n in s.counted if n in of)
    assert listed == Counter({t.label: t.qty for t in types})
    counted = [n for s in st for n in s.counted]
    assert len(counted) == len(set(counted))
    return st, types


@pytest.fixture(scope="module")
def default_robot():
    from tests import cache

    cfg = BuildConfig()
    return cache.cached_robot(cfg, 1.0), cache.cached_design(cfg)[1]


@pytest.mark.slow
def test_the_default_robot_is_built_in_the_robots_order(default_robot):
    mech, design = default_robot
    st = assembly_steps(mech, design)
    _check(mech, st)
    stacks = {s: [x.layer for x in st if x.stage == "stack" and x.side == s and not x.sub]
              for s in "LR"}
    assert stacks["L"] == stacks["R"] == sorted(stacks["L"])     # mirror images, bottom up
    first = {k: min(x.number for x in st if (x.stage, x.side) == k)
             for k in {(x.stage, x.side) for x in st}}
    assert first[("stack", "L")] < first[("unit", "L")] < first[("join", "L")] < first[
        ("chassis", None)] < first[("unit", "R")] < first[("stack", "R")] < first[
        ("join", "R")] < first[("wiring", None)] < first[("deck", None)]
    # the right servo goes on with its centre plates; the right unit isn't on the bench
    servo = next(x for x in st if "R.servo" in x.adds)
    assert servo.stage == "chassis"
    assert all(x.where == "robot" for x in st if (x.stage, x.side) == ("unit", "R"))
    # a Chicago pin: its barrel with its host link, its screw a later layer
    pin = next(b.name for b in mech.bodies if re.fullmatch(r"L\.pin_\w+_screw", b.name))
    at = {p: x.number for x in st for p in x.adds}
    assert at[f"{pin}#barrel"] < at[f"{pin}#screw"]
    host = next(b.rigid_with for b in mech.bodies if b.name == pin)
    assert at[f"{pin}#barrel"] == at[host]
    # the hub plate goes on with the horn, not with its layer
    hub = max((b.name for b in mech.bodies if re.fullmatch(r"L\.crank_plate\d+", b.name)),
              key=lambda n: int(n.rsplit("plate", 1)[1]))
    assert next(x for x in st if hub in x.adds).stage == "unit"
    # what the old prose said and the hooks must keep saying
    text = " ".join(t for x in st for t in x.text)
    for said in ("journal hole", "light press", "capped", "far holes", "rear idler horn",
                 "feeler gauge", "ball-end key", "turn freely", "notches"):
        assert said in text, said


@pytest.mark.slow
def test_the_default_robots_labels(default_robot):
    """Unique, independent of the step order, every part listed in the steps as often as
    it is bought or made, every bought label named from the catalog, each laser part
    measured as its cut file is."""
    from spiderpig.hardware import catalog
    from spiderpig.labels import assembly_order, part_types

    mech, design = default_robot
    _, types = _quantities(mech, design)
    labels = [t.label for t in types]
    assert len(labels) == len(set(labels))
    order = assembly_order(mech, design)
    assert order is not None
    again = part_types(mech, order[::-1])
    assert {t.label: sorted(t.names) for t in again} == {t.label: sorted(t.names)
                                                          for t in types}
    names = [n for t in types for n in t.names]
    assert len(names) == len(set(names))
    for t in types:
        if t.kind == "purchased":
            assert t.name not in (t.key, re.sub(r"^[LR]\.", "", t.ref)), t.label
            if t.key:
                assert t.name == catalog.get(t.key).name
        if t.kind == "printed":
            assert t.file
            assert t.file.startswith(t.label + "_")
    deck = next(t for t in types if t.ref.endswith("deck_plate"))
    assert re.fullmatch(r"DK1\d\dx[67]\d", deck.label)      # its outline, not its thickness
    w, h = map(float, re.findall(r"([\d.]+) x ([\d.]+) mm", deck.name)[0])
    assert w > h > 20


@pytest.mark.slow
def test_klann_lego_quad_robot_steps():
    from tests import cache

    cfg = BuildConfig(linkage="klann_lego", module="quad")
    mech = cache.cached_robot(cfg, 1.0)
    design = cache.cached_design(cfg)[1]
    st, _ = _quantities(mech, design)
    _check(mech, st)
    assert {x.stage for x in st} >= {"stack", "unit", "join", "chassis", "wiring", "deck"}


@pytest.mark.slow
def test_a_mechanism_on_its_own_is_one_side():
    from tests import cache

    cfg = BuildConfig(linkage="hoecken_pantograph", module="single", robot=False)
    mech = cache.cached_side(cfg, 1.0)
    design = cache.cached_design(cfg)[1]
    st, _ = _quantities(mech, design)
    _check(mech, st)
    assert {x.side for x in st} == {None}
    assert {x.stage for x in st} <= {"stack", "unit", "join", "other"}


@pytest.mark.slow
def test_the_docs_quote_the_current_order(default_robot):
    """docs/ARCHITECTURE.md's assembly order is what the hooks and the robot's order say
    now (``python -m spiderpig.guide.prose --write`` after an edit)."""
    from spiderpig.guide.prose import DOC, current, section

    mech, design = default_robot
    assert current(DOC.read_text()) == section(mech, design)


@pytest.mark.slow
def test_the_pdf_has_its_pages(default_robot, tmp_path):
    """A short guide (two steps, two workers) through the whole pipeline, injected with
    the test cache's robot: the PDF's page count is the cover, the parts, the prints, the
    bag labels and the steps."""
    from spiderpig.guide.make import build_guide
    from spiderpig.labels import part_types

    mech, design = default_robot
    rep = build_guide(BuildConfig(), None, tmp_path, jobs=2, max_steps=2, design=design,
                      mech=mech)
    data = rep.pdf.read_bytes()
    assert data.startswith(b"%PDF")
    assert len(re.findall(rb"/Type /Page\b", data)) == rep.pages
    types = part_types(mech)
    n_parts = len(types)
    n_prints = sum(t.kind == "printed" for t in types)
    n_bags = sum(t.kind != "laser" for t in types)
    assert rep.pages == (1 + math.ceil(n_parts / 24) + math.ceil(n_prints / 26)
                         + math.ceil(n_bags / 24) + 2)
    md = (tmp_path / "ASSEMBLY.md").read_text()
    assert "**1. " in md
    assert "**2. " in md


@pytest.mark.slow
def test_the_assembly_hooks_stay_out_of_the_fabrication_key():
    """A sentence of a hook, the robot's order or the labels never invalidates a
    fabrication (``spiderpig.keys``): nothing the fabrication reaches calls them."""
    from spiderpig import keys

    c = keys.closure(keys.FAB_ROOTS)
    assert "spiderpig.construction.assembly" not in c.modules
    assert "spiderpig.labels" not in c.modules
    assert not [s for s in c.symbols if s[1] == "assembly" or re.search(r"\.assembly#", s[1])]
