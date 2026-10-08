"""The assembly guide (``spiderpig guide``, docs/agentlib/GUIDE.md): the structured
assembly order and its hooks, the part labels, the renderer, the PDF."""

from __future__ import annotations

import hashlib
import io
import math
import re

import numpy as np
import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction.assembly import (
    ROBOT_ORDER,
    SIDE_ORDER,
    SideView,
    assembly_steps,
    body_of,
    prose,
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
def test_label_bubbles_never_overlap():
    from PIL import Image

    img = Image.new("RGB", (400, 300), (255, 255, 255))
    marks = [(f"P{i:02d}", Mark(200 + (i % 3) * 4, 150 + (i // 3) * 4, 100 - i))
             for i in range(8)]                    # eight parts in one small spot
    _, placed = bubbles(img, marks)
    assert len(placed) >= 6
    pts = list(placed.values())
    for i, (x, y) in enumerate(pts):
        assert 20 <= x <= 380
        assert 20 <= y <= 280
        for u, v in pts[i + 1:]:
            assert math.hypot(x - u, y - v) >= 42          # two radii apart, and some


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
def test_the_robot_order_is_structured_and_reads_as_prose():
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
    text = prose()
    assert len(text) == len([s for s in ROBOT_ORDER if s.stage != "other"])
    assert all(re.match(r"\d+\. ", t) for t in text)
    assert len(prose(SIDE_ORDER)) == 3


@pytest.mark.no_fabricate
def test_labels_number_by_first_use_and_name_the_print_files():
    from build123d import Box, Location

    from spiderpig.labels import part_types, print_stems
    from spiderpig.mechanism import Body, Mechanism

    def body(name, part, fab, key=None):
        return Body(name, part=part, fab=fab, bom_key=key)

    ring = Box(8, 8, 0.7)
    mech = Mechanism("m", [
        body("L.pin_J3_leg0_spacer_hi", ring, "printed"),
        body("R.pin_J3_leg0_spacer_hi", ring.moved(Location((20, 0, 0))), "printed"),
        body("L.pillar_J2_leg0_ring4", Box(9, 9, 3), "printed"),
        body("L.b1_leg0", Box(40, 12, 3), "laser"),
        body("L.servo_screw0", Box(2, 2, 6), "purchased", "m3_bhcs_8"),
    ], [], meta={"filament": "pla_filament"})
    order = ["L.servo_screw0", "L.pillar_J2_leg0_ring4", "L.b1_leg0",
             "L.pin_J3_leg0_spacer_hi", "R.pin_J3_leg0_spacer_hi"]
    a, b = part_types(mech, order), part_types(mech, order)
    assert [(t.label, t.qty, t.file) for t in a] == [(t.label, t.qty, t.file) for t in b]
    got = {t.label: t for t in a}
    assert set(got) == {"H01", "P01", "C01", "P02"}
    assert got["P01"].file == "P01_ring_3.0mm.stl"
    assert got["P02"].file == "P02_top_spacer_0.7mm.stl"
    assert got["P02"].qty == 2
    stems = print_stems(a)
    assert stems["R.pin_J3_leg0_spacer_hi"] == "P02_top_spacer_0.7mm"


# ---------------------------------------------------------------------------
# slow: real designs (the test cache's fabrications)
# ---------------------------------------------------------------------------


def _check(mech, st):
    """Every body exactly once (whole, or in pieces that cover it), numbered 1..N,
    each step with text, nothing added and shown at once."""
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
    assert all(s.text for s in st)
    for s in st:
        if s.sub:                                    # drawn alone, put on in the next step
            assert not s.context
            assert set(s.adds) <= set(st[s.number].places)


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


@pytest.mark.slow
def test_the_default_robots_labels_are_unique_stable_and_name_its_prints(default_robot):
    from spiderpig.labels import assembly_order, part_types

    mech, design = default_robot
    order = assembly_order(mech, design)
    a = part_types(mech, order)
    labels = [t.label for t in a]
    assert len(labels) == len(set(labels))
    assert all(re.fullmatch(r"[PCH]\d\d M?".replace(" ", ""), x) for x in labels)
    names = [n for t in a for n in t.names]
    assert len(names) == len(set(names))
    assert set(names) == {b.name for b in mech.bodies if b.part is not None
                          and b.fab in ("printed", "laser", "purchased")}
    assert [(t.label, t.file) for t in part_types(mech, order)] == [(t.label, t.file) for t in a]
    files = [t.file for t in a if t.kind == "printed"]
    assert all(f and f.startswith(t) for f, t in zip(files, (t.label for t in a
                                                              if t.kind == "printed"),
                                                      strict=True))
    # numbered in the order the steps first need them
    first = {t.label: min(order.index(n) for n in t.names) for t in a}
    for kind in "PCH":
        mine = sorted(x for x in first if x.startswith(kind) and not x.endswith("M"))
        assert [first[x] for x in mine] == sorted(first[x] for x in mine)


@pytest.mark.slow
def test_klann_lego_quad_robot_steps():
    from tests import cache

    cfg = BuildConfig(linkage="klann_lego", module="quad")
    mech = cache.cached_robot(cfg, 1.0)
    st = assembly_steps(mech, cache.cached_design(cfg)[1])
    _check(mech, st)
    assert {x.stage for x in st} >= {"stack", "unit", "join", "chassis", "wiring", "deck"}


@pytest.mark.slow
def test_a_mechanism_on_its_own_is_one_side():
    from tests import cache

    cfg = BuildConfig(linkage="hoecken_pantograph", module="single", robot=False)
    mech = cache.cached_side(cfg, 1.0)
    st = assembly_steps(mech, cache.cached_design(cfg)[1])
    _check(mech, st)
    assert [x.stage for x in st if x.stage != "stack"]
    assert {x.side for x in st} == {None}
    assert {x.stage for x in st} <= {"stack", "unit", "join", "other"}


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
