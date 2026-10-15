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
def test_the_prints_table_fits_every_cell():
    from reportlab.pdfbase.pdfmetrics import stringWidth

    from spiderpig.guide import pdf
    from spiderpig.guide.doc import Doc, PrintBatch

    long = ["SL8.5x23.9-PETG", "FS14.9-3.0-TPU", "RG9.3-3.0b", "SP8-0.7", "LK142x20aM"]
    doc = Doc("t", [], None, [], [PrintBatch(lab, f"{lab}_" + "very_long_part_name" * 4
                                             + ".stl", 40, "TPU 95A flexible filament, 1.75 mm",
                                             12.345) for lab in long], [], "")
    cols = pdf.print_columns(doc)
    for cells, font in pdf.print_rows(doc):
        for i, (text, (_, _, width, _)) in enumerate(zip(cells, cols, strict=True)):
            used = pdf.BOLD if i == 0 else font
            assert stringWidth(pdf._safe(text), used, 9) <= width + 1e-6, (text, width)
    assert pdf.print_rows(doc)[1][0][0] == "SL8.5x23.9-PETG"       # labels whole


@pytest.mark.no_fabricate
def test_float_noise_never_decides_a_label():
    from spiderpig.labels import _half, _printed_label

    assert _half(96.49999999999999) == _half(96.50000000000003) == "96.5"
    assert _half(44.5) == "44.5"
    assert _half(12.0) == "12"
    assert _printed_label("SP", 7.999999999999, 8.0, 0.6500000000001) == _printed_label(
        "SP", 8.0, 8.0, 0.6499999999999)


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
    assert bought_label("m25_nylon_standoff_mf_6") == "M2.5-NY-SO-MF-6"
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
    """The steps' parts lists against what is bought and made: every bought line of the
    BOM (``bom_from_mechanism``) in some step as often as it is bought, unless it is a
    shop supply (threadlocker, epoxy, filament: the cover's list); every made part once."""
    from spiderpig.hardware.bom import bom_from_mechanism
    from spiderpig.labels import CONSUMABLE, assembly_order, part_types, step_counts

    st = assembly_steps(mech, design)
    types = part_types(mech, assembly_order(mech, design))
    listed: Counter = Counter()
    for counts in step_counts(st, types, mech):
        listed.update(counts)
    of_key = {t.key: t.label for t in types if t.kind == "purchased" and t.key}
    for row in bom_from_mechanism(mech, group=False).purchased:
        if CONSUMABLE.search(row.key):
            continue
        assert row.key in of_key, row.key
        assert listed[of_key[row.key]] == pytest.approx(row.qty), (row.key, row.qty)
    for t in types:
        if t.kind != "purchased":
            assert listed[t.label] == t.qty, t.label
    assert set(listed) == {t.label for t in types}
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
    from spiderpig.labels import bought_by_body

    stacks = {n for n, rows in bought_by_body(mech)[0].items() if len({k for k, _, _ in rows}) > 1}
    names = [n for t in types for n in t.names if n not in stacks]   # (a shim stack is
    assert len(names) == len(set(names))                           # listed under each shim)
    for t in types:
        if t.kind == "purchased":
            assert t.name not in (t.key, re.sub(r"^[LR]\.", "", t.ref)), t.label
            if t.key:
                assert t.name == catalog.get(t.key).name
        if t.kind == "printed":
            assert t.file
            assert t.file.startswith(t.label + "_")
    # a and b of one size say what tells them apart
    by_base: dict[str, list] = {}
    for t in types:
        if re.search(r"\d[a-z]M?$", t.label):
            by_base.setdefault(re.sub(r"[a-z]M?$", "", t.label), []).append(t)
    for same in by_base.values():
        assert len({t.name for t in same}) == len(same), [t.name for t in same]
    # no name says a thing twice (the horn's "in the box")
    for t in types:
        assert t.name.count("box") <= 1, t.name
        assert not t.detail or t.detail not in t.name, (t.name, t.detail)
    # a shim stack is listed as the shims it is bought as, each thickness its own line (the
    # walkers' head spacers since 2026-10-14)
    for t in (t for t in types if t.key and t.key.startswith("shim_din988")):
        assert re.search(r"x (\d+\.)?\d+ mm$", t.name), t.name
    deck = next(t for t in types if t.ref.endswith("deck_plate"))
    assert re.fullmatch(r"DK1\d\d(\.5)?x[67]\d(\.5)?", deck.label)   # its outline
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
    assert {x.stage for x in st} <= {"stack", "unit", "join", "wiring", "other"}
    # no controller named: none (review fix round 2: a stored mechanism keeps its BOM), so
    # no wiring step; named, its board is bought as lines and wired in its last step
    assert not [x for x in st if x.stage == "wiring"]
    named = BuildConfig(linkage="hoecken_pantograph", module="single", robot=False,
                        roles=(("controller", "esp32_bus_servo"),))
    mech = cache.cached_side(named, 1.0)
    st, _ = _quantities(mech, cache.cached_design(named)[1])
    (wiring,) = [x for x in st if x.stage == "wiring"]
    assert wiring.number == len(st)
    assert "esp32_servo_driver" in wiring.extras
    assert "bus servo ports" in " ".join(wiring.text)


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


# ---------------------------------------------------------------------------
# an arm's robot: the turret's hooks and the controller's wiring (2026-10-14)
# ---------------------------------------------------------------------------

TURRET_PARTS = (
    ["base_bottom", "base_top", "yaw_motor", "yaw_shaft", "yaw_clamp_washer",
     "yaw_clamp_screw", "yaw_bearing_lower", "yaw_bearing_upper", "yaw_collar_mid",
     "yaw_collar_under", "yaw_collar_over", "yaw_spring", "yaw_shim_under", "yaw_shim_over",
     "yaw_pulley_large", "yaw_pulley_small", "yaw_belt", "turntable_centre_shims",
     "turntable", "yaw_stop_post", "yaw_stop_post_screw", "yaw_stop", "yaw_stop_nut"]
    + [f"base_spacer{i}" for i in range(3)] + [f"base_col{i}_0" for i in range(7)]
    + [f"base_col_{w}{i}{x}" for i in range(7) for w, x in (("washer", ""), ("screw", "a"),
                                                             ("screw", "b"))]
    + [f"yaw_motor_screw{i}" for i in range(4)] + [f"turntable_lift{i}" for i in range(4)]
    + [f"hub_screw{i}" for i in range(4)] + [f"foot_bracket{i}" for i in range(4)]
    + [f"foot_screw{i}{x}" for i in range(4) for x in "ab"])
"""The turret's parts (``construction.rotary.base_pair.turret_parts``' names)."""


def _robot_view(names, keys=None, lines=(), meta=None):
    from spiderpig.construction.assembly import RobotView

    return RobotView(dict.fromkeys(names, (0.0, 1.0)), dict.fromkeys(names),
                     dict.fromkeys(names, "purchased"), dict(keys or dict.fromkeys(names)),
                     dict(meta or {}), tuple(lines))


@pytest.mark.no_fabricate
def test_the_turret_order_is_structured():
    from spiderpig.construction.assembly import ARM_ORDER, LAYOUT_ORDERS, TURRET_ORDER

    stages = [s.stage for s in TURRET_ORDER]
    side = [s.stage for s in ARM_ORDER if s.stage not in ("wiring", "other")]
    assert stages[:len(side)] == side                                           # the side first
    assert stages[-4:] == ["turret", "mount", "wiring", "other"]
    assert stages.count("wiring") == 1
    assert LAYOUT_ORDERS == {"mirrored_pair": ROBOT_ORDER, "turret": TURRET_ORDER}
    wiring = next(s for s in ROBOT_ORDER if s.stage == "wiring")
    assert "deck's wire slot" in wiring.intro         # the walker's layout: the routing
    assert not wiring.outro                           # the ports: its controller's sentence
    assert [s.stage for s in SIDE_ORDER][-2:] == ["wiring", "other"]     # a mechanism's


@pytest.mark.no_fabricate
def test_the_turret_hook_puts_every_part_on_once_with_its_sentence():
    from spiderpig.construction.assembly import TURRET, _claimed
    from spiderpig.construction.rotary import base_pair

    texts = [(tag, f"sentence of {tag}") for tag, *_ in base_pair.ASSEMBLY]
    ops = base_pair.assembly(_robot_view(TURRET_PARTS, meta={"turret": {"assembly": texts}}))
    claimed = [p.name for op in ops for p in op.pieces]
    assert sorted(claimed) == sorted(TURRET_PARTS)              # each once
    assert [op.tag for op in ops] == [tag for tag, *_ in base_pair.ASSEMBLY]
    assert all(op.text == f"sentence of {op.tag}" for op in ops)
    assert {op.stage for op in ops} == {TURRET, "mount", "wiring"}
    assert next(op for op in ops if op.tag == "cables").pieces == ()     # the wrap: a line
    assert "spiral_wrap_6mm" in next(op for op in ops if op.tag == "cables").extras
    assert _claimed(ops) == set(TURRET_PARTS)


@pytest.mark.no_fabricate
def test_the_wiring_step_is_the_design_controllers():
    """One generic hook over each controller's declared wiring: the walker's deck board
    says the harness (its sentence as the deck said it), the arm's stepper board its
    power, host, ports (named for the design's joints) and the outputs present; none
    (your own) has no step; a board missing (a deck that didn't fit) neither."""
    from spiderpig.actuators.controllers import get
    from spiderpig.construction.assembly import WIRING, controller_ops
    from spiderpig.hardware.electronics import HV_WARNING

    walker = controller_ops(_robot_view(["deck_board", "L.servo"],
                                        {"deck_board": "esp32_servo_driver",
                                         "L.servo": "servo_sts3215"}), get("esp32_bus_servo"))
    assert [(op.stage, op.title, op.tag) for op in walker] == [
        (WIRING, "Bus cables and harness", "wiring")]
    assert walker[0].text.startswith("The harness: the XT30 pigtail")
    assert HV_WARNING in \
        walker[0].text
    assert set(walker[0].extras) == {"resistor_100k", "resistor_33k", "xt30_pigtail_pair",
                                     "dc_plug_5521_pigtail"}
    assert controller_ops(_robot_view(["L.servo"], {"L.servo": "servo_sts3215"}),
                          get("esp32_bus_servo")) == []          # no board: no harness
    assert controller_ops(_robot_view(["deck_board"]), None) == []
    skr = get("skr_pico_pi_zero")
    arm = controller_ops(_robot_view(
        ["drive_q2_motor", "yaw_motor"], {"drive_q2_motor": "stepper_17hs4401",
                                          "yaw_motor": "stepper_17hs4401"},
        lines=[k for k, _ in skr.bom] + ["vacuum_pump_12v", "solenoid_valve_12v_3way"]),
        skr, joints=("q2", "q3", "q1"))
    text = arm[0].text
    assert arm[0].title == "Controller and cables"
    assert "q1 on X, q2 on Y, q3 on Z" in text
    assert "q4" not in text
    assert "vacuum pump on HE (gpio23)" in text
    assert "camera" not in text                       # none bought: its wire unsaid
    assert set(arm[0].extras) == {k for k, _ in skr.bom}


@pytest.mark.no_fabricate
def test_the_wiring_diagram_follows_the_controller():
    from build123d import Box

    from spiderpig.actuators.controllers import get
    from spiderpig.guide.wiring import columns, wiring_of
    from spiderpig.hardware.bom import BomLine
    from spiderpig.mechanism import Body, Mechanism

    def bought(name, key):
        return Body(name, part=Box(1, 1, 1), fab="purchased", bom_key=key)

    skr = get("skr_pico_pi_zero")
    arm = Mechanism("arm", [bought("drive_q2_motor", "stepper_17hs4401"),
                            bought("yaw_motor", "stepper_17hs4401"),
                            bought("tool_camera", "camera_ov5647_m12")], [],
                    bom_extras=[BomLine(k, q, "") for k, q in skr.bom]
                    + [BomLine("csi_cable_pi_zero_30", 1, ""), BomLine("vacuum_pump_12v", 1, "")])
    nodes, links, left = wiring_of(arm, {"yaw_motor": "M1"}, skr)
    roles = [n.role for n in nodes]
    assert sorted(roles) == sorted(["motor", "motor", "camera", "board", "host", "supply",
                                    "jack", "fan", "pump"])
    edges = {(nodes[i].role, nodes[j].role): cs for i, j, cs in links}
    assert edges[("board", "motor")] == ["Stepper motor cable 1 m x 3"]
    assert edges[("host", "camera")] == ["Raspberry Pi Zero camera cable"]
    assert edges[("board", "pump")] == ["Schottky diode 1N5819"]
    assert ("supply", "jack") in edges
    assert left == []
    assert next(n for n in nodes if n.label == "M1").title.startswith("yaw motor: ")
    col = columns(nodes, links)
    assert col[roles.index("supply")] == 0
    assert col[roles.index("camera")] == 4
    walker = Mechanism("w", [bought("L.servo", "servo_sts3215"), bought("R.servo", "servo_sts3215"),
                             bought("deck_board", "esp32_servo_driver"),
                             bought("deck_battery", "lipo_2s_450")], [],
                       bom_extras=[BomLine("lipo_strap_10mm", 1, ""),
                                   BomLine("xt30_pigtail_pair", 1, "")])
    nodes, links, left = wiring_of(walker, {}, get("esp32_bus_servo"))
    assert sorted(n.role for n in nodes) == ["battery", "board", "servo", "servo"]  # no strap
    assert {n.title.split(":")[0] for n in nodes if n.role == "servo"} == {"left servo",
                                                                            "right servo"}
    assert left == ["XT30 pigtail"]           # its link (battery -> protection) isn't drawn


@pytest.fixture(scope="module")
def arm_robot():
    from tests import cache

    cfg = BuildConfig(linkage="palletizer_reach")
    return cfg, cache.cached_robot(cfg, 1.0), cache.cached_design(cfg)[1]


@pytest.mark.slow
def test_the_arm_robot_is_built_side_turret_wiring(arm_robot):
    """The approved arm's guide: its side, then the turret's base step by step on the
    bench, the side stood on its turntable, the controller's wiring step (its sentences,
    its ports, its parts list), every part once."""
    cfg, mech, design = arm_robot
    st = assembly_steps(mech, design, config=cfg)
    _check(mech, st)
    first = {k: min(x.number for x in st if x.stage == k) for k in {x.stage for x in st}}
    assert first["stack"] < first["join"] < first["turret"] < first["mount"] < first["wiring"]
    titles = [x.title for x in st if x.stage == "turret"]
    assert titles[0] == "Base: plates and columns"
    assert "Hard stop" in titles
    wiring = next(x for x in st if x.stage == "wiring")
    text = " ".join(wiring.text)
    assert not wiring.adds
    assert "q1 on X, q2 on Y, q3 on Z" in text
    assert "spiral wrap" in text
    assert {"skr_pico_v1", "stepper_cable_1m", "spiral_wrap_6mm"} <= set(wiring.extras)
    from spiderpig.labels import assembly_order, part_types, step_counts

    types = part_types(mech, assembly_order(mech, design))
    counts = step_counts(st, types, mech)[wiring.number - 1]
    label = {t.key: t.label for t in types if t.key}
    assert counts[label["stepper_cable_1m"]] == 3
