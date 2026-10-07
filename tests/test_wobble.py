"""The link-tilt (wobble) metric (:mod:`construction.wobble`), the Chicago screw pins
(:mod:`construction.pivots.chicago`) and the audit's wobble report."""

from __future__ import annotations

import math

import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction import ConstructionError
from spiderpig.construction.axle import AxleGroup
from spiderpig.construction.contract import check_side
from spiderpig.construction.pivots.chicago import ChicagoAxle, ChicagoShaft
from spiderpig.construction.wobble import (
    Section,
    free_tilt_deg,
    stresses,
    supported_tilt_deg,
)
from spiderpig.hardware.bom import bom_from_mechanism
from spiderpig.hardware.catalog import get
from spiderpig.hardware.fastener_catalog import CHICAGO_LENGTHS, chicago
from spiderpig.stack import verify_plan
from spiderpig.strength import FALLBACK_PIN_LOADS
from spiderpig.tools.audit import pin_loads_for, wobble_check
from tests import cache

T = 1.0

# -- the metric -------------------------------------------------------------------------


def test_free_tilt_is_clearance_over_bearing_length():
    assert free_tilt_deg(0.2, 3.0) == pytest.approx(math.degrees(math.atan(0.2 / 3.0)))
    assert free_tilt_deg(0.2, 3.0) == pytest.approx(3.81, abs=0.01)    # a 3.2 hole, 3 mm rod
    assert free_tilt_deg(0.05, 2.25) < free_tilt_deg(0.2, 3.0)         # a bushing
    assert free_tilt_deg(0.0, 3.0) == 0.0
    assert free_tilt_deg(0.1, 0.0) == 90.0


@pytest.mark.parametrize(("t", "g", "r"), [(3.0, 0.1, 4.0), (3.0, 0.05, 6.0), (3.0, 0.5, 3.5)])
def test_supported_tilt_solves_the_slab_between_faces(t, g, r):
    a = math.radians(supported_tilt_deg(t, g, r))
    assert t * math.cos(a) + 2 * r * math.sin(a) == pytest.approx(t + g)
    assert a == pytest.approx(g / (2 * r), rel=0.15)                   # about g / 2r


def test_supported_tilt_limits():
    assert supported_tilt_deg(3.0, 0.0, 4.0) == 0.0
    assert supported_tilt_deg(3.0, 0.1, 0.0) == 90.0
    assert supported_tilt_deg(3.0, 0.1, 6.0) < supported_tilt_deg(3.0, 0.1, 4.0)


def test_stresses_of_a_two_link_pin():
    """155 N over a 3 mm span on a 3 mm rod, a two-link pin (``F s / 2``, each bore taking
    half the couple): 232.5 N mm over 2.65 mm^3, 87.7 MPa (the pin review's 43.9 MPa was
    ``F s / 4``, both ends held square, which no pin here is)."""
    note = {"span_mm": 3.0, "bearing_len_mm": 3.0, "section": Section.rod(3.0).as_dict()}
    s = stresses(note, 155.0)
    assert s["moment_nmm"] == pytest.approx(155 * 3 / 2)
    assert s["bending_mpa"] == pytest.approx(87.7, abs=0.1)
    assert s["case"] == "two-link pin"
    assert s["bearing_mpa"] == pytest.approx(155 / 9, abs=0.1)
    tube = Section.tube(4.0, 3.0)
    assert tube.z_bend > Section.rod(3.0).z_bend                       # the barrel is stiffer
    assert stresses({**note, "section": tube.as_dict()}, 155.0)["bending_mpa"] < 87.7


# -- the Chicago screw's fit ----------------------------------------------------------------


@pytest.mark.parametrize(("stack", "length"), [(6, 7), (9, 10), (12, 13), (15, 16), (18, 20)])
def test_chicago_fit_takes_up_the_barrel_length(stack, length):
    s = ChicagoShaft()
    f = s.fit(stack, 3.0)
    assert f.length == length
    assert s.min_play - 1e-9 <= f.play < s.min_play + min(s.shim_steps) + 1e-9
    assert stack + s.washer_t + f.shims + f.play == pytest.approx(f.length)
    it = get(chicago(f.length)).dims
    assert it["screw_head_h"] + s.washer_t + f.shims_hi + f.play <= 3.0 + 1e-9
    assert it["head_h"] + f.shims_lo <= 3.0 + 1e-9
    assert s.shim_count(f.shims) * 1.0 >= f.shims > 0 or f.shims == 0


def test_chicago_catalog_and_refusals():
    assert CHICAGO_LENGTHS[:6] == (4, 5, 6, 7, 8, 9)       # Harfington's black series
    for L in CHICAGO_LENGTHS:
        item = get(chicago(L))
        assert item.dims["barrel_d"] == 4.0
        assert item.dims["head_d"] == 8.5          # Harfington's drawing (sources.py)
        assert item.offers
        assert all(o.url.startswith("https://") for o in item.offers)
    for key in ("ptfe_washer_4x8x0p5", "shim_din988_4x8", "threadlocker_222"):
        assert get(key).offers
    cfg = BuildConfig(linkage="klann", module="single", robot=False, pin="chicago")
    ctx = cache.cached_design(cfg)[1].ctx
    ChicagoAxle().dims(ctx, False)
    with pytest.raises(ConstructionError, match="link pin only"):
        ChicagoAxle().dims(ctx, True)
    with pytest.raises(ConstructionError, match="no stock Chicago screw"):
        ChicagoShaft().fit(90.0, 3.0)        # past the longest (80 mm)


# -- built on the Klann single ------------------------------------------------------------------


def _chicago_config(pin: str) -> BuildConfig:
    """The Klann single, its default constructions (Chicago pins, standoff pillars, the hex
    bolt crank; the tilts below were first read on the keyed crank and printed pillars,
    removed on 2026-10-07)."""
    return BuildConfig(linkage="klann", module="single", robot=False, pin=pin)


@pytest.fixture(scope="module", params=["chicago"])
def chicago_side(request):
    """The Klann single side with Chicago pins, from the fabrication cache
    (:mod:`tests.cache`: its plan and parts built once per engine version). Read it, never
    change it. (The bushed variant, ``chicago_bushing``, was removed on 2026-10-07.)"""
    cfg = _chicago_config(request.param)
    tmpl, design = cache.cached_design(cfg)
    return request.param, tmpl, design, cache.cached_side(cfg, T)


def _pin_groups(design) -> list[AxleGroup]:
    return [g for g in design.groups if isinstance(g, AxleGroup) and not g.pillar]


def _outside_claims(design, tmpl, t: float, groups) -> list[str]:
    """:func:`construction.contract.check_side` for ``groups`` alone (its rule for a group
    of no special kind: every body inside the group's own claims, grown by ``TOL``), at
    crank angle ``t``. A pin builds from its own claims (``AxleGroup.realize`` reads no
    other group's work), so it is realized alone."""
    from spiderpig.construction.base import Build, Realized
    from spiderpig.construction.contract import MAX_OUTSIDE, TOL, _outside
    from spiderpig.construction.envelope import claimed_solid

    build = Build(design.ctx, design.plan, tmpl.freeze_at(t))
    problems = []
    for g in groups:
        envelope = claimed_solid(build, build.shapes(g.name), TOL)
        for b in g.realize(build, Realized()).bodies:
            vol = _outside(b.part, envelope)
            if vol > MAX_OUTSIDE:
                problems.append(f"{g.name}/{b.name}: {vol:.3f} mm^3 outside its claims")
    return problems


def _clashes_with(mech, names: set[str]) -> list[tuple[str, str, float]]:
    """``tests.test_pivots._clashes`` (every pair's shared volume over 1e-3 mm^3) for the
    pairs with a part of ``names`` in them, boxes that don't overlap skipped (they share
    nothing)."""
    from spiderpig.construction.contract import _overlap
    from tests.test_pivots import _vol

    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    boxes = {n: p.bounding_box() for n, p in parts.items()}
    out = []
    for a in sorted(names & set(parts)):
        for b in parts:
            if b == a or (b in names and b < a) or not _overlap(boxes[a], boxes[b]):
                continue
            vol = _vol(parts[a], parts[b])
            if vol > 1e-3:
                out.append((a, b, round(vol, 4)))
    return out


def test_chicago_pins_plan_and_stay_in_their_claims(chicago_side):
    """The fast half of :func:`test_chicago_pins_plan_build_and_stay_in_their_claims`: the
    plan re-checked, every Chicago pin's parts inside its claims at two crank angles, and
    none of them meeting another part."""
    key, tmpl, design, fab = chicago_side
    assert verify_plan(design.plan, tmpl) == []
    pins = _pin_groups(design)
    assert pins
    assert _outside_claims(design, tmpl, T, pins) == []
    assert _outside_claims(design, tmpl, 4.38, pins) == []
    stems = tuple(g.name.replace(":", "_") for g in pins)
    mine = {b.name for b in fab.bodies if b.part is not None and b.name.startswith(stems)}
    assert len(mine) >= len(pins)
    assert _clashes_with(fab, mine) == []


@pytest.mark.slow
def test_chicago_pins_plan_build_and_stay_in_their_claims(chicago_side):
    """The whole side's contract (every group) at two crank angles and every pair of its
    parts clash-checked (:func:`test_chicago_pins_plan_and_stay_in_their_claims` checks the
    pins' part of it in the fast tier)."""
    key, tmpl, design, fab = chicago_side
    assert verify_plan(design.plan, tmpl) == []
    assert check_side(design, tmpl.freeze_at(T)) == []
    assert check_side(design, tmpl.freeze_at(4.38)) == []
    from tests.test_pivots import _clashes
    assert _clashes(fab) == []


def test_chicago_hardware_and_bom(chicago_side):
    key, _, design, fab = chicago_side
    pins = [g for g in design.groups if isinstance(g, AxleGroup) and not g.pillar]
    rows = {r.key: r for r in bom_from_mechanism(fab, group=False).purchased}
    screws = sum(r.qty for k, r in rows.items() if k.startswith("chicago_m3_"))
    assert screws == len(pins)
    assert {v["item"] for v in fab.meta["chicago"].values()} <= set(rows)
    assert "ptfe_washer_4x8x0p5" not in rows     # printed head spacers (2026-10-05)
    assert "threadlocker_222" in rows
    for g in pins:
        note = fab.meta["chicago"][g.name]
        # the barrel length's play, plus an upper gap too thin to print
        play = note["play_mm"] - note["unprinted_hi_mm"]
        assert 0.05 - 1e-9 <= play < 0.15 + 1e-9
        assert note["length_mm"] >= note["stack_mm"]


def test_wobble_notes_for_every_axle(chicago_side):
    key, _, design, fab = chicago_side
    axles = [g for g in design.groups if isinstance(g, AxleGroup)]
    notes = fab.meta["wobble"]
    assert set(notes) == {g.name for g in axles}
    for g in axles:
        n = notes[g.name]
        assert {e["link"] for e in n["links"]} == set(g.axis.members)
        for e in n["links"]:
            assert e["tilt_deg"] == min(e["free_deg"], e["supported_deg"])
    host = {g.name: min(g.axis.members, key=lambda m: (design.plan.layers[m], m))
            for g in axles if not g.pillar}
    for name, m in host.items():
        e = next(e for e in notes[name]["links"] if e["link"] == m)
        assert e["tilt_deg"] == 0.0                    # bonded to the barrel
    rep = wobble_check(notes, (119.0, 155.0))
    assert rep["pin"]["joints"] == len(host)
    # the worst pin link tilts on its column's play (the barrel's, at most min_play plus a
    # 0.1 mm shim step, and the printed head spacers' +-0.1 mm, PRINT_TOL) between the
    # washers' 4 mm faces in its clearance gaps: 1.62 deg on 0.225 mm (1.52 on the keyed
    # crank's layout, under 1.0 with the DIN 988 shims the printed spacers replaced)
    from spiderpig.construction.pivots.common import PRINT_TOL
    from spiderpig.materials import washer_od

    face = washer_od(4.0) / 2
    pitch = design.ctx.pitch
    assert rep["pin"]["worst_deg"] == pytest.approx(
        supported_tilt_deg(pitch, rep["pin"]["play_mm"], face), abs=0.01)
    assert rep["pin"]["worst_deg"] <= supported_tilt_deg(pitch, 0.15 + PRINT_TOL, face)
    assert 1 < rep["pin"]["jam"]["safety"] < rep["pin"]["walk"]["safety"]


def test_every_construction_reports_wobble():
    """The standoff pillars and the Chicago pins both leave a wobble note."""
    fab = cache.cached_side(_chicago_config("chicago"), T)
    rep = wobble_check(fab.meta["wobble"])
    # a link's 4.2 mm hole on the 4 mm barrel, 3 mm thick: atan(0.2 / 3)
    assert rep["pin"]["worst_free_deg"] == pytest.approx(3.81, abs=0.01)
    # the standoff's 0.35 mm running fit
    assert rep["pillar"]["worst_free_deg"] == pytest.approx(free_tilt_deg(0.35, 3.0), abs=0.01)
    assert rep["pillar"]["worst_free_deg"] > rep["pin"]["worst_free_deg"]
    assert rep["loads_n"] is None
    assert "walk" not in rep["pin"]


def test_the_fallback_pin_loads_come_from_the_family():
    """Without a sim (``--no-sim``, no MuJoCo) ``klann_lego`` and the other Klann variants
    fall back to the demo Klann's measured loads; a family nobody measured gets none here
    (:func:`strength.design_loads` then uses the generic, most loaded family's)."""
    for key in ("klann", "klann_lego", "klann_patent", "klann_long_legs", "klann_high_step"):
        assert pin_loads_for(key) == FALLBACK_PIN_LOADS["klann"]
    assert pin_loads_for("strider") == FALLBACK_PIN_LOADS["strider"]
    assert pin_loads_for("jansen") is None
    assert pin_loads_for("hoecken") is None


def test_thin_lower_chicago_spacers_are_bonded_gaps_not_prints(chicago_side):
    _, _, _, fab = chicago_side
    from spiderpig.construction.pivots.common import PRINT_MIN

    names = {b.name for b in fab.bodies}
    for pin, note in fab.meta["chicago"].items():
        gap = note["bond_gap_mm"]
        lo = [n for n in names if n.endswith(f"{pin.split(':')[-1]}_spacer_lo")]
        if gap:
            assert gap < PRINT_MIN
            assert not lo                          # no part thinner than a print
        for body in fab.bodies:
            if body.fab == "printed" and body.name.endswith(("_spacer_lo", "_spacer_hi")):
                bb = body.part.bounding_box()
                assert min(bb.size.X, bb.size.Y, bb.size.Z) >= PRINT_MIN - 1e-6, body.name
