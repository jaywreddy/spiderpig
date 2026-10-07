"""The standoff pillar (:mod:`construction.pivots.standoff`): round 6 mm aluminium standoffs,
a column no stock length fills one tapped steel shaft made to its length (never spliced:
the spliced variants were removed on 2026-10-07), its shims and end screws, and its strength
per bay."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig.construction import AXLES
from spiderpig.construction.base import ConstructionError
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.construction.pivots.standoff import StandoffAxle
from spiderpig.construction.wobble import beam, bending_case, stresses
from spiderpig.hardware.catalog import get
from spiderpig.hardware.crank_catalog import gobilda_1501

PITCH = 3.0


def test_registered_as_a_pillar_only():
    s = AXLES["standoff"]
    assert isinstance(s, StandoffAxle)
    assert s.roles == ("pillar",)


def test_a_short_column_is_one_segment():
    s = StandoffAxle()
    assert s.column_axle({3, 5}, top=17, pitch=PITCH) is s     # 48 mm, one stock length


def test_a_short_column_no_stock_length_fills():
    """A column no longer than one goBILDA standoff that no stock length fills is built as
    the shaft only where goBILDA standoffs spliced at link-free layers would have filled it
    (``StandoffAxle._spliceable``, the rule the spliced pillar left behind), else refused."""
    s = StandoffAxle()
    # 33 mm between the plates' faces (layers 1..11), a link under the inner plate (no
    # shims): goBILDA sells 32 and 34, neither within -0.1 / +0.8 of 33
    assert s.segment(33.0) is None
    assert s._spliceable({11}, 12, PITCH, 0, None)            # e.g. 12 + 18 at layer 5
    one = s.column_axle({11}, 12, PITCH)
    assert (one.stock, one.size) == ("shaft", "M3")
    # every layer a link: nowhere to have spliced it, so the short column stays refused
    every = set(range(1, 12))
    assert not s._spliceable(every, 12, PITCH, 0, None)
    assert s.column_axle(every, 12, PITCH) is None
    # too short for any standoff's end threads: 9 mm
    assert s.column_axle({1}, 4, PITCH) is None


def test_shims_stack_in_whole_and_half_millimetres():
    s = StandoffAxle()
    assert s.shims(1.5) == [1.0, 0.5]
    assert s.shims(2.0) == [1.0, 1.0]
    assert s.shims(0.7) == [0.5]            # rounded to the 0.5 mm step
    assert s.shims(0.2) == []               # under half a step: left as play
    # shims let a stock length stand up to max_shims short of its gap under a spacer layer
    assert s.segment(53.0) is None                  # goBILDA 52 is 1 mm short, 54 1 mm long
    assert s.segment(53.0, shims=True) == 52.0


def test_end_screws():
    """A button head and washer through each frame plate into the column's end, the most
    thread up to the end's depth (half the shortest standoff, 6 mm): an M4 x 8 through the
    0.080 in frame plate, an M4 x 6 straight into a cantilever's free end; the shaft's M3."""
    s = StandoffAxle()
    assert s.end_screw(2.032)[:2] == ("m4_bhcs_8", 8)
    assert s.end_screw(PITCH, plate=False)[:2] == ("m4_bhcs_6", 6)
    assert s.one_piece().end_screw(2.032)[:2] == ("m3_bhcs_8", 8)
    assert s.end_screw(30.0) is None                    # no stock screw grips 30 mm


def test_only_lengths_gobilda_sells():
    from spiderpig.hardware.catalog import get
    from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS

    # the M4 standoff listing, fetched 2026-10-04 (the 33, 39 and 45 mm pages are 404s)
    assert {13, 15, 21, 33, 39, 45, 51, 57}.isdisjoint(GOBILDA_LENGTHS)
    assert {12, 18, 19, 27, 43, 54, 60} <= set(GOBILDA_LENGTHS)
    assert all(get(gobilda_1501(L)).offers[0].verified for L in GOBILDA_LENGTHS)


def test_a_long_column_is_one_shaft_made_to_its_length():
    """The default (2026-10-05): a column no single goBILDA length fills is never spliced
    (a splice is a joint mid-span: at the plan's own z the Strider's hand-tight splices
    opened at jam SF 0.5-1.4) but one 6 mm round steel standoff tapped M3 both ends, made to
    its length (MISUMI NETRF6: 0.1 mm steps)."""
    s = AXLES["standoff"]
    links = {2, 5, 9, 14, 20, 21, 25, 30}
    one = s.column_axle(links, 34, PITCH)                      # 99 mm
    assert (one.stock, one.size) == ("shaft", "M3")
    length = one.segment(99.0, True)
    assert length is not None
    assert abs(length - 99.0) <= 0.1 + 1e-9
    item = get(one.segment_key(length))
    assert item.dims["od"] == 6.0
    assert item.offers[0].vendor == "MISUMI"
    # 1018 taken at its hot-rolled 220 MPa: about the goBILDA tube's bending strength, and
    # no splice in it
    steel, alu = one.section(), s.section()
    assert steel.z_bend * steel.yield_mpa == pytest.approx(alu.z_bend * alu.yield_mpa, rel=0.05)
    # a column one stock length fills stays a goBILDA standoff
    assert s.column_axle({3, 5}, 17, PITCH).stock == ""
    # even every layer a link: one shaft, nothing to splice
    assert s.column_axle(set(range(1, 34)), 34, PITCH).stock == "shaft"


def test_pins_are_refused(design):
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import side_problem, template_for

    cfg = BuildConfig(linkage="klann", module="single", robot=False, pin="standoff")
    with pytest.raises(ConstructionError, match="pillar only"):
        side_problem(template_for(cfg), cfg)


def test_a_beam_per_bay():
    """Three supports: each bay is its own simply supported beam (half the span, a quarter
    of the moment of one bay for a mid-bay load)."""
    t = 3.0
    note = {"pitch_mm": t, "span_mm": 0.0, "bearing_len_mm": t, "anchors": [0, 20],
            "layers": {"a": 5}, "section": StandoffAxle().section().as_dict()}
    one = stresses(note, 100.0)
    bays = stresses(dict(note, supports=[0, 10, 20]), 100.0)
    assert bending_case(dict(note, supports=[0, 10, 20])) == "pillar in 2 bays"
    assert bays["moment_nmm"] < one["moment_nmm"]
    za, zb = 1.5, 30.0
    m, _ = beam(np.array([15.0]), np.array([[100.0, 0.0]]), ("bays", (za, zb, 58.5)))
    assert m == pytest.approx(100.0 * (15.0 - za) * (zb - 15.0) / (zb - za))


def test_the_section_against_printed():
    s = StandoffAxle().section()
    printed = math.pi * 6 ** 3 / 32 * 50.0
    assert s.z_bend * s.yield_mpa / printed == pytest.approx(4.36, abs=0.05)





@pytest.mark.slow
def test_the_klann_single_builds_clean_with_standoff_pillars(design, side):
    tmpl, d = design("single", pillar="standoff")
    mech = side("single", pillar="standoff")
    assert check_side(d, mech) == []
    assert clashes(mech) == []
    assert bad_solids(mech) == []
    notes = mech.meta["wobble"]
    pillars = {k: v for k, v in notes.items() if k.startswith("pillar:")}
    assert pillars
    plan, s = d.plan, StandoffAxle()
    # between the frame plates' inner faces at the plan's own z (its clearance gaps and
    # thicker aluminium layers included; since 2026-10-04 the column ends on the inner
    # plate's face, screwed through it, not glued flush in it)
    gap = plan.z(plan.top)[0] - plan.z(0)[1]
    air = sum(max(0.0, plan.t(k) - d.ctx.pitch) for k in range(1, plan.top))
    for v in pillars.values():
        assert v["supports"] == v["anchors"]
        assert len(v["segments_mm"]) == 1                    # one piece, never spliced
        column = sum(v["segments_mm"]) + sum(v["shims_mm"].values())
        if v["anchors"] == [0, d.plan.top]:                 # a beam between the plates
            # stock lengths and shims to within the tolerances the column takes
            assert gap - air - s.max_short - 1e-6 <= column <= gap + s.max_long + 1e-6, \
                (column, gap)
        else:                                               # a cantilever: to its last link
            assert len(v["anchors"]) == 1
            ks = sorted(v["layers"].values())
            # from the plate's inner face to the end layer's face over (under) its last link,
            # at the plan's own z: a clearance gap under the end layer (its washers, where the
            # crank's screw heads sit since the hex crank) is part of the column
            if v["anchors"] == [0]:
                reach = plan.z(ks[-1] + 1)[0] - plan.z(0)[1]
                rng = range(1, ks[-1] + 1)
            else:
                reach = plan.z(plan.top)[0] - plan.z(ks[0] - 1)[1]
                rng = range(ks[0], plan.top)
            reach -= sum(max(0.0, plan.t(k) - d.ctx.pitch) for k in rng)
            assert reach - s.max_short - 1e-6 <= column <= reach + s.max_long + 1e-6, \
                (column, reach)
        assert v["section"]["yield_mpa"] == 240.0
    keys = {b.bom_key for b in mech.bodies if b.name.startswith("pillar_")}
    assert any(k and k.startswith("gobilda_1501_") for k in keys)
    assert "m4_washer" in keys
    # each standoff is modelled at its stock length (BOM) or its gap's span, never shorter
    # than the stock part, and its shims close the column up to the face over it
    for b in mech.bodies:
        if b.name.startswith("pillar_") and "_standoff" in b.name:
            bb = b.part.bounding_box()
            stock = float(b.bom_key.rsplit("_", 1)[1])
            # a stock segment up to max_long over its span is drawn at the span: the column
            # holds its faces that far apart (the note's play), as StandoffAxle.segment allows
            least = stock - s.max_short - air - s.max_long - 1e-3
            assert least <= bb.max.Z - bb.min.Z, b.name
    for name, v in pillars.items():
        stem = name.replace(":", "_")
        for k in v["shims_mm"]:
            sh = next(b for b in mech.bodies if b.name == f"{stem}_shims{k}")
            assert pytest.approx(plan.z(int(k))[0], abs=1e-3) == sh.part.bounding_box().max.Z
