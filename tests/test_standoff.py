"""The standoff pillar (:mod:`construction.pivots.standoff`): round 6 mm aluminium standoffs,
a column longer than any stock length one tapped steel shaft made to its length (the default
since 2026-10-05) or spliced at plate rings (``standoff_hand`` / ``standoff_bench``), and its
strength per bay."""

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
    assert s.splices({3, 5}, top=17, pitch=PITCH) == []       # 48 mm, one stock length


def test_a_splice_goes_where_the_moment_is_least_among_stock_lengths():
    s = AXLES["standoff_hand"]
    sp = s.splices({3, 9, 12}, top=23, pitch=PITCH)           # 66 mm: one splice
    assert len(sp) == 1
    # goBILDA's stock lengths, and since 2026-10-04 a segment may also be up to 2 mm short
    # where DIN 988 shims fit under its upper face (a spacer layer, not a link): layer 18,
    # far past the last link, sees the least moment of the splices that fit
    assert sp == [18]
    faces = [0, *sp, 23]
    for a, b in zip(faces, faces[1:], strict=False):
        assert s.segment((b - a - 1) * PITCH, b - 1 not in {3, 9, 12}) is not None


def test_only_lengths_gobilda_sells():
    from spiderpig.hardware.catalog import get
    from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS, gobilda_1501

    # the M4 standoff listing, fetched 2026-10-04 (the 33, 39 and 45 mm pages are 404s)
    assert {13, 15, 21, 33, 39, 45, 51, 57}.isdisjoint(GOBILDA_LENGTHS)
    assert {12, 18, 19, 27, 43, 54, 60} <= set(GOBILDA_LENGTHS)
    assert all(get(gobilda_1501(L)).offers[0].verified for L in GOBILDA_LENGTHS)


def test_the_moment_at_a_splice():
    from spiderpig.construction.wobble import moment_at_per_newton

    t = 3.0
    note = {"pitch_mm": t, "span_mm": 0.0, "bearing_len_mm": t, "anchors": [0, 20],
            "layers": {"a": 10}, "section": StandoffAxle().section().as_dict()}
    za, zb, zf = 1.5, 58.5, 30.0                       # the link at layer 10: z 30
    near = moment_at_per_newton(note, 6.0)
    mid = moment_at_per_newton(note, zf)
    assert mid == pytest.approx((zf - za) * (zb - zf) / (zb - za))
    assert near < mid / 3


def test_a_long_column_splices_only_at_ring_layers():
    s = AXLES["standoff_hand"]
    links = {2, 5, 9, 14, 20, 21, 25, 30}
    top = 34                                                  # 99 mm between the plates
    sp = s.splices(links, top, PITCH)
    assert sp
    assert not set(sp) & links
    faces = [0, *sp, top]
    for a, b in zip(faces, faces[1:], strict=False):
        length = (b - a - 1) * PITCH
        assert s.min_segment <= length <= s.max_segment
        assert get(gobilda_1501(length)).dims["od"] == 6.0


def test_a_long_column_is_one_shaft_made_to_its_length():
    """The default (2026-10-05): a column no single goBILDA length fills is never spliced
    (a splice is a joint mid-span: at the plan's own z the Strider's hand-tight splices
    opened at jam SF 0.5-1.4) but one 6 mm round steel standoff tapped M3 both ends, made to
    its length (MISUMI NETRF6: 0.1 mm steps)."""
    s = AXLES["standoff"]
    assert s.splice_build == "shaft"
    links = {2, 5, 9, 14, 20, 21, 25, 30}
    assert s.splices(links, 34, PITCH) == []                    # 99 mm: no splice
    one = s.column_axle(links, 34, PITCH)
    assert (one.stock, one.size, one.splice_build) == ("shaft", "M3", "none")
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
    assert s.splices(set(range(1, 34)), 34, PITCH) == []


def test_no_splice_layer_means_no_column():
    s = AXLES["standoff_hand"]
    links = set(range(1, 34))                                 # every layer a link
    assert s.splices(links, 34, PITCH) is None


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


def test_splice_capacity_is_the_gapping_moment():
    """The preload of two round standoffs turned together on steel shims (2026-10-04: the
    end screws' 0.8 N·m on an acrylic ring overclaimed it). The user's decision of
    2026-10-05: the splice is turned on **by hand** in the bottom-up assembly, rated at 0.4
    N·m, with medium threadlocker on the stud; the bench-built column (1.0 N·m in soft-jaw
    pliers, the supported splice of 2026-10-04) stays selectable as ``standoff_bench``."""
    s = AXLES["standoff_hand"]
    assert (s.splice_build, s.splice_nm) == ("hand", 0.4)
    f = 0.4 / (0.2 * 0.004)                              # 500 N
    assert s.splice_capacity_nmm() == pytest.approx(f * (9 + 2.15 ** 2) / 12)
    assert s.splice_capacity_nmm() == pytest.approx(567.6, abs=0.1)
    basis = s.splice_basis()
    for word in ("UNVERIFIED", "by hand", "threadlocker"):
        assert word in basis
    bench = AXLES["standoff_bench"]
    assert (bench.splice_build, bench.splice_nm) == ("bench", 1.0)
    assert bench.splice_capacity_nmm() == pytest.approx(1419.0, abs=0.1)
    for word in ("UNVERIFIED", "first build", "soft-jaw", "bench"):
        assert word in bench.splice_basis()
    assert bench.roles == ("pillar",)


def test_a_bench_built_column_splices_only_under_its_links():
    """A link can't pass a splice's 8 mm shims, so a column spliced on the bench takes its
    links over its top: its splices lie below the lowest link, or it doesn't build."""
    hand, bench = AXLES["standoff_hand"], AXLES["standoff_bench"]
    links = {9, 12, 15}
    top = 23                                                  # 66 mm: one splice
    sp = bench.splices(links, top, PITCH)
    assert sp
    assert max(sp) < min(links)
    assert hand.splices(links, top, PITCH) is not None
    # a link low in the column leaves no room under it for a splice
    assert bench.splices({2, 9, 12}, top, PITCH) is None
    assert hand.splices({2, 9, 12}, top, PITCH)


def test_the_splice_stud_takes_medium_threadlocker():
    s = AXLES["standoff_hand"]
    assert s.splice_lock_key == "threadlocker_243"
    assert get(s.splice_lock_key).offers


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
        column = (sum(v["segments_mm"]) + sum(v["shims_mm"].values())
                  + len(v["splices"]) * d.ctx.pitch)
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
