"""The standoff pillar (:mod:`construction.pivots.standoff`): round 6 mm aluminium standoffs
spliced at plate rings, and its strength per bay."""

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
    s = StandoffAxle()
    sp = s.splices({3, 9, 12}, top=23, pitch=PITCH)           # 66 mm: one splice
    assert len(sp) == 1
    # goBILDA sells 12, 18, 24, 27, 30, 36, 42, 48, 54, 60 mm (of the 3 mm multiples): one
    # splice cuts 66 mm into two of them only at layer 10 (27 + 36) or 13 (36 + 27); 13,
    # past the last link, sees less moment
    assert sp == [13]
    faces = [0, *sp, 23]
    assert all((b - a - 1) * PITCH in s.lengths() for a, b in zip(faces, faces[1:], strict=False))


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
    s = StandoffAxle()
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


def test_no_splice_layer_means_no_column():
    s = StandoffAxle()
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
    s = StandoffAxle()
    f = 0.8 / (0.2 * 0.004)
    assert s.splice_capacity_nmm() == pytest.approx(f * (9 + 2.15 ** 2) / 12)


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
    gap = (d.plan.top - 1) * d.ctx.pitch
    for v in pillars.values():
        assert v["supports"] == v["anchors"]
        column = sum(v["segments_mm"]) + len(v["splices"]) * d.ctx.pitch
        if v["anchors"] == [0, d.plan.top]:                 # a beam between the plates
            assert column == pytest.approx(gap + d.ctx.pitch)   # through the inner plate
        else:                                               # a cantilever: to its last link
            assert len(v["anchors"]) == 1
            ks = sorted(v["layers"].values())
            reach = (ks[-1] if v["anchors"] == [0] else d.plan.top + 1 - ks[0]) * d.ctx.pitch
            assert column == pytest.approx(reach)
        assert v["section"]["yield_mpa"] == 240.0
    keys = {b.bom_key for b in mech.bodies if b.name.startswith("pillar_")}
    assert any(k and k.startswith("gobilda_1501_") for k in keys)
    assert "m4_washer" in keys
