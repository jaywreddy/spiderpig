"""Stage 2 of the joinery plan (2026-10-04): single aluminium crank webs on standoff
crankpins, the thinnest sheet per part, glue-free frame ties and deck rails, printed
spacer sleeves, shim splices, TPU foot socks, the frame chords, the link-plate strength
check and the cut rules' error / warning levels."""

from __future__ import annotations

import math

import pytest

from spiderpig.config import DEFAULT_CRANKS, BuildConfig


def test_each_part_gets_the_thinnest_sheet_that_passes():
    """The user's rule: plates as thin as strength and the service's rules allow. The frame
    plate's 2.4 mm servo holes rule out 0.100 in and up (SendCutSend: a hole at least the
    thickness); its pillar ends rule out 0.063 in; the crank webs take 0.063 in."""
    from spiderpig.materials import role_report, thinnest_sheet

    cfg = BuildConfig()
    assert cfg.frame_sheet == thinnest_sheet("frame") == "al5052_2mm"
    assert cfg.crank_sheet == thinnest_sheet("crank") == "al5052_1p6mm"
    rows = {r["sheet"]: r["why_not"] for r in role_report("frame")}
    assert "minimum" in rows["al5052_3p2mm"]
    assert "SF" in rows["al5052_1p6mm"]


def test_mechanisms_default_to_the_bolt_crank():
    """Upgraded from the keyed crank (2026-10-04): the Hoecken pantograph's short crank was
    what blocked it."""
    assert DEFAULT_CRANKS == {"walker": "bolt", "mechanism": "bolt"}


def test_an_aluminium_crank_sheet_resolves_to_single_webs():
    from spiderpig.construction.crank import BoltCrank

    c = BoltCrank().for_sheet("al5052_1p6mm")
    assert c.single
    assert not c.two_layer_top
    assert not c.two_layer_bottom
    assert not BoltCrank().for_sheet("acrylic_3mm").single


def test_a_crankpin_standoff_is_stock_and_its_shims_fill_the_rest():
    from spiderpig.construction.crank import BoltCrank
    from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS

    c = BoltCrank().for_sheet("al5052_1p6mm")
    j = c.fit_web(free=9.0, gap=4.4, t_lo=1.6, t_top=1.6)
    assert j is not None
    assert all(s in GOBILDA_LENGTHS for s in j.segments)
    assert j.length >= 9.0 - 1e-9
    assert j.shims == pytest.approx(round(9.0 + 4.4 - j.length, 1))
    assert j.engage_lo >= c.pin_min_engage
    assert j.engage_hi >= c.pin_min_engage


def test_edge_distance_under_one_thickness_in_metal_is_an_error(tmp_path):
    """The design-review limits: under 1 x t an error, under 2 x t a warning."""
    from build123d import Box, Cylinder, Pos

    from spiderpig.manufacture import part_issues
    from spiderpig.mechanism import Body

    def plate_with_hole(edge: float):
        t = 2.032
        p = Box(40, 20, t).move(Pos(0, 0, t / 2))
        x = 20 - edge - 2.25
        return Body(name="p", part=p - Cylinder(2.25, 10).move(Pos(x, 0, 0)), fab="laser")

    near = part_issues(plate_with_hole(1.0), "al5052_2mm")
    mid = part_issues(plate_with_hole(3.0), "al5052_2mm")
    far = part_issues(plate_with_hole(5.0), "al5052_2mm")
    assert [i["level"] for i in near if i["rule"] == "edge"] == ["error"]
    assert [i["level"] for i in mid if i["rule"] == "edge"] == ["warning"]
    assert not [i for i in far if i["rule"] == "edge"]


def test_a_splice_is_steel_shims_rated_at_a_hand_tight_clamp():
    from spiderpig.construction.pivots.standoff import StandoffAxle

    a = StandoffAxle()
    assert sum(a.splice_shims(3.0)) == pytest.approx(3.0)
    f = a.splice_nm / (0.2 * 0.004)
    ro, ri = a.od / 2, a.stud_hole / 2
    assert a.splice_capacity_nmm() == pytest.approx(f * (ro * ro + ri * ri) / (4 * ro))
    assert "UNVERIFIED" in a.splice_basis()


def test_frame_chords_join_neighbouring_pillars():
    from spiderpig.construction.plates import chords

    o = (0.0, 0.0)
    two = [(-70.0, 50.0), (70.0, 50.0)]
    assert len(chords(o, two)) == 1
    far = [(-70.0, 0.0), (70.0, 0.0)]              # 180 deg apart: no chord
    assert chords(o, far) == []


def test_a_foot_sock_wraps_the_toe_and_notches_the_flanks():
    from spiderpig.construction.plates import SOCK_T, foot_sock

    sock, notches = foot_sock((0.0, 0.0), (-40.0, 0.0), 6.0, 0.0, 3.0)
    bb = sock.bounding_box()
    assert pytest.approx(6.0 + SOCK_T, abs=0.05) == bb.max.X
    assert pytest.approx(0.0) == bb.min.Z
    assert pytest.approx(3.0) == bb.max.Z
    assert len(notches) == 2


def test_link_plates_are_rated_against_their_sheet():
    """A plate's net section at its most loaded hole; a link that fails acrylic names the
    aluminium sheet that holds it."""
    from spiderpig.strength import link_rows

    cfg = BuildConfig()
    joints = [{"stem": "J3", "links": ["b1_leg0", "b2_leg0", "b3_leg0"], "crank": False,
               "walk": {"n": 5.0}, "jam": {"n": 400.0}}]
    rows = {r["joint"]: r for r in link_rows(cfg, {"source": "sim", "joints": joints})}
    b1 = rows["link:b1"]
    w = 2 * cfg.params.link_radius
    assert b1["jam"]["stress_mpa"] == pytest.approx(2.5 * 400 / ((w - 6.35) * 3.0), rel=1e-3)
    assert b1["jam"]["safety"] < 2
    assert b1["needs"]["sheet"].startswith("al6061")
    assert link_rows(cfg, {"source": "fallback"}) == []


@pytest.mark.slow
def test_the_default_robot_has_no_glue_in_its_structure():
    """Frame ties are standoff chains and screws, the deck rails are screwed, the centre
    plates are clamped: the only adhesive left is the Chicago barrels' epoxy (and the
    battery cradle's CA)."""
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig()
    mech = fabricate(template_for(cfg), cfg)
    glue = [x for x in mech.bom_extras if x.key in ("ca_glue", "acrylic_cement",
                                                     "epoxy_2part")]
    assert {x.key for x in glue} <= {"epoxy_2part", "ca_glue"}
    assert all("cradle" in x.where for x in glue if x.key == "ca_glue")
    names = {b.name for b in mech.bodies}
    assert any(n.startswith("L.tie_standoff") for n in names)
    assert any(n.startswith("L.deck_rail_screw") for n in names)
    assert any(n.endswith("_sock") for n in names)
    assert not any("_ring" in b.name and b.fab == "laser" for b in mech.bodies)
    centre = [b for b in mech.bodies if b.name.startswith("centre_plate")]
    assert centre
    assert all(b.sheet == mech.meta["centre_plate_sheet"] for b in centre)
    assert math.isfinite(mech.meta["tie_engagement_mm"])
