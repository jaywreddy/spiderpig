"""Stage 2 of the joinery plan (2026-10-04): single aluminium crank webs on standoff
crankpins, the thinnest sheet per part, glue-free frame ties and deck rails, printed
spacer sleeves, take-up shims, TPU foot socks, the frame chords, the link-plate strength
check and the cut rules' error / warning levels."""

from __future__ import annotations

import math

import pytest

from spiderpig.config import DEFAULT_CRANKS, BuildConfig


def test_each_part_gets_the_thinnest_sheet_that_passes():
    """The user's rule: plates as thin as strength and the service's rules allow. The frame
    plate's 2.4 mm servo holes rule out 0.100 in and up (SendCutSend: a hole at least the
    thickness); its pillar ends rule out 0.063 in; the crank webs take 0.100 in 6061-T6, the
    thinnest whose hex crankpin pockets hold the jam twist at SF 2 (2026-10-04)."""
    from spiderpig.materials import role_report, thinnest_sheet

    cfg = BuildConfig()
    assert cfg.frame_sheet == thinnest_sheet("frame") == "al5052_2mm"
    assert cfg.crank_sheet == thinnest_sheet("crank") == "al6061_2p5mm"
    rows = {r["sheet"]: r["why_not"] for r in role_report("frame")}
    assert "minimum" in rows["al5052_3p2mm"]
    assert "SF" in rows["al5052_1p6mm"]


def test_mechanisms_default_to_the_bolt_crank():
    """Upgraded from the keyed crank (2026-10-04): the Hoecken pantograph's short crank was
    what blocked it."""
    assert DEFAULT_CRANKS == {"walker": "bolt", "mechanism": "bolt"}


def test_an_aluminium_crank_sheet_sets_the_webs_thickness_and_yield():
    """The single web plates take the crank sheet's thickness and yield; a non-metal crank
    sheet is refused (the acrylic two-plate crank was removed, W2)."""
    from spiderpig.construction.base import ConstructionError
    from spiderpig.construction.crank import BoltCrank
    from spiderpig.materials import sheet

    c = BoltCrank().for_sheet("al5052_1p6mm")
    assert c.web_t == sheet("al5052_1p6mm").thickness
    assert c.web_yield == sheet("al5052_1p6mm").yield_mpa
    with pytest.raises(ConstructionError, match="metal crank sheet"):
        BoltCrank().for_sheet("acrylic_3mm")


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


def test_a_columns_take_up_shims_stack_to_the_step():
    """A standoff pillar's take-up under the face over it: whole 1 mm shims, then
    ``SHIM_STEP`` ones, thickest first, rounded to the step."""
    from spiderpig.construction.pivots.standoff import SHIM_STEP, StandoffAxle

    a = StandoffAxle()
    assert SHIM_STEP == 0.5
    assert a.shims(3.0) == [1.0, 1.0, 1.0]
    assert a.shims(1.5) == [1.0, 0.5]
    assert a.shims(0.6) == [0.5]                 # rounded to the step
    assert a.shims(0.2) == []
    assert sum(a.shims(1.75)) == pytest.approx(2.0)


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


@pytest.fixture(scope="module")
def default_robot():
    """The default design's robot (the Strider double), from the fabrication cache."""
    from tests import cache

    cfg = BuildConfig()
    return cfg, cache.cached_robot(cfg, 1.0)


@pytest.mark.slow
def test_the_default_robot_has_no_glue_in_its_structure(default_robot):
    """Frame ties are standoff chains and screws, the deck rails are screwed, the centre
    plates are clamped, the battery cradle is screwed to the deck (2026-10-05): the only
    adhesive left is the Chicago barrels' epoxy."""
    _cfg, mech = default_robot
    glue = [x for x in mech.bom_extras if x.key in ("ca_glue", "acrylic_cement",
                                                     "epoxy_2part")]
    assert {x.key for x in glue} == {"epoxy_2part"}
    names = {b.name for b in mech.bodies}
    assert any(n.startswith("L.tie_standoff") for n in names)
    assert any(n.startswith("L.deck_rail_screw") for n in names)
    assert any(n.endswith("_sock") for n in names)
    assert not any("_ring" in b.name and b.fab == "laser" for b in mech.bodies)
    centre = [b for b in mech.bodies if b.name.startswith("centre_plate")]
    assert centre
    assert all(b.sheet == mech.meta["centre_plate_sheet"] for b in centre)
    assert math.isfinite(mech.meta["tie_engagement_mm"])


@pytest.mark.slow
def test_the_body_plates_keep_two_thicknesses_round_every_hole(default_robot):
    """The design review's levels on the body side of the default robot: the frame plates
    (a boss round every hole) and the centre plates (the ties moved off the servo's screw
    holes, the outline two thicknesses round each recess, the bump reliefs' corners
    rounded past SendCutSend's 0.8 mm) have no error; nothing anywhere is an error. Since
    the checker measures the webs round non-circular cut-outs too (the assembly audit of
    2026-10-04), the warnings inherent in the STS3215 stay at least 1 x t: the inner
    plate's far front screw holes 2.05 mm from the raised panel's relief, and in the centre
    plates (0.063 in since 2026-10-08, both rear screws kept) the far rear holes 1.62 mm from
    the raised pad's relief (merged with the bus window and channel, open to the edge) and
    the rear ties 2.91 mm from the far head recess bridged into it."""
    from spiderpig.manufacture import check
    from spiderpig.materials import sheet

    cfg, mech = default_robot
    got = check(mech, cfg.sheet)
    assert got["errors"] == {}
    body = [i for i in got["issues"] if i["part"].endswith(("torso", "frame_outer"))
            or i["part"].startswith("centre_plate")]
    assert all(i["level"] == "warning" for i in body)
    assert all(i["value"] >= sheet(i["sheet"]).thickness - 1e-6 for i in body)
    assert {(i["part"].split(".")[-1].rstrip("0123456789"), i["rule"]) for i in body} <= {
        ("torso", "web"), ("centre_plate", "edge")}
    assert not [i for i in body if i["part"].endswith("frame_outer")]
    assert mech.meta["centre_plate_sheet"] == "al5052_1p6mm"      # 2026-10-08: 0.063 in


def test_a_frame_plate_boss_is_two_thicknesses():
    from spiderpig.construction.plates import boss_web
    from spiderpig.materials import sheet

    assert boss_web("al5052_2mm") == pytest.approx(2 * sheet("al5052_2mm").thickness + 0.1)
    assert boss_web(None) == 0.0


def test_the_centre_plates_are_0p063_in():
    """The user's decision of 2026-10-04 (4), the thinnest stack that seats the most rear
    screws, was 0.090 in on the default servo; since 2026-10-08 (both rear screws per servo,
    the bus window) only 0.063 in seats both, so the centre plates go under the frame's
    0.080 in (a thinner sheet only for more screws)."""
    from spiderpig.construction.chassis import _centre_sheet
    from spiderpig.servos import get

    cfg = BuildConfig()
    assert _centre_sheet(get(cfg.servo), cfg.frame_sheet, cfg.params.margin) == "al5052_1p6mm"


def test_frame_plates_of_the_default_sheet_are_their_own_centre_plates():
    """A frame cut from a non-metal sheet keeps it for the centre plates (no aluminium
    stack to rank), and the default sheet's plates are the layer pitch thick."""
    from spiderpig.construction.chassis import _centre_sheet, centre_t
    from spiderpig.fabricate import side_problem, template_for
    from spiderpig.servos import get as servo

    assert _centre_sheet(servo("sts3215"), "acrylic_3mm", 1.0) == "acrylic_3mm"
    assert _centre_sheet(servo("sts3215"), None, 1.0) is None
    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False, frame_sheet="acrylic_3mm")
    ctx, _, _ = side_problem(template_for(cfg), cfg)
    assert centre_t(ctx) == ctx.pitch
