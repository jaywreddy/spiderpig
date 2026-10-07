"""What would clear a stage's failure (:mod:`recommend`): checked before it is said."""

from __future__ import annotations

import pytest

from spiderpig import explain
from spiderpig.config import BuildConfig
from spiderpig.construction.base import Params
from spiderpig.fabricate import design_side, side_problem, static_stage, template_for
from spiderpig.recommend import Gap, scale
from spiderpig.stack import ClearanceError


def _heel(unit: float, crank: str = "", **params) -> BuildConfig:
    """The heel on its own crank (``bolt_round``, config.LINKAGE_CRANKS: the round 6 mm
    standoff), whose 3 mm post radius these numbers are (the hex crank's 8.5 mm sleeve asks
    more of b7, and sizes the post itself: test_the_hex_crank_post_sends_the_heel_up_a_
    scale)."""
    kw = {"crank": crank} if crank else {}
    return BuildConfig(linkage="trotbot_heel", module="single", robot=False,
                       proportions=(("unit", unit),), params=Params(**params), **kw)


def _fail(cfg) -> ClearanceError:
    with pytest.raises(ClearanceError) as e:
        design_side(template_for(cfg), cfg)
    return e.value


def test_the_heel_is_told_the_scale_that_clears_it():
    """At the drawing's 7 mm unit the heel link passes the crankpin's post 6.8 mm off, under
    the 10 mm it needs: x1.47 clears it, rounded up to a 10.5 mm unit, and checked."""
    e = _fail(_heel(7.0))
    (rec,) = e.recommendations
    assert rec.changes == (("unit", 7.0, 10.5),)
    assert rec.why.startswith("scale trotbot_heel x1.50 (the least that clears it is x1.47; "
                              "b7 past crankpin J1 is 6.8 mm, needs 10.0)")
    assert rec.effects.startswith("crank 28.0 -> 42.0 mm; about 1.5x the crank torque")
    # 13 layers (47.564 mm on the round standoff crank and standoff pillars; 37.064 mm on
    # the printed crank and pillars, removed 2026-10-07)
    assert rec.verified.startswith("checked: the static stage passes, and it plans in 13 ")
    assert "what would clear it:\n  unit 7 -> 10.5: scale trotbot_heel x1.50" in str(e)
    # no thinner parts do: the 6 mm crankpin leaves b1 a wall round it
    assert any(n.startswith("no part sizes at this scale clear it within the constructions' "
                            "limits (the least: ") for n in e.notes)


def test_thinner_parts_are_checked_against_the_constructions():
    """At a 10 mm unit b7 passes 9.7 mm off, under the 10.0 its post needs: a 5.5 mm link
    radius clears it (9.5 + the 0.05 slack), and the round crankpin keeps 1.5 mm of b1 round
    its 6 mm hole there, so it is checked and offered after the scale. (At 9 mm, 8.7 off, the
    least that would clear it, a 4.5 mm link, leaves b1 too thin: the scale only.)"""
    e = _fail(_heel(10.0))
    thin = next(r for r in e.recommendations if r.why == "thinner parts at this scale")
    assert thin.changes == (("link_radius", 6.0, 5.5),)
    assert thin.verified.startswith("checked: the static stage passes")
    assert e.recommendations[0].changes == (("unit", 10.0, 10.5),)
    e = _fail(_heel(9.0))
    assert [r.changes for r in e.recommendations] == [(("unit", 9.0, 10.5),)]


@pytest.mark.parametrize(("unit", "params", "dist", "need"), [
    (8.0, {"link_radius": 5.0}, 7.8, 9.0),           # 3 post radius + 5 + 1 margin
    (10.0, {}, 9.7, 10.0),
])
def test_the_heel_just_misses_below_its_scale(unit, params, dist, need):
    cfg = _heel(unit, **params)
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    with pytest.raises(ClearanceError, match=f"it passes crankpin J1 at {dist} mm, under the "
                                             f"{need} mm a post there needs"):
        static_stage(tmpl, problem)


def test_the_hex_crank_post_sends_the_heel_up_a_scale():
    """With the hex crank (``bolt``, the walkers' default; the heel's own is ``bolt_round``)
    the heel at its own 10.5 mm unit passes J1 at 10.2 mm of the 11.2 its 8.5 mm sleeve, a
    4.25 mm post, needs; thinner parts can't help (the crank sizes its post from the hex
    standoff, recommend.gaps_of), so the one recommendation is the scale, checked."""
    e = _fail(_heel(10.5, crank="bolt"))
    assert "it passes crankpin J1 at 10.2 mm, under the 11.2 mm a post there needs" in str(e)
    (rec,) = e.recommendations
    assert rec.changes == (("unit", 10.5, 12.0),)
    # 10 layers (46.964 mm; the keyed crank's, removed 2026-10-07: 15 layers, 43.064 mm)
    assert rec.verified.startswith("checked: the static stage passes, and it plans in 10 layers")


def test_gap_arithmetic():
    gap = Gap("b7 past J1", 6.8, (("crankpin_d", 0.5), ("link_radius", 1.0)), 1.0)
    assert gap.need(Params()) == pytest.approx(10.0)
    assert gap.need(Params(link_radius=4.5, crankpin_d=5.0)) == pytest.approx(8.0)
    # a link across the part itself: no scale clears it
    assert scale(_heel(7.0), [Gap("x", -1.0, (), 1.0)]) is None


def test_explain_prints_what_would_clear_it():
    out = explain.explain("trotbot_heel", params={"unit": 7.0})
    assert "STOP: trotbot_heel: b7 sweeps right across the crank at O" in out
    assert "what would clear it:" in out
    assert "unit 7 -> 10.5" in out    # the round standoff's 6 mm crankpin (hex: -> 12)


def test_a_missed_stroke_is_met_by_the_least_practical_scale_checked():
    """``target_scale`` on the Hoecken mechanism (plans in a fraction of a second): a stroke
    target over the drawing's is met by scaling up, one under it by scaling down to the
    middle of its band, each measured again and planned; a straightness the design meets
    now bounds the scale too, and a target the bound excludes gets a note, not a fix."""
    from spiderpig.api import measure_config
    from spiderpig.recommend import target_scale
    from spiderpig.spec import Target

    cfg = BuildConfig(linkage="hoecken", module="single", robot=False)
    got = measure_config(cfg)
    stroke = got["motion.stroke_mm"]
    assert stroke == pytest.approx(66.89, abs=0.01)
    rec, note = target_scale(cfg, [("motion.stroke_mm", stroke, Target(min=80.0))],
                             measure_config)
    assert note is None
    assert rec.changes == (("unit", 16.0, 19.5),)
    assert rec.why == "stroke_mm scales with unit: x1.22 meets the target"
    assert rec.verified == "checked: stroke_mm 81.53; it plans in 7 layers (27.864 mm)"
    rec, note = target_scale(cfg, [("motion.stroke_mm", stroke, Target(value=50.0, tol=1.0))],
                             measure_config)
    assert rec.changes == (("unit", 16.0, 12.0),)
    assert rec.verified.startswith("checked: stroke_mm 50.17; it plans in 7 layers")
    straight = ("motion.straightness_mm", got["motion.straightness_mm"], Target(max=0.07))
    rec, note = target_scale(cfg, [("motion.stroke_mm", stroke, Target(min=80.0))],
                             measure_config, keep=[straight])
    assert rec is None
    assert note == ("no one scale of unit meets stroke_mm, straightness_mm together (one "
                    "needs x1.2, another at most x1.09)")
