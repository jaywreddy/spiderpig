"""``klann_lego`` and the two mechanisms that were off the default crank (r4, 2026-10-05):
every ``klann_lego`` module on the hex-standoff crank, its 6061 crank rider b1 grown round the
hex sleeve's bore (``plates.rider_bosses`` on the *resolved* crank), and
``hoecken_pantograph`` on the hex crank cut from 0.080 in 6061 (``config.LINKAGE_CRANK_SHEETS``:
on 0.100 in its hub plate keeps 2.16 mm between the crankpin's hex pocket and a horn screw
hole, under 1 x t), ``dwell_rocker`` on the defaults."""

from __future__ import annotations

import pytest

from spiderpig import manufacture, servos
from spiderpig.config import (
    CRANK_SHEET,
    LINKAGE_CRANK_SHEETS,
    BuildConfig,
    ParamError,
    default_crank_sheet,
)
from spiderpig.construction.base import Context
from spiderpig.construction.plates import RIDER_BOSS_T, rider_bosses
from spiderpig.linkage import build_module_template, get
from spiderpig.stack import topology_from_template


def _ctx(cfg: BuildConfig) -> Context:
    tmpl = build_module_template(cfg.module, linkage=cfg.linkage)
    return Context(topo=topology_from_template(tmpl, samples=72), params=cfg.params,
                   pitch=cfg.pitch, servo=servos.get(cfg.servo), config=cfg)


def test_the_defaults():
    for module in ("single", "double", "decker", "quad"):
        cfg = BuildConfig(linkage="klann_lego", module=module)
        assert (cfg.crank, cfg.crank_sheet) == ("bolt", CRANK_SHEET), module
    hp = BuildConfig(linkage="hoecken_pantograph", robot=False)
    assert (hp.crank, hp.crank_sheet) == ("bolt", "al6061_2mm")
    assert hp.crank_sheet == default_crank_sheet(get("hoecken_pantograph"))
    dr = BuildConfig(linkage="dwell_rocker", robot=False)
    assert (dr.crank, dr.crank_sheet) == ("bolt", CRANK_SHEET)
    assert BuildConfig().crank_sheet == CRANK_SHEET == "al6061_2p5mm"
    assert set(LINKAGE_CRANK_SHEETS) == {"hoecken_pantograph"}
    # the 0.100 in sheet stays selectable, and a design naming its linkage's default is
    # the default design (one key)
    thick = BuildConfig(linkage="hoecken_pantograph", robot=False, crank_sheet=CRANK_SHEET)
    assert thick.crank_sheet == CRANK_SHEET
    assert not thick.is_default
    assert BuildConfig(linkage="hoecken_pantograph", robot=False,
                       crank_sheet="al6061_2mm").is_default
    with pytest.raises(ParamError):
        BuildConfig(linkage="hoecken_pantograph", robot=False, crank_sheet="m3_washer")


def test_the_cli_and_spec_take_the_linkages_crank_sheet():
    import argparse

    from spiderpig.config import add_build_args, add_design_args, config_from_args

    p = argparse.ArgumentParser()
    add_design_args(p)
    add_build_args(p)
    hp = config_from_args(p.parse_args(["--linkage", "hoecken_pantograph"]))
    assert hp.crank_sheet == "al6061_2mm"
    named = config_from_args(p.parse_args(["--linkage", "hoecken_pantograph",
                                           "--crank-sheet", CRANK_SHEET]))
    assert named.crank_sheet == CRANK_SHEET
    assert config_from_args(p.parse_args([])).crank_sheet == CRANK_SHEET


@pytest.mark.parametrize("crank", ["bolt", "bolt_round"])
def test_a_metal_rider_grows_round_the_resolved_cranks_bore(crank):
    """The boss round b1's crank bore is sized for the bore the crank really cuts: the hex
    pin's 8.5 mm sleeve (an 8.8 mm hole), not the unresolved crank's round 6 mm pin (6.3 mm),
    which left the hex bore 2.02 mm from b1's edge (a cut-rule error)."""
    cfg = BuildConfig(linkage="klann_lego", module="single", robot=False, crank=crank)
    ctx = _ctx(cfg)
    from spiderpig import construction

    c = construction.CRANKS[crank].resolve(ctx)
    hole = cfg.params.hole(c.rider_d())
    at, r = rider_bosses(ctx)["b1"]
    assert at == "M"
    t = ctx.sheet_t("link", "b1")
    assert r - hole / 2 >= RIDER_BOSS_T * t                     # 1 x t of web: no error
    if crank == "bolt":
        assert hole == pytest.approx(8.8, abs=0.06)
        assert r == pytest.approx(hole / 2 + t + 0.1)
    assert "b3" not in rider_bosses(ctx)                         # acrylic: no boss


@pytest.mark.slow
@pytest.mark.parametrize(("linkage", "module"), [
    ("klann_lego", "single"), ("klann_lego", "double"), ("klann_lego", "decker"),
    ("klann_lego", "quad"), ("hoecken_pantograph", "single"), ("dwell_rocker", "single"),
])
def test_plans_cuts_and_assembles_on_its_default_crank(linkage, module):
    """Each plans on its default crank, breaks no cut rule at the error level and has an
    assembly order (the crank's ``assembly`` note empty)."""
    from spiderpig import api
    from tests import cache

    lk = get(linkage)
    cfg = BuildConfig(linkage=linkage, module=module, robot=lk.kind == "walker")
    api.plan_config(cfg, None)
    # the parts from the fabrication cache (built from this plan when it has none)
    mech = (cache.cached_robot if cfg.robot else cache.cached_side)(cfg, 0.0)
    m = manufacture.check(mech, cfg.sheet)
    assert not m["errors"], manufacture.messages(m, "error")
    assert mech.meta["crank_bolt"]["assembly"] is None


def test_klann_lego_sets_a_lower_servo_torque_limit():
    """``klann_lego``'s 6061 leg b4 holds a jam at SF 2 only under 0.67 N·m (its foot's
    80 mm lever bends it at D): the design sets 0.60 N·m (``config.LINKAGE_TORQUE_LIMITS``),
    4.2 x its 0.14 N·m walking peak, and the BOM tells the builder; every other design keeps
    the servo's own 0.85 N·m."""
    from spiderpig.config import LINKAGE_TORQUE_LIMITS, torque_limit_nm, torque_limit_note

    kl = BuildConfig(linkage="klann_lego", module="quad")
    assert torque_limit_nm(kl) == pytest.approx(0.60) == LINKAGE_TORQUE_LIMITS["klann_lego"]
    assert torque_limit_nm(BuildConfig()) == pytest.approx(0.85)
    assert torque_limit_nm(BuildConfig(linkage="klann", module="quad")) == pytest.approx(0.85)
    note = torque_limit_note(kl)
    assert "0.6 N·m" in note
    assert "31 %" in note
    assert "LINKAGE_TORQUE_LIMITS" in note
    assert "LINKAGE_TORQUE_LIMITS" not in torque_limit_note(BuildConfig())


@pytest.mark.slow
def test_the_pillar_rings_close_the_columns_air():
    """``klann_lego``'s 6061 links make their layers 3.175 mm, so a 60 mm pillar segment
    stood 0.70 mm over the 3 mm rings' stack: 0.8 mm of play, 5.0 deg of tilt. The printed
    rings in those layers now fill them (``StandoffAxle.ring_fill``): the pillars keep only
    the 0.1 mm assumed play (0.61 deg, the Strider double's)."""
    from spiderpig import api
    from tests import cache

    cfg = BuildConfig(linkage="klann_lego", module="quad", robot=False)
    api.plan_config(cfg, None)
    mech = cache.cached_side(cfg, 1.0)          # from the fabrication cache
    notes = {k: v for k, v in mech.meta["wobble"].items() if k.startswith("pillar")}
    assert len(notes) == 4
    for name, note in notes.items():
        assert note["segments_mm"] == [60.0], name
        assert note["play_mm"] == pytest.approx(0.1), name
        assert max(lk["tilt_deg"] for lk in note["links"]) < 1.0, name
    rings = [b for b in mech.bodies if "pillar" in b.name and "_ring" in b.name]
    tall = [b for b in rings if b.part.bounding_box().size.Z > 3.1]
    assert tall
    for b in tall:
        assert abs(b.part.bounding_box().size.Z - 3.175) < 1e-3, b.name
