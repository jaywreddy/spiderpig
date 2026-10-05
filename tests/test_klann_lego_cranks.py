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
    from spiderpig.fabricate import fabricate, template_for

    lk = get(linkage)
    cfg = BuildConfig(linkage=linkage, module=module, robot=lk.kind == "walker")
    tmpl = template_for(cfg)
    api.plan_config(cfg, None)
    mech = fabricate(tmpl, cfg, 0.0)
    m = manufacture.check(mech, cfg.sheet)
    assert not m["errors"], manufacture.messages(m, "error")
    assert mech.meta["crank_bolt"]["assembly"] is None
