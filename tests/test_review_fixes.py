"""Regressions from the adversarial review of 2026-10-05: the frame ties on every servo, shim
heights the BOM can't split, and Chicago pins whose lower spacer is thinner than a print."""

from __future__ import annotations

import pytest
from build123d import Location

from spiderpig.construction.chassis import _chain, _shims
from spiderpig.hardware.bom import BomLine, split_shims
from spiderpig.mechanism import Body
from spiderpig.shapes import disc


@pytest.mark.parametrize(("span", "segs", "take"), [
    (32.0, [30], 2.0),          # the STS3215's chain
    (34.0, [15, 18], 1.0),      # the XL430's
    (23.0, [20], 3.0),          # the XL330's: no M3 pair fills it, 3 mm of shims
    (30.1, [30], 0.0),          # within the plates' 0.1 mm: no 0.1 mm shim
    (31.1, [30], 1.0),
    (30.6, [30], 0.5),
])
def test_tie_chains_take_up_in_whole_steps(span, segs, take):
    got = _chain(span)
    assert got == (segs, take)
    assert sum(_shims(take)) == pytest.approx(take)
    assert all(s in (1.0, 0.5) for s in _shims(take))


def _ring(name: str, t: float) -> Body:
    part = (disc((0, 0), 3.0, 0.0, t) - disc((0, 0), 1.55, -1.0, t + 1.0)).moved(Location())
    return Body(name, part=part, fab="purchased", bom_key="shim_din988_3x6")


def test_a_shim_height_the_steps_cant_make_is_not_dropped():
    ok = _ring("L.tie_shims0", 1.5)                # 1.0 + 0.5: two DIN 433 lines worth
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, ok.name),
                                BomLine("shim_din988_3x6", 1, "frame tie 0, L")],
                               {ok.name: ok})
    assert not notes
    assert sum(x.qty for x in lines if x.key == "m3_washer_433") == 3   # 2 for 1 mm, 1 for 0.5
    odd = _ring("L.tie_shims1", 0.1)               # no step makes 0.1 mm
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, odd.name)], {odd.name: odd})
    assert notes                                   # said, and kept as the family's line
    assert [x.key for x in lines] == ["shim_din988_3x6"]


@pytest.mark.slow
def test_an_xl330_robot_builds_its_frame_ties():
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import design_side, fabricate, template_for

    cfg = BuildConfig(linkage="strider", module="single", servo="xl330_m288", robot=True)
    tmpl = template_for(cfg)
    design_side(tmpl, cfg)
    mech = fabricate(tmpl, cfg, 1.0)
    assert mech.meta["ties"] > 0
    assert mech.meta["tie_shims_mm"] == pytest.approx(3.0)

