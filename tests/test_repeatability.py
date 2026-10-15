"""ISO 9283 pose repeatability by Monte Carlo (:mod:`spiderpig.repeatability`, 2026-10-12):
gravity seats the palletizer's pins, an unseated pin's play counts, re-homing adds."""

from __future__ import annotations

import pytest

from spiderpig import linkage
from spiderpig.linkage.constraints import layout
from spiderpig.repeatability import SEAT_N, GravityLoads, gravity_loads, iso_inputs, iso_rp

pytestmark = pytest.mark.no_fabricate


@pytest.fixture(scope="module")
def pal():
    return linkage.get("palletizer")


def test_gravity_loads_every_palletizer_pin_at_the_iso_poses(pal):
    loads = gravity_loads(pal, pal.values(), link_g_per_mm=0.11, payload_g=50.0)
    assert len(iso_inputs(pal)[0]) == 5
    assert min(min(f) for f in loads.force_n.values()) > SEAT_N
    assert set(loads.torque_nm) == {"b1", "b2"}


def test_seated_pins_dont_count_and_rehoming_does(pal):
    lay = layout(pal)
    clear = {i: 0.05 for i in lay.interfaces if i[1] != "torso"}
    loads = gravity_loads(pal, pal.values(), link_g_per_mm=0.11, payload_g=50.0)
    none = dict.fromkeys(pal.inputs, 0.0)
    seated = iso_rp(pal, pal.values(), clear, loads, none, none, none, {}, 0.0)
    assert seated.session_mm < 0.03
    assert not seated.unseated
    weightless = GravityLoads({i: f * 0 for i, f in loads.force_n.items()},
                              {b: t * 0 for b, t in loads.torque_nm.items()})
    loose = iso_rp(pal, pal.values(), clear, weightless, none, none, none, {}, 0.0)
    assert loose.session_mm > 5 * seated.session_mm
    homed = iso_rp(pal, pal.values(), clear, loads, none, dict.fromkeys(pal.inputs, 0.05),
                   none, {}, 0.0)
    assert homed.powerup_mm > 3 * homed.session_mm
