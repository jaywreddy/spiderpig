"""The diywalkers Klann family (``linkages/klann_variants.py``) against the pages' plans.

Plan coordinates are the pages': drawing units (LEGO hole pitches; the patent
drawing is scaled to a crank of 3) with the crank centre at (10, 10). Joint
positions are the centroids of the joint markers in the plan images (15-19 px
per unit, markers ~7 px across), so they carry about 0.1 unit of reading
error, more at the feet, which hang 10-14 units off their bars. A leg drawn
right of the crank is our orientation +1, one drawn left of it -1; both ride
the one crankpin M.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import linkage

TS = np.linspace(0.0, 2.0 * math.pi, 3600, endpoint=False)
KLANNS = ("klann_patent", "klann_lego", "klann_long_legs", "klann_high_step")

# (linkage, plan image, orientation, crankpin, joints) in plan coordinates.
PLANS = [
    # klanns-linkage-plans: "Klann Patent's Bar Lengths" (the page's left leg only)
    ("klann_patent", "klann-org-bar-lengths", -1, (7.39, 11.44),
     {"C": (0.89, 10.63), "D": (-4.95, 11.03), "E": (1.82, 18.37), "F": (-7.15, 1.37)}),
    # "Klann Joint Map" (J103..J108 right, J3..J8 left; crankpin J1 straight up)
    ("klann_lego", "klann1a-fixed-joint-map", +1, (10.0, 13.0),
     {"C": (16.92, 12.02), "D": (22.85, 11.16), "E": (18.22, 19.99), "F": (22.82, 1.15)}),
    ("klann_lego", "klann1a-fixed-joint-map", -1, (10.0, 13.0),
     {"C": (3.11, 12.02), "D": (-2.84, 11.15), "E": (1.81, 20.0), "F": (-2.78, 1.15)}),
    # "Klann Bar Map" (another crank angle)
    ("klann_lego", "klann-1b-bar-lengths", +1, (7.56, 11.69),
     {"C": (14.55, 11.15), "D": (20.51, 10.65), "E": (18.01, 20.32), "F": (18.18, 0.91)}),
    ("klann_lego", "klann-1b-bar-lengths", -1, (7.56, 11.69),
     {"C": (0.59, 11.17), "D": (-5.38, 10.68), "E": (1.16, 18.24), "F": (-7.61, 0.93)}),
    # klann-high-step-mod: "Klann Linkage with Long Legs" (simulator, crankpin up)
    ("klann_long_legs", "long-legs-with-rockers", +1, (10.0, 13.0),
     {"C": (16.91, 12.0), "D": (22.89, 11.13), "E": (18.21, 19.99), "F": (22.59, -2.87)}),
    ("klann_long_legs", "long-legs-with-rockers", -1, (10.0, 13.0),
     {"C": (3.09, 11.99), "D": (-2.89, 11.12), "E": (1.78, 19.99), "F": (-2.6, -2.88)}),
    # "High-Step Mod" (lower pivot labelled X=4, Y=8)
    ("klann_high_step", "long-leg-klann-and-hi-step-mod", +1, (10.0, 13.0),
     {"C": (16.0, 11.96), "D": (22.8, 10.82), "E": (18.3, 19.79), "F": (22.27, -3.16)}),
    ("klann_high_step", "long-leg-klann-and-hi-step-mod", -1, (10.0, 13.0),
     {"C": (4.06, 11.99), "D": (-2.8, 10.83), "E": (1.67, 19.78), "F": (-2.28, -3.17)}),
]


def _plan_xy(key: str, orientation: int, t: float) -> dict[str, np.ndarray]:
    lk = linkage.get(key)
    unit = float(lk.params["unit"])
    at = lk.solve(orientation).joints_at(t)
    return {j: np.asarray(xy) / unit + 10.0 for j, xy in at.items()}


@pytest.mark.parametrize(("key", "image", "orientation", "pin", "plan"), PLANS,
                         ids=[f"{p[1]}{p[2]:+d}" for p in PLANS])
def test_matches_its_plan_drawing(key, image, orientation, pin, plan):
    """Every joint of the leg sits where the plan draws it, at the plan's crank angle."""
    got = _plan_xy(key, orientation, math.atan2(pin[1] - 10.0, pin[0] - 10.0))
    for j, xy in plan.items():
        err = np.hypot(*(got[j] - xy))
        assert err < 0.25, f"{key} {j}: {got[j]} vs plan {xy} ({image})"


@pytest.mark.parametrize("key", KLANNS)
def test_frame_pivots_are_the_plan_coordinates(key):
    """The frame pivots the pages label, relative to the crank at (10, 10)."""
    plan = {"klann_patent": {"A": (16.6, 8.03), "B": (12.6, 16.9)},   # (3.4, 8.03), (7.4, 16.9)
            "klann_lego": {"A": (17, 8), "B": (13, 17)},               # J103, J105
            "klann_long_legs": {"A": (17, 8), "B": (13, 17)},
            "klann_high_step": {"A": (16, 8), "B": (13, 17)}}[key]    # X=4 on the left leg
    got = _plan_xy(key, +1, 0.0)
    for j, xy in plan.items():
        np.testing.assert_allclose(got[j], xy, atol=1e-9, err_msg=f"{key} {j}")


def _foot_extent(key: str) -> np.ndarray:
    """``[xmin, xmax, ymin, ymax]`` of the page's left foot path, plan units."""
    f = linkage.get(key).solve(-1).evaluate(TS)["F"] / float(linkage.get(key).params["unit"])
    return np.array([f[:, 0].min(), f[:, 0].max(), f[:, 1].min(), f[:, 1].max()]) + 10.0


# The foot-path charts ("Footpaths of Klann Patented Linkage & LEGO approximation",
# "... & Long Legged LEGO approximation"): [xmin, xmax, ymin, ymax] of every
# plotted series (swing and contact dots together), read against the charts'
# tick marks and gridlines. The charts plot 9 of their units per plan unit.
CHART_SCALE = 9.0
CHARTS = {
    "patent_vs_lego": {"klann_patent": (-196.2, -94.2, -70.3, -17.3),
                       "klann_lego": (-203.5, -100.3, -71.9, -22.4)},
    "patent_vs_long_legs": {"klann_patent": (-206.0, -104.0, -105.4, -52.3),
                            "klann_long_legs": (-213.4, -90.2, -107.0, -57.0)},
}


@pytest.mark.parametrize("chart", CHARTS)
def test_foot_path_matches_the_pages_chart(chart):
    """Stride and lift of every plotted foot path, to half a percent of the chart."""
    for key, (x0, x1, y0, y1) in CHARTS[chart].items():
        e = _foot_extent(key) * CHART_SCALE
        assert e[1] - e[0] == pytest.approx(x1 - x0, abs=1.5), f"{key} stride ({chart})"
        assert e[3] - e[2] == pytest.approx(y1 - y0, abs=1.5), f"{key} lift ({chart})"


def test_lego_foot_path_sits_where_the_chart_puts_it_beside_the_patents():
    """The first chart draws both paths about one crank: the LEGO path's offset matches."""
    plotted = CHARTS["patent_vs_lego"]
    shift_page = np.subtract(plotted["klann_lego"], plotted["klann_patent"]) / CHART_SCALE
    shift = _foot_extent("klann_lego") - _foot_extent("klann_patent")
    np.testing.assert_allclose(shift, shift_page, atol=0.15)


def test_high_step_mod_lifts_the_foot_as_the_simulator_draws():
    """The dotted foot paths in the page's simulator pictures, relative to the crank.

    Moving the lower rocker and its pivot one hole toward the crank keeps the
    stride and raises the top of the path by about 2.4 holes.
    """
    traced = {"klann_long_legs": (-9.87, 3.86, -3.07, 2.56),    # long-legs-with-rockers
              "klann_high_step": (-9.89, 3.86, -3.29, 4.98)}    # High-Step Mod panel
    for key, xy in traced.items():
        np.testing.assert_allclose(_foot_extent(key), xy, atol=0.2, err_msg=key)
    lift = {k: np.ptp(_foot_extent(k)[2:]) for k in traced}
    assert lift["klann_high_step"] - lift["klann_long_legs"] > 2.0


@pytest.mark.parametrize("key", ["klann_lego", "klann_long_legs", "klann_high_step"])
def test_lego_builds_are_whole_hole_pitches(key):
    lk = linkage.get(key)
    lengths = {k: v for k, v in lk.params.items() if k not in lk.angles and k != "unit"}
    assert all(v.is_integer for v in lengths.values()), lengths
    assert lk.params["unit"] == 8


def test_klann_patent_is_not_the_default_klann():
    """The diywalkers patent transcription differs from the 2016 proportions."""
    a = linkage.get("klann").solve().evaluate(TS)["F"]
    b = linkage.get("klann_patent").solve().evaluate(TS)["F"]
    assert np.ptp(a[:, 1]) > 1.5 * np.ptp(b[:, 1])      # klann lifts its foot much higher
    assert linkage.get("klann").family == linkage.get("klann_patent").family == "klann"
