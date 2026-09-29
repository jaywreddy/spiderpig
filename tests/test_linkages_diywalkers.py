"""The diywalkers.com linkages match their published plans.

Joint positions below are read off the plan images (centroids of the joint
dots, or intersections of lines fitted to the bars of the Scratch simulator
drawings), in drawing units, with the crank angle taken from the crankpin
as drawn. The frame is calibrated on the fixed pivots, so these check every
moving joint. Linkages that can't be laid out yet (see their modules) are
defined but not registered; they're checked here all the same.
"""

from __future__ import annotations

import functools
import math

import numpy as np
import pytest
import sympy as sp

import linkage
from linkages import fourbar, jansen, sixbar, strider, trotbot

TS = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
DEFINED = {lk.key: lk for lk in (
    fourbar.FOURBAR, linkage.get("fourbar_spot_micro"), linkage.get("fourbar_spot_micro_v2"),
    *sixbar.UNREGISTERED, sixbar.SIXBAR_V3, *trotbot.UNREGISTERED,
)}


@functools.cache
def _step_functions(lk: linkage.Linkage):
    """Each step of the program as its own numpy function (quick to compile at any depth)."""
    fns, before = [], []
    for name, expr in lk.steps:
        args = [linkage.t, *lk.symbols.values(), *(s for n in before for s in linkage.P(n))]
        fns.append((name, sp.lambdify(args, list(expr), modules="numpy")))
        before.append(name)
    return fns


def points(lk: linkage.Linkage, ts) -> dict[str, np.ndarray]:
    """Every point over crank angles ``ts``, in drawing units (``name -> (..., 2)``).

    Registered linkages go through the engine; the others are run step by step.
    """
    ts = np.asarray(ts, dtype=float)
    unit = float(lk.params["unit"])
    if lk.key in linkage.available():
        return {k: v / unit for k, v in lk.solve().evaluate(ts).items()}
    vals, chain, out = lk.defaults, [], {}
    for name, fn in _step_functions(lk):
        xy = np.broadcast_arrays(*fn(ts, *vals, *chain), ts)[:2]
        chain += xy
        out[name] = np.stack(xy, axis=-1) / unit
    return out


# ---------------------------------------------------------------------------
# Joint maps
# ---------------------------------------------------------------------------

# TrotBot's joint map ("schematic-with-joint-labels-2"): crank centre J0 at (0, 10).
TROTBOT_MAP = {
    "J1": (-2.74, 7.06), "J2": (0.14, 12.34), "J4": (-8.77, 16.92), "J5": (-12.50, 12.17),
    "J6": (-13.71, 10.59), "J8": (-5.72, 7.52), "J9": (-3.23, 6.17), "J7": (-13.25, 2.61),
}
TROTBOT_HEEL_MAP = {**TROTBOT_MAP, "J10": (-11.05, 4.06), "J11": (-9.22, 2.22),
                    "J12": (-8.49, 1.52)}
# The toe plan ("toe-schematic-bar-lengths-1"): crank centre at (10, 10).
TROTBOT_TOE_MAP = {
    "J1": (12.13, 13.38), "J2": (10.39, 19.09), "J4": (1.15, 15.22), "J5": (1.93, 9.29),
    "J6": (2.18, 7.31), "J8": (9.75, 11.60), "J9": (12.45, 12.41), "J7": (8.72, 2.66),
    "J10": (9.00, 5.29), "J11": (11.56, 5.29), "J12": (12.56, 5.29), "J13": (5.45, 5.01),
    "J14": (8.04, 5.32), "J15": (7.30, 1.67),
}
# Scratch simulator drawings, crank centre at the origin (units of the sliders).
SCRATCH = {
    # the page's "Spot Micro Ver 2 with longer legs" (its GIF's linkage)
    "fourbar": {"M": (-40.2, 0.0), "K": (-113.0, 32.8), "F": (-142.5, -62.5)},
    # "Spot Micro Ver 1's linkage dimensions", the leg with the crankpin to the left
    "fourbar_spot_micro": {"M": (-29.8, -2.5), "K": (-86.7, 16.8), "F": (-109.5, -49.5)},
    # Ver 2 in the 6-bar simulator (B5 = 0, B6 = 100, B7 = 20), front leg
    "fourbar_spot_micro_v2": {"M": (-40.3, 0.1), "K": (-112.8, 33.0), "F": (-136.5, -53.7)},
    "sixbar": {"M": (-39.5, -6.9), "K": (-111.0, 28.6), "P": (-128.1, 18.0),
               "Q": (-57.3, 1.9), "F": (-139.5, -70.9)},
    "sixbar_v1": {"M": (-32.6, -22.9), "K": (-102.0, 17.1), "P": (-116.0, 2.8),
                  "Q": (-50.0, -12.9), "F": (-137.9, -94.7)},
    "sixbar_v2": {"M": (-39.9, -0.1), "K": (-103.7, 48.3), "P": (-113.5, 46.0),
                  "F": (-137.6, -51.2)},
    # "4-bar to 6-bar" figure (B7 not shown: B6 meets the crankpin, as in Ver 3)
    "sixbar_v3": {"M": (-30.7, -25.8), "K": (-86.8, 31.3), "P": (-101.7, 17.5),
                  "F": (-120.0, -70.2)},
}
# key -> (plan, crank centre on the plan, crankpin, tolerance in drawing units)
PLANS = {
    "trotbot": (TROTBOT_MAP, (0.0, 10.0), "J1", 0.1),
    "trotbot_heel": (TROTBOT_HEEL_MAP, (0.0, 10.0), "J1", 0.1),
    "trotbot_toe": (TROTBOT_TOE_MAP, (10.0, 10.0), "J1", 0.1),
    **{k: (v, (0.0, 0.0), "M", 0.5) for k, v in SCRATCH.items()},
}


def test_every_definition_has_a_plan():
    assert set(PLANS) == set(DEFINED)


@pytest.mark.parametrize("key", sorted(PLANS))
def test_matches_its_plan(key):
    plan, (cx, cy), pin, tol = PLANS[key]
    lk = DEFINED[key]
    px, py = plan[pin]
    at = points(lk, [math.atan2(py - cy, px - cx)])
    for j, (x, y) in plan.items():
        got = at[j][0] + (cx, cy)
        assert math.hypot(got[0] - x, got[1] - y) < tol, f"{key} {j}: {got} vs plan {(x, y)}"


def test_the_4_bar_triangle_is_the_6_bar_simulators():
    """Spot Micro Ver 2: B4 90 at the page's 99 degrees ~ the 6-bar sim's B6 100 from B7 20."""
    lk = linkage.get("fourbar_spot_micro_v2")
    pts = points(lk, TS)
    q = pts["M"] + (pts["K"] - pts["M"]) * 20 / 80
    np.testing.assert_allclose(np.linalg.norm(pts["F"] - q, axis=-1), 100, atol=0.1)


# ---------------------------------------------------------------------------
# The site's Python simulators (diywalkers.com/python-linkage-simulator.html)
# ---------------------------------------------------------------------------
#
# "TrotBot Stationary.py" and "TrotBot, Strider Strandbeest and Klann ver 3.py"
# pick circle intersections by height or side (high / low / left / right)
# rather than by orientation; following them over the whole cycle checks our
# branches everywhere, not only where the plans are drawn.

HIGH, LOW, LEFT = "high", "low", "left"


def _circle(a, b, ra, rb, pick):
    """The site's two-circle intersection, ``a`` radius ``ra``, ``b`` radius ``rb``."""
    d = math.dist(a, b)
    s = (ra**2 - rb**2 + d**2) / (2 * d)
    h = math.sqrt(max(ra**2 - s**2, 0.0))
    mx, my = a[0] + s * (b[0] - a[0]) / d, a[1] + s * (b[1] - a[1]) / d
    p1 = (mx + h * (a[1] - b[1]) / d, my - h * (a[0] - b[0]) / d)
    p2 = (mx - h * (a[1] - b[1]) / d, my + h * (a[0] - b[0]) / d)
    if pick == HIGH:
        return max(p1, p2, key=lambda p: p[1])
    if pick == LOW:
        return min(p1, p2, key=lambda p: p[1])
    return min(p1, p2, key=lambda p: p[0])


def _beyond(a, b, length):
    """``lineExtension``: ``length`` past ``b`` on the ray a -> b."""
    th = math.atan2(b[1] - a[1], b[0] - a[0])
    return b[0] + length * math.cos(th), b[1] + length * math.sin(th)


def _bent(a, b, length, degrees):
    """``lineextendBentDegrees``: from ``a`` away from ``b``, turned by ``degrees``."""
    th = math.atan2(b[1] - a[1], b[0] - a[0]) + math.radians(degrees)
    return a[0] - length * math.cos(th), a[1] - length * math.sin(th)


def _trotbot_site(th, heel):
    j = {"J3": (-7.0, 6.0), "J1": (4 * math.cos(th), 4 * math.sin(th))}
    j["J2"] = _circle(j["J1"], j["J3"], 6, 8, HIGH)
    j["J4"] = _beyond(j["J2"], j["J3"], 2)
    j["J5"] = _circle(j["J1"], j["J4"], 11, 6, LOW)
    j["J6"] = _beyond(j["J4"], j["J5"], 2)
    j["J9"] = _beyond(j["J2"], j["J1"], 1)
    j["J8"] = _circle(j["J1"], j["J2"], 3, 7.55, LEFT)
    j["J7"] = _circle(j["J6"], j["J8"], 8, 9, LOW)
    if heel:
        j["J10"] = _beyond(j["J8"], j["J7"], -2.64)
        j["J11"] = _circle(j["J10"], j["J9"], 2.55, 7.2, LOW)
        j["J12"] = _beyond(j["J10"], j["J11"], 1)
    return j


def _strider_site(th):
    j = {"J2": (-11.0, 8.0), "J6": (11.0, 8.0), "J1": (4 * math.cos(th), 4 * math.sin(th))}
    j["J3"] = _circle(j["J2"], j["J1"], 5, 14, LOW)
    j["J9"] = _bent(j["J1"], j["J3"], 6, 0)
    j["J10"] = _bent(j["J9"], j["J1"], 1, 90)
    j["J7"] = _circle(j["J1"], j["J6"], 14, 5, LOW)
    j["J5"] = _bent(j["J1"], j["J7"], 6, 0)
    j["J11"] = _bent(j["J5"], j["J1"], 1, -90)
    j["J4"] = _circle(j["J3"], j["J11"], 13, 10, LOW)
    j["J8"] = _circle(j["J7"], j["J10"], 13, 10, LOW)
    return j


def _jansen_site(th):
    """Jansen's numbers / 10 (the ver 3 script's Strandbeest)."""
    j = {"A": (-3.8, -0.78), "M": (1.5 * math.cos(th), 1.5 * math.sin(th))}
    j["B"] = _circle(j["A"], j["M"], 4.15, 5, HIGH)
    j["D"] = _circle(j["B"], j["A"], 5.58, 4.01, LEFT)
    j["C"] = _circle(j["M"], j["A"], 6.19, 3.93, LOW)
    j["E"] = _circle(j["D"], j["C"], 3.94, 3.67, LOW)
    j["F"] = _circle(j["C"], j["E"], 4.9, 6.57, LOW)
    return j


SITE = {
    "trotbot": (trotbot.TROTBOT, lambda th: _trotbot_site(th, heel=False), 1.0),
    "trotbot_heel": (trotbot.TROTBOT_HEEL, lambda th: _trotbot_site(th, heel=True), 1.0),
    "strider": (strider.STRIDER, _strider_site, 1.0),
    "jansen": (jansen.JANSEN, _jansen_site, 0.1),
}


@pytest.mark.parametrize("key", sorted(SITE))
def test_follows_the_sites_python_simulator_over_the_whole_cycle(key):
    lk, site, scale = SITE[key]
    ts = TS[::4]
    pts = points(lk, ts)
    for i, th in enumerate(ts):
        for j, want in site(th).items():
            got = pts[j][i] * scale
            assert math.dist(got, want) < 1e-9, f"{key} {j} at {math.degrees(th):.0f} deg"


# ---------------------------------------------------------------------------
# Why some aren't registered
# ---------------------------------------------------------------------------

SWEEPERS = {"trotbot": "b6", "trotbot_heel": "b7", "trotbot_toe": "b6",
            "sixbar": "b4", "sixbar_v1": "b4"}


@pytest.mark.parametrize("key", sorted(SWEEPERS))
def test_unregistered_ones_assemble_but_sweep_a_link_across_the_crank_axis(key):
    """A link hung off the crank rider inside the crank circle crosses O: no layer is free there."""
    lk = DEFINED[key]
    assert key not in linkage.available()
    pts = points(lk, TS)
    assert all(np.isfinite(v).all() for v in pts.values()), f"{key} doesn't assemble"
    feet = {j for _, j in lk.feet}
    assert min(pts[j][:, 1].min() for j in feet) < min(
        v[:, 1].min() for j, v in pts.items() if j not in feet)
    (a, b), = lk.links[SWEEPERS[key]][1]
    ab = pts[b] - pts[a]
    s = np.clip(-(pts[a] * ab).sum(-1) / (ab * ab).sum(-1), 0.0, 1.0)
    reach = np.linalg.norm(pts[a] + ab * s[:, None], axis=-1).min() * float(lk.params["unit"])
    assert reach < 1.0, f"{key} {SWEEPERS[key]} keeps {reach:.1f} mm from O"
