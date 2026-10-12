"""The error budget at a design's output (:mod:`spiderpig.precision`, 2026-10-11): pin play
through the linearised constraints, the inputs' levers, the link tolerances, heat."""

from __future__ import annotations

import math

import pytest
import sympy as sp

from spiderpig import linkage
from spiderpig.linkage import Linkage, Output, P, crank_at, xy
from spiderpig.precision import levers, pivot_options, play_map, precision, tolerances

pytestmark = pytest.mark.no_fabricate


@pytest.fixture(scope="module")
def toy():
    """A one-link arm, registered for the module: b1 O-F, 100 mm, turned by ``q``."""
    def one(p):
        return [("O", xy(0, 0)), ("F", crank_at(P("O"), p["L"], "q"))]

    lk = Linkage(key="toy_precision_arm", name="toy", program=one,
                 params={"L": sp.Integer(100)}, links={"b1": (("O", "F"), (("O", "F"),))},
                 frame=("O",), crank=(), inputs=("q",), ranges={"q": (0.0, 90.0)},
                 drives={"q": ("b1", "O")},
                 output=Output("point", "F", "path", ("O", "F"), body="b1"))
    linkage.engine.REGISTRY[lk.key] = lk
    yield lk
    del linkage.engine.REGISTRY[lk.key]


def test_one_links_numbers_are_its_length(toy):
    assert levers(toy, ()) == {"q": pytest.approx(100.0)}         # mm per radian
    assert tolerances(toy, ()) == {"L": pytest.approx(1.0)}       # mm per mm
    pm = play_map(toy, (), {"O": 0.2})
    assert pm.per_joint == {"O": pytest.approx(1.0)}              # the shift itself
    assert pm.worst_mm == pytest.approx(0.1)                      # half the clearance
    assert pm.rss_mm == pytest.approx(0.1)


def test_the_palletizers_elbow_crank_magnifies_play():
    pal = linkage.get("palletizer")
    pm = play_map(pal, (), dict.fromkeys("OPCERTUVW", 0.1), n=11)
    # a shift on the 40 mm elbow crank's pins turns the 150 mm forearm: several times over
    assert pm.per_joint["C"] > 150 / 40
    assert pm.per_joint["R"] == pytest.approx(pm.per_joint["C"], rel=1e-6)
    # the wrist's plays only tilt the end plate's 44 mm flange arm
    assert pm.per_joint["W"] < pm.per_joint["C"]
    assert pm.worst_mm >= pm.rss_mm > 0
    assert set(pm.per_joint) == set("OPCERTUVW")                 # F is no pin


def test_the_levers_are_the_arms_lengths():
    pal = linkage.get("palletizer")
    lev = levers(pal, ())
    assert lev["q2"] == pytest.approx(130.0, rel=1e-4)            # the upper arm
    assert lev["q3"] == pytest.approx(150.0, rel=1e-4)            # the forearm (decoupled)


def test_bearings_shrink_the_palletizers_play():
    from spiderpig.config import BuildConfig

    cfg = BuildConfig(linkage="palletizer", robot=False)
    reps = {label: precision(c, n=9) for label, c in pivot_options(cfg)}
    plain, ball, both = reps.values()
    assert plain.repeatability_mm > ball.repeatability_mm > both.repeatability_mm
    assert plain.plays.play_mm["O"] == pytest.approx(0.35)       # the standoff pillar
    assert both.plays.play_mm["O"] == pytest.approx(0.043)       # its bearing
    r = plain
    assert r.need_deg["q2"] == pytest.approx(math.degrees(0.1 / 130.0), rel=1e-3)
    assert r.thermal_mm == pytest.approx(70e-6 * 10 * r.reach_mm)  # acrylic links
    assert r.accuracy_links_mm > 0
    d = r.as_dict()
    assert d["repeatability_mm"] == pytest.approx(r.repeatability_mm, abs=1e-4)
    assert any("no transmission" in n for n in r.notes)
    assert r.lines()[0].startswith("precision of palletizer's F")


def test_spiderpig_precision_prints_and_compares(capsys):
    from spiderpig.precision import main

    assert main(["--linkage", "palletizer", "--json"]) == 0
    assert '"repeatability_mm"' in capsys.readouterr().out
    assert main(["--linkage", "palletizer", "--compare"]) == 0
    out = capsys.readouterr().out
    assert "ball pins, ball pillars" in out
