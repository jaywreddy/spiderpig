"""The error budget at a design's output (:mod:`spiderpig.precision`, 2026-10-11): pin play
through the linearised constraints, the inputs' levers, the link tolerances, heat."""

from __future__ import annotations

import math

import pytest
import sympy as sp

from spiderpig import linkage
from spiderpig.linkage import Linkage, Output, P, crank_at, xy
from spiderpig.precision import (
    levers,
    out_of_plane,
    pivot_options,
    play_map,
    precision,
    tolerances,
)

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
    pm = play_map(toy, (), {("O", "b1"): 0.2})                    # the link's clearance
    assert pm.per_joint == {"O": pytest.approx(1.0)}              # the shift itself
    assert set(pm.gain) == {("O", "torso"), ("O", "b1")}          # the frame's, the link's
    assert pm.worst_mm == pytest.approx(0.1)                      # half the clearance
    assert pm.rss_mm == pytest.approx(0.1)


def test_the_palletizers_elbow_crank_magnifies_play():
    from spiderpig.linkage.constraints import layout

    pal = linkage.get("palletizer")
    pm = play_map(pal, (), dict.fromkeys(layout(pal).interfaces, 0.1), n=11)
    # a shift on the 60 mm elbow crank's pins turns the 150 mm forearm: several times over
    assert pm.per_joint["C"] > 150 / 60
    assert pm.per_joint["R"] == pytest.approx(pm.per_joint["C"], rel=1e-6)
    assert sum(pm.at_worst.values()) == pytest.approx(pm.worst_mm)   # the rows add up
    # the wrist's plays only tilt the end plate's 44 mm flange arm
    assert pm.per_joint["W"] < pm.per_joint["C"]
    assert pm.worst_mm >= pm.rss_mm > 0
    assert set(pm.per_joint) == set("OPCERTUVW")                 # F is no pin


def test_the_levers_are_the_arms_lengths():
    pal = linkage.get("palletizer")
    lev = levers(pal, ())
    assert lev["q2"] == pytest.approx(130.0, rel=1e-4)            # the upper arm
    assert lev["q3"] == pytest.approx(150.0, rel=1e-4)            # the forearm (decoupled)


@pytest.mark.slow     # (plans three pivot options: ~9 s)
def test_bearings_shrink_the_palletizers_play():
    from spiderpig.config import BuildConfig

    cfg = BuildConfig(linkage="palletizer", robot=False)
    reps = {label: precision(c, n=9, oop=False) for label, c in pivot_options(cfg)}
    plain, ball, both = reps.values()
    assert plain.repeatability_mm > ball.repeatability_mm > both.repeatability_mm
    assert plain.plays.clearance_mm[("O", "b1")] == pytest.approx(0.35)    # the standoff
    assert both.plays.clearance_mm[("O", "b1")] == pytest.approx(0.043)    # its bearing
    assert plain.plays.clearance_mm[("O", "torso")] == 0.0       # screwed into the plates
    hosts = [b for (j, b), c in plain.plays.clearance_mm.items() if j == "E" and c == 0.0]
    assert len(hosts) == 1                                       # the barrel's bonded link
    r = plain
    assert r.need_deg["q2"] == pytest.approx(math.degrees(0.1 / 130.0), rel=1e-3)
    assert r.thermal_mm == pytest.approx(70e-6 * 10 * r.reach_mm)  # acrylic links
    assert r.accuracy_links_mm > 0
    d = r.as_dict()
    assert d["repeatability_mm"] == pytest.approx(r.repeatability_mm, abs=1e-4)
    assert set(r.inputs) == {"q2", "q3"}                         # its drives, budgeted
    assert r.lines()[0].startswith("precision of palletizer's F")


@pytest.mark.slow     # (plans the palletizer four times: ~20 s)
def test_spiderpig_precision_prints_and_compares(capsys):
    from spiderpig.precision import main

    assert main(["--linkage", "palletizer", "--json", "--planar"]) == 0
    assert '"repeatability_mm"' in capsys.readouterr().out
    assert main(["--linkage", "palletizer", "--compare", "--planar"]) == 0
    out = capsys.readouterr().out
    assert "ball pins, ball pillars" in out


def test_the_gains_predict_a_longer_upper_arm():
    """An independent check of the constraint system: the program's own derivative by the
    upper arm's length equals the gains' prediction for the three holes that length sets
    (the upper arm's at E, the elbow rod's at R, the levelling rod's at T), each shifted
    along its link."""
    import numpy as np

    from spiderpig.linkage.constraints import gains

    pal = linkage.get("palletizer")
    vals = pal.values()
    q = (np.radians([70.0]), np.radians([-10.0]))
    g = gains(pal, vals, q)
    i = list(pal.params).index("upper")
    h = 1e-5
    up, dn = list(vals), list(vals)
    up[i] += h
    dn[i] -= h
    fd = (linkage.LegSolution(1, 0.0, tuple(up), pal.key).at(*q)["F"]
          - linkage.LegSolution(1, 0.0, tuple(dn), pal.key).at(*q)["F"])[0] / (2 * h)
    pts = linkage.LegSolution(1, 0.0, vals, pal.key).at(*q)
    pred = np.zeros(2)
    for joint, body, base in (("E", "b1", "O"), ("R", "b3", "C"), ("T", "b5", "P")):
        u = pts[joint][0] - pts[base][0]
        u /= np.linalg.norm(u)
        pred += g.per[(joint, body)][0] @ (-u)      # the hole further out: the axle follows
    assert pred == pytest.approx(fd, abs=1e-6)


def test_a_walkers_budget_is_in_mm():
    from spiderpig.config import BuildConfig

    r = precision(BuildConfig(linkage="strider", robot=False), n=9)
    # Strider's parameters are in 6.5 mm drawing units: the tolerances per mm, not per unit
    assert max(r.tolerance_mm_per_mm.values()) < 10
    assert r.accuracy_links_mm < 20
    assert r.repeatability_mm > 0                     # its pins and pillars' play
    assert any("crankpins" in n for n in r.notes)     # the crank's play: said, not counted


def test_an_undeclared_input_is_singular():
    from spiderpig.linkage.constraints import gains, sample_inputs

    five = linkage.get("five_bar")                    # t2 drives b3, declared as no drive
    g = gains(five, five.values(), sample_inputs(five, 5))
    assert g.singular == len(g.cond)
    pm = play_map(five, five.values(), {})
    assert pm.worst_mm == math.inf


def test_a_tilt_on_one_pin_rises_the_tip_by_its_lever(toy):
    """The one-link arm tilting on O: the tip rises by the tilt times the link's 100 mm
    (an octagon about the tilt's cone: at most 8 % over), the end float by half itself."""
    t = 0.5
    r = out_of_plane(toy, (), {("O", "b1"): t}, {}, n=9)
    lever = math.radians(t) * 100.0
    assert lever <= r.worst_mm <= lever / math.cos(math.pi / 8) + 1e-9
    assert set(r.by_joint) == {"O"}
    f = out_of_plane(toy, (), {("O", "b1"): t}, {("O", "b1"): 0.2}, n=9)
    assert f.worst_mm == pytest.approx(r.worst_mm + 0.1, abs=1e-9)
    assert out_of_plane(toy, (), {}, {}, n=9).worst_mm == 0.0


def test_the_palletizers_loops_share_out_of_the_plane():
    """Every joint of the palletizer tilting: the result is linear in the tilt, its joints'
    shares add up to it, a joint alone moves nothing (the loops hold it), and making the
    shoulder rigid saves more than a levelling pin (the whole arm hangs on O)."""
    from spiderpig.linkage.constraints import layout

    pal = linkage.get("palletizer")
    lay = layout(pal)
    every = {i: 0.5 for i in lay.interfaces if i[1] != "torso"}
    a = out_of_plane(pal, pal.values(), every, {}, n=5)
    b = out_of_plane(pal, pal.values(), dict.fromkeys(every, 1.0), {}, n=5)
    assert b.worst_mm == pytest.approx(2 * a.worst_mm, rel=1e-6)
    assert sum(sum(v.values()) for v in a.by_joint.values()) == pytest.approx(a.worst_mm,
                                                                               rel=1e-6)
    alone = out_of_plane(pal, pal.values(), {i: 0.5 for i in every if i[0] == "O"}, {}, n=5)
    assert alone.worst_mm == pytest.approx(0.0, abs=1e-9)
    saved = {j: a.worst_mm - out_of_plane(pal, pal.values(),
                                          {i: v for i, v in every.items() if i[0] != j},
                                          {}, n=5).worst_mm
             for j in ("O", "U")}
    assert saved["O"] > 10 * saved["U"] > 0


def test_a_columns_float_is_shared_not_counted_per_link():
    """The palletizer's pins as the ball pivot's wobble notes have them (1.35 deg each, at
    4.25 mm faces, 0.2 mm of float): sharing each column's float among its links (each
    tilt using ``2 r a`` of it) is less than every link at its full tilt and float at once,
    and more than nothing; a column with no float falls back to the links' own tilt."""
    from spiderpig.linkage.constraints import layout

    pal = linkage.get("palletizer")
    lay = layout(pal)
    riders = [i for i in lay.interfaces if i[1] != "torso"]
    tilt = dict.fromkeys(riders, 1.354)
    flt = dict.fromkeys(riders, 0.2)
    each = out_of_plane(pal, pal.values(), tilt, flt, n=5)
    cols = {j: (0.2, {i: (1.487, 4.25) for i in riders if i[0] == j}) for j in lay.axles}
    shared = out_of_plane(pal, pal.values(), tilt, flt, n=5, columns=cols)
    assert 0 < shared.worst_mm < each.worst_mm
    assert sum(sum(v.values()) for v in shared.by_joint.values()) == pytest.approx(
        shared.worst_mm, rel=1e-6)
    none = {j: (0.0, links) for j, (_, links) in cols.items()}
    assert out_of_plane(pal, pal.values(), tilt, flt, n=5, columns=none).worst_mm == \
        pytest.approx(each.worst_mm, rel=1e-9)
