"""Arms (linkage kind ``arm``, 2026-10-11): ranged inputs that drive links, no crank, the
template's one time a closed tour over the inputs' box; the palletizer arm's side
(:mod:`linkages.palletizer`); the planner's bound between the tour's rows."""

from __future__ import annotations

import math

import numpy as np
import pytest
import sympy as sp

from spiderpig import linkage
from spiderpig.linkage import Linkage, Output, P, crank_at, xy
from spiderpig.linkage.engine import TOUR_ROWS, AssemblyError

PAL = linkage.get("palletizer")


def _arm(key: str = "toy_arm", program=None, **kw) -> Linkage:
    """A one-link arm (not registered): b1 O-F turned by ``q`` over 0..90 deg."""
    def one(p):
        return [("O", xy(0, 0)), ("F", crank_at(P("O"), p["L"], "q"))]

    base = {"key": key, "name": "toy", "program": program or one,
            "params": {"L": sp.Integer(100)}, "links": {"b1": (("O", "F"), (("O", "F"),))},
            "frame": ("O",), "crank": (), "inputs": ("q",), "ranges": {"q": (0.0, 90.0)},
            "drives": {"q": ("b1", "O")},
            "output": Output("point", "F", "path", ("O", "F"), body="b1")}
    base.update(kw)
    return Linkage(**base)


def test_the_palletizer_is_an_arm():
    assert PAL.kind == "arm"
    assert PAL.crank == ()
    assert PAL.inputs == ("q2", "q3")
    assert "palletizer" in linkage.available("arm")
    assert "palletizer" not in linkage.available("mechanism")


def test_an_arm_declares_its_inputs():
    _arm()
    with pytest.raises(ValueError, match="has a range and a drive"):
        _arm(ranges={})
    with pytest.raises(ValueError, match="a link turning on a frame joint"):
        _arm(drives={"q": ("b1", "F")})                     # F isn't a frame joint
    with pytest.raises(ValueError, match=r"range 90\.0\.\.0\.0"):
        _arm(ranges={"q": (90.0, 0.0)})
    with pytest.raises(ValueError, match="inputs are the crank angle t"):
        _arm(crank=("O", "F"))                              # a crank: t, not ranges
    with pytest.raises(ValueError, match="one or two inputs"):
        _arm(inputs=("a", "b", "c"), ranges=dict.fromkeys("abc", (0.0, 1.0)),
             drives=dict.fromkeys("abc", ("b1", "O")))


def test_the_tour_is_closed_and_covers_the_box():
    ts = np.linspace(0.0, 2 * math.pi, 20000, endpoint=False)
    q2, q3 = PAL.tour(ts)
    lo2, hi2 = np.radians(PAL.ranges["q2"])
    lo3, hi3 = np.radians(PAL.ranges["q3"])
    assert q2.min() == pytest.approx(lo2)
    assert q2.max() == pytest.approx(hi2, abs=1e-3)
    assert q3.min() == pytest.approx(lo3)
    assert q3.max() == pytest.approx(hi3, abs=1e-3)
    end = PAL.tour(np.array([2 * math.pi - 1e-9]))          # closes on its start
    start = PAL.tour(np.array([0.0]))
    assert end[0][0] == pytest.approx(start[0][0], abs=1e-6)
    assert end[1][0] == pytest.approx(start[1][0], abs=1e-6)
    # constant speed in the unit box: every step the same length
    u2 = (q2 - lo2) / (hi2 - lo2)
    u3 = (q3 - lo3) / (hi3 - lo3)
    steps = np.hypot(np.diff(u2), np.diff(u3))
    assert steps.max() == pytest.approx(steps.mean(), rel=0.05)
    # every row of the second input is visited
    assert len(np.unique(np.round(u3[np.isclose(np.mod(u2, 1.0), 0.5, atol=2e-3)]
                                  * (TOUR_ROWS - 1)))) == TOUR_ROWS


def test_a_one_input_arm_tours_there_and_back():
    arm = _arm()
    q, = arm.tour(np.array([0.0, math.pi / 2, math.pi, 3 * math.pi / 2]))
    assert np.degrees(q) == pytest.approx([0.0, 45.0, 90.0, 45.0])
    assert arm.tour_lateral() == {"O": 0.0, "F": 0.0}


def test_the_lateral_bound_is_a_row_step():
    lat = PAL.tour_lateral()
    step = math.radians((PAL.ranges["q3"][1] - PAL.ranges["q3"][0]) / (TOUR_ROWS - 1))
    assert lat["O"] == lat["E"] == 0.0                      # q3 doesn't move the elbow
    assert lat["W"] == pytest.approx(150 * step, rel=1e-3)  # the forearm swings
    assert lat["C"] == pytest.approx(60 * step, rel=1e-3)   # the 60 mm elbow crank


def test_the_palletizer_assembles_and_keeps_its_tool_level():
    from spiderpig.linkages.palletizer import MIN_PARALLELOGRAM_DEG, parallelogram_margins

    PAL.assert_assembles()
    out = PAL.output_check()
    assert out.broken is None
    assert out.rotation_deg < 1e-9                          # the flange never turns
    assert min(parallelogram_margins().values()) >= MIN_PARALLELOGRAM_DEG - 1e-9
    bad = _arm(key="bent_arm", program=lambda p: [
        ("O", xy(0, 0)), ("F", crank_at(P("O"), p["L"] + 10 * sp.Symbol("q"), "q"))])
    linkage.engine.REGISTRY["bent_arm"] = bad
    try:
        with pytest.raises(AssemblyError, match="not rigid"):
            bad.assert_assembles()
    finally:
        del linkage.engine.REGISTRY["bent_arm"]


def test_an_arms_template_has_no_crank():
    from spiderpig.linkage.assembly import build_module_template

    t = build_module_template("single", linkage="palletizer")
    assert [b.name for b in t.bodies] == [*PAL.links, "torso"]
    assert all("conn" not in a[1] and "coupler" not in a[1] for c in t.connections for a in c)


@pytest.mark.no_fabricate
def test_the_palletizer_plans_and_verifies():
    from spiderpig.config import BuildConfig
    from spiderpig.stack import verify_plan
    from tests import cache

    cfg = BuildConfig(linkage="palletizer", robot=False)
    assert (cfg.module, cfg.crank) == ("single", "")
    tmpl, design = cache.cached_design(cfg)
    assert verify_plan(design.plan, tmpl) == []
    geo = design.plan.topo.geometry
    assert geo.lateral["W"] > 0                              # the bound covers the box
    kinds = {a.name: a.kind for a in design.plan.topo.axes}
    assert kinds["O"] == kinds["P"] == "frame"               # the shoulder: a pillar
    assert {kinds[j] for j in "CERTUVW"} == {"pin"}
    assert design.plan.topo.center is None                   # no crank


@pytest.mark.slow
def test_the_palletizer_side_fabricates_inside_its_claims():
    from spiderpig.config import BuildConfig
    from spiderpig.construction.contract import check_side
    from tests import cache

    cfg = BuildConfig(linkage="palletizer", robot=False)
    tmpl, design = cache.cached_design(cfg)
    for t in (0.0, 2.0, 4.4):
        assert check_side(design, tmpl.freeze_at(t)) == []


def test_an_arm_spec_resolves_checks_and_plans():
    from spiderpig import api

    d = api.resolve({"kind": "arm", "linkage": {"key": "palletizer"}}, store=None)
    assert d.kind == "arm"
    assert d.resolved["legs"]["sides"] == 1
    assert d.resolved["constructions"]["crank"] == ""
    from spiderpig.spec import validate

    errors = validate({"kind": "arm", "linkage": {"key": "palletizer"}, "legs": {"sides": 2}})
    assert [e.path for e in errors] == ["legs.sides"]


def test_a_flattening_parallelogram_is_refused():
    with pytest.raises(AssemblyError, match="nearly locks"):
        PAL.assert_assembles({"ux": 30.0, "uy": 1.0})       # the wrist parallelogram, flat
    from spiderpig.config import BuildConfig, ParamError

    with pytest.raises(ParamError, match="no crank"):
        BuildConfig(linkage="palletizer", robot=False, crank="bolt")


def test_the_bound_covers_poses_off_the_tour():
    """Brute force: two points' sampled-and-bounded distance (the planner's lower bound)
    never exceeds their true distance anywhere in a dense grid over the box."""
    from spiderpig.linkage.assembly import build_module_template
    from spiderpig.stack.topology import topology_from_template

    topo = topology_from_template(build_module_template("single", linkage="palletizer"))
    geo = topo.geometry
    dense = PAL.solve().at(*PAL.box(151))
    names = {a.name: a.joints[0][1] for a in topo.axes}
    for a, b in (("E", "C"), ("W", "R"), ("V", "C"), ("T", "W")):
        bound = geo.dist(("pt", a), ("pt", b))
        true = np.linalg.norm(dense[names[a]] - dense[names[b]], axis=-1).min()
        assert bound <= true + 1e-9, (a, b, bound, true)


@pytest.mark.slow     # (explain plans and fabricates: ~9 s)
def test_explain_and_the_check_report_take_an_arm():
    from spiderpig import api

    d = api.resolve({"kind": "arm", "linkage": {"key": "palletizer"}}, store=None)
    assert not any("second input" in w for w in d.warnings)
    from spiderpig.explain import main as explain

    assert explain(["--linkage", "palletizer"]) == 0


def test_an_arms_toured_geometry_wants_a_lateral_bound_for_a_moving_point():
    from spiderpig.linkage.assembly import build_module_template
    from spiderpig.stack.topology import topology_from_template

    geo = topology_from_template(build_module_template("single", linkage="palletizer")).geometry
    moving = geo.points["W"] + 1.0
    with pytest.raises(ValueError, match="no lateral bound"):
        geo.extend({"new": moving})
    assert geo.extend({"new": moving}, lateral={"new": 1.0}).step["new"] > 1.0
    assert "fixed" in geo.extend({"fixed": np.array([1.0, 2.0])}).points   # a fixed one: fine


@pytest.mark.slow
def test_spiderpig_build_builds_an_arm(tmp_path):
    from spiderpig.build import main as build

    assert build(["--linkage", "palletizer", "--side-only", "--out", str(tmp_path / "b"),
                  "--store", str(tmp_path / "store")]) == 0
    assert (tmp_path / "b" / "laser").is_dir()
