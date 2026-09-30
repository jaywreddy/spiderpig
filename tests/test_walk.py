"""The walking model (``walk.py``), ``/api/walk`` and the parameterized ``/api/glb``.

The support and motion rules are checked on synthetic feet first, then
against a literal per-sample transcription of the contract (SPEC: every
triangle, the most level face containing the centre of mass's projection)
on the default quad, and against the viewer's reference numbers for it.
"""

from __future__ import annotations

import argparse
import asyncio
import itertools
import math
import sys
import time
from pathlib import Path
from urllib.parse import urlencode

import numpy as np
import pytest

import linkage
import walk
from config import BuildConfig, ParamError, config_from_args, parse_phases, parse_proportion

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))

# Phases for the default quad that scripts/tune_gait.py finds (both plan in
# 12 layers like the default): the two cranks half a turn apart, each crank's
# legs 5-6 degrees apart or, the other way round, each crank's legs half a
# turn apart and the cranks 5 degrees apart.
TUNED_QUAD = (0.0, 175.0, 180.0, 355.0)
TUNED_QUAD_B = (0.0, 354.0, 180.0, 174.0)


def _square(y=-100.0, hx=50.0, hz=40.0):
    return np.array([[-hx, y, -hz], [-hx, y, hz], [hx, y, -hz], [hx, y, hz]])


def _cfg(module: str, phases_deg=None, proportions=None, **kw) -> BuildConfig:
    """A robot's config from the phases in degrees and a proportions dict."""
    phases = None if phases_deg is None else tuple(math.radians(p) for p in phases_deg)
    return BuildConfig(module=module, phases=phases, proportions=tuple((proportions or {}).items()),
                       **kw)


@pytest.fixture(scope="module")
def quad():
    return walk.walker(_cfg("quad"))


# ---------------------------------------------------------------------------
# Support on synthetic feet
# ---------------------------------------------------------------------------


def test_level_support():
    s = walk.support(_square(), [0.0, 0.0, 0.0])
    assert not s.degenerate
    assert not s.tipping
    assert s.pitch_deg == pytest.approx(0.0, abs=1e-12)
    assert s.roll_deg == pytest.approx(0.0, abs=1e-12)
    assert s.height == pytest.approx(100.0)
    np.testing.assert_allclose(s.normal, [0, 1, 0], atol=1e-12)
    assert s.contacts.all()
    assert s.margin == pytest.approx(40.0)          # nearest edges: z = +-40


@pytest.mark.parametrize(("axis", "deg"), [(0, 10.0), (0, -7.0), (2, 12.0)])
def test_tilted_support(axis, deg):
    """Feet on ``y = -100 + tan(a) * (x or z)``: pitch (or roll) is ``-a``."""
    feet = _square()
    feet[:, 1] += math.tan(math.radians(deg)) * feet[:, axis]
    s = walk.support(feet, [0.0, 0.0, 0.0])
    got = s.pitch_deg if axis == 0 else s.roll_deg
    other = s.roll_deg if axis == 0 else s.pitch_deg
    assert got == pytest.approx(-deg)
    assert other == pytest.approx(0.0, abs=1e-9)
    assert s.height == pytest.approx(100.0 * math.cos(math.radians(deg)))
    assert s.contacts.all()
    assert not s.tipping


def test_most_level_of_the_faces_containing_the_com():
    """A lower hull of two faces (ABC level, BCD tilted); a centre of mass high
    above their common edge projects into both along their own normals: the
    level one carries the robot. Moved over BCD only, the tilted one does."""
    feet = np.array([[-50, -100, -40], [-50, -100, 40], [50, -100, -40], [50, -90, 40.0]])
    both = [-5.0, 0.0, -5.0]
    faces = _containing_faces(feet, np.array(both))
    assert len(faces) == 2
    s = walk.support(feet, both)
    assert sorted(s.face) == [0, 1, 2]
    assert s.pitch_deg == pytest.approx(0.0, abs=1e-12)
    assert not s.tipping
    s = walk.support(feet, [30.0, 0.0, 25.0])
    assert sorted(s.face) == [1, 2, 3]
    assert s.normal[1] < 0.99
    assert not s.tipping


def test_tipping_rests_on_the_nearest_face():
    s = walk.support(_square(), [200.0, 0.0, 0.0])
    assert s.tipping
    assert not s.degenerate
    np.testing.assert_allclose(s.normal, [0, 1, 0], atol=1e-12)
    assert s.margin == pytest.approx(-150.0)


@pytest.mark.parametrize("feet", [
    [[0, -100, 0], [30, -95, 10]],                                   # two feet
    [[0, -100, 0], [10, -98, 0], [20, -96, 0]],                      # collinear
    [[0, -100, 0], [0.5, -100, 0], [0, -100, 0.5]],                  # area < 1 mm^2
])
def test_degenerate_support(feet):
    feet = np.asarray(feet, dtype=float)
    s = walk.support(feet, [5.0, 0.0, 0.0])
    assert s.degenerate
    assert not s.tipping
    np.testing.assert_allclose(s.normal, [0, 1, 0])
    assert s.height == pytest.approx(100.0)                       # through the lowest foot
    assert s.contacts.tolist() == [bool(y <= -99.5) for y in feet[:, 1]]
    assert np.isfinite(s.margin)
    assert s.margin <= 0


def test_support_is_vectorized():
    rng = np.random.default_rng(3)
    feet = _square()[None] + rng.normal(0, 5, (20, 4, 3))
    com = [1.0, 0.0, -2.0]
    batch = walk.support(feet, com)
    for i in range(len(feet)):
        one = walk.support(feet[i], com)
        for name in ("normal", "height", "pitch_deg", "roll_deg", "contacts", "margin",
                     "tipping", "degenerate"):
            np.testing.assert_allclose(getattr(batch, name)[i], getattr(one, name),
                                       err_msg=name)


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------


def test_rigid_motion_has_no_slip():
    """Feet planted while the body translates and turns: recovered exactly, slip 0."""
    feet = _square() + [[3, 0, 1], [-2, 0, 5], [7, 0, -3], [1, 0, 2]]
    for V, w in (((25.0, 0.0), 0.0), ((3.0, -2.0), 0.5)):
        rates = np.zeros_like(feet)
        rates[:, 0] = -(V[0] + w * feet[:, 2])
        rates[:, 2] = -(V[1] - w * feet[:, 0])
        got_v, got_w, slip = walk.body_velocity(feet, rates, np.ones(4, bool))
        np.testing.assert_allclose(got_v, V, atol=1e-9)
        assert got_w == pytest.approx(w, abs=1e-12)
        assert slip == pytest.approx(0.0, abs=1e-9)


def test_one_contact_does_not_turn():
    feet = _square()
    rates = np.tile([4.0, 0.0, -1.0], (4, 1))
    V, w, slip = walk.body_velocity(feet, rates, np.array([True, False, False, False]))
    np.testing.assert_allclose(V, [-4.0, 1.0])
    assert w == 0.0
    assert slip == 0.0


def test_slip_is_the_rms_residual():
    """Two contacts at one point pulling apart: U = 0, w = 0, residuals +-1 in x."""
    feet = np.zeros((2, 3))
    rates = np.array([[1.0, 0, 0], [-1.0, 0, 0]])
    V, w, slip = walk.body_velocity(feet, rates, np.ones(2, bool))
    np.testing.assert_allclose(V, [0, 0], atol=1e-12)
    assert w == 0.0
    assert slip == pytest.approx(math.sqrt(2 / 4))


def test_simulate_turns_with_one_side_stopped(quad):
    """Only the left side walking: the robot turns (and still moves forward)."""
    n = walk.N_THETA
    trace = walk.simulate(quad, np.full(n, 1.0), np.zeros(n), 2 * math.pi / n)
    assert abs(trace.yaw[-1]) > math.radians(5)
    assert np.isfinite(trace.x).all()
    assert np.isfinite(trace.z).all()
    straight = walk.simulate(quad, np.full(n, 1.0), np.full(n, 1.0), 2 * math.pi / n)
    assert straight.yaw[-1] == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# The default quad: the literal contract, the viewer's reference numbers
# ---------------------------------------------------------------------------


def _triangle_contains(q, a, b, c, n) -> bool:
    signs = [np.dot(np.cross(p1 - p0, q - p0), n) for p0, p1 in ((a, b), (b, c), (c, a))]
    return all(s >= -1e-9 for s in signs) or all(s <= 1e-9 for s in signs)


def _containing_faces(feet, com):
    """Every valid lower-hull face (SPEC support 1-2) containing c's projection: ``(n.y, face)``."""
    out = []
    for i, j, k in itertools.combinations(range(len(feet)), 3):
        cr = np.cross(feet[j] - feet[i], feet[k] - feet[i])
        if np.linalg.norm(cr) / 2 <= walk.AREA_MIN:
            continue
        n = cr / np.linalg.norm(cr)
        n = -n if n[1] < 0 else n
        if n[1] <= 0 or ((feet - feet[i]) @ n).min() < -walk.ON_PLANE:
            continue
        q = com - np.dot(com - feet[i], n) * n
        if _triangle_contains(q, feet[i], feet[j], feet[k], n):
            out.append((n[1], (i, j, k)))
    return out


def test_quad_support_follows_the_contract(quad):
    """At every crank angle: the chosen face is the most level containing one."""
    P, _ = quad.feet_at(walk.theta_grid(), walk.theta_grid())
    sup = walk.support(P, quad.com)
    several = 0
    for i in range(0, walk.N_THETA, 3):
        faces = _containing_faces(P[i], quad.com)
        assert faces, i
        several += len({round(ny, 9) for ny, _ in faces}) > 1
        assert sup.normal[i, 1] == pytest.approx(max(ny for ny, _ in faces), abs=1e-12), i
        assert not sup.tipping[i]
    assert several > 0          # the rule matters (e.g. at 135 deg)


@pytest.mark.parametrize("com", ["nominal", "pivots"])
def test_quad_reference(quad, com):
    """The viewer's numbers for the default quad (centre of mass: the frame
    pivots' centroid there; the nominal mass model here gives the same)."""
    if com == "pivots":
        piv = np.array([leg.joints[j][0] for leg in quad.legs for j in ("O", "A", "B")])
        model = walk.walker(quad.config, com=[piv[:, 0].mean(), piv[:, 1].mean(), 0.0],
                            mass_g=520.0)
    else:
        model = quad
    th = math.radians(135)
    P, _ = model.feet_at(th, th)
    s = walk.support(P[0], model.com)
    assert len(_containing_faces(P[0], model.com)) > 1
    assert s.pitch_deg == pytest.approx(0.0, abs=0.01)
    assert s.contacts.tolist() == [True, False, False, True] * 2   # legs 0 and 3, both sides
    m = walk.straight_walk_metrics(model)
    assert max(abs(v) for v in m["pitch_deg"]) == pytest.approx(8.4, abs=0.1)
    assert m["bob_mm"] == pytest.approx(24.0, abs=0.5)
    assert m["stride_mm"] == pytest.approx(102.0, abs=1.0)
    assert m["direction"] == "+x"
    assert m["min_margin_mm"] == pytest.approx(50.0, abs=0.5)
    assert m["tipping_fraction"] == 0.0
    assert m["degenerate_fraction"] == 0.0
    assert m["roll_deg"] == pytest.approx([0.0, 0.0], abs=1e-9)     # left/right symmetric
    assert m["speed_mm_s"] == pytest.approx(m["stride_mm"] * 52.0 / 60.0)


@pytest.mark.parametrize("module", list(linkage.MODULE_LEGS))
def test_straight_walk_metrics_are_finite(module):
    m = walk.straight_walk_metrics(walk.walker(_cfg(module)))
    for key, value in m.items():
        if key != "direction":
            assert np.isfinite(value).all(), key
    assert len(m["duty"]) == 2 * len(linkage.MODULE_LEGS[module])
    assert m["slip_rms"] == m["slip_rms_mm_per_rad"]
    assert m["slip_rms_mm_per_rev"] == pytest.approx(2 * math.pi * m["slip_rms"])
    if module == "single":          # two feet: never a support triangle
        assert m["degenerate_fraction"] == 1.0
    elif module == "quad":
        assert m["stride_mm"] > 50.0
        assert m["degenerate_fraction"] == 0.0
    else:
        # Two legs per side: the four feet are two mirror-image pairs, always
        # coplanar, so all four stay on the ground (the robot rocks) and the
        # least-squares body velocity, minus their mean velocity round closed
        # paths, integrates to nothing: this model has them walk on the spot.
        assert m["mean_contacts"] == 4.0
        assert m["stride_mm"] == pytest.approx(0.0, abs=1e-6)


def test_better_phases_lower_the_objective(quad):
    ref = walk.straight_walk_metrics(quad)
    base = walk.objective(ref, ref["stride_mm"])
    for phases in (TUNED_QUAD, TUNED_QUAD_B):
        m = walk.straight_walk_metrics(walk.walker(_cfg("quad", phases)))
        assert walk.objective(m, ref["stride_mm"]) < 0.5 * base, phases
        assert m["bob_mm"] < ref["bob_mm"]
        assert m["slip_rms"] < ref["slip_rms"]
        assert m["stride_mm"] > ref["stride_mm"]


@pytest.mark.slow
def test_tuner_improves_the_quad():
    import tune_gait

    tuner, default, best = tune_gait.tune("quad", grid=90.0, top=1)
    assert best.score < 0.5 * default.score
    assert best.candidate.phases[0] == default.candidate.phases[0]      # leg 0 stays
    assert tuner.feasible(best.candidate.phases)
    use = tune_gait.flags("quad", best.candidate)
    assert use["main"].startswith("uv run python cli.py build --module quad --phases ")
    assert use["bake"].startswith("uv run python cli.py bake --module quad --phases ")
    assert use["query"].startswith("?module=quad&phases=")


@pytest.mark.slow
def test_tuner_on_other_linkages():
    """The tuner tunes any linkage: it leaves out the parameters that only scale it, and
    its flags name the linkage."""
    import tune_gait

    assert [tune_gait.scale_params(linkage.get(k)) for k in ("klann", "jansen", "strider")] == \
        [("OA",), ("unit",), ("unit",)]
    tuner, default, best = tune_gait.tune("double", grid=90.0, coarse=60, top=1,
                                          linkage_key="jansen")
    assert tuner.linkage.key == "jansen"
    assert default.metrics is not None
    assert best.score <= default.score
    use = tune_gait.flags("double", best.candidate, "jansen")
    assert use["main"].endswith("--linkage jansen")
    assert use["query"].endswith("&linkage=jansen")


def test_tuner_keeps_crankpins_apart():
    """Coincident crankpins on different cranks can't be planned (quad 0,180,180,0);
    the double's pair shares one by design."""
    import tune_gait

    quad = tune_gait.Tuner("quad", stride_ref=100.0)
    assert not quad.feasible((0.0, 180.0, 180.0, 0.0))
    assert quad.feasible(TUNED_QUAD)
    assert quad.score(tune_gait.Candidate((0.0, 180.0, 182.0, 0.0))).score == math.inf
    assert tune_gait.Tuner("double", stride_ref=0.0).feasible((0.0, 0.0))


# ---------------------------------------------------------------------------
# Design parameters, kinematics, lateral offsets, mass
# ---------------------------------------------------------------------------


def test_normalized_parameters():
    """A design has one config however it was asked for (the module's own phases and the
    linkage's default proportions are dropped), and a bad one fails where it is named."""
    assert _cfg("quad", [0, 180, 90, 270]).phases is None
    assert _cfg("quad", [360, -180, 90, 270]).phases is None
    cfg = _cfg("quad", [0, 175, 180, 355], {"DF": 2.577, "OB": 1.2})
    assert cfg.phases == pytest.approx(tuple(math.radians(p) for p in TUNED_QUAD))
    assert cfg.proportions == (("OB", 1.2),)
    assert cfg.design_json()["proportions"]["OB"] == 1.2
    assert cfg.legs == tuple(zip((1, -1, 1, -1), cfg.phases, strict=True))
    assert not cfg.is_default
    assert cfg.key.startswith("klann_quad_robot_")
    assert _cfg("quad").is_default
    assert _cfg("quad").key == "klann_quad_robot"
    assert _cfg("quad", robot=False).key == "klann_quad_side"
    for bad in (dict(module="octo"), dict(phases_deg=[0, 90]), dict(proportions={"XX": 1}),
                dict(proportions={"OB": -1}), dict(proportions={"DF": float("nan")}),
                dict(servo="none"), dict(linkage="hoecken")):            # a mechanism: no feet
        with pytest.raises(ParamError):
            _cfg(**{"module": "quad", **bad})
    assert parse_phases("0, 90") == (0.0, math.pi / 2)
    assert parse_proportion("DF=2.6") == ("DF", 2.6)
    # the CLIs' parsed arguments, and a field a tool fixes itself (the audit's module per
    # run, explain's side) over the parsed one
    args = argparse.Namespace(linkage="jansen", module="quad", phases=None,
                              proportion=[("m", 14.0)], servo="sts3215")
    assert config_from_args(args) == BuildConfig(linkage="jansen", module="quad",
                                                 proportions=(("m", 14.0),))
    assert config_from_args(args, module="single", robot=False) == BuildConfig(
        linkage="jansen", module="single", robot=False, proportions=(("m", 14.0),))
    for text in ("0,,90", "a,b"):
        with pytest.raises(ParamError):
            parse_phases(text)
    with pytest.raises(ParamError):
        parse_proportion("DF")


def test_parameters_are_the_linkages():
    """Each linkage validates its own parameters: lengths > 0, angles any finite number."""
    unit = float(linkage.get("jansen").params["unit"])
    cfg = _cfg("double", [0, 90], {"m": 14.0, "unit": unit}, linkage="jansen")
    assert (cfg.linkage, cfg.proportions) == ("jansen", (("m", 14.0),))   # the default dropped
    assert cfg.design_json() == {
        "linkage": "jansen", "module": "double", "phases_deg": [0.0, 90.0],
        "proportions": {k: (14.0 if k == "m" else float(v))
                        for k, v in linkage.get("jansen").params.items()}}
    assert _cfg("quad", proportions={"angA": -30.0}).proportions == (("angA", -30.0),)
    for bad in (dict(linkage="octopus"), dict(proportions={"DF": 2.0}),     # Klann's, not Jansen's
                dict(proportions={"m": 0.0}), dict(proportions={"m": float("inf")})):
        with pytest.raises(ParamError):
            _cfg(**{"module": "double", "linkage": "jansen", **bad})
    # the default design's phases, whichever way they were given, are None
    assert _cfg("double", [0, 180], linkage="strider").phases is None


@pytest.mark.parametrize(("key", "module", "phases", "props"), [
    ("klann", "quad", None, None),
    ("klann", "quad", (0, 175, 180, 355), {"DF": 2.4}),
    ("jansen", "double", (0, 90), {"m": 14.0}),
    ("strider", "single", None, {"tail": 5.5}),
])
def test_template_joints_are_the_program(key, module, phases, props):
    """The side template's link joints (what gets built) are the legs' program points,
    and the feet are the linkage's feet of every leg."""
    from fabricate import template_for

    cfg = _cfg(module, phases, props, linkage=key)
    tmpl = template_for(cfg)
    jw = tmpl.sample(walk.theta_grid()).joint_world
    legs = walk.side_legs(cfg)
    feet = walk.side_feet(cfg)
    assert [b for _, b, _ in feet] == [b for b, _ in linkage.feet_of(tmpl)]
    lk = linkage.get(key)
    for leg in legs:
        sfx = "" if len(legs) == 1 else f"_leg{leg.leg}"
        for body, (joints, _) in lk.links.items():
            for j in joints:
                np.testing.assert_allclose(jw[f"{body}{sfx}"][j][:, :2], leg.joints[j], atol=1e-9)
    model = walk.walker(cfg)
    assert len(model.feet) == 2 * len(legs) * len(lk.feet)
    for f, (k, body, joint) in zip(model.feet, feet, strict=False):
        assert f.body == f"L.{body}"
        np.testing.assert_allclose(f.xy, jw[body][joint][:, :2], atol=1e-9)
        assert f.leg == k


def test_phase_is_a_time_shift():
    base = walk.walker(_cfg("decker"))
    moved = walk.walker(_cfg("decker", [0, 180]))
    np.testing.assert_allclose(moved.feet[1].xy, np.roll(base.feet[1].xy, -90, axis=0),
                               atol=1e-9)


@pytest.mark.parametrize(("key", "module"), [("klann", "decker"), ("klann", "quad"),
                                             ("jansen", "double"), ("strider", "single")])
def test_nominal_foot_z_is_the_default_plan(key, module):
    """Klann's from its table, another linkage's from planning its default design once."""
    cfg = _cfg(module, linkage=key)
    assert walk.foot_z_nominal(cfg) == pytest.approx(walk.foot_z_planned(cfg), abs=1e-9)
    assert all(z < 0 for z in walk.foot_z_nominal(cfg))                  # left side: -z
    name, default = next(iter(linkage.get(key).params.items()))
    tuned = _cfg(module, proportions={name: 1.1 * float(default)}, linkage=key)
    assert walk.foot_z_nominal(tuned) == walk.foot_z_nominal(cfg)       # no new plan


def test_foot_z_without_a_layer_plan_is_a_guess(monkeypatch):
    """A default design the planner can't lay out (cached as such): feet a layer apart."""

    def no_plan(*_a, **_k):
        raise ValueError("no layer plan found")

    monkeypatch.setattr(walk, "design_side", no_plan)
    cfg = _cfg("decker", linkage="jansen", thickness=3.1)
    z = walk.foot_z_nominal(cfg)
    assert len(z) == 2
    assert z[1] - z[0] == pytest.approx(3.1)
    assert all(v < 0 for v in z)


@pytest.mark.parametrize(("key", "props", "joint"), [
    ("klann", {"MC": 0.3}, "C"),
    ("jansen", {"j": 20.0}, "E"),
    ("strider", {"rocker": 0.5}, "J3"),
])
def test_invalid_linkage_is_explained(key, props, joint):
    """The first point that can't be placed, and where (the linkage's assembly check)."""
    cfg = _cfg("single", proportions=props, linkage=key)
    with pytest.raises(walk.LinkageError, match=f"joint {joint} can't be placed"):
        walk.side_legs(cfg)
    payload = walk.api_payload(cfg)
    assert payload["valid"] is False
    assert f"{joint} can't be placed" in payload["error"]
    assert "crank angle" in payload["error"]


@pytest.mark.parametrize(("key", "module", "feet"), [
    ("jansen", "double", ["L.b6_leg0", "L.b6_leg1", "R.b6_leg0", "R.b6_leg1"]),
    ("strider", "single", ["L.b3", "L.b7", "R.b3", "R.b7"]),        # a coupled pair: two feet
])
def test_other_linkages_walk_in_the_model(key, module, feet):
    model = walk.walker(_cfg(module, linkage=key))
    assert [f.body for f in model.feet] == feet
    assert [f.leg for f in model.feet] == [0, 1, 0, 1] if module == "double" else [0] * 4
    assert model.z_nominal
    m = walk.straight_walk_metrics(model)
    for k, v in m.items():
        if k != "direction":
            assert np.isfinite(v).all(), k
    assert len(m["duty"]) == 4
    assert model.mass_g > 150.0
    assert model.com[2] == 0.0


def test_nominal_mass_and_servo():
    info = walk.servo_info("sts3215")
    assert info == {"key": "sts3215", "rpm_max": 52.0, "mass_g": 55.0}
    cfg = _cfg("quad")
    com, mass = walk.nominal_mass(cfg, walk.side_legs(cfg))
    assert mass == pytest.approx(460.8, rel=0.01)          # the fabricated quad robot
    assert com[2] == 0.0
    assert abs(com[0]) < 1e-9
    assert com[1] == pytest.approx(2.1, abs=1.0)


# ---------------------------------------------------------------------------
# /api/walk and /api/glb (in-process; no server, no lifespan bake)
# ---------------------------------------------------------------------------


class _Response:
    def __init__(self, status: int, headers: dict, body: bytes) -> None:
        self.status_code, self.headers, self.content = status, headers, body

    def json(self):
        import json

        return json.loads(self.content)


class _AsgiClient:
    """A minimal ``TestClient.get`` stand-in for when ``httpx`` isn't installed."""

    def __init__(self, app) -> None:
        self.app = app

    def get(self, path: str, params=None) -> _Response:
        return asyncio.run(self._get(path, urlencode(params or {}, doseq=True)))

    async def _get(self, path: str, query: str) -> _Response:
        scope = {"type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1",
                 "method": "GET", "scheme": "http", "path": path, "raw_path": path.encode(),
                 "query_string": query.encode(), "root_path": "",
                 "headers": [(b"host", b"testserver")], "client": ("testclient", 50000),
                 "server": ("testserver", 80)}
        sent = False
        messages = []

        async def receive():
            nonlocal sent
            if not sent:
                sent = True
                return {"type": "http.request", "body": b"", "more_body": False}
            await asyncio.sleep(3600)

        async def send(message):
            messages.append(message)

        await self.app(scope, receive, send)
        start = next(m for m in messages if m["type"] == "http.response.start")
        body = b"".join(m.get("body", b"") for m in messages if m["type"] == "http.response.body")
        headers = {k.decode(): v.decode() for k, v in start.get("headers", [])}
        return _Response(start["status"], headers, body)


@pytest.fixture(scope="module")
def server_app():
    from server import app as server_app

    return server_app


@pytest.fixture(scope="module")
def client(server_app):
    try:
        import httpx  # noqa: F401
        from fastapi.testclient import TestClient
    except ImportError:
        return _AsgiClient(server_app.app)
    return TestClient(server_app.app)         # not entered: no lifespan (no default bake)


def test_api_walk_quad(client, server_app):
    client.get("/api/walk", params={"module": "quad"})   # warm: the program compiles, the
    server_app._walk_json.cache_clear()                  # default design plans, once
    t0 = time.perf_counter()
    r = client.get("/api/walk", params={"module": "quad", "phases": "0,180,90,270",
                                        "p.OB": "1.121"})
    elapsed = time.perf_counter() - t0
    assert r.status_code == 200
    w = r.json()
    assert w["valid"] is True
    assert w["error"] is None
    assert w["linkage"] == "klann"
    assert w["module"] == "quad"
    assert w["phases_deg"] == [0, 180, 90, 270]
    klann = linkage.get("klann")
    assert w["proportions"] == {k: float(v) for k, v in klann.params.items()}
    assert w["theta_samples"] == 360
    assert w["z_nominal"] is True
    assert [f["body"] for f in w["feet"]] == [f"{s}.b4_leg{k}" for s in "LR" for k in range(4)]
    for f in w["feet"]:
        assert len(f["xy"]) == 360
        assert (f["z"] < 0) == (f["side"] == "L")
    assert len(w["legs"]) == 4
    for leg in w["legs"]:
        assert set(leg["joints"]) == set(klann.points)
        assert all(len(v) == 360 for v in leg["joints"].values())
    assert [leg["orientation"] for leg in w["legs"]] == [1, -1, 1, -1]
    assert w["links"] == [["O", "M"], ["M", "D"], ["B", "E"], ["A", "C"], ["E", "F"],
                          ["O", "A"], ["O", "B"]]
    assert w["side_z"]["L"] == pytest.approx(-w["side_z"]["R"])
    assert w["side_z"]["L"] < 0
    assert len(w["com"]) == 3
    assert w["servo"] == {"key": "sts3215", "rpm_max": 52.0}
    assert w["metrics"]["stride_mm"] == pytest.approx(102.4, abs=0.5)
    assert elapsed < 1.0


def test_api_walk_parameters(client):
    w = client.get("/api/walk", params={"module": "decker", "phases": "0,180",
                                        "p.DF": "2.4"}).json()
    assert w["valid"]
    assert w["phases_deg"] == [0, 180]
    assert w["proportions"]["DF"] == 2.4
    assert len(w["feet"]) == 4


@pytest.mark.parametrize(("key", "module", "name", "value", "n_feet"), [
    ("jansen", "double", "m", 14.0, 4),
    ("strider", "single", "tail", 5.5, 4),
])
def test_api_walk_other_linkages(client, key, module, name, value, n_feet):
    lk = linkage.get(key)
    w = client.get("/api/walk", params={"linkage": key, "module": module,
                                        f"p.{name}": str(value)}).json()
    assert w["valid"], w["error"]
    assert (w["linkage"], w["module"]) == (key, module)
    assert w["proportions"][name] == value
    assert len(w["feet"]) == n_feet
    assert {f["body"].split(".")[1].split("_")[0] for f in w["feet"]} == {b for b, _ in lk.feet}
    assert w["links"] == [list(link) for link in walk.links_of(lk)]
    for leg in w["legs"]:
        assert set(leg["joints"]) == set(lk.points)
    assert np.isfinite(w["metrics"]["bob_mm"])


def test_api_linkages(client):
    body = client.get("/api/linkages").json()
    assert body["default"] == "klann"
    by_key = {lk["key"]: lk for lk in body["linkages"]}
    assert list(by_key) == linkage.available()
    klann = by_key["klann"]
    assert klann["name"]
    assert klann["family"] == "klann"
    assert klann["source"].startswith("http")
    assert klann["params"][0] == {"name": "OA", "default": 60.0, "angle": False}
    assert {p["name"] for p in klann["params"] if p["angle"]} == {"angA", "angB"}
    assert klann["modules"] == {"single": 1, "double": 2, "decker": 2, "quad": 4}
    assert klann["feet"] == 1
    assert by_key["strider"]["feet"] == 2
    assert by_key["strider"]["modules"]["double"] == 2
    assert by_key["jansen"]["labels"]["b6"] == "foot triangle g-h-i"
    assert [p["name"] for p in by_key["jansen"]["params"]] == list(linkage.get("jansen").params)
    assert (klann["kind"], klann["output"]) == ("walker", None)
    rocker = by_key["crank_rocker"]
    assert (rocker["kind"], rocker["feet"]) == ("mechanism", 0)
    assert rocker["output"]["frame"] == ["G", "E"]


@pytest.mark.parametrize("params", [
    {"linkage": "octopus"},
    {"linkage": "hoecken"},                        # a mechanism doesn't walk
    {"linkage": "jansen", "p.DF": "2.5"},          # Klann's proportion, not Jansen's
    {"linkage": "jansen", "p.m": "-3"},
    {"module": "octo"},
    {"module": "quad", "phases": "0,90"},
    {"module": "quad", "phases": "0,a,90,270"},
    {"module": "quad", "p.XX": "1"},
    {"module": "quad", "p.OB": "-1"},
    {"module": "quad", "p.OB": "abc"},
    {"module": "quad", "p.OB": "inf"},
])
def test_api_walk_rejects_bad_parameters(client, params):
    r = client.get("/api/walk", params=params)
    assert r.status_code == 422
    assert isinstance(r.json()["detail"], str)
    assert r.json()["detail"]


def test_api_walk_invalid_linkage(client):
    r = client.get("/api/walk", params={"module": "quad", "p.MC": "0.3"})
    assert r.status_code == 200
    w = r.json()
    assert w["valid"] is False
    assert "C can't be placed" in w["error"]


class _Calls(list):
    """Stub bake calls (the configs); ``errors["next"]`` makes the next ones raise."""

    def __init__(self) -> None:
        super().__init__()
        self.errors: dict = {}


@pytest.fixture
def stub_bakes(server_app, monkeypatch, tmp_path):
    """``/api/glb`` baking into ``tmp_path`` with a stub bake: the configs it got."""
    calls = _Calls()

    def fake_bake(out, config=None, **_):
        calls.append(config)
        err = calls.errors.get("next")
        if err is not None:
            raise err
        Path(out).write_bytes(b"glTF-stub")

    monkeypatch.setattr(server_app, "bake_gltf", fake_bake)
    monkeypatch.setattr(server_app, "DATA_DIR", tmp_path)
    monkeypatch.setattr(server_app, "_sources_mtime", lambda: 0.0)
    monkeypatch.setattr(server_app, "_FAILED", {})
    monkeypatch.setattr(server_app, "_BAKED", {})
    return calls


def test_api_glb_parameters_are_cached_per_set(client, stub_bakes, tmp_path):
    r = client.get("/api/glb/robot", params={"module": "quad", "phases": "0,180,90,270"})
    assert r.status_code == 200
    assert (tmp_path / "klann_quad_robot.glb").exists()             # the default's path
    q = {"module": "quad", "phases": "0,175,180,355", "p.DF": "2.4"}
    assert client.get("/api/glb/robot", params=q).status_code == 200
    assert client.get("/api/glb/robot", params=q).status_code == 200      # cached
    assert len(stub_bakes) == 2
    config = stub_bakes[1]
    assert (config.module, config.robot) == ("quad", True)
    assert config.proportions == (("DF", 2.4),)
    assert config.phases == pytest.approx(tuple(math.radians(p) for p in TUNED_QUAD))
    assert len(list(tmp_path.glob("klann_quad_robot_*.glb"))) == 1
    assert client.get("/api/glb/klann", params={"phases": "90"}).status_code == 200
    assert (stub_bakes[-1].module, stub_bakes[-1].robot) == ("single", False)
    assert stub_bakes[-1].phases == (math.pi / 2,)


def test_api_glb_linkage_is_part_of_the_design(client, stub_bakes, tmp_path):
    """``linkage=klann`` is the default design; another linkage bakes (and caches) its own."""
    assert client.get("/api/glb/robot", params={"linkage": "klann"}).status_code == 200
    assert stub_bakes == [BuildConfig()]                         # the plain default bake
    assert (tmp_path / "klann_quad_robot.glb").exists()
    for _ in range(2):                                           # the second one is cached
        r = client.get("/api/glb/robot", params={"linkage": "jansen", "module": "double"})
        assert r.status_code == 200
    q = {"linkage": "strider", "module": "double"}               # same module, other linkage
    assert client.get("/api/glb/robot", params=q).status_code == 200
    assert [(c.linkage, c.module) for c in stub_bakes[1:]] == [("jansen", "double"),
                                                                ("strider", "double")]
    assert {p.name for p in tmp_path.glob("*_double_robot.glb")} == {"jansen_double_robot.glb",
                                                                     "strider_double_robot.glb"}


@pytest.mark.parametrize(("mode", "params", "expected"), [
    ("robot", {}, ("klann", "quad", True)),
    ("robot", {"module": "single"}, ("klann", "single", True)),
    ("side", {}, ("klann", "quad", False)),
    ("side", {"module": "double", "linkage": "jansen"}, ("jansen", "double", False)),
    ("klann", {}, ("klann", "single", False)),                   # the ids old URLs use
    ("klann", {"linkage": "crank_rocker"}, ("crank_rocker", "single", False)),   # a mechanism
    ("double", {}, ("klann", "double", False)),
    ("decker", {"module": "decker"}, ("klann", "decker", False)),
    ("double_double", {}, ("klann", "quad", False)),
])
def test_api_glb_mode_ids_are_a_module_and_a_side(client, stub_bakes, tmp_path, mode, params,
                                                  expected):
    """Every id is (linkage, module, robot?) of :class:`config.BuildConfig`; the file is
    the config's key."""
    assert client.get(f"/api/glb/{mode}", params=params).status_code == 200
    linkage_key, module, robot = expected
    assert stub_bakes == [BuildConfig(linkage=linkage_key, module=module, robot=robot)]
    assert (tmp_path / f"{linkage_key}_{module}_{'robot' if robot else 'side'}.glb").exists()


def test_api_modes_are_the_dropdown(client, server_app):
    body = client.get("/api/modes").json()
    assert body["default"] == "robot"
    assert body["modes"] == ["robot", "klann", "double", "decker", "double_double"]
    assert body["labels"]["klann"] == "single (one leg)"
    assert "side" in server_app.MODES                # an id for URLs, not for the dropdown
    assert "side" not in body["modes"]


@pytest.mark.parametrize(("mode", "params"), [
    ("robot", {"module": "octo"}),
    ("robot", {"phases": "0,90"}),
    ("robot", {"p.XX": "2"}),
    ("robot", {"linkage": "octopus"}),
    ("robot", {"linkage": "jansen", "p.DF": "2"}),
    ("robot", {"linkage": "crank_rocker"}),     # a mechanism has no feet: one side only
    ("side", {"linkage": "octopus"}),
    ("klann", {"module": "quad"}),              # a side-only mode is its own module
])
def test_api_glb_rejects_bad_parameters(client, stub_bakes, mode, params):
    r = client.get(f"/api/glb/{mode}", params=params)
    assert r.status_code == 422
    assert r.json()["detail"]
    assert stub_bakes == []


def test_api_glb_unknown_mode(client, stub_bakes):
    assert client.get("/api/glb/octopod").status_code == 404


def test_api_glb_invalid_linkage_is_422(client, stub_bakes):
    r = client.get("/api/glb/robot", params={"p.MC": "0.3"})
    assert r.status_code == 422
    assert "C can't be placed" in r.json()["detail"]
    assert stub_bakes == []                    # caught from the kinematics, before baking


def test_api_glb_unbuildable_design_is_422(client, stub_bakes, server_app):
    from construction import ConstructionError

    for last, err in ((265, ValueError("claim pillar_A can't be built in this layout")),
                      (260, ConstructionError("a tie column is too thin"))):
        stub_bakes.errors["next"] = err
        q = {"phases": f"0,180,90,{last}"}
        r = client.get("/api/glb/robot", params=q)
        assert r.status_code == 422
        assert str(err) in r.json()["detail"]
        n = len(stub_bakes)
        assert client.get("/api/glb/robot", params=q).status_code == 422      # remembered
        assert len(stub_bakes) == n
    assert not list(Path(server_app.DATA_DIR).glob("*.glb"))
