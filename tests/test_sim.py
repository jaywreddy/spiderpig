"""Tests for the MuJoCo model of the walker (:mod:`sim`): the model is the fabricated
robot, it stands, its loops stay closed, it walks and turns, and its servos cope."""

from __future__ import annotations

import math

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from spiderpig import linkage  # noqa: E402
from spiderpig.config import BuildConfig  # noqa: E402
from spiderpig.fabricate import template_for  # noqa: E402
from spiderpig.sim.mjcf import MM, fabricated, load_model  # noqa: E402
from spiderpig.sim.run import (  # noqa: E402
    body_motions,
    kinematic_gait,
    kinematic_qpos,
    loop_errors,
    simulate,
    walk_metrics,
)
from spiderpig.stack import body_class, is_link  # noqa: E402

pytestmark = pytest.mark.slow

# (linkage, module): Klann's single and quad, and a Strider (one coupled pair, two feet)
DESIGNS = (("klann", "single"), ("klann", "quad"), ("strider", "single"))
LOOPS_PER_LEG = {"klann": 2, "strider": 4}     # independent loops of one leg's linkage
SETTLE = 0.3            # s at rest before the drives start
WALK = 0.8              # drive speed, fraction of the servo's no-load speed


@pytest.fixture(scope="module", params=DESIGNS, ids=[f"{k}-{m}" for k, m in DESIGNS])
def built(request):
    """(config, model, metadata, fabricated robot) per design."""
    key, module = request.param
    cfg = BuildConfig(linkage=key, module=module)
    model, meta = load_model(cfg)
    return cfg, model, meta, fabricated(cfg)


def _vmax(meta) -> float:
    return meta["actuators"]["L.drive"]["ctrlrange"][1]


@pytest.fixture(scope="module")
def walk(built):
    """Two seconds of walking at 80 % speed, after settling."""
    cfg, _, meta, _ = built
    w = WALK * _vmax(meta)
    return simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, w, w)], SETTLE + 2.0)


def _side_of(name: str) -> str:
    return name.split(".", 1)[0]


def test_model_compiles_with_the_documented_names(built):
    cfg, model, meta, _ = built
    names = {model.body(i).name for i in range(model.nbody)}
    assert names == {"world", *meta["bodies"]}
    lk = linkage.get(cfg.linkage)
    legs = 2 * len(lk.leg_modules[cfg.module])
    assert len(meta["feet"]) == legs * len(lk.feet)
    assert {info["body"].split(".")[1].split("_")[0] for info in meta["feet"].values()} == \
        {b for b, _ in lk.feet}
    assert model.neq == len(meta["loops"]) == LOOPS_PER_LEG[cfg.linkage] * legs  # Klann: C, E
    assert model.nu == 2
    assert [model.actuator(i).name for i in range(model.nu)] == ["L.drive", "R.drive"]
    assert model.joint("base").type == mujoco.mjtJoint.mjJNT_FREE
    for s in "LR":
        assert model.actuator(f"{s}.drive").trnid[0] == model.joint(f"{s}.crank").id
    for foot, info in meta["feet"].items():
        assert model.site(foot).bodyid == model.body(info["body"]).id
        assert model.geom(foot).bodyid == model.body(info["body"]).id


def test_masses_match_the_fabricated_robot(built):
    """Every part counted once, at its material's density (the servo at its datasheet mass)."""
    _, model, meta, robot = built
    grams = 0.0
    for b in robot.bodies:
        if b.part is None:
            continue
        cm3 = b.part.volume / 1000.0
        if b.fab == "laser":
            grams += 1.19 * cm3                                 # cast acrylic
        elif b.fab == "printed":
            grams += 1.24 * cm3                                 # PLA, solid
        elif b.bom_key == "servo_sts3215":
            grams += 55.0
        elif "horn" in b.name:
            grams += 2.70 * cm3                                 # aluminium
        elif "insert" in (b.bom_key or ""):
            grams += 8.5 * cm3                                  # brass
        else:
            grams += 7.85 * cm3                                 # steel
    total = float(model.body_mass.sum())
    assert total == pytest.approx(meta["mass"]["total"], rel=1e-6)
    assert total == pytest.approx(grams / 1000.0, rel=0.03)
    assert (model.body_mass[1:] > 0).all()
    assert 0.2 < total < 1.0


def test_every_glb_body_has_a_mujoco_body(built):
    """The viewer's glb has one node per fabricated body: each maps to a MuJoCo body."""
    _, model, meta, robot = built
    nodes = meta["nodes"]
    assert set(nodes) == {b.name for b in robot.bodies}
    for name, target in nodes.items():
        assert model.body(target).name == target
        cls = body_class(name)
        if is_link(name):
            assert target == name
        elif cls == "torso":
            assert target == "base"
        elif cls.startswith("conn") or cls == "coupler":
            assert target == f"{_side_of(name)}.conn"
    for b in robot.bodies:
        if b.rigid_with is not None:
            assert nodes[b.name] == nodes[b.rigid_with]


def test_the_model_is_the_template(built):
    """Posed on the kinematics at any crank angle, every loop closes, every foot is
    where the template puts it, and every body's motion (what the viewer applies to
    the glb nodes) takes its joints from ``t_ref`` to ``t``."""
    cfg, model, meta, robot = built
    tmpl = template_for(cfg)
    ref = tmpl.sample(np.array([meta["t_ref"]])).joint_world
    data = mujoco.MjData(model)
    for t in (0.7, 2.0, 3.6, 5.5):
        data.qpos[:] = kinematic_qpos(cfg, t)
        mujoco.mj_kinematics(model, data)
        assert loop_errors(model, data).max() < 1e-6
        now = tmpl.sample(np.array([t])).joint_world
        motion = body_motions(model, data, meta)
        for b in robot.bodies:
            if b.rigid_with is not None or not b.joints or _side_of(b.name) not in "LR":
                continue
            d = motion[meta["nodes"][b.name]]
            side = b.name.split(".", 1)[1]
            for j in b.joints:
                p0 = np.append(ref[side][j.name][0, :2], [0.0, 1.0])
                assert np.abs((d @ p0)[:2] - now[side][j.name][0, :2]).max() < 1e-3
        for s in "LR":
            q = data.qpos[model.joint(f"{s}.crank").qposadr[0]]
            assert meta["t_ref"] + meta["crank_sign"] * q == pytest.approx(t)
        jw = tmpl.sample(np.array([t])).joint_world
        base = data.body("base")
        rot = base.xmat.reshape(3, 3)
        assert sorted((f"{s}.{b}", j) for s in "LR" for b, j in linkage.feet_of(tmpl)) == \
            sorted((info["body"], info["joint"]) for info in meta["feet"].values())
        for foot, info in meta["feet"].items():
            mech = rot.T @ (data.site(foot).xpos - base.xpos) / MM
            want = jw[info["body"].split(".", 1)[1]][info["joint"]][0, :2]
            assert np.abs(mech[:2] - want).max() < 1e-3        # mm


def test_it_starts_on_its_feet(built):
    """At qpos0 the lowest feet just clear the floor, on both sides."""
    _, model, meta, _ = built
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    r = meta["feet"][next(iter(meta["feet"]))]["radius"]
    low = {foot: data.site(foot).xpos[2] - r for foot in meta["feet"]}
    clearance = meta["params"]["clearance"] * MM
    assert min(low.values()) == pytest.approx(clearance, abs=1e-6)
    for s in "LR":
        assert min(z for f, z in low.items() if f.startswith(s)) == pytest.approx(clearance,
                                                                                  abs=1e-6)


def test_it_settles_on_its_feet(built):
    """Drives at rest: the robot comes to rest on its feet, neither sinking nor exploding."""
    cfg, _, _, _ = built
    r = simulate(cfg, (0.0, 0.0), 1.5)
    assert np.isfinite(r.base_pos).all()
    assert np.isfinite(r.torque).all()
    end = r.t > 1.0
    assert r.penetration[end].max() < 2e-3
    assert np.ptp(r.base_pos[end, 2]) < 0.5e-3                  # at rest
    assert r.base_pos[-1, 2] > 0.02
    feet = dict(zip(r.feet, r.foot_contact[end].all(axis=0), strict=True))
    for s in "LR":
        assert any(down for f, down in feet.items() if f.startswith(s)), feet
    if cfg.module == "quad":
        # stands on its feet alone, level to within its rest pitch
        assert not r.body_contact[end].any()
        assert sum(feet.values()) >= 4
        assert math.degrees(r.tilt[-1]) < 10.0
    else:
        # one leg per side: Klann's foot is ahead of the centre of mass, it sits back on
        # its frame; the Strider's pair of feet stand it up, tilted
        assert math.degrees(r.tilt[-1]) < 30.0


def test_loops_stay_closed_while_walking(built, walk):
    r = walk
    assert np.isfinite(r.base_pos).all()
    assert r.loop_error.max() < 0.5e-3, f"loop closure error {r.loop_error.max() * 1e3:.3f} mm"
    assert r.penetration.max() < 5e-3


def test_drive_torque_within_servo_limits(built, walk):
    """The drives turn at the commanded speed without saturating; the mean load is a
    small part of the stall torque and within what a DC-motor servo gives at that speed."""
    m = walk_metrics(walk, skip=SETTLE + 0.5)
    for name, d in m["torque"].items():
        assert d["peak"] <= d["limit"] + 1e-9, name
        assert d["saturated"] < 0.01, (name, d)
        assert d["mean"] < 0.25 * d["limit"], (name, d)
        assert d["envelope_p95"] < 1.0, (name, d)
        assert d["speed"] == pytest.approx(WALK * walk.speed_max, rel=0.05)


@pytest.fixture(scope="module")
def quad():
    cfg = BuildConfig(module="quad")
    _, meta = load_model(cfg)
    return cfg, _vmax(meta)


def _run(cfg, left, right, seconds=3.0):
    r = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, left, right)], SETTLE + seconds)
    return r, walk_metrics(r, skip=SETTLE + 0.7)


def test_quad_walks_forward_and_back(quad):
    cfg, vmax = quad
    w = WALK * vmax
    kin = kinematic_gait(cfg)
    r, m = _run(cfg, w, w)
    assert not m["fell"]
    assert m["body_contact"] == 0.0
    assert m["speed"] > 0
    # consistently forward: every crank revolution gains ground
    t0 = SETTLE + 0.7
    k = r.t >= t0
    revs = np.floor((r.crank[k, 0] - r.crank[k, 0][0]) / (2 * math.pi)).astype(int)
    x = r.base_pos[k, 0]
    gains = [x[revs == i][-1] - x[revs == i][0] for i in range(revs.max())]
    assert len(gains) >= 1
    assert min(gains) > 0.05, gains
    assert abs(m["lateral"]) < 0.1 * m["forward"]
    assert abs(m["heading_drift"]) < 5.0
    # plausible: below the kinematics' no-slip stride, above half of it
    assert 0.5 * kin["stride"] < m["stride"] < kin["stride"], (m["stride"], kin["stride"])
    back_r, back = _run(cfg, -w, -w)
    assert not back["fell"]
    assert back["speed"] < 0
    assert abs(back["speed"]) == pytest.approx(m["speed"], rel=0.25)


def test_quad_turns_on_the_spot(quad):
    """Drives opposed (at 40 % speed): the heading turns while the robot stays near its
    starting point; walking forward at that speed it would go much further."""
    cfg, vmax = quad
    w = 0.4 * vmax
    _, fwd = _run(cfg, w, w)
    _, turn = _run(cfg, w, -w)
    assert not turn["fell"]
    assert abs(turn["heading_drift"]) > 5.0
    moved = math.hypot(turn["forward"], turn["lateral"])
    assert moved < 0.5 * fwd["forward"], (moved, fwd["forward"])
