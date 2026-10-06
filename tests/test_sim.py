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
from spiderpig.hardware.catalog import get  # noqa: E402
from spiderpig.sim.mjcf import (  # noqa: E402
    MIN_CRANK_ARMATURE,
    MM,
    SimParams,
    build_mjcf,
    fabricated,
    load_model,
)
from spiderpig.sim.run import (  # noqa: E402
    PhaseLock,
    _control_fn,
    body_motions,
    compare_with_walk,
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
    from spiderpig.hardware.mass import item_material

    _, model, meta, robot = built
    grams = 0.0
    for b in robot.bodies:
        if b.part is None:
            continue
        cm3 = b.part.volume / 1000.0
        if b.fab == "laser":                                    # its sheet: acrylic, or the
            grams += get(b.sheet or "acrylic_3mm").dims["density"] * cm3   # frame's aluminium
        elif b.fab == "printed":
            grams += 1.24 * cm3                                 # PLA, solid
        elif b.bom_key == "servo_sts3215":
            grams += 55.0
        elif b.bom_key and get(b.bom_key).dims.get("mass_g"):   # the deck's electronics
            grams += get(b.bom_key).dims["mass_g"]
        elif "horn" in b.name:
            grams += 2.70 * cm3                                 # aluminium
        elif "insert" in (b.bom_key or ""):
            grams += 8.5 * cm3                                  # brass
        else:   # steel, unless the item says (aluminium standoffs, PTFE, nylon: round 4)
            grams += {"aluminium": 2.70, "brass": 8.5, "ptfe": 2.2, "nylon": 1.14}.get(
                item_material(b.bom_key), 7.85) * cm3
    total = float(model.body_mass.sum())
    assert SimParams().payload_g == 0.0                 # the electronics deck is modelled
    assert meta["mass"]["by_material"]["electronics"] > 0.05
    assert "payload" not in meta["mass"]["by_material"]
    assert total == pytest.approx(meta["mass"]["total"], rel=1e-6)
    assert total == pytest.approx(grams / 1000.0, rel=0.03)
    assert (model.body_mass[1:] > 0).all()
    assert 0.2 < total < 1.5          # kg (aluminium frame and crank plates since 2026-10-04)
    # a payload rides the base alone, at its com when it sits on the crank axis
    payload = 100.0
    loaded, lmeta = load_model(built[0], SimParams(payload_g=payload))
    assert lmeta["mass"]["by_material"]["payload"] == pytest.approx(payload / 1000.0)
    base = model.body("base").id
    assert loaded.body_mass[base] - model.body_mass[base] == pytest.approx(payload / 1000.0)
    others = [i for i in range(model.nbody) if i != base]
    np.testing.assert_allclose(loaded.body_mass[others], model.body_mass[others])


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


def test_the_loads_that_break_parts_are_measured(built, walk):
    """The pin (loop) forces, the base's vertical acceleration and the per-revolution
    airborne fraction come out of a run: one entry per loop, finite, and the peak at least
    the 99.9th percentile; the feet carry the weight."""
    cfg, model, meta, _ = built
    m = walk_metrics(walk, skip=SETTLE + 0.5)
    assert set(m["loop_force"]) == {lp["name"] for lp in meta["loops"]}
    for v in m["loop_force"].values():
        assert 0.0 <= v["p999"] <= v["peak"] < 1e3
    assert m["loop_force_peak"] == max(v["peak"] for v in m["loop_force"].values())
    assert m["loop_force_p999"] >= 1.0                     # N: a walking pin is loaded
    assert 0.0 < m["accel_z_peak_g"] < 50.0        # a single leg sits on its frame: 0.2 g
    if cfg.module == "quad":                # (a single leg sits on its frame, feet unloaded)
        assert m["foot_force_peak"] > 0.5 * m["mass"] * 9.81 / len(walk.feet)
    assert len(m["airborne_per_rev"]) >= 1
    assert all(0.0 <= a <= 1.0 for a in m["airborne_per_rev"])
    assert m["walks"] == (abs(m["stride"]) >= 5.0 and m["body_contact"] < 0.1)
    assert walk.loop_force.shape == (len(walk.t), model.neq)
    assert np.isfinite(walk.loop_force).all()
    assert np.isfinite(walk.base_acc).all()


def test_drive_torque_within_servo_limits(built, walk):
    """The drives turn at the commanded speed without saturating; the mean load is a
    small part of the stall torque and of the rated torque, and the servo is on its
    speed-torque line (giving all it has at that speed) only now and then at 80 %."""
    m = walk_metrics(walk, skip=SETTLE + 0.5)
    for name, d in m["torque"].items():
        assert d["peak"] <= d["limit"] + 1e-9, name
        assert d["saturated"] < 0.01, (name, d)
        assert d["mean"] < 0.25 * d["limit"], (name, d)
        assert d["rated"] == pytest.approx(5.0 * 9.80665e-2, rel=1e-6)
        assert d["mean_over_rated"] < 1.0, (name, d)
        assert d["at_envelope"] < 0.5, (name, d)
        assert d["speed_droop"] < 0.05, (name, d)
        assert d["speed"] == pytest.approx(WALK * walk.speed_max, rel=0.05)


# The dynamics these tests measured (the phase lock, the held offsets, the steering) are the
# Klann quad's with the keyed crank and printed pillars (16 layers): pinned, so the numbers
# keep their stack. The default (the bolt crank, standoffs: 31 layers) walks in
# test_each_walker_walks_forward_upright.
OLD = {"crank": "keyed", "pillar": "printed"}


@pytest.fixture(scope="module")
def quad():
    cfg = BuildConfig(linkage="klann", module="quad", **OLD)
    _, meta = load_model(cfg)
    return cfg, _vmax(meta)


def _over_line(r) -> float:
    """How far (N·m) the drive torque went past the motor's line at its speed, motoring."""
    k = r.t > SETTLE + 0.7
    tq, w = r.torque[k, 0], r.crank_speed[k, 0]
    avail = r.torque_max * np.clip(1.0 - np.abs(w) / r.speed_max, 0.0, None)
    motoring = tq * w > 0
    return float((np.abs(tq[motoring]) - avail[motoring]).max())


def test_full_speed_torque_stays_on_the_motor_line(quad):
    """At a full command the stiff loop alone would deliver up to the stall torque at
    99 % of the no-load speed (which no DC servo does); with the line enforced the
    drives never exceed ``stall * (1 - |w| / w0)``, the robot still walks, and the
    metrics say how long the servo was flat out and how much speed it gave up."""
    cfg, vmax = quad
    r = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, vmax, vmax)], SETTLE + 3.0)
    m = walk_metrics(r, skip=SETTLE + 0.7)
    assert _over_line(r) < 0.03 * r.torque_max
    assert m["speed"] > 100
    assert not m["fell"]
    for d in m["torque"].values():
        assert 0.0 < d["at_envelope"] <= 1.0
        assert 0.0 <= d["speed_droop"] < 0.1
        assert d["peak"] < 0.5 * d["limit"]
    free = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, vmax, vmax)], SETTLE + 3.0, motor=False)
    assert _over_line(free) > 0.3 * r.torque_max       # what the clamp takes away


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
    assert m["walks"]
    assert m["side_phase_max"] < 3.0            # the lock holds the sides against the mismatch
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


STEER_REQUIREMENT = ("a differential command must not tip the quad (max tilt < 20 deg) and "
                     "must turn it (|yaw rate| >= 5 deg/s); the Klann quad at 0/180/90/270 "
                     "rolls over whenever its sides run more than ~90 deg out of phase "
                     "(measured: (1,-1) over at 1.6 s, (1,0.3) at 0.9 s, (0.6,0.4) at 3.9 s; "
                     "(1,0.6) at 1.6 s with the 100 g payload): a design decision")


def test_a_control_schedule_with_two_entries_at_one_time_takes_the_later():
    """Two entries at the same ``t0`` used to compare the arrays in the sort (a numpy truth
    value error); the sort is by time alone and the later entry wins the tie."""
    f = _control_fn([(0, 0, 0), (0.3, 1.0, 0.0), (0.3, 1.0, 1.0)], 1.0)
    assert f(0.0).tolist() == [0.0, 0.0]
    assert f(0.3).tolist() == [1.0, 1.0]
    assert f(1.0).tolist() == [1.0, 1.0]


MISMATCH_NOTE = ("two open-loop servos drift apart: the right one 5 % slower rolls the Klann "
                 "quad over at 8 s (measured); the phase lock the real bus needs keeps it up")


def test_the_phase_lock_keeps_a_mismatched_pair_walking(quad):
    """10 s at 80 % with the right servo 5 % slower: open loop (``lock=False`` keeps the
    mismatch out, so it is applied by hand) the quad rolls over; through the lock it
    walks on with the sides within a few degrees."""
    cfg, vmax = quad
    u = WALK * vmax
    params = SimParams(servo_mismatch=0.05)
    loose = simulate(cfg, lambda t: (0.0, 0.0) if t < SETTLE else (u, u * 0.95), SETTLE + 10.0,
                     params=params, lock=False, record_every=4)
    m0 = walk_metrics(loose, skip=1.0)
    assert m0["fell"], MISMATCH_NOTE
    assert m0["side_phase_max"] > 45.0
    locked = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, u, u)], SETTLE + 10.0, params=params,
                      record_every=4)
    m = walk_metrics(locked, skip=1.0)
    assert not m["fell"], m["fell_at_s"]
    assert m["max_tilt"] < 15.0
    assert m["side_phase_max"] < 5.0
    assert m["speed"] > 100
    assert abs(m["yaw_rate"]) < 3.0


@pytest.mark.parametrize(("offset_deg", "survives"), [(45.0, True), (180.0, False)])
def test_a_held_side_offset_under_the_lock(quad, offset_deg, survives):
    """The right side held back until the cranks are ``offset_deg`` apart, then both at full
    speed with the lock bounding the offset there: 45 deg walks on (18 deg of tilt at most,
    measured; what ``steering_check`` grants as ``step_deg``), 180 deg rolls it over within
    two seconds (the quasi-static support says the margin is 0 at any offset past 30 deg;
    MuJoCo is kinder up to 45, measured). Open loop, 90 deg and more fell within a second.
    Since the Chicago screws as bought (2026-10-05: taller heads, a wider stack) 180 deg tips it
    to about 25 deg without a fall: past the 20 deg that counts as walking on, so unsafe still."""
    cfg, vmax = quad
    hold = math.radians(offset_deg) / (0.4 * vmax)
    lock = PhaseLock(vmax, mismatch=SimParams().servo_mismatch, max_offset=math.radians(offset_deg))
    r = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, vmax, 0.6 * vmax), (SETTLE + hold, vmax, vmax)],
                 SETTLE + hold + 4.0, lock=lock, record_every=4)
    m = walk_metrics(r, skip=SETTLE + hold)
    if survives:
        assert not m["fell"], (m["fell_at_s"], m["max_tilt"])
        assert m["max_tilt"] < 20.0
    else:
        assert m["fell"] or m["max_tilt"] > 20.0, (m["fell_at_s"], m["max_tilt"])
        assert m["side_phase_max"] < offset_deg + 5.0


def test_the_sim_height_and_pitch_follow_the_quasi_static_support(quad):
    """The wobble is kinematic, not numerical: the base height and pitch against the crank
    angle match the quasi-static support plane (``walk.support``) within 1 mm / 1 deg rms
    (0.6 mm / 0.3 deg measured, unchanged from dt 2e-3 to 2.5e-4), the means apart (the
    sim's base sits a link radius plus the clearance higher)."""
    from spiderpig import walk

    cfg, vmax = quad
    r = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, vmax, vmax)], SETTLE + 4.0)
    sign = r.meta["crank_sign"]
    k = r.t > SETTLE + 1.0
    ang = (r.crank[k, 0] * sign) % (2 * math.pi)
    nb = 72
    b = np.floor(ang / (2 * math.pi) * nb).astype(int) % nb
    h = r.base_pos[k, 2] * 1e3
    p = np.degrees(r.attitude[k, 1])
    hs = np.array([h[b == i].mean() for i in range(nb)])
    ps = np.array([p[b == i].mean() for i in range(nb)])
    w = walk.walker(cfg)
    ths = np.linspace(0.0, 2 * math.pi, nb, endpoint=False)
    P, _ = w.feet_at(ths, ths)
    sup = walk.support(P, w.com)
    hq, pq = np.asarray(sup.height), np.asarray(sup.pitch_deg)
    assert np.sqrt(np.mean(((hs - hs.mean()) - (hq - hq.mean())) ** 2)) < 1.0
    assert np.sqrt(np.mean(((ps - ps.mean()) - (pq - pq.mean())) ** 2)) < 1.0
    assert np.ptp(hs) == pytest.approx(np.ptp(hq), abs=3.0)


@pytest.mark.parametrize(("left", "right"), [
    pytest.param(1.0, -1.0, marks=pytest.mark.xfail(reason=STEER_REQUIREMENT, strict=False)),
    pytest.param(1.0, 0.0, marks=pytest.mark.xfail(reason=STEER_REQUIREMENT, strict=False)),
    pytest.param(1.0, 0.3, marks=pytest.mark.xfail(reason=STEER_REQUIREMENT, strict=False)),
])
def test_quad_steers_without_tipping(quad, left, right):
    """The steering matrix the drive layer would need before it may send a full
    differential (see :data:`STEER_REQUIREMENT`)."""
    cfg, vmax = quad
    _, m = _run(cfg, left * vmax, right * vmax)
    assert not m["fell"], m["fell_at_s"]
    assert m["max_tilt"] < 20.0
    assert abs(m["yaw_rate"]) >= 5.0
    assert math.isfinite(m["turn_radius"])


@pytest.mark.parametrize("armature", [MIN_CRANK_ARMATURE, 5e-3, 2e-2])
def test_the_model_is_well_posed_across_the_crank_armature(quad, armature):
    """The crank's reflected rotor inertia is an estimate: over its plausible range the
    loops stay closed and a straight walk doesn't roll; below it the model is refused."""
    cfg, vmax = quad
    r = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, vmax, vmax)], SETTLE + 2.0,
                 params=SimParams(crank_armature=armature))
    m = walk_metrics(r, skip=SETTLE + 0.5)
    assert m["loop_error"] < 0.2, m["loop_error"]
    assert m["roll_range"] < 1.0, m["roll_range"]
    assert m["speed"] > 100
    assert not m["fell"]


def test_too_little_crank_armature_is_refused():
    with pytest.raises(ValueError, match="ill-posed"):
        build_mjcf(BuildConfig(linkage="klann", module="quad"), SimParams(crank_armature=0.0))


def test_strider_walks_on_its_feet_alone():
    """Strider's ``b4`` / ``b8`` end on the foot joints: their capsules must not touch the
    floor beside the feet (``body_contact`` 0), and the walk reads as the model's."""
    cfg = BuildConfig(linkage="strider", module="quad", **OLD)   # the bolt crank's doesn't plan
    _, meta = load_model(cfg)
    for info in meta["feet"].values():
        assert len(info["links"]) >= 2
        assert info["body"] in info["links"]
    vmax = _vmax(meta)
    r = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, vmax, vmax)], SETTLE + 3.0)
    m = walk_metrics(r, skip=SETTLE + 0.7)
    assert m["body_contact"] == 0.0
    assert not m["fell"]
    assert m["max_tilt"] < 5.0
    assert m["feet_down"] > 3.5
    assert m["speed"] > 100


UNSTABLE = "the quasi-static margin is under 15 mm: this design tips (a design decision)"
WALKERS = [      # the Strider quad keyed (with the bolt crank it doesn't plan in the budget)
    ("klann", "quad"), ("strider", "quad"),
    pytest.param("jansen", "quad", marks=pytest.mark.xfail(reason=UNSTABLE, strict=False)),
    pytest.param("trotbot_heel", "quad", marks=pytest.mark.xfail(reason=UNSTABLE, strict=False)),
]


@pytest.mark.parametrize(("key", "module"), WALKERS)
def test_each_walker_walks_forward_upright(key, module):
    """4 s at full speed: still on its feet (up-vector z > 0.7) and walking the way the
    quasi-static model says (the sign of its stride, in the drive's forward sense)."""
    from spiderpig import walk

    cfg = BuildConfig(linkage=key, module=module, **(OLD if key == "strider" else {}))
    _, meta = load_model(cfg)
    vmax = _vmax(meta)
    r = simulate(cfg, [(0.0, 0.0, 0.0), (SETTLE, vmax, vmax)], SETTLE + 4.0)
    m = walk_metrics(r, skip=SETTLE + 0.7)
    qs = walk.straight_walk_metrics(walk.walker(cfg))
    assert not m["fell"], (m["fell_at_s"], m["fell_axis"])
    assert math.cos(r.tilt[-1]) > 0.7
    assert np.sign(m["speed"]) == np.sign(qs["stride_signed_mm"] * meta["crank_sign"])
    assert abs(m["speed"]) > 50


def test_mujoco_and_the_quasi_static_model_are_compared(quad):
    """The comparison the tune panel and the HUD show: Klann's quad walks 1.9x faster in
    MuJoCo than the quasi-static model says (a known gap, flagged) and one side stands
    on a single foot much of the time; Strider's two agree."""
    cfg, vmax = quad
    _, m = _run(cfg, vmax, vmax, seconds=4.0)
    c = compare_with_walk(m, cfg)
    assert set(c) == {"mujoco", "quasi_static", "speed_ratio", "flags", "notes"}
    assert 1.5 < c["speed_ratio"] < 2.2, c
    assert c["mujoco"]["stride_kinematic_mm"] > c["mujoco"]["stride_mm"] > 0
    assert 0.0 < c["mujoco"]["slip_fraction"] < 0.6          # the feet slip, the stride isn't lost
    assert len(c["notes"]) == 4
    assert "phase-locked" in c["notes"][3]
    assert "speeds_disagree" in c["flags"]
    assert "support_low" in c["flags"]
    assert "does_not_walk" not in c["flags"]
    assert c["mujoco"]["walks"]
    assert c["quasi_static"]["walks"]
    assert "fell" not in c["flags"]
    assert sum(m["feet_down_hist"]) == pytest.approx(1.0)
    assert m["airborne"] < 0.1
    strider = BuildConfig(linkage="strider", module="quad", **OLD)
    _, meta = load_model(strider)
    _, ms = _run(strider, _vmax(meta), _vmax(meta), seconds=3.0)
    cs = compare_with_walk(ms, strider)
    assert abs(cs["speed_ratio"] - 1.0) < 0.25, cs
    assert cs["flags"] == []


def test_a_spin_has_no_full_speed_scaling(quad):
    """Opposed drives: the mean signed crank rate is near zero, which used to scale a
    spin's tiny forward speed to tens of metres a second; now ``revolutions_abs`` says
    how fast the cranks turned and the straight-walk comparison is ``nan``, unflagged."""
    cfg, vmax = quad
    _, m = _run(cfg, 0.4 * vmax, -0.4 * vmax, seconds=2.0)
    assert m["drives_oppose"]
    assert m["revolutions_abs"] > 0.3 > abs(m["revolutions"])
    c = compare_with_walk(m, cfg)
    assert math.isnan(c["mujoco"]["speed_mm_s"])
    assert "speeds_disagree" not in c["flags"]
    _, m = _run(cfg, 0.4 * vmax, 0.4 * vmax, seconds=2.0)
    assert not m["drives_oppose"]
    assert m["revolutions_abs"] == pytest.approx(m["revolutions"])


def test_the_steering_check_follows_its_runs(quad):
    """``steering_check`` runs a 0.4 differential while walking and a half-speed turn in
    place and grants each only when the robot neither fell nor tilted past
    ``STEER_TILT`` (the hello's ``steering``: what the viewer may send by default)."""
    from spiderpig.sim.run import STEER_SECONDS, STEER_TILT, steering_check

    cfg, _ = quad
    s = steering_check(cfg)
    assert s["seconds"] == STEER_SECONDS
    for name, expect in (("walk_turn", 0.4), ("spin", 0.5)):
        t = s["tests"][name]
        unsafe = t["fell"] or t["max_tilt"] >= STEER_TILT
        assert s[{"walk_turn": "turn", "spin": "spin"}[name]] == (0.0 if unsafe else expect), t
        assert t["cmd"] == list({"walk_turn": (1.0, 0.6), "spin": (0.5, -0.5)}[name])
    # the straight run is the gate, and the excursion the lock bounds steering to
    assert not s["forward"]["fell"]
    assert s["forward"]["walks"]
    assert s["forward"]["speed"] > 100
    assert s["tests"]["forward"]["cmd"] == [1.0, 1.0]
    assert s["step_deg"] in (0.0, 45.0, 90.0)
    steps = [k for k in s["tests"] if k.startswith("step_")]
    assert steps, s["tests"].keys()
    if s["step_deg"]:
        t = s["tests"][f"step_{s['step_deg']:.0f}"]
        assert not t["fell"]
        assert t["max_tilt"] < STEER_TILT
        assert t["side_phase_max"] < s["step_deg"] + 5.0
    # the Klann quad with the keyed crank (measured; the fixture's): no unbounded differential, a
    # 90 deg excursion. (With ``--crank printed``'s 12-layer stack it was 45: the keyed
    # crank's 16 layers set the feet 12 mm further out, the stability margin 50 -> 57 mm,
    # test_walk.py::test_quad_reference.)
    assert cfg.crank == "keyed"
    assert s["turn"] == 0.0
    assert s["step_deg"] == 90.0


# ---------------------------------------------------------------------------
# Test drive, round 5 (docs/agentlib/TESTDRIVE.md): the kinematic stride's sign, when and
# how a fall is reported, an exported MJCF runs as is
# ---------------------------------------------------------------------------


def test_r5_the_kinematic_stride_reads_forward_for_every_walker():
    from spiderpig.sim.run import kinematic_gait

    # a side with legs that take turns (one foot alone returns where it started: 0)
    for key, module in (("klann", "quad"), ("trotbot_heel", "quad"), ("strider", "double")):
        kin = kinematic_gait(BuildConfig(linkage=key, module=module))         # entry 14
        assert kin["stride"] > 20, (key, kin["stride"])


def test_r5_the_metrics_say_when_and_how_it_fell_and_an_exported_model_runs(built):
    from spiderpig.sim.mjcf import build_mjcf
    from spiderpig.sim.run import simulate, walk_metrics

    cfg = built[0]
    xml, meta = build_mjcf(cfg)
    r = simulate(cfg, seconds=0.3, model_xml=xml, model_meta=meta)           # entry 6
    m = walk_metrics(r, skip=0.1)
    assert set(m) >= {"fell", "fell_at_s", "fell_axis"}
    assert m["fell_at_s"] is None
    assert m["fell_axis"] is None
    with pytest.raises(ValueError, match="model_meta"):
        simulate(cfg, seconds=0.1, model_xml=xml)
