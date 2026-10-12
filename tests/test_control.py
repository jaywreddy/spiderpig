"""Driving a built arm (2026-10-12, phase 6): :mod:`spiderpig.control`."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig.config import BuildConfig
from spiderpig.control import calibrate, homing, host, klipper
from spiderpig.control.arm import ArmModel, IKError
from spiderpig.control.mapping import Axis, axes
from tests import cache


def _model(**kw):
    cfg = BuildConfig(linkage="palletizer", **kw)
    _, design = cache.cached_design(cfg)
    return cfg, design, ArmModel.from_config(cfg, design)


def _poses(model, n, seed):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        q = {k: rng.uniform(lo + 0.02, hi - 0.02) for k, (lo, hi) in model.ranges.items()}
        if model.yaw is not None:
            ylo, yhi = model.yaw[1]
            q["q1"] = rng.uniform(ylo + 0.02, yhi - 0.02)
        out.append(q)
    return out


@pytest.mark.no_fabricate
@pytest.mark.parametrize("kw", [{}, {"robot": False},
                                {"roles": (("tool", "suction_wrist"),)}],
                         ids=["turret", "side", "wrist"])
def test_the_inverse_undoes_the_forward(kw):
    _, _, model = _model(**kw)
    for q in _poses(model, 12, 7):
        tip = model.fk(q)
        back = model.ik(tip)
        np.testing.assert_allclose(model.fk(back), tip, atol=1e-5)
        for k, v in q.items():           # one solution in the box: the same joints
            assert back[k] == pytest.approx(v, abs=1e-6)


@pytest.mark.no_fabricate
def test_the_inverse_refuses_what_the_arm_cant_reach():
    _, _, model = _model()
    with pytest.raises(IKError, match="out of reach"):
        model.ik((900.0, 0.0, 100.0))
    q = _poses(model, 1, 3)[0]
    tip = model.fk(dict(q, q1=0.0))
    behind = (-tip[0], -tip[1], tip[2])              # the turntable would turn 180 deg
    with pytest.raises(IKError, match="turntable"):
        model.ik(behind)


@pytest.mark.no_fabricate
def test_each_joint_counts_its_motors_steps_through_its_ratio():
    cfg, design, _ = _model(roles=(("turntable", "capstan_preload"),))
    ax = axes(cfg, design)
    assert set(ax) == {"q1", "q2", "q3"}
    q2, q3, q1 = ax["q2"], ax["q3"], ax["q1"]
    assert q2.per_rad == pytest.approx(5.0 * 200 * 16 / (2 * math.pi))
    assert (q2.direction, q3.direction) == (1, -1)   # outer plate's shaft +z, inner's -z
    assert q1.direction == -1                         # the capstan reverses the sense
    assert q1.ratio == pytest.approx(40.5 / 3.5)
    a = q2
    for q in np.linspace(*a.limits, 7):
        assert a.angle(a.steps(float(q))) == pytest.approx(q, abs=0.5 / a.per_rad)
    assert a.full_step(37) == 32
    assert a.full_step(41) == 48
    with pytest.raises(ValueError, match="outside"):
        a.steps(a.limits[1] + 0.1)
    assert a.deg_per_motor_rev == pytest.approx(72.0)


@pytest.mark.no_fabricate
def test_homing_snaps_to_a_full_step_and_a_gauge_sets_the_zero():
    a = Axis("q2", 5.0, 1.8, 16, -1)
    move, q = homing.snap(a, 1003)
    assert (1003 + move) % 16 == 0
    assert abs(move) <= 8
    assert q == pytest.approx(a.angle(1003 + move))
    b = homing.set_from_gauge(a, 63.5, steps=1200)
    assert b.angle(1200) == pytest.approx(math.radians(63.5))
    steps = [s.axis for s in homing.plan({"q1": a, "q2": a, "q3": a})]
    assert steps[0] == "q1"
    assert steps[-1] == "q1,q2,q3"


@pytest.mark.no_fabricate
def test_calibration_finds_the_built_arms_lengths_and_zeros():
    _, _, model = _model(robot=False)
    from spiderpig import linkage

    lk = linkage.get("palletizer")
    built = list(model.values)
    names = list(lk.params)
    built[names.index("upper")] += 0.4
    built[names.index("fore")] -= 0.3
    off = np.array([0.004, -0.006])
    rng = np.random.default_rng(1)
    q = np.stack([rng.uniform(*model.ranges[n], 30) for n in model.inputs], axis=1)
    sol = linkage.LegSolution(1, 0.0, tuple(built), "palletizer")
    meas = sol.at(q[:, 0] + off[0], q[:, 1] + off[1])["F"].reshape(-1, 2)
    fit = calibrate.fit("palletizer", model.values, q, meas, ("upper", "fore"), "F")
    assert fit.rms_mm < 1e-6
    assert fit.values[names.index("upper")] == pytest.approx(built[names.index("upper")],
                                                              abs=1e-6)
    assert fit.offsets["q2"] == pytest.approx(off[0], abs=1e-8)


@pytest.mark.no_fabricate
def test_the_klipper_config_drives_each_joint_in_degrees():
    cfg, design, _ = _model()
    ax = axes(cfg, design)
    text = klipper.printer_cfg(ax)
    for n, a in ax.items():
        assert f"[manual_stepper {n}]" in text
        assert f"rotation_distance: {a.deg_per_motor_rev:.6f}" in text
        assert f"[tmc2209 manual_stepper {n}]" in text
    assert "endstop_pin: ^<Q1_ENDSTOP_PIN>" in text
    assert "<MCU_SERIAL>" in text


@pytest.mark.no_fabricate
def test_a_straight_move_is_synchronised_segments_on_the_line():
    cfg, design, model = _model()
    ax = axes(cfg, design)
    q0 = _poses(model, 1, 11)[0]
    a = model.fk(q0)
    b = a + np.array([15.0, -10.0, 8.0])
    lines, q_end = host.line(model, ax, q0, b, seg_mm=2.0, speed_mm_s=20.0)
    np.testing.assert_allclose(model.fk(q_end), b, atol=1e-5)
    segs = [lines[i:i + 3] for i in range(0, len(lines), 3)]
    assert len(segs) == math.ceil(float(np.linalg.norm(b - a)) / 2.0)
    for seg in segs:                     # all at once, the slowest waiting
        assert [s.endswith("SYNC=1") for s in seg] == [False, False, True]
    first = {s.split()[1].split("=")[1]: s for s in segs[0]}
    assert set(first) == {"q1", "q2", "q3"}
    assert host.mscnt_snap(1008, 16) == 1          # 16 counts a microstep, to 1024
    assert host.mscnt_snap(272, 16) == -1          # back to 256
    home = host.home_gcode(ax)
    assert any("STOP_ON_ENDSTOP=1" in h for h in home)
    assert not any("MOVE=360" in h for h in home)       # never past the yaw's range


def test_the_host_talks_to_moonraker(tmp_path):
    import http.server
    import threading
    import urllib.parse

    got = []

    class H(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            n = int(self.headers["Content-Length"])
            got.append((self.path, urllib.parse.parse_qs(self.rfile.read(n).decode())))
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"result": "ok"}')

        def log_message(self, *a):
            pass

    srv = http.server.HTTPServer(("127.0.0.1", 0), H)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        client = host.Moonraker(f"http://127.0.0.1:{srv.server_port}")
        assert client.script(["M400", "SET_PIN PIN=vacuum_pump VALUE=1"]) == {"result": "ok"}
    finally:
        srv.shutdown()
    path, form = got[0]
    assert path == "/printer/gcode/script"
    assert form["script"] == ["M400\nSET_PIN PIN=vacuum_pump VALUE=1"]


@pytest.mark.slow
def test_the_forward_kinematics_is_where_the_fabricated_cup_is():
    """The model's tip at the fabricated pose is the cup's lip, in the URDF's frame."""
    from spiderpig.urdf import model as urdf_model

    cfg, _, model = _model()
    mech = cache.cached_robot(cfg, 1.0)
    lk = cfg.lk
    q0 = {n: float(v[0]) for n, v in zip(lk.inputs, lk.tour(np.array([1.0])), strict=True)}
    cup = next(b for b in mech.bodies if b.name == "tool_cup").placed_part().bounding_box()
    lip = model.to_base((cup.center().X, cup.min.Y, cup.center().Z))
    np.testing.assert_allclose(model.fk(dict(q0, q1=0.0)), lip, atol=1e-6)
    u = urdf_model(mech, cfg, 1.0)
    np.testing.assert_allclose(model.to_base(u.origins["tool0"]), lip, atol=1e-6)
