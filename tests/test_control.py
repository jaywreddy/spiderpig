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
    _, _, side = _model(robot=False)
    tip = side.fk(_poses(side, 1, 5)[0])
    with pytest.raises(IKError, match="plane"):          # a side alone can't reach across
        side.ik((tip[0], tip[1] + 50.0, tip[2]))


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
    with pytest.raises(ValueError, match="outside"):
        a.steps(a.limits[1] + 0.1)
    assert a.deg_per_motor_rev == pytest.approx(72.0)


@pytest.mark.no_fabricate
def test_homing_returns_to_the_reference_phase_and_a_gauge_sets_the_zero():
    # 16 counts a microstep at 16 microsteps; a full step's phase repeats every 256
    assert homing.phase_move(60, 44, 16) == -1          # a microstep past the reference
    assert homing.phase_move(28, 44, 16) == 1
    assert homing.phase_move(300, 44, 16) == 0          # a whole step on: the same phase
    assert homing.phase_move(60, 44, 16, sign=-1) == 1  # a DIR flipped
    for trigger in range(1024):          # wherever it triggers, it lands on the phase
        m = homing.phase_move(trigger, 44, 16)
        left = (trigger + 16 * m - 44 + 128) % 256 - 128
        assert abs(left) <= 8
    a = Axis("q2", 5.0, 1.8, 16, -1)
    b = homing.set_from_gauge(a, 63.5, steps=1200)
    assert b.angle(1200) == pytest.approx(math.radians(63.5))
    steps = homing.plan({"q1": a, "q2": a, "q3": a})
    assert steps[0].axis == "q1"
    assert "energise" in steps[-1].action                # the gauge read with motors on


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
    # measured in a frame shifted 0.5 mm: the flange's offset fitted too absorbs it
    shifted = meas + np.array([0.5, -0.3])
    frame = calibrate.fit("palletizer", model.values, q, shifted,
                          ("upper", "fore", "fx", "fy"), "F")
    assert frame.rms_mm < 1e-6
    from spiderpig.control.mapping import Axis as Ax

    moved = calibrate.apply_offsets(fit, {"q2": Ax("q2", 5.0, 1.8, 16)})
    assert moved["q2"].angle(0) == pytest.approx(off[0], abs=1e-8)


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
    assert "[idle_timeout]" in text
    idle = text.split("[idle_timeout]")[1].split("\n[")[0]
    assert not [ln for ln in idle.splitlines() if ln.startswith("  ") and "M84" in ln]
    # every line a section, a key, an indented G-code line or a comment; no ';' in G-code
    for ln in text.splitlines():
        if ln.startswith("  "):
            assert ";" not in ln, ln
        elif ln and not ln.startswith(("#", "[")):
            assert ":" in ln, ln


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
    assert all("ACCEL=0" in s for s in lines)       # constant speed: the joints keep time
    home = host.home_gcode(ax)
    assert any("STOP_ON_ENDSTOP=1" in h for h in home)
    lo, hi = (math.degrees(v) for v in ax["q1"].limits)
    for h in home:                       # never farther than the half home is in, slowly
        if "MOVE=" in h:
            move = abs(float(h.split("MOVE=")[1].split()[0]))
            assert move <= max(-lo, hi) + 5.0 + 1e-9
            assert "SPEED=" in h
    assert "DUMP_TMC STEPPER=q1 REGISTER=MSCNT" in home


def test_the_host_talks_to_moonraker(tmp_path):
    import http.server
    import json
    import threading

    got = []

    class H(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            n = int(self.headers["Content-Length"])
            got.append((self.path, self.headers["Content-Type"],
                        json.loads(self.rfile.read(n).decode())))
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
    path, kind, body = got[0]
    assert path == "/printer/gcode/script"
    assert kind == "application/json"
    assert body == {"script": "M400\nSET_PIN PIN=vacuum_pump VALUE=1"}


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
    from spiderpig.urdf import forward

    for q in _poses(model, 4, 2):        # the URDF's own forward, at any yaw
        pose = forward(u, {n: q[n] for n in model.inputs}, {"q1": q["q1"]})
        tip = (pose["tool0"] @ np.append(u.origins["tool0"], 1.0))[:3]
        np.testing.assert_allclose(model.to_base(tip), model.fk(q), atol=1e-6)


def test_an_inclinometer_reads_its_links_angle_off_frame():
    """The SCL3300's frames (the datasheet's codes carry their own CRCs), an answer frame
    decoded, and a fake sensor at 30 deg read through the off-frame protocol: each
    request's answer arrives with the next one."""
    from spiderpig.control import incline

    for word in (incline.READ_ACC_X, incline.READ_ACC_Y, incline.READ_STATUS,
                 incline.SW_RESET, incline.MODE_4, incline.READ_WHOAMI):
        assert incline.frame(word >> 24, (word >> 8) & 0xFFFF) == word
    theta = math.radians(30.0)
    raw = {incline.READ_ACC_X: round(12000 * math.sin(theta)) & 0xFFFF,
           incline.READ_ACC_Y: round(12000 * math.cos(theta)) & 0xFFFF,
           incline.READ_STATUS: 0, incline.READ_WHOAMI: incline.WHOAMI}
    last = [None]

    def xfer(word: int) -> int:       # answers the previous request, status RS = 01
        prev, last[0] = last[0], word
        data = raw.get(prev, 0)
        op = ((prev or 0) >> 24) & 0x3F
        return incline.frame((1 << 6) | op, data)

    s = incline.SCL3300(xfer, offset_deg=incline.offset_from_gauge(30.4, 30.0))
    s.start(sleep=lambda _: None)
    assert s.angle(seconds=0.05, sleep=lambda _: None) == pytest.approx(29.6, abs=0.01)
    with pytest.raises(incline.FrameError):
        incline.decode(incline.frame(0x40, 1) ^ 1)        # a flipped bit: bad CRC


def test_homing_reads_the_inclinometers_and_skips_a_capstans_phase():
    a = Axis("q2", 5.0, 1.8, 16, -1)
    steps = homing.plan({"q1": a, "q2": a, "q3": a}, {"q2": "scl3300", "q3": "gauge"},
                        phase=False)
    assert not any("MSCNT" in s.action for s in steps)
    assert any(s.axis == "q2" and "inclinometer" in s.action for s in steps)
    assert any(s.axis == "q3" and "gauge" in s.action for s in steps)


def test_an_endstop_homes_a_joint_the_mt4_way():
    from spiderpig.actuators import sensors

    a = Axis("q2", 5.0, 1.8, 16, -1)
    steps = homing.plan({"q2": a, "q3": a}, {"q2": "endstop", "q3": "endstop"})
    assert [s.axis for s in steps].count("q2") == 3          # seek, slow re-approach, zero
    assert any("slow" in s.action or "tenth" in s.action for s in steps)
    ends = sensors.get("endstop")
    assert sensors.get("gauge").repeat_deg > ends.repeat_deg > sensors.get("scl3300").repeat_deg
    assert ends.bom_key == "optical_endstop"


def test_a_camera_finds_a_dot_and_the_sightings_fix_the_zeros():
    """A synthetic dot found to a hundredth of a pixel; two sightings of a palletizer whose
    zeros are off by known angles give those angles back."""
    from spiderpig.control import vision

    ys, xs = np.indices((120, 160))
    img = 200.0 - 150.0 * (((xs - 81.37) ** 2 + (ys - 57.62) ** 2) < 8.0 ** 2)
    cx, cy = vision.dot_centroid(img)
    assert (cx, cy) == (pytest.approx(81.37, abs=0.1), pytest.approx(57.62, abs=0.1))
    _, _, model = _model(robot=False)
    true = {"q2": math.radians(0.07), "q3": math.radians(-0.05)}
    sightings = []
    for q2, q3 in ((70.0, -20.0), (95.0, 5.0), (55.0, -35.0)):
        q = {"q2": math.radians(q2), "q3": math.radians(q3)}
        real = model.fk({k: v + true.get(k, 0.0) for k, v in q.items()})
        dot = real[:2] + np.array([0.3, -0.2])          # the dot a little off the tip
        sightings.append(vision.Sighting(q, tuple(dot), (0.3, -0.2)))
    got = vision.zero_offsets(model, sightings, ("q2", "q3"))
    assert got["q2"] == pytest.approx(true["q2"], abs=1e-6)
    assert got["q3"] == pytest.approx(true["q3"], abs=1e-6)


def test_the_sightings_fit_a_capstans_drifting_gain_too():
    from spiderpig.control import vision

    _, _, model = _model(robot=False)
    off, gain = math.radians(0.05), 3e-4          # the q2 capstan warm: its ratio off
    sightings = []
    for q2, q3 in ((45.0, -40.0), (70.0, -20.0), (95.0, 5.0), (110.0, 15.0)):
        q = {"q2": math.radians(q2), "q3": math.radians(q3)}
        real = model.fk({"q2": q["q2"] * (1 + gain) + off, "q3": q["q3"]})
        sightings.append(vision.Sighting(q, tuple(real[:2]), (0.0, 0.0)))
    got = vision.zero_offsets(model, sightings, ("q2", "q3"), fit_gain=True)
    assert got["q2"] == pytest.approx(off, abs=1e-6)
    assert got["q2_gain"] == pytest.approx(gain, abs=1e-6)
    assert got["q3_gain"] == pytest.approx(0.0, abs=1e-6)


def test_a_move_ends_one_sidedly_past_and_back():
    a = Axis("q1", 11.6, 1.8, 16, 1)
    b = Axis("q2", 8.7, 1.8, 16, -1)
    axes = {"q1": a, "q2": b}
    before = {"q1": 0.3, "q2": 1.0}
    end = {"q1": 0.2, "q2": 1.1}               # q1 came in negative, q2 positive
    g = host.one_sided(axes, before, end)
    assert len(g) == 2                         # q1 past, then back
    past = math.degrees(0.2) - host.APPROACH_DEG
    assert f"MOVE={host.position_deg(a, math.radians(past)):.5f}" in g[0]
    assert f"MOVE={host.position_deg(a, 0.2):.5f}" in g[1]
    assert host.one_sided(axes, {"q1": 0.1, "q2": 1.0}, end) == []    # both positive


def test_a_geared_axis_goes_past_its_backlash():
    a = Axis("q2", 20.0, 1.8, 16, 1, backlash_deg=0.8)          # a cheap planetary's
    g = host.one_sided({"q2": a}, {"q2": 1.0}, {"q2": 0.9})
    past = math.degrees(0.9) - 1.5 * 0.8
    assert f"MOVE={host.position_deg(a, math.radians(past)):.5f}" in g[0]


@pytest.mark.no_fabricate
def test_an_arm_buys_its_controller_and_a_walker_none():
    from spiderpig import roles
    from spiderpig.actuators import controllers
    from spiderpig.hardware.catalog import get

    assert roles.choices("controller") == ["skr_pico_pi_zero", roles.NONE]
    assert BuildConfig(linkage="palletizer").role("controller") == "skr_pico_pi_zero"
    assert BuildConfig(linkage="palletizer",
                       roles=(("controller", "none"),)).role("controller") == ""
    assert "controller" not in BuildConfig().applied_roles()
    ctl = controllers.get("skr_pico_pi_zero")
    total = 0.0
    for key, qty in ctl.bom:            # every row bought, priced and dated
        offer = get(key).offer
        assert offer is not None, key
        assert offer.price_usd is not None, key
        total += offer.buy(qty)[1] or 0.0
    assert dict(ctl.bom)["stepper_cable_1m"] == 3
    assert dict(ctl.bom)["diode_1n5819"] == 2
    assert 100.0 < total < 120.0        # the notes' ~110 (2026-10-13)
    assert [ctl.pins(n).port for n in ("q1", "q2", "q3", "q4")] == ["X", "Y", "Z", "E"]
    assert sorted(p.uart_address for _, p in ctl.axes) == [0, 1, 2, 3]
    with pytest.raises(ValueError, match="no motor port"):
        ctl.pins("q5")


@pytest.mark.no_fabricate
def test_a_stall_arms_klipper_config_homes_on_its_drivers_with_every_pin_filled():
    rng = (math.radians(-120.0), math.radians(120.0))
    ax = {"q2": Axis("q2", 5.0, 1.8, 16, 1, 0.0, "nema17_17hs4401", (0.7, 2.0)),
          "q3": Axis("q3", 5.0, 1.8, 16, -1, 0.0, "nema17_17hs4401", (-0.8, 0.35)),
          "q1": Axis("q1", 5.0, 1.8, 64, 1, 0.0, "nema17_17hs4401_x64", rng)}
    cfg = BuildConfig(linkage="palletizer", roles=(("turntable", "mt4_base_stall"),
                                                   ("angle_sensor", "stall")))
    sensors = homing.sensors_of(cfg, ax)
    assert sensors == {"q1": "hard_stop", "q2": "stall", "q3": "stall"}
    text = klipper.printer_cfg(ax, "palletizer", cfg.role("controller"), sensors)
    assert "<" not in text                                  # no placeholder left
    assert "serial: /dev/ttyAMA0" in text
    for n, addr, diag in (("q1", 0, "^gpio4"), ("q2", 2, "^gpio3"), ("q3", 1, "^gpio25")):
        sec = text.split(f"[tmc2209 manual_stepper {n}]")[1].split("\n[")[0]
        assert f"endstop_pin: tmc2209_{n}:virtual_endstop" in text
        assert f"uart_address: {addr}" in sec
        assert "tx_pin: gpio8" in sec
        assert "uart_pin: gpio9" in sec
        assert f"diag_pin: {diag}" in sec
        assert f"driver_SGTHRS: {klipper.SGTHRS}" in sec
    assert "pin: gpio23" in text
    assert "pin: gpio21" in text
    # the host: the turntable first, at a lowered current, onto the stop; no phase snap
    home = host.home_gcode(ax, sensors)
    assert home[0] == "MANUAL_STEPPER STEPPER=q1 ENABLE=1"
    run, _, low = klipper.currents(ax["q1"])
    assert f"SET_TMC_CURRENT STEPPER=q1 CURRENT={low:.2f}" in home
    assert f"SET_TMC_CURRENT STEPPER=q1 CURRENT={run:.2f}" in home
    assert sum("STOP_ON_ENDSTOP=1" in h for h in home) == 3
    assert not any("MSCNT" in h for h in home)
    assert host.homed(ax, sensors) == {n: a.limits[0] for n, a in ax.items()}
    steps = homing.plan(ax, sensors)
    assert not any("MSCNT" in s.action for s in steps)
    assert any(s.axis == "q1" and "virtual_endstop" in s.action for s in steps)
    # an endstop arm on the board: its switch on the port's input, no stall settings
    text = klipper.printer_cfg(ax, "palletizer", "skr_pico_pi_zero")
    assert "endstop_pin: ^gpio4" in text
    assert "virtual_endstop" not in text
    assert "SGTHRS" not in text
    assert "<" not in text
