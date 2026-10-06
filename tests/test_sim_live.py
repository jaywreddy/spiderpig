"""Tests for the live MuJoCo session the viewer drives (:mod:`sim.live`, ``/ws/sim``).

The websocket tests build models in this process (``SPIDERPIG_WORKERS=0``); the Klann
quad's model is seeded from the test cache with its steering check (``tests/_sim.py``:
what the server's build job hands over, made once per engine version), so a session
starts at once, and the server doesn't bake the default robot at startup (its tests are
``test_view.py``'s). The tests of the build itself stub the builder with that cached
output, and ``test_a_finished_build_is_forgotten_after_a_real_build`` (slow) runs the real
one. The server's own worker-process path is :mod:`spiderpig.workers`' and is covered by
the e2e tests.
"""

from __future__ import annotations

import logging
import math
import os
import struct
import threading
import time

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from fastapi.testclient import TestClient  # noqa: E402

from spiderpig.config import BuildConfig  # noqa: E402
from spiderpig.server import app as server  # noqa: E402
from spiderpig.server.app import app  # noqa: E402
from spiderpig.sim.live import HEADER, MAX_CATCH_UP, LiveSim  # noqa: E402
from spiderpig.sim.run import STEER_TILT, body_motions  # noqa: E402
from tests import _sim  # noqa: E402
from tests.tiers import quick  # noqa: E402

# The design these sessions step: the Klann quad (the sim's calibrated reference), named,
# since the server's default is the Strider double.
KLANN_QUAD_WS = "/ws/sim?linkage=klann&module=quad"
KLANN_QUAD = BuildConfig(linkage="klann", module="quad")


@pytest.fixture(autouse=True)
def _in_process(monkeypatch):
    monkeypatch.setenv("SPIDERPIG_WORKERS", "0")
    monkeypatch.setattr(server, "_PREBAKE_DEFAULT", False)
    _sim.seed(KLANN_QUAD, steering=True)


@pytest.fixture(scope="module")
def live():
    _sim.seed(KLANN_QUAD, steering=True)
    return LiveSim(KLANN_QUAD)


def _unpack(sim: LiveSim) -> np.ndarray:
    return np.frombuffer(sim.frame(), dtype="<f4")


def _hello(ws) -> dict:
    """Skip the ``building`` statuses; the hello."""
    while True:
        msg = ws.receive_json()
        if "hello" in msg:
            return msg["hello"]
        assert msg.get("status") == "building", msg
        assert "elapsed" in msg


def test_frame_is_the_documented_layout(live):
    live.reset()
    f = _unpack(live)
    assert f.size == HEADER + 12 * len(live.bodies)
    assert f[0] == 0.0
    np.testing.assert_allclose(f[1:4], live.data.body("base").xpos * 1e3, rtol=1e-5)
    np.testing.assert_allclose(f[4:8], live.data.body("base").xquat, rtol=1e-5)
    live.set_command(1.0, 1.0)
    for _ in range(4):
        live.advance(0.05)
    f, d = _unpack(live), body_motions(live.model, live.data, live.meta)
    np.testing.assert_allclose(f[8:10], live.data.actuator_length[live._act], rtol=1e-5)
    np.testing.assert_allclose(f[11:13], live.data.actuator_force[live._act], rtol=1e-4)
    assert np.abs(f[11:13]).max() > 0.01            # the drives work against the floor
    assert f[13] in (0.0, 1.0)                      # body down
    assert f[14] == pytest.approx(f[8] - f[9], abs=1e-6)       # side phase L - R
    assert abs(f[15]) < 200.0                       # vertical acceleration, m/s^2
    assert 0.0 <= f[16] < 1e3                       # the largest pin load, N
    for k, name in enumerate(live.bodies):
        np.testing.assert_allclose(f[HEADER + 12 * k: HEADER + 12 * (k + 1)],
                                   d[name][:3].ravel(), atol=1e-3)
    live.set_command(0.0, 0.0)


def test_the_session_locks_the_cranks_against_the_servo_mismatch(live):
    """Both drives commanded full ahead for two seconds: with the right servo 3 % slower
    (``SimParams.servo_mismatch``) the lock keeps the sides within a few degrees, where
    open loop they would be ~20 deg apart by then (and rolled over at 8 s)."""
    live.reset()
    assert live.lock.on
    assert live.lock.mismatch == pytest.approx(live.params.servo_mismatch)
    live.set_command(1.0, 1.0)
    for _ in range(40):
        live.advance(0.05)
    phase = abs(live.data.actuator_length[live._act[0]] - live.data.actuator_length[live._act[1]])
    assert math.degrees(phase) < 3.0, math.degrees(phase)
    assert live.upright()
    live.set_command(0.0, 0.0)
    live.reset()
    assert live.lock.ref.tolist() == [0.0, 0.0]     # the reset starts the lock over too


def test_hello_carries_the_servo_ratings_and_the_steering_check(live):
    """The viewer caps its commands to the steering check's verdict and reads the torque
    columns against the ratings. The verdict follows the runs (the Klann quad with its
    real servo model rolls over with a 0.4 differential while walking and tilts 23 deg
    turning in place: turn 0, spin 0; the parametric servo of the test suite weighs
    differently, so only the rule is asserted here)."""
    h = live.hello()
    assert h["header"] == HEADER
    assert h["torque_stall"] == pytest.approx(live.tau)
    assert h["torque_rated"] == live.rated
    assert h["torque_rated"] is None or 0 < h["torque_rated"] < h["torque_stall"]
    steer = h["steering"]
    assert set(steer) == {"turn", "spin", "step_deg", "forward", "seconds", "tests"}
    assert {"forward", "walk_turn", "spin"} <= set(steer["tests"])
    for name, safe in (("walk_turn", steer["turn"]), ("spin", steer["spin"])):
        t = steer["tests"][name]
        assert safe == (0.0 if t["fell"] or t["max_tilt"] >= STEER_TILT else
                        {"walk_turn": 0.4, "spin": 0.5}[name]), (name, t)
    assert steer["step_deg"] in (0.0, 45.0, 90.0)
    assert h["forward"] is steer["forward"]
    assert set(h["forward"]) >= {"fell", "max_tilt", "side_support_low", "speed", "walks",
                                 "body_contact"}
    assert not h["forward"]["fell"]
    assert h["forward"]["walks"]
    assert h["kinematic_stride_mm"] > 100
    assert h["mass"] == pytest.approx(live.meta["mass"]["total"])
    assert h["payload_g"] == live.params.payload_g
    assert h["phase_lock"]["step_deg"] == steer["step_deg"]
    assert live.lock.max_offset == pytest.approx(math.radians(steer["step_deg"]))
    assert live.hello()["steering"] is steer       # computed once per model


@pytest.mark.slow
def test_hello_runs_the_steering_check_for_a_model_without_one():
    """A model that comes without the verdict (one built here, not by the server's job):
    the session runs the check once, and it is the verdict the cache holds (what the other
    tests' seeded model carries)."""
    model, meta = _sim.seed(KLANN_QUAD)
    sim = LiveSim(KLANN_QUAD, model=model,
                  meta={k: v for k, v in meta.items() if k != "steering"})
    assert math.isinf(sim.lock.max_offset)
    steer = sim.hello()["steering"]
    assert _json(steer) == _sim.steering_of(KLANN_QUAD)
    assert sim.lock.max_offset == pytest.approx(math.radians(steer["step_deg"]))
    assert sim.hello()["steering"] is steer


def test_a_requested_reset_lands_on_the_next_advance(live):
    """What the websocket reader does from the event loop: ask, never touch MjData."""
    live.reset()
    live.set_command(1.0, 1.0)
    for _ in range(5):
        live.advance(MAX_CATCH_UP)                  # the most one call steps
    assert live.data.time > 4 * MAX_CATCH_UP
    live.request_reset()
    assert live.data.time > 4 * MAX_CATCH_UP        # nothing happened yet
    live.advance(1 / 60)
    assert live.data.time == pytest.approx(1 / 60, abs=live.model.opt.timestep)
    live.set_command(0.0, 0.0)


def test_resets_and_commands_from_another_thread_while_stepping(live):
    """Resets and commands hammered from a second thread while the physics steps: every
    frame stays finite and the clock never runs backwards within a tick (the reader's
    reset waits for the step: the pattern without the lock crashed the process)."""
    live.reset()
    stop = threading.Event()
    n_reset = [0]

    def hammer():
        while not stop.is_set():
            live.request_reset()
            live.set_command(1.0, -1.0 if n_reset[0] % 2 else 1.0)
            live.reset()                            # a direct reset takes the lock too
            n_reset[0] += 1

    th = threading.Thread(target=hammer, daemon=True)
    th.start()
    t0, bad = time.perf_counter(), 0
    while time.perf_counter() - t0 < 1.5:
        live.advance(1 / 60)
        f = _unpack(live)
        bad += not np.isfinite(f).all()
    stop.set()
    th.join(5)
    assert n_reset[0] > 10
    assert bad == 0
    assert live.data.time < 0.5                     # a reset landed recently
    live.set_command(0.0, 0.0)
    live.reset()


def test_commands_walk_it_forward_and_reset_brings_it_back(live):
    live.reset()
    for _ in range(6):
        live.advance(0.05)                  # settle 0.3 s
    x0 = live.data.body("base").xpos[0]
    live.set_command(1.0, 1.0)
    down = []
    for _ in range(40):
        live.advance(0.05)                  # 2 s
        down.append(_unpack(live)[10])      # feet down
    assert live.data.body("base").xpos[0] - x0 > 0.15
    # most of the time on two feet or more (one frame's count bounces: the demo Klann quad's
    # 31-layer stack reads 0-4 frame to frame)
    assert np.median(down) >= 2, down
    assert live.upright()
    live.set_command(0.0, 0.0)
    live.reset()
    assert live.data.time == 0.0
    assert abs(live.data.body("base").xpos[0]) < 1e-6


def test_a_stalled_tick_catches_up_at_most_max(live):
    """A tick that comes late steps at most three frames' worth of physics (a stalled
    server runs in slow motion instead of bursting 100 ms into one frame)."""
    live.reset()
    live.advance(10.0)
    assert live.data.time == pytest.approx(MAX_CATCH_UP, abs=live.model.opt.timestep)
    assert MAX_CATCH_UP <= 3 / 60 + 1e-9


def test_advance_carries_the_fractional_timestep(live):
    """60 ticks of 1/60 s are 1.000 s of physics, not 60 x round(16.7 ms / 1 ms)."""
    live.reset()
    for _ in range(60):
        live.advance(1 / 60)
    assert live.data.time == pytest.approx(1.0, abs=live.model.opt.timestep)


def test_set_command_rejects_what_would_freeze_the_solver(live):
    for bad in ((math.nan, 1.0), (1e308, math.inf), ("a", "b"), (1.0,), (1, 2, 3)):
        with pytest.raises((ValueError, TypeError)):
            live.set_command(*bad)
    live.set_command(5.0, -5.0)             # clipped to the servo's speed
    assert live.command.tolist() == [live.vmax, -live.vmax]
    live.set_command(0.0, 0.0)


def test_the_drives_hold_the_servo_speed_torque_line(live):
    """At full command the loop can't deliver more than the motor at that speed:
    motoring torque stays on the line ``stall * (1 - |w| / w0)`` or under it (the clamp
    is set from the speed one step earlier: a step's change of speed is the slack)."""
    live.reset()
    live.set_command(1.0, 1.0)
    live.advance(0.1)
    over = 0.0
    for _ in range(100):
        live.advance(1 / 60)
        w = live.data.actuator_velocity[live._act]
        tq = live.data.actuator_force[live._act]
        avail = live.tau * np.clip(1.0 - np.abs(w) / live.vmax, 0.0, None)
        motoring = tq * w > 0
        if motoring.any():
            over = max(over, float((np.abs(tq[motoring]) - avail[motoring]).max()))
    assert over < 0.05 * live.tau, f"{over:.3f} N·m over the motor line"
    live.set_command(0.0, 0.0)


def test_ws_sim_says_hello_then_streams_frames():
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        hello = _hello(ws)
        assert hello["header"] == HEADER
        assert hello["steering"]["spin"] in (0.0, 0.5)   # the server fills the check in
        assert not hello["forward"]["fell"]
        assert hello["torque_stall"] > 0
        assert "base" in hello["bodies"]
        assert set(hello["nodes"].values()) <= set(hello["bodies"])
        assert hello["design"] == {"linkage": "klann", "module": "quad",
                                   "key": BuildConfig(linkage="klann", module="quad").key}
        assert hello["crank_sign"] in (-1, 1)
        ws.send_json({"cmd": [1.0, 1.0]})
        frames = [np.frombuffer(ws.receive_bytes(), dtype="<f4") for _ in range(10)]
        assert all(f.size == HEADER + 12 * len(hello["bodies"]) for f in frames)
        assert frames[-1][0] > frames[0][0]


def test_ws_sim_reports_a_bad_design():
    with TestClient(app) as client, client.websocket_connect("/ws/sim?linkage=nope") as ws:
        assert "error" in ws.receive_json()


def _frame(ws) -> np.ndarray:
    """The next binary frame (a JSON message in between is a failure, shown)."""
    msg = ws.receive()
    assert "bytes" in msg, msg
    return np.frombuffer(msg["bytes"], dtype="<f4")


def _drain_to_json(ws, limit: int = 30) -> dict:
    """The next JSON message among the frames."""
    for _ in range(limit):
        msg = ws.receive()
        if msg.get("text"):
            import json
            return json.loads(msg["text"])
    raise AssertionError("no JSON answer among the frames")


@pytest.fixture
def sessions(monkeypatch):
    """Every :class:`LiveSim` the server makes while the test runs."""
    from spiderpig.sim import live as live_mod

    made = []

    class Recorded(live_mod.LiveSim):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            made.append(self)

    monkeypatch.setattr(live_mod, "LiveSim", Recorded)
    return made


def test_ws_sim_answers_a_bad_command_and_keeps_streaming(sessions):
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        _hello(ws)
        for bad in ('{"cmd": [1, 2, 3]}', '{"cmd": ["a", "b"]}', '{"cmd": [NaN, 1e308]}',
                    "hello?", "[1, 2]"):
            ws.send_text(bad)
            assert "error" in _drain_to_json(ws), bad
        ws.send_json({"cmd": [1.0, 1.0]})
        t0 = np.frombuffer(ws.receive_bytes(), dtype="<f4")[0]
        for _ in range(20):
            f = np.frombuffer(ws.receive_bytes(), dtype="<f4")
        assert f[0] > t0                    # still stepping: the NaN never reached ctrl
        assert np.isfinite(f).all()
        (sim,) = sessions
        assert np.isfinite(sim.data.ctrl).all()
        assert sim.data.warning.number.sum() == 0   # nothing for MUJOCO_LOG.TXT


def test_ws_sim_resets_on_request_while_streaming():
    """``{"reset": true}`` hammered between frames: the clock goes back to zero each time and
    every frame stays finite (the reset is applied by the stepping thread, not under it)."""
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        _hello(ws)
        ws.send_json({"cmd": [1.0, 1.0]})
        resets, ts = 0, []
        for k in range(90):
            f = _frame(ws)
            assert np.isfinite(f).all()
            ts.append(float(f[0]))
            if k % 3 == 0:
                ws.send_json({"reset": True})
                resets += 1
        assert resets == 30
        assert max(ts) < 0.5                        # never got far from zero
        assert sum(b < a for a, b in zip(ts, ts[1:], strict=False)) >= 10   # resets landed


def test_ws_sim_reports_a_physics_failure_and_closes(monkeypatch):
    """An exception inside the session (here: the step) ends that session with
    ``{"error"}`` and close 1011 instead of escaping the handler."""
    from spiderpig.sim import live as live_mod

    class Broken(live_mod.LiveSim):
        def advance(self, seconds):
            raise mujoco.FatalError("Nan, Inf or huge value in QACC")

    monkeypatch.setattr(live_mod, "LiveSim", Broken)
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        _hello(ws)
        msg = _drain_to_json(ws)
        assert "QACC" in msg.get("error", ""), msg


def _machine_is_busy() -> bool:
    """A loaded machine (the 1-minute load over half the cores) can't promise a cadence."""
    return os.getloadavg()[0] > (os.cpu_count() or 1) / 2


def test_ws_sim_streams_at_the_promised_cadence():
    """The sim clock is consistent in every frame: strictly increasing, no tick longer than
    :data:`MAX_CATCH_UP`. On an idle machine besides: at least 55 frames a second and the
    sim clock within 2 % of the wall clock over 3 s (a wall-clock promise: not asserted
    when the machine is loaded, where the session runs in slow motion by design)."""
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        _hello(ws)
        ws.send_json({"cmd": [1.0, 1.0]})
        first = np.frombuffer(ws.receive_bytes(), dtype="<f4")
        wall0, sim0 = time.perf_counter(), first[0]
        n, last, ts = 0, first, [float(first[0])]
        while time.perf_counter() - wall0 < 3.0:
            last = np.frombuffer(ws.receive_bytes(), dtype="<f4")
            ts.append(float(last[0]))
            n += 1
        wall = time.perf_counter() - wall0
        steps = np.diff(ts)
        assert (steps > 0).all()
        assert steps.max() <= MAX_CATCH_UP + 1e-6
        if _machine_is_busy():
            pytest.skip(f"load {os.getloadavg()[0]:.1f}: the cadence isn't the server's to keep "
                        f"({n / wall:.1f} fps, sim/wall {(last[0] - sim0) / wall:.2f})")
        assert n / wall >= 55, f"{n / wall:.1f} fps"
        assert (last[0] - sim0) / wall == pytest.approx(1.0, abs=0.02)


def test_two_sessions_stream_at_once():
    with TestClient(app) as client, \
            client.websocket_connect(KLANN_QUAD_WS) as a, \
            client.websocket_connect(KLANN_QUAD_WS) as b:
        _hello(a)
        _hello(b)
        a.send_json({"cmd": [1.0, 1.0]})
        fa = [_frame(a) for _ in range(30)]
        fb = [_frame(b) for _ in range(30)]
        assert fa[-1][0] > fa[0][0]
        assert fb[-1][0] > fb[0][0]
        assert fa[-1][1] > fb[-1][1]        # only a was told to walk


def test_a_client_leaving_during_the_build_logs_no_traceback(monkeypatch, caplog):
    """The hello is never sent to a closed socket and nothing is logged as an error;
    the build itself finishes for the next client."""
    from spiderpig.sim import mjcf

    xml, meta = _sim.job_output(KLANN_QUAD)        # what the in-process build returns
    started, finished = threading.Event(), threading.Event()

    def slow_build(config):
        started.set()
        time.sleep(1.5)
        finished.set()
        return xml, meta

    monkeypatch.setattr(server, "_build_mjcf_in_process", slow_build)
    monkeypatch.setattr(mjcf, "cached_model", lambda *a: None)
    server._SIM_BUILDS.clear()
    caplog.set_level(logging.INFO)
    with TestClient(app) as client:
        with client.websocket_connect(KLANN_QUAD_WS + "&phases=0,180,90,271") as ws:
            assert ws.receive_json()["status"] == "building"
            assert started.wait(5)
        finished.wait(5)
        time.sleep(0.2)
    assert not [r for r in caplog.records if r.levelno >= logging.ERROR], caplog.text
    assert "Traceback" not in caplog.text
    assert any("client left while building" in r.getMessage() for r in caplog.records)
    server._SIM_BUILDS.clear()


def _slow_builder(monkeypatch, started: threading.Event, release: threading.Event, build):
    """The in-process model builder replaced by one that waits for ``release`` (``build()``
    gives the (xml, meta) it returns), with the compiled-model cache emptied so the server
    has to build."""
    from spiderpig.sim import mjcf

    def slow_build(config):
        started.set()
        release.wait(30)
        return build()

    monkeypatch.setattr(server, "_build_mjcf_in_process", slow_build)
    monkeypatch.setattr(mjcf, "cached_model", lambda *a: None)
    server._SIM_BUILDS.clear()


@pytest.fixture
def quad_mjcf():
    """What the in-process build of the Klann quad returns (the MJCF, its meta and the
    steering check), from the cache."""
    return _sim.job_output(KLANN_QUAD)


def test_a_command_and_a_reset_during_the_build_still_yield_the_hello(monkeypatch, quad_mjcf,
                                                                      sessions):
    """A message while the model builds used to cancel the waiter and kill the session
    with a CancelledError before the hello; now it is kept and applied before the hello,
    and a malformed one is answered after it."""
    xml, meta = quad_mjcf
    started, release = threading.Event(), threading.Event()
    _slow_builder(monkeypatch, started, release, lambda: (xml, dict(meta)))
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        assert ws.receive_json()["status"] == "building"
        assert started.wait(5)
        ws.send_json({"cmd": [1.0, 1.0]})
        ws.send_json({"reset": True})
        ws.send_text('{"cmd": ["a", "b"]}')
        release.set()
        hello = _hello(ws)
        assert "base" in hello["bodies"]
        assert "error" in _drain_to_json(ws)          # the malformed one, after the hello
        f = _frame(ws)
        assert np.isfinite(f).all()
        (sim,) = sessions
        assert sim.command.tolist() == [sim.vmax, sim.vmax]   # the early command landed
    server._SIM_BUILDS.clear()


def test_a_source_change_during_the_build_serves_no_stale_model(monkeypatch, quad_mjcf,
                                                                 sessions):
    """The watcher fires (caches cleared, generation bumped) while a client's model builds:
    that build's result is dropped and built again, so the client's session and the
    cached model are the new sources', and the next client gets them too."""
    from spiderpig.sim import mjcf

    xml, meta = quad_mjcf
    started, release = threading.Event(), threading.Event()
    marker = ["OLD-SOURCES"]
    real_cached = mjcf.cached_model
    _slow_builder(monkeypatch, started, release, lambda: (xml, {**meta, "marker": marker[0]}))
    gen0 = server._SIM_GENERATION
    with TestClient(app) as client:
        with client.websocket_connect(KLANN_QUAD_WS) as ws:
            assert ws.receive_json()["status"] == "building"
            assert started.wait(5)
            # the watcher: what _rebake_all does to the sim state
            mjcf.clear_caches()
            server._SIM_BUILDS.clear()
            server._SIM_GENERATION += 1
            marker[0] = "NEW-SOURCES"
            release.set()
            hello = _hello(ws)
            assert "base" in hello["bodies"]
            assert _drain_to_json(ws).get("status") == "stale"   # its session predates the bump
        (sim,) = sessions
        assert sim.meta["marker"] == "NEW-SOURCES"
        assert server._SIM_BUILDS == {}
        assert gen0 + 1 == server._SIM_GENERATION
        monkeypatch.setattr(mjcf, "cached_model", real_cached)     # the cache answers again
        cached = mjcf.cached_model(BuildConfig(linkage="klann", module="quad"))
        assert cached[1]["marker"] == "NEW-SOURCES"
        with client.websocket_connect(KLANN_QUAD_WS) as ws:
            _hello(ws)
            assert sessions[-1].meta["marker"] == "NEW-SOURCES"
    mjcf.clear_caches()
    server._SIM_BUILDS.clear()


def test_a_source_change_while_streaming_says_stale_and_closes():
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        _hello(ws)
        _frame(ws)
        server._SIM_GENERATION += 1
        msg = _drain_to_json(ws)
        assert msg == {"status": "stale"}
        closed = ws.receive()
        assert closed.get("type") == "websocket.close", closed


def test_a_busy_server_refuses_the_next_session(monkeypatch):
    monkeypatch.setattr(server, "SIM_MAX_SESSIONS", 1)
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as a:
        _hello(a)
        with client.websocket_connect(KLANN_QUAD_WS) as b:
            msg = b.receive_json()
            assert "busy" in msg.get("error", ""), msg
        _frame(a)                                   # the first session streams on
    assert server._SIM_SESSIONS == 0


def _json(doc):
    import json

    return json.loads(json.dumps(doc))


def _forgotten_after_a_build(cfg):
    from spiderpig.sim import mjcf

    server._SIM_BUILDS.clear()
    with TestClient(app) as client, client.websocket_connect(KLANN_QUAD_WS) as ws:
        hello = _hello(ws)
        assert mjcf.cached_model(cfg) is not None
        assert server._SIM_BUILDS == {}
    return hello


def test_a_finished_build_is_forgotten(monkeypatch):
    """The shared build future is dropped once adopted into the model cache (it held the
    MJCF text and metadata for every design ever built otherwise). The model isn't cached
    yet, so the server builds it (the in-process builder: here its cached output)."""
    from spiderpig.sim import mjcf

    built = []

    def build(config):
        built.append(config)
        return _sim.job_output(config)

    monkeypatch.setattr(server, "_build_mjcf_in_process", build)
    mjcf.clear_caches()
    try:
        _forgotten_after_a_build(KLANN_QUAD)
    finally:
        mjcf.clear_caches()
    assert built == [KLANN_QUAD]


@pytest.mark.slow
def test_a_finished_build_is_forgotten_after_a_real_build():
    """The same through the real in-process build (``build_mjcf`` and the steering check,
    from the cached robot rather than a fabrication): the hello carries the very model and
    verdict the cache holds."""
    from spiderpig.sim import mjcf

    mjcf.clear_caches()
    try:
        mjcf.set_fabricated(KLANN_QUAD, _sim.robot_at_ref(KLANN_QUAD))
        hello = _forgotten_after_a_build(KLANN_QUAD)
        _, meta = mjcf.cached_model(KLANN_QUAD)
        xml, cached = _sim.mjcf_of(KLANN_QUAD)
        assert mjcf.build_mjcf(KLANN_QUAD)[0] == xml
        assert _json({k: v for k, v in meta.items() if k not in ("steering", "kinematic")}) \
            == cached
        assert _json(hello["steering"]) == _sim.steering_of(KLANN_QUAD)
    finally:
        mjcf.clear_caches()


# ---------------------------------------------------------------------------
# The glb and the model of one design name the same nodes
# ---------------------------------------------------------------------------


def _glb_node_names(path) -> set[str]:
    """Every node name in a glb (its JSON chunk)."""
    import json

    blob = path.read_bytes()
    assert blob[:4] == b"glTF"
    length, kind = struct.unpack_from("<II", blob, 12)
    assert kind == 0x4E4F534A           # JSON
    doc = json.loads(blob[20:20 + length])
    return {n["name"] for n in doc["nodes"] if n.get("name") and n["name"] != "walker"}


@pytest.mark.parametrize(("linkage", "module"), quick(
    [("klann", "quad"), ("klann", "double"), ("strider", "quad")], keep=[("klann", "quad")]))
def test_the_glb_bake_and_the_model_name_the_same_nodes(linkage, module):
    """What ``PhysicsLink.bind`` relies on: every node of the baked robot is in the
    model's ``nodes`` map, and nothing else is (both from the cache: the bake and the model
    of the cached fabrication, which are a fresh one's, ``test_bake_gltf.py`` and
    ``test_sim.py``'s ``test_recorded_mjcf_current``)."""
    # the Strider quad keyed: with the bolt crank it doesn't plan within the default budget
    old = {"crank": "keyed", "pillar": "printed"} if linkage == "strider" else {}
    cfg = BuildConfig(linkage=linkage, module=module, **old)
    path, _ = _sim.baked(cfg, n_frames=4)
    _, meta = _sim.seed(cfg)
    assert _glb_node_names(path) == set(meta["nodes"])
