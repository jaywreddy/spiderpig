"""The live session (``/ws/sim``, :mod:`sim.live`) of the designs that don't walk (2026-10-14):
an arm's steppers told their joints' angles, a mechanism's crank turned, each by what its
model declares (its actuators, its sites, no free base), never by its kind's name.

The models are the recorded MJCFs (``tests/fixtures/sim/``: no fabrication), adopted as
the server's model of their design (``mjcf.adopt_mjcf``), so a session starts at once; the
websocket runs through the in-process test client and through a real server on a free port
(uvicorn and the ``websockets`` client). The walker's session is ``test_sim_live.py``'s.
"""

from __future__ import annotations

import json
import math
import threading
import time
from contextlib import contextmanager

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from fastapi.testclient import TestClient  # noqa: E402

from spiderpig.config import BuildConfig  # noqa: E402
from spiderpig.server import app as server  # noqa: E402
from spiderpig.sim import mjcf  # noqa: E402
from spiderpig.sim.live import BASE_COLUMNS, LiveSim  # noqa: E402
from spiderpig.sim.run import body_motions  # noqa: E402
from tests import _sim  # noqa: E402

ARM = BuildConfig(linkage="palletizer_reach")       # its robot on its turntable
MECH = BuildConfig(linkage="hoecken_pantograph")    # its one side
RECORDED = {ARM: "palletizer_reach", MECH: "hoecken_pantograph"}


def _live(cfg: BuildConfig) -> LiveSim:
    xml, meta = _sim.recorded_mjcf(RECORDED[cfg])
    return LiveSim(cfg, model=mujoco.MjModel.from_xml_string(xml),
                   meta=json.loads(json.dumps(meta)))


@pytest.fixture(scope="module")
def arm() -> LiveSim:
    return _live(ARM)


@pytest.fixture(scope="module")
def mech() -> LiveSim:
    return _live(MECH)


def _values(sim: LiveSim) -> dict[str, float]:
    f = np.frombuffer(sim.frame(), dtype="<f4")
    assert f.size == sim.header + 12 * len(sim.bodies)
    return dict(zip(sim.columns, f[:sim.header].tolist(), strict=True))


def _run(sim: LiveSim, seconds: float) -> None:
    for _ in range(round(seconds * 60)):
        sim.advance(1 / 60)


# ---------------------------------------------------------------------------
# The session, by what the model declares
# ---------------------------------------------------------------------------


def test_an_arms_hello_declares_its_joints_and_no_crank(arm):
    """An arm's model: steppers on its ranged inputs and turntable (the hello's ``joints``,
    each one's range and start in degrees), no crank, no feet, a fixed base; its frame
    streams each joint's angle and torque and its sites (the tool's tip, the output); no
    walker's steering check is run for it."""
    h = arm.hello()
    assert h["walks"] is False
    assert h["feet"] == 0
    assert h["drives"] == []
    assert set(ARM.lk.inputs) <= set(h["joints"])
    assert "q1" in h["joints"]                          # the turntable
    for q, j in h["joints"].items():
        assert j["lo"] <= j["value"] <= j["hi"], q
        assert j["effort"] > 0
    assert set(h["sites"]) == {"tool0", "output"}
    assert "steering" not in h
    assert "phase_lock" not in h
    assert h["header"] == len(h["columns"])
    assert h["columns"][:len(BASE_COLUMNS)] == list(BASE_COLUMNS)
    assert {"q2.angle", "q2.torque", "tool0.x", "tool0.z", "pin_load"} <= set(h["columns"])
    assert not {"feet_down", "az", "side_phase"} & set(h["columns"])
    assert h["design"]["linkage"] == ARM.linkage
    assert h["robot"] is True
    json.dumps(h, allow_nan=False)                      # what the websocket sends


def test_an_arm_follows_its_joint_targets(arm):
    """``set_joints`` (degrees, any subset) moves the arm there under its steppers; the
    frame reads the joints' angles and the tool's tip in world mm; the base never moves.
    Out of range is clipped to the joint's range; anything else malformed is refused."""
    arm.reset()
    v0 = _values(arm)
    np.testing.assert_allclose(v0["q2.angle"], math.radians(arm.hello()["joints"]["q2"]["value"]),
                               atol=1e-6)
    arm.set_joints({"q2": 60.0, "q1": 30.0})
    _run(arm, 1.5)
    v = _values(arm)
    assert math.degrees(v["q2.angle"]) == pytest.approx(60.0, abs=0.5)
    assert math.degrees(v["q1.angle"]) == pytest.approx(30.0, abs=0.5)
    assert math.degrees(v["q3.angle"]) == pytest.approx(
        arm.hello()["joints"]["q3"]["value"], abs=0.5)               # untouched: held
    moved = math.dist([v0[f"tool0.{a}"] for a in "xyz"], [v[f"tool0.{a}"] for a in "xyz"])
    assert moved > 20.0
    np.testing.assert_allclose([v[f"tool0.{a}"] for a in "xyz"],
                               arm.data.site("tool0").xpos * 1e3, atol=1e-2)   # world mm
    for c in ("base.x", "base.y", "base.z", "base.qw"):
        assert v[c] == pytest.approx(v0[c], abs=1e-4)               # fixed to the world
    d = body_motions(arm.model, arm.data, arm.meta)
    f = np.frombuffer(arm.frame(), dtype="<f4")
    for k, name in enumerate(arm.bodies):
        np.testing.assert_allclose(f[arm.header + 12 * k: arm.header + 12 * (k + 1)],
                                   d[name][:3].ravel(), atol=1e-3)
    arm.set_joints({"q2": 1e3})
    k = arm.joints.index("q2")
    assert arm.targets[k] == pytest.approx(arm.ranges[k, 1])       # clipped to its range
    for bad in ({"q9": 1.0}, {"q2": math.nan}, {"q2": "a"}, {"q2": True}, {}, [1, 2]):
        with pytest.raises(ValueError, match="joint"):
            arm.set_joints(bad)                                     # type: ignore[arg-type]
    with pytest.raises(ValueError, match="no crank"):
        arm.set_command(1.0)
    arm.set_joints({q: j["value"] for q, j in arm.hello()["joints"].items()})


def test_a_reset_puts_the_arm_back_and_its_targets_stand(arm):
    """A reset is qpos0 again (the clock at zero); the joints' targets are commands and
    stand, so the arm goes back to them."""
    arm.set_joints({"q2": 50.0})
    _run(arm, 1.0)
    arm.request_reset()
    arm.advance(1 / 60)
    assert arm.data.time == pytest.approx(1 / 60, abs=arm.model.opt.timestep)
    _run(arm, 1.5)
    assert math.degrees(_values(arm)["q2.angle"]) == pytest.approx(50.0, abs=0.5)
    arm.set_joints({q: j["value"] for q, j in arm.hello()["joints"].items()})


def test_a_mechanisms_live_crank_pushes_no_harder_than_its_set_limit(mech):
    """The live session caps a fixed design's crank servo at the torque limit it is set to
    (``spiderpig control servo`` writes it; the strength rates the joints there), as
    ``sim.run.turn`` and ``hold`` do (review round 3); the hello says so."""
    from spiderpig.config import torque_limit_nm

    h = mech.hello()
    assert h["torque_limit"] == pytest.approx(torque_limit_nm(MECH))
    assert h["torque_limit"] < h["torque_stall"]
    mech.set_command(1.0)
    _run(mech, 0.5)
    act = mech._act
    assert float(np.max(mech.model.actuator_forcerange[act, 1])) <= h["torque_limit"] + 1e-9
    mech.set_command(0.0)


def test_a_mechanisms_crank_turns_and_holds(mech):
    """A mechanism's one crank: its servo's speed is the command (``[speed]``, a fraction of
    the no-load speed), on its speed-torque line; at zero the servo holds the crank where it
    is (position mode: no creep under gravity); its output point streams in world mm."""
    h = mech.hello()
    assert h["walks"] is False
    assert h["feet"] == 0
    assert h["drives"] == ["drive"]
    assert h["joints"] == {}
    assert mech.lock is None
    assert "phase_lock" not in h
    assert "steering" not in h
    assert h["torque_stall"] > 0
    assert h["vmax"] > 0
    assert {"drive.angle", "drive.torque", "output.x", "pin_load"} <= set(h["columns"])
    assert h["robot"] is False
    mech.reset()
    _run(mech, 1.0)
    held = _values(mech)
    assert abs(held["drive.angle"]) < 0.01                      # rad: no creep
    out0 = [held[f"output.{a}"] for a in "xyz"]
    mech.set_command(1.0)
    _run(mech, 2.0)
    turned = _values(mech)
    assert turned["drive.angle"] > 0.6 * mech.vmax * 2.0 - 1.0  # most of the no-load speed
    assert 0 < abs(turned["drive.torque"]) <= mech.tau
    mech.set_command(-0.5)
    _run(mech, 0.5)
    assert _values(mech)["drive.angle"] < turned["drive.angle"]   # backwards too
    mech.set_command(0.0)
    _run(mech, 0.5)
    stopped = _values(mech)["drive.angle"]
    _run(mech, 1.0)
    assert abs(_values(mech)["drive.angle"] - stopped) < 0.01   # stopped, held
    assert math.dist(out0, [_values(mech)[f"output.{a}"] for a in "xyz"]) > 0.0
    for bad in ((1.0, 1.0), (), (math.nan,), ("a",)):
        with pytest.raises(ValueError, match="cmd must be 1 finite number"):
            mech.set_command(*bad)
    with pytest.raises(ValueError, match="no stepped joint"):
        mech.set_joints({"q2": 1.0})
    assert np.isfinite(mech.data.qacc).all()


# ---------------------------------------------------------------------------
# Through the websocket
# ---------------------------------------------------------------------------


@pytest.fixture
def adopted(monkeypatch):
    """The recorded models as the server's models of their designs (forgotten after)."""
    monkeypatch.setenv("SPIDERPIG_WORKERS", "0")
    monkeypatch.setattr(server, "_PREBAKE_DEFAULT", False)
    keys = []
    for cfg, name in RECORDED.items():
        xml, meta = _sim.recorded_mjcf(name)
        key = mjcf.model_config(cfg)
        if mjcf.cached_model(key) is None:
            mjcf.adopt_mjcf(key, mjcf.SimParams(), xml, json.loads(json.dumps(meta)))
            keys.append(key)
    yield
    for key in keys:
        mjcf._MODELS.pop((key, mjcf.SimParams()), None)


def _hello(receive_json) -> dict:
    while True:
        msg = receive_json()
        if "hello" in msg:
            return msg["hello"]
        assert msg.get("status") == "building", msg


def _columns(hello: dict, frame: bytes) -> dict[str, float]:
    f = np.frombuffer(frame, dtype="<f4")
    assert f.size == hello["header"] + 12 * len(hello["bodies"])
    assert np.isfinite(f).all()
    return dict(zip(hello["columns"], f[:hello["header"]].tolist(), strict=True))


def _drive(hello, send, receive) -> None:
    """Drive the design the hello describes by what it declares, and check it moved: an
    arm's joints to a target, a mechanism's crank turned; then a command it has no drive
    for is answered with an error and the session streams on."""
    frames = []

    def frame() -> dict[str, float]:
        while True:
            msg = receive()
            if isinstance(msg, bytes):
                frames.append(msg)
                return _columns(hello, msg)
            assert "error" not in json.loads(msg), msg

    if hello["joints"]:
        q = sorted(hello["joints"])[-1]
        j = hello["joints"][q]
        target = 0.5 * (j["lo"] + j["hi"])
        send({"joints": {q: target}})
        for _ in range(300):
            if abs(math.degrees(frame()[f"{q}.angle"]) - target) < 1.0:
                break
        else:
            raise AssertionError(f"{q} never reached {target} deg")
        send({"cmd": [1.0]})                    # no crank: refused, the session goes on
    else:
        send({"cmd": [1.0] * len(hello["drives"])})
        a0 = frame()[f"{hello['drives'][0]}.angle"]
        for _ in range(120):
            if frame()[f"{hello['drives'][0]}.angle"] > a0 + 1.0:
                break
        else:
            raise AssertionError("the crank never turned")
        send({"joints": {"q2": 10}})            # no stepped joint: refused
    for _ in range(60):
        msg = receive()
        if isinstance(msg, str):
            assert "error" in json.loads(msg)
            break
    else:
        raise AssertionError("no error for a command the design has no drive for")
    t0 = frame()["t"]
    assert frame()["t"] > t0                    # still stepping


@pytest.mark.parametrize("cfg", [ARM, MECH], ids=["arm", "mechanism"])
def test_ws_sim_steps_a_fixed_design_live(adopted, cfg):
    """The design from the query, its robot or side its own (an arm on its turret, a
    mechanism's one side: before, the session forced a robot and a mechanism had none);
    the hello says what drives it, and it is driven by that."""
    client = TestClient(server.app, base_url="http://localhost")
    with client, client.websocket_connect(
            f"ws://localhost/ws/sim?linkage={cfg.linkage}") as ws:
        hello = _hello(ws.receive_json)
        assert hello["walks"] is False
        assert hello["design"]["key"] == cfg.key
        assert set(hello["nodes"].values()) <= set(hello["bodies"])

        def receive():
            msg = ws.receive()
            return msg["bytes"] if msg.get("bytes") is not None else msg["text"]

        _drive(hello, ws.send_json, receive)


@contextmanager
def _served():
    """The app on a free port (uvicorn in a thread): its port."""
    import socket

    import uvicorn

    s = socket.socket()                     # bound here and handed over: no other process
    s.bind(("127.0.0.1", 0))                # can take the port between the two (xdist)
    port = s.getsockname()[1]
    srv = uvicorn.Server(uvicorn.Config(server.app, host="127.0.0.1", port=port,
                                        lifespan="off", log_level="warning"))
    thread = threading.Thread(target=srv.run, kwargs={"sockets": [s]}, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 20
        while not srv.started:
            assert time.monotonic() < deadline, "the server didn't start"
            time.sleep(0.05)
        yield port
    finally:
        srv.should_exit = True
        thread.join(10)
        s.close()


def test_a_real_server_streams_an_arm_and_a_mechanism(adopted):
    """The same over a real socket (uvicorn on a free port, the ``websockets`` client): an
    arm's joints and a mechanism's crank driven live, one session after the other, and
    ``mode=robot`` asks for the arm on its turntable explicitly."""
    from websockets.sync.client import connect

    with _served() as port:
        for query in (f"linkage={ARM.linkage}&mode=robot", f"linkage={MECH.linkage}"):
            with connect(f"ws://127.0.0.1:{port}/ws/sim?{query}", open_timeout=20) as ws:
                hello = _hello(lambda: json.loads(ws.recv(timeout=60)))
                _drive(hello, lambda m: ws.send(json.dumps(m)), lambda: ws.recv(timeout=10))


@pytest.mark.no_fabricate
def test_the_session_takes_the_design_its_view_and_its_robot_from_the_query():
    """``/ws/sim``'s config: ``mode`` (the view on screen) picks the robot or one side as
    ``/api/glb``'s does; without it, the design's own (the linkage's default: an arm's
    robot, a mechanism's side); a walker's model is its robot whatever is asked
    (``mjcf.model_config``). ``/api/linkages`` declares the capabilities the viewer offers
    the live session by (``walks``, ``ranged``)."""
    from starlette.datastructures import QueryParams

    def cfg(q: str) -> BuildConfig:
        return server._sim_config(QueryParams(q))

    assert cfg(f"linkage={ARM.linkage}").robot is True
    assert cfg(f"linkage={ARM.linkage}&mode=side").robot is False
    assert cfg(f"linkage={MECH.linkage}").robot is False
    assert cfg(f"linkage={MECH.linkage}&mode=side") == MECH
    assert cfg("linkage=klann&module=quad").robot is True
    assert mjcf.model_config(cfg("linkage=klann&mode=klann")).robot is True
    assert cfg("linkage=klann&mode=klann").module == "single"
    with pytest.raises(ValueError, match="unknown mode"):
        cfg("linkage=klann&mode=nope")
    infos = {i["key"]: i for i in server.list_linkages()["linkages"]}
    assert (infos[ARM.linkage]["walks"], infos[ARM.linkage]["ranged"]) == (False, True)
    assert (infos[MECH.linkage]["walks"], infos[MECH.linkage]["ranged"]) == (False, False)
    assert (infos["strider"]["walks"], infos["strider"]["ranged"]) == (True, False)


def test_a_bad_message_to_a_fixed_design_is_answered(adopted):
    """Malformed joints (unknown, NaN, a string) and a crank command of the wrong length
    are answered with the reason; the session goes on."""
    client = TestClient(server.app, base_url="http://localhost")
    with client, client.websocket_connect(f"ws://localhost/ws/sim?linkage={ARM.linkage}") as ws:
        _hello(ws.receive_json)
        for bad in ('{"joints": {"q9": 1}}', '{"joints": {"q2": NaN}}',
                    '{"joints": {"q2": "a"}}', '{"joints": []}', '{"cmd": []}'):
            ws.send_text(bad)
            for _ in range(60):
                msg = ws.receive()
                if msg.get("text"):
                    assert "error" in json.loads(msg["text"]), bad
                    break
            else:
                raise AssertionError(f"no answer to {bad}")


# ---------------------------------------------------------------------------
# The glb on screen and the model name the same nodes (what PhysicsLink.bind needs)
# ---------------------------------------------------------------------------


@pytest.mark.slow       # (fabricates and bakes each design once per engine version)
@pytest.mark.parametrize("cfg", [ARM, MECH], ids=["arm", "mechanism"])
def test_the_glb_and_the_live_model_of_a_fixed_design_name_the_same_nodes(cfg):
    """The viewer refuses a glb with a node the model doesn't know: every node the bake
    writes for an arm and a mechanism is in their model's ``nodes``."""
    from tests.test_sim_live import _glb_node_names

    path, _ = _sim.baked(cfg, n_frames=4)
    _, meta = _sim.mjcf_of(cfg)
    assert _glb_node_names(path) <= set(meta["nodes"])
