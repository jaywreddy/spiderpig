"""End-to-end tests for the viewer's drive mode and tune panel (``viewer/src/drive/``).

Set ``SPIDERPIG_SHOTS=<dir>`` to also save screenshots of drive mode and the
tune panel there.
"""

from __future__ import annotations

import json
import math
import os
import re
import urllib.request
from pathlib import Path

import pytest
from playwright.sync_api import Page

from tests.e2e.drive_fixture import flat_walk, walk_response

pytestmark = pytest.mark.e2e

BAKE_TIMEOUT_MS = 180_000
STAND = [2 ** -0.5, 0.0, 0.0, 2 ** -0.5]
SIM_JS = """() => {
    const d = window.__viewer.drive, s = d.sim, w = window.__viewer.walker;
    return { x: s.x, z: s.z, yaw: s.yaw, forward: s.forward, omega: [s.omega.L, s.omega.R],
             walker: w.position.toArray(), last: s.history[s.history.length - 1] ?? null,
             hud: d.hud, active: d.active, preview: d.preview };
}"""


def _open(page: Page, server: str, query: str, driving: bool = True) -> None:
    page.goto(f"{server}/?{query}")
    page.wait_for_function("() => window.__viewer && window.__viewer.ready",
                           timeout=BAKE_TIMEOUT_MS)
    if driving:
        page.wait_for_function("() => window.__viewer.drive.sim.state !== null", timeout=60_000)


def _js(page: Page, expr: str):
    return page.evaluate(f"() => {expr}")


def _hold(page: Page, keys: list[str], ms: int, shot: str | None = None,
          together: bool = False) -> dict:
    """Hold ``keys`` for ``ms``; the sim state just before they're released.

    ``together`` presses them in one task (real key presses land in different
    frames now and then, and a lag between the sides is a phase offset)."""
    js = "([codes, t]) => codes.forEach((code) => dispatchEvent(new KeyboardEvent(t, { code })))"
    if together:
        page.evaluate(js, [keys, "keydown"])
    else:
        for k in keys:
            page.keyboard.down(k)
    page.wait_for_timeout(ms)
    if shot and os.environ.get("SPIDERPIG_SHOTS"):
        page.screenshot(path=str(Path(os.environ["SPIDERPIG_SHOTS"]) / shot))
    state = page.evaluate(SIM_JS)
    if together:
        page.evaluate(js, [keys, "keyup"])
    else:
        for k in keys:
            page.keyboard.up(k)
    return state


def test_drive_forward_moves_along_heading(page: Page, viewer_server: str) -> None:
    """W + Up turn both cranks forward: the robot walks along its heading, and the
    baked model follows the simulated pose; the HUD shows finite metrics."""
    _open(page, viewer_server, "drive=1")
    a = page.evaluate(SIM_JS)
    assert a["active"]
    assert "drive=1" in _js(page, "location.search")

    b = _hold(page, ["KeyW", "ArrowUp"], 3000, shot="drive_walk.png", together=True)
    fwd = a["forward"]
    heading = [fwd * math.cos(a["yaw"]), -fwd * math.sin(a["yaw"])]   # ground (x, z)
    d = [b["x"] - a["x"], b["z"] - a["z"]]
    along = d[0] * heading[0] + d[1] * heading[1]
    across = d[0] * heading[1] - d[1] * heading[0]
    assert along > 20.0, f"robot did not walk forward: {d}"
    assert abs(across) < 0.05 * along, f"robot drifted sideways: along {along}, across {across}"
    assert abs(b["yaw"] - a["yaw"]) < math.radians(1)   # sides in phase: a mirror-symmetric gait
    # The walker node sits at the simulated pose: world (x, -z) on the ground.
    assert b["walker"][0] == pytest.approx(b["x"], abs=1e-3)
    assert b["walker"][1] == pytest.approx(-b["z"], abs=1e-3)

    last = b["last"]
    for key in ("speed", "yawRate", "height", "pitch", "roll", "slip", "margin"):
        assert math.isfinite(last[key]), f"{key} = {last[key]}"
    assert last["height"] > 50.0
    for key in ("speed", "height", "pitch", "contacts", "slip", "margin"):
        assert re.search(r"\d", b["hud"][key]), f"HUD {key}: {b['hud'][key]!r}"

    # Drive off: the viewer is back to the standing, clip-driven robot.
    _js(page, "window.__viewer.drive.setDrive(false)")
    assert _js(page, "window.__viewer.walker.quaternion.toArray()") == pytest.approx(STAND)
    assert "drive=1" not in _js(page, "location.search")


def test_tank_opposite_inputs_turn(page: Page, viewer_server: str) -> None:
    """Left track forward, right track back: the robot turns right (clockwise from above)."""
    _open(page, viewer_server, "drive=1")
    a = page.evaluate(SIM_JS)
    b = _hold(page, ["w", "ArrowDown"], 3000)
    assert b["omega"][0] > 0 > b["omega"][1], b["omega"]
    assert b["yaw"] - a["yaw"] < -math.radians(5), f"yaw {a['yaw']} -> {b['yaw']}"


def test_drive_from_glb_extras(page: Page, viewer_server: str) -> None:
    """With the ``drive`` extras on the glb's ``walker`` node, drive mode needs no
    ``/api/walk``; arcade W + D walks forward while turning right."""
    walk: list[str] = []
    page.on("request", lambda r: walk.append(r.url) if "/api/walk" in r.url else None)
    page.goto(f"{viewer_server}/?scheme=arcade")
    page.wait_for_function("() => window.__viewer && window.__viewer.ready",
                           timeout=BAKE_TIMEOUT_MS)
    body = walk_response("quad")
    extras = {k: body[k] for k in ("theta_samples", "feet", "com", "servo")}
    page.evaluate("""async (extras) => {
        window.__viewer.walker.userData.drive = { ...extras, clip_duration_s: 1.0 };
        await window.__viewer.drive.setDrive(true);
    }""", extras)
    a = page.evaluate(SIM_JS)
    b = _hold(page, ["KeyW", "KeyD"], 3000, together=True)
    assert walk == []
    assert b["omega"][0] > b["omega"][1] >= 0, b["omega"]     # left faster: a right turn
    assert b["yaw"] < a["yaw"]
    assert math.hypot(b["x"] - a["x"], b["z"] - a["z"]) > 20.0


def test_model_known_answers(page: Page, viewer_server: str) -> None:
    """The TS walking model: a flat gait walks without slip at the stance speed;
    opposite cranks yaw it by the least-squares rate; Klann's quad at 135 deg
    rests on its most level containing face."""
    _open(page, viewer_server, "", driving=False)
    quad = walk_response("quad")
    r = page.evaluate(
        """([flat, quad]) => {
            const m = window.__viewer.drive.model, th = 30 * Math.PI / 180;
            const pick = (s) => ({ slip: s.slip, vx: s.vx, vz: s.vz, w: s.w, pitch: s.pitch,
                roll: s.roll, height: s.height, n: s.nContacts, margin: s.margin,
                tipping: s.tipping });
            const f = m.parseDrive(flat), q = m.parseDrive(quad), t135 = 135 * Math.PI / 180;
            return {
                fwd: pick(m.evaluate(f, { L: th, R: th }, { L: 1, R: 1 })),
                turn: pick(m.evaluate(f, { L: th, R: th }, { L: 1, R: -1 })),
                walk: m.straightWalk(f),
                quad: pick(m.evaluate(q, { L: t135, R: t135 }, { L: 1, R: 1 })),
            };
        }""",
        [flat_walk(), quad],
    )
    v = 120 / (4 * math.pi / 3)          # stance speed, mm/rad
    fwd = r["fwd"]
    assert fwd["slip"] < 1e-9
    assert abs(fwd["w"]) < 1e-12
    assert fwd["vx"] == pytest.approx(v)
    assert fwd["vz"] == pytest.approx(0, abs=1e-9)
    assert fwd["n"] == 4
    assert fwd["height"] == pytest.approx(100)
    assert fwd["pitch"] == pytest.approx(0, abs=1e-9)
    assert fwd["roll"] == pytest.approx(0, abs=1e-9)
    assert fwd["margin"] == pytest.approx(15)   # COM (0, 0) over the rectangle x in [-15, 45]
    # Contacts at x = 45, -15 and z = -50, 50: w = sum(dx pz - dz px) / sum(dx^2 + dz^2).
    assert r["turn"]["w"] == pytest.approx(-4 * 50 * v / (4 * 50 ** 2 + 4 * 30 ** 2))
    assert r["turn"]["vx"] == pytest.approx(0, abs=1e-9)
    assert r["walk"]["stride_mm"] == pytest.approx(180, rel=0.02)
    assert r["walk"]["bob_mm"] == pytest.approx(0, abs=1e-9)
    assert -8.4 <= r["quad"]["pitch"] <= 8.4, r["quad"]
    assert not r["quad"]["tipping"], r["quad"]


def test_tune_panel_stick_preview(page: Page, viewer_server: str) -> None:
    """``?tune=1`` opens the stick preview of ``/api/walk``; moving a proportion
    slider re-queries it (debounced) and updates the metrics; the same model drives it."""
    requests: list[str] = []
    page.on("request", lambda r: requests.append(r.url) if "/api/walk" in r.url else None)
    _open(page, viewer_server, "tune=1&p.OB=1.15")
    page.wait_for_function("() => window.__viewer.drive.lastWalk !== null", timeout=60_000)
    s = page.evaluate(SIM_JS)
    assert s["active"]
    assert s["preview"]
    assert _js(page, "window.__viewer.drive.lastWalk.proportions.OB") == pytest.approx(1.15)
    assert not _js(page, "window.__viewer.walker.parent.visible"), "full model still shown"
    assert _js(page, "window.__viewer.drive.view.stick.visible")
    n_legs = _js(page, "window.__viewer.drive.lastWalk.legs.length")
    n_links = _js(page, "window.__viewer.drive.lastWalk.links.length")
    drawn = _js(page, "window.__viewer.drive.view.stick.geometry.drawRange.count")
    assert drawn == 2 * 2 * n_legs * n_links     # both sides, two vertices per link

    # Move a slider as the GUI does; one debounced request follows.
    before = len(requests)
    _js(page, """window.__viewer.drive.tuneGui.controllersRecursive()
        .find((c) => c.property === 'DF').setValue(2.7)""")
    page.wait_for_function(
        "() => Math.abs(window.__viewer.drive.lastWalk.proportions.DF - 2.7) < 1e-9",
        timeout=60_000)
    assert any("p.DF=2.7" in u for u in requests[before:]), requests[before:]
    assert "p.DF=2.7" in _js(page, "location.search")
    metrics = _js(page, """Object.fromEntries(window.__viewer.drive.tuneGui.foldersRecursive()
        .find((f) => f._title.startsWith('metrics')).controllers
        .map((c) => [c.property, c.getValue()]))""")
    assert float(metrics["stride_mm"]) > 10.0, metrics
    assert float(metrics["bob_mm"]) >= 0.0, metrics

    b = _hold(page, ["w", "ArrowUp"], 2000, shot="drive_tune.png")
    assert math.hypot(b["x"] - s["x"], b["z"] - s["z"]) > 10.0


def test_rebuild_parts_returns_to_full_model(page: Page, viewer_server: str) -> None:
    """"Rebuild parts" loads ``/api/glb/robot?<params>`` and drives the full model again."""
    _open(page, viewer_server, "tune=1")
    page.wait_for_function("() => window.__viewer.drive.lastWalk !== null", timeout=60_000)
    glb: list[str] = []
    page.on("request", lambda r: glb.append(r.url) if "/api/glb/" in r.url else None)
    _js(page, "window.__viewer.drive.tune.rebuild()")
    page.wait_for_function("() => !window.__viewer.drive.preview", timeout=BAKE_TIMEOUT_MS)
    assert any("/api/glb/robot?module=quad" in u for u in glb), glb
    assert _js(page, "window.__viewer.walker.parent.visible")
    assert _js(page, "window.__viewer.drive.active")
    assert not _js(page, "window.__viewer.drive.view.stick.visible")


TUNE_JS = """() => {
    const d = window.__viewer.drive, all = d.tuneGui.controllersRecursive();
    const ctl = (p) => all.find((c) => c.property === p);
    const params = d.tuneGui.foldersRecursive().find((f) => f._title.startsWith('parameters'));
    return { linkage: d.design.linkage, module: d.design.module,
             linkages: ctl('linkage')._values, modules: ctl('module')._values,
             sliders: params.controllers.map((c) => [c.property, c._min, c._max]) };
}"""
SET_JS = """([prop, value]) => { window.__viewer.drive.tuneGui.controllersRecursive()
    .find((c) => c.property === prop).setValue(value); }"""
LOADED_JS = "window.__viewer.walker.parent.userData.linkage"


def _catalogue(server: str) -> dict:
    with urllib.request.urlopen(f"{server}/api/linkages") as r:  # noqa: S310
        return {lk["key"]: lk for lk in json.load(r)["linkages"]}


def _sliders_are(tune: dict, info: dict) -> None:
    """One slider per parameter: lengths 0.5x..1.5x the default, angles the default +-45 deg."""
    assert [s[0] for s in tune["sliders"]] == [p["name"] for p in info["params"]]
    for (_, lo, hi), p in zip(tune["sliders"], info["params"], strict=True):
        d = p["default"]
        assert (lo, hi) == pytest.approx((d - 45, d + 45) if p["angle"] else (0.5 * d, 1.5 * d))


def test_linkage_dropdown_switches_the_design(page: Page, viewer_server: str) -> None:
    """The tune panel lists ``/api/linkages``; picking one rebuilds the parameter sliders
    from its parameters, limits the module dropdown to its modules and re-bakes the robot
    with ``linkage=`` (Jansen's mirrored pair here), which then drives on Jansen's feet."""
    catalogue = _catalogue(viewer_server)
    _open(page, viewer_server, "tune=1&module=double")
    page.wait_for_function("() => window.__viewer.drive.lastWalk !== null", timeout=60_000)
    tune = page.evaluate(TUNE_JS)
    assert tune["linkages"] == list(catalogue)
    assert (tune["linkage"], tune["module"]) == ("klann", "double")
    _sliders_are(tune, catalogue["klann"])

    glb: list[str] = []
    page.on("request", lambda r: glb.append(r.url) if "/api/glb/" in r.url else None)
    page.evaluate(SET_JS, ["linkage", "jansen"])
    page.wait_for_function(f"() => {LOADED_JS} === 'jansen' && !window.__viewer.drive.preview",
                           timeout=BAKE_TIMEOUT_MS)
    tune = page.evaluate(TUNE_JS)
    assert (tune["linkage"], tune["module"]) == ("jansen", "double")    # it has a double: kept
    assert tune["modules"] == list(catalogue["jansen"]["modules"])
    _sliders_are(tune, catalogue["jansen"])
    assert any("/api/glb/robot?module=double&linkage=jansen" in u for u in glb), glb
    assert "linkage=jansen" in _js(page, "location.search")
    feet = _js(page, "window.__viewer.walker.userData.drive.feet.map((f) => f.body)")
    assert feet == ["L.b6_leg0", "L.b6_leg1", "R.b6_leg0", "R.b6_leg1"]
    assert _js(page, "window.__viewer.drive.sim.state.feet.length") == 4


def test_two_feet_per_leg(page: Page, viewer_server: str) -> None:
    """A deep link to a Strider (one coupled pair per side: two feet a leg) loads its robot;
    the stick preview draws both sides and the walking model has all four feet; the mode
    dropdown keeps the linkage."""
    _open(page, viewer_server, "tune=1&module=single&linkage=strider")
    page.wait_for_function("() => window.__viewer.drive.lastWalk?.linkage === 'strider'",
                           timeout=60_000)
    assert _js(page, LOADED_JS) == "strider"
    w = _js(page, """(({ lastWalk: w }) => ({ feet: w.feet.map((f) => f.body),
        legs: w.legs.length, links: w.links.length }))(window.__viewer.drive)""")
    assert w["feet"] == ["L.b3", "L.b7", "R.b3", "R.b7"]
    assert w["legs"] == 1
    drawn = _js(page, "window.__viewer.drive.view.stick.geometry.drawRange.count")
    assert drawn == 2 * 2 * w["legs"] * w["links"]      # both sides, two vertices per link
    assert _js(page, "window.__viewer.drive.sim.state.feet.length") == 4
    assert re.fullmatch(r"\d / 4", _js(page, "window.__viewer.drive.hud.contacts"))

    page.select_option("#mode", "klann")                 # one side, one leg: still a Strider
    page.wait_for_function(f"() => window.__viewer.mode === 'klann' && {LOADED_JS} === 'strider'",
                           timeout=BAKE_TIMEOUT_MS)
