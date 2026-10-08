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


def _linkages(server: str) -> dict:
    """``/api/linkages``: the default linkage and each one's default module (what a URL
    that names neither loads), from the server rather than written in here."""
    with urllib.request.urlopen(f"{server}/api/linkages") as r:
        doc = json.load(r)
    return {"default": doc["default"],
            "modules": {lk["key"]: lk["default_module"] for lk in doc["linkages"]}}


def _names(query: str, linkage: str, server: str) -> bool:
    """Does a design query (``glbQuery``, the page's search) name ``linkage``? The default
    linkage is left out of a query (``baseQuery``): a query naming no linkage is it."""
    if f"linkage={linkage}" in query:
        return True
    return "linkage=" not in query and linkage == _linkages(server)["default"]


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
    """Left track forward, right track back: the robot turns right (clockwise from above).
    The robot's left track drives crank side L when its design walks toward +x
    (``forward`` +1, the Klann), else side R (the default Strider walks toward -x: its
    cranks' forward direction is +, its robot-left side R)."""
    _open(page, viewer_server, "drive=1")
    a = page.evaluate(SIM_JS)
    b = _hold(page, ["w", "ArrowDown"], 3000)
    left, right = b["omega"] if a["forward"] > 0 else b["omega"][::-1]
    assert left > 0 > right, (a["forward"], b["omega"])
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
    _open(page, viewer_server, "tune=1&linkage=klann&p.OB=1.15")    # OB, DF: Klann's
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
    assert any("/api/glb/robot?module=double" in u for u in glb), glb   # the default design
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
    with urllib.request.urlopen(f"{server}/api/linkages") as r:
        return {lk["key"]: lk for lk in json.load(r)["linkages"]}


def _sliders_are(tune: dict, info: dict) -> None:
    """One slider per parameter: lengths 0.5x..1.5x the default, angles the default +-45 deg."""
    assert [s[0] for s in tune["sliders"]] == [p["name"] for p in info["params"]]
    for (_, lo, hi), p in zip(tune["sliders"], info["params"], strict=True):
        d = p["default"]
        assert (lo, hi) == pytest.approx((d - 45, d + 45) if p["angle"] else (0.5 * d, 1.5 * d))


def test_linkage_dropdown_switches_the_design(page: Page, viewer_server: str) -> None:
    """The tune panel lists ``/api/linkages``' walkers; picking one rebuilds the sliders
    from its parameters, limits the module dropdown to its modules and re-bakes the robot
    with ``linkage=`` (Jansen's mirrored pair here), which then drives on Jansen's feet."""
    catalogue = _catalogue(viewer_server)
    _open(page, viewer_server, "tune=1&linkage=klann&module=double")
    page.wait_for_function("() => window.__viewer.drive.lastWalk !== null", timeout=60_000)
    tune = page.evaluate(TUNE_JS)
    assert tune["linkages"] == [k for k, lk in catalogue.items() if lk["kind"] == "walker"]
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


def test_a_linkage_in_the_url_gets_its_own_default_module(page: Page,
                                                          viewer_server: str) -> None:
    """``?linkage=klann`` names no module: the page loads the Klann's default module (its
    quad), not the default linkage's (the Strider's double, which it used to take)."""
    want = _linkages(viewer_server)["modules"]["klann"]
    assert want != _linkages(viewer_server)["modules"][_linkages(viewer_server)["default"]]
    _open(page, viewer_server, "tune=1&linkage=klann")
    page.wait_for_function("() => window.__viewer.drive.lastWalk !== null", timeout=60_000)
    assert _js(page, "window.__viewer.walker.parent.userData.module") == want
    assert _js(page, "window.__viewer.drive.design.module") == want
    assert _js(page, "window.__viewer.drive.lastWalk.module") == want


def test_drive_never_swaps_in_the_default_design(page: Page, viewer_server: str) -> None:
    """The page's design can't be built (a module Klann doesn't have: 422): drive mode must
    not load the server's default design in its place (it used to: ``glbQuery`` was empty,
    so ``setDrive`` asked for ``/api/glb/robot`` with no query and drove a Strider)."""
    glb: list[str] = []
    page.on("request", lambda r: glb.append(r.url) if "/api/glb/" in r.url else None)
    _open(page, viewer_server, "drive=1&linkage=klann&module=hex", driving=False)
    page.wait_for_timeout(1000)
    assert glb, "no glb requested"
    assert all("linkage=klann" in u for u in glb), glb
    assert _js(page, "window.__viewer.walker") is None
    assert not _js(page, "window.__viewer.drive.physicsOn")


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


# ---------------------------------------------------------------------------
# Physics drive (``viewer/src/drive/physics.ts``, ``/ws/sim``)
# ---------------------------------------------------------------------------

PHYS_JS = """() => {
    const d = window.__viewer.drive, f = d.physics.frame, w = window.__viewer.walker;
    const kids = w ? w.children.filter((o) => o.userData.name) : [];
    return { on: d.physicsOn, open: d.physics.open, design: d.physics.design,
             frame: f && { t: f.t, x: f.pos.x, y: f.pos.y, z: f.pos.z, feet: f.feetDown },
             bound: kids.filter((o) => !o.matrixAutoUpdate).length, nodes: kids.length,
             status: document.querySelector('#status').textContent, glbQuery: d.glbQuery,
             rootVisible: w ? w.parent.visible : null, stick: d.view.stick.visible,
             preview: d.preview, hud: d.hud, search: location.search };
}"""
PHYS_READY = ("() => window.__viewer?.ready && window.__viewer.drive.physicsOn "
              "&& window.__viewer.drive.physics.frame")


def _open_physics(page: Page, server: str, query: str) -> dict:
    """Open ``query`` and wait for the first physics frame (a cold design bakes and builds);
    a timeout reports the page's status (what the connect said)."""
    from playwright.sync_api import TimeoutError as PlaywrightTimeout

    page.goto(f"{server}/?{query}")
    try:
        page.wait_for_function(PHYS_READY, timeout=BAKE_TIMEOUT_MS)
    except PlaywrightTimeout:
        status = _js(page, "document.querySelector('#status').textContent")
        tune = _js(page, "window.__viewer?.drive?.tune?.status")
        raise AssertionError(f"no physics frame within {BAKE_TIMEOUT_MS} ms; status: {status!r}, "
                             f"tune: {tune!r}") from None
    return page.evaluate(PHYS_JS)


def test_physics_simulates_the_linkage_on_screen(page: Page, viewer_server: str) -> None:
    """``?linkage=strider&physics=1`` connects ``/ws/sim`` for the glb's own design: the
    hello names a Strider, every part of the glb is bound to a model body, the keys walk
    it and the HUD reads the physics (model-only rows blank)."""
    errors: list[str] = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    a = _open_physics(page, viewer_server, "drive=1&linkage=strider&physics=1")
    assert a["design"]["linkage"] == "strider"
    assert a["design"]["module"] == _linkages(viewer_server)["modules"]["strider"]
    assert _names(a["glbQuery"], "strider", viewer_server)
    assert a["bound"] == a["nodes"] > 100
    assert "physics=1" in a["search"]
    assert _names(a["search"], "strider", viewer_server)
    assert "strider" in a["status"]
    page.mouse.click(400, 400)
    page.keyboard.down("KeyW")
    page.keyboard.down("ArrowUp")
    page.wait_for_timeout(3000)
    b = page.evaluate(PHYS_JS)
    page.keyboard.up("KeyW")
    page.keyboard.up("ArrowUp")
    # sim seconds, not wall seconds: a loaded machine (software GL) ticks the server slower
    dt = b["frame"]["t"] - a["frame"]["t"]
    assert dt > 1.0
    moved = math.hypot(b["frame"]["x"] - a["frame"]["x"], b["frame"]["y"] - a["frame"]["y"])
    assert moved / dt > 100
    assert b["frame"]["z"] > 50
    assert re.search(r"\d", b["hud"]["speed"])
    assert "turned" in b["hud"]["cranks"]
    torque = b["hud"]["torque"]
    assert re.search(r"L -?\d\.\d\d · R -?\d\.\d\d N·m \(rated", torque), torque
    assert "real time" in b["hud"]["model"]
    assert b["hud"]["slip"] == "–"
    assert b["hud"]["margin"] == "–"
    assert errors == []
    # The command goes up on the key change itself (not only from the animation loop) and
    # the release lands at once: the robot stops.
    page.wait_for_timeout(1500)
    c = page.evaluate(PHYS_JS)
    d = page.evaluate(PHYS_JS)
    page.wait_for_timeout(1000)
    e = page.evaluate(PHYS_JS)
    crawl = math.hypot(e["frame"]["x"] - d["frame"]["x"], e["frame"]["y"] - d["frame"]["y"])
    assert crawl < 30, (c["frame"], e["frame"])


def test_physics_can_be_turned_on_after_a_no_op_off(page: Page, viewer_server: str) -> None:
    """``setPhysics(false)`` while already off (every drive-off sends one) used to leave a
    settled promise that every later ``setPhysics`` returned, so physics could never go on
    again on that page. Off, drive off and on, then on: it connects."""
    _open(page, viewer_server, "drive=1")
    _js(page, "window.__viewer.drive.setPhysics(false)")
    _js(page, "window.__viewer.drive.setDrive(false)")
    _js(page, "window.__viewer.drive.setDrive(true)")
    page.wait_for_function("() => window.__viewer.drive.sim.state !== null", timeout=60_000)
    _js(page, "void window.__viewer.drive.setPhysics(true).catch(() => {})")
    page.wait_for_function(PHYS_READY, timeout=BAKE_TIMEOUT_MS)
    s = page.evaluate(PHYS_JS)
    assert s["on"]
    assert s["open"]
    assert s["bound"] == s["nodes"]


def test_drive_off_under_physics_hands_the_clip_back(page: Page, viewer_server: str) -> None:
    """Drive off while physics is on: the session closes, every node is the animation's
    again (the clip runs), the walker stands at its baked pose, the status says so and no
    HUD row keeps a physics reading; physics can go on again afterwards."""
    _open_physics(page, viewer_server, "drive=1&physics=1")
    page.mouse.click(400, 400)
    page.keyboard.down("ArrowUp")
    page.keyboard.down("KeyW")
    page.wait_for_timeout(1500)
    page.keyboard.up("ArrowUp")
    page.keyboard.up("KeyW")
    _js(page, "window.__viewer.drive.setDrive(false)")
    page.wait_for_timeout(300)
    s = page.evaluate(PHYS_JS)
    assert not s["on"]
    assert not s["open"]
    assert s["bound"] == 0
    assert "MuJoCo live" not in s["status"]
    assert set(s["hud"].values()) == {"–"}, s["hud"]
    assert "drive=1" not in s["search"]
    assert "physics=1" not in s["search"]
    quat = _js(page, "window.__viewer.walker.quaternion.toArray()")
    assert quat == pytest.approx(STAND, abs=1e-6)
    baked = page.context.new_page()           # the same design, never driven: its pose
    _open(baked, viewer_server, "", driving=False)
    stand = _js(baked, "window.__viewer.walker.position.toArray()")
    baked.close()
    assert _js(page, "window.__viewer.walker.position.toArray()") == pytest.approx(stand,
                                                                                  abs=1e-6)
    # the clip owns the nodes again: scheduled on the mixer, paused at t = 0 (every drive-off)
    assert _js(page, "window.__viewer.action.isScheduled()")
    assert _js(page, "window.__viewer.action.time") == 0
    _js(page, "window.__viewer.drive.setDrive(true)")
    _js(page, "void window.__viewer.drive.setPhysics(true).catch(() => {})")
    page.wait_for_function(PHYS_READY, timeout=BAKE_TIMEOUT_MS)


def test_switching_linkage_during_the_model_build(page: Page, viewer_server: str) -> None:
    """The linkage changes while the physics model of the previous design is still building:
    that build's hello is dropped (it would bind the old design to the new glb) and physics
    connects for the design on screen; nothing throws and the frames draw it."""
    errors: list[str] = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    # The URL carries the module (a plain load's query, ``baseQuery``), not phases: the
    # Klann double is a design whose model no test has built yet.
    _open(page, viewer_server, "drive=1&linkage=klann&module=double")
    _js(page, "void window.__viewer.drive.setPhysics(true).catch(() => {})")
    page.wait_for_function(
        "() => /building/.test(document.querySelector('#status').textContent)", timeout=60_000)
    page.evaluate(SET_JS, ["linkage", "strider"])
    page.wait_for_function(
        f"() => {LOADED_JS} === 'strider' && window.__viewer.drive.physicsOn "
        "&& window.__viewer.drive.physics.design?.linkage === 'strider' "
        "&& window.__viewer.drive.physics.frame",
        timeout=BAKE_TIMEOUT_MS)
    page.wait_for_timeout(500)
    s = page.evaluate(PHYS_JS)
    assert s["design"]["linkage"] == "strider"
    assert _names(s["glbQuery"], "strider", viewer_server)
    assert "klann" not in s["glbQuery"]
    assert s["bound"] == s["nodes"] > 100
    assert "strider" in s["status"]
    page.mouse.click(400, 400)
    page.keyboard.down("ArrowUp")
    page.keyboard.down("KeyW")
    page.wait_for_timeout(2000)
    b = page.evaluate(PHYS_JS)
    page.keyboard.up("ArrowUp")
    page.keyboard.up("KeyW")
    assert math.hypot(b["frame"]["x"] - s["frame"]["x"], b["frame"]["y"] - s["frame"]["y"]) > 50
    assert errors == []


def test_physics_toggle_settles_on_the_last_request(page: Page, viewer_server: str) -> None:
    """On then off before the connect lands: off. A six-call burst ends in the last state
    with one streaming socket; the superseded ones are closed."""
    sockets: list = []
    page.on("websocket", lambda ws: sockets.append(ws) if "/ws/sim" in ws.url else None)
    _open(page, viewer_server, "drive=1")
    on = _js(page, """(async () => { const d = window.__viewer.drive;
        void d.setPhysics(true).catch(() => {});
        await d.setPhysics(false);
        return d.physicsOn; })()""")
    assert on is False
    page.wait_for_timeout(1500)
    s = page.evaluate(PHYS_JS)
    assert not s["on"]
    assert not s["open"]
    _js(page, """(async () => { const d = window.__viewer.drive; let p;
        for (const on of [true, false, true, false, true, false, true]) {
            p = d.setPhysics(on).catch(() => {});
        }
        await p; })()""")
    page.wait_for_function(PHYS_READY, timeout=BAKE_TIMEOUT_MS)
    page.wait_for_timeout(1000)
    s = page.evaluate(PHYS_JS)
    assert s["on"]
    assert s["open"]
    assert "physics=1" in s["search"]
    assert len([ws for ws in sockets if not ws.is_closed()]) == 1, [ws.url for ws in sockets]


def test_the_physics_gate_is_mujocos_verdict_not_the_margin(page: Page, viewer_server: str) -> None:
    """Jansen's double has a negative quasi-static stability margin (-15 mm), which used to
    refuse a design unseen. Now the server's straight run decides (the hello's ``forward``):
    it connects (the margin a warning in the status) or is refused for falling over, the
    status naming MuJoCo's run and never the margin. Either way the walking model's margin
    is not the gate. (The Jansen quad, 3 mm, which this was written for, has no layer plan
    in the planner's budget: no glb, no physics. Until the default module fix the URL's
    ``linkage=jansen`` loaded the double anyway.)"""
    page.goto(f"{viewer_server}/?drive=1&linkage=jansen&module=double&physics=1")
    page.wait_for_function(
        "() => window.__viewer?.ready && ((window.__viewer.drive.physicsOn"
        " && window.__viewer.drive.physics.frame)"
        " || /fell over in MuJoCo/.test(document.querySelector('#status').textContent))",
        timeout=BAKE_TIMEOUT_MS)
    s = page.evaluate(PHYS_JS)
    assert "linkage=jansen" in s["search"]
    walk = _js(page, "window.__viewer.drive.lastPred")
    assert walk["min_margin_mm"] < 15.0, walk
    if s["on"]:
        assert s["open"]
        assert s["design"]["linkage"] == "jansen"
        assert "physics=1" in s["search"]
        assert "margin" in s["status"]
        assert "⚠" in s["status"]
        fwd = _js(page, "window.__viewer.drive.physics.steering.forward")
        assert fwd["fell"] is False
    else:
        assert not s["open"]
        assert "physics=1" not in s["search"]
        assert "fell over in MuJoCo's straight run" in s["status"]
        assert "margin" not in s["status"]


def test_the_steering_row_says_what_the_server_proved(page: Page, viewer_server: str) -> None:
    """The Klann quad cannot skid-steer (it rolls over at |L−R| 0.4) but walks a 45 or 90 deg
    excursion (a grant on the edge of its tilt limit: 90 on the parametric servo since W8)
    that the server's phase lock bounds: the HUD's steering row and the turn
    slider's name say so, the authority defaults to the excursion's differential, and
    the steering key does send a differential (the server bounds the offset)."""
    _open_physics(page, viewer_server, "drive=1&linkage=klann&physics=1")    # its quad
    steer = _js(page, "window.__viewer.drive.physics.steering")
    assert steer["turn"] == 0.0
    assert steer["step_deg"] in (45.0, 90.0)
    s = page.evaluate(PHYS_JS)
    assert "excursion" in s["hud"]["steering"]
    assert _js(page, "window.__viewer.drive.opts.turn") == pytest.approx(0.4)
    cmd = _js(page, "window.__viewer.drive.physicsCommand(1, 0)")
    assert cmd[0] - cmd[1] == pytest.approx(0.4)
    assert _js(page, "window.__viewer.drive.hud.loads") != "–"


def test_strider_turns_on_a_steering_key_untouched(page: Page, viewer_server: str) -> None:
    """The Strider quad skid-steers (the hello grants |L−R| 0.4): forward + left turns it
    by more than 10 deg in 4 s with nothing set on the slider (the authority used to start
    at 0 and only ever be lowered, so the status promised steering that never came)."""
    a = _open_physics(page, viewer_server, "drive=1&linkage=strider&physics=1&scheme=arcade")
    steer = _js(page, "window.__viewer.drive.physics.steering")
    assert steer["turn"] == pytest.approx(0.4)
    assert _js(page, "window.__viewer.drive.opts.turn") == pytest.approx(0.4)
    page.mouse.click(400, 400)
    page.keyboard.down("KeyW")
    page.keyboard.down("KeyA")
    page.wait_for_timeout(4000)
    cmd = _js(page, "window.__viewer.drive.physicsCommand(1, 0)")
    # the heading from the base quaternion: the walking axis (mech +x) on the ground
    turned = _js(page, """(() => { const q = window.__viewer.drive.physics.frame.quat;
        const x = 1 - 2 * (q.y * q.y + q.z * q.z), y = 2 * (q.x * q.y + q.z * q.w);
        return Math.atan2(y, x) * 180 / Math.PI; })()""")
    b = page.evaluate(PHYS_JS)
    page.keyboard.up("KeyW")
    page.keyboard.up("KeyA")
    assert cmd[0] - cmd[1] == pytest.approx(0.4)
    assert abs(turned) > 10.0, (turned, b["hud"], a["frame"], b["frame"])


def test_a_reset_mid_drive_starts_the_hud_over(page: Page, viewer_server: str) -> None:
    """Reset while walking: the sim clock goes back to zero, and the HUD's speed follows
    the new motion within 1.5 s instead of freezing on the old window (every history is
    an epoch of the sim clock); the real-time factor never reads negative or above 1."""
    _open_physics(page, viewer_server, "drive=1&physics=1")
    page.mouse.click(400, 400)
    page.keyboard.down("KeyW")
    page.keyboard.down("ArrowUp")
    page.wait_for_timeout(2500)
    _js(page, "window.__viewer.drive.opts.reset()")
    page.wait_for_timeout(300)
    a = page.evaluate(PHYS_JS)
    page.wait_for_timeout(1500)
    b = page.evaluate(PHYS_JS)
    page.keyboard.up("KeyW")
    page.keyboard.up("ArrowUp")
    assert a["frame"]["t"] < 1.0, a["frame"]
    dt = b["frame"]["t"] - a["frame"]["t"]
    assert dt > 0.5
    measured = math.hypot(b["frame"]["x"] - a["frame"]["x"], b["frame"]["y"] - a["frame"]["y"]) / dt
    shown = float(re.match(r"([\d.]+) mm/s", b["hud"]["speed"]).group(1))
    assert shown == pytest.approx(measured, rel=0.35, abs=25), (shown, measured, b["hud"])
    m = re.search(r"×(-?[\d.]+) real time", b["hud"]["model"])
    assert m, b["hud"]["model"]
    assert 0.0 < float(m.group(1)) <= 1.0, b["hud"]["model"]
    rpm = re.search(r"(\d+) rpm", b["hud"]["cranks"])
    assert rpm is None or float(rpm.group(1)) < 80, b["hud"]["cranks"]


def test_the_url_follows_a_linkage_switch_with_physics_off(page: Page, viewer_server: str) -> None:
    """The tune panel's dropdown re-bakes another linkage with physics off: the deep link
    carries ``linkage=`` once the glb lands (it used to keep the previous robot)."""
    _open(page, viewer_server, "drive=1")
    _js(page, "window.__viewer.drive.setPreview(true)")
    page.wait_for_function("() => window.__viewer.drive.lastWalk !== null", timeout=60_000)
    other = "klann"                     # not the default (Strider): the URL must name it
    assert _linkages(viewer_server)["default"] != other
    page.evaluate(SET_JS, ["linkage", other])
    page.wait_for_function(f"() => {LOADED_JS} === '{other}' && !window.__viewer.drive.preview",
                           timeout=BAKE_TIMEOUT_MS)
    page.wait_for_timeout(300)
    s = page.evaluate(PHYS_JS)
    assert not s["on"]
    assert f"linkage={other}" in s["glbQuery"]
    assert f"linkage={other}" in s["search"]
    assert "drive=1" in s["search"]


def test_stick_preview_turns_physics_off(page: Page, viewer_server: str) -> None:
    """The two modes are exclusive: opening the tune preview hands the parts back and
    closes the session; the stick shows, the robot is hidden, the URL says ``tune=1``."""
    _open_physics(page, viewer_server, "drive=1&physics=1")
    _js(page, "window.__viewer.drive.setPreview(true)")
    page.wait_for_function(
        "() => window.__viewer.drive.preview && !window.__viewer.drive.physicsOn", timeout=60_000)
    page.wait_for_timeout(500)
    s = page.evaluate(PHYS_JS)
    assert not s["open"]
    assert s["stick"]
    assert s["rootVisible"] is False
    assert "tune=1" in s["search"]
    assert "physics=1" not in s["search"]


def test_switching_linkage_with_physics_on_reconnects(page: Page, viewer_server: str) -> None:
    """The tune panel's linkage dropdown re-bakes the robot: physics reconnects for the new
    design (the hello echoes it, every node binds) and nothing throws on the page."""
    errors: list[str] = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    _open_physics(page, viewer_server, "drive=1&physics=1")
    other = "klann"                     # not the default (Strider): a real switch
    assert _linkages(viewer_server)["default"] != other
    page.evaluate(SET_JS, ["linkage", other])
    page.wait_for_function(
        f"() => {LOADED_JS} === '{other}' && window.__viewer.drive.physicsOn "
        f"&& window.__viewer.drive.physics.design?.linkage === '{other}' "
        "&& window.__viewer.drive.physics.frame",
        timeout=BAKE_TIMEOUT_MS)
    page.wait_for_timeout(500)
    s = page.evaluate(PHYS_JS)
    assert s["design"]["linkage"] == other
    assert s["bound"] == s["nodes"]
    assert f"linkage={other}" in s["glbQuery"]
    assert f"linkage={other}" in s["search"]
    assert "physics=1" in s["search"]
    assert errors == []
