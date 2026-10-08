"""End-to-end Playwright tests for the walker viewer.

Boots the FastAPI dev server (see ``tests/conftest.py``), loads the page
in a real Chromium instance, and drives it the way a user would. Each test
talks to the page via the ``window.__viewer`` hook exposed by
``viewer/src/main.ts`` so we can inspect mixer/action state without
screen-scraping.

The default mode is the whole robot; the side-only modes are baked on first
request, so switching to one can take a while on a cold cache.
"""

from __future__ import annotations

import json
import urllib.request

import pytest
from playwright.sync_api import Page, expect

pytestmark = pytest.mark.e2e

BAKE_TIMEOUT_MS = 180_000   # a mode's first request bakes it
SIDE_MODES = ["klann", "double", "decker", "double_double"]

# World-space box of every mesh under the walker node, at each clip time given.
_WORLD_BOXES_JS = """(times) => {
    const v = window.__viewer;
    const w = v.walker;
    return times.map((t) => {
        v.seek(t);
        w.updateMatrixWorld(true);
        const box = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
        w.traverse((o) => {
            if (!o.isMesh) return;
            const pos = o.geometry.attributes.position;
            const e = o.matrixWorld.elements;
            for (let i = 0; i < pos.count; i++) {
                const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
                const p = [
                    e[0] * x + e[4] * y + e[8] * z + e[12],
                    e[1] * x + e[5] * y + e[9] * z + e[13],
                    e[2] * x + e[6] * y + e[10] * z + e[14],
                ];
                for (let k = 0; k < 3; k++) {
                    box.min[k] = Math.min(box.min[k], p[k]);
                    box.max[k] = Math.max(box.max[k], p[k]);
                }
            }
        });
        return box;
    });
}"""


# The same meshes' box in the walker node's own frame (the model's: the linkage in xy, the
# layer stack along z), at clip time 0.
_MODEL_BOX_JS = """() => {
    const v = window.__viewer, w = v.walker;
    v.seek(0);
    w.updateMatrixWorld(true);
    const inv = w.matrixWorld.clone().invert();
    const box = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
    w.traverse((o) => {
        if (!o.isMesh) return;
        const pos = o.geometry.attributes.position;
        const e = inv.clone().multiply(o.matrixWorld).elements;
        for (let i = 0; i < pos.count; i++) {
            const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
            const p = [
                e[0] * x + e[4] * y + e[8] * z + e[12],
                e[1] * x + e[5] * y + e[9] * z + e[13],
                e[2] * x + e[6] * y + e[10] * z + e[14],
            ];
            for (let k = 0; k < 3; k++) {
                box.min[k] = Math.min(box.min[k], p[k]);
                box.max[k] = Math.max(box.max[k], p[k]);
            }
        }
    });
    return box;
}"""


def _wait_ready(page: Page, timeout_ms: int = BAKE_TIMEOUT_MS) -> None:
    page.wait_for_function("() => window.__viewer && window.__viewer.ready", timeout=timeout_ms)


def test_modes_endpoint(viewer_server: str) -> None:
    """The robot is the default; the side-only ids old URLs use are still served."""
    with urllib.request.urlopen(f"{viewer_server}/api/modes") as r:
        body = json.load(r)
    assert body["default"] == "robot"
    assert body["modes"][0] == "robot"
    assert set(SIDE_MODES) <= set(body["modes"])


def test_viewer_loads_glb(page: Page, viewer_server: str) -> None:
    """Page loads the default (robot) GLB, init finishes, status updates."""
    responses: list[tuple[str, int]] = []
    page.on("response", lambda r: responses.append((r.url, r.status)))

    page.goto(viewer_server)
    _wait_ready(page)

    expect(page.locator("#status")).to_contain_text("nodes")
    expect(page.locator("#status")).to_contain_text("tracks")
    assert page.evaluate("() => window.__viewer.mode") == "robot"
    assert page.locator("#mode").input_value() == "robot"

    glb_responses = [(u, s) for (u, s) in responses if "/api/glb/" in u]
    assert glb_responses, "no /api/glb/* request was made"
    assert all(s == 200 for (_, s) in glb_responses), f"bad glb responses: {glb_responses}"
    assert any(u.endswith("/api/glb/robot") for (u, _) in glb_responses)


def test_renders_pixels(page: Page, viewer_server: str) -> None:
    """The three.js canvas actually draws something (non-uniform pixels).

    Uses Playwright's screenshot instead of ``gl.readPixels``: three.js runs
    the renderer with the default ``preserveDrawingBuffer = false``, so a
    post-frame read-back returns a cleared buffer on most GPUs.
    """
    page.goto(viewer_server)
    _wait_ready(page)
    # Give a few animation frames to render.
    page.wait_for_timeout(200)

    png = page.locator("#stage").screenshot()
    # Count distinct bytes in the PNG — a uniform clear yields a tiny palette.
    distinct = len(set(png))
    assert distinct > 40, f"canvas looks blank (png byte palette size={distinct})"


def test_robot_stands_on_its_feet(page: Page, viewer_server: str) -> None:
    """World Z is up: over the gait the lowest foot touches z = 0, the model's axes land
    where they should (its linkage plane upright, xy -> world xz; its layer stack, model
    z, horizontal along world y), and the camera frames all of it."""
    page.goto(viewer_server)
    _wait_ready(page)

    q = page.evaluate("() => window.__viewer.walker.quaternion.toArray()")
    assert q == pytest.approx([2 ** -0.5, 0.0, 0.0, 2 ** -0.5], abs=1e-6)

    # Every keyframe: the gait's lowest point is the ground and nothing sinks
    # below it. The frame is held fixed, so the lowest foot rises a little
    # between stance strokes (~16 mm for the quad: a real robot would bob).
    times = page.evaluate("() => Array.from(window.__viewer.action.getClip().tracks[0].times)")
    lows = [b["min"][2] for b in page.evaluate(_WORLD_BOXES_JS, times)]
    assert min(lows) == pytest.approx(0.0, abs=0.05), f"feet are not on the ground: {min(lows)}"
    assert max(lows) < 20.0, f"no foot near the ground mid-gait: {max(lows)}"

    model = page.evaluate(_MODEL_BOX_JS)
    m = [hi - lo for lo, hi in zip(model["min"], model["max"], strict=True)]
    boxes = page.evaluate(_WORLD_BOXES_JS, [0.0, 0.25, 0.5, 0.75])
    for b in boxes:
        assert b["min"][2] > -0.05, f"something sinks below the ground: {b}"
    size = [hi - lo for lo, hi in zip(boxes[0]["min"], boxes[0]["max"], strict=True)]
    # world (x, y, z) extents are the model's (x, z, y): the stack horizontal, y up
    assert size == pytest.approx([m[0], m[2], m[1]], abs=1e-3), (size, m)

    # Every corner of the robot's box (at t = 0, as framed on load) projects
    # inside the viewport: normalized device coordinates within [-1, 1].
    page.evaluate("() => window.__viewer.setView('three-quarter')")
    inside = page.evaluate(
        """(box) => {
            const cam = window.__viewer.camera;
            cam.updateMatrixWorld(true);
            const v = cam.matrixWorldInverse.elements, p = cam.projectionMatrix.elements;
            const mul = (m, x) => [0, 1, 2, 3].map(
                (r) => m[r] * x[0] + m[4 + r] * x[1] + m[8 + r] * x[2] + m[12 + r] * x[3]);
            const out = [];
            for (let i = 0; i < 8; i++) {
                const c = [i & 1 ? box.max[0] : box.min[0], i & 2 ? box.max[1] : box.min[1],
                           i & 4 ? box.max[2] : box.min[2], 1];
                const clip = mul(p, mul(v, c));
                out.push([clip[0] / clip[3], clip[1] / clip[3]]);
            }
            return out;
        }""",
        boxes[-1],
    )
    for x, y in inside:
        assert -1.0 <= x <= 1.0, f"robot runs off the viewport horizontally: {inside}"
        assert -1.0 <= y <= 1.0, f"robot runs off the viewport vertically: {inside}"


def test_play_button_advances_animation(page: Page, viewer_server: str) -> None:
    """Clicking play makes action.time advance over wall-clock time."""
    page.goto(viewer_server)
    _wait_ready(page)

    before = page.evaluate("() => window.__viewer.action.time")
    assert before == 0.0

    page.locator("#play").click()
    expect(page.locator("#play")).to_have_text("⏸")  # pause glyph

    # The mixer advances with wall-clock time (frames may be slow on software GL).
    page.wait_for_function("() => window.__viewer.action.time > 0.05", timeout=10_000)

    # Readout reflects the advance.
    expect(page.locator("#readout")).not_to_have_text("t 0.000s", timeout=10_000)
    readout = page.locator("#readout").inner_text()
    assert not readout.startswith("t 0.000s"), f"readout stuck: {readout!r}"


def test_mesh_nodes_actually_move(page: Page, viewer_server: str) -> None:
    """Seeking to a different frame must change each moving body's world matrix.

    Catches the failure mode where the animation tracks bind to the right
    nodes and ``action.time`` advances, but all tracks hold identity —
    meshes stay frozen visually despite the mixer reporting progress.
    """
    page.goto(viewer_server)
    _wait_ready(page)

    def sample(t: float) -> dict[str, list]:
        """A probe point's world position per body node, plus what it rides.

        Transforming a non-origin probe point with ``matrixWorld`` captures
        both translation AND rotation — a pure-rotation body (e.g. ``conn``
        pivoting around the fixed O) still shows motion.
        """
        return page.evaluate(
            """(t) => {
                const v = window.__viewer;
                v.seek(t);
                v.walker.updateMatrixWorld(true);
                const out = {};
                for (const o of v.walker.children) {
                    const body = o.userData && o.userData.body;
                    if (!body) continue;
                    const m = o.matrixWorld.elements;
                    out[body] = [
                        m[0] + m[4] + m[12],
                        m[1] + m[5] + m[13],
                        m[2] + m[6] + m[14],
                        o.userData.rigid_with || "",
                    ];
                }
                return out;
            }""",
            t,
        )

    a = sample(0.0)
    b = sample(0.5)
    assert any(n.startswith("L.") for n in a)
    assert any(n.startswith("R.") for n in a)

    def cls(name: str) -> str:
        return name.split(".", 1)[-1].split("_leg")[0]

    # The frames (torso) and everything riding them (servos, pillars, the
    # chassis) are world-fixed; the coupler's joints all sit on the fixed crank
    # centre O. Everything else moves.
    def is_static(name: str) -> bool:
        return cls(name) in ("torso", "coupler") or cls(a[name][3]) == "torso"

    moving = [n for n in a if not is_static(n)]
    assert moving, f"no moving bodies found in {list(a)}"
    for name in moving:
        dx = max(abs(a[name][i] - b[name][i]) for i in range(3))
        assert dx > 1e-3, f"body {name!r} did not move between t=0 and t=0.5 (Δ={dx})"
    for name in (n for n in a if is_static(n)):
        dx = max(abs(a[name][i] - b[name][i]) for i in range(3))
        assert dx < 1e-3, f"static body {name!r} moved (Δ={dx})"


def test_play_pause_toggle(page: Page, viewer_server: str) -> None:
    """Second click pauses: action.time stays put across the subsequent wait."""
    page.goto(viewer_server)
    _wait_ready(page)

    page.locator("#play").click()
    page.wait_for_timeout(200)
    page.locator("#play").click()  # pause
    expect(page.locator("#play")).to_have_text("▶")  # play glyph

    t1 = page.evaluate("() => window.__viewer.action.time")
    page.wait_for_timeout(300)
    t2 = page.evaluate("() => window.__viewer.action.time")
    assert abs(t2 - t1) < 0.02, f"paused action still advanced: {t1} -> {t2}"


def test_slider_seeks(page: Page, viewer_server: str) -> None:
    """Dragging the slider sets action.time and updates the readout."""
    page.goto(viewer_server)
    _wait_ready(page)

    page.evaluate(
        """() => {
            const s = document.getElementById('slider');
            s.value = '0.5';
            s.dispatchEvent(new Event('input', {bubbles: true}));
        }"""
    )
    t = page.evaluate("() => window.__viewer.action.time")
    # Slider step is clipDuration/1000, so exact 0.5 isn't reachable — tolerate a step.
    assert abs(t - 0.5) < 2e-3, f"action.time did not follow slider: {t}"
    expect(page.locator("#readout")).to_contain_text("0.500s")


def test_deep_link_selects_mode(page: Page, viewer_server: str) -> None:
    """``?mode=`` picks the initial assembly (old side-only ids included)."""
    page.goto(f"{viewer_server}/?mode=klann")
    _wait_ready(page)
    assert page.evaluate("() => window.__viewer.mode") == "klann"
    assert page.locator("#mode").input_value() == "klann"


@pytest.mark.parametrize("mode_id", SIDE_MODES)
def test_mode_toggle_swaps_assembly(page: Page, viewer_server: str, mode_id: str) -> None:
    """Selecting a different mode swaps the GLB and rebinds the mixer.

    One side has strictly fewer animated bodies than the whole robot, so the
    track count is a reliable swap indicator; switching back restores it.
    """
    page.goto(viewer_server)
    _wait_ready(page)

    robot_tracks = page.evaluate("() => window.__viewer.action.getClip().tracks.length")
    assert page.evaluate("() => window.__viewer.mode") == "robot"

    page.select_option("#mode", mode_id)
    page.wait_for_function(
        "(m) => window.__viewer && window.__viewer.mode === m && window.__viewer.action",
        arg=mode_id,
        timeout=BAKE_TIMEOUT_MS,
    )
    side_tracks = page.evaluate("() => window.__viewer.action.getClip().tracks.length")
    assert 0 < side_tracks < robot_tracks, (
        f"mode {mode_id} should have fewer tracks than the robot; "
        f"robot={robot_tracks} {mode_id}={side_tracks}"
    )

    page.select_option("#mode", "robot")
    page.wait_for_function(
        "() => window.__viewer.mode === 'robot'", timeout=BAKE_TIMEOUT_MS,
    )
    assert page.evaluate("() => window.__viewer.action.getClip().tracks.length") == robot_tracks
