"""End-to-end Playwright tests for the Klann viewer.

Boots the FastAPI dev server (see ``tests/conftest.py``), loads the page
in a real Chromium instance, and drives it the way a user would. Each test
talks to the page via the ``window.__viewer`` hook exposed by
``viewer/main.js`` so we can inspect mixer/action state without screen-scraping.
"""

from __future__ import annotations

import pytest
from playwright.sync_api import Page, expect

pytestmark = pytest.mark.e2e


def _wait_ready(page: Page, timeout_ms: int = 20_000) -> None:
    page.wait_for_function("() => window.__viewer && window.__viewer.ready", timeout=timeout_ms)


def test_viewer_loads_glb(page: Page, viewer_server: str) -> None:
    """Page loads, /api/klann.glb returns 200, init finishes, status updates."""
    responses: list[tuple[str, int]] = []
    page.on("response", lambda r: responses.append((r.url, r.status)))

    page.goto(viewer_server)
    _wait_ready(page)

    expect(page.locator("#status")).to_contain_text("nodes")
    expect(page.locator("#status")).to_contain_text("tracks")

    glb_responses = [(u, s) for (u, s) in responses if "/api/glb/" in u]
    assert glb_responses, "no /api/glb/* request was made"
    assert all(s == 200 for (_, s) in glb_responses), f"bad glb responses: {glb_responses}"


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


def test_play_button_advances_animation(page: Page, viewer_server: str) -> None:
    """Clicking play makes action.time advance over wall-clock time."""
    page.goto(viewer_server)
    _wait_ready(page)

    before = page.evaluate("() => window.__viewer.action.time")
    assert before == 0.0

    page.locator("#play").click()
    expect(page.locator("#play")).to_have_text("\u23f8")  # pause glyph

    # Wait ~300ms of real time and confirm the mixer advanced.
    page.wait_for_timeout(350)
    after = page.evaluate("() => window.__viewer.action.time")
    assert after > 0.05, f"action.time did not advance after play: before={before} after={after}"

    # Readout reflects the advance.
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

    def sample(t: float) -> dict[str, list[float]]:
        """Return a representative mesh-vertex world position per body node.

        Transforming a non-origin probe point with ``matrixWorld`` captures
        both translation AND rotation — a pure-rotation body (e.g. ``conn``
        pivoting around the fixed O) still shows motion.
        """
        return page.evaluate(
            """(t) => {
                const v = window.__viewer;
                v.action.time = t;
                v.mixer.update(0);
                const root = v.mixer.getRoot();
                root.updateMatrixWorld(true);
                const out = {};
                for (const o of root.children) {
                    const m = o.matrixWorld.elements;
                    // Probe point (1, 1, 0) transformed by the 4x4 world matrix.
                    out[o.name] = [
                        m[0] + m[4] + m[12],
                        m[1] + m[5] + m[13],
                        m[2] + m[6] + m[14],
                    ];
                }
                return out;
            }""",
            t,
        )

    a = sample(0.0)
    b = sample(0.5)
    # torso is the fixed frame; coupler pivots on the fixed crank centre O
    # so a probe point tied to its single-joint anchor doesn't translate.
    # torso is world-fixed; coupler has a single anchor joint on the fixed
    # crank centre O, so its probe point doesn't translate. Names differ
    # with/without the _leg0 suffix depending on n_legs.
    static = {"torso", "torso_leg0", "coupler", "coupler_leg0"}
    moving = [n for n in a if n not in static]
    assert moving, f"no moving bodies found in {list(a)}"
    for name in moving:
        dx = max(abs(a[name][i] - b[name][i]) for i in range(3))
        assert dx > 1e-3, f"body {name!r} did not move between t=0 and t=0.5 (Δ={dx})"


def test_play_pause_toggle(page: Page, viewer_server: str) -> None:
    """Second click pauses: action.time stays put across the subsequent wait."""
    page.goto(viewer_server)
    _wait_ready(page)

    page.locator("#play").click()
    page.wait_for_timeout(200)
    page.locator("#play").click()  # pause
    expect(page.locator("#play")).to_have_text("\u25b6")  # play glyph

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


@pytest.mark.parametrize("mode_id", ["double", "double_double"])
def test_mode_toggle_swaps_assembly(page: Page, viewer_server: str, mode_id: str) -> None:
    """Selecting a different mode swaps the GLB and rebinds the mixer.

    The multi-leg assemblies produce strictly more animated bodies than the
    single-leg default, so the track count is a reliable swap indicator.
    """
    page.goto(viewer_server)
    _wait_ready(page)

    base_tracks = page.evaluate("() => window.__viewer.action.getClip().tracks.length")
    assert page.evaluate("() => window.__viewer.mode") == "klann"

    page.select_option("#mode", mode_id)
    page.wait_for_function(
        "(m) => window.__viewer && window.__viewer.mode === m && window.__viewer.action",
        arg=mode_id,
        timeout=20_000,
    )

    new_tracks = page.evaluate("() => window.__viewer.action.getClip().tracks.length")
    assert new_tracks > base_tracks, (
        f"mode {mode_id} should add tracks; base={base_tracks} new={new_tracks}"
    )
