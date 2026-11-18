"""Browser settings for the viewer e2e tests.

Headless Chromium renders WebGL in software (SwiftShader) unless told to use
the GPU through ANGLE's EGL path: SwiftShader draws the viewer's animation loop
at about 1 frame a second and holds up the page's input and sockets with it, far
too slow for a timed drive; with the GPU it runs at the display's rate. The EGL
path loads the system's EGL (``libegl1``, ``libegl-mesa0``): on a machine with
no GPU (a CI runner) that is mesa's llvmpipe, still slow to draw but off the
page's thread. Without that library Chromium falls back to SwiftShader silently,
so the ``browser`` fixture refuses SwiftShader and names the fix. The robot's
translucent, environment-mapped parts still cost per pixel, so a smaller
viewport keeps frames (and screenshots) quick.
"""

from __future__ import annotations

import pytest

GPU_ARGS = ["--use-angle=gl-egl", "--enable-gpu", "--ignore-gpu-blocklist"]
# Chromium can hang at start-up waiting on the desktop keyring (gnome-keyring / kwallet)
# on a machine with a session bus: keep its password store in-process. Playwright passes
# these today; stated here so an upgrade or another launcher can't bring the hang back.
KEYRING_ARGS = ["--password-store=basic", "--use-mock-keychain"]
RENDERER_JS = """() => {
    const gl = document.createElement('canvas').getContext('webgl2');
    if (!gl) return 'no WebGL2';
    const info = gl.getExtension('WEBGL_debug_renderer_info');
    return gl.getParameter(info ? info.UNMASKED_RENDERER_WEBGL : gl.RENDERER);
}"""


@pytest.fixture(scope="session")
def browser_type_launch_args(browser_type_launch_args):
    args = [*browser_type_launch_args.get("args", []), *GPU_ARGS, *KEYRING_ARGS]
    return {**browser_type_launch_args, "args": args}


@pytest.fixture(scope="session")
def browser_context_args(browser_context_args):
    return {**browser_context_args, "viewport": {"width": 960, "height": 600}}


@pytest.fixture(scope="session")
def browser(browser):
    """pytest-playwright's browser, once its WebGL is known not to be SwiftShader: the drive
    tests time out on it one by one (each a few minutes) instead of failing at once."""
    page = browser.new_page()
    try:
        renderer = page.evaluate(RENDERER_JS)
    finally:
        page.close()
    if "SwiftShader" in renderer:
        pytest.fail(f"WebGL renders on {renderer!r}: Chromium's EGL path found no system EGL "
                    "(install libegl1 and libegl-mesa0; no GPU: mesa's llvmpipe)", pytrace=False)
    return browser
