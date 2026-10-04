"""Browser settings for the viewer e2e tests.

Headless Chromium renders WebGL in software (SwiftShader) unless told to use
the GPU through ANGLE's EGL path: software GL runs the viewer's animation loop
at about 2 frames a second, far too slow for a timed drive; with the GPU it
runs at the display's rate. The robot's translucent, environment-mapped parts
still cost per pixel, so a smaller viewport keeps frames (and screenshots)
quick.
"""

from __future__ import annotations

import pytest

GPU_ARGS = ["--use-angle=gl-egl", "--enable-gpu", "--ignore-gpu-blocklist"]


@pytest.fixture(scope="session")
def browser_type_launch_args(browser_type_launch_args):
    args = [*browser_type_launch_args.get("args", []), *GPU_ARGS]
    return {**browser_type_launch_args, "args": args}


@pytest.fixture(scope="session")
def browser_context_args(browser_context_args):
    return {**browser_context_args, "viewport": {"width": 960, "height": 600}}
