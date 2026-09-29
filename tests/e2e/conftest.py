"""Browser settings for the viewer e2e tests.

Headless Chromium renders WebGL in software (SwiftShader); the robot's
translucent, environment-mapped parts cost per pixel, so a smaller viewport
keeps frames (and screenshots) quick.
"""

from __future__ import annotations

import pytest


@pytest.fixture(scope="session")
def browser_context_args(browser_context_args):
    return {**browser_context_args, "viewport": {"width": 960, "height": 600}}
