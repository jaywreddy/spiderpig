"""Walking data for the drive-mode e2e tests.

:func:`walk_response` is ``GET /api/walk``'s body for a design, from the
walking model itself (:func:`walk.api_payload`).

:func:`flat_walk` is a synthetic gait with known answers: three legs a side,
120 deg apart, each foot on a flat stance line (y = -100) for 240 deg of the
crank, moving at -120 / (4 pi / 3) mm/rad.
"""

from __future__ import annotations

import math

N = 360
SERVO = {"key": "sts3215", "rpm_max": 52.0}


def walk_response(module: str = "quad", linkage: str = "klann") -> dict:
    """``GET /api/walk?module=<module>&linkage=<linkage>``'s body."""
    import walk

    return walk.api_payload(walk.make_config(module, linkage=linkage))


def flat_walk() -> dict:
    """The synthetic gait (see the module docstring), SPEC-shaped."""
    feet = []
    for side, z in (("L", -50.0), ("R", 50.0)):
        for leg in range(3):
            xy = []
            for i in range(N):
                a = (2 * math.pi * i / N + 2 * math.pi * leg / 3) % (2 * math.pi)
                if a < 4 * math.pi / 3:
                    xy.append([60 - 120 * a / (4 * math.pi / 3), -100.0])
                else:
                    u = (a - 4 * math.pi / 3) / (2 * math.pi / 3)
                    xy.append([-60 + 120 * u, -100 + 30 * math.sin(math.pi * u)])
            feet.append({"body": f"{side}.b4_leg{leg}", "side": side, "leg": leg, "z": z, "xy": xy})
    return {"theta_samples": N, "feet": feet, "com": [0.0, 0.0, 0.0], "servo": SERVO}
