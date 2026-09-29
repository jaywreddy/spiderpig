"""SPEC-shaped walking data for the drive-mode e2e tests.

:func:`walk_response` stands in for ``GET /api/walk`` while the server has no
such endpoint: one side's joint trajectories from :mod:`klann` (mech frame,
360 crank angles), both sides' feet at the b4 plates' mid-planes in the
module's default layer plan (``z_nominal``) and a nominal centre of mass. It
carries no ``metrics``: the viewer then computes them from the feet.

:func:`flat_walk` is a synthetic gait with known answers: three legs a side,
120 deg apart, each foot on a flat stance line (y = -100) for 240 deg of the
crank, moving at -120 / (4 pi / 3) mm/rad.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from functools import cache

import numpy as np

from klann import MODULE_LEGS, POINTS, PROPORTIONS, create_klann_geometry

N = 360
LINKS = [["O", "M"], ["M", "D"], ["B", "E"], ["A", "C"], ["E", "F"], ["O", "A"], ["O", "B"]]
SERVO = {"key": "sts3215", "rpm_max": 52.0}


def _b4(module: str, k: int) -> str:
    return "b4" if module == "single" else f"b4_leg{k}"


@cache
def foot_z(module: str) -> tuple[float, ...]:
    """Left side: each leg's b4 mid-plane z (default plan, robot mid-plane at z = 0)."""
    from construction.robot import mid_plane
    from fabricate import BuildConfig, design_side, template_for

    cfg = BuildConfig(module=module)
    design = design_side(template_for(cfg), cfg)
    plan = design.plan
    return tuple(sum(plan.z(plan.layers[_b4(module, k)])) / 2 - mid_plane(design)
                 for k in range(len(MODULE_LEGS[module])))


def walk_response(query: Mapping[str, str] | None = None) -> tuple[int, dict]:
    """``(status, body)`` of ``GET /api/walk?<query>`` (422 for bad parameters)."""
    q = dict(query or {})
    module = q.get("module", "quad")
    if module not in MODULE_LEGS:
        return 422, {"detail": f"unknown module {module!r}"}
    legs_def = MODULE_LEGS[module]
    try:
        phases = ([float(v) for v in q["phases"].split(",")] if q.get("phases")
                  else [math.degrees(ph) for _, ph in legs_def])
        props = {k[2:]: float(v) for k, v in q.items() if k.startswith("p.")}
    except ValueError as e:
        return 422, {"detail": str(e)}
    if len(phases) != len(legs_def) or set(props) - set(PROPORTIONS):
        return 422, {"detail": "wrong phase count or unknown proportion"}
    thetas = np.linspace(0.0, 2 * math.pi, N, endpoint=False)
    body = {
        "valid": True, "error": None, "module": module, "phases_deg": phases,
        "proportions": {k: props.get(k, float(v)) for k, v in PROPORTIONS.items()},
        "theta_samples": N, "links": LINKS, "servo": SERVO, "z_nominal": True,
    }
    legs = []
    for k, ((orientation, _), ph) in enumerate(zip(legs_def, phases, strict=True)):
        leg = create_klann_geometry(orientation, math.radians(ph), props or None)
        with np.errstate(invalid="ignore"):
            pts = leg.evaluate(thetas)
        for name in POINTS:
            bad = ~np.isfinite(pts[name]).all(axis=1)
            if bad.any():
                deg = math.degrees(thetas[int(np.argmax(bad))])
                return 200, {**body, "valid": False,
                             "error": f"leg {k}: joint {name} can't close at {deg:.0f} deg"}
        legs.append({"leg": k, "orientation": orientation, "phase_deg": ph,
                     "joints": {n: pts[n].round(4).tolist() for n in POINTS}})
    zs = foot_z(module)
    body["legs"] = legs
    body["feet"] = [{"body": f"{side}.{_b4(module, leg['leg'])}", "side": side, "leg": leg["leg"],
                     "z": sign * z, "xy": leg["joints"]["F"]}
                    for side, sign in (("L", 1.0), ("R", -1.0))
                    for leg, z in zip(legs, zs, strict=True)]
    body["side_z"] = {"L": float(np.mean(zs)), "R": -float(np.mean(zs))}
    pivots = np.array([leg["joints"][j][0] for leg in legs for j in ("O", "A", "B")])
    body["com"] = [float(pivots[:, 0].mean()), float(pivots[:, 1].mean()), 0.0]
    return 200, body


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
