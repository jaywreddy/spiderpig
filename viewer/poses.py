"""Per-frame rigid-body pose computation for the Klann viewer.

Each animation frame re-solves the Klann :class:`mechanism.Mechanism` at a
given crank phase and reads off each body's 4×4 world pose — the same SE(3)
math build123d uses to stack links correctly in the STEP export. This keeps
the viewer, the STEP file, and the downstream physics consumers all on one
source of truth.

The output dict from :func:`bake_frames` is the on-disk schema consumed by
``viewer/main.js``.
"""

from __future__ import annotations

import math

import numpy as np

BODY_COLORS: dict[str, str] = {
    "torso": "#d4af00",
    "coupler": "#3060ff",
    "conn": "#e07020",
    "b1": "#3aa86b",
    "b2": "#3aa86b",
    "b3": "#3aa86b",
    "b4": "#3aa86b",
}

# Public render order (also the body_names list shipped to JS).
BODY_NAMES: list[str] = ["torso", "coupler", "conn", "b1", "b2", "b3", "b4"]


def _foot_xy(phase: float) -> tuple[float, float]:
    from klann import create_klann_geometry  # lazy: avoid sympy on import

    _O, _A, _B, _C, _D, _E, F, *_ = create_klann_geometry(orientation=1, phase=phase)
    return float(F.x.evalf()), float(F.y.evalf())


def bake_frames(n_frames: int = 60, fps: int = 30) -> dict:
    """Solve the linkage at ``n_frames`` phases over one full cycle.

    Returns the dict that gets serialised to ``viewer/data/frames.json``.
    Pose matrices are stored as flat 16-element lists in **column-major**
    order, matching three.js's ``Matrix4.elements`` layout.
    """
    from klann import solve_klann_at_phase  # lazy: avoid sympy on import

    phases = np.linspace(0.0, 2.0 * math.pi, n_frames, endpoint=False)
    frames = []
    foot_path: list[list[float]] = []

    for p in phases:
        mech = solve_klann_at_phase(float(p), with_parts=False).solved()
        poses = {}
        for name in BODY_NAMES:
            m = mech.body(name).pose.matrix
            # column-major: three.js expects column 0, then column 1, ...
            poses[name] = m.T.flatten().tolist()
        foot_path.append(list(_foot_xy(float(p))))
        frames.append({"phase": float(p), "poses": poses})

    return {
        "fps": fps,
        "n_frames": n_frames,
        "body_names": list(BODY_NAMES),
        "body_colors": dict(BODY_COLORS),
        "foot_path": foot_path,
        "frames": frames,
    }
