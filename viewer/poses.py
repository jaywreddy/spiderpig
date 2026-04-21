"""Per-frame rigid-body pose computation for the Klann viewer.

Each link in :class:`klann.KlannLinkage` is a rigid 2D body. Across a crank
phase sweep the joint world XY positions move (per
:func:`klann.create_klann_geometry`) but each body's intrinsic geometry is
fixed. So for every animation frame we just need to find the rigid SE(3)
transform that maps each body's reference (phase=1) joint positions to its
current (phase=p) joint positions. That's a 2D Kabsch alignment, lifted to
4×4 with z-rotation only.

The output dict from :func:`bake_frames` is the on-disk schema consumed by
``viewer/main.js``.
"""

from __future__ import annotations

import math

import numpy as np

# Geometry import is lazy so this module is testable without sympy installed
# at import time, and so kabsch_2d is reusable in tests.

REFERENCE_PHASE = 1.0

# Map body name -> joint names whose positions live in the analytic solution.
# Order matters for Kabsch consistency between reference and current frames.
BODY_JOINTS: dict[str, list[str]] = {
    "torso": ["A", "O", "B"],   # all ground-fixed -> identity always
    "coupler": ["O"],           # 1 joint, rotation derived from M
    "conn": ["O", "M"],
    "b1": ["M", "C", "D"],
    "b2": ["B", "E"],
    "b3": ["A", "C"],
    "b4": ["E", "D"],
}

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


def kabsch_2d(ref_xy: np.ndarray, new_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Closed-form 2D rigid alignment.

    Returns ``(R, t)`` such that ``new ≈ R @ ref + t``. ``R`` is 2×2 and
    proper (det = +1); ``t`` is length-2.
    """
    ref = np.asarray(ref_xy, dtype=float)
    new = np.asarray(new_xy, dtype=float)
    if ref.shape != new.shape or ref.ndim != 2 or ref.shape[1] != 2:
        raise ValueError(f"Need matching (N, 2) arrays, got {ref.shape} and {new.shape}")

    c_ref = ref.mean(axis=0)
    c_new = new.mean(axis=0)
    q_ref = ref - c_ref
    q_new = new - c_new

    h = q_ref.T @ q_new  # (2, 2)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    diag = np.diag([1.0, d])
    r = vt.T @ diag @ u.T

    t = c_new - r @ c_ref
    return r, t


def _lift_to_4x4(r2: np.ndarray, t2: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:2, :2] = r2
    m[:2, 3] = t2
    return m


def _rotz_4x4(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    m = np.eye(4)
    m[0, 0] = c
    m[0, 1] = -s
    m[1, 0] = s
    m[1, 1] = c
    return m


def body_pose_for_phase(
    body_name: str,
    ref_joints_xy: dict[str, tuple[float, float]],
    new_joints_xy: dict[str, tuple[float, float]],
) -> np.ndarray:
    """Return the 4×4 world pose for ``body_name`` at the current phase.

    ``ref_joints_xy`` and ``new_joints_xy`` map joint names (``A``, ``O``, …)
    to their world XY positions at the reference and current phase
    respectively.
    """
    joints = BODY_JOINTS[body_name]

    if body_name == "torso":
        return np.eye(4)

    if body_name == "coupler":
        # Single fixed joint at O; rotate about Z so the shaft tracks M.
        mx, my = new_joints_xy["M"]
        return _rotz_4x4(math.atan2(my, mx))

    ref = np.array([ref_joints_xy[j] for j in joints], dtype=float)
    new = np.array([new_joints_xy[j] for j in joints], dtype=float)
    r2, t2 = kabsch_2d(ref, new)
    return _lift_to_4x4(r2, t2)


def _solve_joint_positions(phase: float) -> tuple[dict[str, tuple[float, float]], tuple[float, float]]:
    """Evaluate the Klann solver at ``phase`` and return joint XY + foot tip XY."""
    from klann import create_klann_geometry  # lazy: avoid sympy on import

    O, A, B, C, D, E, F, M, *_ = create_klann_geometry(orientation=1, phase=phase)
    joints = {
        "O": (float(O.x.evalf()), float(O.y.evalf())),
        "A": (float(A.x.evalf()), float(A.y.evalf())),
        "B": (float(B.x.evalf()), float(B.y.evalf())),
        "C": (float(C.x.evalf()), float(C.y.evalf())),
        "D": (float(D.x.evalf()), float(D.y.evalf())),
        "E": (float(E.x.evalf()), float(E.y.evalf())),
        "M": (float(M.x.evalf()), float(M.y.evalf())),
    }
    foot = (float(F.x.evalf()), float(F.y.evalf()))
    return joints, foot


def bake_frames(n_frames: int = 60, fps: int = 30) -> dict:
    """Solve the linkage at ``n_frames`` phases over one full cycle.

    Returns the dict that gets serialised to ``viewer/data/frames.json``.
    Pose matrices are stored as flat 16-element lists in **column-major**
    order, matching three.js's ``Matrix4.elements`` layout.
    """
    ref_joints, _ = _solve_joint_positions(REFERENCE_PHASE)

    phases = np.linspace(0.0, 2.0 * math.pi, n_frames, endpoint=False)
    frames = []
    foot_path: list[list[float]] = []

    for p in phases:
        new_joints, foot = _solve_joint_positions(float(p))
        foot_path.append([foot[0], foot[1]])
        poses = {}
        for name in BODY_NAMES:
            m = body_pose_for_phase(name, ref_joints, new_joints)
            # column-major: three.js expects column 0, then column 1, ...
            poses[name] = m.T.flatten().tolist()
        frames.append({"phase": float(p), "poses": poses})

    return {
        "fps": fps,
        "n_frames": n_frames,
        "body_names": list(BODY_NAMES),
        "body_colors": dict(BODY_COLORS),
        "foot_path": foot_path,
        "frames": frames,
    }
