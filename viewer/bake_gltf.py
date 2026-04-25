"""Bake the Klann viewer data as a single self-contained ``klann.glb``.

Pipeline
--------
1. Build one reference :class:`Mechanism` at ``t=0`` with parts enabled.
2. Tessellate each *unique* body geometry class (``torso``, ``coupler``,
   ``conn``, ``b1``..``b4``) via ``build123d.Part.tessellate``; multiple
   legs reference the same mesh.
3. Sample ``t`` over ``[0, 2π)`` at ``n_frames`` time steps, solve the
   multi-leg mechanism at each step, and extract ``(translation,
   rotation_xyzw)`` per body. Enforce quaternion shortest-path continuity.
4. Emit a single glTF animation with 2 channels (T, R) per body node,
   LINEAR interpolation. Pack as binary ``.glb`` via ``pygltflib``.

The frontend (``viewer/main.js``) loads the ``.glb`` with
``THREE.GLTFLoader`` and plays the animation via ``THREE.AnimationMixer``.

Usage
-----
    uv run python viewer/bake_gltf.py
    uv run python viewer/bake_gltf.py --frames 120 --duration 1.5 --legs 8
"""

from __future__ import annotations

import argparse
import logging
import math
import re
import sys
import time
from collections import defaultdict
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pygltflib

logger = logging.getLogger("bake_gltf")


@dataclass
class _Profiler:
    """Lightweight perf recorder: nested wall-clock timers + counters + metrics.

    Durations are accumulated per label so the same bracket can be entered
    many times (e.g. once per animation frame) and reported as total / mean /
    p50 / p95.
    """

    enabled: bool = True
    _durations: dict[str, list[float]] = field(
        default_factory=lambda: defaultdict(list)
    )
    _counters: dict[str, int] = field(default_factory=lambda: defaultdict(int))
    _metrics: dict[str, float] = field(default_factory=dict)

    @contextmanager
    def timed(self, label: str):
        if not self.enabled:
            yield
            return
        start = time.perf_counter()
        try:
            yield
        finally:
            self._durations[label].append(time.perf_counter() - start)

    def bump(self, label: str, n: int = 1) -> None:
        if self.enabled:
            self._counters[label] += n

    def set_metric(self, key: str, value: float) -> None:
        if self.enabled:
            self._metrics[key] = float(value)

    def log_summary(self) -> None:
        if not self.enabled:
            return

        bake_total = sum(self._durations.get("bake_total", [])) or None

        rows = []
        for label, times_ in self._durations.items():
            n = len(times_)
            total = sum(times_)
            mean_ms = (total / n) * 1000.0 if n else 0.0
            ts = sorted(times_)
            p50 = ts[n // 2] * 1000.0 if n else 0.0
            p95 = ts[min(n - 1, int(n * 0.95))] * 1000.0 if n else 0.0
            pct = (total / bake_total * 100.0) if bake_total else 0.0
            rows.append((label, n, total, mean_ms, p50, p95, pct))
        rows.sort(key=lambda r: -r[2])

        lines = [
            "bake profile summary:",
            f"  {'label':40} {'calls':>6} {'total_s':>9} "
            f"{'mean_ms':>9} {'p50_ms':>9} {'p95_ms':>9} {'%bake':>6}",
            f"  {'-' * 40} {'-' * 6} {'-' * 9} {'-' * 9} "
            f"{'-' * 9} {'-' * 9} {'-' * 6}",
        ]
        for label, n, total, mean_ms, p50, p95, pct in rows:
            lines.append(
                f"  {label:40} {n:6d} {total:9.3f} "
                f"{mean_ms:9.3f} {p50:9.3f} {p95:9.3f} {pct:6.1f}"
            )
        if self._counters:
            lines.append("  counters:")
            for k, v in sorted(self._counters.items()):
                lines.append(f"    {k}: {v}")
        if self._metrics:
            lines.append("  metrics:")
            for k, v in sorted(self._metrics.items()):
                # Integers stay integers for readability (vert counts, bytes).
                if v == int(v):
                    lines.append(f"    {k}: {int(v)}")
                else:
                    lines.append(f"    {k}: {v:.3f}")

        logger.info("\n".join(lines))

# Make sibling modules importable when invoked as ``viewer/bake_gltf.py``.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import klann  # noqa: E402
from klann import (  # noqa: E402
    build_double_decker_klann,
    build_double_decker_template,
    build_double_double_decker_klann,
    build_double_double_decker_template,
    build_double_klann,
    build_double_template,
    build_klann_mechanism,
    build_klann_template,
    build_multi_leg_mechanism,
    build_multi_leg_template,
    create_klann_geometry,
)
from mechanism import MechanismTemplate  # noqa: E402
from shapes import THICKNESS  # noqa: E402


# Assembly builders keyed by CLI mode. The bake pipeline calls each once for
# a reference build with geometry, then once per frame without geometry for
# pure kinematic sampling.
def _build_assembly(mode: str, *, t: float, n_legs: int, thickness: float, with_parts: bool):
    if mode == "single":
        sol = create_klann_geometry(orientation=1, phase=0.0)
        return build_klann_mechanism(sol, t=t, thickness=thickness, with_parts=with_parts)
    if mode == "multi":
        return build_multi_leg_mechanism(
            n_legs, t=t, thickness=thickness, with_parts=with_parts
        )
    if mode == "double":
        return build_double_klann(t=t, thickness=thickness, with_parts=with_parts)
    if mode == "decker":
        return build_double_decker_klann(t=t, thickness=thickness, with_parts=with_parts)
    if mode == "quad":
        return build_double_double_decker_klann(
            t=t, thickness=thickness, with_parts=with_parts
        )
    raise ValueError(f"unknown mode: {mode!r}")


def _build_template(mode: str, *, n_legs: int, thickness: float) -> MechanismTemplate:
    """Parametric-in-t dispatcher. Built once per bake; sampled over all frames."""
    if mode == "single":
        sol = create_klann_geometry(orientation=1, phase=0.0)
        return build_klann_template(sol, thickness=thickness)
    if mode == "multi":
        return build_multi_leg_template(n_legs, thickness=thickness)
    if mode == "double":
        return build_double_template(thickness=thickness)
    if mode == "decker":
        return build_double_decker_template(thickness=thickness)
    if mode == "quad":
        return build_double_double_decker_template(thickness=thickness)
    raise ValueError(f"unknown mode: {mode!r}")

# Per-class colour overrides (RGB 0-1). Mirror the prior viewer palette so
# the three.js scene looks the same after the STL→glTF swap.
_CLASS_COLORS: dict[str, tuple[float, float, float]] = {
    "torso": (0.831, 0.686, 0.000),       # #d4af00 yellow
    "coupler": (0.188, 0.376, 1.000),     # #3060ff blue
    "conn": (0.878, 0.439, 0.125),        # #e07020 orange
    "conn_upper": (0.878, 0.439, 0.125),  # upper-deck combined crank, same orange
    "b1": (0.227, 0.659, 0.420),          # #3aa86b green
    "b2": (0.227, 0.659, 0.420),
    "b3": (0.227, 0.659, 0.420),
    "b4": (0.227, 0.659, 0.420),
    "standoff": (0.831, 0.686, 0.000),    # match torso palette
}

_BODY_CLASSES = (
    "torso", "coupler", "conn", "conn_upper", "b1", "b2", "b3", "b4", "standoff",
)

_STANDOFF_RE = re.compile(r"^(standoff)\d+$")
# Joinery body names follow ``{name_prefix}{index}_{piece_name}`` (see
# joinery.py). Collapse the per-instance index so all instances of the same
# joinery type share one mesh class — matches what _STANDOFF_RE does for the
# legacy standoff bodies.
_JOINERY_RE = re.compile(r"^([a-z][a-z_]*?)(\d+)_([a-z_]+)$")


def _class_of(body_name: str) -> str:
    """Strip leg/index suffixes: ``"b1_leg3"`` -> ``"b1"``, ``"standoff2"`` -> ``"standoff"``,
    ``"clevis_pin0_pin"`` -> ``"clevis_pin_pin"``."""
    base = body_name.rsplit("_leg", 1)[0]
    m = _STANDOFF_RE.match(base)
    if m:
        return m.group(1)
    m = _JOINERY_RE.match(base)
    if m:
        return f"{m.group(1)}_{m.group(3)}"
    return base


def _body_joint_world(body) -> dict[str, np.ndarray]:
    """World-space XYZ of each joint on ``body`` under its current pose."""
    return {
        j.name: np.asarray((body.pose @ j.pose).matrix[:3, 3], dtype=float)
        for j in body.joints
    }


def _rigid_planar(p0: np.ndarray, p1: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid 2.5D transform (XY rotation + XYZ translation) mapping ``p0`` onto ``p1``.

    ``p0``, ``p1`` are ``(N, 3)``. The Klann mechanism is planar: bodies
    rotate only about the Z axis, and Z offsets come from per-joint layers.
    Uses the 2D closed-form Procrustes solution over XY and a straight
    centroid-delta for Z.
    """
    n = p0.shape[0]
    c0 = p0.mean(axis=0)
    c1 = p1.mean(axis=0)
    if n == 1:
        # Under-constrained rotation → pure translation.
        return np.eye(3), c1 - c0
    v0 = p0 - c0
    v1 = p1 - c1
    sxy = float((v0[:, 0] * v1[:, 1] - v0[:, 1] * v1[:, 0]).sum())
    cxy = float((v0[:, 0] * v1[:, 0] + v0[:, 1] * v1[:, 1]).sum())
    theta = math.atan2(sxy, cxy)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    t = c1 - R @ c0
    return R, t


def _rigid_planar_batch(
    p0: np.ndarray, p1: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized :func:`_rigid_planar` over a ``(T, N, 3)`` batch.

    Returns ``(theta, translations, quaternions_xyzw)`` where ``theta`` is
    the Z-rotation angle in radians shape ``(T,)``, translations are
    ``(T, 3)`` and quaternions are ``(T, 4)`` in ``[qx, qy, qz, qw]`` order.
    """
    t_count, n, _ = p0.shape
    c0 = p0.mean(axis=1)                                # (T, 3)
    c1 = p1.mean(axis=1)                                # (T, 3)
    if n == 1:
        theta = np.zeros(t_count, dtype=float)
        trans = c1 - c0
    else:
        v0 = p0 - c0[:, None, :]                        # (T, N, 3)
        v1 = p1 - c1[:, None, :]
        sxy = (v0[..., 0] * v1[..., 1] - v0[..., 1] * v1[..., 0]).sum(axis=1)
        cxy = (v0[..., 0] * v1[..., 0] + v0[..., 1] * v1[..., 1]).sum(axis=1)
        theta = np.arctan2(sxy, cxy)                    # (T,)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        # Apply R(theta) to each c0: (T, 3) ← 2D rotation on XY, Z passthrough.
        rot_c0 = np.stack(
            [cos_t * c0[:, 0] - sin_t * c0[:, 1],
             sin_t * c0[:, 0] + cos_t * c0[:, 1],
             c0[:, 2]],
            axis=1,
        )
        trans = c1 - rot_c0
    half = theta / 2.0
    q = np.stack(
        [np.zeros_like(theta), np.zeros_like(theta), np.sin(half), np.cos(half)],
        axis=1,
    )
    return theta, trans, q


def _quat_xyzw_from_z_rot(R: np.ndarray) -> np.ndarray:
    """Quaternion ``[qx, qy, qz, qw]`` for a rotation about Z embedded in ``R``."""
    theta = math.atan2(R[1, 0], R[0, 0])
    return np.array([0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)], dtype=float)


def _quat_hemisphere_continuous(q: np.ndarray) -> np.ndarray:
    """Flip signs of ``(T, 4)`` quaternions so adjacent samples stay in the
    same hemisphere (avoids the 2π ambiguity during LINEAR interpolation)."""
    q = q.copy()
    for i in range(1, q.shape[0]):
        if float(np.dot(q[i - 1], q[i])) < 0.0:
            q[i] = -q[i]
    return q


def _tessellate(part, tolerance: float = 0.1) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return ``(positions (nv,3) float32, normals (nv,3) float32, indices (nt*3,) uint32)``.

    Normals are smooth-shaded (per-vertex average of adjacent face normals).
    Three.js can override to flat shading client-side if desired.
    """
    verts, tris = part.tessellate(tolerance=tolerance)
    positions = np.array([(v.X, v.Y, v.Z) for v in verts], dtype=np.float32)
    triangles = np.array(tris, dtype=np.uint32)

    # Per-face normal via cross product.
    p0 = positions[triangles[:, 0]]
    p1 = positions[triangles[:, 1]]
    p2 = positions[triangles[:, 2]]
    face_n = np.cross(p1 - p0, p2 - p0)
    face_len = np.linalg.norm(face_n, axis=1, keepdims=True)
    face_n = np.where(face_len > 0, face_n / np.where(face_len > 0, face_len, 1.0), 0.0)

    # Accumulate per-vertex.
    normals = np.zeros_like(positions)
    for i in range(3):
        np.add.at(normals, triangles[:, i], face_n)
    vlen = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.where(vlen > 0, normals / np.where(vlen > 0, vlen, 1.0), 0.0)

    return positions.astype(np.float32), normals.astype(np.float32), triangles.flatten()


class _Packer:
    """Append-only binary blob that hands out :class:`pygltflib.BufferView` indices."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.buffer_views: list[pygltflib.BufferView] = []

    def add(self, data: bytes, *, target: int | None = None) -> int:
        # glTF requires 4-byte-aligned bufferViews.
        pad = (4 - (len(self._buf) % 4)) % 4
        if pad:
            self._buf.extend(b"\x00" * pad)
        byte_offset = len(self._buf)
        self._buf.extend(data)
        bv = pygltflib.BufferView(
            buffer=0, byteOffset=byte_offset, byteLength=len(data)
        )
        if target is not None:
            bv.target = target
        self.buffer_views.append(bv)
        return len(self.buffer_views) - 1

    @property
    def bytes(self) -> bytes:
        return bytes(self._buf)


def _accessor_for(
    bv_index: int,
    count: int,
    *,
    component_type: int,
    accessor_type: str,
    min_vals: list[float] | None = None,
    max_vals: list[float] | None = None,
) -> pygltflib.Accessor:
    acc = pygltflib.Accessor(
        bufferView=bv_index,
        componentType=component_type,
        count=count,
        type=accessor_type,
    )
    if min_vals is not None:
        acc.min = [float(v) for v in min_vals]
    if max_vals is not None:
        acc.max = [float(v) for v in max_vals]
    return acc


def bake_gltf(
    out: Path,
    *,
    n_frames: int = 120,
    duration_s: float = 1.0,
    n_legs: int = 1,
    thickness: float = THICKNESS,
    mode: str = "multi",
    verbose: bool = False,
    profile: bool = True,
    cprofile_out: Path | None = None,
) -> None:
    """Write ``<out>`` as a self-contained binary glTF describing the Klann
    walker plus its TRS animation over one crank revolution.

    ``mode`` selects which assembly to bake: ``single``/``multi`` (default,
    phase-stacked N legs), or ``double``/``decker``/``quad`` from the
    2016-style assembly builders.

    Profiling
    ---------
    When ``profile`` is true, per-stage wall-clock times, call counts and
    output-size metrics are emitted via the ``bake_gltf`` logger at INFO
    level. Set ``cprofile_out`` to a path to additionally dump a
    ``cProfile`` .prof file (plus a ``<path>.txt`` of the top-30 cumulative
    hot functions) for deep dives.
    """
    if verbose and logger.level > logging.DEBUG:
        logger.setLevel(logging.DEBUG)

    prof = _Profiler(enabled=profile)

    pr = None
    if cprofile_out is not None:
        import cProfile  # noqa: PLC0415
        pr = cProfile.Profile()
        pr.enable()

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)

    logger.info(
        "bake: mode=%s n_legs=%d n_frames=%d duration_s=%.3f out=%s",
        mode, n_legs, n_frames, duration_s, out,
    )
    prof.set_metric("n_frames", n_frames)
    prof.set_metric("n_legs", n_legs)

    klann._BAKE_PROFILER = prof if profile else None
    try:
        with prof.timed("bake_total"):
            # --- stage 1: reference mechanism build (with parts) ---
            with prof.timed("1_reference_build"):
                ref_mech = _build_assembly(
                    mode, t=0.0, n_legs=n_legs, thickness=thickness, with_parts=True
                )
            prof.set_metric("n_bodies", len(ref_mech.bodies))
            logger.debug("reference mech: %d bodies", len(ref_mech.bodies))

            # --- stage 2: tessellate one mesh per unique body class ---
            logger.debug("tessellating unique geometry classes…")
            class_parts: dict[str, object] = {}
            for body in ref_mech.bodies:
                cls = _class_of(body.name)
                if cls not in class_parts and body.part is not None:
                    class_parts[cls] = body.part

            class_mesh: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
            with prof.timed("2_tessellate_total"):
                for cls, part in class_parts.items():
                    with prof.timed(f"2_tessellate.{cls}"):
                        class_mesh[cls] = _tessellate(part)
                    nv = len(class_mesh[cls][0])
                    nt = len(class_mesh[cls][2]) // 3
                    prof.set_metric(f"verts.{cls}", nv)
                    prof.set_metric(f"tris.{cls}", nt)
                    logger.debug("  %s: %d verts, %d tris", cls, nv, nt)

            # --- stage 3: pack per-class geometry accessors + materials ---
            packer = _Packer()
            accessors: list[pygltflib.Accessor] = []
            class_primitive: dict[str, pygltflib.Primitive] = {}
            class_material_idx: dict[str, int] = {}
            materials: list[pygltflib.Material] = []

            with prof.timed("3_gltf_pack_geometry"):
                # Pack canonical classes first for deterministic ordering, then
                # any joinery (or other) classes the mechanism introduced.
                ordered_classes = list(_BODY_CLASSES) + [
                    c for c in class_mesh if c not in _BODY_CLASSES
                ]
                for cls in ordered_classes:
                    if cls not in class_mesh:
                        continue
                    positions, normals, indices = class_mesh[cls]

                    pos_bv = packer.add(positions.tobytes(), target=pygltflib.ARRAY_BUFFER)
                    pos_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            pos_bv, len(positions),
                            component_type=pygltflib.FLOAT,
                            accessor_type=pygltflib.VEC3,
                            min_vals=positions.min(axis=0).tolist(),
                            max_vals=positions.max(axis=0).tolist(),
                        )
                    )

                    nrm_bv = packer.add(normals.tobytes(), target=pygltflib.ARRAY_BUFFER)
                    nrm_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            nrm_bv, len(normals),
                            component_type=pygltflib.FLOAT,
                            accessor_type=pygltflib.VEC3,
                        )
                    )

                    idx_bv = packer.add(indices.tobytes(), target=pygltflib.ELEMENT_ARRAY_BUFFER)
                    idx_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            idx_bv, len(indices),
                            component_type=pygltflib.UNSIGNED_INT,
                            accessor_type=pygltflib.SCALAR,
                        )
                    )

                    color = _CLASS_COLORS.get(cls, (0.55, 0.55, 0.55))
                    material_idx = len(materials)
                    materials.append(
                        pygltflib.Material(
                            name=cls,
                            pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(
                                baseColorFactor=[color[0], color[1], color[2], 1.0],
                                metallicFactor=0.15,
                                roughnessFactor=0.75,
                            ),
                            doubleSided=True,
                        )
                    )
                    class_material_idx[cls] = material_idx

                    class_primitive[cls] = pygltflib.Primitive(
                        attributes=pygltflib.Attributes(POSITION=pos_acc, NORMAL=nrm_acc),
                        indices=idx_acc,
                        material=material_idx,
                        mode=pygltflib.TRIANGLES,
                    )

                meshes: list[pygltflib.Mesh] = []
                class_mesh_idx: dict[str, int] = {}
                for cls, prim in class_primitive.items():
                    class_mesh_idx[cls] = len(meshes)
                    meshes.append(pygltflib.Mesh(name=cls, primitives=[prim]))

            # --- canonical anchors: joint world positions when each class was tessellated ---
            #
            # Body parts were tessellated once from ref_mech at t=0, in *world coords*.
            # That makes the class mesh carry the body's t=0 pose already, so Mechanism.solved()
            # returns identity transforms — there's no motion left to animate.
            #
            # To recover motion, for each body at each frame we compute the rigid transform
            # that maps its anchor joints (leg-0 @ t=0 of the same class) onto its current
            # joint positions. Single-joint bodies (the coupler) degrade to pure translation.
            class_anchor_joints: dict[str, dict[str, np.ndarray]] = {}
            for body in ref_mech.bodies:
                cls = _class_of(body.name)
                if cls not in class_anchor_joints and body.part is not None:
                    class_anchor_joints[cls] = _body_joint_world(body)

            # --- stage 4: sample animation ---
            #
            # Build one parametric-in-t template and evaluate it for every frame
            # in a single vectorized pass. Stages 1-2 of the pipeline
            # (symbolic SymPy build + lambdify) run exactly once per leg here,
            # instead of once per leg per frame. The per-frame cost collapses
            # to batched numpy.
            logger.debug("sampling %d frames over %.3fs…", n_frames, duration_s)
            ts = np.linspace(0.0, 2.0 * math.pi, n_frames, endpoint=False)
            times = np.linspace(0.0, duration_s, n_frames, endpoint=False, dtype=np.float32)

            body_names = [b.name for b in ref_mech.bodies]
            translations = {name: np.zeros((n_frames, 3), dtype=np.float32) for name in body_names}
            rotations = {name: np.zeros((n_frames, 4), dtype=np.float32) for name in body_names}

            with prof.timed("4_animation_sample_total"):
                with prof.timed("4.1_template_build"):
                    template = _build_template(
                        mode, n_legs=n_legs, thickness=thickness
                    )
                with prof.timed("4.2_template_sample"):
                    sampled = template.sample(ts)
                with prof.timed("4.3_trs_batch"):
                    for body_name in body_names:
                        prof.bump("body_extract.calls")
                        cls = _class_of(body_name)
                        anchor = class_anchor_joints.get(cls)
                        body_joints = sampled.joint_world.get(body_name)
                        if anchor is None or body_joints is None:
                            rotations[body_name][:, 3] = 1.0
                            prof.bump("body_extract.skipped_no_mesh")
                            continue
                        names = [n for n in anchor if n in body_joints]
                        if not names:
                            # Class mesh was tessellated from a differently-shaped
                            # body of the same name (mechanism-side naming bug in
                            # e.g. 'quad' conn). Fall back to identity so the bake
                            # doesn't fail — that body will stay at the class
                            # canonical pose.
                            rotations[body_name][:, 3] = 1.0
                            prof.bump("body_extract.skipped_no_anchor_match")
                            continue
                        anchor_xyz = np.stack([anchor[n] for n in names], axis=0)
                        p0 = np.broadcast_to(
                            anchor_xyz, (n_frames, anchor_xyz.shape[0], 3)
                        )
                        p1 = np.stack([body_joints[n] for n in names], axis=1)
                        _theta, trans, q = _rigid_planar_batch(p0, p1)
                        q = _quat_hemisphere_continuous(q)
                        translations[body_name] = trans.astype(np.float32)
                        rotations[body_name] = q.astype(np.float32)

            # --- shared time accessor ---
            time_bv = packer.add(times.tobytes())
            time_acc = len(accessors)
            accessors.append(
                _accessor_for(
                    time_bv, n_frames,
                    component_type=pygltflib.FLOAT,
                    accessor_type=pygltflib.SCALAR,
                    min_vals=[float(times.min())],
                    max_vals=[float(times.max())],
                )
            )

            # --- stage 5: nodes + per-body animation samplers/channels ---
            nodes: list[pygltflib.Node] = []
            animation_samplers: list[pygltflib.AnimationSampler] = []
            animation_channels: list[pygltflib.AnimationChannel] = []

            with prof.timed("5_gltf_nodes_channels"):
                for body in ref_mech.bodies:
                    cls = _class_of(body.name)
                    mesh_idx = class_mesh_idx.get(cls)

                    initial_t = translations[body.name][0]
                    initial_q = rotations[body.name][0]

                    node = pygltflib.Node(
                        name=body.name,
                        translation=[float(initial_t[0]), float(initial_t[1]), float(initial_t[2])],
                        rotation=[float(initial_q[0]), float(initial_q[1]),
                                  float(initial_q[2]), float(initial_q[3])],
                    )
                    if mesh_idx is not None:
                        node.mesh = mesh_idx
                    node_idx = len(nodes)
                    nodes.append(node)

                    # translation sampler / channel
                    t_bv = packer.add(translations[body.name].tobytes())
                    t_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            t_bv, n_frames,
                            component_type=pygltflib.FLOAT,
                            accessor_type=pygltflib.VEC3,
                        )
                    )
                    trs_sampler_idx = len(animation_samplers)
                    animation_samplers.append(
                        pygltflib.AnimationSampler(
                            input=time_acc, output=t_acc, interpolation="LINEAR"
                        )
                    )
                    animation_channels.append(
                        pygltflib.AnimationChannel(
                            sampler=trs_sampler_idx,
                            target=pygltflib.AnimationChannelTarget(
                                node=node_idx, path="translation"
                            ),
                        )
                    )

                    # rotation sampler / channel
                    r_bv = packer.add(rotations[body.name].tobytes())
                    r_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            r_bv, n_frames,
                            component_type=pygltflib.FLOAT,
                            accessor_type=pygltflib.VEC4,
                        )
                    )
                    rot_sampler_idx = len(animation_samplers)
                    animation_samplers.append(
                        pygltflib.AnimationSampler(
                            input=time_acc, output=r_acc, interpolation="LINEAR"
                        )
                    )
                    animation_channels.append(
                        pygltflib.AnimationChannel(
                            sampler=rot_sampler_idx,
                            target=pygltflib.AnimationChannelTarget(
                                node=node_idx, path="rotation"
                            ),
                        )
                    )

            animation = pygltflib.Animation(
                name="walk", samplers=animation_samplers, channels=animation_channels
            )

            # --- stage 6: foot-path extra (leg 0 for reference) ---
            with prof.timed("6_foot_path_extra"):
                sol0 = create_klann_geometry(orientation=1, phase=0.0)
                foot_samples = 64
                foot_path = [
                    list(sol0.joints_at(float(tv))["F"])
                    for tv in np.linspace(0.0, 2.0 * math.pi, foot_samples, endpoint=False)
                ]

            scene = pygltflib.Scene(nodes=list(range(len(nodes))))
            scene.extras = {"foot_path": foot_path}

            # --- stage 7: assemble + binary-serialize glTF ---
            with prof.timed("7_serialize"):
                blob = packer.bytes
                gltf = pygltflib.GLTF2(
                    asset=pygltflib.Asset(version="2.0", generator="spiderpig/bake_gltf"),
                    scene=0,
                    scenes=[scene],
                    nodes=nodes,
                    meshes=meshes,
                    materials=materials,
                    accessors=accessors,
                    bufferViews=packer.buffer_views,
                    buffers=[pygltflib.Buffer(byteLength=len(blob))],
                    animations=[animation],
                )
                gltf.set_binary_blob(blob)
                gltf.save_binary(str(out))

            prof.set_metric("blob_bytes", len(blob))
            prof.set_metric("gltf_bytes", out.stat().st_size)
            prof.set_metric("animation_channels", len(animation_channels))
            prof.set_metric("accessors", len(accessors))
    finally:
        klann._BAKE_PROFILER = None
        if pr is not None:
            pr.disable()
            cprofile_out = Path(cprofile_out)
            cprofile_out.parent.mkdir(parents=True, exist_ok=True)
            pr.dump_stats(str(cprofile_out))
            txt_path = cprofile_out.with_suffix(cprofile_out.suffix + ".txt")
            import pstats  # noqa: PLC0415
            with open(txt_path, "w") as f:
                pstats.Stats(str(cprofile_out), stream=f).sort_stats(
                    "cumulative"
                ).print_stats(30)
            logger.info("cProfile dumped to %s (top-30 in %s)", cprofile_out, txt_path)

        # Peak resident set (linux: ru_maxrss is KB; mac: bytes — treat as linux here).
        try:
            import resource  # noqa: PLC0415
            ru = resource.getrusage(resource.RUSAGE_SELF)
            prof.set_metric("peak_rss_mb", ru.ru_maxrss / 1024.0)
        except ImportError:
            pass

        prof.log_summary()

    logger.info("wrote %s (%d B)", out, out.stat().st_size)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--out",
        type=Path,
        default=_REPO_ROOT / "viewer" / "data" / "klann.glb",
        help="Output .glb path (default: viewer/data/klann.glb).",
    )
    p.add_argument("--frames", type=int, default=120, help="Animation frame count.")
    p.add_argument(
        "--duration", type=float, default=1.0, help="Animation duration in seconds."
    )
    p.add_argument("--legs", type=int, default=1, help="Number of legs to stack in Z.")
    p.add_argument(
        "--mode",
        choices=["single", "multi", "double", "decker", "quad"],
        default="multi",
        help="Assembly kind (default: multi — N phase-stacked legs).",
    )
    p.add_argument(
        "--profile",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Emit per-stage wall-clock profile summary (default: on).",
    )
    p.add_argument(
        "--cprofile",
        type=Path,
        default=None,
        metavar="PATH",
        help="Also dump a cProfile .prof file plus <PATH>.txt top-30 report.",
    )
    p.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO).",
    )
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    bake_gltf(
        args.out,
        n_frames=args.frames,
        duration_s=args.duration,
        n_legs=args.legs,
        mode=args.mode,
        profile=args.profile,
        cprofile_out=args.cprofile,
    )


if __name__ == "__main__":
    main()
