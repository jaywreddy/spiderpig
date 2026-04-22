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
import math
import sys
from pathlib import Path

import numpy as np
import pygltflib

# Make sibling modules importable when invoked as ``viewer/bake_gltf.py``.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from klann import (  # noqa: E402
    build_multi_leg_mechanism,
    create_klann_geometry,
)
from shapes import THICKNESS  # noqa: E402

# Per-class colour overrides (RGB 0-1). Mirror the prior viewer palette so
# the three.js scene looks the same after the STL→glTF swap.
_CLASS_COLORS: dict[str, tuple[float, float, float]] = {
    "torso": (0.831, 0.686, 0.000),     # #d4af00 yellow
    "coupler": (0.188, 0.376, 1.000),   # #3060ff blue
    "conn": (0.878, 0.439, 0.125),      # #e07020 orange
    "b1": (0.227, 0.659, 0.420),        # #3aa86b green
    "b2": (0.227, 0.659, 0.420),
    "b3": (0.227, 0.659, 0.420),
    "b4": (0.227, 0.659, 0.420),
}

_BODY_CLASSES = ("torso", "coupler", "conn", "b1", "b2", "b3", "b4")


def _class_of(body_name: str) -> str:
    """``"b1_leg3"`` -> ``"b1"``; ``"coupler_leg0"`` -> ``"coupler"``."""
    return body_name.rsplit("_leg", 1)[0]


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


def _quat_xyzw_from_z_rot(R: np.ndarray) -> np.ndarray:
    """Quaternion ``[qx, qy, qz, qw]`` for a rotation about Z embedded in ``R``."""
    theta = math.atan2(R[1, 0], R[0, 0])
    return np.array([0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)], dtype=float)


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
    verbose: bool = False,
) -> None:
    """Write ``<out>`` as a self-contained binary glTF describing the multi-leg
    Klann walker plus its TRS animation over one crank revolution.
    """
    log = print if verbose else (lambda *_a, **_k: None)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)

    log(f"[bake] reference mech (parts=True, n_legs={n_legs})…")
    ref_mech = build_multi_leg_mechanism(
        n_legs, t=0.0, thickness=thickness, with_parts=True
    )

    log("[bake] tessellate unique geometry classes…")
    class_parts: dict[str, object] = {}
    for body in ref_mech.bodies:
        cls = _class_of(body.name)
        if cls not in class_parts and body.part is not None:
            class_parts[cls] = body.part
    class_mesh: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for cls, part in class_parts.items():
        class_mesh[cls] = _tessellate(part)
        log(f"  {cls}: {len(class_mesh[cls][0])} verts, {len(class_mesh[cls][2])//3} tris")

    packer = _Packer()
    accessors: list[pygltflib.Accessor] = []

    # --- geometry: pack per-class position/normal/index accessors ---
    class_primitive: dict[str, pygltflib.Primitive] = {}
    class_material_idx: dict[str, int] = {}
    materials: list[pygltflib.Material] = []

    for cls in _BODY_CLASSES:
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

    # --- sample animation ---
    log(f"[bake] sampling {n_frames} frames over {duration_s:.3f}s…")
    ts = np.linspace(0.0, 2.0 * math.pi, n_frames, endpoint=False)
    times = np.linspace(0.0, duration_s, n_frames, endpoint=False, dtype=np.float32)

    body_names = [b.name for b in ref_mech.bodies]
    translations = {name: np.zeros((n_frames, 3), dtype=np.float32) for name in body_names}
    rotations = {name: np.zeros((n_frames, 4), dtype=np.float32) for name in body_names}
    prev_q: dict[str, np.ndarray | None] = dict.fromkeys(body_names)

    for fi, t_val in enumerate(ts):
        mech = build_multi_leg_mechanism(
            n_legs, float(t_val), thickness=thickness, with_parts=False
        ).solved()
        for body in mech.bodies:
            cls = _class_of(body.name)
            anchor = class_anchor_joints.get(cls)
            if anchor is None:
                # Body has no mesh; animation values are ignored.
                translations[body.name][fi] = 0.0
                rotations[body.name][fi] = (0.0, 0.0, 0.0, 1.0)
                continue
            now = _body_joint_world(body)
            names = [n for n in anchor if n in now]
            p0 = np.stack([anchor[n] for n in names], axis=0)
            p1 = np.stack([now[n] for n in names], axis=0)
            R, t_vec = _rigid_planar(p0, p1)
            q = _quat_xyzw_from_z_rot(R).astype(np.float32)
            prev = prev_q[body.name]
            if prev is not None and float(np.dot(prev, q)) < 0.0:
                q = -q
            translations[body.name][fi] = t_vec.astype(np.float32)
            rotations[body.name][fi] = q
            prev_q[body.name] = q

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

    # --- nodes + per-body animation samplers/channels ---
    nodes: list[pygltflib.Node] = []
    animation_samplers: list[pygltflib.AnimationSampler] = []
    animation_channels: list[pygltflib.AnimationChannel] = []

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

    # --- foot-path extra (leg 0 for reference) ---
    sol0 = create_klann_geometry(orientation=1, phase=0.0)
    foot_samples = 64
    foot_path = [
        list(sol0.joints_at(float(tv))["F"])
        for tv in np.linspace(0.0, 2.0 * math.pi, foot_samples, endpoint=False)
    ]

    scene = pygltflib.Scene(nodes=list(range(len(nodes))))
    scene.extras = {"foot_path": foot_path}

    # --- assemble glTF ---
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
    log(f"[bake] wrote {out} ({out.stat().st_size} B)")


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
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    bake_gltf(
        args.out,
        n_frames=args.frames,
        duration_s=args.duration,
        n_legs=args.legs,
        verbose=True,
    )


if __name__ == "__main__":
    main()
