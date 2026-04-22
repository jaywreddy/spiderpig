"""Tests for the glTF bake pipeline."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pygltflib
import pytest

# viewer/ is a sibling of the package modules; make it importable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))

from bake_gltf import bake_gltf  # noqa: E402


def _read_accessor(gltf: pygltflib.GLTF2, accessor_idx: int) -> np.ndarray:
    blob = gltf.binary_blob()
    acc = gltf.accessors[accessor_idx]
    bv = gltf.bufferViews[acc.bufferView]
    offset = (bv.byteOffset or 0) + (acc.byteOffset or 0)

    comp = {
        pygltflib.FLOAT: ("f4", 4),
        pygltflib.UNSIGNED_INT: ("u4", 4),
        pygltflib.UNSIGNED_SHORT: ("u2", 2),
    }[acc.componentType]
    dtype = np.dtype(comp[0])

    ntype_components = {
        pygltflib.SCALAR: 1,
        pygltflib.VEC2: 2,
        pygltflib.VEC3: 3,
        pygltflib.VEC4: 4,
    }[acc.type]

    count = acc.count * ntype_components
    arr = np.frombuffer(blob, dtype=dtype, count=count, offset=offset)
    if ntype_components > 1:
        arr = arr.reshape(acc.count, ntype_components)
    return arr


def test_gltf_emits_valid_file(tmp_path):
    out = tmp_path / "klann.glb"
    bake_gltf(out, n_frames=4, n_legs=2, thickness=3.0, duration_s=0.5)
    assert out.is_file()
    assert out.stat().st_size > 1024

    gltf = pygltflib.GLTF2().load(str(out))

    # 7 unique meshes (torso, coupler, conn, b1..b4).
    assert len(gltf.meshes) == 7
    # 2 legs × 7 bodies = 14 nodes.
    assert len(gltf.nodes) == 14

    # one animation
    assert len(gltf.animations) == 1
    anim = gltf.animations[0]
    # 2 channels per body node: translation + rotation.
    assert len(anim.channels) == 14 * 2

    # foot_path tucked in scene.extras
    scene = gltf.scenes[gltf.scene]
    assert "foot_path" in (scene.extras or {})
    assert len(scene.extras["foot_path"]) == 64


def test_quaternion_shortest_path(tmp_path):
    out = tmp_path / "klann.glb"
    bake_gltf(out, n_frames=32, n_legs=1, thickness=3.0, duration_s=1.0)
    gltf = pygltflib.GLTF2().load(str(out))

    anim = gltf.animations[0]
    rotation_channels = [
        ch for ch in anim.channels if ch.target.path == "rotation"
    ]
    assert rotation_channels, "expected at least one rotation channel"

    for ch in rotation_channels:
        sampler = anim.samplers[ch.sampler]
        q = _read_accessor(gltf, sampler.output)
        assert q.shape == (32, 4)
        # Consecutive quaternions must land on the same hemisphere.
        dots = np.einsum("ij,ij->i", q[:-1], q[1:])
        assert np.all(dots >= -1e-6), (
            f"shortest-path violated on node {ch.target.node}: min dot {dots.min()}"
        )


def test_gltf_mesh_instancing(tmp_path):
    """All legs of the same class share a single mesh index."""
    out = tmp_path / "klann.glb"
    bake_gltf(out, n_frames=2, n_legs=3, thickness=3.0, duration_s=0.1)
    gltf = pygltflib.GLTF2().load(str(out))

    # gather mesh indices per class
    by_class: dict[str, set[int]] = {}
    for node in gltf.nodes:
        if node.mesh is None:
            continue
        cls = node.name.rsplit("_leg", 1)[0]
        by_class.setdefault(cls, set()).add(node.mesh)

    for cls, mesh_set in by_class.items():
        assert len(mesh_set) == 1, f"class {cls} uses {len(mesh_set)} meshes"


@pytest.mark.parametrize("n_legs", [1, 2])
def test_gltf_animation_duration(tmp_path, n_legs):
    out = tmp_path / f"klann_{n_legs}.glb"
    duration = 0.25
    bake_gltf(out, n_frames=8, n_legs=n_legs, thickness=3.0, duration_s=duration)
    gltf = pygltflib.GLTF2().load(str(out))

    # time accessor is shared across samplers; grab the input of the first.
    anim = gltf.animations[0]
    t_acc = gltf.accessors[anim.samplers[0].input]
    assert t_acc.count == 8
    # min/max bound the keyframe range.
    assert t_acc.min[0] == pytest.approx(0.0)
    assert t_acc.max[0] == pytest.approx(duration * 7 / 8)
