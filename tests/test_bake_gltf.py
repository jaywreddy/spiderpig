"""Tests for the glTF bake pipeline."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pygltflib
import pytest

# viewer/ is a sibling of the package modules; make it importable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))

from bake_gltf import _build_assembly, _mesh_key, bake_gltf  # noqa: E402


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


def _quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_gltf_emits_valid_file(tmp_path):
    out = tmp_path / "klann.glb"
    bake_gltf(out, n_frames=4, n_legs=2, thickness=3.0, duration_s=0.5, with_joinery=False)
    assert out.is_file()
    assert out.stat().st_size > 1024

    gltf = pygltflib.GLTF2().load(str(out))
    mech = _build_assembly("multi", t=0.0, n_legs=2, with_joinery=False)

    # one node per body; b1..b4 of both legs share 4 meshes, the rest are per-body
    assert [n.name for n in gltf.nodes] == [b.name for b in mech.bodies]
    with_part = [b.name for b in mech.bodies if b.part is not None]
    assert len(gltf.meshes) == len({_mesh_key(n) for n in with_part})
    assert {"b1", "b2", "b3", "b4"} <= {m.name for m in gltf.meshes}

    anim = gltf.animations[0]
    assert len(anim.channels) == 2 * len(gltf.nodes)  # translation + rotation

    scene = gltf.scenes[gltf.scene]
    assert len(scene.extras["foot_path"]) == 64


def test_gltf_joinery_adds_pins_and_caps(tmp_path):
    """Pins and press-on caps appear at every pivot only when joinery is on."""
    with_j = tmp_path / "with.glb"
    without = tmp_path / "without.glb"
    bake_gltf(with_j, n_frames=2, mode="single", duration_s=0.5)
    bake_gltf(without, n_frames=2, mode="single", duration_s=0.5, with_joinery=False)

    names = {n.name for n in pygltflib.GLTF2().load(str(with_j)).nodes}
    bare = {n.name for n in pygltflib.GLTF2().load(str(without)).nodes}
    # C, D, E link pins and the A, B frame pivots each get a pin + cap
    for axis in ("C", "D", "E"):
        assert {f"pin_{axis}", f"cap_{axis}"} <= names
    for axis in ("A", "B"):
        assert {f"frame_pin_{axis}", f"frame_cap_{axis}"} <= names
    assert not any(n.startswith(("pin_", "cap_", "frame_pin_")) for n in bare)
    # the crankshaft is structure, not joinery: present either way
    assert any(n.startswith("crankpin_") for n in bare)


def test_quaternion_shortest_path(tmp_path):
    out = tmp_path / "klann.glb"
    bake_gltf(out, n_frames=32, n_legs=1, thickness=3.0, duration_s=1.0)
    gltf = pygltflib.GLTF2().load(str(out))

    anim = gltf.animations[0]
    rotation_channels = [ch for ch in anim.channels if ch.target.path == "rotation"]
    assert rotation_channels, "expected at least one rotation channel"
    for ch in rotation_channels:
        q = _read_accessor(gltf, anim.samplers[ch.sampler].output)
        assert q.shape == (32, 4)
        dots = np.einsum("ij,ij->i", q[:-1], q[1:])
        assert np.all(dots >= -1e-6), (
            f"shortest-path violated on node {ch.target.node}: min dot {dots.min()}"
        )


def test_gltf_mesh_instancing(tmp_path):
    """Every leg's b1 (b2, b3, b4) instances one shared mesh."""
    out = tmp_path / "klann.glb"
    bake_gltf(out, n_frames=2, n_legs=3, thickness=3.0, duration_s=0.1)
    gltf = pygltflib.GLTF2().load(str(out))
    by_class: dict[str, set[int]] = {}
    for node in gltf.nodes:
        cls = node.name.rsplit("_leg", 1)[0]
        if node.mesh is not None and cls in {"b1", "b2", "b3", "b4"}:
            by_class.setdefault(cls, set()).add(node.mesh)
    assert set(by_class) == {"b1", "b2", "b3", "b4"}
    for cls, meshes in by_class.items():
        assert len(meshes) == 1, f"class {cls} uses {len(meshes)} meshes"


@pytest.mark.parametrize("mode", ["single", "double"])
def test_animation_reproduces_fabricated_geometry(tmp_path, mode):
    """Posing each node's mesh by its animation at frame k must land on the part
    fabricated directly at that crank angle (shared link meshes, slot Z offsets
    and hardware riding its host all included)."""
    n_frames = 12
    out = tmp_path / f"{mode}.glb"
    bake_gltf(out, n_frames=n_frames, mode=mode, duration_s=1.0)
    gltf = pygltflib.GLTF2().load(str(out))
    anim = gltf.animations[0]
    trs: dict[tuple[int, str], np.ndarray] = {}
    for ch in anim.channels:
        out_acc = anim.samplers[ch.sampler].output
        trs[(ch.target.node, ch.target.path)] = _read_accessor(gltf, out_acc)

    for frame in (0, 5):
        mech = _build_assembly(mode, t=2.0 * np.pi * frame / n_frames)
        parts = {b.name: b.part for b in mech.bodies if b.part is not None}
        for i, node in enumerate(gltf.nodes):
            if node.mesh is None:
                continue
            v = _read_accessor(gltf, gltf.meshes[node.mesh].primitives[0].attributes.POSITION)
            posed = v @ _quat_matrix(trs[(i, "rotation")][frame]).T + trs[(i, "translation")][frame]
            bb = parts[node.name].bounding_box()
            np.testing.assert_allclose(
                np.concatenate([posed.min(0), posed.max(0)]),
                [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z],
                atol=0.05, err_msg=f"{mode} {node.name} frame {frame}",
            )


@pytest.mark.parametrize("n_legs", [1, 2])
def test_gltf_animation_duration(tmp_path, n_legs):
    out = tmp_path / f"klann_{n_legs}.glb"
    duration = 0.25
    bake_gltf(out, n_frames=8, n_legs=n_legs, thickness=3.0, duration_s=duration)
    gltf = pygltflib.GLTF2().load(str(out))
    t_acc = gltf.accessors[gltf.animations[0].samplers[0].input]
    assert t_acc.count == 8
    assert t_acc.min[0] == pytest.approx(0.0)
    assert t_acc.max[0] == pytest.approx(duration * 7 / 8)
