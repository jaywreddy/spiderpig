"""Tests for the glTF bake pipeline.

Two bakes are shared by the module: one side with a single leg (``single``)
and the robot with a single leg per side (``robot`` / ``module="single"``);
a third, the robot with its leg's crank phase moved (``phased``), is baked
for the design-parameter tests only, and a fourth, a Strider robot
(``strider``), for another linkage's.
"""

from __future__ import annotations

import logging
import math
from dataclasses import replace

import numpy as np
import pygltflib
import pytest

from spiderpig import linkage, walk
from spiderpig.bake import _MATERIALS, _congruent, _Planar, bake_gltf
from spiderpig.config import BuildConfig, ParamError
from spiderpig.fabricate import fabricate, template_for
from spiderpig.hardware.mass import part_props

pytestmark = pytest.mark.slow

KLANN = linkage.get("klann")
N_FRAMES = 12
DURATION = 0.5
CASES = {
    "side": BuildConfig(linkage="klann", module="single", robot=False),
    "robot": BuildConfig(linkage="klann", module="single"),
}
EXTRA = {"phased": BuildConfig(linkage="klann", module="single", phases=(math.pi / 2,)),
         "strider": BuildConfig(linkage="strider", module="single"),
         "mechanism": BuildConfig(linkage="crank_rocker", module="single", robot=False)}
PHASED = EXTRA["phased"]


def _assembly(config: BuildConfig, t: float):
    """The fabricated walker at crank angle ``t`` (parts in world coordinates)."""
    return fabricate(template_for(config), config, t)


class _Bakes(dict):
    """``case -> GLTF2``, each case baked once per module; ``logs[case]`` its log."""

    def __init__(self, root) -> None:
        super().__init__()
        self.root, self.logs = root, {}

    def __call__(self, case: str) -> pygltflib.GLTF2:
        if case not in self:
            out = self.root.mktemp("glb") / f"{case}.glb"
            lines: list[str] = []
            handler = logging.Handler()
            handler.emit = lambda record: lines.append(record.getMessage())
            log = logging.getLogger("bake_gltf")
            log.addHandler(handler)
            level = log.level
            log.setLevel(logging.INFO)
            try:
                bake_gltf(out, EXTRA.get(case) or CASES[case], n_frames=N_FRAMES,
                          duration_s=DURATION)
            finally:
                log.removeHandler(handler)
                log.setLevel(level)
            self.logs[case] = "\n".join(lines) + "\n"
            self[case] = pygltflib.GLTF2().load(str(out))
        return self[case]


@pytest.fixture(scope="module")
def bakes(tmp_path_factory):
    return _Bakes(tmp_path_factory)


@pytest.fixture(params=list(CASES))
def baked(request, bakes):
    """``(case, config, gltf)`` for each shared bake."""
    return request.param, CASES[request.param], bakes(request.param)


@pytest.fixture
def robot_gltf(bakes):
    return bakes("robot")


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


def _tracks(gltf) -> dict[tuple[int, str], np.ndarray]:
    anim = gltf.animations[0]
    return {
        (ch.target.node, ch.target.path): _read_accessor(gltf, anim.samplers[ch.sampler].output)
        for ch in anim.channels
    }


def _root(gltf):
    return gltf.nodes[gltf.scenes[gltf.scene].nodes[0]]


def test_nodes_meshes_and_channels(baked):
    """A root node with one animated child per body; every part has a mesh."""
    case, cfg, gltf = baked
    mech = _assembly(cfg, 0.0)

    scene = gltf.scenes[gltf.scene]
    assert len(scene.nodes) == 1
    root = _root(gltf)
    assert root.name == "walker"
    assert root.mesh is None
    assert len(gltf.nodes) == len(mech.bodies) + 1
    children = [gltf.nodes[i] for i in root.children]
    assert [n.name for n in children] == [b.name for b in mech.bodies]

    parts = {b.name: b for b in mech.bodies}
    for node in children:
        body = parts[node.name]
        assert (node.mesh is not None) == (body.part is not None), node.name
        # pygltflib drops None-valued keys on save: absent means None
        assert node.extras.get("fab") == body.fab
        assert node.extras.get("rigid_with") == body.rigid_with
        assert node.extras.get("bom") == body.bom_key
        assert node.extras["body"] == body.name

    anim = gltf.animations[0]
    targets = {(ch.target.node, ch.target.path) for ch in anim.channels}
    assert len(anim.channels) == 2 * len(children)       # translation + rotation
    assert targets == {(i, p) for i in root.children for p in ("translation", "rotation")}

    assert len(scene.extras["foot_path"]) == 64
    assert scene.extras["robot"] is (case == "robot")
    if case == "robot":
        names = {n.name for n in children}
        assert {"L.b1", "R.b1", "L.servo", "R.servo"} <= names
        assert any(n.startswith("centre_plate") for n in names)


def test_root_stands_the_walker_up(baked):
    """The root turns model +Y to +Z and puts the lowest point of the gait on z = 0."""
    _case, _kw, gltf = baked
    root = _root(gltf)
    r = _quat_matrix(root.rotation)
    np.testing.assert_allclose(r @ [0, 1, 0], [0, 0, 1], atol=1e-9)    # up
    np.testing.assert_allclose(r @ [0, 0, 1], [0, -1, 0], atol=1e-9)   # stack: horizontal
    np.testing.assert_allclose(r @ [1, 0, 0], [1, 0, 0], atol=1e-9)    # walking axis

    trs = _tracks(gltf)
    lowest, highest = math.inf, -math.inf
    for i in root.children:
        node = gltf.nodes[i]
        if node.mesh is None:
            continue
        v = _read_accessor(gltf, gltf.meshes[node.mesh].primitives[0].attributes.POSITION)
        for k in range(N_FRAMES):
            local = v @ _quat_matrix(trs[(i, "rotation")][k]).T + trs[(i, "translation")][k]
            world = local @ r.T + root.translation
            lowest = min(lowest, world[:, 2].min())
            highest = max(highest, world[:, 2].max())
    assert lowest == pytest.approx(0.0, abs=1e-3)
    assert highest > 100.0


def test_animation_reproduces_fabricated_geometry(baked):
    """Posing each node's mesh by its animation at frame k lands on the part
    fabricated directly at that crank angle: shared meshes (other legs, the
    mirrored right side), their Z offsets and hardware riding its host included."""
    _case, cfg, gltf = baked
    trs = _tracks(gltf)
    root = _root(gltf)
    for frame in (0, 5):
        mech = _assembly(cfg, 2.0 * np.pi * frame / N_FRAMES)
        parts = {b.name: b.part for b in mech.bodies if b.part is not None}
        for i in root.children:
            node = gltf.nodes[i]
            if node.mesh is None:
                continue
            v = _read_accessor(gltf, gltf.meshes[node.mesh].primitives[0].attributes.POSITION)
            posed = v @ _quat_matrix(trs[(i, "rotation")][frame]).T + trs[(i, "translation")][frame]
            bb = parts[node.name].bounding_box()
            np.testing.assert_allclose(
                np.concatenate([posed.min(0), posed.max(0)]),
                [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z],
                atol=0.05, err_msg=f"{node.name} frame {frame}",
            )


def test_mirrored_side_shares_only_congruent_meshes(robot_gltf):
    """Right-side link plates (z-mirrored extrusions) reuse the left side's
    meshes; the servo, which isn't symmetric about its mid-plane, doesn't."""
    mesh = {n.name: n.mesh for n in robot_gltf.nodes}
    for link in ("b1", "b2", "b3", "b4", "torso", "frame_outer"):
        assert mesh[f"L.{link}"] == mesh[f"R.{link}"], link
    assert mesh["L.servo"] != mesh["R.servo"]
    with_part = [m for m in mesh.values() if m is not None]
    assert len(robot_gltf.meshes) < len(with_part)


def test_congruence_check():
    """A Z-mirror is a Z shift only for a part symmetric about its mid-plane."""
    from build123d import Axis, Location, Plane

    from spiderpig.shapes import Cut, disc
    from spiderpig.shapes import plate as make_plate

    plate = make_plate([((0, 0), (40, 10), 6.0)], 0.0, 3.0, [Cut((0, 0), 4.0), Cut((40, 10), 4.0)])
    pin = disc((5, 5), 3.0, 0.0, 12.0).fuse(disc((5, 5), 4.5, 0.0, 2.0))   # head at the bottom
    for part, symmetric in ((plate, True), (pin, False)):
        mirrored = part.mirror(Plane.XY).moved(Location((0, 0, -20.0)))
        a, b = part_props(part), part_props(mirrored)
        g = _Planar(dz=float(b.com[2] - a.com[2]))
        assert _congruent(a, b, g) is symmetric
    # a turned and shifted plate is congruent under the matching motion only
    moved = plate.rotate(Axis.Z, 30).moved(Location((7.0, -3.0, 9.0)))
    a, b = part_props(plate), part_props(moved)
    assert _congruent(a, b, _Planar(math.radians(30), (7.0, -3.0), 9.0))
    assert not _congruent(a, b, _Planar(0.0, (7.0, -3.0), 9.0))


def test_materials_follow_fab(robot_gltf):
    """Laser plates are translucent acrylic; printed, servo and metal are opaque."""
    by_node = {}
    for node in robot_gltf.nodes:
        if node.mesh is not None:
            mat = robot_gltf.materials[robot_gltf.meshes[node.mesh].primitives[0].material]
            by_node[node.name] = mat
    assert by_node["L.b1"].name == "acrylic"
    assert by_node["L.torso"].name == "acrylic_frame"
    assert by_node["centre_plate0"].name == "acrylic_frame"
    assert by_node["L.pin_C_screw"].name == "metal"        # the default pin: a Chicago screw
    assert by_node["L.pin_C_shims_hi"].name == "metal"
    standoff = next(n for n in by_node if n.startswith("L.pillar_") and "_standoff" in n)
    assert by_node[standoff].name == "metal"              # the default pillar: a standoff
    plate = next(n for n in by_node if n.startswith("L.crank_plate"))
    assert by_node[plate].name.startswith("acrylic")      # the bolt crank's plates
    assert by_node["L.servo"].name == "servo"
    assert by_node["L.servo_horn"].name == "metal"
    for name, mat in by_node.items():
        rgba = _MATERIALS[mat.name].rgba
        assert mat.pbrMetallicRoughness.baseColorFactor == pytest.approx(list(rgba)), name
        assert mat.alphaMode == ("BLEND" if mat.name.startswith("acrylic") else "OPAQUE"), name


def test_quaternion_shortest_path(baked):
    _case, _kw, gltf = baked
    anim = gltf.animations[0]
    rotation_channels = [ch for ch in anim.channels if ch.target.path == "rotation"]
    assert rotation_channels, "expected at least one rotation channel"
    for ch in rotation_channels:
        q = _read_accessor(gltf, anim.samplers[ch.sampler].output)
        assert q.shape == (N_FRAMES, 4)
        dots = np.einsum("ij,ij->i", q[:-1], q[1:])
        assert np.all(dots >= -1e-6), (
            f"shortest-path violated on node {ch.target.node}: min dot {dots.min()}"
        )


def test_gltf_animation_duration(baked):
    _case, _kw, gltf = baked
    t_acc = gltf.accessors[gltf.animations[0].samplers[0].input]
    assert t_acc.count == N_FRAMES
    assert t_acc.min[0] == pytest.approx(0.0)
    assert t_acc.max[0] == pytest.approx(DURATION * (N_FRAMES - 1) / N_FRAMES)


def test_bad_designs_are_rejected_before_baking():
    """What is baked is a :class:`config.BuildConfig`, which validates itself."""
    with pytest.raises(ParamError, match="unknown module"):
        BuildConfig(module="octo")
    with pytest.raises(ParamError, match="unknown linkage"):
        BuildConfig(linkage="octopus")
    with pytest.raises(ParamError, match="is a mechanism, not a walker"):
        BuildConfig(linkage="crank_rocker", module="single")


# ---------------------------------------------------------------------------
# The walking model's data and the design parameters
# ---------------------------------------------------------------------------


def _foot_z(gltf, node_idx: int) -> float:
    """Mid-plane z of a node's mesh at frame 0 (rotations are about z)."""
    node = gltf.nodes[node_idx]
    v = _read_accessor(gltf, gltf.meshes[node.mesh].primitives[0].attributes.POSITION)
    z = v[:, 2] + _tracks(gltf)[(node_idx, "translation")][0][2]
    return float(z.min() + z.max()) / 2


def test_drive_extras(baked):
    """The robot's root node carries the walking model's data (the drive contract);
    one side on its own can't stand, so it has none."""
    case, _kw, gltf = baked
    root = _root(gltf)
    if case == "side":
        assert "drive" not in root.extras
        return
    drive = root.extras["drive"]
    assert drive["theta_samples"] == walk.N_THETA == 360
    assert drive["clip_duration_s"] == pytest.approx(DURATION)
    feet = drive["feet"]
    assert [(f["body"], f["side"], f["leg"]) for f in feet] == [("L.b4", "L", 0), ("R.b4", "R", 0)]
    index = {n.name: i for i, n in enumerate(gltf.nodes)}
    path = KLANN.solve().evaluate(walk.theta_grid())["F"]
    for f in feet:
        np.testing.assert_allclose(f["xy"], path, atol=1e-3)
        assert f["z"] == pytest.approx(_foot_z(gltf, index[f["body"]]), abs=0.01)
    assert feet[0]["z"] < 0
    assert feet[1]["z"] == pytest.approx(-feet[0]["z"])
    com = np.array(drive["com"])
    assert com.shape == (3,)
    assert np.isfinite(com).all()
    assert abs(com[2]) < 2.0                        # the sides are mirror images
    assert drive["mass_g"] > 150.0
    assert drive["servo"] == {"key": "sts3215", "rpm_max": 52.0}
    assert drive["params"] == {"linkage": "klann", "module": "single", "phases_deg": [0.0],
                               "proportions": {k: float(v) for k, v in KLANN.params.items()}}
    assert drive["z_nominal"] is False
    assert drive["com_nominal"] is False
    assert drive["metrics"]["degenerate_fraction"] == 1.0        # two feet


def test_phases_are_baked(bakes):
    """A crank phase moves the leg in the animation and in the drive data alike."""
    gltf = bakes("phased")
    drive = _root(gltf).extras["drive"]
    assert drive["params"]["phases_deg"] == [90.0]
    base = KLANN.solve().evaluate(walk.theta_grid())["F"]
    np.testing.assert_allclose(drive["feet"][0]["xy"], np.roll(base, -90, axis=0), atol=1e-3)
    trs = _tracks(gltf)
    frame = 3
    mech = _assembly(PHASED, 2.0 * np.pi * frame / N_FRAMES)
    parts = {b.name: b.part for b in mech.bodies}
    for i, node in enumerate(gltf.nodes):
        if node.name not in ("L.b4", "R.b4", "L.b1"):
            continue
        v = _read_accessor(gltf, gltf.meshes[node.mesh].primitives[0].attributes.POSITION)
        posed = v @ _quat_matrix(trs[(i, "rotation")][frame]).T + trs[(i, "translation")][frame]
        bb = parts[node.name].bounding_box()
        np.testing.assert_allclose(
            np.concatenate([posed.min(0), posed.max(0)]),
            [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z], atol=0.05)


def test_design_parameters_are_normalized():
    """One config (and one file name) per design, however it was asked for."""
    assert BuildConfig() == BuildConfig(linkage="strider", module="double")   # the default
    assert BuildConfig().is_default
    assert BuildConfig().key == "strider_double_robot"
    assert BuildConfig(linkage="klann") == BuildConfig(linkage="klann", module="quad")
    default = BuildConfig(linkage="klann")
    quarter = (0.0, math.pi, math.pi / 2, 3 * math.pi / 2)
    assert BuildConfig(linkage="klann", module="quad", phases=quarter) == default
    assert BuildConfig(linkage="klann", proportions=(("DF", 2.577),)) == default
    assert default.is_default
    assert default.key == "klann_quad_robot"
    other = BuildConfig(linkage="klann", phases=(0.0, math.pi, math.pi / 2, 1.0),
                        proportions=(("DF", 2.4),))
    assert not other.is_default
    assert other.proportions == (("DF", 2.4),)
    assert other.key == BuildConfig(linkage="klann", phases=[0.0, math.pi, math.pi / 2, 1.0],
                                    proportions={"DF": 2.4}.items()).key
    assert other.key.startswith("klann_quad_robot_")
    assert other.key != default.key
    assert BuildConfig(linkage="klann", module="single").is_default   # its module's default
    assert BuildConfig(linkage="klann", module="single") != default
    with pytest.raises(ValueError, match="4 legs"):
        BuildConfig(linkage="klann", phases=(0.0,))
    with pytest.raises(ValueError, match="unknown klann proportions"):
        BuildConfig(linkage="klann", proportions=(("XX", 1.0),))
    jansen = BuildConfig(linkage="jansen", module="double", proportions=(("m", 14.0),))
    assert (jansen.linkage, jansen.proportions) == ("jansen", (("m", 14.0),))
    assert not jansen.is_default
    assert BuildConfig(linkage="jansen", module="double").is_default
    assert BuildConfig(linkage="klann") == default
    assert jansen.key != BuildConfig(linkage="strider", module="double").key
    with pytest.raises(ValueError, match="unknown linkage"):
        BuildConfig(linkage="octopus")
    with pytest.raises(ValueError, match="unknown strider proportions"):
        replace(jansen, linkage="strider")              # a replaced config validates too


def test_other_linkage_bake(bakes):
    """A Strider robot (one leg per side, a coupled pair with two feet): both feet of
    each side drive the walking model, the foot path is its first foot's, and the
    profile counts a leg per side."""
    gltf = bakes("strider")
    scene = gltf.scenes[gltf.scene]
    assert scene.extras["linkage"] == "strider"
    lk = linkage.get("strider")
    path = lk.solve().evaluate(np.linspace(0.0, 2 * math.pi, 64, endpoint=False))["J4"]
    np.testing.assert_allclose(scene.extras["foot_path"], path, atol=1e-9)
    drive = _root(gltf).extras["drive"]
    assert [f["body"] for f in drive["feet"]] == ["L.b3", "L.b7", "R.b3", "R.b7"]
    assert drive["params"]["linkage"] == "strider"
    index = {n.name: i for i, n in enumerate(gltf.nodes)}
    pts = lk.solve().evaluate(walk.theta_grid())
    for f, joint in zip(drive["feet"], ("J4", "J8", "J4", "J8"), strict=True):
        np.testing.assert_allclose(f["xy"], pts[joint], atol=1e-3)
        assert f["z"] == pytest.approx(_foot_z(gltf, index[f["body"]]), abs=0.01)
    assert "    n_legs: 2\n" in bakes.logs["strider"]              # both sides: 4 feet, 2 legs


def test_mechanism_bake(bakes):
    """A mechanism bakes one side: no feet, no drive; its output's path instead."""
    gltf = bakes("mechanism")
    scene = gltf.scenes[gltf.scene]
    lk = linkage.get("crank_rocker")
    pin = lk.solve().evaluate(np.linspace(0.0, 2 * math.pi, 64, endpoint=False))["E"]
    np.testing.assert_allclose(scene.extras["output_path"], pin, atol=1e-9)
    assert scene.extras["output"]["name"] == "b2"
    assert "foot_path" not in scene.extras
    assert "drive" not in (_root(gltf).extras or {})
    with pytest.raises(ParamError, match="crank_rocker is a mechanism, not a walker"):
        BuildConfig(linkage="crank_rocker", module="single", robot=True)
