"""An arm's robot (2026-10-12, phase 4): the turret layout (:mod:`construction.rotary`),
its tool (:mod:`construction.tools`), the spatial joints over the side
(:mod:`spiderpig.kinematics`), the bake's general rotations and the URDF
(:mod:`spiderpig.urdf`)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spiderpig import roles as rolemod
from spiderpig.config import BuildConfig, ParamError
from spiderpig.construction.rotary import PRESETS, SPACING, SUPPORTS
from spiderpig.kinematics import (
    RobotTree,
    SpatialJoint,
    planar_matrix,
    quat_from_matrix,
    rigid_about,
    rotation,
)
from tests import cache

ARM = {"linkage": "palletizer"}


def _robot(**kw) -> BuildConfig:
    return BuildConfig(**ARM, **kw)


def _plans(cfg):
    from spiderpig.construction.robot import Turret

    _, design = cache.cached_design(cfg)
    return design, *Turret().plans(design, cfg)


# -- the kinematics ---------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_rotations_and_their_quaternions():
    r = rotation((0, 1, 0), [math.pi / 2])[0]
    np.testing.assert_allclose(r @ [1, 0, 0], [0, 0, -1], atol=1e-12)   # right-handed about y
    m = rigid_about((0, 0, 1), (10, 0, 0), [math.pi])[0]
    np.testing.assert_allclose(m @ [0, 0, 0, 1], [20, 0, 0, 1], atol=1e-12)
    rng = np.random.default_rng(3)
    axes = rng.normal(size=(20, 3))
    angles = rng.uniform(-3.1, 3.1, 20)
    for k, a in zip(axes, angles, strict=True):
        q = quat_from_matrix(rotation(k, [a]))[0]
        x, y, z, w = q
        back = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(back, rotation(k, [a])[0], atol=1e-9)
    p = planar_matrix(np.array([math.pi / 2]), np.array([[1.0, 2.0, 3.0]]))[0]
    np.testing.assert_allclose(p @ [1, 0, 0, 1], [1, 3, 3, 1], atol=1e-12)


@pytest.mark.no_fabricate
def test_a_tree_composes_its_joints_parents_first():
    yaw = SpatialJoint("q1", (0, 1, 0), (0, 0, 0), (-90, 90))
    roll = SpatialJoint("q4", (0, 1, 0), (100, 0, 0), (-180, 180), parent="q1", host="b8")
    tree = RobotTree((yaw, roll), {"b1": "q1", "cup": "q4"})
    assert RobotTree.from_meta(tree.to_meta()) == tree
    assert RobotTree.from_meta(None) is None
    assert [j.name for j in tree.chain("q4")] == ["q1", "q4"]
    ts = np.linspace(0, 2 * math.pi, 9)
    sweep = tree.sweep(ts)
    assert sweep["q1"].min() == pytest.approx(-math.pi / 2)
    assert sweep["q4"].max() == pytest.approx(math.pi)
    n = 1
    shift = planar_matrix(np.zeros(n), np.array([[0.0, 5.0, 0.0]]))    # b8 lifted 5 mm
    joints = tree.joint_transforms({"q1": np.array([math.pi / 2]), "q4": np.array([math.pi])},
                                   {"b8": shift})
    # the cup at the roll axis' side: turned about it, lifted with b8, turned with the yaw
    cup = tree.body_transform("cup", "b8", joints, {"b8": shift}, n)[0] @ [110, 0, 0, 1]
    np.testing.assert_allclose(cup, [0, 5, -90, 1], atol=1e-9)
    link = tree.body_transform("b1", "b1", joints, {"b1": shift}, n)[0] @ [100, 0, 0, 1]
    np.testing.assert_allclose(link, [0, 5, -100, 1], atol=1e-9)
    with pytest.raises(ValueError, match="must come first"):
        RobotTree((roll, yaw))
    with pytest.raises(ValueError, match="reference pose"):
        SpatialJoint("q1", (0, 1, 0), (0, 0, 0), (10, 90))


# -- the config, the roles, the presets ----------------------------------------------------


@pytest.mark.no_fabricate
def test_an_arm_makes_a_turret_robot():
    c = _robot()
    assert (c.robot, c.layout, c.sides, c.mirrored) == (True, "turret", 1, False)
    assert BuildConfig().layout == "mirrored_pair"
    assert BuildConfig().sides == 2
    assert _robot(robot=False).layout == ""
    with pytest.raises(ParamError, match="not a walker or an arm"):
        BuildConfig(linkage="hoecken", module="single", robot=True)
    assert rolemod.choices("turntable") == sorted(PRESETS)
    assert rolemod.choices("tool") == ["suction_cup", "suction_wrist", rolemod.NONE]
    assert (c.role("turntable"), c.role("tool")) == ("belt_preload", "suction_cup")
    assert BuildConfig().applied_roles() == {}               # the walkers have neither


@pytest.mark.no_fabricate
def test_a_preloaded_pair_has_no_play_and_a_plain_one_tilts_by_it():
    from spiderpig.construction.pivots.ball import internal_clearance
    from spiderpig.construction.rotary import SHAFT_FIT

    plain, pre = SUPPORTS["deep_groove_pair"], SUPPORTS["preloaded_pair"]
    c = internal_clearance("bearing_608zz") + SHAFT_FIT
    t = plain.error_terms(SPACING)
    assert t.radial_play_mm == pytest.approx(c)
    assert t.tilt_deg == pytest.approx(math.degrees(c / SPACING))
    assert pre.error_terms(SPACING).tilt_deg == 0.0
    assert pre.capacity_nm(SPACING) > 1.0 > plain.capacity_nm(SPACING) == 0.0
    belt, cap = PRESETS["belt_preload"], PRESETS["capstan_preload"]
    assert belt.parts[1].ratio(None) == 5.0
    assert cap.parts[1].ratio(None) == pytest.approx((40.0 + 0.5) / 3.5)


# -- the turret's plan (no parts) ----------------------------------------------------------


@pytest.mark.no_fabricate
@pytest.mark.parametrize("preset", sorted(PRESETS))
def test_the_turret_stands_under_the_side_and_holds_together(preset):
    from spiderpig.construction.rotary.turret import _lows

    cfg = _robot(roles=(("turntable", preset),))
    design, tp, tool = _plans(cfg)
    inside, outside, _ = _lows(design, tool)
    assert tp.y_t < outside
    assert tp.y_t < inside - 10
    h = tp.h
    assert h["bt_top"] < h["drive_bot"] < h["hub_bot"] < h["hub_top"] < h["table"]
    assert tp.y_ground == pytest.approx(tp.y_t - h["table"])
    assert h["shaft_top"] - h["bt_bot"] <= tp.preset.parts[2].shaft_len + 1e-9
    assert tp.r_base > tp.c + tp.preset.parts[2].frame / math.sqrt(2)
    assert len(tp.brackets) == 4
    assert {pl for pl, _ in tp.brackets} == {"outer", "inner"}
    lo, hi = tp.range_deg
    assert lo < 0 < hi
    terms = tp.error_terms()
    if tp.preset.support == "preloaded_pair":
        assert terms.tilt_deg == 0.0
    if tp.preset.transmission == "capstan":
        assert terms.backlash_deg == 0.0
        from spiderpig.construction.transmissions.capstan import CORD_PITCH, WRAPS

        core = h["n_core"] * design.ctx.pitch                # the wraps and their walk fit
        assert core >= (WRAPS + (hi - lo) / 360 * tp.preset.parts[1].ratio()) * CORD_PITCH


@pytest.mark.no_fabricate
def test_the_tool_finds_room_beside_the_end_plate():
    from spiderpig.construction.tools import TOOLS, ToolPlan

    for key in ("suction_cup", "suction_wrist"):
        _, _tp, tool = _plans(_robot(roles=(("tool", key),)))
        assert isinstance(tool, ToolPlan)
        assert tool.body == "b8"
        assert tool.tool is TOOLS[key]
        assert len(tool.regions) == (2 if TOOLS[key].wrist else 1)
    _, _, none = _plans(_robot(roles=(("tool", rolemod.NONE),)))
    assert none is None


@pytest.mark.no_fabricate
def test_the_turntable_joins_the_precision_budget():
    from spiderpig.precision import precision

    _, side = cache.cached_design(_robot())

    def budget(cfg):
        return precision(cfg, side, n=9)

    pre = budget(_robot())
    plain = budget(_robot(roles=(("turntable", "mt4"),)))
    cap = budget(_robot(roles=(("turntable", "capstan_preload"),)))
    assert set(pre.inputs) == {"q1", "q2", "q3"}
    assert pre.tilt_mm == 0.0 < plain.tilt_mm
    assert plain.repeatability_mm == pytest.approx(pre.repeatability_mm + plain.tilt_mm)
    assert cap.inputs["q1"].backlash_mm == 0.0 < pre.inputs["q1"].backlash_mm
    assert cap.inputs["q1"].resolution_mm < pre.inputs["q1"].resolution_mm
    assert pre.lever_mm_per_rad["q1"] > 200.0                 # the flange's reach
    alone = budget(_robot(robot=False))
    assert "q1" not in alone.inputs
    assert alone.tilt_mm == 0.0


# -- the robot built -----------------------------------------------------------------------


THREADED = (("_motor_screw", "_motor"), ("_clamp", "torso"), ("_clamp", "frame_outer"),
            ("_bolt", "_pulley_large"), ("base_col", "base_col_screw"),
            ("yaw_endstop_post", "yaw_endstop_screw"), ("wrist_screw", "wrist_motor"))
"""Pairs that meet by design: a screw in the tapped hole it threads into."""


def _clashes(mech) -> list[tuple[str, str, float]]:
    from tests._construction import overlapping_pairs

    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in overlapping_pairs(parts):
        if any((x in a and y in b) or (x in b and y in a) for x, y in THREADED):
            continue
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > 1e-3:
            out.append((a, b, round(vol, 3)))
    return out


@pytest.mark.slow
@pytest.mark.parametrize("roles", [(), (("turntable", "capstan_preload"),),
                                   (("turntable", "mt4"), ("tool", "suction_wrist"))],
                         ids=["belt", "capstan", "mt4-wrist"])
def test_the_turret_robot_is_whole_and_its_parts_meet_only_where_they_thread(roles):
    from spiderpig.fabricate import split_parts
    from spiderpig.hardware.bom import bom_from_mechanism

    cfg = _robot(roles=roles)
    mech = cache.cached_robot(cfg, 1.0)
    assert split_parts(mech) == []
    assert _clashes(mech) == []
    tree = RobotTree.from_meta(mech.meta["tree"])
    assert tree is not None
    turning = {n for n, j in tree.carried.items() if j == "q1"}
    assert {"turntable", "torso", "b1", "foot_bracket0"} <= turning
    assert "base_top" not in tree.carried
    assert "yaw_motor" not in tree.carried
    rows = {r.key: r.qty for r in bom_from_mechanism(mech).purchased}
    assert rows["bearing_608zz"] == 2
    assert rows["collar_8mm"] == 4
    assert rows["shaft_8mm_h6_100"] == 1
    assert rows["suction_cup_m5_10"] == 1
    assert rows["optical_endstop"] == 1
    steppers = sum(q for k, q in rows.items() if k.startswith("stepper_"))
    assert steppers == (4 if "suction_wrist" in dict(roles).values() else 3)


@pytest.mark.slow
def test_the_bake_turns_the_side_on_its_turntable():
    """The baked end plate's flange is the linkage's, yawed by the sweep about the axis."""
    import pygltflib

    from spiderpig.bake import T_REF, bake_gltf
    from spiderpig.fabricate import template_for

    cfg = _robot()
    mech = cache.cached_robot(cfg, T_REF)
    out = cache.cache_dir() / "turret_bake_test.glb"
    bake_gltf(out, cfg, n_frames=24, profile=False, fabricated=mech)
    g = pygltflib.GLTF2().load(str(out))
    blob = g.binary_blob()
    names = [n.name for n in g.nodes]

    def track(node: str, path: str) -> np.ndarray:
        i = names.index(node)
        ch = next(c for c in g.animations[0].channels if c.target.node == i
                  and c.target.path == path)
        acc = g.accessors[g.animations[0].samplers[ch.sampler].output]
        bv = g.bufferViews[acc.bufferView]
        width = 3 if path == "translation" else 4
        return np.frombuffer(blob, np.float32, acc.count * width,
                             bv.byteOffset).reshape(-1, width)

    tree = RobotTree.from_meta(mech.meta["tree"])
    assert tree is not None
    ts = T_REF + np.linspace(0, 2 * math.pi, 24, endpoint=False)
    yaw = tree.sweep(ts - T_REF)["q1"]
    by = {b.name: b for b in mech.bodies}
    f0 = (by["b8"].pose @ by["b8"].joint("F").pose).matrix[:3, 3]
    f_t = template_for(cfg).sample(ts).joint_world["b8"]["F"]
    trans, rot = track("b8", "translation"), track("b8", "rotation")
    o = np.asarray(tree.joint("q1").origin)
    for k in range(len(ts)):
        x, y, z, w = rot[k]
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        got = r @ f0 + trans[k]
        want = rotation((0, 1, 0), [yaw[k]])[0] @ (f_t[k] - o) + o
        np.testing.assert_allclose(got, want, atol=1e-2)
    i = names.index("base_top")
    assert not any(c.target.node == i and c.target.path == "rotation" and
                   np.ptp(track("base_top", "rotation"), axis=0).max() > 1e-6
                   for c in g.animations[0].channels)


@pytest.mark.slow
def test_the_urdf_moves_as_the_linkage_does(tmp_path):
    from spiderpig import linkage, urdf

    cfg = _robot(roles=(("tool", "suction_wrist"),))
    mech = cache.cached_robot(cfg, 1.0)
    files = urdf.write(mech, cfg, 1.0, tmp_path, "arm")
    root = ET.parse(files[0]).getroot()
    joints = {j.get("name"): j for j in root.iter("joint")}
    assert {"q1", "q2", "q3", "q4"} <= set(joints)
    assert joints["q2"].find("mimic") is None
    mimics = [j for j in joints.values() if j.find("mimic") is not None]
    assert mimics
    assert all(j.find("mimic").get("joint") in ("q2", "q3") for j in mimics)
    links = {e.get("name") for e in root.iter("link")}
    for j in joints.values():                   # a tree: every joint's links exist
        assert j.find("parent").get("link") in links
        assert j.find("child").get("link") in links
    assert all((tmp_path / "meshes" / f"{n}.stl").exists() for n in ("b1", "torso", "tool"))
    m = urdf.model(mech, cfg, 1.0)
    assert m.closures                           # the parallelograms' loops, left to the map
    lk = cfg.lk
    by = {b.name: b for b in mech.bodies}
    f0 = np.append((by["b8"].pose @ by["b8"].joint("F").pose).matrix[:3, 3], 1.0)
    q2, q3 = (np.ravel(g) for g in lk.box(5))
    sol = linkage.LegSolution(1, 0.0, cfg.values, lk.key).at(q2, q3)["F"].reshape(-1, 2)
    o = np.asarray(m.origins["torso"])
    for i in range(len(q2)):
        for yaw in (0.0, 0.7):
            pose = urdf.forward(m, {"q2": float(q2[i]), "q3": float(q3[i])}, {"q1": yaw})
            got = (pose["b8"] @ f0)[:3]
            want = rotation((0, 1, 0), [yaw])[0] @ (np.append(sol[i], f0[2]) - o) + o
            np.testing.assert_allclose(got, want, atol=1e-6)
