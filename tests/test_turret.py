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
from spiderpig.construction.rotary import PRESETS, SUPPORTS
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
    assert (c.role("turntable"), c.role("tool")) == ("mt4_compact", "suction_cup")
    assert BuildConfig().applied_roles() == {}               # the walkers have neither


@pytest.mark.no_fabricate
def test_the_turntable_is_mt4_compact_its_pair_preloaded():
    assert list(PRESETS) == ["mt4_compact"] == rolemod.choices("turntable")
    pair = SUPPORTS["base_pair"]
    t = pair.error_terms(48.0)
    assert t.radial_play_mm == t.tilt_deg == 0.0
    assert pair.capacity_nm(48.0) > 1.0
    preset = PRESETS["mt4_compact"]
    assert preset.parts[1].ratio(None) == pytest.approx(60 / 16)
    assert (preset.support, preset.sensor) == ("base_pair", "hard_stop")


@pytest.mark.no_fabricate
def test_a_pruned_turntable_transmission_or_mill_fails_naming_its_replacement():
    """The turret pruned to mt4_compact (2026-10-14): a removed key names what replaced it,
    in a config, on the command line and in a spec."""
    import argparse

    from spiderpig.config import add_build_args, add_design_args
    from spiderpig.spec import validate

    for key in ("belt_preload", "mt4", "capstan_preload", "capstan_precise", "mt4_base",
                "mt4_base_stall"):
        with pytest.raises(ParamError, match=rf"turntable {key!r}.*use turntable='mt4_compact'"):
            _robot(roles=(("turntable", key),))
    for q in ("transmission", "transmission@q2"):
        with pytest.raises(ParamError, match="use transmission='belt'"):
            _robot(roles=((q, "capstan"),))
    with pytest.raises(ParamError, match=r"role 'mill'.*removed on 2026-10-14"):
        _robot(roles=(("mill", "carvera"),))
    p = argparse.ArgumentParser(exit_on_error=False)
    add_design_args(p)
    add_build_args(p)
    with pytest.raises(argparse.ArgumentError, match="mt4_compact"):
        p.parse_args(["--linkage", "palletizer", "--turntable", "belt_preload"])
    errors = validate({"kind": "arm", "linkage": {"key": "palletizer"},
                       "constructions": {"turntable": "capstan_preload"}})
    assert any(e.path == "constructions.turntable" and "mt4_compact" in e.message
               for e in errors)


# -- the turret's plan (no parts) ----------------------------------------------------------


@pytest.mark.no_fabricate
@pytest.mark.parametrize("linkage", ["palletizer", "palletizer_reach"])
def test_the_turret_stands_under_the_side_and_holds_together(linkage):
    from spiderpig.construction.rotary.turret import _lows

    cfg = BuildConfig(linkage=linkage)
    design, tp, _tool = _plans(cfg)
    inside, outside, _ = _lows(design, (tp.x_a, tp.keep_r, tp.r_base))
    assert tp.y_t < outside
    assert tp.y_t < inside - 9
    h = tp.h                            # the bearings in the base's plates
    assert h["lb_bot"] < h["bp_top"] < h["ub_bot"] < h["bt_bot"] < h["bt_top"] < h["ub_top"]
    assert h["bt_top"] < h["drive_bot"] < h["drive_top"] < h["tt_bot"] < h["table"]
    assert h["tt_bot"] >= h["shaft_top"] + 1.0 - 1e-9     # over the motor's shaft
    assert tp.spacing > 40.0                              # the plates' column apart
    assert tp.r_table < tp.keep_r
    assert tp.y_ground == pytest.approx(tp.y_t - h["table"])
    assert h["shaft_top"] - h["bt_bot"] <= tp.preset.parts[2].shaft_len + 1e-9
    motor = tp.preset.parts[2]          # the base past the motor: its lobe past the far face
    assert tp.r_base > tp.c + motor.frame / 2
    assert len(tp.brackets) == 4
    assert {pl for pl, _ in tp.brackets} == {"outer", "inner"}
    lo, hi = tp.range_deg
    assert lo < 0 < hi
    assert tp.error_terms().tilt_deg == 0.0


@pytest.mark.slow
@pytest.mark.no_fabricate
def test_the_base_keeps_the_arm_out_of_it_and_the_tool_reaches_its_plate():
    """Round 7's turret (``base_pair``): the bearings in the base, only the links over its
    keep-out above the turntable, so the tool gets down to the bench over a band 110 mm
    deep or more (the links' lengths bound it, not the turret: round 7)."""
    from spiderpig.construction.rotary.base_pair import racking

    _, tp, _ = _plans(BuildConfig(linkage="palletizer_reach"))
    assert tp.h["reach_hi"] - tp.h["reach_lo"] >= 110.0
    assert tp.h["reach_lo"] > tp.keep_r
    assert any("gets down to the bench" in n for n in tp.notes)
    per_nm, note = racking(tp)
    assert 0.0 < per_nm < 1e-3
    assert "columns" in note


@pytest.mark.no_fabricate
def test_the_tool_finds_room_beside_the_end_plate():
    from spiderpig.construction.tools import TOOLS, ToolPlan

    for key in ("suction_cup", "suction_wrist"):
        _, _tp, tool = _plans(_robot(roles=(("tool", key),)))
        assert isinstance(tool, ToolPlan)
        assert tool.body == "b8"
        assert tool.tool is TOOLS[key]
        assert len(tool.regions) == (2 if TOOLS[key].wrist else 4)   # the plates: 4 boxes
    _, _, none = _plans(_robot(roles=(("tool", rolemod.NONE),)))
    assert none is None


@pytest.mark.no_fabricate
def test_the_turntable_joins_the_precision_budget():
    from spiderpig.precision import precision

    _, side = cache.cached_design(_robot())

    def budget(cfg):
        return precision(cfg, side, n=9)

    pre = budget(_robot())
    assert set(pre.inputs) == {"q1", "q2", "q3"}
    assert pre.tilt_mm == 0.0                                 # the pair preloaded
    assert pre.inputs["q1"].backlash_mm > 0.0                 # the belt's teeth
    assert any("rack" in n for n in pre.notes)                # the base's columns
    assert pre.lever_mm_per_rad["q1"] > 200.0                 # the flange's reach
    assert pre.inputs["q1"].homing_mm > 0.0
    assert pre.turret_accuracy_mm > 0.0
    assert pre.repeatability_mm == pytest.approx(
        pre.plays.worst_mm + sum(b.backlash_mm + b.homing_mm for b in pre.inputs.values()))
    alone = budget(_robot(robot=False))
    assert "q1" not in alone.inputs
    assert alone.tilt_mm == 0.0


# -- the robot built -----------------------------------------------------------------------


@pytest.mark.slow
@pytest.mark.parametrize("roles", [(), (("tool", "suction_wrist"), ("angle_sensor", "stall"))],
                         ids=["cup", "wrist-stall"])
def test_the_turret_robot_is_whole_and_its_parts_meet_only_where_they_thread(roles):
    from spiderpig.construction.contract import clashes
    from spiderpig.fabricate import split_parts
    from spiderpig.hardware.bom import bom_from_mechanism

    cfg = _robot(roles=roles)
    mech = cache.cached_robot(cfg, 1.0)
    assert split_parts(mech) == []
    for t in (1.0, 2.6, 4.4):          # over the tour: the tool and the links past the turret
        assert clashes(cache.cached_robot(cfg, t)) == [], t   # a screw in its thread is not one
    for a, b in mech.meta["fastened"]:  # and each pair said to thread does meet
        assert {a, b} <= {x.name for x in mech.bodies}
    tree = RobotTree.from_meta(mech.meta["tree"])
    assert tree is not None
    turning = {n for n, j in tree.carried.items() if j == "q1"}
    assert {"turntable", "torso", "b1", "foot_bracket0"} <= turning
    assert "base_top" not in tree.carried
    assert "yaw_motor" not in tree.carried
    rows = {r.key: r.qty for r in bom_from_mechanism(mech).purchased}
    from spiderpig.construction.rotary.base_pair import BRACKET_KEY, SHAFT_KEY

    assert rows["bearing_f608zz"] == 2                      # the base's bearings
    assert "optical_endstop" not in rows                    # no switch: a hard stop
    assert {"yaw_stop", "yaw_stop_post"} <= {b.name for b in mech.bodies}
    assert rows["collar_8mm"] == 3
    assert rows[SHAFT_KEY] == 1
    assert rows["m4_hex_standoff_45"] == 7          # three round the bearings
    assert rows[BRACKET_KEY] == 4
    assert not any(b.fab == "printed" for b in mech.bodies
                   if b.name.startswith(("base_", "yaw_", "turntable", "foot_")))
    assert rows["suction_cup_m5_10"] == 1
    from spiderpig.actuators.controllers import get as controller  # the electronics

    for key, qty in controller(cfg.role("controller")).bom:
        assert rows[key] == qty, key
    steppers = sum(q for k, q in rows.items()
                   if k.startswith("stepper_") and not k.startswith("stepper_cable"))
    assert steppers == (4 if "suction_wrist" in dict(roles).values() else 3)


@pytest.mark.slow
@pytest.mark.parametrize("tool", ["suction_cup", "suction_wrist"])
def test_the_bake_turns_the_side_on_its_turntable(tool):
    """The baked end plate's flange is the linkage's, yawed by the sweep about the axis; the
    wrist's cup (turning about its own axis on the end plate) keeps its place on the flange."""
    import pygltflib

    from spiderpig.bake import T_REF, bake_gltf
    from spiderpig.fabricate import template_for

    cfg = _robot(roles=(("tool", tool),))
    mech = cache.cached_robot(cfg, T_REF)
    out = cache.cache_dir() / f"turret_bake_test_{tool}.glb"
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
    body, p0 = "b8", f0
    if tool == "suction_wrist":           # (the bake's branch for a hosted joint)
        body = "tool_cup"
        bb = by[body].placed_part().bounding_box()
        p0 = np.array([bb.center().X, bb.center().Y, bb.center().Z])
    trans, rot = track(body, "translation"), track(body, "rotation")
    o = np.asarray(tree.joint("q1").origin)
    for k in range(len(ts)):
        x, y, z, w = rot[k]
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        got = r @ p0 + trans[k]
        want = rotation((0, 1, 0), [yaw[k]])[0] @ (f_t[k] + (p0 - f0) - o) + o
        np.testing.assert_allclose(got, want, atol=1e-2)
    i = names.index("base_top")
    assert not any(c.target.node == i and c.target.path == "rotation" and
                   np.ptp(track("base_top", "rotation"), axis=0).max() > 1e-6
                   for c in g.animations[0].channels)


def _xml_fk(root, values: dict[str, float]) -> dict[str, np.ndarray]:
    """Every link's pose in ``base_link`` (m) from the written URDF alone: each joint's
    origin (xyz, rpy) then its rotation about its axis by its value (a mimic's from its
    leader's)."""
    from spiderpig.kinematics import rotation as rot

    def rpy(r, p_, y):
        return rot((0, 0, 1), [y])[0] @ rot((0, 1, 0), [p_])[0] @ rot((1, 0, 0), [r])[0]

    joints = list(root.iter("joint"))
    val = {}
    for j in joints:
        mim = j.find("mimic")
        name = j.get("name")
        if mim is not None:
            val[name] = float(mim.get("multiplier")) * values.get(mim.get("joint"), 0.0) + \
                float(mim.get("offset"))
        else:
            val[name] = values.get(name, 0.0)
    pose = {"base_link": np.eye(4)}
    pending = joints[:]
    while pending:
        for j in list(pending):
            parent, child = j.find("parent").get("link"), j.find("child").get("link")
            if parent not in pose:
                continue
            o = j.find("origin")
            m = np.eye(4)
            m[:3, :3] = rpy(*(float(v) for v in o.get("rpy").split()))
            m[:3, 3] = [float(v) for v in o.get("xyz").split()]
            if j.get("type") != "fixed":
                ax = [float(v) for v in j.find("axis").get("xyz").split()]
                r = np.eye(4)
                r[:3, :3] = rot(ax, [val[j.get("name")]])[0]
                m = m @ r
            pose[child] = pose[parent] @ m
            pending.remove(j)
    return pose


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
    assert float(joints["q2"].find("limit").get("effort")) > 1.0     # 0.4 N·m x 5
    m = urdf.model(mech, cfg, 1.0)
    assert m.closures                           # the parallelograms' loops, left to the map
    lk = cfg.lk
    by = {b.name: b for b in mech.bodies}
    f0 = np.append((by["b8"].pose @ by["b8"].joint("F").pose).matrix[:3, 3], 1.0)
    q2, q3 = (np.ravel(g) for g in lk.box(5))
    sol = linkage.LegSolution(1, 0.0, cfg.values, lk.key).at(q2, q3)["F"].reshape(-1, 2)
    o = np.asarray(m.origins["torso"])
    side = __import__("json").loads(files[1].read_text())
    to_ros = rotation((1, 0, 0), [math.pi / 2])[0]
    for i in range(len(q2)):
        for yaw, roll in ((0.0, 0.0), (0.7, 1.2)):
            q = {"q2": float(q2[i]), "q3": float(q3[i])}
            pose = urdf.forward(m, q, {"q1": yaw, "q4": roll})
            got = (pose["b8"] @ f0)[:3]
            want = rotation((0, 1, 0), [yaw])[0] @ (np.append(sol[i], f0[2]) - o) + o
            np.testing.assert_allclose(got, want, atol=1e-6)
            # the written XML on its own, its passive joints from the sidecar
            vals = {n: q[n] - m.zero[n] for n in m.zero} | {"q1": yaw, "q4": roll}
            for name, j in side["joints"].items():
                if name not in vals and j["mimic"] is None:
                    vals[name] = sum(c * vals[k] for k, c in j["coefficients"].items())
            xml = _xml_fk(root, vals)
            tip = (pose["tool0"] @ np.append(m.origins["tool0"], 1.0))[:3]
            np.testing.assert_allclose(xml["tool0"][:3, 3] * 1000.0,
                                       to_ros @ (tip - m.floor), atol=1e-4)
            # the wrist turns the cup about its own axis: the tip stays, the frame turns
            np.testing.assert_allclose(xml["tool0"][:3, :3] @ [0, 1, 0],
                                       to_ros @ [0, 1, 0], atol=1e-9)
    turned = _xml_fk(root, {"q4": 1.0})["tool0"][:3, :3]
    assert not np.allclose(turned, _xml_fk(root, {})["tool0"][:3, :3])


@pytest.mark.no_fabricate
def test_the_pose_endpoint_names_its_joints_in_degrees():
    from fastapi.testclient import TestClient

    from spiderpig.server import app as srv

    client = TestClient(srv.app, base_url="http://localhost")
    r = client.get("/api/pose/robot?linkage=palletizer&j.q2=seventy")
    assert r.status_code == 422
    assert "degrees" in r.json()["detail"]
    assert client.get("/api/pose/nope?linkage=palletizer").status_code == 404
    info = next(x for x in client.get("/api/linkages").json()["linkages"]
                if x["key"] == "palletizer")
    assert info["inputs"] == ["q2", "q3"]
    assert info["ranges"]["q2"] == [40.0, 115.0]


@pytest.mark.slow
def test_posing_the_baked_arm_lands_on_the_bakes_own_frames():
    """At a frame's inputs and joint angles, each node posed (``spiderpig.pose``) is that
    frame's: the viewer's sliders show what the bake animates."""
    import pygltflib

    from spiderpig.bake import T_REF, bake_gltf
    from spiderpig.pose import node_poses

    cfg = _robot(roles=(("tool", "suction_wrist"),))
    out = cache.cache_dir() / "turret_pose_test.glb"
    bake_gltf(out, cfg, n_frames=24, profile=False, fabricated=cache.cached_robot(cfg, T_REF))
    g = pygltflib.GLTF2().load(str(out))
    blob = g.binary_blob()
    names = [n.name for n in g.nodes]

    def matrix(i: int, k: int) -> np.ndarray:
        vals = {}
        for path, w in (("translation", 3), ("rotation", 4)):
            ch = next(c for c in g.animations[0].channels if c.target.node == i
                      and c.target.path == path)
            acc = g.accessors[g.animations[0].samplers[ch.sampler].output]
            bv = g.bufferViews[acc.bufferView]
            vals[path] = np.frombuffer(blob, np.float32, acc.count * w,
                                       bv.byteOffset).reshape(-1, w)[k]
        x, y, z, w_ = vals["rotation"]
        m = np.eye(4)
        m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w_), 2 * (x * z + y * w_)],
                     [2 * (x * y + z * w_), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w_)],
                     [2 * (x * z - y * w_), 2 * (y * z + x * w_), 1 - 2 * (x * x + y * y)]]
        m[:3, 3] = vals["translation"]
        return m

    tree = RobotTree.from_meta(g.scenes[0].extras["tree"])
    assert tree is not None
    lk = cfg.lk
    for k in (5, 13):
        t = T_REF + 2 * math.pi * k / 24
        q = {n: math.degrees(float(v[0])) for n, v in zip(lk.inputs, lk.tour(np.array([t])),
                                                          strict=True)}
        q.update({n: math.degrees(float(v[0]))
                  for n, v in tree.sweep(np.array([t - T_REF])).items()})
        poses = node_poses(cfg, out, q)
        for name in ("b8", "b4", "turntable", "tool_cup", "base_top", "drive_q2_motor"):
            i = names.index(name)
            pose = np.array(poses[name]).reshape(4, 4).T
            np.testing.assert_allclose(pose @ matrix(i, 0), matrix(i, k), atol=1e-4)


@pytest.mark.slow
def test_the_arms_mujoco_model_closes_its_loops(tmp_path):
    """MuJoCo's model of the same tree (``urdf.mjcf``): the motors' position actuators set,
    the loops held by their pins, the tip settles where the kinematics put it."""
    import mujoco

    from spiderpig import urdf
    from spiderpig.control.arm import ArmModel

    cfg = _robot()
    mech = cache.cached_robot(cfg, 1.0)
    files = urdf.write(mech, cfg, 1.0, tmp_path, "arm")
    model = mujoco.MjModel.from_xml_path(str(files[2]))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0.0
    m = urdf.model(mech, cfg, 1.0)
    _, design = cache.cached_design(cfg)
    arm = ArmModel.from_config(cfg, design)
    q = {"q2": m.zero["q2"] + 0.2, "q3": m.zero["q3"] - 0.15, "q1": 0.5}
    for n in ("q1", "q2", "q3"):
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        data.ctrl[a] = q[n] - m.zero.get(n, 0.0)
    for _ in range(20000):
        mujoco.mj_step(model, data)
    tip = arm.to_base(data.site("tool0").xpos * 1000.0)
    np.testing.assert_allclose(tip, arm.fk(q), atol=0.01)


@pytest.mark.no_fabricate
def test_a_stored_arm_loads_as_the_robot_it_was_resolved_to(tmp_path):
    from spiderpig.api import derive, load, resolve

    d = resolve({"kind": "arm", "linkage": {"key": "palletizer"}}, tmp_path)
    back = load(d.id, tmp_path)
    assert back.config == d.config
    assert back.config.layout == "turret"
    side = resolve({"kind": "arm", "linkage": {"key": "palletizer"}, "legs": {"robot": False}},
                   tmp_path)
    assert load(side.id, tmp_path).config.robot is False
    assert derive(side, {"materials": {"thickness_mm": 3.1}}).config.robot is False


@pytest.mark.slow
def test_the_wrists_mujoco_model_stands_under_gravity(tmp_path):
    import mujoco

    from spiderpig import urdf

    cfg = _robot(roles=(("tool", "suction_wrist"),))
    files = urdf.write(cache.cached_robot(cfg, 1.0), cfg, 1.0, tmp_path, "arm")
    model = mujoco.MjModel.from_xml_path(str(files[2]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    start = data.site("tool0").xpos.copy()
    for i in range(6000):                       # 3 s held by its motors, never reset
        mujoco.mj_step(model, data)
        assert np.isfinite(data.qacc).all(), i
    assert data.time == pytest.approx(6000 * model.opt.timestep)
    assert np.linalg.norm(data.site("tool0").xpos - start) < 0.01    # it sags under 10 mm
