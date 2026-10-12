"""An arm's baked model posed at joint angles (the viewer's joint sliders, 2026-10-12).

The bake (:mod:`spiderpig.bake`) animates an arm over its inputs' tour; to pose it anywhere
in its box, the viewer asks for each node's transform relative to the clip's first frame
(:func:`node_poses`): the planar motion of the node's anchor body from the reference pose
to the inputs given (the linkage's points at those inputs, :meth:`linkage.LegSolution.at`,
fitted per body as the bake fits them, :func:`walk.planar_fit`), composed with the spatial
joints at the angles given (:class:`kinematics.RobotTree`), over the same composition at
the first frame. The node's matrix there times it is the node posed: no part is
fabricated, the glb's own node extras (``rigid_with``) and scene metadata (its ``tree``)
say who rides whom.
"""

from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path

import numpy as np

from spiderpig import linkage
from spiderpig.config import BuildConfig
from spiderpig.kinematics import RobotTree, planar_matrix


@lru_cache(maxsize=16)
def _reference(config: BuildConfig) -> tuple[dict, tuple[float, ...]]:
    """The template's joints at the clip's first frame and the inputs there (per config,
    built once: the template checks its loops as it is built)."""
    from spiderpig.bake import T_REF
    from spiderpig.construction.robot import layout_of
    from spiderpig.fabricate import template_for

    tmpl = template_for(config)
    if config.robot:
        tmpl = layout_of(config).template(tmpl)
    ts = np.array([T_REF])
    ref = {b.name: ({j.name: tmpl.sample(ts).joint_world[b.name][j.name][0] for j in b.joints})
           for b in tmpl.bodies if b.joints}
    return ref, tuple(float(v[0]) for v in config.lk.tour(ts))


@lru_cache(maxsize=16)
def _glb(path: str, mtime: float) -> tuple[dict, dict | None]:
    """A glb's nodes' extras (``name -> extras``) and its scene's ``tree`` (per file and
    modification time: a rebake reads it again)."""
    import json
    import struct

    with open(path, "rb") as fh:            # the JSON chunk only: not the 6 MB of buffers
        fh.read(12)
        length, _ = struct.unpack("<II", fh.read(8))
        doc = json.loads(fh.read(length))
    nodes = {n["name"]: (n.get("extras") or {}) for n in doc.get("nodes", []) if n.get("name")}
    scene = doc["scenes"][doc.get("scene", 0)]
    return nodes, (scene.get("extras") or {}).get("tree")


def inputs0(config: BuildConfig) -> dict[str, float]:
    """The planar inputs at the clip's first frame (degrees: where the sliders start)."""
    _, q0 = _reference(config)
    return {n: math.degrees(v) for n, v in zip(config.lk.inputs, q0, strict=True)}


def node_poses(config: BuildConfig, glb: Path, q_deg: dict[str, float]) -> dict[str, list[float]]:
    """Each node's 4x4 (column-major, three.js's order) taking it from the clip's first
    frame to the joints ``q_deg`` (degrees: the planar inputs' design angles, the robot's
    ``q1`` / ``q4`` from their zeros; a joint not given stays where the first frame has it)."""
    from spiderpig import walk

    lk = config.lk
    if lk.kind != "arm":
        raise ValueError(f"{lk.key} is a {lk.kind}: joint poses are an arm's")
    unknown = sorted(set(q_deg) - set(lk.inputs) - {"q1", "q4"})
    if unknown:
        raise ValueError(f"unknown joints {unknown}; {lk.key}'s are {list(lk.inputs)}, and a "
                         "robot's q1 and q4")
    glb = Path(glb)
    extras, tree_meta = _glb(str(glb), glb.stat().st_mtime)
    tree = RobotTree.from_meta(tree_meta)
    ref, q0 = _reference(config)
    qs = [math.radians(q_deg[n]) if n in q_deg else q0[i] for i, n in enumerate(lk.inputs)]
    pts = linkage.LegSolution(1, 0.0, config.values, lk.key).at(*(np.atleast_1d(v) for v in qs))
    planar, planar0 = {}, {}
    for body, joints_ref in ref.items():
        names = [n for n in joints_ref if n in pts]
        if not names:
            continue
        p0 = np.stack([joints_ref[n] for n in names])[None]
        p1 = np.stack([np.append(np.asarray(pts[n]).reshape(-1, 2)[0], joints_ref[n][2])
                       for n in names])[None]
        th, tr = walk.planar_fit(p0, p1)
        planar[body] = planar_matrix(th, tr)
        planar0[body] = np.eye(4)[None]
    joints = joints0 = {}
    if tree is not None:
        first = tree.sweep(np.zeros(1))
        angles = {j.name: (np.array([math.radians(q_deg[j.name])]) if j.name in q_deg
                           else first[j.name]) for j in tree.joints}
        joints = tree.joint_transforms(angles, planar)
        joints0 = tree.joint_transforms(first, planar0)
    out: dict[str, list[float]] = {}
    for name, ex in extras.items():
        if "body" not in ex:
            continue
        anchor = _anchor(name, extras, planar)
        if tree is not None:
            t1 = tree.body_transform(name, anchor, joints, planar, 1)[0]
            t0 = tree.body_transform(name, anchor, joints0, planar0, 1)[0]
        else:
            t1 = planar[anchor][0] if anchor else np.eye(4)
            t0 = np.eye(4)
        out[name] = (t1 @ np.linalg.inv(t0)).T.ravel().tolist()
    return out


def _anchor(name: str, extras: dict, planar: dict) -> str | None:
    """The kinematic body a node rides: itself, else its ``rigid_with`` chain's first."""
    seen = set()
    cur: str | None = name
    while cur is not None and cur not in seen:
        if cur in planar:
            return cur
        seen.add(cur)
        cur = (extras.get(cur) or {}).get("rigid_with")
    return None
