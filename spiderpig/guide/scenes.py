"""A guide step's picture in a pose of its own (``construction.assembly.Scene``): a decanting
cradle's use, its trip apart and back, and its first build's passive side lowered onto it
lying down (design review round 2's M4, 2026-10-14: the use steps had no picture).

From the fabricated cradle the guide draws (its crank at ``t``), each body's rigid motion
to the scene's crank angle (its anchor's joints fitted in the plane, as the bake animates
it: :func:`spiderpig.walk.planar_fit`), then the whole laid on a side (the riser's blocks
moved under the drive side's outer plate: ``cradle_pack.lay_blocks``), then each piece's
move. :func:`rows` gives the renderer's rows, each with its 4 x 4 (``guide.render.items_of``).
"""

from __future__ import annotations

import math
import re

import numpy as np


def _anchor_motion(mech, config, t_to: float) -> dict[str, np.ndarray]:
    """Each body's planar motion (4 x 4, world) from ``mech``'s pose to crank angle ``t_to``:
    its anchor's joints (``walk.anchor_of``) where they are against where the robot template
    puts them then."""
    from spiderpig import walk
    from spiderpig.construction.robot import robot_template
    from spiderpig.fabricate import template_for

    tmpl = robot_template(template_for(config))
    sampled = tmpl.sample(np.array([float(t_to)]))
    by = {b.name: b for b in mech.bodies}
    fits: dict[str, np.ndarray] = {}
    out: dict[str, np.ndarray] = {}
    for b in mech.bodies:
        anchor = walk.anchor_of(b, by)
        m = np.eye(4)
        if anchor is not None and anchor in sampled.joint_world:
            if anchor not in fits:
                body = by[anchor]
                now = sampled.joint_world[anchor]
                names = [j.name for j in body.joints if j.name in now]
                p0 = np.array([(body.pose @ j.pose).matrix[:3, 3] for j in body.joints
                               if j.name in now], float)
                p1 = np.array([now[n][0] for n in names], float)
                p0[:, 2] = 0.0
                p1[:, 2] = 0.0
                th, tr = walk.planar_fit(p0[None], p1[None])
                c, s = math.cos(float(th[0])), math.sin(float(th[0]))
                f = np.eye(4)
                f[:2, :2] = [[c, -s], [s, c]]
                f[:2, 3] = tr[0][:2]
                fits[anchor] = f
            m = fits[anchor]
        out[b.name] = m
    return out


def crank_t(mech, lk) -> float:
    """The crank angle the cradle ``mech`` was fabricated at: where the linkage puts its
    crankpin (``lk.crank``'s second joint) where the drive side's crank holds it."""
    pin = lk.crank[1]
    at = next((np.asarray((b.pose @ j.pose).matrix[:2, 3], float)
               for b in mech.bodies if b.name.startswith("L.") for j in b.joints
               if j.name == pin), None)
    if at is None:
        return 0.0
    ts = np.linspace(0.0, 2.0 * math.pi, 7200, endpoint=False)
    pts = lk.solve().evaluate(ts)[pin]
    return float(ts[int(np.argmin(np.hypot(*(pts - at).T)))])


def _rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    m = np.eye(4)
    m[1:3, 1:3] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return m


def _block_matrices(mech, lk, drive_side: str) -> dict[str, np.ndarray]:
    """The riser's blocks (``decanter_riser_block*``), each on its lift
    (``decanter_riser_lift*``), moved from the riser to stand under the drive side's outer
    plate (``construction.cradle.LAY_BLOCKS``), as ``cradle_pack.lay_blocks`` does their
    points: 4 x 4 each, before the lay."""
    from spiderpig.construction import cradle as cr

    by = {b.name: b for b in mech.bodies if b.part is not None}
    plate = by.get(f"{drive_side}.frame_outer")
    nodes = cr.LAY_BLOCKS.get(lk.key, ())
    if plate is None or not nodes:
        return {}
    bb = plate.placed_part().bounding_box()
    out_z = 1.0 if (bb.min.Z + bb.max.Z) / 2 > 0 else -1.0
    face = bb.max.Z if out_z > 0 else bb.min.Z
    lip = np.asarray(mech.meta["cradle_lip"], float)
    pts = lk.solve().evaluate(np.array([crank_t(mech, lk)]))
    stand = cr.stand_of(lk)
    out = {}
    names = sorted(n for n in by if n.startswith("decanter_riser_block"))
    for node, name in zip(nodes, names, strict=False):
        if not isinstance(node, str):
            at = lip + np.asarray(node, float)
        elif node in stand.nodes:
            at = lip + np.asarray(stand.nodes[node], float)
        else:
            at = pts[node][0]
        b = by[name].placed_part().bounding_box()
        c = np.array([(b.min.X + b.max.X) / 2, b.max.Y, (b.min.Z + b.max.Z) / 2])
        # (x, y, z) -> (x, z, -y * out_z) about its boss's top's middle, then onto the node
        rot = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -out_z, 0.0]])
        m = np.eye(4)
        m[:3, :3] = rot
        m[:3, 3] = np.array([at[0], at[1], face]) - rot @ c
        out[name] = m
        lift = name.replace("_block", "_lift")    # its lift goes with it (under it)
        if lift in by:
            out[lift] = m
    return out


def pieces_of(mech, lk) -> dict[str, list[str]]:
    """Each body of the cradle ``mech`` by its piece (``cradle_pack``'s module pieces, or its
    flat pack's), plus ``bottle``, ``decanter``, ``risers`` and ``blocks`` (the riser's two)."""
    from spiderpig import cradle_pack as cp
    from spiderpig.construction import cradle as cr

    infos = cp.part_infos(mech)
    fastened = [tuple(p) for p in mech.meta.get("fastened", ())]
    link = cr.cradle_link(lk)
    if lk.key in cr.PACK_POSES:
        pcs, _, _ = cp.module_pieces(infos, mech.connections, link, lk, fastened)
    else:
        pcs, _ = cp.pieces(infos, mech.connections, link, fastened)
    out = {pc.key: list(pc.parts) for pc in pcs}
    names = {b.name for b in mech.bodies if b.part is not None}
    out["bottle"] = [n for n in ("bottle",) if n in names]
    out["decanter"] = [n for n in ("decanter",) if n in names]
    out["blocks"] = sorted(n for n in names
                           if n.startswith(("decanter_riser_block", "decanter_riser_lift")))
    out.setdefault("risers", sorted(n for n in names if n.startswith("decanter_riser")))
    out["board"] = out.get("base", [])
    return out


def rows(scene, mech, config, placed: list[str], adds: list[str], fill_of,
         context_fill, placed_fill) -> list:
    """The renderer's rows for a step whose picture is ``scene``: ``placed`` the bodies on so
    far (this step's ``adds`` among them, highlighted), drawn as the scene says (its pieces,
    else all of ``placed``), each with its 4 x 4; ``scene.marks`` highlighted too.
    ``fill_of(name)`` a highlighted body's fill."""
    from spiderpig.construction import cradle as cr
    from spiderpig.construction.assembly import body_of

    lk = config.lk
    t0 = crank_t(mech, lk)
    pose = scene.pose
    t = t0
    if pose == "pack":
        got = cr.pack_pose(lk)
        t = got[0] if got is not None else t0
    elif pose in ("rest", "full", "level"):
        n = 2880
        ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        pts = lk.solve().evaluate(ts)
        o, ax = cr.mouth_frame(lk)
        d = pts[ax] - pts[o]
        tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        k0 = int(np.argmin(tilt))
        if pose == "rest":
            t = float(ts[k0])
        elif pose == "full":
            t = float(ts[int(np.argmax(tilt))])
        else:
            order = np.roll(np.arange(n), -k0)
            t = float(ts[next(m for m in order if tilt[m] >= 0.0)])
    motion = _anchor_motion(mech, config, t) if abs(t - t0) > 1e-9 else {}
    pieces = pieces_of(mech, lk)
    drive = "L"                         # (the drive side: the side as planned)
    if scene.show:
        want: set[str] = set()
        for key in scene.show:
            if key == "*":
                want |= {n for k, v in pieces.items() if k not in ("bottle", "decanter",
                                                                   "risers", "blocks")
                         for n in v}
            else:
                want |= set(pieces.get(key, ()))
        shown = [p for p in placed if body_of(p) in want]
    else:
        shown = list(placed)
    piece_of = {n: k for k, v in pieces.items() if k not in ("board", "blocks") for n in v}
    for n in pieces.get("blocks", ()):
        piece_of[n] = "blocks"
    blocks = _block_matrices(mech, lk, drive) if scene.lay else {}
    lay = np.eye(4)
    if scene.lay == "drive":
        lay = _rot_x(-90.0 if drive == "L" else 90.0)
    elif scene.lay == "passive":
        lay = _rot_x(90.0 if drive == "L" else -90.0)
    moves = {k: np.array([dx, dy, dz]) for k, dx, dy, dz in scene.moves}
    marks = [re.compile(m) for m in scene.marks]
    adds_set = set(adds)
    out = []
    for p in shown:
        name = body_of(p)
        m = blocks.get(name) if scene.lay == "drive" else None
        m = m if m is not None else motion.get(name, np.eye(4))
        m = lay @ m
        shift = moves.get(piece_of.get(name, ""))
        if shift is not None:
            m = m.copy()
            m[:3, 3] += shift
        hit = p in adds_set or any(r.fullmatch(name) for r in marks)
        fill = fill_of(name) if hit else (placed_fill if p in adds_set else context_fill)
        out.append([p, fill, hit, np.round(m, 6).tolist()])
    return out


def box_rows(scene, mech, config, kit: dict, fill_of, context_fill, spacer_fill
             ) -> tuple[list, dict[str, tuple[np.ndarray, np.ndarray]]]:
    """A box step's picture (``guide.make.with_box_steps``, packing review 2's M2: the pack,
    not the assembled cradle): the pack payload ``kit``'s pieces ``scene.show`` names (its
    modules' and spacers' keys) where it puts them, each body moved to the pack pose
    (:func:`_anchor_motion`) and then by its piece's matrix (or its own), the spacers and the
    carton's floor as meshes of their own (``extra``: name -> positions, triangles, in the
    box's frame); ``scene.marks`` (keys) highlighted: a piece's bodies by ``fill_of``, a
    spacer in ``spacer_fill``. The renderer's rows and the extra meshes."""
    from spiderpig import voxpack as vp
    from spiderpig.construction import cradle as cr

    lk = config.lk
    got = cr.pack_pose(lk)
    t = got[0] if got is not None else crank_t(mech, lk)
    motion = _anchor_motion(mech, config, t) if abs(t - crank_t(mech, lk)) > 1e-9 else {}
    show, marks = set(scene.show), set(scene.marks)
    rows: list = []
    extra: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    names = {b.name for b in mech.bodies if b.part is not None}
    lo = np.asarray(kit["box"]["floor"], float)
    hi = lo + np.asarray(kit["box"]["inside_mm"], float)
    floor = vp.box_mesh((lo[0], lo[1] - 3.0, lo[2]), (hi[0], lo[1], hi[2]))
    extra["carton_floor"] = floor
    rows.append(["carton_floor", context_fill, False, np.eye(4).tolist()])
    for mod in kit["modules"]:
        if mod["key"] not in show:
            continue
        hit = mod["key"] in marks
        for n in mod["parts"]:
            if n not in names:
                continue
            m = np.asarray(mod.get("part_matrix", {}).get(n, mod["matrix"]), float)
            m = m @ motion.get(n, np.eye(4))
            rows.append([n, fill_of(n) if hit else context_fill, hit, np.round(m, 6).tolist()])
    for d in kit.get("spacers") or []:
        if d["key"] not in show:
            continue
        hit = d["key"] in marks
        extra[d["key"]] = vp.Spacer.from_json(d).mesh()
        rows.append([d["key"], spacer_fill if hit else context_fill, hit, np.eye(4).tolist()])
    return rows, extra
