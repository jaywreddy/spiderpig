"""Each design's own pin loads, from MuJoCo: walking and jammed at the servo's torque limit.

What the audit's strength check (:func:`construction.wobble.stresses`) needs per joint:
the in-plane force each link puts on its pin, as vectors, so the bending case is the
measured one (which link pushes against which), and how big it gets:

* **walking**: :func:`sim.run.simulate` of the robot (``WALK_SECONDS``, both drives full
  forward through the phase lock, from ``WALK_SKIP`` on), the force on every link at every
  joint each step. A joint's walking load is the ``WALK_PERCENTILE`` percentile over the
  steps of its most loaded link's force (a single-step contact spike is not a load the
  pin lives with), ``peak`` the largest;
* **jammed**: the base welded where it stands, one foot of the left side caught (a foot
  under something), the left drive commanded full speed one way or the other with its
  torque limited to the servo's firmware limit
  (:func:`config.torque_limit_nm`: :data:`servos.spec.TORQUE_LIMIT_FRACTION` of stall,
  45 %, or the linkage's lower limit), held until it stalls, at ``JAM_ANGLES`` crank
  angles per foot, both
  directions, the foot *pinned* to the world (``JAM_MODES``; ``"path"`` instead blocks
  it along its path only, see there), the floor's contacts off (``JAM_FLOOR``: the
  caught foot is the only hold besides the weld, no other foot shares the stalled
  torque through the floor). A joint's jam load is the largest of them (and at
  least its walking peak). The weld and the pin are as stiff as the loop equalities, so
  the foot stays within a fraction of a mm and every case stalls; the result records the
  stalled fraction and each case's foot drift, and warns when a case did not stall. A
  pinned leg is one constraint over its freedom, so how its load splits depends on where
  the give is (``docs/audit/STRENGTH.md`` has how much).

The forces: a tree hinge's from ``cfrc_int`` (``mj_rnePostConstraint``: what the parent
puts on the child), a loop's ``connect`` from its constraint rows (body1 takes the row
force, body2 the opposite), both turned into the base's frame (mech ``x``, ``y``: the
mechanism's plane). Joints are grouped per side by their point and name stem, so a
pillar two legs share (Strider's ``J2``) is one joint with both links on it, and both
sides' samples count for the walking load (the right side is the left mirrored in
``z``: the same in-plane forces).

Each joint keeps up to ``PATTERNS`` of its heaviest samples per case as patterns (each
link's force over the largest), which :func:`construction.wobble.stresses` scales by the
case's load. The crank's: the drives' walking torque (the same percentile) and the jam
torque, the limit.

:func:`design_loads` caches the result per design in the store (``pin_loads/<key>.json``,
valid for the package's sources: :func:`spiderpig.design.source_version`), so an audit
simulates a design once.
"""

from __future__ import annotations

import json
import math
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from spiderpig.config import BuildConfig, torque_limit_nm

VERSION = 3                # 3: the floor's contacts off during the jam (see JAM_FLOOR)
WALK_SECONDS = 3.0
WALK_SKIP = 0.5
WALK_PERCENTILE = 99.0
JAM_ANGLES = 24           # the Strider double's worst jam: 12 angles under-read a joint by
                           # up to 16 %, 24 within 1 % of 48
JAM_STEPS = 200            # per jam case (1 ms steps): the drive stalls within ~50
JAM_AVERAGE = 50           # the last steps of a case averaged
PATTERNS = 6
# The caught foot's holds: "pinned" to the world; "path", blocked along its path only (a
# light slider free across it), is statically determinate but rigid-body-exact near the
# foot's dead points, where the foot moves ~1 mm per crank radian and the torque over that
# ratio reads 6x the pinned load on the Klann single (a real joint's 0.2 mm clearance lets
# the crank pass those). Off by default; docs/audit/STRENGTH.md has the comparison.
JAM_MODES: tuple[str, ...] = ("pinned",)
JAM_PATH_DT = 1e-3         # rad of crank: the foot's path direction from the template
# The jam's weld and foot pin: as stiff as the model's own loop equalities
# (:attr:`sim.mjcf.SimParams.eq_solref` / ``eq_solimp``), so the base and the pinned foot
# stay put under a stalled drive. MuJoCo's default (0.02 s, 0.9-0.95) let the foot creep
# 10-40 mm and a third of the Strider double's cases never stalled; and a soft weld alone
# lets the base give ~4 mm, which puts the leg's compliance in the base mount (a pinned
# leg is statically indeterminate, so where the give is sets how the load splits).
# The floor during the jam: off. With the base welded and one foot pinned, the floor can
# only add load paths the jam is not about: a foot of the same side resting on it (the
# base is welded where it stood, the feet just clear or touch the floor) takes part of the
# stalled drive's torque as a second, unmodelled hold, so the caught leg's joints read
# low and the result depends on how deep the other feet happen to sit at that crank
# angle. Off, the caught foot is the only hold besides the weld, which is the case the
# strength check names (one foot caught under something); the stall bookkeeping (the
# stalled fraction, each case's foot drift) is unchanged. (Before VERSION 3 the floor
# stayed on.)
JAM_FLOOR = False
STALL_FRACTION = 0.95      # a case stalled: its drive at this fraction of the limit
STALL_WARN = 1.0           # below this fraction of stalled cases: a warning in the note


def _stem(name: str) -> str:
    """``L.J3_leg0#1`` -> ``J3``."""
    name = re.sub(r"^[LR]\.", "", name)
    name = re.sub(r"#\d+$", "", name)
    return re.sub(r"_leg\d+$", "", name)


class JointIndex:
    """Every pin of the model: which bodies meet there and where their force comes from."""

    def __init__(self, model, meta: dict) -> None:
        import mujoco

        self.base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        bodies = meta["bodies"]
        kind = {n: b["kind"] for n, b in bodies.items()}
        self.kind = kind
        groups: dict[tuple, dict] = {}

        def group(side: str, stem: str, xy) -> dict:
            key = (side, stem, round(float(xy[0]), 1), round(float(xy[1]), 1))
            return groups.setdefault(key, {"side": side, "stem": stem,
                                           "xy_mm": [round(float(xy[0]), 3),
                                                     round(float(xy[1]), 3)],
                                           "bodies": set(), "hinges": [], "loops": []})

        for name, b in bodies.items():
            if b["parent"] is None or b["kind"] == "crank":
                continue          # the base; a crank turns on O (its torque is the drive's)
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            xy = np.asarray(b["ref_pos"][:2]) * 1e3
            g = group(b["side"], b["pivot"], xy)
            g["bodies"] |= {name, b["parent"]}
            g["hinges"].append((bid, name, b["parent"]))
        eq_ids = np.flatnonzero(model.eq_type == mujoco.mjtEq.mjEQ_CONNECT)
        for e in eq_ids:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, int(e))
            if name is None or name.startswith("jam."):
                continue
            b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.eq_obj1id[e]))
            b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.eq_obj2id[e]))
            xy = (np.asarray(bodies[b1]["ref_pos"][:2]) + model.eq_data[e, 0:2]) * 1e3
            side = name.split(".", 1)[0]
            g = group(side, _stem(name), xy)
            g["bodies"] |= {b1, b2}
            g["loops"].append((int(e), b1, b2))
        self.joints = []
        for g in groups.values():
            links = sorted(b for b in g["bodies"] if kind.get(b) not in ("base", "crank"))
            if not links:
                continue
            g["links"] = links
            g["frame"] = any(kind.get(b) == "base" for b in g["bodies"])
            g["crank"] = any(kind.get(b) == "crank" for b in g["bodies"])
            self.joints.append(g)
        self.joints.sort(key=lambda g: (g["side"], g["stem"], g["links"]))
        # one row per (joint, link): the force that link takes from its pin
        self.rows: list[tuple[int, str]] = [(j, m) for j, g in enumerate(self.joints)
                                            for m in g["links"]]
        row = {jm: i for i, jm in enumerate(self.rows)}
        hin, lo = [], []
        for j, g in enumerate(self.joints):
            for bid, child, parent in g["hinges"]:
                hin.append((bid, row.get((j, child), -1), row.get((j, parent), -1)))
            for e, b1, b2 in g["loops"]:
                lo.append((e, row.get((j, b1), -1), row.get((j, b2), -1)))
        self.hinges, self.loops = hin, lo

    def forces(self, model, data) -> np.ndarray:
        """(rows, 2): the in-plane force (N, base frame) on each link at each of its pins."""
        import mujoco

        mujoco.mj_rnePostConstraint(model, data)
        rot = data.xmat[self.base].reshape(3, 3)
        out = np.zeros((len(self.rows), 3))
        for bid, rc, rp in self.hinges:
            f = data.cfrc_int[bid, 3:6]              # on the child, from its parent
            if rc >= 0:
                out[rc] += f
            if rp >= 0:
                out[rp] -= f
        if self.loops:
            n = data.nefc
            eq_rows = np.flatnonzero(data.efc_type[:n] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
            ids = data.efc_id[eq_rows]
            for e, r1, r2 in self.loops:
                rows = eq_rows[ids == e]
                if rows.size < 3:
                    continue
                f = data.efc_force[rows[:3]]         # on body1; body2 the opposite
                if r1 >= 0:
                    out[r1] += f
                if r2 >= 0:
                    out[r2] -= f
        return (out @ rot)[:, :2]                    # world -> base (mech) frame


def _summary(index: JointIndex, samples: np.ndarray, percentile: float | None) -> list[dict]:
    """Per joint of ``index``: the load (the percentile, else the largest, of its most
    loaded link's force over ``samples`` (S, rows, 2)), the peak and the heaviest patterns."""
    out = []
    mag = np.hypot(samples[..., 0], samples[..., 1]) if samples.size else samples
    for j in range(len(index.joints)):
        rows = [i for i, (jj, _) in enumerate(index.rows) if jj == j]
        if not samples.size:
            out.append({"n": 0.0, "peak": 0.0, "patterns": []})
            continue
        per = mag[:, rows].max(axis=1)
        peak = float(per.max())
        load = float(np.percentile(per, percentile)) if percentile is not None else peak
        pats = []
        for s in np.argsort(per)[::-1][:PATTERNS]:
            if per[s] <= 1e-9:
                break
            f = samples[s][rows] / per[s]
            pats.append({_side_free(index.rows[i][1]): [round(float(v), 4) for v in f[k]]
                         for k, i in enumerate(rows)})
        out.append({"n": round(load, 3), "peak": round(peak, 3), "patterns": pats})
    return out


def _side_free(body: str) -> str:
    return re.sub(r"^[LR]\.", "", body)


def walk_loads(config: BuildConfig, seconds: float = WALK_SECONDS, skip: float = WALK_SKIP,
               percentile: float = WALK_PERCENTILE) -> dict:
    """The walking case (see the module doc): per joint, and the drives' torque."""
    from spiderpig.sim.mjcf import load_model
    from spiderpig.sim.run import simulate, walk_metrics

    model, meta = load_model(config)
    index = JointIndex(model, meta)
    rows: list[np.ndarray] = []
    times: list[float] = []

    def observe(m, d) -> None:
        times.append(float(d.time))
        rows.append(index.forces(m, d))

    result = simulate(config, seconds=seconds, record_every=2, observe=observe)
    t = np.asarray(times)
    keep = t >= t[0] + skip if t.size else t.astype(bool)
    samples = np.asarray(rows)[keep] if rows else np.zeros((0, len(index.rows), 2))
    # both sides as samples of the same joints (the right is the left mirrored in z)
    merged = _merge_sides(index, samples)
    tq = np.abs(result.torque[result.t >= result.t[0] + skip])
    m = walk_metrics(result, skip=skip)
    return {"index": merged[0], "joints": _summary(merged[0], merged[1], percentile),
            "torque_nm": round(float(np.percentile(tq, percentile)), 4) if tq.size else 0.0,
            "torque_peak_nm": round(float(tq.max()), 4) if tq.size else 0.0,
            "walks": bool(m["walks"]), "fell": bool(m["fell"]),
            "speed_mm_s": round(float(m["speed"]), 2),
            "loop_force_peak_n": round(float(m["loop_force_peak"]), 3)}


class _SideFree:
    """A :class:`JointIndex` view with one entry per joint across both sides."""

    def __init__(self, joints, rows) -> None:
        self.joints, self.rows = joints, rows


def _merge_sides(index: JointIndex, samples: np.ndarray):
    """Both sides' joints as one: the left's joints, each right joint's samples appended
    to its twin's (same stem, the same links side-free)."""
    left = [g for g in index.joints if g["side"] == "L"] or index.joints
    key = {j: (g["stem"], tuple(_side_free(b) for b in g["links"]))
           for j, g in enumerate(index.joints)}
    joints = [dict(g, links=[_side_free(b) for b in g["links"]]) for g in left]
    lkeys = [(g["stem"], tuple(g["links"])) for g in joints]
    rows = [(j, m) for j, g in enumerate(joints) for m in g["links"]]
    pos = {(lkeys[j], m): i for i, (j, m) in enumerate(rows)}
    parts = []
    for side in sorted({g["side"] for g in index.joints}):
        part = np.zeros((samples.shape[0], len(rows), 2))
        hit = False
        for i, (j, m) in enumerate(index.rows):
            g = index.joints[j]
            if g["side"] != side:
                continue
            k = pos.get((key[j], _side_free(m)))
            if k is not None:
                part[:, k] = samples[:, i]
                hit = True
        if hit:
            parts.append(part)
    merged = np.concatenate(parts) if parts else np.zeros((0, len(rows), 2))
    return _SideFree(joints, rows), merged


def jam_loads(config: BuildConfig, angles: int = JAM_ANGLES, steps: int = JAM_STEPS,
              average: int = JAM_AVERAGE, modes: tuple[str, ...] = JAM_MODES) -> dict:
    """The jammed case (see the module doc): per joint of the left side."""
    import mujoco

    from spiderpig.sim.mjcf import SimParams, _v, build_mjcf, load_model
    from spiderpig.sim.run import kinematic_qpos

    stiff = {"solref": _v(SimParams.eq_solref), "solimp": _v(SimParams.eq_solimp)}
    xml, meta = build_mjcf(config)
    root = ET.fromstring(xml)
    eq = root.find("equality")
    if eq is None:
        eq = ET.SubElement(root, "equality")
    ET.SubElement(eq, "weld", name="jam.base", body1="base", **stiff)
    feet = [f for f in meta["feet"] if f.startswith("L.")]
    base_model, _ = load_model(config)
    world = root.find("worldbody")
    for f in feet:
        sid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_SITE, f)
        pos = base_model.site_pos[sid]
        anchor = " ".join(f"{v:.9g}" for v in pos)
        ET.SubElement(eq, "connect", name=f"jam.{f}", body1=meta["feet"][f]["body"],
                      anchor=anchor, active="false", **stiff)
        if "path" not in modes:
            continue
        # blocked along its path only: pinned to a 1 g slider free across the path
        sl = ET.SubElement(world, "body", name=f"jam.{f}.slider")
        ET.SubElement(sl, "joint", name=f"jam.{f}.slider", type="slide", axis="1 0 0")
        ET.SubElement(sl, "inertial", pos="0 0 0", mass="0.001",
                      diaginertia="1e-9 1e-9 1e-9")
        ET.SubElement(eq, "connect", name=f"jam.{f}.path", body1=meta["feet"][f]["body"],
                      body2=f"jam.{f}.slider", anchor=anchor, active="false", **stiff)
    if not JAM_FLOOR:
        # isolate the caught foot: the floor takes no part in the jam (JAM_FLOOR)
        for geom in world.iter("geom"):
            if geom.get("name") == "floor":
                geom.set("conaffinity", "0")
                geom.set("contype", "0")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    index = JointIndex(model, meta)
    left = [j for j, g in enumerate(index.joints) if g["side"] == "L"]
    lrows = [i for i, (j, _) in enumerate(index.rows) if j in set(left)]
    limit = torque_limit_nm(config)
    vmax = meta["actuators"]["L.drive"]["ctrlrange"][1]
    act = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{s}.drive")
           for s in ("L", "R")}
    model.actuator_forcerange[act["L"]] = (-limit, limit)
    def _id(kind, name):
        return mujoco.mj_name2id(model, kind, name)

    eqs = {(f, mode): _id(mujoco.mjtObj.mjOBJ_EQUALITY, f"jam.{f}" + suffix)
           for f in feet for mode, suffix in (("pinned", ""), ("path", ".path"))
           if mode in modes}
    sliders = {f: (_id(mujoco.mjtObj.mjOBJ_BODY, f"jam.{f}.slider"),
                   _id(mujoco.mjtObj.mjOBJ_JOINT, f"jam.{f}.slider"))
               for f in feet if "path" in modes}
    sites = {f: _id(mujoco.mjtObj.mjOBJ_SITE, f) for f in feet}
    base_id = _id(mujoco.mjtObj.mjOBJ_BODY, "base")
    nq = base_model.nq                              # the sliders' qpos come after the robot's
    cases, samples, stalled, drift = [], [], [], []
    t_ref = meta["t_ref"]

    def pose(qpos):
        mujoco.mj_resetData(model, data)
        data.qpos[:nq] = qpos
        for e in eqs.values():                      # only this case's foot is held
            data.eq_active[e] = 0
        mujoco.mj_forward(model, data)

    for k in range(angles):
        t = t_ref + 2 * math.pi * k / angles
        qpos = kinematic_qpos(config, t)
        ahead = kinematic_qpos(config, t + JAM_PATH_DT) if "path" in modes else qpos
        for f in feet:
            pose(ahead)
            p1 = data.site_xpos[sites[f]].copy()
            for mode in modes:
                for direction in (1.0, -1.0):
                    pose(qpos)
                    anchor = data.site_xpos[sites[f]].copy()
                    path = p1 - anchor
                    path /= max(float(np.linalg.norm(path)), 1e-12)
                    e = eqs[f, mode]
                    if mode == "pinned":
                        model.eq_data[e, 3:6] = anchor
                    else:
                        normal = data.xmat[base_id].reshape(3, 3)[:, 2]   # mech z
                        across = np.cross(normal, path)
                        body, joint = sliders[f]
                        model.body_pos[body] = anchor
                        model.jnt_axis[joint] = across / np.linalg.norm(across)
                        model.eq_data[e, 3:6] = 0.0
                        mujoco.mj_forward(model, data)
                    data.eq_active[e] = 1
                    acc = np.zeros((len(index.rows), 2))
                    torque = 0.0
                    for s in range(steps):
                        data.ctrl[act["L"]] = direction * vmax
                        data.ctrl[act["R"]] = 0.0
                        mujoco.mj_step(model, data)
                        if s >= steps - average:
                            acc += index.forces(model, data)
                            torque += abs(float(data.actuator_force[act["L"]]))
                    acc /= average
                    torque /= average
                    moved = data.site_xpos[sites[f]] - anchor
                    if mode == "path":
                        moved = float(moved @ path) * path
                    dev = float(np.linalg.norm(moved)) * 1000.0
                    stalled.append(torque >= STALL_FRACTION * limit)
                    drift.append(dev)
                    cases.append({"t": round(t, 4), "foot": f, "hold": mode,
                                  "direction": int(direction), "torque_nm": round(torque, 4),
                                  "foot_drift_mm": round(dev, 3),
                                  "stalled": bool(stalled[-1])})
                    samples.append(acc)
    arr = np.asarray(samples)
    view = _SideFree([dict(index.joints[j], links=[_side_free(b) for b in
                                                     index.joints[j]["links"]]) for j in left],
                     [(left.index(j), _side_free(m)) for j, m in
                      (index.rows[i] for i in lrows)])
    return {"index": view, "joints": _summary(view, arr[:, lrows] if arr.size else arr, None),
            "torque_nm": limit, "cases": len(cases), "floor": JAM_FLOOR,
            "stalled": round(float(np.mean(stalled)), 3) if stalled else 0.0,
            "foot_drift_mm": round(max(drift, default=0.0), 3),
            "unstalled": [c for c in cases if not c["stalled"]]}


def simulate_loads(config: BuildConfig) -> dict:
    """Both cases for ``config`` (the robot), as the cache stores them."""
    from dataclasses import replace

    from spiderpig.sim.run import crank_joint_factor

    config = replace(config, robot=True)
    t0 = time.time()
    walk = walk_loads(config)
    jam = jam_loads(config)
    joints = []
    jam_by = {(g["stem"], tuple(g["links"])): s
              for g, s in zip(jam["index"].joints, jam["joints"], strict=True)}
    for g, w in zip(walk["index"].joints, walk["joints"], strict=True):
        j = jam_by.get((g["stem"], tuple(g["links"])), {"n": 0.0, "peak": 0.0, "patterns": []})
        joints.append({"stem": g["stem"], "links": list(g["links"]), "xy_mm": g["xy_mm"],
                       "frame": g["frame"], "crank": g["crank"],
                       "walk": w, "jam": dict(j, n=round(max(j["n"], w["peak"]), 3))})
    return {
        "version": VERSION, "source": "sim", "config": config.design_json(),
        "walk_percentile": WALK_PERCENTILE, "walk_seconds": WALK_SECONDS,
        "jam_cases": jam["cases"], "jam_stalled": jam["stalled"], "jam_floor": jam["floor"],
        "jam_foot_drift_mm": jam["foot_drift_mm"], "jam_unstalled": jam["unstalled"],
        "torque_limit_nm": jam["torque_nm"],
        "walk_torque_nm": walk["torque_nm"], "walk_torque_peak_nm": walk["torque_peak_nm"],
        "joint_moment_factor": crank_joint_factor(config),
        "walks": walk["walks"], "fell": walk["fell"], "speed_mm_s": walk["speed_mm_s"],
        "loop_force_peak_n": walk["loop_force_peak_n"],
        "joints": joints, "seconds": round(time.time() - t0, 1),
    }


def cache_path(config: BuildConfig, store=None) -> Path:
    from dataclasses import replace

    from spiderpig.store import Store

    store = Store.of(store) if store is not None else Store.default()
    return store.root / "pin_loads" / f"{replace(config, robot=True).key}.json"


def design_loads(config: BuildConfig, store=None, *, refresh: bool = False,
                 cached_only: bool = False) -> dict | None:
    """:func:`simulate_loads` for ``config``, cached in ``store`` (the project's by default)
    while the package's sources are unchanged; ``cached_only``: None rather than
    simulating. Raises ``ImportError`` without MuJoCo."""
    import mujoco  # noqa: F401  (the caller falls back without it)

    from spiderpig.design import source_version

    path = cache_path(config, store)
    if not refresh and path.exists():
        try:
            doc = json.loads(path.read_text())
            if doc.get("version") == VERSION and doc.get("source_version") == source_version():
                return doc
        except (OSError, ValueError):
            pass
    if cached_only:
        return None
    doc = simulate_loads(config)
    doc["source_version"] = source_version()
    if doc["jam_stalled"] < STALL_WARN:
        import warnings

        warnings.warn(f"{config.key}: only {doc['jam_stalled']:.0%} of the jam cases stalled "
                      f"(pinned foot drifted up to {doc['jam_foot_drift_mm']:g} mm): the jam "
                      f"loads are undersampled", RuntimeWarning, stacklevel=2)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(f".{time.time_ns()}.tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    tmp.replace(path)
    return doc
