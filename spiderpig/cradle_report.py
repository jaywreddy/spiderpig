"""How a decanting cradle does its job: the swing, the loads, the hand, the stand, the fit.

A cradle (:mod:`construction.cradle`) is judged here, not by the walking model or MuJoCo:

* **The swing**: the tilt range and how still the bottle's mouth stays
  (:meth:`linkage.Linkage.output_check`), each loop's transmission angles.
* **The bottles** (:data:`construction.cradle.SIZES`): each one's setting (the two V
  blocks' rise, the base stop's place: :func:`construction.cradle.setting`) and where its
  mouth sits off the pivot there (:func:`construction.cradle.seat`).
* **The saddles** (:func:`saddles`): the loads on their printed parts and steel (the V
  blocks, the thumbscrews, the post, the lip gauge, the base stop) and the stresses in
  them, a sealed full bottle of each size at its own setting.
* **The loads**: a quasi-static planar solve (:func:`statics`) of the side's bodies, the
  cradle box's mass and a **sealed full 6 L methuselah** (the worst a cradle carries: nothing
  pours out of it) on the cradle, gravity alone, at every crank angle: the crank's torque,
  every pin's force. A **jam** case puts a firm push at the grip on a blocked cradle
  (:data:`JAM_HAND_N`, about twice what the worst turn takes, through the worm at a hand's
  speed: :func:`jam_nm`), away from the crank-rocker's toggles (where nothing can bound
  it). The pins are rated by :mod:`spiderpig.strength` at these loads ("walk" the bottle's,
  "jam" the jam's).
* **The hand**: the crank torque through the worm (its ratio, its static efficiency: the
  slow end of a hand's speed) at the handle's throw; turns per pour.
* **The rod**: Euler buckling of the connecting rod at its largest compression.
* **The stand**: the centre of mass of the cradle with each bottle, full and empty, at its
  own setting, over the swing, against the feet (the base ties' line along x, the two
  outer plates across).
* **Clashes**: every part against every other at angles across the swing
  (:func:`construction.contract.clashes`), the bottle and the decanter included.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

from spiderpig.config import BuildConfig, is_cradle
from spiderpig.construction import cradle as cr
from spiderpig.construction.handcrank import (
    CRANKPIN_D,
    HANDLE_R,
    NRV040,
    HandCrank,
    WormBox,
    handle_r,
)
from spiderpig.fabricate import design_side, fabricate, template_for
from spiderpig.hardware import mass as mass_mod
from spiderpig.hardware.cradle_catalog import NETRF10_YIELD, NETRF12_ID
from spiderpig.stack import body_class

G = 9.81
SHARE = 1.0             # a pin on both sides of the cradle carries this share of the planar
#                         load: all of it. The rod drives one side, whose pivots carry half
#                         the weight and the rod's whole force (the other side's, half the
#                         weight): the planar sum bounds both (the 2026-10-10 review)
TOGGLE_DEG = 30.0       # the jam case leaves out crank angles this close to a toggle: there
#                         the cradle moves under 1° per 10° of crank (it is at its end), and a
#                         block's force grows without bound
E_ALU = 69000.0         # MPa, 5052 / 6061
WARN_HANDLE_N = 40.0    # a handle force a hand turns steadily for a minute
JAM_HAND_N = 40.0       # the jam: a firm push at the grip against a blocked cradle (about twice
#                         the worst turn's: a hand that feels it stop and pushes on)


def jam_nm(box: WormBox = NRV040, grip_r: float = HANDLE_R) -> float:
    """The jam case's crank torque (N·m): :data:`JAM_HAND_N` at the grip (``grip_r`` mm)
    through the worm at a hand's speed (``WormBox.eta_hand``: ``config.PRODUCT_JAM_NM`` is
    this number)."""
    return JAM_HAND_N * grip_r / 1000.0 * box.ratio * box.eta_hand


@dataclass
class Rigid:
    """A moving body of the planar model: its joints (side's names) and its mass (kg) with
    the centre of mass written in its frame (``along``, ``across`` from ``a`` toward ``b``)."""

    name: str
    joints: tuple[str, ...]
    a: str
    b: str
    mass: float
    along: float
    across: float


def _frame_coords(p, a, b) -> tuple[float, float]:
    u = (b - a) / max(float(np.linalg.norm(b - a)), 1e-12)
    n = np.array([-u[1], u[0]])
    d = p - a
    return float(d @ u), float(d @ n)


def _from_frame(along, across, a, b):
    u = (b - a) / np.linalg.norm(b - a, axis=-1, keepdims=True)
    n = np.stack([-u[..., 1], u[..., 0]], axis=-1)
    return a + along * u + across * n


def _host(b, by) -> str:
    """The kinematic body a part rides (its ``rigid_with`` chain's end)."""
    seen = 0
    while b.rigid_with is not None and b.rigid_with in by and seen < 20:
        b = by[b.rigid_with]
        seen += 1
    return b.name


def part_mass_kg(config: BuildConfig, b) -> float:
    """A part's mass (kg): :mod:`hardware.mass`'s (a bought part at its catalogued mass)."""
    _, dens, fixed = mass_mod.material_of(b, config.sheet, "pla_filament", None)
    if fixed is not None:
        return fixed / 1000.0
    return mass_mod.part_props(b.placed_part()).volume * dens / 1e6


def masses(config: BuildConfig, mech) -> tuple[dict[str, float], dict[str, np.ndarray]]:
    """Per side body class (both sides' parts summed: the motion is planar), its mass (kg)
    and first moment (kg mm, world XY) in the fabricated cradle ``mech`` (its V blocks at
    the drawn bottle's setting, their lowest: the load case's)."""
    by = {b.name: b for b in mech.bodies}
    m_of: dict[str, float] = {}
    mom: dict[str, np.ndarray] = {}
    for b in mech.bodies:
        if b.part is None or b.fab == "display":
            continue
        cls = body_class(_host(b, by))
        m = part_mass_kg(config, b)
        com = mass_mod.part_props(b.placed_part()).com[:2]
        m_of[cls] = m_of.get(cls, 0.0) + m
        mom[cls] = mom.get(cls, np.zeros(2)) + m * com
    return m_of, mom


def _world_joints(config: BuildConfig, ts) -> dict[str, np.ndarray]:
    return config.lk.solve(1, 0.0, dict(config.proportions)).evaluate(np.asarray(ts, float))


def planar_model(config: BuildConfig, mech, t_ref: float, bottle: cr.BottleSize | None
                 ) -> tuple[list[Rigid], list[tuple[str, str, str]]]:
    """The moving bodies (:class:`Rigid`) and the joints ``(point, body, body-or-ground)``."""
    lk = config.lk
    m_of, moments = masses(config, mech)
    pts = _world_joints(config, [t_ref])
    links = {b: js for b, (js, _) in lk.links.items()}
    links["conn"] = lk.crank
    out = []
    for name, js in links.items():
        a, b = js[0], js[1]
        m = m_of.get(name, 0.0)
        com = moments.get(name, np.zeros(2)) / m if m > 0 else (pts[a][0] + pts[b][0]) / 2
        if name == cr.cradle_link(lk) and bottle is not None:
            y0, slope, x0 = cr.seat(bottle)
            cf_com = np.array([x0 + bottle.com_from_mouth * math.cos(slope),
                               y0 + bottle.com_from_mouth * math.sin(slope)])
            o, ax = cr.mouth_frame(lk)
            lip, d = pts[o][0], pts[ax][0] - pts[o][0]
            tilt = math.atan2(d[1], d[0])
            c, s = math.cos(tilt), math.sin(tilt)
            wcom = lip + np.array([c * cf_com[0] - s * cf_com[1], s * cf_com[0] + c * cf_com[1]])
            com = (com * m + wcom * bottle.mass_full_kg) / (m + bottle.mass_full_kg)
            m += bottle.mass_full_kg
        along, across = _frame_coords(com, pts[a][0], pts[b][0])
        out.append(Rigid(name, tuple(js), a, b, m, along, across))
    ground = set(lk.frame)
    owners: dict[str, list[str]] = {}
    for r in out:
        for j in r.joints:
            owners.setdefault(j, []).append(r.name)
    joints = []
    for j, own in owners.items():
        if j in ground:
            joints += [(j, o, "ground") for o in own]
        elif len(own) >= 2:
            joints += [(j, own[0], o) for o in own[1:]]
    return out, joints


def statics(config: BuildConfig, rigid: list[Rigid], joints, ts, jam_nm: float = 0.0):
    """Per crank angle: the crank torque that holds the cradle under gravity (N·m, positive
    turns the crank forward) and every joint's force (N); with ``jam_nm``, the cradle
    blocked (its rotation held at the output pivot) and ``jam_nm`` on the crank instead.
    (Lengths are mm: moments are solved in N·mm.)"""
    lk = config.lk
    pts = _world_joints(config, ts)
    names = [r.name for r in rigid]
    idx = {n: i for i, n in enumerate(names)}
    nj = len(joints)
    n_unk = 2 * nj + 1
    n_eq = 3 * len(rigid)
    crank_i = idx["conn"]
    cradle = cr.cradle_link(lk)
    torques = np.zeros(len(ts))
    forces = np.zeros((len(ts), nj))
    for k in range(len(ts)):
        A = np.zeros((n_eq, n_unk))
        rhs = np.zeros(n_eq)
        for r in rigid:
            i = idx[r.name]
            a, b = pts[r.a][k], pts[r.b][k]
            com = _from_frame(r.along, r.across, a, b)
            w = -r.mass * G
            rhs[3 * i + 1] -= w                       # sum Fy + w = 0
            rhs[3 * i + 2] -= com[0] * w              # moments about the origin: x Fy - y Fx
        for jk, (j, b1, b2) in enumerate(joints):
            p = pts[j][k]
            for body, sign in ((b1, 1.0), (b2, -1.0)):
                if body == "ground":
                    continue
                i = idx[body]
                A[3 * i, 2 * jk] += sign
                A[3 * i + 1, 2 * jk + 1] += sign
                A[3 * i + 2, 2 * jk] += -sign * p[1]
                A[3 * i + 2, 2 * jk + 1] += sign * p[0]
        if jam_nm:
            # the drive torque is given; the cradle's rotation held by an extra moment
            A[3 * crank_i + 2, -1] = 0.0
            rhs[3 * crank_i + 2] -= jam_nm * 1000.0
            A[3 * idx[cradle] + 2, -1] = 1.0
        else:
            A[3 * crank_i + 2, -1] = 1.0              # the drive's torque on the crank
        sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
        torques[k] = sol[-1] / 1000.0
        forces[k] = np.hypot(sol[0:2 * nj:2], sol[1:2 * nj:2])
    return torques, forces


def _toggles(config: BuildConfig, ts) -> np.ndarray:
    """Crank angles near a toggle (the tilt's turning points): the jam case skips them."""
    lk = config.lk
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    ends = [int(np.argmin(tilt)), int(np.argmax(tilt))]
    near = np.zeros(len(ts), bool)
    for e in ends:
        diff = np.degrees(np.abs((ts - ts[e] + math.pi) % (2 * math.pi) - math.pi))
        near |= diff < TOGGLE_DEG
    return near


def report(config: BuildConfig, clash_angles: int = 6, store=None) -> dict:
    """The cradle's verdict (the module docstring); ``clash_angles`` 0 skips the clashes."""
    if not is_cradle(config):
        raise ValueError(f"{config.linkage} is not a decanting cradle")
    lk = config.lk
    tmpl = template_for(config)
    design = design_side(tmpl, config)
    ts = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    out: dict = {"linkage": config.linkage, "name": lk.name}

    # the swing
    oc = lk.output_check(dict(config.proportions))
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))
    steps = [s for s in lk.check(dict(config.proportions)) if s.kind == "closure"]
    out["swing"] = {
        "rest_deg": round(float(tilt.min()), 2), "full_deg": round(float(tilt.max()), 2),
        "swing_deg": round(float(np.ptp(tilt)), 2),
        "mouth_wander_mm": round(float(oc.wander_mm or 0.0), 6),
        "transmission_deg": {s.point: [round(v, 1) for v in s.angle_deg] for s in steps},
        "worst_transmission_deg": round(min(min(s.angle_deg[0], 180 - s.angle_deg[1])
                                            for s in steps), 1),
    }
    # the bottles in the saddles, each at its own setting: how far its mouth moves
    out["bottles"] = []
    swing = math.radians(float(np.ptp(tilt)))
    for size in cr.SIZES:
        y0, slope, x0 = cr.seat(size)
        r = math.hypot(x0, y0)
        out["bottles"].append({"bottle": size.name, "mouth_off_pivot_mm":
                               [round(x0, 2), round(y0, 2)],
                               "mouth_wander_mm": round(2 * r * math.sin(swing / 2), 2),
                               "nose_up_deg": round(-math.degrees(slope), 2),
                               "setting_mm": _setting_row(size),
                               "full_kg": size.mass_full_kg})

    mech = fabricate(tmpl, config, 1.0, store=store)
    # the loads: a sealed full 6 L methuselah; the jam: a firm push at the grip
    rigid, joints = planar_model(config, mech, 1.0, cr.DESIGN)
    torque, forces = statics(config, rigid, joints, ts)
    near = _toggles(config, ts)
    _, jam_f = statics(config, rigid, joints, ts, jam_nm=jam_nm(grip_r=handle_r(config)))
    jam_f = np.where(near[:, None], 0.0, jam_f)
    box = NRV040
    peak = float(np.abs(torque).max())
    handle = peak / (box.ratio * box.eta_static)
    hand = (mech.meta.get("hand_crank") or {}).get("handle") or {}
    grip_r = float(hand.get("grip_r_mm", 60.0))
    out["drive"] = {
        "gearbox": box.key, "ratio": box.ratio, "static_efficiency": box.eta_static,
        "irreversible": box.irreversible(),
        "crank_torque_peak_nm": round(peak, 2), "gearbox_rating_nm": box.rated_nm,
        "gearbox_sf": round(box.rated_nm / max(peak, 1e-9), 2),
        "handle_torque_nm": round(handle, 3),
        "handle_force_n": round(handle / (grip_r / 1000.0), 1),
        "grip_radius_mm": grip_r,
        "jam_hand_force_n": JAM_HAND_N,
        "jam_torque_nm": round(jam_nm(box, grip_r), 2),
        "handle_turns_per_pour": round(box.ratio / 2.0, 1),
        "deg_per_handle_turn_mean": round(float(np.ptp(tilt)) / (box.ratio / 2.0), 2),
        "load_case": f"a sealed full {cr.DESIGN.name} ({cr.DESIGN.mass_full_kg} kg)",
    }
    if out["drive"]["handle_force_n"] > WARN_HANDLE_N:
        out.setdefault("warnings", []).append(
            f"the handle needs {out['drive']['handle_force_n']} N at its grip at worst")
    rider = HandCrank.rider(design.ctx)
    rows = joint_rows(config, rider, joints, forces, jam_f)
    out["joints"] = rows
    # the rod's buckling at its largest force (compression or not: the bound)
    rod_len = float(np.linalg.norm(pts[lk.crank[1]][0] - pts[
        next(j for j in lk.links[rider][0] if j != lk.crank[1])][0]))
    t_rod = design.ctx.sheet_t("link", rider)
    w = 2 * design.ctx.params.link_radius
    inertia = w * t_rod ** 3 / 12.0
    p_cr = math.pi ** 2 * E_ALU * inertia / rod_len ** 2
    rod_jk = next(jk for jk, (j, b1, b2) in enumerate(joints) if j == lk.crank[1])
    f_rod = float(forces[:, rod_jk].max())
    f_rod_jam = float(jam_f[:, rod_jk].max())
    out["rod"] = {"length_mm": round(rod_len, 1), "section_mm": [round(w, 2), round(t_rod, 3)],
                  "euler_n": round(p_cr, 1), "bottle_n": round(f_rod, 1),
                  "jam_n": round(f_rod_jam, 1), "sf_bottle": round(p_cr / max(f_rod, 1e-9), 2),
                  "sf_jam": round(p_cr / max(f_rod_jam, 1e-9), 2)}
    # the crankpin: a NETRF12 standing in the crank block, the rod's force at the rod's
    # mid-plane over the block's top (a cantilever that long out of its socket): it bends at
    # the block's top, taken as the 12 x 5.0 tube its tapped end may be there, at 1018's
    # hot-rolled minimum yield
    note = mech.meta.get("hand_crank") or {}
    rod_mid = design.plan.z(design.plan.layers[rider])[0] + design.ctx.sheet_t(
        "link", rider) / 2
    lever = float(note.get("crankpin_lever_mm", rod_mid - (note.get("arm_z_mm") or
                                                           [0.0, 0.0])[1]))
    od, bore = CRANKPIN_D, NETRF12_ID
    cap = math.pi * (od ** 4 - bore ** 4) / (32 * od) * NETRF10_YIELD / 1000.0  # N·m
    out["crankpin"] = {"lever_mm": round(lever, 2), "capacity_nm": round(cap, 2),
                       "bottle_nm": round(f_rod * lever / 1000.0, 2),
                       "jam_nm": round(f_rod_jam * lever / 1000.0, 2),
                       "sf_bottle": round(cap * 1000.0 / max(f_rod * lever, 1e-9), 2),
                       "sf_jam": round(cap * 1000.0 / max(f_rod_jam * lever, 1e-9), 2)}
    out["stand"] = stability(config, mech, design, ts)
    out["strength"] = strength_check(config, mech, rows, peak)
    moving = {*cr.REAR_MOVES, *cr.STOP_MOVES}
    out["saddles"] = saddles(float(tilt.min()), float(tilt.max()), sum(
        part_mass_kg(config, b) for b in mech.bodies if b.name in moving))
    if clash_angles:
        from spiderpig.construction.contract import clashes

        found = []
        ends = [float(ts[int(np.argmin(tilt))]), float(ts[int(np.argmax(tilt))])]
        for t in [*ends, *np.linspace(0.0, 2.0 * math.pi, clash_angles, endpoint=False)]:
            m = fabricate(tmpl, config, float(t), store=store)
            found.extend(dict(c, crank_deg=round(math.degrees(t), 1)) for c in clashes(m))
            found.extend(dict(c, crank_deg=round(math.degrees(t), 1)) for c in wheel_sweep(m))
        out["clashes"] = found
    return out


def _setting_row(size: cr.BottleSize) -> list[float]:
    """``size``'s setting (``construction.cradle.setting``): the front V block's rise, the
    rear one's, the base stop's face along the axis from the mouth (mm)."""
    s = cr.setting(size)
    return [round(s.front, 2), round(s.rear, 2), round(s.stop, 1)]


WINE_KG = {"standard": 0.75, "magnum": 1.5, "jeroboam": 3.0, "imperial": 6.0,
           "methuselah": 6.0}
"""The wine in each bottle size full (``construction.cradle.SIZES``), kg: what an empty one
weighs less."""


def stability(config: BuildConfig, mech, design, ts) -> dict:
    """The whole cradle's centre of mass over the swing, each bottle full and empty, against
    the base plate's feet (their pads' outer edges): its margin (mm) to the nearest tipping
    edge, front, rear and sideways (the gearbox and the hand wheel weigh on the drive side),
    the tilt it tips at and the push at the bottle's highest point that tips it, each the
    worst over the swing (the box's parts measured as ``mech`` was built, turned with it)."""
    lk = config.lk
    stand = cr.stand_of(lk)
    by = {b.name: b for b in mech.bodies}
    link = cr.cradle_link(lk)
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    meta = mech.meta or {}
    if "cradle_lip" in meta:                  # the pose the parts were built in
        lip0, t0 = np.asarray(meta["cradle_lip"], float), float(meta["cradle_tilt"])
    else:
        d0 = pts[ax][0] - pts[o][0]
        lip0, t0 = pts[o][0], math.atan2(d0[1], d0[0])
    table = float(lip0[1]) - cr.LIP_HEIGHT
    # every part's mass and centre: what stands, what turns with the box (about the mouth)
    still_m, still_c = 0.0, np.zeros(3)
    box_m, box_c = 0.0, np.zeros(3)
    moves = {**dict.fromkeys(cr.FRONT_MOVES, "front"), **dict.fromkeys(cr.REAR_MOVES, "rear"),
             **dict.fromkeys(cr.STOP_MOVES, "stop")}
    moved_m: dict[str, float] = dict.fromkeys(("front", "rear", "stop"), 0.0)  # (it moves)
    for b in mech.bodies:
        if b.part is None or b.fab == "display":
            continue
        m = part_mass_kg(config, b)
        c = np.asarray(mass_mod.part_props(b.placed_part()).com, float)
        if body_class(_host(b, by)) == link:
            box_m += m
            box_c += m * c
            if b.name in moves:
                moved_m[moves[b.name]] += m
        else:
            still_m += m
            still_c += m * c
    box_rel0 = (box_c / box_m)[:2] - lip0 if box_m else np.zeros(2)
    box_z = float(box_c[2] / box_m) if box_m else 0.0
    inset, pad = 25.0, 12.0                    # the feet (construction.cradle._base_plate)
    x_front = float(lip0[0]) + stand.base[0] + inset - pad
    x_rear = float(lip0[0]) + stand.base[1] - inset + pad
    z_edge = cr.BASE_HALF - inset + pad
    rows = []
    for size in cr.SIZES:
        # the box's centre with its V blocks and base stop at this bottle's setting
        d = cr.offsets(size)
        shift = (moved_m["front"] * np.array([0.0, d.front])
                 + moved_m["rear"] * np.array([0.0, d.rear])
                 + moved_m["stop"] * np.array([d.stop, d.rear])) / max(box_m, 1e-9)
        box_rel = box_rel0 + np.array([math.cos(t0) * shift[0] - math.sin(t0) * shift[1],
                                       math.sin(t0) * shift[0] + math.cos(t0) * shift[1]])
        y0, slope, x0 = cr.seat(size)
        com_cf = np.array([x0 + size.com_from_mouth * math.cos(slope),
                           y0 + size.com_from_mouth * math.sin(slope)])
        top_cf = np.array([x0 + size.length * math.cos(slope),
                           y0 + size.length * math.sin(slope) + size.r])
        wine = WINE_KG[size.key]
        for state, mb in (("full", size.mass_full_kg), ("empty", size.mass_full_kg - wine)):
            least: dict[str, tuple] = {}       # tip, push, margin: each its worst (and edge)
            for k in range(len(ts)):
                lip = pts[o][k]
                d = pts[ax][k] - pts[o][k]
                tl = math.atan2(d[1], d[0])
                c, s_ = math.cos(tl - t0), math.sin(tl - t0)
                box = lip + np.array([c * box_rel[0] - s_ * box_rel[1],
                                      s_ * box_rel[0] + c * box_rel[1]])
                c2, s2 = math.cos(tl), math.sin(tl)
                bot = lip + np.array([c2 * com_cf[0] - s2 * com_cf[1],
                                      s2 * com_cf[0] + c2 * com_cf[1]])
                top = lip[1] + s2 * top_cf[0] + c2 * top_cf[1]
                m = still_m + box_m + mb
                cx = (still_c[0] + box_m * box[0] + mb * bot[0]) / m
                cy = (still_c[1] + box_m * box[1] + mb * bot[1]) / m
                cz = (still_c[2] + box_m * box_z) / m        # the bottle on the bay's middle
                h = cy - table
                for edge, margin in (("front", cx - x_front), ("rear", x_rear - cx),
                                     ("side", z_edge - abs(cz))):
                    got = {"tip": math.degrees(math.atan2(margin, h)),
                           "push": m * G * margin / max(float(top - table), 1.0),
                           "margin": margin}
                    for key, v in got.items():
                        if key not in least or v < least[key][0]:
                            least[key] = (v, edge)
            rows.append({"bottle": size.name, "state": state, "kg": round(mb, 2),
                         "margin_mm": round(least["margin"][0], 1), "edge": least["tip"][1],
                         "tip_deg": round(least["tip"][0], 1),
                         "push_at_top_n": round(least["push"][0], 1),
                         "push_edge": least["push"][1]})
    return {"feet": {"x_mm": [round(x_front, 1), round(x_rear, 1)], "z_mm": round(z_edge, 1)},
            "device_kg": round(still_m + box_m, 3),
            "cases": rows, "worst_margin_mm": min(r["margin_mm"] for r in rows),
            "worst_tip_deg": min(r["tip_deg"] for r in rows)}


PLA_ALLOW = 10.0        # MPa: a printed PLA part under a load that stays (a quarter of its
#                         ~40 MPa across its layers: it creeps, it doesn't break)
STAINLESS_YIELD = 210.0     # MPa: the thumbscrews' 18-8, as annealed 304 (no class named)
CORKSCREW_N = 13.0      # a corkscrew's push into the cork (the mechanical review: 6-13 N)
LOCK_TORQUE_NM = 0.5    # the base stop's knob (26 mm) turned tight by hand
NUT_FACTOR = 0.2        # a dry steel screw's preload: torque / (factor x diameter)
PLA_FRICTION = 0.3
TPU_FRICTION = 0.5      # the rear lead screw's TPU 95A brake washer on aluminium and PLA, dry
LEAD_FRICTION = 0.08    # its bronze nut on the stainless screw, greased: the least it may be
KNOCK_N = 50.0          # a knock sideways on the rear knob (a hand, a bottle going in)
COM_MODEL = {"standard": 191.0, "magnum": 230.0, "jeroboam": 325.0, "imperial": 319.0,
             "methuselah": 390.0}
"""A full bottle's centre of mass along its axis from the mouth (mm) by review round 1's shell
model (glass shell, neck walls, base and punt scaled to each bottle's empty mass, the wine
filled from the base: 31-36 % of the length from the base, as a 750 ml's usual 11 cm up a 30
cm bottle). The checks take the larger of this and the bottle's own figure
(``BottleSize.com_from_mouth``): the further from the mouth, the more the bottle leans on the
rear support's end and the gate's couple pitches the rear V block."""


def com_of(size: cr.BottleSize, toward_mouth: bool = False) -> float:
    """``size``'s centre of mass for the checks (:data:`COM_MODEL`, the bottle's figure): the
    one further from the mouth, or nearer it (``toward_mouth``: the front V's share)."""
    pick = min if toward_mouth else max
    return pick(size.com_from_mouth, COM_MODEL.get(size.key, size.com_from_mouth))
BASE_RING = 10.0        # a bottle's base rests on a ring this far in from its body's radius
HEEL = 10.0             # its body's last 10 mm, rounded into the base, rest on nothing


def balance(rest_deg: float = -50.0, full_deg: float = 30.0) -> dict:
    """Where the saddles' reaction on each bottle centres along its axis (from the mouth),
    at rest, lying level and at full tilt, against the support it rests on: the front V's
    front end to the rear support's end (the rear V and the base stop's rails' flanges
    behind it, as far as the body's heel). At rest the gate pushes the base back below the
    axis (as low as it can: the base's ring or the gate's bottom edge, or the V's grip on
    the body, whichever is lower), at full tilt the lip gauge pushes the rim's top forward
    above it: either is a couple with the weight's pull along the axis, and moves the
    centre toward the base by its depth x tan(tilt) (review round 1, B1). The centre of mass:
    :func:`com_of`. A centre past the support's end lifts the neck off the front V: ``ok``
    false."""
    front, _ = cr.SADDLES
    x0 = front.vee[0]
    flange_end = cr.RAIL_END - cr.CLEAR
    rows = []
    for size in cr.SIZES:
        at = cr.setting(size)
        end = min(flange_end, size.length - HEEL)
        com = com_of(size)
        gate = min(size.r - BASE_RING, -(cr.GATE_BOTTOM + at.rear))
        depth_rest = max(gate, size.r / math.sqrt(2))
        rim = (cr.GAUGE_PAD[0] + size.r_neck) / 2
        for case, deg, depth in (("rest", rest_deg, depth_rest), ("level", 0.0, 0.0),
                                 ("full", full_deg, rim)):
            x = com + depth * math.tan(math.radians(abs(deg)))
            rows.append({"bottle": size.name, "case": case, "tilt_deg": round(deg, 1),
                         "com_mm": round(com, 1), "couple_depth_mm": round(depth, 1),
                         "centre_mm": round(x, 1), "support_mm": [x0, round(end, 1)],
                         "margin_mm": round(end - x, 1), "ok": x0 < x < end})
    return {"rows": rows, "worst_margin_mm": min(r["margin_mm"] for r in rows),
            "ok": all(r["ok"] for r in rows)}


def _section(zy) -> tuple[float, float, float]:
    """A polygon's (z, y) area, second moment about its horizontal centroidal axis and its
    farthest fibre from that axis (mm): a rail's section under a vertical load."""
    pts = [*zy, zy[0]]
    a = cy = iy = 0.0
    for (z0, y0), (z1, y1) in itertools.pairwise(pts):
        cross = z0 * y1 - z1 * y0
        a += cross
        cy += (y0 + y1) * cross
        iy += (y0 * y0 + y0 * y1 + y1 * y1) * cross
    a /= 2.0
    cy /= 6.0 * a
    i = iy / 12.0 - a * cy * cy
    far = max(abs(y - cy) for _, y in zy)
    return abs(a), abs(i), far


def saddles(rest_deg: float = -50.0, full_deg: float = 30.0, moving_kg: float = 0.0
            ) -> dict:
    """The saddles' loads and stresses, a sealed full bottle of each size at its own setting
    over the swing (``rest_deg`` to ``full_deg``), its weight the worst way for each part
    (its centre of mass :func:`com_of`), and :func:`balance`:

    * the rear V block on its tie rods (a 6 L methuselah's V blocks sit on them);
    * a V block hung on its thumbscrew (the heaviest a 6 L set a hair off the rods): the
      screw's barrel nut in the block, the roll couple of a load at the axis held at the
      block's end (the post, stiffer, and the screw share it), and at rest the pitch of the
      bottle's pull along the axis, 121 mm over the post's root, on the post too;
    * the rear lead screw: its brake (the collar's TPU washer under the block's load) against
      the thread turning back under it, its span and its tower under a knock at the knob;
    * the base stop at rest (the bottle's pull down the axis and a corkscrew's push): its
      gate's plate in bending between its ears, its lock's grip (the knob's preload on two
      faces) and the T-slot's lips under that preload; its rails as cantilevers from their
      sockets, the body's share on their flanges at worst (the bottle on the front V and the
      flanges' end), the sockets' floors under their tongues;
    * the lip gauge at full tilt (the weight down the axis, no friction counted): its pad,
      its arm across the mouth, its bar beside the neck, its bridge over the post (the push
      is off their lines by the hinge's 41 mm across and the rim's height), the post (bent
      by the push, twisted by its turn) and the pin (the turn's couple on its knuckles).

    Each row: the load (N, or N·m where the case says), the stress (MPa) and its limit
    (:data:`PLA_ALLOW` for a printed part, the steel's yield), their ratio ``sf``.
    ``moving_kg``: what rides the rear V block (itself, its liner, the base stop), hung with
    the bottle."""
    g = G
    heavy = max(cr.SIZES, key=lambda z: z.mass_full_kg)
    w_max = heavy.mass_full_kg * g
    front, rear = cr.SADDLES
    rows: list[dict] = []

    def row(part: str, case: str, load: float, stress: float, limit: float) -> None:
        rows.append({"part": part, "case": case, "load_n": round(load, 1),
                     "stress_mpa": round(stress, 2), "limit_mpa": limit,
                     "sf": round(limit / max(stress, 1e-9), 2)})

    s_rest = math.sin(math.radians(-rest_deg))
    c_rest = math.cos(math.radians(rest_deg))
    # the rear V block on its two rods (the 6 L, lying level): each seat a 9 mm wide arc
    # over 24 mm of rod
    seat_area = 2.0 * math.sqrt(cr.SEAT_R ** 2 - (cr.SEAT_R - cr.SEAT_DEPTH) ** 2) * 24.0
    row("rear V block's seats on the tie rods", "a full 6 L methuselah, level", w_max,
        w_max / 2 / seat_area, PLA_ALLOW)
    # hung on its lead screw: the barrel nut bears up on the bore's end over it
    hung = (heavy.mass_full_kg + moving_kg) * g
    nd = cr.LEAD_NUT[0]
    ring = math.pi / 4 * ((nd + 0.2) ** 2 - (cr.LEAD_SCREW[0] + 1.0) ** 2)
    row("rear barrel nut on the V block", "a full 6 L, hung, level", hung, hung / ring,
        PLA_ALLOW)
    zs = cr.REAR_LEAD[1]
    xp = cr.REAR_POST[0]
    post_i = math.pi * cr.POST_D ** 4 / 64
    post_s = math.pi * cr.POST_D ** 3 / 32
    root = cr.LEAD_SCREW[2]
    screw_i = math.pi * root ** 4 / 64
    screw_s = math.pi * root ** 3 / 32
    share = post_i / (post_i + screw_i)
    couple = hung * zs                                        # N·mm, about the axis
    row("rear post (NETRF12, bending)", "the roll of a full 6 L hung at the screw (N·m)",
        couple * share / 1000, couple * share / post_s, NETRF10_YIELD)
    row("rear lead screw (bending)", "its share of that roll (N·m)",
        couple * (1 - share) / 1000, couple * (1 - share) / screw_s, STAINLESS_YIELD)
    # at rest, a bottle hung on the screw (any but the methuselah, whose block sits on its
    # rods): its pull along the axis, at the axis, pitches the block about the post's root
    worst = (0.0, None)
    for size in cr.SIZES:
        if cr.setting(size).rear <= 1e-9:
            continue
        m = (size.mass_full_kg + moving_kg) * g
        com = com_of(size)
        pitch = m * c_rest * (com - xp) + (m * s_rest + CORKSCREW_N) * -cr.REAR_FOOT_TOP
        roll = m * c_rest * zs
        both = math.hypot(pitch, roll)
        if both > worst[0]:
            worst = (both, size)
    both, size = worst
    if size is not None:
        row("rear post (NETRF12, bending at rest)", f"a full {size.name} hung: the pull "
            "along the axis and the roll (N·m)", both * share / 1000,
            both * share / post_s, NETRF10_YIELD)
    # the V block at the post's boss: the post's share of the roll into its flats and the
    # gusset under them
    depth = cr.REAR_FLANGE[1] - cr.GUSSET[1]
    row("rear V block at the post's boss", "the post's share of the roll (N·m)",
        couple * share / 1000, couple * share / (cr.GUSSET[0] * depth ** 2 / 6), PLA_ALLOW)
    # the lead screw's brake: the collar's TPU washer, pressed by the block's load, against
    # the thread turned back by it (its lead 8 mm: no self-locking)
    od, d = cr.COLLAR[0] / 2, cr.LEAD_SCREW[0] / 2 + 0.3
    r_brake = 2.0 / 3.0 * (od ** 3 - d ** 3) / (od ** 2 - d ** 2)
    pitch_d = cr.LEAD_SCREW[0] - 1.0                          # its mean diameter, Tr8 x 2
    lead_a = math.atan(cr.LEAD / (math.pi * pitch_d))
    phi = math.atan(LEAD_FRICTION / math.cos(math.radians(15.0)))
    back = cr.LEAD / (2 * math.pi) * math.tan(lead_a - phi) / math.tan(lead_a)  # mm per N
    rows.append({"part": "rear lead screw's brake (no turning back)",
                 "case": "any load on the block: its brake's grip per its pull back (mm·N/N)",
                 "load_n": round(back, 3), "grip_n": round(TPU_FRICTION * r_brake, 3),
                 "sf": round(TPU_FRICTION * r_brake / back, 2)})
    # a knock at the rear knob: into the tower's arm (the screw's top in its bushing), the
    # tower bending at its root; the screw between the nut and the bushing, knocked midway
    tx0, tx1, tz0, tz1 = cr.TOWER
    k0, k1 = cr.REAR_KNOB
    lever = (k0 + k1) / 2 - cr.REAR_FOOT_TOP
    t_s = min((tx1 - tx0) * (tz1 - tz0) ** 2, (tz1 - tz0) * (tx1 - tx0) ** 2) / 6
    row("rear knob's tower (bending)", "a 50 N knock sideways at the knob", KNOCK_N,
        KNOCK_N * lever / t_s, PLA_ALLOW)
    span = cr.ARM[1] - cr.LEAD_BOSS[2]
    row("rear lead screw (bending)", "a 50 N knock midway between the nut and the bushing",
        KNOCK_N, KNOCK_N * span / 4 / screw_s, STAINLESS_YIELD)
    share_f = max(max((cr.SADDLES[1].vee[0] + cr.SADDLES[1].vee[1]) / 2
                      - com_of(z, toward_mouth=True), 0.0)
                  / ((cr.SADDLES[1].vee[0] + cr.SADDLES[1].vee[1]) / 2 - sum(front.vee) / 2)
                  * z.mass_full_kg * g for z in cr.SIZES)
    row("front thumbscrew's nut on the V block", "the heaviest front share, level", share_f,
        share_f / (3 * math.sqrt(3) / 2 * (cr.M6_HEX[0] / math.sqrt(3)) ** 2
                   - math.pi * 3.3 ** 2), PLA_ALLOW)
    # the base stop at rest: down the axis, the gate's plate a beam between its ears
    push = w_max * s_rest + CORKSCREW_N
    span = 2 * (cr.GROOVE[0] + cr.GROOVE[1]) / 2
    height = cr.REAR_FLANGE[1] + 6.0 - cr.GATE_BOTTOM
    row("base stop's gate (bending)", "a full 6 L at rest, a corkscrew's push", push,
        push * span / 4 / (height * cr.GATE_T ** 2 / 6), PLA_ALLOW)
    preload = LOCK_TORQUE_NM * 1000 / (NUT_FACTOR * 5.0)
    grip = 2 * PLA_FRICTION * preload
    rows.append({"part": "base stop's lock (its grip)", "case": "the same: its knob tight "
                 "by hand, the ear on the rail and the washer on the T-slot's lips",
                 "load_n": round(push, 1), "grip_n": round(grip, 1),
                 "sf": round(grip / push, 2)})
    _, neck, lip, (ww, _), _, _, _ = cr.TSLOT
    wd = cr.WASHER[0]
    reach = ww / 2 - (neck / 2 + wd / 2) / 2                   # the load's lever to the root
    width = wd + 2 * lip                                       # along the slot (45° spread)
    row("base stop's T-slot lips (bending)", "the lock's preload", preload,
        preload / 2 * reach / (width * lip ** 2 / 6), PLA_ALLOW)
    # the rails: the body's share on the flanges at worst, each rail a cantilever from its
    # socket; the socket's floor under the tongue's back end
    lo, hi, _ = cr.REAR_FLANGE
    yb, ft = cr.FLANGE
    apex = rear.apex

    def zv(y: float) -> float:
        return y - apex

    _, rail_i, far = _section([(zv(hi), hi), (cr.RAIL_Z, hi), (cr.RAIL_Z, lo),   # (the -z
                               (zv(lo) + ft, lo), (zv(yb) + ft, yb), (zv(yb), yb)])  # one's)
    xr, xf = rear.vee[1], sum(front.vee) / 2
    m_rail, case = 0.0, ""
    for z in cr.SIZES:
        end = min(cr.RAIL_END - cr.CLEAR, z.length - HEEL)
        if end <= xr:
            continue
        com = com_of(z)
        for name, deg, depth in (("at rest", rest_deg, max(min(
                z.r - BASE_RING, -(cr.GATE_BOTTOM + cr.setting(z).rear)),
                z.r / math.sqrt(2))), ("level", 0.0, 0.0)):
            wn = z.mass_full_kg * g * math.cos(math.radians(deg))
            x = com + depth * math.tan(math.radians(abs(deg)))
            on = wn * (x - xf) / (end - xf)                    # on the flanges' end at worst
            m = on * (end - xr) / 2                            # per rail
            if m > m_rail:
                m_rail, case = m, f"a full {z.name} {name}: its body on the flanges' end (N·m)"
    row("base stop's rails (bending at their sockets)", case, m_rail / 1000,
        m_rail * far / rail_i, PLA_ALLOW)
    tx0_, tz0_, tz1_, _ = cr.TONGUE
    bear = m_rail / ((xr - tx0_) * 2 / 3)                      # the tongue's ends' bearing
    t_floor = cr.SOCKET[3] - cr.RAIL_HEAD_SINK / 2
    span_f = tz1_ - tz0_ + 2 * cr.CLEAR
    row("base stop's sockets (their floors under the tongues)", "the same", bear,
        3 * bear * span_f / (2 * (xr - tx0_) / 2 * t_floor ** 2), PLA_ALLOW)
    # the lip gauge at full tilt: the weight down the axis on its pad (on the rim's top)
    pull = w_max * math.sin(math.radians(full_deg))
    bottom, half = cr.GAUGE_PAD
    ax0, ax1 = cr.GAUGE_FRONT
    y0, y1, bz0, bz1 = cr.GAUGE_BAR
    by0, by1, rz0, rz1 = cr.GAUGE_BRIDGE
    yp, zp = cr.GAUGE_PIVOT
    rim = (bottom + max(z.r_neck for z in cr.SIZES)) / 2
    t = ax1 - ax0                                              # their depth along x
    row("lip gauge's pad (bending)", "a full 6 L at full tilt, no friction", pull,
        pull * (y0 - bottom) / ((2 * half) * t ** 2 / 6), PLA_ALLOW)
    row("lip gauge's arm (bending)", "the same", pull,
        pull * -bz1 / ((y1 - y0) * t ** 2 / 6), PLA_ALLOW)
    bw, bh = bz1 - bz0, y1 - y0
    row("lip gauge's bar (bending both ways)", "the same", pull,
        pull * -(bz0 + bz1) / 2 / (bh * bw ** 2 / 6)
        + pull * ((y0 + y1) / 2 - rim) / (bw * bh ** 2 / 6), PLA_ALLOW)
    rw, rh = rz1 - rz0, by1 - by0
    row("lip gauge's bridge (bending both ways)", "the same", pull,
        pull * -zp / (rh * rw ** 2 / 6) + pull * ((by0 + by1) / 2 - rim) / (rw * rh ** 2 / 6),
        PLA_ALLOW)
    px0, px1, _ = cr.GAUGE_POST
    pl, pw = px1 - px0, -cr.FRONT_HALF - cr.CLEAR - cr.FRONT_W[0]   # along x, across
    turn = pull * -zp                                          # N·mm, about the post's axis
    sigma = pull * (yp - cr.FRONT_TOP) / (pw * pl ** 2 / 6)
    a, b = max(pl, pw), min(pl, pw)
    tau = turn / ((1 / 3 - 0.21 * b / a * (1 - (b / a) ** 4 / 12)) * a * b ** 2)
    row("lip gauge's post (bent and twisted)", "the same (its knuckle on the post's back)",
        pull, math.sqrt(sigma ** 2 + 3 * tau ** 2), PLA_ALLOW)
    (f0, f1), (b0, b1) = cr.GAUGE_KNUCKLES
    spacing = (b0 + b1) / 2 - (f0 + f1) / 2
    on_pin = turn / spacing                                    # each knuckle on the pin
    m_pin = on_pin * ((f1 - f0) / 2 + (px0 - f1))
    row("lip gauge's pin (M5, bending)", "its knuckles' couple, the push's turn", on_pin,
        m_pin / (math.pi * 4.134 ** 3 / 32), STAINLESS_YIELD)
    row("lip gauge's post's bore (bearing)", "the pin's couple in it", on_pin,
        6 * turn / ((cr.TIE_HOLE - 0.2) * pl ** 2), PLA_ALLOW)
    worst_sf = min(r["sf"] for r in rows)
    return {"rows": rows, "worst_sf": worst_sf, "balance": balance(rest_deg, full_deg),
            "gauge": cr.gauge_return()}


def wheel_sweep(mech) -> list[dict]:
    """What the hand crank's turning would hit: its tube about the input shaft, the disc its
    hub's thumb screw sweeps, and the disc its arm and grip sweep, against every part but the
    crank's own and the gearbox it rides (the crank is drawn at rest)."""
    from build123d import Axis, Cylinder, Location

    w = (mech.meta.get("hand_crank") or {}).get("handle")
    if not w:
        return []
    end, d = np.asarray(w["end_xy"], float), np.asarray(w["dir"], float)
    z = float(w["axis_z"]) - float(mech.meta.get("mid_plane", 0.0))
    a = math.degrees(math.atan2(d[1], d[0]))
    reach, t_arm = float(w["reach_mm"]), float(w["arm_t_mm"])

    def cyl(r: float, t0: float, t1: float):
        # along +x from the shaft's end, then turned to the shaft's way (as it is built)
        return Cylinder(r, t1 - t0, rotation=(0.0, 90.0, 0.0)).moved(
            Location(((t0 + t1) / 2, 0.0, 0.0))).rotate(Axis.Z, a).moved(
            Location((float(end[0]), float(end[1]), z)))

    sweep = cyl(float(w["hub_r_mm"]), -float(w.get("hub_len_mm", 0.0)), reach - t_arm) + cyl(
        float(w["r_mm"]), reach - t_arm, reach + float(w["grip_len_mm"]))
    if "thumb_r_mm" in w:               # the hub's thumb screw's head, turning with it
        sweep = sweep + cyl(float(w["thumb_r_mm"]), *map(float, w["thumb_x_mm"]))
    out = []
    box = sweep.bounding_box()
    for b in mech.bodies:
        if b.part is None or "handle" in b.name or b.name.endswith("gearbox"):
            continue
        part = b.placed_part()
        bb = part.bounding_box()
        if not all(max(getattr(box.min, c), getattr(bb.min, c)) < min(getattr(box.max, c),
                                                                       getattr(bb.max, c))
                   for c in "XYZ"):
            continue
        inter = sweep & part
        v = 0.0 if inter is None else sum(x.volume for x in inter.solids())
        if v > 1.0:
            out.append({"a": "hand wheel's sweep", "b": b.name, "mm3": round(v, 1)})
    return out


def joint_rows(config: BuildConfig, rider: str, joints, forces, jam_f) -> list[dict]:
    """Each joint's peak force over the turn (N), the bottle's and the jam's, per side
    (:data:`SHARE` on the pins both sides have)."""
    lk = config.lk
    rows = []
    for jk, (j, b1, b2) in enumerate(joints):
        on_drive = j in lk.crank or rider in (b1, b2)
        share = 1.0 if on_drive else SHARE
        rows.append({"joint": j, "bodies": [b1, b2], "drive_side_only": on_drive,
                     "bottle_n": round(float(forces[:, jk].max()) * share, 1),
                     "jam_n": round(float(jam_f[:, jk].max()) * share, 1)})
    return rows


def loads(config: BuildConfig, store=None) -> dict:
    """The joint loads :mod:`spiderpig.strength` rates a cradle at (what
    :func:`strength.design_loads` returns for one: the audit's, ``explain``'s and verify's
    pin and plate rows): a sealed full 6 L methuselah's walking, a firm push at the grip
    jammed (:func:`jam_nm`)."""
    tmpl = template_for(config)
    design = design_side(tmpl, config)
    mech = fabricate(tmpl, config, 1.0, store=store)
    ts = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    rigid, joints = planar_model(config, mech, 1.0, cr.DESIGN)
    torque, forces = statics(config, rigid, joints, ts)
    _, jam_f = statics(config, rigid, joints, ts, jam_nm=jam_nm(grip_r=handle_r(config)))
    jam_f = np.where(_toggles(config, ts)[:, None], 0.0, jam_f)
    rows = joint_rows(config, HandCrank.rider(design.ctx), joints, forces, jam_f)
    return loads_doc(config, rows, float(np.abs(torque).max()))


def loads_doc(config: BuildConfig, rows: list[dict], crank_nm: float) -> dict:
    """:func:`joint_rows` as :func:`strength.check`'s loads document (source ``statics``:
    per joint, as a sim's)."""
    lk = config.lk
    ground = set(lk.frame)
    joints = []
    for r in rows:
        bodies = [b for b in r["bodies"] if b != "ground"]
        joints.append({"stem": r["joint"], "crank": r["joint"] in lk.crank,
                       "frame": r["joint"] in ground, "links": bodies,
                       "walk": {"n": r["bottle_n"], "patterns": None},
                       "jam": {"n": r["jam_n"], "patterns": None}})
    return {"source": "statics", "note": "a cradle's: a sealed full 6 L methuselah (walk), a firm "
            f"push at the grip on a blocked cradle ({JAM_HAND_N:g} N: {jam_nm():g} N·m at "
            "the crank, jam); cradle_report",
            "walk_n": max((j["walk"]["n"] for j in joints), default=0.0),
            "jam_n": max((j["jam"]["n"] for j in joints), default=0.0), "joints": joints,
            "torque_limit_nm": jam_nm(grip_r=handle_r(config)),
            "walk_torque_nm": round(crank_nm, 2),
            "joint_moment_factor": 1.0,
            "crank_basis": f"statics: the bottle's largest torque over a pour, a {JAM_HAND_N:g} "
                           "N push at the grip jammed"}


def strength_check(config: BuildConfig, mech, rows: list[dict], crank_nm: float) -> dict:
    """The pins and link plates rated by :func:`spiderpig.strength.check` at the cradle's
    loads: "walk" a sealed full 6 L methuselah's, "jam" a firm push at the grip on a blocked
    cradle (per side, as :func:`report` shares them)."""
    from spiderpig import strength

    st = strength.check(mech.meta.get("wobble") or {}, mech.meta, config,
                        loads_doc(config, rows, crank_nm))
    keep = ("joint", "kind", "links", "span_mm", "section", "sheet", "thickness_mm")
    return {"rows": [{**{k: r[k] for k in keep if k in r},
                      "walk_sf": (r.get("walk") or {}).get("safety"),
                      "jam_sf": (r.get("jam") or {}).get("safety"),
                      "walk_n": (r.get("walk") or {}).get("load_n"),
                      "jam_n": (r.get("jam") or {}).get("load_n")} for r in st["rows"]],
            "worst": st["worst"], "findings": st["findings"]}


# -- the cradle page's swing (``/api/cradle/{key}``, ``viewer/decant.html``) ------------

VERDICT = {
    "decanter_hinge": ("build", "The one to build: a four-bar crank-rocker whose rocker is the "
                       "cradle, hinged at the bottle's mouth. From 50° neck up (an opened "
                       "full bottle goes in) to 30° neck down, a 750 ml to a 6 L. Every check "
                       "passes with a full 6 L: the joints hold a forced crank (jam SF 1.6 at "
                       "worst), the gearbox 1.44 x its rating, the stand tips past 25°."),
    "decanter_rcm": ("build", "To build too: an eight-bar double parallelogram turns the "
                     "cradle about a point with nothing there, so nothing stands at the "
                     "mouth; its pivots hang under the mouth, its crank behind. Every check "
                     "passes with a full 6 L: the joints hold a forced crank (jam SF 1.31 at "
                     "worst), the gearbox 1.52 x its rating, the stand tips past 25°."),
}
"""Each cradle's standing (``docs/agentlib/DECANTER.md``): ``build`` or ``study``, and why."""


def motion(config: BuildConfig, n: int = 360) -> dict:
    """A cradle's swing over one crank turn, sampled as its bake's clip is (``n`` crank angles
    from 0, the clip's time over its length): every joint's path (mm, the linkage's plane),
    the bottle's tilt (degrees, neck down positive), the handle's turns from rest (the clip's
    direction), how far the mouth wanders; the bodies' outlines, the drawn bottle, every
    bottle and the V blocks (at the drawn bottle's setting) in the cradle's frame (a point
    ``(x, y)`` there is at ``mouth + R(tilt) (x, y)``), for the cradle page's diagram; each
    bottle's setting (:func:`_setting_row`) and the parts a setting moves (``adjust``: the
    page moves their nodes from the drawn bottle's setting to the one picked)."""
    if not is_cradle(config):
        raise ValueError(f"{config.linkage} is not a decanting cradle")
    lk = config.lk
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))
    tilt -= 360.0 * round(float(tilt.mean()) / 360.0)
    rest, full = int(np.argmin(tilt)), int(np.argmax(tilt))
    mouth = cr.mouth_name(lk)
    lip = pts[mouth]
    wander = float(np.linalg.norm(lip - lip.mean(axis=0), axis=1).max())
    ratio = NRV040.ratio
    turns = ((np.arange(n) - rest) % n) / n * ratio
    def outline_of(size: cr.BottleSize) -> list[tuple[float, float]]:
        y0, slope, x0 = cr.seat(size)
        c, s = math.cos(slope), math.sin(slope)
        prof = size.profile()
        return ([(x0 + u * c - r * s, y0 + u * s + r * c) for u, r in prof]
                + [(x0 + u * c + r * s, y0 + u * s - r * c) for u, r in reversed(prof)])

    kind, why = VERDICT.get(lk.key, ("study", ""))

    def r2(a) -> list:
        return np.round(np.asarray(a, dtype=float), 2).tolist()

    return {
        "key": lk.key, "name": lk.name, "notes": lk.notes, "source": lk.source,
        "verdict": kind, "why": why, "labels": dict(lk.labels),
        "frame": list(lk.frame), "crank": list(lk.crank), "mouth": mouth, "axis": ax,
        "output_link": lk.output.link if lk.output else None,
        "links": {b: {"joints": list(js), "edges": [list(e) for e in edges]}
                  for b, (js, edges) in lk.links.items()},
        "points": {k: r2(v) for k, v in pts.items()},
        "tilt_deg": r2(tilt), "handle_turns": r2(turns),
        "rest": rest, "full": full, "n": n,
        "rest_deg": round(float(tilt[rest]), 1), "full_deg": round(float(tilt[full]), 1),
        "turns_to_full": round(float(turns[full]), 1), "ratio": ratio,
        "mouth_wander_mm": round(wander, 3),
        "bottle": {"key": cr.SHOWN.key, "name": cr.SHOWN.name,
                   "outline": r2(outline_of(cr.SHOWN)), "setting": _setting_row(cr.SHOWN)},
        "bottles": [{"key": z.key, "name": z.name, "outline": r2(outline_of(z)),
                     "setting": _setting_row(z)} for z in cr.SIZES],
        "adjust": {"front": list(cr.FRONT_MOVES), "rear": list(cr.REAR_MOVES),
                   "stop": list(cr.STOP_MOVES)},
        "saddles": [{"name": sd.name, "x": list(sd.vee), "y": [sd.bottom, sd.top],
                     "moves": key} for sd, key in zip(cr.SADDLES, ("front", "rear"),
                                                    strict=True)],
        "ties": {k: list(v) for k, v in _ties_of().items()},
    }


def _ties_of() -> dict[str, tuple[float, float]]:
    from spiderpig.linkages.decanters import TIES

    return dict(TIES)


ROOM_BAND = 5.0         # the room's height steps (mm)
_MOVING = ("tie_", "front_saddle", "rear_saddle", "led", "base_stop", "bottle")


def _surface_points(part, step: float = 5.0) -> np.ndarray:
    """Points over ``part``'s surface no more than about ``step`` apart (its tessellation's
    triangles filled in, so a long tie rod's middle counts)."""
    verts, tris = part.tessellate(0.5, 0.5)
    v = np.array([[p.X, p.Y, p.Z] for p in verts], float)
    out = [v]
    for a, b, c in tris:
        pa, pb, pc = v[a], v[b], v[c]
        n = math.ceil(max(np.linalg.norm(pb - pa), np.linalg.norm(pc - pa),
                          np.linalg.norm(pc - pb)) / step)
        if n <= 1:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        keep = i + j <= n
        u, w = i[keep] / n, j[keep] / n
        out.append(pa + np.outer(u, pb - pa) + np.outer(w, pc - pa))
    return np.concatenate(out)


def room(config: BuildConfig, mech, n: int = 72) -> dict:
    """The room a vessel has under the mouth (``/api/cradle/{key}/room``, the cradle page's
    fit check): at each height over the table (every :data:`ROOM_BAND` mm up to the mouth),
    the radius round the decanter's axis that no part of the cradle box or the bottle drawn
    (a 6 L methuselah, the V blocks at their lowest) enters over the whole swing (``room``),
    and from the pour on (the cradle past -20°, where an opened full bottle starts to pour:
    ``room_pouring``). ``mech``: the cradle fabricated at crank angle 0. The side stacks
    stand outside the bay (``construction.cradle.BAY``), so only the bay's parts count. A
    thinner bottle's V blocks ride higher, away from the decanter at every tilt: the
    drawn setting bounds them."""
    lip = np.asarray(mech.meta["cradle_lip"], float)
    axis_x = float(lip[0]) + cr.DECANTER_X
    table = float(lip[1]) - cr.LIP_HEIGHT
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = _world_joints(config, np.concatenate([[0.0], ts]))
    o, ax = cr.mouth_frame(config.lk)
    d = pts[ax] - pts[o]
    tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    moving, fixed = [], []
    for b in mech.bodies:
        if b.part is None or b.name.startswith(("L.", "R.")) or b.name.startswith("decanter"):
            continue
        p = _surface_points(b.placed_part())
        (moving if b.name.startswith(_MOVING) else fixed).append(p)
    mv = np.concatenate(moving)
    t0 = tilt[0]
    fx = np.concatenate(fixed) if fixed else np.zeros((0, 3))
    bands = np.arange(0.0, cr.LIP_HEIGHT + 1e-6, ROOM_BAND)

    def radii(points: np.ndarray) -> np.ndarray:
        h = points[:, 1] - table
        r = np.hypot(points[:, 0] - axis_x, points[:, 2])
        k = np.floor(h / ROOM_BAND).astype(int)
        ok = (k >= 0) & (k < len(bands))
        out = np.full(len(bands), np.inf)
        np.minimum.at(out, k[ok], r[ok])
        return out

    whole = radii(fx) if len(fx) else np.full(len(bands), np.inf)
    pouring = whole.copy()
    rel = mv[:, :2] - lip
    for t in tilt[1:]:
        a = t - t0
        ca, sa = math.cos(a), math.sin(a)
        p = np.column_stack([lip[0] + ca * rel[:, 0] - sa * rel[:, 1],
                             lip[1] + sa * rel[:, 0] + ca * rel[:, 1], mv[:, 2]])
        rr = radii(p)
        whole = np.minimum(whole, rr)
        if math.degrees(t) >= -20.0:
            pouring = np.minimum(pouring, rr)
    cap = float(cr.BAY / 2)                 # past the bay: the side stacks
    return {
        "room": [[float(h), round(float(min(r, cap)), 1)]
                 for h, r in zip(bands, whole, strict=True)],
        "room_pouring": [[float(h), round(float(min(r, cap)), 1)]
                         for h, r in zip(bands, pouring, strict=True)],
        "axis_x": axis_x, "mouth_over_table": cr.LIP_HEIGHT,
    }
