"""How a decanting cradle does its job: the swing, the loads, the hand, the stand, the fit.

A cradle (:mod:`construction.cradle`) is judged here, not by the walking model or MuJoCo:

* **The swing**: the tilt range and how still the bottle's mouth stays
  (:meth:`linkage.Linkage.output_check`), each loop's transmission angles.
* **The bottles** (:data:`construction.cradle.SIZES`): where each one's mouth sits off the
  pivot in the saddles (:func:`construction.cradle.seat`).
* **The loads**: a quasi-static planar solve (:func:`statics`) of the side's bodies, the
  cradle box's mass and a **sealed full 3 L bottle** (the worst a cradle carries: nothing
  pours out of it) on the cradle, gravity alone, at every crank angle: the crank's torque,
  every pin's force. A **jam** case puts the gearbox's rated torque on the crank with the
  cradle blocked, away from the crank-rocker's toggles (where nothing can bound it): what the
  hand can push through the worm before the gearbox itself is the weak part. The pins are
  rated by :mod:`spiderpig.strength` at these loads ("walk" the bottle's, "jam" the jam's).
* **The hand**: the crank torque through the worm (its ratio, its static efficiency: the
  slow end of a hand's speed) at the handle's throw; turns per pour.
* **The rod**: Euler buckling of the connecting rod at its largest compression.
* **The stand**: the centre of mass of the cradle with each bottle, full and empty, over
  the swing, against the feet (the base ties' line along x, the two outer plates across).
* **Clashes**: every part against every other at angles across the swing
  (:func:`construction.contract.clashes`), the bottle and the decanter included.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig.config import BuildConfig, is_cradle
from spiderpig.construction import cradle as cr
from spiderpig.construction.handcrank import CRANKPIN, NRV030, HandCrank
from spiderpig.fabricate import design_side, fabricate, template_for
from spiderpig.hardware import mass as mass_mod
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
CRANKPIN_YIELD = 965.0  # MPa: the shoulder screw's alloy steel (McMaster 92981A, 140 ksi)


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
    and first moment (kg mm, world XY) in the fabricated cradle ``mech``."""
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
    # the bottles in the saddles, each on its adapters: how far its mouth moves
    out["bottles"] = []
    swing = math.radians(float(np.ptp(tilt)))
    for size in cr.SIZES:
        y0, slope, x0 = cr.seat(size)
        r = math.hypot(x0, y0)
        out["bottles"].append({"bottle": size.name, "mouth_off_pivot_mm":
                               [round(x0, 2), round(y0, 2)],
                               "mouth_wander_mm": round(2 * r * math.sin(swing / 2), 2),
                               "nose_up_deg": round(-math.degrees(slope), 2),
                               "adapters_mm": [round(v, 1) for v in cr.adapter(size)],
                               "full_kg": size.mass_full_kg})

    mech = fabricate(tmpl, config, 1.0, store=store)
    # the loads: a sealed full 3 L bottle; the jam at the gearbox's rating
    rigid, joints = planar_model(config, mech, 1.0, cr.JEROBOAM)
    torque, forces = statics(config, rigid, joints, ts)
    near = _toggles(config, ts)
    _, jam_f = statics(config, rigid, joints, ts, jam_nm=NRV030.rated_nm)
    jam_f = np.where(near[:, None], 0.0, jam_f)
    box = NRV030
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
        "jam_hand_force_n": round(box.rated_nm / (box.ratio * box.eta_dynamic)
                                  / (grip_r / 1000.0), 1),
        "handle_turns_per_pour": round(box.ratio / 2.0, 1),
        "deg_per_handle_turn_mean": round(float(np.ptp(tilt)) / (box.ratio / 2.0), 2),
        "load_case": f"a sealed full {cr.JEROBOAM.name} ({cr.JEROBOAM.mass_full_kg} kg)",
    }
    if out["drive"]["handle_force_n"] > WARN_HANDLE_N:
        out.setdefault("warnings", []).append(
            f"the handle needs {out['drive']['handle_force_n']} N at its grip at worst")
    # each joint's peak force, per side (SHARE on the cradle's shared pins)
    rider = HandCrank.rider(design.ctx)
    rows = []
    for jk, (j, b1, b2) in enumerate(joints):
        on_drive = j in lk.crank or rider in (b1, b2)
        share = 1.0 if on_drive else SHARE
        rows.append({"joint": j, "bodies": [b1, b2], "drive_side_only": on_drive,
                     "bottle_n": round(float(forces[:, jk].max()) * share, 1),
                     "jam_n": round(float(jam_f[:, jk].max()) * share, 1)})
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
    # the crankpin: a shoulder screw held in the arm by its thread, the rod's force at the
    # rod's mid-plane over the arm's top (a cantilever): its thread's root bends
    note = mech.meta.get("hand_crank") or {}
    arm_top = (note.get("arm_z_mm") or [0.0, 0.0])[1]
    rod_mid = design.plan.z(design.plan.layers[rider])[0] + design.ctx.sheet_t(
        "link", rider) / 2
    lever = rod_mid - arm_top
    pitch = {6.0: 1.0, 8.0: 1.25}[CRANKPIN.thread]
    d3 = CRANKPIN.thread - 1.2269 * pitch        # the thread's minor diameter (ISO 724)
    cap = math.pi * d3 ** 3 / 32 * CRANKPIN_YIELD / 1000.0       # N·m at yield
    out["crankpin"] = {"lever_mm": round(lever, 2), "capacity_nm": round(cap, 2),
                       "bottle_nm": round(f_rod * lever / 1000.0, 2),
                       "jam_nm": round(f_rod_jam * lever / 1000.0, 2),
                       "sf_bottle": round(cap * 1000.0 / max(f_rod * lever, 1e-9), 2),
                       "sf_jam": round(cap * 1000.0 / max(f_rod_jam * lever, 1e-9), 2)}
    out["stand"] = stability(config, mech, design, ts)
    out["strength"] = strength_check(config, mech, rows, peak)
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


def stability(config: BuildConfig, mech, design, ts) -> dict:
    """The whole cradle's centre of mass over the swing, each bottle full and empty, against
    the base plate's feet (their pads' outer edges): its margin (mm) to the nearest tipping
    edge, front, rear and sideways (the gearbox and the hand wheel weigh on the drive side),
    the tilt it tips at and the push at the bottle's highest point that tips it."""
    lk = config.lk
    stand = cr.stand_of(lk)
    by = {b.name: b for b in mech.bodies}
    link = cr.cradle_link(lk)
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    lip0 = pts[o][0]
    d0 = pts[ax][0] - pts[o][0]
    t0 = math.atan2(d0[1], d0[0])
    table = float(lip0[1]) - cr.LIP_HEIGHT
    # every part's mass and centre: what stands, what turns with the box (about the mouth)
    still_m, still_c = 0.0, np.zeros(3)
    box_m, box_c = 0.0, np.zeros(3)
    for b in mech.bodies:
        if b.part is None or b.fab == "display":
            continue
        m = part_mass_kg(config, b)
        c = np.asarray(mass_mod.part_props(b.placed_part()).com, float)
        if body_class(_host(b, by)) == link:
            box_m += m
            box_c += m * c
        else:
            still_m += m
            still_c += m * c
    box_rel = (box_c / box_m)[:2] - lip0 if box_m else np.zeros(2)
    box_z = float(box_c[2] / box_m) if box_m else 0.0
    inset, pad = 25.0, 12.0                    # the feet (construction.cradle._base_plate)
    x_front = float(lip0[0]) + stand.base[0] + inset - pad
    x_rear = float(lip0[0]) + stand.base[1] - inset + pad
    z_edge = cr.BASE_HALF - inset + pad
    rows = []
    for size in cr.SIZES:
        y0, slope, x0 = cr.seat(size)
        com_cf = np.array([x0 + size.com_from_mouth * math.cos(slope),
                           y0 + size.com_from_mouth * math.sin(slope)])
        top_cf = np.array([x0 + size.length * math.cos(slope),
                           y0 + size.length * math.sin(slope) + size.r])
        wine = 3.0 if size is cr.JEROBOAM else 1.5
        for state, mb in (("full", size.mass_full_kg), ("empty", size.mass_full_kg - wine)):
            worst = (math.inf, "", 0.0, 0.0, 0.0)
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
                    if margin < worst[0]:
                        worst = (margin, edge, h, float(top - table), m)
            margin, edge, h, top_h, m = worst
            rows.append({"bottle": size.name, "state": state, "kg": round(mb, 2),
                         "margin_mm": round(margin, 1), "edge": edge,
                         "tip_deg": round(math.degrees(math.atan2(margin, h)), 1),
                         "push_at_top_n": round(m * G * margin / max(top_h, 1.0), 1)})
    return {"feet": {"x_mm": [round(x_front, 1), round(x_rear, 1)], "z_mm": round(z_edge, 1)},
            "device_kg": round(still_m + box_m, 3),
            "cases": rows, "worst_margin_mm": min(r["margin_mm"] for r in rows),
            "worst_tip_deg": min(r["tip_deg"] for r in rows)}


def wheel_sweep(mech) -> list[dict]:
    """What the hand crank's turning would hit: its tube about the input shaft and the disc
    its arm and grip sweep there, against every part but the crank's own and the gearbox it
    rides (the crank is drawn at rest)."""
    from build123d import Cylinder, Location

    w = (mech.meta.get("hand_crank") or {}).get("handle")
    if not w:
        return []
    end, d = np.asarray(w["end_xy"], float), np.asarray(w["dir"], float)
    z = float(w["axis_z"]) - float(mech.meta.get("mid_plane", 0.0))
    a = math.degrees(math.atan2(d[1], d[0]))
    reach, t_arm = float(w["reach_mm"]), float(w["arm_t_mm"])

    def cyl(r: float, t0: float, t1: float):
        mid = end + d * (t0 + t1) / 2
        return Cylinder(r, t1 - t0, rotation=(0.0, 90.0, a)).moved(
            Location((float(mid[0]), float(mid[1]), z)))

    sweep = cyl(float(w["hub_r_mm"]), 0.0, reach - t_arm) + cyl(
        float(w["r_mm"]), reach - t_arm, reach + float(w["grip_len_mm"]))
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


def strength_check(config: BuildConfig, mech, rows: list[dict], crank_nm: float) -> dict:
    """The pins and link plates rated by :func:`spiderpig.strength.check` at the cradle's
    loads: "walk" a sealed full 3 L bottle's, "jam" the gearbox's rated torque on a blocked
    cradle (per side, as :func:`report` shares them)."""
    from spiderpig import strength

    lk = config.lk
    ground = set(lk.frame)
    joints = []
    for r in rows:
        bodies = [b for b in r["bodies"] if b != "ground"]
        joints.append({"stem": r["joint"], "crank": r["joint"] in lk.crank,
                       "frame": r["joint"] in ground, "links": bodies,
                       "walk": {"n": r["bottle_n"], "patterns": None},
                       "jam": {"n": r["jam_n"], "patterns": None}})
    loads = {"source": "sim", "note": "a cradle's: a sealed full 3 L bottle (walk), the "
             "gearbox's rated torque on a blocked cradle (jam); cradle_report",
             "walk_n": max((j["walk"]["n"] for j in joints), default=0.0),
             "jam_n": max((j["jam"]["n"] for j in joints), default=0.0), "joints": joints,
             "torque_limit_nm": NRV030.rated_nm, "walk_torque_nm": crank_nm,
             "joint_moment_factor": 1.0}
    st = strength.check(mech.meta.get("wobble") or {}, mech.meta, config, loads)
    keep = ("joint", "kind", "links", "span_mm", "section", "sheet", "thickness_mm")
    return {"rows": [{**{k: r[k] for k in keep if k in r},
                      "walk_sf": (r.get("walk") or {}).get("safety"),
                      "jam_sf": (r.get("jam") or {}).get("safety"),
                      "walk_n": (r.get("walk") or {}).get("load_n"),
                      "jam_n": (r.get("jam") or {}).get("load_n")} for r in st["rows"]],
            "worst": st["worst"], "findings": st["findings"]}
