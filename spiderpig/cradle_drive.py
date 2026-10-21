"""A decanting cradle's drive limits (:mod:`construction.handcrank`), for
:func:`cradle_report.report`: the gearbox against its maximum, the hand crank's slip clutch
against the worst turn and its drift, the worm's efficiency band, the input's and the output's
radial loads, the keyed plates and the end screw, and the jam at the clutch's cap rated over
the whole turn, the crank-rocker's toggles included (design review rounds 1 and 2,
2026-10-14).

* **The gearbox** (``WormBox.rated_nm``: Phidgets' TRM4402_0, "Maximum Strength of Gears 85
  N·m", a maximum: round 2's M-1 withdrew round 1's "rating at a hand's speed" and its 2 x
  momentary limit) against the walking peak and against the clutch's cap
  (:data:`GEARBOX_CAP_FLOOR`).
* **The clutch** (:func:`drive_limits`): set to slip at ``handcrank.CLUTCH_SET`` x the start
  the user measures, never over its ceiling (``handcrank.clutch_ceiling``: design review round
  3's M1), checked within ``handcrank.CLUTCH_CHECK`` on its stub; its cap the check's top
  through the worm at the hand's fastest (``handcrank.HAND_RPM``: the crank torque no push at
  the grip exceeds while the check holds, ``config.PRODUCT_CLUTCH_NM``, the jam's torque
  ``cradle_report.jam_nm``), the same set on a new box, more at a fast spin
  (``handcrank.HAND_FAST_RPM``); its drift unchecked over the greased friction and the seats'
  wear, and dry (``handcrank.clutch_drift``), with the cap it would give; the plungers'
  settings and their reach (``handcrank.plunger_setting``, ``plunger_reach``), the grip's
  couple on the journal (``handcrank.journal_share``).
* **The riser** (:func:`riser_rating`): the gearbox's clamped stack on four A2-70 studs,
  walking and at the cap over the whole turn (design review round 3's M2).
* **The worm's efficiency band** (round 2's M-5): the grip at the worst start run in and new
  (``handcrank.RUNIN_LOSS``); the clutch's margin over the start, set on a measurement, is the
  same at either.
* **The capped jam** (:func:`clutch_case`): the cap on a blocked cradle (held by a couple,
  as the jam case) over the whole turn, rated by the report's own model
  (``cradle_report.structure``, its joint rows and :func:`spiderpig.strength.check`) outside
  a band round the toggles narrowing to nothing, the gearbox's output shaft's radial load
  among the parts (round 2's M-2). Round each toggle the rod's force goes as the cap over the
  sine of the crank's angle to the rod: no torque bounds it there, so the case reports each
  part's least safety factor outside each band, and the band each holds 1.3 and 1 outside.
"""

from __future__ import annotations

import math

import numpy as np

JAM_FLOOR = 1.3         # the jam safety factor every part keeps (tests/test_cradle.py)
GEARBOX_CAP_FLOOR = 1.25
"""The gearbox's maximum over the clutch's cap at least: Phidgets' 85 N·m is a maximum with
no basis given (Motovario's own pattern is rated 84 N·m at every speed to 1750 rpm in its ATEX
catalogue, its momentary peaks to 2 x that), and the cap is already taken at the bench
check's top and the hand's fastest."""
BANDS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0)
"""The excluded bands round each toggle the capped case is rated outside (crank degrees)."""
OUTPUT = "gearbox output (overhung)"
RISER = "gearbox riser"
RISER_MU = 0.15
"""The riser's faces' friction for its slip (steel, 6061, 5052, the gearbox's die-cast face,
bare but perhaps oiled by a weeping seal): RoyMech's greasy mild steel's middle (0.09-0.19);
dry steel on aluminium is 0.47-0.61 (``handcrank.DRY_MU``)."""
SS304_YIELD = 205.0     # MPa: 304 annealed (ASTM A276's minimum): a riser part's whose item
#                         names no yield
E_STEEL, E_ALU_ = 200000.0, 69000.0


def riser_rating(box, notes: dict, f, t_nm) -> dict:
    """The gearbox's riser (``handcrank.riser_for``, design review round 3's M2: four M8
    studs had carried the gearbox in bending over 18.7 mm, rated nowhere) at a rod force ``f``
    (N, at the rod's mid-plane) and a crank torque ``t_nm`` (N·m, the housing's reaction),
    each a number or an array. The rigid housing hangs from the plate on four clamped
    columns (the ring and a sleeve faced to the gearbox's height: no shims since
    simplification round 2) on the 85 mm PCD: each takes a quarter of the
    force and its share of the turn, ``V`` (the worst, the two aligned), the force's moment
    about the columns' mid-height as push-pull pairs, ``N`` (the worst stud's), and the
    columns' end moments, ``m = V H / 2`` (fixed at the face and at the plate: the four keep
    the housing from tilting). The clamp from the thin nut's torque over the nut factor's band
    (``handcrank.RISER_NUT_K``), the stud's share of the pull ``phi`` (its stiffness and the
    stack's). Rated: the stud (A2-70's proof, its tension with the joint closed or, open,
    the column's moment on the sleeve's radius, with the side load's shear if it slips: von
    Mises), the sleeve's bending (open), the ring's bearing under the sleeve; and its slip at
    :data:`RISER_MU` (the friction; the snug sleeve and the stud's shear hold it if it does).
    ``least``: the least of the strength rows."""
    from spiderpig.construction.handcrank import (
        HAND_CRANK,
        RISER_NUT_K,
        RISER_NUT_NM,
        RISER_SLEEVE,
        RISER_STUD,
        STUD_PROOF_MPA,
    )

    f = np.asarray(f, float)
    t_nm = np.asarray(t_nm, float)
    r = box.pcd / 2
    h = float(notes.get("riser_height_mm", 25.2))
    face = float(notes.get("face_z_mm", -h))
    lever = float(notes.get("rod_mid_z_mm", 0.0)) - (face + h / 2)
    d = RISER_STUD[0]
    a_s = 36.6                                      # M8's stress area (ISO 898-1)
    a_minor = math.pi / 4 * (d - 1.22687 * 1.25) ** 2
    from spiderpig.hardware.catalog import get

    sl_od, sl_id, sl_key = RISER_SLEEVE
    sleeve = get(sl_key).dims
    e_sl = float(sleeve.get("e_mpa", 193000.0))
    y_sl = float(sleeve.get("yield_mpa", SS304_YIELD))
    a_sl = math.pi / 4 * (sl_od ** 2 - sl_id ** 2)
    z_sl = math.pi * (sl_od ** 4 - sl_id ** 4) / (32 * sl_od)
    kern = (sl_od ** 2 + sl_id ** 2) / (8 * sl_od)
    t_ring = float(notes.get("riser_ring_mm", 3.175))
    sl_len = float(notes.get("riser_sleeve_mm", h - t_ring))
    t_plate = float(notes.get("plate_inner_z_mm", 4.75)) - (face + h)
    k_b = E_STEEL * a_s / (h + t_plate)
    k_m = 1.0 / (sl_len / (e_sl * a_sl) + t_ring / (E_ALU_ * a_sl)
                 + t_plate / (E_ALU_ * math.pi / 4 * (13.0 ** 2 - (d + 0.6) ** 2)))
    phi = k_b / (k_b + k_m)
    p_lo = RISER_NUT_NM * 1000.0 / (RISER_NUT_K[1] * d)
    p_hi = RISER_NUT_NM * 1000.0 / (RISER_NUT_K[0] * d)
    v = f / 4 + t_nm * 1000.0 / (4 * r)
    n = f * lever / (2 * r)
    m = v * h / 2
    clamp = p_lo - (1 - phi) * n                    # the tension side's least
    opens = m > clamp * kern
    t_stud = np.maximum(p_hi + phi * n, n + m / (sl_od / 2))
    vm = np.sqrt((t_stud / a_s) ** 2 + 3 * (v / a_minor) ** 2)
    sf_stud = STUD_PROOF_MPA / vm
    bearing = (p_hi + (1 - phi) * n) / a_sl + m / z_sl
    # the sleeve: in bending once the joint opens, and crushed by the clamp and the column's
    # moment at its most (the simplification round 1, 2026-10-21: unrated before, 304's
    # annealed 205 MPa took 222 at B's cap)
    sf_sleeve = np.where(opens, y_sl / np.maximum(m / z_sl, 1e-9), np.inf)
    sf_crush = y_sl / bearing
    sf_ring = HAND_CRANK.bearing_mpa / bearing
    least = np.minimum(np.minimum(np.minimum(sf_stud, sf_sleeve), sf_ring), sf_crush)
    slip = RISER_MU * np.maximum(clamp, 0.0) / np.maximum(v, 1e-9)

    def out(x):
        x = np.asarray(x, float)
        return round(float(x), 2) if x.ndim == 0 else x
    return {"side_n": out(v), "axial_n": out(n), "end_moment_nm": out(m / 1000.0),
            "clamp_n": [round(p_lo), round(p_hi)], "phi": round(phi, 3),
            "lever_mm": round(lever, 1), "opens": bool(np.any(opens)),
            "sf_stud": out(sf_stud), "sf_sleeve": out(np.minimum(sf_sleeve, 99.0)),
            "sf_sleeve_crush": out(sf_crush),
            "sf_ring_bearing": out(sf_ring), "least": out(least),
            "slip_sf": out(slip), "slip_mu": RISER_MU}


def drive_limits(box, spec, peak: float, start_nm: float, notes: dict) -> dict:
    """The drive's rows: ``box`` the gearbox, ``spec`` the hand crank's arm
    (``handcrank.HandleSpec``), ``peak`` the bottle's largest crank torque (N·m), ``start_nm``
    the input torque that starts it (the catalogue's static efficiency), ``notes`` the build's
    (``notes["hand_crank"]``: the rod's mid-plane and the hub's end, for the output's radial
    load). The gearbox's maximum against the walking peak and the clutch's cap; the grip at
    the worst start over the efficiency band; the clutch as set, checked, drifted unchecked
    and dry; the grip's force at the clutch's top on the input shaft against the catalogue's
    radial load there; the output shaft's allowed radial load at the rod's plane (the forces
    are the report's); the keyed plates' keyway, one plate bearing alone and the lot; the
    shaft's end screw's torque against its grade's."""
    from spiderpig.construction.handcrank import (
        CLUTCH_CHECK,
        CLUTCH_SET,
        END_SCREW_NM,
        HAND_CRANK,
        HAND_FAST_RPM,
        HAND_RPM,
        KEYED_PLATES,
        LIGHT_LOAD_N,
        PLUNGER,
        SEAT_WEAR,
        clutch,
        clutch_drift,
        journal_share,
        plunger_reach,
        plunger_setting,
    )
    from spiderpig.cradle_report import JAM_HAND_N, push_nm

    grip_r = spec.throw
    # the ceiling (design review round 3's M1): the drawn 6 L's window top, whatever the
    # start the user reads
    ceiling = CLUTCH_SET * (1 + CLUTCH_CHECK) * start_nm
    got = clutch(start_nm, grip_r, box, ceiling)
    cap = got["crank_cap_nm"]
    push = push_nm(box, grip_r)
    eta_hand = got["eta_hand"]
    new = box.eta_new()
    start_new = start_nm * box.eta_static / new
    # the grip's force on the input shaft, its middle this far past the shaft's shoulder
    x_in = box.input_len + spec.reach + spec.grip[1] / 2 + 1.6     # (a washer under it)
    allowed_in = box.input_allowed(x_in)
    at_slip = got["hi_nm"] * 1000.0 / grip_r
    # the output's: the rod's force at its mid-plane, past the hub's end
    x_out = float(notes.get("rod_mid_z_mm", 0.0)) - float(notes.get("hub_end_z_mm", 0.0))
    drift = clutch_drift()
    unchecked = got["set_nm"] * drift["greased_worn"][1] * box.ratio * eta_hand
    # set on a new box's start, not checked again once it has run in (its efficiency at
    # speed up, its start down): the most it then lets through, under the ceiling (and
    # without it, as round 3's review found it), and the margin it leaves the new box
    new_set = clutch(start_new, grip_r, box, ceiling)
    set_new = new_set["hi_nm"] * box.ratio * eta_hand
    set_new_open = clutch(start_new, grip_r, box)["hi_nm"] * box.ratio * eta_hand
    one = HAND_CRANK.keyway_one()
    lot = HAND_CRANK.capacity()["arm keyway"]
    return {
        "gearbox_rating_nm": box.rated_nm,
        "gearbox_rating_basis": "Phidgets' maximum strength of gears (TRM4402_0)",
        "gearbox_cap_sf": round(box.rated_nm / cap, 2),
        "gearbox_cap_floor": GEARBOX_CAP_FLOOR,
        "eta": {"static": box.eta_static, "static_new": new, "hand": eta_hand,
                "hand_rpm": HAND_RPM},
        "grip_light_load_n": LIGHT_LOAD_N, "grip_load": spec.load,
        "grip_mm": list(spec.grip),
        "handle_force_band_n": [round(start_nm * 1000.0 / grip_r, 1),
                                round(start_new * 1000.0 / grip_r, 1)],
        "jam_push_nm": round(push, 2), "jam_hand_force_n": JAM_HAND_N,
        "input_overhung": {"x_mm": round(x_in, 1), "allowed_n": round(allowed_in, 1),
                           "at_slip_n": round(at_slip, 1),
                           "sf": round(allowed_in / max(at_slip, 1e-9), 2)},
        "output_overhung": {"x_mm": round(x_out, 1),
                            "allowed_n": round(box.output_allowed(x_out), 1),
                            "basis": "Fr2 a / (b + x): Motovario's static/limited-operation "
                                     "maximum (Phidgets: 4.8 kN, no place given)"},
        "clutch": {
            "set_factor": CLUTCH_SET, "check": CLUTCH_CHECK,
            "start_input_nm": round(start_nm, 3), "start_input_new_nm": round(start_new, 3),
            "set_input_nm": got["set_nm"], "set_grip_n": got["set_grip_n"],
            "checked_input_nm": [got["lo_nm"], got["hi_nm"]],
            "margin": round(got["lo_nm"] / max(start_nm, 1e-9), 2),
            "margin_floor": 1.1,
            "crank_cap_nm": cap, "cap_x_peak": round(cap / max(peak, 1e-9), 2),
            "cap_x_jam_push": round(cap / push, 2),
            "drift_unchecked": drift,
            "unchecked_cap_nm": round(unchecked, 1),
            "unchecked_gearbox_sf": round(box.rated_nm / unchecked, 2),
            "unchecked_margin": round(CLUTCH_SET * drift["greased_worn"][0], 2),
            "set_new_cap_nm": round(set_new, 1),
            "set_new_gearbox_sf": round(box.rated_nm / set_new, 2),
            "set_new_no_ceiling_cap_nm": round(set_new_open, 1),
            "new_box_margin": new_set["margin"],
            "ceiling_input_nm": round(ceiling, 3), "ceiling_grip_n": got["hi_grip_n"],
            "fast_rpm": HAND_FAST_RPM, "fast_cap_nm": got["fast_cap_nm"],
            "fast_gearbox_sf": round(box.rated_nm / got["fast_cap_nm"], 2),
            "journal": journal_share(spec),
            "seat_wear_mm": SEAT_WEAR,
            "plunger_n": got["plunger_n"], "plunger_range_n": list(PLUNGER["force"]),
            "plunger_setting": plunger_setting(),
            "plunger_reach": plunger_reach(got["set_nm"]),
            "checked": "on its printed stub in the riser locator's pocket: the last build "
                       "step, every set-up, after greasing, after a hot car, and every month or "
                       "ten pours standing set up",
        },
        "keyway": {"plates": KEYED_PLATES, "one_plate_nm": round(one, 1),
                   "plates_nm": round(lot, 1),
                   "one_plate_sf_walk": round(one / max(peak, 1e-9), 2),
                   "one_plate_sf_cap": round(one / cap, 2),
                   "plates_sf_cap": round(lot / cap, 2)},
        "end_screw": {"nm": END_SCREW_NM, "grade": "A2-70 M10 ISO 7380",
                      "grade_max_nm": 15.0,
                      "basis": "TR Fastenings' maximum for ISO 7380 in A2-70 (its socket "
                               "screws' table)"},
    }


def _offsets(ts, tilt) -> tuple[np.ndarray, np.ndarray]:
    """Each crank angle's distance (degrees) from the rest toggle and from the full one."""
    deg = np.degrees(np.asarray(ts, float))
    rest, full = (np.abs((deg - deg[e] + 180.0) % 360.0 - 180.0)
                  for e in (int(np.argmin(tilt)), int(np.argmax(tilt))))
    return rest, full


def clutch_case(config, mech, ts, tilt, cap: float, rod: dict, crankpin: dict, jam_nm: float,
                output_allowed_n: float | None = None, riser: tuple | None = None) -> dict:
    """The clutch's cap ``cap`` (N·m) on a blocked cradle (``mech`` fabricated; ``ts`` the
    crank angles, ``tilt`` the bottle's there, degrees) over the whole turn: for each
    excluded band round the toggles (:data:`BANDS`, crank degrees; 0: none) every part rated
    outside it at the cap by the report's own model (``cradle_report.structure`` with that
    ``skip``, its joint rows, :func:`spiderpig.strength.check`, the crank's row at ``cap``),
    the rod's buckling (``rod``), the crankpin's bending (``crankpin``) and the gearbox's
    output shaft's radial load (``output_allowed_n``, at the rod's plane) and the gearbox's
    riser (``riser``: the box and the build's notes, :func:`riser_rating` at the rod's force and
    the cap) from the rod's force; each part's least safety factor and the worst. Per part,
    the least band it holds :data:`JAM_FLOOR` and 1 outside; the band outside which every
    part holds 1.3, in crank degrees, tilt from each end and handle turns; the rod's force
    round each toggle at 1° (``rod_by_deg``). ``jam_nm``: the 40 N push's torque, for
    comparison."""
    from spiderpig import strength
    from spiderpig.construction import cradle as cr
    from spiderpig.cradle_report import (
        joint_rows,
        loads_doc,
        planar_model,
        statics,
        structure,
    )

    lk = config.lk
    ts = np.asarray(ts, float)
    rest, full = _offsets(ts, tilt)
    off = np.minimum(rest, full)
    rigid, joints = planar_model(config, mech, 1.0, cr.DESIGN)
    torque, forces = statics(config, rigid, joints, ts)
    _, cap_f = statics(config, rigid, joints, ts, jam_nm=cap)
    peak = float(np.abs(torque).max())
    rider = next(b for b, (js, _) in lk.links.items() if lk.crank[1] in js)
    rod_jk = next(jk for jk, (j, *_) in enumerate(joints) if j == lk.crank[1])
    f_rod = np.maximum(cap_f[:, rod_jk], 1e-9)
    rod_sf = rod["euler_n"] / f_rod
    from spiderpig.cradle_report import rod_secant_sf

    secant_sf = rod_secant_sf(f_rod, rod) if "eccentricity_mm" in rod else rod_sf
    pin_sf = crankpin["capacity_nm"] * 1000.0 / (f_rod * crankpin["lever_mm"])
    out_sf = None if output_allowed_n is None else output_allowed_n / f_rod
    riser_sf = None if riser is None else np.asarray(
        riser_rating(riser[0], riser[1], f_rod, np.full_like(f_rod, cap))["least"], float)

    def outside(b: float) -> dict:
        skip = off < b - 1e-9
        jam_f = np.where(skip[:, None], 0.0, cap_f)
        rows = joint_rows(config, rider, joints, forces, jam_f)
        struct = structure(config, mech, crank_nm=cap, ts=ts, skip=skip)
        doc = loads_doc(config, rows, peak, struct)
        doc["torque_limit_nm"] = cap
        st = strength.check(mech.meta.get("wobble") or {}, mech.meta, config, doc)
        least = {r["joint"]: round(float((r.get("jam") or {})["safety"]), 2)
                 for r in st["rows"] if (r.get("jam") or {}).get("safety") is not None}
        keep = ~skip
        least["rod (buckling)"] = round(float(rod_sf[keep].min()), 2)
        least["rod (beam-column)"] = round(float(secant_sf[keep].min()), 2)
        least["crankpin"] = round(float(pin_sf[keep].min()), 2)
        if out_sf is not None:
            least[OUTPUT] = round(float(out_sf[keep].min()), 2)
        if riser_sf is not None:
            least[RISER] = round(float(riser_sf[keep].min()), 2)
        worst = min(least, key=least.__getitem__)
        return {"band_deg": b, "worst": [least[worst], worst], "parts": least,
                "rod_n": round(float(f_rod[keep].max()), 1)}

    by_band = [outside(b) for b in BANDS]
    names = by_band[-1]["parts"]
    parts = {n: {"holds_1p3_outside_deg": next((r["band_deg"] for r in by_band
                                                if r["parts"].get(n, 0.0) >= JAM_FLOOR), None),
                 "holds_1_outside_deg": next((r["band_deg"] for r in by_band
                                              if r["parts"].get(n, 0.0) >= 1.0), None)}
             for n in names}
    holds = next((r["band_deg"] for r in by_band if r["worst"][0] >= JAM_FLOOR), None)
    ends = (float(np.min(tilt)), float(np.max(tilt)))
    tilt_in = ([round(float(np.max(np.abs(tilt[side <= holds] - end))), 2)
                for side, end in ((rest, ends[0]), (full, ends[1]))]
               if holds else [0.0, 0.0])

    def band(v: np.ndarray, floor: float, side: np.ndarray) -> float:
        """How far from a toggle (crank degrees) ``v`` falls under ``floor`` nearest it."""
        near = (side < 90.0) & (v < floor)
        return round(float(side[near].max()), 1) if near.any() else 0.0

    return {"crank_nm": cap, "jam_push_nm": jam_nm,
            "basis": "the clutch's most through the worm at the hand's fastest, on a blocked "
                     "cradle (a couple on the cradle link), every part rated outside each "
                     "band round the toggles by the report's own structure and strength model",
            "by_band": by_band, "parts": parts,
            "holds_1p3_outside_deg": holds, "holds_1p3_tilt_deg": tilt_in,
            "holds_1p3_turns": round(holds * 30.0 / 360.0, 2) if holds is not None else None,
            "rod_buckles_within_deg": [band(rod_sf, 1.0, rest), band(rod_sf, 1.0, full)],
            "rod_under_1p3_within_deg": [band(rod_sf, JAM_FLOOR, rest),
                                         band(rod_sf, JAM_FLOOR, full)],
            "away": by_band[-1]["worst"]}


def verdict(key: str, rep: dict) -> tuple[str, str]:
    """A cradle's standing and why (``cradle_report.VERDICT``), with this report's own
    numbers: the jam at the clutch's cap away from the toggles, the gearbox against its
    maximum walking and at the cap, the grip's force over the efficiency band, the stand's
    tip."""
    from spiderpig.cradle_report import VERDICT

    kind, why = VERDICT.get(key, ("study", ""))
    capped = rep.get("jam_capped") or {}
    drive = rep.get("drive") or {}
    stand = rep.get("stand") or {}
    bits = []
    if capped:
        from spiderpig.cradle_report import OPEN_AT_CAP

        open_ = OPEN_AT_CAP.get(key, {})
        parts = (capped.get("by_band") or [{}])[-1].get("parts") or {}
        held = {k: v for k, v in parts.items() if k not in open_}
        if held:
            part = min(held, key=held.__getitem__)
            bits.append(f"at the clutch's {capped['crank_nm']:g} N·m cap (checked at set-up) "
                        f"every {'other ' if open_ else ''}part holds jam SF {held[part]:g} or "
                        f"more 30° and more from the toggles ({part} least)")
        under = sorted((v, k) for k, v in parts.items() if k in open_ and v < JAM_FLOOR)
        if under:
            bits.append("open: " + ", ".join(f"{k} {v:g}" for v, k in under) + " at the cap "
                        "(cradle_report.OPEN_AT_CAP)")
    if drive:
        bits.append(f"the gearbox {drive['gearbox_sf']:g} x the walking peak and "
                    f"{drive['gearbox_cap_sf']:g} x the cap on its "
                    f"{drive['gearbox_rating_nm']:g} N·m maximum")
        lo, hi = drive["handle_force_band_n"]
        bits.append(f"the grip {lo:g} N at worst run in, {hi:g} N new ({drive['grip_load']}-"
                    f"load crank, {drive['grip_radius_mm']:g} mm)")
    if stand:
        bits.append(f"the stand tips at {stand['worst_tip_deg']:g}°")
    return kind, (why + (" This report: " + "; ".join(bits) + "." if bits else ""))
