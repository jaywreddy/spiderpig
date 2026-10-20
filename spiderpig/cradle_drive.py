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
* **The clutch** (:func:`drive_limits`): set on the bench to slip at
  ``handcrank.CLUTCH_SET`` x the start the user measures, checked within
  ``handcrank.CLUTCH_CHECK``; its cap the check's top through the worm at the hand's fastest
  (``handcrank.HAND_RPM``: the crank torque no push at the grip exceeds,
  ``config.PRODUCT_CLUTCH_NM``, the jam's torque ``cradle_report.jam_nm``); its drift
  unchecked over the greased friction and the seats' wear, and dry
  (``handcrank.clutch_drift``), with the cap it would give.
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
        HAND_RPM,
        KEYED_PLATES,
        LIGHT_LOAD_N,
        PLUNGER,
        SEAT_WEAR,
        clutch,
        clutch_drift,
    )
    from spiderpig.cradle_report import JAM_HAND_N, push_nm

    grip_r = spec.throw
    got = clutch(start_nm, grip_r, box)
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
    # speed up, its start down): the most it then lets through
    set_new = clutch(start_new, grip_r, box)["hi_nm"] * box.ratio * eta_hand
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
            "seat_wear_mm": SEAT_WEAR,
            "plunger_n": got["plunger_n"], "plunger_range_n": list(PLUNGER["force"]),
        },
        "keyway": {"plates": KEYED_PLATES, "one_plate_nm": round(one, 1),
                   "plates_nm": round(lot, 1),
                   "one_plate_sf_walk": round(one / max(peak, 1e-9), 2),
                   "one_plate_sf_cap": round(one / cap, 2),
                   "plates_sf_cap": round(lot / cap, 2)},
        "end_screw": {"nm": END_SCREW_NM, "grade": "A2-70 M10",
                      "grade_max_nm": 30.0, "basis": "TR Fastenings, A2-70 at friction 0.1"},
    }


def _offsets(ts, tilt) -> tuple[np.ndarray, np.ndarray]:
    """Each crank angle's distance (degrees) from the rest toggle and from the full one."""
    deg = np.degrees(np.asarray(ts, float))
    rest, full = (np.abs((deg - deg[e] + 180.0) % 360.0 - 180.0)
                  for e in (int(np.argmin(tilt)), int(np.argmax(tilt))))
    return rest, full


def clutch_case(config, mech, ts, tilt, cap: float, rod: dict, crankpin: dict, jam_nm: float,
                output_allowed_n: float | None = None) -> dict:
    """The clutch's cap ``cap`` (N·m) on a blocked cradle (``mech`` fabricated; ``ts`` the
    crank angles, ``tilt`` the bottle's there, degrees) over the whole turn: for each
    excluded band round the toggles (:data:`BANDS`, crank degrees; 0: none) every part rated
    outside it at the cap by the report's own model (``cradle_report.structure`` with that
    ``skip``, its joint rows, :func:`spiderpig.strength.check`, the crank's row at ``cap``),
    the rod's buckling (``rod``), the crankpin's bending (``crankpin``) and the gearbox's
    output shaft's radial load (``output_allowed_n``, at the rod's plane) from the rod's
    force; each part's least safety factor and the worst. Per part, the least band it holds
    :data:`JAM_FLOOR` and 1 outside; the band outside which every part holds 1.3, in crank
    degrees, tilt from each end and handle turns; the rod's force round each toggle at 1°
    (``rod_by_deg``). ``jam_nm``: the 40 N push's torque, for comparison."""
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
