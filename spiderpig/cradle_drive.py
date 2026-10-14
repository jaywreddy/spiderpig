"""A decanting cradle's drive limits (:mod:`construction.handcrank`), for
:func:`cradle_report.report`: the hand crank's slip clutch against the worst turn, what the
gearbox and the keyed plates see at the jams, and the jam at the clutch's cap rated over the
whole turn, the crank-rocker's toggles included (design review round 1, 2026-10-14).

* **The clutch** (:func:`drive_limits`): set on the bench to slip at
  ``handcrank.CLUTCH_SLIP_N`` at the grip, within ``handcrank.CLUTCH_SCATTER`` either way
  after; its least slip must clear the worst starting turn (the worm's static efficiency)
  by ``handcrank.CLUTCH_MARGIN``, and its most, through the worm at a hand's speed, is the
  crank torque no push at the grip exceeds (its cap: ``config.PRODUCT_CLUTCH_NM``).
* **The capped jam** (:func:`clutch_case`): the cap on a blocked cradle (held by a couple,
  as the jam case) over the whole turn, rated by the report's own model
  (``cradle_report.structure``, its joint rows and :func:`spiderpig.strength.check`) outside
  a band round the toggles narrowing to nothing. Round each toggle the rod's force goes as
  the cap over the sine of the crank's angle to the rod: no torque bounds it there, so the
  case reports each part's least safety factor outside each band, and the band each holds
  1.3 and 1 outside.
"""

from __future__ import annotations

import numpy as np

JAM_FLOOR = 1.3         # the jam safety factor every part keeps (tests/test_cradle.py)
BANDS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0)
"""The excluded bands round each toggle the capped case is rated outside (crank degrees)."""


def drive_limits(box, grip_r: float, peak: float, start_nm: float) -> dict:
    """The drive's rows at the jams: ``box`` the gearbox, ``grip_r`` the grip's radius (mm),
    ``peak`` the bottle's largest crank torque (N·m) and ``start_nm`` the input torque that
    starts it (the static efficiency). The jam's push (``cradle_report.jam_nm``) and the
    clutch's cap against the gearbox's rating; the clutch's slip (input N·m and at the grip)
    against the worst start; the keyed plates' keyway, one plate bearing alone (before the
    four are filed together on the key) and the four."""
    from spiderpig.construction.handcrank import (
        CLUTCH_MARGIN,
        CLUTCH_SLIP_N,
        HAND_CRANK,
        clutch,
    )
    from spiderpig.cradle_report import JAM_HAND_N, jam_nm

    got = clutch(grip_r, box.ratio, box.eta_hand)
    jam = jam_nm(box, grip_r)
    cap = got["crank_cap_nm"]
    one = HAND_CRANK.keyway_one()
    four = HAND_CRANK.capacity()["arm keyway"]
    return {
        "gearbox_jam_x_rating": round(jam / box.rated_nm, 2),
        "clutch": {
            "slip_grip_n": CLUTCH_SLIP_N,
            "slip_input_nm": [got["slip_lo_nm"], got["slip_nm"], got["slip_hi_nm"]],
            "slip_grip_range_n": [round(got["slip_lo_nm"] / grip_r * 1000.0, 1),
                                  round(got["slip_hi_nm"] / grip_r * 1000.0, 1)],
            "start_input_nm": round(start_nm, 3),
            "margin": round(got["slip_lo_nm"] / max(start_nm, 1e-9), 2),
            "margin_floor": CLUTCH_MARGIN,
            "crank_cap_nm": cap,
            "cap_x_peak": round(cap / max(peak, 1e-9), 2),
            "cap_x_jam_push": round(cap / jam, 2),
            "jam_push_n": JAM_HAND_N,
            "gearbox_cap_x_rating": round(cap / box.rated_nm, 2),
            "spring_n": [got["spring_set_n"], got["spring_n"]],      # seated, slipping
        },
        "keyway": {"one_plate_nm": round(one, 1), "four_plates_nm": round(four, 1),
                   "one_plate_sf_walk": round(one / max(peak, 1e-9), 2),
                   "one_plate_sf_cap": round(one / cap, 2),
                   "four_plates_sf_cap": round(four / cap, 2)},
    }


def _offsets(ts, tilt) -> tuple[np.ndarray, np.ndarray]:
    """Each crank angle's distance (degrees) from the rest toggle and from the full one."""
    deg = np.degrees(np.asarray(ts, float))
    rest, full = (np.abs((deg - deg[e] + 180.0) % 360.0 - 180.0)
                  for e in (int(np.argmin(tilt)), int(np.argmax(tilt))))
    return rest, full


def clutch_case(config, mech, ts, tilt, cap: float, rod: dict, crankpin: dict, jam_nm: float
                ) -> dict:
    """The clutch's cap ``cap`` (N·m) on a blocked cradle (``mech`` fabricated; ``ts`` the
    crank angles, ``tilt`` the bottle's there, degrees) over the whole turn: for each
    excluded band round the toggles (:data:`BANDS`, crank degrees; 0: none) every part rated
    outside it at the cap by the report's own model (``cradle_report.structure`` with that
    ``skip``, its joint rows, :func:`spiderpig.strength.check`, the crank's row at ``cap``),
    the rod's buckling (``rod``) and the crankpin's bending (``crankpin``) from the rod's force;
    each part's least safety factor and the worst. Per part, the least band it holds
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
    pin_sf = crankpin["capacity_nm"] * 1000.0 / (f_rod * crankpin["lever_mm"])

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
        least["crankpin"] = round(float(pin_sf[keep].min()), 2)
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
            "basis": "the clutch's most through the worm at a hand's speed, on a blocked cradle "
                     "(a couple on the cradle link), every part rated outside each band round "
                     "the toggles by the report's own structure and strength model",
            "by_band": by_band, "parts": parts,
            "holds_1p3_outside_deg": holds, "holds_1p3_tilt_deg": tilt_in,
            "holds_1p3_turns": round(holds * 30.0 / 360.0, 2) if holds is not None else None,
            "rod_buckles_within_deg": [band(rod_sf, 1.0, rest), band(rod_sf, 1.0, full)],
            "rod_under_1p3_within_deg": [band(rod_sf, JAM_FLOOR, rest),
                                         band(rod_sf, JAM_FLOOR, full)],
            "away": by_band[-1]["worst"]}


def verdict(key: str, rep: dict) -> tuple[str, str]:
    """A cradle's standing and why (``cradle_report.VERDICT``), with this report's own
    numbers: the jam at the clutch's cap away from the toggles, the gearbox's walking
    safety factor, the stand's tip."""
    from spiderpig.cradle_report import VERDICT

    kind, why = VERDICT.get(key, ("study", ""))
    capped = rep.get("jam_capped") or {}
    drive = rep.get("drive") or {}
    stand = rep.get("stand") or {}
    bits = []
    if capped:
        sf, part = capped["away"]
        bits.append(f"at the clutch's {capped['crank_nm']:g} N·m cap every part holds jam SF "
                    f"{sf:g} or more 30° and more from the toggles ({part} least)")
    if drive:
        bits.append(f"the gearbox {drive['gearbox_sf']:g} x its rating walking")
    if stand:
        bits.append(f"the stand tips at {stand['worst_tip_deg']:g}°")
    return kind, (why + (" This report: " + "; ".join(bits) + "." if bits else ""))
