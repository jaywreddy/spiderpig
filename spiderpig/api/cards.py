"""The linkages' cards: :func:`list_linkages`, :func:`describe`, sensitivities, foot paths."""


from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np

from spiderpig import linkage
from spiderpig.api import walking  # (module_stride: through the module, where a patch goes)
from spiderpig.api.walking import WALKS_MM
from spiderpig.design import (
    jsonable,
    source_version,
)
from spiderpig.spec import (
    nearest,
)
from spiderpig.store import PROJECT, Store, _read_json, _write_json

# ---------------------------------------------------------------------------
# Linkage cards
# ---------------------------------------------------------------------------


def list_linkages(kind: str | None = None) -> list[dict]:
    """Every registered linkage (``kind``: ``walker`` / ``mechanism`` keeps those): key,
    name, family, kind, its modules (legs per side) and parameters."""
    out = []
    for key in linkage.available(kind):
        lk = linkage.get(key)
        out.append({"key": key, "name": lk.name, "family": lk.family or key, "kind": lk.kind,
                    "modules": {m: len(legs) for m, legs in lk.leg_modules.items()},
                    "params": {k: float(v) for k, v in lk.params.items()},
                    "feet_per_leg": len(lk.feet),
                    "output": lk.output.motion if lk.output else None})
    return out


def _linkage(key: str) -> linkage.Linkage:
    try:
        return linkage.get(key)
    except KeyError:
        near = nearest(key, linkage.available())
        raise KeyError(f"unknown linkage {key!r}" + (f"; did you mean {near!r}?" if near
                                                     else "")) from None


# -- per-code caches in the store (the linkage cards, the guide's tables) ----------------
# ``<store>/cache/<source version>/<name>.json``: a document computed from the code alone,
# kept per :func:`spiderpig.design.source_version`, so any edit of the checkout starts it
# afresh; path components never start with a dot (no ``..``).
_CACHE_NAME = re.compile(r"[A-Za-z0-9_+-][A-Za-z0-9_.+-]*(/[A-Za-z0-9_+-][A-Za-z0-9_.+-]*)*")


def _cache_path(st: Store, name: str) -> Path:
    version = source_version()
    if not _CACHE_NAME.fullmatch(name) or not _CACHE_NAME.fullmatch(version):
        raise ValueError(f"not a cache name: {name!r} / {version!r}")
    return st.root / "cache" / version / f"{name}.json"


def _read_cache(st: Store | None, name: str):
    return None if st is None else _read_json(_cache_path(st, name))


def _write_cache(st: Store | None, name: str, doc) -> None:
    if st is not None:
        _write_json(_cache_path(st, name), doc)


def describe(key: str, store: Store | str | Path | None = PROJECT) -> dict:
    """One linkage's card: its parameters (default, angle or length, which only scale it),
    links and labels, feet or output, modules with their default phases, the closures at
    the defaults (margins, transmission angles, toggles) and one foot's path numbers (a
    walker) or the output check (a mechanism); JSON values throughout.

    A card is a function of the code alone (the walk model over every module is the slow
    part: seconds for a four-legged linkage with many feet), so with a store it is kept
    there per :func:`spiderpig.design.source_version` (``cache/<version>/cards/<key>.json``)
    and read back in every later session on the same code; ``store=None`` computes it."""
    lk = _linkage(key)
    st = Store.of(store)
    name = f"cards/{lk.key}"
    if (doc := _read_cache(st, name)) is not None:
        return doc
    card = jsonable(_card(lk))
    _write_cache(st, name, card)
    return card


def scale_params_table(store: Store | str | Path | None = PROJECT) -> dict[str, list[str]]:
    """Every linkage's scale parameters (:func:`linkage.scale_params`: the ones that only
    resize it), by key; kept in the store per :func:`spiderpig.design.source_version` like
    the cards, since finding them compiles every linkage's program (seconds per session)."""
    st = Store.of(store)
    doc = _read_cache(st, "scale_params")
    if doc is not None and set(doc) == set(linkage.available()):
        return {k: list(v) for k, v in doc.items()}
    table = {key: list(linkage.scale_params(linkage.get(key))) for key in linkage.available()}
    _write_cache(st, "scale_params", table)
    return table


def _card(lk: linkage.Linkage) -> dict:
    key = lk.key
    scale = linkage.scale_params(lk)
    card = {
        "key": key, "name": lk.name, "family": lk.family or key, "kind": lk.kind,
        "source": lk.source, "notes": lk.notes,
        "params": [{"name": k, "default": float(v), "angle": k in lk.angles,
                    "signed": k in lk.signed, "scale": k in scale}
                   for k, v in lk.params.items()],
        "scale_params": list(scale),
        "modules": {m: {"legs": len(legs),
                        "phases_deg": [round(math.degrees(ph), 6) for _, ph in legs],
                        "orientations": [o for o, _ in legs]}
                    for m, legs in lk.leg_modules.items()},
        "links": {b: {"joints": list(js), "label": lk.labels.get(b, "")}
                  for b, (js, _) in lk.links.items()},
        "frame": list(lk.frame), "crank": list(lk.crank), "inputs": list(lk.inputs),
        "feet": [list(f) for f in lk.feet],
        "output": jsonable(lk.output) if lk.output else None,
        "closures": [_step_dict(s) for s in lk.check() if s.kind == "closure"],
    }
    if lk.kind == "walker":
        card["foot_path"] = foot_path(lk, {})
        card["sensitivity"] = sensitivity(lk)
        for m, doc in card["modules"].items():
            s = walking.module_stride(key, m)
            doc["stride_mm"] = s
            doc["walks"] = s is not None and s >= WALKS_MM
    else:
        try:
            card["output_check"] = _output_dict(lk.output_check())
        except linkage.AssemblyError as e:
            card["output_check"] = {"error": str(e)}
        else:
            card["sensitivity"] = output_sensitivity(lk)
    return card


OUTPUT_SENSITIVITY_KEYS = ("stroke_mm", "straightness_mm", "extent_x_mm", "extent_y_mm",
                           "on_line_fraction", "rotation_deg", "swing_deg", "dwell_deg")


def _output_numbers(lk: linkage.Linkage, params: dict) -> dict[str, float | None]:
    c = _output_dict(lk.output_check(params or None))
    ex = c.get("extent_mm") or [None, None]
    return {"stroke_mm": c["stroke_mm"], "straightness_mm": c["straightness_mm"],
            "extent_x_mm": ex[0], "extent_y_mm": ex[1], "on_line_fraction": c["on_line_fraction"],
            "rotation_deg": c["rotation_deg"], "swing_deg": c["swing_deg"],
            "dwell_deg": c["dwell_deg"]}


def output_sensitivity(lk: linkage.Linkage) -> dict:
    """A mechanism's answer to the walkers' :func:`sensitivity`: what +10 % of each length
    (+5° of an angle) does to the output's numbers at the defaults, in percent (the
    stroke, the straightness band, the path's extent, and the rotation, swing or dwell it
    measures), so a designer knows a stroke that scales with ``unit`` from one that
    depends on a proportion. ``null`` where the loops no longer close."""
    at_defaults = _output_numbers(lk, {})
    base = {k: v for k in OUTPUT_SENSITIVITY_KEYS if (v := at_defaults.get(k)) is not None}
    keys = list(base)
    out = {}
    for name, default in lk.params.items():
        angle = name in lk.angles
        value = (float(default) + SENSITIVITY_DEG if angle
                 else float(default) * (1 + SENSITIVITY_STEP))
        try:
            with np.errstate(invalid="ignore", divide="ignore"):
                got = _output_numbers(lk, {name: value})
        except (ValueError, linkage.AssemblyError):
            out[name] = None
            continue
        now = {k: v for k in keys if (v := got.get(k)) is not None and math.isfinite(v)}
        if len(now) != len(keys):
            out[name] = None
            continue
        out[name] = {"step": f"+{SENSITIVITY_DEG:g}°" if angle else f"+{SENSITIVITY_STEP:.0%}",
                     **{k: (round(100.0 * (now[k] - base[k]) / base[k], 1) if base[k] else None)
                        for k in keys}}
    return out


def foot_path(lk: linkage.Linkage, params: dict, n: int = 720) -> dict:
    """One foot's path over a revolution (leg 0's first foot, its defaults unless
    ``params``): ``lift_mm`` (vertical travel), the stance stride and fraction within 2 mm
    of the lowest point, the crank radius, the leg's height and width."""
    ts = 2.0 * math.pi * np.arange(n) / n
    pts = lk.solve(params=params or None).evaluate(ts)
    f = pts[lk.feet[0][1]]
    y0 = float(f[:, 1].min())
    stance = f[:, 1] <= y0 + 2.0
    top = max(float(pts[j][:, 1].max()) for j in lk.points)
    xs = np.concatenate([pts[j][:, 0] for j in lk.points])
    return {
        "lift_mm": float(np.ptp(f[:, 1])),
        "stance_stride_mm": float(np.ptp(f[stance, 0])) if stance.any() else 0.0,
        "stance_fraction": float(stance.mean()),
        "crank_radius_mm": float(np.linalg.norm(pts[lk.crank[1]][0])),
        "height_mm": top - y0,
        "width_mm": float(np.ptp(xs)),
    }


SENSITIVITY_STEP = 0.10     # a length parameter is moved by +10 %
SENSITIVITY_DEG = 5.0       # an angle by +5°


def sensitivity(lk: linkage.Linkage) -> dict:
    """What each parameter does to one foot's path, from its default: the percent change
    of ``lift_mm``, ``stance_stride_mm``, ``height_mm`` and ``width_mm`` for +10 % of a
    length (``+5°`` of an angle), so a designer knows which raise the leg, lengthen the
    stride or grow the envelope before trying. ``null`` where the loops no longer close."""
    base = foot_path(lk, {})
    keys = ("lift_mm", "stance_stride_mm", "height_mm", "width_mm")
    out = {}
    for name, default in lk.params.items():
        angle = name in lk.angles
        value = (float(default) + SENSITIVITY_DEG if angle
                 else float(default) * (1 + SENSITIVITY_STEP))
        try:
            with np.errstate(invalid="ignore", divide="ignore"):   # a loop that can't close
                fp = foot_path(lk, {name: value})
        except (ValueError, linkage.AssemblyError):
            out[name] = None
            continue
        if not all(math.isfinite(fp[k]) for k in keys):
            out[name] = None
            continue
        out[name] = {"step": f"+{SENSITIVITY_DEG:g}°" if angle else f"+{SENSITIVITY_STEP:.0%}",
                     **{k: (round(100.0 * (fp[k] - base[k]) / base[k], 1) if base[k] else None)
                        for k in keys}}
    return out


def _step_dict(s) -> dict:
    return {"point": s.point, "kind": s.kind, "refs": list(s.refs), "radii_mm": s.radii,
            "margin_mm": s.margin_mm, "worst_deg": s.worst_deg, "fails_deg": s.fails_deg,
            "transmission_deg": s.angle_deg, "fail_fraction": s.fail_fraction,
            "toggles": s.toggles, "invalid": s.invalid, "text": s.describe()}


def _output_dict(c) -> dict:
    return {"name": c.output.name, "motion": c.output.motion, "point": c.output.point,
            "extent_mm": list(c.extent_mm), "stroke_mm": c.stroke_mm,
            "straightness_mm": c.straightness_mm, "on_line_fraction": c.on_line,
            "rotation_deg": c.rotation_deg, "swing_deg": c.swing_deg, "dwell_deg": c.dwell_deg,
            "broken": c.broken, "text": c.describe()}
