"""How a decanting cradle comes apart for transport, and how its pieces stack in a box.

The cradle (:mod:`construction.cradle`) comes apart where it is already fastened, with
no new joint:

1. the hand crank off the gearbox's input shaft (its set screw);
2. the cradle off the base plate (the brackets' nuts: the carriage bolts stay in the plate);
3. each side's inner plate off (its pillar's and columns' screws, from the bay side);
4. the rod's pin out of the drive side's cradle link (its Chicago screw: the barrel stays
   in the rod);
5. each side panel off its cradle link (the link's bore slides off the pillar), leaving
   the cradle box (both links, the tie rods, the saddles, the light) in one piece;
6. the base stop off the rear saddle (its two screws).

:func:`flat_pack` sorts the fabricated cradle's parts into those modules
(:data:`MODULES`), lays each one in its flattest orientation and drops it into a box,
largest first, onto what is already in (each part's bounding box: a conservative stack),
the box's floor at the table under the cradle: the viewer's flat-pack view
(``/api/cradle/{key}/pack``, ``viewer/decant.html``) flies each module there. Every pose is
the fabricated cradle's at crank angle 0, the baked clip's first frame.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

import numpy as np

from spiderpig.construction import cradle as cr

CLEAR = 3.0         # between stacked pieces (foam, cardboard)
WALL = 5.0          # the box's corrugated wall
GRID = 15.0         # the drop's candidate positions (mm)


@dataclass(frozen=True)
class Module:
    """A piece that travels whole: ``key``, a ``label``, ``how`` it comes off, and the part
    names (side-prefixed: ``L.``, ``R.``) it takes."""

    key: str
    label: str
    how: str


MODULES = (
    Module("base", "Base plate", "Undo the six M6 nuts on the brackets' feet and lift the "
           "cradle off; the carriage bolts stay in the plate (their nuts back on)."),
    Module("passive", "Passive side panel", "Slides off its cradle link once its inner "
           "plate is off: the link's bore off the pillar."),
    Module("drive", "Drive side panel (gearbox, crank, rod)", "Slides off its cradle link "
           "once its inner plate is off and the rod's pin is out (hold the barrel's head, "
           "unscrew the Chicago screw; it goes back into the barrel for the trip)."),
    Module("cradle", "Cradle box (both links, tie rods, saddles, light)", "What's left: one "
           "rigid piece."),
    Module("inner_L", "Drive side's inner plate", "Its pillar's and columns' screws from the "
                                                   "bay side (they go back in its holes)."),
    Module("inner_R", "Passive side's inner plate", "As the drive side's."),
    Module("base_stop", "Base stop", "Its two screws from the rear saddle."),
    Module("hand_crank", "Hand crank", "Loosen its set screw and pull it off the input "
                                       "shaft."),
    Module("adapters", "Bottle adapters", "Lift them out of the saddles."),
)

DISPLAY = ("bottle", "decanter", "decanter_riser")     # drawn in the model, never shipped


def module_of(name: str, link: str, over: set[str], inner: set[str]) -> str | None:
    """The :data:`MODULES` key a part travels in (``None``: display only, not packed).
    ``link``: the cradle link's class; ``over``: the side parts between the cradle link and
    the inner plate (loose: they come off with the cradle box); ``inner``: the inner plates'
    own screws."""
    if name in DISPLAY:
        return None
    if name in inner or name.startswith("bay_tie"):
        return "inner_R" if name.startswith("R.") else "inner_L"
    side, stem = (name[:1], name[2:]) if name[1:2] == "." else ("", name)
    if stem == "torso":
        return f"inner_{side}"
    if stem.startswith("handle"):
        return "hand_crank"
    if stem.startswith("base_stop"):
        return "base_stop"
    if "magnum_liner" in stem or stem.endswith("_collar"):
        return "adapters"
    if stem in ("base_plate",) or stem.startswith("foot_boot") or re.fullmatch(
            r"foot_bracket_\w+_(screw1|nut1)", stem):
        return "base"
    if re.match(r"foot_bracket_\w+L(_|$)", stem):
        return "drive"
    if re.match(r"foot_bracket_\w+R(_|$)", stem):
        return "passive"
    if stem == link or name in over or stem.startswith(("tie_", "front_saddle", "rear_saddle",
                                                        "led")):
        return "cradle"
    if side == "L":
        return "drive"
    if side == "R":
        return "passive"
    return "cradle"


def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def _boxes(parts) -> np.ndarray:
    """``(n, 8, 3)`` corners of each part's bounding box."""
    out = []
    for p in parts:
        bb = p.bounding_box()
        lo, hi = (bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z)
        out.append([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1], (lo, hi)[(i >> 2) & 1][2]]
                    for i in range(8)])
    return np.asarray(out, float)


def _aabbs(corners: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Each part's bounding box after ``rot``: ``(n, 2, 3)`` (min, max)."""
    r = corners @ rot.T
    return np.stack([r.min(axis=1), r.max(axis=1)], axis=1)


def _drop(boxes: np.ndarray, placed: np.ndarray, floor: float) -> float:
    """The lowest lift (along y) that sets ``boxes`` (``(n, 2, 3)``, at lift 0) on the floor
    or on ``placed`` where they overlap in x and z."""
    lift = floor - boxes[:, 0, 1].min()
    if len(placed):
        a, b = boxes[:, None], placed[None]
        over = ((a[..., 0, 0] < b[..., 1, 0]) & (b[..., 0, 0] < a[..., 1, 0])
                & (a[..., 0, 2] < b[..., 1, 2]) & (b[..., 0, 2] < a[..., 1, 2]))
        if over.any():
            need = (b[..., 1, 1] + CLEAR - a[..., 0, 1])[over]
            lift = max(lift, float(need.max()))
    return lift


def flat_pack(mech, lk) -> dict:
    """The flat pack of the fabricated cradle ``mech`` (crank angle 0): its modules (key,
    label, how, parts, ``matrix``: the 4 x 4 row-major transform from the assembled pose to
    the packed one, in the cradle's own frame: x along the linkage, y up, z across the bay),
    the box (its inside and outside sizes, mm, and its floor's corner), and the steps."""
    link = cr.cradle_link(lk)
    by = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    # the side parts between the cradle link and the inner plate: loose once the plate is off
    over: set[str] = set()
    inner: set[str] = set()
    for s in ("L", "R"):
        lb, tb = by.get(f"{s}.{link}"), by.get(f"{s}.torso")
        if lb is None or tb is None:
            continue
        lz = lb.bounding_box()
        tz = tb.bounding_box()
        bay = 1.0 if tz.center().Z > lz.center().Z else -1.0
        link_face = lz.max.Z if bay > 0 else lz.min.Z
        t_face = tz.max.Z if bay > 0 else -tz.min.Z          # the inner plate's bay face
        for n, b in by.items():
            if not n.startswith(f"{s}.") or n in (f"{s}.{link}", f"{s}.torso"):
                continue
            bb = b.bounding_box()
            lo, hi = (bb.min.Z, bb.max.Z) if bay > 0 else (-bb.max.Z, -bb.min.Z)
            face = link_face if bay > 0 else -link_face
            if hi >= t_face - 0.5:          # a head on the inner plate's bay face: its screw
                inner.add(n)
            elif lo >= face - 1e-6:         # wholly between the link and the plate: loose
                over.add(n)
    groups: dict[str, list[str]] = {}
    for n in by:
        k = module_of(n, link, over, inner)
        if k is not None:
            groups.setdefault(k, []).append(n)
    # the cradle box's tilt at crank angle 0 (the pose ``mech`` is in), from the linkage
    o, ax = cr.mouth_frame(lk)
    pts = lk.solve(1, 0.0).evaluate(np.array([0.0]))
    d = np.asarray(pts[ax], float).reshape(-1, 2)[0] - np.asarray(pts[o], float).reshape(-1, 2)[0]
    tilt0 = math.atan2(float(d[1]), float(d[0]))
    base = by["base_plate"].bounding_box()
    floor = base.min.Y - (cr.FOOT_PAD_H - 2.0)          # the table (the feet's tips)
    x0, x1 = base.min.X - CLEAR, base.max.X + CLEAR
    z0, z1 = base.min.Z - CLEAR, base.max.Z + CLEAR
    placed = np.zeros((0, 2, 3))
    out_mods = []
    order = [m for m in MODULES if m.key in groups]
    for m in order:
        names = groups[m.key]
        corners = _boxes([by[n] for n in names])
        flat = corners.reshape(-1, 3)
        centre = (flat.min(axis=0) + flat.max(axis=0)) / 2
        corners = corners - centre
        if m.key in ("drive", "passive"):             # flat, either face down, either way round
            down = _rot("x", -90 if m.key == "drive" else 90)
            rots = [r @ down for r in (np.eye(3), _rot("x", 180), _rot("y", 180),
                                       _rot("z", 180))]
        elif m.key == "base":
            rots = [np.eye(3)]
        elif m.key == "cradle":
            level = _rot("z", -math.degrees(tilt0))
            rots = [level, _rot("x", 90) @ level]
        else:
            rots = [np.eye(3), _rot("x", 90), _rot("x", -90), _rot("z", 90),
                    _rot("y", 90) @ _rot("x", 90), _rot("z", 90) @ _rot("x", 90)]
        best = None
        for rot in rots:
            boxes = _aabbs(corners, rot)
            lo, hi = boxes[:, 0].min(axis=0), boxes[:, 1].max(axis=0)
            if m.key == "base":                               # centred in the footprint
                cands = [((x0 + x1) / 2 - (lo[0] + hi[0]) / 2, (z0 + z1) / 2 - (lo[2] + hi[2]) / 2)]
            else:
                xs = np.arange(x0 - lo[0], x1 - hi[0] + 1e-6, GRID)
                zs = np.arange(z0 - lo[2], z1 - hi[2] + 1e-6, GRID)
                cands = [(float(x), float(z)) for x in xs for z in zs] or [
                    ((x0 + x1) / 2 - (lo[0] + hi[0]) / 2, (z0 + z1) / 2 - (lo[2] + hi[2]) / 2)]
            for dx, dz in cands:
                shifted = boxes + np.array([dx, 0.0, dz])
                dy = _drop(shifted, placed, floor + CLEAR)
                top = dy + hi[1]
                if best is None or (top, dy) < (best[0], best[1]):
                    best = (top, dy, rot, np.array([dx, dy, dz]))
        assert best is not None
        _, _, rot, t = best
        placed = np.concatenate([placed, _aabbs(corners, rot) + t])
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = t - rot @ centre            # p -> rot (p - centre) + t
        out_mods.append({"key": m.key, "label": m.label, "how": m.how, "parts": sorted(names),
                         "matrix": np.round(mat, 6).tolist()})
    lo, hi = placed[:, 0].min(axis=0), placed[:, 1].max(axis=0)
    inside = [x1 - x0, float(hi[1] - floor) + CLEAR, z1 - z0]
    return {
        "modules": out_mods,
        "box": {"inside_mm": [round(v, 1) for v in inside],
                "outside_mm": [round(v + 2 * WALL, 1) for v in inside],
                "floor": [round(x0, 2), round(floor, 2), round(z0, 2)],
                "volume_l": round(math.prod(v + 2 * WALL for v in inside) / 1e6, 1)},
        "steps": [f"{m.label}: {m.how}" for m in order],
        "packed_mm": [round(float(v), 1) for v in hi - lo],
    }


def payload(linkage: str, store: str | None = None) -> dict:
    """``/api/cradle/{key}/pack``'s and ``/room``'s bodies: :func:`flat_pack` and
    :func:`spiderpig.cradle_report.room` of the cradle fabricated at crank angle 0 (the baked
    clip's first frame; the fabrication cache in ``store`` keeps it), a package function for
    :func:`spiderpig.workers.submit`."""
    from spiderpig.config import BuildConfig
    from spiderpig.cradle_report import room
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.store import Store

    cfg = BuildConfig(linkage=linkage, robot=False)
    mech = fabricate(template_for(cfg), cfg, 0.0, store=Store.of(store) if store else None)
    return {"pack": flat_pack(mech, cfg.lk), "room": room(cfg, mech)}
