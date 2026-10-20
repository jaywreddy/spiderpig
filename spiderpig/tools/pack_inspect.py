"""Judge a decanting cradle's packing, and look at it, in seconds and with no browser.

    spiderpig pack-inspect decanter_rcm                  # its plan's packing, the automatic one
    spiderpig pack-inspect decanter_rcm --plan default   # the plan's alone
    spiderpig pack-inspect decanter_rcm --plan mine.json # a plan of your own (Plan.to_json)
    spiderpig pack-inspect --layout pack.json            # a saved layout (the /pack body)
    spiderpig pack-inspect decanter_hinge --search --seconds 120 --workers 6

Each candidate (a plan's packing, ``auto``: none, a saved layout, the search's best) gets
three files in ``--out`` (``build/pack/<linkage>/``): ``<name>.png``, a contact sheet (the
top, front and side views and an isometric one, each piece in its own colour and numbered
in the order it goes in, the box drawn round them; three horizontal slices through the box
at 25, 50 and 75 % of its height, each piece's cells and the empty ones; the legend: each
piece's mass, its centre of mass's height, how far it slides, what it rests on, whether it
is supported; the pads drawn in black and listed, a cut list),
``<name>.json`` (:func:`spiderpig.voxpack.metrics`: the box, the true fill, the air per
layer, the unsupported pieces, the pads measured on the real meshes and their cut list,
each piece's free travel and the ``restraint``, the real-shape overlaps, the order's
faults, the mass centre's height, the order; the pieces ``missing``) and
``<name>.layout.json`` (the packing itself, the ``/api/cradle/{key}/pack`` body:
``--layout`` reads it back, the viewer flies it). A ``summary.json`` lists them all, and
the table is printed.

A packing is whole when every piece is placed and every part is in a piece. A plan's step
with no room stops the packer with the pieces placed so far (``voxpack.Layout.missing``):
that candidate's sheet heads red, naming the missing pieces and why, and the command exits
1 (:func:`complete`). An unsupported piece, an overlap, an order fault or a pad with no
room for its block heads the sheet red too, but is a verdict, not an error.

A layout is judged by itself: its pieces' meshes moved by its own matrices, voxelized
afresh on one grid (:func:`spiderpig.voxpack.occupancy`), whoever made it, in the box they
need (their bounds, :data:`spiderpig.voxpack.CLEAR` more: an edited layout's box follows its
pieces; the box it claims, and whether a piece is past it, are in its metrics). The
viewer's pack view edits one (Edit the packing: move and turn a piece; Export layout).

The cradle's parts come from :func:`spiderpig.cradle_pack.cached_input`: the first run
fabricates it at its pack pose (through the store's fabrication cache) and tessellates it,
minutes cold; it is kept in ``<store>/pack/`` and every run after starts in a second or
two. The pictures are :mod:`spiderpig.guide.render`'s (numpy and Pillow: no GPU, no
display).

``--search`` anneals the order, the orientations and the corners round the plan
(:func:`spiderpig.voxpack.search`, ``--seeds`` seeds, ``--seconds`` each, ``--workers``
processes at once, at most 6; without ``--seconds``, 60 s a seed, less when the seeds'
rounds would take more than :data:`SEARCH_BUDGET_S` in all); its best is the ``search``
candidate, its choices in its layout's ``seq``, and the spread across the seeds
(:func:`spiderpig.voxpack.spread`) in its metrics' ``search``: one seed's answer is a
sample, so quote the spread with it.

``--split KEY=GLOB`` packs a what-if (the parts come off their pieces as one of their own):
every piece keeps the orientation it had whole, the new one its parent's (``--relevel KEY``
levels a piece afresh).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import time
from pathlib import Path

import numpy as np

log = logging.getLogger("pack_inspect")

VIEW_SIZE = (600, 470)
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7",
           "#e34948"]
"""The pieces' colours in the order they go in (the dataviz skill's categorical eight, fixed
order); past eight, the same hues lighter. Every piece is numbered too: identity is never
colour alone."""
BASE_COLOUR = "#a8a6a0"     # the base board: neutral, the floor
INK = (24, 24, 28)
MUTED = (110, 108, 104)
EMPTY = (255, 255, 255)
OVERLAP = (20, 20, 20)
VIEWS = {
    "top": {"eye": (0.0, 1.0, 0.0), "up": (0.0, 0.0, -1.0), "light": (0.3, 1.0, 0.5)},
    "front": {"eye": (0.0, 0.0, 1.0), "up": (0.0, 1.0, 0.0), "light": (0.3, 0.6, 1.0)},
    "side": {"eye": (-1.0, 0.0, 0.0), "up": (0.0, 1.0, 0.0), "light": (-1.0, 0.6, 0.3)},
    "iso": {"eye": (1.0, 0.85, 1.25), "up": (0.0, 1.0, 0.0), "light": (0.4, 1.0, 0.6)},
}
CAPTIONS = {"top": ("top", 0, 2), "front": ("front", 0, 1), "side": ("side", 2, 1),
            "iso": ("isometric", None, None)}


SEARCH_SECONDS = 60.0
"""A seed's anneal by default (``--seconds``)."""
SEARCH_BUDGET_S = 150.0
"""A ``--search``'s whole time when ``--seconds`` isn't given: its rounds of ``--workers``
seeds at once share it, each round :data:`SEARCH_SECONDS` at most, so a default
``pack-inspect --search`` (with the input's first fabrication and the sheets) stays an
agent's step, under 4 minutes."""


def search_seconds(seeds: int | None, processes: int) -> float:
    """Each seed's anneal when ``--seconds`` isn't given: :data:`SEARCH_SECONDS`, or less when
    ``seeds`` (default one per process) take more rounds of ``processes`` at once (one after
    another with ``SPIDERPIG_WORKERS=0`` or one process) than :data:`SEARCH_BUDGET_S` holds."""
    from spiderpig import workers

    n = max(1, seeds if seeds is not None else processes)
    rounds = math.ceil(n / processes) if workers.enabled() and processes > 1 else n
    return min(SEARCH_SECONDS, SEARCH_BUDGET_S / rounds)


def _rgb(hex_: str) -> tuple[int, int, int]:
    h = hex_.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def colours(keys: list[str], base: str | None) -> dict[str, tuple[int, int, int]]:
    """Each piece's colour: the base board's neutral, the others :data:`PALETTE` in order
    (lighter past eight)."""
    out = {}
    n = 0
    for k in keys:
        if k == base:
            out[k] = _rgb(BASE_COLOUR)
            continue
        c = np.array(_rgb(PALETTE[n % len(PALETTE)]), float)
        if n >= len(PALETTE):
            c = c + (255 - c) * 0.45
        out[k] = (int(c[0]), int(c[1]), int(c[2]))
        n += 1
    return out


# -- a layout's pieces where it put them -----------------------------------------------------


def placed_pieces(inp: dict, layout: dict) -> list:
    """The pieces of ``layout`` (a pack payload) in its order, each part's mesh (of the
    cradle's input ``inp``, :func:`spiderpig.cradle_pack.gather`) moved by its piece's
    matrix or its own (:class:`spiderpig.voxpack.Packed`)."""
    from spiderpig import voxpack as vp

    out = []
    for mod in layout["modules"]:
        meshes, names, mass, mom, vol = [], [], 0.0, np.zeros(3), 0.0
        for n in mod["parts"]:
            m = np.asarray(mod.get("part_matrix", {}).get(n, mod["matrix"]), float)
            pos, tri = inp["meshes"][n]
            if len(tri):
                meshes.append((np.asarray(pos, float) @ m[:3, :3].T + m[:3, 3],
                               np.asarray(tri, np.int64)))
                names.append(n)
            p = inp["parts"][n]
            c = m[:3, :3] @ np.asarray(p["com"], float) + m[:3, 3]
            mass += p["mass"]
            mom += p["mass"] * c
            vol += p["volume"]
        com = mom / mass if mass > 0 else np.concatenate([q for q, _ in meshes]).mean(axis=0)
        out.append(vp.Packed(mod["key"], mod.get("label", mod["key"]), meshes, mass, com, vol,
                             names))
    return out


def missing_of(inp: dict, layout: dict) -> tuple[list[str], list[str]]:
    """What ``layout`` leaves out of the cradle: the pieces its packer couldn't place (the
    payload's ``missing``: a plan's step with no room stops it) and the input's parts in no
    piece (whoever made it: an edited layout's, an old payload's)."""
    have = {n for mod in layout["modules"] for n in mod["parts"]}
    return list(layout.get("missing") or []), sorted(set(inp["parts"]) - have)


def inside_of(layout: dict) -> tuple[np.ndarray, np.ndarray] | None:
    """The box's inside a layout says it is in (its lower corner and upper), if it says."""
    box = layout.get("box")
    if not box or "floor" not in box:
        return None
    lo = np.asarray(box["floor"], float)
    return lo, lo + np.asarray(box["inside_mm"], float)


# -- the contact sheet -----------------------------------------------------------------------


def _box_item(lo: np.ndarray, hi: np.ndarray):
    from spiderpig.guide.render import Item

    corners = np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1], (lo, hi)[(i >> 2) & 1][2]]
                        for i in range(8)])
    return Item("box", corners, np.zeros((0, 3), np.int64)), corners


EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5),
         (2, 6), (3, 7)]


PAD_INK = (20, 20, 20)
"""The pads' outlines over every view (they are under the pieces: drawn through them)."""


def pad_boxes(metrics: dict) -> list[tuple[str, np.ndarray]]:
    """Each pad of ``metrics``' cut list as a block: its id and its eight corners (mm, the
    box's frame: its footprint from where it stands to the piece over it)."""
    out = []
    for p in metrics.get("pads", {}).get("cut_list", []):
        if p.get("base_mm") is None:
            continue
        x0, z0, x1, z1 = p["footprint"]
        lo = np.array([x0, p["base_mm"], z0])
        hi = np.array([x1, p["base_mm"] + p["height_mm"], z1])
        out.append((p["id"], np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1],
                                        (lo, hi)[(i >> 2) & 1][2]] for i in range(8)])))
    return out


def _view(pieces, cols, numbers, lo, hi, name: str, pads=()):
    """One view: the picture, the box's edges over it, the ``pads``' (:func:`pad_boxes`)
    through it (their ids from above), each piece's number on it."""
    from PIL import ImageDraw

    from spiderpig.guide.render import Item, View, font, projector, render

    items = []
    for pc in pieces:
        if not pc.meshes:
            continue
        pos = np.concatenate([p for p, _ in pc.meshes])
        base = np.cumsum([0] + [len(p) for p, _ in pc.meshes])[:-1]
        tri = np.concatenate([t + b for (_, t), b in zip(pc.meshes, base, strict=True)])
        items.append(Item(pc.key, pos, tri, fill=cols[pc.key], mark=True))
    box, corners = _box_item(lo, hi)
    items.append(box)
    v = VIEWS[name]
    view = View(eye=v["eye"], up=v["up"], light=v["light"], size=VIEW_SIZE, margin=0.07)
    img, marks = render(items, view)
    to_px, _ = projector(items, view)
    d = ImageDraw.Draw(img)
    xs, ys = to_px(corners)
    for a, b in EDGES:
        d.line((float(xs[a]), float(ys[a]), float(xs[b]), float(ys[b])), fill=(120, 96, 60),
               width=1)
    small = font(11)
    for pid, pc in pads:
        px, py = to_px(pc)
        for a, b in EDGES:
            d.line((float(px[a]), float(py[a]), float(px[b]), float(py[b])), fill=PAD_INK,
                   width=2)
        if name == "top":
            d.text((float(px.max()) + 2, float(py.min())), pid, fill=PAD_INK, font=small,
                   stroke_width=2, stroke_fill=(255, 255, 255))
    f = font(13)
    for k, m in marks.items():
        if m.n <= 0:
            continue
        r = 10
        d.ellipse((m.x - r, m.y - r, m.x + r, m.y + r), fill=(255, 255, 255), outline=INK,
                  width=2)
        d.text((m.x, m.y), str(numbers[k]), fill=INK, font=f, anchor="mm")
    return img


def _slice(grid, frac: float, cols_by_index, size) -> tuple:
    """The box's cells at ``frac`` of its height, from above (x right, z down the page):
    each piece's colour, white empty, black where two pieces claim a cell."""
    from PIL import Image, ImageDraw

    nx, ny, nz = grid.count.shape
    j = min(ny - 1, max(0, int(frac * ny)))
    owner = grid.owner()[:, j, :]
    count = grid.count[:, j, :]
    rgb = np.empty((nz, nx, 3), np.uint8)
    rgb[:] = EMPTY
    for i, c in enumerate(cols_by_index):
        rgb[(owner == i).T] = c
    rgb[(count > 1).T] = OVERLAP
    scale = max(1, int(min((size[0] - 40) / nx, (size[1] - 40) / nz)))
    img = Image.fromarray(rgb, "RGB").resize((nx * scale, nz * scale), Image.Resampling.NEAREST)
    out = Image.new("RGB", size, (255, 255, 255))
    ox, oy = (size[0] - img.width) // 2, (size[1] - img.height) // 2
    out.paste(img, (ox, oy))
    d = ImageDraw.Draw(out)
    d.rectangle((ox - 1, oy - 1, ox + img.width, oy + img.height), outline=(120, 96, 60), width=2)
    air = 1.0 - float((count > 0).mean())
    y_mm = (j + 0.5) * grid.cell
    return out, y_mm, air


def sheet(title: str, subtitle: str, pieces, metrics: dict, grid, base: str | None,
          path: Path) -> Path:
    """The contact sheet (the module's docstring) of ``pieces`` (:func:`placed_pieces`),
    judged by ``metrics`` on ``grid``, written to ``path``."""
    from PIL import Image, ImageDraw

    from spiderpig.guide.render import font

    keys = [pc.key for pc in pieces]
    cols = colours(keys, base)
    numbers = {k: i + 1 for i, k in enumerate(keys)}
    lo, hi = grid.lo, grid.hi
    vw, vh = VIEW_SIZE
    pad = 14
    head = 96
    cap = 30
    W = 4 * vw + 5 * pad
    H = head + 2 * (vh + cap) + 3 * pad
    rows = metrics["pieces"]
    cut = metrics.get("pads", {}).get("cut_list", [])
    H = max(H, head + vh + cap + 2 * pad + 40 + 26 * (len(rows) + 1) + 8 * 22 + 16
            + (24 + 19 * len(cut) if cut else 0))
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    d.text((pad, 10), title, fill=INK, font=font(26))
    d.text((pad, 46), subtitle, fill=INK, font=font(17, bold=False))
    bad, notes = [], []
    missing, lost = metrics.get("missing", []), metrics.get("missing_parts", [])
    if missing:
        why = "; ".join(metrics.get("warnings", []))
        bad.append(f"INCOMPLETE: {len(missing)} of {len(missing) + len(keys)} pieces not placed"
                   + (f" ({why})" if why else "") + ": " + ", ".join(missing))
    elif lost:
        bad.append(f"INCOMPLETE: {len(lost)} parts in no piece: " + ", ".join(lost[:6])
                   + (" .." if len(lost) > 6 else ""))
    if metrics["unsupported"]:
        bad.append("unsupported: " + ", ".join(f"{numbers[k]} {k}"
                                                for k in metrics["unsupported"]))
    if metrics["overlaps"]:
        bad.append(f"{len(metrics['overlaps'])} overlaps")
    if metrics["order_faults"]:
        bad.append(f"{len(metrics['order_faults'])} order faults")
    tight = [p["id"] for p in cut if p.get("fits") is False]
    if tight:
        bad.append("no room for pads " + ", ".join(tight))
    if metrics["padded"]:
        notes.append("on pads: " + ", ".join(
            f"{numbers[r['key']]} ({len(r['pads'])})" for r in rows if r["pads"]))
    rs = metrics.get("restraint")
    if rs and rs["loose"]:
        notes.append(f"{len(rs['loose'])} slide more than {rs['held_mm']:g} mm, up to "
                     f"{rs['max_slide_mm']:.0f}")
    text = "; ".join(bad + notes) if bad else "; ".join(
        ["every piece supported, no overlaps, the order holds", *notes])
    d.text((pad, 70), text, fill=(180, 30, 30) if bad else (0, 110, 0), font=font(16))
    boxes = pad_boxes(metrics)
    size = hi - lo
    for n, name in enumerate(VIEWS):
        x = pad + n * (vw + pad)
        y = head
        img.paste(_view(pieces, cols, numbers, lo, hi, name, boxes), (x, y))
        label, a, b = CAPTIONS[name]
        axes = "xyz"
        dims = (f"{label}: {axes[a]} {size[a]:.0f} x {axes[b]} {size[b]:.0f} mm" if a is not None
                else f"{label}: {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm inside")
        d.text((x + 4, y + vh + 4), dims, fill=MUTED, font=font(15, bold=False))
        d.rectangle((x, y, x + vw - 1, y + vh - 1), outline=(225, 225, 225))
    by_index = [cols[k] for k in keys]
    y2 = head + vh + cap + pad
    for n, frac in enumerate((0.25, 0.5, 0.75)):
        x = pad + n * (vw + pad)
        sl, y_mm, air = _slice(grid, frac, by_index, VIEW_SIZE)
        img.paste(sl, (x, y2))
        d.text((x + 4, y2 + vh + 4), f"slice at {y_mm:.0f} mm ({frac:.0%} up): {air:.0%} air "
               "(x right, z down)", fill=MUTED, font=font(15, bold=False))
    # the legend
    x = pad + 3 * (vw + pad)
    y = y2
    d.text((x, y), "#  piece", fill=INK, font=font(15))
    d.text((x + 285, y), "kg", fill=INK, font=font(15))
    d.text((x + 330, y), "CoM mm", fill=INK, font=font(15))
    d.text((x + 405, y), "slide", fill=INK, font=font(15))
    d.text((x + 470, y), "rests on", fill=INK, font=font(15))
    y += 24
    by_key = {r["key"]: r for r in rows}
    f = font(14, bold=False)
    for k in keys:
        r = by_key[k]
        d.rounded_rectangle((x, y + 1, x + 22, y + 19), radius=4, fill=cols[k])
        d.text((x + 11, y + 10), str(numbers[k]), fill=INK, font=font(12), anchor="mm")
        label = r["label"] if len(r["label"]) <= 31 else r["label"][:30] + "."
        d.text((x + 30, y + 2), label, fill=INK, font=f)
        d.text((x + 285, y + 2), f"{r['mass_kg']:.2f}", fill=INK, font=f)
        d.text((x + 330, y + 2), f"{r['com_height_mm']:.0f}", fill=INK, font=f)
        free = {w: v for w, v in r.get("free_mm", {}).items() if w != "+y"}
        if free:
            way = max(sorted(free), key=free.__getitem__)
            d.text((x + 405, y + 2), f"{free[way]:.0f} {way}", fill=INK, font=f)
        on = ", ".join(str(numbers.get(o, o)) for o in r["on"]) or "nothing"
        flag = "" if r["supported"] else " TIPS"
        if r["pads"]:
            on += f" +{len(r['pads'])} pads"
        d.text((x + 470, y + 2), on + flag, fill=(180, 30, 30) if flag else INK, font=f)
        y += 26
    y += 8
    for line in (f"true fill {metrics['fill']:.1%} of the inside ({metrics['cell_fill']:.0%} of "
                 f"its {metrics['cell_mm']:g} mm cells)",
                 f"mass {metrics['mass_kg']:.2f} kg, its centre {metrics['mass_centre_mm']:.0f} "
                 f"mm up ({metrics['mass_centre_frac']:.0%} of the height)",
                 "air per layer, bottom up: " + " ".join(f"{a['air']:.0%}"
                                                          for a in metrics["air_per_layer"]),
                 "slide: the most it moves along the floor before it meets a neighbour",
                 "    or the wall, and which way (mm, cells: the real is up to 2 more)",
                 "pads (black): blocks cut to the real gap under a piece (the cut list)",
                 "TIPS: its centre of mass is over nothing it rests on"):
        d.text((x, y), line, fill=MUTED, font=f)
        y += 22
    if cut:
        pm = metrics["pads"]
        d.text((x, y + 2), f"pads: {pm['count']}, {pm['volume_cm3']:.0f} cm3, the tallest "
               f"{pm['tallest_mm']:.1f} mm (w x d x height, under / on)", fill=INK, font=font(14))
        y += 24
        for p in cut:
            on = (f"{numbers.get(p['on'], p['on'])} {p['on_part']}" if p["on"] != "floor"
                  else "the floor")
            fits = p.get("fits", True)
            note = {False: "  NO ROOM for a block", None: "  (the voxels': unmeasured)"}.get(
                fits, "")
            d.text((x, y), f"{p['id']:>4}  {p['size_mm'][0]:.0f} x {p['size_mm'][1]:.0f} x "
                   f"{p['height_mm']:.1f}  under {p['under']}, on {on}{note}",
                   fill=INK if fits else (180, 30, 30), font=font(13, bold=False))
            y += 19
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path, compress_level=6)
    return path


# -- candidates ------------------------------------------------------------------------------


def judge(inp: dict, layout: dict, name: str, out: Path, draw: bool = True,
          cell: float | None = None) -> dict:
    """Metrics, sheet and layout files of one candidate; its summary row."""
    from spiderpig import voxpack as vp

    t0 = time.perf_counter()
    pieces = placed_pieces(inp, layout)
    grid = vp.occupancy(pieces, None, cell or vp.CELL)     # (the box its pieces need)
    m = vp.metrics(pieces, grid=grid)
    said = inside_of(layout)
    if said is not None:          # a box the layout claims, its pieces past it (an edit's)
        m["claimed_box_inside_mm"] = [round(float(v), 1) for v in said[1] - said[0]]
        m["past_claimed_box"] = bool((grid.lo < said[0] - 0.5).any()
                                     or (grid.hi > said[1] + 0.5).any())
    m["name"] = name
    m["plan"] = layout.get("plan")
    m["linkage"] = inp["linkage"]
    m["missing"], m["missing_parts"] = missing_of(inp, layout)
    m["warnings"] = list(layout.get("warnings") or [])
    if layout.get("search"):
        m["search"] = layout["search"]
    t1 = time.perf_counter()
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{name}.json").write_text(json.dumps(m, indent=1))
    (out / f"{name}.layout.json").write_text(json.dumps(layout))
    png = None
    if draw:
        base = next((pc.key for pc in pieces if pc.key == "base"), None)
        b = m["box"]
        title = (f"{inp['linkage']}: {name}"
                 + (f"  ({layout['plan']})" if layout.get("plan") else ""))
        sub = (f"box {b['outside_mm'][0]:.0f} x {b['outside_mm'][1]:.0f} x "
               f"{b['outside_mm'][2]:.0f} mm outside (x, height, z), {b['volume_l']:.1f} L; "
               f"true fill {m['fill']:.1%}; {m['count']} pieces; mass centre "
               f"{m['mass_centre_mm']:.0f} mm up ({m['mass_centre_frac']:.0%})")
        png = sheet(title, sub, pieces, m, grid, base, out / f"{name}.png")
    t2 = time.perf_counter()
    log.info("%s: metrics %.1f s, sheet %.1f s", name, t1 - t0, t2 - t1)
    return {"name": name, "plan": layout.get("plan"), "volume_l": m["box"]["volume_l"],
            "outside_mm": m["box"]["outside_mm"], "fill": m["fill"],
            "missing": m["missing"], "missing_parts": len(m["missing_parts"]),
            "unsupported": m["unsupported"], "overlaps": len(m["overlaps"]),
            "order_faults": len(m["order_faults"]), "mass_centre_frac": m["mass_centre_frac"],
            "pads": m["pads"]["count"], "tallest_pad_mm": m["pads"]["tallest_mm"],
            "max_slide_mm": m["restraint"]["max_slide_mm"],
            "search": m.get("search"),
            "sheet": str(png) if png else None, "metrics": str(out / f"{name}.json"),
            "layout": str(out / f"{name}.layout.json")}


def complete(row: dict) -> bool:
    """Whether a candidate (:func:`judge`'s row) is a whole packing: every piece placed,
    every part in a piece, its box finite. ``main`` exits 1 when one isn't."""
    return (not row["missing"] and not row["missing_parts"]
            and math.isfinite(row["volume_l"]))


def _plan_arg(arg: str, linkage: str):
    from spiderpig import cradle_pack
    from spiderpig import voxpack as vp

    if arg == "default":
        return "default", cradle_pack.PACK_PLANS.get(linkage)
    if arg == "auto":
        return "auto", None
    p = Path(arg)
    return p.stem, vp.Plan.from_json(json.loads(p.read_text()))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="spiderpig pack-inspect",
                                 description=(__doc__ or "").split("\n\n")[0])
    ap.add_argument("linkage", nargs="?", default=None,
                    help="the cradle (decanter_rcm, decanter_hinge; default: a --layout's own)")
    ap.add_argument("--layout", action="append", default=[], metavar="JSON",
                    help="a saved packing to judge (a /api/cradle/{key}/pack body, or a "
                         "<name>.layout.json this wrote); repeatable")
    ap.add_argument("--plan", nargs="*", default=None, metavar="PLAN",
                    help="pack by: default (the linkage's cradle_pack.PACK_PLANS), auto (no "
                         "plan), or a plan's JSON (voxpack.Plan.to_json); default: default and "
                         "auto, or none with --layout")
    ap.add_argument("--search", action="store_true",
                    help="anneal the order, orientations and corners round the (first) plan")
    ap.add_argument("--seconds", type=float, default=None,
                    help="each seed's anneal (60; less when the seeds' rounds would take the "
                         f"search past {SEARCH_BUDGET_S:g} s in all)")
    ap.add_argument("--workers", type=int, default=4,
                    help="the search's processes at once (4, at most 6)")
    ap.add_argument("--seed", type=int, default=0, help="the search's first seed (0)")
    ap.add_argument("--seeds", type=int, default=None, metavar="N",
                    help="the search's seeds: N anneals (seeds --seed .. --seed + N - 1, "
                         "--workers at a time, each --seconds), the best kept and the spread "
                         "across them in its metrics (search); default one per worker")
    ap.add_argument("--cell", type=float, default=None, help="the voxels, mm (3)")
    ap.add_argument("--store", default=None, help="the project store (default $SPIDERPIG_STORE "
                                                  "or ./.spiderpig)")
    ap.add_argument("--out", type=Path, default=None, help="default build/pack/<linkage>")
    ap.add_argument("--no-sheet", action="store_true", help="the metrics only")
    ap.add_argument("--split", action="append", default=[], metavar="KEY=GLOB[,GLOB]",
                    help="a what-if: the parts matching the patterns (fnmatch, part names) "
                         "come off their pieces as a piece of their own (a design change "
                         "packed before it is made); repeatable; candidates named whatif_*. "
                         "Every piece keeps its orientation (a new one its parent's)")
    ap.add_argument("--relevel", action="append", default=[], metavar="KEY",
                    help="with --split: level this piece afresh (cradle_pack.level) instead of "
                         "keeping its parent's orientation; repeatable")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(name)s %(message)s")
    logging.getLogger("build123d").setLevel(logging.WARNING)
    from spiderpig import cradle_pack
    from spiderpig import voxpack as vp
    from spiderpig.store import PROJECT, Store

    layouts = []
    for f in args.layout:
        doc = json.loads(Path(f).read_text())
        layouts.append((Path(f).name.removesuffix(".json").removesuffix(".layout"),
                        doc.get("pack", doc)))
    linkage = args.linkage or next((d.get("linkage") for _, d in layouts if d.get("linkage")),
                                   None)
    if linkage is None:
        ap.error("name the cradle (a --layout without its linkage)")
    store = Store.of(args.store or PROJECT)
    out = args.out or Path("build/pack") / linkage
    cell = args.cell or vp.CELL
    t0 = time.perf_counter()
    inp = cradle_pack.cached_input(linkage, store)
    log.info("input: %d parts, %.1f s", len(inp["parts"]), time.perf_counter() - t0)
    plans = args.plan if args.plan is not None else ([] if layouts else ["default", "auto"])
    splits = {}
    for arg in args.split:
        key, _, pats = arg.partition("=")
        if not key or not pats:
            ap.error(f"--split {arg!r}: KEY=GLOB[,GLOB]")
        splits[key] = pats.split(",")
    tag = "whatif_" if splits else ""
    cands = list(layouts)
    for arg in plans:
        name, plan = _plan_arg(arg, linkage)
        if arg == "default" and plan is None:
            log.info("%s has no plan: default is auto", linkage)
        t = time.perf_counter()
        cands.append((tag + name, cradle_pack.pack_input(inp, plan, cell, splits=splits,
                                                         relevel=args.relevel)))
        log.info("%s: packed in %.1f s", name, time.perf_counter() - t)
    if args.search:
        name, plan = _plan_arg((args.plan or ["default"])[0], linkage)
        t = time.perf_counter()
        _, _, items, _ = cradle_pack.pack_items(inp, splits, args.relevel)
        procs = min(max(1, args.workers), 6)
        seconds = (args.seconds if args.seconds is not None
                   else search_seconds(args.seeds, procs))
        best = vp.search(items, plan, seconds, procs, args.seed, cell, log=log.info,
                         seeds=args.seeds)
        lay = cradle_pack.pack_input(inp, plan, cell, seq=best.seq, splits=splits,
                                     relevel=args.relevel)
        lay["plan"] = f"search round {plan.name if plan else 'auto'}"
        lay["search"] = {"seconds": seconds, "first_seed": args.seed,
                         **vp.spread(best.runs), "runs": best.runs}
        cands.append((tag + "search", lay))
        log.info("search: %.1f s, score %.1f", time.perf_counter() - t, best.score)
    rows = [judge(inp, lay, name, out, not args.no_sheet, cell) for name, lay in cands]
    (out / "summary.json").write_text(json.dumps(rows, indent=1))
    log.info("%.1f s in all (the engine's import aside)", time.perf_counter() - t0)
    w = max(len(r["name"]) for r in rows)
    print(f"{'candidate':{w}}  {'box mm (outside)':>20}  {'litres':>6}  {'fill':>6}  "
          f"{'pads':>4}  {'slide':>5}  unsupported")
    for r in rows:
        dims = " x ".join(f"{v:.0f}" for v in r["outside_mm"])
        bad = ", ".join(r["unsupported"]) or "-"
        extra = (f"  overlaps {r['overlaps']}" if r["overlaps"] else "") + (
            f"  order faults {r['order_faults']}" if r["order_faults"] else "")
        if not complete(r):
            extra += (f"  INCOMPLETE: missing {', '.join(r['missing']) or '-'} "
                      f"({r['missing_parts']} parts in no piece)")
        if r["search"]:
            sp, lit = r["search"], r["search"].get("litres", {})
            extra += (f"  {sp['seeds']} seeds: {lit.get('best')}-{lit.get('worst')} L, "
                      f"{sp.get('at_best', 0)} at the best")
        print(f"{r['name']:{w}}  {dims:>20}  {r['volume_l']:6.1f}  {r['fill']:6.1%}  "
              f"{r['pads']:4d}  {r['max_slide_mm']:5.0f}  {bad}{extra}")
        if r["sheet"]:
            print(f"{'':{w}}  {r['sheet']}")
    return 0 if all(complete(r) for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
