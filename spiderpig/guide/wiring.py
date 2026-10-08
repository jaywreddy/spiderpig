"""The guide's wiring step: a block diagram drawn from what the design buys.

The electrical parts are the robot's bought bodies whose catalog category is
``electronics`` or ``servo`` (each a box, with its label) and the cables are the bought
items, bodies or BOM lines, whose key or name says cable, lead, pigtail, plug or harness
(each on the link it serves). The links follow the power and the bus: battery, protection
board, switch, board, servos (and the charger into the protection board), each between
the roles a design has. Nothing here is specific to one design: a cable this can't place
is listed under the diagram, and a design without electronics has no wiring step.
"""

from __future__ import annotations

import itertools
import re
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw

from spiderpig.guide.render import INK, font

ROLES = (   # (role, what the catalog key or name says), first match wins
    ("servo", r"servo_(?!driver)|^servo$|sts\d|xl\d"),
    ("battery", r"lipo|battery"),
    ("charger", r"charger|ip2326"),
    ("protection", r"bms|protection"),
    ("switch", r"switch|toggle"),
    ("board", r"driver|esp32|board|controller"),
)
CABLE = r"cable|lead|pigtail|plug|harness|wire|extension"
LINKS = (("battery", "protection"), ("charger", "protection"), ("protection", "switch"),
         ("switch", "board"), ("board", "servo"))
EDGE_OF = (  # which link a cable serves, by its key or name
    (r"xt30|xt60|battery", ("battery", "protection")),
    (r"dc_plug|5521|barrel", ("switch", "board")),
    (r"y_cable|bus|servo|3.?pin|extension", ("board", "servo")),
)


@dataclass
class Node:
    role: str
    title: str
    label: str = ""


def _role(text: str) -> str | None:
    return next((r for r, pat in ROLES if re.search(pat, text, re.I)), None)


def wiring_of(mech, label_of: dict[str, str]) -> tuple[list[Node], list[tuple], list[str]]:
    """The nodes (one per electrical body), the links ``(a, b, cables)`` between node
    indices and the cables no link takes, from ``mech`` (``label_of``: body -> label)."""
    from spiderpig.hardware import catalog

    def item(key):
        try:
            return catalog.get(key)
        except KeyError:
            return None

    nodes: list[Node] = []
    cables: list[tuple[str, str]] = []
    for b in mech.bodies:
        if b.fab != "purchased" or not b.bom_key:
            continue
        it = item(b.bom_key)
        cat = getattr(it, "category", "")
        text = f"{b.bom_key} {getattr(it, 'name', '')}"
        if re.search(CABLE, text, re.I):
            cables.append((b.bom_key, _short(getattr(it, "name", b.bom_key))))
            continue
        role = _role(b.bom_key) or (_role(text) if cat in ("electronics", "servo") else None)
        if cat not in ("electronics", "servo") or role is None:
            continue
        side = re.match(r"^([LR])\.", b.name)
        title = getattr(it, "name", b.bom_key).split(",")[0].split("(")[0].strip()
        if side and role == "servo":
            title = f"{'left' if side.group(1) == 'L' else 'right'} servo: {title}"
        nodes.append(Node(role, title, label_of.get(b.name, "")))
    for x in mech.bom_extras:
        it = item(x.key)
        if it is not None and re.search(CABLE, f"{x.key} {it.name}", re.I):
            cables.append((x.key, _short(it.name) + (f" x {x.qty:g}" if x.qty != 1 else "")))
    links: list[tuple[int, int, list[str]]] = []
    cables = list(dict.fromkeys(cables))
    roles = {n.role for n in nodes}
    chain = [r for r in ("battery", "protection", "switch", "board") if r in roles]
    pairs = list(itertools.pairwise(chain))
    if "charger" in roles and "protection" in roles:
        pairs.append(("charger", "protection"))
    if "board" in roles:
        pairs.append(("board", "servo"))
    left = list(cables)
    for a, b in pairs:
        on = [c for c in left if any(ab == (a, b) and re.search(pat, " ".join(c), re.I)
                                     for pat, ab in EDGE_OF)]
        left = [c for c in left if c not in on]
        names = [name for _, name in on]
        if not names and (a, b) == ("board", "servo"):
            names = ["the servo's own bus cable"]
        for i, n in enumerate(nodes):
            if n.role != a:
                continue
            for j, m in enumerate(nodes):
                if m.role == b:
                    links.append((i, j, names))
    return nodes, links, [name for _, name in left]


def _short(name: str) -> str:
    """A catalog name without its notes: "XT30 pigtail, 16 AWG" -> "XT30 pigtail"."""
    return re.split(r"[,(]", name)[0].strip()


def draw(nodes: list[Node], links: list[tuple], left: list[str], path: Path,
         size: tuple[int, int] = (1200, 900)) -> None:
    """The block diagram as a PNG: power on the left, the board in the middle, the servos
    on the right; each link with its cables."""
    w, h = size
    img = Image.new("RGB", size, (255, 255, 255))
    d = ImageDraw.Draw(img)
    cols = {"battery": 0, "charger": 0, "protection": 1, "switch": 2, "board": 3, "servo": 4}
    ncol = 5
    per: dict[int, list[int]] = {}
    for i, n in enumerate(nodes):
        per.setdefault(cols.get(n.role, 2), []).append(i)
    bw, bh = 200, 110
    at: dict[int, tuple[float, float]] = {}
    usable = h - 160
    for c, idx in per.items():
        for k, i in enumerate(idx):
            x = 60 + c * (w - 120 - bw) / (ncol - 1)
            y = 60 + (k + 1) * usable / (len(idx) + 1) - bh / 2
            at[i] = (x, y)
    small, big = font(15, bold=False), font(18)
    legend: dict[tuple[str, ...], int] = {}        # each set of cables: its number
    for i, j, cs in links:
        (x0, y0), (x1, y1) = at[i], at[j]
        a = (x0 + bw, y0 + bh / 2) if x1 > x0 else (x0 + bw / 2, y0 + bh)
        b = (x1, y1 + bh / 2) if x1 > x0 else (x1 + bw / 2, y1)
        d.line((*a, *b), fill=INK, width=4)
        if cs:      # a numbered tag on the link; the cables listed under the diagram
            k = legend.setdefault(tuple(cs), len(legend) + 1)
            mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            d.ellipse((mx - 13, my - 13, mx + 13, my + 13), fill=(255, 255, 255),
                      outline=(150, 80, 0), width=3)
            d.text((mx, my), str(k), fill=(150, 80, 0), font=big, anchor="mm")
    y = h - 40 - 24 * (len(legend) + (1 if left else 0))
    for cs, k in legend.items():
        d.text((60, y), f"{k}: " + "; ".join(cs), fill=(150, 80, 0), font=small)
        y += 24
    for i, n in enumerate(nodes):
        x, y = at[i]
        fill = (253, 226, 196) if n.role in ("servo", "board") else (232, 236, 244)
        d.rounded_rectangle((x, y, x + bw, y + bh), 12, fill=fill, outline=INK, width=3)
        d.text((x + bw / 2, y + 26), n.label or n.role, fill=INK, font=big, anchor="mm")
        words = n.title.split()
        lines, cur = [], ""
        for wd in words:
            if len(cur) + len(wd) > 22:
                lines.append(cur)
                cur = wd
            else:
                cur = f"{cur} {wd}".strip()
        lines.append(cur)
        d.multiline_text((x + bw / 2, y + 66), "\n".join(lines[:3]), fill=INK, font=small,
                         anchor="mm", align="center")
    if left:
        d.text((60, y), "Also: " + "; ".join(left), fill=INK, font=small)
    img.save(path, compress_level=6)
