"""The guide's wiring step: a block diagram drawn from what the design buys and what its
controller declares.

The design's controller (the role ``controller``, :mod:`spiderpig.actuators.controllers`)
says which bought part is which box (:attr:`Controller.nodes`: a walker's battery,
protection board, switch, board and servos; an arm's supply, board, host computer, steppers,
camera, pump and valve) and how they connect (:attr:`Controller.wiring`). The parts are the
robot's bought bodies and its BOM lines; the cables are the bought items whose key or name
says cable, lead, pigtail, plug or harness (:data:`controllers.CABLE`), each on the link
whose pattern it matches. Nothing here is specific to one design or kind: a cable this can't
place is listed under the diagram, and a design with no controller (your own electronics)
draws its motors alone.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw

from spiderpig.guide.render import INK, font

ELECTRICAL = ("electronics", "servo", "motor", "pneumatic", "sensor")
"""The catalog categories whose parts can be boxes of the diagram."""
WHO = {"L": "left", "R": "right"}


@dataclass
class Node:
    role: str
    title: str
    label: str = ""


def _short(name: str) -> str:
    """A catalog name without its notes: "XT30 pigtail, 16 AWG" -> "XT30 pigtail"."""
    return re.split(r"[,(]", name)[0].strip()


def _who(body: str) -> str:
    """A body's name as its box's first words: a side's ``L.servo`` -> "left servo", a
    drive's ``drive_q2_motor`` -> "drive q2 motor"."""
    side = re.match(r"^([LR])\.(.*)$", body)
    if side:
        return f"{WHO[side.group(1)]} {side.group(2).split('_')[0]}"
    return body.replace("_", " ")


def parts_of(mech) -> list[tuple[str, str, str, str, float]]:
    """Every bought part: ``(body or "", catalog key, name, category, qty)``, the bodies
    first, then the BOM lines no body is."""
    from spiderpig.hardware import catalog

    def item(key):
        try:
            return catalog.get(key)
        except KeyError:
            return None

    out = []
    for b in mech.bodies:
        if b.fab != "purchased" or not b.bom_key:
            continue
        it = item(b.bom_key)
        out.append((b.name, b.bom_key, getattr(it, "name", b.bom_key),
                    getattr(it, "category", ""), 1.0))
    for x in mech.bom_extras:
        it = item(x.key)
        if it is not None:
            out.append(("", x.key, it.name, it.category, float(x.qty)))
    return out


def wiring_of(mech, label_of: dict[str, str], ctl=None
              ) -> tuple[list[Node], list[tuple], list[str]]:
    """The nodes (one per electrical part the controller ``ctl`` names), the links ``(a, b,
    cables)`` between node indices (its :attr:`Controller.wiring`), and the cables no link
    takes, from ``mech`` (``label_of``: body -> label). ``ctl`` ``None``: no controller (your
    own), the motors alone."""
    from spiderpig.actuators.controllers import CABLE, Controller

    ctl = ctl if ctl is not None else Controller("none", "", (), nodes=(
        ("motor", r"servo_(?!driver)|sts\d|xl\d|^stepper_(?!cable)"),))
    nodes: list[Node] = []
    cables: list[tuple[str, str]] = []           # (key, what the diagram says)
    accessories: list[tuple[str, str]] = []      # bought, no box: a diode on a link
    seen_line: set[str] = set()
    bodies: list[str] = []
    lines: list[tuple[str, str, str]] = []       # (role, key, name): a box no body draws
    for body, key, name, cat, qty in parts_of(mech):
        if not body and key in seen_line:
            continue
        seen_line.add(key)
        # a cable by its key first (a servo's bus cable is no servo), then a box by its key,
        # then a cable by its name (a supply's name says plug), then a box by its name
        role = ctl.role_of(key) if cat in ELECTRICAL else None
        if re.search(CABLE, key, re.I) or (role is None and re.search(CABLE, name, re.I)):
            cables.append((key, _short(name) + (f" x {qty:g}" if not body and qty != 1 else "")))
            continue
        role = role or (ctl.role_of("", name) if cat in ELECTRICAL else None)
        if role is None:
            if cat in ELECTRICAL:
                accessories.append((key, _short(name)))
            continue
        if not body:
            lines.append((role, key, name))
            continue
        nodes.append(Node(role, _short(name), label_of.get(body, "")))
        bodies.append(body)
    drawn = {n.role for n in nodes}
    for role, key, name in lines:    # a BOM line is a box only where no body is its role's
        if role in drawn:
            accessories.append((key, _short(name)))
            continue
        nodes.append(Node(role, _short(name)))
        bodies.append("")
    roles = {n.role for n in nodes}
    for n, body in zip(nodes, bodies, strict=True):     # several of a role: which is which
        if body and sum(m.role == n.role for m in nodes) > 1:
            n.title = f"{_who(body)}: {n.title}"
    links: list[tuple[int, int, list[str]]] = []
    left = list(dict.fromkeys(cables))
    extra = list(dict.fromkeys(accessories))
    for w in ctl.wiring:
        if w.a not in roles or w.b not in roles:
            continue
        on = [c for c in left + extra
              if w.cables and re.search(w.cables, " ".join(c), re.I)]
        left = [c for c in left if c not in on]
        names = list(dict.fromkeys(n for _, n in on)) or ([w.via] if w.via else [])
        for i, n in enumerate(nodes):
            if n.role != w.a:
                continue
            for j, m in enumerate(nodes):
                if m.role == w.b:
                    links.append((i, j, names))
    return nodes, links, [name for _, name in left]


def columns(nodes: list[Node], links: list[tuple]) -> dict[int, int]:
    """Each node's column: its depth along the links from the nodes nothing feeds (the
    power on the left, the motors on the right)."""
    into: dict[int, set[int]] = {i: set() for i in range(len(nodes))}
    for i, j, _ in links:
        into[j].add(i)
    col: dict[int, int] = {}

    def depth(i: int, trail: tuple[int, ...] = ()) -> int:
        if i in col:
            return col[i]
        ins = [k for k in into[i] if k not in trail]
        col[i] = 0 if not ins else 1 + max(depth(k, (*trail, i)) for k in ins)
        return col[i]

    for i in range(len(nodes)):
        depth(i)
    return col


def draw(nodes: list[Node], links: list[tuple], left: list[str], path: Path,
         size: tuple[int, int] = (1200, 900)) -> None:
    """The block diagram as a PNG: columns by depth along the links (the power on the left,
    the motors on the right); each link with its cables."""
    w, h = size
    img = Image.new("RGB", size, (255, 255, 255))
    d = ImageDraw.Draw(img)
    col = columns(nodes, links)
    ncol = max(col.values(), default=0) + 1
    per: dict[int, list[int]] = {}
    for i in range(len(nodes)):
        per.setdefault(col[i], []).append(i)
    bw = min(200, (w - 120) / ncol - 20) if ncol > 1 else 200
    usable = h - 160
    crowd = max((len(v) for v in per.values()), default=1)
    bh = max(60, min(110, usable / crowd - 12))
    at: dict[int, tuple[float, float]] = {}
    for c, idx in per.items():
        for k, i in enumerate(idx):
            x = 60 + (c * (w - 120 - bw) / (ncol - 1) if ncol > 1 else (w - bw) / 2 - 60)
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
        x, yy = at[i]
        fill = (253, 226, 196) if col[i] == ncol - 1 or n.role == "board" else (232, 236, 244)
        d.rounded_rectangle((x, yy, x + bw, yy + bh), 12, fill=fill, outline=INK, width=3)
        d.text((x + bw / 2, yy + 22), n.label or n.role, fill=INK, font=big, anchor="mm")
        words = n.title.split()
        lines, cur = [], ""
        for wd in words:
            if len(cur) + len(wd) > bw // 9:
                lines.append(cur)
                cur = wd
            else:
                cur = f"{cur} {wd}".strip()
        lines.append(cur)
        d.multiline_text((x + bw / 2, yy + bh / 2 + 12), "\n".join(lines[:3]), fill=INK,
                         font=small, anchor="mm", align="center")
    if left:
        d.text((60, y), "Also: " + "; ".join(left), fill=INK, font=small)
    img.save(path, compress_level=6)
