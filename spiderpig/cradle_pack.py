"""How a decanting cradle comes apart for transport, and how its pieces stack in a box.

**As modules** (:func:`module_pieces`: B, ``construction.cradle.PACK_POSES``, the user's of
2026-10-13). It is cranked to its pack pose first (:func:`pack_t`: the crank's arm along the
drive side's outer plate's rear leg, a mark seen without counting turns), then comes apart
by hand into three modules and its hand crank (``construction.cradle.module_steps``, round
1's stand review):

1. the hand crank off (its thumb screw): nothing turns without it;
2. the cradle off the base board (the brackets' thumb knobs: the carriage bolts stay in the
   board, the washers and knobs back on them), by its outer plates' top members;
3. the cradle laid on its drive side (the gearbox down), on a towel;
4. the clips of the cradle link's pins on the passive side out from the bay (B's X and Y:
   quick-release clevis pins, :mod:`construction.pivots.clevis`; they go back in the pins'
   holes, with the spacers that sat over the cradle link);
5. the passive side panel lifted straight up off the pins, level, its pins with it (their
   heads on its links' outer faces: on the passive side down they would drop out), its links
   unable to swing (:func:`slide_hits` checks nothing is in the way of its move); the drive
   side and the cradle box stay one unit, lying as they go in the box, the worm holding
   them as cranked. (Standing, the drive module tipped onto its cradle box once the pins
   cleared, and the panel fell the other way.)

It goes back together the other way (the payload's ``reassembly``): the drive module on its
drive side, the passive panel's links set to the pose by paint marks (taped while it turns
over), the panel lowered level onto the cradle link's holes, the clips in, the cradle stood
up onto the board between its cleats, its passive side's knobs snugged last after a whole
pour, the first build's checks again.

**Flat** (:func:`pieces`: A, whose cradle link rides pillar G between its side's plates).
The cradle comes apart where it is already fastened, with no new joint, in the order
:func:`pieces` gives as its steps (a pack for shipping once: hex keys, threadlocker and an
E-ring's pliers, not a breakdown for every trip):

1. the hand crank off the gearbox's input shaft (its thumb screw);
2. the cradle off the base board (the brackets' thumb knobs: the carriage bolts stay in the
   board);
3. each side's inner plate off (its screws from the bay side, back in its holes);
4. the cradle link's own joints with its side (``mech.connections``): a pivot pin's upper
   E-ring off (the pin stays in the side's links, the ring back on it for the trip), a
   clevis pin's clip out, or a Chicago screw out of its barrel (``chicago6``: it goes back
   into the barrel, which stays in the side's link), a pillar slid out of the link's bore;
   each side panel then lifts off, leaving the cradle box whole;
5. the cradle box apart: the base stop and the light off their saddles, one cradle link
   off the tie rods (their screws), the saddles slid off the rods (their set screws), the
   other link off. Each tie rod travels with its two screws back in its ends, each cradle
   link with the loose spacers that sat over it.

A side's link left hanging on one joint once its cradle link is off (B's b5 on J2) swings
about it into its panel (:func:`tuck`).

**How it stacks** (:mod:`spiderpig.voxpack`: the parts' real shapes, tessellated and filled
solid on :data:`CELL` voxels, :data:`CLEAR` of foam between two pieces):

* every piece but the panels and the base board is first *levelled*: turned about z (the
  mechanism's plane normal) to the angle, by the degree (and the cradle's own tilt), that
  gives its xy bounding box the least area (:func:`level`); a side's pieces know their
  ``outer`` and ``bay`` faces;
* the base board stays where it is, at the bottom; the box's footprint is centred on it and
  as large as the largest piece lying flat;
* the pieces go in one after another, each dropped straight down onto what is in (its own
  bottom over the box's heightmap, column by column), so a piece drops into another's open
  triangles or bay, and the order they drop in is an order they can be put in; each takes
  the orientation and slot that raise the box least, then the lowest, the one nearest a
  corner, where it rests on what is under it (its centre of mass over its contacts, or
  over foam pads under it: ``voxpack.support``);
* the order and the orientations follow the cradle's plan (:data:`PACK_PLANS`), what it
  leaves free picked as above (the panels first, in every order, the rest largest first).

:func:`flat_pack` returns each piece (``modules``: its parts and its 4 x 4 transform from the
assembled pose to the packed one, how it rests), in that order, with the box, its true fill
and the steps: the viewer's pack (``/api/cradle/{key}/pack``, ``viewer/decant.html``) flies
each one there; ``spiderpig pack-inspect`` draws and judges it. Every pose is the fabricated
cradle's at its pack pose (``pose``: crank angle 0, the baked clip's first frame, for a flat
pack; B's, its arm along the rear leg, about 5 1/2 handle turns from rest).
"""

from __future__ import annotations

import contextlib
import math
import re
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from spiderpig import voxpack as vp
from spiderpig.construction import cradle as cr
from spiderpig.mechanism import is_frame, is_link

SIDES = ("L", "R")


# -- the pieces ------------------------------------------------------------------------------


@dataclass
class Piece:
    """What travels as one: ``key``, a ``label``, its ``role`` in the stack (``"base"``: the
    bottom, as it is; ``"panel"``: dropped flat; ``"fill"``: levelled, anywhere), its part
    names and ``how`` it comes off (the step that frees it)."""

    key: str
    label: str
    role: str
    parts: list[str] = field(default_factory=list)
    how: str = ""
    hanging: list[tuple[str, str]] = field(default_factory=list)
    """A panel's links left hanging on one joint once its cradle link is off (the link, the
    joint): :func:`tuck` swings each into the panel."""
    moved: dict[str, np.ndarray] = field(default_factory=dict)
    """Its parts moved within it before it packs (4 x 4, the assembled pose's frame)."""


@dataclass(frozen=True)
class PartInfo:
    """A fabricated part as :func:`pieces` reads it: its body's ``fab``, ``bom_key`` and
    ``rigid_with`` host, and ``points`` (the assembled pose): its bounding box's corners."""

    name: str
    fab: str | None
    bom_key: str | None
    rigid_with: str | None
    points: np.ndarray

    @property
    def lo(self) -> np.ndarray:
        return self.points.min(axis=0)

    @property
    def hi(self) -> np.ndarray:
        return self.points.max(axis=0)


def _side(name: str) -> tuple[str, str]:
    """``"L.b6"`` -> ``("L", "b6")``; a part of neither side -> ``("", name)``."""
    return (name[:1], name[2:]) if name[1:2] == "." and name[:1] in SIDES else ("", name)


def _size(key: str | None) -> str:
    """A fastener's size from its catalog key: ``m5_bhcs_12`` -> ``M5``."""
    m = re.match(r"m(\d+)_", key or "")
    return f"M{m.group(1)}" if m else ""


def _join(xs: Sequence[str]) -> str:
    return xs[0] if len(xs) == 1 else ", ".join(xs[:-1]) + " and " + xs[-1]


def pieces(parts: dict[str, PartInfo], connections, link: str, fastened=()
           ) -> tuple[list[Piece], list[tuple[str, list[str]]]]:
    """The pieces the cradle comes apart into (the module's docstring) and the steps that
    free them, in order (each step's text and the keys of the pieces it frees). ``parts``:
    every fabricated part but the display ones; ``connections``: the mechanism's (a side's
    link pairs); ``link``: the cradle link's class; ``fastened``: the cradle's
    ``meta["fastened"]`` pairs (a part, what it is fastened to)."""
    out: dict[str, Piece] = {}

    def put(key: str, label: str, role: str, name: str) -> None:
        out.setdefault(key, Piece(key, label, role)).parts.append(name)

    crank_side = next((s for s in SIDES if f"{s}.conn" in parts), "L")
    panel ={s: ("drive" if s == crank_side else "passive") for s in SIDES}
    side_name = {s: ("drive side" if s == crank_side else "passive side") for s in SIDES}
    side_word = {s: f"the {side_name[s]}" for s in SIDES}
    # each side: its inner plate's screws (a head on its bay face), the loose parts between
    # its cradle link and its inner plate, the cradle link's joints with the side
    inner: dict[str, set[str]] = {s: set() for s in SIDES}
    over: dict[str, set[str]] = {s: set() for s in SIDES}
    joints: dict[str, list[tuple[str, str]]] = {s: [] for s in SIDES}      # (kind, joint)
    for s in SIDES:
        ln, tn = f"{s}.{link}", f"{s}.torso"
        if ln not in parts or tn not in parts:
            continue
        lb, tb = parts[ln], parts[tn]
        bay = 1.0 if (tb.lo[2] + tb.hi[2]) > (lb.lo[2] + lb.hi[2]) else -1.0
        link_face = bay * (lb.hi[2] if bay > 0 else lb.lo[2])
        t_face = bay * (tb.hi[2] if bay > 0 else tb.lo[2])       # the inner plate's bay face
        for n, p in parts.items():
            if _side(n)[0] != s or n in (ln, tn):
                continue
            lo, hi = sorted((bay * p.lo[2], bay * p.hi[2]))
            if hi >= t_face - 0.5:          # a head on the inner plate's bay face: its screw
                inner[s].add(n)
            elif lo >= link_face - 1e-6:    # wholly between the link and the plate: loose
                over[s].add(n)
        for (_, a, ja), (_, b, jb) in connections:
            for mine, other, j in ((a, b, ja), (b, a, jb)):
                if mine == ln and (other in parts or is_frame(other)):
                    joints[s].append(("pillar" if is_frame(other) else "pin", j))
    through = _through_board(parts)
    saddles = [s.name for s in cr.SADDLES]
    for n in sorted(parts):
        p = parts[n]
        s, stem = _side(n)
        if s:
            if n == f"{s}.{link}" or n in over[s]:
                put(f"link_{s}", f"{side_name[s].capitalize()}'s cradle link ({link})",
                    "fill", n)
            elif stem == "torso" or n in inner[s]:
                put(f"inner_{s}", f"{side_name[s].capitalize()}'s inner plate", "fill", n)
            elif stem.startswith("handle"):
                put("hand_crank", "Hand crank", "fill", n)
            else:
                put(panel[s], f"{panel[s].capitalize()} side panel", "panel", n)
            continue
        host = _side(p.rigid_with or "")[0] or crank_side
        if _board_part(n):
            put("base", "Base board", "base", n)
        elif n.startswith("foot_bracket"):
            if _on_board(p, through):
                put("base", "Base board", "base", n)     # a carriage bolt or its nut
            else:
                put(panel[host], f"{panel[host].capitalize()} side panel", "panel", n)
        elif m := re.fullmatch(r"((?:base_|bay_)?tie_[A-Za-z0-9]+)(?:_screw[LR])?", n):
            put(m.group(1), f"Tie rod {m.group(1).removeprefix('tie_')}", "fill", n)
        elif any(n == sd or n.startswith(f"{sd}_") for sd in saddles):
            sd = next(sd for sd in saddles if n.startswith(sd))
            put(sd, f"{sd.replace('_', ' ').capitalize()} and its liner", "fill", n)
        elif n.startswith("led"):
            put("light", "Light in its clip", "fill", n)
        elif n.startswith("base_stop"):
            put("base_stop", "Base stop", "fill", n)
        else:                               # (a part no rule knows: it travels on its own)
            put(n, n.replace("_", " ").capitalize(), "fill", n)
    for s in SIDES:
        if panel[s] in out:
            _hanging(out, connections, link, s, panel[s])
    fast: dict[str, set[str]] = {}
    for a, b in fastened:
        fast.setdefault(a, set()).add(b)

    def screws_into(key: str, into: str) -> list[str]:
        return [n for n in out[key].parts if into in fast.get(n, ())
                and parts[n].fab == "purchased"]

    def size_of(names: Sequence[str]) -> str:
        return next((z for n in names if (z := _size(parts[n].bom_key))), "")

    steps: list[tuple[str, list[str]]] = []
    if "hand_crank" in out:
        steps.append((_hand_crank_step(out["hand_crank"].parts), ["hand_crank"]))
    if "base" in out:
        steps.append((_base_step(out["base"].parts, parts, "the cradle"), ["base"]))
    inners = [f"inner_{s}" for s in SIDES if f"inner_{s}" in out]
    if inners:
        held = {s: [n for n in out[f"inner_{s}"].parts if not n.endswith(".torso")]
                for s in SIDES if f"inner_{s}" in out}
        size = size_of([n for v in held.values() for n in v])
        counts = {len(v) for v in held.values()}
        many = (f"its {counts.pop()} {size} screws" if len(counts) == 1 else
                _join([f"{len(v)} on {side_word[s]}" for s, v in held.items()])
                + f" {size} screws")
        steps.append((f"Inner plates: take each off, {many} from the bay side; they go back "
                      "in its holes for the trip.", inners))
    pins: dict[str, list[str]] = {}
    pillars: dict[str, list[str]] = {}
    for s in SIDES:
        for kind, j in joints[s]:
            (pins if kind == "pin" else pillars).setdefault(j, []).append(s)
    if pins:
        names = sorted(pins)
        # (a Chicago screw's body is the pin's _screw; a pivot pin's, with its rings, _pin)
        pin_parts = [n for j in names for s in pins[j]
                     for n in (f"{s}.pin_{j}_screw", f"{s}.pin_{j}_pin") if n in parts]
        chicago = all("chicago" in (parts[n].bom_key or "") for n in pin_parts)
        clevis = all(f"{n.removesuffix('_pin')}_clip" in parts for n in pin_parts)
        sides = sorted({s for j in names for s in pins[j]})
        on = "both sides" if len(sides) == 2 else side_word[sides[0]]
        many = len(pin_parts) > 1
        head = (f"Pin{'s' if len(names) > 1 else ''} {_join(names)} (the cradle link's, "
                f"on {on}): ")
        if clevis:
            steps.append((head + _clips_out(len(pin_parts)), []))
        elif chicago:
            undo = (f"undo each one's Chicago screw ({len(pin_parts)})" if many
                    else "undo its Chicago screw")
            steps.append((f"{head}{undo} from the bay side, holding the barrel's head with "
                          "taped pliers. The barrel stays in its link; the screw goes back "
                          "into it for the trip.", []))
        else:       # a pivot pin (pivot8): an E-ring over the cradle link
            undo = (f"take each one's upper E-ring off ({len(pin_parts)})" if many
                    else "take its upper E-ring off")
            steps.append((f"{head}{undo} from the bay side (E-ring pliers, or a flat "
                          "screwdriver in its opening). The pin stays in the side's links; "
                          "the ring goes back on it for the trip.", []))
    panels = [panel[s] for s in SIDES if panel[s] in out]
    if panels:
        slide = (f" (its bore slides off pillar {_join(sorted(pillars))})" if pillars else "")
        whole = (f" Its cradle link rides pillar {_join(sorted(pillars))} between the side's "
                 "plates, so a side can't come off it at pins alone: the cradle comes apart "
                 "flat." if pillars else "")
        steps.append((f"Side panels: lift each one off its cradle link{slide}. The cradle box "
                      "(both cradle links, the tie rods, the saddles, the light, the base "
                      f"stop) comes away whole.{whole}", panels))
    for key, saddle, what in (("base_stop", saddles[-1], "Base stop"),
                              ("light", saddles[0], "Light")):
        if key in out:
            off = screws_into(key, saddle) or [n for n in out[key].parts if "screw" in n
                                               and "set_screw" not in n]
            whose = "its clip's" if key == "light" else "its"
            stays = "; the light stays in the clip" if key == "light" else ""
            many = f"{len(off)} screws" if len(off) != 1 else "screw"
            steps.append((f"{what}: take {whose} {many} out of the "
                          f"{saddle.replace('_', ' ')}{stays}, the screws back in its "
                          "holes.", [key]))
    ties = [k for k in out if re.fullmatch(r"tie_[A-Za-z0-9]+", k)]
    if ties:
        other = next(s for s in SIDES if s != crank_side)
        for s, last in ((other, False), (crank_side, True)):     # the passive link first
            tie_screws = [n for k in ties for n in out[k].parts if n.endswith(f"_screw{s}")]
            keys = [f"link_{s}"] + (ties if last else [])
            spacers = [n for n in out.get(f"link_{s}", Piece("", "", "")).parts
                       if n != f"{s}.{link}"]
            loose = " Its loose spacers travel with it." if spacers else ""
            tail = " Each tie rod travels with its two screws back in its ends." if last else ""
            steps.append((f"{side_name[s].capitalize()}'s cradle link: undo its "
                          f"{len(tie_screws)} {size_of(tie_screws)} tie screws (threadlocker: a "
                          "hex key frees them; a fresh drop when it goes back together) and "
                          f"lift it off the tie rods.{loose}{tail}", keys))
            if not last:
                have = [sd for sd in saddles if sd in out]
                sets = [n for sd in have for n in out[sd].parts if "set_screw" in n]
                per = len(sets) // max(1, len(have))
                steps.append((f"Saddles: loosen each one's {per} {size_of(sets)} set screws "
                              "and slide it off the tie rods with its liner.", have))
    else:
        steps.append(("Cradle links: what's left.", [f"link_{s}" for s in SIDES]))
    done = {k for _, keys in steps for k in keys}
    rest = [k for k in out if k not in done]
    if rest:
        steps.append(("Everything else travels loose: "
                      + _join([out[k].label.lower() for k in rest]) + ".", rest))
    for text, keys in steps:
        for k in keys:
            if k in out and not out[k].how:
                out[k].how = text
    return list(out.values()), steps


def pack_t(config) -> float:
    """The crank angle (radians) a cradle packs at: its pack pose's
    (``construction.cradle.pack_pose``: the crank's arm along an outer-plate member), else
    0 (the flat pack: the baked clip's first frame)."""
    got = cr.pack_pose(config.lk)
    return 0.0 if got is None else got[0]


def _hanging(out: dict[str, Piece], connections, link: str, s: str, key: str) -> None:
    """Side ``s``'s links in piece ``key`` left hanging on one joint once its cradle link
    is off (B's b5 on J2, A's rod on the crankpin): each swings about it (:func:`tuck`)."""
    held: dict[str, list[str]] = {}
    for (_, a, ja), (_, b, jb) in connections:
        if _side(a)[0] == s and f"{s}.{link}" not in (a, b):
            held.setdefault(a, []).append(ja)
            held.setdefault(b, []).append(jb)
    for body, js in sorted(held.items()):
        if len(js) == 1 and is_link(body) and body in out[key].parts:
            out[key].hanging.append((body, js[0]))


def _through_board(parts: dict[str, PartInfo]) -> list[np.ndarray]:
    """The base board's bolts (each one's xz axis): a bracket's purchased part through it."""
    base = parts.get("base_plate")
    if base is None:
        return []
    return [(p.lo + p.hi)[[0, 2]] / 2 for n, p in parts.items()
            if n.startswith("foot_bracket") and p.fab == "purchased" and p.lo[1] < base.lo[1]
            and p.hi[1] > base.hi[1]]


def _on_board(p: PartInfo, through: Sequence[np.ndarray]) -> bool:
    """A bracket's carriage bolt, its washer, its nut or its thumb knob (on a bolt's axis):
    it stays with the board."""
    axis = (p.lo + p.hi)[[0, 2]] / 2
    return ((p.fab == "purchased" or p.name.endswith("_knob"))
            and any(np.hypot(*(axis - t)) < 1.0 for t in through))


def _board_part(name: str) -> bool:
    """The base board, its cleats, its jack locator's layers and its TPU feet."""
    return name == "base_plate" or name.startswith(("base_cleat_", "jack_locator_",
                                                    "foot_boot"))


def module_pieces(parts: dict[str, PartInfo], connections, link: str, lk, fastened=()
                  ) -> tuple[list[Piece], list[tuple[str, list[str]]], list[str]]:
    """The modules a cradle that packs whole (``construction.cradle.PACK_POSES``) comes apart
    into, the steps that free them and the steps that put it back together
    (``construction.cradle.module_steps``): the base board (its feet, cleats and jack
    locator, its carriage bolts and their washers, nuts and thumb knobs); the passive side
    panel (every part of that side but its cradle link, its brackets: it lifts off the
    cradle link's pins, the cradle lying on its drive side, once their clips are out); the
    drive side with the cradle box (both cradle links, the tie rods, the saddles and what is
    in them, the light, the base stop: the worm holds it as it was cranked); the hand crank.
    ``lk`` the cradle's linkage; the rest as :func:`pieces`."""
    out: dict[str, Piece] = {}

    def put(key: str, label: str, role: str, name: str) -> None:
        out.setdefault(key, Piece(key, label, role)).parts.append(name)

    crank_side = next((s for s in SIDES if f"{s}.conn" in parts), "L")
    other = next(s for s in SIDES if s != crank_side)
    through = _through_board(parts)
    drive = ("drive", "Drive side with the cradle box", "panel")
    passive = ("passive", "Passive side panel", "panel")
    for n in sorted(parts):
        p = parts[n]
        s, stem = _side(n)
        if s == crank_side and stem.startswith("handle"):
            put("hand_crank", "Hand crank", "fill", n)
        elif s:
            put(*(passive if s == other and stem != link else drive), n)
        elif _board_part(n) or (n.startswith("foot_bracket") and _on_board(p, through)):
            put("base", "Base board", "base", n)
        elif n.startswith("foot_bracket"):
            host = _side(p.rigid_with or "")[0] or crank_side
            put(*(passive if host == other else drive), n)
        else:
            put(*drive, n)
    if "passive" in out:
        _hanging(out, connections, link, other, "passive")
    pins = sorted({j for (_, a, ja), (_, b, jb) in connections
                   for mine, o, j in ((a, b, ja), (b, a, jb))
                   if mine == f"{other}.{link}" and o in parts and not is_frame(o)})
    breakdown, reassembly = cr.module_steps(lk, pins)
    frees = {"hand_crank": ["hand_crank"], "base": ["base"], "lift": ["passive", "drive"]}
    steps: list[tuple[str, list[str]]] = []
    for text, tag in breakdown:
        if tag == "base" and "base" in out:
            text = _base_step(out["base"].parts, parts, "the cradle", text)
        keys = [k for k in frees.get(tag, []) if k in out]
        if tag in frees and not keys:
            continue                        # (no such piece: no step)
        steps.append((text, keys))
    for text, keys in steps:
        for k in keys:
            if k in out and not out[k].how:
                out[k].how = text
    return list(out.values()), steps, reassembly


def slide_hits(panel: dict, unit: dict, step: float = 0.5) -> list[tuple[str, str, float]]:
    """What side panel ``panel`` (each part's placed shape, by name) meets as it slides
    straight off along z, away from the bay (its own side's way: the bay's middle is z 0),
    against ``unit`` (the drive side and the cradle box): each (panel part, unit part, how far
    it had slid, mm) at its first meeting. The panel goes in ``step`` mm steps (under the
    thinnest part either way), each part only as far as it takes to pass every unit part in
    its way (a box test first, then the solids')."""
    from build123d import Location

    from spiderpig.shapes import moved

    def box(sh) -> np.ndarray:
        bb = sh.bounding_box()
        return np.array([[bb.min.X, bb.min.Y, bb.min.Z], [bb.max.X, bb.max.Y, bb.max.Z]])

    pb = {n: box(sh) for n, sh in panel.items()}
    ub = {n: box(sh) for n, sh in unit.items()}
    way = 1.0 if sum(b[0, 2] + b[1, 2] for b in pb.values()) > 0 else -1.0
    hits = []
    for pn, sh in panel.items():
        a = pb[pn]
        for un, q in unit.items():
            c = ub[un]
            if not ((a[0, :2] < c[1, :2]) & (c[0, :2] < a[1, :2])).all():
                continue
            travel = (c[1, 2] - a[0, 2]) if way > 0 else (a[1, 2] - c[0, 2])
            for d in np.arange(0.0, travel + step, step):
                z0, z1 = a[0, 2] + way * d, a[1, 2] + way * d
                if z1 <= c[0, 2] or z0 >= c[1, 2]:
                    continue
                inter = moved(sh, Location((0.0, 0.0, float(way * d)))) & q
                vol = sum(x.volume for x in inter.solids()) if inter is not None else 0.0
                if vol > 0.01:
                    hits.append((pn, un, round(float(d), 2)))
                    break
    return hits


def _hand_crank_step(names: Sequence[str]) -> str:
    """How the hand crank comes off: its thumb screw by hand (or a set screw's key)."""
    thumbs = [n for n in names if "thumb_screw" in n]
    if thumbs:
        return ("Hand crank: loosen its thumb screw by hand and pull it off the gearbox's input "
                "shaft; the screw goes back into its hub.")
    sets = [n for n in names if "set_screw" in n]
    many = f"{len(sets)} set screws" if len(sets) != 1 else "its set screw"
    return f"Hand crank: loosen {many} and pull it off the gearbox's input shaft."


def _base_step(names: Sequence[str], parts: dict[str, PartInfo], what: str,
               text: str = "") -> str:
    """How ``what`` comes off the base board: the nuts (or thumb knobs) on its brackets'
    carriage bolts (``text``: the words to give it, their count put in)."""
    knobs = [n for n in names if n.endswith("_knob")]
    nuts = [n for n in names if "nut" in (parts[n].bom_key or "")]
    size = next((z for n in nuts if (z := _size(parts[n].bom_key))), "")
    if text:
        return text.replace("the brackets' thumb knobs", f"the {len(knobs)} thumb knobs on "
                            "the brackets' feet") if knobs else text
    if knobs:
        return (f"Base board: undo the {len(knobs)} thumb knobs on the brackets' feet by hand "
                f"and lift {what} off it. The carriage bolts stay in the board, their "
                "washers and knobs back on.")
    thumb = all("thumb" in (parts[n].bom_key or "") for n in nuts)
    kind = "thumb nuts" if thumb else "nuts"
    how = " by hand" if thumb else ""
    return (f"Base board: undo the {len(nuts)} {size} {kind} on the brackets' feet{how} and "
            f"lift {what} off it. The carriage bolts stay in the board, their nuts back on.")


def _clips_out(n: int) -> str:
    many = f"pull each one's hairpin cotter (R-clip, {n})" if n > 1 else "pull its R-clip"
    return (f"{many} from the bay side, by hand. The pin stays in the side's links; the clip "
            "goes back in its hole for the trip.")


# -- the packing (:mod:`spiderpig.voxpack`) -------------------------------------------------

CLEAR = vp.CLEAR    # between two pieces and from a piece to the box's wall (foam, cardboard)
WALL = vp.WALL      # the box's corrugated wall
CELL = vp.CELL      # the voxels' size


def level(points: Sequence[np.ndarray], tilt: float = 0.0) -> float:
    """The turn about z (degrees) that gives the parts' ``points`` together the least xy
    bounding box: every whole degree and ``-tilt`` (degrees: the cradle's own tilt), the
    least area winning; within 0.5 % of it, ``-tilt``, then no turn, then the least turn."""
    from scipy.spatial import ConvexHull, QhullError

    xy = np.concatenate([p[:, :2] for p in points])
    if len(xy) > 8:
        with contextlib.suppress(QhullError):       # (collinear: all of them)
            xy = xy[ConvexHull(xy).vertices]
    cand = np.unique(np.concatenate([np.arange(-89.0, 91.0), [((-tilt + 90) % 180) - 90]]))
    a = np.radians(cand)
    c, s = np.cos(a)[:, None], np.sin(a)[:, None]
    x = c * xy[:, 0] - s * xy[:, 1]
    y = s * xy[:, 0] + c * xy[:, 1]
    area = (x.max(axis=1) - x.min(axis=1)) * (y.max(axis=1) - y.min(axis=1))
    ok = area <= area.min() * 1.005
    pref = np.where(np.isclose(cand, ((-tilt + 90) % 180) - 90), 0, np.where(cand == 0, 1, 2))
    best = min(np.flatnonzero(ok), key=lambda i: (pref[i], abs(cand[i])))
    return float(cand[best])


PACK_PLANS: dict[str, vp.Plan] = {
    "decanter_rcm": vp.Plan(
        "B: the passive panel flat on the board, the drive side on its gearbox over it",
        steps=(
            vp.Step("passive", down="bay", spin=0, corner="+x-z",
                    why="in first, as it comes off the pins: flat on the board, its outer "
                        "face up, to the board's +x end"),
            vp.Step("drive", down="outer", spin=180, corner="-x-z",
                    why="as it lies when the panel comes off (round 1's breakdown: on its "
                        "drive side): the gearbox down, its plates over the panel's, the "
                        "cradle box up; it rests on the panel, no pads, the heaviest piece "
                        "low"),
            vp.Step("hand_crank",
                    why="on the board under the drive side's plates, wherever it drops lowest"),
        ),
        note="B's pieces: the board, the two side modules, the hand crank. The drive side "
             "with its cradle box is 441 mm across the bay whichever way it lies: with the "
             "board at the bottom the box is about 211 L, the board's footprint times that. "
             "(Before round 1's pack pose the drive side went cradle box down, on 7 pads: "
             "on its gearbox it tipped. Since round 1's structure the panel's inner plate "
             "has a triangle and a leg on two columns where the gearbox had dropped past it, "
             "the panel turned the other way and to the +x end: turned as before, the "
             "drive side lands on the panel and tips.)"),
    "decanter_hinge": vp.Plan(
        "A: both side panels face down on the board, end to end, the loose parts in their "
        "open triangles",
        steps=(
            vp.Step("passive", down="outer",
                    why="its plates flat on the board, its pillars up (foam pads level it)"),
            vp.Step("drive", down="outer", spin=180,
                    why="turned end for end: its plates lie beside the passive panel's, their "
                        "pillars between each other's links, both on the board"),
        ),
        note="A's pieces: the board, the side panels, the inner plates, the cradle links, "
             "the tie rods, the saddles, the light, the base stop, the hand crank; what the "
             "plan leaves free (the saddles, the base stop, the links, the ties, the plates, "
             "the light, the crank) drops into the panels' openings, the largest first."),
}
"""Each cradle's pack plan (:class:`spiderpig.voxpack.Plan`): the human packer's intent, per
piece (its face down, where it goes, what goes in first); what a plan leaves free the packer
picks (:meth:`spiderpig.voxpack.Packer.greedy`), a piece it doesn't name too. ``spiderpig
pack-inspect`` judges a plan (and searches round it: ``--search``)."""


# -- the cradle's flat pack ------------------------------------------------------------------


def part_infos(mech) -> dict[str, PartInfo]:
    """The fabricated cradle's parts (not the display ones) as :func:`pieces` reads them:
    each by its bounding box's corners."""
    bodies = [b for b in mech.bodies if b.part is not None and b.fab != "display"]
    corners = {}
    for b in bodies:
        bb = b.placed_part().bounding_box()
        corners[b.name] = _corners((bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z))
    return {b.name: PartInfo(b.name, b.fab, b.bom_key, b.rigid_with, corners[b.name])
            for b in bodies}


def _corners(lo, hi) -> np.ndarray:
    return np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1], (lo, hi)[(i >> 2) & 1][2]]
                     for i in range(8)], float)


def tuck(group: Sequence[np.ndarray], pivot, others: Sequence[np.ndarray]) -> float:
    """The swing (degrees about z through ``pivot``, xy) that tucks a link left hanging on
    one joint into its panel: ``group`` its parts' points (the link's, its hardware's),
    ``others`` the rest of the panel's. The least xy bounding box of the whole panel among
    the angles it swings to either way before a part of it meets another part's box (the
    joint's own hardware, coaxial with the pivot, aside; boxes that meet as assembled
    don't count); of those within 0.1 % of it, the one squarest to the panel (the link's
    own box the least), then the least swing; none unless it saves 1 %."""
    px, py = float(pivot[0]), float(pivot[1])
    ob = np.array([[p.min(axis=0), p.max(axis=0)] for p in others])
    span = (ob[:, 1, :2] - ob[:, 0, :2]).max(axis=1)
    coax = ((span <= 40.0) & (ob[:, 0, 0] <= px) & (px <= ob[:, 1, 0])
            & (ob[:, 0, 1] <= py) & (py <= ob[:, 1, 1]))
    check = ob[~coax]
    lo0, hi0 = ob[:, 0, :2].min(axis=0), ob[:, 1, :2].max(axis=0)
    # every whole degree at once: each part's box (angle, part, min/max, xyz)
    deg = np.arange(-180.0, 181.0)
    a = np.radians(deg)[:, None]
    pts = np.concatenate(group)
    starts = np.cumsum([0] + [len(p) for p in group])[:-1]
    x = np.cos(a) * (pts[:, 0] - px) - np.sin(a) * (pts[:, 1] - py) + px
    y = np.sin(a) * (pts[:, 0] - px) + np.cos(a) * (pts[:, 1] - py) + py
    b = np.empty((len(deg), len(group), 2, 3))
    for k, v in enumerate((x, y)):
        b[:, :, 0, k] = np.minimum.reduceat(v, starts, axis=1)
        b[:, :, 1, k] = np.maximum.reduceat(v, starts, axis=1)
    b[:, :, 0, 2] = [p[:, 2].min() for p in group]
    b[:, :, 1, 2] = [p[:, 2].max() for p in group]
    ov = (np.minimum(b[:, :, None, 1], check[None, None, :, 1])
          - np.maximum(b[:, :, None, 0], check[None, None, :, 0]))
    hit = np.asarray((ov > 0.1).all(axis=-1))           # (angle, part, other)
    zero = 180
    new = (hit & ~hit[zero][None]).any(axis=(1, 2))
    area = np.prod(np.maximum(hi0, b[:, :, 1, :2].max(axis=1))
                   - np.minimum(lo0, b[:, :, 0, :2].min(axis=1)), axis=-1)
    own = np.prod(b[:, :, 1, :2].max(axis=1) - b[:, :, 0, :2].min(axis=1), axis=-1)
    area0 = float(area[zero])
    cands = [(area0, float(own[zero]), 0.0)]
    for step in (1, -1):                # either way round, as far as it swings freely
        i = zero + step
        while 0 <= i < len(deg) and not new[i]:
            cands.append((float(area[i]), float(own[i]), float(deg[i])))
            i += step
    least = min(c[0] for c in cands)
    if least >= 0.99 * area0:
        return 0.0
    near = [c for c in cands if c[0] <= least * 1.001]
    square = min(c[1] for c in near)
    return min((c for c in near if c[1] <= square * 1.01), key=lambda c: abs(c[2]))[2]


MESH_TOLERANCE = (0.5, 0.5)
"""The parts' tessellation for the packing (``mesh.tessellate_many``: relative, angular)."""


def gather(mech, lk, t: float = 0.0, config=None) -> dict:
    """What the packing needs of the fabricated cradle ``mech`` at crank angle ``t`` (its
    pack pose, :func:`pack_t`), with no OCCT after it (:func:`cached_input` keeps it): the
    linkage, its cradle link's class, the pose (``t``, the bottle's ``tilt``, the handle
    ``turns`` from rest or None: ``construction.cradle.pack_pose``), each part's bounding
    box, fabrication, catalog key, host, volume, mass (kg) and centre of mass, its
    tessellation (``meshes``: positions, triangles) in the assembled pose, the mechanism's
    ``connections``, the cradle's ``fastened`` pairs, each joint's xy and, for modules,
    ``slide_off`` (:func:`slide_hits`: the passive side panel off its cradle link's pins)."""
    from spiderpig.config import BuildConfig
    from spiderpig.cradle_report import part_mass_kg
    from spiderpig.hardware import mass as mass_mod
    from spiderpig.mesh import tessellate_many

    cfg = config or BuildConfig(linkage=lk.key, robot=False)
    bodies = [b for b in mech.bodies if b.part is not None and b.fab != "display"]
    infos = part_infos(mech)
    shapes = [b.placed_part() for b in bodies]
    meshes = tessellate_many(shapes, *MESH_TOLERANCE)
    parts, mesh_of = {}, {}
    for b, sh, (pos, idx, _) in zip(bodies, shapes, meshes, strict=True):
        props = mass_mod.part_props(sh)
        i = infos[b.name]
        parts[b.name] = {"fab": b.fab, "bom_key": b.bom_key, "rigid_with": b.rigid_with,
                         "lo": i.lo.tolist(), "hi": i.hi.tolist(),
                         "volume": float(props.volume), "mass": float(part_mass_kg(cfg, b)),
                         "com": [float(v) for v in props.com]}
        mesh_of[b.name] = (np.asarray(pos, np.float32).reshape(-1, 3),
                           np.asarray(idx, np.uint32).reshape(-1, 3))
    pose = cr.pack_pose(lk)
    turns = None if pose is None else round(abs(pose[2]), 1)
    out = {"linkage": lk.key, "link": cr.cradle_link(lk), "t": float(t),
           "tilt": math.degrees(float(mech.meta.get("cradle_tilt", 0.0))), "turns": turns,
           "parts": parts, "meshes": mesh_of,
           "connections": [[list(a), list(b)] for a, b in mech.connections],
           "fastened": [list(p) for p in mech.meta.get("fastened", ())],
           "joints": {f"{b.name}|{j.name}": [float(v) for v in
                                             (b.pose.matrix @ j.pose.matrix)[:2, 3]]
                      for b in mech.bodies for j in b.joints},
           "slide_off": None}
    if turns is not None:
        pcs, _, _ = _pieces_of(out, lk)
        by = {pc.key: pc for pc in pcs}
        placed = dict(zip([b.name for b in bodies], shapes, strict=True))
        # the clips out are the cradle link's pins' (X and Y on B); a clevis pin inside the
        # side (B's K and J2) keeps its clip and slides off with the panel
        on_link = set(lk.links[cr.cradle_link(lk)][0])
        out_ = sorted(n for n in by["passive"].parts if n.endswith("_clip")
                      and n.rsplit(".pin_", 1)[-1].removesuffix("_clip") in on_link)
        panel = {n: placed[n] for n in by["passive"].parts if n not in out_}
        unit = {n: placed[n] for n in by["drive"].parts}
        out["slide_off"] = {"hits": [list(h) for h in slide_hits(panel, unit)],
                            "clips_out": out_}
    return out


def _pieces_of(inp: dict, lk=None
               ) -> tuple[list[Piece], list[tuple[str, list[str]]], list[str]]:
    """:func:`module_pieces` or :func:`pieces` of a :func:`gather`'s input: the pieces, the
    steps that free them and (modules) the steps back together. ``lk``: its linkage
    (default: the registry's by the input's key)."""
    infos = {n: PartInfo(n, p["fab"], p["bom_key"], p["rigid_with"], _corners(p["lo"], p["hi"]))
             for n, p in inp["parts"].items()}
    conns = [(tuple(a), tuple(b)) for a, b in inp["connections"]]
    fastened = [tuple(p) for p in inp["fastened"]]
    if inp["turns"] is not None:
        if lk is None:
            from spiderpig import linkage

            lk = linkage.get(inp["linkage"])
        return module_pieces(infos, conns, inp["link"], lk, fastened)
    pcs, steps = pieces(infos, conns, inp["link"], fastened)
    return pcs, steps, []


def pack_items(inp: dict, splits: dict[str, Sequence[str]] | None = None
               ) -> tuple[list[Piece], list[tuple[str, list[str]]], list[vp.Item],
                          dict[str, np.ndarray]]:
    """The pieces of :func:`gather`'s input ``inp``, the steps that free them, each as a
    :class:`spiderpig.voxpack.Item` (its parts' meshes, a hanging link swung in: :func:`tuck`;
    a fill piece levelled: :func:`level`; a side's ``outer`` and ``bay`` faces, and the
    ``cradle box``'s where the piece carries it: its bay's), and each
    moved part's own 4 x 4 within its piece. ``splits``: a what-if (``spiderpig
    pack-inspect --split``), a new piece's key -> the patterns (``fnmatch``) of the parts it
    takes from theirs."""
    pcs, steps, _ = _pieces_of(inp)
    if splits:
        pcs = _split(pcs, splits)
    mesh = {n: (np.asarray(p, float), np.asarray(t)) for n, (p, t) in inp["meshes"].items()}
    com = {n: np.asarray(p["com"], float) for n, p in inp["parts"].items()}
    rigid = {n: p["rigid_with"] for n, p in inp["parts"].items()}
    moved: dict[str, np.ndarray] = {}
    for pc in pcs:              # each hanging link swung into its panel (:func:`tuck`)
        for body, joint in pc.hanging:
            group = [n for n in pc.parts if n == body or rigid[n] == body]
            rest = [mesh[n][0] for n in pc.parts if n not in group]
            pivot = np.asarray(inp["joints"][f"{body}|{joint}"], float)
            deg = tuck([mesh[n][0] for n in group], pivot, rest)
            if deg == 0.0:
                continue
            m = np.eye(4)
            m[:3, :3] = vp.rot_about("z", deg)
            m[:2, 3] = pivot - m[:2, :2] @ pivot
            for n in group:
                mesh[n] = (mesh[n][0] @ m[:3, :3].T + m[:3, 3], mesh[n][1])
                com[n] = m[:3, :3] @ com[n] + m[:3, 3]
                pc.moved[n] = m @ pc.moved.get(n, np.eye(4))
                moved[n] = pc.moved[n]
    items = []
    for pc in pcs:
        ms = [mesh[n] for n in pc.parts if len(mesh[n][1])]
        mass = sum(inp["parts"][n]["mass"] for n in pc.parts)
        c = (sum(inp["parts"][n]["mass"] * com[n] for n in pc.parts) / mass if mass > 0
             else np.concatenate([p for p, _ in ms]).mean(axis=0))
        pre = np.eye(3)
        if pc.role == "fill":
            pre = vp.rot_about("z", level([p for p, _ in ms], inp["tilt"]))
        faces = {}
        sides = [_side(n)[0] for n in pc.parts if _side(n)[0]]
        if sides and pc.role != "base":
            s = max(set(sides), key=lambda x: (sides.count(x), x))
            zs = np.concatenate([mesh[n][0][:, 2] for n in pc.parts if _side(n)[0] == s])
            out = np.array([0.0, 0.0, 1.0 if zs.mean() > 0 else -1.0])
            faces = {"outer": pre @ out, "bay": -(pre @ out)}
            if any(n.startswith("tie_") for n in pc.parts):     # (B's drive side's)
                faces["cradle box"] = faces["bay"]
        items.append(vp.Item(pc.key, pc.label, pc.role, ms, float(mass), np.asarray(c, float),
                             float(sum(inp["parts"][n]["volume"] for n in pc.parts)), pre,
                             faces))
    return pcs, steps, items, moved


def packed_meshes(items: Sequence[vp.Item], layout: vp.Layout) -> dict[str, list[vp.Mesh]]:
    """Each piece's meshes where ``layout`` puts it (the box's frame)."""
    by = {it.key: it for it in items}
    out = {}
    for p in layout.placed:
        out[p.key] = [(pos @ p.rot.T + p.shift, tri) for pos, tri in by[p.key].meshes]
    return out


def _split(pcs: list[Piece], splits: dict[str, Sequence[str]]) -> list[Piece]:
    """``pcs`` with each split's parts (by pattern) moved into a piece of its own (a
    what-if: no step frees it); a piece left empty goes."""
    import fnmatch

    out = []
    new = {k: Piece(k, f"What if: {k}", "fill", how="(a what-if)") for k in splits}
    for pc in pcs:
        keep = []
        for n in pc.parts:
            k = next((k for k, pats in splits.items()
                      if any(fnmatch.fnmatchcase(n, p) for p in pats)), None)
            (new[k].parts if k is not None else keep).append(n)
        pc.parts = keep
        pc.hanging = [(b, j) for b, j in pc.hanging if b in keep]
        if keep:
            out.append(pc)
    return out + [pc for pc in new.values() if pc.parts]


def pack_input(inp: dict, plan: vp.Plan | None | str = "default", cell: float = CELL,
               seq: Sequence[vp.Choice] | None = None,
               splits: dict[str, Sequence[str]] | None = None) -> dict:
    """:func:`flat_pack`'s answer from :func:`gather`'s input ``inp``: packed by ``plan``
    (``"default"``: :data:`PACK_PLANS`' for the linkage; None: automatic), or by an explicit
    sequence of choices ``seq`` (``spiderpig pack-inspect --search``'s); ``splits`` a
    what-if (:func:`pack_items`)."""
    if plan == "default":
        plan = PACK_PLANS.get(inp["linkage"])
    assert plan is None or isinstance(plan, vp.Plan)
    pcs, steps, items, _ = pack_items(inp, splits)
    layout, packer = vp.pack(items, plan, cell, seq)
    return layout_payload(inp, pcs, steps, items, layout, packer.plan)


def layout_payload(inp: dict, pcs: list[Piece], steps, items: Sequence[vp.Item],
                   layout: vp.Layout, plan: vp.Plan) -> dict:
    """The payload (:func:`flat_pack`'s) of ``layout``."""
    by = {pc.key: pc for pc in pcs}
    faces = {it.key: it.faces for it in items}
    placed = packed_meshes(items, layout)
    allp = np.concatenate([pos for ms in placed.values() for pos, _ in ms])
    lo, hi = allp.min(axis=0) - CLEAR, allp.max(axis=0) + CLEAR
    inside = [float(v) for v in hi - lo]
    mods = []
    for p in layout.placed:
        pc = by[p.key]
        mod = {"key": p.key, "label": pc.label, "how": pc.how, "parts": sorted(pc.parts),
               "matrix": np.round(p.matrix(), 6).tolist(), "rests": p.rest.how,
               "on": list(p.rest.on), "pads": [list(q) for q in p.rest.pads]}
        mod["down"], mod["spin"] = vp.describe(p.shape.ri, faces[p.key])
        if pc.moved:
            mod["part_matrix"] = {n: np.round(p.matrix() @ m, 6).tolist()
                                  for n, m in sorted(pc.moved.items())}
        mods.append(mod)
    vol = sum(it.volume for it in items)
    mass = sum(it.mass for it in items)
    com_y = (sum(it.mass * (p.rot @ it.com + p.shift)[1]
                 for it in items for p in layout.placed if p.key == it.key) / mass
             if mass > 0 else float("nan"))
    t = inp["t"]
    out = {
        "linkage": inp["linkage"],
        "modules": mods,
        "box": {"inside_mm": [round(v, 1) for v in inside],
                "outside_mm": [round(v + 2 * WALL, 1) for v in inside],
                "floor": [round(float(v), 2) for v in lo],
                "volume_l": round(math.prod(v + 2 * WALL for v in inside) / 1e6, 1)},
        "steps": [text for text, _ in steps],
        "packed_mm": [round(v - 2 * CLEAR, 1) for v in inside],    # (what's in it)
        "plan": plan.name,
        "seq": [[c.key, c.ri, c.corner] for c in layout.seq],
        "fill": round(vol / math.prod(inside), 4),
        "mass_centre_mm": round(com_y - float(lo[1]), 1),
        "unsupported": [p.key for p in layout.placed if not p.supported],
        "mode": "flat" if inp["turns"] is None else "modules",
        "pose": {"crank_t": round(t, 6), "clip_fraction": round(t / (2.0 * math.pi), 6),
                 "tilt_deg": round(inp["tilt"], 2), "turns_from_rest": inp["turns"]},
    }
    if inp["turns"] is not None:
        out["reassembly"] = _pieces_of(inp)[2]
    if inp.get("slide_off") is not None:
        out["slide_off"] = inp["slide_off"]
    return out


def flat_pack(mech, lk, t: float = 0.0, config=None, plan: vp.Plan | None | str = "default"
              ) -> dict:
    """How the fabricated cradle ``mech`` (at crank angle ``t``: :func:`pack_t`) packs:
    flat (:func:`pieces`), or as modules where it is cranked to its pack pose first
    (``construction.cradle.PACK_POSES``, :func:`module_pieces`), by its :data:`PACK_PLANS`
    plan (``plan``).
    Its pieces (``modules``, in the order they go in the box, bottom up: key, label, how it
    comes off, parts, ``matrix``: the 4 x 4 row-major transform from the assembled pose to
    the packed one, in the cradle's own frame: x along the linkage, y up, z across the bay;
    ``part_matrix``, a part's own where it moves within its piece: a hanging link swung in;
    ``rests``: on what, ``on``, directly or on foam ``pads``, or not; ``down`` and ``spin``:
    the face it lies on and its turn), the box (its inside and
    outside sizes, mm, and its inside's lower corner, ``floor``), the steps that take it
    apart, the ``plan``'s name, the true ``fill`` (the parts' own volumes over the box's
    inside), the mass centre's height over the floor, the ``unsupported`` pieces, ``mode``
    (``"flat"`` or ``"modules"``), the ``pose`` the transforms start from (its crank angle,
    its clip fraction ``t / 2 pi``, the bottle's tilt, the handle turns from rest) and, for
    modules, ``reassembly`` (the steps back together) and ``slide_off``: what the passive
    side panel meets coming off its cradle link's pins (:func:`slide_hits`, its clips out:
    none)."""
    return pack_input(gather(mech, lk, t, config), plan)


# -- the packing's input, kept ---------------------------------------------------------------


def input_path(store, cfg, t: float) -> Path | None:
    """Where :func:`cached_input` keeps the input of ``cfg``'s cradle at crank angle ``t``:
    ``<store>/pack/<the fabrication's entry>-<the code's key>.npz`` (None: no store or no
    key). The fabrication cache's entry names the fabricated parts (:mod:`spiderpig.fabcache`:
    the code, the design, its plan, ``t``); :func:`spiderpig.keys.source_key` of
    :func:`gather` the code that reads them."""
    from spiderpig import fabcache, keys
    from spiderpig.fabricate import design_side, template_for
    from spiderpig.store import Store

    st = Store.of(store)
    if st is None:
        return None
    tmpl = template_for(cfg)
    entry = fabcache.entry_name(tmpl, cfg, design_side(tmpl, cfg), t)
    key = keys.source_key(("spiderpig.cradle_pack:gather", "spiderpig.cradle_pack:_save_input",
                           "spiderpig.cradle_pack:_load_input"), "pack")
    return st.root / "pack" / fabcache.folder_name() / f"{entry}-{key}.npz"


def _save_input(inp: dict, path: Path) -> None:
    import json
    import os
    import tempfile

    arrays = {}
    for n, (p, t) in inp["meshes"].items():
        arrays[f"p|{n}"] = np.asarray(p, np.float32)
        arrays[f"t|{n}"] = np.asarray(t, np.uint32)
    meta = {k: v for k, v in inp.items() if k != "meshes"}
    arrays["meta"] = np.frombuffer(json.dumps(meta).encode(), np.uint8)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".pack-", suffix=".npz")
    os.close(fd)
    np.savez_compressed(tmp, **arrays)
    os.replace(tmp, path)


def _load_input(path: Path) -> dict:
    import json

    with np.load(path) as data:
        inp = json.loads(bytes(data["meta"]).decode())
        names = [k[2:] for k in data.files if k.startswith("p|")]
        inp["meshes"] = {n: (data[f"p|{n}"], data[f"t|{n}"]) for n in names}
    return inp


def cached_input(linkage: str, store=None, reuse_plan: bool = True) -> dict:
    """:func:`gather`'s input of ``linkage``'s cradle at its pack pose (:func:`pack_t`),
    from ``store``'s ``pack/`` when there (:func:`input_path`), else fabricated (through the
    store's fabrication cache) and kept: seconds once kept. ``reuse_plan``: plan the design
    through the store (:func:`spiderpig.stages.planning.plan_config`: its plan re-made, not
    searched for again; the design recorded there), not afresh."""
    import logging

    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.store import Store

    cfg = BuildConfig(linkage=linkage, robot=False)
    t = pack_t(cfg)
    at = Store.of(store) if store is not None else None
    if at is not None and reuse_plan:
        from spiderpig.stages.planning import plan_config

        with contextlib.suppress(Exception):    # (the store's plan, re-made: no search)
            plan_config(cfg, at)
    path = input_path(at, cfg, t) if at is not None else None
    if path is not None and path.is_file():
        try:
            return _load_input(path)
        except (OSError, ValueError, KeyError) as e:
            logging.getLogger(__name__).warning("%s: unreadable (%s): gathering again", path, e)
    mech = fabricate(template_for(cfg), cfg, t, store=at)
    inp = gather(mech, cfg.lk, t, cfg)
    if path is not None:
        _save_input(inp, path)
    return inp


def payload(linkage: str, store: str | None = None) -> dict:
    """``/api/cradle/{key}/pack``'s and ``/room``'s bodies: :func:`flat_pack` of the cradle
    fabricated at its pack pose (:func:`pack_t`) and :func:`spiderpig.cradle_report.room` of
    it at crank angle 0 (the baked clip's first frame; the fabrication cache in ``store``
    keeps both), a package function for :func:`spiderpig.workers.submit`."""
    from spiderpig.config import BuildConfig
    from spiderpig.cradle_report import room
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.store import Store

    cfg = BuildConfig(linkage=linkage, robot=False)
    tmpl = template_for(cfg)
    at = Store.of(store) if store else None
    mech = fabricate(tmpl, cfg, 0.0, store=at)
    inp = cached_input(linkage, at, reuse_plan=False) if at is not None else None
    if inp is None:
        t = pack_t(cfg)
        packed = mech if t == 0.0 else fabricate(tmpl, cfg, t, store=at)
        inp = gather(packed, cfg.lk, t, cfg)
    return {"pack": pack_input(inp), "room": room(cfg, mech)}
