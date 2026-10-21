"""How a decanting cradle comes apart for transport, and how its pieces stack in a box.

**As modules** (:func:`module_pieces`: B, ``construction.cradle.PACK_POSES``, the user's of
2026-10-13; the cradle box on its own since the pack's turn 1 of 2026-10-14). It is cranked
to its pack pose first (:func:`pack_t`: the crank's arm along the drive side's outer plate's
rear leg, a mark seen without counting turns), then comes apart by hand into the base
board, the two side panels, the cradle box and its hand crank
(``construction.cradle.module_steps``):

1. the hand crank off (its thumb screw): nothing turns without it;
2. the cradle off the base board (the brackets' thumb knobs: the carriage bolts stay in the
   board, the washers and knobs back on them), by its outer plates' top members;
3. the cradle laid on its drive side on a towel, on its gearbox and the riser's two
   blocks (``construction.cradle.LAY_BLOCKS``: it lies level);
4. the clips of the cradle link's pins on the passive side out from the bay (B's X and Y:
   quick-release clevis pins, :mod:`construction.pivots.clevis`; they go back in the pins'
   holes);
5. the passive side panel lifted straight up off the pins, level, its pins with it (their
   heads on its links' outer faces, their TPU keepers on them), its links unable to swing
   (:func:`slide_hits` checks nothing is in the way of its move);
6. the drive side's two clips out, from the side;
7. the cradle box lifted straight up off the drive side's pins, level: each pin stays in
   its link, held by its keeper (``construction.cradle.KEEPER``: its head is under the
   link with nothing under it, and a bare pin would drop out); the drive side lies as it
   goes in the box, the worm holding its links as cranked. (Standing, the drive module
   tipped onto its cradle box once the pins cleared, and the panel fell the other way.)

It goes back together the other way (the payload's ``reassembly``): the drive side on its
blocks, the cradle box lowered onto its pins, its weight held until both are in (pin X, then
the box turned about it and b5 swung until pin Y lines up: never resting on a pin's tip,
which would push it out through its keeper), their clips in; the passive panel's links set
to the pose by paint marks (b3 taped to its bar while it turns over), the panel lowered
level onto the cradle link's holes the same way, the clips in, the cradle stood up onto the
board between its cleats, the riser back in its locator, the passive side's knobs snugged
at level, the first build's checks again. Each resting pose is rated (:func:`rest_poses`).

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
  over pads under it: ``voxpack.support``, each pad a block measured to the real gap,
  ``voxpack.measure_pads``);
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
        elif n.startswith(_SIDE_EXTRAS):
            if _on_board(p, through):
                put("base", "Base board", "base", n)     # a carriage bolt or its nut
            else:
                put(panel[host], f"{panel[host].capitalize()} side panel", "panel", n)
        elif n.startswith(RISER):
            put("risers", RISER_LABEL, "fill", n)
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
    """The base board, its cleats, its riser locator's layers and its TPU feet."""
    return name == "base_plate" or name.startswith(("base_cleat_", "riser_locator_",
                                                    "foot_boot"))


_SIDE_EXTRAS = ("foot_bracket",)
"""A side's own parts the cradle adds by name (a foot's bracket and its screws): each
travels with the side it's screwed to (its host)."""
RISER = "decanter_riser"
RISER_LABEL = "Riser (its blocks and plates)"
"""The decanter's printed riser (``construction.cradle.riser_pieces``): it travels as one
piece, its plates on its blocks (its blocks are the drive panel's lay-flat blocks on the way
apart and back together)."""


def module_pieces(parts: dict[str, PartInfo], connections, link: str, lk, fastened=()
                  ) -> tuple[list[Piece], list[tuple[str, list[str]]], list[str]]:
    """The modules a cradle that packs whole (``construction.cradle.PACK_POSES``) comes apart
    into, the steps that free them and the steps that put it back together
    (``construction.cradle.module_steps``): the base board (its feet, cleats and jack
    locator, its carriage bolts and their washers, nuts and thumb knobs); the passive side
    panel (every part of that side but its cradle link, its brackets: it lifts off the
    cradle link's pins, the cradle lying on its drive side, once their clips are out); the
    cradle box (both cradle links, the tie rods, the saddles and what is in them, the light,
    the base stop: it lifts off the drive side's pins once their clips are out, each pin
    held in its link by its keeper, ``construction.cradle.KEEPER``); the drive side panel
    (the gearbox and every part of that side but its cradle link and the hand crank: the
    worm holds its links as cranked, but its link b5, left on one pin, :func:`tuck`); the
    hand crank. ``lk`` the cradle's linkage; the rest as :func:`pieces`."""
    out: dict[str, Piece] = {}

    def put(key: str, label: str, role: str, name: str) -> None:
        out.setdefault(key, Piece(key, label, role)).parts.append(name)

    crank_side = next((s for s in SIDES if f"{s}.conn" in parts), "L")
    other = next(s for s in SIDES if s != crank_side)
    through = _through_board(parts)
    drive = ("drive", "Drive side panel", "panel")
    passive = ("passive", "Passive side panel", "panel")
    box = ("box", "Cradle box", "fill")
    risers = ("risers", RISER_LABEL, "fill")
    for n in sorted(parts):
        p = parts[n]
        s, stem = _side(n)
        if s == crank_side and stem.startswith("handle"):
            put("hand_crank", "Hand crank", "fill", n)
        elif s:
            put(*(box if stem == link else passive if s == other else drive), n)
        elif _board_part(n) or (n.startswith("foot_bracket") and _on_board(p, through)) or (
                lk.key in cr.BOARD_BRACKETS and re.fullmatch(r"foot_bracket_[A-Za-z0-9]+", n)):
            put("base", "Base board", "base", n)       # (a bracket bolted to the board)
        elif n.startswith(_SIDE_EXTRAS):
            host = _side(p.rigid_with or "")[0] or crank_side
            put(*(passive if host == other else drive), n)
        elif n.startswith(RISER):
            put(*risers, n)
        else:
            put(*box, n)
    for s, key in ((other, "passive"), (crank_side, "drive")):
        if key in out:
            _hanging(out, connections, link, s, key)
    pins = sorted({j for (_, a, ja), (_, b, jb) in connections
                   for mine, o, j in ((a, b, ja), (b, a, jb))
                   if mine == f"{other}.{link}" and o in parts and not is_frame(o)})
    breakdown, reassembly = cr.module_steps(lk, pins)
    frees = {"pose": ["risers"], "hand_crank": ["hand_crank"], "base": ["base"],
             "lift": ["passive"], "box": ["box", "drive"]}
    steps: list[tuple[str, list[str]]] = []
    for m in breakdown:
        text = m.words
        if m.tag == "base" and "base" in out:
            text = _base_step(out["base"].parts, parts, "the cradle", text)
        keys = [k for k in frees.get(m.tag, []) if k in out]
        if m.tag in frees and not keys and m.tag != "pose":
            continue                        # (no such piece: no step)
        steps.append((text, keys))
    for text, keys in steps:
        for k in keys:
            if k in out and not out[k].how:
                out[k].how = text
    return list(out.values()), steps, [m.words for m in reassembly]


def slide_hits(panel: dict, unit: dict, step: float = 0.5, way: float | None = None
               ) -> list[tuple[str, str, float]]:
    """What side panel ``panel`` (each part's placed shape, by name) meets as it slides
    straight off along z, away from the bay (its own side's way: the bay's middle is z 0;
    ``way``, +1 or -1, another way: the cradle box lifting off the drive side), against
    ``unit`` (what stays): each (panel part, unit part, how far it had slid, mm) at its first
    meeting. The panel goes in ``step`` mm steps (under the thinnest part either way), each
    part only as far as it takes to pass every unit part in its way (a box test first, then
    the solids')."""
    from build123d import Location

    from spiderpig.shapes import moved

    def box(sh) -> np.ndarray:
        bb = sh.bounding_box()
        return np.array([[bb.min.X, bb.min.Y, bb.min.Z], [bb.max.X, bb.max.Y, bb.max.Z]])

    pb = {n: box(sh) for n, sh in panel.items()}
    ub = {n: box(sh) for n, sh in unit.items()}
    if way is None:
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
        "B: the cradle box on the board as it stands, the hand crank in its bay, the drive "
        "panel over it, the passive panel on top",
        steps=(
            vp.Step("box", down="bottom", spin=0, corner="-x-z",
                    why="in first, standing as in use (its axis level, its saddles' open "
                        "tops up), on the board along its length, toward its -z side: 723 "
                        "mm from its cradle links' front pins to its rails' end, the box's "
                        "length"),
            vp.Step("hand_crank", into="box", region=(0.0, 0.03, 0.5, 0.48), after=("box",),
                    why="in the cradle box's bay, on the board between its two cradle "
                        "links, in front of the front saddle: caged by the box (the packing "
                        "review's m3; it had lain loose on the board beside the box, 134 mm "
                        "free to slide)"),
            vp.Step("drive", down="outer", spin=0, after=("box", "hand_crank"),
                    why="as it lay when the box came off, its outer plate down: its plates "
                        "over the box, the gearbox down beside it onto the board, on printed "
                        "spacers (the heaviest module low)"),
            vp.Step("passive", down="bay", spin=180, corner="+x+z", after=("drive",),
                    why="last, its bay face down over the drive panel's, turned end for end, "
                        "its links between the drive panel's, on printed spacers (a quarter "
                        "turn until the side frames' redesign of 2026-10-20: its inner "
                        "plate's bottom chord then lands on the drive panel's b2 and b3, "
                        "171 L)"),
        ),
        note="B's pieces since pack turn 1 (2026-10-14): the board (its four brackets on it "
             "since turn 2), the passive side panel, the cradle box, the drive side panel, "
             "the hand crank, the riser; the printed spacers where the pads were and beside "
             "what would slide (voxpack.spacers, turn 2). The box's length is the cradle "
             "box's (723 mm levelled, 41 mm longer than the board) and its height the board, "
             "the cradle box standing (178 mm; 249 since simplification round 2, b6's boss at "
             "X and its racking brace's pad at Br) and the two panels nested over it. Before "
             "turn 1 the drive side carried the cradle box, 441 mm across the bay whichever "
             "way it lay: the box was the board's footprint times that, about 211 L. (The other "
             "orders tried: the passive panel on the board, the drive panel bay to bay over "
             "it and the box on top, 182 L; the drive panel on the board under the passive, "
             "the box on top, 186 L; turn 2: the riser in the box's bay too, 158.5 L, the "
             "drive panel then over it.)"),
    "decanter_hinge": vp.Plan(
        "A: both side panels face down on the board, end to end, the loose parts in their "
        "open triangles",
        steps=(
            vp.Step("passive", down="outer",
                    why="its plates flat on the board, its pillars up (foam pads level it)"),
            vp.Step("drive", down="outer", spin=90,
                    why="turned a quarter: its plates lie across the passive panel's, their "
                        "pillars between each other's links, both on the board (end for end "
                        "until the side frames' redesign of 2026-10-20: its gearbox then "
                        "lands on the passive panel's wider outer plate, 107 L)"),
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
    ``slide_off`` (:func:`slide_hits`: the passive side panel off its cradle link's pins;
    ``box``: the cradle box off the drive side's)."""
    from spiderpig.config import BuildConfig
    from spiderpig.hardware import mass as mass_mod
    from spiderpig.mesh import tessellate_many

    cfg = config or BuildConfig(linkage=lk.key, robot=False)

    def part_mass_kg(b) -> float:          # (``cradle_report.part_mass_kg``'s)
        _, dens, fixed = mass_mod.material_of(b, cfg.sheet, "pla_filament", None)
        if fixed is not None:
            return fixed / 1000.0
        return mass_mod.part_props(b.placed_part()).volume * dens / 1e6

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
                         "volume": float(props.volume), "mass": float(part_mass_kg(b)),
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

        def clips(key: str) -> list[str]:
            return sorted(n for n in by[key].parts if n.endswith("_clip")
                          and n.rsplit(".pin_", 1)[-1].removesuffix("_clip") in on_link)

        # the passive panel off the cradle box and the drive side (up, away from the bay),
        # then the cradle box off the drive side (up: away from it)
        out_ = clips("passive")
        panel = {n: placed[n] for n in by["passive"].parts if n not in out_}
        box = {n: placed[n] for n in by["box"].parts}
        drive_out = clips("drive")
        drive = {n: placed[n] for n in by["drive"].parts if n not in drive_out}
        up = -1.0 if np.mean([placed[n].center().Z for n in drive]) > 0 else 1.0
        out["slide_off"] = {"hits": [list(h) for h in slide_hits(panel, {**box, **drive})],
                            "clips_out": out_,
                            "box": {"hits": [list(h) for h in slide_hits(box, drive,
                                                                         way=up)],
                                    "clips_out": drive_out}}
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


def pack_items(inp: dict, splits: dict[str, Sequence[str]] | None = None,
               relevel: Sequence[str] = ()
               ) -> tuple[list[Piece], list[tuple[str, list[str]]], list[vp.Item],
                          dict[str, np.ndarray]]:
    """The pieces of :func:`gather`'s input ``inp``, the steps that free them, each as a
    :class:`spiderpig.voxpack.Item` (its parts' meshes, a hanging link swung in: :func:`tuck`;
    a fill piece levelled: :func:`level`, but B's cradle box stood as in use, its axis level;
    a side's ``outer`` and ``bay`` faces, and the
    ``cradle box``'s where the piece carries it: its bay's), and each
    moved part's own 4 x 4 within its piece. ``splits``: a what-if (``spiderpig
    pack-inspect --split``), a new piece's key -> the patterns (``fnmatch``) of the parts it
    takes from theirs. A split changes no piece's orientation: each piece keeps the one it
    had whole, a new piece its parent's (the piece most of its mass came from: a plan's
    ``down`` names the same face, B's cradle box less its base stop still stands as in use),
    but the pieces ``relevel`` names (keys), levelled afresh."""
    pcs, steps, _ = _pieces_of(inp)
    if not splits:
        return (pcs, steps, *_items(inp, pcs))
    whole = {it.key: it.pre for it in _items(inp, _pieces_of(inp)[0])[0]}
    pcs, parent = _split(pcs, splits, {n: p["mass"] for n, p in inp["parts"].items()})
    pres = {pc.key: whole[parent.get(pc.key, pc.key)] for pc in pcs
            if pc.key not in relevel and parent.get(pc.key, pc.key) in whole}
    return (pcs, steps, *_items(inp, pcs, pres))


def _items(inp: dict, pcs: list[Piece], pres: dict[str, np.ndarray] | None = None
           ) -> tuple[list[vp.Item], dict[str, np.ndarray]]:
    """:func:`pack_items`' items of pieces ``pcs`` (each fill piece levelled, but those
    ``pres`` turns as it says) and the moved parts' own 4 x 4s."""
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
        sides = [_side(n)[0] for n in pc.parts if _side(n)[0]]
        box_alone = set(sides) == set(SIDES) and any(n.startswith("tie_") for n in pc.parts)
        pre = np.eye(3)
        if pres is not None and pc.key in pres:
            pre = pres[pc.key]
        elif box_alone:
            # the cradle box stands as in use, its axis level (the cradle's tilt undone), as
            # its "bottom" face says: not the least footprint's turn, which stood it on end
            # once b6's boss at X and the racking brace's pad made it squarer (simplification
            # round 2; until then the two had agreed)
            pre = vp.rot_about("z", ((-inp["tilt"] + 90) % 180) - 90)
        elif pc.role == "fill":
            pre = vp.rot_about("z", level([p for p, _ in ms], inp["tilt"]))
        faces = {}
        if box_alone:
            # the cradle box on its own (B's): a cradle link each side, as it stands in use
            drive_z = -1.0 if any(f"{s}.conn" in inp["parts"] for s in SIDES[:1]) else 1.0
            faces = {"drive link": pre @ np.array([0.0, 0.0, drive_z]),
                     "passive link": pre @ np.array([0.0, 0.0, -drive_z]),
                     "bottom": np.array([0.0, -1.0, 0.0])}
        elif sides and pc.role != "base":
            s = max(set(sides), key=lambda x: (sides.count(x), x))
            zs = np.concatenate([mesh[n][0][:, 2] for n in pc.parts if _side(n)[0] == s])
            out = np.array([0.0, 0.0, 1.0 if zs.mean() > 0 else -1.0])
            faces = {"outer": pre @ out, "bay": -(pre @ out)}
            if any(n.startswith("tie_") for n in pc.parts):     # (B's drive side's)
                faces["cradle box"] = faces["bay"]
        items.append(vp.Item(pc.key, pc.label, pc.role, ms, float(mass), np.asarray(c, float),
                             float(sum(inp["parts"][n]["volume"] for n in pc.parts)), pre,
                             faces, [n for n in pc.parts if len(mesh[n][1])]))
    return items, moved


def packed_meshes(items: Sequence[vp.Item], layout: vp.Layout) -> dict[str, list[vp.Mesh]]:
    """Each piece's meshes where ``layout`` puts it (the box's frame)."""
    by = {it.key: it for it in items}
    out = {}
    for p in layout.placed:
        out[p.key] = [(pos @ p.rot.T + p.shift, tri) for pos, tri in by[p.key].meshes]
    return out


def _split(pcs: list[Piece], splits: dict[str, Sequence[str]], mass: dict[str, float]
           ) -> tuple[list[Piece], dict[str, str]]:
    """``pcs`` with each split's parts (by pattern) moved into a piece of its own (a
    what-if: no step frees it); a piece left empty goes. And each new piece's parent: the
    piece the most of its ``mass`` (each part's, kg) came from."""
    import fnmatch

    out = []
    new = {k: Piece(k, f"What if: {k}", "fill", how="(a what-if)") for k in splits}
    took: dict[str, dict[str, float]] = {k: {} for k in splits}
    for pc in pcs:
        keep = []
        for n in pc.parts:
            k = next((k for k, pats in splits.items()
                      if any(fnmatch.fnmatchcase(n, p) for p in pats)), None)
            (new[k].parts if k is not None else keep).append(n)
            if k is not None:
                took[k][pc.key] = took[k].get(pc.key, 0.0) + mass.get(n, 0.0)
        pc.parts = keep
        pc.hanging = [(b, j) for b, j in pc.hanging if b in keep]
        if keep:
            out.append(pc)
    parent = {k: max(sorted(t), key=t.__getitem__) for k, t in took.items() if t}
    return out + [pc for pc in new.values() if pc.parts], parent


def pack_input(inp: dict, plan: vp.Plan | None | str = "default", cell: float = CELL,
               seq: Sequence[vp.Choice] | None = None,
               splits: dict[str, Sequence[str]] | None = None,
               relevel: Sequence[str] = ()) -> dict:
    """:func:`flat_pack`'s answer from :func:`gather`'s input ``inp``: packed by ``plan``
    (``"default"``: :data:`PACK_PLANS`' for the linkage; None: automatic), or by an explicit
    sequence of choices ``seq`` (``spiderpig pack-inspect --search``'s); ``splits`` a
    what-if, ``relevel`` the pieces it levels afresh (:func:`pack_items`)."""
    if plan == "default":
        plan = PACK_PLANS.get(inp["linkage"])
    assert plan is None or isinstance(plan, vp.Plan)
    pcs, steps, items, _ = pack_items(inp, splits, relevel)
    layout, packer = vp.pack(items, plan, cell, seq)
    return layout_payload(inp, pcs, steps, items, layout, packer.plan, cell)


def layout_payload(inp: dict, pcs: list[Piece], steps, items: Sequence[vp.Item],
                   layout: vp.Layout, plan: vp.Plan, cell: float = CELL) -> dict:
    """The payload (:func:`flat_pack`'s) of ``layout``: a piece's ``pads`` measured on the
    real meshes (``voxpack.measure_pads``); a layout its plan stopped (a step with no room)
    says so (``missing``: the pieces it didn't place, ``warnings``: why; its ``score`` is
    None), and is judged as what it is, part of a packing."""
    by = {pc.key: pc for pc in pcs}
    faces = {it.key: it.faces for it in items}
    mods = []
    for p in layout.placed:
        pc = by[p.key]
        mod = {"key": p.key, "label": pc.label, "how": pc.how, "parts": sorted(pc.parts),
               "matrix": np.round(p.matrix(), 6).tolist(), "rests": p.rest.how,
               "on": list(p.rest.on)}
        mod["down"], mod["spin"] = vp.describe(p.shape.ri, faces[p.key])
        if pc.moved:
            mod["part_matrix"] = {n: np.round(p.matrix() @ m, 6).tolist()
                                  for n, m in sorted(pc.moved.items())}
        mods.append(mod)
    # the pieces as a judge moves them (by the payload's own matrices: the same box to the
    # last bit), their pads as it sees them, and the printed spacers in their place
    packed = [module_packed(inp, mod) for mod in mods]
    lo, hi = vp.box_of(packed)
    inside = [float(v) for v in hi - lo]
    grid = vp.occupancy(packed, (lo, hi), cell)       # (the judge's grid: its cells)
    judged = {r["key"]: r for r in vp.metrics(packed, (lo, hi), cell, grid=grid)["pieces"]}
    kit = (vp.spacers(grid, [judged[m["key"]] for m in mods]) if not layout.missing
           else [])                     # (a packing its plan stopped: no spacers for it)
    _number(kit, len(mods))
    for mod in mods:
        mod["pads"] = [{k: v for k, v in q.items() if k != "piece"}
                       for q in judged[mod["key"]]["pads"]]
    vol = sum(it.volume for it in items)
    mass = sum(it.mass for it in items)
    com_y = (sum(it.mass * (p.rot @ it.com + p.shift)[1]
                 for it in items for p in layout.placed if p.key == it.key) / mass
             if mass > 0 else float("nan"))
    t = inp["t"]
    by_placed = {p.key for p in layout.placed}
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
        "spacers": [spacer_json(sp, by[sp.under].label if sp.under in by else
                                "Spacer " + sp.under.removeprefix("spacer_").lstrip("0"))
                    for sp in kit],
        "missing": list(dict.fromkeys([*layout.missing, *(it.key for it in items
                                                          if it.key not in by_placed)])),
        "warnings": list(layout.warnings),
        "score": round(layout.score, 2) if math.isfinite(layout.score) else None,
        "mode": "flat" if inp["turns"] is None else "modules",
        "pose": {"crank_t": round(t, 6), "clip_fraction": round(t / (2.0 * math.pi), 6),
                 "tilt_deg": round(inp["tilt"], 2), "turns_from_rest": inp["turns"]},
    }
    if inp["turns"] is not None:
        out["reassembly"] = _pieces_of(inp)[2]
    if inp.get("slide_off") is not None:
        out["slide_off"] = inp["slide_off"]
    return out


def _number(kit: list[vp.Spacer], n_pieces: int) -> None:
    """Spacers keyed ``spacer_01`` .. in the order they go in (their ranks among the
    pieces'), what they stand on or hold renamed with them."""
    order = sorted(range(len(kit)), key=lambda i: (kit[i].rank, i))
    names = {kit[i].key: f"spacer_{n:02d}" for n, i in enumerate(order, 1)}
    for sp in kit:
        sp.key = names[sp.key]
        sp.under = names.get(sp.under, sp.under)
        sp.on = tuple(names.get(o, o) for o in sp.on)
    kit.sort(key=lambda sp: sp.key)
    _ = n_pieces


def module_packed(inp: dict, mod: dict) -> vp.Packed:
    """A payload's piece ``mod`` where it put it (:class:`voxpack.Packed`): each part's mesh
    (of :func:`gather`'s input ``inp``) moved by the piece's matrix or its own, its mass,
    centre of mass and volume."""
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
    return vp.Packed(mod["key"], mod.get("label", mod["key"]), meshes, mass, com, vol, names)


SPACER_FILAMENT = "tpu95a_filament"
SPACER_FILL = 0.25
"""A printed spacer's filament (TPU 95A: it gives under a knock and grips what it holds)
and how much of its solid a print fills (two walls, 15 % gyroid): its mass."""


def spacer_json(sp: vp.Spacer, carries: str) -> dict:
    """A spacer in a pack's payload (``spacers``): :meth:`voxpack.Spacer.to_json`, its label
    (``Spacer 4a``) and words (what it carries and stands on, the pads it replaces) and its
    mass (kg, at :data:`SPACER_FILL`)."""
    from spiderpig.hardware.mass import filament_density

    d = sp.to_json()
    on = ", ".join(sp.on)
    d["label"] = "Spacer " + sp.key.removeprefix("spacer_").lstrip("0")
    d["words"] = (f"under the {carries.lower()} ({', '.join(sp.bears)}), on {on}; in place of "
                  f"pad{'s' if len(sp.pads) > 1 else ''} {', '.join(sp.pads)}")
    d["mass_kg"] = round(sp.volume / 1000.0 * filament_density(SPACER_FILAMENT) * SPACER_FILL
                         / 1000.0, 4)
    return d


def pack_kit(config, store=None) -> dict | None:
    """The pack of cradle ``config`` (:func:`pack_input` by its plan, from
    :func:`cached_input`): its payload, whose ``spacers`` the build prints and the guide
    places (:func:`box_steps`); None for a design with no pack plan."""
    if config.linkage not in PACK_PLANS:
        return None
    return pack_input(cached_input(config.linkage, store, config=config))


SPACER_STL_TURN = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
"""A spacer's print pose (:func:`spacer_stl`): as it goes in the box, the box's up (y) the
bed's (z)."""


def spacer_stl(d: dict, path: Path) -> list[float]:
    """A payload's spacer as a binary STL at ``path``, lying as it goes in the box, on the bed
    (its lowest at z 0, its corner at the origin); its size (x, y, z mm)."""
    from spiderpig.mesh import write_stl

    pos, tri = vp.Spacer.from_json(d).mesh()
    q = pos @ np.asarray(SPACER_STL_TURN).T
    q -= q.min(axis=0)
    write_stl(path, q, tri)
    return [round(float(v), 1) for v in q.max(axis=0)]


PART_WORDS = {"base_plate": "the board", "gearbox": "the gearbox", "frame_outer": "outer plate",
              "torso": "inner plate", "front_saddle_gauge": "the lip gauge",
              "base_stop_rail1_liner": "the base stop's rail", "risers": "the riser",
              "hand_crank": "the hand crank", "box": "the cradle box", "base": "the board",
              "drive": "the drive side panel", "passive": "the passive side panel"}
"""How :func:`part_words` names some parts and pieces."""


def part_words(name: str) -> str:
    """A part's or piece's name as the guide says it: ``L.gearbox`` -> ``the drive side's
    gearbox``, ``R.b6`` -> ``the passive side's link b6``, ``L.pin_X_clip`` -> ``the drive
    side's pin X's clip``."""
    side, stem = _side(name)
    whose = {"L": "the drive side's ", "R": "the passive side's ", "": ""}[side]
    if stem in PART_WORDS:
        w = PART_WORDS[stem]
        return (whose + w.removeprefix("the ")) if side else w
    if re.fullmatch(r"b\d+", stem):
        return f"{whose}link {stem}"
    if m := re.fullmatch(r"pin_([A-Za-z0-9]+)_(pin|clip)", stem):
        return f"{whose}pin {m.group(1)}" + ("'s clip" if m.group(2) == "clip" else "")
    if stem.startswith("spacer_"):
        return "spacer " + stem.removeprefix("spacer_")
    return (whose or "the ") + stem.replace("_", " ")


def box_steps(layout: dict) -> tuple[list[tuple[str, list[str]]], list[tuple[str, list[str]]]]:
    """Into the box and out of it, the guide's steps (each a title and its items) from a pack
    payload: each piece in its turn (:func:`in_order`), the spacers that go in before it
    first, an item each (on what it stands on: a support where the piece it carries comes
    down, its lips round what it bears on; a restraint against the side of the piece it
    holds), then the piece, its face down and where its plan puts it (the step's ``why`` to
    its first colon); out, the other way round, each piece and then the spacers that went in
    before it."""
    plan = PACK_PLANS.get(layout.get("linkage", ""))
    labels = {m["key"]: _the(m.get("label", m["key"])) for m in layout["modules"]}
    labels.update({d["key"]: d.get("label", d["key"]) for d in layout.get("spacers") or []})

    def on(names) -> str:
        return _join([labels.get(o, part_words(o)) if o != "floor" else "the carton's floor"
                      for o in names])

    into: list[tuple[str, list[str]]] = []
    waiting: list[dict] = []
    for kind, d in in_order(layout):
        if kind == "spacer":
            waiting.append(d)
            continue
        said = []
        for sp in waiting:
            if sp.get("why", "support").startswith("restraint"):
                said.append(f"{sp['label']} on {on(sp['on'])}, against "
                            f"{labels.get(sp['under'], part_words(sp['under']))}'s side "
                            "(where it goes, if it isn't in yet).")
            else:
                bears = _join([part_words(n) for n in sp.get("bears", ())])
                said.append(f"{sp['label']} on {on(sp['on'])}, under where {labels[d['key']]} "
                            f"comes down ({bears}): its lips seat round them.")
        waiting = []
        face = {"bottom": "standing as in use", "outer": "its outer plate down",
                "bay": "its bay face down"}.get(d.get("down", "")
                                                if d["key"] in ("box", "drive", "passive")
                                                else "", "")
        step = plan.step(d["key"]) if plan is not None else None
        where = (step.why.split(":")[0].split(" (")[0] if step is not None and step.why
                 else "")
        text = (("Then " if said else "") + labels[d["key"]]
                + (f", {where}" if where else f", {face}" if face else "")
                + (f", on {on(d.get('on', []))}" if d.get("on") and not where else "") + ".")
        into.append((d.get("label", d["key"]), [*said, text[0].upper() + text[1:]]))
    if waiting:
        into.append(("The last spacers", [_join([sp["label"] for sp in waiting]) + "."]))
    out: list[tuple[str, list[str]]] = []
    after: list[str] = []
    for kind, d in reversed(in_order(layout)):     # (a piece, then the spacers before it)
        if kind == "spacer":
            after.append(d["label"])
            continue
        if out and after:
            out[-1][1].append(f"Then {_join(after)}.")
        after = []
        out.append((d.get("label", d["key"]), [f"Lift {labels[d['key']]} straight up and out."]))
    if after and out:
        out[-1][1].append(f"Then {_join(after)}.")
    return into, out


def _the(label: str) -> str:
    """``Drive side panel`` -> ``the drive side panel``."""
    return "the " + label[0].lower() + label[1:]


def spacer_packed(d: dict) -> vp.Packed:
    """A payload's spacer (``spacers``) as a packed piece (:class:`voxpack.Packed`: its cells'
    surface, its mass at :data:`SPACER_FILL`)."""
    sp = vp.Spacer.from_json(d)
    pos, tri = sp.mesh()
    vol, cen = vp.mesh_volume(pos, tri)
    return vp.Packed(sp.key, d.get("label", sp.key), [(pos, tri)],
                     float(d.get("mass_kg", 0.0)), cen, vol, [sp.key])


def in_order(layout: dict) -> list[tuple[str, dict]]:
    """A payload's pieces and spacers in the order they go in (``("piece", module)`` or
    ``("spacer", spacer)``): each spacer by its ``rank`` among the pieces' places."""
    seq = [(float(i), 0, ("piece", m)) for i, m in enumerate(layout["modules"])]
    seq += [(float(d.get("rank", 0.0)), 1, ("spacer", d))
            for d in layout.get("spacers") or []]
    return [x for _, _, x in sorted(seq, key=lambda t: (t[0], t[1]))]


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
    ``rests``: on what, ``on``, directly or on ``pads`` (``voxpack.measure_pads``: each a
    block to the real gap, a cut list), or not; ``down`` and ``spin``: the face it lies on
    and its turn), the box (its inside and
    outside sizes, mm, and its inside's lower corner, ``floor``), the steps that take it
    apart, the ``plan``'s name, the true ``fill`` (the parts' own volumes over the box's
    inside), the mass centre's height over the floor, the ``unsupported`` pieces, the
    ``missing`` ones (a plan's step with no room stops it: ``warnings`` say why, ``score``
    None), ``mode``
    (``"flat"`` or ``"modules"``), the ``pose`` the transforms start from (its crank angle,
    its clip fraction ``t / 2 pi``, the bottle's tilt, the handle turns from rest) and, for
    modules, ``reassembly`` (the steps back together) and ``slide_off``: what the passive
    side panel meets coming off its cradle link's pins (:func:`slide_hits`, its clips out:
    none), and under ``box`` what the cradle box meets lifting off the drive side's."""
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


def cached_input(linkage: str, store=None, reuse_plan: bool = True, config=None) -> dict:
    """:func:`gather`'s input of ``linkage``'s cradle at its pack pose (:func:`pack_t`),
    from ``store``'s ``pack/`` when there (:func:`input_path`), else fabricated (through the
    store's fabrication cache) and kept: seconds once kept. ``reuse_plan``: plan the design
    through the store (:func:`spiderpig.stages.planning.plan_config`: its plan re-made, not
    searched for again; the design recorded there), not afresh. ``config``: the design (a
    ``BuildConfig`` of the cradle; default the linkage's own)."""
    import logging

    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.store import Store

    cfg = config or BuildConfig(linkage=linkage, robot=False)
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


# -- how each piece rests on the table on the way apart and back ------------------------------

G = 9.81


def resting(points: np.ndarray, com: np.ndarray, mass: float, down: np.ndarray,
            owner: Sequence[str] | None = None) -> dict:
    """How a rigid body (``points``, its centre of mass ``com``, ``mass`` kg) set down with
    ``down`` toward the table settles: on the stable face of its points' convex hull (the
    centre of mass over it) whose outward normal is nearest ``down``. That face's ``tilt``
    off ``down`` (degrees), the ``margin`` (mm) from the centre's foot to the face's nearest
    edge, the centre's height over it, the tip angle (``atan(margin / height)``), the push at
    the body's top that tips it (``m g margin / top``) and what touches (``on``: the
    ``owner`` of the face's points, by name)."""
    from scipy.spatial import ConvexHull, QhullError

    down = np.asarray(down, float) / np.linalg.norm(down)
    hull = ConvexHull(points)
    faces: list[tuple[np.ndarray, float]] = []
    for eq in hull.equations:               # each plane once (a face is many triangles)
        if not any(np.dot(n, eq[:3]) > 1 - 2e-6 and abs(off - eq[3]) < 0.05
                   for n, off in faces):
            faces.append((eq[:3].copy(), float(eq[3])))
    best = None
    for n, off in faces:
        h = -(n @ com + off)                     # the centre's height over the face
        if h <= 0:
            continue
        on = points[np.abs(points @ n + off) < 0.5]
        u = np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0])
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        q = np.column_stack([on @ u, on @ v])
        if len(q) < 3:
            continue
        try:
            h2 = ConvexHull(q)
        except QhullError:                       # (collinear: an edge, not a face)
            continue
        c2 = np.array([com @ u, com @ v])
        margin = float(np.min(-(h2.equations[:, :2] @ c2 + h2.equations[:, 2])))
        if margin <= 0:
            continue
        score = float(n @ down)
        if best is None or score > best[0]:
            best = (score, n, off, h, margin)
    if best is None:
        raise ValueError("no stable face: it can't rest")
    score, n, off, h, margin = best
    top = float(np.max(-(points @ n + off)))
    on_names: list[str] = []
    if owner is not None:
        on_names = sorted({owner[int(i)]
                           for i in np.flatnonzero(np.abs(points @ n + off) < 0.5)})
    return {"tilt_deg": round(math.degrees(math.acos(min(1.0, score))), 1),
            "margin_mm": round(margin, 1), "com_mm": round(float(h), 1),
            "tip_deg": round(math.degrees(math.atan2(margin, h)), 1),
            "push_n": round(mass * G * margin / max(top, 1.0), 1),
            "top_mm": round(top, 1), "on": on_names}


def lay_blocks(inp: dict, lk) -> list[np.ndarray]:
    """The riser's two blocks' points where they stand under the drive side's outer plate,
    the cradle lying on its drive side (``construction.cradle.LAY_BLOCKS``): each block, on
    its lift (``construction.cradle.RISER_LIFT``), moved from the riser (its meshes in
    ``inp``) to stand outboard of the plate's outer face, its boss against it, centred on its
    node (a post's screw head in its relief), its length along x."""
    nodes = cr.LAY_BLOCKS.get(lk.key, ())
    if not nodes:
        return []
    crank = next((s for s in SIDES if f"{s}.conn" in inp["parts"]), "L")
    lo, hi = (np.asarray(inp["parts"][f"{crank}.frame_outer"][k], float) for k in ("lo", "hi"))
    out_z = 1.0 if (lo[2] + hi[2]) / 2 > 0 else -1.0       # outboard along z
    face = hi[2] if out_z > 0 else lo[2]
    pts = lk.solve().evaluate(np.array([inp["t"]]))
    stand = cr.stand_of(lk)
    mouth = pts[cr.mouth_name(lk)][0]
    out = []
    names = sorted(n for n in inp["meshes"] if n.startswith(RISER + "_block"))
    for node, name in zip(nodes, names, strict=False):
        if not isinstance(node, str):           # (x, y) from the mouth
            at = mouth + np.asarray(node, float)
        else:
            at = (mouth + np.asarray(stand.nodes[node], float) if node in stand.nodes
                  else pts[node][0])
        p = np.asarray(inp["meshes"][name][0], float)
        c = (p.min(axis=0) + p.max(axis=0)) / 2
        top = p[:, 1].max()
        lift = name.replace("_block", "_lift")    # its lift, under it in the riser: with it
        if lift in inp["meshes"]:
            p = np.concatenate([p, np.asarray(inp["meshes"][lift][0], float)])
        q = p - [c[0], top, c[2]]               # its boss's top's middle at the origin
        # its height (y) outboard along the plate's normal, its width (z) along y, its
        # length (x) along x: the boss up against the face
        q = np.column_stack([q[:, 0], q[:, 2], -q[:, 1] * out_z])
        out.append(q + np.array([at[0], at[1], face]))
    return out


def rest_poses(inp: dict, lk) -> list[dict]:
    """Each of the guide's resting poses of cradle ``lk`` (``construction.cradle.REST_POSES``)
    rated by :func:`resting` from :func:`gather`'s input ``inp`` (its pack pose): the pieces'
    parts' meshes, masses and centres, set down drive side down, passive side down, the
    cradle box levelled and turned over (its saddles' tops down), or standing on its feet (as
    on its board), on the riser's blocks where the pose says (:func:`lay_blocks`; their own
    weight left out)."""
    poses = cr.REST_POSES.get(lk.key, ())
    if not poses:
        return []
    pcs, _, _ = _pieces_of(inp, lk)
    by = {pc.key: pc for pc in pcs}
    crank = next((s for s in SIDES if f"{s}.conn" in inp["parts"]), "L")
    lo, hi = (np.asarray(inp["parts"][f"{crank}.frame_outer"][k], float) for k in ("lo", "hi"))
    drive_out = np.array([0.0, 0.0, 1.0 if (lo[2] + hi[2]) / 2 > 0 else -1.0])
    level = vp.rot_about("z", -inp["tilt"])         # the box levelled (as it packs)
    rows = []
    for rest in poses:
        keys = ([k for k in by if k not in ("base", "hand_crank", "risers")]
                if rest.pieces == ("*",) else list(rest.pieces))
        names = [n for k in keys for n in by[k].parts]
        mass = sum(inp["parts"][n]["mass"] for n in names)
        com = np.sum([inp["parts"][n]["mass"] * np.asarray(inp["parts"][n]["com"], float)
                      for n in names], axis=0) / mass
        pts = [np.asarray(inp["meshes"][n][0], float) for n in names]
        owner = [n for n, p in zip(names, pts, strict=True) for _ in range(len(p))]
        if rest.blocks:
            blocks = lay_blocks(inp, lk)
            pts += blocks
            owner += [f"riser block {i}" for i, b in enumerate(blocks) for _ in range(len(b))]
        down = {"drive": drive_out, "passive": -drive_out,
                "top": level.T @ np.array([0.0, 1.0, 0.0]),
                "feet": np.array([0.0, -1.0, 0.0])}[rest.down]
        got = resting(np.concatenate(pts), com, mass, down, owner)
        rows.append({"pose": rest.key, "words": rest.words, "kg": round(mass, 2), **got})
    return rows
