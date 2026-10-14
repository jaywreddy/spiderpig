"""How a decanting cradle comes apart for transport, and how its pieces stack in a box.

**As modules** (:func:`module_pieces`: B, :data:`PACK_TURNS`, the user's of 2026-10-13). It
is cranked to its pack pose first (:func:`pack_t`), then comes apart by hand into three
modules and its hand crank:

1. the cradle off the base board (the brackets' thumb nuts: the carriage bolts stay in the
   board, the nuts back on them), both sides and the cradle box together;
2. the clips of the cradle link's pins on the passive side out (B's X and Y: quick-release
   clevis pins, :mod:`construction.pivots.clevis`; they go back in the pins' holes);
3. the passive side panel slid straight off its cradle link, outward, its pins with it
   (:func:`slide_hits` checks nothing is in its way); the drive side and the cradle box stay
   one unit, the worm holding them as cranked;
4. the hand crank off (its thumb screw).

The other bottles' adapters and collars travel loose round them.

**Flat** (:func:`pieces`: A, whose cradle link rides pillar G between its side's plates).
The cradle comes apart where it is already fastened, with no new joint, in the order
:func:`pieces` gives as its steps:

1. the bottle adapters out of the saddles (every adapter travels loose);
2. the hand crank off the gearbox's input shaft (its thumb screw);
3. the cradle off the base board (the brackets' thumb nuts: the carriage bolts stay in the
   board);
4. each side's inner plate off (its screws from the bay side, back in its holes);
5. the cradle link's own joints with its side (``mech.connections``): a pivot pin's upper
   E-ring off (the pin stays in the side's links, the ring back on it for the trip), a
   clevis pin's clip out, or a Chicago screw out of its barrel (``chicago6``: it goes back
   into the barrel, which stays in the side's link), a pillar slid out of the link's bore;
   each side panel then lifts off, leaving the cradle box whole;
6. the cradle box apart: the base stop and the light off their saddles, one cradle link
   off the tie rods (their screws), the saddles slid off the rods (their set screws), the
   other link off. Each tie rod travels with its two screws back in its ends, each cradle
   link with the loose spacers that sat over it.

A side's link left hanging on one joint once its cradle link is off (B's b5 on J2) swings
about it into its panel (:func:`tuck`).

**How it stacks** (:func:`pack`, on per-part bounding boxes, each grown by half of
:data:`CLEAR`):

* every piece but the panels and the base board is first *levelled*: turned about z (the
  mechanism's plane normal) to the angle, by the degree (and the cradle's own tilt), that
  gives its xy bounding box the least area;
* the base board stays where it is, at the bottom; the box's footprint is centred on it and
  as large as the largest piece lying flat;
* each side panel is dropped onto a heightmap of what is in (a rectangle max-filter per
  part), both orders, face up or down, either way round (and quarter turns where it fits);
* the other pieces, largest first, each in its six axis orientations, go to the lowest free
  slot in a voxel grid of the box (:data:`CELL` across, :data:`CELL_Y` up; a 3-D
  integral-volume box sum per part), cavities under a panel's frame included; the slot
  that raises the box least wins, then the lowest, the flattest, the one laid along the
  box, the one nearest a corner. A slot that would put a piece both over and under another
  (it couldn't be put in) is refused;
* the best few panel arrangements (:data:`ARRANGEMENTS`) are each filled, the lowest box
  kept; the pieces go in the box in the order the stack allows, bottom up (a topological
  sort: what is under a piece goes in first).

:func:`flat_pack` returns each piece (``modules``: its parts and its 4 x 4 transform from the
assembled pose to the packed one), in that order, with the box and the steps: the viewer's
pack (``/api/cradle/{key}/pack``, ``viewer/decant.html``) flies each one there. Every pose
is the fabricated cradle's at its pack pose (``pose``: crank angle 0, the baked clip's first
frame, for a flat pack; B's, 4 handle turns from rest).
"""

from __future__ import annotations

import contextlib
import math
import re
from collections.abc import Sequence
from dataclasses import dataclass, field
from itertools import permutations, product

import numpy as np

from spiderpig.construction import cradle as cr
from spiderpig.mechanism import is_frame, is_link

CLEAR = 3.0         # between two pieces and from a piece to the box's wall (foam, cardboard)
WALL = 5.0          # the box's corrugated wall
CELL = 5.0          # the packing grid across the box (x, z), mm
CELL_Y = 2.0        # and up it
ARRANGEMENTS = 3    # the side panels' best arrangements, each filled (the lowest box kept)
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
    """A fabricated part as :func:`pieces` and :func:`pack` read it: its body's ``fab``,
    ``bom_key`` and ``rigid_with`` host, and ``points`` (the assembled pose): its bounding
    box's corners, or its tessellation's vertices where it may turn by any angle."""

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


def _bottle_size(key: str) -> str:
    return next((s.name for s in cr.SIZES if s.key == key), key)


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
        if n == "base_plate" or n.startswith("foot_boot"):
            put("base", "Base board", "base", n)
        elif n.startswith("foot_bracket"):
            if _on_board(p, through):
                put("base", "Base board", "base", n)     # a carriage bolt or its nut
            else:
                put(panel[host], f"{panel[host].capitalize()} side panel", "panel", n)
        elif m := re.fullmatch(r"((?:base_|bay_)?tie_[A-Za-z0-9]+)(?:_screw[LR])?", n):
            put(m.group(1), f"Tie rod {m.group(1).removeprefix('tie_')}", "fill", n)
        elif n in saddles or any(n == f"{sd}_liner" or n.startswith(f"{sd}_set_screw")
                                 for sd in saddles):
            sd = next(sd for sd in saddles if n.startswith(sd))
            put(sd, f"{sd.replace('_', ' ').capitalize()} and its liner", "fill", n)
        elif n.endswith(("_liner", "_collar")):
            if m := re.fullmatch(r"(\w+_saddle)_(\w+)_liner", n):
                label = (f"Adapter: {_bottle_size(m.group(2))}, "
                         f"{m.group(1).replace('_', ' ')}")
            else:
                label = f"Collar: {_bottle_size(n.removesuffix('_collar'))}"
            put(n, label, "fill", n)
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
    adapters = [k for k, pc in out.items() if pc.label.startswith(("Adapter", "Collar"))]
    if adapters:
        steps.append(("Bottle adapters: lift those in the saddles out. Every adapter and "
                      "collar travels loose.", adapters))
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


PACK_TURNS: dict[str, float] = {"decanter_rcm": 4.0}
"""The cradles that pack as modules (:func:`module_pieces`), each one's pack pose: the
handle turns from rest (the bottle neck up) it is cranked to first. B (2026-10-13, the
user's three modules): its passive side's cradle link b6 overlaps that side's inner bar
O1-O2 at rest, by about 2 mm until the bottle is 46° neck up; past that it clears it (14.6
mm at 35°, 4 turns from rest; 38 mm at full tilt), so the side slides straight off it. Four
turns, not full tilt, as the box goes: at full tilt the cradle box stands up out of the side
(a 259 L box against about 190). A (not here) packs flat: its cradle link rides pillar G
between its side's plates."""


def pack_t(config) -> float:
    """The crank angle (radians) a cradle packs at: :data:`PACK_TURNS` handle turns past rest
    (the worm's ratio a crank turn), else 0 (the flat pack: the baked clip's first frame)."""
    from spiderpig.construction.handcrank import NRV040

    turns = PACK_TURNS.get(config.linkage)
    if turns is None:
        return 0.0
    lk = config.lk
    ts = np.linspace(0.0, 2.0 * math.pi, 2880, endpoint=False)
    pts = lk.solve().evaluate(ts)
    o, ax = cr.mouth_frame(lk)
    d = pts[ax] - pts[o]
    rest = float(ts[int(np.argmin(np.unwrap(np.arctan2(d[:, 1], d[:, 0]))))])
    return float((rest + turns * 2.0 * math.pi / NRV040.ratio) % (2.0 * math.pi))


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
    """A bracket's carriage bolt or its nut (on a bolt's axis): it stays with the board."""
    axis = (p.lo + p.hi)[[0, 2]] / 2
    return p.fab == "purchased" and any(np.hypot(*(axis - t)) < 1.0 for t in through)


def module_pieces(parts: dict[str, PartInfo], connections, link: str, turns: float,
                  tilt: float, fastened=()) -> tuple[list[Piece], list[tuple[str, list[str]]]]:
    """The modules a cradle that packs whole (:data:`PACK_TURNS`) comes apart into, and the
    steps that free them: the base board (its feet, its carriage bolts and their thumb nuts);
    the passive side panel (every part of that side but its cradle link, its brackets: it
    slides off the cradle link once the link's pins' clips are out); the drive side with the
    cradle box (both cradle links, the tie rods, the saddles and what is in them, the light,
    the base stop: the worm holds it as it was cranked); the hand crank; the spare bottle
    sizes' adapters and collars, loose. ``turns`` the pack pose's handle turns from rest,
    ``tilt`` the bottle's there (degrees); the rest as :func:`pieces`."""
    out: dict[str, Piece] = {}

    def put(key: str, label: str, role: str, name: str) -> None:
        out.setdefault(key, Piece(key, label, role)).parts.append(name)

    crank_side = next((s for s in SIDES if f"{s}.conn" in parts), "L")
    other = next(s for s in SIDES if s != crank_side)
    through = _through_board(parts)
    held_by = {a for a, _ in fastened}
    drive = ("drive", "Drive side with the cradle box", "panel")
    passive = ("passive", "Passive side panel", "panel")
    for n in sorted(parts):
        p = parts[n]
        s, stem = _side(n)
        spare = re.fullmatch(r"(\w+_saddle)_(\w+)_liner", n)
        if s == crank_side and stem.startswith("handle"):
            put("hand_crank", "Hand crank", "fill", n)
        elif s:
            put(*(passive if s == other and stem != link else drive), n)
        elif n == "base_plate" or n.startswith("foot_boot") or (
                n.startswith("foot_bracket") and _on_board(p, through)):
            put("base", "Base board", "base", n)
        elif n.startswith("foot_bracket"):
            host = _side(p.rigid_with or "")[0] or crank_side
            put(*(passive if host == other else drive), n)
        elif (spare or n.endswith("_collar")) and n not in held_by:
            # a spare size's adapter or collar (lying on the table): it travels loose
            label = (f"Adapter: {_bottle_size(spare.group(2))}, "
                     f"{spare.group(1).replace('_', ' ')}" if spare
                     else f"Collar: {_bottle_size(n.removesuffix('_collar'))}")
            put(n, label, "fill", n)
        else:
            put(*drive, n)
    if "passive" in out:
        _hanging(out, connections, link, other, "passive")
    pins = sorted({j for (_, a, ja), (_, b, jb) in connections
                   for mine, o, j in ((a, b, ja), (b, a, jb))
                   if mine == f"{other}.{link}" and o in parts and not is_frame(o)})
    clips = [n for j in pins if (n := f"{other}.pin_{j}_clip") in parts]
    steps: list[tuple[str, list[str]]] = [
        (f"Crank it to the pack pose: to rest (the bottle neck up, where the cradle stops "
         f"coming back), then {turns:g} handle turns on, the bottle {abs(tilt):.0f}° "
         f"{'neck up' if tilt < 0 else 'neck down'}. There the passive side's cradle link "
         "clears that side's inner bar, so the side slides straight off it.", [])]
    if "base" in out:
        steps.append((_base_step(out["base"].parts, parts,
                                 "the cradle (both sides and the cradle box together)"),
                      ["base"]))
    if pins:
        what = (f"Pin{'s' if len(pins) > 1 else ''} {_join(pins)} (the cradle link's, on the "
                "passive side): ")
        steps.append((what + (_clips_out(len(clips)) if clips else
                              "take each one's retainer off from the bay side."), []))
    if "passive" in out:
        steps.append(("Passive side panel: slide it straight off its cradle link, outward (its "
                      "pins come out of the link with it). The drive side and the cradle box "
                      "stay one unit: the worm holds them where they were cranked to.",
                      ["passive", "drive"]))
    if "hand_crank" in out:
        steps.append((_hand_crank_step(out["hand_crank"].parts), ["hand_crank"]))
    spares = [k for k, pc in out.items() if pc.label.startswith(("Adapter", "Collar"))]
    if spares:
        steps.append(("The other bottles' adapters and collars travel loose round the "
                      "modules.", spares))
    for text, keys in steps:
        for k in keys:
            if k in out and not out[k].how:
                out[k].how = text
    return list(out.values()), steps


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


def _base_step(names: Sequence[str], parts: dict[str, PartInfo], what: str) -> str:
    """How ``what`` comes off the base board: the nuts on its brackets' carriage bolts."""
    nuts = [n for n in names if "nut" in (parts[n].bom_key or "")]
    size = next((z for n in nuts if (z := _size(parts[n].bom_key))), "")
    thumb = all("thumb" in (parts[n].bom_key or "") for n in nuts)
    kind = "thumb nuts" if thumb else "nuts"
    how = " by hand" if thumb else ""
    return (f"Base board: undo the {len(nuts)} {size} {kind} on the brackets' feet{how} and "
            f"lift {what} off it. The carriage bolts stay in the board, their nuts back on.")


def _clips_out(n: int) -> str:
    many = f"pull each one's hairpin cotter (R-clip, {n})" if n > 1 else "pull its R-clip"
    return (f"{many} from the bay side, by hand. The pin stays in the side's links; the clip "
            "goes back in its hole for the trip.")


# -- the packing ---------------------------------------------------------------------------


def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    c, s = round(c, 15), round(s, 15)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def _aabbs(points: Sequence[np.ndarray], rot: np.ndarray) -> np.ndarray:
    """Each part's bounding box after ``rot``: ``(n, 2, 3)`` (min, max)."""
    out = np.empty((len(points), 2, 3))
    for i, p in enumerate(points):
        q = p @ rot.T
        out[i, 0], out[i, 1] = q.min(axis=0), q.max(axis=0)
    return out


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


def _axis_rotations() -> list[np.ndarray]:
    """The six ways an axis-aligned box lies: a proper rotation per permutation of its
    axes."""
    out = []
    for perm in permutations(range(3)):
        m = np.zeros((3, 3))
        for i, j in enumerate(perm):
            m[j, i] = 1.0
        if np.linalg.det(m) < 0:
            m[2] = -m[2]             # (flip what goes across the box: keeps it proper)
        out.append(m)
    return out


@dataclass
class Placed:
    """A piece in the box: ``rot`` and ``shift`` (packed = rot @ assembled + shift), its
    parts' packed bounding boxes ``boxes`` (``(n, 2, 3)``)."""

    key: str
    rot: np.ndarray
    shift: np.ndarray
    boxes: np.ndarray

    def matrix(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3] = self.rot
        m[:3, 3] = self.shift
        return m


def _merged(boxes: np.ndarray, waste: float = 0.04) -> np.ndarray:
    """Fewer boxes covering the same parts, for the grid's queries: greedily, the two whose
    union wastes least merged while it wastes under ``waste`` of what they cover (a box
    inside another is merged first: it wastes nothing)."""
    bs = np.asarray(boxes, float).copy()
    while len(bs) > 1:
        lo = np.minimum(bs[:, None, 0], bs[None, :, 0])
        hi = np.maximum(bs[:, None, 1], bs[None, :, 1])
        il = np.maximum(bs[:, None, 0], bs[None, :, 0])
        ih = np.minimum(bs[:, None, 1], bs[None, :, 1])
        v = np.prod(bs[:, 1] - bs[:, 0], axis=-1)
        cover = v[:, None] + v[None, :] - np.prod(np.clip(ih - il, 0.0, None), axis=-1)
        extra = np.prod(hi - lo, axis=-1) - cover
        np.fill_diagonal(extra, np.inf)
        extra = np.where(extra <= waste * cover + 1e-6, extra, np.inf)
        k = int(np.argmin(extra))
        i, j = divmod(k, len(bs))
        if not np.isfinite(extra[i, j]):
            break
        bs[i] = np.stack([lo[i, j], hi[i, j]])
        bs = np.delete(bs, j, axis=0)
    return bs


def _turn(boxes: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Axis-aligned ``boxes`` (``(n, 2, 3)``) turned by ``rot``, a quarter-turn rotation
    (their bounding boxes stay exact)."""
    a, b = boxes[:, 0] @ rot.T, boxes[:, 1] @ rot.T
    return np.stack([np.minimum(a, b), np.maximum(a, b)], axis=1)


class _Grid:
    """The box's packing grid: the heightmap (``top``, per x-z cell) the panels drop onto,
    the voxel occupancy the rest fills (``occ``, :data:`CELL` x :data:`CELL_Y` x
    :data:`CELL`), both from every placed part's grown box."""

    def __init__(self, x0: float, y0: float, z0: float, nx: int, nz: int, ny: int):
        self.o = np.array([x0, y0, z0])
        self.nx, self.nz, self.ny = nx, nz, ny
        self.top = np.full((nx, nz), y0)
        self.occ = np.zeros((nx, ny, nz), np.uint8)
        self._sum: np.ndarray | None = None

    def cells(self, box: np.ndarray) -> tuple[int, int, int, int, int, int]:
        """The cells a grown box touches (conservative), clipped to the grid."""
        g = np.array([CELL, CELL_Y, CELL])
        lo = np.floor((box[0] - self.o) / g + 1e-6).astype(int)
        hi = np.ceil((box[1] - self.o) / g - 1e-6).astype(int)
        lo = np.clip(lo, 0, [self.nx, self.ny, self.nz])
        hi = np.clip(hi, 0, [self.nx, self.ny, self.nz])
        return int(lo[0]), int(hi[0]), int(lo[1]), int(hi[1]), int(lo[2]), int(hi[2])

    def grow(self, ny: int) -> None:
        if ny > self.ny:
            self.occ = np.concatenate([self.occ, np.zeros((self.nx, ny - self.ny, self.nz),
                                                          np.uint8)], axis=1)
            self.ny = ny
            self._sum = None

    def add(self, grown: np.ndarray) -> None:
        for b in grown:
            need = math.ceil((b[1, 1] - self.o[1]) / CELL_Y - 1e-6)
            self.grow(need)
            i0, i1, j0, j1, k0, k1 = self.cells(b)
            self.top[i0:i1, k0:k1] = np.maximum(self.top[i0:i1, k0:k1], b[1, 1])
            if i1 <= i0 or j1 <= j0 or k1 <= k0:
                continue
            self.occ[i0:i1, j0:j1, k0:k1] = 1
            if self._sum is not None:       # the integral, the box's own prefix sums added
                cx = np.clip(np.arange(i0, self.nx + 1) - i0, 0, i1 - i0).astype(np.int32)
                cy = np.clip(np.arange(j0, self.ny + 1) - j0, 0, j1 - j0).astype(np.int32)
                cz = np.clip(np.arange(k0, self.nz + 1) - k0, 0, k1 - k0).astype(np.int32)
                self._sum[i0:, j0:, k0:] += cx[:, None, None] * cy[None, :, None] * cz
                # (a cell counted twice only makes a sum that was already > 0 larger)

    def integral(self) -> np.ndarray:
        """The occupancy's 3-D prefix sums, ``(nx + 1, ny + 1, nz + 1)``."""
        if self._sum is None:
            s = np.zeros((self.nx + 1, self.ny + 1, self.nz + 1), np.int32)
            s[1:, 1:, 1:] = self.occ.cumsum(0, dtype=np.int32).cumsum(1).cumsum(2)
            self._sum = s
        return self._sum


def _win_max(h: np.ndarray, w: int, d: int) -> np.ndarray:
    """``out[u, v] = h[u:u+w, v:v+d].max()``."""
    from numpy.lib.stride_tricks import sliding_window_view

    a = sliding_window_view(h, w, axis=0).max(axis=-1)
    return sliding_window_view(a, d, axis=1).max(axis=-1)


@dataclass
class _Body:
    """A piece's parts in its own frame (after ``pre``: the panels' none, the others'
    :func:`level` turn): their boxes and, grown by half of :data:`CLEAR` and merged, the
    boxes the grid is asked about."""

    pre: np.ndarray
    boxes: np.ndarray
    query: np.ndarray

    @classmethod
    def of(cls, points: Sequence[np.ndarray], pre: np.ndarray) -> _Body:
        own = _aabbs(points, pre)
        c = CLEAR / 2
        return cls(pre, own, _merged(own + np.array([-c, c])[:, None]))


@dataclass
class _Shape:
    """A piece in one orientation: ``rot`` (from the assembled pose), its parts' own boxes,
    their grown union's corner ``lo`` and size, the query boxes in cells from that corner
    (``cells``: a0, a1, b0, b1, c0, c1 along x, y, z) and each one's bottom over it."""

    rot: np.ndarray
    boxes: np.ndarray
    lo: np.ndarray
    size: np.ndarray
    cells: np.ndarray
    ylo: np.ndarray
    span: tuple[int, int, int]  # its size in cells

    @classmethod
    def of(cls, body: _Body, turn: np.ndarray) -> _Shape:
        own = _turn(body.boxes, turn)
        q = _turn(body.query, turn)
        lo, hi = q[:, 0].min(axis=0), q[:, 1].max(axis=0)
        q = q - lo
        g = np.array([CELL, CELL_Y, CELL])
        a = np.floor(q[:, 0] / g + 1e-6).astype(int)
        b = np.maximum(np.ceil(q[:, 1] / g - 1e-6).astype(int), a + 1)
        span = np.ceil((hi - lo) / g - 1e-6).astype(int)
        cells = np.stack([a[:, 0], b[:, 0], a[:, 1], b[:, 1], a[:, 2], b[:, 2]], axis=1)
        return cls(turn @ body.pre, own, lo, hi - lo, cells, q[:, 0, 1],
                   (int(span[0]), int(span[1]), int(span[2])))

    def placed(self, key: str, at: np.ndarray) -> Placed:
        """At ``at``: where its grown union's lower corner goes."""
        shift = at - self.lo
        return Placed(key, self.rot, shift, self.boxes + shift)


class _Order:
    """Which piece must go in before which (a part over another's, in plan): the DAG the
    insertion order sorts, kept transitively closed."""

    TOL = 0.05

    def __init__(self) -> None:
        self.placed: list[Placed] = []
        self.reach = np.zeros((0, 0), bool)
        self._boxes = np.zeros((0, 2, 3))           # every placed part's, and its piece
        self._owner = np.zeros(0, int)

    def relations(self, boxes: np.ndarray) -> tuple[set[int], set[int]] | None:
        """(the pieces that must go in before ``boxes``, those after), or None when one is
        both over and under them, or the two can't be ordered at all."""
        t, b = self.TOL, self._boxes
        ov = ((boxes[:, None, 1, 0] > b[None, :, 0, 0] + t)
              & (b[None, :, 1, 0] > boxes[:, None, 0, 0] + t)
              & (boxes[:, None, 1, 2] > b[None, :, 0, 2] + t)
              & (b[None, :, 1, 2] > boxes[:, None, 0, 2] + t))
        under = ((boxes[:, None, 1, 1] <= b[None, :, 0, 1] + t) & ov).any(axis=0)
        above = ((boxes[:, None, 0, 1] >= b[None, :, 1, 1] - t) & ov).any(axis=0)
        after = {int(i) for i in np.unique(self._owner[under])}
        before = {int(i) for i in np.unique(self._owner[above])}
        if before & after:
            return None
        if any(self.reach[a, b] for a in after for b in before):
            return None
        return before, after

    def add(self, p: Placed, rel: tuple[set[int], set[int]]) -> None:
        n = len(self.placed)
        self._boxes = np.concatenate([self._boxes, p.boxes])
        self._owner = np.concatenate([self._owner, np.full(len(p.boxes), n)])
        r = np.zeros((n + 1, n + 1), bool)
        r[:n, :n] = self.reach
        before, after = rel
        for b in before:
            r[b, n] = True
        for a in after:
            r[n, a] = True
        for k in range(n + 1):              # (closure through the new piece)
            r |= r[:, k:k + 1] & r[k:k + 1, :]
        self.reach = r
        self.placed.append(p)

    def order(self) -> list[int]:
        """Bottom up: of the pieces whose predecessors are in, the lowest first."""
        n = len(self.placed)
        low = [float(p.boxes[:, 0, 1].min()) for p in self.placed]
        done: list[int] = []
        left = set(range(n))
        while left:
            ready = [i for i in left if not any(self.reach[j, i] for j in left if j != i)]
            i = min(ready, key=lambda i: (low[i], i))
            done.append(i)
            left.remove(i)
        return done


@dataclass
class Packed:
    """:func:`pack`'s answer: each piece's placement, the insertion order (keys, bottom up),
    the box's inside (``(2, 3)``: its lower corner and upper)."""

    placed: dict[str, Placed]
    order: list[str]
    inside: np.ndarray


def _fill(grid: _Grid, order: _Order, fills, best: float) -> float | None:
    """Fill ``fills`` (key, its orientations, the flattest first) into ``grid`` in turn, each
    to its best free slot (the module's docstring); the box's inside top once done, or None
    once it can't beat ``best``."""
    top = max((float(p.boxes[:, 1, 1].max()) + CLEAR for p in order.placed),
              default=grid.o[1])
    for key, shapes in fills:
        while True:
            cands = []
            s = grid.integral()
            for rank, sh in enumerate(shapes):
                pw, ph, pd = sh.span
                nu, nw, nv = grid.nx - pw + 1, grid.ny - ph + 1, grid.nz - pd + 1
                if min(nu, nw, nv) <= 0:
                    continue
                tot = np.zeros((nu, nw, nv), np.int32)     # what each slot's boxes hold
                for a0, a1, b0, b1, c0, c1 in sh.cells:
                    tot += s[a1:a1 + nu, b1:b1 + nw, c1:c1 + nv]
                    tot -= s[a0:a0 + nu, b1:b1 + nw, c1:c1 + nv]
                    tot -= s[a1:a1 + nu, b0:b0 + nw, c1:c1 + nv]
                    tot -= s[a1:a1 + nu, b1:b1 + nw, c0:c0 + nv]
                    tot += s[a0:a0 + nu, b0:b0 + nw, c1:c1 + nv]
                    tot += s[a0:a0 + nu, b1:b1 + nw, c0:c0 + nv]
                    tot += s[a1:a1 + nu, b0:b0 + nw, c0:c0 + nv]
                    tot -= s[a0:a0 + nu, b0:b0 + nw, c0:c0 + nv]
                free = tot == 0
                has = free.any(axis=1)
                if not has.any():
                    continue
                w = free.argmax(axis=1)
                u, v = np.nonzero(has)
                w = w[u, v]
                y = grid.o[1] + w * CELL_Y
                ytop = y + sh.size[1]
                gap_x = np.minimum(u, nu - 1 - u)
                gap_z = np.minimum(v, nv - 1 - v)
                cands.append((np.maximum(top, ytop + CLEAR / 2), y, ytop,
                              np.full(len(u), rank), gap_x + gap_z, u, w, v, rank))
            if cands:
                break
            grid.grow(grid.ny + math.ceil(min(sh.size[1] for sh in shapes) / CELL_Y) + 1)
        cat = [np.concatenate([c[i] for c in cands]) for i in range(8)]
        keys = (cat[4], cat[3], np.round(cat[2], 3), np.round(cat[1], 3), np.round(cat[0], 3))
        idx = np.lexsort(keys)
        done = False
        for i in idx[:4000]:
            sh = shapes[int(cat[3][i])]
            at = grid.o + np.array([cat[5][i] * CELL, cat[6][i] * CELL_Y, cat[7][i] * CELL])
            p = sh.placed(key, at)
            rel = order.relations(p.boxes)
            if rel is None:
                continue
            order.add(p, rel)
            grid.add(p.boxes + np.array([-CLEAR / 2, CLEAR / 2])[:, None])
            top = max(top, float(p.boxes[:, 1, 1].max()) + CLEAR)
            done = True
            break
        if not done:
            raise RuntimeError(f"no room for {key} in the box")
        if top > best + 1e-6:
            return None
    return top


def pack(items: Sequence[tuple[str, str, Sequence[np.ndarray]]], tilt: float = 0.0) -> Packed:
    """Pack ``items`` (key, role, each part's points in the assembled pose; roles as
    :class:`Piece`'s) in one box (the module's docstring). ``tilt``: the cradle's tilt
    (degrees) in that pose, an angle :func:`level` tries."""
    c = CLEAR / 2
    pad = np.array([-c, c])[:, None]
    base = [(k, pts) for k, r, pts in items if r == "base"]
    panels = [(k, pts) for k, r, pts in items if r == "panel"]
    fills = [(k, pts) for k, r, pts in items if r == "fill"]
    # the footprint: as large as the base board and each panel lying flat, either face
    flat = {k: _aabbs(pts, _rot("x", -90)) for k, pts in panels}
    ext = [_aabbs(pts, np.eye(3)) for _, pts in base] + list(flat.values())
    sizes = np.array([[b[:, 1, 0].max() - b[:, 0, 0].min(), b[:, 1, 2].max() - b[:, 0, 2].min()]
                      for b in ext]) + 2 * c if ext else np.zeros((0, 2))
    # each fill piece, levelled: the turn about z, then its six ways to lie
    lev = {k: _rot("z", level(pts, tilt)) for k, pts in fills}
    dims = {k: np.sort(np.ptp(np.concatenate([p @ lev[k].T for p in pts]), axis=0))
            for k, pts in fills}
    fx = float(sizes[:, 0].max()) if len(sizes) else 0.0
    fz = float(sizes[:, 1].max()) if len(sizes) else 0.0
    for d in dims.values():                 # (every piece lies flat in it somewhere)
        if fx >= fz:
            fx, fz = max(fx, d[2] + 2 * c), max(fz, d[1] + 2 * c)
        else:
            fx, fz = max(fx, d[1] + 2 * c), max(fz, d[2] + 2 * c)
    nx, nz = math.ceil(fx / CELL - 1e-6), math.ceil(fz / CELL - 1e-6)
    long_axis = 0 if nx >= nz else 2
    if base:
        b0 = np.concatenate([_aabbs(pts, np.eye(3)) for _, pts in base])
        centre = (b0[:, 0].min(axis=0) + b0[:, 1].max(axis=0)) / 2
        y0 = float(b0[:, 0, 1].min()) - c
    else:
        allp = np.concatenate([np.concatenate(pts) for _, _, pts in items])
        centre = (allp.min(axis=0) + allp.max(axis=0)) / 2
        y0 = float(allp[:, 1].min()) - c
    x0, z0 = centre[0] - nx * CELL / 2, centre[2] - nz * CELL / 2
    fill_shapes = []
    for k, pts in sorted(fills, key=lambda kp: (-float(np.prod(dims[kp[0]])), kp[0])):
        shapes = []
        seen = set()
        body = _Body.of(pts, lev[k])
        for r in _axis_rotations():
            sh = _Shape.of(body, r)
            sig = (tuple(np.round(sh.size, 1)), tuple(map(tuple, sh.cells.tolist())))
            if sig not in seen:
                seen.add(sig)
                shapes.append(sh)
        # the flattest first: the least height, its greatest extent along the box
        shapes.sort(key=lambda sh: (round(float(sh.size[1]), 1),
                                    -float(sh.size[long_axis])))
        fill_shapes.append((k, shapes))

    def fresh() -> tuple[_Grid, _Order]:
        g = _Grid(x0, y0, z0, nx, nz, 1)
        o = _Order()
        for k, pts in base:
            p = Placed(k, np.eye(3), np.zeros(3), _aabbs(pts, np.eye(3)))
            o.add(p, (set(range(len(o.placed))), set()))
            g.add(p.boxes + pad)
        return g, o

    # the panels: every order, face and turn, each dropped on the heightmap of what is in
    turns = []
    for k, pts in panels:
        opts = []
        body = _Body.of(pts, np.eye(3))
        for face, spin in product((-90, 90), (0, 180, 90, 270)):
            sh = _Shape.of(body, _rot("y", spin) @ _rot("x", face))
            if sh.span[0] <= nx and sh.span[2] <= nz:
                opts.append(sh)
        turns.append((k, opts))
    arrangements: list[tuple[float, list[tuple[str, _Shape, np.ndarray]]]] = []

    def drop(top: np.ndarray, sh: _Shape, cache: dict) -> np.ndarray:
        """Where ``sh`` comes to rest on ``top``, nearest a corner: its grown corner."""
        pw, _, pd = sh.span
        nu, nv = nx - pw + 1, nz - pd + 1
        need = np.full((nu, nv), y0)
        for (a0, a1, _, _, c0, c1), yl in zip(sh.cells, sh.ylo, strict=True):
            wm = cache.get((a1 - a0, c1 - c0))
            if wm is None:
                wm = cache[(a1 - a0, c1 - c0)] = _win_max(top, a1 - a0, c1 - c0)
            need = np.maximum(need, wm[a0:a0 + nu, c0:c0 + nv] - yl)
        u = np.arange(nu)[:, None]
        v = np.arange(nv)[None, :]
        gap = np.minimum(u, nu - 1 - u) + np.minimum(v, nv - 1 - v)
        iu, iv = divmod(int(np.lexsort((gap.ravel(), need.ravel()))[0]), nv)
        return np.array([x0 + iu * CELL, float(need[iu, iv]), z0 + iv * CELL])

    def arrange(top: np.ndarray, left: list[int], done: list) -> None:
        if not left:
            arrangements.append((float(top.max()), done))
            return
        for i in left:                  # each panel next, each way it lies (one heightmap)
            k, opts = turns[i]
            cache: dict = {}
            for sh in opts:
                at = drop(top, sh, cache)
                raised = top.copy()
                for b in sh.placed(k, at).boxes + pad:
                    i0, i1, _, _, k0, k1 = g0.cells(b)
                    raised[i0:i1, k0:k1] = np.maximum(raised[i0:i1, k0:k1], b[1, 1])
                arrange(raised, [j for j in left if j != i], [*done, (k, sh, at)])

    g0, _ = fresh()
    arrange(g0.top, list(range(len(turns))), [])
    arrangements.sort(key=lambda a: a[0])
    best: tuple[float, _Order] | None = None
    tried: set[tuple] = set()
    for under, done in arrangements:
        sig = tuple((k, tuple(np.round(at, 1)), tuple(np.round(sh.rot, 3).ravel()))
                    for k, sh, at in done)
        if sig in tried:
            continue
        if len(tried) >= ARRANGEMENTS or (best and best[0] <= under + c + 1e-6):
            break               # (the panels alone stand as high as the best box: done)
        tried.add(sig)
        g, o = fresh()
        ok = True
        for k, sh, at in done:
            p = sh.placed(k, at)
            rel = o.relations(p.boxes)
            if rel is None:
                ok = False
                break
            o.add(p, rel)
            g.add(p.boxes + pad)
        if not ok:
            continue
        g.grow(math.ceil((g.top.max() - y0) / CELL_Y - 1e-6))    # (more if need be)
        top = _fill(g, o, fill_shapes, best[0] if best else math.inf)
        if top is not None and (best is None or top < best[0] - 1e-6):
            best = (top, o)
    assert best is not None
    _, o = best
    allb = np.concatenate([p.boxes for p in o.placed])
    inside = np.stack([allb[:, 0].min(axis=0) - CLEAR, allb[:, 1].max(axis=0) + CLEAR])
    seq = o.order()
    return Packed({p.key: p for p in o.placed}, [o.placed[i].key for i in seq], inside)


# -- the cradle's flat pack ------------------------------------------------------------------


def part_infos(mech) -> dict[str, PartInfo]:
    """The fabricated cradle's parts (not the display ones) as :func:`pieces` and
    :func:`pack` read them: a part that only ever turns by quarter turns (the base board's,
    the panels') by its bounding box's corners, the rest by their tessellations."""
    bodies = [b for b in mech.bodies if b.part is not None and b.fab != "display"]
    shapes = {b.name: b.placed_part() for b in bodies}
    corners = {}
    for n, sh in shapes.items():
        bb = sh.bounding_box()
        lo, hi = (bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z)
        corners[n] = np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1],
                                (lo, hi)[(i >> 2) & 1][2]] for i in range(8)], float)
    return {b.name: PartInfo(b.name, b.fab, b.bom_key, b.rigid_with, corners[b.name])
            for b in bodies}


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


def flat_pack(mech, lk, t: float = 0.0) -> dict:
    """How the fabricated cradle ``mech`` (at crank angle ``t``: :func:`pack_t`) packs:
    flat (:func:`pieces`), or as modules where it is cranked to its pack pose first
    (:data:`PACK_TURNS`, :func:`module_pieces`). Its pieces (``modules``, in the order they
    go in the box, bottom up: key, label, how it comes off, parts, ``matrix``: the 4 x 4
    row-major transform from the assembled pose to the packed one, in the cradle's own frame:
    x along the linkage, y up, z across the bay; and ``part_matrix``, a part's own where it
    moves within its piece: a hanging link swung in), the box (its inside and outside sizes,
    mm, and its inside's lower corner, ``floor``), the steps that take it apart, ``mode``
    (``"flat"`` or ``"modules"``), the ``pose`` the transforms start from (its crank angle,
    its clip fraction ``t / 2 pi``, the bottle's tilt, the handle turns from rest) and, for
    modules, ``slide_off``: what the passive side panel meets sliding off its cradle link
    (:func:`slide_hits`, its clips out: none)."""
    from spiderpig.mesh import tessellate_many

    link = cr.cradle_link(lk)
    infos = part_infos(mech)
    tilt = math.degrees(float(mech.meta.get("cradle_tilt", 0.0)))
    fastened = mech.meta.get("fastened", ())
    turns = PACK_TURNS.get(lk.key)
    if turns is not None:
        pcs, steps = module_pieces(infos, mech.connections, link, turns, tilt, fastened)
    else:
        pcs, steps = pieces(infos, mech.connections, link, fastened)
    # the parts that turn by any angle (the levelled pieces', the hanging links'): their
    # tessellations
    turned = [n for pc in pcs if pc.role == "fill" for n in pc.parts]
    turned += [n for pc in pcs for body, _ in pc.hanging for n in pc.parts
               if n == body or infos[n].rigid_with == body]
    shapes = {b.name: b for b in mech.bodies if b.name in set(turned)}
    meshes = tessellate_many([shapes[n].placed_part() for n in turned], 0.5, 0.5)
    for n, (pos, _, _) in zip(turned, meshes, strict=True):
        if len(pos):
            infos[n] = PartInfo(n, infos[n].fab, infos[n].bom_key, infos[n].rigid_with,
                                np.asarray(pos, float))
    joints = {(b.name, j.name): (b.pose.matrix @ j.pose.matrix)[:2, 3]
              for b in mech.bodies for j in b.joints}
    out = pack_cradle(pcs, steps, infos, joints, tilt)
    out["mode"] = "flat" if turns is None else "modules"
    out["pose"] = {"crank_t": round(t, 6), "clip_fraction": round(t / (2.0 * math.pi), 6),
                   "tilt_deg": round(tilt, 2), "turns_from_rest": turns}
    if turns is not None:
        by = {pc.key: pc for pc in pcs}
        placed = {b.name: b.placed_part() for b in mech.bodies
                  if b.part is not None and b.fab != "display"}
        panel = {n: placed[n] for n in by["passive"].parts if not n.endswith("_clip")}
        unit = {n: placed[n] for n in by["drive"].parts}
        out["slide_off"] = {"hits": [list(h) for h in slide_hits(panel, unit)],
                            "clips_out": sorted(n for n in by["passive"].parts
                                                if n.endswith("_clip"))}
    return out


def pack_cradle(pcs: list[Piece], steps, infos: dict[str, PartInfo], joints, tilt: float
                ) -> dict:
    """:func:`flat_pack`'s answer from its pieces and steps (:func:`pieces`), the parts'
    ``infos`` (a hanging link's and the levelled pieces' by their tessellations), ``joints``
    (a body and joint's xy as assembled) and the cradle's ``tilt`` (degrees)."""
    for pc in pcs:              # each hanging link swung into its panel (:func:`tuck`)
        for body, joint in pc.hanging:
            group = [n for n in pc.parts if n == body or infos[n].rigid_with == body]
            rest = [infos[n].points for n in pc.parts if n not in group]
            pivot = np.asarray(joints[(body, joint)], float)
            deg = tuck([infos[n].points for n in group], pivot, rest)
            if deg == 0.0:
                continue
            m = np.eye(4)
            m[:3, :3] = _rot("z", deg)
            m[:2, 3] = pivot - m[:2, :2] @ pivot
            for n in group:
                i = infos[n]
                infos[n] = PartInfo(n, i.fab, i.bom_key, i.rigid_with,
                                    i.points @ m[:3, :3].T + m[:3, 3])
                pc.moved[n] = m @ pc.moved.get(n, np.eye(4))
    by = {pc.key: pc for pc in pcs}
    res = pack([(pc.key, pc.role, [infos[n].points for n in pc.parts]) for pc in pcs], tilt)
    mods = []
    for k in res.order:
        pc, p = by[k], res.placed[k]
        mod = {"key": k, "label": pc.label, "how": pc.how, "parts": sorted(pc.parts),
               "matrix": np.round(p.matrix(), 6).tolist()}
        if pc.moved:
            mod["part_matrix"] = {n: np.round(p.matrix() @ m, 6).tolist()
                                  for n, m in sorted(pc.moved.items())}
        mods.append(mod)
    lo, hi = res.inside
    inside = [float(v) for v in hi - lo]
    return {
        "modules": mods,
        "box": {"inside_mm": [round(v, 1) for v in inside],
                "outside_mm": [round(v + 2 * WALL, 1) for v in inside],
                "floor": [round(float(v), 2) for v in lo],
                "volume_l": round(math.prod(v + 2 * WALL for v in inside) / 1e6, 1)},
        "steps": [text for text, _ in steps],
        "packed_mm": [round(v - 2 * CLEAR, 1) for v in inside],    # (what's in it)
    }


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
    t = pack_t(cfg)
    packed = mech if t == 0.0 else fabricate(tmpl, cfg, t, store=at)
    return {"pack": flat_pack(packed, cfg.lk, t), "room": room(cfg, mech)}
