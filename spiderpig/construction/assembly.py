"""The assembly order, structured: what the assembly guide (``spiderpig guide``) and the
docs render.

**Who says what.** Each group of a side declares how its own parts go on through its
``assembly`` hook (:meth:`construction.base.Group.assembly`): a list of :class:`Op`, each
a few of its bodies (or a :class:`Piece` of one: a Chicago pin's barrel now, its screw
later), the stage they go on in (:data:`STACK` the leg stack on the outer plate, bottom up;
:data:`UNIT` the inner-plate unit; :data:`JOIN` the unit onto the stack), a sort key (the
layer, for the stack), a tag the robot's order may move, a sentence, and whether it is
built on the bench first (``sub``). The robot-level parts (the chassis, the frame ties'
columns, the deck) declare theirs the same way (:func:`construction.chassis.assembly`,
:func:`construction.deck.assembly`). The robot's order itself is data here
(:data:`ROBOT_ORDER`; a side on its own :data:`SIDE_ORDER`): its stages, which tags make
each step, the sentences a stage puts in place of a hook's (the right side's inner plate
goes on on the robot, not on the bench), and the parts it moves (the right servo goes on
with the centre plates).

A body no hook claims still gets a step: on a side, the step of the layer its bottom face
sits in (the gap under a layer goes with that layer), with a generic sentence; on the
robot, a last step. :func:`assembly_steps` checks every body is added exactly once (whole, or in
pieces that cover it).

The hooks are read by name (``.assembly``), so neither they nor this module are in the
fabrication's code key (:mod:`spiderpig.keys`): an edit of a sentence keeps every cache.
"""

from __future__ import annotations

import re
from collections import defaultdict
from dataclasses import dataclass, field, replace

from spiderpig.hardware.sheet_catalog import SCS_DEBURR_MM

STACK, UNIT, JOIN = "stack", "unit", "join"
CHASSIS, WIRING, DECK, OTHER = "chassis", "wiring", "deck", "other"
CRADLE, FINISH, USE = "cradle", "finish", "use"     # a decanting cradle's own stages
SHOP = "shop"           # ... and its shop work before the first part goes on
SIDES = ("L", "R")
EPS = 0.05


@dataclass(frozen=True)
class Piece:
    """A body, or a portion of one (``portion`` its name, ``z`` its z interval in the
    hook's coordinates: side-z for a side's hook, robot-z for the robot's)."""

    name: str
    portion: str = ""
    z: tuple[float, float] | None = None


@dataclass(frozen=True)
class Scene:
    """How a step's picture poses what it shows, where the assembled pose won't do (a
    decanting cradle's use, its trip apart and back, its passive side lowered on lying down:
    :mod:`spiderpig.guide.scenes` draws it). ``pose``: the crank angle (``"built"`` the
    fabricated one, ``"rest"``, ``"level"``, ``"full"``, ``"pack"``); ``lay``: ``""``
    standing, ``"drive"`` laid on its drive side on the riser's blocks
    (``construction.cradle.LAY_BLOCKS``), ``"passive"`` on its passive side's outer plate;
    ``show``: the pieces drawn (``cradle_pack``'s keys: ``board``, ``drive``, ``passive``,
    ``box``, ``hand_crank``, ``risers``, ``blocks``, and ``bottle``, ``decanter``; ``()``
    every part the step has, as an assembly step's picture does); ``moves``: a piece moved
    ``(dx, dy, dz)`` mm after the lay (y up); ``marks``: regexes of the bodies highlighted
    (what the step handles); ``views``: the cameras to choose among (``guide.render.VIEWS``)."""

    pose: str = "built"
    lay: str = ""
    show: tuple[str, ...] = ()
    moves: tuple[tuple[str, float, float, float], ...] = ()
    marks: tuple[str, ...] = ()
    views: tuple[str, ...] = ("robot", "robot-", "robot^")


@dataclass(frozen=True)
class Op:
    """Some bodies that go on together."""

    stage: str
    key: tuple = ()
    pieces: tuple[Piece, ...] = ()
    title: str = ""
    text: str = ""
    tag: str = ""
    sub: bool = False            # built on the bench, drawn alone, put on in the next step
    side: str | None = None      # a side's hook: filled in; the robot's hooks say
    count: bool = True           # its parts count in its step's parts list (a later piece
    #                              of a body counted already: False)
    extras: tuple[str, ...] = ()  # the BOM's lines of these catalog keys that are no body's
    #                               (a harness's resistors, a strap): bought for this step
    scene: Scene | None = None   # its step's picture's pose (a cradle's use; else assembled)
    lines: tuple[str, ...] = ()  # more sentences after ``text``, each its own item (a list)


def whole(*names: str) -> tuple[Piece, ...]:
    return tuple(Piece(n) for n in names)


@dataclass
class SideView:
    """What a side's hook reads: its side's bodies by bare name (no ``L.`` / ``R.``), their
    z interval in side coordinates (the outer frame plate at 0, up the stack), host, fab
    and catalog key, and the plan's layers."""

    side: str                                  # "L", "R" or "" (a side on its own)
    z: dict[str, tuple[float, float]]
    hosts: dict[str, str | None]
    fab: dict[str, str | None]
    keys: dict[str, str | None]
    layers: dict[int, tuple[float, float]]
    ctx: object = None                         # the side's construction.base.Context

    @property
    def top(self) -> int:
        return max(k for k in self.layers if k >= 0) - 1    # (layers holds top + 1)

    def slot(self, z0: float) -> int:
        """The layer a part whose bottom face is at ``z0`` goes on with: the first layer
        whose top is above it (the gap under a layer belongs to that layer); -1 under the
        outer plate, ``top + 1`` over the inner one."""
        for k in sorted(self.layers):
            if z0 < self.layers[k][1] - EPS:
                return k
        return max(self.layers) + 1

    def named(self, pattern: str) -> list[str]:
        rx = re.compile(pattern)
        return sorted((n for n in self.z if rx.fullmatch(n)), key=lambda n: (self.z[n], n))


@dataclass
class RobotView:
    """What a robot-level hook reads: every body by full name, its robot-z interval (the
    mid-plane at 0, the left side below), host, fab, catalog key; ``meta`` the robot's."""

    z: dict[str, tuple[float, float]]
    hosts: dict[str, str | None]
    fab: dict[str, str | None]
    keys: dict[str, str | None]
    meta: dict = field(default_factory=dict)

    def named(self, pattern: str) -> list[str]:
        rx = re.compile(pattern)
        return sorted((n for n in self.z if rx.fullmatch(n)), key=lambda n: (self.z[n], n))


# ---------------------------------------------------------------------------
# the robot's order
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Stage:
    """One stage of the order: ``tags`` the tags of each step (``"R.unit.servo"`` takes
    another stage's op; ``()``: by layer for the stack, one step per tag else), ``text`` a
    sentence in place of the hooks' for a tag, ``intro`` the stage's first sentence,
    ``where`` "bench" or "robot" (the picture: the stage alone, or everything so far),
    ``strict``: only its ``tags`` (else the stage's other ops follow them, a step a tag),
    ``titles`` a step's title by its first op's tag."""

    stage: str
    side: str | None
    title: str
    tags: tuple[tuple[str, ...], ...] = ()
    text: tuple[tuple[str, str], ...] = ()
    intro: str = ""
    where: str = "bench"
    strict: bool = False
    titles: tuple[tuple[str, str], ...] = ()


ROBOT_ORDER: tuple[Stage, ...] = (
    # (the assembly audit of 2026-10-04: the order every fastener of the default walker
    # can be driven in)
    Stage(STACK, "L", "Left side: leg stack",
          intro="Each side's leg stack is built bottom up on its outer frame plate, in the "
                "plan's layer order."),
    Stage(UNIT, "L", "Left side: inner-plate unit",
          tags=(("plate", "servo", "servo_screws"), ("horn",), ("rail",),
                 ("ties", "tie_screws")),
          intro="On the bench, loose (the right side's is put together on the robot)."),
    Stage(JOIN, "L", "Left side: unit onto the leg stack", where="robot",
          intro="The unit onto its leg stack: the hub plate's hex pocket over the hub "
                "chain's standoff (turn the crank to line it up), the pillars' tops into "
                "the inner plate."),
    Stage(CHASSIS, None, "Centre plates", where="robot",
          tags=(("studs",), ("plates_L",), ("plates_mid",), ("R.unit.servo", "plates_R")),
          text=(("studs", "The M3 set-screw studs into the left chains' ends "
                          "(threadlocker)."),
                ("servo", ""))),
    Stage(UNIT, "R", "Right side: inner plate, on the robot", where="robot",
          tags=(("ties",), ("plate", "servo_screws", "tie_screws"), ("rail",), ("horn",)),
          text=(("ties", "Turn the right tie chains onto the studs from the inner plate's "
                         "side (they turn freely: no inner plate yet), shims on their "
                         "ends."),
                ("plate", "The right inner plate onto the servo's front and onto the "
                          "chains."),
                ("tie_screws", "The chains' M3 button heads from the leg side "
                               "(threadlocker).")),
          intro="Not on the bench: the chains must turn onto the studs before the inner "
                "plate holds them."),
    Stage(STACK, "R", "Right side: leg stack",
          intro="Build the right leg stack as the left one: the mirror image."),
    Stage(JOIN, "R", "Right side: body onto the leg stack", where="robot",
          intro="Turn the body over onto the right leg stack: the hub plate's pocket over "
                "its hub chain's standoff, the pillars' tops into the inner plate."),
    Stage(WIRING, None, "Wiring", where="robot"),
    Stage(DECK, None, "Deck", where="robot", tags=(("electronics",), ("deck",))),
    Stage(OTHER, None, "Remaining parts", where="robot"),
)
"""The whole robot's order (the default walker: chicago pins, standoff pillars, the hex
bolt crank, the frame ties): what the pivots', the crank's, the chassis' and the deck's
docstrings defer to."""

USE_TAGS: tuple[tuple[str, ...], ...] = (("where",), ("setting",), ("riser",), ("decant",),
                                          ("limits",))
"""A cradle's use steps, in this order (design review round 2's M4: untagged they had sorted
alphabetically, the trip apart before the setting); the trip's own (``pack_01`` ...,
``unpack_01`` ...: ``construction.cradle.module_steps``) follow, by tag."""

def _inch(mm: float) -> str:
    return f"{mm / 25.4:g}"


AS_CUT = "<the parts that arrive as cut>"
"""Where :data:`CRADLE_ARRIVAL` names the parts that arrive as cut: :func:`as_cut_words`, from
the fabrication (:func:`assembly_steps`)."""
THICK_PINS = "<the pins whose links may outgrow their play>"
"""Where :data:`CRADLE_ARRIVAL` names the pins whose links, both at their sheets' thickest,
come out over the pin's play: :func:`thick_pins_words`."""

CRADLE_ARRIVAL = (
    "First the cut parts, as they arrive. Write each one's label on it with the paint pen "
    "(the first word of its file's name in ORDER.md's upload list): some differ only by a "
    "hole or a slot, the crank block's plates among them (the gearbox's key drops through "
    "a clear plate's slot, not through a keyed one's: those are filed to it later). "
    f"SendCutSend deburrs only parts of at least {_inch(SCS_DEBURR_MM[0])} x "
    f"{_inch(SCS_DEBURR_MM[1])} in: {AS_CUT} come as cut (ORDER.md's upload list marks them "
    "\"as cut: by hand\"). Deburr each by hand before it is stacked, both faces and each "
    "pin hole's edges (a deburring tool, then fine sandpaper on a flat block); the dross is "
    "on the laser's exit face. The clutch's seat ring: mark its entry face (the clean one) "
    "with the paint pen and break each of its six holes' edges on that face alike, two "
    "turns of a countersink by hand (the deburring tool's countersink blade): those edges "
    "are the clutch's ball seats, and it goes on that face out, to the balls. Caliper every "
    "link at its pin holes and note it: a sheet may run thick (SendCutSend's tables: up to "
    "+0.69 mm at 0.250 in, +0.23 mm at 0.187 in), and the pins are cut from these figures "
    f"(the pins' step: each pin's stack + 0.3 mm). {THICK_PINS}Each pin, once cut, must "
    "slide through its "
    "links' holes by hand before it goes on: the 0.250 in rod's can come out up to 0.05 mm "
    "under (SendCutSend's +/-0.005 in and its taper on thick sheet, the hole narrowest at "
    "the exit face); open a tight one from that face, fine sandpaper rolled round a drill "
    "bit's shank a size under the pin, a few twists at a time, until the pin slides.")
"""What a cradle's guide says of the cut parts before the first one goes on (design review
round 2's M3 and M5: the links SendCutSend can't deburr, the sheets' thickness against the
pins' take-up; round 3's laser-cut parts minors 2, 3, 4 and 6: the clutch's seats broken
alike on the clean face, the as-cut parts named, the 0.250 in rod's 8.2 mm holes against a
g6 pin, a label on every part). The parts that arrive as cut: :func:`as_cut_words` (and
ORDER.md, :func:`hardware.bom.hand_deburred`)."""


def as_cut_words(mech) -> str:
    """The laser-cut parts of ``mech`` whose blank is under their service's deburring
    minimum (:func:`hardware.bom.deburr_least`, the rule of
    :func:`manufacture.deburr_issue`), by name and once each, for :data:`AS_CUT`: ``"b4
    and b5"``; ``"the narrower links"`` when none or not known."""
    from spiderpig.hardware.bom import deburr_least
    from spiderpig.layout import laser_bodies, sheet_key
    from spiderpig.manufacture import blank_of

    default = str(mech.meta.get("sheet") or "")
    names: list[str] = []
    for b in laser_bodies(mech):
        name = bare(b.name)
        least = deburr_least(sheet_key(b, default)) if default or b.sheet else None
        if name in names or least is None:
            continue
        w, h = blank_of(b)
        if w < least[0] - 1e-3 or h < least[1] - 1e-3:
            names.append(name)
    if not names:
        return "the narrower links"
    names.sort()
    return names[0] if len(names) == 1 else ", ".join(names[:-1]) + " and " + names[-1]


def thick_pins_words(mech) -> str:
    """For :data:`THICK_PINS`: the pins (``mech.meta["wobble"]``'s ``pin:`` notes) whose
    links, each at its sheet's published most (``thickness_tol``), outgrow the pin's play as
    drawn (``cradle_report.take_up``'s worst case, simplification round 2: b3 in 6061 0.187
    in put J and J2 there beside K and P), named with a sentence of what to do; empty when
    none."""
    from spiderpig.hardware.catalog import get
    from spiderpig.layout import laser_bodies, sheet_key

    default = str(mech.meta.get("sheet") or "")
    sheets = {bare(b.name): sheet_key(b, default) for b in laser_bodies(mech)}
    short: dict[str, float] = {}
    for name, note in (mech.meta.get("wobble") or {}).items():
        if not name.startswith("pin:") or not note.get("links"):
            continue
        over = 0.0
        for ln in note["links"]:
            key = sheets.get(bare(str(ln.get("link", ""))))
            if key:
                over += float((get(key).dims.get("thickness_tol") or (0.0, 0.0))[0])
        margin = float(note.get("play_mm") or 0.0) - over
        if margin < 0:
            short[name[4:]] = margin
    if not short:
        return ""
    pins = sorted(short, key=lambda p: (len(p), p))
    words = pins[0] if len(pins) == 1 else ", ".join(pins[:-1]) + " and " + pins[-1]
    lo, hi = min(short.values()), max(short.values())
    span = f"{-hi:.2f} mm" if abs(hi - lo) < 0.005 else f"{-hi:.2f} to {-lo:.2f} mm"
    many = len(pins) > 1
    return (f"{'Pins' if many else 'Pin'} {words} {'each carry' if many else 'carries'} "
            "links that, all at their sheets' thickest, outgrow the pin's play as drawn (by "
            f"{span}): caliper those links first, and where a pin's links bind once its end "
            "is on, sand the printed ring or spacer between them by the excess. ")


CRADLE_FLAT = (
    "Lay the base board on the flat table first: SendCutSend's sheet is flat to +/- 0.030 in "
    "a foot before cutting (about 1.7 mm over the board). If it bows, its concave face goes "
    "down (the cradle's weight then flattens it), and its cleats are glued on that way.")
"""A cradle's board flatness check (design review round 2's minor 9), before the drive side
stands on it and before its cleats are glued."""

FIRST_POUR = Stage(FINISH, None, "Before the first pour", where="robot",
                   tags=(("clutch",),), strict=True,
                   intro="The cradle stands built on its board: only now can a bottle go in.")
"""A cradle's last build step (``construction.handcrank.HandCrank.first_pour``): the clutch
set on a start measured with a bottle in the cradle, the worm's hold, the toggle zones'
marks; the trip's way back sends the user here (design review round 3's stand and use M2: the
clutch had been set at the hand crank's step, before the cradle box existed)."""

SHOP_STAGE = Stage(SHOP, None, "Shop work, before the build",
                   tags=(("arrival",), ("rods",), ("pins",), ("taps",), ("carvera",)),
                   strict=True, intro=CRADLE_ARRIVAL)
"""A cradle's first stage (simplification round 1, 2026-10-21: its posts, tie rods, pins and
crankpin are 6061 rod made at home, its cleats and riser locator cut on the user's Carvera;
``construction.cradle.shop_ops``): the parts as they arrive, the rods cut and faced, the pins
cut to their stacks, every rod end drilled and tapped, the birch."""

CRADLE_ORDER: tuple[Stage, ...] = (
    # a decanting cradle (construction.cradle): the shop work first, then the drive side
    # bottom up with its tie rods, bolted to the base plate (it can't stand alone: the
    # gearbox outboard, the rods and saddles inboard), the box on the rods, then the passive
    # side: its cradle link onto the rods, its stack slid in from outside, what goes over
    # the link, its inner plate last (2026-10-10 review round 2: the plate had gone on a bay
    # tie A doesn't have)
    SHOP_STAGE,
    Stage(STACK, "L", "Drive side: stack",
          intro="The drive side (the crank's) is built bottom up on its outer frame plate, "
                "in the plan's layer order, the gearbox under the plate first: it stands "
                "the plate up off the bench while the stack goes on. The posts (pillars "
                "and columns) and tie rods are the 6061 rod faced and tapped in the shop "
                "steps; a link turns on each pivot post's, each pin's and the crankpin's "
                "hard coat: wipe each of those with a thin film of the food-grade grease "
                "(a rag) as it goes in (the bare columns and tie rods take none). Break the "
                "laser-cut edges where hands go (a file or fine sandpaper): the bay side and "
                "the frame plates' tops."),
    Stage(UNIT, "L", "Drive side: inner plate",
          tags=(("plate", "bay_tie"),),
          text=(("plate", "Lay the drive side's inner plate down, its bay face down."),),
          intro="On the bench."),
    Stage(JOIN, "L", "Drive side: inner plate onto the stack", where="robot",
          tags=(("pillar_screws", "column_screws"),),
          intro="The inner plate onto its stack: the tops of the posts that reach it into "
                "it. Once their screws are tight the cradle link must still swing freely on "
                "the post it rides (its printed parts stop 0.3 mm short of the post's "
                "length): turn the gearbox's input shaft by hand (the worm can't be "
                "turned from the link) and feel for a bind; if it binds, sand the spacer "
                "under the inner plate."),
    Stage(FINISH, None, "Drive side onto the base board", where="robot",
          tags=(("studs_L",), ("base",)), strict=True,       # (B's studs on the bench first:
          #                                                   it drops into its brackets)
          intro=CRADLE_FLAT + " The drive side stands on the base board from here on, the "
                "board bare: its cleats and its finish go on once both sides stand on it."),
    Stage(FINISH, None, "Hand crank", where="robot", tags=(("handle",),), strict=True,
          intro="On now, to turn the crank for the checks to come."),
    Stage(CRADLE, None, "The cradle box", where="robot",
          tags=(("saddles",), ("light",), ("base_stop",)),
          intro="The tie rods stand across the bay from the drive side's cradle link."),
    Stage(UNIT, "R", "Passive side: cradle link, on the cradle", where="robot",
          tags=(("R.stack.cradle_link", "tie_screws"),), strict=True,
          text=(("cradle_link", "The passive side's cradle link onto the tie rods' free "
                                "ends."),),
          intro="On the cradle, not on the bench: the tie rods hold the passive side's "
                "cradle link."),
    Stage(STACK, "R", "Passive side: stack",
          intro="The passive side's stack, on the bench, bottom up on its outer plate: the "
                "drive side's mirror image without the gearbox, the crank and the rod, and "
                "without its cradle link (on the cradle already) and what goes over it."),
    Stage(JOIN, "R", "Passive side: stack onto the cradle", where="robot",
          tags=(("over_link",), ("R.unit.plate", "R.unit.bay_tie"),
                ("pillar_screws", "column_screws")),
          titles=(("pillar_screws", "Inner plate's screws"),
                  ("column_screws", "Inner plate's screws")),
          text=(("plate", "The passive side's inner plate over the posts' tops, "
                          "its bay face to the bay."),),
          intro="The passive stack set down on the base board 7-15 mm out from its place "
                "(inside its carriage bolts) and pushed home, the post the cradle link "
                "rides up through the link; then what goes over the link, and the "
                "inner plate. With its screws tight, turn the crank "
                "through a whole pour: both cradle links must swing freely."),
    Stage(FINISH, None, "Passive side onto the base board", where="robot",
          tags=(("base_R",),), strict=True),
    Stage(FINISH, None, "The board's cleats and finish", where="robot",
          tags=(("board",),), strict=True),
    FIRST_POUR,
    Stage(USE, None, "Using it", where="robot", tags=USE_TAGS),
    Stage(OTHER, None, "Remaining parts", where="robot"),
)
"""A decanting cradle's order (:mod:`construction.cradle`): what the drive's, the stand's
and the cradle box's hooks defer to. The passive side's parts over its cradle link go on
with the join (``over_link``: :func:`assembly_steps`). A's: its cradle link rides pillar
G between the passive side's plates, so that side's stack goes on standing, onto the
cradle. B's is :data:`CRADLE_MODULE_ORDER`."""

CRADLE_MODULE_ORDER: tuple[Stage, ...] = (
    # a cradle that comes apart in modules (construction.cradle.PACK_POSES: B): the first
    # build puts the passive side on as the trip's way back does, its panel built flat on
    # the bench and lowered onto its cradle link, the cradle lying on its drive side (design
    # review round 2's M1, 2026-10-14: standing, its stack had to be pushed onto two hidden
    # pins, its inner plate screwed on inside the bay, and its cleats couldn't be glued)
    *CRADLE_ORDER[:next(i for i, st in enumerate(CRADLE_ORDER)       # to the cradle box
                        if st.title == "The cradle box") + 1],
    Stage(FINISH, None, "Laid on its drive side", where="robot", tags=(("lay",),),
          strict=True),
    Stage(UNIT, "R", "Passive side: cradle link, on the cradle", where="robot",
          tags=(("R.stack.cradle_link", "tie_screws"),), strict=True,
          text=(("cradle_link", "The passive side's cradle link onto the tie rods' free "
                                "ends, which stand up from the cradle lying on its drive "
                                "side."),),
          intro="On the cradle, not on the bench: the tie rods hold the passive side's "
                "cradle link."),
    Stage(STACK, "R", "Passive side panel: stack",
          intro="The passive side panel is built whole on the bench, lying on its outer "
                "plate, as the drive side was: its stack bottom up (the drive side's mirror "
                "image without the gearbox, the crank and the rod), its inner plate and its "
                "brackets; all but its cradle link, which is on the cradle already."),
    Stage(UNIT, "R", "Passive side panel: inner plate", tags=(("plate", "bay_tie"),),
          strict=True,
          text=(("plate", "Lay the passive side's inner plate down, its bay face down."),),
          intro="On the bench."),
    Stage(JOIN, "R", "Passive side panel: inner plate onto the stack",
          tags=(("pillar_screws", "column_screws"),), strict=True,
          intro="The inner plate onto its stack, on the bench: the tops of the posts "
                "that reach it into it. Once their screws are tight the links must "
                "swing freely."),
    Stage(FINISH, None, "Passive side panel: brackets", tags=(("brackets_R",),),
          strict=True),
    Stage(JOIN, "R", "Passive side panel onto the cradle link", where="robot",
          tags=(("over_link",),), strict=True),
    Stage(FINISH, None, "Onto the base board: its cleats and finish", where="robot",
          tags=(("board",),), strict=True),
    FIRST_POUR,
    Stage(USE, None, "Using it", where="robot", tags=USE_TAGS),
    Stage(OTHER, None, "Remaining parts", where="robot"),
)
"""A cradle that packs as modules (B, ``construction.cradle.PACK_POSES``): A's order to the
cradle box, then the cradle laid on its drive side on the riser's blocks, the passive cradle
link onto the tie rods' tops, the passive side panel built whole on the bench (its stack,
inner plate and brackets) and lowered onto its cradle link as the trip's way back does
(``over_link``: its pins' clips), and the cradle stood on its board: the cleats glued, the
board finished, the knobs snugged."""

SIDE_ORDER: tuple[Stage, ...] = (
    Stage(STACK, "", "Leg stack",
          intro="The leg stack is built bottom up on the outer frame plate, in the plan's "
                "layer order."),
    Stage(UNIT, "", "Inner-plate unit",
          tags=(("plate", "servo", "servo_screws"), ("horn",), ("rail",),
                 ("ties", "tie_screws"))),
    Stage(JOIN, "", "Unit onto the leg stack", where="robot",
          intro="The unit onto its leg stack: the hub plate's hex pocket over the hub "
                "chain's standoff (turn the crank to line it up), the pillars' tops into "
                "the inner plate."),
    Stage(OTHER, None, "Remaining parts", where="robot"),
)
"""A side on its own (a mechanism)."""

WIRING_TEXT = (
    "The bus plugs are already in (the centre plates' steps: the top-entry sockets take a "
    "plug only through the plates' window): each servo's wires from its channel's end at "
    "the centre plates' +x edge up through the deck's wire slot over it to its own one of "
    "the board's two bus servo ports (connector first), before the deck goes in.",
)


# ---------------------------------------------------------------------------
# steps
# ---------------------------------------------------------------------------


@dataclass
class Step:
    """One numbered step. ``adds`` / ``context`` / ``places`` hold piece ids (a body's
    name, or ``name#portion``); ``clip`` each portion's robot-z interval."""

    number: int
    title: str
    stage: str
    stage_title: str
    side: str | None
    adds: list[str]
    context: list[str]
    places: list[str]
    text: list[str]
    where: str                  # "bench" or "robot"
    sub: bool = False
    counted: list[str] = field(default_factory=list)   # the bodies its parts list counts
    clip: dict[str, tuple[float, float]] = field(default_factory=dict)
    layer: int | None = None    # a stack step's layer
    extras: list[str] = field(default_factory=list)    # its ops' ``extras``
    scene: Scene | None = None  # its picture's pose (its first op's with one)


def body_of(piece_id: str) -> str:
    return piece_id.split("#", 1)[0]


def side_of(name: str) -> str | None:
    return name[0] if re.match(r"^[LR]\.", name) else None


def bare(name: str) -> str:
    return re.sub(r"^[LR]\.", "", name)


def rows_of(mech) -> list[dict]:
    """Each body's name, robot-z interval (None without a part), host, fab and key."""
    rows = []
    for b in mech.bodies:
        z = None
        if b.part is not None:
            bb = b.placed_part().bounding_box()
            z = (float(bb.min.Z), float(bb.max.Z))
        rows.append({"name": b.name, "z": z, "rigid_with": b.rigid_with, "fab": b.fab,
                         "bom_key": b.bom_key})
    return rows


def layers_of(plan) -> dict[int, tuple[float, float]]:
    """The plan's layers' side-z intervals, the outside ones (-1, top + 1) included."""
    return {k: tuple(plan.z(k)) for k in range(-1, plan.top + 2)}


def _to_side(z: tuple[float, float], side: str, mid: float) -> tuple[float, float]:
    if side == "L":
        return (z[0] + mid, z[1] + mid)
    if side == "R":
        return (mid - z[1], mid - z[0])
    return z


def _to_robot(z: tuple[float, float], side: str, mid: float) -> tuple[float, float]:
    if side == "L":
        return (z[0] - mid, z[1] - mid)
    if side == "R":
        return (mid - z[1], mid - z[0])
    return z


def side_ops(view: SideView, groups) -> list[Op]:
    """Every op of one side: its groups' hooks', then the per-layer default."""
    ops: list[Op] = []
    for g in groups:
        hook = getattr(g, "assembly", None)
        if hook is not None:
            ops.extend(hook(view))
    claimed = _claimed(ops)
    by_slot: dict[int, list[str]] = defaultdict(list)
    for n in view.z:
        if n not in claimed:
            by_slot[view.slot(view.z[n][0])].append(n)
    for k, names in sorted(by_slot.items()):
        spacers = all(re.search(r"ring|spacer|shim|collar", n) for n in names)
        text = ("The printed rings and spacers drawn, onto their pillars, pins and crankpins."
                if spacers else "The other parts drawn, where they sit.")
        ops.append(Op(STACK, (k, 3), whole(*sorted(names, key=lambda n: view.z[n])),
                      text=text, tag="layer"))
    return ops


def _claimed(ops: list[Op]) -> set[str]:
    return {p.name for op in ops for p in op.pieces}


def assembly_steps(mech, design, robot: bool | None = None) -> list[Step]:
    """The guide's steps for a fabricated ``mech`` (a robot, or a side on its own) of
    ``design`` (its ``SideDesign``: the groups and the plan)."""
    from spiderpig.construction import chassis, deck

    rows = [r for r in rows_of(mech) if r["z"] is not None]
    cradle = bool(mech.meta.get("cradle"))      # both sides and the cradle box
    robot = bool(mech.meta.get("robot") or cradle) if robot is None else robot
    mid = float(mech.meta.get("mid_plane", 0.0)) if robot else 0.0
    layers = layers_of(design.plan)
    sides = SIDES if robot else ("",)
    ops: list[Op] = []
    clip_side: dict[str, str] = {}
    for s in sides:
        mine = [r for r in rows if (side_of(r["name"]) or "") == s]
        prefix = f"{s}." if s else ""
        if robot:     # the robot's own parts with a side's prefix: its hooks' (rear screws,
            #           the ties' columns, the deck rails)
            mine = [r for r in mine if not _robot_level(bare(r["name"]))]
        view = SideView(s, {bare(r["name"]): _to_side(r["z"], s, mid) for r in mine},
                        {bare(r["name"]): r["rigid_with"] and bare(r["rigid_with"])
                         for r in mine},
                        {bare(r["name"]): r["fab"] for r in mine},
                        {bare(r["name"]): r["bom_key"] for r in mine},
                        layers, design.ctx)
        for op in side_ops(view, design.groups):
            pieces = tuple(Piece(prefix + p.name, p.portion,
                                 None if p.z is None else _to_robot(p.z, s, mid))
                           for p in op.pieces)
            ops.append(replace(op, pieces=pieces, side=s, extras=()))
        clip_side.update({prefix + bare(r["name"]): s for r in mine})
    if robot:
        rv = RobotView({r["name"]: r["z"] for r in rows}, {r["name"]: r["rigid_with"]
                                                             for r in rows},
                       {r["name"]: r["fab"] for r in rows},
                       {r["name"]: r["bom_key"] for r in rows}, dict(mech.meta))
        if cradle:
            from spiderpig.construction import cradle as cradle_mod

            ops += cradle_mod.assembly(rv)
        else:
            ops += chassis.assembly(rv) + deck.assembly(rv)
        left = sorted(set(rv.z) - _claimed(ops))
        if left:
            ops.append(Op(OTHER, (), whole(*left), "Remaining parts",
                          "The remaining parts, where they are drawn.", tag="other"))
    if cradle:
        # the passive side's parts over its cradle link (which goes on the tie rods, on the
        # cradle) go on once its stack has met the link there
        k_link = mech.meta.get("cradle_layer")
        ops = [replace(op, stage=JOIN, tag="over_link")
               if k_link is not None and op.side == "R" and op.stage == STACK and op.key
               and isinstance(op.key[0], (int, float)) and op.key[0] > k_link else op
               for op in ops]
    _check(ops, {r["name"] for r in rows})
    order = ((CRADLE_MODULE_ORDER if mech.meta.get("cradle_modules") else CRADLE_ORDER)
             if cradle else ROBOT_ORDER if robot else SIDE_ORDER)
    if cradle:
        words, thick = as_cut_words(mech), thick_pins_words(mech)
        order = tuple(replace(st, intro=st.intro.replace(AS_CUT, words)
                              .replace(THICK_PINS, thick))
                      if AS_CUT in st.intro or THICK_PINS in st.intro
                      else st for st in order)
    return _order(ops, order)


def _robot_level(name: str) -> bool:
    return bool(re.match(r"(rear_screw|tie_|deck_|centre_plate)", name))


def _check(ops: list[Op], names: set[str]) -> None:
    whole_of: dict[str, int] = defaultdict(int)
    parts_of: dict[str, list[str]] = defaultdict(list)
    for op in ops:
        for p in op.pieces:
            if p.portion:
                parts_of[p.name].append(p.portion)
            else:
                whole_of[p.name] += 1
    twice = [n for n, c in whole_of.items() if c > 1 or (c and parts_of.get(n))]
    split = [n for n, ps in parts_of.items() if len(ps) != len(set(ps)) or len(ps) < 2]
    stray = (set(whole_of) | set(parts_of)) - names
    missing = names - set(whole_of) - set(parts_of)
    if twice or split or stray or missing:
        raise ValueError(f"assembly ops: twice {sorted(twice)[:5]}, split {sorted(split)[:5]}"
                         f", unknown {sorted(stray)[:5]}, missing {sorted(missing)[:5]}")


def _pid(p: Piece) -> str:
    return f"{p.name}#{p.portion}" if p.portion else p.name


def _order(ops: list[Op], order: tuple[Stage, ...]) -> list[Step]:
    """The steps, stage by stage."""
    used: set[int] = set()
    groups: list[tuple[Stage, list[Op], int | None]] = []
    for st in order:
        mine = [i for i, op in enumerate(ops) if i not in used and op.stage == st.stage
                and (st.side is None or (op.side or "") == st.side)]
        if st.stage == STACK:
            # by layer: each bench sub-assembly before the layer it goes on in
            slots: dict[tuple, list[int]] = defaultdict(list)
            for i in mine:
                op = ops[i]
                k = op.key[0] if op.key else 0
                slots[(k, 0, op.key, op.title) if op.sub else (k, 1, (), "")].append(i)
            for sk in sorted(slots, key=lambda t: (t[0], t[1], repr(t[2]))):
                # within a layer by key: the sleeves, the links, the barrels, the screws
                group = sorted(slots[sk], key=lambda i: (ops[i].key, i))
                groups.append((st, [ops[i] for i in group], sk[0]))
                used.update(slots[sk])
                if sk[1] == 0 and (sk[0], 1, (), "") not in slots:
                    groups.append((st, [], sk[0]))   # a sub-assembly onto its own layer
            continue
        specs = list(st.tags)
        tags = {t for spec in specs for t in spec if "." not in t}
        extra = [] if st.strict else sorted({ops[i].tag for i in mine
                                             if ops[i].tag not in tags})
        specs += [(t,) for t in extra]
        for spec in specs:
            chosen = []
            for t in spec:
                if "." in t:                         # another stage's op: "R.unit.servo"
                    s, sg, tag = t.split(".")
                    chosen += [i for i, op in enumerate(ops) if i not in used
                               and (op.side or "") == s and op.stage == sg and op.tag == tag]
                else:
                    chosen += [i for i in mine if i not in used and ops[i].tag == t]
            chosen = sorted(set(chosen), key=lambda i: (ops[i].key, i))
            if not chosen:
                continue
            subs = [i for i in chosen if ops[i].sub]
            rest = [i for i in chosen if not ops[i].sub]
            groups.extend((st, [ops[i] for i in part], None)
                          for part in [[i] for i in subs] + ([rest] if rest else []))
            used.update(chosen)
        if st.stage == WIRING and not any(s is st for s, _, _ in groups):
            groups.append((st, [], None))       # a wiring step even with no hook's say
    leftover = [op for i, op in enumerate(ops) if i not in used]
    if leftover:
        raise ValueError(f"ops no stage takes: {[(o.stage, o.side, o.tag) for o in leftover]}")
    return _numbered(groups)


def _numbered(groups) -> list[Step]:
    out: list[Step] = []
    placed: list[str] = []
    stage_of: dict[str, tuple] = {}
    counted: set[str] = set()
    pending: list[str] = []
    for st, gops, layer in groups:
        pieces = [p for op in gops for p in op.pieces]
        adds = [_pid(p) for p in pieces]
        sub = bool(gops) and all(op.sub for op in gops)
        key = (st.stage, st.side)
        if sub:
            context: list[str] = []
            places: list[str] = []
        else:
            if st.where == "bench":
                context = [n for n in placed if stage_of[n] == key]
            else:
                context = list(placed)
            places, pending = pending, []
            if st.stage == JOIN:        # the unit (made on the bench) or the stack goes on
                unit_bench = any(s.stage == UNIT and s.side == st.side and s.where == "bench"
                                 for s, _, _ in groups)
                src = (UNIT if unit_bench else STACK, st.side)
                places = [n for n in placed if stage_of[n] == src]
            context = [n for n in context if n not in places]
        texts: list[str] = []
        first = not any(s is st for s, _, _ in groups[:len(out)])
        if first and st.intro:
            texts.append(st.intro)
        override = dict(st.text)
        for op in gops:
            t = override.get(op.tag, op.text)
            for line in (t, *op.lines):
                if line and line not in texts:
                    texts.append(line)
        if st.stage == WIRING:
            texts += [t for t in WIRING_TEXT if t not in texts]
        count = [body_of(_pid(p)) for op in gops if op.count for p in op.pieces
                 if body_of(_pid(p)) not in counted]
        counted.update(count)
        title = (dict(st.titles).get(gops[0].tag) if gops else None) or _title(st, gops, layer)
        if not gops and places and out and st.stage == STACK:
            title = f"Layer {layer}: {out[-1].title[0].lower()}{out[-1].title[1:]}"
        clip = {_pid(p): p.z for p in pieces if p.portion and p.z is not None}
        if places and st.stage == STACK and out:
            texts.insert(0, f"Put on the sub-assembly from step {out[-1].number}.")
        out.append(Step(len(out) + 1, title, st.stage, st.title, st.side or None, adds,
                        context, places, texts, st.where, sub, count, clip, layer,
                        [k for op in gops for k in op.extras],
                        next((op.scene for op in gops if op.scene is not None), None)))
        placed += list(adds)
        stage_of.update(dict.fromkeys(adds, key))
        if sub:
            pending = list(adds)
    # a portion shown before its body's other portions: clip the context too
    allclip = {k: v for s in out for k, v in s.clip.items()}
    for s in out:
        s.clip.update({k: allclip[k] for k in s.context + s.places if k in allclip})
    return out


def _title(st: Stage, gops: list[Op], layer: int | None) -> str:
    named = next((op.title for op in gops if op.title), "")
    if st.stage == STACK and gops and not gops[0].sub:
        if layer is not None and layer >= 0 and not named:
            return f"Layer {layer}"
        if layer is not None and layer >= 0:
            return f"Layer {layer}: {named[0].lower() + named[1:]}"
    if named:
        return named
    if st.stage == WIRING:
        return "Bus cables"
    short = st.title.split(": ", 1)[-1]
    return short[0].upper() + short[1:]


def prose(steps: list[Step]) -> list[str]:
    """The order as numbered paragraphs, one per stage, from a design's steps: the stage's
    title, then every sentence its steps say, once, in order, a bench sub-assembly's
    marked as such (what the docs quote: ``python -m spiderpig.guide.prose``)."""
    out: list[str] = []
    seen_stage: list[str] = []
    paras: dict[str, list[str]] = {}
    stacked: set[str] = set()       # the second side's stack says only what's new
    for st in steps:
        if st.stage_title not in paras:
            seen_stage.append(st.stage_title)
            paras[st.stage_title] = []
        para = paras[st.stage_title]
        for t in st.text:
            if t.startswith("Put on the sub-assembly from step"):
                continue
            if st.stage == STACK and t in stacked and t not in para:
                continue
            if st.stage == STACK:
                stacked.add(t)
            if st.sub:      # a bench sub-assembly says which
                where = st.title if "bench" in st.title else f"{st.title}, on the bench"
                t = f"{where}: {t[0].lower()}{t[1:]}"
            if t not in para:
                para.append(t)
    for i, title in enumerate(seen_stage):
        out.append(f"{i + 1}. **{title}.** " + " ".join(paras[title]))
    return out
