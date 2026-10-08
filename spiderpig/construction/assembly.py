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
from dataclasses import dataclass, field

STACK, UNIT, JOIN = "stack", "unit", "join"
CHASSIS, WIRING, DECK, OTHER = "chassis", "wiring", "deck", "other"
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
    ``where`` "bench" or "robot" (the picture: the stage alone, or everything so far)."""

    stage: str
    side: str | None
    title: str
    tags: tuple[tuple[str, ...], ...] = ()
    text: tuple[tuple[str, str], ...] = ()
    intro: str = ""
    where: str = "bench"


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
          tags=(("studs",), ("plates_L",), ("R.unit.servo", "plates_R")),
          text=(("studs", "The M3 set-screw studs into the left chains' ends "
                          "(threadlocker)."),
                ("servo", ""),
                ("plates_L", "The left servo's own centre plates on its rear face over the "
                             "studs, its rear screw through them."),
                ("plates_R", "The right servo's own plates screwed to the right servo the "
                             "same way, then that servo and its plates onto the studs, rear "
                             "faces together."))),
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
    "Each servo's bus plug into its socket, the cable along the centre plates' slot from "
    "their far edge and up to the board through the deck's wire slot (connector first), "
    "before the deck goes in.",
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
            ops += hook(view)
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
    robot = bool(mech.meta.get("robot")) if robot is None else robot
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
            ops.append(Op(op.stage, op.key, pieces, op.title, op.text, op.tag, op.sub, s,
                          op.count))
        clip_side.update({prefix + bare(r["name"]): s for r in mine})
    if robot:
        rv = RobotView({r["name"]: r["z"] for r in rows}, {r["name"]: r["rigid_with"]
                                                             for r in rows},
                       {r["name"]: r["fab"] for r in rows},
                       {r["name"]: r["bom_key"] for r in rows}, dict(mech.meta))
        ops += chassis.assembly(rv) + deck.assembly(rv)
        left = sorted(set(rv.z) - _claimed(ops))
        if left:
            ops.append(Op(OTHER, (), whole(*left), "Remaining parts",
                          "The remaining parts, where they are drawn.", tag="other"))
    _check(ops, {r["name"] for r in rows})
    return _order(ops, ROBOT_ORDER if robot else SIDE_ORDER)


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
            continue
        specs = list(st.tags)
        tags = {t for spec in specs for t in spec if "." not in t}
        extra = sorted({ops[i].tag for i in mine if ops[i].tag not in tags})
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
        if st.stage == WIRING:
            groups.append((st, [], None))
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
            if t and t not in texts:
                texts.append(t)
        if st.stage == WIRING:
            texts += [t for t in WIRING_TEXT if t not in texts]
        count = [body_of(_pid(p)) for op in gops if op.count for p in op.pieces
                 if body_of(_pid(p)) not in counted]
        counted.update(count)
        title = _title(st, gops, layer)
        clip = {_pid(p): p.z for p in pieces if p.portion and p.z is not None}
        if places and st.stage == STACK and out:
            texts.insert(0, f"Put on the sub-assembly from step {out[-1].number}.")
        out.append(Step(len(out) + 1, title, st.stage, st.title, st.side or None, adds,
                        context, places, texts, st.where, sub, count, clip, layer))
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


def prose(order: tuple[Stage, ...] = ROBOT_ORDER) -> list[str]:
    """The order as numbered paragraphs: each stage's title, its intro and its sentences
    (what the docs quote; the guide has the per-design steps)."""
    out = []
    for i, st in enumerate(o for o in order if o.stage != OTHER):
        parts = [st.intro] + [t for _, t in st.text]
        if st.stage == WIRING:
            parts += list(WIRING_TEXT)
        out.append(f"{i + 1}. {st.title}. " + " ".join(p for p in parts if p))
    return out
