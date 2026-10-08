"""The assembly guide's step model: numbered steps generated from a fabricated robot and its
layer plan (prototype; docs/agentlib/GUIDE.md).

A :class:`Step` adds bodies (each body of the robot exactly once over the guide), shows the
bodies already in place as context, and says what it adds as :class:`Callout` lines (a part
type's label and quantity) and text. Steps come from :class:`Op` records: what a
construction says about putting its own parts on (a stage, a sort key, the bodies, a
sentence). In this prototype the ops come from one rule table over body names
(:data:`RULES`, standing in for each construction's ``assembly`` hook); the leg stack's
order is the plan's: every body not claimed by a rule joins the step of the layer its
bottom face sits in, bottom up, the gap under a layer going with that layer.
"""

from __future__ import annotations

import re
from collections import Counter, defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field

SIDES = ("L", "R")


@dataclass(frozen=True)
class Op:
    """A construction's say about some of its bodies: they go on together, in ``stage``
    (a robot-level phase), ordered by ``key`` within it, with ``text``. ``sub``: built
    on the bench first, drawn on its own, then placed (a sub-assembly)."""

    stage: tuple
    key: tuple
    bodies: tuple[str, ...]
    title: str
    text: str = ""
    sub: bool = False


@dataclass
class Callout:
    label: str          # the part type's identifier (P07, C03, H12)
    qty: int
    name: str           # what it is (catalog name, printed part, laser part)
    ref: str            # a body of the type (its thumbnail)


@dataclass
class Step:
    number: int
    title: str
    adds: list[str]
    context: list[str]
    text: list[str]
    view: str                       # "bench_L", "bench_R", "robot", "part"
    callouts: list[Callout] = field(default_factory=list)
    sub: bool = False               # a bench sub-assembly (no context)
    places: list[str] = field(default_factory=list)   # a sub-assembly put on in this step
    explode: float = 0.0            # how far the added parts are drawn lifted (mm)


# ---------------------------------------------------------------------------
# where a body sits in its side's stack
# ---------------------------------------------------------------------------


def side_of(name: str) -> str | None:
    return name[0] if re.match(r"^[LR]\.", name) else None


def bare(name: str) -> str:
    return re.sub(r"^[LR]\.", "", name)


def side_z(zlo: float, zhi: float, side: str, mid: float) -> tuple[float, float]:
    """A robot-z interval in its side's own coordinates (outer plate at 0, up the stack)."""
    return (zlo + mid, zhi + mid) if side == "L" else (mid - zhi, mid - zlo)


def slot_of(z0: float, layers: dict[int, tuple[float, float]], eps: float = 0.05) -> int:
    """The layer a body whose bottom face is at side-z ``z0`` goes on with: the first
    layer whose top is above it (the gap under a layer belongs to that layer)."""
    for k in sorted(layers):
        if z0 < layers[k][1] - eps:
            return k
    return max(layers) + 1


# ---------------------------------------------------------------------------
# the rules (each a construction's assembly hook, in the real thing)
# ---------------------------------------------------------------------------

Rule = Callable[[str, dict], "Op | None"]


def _crank_rules(names: list[str], side: str, z: dict,
                 slot: Callable) -> tuple[list[Op], list[str]]:
    """The bolt crank (construction.crank): each web but the hub plate is a bench unit
    with the hex standoff of the chain above it, that standoff's lower screw, washer and
    collar; the lowest web also takes the journal stub, its screw and thrust sleeve. The
    unit goes on at its web's layer. The hub plate (the highest web) goes with the servo."""
    webs = sorted((n for n in names if re.fullmatch(r"crank_plate\d+", bare(n))),
                  key=lambda n: z[n][0])
    if not webs:
        return [], []
    hub, rest = webs[-1], webs[:-1]
    ops = []
    for i, web in enumerate(rest):
        wz0 = z[web][0]
        unit = [web]
        for n in names:
            m = re.fullmatch(r"crank_pin_(J\w+)", bare(n))
            if m and abs(z[n][0] - wz0) < 1.0:       # the hex standing on this web
                j = m.group(1)
                unit += [n] + [f"{side}.crank_pin_{p}_lo_{j}" for p in
                               ("collar", "washer", "screw")
                               if f"{side}.crank_pin_{p}_lo_{j}" in z]
        text = ("Screw the hex standoff to the web from below (wide washer, threadlocker), "
                "the printed collar between them.")
        if i == 0:
            unit += [n for n in names if bare(n).startswith("crank_stub")]
            text = ("Screw the stub standoff to the lowest web (button head from above) and "
                    "slide its printed thrust sleeve over it, up to the web. " + text)
        ops.append(Op(("side", side, 1), (slot(wz0), 0), tuple(unit),
                      f"Crank web {bare(web).removeprefix('crank_plate')}", text, sub=True))
    return ops, [hub]


def side_ops(names: list[str], side: str, z: dict, layers: dict, hosts: dict) -> list[Op]:
    """Every op of one side's bodies (``names``, prefixed), each body in exactly one."""

    def slot(z0: float) -> int:
        return slot_of(z0, layers)

    taken: set[str] = set()
    ops: list[Op] = []

    def take(op: Op) -> None:
        ops.append(op)
        taken.update(op.bodies)

    def pick(pattern: str) -> list[str]:
        return [n for n in names if n not in taken and re.fullmatch(pattern, bare(n))]

    # the pillars (construction.pivots.standoff): the column first, on the bare plate
    take(Op(("side", side, 1), (-1, 0),
            tuple(pick(r"frame_outer") + pick(r"pillar_\w+_(standoff0|screw0|washer0)")),
            "Outer frame plate and pillar columns",
            "Lay the outer frame plate down, leg side up. Screw each pillar's standoff "
            "column to it: button head and washer from outside, threadlocker, 0.8 N·m "
            "while the column is bare to hold."))
    crank_ops, hub = _crank_rules(names, side, z, slot)
    for op in crank_ops:
        take(op)
    # the inner-plate unit: the servo, its horn, the deck rail, the frame ties' chains
    take(Op(("unit", side, 0), (0,),
            tuple(pick(r"torso") + pick(r"servo(_screw\d+)?")),
            "Servo onto the inner plate",
            "The servo on the inner plate, its two far front screws from the leg side."))
    take(Op(("unit", side, 0), (1,),
            tuple(pick(r"servo_horn\w*") + pick(r"crank_horn_\w+") + hub),
            "Horn and hub plate",
            "The horn on the spline with its centre screw; the hub plate on the horn, the "
            "horn screws up through it from below with their shims."))
    take(Op(("unit", side, 0), (3,),
            tuple(pick(r"deck_rail\w*") + pick(r"deck_insert\d+")),
            "Deck rail", "The deck rail on the inner plate, its two screws from the leg "
            "side, nuts in the rail; the heat-set inserts in the rail."))
    take(Op(("unit", side, 0), (4,),
            tuple(pick(r"tie_\w+")),
            "Frame-tie chains",
            "The frame ties' standoff chains: shims at the plate, the M3 button head up "
            "through the plate from the leg side, threadlocker."))
    take(Op(("join", side, 0), (0,),
            tuple(pick(r"pillar_\w+_(screw|washer)\d+")),
            "Unit onto the leg stack",
            "The unit onto its leg stack: the hub plate's hex pocket over the hub chain's "
            "standoff (turn the crank to line it up), the pillars' tops into the inner "
            "plate; each pillar's inner screw from the servo bay (a ball-end key)."))
    # Chicago pins go in with their host link
    by_host: dict[str, list[str]] = defaultdict(list)
    for n in pick(r"pin_\w+_screw"):
        by_host[hosts.get(n) or n].append(n)
    rest = [n for n in names if n not in taken]
    slots: dict[int, list[str]] = defaultdict(list)
    for n in rest:
        if n in z and not any(n in v for v in by_host.values()):
            slots[slot(z[n][0])].append(n)
    for host, pins in by_host.items():
        slots[slot(z[host][0]) if host in z else 0].extend(pins)
    for k in sorted(slots):
        ops.append(Op(("side", side, 1), (k, 1), tuple(sorted(slots[k], key=lambda n: z[n])),
                      f"Layer {k}", ""))
        taken.update(slots[k])
    missing = [n for n in names if n not in taken and n in z]
    assert not missing, missing
    return ops


def robot_ops(names: list[str]) -> list[Op]:
    """The chassis and the deck (construction.chassis, construction.deck)."""
    def pick(pattern: str) -> tuple[str, ...]:
        return tuple(n for n in names if re.fullmatch(pattern, n))

    return [
        Op(("chassis", None, 0), (0,), pick(r"tie_stud\d+|centre_plate\d+|[LR]\.rear_screw\d+"),
           "Centre plates and studs",
           "The M3 set-screw studs into the left chains' ends (threadlocker); the left "
           "servo's own centre plates on its rear face over the studs, its rear screw "
           "through them; the right servo's own plates screwed to it the same way, then that "
           "servo and its plates onto the studs, rear faces together."),
        Op(("deck", None, 0), (0,),
           pick(r"deck_(plate|standoff\d+|nut\d+|board_screw\d+|board|battery|cradle\w*|"
                r"charger|bms|switch)"),
           "Deck electronics (on the bench)",
           "The board on its nylon standoffs, the battery cradle screwed down (two M3 button "
           "heads through its ears, nuts under the deck), the battery, charger, BMS and "
           "switch.", sub=True),
        Op(("deck", None, 1), (0,), pick(r"deck_screw\d+"),
           "Deck onto the rails",
           "Bus cables first: each plug into its servo's socket along the centre plates' "
           "slot, up through the deck's wire slot. Lower the deck straight down between "
           "the inner plates past the pillars' inner heads onto the rails; its four screws "
           "into the rails' inserts."),
    ]


def rows_of(mech) -> list[dict]:
    """What :func:`steps` reads of a fabricated robot: each body's name, the robot-z
    interval of its placed part (None without one) and what it moves with."""
    rows = []
    for b in mech.bodies:
        z = None
        if b.part is not None:
            bb = b.placed_part().bounding_box()
            z = [float(bb.min.Z), float(bb.max.Z)]
        rows.append({"name": b.name, "z": z, "rigid_with": b.rigid_with})
    return rows


def layers_of(plan) -> dict[int, tuple[float, float]]:
    """The plan's layers' side-z intervals, the outside ones (-1, top + 1) included."""
    return {k: tuple(plan.z(k)) for k in range(-1, plan.top + 2)}


STAGE_ORDER = [("side", "L", 1), ("unit", "L", 0), ("join", "L", 0), ("chassis", None, 0),
               ("unit", "R", 0), ("side", "R", 1), ("join", "R", 0), ("deck", None, 0),
               ("deck", None, 1)]
VIEW = {"side": "bench", "unit": "unit", "join": "robot", "chassis": "robot", "deck": "robot"}


def steps(bodies: list[dict], layers: dict[int, tuple[float, float]], mid: float,
          labels: Callable[[list[str]], list[Callout]] | None = None) -> list[Step]:
    """The guide's steps for a robot. ``bodies``: ``name``, ``z`` (robot-z interval of
    the placed part, None without one), ``rigid_with``."""
    z_robot = {b["name"]: b["z"] for b in bodies if b["z"] is not None}
    hosts = {b["name"]: b["rigid_with"] for b in bodies}
    ops: list[Op] = []
    for s in SIDES:
        names = [n for n in z_robot if side_of(n) == s and not bare(n).startswith("rear_")]
        z = {n: side_z(z_robot[n][0], z_robot[n][1], s, mid) for n in names}
        ops += side_ops(names, s, z, layers, hosts)
    ops += robot_ops([n for n in z_robot if side_of(n) is None or bare(n).startswith(
        "rear_screw")])
    ops = [op for op in ops if op.bodies]
    seen = Counter(n for op in ops for n in op.bodies)
    dup = [n for n, c in seen.items() if c > 1]
    assert not dup, f"bodies in two steps: {dup}"
    ops.sort(key=lambda op: (STAGE_ORDER.index(op.stage), op.key, op.sub is False))
    out: list[Step] = []
    placed: list[str] = []
    stage_of: dict[str, tuple] = {}
    pending: list[str] = []          # a sub-assembly waiting for the step that places it
    for op in ops:
        kind, side = op.stage[0], op.stage[1]
        view = f"{VIEW[kind]}_{side}" if side and VIEW[kind] in ("bench", "unit") else "robot"
        # the context: what this stage's picture shows already in place
        if op.sub:
            context = []
        elif kind in ("side", "unit"):
            context = [n for n in placed if stage_of[n] == op.stage]
        else:
            context = list(placed)
        places: list[str] = []
        if not op.sub:
            places, pending = pending, []
            if kind == "join":       # the side's unit goes on now
                places = [n for n in placed if stage_of[n] == ("unit", side, 0)]
            context = [n for n in context if n not in places]
        title = op.title if not op.title.startswith("Layer") else _layer_title(op, side)
        text = [op.text] if op.text else []
        if kind == "side" and not op.sub and op.title.startswith("Layer"):
            text = [_layer_text(op)]
        if op.sub:
            text.append("Build this on the bench; it goes on in the next step.")
        elif places and kind == "side":
            text.insert(0, f"Put on the unit from step {out[-1].number} first.")
        out.append(Step(len(out) + 1, f"{'Left' if side == 'L' else 'Right'} side: {title}"
                        if side else title, list(op.bodies), context, text, view,
                        sub=op.sub, places=places,
                        explode=0.0 if op.sub or kind != "side" else 14.0))
        placed += list(op.bodies)
        stage_of.update(dict.fromkeys(op.bodies, op.stage))
        if op.sub:
            pending = list(op.bodies)
    if labels is not None:
        for st in out:
            st.callouts = labels(st.adds)
    return out


def _in_unit(name: str, steps_so_far: list[Step], side: str) -> bool:
    return any(name in st.adds for st in steps_so_far if st.view == f"unit_{side}")


def _layer_title(op: Op, side: str) -> str:
    return f"layer {op.title.split()[-1]}"


def _layer_text(op: Op) -> str:
    links = [bare(n) for n in op.bodies if re.fullmatch(r"b\d+_leg\d+", bare(n))]
    pins = [n for n in op.bodies if re.fullmatch(r"pin_\w+_screw", bare(n))]
    parts = []
    if links:
        parts.append(f"Place link{'s' if len(links) > 1 else ''} {', '.join(links)}")
    if pins:
        parts.append(f"{len(pins)} Chicago pin{'s' if len(pins) > 1 else ''} "
                     "(the barrel bonded into its host link, the screw from the cap side "
                     "once the link above is on)")
    rest = len(op.bodies) - len(links) - len(pins)
    if rest:
        parts.append(f"{rest} rings and spacers onto the pillars, pins and crankpins, "
                     "as drawn")
    return "; ".join(parts) + "."
