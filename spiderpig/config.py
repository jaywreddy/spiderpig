"""The build: what to make and how, as one validated, hashable record.

:class:`BuildConfig` is the key every cache uses (side designs, baked
``.glb`` files, the MuJoCo model) and the argument every stage takes. It
validates itself on construction: the linkage exists (and walks, when a
robot is asked for), the module is one of the linkage's, the leg phases
(radians, one per leg) fit the module and are ``None`` when they are the
module's own, the proportions name the linkage's parameters, lengths are
positive, and only the overrides that differ from the defaults are kept,
in the linkage's order. So a design has exactly one config however it was
asked for, and a bad one fails where it is named (:class:`ParamError`, a
``ValueError``).

The CLIs share ``--linkage`` / ``--phases`` (degrees) / ``--proportion
NAME=VALUE`` and the build options (:func:`add_design_args`,
:func:`add_build_args`, :func:`config_from_args`); the server takes
``linkage=``, ``module=``, ``phases=`` and ``p.NAME=``
(:func:`design_from_query`).
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, fields, replace
from functools import cache

from spiderpig import linkage, servos
from spiderpig.fit import Params
from spiderpig.hardware.catalog import sheet_thickness


class ParamError(ValueError):
    """Bad design parameters (unknown linkage, module or proportion, wrong phase count, ...)."""


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


DEFAULT_CRANKS = {"walker": "bolt", "mechanism": "bolt", "arm": ""}
"""The crank a design gets when it names none, per kind (:func:`default_crank`): the bolt
crank for both (the crank study of 2026-10-03; mechanisms since 2026-10-04), which is the
single-plate web crank with hex-standoff crankpins in hex pockets (the user's decision of
2026-10-04 (1), ``BoltCrank.pin="hex"``; jam SF >= 2.25 on 0.100 in 6061-T6). The round
friction-clamped standoff it replaced is ``bolt_round``. Which construction a design gets is
data here, not code."""

LINKAGE_CRANKS: dict[str, str] = {
    # hoecken_pantograph and dwell_rocker plan with the hex crank since it merged
    # (2026-10-04; 9 layers each): the hub plate caps a crankpin under the horn spacer
    # (BoltCrank.hub_capped).
    #
    # TrotBot's heel and toe: b7 passes crankpin J1 at 10.2 mm; the hex crankpin's 8.5 mm
    # sleeve needs 11.2 (the static stage stops). The round 6 mm standoff clears it and plans
    # (its friction clamp: jam SF 3.57, UNVERIFIED); the hex crank at unit 12 rescales it.
    "trotbot_heel": "bolt_round",
    "trotbot_toe": "bolt_round",
}
"""Per linkage: the crank it gets when it names none, where :data:`DEFAULT_CRANKS`'
doesn't plan (each with why)."""


DEFAULT_PIVOTS: dict[str, tuple[str, str]] = {
    "walker": ("chicago", "standoff"),
    "mechanism": ("chicago", "standoff"),
    "arm": ("ball_pair", "ball_pair_pillar"),
}
"""The ``(pin, pillar)`` a design gets when it names none, per kind (:func:`default_pin`,
:func:`default_pillar`): a walker's and a mechanism's plain holes (the Chicago screw, the
standoff column; D1, 2026-10-07); an arm's ball bearings at every pivot (the user's call of
2026-10-11 for the palletizer: plain holes' play, at the arm's levers, is millimetres at the
tool; ``spiderpig precision --compare``), since 2026-10-14 the approved arm's: each pivot a
preloaded bearing pair on a ground pin (ISO 9283 RP 0.047 mm in a session; the single
``ball``: 0.2 mm), which also holds a crash at the tool where the single bearing's barrel
bent (the plain ``palletizer``'s elbow pin E, jam SF 0.44: DECISIONS.md 2026-10-14)."""

LINKAGE_PIVOTS: dict[str, tuple[str, str]] = {
    # empty since the approved arm's pivots became every arm's (2026-10-14)
}
"""Per linkage: the ``(pin, pillar)`` it gets when it names none, where its kind's
:data:`DEFAULT_PIVOTS` aren't its own design's (each with why)."""

LINKAGE_ROLES: dict[str, dict[str, str]] = {
    # the user's arm as approved on 2026-10-14: sensorless homing on hard stops, the camera
    # sets every zero (its turntable, mt4_compact, is every arm's)
    "palletizer_reach": {"angle_sensor": "stall",
                         "homing_reference": "vision_fiducial_ov5647"},
}
"""Per linkage: the keys its roles (:mod:`spiderpig.roles`) get when the config names none,
where the role's own default isn't its design's (:func:`roles.default_key`)."""

KIND_FIT: dict[str, dict[str, float]] = {
    "arm": {"link_radius": 6.6},
}
"""Per kind: the :class:`fit.Params` fields a design gets where :class:`fit.Params`' own
default doesn't build its :data:`DEFAULT_PIVOTS` (each with why): an arm's links are
6.6 mm in radius, a pillar's 10 mm bearing seat (``ball_pillar``, ``ball_pair_pillar``)
plus ``min_wall`` (``BearingSeat.check``). Applied when the config's ``params`` are the
plain defaults (:func:`default_params`); a spec's ``fit`` starts from them
(``FitSpec.params``)."""


def default_pin(lk: linkage.Linkage) -> str:
    """The pin construction ``lk`` gets when the config names none (a product's own,
    :data:`PRODUCT_PINS`; else :data:`LINKAGE_PIVOTS`, else :data:`DEFAULT_PIVOTS`)."""
    if lk.product in PRODUCT_PINS:
        return PRODUCT_PINS[lk.product]
    return LINKAGE_PIVOTS.get(lk.key, DEFAULT_PIVOTS[lk.kind])[0]


def default_pillar(lk: linkage.Linkage) -> str:
    """The pillar construction ``lk`` gets when the config names none
    (:data:`LINKAGE_PIVOTS`, else :data:`DEFAULT_PIVOTS`)."""
    return LINKAGE_PIVOTS.get(lk.key, DEFAULT_PIVOTS[lk.kind])[1]


def default_params(lk: linkage.Linkage, given: Mapping[str, float] | None = None) -> Params:
    """:class:`fit.Params` for ``lk``: :data:`KIND_FIT`'s fields for its kind under
    ``given`` (the fields a config or spec names)."""
    return Params(**{**KIND_FIT.get(lk.kind, {}), **dict(given or {})})


def config_pin(config: BuildConfig) -> str:
    """:func:`default_pin` of a config's linkage (the pin role's default)."""
    return default_pin(config.lk)


def config_pillar(config: BuildConfig) -> str:
    """:func:`default_pillar` of a config's linkage (the pillar role's default)."""
    return default_pillar(config.lk)


MODULE_CRANKS: dict[tuple[str, str], str] = {
    # empty since BoltCrank.hex_gap_fit (2026-10-05) put the Strider decker and quad on the
    # hex crank (git log has the debug)
}
"""Per (linkage, module): the crank it gets when it names none, ahead of
:data:`LINKAGE_CRANKS` (each with why)."""


CRANK_SHEET = "al6061_2p5mm"
"""The crank's plates when nothing names a sheet: 0.100 in 6061-T6, the thinnest whose hex
pockets hold the walkers' jam twist (2 x 0.85 N·m) at SF 2 (``materials.thinnest_sheet
("crank")``, 2026-10-04)."""

LINKAGE_CRANK_SHEETS: dict[str, str] = {
    # hoecken_pantograph (r4, 2026-10-05): its 12 mm crank puts crankpin M's hex pocket in
    # the hub plate 2.16 mm from the horn's screw holes (r 7 mm), under 1 x the 2.54 mm
    # sheet (a cut-rule error). On 0.080 in 6061 (2.03 mm) that web is over 1 x t (a
    # warning) and the hex holds the drive's torque at SF 3.89 (factor 1, a mechanism). The
    # round standoff plans but screws over the hub plate (no assembly order). The 0.100 in
    # sheet stays selectable (--crank-sheet).
    "hoecken_pantograph": "al6061_2mm",
}
"""Per linkage: the crank sheet it gets when the config names none, where :data:`CRANK_SHEET`
breaks a rule (each with why)."""


REMOVED = "2026-10-07"
PRUNED = "2026-10-14"           # the turret pruned to mt4_compact (the user's call)
_PIVOTS_GONE = {            # a removed pin or pillar key -> what it was
    "printed": "the printed snap-together axle",
    "rod": "the 3 mm rod with push-on clips",
    "bolt": "the M3 socket-head screw and nylock",
    "bearing": "the MF63ZZ flanged bearings on a 3 mm rod",
    "bushing": "the igus GFM-0304 bushings on a 3 mm rod",
    "ptfe": "the PTFE-lined 3 mm rod",
    "chicago_bushing": "the Chicago screw in igus GFM-0405 bushings",
    "standoff_hand": "the standoff column spliced in the stack by hand",
    "standoff_bench": "the standoff column spliced on the bench",
    "standoff_m3": "the spliced column of uxcell M3 standoffs",
}
REMOVED_CONSTRUCTIONS: dict[str, dict[str, tuple[str, str, str]]] = {
    "crank": {
        "printed": ("bolt", REMOVED, "the printed crankshaft held by clamp friction alone"),
        "keyed": ("bolt", REMOVED, "the printed crankshaft keyed by brass hex standoffs"),
        "keyed_float": ("bolt", REMOVED, "the keyed printed crankshaft with sliding keys"),
        "bolt_hub_screw": ("bolt", REMOVED, "the hex crank screwed over its hub plate"),
        "bolt_unretained": ("bolt", REMOVED, "the hex crank without its thrust sleeves"),
    },
    # the removed MF63ZZ "bearing" pivot's replacement is the ball pivot (2026-10-11)
    "pin": {k: ("ball" if k == "bearing" else "chicago", REMOVED, w)
            for k, w in _PIVOTS_GONE.items() if "standoff" not in k},
    "pillar": {k: ("ball_pillar" if k == "bearing" else "standoff", REMOVED, w)
               for k, w in _PIVOTS_GONE.items()
               if k not in ("ptfe", "chicago_bushing")},          # (those two: pins only)
    # an arm's turntable: only the approved mt4_compact kept (2026-10-14)
    "turntable": {k: ("mt4_compact", PRUNED, w) for k, w in {
        "belt_preload": "GT2 belt 5:1 on a preloaded 608 pair in the classic turret's hub",
        "mt4": "GT2 belt 5:1 on a plain 608 pair in the classic turret's hub",
        "capstan_preload": "cord capstan 11.6:1 on a preloaded 608 pair in the classic "
                           "turret's hub",
        "capstan_precise": "cord capstan 11.6:1 on a preloaded 608 pair, an Omron slot "
                           "sensor",
        "mt4_base": "GT2 belt 5:1 on the base's F608 pair, an optical endstop",
        "mt4_base_stall": "GT2 belt 5:1 on the base's F608 pair, homed on a hard stop",
    }.items()},
    "transmission": {
        "capstan": ("belt", PRUNED, "the Dyneema cord capstan on a laser-cut drum"),
    },
}
"""Construction keys removed from :mod:`construction` (D1, W2; the turret's, 2026-10-14),
per :class:`BuildConfig` field or :mod:`spiderpig.roles` role (a per-slot role's keys under
its plain name): ``key -> (replacement, date removed, what it was)``; a config, spec,
stored design or CLI option naming one fails with its replacement
(:func:`removed_construction`)."""

REMOVED_ROLES: dict[str, tuple[str, str]] = {
    "mill": (PRUNED, "the user's mill, which machined the classic turret's bearing housings"),
}
"""Roles removed from :mod:`spiderpig.roles`: ``name -> (date removed, what it chose)``; a
config, spec or stored design naming one fails with that (:func:`removed_role`)."""


REMOVED_PARAMS: dict[str, tuple[str, float, str]] = {
    "glue_fit": (REMOVED, 0.15, "a part glued into a laser-cut hole"),
    "neck_d": (REMOVED, 4.0, "the printed axle's neck where a link passes"),
    "head_d": (REMOVED, 8.5, "the printed axle's head outside the plates"),
    "journal_d": (REMOVED, 12.0, "the printed crank's body on O"),
    "stub_d": (REMOVED, 8.0, "the printed crank's journal stub"),
    "hub_thickness": (REMOVED, 5.0, "the printed crank's hub under the horn"),
}
"""Fields of :class:`construction.base.Params` (a spec's ``fit``) removed because nothing read
them once the printed axle and crank were gone (W5): ``name -> (date removed, its last
default, what it sized)``. A spec naming one is invalid with that message
(:func:`removed_param`); a stored design's resolved record, which writes every field in,
loads when it holds the old default and fails with the message otherwise."""


def removed_param(name: str, value=None) -> str | None:
    """Why ``fit.<name>`` can't be given any more, or ``None`` (not removed; or, with
    ``value``, the removed field's last default, which built the same parts)."""
    gone = REMOVED_PARAMS.get(name)
    if gone is None or (value is not None and value == gone[1]):
        return None
    when, default, what = gone
    return (f"fit.{name} ({what}) was removed on {when}: no construction reads it since the "
            f"printed axle and crank went; drop it (it built as {default:g}; "
            "config.REMOVED_PARAMS)")


def removed_role(name: str) -> str | None:
    """Why role ``name`` can't be given any more, or ``None`` (not removed)."""
    gone = REMOVED_ROLES.get(name)
    if gone is None:
        return None
    when, what = gone
    return (f"role {name!r} ({what}) was removed on {when}: drop it "
            "(config.REMOVED_ROLES)")


def removed_construction(field: str, key) -> tuple[str, str] | None:
    """(why ``field`` can't be ``key`` any more, its replacement), or ``None``."""
    gone = REMOVED_CONSTRUCTIONS.get(field, {}).get(key) if isinstance(key, str) else None
    if gone is None:
        return None
    replacement, when, what = gone
    return (f"{field} {key!r} ({what}) was removed on {when}; use {field}={replacement!r} "
            "(config.REMOVED_CONSTRUCTIONS)"), replacement


def default_crank_sheet(lk: linkage.Linkage) -> str:
    """The crank sheet ``lk`` gets when the config names none (a decanting cradle's: its
    keyed arm's two 0.125 in 6061-T6 plates; :data:`LINKAGE_CRANK_SHEETS`, else
    :data:`CRANK_SHEET`)."""
    if lk.product == CRADLE:
        return "al6061_3p2mm"
    return LINKAGE_CRANK_SHEETS.get(lk.key, CRANK_SHEET)


LINKAGE_TORQUE_LIMITS: dict[str, float] = {
    # klann_lego (2026-10-05): its 6061 leg b4 bends at D under the foot's 80 mm lever; at
    # the servo's 0.85 N·m limit the jam rates it SF 1.58 (strength.link_rows), and no
    # thicker 6061 fits its layer. The quad walks at 0.13 N·m (p99; 0.14 peak, 146 mm/s in
    # the sim), so 0.60 N·m keeps 4.2 x of headroom over walking (the Strider double's 0.85
    # over its 0.18 is 4.7 x) and puts every joint and link at jam SF >= 2 with no part
    # added or changed.
    "klann_lego": 0.60,
    # the mechanisms (2026-10-14, when their jam was first rated: strength.crank_loads, the
    # output blocked while the servo pushes): at the STS3215's 0.85 N·m every one of them
    # failed (jam SF 0.22-0.85: small cranks on acrylic links, levers up to 10 x near the
    # output's dead points). Each limit is the largest (rounded down) that puts every joint
    # and link at jam SF >= 2 with no part changed; a turn at full speed in MuJoCo
    # (sim.run.turn) peaks at 0.006-0.031 N·m, so each keeps 6.8 x (hoecken_pantograph) or
    # more of headroom over its own motion (klann_lego's rule: 4.2 x). What a mechanism
    # drives beyond its own links needs its limit raised and its links rated again.
    "hoecken": 0.36,
    "watt_crank": 0.19,
    "peaucellier_crank": 0.17,
    "parallelogram_lift": 0.19,
    "watt_table_lift": 0.19,
    "hoecken_pantograph": 0.21,
    "crank_rocker": 0.19,
    "rocker_amplifier": 0.09,
    "dwell_rocker": 0.11,
    "hoecken_table": 0.21,
}
"""Per linkage: a servo torque limit (N·m) under the servo's own
(:attr:`servos.spec.ServoSpec.torque_limit_nm`), where a joint or link holds a jam at SF 2
only below it (each with why): :func:`torque_limit_nm`."""


def torque_limit_nm(config: BuildConfig) -> float | None:
    """The torque limit to set in the robot's servo firmware (N·m), the one the jam loads
    and the joints' ratings assume: :func:`servo_torque_limit_nm` of its servo and its
    linkage, or a product's (:data:`PRODUCT_JAM_NM`: no servo, the hand through the
    gearbox); ``None`` for a servo with no stall torque."""
    if config.lk.product and config.linkage in PRODUCT_JAM_NM:   # no servo: the hand
        return PRODUCT_JAM_NM[config.linkage]                   # through the gearbox
    return servo_torque_limit_nm(config.servo, config.linkage)


def servo_torque_limit_nm(servo: str, linkage_key: str) -> float | None:
    """Servo ``servo``'s torque limit on linkage ``linkage_key`` (N·m): the servo's own
    (``TORQUE_LIMIT_FRACTION`` of stall, at most ``JAM_TORQUE_NM``), lowered to the
    linkage's :data:`LINKAGE_TORQUE_LIMITS` entry; ``None`` for a servo with no stall
    torque. (An exported MJCF's meta names both: :func:`sim.run.turn` reads them there.)"""
    limit = servos.get(servo).torque_limit_nm
    own = LINKAGE_TORQUE_LIMITS.get(linkage_key)
    if limit is None or own is None:
        return limit
    return min(limit, own)


def torque_limit_note(config: BuildConfig) -> str | None:
    """What the builder must set for :func:`torque_limit_nm` (a BOM / ORDER.md note), or
    ``None`` when the design's motors aren't a crank's servo (its motor role, not
    ``servo``: :func:`roles.motor_role`; an arm's steppers have no firmware limit) or the
    servo has no limit. Every crank's servo gets it: a walker's, a walker side's and a
    mechanism's (2026-10-14; a walker's robot alone before, so a mechanism's far lower
    limit, :data:`LINKAGE_TORQUE_LIMITS`, never reached its BOM)."""
    from spiderpig import roles as rolemod

    if rolemod.motor_role(config.lk) != "servo":
        return None
    limit = torque_limit_nm(config)
    spec = servos.get(config.servo)
    stall = spec.stall_torque_nm
    if limit is None or not stall or config.lk.product:
        return None
    why = (" (lowered for this linkage: config.LINKAGE_TORQUE_LIMITS)"
           if config.linkage in LINKAGE_TORQUE_LIMITS else "")
    jam = "a jammed foot" if config.lk.walks else "a blocked output"
    return (f"Servo firmware: before the first run, set the torque limit of every servo "
            f"({spec.name.split(' (')[0]}) to {limit:g} N·m, {limit / stall * 100:.0f} % of "
            f"its {stall:.2f} N·m stall{why}: every joint and link is rated (spiderpig audit, "
            f"strength) at {jam} holding the servo at that limit.")


def config_crank(config: BuildConfig) -> str:
    """:func:`default_crank` of a config's linkage and module (the crank role's default)."""
    return default_crank(config.lk, config.module)


LINKAGE_OBJECTIVES: dict[str, str] = {
    # none yet: a walker on "height" (its true lowest stack) searches every taller size
    # with no route-cost pruning; the Strider double's proof doesn't end in minutes
    # (DECISIONS.md 2026-10-14)
}
"""Per linkage: the planner's objective where its kind's isn't wanted (each with why):
:func:`plan_objective`."""


def plan_objective(lk: linkage.Linkage) -> str:
    """The planner's objective (:attr:`stack.StackSpec.objective`,
    :data:`stack.plan.OBJECTIVES`) for ``lk``: its own (:data:`LINKAGE_OBJECTIVES`), else a
    product's (a decanting cradle's) "height", the lowest stack (then the route, the stock
    parts' cost, the prints), proven in seconds; else, with no crank (an arm),
    "layers_height", the fewest layers, then the lowest stack in them, then the shortest
    axle spans (2026-10-21: with no crank router there is no route cost to prune, so
    "layers" kept the first plan its search found: the merged search's was taller than
    master's and the arm's elbow pin's span longer; on "height" the turret's proof doesn't
    end in its budget); else a walker's or a mechanism's "layers", the fewest layers, then
    the cheapest crank route (equal plans found rank by their height; 2026-10-14: on
    "height" a walker's crank router can't prune on its route's cost, and its taller sizes,
    which may be lower, are searched without end)."""
    if lk.key in LINKAGE_OBJECTIVES:
        return LINKAGE_OBJECTIVES[lk.key]
    if lk.product:
        return "height"
    return "layers" if lk.crank else "layers_height"


def default_crank(lk: linkage.Linkage, module: str = "") -> str:
    """The crank construction ``lk`` (in ``module``) gets when the config names none: a
    product's (:data:`PRODUCT_CRANKS`: a decanting cradle's hand crank), its module's
    (:data:`MODULE_CRANKS`), else its own (:data:`LINKAGE_CRANKS`), else its kind's
    (:data:`DEFAULT_CRANKS`)."""
    if lk.product in PRODUCT_CRANKS:
        return PRODUCT_CRANKS[lk.product]
    return MODULE_CRANKS.get((lk.key, module),
                             LINKAGE_CRANKS.get(lk.key, DEFAULT_CRANKS[lk.kind]))


CRADLE = "cradle"
SHEET = "acrylic_3mm"
PRODUCT_CRANKS: dict[str, str] = {CRADLE: "hand"}
"""Per product (:attr:`linkage.Linkage.product`): the crank it is built with, the only one
(a decanting cradle: the hand-cranked worm drive, :mod:`construction.handcrank`)."""

FRAME_SHEET = "al5052_2mm"
PRODUCT_FRAME_SHEETS: dict[str, str] = {CRADLE: "al5052_4p8mm"}
"""Per product: the frame plates' sheet when the config names the default
(:data:`FRAME_SHEET`). A decanting cradle's frame plates are its stand (2026-10-10): 0.187 in
5052, 3.4 x as stiff across as 0.125 in (the side's sway under the hand and a bumped bottle)
and still cut with the M5 screws' 5.5 mm holes (SendCutSend: a hole at least the
thickness; 0.250 in would need 6.35)."""

PRODUCT_PARAMS: dict[str, dict[str, float]] = {
    CRADLE: {"link_radius": 12.0, "frame_radius": 12.0, "spacer_d": 12.5},
}
"""Per product: the shared dimensions (:class:`fit.Params`) a design gets when it names
the defaults. A decanting cradle's links are 22 mm wide (two plate thicknesses of web round
its pins' 8.2 mm holes, the 8 mm pivot pin's or the M6 Chicago barrel's: SendCutSend's rule)
and its frame arms 24 mm; its pins' spacer rings 12.5 mm (the 8 mm pin and a wall)."""

PRODUCT_PINS: dict[str, str] = {CRADLE: "rod516"}
"""Per product: the link pin a design gets when it names the default (``chicago``): a
decanting cradle's 5/16 in 6061-T6 rod pins, cut to their stacks, an M3 button head and a
washer on each tapped end (:data:`construction.pivots.rod_pin.ROD516`, simplification round
1, 2026-10-21: the user's materials rule, no steel in a structural part; the 8 mm hardened
steel pivot pins of 2026-10-13, ``pivot8``, and ``chicago6`` are still there, ``--pin``), but
at :data:`PRODUCT_PIN_JOINTS`."""

PRODUCT_PIN_JOINTS: dict[str, dict[str, str]] = {
    "decanter_rcm": {"X": "rodclip516", "Y": "rodclip516", "P": "rod38"},
    "decanter_hinge": {"E": "rod38"},
}
"""Per product linkage, with its product's own pin (:data:`PRODUCT_PINS`): the joints that take
another pin instead. B's X and Y, where its cradle link meets each side, take a
quick-release pin (``rodclip516``: a hairpin cotter through the rod's drilled hole; the
clevis pins' since 2026-10-13: the passive side panel slides off its cradle link once their
two clips are out, ``cradle_pack``). The rod pins P (B) and E (A) are 3/8 in (``rod38``): in
5/16 in 6061 they hold the clutch's cap at about 1.0 (the hardened 8 mm pin had held 2.75).
K, J2 and J take the 5/16 in screw-ended pin (simplification round 1: K and J2 had been
18-8 clevis pins for their strength alone, design review round 1; a clip at J stands in the
cradle link's layer under b6's rear arm, no plan)."""

PRODUCT_CHORD_R: dict[str, dict[str, dict[tuple[str, str], float]]] = {
    "decanter_rcm": {"b1": {("M", "P"): 13.0}, "b3": {("O1", "J"): 12.5, ("O1", "J2"): 12.5},
                     "b4": {("X", "K"): 14.0}, "b5": {("Y", "J2"): 13.0},
                     "b6": {("X", "Tr1"): 32.0, ("X", "Y"): 16.0, ("Tr1", "Tr2"): 32.0,
                            ("Tr2", "Br"): 38.0}},
    "decanter_hinge": {"b1": {("M", "E"): 13.0}, "b2": {("E", "Tr1"): 16.0}},
}
"""Per product linkage, per link: a chord (a pair of its joints, either way round) cut wider
than ``Params.link_radius``, its own half-width (mm), each from the cradle's links rated
in bending (design review round 1, 2026-10-14; ``cradle_report.structure``):
- B's cradle link b6: its rear arm X-Tr1, a cantilever off X carrying the rear saddle, 64
  mm wide (24 held the jam's couple at SF 0.3 in 0.125 in; 40 held the clutch's cap at 1.15
  on 5052's minimum, design review round 3; 48 until simplification round 2, 2026-10-21, when
  the racking brace's couple came in at its socket Br, behind the rear tie: the arm and the
  rear chords Tr1-Tr2 (64) and Tr2-Br (76, the socket's pad) carry it to X in bending, the
  arm's boss at X, :data:`PRODUCT_BOSS_R`, spreading it into the front); its chord X-Y, 284
  mm in compression under the jam, 32 mm wide against buckling out of its plane.
- A's cradle link b2: its chord E-Tr1, in compression from the rod's pin to the rear tie
  under the jam, 32 mm wide against buckling (A has no racking brace: its hand crank sweeps
  the bay's drive end behind the rear saddle, where B's goes; ``decanters.BRACE``).
- B's bar b4 (simplification round 1, 2026-10-21: in 0.187 in 5052): its chord X-K, in
  compression under the jam, 28 mm wide against buckling out of its plane, its bosses'
  width already (the blank unchanged); 24 mm in 5052 held the cap at 1.42 nominal.
- B's cradle link b5, 26 mm wide (the same round; design review round 3's laser-cut parts
  minor 3): at 24 mm it came as cut, not deburred.
- B's arm b3 (simplification round 2, 2026-10-21: in 6061-T6 0.187 in, out of 7075): its
  chords O1-J and O1-J2 25 mm wide, its bending at O1 at the clutch's cap SF 1.37 on
  6061's minimum (1.27 at 24 mm); 26 mm and wider met b2 or pin K's clevis in their shared
  layer (round 1), 25 plans in B's 15 layers, proven optimal.
- Each rod b1 (0.250 in), 26 mm wide (design review round 2, 2026-10-15, laser-cut parts
  M3): SendCutSend deburrs a part of at least 1 x 3 in; at 25.2 mm (24 and its crank
  bore's boss) it came as cut, its exit face's dross on its crankpin's spacer.
The planner claims each at its width (:class:`construction.plates.LinkPlates`)."""

PRODUCT_BOSS_R: dict[str, dict[str, dict[str, float]]] = {
    "decanter_rcm": {"b1": {"M": 16.2}, "b2": {"O2": 17.0},
                     "b3": {"O1": 17.0, "J": 14.0, "J2": 14.0}, "b6": {"X": 50.0, "Br": 38.0}},
    "decanter_hinge": {"b1": {"M": 16.2}, "b2": {"G": 17.0}},
}
"""Per product linkage, per link: a boss (a disc, its radius in mm) round the hole at a
joint, where the link's width left the hole's edge in the cut rules' warning band (design
review round 2, 2026-10-15, laser-cut parts minor 1; SendCutSend: two thicknesses of
material from a hole to an edge, in 7075 "to prevent microcracks"):
- each rod b1 at its crank bore M (13.05 mm since simplification round 1's 1/2 in crankpin,
  r 16.2; 12.35 and r 15.8 before): 9.7 mm of web, 1.5 x its 0.250 in (6.45 at 26 mm wide);
- B's arms b2 at O2 and b3 at O1, A's cradle link b2 at G, each a pillar's 12.35 mm bore
  in 0.187-0.190 in: 10.8 mm, 2.2 x (5.8 at 24 mm).
- B's 7075 links b3 and b4 at every 8.2 mm pin hole, r 14 (design review round 3,
  2026-10-20, laser-cut parts M3: 9.9 mm, 2.05 x the 0.190 in; at 24 mm wide 7.9, 1.64 x,
  under SendCutSend's 7075 rule; b4's blank then 28 mm, over the 25.27 mm it deburrs). b4
  needs none since simplification round 1 (2026-10-21): in 5052, its whole chord X-K is 28
  mm wide (:data:`PRODUCT_CHORD_R`), the bosses' width. b3 is 6061 since round 2, whose
  2 t is 7075's rule only: its J and J2 bosses stay (they don't set its 312 x 34 blank,
  O1's does, and its plan was proven with them).
- B's cradle link b6 at X, r 50 (simplification round 2, 2026-10-21): the racking brace's
  couple reaches the front of the link only through X (the rear arm hangs off it alone: no
  chord from the front ties to the rear clears pillar O1), and the boss spreads it into the
  chord Tf2-X, the one that passes pillar O1 by the planner's margin and can't grow
  (:func:`cradle_structure.link_bending` and the box's rows rate its end past the boss; the
  arm's 48 mm end had been the boss there: 1.22 at the clutch's cap with the brace).
- B's racking brace's socket at Br (the same round), r 38: 11.4 mm of web round the 53.2 mm
  toothed socket (the chord Tr2-Br's own end, 76 mm wide, until the way back cut b6 to its
  band, :data:`PRODUCT_BAND`, which keeps this disc whole).
Each outline is closed round its boss and the planner claims the boss as a disc
(:func:`construction.plates.product_bosses`)."""


PRODUCT_BAND: dict[str, dict[str, tuple[str, str, float, float]]] = {
    "decanter_rcm": {"b6": ("Lc", "Ax", -162.5, 49.0)},
}
"""Per product linkage, per link: the band its outline is cut to, ``(o, x, lo, hi)``: between
the lines ``y = lo`` and ``y = hi`` (mm) in the frame with its origin at joint ``o`` and its x
toward joint ``x``. B's cradle link b6 in the cradle's frame (the mouth Lc at the origin, x
along the bottle's axis: ``decanters.TIES``'), simplification round 2's way back (2026-10-21):
the racking brace's boss at X (r 50) and its wider rear chords (X-Tr1 and Tr1-Tr2 64 mm,
Tr2-Br 76) had stood the cradle box 249 mm tall in the pack (178 before them). Cut to the
band, b6 keeps all it needs inward: the band's bottom 0.5 mm under Br's pad (r 38 round the
socket, :data:`PRODUCT_BOSS_R`: the socket's web whole), its top 1 mm under the front saddle's
lip gauge's top, the box's highest part but b6; the cut takes X's boss's top, the rear arm's
round ends and Tr1-Tr2's and Tr2-Br's lower edges near Tr2. ``cradle_structure`` rates every
section on what the cut leaves (:class:`cradle_structure.Band`)."""


def band(config, link: str) -> tuple[str, str, float, float] | None:
    """The band ``config`` cuts link ``link``'s outline to (:data:`PRODUCT_BAND`; none for a
    walker)."""
    from spiderpig.stack import body_class

    return PRODUCT_BAND.get(config.linkage, {}).get(body_class(link))


def boss_r(config, link: str) -> dict[str, float]:
    """The bosses ``config`` grows on link ``link``: joint -> radius (mm)
    (:data:`PRODUCT_BOSS_R`; none for a walker)."""
    from spiderpig.stack import body_class

    return dict(PRODUCT_BOSS_R.get(config.linkage, {}).get(body_class(link), {}))


def chord_r(config, link: str, a: str, b: str) -> float:
    """The half-width (mm) ``config`` cuts link ``link``'s chord ``a``-``b`` (joint or
    topology point names: a lone point is ``link.joint``): :data:`PRODUCT_CHORD_R`'s, else
    ``Params.link_radius``."""
    from spiderpig.stack import body_class

    own = PRODUCT_CHORD_R.get(config.linkage, {}).get(body_class(link), {})
    if own:
        a, b = a.rsplit(".", 1)[-1], b.rsplit(".", 1)[-1]
        r = own.get((a, b), own.get((b, a)))
        if r is not None:
            return r
    return config.params.link_radius

PRODUCT_SHEETS: dict[str, str] = {CRADLE: "al5052_3p2mm"}
"""Per product: the default sheet (the layer pitch, the printed rings' and the order's
material) when the config names the default (:data:`SHEET`): a decanting cradle is all
aluminium, its links' 0.125 in 5052 (:data:`materials.CRADLE_LINK_SHEET`)."""

PRODUCT_JAM_NM: dict[str, float] = {"decanter_hinge": 64.9, "decanter_rcm": 63.7}
"""Per product linkage: the crank torque its joints are rated at in a jam, in place of a
servo's firmware limit (:func:`torque_limit_nm`): a decanting cradle's, the most its hand
crank's slip clutch lets through, however hard the push (:data:`PRODUCT_CLUTCH_NM`, since
design review round 1; a firm 40 N push at the grip had been the case:
:func:`spiderpig.cradle_report.jam_nm`, which a test holds these to)."""

PRODUCT_CLUTCH_NM: dict[str, float] = {"decanter_hinge": 64.9, "decanter_rcm": 63.7}
"""Per product linkage: the most its hand crank's slip clutch lets onto the crank, however
hard the push (design review round 2's B-1, 2026-10-14): set to slip at
``construction.handcrank.CLUTCH_SET`` x the start the user measures (a full 6 L's, drawn:
``handcrank.DESIGN_START_NM``), its check's pulls never over the ceiling
(``handcrank.clutch_ceiling``, design review round 3's M1, 2026-10-20: so a gearbox set new
and run in after, or a start read high, can't take it over this), checked within
``CLUTCH_CHECK`` on its stub at every set-up, monthly and after greasing, through the NRV050's
30:1 worm at the hand's 100 rpm (``handcrank.HAND_RPM``: ``handcrank.clutch``, which a test
holds these to; spun at 200 rpm, 68.1 and 69.1: ``cradle_drive``'s ``fast_cap_nm``, the use
limits' "turn it, don't spin it"). Under the gearbox's 85 N·m maximum with margin
(``cradle_drive.GEARBOX_CAP_FLOOR``). The crank torque to re-run a strength check at: a bound,
unlike the push (``cradle_report``'s ``jam_capped`` rates it over the whole turn, the
toggles included), while the check is kept (unchecked, the friction can carry it to the
gearbox's maximum: ``handcrank.clutch_drift``). 67.2 and 63.9 until simplification round 1:
the lighter cradle, its aluminium posts, tie rods, rear post and pins
(``handcrank.DESIGN_START_NM``); B's 61.8 until round 2's racking brace (its tube and its
cradle links' wider rear, 0.46 kg on the box: 63.9), then its way back (b6 cut to its band:
63.7)."""


def product_defaults(product: str) -> dict:
    """The fields a product sets where the config names the default (the sheets, the pin,
    the shared dimensions): what :class:`BuildConfig` fills in, and what a design moved to
    another linkage drops (:func:`own_constructions`)."""
    out: dict = {}
    if product in PRODUCT_SHEETS:
        out["sheet"] = PRODUCT_SHEETS[product]
    if product in PRODUCT_FRAME_SHEETS:
        out["frame_sheet"] = PRODUCT_FRAME_SHEETS[product]
    if product in PRODUCT_PINS:
        out["pin"] = PRODUCT_PINS[product]
    return out


def merged_params(params: Params, product: str) -> Params:
    """``params`` with a product's dimensions (:data:`PRODUCT_PARAMS`) in every field it
    leaves at the default: one fit override keeps the rest of the product's."""
    mine = PRODUCT_PARAMS.get(product, {})
    base = Params()
    return replace(params, **{k: v for k, v in mine.items()
                              if getattr(params, k) == getattr(base, k)})


def is_cradle(config) -> bool:
    """Whether ``config`` builds a decanting cradle (:mod:`construction.cradle`)."""
    return config.lk.product == CRADLE
ROLE_FIELDS: dict[str, str] = {"servo": "servo", "crank_sheet": "crank"}
""":class:`BuildConfig` fields that belong to a role (:mod:`spiderpig.roles`): the drive
servo and the crank's sheet, a walker's and a mechanism's. Where the role isn't a slot of
the design's kind (an arm: steppers, the role ``actuator``, and no crank) the field holds
its default and naming another is refused (:func:`unused_field`). The default stays in the
field, the key and the resolved spec rather than being left out: every stored arm's id
(``BuildConfig.key``, ``design.spec_hash``, ``side_hash``) hashes it, so leaving it out
would move them all (2026-10-14)."""


def role_field_default(lk: linkage.Linkage, name: str) -> str:
    """The value field ``name`` of :data:`ROLE_FIELDS` holds for ``lk`` when nothing names
    it."""
    return servos.DEFAULT if name == "servo" else default_crank_sheet(lk)


def unused_field(lk: linkage.Linkage, name: str, value) -> str | None:
    """Why ``value`` can't be ``lk``'s field ``name`` (:data:`ROLE_FIELDS`: its role isn't a
    slot of ``lk``'s kind and the value isn't the field's default), or ``None``."""
    from spiderpig import roles as rolemod

    role = ROLE_FIELDS.get(name)
    if role is None or rolemod.applies(role, lk.kind) or value in (None, ""):
        return None
    if value == role_field_default(lk, name):
        return None
    r = rolemod.get(role)
    article = "an" if lk.kind[0] in "aeiou" else "a"
    return (f"{lk.key} is {article} {lk.kind}: it has no {r.title} ({name} {value!r}; the role "
            f"{role!r} is a {' or '.join(r.kinds)}'s"
            + ("; its motors are the role 'actuator'" if name == "servo"
               and rolemod.applies("actuator", lk.kind) else "") + ")")


ROBOT_LAYOUTS: dict[str, str] = {"walker": "mirrored_pair", "arm": "turret"}
"""The robot each linkage kind makes (:data:`construction.robot.LAYOUTS`): a walker's two
mirrored sides with the chassis between their servos; an arm's side standing on a turntable
(2026-10-12). A mechanism makes none: it is one side."""


def robot_layout(lk: linkage.Linkage) -> str:
    """The robot ``lk`` makes (:data:`ROBOT_LAYOUTS`), ``""`` for none: ``"turret"`` is the
    one whose robot is a choice (``legs.robot``, the turntable), not a second side."""
    return ROBOT_LAYOUTS.get(lk.kind, "")


@dataclass(frozen=True)
class BuildConfig:
    """What to build and how. Construction keys refer to :mod:`construction` registries."""

    linkage: str = linkage.DEFAULT    # see linkage.available()
    module: str = ""                  # legs per side: one of the linkage's modules ("": its
    #                                   default, config.default_module: Strider's double)
    robot: bool = None  # type: ignore[assignment]   # the whole robot (ROBOT_LAYOUTS: a
    #                                   walker's two mirrored sides, servos back to back in one
    #                                   frame; an arm's side on its turntable), else one side;
    #                                   None: the linkage's kind's (default_robot: a
    #                                   mechanism is one side), resolved before any key
    phases: tuple[float, ...] | None = None          # crank phase per leg (rad); None = module's
    proportions: tuple[tuple[str, float], ...] = ()  # overrides of the linkage's params
    sheet: str = SHEET                # the default sheet (the links, rings, deck): it sets
    #                                   the layer pitch (spiderpig.materials)
    thickness: float | None = None    # override the sheet's nominal thickness
    frame_sheet: str = FRAME_SHEET      # the frame and centre plates (aluminium: acrylic
    #                                     can't take their load; the user's call 2026-10-04):
    #                                     0.080 in 5052, the thinnest stock that passes
    #                                     (materials.thinnest_sheet("frame"); tests pin it)
    crank_sheet: str = ""               # the crank's laser-cut plates ("": default_crank_sheet:
    #                                     the linkage's, LINKAGE_CRANK_SHEETS, else CRANK_SHEET,
    #                                     0.100 in 6061-T6, the thinnest whose hex pockets hold
    #                                     the jam twist at SF 2: materials.thinnest_sheet
    #                                     ("crank"), 2026-10-04)
    heads: str = "best"               # fasteners' heads: "sink" into the layer beside their
    #                                   link, "gap" (a thin clearance gap where a link passes),
    #                                   "best" (sunk, else in gaps; stack.StackSpec)
    link_sheets: tuple[tuple[str, str], ...] | None = None   # link class -> sheet; None: the
    #                                   linkage's (materials.default_link_sheets: a Klann
    #                                   variant's foot links in 6061)
    servo: str = servos.DEFAULT
    pillar: str = ""                  # frame pivots ("": default_pillar, the kind's,
    #                                   DEFAULT_PIVOTS: a walker's 6 mm round standoff column,
    #                                   a stock goBILDA one or else one steel standoff made to
    #                                   length, never spliced, construction.pivots.standoff;
    #                                   an arm's ball_pair_pillar, MR106ZZ pairs in hubs
    #                                   beside its links on a ground shaft)
    pin: str = ""                     # pivots between links ("": default_pin, the kind's: a
    #                                   walker's M3 Chicago screw (4 mm barrel), printed rings
    #                                   and head spacers, construction.pivots.chicago; an
    #                                   arm's ball_pair, preloaded MR84ZZ pairs on a ground
    #                                   pin)
    crank: str = ""                   # the crankshaft ("": default_crank: the linkage's own,
    #                                   LINKAGE_CRANKS, else its kind's, DEFAULT_CRANKS: "bolt",
    #                                   construction.crank.BoltCrank, single aluminium web plates
    #                                   on the crank sheet, hex-standoff crankpins in hex
    #                                   pockets; "bolt_round": the round friction-clamped
    #                                   standoff). The keys removed on 2026-10-07:
    #                                   REMOVED_CONSTRUCTIONS
    roles: tuple[tuple[str, str], ...] = ()   # (role, construction key) for every role
    #                                   without a field of its own (spiderpig.roles: the
    #                                   later kinds of part), only those that differ from
    #                                   the role's default, sorted
    params: Params = field(default_factory=Params)

    def __post_init__(self) -> None:
        try:
            lk = linkage.get(self.linkage)
        except KeyError as e:
            raise ParamError(e.args[0]) from None
        if self.robot is None:
            object.__setattr__(self, "robot", lk.kind in ROBOT_LAYOUTS)   # (default_robot)
        if self.robot and lk.kind not in ROBOT_LAYOUTS:
            # a cradle is one side too (robot=False): construction.cradle builds both from it
            raise ParamError(f"{lk.key} is a {lk.kind}, not a walker or an arm: no robot is made "
                             f"of it (a walker's two sides, an arm on its turntable), so it "
                             f"builds one side (robot=False; --side-only on the command line) "
                             f"(walkers: {', '.join(linkage.available('walker'))}; arms: "
                             f"{', '.join(linkage.available('arm'))})")
        if not self.module:
            object.__setattr__(self, "module", default_module(lk.key))
        if not self.crank:
            object.__setattr__(self, "crank", default_crank(lk, self.module))
        if not self.crank_sheet:
            object.__setattr__(self, "crank_sheet", default_crank_sheet(lk))
        if lk.product:          # a product's own sheets where the config names the plain ones
            plain = {"sheet": SHEET, "frame_sheet": FRAME_SHEET}
            for what, value in product_defaults(lk.product).items():
                if what in plain and getattr(self, what) == plain[what]:
                    object.__setattr__(self, what, value)
        if not self.pin:
            object.__setattr__(self, "pin", default_pin(lk))
        if not self.pillar:
            object.__setattr__(self, "pillar", default_pillar(lk))
        if self.params == Params() and lk.kind in KIND_FIT:
            object.__setattr__(self, "params", default_params(lk))
        if lk.product:          # ... and its shared dimensions where the config names none
            object.__setattr__(self, "params", merged_params(self.params, lk.product))
        hand = PRODUCT_CRANKS.get(lk.product)
        if (self.crank == "hand") != (hand == "hand"):
            cradles = [k for k in linkage.available() if linkage.get(k).product]
            raise ParamError(f"crank {self.crank!r}: the hand crank builds a decanting cradle "
                             f"and a cradle only that ({lk.key} is "
                             f"{'a cradle' if hand else 'not'}; cradles: {', '.join(cradles)})")
        for what in ("crank", "pin", "pillar"):
            if (gone := removed_construction(what, getattr(self, what))) is not None:
                raise ParamError(gone[0])
        if self.module not in lk.leg_modules:
            raise ParamError(f"unknown module {self.module!r}; have {list(lk.leg_modules)}")
        if self.servo not in servos.available():
            raise ParamError(f"unknown servo {self.servo!r}; have {servos.available()}")
        object.__setattr__(self, "phases", self._phases(lk))
        for what in ROLE_FIELDS:
            if (why := unused_field(lk, what, getattr(self, what))) is not None:
                raise ParamError(why)
        if lk.ranged and (self.crank or self.phases is not None):
            raise ParamError(f"{lk.key} is an arm: its inputs drive links, so it has no crank "
                             f"and no crank phases (crank {self.crank!r}, phases "
                             f"{self.phases})")
        object.__setattr__(self, "proportions", self._proportions(lk))
        object.__setattr__(self, "link_sheets", self._link_sheets(lk))
        object.__setattr__(self, "roles", self._roles(lk))
        if self.heads not in ("best", "sink", "gap"):
            raise ParamError(f"heads must be best, sink or gap, got {self.heads!r}")
        from spiderpig.hardware.catalog import get

        for what in ("sheet", "frame_sheet", "crank_sheet"):
            key = getattr(self, what)
            try:
                if get(key).category != "sheet":
                    raise KeyError(key)
            except KeyError:
                raise ParamError(f"{what} {key!r} is not a sheet in the catalog") from None
        from spiderpig.materials import sheet

        if not sheet(self.crank_sheet).metal:
            raise ParamError(f"crank_sheet {self.crank_sheet!r} is not metal (the acrylic crank "
                             f"was removed on {REMOVED}): an aluminium sheet, e.g. {CRANK_SHEET!r}")

    def _roles(self, lk: linkage.Linkage) -> tuple[tuple[str, str], ...]:
        """The roles named, validated: each a role of :mod:`spiderpig.roles` kept in
        ``roles`` (a role with a field of its own is set there), or a per-slot role's
        ``"<role>@<slot>"``, applying to this linkage's kind; :data:`roles.NONE` only for an
        optional role; a plain key equal to the role's default is dropped (one config per
        design). The keys are checked against the role's registry where it is built, as
        the pins are (the slots too: they are the design's)."""
        from spiderpig import roles as rolemod

        out = {}
        for qualified, key in (dict(self.roles) if self.roles else {}).items():
            name, slot = rolemod.split(str(qualified))
            if (why := removed_role(name)) is not None:
                raise ParamError(why)
            try:
                role = rolemod.get(name)
            except KeyError as e:
                raise ParamError(e.args[0]) from None
            if not role.stored_in_roles:
                raise ParamError(f"role {name!r} is the config's {role.field!r} field")
            if not rolemod.applies(name, lk.kind):
                raise ParamError(f"{lk.key} is a {lk.kind}: it has no {role.title} role "
                                 f"({name!r} is a {' or '.join(role.kinds)}'s)")
            if slot and (role.per == "design" or not slot.isidentifier()):
                raise ParamError(f"role {name!r} is one per design: no slot {slot!r}"
                                 if role.per == "design" else
                                 f"role {qualified!r}: the slot must be an identifier")
            if not isinstance(key, str) or not key:
                raise ParamError(f"role {qualified!r} wants a construction key, got {key!r}")
            if key == rolemod.NONE and not role.optional:
                raise ParamError(f"role {name!r} can't be left empty ({rolemod.NONE!r})")
            if (gone := removed_construction(name, key)) is not None:
                raise ParamError(gone[0])
            out[str(qualified)] = key
        plain = {q: rolemod.default_key(q, self) for q in out if "@" not in q}
        return tuple(sorted((q, k) for q, k in out.items() if plain.get(q) != k))

    def role(self, name: str, slot: str = "") -> str:
        """The construction key that fills role ``name`` (at ``slot``, for a per-drive or
        per-joint role: its own key, else the role's); ``""``: the role is empty."""
        from spiderpig import roles as rolemod

        role = rolemod.get(name)
        if role.field:
            return str(getattr(self, role.field))
        named = dict(self.roles)
        key = (named.get(f"{name}@{slot}") if slot else None) or named.get(name)
        key = key or rolemod.default_key(name, self)
        return "" if key == rolemod.NONE else key

    def applied_roles(self) -> dict[str, str]:
        """Every :mod:`spiderpig.roles` role kept in ``roles`` that applies to this design,
        with the key that fills it (:data:`roles.NONE` for an emptied one), and every
        per-slot key named: what the resolved spec writes in. A role filled by its kind's
        ``implicit`` key, its default here, is left out (:class:`roles.Role`: the designs
        stored before it reached their kind keep their ids, and read back the same)."""
        from spiderpig import roles as rolemod

        kind = self.lk.kind
        named = dict(self.roles)
        out = {}
        for r in rolemod.extra():
            if kind not in r.kinds:
                continue
            default = rolemod.default_key(r.name, self)
            key = named.get(r.name) or default
            implicit = r.implicit_for(kind)
            if not (implicit and key == implicit == default):
                out[r.name] = key
        out.update({q: k for q, k in named.items() if "@" in q})
        return out

    def _link_sheets(self, lk: linkage.Linkage) -> tuple[tuple[str, str], ...] | None:
        """Link class -> sheet, validated; ``None`` when it is the linkage's own."""
        if self.link_sheets is None:
            return None
        from spiderpig.hardware.catalog import get
        from spiderpig.materials import default_link_sheets

        out = {}
        for link, key in dict(self.link_sheets).items():
            if link not in lk.links:
                raise ParamError(f"link_sheets: {lk.key} has no link {link!r} "
                                 f"(have {list(lk.links)})")
            try:
                ok = get(key).category == "sheet"
            except KeyError:
                ok = False
            if not ok:
                raise ParamError(f"link_sheets: {key!r} is not a sheet in the catalog")
            out[link] = key
        if out == default_link_sheets(lk):
            return None
        return tuple(sorted(out.items()))

    def _phases(self, lk: linkage.Linkage) -> tuple[float, ...] | None:
        """Radians per leg; ``None`` for the module's own (mod 2 pi), however given."""
        if self.phases is None:
            return None
        legs = lk.leg_modules[self.module]
        try:
            values = tuple(float(p) for p in self.phases)
        except (TypeError, ValueError):
            raise ParamError(f"phases must be numbers, got {list(self.phases)!r}") from None
        if len(values) != len(legs):
            raise ParamError(f"{self.module} has {len(legs)} legs per side, got {len(values)} "
                             "phases")
        if not all(math.isfinite(p) for p in values):
            raise ParamError(f"phases must be finite numbers, got {list(values)}")
        if all(abs(_wrap(a - ph)) < 1e-9 for a, (_, ph) in zip(values, legs, strict=True)):
            return None
        return values

    def _proportions(self, lk: linkage.Linkage) -> tuple[tuple[str, float], ...]:
        """Overrides that differ from the defaults, validated, in the linkage's order."""
        overrides = dict(self.proportions)
        unknown = sorted(set(overrides) - set(lk.params))
        if unknown:
            raise ParamError(f"unknown {lk.key} proportions {unknown}; have {list(lk.params)}")
        out = []
        for name, default in lk.params.items():
            if name not in overrides:
                continue
            try:
                v = float(overrides[name])
            except (TypeError, ValueError):
                raise ParamError(f"proportion {name} must be a number, "
                                 f"got {overrides[name]!r}") from None
            if not math.isfinite(v):
                raise ParamError(f"proportion {name} must be a finite number, "
                                 f"got {overrides[name]!r}")
            if name not in lk.signed and v <= 0:       # angles and coordinates may be <= 0
                raise ParamError(f"proportion {name} is a length and must be > 0, got {v:g}")
            if abs(v - float(default)) > 1e-12:
                out.append((name, v))
        return tuple(out)

    # -- what the stages read ---------------------------------------------------

    @property
    def lk(self) -> linkage.Linkage:
        return linkage.get(self.linkage)

    @property
    def legs(self) -> linkage.LegList:
        """``(orientation, crank phase in rad)`` per leg of one side."""
        legs = self.lk.leg_modules[self.module]
        if self.phases is None:
            return legs
        return tuple((o, ph) for (o, _), ph in zip(legs, self.phases, strict=True))

    @property
    def pitch(self) -> float:
        """The layer pitch: the sheet's thickness."""
        return sheet_thickness(self.sheet, self.thickness)

    @property
    def values(self) -> tuple[float, ...]:
        """Every parameter of the linkage, overrides applied."""
        return self.lk.values(dict(self.proportions))

    def side_config(self) -> BuildConfig:
        """The config of the side alone: ``robot`` off, the robot-level roles
        (:attr:`roles.Role.robot`: the turntable, the tool) dropped, so every robot of one
        side shares its design and plan. A linkage that makes no robot (a mechanism: its
        side is the whole machine, :func:`robot_layout`) keeps them: there is no robot round
        its side to hold them (its controller's lines are the side's,
        :func:`construction.robot.controller_lines`)."""
        from spiderpig import roles as rolemod

        if not robot_layout(self.lk):
            return replace(self, robot=False)
        roles = tuple((k, v) for k, v in self.roles
                      if not rolemod.get(rolemod.split(k)[0]).robot)
        return replace(self, robot=False, roles=roles)

    @property
    def layout(self) -> str:
        """The robot's layout (:data:`ROBOT_LAYOUTS`, :data:`construction.robot.LAYOUTS`):
        ``""`` for one side."""
        return ROBOT_LAYOUTS[self.lk.kind] if self.robot else ""

    @property
    def mirrored(self) -> bool:
        """Two mirror-image sides in one frame (a walker's robot)."""
        return self.layout == "mirrored_pair"

    @property
    def sides(self) -> int:
        """How many planar sides it builds: 2 for a mirrored pair, else 1."""
        return 2 if self.mirrored else 1

    @property
    def is_default(self) -> bool:
        """The linkage's default design (the module's phases, its proportions), built the
        default way."""
        return self == BuildConfig(linkage=self.linkage, module=self.module, robot=self.robot)

    @property
    def key(self) -> str:
        """A file-name stem: ``strider_double_robot``, plus a hash for anything but a default."""
        stem = f"{self.linkage}_{self.module}_{'robot' if self.robot else 'side'}"
        if self.is_default:
            return stem
        # the fields that differ from BuildConfig()'s, so a field added later (at its
        # default) leaves every key as it was; the defaults are constants: one key per config
        base = _base_config()
        diff = tuple((f.name, getattr(self, f.name)) for f in fields(self)
                     if getattr(self, f.name) != getattr(base, f.name))
        return f"{stem}_{hashlib.sha1(repr(diff).encode()).hexdigest()[:12]}"

    def design_json(self) -> dict:
        """The design parameters as JSON: linkage, module, phases (deg) and all proportions."""
        return {
            "linkage": self.linkage,
            "module": self.module,
            "phases_deg": [round(math.degrees(ph), 6) for _, ph in self.legs],
            "proportions": dict(zip(self.lk.params, self.values, strict=True)),
        }


@cache
def _base_config() -> BuildConfig:
    """``BuildConfig()``, made once (:attr:`BuildConfig.key` compares against it)."""
    return BuildConfig()


# ---------------------------------------------------------------------------
# From the command line and the query string
# ---------------------------------------------------------------------------


def parse_phases(text: str) -> tuple[float, ...]:
    """``"0,180,90,270"`` -> radians (degrees in; the count is checked by the config)."""
    try:
        values = [float(s) for s in str(text).split(",")]
    except ValueError:
        raise ParamError(f"phases must be comma-separated numbers (degrees), got {text!r}") \
            from None
    if not all(math.isfinite(v) for v in values):
        raise ParamError(f"phases must be finite numbers, got {text!r}")
    return tuple(math.radians(v) for v in values)


def parse_proportion(item: str) -> tuple[str, float]:
    """``"DF=2.6"`` -> ``("DF", 2.6)`` (the config checks the name)."""
    name, sep, value = str(item).partition("=")
    name = name.strip()
    if not sep or not name:
        raise ParamError(f"expected NAME=VALUE, got {item!r}")
    try:
        return name, float(value)
    except ValueError:
        raise ParamError(f"proportion {name} must be a number, got {value!r}") from None


DEFAULT_MODULE = "quad"           # a walker's, when neither it nor the linkage says


def default_module(key: str) -> str:
    """The module a design gets when none is asked for: the linkage's own
    (``Linkage.default_module``: Strider's ``double``), else a walker's ``quad`` (its
    first module if it has no quad), a mechanism's one module (:class:`ParamError` for
    an unknown linkage)."""
    try:
        lk = linkage.get(key)
    except KeyError as e:
        raise ParamError(e.args[0]) from None
    if lk.default_module:
        return lk.default_module
    if DEFAULT_MODULE in lk.leg_modules:
        return DEFAULT_MODULE
    return next(iter(lk.leg_modules))


def default_robot(key: str) -> bool:
    """Whether a design builds the whole robot when nothing says: a walker (its two sides)
    and an arm (on its turntable) do, a mechanism is one side."""
    try:
        return linkage.get(key).kind in ROBOT_LAYOUTS
    except KeyError as e:
        raise ParamError(e.args[0]) from None


def _kept(field: str):
    """An argparse ``type`` for ``field`` (a config field or a role): a removed key's error
    names its replacement (argparse: exit 2)."""
    import argparse

    def check(text: str) -> str:
        if gone := removed_construction(field, text):
            raise argparse.ArgumentTypeError(gone[0])
        return text
    return check


def add_design_args(p) -> None:
    """``--linkage``, ``--module``, ``--phases`` and ``--proportion`` on an argparse parser.
    ``--module`` defaults to ``None``: :func:`config_from_args` fills in the linkage's
    (:func:`default_module`), so a mechanism needs no ``--module single``."""
    import argparse

    def arg(fn):
        def convert(text):
            try:
                return fn(text)
            except ParamError as e:
                raise argparse.ArgumentTypeError(str(e)) from None
        convert.__name__ = fn.__name__
        return convert

    p.add_argument("--linkage", choices=linkage.available(), default=linkage.DEFAULT,
                   help=f"the linkage (default {linkage.DEFAULT})")
    p.add_argument("--module", default=None,
                   help="legs per side (the linkage's modules): single, double (mirrored "
                   "pair), decker (two legs on one crankshaft), quad (two mirrored deckers). "
                   f"Default: the linkage's ({linkage.DEFAULT}'s "
                   f"{default_module(linkage.DEFAULT)}; a walker's {DEFAULT_MODULE} otherwise, "
                   "a mechanism's single)")
    p.add_argument("--phases", type=arg(parse_phases), default=None, metavar="DEG,...",
                   help="crank phase of every leg of a side, in degrees (default: the "
                   "module's, e.g. quad 0,180,90,270)")
    names = "; ".join(f"{k}: {', '.join(linkage.get(k).params)}" for k in linkage.available())
    p.add_argument("--proportion", type=arg(parse_proportion), action="append", default=None,
                   metavar="NAME=VALUE",
                   help=f"override one of the linkage's parameters (repeatable; lengths in mm "
                   f"or the linkage's unit, angles in degrees): {names}")


def add_build_args(p) -> None:
    """The build options: ``--servo``, the constructions, the sheet."""
    from spiderpig import construction  # (the registries: not at import, CAD-heavy)

    d = BuildConfig()
    axles, cranks = sorted(construction.AXLES), sorted(construction.CRANKS)

    p.add_argument("--servo", default=d.servo, choices=servos.available(),
                   help=f"servo model (default {d.servo})")
    pivots = "; ".join(f"a {k}'s {v[0]} / {v[1]}" for k, v in DEFAULT_PIVOTS.items())
    p.add_argument("--pillar", default=None, choices=axles, type=_kept("pillar"),
                   help=f"construction of the frame pivots (default: the kind's; {pivots})")
    p.add_argument("--pin", default=None, choices=axles, type=_kept("pin"),
                   help=f"construction of the pivots between links (default: the kind's; "
                        f"{pivots})")
    p.add_argument("--crank", default=None, choices=cranks, type=_kept("crank"),
                   help="crank construction (default: "
                        + ", ".join(f"{v} for a {k}" for k, v in DEFAULT_CRANKS.items() if v)
                        + "; " + ", ".join(f"{v} for {k}" for k, v in LINKAGE_CRANKS.items())
                        + ")")
    from spiderpig import roles as rolemod

    for role in rolemod.extra():          # every later role: --<role> KEY (default: its own)
        p.add_argument(f"--{role.name.replace('_', '-')}", dest=f"role_{role.name}",
                       default=None, choices=rolemod.choices(role.name),
                       type=_kept(role.name),
                       help=f"{role.title}: {role.help} (a {' or '.join(role.kinds)}'s; "
                            "default: the design's own)")
    p.add_argument("--role", dest="role_slots", action="append", default=None,
                   metavar="ROLE@SLOT=KEY",
                   help="one slot's construction for a per-drive or per-joint role, e.g. "
                        "transmission@q2=belt (repeatable)")
    p.add_argument("--sheet", default=d.sheet, help=f"sheet stock catalog item (default {d.sheet})")
    p.add_argument("--thickness", type=float, default=None,
                   help="measured sheet thickness in mm (default: the sheet's nominal)")
    p.add_argument("--frame-sheet", dest="frame_sheet", default=d.frame_sheet,
                   help=f"sheet of the frame and centre plates (default {d.frame_sheet})")
    p.add_argument("--crank-sheet", dest="crank_sheet", default=None,
                   help=f"sheet of the crank's plates (default {CRANK_SHEET}; "
                        + ", ".join(f"{v} for {k}" for k, v in LINKAGE_CRANK_SHEETS.items())
                        + ")")
    p.add_argument("--heads", default=d.heads, choices=("best", "sink", "gap"),
                   help="fasteners' heads: sunk into a layer, in thin clearance gaps, or "
                        "best: sunk, and in gaps only when no sunk plan exists (the plan's "
                        f"proof says which was searched; default {d.heads})")
    p.add_argument("--link-sheet", dest="link_sheet", action="append", default=None,
                   metavar="LINK=SHEET",
                   help="cut a link class from another sheet, e.g. b4=al6061_3p2mm "
                        "(repeatable; default: the linkage's, a Klann variant's foot links "
                        "in 6061 aluminium)")


def config_from_args(args, **fixed) -> BuildConfig:
    """The config the parsed arguments ask for (:class:`ParamError` if they don't fit).

    ``fixed`` overrides fields (``robot=False``); build options the parser lacks keep
    their defaults. A module not given (``None``) is the linkage's
    (:func:`default_module`); ``robot`` not given, or given as ``None``, is the
    linkage's kind's (:func:`default_robot`: a mechanism builds one side).
    """
    d = BuildConfig()
    fields = {k: getattr(args, k, getattr(d, k)) for k in ("linkage", "sheet", "thickness",
                                                            "servo", "pillar", "pin",
                                                            "frame_sheet", "crank_sheet",
                                                            "heads")}
    if getattr(args, "link_sheet", None):
        pairs = []
        for item in args.link_sheet:
            link, sep, key = str(item).partition("=")
            if not sep:
                raise ParamError(f"--link-sheet wants LINK=SHEET, got {item!r}")
            pairs.append((link.strip(), key.strip()))
        fields["link_sheets"] = tuple(pairs)
    from spiderpig import roles as rolemod

    named = {r.name: getattr(args, f"role_{r.name}", None) for r in rolemod.extra()}
    for item in getattr(args, "role_slots", None) or ():
        q, sep, key = str(item).partition("=")
        if not sep or "@" not in q:
            raise ParamError(f"--role wants ROLE@SLOT=KEY, got {item!r}")
        named[q.strip()] = key.strip()
    if any(named.values()):
        fields["roles"] = tuple((k, v) for k, v in named.items() if v)
    fields["crank"] = getattr(args, "crank", None) or ""        # the kind's default
    fields["pin"] = getattr(args, "pin", None) or ""            # the kind's default
    fields["pillar"] = getattr(args, "pillar", None) or ""
    fields["crank_sheet"] = getattr(args, "crank_sheet", None) or ""   # the linkage's
    fields.update(phases=getattr(args, "phases", None),
                  proportions=tuple(getattr(args, "proportion", None) or ()))
    fields.update(fixed)
    fields["module"] = (fixed.get("module") or getattr(args, "module", None)
                        or default_module(fields["linkage"]))
    if fields.get("robot") is None:
        fields["robot"] = default_robot(fields["linkage"])
    return BuildConfig(**fields)


def query_proportions(query: Mapping) -> list[tuple[str, float]]:
    """The ``p.<NAME>=<value>`` items of a query string, in order (a multi-dict's every
    item)."""
    multi = getattr(query, "multi_items", None)       # (starlette's QueryParams)
    items = multi() if multi is not None else query.items()
    return [parse_proportion(f"{k[2:]}={v}") for k, v in items if k.startswith("p.")]


def own_constructions(base: BuildConfig) -> dict:
    """``crank``, ``crank_sheet``, ``pin`` and ``pillar`` as ``base`` *chose* them: each one
    its linkage and module would get anyway (:func:`default_crank`,
    :func:`default_crank_sheet`, :func:`default_pin`, :func:`default_pillar`, filled in by
    the config) is ``""``, so a design moved to another linkage or module gets that
    one's own default instead of carrying the old one's (TrotBot heel's ``bolt_round``,
    the Hoecken pantograph's 0.080 in crank sheet); a product's (a decanting cradle's)
    sheets, pin and dimensions, where they are its defaults, the plain ones."""
    lk = linkage.get(base.linkage)
    out: dict = {"crank": "" if base.crank == default_crank(lk, base.module) else base.crank,
                 "crank_sheet": ("" if base.crank_sheet == default_crank_sheet(lk)
                                 else base.crank_sheet),
                 "pin": "" if base.pin == default_pin(lk) else base.pin,
                 "pillar": "" if base.pillar == default_pillar(lk) else base.pillar}
    if lk.product:      # a product's own sheets and dimensions: back to the plain ones
        plain = {"sheet": SHEET, "frame_sheet": FRAME_SHEET}
        for what, value in product_defaults(lk.product).items():
            if what in plain and getattr(base, what) == value:
                out[what] = plain[what]
        mine = PRODUCT_PARAMS.get(lk.product, {})
        out["params"] = replace(base.params, **{
            k: getattr(Params(), k) for k, v in mine.items() if getattr(base.params, k) == v})
    return out


def design_from_query(query: Mapping, base: BuildConfig | None = None,
                      **fixed) -> BuildConfig:
    """The config a query string asks for: ``linkage``, ``module`` (default: the linkage's,
    :func:`default_module`), ``phases`` (degrees, comma-separated) and ``p.<NAME>=<value>``;
    other keys are ignored. ``fixed`` (``robot``, a mode's ``module``) wins.

    With ``base`` (a stored design's config), the query applies on top of it: its servo,
    sheets, thickness, heads, constructions and fit, which no query string expresses,
    stay; ``module``, ``phases`` and ``p.NAME`` change. Another ``linkage`` starts from
    that linkage's defaults (its own parameters, phases and link sheets) and keeps the
    materials and constructions; a crank or crank sheet that was only the old linkage's
    (or module's) default is not kept (:func:`own_constructions`)."""
    proportions = query_proportions(query)
    if base is None:
        key = query.get("linkage") or linkage.DEFAULT
        kw = {"linkage": key,
              "module": query.get("module") or default_module(key),
              "phases": parse_phases(query["phases"]) if query.get("phases") else None,
              "proportions": tuple(proportions)}
        return BuildConfig(**{**kw, **fixed})
    own = own_constructions(base)
    if (query.get("linkage") or base.linkage) != base.linkage:
        # (link_sheets name a linkage's own links: kept only with its linkage)
        from spiderpig import roles as rolemod

        lk_new = linkage.get(query.get("linkage") or base.linkage)
        kind = lk_new.kind
        materials = {"sheet": base.sheet, "thickness": base.thickness,
                     "servo": (servos.DEFAULT if unused_field(lk_new, "servo", base.servo)
                               else base.servo),     # (an arm has no drive servo)
                     "frame_sheet": base.frame_sheet, "heads": base.heads,
                     "params": (Params() if base.params == default_params(base.lk)
                                else base.params),
                     "roles": tuple((n, k) for n, k in base.roles if rolemod.applies(n, kind)),
                     **own}
        if unused_field(lk_new, "crank_sheet", materials["crank_sheet"]):
            materials["crank_sheet"] = ""       # (an arm has no crank: its default)
        return design_from_query(query, **{**materials, **fixed})
    props = dict(base.proportions)
    props.update(proportions)
    module = fixed.get("module") or query.get("module") or base.module
    if query.get("phases"):
        phases = parse_phases(query["phases"])
    else:
        phases = base.phases if module == base.module else None     # the module's own
    rest = {k: v for k, v in fixed.items() if k != "module"}
    rest.setdefault("crank", base.crank if module == base.module else own["crank"])
    return replace(base, module=module, phases=phases,
                   proportions=tuple(sorted(props.items())), **rest)
