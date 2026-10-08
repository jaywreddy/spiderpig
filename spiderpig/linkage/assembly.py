"""One leg as a kinematic template, the composition of legs into a side (a leg module:
one crankshaft, one frame) and the feet of a template or a fabricated robot.
"""

from __future__ import annotations

import itertools
from collections.abc import Mapping, Sequence

from spiderpig.linkage.engine import DEFAULT, LegList, LegSolution, Linkage, Module, get
from spiderpig.mechanism import (
    BodyTemplate,
    JointTemplate,
    MechanismTemplate,
    body_class,
    translation_pose_at,
)

# ---------------------------------------------------------------------------
# One leg as a kinematic template
# ---------------------------------------------------------------------------
#
# Every joint sits at z = 0 in its body's frame: the mechanism is planar, and
# which Z slot each part occupies is a fabrication decision made by
# :mod:`stack` (see :mod:`fabricate`). Joint XY comes straight from the
# compiled symbolic program.


def leg_bodies(lk: Linkage) -> list[tuple[str, tuple[str, ...], tuple[tuple[str, str], ...], str]]:
    """``(body, joints, outline, colour)`` in template order: coupler, links, crank, torso."""
    crank_outline = tuple(("O", pin) for pin in lk.crank[1:])
    return [
        ("coupler", ("O",), (), "blue"),
        *((b, js, outline, "green") for b, (js, outline) in lk.links.items()),
        ("conn", lk.crank, crank_outline, "orange"),
        ("torso", lk.frame, (), "yellow"),
    ]


def leg_connections(lk: Linkage) -> list[tuple[str, str, str]]:
    """``(body_a, body_b, joint)``: bodies sharing a joint name are pinned there.

    Each joint's bodies are chained in template order; the shaft coupler hangs
    off the crank at ``O``.
    """
    order = [(b, js) for b, js, _, _ in leg_bodies(lk) if b != "coupler"]
    out: list[tuple[str, str, str]] = []
    for j in dict.fromkeys(j for _, js in order for j in js):
        on = [b for b, js in order if j in js]
        out += [(a, b, j) for a, b in itertools.pairwise(on)]
    out.append(("conn", "coupler", "O"))
    return out


def build_leg_template(solution: LegSolution, *, name_suffix: str = "") -> MechanismTemplate:
    """One leg whose joint poses are closures over the compiled program."""
    lk = solution.linkage
    callables = solution.callables
    bodies = [
        BodyTemplate(
            name=f"{name}{name_suffix}",
            joints=[JointTemplate(name=j, pose_at=translation_pose_at(callables[j], z=0.0))
                    for j in joints],
            outline=outline,
            color=color,
        )
        for name, joints, outline, color in leg_bodies(lk)
    ]
    connections = [
        ((0, f"{a}{name_suffix}", j), (0, f"{b}{name_suffix}", j))
        for a, b, j in leg_connections(lk)
    ]
    return MechanismTemplate(name=f"{lk.key}{name_suffix}", bodies=bodies,
                             connections=connections)


# ---------------------------------------------------------------------------
# Composition primitives for multi-leg assemblies
# ---------------------------------------------------------------------------
#
# The 2016 project stacked 2–4 Klann legs into walker sculptures by sharing
# rigid bodies across legs (one crank driving a mirrored pair, one torso
# carrying every leg's pivots, one shaft coupler). Each pattern is a small,
# pure template rewrite: drop some bodies and redirect the edges that
# targeted them.


def _merge(name: str, tmpls: list[MechanismTemplate]) -> MechanismTemplate:
    return MechanismTemplate(
        name=name,
        bodies=[b for t_ in tmpls for b in t_.bodies],
        connections=[c for t_ in tmpls for c in t_.connections],
    )


def _rewrite_connections(connections, rewriter):
    """Map each endpoint via ``rewriter(idx, body, joint)``."""
    return [(rewriter(*a), rewriter(*b)) for a, b in connections]


def _fuse(
    tmpl: MechanismTemplate,
    olds: list[str],
    fused: BodyTemplate,
    rename: dict[tuple[str, str], str],
) -> MechanismTemplate:
    """Replace bodies ``olds`` by ``fused`` (at the first one's position).

    ``rename`` maps ``(old_body, old_joint)`` to the fused body's joint name.
    """
    drop = set(olds)
    bodies: list[BodyTemplate] = []
    for b in tmpl.bodies:
        if b.name == olds[0]:
            bodies.append(fused)
        elif b.name not in drop:
            bodies.append(b)

    def rewrite(idx, body, joint):
        if body in drop:
            return (idx, fused.name, rename[(body, joint)])
        return (idx, body, joint)

    return MechanismTemplate(
        name=tmpl.name, bodies=bodies,
        connections=_rewrite_connections(tmpl.connections, rewrite),
    )


def _suffixed_union(tmpl, olds, suffixes, keep_shared=()) -> tuple[list, dict]:
    """Joints of ``olds`` renamed ``{joint}{suffix}``; ``keep_shared`` joints appear once."""
    joints: list[JointTemplate] = []
    rename: dict[tuple[str, str], str] = {}
    for old, suffix in zip(olds, suffixes, strict=True):
        for j in tmpl.body(old).joints:
            name = j.name if j.name in keep_shared else f"{j.name}{suffix}"
            rename[(old, j.name)] = name
            if all(x.name != name for x in joints):
                joints.append(JointTemplate(name=name, pose_at=j.pose_at))
    return joints, rename


def combine_connectors(
    tmpl: MechanismTemplate, *suffixes: str, new_name: str = "conn",
) -> MechanismTemplate:
    """Fuse these legs' crank links into one rigid crank (``Project/main.py:494``).

    The cranks share the pivot O, so one rigid body driven by one shaft
    co-rotates the legs.
    """
    olds = [f"conn{s}" for s in suffixes]
    joints, rename = _suffixed_union(tmpl, olds, list(suffixes))
    outline = tuple(
        (f"{p}{sfx}", f"{q}{sfx}")
        for old, sfx in zip(olds, suffixes, strict=True)
        for p, q in tmpl.body(old).outline
    )
    fused = BodyTemplate(
        name=new_name, joints=joints, outline=outline, color=tmpl.body(olds[0]).color,
    )
    return _fuse(tmpl, olds, fused, rename)


def fuse_couplers(
    tmpl: MechanismTemplate, suffixes: list[str], *, new_name: str = "coupler",
) -> MechanismTemplate:
    """Collapse per-leg shaft couplers into one body with an ``O{suffix}`` joint per leg."""
    olds = [f"coupler{s}" for s in suffixes]
    joints, rename = _suffixed_union(tmpl, olds, suffixes)
    primary = tmpl.body(olds[0])
    fused = BodyTemplate(
        name=new_name, joints=joints, color=primary.color, base_pose=primary.base_pose,
    )
    return _fuse(tmpl, olds, fused, rename)


def fuse_torsos(
    tmpl: MechanismTemplate, suffixes: list[str], *, new_name: str = "torso",
) -> MechanismTemplate:
    """One frame carrying every listed leg's fixed pivots and the shared crank centre O.

    2016 analogue: the single hulled ``torso`` of ``DoubleKlannLinkage``
    (``Project/main.py:587``).
    """
    olds = [f"torso{s}" for s in suffixes]
    joints, rename = _suffixed_union(tmpl, olds, suffixes, keep_shared=("O",))
    fused = BodyTemplate(name=new_name, joints=joints, color=tmpl.body(olds[0]).color)
    return _fuse(tmpl, olds, fused, rename)


# ---------------------------------------------------------------------------
# Assemblies
# ---------------------------------------------------------------------------


def legs_template(name: str, legs: Sequence[tuple[int, float]],
                  params: Mapping[str, float] | None = None,
                  linkage: str = DEFAULT) -> MechanismTemplate:
    """Legs kept as separate bodies, suffixed ``_leg<k>``."""
    lk = get(linkage)
    return _merge(name, [
        build_leg_template(lk.solve(o, ph, params), name_suffix=f"_leg{k}")
        for k, (o, ph) in enumerate(legs)
    ])


def module_of(module: str, linkage: str = DEFAULT) -> Module:
    mods = get(linkage).modules_of
    if module not in mods:
        raise ValueError(f"unknown module {module!r}; have {sorted(mods)}")
    return mods[module]


def module_legs(module: str, linkage: str = DEFAULT) -> LegList:
    return module_of(module, linkage).legs


def crank_name(k: int) -> str:
    """The k-th shared crank body of a side: ``conn``, ``conn_upper``, ``conn_upper2``..."""
    return "conn" if k == 0 else f"conn_upper{k if k > 1 else ''}"


def build_module_template(
    module: str = "quad",
    phases: Sequence[float] | None = None,
    proportions: Mapping[str, float] | None = None,
    linkage: str = DEFAULT,
) -> MechanismTemplate:
    """One side's legs on one crankshaft and one frame.

    ``phases`` (radians, one per leg) replaces the module's default crank
    phases; ``proportions`` overrides some of the linkage's parameters. The
    legs the module says share a crank (:attr:`Module.cranks`) become one
    crank body each (:func:`crank_name`). The template's ``meta`` records
    all of it, so caches keyed on it stay honest.
    """
    lk = get(linkage)
    lk.assert_assembles(proportions)
    lk.assert_output(proportions)
    mod = module_of(module, linkage)
    legs = mod.legs
    if phases is not None:
        if len(phases) != len(legs):
            raise ValueError(f"{module} has {len(legs)} legs, got {len(phases)} phases")
        legs = tuple((o, float(ph)) for (o, _), ph in zip(legs, phases, strict=True))
    if len(legs) == 1:
        (o, ph), = legs
        tmpl = build_leg_template(lk.solve(o, ph, proportions))
    else:
        tmpl = legs_template(f"{lk.key}_{module}", legs, proportions, linkage)
        suffixes = [f"_leg{k}" for k in range(len(legs))]
        for k, group in enumerate(mod.cranks):
            tmpl = combine_connectors(tmpl, *(suffixes[i] for i in group), new_name=crank_name(k))
        tmpl = fuse_couplers(tmpl, suffixes)
        tmpl = fuse_torsos(tmpl, suffixes)
    tmpl.meta = {
        "linkage": lk.key,
        "module": module,
        "phases": tuple(ph for _, ph in legs),
        "proportions": lk.values(proportions),
    }
    return tmpl


def feet_of(tmpl_or_mech) -> list[tuple[str, str]]:
    """``(body, joint)`` of every foot in a side template or a fabricated robot.

    Reads the linkage from ``meta`` and matches foot links by class (so
    ``L.b4_leg2`` is a Klann foot).
    """
    lk = get(tmpl_or_mech.meta.get("linkage", DEFAULT))
    return [(b.name, j) for b in tmpl_or_mech.bodies for cls, j in lk.feet
            if body_class(b.name) == cls]
