"""Planar linkages (walking legs and mechanisms) as symbolic straight-line programs.

A :class:`Linkage` is one leg, or one mechanism, written as a short program:
each step names a point and gives it as a small sympy expression over earlier
points' symbols (:func:`P`), the crank angle :data:`t` and the linkage's parameters (exact
rationals: lengths in mm or in a drawing unit, angles in degrees). The
geometry helpers below (:func:`crank`, :func:`circle_x_circle`,
:func:`extend`, :func:`offset`, ...) keep each step to one construction a
draughtsman would make with compass and ruler.

Nothing is substituted until :meth:`Linkage.compiled`, which chains the
steps, runs CSE and lambdifies once per process. Every leg of every assembly
evaluates that one compiled program:

* a leg's ``phase`` is a time shift, ``t + phase``;
* its mirror image (``orientation=-1``) is the reflection ``x -> -x`` of the
  leg at crank angle ``pi - (t + phase)``. The crank centre ``O`` is the
  origin and every crankpin turns with ``t``, so the mirrored crankpin at
  ``t`` is the original's: mirrored legs ride the same crank.

Conventions every definition follows: ``O`` is the crank centre at the
origin, ``y`` points up (a walker's feet are the lowest points), the crank
turns counter-clockwise with ``t``. Links are bodies named ``b<k>``
(:func:`is_link` in :mod:`stack`); ``torso`` carries the fixed pivots,
``conn`` the crank.

A walker declares ``feet``; a mechanism (a building block: a straight line, a
lift, a rocker) declares its :class:`Output` instead, which
:meth:`Linkage.output_check` measures and whose promises the template stage
enforces (:class:`OutputError`). A mechanism may take a second input ``t2``
(:func:`crank_at`); fabrication drives one.

Definitions live in :mod:`linkages` and register themselves; see
:func:`get` / :func:`available`.

The package: :mod:`linkage.engine` (the steps, :class:`Linkage`, the registry,
:class:`LegSolution`), :mod:`linkage.checks` (the stage checks: loop closure and
transmission angles, a mechanism's output against its promises) and
:mod:`linkage.assembly` (one leg as a :class:`mechanism.MechanismTemplate`, the
composition of legs into a side's leg module, the feet). Everything is
re-exported here.
"""

from __future__ import annotations

from spiderpig.linkage.assembly import (
    build_leg_template,
    build_module_template,
    combine_connectors,
    crank_name,
    feet_of,
    fuse_couplers,
    fuse_torsos,
    leg_bodies,
    leg_connections,
    legs_template,
    module_legs,
    module_of,
)
from spiderpig.linkage.checks import (
    ON_LINE_MM,
    STILL_DEG,
    TOGGLE_DEG,
    OutputCheck,
    StepCheck,
    check_output,
    check_steps,
)
from spiderpig.linkage.engine import (
    DEFAULT,
    MODULE_LEGS,
    MODULES,
    MOTIONS,
    REGISTRY,
    AssemblyError,
    LegList,
    LegSolution,
    Linkage,
    LinkSpec,
    Module,
    Output,
    OutputError,
    P,
    Steps,
    available,
    circle_x_circle,
    crank,
    crank_at,
    extend,
    get,
    offset,
    polar,
    register,
    rigid,
    rotate,
    scale_params,
    t,
    xy,
)

__all__ = [
    "DEFAULT", "MODULE_LEGS", "MODULES", "MOTIONS", "ON_LINE_MM", "REGISTRY", "STILL_DEG",
    "TOGGLE_DEG", "AssemblyError", "LegList", "LegSolution", "Linkage", "LinkSpec", "Module",
    "Output", "OutputCheck", "OutputError", "P", "StepCheck", "Steps", "available",
    "build_leg_template", "build_module_template", "check_output", "check_steps",
    "circle_x_circle", "combine_connectors", "crank", "crank_at", "crank_name", "extend",
    "feet_of", "fuse_couplers", "fuse_torsos", "get", "leg_bodies", "leg_connections",
    "legs_template", "module_legs", "module_of", "offset", "polar", "register", "rigid",
    "rotate", "scale_params", "t", "xy",
]
