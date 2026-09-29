"""The Klann linkage, and the historical entry points built on it.

The machinery lives in :mod:`linkage` (the generic straight-line-program
engine) and the definition in :mod:`linkages.klann`. This module keeps the
Klann-named API: :data:`PROPORTIONS`, :data:`STEPS`, :class:`KlannSolution`,
the template builders and the single-t mechanisms.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any

import sympy as sp

import linkage
from linkage import (  # noqa: F401 - re-exported
    MODULE_LEGS,
    P,
    _fuse,
    _merge,
    _suffixed_union,
    circle_x_circle,
    combine_connectors,
    extend,
    fuse_couplers,
    fuse_torsos,
    rotate,
)
from linkage import LegSolution as KlannSolution
from linkages.klann import KLANN
from mechanism import Mechanism, MechanismTemplate

_pt = P
t = linkage.t   # the crank angle

# Joseph Klann's published proportions. Lengths are multiples of ``OA`` (the
# crank-centre-to-A distance, in mm); angles are in degrees.
PROPORTIONS: dict[str, sp.Expr] = dict(KLANN.params)
PARAMS: dict[str, sp.Symbol] = KLANN.symbols
STEPS: list[tuple[str, sp.Matrix]] = KLANN.steps
POINTS: tuple[str, ...] = KLANN.points

# Link topology: joints along each bar. The first two fix the link's frame.
LINKS: dict[str, tuple[str, ...]] = {"conn": KLANN.crank,
                                     **{b: js for b, (js, _) in KLANN.links.items()}}


def closed_form() -> dict[str, sp.Matrix]:
    """Fully substituted point expressions (for inspection and differentiation)."""
    return KLANN.closed_form()


def compile_program() -> Callable[..., list]:
    """The lambdified program ``(t, *params) -> [Ox, Oy, Ax, ...]`` (compiled once)."""
    return KLANN.compiled


def proportion_values(overrides: Mapping[str, float] | None = None) -> tuple[float, ...]:
    """Proportion values in ``PROPORTIONS`` order, Klann's unless overridden."""
    return KLANN.values(overrides)


def create_klann_geometry(
    orientation: int = 1, phase: float = 0.0, proportions: Mapping[str, float] | None = None,
) -> KlannSolution:
    """The Klann leg of the given chirality, with crank angle ``t + phase``.

    ``proportions`` overrides some of :data:`PROPORTIONS` (same units). Cheap:
    the symbolic program is compiled once per process; this only records the
    parameters.
    """
    return KLANN.solve(orientation, phase, proportions)


def build_klann_template(solution: KlannSolution, *, name_suffix: str = "") -> MechanismTemplate:
    """One Klann leg whose joint poses are closures over the compiled program."""
    return linkage.build_leg_template(solution, name_suffix=name_suffix)


def _legs(name: str, legs: list[tuple[int, float]],
          proportions: Mapping[str, float] | None = None) -> MechanismTemplate:
    return linkage.legs_template(name, legs, proportions, "klann")


def build_module_template(
    module: str = "quad",
    phases: Sequence[float] | None = None,
    proportions: Mapping[str, float] | None = None,
    linkage_key: str = "klann",
) -> MechanismTemplate:
    """One side's legs on one crankshaft and one frame.

    See :func:`linkage.build_module_template`.
    """
    return linkage.build_module_template(module, phases, proportions, linkage_key)


def build_multi_leg_template(n_legs: int) -> MechanismTemplate:
    """``n_legs`` same-chirality legs at phases ``2πk/n``, kept as separate bodies.

    Fabrication puts them on one crankshaft (their crank centres coincide).
    """
    if n_legs < 1:
        raise ValueError(f"n_legs must be >= 1, got {n_legs}")
    return _legs("klann_multi", [(1, 2.0 * math.pi * k / n_legs) for k in range(n_legs)])


def build_double_template() -> MechanismTemplate:
    """Mirrored pair sharing one crank and one torso (``DoubleKlannLinkage``)."""
    return build_module_template("double")


def build_double_decker_template() -> MechanismTemplate:
    """Two same-chirality legs 90° apart on one crankshaft (``DoubleDeckerKlannLinkage``)."""
    return build_module_template("decker")


def build_double_double_decker_template() -> MechanismTemplate:
    """Four-leg walker: two mirrored pairs (phases 0/π and π/2 / 3π/2) on one crankshaft.

    2016 analogue: ``DoubleDoubleDeckerKlannLinkage`` (``Project/main.py:835``).
    """
    return build_module_template("quad")


# ---------------------------------------------------------------------------
# Single-t mechanisms: a template frozen at t, optionally fabricated
# ---------------------------------------------------------------------------


def _realize(
    tmpl: MechanismTemplate,
    t: float,
    *,
    thickness: float | None,
    with_parts: bool,
    with_joinery: bool = True,
) -> Mechanism:
    if not with_parts:
        return tmpl.freeze_at(t)
    from fabricate import BuildConfig, fabricate  # lazy: pulls in build123d

    return fabricate(tmpl, BuildConfig(robot=False, thickness=thickness), t)


def build_klann_mechanism(
    solution: KlannSolution,
    t: float,
    *,
    thickness: float | None = None,
    name_suffix: str = "",
    with_parts: bool = True,
    with_joinery: bool = True,
) -> Mechanism:
    """One Klann leg at crank angle ``t``; with parts, fabricated as a standalone walker."""
    tmpl = build_klann_template(solution, name_suffix=name_suffix)
    return _realize(tmpl, t, thickness=thickness, with_parts=with_parts, with_joinery=with_joinery)


def build_multi_leg_mechanism(n_legs: int, t: float, **kw) -> Mechanism:
    return _realize(build_multi_leg_template(n_legs), t, **_kw(kw))


def build_double_klann(t: float, **kw) -> Mechanism:
    return _realize(build_double_template(), t, **_kw(kw))


def build_double_decker_klann(t: float, **kw) -> Mechanism:
    return _realize(build_double_decker_template(), t, **_kw(kw))


def build_double_double_decker_klann(t: float, **kw) -> Mechanism:
    return _realize(build_double_double_decker_template(), t, **_kw(kw))


def _kw(kw: dict[str, Any]) -> dict[str, Any]:
    return {
        "thickness": kw.get("thickness"),
        "with_parts": kw.get("with_parts", True),
        "with_joinery": kw.get("with_joinery", True),
    }


def KlannLinkage(name: str = "klann") -> Mechanism:
    """A single fabricated Klann leg at the reference time ``t=1.0``."""
    mech = build_klann_mechanism(create_klann_geometry(), t=1.0)
    mech.name = name
    return mech
