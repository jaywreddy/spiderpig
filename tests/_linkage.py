"""Recorded inputs of the linkage / walk / server tiers (PLAN P1), and the seam they use.

**Foot z** (``tests/fixtures/linkage/foot_z.json``). The walking model needs each foot's
lateral z, which comes from the layer plan of the linkage's *default* design
(:func:`spiderpig.walk.foot_z_nominal` -> ``walk._default_plan_z`` -> ``design_side``):
planning is the planner's business and takes from 0.3 s (Klann single) to 60 s (the
Jansen quad's search, which ends at the CPU deadline without a plan). The fast tests read
the recorded z instead (:func:`recorded_foot_z_ctx` puts them in ``walk._default_plan_z``'s
place); a config that isn't recorded is planned live, as before. :func:`foot_z_doc` is the
generator, ``test_walk.py::test_foot_z_fixture_is_current`` the currency test (slow); both
search under :func:`node_budget` (no clock, :data:`FOOT_Z_NODES` search steps), so what
they record doesn't depend on the machine's load.

**The walk reference** (``tests/fixtures/linkage/walk_reference.json``): the demo Klann quad
on the default constructions (the reference was the ``--crank printed`` quad on the
materials before 2026-10-04 until that crank was removed, 2026-10-07), its feet and centre
of mass as ``/api/walk`` sends them, the Python model's straight-walk metrics, and
:data:`QUAD_REFERENCE`, the numbers both models must give (``test_walk.py::
test_quad_reference`` and ``viewer/src/drive/model.test.ts`` read the same file).
"""

from __future__ import annotations

import functools
import math
from collections.abc import Iterator
from contextlib import contextmanager

import pytest

from spiderpig import walk
from spiderpig.config import BuildConfig


def _klann(module: str, **kw) -> BuildConfig:
    return BuildConfig(linkage="klann", module=module, **kw)


FOOT_Z_CONFIGS: tuple[BuildConfig, ...] = (
    # test_walk: the default designs its walkers and /api/walk use
    *(_klann(m) for m in ("single", "double", "decker", "quad")),   # the quad: the walk
    #                                                                   reference's
    BuildConfig(linkage="jansen", module="double"),
    BuildConfig(linkage="jansen", module="quad"),                    # no plan: the guess
    BuildConfig(linkage="fourbar", module="quad"),                   # plans, and tips
    BuildConfig(linkage="strider", module="single"),
    BuildConfig(),                                                   # the Strider double
    # test_view: the stored XL330 Klann quad, and the tune panel's edits on top of it
    _klann("quad", servo="xl330_m288"),
    _klann("double", servo="xl330_m288"),
    BuildConfig(linkage="jansen", module="double", servo="xl330_m288"),
)
"""The default designs whose foot z the fast tests read (``walk._default_plan_z``'s keys:
the module's phases and the linkage's proportions, the robot, any servo and materials)."""


_REAL_DEFAULT_PLAN_Z = walk._default_plan_z


FOOT_Z_NODES = 4000
"""The search steps the foot z generator's searches may take in all (``StackSpec.
max_total_nodes``; the default is 60000 within a 60 CPU-s deadline). Every recorded
config that plans takes at most 1521 (the Klann double, 2026-10-07: the same z as with the
default budgets); the Jansen quad's search finds no plan at 4000 steps (6 CPU-s), nor at
60000 with the clock out (167 CPU-s), nor in the product's 60 s."""


@contextmanager
def node_budget(nodes: int = FOOT_Z_NODES) -> Iterator[None]:
    """Every side's search bounded by ``nodes`` search steps alone (the CPU deadline taken
    out): deterministic, whatever the load."""
    from spiderpig import fabricate, stack

    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(stack, "MAX_SECONDS", math.inf)
        mp.setattr(fabricate, "StackSpec",
                   functools.partial(stack.StackSpec, max_total_nodes=nodes))
        yield


def _live(config: BuildConfig):
    """What ``walk._default_plan_z`` answers from the planner (uncached)."""
    return _REAL_DEFAULT_PLAN_Z.__wrapped__(config)


def foot_z_doc() -> dict:
    """The generator of ``foot_z.json``: ``{config.key: {"config": repr, "z": [...] | None}}``
    (``None``: no layer plan, so the walking model guesses), each searched under
    :func:`node_budget`."""
    from spiderpig import fabricate

    out = {}
    memo = dict(fabricate._DESIGNS), dict(fabricate._LAYOUTS)
    try:
        with node_budget():
            for cfg in FOOT_Z_CONFIGS:
                fabricate._DESIGNS.clear()      # searched here, not answered from the memo
                fabricate._LAYOUTS.clear()      # or a seed (this process's other tests'
                z = _live(cfg)                  # are put back after)
                out[cfg.key] = {"config": repr(cfg), "z": None if z is None else list(z)}
    finally:
        for d, kept in zip((fabricate._DESIGNS, fabricate._LAYOUTS), memo, strict=True):
            d.clear()
            d.update(kept)
    return out


def _consume(name: str, make) -> dict:
    """A fixture's data for a test that reads it (:func:`tests.cache.recorded`); under
    ``--regen`` one that exists is read as it is: its currency test rewrites it, once."""
    from tests import cache

    if cache._regen() and cache.fixture_path("linkage", name).is_file():
        return cache.read_fixture("linkage", name)["data"]
    return cache.recorded("linkage", name, make)


def recorded_foot_z() -> dict[str, list[float] | None]:
    """``config.key -> z`` from the recorded fixture."""
    return {k: v["z"] for k, v in _consume("foot_z", foot_z_doc).items()}


@contextmanager
def recorded_foot_z_ctx() -> Iterator[dict]:
    """``walk._default_plan_z`` answering from the fixture (a config it doesn't hold: from
    the planner, as before); yields the recorded table."""
    table = recorded_foot_z()
    real = walk._default_plan_z

    def lookup(config: BuildConfig):
        if config.key in table:
            z = table[config.key]
            return None if z is None else tuple(z)
        return real(config)

    lookup.cache_clear = real.cache_clear           # what the server's watcher calls
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(walk, "_default_plan_z", lookup)
        yield table


def seed_default_plan(config: BuildConfig) -> None:
    """Seed ``config``'s plan, and the single module's its search would plan first for the
    leg hint (``fabricate._leg_hint``), from the test cache (:func:`tests.cache.seed_plan`):
    ``design_side`` then re-makes and verifies them instead of searching."""
    from tests import cache

    cache.seed_plan(config)
    hint = cache._hint_config(config)
    if hint is not None:
        cache.seed_plan(hint)


@functools.cache
def _seeded_live(config: BuildConfig):
    """The planner's answer, its plan seeded from the test cache first
    (:func:`seed_default_plan`)."""
    seed_default_plan(config)
    return _REAL_DEFAULT_PLAN_Z.__wrapped__(config)


def _clear_live() -> None:
    _seeded_live.cache_clear()
    _REAL_DEFAULT_PLAN_Z.cache_clear()


def use_live_foot_z(monkeypatch) -> None:
    """Undo :func:`recorded_foot_z_ctx` for one test: the planner answers again (each
    default design's plan seeded from the test cache, re-made and verified)."""
    live = functools.wraps(_REAL_DEFAULT_PLAN_Z)(lambda config: _seeded_live(config))
    live.cache_clear = _clear_live              # what the server's watcher calls
    monkeypatch.setattr(walk, "_default_plan_z", live)


# ---------------------------------------------------------------------------
# The walk reference
# ---------------------------------------------------------------------------

REFERENCE_CONFIG = _klann("quad")
"""The demo Klann quad, the default design (the bolt crank, standoff pillars, Chicago pins):
13 layers, the feet at z -95.8 / -49.2 / -56.0 mm. (The reference was the ``--crank
printed`` quad until 2026-10-07: 12 layers, the feet at -62 / -50 mm, a 50.0 mm least
margin; the same stride, bob and pitch within their tolerances.)"""

QUAD_REFERENCE = {
    "foot_z": [-95.8415, -95.8415, -49.1915, -55.966499999999996],
    "contacts_135": [True, False, False, True] * 2,     # legs 0 and 3, both sides, at 135 deg
    "pitch_deg_max_abs": [8.4, 0.1],                     # [value, abs tolerance]
    "bob_mm": [24.0, 0.5],
    "stride_mm": [102.0, 1.0],
    "min_margin_mm": [52.6, 0.5],
    "direction": "+x",
    "tipping_fraction": 0.0,
    "degenerate_fraction": 0.0,
    "roll_deg": [0.0, 0.0],
    "rpm_max": 52.0,
}
"""The viewer's numbers for :data:`REFERENCE_CONFIG` (the nominal centre of mass)."""


def walk_reference_doc() -> dict:
    """The generator of ``walk_reference.json``: the reference quad's feet and centre of
    mass (``/api/walk``'s fields the viewer's model reads, at the planned foot z), the
    Python model's metrics of them, and :data:`QUAD_REFERENCE`."""
    model = walk.walker(REFERENCE_CONFIG, feet_z=_live(REFERENCE_CONFIG))
    servo = walk.servo_info(REFERENCE_CONFIG.servo)
    metrics = walk.straight_walk_metrics(model, rpm_max=servo["rpm_max"])
    return {
        "config": repr(REFERENCE_CONFIG),
        "walk": walk.jsonable({
            "theta_samples": model.n,
            "feet": [f.as_json(digits=6) for f in model.feet],
            "com": [float(c) for c in model.com],
            "servo": {"key": servo["key"], "rpm_max": servo["rpm_max"]},
        }),
        "metrics": walk.jsonable(metrics),
        "reference": QUAD_REFERENCE,
    }


def walk_reference() -> dict:
    """The walk reference's data (``walk_reference.json``)."""
    return _consume("walk_reference", walk_reference_doc)


STRIDER_REFERENCE_CONFIG = BuildConfig()
"""The project's default design, the Strider double (its default constructions and
materials): the second design the viewer's model is held to the Python one on."""


def strider_walk_reference_doc() -> dict:
    """The generator of ``walk_reference_strider.json``: the default Strider double's feet
    and centre of mass as ``/api/walk`` sends them (at its planned foot z) and the Python
    model's straight-walk metrics of them (``viewer/src/drive/model.test.ts`` checks the
    viewer's model gives the same)."""
    cfg = STRIDER_REFERENCE_CONFIG
    model = walk.walker(cfg, feet_z=_live(cfg))
    servo = walk.servo_info(cfg.servo)
    metrics = walk.straight_walk_metrics(model, rpm_max=servo["rpm_max"])
    return {
        "config": repr(cfg),
        "walk": walk.jsonable({
            "theta_samples": model.n,
            "feet": [f.as_json(digits=6) for f in model.feet],
            "com": [float(c) for c in model.com],
            "servo": {"key": servo["key"], "rpm_max": servo["rpm_max"]},
        }),
        "metrics": walk.jsonable(metrics),
    }


def strider_walk_reference() -> dict:
    """The Strider double's walk reference (``walk_reference_strider.json``)."""
    return _consume("walk_reference_strider", strider_walk_reference_doc)
