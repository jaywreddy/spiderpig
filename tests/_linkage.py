"""Recorded inputs of the linkage / walk / server tiers (PLAN P1), and the seam they use.

**Foot z** (``tests/fixtures/linkage/foot_z.json``). The walking model needs each foot's
lateral z, which comes from the layer plan of the linkage's *default* design
(:func:`spiderpig.walk.foot_z_nominal` -> ``walk._default_plan_z`` -> ``design_side``):
planning is the planner's business and takes from 0.3 s (Klann single) to 122 s (the
Jansen quad's search, which ends without a plan). The fast tests read the recorded z
instead (:func:`use_recorded_foot_z` puts them in ``walk._default_plan_z``'s place); a
config that isn't recorded is planned live, as before. :func:`foot_z_doc` is the
generator, ``test_walk.py::test_foot_z_fixture_is_current`` the currency test (slow).

**The walk reference** (``tests/fixtures/linkage/walk_reference.json``): the Klann quad the
viewer's reference numbers were taken on (``--crank printed``, the materials before
2026-10-04), its feet and centre of mass as ``/api/walk`` sends them, the Python model's
straight-walk metrics, and :data:`QUAD_REFERENCE`, the numbers both models must give
(``test_walk.py::test_quad_reference`` and ``viewer/src/drive/model.test.ts`` read the same
file).
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

import pytest

from spiderpig import walk
from spiderpig.config import BuildConfig

OLD = {"frame_sheet": "acrylic_3mm", "link_sheets": (), "heads": "sink"}
"""The materials and full-layer heads the walk reference numbers were taken with."""


def _klann(module: str, **kw) -> BuildConfig:
    return BuildConfig(linkage="klann", module=module, **kw)


FOOT_Z_CONFIGS: tuple[BuildConfig, ...] = (
    # test_walk: the default designs its walkers and /api/walk use
    *(_klann(m) for m in ("single", "double", "decker", "quad")),
    _klann("quad", crank="printed", pillar="printed", **OLD),       # the walk reference
    _klann("quad", crank="keyed", pillar="printed", **OLD),
    BuildConfig(linkage="jansen", module="double"),
    BuildConfig(linkage="jansen", module="quad"),                    # no plan: the guess
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


def _live(config: BuildConfig):
    """What ``walk._default_plan_z`` answers from the planner (uncached)."""
    return _REAL_DEFAULT_PLAN_Z.__wrapped__(config)


def foot_z_doc() -> dict:
    """The generator of ``foot_z.json``: ``{config.key: {"config": repr, "z": [...] | None}}``
    (``None``: no layer plan, so the walking model guesses)."""
    out = {}
    for cfg in FOOT_Z_CONFIGS:
        z = _live(cfg)
        out[cfg.key] = {"config": repr(cfg), "z": None if z is None else list(z)}
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


def use_live_foot_z(monkeypatch) -> None:
    """Undo :func:`recorded_foot_z_ctx` for one test (the planner answers again)."""
    monkeypatch.setattr(walk, "_default_plan_z", _REAL_DEFAULT_PLAN_Z)


# ---------------------------------------------------------------------------
# The walk reference
# ---------------------------------------------------------------------------

REFERENCE_CONFIG = _klann("quad", crank="printed", pillar="printed", **OLD)
"""The Klann quad on ``--crank printed``: 12 layers, the feet at z -62 / -50 mm."""

QUAD_REFERENCE = {
    "foot_z": [-62.0, -62.0, -50.0, -50.0],
    "contacts_135": [True, False, False, True] * 2,     # legs 0 and 3, both sides, at 135 deg
    "pitch_deg_max_abs": [8.4, 0.1],                     # [value, abs tolerance]
    "bob_mm": [24.0, 0.5],
    "stride_mm": [102.0, 1.0],
    "min_margin_mm": [50.0, 0.5],
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
