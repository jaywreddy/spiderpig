"""The recorded inputs of the strength and hardware tiers (``tests/fixtures/strength/``,
``tests/fixtures/hardware/``) for the user's three order designs, and how each is made.

* ``strength/meta_<design>.json``: the robot's ``mech.meta`` (the wobble notes, the crank's,
  the Chicago pins', the deck's and the chassis' notes), from the fabrication cache;
* ``strength/pin_loads_<design>.json``: the design's simulated pin loads
  (:func:`spiderpig.sim.loads.design_loads`, MuJoCo: the document the store caches under
  ``pin_loads/``, less its ``seconds`` and ``source_version``);
* ``hardware/bom_<design>.json``: the robot's BOM (``bom_from_mechanism(...).as_dict()``).

The fast tests take them as input (:func:`tests.cache.recorded`); each has a slow
``fixture_regen`` currency test (``test_strength.py``, ``test_bom.py``) that makes it again
from the engine. See ``docs/agentlib/TESTING.md``.
"""

from __future__ import annotations

from spiderpig.config import BuildConfig
from tests import cache

ORDER_DESIGNS: dict[str, BuildConfig] = {
    "strider_double": BuildConfig(),                      # the project's default design
    "strider_quad": BuildConfig(module="quad"),
    "klann_lego_quad": BuildConfig(linkage="klann_lego", module="quad"),
}
"""The user's three order designs (the identity gate's first three), robots."""

LOADS_VOLATILE = ("seconds", "source_version")
"""What a pin-loads document says about its run rather than the loads: left out."""


def make_meta(name: str) -> dict:
    """The order design's robot's ``meta`` (as JSON reads it back)."""
    return cache._jsonable(cache.cached_robot(ORDER_DESIGNS[name]).meta)


def make_pin_loads(name: str, store=None) -> dict:
    """The order design's simulated pin loads (MuJoCo, minutes), cached in ``store``."""
    from spiderpig.sim import loads as sim_loads

    doc = sim_loads.design_loads(ORDER_DESIGNS[name], store)
    return {k: v for k, v in doc.items() if k not in LOADS_VOLATILE}


def make_bom(name: str) -> dict:
    """The order design's robot's BOM, grouped (``bom_from_mechanism(...).as_dict()``)."""
    from spiderpig.hardware.bom import bom_from_mechanism

    return bom_from_mechanism(cache.cached_robot(ORDER_DESIGNS[name])).as_dict()


def _recorded(module: str, name: str, make):
    """:func:`tests.cache.recorded`, except that under ``--regen`` a fixture already there
    is read, not made again: only its own currency test rewrites it (a pin-loads document
    is minutes of MuJoCo, and the meta's currency test reads the loads)."""
    if cache._regen() and (doc := cache.read_fixture(module, name)) is not None:
        return doc["data"]
    return cache.recorded(module, name, make)


def meta(name: str) -> dict:
    return _recorded("strength", f"meta_{name}", lambda: make_meta(name))


def pin_loads(name: str) -> dict:
    return _recorded("strength", f"pin_loads_{name}", lambda: make_pin_loads(name))


def bom(name: str) -> dict:
    return _recorded("hardware", f"bom_{name}", lambda: make_bom(name))


def _differences(a, b, rel: float, path: str = "") -> list[str]:
    """Where ``b`` differs from ``a``: numbers beyond ``rel`` (relative, and as much
    absolute for numbers near zero), anything else unequal."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = [f"  {path}/{k}: {'added' if k in b else 'removed'}" for k in sorted(set(a) ^ set(b))]
        for k in sorted(set(a) & set(b)):
            out += _differences(a[k], b[k], rel, f"{path}/{k}")
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [f"  {path}: {len(a)} items -> {len(b)}"]
        return [d for i, (x, y) in enumerate(zip(a, b, strict=True))
                for d in _differences(x, y, rel, f"{path}[{i}]")]
    numbers = (isinstance(a, (int, float)) and isinstance(b, (int, float))
               and not isinstance(a, bool) and not isinstance(b, bool))
    if numbers and abs(a - b) <= rel * max(abs(a), abs(b), 1.0):
        return []
    return [] if a == b else [f"  {path}: {a!r:.100} -> {b!r:.100}"]


def assert_current(module: str, name: str, data, rel: float = 1e-9):
    """:func:`tests.cache.assert_current` with numbers compared within ``rel`` (a sim or a
    fabrication on another machine may move the last digits): ``data``, made now from the
    engine, must match the recorded fixture; with ``--regen`` (or none recorded yet) it is
    written instead."""
    fresh = cache._jsonable(data)
    doc = cache.read_fixture(module, name)
    if cache._regen() or doc is None:
        return cache.assert_current(module, name, lambda: fresh)
    diff = _differences(doc["data"], fresh, rel)
    assert not diff, (f"tests/fixtures/{module}/{name}.json no longer matches the engine:\n"
                      + "\n".join(diff[:40]) + "\nIf the change is intended: `mise run "
                      "test-fixtures` (pytest --regen -m fixture_regen) rewrites it.")
    return fresh


def strength_loads(name: str, monkeypatch, doc: dict | None = None) -> dict:
    """:func:`spiderpig.strength.design_loads` of the order design on its recorded pin
    loads (``doc``, else the fixture's), as the audit has them: the sim's document is the
    recorded one (``sim.loads.design_loads`` replaced for the call)."""
    from spiderpig import strength
    from spiderpig.sim import loads as sim_loads

    doc = pin_loads(name) if doc is None else doc
    with monkeypatch.context() as mp:
        mp.setattr(sim_loads, "design_loads", lambda *a, **k: dict(doc))
        return strength.design_loads(ORDER_DESIGNS[name])
