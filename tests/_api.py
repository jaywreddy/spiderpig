"""Helpers for the API module's tests (``test_spiderpig_*``, ``test_review_fixes``,
``test_export``): designs built from the test cache instead of fabricated again.

``built(design, t)`` is :func:`spiderpig.api.build` on the cache's seam: the plan seeded
(:func:`tests.cache.cached_design`), the fabrication at ``t`` from
:func:`tests.cache.cached_side` / :func:`~tests.cache.cached_robot` (what
:func:`api.fabricate_at` builds: one side, or the robot with its ties and chassis) attached
with :func:`api.attach_build`, which is what ``build`` does after fabricating. The
attached mechanism is :func:`own`'s copy, so a test may edit and ``recheck`` its parts
(``recheck`` writes accepted solids into the mechanism's bodies) without touching the
process's shared entry. What it does not give: the build's captured construction warnings
(``BuildReport.warnings`` is empty; a test about them builds).

``HOECKEN`` is the tiny design (a mechanism: one side, 45 bodies, plan 0.3 s, a
fabrication 3 s, a verify ``standard`` ~10 s); the Klann single is the smallest walker.
"""

from __future__ import annotations

import copy

from spiderpig.config import BuildConfig

HOECKEN = {"kind": "mechanism", "linkage": {"key": "hoecken"}}
HOECKEN_CFG = BuildConfig(linkage="hoecken", module="single", robot=False)


def own(mech):
    """A copy of a (shared, cached) mechanism a test may change: its own bodies, meta,
    connections and BOM extras around the same solids (build123d shapes are never edited
    in place: an edit makes a new solid)."""
    out = copy.copy(mech)
    out.bodies = [copy.copy(b) for b in mech.bodies]
    out.connections = list(mech.connections)
    out.meta = dict(mech.meta)
    out.bom_extras = list(mech.bom_extras)
    return out


def fabricated(cfg: BuildConfig, t: float = 1.0):
    """``cfg`` fabricated at ``t`` from the cache: the robot, or one side (shared: never
    change it, :func:`own` a copy)."""
    from tests import cache

    return cache.cached_robot(cfg, t) if cfg.robot else cache.cached_side(cfg, t)


def seed(cfg: BuildConfig) -> None:
    """``cfg``'s plan from the cache (:func:`tests.cache.cached_design`): a later
    ``api.plan`` of a design with this config re-makes it instead of searching."""
    from tests import cache

    cache.cached_design(cfg)


_PROPS: dict[int, dict] = {}
"""id of a shared cached mechanism -> its parts' measured properties (``attach_build``'s
``props``): measured once per process, whichever test attaches it first."""


def built(design, t: float = 1.0):
    """:func:`spiderpig.api.build` of ``design`` at ``t`` from the cache (see the module's
    docstring): the build's report."""
    from spiderpig import api

    seed(design.config)
    shared = fabricated(design.config, t)
    props = _PROPS.setdefault(id(shared), {})
    return api.attach_build(design, own(shared), t, props=props)


def fabricate_from_cache(design, t: float):
    """A stand-in for :func:`spiderpig.api.fabricate_at` (monkeypatched by a test that
    counts or forces fabrications): the same parts, from the cache, the test's own copy."""
    return own(fabricated(design.config, t))
