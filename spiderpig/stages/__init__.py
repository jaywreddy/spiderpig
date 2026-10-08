"""The design's first stages, under the bake, the build and the API: a spec resolved into
a design (:mod:`.resolve`: :func:`~.resolve.resolve`, :func:`~.resolve.spec_of`,
:func:`~.resolve.config_warnings`), the stage records on the handle and in the store
(:mod:`.records`), their reports (:mod:`.reports`), and the check and the plan
(:mod:`.planning`: :func:`~.planning.check`, :func:`~.planning.plan`,
:func:`~.planning.plan_config`).

``spiderpig bake`` / ``spiderpig build`` and the build's up-to-date check plan through the
store as every command does, so these sit below them (``pyproject.toml``'s import
contracts); :mod:`spiderpig.api` re-exports every name, its public surface unchanged. A
patch goes on the module here that reads the name (``stages.planning.design_side``).
"""
