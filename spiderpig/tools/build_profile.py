"""``spiderpig build --profile [--profile-json FILE] [build options]``: the build's stage
timings, logged on the ``spiderpig.build`` logger (``spiderpig/tools/profiler.py``).

:mod:`spiderpig.cli` sends ``build`` here when ``--profile`` or ``--profile-json`` is given;
everything else goes to :func:`spiderpig.build.main` unchanged. The stages are timed by
wrapping, for the length of the run, the functions the build calls (its module's names,
``api.plan_config``, ``Mechanism.export_step`` / ``export_stl``, ``Bom.write``,
``order_markdown``), so :mod:`spiderpig.build` itself, which is in the engine's hash
(:func:`spiderpig.design.engine_version`), carries no profiling code: measuring a build never
re-keys a store, the test cache or CI's cache. A call made inside another stage counts in the
outer one (no time counted twice).

Stages (:data:`STAGES`): ``import`` the process's age once the build's modules are imported
(the interpreter and every import), ``template`` the options' checks and the template,
``plan`` the layer plan through the store, ``fabricate`` the robot, ``step`` / ``stl`` its two
files, ``group`` the laser and printed parts grouped (mass properties), ``prints`` the print
STLs, ``dxf_sheets`` / ``dxf_parts`` the packed sheets and the per-part DXFs, ``bom`` the BOM
and its files, ``order`` ``ORDER.md`` and ``manifest.json``. ``build_total`` is the wall time
since the process started; ``unaccounted_pct`` what no stage holds (prints, the plan's
description). The interpreter's exit after the summary (~1 s) is outside it.
"""

from __future__ import annotations

import argparse
import functools
import json
import logging
import time
from contextlib import ExitStack, contextmanager
from pathlib import Path

from spiderpig.tools.profiler import Profiler, process_age

STAGES = ("import", "template", "plan", "fabricate", "step", "stl", "group", "prints",
          "dxf_sheets", "dxf_parts", "bom", "order")
logger = logging.getLogger("spiderpig.build")


def _targets():
    """(object, attribute, stage) of every call the build's stages are made of."""
    from spiderpig import api
    from spiderpig import build as build_mod
    from spiderpig.hardware import bom as bom_mod
    from spiderpig.hardware import order as order_mod
    from spiderpig.mechanism import Mechanism

    return [
        (build_mod, "clear_generated", "template"),
        (api, "config_warnings", "template"),
        (build_mod, "template_for", "template"),
        (api, "plan_config", "plan"),
        (build_mod, "design_side", "plan"),
        (build_mod, "fabricate", "fabricate"),
        (Mechanism, "export_step", "step"),
        (Mechanism, "export_stl", "stl"),
        (build_mod, "group_made", "group"),
        (build_mod, "export_prints", "prints"),
        (build_mod, "printed_filaments", "prints"),
        (build_mod, "save_sheets", "dxf_sheets"),
        (build_mod, "sheet_lines", "dxf_sheets"),
        (build_mod, "save_parts", "dxf_parts"),
        (build_mod, "bom_from_mechanism", "bom"),
        (bom_mod.Bom, "write", "bom"),
        (order_mod, "order_markdown", "order"),
        (build_mod, "_write_manifest", "order"),
    ]


@contextmanager
def instrumented(prof: Profiler):
    """The build's calls timed into ``prof`` while inside; the originals put back after."""
    active: list[str] = []

    def wrap(fn, stage):
        @functools.wraps(fn)
        def timed(*a, **kw):
            if active:                      # inside another stage: it counts there
                return fn(*a, **kw)
            active.append(stage)
            try:
                with prof.timed(stage):
                    return fn(*a, **kw)
            finally:
                active.pop()
        return timed

    with ExitStack() as undo:
        for obj, name, stage in _targets():
            original = obj.__dict__[name] if isinstance(obj, type) else getattr(obj, name)
            setattr(obj, name, wrap(original, stage))
            undo.callback(setattr, obj, name, original)
        yield


def main(argv=None) -> int:
    import spiderpig.build as build_mod  # the imports, counted in `import`

    age = process_age()
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--profile", action="store_true")
    p.add_argument("--profile-json", type=Path, default=None)
    ours, rest = p.parse_known_args(argv)
    if not logging.getLogger().handlers and not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
        logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    prof = Profiler(name="build", total="build_total", logger_name=logger.name)
    started = time.perf_counter()
    if age is not None:
        prof.add("import", age)
    try:
        with instrumented(prof):
            return build_mod.main(rest)
    finally:
        wall = time.perf_counter() - started + (age or 0.0)
        summed = sum(v for k, v in prof.as_dict()["stages"].items() if k in STAGES)
        prof.add("build_total", wall)
        prof.set_metric("wall_s", wall)
        prof.set_metric("stages_sum_s", summed)
        prof.set_metric("unaccounted_pct", 100.0 * (wall - summed) / wall if wall else 0.0)
        prof.log_summary()
        if ours.profile_json is not None:
            ours.profile_json.parent.mkdir(parents=True, exist_ok=True)
            ours.profile_json.write_text(json.dumps(
                {**prof.as_dict(), "wall_s": wall, "stages_sum_s": summed,
                 "import_measured": age is not None}, indent=1))


if __name__ == "__main__":
    raise SystemExit(main())
