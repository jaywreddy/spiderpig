"""Print what each stage of the pipeline says about a design.

    uv run python explain.py --linkage strider --module double
    uv run python explain.py --linkage trotbot_heel --proportion unit=7

1. program: :meth:`linkage.Linkage.check`. Each loop's closing margin and
   transmission angle; a loop that can't close raises
   :class:`linkage.AssemblyError` when the template is built. A mechanism's
   :meth:`linkage.Linkage.output_check`; a broken promise raises
   :class:`linkage.OutputError` there too.
2. static facts: :func:`fabricate.side_clearances`, each link that sweeps
   through some group's keep-out, so the two can never share a layer; the
   crank's (:func:`construction.route.crank_facts`): which links need O
   free and which crank points they clear, the body's underside and the
   ground clearance. A link no crank point clears stops here
   (:class:`stack.ClearanceError`, with what would clear it).
3. plan: :func:`fabricate.design_side`. The layer plan, the crank's route and
   whether the plan is proven the thinnest, or a :class:`stack.PlanError`
   naming what blocked it, the static clearances involved and what would
   clear it (:mod:`recommend`: each recommendation checked by re-running).

Nothing here computes anything the pipeline doesn't: the stages report
their own failures.
"""

from __future__ import annotations

import argparse

from spiderpig import linkage


def explain(key: str, module: str = "single", params=None, phases=None) -> str:
    """The stages' verdicts for a linkage, module and proportions at the default servo,
    sheet, constructions and fit (the CLI); :func:`explain_config` takes a full config."""
    from spiderpig.config import BuildConfig

    config = BuildConfig(linkage=key, module=module, robot=False, phases=phases,
                         proportions=tuple(sorted((params or {}).items())))
    return explain_config(config)


def explain_config(config, side=None, plan_failure: str | None = None) -> str:
    """The stages' verdicts for one side of ``config`` (its servo, sheet, constructions
    and fit included). ``side``: an already designed :class:`fabricate.SideDesign` to
    describe instead of solving again; ``plan_failure``: the planner's recorded message
    for a design known to fail, printed instead of searching again."""
    from dataclasses import replace

    from spiderpig.fabricate import (
        design_side,
        ground_clearance,
        side_problem,
        static_stage,
        template_for,
    )

    config = replace(config, robot=False)
    key, module = config.linkage, config.module
    params = dict(config.proportions) or None
    lk = linkage.get(key)
    lines = [f"{lk.name} [{key}], module {module}", "", "1. program"]
    steps = lk.check(params)
    lines += [f"  {s.describe()}" for s in steps]
    if lk.output and all(s.fails_deg is None for s in steps):
        lines.append(f"  output: {lk.output_check(params).describe()}")
    try:
        tmpl = template_for(config)
        ctx, _, problem = side_problem(tmpl, config)
    except ValueError as e:     # AssemblyError / OutputError; ConstructionError (e.g. the drive)
        return "\n".join([*lines, "", f"STOP: {e}"])
    lines += ["", f"2. static facts ({len(problem.clearances)} clearances)"]
    lines += [f"  {c.describe()}" for c in problem.clearances]
    if problem.router is not None:
        f = problem.router.facts
        for link, d in f.o_free.items():
            lines.append(f"  {link} passes O at {max(d, 0.0):.1f} mm: its layer needs the crank "
                         f"running along {', '.join(f.hosts[link]) or 'no crank point'}")
        lines += [f"  detour {d.name}: {d.r:g} mm from O, {d.angle:g}° from the first crankpin, "
                  f"sweeping {d.sweep:g} mm" for d in f.detours]
        lines.append(f"  body's underside: lowest at {f.envelope.lowest:.1f} mm (O at 0); what "
                     f"the planner adds to the crank may sweep {f.allow:.1f} mm about O")
    if (gc := ground_clearance(tmpl, ctx)) is not None:
        lines.append(f"  ground clearance: {gc:.1f} mm")
    try:        # a recorded failure carries its own recommendations: don't check them again
        static_stage(tmpl, problem, None if plan_failure is not None else config)
    except ValueError as e:
        return "\n".join([*lines, "", f"STOP: {plan_failure or e}"])
    lines += ["", "3. plan"]
    if plan_failure is not None and side is None:
        lines.append(f"  STOP: {plan_failure}")
        return "\n".join(lines)
    try:
        d = side if side is not None else design_side(tmpl, config)
        lines.append(f"  {d.plan.top + 1} layers, {d.plan.height:.0f} mm; "
                     + ("optimal: " if d.plan.optimal else "not proven optimal: ") + d.plan.proof)
        if "crank" in d.plan.choices:
            lines.append(f"  crank: {d.plan.choices['crank']}")
        lines += d.plan.describe().splitlines()
    except ValueError as e:
        lines.append(f"  STOP: {e}")
    return "\n".join(lines)


def main(argv=None) -> int:
    from spiderpig.config import ParamError, add_design_args, config_from_args

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    add_design_args(ap)
    ap.set_defaults(module="single")
    args = ap.parse_args(argv)
    try:
        config = config_from_args(args, robot=False)
    except ParamError as e:
        ap.error(str(e))
    print(explain(config.linkage, config.module, dict(config.proportions) or None,
                  config.phases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
