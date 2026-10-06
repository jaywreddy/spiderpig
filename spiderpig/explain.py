"""Print what each stage of the pipeline says about a design.

    spiderpig explain --linkage strider --module double
    spiderpig explain --linkage trotbot_heel --proportion unit=7
    spiderpig explain --module quad --pin bearing --servo xl330_m288 --thickness 2

The build options (``--servo``, ``--pillar`` / ``--pin`` / ``--crank``, ``--sheet``,
``--thickness``) are the same as ``spiderpig build``'s: the static facts and the
plan depend on them.

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

4. strength (``--strength``): :func:`strength.check` on the side fabricated at
   ``t = 1``: every pin's, pillar's and the crank's safety factor at the design's own
   loads (MuJoCo, cached per design in the store: :mod:`sim.loads`), and each joint
   under the limits (jam SF 1 an error, 2 jammed or 3 walking a warning) with its fixes.

Nothing here computes anything the pipeline doesn't: the stages report
their own failures.
"""

from __future__ import annotations

import argparse
import sys

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
        if side is not None:        # a designed side carries its static facts
            ctx, clearances, facts = side.ctx, side.clearances, side.facts
            problem = None
        else:
            # the static facts need no leg hint (the single module's plan): hint=False
            ctx, _, problem = side_problem(tmpl, config, hint=False)
            clearances = problem.clearances
            facts = problem.router.facts if problem.router is not None else None
    except ValueError as e:     # AssemblyError / OutputError; ConstructionError (e.g. the drive)
        return "\n".join([*lines, "", f"STOP: {e}"])
    lines += ["", f"2. static facts ({len(clearances)} clearances)"]
    lines += [f"  {c.describe()}" for c in clearances]
    if facts is not None:
        f = facts
        for link, d in f.o_free.items():
            lines.append(f"  {link} passes O at {max(d, 0.0):.1f} mm: its layer needs the crank "
                         f"running along {', '.join(f.hosts[link]) or 'no crank point'}")
        lines += [f"  detour {d.name}: {d.r:g} mm from O, {d.angle:g}° from the first crankpin, "
                  f"sweeping {d.sweep:g} mm" for d in f.detours]
        lines.append(f"  body's underside: lowest at {f.envelope.lowest:.1f} mm (O at 0); what "
                     f"the planner adds to the crank may sweep {f.allow:.1f} mm about O")
    gc = side.ground_clearance_mm if side is not None else ground_clearance(tmpl, ctx)
    if gc is not None:
        lines.append(f"  ground clearance: {gc:.1f} mm")
    if problem is not None:     # a designed side passed the static stage
        try:    # a recorded failure carries its own recommendations: don't check them again
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
    from spiderpig.config import ParamError, add_build_args, add_design_args, config_from_args

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    add_design_args(ap)
    add_build_args(ap)
    ap.add_argument("--store", metavar="PATH",
                    help="the design store the options resolve into, whose plan is reused "
                         "(default: $SPIDERPIG_STORE, else ./.spiderpig)")
    ap.add_argument("--strength", action="store_true",
                    help="4. the joints' safety factors at the design's own loads (builds "
                         "the side and simulates the robot once per design, ~30 s)")
    ap.add_argument("--no-sim", action="store_true",
                    help="with --strength: the family's loads, no sim")
    args = ap.parse_args(argv)
    try:
        config = config_from_args(args, robot=False)
    except ParamError as e:
        ap.error(str(e))
    from spiderpig import api
    from spiderpig.store import Store

    for w in api.config_warnings(config, sides=2):     # what resolve would warn about (one
        print(f"warning: {w}", file=sys.stderr)        # side is what explain always shows)
    # the plan through the store (api.plan_config): the stored design's when it holds one
    store = Store.of(args.store) if args.store else Store.default()
    try:
        side, failure = api.plan_config(config, store), None
    except ValueError as e:
        side, failure = None, str(e)
    print(explain_config(config, side=side, plan_failure=failure))
    if args.strength and side is not None:
        print("\n" + strength_text(config, side, store, sim=not args.no_sim))
    return 0


def strength_text(config, side, store=None, sim: bool = True) -> str:
    """Stage 4: the strength check of ``side`` (designed) at the design's loads."""
    from dataclasses import replace

    from spiderpig import strength
    from spiderpig.fabricate import fabricate_side, template_for
    from spiderpig.tools.audit import strength_lines

    fab = fabricate_side(side, template_for(config).freeze_at(1.0))
    from spiderpig.config import default_robot

    # a walker's loads are the robot's (it walks on both sides); a mechanism has one side
    loads = strength.design_loads(replace(config, robot=default_robot(config.linkage)), store,
                                  sim=sim)
    st = strength.check(fab.meta.get("wobble") or {}, fab.meta, config, loads)
    return "\n".join(["4. strength", *(f"  {x}" for x in strength_lines(st))])


if __name__ == "__main__":
    raise SystemExit(main())
