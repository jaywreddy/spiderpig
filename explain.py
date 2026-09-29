"""Print what each stage of the pipeline says about a design.

    uv run python explain.py --linkage strider --module double

1. program: :meth:`linkage.Linkage.check`. Each loop's closing margin and
   transmission angle; a loop that can't close raises
   :class:`linkage.AssemblyError` when the template is built.
2. static clearances: :func:`fabricate.side_clearances`. Each link that
   sweeps through some group's keep-out, so the two can never share a layer.
3. plan: :func:`fabricate.design_side`. The layer plan, or a
   :class:`stack.PlanError` naming what blocked it, whether stacking a
   smaller module's plan also failed, and the static clearances involved.

Nothing here computes anything the pipeline doesn't: the stages report
their own failures.
"""

from __future__ import annotations

import argparse
import math

import linkage


def explain(key: str, module: str = "single", params=None, phases=None) -> str:
    from fabricate import BuildConfig, design_side, side_clearances, side_problem, template_for

    lk = linkage.get(key)
    config = BuildConfig(linkage=key, module=module, robot=False, phases=phases,
                         proportions=tuple(sorted((params or {}).items())))
    lines = [f"{lk.name} [{key}], module {module}", "", "1. program"]
    lines += [f"  {s.describe()}" for s in lk.check(params)]
    try:
        tmpl = template_for(config)
    except linkage.AssemblyError as e:
        return "\n".join([*lines, "", f"STOP: {e}"])
    ctx, groups, _ = side_problem(tmpl, config)
    clear = side_clearances(ctx, groups)
    lines += ["", f"2. static clearances ({len(clear)})"]
    lines += [f"  {c.describe()}" for c in clear]
    lines += ["", "3. plan"]
    try:
        d = design_side(tmpl, config)
        lines.append(f"  {d.plan.top + 1} layers, {d.plan.height:.0f} mm")
        lines += d.plan.describe().splitlines()
    except ValueError as e:
        lines.append(f"  STOP: {e}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--linkage", default=linkage.DEFAULT, choices=linkage.available())
    ap.add_argument("--module", default="single")
    ap.add_argument("--phases", default=None, help="comma-separated degrees, one per leg")
    ap.add_argument("--param", action="append", default=[], help="NAME=VALUE (repeatable)")
    args = ap.parse_args(argv)
    params = {k: float(v) for k, v in (p.split("=", 1) for p in args.param)} or None
    phases = (tuple(math.radians(float(x)) for x in args.phases.split(","))
              if args.phases else None)
    print(explain(args.linkage, args.module, params, phases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
