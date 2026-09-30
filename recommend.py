"""What would clear a failure of the planner's stages, checked before it is said.

A static failure is a gap between two numbers: how close the linkage brings
a link to something (a crankpin's post, an axle's neck), and what the parts
there need (post or neck radius + link half-width + margin). Two ways close
it:

* **scale** the linkage (its parameters that only scale it, like ``unit`` or
  Klann's ``OA``, :func:`linkage.scale_params`): distances between the
  linkage's points grow with it, part sizes don't. The least scale that
  clears every gap, rounded up to a practical value (0.5 mm steps, 0.05 for
  sub-2 mm units: the distances are lower bounds, so a zero-margin fit fails
  by the sampling correction);
* **thinner parts** at this scale: the :class:`construction.base.Params` part
  sizes behind the gap, within what each construction's ``dims()`` accepts
  (a thinner link needs a thinner axle, ``min_wall`` round its hole).

Each :class:`stack.Recommendation` is verified: the static stage is run again
with it (and the plan, where that's cheap), and it says what that showed.
The checks are bounded: together they get one planner deadline
(:attr:`stack.StackSpec.max_seconds`, or ``seconds``), each planning run
inside it what is left, so a failure that took the planner its whole
budget can't take many times that to advise on. A candidate left unchecked
when it runs out is noted, never recommended.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import linkage
from stack import Clearance, Deadline, Recommendation, StackSpec


@dataclass(frozen=True)
class Gap:
    """``have`` (mm, scales with the linkage) must reach ``margin`` plus the part sizes
    ``terms`` add up to (``Params`` field -> coefficient)."""

    what: str
    have: float
    terms: tuple[tuple[str, float], ...]
    margin: float

    def need(self, params) -> float:
        return self.margin + sum(c * getattr(params, f) for f, c in self.terms)


def gaps_of(failures=(), clearances: tuple[Clearance, ...] = (), params=None) -> list[Gap]:
    """The gaps behind the static stage's failures (:class:`construction.route.NoCrankPoint`)
    and the static clearances behind a plan's failure (an axle's neck)."""
    out = [Gap(f"{f.link} past crankpin {f.pin}", f.dist,
               (("crankpin_d", 0.5), ("link_radius", 1.0)), f.margin) for f in failures]
    for c in clearances:
        if c.keepout.span and c.dist > 0:
            out.append(Gap(f"{c.link} past {c.keepout.owner}", c.dist,
                           (("neck_d", 0.5), ("link_radius", 1.0)),
                           c.need - c.keepout.r - params.link_radius))
    return out


def _step(value: float) -> float:
    return 0.5 if value >= 2 else 0.05


def _up(value: float, step: float) -> float:
    return round(math.ceil(value / step - 1e-9) * step, 6)


class _OutOfTime(Exception):
    """The deadline for checking recommendations ran out before a check could start."""


def _check(config) -> str | None:
    """None if ``config`` passes the static stage; else why not."""
    from fabricate import side_problem, static_stage, template_for

    try:
        tmpl = template_for(config)
        _, _, problem = side_problem(tmpl, config, hint=False)
        static_stage(tmpl, problem)
    except ValueError as e:
        return str(e).splitlines()[0]
    return None


def _verify(config, plan: bool, deadline: Deadline | None = None) -> str | None:
    """What re-running showed, or None if it still fails (or its search ran out of the
    deadline). ``plan``: the plan failed, so it is planned again; else the plan is checked
    where that's cheap (a single or double module; for a bigger one, its single module).
    :class:`_OutOfTime` when ``deadline`` had run out before the check."""
    from fabricate import design_side, template_for

    deadline = deadline or Deadline(StackSpec().max_seconds)
    if deadline.expired:
        raise _OutOfTime
    if _check(config) is not None:
        return None
    trial = config
    if not plan and config.module not in ("single", "double"):
        trial = replace(config, module="single", phases=None)
    try:
        d = design_side(template_for(trial), trial, advise=False, deadline=deadline)
    except ValueError:
        return None
    where = "" if trial is config else " (its single module)"
    return (f"checked: the static stage passes, and it plans{where} in {d.plan.top + 1} "
            f"layers ({d.plan.height:g} mm)")


def scale(config, gaps: list[Gap], plan: bool = False,
          deadline: Deadline | None = None) -> Recommendation | None:
    """The least practical uniform scale of the linkage that clears every gap (verified)."""
    lk = linkage.get(config.linkage)
    names = linkage.scale_params(lk)
    open_ = [g for g in gaps if g.have > 0]
    if not names or not open_:
        return None
    name = names[0]
    props = dict(config.proportions)
    now = float(props.get(name, lk.params[name]))
    least = max(g.need(config.params) / g.have for g in open_)
    step = _step(now)
    value = _up(now * max(least, 1.0), step)
    if value <= now:
        value = _up(now + step, step)
    for _ in range(3 if plan else 6):
        trial = replace(config, proportions=tuple(sorted({**props, name: value}.items())))
        try:
            verified = _verify(trial, plan, deadline)
        except _OutOfTime:
            raise _OutOfTime(f"a scale of the linkage from {name} {value:g} up") from None
        if verified is not None:
            s = value / now
            crank = math.hypot(*lk.solve(params=props).joints_at(0.0)[lk.crank[1]])
            worst = max(open_, key=lambda g: g.need(config.params) / g.have)
            return Recommendation(
                ((name, now, value),),
                why=(f"scale {config.linkage} x{s:.2f} (the least that clears it is x{least:.2f}"
                     f"; {worst.what} is {worst.have:.1f} mm, needs "
                     f"{worst.need(config.params):.1f})"),
                effects=f"crank {crank:.1f} -> {crank * s:.1f} mm; about {s:.1f}x the crank "
                        "torque for the same foot force",
                verified=verified)
        value = _up(value + step, step)
    return None


def thinner(config, gaps: list[Gap], plan: bool = False,
            deadline: Deadline | None = None) -> Recommendation | str:
    """Thinner parts at this scale that clear every gap and every construction accepts
    (verified); or why none does."""
    p = config.params
    fields = sorted({f for g in gaps for f, _ in g.terms})
    if any(g.have <= 0 for g in gaps):
        return "no part size clears a link that sweeps right across the part"

    def options(f: str) -> list[float]:
        now = getattr(p, f)
        return [round(now - 0.5 * i, 3) for i in range(0, 13) if now - 0.5 * i >= 2.0]

    cands = []
    for values in _product([options(f) for f in fields]):
        q = replace(p, **dict(zip(fields, values, strict=True)))
        if all(g.need(q) <= g.have - 0.05 for g in gaps):
            loss = sum((getattr(p, f) - v) / getattr(p, f) for f, v in zip(fields, values,
                                                                           strict=True))
            cands.append((loss, q))
    why = None                  # why the least change that clears the gaps can't be built
    for i, (_, q) in enumerate(sorted(cands, key=lambda c: c[0])[:8]):
        # a thinner link keeps min_wall round its axles' holes: thin the axle with it
        axle = min(p.axle_d, math.floor(4 * (q.link_radius - q.min_wall) - 2 * q.running_fit) / 2)
        q = replace(q, axle_d=max(axle, 2.0), neck_d=min(q.neck_d, max(axle, 2.0)))
        trial = replace(config, params=q)
        try:
            verified = _verify(trial, plan, deadline)
        except _OutOfTime:
            raise _OutOfTime("thinner parts" + (f" beyond the {i} sizes checked" if i else "")
                             ) from None
        if verified is not None:
            changes = tuple((f, getattr(p, f), getattr(q, f))
                            for f in ("link_radius", "crankpin_d", "axle_d", "neck_d")
                            if getattr(q, f) != getattr(p, f))
            return Recommendation(changes, why="thinner parts at this scale", verified=verified)
        why = why or _check(trial) or "it still doesn't plan"
    if why is None:
        return "no part sizes at this scale clear it"
    return ("no part sizes at this scale clear it within the constructions' limits (the "
            f"least: {why})")


def _product(lists):
    if not lists:
        yield ()
        return
    for v in lists[0]:
        for rest in _product(lists[1:]):
            yield (v, *rest)


_DONE: dict[tuple, tuple[list[Recommendation], list[str]]] = {}


def recommend(config, failures=(), clearances=(), plan: bool = False,
              seconds: float | None = None) -> tuple[list[Recommendation], list[str]]:
    """Checked recommendations for these failures, and notes on what can't help (remembered
    per design: checking them plans other designs). The checks share ``seconds`` of wall
    clock (the planner's own ``StackSpec.max_seconds`` by default); what they didn't get
    to is noted."""
    gaps = gaps_of(failures, tuple(clearances), config.params)
    if not gaps:
        return [], []
    key = (config, tuple(gaps), plan, seconds)
    if key not in _DONE:
        _DONE[key] = _recommend(config, gaps, plan, seconds)
    return _DONE[key]


def _recommend(config, gaps: list[Gap], plan: bool,
               seconds: float | None = None) -> tuple[list[Recommendation], list[str]]:
    deadline = Deadline(StackSpec().max_seconds if seconds is None else seconds)
    recs, notes, unchecked = [], [], []
    try:
        if (r := scale(config, gaps, plan, deadline)) is not None:
            recs.append(r)
    except _OutOfTime as e:
        unchecked.append(str(e) or "a scale of the linkage")
    try:
        t = thinner(config, gaps, plan, deadline)
    except _OutOfTime as e:
        unchecked.append(str(e) or "thinner parts")
    else:
        if isinstance(t, Recommendation):
            recs.append(t)
        else:
            notes.append(t)
    if unchecked:
        notes.append(f"not checked, the {deadline.seconds:g} s for checking what would clear "
                     f"it ran out: {' and '.join(unchecked)}")
    return recs, notes
