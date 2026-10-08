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
from typing import TYPE_CHECKING

from spiderpig import linkage
from spiderpig.stack import Clearance, Deadline, Recommendation, StackSpec

if TYPE_CHECKING:
    from spiderpig.construction.base import Params


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


def gaps_of(failures=(), clearances: tuple[Clearance, ...] = (),
            params: Params | None = None) -> list[Gap]:
    """The gaps behind the static stage's failures (:class:`construction.route.NoCrankPoint`)
    and the static clearances behind a plan's failure (an axle's neck)."""
    # the post is the crank's own (its standoff or sleeve, BoltCrank.rider_d): no Params
    # field shrinks it, so it is part of the margin
    out = [Gap(f"{f.link} past crankpin {f.pin}", f.dist, (("link_radius", 1.0),),
               f.margin + f.post) for f in failures]
    # the axle's narrowest ring is its construction's own (a standoff's or a Chicago
    # barrel's ring round its bore): a thinner link narrows the gap's need, no Params
    # field narrows the ring
    if clearances:
        assert params is not None  # recommend(), the one caller, passes the config's
        out.extend(Gap(f"{c.link} past {c.keepout.owner}", c.dist, (("link_radius", 1.0),),
                       c.need - params.link_radius)
                   for c in clearances if c.keepout.span and c.dist > 0)
    return out


def _step(value: float) -> float:
    return 0.5 if value >= 2 else 0.05


def _up(value: float, step: float) -> float:
    return round(math.ceil(value / step - 1e-9) * step, 6)


class _OutOfTime(Exception):
    """The deadline for checking recommendations ran out before a check could start."""


def _check(config) -> str | None:
    """None if ``config`` passes the static stage; else why not."""
    from spiderpig.fabricate import side_problem, static_stage, template_for

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
    from spiderpig.fabricate import design_side, template_for

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
    if trial is config:
        own = "" if config.module == "single" else f" ({config.module} module, the design's own)"
        return (f"checked: the static stage passes, and it plans{own} in {d.plan.top + 1} layers "
                f"({d.plan.height:g} mm)")
    return (f"checked: the static stage passes, and its single module plans in "
            f"{d.plan.top + 1} layers ({d.plan.height:g} mm); the {config.module} module's own "
            f"plan is not checked here (plan the derived design: the planner's deadline is "
            f"{StackSpec().max_seconds:g} s, and a bigger module stacks taller)")


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
        return [round(now - 0.5 * i, 3) for i in range(13) if now - 0.5 * i >= 2.0]

    cands = []
    for values in _product([options(f) for f in fields]):
        q = replace(p, **dict(zip(fields, values, strict=True)))
        if all(g.need(q) <= g.have - 0.05 for g in gaps):
            loss = sum((getattr(p, f) - v) / getattr(p, f) for f, v in zip(fields, values,
                                                                           strict=True))
            cands.append((loss, q))
    why = None                  # why the least change that clears the gaps can't be built
    for i, (_, q) in enumerate(sorted(cands, key=lambda c: c[0])[:8]):
        trial = replace(config, params=q)
        try:
            verified = _verify(trial, plan, deadline)
        except _OutOfTime:
            raise _OutOfTime("thinner parts" + (f" beyond the {i} sizes checked" if i else "")
                             ) from None
        if verified is not None:
            changes = tuple((f, getattr(p, f), getattr(q, f)) for f in fields
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


def default_scale(config, plan: bool = True,
                  deadline: Deadline | None = None) -> Recommendation | None:
    """A plan that failed on no link-to-axle gap (crank routes, pin heads against the
    plates: the stack's own room) after the linkage was scaled *down* from its registered
    default: the default scale, verified. ``None`` when the linkage is at or above its
    default, or the default doesn't plan either."""
    lk = linkage.get(config.linkage)
    names = linkage.scale_params(lk)
    if not names:
        return None
    name = names[0]
    props = dict(config.proportions)
    now, default = float(props.get(name, lk.params[name])), float(lk.params[name])
    if now >= default:
        return None
    trial = replace(config, proportions=tuple(sorted({**props, name: default}.items())))
    verified = _verify(trial, plan, deadline)          # _OutOfTime crosses to the caller
    if verified is None:
        return None
    return Recommendation(
        ((name, now, default),),
        why=(f"back to the linkage's default scale (x{default / now:.2f}): the search ran "
             f"into the stack's own room (the crank's route, pin heads against the frame "
             f"plates), which more distance between the joints clears"),
        effects=f"every length x{default / now:.2f}; the foot path and the envelope with it",
        verified=verified)


def target_scale(config, misses, measure, deadline: Deadline | None = None,
                 tries: int = 4, keep=()) -> tuple[Recommendation | None, str | None]:
    """The least practical scale of the linkage that meets every missed target in
    ``misses`` (``(path, value, target)`` triples of metrics linear in the scale parameter:
    a mechanism's stroke and straightness, a walker's lift), checked by measuring them
    again (``measure(config) -> {path: value}``) and by the static stage and the design's
    own plan. ``keep``: the scaled targets the design meets now (the same triples), which
    the scale must keep meeting: they bound it too and are measured again with the
    misses. ``(recommendation, note)``: the note says why none is given."""
    lk = linkage.get(config.linkage)
    names = linkage.scale_params(lk)
    if not names:
        return None, f"no parameter of {config.linkage} only scales it: nothing to scale"
    name = names[0]
    props = dict(config.proportions)
    now = float(props.get(name, lk.params[name]))
    lo, hi = 0.0, math.inf                     # the scale factors the targets allow
    bound = [*misses, *keep]
    for _path, v, t in bound:
        if v <= 0:
            continue
        if t.min is not None:
            lo = max(lo, t.min / v)
        if t.max is not None:
            hi = min(hi, t.max / v)
        if t.value is not None:
            tol = t.tol if t.tol is not None else abs(t.value) * 0.05
            lo, hi = max(lo, (t.value - tol) / v), min(hi, (t.value + tol) / v)
    metrics = ", ".join(p.split(".")[-1] for p, _, _ in misses)
    if lo > hi + 1e-9:
        every = ", ".join(p.split(".")[-1] for p, _, _ in bound)
        return None, (f"no one scale of {name} meets {every} together (one needs x{lo:.3g}, "
                      f"another at most x{hi:.3g})")
    step = _step(now)
    if lo > 0 and hi < math.inf:
        # a band (a value and its tolerance, or a min and a max): from its middle outward,
        # on the practical steps (an end's step can fall outside a narrow band)
        s = (lo + hi) / 2
        mid = now * s
        base = round(round(mid / step) * step, 6)
        values = sorted({round(base + k * step, 6) for k in range(-tries, tries + 1)},
                        key=lambda v: (abs(v - mid), v))
        values = [v for v in values if abs(v - now) > 1e-9][:tries]
    else:
        s = lo if lo > 0 else hi
        value = (_up(now * s, step) if s >= 1
                 else round(math.floor(now * s / step + 1e-9) * step, 6))
        if abs(value - now) < 1e-9:
            value = _up(now + step, step) if s >= 1 else round(now - step, 6)
        values = []
        for _ in range(tries):              # away from the bound, toward what meets it
            values.append(value)
            value = _up(value + step, step) if s >= 1 else round(value - step, 6)
    deadline = deadline or Deadline(StackSpec().max_seconds)
    for value in values:
        if value <= 0:
            break
        trial = replace(config, proportions=tuple(sorted({**props, name: value}.items())))
        try:
            got = measure(trial)
        except ValueError as e:                  # a loop that no longer closes
            return None, f"{name} {value:g} breaks the linkage: {str(e).splitlines()[0]}"
        if all(t.check(got[p])[0] for p, _, t in bound if p in got):
            try:
                verified = _verify(trial, True, deadline)
            except _OutOfTime:
                return None, (f"not checked, the {deadline.seconds:g} s for checking what "
                              f"would meet it ran out: {name} {value:g}")
            if verified is None:
                why = _check(trial) or "it doesn't plan"
                return None, f"{name} {value:g} meets {metrics} but doesn't build: {why}"
            what = "; ".join(f"{p.split('.')[-1]} {got[p]:.4g}" for p, _, _ in misses if p in got)
            plans = verified.removeprefix("checked: the static stage passes, and ")
            one = len(misses) == 1
            return Recommendation(
                ((name, now, value),),
                why=(f"{metrics} scale{'s' if one else ''} with {name}: x{value / now:.3g} "
                     f"meets the target{'' if one else 's'}"),
                effects=(f"every length x{value / now:.2f}; the envelope and the crank torque "
                         "with it"),
                verified=f"checked: {what}; {plans}"), None
    return None, f"no practical {name} near x{s:.3g} ({now * s:.3g}) meets {metrics}"


CONFIG_LEVERS = {"thickness_mm": "thickness", "sheet": "sheet", "servo": "servo",
                 "pillar": "pillar", "pin": "pin", "crank": "crank"}
"""A construction's change (``ConstructionError.changes``) by its ``BuildConfig`` field:
every one the spec sets (:data:`failure.CONFIG_FIELDS`)."""


def _val(v) -> str:
    """A change's value as written: a number in ``g`` form, a key as it is."""
    return f"{v:g}" if isinstance(v, (int, float)) and not isinstance(v, bool) else str(v)


def construction_fix(config, exc, seconds: float | None = None
                     ) -> tuple[list[Recommendation], list[str]]:
    """What a construction said would clear its :class:`construction.base.ConstructionError`
    (``exc.changes``: a config field such as ``thickness_mm``, or a ``Params`` field),
    checked by re-running the static stage and the plan with it (the design's own module,
    within one planner deadline); and notes on what wasn't checked or didn't build."""
    changes: tuple[tuple[str, object, object], ...] = tuple(getattr(exc, "changes", ()) or ())
    if not changes:
        return [], []
    deadline = Deadline(StackSpec().max_seconds if seconds is None else seconds)
    fields: dict = {}
    params: dict = {}
    unknown = []
    for name, _before, after in changes:
        if name in CONFIG_LEVERS:
            fields[CONFIG_LEVERS[name]] = after
        elif hasattr(config.params, name):
            params[name] = after
        else:
            unknown.append(name)
    notes = [f"{n} is not a field this API sets: apply it by hand" for n in unknown]
    if not fields and not params:
        return [], notes
    trial = replace(config, **fields)
    if params:
        trial = replace(trial, params=replace(config.params, **params))
    what = ", ".join(f"{n} {_val(b)} -> {_val(a)}" for n, b, a in changes)
    try:
        verified = _verify(trial, plan=True, deadline=deadline)
    except _OutOfTime:
        return [], [*notes, f"not checked, the {deadline.seconds:g} s for checking what would "
                            f"clear it ran out: {what}"]
    if verified is None:
        why = _check(trial) or "it still doesn't plan"
        return [], [*notes, f"{what} doesn't build either: {why}"]
    why = getattr(exc, "lever", "") or str(exc).split(":", 1)[0]
    return [Recommendation(changes, why=why, verified=verified)], notes


_DONE: dict[tuple, tuple[list[Recommendation], list[str]]] = {}


def recommend(config, failures=(), clearances=(), plan: bool = False,
              seconds: float | None = None) -> tuple[list[Recommendation], list[str]]:
    """Checked recommendations for these failures, and notes on what can't help (remembered
    per design: checking them plans other designs). The checks share ``seconds`` of wall
    clock (the planner's own ``StackSpec.max_seconds`` by default); what they didn't get
    to is noted. A plan failure with no gap behind it (``plan`` and no ``clearances`` that
    open one) is answered with :func:`default_scale` when the linkage was scaled down,
    else with a note saying why nothing is recommended."""
    gaps = gaps_of(failures, tuple(clearances), config.params)
    if not gaps and not plan:
        return [], []
    key = (config, tuple(gaps), plan, seconds)
    if key not in _DONE:
        _DONE[key] = _recommend(config, gaps, plan, seconds)
    return _DONE[key]


def _recommend(config, gaps: list[Gap], plan: bool,
               seconds: float | None = None) -> tuple[list[Recommendation], list[str]]:
    deadline = Deadline(StackSpec().max_seconds if seconds is None else seconds)
    recs, notes, unchecked = [], [], []
    if not gaps:
        try:
            if (r := default_scale(config, plan, deadline)) is not None:
                recs.append(r)
        except _OutOfTime:
            unchecked.append("the linkage's default scale")
        if not recs:
            lk = linkage.get(config.linkage)
            names = linkage.scale_params(lk)
            scaled_down = bool(names) and float(dict(config.proportions).get(
                names[0], lk.params[names[0]])) < float(lk.params[names[0]])
            notes.append(
                "no link passes an axle too closely, so no part size or scale could be "
                "computed from a gap: what blocked the search is the stack's own room "
                "(the crank's route, pin heads against the frame plates)"
                + ("; the linkage's default scale doesn't plan either"
                   if scaled_down and not unchecked else
                   "; levers left: a bigger scale of the linkage, another module"))
        if unchecked:
            notes.append(f"not checked, the {deadline.seconds:g} s for checking what would "
                         f"clear it ran out: {' and '.join(unchecked)}")
        return recs, notes
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
