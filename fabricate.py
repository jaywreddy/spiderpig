"""Fabrication: turn a :class:`stack.StackPlan` into build123d parts.

Given a mechanism frozen at some crank angle ``t`` and the plan for its
assembly, :func:`fabricate` returns a copy whose bodies carry parts built in
world coordinates at ``t``:

* every leg link (b1..b4) as a laser-cut plate in its slot, drilled at its pivots;
* the frame (the ``torso`` body) as one printed solid: a plate in the top
  slot with arms to every fixed pivot, posts down toward the links wherever
  they clear, and a servo pad around the crank centre;
* the crankshaft as rigid segments (webs + journal) with crankpins across the
  b1 slots; the top segment carries the hub stub and the key the servo grips;
* with ``joinery``: a pin and a separate press-on cap at every pin and pivot,
  plus sleeves in the gaps where they fit.

Hardware bodies have no joints; ``Body.rigid_with`` names the body they move
with, and their parts share that body's frame.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from mechanism import Body, Mechanism
from shapes import (
    BUFF,
    FLANGE_R,
    HOLE_R,
    HUB_R,
    JOURNAL_R,
    KEY,
    PIN_R,
    cap,
    disc,
    drill,
    link_plate,
    pill,
    pin,
    sleeve,
    square_key,
)
from stack import StackPlan, StackSpec, body_class, problem_from_template

_PLANS: dict[tuple, StackPlan] = {}

# Servo (SG90-class) on the frame plate: output shaft on O, body along ``u``.
_SERVO_SHAFT_OFFSET = 5.5   # shaft axis to servo body centre
_SERVO_HOLE_SPAN = 27.5     # between the two mounting-tab screw holes
_SERVO_SCREW_R = 1.0
_KEY_HEIGHT = 5.0


def plan_for(tmpl, spec: StackSpec | None = None) -> StackPlan:
    """Solve (once per process) the stack plan for a mechanism template."""
    spec = spec or StackSpec()
    key = (tmpl.name, tuple(b.name for b in tmpl.bodies), tuple(tmpl.connections), spec)
    if key not in _PLANS:
        _PLANS[key] = problem_from_template(tmpl, spec).solve()
    return _PLANS[key]


def _runs(slots: list[int]) -> list[list[int]]:
    """Split sorted slots into runs of consecutive integers."""
    runs: list[list[int]] = []
    for s in slots:
        if runs and s == runs[-1][-1] + 1:
            runs[-1].append(s)
        else:
            runs.append([s])
    return runs


def _union(parts):
    parts = [p for p in parts if p is not None]
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def fabricate(mech: Mechanism, plan: StackPlan, *, joinery: bool = True) -> Mechanism:
    """Attach parts (and hardware bodies) for ``plan`` to a copy of ``mech``."""
    bodies = {b.name: replace(b) for b in mech.bodies}

    def pos(body: str, joint: str) -> np.ndarray:
        b = bodies[body]
        return (b.pose @ b.joint(joint).pose).matrix[:2, 3]

    z = plan.z
    crank = plan.crank
    pivots = {n for ax in plan.axes for n in ax.joints}
    if crank is not None:
        pivots |= {n for nodes in crank.joints.values() for n in nodes}
        pivots |= set(crank.center_joints)

    # -- leg links ------------------------------------------------------------
    for name, slot in plan.slots.items():
        b = bodies[name]
        segs = [(pos(name, p), pos(name, q)) for p, q in b.outline]
        holes = [pos(name, j.name) for j in b.joints if (name, j.name) in pivots]
        b.part = link_plate(segs, *z(slot), holes=holes)

    hardware: list[Body] = []
    frame = next(n for n in bodies if body_class(n) == "torso")
    center = pos(*crank.center_joints[0]) if crank else np.zeros(2)
    axis_xy = {i: pos(*ax.joints[0]) for i, ax in enumerate(plan.axes)}
    extra: dict[int, dict[int, float]] = {}   # optional discs added so far

    # -- frame ----------------------------------------------------------------
    pz0, pz1 = z(plan.plate)
    frame_axes = [i for i, ax in enumerate(plan.axes) if ax.kind == "frame"]
    shapes = [pill(center, axis_xy[i], BUFF, pz0, pz1) for i in frame_axes]
    away = -sum((axis_xy[i] - center for i in frame_axes), np.zeros(2))
    u = away / (np.linalg.norm(away) or 1.0)
    servo_mid = center + u * _SERVO_SHAFT_OFFSET
    screws = [servo_mid + u * _SERVO_HOLE_SPAN / 2, servo_mid - u * _SERVO_HOLE_SPAN / 2]
    shapes.append(pill(screws[0] + u * 3, screws[1] - u * 3, BUFF + 1, pz0, pz1))
    post_bottom = pz0
    for i in frame_axes:
        _, hi = plan.span(plan.axes[i])
        for k in range(plan.plate - 1, hi, -1):
            if not plan.disc_fits(i, k, FLANGE_R, extra):
                break
            extra.setdefault(i, {})[k] = FLANGE_R
            shapes.append(disc(axis_xy[i], FLANGE_R, z(k)[0], pz0 + 0.5))
            post_bottom = min(post_bottom, z(k)[0])
    bodies[frame].part = drill(
        _union(shapes),
        [(axis_xy[i], HOLE_R) for i in frame_axes]
        + [(center, HUB_R + 0.5)]
        + [(s, _SERVO_SCREW_R) for s in screws],
        post_bottom, pz1,
    )

    # -- crankshaft -----------------------------------------------------------
    if crank is not None:
        host = next(n for n in bodies if body_class(n).startswith("conn"))
        pin_xy = {p: pos(*nodes[0]) for p, nodes in crank.joints.items()}
        runs = _runs(list(plan.journal_slots))
        for idx, run in enumerate(reversed(runs)):   # top run first
            shapes = []
            web_pins: set[str] = set()
            for k in run:
                shapes.append(disc(center, JOURNAL_R, *z(k)))
                for p in plan.webs.get(k, ()):
                    shapes.append(pill(center, pin_xy[p], BUFF, *z(k)))
                    web_pins.add(p)
            seg = drill(_union(shapes), [(pin_xy[p], PIN_R) for p in sorted(web_pins)],
                        z(run[0])[0], z(run[-1])[1])
            if idx == 0:
                seg = seg + disc(center, HUB_R, pz0, pz1) + square_key(
                    center, KEY, pz1, pz1 + _KEY_HEIGHT
                )
                bodies[host].part = seg
            else:
                hardware.append(Body(f"crank{idx}", part=seg, color="orange", rigid_with=host))
        for p, xy in pin_xy.items():
            slots = [plan.slots[b] for b, q in crank.riders.items() if q == p]
            slots += [k for k, ps in plan.webs.items() if p in ps]
            flange = plan.pin_flanges.get(p)
            lo, hi = min(slots), max(slots)
            part = disc(xy, PIN_R, z(lo)[0], z(hi)[1])
            if flange is not None:
                part = part + disc(xy, FLANGE_R, *z(flange))
            hardware.append(Body(f"crankpin_{p}", part=part, color="gray", rigid_with=host))

    # -- pins, caps, sleeves --------------------------------------------------
    if joinery:
        for i, ax in enumerate(plan.axes):
            xy = axis_xy[i]
            members = sorted(ax.members, key=lambda n: plan.slots[n])
            member_slots = {plan.slots[n] for n in members}
            lo, hi = plan.span(ax)
            framed = ax.kind == "frame"
            top_slot = plan.plate + 1 if framed else hi + 1
            prefix = "frame_" if framed else ""
            head_host = frame if framed else members[0]
            cap_host = frame if framed else members[-1]
            hardware.append(Body(
                f"{prefix}pin_{ax.name}", part=pin(xy, z(lo - 1), z(top_slot)[1]),
                color="gray", rigid_with=head_host,
            ))
            hardware.append(Body(
                f"{prefix}cap_{ax.name}", part=cap(xy, *z(top_slot)),
                color="gray", rigid_with=cap_host,
            ))
            for k in range(lo + 1, hi):
                if k in member_slots or not plan.disc_fits(i, k, FLANGE_R, extra):
                    continue
                extra.setdefault(i, {})[k] = FLANGE_R
                hardware.append(Body(
                    f"{prefix}sleeve_{ax.name}_{k}", part=sleeve(xy, *z(k)),
                    color="gray", rigid_with=head_host,
                ))

    return Mechanism(
        name=mech.name,
        bodies=list(bodies.values()) + hardware,
        connections=list(mech.connections),
    )


__all__ = ["fabricate", "plan_for"]
