"""Independent verification of a plan on a fresh, denser sampling (:func:`verify_plan`)."""


from __future__ import annotations

import itertools
from typing import TYPE_CHECKING

import numpy as np

from spiderpig.stack.geometry import Geometry, made
from spiderpig.stack.plan_z import settle
from spiderpig.stack.topology import topology_from_template

if TYPE_CHECKING:
    from spiderpig.stack.geometry import Placed
    from spiderpig.stack.plan import StackPlan


# ---------------------------------------------------------------------------
# Independent verification
# ---------------------------------------------------------------------------


def verify_plan(plan: StackPlan, tmpl=None, samples: int = 2880, tol: float = 1e-6) -> list[str]:
    """Re-check a plan from scratch: every claim re-evaluated at the plan's z, the heads it
    sank sunk again, every pair in one layer or one clearance gap tested, every gap and
    sunk head checked for height.

    With ``tmpl`` the geometry is re-sampled (denser than the solver's), so
    the check doesn't reuse any solver table. Returns human-readable
    violations (empty when valid).
    """
    topo = topology_from_template(tmpl, samples) if tmpl is not None else plan.topo
    if tmpl is not None:
        # fixed points a group added to the plan's geometry (e.g. the servo's mounting
        # screws) aren't joints of the template: carry them over, and put the points fixed
        # to the crank back on it
        fixed = {k: v[0] for k, v in plan.topo.geometry.points.items()
                 if k not in topo.geometry.points and k not in plan.topo.crank_points
                 and not np.ptp(v, axis=0).any()}
        if fixed:
            topo.geometry = Geometry({**topo.geometry.points, **fixed})
        for name, (r, ang) in plan.topo.crank_points.items():
            topo.add_crank_point(name, r, ang)
    geo, sp, layout = topo.geometry, plan.spec, plan.layout
    bad: list[str] = []
    made_: list[Placed] = []
    for c in plan.claims:
        out, why = made(c, layout)
        if out is None:
            bad.append(why)
            continue
        made_.extend(out)
    shapes = settle(made_, plan.sunk, layout)
    for n in topo.links:
        if n not in plan.layers:
            bad.append(f"{n} has no layer")
        elif not 0 < plan.layers[n] < plan.top:
            bad.append(f"{n} sits in layer {plan.layers[n]}, outside the frame plates")
    for p in shapes:
        if not p.seat and not p.gap and p.layer in (0, plan.top):
            bad.append(f"{p.label or p.group} sits in frame-plate layer {p.layer}")
        if p.height > 0:
            room = layout.gap(p.layer) if p.gap else layout.t(p.layer)
            if p.height > room + tol:
                where = f"the {room:g} mm gap over layer {p.layer}" if p.gap else \
                    f"layer {p.layer} ({room:g} mm)"
                bad.append(f"{p.label or p.group} needs {p.height:.2f} mm, more than {where}")
    by_slot: dict[float, list[Placed]] = {}
    for p in shapes:
        if not p.seat:
            by_slot.setdefault(p.slot, []).append(p)
    for slot, live in sorted(by_slot.items()):
        for a, b in itertools.combinations(live, 2):
            if a.group == b.group:
                continue
            need = a.shape.r + b.shape.r + sp.margin
            d = geo.dist(a.shape.core, b.shape.core)
            if d < need - tol:
                where = f"gap over layer {int(slot)}" if slot != int(slot) else f"layer {slot}"
                bad.append(f"{where}: {a.label or a.group} x {b.label or b.group} "
                           f"clear {d - a.shape.r - b.shape.r:.2f} mm (need {sp.margin:.2f})")
    return bad
