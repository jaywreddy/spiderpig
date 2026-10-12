"""An arm's kinematics for control: forward and inverse, in its base's frame.

**The base frame** is the URDF's ``base_link`` (:mod:`spiderpig.urdf`): mm, its origin on
the base's floor under the turntable's axis, z up, x forward (the side's reach at yaw 0), y
to the left. One side alone (no turntable) has the side's own frame (x forward, z up, y
across the stack, the shoulder ``O`` at the origin).

**The tool's tip** is the output point less the tool's drop (:mod:`construction.tools`),
on the tool's axis across the stack; with no tool, the output point itself.

**Forward** (:meth:`ArmModel.fk`): the planar inputs place the output point
(:meth:`linkage.LegSolution.at`), the tool hangs under it, the turntable turns it all about
the vertical. **Inverse** (:meth:`ArmModel.ik`): the turntable's angle turns the target's
horizontal place onto the side's plane (the tip stands off that plane by the tool axis's
offset from the turntable's axis, so the angle is the target's bearing less that offset's
own), then the planar inputs are solved for the output point the tip needs: a coarse
search of the inputs' box for the start, then bounded least squares
(:func:`scipy.optimize.least_squares`) to :data:`TOL_MM`. A target the arm can't reach, or
reaches only outside a joint's range, raises :class:`IKError`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from spiderpig import linkage

TOL_MM = 1e-6       # the inverse kinematics' residual (mm)


class IKError(ValueError):
    """A target the arm can't reach (or only outside a joint's range)."""


@dataclass(frozen=True)
class ArmModel:
    """What control needs of a design (module docstring). ``yaw``: the turntable's axis
    in robot coordinates ``(x_a, y_ground, z_a)`` and its range (rad), ``None`` for a side
    alone; ``tool``: the tip's drop under the output point and its z (robot coordinates);
    ``ranges``: the planar inputs' (rad)."""

    linkage: str
    values: tuple[float, ...]
    output: str
    inputs: tuple[str, ...]
    ranges: dict[str, tuple[float, float]]
    yaw: tuple[tuple[float, float, float], tuple[float, float]] | None = None
    tool: tuple[float, float] = (0.0, 0.0)
    _grid: dict = field(default_factory=dict, compare=False, repr=False)

    @classmethod
    def from_config(cls, config, design=None) -> ArmModel:
        """The model of ``config``'s arm (its turret and tool when it builds the robot).
        ``design``: the side's design, when the caller holds it (else designed here)."""
        from spiderpig.fabricate import design_side, template_for

        lk = config.lk
        if lk.kind != "arm" or lk.output is None:
            raise ValueError(f"{lk.key} is a {lk.kind}: control drives an arm's inputs")
        side = config.side_config()
        if design is None:
            design = design_side(template_for(side), side)
        ranges = {q: (math.radians(lo), math.radians(hi)) for q, (lo, hi) in lk.ranges.items()}
        yaw, tool = None, (0.0, 0.0)
        if config.layout == "turret":
            from spiderpig.construction.robot import Turret

            tp, tplan = Turret().plans(design, config)
            yaw = ((tp.x_a, tp.y_ground, tp.z_a), (math.radians(tp.range_deg[0]),
                                                   math.radians(tp.range_deg[1])))
            if tplan is not None:
                from spiderpig.construction.tools import axis_z

                plan = design.plan
                lo, hi = plan.z(plan.layers[tplan.body])
                tool = (tplan.tool.drop(),
                        axis_z(tplan, lo, hi, plan.z(0)[1], plan.z(plan.top)[0]))
        return cls(lk.key, config.values, lk.output.point, tuple(lk.inputs), ranges, yaw, tool)

    # -- frames --------------------------------------------------------------------------

    def to_base(self, p) -> np.ndarray:
        """A robot point (x, y, z: the side's) in the base frame."""
        p = np.asarray(p, dtype=float)
        if self.yaw is None:
            return np.array([p[0], p[2], p[1]])
        (xa, yg, za), _ = self.yaw
        return np.array([p[0] - xa, -(p[2] - za), p[1] - yg])

    def solution(self) -> linkage.LegSolution:
        return linkage.LegSolution(1, 0.0, self.values, self.linkage)

    def output_at(self, q: dict[str, float]) -> np.ndarray:
        """The output point (the side's plane, mm) at the planar inputs ``q`` (rad)."""
        pts = self.solution().at(*(np.atleast_1d(q[n]) for n in self.inputs))
        return np.asarray(pts[self.output], dtype=float).reshape(-1, 2)[0]

    # -- forward -------------------------------------------------------------------------

    def fk(self, q: dict[str, float]) -> np.ndarray:
        """The tool's tip in the base frame (mm) at the joints ``q`` (rad: the planar
        inputs' design angles and the turntable's ``q1`` from its zero)."""
        f = self.output_at(q)
        drop, z = self.tool
        tip = self.to_base((f[0], f[1] - drop, z))
        psi = float(q.get("q1", 0.0)) if self.yaw is not None else 0.0
        c, s = math.cos(psi), math.sin(psi)
        return np.array([c * tip[0] - s * tip[1], s * tip[0] + c * tip[1], tip[2]])

    # -- inverse -------------------------------------------------------------------------

    def _start(self, target: np.ndarray) -> np.ndarray:
        """The box's sampled inputs nearest the output point ``target`` (a start)."""
        if "pts" not in self._grid:
            n = 41
            axes = [np.linspace(*self.ranges[q], n) for q in self.inputs]
            mesh = np.meshgrid(*axes, indexing="ij")
            flat = [m.ravel() for m in mesh]
            pts = self.solution().at(*flat)[self.output].reshape(-1, 2)
            self._grid["pts"], self._grid["q"] = pts, np.stack(flat, axis=1)
        d = np.linalg.norm(self._grid["pts"] - target, axis=1)
        return self._grid["q"][int(np.nanargmin(d))]

    def ik(self, xyz) -> dict[str, float]:
        """The joints (rad) that put the tool's tip at ``xyz`` (base frame, mm)
        (module docstring); :class:`IKError` when none does."""
        from scipy.optimize import least_squares

        x, y, z = (float(v) for v in xyz)
        drop, ztool = self.tool
        q: dict[str, float] = {}
        if self.yaw is not None:
            (xa, yg, za), (ylo, yhi) = self.yaw
            off = -(ztool - za)                    # the tip's y in the base at yaw 0
            rho = math.hypot(x, y)
            if rho < abs(off) + 1e-9:
                raise IKError(f"({x:.1f}, {y:.1f}) is inside the tool's offset from the axis")
            reach = math.sqrt(rho * rho - off * off)   # the tip's x in the base at yaw 0
            psi = math.atan2(y, x) - math.atan2(off, reach)
            psi = (psi + math.pi) % (2 * math.pi) - math.pi
            if not ylo - 1e-9 <= psi <= yhi + 1e-9:
                raise IKError(f"the turntable would turn {math.degrees(psi):.1f} deg, outside "
                              f"{math.degrees(ylo):.0f}..{math.degrees(yhi):.0f}")
            q["q1"] = psi
            target = np.array([reach + xa, z + yg + drop])
        else:
            target = np.array([x, z + drop])
        lo = np.array([self.ranges[n][0] for n in self.inputs])
        hi = np.array([self.ranges[n][1] for n in self.inputs])

        def resid(v):
            with np.errstate(all="ignore"):
                f = self.output_at(dict(zip(self.inputs, v, strict=True)))
            return np.nan_to_num(f - target, nan=1e6)

        sol = least_squares(resid, self._start(target), bounds=(lo, hi), xtol=1e-14,
                            ftol=1e-14, gtol=1e-14)
        err = float(np.linalg.norm(resid(sol.x)))
        if err > TOL_MM:
            raise IKError(f"({x:.1f}, {y:.1f}, {z:.1f}) is out of reach: the nearest the "
                          f"inputs' box gets is {err:.2f} mm off")
        q.update({n: float(v) for n, v in zip(self.inputs, sol.x, strict=True)})
        return q

