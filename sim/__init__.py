"""Physics simulation of the fabricated walker with MuJoCo.

:mod:`sim.mjcf` builds the MJCF model (and the metadata the browser viewer
needs) from a :class:`fabricate.BuildConfig`; :mod:`sim.run` steps it under
crank-speed commands and measures how the robot walks. The same MJCF runs in
the browser through MuJoCo's WebAssembly bindings.
"""

from __future__ import annotations

from sim.mjcf import SimParams, build_mjcf, load_model
from sim.run import SimResult, kinematic_gait, kinematic_qpos, simulate, walk_metrics

__all__ = [
    "SimParams", "SimResult", "build_mjcf", "kinematic_gait", "kinematic_qpos", "load_model",
    "simulate", "walk_metrics",
]
