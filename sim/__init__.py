"""Physics simulation of the fabricated walker with MuJoCo.

:mod:`sim.mjcf` builds the MJCF model (and the metadata the browser viewer
needs) from a :class:`fabricate.BuildConfig`; :mod:`sim.run` steps it under
crank-speed commands and measures how the robot walks. The same MJCF runs in
the browser through MuJoCo's WebAssembly bindings.
"""
