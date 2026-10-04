"""Physics simulation of the fabricated walker with MuJoCo.

:mod:`sim.mjcf` builds the MJCF model (and the metadata the browser viewer
needs) from a :class:`fabricate.BuildConfig`; :mod:`sim.run` steps it under
crank-speed commands and measures how the robot walks; :mod:`sim.live` is the
session the server streams to the viewer over ``/ws/sim`` (commands in, binary
pose frames out), which is how the browser drives the model: nothing runs
MuJoCo in the browser itself.
"""
