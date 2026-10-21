"""Physics simulation of a fabricated design with MuJoCo.

:mod:`sim.mjcf` builds the MJCF model of any design (and the metadata the browser viewer
needs) from a :class:`fabricate.BuildConfig`; :mod:`sim.run` steps a walker's under
crank-speed commands and measures how the robot walks, and holds any other design's at its
pose (:func:`sim.run.hold`); :mod:`sim.live` is the
session the server streams to the viewer over ``/ws/sim`` (commands in, binary
pose frames out), which is how the browser drives the model: nothing runs
MuJoCo in the browser itself.
"""
