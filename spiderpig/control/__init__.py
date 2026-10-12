"""Driving a built arm (2026-10-12, phase 6): kinematics, steps, homing, calibration, and
the files and host code that run it on a 3D-printer controller board.

* :mod:`.arm`: the arm's forward and inverse kinematics in the frame its URDF's
  ``base_link`` has (:class:`.arm.ArmModel`, made from a design);
* :mod:`.mapping`: each joint's angle to its motor's steps, through its transmission's
  exact ratio (:class:`.mapping.Axis`);
* :mod:`.homing`: how each joint finds its zero (the turntable's optical flag; the planar
  inputs by an angle gauge on their links), then the snap to the motor's nearest full step;
* :mod:`.calibrate`: the link lengths and the joints' zeros fitted to measured poses;
* :mod:`.klipper`: a Klipper ``printer.cfg`` driving each joint as a ``manual_stepper``;
* :mod:`.host`: a Python driver that plans straight moves as synchronised joint segments
  and streams them to Klipper through Moonraker.

``spiderpig control`` is its command line (:mod:`.cli`).
"""
