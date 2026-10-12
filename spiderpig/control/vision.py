"""Homing to a reference sheet by camera: the joints' zeros from fiducials (2026-10-12,
the user's: "vision based homing to a fiducial target with a reference sheet").

The switches (:mod:`.homing`) find each joint coarsely: any slot switch will do. Then the
camera on the tool, looking straight down, sights dots printed on a flat reference sheet
on the table, each at a known place in the base frame (:mod:`.arm`). At each fiducial the
arm is sent to stand the camera over it (:meth:`arm.ArmModel.ik` of the dot's place, the
camera's offset from the tip taken off), the image is taken at rest, and the dot's centre
(:func:`dot_centroid`: an intensity-weighted centroid of the dark dot, sub-pixel) less
the image's centre, times the scale, is where the tip really is against where the model
put it (:class:`Sighting`). Two sightings or more fix all three zeros at once
(:func:`zero_offsets`: least squares of the tip's horizontal place over the offsets of
``q1``, ``q2``, ``q3``); more spread over the sheet average out the dots' print error.

What it fixes is what a switch can't: every homing's zero to the sheet, not to a flag,
and with it whatever has moved since (the links' sag and creep, the frame's heat; a
capstan's pitch radius growing with its motor's heat is a gain, not an offset:
``fit_gain`` fits one per joint from sightings spread over its range), to the camera's
measurement (about :data:`VISION_REPEAT_MM` at the tip: a Raspberry Pi Camera Module 3
*Wide* focuses from 5 cm, so about 50 mm over the sheet (the standard module only from
10 cm, about 28 um a pixel there), its lens locked; a dot's centroid repeats to a tenth of
a pixel; focus, lighting and the mount's heat budgeted on top). The sheet is an FR-4
printed circuit board (copper dots, its etch the dots' places) screwed to the base plate:
paper swells tens of um with the air's humidity from day to day. Its own accuracy is the
arm's accuracy, not its repeatability: a chrome-on-glass target, or the dots' places
measured once, makes it the arm's too. After a coarse homing each axis snaps to its
driver's phase (:mod:`.homing`), the camera then catching a capstan's slip (a whole
cycle's jump).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

VISION_REPEAT_MM = 0.01    # the tip's place from one sighting (an estimate, unverified)


def dot_centroid(image: np.ndarray, threshold: float | None = None) -> tuple[float, float]:
    """The centre (x, y pixels, sub-pixel) of the dark dot in a grey ``image`` (2-D, light
    background): the centroid of how far each pixel is under ``threshold`` (default:
    halfway between the image's 5th and 95th percentiles)."""
    img = np.asarray(image, dtype=float)
    if img.ndim != 2:
        raise ValueError("a grey image, 2-D")
    if threshold is None:
        lo, hi = np.percentile(img, [5, 95])
        threshold = (lo + hi) / 2
    w = np.clip(threshold - img, 0.0, None)
    total = float(w.sum())
    if total <= 0:
        raise ValueError("no dot under the threshold")
    ys, xs = np.indices(img.shape)
    return float((w * xs).sum() / total), float((w * ys).sum() / total)


@dataclass(frozen=True)
class Sighting:
    """One fiducial seen: the joints commanded ``q`` (rad), the dot's known place on the
    sheet ``dot_xy`` (base frame, mm), and where the camera saw it from its centre
    ``seen_mm`` (the image's x, y scaled to mm and turned into the base frame's x, y)."""

    q: dict[str, float]
    dot_xy: tuple[float, float]
    seen_mm: tuple[float, float]

    @property
    def tip_xy(self) -> np.ndarray:
        """Where the tip really was: the dot less where it appeared from the camera's
        centre (the camera's offset from the tip calibrated out)."""
        return np.asarray(self.dot_xy, dtype=float) - np.asarray(self.seen_mm, dtype=float)


def image_offset_mm(image: np.ndarray, mm_per_px: float, yaw: float = 0.0,
                    rotation: float = 0.0) -> tuple[float, float]:
    """The dot's offset from the image's centre in the base frame (mm): the centroid's
    pixels times ``mm_per_px``, turned by the camera's mounting ``rotation`` and the
    turntable's ``yaw`` (rad; the camera turns with it)."""
    cx, cy = dot_centroid(image)
    h, w = np.asarray(image).shape
    dx, dy = (cx - (w - 1) / 2) * mm_per_px, -(cy - (h - 1) / 2) * mm_per_px
    a = yaw + rotation
    c, s = math.cos(a), math.sin(a)
    return c * dx - s * dy, s * dx + c * dy


def zero_offsets(model, sightings: list[Sighting],
                 joints: tuple[str, ...] = ("q1", "q2", "q3"),
                 fit_gain: bool = False) -> dict[str, float]:
    """The joints' zero offsets (rad: the true angle is the commanded one plus it) that put
    the model's tip where the sightings saw it (least squares of the tips' horizontal
    places). ``model``: an :class:`arm.ArmModel` (``fk``). ``fit_gain``: each joint's gain
    too (``<joint>_gain``: the true angle ``q (1 + gain) + offset``: a capstan's ratio
    drifting with its heat), from sightings spread over each joint's range."""
    from scipy.optimize import least_squares

    names = [j for j in joints if (j == "q1" and model.yaw is not None) or j != "q1"]
    unknowns = len(names) * (2 if fit_gain else 1)
    if 2 * len(sightings) < unknowns:
        raise ValueError(f"{unknowns} unknowns need {math.ceil(unknowns / 2)} sightings "
                         f"or more, got {len(sightings)}")

    def resid(x: np.ndarray) -> np.ndarray:
        out = []
        for s in sightings:
            q = dict(s.q)
            for i, n in enumerate(names):
                gain = x[len(names) + i] if fit_gain else 0.0
                q[n] = q.get(n, 0.0) * (1 + gain) + x[i]
            out.extend(model.fk(q)[:2] - s.tip_xy)
        return np.asarray(out)

    r = least_squares(resid, np.zeros(unknowns), x_scale=1e-3)
    out = dict(zip(names, (float(v) for v in r.x[:len(names)]), strict=True))
    if fit_gain:
        out.update({f"{n}_gain": float(v)
                    for n, v in zip(names, r.x[len(names):], strict=True)})
    return out
