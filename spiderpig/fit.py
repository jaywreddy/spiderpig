"""The dimensions every construction shares (:class:`Params`, a spec's ``fit``).

Apart from :mod:`construction.base`, which re-exports it, so that :mod:`config` (a
:class:`config.BuildConfig` holds one) validates a design without importing the
constructions and the CAD kernel under them.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Params:
    """Dimensions every construction shares (mm). Defaults suit FDM + 3 mm sheet."""

    margin: float = 1.0            # clearance between parts that move relative to each other
    # laser-cut plates
    link_radius: float = 6.0       # half-width of a leg link (pill radius)
    frame_radius: float = 7.0      # half-width of a frame plate arm
    min_wall: float = 1.5          # thinnest ring (or link) wall around a hole
    # fits (diametral clearances)
    running_fit: float = 0.35      # a part that turns in a laser-cut hole
    print_fit: float = 0.3         # two printed parts that slide together
    # axles (the printed axle's, removed on 2026-10-07; what nothing read since went on
    # 2026-10-07 too: config.REMOVED_PARAMS)
    axle_d: float = 6.0            # the diameter plates turn on
    spacer_d: float = 8.5          # shoulder beside a link (built-in spacer)
    # the crank (the printed crank's, removed on 2026-10-07; the bolt crank's are its own)
    crankpin_d: float = 6.0        # post b1 turns on
    web_radius: float = 6.0        # half-width of a crank web (O to crankpin)
    # the servo on the inner frame plate
    servo_screw_web_t: float = 1.0  # a front screw is left out when its hole would leave
    #                                less web than this many of the inner plate's thicknesses
    #                                to the horn's hole or a relief (the cut rules' error
    #                                level; the assembly audit of 2026-10-04: the STS3215's
    #                                near front holes leave 1.01 mm in 0.080 in); 0 keeps all

    def hole(self, d: float) -> float:
        """Finished hole diameter for a part of diameter ``d`` turning in it."""
        return d + self.running_fit
