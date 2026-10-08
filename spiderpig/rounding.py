"""Measured numbers rounded for reports, the same however OCCT got to them.

A measured distance can sit on a rounding tie (the demo Klann quad's ``b4`` links have a
hole 3.925 mm from their edge): BRepExtrema returns 3.92499999... or 3.92500000...1 with
OCCT's thread count (and on a mirrored twin), and ``round(x, 2)`` then says 3.92 or 3.93.
:func:`rounded` first settles the value :data:`SETTLE` digits past the ones it keeps (the
float nearest 3.925000), then rounds that as ``round`` does, so both say what ``round``
says of the tie's own float (3.925 is 3.92499999999999982..., so 3.92). A value off a tie
reads as ``round`` reads it, and so does a design's exact tie (an 8.85 mm bore: 8.8).
"""

from __future__ import annotations

SETTLE = 4
"""Digits past those kept a measurement is settled to before it is rounded for a report
(a 2-digit report settles to 1e-6; a 5-digit one to 1e-9, so 4.6e-6 still reads 0)."""


def rounded(x: float, digits: int) -> float:
    """``round(x, digits)`` of ``x`` settled to ``digits + SETTLE`` decimals: the same on
    either side of a tie within ``10**-(digits + SETTLE)``."""
    x = float(x)
    if digits + SETTLE > 15:
        return round(x, digits)
    return round(round(x, digits + SETTLE), digits) + 0.0


def fixed(x: float, digits: int) -> str:
    """``f"{x:.{digits}f}"`` of :func:`rounded`: the text a report prints for ``x``."""
    return f"{rounded(x, digits):.{digits}f}"
