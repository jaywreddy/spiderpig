"""Measured numbers rounded for reports, the same however OCCT got to them.

A measured distance can sit on a rounding tie (``R.b4_leg0``'s hole is 3.925 mm from the
klann quad's link edge): BRepExtrema returns 3.92499999... or 3.92500000...1 depending on
OCCT's thread count, and ``round(x, 2)`` then says 3.92 or 3.93. :func:`rounded` first
settles the value :data:`SETTLE` digits past the ones it keeps (3.925000 for two: far
under any tolerance a report reads), then rounds half away from zero, so both say 3.93.
"""

from __future__ import annotations

from decimal import ROUND_HALF_UP, Decimal

SETTLE = 4
"""Digits past those kept a measurement is settled to before it is rounded for a report
(a 2-digit report settles to 1e-6; a 5-digit one to 1e-9, so 4.6e-6 still reads 0)."""


def rounded(x: float, digits: int) -> float:
    """``x`` to ``digits`` decimals, ties (within ``10**-(digits + SETTLE)``) half away
    from zero."""
    x = float(x)
    if digits + SETTLE > 15 or x != x or x in (float("inf"), float("-inf")):
        return round(x, digits)
    settled = Decimal(repr(round(x, digits + SETTLE)))
    return float(settled.quantize(Decimal(1).scaleb(-digits), rounding=ROUND_HALF_UP)) + 0.0


def fixed(x: float, digits: int) -> str:
    """``f"{x:.{digits}f}"`` of :func:`rounded`: the text a report prints for ``x``."""
    return f"{rounded(x, digits):.{digits}f}"
