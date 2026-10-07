"""Measured numbers rounded for reports, the same however OCCT got to them.

A measured distance can sit on a rounding tie (``R.b4_leg0``'s hole is 3.925 mm from the
klann quad's link edge): BRepExtrema returns 3.92499999... or 3.92500000...1 depending on
OCCT's thread count, and ``round(x, 2)`` then says 3.92 or 3.93. :func:`rounded` first
rounds to 1e-6 (far under any tolerance a report reads), then half away from zero, so
both say 3.93.
"""

from __future__ import annotations

from decimal import ROUND_HALF_UP, Decimal

SETTLE = 6
"""Digits a measurement is settled to before it is rounded for a report."""


def rounded(x: float, digits: int) -> float:
    """``x`` to ``digits`` decimals, ties (within 1e-6) half away from zero."""
    x = float(x)
    if digits >= SETTLE or x != x or x in (float("inf"), float("-inf")):
        return round(x, digits)
    settled = Decimal(repr(round(x, SETTLE)))
    return float(settled.quantize(Decimal(1).scaleb(-digits), rounding=ROUND_HALF_UP)) + 0.0


def fixed(x: float, digits: int) -> str:
    """``f"{x:.{digits}f}"`` of :func:`rounded`: the text a report prints for ``x``."""
    return f"{rounded(x, digits):.{digits}f}"
