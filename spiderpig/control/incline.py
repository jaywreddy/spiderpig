"""Reading a link's angle from a Murata SCL3300-D01 inclinometer on it (2026-10-12, round 3
of the arm's precision review; :mod:`actuators.sensors` ``scl3300``).

The sensor sits flat on the link, its x along the link (towards its far end), its y in the
link's plane, square to x: at rest it reads gravity's reaction, so a link at ``theta`` from
horizontal reads ``(g sin theta, g cos theta)`` and its angle is ``atan2(x, y)`` (the scale
cancels, any of its modes), less the offset found once at assembly against the gauge
(:func:`offset_from_gauge`). Averaged ``seconds`` at rest (its noise 0.001 deg/sqrt(Hz)),
what homing reads (:func:`homing.set_from_gauge` with this reading) and what the host reads
before each final approach, to correct the drive's sag.

**The SPI frame** (the SCL3300 datasheet, "SPI frame"; the codes are the datasheet's and
are unverified on hardware here): 32 bits, MSB first, mode 0, up to 8 MHz: an 8-bit
operation code (read/write and address, with the bank's bits), 16 bits of data and an
8-bit CRC (polynomial 0x1D, seed 0xFF, the result inverted) over the first 24. A request's
answer comes back in the *next* frame (off-frame protocol), with its return status in the
answer's top two bits (``RS``: 01 normal; 11 error; 00 starting up). Start-up: a software
reset, mode 4 (the inclination mode, 10 Hz low-pass), wait 25 ms, read the status twice.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

READ_ACC_X = 0x040000F7
READ_ACC_Y = 0x080000FD
READ_ACC_Z = 0x0C0000FB
READ_STATUS = 0x180000E5
READ_WHOAMI = 0x40000091
SW_RESET = 0xB4002098
MODE_4 = 0xB4000338             # inclination mode: 12000 LSB/g, 10 Hz low-pass
ENABLE_ANGLES = 0xB0001F6F
WHOAMI = 0xC1                   # what the WHOAMI register holds


def crc8(data24: int) -> int:
    """The SCL3300's CRC over a frame's first 24 bits (polynomial 0x1D, seed 0xFF,
    inverted)."""
    crc = 0xFF
    for i in range(23, -1, -1):
        bit = (data24 >> i) & 1
        top = (crc >> 7) & 1
        crc = (crc << 1) & 0xFF
        if bit ^ top:
            crc ^= 0x1D
    return (~crc) & 0xFF


def frame(op: int, data: int = 0) -> int:
    """A request frame: ``op`` (8 bits), ``data`` (16), its CRC."""
    body = ((op & 0xFF) << 16) | (data & 0xFFFF)
    return (body << 8) | crc8(body)


class FrameError(ValueError):
    """An answer frame with a bad CRC or an error status."""


@dataclass(frozen=True)
class Answer:
    status: int        # RS: 1 normal, 3 error, 0 starting up
    op: int            # the operation it answers (without RS)
    data: int          # 16 bits

    @property
    def signed(self) -> int:
        return self.data - 0x10000 if self.data & 0x8000 else self.data


def decode(word: int) -> Answer:
    """An answer frame (32 bits), its CRC checked."""
    body, crc = word >> 8, word & 0xFF
    if crc8(body) != crc:
        raise FrameError(f"bad CRC in {word:#010x}")
    return Answer((body >> 22) & 0x3, (body >> 16) & 0x3F, body & 0xFFFF)


def angle_deg(acc_x: float, acc_y: float) -> float:
    """The link's angle from horizontal (degrees) from the sensor's x (along the link) and
    y (square to it in the link's plane) at rest."""
    return math.degrees(math.atan2(acc_x, acc_y))


def offset_from_gauge(sensor_deg: float, gauge_deg: float) -> float:
    """The mounting offset (degrees) to subtract from the sensor's angle: found once at
    assembly with the link where the gauge reads ``gauge_deg``."""
    return sensor_deg - gauge_deg


Xfer = Callable[[int], int]
"""One full-duplex 32-bit exchange on the sensor's chip select (spidev's ``xfer2`` over four
bytes, wrapped)."""


class SCL3300:
    """One sensor on an SPI bus (``xfer``: :data:`Xfer`)."""

    def __init__(self, xfer: Xfer, offset_deg: float = 0.0):
        self.xfer = xfer
        self.offset_deg = offset_deg

    def _request(self, word: int) -> Answer:
        return decode(self.xfer(word))

    def start(self, sleep: Callable[[float], None] = time.sleep) -> None:
        """Reset, inclination mode, the angles on; then the status read twice (the first
        answer is stale) and the WHOAMI checked."""
        self.xfer(SW_RESET)
        sleep(0.001)
        self.xfer(MODE_4)
        self.xfer(ENABLE_ANGLES)
        sleep(0.025)
        for _ in range(2):
            self.xfer(READ_STATUS)
        self.xfer(READ_WHOAMI)
        who = self._request(READ_WHOAMI)
        if who.data & 0xFF != WHOAMI:
            raise FrameError(f"WHOAMI {who.data:#06x}, not an SCL3300 ({WHOAMI:#04x})")

    def accel(self) -> tuple[int, int]:
        """One (x, y) reading, raw counts (off-frame: each request's answer comes with the
        next)."""
        self.xfer(READ_ACC_X)
        x = self._request(READ_ACC_Y)
        y = self._request(READ_STATUS)
        for a in (x, y):
            if a.status == 3:
                raise FrameError("the sensor reports an error (read STATUS)")
        return x.signed, y.signed

    def angle(self, seconds: float = 0.5, rate_hz: float = 100.0,
              sleep: Callable[[float], None] = time.sleep) -> float:
        """The link's angle (degrees, its offset taken off), the mean of ``seconds`` of
        readings at ``rate_hz`` (the arm at rest)."""
        n = max(1, round(seconds * rate_hz))
        sx = sy = 0.0
        for _ in range(n):
            x, y = self.accel()
            sx, sy = sx + x, sy + y
            sleep(1.0 / rate_hz)
        return angle_deg(sx, sy) - self.offset_deg


def spidev_xfer(bus: int = 0, device: int = 0, hz: int = 2_000_000) -> Xfer:
    """:data:`Xfer` over Linux's spidev (the ``spidev`` package, on the host)."""
    import spidev  # type: ignore[import-not-found]

    spi = spidev.SpiDev()
    spi.open(bus, device)
    spi.mode, spi.max_speed_hz = 0, hz

    def xfer(word: int) -> int:
        b = spi.xfer2(list(word.to_bytes(4, "big")))
        return int.from_bytes(bytes(b), "big")
    return xfer
