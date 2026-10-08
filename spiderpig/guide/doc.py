"""What the guide's PDF is laid out from (:mod:`guide.pdf`): plain data, every picture a
PNG file on disk (the renderer's output, or the image cache's)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class PartEntry:
    """One part type: its label (``P07``, ``C03``, ``H12``), what it is, how many."""

    label: str
    kind: str                     # "printed", "laser" or "purchased"
    name: str                     # "printed ring 9.3 x 9.3 x 3.0 mm", a catalog name
    qty: int
    thumb: Path
    file: str | None = None       # its print STL (printed), else None
    detail: str = ""              # a short line for the bag label: size, thickness


@dataclass
class Callout:
    """A part type a step uses, and how many."""

    label: str
    qty: int
    name: str
    thumb: Path


@dataclass
class StepEntry:
    number: int
    title: str
    stage: str                    # "Left side: leg stack", "Chassis", "Wiring"
    text: list[str]
    image: Path
    callouts: list[Callout] = field(default_factory=list)
    sub: bool = False             # a bench sub-assembly


@dataclass
class PrintBatch:
    """One print STL: its label, file, how many, the filament and grams each."""

    label: str
    file: str
    qty: int
    filament: str
    grams_each: float


@dataclass
class Doc:
    title: str
    subtitle: list[str]
    cover: Path
    parts: list[PartEntry]
    prints: list[PrintBatch]
    steps: list[StepEntry]
    footer: str
    supplies: list[str] = field(default_factory=list)   # shop supplies, no step's part
