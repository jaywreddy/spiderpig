"""Bake Klann linkage viewer data: per-body STLs + per-frame poses.

Writes into ``viewer/data/`` by default. The frontend at ``viewer/main.js``
consumes these directly (no runtime geometry generation in the browser).

Usage
-----
    uv run python viewer/bake.py
    uv run python viewer/bake.py --frames 120 --fps 60 --out viewer/data/
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# viewer/ sits alongside klann.py / shapes.py / mechanism.py. Make them
# importable even when this script is invoked as ``viewer/bake.py``.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from build123d import export_stl  # noqa: E402

from klann import KlannLinkage  # noqa: E402
from poses import bake_frames  # noqa: E402


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Bake viewer data for the Klann mechanism.")
    p.add_argument(
        "--out",
        type=Path,
        default=_REPO_ROOT / "viewer" / "data",
        help="Output directory (parts/ + frames.json). Default: viewer/data",
    )
    p.add_argument("--frames", type=int, default=60, help="Animation frame count.")
    p.add_argument("--fps", type=int, default=30, help="Animation playback FPS.")
    return p.parse_args()


def bake(
    out: Path,
    n_frames: int = 60,
    fps: int = 30,
    *,
    parts: bool = True,
    verbose: bool = False,
) -> None:
    """Run the bake: writes ``out/parts/*.stl`` and ``out/frames.json``.

    Set ``parts=False`` to skip STL regeneration (fast path when only
    phase/pose math changed — geometry is unaffected).
    """
    out.mkdir(parents=True, exist_ok=True)
    log = print if verbose else (lambda *_a, **_k: None)

    if parts:
        parts_dir = out / "parts"
        parts_dir.mkdir(parents=True, exist_ok=True)
        mech = KlannLinkage()
        for body in mech.bodies:
            if body.part is None:
                continue
            path = parts_dir / f"{body.name}.stl"
            export_stl(body.part, str(path))
            log(f"wrote {path} ({path.stat().st_size} B)")

    data = bake_frames(n_frames=n_frames, fps=fps)
    frames_path = out / "frames.json"
    frames_path.write_text(json.dumps(data))
    log(f"wrote {frames_path} ({frames_path.stat().st_size} B, {n_frames} frames)")


def main() -> None:
    args = _parse_args()
    bake(args.out, n_frames=args.frames, fps=args.fps, verbose=True)


if __name__ == "__main__":
    main()
