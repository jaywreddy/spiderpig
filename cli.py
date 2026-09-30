"""One entry point for every tool: ``python cli.py <command> [options]``.

    uv run python cli.py build --module single --out build/single   # STEP/STL/DXF/BOM
    uv run python cli.py bake --linkage jansen --module double        # the viewer's .glb
    uv run python cli.py audit --linkage strider                      # does it go together?
    uv run python cli.py explain --linkage trotbot_heel               # each stage's verdict
    uv run python cli.py tune --module decker --grid 5                # crank phases
    uv run python cli.py sim --left 40rpm --right 40rpm               # MuJoCo
    uv run python cli.py report --out build/linkages.json             # every linkage

Each command is a module's ``main(argv)``; ``python cli.py <command> --help``
shows its options. The design and build options are the same everywhere
(:mod:`config`). ``mise run build|bake|audit|explain|tune|sim|report`` runs
these.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
# Tools that live beside the packages, not in them.
for sub in ("scripts", "viewer"):
    if str(ROOT / sub) not in sys.path:
        sys.path.append(str(ROOT / sub))

COMMANDS: dict[str, tuple[str, str]] = {      # command -> (module, one line)
    "build": ("main", "build the robot: STEP/STL, print STLs, DXF sheets, BOM"),
    "bake": ("bake_gltf", "bake the animated .glb for the viewer"),
    "audit": ("audit_fab", "fabrication audit: plan, contract, clashes, DXF, BOM"),
    "explain": ("explain", "what each pipeline stage says about a design"),
    "tune": ("tune_gait", "search crank phases (and proportions) for a smoother walk"),
    "sim": ("sim_walk", "simulate the walker in MuJoCo"),
    "report": ("linkage_report", "compare every registered linkage"),
}


def usage() -> str:
    width = max(len(c) for c in COMMANDS)
    lines = [f"usage: {Path(sys.argv[0]).name} <command> [options]", "", "commands:"]
    lines += [f"  {c:{width}}  {line}" for c, (_, line) in COMMANDS.items()]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(usage())
        return 0 if argv else 2
    command, rest = argv[0], argv[1:]
    if command not in COMMANDS:
        print(f"unknown command {command!r}\n\n{usage()}", file=sys.stderr)
        return 2
    module = importlib.import_module(COMMANDS[command][0])
    sys.argv[0] = f"{Path(sys.argv[0]).name} {command}"       # argparse's prog
    return int(module.main(rest) or 0)


if __name__ == "__main__":
    sys.exit(main())
