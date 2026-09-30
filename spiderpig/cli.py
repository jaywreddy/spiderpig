"""One entry point for every tool: ``spiderpig <command> [options]`` (the console script;
``python -m spiderpig.cli`` from a checkout).

    spiderpig build --module single --out build/single   # STEP/STL/DXF/BOM
    spiderpig bake --linkage jansen --module double        # the viewer's .glb
    spiderpig audit --linkage strider                      # does it go together?
    spiderpig explain --linkage trotbot_heel               # each stage's verdict
    spiderpig tune --module decker --grid 5                # crank phases
    spiderpig sim --left 40rpm --right 40rpm               # MuJoCo
    spiderpig report --out build/linkages.json             # every linkage
    spiderpig mcp --store .spiderpig                       # MCP server (stdio)
    spiderpig view 1a2b3c4d5e6f7a8b --open                 # the viewer for a stored design

Each command is a module's ``main(argv)``; ``spiderpig <command> --help`` shows its
options. The design and build options are the same everywhere
(:mod:`spiderpig.config`). ``mise run build|bake|audit|explain|tune|sim|report`` runs
these from a checkout. Nothing here imports the engine until a command runs, so
``--help`` is instant.
"""

from __future__ import annotations

import importlib
import sys

COMMANDS: dict[str, tuple[str, str]] = {      # command -> (module, one line)
    "build": ("spiderpig.build", "build the robot: STEP/STL, print STLs, DXF sheets, BOM"),
    "bake": ("spiderpig.bake", "bake the animated .glb for the viewer"),
    "audit": ("spiderpig.tools.audit", "fabrication audit: plan, contract, clashes, DXF, BOM"),
    "explain": ("spiderpig.explain", "what each pipeline stage says about a design"),
    "tune": ("spiderpig.tools.tune",
             "search crank phases (and proportions) for a smoother walk"),
    "sim": ("spiderpig.tools.sim_walk", "simulate the walker in MuJoCo"),
    "report": ("spiderpig.tools.report", "compare every registered linkage"),
    "mcp": ("spiderpig.mcp", "serve the agent-facing API over MCP (stdio; --store PATH)"),
    "view": ("spiderpig.view",
             "serve the viewer for a stored design, no Node needed (prints the URL; --open)"),
}


def usage() -> str:
    width = max(len(c) for c in COMMANDS)
    lines = ["usage: spiderpig <command> [options]", "", "commands:"]
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
    sys.argv[0] = f"spiderpig {command}"       # argparse's prog
    return int(module.main(rest) or 0)


if __name__ == "__main__":
    sys.exit(main())
