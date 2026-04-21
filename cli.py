"""Cross-platform task runner for the spiderpig project.

Thin entry point that dispatches to the right Python module for each verb.
No shell required — works identically on Windows, macOS, and Linux.

Usage
-----
    uv run python cli.py view        # dev server w/ live reload
    uv run python cli.py build       # STEP/STL/DXF -> build/
    uv run python cli.py bake        # viewer data -> viewer/data/
    uv run python cli.py test
    uv run python cli.py clean
"""

from __future__ import annotations

import argparse
import shutil
import sys
import webbrowser
from pathlib import Path
from threading import Timer

_REPO_ROOT = Path(__file__).resolve().parent


def _cmd_build(args: argparse.Namespace) -> int:
    import main  # noqa: PLC0415

    sys.argv = ["main.py", *args.rest]
    main.main()
    return 0


def _cmd_bake(args: argparse.Namespace) -> int:
    sys.path.insert(0, str(_REPO_ROOT / "viewer"))
    import bake  # noqa: PLC0415

    sys.argv = ["bake.py", *args.rest]
    bake.main()
    return 0


def _cmd_view(args: argparse.Namespace) -> int:
    import uvicorn  # noqa: PLC0415

    host, port = args.host, args.port
    url = f"http://{host}:{port}"
    if not args.no_browser:
        Timer(0.8, lambda: webbrowser.open(url)).start()
    print(f"→ {url}  (Ctrl+C to stop)")
    uvicorn.run("server.app:app", host=host, port=port, log_level="info")
    return 0


def _cmd_test(args: argparse.Namespace) -> int:
    import pytest  # noqa: PLC0415

    return pytest.main(args.rest or ["-v"])


def _cmd_clean(_args: argparse.Namespace) -> int:
    for target in ("build", "viewer/data"):
        path = _REPO_ROOT / target
        if path.exists():
            shutil.rmtree(path)
            print(f"removed {path}")
        else:
            print(f"(skip) {path} absent")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="spiderpig", description=__doc__.strip().splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("build", help="Generate STEP/STL/DXF in build/").add_argument(
        "rest", nargs=argparse.REMAINDER, help="Args forwarded to main.py"
    )
    sub.add_parser("bake", help="Bake viewer data into viewer/data/").add_argument(
        "rest", nargs=argparse.REMAINDER, help="Args forwarded to viewer/bake.py"
    )

    pv = sub.add_parser("view", help="Start dev server + live-reload viewer")
    pv.add_argument("--host", default="127.0.0.1")
    pv.add_argument("--port", type=int, default=8000)
    pv.add_argument("--no-browser", action="store_true", help="Don't auto-open the browser")

    sub.add_parser("test", help="Run pytest").add_argument(
        "rest", nargs=argparse.REMAINDER, help="Args forwarded to pytest"
    )
    sub.add_parser("clean", help="Remove build/ and viewer/data/")

    return p


def main() -> int:
    args = _build_parser().parse_args()
    return {
        "build": _cmd_build,
        "bake": _cmd_bake,
        "view": _cmd_view,
        "test": _cmd_test,
        "clean": _cmd_clean,
    }[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
