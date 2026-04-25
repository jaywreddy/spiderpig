"""Stop dev-server processes (Vite, uvicorn, anything launched by ``mise run view``).

Two modes:

- ``--mine`` (default): kill processes whose command line references *this*
  worktree's absolute path AND whose process name is a known dev-server
  binary (``node``, ``python``, ``uvicorn``, ``npm``). Safe for
  parallel-worktree dev — touches only this checkout's servers, and the
  process-name allow-list keeps editors/agents/terminals open in the
  worktree from getting killed.
- ``--port N``: kill whoever is currently listening on TCP port N
  (regardless of which worktree they belong to). Use to recover :5173 /
  :8000 from an orphaned process you don't own.

Cross-platform via stdlib subprocess: PowerShell on Windows, lsof on
macOS/Linux. No psutil dependency.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# Allow-list of process names that can be killed by ``--mine``. Process
# matching is case-insensitive and the ``.exe`` suffix is stripped before
# compare, so ``node.exe`` matches ``node``. Keep this list tight: it's the
# only thing standing between ``mise run kill`` and your editor.
DEV_PROCESS_NAMES = {"node", "python", "pythonw", "uvicorn", "npm"}


def _log(msg: str) -> None:
    print(f"[kill] {msg}", flush=True)


def _normalize_proc_name(name: str) -> str:
    return re.sub(r"\.exe$", "", name.strip(), flags=re.IGNORECASE).lower()


def _kill_by_path_windows(path: Path) -> int:
    """Kill dev-server processes whose CommandLine contains ``path``.

    Filters by name allow-list (``DEV_PROCESS_NAMES``) so processes like
    Code.exe / claude.exe / pwsh.exe that happen to have the worktree path
    in their command line are NOT killed. Excludes the current Python
    process and its parent (mise) so we don't self-terminate.
    """
    p = str(path)
    name_filter = ",".join(f"'{n}'" for n in sorted(DEV_PROCESS_NAMES))
    script = (
        f"$path = '{p.replace(chr(39), chr(39) + chr(39))}';"
        f"$allowed = @({name_filter});"
        "$me = $PID;"
        "$parent = (Get-CimInstance Win32_Process -Filter \"ProcessId = $me\").ParentProcessId;"
        "$matches = Get-CimInstance Win32_Process | Where-Object {"
        "  $_.CommandLine -and $_.CommandLine -like \"*$path*\""
        "  -and $_.ProcessId -ne $me -and $_.ProcessId -ne $parent"
        "  -and ($allowed -contains ($_.Name -replace '\\.exe$', '').ToLower())"
        "};"
        "if (-not $matches) { Write-Host '[kill] no matching processes'; exit 0 };"
        "$matches | ForEach-Object {"
        "  Write-Host \"[kill] $($_.ProcessId) $($_.Name)\";"
        "  Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue"
        "}"
    )
    return subprocess.call(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script]
    )


def _kill_by_path_unix(path: Path) -> int:
    """Kill dev-server processes (name-allowlisted) whose command references ``path``."""
    me = os.getpid()
    ppid = os.getppid()
    out = subprocess.check_output(["ps", "-eo", "pid=,comm=,command="]).decode()
    killed = 0
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        pid_str, comm, cmd = parts
        try:
            pid = int(pid_str)
        except ValueError:
            continue
        if pid in (me, ppid):
            continue
        if str(path) not in cmd:
            continue
        if _normalize_proc_name(Path(comm).name) not in DEV_PROCESS_NAMES:
            continue
        _log(f"{pid} {comm}")
        with open(os.devnull, "wb") as devnull:
            subprocess.call(
                ["kill", "-TERM", str(pid)], stdout=devnull, stderr=devnull
            )
        killed += 1
    if killed == 0:
        _log("no matching processes")
    return 0


def _kill_by_port_windows(port: int) -> int:
    script = (
        f"$port = {port};"
        "$conns = Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue;"
        "if (-not $conns) { Write-Host \"[kill] nothing listening on $port\"; exit 0 };"
        "$conns | Select-Object -Expand OwningProcess -Unique | ForEach-Object {"
        "  $proc = Get-Process -Id $_ -ErrorAction SilentlyContinue;"
        "  if ($proc) {"
        "    Write-Host \"[kill] $($proc.Id) $($proc.ProcessName)\";"
        "    Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue"
        "  }"
        "}"
    )
    return subprocess.call(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script]
    )


def _kill_by_port_unix(port: int) -> int:
    """Kill whoever is listening on ``port`` via lsof."""
    try:
        out = subprocess.check_output(["lsof", "-ti", f"tcp:{port}", "-sTCP:LISTEN"])
    except subprocess.CalledProcessError:
        _log(f"nothing listening on {port}")
        return 0
    pids = [p for p in out.decode().split() if p.strip()]
    if not pids:
        _log(f"nothing listening on {port}")
        return 0
    for pid in pids:
        _log(f"{pid}")
        with open(os.devnull, "wb") as devnull:
            subprocess.call(
                ["kill", "-TERM", pid], stdout=devnull, stderr=devnull
            )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group()
    g.add_argument(
        "--mine", action="store_true", default=True,
        help="kill processes for this worktree (default)",
    )
    g.add_argument(
        "--port", type=int, default=None,
        help="kill whoever is listening on this TCP port",
    )
    args = ap.parse_args()

    is_windows = sys.platform == "win32"

    if args.port is not None:
        _log(f"port mode: stopping listeners on :{args.port}")
        return (_kill_by_port_windows if is_windows else _kill_by_port_unix)(args.port)

    _log(f"path mode: stopping processes referencing {REPO_ROOT}")
    return (_kill_by_path_windows if is_windows else _kill_by_path_unix)(REPO_ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
