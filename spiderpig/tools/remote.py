"""Run heavy work (the full test suite, audits, sims, bakes) on a bigger machine.

``mise run remote -- <command>`` / ``mise run remote-test [-- pytest args]``:

1. take a lock on this worktree's remote folder (a second run from the same worktree
   waits; other worktrees and other machines get their own folder);
2. ``rsync`` the working tree as it is, uncommitted edits included (not git), without
   ``.venv``, ``node_modules``, the store, build outputs or caches;
3. ``uv sync --locked`` there (uv-managed Python 3.12; uv's cache, the interpreter and
   the venv all live under the remote folder);
4. run the command there with its output streamed here (and teed to a log);
5. copy the log and the remote ``build/`` (audit reports, exports, junit XML) back to
   ``build/remote/<run>/`` here.

The host is ``$SPIDERPIG_REMOTE`` (default ``root@ao-server``), the folder
``$SPIDERPIG_REMOTE_DIR`` (default ``spiderpig-ci``, relative to the remote home). The
remote run gets ``SPIDERPIG_STORE=.spiderpig`` of its own folder (never this one's).
The exit status is the remote command's.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HOST = os.environ.get("SPIDERPIG_REMOTE", "root@ao-server")
BASE = os.environ.get("SPIDERPIG_REMOTE_DIR", "spiderpig-ci")
# pytest workers there: 10 cores (20 threads), 60 GB, other work beside; 12 measured
# 5:42 against 5:29 at 16 and 6:55 at 20, and 16+ ran a CPU-time planner deadline out once
WORKERS = os.environ.get("SPIDERPIG_REMOTE_WORKERS", "12")

EXCLUDES = [
    ".git/", ".venv/", "node_modules/", ".spiderpig/", "build/", "dist/", "__pycache__/",
    "*.pyc", ".pytest_cache/", ".ruff_cache/", ".mypy_cache/", ".claude/", "MUJOCO_LOG.TXT",
    "spiderpig/viewer/dist/", "viewer/data/", "mise.local.toml", ".remote/",
    "/.python/", "/.uv-cache/",
]


def remote_dir() -> str:
    """This worktree's folder on the remote: the checkout's name plus a hash of the
    machine and path, so worktrees never share one."""
    key = f"{socket.gethostname()}:{ROOT}".encode()
    return f"{BASE}/{ROOT.name}-{hashlib.sha1(key).hexdigest()[:10]}"


def ssh(*args: str) -> list[str]:
    return ["ssh", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=30", HOST, *args]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="remote", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", action="store_true",
                    help=f"run pytest with xdist (-n {WORKERS}); the arguments go to pytest")
    ap.add_argument("--no-sync", action="store_true", help="skip rsync and uv sync")
    ap.add_argument("command", nargs=argparse.REMAINDER)
    a = ap.parse_args(argv)
    cmd = a.command[1:] if a.command[:1] == ["--"] else a.command
    run_id = time.strftime("%Y%m%d-%H%M%S")
    rdir = remote_dir()
    if a.test:
        junit = f"build/junit-{run_id}.xml"
        n = [] if any(x.startswith(("-n", "--numprocesses")) for x in cmd) else ["-n", WORKERS]
        dist = [] if any(x.startswith("--dist") for x in cmd) else ["--dist", "worksteal"]
        cmd = ["uv", "run", "--no-sync", "pytest", "-p", "no:warnings", *n, *dist,
               f"--junitxml={junit}", *cmd]
    elif not cmd:
        ap.error("no command (e.g. mise run remote -- uv run python -m spiderpig.cli audit)")

    # 1. the lock: a remote flock held for as long as this ssh lives
    lock = subprocess.Popen(
        # beside the folder, not in it: rsync --delete would remove it
        ssh(f"mkdir -p {rdir} && exec flock {rdir}.lock sh -c 'echo locked; cat >/dev/null'"),
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    print(f"[remote] {HOST}:{rdir} (waiting for the worktree lock)", file=sys.stderr, flush=True)
    if lock.stdout.readline().strip() != "locked":
        print("[remote] could not take the remote lock", file=sys.stderr)
        return 255
    try:
        t0 = time.monotonic()
        if not a.no_sync:
            # 2. the tree, uncommitted edits included; excluded paths survive --delete
            rsync = ["rsync", "-az", "--delete", *(f"--exclude={e}" for e in EXCLUDES),
                     f"{ROOT}/", f"{HOST}:{rdir}/"]
            subprocess.run(rsync, check=True)
            # 3. the environment, everything uv owns under the remote folder
            sync = ("uv python install 3.12 -q && "
                    "UV_PYTHON_PREFERENCE=only-managed uv sync --locked --python 3.12 -q")
            subprocess.run(ssh(f"cd {rdir} && {_env(rdir)} {sync}"), check=True)
            print(f"[remote] synced in {time.monotonic() - t0:.0f} s", file=sys.stderr,
                  flush=True)
        # 4. the command, streamed and logged
        log = f".remote/{run_id}.log"
        line = " ".join(shlex.quote(c) for c in cmd)
        script = (f"cd {rdir} && rm -rf build && mkdir -p build .remote && "
                  f"{_env(rdir)} SPIDERPIG_STORE=$PWD/.spiderpig "
                  f"bash -o pipefail -c {shlex.quote(f'{line} 2>&1 | tee {log}')}")
        print(f"[remote] $ {line}", file=sys.stderr, flush=True)
        t1 = time.monotonic()
        rc = subprocess.run(ssh("-t", script) if sys.stdout.isatty() else ssh(script)).returncode
        took = time.monotonic() - t1
        # 5. the log and build/ back here
        out = ROOT / "build" / "remote" / run_id
        out.mkdir(parents=True, exist_ok=True)
        subprocess.run(["rsync", "-az", f"{HOST}:{rdir}/{log}", f"{HOST}:{rdir}/build/",
                        f"{out}/"], check=False)
        print(f"[remote] exit {rc} after {took:.0f} s; log and build/ in {out}",
              file=sys.stderr)
        return rc
    finally:
        lock.stdin.close()
        lock.wait(timeout=30)


def _env(rdir: str) -> str:
    """uv's state under the remote folder, never the remote user's own."""
    return (f"UV_CACHE_DIR=$HOME/{BASE}/.uv-cache UV_PYTHON_INSTALL_DIR=$HOME/{BASE}/.python "
            "UV_PROJECT_ENVIRONMENT=.venv "
            "PYTHONUNBUFFERED=1")


if __name__ == "__main__":
    sys.exit(main())
