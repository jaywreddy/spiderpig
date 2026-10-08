"""Seam tests of the developer tools and the CLI front ends, no fabrication, no network,
no MuJoCo: ``spiderpig/tools/dev.py`` (ports, banner, the dev servers' supervisor with a
fake ``Popen``), ``tools/kill_dev.py`` (process-name matching, the unix and Windows kill
helpers with ``subprocess`` faked, ``main``'s modes), ``tools/remote.py`` (the remote
folder, the ssh / rsync commands ``main`` would run, recorded), ``tools/tune.py`` (the
phase arithmetic, flags, the grid and pattern searches and ``main`` on a fake walk
model), ``tools/sim_walk.py`` (drive speeds, argument handling, the report and ``main``
on a fake simulator) and ``spiderpig/build.py`` (arguments, ``--list``, file stems,
clearing generated files, the manifest, ``main``'s early exits)."""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import socket
import subprocess
import sys
import threading
import zlib
from pathlib import Path
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.no_fabricate

from spiderpig import build  # noqa: E402
from spiderpig.tools import dev, kill_dev, remote, sim_walk, tune  # noqa: E402

# ---------------------------------------------------------------------------
# dev.py
# ---------------------------------------------------------------------------


def test_hashed_port_is_crc32_of_the_worktree_and_salt():
    h = zlib.crc32(f"{dev.REPO_ROOT}|vite".encode())
    assert dev._hashed_port("vite", 5500, 5999) == 5500 + h % 500
    # stable, in range, and the salt matters (vite and api differ in general)
    for salt in ("vite", "api", "x", ""):
        p = dev._hashed_port(salt, 8500, 8999)
        assert 8500 <= p <= 8999
        assert p == dev._hashed_port(salt, 8500, 8999)
    assert dev._hashed_port("anything", 7000, 7000) == 7000      # a one-port range


def test_module_ports_honour_env_or_hash(monkeypatch):
    """``VITE_PORT`` / ``API_PORT`` pin the ports; unset, each is the worktree path's CRC32
    port in its range (the module reads them at import: reloaded under each environment)."""
    import importlib
    import zlib

    monkeypatch.setenv("VITE_PORT", "5173")
    monkeypatch.setenv("API_PORT", "8000")
    pinned = importlib.reload(dev)
    assert (pinned.WEB_PORT, pinned.API_PORT) == (5173, 8000)
    assert pinned.VIEWER_URL == "http://localhost:5173"
    assert pinned.API_HOST_PORT == ("127.0.0.1", 8000)
    monkeypatch.delenv("VITE_PORT")
    monkeypatch.delenv("API_PORT")
    hashed = importlib.reload(dev)
    root = hashed.REPO_ROOT
    assert 5500 + zlib.crc32(f"{root}|vite".encode()) % 500 == hashed.WEB_PORT
    assert 8500 + zlib.crc32(f"{root}|api".encode()) % 500 == hashed.API_PORT
    monkeypatch.undo()
    importlib.reload(dev)


class _Out:
    def __init__(self, encoding, tty=False):
        self.encoding = encoding
        self._tty = tty
        self.text = ""

    def isatty(self):
        return self._tty

    def write(self, s):
        self.text += s

    def flush(self):
        pass


@pytest.mark.parametrize(("enc", "want"), [("UTF-8", True), ("utf8", True), ("cp1252", False),
                                       ("ascii", False), (None, False)])
def test_supports_unicode(monkeypatch, enc, want):
    monkeypatch.setattr(sys, "stdout", _Out(enc))
    assert dev._supports_unicode() is want


def test_hyperlink_osc8_only_on_a_tty(monkeypatch):
    monkeypatch.setattr(sys, "stdout", _Out("utf-8", tty=False))
    assert dev._hyperlink("http://a", "A") == "A"
    monkeypatch.setattr(sys, "stdout", _Out("utf-8", tty=True))
    assert dev._hyperlink("http://a", "A") == "\x1b]8;;http://a\x1b\\A\x1b]8;;\x1b\\"


def test_port_open_on_a_listening_then_closed_port():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    try:
        assert dev._port_open("127.0.0.1", port, timeout=0.2) is True
    finally:
        srv.close()
    assert dev._port_open("127.0.0.1", port, timeout=0.2) is False


@pytest.mark.parametrize(("uni", "bar", "arrow"), [(True, "━", "→"), (False, "-", "->")])
def test_print_banner(monkeypatch, capsys, uni, bar, arrow):
    monkeypatch.setattr(dev, "_supports_unicode", lambda: uni)
    monkeypatch.setattr(dev, "VIEWER_URL", "http://localhost:5555")
    monkeypatch.setattr(dev, "API_PORT", 8555)
    dev._print_banner()
    lines = capsys.readouterr().out.splitlines()
    assert lines[0] == ""
    assert lines[-1] == ""
    assert lines[1] == bar * 60 == lines[-2]
    assert lines[2] == "  spiderpig viewer ready"
    assert lines[3] == f"  {arrow} http://localhost:5555    (click to open)"
    assert lines[4] == "  api: 127.0.0.1:8555  (proxied via vite)"


@pytest.mark.parametrize("no_browser", [False, True])
def test_wait_then_announce_ready(monkeypatch, capsys, no_browser):
    opened = []
    monkeypatch.setattr(dev, "_port_open", lambda host, port: True)
    monkeypatch.setattr(dev.webbrowser, "open", lambda url, new: opened.append((url, new)))
    if no_browser:
        monkeypatch.setenv("SPIDERPIG_NO_BROWSER", "1")
    else:
        monkeypatch.delenv("SPIDERPIG_NO_BROWSER", raising=False)
    dev._wait_then_announce(threading.Event())
    assert "spiderpig viewer ready" in capsys.readouterr().out
    assert opened == ([] if no_browser else [(dev.VIEWER_URL, 2)])


def test_wait_then_announce_times_out_or_stops(monkeypatch, capsys):
    monkeypatch.setattr(dev, "_port_open", lambda host, port: False)
    monkeypatch.setattr(dev, "READY_TIMEOUT_S", 0.0)
    dev._wait_then_announce(threading.Event())
    out = capsys.readouterr().out
    assert out.startswith("[dev] servers did not both come up within 0s")
    assert f"open {dev.VIEWER_URL} manually" in out
    stop = threading.Event()
    stop.set()
    monkeypatch.setattr(dev, "READY_TIMEOUT_S", 30.0)
    dev._wait_then_announce(stop)                 # stopped: no banner, no complaint
    assert capsys.readouterr().out == ""


class _Proc:
    """A fake Popen: ``polls`` is what poll() returns in turn (the last repeats)."""

    def __init__(self, polls, pid, wait_raises=None):
        self.polls = list(polls)
        self.pid = pid
        self.terminated = 0
        self.waits = []
        self.wait_raises = list(wait_raises or [])

    def poll(self):
        return self.polls.pop(0) if len(self.polls) > 1 else self.polls[0]

    def terminate(self):
        self.terminated += 1

    def wait(self, timeout=None):
        self.waits.append(timeout)
        if self.wait_raises:
            raise self.wait_raises.pop(0)
        return 0


def _dev_main(monkeypatch, api, web):
    started, handlers = [], {}

    def popen(cmd, cwd=None, env=None, **kw):
        started.append((cmd, cwd, env))
        return api if len(started) == 1 else web

    monkeypatch.setattr(dev.subprocess, "Popen", popen)
    monkeypatch.setattr(dev.signal, "signal", lambda sig, h: handlers.__setitem__(sig, h))
    monkeypatch.setattr(dev, "_wait_then_announce", lambda stop: None)
    rc = dev.main()
    return rc, started, handlers


def test_dev_main_api_exit_stops_web(monkeypatch, capsys):
    api, web = _Proc([3], 11), _Proc([None], 22)
    rc, started, handlers = _dev_main(monkeypatch, api, web)
    assert rc == 3
    (api_cmd, api_cwd, _), (web_cmd, web_cwd, web_env) = started
    assert api_cmd[1:5] == ["-u", "-m", "uvicorn", "spiderpig.server.app:app"]
    assert api_cmd[-2:] == ["--port", str(dev.API_PORT)]
    assert api_cwd == dev.REPO_ROOT
    assert web_cmd == ["npm", "run", "dev"]
    assert web_cwd == dev.VIEWER_DIR
    assert web_env["VITE_PORT"] == str(dev.WEB_PORT)
    assert web_env["VITE_API_PORT"] == str(dev.API_PORT)
    assert web.terminated == 1
    assert api.terminated == 0
    assert web.waits == [5]
    import signal
    assert set(handlers) >= {signal.SIGINT, signal.SIGTERM}
    out = capsys.readouterr().out
    assert "[dev] FastAPI pid=11" in out
    assert "Vite pid=22" in out
    assert "api exited rc=3; shutting down web" in out


def test_dev_main_web_exit_stops_api_after_a_wait(monkeypatch, capsys):
    api = _Proc([None], 11, wait_raises=[subprocess.TimeoutExpired("api", 0.5)])
    web = _Proc([None, 1], 22)
    rc, _, handlers = _dev_main(monkeypatch, api, web)
    assert rc == 1
    assert api.terminated == 1
    assert web.terminated == 0
    assert api.waits == [0.5, 5]
    assert "web exited rc=1; shutting down api" in capsys.readouterr().out
    import signal
    handlers[signal.SIGINT]()               # the handler terminates whatever still runs
    assert api.terminated == 2


def test_dev_main_keyboard_interrupt(monkeypatch):
    api = _Proc([None], 11, wait_raises=[KeyboardInterrupt()])
    web = _Proc([None], 22)
    rc, _, _ = _dev_main(monkeypatch, api, web)
    assert rc == 0
    assert api.terminated == 1
    assert web.terminated == 1


# ---------------------------------------------------------------------------
# kill_dev.py
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(("raw", "want"), [("node", "node"), ("node.exe", "node"),
                                       ("Python.EXE", "python"), ("  npm  ", "npm"),
                                       ("exe", "exe"), ("my.exe.bak", "my.exe.bak"),
                                       ("Code.exe", "code")])
def test_normalize_proc_name(raw, want):
    assert kill_dev._normalize_proc_name(raw) == want


def test_kill_by_path_unix_kills_only_allowlisted_processes_of_this_path(monkeypatch, capsys):
    path = Path("/work/tree")
    me, parent = os.getpid(), os.getppid()
    ps = "\n".join([
        f"{me} python python -m x /work/tree",            # this process: never
        f"{parent} python python /work/tree/mise",        # its parent: never
        "101 /usr/bin/node node /work/tree/viewer/vite",  # killed
        "102 node.exe node /elsewhere/vite",              # another worktree
        "103 code code /work/tree",                       # not a dev server
        "104 Python.exe python -m uvicorn /work/tree",    # killed (.exe, case)
        "",
        "short line",
        "abc node /work/tree",                            # malformed: 2 fields
        "xyz node cmd /work/tree",                        # pid not a number
    ]).encode()
    calls = []
    monkeypatch.setattr(kill_dev.subprocess, "check_output", lambda cmd: ps)
    monkeypatch.setattr(kill_dev.subprocess, "call",
                        lambda cmd, **kw: calls.append(cmd) or 0)
    assert kill_dev._kill_by_path_unix(path) == 0
    assert calls == [["kill", "-TERM", "101"], ["kill", "-TERM", "104"]]
    out = capsys.readouterr().out
    assert "[kill] 101 /usr/bin/node" in out
    assert "[kill] 104 Python.exe" in out
    assert "no matching processes" not in out


def test_kill_by_path_unix_nothing(monkeypatch, capsys):
    monkeypatch.setattr(kill_dev.subprocess, "check_output", lambda cmd: b"1 init /sbin/init\n")
    monkeypatch.setattr(kill_dev.subprocess, "call", lambda *a, **k: pytest.fail("killed"))
    assert kill_dev._kill_by_path_unix(Path("/nowhere")) == 0
    assert capsys.readouterr().out == "[kill] no matching processes\n"


def test_kill_by_port_unix(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(kill_dev.subprocess, "call", lambda cmd, **kw: calls.append(cmd) or 0)

    def failing(cmd):
        assert cmd == ["lsof", "-ti", "tcp:5173", "-sTCP:LISTEN"]
        raise subprocess.CalledProcessError(1, cmd)

    monkeypatch.setattr(kill_dev.subprocess, "check_output", failing)
    assert kill_dev._kill_by_port_unix(5173) == 0
    monkeypatch.setattr(kill_dev.subprocess, "check_output", lambda cmd: b"  \n")
    assert kill_dev._kill_by_port_unix(5173) == 0
    assert calls == []
    assert capsys.readouterr().out == ("[kill] nothing listening on 5173\n" * 2)
    monkeypatch.setattr(kill_dev.subprocess, "check_output", lambda cmd: b"42\n43\n")
    assert kill_dev._kill_by_port_unix(5173) == 0
    assert calls == [["kill", "-TERM", "42"], ["kill", "-TERM", "43"]]


def test_kill_windows_scripts(monkeypatch):
    calls = []
    monkeypatch.setattr(kill_dev.subprocess, "call", lambda cmd: calls.append(cmd) or 7)
    assert kill_dev._kill_by_path_windows(Path("C:/it's here")) == 7
    assert kill_dev._kill_by_port_windows(8123) == 7
    (path_cmd, port_cmd) = calls
    assert path_cmd[:4] == ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command"]
    # a quote in the path is doubled for PowerShell's single-quoted string
    assert "$path = 'C:/it''s here';" in path_cmd[4]
    assert "$allowed = @('node','npm','python','pythonw','uvicorn');" in path_cmd[4]
    assert port_cmd[4].startswith("$port = 8123;")


@pytest.mark.parametrize(("platform", "argv", "want"), [
    ("linux", [], ("path_unix", kill_dev.REPO_ROOT)),
    ("linux", ["--mine"], ("path_unix", kill_dev.REPO_ROOT)),
    ("linux", ["--port", "5173"], ("port_unix", 5173)),
    ("win32", [], ("path_windows", kill_dev.REPO_ROOT)),
    ("win32", ["--port", "80"], ("port_windows", 80)),
])
def test_kill_dev_main_dispatch(monkeypatch, capsys, platform, argv, want):
    seen = []
    for name in ("path_unix", "port_unix", "path_windows", "port_windows"):
        kind, osname = name.split("_")
        monkeypatch.setattr(kill_dev, f"_kill_by_{kind}_{osname}",
                            lambda arg, name=name: seen.append((name, arg)) or 5)
    monkeypatch.setattr(kill_dev, "sys", SimpleNamespace(platform=platform))
    monkeypatch.setattr(sys, "argv", ["kill_dev", *argv])
    assert kill_dev.main() == 5
    assert seen == [want]
    out = capsys.readouterr().out
    assert ("port mode" if "port" in want[0] else "path mode") in out


def test_kill_dev_main_rejects_both_modes(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["kill_dev", "--mine", "--port", "1"])
    with pytest.raises(SystemExit) as e:
        kill_dev.main()
    assert e.value.code == 2
    assert "not allowed with argument" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# remote.py
# ---------------------------------------------------------------------------


def test_remote_dir_and_ssh(monkeypatch):
    import hashlib

    key = f"{socket.gethostname()}:{remote.ROOT}".encode()
    want = f"{remote.BASE}/{remote.ROOT.name}-{hashlib.sha1(key).hexdigest()[:10]}"
    assert remote.remote_dir() == want
    monkeypatch.setattr(remote, "ROOT", Path("/a/b/tree"))
    monkeypatch.setattr(remote, "BASE", "ci")
    other = remote.remote_dir()
    assert other.startswith("ci/tree-")
    assert len(other) == len("ci/tree-") + 10
    monkeypatch.setattr(remote, "HOST", "me@box")
    assert remote.ssh("ls", "-l") == ["ssh", "-o", "BatchMode=yes", "-o",
                                      "ServerAliveInterval=30", "me@box", "ls", "-l"]


def test_remote_env_keeps_uv_state_under_the_base(monkeypatch):
    monkeypatch.setattr(remote, "BASE", "ci")
    env = remote._env("ci/x")
    assert env == ("UV_CACHE_DIR=$HOME/ci/.uv-cache UV_PYTHON_INSTALL_DIR=$HOME/ci/.python "
                   "UV_PROJECT_ENVIRONMENT=.venv PYTHONUNBUFFERED=1")


class _Lock:
    def __init__(self, first_line):
        self.stdout = SimpleNamespace(readline=lambda: first_line)
        self.closed = False
        self.waited = None
        self.stdin = SimpleNamespace(close=lambda: setattr(self, "closed", True))

    def wait(self, timeout=None):
        self.waited = timeout


def _remote(monkeypatch, tmp_path, argv, first_line="locked\n", rc=0):
    monkeypatch.setattr(remote, "ROOT", tmp_path)
    monkeypatch.setattr(remote, "HOST", "u@h")
    lock = _Lock(first_line)
    popened, ran = [], []

    def popen(cmd, **kw):
        popened.append(cmd)
        return lock

    def run(cmd, check=False):
        ran.append(cmd)
        return SimpleNamespace(returncode=rc)

    monkeypatch.setattr(remote.subprocess, "Popen", popen)
    monkeypatch.setattr(remote.subprocess, "run", run)
    out = remote.main(argv)
    return out, lock, popened, ran


def test_remote_main_runs_the_command_and_fetches_build(monkeypatch, tmp_path, capsys):
    rc, lock, popened, ran = _remote(monkeypatch, tmp_path,
                                     ["--", "uv", "run", "echo", "a b"], rc=4)
    assert rc == 4
    rdir = remote.remote_dir()
    assert popened[0][:6] == ["ssh", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=30",
                              "u@h"]
    assert f"flock {rdir}.lock" in popened[0][6]
    rsync, sync, cmd, fetch = ran
    assert rsync[:3] == ["rsync", "-az", "--delete"]
    assert "--exclude=.git/" in rsync
    assert rsync[-2:] == [f"{tmp_path}/", f"u@h:{rdir}/"]
    assert "uv sync --locked" in sync[-1]
    assert sync[-1].startswith(f"cd {rdir} && ")
    assert "-t" not in cmd                         # not a tty under capture
    inner = shlex.split(cmd[-1])[-1]               # what bash -o pipefail -c runs
    assert inner.startswith("uv run echo 'a b' 2>&1 | tee .remote/")
    assert "SPIDERPIG_STORE=$PWD/.spiderpig" in cmd[-1]
    assert fetch[0:2] == ["rsync", "-az"]
    assert fetch[2].startswith(f"u@h:{rdir}/.remote/")
    out_dir = Path(fetch[-1])
    assert out_dir.parent == tmp_path / "build" / "remote"
    assert out_dir.is_dir()
    assert lock.closed
    assert lock.waited == 30
    err = capsys.readouterr().err
    assert "[remote] $ uv run echo 'a b'" in err
    assert "[remote] exit 4" in err


@pytest.mark.parametrize(("extra", "n", "dist"), [([], ["-n", remote.WORKERS],
                                             ["--dist", "worksteal"]),
                                            (["-n", "3", "--dist=load"], [], [])])
def test_remote_main_test_mode(monkeypatch, tmp_path, extra, n, dist):
    _, _, _, ran = _remote(monkeypatch, tmp_path, ["--test", "--no-sync", "--", *extra, "tests/x"])
    assert len(ran) == 2                           # --no-sync: no rsync, no uv sync
    line = shlex.split(ran[0][-1])[-1]
    head = " ".join(["uv", "run", "--no-sync", "pytest", "-p", "no:warnings", *n, *dist])
    assert head + " --junitxml=build/junit-" in line
    assert line.count(" -n ") == 1
    assert " tests/x 2>&1 | tee .remote/" in line


def test_remote_main_needs_a_command(monkeypatch, tmp_path, capsys):
    with pytest.raises(SystemExit) as e:
        _remote(monkeypatch, tmp_path, [])
    assert e.value.code == 2
    assert "no command" in capsys.readouterr().err


def test_remote_main_lock_refused(monkeypatch, tmp_path, capsys):
    rc, lock, _, ran = _remote(monkeypatch, tmp_path, ["true"], first_line="")
    assert rc == 255
    assert ran == []
    assert "could not take the remote lock" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# tune.py
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(("a", "b", "want"), [(0, 0, 0), (0, 180, 180), (10, 350, 20),
                                        (350, 10, 20), (0, 360, 0), (90, -90, 180),
                                        (5, 725, 0)])
def test_gap(a, b, want):
    assert tune._gap(a, b) == pytest.approx(want)


def test_fmt_and_phases_arg():
    assert tune._fmt([1.0, 2.345]) == "[1.00, 2.35]"
    assert tune._fmt(1.23456) == "1.235"
    assert tune._fmt(3) == "3"
    assert tune._fmt(True) == "True"
    assert tune._fmt("x") == "x"
    assert tune._phases_arg((0.0, 175.0, 180.5, 355)) == "0,175,180.5,355"
    assert tune._phases_arg(()) == ""


def test_candidate_is_hashable_and_scored_holds_it():
    a = tune.Candidate((0.0, 180.0))
    assert a == tune.Candidate((0.0, 180.0))
    assert len({a, tune.Candidate((0.0, 180.0))}) == 1
    with pytest.raises(AttributeError):
        a.phases = (1.0,)                              # frozen
    s = tune.Scored(a, 1.5, None)
    assert s.candidate is a
    assert s.score == 1.5
    assert s.metrics is None


def test_flags():
    c = tune.Candidate((0.0, 175.0), (("crank", 4.25), ("bar", 14.0)))
    f = tune.flags("double", c)
    assert f["main"] == ("spiderpig build --module double --phases 0,175 "
                         "--proportion crank=4.25 --proportion bar=14")
    assert f["bake"] == f["main"].replace("build", "bake", 1)
    assert f["query"] == "?module=double&phases=0,175&p.crank=4.25&p.bar=14"
    g = tune.flags("quad", tune.Candidate((0.0,)), "jansen")
    assert g["main"] == "spiderpig build --module quad --phases 0 --linkage jansen"
    assert g["query"] == "?module=quad&phases=0&linkage=jansen"


TARGET = 100.0          # the fake walk's best second phase
CRANK = 4.2             # and its best Strider crank
FLOOR = 1.0 + 0.5 + 10 * abs(4.0 - CRANK)   # pitch + roll ranges, the default crank's cost


def _fake_metrics(self, c, n=None, feet_z=None):
    """A walk whose bob is the second leg's distance from TARGET (plus the crank's)."""
    self.evaluations += 1
    crank = dict(c.proportions).get("crank", 4.0)
    bob = (tune._gap(c.phases[1], TARGET) if len(c.phases) > 1 else 0.0) \
        + 10 * abs(crank - CRANK)
    return {"stride_mm": 100.0, "bob_mm": bob, "pitch_deg": [0.0, 1.0],
            "roll_deg": [0.0, 0.5], "slip_rms_mm_per_rev": 0.0, "min_margin_mm": 5.0,
            "tipping_fraction": 0.0, "degenerate_fraction": 0.0, "speed_mm_s": 30.0,
            "yaw_deg_per_rev": 0.0, "mean_contacts": 3.0, "walks": True}


@pytest.fixture
def fake_walk(monkeypatch):
    monkeypatch.setattr(tune.Tuner, "metrics", _fake_metrics)


def test_tuner_default_pairs_and_feasible(fake_walk):
    t = tune.Tuner("quad", stride_ref=100.0)
    assert t.default.phases == (0.0, 180.0, 90.0, 270.0)
    assert t.evaluations == 0                       # stride_ref given: no default walk
    assert t.feasible((0.0, 180.0, 90.0, 270.0))
    assert not t.feasible((0.0, 180.0, 90.0, 93.0))   # 3 deg apart < MIN_GAP
    assert t.feasible((0.0, 180.0, 90.0, 95.0))       # exactly the gap
    cfg = t.config(tune.Candidate((0.0, 180.0, 90.0, 270.0)))
    assert cfg.module == "quad"
    assert cfg.linkage == "strider"
    t2 = tune.Tuner("double")                       # stride_ref from the default's walk
    assert t2.stride_ref == 100.0
    assert t2.evaluations == 1


def test_tuner_score_memoizes_and_rejects_infeasible(fake_walk):
    t = tune.Tuner("double", stride_ref=100.0, min_gap=30.0)
    s = t.score(tune.Candidate((0.0, 90.0)))
    assert s.score == pytest.approx(10.0 + FLOOR)        # bob + the rest
    assert t.score(tune.Candidate((360.0, 450.0))) is s  # same phases mod 360
    assert t.evaluations == 1
    bad = t.score(tune.Candidate((0.0, 10.0)))           # 10 deg < min_gap
    assert bad.score == math.inf
    assert bad.metrics is None
    assert t.evaluations == 1


def test_grid_and_pattern_search_find_the_target(fake_walk, capsys):
    t = tune.Tuner("double", stride_ref=100.0)
    top = tune.grid_search(t, 30.0, 60, 3)
    assert [s.candidate.phases[1] for s in top] == [90.0, 120.0, 60.0]
    assert top[0].score <= top[1].score <= top[2].score
    best = tune.pattern_search(t, top[0].candidate)
    assert best.candidate.phases == (0.0, TARGET)
    assert best.score == pytest.approx(FLOOR)
    single = tune.Tuner("single", stride_ref=100.0)
    (only,) = tune.grid_search(single, 30.0, 60, 3)
    assert only.candidate == single.default


def test_pattern_search_with_proportions_stays_in_bounds(fake_walk):
    t = tune.Tuner("double", stride_ref=100.0)
    start = tune.Candidate((0.0, TARGET))
    best = tune.pattern_search(t, start, ("crank",), 10.0)
    crank = dict(best.candidate.proportions)["crank"]
    assert crank == pytest.approx(CRANK)
    narrow = tune.pattern_search(t, start, ("crank",), 2.0)    # 4.2 is 5 % out: capped at 2 %
    assert dict(narrow.candidate.proportions)["crank"] == pytest.approx(4.08)


def test_tune_returns_default_and_best(fake_walk):
    tuner, default, best = tune.tune("double", grid=90.0, top=2)
    assert default.candidate == tuner.default
    assert default.score == pytest.approx(80 + FLOOR)
    assert best.candidate.phases == (0.0, TARGET)
    assert tuner.phase_best is best


def test_tune_main_prints_and_writes_json(fake_walk, tmp_path, capsys):
    out = tmp_path / "best.json"
    assert tune.main(["--module", "double", "--grid", "90", "--top", "2",
                      "--json", str(out)]) == 0
    text = capsys.readouterr().out
    assert "strider double:" in text
    assert "stride reference 100.0 mm/rev" in text
    assert "spiderpig build --module double --phases 0,100" in text
    assert "?module=double&phases=0,100" in text
    data = json.loads(out.read_text())
    assert data["best"]["phases_deg"] == [0.0, TARGET]
    assert data["default"]["phases_deg"] == [0.0, 180.0]
    assert data["use"]["main"] == "spiderpig build --module double --phases 0,100"


def test_tune_main_plan_falls_back_on_default_proportions(fake_walk, monkeypatch, capsys):
    calls = []

    def plan(tuner, c):
        calls.append(c)
        if c.proportions:
            return {"ok": False, "error": "no layers"}
        return {"ok": True, "layers": 9, "foot_z": [1.04, 2.0], "objective": 1.5}

    monkeypatch.setattr(tune, "plan", plan)
    assert tune.main(["--module", "double", "--grid", "90", "--top", "1",
                      "--proportions", "10", "--names", "crank", "--plan"]) == 0
    text = capsys.readouterr().out
    assert len(calls) == 2
    assert calls[0].proportions
    assert not calls[1].proportions
    assert "plan: the best design can't be built as is: no layers" in text
    assert "best with the default proportions: phases 0,100" in text
    assert "plan: 9 layers, foot z [1.0, 2.0]" in text
    assert "proportion crank" in text


def test_tune_main_argument_errors(capsys):
    with pytest.raises(SystemExit) as e:
        tune.main(["--help"])
    assert e.value.code == 0
    assert "--proportions PCT" in capsys.readouterr().out
    for argv, msg in ((["--module", "hex"], "unknown module 'hex'"),
                      (["--names", "crank,nope"], "unknown proportions ['nope']"),
                      (["--linkage", "hoecken_pantograph"], "invalid choice")):
        with pytest.raises(SystemExit) as e:
            tune.main(argv)
        assert e.value.code == 2
        assert msg in capsys.readouterr().err


def test_tune_main_single_has_no_phases(capsys):
    assert tune.main(["--module", "single"]) == 0
    assert "single: one leg per side, so no phases to tune" in capsys.readouterr().out


def test_tune_plan_reports_a_failing_design(monkeypatch, fake_walk):
    from spiderpig import fabricate

    def boom(config):
        raise ValueError("can't template")

    monkeypatch.setattr(fabricate, "template_for", boom)
    t = tune.Tuner("double", stride_ref=100.0)
    assert tune.plan(t, t.default) == {"ok": False, "error": "can't template"}


# ---------------------------------------------------------------------------
# sim_walk.py
# ---------------------------------------------------------------------------


def test_parse_speed():
    rpm = sim_walk.RPM
    assert sim_walk.parse_speed("40rpm", 10.0) == pytest.approx(40 * rpm)
    assert sim_walk.parse_speed(" -40RPM ", 10.0) == pytest.approx(-40 * rpm)
    assert sim_walk.parse_speed("80%", 10.0) == pytest.approx(8.0)
    assert sim_walk.parse_speed("0.5", 10.0) == pytest.approx(5.0)
    assert sim_walk.parse_speed("-1", 10.0) == pytest.approx(-10.0)    # the bound itself
    with pytest.raises(argparse.ArgumentTypeError, match="'1.5': plain numbers are fractions"):
        sim_walk.parse_speed("1.5", 10.0)
    with pytest.raises(ValueError, match="could not convert string to float: 'fast'"):
        sim_walk.parse_speed("fast", 10.0)


def test_sim_args_defaults_and_mjcf(tmp_path, capsys):
    a = sim_walk._args([])
    assert (a.seconds, a.left, a.right, a.settle, a.skip) == (4.0, "0.8", "0.8", 0.5, 1.0)
    assert a.model is None
    assert a.config.linkage == "strider"
    xml = tmp_path / "m.xml"
    xml.write_text("<mujoco/>")
    with pytest.raises(SystemExit) as e:
        sim_walk._args(["--mjcf", str(xml)])          # no .json beside it
    assert e.value.code == 2
    assert "--mjcf needs" in capsys.readouterr().err
    xml.with_suffix(".json").write_text('{"k": 1}')
    a = sim_walk._args(["--mjcf", str(xml)])
    assert a.model == ("<mujoco/>", {"k": 1})
    assert f"running {xml}" in capsys.readouterr().err


def test_sim_args_bad_design_and_unknown_id(tmp_path, capsys):
    with pytest.raises(SystemExit):
        sim_walk._args(["--module", "hexapod"])
    assert "hexapod" in capsys.readouterr().err
    with pytest.raises(SystemExit) as e:
        sim_walk._args(["0123456789abcdef", "--store", str(tmp_path / "store")])
    assert e.value.code == 2
    assert "no design '0123456789abcdef'" in capsys.readouterr().err


def _sim_metrics(**over):
    torque = {"peak": 0.5, "peak_fraction": 0.4, "limit": 1.2, "mean": 0.2, "rms": 0.25,
              "saturated": 0.01, "mean_over_rated": 0.5, "rated": 0.4, "at_envelope": 0.1,
              "speed_droop": 0.05, "speed_under_load": 4.0, "power": 1.5}
    m = {"mass": 0.85, "speed": 30.0, "lateral": 2.0, "heading_drift": 1.5, "yaw_rate": 0.25,
         "drives_oppose": False, "stride": 40.0, "revolutions": 3.0, "revolutions_abs": 3.0,
         "height": 50.0, "bob": 2.0, "pitch_range": 3.0, "roll_range": 1.0, "max_tilt": 4.0,
         "fell": False, "body_contact": 0.0, "walks": True, "side_phase": 0.5,
         "side_phase_max": 2.0, "torque": {"left": torque, "right": dict(torque, rated=None)},
         "feet_down": 2.5, "airborne": 0.0, "side_support_low": 0.1, "slip": 5.0,
         "slip_max": 20.0, "airborne_per_rev": [0.0, 0.1], "accel_z_peak_g": 1.2,
         "foot_force_peak": 9.0, "loop_force_p999": 30.0, "loop_force_peak": 40.0,
         "torque_limit_recommended": 0.6, "torque_limit": 1.2, "joint_moment_at_limit": 0.8,
         "crank_capacity_nm": 1.7, "crank_weakest": "hex pocket", "joint_moment_peak": 0.4,
         "joint_moment_factor": 1.3, "loop_error": 0.01, "penetration": 0.5,
         "torque_peak": 0.5}
    m.update(over)
    return m


KIN = {"stride": 50.0, "bob": 1.0}


def test_sim_report_walking(capsys):
    cfg = sim_walk.BuildConfig()
    sim_walk._report(cfg, _sim_metrics(), KIN, 1.0, 1.0, 4.0, sim_walk.SimParams())
    out = capsys.readouterr().out
    assert out.startswith("strider double robot, ")
    assert "850 g (payload 0 g of it), 4 s" in out
    assert "phase-locked (PI 1/4) against a 3 % slower right servo" in out
    assert "stride 40.0 mm/rev: the feet slip 20 % of the 50 mm" in out
    assert "fell over: no;" in out
    assert "DOES NOT WALK" not in out
    assert "no rated torque in the catalog" in out
    assert "50 % of the 0.40 N·m rated" in out
    assert "airborne per revolution 0, 10 %" in out
    assert "its weakest element (hex pocket) 1.70" in out
    assert "set the servo's torque limit to 0.60 N·m (50 % of stall)" in out


def test_sim_report_turning_fallen_open_loop(capsys):
    params = sim_walk.SimParams(phase_lock_kp=0.0, phase_lock_ki=0.0)
    m = _sim_metrics(drives_oppose=True, fell=True, fell_at_s=1.25, fell_axis="roll",
                     walks=False, torque_limit_recommended=None, crank_capacity_nm=None)
    sim_walk._report(sim_walk.BuildConfig(), m, KIN, 1.0, -1.0, 2.0, params)
    out = capsys.readouterr().out
    assert "open loop, right servo 3 % slow" in out
    assert "3.00 crank revolutions (the drives opposed: a turn, no stride)" in out
    assert "fell over: YES (roll at 1.2 s into the run)" in out
    assert "DOES NOT WALK" in out
    assert "set the servo's torque limit" not in out
    assert "weakest element" not in out


def _fake_sim(monkeypatch, calls):
    monkeypatch.setattr(sim_walk, "simulate",
                        lambda cfg, controls, seconds, params=None, **kw:
                        calls.append(("sim", controls, seconds, params, kw)) or "result")
    monkeypatch.setattr(sim_walk, "walk_metrics", lambda result, skip: _sim_metrics())
    monkeypatch.setattr(sim_walk, "kinematic_gait", lambda cfg: KIN)
    monkeypatch.setattr(sim_walk, "compare_with_walk", lambda m, cfg: {
        "quasi_static": {"speed_mm_s": 31.0, "stride_mm": 45.0, "bob_mm": 1.5,
                         "mean_contacts": 3.0, "min_margin_mm": 8.0},
        "mujoco": {"speed_mm_s": 60.0, "feet_down": 2.5}, "speed_ratio": 0.9,
        "flags": ["slips"]})
    monkeypatch.setattr(sim_walk, "build_mjcf", lambda cfg, params: ("<mujoco/>", {"a": 1}))


def test_sim_main_text_with_sweep_and_xml(monkeypatch, tmp_path, capsys):
    calls = []
    _fake_sim(monkeypatch, calls)
    xml = tmp_path / "out" / "q.xml"
    assert sim_walk.main(["--left", "50%", "--right", "20rpm", "--seconds", "2",
                          "--contact-sweep", "--xml", str(xml)]) == 0
    assert xml.read_text() == "<mujoco/>"
    assert json.loads(xml.with_suffix(".json").read_text()) == {"a": 1}
    (_, controls, seconds, params, kw), *sweep = calls
    assert controls[0] == (0.0, 0.0, 0.0)
    assert controls[1][0] == 0.5
    assert controls[1][2] == pytest.approx(20 * sim_walk.RPM)
    assert seconds == 2.5
    assert kw == {"model_xml": None, "model_meta": None}
    assert [p.contact_solref for _, _, _, p, _ in sweep] == [(0.005, 1.0), (0.02, 1.0)]
    out = capsys.readouterr().out
    assert "over solref 5-20 ms: speed 30-30 mm/s" in out
    assert "vs model  quasi-static 31 mm/s" in out
    assert "flags: slips" in out


def test_sim_main_json(monkeypatch, capsys):
    calls = []
    _fake_sim(monkeypatch, calls)
    assert sim_walk.main(["--json"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data["kinematic"] == KIN
    assert data["contact_sweep"] is None
    assert data["comparison"]["flags"] == ["slips"]
    assert data["metrics"]["speed"] == 30.0


# ---------------------------------------------------------------------------
# build.py
# ---------------------------------------------------------------------------


def test_build_parse_args():
    a = build._parse_args([])
    assert a.name == "strider"
    assert a.out == Path("build")
    assert a.config.robot
    assert a.kerf is None
    assert a.sheet_size is None
    assert not a.no_dxf
    b = build._parse_args(["--side-only", "--name", "x", "--kerf", "0.1",
                           "--sheet-size", "300", "200", "--out", "o"])
    assert not b.config.robot
    assert b.name == "x"
    assert b.kerf == 0.1
    assert b.sheet_size == [300.0, 200.0]
    assert b.out == Path("o")


def test_build_parse_args_error(capsys):
    with pytest.raises(SystemExit) as e:
        build._parse_args(["--proportion", "nope=1"])
    assert e.value.code == 2
    assert "nope" in capsys.readouterr().err


def test_list_options(capsys):
    assert build.main(["--list"]) == 0
    out = capsys.readouterr().out
    assert out.startswith("linkages (--linkage;")
    assert "  strider " in out
    assert "crank=4" in out
    for title in ("servos (full rotation):", "pillars / pins (--pillar, --pin):",
                  "cranks (--crank):", "sheet stock (--sheet):"):
        assert title in out
    assert "  standoff " in out
    assert "  bolt " in out


def test_file_stem_collisions():
    taken: set[str] = set()
    assert build._file_stem("L.b1", taken) == "b1"
    assert build._file_stem("R.b1", taken) == "R_b1"          # b1 taken: the full name
    assert build._file_stem("spacer 3.5 mm", taken) == "spacer_3.5_mm"
    assert build._file_stem("spacer/3.5 mm", taken) == "spacer_3_5_mm"   # taken: no dots
    assert build._file_stem("a.b", taken) == "a.b"
    assert build._file_stem("a.b", taken) == "a_b"             # dots go on a collision
    assert taken >= {"b1", "R_b1", "spacer_3.5_mm", "spacer_3_5_mm", "a.b", "a_b"}


def test_clear_generated(tmp_path):
    keep = tmp_path / "laser"
    (keep / "parts" / "x").mkdir(parents=True)
    (keep / "a.dxf").write_text("")
    (keep / "parts" / "x" / "b.DXF").write_text("")
    (keep / "parts" / "order.csv").write_text("")
    (keep / "notes.txt").write_text("mine")
    build.clear_generated(keep)
    assert sorted(p.relative_to(keep).as_posix() for p in keep.rglob("*")) == ["notes.txt"]
    gone = tmp_path / "print"
    (gone / "sub").mkdir(parents=True)
    (gone / "p.stl").write_text("")
    (gone / "sub" / "parts.csv").write_text("")
    build.clear_generated(gone)
    assert not gone.exists()
    build.clear_generated(tmp_path / "missing")              # no folder: nothing to do
    assert build.GENERATED == (".dxf", ".stl", ".csv")


def test_write_manifest(tmp_path):
    args = build._parse_args(["--kerf", "0.15", "--sheet-size", "300", "200", "--name", "n"])
    build._write_manifest(tmp_path, args.config, args)
    m = json.loads((tmp_path / "manifest.json").read_text())
    assert m["written_by"] == "spiderpig build"
    assert m["kerf_mm"] == 0.15
    assert m["sheet_size_mm"] == [300.0, 200.0]
    assert m["name"] == "n"
    assert len(m["design"]) == 16
    assert m["engine_version"]
    plain = tmp_path / "plain"
    plain.mkdir()
    build._write_manifest(plain, args.config)                 # no args: no fit options
    p = json.loads((plain / "manifest.json").read_text())
    assert p["kerf_mm"] is None
    assert p["sheet_size_mm"] is None
    assert p["name"] is None
    assert p["design"] != m["design"]                         # the kerf shapes the id


def test_build_main_clears_old_outputs_then_stops_on_assembly(monkeypatch, tmp_path, capsys):
    from spiderpig import linkage

    out = tmp_path / "b"
    (out / "laser").mkdir(parents=True)
    (out / "laser" / "old.dxf").write_text("")
    (out / "notes.md").write_text("mine")
    for f in ("manifest.json", "ORDER.md", "bom.csv", "bom.md", "bom.json"):
        (out / f).write_text("stale")

    def no_loop(config):
        raise linkage.AssemblyError("bar b3 short by 1 mm")

    monkeypatch.setattr(build, "template_for", no_loop)
    assert build.main(["--out", str(out)]) == 2
    assert sorted(p.name for p in out.iterdir()) == ["notes.md"]
    assert ("error: the linkage can't be assembled: bar b3 short by 1 mm"
            in capsys.readouterr().err)


def test_build_main_no_plan(monkeypatch, tmp_path, capsys):
    from spiderpig import api

    monkeypatch.setattr(build, "template_for", lambda config: object())

    def no_plan(config, store):
        assert store.root == tmp_path / "store"
        raise ValueError("40 layers ruled out")

    monkeypatch.setattr(api, "plan_config", no_plan)
    rc = build.main(["--out", str(tmp_path / "o"), "--store", str(tmp_path / "store"),
                     "--phases", "0,170"])
    assert rc == 2
    cap = capsys.readouterr()
    assert "error: no layer plan: 40 layers ruled out" in cap.err
    assert "design: strider linkage, leg phases 0,170 deg" in cap.out


def test_export_prints_writes_stls_and_parts_csv(tmp_path):
    import csv

    from build123d import Box, Location

    from spiderpig.hardware.bom import MadeGroup
    from spiderpig.hardware.mass import filament_density

    box = Box(10, 20, 4).moved(Location((5, 7, 30)))      # off the plate, off centre
    ring = SimpleNamespace(name="L.ring", part=box)
    sock = SimpleNamespace(name="L.sock", part=Box(2, 2, 2))
    groups = [MadeGroup("printed", ring, ["L.ring", "R.ring", "L.ring2"], ["R.ring"]),
              MadeGroup("printed", sock, ["L.sock", "R.sock"])]
    fil = {"L.ring": None, "R.ring": None, "L.ring2": None,
           "L.sock": "tpu95a_filament", "R.sock": "petg_filament"}
    rows = build.export_prints(groups, tmp_path / "print", filaments=fil)
    # the socks split per filament; the second takes the full name ("sock" is taken)
    assert sorted(p.name for p in (tmp_path / "print").iterdir()) == [
        "L_sock.stl", "parts.csv", "ring.stl", "ring_mirrored.stl", "sock.stl"]
    ring_row = rows[0]
    assert ring_row["file"] == "ring.stl"
    assert ring_row["qty"] == 3
    assert ring_row["mirrored"] == 1
    assert ring_row["print"] == "print 2, and 1 mirrored (ring_mirrored.stl)"
    assert ring_row["size_mm"] == "10.0 x 20.0 x 4.0"
    assert ring_row["filament"] == ""
    assert ring_row["grams_each_100pct"] == round(0.8 * 1.24, 1)
    petg, tpu = rows[1:]                                # sorted by filament key
    assert (petg["file"], petg["parts"], petg["qty"]) == ("sock.stl", "R.sock", 1)
    assert (tpu["file"], tpu["parts"]) == ("L_sock.stl", "L.sock")
    assert tpu["filament"] == "TPU 95A flexible filament"
    assert tpu["grams_each_100pct"] == round(0.008 * filament_density("tpu95a_filament"), 1)
    with open(tmp_path / "print" / "parts.csv") as f:
        assert [r["file"] for r in csv.DictReader(f)] == [r["file"] for r in rows]
    placed = build._on_plate(box).bounding_box()      # stands on z = 0, centred in XY
    assert pytest.approx((0.0, 0.0, 0.0), abs=1e-9) == (
        placed.min.Z, placed.min.X + placed.max.X, placed.min.Y + placed.max.Y)


def test_export_prints_split_row_names_its_own_filament(tmp_path):
    from build123d import Box

    from spiderpig.hardware.bom import MadeGroup

    sock = SimpleNamespace(name="L.sock", part=Box(2, 2, 2))
    rows = build.export_prints([MadeGroup("printed", sock, ["L.sock", "R.sock"])],
                               tmp_path / "print",
                               filaments={"L.sock": "tpu95a_filament",
                                          "R.sock": "petg_filament"})
    got = {r["parts"]: r["filament"] for r in rows}
    assert set(got) == {"L.sock", "R.sock"}
    assert got["L.sock"] == "TPU 95A flexible filament"
    assert got["R.sock"] == "PETG filament"


def test_export_prints_split_row_names_its_stl_after_its_own_part(tmp_path):
    """The PLA row (L.sock, the group's ref) comes first and takes ``sock.stl``; the TPU
    row's STL is named after its own R.sock, not after the other filament's ref."""
    from build123d import Box

    from spiderpig.hardware.bom import MadeGroup

    sock = SimpleNamespace(name="L.sock", part=Box(2, 2, 2))
    rows = build.export_prints([MadeGroup("printed", sock, ["L.sock", "R.sock"])],
                               tmp_path / "print",
                               filaments={"L.sock": "pla_filament",
                                          "R.sock": "tpu95a_filament"})
    assert {r["parts"]: r["file"] for r in rows} == {"L.sock": "sock.stl",
                                                     "R.sock": "R_sock.stl"}
    assert (tmp_path / "print" / "R_sock.stl").is_file()


def test_export_prints_empty(tmp_path):
    assert build.export_prints([], tmp_path / "p") == []
    assert (tmp_path / "p" / "parts.csv").read_text().strip() == "file"


def test_grid_search_reports_progress(fake_walk, capsys):
    t = tune.Tuner("double", stride_ref=100.0)
    top = tune.grid_search(t, 1.0, 60, 1)
    assert top[0].candidate.phases == (0.0, TARGET)
    assert t.evaluations == 360 - 9                 # within 5 deg of leg 0: infeasible
    assert capsys.readouterr().err == "  grid 250/360\n"


def test_tune_plan_success(monkeypatch, fake_walk):
    from spiderpig import fabricate, walk

    design = SimpleNamespace(plan=SimpleNamespace(top=8))
    monkeypatch.setattr(fabricate, "template_for", lambda config: "tmpl")
    monkeypatch.setattr(fabricate, "design_side", lambda tmpl, config: design)
    monkeypatch.setattr(walk, "foot_z_planned", lambda config, d: [3.0, 6.0])
    t = tune.Tuner("double", stride_ref=100.0)
    p = tune.plan(t, tune.Candidate((0.0, TARGET)))
    assert p["ok"] is True
    assert p["layers"] == 9
    assert p["foot_z"] == [3.0, 6.0]
    assert p["objective"] == pytest.approx(FLOOR)


def test_tune_main_warns_when_nothing_walks(monkeypatch, capsys):
    def still(self, c, n=None, feet_z=None):
        m = _fake_metrics(self, c, n, feet_z)
        return dict(m, stride_mm=0.0, walks=False, degenerate_fraction=1.0)

    monkeypatch.setattr(tune.Tuner, "metrics", still)
    assert tune.main(["--module", "double", "--grid", "180", "--top", "1"]) == 0
    out = capsys.readouterr().out
    assert "note: the default design doesn't walk in this model" in out
    assert "WARNING: the default design does not walk (stride 0.0 mm/rev" in out
    assert "on fewer than three feet 100 % of the cycle" in out
