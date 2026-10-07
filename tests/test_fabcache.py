"""The fabrication cache (:mod:`spiderpig.fabcache`): served when present, built once under
concurrency, never failing its caller, keyed by everything that shapes a part, cleaned by
the store's gc; and (slow) a loaded fabrication equals a fresh one on the identity gate's
six designs: every part's measurements, the BOM and the DXF entities."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import replace
from pathlib import Path

import pytest

from spiderpig import fabcache
from spiderpig.config import BuildConfig
from spiderpig.mechanism import Body, Mechanism
from spiderpig.store import Store


@pytest.fixture
def on(monkeypatch, tmp_path):
    """The cache on (the suite turns it off), entries by a fixed name (no key computed)."""
    monkeypatch.setenv(fabcache.ENV, "on")
    monkeypatch.setattr(fabcache, "folder_name", lambda: "fab-test-key")
    monkeypatch.setattr(fabcache, "entry_name", lambda tmpl, cfg, d, t: f"entry_t{t!r}")
    return Store.of(tmp_path / "store")


def _mech(n: int = 2) -> Mechanism:
    from build123d import Box

    part = Box(1, 2, 3)
    return Mechanism("m", [Body(f"b{i}", part=part, fab="printed") for i in range(n)]
                     + [Body("abstract")], meta={"k": 1}, bom_extras=["x"])


class _Counter:
    def __init__(self, delay: float = 0.0):
        self.calls, self.delay = 0, delay

    def __call__(self):
        self.calls += 1
        time.sleep(self.delay)
        return _mech()


def test_off_or_without_a_store_it_builds(monkeypatch, tmp_path):
    build = _Counter()
    fabcache.fabricated(None, None, None, None, 1.0, build)
    monkeypatch.setenv(fabcache.ENV, "off")
    fabcache.fabricated(tmp_path, None, None, None, 1.0, build)
    assert build.calls == 2
    assert not (tmp_path / "fab").exists()


def test_served_when_present_and_the_callers_own_object(on):
    build = _Counter()
    first = fabcache.fabricated(on, None, None, None, 1.0, build)
    second = fabcache.fabricated(on, None, None, None, 1.0, build)
    third = fabcache.fabricated(on, None, None, None, 1.0, build)
    assert build.calls == 1
    assert second is not third
    assert second.bodies[0] is not third.bodies[0]
    assert [b.name for b in second.bodies] == [b.name for b in first.bodies]
    assert second.meta == first.meta
    assert second.bom_extras == first.bom_extras
    assert second.bodies[0].part is second.bodies[1].part      # one shape, shared
    assert second.bodies[-1].part is None
    assert abs(second.bodies[0].part.volume - 6.0) < 1e-9
    fabcache.fabricated(on, None, None, None, 2.0, build)       # another entry
    assert build.calls == 2


def test_concurrent_callers_build_once(on):
    build = _Counter(delay=0.5)
    got = []
    threads = [threading.Thread(target=lambda: got.append(
        fabcache.fabricated(on, None, None, None, 1.0, build))) for _ in range(3)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert build.calls == 1
    assert len(got) == 3


def test_a_bad_entry_is_rebuilt_and_a_failed_write_never_fails(on, caplog, monkeypatch):
    build = _Counter()
    fabcache.fabricated(on, None, None, None, 1.0, build)
    entry = on.root / "fab" / "fab-test-key" / "entry_t1.0"
    (entry / "mechanism.pickle").write_bytes(b"not a pickle")
    with caplog.at_level(logging.WARNING, logger="spiderpig.fabcache"):
        fabcache.fabricated(on, None, None, None, 1.0, build)
    assert build.calls == 2
    assert "unreadable" in caplog.text
    assert fabcache.load_mechanism(entry).name == "m"           # written again

    def broken(mech, d):
        raise OSError("disk full")

    monkeypatch.setattr(fabcache, "dump_mechanism", broken)
    with caplog.at_level(logging.WARNING, logger="spiderpig.fabcache"):
        mech = fabcache.fabricated(on, None, None, None, 3.0, build)
    assert mech.name == "m"
    assert "not cached" in caplog.text
    assert not list((on.root / "fab" / "fab-test-key").glob("*.tmp"))


def test_gc_removes_other_keys_and_leftovers(on):
    build = _Counter()
    fabcache.fabricated(on, None, None, None, 1.0, build)
    fab = on.root / "fab"
    (fab / "fab-older-key" / "entry").mkdir(parents=True)
    leftover = fab / "fab-test-key" / ".entry.123.abcd.tmp"
    leftover.mkdir()
    old = time.time() - 7200
    import os

    os.utime(leftover, (old, old))
    gone = on.gc(keep=[])           # (no designs: the cache is cleaned all the same)
    assert gone == []
    assert not (fab / "fab-older-key").exists()
    assert not leftover.exists()
    assert (fab / "fab-test-key" / "entry_t1.0").is_dir()
    entry = fab / "fab-test-key" / "entry_t1.0"
    os.utime(entry, (old, old))
    on.gc(older_than=3600)
    assert not entry.exists()


def test_the_servo_model_in_play_is_in_the_key(monkeypatch):
    cfg = BuildConfig()
    monkeypatch.setenv("SPIDERPIG_SERVO_CAD", "0")
    assert fabcache.servo_state(cfg) == ("parametric",)
    monkeypatch.setenv("SPIDERPIG_SERVO_CAD", "1")      # (offline, empty model cache)
    state = fabcache.servo_state(cfg)
    assert state[0] == "cad"
    assert state[1]
    assert not any(ok for _, ok in state[1])


def test_a_plan_re_made_from_the_store_is_the_solved_plans_entry():
    """The entry is keyed by the plan; the plan a store re-makes (``fabricate._reuse``) and
    the one a solve found are the same entry (the solve's spec names the head search
    that found it, the re-made one the search asked for)."""
    from spiderpig import fabricate

    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False)
    tmpl = fabricate.template_for(cfg)
    _, _, problem = fabricate.side_problem(tmpl, cfg)
    solved = problem.solve()
    _, _, again = fabricate.side_problem(tmpl, cfg)
    remade = fabricate._reuse(again, solved)
    assert remade is not None
    assert fabcache.plan_fingerprint(remade) == fabcache.plan_fingerprint(solved)
    moved = replace(solved, layers={**solved.layers, next(iter(solved.layers)): 99})
    assert fabcache.plan_fingerprint(moved) != fabcache.plan_fingerprint(solved)


# ---------------------------------------------------------------------------
# fidelity (slow): the identity gate's six designs, loaded vs fresh
# ---------------------------------------------------------------------------


def _gate():
    from tests.gate import identity_gate

    return identity_gate


def _dxf_docs(mech, sheet: str, out: Path) -> dict:
    from spiderpig.hardware.bom import group_made
    from spiderpig.layout import save_parts, save_sheets

    save_sheets(mech, out / "laser" / "sheet", default=sheet)
    save_parts(group_made(mech.bodies, "laser"), out / "laser" / "parts", sheet)
    gate = _gate()
    return {str(f.relative_to(out)): gate._dxf_entities(f)
            for f in sorted(out.rglob("*.dxf"))}


@pytest.mark.slow
@pytest.mark.parametrize("name", ["strider_double", "strider_quad", "klann_lego_quad",
                                  "klann_quad", "hoecken_pantograph", "dwell_rocker"])
def test_a_loaded_fabrication_equals_a_fresh_one(name, tmp_path, monkeypatch):
    from spiderpig import build, fabricate
    from spiderpig.hardware.bom import bom_from_mechanism
    from tests import cache

    gate = _gate()
    monkeypatch.setenv(fabcache.ENV, "on")
    cfg = build._parse_args(gate.DESIGNS[name] + ["--out", str(tmp_path)]).config
    cache.cached_design(cfg)                    # the plan (seeded from the test cache)
    tmpl = fabricate.template_for(cfg)
    store = Store.of(tmp_path / "store")
    calls = []
    real = fabricate.fabricate_side
    monkeypatch.setattr(fabricate, "fabricate_side",
                        lambda *a, **kw: calls.append(1) or real(*a, **kw))
    fresh = fabricate.fabricate(tmpl, cfg, 1.0, store=store)
    loaded = fabricate.fabricate(tmpl, cfg, 1.0, store=store)
    assert len(calls) == 1, "the second fabrication wasn't served from the cache"
    assert loaded is not fresh

    def parts(m):
        return [gate._part_doc(b) for b in m.bodies if b.part is not None]

    a, b = parts(fresh), parts(loaded)
    assert [d["name"] for d in a] == [d["name"] for d in b]          # body order
    diffs = [d for x, y in zip(a, b, strict=True) for d in gate.deep_diff(x, y, x["name"])]
    assert not diffs, (len(diffs), diffs[:10])
    assert [b.name for b in fresh.bodies] == [b.name for b in loaded.bodies]
    assert fresh.meta == loaded.meta
    assert bom_from_mechanism(fresh).as_dict() == bom_from_mechanism(loaded).as_dict()
    assert _dxf_docs(fresh, cfg.sheet, tmp_path / "a") == \
        _dxf_docs(loaded, cfg.sheet, tmp_path / "b")
