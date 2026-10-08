"""Joint strength (:mod:`spiderpig.strength`): the bending cases of
:func:`construction.wobble.beam`, the per-design loads (:mod:`spiderpig.sim.loads`), the
findings (warnings and errors with fixes) and where they surface (audit, verify)."""

from __future__ import annotations

import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from spiderpig import strength
from spiderpig.config import BuildConfig
from spiderpig.construction.pivots.chicago import ChicagoShaft, chicago_section
from spiderpig.construction.wobble import (
    Section,
    beam,
    bending_case,
    moment_per_newton,
    stresses,
)
from tests import _strength

_LOADS_REL = 1e-3
"""How far a recorded pin load may move in its currency test (MuJoCo on another machine)."""


def note(layers: dict, anchors=(), section: Section | None = None, pitch: float = 3.0):
    ks = sorted(layers.values())
    return {"links": [{"link": m} for m in layers], "layers": dict(layers),
            "anchors": list(anchors), "pitch_mm": pitch, "bearing_len_mm": pitch,
            "span_mm": (ks[-1] - ks[0]) * pitch,
            "section": (section or Section.rod(3.0)).as_dict()}


# -- the bending cases ------------------------------------------------------------------


@pytest.mark.parametrize("s", [3.0, 6.0, 12.0])
def test_a_two_link_pin_bends_by_half_the_couple(s):
    m, v = beam(np.array([0.0, s]), np.array([[10.0, 0.0], [-10.0, 0.0]]), ("free",))
    assert m == pytest.approx(10.0 * s / 2)
    assert v == pytest.approx(10.0)


def test_a_clevis_is_f_a_b_over_s():
    """The middle link against the outer two, half each: ``F a b / s``."""
    zs = np.array([0.0, 3.0, 9.0])
    f = np.array([[-10.0 * 6 / 9, 0.0], [10.0, 0.0], [-10.0 * 3 / 9, 0.0]])  # balanced
    m, _ = beam(zs, f, ("free",))
    assert m == pytest.approx(10.0 * 3.0 * 6.0 / 9.0)


def test_pillars_are_cantilevers_or_beams_between_the_plates():
    zs = np.array([12.0])
    f = np.array([[10.0, 0.0]])
    m, v = beam(zs, f, ("cantilever", 1.5, 1.0))          # glued in plate 0, face at 1.5
    assert m == pytest.approx(10.0 * 10.5)
    assert v == pytest.approx(10.0)
    m, _ = beam(zs, f, ("simple", 1.5, 22.5))
    assert m == pytest.approx(10.0 * 10.5 * 10.5 / 21.0)


def test_the_case_comes_from_the_layers():
    assert bending_case(note({"a": 2, "b": 3})) == "two-link pin"
    assert bending_case(note({"a": 2, "b": 3, "c": 5})) == "three-link pin"
    assert bending_case(note({"a": 4}, anchors=[0])) == "cantilever pillar"
    assert bending_case(note({"a": 4, "b": 11}, anchors=[0, 17])) == "pillar between plates"
    # the worst pattern of a load's size: two links F against -F, the clevis
    assert moment_per_newton(note({"a": 0, "b": 1}))[0] == pytest.approx(1.5)
    three = note({"a": 0, "b": 1, "c": 2})
    assert moment_per_newton(three)[0] == pytest.approx(2.0)     # outer pair: F L / 3
    clevis = [{"b": (1.0, 0.0), "a": (-0.5, 0.0), "c": (-0.5, 0.0)}]
    assert moment_per_newton(three, clevis)[0] == pytest.approx(1.5)


def test_measured_patterns_give_the_measured_moment():
    """A pattern from the sim (vectors, not one axis) scales with the load."""
    n = note({"a": 0, "b": 4})
    pat = [{"a": (0.6, 0.8), "b": (-0.6, -0.8)}]
    s = stresses(n, 20.0, pat)
    assert s["moment_nmm"] == pytest.approx(20.0 * 12.0 / 2)


def test_a_liner_is_limited_by_its_bearing_pressure():
    """A section with a liner's bearing limit (``Section.bearing_limit_mpa``: the PTFE
    liner's 10 MPa, the removed ``ptfe`` pin's) is rated on it where the shaft holds more."""
    liner = replace(Section.rod(3.0, name="3 mm rod in a PTFE liner"), bearing_limit_mpa=10.0)
    assert liner.as_dict()["bearing_limit_mpa"] == 10.0
    n = note({"a": 0, "b": 1}, section=liner)
    s = stresses(n, 45.0)
    assert s["bearing_mpa"] == pytest.approx(45.0 / 9.0, abs=0.05)
    assert s["governs"] == "liner bearing"
    assert s["safety"] == pytest.approx(10.0 / (45.0 / 9.0), abs=0.01)
    # the same load on the Chicago barrel: the shaft governs, and holds it
    c = stresses(note({"a": 0, "b": 1}, section=chicago_section(ChicagoShaft())), 45.0)
    assert c["governs"] == "shaft"
    assert c["safety"] > s["safety"]


# -- loads ------------------------------------------------------------------------------


def test_the_loads_say_where_they_came_from():
    lo = strength.design_loads(BuildConfig(), sim=False)
    assert lo["source"] == "fallback"
    assert "--no-sim" in lo["note"]
    assert (lo["walk_n"], lo["jam_n"]) == strength.FALLBACK_PIN_LOADS["strider"]
    assert lo["torque_limit_nm"] == pytest.approx(0.85)
    jansen = strength.design_loads(BuildConfig(linkage="jansen"), sim=False)
    assert (jansen["walk_n"], jansen["jam_n"]) == strength.GENERIC_PIN_LOADS
    assert "most loaded" in jansen["note"]
    over = strength.design_loads(BuildConfig(), override=(5.0, 50.0))
    assert over["source"] == "override"
    assert over["jam_n"] == 50.0
    mech = strength.design_loads(BuildConfig(linkage="parallelogram_lift", module="single",
                                             robot=False))
    assert mech["source"] == "none"


def test_a_jam_that_did_not_stall_says_so(monkeypatch):
    """Jam cases that never reached the torque limit under-read the jam loads: the loads
    note says how many and how far the pinned foot drifted."""
    from spiderpig.sim import loads as sim_loads

    joint = {"stem": "J4", "links": ["b3_leg0", "b4_leg0"], "crank": False, "frame": False,
             "walk": {"n": 4.0, "peak": 5.0}, "jam": {"n": 40.0}}
    doc = {"source": "sim", "walk_percentile": 99.0, "walk_seconds": 3.0,
           "torque_limit_nm": 0.85, "jam_cases": 96, "jam_foot_drift_mm": 0.06,
           "jam_stalled": 1.0, "joints": [joint]}
    monkeypatch.setattr(sim_loads, "design_loads", lambda *a, **k: dict(doc))
    ok = strength.design_loads(BuildConfig())
    assert "100% stalled" in ok["note"]
    assert "WARNING" not in ok["note"]
    doc.update(jam_stalled=0.667, jam_foot_drift_mm=41.0)
    short = strength.design_loads(BuildConfig())
    assert short["source"] == "sim"
    assert "WARNING: 32 of 96 jam cases never reached the torque limit" in short["note"]
    assert "41 mm" in short["note"]


def test_a_joint_takes_its_own_sim_loads():
    loads = {"source": "sim", "walk_n": 9.0, "jam_n": 90.0, "joints": [
        {"stem": "J3", "links": ["b1_leg0", "b2_leg0", "b3_leg0"], "frame": False,
         "crank": False, "walk": {"n": 2.0, "peak": 3.0, "patterns": []},
         "jam": {"n": 40.0, "peak": 40.0, "patterns": []}},
        {"stem": "J3", "links": ["b1_leg1", "b2_leg1", "b3_leg1"], "frame": False,
         "crank": False, "walk": {"n": 4.0, "peak": 5.0, "patterns": []},
         "jam": {"n": 45.0, "peak": 45.0, "patterns": []}}]}
    n = note({"b3_leg1": 9, "b2_leg1": 10, "b1_leg1": 11})
    jl = strength.joint_loads("pin:J3_leg1", n, loads)
    assert (jl["walk_n"], jl["jam_n"], jl["basis"]) == (4.0, 45.0, "sim")
    other = strength.joint_loads("pin:J9_leg0", note({"b9_leg0": 1, "b8_leg0": 2}), loads)
    assert other["jam_n"] == 90.0
    assert "largest" in other["basis"]


# -- findings ---------------------------------------------------------------------------


def _check(notes, jam, walk=1.0, crank="bolt"):
    cfg = BuildConfig(crank=crank)
    loads = strength.uniform_loads(walk, jam, "override", "test")
    loads["torque_limit_nm"] = 0.85
    return strength.check(notes, {}, cfg, loads)


def test_warnings_and_errors_name_the_joint_sf_load_and_a_fix():
    notes = {"pin:J4_leg0": note({"b3_leg0": 2, "b4_leg0": 6},
                                 section=chicago_section(ChicagoShaft()))}
    ok = _check(notes, 20.0)
    assert [f for f in ok["findings"] if f["kind"] == "pin"] == []
    row = next(r for r in ok["rows"] if r["joint"] == "pin:J4_leg0")
    assert row["case"] == "two-link pin"
    assert row["span_mm"] == 12.0
    # a jam SF between 1 and 2: a warning; under 1: an error
    sf20 = row["jam"]["safety"]
    warn = _check(notes, 20.0 * sf20 / 1.5)
    f = next(f for f in warn["findings"] if f["kind"] == "pin")
    assert f["level"] == "warning"
    assert f["sf_jam"] == pytest.approx(1.5, abs=0.02)
    assert "pin:J4_leg0" in f["message"]
    assert "jam SF" in f["message"]
    assert " N" in f["message"]
    assert any("shorter span" in x for x in f["fixes"])
    err = _check(notes, 20.0 * sf20 / 0.5)
    f = next(f for f in err["findings"] if f["kind"] == "pin")
    assert f["level"] == "error"
    assert f["sf_jam"] < 1.0
    assert err["findings"][0]["level"] == "error"            # worst first
    # a walking SF under 3 alone is a warning
    walk = _check(notes, 20.0, walk=20.0 * sf20 / 2.5)
    assert any(f["level"] == "warning" and f["kind"] == "pin" and f["sf_walk"] < 3
               for f in walk["findings"])


def test_the_crank_twist_against_each_element_of_the_joint():
    """Every crank is rated element by element: the hex crank's weakest element (the hex in
    its pocket) holds the jam twist at SF 2, the round standoff's (its webs' clamp on the
    crankpin) with a warning's margin; the old 1.8 N·m post-shell figure is gone. (The
    keyed and printed cranks' rows went with them on 2026-10-07.)"""
    bolt = _check({}, 0.0, crank="bolt")
    br = next(r for r in bolt["rows"] if r["kind"] == "crank")
    assert br["factor"] == pytest.approx(2.0, abs=0.05)             # Strider: pins 180° apart
    assert not any("post shell" in k for k in br["capacity_nm"])
    cap = min(br["capacity_nm"].values())
    assert br["jam"]["safety"] == pytest.approx(cap / (br["factor"] * 0.85), abs=0.01)
    # the single aluminium webs' hex standoff crankpin (the default since 2026-10-04): the
    # hex in its 0.100 in 6061 pocket holds the jam twist at SF 2 (no finding)
    assert br["weakest"].startswith("hex 5.5 AF in its plate's pocket")
    assert br["jam"]["safety"] >= 2.0
    assert not any(f["kind"] == "crank" for f in bolt["findings"])
    # the friction-clamped round standoff (--crank bolt_round) stays a warning
    rnd = _check({}, 0.0, crank="bolt_round")
    rr = next(r for r in rnd["rows"] if r["kind"] == "crank")
    assert rr["weakest"].startswith("web clamped")
    assert 1.0 < rr["jam"]["safety"] < 2.0
    bf = next(f for f in rnd["findings"] if f["kind"] == "crank")
    assert bf["level"] == "warning"
    assert "torque limit" in bf["fixes"][0]


def test_errors_fail_the_audit_and_warnings_dont():
    from spiderpig.tools.audit import strength_lines, strength_messages

    notes = {"pin:J4_leg0": note({"b3_leg0": 2, "b4_leg0": 6})}
    st = _check(notes, 400.0, crank="bolt_round")       # (the hex crank holds: no warning)
    errors, warns = strength_messages(st, "error"), strength_messages(st, "warning")
    assert errors
    assert all(e.startswith("strength: pin:J4_leg0") for e in errors)
    assert any("crank" in w for w in warns)
    text = "\n".join(strength_lines(st))
    assert "**ERROR** pin:J4_leg0" in text
    assert "| crank |" in text


def test_verify_fails_an_overloaded_joint(monkeypatch):
    from spiderpig import verify

    notes = {"pin:J4_leg0": note({"b3_leg0": 2, "b4_leg0": 6})}
    design = SimpleNamespace(mech=SimpleNamespace(meta={"wobble": notes}),
                             config=BuildConfig(), store=None, kind="walker")

    def loads(config, store=None, override=None, sim=True):
        assert sim == "cached"                                   # standard: no sim run
        out = strength.uniform_loads(1.0, 400.0, "override", "test")
        out["torque_limit_nm"] = 0.85
        return out

    monkeypatch.setattr(strength, "design_loads", loads)
    rep = verify.VerifyReport("standard")
    rows = verify._strength_rows(design, rep, "standard")
    assert rows[0].requirement == "strength.joints"
    assert not rows[0].passed
    f = rep.failures[0]
    assert (f.stage, f.code) == ("strength", "joint_overload")
    assert f.culprits[0]["joint"] == "pin:J4_leg0"
    assert f.culprits[0]["fixes"]
    assert math.isfinite(f.numbers["sf_jam_min"])

    def family(config, store=None, override=None, sim=True):
        out = strength.uniform_loads(1.0, 400.0, "fallback", "no sim")
        out["torque_limit_nm"] = 0.85
        return out

    monkeypatch.setattr(strength, "design_loads", family)
    rep = verify.VerifyReport("standard")
    rows = verify._strength_rows(design, rep, "standard")
    assert rep.failures == []                   # an estimate: a soft row, verify full decides
    assert not rows[0].passed
    assert not rows[0].hard
    assert rows[0].tier == "estimated"


# -- the sim's loads -----------------------------------------------------------------------


@pytest.mark.slow
@pytest.mark.fixture_regen
def test_the_default_designs_own_loads(tmp_path):
    """MuJoCo: every joint of the Strider double with its walking and jam loads, the jam
    at the torque limit above walking, cached per design in the store. Also the currency
    test of ``tests/fixtures/strength/pin_loads_strider_double.json`` (the fast tests'
    input)."""
    pytest.importorskip("mujoco")
    from spiderpig.sim import loads as sim_loads

    cfg = BuildConfig()
    doc = sim_loads.design_loads(cfg, tmp_path)
    assert cfg == _strength.ORDER_DESIGNS["strider_double"]
    _strength.assert_current("strength", "pin_loads_strider_double",
                             {k: v for k, v in doc.items() if k not in _strength.LOADS_VOLATILE},
                             rel=_LOADS_REL)
    assert doc["source"] == "sim"
    assert doc["torque_limit_nm"] == pytest.approx(0.85)
    pins = [j for j in doc["joints"] if not j["crank"] and not j["frame"]]
    assert {j["stem"] for j in pins} >= {"J3", "J4", "J7", "J8", "J10", "J11"}
    assert all(j["jam"]["n"] >= j["walk"]["peak"] >= j["walk"]["n"] > 0 for j in pins)
    assert 10 < max(j["jam"]["n"] for j in pins) < 200
    assert max(j["walk"]["n"] for j in pins) < 30
    pillars = [j for j in doc["joints"] if j["frame"]]
    assert {tuple(j["links"]) for j in pillars} >= {("b1_leg0", "b1_leg1")}
    # every jam case stalls against a pinned foot that stays put (a soft pin let a third
    # of them turn on through it, under-reading leg0's joints)
    assert doc["jam_stalled"] == 1.0
    assert doc["jam_unstalled"] == []
    assert doc["jam_foot_drift_mm"] < 1.0
    # the two legs are mirror images: their jam loads agree
    by = {}
    for j in pins:
        by.setdefault(j["stem"], []).append(j["jam"]["n"])
    for stem, ns in by.items():
        assert len(ns) == 2, (stem, ns)
        assert max(ns) <= 1.1 * min(ns), (stem, ns)
    assert sim_loads.cache_path(cfg, tmp_path).exists()
    again = sim_loads.design_loads(cfg, tmp_path, cached_only=True)
    assert again == doc


@pytest.mark.slow
def test_the_jam_isolates_the_caught_foot_from_the_floor(monkeypatch):
    """The jam runs with the floor's contacts off (``sim.loads.JAM_FLOOR``): with the base
    welded and one foot pinned, the other feet resting on the floor would take part of the
    stalled torque. The model the jam steps has no contact at all, and the result says so."""
    pytest.importorskip("mujoco")
    import mujoco

    from spiderpig.sim import loads

    assert loads.VERSION >= 3
    assert loads.JAM_FLOOR is False
    seen: list[int] = []
    step = mujoco.mj_step

    def counting(model, data, *a):
        step(model, data, *a)
        seen.append(int(data.ncon))

    monkeypatch.setattr(mujoco, "mj_step", counting)
    cfg = BuildConfig(linkage="klann", module="single")
    jam = loads.jam_loads(cfg, angles=2, steps=30, average=10)
    assert jam["floor"] is False
    assert seen
    assert max(seen) == 0


# -- the recorded inputs: the order designs' meta and pin loads ----------------------------


@pytest.mark.slow
@pytest.mark.fixture_regen
@pytest.mark.parametrize("name", [n for n in _strength.ORDER_DESIGNS if n != "strider_double"])
def test_the_order_designs_pin_loads_current(name, tmp_path):
    """MuJoCo: the quads' recorded pin loads (the Strider double's:
    :func:`test_the_default_designs_own_loads`) are still what the sim gives."""
    pytest.importorskip("mujoco")
    doc = _strength.make_pin_loads(name, tmp_path)
    assert doc["source"] == "sim"
    _strength.assert_current("strength", f"pin_loads_{name}", doc, rel=_LOADS_REL)


@pytest.mark.slow
@pytest.mark.fixture_regen
@pytest.mark.parametrize("name", list(_strength.ORDER_DESIGNS))
def test_the_order_designs_meta_current(name, monkeypatch):
    """The recorded ``meta`` is the robot's (the fabrication cache's, one build per engine),
    and the strength check reads it as it reads the built robot's own (tuples and all)."""
    from spiderpig import strength
    from tests import cache

    mech = cache.cached_robot(_strength.ORDER_DESIGNS[name])
    data = _strength.assert_current("strength", f"meta_{name}", mech.meta)
    cfg = _strength.ORDER_DESIGNS[name]
    loads = _strength.strength_loads(name, monkeypatch)
    live = strength.check(mech.meta.get("wobble") or {}, mech.meta, cfg, loads)
    assert strength.check(data["wobble"], data, cfg, loads) == cache._jsonable(live)


def _order_check(name: str, monkeypatch, doc: dict | None = None):
    """The audit's strength check of an order design on its recorded ``meta`` and pin
    loads (``doc``: other pin loads)."""
    meta = _strength.meta(name)
    loads = _strength.strength_loads(name, monkeypatch, doc)
    cfg = _strength.ORDER_DESIGNS[name]
    return meta, loads, strength.check(meta["wobble"], meta, cfg, loads)


@pytest.mark.parametrize("name", list(_strength.ORDER_DESIGNS))
def test_the_order_designs_joints_hold_their_own_loads(name, monkeypatch):
    """Every pivot of an order design rated at its own sim joint's loads (the wobble notes'
    names and the sim's joints agree), the crank at the torque limit on its hex pockets,
    every loaded link plate at its own sheet; no joint under the audit's error limit."""
    from spiderpig.config import torque_limit_nm
    from spiderpig.construction.crank import BoltCrank
    from spiderpig.materials import link_sheets
    from spiderpig.tools.audit import strength_messages

    cfg = _strength.ORDER_DESIGNS[name]
    meta, loads, st = _order_check(name, monkeypatch)
    assert loads["source"] == "sim"
    assert "MuJoCo" in loads["note"]
    assert "WARNING" not in loads["note"]
    rows = {r["joint"]: r for r in st["rows"]}
    notes = meta["wobble"]
    assert {j for j in rows
            if not j.startswith(("crank", "link:", "centre_plates"))} == set(notes)
    for joint, note in notes.items():
        r = rows[joint]
        assert r["basis"] == "sim", joint                 # its own sim joint's loads
        assert r["links"] == [e["link"] for e in note["links"]]
        assert r["jam"]["load_n"] >= r["walk"]["load_n"] > 0, joint
        assert r["jam"]["safety"] <= r["walk"]["safety"], joint
    assert {r["kind"] for r in st["rows"]} == {"pin", "pillar", "crank", "link", "chassis"}
    assert rows["centre_plates"]["jam"]["safety"] > strength.JAM_WARN
    crank = rows["crank"]
    assert crank["construction"] == cfg.crank
    assert isinstance(BoltCrank().for_sheet(cfg.crank_sheet), BoltCrank)
    assert crank["weakest"].startswith("hex 5.5 AF in its plate's pocket")
    assert crank["factor"] == pytest.approx(loads["joint_moment_factor"])
    assert crank["jam"]["torque_nm"] == pytest.approx(torque_limit_nm(cfg))
    cap = min(crank["capacity_nm"].values())
    assert crank["jam"]["safety"] == pytest.approx(cap / crank["jam"]["moment_nm"], abs=0.01)
    sheets = link_sheets(cfg)
    for r in (r for r in st["rows"] if r["kind"] == "link"):
        key = r["links"][0]
        assert r["sheet"] == sheets.get(key, cfg.sheet), key
    assert strength_messages(st, "error") == []
    assert all(f["fixes"] for f in st["findings"])
    for kind, worst in st["worst"].items():
        assert worst["jam"]["safety"] >= strength.JAM_ERROR, kind


@pytest.mark.parametrize("name", list(_strength.ORDER_DESIGNS))
def test_an_order_design_overloaded_fails_with_fixes(name, monkeypatch):
    """The same design with every joint's jam load scaled until its weakest pin is at SF
    0.5: that pin is an error naming itself, its load and fixes; each other construction a
    fix offers rates it higher than now."""
    import copy
    import re

    from spiderpig.tools.audit import strength_lines, strength_messages

    _, _, st = _order_check(name, monkeypatch)
    weakest = st["worst"]["pin"]["jam"]
    scale = weakest["safety"] / 0.5
    doc = copy.deepcopy(_strength.pin_loads(name))
    for j in doc["joints"]:
        j["jam"]["n"] *= scale
    _, _, hot = _order_check(name, monkeypatch, doc)
    f = next(f for f in hot["findings"] if f["joint"] == weakest["joint"])
    assert f["level"] == "error"
    assert f["sf_jam"] == pytest.approx(0.5, abs=0.02)
    assert hot["findings"][0]["level"] == "error"                       # worst first
    assert f"{weakest['joint']} (" in f["message"]
    assert "jam SF" in f["message"]
    assert f["fixes"]
    for fix in f["fixes"]:
        m = re.match(r"--(?:pin|pillar) \S+ \(.*?\): jam SF ([\d.]+)", fix)
        if m:
            assert float(m.group(1)) > f["sf_jam"], fix
    errors = strength_messages(hot, "error")
    assert any(e.startswith(f"strength: {weakest['joint']} ") for e in errors)
    assert f"**ERROR** {weakest['joint']}" in "\n".join(strength_lines(hot))


@pytest.mark.parametrize("name", list(_strength.ORDER_DESIGNS))
def test_the_order_designs_audit_report(name, monkeypatch):
    """The audit's wobble, strength, crank and Chicago sections of an order design, from
    its recorded ``meta`` and pin loads (:func:`tools.audit.markdown`)."""
    from spiderpig.tools.audit import markdown, wobble_check

    meta, loads, st = _order_check(name, monkeypatch)
    wob = wobble_check(meta["wobble"], (loads["walk_n"], loads["jam_n"]))
    for kind in ("pin", "pillar"):
        row = wob[kind]
        assert row["joints"] == sum(1 for j in meta["wobble"] if j.startswith(kind + ":"))
        assert 0 <= row["mean_deg"] <= row["worst_deg"] <= row["worst_free_deg"]
        assert row["jam"]["safety"] <= row["walk"]["safety"]
    assert wob["pin"]["worst_deg"] < 2.0                   # Chicago pins: under the warning
    assert wob["warnings"] == []
    rep = {"layers": meta["layers"], "parts": 0, "plan": "(recorded)", "plan_violations": [],
           "contract": {"t=1": []}, "clash": {"t=1": []}, "solids": {"t=1": []},
           "dxf_sheets": 0, "bom": {}, "snap": {"joints": 0, "worst_pct": None,
                                                "max_pct": None, "relieved": {},
                                                "problems": []},
           "wobble": wob, "strength": st, "chicago": meta["chicago"],
           "crank_bolt": meta["crank_bolt"], "chassis": {}, "seconds": 0.0,
           "problems": [], "warnings": []}
    md = markdown({"config": {"linkage": _strength.ORDER_DESIGNS[name].linkage},
                   "modules": {name: rep}})
    assert f"| {name} | {meta['layers']} |" in md
    assert "| OK |" in md
    for r in st["rows"]:
        if r["kind"] in ("pin", "pillar"):
            assert f"| {r['joint']} | {', '.join(r['links'])} | {r['case']} |" in md
    assert "| crank | crankpins (bolt) |" in md
    assert f"Crank (per side): {meta['crank_bolt']['plates']} single aluminium plates" in md
    assert "hex standoff in hex pockets" in md
    assert "Chicago screws (per side): " in md
    assert f"Wobble, pins: worst {wob['pin']['worst_deg']:.2f} deg" in md
