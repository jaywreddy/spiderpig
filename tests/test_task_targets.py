"""The output's precision and the parts' strength as task targets for every kind
(2026-10-14): the spec's ``precision.*`` targets refused by capability, the budget's rows in
``verify`` and ``api.precision``, a ranged linkage's static loads (:func:`strength.arm_loads`)
and the approved arm's numbers."""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any, cast

import pytest
import sympy as sp

from spiderpig import api, linkage, strength
from spiderpig.api import accuracy
from spiderpig.api.reports import PrecisionReport
from spiderpig.config import BuildConfig
from spiderpig.construction.wobble import Section
from spiderpig.linkage import Linkage, Output, P, crank_at, xy
from spiderpig.linkage import engine as linkage_engine
from spiderpig.repeatability import ACCEL_FACTOR, DRUM_G, HARDWARE_G, TOOL_G, static_loads
from spiderpig.spec import Spec, spec_schema, validate

ARM = "palletizer_reach"
G = 9.81


def _mechanism() -> str:
    """A cranked mechanism with an output (no ranged inputs), not a product (a decanting
    cradle, hand-cranked: no task's candidate, rated on its own statics)."""
    return next(k for k in linkage.available("mechanism")
                if linkage.get(k).output is not None and not linkage.get(k).ranged
                and not linkage.get(k).product)


@pytest.fixture(scope="module")
def toy():
    """A one-link arm: b1 O-F, 100 mm, turned by ``q`` over 0..90 deg."""
    def one(p):
        return [("O", xy(0, 0)), ("F", crank_at(P("O"), p["L"], "q"))]

    lk = Linkage(key="toy_task_arm", name="toy", program=one,
                 params={"L": sp.Integer(100)}, links={"b1": (("O", "F"), (("O", "F"),))},
                 frame=("O",), crank=(), inputs=("q",), ranges={"q": (0.0, 90.0)},
                 drives={"q": ("b1", "O")},
                 output=Output("point", "F", "path", ("O", "F"), body="b1"))
    linkage_engine.REGISTRY[lk.key] = lk
    yield lk
    del linkage_engine.REGISTRY[lk.key]


# -- the spec: a target its linkage can't have is refused, by capability ----------------


@pytest.mark.no_fabricate
def test_a_walker_has_no_output_to_budget():
    errors = validate({"kind": "walker", "linkage": {"key": "strider"},
                       "precision": {"repeatability_mm": {"max": 1.0}}})
    assert [e.path for e in errors] == ["precision.repeatability_mm"]
    assert "has feet, no output" in errors[0].message


@pytest.mark.no_fabricate
def test_a_cranked_mechanism_has_a_budget_but_no_iso_pose_or_payload():
    key = _mechanism()
    ok = {"kind": "mechanism", "linkage": {"key": key},
          "precision": {"repeatability_mm": {"max": 1.0}, "accuracy_mm": {"max": 5.0}}}
    assert validate(ok) == []
    errors = validate({**ok, "precision": {"rp_mm": {"max": 0.1}, "payload_g": 20}})
    assert sorted(e.path for e in errors) == ["precision.payload_g", "precision.rp_mm"]
    assert all("turns a crank" in e.message for e in errors)


@pytest.mark.no_fabricate
def test_an_arm_takes_every_precision_target_and_its_payload():
    doc = {"kind": "arm", "linkage": {"key": ARM},
           "precision": {"rp_mm": {"max": 0.1}, "rp_powerup_mm": {"max": 0.2},
                         "repeatability_mm": {"max": 2.0}, "out_of_plane_mm": {"max": 1.0},
                         "payload_sag_mm": {"max": 1.0}, "accuracy_mm": {"max": 20.0},
                         "payload_g": 30}}
    assert validate(doc) == []
    spec = Spec.from_dict(doc)
    assert spec.payload_g == 30.0
    assert set(spec.precision) == set(doc["precision"]) - {"payload_g"}
    assert Spec.from_dict(spec.to_dict()) == spec
    assert [e.path for e in validate({**doc, "precision": {"payload_g": -1}})] \
        == ["precision.payload_g"]
    assert [e.path for e in validate({**doc, "precision": {"rp": {"max": 1}}})] \
        == ["precision.rp"]
    props = spec_schema()["properties"]["precision"]["properties"]
    assert {"rp_mm", "repeatability_mm", "payload_g"} <= set(props)
    targets = {f.path: t for f, t in spec.targets()}
    assert targets['precision.rp_mm'].hard
    assert not targets['precision.accuracy_mm'].hard


@pytest.mark.no_fabricate
def test_a_spec_without_precision_keeps_its_resolved_form():
    d = api.resolve({"kind": "arm", "linkage": {"key": ARM}}, store=None)
    assert "precision" not in d.resolved              # (an older design keeps its id)
    t = api.resolve({"kind": "arm", "linkage": {"key": ARM},
                     "precision": {"rp_mm": {"max": 0.1}, "payload_g": 20}}, store=None)
    assert t.resolved["precision"] == {"rp_mm": {"max": 0.1, "hard": True}, "payload_g": 20.0}
    assert t.id != d.id


# -- the budget's rows ------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_the_budgets_rows_judge_each_target():
    d = api.resolve({"kind": "arm", "linkage": {"key": ARM},
                     "precision": {"rp_mm": {"max": 0.03}, "accuracy_mm": {"max": 50.0},
                                   "repeatability_mm": {"max": 1.0}}}, store=None)
    rep = PrecisionReport(complete=True, payload_g=50.0,
                          metrics={"rp_mm": 0.04, "rp_powerup_mm": 0.06, "accuracy_mm": 9.0,
                                   "repeatability_mm": None},
                          budget={"singular_poses": 3})
    rows = {r.requirement: r for r in accuracy.rows_of(d, rep)}
    assert not rows['precision.rp_mm'].passed
    assert rows['precision.rp_mm'].hard
    assert rows["precision.accuracy_mm"].passed
    assert rows["precision.rp_powerup_mm"].target is None      # informational
    unbounded = rows["precision.repeatability_mm"]
    assert unbounded.value is None
    assert not unbounded.passed
    assert "singular at 3" in unbounded.detail
    assert "precision.out_of_plane_mm" not in rows              # not measured: no row


@pytest.mark.no_fabricate
def test_a_walker_has_no_precision_rows():
    from spiderpig import verify

    d = api.resolve({"kind": "walker", "linkage": {"key": "strider"}}, store=None)
    assert verify._precision_rows(d, verify.VerifyReport("quick"), "quick") == []
    assert accuracy.measure(d, planar=True).skipped


# -- a ranged linkage's static loads ----------------------------------------------------


@pytest.mark.no_fabricate
def test_the_static_loads_of_one_link(toy):
    """One 100 mm link driven at O: every gram rests on O's pin, the drive holds the
    moment; a crash at 2 N·m pushes the tip with 2 / 0.1 = 20 N, all of it on O."""
    st = static_loads(toy, (), link_g_per_mm=0.1, payload_g=50.0, capacity_nm={"q": 2.0})
    grams = 0.1 * 100 + 2 * HARDWARE_G + DRUM_G + TOOL_G + 50.0
    weight = grams / 1000 * G
    assert st.gravity[("O", "b1")] == pytest.approx(weight, rel=1e-6)
    assert st.operating[("O", "b1")] == pytest.approx(weight * ACCEL_FACTOR, rel=1e-6)
    assert st.crash[("O", "b1")] == pytest.approx(20.0, rel=1e-3)
    assert st.crash_force_n == pytest.approx(20.0, rel=1e-6)
    moment = (0.1 * 100 * 50 + HARDWARE_G * 100 + (TOOL_G + 50.0) * 100) / 1e6 * G   # N·m
    assert st.drive_nm["q"] == pytest.approx(moment * math.cos(math.radians(9.0)), rel=1e-6)
    assert st.singular == 0


@pytest.fixture
def arm_loads(monkeypatch):
    """The approved arm's static loads, its drives' capacity given (no plan)."""
    monkeypatch.setattr(strength, "drive_capacity", lambda config: {"q2": 2.0, "q3": 2.0})
    return strength.design_loads(BuildConfig(linkage=ARM), sim=False)


@pytest.mark.no_fabricate
def test_an_arm_is_rated_at_its_static_loads(arm_loads):
    lo = arm_loads
    assert lo["source"] == "static"
    assert lo["torque_limit_nm"] is None                  # no servo: the drives' current
    assert 'crash' in lo['note']
    assert '50 g payload' in lo['note']
    by = {j["stem"]: j for j in lo["joints"]}
    assert set(by) == set("CEOPRTUVW")                    # every pin and pillar
    assert by['O']['frame']
    assert by['P']['frame']
    assert not by['E']['frame']
    for j in lo["joints"]:
        assert 0 < j["walk"]["n"] < j["jam"]["n"]
        assert j["jam"]["n"] == pytest.approx(j["jam"]["gravity_n"] + j["jam"]["crash_n"],
                                              abs=2e-3)
    light = strength.design_loads(BuildConfig(linkage=ARM), payload_g=0.0)
    assert light["walk_n"] < lo["walk_n"]                  # the payload is in it


def _note(links: dict[str, int]) -> dict:
    ks = sorted(links.values())
    return {"links": [{"link": m} for m in links], "layers": dict(links), "anchors": [],
            "pitch_mm": 3.0, "bearing_len_mm": 3.0, "span_mm": (ks[-1] - ks[0]) * 3.0,
            "section": Section.rod(4.0).as_dict()}


@pytest.mark.no_fabricate
def test_an_arms_joint_takes_its_own_static_load_and_the_fix_is_its_current(arm_loads):
    notes = {"pin:E": _note({"b1": 2, "b4": 6, "b6": 9})}
    cfg = BuildConfig(linkage=ARM)
    st = strength.check(notes, {}, cfg, arm_loads)
    row = next(r for r in st["rows"] if r["joint"] == "pin:E")
    e = next(j for j in arm_loads["joints"] if j["stem"] == "E")
    assert row["basis"] == "static"
    assert row["jam"]["load_n"] == pytest.approx(e["jam"]["n"], abs=0.01)
    assert row["walk"]["load_n"] == pytest.approx(e["walk"]["n"], abs=0.01)
    links = {r["joint"] for r in st["rows"] if r["kind"] == "link"}
    assert {"link:b1", "link:b4"} <= links                # the plates at the same loads
    # a crash 20 times as hard: the fix names the share of the drives' current that holds
    hard = dict(arm_loads, joints=[dict(j, jam=dict(j["jam"], n=j["jam"]["n"] * 20,
                                                    crash_n=j["jam"]["crash_n"] * 20))
                                   for j in arm_loads["joints"]])
    st = strength.check(notes, {}, cfg, hard)
    f = next(f for f in st["findings"] if f["joint"] == "pin:E")
    assert any("holding torque cut to" in x for x in f["fixes"])


@pytest.mark.no_fabricate
def test_verify_fails_an_arms_overloaded_joint(monkeypatch, arm_loads):
    """The static loads are the design's own: an overload fails ``verify``."""
    from spiderpig import verify

    hard = dict(arm_loads, joints=[dict(j, jam=dict(j["jam"], n=j["jam"]["n"] * 100))
                                   for j in arm_loads["joints"]])
    monkeypatch.setattr(strength, "design_loads", lambda *a, **k: hard)
    design = SimpleNamespace(mech=SimpleNamespace(meta={"wobble": {
        "pin:E": _note({"b1": 2, "b4": 6, "b6": 9})}}), config=BuildConfig(linkage=ARM),
        store=None, spec=SimpleNamespace(payload_g=None))
    rep = verify.VerifyReport("standard")
    rows = verify._strength_rows(cast(Any, design), rep, "standard")
    assert rows[0].requirement == 'strength.joints'
    assert not rows[0].passed
    assert rows[0].hard
    assert rows[0].tier == "estimated"
    assert (rep.failures[0].stage, rep.failures[0].code) == ("strength", "joint_overload")


@pytest.mark.no_fabricate
def test_a_cranked_mechanism_is_rated_at_its_own_static_loads():
    """No longer "not rated" (2026-10-14): a cranked mechanism's jam is the arm's crash, its
    output blocked while its servo pushes at its torque limit (tests/test_machines.py)."""
    from spiderpig import verify

    design = SimpleNamespace(mech=SimpleNamespace(meta={}), store=None,
                             config=BuildConfig(linkage=_mechanism(), robot=False),
                             spec=SimpleNamespace(payload_g=None))
    rep = verify.VerifyReport("standard")
    rows = verify._strength_rows(cast(Any, design), rep, "standard")
    assert rows[0].requirement == "strength.joints"
    assert rows[0].value is not None
    assert rows[0].hard
    assert "over a turn of the crank" in rows[0].detail


@pytest.mark.no_fabricate
def test_a_cranked_mechanisms_precision_is_its_whole_budget_unfabricated(tmp_path):
    """``api.precision`` of a cranked mechanism: no links' tilt to read (no ranged inputs),
    so its budget is whole from the plan alone (``complete``), nothing fabricated; its
    targets' rows, no ISO 9283 RP (an arm's), and the report stored."""
    from tests import _api

    _api.seed(_api.HOECKEN_CFG)                 # (the plan from the test cache)
    store = tmp_path / "store"
    d = api.resolve({**_api.HOECKEN, "precision": {"repeatability_mm": {"max": 50.0},
                                                   "accuracy_mm": {"max": 50.0}}}, store)
    rep = api.precision(d)
    assert rep.ok
    assert rep.complete
    assert rep.metrics["repeatability_mm"] > 0
    assert rep.metrics["accuracy_mm"] > 0
    assert "rp_mm" not in rep.metrics
    rows = {r.requirement: r for r in rep.rows}
    assert rows["precision.repeatability_mm"].passed
    assert rows["precision.accuracy_mm"].passed
    assert (store / "designs" / d.id / "precision.json").is_file()
    assert api.precision(api.load(d.id, store)).metrics == rep.metrics   # from the store


# -- the approved arm, end to end -------------------------------------------------------


@pytest.mark.slow     # (fabricates the arm's side for its links' tilt: ~40 s)
def test_the_approved_arms_precision_is_a_stored_report(tmp_path):
    from spiderpig import verify

    store = tmp_path / "store"
    d = api.resolve({"kind": "arm", "linkage": {"key": ARM},
                     "precision": {"rp_mm": {"max": 0.1}, "accuracy_mm": {"max": 50.0}}},
                    store)
    quick = api.verify(d, "quick")
    assert "precision.rp_mm" in quick.unverified          # (it needs the links' tilt)
    assert any(r.requirement == "precision.accuracy_mm" and r.passed for r in quick.rows)
    rep = api.precision(d)
    assert rep.ok
    assert rep.complete
    assert 0 < rep.metrics["rp_mm"] < 0.1 < rep.metrics["repeatability_mm"]
    assert rep.metrics['out_of_plane_mm'] > 0
    assert rep.metrics['payload_sag_mm'] > 0
    assert any("ISO 9283" in line for line in rep.text)
    assert (store / "designs" / d.id / "precision.json").is_file()
    again = api.load(d.id, store)
    assert api.precision(again).metrics == rep.metrics    # from the store
    rows = {r.requirement: r for r in
            verify._precision_rows(again, verify.VerifyReport("quick"), "quick")}
    assert rows["precision.rp_mm"].passed                 # quick reads the stored report


@pytest.mark.slow     # (plans the arm and fabricates its side: ~50 s)
def test_the_approved_arm_holds_its_own_loads():
    from tests import cache

    cfg = BuildConfig(linkage=ARM)
    lo = strength.design_loads(cfg)
    assert lo["capacity_nm"] == {"q2": pytest.approx(2.0), "q3": pytest.approx(2.0)}
    mech = cache.cached_side(cfg, 0.0)
    st = strength.check(mech.meta["wobble"], mech.meta, cfg.side_config(), lo)
    assert not [f for f in st["findings"] if f["level"] == "error"]
    assert {r["basis"] for r in st["rows"] if r["kind"] in ("pin", "pillar")} == {"static"}
    assert st["worst"]["pin"]["jam"]["joint"] == "pin:E"   # the elbow's three links
