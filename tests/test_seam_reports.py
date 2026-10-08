"""Seam tests of the reports around the strength check, on hand-made inputs: the pin loads'
bookkeeping (:mod:`spiderpig.sim.loads`: joint names, the joint index of a tiny MJCF, the
per-joint summary, the sides merged, the store's cache read and written without
simulating), the audit's report formatting and checks (:mod:`spiderpig.tools.audit`:
cut rules, strength lines, snap, wobble, safety-factor cells, the markdown), the strength
levels and loads (:mod:`spiderpig.strength`) and verify's row helpers
(:mod:`spiderpig.verify`). No design is fabricated and MuJoCo never steps
(``no_fabricate``)."""

from __future__ import annotations

import json
import re
import warnings
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from spiderpig import manufacture, materials, strength
from spiderpig import verify as vf
from spiderpig.config import BuildConfig
from spiderpig.construction.wobble import Section
from spiderpig.failure import Failure
from spiderpig.sim import loads as sl
from spiderpig.spec import Target
from spiderpig.tools import audit
from spiderpig.verify import (
    Row,
    VerifyReport,
    _cost_item,
    _cut_rule_rows,
    _fail,
    _push,
    _stage_row,
    _unpriced_item,
    fall_detail,
    least_transmission_angle,
    mass_by_group,
)

try:            # imported with the module, not in a test's time (MuJoCo's import is ~0.4 s)
    import mujoco
except ImportError:
    mujoco = None

pytestmark = pytest.mark.no_fabricate


# -- sim.loads: names ---------------------------------------------------------------------


@pytest.mark.parametrize(("name", "stem"), [
    ("L.J3_leg0#1", "J3"), ("R.J7_leg12", "J7"), ("J2", "J2"), ("L.J2#4", "J2"),
    ("X.J2_leg0", "X.J2"),            # only an L. / R. side prefix is stripped
    ("L.J3_legA", "J3_legA"),         # the leg suffix is digits only
])
def test_stem_strips_side_leg_and_loop_index(name, stem):
    assert sl._stem(name) == stem


def test_side_free_strips_only_a_leading_side():
    assert sl._side_free("L.b1_leg0") == "b1_leg0"
    assert sl._side_free("R.foot") == "foot"
    assert sl._side_free("base") == "base"
    assert sl._side_free("b1.L.x") == "b1.L.x"


# -- sim.loads: the joint index of a tiny model -------------------------------------------

_MJCF = """
<mujoco>
  <option gravity="0 -9.81 0"/>
  <worldbody>
    <body name="base" pos="0 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="1e-3 1e-3 1e-3"/>
      <body name="L.b1_leg0" pos="0.01 0.02 0">
        <joint name="h1" type="hinge" axis="0 0 1"/>
        <inertial pos="0.01 0 0" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>
        <body name="L.b2_leg0" pos="0.02 -0.02 0">
          <joint name="h2" type="hinge" axis="0 0 1"/>
          <inertial pos="0.01 0 0" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>
        </body>
      </body>
      <body name="L.crank" pos="0 0 0">
        <joint name="hc" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="0.02" diaginertia="1e-5 1e-5 1e-5"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <connect name="L.J3_leg0#1" body1="L.b2_leg0" body2="L.crank" anchor="0.01 0 0"/>
    <connect name="jam.L.foot" body1="L.b2_leg0" anchor="0 0 0" active="false"/>
    <connect body1="L.b1_leg0" anchor="0 0 0" active="false"/>
    <connect name="L.J5" body1="base" body2="L.crank" anchor="0 0 0"/>
    <connect name="L.J2_leg0#1" body1="L.b1_leg0" body2="L.b2_leg0" anchor="0.02 -0.02 0"/>
    <connect name="L.J3_leg0#2" body1="L.b2_leg0" body2="L.crank" anchor="0.01 0 0"
             active="false"/>
    <connect name="L.J6" body1="L.b1_leg0" body2="L.b2_leg0" anchor="0.03 0 0"/>
  </equality>
</mujoco>
"""

_META = {"bodies": {
    "base": {"parent": None, "kind": "base", "side": "", "pivot": "", "ref_pos": [0, 0, 0]},
    "L.b1_leg0": {"parent": "base", "kind": "link", "side": "L", "pivot": "J1",
                  "ref_pos": [0.01, 0.02, 0.0]},
    "L.b2_leg0": {"parent": "L.b1_leg0", "kind": "link", "side": "L", "pivot": "J2",
                  "ref_pos": [0.03, 0.0, 0.0]},
    "L.crank": {"parent": "base", "kind": "crank", "side": "L", "pivot": "O",
                "ref_pos": [0.0, 0.0, 0.0]},
}}


@pytest.fixture(scope="module")
def tiny():
    if mujoco is None:
        pytest.skip("MuJoCo isn't installed")
    model = mujoco.MjModel.from_xml_string(_MJCF)
    return mujoco, model


def test_joint_index_groups_hinges_and_loops_per_side_stem_and_point(tiny):
    """A hinge is a joint at its child's reference point (the crank's skipped: its torque is
    the drive's); a connect loop one at body1's point plus its anchor, named by its stem;
    the jam's equalities and unnamed ones are not joints, a loop on a hinge's point and stem
    joins it, and one between base and crank alone has no link. Rows: one per (joint,
    link), base and crank out."""
    _, model = tiny
    idx = sl.JointIndex(model, _META)
    assert [(g["side"], g["stem"], g["xy_mm"], g["links"], g["frame"], g["crank"])
            for g in idx.joints] == [
        ("L", "J1", [10.0, 20.0], ["L.b1_leg0"], True, False),
        ("L", "J2", [30.0, 0.0], ["L.b1_leg0", "L.b2_leg0"], False, False),
        ("L", "J3", [40.0, 0.0], ["L.b2_leg0"], False, True),
        ("L", "J6", [40.0, 20.0], ["L.b1_leg0", "L.b2_leg0"], False, False),
    ]
    assert idx.rows == [(0, "L.b1_leg0"), (1, "L.b1_leg0"), (1, "L.b2_leg0"),
                        (2, "L.b2_leg0"), (3, "L.b1_leg0"), (3, "L.b2_leg0")]
    # J1's parent is the base (no row), J2's child row 2 against its parent row 1
    assert [(rc, rp) for _, rc, rp in idx.hinges] == [(0, -1), (2, 1)]
    assert [(rc, rp) for _, rc, rp in idx.loops] == [(1, 2), (3, -1), (3, -1), (4, 5)]


def test_joint_index_forces_are_equal_and_opposite_across_a_hinge(tiny):
    """The force a hinge (and a loop) puts on one link is the opposite of what the other
    takes, in the base's plane (no stepping: one forward pass under in-plane gravity); an
    inactive loop adds nothing."""
    mujoco, model = tiny
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    idx = sl.JointIndex(model, _META)
    f = idx.forces(model, data)
    assert f.shape == (6, 2)
    np.testing.assert_allclose(f[1], -f[2], atol=1e-12)
    np.testing.assert_allclose(f[4], -f[5], atol=1e-12)     # the J6 loop
    assert np.hypot(*f[4]) > 1e-6
    assert np.hypot(*f[0]) > 0.5           # J1 carries both links' weight (~0.98 N) and more


# -- sim.loads: the per-joint summary and the sides merged --------------------------------


def _view(joints, rows):
    return sl._SideFree(joints, rows)


def test_summary_takes_the_percentile_peak_and_heaviest_patterns():
    view = _view([{"stem": "J1"}, {"stem": "J2"}],
                 [(0, "L.a"), (0, "L.b"), (1, "L.c")])
    s = np.zeros((4, 3, 2))
    s[:, 0, 0] = [1.0, 2.0, 3.0, 4.0]      # joint 0's link a
    s[:, 1, 1] = [0.5, 5.0, 0.5, 0.5]      # its link b: step 1 is its heaviest
    out = sl._summary(view, s, None)       # None: the largest
    assert out[0]["n"] == 5.0
    assert out[0]["peak"] == 5.0
    pats = out[0]["patterns"]
    assert len(pats) == 4                                   # every step, heaviest first
    assert pats[0] == {"a": [0.4, 0.0], "b": [0.0, 1.0]}   # step 1 over its 5 N, side-free
    assert pats[1] == {"a": [1.0, 0.0], "b": [0.0, 0.125]}
    # joint 1 never loaded: no load, no pattern (a zero step ends the patterns)
    assert out[1] == {"n": 0.0, "peak": 0.0, "patterns": []}
    p50 = sl._summary(view, s, 50.0)
    assert p50[0]["n"] == pytest.approx(3.5)                # median of 2, 3, 4, 5
    assert p50[0]["peak"] == 5.0


def test_summary_keeps_at_most_patterns_and_handles_no_samples():
    view = _view([{"stem": "J1"}], [(0, "L.a")])
    s = np.zeros((10, 1, 2))
    s[:, 0, 0] = np.arange(1.0, 11.0)
    out = sl._summary(view, s, None)
    assert len(out[0]["patterns"]) == sl.PATTERNS
    assert out[0]["patterns"][0] == {"a": [1.0, 0.0]}
    empty = sl._summary(view, np.zeros((0, 1, 2)), 99.0)
    assert empty == [{"n": 0.0, "peak": 0.0, "patterns": []}]


def test_merge_sides_appends_the_right_samples_to_their_left_twins():
    joints = [{"side": "L", "stem": "J1", "links": ["L.a", "L.b"]},
              {"side": "R", "stem": "J1", "links": ["R.a", "R.b"]},
              {"side": "R", "stem": "J9", "links": ["R.z"]}]      # no left twin: dropped
    rows = [(0, "L.a"), (0, "L.b"), (1, "R.a"), (1, "R.b"), (2, "R.z")]
    idx = SimpleNamespace(joints=joints, rows=rows)
    s = np.arange(2 * 5 * 2, dtype=float).reshape(2, 5, 2)
    view, merged = sl._merge_sides(idx, s)
    assert [g["links"] for g in view.joints] == [["a", "b"]]
    assert view.rows == [(0, "a"), (0, "b")]
    assert merged.shape == (4, 2, 2)
    np.testing.assert_array_equal(merged[:2], s[:, 0:2])        # the left's own
    np.testing.assert_array_equal(merged[2:], s[:, 2:4])        # the right's, appended
    # one side only (no left): its joints stand for both
    only_r = SimpleNamespace(joints=joints[1:2], rows=[(0, "R.a"), (0, "R.b")])
    view, merged = sl._merge_sides(only_r, s[:, 2:4])
    assert view.rows == [(0, "a"), (0, "b")]
    np.testing.assert_array_equal(merged, s[:, 2:4])


# -- sim.loads: the store's cache ---------------------------------------------------------


def test_cache_path_is_the_robots_key_under_the_store(tmp_path):
    cfg = BuildConfig()
    p = sl.cache_path(cfg, tmp_path)
    assert p == tmp_path / "pin_loads" / f"{cfg.key}.json"
    # a side asks for the robot's loads: the same file
    assert sl.cache_path(replace(cfg, robot=False), tmp_path) == p


@pytest.fixture
def no_sim(monkeypatch):
    """design_loads' simulation refused, the source version pinned (no package hash)."""
    pytest.importorskip("mujoco")
    from spiderpig import design

    monkeypatch.setattr(design, "source_version", lambda: "test+src.1")

    def boom(config):
        raise AssertionError("simulate_loads ran")

    monkeypatch.setattr(sl, "simulate_loads", boom)


def test_design_loads_reads_a_current_cache_without_simulating(tmp_path, no_sim):
    cfg = BuildConfig()
    path = sl.cache_path(cfg, tmp_path)
    path.parent.mkdir(parents=True)
    doc = {"version": sl.VERSION, "source_version": "test+src.1", "joints": [], "x": 7}
    path.write_text(json.dumps(doc))
    assert sl.design_loads(cfg, tmp_path) == doc
    assert sl.design_loads(cfg, tmp_path, cached_only=True) == doc


@pytest.mark.parametrize("text", [
    json.dumps({"version": sl.VERSION, "source_version": "old"}),     # sources changed
    json.dumps({"version": sl.VERSION - 1, "source_version": "test+src.1"}),
    "{not json",
])
def test_design_loads_cached_only_ignores_a_stale_or_broken_cache(tmp_path, no_sim, text):
    cfg = BuildConfig()
    path = sl.cache_path(cfg, tmp_path)
    path.parent.mkdir(parents=True)
    path.write_text(text)
    assert sl.design_loads(cfg, tmp_path, cached_only=True) is None
    with pytest.raises(AssertionError, match="simulate_loads ran"):
        sl.design_loads(cfg, tmp_path)


def test_design_loads_cached_only_without_a_cache_is_none(tmp_path, no_sim):
    assert sl.design_loads(BuildConfig(), tmp_path, cached_only=True) is None
    assert not (tmp_path / "pin_loads").exists()


def test_design_loads_refresh_simulates_stamps_writes_and_warns(tmp_path, no_sim,
                                                                monkeypatch):
    """``refresh`` re-simulates over a current cache; the result is stamped with the
    sources' version, written atomically, and a jam that didn't all stall warns."""
    cfg = BuildConfig()
    path = sl.cache_path(cfg, tmp_path)
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({"version": sl.VERSION, "source_version": "test+src.1"}))
    fresh = {"version": sl.VERSION, "jam_stalled": 0.5, "jam_foot_drift_mm": 3.0}
    monkeypatch.setattr(sl, "simulate_loads", lambda config: dict(fresh))
    with pytest.warns(RuntimeWarning, match=r"only 50% of the jam cases stalled \(pinned "
                                            r"foot drifted up to 3 mm\)"):
        doc = sl.design_loads(cfg, tmp_path, refresh=True)
    assert doc == dict(fresh, source_version="test+src.1")
    assert json.loads(path.read_text()) == doc
    assert [p.name for p in path.parent.iterdir()] == [path.name]      # no tmp left
    fresh["jam_stalled"] = 1.0
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        sl.design_loads(cfg, tmp_path, refresh=True)


# -- tools.audit: the fallback loads and cut rules ----------------------------------------


def test_pin_loads_for_takes_the_family_or_none():
    assert audit.pin_loads_for("strider") == (6.4, 38.0)
    assert audit.pin_loads_for("klann") == (119.0, 155.0)
    assert audit.pin_loads_for("klann_lego") == (119.0, 155.0)      # its family's
    assert audit.pin_loads_for("jansen") is None                     # nobody measured it
    assert audit.pin_loads_for("no_such_linkage") is None


def _issue(rule, part, level="warning", **kw):
    return dict({"rule": rule, "part": part, "sheet": "al5052_2mm", "level": level,
                 "detail": f"{part} detail"}, **kw)


_CUTS = {"sheets": {"al5052_2mm": "sendcutsend", "acrylic_3mm": "ponoko"}, "parts": 12,
         "errors": {"edge": 2},
         "issues": [_issue("web", "p1"),
                    _issue("edge", "p2", "error", why="the web distorts", fix="move it"),
                    _issue("edge", "p3", "error"),
                    _issue("edge", "p4", why="w4", fix="f4"),
                    _issue("odd_rule", "p5")]}


def test_manufacture_messages_one_per_rule_at_its_level_worst_first_listed():
    assert audit.manufacture_messages(_CUTS, "error") == [
        "manufacture: 2 part(s) break the hole-to-edge distance rule; worst p2 "
        "(al5052_2mm): p2 detail; the web distorts (fix: move it)"]
    assert audit.manufacture_messages(_CUTS) == [
        "manufacture: 1 part(s) break the hole-to-edge distance rule; worst p4 "
        "(al5052_2mm): p4 detail; w4 (fix: f4)",
        "manufacture: 1 part(s) break the odd_rule rule; worst p5 (al5052_2mm): p5 detail",
        "manufacture: 1 part(s) break the web round a cut-out rule; worst p1 "
        "(al5052_2mm): p1 detail"]
    assert audit.manufacture_messages({"issues": []}, "error") == []


def test_cut_cell_counts_errors_and_warnings():
    assert audit.cut_cell({}) == "-"
    assert audit.cut_cell({"manufacture": {}}) == "-"
    assert audit.cut_cell({"manufacture": {"issues": [], "errors": {}}}) == "ok"
    assert audit.cut_cell({"manufacture": _CUTS}) == "2 err, 3 warn"
    assert audit.cut_cell({"manufacture": {"issues": [_issue("web", "p")],
                                           "errors": None}}) == "0 err, 1 warn"


def test_cut_lines_header_dxf_and_a_row_per_issue_errors_first():
    lines = audit.cut_lines(_CUTS)
    assert lines[0].startswith("Cut rules (al5052_2mm: sendcutsend, acrylic_3mm: ponoko; "
                               "12 laser-cut parts): a hole closer than 1 x the thickness")
    assert lines[1:3] == ["", "| level | rule | part | sheet | what | why | fix |"]
    body = lines[4:]
    assert [r.split(" | ")[0] for r in body] == ["| error"] * 2 + ["| warning"] * 3
    assert body[0] == ("| error | hole-to-edge distance | p2 | al5052_2mm | p2 detail "
                       "| the web distorts | move it |")
    assert body[-1] == "| warning | odd_rule | p5 | al5052_2mm | p5 detail |  |  |"
    clean = dict(_CUTS, issues=[], dxf_worst_mm=0.01234, dxf={"a": 1, "b": 2},
                 kerf={"sendcutsend": 0.0, "ponoko": 0.2})
    lines = audit.cut_lines(clean)
    assert lines[1].startswith("DXF contours against the solids: at most 0.0123 mm off "
                               "over 2 parts")
    assert lines[1].endswith("kerf per sheet: sendcutsend 0 mm, ponoko 0.2 mm.")
    assert lines[-2:] == ["", "Every part passes."]


# -- tools.audit: strength ----------------------------------------------------------------


def _finding(level, msg, *fixes):
    return {"level": level, "message": msg, "fixes": list(fixes)}


def test_strength_messages_keep_one_level_and_the_first_fix():
    st = {"findings": [_finding("error", "pin:J1 jam SF 0.5", "fix A", "fix B"),
                       _finding("warning", "crank jam SF 1.5", "lower torque"),
                       _finding("error", "pillar:J2 jam SF 0.9", "fix C")]}
    assert audit.strength_messages(st, "error") == [
        "strength: pin:J1 jam SF 0.5 (fix: fix A)",
        "strength: pillar:J2 jam SF 0.9 (fix: fix C)"]
    assert audit.strength_messages(st, "warning") == [
        "strength: crank jam SF 1.5 (fix: lower torque)"]
    assert audit.strength_messages({"findings": []}, "error") == []


def test_sf_cell_jam_then_walk():
    def rep(**worst):
        return {"strength": {"worst": worst}}

    assert audit.sf_cell({}, "pin") == "-"
    assert audit.sf_cell(rep(), "pin") == "-"
    assert audit.sf_cell(rep(pin={"jam": {"safety": 1.5}, "walk": {"safety": 3.25}}),
                         "pin") == "1.5 / 3.25"
    assert audit.sf_cell(rep(pin={"jam": {"safety": 2.0}}), "pin") == "2"
    assert audit.sf_cell(rep(crank={"walk": {"safety": 7.0}}), "crank") == "- / 7"
    assert audit.sf_cell(rep(pin={"jam": {"safety": 2.0}}), "pillar") == "-"


_ST = {
    "loads": {"note": "the design's own", "source": "sim"},
    "rows": [
        {"kind": "pin", "joint": "pin:J4_leg0", "links": ["b3_leg0", "b4_leg0"],
         "case": "two-link pin", "span_mm": 12.0, "section": "chicago",
         "walk": {"load_n": 4.0, "safety": 9.5}, "jam": {"load_n": 40.0, "safety": 1.2}},
        {"kind": "pillar", "joint": "pillar:J2", "links": ["b2_leg0"],
         "case": "cantilever pillar", "span_mm": 7.5, "section": "standoff",
         "walk": None, "jam": None},
        {"kind": "crank", "construction": "bolt", "factor": 2.0, "weakest": "hex",
         "capacity_nm": {"hex": 3.5, "torsion": 9.0},
         "walk": {"torque_nm": 0.2, "safety": 8.75}, "jam": {"torque_nm": 0.85,
                                                           "safety": 2.06}},
        {"kind": "link", "joint": "link:b1", "pins": ["J1", "J3"], "bending": False,
         "sheet": "acrylic_3mm", "thickness_mm": 3.0, "needs": {"sheet": "al6061"},
         "walk": {"load_n": 5.0, "safety": 6.0}, "jam": {"load_n": 50.0, "safety": 1.5}},
        {"kind": "link", "joint": "link:b2", "pins": ["J1", "J2", "J3"], "bending": True,
         "sheet": "acrylic_3mm", "thickness_mm": 3.0, "needs": None,
         "walk": None, "jam": None},
    ],
    "findings": [_finding("warning", "pin:J4_leg0 jam SF 1.2", "a", "b")],
}


def test_strength_lines_one_row_per_joint_crank_and_link():
    lines = audit.strength_lines(_ST)
    assert lines[0] == ("Joint strength (jam SF under 1 fails, under 2 jammed or 3 walking "
                        "warns). Loads: the design's own.")
    assert lines[2].startswith("| joint | links | case |")
    assert lines[4:9] == [
        "| pin:J4_leg0 | b3_leg0, b4_leg0 | two-link pin | 12 | chicago | 4.0 | 9.5 "
        "| 40.0 | 1.2 |",
        "| pillar:J2 | b2_leg0 | cantilever pillar | 7.5 | standoff | - | - | - | - |",
        "| crank | crankpins (bolt) | twist x2 | - | hex 3.5 N·m | 0.2 N·m | 8.75 "
        "| 0.85 N·m | 2.06 |",
        "| link:b1 | J1, J3 | plate in tension | - | acrylic_3mm 3 mm (needs al6061) "
        "| 5.0 | 6.0 | 50.0 | 1.5 |",
        "| link:b2 | J1, J2, J3 | plate in bending | - | acrylic_3mm 3 mm | - | - | - | - |",
    ]
    assert lines[9:] == ["", "Strength findings:", "",
                         "* **WARNING** pin:J4_leg0 jam SF 1.2. Fixes: a; b."]
    assert audit.strength_lines(dict(_ST, findings=[]))[-1].startswith("| link:b2 |")


# -- tools.audit: snap and wobble ---------------------------------------------------------


def test_snap_check_worst_relieved_and_problems_over_the_limit():
    empty = audit._snap_check({})
    assert empty == {"joints": 0, "worst_pct": None, "max_pct": None, "relieved": {},
                     "problems": []}
    assert audit._snap_cell(empty) == "-"
    strains = {"pin:J1": {"strain_pct": 3.0, "max_pct": 4.0, "engage_mm": 0.6},
               "pin:J2": {"strain_pct": 4.0, "max_pct": 4.0, "engage_mm": 0.4},   # at it
               "pin:J3": {"strain_pct": 5.25, "max_pct": 4.0, "engage_mm": 0.6}}
    snap = audit._snap_check(strains)
    assert snap["joints"] == 3
    assert (snap["worst_pct"], snap["max_pct"]) == (5.25, 4.0)
    assert snap["relieved"] == {"pin:J2": 0.4}          # a lip shallower than another's
    assert snap["problems"] == ["pin:J3: prongs strain 5.2 % (max 4 %)"]
    assert audit._snap_cell(snap) == "5.2 % of 4 %"


def _link(link, tilt, free=1.0):
    return {"link": link, "tilt_deg": tilt, "free_deg": free}


def _wnote(links, play=0.1, span=6.0, layers=None):
    out = {"links": links, "play_mm": play, "play_basis": "printed rings", "span_mm": span,
           "bearing_len_mm": 3.0, "pitch_mm": 3.0, "section": Section.rod(3.0).as_dict()}
    if layers:
        out["layers"] = layers
    return out


_WOBBLE = {
    "pin:J1": _wnote([_link("b1", 0.5), _link("b2", 2.0, 4.0)], play=0.1, span=3.0,
                     layers={"b1": 1, "b2": 2}),
    "pin:J2": _wnote([_link("b3", 2.5, 3.0), _link("b4", 1.0)], play=0.2, span=9.0,
                     layers={"b3": 1, "b4": 4}),
    "pillar:J5": _wnote([_link("b5", 0.25, 0.5)], play=0.05, span=12.0),
    "other:J9": _wnote([_link("x", 9.0)]),          # neither kind: not counted
}


def test_wobble_check_worst_mean_and_warnings_past_two_degrees():
    w = audit.wobble_check(_WOBBLE)
    assert w["loads_n"] is None
    assert w["pin"] == {"joints": 2, "links": 4, "worst_deg": 2.5, "worst_at": "pin:J2 b3",
                        "mean_deg": 1.5, "worst_free_deg": 4.0, "play_mm": 0.2,
                        "play_basis": "printed rings", "max_span_mm": 9.0}
    assert w["pillar"]["worst_at"] == "pillar:J5 b5"
    assert w["warnings"] == ["pin:J2 b3: 2.5 deg of tilt"]   # 2.0 exactly is not past it
    assert audit._wobble_cell(w) == "2.50 deg (4.0 free)"
    assert audit._wobble_cell(w, "pillar") == "0.25 deg (0.5 free)"
    assert audit._wobble_cell({"warnings": []}) == "-"
    assert audit.wobble_check({}) == {"loads_n": None, "warnings": []}


def test_wobble_check_with_loads_rates_each_kind_at_them():
    w = audit.wobble_check(_WOBBLE, (10.0, 100.0))
    assert w["loads_n"] == [10.0, 100.0]
    for kind in ("pin", "pillar"):
        walk, jam = w[kind]["walk"], w[kind]["jam"]
        assert set(walk) == {"bending_mpa", "shear_mpa", "bearing_mpa", "safety"}
        # the stresses scale with the load: ten times the load, a tenth of the SF
        assert jam["bending_mpa"] == pytest.approx(10 * walk["bending_mpa"], rel=0.02)
        assert jam["safety"] == pytest.approx(walk["safety"] / 10, rel=0.02)
    # the pin's worst is the longer span (J2: links 9 mm apart)
    j2 = strength.stresses(_WOBBLE["pin:J2"], 10.0)
    assert w["pin"]["walk"]["safety"] == j2["safety"]


# -- tools.audit: the markdown ------------------------------------------------------------


def _module_report(problems=(), warns=()):
    return {
        "layers": 14, "parts": 120, "plan": "layer plan text", "plan_violations": [],
        "clash": {"t=1": [{"a": "x", "b": "y", "mm3": 1.0}], "t=4.38": []},
        "contract": {"t=0": [], "t=1.6": ["c"]}, "dxf_sheets": 3,
        "bom": {"items": 42, "cost_usd": 123.456, "unpriced": ["widget"],
                "unverified_links": ["gizmo"],
                "cuts": [{"count": 4, "name": "3 mm rod", "total_mm": 100.4, "stock_mm": 300.0,
                          "pieces": [(25.05, 2), (25.15, 2)]}]},
        "snap": {"worst_pct": None, "relieved": {"pin:J1": 0.4}},
        "wobble": audit.wobble_check(_WOBBLE),
        "strength": dict(_ST, worst={"pin": {"jam": {"safety": 1.2}, "walk": {"safety": 9.5}},
                                     "crank": {"jam": {"safety": 2.06}}}),
        "manufacture": {"sheets": {"al5052_2mm": "sendcutsend"}, "parts": 9, "errors": {},
                        "issues": []},
        "chassis": {"ties": 2, "rear_screw": "M3"},
        "chicago": {"J1": {"length_mm": 10.0, "play_mm": 0.1},
                    "J2": {"length_mm": 10.0, "play_mm": 0.1},
                    "J3": {"length_mm": 8.0, "play_mm": 0.2}},
        "crank_bolt": {"plates": 4, "crankpin": "hex standoffs",
                       "chains": [{"at": "J1", "standoff": "hex_m3_20", "shims_mm": 0.5}],
                       "journals": [{"at": "O", "standoff": "hex_m3_8"}]},
        "seconds": 12.5, "problems": list(problems), "warnings": list(warns),
    }


def test_markdown_table_row_and_module_sections():
    report = {"config": {"linkage": "strider", "pin": "chicago"},
              "modules": {"double": _module_report(["clash t=1: x x y 1.0 mm^3"],
                                                   ["wobble: too much"])}}
    md = audit.markdown(report)
    lines = md.splitlines()
    assert lines[0] == "# Fabrication audit"
    assert lines[2] == "Config: linkage `strider`, pin `chicago`"
    row = next(x for x in lines if x.startswith("| double |"))
    assert row == ("| double | 14 | 120 | 1 | 1 | 0 | 3 | 42 | $123.46 | - "
                   "| 2.50 deg (4.0 free) | 0.25 deg (0.5 free) | 1.2 / 9.5 | - | 2.06 | - "
                   "| ok | FAIL |")
    for want in [
        "## double", "layer plan text", "Chassis: ties 2, rear_screw M3",
        "Clash check at t=1, t=4.38; contract at t=0, t=1.6. 12.5 s.",
        "* clash t=1: x x y 1.0 mm^3", "* wobble: too much",
        "Wobble, pins: worst 2.50 deg (pin:J2 b3), mean 1.50, free 4.0 deg; play 0.2 mm "
        "(printed rings), longest span 9 mm.",
        "Chicago screws (per side): 1 x 8 mm, 2 x 10 mm; play 0.1, 0.2 mm.",
        "Crank (per side): 4 single aluminium plates; crankpins and journals hex standoffs: "
        "J1 20 mm + 0.5 mm shims, O 8 mm.",
        "Snap lips relieved (engage mm): pin:J1 0.40.",
        "Cut to length: 4 pieces of 3 mm rod (100 mm in all, from 300 mm stock): "
        "2 x 25.1, 2 x 25.1 mm; deburr every cut end.",
        "No listed price: widget.", "Preferred offer not verified: gizmo.",
        "Every part passes.", "* **WARNING** pin:J4_leg0 jam SF 1.2. Fixes: a; b.",
    ]:
        assert want in lines, want
    assert lines.index("Problems:") < lines.index("Warnings:")


def test_markdown_ok_module_has_no_problem_or_optional_sections():
    rep = _module_report()
    for k in ("chicago", "crank_bolt", "strength", "manufacture"):
        rep.pop(k)
    rep["bom"] = {}
    rep["snap"] = {"worst_pct": 1.0, "max_pct": 4.0, "relieved": {}}
    rep["wobble"] = audit.wobble_check({k: v for k, v in _WOBBLE.items()
                                        if k.startswith("pin:")})
    md = audit.markdown({"config": {}, "modules": {"single": rep}})
    row = next(x for x in md.splitlines() if x.startswith("| single |"))
    assert row.endswith("| $0.00 | 1.0 % of 4 % | 2.50 deg (4.0 free) | - "
                        "| - | - | - | - | - | OK |")
    assert "Wobble, pins:" in md
    assert "Wobble, pillars:" not in md
    assert "| - | $0.00 |" in row                       # no BOM: no item count
    for gone in ("Problems:", "Warnings:", "Chicago screws", "Crank (per side)",
                 "Snap lips", "Cut to length", "Joint strength", "Cut rules"):
        assert gone not in md


# -- strength: levels, loads and the crank ------------------------------------------------


@pytest.mark.parametrize(("jam", "walk", "lv"), [
    (0.99, 10.0, "error"), (1.0, 10.0, "warning"), (1.99, None, "warning"),
    (2.0, 3.0, None), (2.0, 2.99, "warning"), (None, 2.99, "warning"), (None, None, None),
    (0.5, None, "error"),
])
def test_strength_level_boundaries(jam, walk, lv):
    row = {"jam": None if jam is None else {"safety": jam},
           "walk": None if walk is None else {"safety": walk}}
    assert strength.level(row) == lv


def test_strength_stem_and_uniform_loads():
    assert strength._stem("pin:J3_leg1") == "J3"
    assert strength._stem("pillar:J2") == "J2"
    assert strength._stem("J7_leg0") == "J7"
    assert strength.uniform_loads(1.0, 2.0, "s", "n") == {
        "source": "s", "note": "n", "walk_n": 1.0, "jam_n": 2.0, "joints": []}
    assert strength.family_loads("nope") is None
    jl = strength.joint_loads("pin:J1", {"links": []}, {"source": "override", "walk_n": 3.0,
                                                        "jam_n": 30.0})
    assert jl == {"walk_n": 3.0, "jam_n": 30.0, "walk_patterns": None, "jam_patterns": None,
                  "basis": "override"}


def test_strength_design_loads_falls_back_when_the_sim_fails_or_has_nothing(monkeypatch):
    def failing(*a, **k):
        raise RuntimeError("model broke")

    monkeypatch.setattr(sl, "design_loads", failing)
    lo = strength.design_loads(BuildConfig())
    assert lo["source"] == "fallback"
    assert lo["note"] == ("no sim: the sim failed (RuntimeError: model broke); the strider "
                          "family's measured peaks, conservative")
    monkeypatch.setattr(sl, "design_loads", lambda *a, **k: None)
    lo = strength.design_loads(BuildConfig(), sim="cached")
    assert lo["note"].startswith("no sim: no simulated loads stored for it yet;")
    assert (lo["walk_n"], lo["jam_n"]) == (6.4, 38.0)

    def no_mujoco(*a, **k):
        raise ImportError("mujoco")

    monkeypatch.setattr(sl, "design_loads", no_mujoco)
    lo = strength.design_loads(BuildConfig(linkage="jansen"))
    assert lo["note"] == ("no sim: MuJoCo isn't installed; the most loaded measured "
                          "family's (the demo Klann), conservative")
    assert (lo["walk_n"], lo["jam_n"], lo["torque_limit_nm"]) == (119.0, 155.0, 0.85)
    over = strength.design_loads(BuildConfig(), override=(5.0, 50.0))
    assert (over["source"], over["note"]) == ("override", "--pin-load 5,50 N on every joint")
    mech = strength.design_loads(BuildConfig(linkage="parallelogram_lift", module="single",
                                             robot=False))
    assert (mech["source"], mech["walk_n"], mech["jam_n"]) == ("none", 0.0, 0.0)


def test_strength_joint_loads_match_the_sim_joint_by_stem_kind_and_links():
    def sj(stem, links, frame, walk, jam, crank=False):
        return {"stem": stem, "links": links, "frame": frame, "crank": crank,
                "walk": {"n": walk, "patterns": []},
                "jam": {"n": jam, "patterns": [{"b1": [1.0, 0.0]}]}}

    loads = {"source": "sim", "walk_n": 9.0, "jam_n": 90.0, "joints": [
        sj("J2", ["b1_leg0"], True, 1.0, 10.0),            # the pillar
        sj("J2", ["b1_leg0", "b2_leg0"], False, 2.0, 20.0),  # a pin of the same stem
        sj("J2", ["b1_leg0"], False, 7.0, 70.0, crank=True)]}
    note = {"links": [{"link": "b1_leg0"}]}
    pillar = strength.joint_loads("pillar:J2", note, loads)
    assert (pillar["walk_n"], pillar["jam_n"], pillar["basis"]) == (1.0, 10.0, "sim")
    assert pillar["walk_patterns"] is None                    # an empty list: none
    assert pillar["jam_patterns"] == [{"b1": [1.0, 0.0]}]
    pin = strength.joint_loads("pin:J2_leg0", note, loads)
    assert pin["jam_n"] == 20.0
    stranger = strength.joint_loads("pin:J2_leg0", {"links": [{"link": "b9_leg0"}]}, loads)
    assert (stranger["jam_n"], stranger["basis"]) == (
        90.0, "sim, the design's largest (no sim joint matched)")


def test_rider_hole_is_the_cranks_rider_bore_else_the_crankpins():
    cfg = BuildConfig()
    assert strength.rider_hole(cfg) == pytest.approx(cfg.params.hole(8.5))   # the sleeve
    plain = replace(cfg, crank="nope")
    assert strength.rider_hole(plain) == cfg.params.hole(cfg.params.crankpin_d)


def test_strength_sim_loads_take_the_largest_non_crank_joint(monkeypatch):
    j = [{"stem": "J1", "links": ["a"], "crank": False, "frame": True,
          "walk": {"n": 3.0}, "jam": {"n": 30.0}},
         {"stem": "J2", "links": ["b"], "crank": False, "frame": False,
          "walk": {"n": 5.0}, "jam": {"n": 20.0}},
         {"stem": "O", "links": ["c"], "crank": True, "frame": False,
          "walk": {"n": 99.0}, "jam": {"n": 999.0}}]
    doc = {"walk_percentile": 99.0, "walk_seconds": 3.0, "torque_limit_nm": 0.85,
           "jam_cases": 48, "jam_stalled": 1.0, "joints": j}
    monkeypatch.setattr(sl, "design_loads", lambda *a, **k: dict(doc))
    lo = strength.design_loads(BuildConfig())
    assert (lo["source"], lo["walk_n"], lo["jam_n"]) == ("sim", 5.0, 30.0)
    assert lo["note"] == ("the design's own, MuJoCo: walking p99 over 3 s, jammed at the "
                          "0.85 N·m torque limit (48 cases, a foot pinned, 100% stalled)")
    doc.update(jam_stalled=0.75)                    # (no drift recorded: 0 mm)
    assert strength.design_loads(BuildConfig())["note"].endswith(
        "75% stalled); WARNING: 12 of 48 jam cases never reached the torque limit (pinned "
        "foot drifted up to 0 mm), so the jam loads are undersampled")


def test_crank_strength_without_a_walking_torque_rates_the_jam_only(monkeypatch):
    cfg = BuildConfig()
    loads = {"torque_limit_nm": 0.85, "joint_moment_factor": 2.0}
    meta = {"crank_bolt": {"chains": [{"capacity_nm": {"hex": 4.0, "torsion": 9.0}}],
                           "journals": [{"capacity_nm": {"hex": 3.4}}]}}
    row = strength.crank_strength(meta, cfg, loads)
    assert row["capacity_nm"] == {"hex": 3.4, "torsion": 9.0}     # the least per element
    assert row["weakest"] == "hex"
    assert row["walk"] is None
    assert row["jam"] == {"torque_nm": 0.85, "moment_nm": 1.7, "safety": 2.0}
    assert row["basis"] == "the firmware torque limit jammed (no walking sim)"
    assert strength.level(row) is None
    assert strength.crank_strength(meta, replace(cfg, crank="nope"), loads) is None
    # no factor in the loads: the crank's own (chord / crank radius), with a walking torque
    from spiderpig.sim import run

    monkeypatch.setattr(run, "crank_joint_factor", lambda config: 1.0)
    row = strength.crank_strength(meta, cfg, {"torque_limit_nm": 0.85,
                                              "walk_torque_nm": 0.17})
    assert row["factor"] == 1.0
    assert row["walk"] == {"torque_nm": 0.17, "moment_nm": 0.17, "safety": 20.0}
    assert row["basis"] == "sim walking torque, the firmware torque limit jammed"


def test_link_fixes_name_the_sheet_and_a_torque_limit():
    row = {"kind": "link", "links": ["b1"], "needs": {"sheet": "al6061", "jam_safety": 4.2},
           "jam": {"safety": 1.0}}
    assert strength.fixes(row, None, {"torque_limit_nm": 0.8}, BuildConfig()) == [
        "cut b1 from al6061 (--link-sheet b1=al6061): jam SF 4.2",
        "a servo torque limit of 0.40 N·m (now 0.8) for jam SF 2"]
    plain = {"kind": "link", "links": ["b1"], "needs": None, "jam": {"safety": 2.5}}
    assert strength.fixes(plain, None, {}, BuildConfig()) == [
        "a wider link (Params.link_radius) or a stronger sheet"]


# -- verify: rows and their helpers -------------------------------------------------------


def test_row_describe_and_round_trip():
    r = Row("geometry.stack_mm", "plan", 66.54321, "<= 70", True, "proven", unit="mm")
    assert r.describe() == "geometry.stack_mm: 66.54 mm vs <= 70 [ok, proven]"
    soft = Row("walk.speed", "walk", 3, None, False, "estimated", hard=False, detail="slow")
    assert soft.describe() == "walk.speed: 3 [MISS, estimated, soft] slow"
    hard = Row("stage.plan", "plan", "no_plan", None, False, "proven")
    assert hard.describe() == "stage.plan: no_plan [FAIL, proven]"
    d = r.to_dict()
    assert d["pass"] is True
    assert "passed" not in d
    assert Row.from_dict(d) == r
    assert Row.from_dict({"requirement": "x", "passed": 1}) == Row("x", "", None, None, True,
                                                                   "")


def test_verify_report_describe_and_round_trip():
    rep = VerifyReport("quick", ok=False, score=0.5, seconds=1.25,
                       rows=[Row("a", "s", 1.0, None, True, "proven")],
                       failures=[Failure("plan", "no_plan", "no plan\nmore")],
                       unverified=["b", "c"])
    assert rep.describe().splitlines() == [
        "verify quick: FAIL, score 0.50 (1.2 s)", "  a: 1 [ok, proven]", "failures:",
        "  plan (no_plan): no plan", "not verified at this level: b, c"]
    back = VerifyReport.from_dict(rep.to_dict())
    assert back.to_dict() == rep.to_dict()
    assert VerifyReport("standard").describe() == "verify standard: ok, score 1.00 (0.0 s)"


def test_least_transmission_angle_folds_about_ninety():
    assert least_transmission_angle([]) == (None, "")
    assert least_transmission_angle([{"point": "J1"}, {"transmission_deg": [None, None]}]) \
        == (None, "")
    closures = [{"point": "J1", "transmission_deg": [50.0, 120.0]},     # 50
                {"point": "J2", "transmission_deg": [60.0, 140.0]},     # 180-140 = 40
                {"point": "J3", "transmission_deg": [45.0, 90.0]}]      # 45
    assert least_transmission_angle(closures) == (40.0, "J2")


def test_stage_row_push_and_fail():
    ok = _stage_row("stage.plan", "plan", [], "14 layers")
    assert (ok.value, ok.passed, ok.tier, ok.hard) == ("14 layers", True, "proven", True)
    bad = _stage_row("stage.plan", "plan", [Failure("plan", "no_plan", "none\nwhy")], "x",
                     tier="measured")
    assert (bad.value, bad.passed, bad.detail, bad.tier) == ("no_plan", False, "none",
                                                             "measured")
    rows: list = []
    _push(rows, None)
    _push(rows, ok)
    assert rows == [ok]
    rep = VerifyReport("quick")
    _fail(rep, [], "build", "clash")
    assert rep.failures == []
    _fail(rep, ["a x b", "c x d"], "build", "clash")
    (f,) = rep.failures
    assert (f.stage, f.code, f.message) == ("build", "clash", "a x b; c x d")
    assert f.culprits == [{"text": "a x b"}, {"text": "c x d"}]


def test_cut_rule_rows_fail_on_errors_and_count_warnings_soft():
    rep = VerifyReport("standard")
    rows = _cut_rule_rows(_CUTS, rep)
    assert [(r.requirement, r.value, r.passed, r.hard) for r in rows] == [
        ("manufacture.cut_rules", 2, False, True), ("manufacture.warnings", 3, True, False)]
    (f,) = rep.failures
    assert (f.stage, f.code) == ("manufacture", "cut_rule")
    assert [c["part"] for c in f.culprits] == ["p2", "p3"]
    assert f.culprits[0]["why"] == "the web distorts"
    clean = VerifyReport("standard")
    (row,) = _cut_rule_rows({"issues": [], "parts": 7}, clean)
    assert row.passed
    assert clean.failures == []
    assert row.detail == "7 laser-cut parts within every service's hard limits"


def test_cost_and_unpriced_items():
    r = SimpleNamespace(name="M3 screw", qty=4.0, cost_usd=2.5, pack_qty=100)
    assert _cost_item(r) == "M3 screw x 4 $2.50 (a pack of 100)"
    one = SimpleNamespace(name="servo", qty=1, cost_usd=20.0, pack_qty=1)
    assert _cost_item(one) == "servo $20.00"
    u = SimpleNamespace(qty=3.0, name="shim", packs=2, pack_qty=10, vendor="McMaster")
    assert _unpriced_item(u) == "3 x shim (2 packs of 10 at McMaster)"
    u1 = SimpleNamespace(qty=1.0, name="rod", packs=1, pack_qty=1, vendor="")
    assert _unpriced_item(u1) == "1 x rod (1)"
    assert _unpriced_item(SimpleNamespace(qty=5.0, name="nut", packs=1, pack_qty=50,
                                          vendor=None)) == "5 x nut (1 pack of 50)"


def test_mass_by_group_heaviest_first():
    br = SimpleNamespace(parts=[{"group": "links:b1", "mass_g": 10.0},
                                {"group": "drive", "mass_g": 115.2},
                                {"group": "links:b2", "mass_g": 12.0},
                                {"group": "", "mass_g": None},
                                {"mass_g": 3.0}])
    assert mass_by_group(br) == "by group: drive 115 g, links 22 g, other 3 g"
    assert mass_by_group(SimpleNamespace(parts=None)) == ""


def test_fall_detail_says_when_how_and_what_the_model_saw():
    assert fall_detail({"max_tilt": 3.21}) == "max tilt 3.2 deg"
    m = {"max_tilt": 80.0, "fell": True, "fell_at_s": 1.25, "fell_axis": "roll"}
    assert fall_detail(m) == ("max tilt 80.0 deg; fell over roll at 1.2 s into the 4 s run "
                              "(the drives run from the start)")
    assert fall_detail({"max_tilt": 80.0, "fell": True}) == "max tilt 80.0 deg"

    def design(tip, ok=True):
        return SimpleNamespace(reports={"walk": SimpleNamespace(
            ok=ok, metrics={"tipping_fraction": tip})})

    assert fall_detail(m, design(0.01)).endswith(
        "the quasi-static model's tipping fraction is 0.01 (it saw no tipping: the fall is "
        "dynamic, or the sim's contacts; a lower stack, a slower drive or other phases are "
        "the levers)")
    assert fall_detail(m, design(0.3)).endswith("is 0.30 (it predicted the risk)")
    assert fall_detail(m, design(0.3, ok=False)) == fall_detail(m)


# -- sim.loads: the walking case and both cases together, the sim faked -------------------


def test_walk_loads_samples_after_the_skip_and_takes_the_torque_percentile(tiny,
                                                                           monkeypatch):
    """The walking case on the tiny model: the sim faked (one forward pass per observed
    step), the samples before ``skip`` dropped, the drives' torque as its percentile and
    peak, the walk metrics rounded."""
    mujoco, model = tiny
    from spiderpig.sim import mjcf, run

    data = mujoco.MjData(model)
    seen = []

    def simulate(config, seconds, record_every, observe):
        for k in range(4):
            data.time = 0.25 * k
            mujoco.mj_forward(model, data)
            observe(model, data)
            seen.append(data.time)
        return SimpleNamespace(t=np.array([0.0, 0.25, 0.5, 0.75]),
                               torque=np.array([-9.0, 9.0, -1.0, 2.0]))

    monkeypatch.setattr(mjcf, "load_model", lambda config: (model, _META))
    monkeypatch.setattr(run, "simulate", simulate)
    monkeypatch.setattr(run, "walk_metrics", lambda result, skip: {
        "walks": 1, "fell": 0, "speed": 12.3456, "loop_force_peak": 1.23456})
    out = sl.walk_loads(BuildConfig())
    assert seen == [0.0, 0.25, 0.5, 0.75]
    assert out["torque_nm"] == pytest.approx(1.99)          # p99 of |-1|, |2|
    assert out["torque_peak_nm"] == 2.0
    assert (out["walks"], out["fell"], out["speed_mm_s"], out["loop_force_peak_n"]) == (
        True, False, 12.35, 1.235)
    assert [g["links"] for g in out["index"].joints] == [
        ["b1_leg0"], ["b1_leg0", "b2_leg0"], ["b2_leg0"], ["b1_leg0", "b2_leg0"]]
    assert len(out["joints"]) == 4
    # the pose never changes: the two kept samples are alike, the load is their size
    j1 = out["joints"][0]
    assert j1["n"] == pytest.approx(j1["peak"])
    assert j1["peak"] > 0.5


_JAM_MJCF = (_MJCF
             .replace('    <connect name="jam.L.foot" body1="L.b2_leg0" anchor="0 0 0" '
                      'active="false"/>\n', "")
             .replace('<joint name="h2" type="hinge" axis="0 0 1"/>',
                      '<joint name="h2" type="hinge" axis="0 0 1"/>\n'
                      '          <site name="L.foot" pos="0.02 0 0"/>')
             .replace("<worldbody>", '<worldbody>\n    <geom name="floor" type="plane" '
                                     'size="1 1 0.1"/>')
             .replace("</mujoco>", '  <actuator>\n'
                      '    <velocity name="L.drive" joint="hc" kv="1" ctrlrange="-5 5" '
                      'forcelimited="true" forcerange="-1 1"/>\n'
                      '    <velocity name="R.drive" joint="h1" kv="1" ctrlrange="-5 5"/>\n'
                      '  </actuator>\n</mujoco>'))


def test_jam_loads_holds_each_foot_both_ways_at_the_torque_limit(monkeypatch):
    """The jam's bookkeeping on the tiny model with a foot site and two drives: a case per
    crank angle, foot, hold and direction, the left drive limited to the firmware's torque
    (here it stalls at once), the right one idle, the floor's contacts switched off, the
    forces averaged over each case's last steps. MuJoCo never integrates: its step is a
    forward pass here, so the foot stays where it was caught (no drift)."""
    if mujoco is None:
        pytest.skip("MuJoCo isn't installed")
    from spiderpig.sim import mjcf, run

    meta = dict(_META, feet={"L.foot": {"body": "L.b2_leg0"}},
                actuators={"L.drive": {"ctrlrange": [-5.0, 5.0]}}, t_ref=0.0)
    monkeypatch.setattr(mjcf, "build_mjcf", lambda config: (_JAM_MJCF, meta))
    monkeypatch.setattr(mjcf, "load_model", lambda config: (
        mujoco.MjModel.from_xml_string(_JAM_MJCF), meta))
    monkeypatch.setattr(run, "kinematic_qpos", lambda config, t: np.array([0.1 * t, 0.0, 0.0]))
    steps, models = [], []

    def step(model, data):
        steps.append(float(data.ctrl[0]))
        models.append(model)
        mujoco.mj_forward(model, data)

    monkeypatch.setattr(mujoco, "mj_step", step)
    out = sl.jam_loads(BuildConfig(), angles=2, steps=3, average=2, modes=("pinned", "path"))
    assert out["cases"] == 2 * 1 * 2 * 2              # angles x feet x holds x directions
    assert len(steps) == out["cases"] * 3
    assert steps[:6] == [5.0] * 3 + [-5.0] * 3          # the left drive full one way, then back
    assert (out["torque_nm"], out["stalled"], out["floor"], out["unstalled"]) == (
        0.85, 1.0, False, [])
    assert out["foot_drift_mm"] == 0.0
    model = models[0]
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    assert (model.geom_contype[floor], model.geom_conaffinity[floor]) == (0, 0)
    eq = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, e)
          for e in range(model.neq)}
    assert {"jam.base", "jam.L.foot", "jam.L.foot.path"} <= eq
    assert [g["links"] for g in out["index"].joints] == [
        ["b1_leg0"], ["b1_leg0", "b2_leg0"], ["b2_leg0"], ["b1_leg0", "b2_leg0"]]
    assert len(out["joints"]) == 4
    assert all(j["n"] == j["peak"] for j in out["joints"])     # the jam: the largest case
    assert out["joints"][0]["n"] > 0
    # a model with no equality section gets one; pinned only: no sliders
    bare = re.sub(r"\s*<equality>.*</equality>", "", _JAM_MJCF, flags=re.S)
    monkeypatch.setattr(mjcf, "build_mjcf", lambda config: (bare, meta))
    monkeypatch.setattr(mjcf, "load_model", lambda config: (
        mujoco.MjModel.from_xml_string(bare), meta))
    models.clear()
    out = sl.jam_loads(BuildConfig(), angles=1, steps=1, average=1)
    assert out["cases"] == 2
    eq = {mujoco.mj_id2name(models[0], mujoco.mjtObj.mjOBJ_EQUALITY, e)
          for e in range(models[0].neq)}
    assert eq == {"jam.base", "jam.L.foot"}
    assert [g["links"] for g in out["index"].joints] == [["b1_leg0"], ["b1_leg0", "b2_leg0"]]


def test_simulate_loads_pairs_the_jam_with_the_walk_per_joint(monkeypatch):
    """A joint's jam load is at least its walking peak; a walking joint with no jam twin
    takes a zero jam; the document carries every case's bookkeeping."""
    from spiderpig.sim import run

    joints = [{"stem": "J1", "links": ["a"], "xy_mm": [1.0, 2.0], "frame": True,
               "crank": False},
              {"stem": "J2", "links": ["a", "b"], "xy_mm": [3.0, 4.0], "frame": False,
               "crank": False}]
    walk = {"index": sl._SideFree(joints, []),
            "joints": [{"n": 2.0, "peak": 9.0, "patterns": []},
                       {"n": 1.0, "peak": 3.0, "patterns": [{"a": [1.0, 0.0]}]}],
            "torque_nm": 0.2, "torque_peak_nm": 0.3, "walks": True, "fell": False,
            "speed_mm_s": 50.0, "loop_force_peak_n": 4.0}
    jam = {"index": sl._SideFree(joints[:1], []),
           "joints": [{"n": 5.0, "peak": 5.0, "patterns": []}],
           "torque_nm": 0.85, "cases": 48, "floor": False, "stalled": 1.0,
           "foot_drift_mm": 0.02, "unstalled": []}
    seen = {}

    def fake_walk(config):
        seen["robot"] = config.robot
        return walk

    monkeypatch.setattr(sl, "walk_loads", fake_walk)
    monkeypatch.setattr(sl, "jam_loads", lambda config: jam)
    monkeypatch.setattr(run, "crank_joint_factor", lambda config: 2.0)
    doc = sl.simulate_loads(BuildConfig(robot=False, module="single"))
    assert seen["robot"] is True                       # always the robot's loads
    assert doc["version"] == sl.VERSION
    assert doc["source"] == "sim"
    assert (doc["jam_cases"], doc["jam_stalled"], doc["torque_limit_nm"]) == (48, 1.0, 0.85)
    assert (doc["walk_torque_nm"], doc["joint_moment_factor"]) == (0.2, 2.0)
    j1, j2 = doc["joints"]
    assert j1["jam"]["n"] == 9.0                       # the walking peak over the jam's 5
    assert j1["walk"] == walk["joints"][0]
    assert j2["jam"] == {"n": 3.0, "peak": 0.0, "patterns": []}   # no jam twin
    assert (j2["stem"], j2["links"], j2["xy_mm"], j2["frame"]) == ("J2", ["a", "b"],
                                                                   [3.0, 4.0], False)


# -- tools.audit: one module's audit and the CLI, every stage faked -----------------------


def _fake_audit(monkeypatch, *, dxf_error=None, bom_error=None, deck=None):
    from spiderpig import api

    plan = SimpleNamespace(top=13, height=66.5, describe=lambda: "PLAN", gaps={3: 1.5},
                           heads="gap", thick={2: 3.2})
    meta = {"snap_strain": {"pin:J1": {"strain_pct": 5.0, "max_pct": 4.0,
                                       "engage_mm": 0.6}},
            "wobble": _WOBBLE, "chicago": {"J1": {"length_mm": 10.0, "play_mm": 0.1}},
            "crank_bolt": {"assembly": "no order drives the hub screw"},
            "ties": 2, "rear_screws_per_servo": 1, "irrelevant": 9,
            "deck": deck}
    mech = SimpleNamespace(bodies=[SimpleNamespace(part=1), SimpleNamespace(part=None),
                                   SimpleNamespace(part=2)], meta=meta, bom_extras=[])
    calls = {"fabricate": []}
    monkeypatch.setattr(audit, "template_for", lambda config: SimpleNamespace(
        freeze_at=lambda t: t))
    monkeypatch.setattr(api, "plan_config", lambda config, store: SimpleNamespace(plan=plan))
    monkeypatch.setattr(audit, "verify_plan", lambda p, tmpl: ["J3 meets b4"])
    monkeypatch.setattr(audit, "check_sides", lambda design, tmpl, ts: [
        ["b1 outside"] if t == 0 else [] for t in ts])

    def fabricate(tmpl, config, t):
        calls["fabricate"].append(t)
        return mech

    monkeypatch.setattr(audit, "fabricate", fabricate)
    monkeypatch.setattr(audit, "clashes", lambda m: [{"a": "x", "b": "y", "mm3": 2.5}])
    monkeypatch.setattr(audit, "bad_solids", lambda m: [{"part": "p", "solids": 2,
                                                         "valid": False}])
    loads = {"source": "sim", "jam_stalled": 0.5, "joints": []}
    monkeypatch.setattr(strength, "design_loads",
                        lambda config, store, override, sim: loads)
    st = {"loads": loads, "rows": [], "worst": {},
          "findings": [_finding("error", "pin:J1 jam SF 0.5", "fix A"),
                       _finding("warning", "crank jam SF 1.5", "fix B")]}
    monkeypatch.setattr(strength, "check", lambda notes, m, config, lo: st)
    monkeypatch.setattr(audit, "deck_clearance", lambda m: {
        "overlapping": [["horn", "deck"]], "blocked": [["deck", "J2 head", 1.5]]})

    def sheet_lines(m, sheet):
        if dxf_error:
            raise ValueError(dxf_error)
        return [SimpleNamespace(key="acrylic_3mm", qty=2.0)]

    monkeypatch.setattr(audit, "sheet_lines", sheet_lines)
    monkeypatch.setattr(manufacture, "check", lambda m, sheet: _CUTS)

    def bom_from_mechanism(m, group):
        if bom_error:
            raise KeyError(bom_error)
        row = SimpleNamespace(key="m3", verified=False, same_pack_as=None)
        return SimpleNamespace(purchased=[row], cost_usd=12.345, unpriced=[row],
                               as_dict=lambda: {"cuts": []})

    monkeypatch.setattr(audit, "bom_from_mechanism", bom_from_mechanism)
    return mech, calls


def test_audit_module_collects_every_check_into_problems_and_warnings(monkeypatch):
    mech, calls = _fake_audit(monkeypatch)
    rep = audit.audit_module("double", BuildConfig(), [0.0, 1.6], [1.0, 4.38])
    assert calls["fabricate"] == [1.0, 4.38]
    assert (rep["layers"], rep["stack_mm"], rep["plan"], rep["parts"]) == (14, 66.5, "PLAN",
                                                                           2)
    assert rep["contract"] == {"t=0": ["b1 outside"], "t=1.6": []}
    assert rep["chassis"] == {"ties": 2, "rear_screws_per_servo": 1}
    assert rep["deck"]["fitted"] is False
    assert (rep["dxf_sheets"], rep["dxf_error"], rep["sheets"]) == (2, None,
                                                                    {"acrylic_3mm": 2})
    assert len(mech.bom_extras) == 1
    assert rep["bom"] == {"items": 1, "cost_usd": 12.35, "unpriced": ["m3"],
                          "unverified_links": ["m3"], "cuts": []}
    assert rep["plate_z"] == {"gaps_mm": {"3": 1.5}, "heads": "gap", "thick_mm": {"2": 3.2}}
    assert rep["problems"] == [
        "plan: J3 meets b4",
        "contract t=0: b1 outside",
        "clash t=1: x x y 2.5 mm^3",
        "clash t=4.38: x x y 2.5 mm^3",
        "solid t=1: p (2 solids, valid=False)",
        "solid t=4.38: p (2 solids, valid=False)",
        "snap: pin:J1: prongs strain 5.0 % (max 4 %)",
        "deck: horn sweeps through deck",
        "deck: deck can't be lowered past J2 head (1.5 mm^3)",
        "strength: pin:J1 jam SF 0.5 (fix: fix A)",
        *audit.manufacture_messages(_CUTS, "error"),
        "assembly: no order drives the hub screw",
    ]
    assert rep["warnings"] == [
        "wobble: pin:J2 b3: 2.5 deg of tilt",
        "strength: crank jam SF 1.5 (fix: fix B)",
        *audit.manufacture_messages(_CUTS, "warning"),
        "strength: only 50% of the jam cases stalled: the jam loads are undersampled",
        "no electronics deck: None",
        "only 1 rear screw(s) per servo into the centre plates",
    ]


def test_audit_module_reports_a_dxf_and_bom_failure_and_a_side_has_no_deck(monkeypatch):
    _fake_audit(monkeypatch, dxf_error="a part is wider than the sheet", bom_error="nut_m9",
                deck={"fitted": True})
    side = BuildConfig(robot=False, module="single")
    rep = audit.audit_module("single", side, [], [1.0])
    assert (rep["dxf_sheets"], rep["dxf_error"]) == (0, "a part is wider than the sheet")
    assert "sheets" not in rep
    assert rep["bom"] == {}
    assert rep["bom_error"] == "'nut_m9'"
    assert "deck" not in rep
    assert "dxf: a part is wider than the sheet" in rep["problems"]
    assert "bom: 'nut_m9'" in rep["problems"]
    assert not any(p.startswith("deck:") for p in rep["problems"])
    assert not any("rear screw" in w or "electronics" in w for w in rep["warnings"])


def test_audit_main_writes_the_report_and_fails_on_problems(monkeypatch, tmp_path, capsys):
    seen = []

    def module(m, config, ts_contract, ts_clash, store, pin_loads, sim):
        seen.append((m, config.module, ts_contract, ts_clash, store.root, pin_loads, sim))
        rep = _module_report(["plan: x"] if m == "double" else [], ["w"])
        rep["strength"]["loads"] = {"source": "override"}
        return rep

    monkeypatch.setattr(audit, "audit_module", module)
    out = tmp_path / "out"
    code = audit.main(["--modules", "single,double", "--out", str(out), "--store",
                       str(tmp_path / "store"), "--pin-load", "5,50", "--no-sim",
                       "--ts-clash", "2", "--ts-contract", "0,3"])
    assert code == 1
    assert seen == [(m, m, [0.0, 3.0], [2.0], tmp_path / "store", (5.0, 50.0), False)
                    for m in ("single", "double")]
    report = json.loads((out / "audit.json").read_text())
    assert list(report["modules"]) == ["single", "double"]
    assert report["config"]["linkage"] == "strider"
    assert (out / "audit.md").read_text() == audit.markdown(report)
    text = capsys.readouterr().out
    assert "== single" in text
    assert "  plan: x" in text
    assert "  warning: w" in text
    assert ("joint SF jam / walk: pin 1.2 / 9.5, pillar -, crank 2.06, link plate -, "
            "centre plates - "
            "(override loads)") in text
    assert "OK (12.5 s)" in text
    assert "FAIL (12.5 s)" in text
    code = audit.main(["--modules", "single", "--out", str(out), "--store",
                       str(tmp_path / "store"), "--phases", "90", "--proportion",
                       "unit=12"])
    assert code == 0
    report = json.loads((out / "audit.json").read_text())
    assert report["config"]["phases_deg"] == [90.0]     # (the default phase is dropped)
    assert report["config"]["proportions"] == {"unit": 12.0}


@pytest.mark.parametrize("argv", [["--pin-load", "5"], ["--pin-load", "a,b"],
                                  ["--modules", "no_such_module"]])
def test_audit_main_refuses_bad_options(argv, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(audit, "audit_module", lambda *a, **k: pytest.fail("audited"))
    with pytest.raises(SystemExit) as e:
        audit.main([*argv, "--out", str(tmp_path)])
    assert e.value.code == 2
    assert "error:" in capsys.readouterr().err


# -- strength: rows, findings and fixes on hand-made notes --------------------------------


def _snote(layers, anchors=(), section=None):
    ks = sorted(layers.values())
    return {"links": [{"link": m} for m in layers], "layers": dict(layers),
            "anchors": list(anchors), "pitch_mm": 3.0, "bearing_len_mm": 3.0,
            "span_mm": (ks[-1] - ks[0]) * 3.0,
            "section": (section or Section.rod(3.0)).as_dict()}


def test_strength_check_rows_worst_and_findings_with_their_fixes():
    """A rod pin far apart and a one-plate pillar, overloaded: errors first with the other
    construction, the shorter span and the plates' anchor as fixes (each recomputed), the
    round crank's clamp a warning with its threadlocker fix."""
    notes = {"pin:J4_leg0": _snote({"b3_leg0": 2, "b4_leg0": 6}),
             "pillar:J2": _snote({"b2_leg0": 4}, anchors=[0])}
    loads = strength.uniform_loads(30.0, 300.0, "override", "test")
    loads.update(torque_limit_nm=0.85, joint_moment_factor=2.0)
    st = strength.check(notes, {}, BuildConfig(crank="bolt_round"), loads)
    kinds = [r["kind"] for r in st["rows"]]
    assert kinds == ["pillar", "pin", "crank"]
    pin = st["rows"][1]
    assert (pin["case"], pin["span_mm"], pin["section"], pin["basis"]) == (
        "two-link pin", 12.0, "rod", "override")
    assert pin["jam"]["load_n"] == 300.0
    assert st["worst"]["pin"]["jam"] == {"safety": pin["jam"]["safety"],
                                         "joint": "pin:J4_leg0"}
    levels = [f["level"] for f in st["findings"]]
    assert levels == sorted(levels, key=lambda lv: lv != "error")     # errors first
    fp = next(f for f in st["findings"] if f["joint"] == "pin:J4_leg0")
    assert fp["level"] == "error"
    assert fp["message"].startswith("pin:J4_leg0 (b3_leg0, b4_leg0; two-link pin, 12 mm "
                                    "span, rod): jam SF ")
    assert fp["fixes"][0].startswith("--pin chicago (")
    assert any(x.startswith("a shorter span (12 -> 3 mm") for x in fp["fixes"])
    assert fp["fixes"][-1].startswith("a servo torque limit of ")
    fl = next(f for f in st["findings"] if f["joint"] == "pillar:J2")
    assert any(x.startswith("anchor the pillar in both frame plates") for x in fl["fixes"])
    fc = next(f for f in st["findings"] if f["kind"] == "crank")
    assert fc["level"] == "warning"
    assert any("threadlocker" in x for x in fc["fixes"])
    assert st["loads"] == {k: v for k, v in loads.items() if k != "joints"}


def test_strength_check_without_loads_rates_only_the_crank():
    loads = {"source": "none", "torque_limit_nm": 0.85, "joint_moment_factor": 2.0}
    st = strength.check({"pin:J1": _snote({"a": 1, "b": 2})}, {}, BuildConfig(), loads)
    assert [r["kind"] for r in st["rows"]] == ["crank"]
    assert strength.crank_capacity({}, replace(BuildConfig(), crank="nope")) is None
    assert strength.bolt_crank("bolt")
    assert not strength.bolt_crank("nope")


def test_crank_fixes_a_thicker_sheet_for_a_hex_pocket():
    row = {"kind": "crank", "factor": 2.0, "construction": "bolt",
           "weakest": "hex 5.5 AF in its plate's pocket",
           "capacity_nm": {"hex 5.5 AF in its plate's pocket": 2.0},
           "jam": {"torque_nm": 0.85}}
    out = strength.fixes(row, None, {}, BuildConfig())
    assert out[0] == ("set the servo's torque limit to 0.50 N·m or less (jam SF 2; now "
                      "0.85)")
    thick = materials.sheet("al6061_3p2mm").thickness
    now = materials.sheet(BuildConfig().crank_sheet).thickness
    assert out[1] == (f"a thicker crank sheet (--crank-sheet al6061_3p2mm: {thick:g} mm of "
                      f"hex pocket against {now:g} mm now)")
    no_jam = dict(row, jam=None)
    assert strength.fixes(no_jam, None, {}, BuildConfig())[0] == (
        "keep the servo's torque limit under 0.50 N·m")


def test_a_pin_no_construction_clears_says_so():
    """A pin already on the Chicago barrel in adjacent layers, its jam SF fine but its
    walking SF short: no other section, no shorter span, no torque fix for the walking
    case: the catch-all."""
    from spiderpig.construction.pivots.chicago import ChicagoShaft, chicago_section

    n = _snote({"a": 1, "b": 2}, section=chicago_section(ChicagoShaft()))
    row = {"kind": "pin", "joint": "pin:J1", "jam": {"safety": 5.0}, "walk": {"safety": 1.0}}
    loads = strength.uniform_loads(10.0, 10.0, "override", "t")
    assert strength.fixes(row, n, loads, BuildConfig()) == [
        "no listed pin or pillar construction clears it at this load: a lower servo torque "
        "limit, or a linkage scaled up (stiffer links, shorter relative spans)"]


def test_link_rows_net_section_bending_foot_lever_and_needs(monkeypatch):
    """Links of a stand-in linkage (its points by hand, no solve) at hand-made sim loads,
    on 3 mm acrylic, 12 mm wide: a two-pin link's net section at its hole (``LINK_KT F /
    ((w - d) t)``), a link of three pins also a beam between its farthest pins (``F L / 4``
    over ``t w^2 / 6``), a foot link off its loaded pin a lever (``F a`` over the net
    section's modulus); acrylic short of jam SF 2 names the aluminium that would hold it."""
    from spiderpig import linkage

    cfg = BuildConfig()             # (validated against the real linkage, before the swap)
    pts = {"J1": (0.0, 0.0), "J3": (30.0, 0.0), "J9": (200.0, 0.0), "J4": (100.0, 0.0),
           "J7": (0.0, 50.0), "J8": (0.0, 90.0)}
    stand_in = SimpleNamespace(
        key="strider", family="strider", crank=("O", "J1"), feet=(("b3", "J4"), ("b7", "J8")),
        links={"b2": (), "b3": (), "b7": (), "b5": ()},
        solve=lambda params: SimpleNamespace(joints_at=lambda t: pts))
    monkeypatch.setattr(linkage, "get", lambda key: stand_in)

    def j(stem, link, walk, jam):
        return {"stem": stem, "links": [f"{link}_leg0"], "walk": {"n": walk},
                "jam": {"n": jam}, "crank": False, "frame": False}

    loads = {"source": "sim", "joints": [j("J4", "b3", 5.0, 50.0), j("J3", "b2", 5.0, 50.0),
                                         j("J9", "b2", 5.0, 50.0), j("J1", "b2", 5.0, 50.0),
                                         j("J7", "b7", 1.0, 20.0)]}
    rows = {r["joint"]: r for r in strength.link_rows(cfg, loads)}
    assert sorted(rows) == ["link:b2", "link:b3", "link:b7"]      # b5: no loaded pin
    t, w, d = 3.0, 12.0, strength.LINK_HOLE
    b3 = rows["link:b3"]
    sig = 2.5 * 50.0 / ((w - d) * t)
    assert b3["jam"] == {"stress_mpa": round(sig, 2), "load_n": 50.0,
                         "safety": round(50.0 / sig, 2)}
    assert (b3["sheet"], b3["thickness_mm"], b3["allowable_mpa"], b3["bending"],
            b3["needs"]) == ("acrylic_3mm", 3.0, 50.0, False, None)
    b2 = rows["link:b2"]
    assert (b2["bending"], b2["pins"]) == (True, ["J1", "J3", "J9"])
    beam = 50.0 * 200.0 / 4 / (t * w * w / 6)               # 34.72 MPa over the crank bore's
    assert b2["jam"]["stress_mpa"] == round(beam, 2)
    assert b2["jam"]["safety"] == round(50.0 / beam, 2)
    al = materials.sheet(materials.FOOT_SHEET)
    assert b2["needs"] == {"sheet": al.key, "jam_safety": round(
        al.yield_mpa / (round(beam, 2) * t / al.thickness), 2)}
    b7 = rows["link:b7"]                       # the foot J8 40 mm off its one loaded pin
    zn = t * (w ** 3 - d ** 3) / (6 * w)
    assert b7["jam"]["stress_mpa"] == round(20.0 * 40.0 / zn, 2)
    assert b7["walk"]["stress_mpa"] == round(1.0 * 40.0 / zn, 2)
    assert strength.link_rows(cfg, dict(loads, source="override")) == []
    assert strength.link_rows(cfg, dict(loads, joints=[])) == []


# -- verify: target rows, cost, strength, sim and mass rows on a stand-in design ----------


def _design(motion=None, budget=None, size=None, allowance=None, **kw):
    spec = SimpleNamespace(motion=motion or {}, budget=budget or {}, size=size or {},
                           allowance_usd=allowance)
    return SimpleNamespace(spec=spec, config=BuildConfig(), store=None, reports={}, **kw)


def test_target_row_informational_met_missed_and_estimated():
    f = vf.target_field("motion", "stride_mm")              # soft by default
    assert vf.target_row(_design(), f, None, "walk") is None
    info = vf.target_row(_design(), f, 50, "walk")
    assert (info.value, info.target, info.passed, info.hard, info.unit) == (
        50.0, None, True, True, "mm/rev")
    d = _design(motion={"stride_mm": Target(min=40.0)})
    miss = vf.target_row(d, f, 30.0, "walk")
    assert (miss.passed, miss.hard, miss.target) == (False, False, ">= 40")
    assert miss.score == pytest.approx(0.75)                  # 1 - 10 / 40
    unmeasured = vf.target_row(d, f, None, "walk")
    assert (unmeasured.passed, unmeasured.detail) == (False, "not measured by this design")
    hard = vf.target_row(_design(motion={"stride_mm": Target(min=40.0, hard=True)}), f,
                         30.0, "walk")
    assert (hard.hard, hard.score) == (True, None)
    est = vf.target_row(_design(motion={"stride_mm": Target(min=40.0, hard=True)}), f,
                        30.0, "walk", estimate=True)
    assert (est.hard, est.score, est.passed) == (False, None, False)


def test_walk_rows_one_per_walk_metric_with_the_rpm():
    m = {"stride_mm": 60.0, "speed_mm_s": 30.0, "bob_mm": 2.0, "slip_rms_mm_per_rev": 0.5,
         "tipping_fraction": 0.0, "rpm_max": 45.0}
    rows = vf.walk_rows(_design(), m)
    assert [r.requirement for r in rows] == ["motion.stride_mm", "motion.speed_mm_s",
                                             "motion.bob_mm", "motion.slip_mm_per_rev",
                                             "motion.tipping_fraction"]
    assert rows[1].detail == "at the servo's no-load 45 rpm"
    assert vf.walk_rows(_design(), {"rpm_max": 45.0}) == []


def _bom_row(key, name, qty=1.0, cost=1.0, verified=True, pack=1, packs=1, vendor="v"):
    return SimpleNamespace(key=key, name=name, qty=qty, cost_usd=cost, verified=verified,
                           same_pack_as=None, pack_qty=pack, packs=packs, vendor=vendor)


def test_cost_row_lower_bound_with_unpriced_items_and_the_allowance():
    priced = [_bom_row("servo", "servo", cost=20.0),
              _bom_row("m3", "M3", 4.0, 2.0, False, pack=100)]
    unpriced = [_bom_row("shim", "shim", 2.0, 0.0), _bom_row("rod", "rod", 5.0, 0.0)]
    bom = SimpleNamespace(purchased=priced + unpriced, unpriced=unpriced, cost_usd=22.0)
    row = vf.cost_row(_design(budget={"cost_usd": Target(max=100.0)}), bom)
    assert (row.value, row.passed, row.hard) == (22.0, False, True)
    assert row.detail.startswith("at least; the target can't be verified while items are "
                                 "unpriced")
    assert ("4 items, the largest servo $20.00; M3 x 4 $2.00 (a pack of 100); 2 unpriced, so "
            "the total is a lower bound: 5 x rod (1 at v); 2 x shim (1 at v); "
            "1 unverified links") in row.detail
    soft = vf.cost_row(_design(budget={"cost_usd": Target(max=100.0, hard=False)}), bom)
    assert soft.passed
    assert soft.detail.startswith("at least (the verdict is on the priced part): ")
    allowed = vf.cost_row(_design(budget={"cost_usd": Target(max=100.0)}, allowance=10.0),
                          bom)
    assert (allowed.value, allowed.passed) == (32.0, True)
    assert allowed.detail.startswith("$22.00 priced + $10.00 allowed (budget.allowance_usd) "
                                     "for the 2 unpriced items: ")
    info = vf.cost_row(_design(), SimpleNamespace(purchased=priced, unpriced=[],
                                                  cost_usd=22.0))
    assert (info.passed, info.target) == (True, None)


def test_cost_floor_rows_refute_a_ceiling_or_inform(monkeypatch):
    monkeypatch.setattr(vf, "cost_floor", lambda d: (150.0, ["servo x 2 $40.00"], ["epoxy"]))
    (row,) = vf._cost_floor_rows(_design())
    assert (row.requirement, row.value, row.hard, row.unit) == (
        "budget.cost_floor_usd", 150.0, False, "USD")
    assert row.detail == ("before a build, from the catalog: servo x 2 $40.00; unpriced: "
                          "epoxy; " + vf.FLOOR_LEAVES_OUT)
    (over,) = vf._cost_floor_rows(_design(budget={"cost_usd": Target(value=100.0)}))
    assert (over.requirement, over.passed) == ("budget.cost_usd", False)  # over 100 + 5
    assert over.detail.startswith("a lower bound already over the target: ")
    (under,) = vf._cost_floor_rows(_design(budget={"cost_usd": Target(max=200.0)}))
    assert under.requirement == "budget.cost_floor_usd"

    def missing(d):
        raise KeyError("servo")

    monkeypatch.setattr(vf, "cost_floor", missing)
    assert vf._cost_floor_rows(_design()) == []


def test_strength_rows_fail_only_on_the_designs_own_loads(monkeypatch):
    err = {"level": "error", "kind": "pin", "joint": "pin:J1", "sf_jam": 0.5, "sf_walk": 4.0,
           "load_jam": 80.0, "message": "pin:J1 jam SF 0.5", "fixes": ["fix A"]}
    warn = {"level": "warning", "message": "crank jam SF 1.5", "fixes": ["fix B"]}
    st = {"rows": [{"jam": {"safety": 0.5}}, {"jam": None}, {"jam": {"safety": 1.5}}],
          "findings": [err, warn]}
    seen = {}

    def loads(config, store, sim):
        seen["sim"] = sim
        return {"source": seen["source"], "note": "n"}

    monkeypatch.setattr(strength, "design_loads", loads)
    monkeypatch.setattr(strength, "check", lambda notes, meta, config, lo: st)
    d = _design(mech=SimpleNamespace(meta={}))
    seen["source"] = "sim"
    rep = VerifyReport("full")
    rows = vf._strength_rows(d, rep, "full")
    assert seen["sim"] is True
    assert [(r.requirement, r.value, r.passed, r.tier, r.hard) for r in rows] == [
        ("strength.joints", 0.5, False, "measured", True),
        ("strength.warnings", 1, True, "measured", False)]
    assert rows[1].detail == "crank jam SF 1.5 (fix: fix B)"
    (f,) = rep.failures
    assert (f.stage, f.code, f.numbers) == ("strength", "joint_overload", {"sf_jam_min": 0.5})
    assert f.culprits[0]["fixes"] == ["fix A"]
    seen["source"] = "fallback"
    rep = VerifyReport("standard")
    rows = vf._strength_rows(d, rep, "standard")
    assert seen["sim"] == "cached"
    assert rep.failures == []
    assert (rows[0].tier, rows[0].hard) == ("estimated", False)
    assert "(an estimate: verify full simulates the design's own)" in rows[0].detail

    def broken(config, store, sim):
        raise RuntimeError("no store")

    monkeypatch.setattr(strength, "design_loads", broken)
    (row,) = vf._strength_rows(d, VerifyReport("full"), "full")
    assert (row.value, row.detail, row.hard) == (None, "no loads: no store", False)


def test_sim_rows_speed_stays_up_and_torque(monkeypatch):
    pytest.importorskip("mujoco")
    from spiderpig.sim import run

    m = {"speed": 40.0, "stride": 55.0, "fell": False, "max_tilt": 4.0, "torque_peak": 0.5,
         "torque_limit": 1.9, "saturates": False}
    monkeypatch.setattr(run, "simulate", lambda config, seconds: "result")
    monkeypatch.setattr(run, "walk_metrics", lambda result: m)
    # the sim's second measurement is never hard, against a hard target too
    d = _design(motion={"speed_mm_s": Target(min=10.0, hard=True),
                        "stride_mm": Target(min=10.0, hard=True)})
    rows = vf._sim_rows(d, VerifyReport("full"))
    assert [(r.requirement, r.value, r.passed, r.hard) for r in rows] == [
        ("sim.speed_mm_s", 40.0, True, False), ("sim.stride_mm", 55.0, True, False),
        ("sim.stays_up", True, True, True), ("sim.torque", 0.5, True, False)]
    assert rows[2].detail == "max tilt 4.0 deg"
    assert rows[3].detail == "peak 0.500 N·m of 1.9 stall"

    def boom(config, seconds):
        raise RuntimeError("bad model")

    monkeypatch.setattr(run, "simulate", boom)
    rep = VerifyReport("full")
    (row,) = vf._sim_rows(_design(), rep)
    assert (row.requirement, row.value, row.passed) == ("sim.run", "failed", False)
    assert rep.failures[0].code == "sim_failed"


def test_mass_estimate_names_what_it_is_made_of(monkeypatch):
    from spiderpig import walk

    monkeypatch.setattr(walk, "side_legs", lambda cfg: [])
    monkeypatch.setattr(walk, "nominal_mass_breakdown", lambda cfg, legs, robot: {
        "links": 100.0, "servos": 110.0, "plates": 200.0, "printed": 30.0, "deck": 80.0,
        "total": 520.0, "note": "nominal"})
    d = _design(size={"mass_g": Target(max=1000.0, hard=True)})
    row = vf._mass_estimate(d, SimpleNamespace(mass_g=None))
    assert (row.requirement, row.value, row.hard, row.tier) == (
        "size.mass_g", 520.0, False, "estimated")
    assert row.detail.startswith(
        "estimated before a build: links 100 g, 2 servos 110 g, frame and centre plates "
        "200 g, printed crank, pillars, pins and ties 30 g, electronics deck 80 g (nominal)")
    assert row.detail.endswith(vf.ESTIMATE_NOTE)

    def fails(cfg, legs, robot):
        raise ValueError("no legs")

    monkeypatch.setattr(walk, "nominal_mass_breakdown", fails)
    assert vf._mass_estimate(_design(), SimpleNamespace(mass_g=None)) is None
    row = vf._mass_estimate(d, SimpleNamespace(mass_g=480.0))
    assert (row.value, row.hard, row.passed) == (480.0, False, True)
    assert row.detail.startswith("the walk model's nominal mass; an estimate")


def test_stack_floor_note_names_the_modules_with_fewer_legs_that_walk(monkeypatch):
    from spiderpig import api

    strides = {"single": 0.0, "double": 40.0}
    monkeypatch.setattr(api, "module_stride", lambda key, module: strides.get(module))
    pr = SimpleNamespace(height_mm=66.5, n_layers=14)
    lk = SimpleNamespace(leg_modules={"single": [0], "double": [0, 1], "quad": [0, 1, 2, 3]})

    def note(module, kind="walker"):
        d = SimpleNamespace(config=replace(BuildConfig(), module=module), kind=kind, lk=lk)
        return vf.stack_floor_note(d, pr)

    head = ("66.5 mm is proven the thinnest for strider's {} module on 3 mm layers (14 "
            "layers, the frame plates included)")
    assert note("quad") == (head.format("quad") + "; a thinner stack needs fewer legs a side: "
                            "of strider's modules with fewer, double walk; the sheet sets "
                            "the layer pitch")
    assert note("double") == (head.format("double") + "; a thinner stack needs fewer legs a "
                              "side, and no module of strider with fewer walks (single stand "
                              "still in the walk model); the sheet sets the layer pitch")
    assert note("single") == (head.format("single") + "; no module of this linkage has fewer "
                              "legs a side; the sheet sets the layer pitch")
    assert note("single", "mechanism") == head.format("single") + "; the sheet sets the " \
                                                                  "layer pitch"


def test_cost_floor_prices_what_every_build_buys(monkeypatch):
    """The servos (one per side), a blank of each sheet no service cuts, the Chicago pins'
    epoxy and the threadlocker the standoff pillars take, less the shop's supplies on hand
    (the default design's sheets are all cut by SendCutSend: its uploads, in no total)."""
    from spiderpig.hardware.bom import ON_HAND
    from spiderpig.hardware.catalog import get

    cfg = BuildConfig()
    total, priced, unpriced = vf.cost_floor(SimpleNamespace(config=cfg))
    servo = get(vf.servos.get(cfg.servo).bom_key).name
    assert priced[0].startswith(f"{servo} x 2 $")
    # (the epoxy's first offer, on the Amazon cart, has no price: named, not counted)
    assert any(line.startswith(get("epoxy_2part").name) for line in priced + unpriced)
    names = " ".join(priced + unpriced)
    for key in ON_HAND:
        assert get(key).name not in names
    for key in (cfg.sheet, cfg.frame_sheet, cfg.crank_sheet):  # cut by a service: its least
        assert any(line.startswith(f"SendCutSend cutting, {get(key).name}: one part at least")
                   for line in priced)                          # cut, not a blank
        assert not any(line.startswith(get(key).name) for line in priced)
    side_total, side_priced, _ = vf.cost_floor(SimpleNamespace(config=replace(
        cfg, robot=False, module="single")))
    assert side_priced[0].startswith(f"{servo} $")
    assert 0 < side_total < total
    assert total == pytest.approx(round(total, 2))
    real = vf.catalog_item
    key = vf.servos.get(cfg.servo).bom_key
    item = real(key)

    def no_price(k):
        return SimpleNamespace(name=item.name, offer=None) if k == key else real(k)

    monkeypatch.setattr(vf, "catalog_item", no_price)
    less, priced2, unpriced2 = vf.cost_floor(SimpleNamespace(config=cfg))
    assert unpriced2 == [item.name, *unpriced]
    assert less < total
    assert not any(line.startswith(item.name) for line in priced2)


def test_envelope_estimate_is_the_sweep_plus_the_plates(monkeypatch):
    from spiderpig.construction import robot

    monkeypatch.setattr(robot, "mid_plane", lambda side: 40.0)
    plan = SimpleNamespace(
        topo=SimpleNamespace(geometry=SimpleNamespace(points={
            "J1": [(0.0, 0.0), (10.0, 0.0)], "J2": [(0.0, 20.0), (5.0, 5.0)]})),
        placed=[SimpleNamespace(layer=-1), SimpleNamespace(layer=0),
                SimpleNamespace(layer=5)],
        spec=SimpleNamespace(pitch=3.0))
    d = _design(side=SimpleNamespace(plan=plan))
    r = max(d.config.params.link_radius, d.config.params.frame_radius)
    rows = vf._envelope_estimate(d)
    assert [(x.requirement, x.value, x.tier, x.hard) for x in rows] == [
        ("size.envelope_x_mm", 10.0 + 2 * r, "estimated", True),     # informational
        ("size.envelope_y_mm", 20.0 + 2 * r, "estimated", True),
        ("size.envelope_z_mm", 2 * (40.0 + 3.0), "estimated", True)]
    assert rows[2].detail.startswith("the two stacks + the chassis + 3 mm of axle heads "
                                     "outside each outer plate; measured after a build")
    side = _design(side=SimpleNamespace(plan=plan))
    side.config = replace(side.config, robot=False, module="single")
    z = vf._envelope_estimate(side)[2]
    assert z.value == 43.0
    assert z.detail.startswith("the stack + the servo on the inner plate + 3 mm")


def test_sim_rows_without_mujoco_say_how_to_get_it(monkeypatch):
    real = vf.importlib.util.find_spec
    monkeypatch.setattr(vf.importlib.util, "find_spec",
                        lambda name, *a: None if name == "mujoco" else real(name, *a))
    (row,) = vf._sim_rows(_design(), VerifyReport("full"))
    assert (row.requirement, row.value, row.passed, row.hard, row.detail) == (
        "sim.mujoco", "not installed", True, False, "pip install mujoco to simulate")


def test_done_scores_soft_rows_and_leaves_estimated_hard_targets_unverified(monkeypatch):
    monkeypatch.setattr(vf.api, "_commit", lambda design, stage, rep, op: (stage, op, rep))
    stride, mass, cost = (vf.target_field("motion", "stride_mm"),
                          vf.target_field("size", "mass_g"),
                          vf.target_field("budget", "cost_usd"))
    targets = [(stride, Target(min=40.0, weight=1.0, hard=False)),
               (mass, Target(max=1000.0, hard=True)), (cost, Target(max=100.0, hard=True))]
    design = SimpleNamespace(spec=SimpleNamespace(targets=lambda: targets))
    rep = VerifyReport("quick", rows=[
        Row("motion.stride_mm", "walk", 30.0, ">= 40", False, "measured", False, score=0.75),
        Row("size.mass_g", "walk", 520.0, "<= 1000", True, "estimated", False, score=None),
        Row("stage.plan", "plan", "14 layers", None, True, "proven")])
    stage, op, out = vf._done(design, rep, 0.0)
    assert (stage, op) == ("verify", "verify:quick")
    assert out.unverified == ["budget.cost_usd", "size.mass_g"]
    assert out.ok                              # a soft miss lowers the score only
    assert out.score == 0.75
    rep.rows.append(Row("budget.cost_usd", "bom", 150.0, "<= 100", False, "estimated", True))
    _, _, out = vf._done(design, rep, 0.0)
    assert not out.ok
    assert out.unverified == ["size.mass_g"]
