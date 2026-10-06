"""Tests for the printed crankshafts (:mod:`construction.crank`): the keyed one (the default)
and the friction-only one, with every servo.

The Klann single sides they look at come from the fabrication cache (:mod:`tests.cache`);
a test of the crank alone realizes the crank group alone (:func:`tests._construction.
realize_groups`), and the contract's fast cases check the crank and the drive
(``check_side(groups=)``), the whole side in the slow tier."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig import linkage, servos
from spiderpig.config import BuildConfig
from spiderpig.construction import CRANKS as CRANK_REGISTRY
from spiderpig.construction.base import FRAME_OUTER, Build, ConstructionError
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.construction.crank import (
    BHCS,
    NUT_H,
    STANDOFF_KEY,
    CrankRoute,
    KeyedCrank,
    PrintedCrank,
    Run,
    chains_of,
    default_route,
    hex_play,
    route_of,
    standoff_dims,
)
from spiderpig.fabricate import (
    SideDesign,
    design_side,
    fabricate_side,
    side_problem,
    template_for,
)
from spiderpig.hardware.bom import bom_from_mechanism
from spiderpig.servos import cad as cadlib
from spiderpig.servos import model
from spiderpig.shapes import disc
from spiderpig.stack import Axis, Layout, verify_plan
from tests import cache
from tests._construction import overlapping_pairs, realize_groups
from tests.tiers import quick

TEMPLATES = {         # the Klann's: the default linkage is the Strider, whose crank differs
    "single": lambda: linkage.build_module_template("single", linkage="klann"),
    "double": lambda: linkage.build_module_template("double", linkage="klann"),
    "decker": lambda: linkage.build_module_template("decker", linkage="klann"),
    "quad": lambda: linkage.build_module_template("quad", linkage="klann"),
}
SERVOS = servos.available()
OTHERS = [s for s in SERVOS if s != servos.DEFAULT]


@pytest.fixture(scope="module", autouse=True)
def _offline(tmp_path_factory):
    """No downloads in tests: servos are drawn parametrically."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv(cadlib.OFFLINE_ENV, "1")
        mp.setenv(cadlib.CACHE_ENV, str(tmp_path_factory.mktemp("cad")))
        for f in (model.cad_servo, model._servo_part, cadlib._load_cached):
            f.cache_clear()
        yield
        for f in (model.cad_servo, model._servo_part, cadlib._load_cached):
            f.cache_clear()


CRANKS = ["keyed", "printed"]


def _cfg(mode="single", servo=servos.DEFAULT, crank="keyed") -> BuildConfig:
    return BuildConfig(linkage="klann", module=mode, robot=False, pillar="printed",
                       servo=servo, crank=crank)


def _design(mode="single", servo=servos.DEFAULT, crank="keyed"):
    """``(template, design)`` of the Klann ``mode`` side (the plan seeded from the cache)."""
    return cache.cached_design(_cfg(mode, servo, crank))


def _side(t=1.0, servo=servos.DEFAULT, crank="keyed", mode="single"):
    """``(design, the fabricated side at t)`` of the Klann ``mode`` side, from the cache."""
    cfg = _cfg(mode, servo, crank)
    return _design(mode, servo, crank)[1], cache.cached_side(cfg, t)


def _clashes(mech) -> list[tuple[str, str, float]]:
    """Every pair of parts sharing more than 1e-3 mm^3 (a pair whose boxes don't meet
    shares none)."""
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in overlapping_pairs(parts):
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > 1e-3:
            out.append((a, b, round(vol, 4)))
    return out


def _volume(shape) -> float:
    return 0.0 if shape is None else sum(s.volume for s in shape.solids())


# -- the contract ---------------------------------------------------------------------


@pytest.mark.slow
@pytest.mark.parametrize("crank", CRANKS)
@pytest.mark.parametrize("mode", sorted(TEMPLATES))
@pytest.mark.parametrize("t", [0.0, 2.2, 4.38])
def test_crank_stays_inside_its_claims(mode, t, crank):
    """The whole side (the fast tier checks the crank and the drive alone, below)."""
    tmpl, design = _design(mode, crank=crank)
    assert check_side(design, tmpl.freeze_at(t)) == []


@pytest.mark.parametrize("crank", CRANKS)
def test_the_crank_alone_stays_inside_its_claims(crank):
    tmpl, design = _design(crank=crank)
    assert check_side(design, tmpl.freeze_at(2.2), groups=("drive", "crank")) == []


@pytest.mark.slow
@pytest.mark.parametrize("mode", sorted(TEMPLATES))
@pytest.mark.parametrize("servo", OTHERS)
def test_every_servo_couples_inside_the_claims(servo, mode):
    """The whole side (the fast tier checks the crank and the drive alone, below)."""
    tmpl, design = _design(mode, servo)
    assert check_side(design, tmpl.freeze_at(2.2)) == []


@pytest.mark.parametrize("servo", OTHERS)
def test_every_servo_couples_the_crank_inside_the_claims(servo):
    tmpl, design = _design(servo=servo)
    assert check_side(design, tmpl.freeze_at(2.2), groups=("drive", "crank")) == []


# -- the parts ------------------------------------------------------------------------


@pytest.mark.parametrize("crank", CRANKS)
@pytest.mark.parametrize("mode", quick(["single", "double", "quad"], ["single"]))
def test_every_segment_is_one_valid_solid(mode, crank):
    design, mech = _side(1.0, crank=crank, mode=mode)
    segs = [b for b in mech.bodies if b.name.startswith("crank_seg")]
    assert len(segs) == len(_runs(design)) + 1
    for b in mech.bodies:
        if b.name.startswith("crank"):
            assert len(b.part.solids()) == 1, b.name
            assert b.part.is_valid, b.name
            assert b.rigid_with == design.plan.topo.crank_bodies[0]
            assert b.fab == ("printed" if b.name.startswith("crank_seg") else "purchased")
            if b.fab == "purchased":
                assert b.bom_key, b.name


@pytest.mark.parametrize("servo", quick(SERVOS, [servos.DEFAULT]))
@pytest.mark.parametrize("t", quick([1.0, 4.38], [4.38]))
def test_single_side_parts_do_not_intersect(servo, t):
    """Every body, screws, nuts and keys included."""
    _, mech = _side(t, servo)
    assert any(b.name.startswith("servo_screw") for b in mech.bodies)
    assert any(b.name.startswith("crank_nut") for b in mech.bodies)
    assert any(b.name.startswith("crank_key") for b in mech.bodies)
    assert _clashes(mech) == []


@pytest.mark.parametrize("servo", SERVOS)
def test_horn_screws_land_in_the_horn_holes(servo):
    tmpl = _design(servo=servo)[0]
    design, mech = _side(1.0, servo)
    frozen = tmpl.freeze_at(1.0)
    build = Build(design.ctx, design.plan, frozen)
    spec = servos.get(servo)
    pat = spec.horn.pattern
    iface = design.ctx.interfaces["drive"]
    o = build.xy("O")
    theta = design.drive.horn_angle(build)
    holes = [o + pat.pcd / 2 * np.array([math.cos(theta + 2 * math.pi * k / pat.count),
                                         math.sin(theta + 2 * math.pi * k / pat.count)])
             for k in range(pat.count)]
    plate_top = design.plan.z(design.plan.top)[1]
    horn_face = plate_top - spec.horn_face_depth
    horn = mech.body("servo_horn").part
    screws = [b for b in mech.bodies if b.name.startswith("crank_horn_screw")]
    assert len(screws) == pat.count
    used = set()
    for b in screws:
        bb = b.part.bounding_box()
        c = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2])
        k = min(range(len(holes)), key=lambda i: np.linalg.norm(holes[i] - c))
        assert np.linalg.norm(holes[k] - c) < 1e-3
        used.add(k)
        engage = bb.max.Z - horn_face
        assert engage >= min(pat.thread_depth, 1.5) - 1e-6
        assert engage <= min(pat.reach, spec.horn_face_depth) + 1e-6
        assert plate_top + 1e-6 >= bb.max.Z
        # the shank is in the hole, not in the horn's metal
        hole = disc(tuple(holes[k]), pat.thread_d / 2, horn_face, bb.max.Z)
        assert _volume(b.part & hole) > 0.5 * math.pi * (0.97 * pat.thread_d / 2) ** 2 * engage
        assert _volume(b.part & horn) < 1e-3
        assert b.bom_key.startswith("m" + pat.thread[1:].replace(".", "p") + "_")
    assert used == set(range(pat.count))
    # the hub bolts on at the coupling face and never rises above it
    top = mech.body(f"crank_seg{len(_runs(design))}").part.bounding_box()
    assert pytest.approx(plate_top - iface.horn_face_depth) == top.max.Z


def _runs(design) -> tuple[Run, ...]:
    return route_of(design.plan.layout, design.plan.topo.axes_of("crankpin")).runs


@pytest.mark.parametrize("servo", SERVOS)
def test_the_hub_turns_clear_of_the_inner_plate(servo):
    tmpl = _design(servo=servo)[0]
    design, mech = _side(1.0, servo)
    hub = mech.body(f"crank_seg{len(_runs(design))}").part
    plate_bottom = design.plan.z(design.plan.top)[0]
    margin = design.ctx.params.margin
    r = design.ctx.interfaces["drive"].horn_radius
    o = tuple(Build(design.ctx, design.plan, tmpl.freeze_at(1.0)).xy("O"))
    outside = disc(o, 60, plate_bottom - margin + 1e-3, plate_bottom + 3) - disc(
        o, r + 1e-3, plate_bottom - margin, plate_bottom + 4)
    assert _volume(hub & outside) < 1e-6


# -- the crankpin joints ----------------------------------------------------------------


def test_default_route_merges_adjacent_riders_of_one_pin():
    pins = [Axis("M", "crankpin", ("a", "b"), ()), Axis("N", "crankpin", ("c",), ())]
    route = default_route(Layout({"a": 2, "b": 3, "c": 6}, 8, 3.0), pins)
    assert route.runs == (Run("M", 2, 3), Run("N", 6, 6))
    assert route.bearing
    route = default_route(Layout({"a": 2, "b": 5, "c": 3}, 8, 3.0), pins)
    assert route.runs == (Run("M", 2, 2), Run("M", 5, 5), Run("N", 3, 3))


@pytest.mark.parametrize("crank", CRANKS)
@pytest.mark.parametrize("mode", quick(["single", "double", "quad"], ["single"]))
def test_crankpins_are_screwed_through(mode, crank):
    design, mech = _side(1.0, crank=crank, mode=mode)
    gaps = _runs(design)
    construction = {"keyed": KeyedCrank(), "printed": PrintedCrank()}[crank]
    top = 2 if crank == "keyed" else 1                 # the keyed chain's top web: two layers
    for g in gaps:
        screw = mech.body(f"crank_screw_{g.at}")
        nut = mech.body(f"crank_nut_{g.at}")
        assert nut.bom_key == "m3_nut"
        assert screw.bom_key.startswith("m3_bhcs_")      # an ISO 4762 head won't fit 3 mm webs
        sb, nb = screw.part.bounding_box(), nut.part.bounding_box()
        # head under the lower web, tip in the nut, both inside the webs either side
        assert design.plan.z(g.lo - 1)[0] - 1e-6 <= sb.min.Z
        assert design.plan.z(g.hi + top)[1] + 1e-6 >= sb.max.Z
        assert design.plan.z(g.hi + top)[1] + 1e-6 >= nb.max.Z
        assert construction.min_nut_engage - 1e-6 <= sb.max.Z - nb.min.Z
        # the post carries b1 with end play
        post_len = (g.hi - g.lo + 1) * design.ctx.pitch + construction.axial_play
        assert post_len > (g.hi - g.lo + 1) * design.ctx.pitch
    if mode == "double":                               # both legs on one post: one joint
        assert len(gaps) == 1
        assert gaps[0].hi == gaps[0].lo + 1
        if crank == "printed":
            assert mech.body(f"crank_screw_{gaps[0].at}").bom_key == BHCS["3"].key(10)


def test_b1_has_end_play():
    tmpl = _design()[0]
    design, mech = _side(1.0)
    frozen = tmpl.freeze_at(1.0)
    (g,) = _runs(design)
    b1_bottom = design.plan.z(g.lo)[0]
    play = PrintedCrank().axial_play
    post = next(g for g in design.groups if g.name == "crank").dims(design.ctx).post
    xy = tuple(Build(design.ctx, design.plan, frozen).xy(g.at))
    lower = mech.body("crank_seg0").part
    slab = disc(xy, 100, b1_bottom - play + 1e-3, b1_bottom) - disc(xy, post + 0.2,
                                                                     b1_bottom - 1, b1_bottom + 1)
    assert _volume(lower & slab) < 1e-6                # nothing but the post near b1


def test_post_joint_lengths():
    crank = PrintedCrank()
    one = crank.post_joint(3.0, 5.85, 9.0, 12.0)       # one b1 between 3 mm webs
    assert one.screw is BHCS["3"]
    assert one.length == 6
    assert one.engagement >= crank.min_nut_engage
    two = crank.post_joint(3.0, 5.85, 12.0, 15.0)      # two b1s on one post
    assert two.length == 10
    assert two.engagement == pytest.approx(NUT_H)
    thick = crank.post_joint(4.5, 8.85, 13.5, 18.0)    # 4.5 mm sheet: a socket head fits
    assert thick.screw.kind == "shcs"


@pytest.mark.parametrize("crank", CRANKS)
def test_too_thin_sheet_is_refused(crank):
    with pytest.raises(ConstructionError, match="crankpin joint"):
        design_side(TEMPLATES["single"](), BuildConfig(linkage="klann", module="single",
                                                       robot=False, thickness=2.0, crank=crank,
                                                       pillar="printed"))


# -- the keyed crank ------------------------------------------------------------------------

KEYED = [("strider", "double"), ("klann", "quad")]
_KEYED: dict[tuple, tuple] = {}


def _keyed(linkage_key: str, module: str, t: float = 1.0):
    """(template, design, the side at ``t``) of a keyed design (the default before the bolt
    crank, with its printed pillars), once per session."""
    if (linkage_key, module) not in _KEYED:
        cfg = BuildConfig(linkage=linkage_key, module=module, robot=False, crank="keyed",
                          pillar="printed")
        tmpl = template_for(cfg)
        _KEYED[(linkage_key, module)] = tmpl, design_side(tmpl, cfg)
    tmpl, design = _KEYED[(linkage_key, module)]
    return tmpl, design, fabricate_side(design, tmpl.freeze_at(t))


def _segments(design) -> list[range]:
    """The layers of each printed segment, in ``crank_seg<i>`` order (the layers between runs)."""
    in_run = {k for r in _runs(design) for k in range(r.lo, r.hi + 1)}
    out: list[list[int]] = []
    for k in range(design.plan.top):
        if k in in_run:
            continue
        if out and out[-1][1] == k - 1:
            out[-1][1] = k
        else:
            out.append([k, k])
    return [range(a, b + 1) for a, b in out]


def _bbox(mech, name):
    return mech.body(name).part.bounding_box()


@pytest.mark.slow
@pytest.mark.parametrize(("linkage_key", "module"), KEYED)
def test_every_keyed_post_has_a_key_and_every_chain_its_clamp(linkage_key, module):
    """One brass key per run, bottomed in its post's cavity with its float left under the
    socket's ceiling, at least 1.5 mm of hex either side; 1 mm of floor between the socket
    and the nut in the two-layer top web, 0.8 mm between the head and the first cavity."""
    tmpl, design, mech = _keyed(linkage_key, module)
    crank = KeyedCrank()
    af, length = standoff_dims(STANDOFF_KEY)
    z, segs = design.plan.z, _segments(design)
    seg_of = {k: i for i, r in enumerate(segs) for k in r}
    runs = _runs(design)
    keys = [b for b in mech.bodies if b.name.startswith("crank_key")]
    assert len(keys) == len(runs)
    assert {b.bom_key for b in keys} == {STANDOFF_KEY} == {crank.standoff_key}
    for c in chains_of(runs):
        at = c[0].at
        head_top = _bbox(mech, f"crank_screw_{at}").min.Z + BHCS["3"].head_h
        nb = _bbox(mech, f"crank_nut_{at}")
        for r in c:
            kb = _bbox(mech, f"crank_key_{at}_{r.lo}")
            assert pytest.approx(length) == kb.max.Z - kb.min.Z
            post_top = _bbox(mech, f"crank_seg{seg_of[r.hi + 1]}").min.Z   # the web's underside
            cavity, in_socket = post_top - kb.min.Z, kb.max.Z - post_top
            assert cavity >= crank.min_socket - 1e-6
            assert in_socket >= crank.min_socket - 1e-6
            assert length - cavity >= crank.min_socket - 1e-6
            if r is c[0]:
                assert kb.min.Z - head_top >= crank.min_web_floor - 1e-6
            if r is c[-1]:                            # socket ceiling: the key top + its float
                assert nb.min.Z - (kb.max.Z + crank.key_float) >= crank.min_key_floor - 1e-6
                assert z(c[-1].hi + 1)[0] <= nb.min.Z
                assert z(c[-1].hi + 2)[1] + 1e-6 >= nb.max.Z
    bom = {}
    for b in mech.bodies:
        if b.name.startswith("crank") and b.fab == "purchased":
            bom[b.bom_key] = bom.get(b.bom_key, 0) + 1
    if linkage_key == "strider":
        assert bom == {STANDOFF_KEY: 4, "m3_bhcs_16": 2, "m3_nut": 2, "m3_shcs_6": 4}
    assert {r.key: r.qty for r in bom_from_mechanism(mech).purchased}[STANDOFF_KEY] == len(runs)


@pytest.mark.slow
@pytest.mark.parametrize(("linkage_key", "module"), KEYED)
def test_the_clamp_loop_spans_each_chain(linkage_key, module):
    """The nut bears in the chain's highest segment and the head in its lowest, so every
    rider interface between them is clamped (a standoff used as the nut would clamp only
    the segment its post is in)."""
    tmpl, design, mech = _keyed(linkage_key, module)
    segs = _segments(design)
    seg_of = {k: i for i, r in enumerate(segs) for k in r}
    for c in chains_of(_runs(design)):
        at = c[0].at
        lo = _bbox(mech, f"crank_seg{seg_of[c[0].lo - 1]}")
        hi = _bbox(mech, f"crank_seg{seg_of[c[-1].hi + 1]}")
        head_top = _bbox(mech, f"crank_screw_{at}").min.Z + BHCS["3"].head_h
        nut_bottom = _bbox(mech, f"crank_nut_{at}").min.Z
        assert lo.min.Z <= head_top <= lo.max.Z
        assert hi.min.Z <= nut_bottom <= hi.max.Z
        assert seg_of[c[-1].hi + 1] == seg_of[c[-1].hi + 2]  # the two-layer web is one segment


@pytest.mark.slow
@pytest.mark.parametrize(("linkage_key", "module"), KEYED)
@pytest.mark.parametrize("t", [0.0, math.pi / 2, math.pi, 3 * math.pi / 2])
def test_keyed_sides_are_clash_free_round_the_cycle(linkage_key, module, t):
    tmpl, design, mech = _keyed(linkage_key, module, t)
    assert bad_solids(mech) == []
    assert clashes(mech) == []
    assert check_side(design, tmpl.freeze_at(t)) == []


@pytest.mark.slow
def test_the_default_designs_pay_two_layers_for_the_keyed_crank():
    """19 layers (57 mm) on the Strider double, 17 (51 mm) on the Klann quad, proven, each
    chain's top web two layers thick with the crank on its axis in the second. (One more
    than before the 0.080 in frame plates: the horn's face under the inner plate drops the
    hub a layer.)"""
    for (lk, module), (layers, mm) in zip(KEYED, ((19, 57.0), (17, 51.0)), strict=True):
        tmpl, design, _ = _keyed(lk, module)
        plan = design.plan
        assert (plan.top + 1, plan.optimal) == (layers, True)
        # the 3 mm layers, and what the aluminium plates and any gaps add (2026-10-04)
        assert plan.height == pytest.approx(mm + sum(plan.t(k) - 3.0 for k in range(layers))
                                            + sum(plan.gaps.values()))
        run_layers = {k for r in _runs(design) for k in range(r.lo, r.hi + 1)}
        for c in chains_of(_runs(design)):
            k = c[-1].hi + 2
            assert k not in run_layers
            labels = {p.label for p in plan.shapes("crank") if p.layer == k}
            assert f"web {c[0].at}" in labels
            assert labels & {"crank body", "crank hub"}


def test_keyed_post_joint_numbers():
    """The tightest keyed joint at 3 mm layers: socket 2.4, cavity 2.4 (a 4 mm key with 0.8
    of float), 1.0 of floor under the nut, 1.8 over the head, 2.4 mm of thread."""
    crank = KeyedCrank()
    j = crank.post_joint(0.0, 2.85, 6.0, 11.85, first_post_top=6.0)
    assert (j.screw, j.length) == (BHCS["3"], 10)
    assert (j.socket, j.cavity) == pytest.approx((2.4, 2.4))
    assert j.cavity + j.socket - standoff_dims()[1] == pytest.approx(crank.key_float)
    assert (j.key_floor, j.head_floor, j.engagement) == pytest.approx((1.0, 1.8, NUT_H))
    assert crank.least_pitch(2.0) == 3.0
    # a one-layer top web holds no socket under a nut
    assert crank.post_joint(0.0, 2.85, 6.0, 8.85) is None


@pytest.mark.slow
def test_a_five_mm_key_is_refused_for_the_floor_between_two_runs():
    """A 5 mm key's cavity (3.4 mm) overflows a one-layer post into the web below it, 0.05 mm
    over the socket of the run below: KeyedCrank.dims says so; the 4 mm key leaves 1.2."""
    tmpl, design, _ = _keyed("strider", "double")
    with pytest.raises(ConstructionError, match=r"5 mm hex key leaves 0\.05 mm of web"):
        KeyedCrank(standoff_key="m3_hex_standoff_ff_5").dims(design.ctx)
    assert KeyedCrank().dims(design.ctx).post == pytest.approx(4.25)   # 8.5 mm over 6.0


def test_hex_play_of_a_key_in_its_pockets():
    """A 5.0 AF key turns until its corners meet the pockets' flats: 3.13 deg in a 5.15
    pocket, twice that between a post and its web (two pockets); a 5.65 pocket (cut for a
    5.5 kit) 18.1 deg; none when the pocket is no wider than the key."""
    assert hex_play(5.0, 5.15) == pytest.approx(3.13, abs=0.01)
    assert hex_play(5.0, 5.65) == pytest.approx(18.13, abs=0.01)
    assert hex_play(5.0, 5.0) == hex_play(5.0, 4.9) == 0.0
    pressed, floating = KeyedCrank(), CRANK_REGISTRY["keyed_float"]
    assert (pressed.key_fit, floating.key_fit) == ("press", "float")
    assert pressed.pocket_af() == pytest.approx(standoff_dims()[0])
    assert floating.pocket_af() == pytest.approx(standoff_dims()[0] + 0.15)
    assert pressed.key_play() == 0.0
    assert pressed.key_play(0.05) == pytest.approx(2.02, abs=0.01)
    assert floating.key_play() == pytest.approx(6.25, abs=0.01)
    assert KeyedCrank(key_af=5.5).pocket_af() == pytest.approx(5.5)   # a measured kit
    with pytest.raises(ConstructionError, match="key_fit"):
        KeyedCrank(key_fit="glued").pocket_af()


@pytest.mark.slow
@pytest.mark.parametrize(("linkage_key", "module"), KEYED)
def test_pressed_keys_fill_their_pockets_and_the_screws_are_threadlocked(linkage_key, module):
    """The default keyed crank cuts its hex pockets to the key (no play, the meta says so)
    and buys a drop of threadlocker per chain screw; the sliding-fit one neither."""
    tmpl, design, mech = _keyed(linkage_key, module)
    note = mech.meta["crank_key"]
    chains = chains_of(_runs(design))
    assert note["fit"] == "press"
    assert note["play_deg"] == 0.0
    assert note["threadlocker"]
    assert note["pocket_af_mm"] == pytest.approx(note["key_af_mm"])
    assert note["keys"] == len(_runs(design))
    locks = [e for e in mech.bom_extras if e.key == "threadlocker_222"
             and e.where.startswith("crank screw")]
    assert len(locks) == len(chains)
    assert sum(e.qty for e in locks) == pytest.approx(0.01 * len(chains))
    assert CRANK_REGISTRY["keyed_float"].lock_key is None


# -- crank routes: hand-made plans of what the planner may choose ---------------------------

# the printed crank's joints: its one-layer webs are what these layouts leave room for
KLANN = BuildConfig(linkage="klann", robot=False, module="single", crank="printed",
                    pillar="printed")
TROTBOT = BuildConfig(robot=False, module="single", linkage="trotbot", crank="printed",
                      pillar="printed")
TROT_LAYERS = {"b4": 2, "b3": 3, "b5": 4, "b2": 5, "b1": 6, "b6": 7}
ROUTES = {  # config, link layers, top, route, detour point, screw length per joint
    # b1's run goes on over an empty layer above it (below it)
    "past b1": (KLANN, {"b1": 2, "b3": 4, "b4": 4, "b2": 5}, 7,
                CrankRoute((Run("M", 2, 3),)), None, {"M": 10}),
    "under b1": (KLANN, {"b1": 3, "b3": 4, "b4": 4, "b2": 5}, 7,
                 CrankRoute((Run("M", 2, 3),)), None, {"M": 10}),
    # a detour opposite the crankpin in an empty layer; its lower web shares layer 3 with
    # the upper web of b1's run
    "detour": (KLANN, {"b1": 2, "b3": 3, "b4": 3, "b2": 5}, 7,
               CrankRoute((Run("M", 2, 2), Run("X", 4, 4))), ("X", 14.0, 180.0),
               {"M": 6, "X": 6}),
    "no bearing": (KLANN, {"b1": 2, "b3": 3, "b4": 3, "b2": 4}, 6,
                   CrankRoute((Run("M", 2, 2),), bearing=False), None, {"M": 6}),
    # TrotBot's B8 (b6) sweeps across O: it sits in the crankpin's run beside the post (with
    # pin J8's head and cap, which would sweep across a web), b4 at the run's foot
    "trotbot": (TROTBOT, TROT_LAYERS, 12, CrankRoute((Run("J1", 2, 8),)), None, {"J1": 25}),
    # ... or b4 on a run of its own, its upper web on the next run's lower web: one screw
    "trotbot chain": (TROTBOT, TROT_LAYERS, 12,
                      CrankRoute((Run("J1", 2, 2), Run("J1", 5, 8))), None, {"J1": 25}),
}
_ROUTED: dict[str, tuple] = {}


def _routed(config, layers, top, route, point=None):
    """One side planned by hand along ``route`` (``point``: a detour's (name, r, degrees))."""
    tmpl = template_for(config)
    ctx, groups, problem = side_problem(tmpl, config)
    if point is not None:
        ctx.topo.add_crank_point(*point)
    plan = problem.plan(layers, top, {"crank": route})
    assert verify_plan(plan) == []
    return tmpl, SideDesign(config, ctx, groups, plan)


def _route_case(name: str):
    """(template, design, the side fabricated at t = 4.38) for a case of :data:`ROUTES`."""
    if name not in _ROUTED:
        config, layers, top, route, point, _ = ROUTES[name]
        tmpl, design = _routed(config, layers, top, route, point)
        _ROUTED[name] = tmpl, design, fabricate_side(design, tmpl.freeze_at(4.38))
    return _ROUTED[name]


def _part(mech, name: str):
    return next(b.part for b in mech.bodies if b.name == name)


_CRANK_ONLY: dict[str, tuple] = {}


def _route_crank(name: str):
    """(template, design, the crank group's :class:`Realized` at t = 4.38) for a case of
    :data:`ROUTES`: the crank realized alone, as :func:`fabricate_side` builds it."""
    if name not in _CRANK_ONLY:
        if name in _ROUTED:
            tmpl, design, _ = _ROUTED[name]
        else:
            config, layers, top, route, point, _ = ROUTES[name]
            tmpl, design = _routed(config, layers, top, route, point)
        _CRANK_ONLY[name] = tmpl, design, realize_groups(design, 4.38, ["crank"],
                                                         tmpl.freeze_at(4.38))
    return _CRANK_ONLY[name]


@pytest.mark.slow
@pytest.mark.parametrize("case", sorted(ROUTES))
def test_every_route_builds_inside_its_claims(case):
    tmpl, design, mech = _route_case(case)
    assert check_side(design, tmpl.freeze_at(2.2)) == []
    assert clashes(mech) == []
    crank = [b for b in mech.bodies if b.name.startswith("crank")]
    for b in crank:
        assert len(b.part.solids()) == 1, b.name
        assert b.part.is_valid, b.name
    runs, lengths = ROUTES[case][3].runs, ROUTES[case][-1]
    assert len([b for b in crank if b.name.startswith("crank_seg")]) == len(runs) + 1
    # one joint per point (a chain of runs along one point shares its screw)
    screws = {b.name: b.bom_key for b in crank if b.name.startswith("crank_screw")}
    assert screws == {f"crank_screw_{at}": f"m3_bhcs_{n}" for at, n in lengths.items()}
    nuts = {b.name: b.bom_key for b in crank if b.name.startswith("crank_nut")}
    assert nuts == {f"crank_nut_{at}": "m3_nut" for at in lengths}
    bought = {r.key for r in bom_from_mechanism(mech).purchased}
    assert {*screws.values(), *nuts.values()} <= bought


@pytest.mark.parametrize(("case", "rider", "bare"), [("past b1", 2, 3), ("under b1", 3, 2)])
def test_a_run_past_its_rider_is_bare_with_end_play_at_the_rider_only(case, rider, bare):
    tmpl, design, mech = _route_crank(case)
    z, play = design.plan.z, PrintedCrank().axial_play
    xy = tuple(Build(design.ctx, design.plan, tmpl.freeze_at(4.38)).xy("M"))
    lower, upper = _part(mech, "crank_seg0"), _part(mech, "crank_seg1")
    # the post runs bare through the layer b1 isn't in
    post = disc(xy, 3.01, *z(bare))
    assert _volume(lower & (disc(xy, 100, *z(bare)) - post)) < 1e-6
    assert _volume(lower & post) > 0.9 * math.pi * (3**2 - 1.7**2) * design.ctx.pitch
    # end play on b1's side only: the web above the run is set back if b1 is under it,
    # the web below if b1 is over it
    assert pytest.approx(z(4)[0] + (play if rider == 3 else 0)) == upper.bounding_box().min.Z
    under = disc(xy, 100, z(2)[0] - play + 1e-3, z(2)[0] - 1e-3) - disc(xy, 3.01, 0, 100)
    assert (_volume(lower & under) > 1.0) == (rider == 3)


def test_a_crank_point_turns_with_the_crank():
    config, layers, top, route, point, _ = ROUTES["detour"]
    tmpl, design = _routed(config, layers, top, route, point)
    for t in (0.0, 1.0, 4.38):
        build = Build(design.ctx, design.plan, tmpl.freeze_at(t))
        x, m = build.xy("X") - build.xy("O"), build.xy("M") - build.xy("O")
        assert np.linalg.norm(x) == pytest.approx(14.0)
        assert np.dot(x, m) / np.linalg.norm(m) == pytest.approx(-14.0)   # opposite M


def test_without_the_bearing_the_crank_ends_at_its_lowest_web():
    tmpl, design, got = _route_crank("no bearing")
    bottom = _part(got, "crank_seg0").bounding_box().min.Z
    assert pytest.approx(design.plan.z(1)[0]) == bottom   # the web's face, no journal stub
    assert FRAME_OUTER not in got.cuts                 # no journal hole in the outer plate


def test_a_joint_no_stock_screw_fits_is_reported():
    """TrotBot's shortest run (b4, J8's head, b1, b6, J8's cap): webs 21 mm apart. The plan's
    z checks it (the crank's ``check_route``), so the layering doesn't even make a plan; the
    construction would say so too."""
    from spiderpig.stack import PlanReject

    layers = {"b4": 2, "b2": 3, "b1": 4, "b5": 4, "b6": 5, "b3": 5}
    with pytest.raises(PlanReject, match=r"no stock screw fits the crankpin joint at J1"):
        _routed(TROTBOT, layers, 9, CrankRoute((Run("J1", 2, 6),)))


def test_pockets_that_meet_are_reported():
    """A detour 4 mm from the crankpin: X's head pocket would cut M's nut out of layer 3."""
    config, layers, top, route, _, _ = ROUTES["detour"]
    tmpl, design = _routed(config, layers, top, route, ("X", 22.0, 8.0))
    with pytest.raises(ConstructionError, match="pockets for M and X"):
        fabricate_side(design, tmpl.freeze_at(1.0))


# -- the Strider decker and quad on the hex crank (the debug of 2026-10-05) -----------------

def _hex_crank():
    from spiderpig.construction.crank import BoltCrank

    return BoltCrank().for_sheet("al6061_2p5mm")


def test_a_span_between_stock_hex_lengths_opens_the_chain_gaps():
    """No stock hex standoff fits a span of 26.5 mm capped in the hub plate (25 mm is 1.5
    short, 30 mm stands 3.5 past the lower plate): the gaps along the chain open, the one
    over the lowest web first, each up to the 4 mm a gap holds, to the next stock length;
    what is already in a gap counts."""
    from spiderpig.stack import GAP_MAX

    c = _hex_crank()
    t = c.web_t
    assert c.fit_hex(26.5, t, t, out_hi_max=0.0, capped=True) is None
    j = c.hex_gap_fit(26.5, [(9, 2.8), (10, 2.5)], t, t, out_hi_max=0.0, capped=True)
    assert j is not None
    assert j.length == 30.0
    gaps = dict(j.gaps)
    assert gaps[9] == pytest.approx(GAP_MAX)                 # the lowest web's gap first
    assert sum(g - h for g, h in ((gaps[9], 2.8), (gaps.get(10, 2.5), 2.5))) == pytest.approx(
        j.span - 26.5)
    assert j.gap == gaps[9]
    assert j.out_lo <= c.protrude_max + 1e-9
    assert j.out_hi == 0.0
    # no room left in the gaps: none
    assert c.hex_gap_fit(26.5, [(9, GAP_MAX)], t, t, out_hi_max=0.0, capped=True) is None


def test_the_upper_end_of_a_hex_pin_uses_the_air_over_its_plate():
    """What a standoff stands past its plates goes where the two gaps' needs come out even,
    the upper stack using the air over its plate in its layer first (a 0.100 in plate in a
    3 mm layer: 0.46 mm)."""
    c = _hex_crank()
    t = c.web_t
    span = 3.0 + 2 * 3.0 + t + 0.5                    # a 12 mm standoff stands ~0.46 past
    plain = c.fit_hex(span, t, t)
    air = c.fit_hex(span, t, t, air_hi=3.0 - t)
    extra = plain.length - span
    assert plain.out_lo + plain.out_hi == pytest.approx(extra)
    assert air.out_lo + air.out_hi == pytest.approx(extra)
    assert max(air.out_lo, air.out_hi - (3.0 - t)) <= max(plain.out_lo, plain.out_hi) + 1e-9
    assert air.out_hi >= air.out_lo


@pytest.mark.slow
@pytest.mark.parametrize("module", ["decker", "quad"])
def test_the_strider_decker_and_quad_plan_on_the_hex_crank(module):
    """The Strider's decker and quad default to the hex-standoff crank (they were on the
    round friction crank, whose hub chain screw no assembly order drives): each plans, every
    chain's standoff a stock length at the plan's z, the hub chain capped (no screw over the
    hub plate), and the side builds inside its claims."""
    from spiderpig.construction.crank import chains_of as chains

    cfg = BuildConfig(module=module)
    assert cfg.crank == "bolt"
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg, advise=False)
    plan = design.plan
    assert verify_plan(plan, tmpl) == []
    crank = next(g for g in design.groups if g.name == "crank")
    c = crank.construction.resolve(design.ctx)
    assert c.hex
    route = route_of(plan.layout, [a.name for a in design.ctx.topo.axes_of("crankpin")])
    hub = max(k for k in range(plan.top) if any(
        p.label == "crank hub" for p in plan.shapes("crank") if p.layer == k))
    for ch in chains(route.runs):
        capped = ch[-1].hi + 1 == hub and c.hub_capped(design.ctx, ch[0].at)
        j = c.chain_fit_web(plan.layout, ch[0].lo, ch[-1].hi, t=c.web_t, hub=hub,
                            capped=capped)
        assert j is not None, ch
        assert j.gap == 0.0, ch                       # a stock length as the plan stands
        if ch[-1].hi + 1 == hub:
            assert capped
            assert not j.screw_hi
