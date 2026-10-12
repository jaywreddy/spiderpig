"""The ball-bearing pivots (:mod:`construction.pivots.ball`, 2026-10-11): ``ball`` (an
MR84ZZ in every rider link of a Chicago-screw pin) and ``ball_pillar`` (an MR106ZZ in every
link on a standoff pillar), their vendor models (:mod:`hardware.cad`, McMaster-Carr's STEP
through ``spiderpig cad``) and what they report.

Checked on the Klann single (the fabrication cache, :mod:`tests.cache`): the bearings, the
holes, the contract, clashes, the BOM and the wobble; and the ways they refuse a design."""

from __future__ import annotations

import json
from dataclasses import replace

import pytest

from spiderpig import construction
from spiderpig.config import BuildConfig
from spiderpig.construction import ConstructionError
from spiderpig.construction.axle import AxleGroup
from spiderpig.construction.base import Build, Params, Realized
from spiderpig.construction.contract import check_side
from spiderpig.hardware.bom import bom_from_mechanism
from tests import cache

T = 1.0


def _config(**kw) -> BuildConfig:
    return BuildConfig(**{"linkage": "klann", "module": "single", "robot": False, **kw})


@pytest.fixture(scope="module")
def ball_design():
    """The ball-pin Klann single's design alone (its plan from the cache; nothing built)."""
    return cache.cached_design(_config(pin="ball"))


@pytest.fixture(scope="module")
def ball():
    cfg = _config(pin="ball")
    tmpl, design = cache.cached_design(cfg)
    build = Build(design.ctx, design.plan, tmpl.freeze_at(T))
    fab = cache.cached_side(cfg, T)
    pins = [g for g in design.groups if isinstance(g, AxleGroup) and not g.pillar]
    return tmpl, design, build, fab, pins


# -- the parts and their numbers (no fabrication) -------------------------------------


@pytest.mark.no_fabricate
def test_the_bearings_are_in_the_catalog_with_a_vendor_model():
    from spiderpig.hardware import cad
    from spiderpig.hardware.catalog import get

    for key, (d, od, w) in {"bearing_mr84zz": (4, 8, 3), "bearing_mr106zz": (6, 10, 3)}.items():
        it = get(key)
        assert it.category == "bearing"
        assert (it.dims["id"], it.dims["od"], it.dims["w"]) == (d, od, w)
        assert it.dims["radial_clearance_mm"] == [0.002, 0.013]
        mcm = next(o for o in it.offers if "mcmaster.com" in o.url)
        assert mcm.price_usd is None                      # nothing priced from memory there
        assert it.offers[0].price_usd is not None          # the BOM's: a generic pack,
        assert "estimate" in it.offers[0].note             # said to be an estimate
        ref = cad.part_cad(key)
        assert ref is not None
        assert (ref.vendor, ref.sha256, ref.url) == ("mcmaster", "", "")


@pytest.mark.no_fabricate
def test_dims_seat_the_bearing_and_keep_the_shaft(ball_design):
    _, design = ball_design
    ctx = design.ctx
    pin = construction.axle("ball").dims(ctx, False)
    plain = construction.axle("chicago").dims(ctx, False)
    assert pin.seat == pytest.approx((8.0 + 0.2) / 2)      # the bearing's bonded seat
    assert pin.axle == plain.axle                          # the barrel the rings turn on
    assert pin.neck == plain.neck
    pillar = construction.axle("ball_pillar").dims(
        replace(ctx, params=Params(link_radius=6.6)), True)
    assert pillar.seat == pytest.approx((10.0 + 0.2) / 2)


@pytest.mark.no_fabricate
def test_the_ball_pivots_refuse_what_they_cannot_seat(ball_design):
    _, design = ball_design
    ctx = design.ctx
    with pytest.raises(ConstructionError, match=r"link_radius 6\.6 would") as e:
        construction.axle("ball_pillar").dims(ctx, True)  # 6 mm links: no wall round 10 mm
    assert list(e.value.changes) == [("link_radius", 6.0, 6.6)]
    with pytest.raises(ConstructionError, match=r"wider than the 2\.9 mm links"):
        construction.axle("ball").dims(replace(ctx, pitch=2.9), False)
    with pytest.raises(ConstructionError, match="link pin only"):
        construction.axle("ball").dims(ctx, True)
    with pytest.raises(ConstructionError, match="pillar only"):
        construction.axle("ball_pillar").dims(replace(ctx, params=Params(link_radius=6.6)),
                                              False)


@pytest.mark.no_fabricate
def test_the_error_terms_are_the_bearings(ball_design):
    _, design = ball_design
    ctx = design.ctx
    ball_terms = construction.axle("ball").error_terms(ctx)
    plain = construction.axle("chicago").error_terms(ctx)
    assert ball_terms.radial_play_mm == pytest.approx(0.05 + 0.013)   # shaft gap + balls
    assert ball_terms.radial_play_mm < plain.radial_play_mm / 3
    assert ball_terms.axial_play_mm == plain.axial_play_mm           # the same barrel
    pillar = construction.axle("ball_pillar").error_terms(ctx)
    assert pillar.radial_play_mm == pytest.approx(0.03 + 0.013)
    assert pillar.source.startswith("ball_pillar")
    # the tilt: the bore rocking on the barrel over 3 mm, plus the rings' free misalignment
    import math

    from spiderpig.construction.pivots.ball import internal_tilt

    assert math.degrees(internal_tilt("bearing_mr84zz")) == pytest.approx(0.533, abs=1e-3)
    assert ball_terms.tilt_deg == pytest.approx(math.degrees(math.atan2(0.05, 3.0)) + 0.533,
                                                abs=2e-3)


# -- the fabricated side ------------------------------------------------------------


def test_every_rider_seats_one_bearing(ball):
    _, _, build, fab, pins = ball
    bearings = {b.name: b for b in fab.bodies if "_bearing_" in b.name}
    riders = [(g, m) for g in pins for m in g.axis.members if m != _host(build, g)]
    assert len(bearings) == len(riders)
    for g, m in riders:
        b = bearings[f"{g.name.replace(':', '_')}_bearing_{m}"]
        assert (b.fab, b.bom_key, b.rigid_with) == ("purchased", "bearing_mr84zz", m)
        z0, z1 = build.z(build.layers[m])
        bb = b.part.bounding_box()
        assert z0 - 1e-6 <= bb.min.Z                  # inside its link's layer
        assert z1 + 1e-6 >= bb.max.Z
        assert pytest.approx(8.0, abs=1e-6) == bb.size.X


def _host(build, group) -> str:
    from spiderpig.construction.pivots.common import host_of

    return host_of(build, group)


def test_a_riders_bore_is_the_bearings_seat(ball):
    _, _, build, _, pins = ball
    for g in pins:
        got = g.realize(build, Realized())
        host = _host(build, g)
        for m in g.axis.members:
            ds = [c.d for c in got.cuts[m]]
            assert ds == [pytest.approx(4.15 if m == host else 8.2)], (g.name, m)


def test_the_ball_pins_stay_inside_their_claims(ball):
    tmpl, design, *_, pins = ball
    assert check_side(design, tmpl.freeze_at(4.38), groups=[g.name for g in pins]) == []


def test_the_bom_buys_the_bearings_and_their_epoxy(ball):
    fab = ball[3]
    bom = bom_from_mechanism(fab)
    rows = {r.key: r for r in bom.purchased}
    n = sum(1 for b in fab.bodies if "_bearing_" in b.name)
    assert rows["bearing_mr84zz"].qty == n
    assert "epoxy_2part" in rows


def test_the_wobble_is_the_bearings(ball):
    *_, fab, pins = ball
    for g in pins:
        note = fab.meta["wobble"][g.name]
        assert note["bearing_len_mm"] == pytest.approx(3.0)
        for e in note["links"]:
            assert e["clearance_mm"] in (0.0, pytest.approx(0.0779))
            assert e["bearing_mm"] == pytest.approx(3.0)        # over the bearing's width
    chicago = cache.cached_side(_config(), T).meta["wobble"]
    for g in pins:      # a rider never tilts more on its bearing than in a plain hole
        assert fab.meta["wobble"][g.name]["worst_deg"] <= chicago[g.name]["worst_deg"] + 1e-9


@pytest.mark.slow
def test_a_ball_pillar_side_builds_and_holds_its_contract():
    cfg = _config(pillar="ball_pillar", params=Params(link_radius=6.6))
    tmpl, design = cache.cached_design(cfg)
    assert check_side(design, tmpl.freeze_at(T)) == []
    fab = cache.cached_side(cfg, T)
    pillars = [g for g in design.groups if isinstance(g, AxleGroup) and g.pillar]
    n = sum(len(g.axis.members) for g in pillars)
    assert sum(1 for b in fab.bodies if b.bom_key == "bearing_mr106zz") == n
    for g in pillars:
        assert all(e["clearance_mm"] == pytest.approx(0.051)
                   for e in fab.meta["wobble"][g.name]["links"])


# -- the vendor model ---------------------------------------------------------------


def _step(tmp_path, od: float, bore: float, w: float, name: str):
    """A ring exported as STEP, lying on its side (axis along x), off the origin: what a
    vendor's file may look like."""
    from build123d import Axis, Location, export_step

    from spiderpig.shapes import ring

    part = ring((0.0, 0.0), od, bore, 0.0, w).rotate(Axis.Y, 90).moved(Location((5, 7, -3)))
    path = tmp_path / name
    export_step(part, str(path))
    return path


def test_an_imported_model_is_aligned_and_size_checked(monkeypatch, tmp_path):
    from spiderpig.construction.pivots.ball import bearing_solid
    from spiderpig.hardware import cad

    monkeypatch.setenv(cad.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.delenv(cad.PART_CAD_ENV, raising=False)
    ref = cad.part_cad("bearing_mr84zz")
    assert ref is not None
    assert cad.state(ref) == ("parametric", "")
    good = _step(tmp_path, 8.0, 4.0, 3.0, "vendor.step")
    path = cad.import_file(ref, good)
    assert cad.state(ref) == ("cad", cad.sha256_file(path))
    bb = bearing_solid("bearing_mr84zz", (10.0, 20.0), 1.0).bounding_box()
    assert (pytest.approx(1.0), pytest.approx(4.0)) == (bb.min.Z, bb.max.Z)
    assert (pytest.approx(10.0), pytest.approx(20.0)) == (bb.center().X, bb.center().Y)
    assert pytest.approx(8.0) == bb.size.X
    # the wrong part (a 9 mm OD) is never drawn: the catalog's ring instead (re-imported in
    # this process: the model memo is keyed by the file's hash, never served stale)
    cad.import_file(ref, _step(tmp_path, 9.0, 4.0, 3.0, "wrong.step"))
    assert pytest.approx(8.0) == \
        bearing_solid("bearing_mr84zz", (0, 0), 0.0).bounding_box().size.X
    assert cad.load_axisymmetric(ref, 8.0, 3.0) is None
    monkeypatch.setenv(cad.PART_CAD_ENV, "0")
    cad.import_file(ref, good)
    assert cad.load_axisymmetric(ref, 8.0, 3.0) is None      # vendor models switched off


@pytest.mark.no_fabricate
def test_the_fab_cache_keys_on_the_models_the_pins_draw(monkeypatch, tmp_path):
    from spiderpig.fabcache import parts_cad_state
    from spiderpig.hardware import cad

    monkeypatch.setenv(cad.CACHE_ENV, str(tmp_path / "cache"))
    assert parts_cad_state(_config()) == ()                 # the default pins draw none
    assert parts_cad_state(_config(pin="ball")) == (("bearing_mr84zz", ("parametric", "")),)
    ref = cad.part_cad("bearing_mr84zz")
    assert ref is not None
    src = tmp_path / "b.step"
    src.write_bytes(b"ISO-10303-21; a bearing")
    cad.import_file(ref, src)
    got = parts_cad_state(_config(pin="ball"))
    assert got[0][1][0] == "cad"
    monkeypatch.setenv(cad.PART_CAD_ENV, "0")
    assert parts_cad_state(_config(pin="ball"))[0][1] == ("parametric", "")


@pytest.mark.no_fabricate
def test_spiderpig_cad_imports_and_lists(monkeypatch, tmp_path, capsys):
    from spiderpig.hardware import cad
    from spiderpig.tools import cad as cad_cli

    monkeypatch.setenv(cad.CACHE_ENV, str(tmp_path / "cache"))
    src = tmp_path / "57155K000.STEP"
    src.write_bytes(b"ISO-10303-21; an MR84")
    assert cad_cli.main(["import", "bearing_mr84zz", str(src), "--part-number", "57155K000"]) == 0
    assert cad_cli.main(["list"]) == 0
    out = capsys.readouterr().out
    assert "part 57155K000" in out
    assert "bearing_mr106zz" in out
    assert "parametric (not in the cache)" in out
    ref = cad.part_cad("bearing_mr84zz")
    assert ref is not None
    meta = json.loads((cad.cached_path(ref).with_suffix(".step.json")).read_text())
    assert meta["part_number"] == "57155K000"
    bad = tmp_path / "x.step"
    bad.write_bytes(b"not a step file")
    with pytest.raises(ValueError, match="not a STEP file"):
        cad_cli.main(["import", "bearing_mr84zz", str(bad)])
    monkeypatch.setenv(cad_cli.STATE_ENV, str(tmp_path / "no-session.json"))
    assert cad_cli.main(["fetch"]) == 2          # no McMaster session saved: log in first


@pytest.mark.no_fabricate
def test_a_model_imported_since_makes_the_build_stale(monkeypatch, tmp_path):
    from spiderpig import uptodate
    from spiderpig.hardware import cad

    monkeypatch.setenv(cad.CACHE_ENV, str(tmp_path / "cache"))
    assert uptodate.part_files(_config()) == []              # the default pins draw none
    before = uptodate.part_files(_config(pin="ball"))
    assert [r[1] for r in before] == [False]
    assert uptodate.part_files_unchanged(before)
    ref = cad.part_cad("bearing_mr84zz")
    assert ref is not None
    src = tmp_path / "b.step"
    src.write_bytes(b"ISO-10303-21; a bearing")
    cad.import_file(ref, src)
    assert not uptodate.part_files_unchanged(before)          # imported since: rebuild
    now = uptodate.part_files(_config(pin="ball"))
    assert uptodate.part_files_unchanged(now)
    src.write_bytes(b"ISO-10303-21; another bearing, longer")
    cad.import_file(ref, src)
    assert not uptodate.part_files_unchanged(now)             # re-imported: rebuild


def test_a_ball_pin_robot_buys_a_bearing_per_rider_per_side_and_guides_them():
    from spiderpig.construction.assembly import assembly_steps

    cfg = BuildConfig(linkage="klann", module="single", pin="ball")
    fab = cache.cached_robot(cfg, T)
    _, design = cache.cached_design(cfg)
    n = sum(1 for b in fab.bodies if b.bom_key == "bearing_mr84zz")
    side = sum(1 for b in cache.cached_side(replace(cfg, robot=False), T).bodies
               if b.bom_key == "bearing_mr84zz")
    assert n == 2 * side > 0
    rows = {r.key: r for r in bom_from_mechanism(fab).purchased}
    assert rows["bearing_mr84zz"].qty == n
    steps = assembly_steps(fab, design)
    bench = [s for s in steps if any("_bearing_" in p for p in s.adds)]
    assert bench
    for s in bench:                      # each bench step bonds only bearings, before its layer
        assert all("_bearing_" in p for p in s.adds)
    # every bearing is added once (assembly._check), each in a step before its link's
    first: dict[str, int] = {}
    for s in steps:
        for p in s.adds:
            first.setdefault(p.split("#")[0], s.number)
    for b in fab.bodies:
        if b.bom_key == "bearing_mr84zz":
            assert first[b.name] < first[b.rigid_with], b.name


def test_a_bearing_weighs_its_catalog_mass():
    from spiderpig.hardware.mass import material_of
    from spiderpig.mechanism import Body
    from spiderpig.servos import get as servo

    b = Body("pin_C_bearing_b1", fab="purchased", bom_key="bearing_mr84zz")
    assert material_of(b, "acrylic_3mm", None, servo("sts3215")) == ("steel", 0.0, 0.62)

