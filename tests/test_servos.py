"""Tests for :mod:`servos`: the data, the model fetcher, the models and the mount."""

from __future__ import annotations

import hashlib
import math
import pathlib
import re
import zipfile
from dataclasses import replace

import numpy as np
import pytest
from build123d import Cylinder, Location

from spiderpig import servos
from spiderpig.hardware import catalog
from spiderpig.servos import cad as cadlib
from spiderpig.servos import model
from spiderpig.servos.mount import servo_to_world
from spiderpig.servos.spec import CadRef
from tests.conftest import clear_model_caches

KEYS = servos.available()


@pytest.fixture
def offline(monkeypatch, tmp_path):
    """No downloads and an empty model cache (the session's is shared)."""
    monkeypatch.setenv(cadlib.OFFLINE_ENV, "1")
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    clear_model_caches()
    yield tmp_path
    clear_model_caches()


# -- data ---------------------------------------------------------------------------


def test_the_expected_servos_are_registered():
    assert {"sts3215", "xl430_w250", "xl330_m288"} <= set(KEYS)
    assert servos.DEFAULT in KEYS


@pytest.mark.parametrize("key", KEYS)
def test_every_servo_can_drive_a_crank_and_be_bought(key):
    s = servos.get(key)
    assert s.continuous
    item = catalog.get(s.bom_key)
    assert item.category == "servo"
    assert item.offers
    assert item.offers[0].url.startswith("https://")
    assert any(o.verified for o in item.offers)
    assert s.torque_kgcm
    assert s.voltage
    assert s.interface
    assert s.sources


@pytest.mark.parametrize("key", KEYS)
def test_every_servo_has_a_horn_pattern_and_mount_holes(key):
    s = servos.get(key)
    pat = s.horn.pattern
    assert pat.count >= 3
    assert pat.pcd > 0
    assert pat.thread_d > 0
    assert pat.hole_d > pat.thread_d                 # a clearance hole for the thread
    assert pat.pcd / 2 + pat.hole_d / 2 < s.horn.diameter / 2
    assert pat.thread_depth
    assert pat.reach
    assert pat.reach <= s.horn.thickness + 2.5
    assert len(s.mount) >= 4
    assert len(s.rear_mount) >= 4
    for mh in s.mount + s.rear_mount:
        assert mh.screw
        assert re.match(r"^m\d", mh.screw)
    L, W, H = s.body
    near, far = s.axis_offset - L / 2, s.axis_offset + L / 2
    assert near < 0 < far                            # the axis is on the body
    for mh in s.mount:
        assert near < mh.x < far
        assert abs(mh.y) < W / 2
    assert s.rear_z == pytest.approx(s.mount_face_z - H)
    assert s.horn_face_depth > 0


@pytest.mark.parametrize("key", KEYS)
def test_every_servo_names_a_pinned_model(key):
    for ref in servos.get(key).cads:
        assert re.fullmatch(r"[0-9a-f]{64}", ref.sha256)
        assert ref.url.startswith("https://")
        assert len(ref.transform) == 16
        m = np.array(ref.transform, dtype=float).reshape(4, 4)
        assert np.linalg.det(m[:3, :3]) == pytest.approx(1.0)   # a rotation, no mirror



# -- model fetcher ---------------------------------------------------------------------


def _ref(tmp_path, data: bytes, **kw) -> CadRef:
    src = tmp_path / "src" / "part.step"
    src.parent.mkdir(parents=True, exist_ok=True)
    src.write_bytes(data)
    base = dict(url=src.as_uri(), sha256=hashlib.sha256(data).hexdigest(), filename="part.step")
    base.update(kw)
    return CadRef(**base)


def test_fetch_caches_a_file_that_matches_its_hash(monkeypatch, tmp_path):
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.delenv(cadlib.OFFLINE_ENV, raising=False)
    ref = _ref(tmp_path, b"ISO-10303-21; a model")
    path = cadlib.fetch(ref)
    assert path == cadlib.cached_path(ref)
    assert path.read_bytes() == b"ISO-10303-21; a model"
    (tmp_path / "src" / "part.step").unlink()        # the cache answers from now on
    assert cadlib.fetch(ref) == path


def test_fetch_refuses_a_file_that_does_not_match(monkeypatch, tmp_path):
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.delenv(cadlib.OFFLINE_ENV, raising=False)
    ref = replace(_ref(tmp_path, b"the real model"), sha256="0" * 64)
    assert cadlib.fetch(ref) is None
    assert not cadlib.cached_path(ref).exists()
    # a tampered cache file isn't used either
    good = _ref(tmp_path, b"the real model")
    cadlib.fetch(good)
    cadlib.cached_path(good).write_bytes(b"tampered")
    (tmp_path / "src" / "part.step").unlink()
    assert cadlib.fetch(good) is None


def test_offline_never_downloads(monkeypatch, tmp_path):
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.setenv(cadlib.OFFLINE_ENV, "1")
    ref = _ref(tmp_path, b"a model")
    assert cadlib.fetch(ref) is None
    assert cadlib.fetch(replace(ref, url="https://invalid.example/nothing.step")) is None
    monkeypatch.setenv(cadlib.OFFLINE_ENV, "0")
    assert cadlib.fetch(ref) is not None


def test_fetch_extracts_a_zip_member(monkeypatch, tmp_path):
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.delenv(cadlib.OFFLINE_ENV, raising=False)
    member = b"the servo model"
    archive = tmp_path / "model.zip"
    with zipfile.ZipFile(archive, "w") as z:
        z.writestr("docs/readme.txt", "hello")
        z.writestr("ST.step", member)
    ref = CadRef(url=archive.as_uri(), sha256=hashlib.sha256(member).hexdigest(),
                 filename="ST.step", member="ST.step",
                 archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    assert cadlib.fetch(ref).read_bytes() == member
    wrong = replace(ref, filename="other.step", archive_sha256="f" * 64)
    assert cadlib.fetch(wrong) is None



def test_load_never_raises(monkeypatch, tmp_path):
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.delenv(cadlib.OFFLINE_ENV, raising=False)
    ref = _ref(tmp_path, b"not a STEP file")
    assert cadlib.load(ref) is None


# -- models -------------------------------------------------------------------------


@pytest.mark.parametrize("key", KEYS)
def test_parametric_servo_offline(offline, key):
    s = servos.get(key)
    part = model.parametric_servo(s)
    assert len(part.solids()) == 1
    assert part.is_valid
    bb = part.bounding_box()
    L, W, _ = s.body
    assert pytest.approx(s.axis_offset - L / 2) == bb.min.X
    assert pytest.approx(s.axis_offset + L / 2) == bb.max.X
    assert pytest.approx(W / 2) == bb.max.Y
    # without its horn nothing stands beyond the face the servo rests on but the panel,
    # the spline and whatever passes through the horn
    front = max((r.height for r in s.front_reliefs if r.solid), default=0.0)
    top = max(s.mount_face_z + front, s.spline_top)
    if model.center_boss_on_servo(s):
        top = max(top, s.horn_bottom + s.horn.center_boss[1])
    assert pytest.approx(top, abs=0.02) == bb.max.Z
    got = model.servo_part(s, cad=False)
    assert got.is_valid
    assert len(got.solids()) == 1
    # offline with an empty cache, every servo falls back to the parametric model
    assert model.servo_part(s).volume == pytest.approx(got.volume)


def test_the_sts3215_model_lines_up_when_cached(offline, monkeypatch):
    """Checks the manufacturer model's transform if `mise run fetch-cad` has cached it."""
    monkeypatch.setenv(cadlib.CACHE_ENV, str(pathlib.Path.home() / ".cache" / "spiderpig" / "cad"))
    s = servos.get("sts3215")
    if cadlib.fetch(s.cad, allow_download=False) is None:
        pytest.skip("STS3215 model not cached (run `mise run fetch-cad`)")
    clear_model_caches()
    part = model.cad_servo(s)
    assert part is not None
    assert part.is_valid
    bb = part.bounding_box()
    assert pytest.approx((-10.2, 35.2), abs=0.1) == (bb.min.X, bb.max.X)
    assert s.seat_height + 2.5 > bb.max.Z         # the fused horn is gone (panel top at 2.6)


@pytest.mark.parametrize("key", KEYS)
def test_horn_has_its_screw_holes(key):
    s = servos.get(key)
    horn = model.horn_part(s)
    assert horn.is_valid
    assert len(horn.solids()) == 1
    pat = s.horn.pattern
    face = s.horn_bottom
    for k in range(pat.count):
        a = math.radians(pat.angle_deg) + 2 * math.pi * k / pat.count
        probe = Cylinder(0.97 * pat.thread_d / 2, 4).moved(
            Location((pat.pcd / 2 * math.cos(a), pat.pcd / 2 * math.sin(a), face - 2)))
        inter = horn & probe
        assert inter is None or sum(x.volume for x in inter.solids()) < 1e-6
    # and a screw beside a hole would bite
    a = math.radians(pat.angle_deg)
    r = pat.pcd / 2 + pat.thread_d
    probe = Cylinder(0.5, 2).moved(Location((r * math.cos(a), r * math.sin(a), face - 1)))
    inter = horn & probe
    assert inter is not None
    assert sum(x.volume for x in inter.solids()) > 0


def test_servo_to_world_puts_the_output_on_o_face_down():
    o, u, face_z = (12.0, -3.0), (0.6, 0.8), 21.5
    m = servo_to_world(o, u, face_z)
    assert m @ np.array([0, 0, 0, 1.0]) == pytest.approx([12.0, -3.0, 21.5, 1.0])
    assert m[:3, 2] == pytest.approx([0, 0, -1])         # output face down
    assert m[:3, 0] == pytest.approx([0.6, 0.8, 0])      # servo +x along u
    assert np.linalg.det(m[:3, :3]) == pytest.approx(1.0)
    # the horn's outer face ends up horn_face_depth below the plate the servo stands on
    s = servos.get("sts3215")
    plate_top = 21.0
    m = servo_to_world(o, u, plate_top + s.mount_face_z)
    assert (m @ np.array([0, 0, s.horn_bottom, 1.0]))[2] == pytest.approx(
        plate_top - s.horn_face_depth)


# -- mount ---------------------------------------------------------------------------


@pytest.mark.parametrize("key", KEYS)
def test_drive_interface_couples_below_the_plate(design, key):
    _, d = design("single", key)
    iface = d.ctx.interfaces["drive"]
    s = servos.get(key)
    assert iface.horn_face_depth >= d.ctx.pitch - 1e-9
    assert iface.horn_radius == pytest.approx(s.horn.diameter / 2)
    assert iface.screw_count == s.horn.pattern.count
    assert iface.screw_pcd == pytest.approx(s.horn.pattern.pcd)
    spacer = d.drive.spacer(d.ctx)
    assert iface.horn_face_depth == pytest.approx(s.horn_face_depth + spacer)
    # the bolt crank's hub is whole plates: the spacer puts the horn's face on a layer
    # boundary under the 3.175 mm aluminium inner plate (the STS3215's 3.2 mm -> 6.175, the
    # XL430's 2 -> 3.175; the XL330's 3 -> 6.175: 0.175 mm is too thin to print)
    plate = d.ctx.sheet_t("frame")
    n = (iface.horn_face_depth - plate) / d.ctx.pitch
    assert n == pytest.approx(round(n))
    assert iface.horn_layers == round(n)
    assert spacer > 0


@pytest.mark.parametrize("key", KEYS)
def test_mount_screws_clear_the_crank_and_are_claimed(design, side, key):
    _, d = design("single", key)
    drive, ctx = d.drive, d.ctx
    screws = drive.front_screws(ctx)
    assert len(screws) >= 2
    hub = next(p.shape.r for p in d.plan.shapes("crank") if p.label == "crank hub")
    iface = ctx.interfaces["drive"]
    if iface.horn_layers >= 1:      # the hub a layer under the horn: the heads meet the horn
        hub = iface.horn_radius
    for _, mh, sk, _ in screws:
        assert math.hypot(mh.x, mh.y) - sk.head_d / 2 >= hub + ctx.params.margin
    if key == "sts3215":
        # the two far front screws: the hub stepped down a layer clears all four heads
        # (2026-10-04), but the near holes (r 13.19) leave the inner plate 1.01 mm of web to
        # the horn's hole, under 1 x t (the assembly audit of 2026-10-04, Params.
        # servo_screw_web_t); the far ones 2.05 to the raised panel's relief
        assert sorted({mh.x for _, mh, _, _ in screws}) == [29.0]
        webs = {mh.x: drive.screw_web(ctx, mh) for mh in drive.spec.mount}
        assert webs[8.3] == pytest.approx(1.01, abs=0.01)
        assert webs[29.0] == pytest.approx(2.05, abs=0.01)
        assert min(drive.screw_web(ctx, mh) for _, mh, _, _ in screws) >= ctx.sheet_t("frame")
        every = drive.front_screws(replace(ctx, params=replace(ctx.params,
                                                               servo_screw_web_t=0.0)))
        assert len(every) == 4
    heads = [p for p in d.plan.shapes("drive") if p.label == "servo screw head"]
    assert {p.layer for p in heads} == {d.plan.top - 1}
    assert {p.shape.at for p in heads} == {name for name, *_ in screws}
    bodies = {b.name: b for b in side("single", 1.0, key).bodies}
    for i in range(len(screws)):
        b = bodies[f"servo_screw{i}"]
        assert b.fab == "purchased"
        assert b.bom_key == screws[i][1].screw
        bb = b.part.bounding_box()
        assert d.plan.z(d.plan.top)[1] < bb.max.Z      # into the servo
        assert d.plan.z(d.plan.top)[0] > bb.min.Z      # head under the plate


def test_the_verifier_sees_the_screw_heads(design):
    """Negative control: a screw head moved onto the crank hub is a violation."""
    from spiderpig.stack import Geometry, verify_plan

    single, d = design("single", crank="keyed", pillar="printed")   # its hub under the plate
    plan = d.plan
    assert verify_plan(plan, single) == []              # the fixed points carry over
    pts = dict(plan.topo.geometry.points)
    name = next(n for n in pts if n.startswith("servo.screw"))
    pts[name] = pts["O"][0] + np.array([12.0, 0.0])     # just outside the horn hole
    broken = replace(plan, topo=replace(plan.topo, geometry=Geometry(pts)))
    assert any("servo screw head" in v and "crank hub" in v for v in verify_plan(broken))


def test_no_models_are_checked_in():
    """Manufacturer models are downloaded at build time (``mise run fetch-cad``), never vendored."""
    root = pathlib.Path(__file__).resolve().parents[1]
    assert not list((root / "spiderpig" / "servos").rglob("*.st*p"))


def test_a_derived_record_is_cached_beside_the_model(monkeypatch, tmp_path):
    """What is derived from a manufacturer's model (which solids the strip keeps) is
    recorded beside the download and read back by the next process."""
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    ref = _ref(tmp_path, b"not needed: the record is what is cached")
    path = cadlib.prepared_path(ref, "strip v1")
    assert path.parent == tmp_path / "cache"
    assert path.suffix == ".json"
    assert cadlib.prepared_path(ref, "strip v2") != path
    assert cadlib.read_prepared(path) is None
    cadlib.write_prepared(path, {"solids": 3, "keep": [0, 2]})
    assert cadlib.read_prepared(path) == {"solids": 3, "keep": [0, 2]}
    path.write_text("garbage")
    assert cadlib.read_prepared(path) is None       # a bad file is as good as none


def test_the_strip_uses_the_recorded_indices(monkeypatch, tmp_path):
    """``cad_servo`` strips by the recorded indices (no bounding boxes), records them when
    there is no record, and ignores a record of another solid count."""
    from build123d import Box, Compound, export_step

    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.delenv(cadlib.OFFLINE_ENV, raising=False)
    horn = Box(2, 2, 2).moved(Location((0, 0, 6)))
    body = Compound(children=[Box(10, 10, 10), horn, Box(3, 3, 3).moved(Location((20, 0, 0)))])
    src = tmp_path / "src" / "m.step"
    src.parent.mkdir(parents=True)
    export_step(body, str(src))
    data = src.read_bytes()
    ref = CadRef(url=src.as_uri(), sha256=hashlib.sha256(data).hexdigest(), filename="m.step",
                 strip=((-1.0, -1.0, 5.0, 1.0, 1.0, 7.0),))
    spec = replace(servos.get("sts3215"), cad=ref)
    clear_model_caches()
    got = model.cad_servo(spec)
    assert got.volume == pytest.approx(1000 + 27)
    path = cadlib.prepared_path(ref, model._strip_key(ref))
    doc = cadlib.read_prepared(path)
    assert doc["solids"] == 3
    assert len(doc["keep"]) == 2

    def no_boxes(shape, ref):
        raise AssertionError("the recorded indices should have been used")

    monkeypatch.setattr(model, "strip_indices", no_boxes)
    clear_model_caches()
    assert model.cad_servo(spec).volume == pytest.approx(1027)
    monkeypatch.undo()
    monkeypatch.setenv(cadlib.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.delenv(cadlib.OFFLINE_ENV, raising=False)
    cadlib.write_prepared(path, {"solids": 99, "keep": [0]})        # not this model's
    clear_model_caches()
    assert model.cad_servo(spec).volume == pytest.approx(1027)
    assert cadlib.read_prepared(path)["solids"] == 3
    clear_model_caches()
