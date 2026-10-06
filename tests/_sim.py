"""The sim and bake tests' seams (P6, ``docs/agentlib/TESTING.md``): MuJoCo models and
fabrications at the model's reference angle from the test cache, so a test steps a model
without fabricating the robot (20-45 s a design) or building its MJCF (3-4 s) again.

* :func:`robot_at_ref` -- the robot of a config at :data:`spiderpig.sim.mjcf.T_REF` (the
  angle :func:`spiderpig.sim.mjcf.fabricated` and the bake fabricate at), a **private
  copy** of :func:`tests.cache.cached_robot`'s: meshing a part stores its triangulation
  on the shape, and a shape keeps its first one (``mesh.py``), so the MJCF's hulls (0.5 mm)
  and the bake's meshes (0.1 mm) must never mesh the same shapes in one process.
* :func:`mjcf_of` -- ``build_mjcf(cfg, params)`` of that robot (:func:`spiderpig.sim.mjcf
  .set_fabricated`), cached on disk beside the fabrications (``CACHE_DIR/sim/``: keyed by
  the engine version like every entry, so it is always the engine's own model).
* :func:`seed` -- that MJCF compiled as :func:`spiderpig.sim.mjcf.load_model`'s answer
  (:func:`spiderpig.sim.mjcf.adopt_mjcf`), so ``load_model`` / ``simulate`` /
  ``steering_check`` / ``LiveSim`` of the config step it at once; ``steering=True`` adds
  :func:`spiderpig.sim.run.steering_check`'s verdict to the meta, as the server's build
  job does (``server.app._build_mjcf_job``), cached on disk too.
* :func:`recorded_mjcf` -- the recorded MJCF of a design (``tests/fixtures/sim/``): the
  XML (gzip, base64) and the meta of an export, a fast test's input whatever the engine.

The model built from a cached (BREP round-tripped) robot is byte for byte the one built
from a fresh fabrication (measured on the Klann single, the Strider single and the Klann
quad; ``test_sim.py::test_recorded_mjcf_current`` checks it against a fresh build).
"""

from __future__ import annotations

import base64
import gzip
import hashlib
import json
import tempfile
from dataclasses import asdict, replace
from pathlib import Path

from tests import cache

_MEMO: dict = {}


def private(mech):
    """A copy of ``mech`` whose parts are shapes of its own (a BinTools BREP round trip, as
    the cache stores them: every double, location and shared sub-shape kept)."""
    with tempfile.TemporaryDirectory(prefix="spiderpig-sim-") as d:
        cache.dump_mechanism(mech, Path(d))
        return cache.load_mechanism(Path(d))


def robot_at_ref(cfg):
    """A private copy of the robot of ``cfg`` at the model's reference angle (cached)."""
    from spiderpig.sim.mjcf import T_REF

    return private(cache.cached_robot(cfg, T_REF))


def side_at_ref(cfg):
    """A private copy of one side of ``cfg`` at the bake's reference angle (cached)."""
    from spiderpig.bake import T_REF

    return private(cache.cached_side(cfg, T_REF))


def _params_tag(params) -> str:
    return hashlib.sha256(repr(sorted(asdict(params).items())).encode()).hexdigest()[:12]


def _entry(cfg, params) -> Path:
    return cache.cache_dir() / "sim" / cache._slug(f"{cfg.key}_{_params_tag(params)}")


def _build(cfg, params) -> tuple[str, dict]:
    from spiderpig.sim import mjcf

    mjcf.set_fabricated(cfg, robot_at_ref(cfg))
    return mjcf.build_mjcf(cfg, params)


def mjcf_of(cfg, params=None) -> tuple[str, dict]:
    """``(xml, meta)``: :func:`spiderpig.sim.mjcf.build_mjcf` of ``cfg`` (robot) and
    ``params`` on the cached robot, from ``CACHE_DIR/sim/`` when there. The meta is what
    JSON gives back (as an exported model's ``.json``); one pair per key per process:
    never mutate it."""
    from spiderpig.sim.mjcf import SimParams

    cfg, params = replace(cfg, robot=True), params or SimParams()
    memo = ("mjcf", cfg, params)
    if memo in _MEMO:
        return _MEMO[memo]
    if not cache.enabled():
        xml, meta = _build(cfg, params)
        _MEMO[memo] = xml, json.loads(json.dumps(meta))
        return _MEMO[memo]
    entry = _entry(cfg, params)
    with cache._locked(entry):
        if not (entry / "meta.json").is_file():
            xml, meta = _build(cfg, params)

            def write(d: Path) -> None:
                (d / "model.xml").write_text(xml)
                (d / "meta.json").write_text(json.dumps(meta))

            cache._publish(entry, write)
        _MEMO[memo] = ((entry / "model.xml").read_text(),
                       json.loads((entry / "meta.json").read_text()))
    return _MEMO[memo]


def steering_of(cfg, params=None) -> dict:
    """:func:`spiderpig.sim.run.steering_check` of :func:`mjcf_of`'s model (cached on disk:
    MuJoCo is deterministic, the check is ~15 s of stepping)."""
    from spiderpig.sim.mjcf import SimParams
    from spiderpig.sim.run import steering_check

    cfg, params = replace(cfg, robot=True), params or SimParams()
    memo = ("steering", cfg, params)
    if memo in _MEMO:
        return _MEMO[memo]
    xml, meta = mjcf_of(cfg, params)

    def run() -> dict:
        return json.loads(json.dumps(steering_check(cfg, params, xml=xml, meta=meta)))

    if not cache.enabled():
        _MEMO[memo] = run()
        return _MEMO[memo]
    path = _entry(cfg, params) / "steering.json"
    with cache._locked(path):
        if not path.is_file():
            cache._write_text(path, json.dumps(run()))
        _MEMO[memo] = json.loads(path.read_text())
    return _MEMO[memo]


def seed(cfg, params=None, *, steering: bool = False):
    """``(model, meta)``: :func:`mjcf_of`'s model adopted as :func:`spiderpig.sim.mjcf
    .load_model`'s answer for ``cfg`` and ``params`` (its meta a copy of the cached one,
    which sessions may write to: ``LiveSim`` fills in ``steering`` and ``kinematic``);
    ``steering``: with the steering check's verdict in the meta, as the server's build job
    hands it over."""
    from spiderpig.sim import mjcf

    cfg, params = replace(cfg, robot=True), params or mjcf.SimParams()
    xml, meta = mjcf_of(cfg, params)
    if cfg not in mjcf._FABRICATED:
        # a model asked for with params nobody seeded is built from the cached robot (3-5 s),
        # not from a fresh fabrication (20-45 s)
        mjcf.set_fabricated(cfg, robot_at_ref(cfg))
    model, have = mjcf.adopt_mjcf(cfg, params, xml, json.loads(json.dumps(meta)))
    if steering and "steering" not in have:
        have["steering"] = json.loads(json.dumps(steering_of(cfg, params)))
    return model, have


def job_output(cfg) -> tuple[str, dict]:
    """What ``server.app._build_mjcf_job(cfg)`` returns (the MJCF and its meta with the
    steering check), from the cache: a fresh meta each call."""
    xml, meta = mjcf_of(cfg)
    return xml, {**json.loads(json.dumps(meta)), "steering": steering_of(cfg)}


# ---------------------------------------------------------------------------
# bakes
# ---------------------------------------------------------------------------


def bake(out: Path, cfg, **kw) -> str:
    """:func:`spiderpig.bake.bake_gltf` of ``cfg`` into ``out`` (keyword arguments passed
    on), its ``bake_gltf`` log (INFO: the profile summary) returned."""
    import logging

    from spiderpig.bake import bake_gltf

    lines: list[str] = []
    handler = logging.Handler()
    handler.emit = lambda record: lines.append(record.getMessage())
    log = logging.getLogger("bake_gltf")
    log.addHandler(handler)
    level = log.level
    log.setLevel(logging.INFO)
    try:
        bake_gltf(out, cfg, **kw)
    finally:
        log.removeHandler(handler)
        log.setLevel(level)
    return "\n".join(lines) + "\n"


def baked(cfg, n_frames: int, duration_s: float = 1.0) -> tuple[Path, str]:
    """``(glb, log)``: :func:`bake` of ``cfg`` from the cached fabrication at the bake's
    reference angle (a private copy: the bake meshes it) and the cached design, from
    ``CACHE_DIR/bakes/`` when there (keyed by the engine version, the config and the
    clip). Byte for byte the bake from scratch (measured on every case of
    ``test_bake_gltf.py``; its ``test_a_fresh_bake_is_the_cached_one`` checks it). Read
    the file, never write it."""
    memo = ("bake", cfg, n_frames, float(duration_s))
    if memo in _MEMO:
        return _MEMO[memo]

    def make(d: Path) -> str:
        mech = robot_at_ref(cfg) if cfg.robot else side_at_ref(cfg)
        _, side = cache.cached_design(cfg)
        return bake(d / "walker.glb", cfg, n_frames=n_frames, duration_s=duration_s,
                    fabricated=mech, side=side)

    if not cache.enabled():
        d = Path(tempfile.mkdtemp(prefix="spiderpig-bake-"))
        _MEMO[memo] = d / "walker.glb", make(d)
        return _MEMO[memo]
    entry = cache.cache_dir() / "bakes" / cache._slug(f"{cfg.key}_{n_frames}_{duration_s!r}")
    with cache._locked(entry):
        if not entry.is_dir():
            cache._publish(entry, lambda d: (d / "bake.log").write_text(make(d)))
    _MEMO[memo] = entry / "walker.glb", (entry / "bake.log").read_text()
    return _MEMO[memo]


# ---------------------------------------------------------------------------
# recorded MJCFs (tests/fixtures/sim/mjcf_<name>.json)
# ---------------------------------------------------------------------------


def mjcf_doc(xml: str, meta: dict) -> dict:
    """An MJCF and its meta as a recorded fixture's data: the XML gzipped (deterministic:
    no timestamp) and base64'd beside its digest, the meta as it is."""
    return {"xml_sha256": hashlib.sha256(xml.encode()).hexdigest(), "xml_bytes": len(xml),
            "xml_gz_b64": base64.b64encode(gzip.compress(xml.encode(), mtime=0)).decode(),
            "meta": meta}


def mjcf_from_doc(doc: dict) -> tuple[str, dict]:
    """The ``(xml, meta)`` of a recorded MJCF (its digest checked)."""
    xml = gzip.decompress(base64.b64decode(doc["xml_gz_b64"])).decode()
    if hashlib.sha256(xml.encode()).hexdigest() != doc["xml_sha256"]:
        raise ValueError("the recorded MJCF doesn't match its digest")
    return xml, doc["meta"]


RECORDED = {"klann_single": ("klann", "single"), "strider_single": ("strider", "single")}
"""The recorded MJCFs: fixture name -> (linkage, module), each the default build."""


def fresh_mjcf(name: str) -> dict:
    """A recorded MJCF's data made from the engine: a fresh fabrication (not the cache's)
    and its MJCF built from scratch, in a process of its own (so this process's model
    caches, which other tests seeded, are left alone): the currency test's ``make``."""
    import subprocess
    import sys

    with tempfile.TemporaryDirectory(prefix="spiderpig-sim-") as d:
        out = Path(d) / "doc.json"
        subprocess.run([sys.executable, "-c", "import sys; from tests._sim import _fresh_main; "
                        "_fresh_main(sys.argv[1], sys.argv[2])", name, str(out)],
                       check=True, cwd=cache.REPO)
        return json.loads(out.read_text())


def _fresh_main(name: str, out: str) -> None:
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.sim import mjcf

    key, module = RECORDED[name]
    cfg = BuildConfig(linkage=key, module=module)
    mjcf.set_fabricated(cfg, fabricate(template_for(cfg), cfg, mjcf.T_REF))
    Path(out).write_text(json.dumps(mjcf_doc(*mjcf.build_mjcf(cfg))))


def recorded_mjcf(name: str) -> tuple[str, dict]:
    """The recorded MJCF ``name`` (:data:`RECORDED`): ``(xml, meta)``."""
    return mjcf_from_doc(cache.recorded("sim", f"mjcf_{name}", lambda: fresh_mjcf(name)))
