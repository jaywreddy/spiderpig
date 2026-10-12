"""The platform's registries (W-platform phase 0): the fab methods, the construction roles,
the material constants, the vendor-neutral CAD cache and the joints' error terms. None of
them moves a part: the identity gate holds the walkers byte-identical."""

from __future__ import annotations

import hashlib
import math

import pytest

from spiderpig import fabmethods
from spiderpig import roles as rolemod
from spiderpig.config import BuildConfig, ParamError

pytestmark = pytest.mark.no_fabricate


# -- fab methods ----------------------------------------------------------------------


def test_fab_methods_keep_the_output_order():
    assert fabmethods.MADE == ("laser", "printed")
    assert fabmethods.KINDS == ("printed", "laser", "purchased")
    assert fabmethods.folders() == ("laser", "print")
    assert fabmethods.get("laser").flat
    assert not fabmethods.get("printed").flat
    assert not fabmethods.get("purchased").made
    with pytest.raises(KeyError, match="unknown fab method"):
        fabmethods.get("milled_by_elves")


def test_a_fab_method_registers_once():
    m = fabmethods.get("laser")
    assert fabmethods.register(m) is m             # the same record again: a no-op
    with pytest.raises(ValueError, match="twice"):
        fabmethods.register(fabmethods.FabMethod("laser", "Lasered", made=True))


# -- materials ------------------------------------------------------------------------


def test_sheets_carry_stiffness_and_expansion():
    from spiderpig.materials import sheet

    acrylic, alu = sheet("acrylic_3mm"), sheet("al5052_2mm")
    assert acrylic.modulus_gpa == pytest.approx(3.2)
    assert alu.modulus_gpa == pytest.approx(69.0)
    assert acrylic.cte_um_per_m_k == pytest.approx(70.0)
    assert alu.cte_um_per_m_k == pytest.approx(23.6)
    # 5052 and 6061 are equally stiff: a stiffer link needs a thicker or wider one
    assert sheet("al6061_2p5mm").modulus_gpa == alu.modulus_gpa


# -- roles ----------------------------------------------------------------------------


def test_the_original_roles_are_config_fields():
    c = BuildConfig()
    for name in ("pillar", "pin", "crank"):
        role = rolemod.get(name)
        assert role.field == name
        assert not role.stored_in_roles
        assert c.role(name) == getattr(c, name)
    assert c.roles == ()
    assert c.applied_roles() == {}
    with pytest.raises(KeyError, match="unknown construction role"):
        rolemod.get("flux_capacitor")


@pytest.fixture
def toy_role(monkeypatch):
    """A later role (kept in ``BuildConfig.roles``) over the axle registry, mechanisms only."""
    role = rolemod.Role("toy", "a toy part", "spiderpig.construction:AXLES", default="chicago",
                        kinds=("mechanism",), help="for the tests")
    monkeypatch.setitem(rolemod.ROLES, "toy", role)
    return role


def test_a_later_role_lives_in_roles(toy_role):
    c = BuildConfig(linkage="dwell_rocker", robot=False, roles=(("toy", "standoff"),))
    assert c.roles == (("toy", "standoff"),)
    assert c.role("toy") == "standoff"
    assert c.applied_roles() == {"toy": "standoff"}
    # the default is dropped: one config per design
    same = BuildConfig(linkage="dwell_rocker", robot=False, roles=(("toy", "chicago"),))
    assert same.roles == ()
    assert same == BuildConfig(linkage="dwell_rocker", robot=False)
    assert same.applied_roles() == {"toy": "chicago"}       # still written in, resolved


def test_a_role_is_refused_where_it_does_not_apply(toy_role):
    with pytest.raises(ParamError, match="has no a toy part"):
        BuildConfig(roles=(("toy", "standoff"),))            # the Strider is a walker
    with pytest.raises(ParamError, match="unknown construction role"):
        BuildConfig(linkage="dwell_rocker", robot=False, roles=(("nope", "x"),))
    with pytest.raises(ParamError, match="config's 'pin' field"):
        BuildConfig(roles=(("pin", "chicago"),))


def test_a_later_role_round_trips_through_the_spec(toy_role):
    from spiderpig.api.store_ops import _config_from_resolved
    from spiderpig.spec import Spec, validate
    from spiderpig.stages.resolve import resolve

    doc = {"kind": "mechanism", "linkage": {"key": "dwell_rocker"},
           "constructions": {"toy": "standoff"}}
    assert validate(doc) == []
    spec = Spec.from_dict(doc)
    assert spec.constructions.roles == {"toy": "standoff"}
    assert spec.to_dict()["constructions"] == {"toy": "standoff"}     # flat, as given
    design = resolve(spec, store=None)
    assert design.config.role("toy") == "standoff"
    assert design.resolved["constructions"]["toy"] == "standoff"
    assert _config_from_resolved(design.resolved) == design.config
    errors = validate({**doc, "constructions": {"toy": "no_such_axle"}})
    assert [e.path for e in errors] == ["constructions.toy"]


def test_walkers_resolve_as_before():
    """No later role applies to the walkers yet: their resolved spec, and so their spec
    hash and every stored plan, is unchanged."""
    from spiderpig.spec import Spec
    from spiderpig.stages.resolve import resolve

    design = resolve(Spec.from_dict({"kind": "walker", "linkage": {"key": "strider"}}),
                     store=None)
    assert set(design.resolved["constructions"]) == {"pillar", "pin", "crank", "heads"}


def test_the_cli_offers_every_later_role(toy_role):
    import argparse

    from spiderpig.config import add_build_args, add_design_args, config_from_args

    p = argparse.ArgumentParser()
    add_design_args(p)
    add_build_args(p)
    args = p.parse_args(["--linkage", "dwell_rocker", "--toy", "standoff"])
    assert config_from_args(args, robot=False).role("toy") == "standoff"


# -- vendor CAD -----------------------------------------------------------------------


def test_cad_import_file_pins_the_hash(monkeypatch, tmp_path):
    from spiderpig.hardware import cad

    monkeypatch.setenv(cad.CACHE_ENV, str(tmp_path / "cache"))
    monkeypatch.setenv(cad.OFFLINE_ENV, "1")
    data = b"ISO-10303-21; a bearing"
    ref = cad.CadRef(url="", sha256=hashlib.sha256(data).hexdigest(), filename="b.step",
                     vendor="mcmaster", part_number="0000K00")
    assert cad.fetch(ref, allow_download=True) is None      # no url: only the cache
    src = tmp_path / "download.step"
    src.write_bytes(data)
    assert cad.import_file(ref, src) == cad.cached_path(ref)
    assert cad.fetch(ref) == cad.cached_path(ref)
    src.write_bytes(b"another file")
    with pytest.raises(ValueError, match="pinned"):
        cad.import_file(ref, src)


def test_the_servos_cad_module_is_the_generic_one():
    from spiderpig.hardware import cad
    from spiderpig.servos import cad as servo_cad
    from spiderpig.servos.spec import CadRef

    assert servo_cad.CadRef is CadRef is cad.CadRef
    assert servo_cad.fetch is cad.fetch
    assert servo_cad.load is cad.load


# -- joints' error terms --------------------------------------------------------------


def test_error_terms_add_in_series():
    from spiderpig.construction.joints import ErrorTerms

    a = ErrorTerms(radial_play_mm=0.1, backlash_deg=0.2, resolution_deg=0.01,
                   stiffness_nm_per_rad=1000.0, source="a")
    b = ErrorTerms(radial_play_mm=0.05, resolution_deg=0.05, stiffness_nm_per_rad=1000.0,
                   source="b")
    s = a + b
    assert s.radial_play_mm == pytest.approx(0.15)
    assert s.backlash_deg == pytest.approx(0.2)
    assert s.resolution_deg == pytest.approx(0.05)
    assert s.stiffness_nm_per_rad == pytest.approx(500.0)
    assert s.source == "a + b"
    assert (a + ErrorTerms()).stiffness_nm_per_rad == 1000.0     # rigid adds nothing
    # 0.1 mm of play rocks a 100 mm link 0.057 deg, on top of the backlash
    assert a.lost_motion_deg(100.0) == pytest.approx(0.2 + math.degrees(0.001))
    assert s.as_dict()["stiffness_nm_per_rad"] == pytest.approx(500.0)
    assert ErrorTerms().as_dict()["stiffness_nm_per_rad"] is None


def test_every_axle_reports_its_error_terms():
    from spiderpig import construction
    from spiderpig.construction.base import Params

    class Ctx:
        params = Params()
        config = None

    for key, axle in construction.AXLES.items():
        terms = axle.error_terms(Ctx())
        assert terms.source.startswith(key)
        assert terms.radial_play_mm > 0          # both plain-hole pivots have play
    assert construction.axle("chicago").error_terms(Ctx()).radial_play_mm == pytest.approx(0.2)
    assert construction.axle("standoff").error_terms(Ctx()).radial_play_mm == pytest.approx(
        Params().running_fit)
