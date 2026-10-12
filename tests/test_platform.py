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
    assert fabmethods.made() == ("laser", "printed", "milled")
    assert fabmethods.kinds() == ("printed", "laser", "milled", "purchased")
    assert fabmethods.folders() == ("laser", "print", "mill")
    assert fabmethods.flat("laser")
    assert not fabmethods.flat("printed")
    assert not fabmethods.flat(None)
    assert (fabmethods.stock("laser"), fabmethods.stock("printed")) == ("sheet", "filament")
    assert not fabmethods.get("purchased").made
    with pytest.raises(KeyError, match="unknown fab method"):
        fabmethods.get("milled_by_elves")


def test_a_registered_fab_method_reaches_every_listing(monkeypatch):
    m = fabmethods.FabMethod("etched", "Etched", made=True, order=3, flat=True,
                             stock="sheet", folder="etch")
    monkeypatch.setitem(fabmethods.METHODS, "etched", m)
    assert fabmethods.made() == ("laser", "printed", "milled", "etched")
    assert fabmethods.kinds() == ("printed", "laser", "milled", "etched", "purchased")
    assert fabmethods.folders() == ("laser", "print", "mill", "etch")
    assert fabmethods.flat("etched")


def test_a_fab_method_registers_once_and_bodies_name_one():
    from spiderpig.mechanism import Body

    m = fabmethods.get("laser")
    assert fabmethods.register(m) is m             # the same record again: a no-op
    with pytest.raises(ValueError, match="twice"):
        fabmethods.register(fabmethods.FabMethod("laser", "Lasered", made=True, order=1))
    assert Body("plate", fab="laser").fab == "laser"
    with pytest.raises(ValueError, match="unknown fab method 'print'"):
        Body("ring", fab="print")


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


def test_the_material_tables_name_the_same_materials():
    from spiderpig.hardware.mass import CTE_UM_PER_M_K, DENSITY, MODULUS_GPA

    assert set(DENSITY) == set(MODULUS_GPA) == set(CTE_UM_PER_M_K)


def test_a_sheet_of_an_unknown_material_has_no_made_up_constants(monkeypatch):
    from spiderpig import materials

    real = materials.get

    from types import SimpleNamespace

    item = SimpleNamespace(name="a G10 sheet", dims={"thickness": 1.6, "material": "g10"})
    monkeypatch.setattr(materials, "get", lambda k: item if k == "g10_test" else real(k))
    materials.sheet.cache_clear()
    try:
        with pytest.raises(KeyError, match="no modulus_gpa for material 'g10'"):
            materials.sheet("g10_test")
    finally:
        materials.sheet.cache_clear()


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


TOYS = {"spring": object(), "damper": object()}
"""A toy role's registry (no construction declares ``roles``: each serves the role)."""


@pytest.fixture
def toy_role(monkeypatch):
    """A later role (kept in ``BuildConfig.roles``), one per drive, optional, mechanisms
    only."""
    role = rolemod.Role("toy", "toy part", "tests.test_platform:TOYS", ("mechanism",),
                        default="spring", per="drive", optional=True, help="for the tests")
    monkeypatch.setitem(rolemod.ROLES, "toy", role)
    return role


MECH = {"linkage": "dwell_rocker", "robot": False}


def test_a_role_names_its_kinds_and_a_valid_per():
    with pytest.raises(TypeError):
        rolemod.Role("x", "x", "tests.test_platform:TOYS")      # kinds: required
    with pytest.raises(ValueError, match="linkage kinds"):
        rolemod.Role("x", "x", "tests.test_platform:TOYS", ())
    with pytest.raises(ValueError, match="per is one of"):
        rolemod.Role("x", "x", "tests.test_platform:TOYS", ("mechanism",), per="leg")


def test_a_later_role_lives_in_roles(toy_role):
    c = BuildConfig(**MECH, roles=(("toy", "damper"),))
    assert c.roles == (("toy", "damper"),)
    assert c.role("toy") == "damper"
    assert c.applied_roles() == {"toy": "damper"}
    # the default is dropped: one config per design
    same = BuildConfig(**MECH, roles=(("toy", "spring"),))
    assert same.roles == ()
    assert same == BuildConfig(**MECH)
    assert same.applied_roles() == {"toy": "spring"}       # still written in, resolved


def test_a_per_drive_role_takes_a_key_per_slot(toy_role):
    c = BuildConfig(**MECH, roles=(("toy@q2", "damper"),))
    assert c.role("toy", "q2") == "damper"
    assert c.role("toy", "q3") == "spring"                  # the role's own, else default
    assert c.role("toy") == "spring"
    assert c.applied_roles() == {"toy": "spring", "toy@q2": "damper"}
    # an optional role empties with "none"
    empty = BuildConfig(**MECH, roles=(("toy", rolemod.NONE),))
    assert empty.role("toy") == ""
    assert empty.role("toy", "q2") == ""
    assert empty.applied_roles() == {"toy": rolemod.NONE}
    assert rolemod.choices("toy") == ["damper", "spring", rolemod.NONE]


def test_a_role_is_refused_where_it_does_not_apply(toy_role, monkeypatch):
    with pytest.raises(ParamError, match="has no toy part role"):
        BuildConfig(roles=(("toy", "damper"),))             # the Strider is a walker
    with pytest.raises(ParamError, match="unknown construction role"):
        BuildConfig(**MECH, roles=(("nope", "x"),))
    with pytest.raises(ParamError, match="config's 'pin' field"):
        BuildConfig(roles=(("pin", "chicago"),))
    with pytest.raises(ParamError, match="the slot must be an identifier"):
        BuildConfig(**MECH, roles=(("toy@2 x", "damper"),))
    per_design = rolemod.Role("solo", "solo part", "tests.test_platform:TOYS", ("mechanism",),
                              default="spring")
    monkeypatch.setitem(rolemod.ROLES, "solo", per_design)
    with pytest.raises(ParamError, match="one per design"):
        BuildConfig(**MECH, roles=(("solo@q2", "damper"),))
    with pytest.raises(ParamError, match="can't be left empty"):
        BuildConfig(**MECH, roles=(("solo", rolemod.NONE),))


def test_choices_keep_the_constructions_that_serve_the_role():
    assert rolemod.choices("pin") == ["ball", "chicago"]   # the pillars' don't serve pins
    assert rolemod.choices("pillar") == ["ball_pillar", "standoff"]


def test_a_later_role_round_trips_through_the_spec(toy_role):
    from spiderpig.api.store_ops import _config_from_resolved
    from spiderpig.spec import Spec, validate
    from spiderpig.stages.resolve import resolve

    doc = {"kind": "mechanism", "linkage": {"key": "dwell_rocker"},
           "constructions": {"toy": "damper", "toy@q2": "none"}}
    assert validate(doc) == []
    spec = Spec.from_dict(doc)
    assert spec.constructions.roles == {"toy": "damper", "toy@q2": "none"}
    assert spec.to_dict()["constructions"] == doc["constructions"]    # flat, as given
    design = resolve(spec, store=None)
    assert design.config.role("toy") == "damper"
    assert design.config.role("toy", "q2") == ""
    assert design.resolved["constructions"]["toy"] == "damper"
    assert _config_from_resolved(design.resolved) == design.config
    errors = validate({**doc, "constructions": {"toy": "no_such_toy"}})
    assert [e.path for e in errors] == ["constructions.toy"]


def test_the_validator_refuses_a_role_of_another_kind(toy_role):
    from spiderpig.spec import validate

    errors = validate({"kind": "walker", "linkage": {"key": "strider"},
                       "constructions": {"toy": "damper"}})
    assert [e.path for e in errors] == ["constructions.toy"]
    assert "no toy part role" in errors[0].message


def test_a_stored_design_naming_an_unknown_role_fails(toy_role):
    from spiderpig.api.store_ops import _config_from_resolved
    from spiderpig.spec import Spec
    from spiderpig.stages.resolve import resolve

    design = resolve(Spec.from_dict({"kind": "mechanism", "linkage": {"key": "dwell_rocker"},
                                     "constructions": {"toy": "damper"}}), store=None)
    gone = {**design.resolved, "constructions": {**design.resolved["constructions"],
                                                 "flux": "capacitor"}}
    with pytest.raises(ParamError, match="no longer has"):
        _config_from_resolved(gone)


def test_a_role_follows_a_switch_of_linkage(toy_role):
    from spiderpig.config import design_from_query

    base = BuildConfig(**MECH, roles=(("toy", "damper"),))
    other = design_from_query({"linkage": "crank_rocker"}, base, robot=False)
    assert other.role("toy") == "damper"                     # a mechanism: kept
    walker = design_from_query({"linkage": "strider"}, base, robot=True)
    assert walker.roles == ()                                # a walker has no toy part


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
    args = p.parse_args(["--linkage", "dwell_rocker", "--toy", "damper",
                         "--role", "toy@q2=spring"])
    c = config_from_args(args, robot=False)
    assert c.role("toy") == "damper"
    assert c.role("toy", "q2") == "spring"


def test_the_cache_keys_follow_a_roles_registry_path():
    """A role names its registry and default by "module:NAME": the incremental keys reach
    them (an edit there re-makes what they shape)."""
    from spiderpig import keys

    syms = keys.closure(keys.PLAN_ROOTS).symbols
    assert ("spiderpig.config", "config_crank") in syms
    assert ("spiderpig.construction", "AXLES") in keys.closure(keys.FAB_ROOTS).symbols


def test_a_config_key_hashes_only_what_differs_from_the_defaults():
    a = BuildConfig(heads="gap")
    assert a.key.startswith("strider_double_robot_")
    assert a.key != BuildConfig(heads="sink").key
    assert BuildConfig().key == "strider_double_robot"


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


def test_an_axle_groups_terms_come_from_what_it_built():
    from spiderpig import construction
    from spiderpig.construction.axle import AxleGroup
    from spiderpig.construction.base import Params, Realized
    from spiderpig.stack import Axis

    class Ctx:
        params = Params()
        config = None

    group = AxleGroup(Axis("C", "pin", ("b1", "b2"), ()), construction.axle("chicago"))
    plain = group.error_terms(Ctx())
    assert plain is not None
    assert plain.radial_play_mm == pytest.approx(0.2)
    note = {"links": [{"clearance_mm": 0.0}, {"clearance_mm": 0.2}], "play_mm": 0.07,
            "worst_deg": 0.41, "play_basis": "the barrel's stock length"}
    got = Realized(notes={"wobble": {group.name: note}})
    built = group.error_terms(Ctx(), got)
    assert built is not None
    assert (built.radial_play_mm, built.axial_play_mm, built.tilt_deg) == (0.2, 0.07, 0.41)
    assert built.notes[-1] == "end float: the barrel's stock length"


def test_the_strength_alternatives_are_the_registered_sections():
    from spiderpig.strength import _alt_sections

    pin = _alt_sections("pin", {"section": {"name": "something else"}})
    assert [(k, s.name) for k, s in pin] == [("--pin chicago", "chicago barrel 4 x 3 tube")]
    pillar = _alt_sections("pillar", {"section": {"name": "something else"}})
    assert [k for k, _ in pillar] == ["--pillar standoff"]
    assert _alt_sections("pin", {"section": {"name": "chicago barrel 4 x 3 tube"}}) == []

