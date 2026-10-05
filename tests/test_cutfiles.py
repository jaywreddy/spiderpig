"""Cut files and the BOM's fitting lines: exact DXF contours read back against the solid,
the kerf per sheet and service, the DXF sets per service, the filament per printed part,
the horn screws' shim stacks and threadlocker, the splice studs' threadlocker, and the cut
rules measured to pockets and windows. Synthetic parts: no plan, quick."""

from __future__ import annotations

import csv
import math

import ezdxf
import pytest
from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    Ellipse,
    Locations,
    Mode,
    Rectangle,
    SlotCenterToCenter,
    extrude,
)

from spiderpig import manufacture
from spiderpig.hardware.bom import (
    PRESS_FILAMENT,
    TPU_FILAMENT,
    BomLine,
    bom_from_mechanism,
    fitting_lines,
    part_filament,
    shim_breakdown,
)
from spiderpig.layout import (
    CHORD_TOL,
    DEFAULT_KERF,
    fidelity,
    save_sheets,
    section_of,
    sheet_kerf,
    sheet_service,
    wire_vertices,
)
from spiderpig.mechanism import Body, Mechanism
from spiderpig.shapes import disc


def _plate(t: float = 3.0, curve: bool = False, hole_at: float = -20.0):
    """A 40 x 12 slot with a round hole, a dog-boned rectangular pocket (lines and arcs)
    and, with ``curve``, an elliptic window (a B-spline in the section)."""
    with BuildPart() as p:
        with BuildSketch():
            SlotCenterToCenter(40, 12)
            with Locations((hole_at, 0)):
                Circle(2, mode=Mode.SUBTRACT)
            Rectangle(6, 3, mode=Mode.SUBTRACT)
            with Locations((3, 1.5), (-3, 1.5), (3, -1.5), (-3, -1.5)):
                Circle(0.5, mode=Mode.SUBTRACT)
            if curve:
                with Locations((11, 0)):
                    Ellipse(2.5, 1.2, mode=Mode.SUBTRACT)
        extrude(amount=t)
    return p.part


def _body(name="plate", sheet=None, **kw) -> Body:
    return Body(name, part=_plate(**kw), fab="laser", sheet=sheet)


# ---------------------------------------------------------------------------
# DXF fidelity
# ---------------------------------------------------------------------------


def test_lines_and_arcs_are_emitted_exactly():
    """Every edge's start is a vertex and every arc a bulge: the contour read back is the
    section's to the sampling's resolution, its area to 1e-4."""
    sk = section_of(_body())
    for w in sk.wires():
        if len(w.edges()) == 1:
            continue                                     # the round hole: a CIRCLE
        verts = wire_vertices(w)
        # one vertex per edge, none sampled; an arc past a half turn (a dog-bone's 270
        # degrees) is two bulges (a bulge is finite up to a half circle)
        big = sum(1 for e in w.edges() if e.geom_type.name == "CIRCLE"
                  and e.length > math.pi * e.radius + 1e-6)
        assert len(verts) == len(w.edges()) + big
        starts = {(round(e.start_point().X, 6), round(e.start_point().Y, 6))
                  for e in w.edges()} | {(round(e.end_point().X, 6), round(e.end_point().Y, 6))
                                         for e in w.edges()}
        assert starts <= {(round(x, 6), round(y, 6)) for x, y, _ in verts}   # every end kept
        arcs = [e for e in w.edges() if e.geom_type.name == "CIRCLE"]
        assert sum(1 for *_, b in verts if b) == len(arcs) + big
    f = fidelity(sk, tol=1e-4)
    assert f["deviation_mm"] < 1e-3
    assert f["area_rel"] < 1e-4
    assert f["entities"] == {"LWPOLYLINE": 2, "CIRCLE": 1}


def test_other_curves_are_flattened_within_the_chord_tolerance():
    sk = section_of(_body(curve=True))
    f = fidelity(sk)
    assert 0.0 < f["deviation_mm"] <= CHORD_TOL + 0.005
    assert f["area_rel"] < manufacture.DXF_AREA_REL
    assert manufacture.dxf_issue(Body("p"), f) is None


def test_the_cut_rules_report_a_dxf_off_the_solid():
    good = {"deviation_mm": 0.001, "area_rel": 1e-5, "area_mm2": 10.0, "dxf_area_mm2": 10.0}
    assert manufacture.dxf_issue(Body("p"), good) is None
    bad = dict(good, deviation_mm=0.2)
    issue = manufacture.dxf_issue(Body("p"), bad)
    assert issue["rule"] == "dxf"
    assert issue["level"] == "error"
    m = manufacture.check(Mechanism(name="m", bodies=[_body(sheet="acrylic_3mm")]),
                          "acrylic_3mm")
    assert m["dxf"]["plate"]["deviation_mm"] < manufacture.DXF_DEVIATION
    assert m["dxf_worst_mm"] == m["dxf"]["plate"]["deviation_mm"]
    assert "dxf" not in m["by_rule"]
    assert manufacture.summary(m)["kerf"] == {"acrylic_3mm": 0.2}


# ---------------------------------------------------------------------------
# Kerf per sheet and service
# ---------------------------------------------------------------------------


def test_kerf_per_sheet_and_service():
    assert sheet_kerf("al5052_2mm") == 0.0               # SendCutSend compensates itself
    assert sheet_kerf("al6061_2p5mm") == 0.0
    assert sheet_kerf("acrylic_3mm") == pytest.approx(0.2)   # Ponoko: the laser follows
    assert sheet_kerf("acrylic_1p5mm") == pytest.approx(0.2)
    assert sheet_kerf("plywood_3mm") == DEFAULT_KERF     # no service named
    assert sheet_service("al5052_2mm") == "SendCutSend"
    assert sheet_service("acrylic_3mm") == "Ponoko"


def _radii(path) -> list[float]:
    return sorted(e.dxf.radius for e in ezdxf.readfile(str(path)).modelspace()
                  if e.dxftype() == "CIRCLE")


def test_dxf_sets_split_per_service_each_with_its_kerf(tmp_path):
    mech = Mechanism(name="m", bodies=[_body("a", sheet="acrylic_3mm"),
                                       _body("b", sheet="al5052_2mm", t=2.032)])
    files = save_sheets(mech, tmp_path / "s", default="acrylic_3mm")
    names = sorted(f.name for f in files)
    assert names == ["s_Ponoko_acrylic_3mm_0.dxf", "s_SendCutSend_al5052_2mm_0.dxf"]
    ponoko, scs = sorted(files)
    assert _radii(scs) == pytest.approx([2.0])            # drawn at size
    assert _radii(ponoko) == pytest.approx([1.9])         # the hole shrunk by half the kerf
    with open(tmp_path / "s_parts.csv") as f:
        rows = {r["part"]: r for r in csv.DictReader(f)}
    assert (rows["a"]["service"], float(rows["a"]["kerf_mm"])) == ("Ponoko", 0.2)
    assert (rows["b"]["service"], float(rows["b"]["kerf_mm"])) == ("SendCutSend", 0.0)
    # bulged polylines: the slot's ends are two arcs, not 96-gons
    doc = ezdxf.readfile(str(scs))
    def span(e):
        xs = [p[0] for p in e.get_points("xy")]
        return max(xs) - min(xs)

    outline = max((e for e in doc.modelspace() if e.dxftype() == "LWPOLYLINE"), key=span)
    assert len(outline) == 4
    assert sum(1 for p in outline.get_points("b") if p[0]) == 2
    # a kerf given overrides every sheet's
    files = save_sheets(mech, tmp_path / "o", default="acrylic_3mm", kerf=0.1)
    assert all(_radii(f) == pytest.approx([1.95]) for f in files)


# ---------------------------------------------------------------------------
# The cut rules to pockets and windows
# ---------------------------------------------------------------------------


def test_the_edge_rules_measure_to_pockets_and_windows():
    """A hole beside the dog-boned pocket (a window through the plate) is measured to it:
    in 2 mm aluminium 0.8 mm of web is under 1 x the thickness, an error; the hole-to-edge
    distance is exact (to the arc, not to points round it)."""
    b = _body(sheet="al5052_2mm", t=2.032, hole_at=-5.8)   # by the pocket's dog-bones
    issues = {i["rule"]: i for i in manufacture.part_issues(b, "al5052_2mm")}
    web = issues["web"]
    assert web["level"] == "error"
    assert "hole" in web["detail"]
    # to the dog-bone circle at (-3, 1.5), r 0.5, from the 4 mm hole at (-5.8, 0)
    assert web["value"] == pytest.approx(math.hypot(2.8, 1.5) - 0.5 - 2.0, abs=0.01)
    # hole-to-edge exact: centred on the slot's left arc, 6 - 2 = 4 mm (2 t is 4.06: a warning)
    b = _body(sheet="al5052_2mm", t=2.032, hole_at=-20.0)
    edge = {i["rule"]: i for i in manufacture.part_issues(b, "al5052_2mm")}["edge"]
    assert edge["value"] == pytest.approx(4.0, abs=1e-3)
    assert edge["level"] == "warning"


# ---------------------------------------------------------------------------
# The BOM: filament per part, shims, threadlocker
# ---------------------------------------------------------------------------


def _printed(name: str) -> Body:
    return Body(name, part=disc((0, 0), 4.0, 0.0, 3.0), fab="printed")


def _fitted_mech(servo: str = "sts3215") -> Mechanism:
    shim = disc((0, 0), 2.95, 0.0, 0.7) - disc((0, 0), 1.55, -1.0, 1.7)
    meta = {"filament": "pla_filament", "servo": servo,
            "crank_bolt": {"chains": [{"at": "J1_leg1", "sleeve_press_mm": 0.1},
                                      {"at": "J1_leg0", "sleeve_press_mm": 0.0}]}}
    return Mechanism(name="m", meta=meta, bodies=[
        _printed("L.b3_leg0_sock"), _printed("R.b3_leg0_sock"),
        _printed("L.crank_pin_sleeve_J1_leg1"), _printed("L.crank_pin_sleeve_J1_leg0"),
        _printed("L.servo_horn_spacer"),
        Body("L.crank_horn_shims0", part=shim, fab="purchased", bom_key="shim_din988_3x6"),
        *[Body(f"L.crank_horn_screw{i}", part=disc((0, 0), 1.5, 0, 8), fab="purchased",
               bom_key="m3_bhcs_8") for i in range(4)],
        Body("L.pillar_J2_leg0_stud3", part=disc((0, 0), 2, 0, 8), fab="purchased",
             bom_key="m3_bhcs_8"),
        Body("tie_stud0", part=disc((0, 0), 2, 0, 8), fab="purchased", bom_key="m3_bhcs_8"),
    ])


def test_each_printed_part_names_its_filament():
    mech = _fitted_mech()
    fil = {b.name: part_filament(b, mech.meta, "pla_filament") for b in mech.bodies
           if b.fab == "printed"}
    assert fil["L.b3_leg0_sock"] == TPU_FILAMENT
    assert fil["L.crank_pin_sleeve_J1_leg1"] == PRESS_FILAMENT       # pressed on its hex
    assert fil["L.crank_pin_sleeve_J1_leg0"] == "pla_filament"       # a slide fit
    assert fil["L.servo_horn_spacer"] == "pla_filament"
    bom = bom_from_mechanism(mech, group=False)
    rows = {r.key: r for r in bom.purchased}
    assert {TPU_FILAMENT, PRESS_FILAMENT, "pla_filament"} <= set(rows)
    vol = math.pi * 16 * 3 / 1000                               # one printed disc, cm3
    assert rows[TPU_FILAMENT].qty == pytest.approx(round(2 * vol * 1.22 / 750, 3))
    assert rows[TPU_FILAMENT].url.startswith("https://")         # a source to buy it from
    made = {m.name: m.material for m in bom.made}
    assert made["L.b3_leg0_sock"].startswith("TPU 95A")
    assert made["L.crank_pin_sleeve_J1_leg1"].startswith("PETG")
    assert bom.printed_g == pytest.approx(sum(bom.filaments.values()))
    assert "TPU 95A" in bom.markdown()


def test_a_shape_printed_in_two_filaments_is_two_rows():
    mech = _fitted_mech()
    bom = bom_from_mechanism(mech)            # every printed disc is the same shape
    printed = [m for m in bom.made if m.method == "printed"]
    assert sorted(m.qty for m in printed) == [1, 2, 2]
    assert len({m.material for m in printed}) == 3


def test_horn_screws_get_their_shim_stack_and_threadlocker():
    assert shim_breakdown(0.7, (0.1, 0.2, 0.3, 0.5, 1.0)) == [0.5, 0.2]
    assert shim_breakdown(1.2, (0.1, 0.2, 0.3, 0.5, 1.0)) == [1.0, 0.2]
    lines, notes, replaced = fitting_lines(_fitted_mech())
    shims = [x for x in lines if x.key == "shim_din988_3x6"]
    assert [x.where for x in shims] == [
        "L.crank_horn_shims0: horn screw 0, DIN 988 shims 0.5 + 0.2 mm (0.7 mm) under its "
        "head"]
    assert replaced == {"L.crank_horn_shims0"}
    assert sum(x.qty for x in lines if x.key == "threadlocker_222") == pytest.approx(0.04)
    assert any("0.5 + 0.2 mm" in n for n in notes)
    bom = bom_from_mechanism(_fitted_mech(), group=False)
    # the BOM orders the stack per thickness (split_shims): the 0.5 mm ring bought as a DIN 433
    # washer (bom.SHIM_AS), the 0.2 mm one a DIN 988 shim
    rows = {r.key: r for r in bom.purchased
            if r.key.startswith("shim_din988_3x6") or r.key == "m3_washer_433"}
    assert set(rows) == {"m3_washer_433", "shim_din988_3x6_t0p2"}
    assert all(r.qty == 1 and "0.5 + 0.2" in r.where[0] for r in rows.values())
    # a plastic horn (the XL330's, self-tapping): no threadlocker into it
    lines, _, _ = fitting_lines(_fitted_mech("xl330_m288"))
    assert not any(x.key == "threadlocker_222" for x in lines)
    lines, _, _ = fitting_lines(_fitted_mech("xl430_w250"))     # the HN11-N101: tapped metal
    assert any(x.key == "threadlocker_222" for x in lines)


def test_splice_studs_get_medium_threadlocker():
    lines, notes, _ = fitting_lines(_fitted_mech())
    studs = [x for x in lines if x.key == "threadlocker_243"]
    assert [x.where.split(":")[0] for x in studs] == ["L.pillar_J2_leg0_stud3"]  # not the ties'
    assert "243 or 263" in studs[0].where
    assert "off the acrylic" in studs[0].where
    assert any("splice stud" in n for n in notes)


def test_splice_stud_threadlocker_is_not_counted_twice():
    """The pillar construction lists its splice studs' 243 itself (``splice_lock_key``, in
    ``bom_extras``); the fitting lines then add only the note (merge of r4, 2026-10-05)."""
    mech = _fitted_mech()
    mech.bom_extras.append(BomLine("threadlocker_243", 0.01,
                                   "pillar_J2_leg0 splice studs (metal to metal only)"))
    lines, notes, _ = fitting_lines(mech)
    assert not any(x.key == "threadlocker_243" for x in lines)
    assert any("splice stud" in n for n in notes)
