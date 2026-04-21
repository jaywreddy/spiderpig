"""End-to-end smoke test for STEP / STL / DXF emission."""

from __future__ import annotations

import ezdxf
import pytest

from klann import KlannLinkage


@pytest.fixture(scope="module")
def solved_mech():
    return KlannLinkage().solved()


def test_step_export_nonempty(tmp_path, solved_mech):
    path = tmp_path / "klann.step"
    solved_mech.export_step(path)
    assert path.exists()
    assert path.stat().st_size > 1024, "STEP file suspiciously small"


def test_stl_export_nonempty(tmp_path, solved_mech):
    path = tmp_path / "klann.stl"
    solved_mech.export_stl(path)
    assert path.exists()
    assert path.stat().st_size > 1024, "STL file suspiciously small"


def test_dxf_sheets_are_mm_and_have_entities(tmp_path, solved_mech):
    prefix = tmp_path / "klann_sheet"
    solved_mech.save_layouts(prefix)

    sheets = sorted(tmp_path.glob("klann_sheet_*.dxf"))
    assert sheets, "no DXF sheets written"

    total_polylines = 0
    total_circles = 0
    for s in sheets:
        doc = ezdxf.readfile(str(s))
        assert doc.units == ezdxf.units.MM
        for e in doc.modelspace():
            if e.dxftype() == "LWPOLYLINE":
                total_polylines += 1
            elif e.dxftype() == "CIRCLE":
                total_circles += 1

    # 5 laser-cut bodies => >= 5 outer polylines
    assert total_polylines >= 5
    # each link has at least one pin hole
    assert total_circles >= 5
