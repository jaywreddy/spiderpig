"""Which module's tier each test file belongs to (``mise run test-<module>``).

Every test carries exactly one module marker (``conftest.pytest_collection_modifyitems``
checks it): its own ``@pytest.mark.<module>`` when it has one, else its file's default
from :data:`MODULE_OF_FILE`. A new test file needs a line here (the run stops at
collection otherwise). A package edits only the lines of the files it owns: one plain
entry per file, so merges stay trivial.
"""

from __future__ import annotations

MODULES = ("linkage", "planner", "construction", "hardware", "strength", "api", "sim",
           "server")
"""The module markers, one per area of the engine (``docs/agentlib/TESTING.md``)."""

MODULE_OF_FILE: dict[str, str] = {
    # linkage / kinematics / walk (P1)
    "test_linkage.py": "linkage",
    "test_linkages_diywalkers.py": "linkage",
    "test_linkages_klann.py": "linkage",
    "test_multi_linkage.py": "linkage",
    "test_transforms.py": "linkage",
    "test_mechanisms.py": "linkage",
    "test_walk.py": "linkage",
    # server / view (P1)
    "test_view.py": "server",
    "e2e/test_drive.py": "server",
    "e2e/test_viewer.py": "server",
    # the planner (P2)
    "test_stack.py": "planner",
    "test_route.py": "planner",
    "test_seam_stack.py": "planner",
    "test_planner_bounds.py": "planner",
    "test_stage_checks.py": "planner",
    "test_recommend.py": "planner",
    "test_clearance.py": "planner",
    # constructions and fabrication (P3)
    "test_axle.py": "construction",
    "test_pivots.py": "construction",
    "test_crank.py": "construction",
    "test_bolt_crank.py": "construction",
    "test_standoff.py": "construction",
    "test_servos.py": "construction",
    "test_robot.py": "construction",
    "test_deck.py": "construction",
    "test_joinery.py": "construction",
    "test_klann_lego_cranks.py": "construction",
    "test_fabricate.py": "construction",
    "test_contract.py": "construction",
    "test_seam_crank.py": "construction",
    "test_seam_pivots.py": "construction",
    "test_seam_chassis.py": "construction",
    # hardware / BOM / order / layout / manufacture (P4)
    "test_bom.py": "hardware",
    "test_order.py": "hardware",
    "test_cutfiles.py": "hardware",
    "test_seam_hardware.py": "hardware",
    # strength / wobble / loads (P4)
    "test_strength.py": "strength",
    "test_wobble.py": "strength",
    # API / store / verify / MCP / CLI build (P5)
    "test_spiderpig_api.py": "api",
    "test_spiderpig_store.py": "api",
    "test_spiderpig_mcp.py": "api",
    "test_review_fixes.py": "api",
    "test_export.py": "api",
    "test_removed_constructions.py": "api",
    # the measuring stick (W0): the build profiler, the doc check
    "test_build_profile.py": "api",
    "test_doc_check.py": "api",
    "test_scorecard.py": "api",
    # sim and bake (P6)
    "test_sim.py": "sim",
    "test_sim_live.py": "sim",
    "test_bake_gltf.py": "sim",
}
"""Test file (relative to ``tests/``) -> its default module."""
