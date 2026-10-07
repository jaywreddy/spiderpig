"""The scorecard's ``--compare`` (``tests/scorecard.py``): exit codes up front, and a warning
when two scorecards' timings aren't comparable."""

from __future__ import annotations

from tests import scorecard


def _card(rc=0, workers=4, load=3.0, wall=100.0):
    return {"meta": {"host": "box"},
            "loadavg": {"start": [load, load, load], "quick": {"before": [load] * 3,
                                                              "after": [load] * 3}},
            "quick": {"rc": rc, "workers": workers, "wall_s": wall, "tests": 10},
            "build": {"cold": {"rc": [0, 0, 0], "wall_s": 40.0}}}


def test_an_exit_code_change_is_shown_first():
    text = scorecard.compare_report(_card(), _card(rc=1))
    assert text.splitlines()[0] == "EXIT CODES (changed, or non-zero):"
    assert "!! quick.rc: 0 -> 1" in text
    assert scorecard.rc_changes(_card(), _card(rc=1)) == [("quick.rc", 0.0, 1.0)]


def test_a_non_zero_exit_code_is_shown_even_unchanged():
    assert scorecard.rc_changes(_card(rc=2), _card(rc=2)) == [("quick.rc", 2.0, 2.0)]
    assert scorecard.rc_changes(_card(), _card()) == []


def test_other_workers_or_load_warn():
    assert scorecard.comparability(_card(), _card()) == []
    warn = scorecard.comparability(_card(), _card(workers=12))
    assert any("quick.workers: 4 vs 12" in w for w in warn)
    assert any("load" in w for w in scorecard.comparability(_card(load=2.0),
                                                           _card(load=5.0)))
    assert not scorecard.comparability(_card(load=2.0), _card(load=3.9))
    assert scorecard.compare_report(_card(), _card(workers=12)).startswith("WARNING:")


def test_the_deltas():
    rows = scorecard.compare(_card(), _card(wall=50.0))
    assert ("quick.wall_s", 100.0, 50.0) in rows
    assert all(not k.startswith("loadavg.") for k, *_ in rows)
