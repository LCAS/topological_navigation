"""Tests for ``topo_stats.py`` – the topological navigation statistics CLI."""

import sys
import io
import tempfile
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

from topological_navigation.nav_stats_db import NavStatsDB, compute_map_hash

# ---------------------------------------------------------------------------
# Minimal map YAML fixtures
# ---------------------------------------------------------------------------

_MAP_YAML = """\
meta:
  origin:
    latitude: 51.2096316575512
    longitude: 0.4946125429852941
name: Driscoll Field
pointset: driscoll_field
nodes:
  - meta:
      node: WP1
      map: driscoll_field
      pointset: driscoll_field
    node:
      name: WP1
      edges:
        - edge_id: WP1_WP2
          node: WP2
          action: navigate_to_pose
          properties:
            max_speed: 0.5
            weight: 1.0
      pose:
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - meta:
      node: WP2
      map: driscoll_field
      pointset: driscoll_field
    node:
      name: WP2
      edges: []
      pose:
        position: {x: 10.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"""

_MAP_HASH = compute_map_hash(_MAP_YAML)


@pytest.fixture
def db_path(tmp_path):
    return str(tmp_path / "stats.db")


@pytest.fixture
def populated_db(db_path):
    """Database with one stored map and several traversal records."""
    db = NavStatsDB(db_path)
    db.store_map(_MAP_YAML)
    for status in ["success", "success", "failed", "aborted"]:
        db.record_traversal(
            map_name="Driscoll Field",
            map_hash=_MAP_HASH,
            edge_id="WP1_WP2",
            origin="WP1",
            target="WP2",
            status=status,
            duration_s=10.0,
            edge_length=10.0,
        )
    db.close()
    return db_path


# ---------------------------------------------------------------------------
# Helper: run the CLI and capture stdout / exit code
# ---------------------------------------------------------------------------

def _run(args, capsys=None):
    """Run topo_stats main() with *args* (list of str).

    Returns (stdout_text, exit_code).
    """
    from topological_navigation.scripts.topo_stats import main

    with patch("sys.argv", ["topo_stats"] + args):
        try:
            main()
        except SystemExit as e:
            exit_code = e.code or 0
        else:
            exit_code = 0

    captured = capsys.readouterr() if capsys else None
    return (captured.out if captured else ""), exit_code


# ---------------------------------------------------------------------------
# map list
# ---------------------------------------------------------------------------

def test_map_list_empty(db_path, capsys):
    out, rc = _run([db_path, "map", "list"], capsys)
    assert rc == 0
    assert "No maps" in out


def test_map_list_shows_map(populated_db, capsys):
    out, rc = _run([populated_db, "map", "list"], capsys)
    assert rc == 0
    assert "Driscoll Field" in out
    assert _MAP_HASH in out


# ---------------------------------------------------------------------------
# map show
# ---------------------------------------------------------------------------

def test_map_show(populated_db, capsys):
    out, rc = _run([populated_db, "map", "show", "Driscoll Field"], capsys)
    assert rc == 0
    data = yaml.safe_load(out)
    assert data["map_name"] == "Driscoll Field"
    assert data["map_hash"] == _MAP_HASH
    assert abs(data["latitude"] - 51.2096316575512) < 1e-6


def test_map_show_by_hash(populated_db, capsys):
    out, rc = _run([populated_db, "map", "show", _MAP_HASH], capsys)
    assert rc == 0
    data = yaml.safe_load(out)
    assert data["map_hash"] == _MAP_HASH


def test_map_show_not_found(db_path, capsys):
    _out, rc = _run([db_path, "map", "show", "nonexistent"], capsys)
    assert rc == 1


# ---------------------------------------------------------------------------
# map export / import
# ---------------------------------------------------------------------------

def test_map_export(populated_db, capsys):
    out, rc = _run([populated_db, "map", "export", "Driscoll Field"], capsys)
    assert rc == 0
    assert "Driscoll Field" in out
    parsed = yaml.safe_load(out)
    assert parsed["name"] == "Driscoll Field"


def test_map_import_and_list(db_path, tmp_path, capsys):
    # Write map to a temp YAML file
    p = tmp_path / "mymap.yaml"
    p.write_text(_MAP_YAML, encoding="utf-8")
    _out, rc = _run([db_path, "map", "import", str(p)], capsys)
    assert rc == 0
    out, rc2 = _run([db_path, "map", "list"], capsys)
    assert rc2 == 0
    assert "Driscoll Field" in out


def test_map_import_missing_file(db_path, capsys):
    _out, rc = _run([db_path, "map", "import", "/nonexistent/path.yaml"], capsys)
    assert rc == 1


# ---------------------------------------------------------------------------
# map rm
# ---------------------------------------------------------------------------

def test_map_rm(populated_db, capsys):
    _out, rc = _run([populated_db, "map", "rm", "Driscoll Field"], capsys)
    assert rc == 0
    out2, rc2 = _run([populated_db, "map", "list"], capsys)
    assert "Driscoll Field" not in out2


def test_map_rm_not_found(db_path, capsys):
    _out, rc = _run([db_path, "map", "rm", "nonexistent"], capsys)
    assert rc == 1


# ---------------------------------------------------------------------------
# map stats
# ---------------------------------------------------------------------------

def test_map_stats(populated_db, capsys):
    out, rc = _run([populated_db, "map", "stats", "Driscoll Field"], capsys)
    assert rc == 0
    assert "num_nodes" in out
    assert "num_edges" in out
    assert "total_length_m" in out
    assert "bbox_area_sqm" in out
    # The simple map has 2 nodes, 1 edge, length=10.0
    assert "2" in out   # num_nodes
    assert "1" in out   # num_edges
    assert "10.0" in out


def test_map_stats_not_found(db_path, capsys):
    _out, rc = _run([db_path, "map", "stats", "nonexistent"], capsys)
    assert rc == 1


# ---------------------------------------------------------------------------
# traversals summary
# ---------------------------------------------------------------------------

def test_traversals_summary(populated_db, capsys):
    out, rc = _run([populated_db, "traversals", "summary"], capsys)
    assert rc == 0
    assert "Driscoll Field" in out
    assert "4" in out  # total traversals


def test_traversals_summary_empty(db_path, capsys):
    out, rc = _run([db_path, "traversals", "summary"], capsys)
    assert rc == 0
    assert "No traversal" in out


def test_traversals_summary_with_filter(populated_db, capsys):
    out, rc = _run(
        [populated_db, "traversals", "summary", "--filter", "status = 'success'"],
        capsys,
    )
    assert rc == 0
    assert "2" in out   # 2 successes


# ---------------------------------------------------------------------------
# traversals edge_stats
# ---------------------------------------------------------------------------

def test_edge_stats(populated_db, capsys):
    out, rc = _run(
        [populated_db, "traversals", "edge_stats", "Driscoll Field"],
        capsys,
    )
    assert rc == 0
    data = yaml.safe_load(out)
    assert data["map_name"] == "Driscoll Field"
    assert data["map_hash"] == _MAP_HASH
    es = data["edge_statistics"]["WP1_WP2"]
    assert es["total_traversals"] == 4
    assert es["success"] == 2
    assert es["failed"] == 1
    assert es["aborted"] == 1


def test_edge_stats_with_filter(populated_db, capsys):
    out, rc = _run(
        [populated_db, "traversals", "edge_stats", "Driscoll Field",
         "--filter", "status = 'success'"],
        capsys,
    )
    assert rc == 0
    data = yaml.safe_load(out)
    es = data["edge_statistics"]["WP1_WP2"]
    assert es["total_traversals"] == 2
    assert es["success"] == 2


def test_edge_stats_map_not_found(db_path, capsys):
    _out, rc = _run(
        [db_path, "traversals", "edge_stats", "nonexistent"],
        capsys,
    )
    assert rc == 1


# ---------------------------------------------------------------------------
# traversals map_stats
# ---------------------------------------------------------------------------

def test_map_stats_traversals(populated_db, capsys):
    out, rc = _run(
        [populated_db, "traversals", "map_stats", "Driscoll Field"],
        capsys,
    )
    assert rc == 0
    data = yaml.safe_load(out)
    assert data["map_name"] == "Driscoll Field"
    assert data["map_hash"] == _MAP_HASH
    assert data["total_traversals"] == 4
    assert data["outcomes"]["success"] == 2
    assert data["outcomes"]["failed"] == 1
    assert data["outcomes"]["aborted"] == 1
    assert data["outcomes"]["success_pct"] == 50.0


def test_map_stats_traversals_topn(populated_db, capsys):
    out, rc = _run(
        [populated_db, "traversals", "map_stats", "Driscoll Field",
         "--topn_failures=1", "--topn_success=1", "--topn_aborted=1"],
        capsys,
    )
    assert rc == 0
    data = yaml.safe_load(out)
    assert "top_edges_by_failures" in data
    assert "top_edges_by_success" in data
    assert "top_edges_by_aborted" in data
    assert data["top_edges_by_failures"][0]["edge_id"] == "WP1_WP2"


def test_map_stats_traversals_map_not_found(db_path, capsys):
    _out, rc = _run(
        [db_path, "traversals", "map_stats", "nonexistent"],
        capsys,
    )
    assert rc == 1


# ---------------------------------------------------------------------------
# map merge
# ---------------------------------------------------------------------------

def test_map_merge(populated_db, db_path, capsys):
    _out, rc = _run([db_path, "map", "merge", populated_db], capsys)
    assert rc == 0
    out, rc2 = _run([db_path, "map", "list"], capsys)
    assert rc2 == 0
    assert "Driscoll Field" in out
    out2, rc3 = _run([db_path, "traversals", "summary"], capsys)
    assert rc3 == 0
    assert "4" in out2


def test_map_merge_missing_source(db_path, capsys):
    _out, rc = _run([db_path, "map", "merge", "/nonexistent/other.db"], capsys)
    assert rc == 1


# ---------------------------------------------------------------------------
# coverage summary
# ---------------------------------------------------------------------------

def test_coverage_summary_single_map(populated_db, capsys):
    out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "Driscoll Field"],
        capsys,
    )
    assert rc == 0
    assert "Coverage Summary" in out
    assert "Selected edges | 1" in out
    assert "Covered edges | 1" in out


def test_coverage_summary_all_maps(populated_db, capsys):
    out, rc = _run(
        [populated_db, "coverage", "summary", "-a"],
        capsys,
    )
    assert rc == 0
    assert "Coverage Summary" in out


def test_coverage_summary_requires_selection(populated_db, capsys):
    _out, rc = _run([populated_db, "coverage", "summary"], capsys)
    assert rc != 0


def test_coverage_summary_map_not_found(populated_db, capsys):
    _out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "nonexistent"],
        capsys,
    )
    assert rc == 1


def test_coverage_summary_name_filter(populated_db, capsys):
    out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "Driscoll Field",
         "--filter", "name:WP1"],
        capsys,
    )
    assert rc == 0
    assert "Selected edges | 1" in out


def test_coverage_summary_name_filter_no_match(populated_db, capsys):
    out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "Driscoll Field",
         "--filter", "name:NoSuchNode*"],
        capsys,
    )
    assert rc == 0
    assert "Selected edges | 0" in out


def test_coverage_summary_default_min_success_is_two(populated_db, capsys):
    # WP1_WP2 has exactly 2 recorded successes; default threshold covers it.
    out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "Driscoll Field"],
        capsys,
    )
    assert rc == 0
    assert "Covered edges | 1" in out


def test_coverage_summary_min_success_raises_threshold(populated_db, capsys):
    # Requiring 3 successes makes the (2-success) edge uncovered.
    out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "Driscoll Field",
         "--min-success", "3"],
        capsys,
    )
    assert rc == 0
    assert "Covered edges | 0" in out


def test_coverage_summary_min_success_lowers_threshold(populated_db, capsys):
    out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "Driscoll Field",
         "--min-success", "1"],
        capsys,
    )
    assert rc == 0
    assert "Covered edges | 1" in out


def test_coverage_summary_min_success_invalid_value(populated_db, capsys):
    _out, rc = _run(
        [populated_db, "coverage", "summary", "-m", "Driscoll Field",
         "--min-success", "0"],
        capsys,
    )
    assert rc == 1


# ---------------------------------------------------------------------------
# coverage report
# ---------------------------------------------------------------------------

def test_coverage_report_stdout(populated_db, capsys):
    out, rc = _run(
        [populated_db, "coverage", "report", "-m", "Driscoll Field"],
        capsys,
    )
    assert rc == 0
    assert "Route Coverage Sign-off Report" in out
    assert "WP1_WP2" in out
    assert "Per-Edge Detail" in out


def test_coverage_report_to_file(populated_db, tmp_path, capsys):
    out_file = tmp_path / "report.md"
    _out, rc = _run(
        [populated_db, "coverage", "report", "-m", "Driscoll Field",
         "-o", str(out_file)],
        capsys,
    )
    assert rc == 0
    content = out_file.read_text(encoding="utf-8")
    assert "Route Coverage Sign-off Report" in content
    assert "WP1_WP2" in content


# ---------------------------------------------------------------------------
# coverage svg
# ---------------------------------------------------------------------------

def test_coverage_svg(populated_db, tmp_path, capsys):
    out_file = tmp_path / "coverage.svg"
    _out, rc = _run(
        [populated_db, "coverage", "svg", "-m", "Driscoll Field",
         "-o", str(out_file)],
        capsys,
    )
    assert rc == 0
    content = out_file.read_text(encoding="utf-8")
    assert "<svg" in content
    assert "WP1_WP2" in content
