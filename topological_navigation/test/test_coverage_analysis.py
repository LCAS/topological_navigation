"""Tests for ``coverage_analysis.py`` -- route sign-off coverage computation."""

import pytest

from topological_navigation.coverage_analysis import (
    DEFAULT_MIN_SUCCESS_TRAVERSALS,
    build_merged_graph_for_maps,
    compute_coverage,
    generate_coverage_svg,
    merge_graphs,
    render_markdown_report,
    render_markdown_summary,
    resolve_map_hashes,
)
from topological_navigation.nav_stats_db import NavStatsDB, compute_map_hash
from topological_navigation.networkx_utils import build_graph_from_tmap

_MAP_YAML = """\
meta:
  origin:
    latitude: 51.0
    longitude: 0.0
name: TestMap
pointset: testmap
nodes:
  - meta:
      node: WP1
      map: testmap
      pointset: testmap
      tag: ["zone_a"]
    node:
      name: WP1
      edges:
        - edge_id: WP1_WP2
          node: WP2
          action: navigate_to_pose
      pose:
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - meta:
      node: WP2
      map: testmap
      pointset: testmap
      tag: ["zone_b"]
    node:
      name: WP2
      edges:
        - edge_id: WP2_WP3
          node: WP3
          action: navigate_to_pose
      pose:
        position: {x: 10.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - meta:
      node: WP3
      map: testmap
      pointset: testmap
      tag: ["zone_b"]
    node:
      name: WP3
      edges: []
      pose:
        position: {x: 20.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"""

_MAP_HASH = compute_map_hash(_MAP_YAML)


def _make_db(tmp_path, add_traversals=True):
    db = NavStatsDB(str(tmp_path / "stats.db"))
    db.store_map(_MAP_YAML)
    if add_traversals:
        db.record_traversal(
            map_name="TestMap", map_hash=_MAP_HASH, edge_id="WP1_WP2",
            origin="WP1", target="WP2", status="success", duration_s=5.0,
        )
        db.record_traversal(
            map_name="TestMap", map_hash=_MAP_HASH, edge_id="WP1_WP2",
            origin="WP1", target="WP2", status="success", duration_s=5.0,
        )
    return db


# ---------------------------------------------------------------------------
# merge_graphs
# ---------------------------------------------------------------------------

def test_merge_graphs_dedups_nodes_and_edges():
    import yaml
    tmap = yaml.safe_load(_MAP_YAML)
    g1 = build_graph_from_tmap(tmap)
    g2 = build_graph_from_tmap(tmap)
    merged = merge_graphs([g1, g2])
    assert merged.number_of_nodes() == 3
    assert merged.number_of_edges() == 2


# ---------------------------------------------------------------------------
# compute_coverage
# ---------------------------------------------------------------------------

def test_compute_coverage_basic(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes)
    s = report.summary()
    assert s["total_edges"] == 2
    assert s["covered_edges"] == 1
    assert s["uncovered_edges"] == 1
    assert s["coverage_pct"] == 50.0
    db.close()


def test_compute_coverage_with_name_filter(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes, filters=["name:WP1"])
    s = report.summary()
    # Only WP1_WP2 is incident to WP1
    assert s["total_edges"] == 1
    assert s["covered_edges"] == 1
    db.close()


def test_compute_coverage_with_tag_filter(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes, filters=["tag:zone_b"])
    s = report.summary()
    # WP2 and WP3 carry zone_b, so both edges are incident (WP1_WP2 via WP2,
    # WP2_WP3 via both)
    assert s["total_edges"] == 2
    db.close()


def test_compute_coverage_tag_summary(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes)
    tags = report.tag_summary()
    assert "zone_a" in tags
    assert "zone_b" in tags
    assert tags["zone_a"]["total_edges"] == 1
    assert tags["zone_a"]["covered_edges"] == 1
    assert tags["zone_b"]["total_edges"] == 2
    assert tags["zone_b"]["covered_edges"] == 1
    db.close()


def test_compute_coverage_no_traversals(tmp_path):
    db = _make_db(tmp_path, add_traversals=False)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes)
    s = report.summary()
    assert s["total_edges"] == 2
    assert s["covered_edges"] == 0
    assert s["coverage_pct"] == 0.0
    db.close()


def test_default_min_success_threshold_is_two():
    assert DEFAULT_MIN_SUCCESS_TRAVERSALS == 2


def test_compute_coverage_default_min_success_is_two(tmp_path):
    # _make_db records exactly 2 successful traversals for WP1_WP2.
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes)
    edge = next(e for e in report.edges if e.edge_id == "WP1_WP2")
    assert edge.min_success == 2
    assert edge.covered is True
    db.close()


def test_compute_coverage_min_success_configurable_raises_threshold(tmp_path):
    # With only 2 successes, requiring 3 should make the edge uncovered.
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes, min_success=3)
    s = report.summary()
    edge = next(e for e in report.edges if e.edge_id == "WP1_WP2")
    assert edge.covered is False
    assert s["covered_edges"] == 0
    db.close()


def test_compute_coverage_min_success_configurable_lowers_threshold(tmp_path):
    # With min_success=1, a single success is already enough to be covered.
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes, min_success=1)
    edge = next(e for e in report.edges if e.edge_id == "WP1_WP2")
    assert edge.covered is True
    db.close()


def test_compute_coverage_rejects_non_positive_min_success(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    with pytest.raises(ValueError):
        compute_coverage(graph, db, hashes, min_success=0)
    db.close()


def test_resolve_map_hashes_not_found(tmp_path):
    db = _make_db(tmp_path)
    try:
        resolve_map_hashes(db, ["nonexistent"])
        assert False, "expected ValueError"
    except ValueError:
        pass
    db.close()


# ---------------------------------------------------------------------------
# Markdown rendering
# ---------------------------------------------------------------------------

def test_render_markdown_summary(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes)
    md = render_markdown_summary(report)
    assert "Coverage Summary" in md
    assert "50.0" in md
    db.close()


def test_render_markdown_report_lists_uncovered_edges(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes)
    md = render_markdown_report(report, map_names=["TestMap"])
    assert "WP2_WP3" in md
    assert "Uncovered Edges" in md
    db.close()


# ---------------------------------------------------------------------------
# SVG rendering
# ---------------------------------------------------------------------------

def test_generate_coverage_svg(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes)
    out_path = tmp_path / "out.svg"
    svg = generate_coverage_svg(graph, report, str(out_path))
    assert "<svg" in svg
    assert out_path.read_text(encoding="utf-8") == svg
    db.close()


def test_generate_coverage_svg_greys_out_unselected(tmp_path):
    db = _make_db(tmp_path)
    graph = build_merged_graph_for_maps(db, ["TestMap"])
    hashes = resolve_map_hashes(db, ["TestMap"])
    report = compute_coverage(graph, db, hashes, filters=["name:WP1"])
    out_path = tmp_path / "out.svg"
    svg = generate_coverage_svg(graph, report, str(out_path))
    # The unselected WP2_WP3 edge should be drawn with the "not selected" grey colour
    assert "#cccccc" in svg
    db.close()
