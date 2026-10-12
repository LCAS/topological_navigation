"""Tests for ``nav_stats_db.py`` -- SQLite traversal statistics persistence."""

import json
import tempfile
import os
from datetime import datetime

import pytest

from topological_navigation.nav_stats_db import NavStatsDB, compute_map_hash


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def tmp_db(tmp_path):
    """Return an open NavStatsDB backed by a temporary file."""
    db = NavStatsDB(str(tmp_path / "test_stats.db"))
    yield db
    db.close()


# ---------------------------------------------------------------------------
# compute_map_hash
# ---------------------------------------------------------------------------

def test_compute_map_hash_deterministic():
    h1 = compute_map_hash("some map yaml content")
    h2 = compute_map_hash("some map yaml content")
    assert h1 == h2


def test_compute_map_hash_differs_for_different_content():
    h1 = compute_map_hash("map_a: true")
    h2 = compute_map_hash("map_b: true")
    assert h1 != h2


def test_compute_map_hash_short():
    h = compute_map_hash("x")
    assert len(h) == 12
    assert all(c in "0123456789abcdef" for c in h)


# ---------------------------------------------------------------------------
# Database creation
# ---------------------------------------------------------------------------

def test_db_file_created(tmp_path):
    db_path = str(tmp_path / "new.db")
    db = NavStatsDB(db_path)
    db.close()
    assert os.path.isfile(db_path)


def test_db_parent_dirs_created(tmp_path):
    db_path = str(tmp_path / "deep" / "nested" / "stats.db")
    db = NavStatsDB(db_path)
    db.close()
    assert os.path.isfile(db_path)


def test_db_can_be_reopened(tmp_path):
    """Existing database is opened and reused without error."""
    db_path = str(tmp_path / "reopen.db")
    db1 = NavStatsDB(db_path)
    db1.record_traversal(
        map_name="map1", map_hash="abc", edge_id="A_B",
        origin="A", target="B", status="success",
    )
    db1.close()

    db2 = NavStatsDB(db_path)
    ids = db2.edge_ids()
    db2.close()
    assert "A_B" in ids


# ---------------------------------------------------------------------------
# record_traversal
# ---------------------------------------------------------------------------

def test_record_returns_rowid(tmp_db):
    rowid = tmp_db.record_traversal(
        map_name="test_map", map_hash="hash1", edge_id="A_B",
        origin="A", target="B", status="success",
    )
    assert isinstance(rowid, int)
    assert rowid >= 1


def test_record_and_query(tmp_db):
    tmp_db.record_traversal(
        map_name="mymap", map_hash="hh", edge_id="N1_N2",
        origin="N1", target="N2", status="success",
        start_time=datetime(2024, 1, 1, 12, 0, 0),
        end_time=datetime(2024, 1, 1, 12, 0, 30),
        duration_s=30.0,
        edge_length=10.0,
    )
    rows = tmp_db.query("SELECT * FROM traversals")
    assert len(rows) == 1
    r = rows[0]
    assert r["map_name"] == "mymap"
    assert r["edge_id"] == "N1_N2"
    assert r["status"] == "success"
    assert abs(r["duration_s"] - 30.0) < 0.001
    assert abs(r["avg_speed"] - 10.0 / 30.0) < 0.0001


def test_record_failed_with_reason(tmp_db):
    tmp_db.record_traversal(
        map_name="map", map_hash="h", edge_id="X_Y",
        origin="X", target="Y", status="failed",
        failure_reason="timeout",
    )
    rows = tmp_db.query("SELECT failure_reason, status FROM traversals")
    assert rows[0]["failure_reason"] == "timeout"
    assert rows[0]["status"] == "failed"


def test_record_segment(tmp_db):
    tmp_db.record_traversal(
        map_name="map", map_hash="h", edge_id="A_B",
        origin="A", target="C", status="success",
        is_segment=True,
        segment_edges=["A_B", "B_C"],
    )
    rows = tmp_db.query("SELECT is_segment, segment_edges FROM traversals")
    assert rows[0]["is_segment"] == 1
    decoded = json.loads(rows[0]["segment_edges"])
    assert decoded == ["A_B", "B_C"]


def test_record_no_segment(tmp_db):
    tmp_db.record_traversal(
        map_name="map", map_hash="h", edge_id="A_B",
        origin="A", target="B", status="success",
    )
    rows = tmp_db.query("SELECT is_segment, segment_edges FROM traversals")
    assert rows[0]["is_segment"] == 0
    assert rows[0]["segment_edges"] is None


def test_avg_speed_none_when_no_length(tmp_db):
    tmp_db.record_traversal(
        map_name="map", map_hash="h", edge_id="A_B",
        origin="A", target="B", status="success",
        duration_s=10.0,
    )
    rows = tmp_db.query("SELECT avg_speed FROM traversals")
    assert rows[0]["avg_speed"] is None


def test_avg_speed_none_when_zero_duration(tmp_db):
    tmp_db.record_traversal(
        map_name="map", map_hash="h", edge_id="A_B",
        origin="A", target="B", status="success",
        duration_s=0.0,
        edge_length=5.0,
    )
    rows = tmp_db.query("SELECT avg_speed FROM traversals")
    assert rows[0]["avg_speed"] is None


def test_duration_derived_from_timestamps(tmp_db):
    t0 = datetime(2024, 6, 1, 10, 0, 0)
    t1 = datetime(2024, 6, 1, 10, 0, 45)
    tmp_db.record_traversal(
        map_name="map", map_hash="h", edge_id="A_B",
        origin="A", target="B", status="success",
        start_time=t0, end_time=t1,
    )
    rows = tmp_db.query("SELECT duration_s FROM traversals")
    assert abs(rows[0]["duration_s"] - 45.0) < 0.001


def test_multiple_records(tmp_db):
    for i in range(5):
        tmp_db.record_traversal(
            map_name="map", map_hash="h", edge_id="A_B",
            origin="A", target="B", status="success",
            duration_s=float(10 + i),
        )
    rows = tmp_db.query("SELECT COUNT(*) AS cnt FROM traversals")
    assert rows[0]["cnt"] == 5


# ---------------------------------------------------------------------------
# edge_ids
# ---------------------------------------------------------------------------

def test_edge_ids_empty(tmp_db):
    assert tmp_db.edge_ids() == []


def test_edge_ids_sorted(tmp_db):
    for eid in ["Z_A", "A_B", "M_N"]:
        tmp_db.record_traversal(
            map_name="m", map_hash="h", edge_id=eid,
            origin="x", target="y", status="success",
        )
    assert tmp_db.edge_ids() == ["A_B", "M_N", "Z_A"]


def test_edge_ids_with_filter(tmp_db):
    tmp_db.record_traversal(
        map_name="mapA", map_hash="h", edge_id="A_B",
        origin="A", target="B", status="success",
    )
    tmp_db.record_traversal(
        map_name="mapB", map_hash="h", edge_id="C_D",
        origin="C", target="D", status="success",
    )
    ids = tmp_db.edge_ids(where="map_name = 'mapA'")
    assert ids == ["A_B"]


# ---------------------------------------------------------------------------
# edge_stats
# ---------------------------------------------------------------------------

def test_edge_stats_counts(tmp_db):
    for status in ["success", "success", "failed", "aborted"]:
        tmp_db.record_traversal(
            map_name="m", map_hash="h", edge_id="E1_E2",
            origin="E1", target="E2", status=status, duration_s=10.0,
        )
    s = tmp_db.edge_stats("E1_E2")
    assert s["total"] == 4
    assert s["success"] == 2
    assert s["failed"] == 1
    assert s["aborted"] == 1


def test_edge_stats_avg_duration(tmp_db):
    for d in [10.0, 20.0, 30.0]:
        tmp_db.record_traversal(
            map_name="m", map_hash="h", edge_id="X_Y",
            origin="X", target="Y", status="success", duration_s=d,
        )
    s = tmp_db.edge_stats("X_Y")
    assert abs(s["avg_duration_s"] - 20.0) < 0.001
    assert abs(s["min_duration_s"] - 10.0) < 0.001
    assert abs(s["max_duration_s"] - 30.0) < 0.001


def test_edge_stats_unknown_edge(tmp_db):
    s = tmp_db.edge_stats("nonexistent")
    assert s == {}


def test_edge_stats_with_filter(tmp_db):
    tmp_db.record_traversal(
        map_name="mapA", map_hash="h", edge_id="A_B",
        origin="A", target="B", status="success", duration_s=5.0,
    )
    tmp_db.record_traversal(
        map_name="mapB", map_hash="h", edge_id="A_B",
        origin="A", target="B", status="failed", duration_s=8.0,
    )
    # With filter only mapA
    s = tmp_db.edge_stats("A_B", where="map_name = 'mapA'")
    assert s["total"] == 1
    assert s["success"] == 1


# ---------------------------------------------------------------------------
# store_map / get_map / list_maps / delete_map
# ---------------------------------------------------------------------------

_SIMPLE_YAML = """\
meta:
  origin:
    latitude: 51.2096
    longitude: 0.4946
name: Test Field
pointset: test_field
nodes: []
"""

_ANOTHER_YAML = """\
meta:
  origin:
    latitude: 53.2686
    longitude: -0.5245
name: Another Map
pointset: another_map
nodes: []
"""


def test_store_map_returns_hash(tmp_db):
    h = tmp_db.store_map(_SIMPLE_YAML)
    expected = compute_map_hash(_SIMPLE_YAML)
    assert h == expected


def test_store_map_idempotent(tmp_db):
    """Storing the same map twice should produce exactly one row."""
    tmp_db.store_map(_SIMPLE_YAML)
    tmp_db.store_map(_SIMPLE_YAML)
    rows = tmp_db.query("SELECT COUNT(*) AS cnt FROM topological_maps")
    assert rows[0]["cnt"] == 1


def test_store_map_extracts_metadata(tmp_db):
    tmp_db.store_map(_SIMPLE_YAML)
    rows = tmp_db.query("SELECT * FROM topological_maps")
    assert len(rows) == 1
    r = rows[0]
    assert r["map_name"] == "Test Field"
    assert abs(r["latitude"] - 51.2096) < 1e-4
    assert abs(r["longitude"] - 0.4946) < 1e-4


def test_store_map_stores_full_yaml(tmp_db):
    tmp_db.store_map(_SIMPLE_YAML)
    rows = tmp_db.query("SELECT map_data FROM topological_maps")
    assert rows[0]["map_data"] == _SIMPLE_YAML


def test_get_map_by_hash(tmp_db):
    h = tmp_db.store_map(_SIMPLE_YAML)
    m = tmp_db.get_map(h)
    assert m is not None
    assert m["map_hash"] == h
    assert m["map_name"] == "Test Field"


def test_get_map_by_name(tmp_db):
    tmp_db.store_map(_SIMPLE_YAML)
    m = tmp_db.get_map("Test Field")
    assert m is not None
    assert m["map_name"] == "Test Field"


def test_get_map_not_found(tmp_db):
    assert tmp_db.get_map("nonexistent") is None


def test_list_maps_empty(tmp_db):
    assert tmp_db.list_maps() == []


def test_list_maps_multiple(tmp_db):
    tmp_db.store_map(_SIMPLE_YAML)
    tmp_db.store_map(_ANOTHER_YAML)
    maps = tmp_db.list_maps()
    assert len(maps) == 2
    names = {m["map_name"] for m in maps}
    assert "Test Field" in names
    assert "Another Map" in names


def test_delete_map_by_hash(tmp_db):
    h = tmp_db.store_map(_SIMPLE_YAML)
    count = tmp_db.delete_map(h)
    assert count == 1
    assert tmp_db.get_map(h) is None


def test_delete_map_by_name(tmp_db):
    tmp_db.store_map(_SIMPLE_YAML)
    count = tmp_db.delete_map("Test Field")
    assert count == 1
    assert tmp_db.get_map("Test Field") is None


def test_delete_map_not_found(tmp_db):
    count = tmp_db.delete_map("nonexistent")
    assert count == 0


def test_store_map_added_at_populated(tmp_db):
    tmp_db.store_map(_SIMPLE_YAML)
    rows = tmp_db.query("SELECT added_at FROM topological_maps")
    assert rows[0]["added_at"] is not None
    assert len(rows[0]["added_at"]) > 0


# ---------------------------------------------------------------------------
# merge_from
# ---------------------------------------------------------------------------

def test_merge_from_copies_maps_and_traversals(tmp_path):
    src_path = str(tmp_path / "src.db")
    dst_path = str(tmp_path / "dst.db")

    src = NavStatsDB(src_path)
    h = src.store_map(_SIMPLE_YAML)
    src.record_traversal(
        map_name="Test Field", map_hash=h, edge_id="A_B",
        origin="A", target="B", status="success", duration_s=5.0,
    )
    src.close()

    dst = NavStatsDB(dst_path)
    result = dst.merge_from(src_path)
    assert result["maps_added"] == 1
    assert result["maps_skipped"] == 0
    assert result["traversals_added"] == 1

    assert dst.get_map("Test Field") is not None
    trav = dst.query("SELECT * FROM traversals")
    assert len(trav) == 1
    assert trav[0]["edge_id"] == "A_B"
    dst.close()


def test_merge_from_skips_existing_maps(tmp_path):
    src_path = str(tmp_path / "src.db")
    dst_path = str(tmp_path / "dst.db")

    src = NavStatsDB(src_path)
    src.store_map(_SIMPLE_YAML)
    src.close()

    dst = NavStatsDB(dst_path)
    dst.store_map(_SIMPLE_YAML)  # already present with the same hash
    result = dst.merge_from(src_path)
    assert result["maps_added"] == 0
    assert result["maps_skipped"] == 1

    maps = dst.list_maps()
    assert len(maps) == 1
    dst.close()


def test_merge_from_is_additive_for_traversals(tmp_path):
    """Merging the same source twice should double the traversal count."""
    src_path = str(tmp_path / "src.db")
    dst_path = str(tmp_path / "dst.db")

    src = NavStatsDB(src_path)
    h = src.store_map(_SIMPLE_YAML)
    src.record_traversal(
        map_name="Test Field", map_hash=h, edge_id="A_B",
        origin="A", target="B", status="success", duration_s=5.0,
    )
    src.close()

    dst = NavStatsDB(dst_path)
    dst.merge_from(src_path)
    dst.merge_from(src_path)
    trav = dst.query("SELECT * FROM traversals")
    assert len(trav) == 2
    dst.close()


def test_merge_from_multiple_sources(tmp_path):
    src1_path = str(tmp_path / "src1.db")
    src2_path = str(tmp_path / "src2.db")
    dst_path = str(tmp_path / "dst.db")

    src1 = NavStatsDB(src1_path)
    h1 = src1.store_map(_SIMPLE_YAML)
    src1.record_traversal(
        map_name="Test Field", map_hash=h1, edge_id="A_B",
        origin="A", target="B", status="success", duration_s=5.0,
    )
    src1.close()

    src2 = NavStatsDB(src2_path)
    h2 = src2.store_map(_ANOTHER_YAML)
    src2.record_traversal(
        map_name="Another Map", map_hash=h2, edge_id="C_D",
        origin="C", target="D", status="failed", duration_s=5.0,
    )
    src2.close()

    dst = NavStatsDB(dst_path)
    dst.merge_from(src1_path)
    dst.merge_from(src2_path)

    assert len(dst.list_maps()) == 2
    trav = dst.query("SELECT edge_id FROM traversals ORDER BY edge_id")
    assert [r["edge_id"] for r in trav] == ["A_B", "C_D"]
    dst.close()
