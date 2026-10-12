"""Unit tests for the map_merger module.

Covers GPS reprojection, zero-origin handling, node/edge name
deduplication, meta/fields merging, schema validity of merged output, and
CLI behaviour.
"""

import os

import pytest
from pyproj import Transformer

from topological_navigation.map_analyser import analyse_map, main as analyser_main
from topological_navigation.map_merger import MergeResult, merge_maps
from topological_navigation.tmap_utils import load_tmap2_file


FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")
SIMPLE_MAP = os.path.join(FIXTURES, "simple_map.yaml")
COMPLEX_MAP = os.path.join(FIXTURES, "complex_map.yaml")
MERGE_A = os.path.join(FIXTURES, "mergeable_map_a.yaml")
MERGE_B = os.path.join(FIXTURES, "mergeable_map_b.yaml")


def _expected_offset(ref_lat, ref_lon, lat, lon):
    """Independent re-computation of the expected (east, north) offset for assertions."""
    aeqd_crs = f"+proj=aeqd +lat_0={ref_lat} +lon_0={ref_lon} +datum=WGS84 +units=m +no_defs"
    transformer = Transformer.from_crs("EPSG:4326", aeqd_crs, always_xy=True)
    return transformer.transform(lon, lat)


def _node(tmap, name):
    for entry in tmap["nodes"]:
        if entry["node"]["name"] == name:
            return entry
    raise KeyError(name)


class TestReprojection:
    def test_reference_map_positions_unchanged(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        wp1 = _node(merged, "WP1")
        assert wp1["node"]["pose"]["position"]["x"] == pytest.approx(0.0)
        assert wp1["node"]["pose"]["position"]["y"] == pytest.approx(0.0)

    def test_second_map_positions_reprojected(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        assert isinstance(result, MergeResult)
        merged = load_tmap2_file(str(out))

        east, north = _expected_offset(51.5000000, -0.1000000, 51.5010000, -0.0990000)

        wp1_2 = _node(merged, "WP1_2")
        assert wp1_2["node"]["pose"]["position"]["x"] == pytest.approx(east, abs=1e-3)
        assert wp1_2["node"]["pose"]["position"]["y"] == pytest.approx(north, abs=1e-3)
        assert wp1_2["node"]["pose"]["position"]["z"] == pytest.approx(5.0)  # 15 - 10 altitude diff

        wp2_2 = _node(merged, "WP2_2")
        assert wp2_2["node"]["pose"]["position"]["x"] == pytest.approx(east, abs=1e-3)
        assert wp2_2["node"]["pose"]["position"]["y"] == pytest.approx(5.0 + north, abs=1e-3)

        offsets_by_file = {f: (e, n, a, r) for f, e, n, a, r in result.per_map_offsets}
        assert offsets_by_file[MERGE_A][3] is False
        assert offsets_by_file[MERGE_B][3] is True

    def test_zero_origin_maps_merge_unchanged(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([SIMPLE_MAP, COMPLEX_MAP], output_file=str(out))
        merged = load_tmap2_file(str(out))

        wp1 = _node(merged, "WP1")
        assert wp1["node"]["pose"]["position"]["x"] == pytest.approx(0.0)
        assert wp1["node"]["pose"]["position"]["y"] == pytest.approx(0.0)
        assert any("no real GPS origin" in w for w in result.warnings)


class TestNameDeduplication:
    def test_colliding_node_names_are_suffixed(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        names = [entry["node"]["name"] for entry in merged["nodes"]]
        assert names == ["WP1", "WP2", "WP1_2", "WP2_2"]
        assert (MERGE_B, "WP1", "WP1_2") in result.node_renames
        assert (MERGE_B, "WP2", "WP2_2") in result.node_renames

    def test_edge_target_rewritten_after_node_rename(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        wp1_2 = _node(merged, "WP1_2")
        assert wp1_2["node"]["edges"][0]["node"] == "WP2_2"

    def test_colliding_edge_ids_are_suffixed(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        wp1 = _node(merged, "WP1")
        wp1_2 = _node(merged, "WP1_2")
        assert wp1["node"]["edges"][0]["edge_id"] == "WP1_WP2"
        assert wp1_2["node"]["edges"][0]["edge_id"] == "WP1_WP2_2"
        assert (MERGE_B, "WP1_WP2", "WP1_WP2_2") in result.edge_renames

    def test_per_node_meta_rewritten_to_merged_identity(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        wp1_2 = _node(merged, "WP1_2")
        assert wp1_2["meta"]["map"] == "mergeable_map_a"
        assert wp1_2["meta"]["pointset"] == "mergeable_map_a"
        assert wp1_2["meta"]["node"] == "WP1_2"


class TestMetaMerge:
    def test_fields_concatenated_and_renumbered(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        fields = merged["meta"]["fields"]
        assert [f["name"] for f in fields] == ["boundary_a", "boundary_b"]
        assert [f["field_number"] for f in fields] == [1, 2]
        assert result.fields_merged == 2

    def test_transformation_conflict_keeps_first_and_warns(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        assert merged["transformation"]["topo_frame_id"] == "map_a_frame"
        assert any("transformation" in w for w in result.warnings)

    def test_origin_is_always_reference_maps_origin(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        assert merged["meta"]["origin"]["latitude"] == pytest.approx(51.5)
        assert merged["meta"]["origin"]["longitude"] == pytest.approx(-0.1)

    def test_last_updated_is_regenerated(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        merged = load_tmap2_file(str(out))

        assert merged["meta"]["last_updated"] not in ("01-01-2026_00-00-00", "02-02-2026_00-00-00")

    def test_name_override(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out), name="custom_name")
        merged = load_tmap2_file(str(out))

        assert merged["name"] == "custom_name"
        assert merged["metric_map"] == "custom_name"
        assert merged["pointset"] == "custom_name"


class TestSchemaValidity:
    def test_merged_output_is_schema_valid(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        assert result.schema_valid is True

    def test_analyse_map_reports_no_new_orphans_or_overlaps(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        analysis = analyse_map(str(out))

        # Each source map already has WP1 as an orphan (no incoming edges);
        # merging must not introduce any orphans beyond the renamed originals.
        assert set(analysis.orphaned_nodes) == {"WP1", "WP1_2"}
        assert analysis.overlaps == []

    def test_invalid_input_raises(self, tmp_path):
        bad = tmp_path / "bad.yaml"
        bad.write_text("name: bad\n")  # missing required schema fields
        with pytest.raises(ValueError):
            merge_maps([MERGE_A, str(bad)], output_file=str(tmp_path / "out.yaml"))

    def test_single_file_raises(self):
        with pytest.raises(ValueError):
            merge_maps([MERGE_A])


class TestDerivedOutputPath:
    def test_default_output_path_derived_from_first_input(self, tmp_path):
        src_a = tmp_path / "site_a.tmap2.yaml"
        src_a.write_text(open(MERGE_A, encoding="utf-8").read())
        src_b = tmp_path / "site_b.tmap2.yaml"
        src_b.write_text(open(MERGE_B, encoding="utf-8").read())

        result = merge_maps([str(src_a), str(src_b)])
        expected = tmp_path / "site_a.merged.tmap2.yaml"
        assert result.output_file == str(expected)
        assert expected.is_file()


class TestFormatReport:
    def test_format_report_contains_all_sections(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        report = result.format_report()
        for section in (
            "[Reference origin]", "[Per-map reprojection offsets]",
            "[Node renames]", "[Edge renames]", "[Connecting edges]", "[Warnings]",
            "[Schema validation of merged output]",
        ):
            assert section in report


class TestConnectClosest:
    def test_disabled_by_default(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        assert result.connecting_edges == []

    def test_adds_one_bidirectional_edge_between_closest_nodes(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out), connect_closest=True)

        assert len(result.connecting_edges) == 1
        node_a, node_b, edge_id_ab, edge_id_ba, dist = result.connecting_edges[0]
        assert node_a in ("WP1", "WP2")
        assert node_b in ("WP1_2", "WP2_2")
        assert dist > 0

        merged = load_tmap2_file(str(out))
        edges_a = {e["edge_id"]: e for e in _node(merged, node_a)["node"]["edges"]}
        edges_b = {e["edge_id"]: e for e in _node(merged, node_b)["node"]["edges"]}
        assert edges_a[edge_id_ab]["node"] == node_b
        assert edges_b[edge_id_ba]["node"] == node_a
        assert edges_a[edge_id_ab]["action"] == "navigate_to_pose"
        assert edges_a[edge_id_ab]["action_type"] == "nav2_msgs/action/NavigateToPose"

    def test_connecting_edge_is_actually_the_closest_pair(self, tmp_path):
        out = tmp_path / "merged.yaml"
        result = merge_maps([MERGE_A, MERGE_B], output_file=str(out), connect_closest=True)
        merged = load_tmap2_file(str(out))

        a_names = ("WP1", "WP2")
        b_names = ("WP1_2", "WP2_2")

        def pos(name):
            p = _node(merged, name)["node"]["pose"]["position"]
            return p["x"], p["y"], p["z"]

        def dist(n1, n2):
            p1, p2 = pos(n1), pos(n2)
            return sum((c1 - c2) ** 2 for c1, c2 in zip(p1, p2)) ** 0.5

        expected_dist = min(dist(a, b) for a in a_names for b in b_names)
        _, _, _, _, actual_dist = result.connecting_edges[0]
        assert actual_dist == pytest.approx(expected_dist, abs=1e-6)

    def test_merged_map_becomes_a_single_connected_component(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out), connect_closest=True)
        analysis = analyse_map(str(out))
        assert len(analysis.disconnected_components) == 1

    def test_without_connect_closest_stays_disconnected(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps([MERGE_A, MERGE_B], output_file=str(out))
        analysis = analyse_map(str(out))
        assert len(analysis.disconnected_components) == 2

    def test_custom_connect_action(self, tmp_path):
        out = tmp_path / "merged.yaml"
        merge_maps(
            [MERGE_A, MERGE_B], output_file=str(out), connect_closest=True,
            connect_action="custom_action", connect_action_type="custom_pkg/action/Custom",
        )
        merged = load_tmap2_file(str(out))
        connecting = [
            edge
            for entry in merged["nodes"]
            for edge in entry["node"]["edges"]
            if edge["edge_id"].startswith("connect_")
        ]
        assert connecting
        for edge in connecting:
            assert edge["action"] == "custom_action"
            assert edge["action_type"] == "custom_pkg/action/Custom"

    def test_cli_connect_closest_flag(self, tmp_path):
        out = tmp_path / "cli_merged.yaml"
        analyser_main(["merge", MERGE_A, MERGE_B, "-o", str(out), "--connect-closest"])
        merged = load_tmap2_file(str(out))
        assert any(
            edge["edge_id"].startswith("connect_")
            for entry in merged["nodes"]
            for edge in entry["node"]["edges"]
        )


class TestCli:
    def test_merge_command_writes_file_and_prints_report(self, tmp_path, capsys):
        out = tmp_path / "cli_merged.yaml"
        analyser_main(["merge", MERGE_A, MERGE_B, "-o", str(out)])
        assert out.is_file()
        captured = capsys.readouterr()
        assert "Merge report" in captured.out

    def test_merge_command_requires_two_files(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            analyser_main(["merge", MERGE_A])
        assert exc_info.value.code == 2

    def test_merge_command_exits_two_for_missing_file(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            analyser_main(["merge", MERGE_A, "/no/such/file.yaml"])
        assert exc_info.value.code == 2
