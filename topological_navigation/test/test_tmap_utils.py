"""Tests for tmap_utils module."""

import pytest
import yaml

from topological_navigation.tmap_utils import (
    DEFAULT_NAVIGATION_CONFIG_FILENAME,
    NAVIGATION_CONFIG_FILE_KEY,
    CustomSafeLoader,
    NoAliasDumper,
    get_edge_from_id_tmap2,
    get_node_from_tmap2,
    load_tmap2_file,
    save_tmap2_file,
)


# ---------- Fixtures --------------------------------------------------

@pytest.fixture
def sample_tmap():
    """Build a minimal tmap2 structure for testing."""
    return {
        "nodes": [
            {
                "node": {
                    "name": "WP1",
                    "pose": {
                        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "edges": [
                        {
                            "edge_id": "WP1_WP2",
                            "node": "WP2",
                            "action": "navigate_to_pose",
                        },
                        {
                            "edge_id": "WP1_WP3",
                            "node": "WP3",
                            "action": "row_traversal",
                        },
                    ],
                }
            },
            {
                "node": {
                    "name": "WP2",
                    "pose": {
                        "position": {"x": 5.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "edges": [],
                }
            },
        ]
    }


# ---------- CustomSafeLoader ------------------------------------------

class TestCustomSafeLoader:
    """Tests for CustomSafeLoader YAML loading."""

    def test_int_pose_keys_converted_to_float(self):
        """Int values for x, y, z, w are converted to float."""
        doc = "position:\n  x: 1\n  y: 2\n  z: 0\n"
        data = yaml.load(doc, Loader=CustomSafeLoader)
        assert isinstance(data["position"]["x"], float)
        assert isinstance(data["position"]["y"], float)
        assert isinstance(data["position"]["z"], float)

    def test_float_values_remain_float(self):
        """Already-float values remain unchanged."""
        doc = "x: 1.5\ny: 2.5\nz: 0.0\nw: 1.0\n"
        data = yaml.load(doc, Loader=CustomSafeLoader)
        assert data["x"] == 1.5
        assert data["w"] == 1.0

    def test_tolerance_keys_converted(self):
        """Tolerance keys with int values are converted to float."""
        doc = "xy_goal_tolerance: 1\nyaw_goal_tolerance: 2\n"
        data = yaml.load(doc, Loader=CustomSafeLoader)
        assert isinstance(data["xy_goal_tolerance"], float)
        assert isinstance(data["yaw_goal_tolerance"], float)

    def test_non_pose_int_keys_unchanged(self):
        """Keys other than pose/tolerance keep their original type."""
        doc = "capacity: 3\nname: test\n"
        data = yaml.load(doc, Loader=CustomSafeLoader)
        assert isinstance(data["capacity"], int)
        assert isinstance(data["name"], str)

    def test_orientation_w_converted(self):
        """Orientation quaternion w-component int is converted to float."""
        doc = "w: 1\n"
        data = yaml.load(doc, Loader=CustomSafeLoader)
        assert isinstance(data["w"], float)
        assert data["w"] == 1.0


# ---------- NoAliasDumper ----------------------------------------------

class TestNoAliasDumper:
    """Tests for NoAliasDumper YAML output."""

    def test_no_aliases_in_output(self):
        """Duplicate objects are written without YAML anchors/aliases."""
        shared = {"x": 1.0, "y": 2.0}
        data = {"a": shared, "b": shared}
        output = yaml.dump(data, Dumper=NoAliasDumper)
        assert "&" not in output
        assert "*" not in output

    def test_roundtrip_preserves_data(self):
        """Dump then load produces equivalent data."""
        data = {"nodes": [{"name": "A"}, {"name": "B"}]}
        output = yaml.dump(data, Dumper=NoAliasDumper)
        loaded = yaml.safe_load(output)
        assert loaded == data


# ---------- split map IO -----------------------------------------------

class TestSplitMapIo:
    """Tests for split map loading and saving."""

    def test_load_tmap2_file_merges_navigation_config(self, tmp_path):
        """Definitions/actions can be loaded from a sidecar YAML file."""
        config_path = tmp_path / DEFAULT_NAVIGATION_CONFIG_FILENAME
        config_path.write_text(
            yaml.safe_dump(
                {
                    "definitions": {"default_bt": "<root/>"},
                    "actions": {
                        "navigate_to_pose": {
                            "action_type": "nav2_msgs.action.NavigateToPose",
                            "action_server": "/navigate_to_pose",
                            "composable": False,
                        }
                    },
                }
            ),
            encoding="utf-8",
        )
        main_path = tmp_path / "map.tmap2.yaml"
        main_path.write_text(
            yaml.safe_dump(
                {
                    NAVIGATION_CONFIG_FILE_KEY: DEFAULT_NAVIGATION_CONFIG_FILENAME,
                    "meta": {"last_updated": "01-01-2026_00-00-00"},
                    "metric_map": "test_map",
                    "name": "test_map",
                    "pointset": "test_map",
                    "nodes": [],
                }
            ),
            encoding="utf-8",
        )

        loaded, layout = load_tmap2_file(main_path, return_layout=True)

        assert loaded["definitions"]["default_bt"] == "<root/>"
        assert "navigate_to_pose" in loaded["actions"]
        assert loaded[NAVIGATION_CONFIG_FILE_KEY] == DEFAULT_NAVIGATION_CONFIG_FILENAME
        assert layout["section_sources"] == {
            "definitions": "external",
            "actions": "external",
        }

    def test_load_tmap2_file_accepts_explicit_navigation_config(self, tmp_path):
        """A caller can pass the actions/definitions sidecar separately."""
        config_path = tmp_path / DEFAULT_NAVIGATION_CONFIG_FILENAME
        config_path.write_text(
            yaml.safe_dump(
                {
                    "definitions": {"default_bt": "<root/>"},
                    "actions": {
                        "navigate_to_pose": {
                            "action_type": "nav2_msgs.action.NavigateToPose",
                            "action_server": "/navigate_to_pose",
                            "action_goal_template": {
                                "behavior_tree": "${definitions.default_bt}",
                            },
                        }
                    },
                }
            ),
            encoding="utf-8",
        )
        main_path = tmp_path / "map.tmap2.yaml"
        main_path.write_text(
            yaml.safe_dump(
                {
                    "meta": {"last_updated": "01-01-2026_00-00-00"},
                    "metric_map": "test_map",
                    "name": "test_map",
                    "pointset": "test_map",
                    "nodes": [],
                }
            ),
            encoding="utf-8",
        )

        loaded, layout = load_tmap2_file(
            main_path,
            return_layout=True,
            navigation_config_file=DEFAULT_NAVIGATION_CONFIG_FILENAME,
        )

        assert (
            loaded[NAVIGATION_CONFIG_FILE_KEY]
            == DEFAULT_NAVIGATION_CONFIG_FILENAME
        )
        assert loaded["definitions"]["default_bt"] == "<root/>"
        assert "navigate_to_pose" in loaded["actions"]
        assert layout["config_path"] == str(config_path)
        assert layout["section_sources"] == {
            "definitions": "external",
            "actions": "external",
        }

    def test_main_map_sections_take_precedence(self, tmp_path):
        """Inline definitions/actions are kept when a sidecar file is present."""
        config_path = tmp_path / "nav_config.yaml"
        config_path.write_text(
            yaml.safe_dump(
                {
                    "definitions": {"default_bt": "<external/>"},
                    "actions": {"navigate_to_pose": {"action_server": "/external"}},
                }
            ),
            encoding="utf-8",
        )
        main_path = tmp_path / "map.tmap2.yaml"
        main_path.write_text(
            yaml.safe_dump(
                {
                    NAVIGATION_CONFIG_FILE_KEY: "nav_config.yaml",
                    "meta": {"last_updated": "01-01-2026_00-00-00"},
                    "metric_map": "test_map",
                    "name": "test_map",
                    "pointset": "test_map",
                    "definitions": {"default_bt": "<inline/>"},
                    "actions": {"navigate_to_pose": {"action_server": "/inline"}},
                    "nodes": [],
                }
            ),
            encoding="utf-8",
        )

        loaded, layout = load_tmap2_file(main_path, return_layout=True)

        assert loaded["definitions"]["default_bt"] == "<inline/>"
        assert loaded["actions"]["navigate_to_pose"]["action_server"] == "/inline"
        assert layout["section_sources"] == {
            "definitions": "main",
            "actions": "main",
        }

    def test_save_tmap2_file_preserves_split_layout(self, tmp_path):
        """Saving a split map keeps definitions/actions in the sidecar file."""
        main_path = tmp_path / "map.tmap2.yaml"
        config_path = tmp_path / "nav_config.yaml"
        tmap = {
            NAVIGATION_CONFIG_FILE_KEY: "nav_config.yaml",
            "meta": {"last_updated": "01-01-2026_00-00-00"},
            "metric_map": "test_map",
            "name": "test_map",
            "pointset": "test_map",
            "definitions": {"default_bt": "<root/>"},
            "actions": {
                "navigate_to_pose": {
                    "action_type": "nav2_msgs.action.NavigateToPose",
                    "action_server": "/navigate_to_pose",
                    "composable": False,
                }
            },
            "nodes": [],
        }
        layout = {
            "main_path": str(main_path),
            NAVIGATION_CONFIG_FILE_KEY: "nav_config.yaml",
            "config_path": str(config_path),
            "section_sources": {
                "definitions": "external",
                "actions": "external",
            },
        }

        save_tmap2_file(tmap, main_path, layout=layout)

        main_data = yaml.safe_load(main_path.read_text(encoding="utf-8"))
        config_data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        reloaded = load_tmap2_file(main_path)

        assert main_data[NAVIGATION_CONFIG_FILE_KEY] == "nav_config.yaml"
        assert "definitions" not in main_data
        assert "actions" not in main_data
        assert config_data["definitions"]["default_bt"] == "<root/>"
        assert "navigate_to_pose" in config_data["actions"]
        assert reloaded == tmap

    def test_save_tmap2_file_keeps_inline_sections_inline(self, tmp_path):
        """Existing sidecar files are left alone when sections are sourced inline."""
        main_path = tmp_path / "map.tmap2.yaml"
        config_path = tmp_path / "nav_config.yaml"
        config_path.write_text(
            yaml.safe_dump({"definitions": {"default_bt": "<external/>"}}),
            encoding="utf-8",
        )
        tmap = {
            NAVIGATION_CONFIG_FILE_KEY: "nav_config.yaml",
            "meta": {"last_updated": "01-01-2026_00-00-00"},
            "metric_map": "test_map",
            "name": "test_map",
            "pointset": "test_map",
            "definitions": {"default_bt": "<inline/>"},
            "actions": {"navigate_to_pose": {"action_server": "/inline"}},
            "nodes": [],
        }
        layout = {
            "main_path": str(main_path),
            NAVIGATION_CONFIG_FILE_KEY: "nav_config.yaml",
            "config_path": str(config_path),
            "section_sources": {
                "definitions": "main",
                "actions": "main",
            },
        }

        save_tmap2_file(tmap, main_path, layout=layout)

        main_data = yaml.safe_load(main_path.read_text(encoding="utf-8"))
        config_data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

        assert main_data["definitions"]["default_bt"] == "<inline/>"
        assert main_data["actions"]["navigate_to_pose"]["action_server"] == "/inline"
        assert config_data["definitions"]["default_bt"] == "<external/>"


# ---------- get_node_from_tmap2 ----------------------------------------

class TestGetNodeFromTmap2:
    """Tests for get_node_from_tmap2."""

    def test_existing_node(self, sample_tmap):
        """Returns the matching node entry dict."""
        result = get_node_from_tmap2(sample_tmap, "WP1")
        assert result is not None
        assert result["node"]["name"] == "WP1"

    def test_second_node(self, sample_tmap):
        """Can find a node that is not first in the list."""
        result = get_node_from_tmap2(sample_tmap, "WP2")
        assert result is not None
        assert result["node"]["name"] == "WP2"

    def test_nonexistent_node(self, sample_tmap):
        """Returns None when node name is not in the map."""
        result = get_node_from_tmap2(sample_tmap, "MISSING")
        assert result is None

    def test_empty_map(self):
        """Returns None for an empty map."""
        result = get_node_from_tmap2({"nodes": []}, "WP1")
        assert result is None


# ---------- get_edge_from_id_tmap2 -------------------------------------

class TestGetEdgeFromIdTmap2:
    """Tests for get_edge_from_id_tmap2."""

    def test_existing_edge(self, sample_tmap):
        """Returns the matching edge dict."""
        result = get_edge_from_id_tmap2(sample_tmap, "WP1", "WP1_WP2")
        assert result is not None
        assert result["edge_id"] == "WP1_WP2"
        assert result["node"] == "WP2"

    def test_second_edge(self, sample_tmap):
        """Can find an edge that is not the first one."""
        result = get_edge_from_id_tmap2(sample_tmap, "WP1", "WP1_WP3")
        assert result is not None
        assert result["edge_id"] == "WP1_WP3"

    def test_nonexistent_edge(self, sample_tmap):
        """Returns None when edge_id doesn't exist on the node."""
        result = get_edge_from_id_tmap2(sample_tmap, "WP1", "MISSING_EDGE")
        assert result is None

    def test_nonexistent_node(self, sample_tmap):
        """Returns None when node doesn't exist."""
        result = get_edge_from_id_tmap2(sample_tmap, "MISSING", "WP1_WP2")
        assert result is None

    def test_node_with_no_edges(self, sample_tmap):
        """Returns None when the node has an empty edges list."""
        result = get_edge_from_id_tmap2(sample_tmap, "WP2", "anything")
        assert result is None
