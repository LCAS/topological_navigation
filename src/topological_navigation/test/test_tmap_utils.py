"""Tests for tmap_utils module.

Covers CustomSafeLoader, NoAliasDumper, get_node_from_tmap2,
and get_edge_from_id_tmap2.
"""

import yaml
import pytest

from topological_navigation.tmap_utils import (
    CustomSafeLoader,
    NoAliasDumper,
    get_node_from_tmap2,
    get_edge_from_id_tmap2,
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
