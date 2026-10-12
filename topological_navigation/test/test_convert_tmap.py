"""Tests for convert_tmap module.

Covers _map_action_name, _convert_node, _convert_transformation, and
convert_tmap.
"""

import pytest

from topological_navigation.convert_tmap import (
    _map_action_name,
    _convert_node,
    _convert_transformation,
    convert_tmap,
)


# ---------- _map_action_name ------------------------------------------

class TestMapActionName:
    """Tests for CamelCase -> snake_case mapping."""

    def test_known_actions(self):
        assert _map_action_name("NavigateToPose") == "navigate_to_pose"
        assert _map_action_name("RowOperation") == "row_traversal"
        assert _map_action_name("GoalAlign") == "goal_align"

    def test_already_snake_case_known(self):
        """Known lowercase forms also map correctly via the dict."""
        assert _map_action_name("RowTraversal") == "row_traversal"

    def test_unknown_camel_case(self):
        """Unknown CamelCase names are converted to snake_case."""
        result = _map_action_name("CustomBehavior")
        assert result == "custom_behavior"

    def test_unknown_single_word(self):
        """Single-word names stay lowercase."""
        assert _map_action_name("stop") == "stop"


# ---------- _convert_node ---------------------------------------------

class TestConvertNode:
    """Tests for converting a single node entry."""

    @pytest.fixture
    def old_node(self):
        return {
            "meta": {
                "map": "test",
                "node": "A",
                "pointset": "test",
                "tag": "old_tag",
            },
            "node": {
                "name": "A",
                "pose": {"position": {"x": 1, "y": 2, "z": 0},
                         "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}},
                "edges": [
                    {
                        "action": "NavigateToPose",
                        "action_type": "nav2_msgs/action/NavigateToPose",
                        "edge_id": "A_B",
                        "node": "B",
                    }
                ],
                "properties": {"xy_goal_tolerance": 0.3},
                "verts": [{"x": -1, "y": -1}, {"x": 1, "y": 1}],
                "localise_by_topic": "some_topic",
                "parent_frame": "map",
            },
        }

    def test_action_type_removed(self, old_node):
        """action_type must be removed from converted edges."""
        result = _convert_node(old_node)
        for edge in result["node"]["edges"]:
            assert "action_type" not in edge

    def test_action_name_mapped(self, old_node):
        """Edge action names are converted to snake_case."""
        result = _convert_node(old_node)
        assert result["node"]["edges"][0]["action"] == "navigate_to_pose"

    def test_localise_by_topic_removed(self, old_node):
        """localise_by_topic is stripped from the node."""
        result = _convert_node(old_node)
        assert "localise_by_topic" not in result["node"]

    def test_parent_frame_removed(self, old_node):
        """parent_frame is stripped from the node."""
        result = _convert_node(old_node)
        assert "parent_frame" not in result["node"]

    def test_tag_removed_from_meta(self, old_node):
        """tag is stripped from the meta."""
        result = _convert_node(old_node)
        assert "tag" not in result["meta"]

    def test_properties_preserved(self, old_node):
        """Node properties are kept as-is."""
        result = _convert_node(old_node)
        assert result["node"]["properties"]["xy_goal_tolerance"] == 0.3

    def test_verts_preserved(self, old_node):
        """Influence zone vertices are kept."""
        result = _convert_node(old_node)
        assert len(result["node"]["verts"]) == 2

    def test_core_fields(self, old_node):
        """Name and pose are preserved."""
        result = _convert_node(old_node)
        assert result["node"]["name"] == "A"
        assert "pose" in result["node"]


# ---------- _convert_transformation -----------------------------------

class TestConvertTransformation:
    """Tests for transformation block conversion."""

    def test_child_renamed(self):
        """'child' key becomes 'topo_frame_id'."""
        result = _convert_transformation({"child": "map"})
        assert result["topo_frame_id"] == "map"
        assert "child" not in result

    def test_already_new_format(self):
        """If topo_frame_id is already there (no child), it's preserved."""
        result = _convert_transformation({"topo_frame_id": "map"})
        assert result["topo_frame_id"] == "map"

    def test_extra_keys_preserved(self):
        """Other transformation keys are kept."""
        result = _convert_transformation({
            "child": "map",
            "rotation": {"w": 1.0},
            "translation": {"x": 0.0},
        })
        assert "rotation" in result
        assert "translation" in result


# ---------- convert_tmap (integration) ---------------------------------

class TestConvertTmap:
    """Integration test for full tmap conversion."""

    @pytest.fixture
    def old_map(self):
        return {
            "meta": {"last_updated": "01-01-2026"},
            "metric_map": "test_map",
            "name": "test_map",
            "pointset": "test_map",
            "transformation": {
                "child": "test_map",
                "parent": "map",
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "nodes": [
                {
                    "meta": {"map": "test_map", "node": "A", "pointset": "test_map"},
                    "node": {
                        "name": "A",
                        "pose": {"position": {"x": 0, "y": 0, "z": 0},
                                 "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}},
                        "edges": [
                            {
                                "action": "NavigateToPose",
                                "action_type": "nav2_msgs/action/NavigateToPose",
                                "edge_id": "A_B",
                                "node": "B",
                            },
                            {
                                "action": "RowOperation",
                                "action_type": "nav2_msgs/action/NavigateThroughPoses",
                                "edge_id": "A_C",
                                "node": "C",
                            },
                        ],
                        "verts": [{"x": -1, "y": -1}, {"x": 1, "y": 1}],
                    },
                },
                {
                    "meta": {"map": "test_map", "node": "B", "pointset": "test_map"},
                    "node": {
                        "name": "B",
                        "pose": {"position": {"x": 5, "y": 0, "z": 0},
                                 "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}},
                        "edges": [],
                    },
                },
                {
                    "meta": {"map": "test_map", "node": "C", "pointset": "test_map"},
                    "node": {
                        "name": "C",
                        "pose": {"position": {"x": 0, "y": 5, "z": 0},
                                 "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}},
                        "edges": [],
                    },
                },
            ],
        }

    def test_top_level_keys(self, old_map):
        """Converted map has expected top-level keys."""
        result = convert_tmap(old_map)
        for key in ("meta", "transformation", "nodes"):
            assert key in result

    def test_no_top_level_actions_or_definitions(self, old_map):
        """The current converter does not emit action configuration."""
        result = convert_tmap(old_map)
        assert "actions" not in result
        assert "definitions" not in result

    def test_transformation_converted(self, old_map):
        """Transformation block has topo_frame_id."""
        result = convert_tmap(old_map)
        assert "topo_frame_id" in result["transformation"]

    def test_node_count_preserved(self, old_map):
        """All nodes are carried over."""
        result = convert_tmap(old_map)
        assert len(result["nodes"]) == len(old_map["nodes"])

    def test_edges_have_no_action_type(self, old_map):
        """Converted edges must not contain action_type."""
        result = convert_tmap(old_map)
        for node_entry in result["nodes"]:
            for edge in node_entry["node"]["edges"]:
                assert "action_type" not in edge

    def test_edge_actions_are_snake_case(self, old_map):
        """Converted edge actions use snake_case."""
        result = convert_tmap(old_map)
        edge = result["nodes"][0]["node"]["edges"][0]
        assert edge["action"] == "navigate_to_pose"

    def test_empty_nodes(self):
        """Converting a map with no nodes still produces valid output."""
        result = convert_tmap({"nodes": [], "transformation": {}})
        assert result["nodes"] == []
