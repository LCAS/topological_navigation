"""Tests for convert_tmap module.

Covers _normalise_action_type, _map_action_name, _discover_actions,
_build_actions_section, _build_definitions, _convert_node,
_convert_transformation, and convert_tmap.
"""

import pytest

from topological_navigation.convert_tmap import (
    _normalise_action_type,
    _map_action_name,
    _discover_actions,
    _build_actions_section,
    _build_definitions,
    _convert_node,
    _convert_transformation,
    convert_tmap,
    _BUILTIN_BT_DEFS,
)


# ---------- _normalise_action_type ------------------------------------

class TestNormaliseActionType:
    """Tests for slash-to-dot conversion."""

    def test_slash_to_dot(self):
        assert _normalise_action_type("nav2_msgs/action/NavigateToPose") == \
               "nav2_msgs.action.NavigateToPose"

    def test_already_dotted(self):
        assert _normalise_action_type("nav2_msgs.action.NavigateToPose") == \
               "nav2_msgs.action.NavigateToPose"

    def test_empty_string(self):
        assert _normalise_action_type("") == ""


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


# ---------- _discover_actions -----------------------------------------

class TestDiscoverActions:
    """Tests for action discovery from edge data."""

    def _nodes(self, *actions):
        """Build minimal node list with the given edge actions."""
        return [
            {
                "node": {
                    "edges": [
                        {"action": a, "action_type": "nav2_msgs/action/NavigateToPose"}
                        for a in actions
                    ]
                }
            }
        ]

    def test_discovers_known_action(self):
        discovered = _discover_actions(self._nodes("NavigateToPose"))
        assert "navigate_to_pose" in discovered

    def test_deduplicates(self):
        nodes = self._nodes("NavigateToPose", "NavigateToPose")
        discovered = _discover_actions(nodes)
        assert len([k for k in discovered if k == "navigate_to_pose"]) == 1

    def test_multiple_actions(self):
        nodes = self._nodes("NavigateToPose", "RowOperation")
        discovered = _discover_actions(nodes)
        assert "navigate_to_pose" in discovered
        assert "row_traversal" in discovered

    def test_empty_nodes(self):
        assert _discover_actions([]) == {}

    def test_unknown_action_uses_normalised_edge_type(self):
        """Unknown actions fall back to the edge-level action_type."""
        nodes = [{"node": {"edges": [
            {"action": "MyCustomAction", "action_type": "my_pkg/action/Custom"}
        ]}}]
        disc = _discover_actions(nodes)
        assert "my_custom_action" in disc
        assert disc["my_custom_action"] == "my_pkg.action.Custom"

    def test_missing_action_type_defaults(self):
        """Missing edge action_type defaults to NavigateToPose."""
        nodes = [{"node": {"edges": [{"action": "SomeUnknown"}]}}]
        disc = _discover_actions(nodes)
        assert disc["some_unknown"] == "nav2_msgs.action.NavigateToPose"


# ---------- _build_actions_section ------------------------------------

class TestBuildActionsSection:
    """Tests for building the top-level actions dict."""

    def test_structure(self):
        discovered = {"navigate_to_pose": "nav2_msgs.action.NavigateToPose"}
        actions = _build_actions_section(discovered, _BUILTIN_BT_DEFS)
        assert "navigate_to_pose" in actions
        entry = actions["navigate_to_pose"]
        assert "composable" in entry
        assert "action_type" in entry
        assert "action_server" in entry
        assert "action_goal_template" in entry

    def test_single_pose_action_template(self):
        """Single-pose actions get a 'pose' key, not 'poses'."""
        discovered = {"navigate_to_pose": "nav2_msgs.action.NavigateToPose"}
        section = _build_actions_section(discovered, _BUILTIN_BT_DEFS)
        tpl = section["navigate_to_pose"]["action_goal_template"]
        assert "pose" in tpl
        assert "poses" not in tpl

    def test_multi_pose_action_template(self):
        """Multi-pose actions get a 'poses' key."""
        discovered = {"row_traversal": "nav2_msgs.action.NavigateThroughPoses"}
        section = _build_actions_section(discovered, _BUILTIN_BT_DEFS)
        tpl = section["row_traversal"]["action_goal_template"]
        assert "poses" in tpl

    def test_bt_reference_added(self):
        """behavior_tree reference is added when definition exists."""
        discovered = {"navigate_to_pose": "nav2_msgs.action.NavigateToPose"}
        section = _build_actions_section(discovered, _BUILTIN_BT_DEFS)
        tpl = section["navigate_to_pose"]["action_goal_template"]
        assert "behavior_tree" in tpl


# ---------- _build_definitions ----------------------------------------

class TestBuildDefinitions:
    """Tests for building the definitions section."""

    def test_builtin_definitions_used(self):
        discovered = {"navigate_to_pose": "nav2_msgs.action.NavigateToPose"}
        defs = _build_definitions(discovered, None, _BUILTIN_BT_DEFS)
        assert "default_bt" in defs

    def test_row_traversal_bt_included(self):
        discovered = {"row_traversal": "nav2_msgs.action.NavigateThroughPoses"}
        defs = _build_definitions(discovered, None, _BUILTIN_BT_DEFS)
        assert "row_traversal_bt" in defs

    def test_empty_discovered(self):
        defs = _build_definitions({}, None, _BUILTIN_BT_DEFS)
        assert isinstance(defs, dict)


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
        for key in ("meta", "transformation", "definitions", "actions", "nodes"):
            assert key in result

    def test_transformation_converted(self, old_map):
        """Transformation block has topo_frame_id."""
        result = convert_tmap(old_map)
        assert "topo_frame_id" in result["transformation"]

    def test_actions_discovered(self, old_map):
        """Both navigate_to_pose and row_traversal should be discovered."""
        result = convert_tmap(old_map)
        assert "navigate_to_pose" in result["actions"]
        assert "row_traversal" in result["actions"]

    def test_definitions_present(self, old_map):
        """Definitions section should contain BT XML."""
        result = convert_tmap(old_map)
        assert len(result["definitions"]) > 0

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
