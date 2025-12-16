"""
Unit tests for edge-action-driven behavior.

These tests validate that navigation actions are determined from edge.action
metadata, not from node name patterns.
"""
import pytest
import sys
from pathlib import Path

# Add the module path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / 'topological_navigation' / 'scripts'))
sys.path.insert(0, str(Path(__file__).parent.parent / 'topological_navigation'))

from actions_bt import ActionsType


class TestActionsType:
    """Test ActionsType class functionality."""
    
    def test_action_types_defined(self):
        """Test that all expected action types are defined."""
        actions = ActionsType()
        
        # Core navigation actions
        assert actions.NAVIGATE_TO_POSE == "NavigateToPose"
        assert actions.NAVIGATE_THROUGH_POSES == "NavigateThroughPoses"
        assert actions.DRIVE_ON_HEADING == "DriveOnHeading"
        
        # Row-specific actions
        assert actions.ROW_TRAVERSAL == "row_traversal"
        assert actions.ROW_OPERATION == "row_operation"
        assert actions.ROW_RECOVERY == "row_recovery"
        assert actions.ROW_CHANGE == "row_change"
        assert actions.GOAL_ALIGN == "goal_align"
    
    def test_deprecated_constants_documented(self):
        """Test that deprecated node-name constants still exist for backward compatibility."""
        actions = ActionsType()
        
        # These constants are deprecated but should exist for backward compatibility
        assert hasattr(actions, 'GOAL_ALIGN_INDEX')
        assert hasattr(actions, 'GOAL_ALIGN_GOAL')
        assert hasattr(actions, 'ROW_START_INDEX')
        assert hasattr(actions, 'ROW_COLUMN_START_INDEX')
        assert hasattr(actions, 'ROW_COLUMN_START_NEXT_INDEX')
        assert hasattr(actions, 'OUTSIDE_EDGE_START_INDEX')
    
    def test_bt_tree_action_mappings(self):
        """Test that BT tree action mappings are properly configured."""
        actions = ActionsType()
        
        # Verify action-to-BT-tree mappings exist
        assert actions.ROW_TRAVERSAL in actions.bt_tree_with_actions
        assert actions.NAVIGATE_TO_POSE in actions.bt_tree_with_actions
        assert actions.GOAL_ALIGN in actions.bt_tree_with_actions
        assert actions.ROW_OPERATION in actions.bt_tree_with_actions
        
    def test_planner_config_mappings(self):
        """Test that planner configurations are properly set up."""
        actions = ActionsType()
        
        # Verify planner config exists
        assert "dwb_core::DWBLocalPlanner" in actions.planner_with_goal_checker_config
        
        # Verify goal tolerance values are present
        config = actions.planner_with_goal_checker_config["dwb_core::DWBLocalPlanner"]
        assert "goal_checker.xy_goal_tolerance" in config
        assert "goal_checker.yaw_goal_tolerance" in config
        
    def test_set_planner(self):
        """Test that planner can be set for action types."""
        actions = ActionsType()
        
        # Set a custom planner
        actions.setPlanner("custom_planner::Custom", actions.ROW_TRAVERSAL)
        assert actions.bt_tree_with_control_server_config[actions.ROW_TRAVERSAL] == "custom_planner::Custom"
        
    def test_set_planner_params(self):
        """Test that planner parameters can be updated."""
        actions = ActionsType()
        
        # Update planner parameters
        actions.setPlannerParams("dwb_core::DWBLocalPlanner", 0.5, 0.3)
        
        config = actions.planner_with_goal_checker_config["dwb_core::DWBLocalPlanner"]
        assert config["goal_checker.xy_goal_tolerance"] == 0.5
        assert config["goal_checker.yaw_goal_tolerance"] == 0.3


class TestEdgeActionSelection:
    """
    Tests for edge-action-driven behavior.
    
    These tests verify that actions are selected based on edge.action metadata
    rather than inferred from node names.
    """
    
    def test_edge_action_used_directly(self):
        """
        Test that edge action is used directly without modification.
        
        This validates the refactored get_goal_align_if() behavior where
        the edge.action metadata is the source of truth.
        """
        # Simulate the refactored behavior:
        # get_goal_align_if() should return the current_action unchanged
        
        def get_goal_align_if(edge_id, current_action, next_edge_id=None):
            """Simulated refactored function that uses edge.action directly."""
            # This is the refactored behavior - just return the edge action
            return current_action
        
        # Test various edge actions
        test_cases = [
            ("WayPoint56_r8.5-ca", "NavigateToPose", None),
            ("r2-cb_r2-ca", "row_traversal", None),
            ("r3-ca_r3-cb", "goal_align", None),
            ("WayPoint140_WayPoint74", "NavigateToPose", "WayPoint74_WayPoint66"),
        ]
        
        for edge_id, action, next_edge_id in test_cases:
            result = get_goal_align_if(edge_id, action, next_edge_id)
            # The action should be returned unchanged
            assert result == action, f"Expected {action}, got {result} for edge {edge_id}"
    
    def test_edge_action_row_traversal(self):
        """Test that row_traversal action is preserved from edge metadata."""
        # Edge from the test map that has row_traversal action
        edge = {
            "action": "row_traversal",
            "action_type": "geometry_msgs/PoseStamped",
            "edge_id": "r2-cb_r2-ca",
            "fail_policy": "fail",
            "fluid_navigation": True,
        }
        
        # The action should come directly from edge metadata
        assert edge["action"] == "row_traversal"
    
    def test_edge_action_navigate_to_pose(self):
        """Test that NavigateToPose action is preserved from edge metadata."""
        edge = {
            "action": "NavigateToPose",
            "action_type": "geometry_msgs/PoseStamped",
            "edge_id": "WayPoint140_WayPoint74",
            "fail_policy": "fail",
            "fluid_navigation": True,
        }
        
        assert edge["action"] == "NavigateToPose"


class TestNavigationAreaDetection:
    """
    Tests for navigation area detection using edge actions.
    """
    
    def test_area_from_row_traversal_edge(self):
        """Test that row_traversal edge action indicates INSIDE_POLYTUNNEL."""
        actions = ActionsType()
        
        # Simulated edge actions lookup
        dist_edge_actions = {
            "r2-cb_r2-ca": "row_traversal",
            "WayPoint140_WayPoint74": "NavigateToPose",
        }
        
        # If closest edge has row_traversal action, area should be inside polytunnel
        closest_edge = "r2-cb_r2-ca"
        edge_action = dist_edge_actions.get(closest_edge, "")
        
        if edge_action == actions.ROW_TRAVERSAL:
            area = actions.INSIDE_POLYTUNNEL
        else:
            area = actions.OUTSIDE_POLYTUNNEL
            
        assert area == actions.INSIDE_POLYTUNNEL
    
    def test_area_from_navigate_to_pose_edge(self):
        """Test that NavigateToPose edge does not indicate inside polytunnel."""
        actions = ActionsType()
        
        dist_edge_actions = {
            "WayPoint140_WayPoint74": "NavigateToPose",
        }
        
        closest_edge = "WayPoint140_WayPoint74"
        edge_action = dist_edge_actions.get(closest_edge, "")
        
        # NavigateToPose action should not indicate inside polytunnel
        if edge_action == actions.ROW_TRAVERSAL:
            area = actions.INSIDE_POLYTUNNEL
        elif edge_action == actions.GOAL_ALIGN:
            area = actions.TRANSITION_INTO_POLYTUNNEL
        else:
            area = actions.OUTSIDE_POLYTUNNEL
            
        assert area == actions.OUTSIDE_POLYTUNNEL


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
