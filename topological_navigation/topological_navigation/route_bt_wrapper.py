#!/usr/bin/env python3
"""Action-driven wrapper over nav2_route with BT-aware segment execution."""

from __future__ import annotations

import json
import os
import random

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Goals
from nav2_msgs.action import ComputeRoute, NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from topological_navigation_msgs.action import ExecuteNamedWaypoints


class RouteBtWrapper(Node):
    def __init__(self) -> None:
        super().__init__("route_bt_wrapper")

        self.declare_parameter("route_action_name", "/compute_route")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("navigate_through_poses_action", "/navigate_through_poses")
        self.declare_parameter("named_route_action", "/execute_named_waypoints")
        self.declare_parameter("route_frame", "map")
        self.declare_parameter("graph_file", "")
        self.declare_parameter("behavior_map_file", "")
        self.declare_parameter("default_behavior_tree", "")
        self.declare_parameter("randomize_edge_behaviors", True)
        self.declare_parameter("edge_behavior_seed", 42)
        self.declare_parameter("controller_selector_topic", "/controller_selector")
        self.declare_parameter("default_controller_id", "FollowPath")
        self.declare_parameter("slow_controller_id", "SlowFollowPath")
        self.declare_parameter("reverse_controller_id", "ReverseFollowPath")
        self.declare_parameter("slow_behavior_tree", "")
        self.declare_parameter("reverse_behavior_tree", "")

        self.route_frame = str(self.get_parameter("route_frame").value)
        self.graph_file = str(self.get_parameter("graph_file").value)

        behavior_map_file = str(self.get_parameter("behavior_map_file").value)
        default_bt = str(self.get_parameter("default_behavior_tree").value)
        self.default_bt, self.edge_bts = self._load_behavior_map(behavior_map_file, default_bt)
        self.edge_controller_overrides: dict[int, str] = {}
        self.graph_edge_ids, self.edge_pairs = self._load_edge_info(self.graph_file)
        self._assign_random_edge_behaviors()
        self.name_to_node_id = self._load_name_to_id(self.graph_file)

        self.controller_selector_pub = self.create_publisher(
            String,
            str(self.get_parameter("controller_selector_topic").value),
            10,
        )

        self.compute_route_client = ActionClient(
            self,
            ComputeRoute,
            str(self.get_parameter("route_action_name").value),
        )
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
        )
        self.nav_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            str(self.get_parameter("navigate_through_poses_action").value),
        )

        self.named_route_action_server = ActionServer(
            self,
            ExecuteNamedWaypoints,
            str(self.get_parameter("named_route_action").value),
            execute_callback=self._execute_named_waypoints,
        )

        self.get_logger().info(
            "Ready for named waypoint goals on "
            + str(self.get_parameter("named_route_action").value)
        )

    def _load_behavior_map(self, file_path: str, default_bt: str) -> tuple[str, dict[int, str]]:
        edge_bts: dict[int, str] = {}
        if not file_path:
            return default_bt, edge_bts

        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        map_default = os.path.expandvars(str(data.get("default_behavior_tree", "")))
        resolved_default = default_bt or map_default

        for entry in data.get("edge_behaviors", []):
            edge_id = int(entry["edge_id"])
            bt_file = os.path.expandvars(str(entry["bt_file"]))
            edge_bts[edge_id] = bt_file

        return resolved_default, edge_bts

    def _load_name_to_id(self, graph_file: str) -> dict[str, int]:
        if not graph_file:
            self.get_logger().warn("graph_file is empty; waypoint-name lookup disabled")
            return {}

        with open(graph_file, "r", encoding="utf-8") as f:
            graph = json.load(f)

        lookup: dict[str, int] = {}
        for feature in graph.get("features", []):
            geometry = feature.get("geometry", {})
            if geometry.get("type") != "Point":
                continue

            props = feature.get("properties", {})
            node_id = int(props["id"])
            waypoint_name = str(props.get("name", str(node_id)))
            lookup[waypoint_name] = node_id

        self.get_logger().info(f"Loaded {len(lookup)} waypoint names from graph")
        return lookup

    def _load_edge_info(self, graph_file: str) -> tuple[list[int], dict[int, tuple[int, int]]]:
        if not graph_file:
            return [], {}

        with open(graph_file, "r", encoding="utf-8") as f:
            graph = json.load(f)

        edge_ids: list[int] = []
        edge_pairs: dict[int, tuple[int, int]] = {}
        for feature in graph.get("features", []):
            geometry = feature.get("geometry", {})
            geometry_type = geometry.get("type")
            if geometry_type not in ("LineString", "MultiLineString"):
                continue

            props = feature.get("properties", {})
            if "id" not in props:
                continue
            edge_id = int(props["id"])
            edge_ids.append(edge_id)

            if "startid" in props and "endid" in props:
                a = int(props["startid"])
                b = int(props["endid"])
                edge_pairs[edge_id] = (min(a, b), max(a, b))
            else:
                edge_pairs[edge_id] = (edge_id, edge_id)

        edge_ids = sorted(set(edge_ids))
        self.get_logger().info(f"Loaded {len(edge_ids)} route edges from graph")
        return edge_ids, edge_pairs

    def _assign_random_edge_behaviors(self) -> None:
        if not bool(self.get_parameter("randomize_edge_behaviors").value):
            return

        pair_to_edges: dict[tuple[int, int], list[int]] = {}
        for edge_id in self.graph_edge_ids:
            pair = self.edge_pairs.get(edge_id, (edge_id, edge_id))
            pair_to_edges.setdefault(pair, []).append(edge_id)

        pair_keys = list(pair_to_edges.keys())
        if len(pair_keys) < 3:
            self.get_logger().warn("Not enough node pairs to assign random 1/3 behavior groups")
            return

        slow_controller = str(self.get_parameter("slow_controller_id").value)
        reverse_controller = str(self.get_parameter("reverse_controller_id").value)
        slow_bt = str(self.get_parameter("slow_behavior_tree").value)
        reverse_bt = str(self.get_parameter("reverse_behavior_tree").value)

        seed = int(self.get_parameter("edge_behavior_seed").value)
        rng = random.Random(seed)
        rng.shuffle(pair_keys)

        group_size = len(pair_keys) // 3
        slow_pairs = pair_keys[:group_size]
        reverse_pairs = pair_keys[group_size : 2 * group_size]

        slow_edges: list[int] = []
        reverse_edges: list[int] = []

        for pair in slow_pairs:
            for edge_id in pair_to_edges[pair]:
                self.edge_controller_overrides[edge_id] = slow_controller
                if slow_bt:
                    self.edge_bts[edge_id] = slow_bt
                slow_edges.append(edge_id)

        for pair in reverse_pairs:
            for edge_id in pair_to_edges[pair]:
                self.edge_controller_overrides[edge_id] = reverse_controller
                if reverse_bt:
                    self.edge_bts[edge_id] = reverse_bt
                reverse_edges.append(edge_id)

        self.get_logger().info(
            "Assigned random pair behaviors: "
            + f"{len(slow_pairs)} slow pairs ({len(slow_edges)} edges), "
            + f"{len(reverse_pairs)} reverse pairs ({len(reverse_edges)} edges), "
            + f"{len(pair_keys) - len(slow_pairs) - len(reverse_pairs)} default pairs"
        )

    def _resolve_waypoint(self, name: str) -> int:
        if name in self.name_to_node_id:
            return self.name_to_node_id[name]

        # Fallback to numeric ID if provided as a string.
        try:
            return int(name)
        except ValueError as ex:
            raise KeyError(name) from ex

    def _publish_feedback(
        self,
        goal_handle,
        status: str,
        current_from: str = "",
        current_to: str = "",
    ) -> None:
        feedback = ExecuteNamedWaypoints.Feedback()
        feedback.status = status
        feedback.current_from = current_from
        feedback.current_to = current_to
        goal_handle.publish_feedback(feedback)

    def _compute_route_between_ids(self, start_id: int, goal_id: int):
        if not self.compute_route_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("ComputeRoute action server unavailable")

        goal = ComputeRoute.Goal()
        goal.use_poses = False
        goal.use_start = False
        goal.start_id = start_id
        goal.goal_id = goal_id

        goal_future = self.compute_route_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ComputeRoute goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("ComputeRoute returned no result")

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"ComputeRoute did not succeed, status={wrapped_result.status}")

        result = wrapped_result.result

        # Jazzy+ exposes error_code/error_msg; Humble does not.
        if hasattr(result, "error_code") and result.error_code != 0:
            raise RuntimeError(
                f"ComputeRoute failed: code={result.error_code}, msg={result.error_msg}"
            )

        if not hasattr(result, "route") or len(result.route.nodes) < 2:
            raise RuntimeError("ComputeRoute returned an empty or invalid route")

        return result.route

    def _build_segments(self, route_msg):
        if len(route_msg.edges) == 0 or len(route_msg.nodes) < 2:
            return []

        segments = []
        current = None

        for i, edge in enumerate(route_msg.edges):
            edge_id = int(edge.edgeid)
            bt = self.edge_bts.get(edge_id, self.default_bt)
            controller_id = self.edge_controller_overrides.get(
                edge_id,
                str(self.get_parameter("default_controller_id").value),
            )
            pose = self._pose_from_node(route_msg.nodes[i + 1])

            if (
                current is None
                or current["behavior_tree"] != bt
                or current["controller_id"] != controller_id
            ):
                current = {
                    "behavior_tree": bt,
                    "controller_id": controller_id,
                    "edge_ids": [edge_id],
                    "poses": [pose],
                }
                segments.append(current)
            else:
                current["edge_ids"].append(edge_id)
                current["poses"].append(pose)

        return segments

    def _merge_consecutive_segments(self, segments):
        if not segments:
            return []

        merged = [segments[0]]
        for seg in segments[1:]:
            last = merged[-1]
            if (
                last["behavior_tree"] == seg["behavior_tree"]
                and last["controller_id"] == seg["controller_id"]
            ):
                last["edge_ids"].extend(seg["edge_ids"])
                last["poses"].extend(seg["poses"])
            else:
                merged.append(seg)
        return merged

    def _select_controller(self, controller_id: str) -> None:
        msg = String()
        msg.data = controller_id
        self.controller_selector_pub.publish(msg)

    def _pose_from_node(self, node_msg) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.route_frame
        pose.pose.position.x = float(node_msg.position.x)
        pose.pose.position.y = float(node_msg.position.y)
        pose.pose.position.z = float(node_msg.position.z)
        pose.pose.orientation.w = 1.0
        return pose

    def _execute_segments(self, segments, goal_handle=None) -> bool:
        for seg in segments:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False

            poses = seg["poses"]
            bt = seg["behavior_tree"]
            controller_id = seg["controller_id"]

            # Select controller for this segment in BT navigator.
            self._select_controller(controller_id)

            ok = self._execute_segment_goal(poses, bt)
            if ok:
                continue

            default_controller = str(self.get_parameter("default_controller_id").value)
            if controller_id != default_controller or bt != self.default_bt:
                self.get_logger().warn(
                    "Segment failed with edge-specific behavior; retrying with default controller/BT"
                )
                self._select_controller(default_controller)
                if self._execute_segment_goal(poses, self.default_bt):
                    continue

            if len(poses) > 1:
                self.get_logger().warn(
                    "Merged segment still failed; retrying as per-pose NavigateToPose goals"
                )
                self._select_controller(default_controller)
                if self._execute_segment_as_single_pose_goals(poses, self.default_bt):
                    continue

            return False

        return True

    def _execute_segment_goal(self, poses, bt: str) -> bool:
        if len(poses) > 1:
            if not self.nav_through_poses_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("NavigateThroughPoses action server unavailable")
                return False

            goal = NavigateThroughPoses.Goal()

            # Humble uses PoseStamped[] while newer distros may use nav_msgs/Goals.
            assigned = False
            try:
                goal.poses = poses
                assigned = True
            except (AssertionError, TypeError):
                assigned = False

            if not assigned:
                goals_msg = Goals()
                if hasattr(goals_msg, "goals"):
                    goals_msg.goals = poses
                elif hasattr(goals_msg, "poses"):
                    goals_msg.poses = poses
                if hasattr(goals_msg, "header"):
                    goals_msg.header.frame_id = self.route_frame
                goal.poses = goals_msg

            # Through-poses goals in this setup use default through-poses BT.
            # Edge behavior is applied via controller selection, not BT override.
            future = self.nav_through_poses_client.send_goal_async(goal)
        else:
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("NavigateToPose action server unavailable")
                return False

            goal = NavigateToPose.Goal()
            goal.pose = poses[0]
            if bt:
                goal.behavior_tree = bt
            future = self.nav_to_pose_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Navigation segment goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("Navigation segment returned no result")
            return False

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Navigation segment failed with status={wrapped_result.status}"
            )
            return False

        result = wrapped_result.result
        if hasattr(result, "error_code") and result.error_code != 0:
            self.get_logger().error(
                f"Navigation segment failed: code={result.error_code}, msg={result.error_msg}"
            )
            return False

        return True

    def _execute_segment_as_single_pose_goals(self, poses, bt: str) -> bool:
        for pose in poses:
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("NavigateToPose action server unavailable")
                return False

            goal = NavigateToPose.Goal()
            goal.pose = pose
            if bt:
                goal.behavior_tree = bt

            future = self.nav_to_pose_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error("Single-pose retry goal rejected")
                return False

            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            wrapped_result = result_future.result()
            if wrapped_result is None:
                self.get_logger().error("Single-pose retry returned no result")
                return False

            if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"Single-pose retry failed with status={wrapped_result.status}"
                )
                return False

            result = wrapped_result.result
            if hasattr(result, "error_code") and result.error_code != 0:
                self.get_logger().error(
                    "Single-pose retry failed: "
                    + f"code={result.error_code}, msg={result.error_msg}"
                )
                return False

        return True

    def _execute_named_waypoints(self, goal_handle):
        waypoint_names = [str(n) for n in goal_handle.request.waypoint_names]
        do_execute = bool(goal_handle.request.execute_navigation)

        result = ExecuteNamedWaypoints.Result()

        if len(waypoint_names) < 2:
            result.success = False
            result.message = "At least 2 waypoint names are required"
            goal_handle.abort()
            return result

        try:
            node_ids = [self._resolve_waypoint(name) for name in waypoint_names]
        except KeyError as ex:
            result.success = False
            result.message = f"Unknown waypoint name: {ex.args[0]}"
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "planning", waypoint_names[0], waypoint_names[-1])

        all_segments = []
        try:
            for i in range(len(node_ids) - 1):
                from_name = waypoint_names[i]
                to_name = waypoint_names[i + 1]
                self._publish_feedback(goal_handle, "planning_leg", from_name, to_name)

                route_msg = self._compute_route_between_ids(node_ids[i], node_ids[i + 1])
                leg_segments = self._build_segments(route_msg)
                all_segments.extend(leg_segments)
        except Exception as ex:  # noqa: BLE001
            result.success = False
            result.message = str(ex)
            goal_handle.abort()
            return result

        merged_segments = self._merge_consecutive_segments(all_segments)
        self.get_logger().info(
            "Planned "
            + str(len(merged_segments))
            + " merged segment(s) across "
            + str(len(waypoint_names))
            + " named waypoints"
        )

        if not merged_segments:
            result.success = False
            result.message = "No executable segments were generated"
            goal_handle.abort()
            return result

        if do_execute:
            self._publish_feedback(goal_handle, "executing")
            ok = self._execute_segments(merged_segments, goal_handle)
            if not ok:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Goal canceled"
                    return result
                result.success = False
                result.message = "Execution failed"
                goal_handle.abort()
                return result
            result.success = True
            result.message = "Route planned and executed successfully"
        else:
            result.success = True
            result.message = (
                "Route planned successfully with "
                + str(len(merged_segments))
                + " merged segment(s)"
            )

        goal_handle.succeed()
        return result


def main() -> None:
    rclpy.init()
    node = RouteBtWrapper()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
