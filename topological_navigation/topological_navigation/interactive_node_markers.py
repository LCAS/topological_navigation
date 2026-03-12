#!/usr/bin/env python3
"""Interactive node markers that preview and execute topological routes on click."""

from __future__ import annotations

import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav2_msgs.action import ComputeRoute
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from topological_navigation_msgs.action import ExecuteNamedWaypoints
from topological_navigation_msgs.msg import ClosestNode
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)


class InteractiveNodeMarkers(Node):
    def __init__(self) -> None:
        super().__init__("interactive_node_markers")

        self.declare_parameter("graph_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("named_route_action", "/execute_named_waypoints")
        self.declare_parameter("route_action_name", "/compute_route")
        self.declare_parameter("closest_node_topic", "/closest_node")
        self.declare_parameter("interactive_markers_namespace", "topological_nodes")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("pose_cov_topic", "/amcl_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("selected_route_topic", "/selected_topological_route")
        self.declare_parameter("closest_node_max_age_sec", 1.0)
        self.declare_parameter("marker_scale", 0.2)
        self.declare_parameter("marker_z_offset", 0.05)
        self.declare_parameter("click_sphere_scale_factor", 0.65)
        self.declare_parameter("node_capture_radius", 0.15)

        self.graph_file = str(self.get_parameter("graph_file").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.interactive_markers_namespace = str(
            self.get_parameter("interactive_markers_namespace").value
        )
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.marker_z_offset = float(self.get_parameter("marker_z_offset").value)
        self.click_sphere_scale_factor = float(
            self.get_parameter("click_sphere_scale_factor").value
        )

        self.node_positions, self.node_name_to_id = self._load_nodes(self.graph_file)
        self.current_node_name = ""
        self.current_node_stamp_ns = 0
        self.last_pose_xy: tuple[float, float] | None = None
        self.last_target_name = ""
        self.goal_in_progress = False
        self.preview_request_in_flight = False
        self._current_goal_handle = None
        self._nav_generation = 0

        self.closest_sub = self.create_subscription(
            ClosestNode,
            str(self.get_parameter("closest_node_topic").value),
            self._closest_cb,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._pose_cb,
            10,
        )
        self.pose_cov_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_cov_topic").value),
            self._pose_cov_cb,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_cb,
            10,
        )

        self.exec_client = ActionClient(
            self,
            ExecuteNamedWaypoints,
            str(self.get_parameter("named_route_action").value),
        )
        self.compute_route_client = ActionClient(
            self,
            ComputeRoute,
            str(self.get_parameter("route_action_name").value),
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.selected_route_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("selected_route_topic").value),
            marker_qos,
        )

        self.server = InteractiveMarkerServer(self, self.interactive_markers_namespace)
        self._create_markers()
        self.server.applyChanges()

        self.get_logger().info(
            f"Interactive node markers ready for {len(self.node_positions)} nodes"
        )

    def _load_nodes(self, graph_file: str) -> tuple[dict[str, tuple[float, float]], dict[str, int]]:
        if not graph_file:
            raise RuntimeError("graph_file parameter is required")

        with open(graph_file, "r", encoding="utf-8") as f:
            graph = json.load(f)

        positions: dict[str, tuple[float, float]] = {}
        name_to_id: dict[str, int] = {}
        for feature in graph.get("features", []):
            if feature.get("geometry", {}).get("type") != "Point":
                continue
            coords = feature["geometry"].get("coordinates", [0.0, 0.0])
            props = feature.get("properties", {})
            name = str(props.get("name", props.get("id")))
            positions[name] = (float(coords[0]), float(coords[1]))
            if "id" in props:
                name_to_id[name] = int(props["id"])

        if not positions:
            raise RuntimeError("No point nodes found in graph")
        return positions, name_to_id

    def _closest_cb(self, msg: ClosestNode) -> None:
        self.current_node_name = msg.node_name
        self.current_node_stamp_ns = self.get_clock().now().nanoseconds

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.last_pose_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _pose_cov_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_pose_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_pose_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _nearest_node_from_pose(self) -> str:
        if self.last_pose_xy is None:
            return ""

        x, y = self.last_pose_xy
        best_name = ""
        best_dist_sq = float("inf")
        for name, (nx, ny) in self.node_positions.items():
            dist_sq = (x - nx) ** 2 + (y - ny) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_name = name
        return best_name

    def _select_start_node(self) -> str:
        """Return the nearest graph node to the robot's current pose."""
        return self._nearest_node_from_pose()

    def _create_markers(self) -> None:
        for name, (x, y) in self.node_positions.items():
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = self.frame_id
            int_marker.name = name
            int_marker.description = f"Node {name}"
            int_marker.scale = self.marker_scale
            int_marker.pose.position.x = x
            int_marker.pose.position.y = y
            int_marker.pose.position.z = self.marker_z_offset
            int_marker.pose.orientation.w = 1.0

            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere_scale = self.marker_scale * self.click_sphere_scale_factor
            sphere.scale.x = sphere_scale
            sphere.scale.y = sphere_scale
            sphere.scale.z = sphere_scale
            sphere.color = ColorRGBA(r=0.15, g=0.85, b=0.2, a=0.95)

            label = Marker()
            label.type = Marker.TEXT_VIEW_FACING
            label.text = name
            label.scale.z = self.marker_scale * 0.35
            label.pose.position.z = self.marker_scale * 0.55
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

            ctrl = InteractiveMarkerControl()
            ctrl.always_visible = True
            ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
            ctrl.markers.append(sphere)
            ctrl.markers.append(label)
            int_marker.controls.append(ctrl)

            self.server.insert(int_marker)
            self.server.setCallback(int_marker.name, self._marker_feedback)

    def _publish_route_preview(self, start_name: str, target_name: str) -> None:
        start_id = self.node_name_to_id.get(start_name)
        target_id = self.node_name_to_id.get(target_name)
        if start_id is None or target_id is None:
            return

        if self.preview_request_in_flight:
            return

        if not self.compute_route_client.server_is_ready() and not self.compute_route_client.wait_for_server(
            timeout_sec=0.0
        ):
            self.get_logger().debug("ComputeRoute action server unavailable for route preview")
            return

        goal = ComputeRoute.Goal()
        goal.use_poses = False
        goal.use_start = True
        goal.start_id = start_id
        goal.goal_id = target_id

        self.preview_request_in_flight = True
        goal_future = self.compute_route_client.send_goal_async(goal)
        goal_future.add_done_callback(self._preview_goal_response_cb)

    def _preview_goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.preview_request_in_flight = False
            self.get_logger().warn(f"ComputeRoute preview request failed: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.preview_request_in_flight = False
            self.get_logger().debug("ComputeRoute preview goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._preview_result_cb)

    def _preview_result_cb(self, future) -> None:
        self.preview_request_in_flight = False
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"ComputeRoute preview result failed: {exc}")
            return

        if wrapped_result is None or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug("ComputeRoute preview did not succeed")
            return

        result = wrapped_result.result
        if not hasattr(result, "route") or len(result.route.nodes) < 2:
            self.get_logger().debug("ComputeRoute preview returned no valid route")
            return

        route = result.route

        stamp = self.get_clock().now().to_msg()

        route_marker = Marker()
        route_marker.header.frame_id = self.frame_id
        route_marker.header.stamp = stamp
        route_marker.ns = "selected_route"
        route_marker.id = 0
        route_marker.type = Marker.LINE_STRIP
        route_marker.action = Marker.ADD
        route_marker.scale.x = 0.05
        route_marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.95)
        route_marker.lifetime = Duration(seconds=0.0).to_msg()

        for node in route.nodes:
            p = Point()
            p.x = float(node.position.x)
            p.y = float(node.position.y)
            p.z = 0.04
            route_marker.points.append(p)

        endpoints = Marker()
        endpoints.header.frame_id = self.frame_id
        endpoints.header.stamp = stamp
        endpoints.ns = "selected_route_endpoints"
        endpoints.id = 1
        endpoints.type = Marker.SPHERE_LIST
        endpoints.action = Marker.ADD
        endpoints.scale.x = 0.10
        endpoints.scale.y = 0.10
        endpoints.scale.z = 0.10
        endpoints.color = ColorRGBA(r=1.0, g=0.95, b=0.2, a=0.95)
        endpoints.lifetime = Duration(seconds=0.0).to_msg()

        if route_marker.points:
            endpoints.points.append(route_marker.points[0])
            endpoints.points.append(route_marker.points[-1])

        msg = MarkerArray()
        msg.markers = [route_marker, endpoints]
        self.selected_route_pub.publish(msg)

    def _marker_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return

        target = feedback.marker_name
        if target not in self.node_positions:
            self.get_logger().warn(
                f"Clicked marker '{target}' not found in graph nodes; known examples: {list(self.node_positions.keys())[:5]}"
            )
            return

        if self.goal_in_progress:
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None
            self.goal_in_progress = False
            self.get_logger().info(f"Cancelling current navigation to re-route to {target}")

        # Prefer a fresh /closest_node update; fallback to nearest node from pose.
        start = self._select_start_node()
        if not start:
            self.get_logger().warn("No robot pose available yet; wait for localization.")
            return

        if start == target:
            self.get_logger().info(f"Already at node {target}")
            return

        self._nav_generation += 1
        gen = self._nav_generation
        self.goal_in_progress = True
        self.get_logger().info(f"Marker click: {start} -> {target}")
        self._send_navigation_goal(start, target, gen)

    def _send_navigation_goal(self, start: str, target: str, gen: int) -> None:
        """Send the ExecuteNamedWaypoints goal after start/target have been resolved."""
        if gen != self._nav_generation:
            self.goal_in_progress = False
            return
        if not self.exec_client.server_is_ready() and not self.exec_client.wait_for_server(
            timeout_sec=0.2
        ):
            self.goal_in_progress = False
            self.get_logger().warn("execute_named_waypoints action server unavailable")
            return
        self._publish_route_preview(start, target)
        goal = ExecuteNamedWaypoints.Goal()
        goal.waypoint_names = [start, target]
        goal.execute_navigation = True
        future = self.exec_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response_cb(f, target, gen))
        self.get_logger().info(f"Navigating: {start} -> {target}")

    def _goal_response_cb(self, future, target: str, generation: int) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            if generation == self._nav_generation:
                self.goal_in_progress = False
            self.get_logger().warn("Marker navigation goal rejected")
            return

        if generation != self._nav_generation:
            # A newer click arrived while this goal was being sent — cancel it
            handle.cancel_goal_async()
            return

        self._current_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self._result_cb(f, target, generation))

    def _result_cb(self, future, target: str, generation: int) -> None:
        if generation != self._nav_generation:
            return  # Stale result from a cancelled/superseded goal
        self.goal_in_progress = False
        self._current_goal_handle = None
        wrapped = future.result()
        if wrapped is None:
            self.get_logger().warn("Marker navigation returned no result")
            return

        result = wrapped.result
        if getattr(result, "success", False):
            self.last_target_name = target
            self.get_logger().info(f"Marker navigation complete: {result.message}")
        else:
            self.get_logger().warn(f"Marker navigation failed: {getattr(result, 'message', '')}")


def main() -> None:
    rclpy.init()
    node = InteractiveNodeMarkers()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
