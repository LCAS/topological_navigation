#!/usr/bin/env python3
"""Interactive markers for graph nodes; click a node to navigate to it."""

from __future__ import annotations

import json

import rclpy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from topological_navigation_msgs.action import ExecuteNamedWaypoints
from topological_navigation_msgs.msg import ClosestNode
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


class InteractiveNodeMarkers(Node):
    def __init__(self) -> None:
        super().__init__("interactive_node_markers")

        self.declare_parameter("graph_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("named_route_action", "/execute_named_waypoints")
        self.declare_parameter("closest_node_topic", "/closest_node")
        self.declare_parameter("marker_scale", 0.4)

        self.graph_file = str(self.get_parameter("graph_file").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)

        self.node_positions = self._load_nodes(self.graph_file)
        self.current_node_name = ""
        self.last_target_name = ""
        self.goal_in_progress = False

        self.closest_sub = self.create_subscription(
            ClosestNode,
            str(self.get_parameter("closest_node_topic").value),
            self._closest_cb,
            10,
        )

        self.client = ActionClient(
            self,
            ExecuteNamedWaypoints,
            str(self.get_parameter("named_route_action").value),
        )

        self.server = InteractiveMarkerServer(self, "topological_nodes")
        self._create_markers()
        self.server.applyChanges()

        self.get_logger().info(
            f"Interactive node markers ready for {len(self.node_positions)} nodes"
        )

    def _load_nodes(self, graph_file: str) -> dict[str, tuple[float, float]]:
        if not graph_file:
            raise RuntimeError("graph_file parameter is required")

        with open(graph_file, "r", encoding="utf-8") as f:
            graph = json.load(f)

        out: dict[str, tuple[float, float]] = {}
        for feature in graph.get("features", []):
            if feature.get("geometry", {}).get("type") != "Point":
                continue
            coords = feature["geometry"].get("coordinates", [0.0, 0.0])
            props = feature.get("properties", {})
            name = str(props.get("name", props.get("id")))
            out[name] = (float(coords[0]), float(coords[1]))

        if not out:
            raise RuntimeError("No point nodes found in graph")
        return out

    def _closest_cb(self, msg: ClosestNode) -> None:
        self.current_node_name = msg.node_name

    def _create_markers(self) -> None:
        for name, (x, y) in self.node_positions.items():
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = self.frame_id
            int_marker.name = name
            int_marker.description = f"Node {name}"
            int_marker.scale = self.marker_scale
            int_marker.pose.position.x = x
            int_marker.pose.position.y = y
            int_marker.pose.orientation.w = 1.0

            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere.scale.x = self.marker_scale * 0.45
            sphere.scale.y = self.marker_scale * 0.45
            sphere.scale.z = self.marker_scale * 0.45
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

    def _marker_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return

        target = feedback.marker_name
        if target not in self.node_positions:
            return

        if self.goal_in_progress:
            self.get_logger().warn("Navigation in progress; ignoring marker click")
            return

        start = self.current_node_name or self.last_target_name
        if start not in self.node_positions:
            self.get_logger().warn(
                "No localized start node yet on /closest_node. Wait for localization before clicking markers."
            )
            return

        if start == target:
            self.get_logger().info(f"Already at node {target}")
            return

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("execute_named_waypoints action server unavailable")
            return

        goal = ExecuteNamedWaypoints.Goal()
        goal.waypoint_names = [start, target]
        goal.execute_navigation = True

        self.goal_in_progress = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response_cb(f, target))
        self.get_logger().info(f"Marker click navigation: {start} -> {target}")

    def _goal_response_cb(self, future, target: str) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_in_progress = False
            self.get_logger().warn("Marker navigation goal rejected")
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self._result_cb(f, target))

    def _result_cb(self, future, target: str) -> None:
        self.goal_in_progress = False
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
