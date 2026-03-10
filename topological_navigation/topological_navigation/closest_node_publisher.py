#!/usr/bin/env python3
"""Publish the closest topological node name and distance from live robot pose."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from topological_navigation_msgs.msg import ClosestNode


class ClosestNodePublisher(Node):
    def __init__(self) -> None:
        super().__init__("closest_node_publisher")

        self.declare_parameter("graph_file", "")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("pose_cov_topic", "/amcl_pose")
        self.declare_parameter("closest_node_topic", "/closest_node")

        self.graph_file = str(self.get_parameter("graph_file").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.pose_cov_topic = str(self.get_parameter("pose_cov_topic").value)
        self.closest_node_topic = str(self.get_parameter("closest_node_topic").value)

        self.nodes = self._load_nodes(self.graph_file)
        if not self.nodes:
            self.get_logger().warn("No graph nodes loaded; closest-node output will be empty")

        self.pub = self.create_publisher(ClosestNode, self.closest_node_topic, 10)
        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)
        self.sub_cov = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_cov_topic,
            self._pose_cov_cb,
            10,
        )

        self.get_logger().info(
            f"Listening on {self.pose_topic} and {self.pose_cov_topic}, publishing closest nodes on {self.closest_node_topic}"
        )

    def _load_nodes(self, graph_file: str):
        if not graph_file:
            self.get_logger().warn("graph_file parameter is empty")
            return []

        with open(graph_file, "r", encoding="utf-8") as f:
            graph = json.load(f)

        nodes = []
        for feature in graph.get("features", []):
            geometry = feature.get("geometry", {})
            if geometry.get("type") != "Point":
                continue

            props = feature.get("properties", {})
            coords = geometry.get("coordinates", [0.0, 0.0])
            name = str(props.get("name", props.get("id", "unknown")))
            nodes.append((name, float(coords[0]), float(coords[1])))

        self.get_logger().info(f"Loaded {len(nodes)} topological nodes from graph")
        return nodes

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._publish_closest(msg.pose.position.x, msg.pose.position.y)

    def _pose_cov_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._publish_closest(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _publish_closest(self, x_raw: float, y_raw: float) -> None:
        if not self.nodes:
            return

        x = float(x_raw)
        y = float(y_raw)

        best_name = ""
        best_dist = float("inf")

        for name, nx, ny in self.nodes:
            dist = math.hypot(x - nx, y - ny)
            if dist < best_dist:
                best_dist = dist
                best_name = name

        out = ClosestNode()
        out.node_name = best_name
        out.distance = float(best_dist)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ClosestNodePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
