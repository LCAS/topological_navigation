#!/usr/bin/env python3
"""Periodically send random topological-node navigation goals."""

from __future__ import annotations

import json
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from topological_navigation_msgs.action import ExecuteNamedWaypoints
from topological_navigation_msgs.msg import ClosestNode


class RandomTargetNavigator(Node):
    def __init__(self) -> None:
        super().__init__("random_target_navigator")

        self.declare_parameter("graph_file", "")
        self.declare_parameter("named_route_action", "/execute_named_waypoints")
        self.declare_parameter("closest_node_topic", "/closest_node")
        self.declare_parameter("period_sec", 30.0)
        self.declare_parameter("execute_navigation", True)
        self.declare_parameter("random_seed", 42)

        graph_file = str(self.get_parameter("graph_file").value)
        action_name = str(self.get_parameter("named_route_action").value)
        closest_topic = str(self.get_parameter("closest_node_topic").value)
        period_sec = float(self.get_parameter("period_sec").value)
        self.execute_navigation = bool(self.get_parameter("execute_navigation").value)
        seed = int(self.get_parameter("random_seed").value)

        self.rng = random.Random(seed)
        self.nodes = self._load_waypoint_names(graph_file)
        if len(self.nodes) < 2:
            raise RuntimeError("random_target_navigator requires at least 2 graph nodes")

        self.closest_node_name = ""
        self.last_target = ""
        self.goal_in_progress = False

        self.closest_sub = self.create_subscription(
            ClosestNode,
            closest_topic,
            self._closest_node_cb,
            10,
        )
        self.client = ActionClient(self, ExecuteNamedWaypoints, action_name)
        self.timer = self.create_timer(period_sec, self._dispatch_random_goal)

        self.get_logger().info(
            f"Random navigator active with {len(self.nodes)} nodes, dispatch period={period_sec:.1f}s"
        )

    def _load_waypoint_names(self, graph_file: str) -> list[str]:
        if not graph_file:
            raise RuntimeError("graph_file parameter is required")

        with open(graph_file, "r", encoding="utf-8") as f:
            graph = json.load(f)

        names: list[str] = []
        for feature in graph.get("features", []):
            if feature.get("geometry", {}).get("type") != "Point":
                continue
            props = feature.get("properties", {})
            names.append(str(props.get("name", props.get("id"))))

        return names

    def _closest_node_cb(self, msg: ClosestNode) -> None:
        self.closest_node_name = msg.node_name

    def _pick_next_target(self, start_node: str) -> str:
        candidates = [n for n in self.nodes if n != start_node]
        return self.rng.choice(candidates)

    def _dispatch_random_goal(self) -> None:
        if self.goal_in_progress:
            return

        if not self.client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Waiting for execute_named_waypoints action server")
            return

        start = self.closest_node_name or self.last_target or self.nodes[0]
        if start not in self.nodes:
            start = self.nodes[0]

        target = self._pick_next_target(start)

        goal = ExecuteNamedWaypoints.Goal()
        goal.waypoint_names = [start, target]
        goal.execute_navigation = self.execute_navigation

        self.goal_in_progress = True
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(
            lambda future: self._goal_response_cb(future, start=start, target=target)
        )

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Random goal feedback: {feedback.status} ({feedback.current_from} -> {feedback.current_to})"
        )

    def _goal_response_cb(self, future, start: str, target: str) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.goal_in_progress = False
            self.get_logger().warn(f"Random goal rejected: {start} -> {target}")
            return

        self.get_logger().info(f"Random goal accepted: {start} -> {target}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._result_cb(f, target))

    def _result_cb(self, future, target: str) -> None:
        self.goal_in_progress = False
        wrapped = future.result()
        if wrapped is None:
            self.get_logger().warn("Random goal result was empty")
            return

        result = wrapped.result
        if result.success:
            self.last_target = target
            self.get_logger().info(f"Random goal completed: {result.message}")
        else:
            self.get_logger().warn(f"Random goal failed: {result.message}")


def main() -> None:
    rclpy.init()
    node = RandomTargetNavigator()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
