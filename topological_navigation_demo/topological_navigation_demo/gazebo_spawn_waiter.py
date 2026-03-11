#!/usr/bin/env python3
"""Spawn TurtleBot in Gazebo after /spawn_entity becomes available."""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node


class GazeboSpawnWaiter(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_spawn_waiter")

        self.declare_parameter("entity_name", "waffle")
        self.declare_parameter("sdf_file", "")
        self.declare_parameter("x", -2.0)
        self.declare_parameter("y", -0.5)
        self.declare_parameter("z", 0.01)
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("retry_period_sec", 2.0)

        self.entity_name = str(self.get_parameter("entity_name").value)
        self.sdf_file = str(self.get_parameter("sdf_file").value)
        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.retry_period_sec = float(self.get_parameter("retry_period_sec").value)

        self.pose = Pose()
        self.pose.position.x = float(self.get_parameter("x").value)
        self.pose.position.y = float(self.get_parameter("y").value)
        self.pose.position.z = float(self.get_parameter("z").value)
        self.pose.orientation.w = 1.0

        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        self.timer = self.create_timer(self.retry_period_sec, self._attempt_spawn)
        self.request_in_flight = False

        self.get_logger().info(f"Spawn waiter active for entity '{self.entity_name}'")

    def _resolve_sdf_path(self) -> Path:
        if self.sdf_file:
            return Path(self.sdf_file)

        model = os.environ.get("TURTLEBOT3_MODEL", "waffle")
        return Path(
            f"/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_{model}/model.sdf"
        )

    def _attempt_spawn(self) -> None:
        if self.request_in_flight:
            return

        if not self.client.wait_for_service(timeout_sec=0.1):
            return

        sdf_path = self._resolve_sdf_path()
        if not sdf_path.exists():
            self.get_logger().error(f"SDF file does not exist: {sdf_path}")
            return

        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.robot_namespace = ""
        req.reference_frame = self.reference_frame
        req.initial_pose = self.pose
        req.xml = sdf_path.read_text(encoding="utf-8")

        self.request_in_flight = True
        future = self.client.call_async(req)
        future.add_done_callback(self._spawn_done)

    def _spawn_done(self, future) -> None:
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"Spawn request failed, will retry: {ex}")
            return

        if response is not None and response.success:
            self.get_logger().info(f"Robot spawned successfully: {response.status_message}")
            self.timer.cancel()
            return

        message = response.status_message if response is not None else "no response"
        if "already exists" in message.lower():
            self.get_logger().info("Robot already exists in Gazebo")
            self.timer.cancel()
            return

        self.get_logger().warn(f"Spawn returned failure, retrying: {message}")


def main() -> None:
    rclpy.init()
    node = GazeboSpawnWaiter()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
