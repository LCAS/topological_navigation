#!/usr/bin/env python3
"""Publish an initial pose a few times to bootstrap AMCL localization."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", -2.0)
        self.declare_parameter("y", -0.5)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_count", 8)
        self.declare_parameter("publish_interval_sec", 1.5)

        topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.publish_count = int(self.get_parameter("publish_count").value)
        period = float(self.get_parameter("publish_interval_sec").value)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.sent = 0
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Publishing initial pose on {topic} at ({self.x:.2f}, {self.y:.2f}) for {self.publish_count} cycles"
        )

    def _on_timer(self) -> None:
        if self.sent >= self.publish_count:
            self.timer.cancel()
            self.get_logger().info("Initial pose publishing complete")
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        half_yaw = self.yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # Keep covariance modest so AMCL accepts this as a strong prior.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher.publish(msg)
        self.sent += 1


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
