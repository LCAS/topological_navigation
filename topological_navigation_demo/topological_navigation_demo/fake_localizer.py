#!/usr/bin/env python3
"""Fake localizer for demo use.

Reads the robot's ground-truth pose from Gazebo ``/gazebo/model_states``,
computes the ``map → odom`` transform that makes Nav2's TF tree consistent
with reality, and broadcasts it at 20 Hz.  Also republishes the ground-truth
pose on ``/amcl_pose`` so downstream nodes (closest_node_publisher, etc.) that
expect an AMCL pose continue to work without modification.

Because the ground truth is used directly, the robot's estimated position in
Nav2 always matches the simulation — no particle-filter drift.
"""

from __future__ import annotations

import math

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


# ---------------------------------------------------------------------------
# 2-D SE(2) helpers
# ---------------------------------------------------------------------------

def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """Return (qx, qy, qz, qw) for a pure yaw rotation."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _se2_compose(
    x1: float, y1: float, t1: float,
    x2: float, y2: float, t2: float,
) -> tuple[float, float, float]:
    """Compose two 2-D rigid transforms: T1 followed by T2-in-T1's frame."""
    c, s = math.cos(t1), math.sin(t1)
    return x1 + c * x2 - s * y2, y1 + s * x2 + c * y2, t1 + t2


def _se2_inverse(x: float, y: float, t: float) -> tuple[float, float, float]:
    """Inverse of a 2-D rigid transform."""
    c, s = math.cos(t), math.sin(t)
    return -c * x - s * y, s * x - c * y, -t


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FakeLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("fake_localizer")

        self.declare_parameter("robot_model_name", "waffle")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("broadcast_rate_hz", 20.0)

        self._robot_name = str(self.get_parameter("robot_model_name").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        self._robot_pose: tuple[float, float, float] | None = None  # (x, y, yaw) in world/map

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            10,
        )

        self.create_subscription(
            ModelStates,
            str(self.get_parameter("model_states_topic").value),
            self._model_states_cb,
            10,
        )

        rate = float(self.get_parameter("broadcast_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate), self._broadcast)

        # Broadcast an identity map→odom immediately so the map frame exists
        # for Nav2 even before the first Gazebo model-states message arrives.
        self._emit_identity_tf()

        self.get_logger().info(
            f"Fake localizer ready — tracking '{self._robot_name}' "
            f"using Gazebo ground truth"
        )

    # ------------------------------------------------------------------

    def _emit_identity_tf(self) -> None:
        """Broadcast a zero map→odom TF so the map frame exists immediately."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._map_frame
        tf_msg.child_frame_id = self._odom_frame
        tf_msg.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_msg)

    def _model_states_cb(self, msg: ModelStates) -> None:
        try:
            idx = msg.name.index(self._robot_name)
        except ValueError:
            return

        p = msg.pose[idx]
        yaw = _quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        self._robot_pose = (float(p.position.x), float(p.position.y), yaw)

    def _broadcast(self) -> None:
        if self._robot_pose is None:
            # Keep the map frame alive until Gazebo reports the robot pose.
            self._emit_identity_tf()
            return

        # Lookup current odom → base_footprint from the running TF tree.
        try:
            t = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        gx, gy, gyaw = self._robot_pose
        ox = float(t.transform.translation.x)
        oy = float(t.transform.translation.y)
        oyaw = _quat_to_yaw(
            float(t.transform.rotation.x),
            float(t.transform.rotation.y),
            float(t.transform.rotation.z),
            float(t.transform.rotation.w),
        )

        # T_map_odom = T_map_base * inv(T_odom_base)
        inv_ox, inv_oy, inv_oyaw = _se2_inverse(ox, oy, oyaw)
        mo_x, mo_y, mo_yaw = _se2_compose(gx, gy, gyaw, inv_ox, inv_oy, inv_oyaw)

        now = self.get_clock().now().to_msg()

        # Broadcast map → odom
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self._map_frame
        tf_msg.child_frame_id = self._odom_frame
        tf_msg.transform.translation.x = mo_x
        tf_msg.transform.translation.y = mo_y
        tf_msg.transform.translation.z = 0.0
        qx, qy, qz, qw = _yaw_to_quat(mo_yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

        # Publish ground-truth /amcl_pose for downstream consumers
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self._map_frame
        pose_msg.pose.pose.position.x = gx
        pose_msg.pose.pose.position.y = gy
        pose_msg.pose.pose.position.z = 0.0
        _, _, mqz, mqw = _yaw_to_quat(gyaw)
        pose_msg.pose.pose.orientation.z = mqz
        pose_msg.pose.pose.orientation.w = mqw
        # Zero covariance — perfect ground-truth pose.
        self._amcl_pub.publish(pose_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
