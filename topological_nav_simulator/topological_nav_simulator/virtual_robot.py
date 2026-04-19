# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""Virtual robot that tracks simulated pose, publishes TF and RViz markers."""

import math

import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def _euler_from_quaternion(q):
    """Extract yaw from quaternion (z-axis rotation)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    """Create quaternion from yaw angle."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _lerp(a, b, t):
    """Linear interpolation between a and b."""
    return a + (b - a) * t


def _slerp_yaw(yaw_a, yaw_b, t):
    """Spherical linear interpolation for yaw angles."""
    diff = yaw_b - yaw_a
    # Normalize to [-pi, pi]
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return yaw_a + diff * t


class VirtualRobot:
    """Manages virtual robot state, TF broadcasting, and RViz marker publishing."""

    def __init__(self, node, map_frame, base_frame, odom_frame):
        self._node = node
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._odom_frame = odom_frame

        # Current pose
        self.pose = Pose()
        self.pose.orientation.w = 1.0

        # TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(node)

        # Publishers
        self._marker_pub = node.create_publisher(
            MarkerArray, '/virtual_robot/markers', 10
        )
        self._odom_pub = node.create_publisher(
            Odometry, '/odometry/global', 10
        )
        self._pose_pub = node.create_publisher(
            PoseStamped, '/virtual_robot/pose', 10
        )

        # Trail of past positions for visualization
        self._trail_points = []
        self._max_trail_points = 500

        node.get_logger().info(
            f'Virtual robot initialized: {map_frame} -> {odom_frame} -> {base_frame}'
        )

    def set_pose(self, pose):
        """Set the robot pose directly."""
        self.pose = pose

    def publish_all(self):
        """Publish TF, odometry, markers, and pose."""
        now = self._node.get_clock().now().to_msg()
        self._publish_tf(now)
        self._publish_odometry(now)
        self._publish_pose(now)
        self._publish_markers(now)

    def _publish_tf(self, stamp):
        """Broadcast map->odom (identity) and odom->base_link transforms."""
        # map -> odom (identity)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = stamp
        t_map_odom.header.frame_id = self._map_frame
        t_map_odom.child_frame_id = self._odom_frame
        t_map_odom.transform.rotation.w = 1.0

        # odom -> base_link (robot pose)
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = self._odom_frame
        t_odom_base.child_frame_id = self._base_frame
        t_odom_base.transform.translation.x = self.pose.position.x
        t_odom_base.transform.translation.y = self.pose.position.y
        t_odom_base.transform.translation.z = self.pose.position.z
        t_odom_base.transform.rotation = self.pose.orientation

        self._tf_broadcaster.sendTransform([t_map_odom, t_odom_base])

    def _publish_odometry(self, stamp):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose = self.pose
        self._odom_pub.publish(odom)

    def _publish_pose(self, stamp):
        """Publish pose stamped."""
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self._map_frame
        ps.pose = self.pose
        self._pose_pub.publish(ps)

    def _publish_markers(self, stamp):
        """Publish RViz markers for the virtual robot."""
        markers = MarkerArray()
        header = Header(stamp=stamp, frame_id=self._map_frame)

        # --- Robot body: a cube ---
        body = Marker()
        body.header = header
        body.ns = 'virtual_robot'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose = self.pose
        body.scale = Vector3(x=0.5, y=0.3, z=0.3)
        body.color = ColorRGBA(r=0.1, g=0.7, b=1.0, a=0.9)
        body.lifetime.sec = 0
        markers.markers.append(body)

        # --- Direction arrow ---
        arrow = Marker()
        arrow.header = header
        arrow.ns = 'virtual_robot'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = self.pose
        arrow.scale = Vector3(x=0.6, y=0.12, z=0.12)
        arrow.color = ColorRGBA(r=1.0, g=0.3, b=0.1, a=0.9)
        arrow.lifetime.sec = 0
        markers.markers.append(arrow)

        # --- Label text ---
        label = Marker()
        label.header = header
        label.ns = 'virtual_robot'
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = Pose()
        label.pose.position.x = self.pose.position.x
        label.pose.position.y = self.pose.position.y
        label.pose.position.z = self.pose.position.z + 0.5
        label.pose.orientation.w = 1.0
        label.scale = Vector3(x=0.0, y=0.0, z=0.2)
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = '\U0001F916 Sim Robot'
        label.lifetime.sec = 0
        markers.markers.append(label)

        # --- Footprint ring (circle of small spheres) ---
        footprint = Marker()
        footprint.header = header
        footprint.ns = 'virtual_robot'
        footprint.id = 3
        footprint.type = Marker.LINE_STRIP
        footprint.action = Marker.ADD
        footprint.pose = self.pose
        footprint.scale = Vector3(x=0.03, y=0.0, z=0.0)
        footprint.color = ColorRGBA(r=0.1, g=1.0, b=0.3, a=0.6)
        num_pts = 32
        radius = 0.35
        for i in range(num_pts + 1):
            angle = 2.0 * math.pi * i / num_pts
            pt = Point(
                x=radius * math.cos(angle),
                y=radius * math.sin(angle),
                z=0.0,
            )
            footprint.points.append(pt)
        footprint.lifetime.sec = 0
        markers.markers.append(footprint)

        # --- Trail (breadcrumb path) ---
        self._trail_points.append(Point(
            x=self.pose.position.x,
            y=self.pose.position.y,
            z=0.02,
        ))
        if len(self._trail_points) > self._max_trail_points:
            self._trail_points = self._trail_points[-self._max_trail_points:]

        if len(self._trail_points) > 1:
            trail = Marker()
            trail.header = header
            trail.ns = 'virtual_robot'
            trail.id = 4
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.pose.orientation.w = 1.0
            trail.scale = Vector3(x=0.04, y=0.0, z=0.0)
            trail.color = ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.4)
            trail.points = list(self._trail_points)
            trail.lifetime.sec = 0
            markers.markers.append(trail)

        self._marker_pub.publish(markers)

    def clear_trail(self):
        """Clear the breadcrumb trail."""
        self._trail_points.clear()
