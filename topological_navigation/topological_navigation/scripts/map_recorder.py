#!/usr/bin/env python3
"""Topological map recording node for ROS 2.

Drive the robot around and let this node build a topological map on the fly:

- A ``record`` ROS 2 action starts continuous recording; a new node is added
  automatically whenever the robot has moved ``node_distance`` metres from
  the last recorded node. Feedback is published as nodes are added and
  recording stops when the action goal is cancelled.
- The map being built is continuously published on the latched
  ``/recorded_tmap`` topic so it can be inspected, loaded, or handed off to a
  central coordinator while recording is still in progress.
- Loop closure is automatic: if the robot re-enters the influence zone of an
  already recorded node, that existing node is reused/linked instead of
  creating a duplicate.
- Nodes are connected successively (each new/linked node to the previously
  recorded one) with bidirectional edges.
- Recorded nodes carry a ``map.source: recording`` property.
- Services are provided to explicitly add the current pose as a node, delete
  the last added node, reset/clear the map, save the map to a file, and load
  a (partial) map from a file so that recording can continue from it.

Maintainer: Marc Hanheide <marc@hanheide.net>
"""

import os

import yaml

import rclpy
import rclpy.duration
import rclpy.executors
from rclpy import Parameter
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory

from topological_navigation_msgs.action import RecordMap
from topological_navigation_msgs.srv import LoadTmap, WriteTopologicalMap

from topological_navigation.map_recorder_utils import MapRecorderCore
from topological_navigation.tmap_utils import load_tmap2_file, save_tmap2_file


class MapRecorder(Node):
    """ROS 2 node implementing the topological map recording action/services."""

    def __init__(self):
        super().__init__('map_recorder')

        # -- ROS parameters ---------------------------------------------
        self.declare_parameter('node_distance', 1.0)
        self.declare_parameter('pointset', 'recorded_map')
        self.declare_parameter('site_name', 'map')
        self.declare_parameter('tmap_dir', '')
        self.declare_parameter('pose_topic', '/odometry/global')
        self.declare_parameter(
            'source_tmap_topic', Parameter.Type.STRING)

        self.node_distance = self.get_parameter('node_distance').value
        self.pointset = self.get_parameter('pointset').value
        self.site_name = self.get_parameter('site_name').value
        self.tmap_dir = self.get_parameter('tmap_dir').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.source_tmap_topic = self.get_parameter_or(
            'source_tmap_topic',
            Parameter('source_tmap_topic', Parameter.Type.STRING, '/topological_map_2'),
        ).value

        # -- Load templates from package config ---------------------------
        toponav_dir = get_package_share_directory('topological_navigation')
        config_dir = os.path.join(toponav_dir, 'config')
        self.template_node = self._load_yaml(
            os.path.join(config_dir, 'template_node.yaml'))
        self.template_edge = self._load_yaml(
            os.path.join(config_dir, 'template_edge.yaml'))
        self.template_action = self._load_yaml(
            os.path.join(config_dir, 'template_action.yaml'))

        # -- Core recording logic (no ROS dependency) ----------------------
        self.core = MapRecorderCore(
            self.template_node, self.template_edge, self.template_action,
            pointset=self.pointset, site_name=self.site_name,
        )

        # -- State ----------------------------------------------------------
        self._robot_pose = None  # dict with x, y, z, qx, qy, qz, qw
        self._recording = False
        self._cancel_requested = False
        self._source_tmap = None

        # -- QoS (latched, same pattern as map_manager2 / manual_topomapping)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # -- Publishers -------------------------------------------------------
        self.recorded_tmap_pub = self.create_publisher(
            String, '/recorded_tmap', latched_qos)

        # -- Subscribers --------------------------------------------------------
        self.create_subscription(
            Odometry, self.pose_topic, self._pose_cb, 10)
        self.create_subscription(
            String, self.source_tmap_topic, self._source_tmap_cb, latched_qos)

        # -- Services ----------------------------------------------------------
        self.create_service(Trigger, '~/add_node', self._add_node_srv_cb)
        self.create_service(
            Trigger, '~/delete_last_node', self._delete_last_node_srv_cb)
        self.create_service(Trigger, '~/reset_map', self._reset_map_srv_cb)
        self.create_service(
            WriteTopologicalMap, '~/save_map', self._save_map_srv_cb)
        self.create_service(LoadTmap, '~/load_map', self._load_map_srv_cb)

        # -- Action server --------------------------------------------------
        self._record_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, RecordMap, '/record',
            execute_callback=self._execute_record_cb,
            cancel_callback=self._cancel_record_cb,
            callback_group=self._record_cb_group,
        )

        self._publish_recorded_tmap()
        self.get_logger().info(
            "map_recorder ready: node_distance=%.2f, pointset='%s'"
            % (self.node_distance, self.pointset))

    # ==================================================================
    # YAML helpers
    # ==================================================================

    @staticmethod
    def _load_yaml(filename):
        with open(filename, 'r') as fh:
            return yaml.safe_load(fh)

    # ==================================================================
    # Subscriber callbacks
    # ==================================================================

    def _pose_cb(self, msg):
        pose = msg.pose.pose
        self._robot_pose = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
        }

    def _source_tmap_cb(self, msg):
        """Remember the currently active map so new recordings can inherit its meta data."""
        try:
            self._source_tmap = yaml.safe_load(msg.data)
        except yaml.YAMLError as exc:
            self.get_logger().warning(f"Failed to parse source tmap: {exc}")

    # ==================================================================
    # Action server: record
    # ==================================================================

    def _execute_record_cb(self, goal_handle):
        map_name = goal_handle.request.map_name or self.pointset
        node_distance = goal_handle.request.node_distance or self.node_distance

        self.core.pointset = map_name
        if not self.core.tmap.get("nodes"):
            self.core.reset(source_tmap=self._source_tmap)
            self.core.pointset = map_name

        self._recording = True
        self._cancel_requested = False

        self.get_logger().info(
            "[RECORD] Started: map_name='%s', node_distance=%.2f"
            % (map_name, node_distance))

        rate_period = 0.2  # seconds between distance checks
        while rclpy.ok() and not self._cancel_requested:
            if self._robot_pose is not None:
                _, created, message = self.core.add_node(
                    self._robot_pose, node_distance=node_distance)
                # A loop closure also mutates the map (a new edge is added
                # to link back to the reused node) even though `created` is
                # False, so republish in that case too.
                if created or message.startswith("loop closure"):
                    self._publish_recorded_tmap()
                    self._publish_record_feedback(goal_handle, message)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=rate_period))

        self._recording = False

        result = RecordMap.Result()
        result.num_nodes = self.core.num_nodes()
        if self._cancel_requested:
            goal_handle.canceled()
            result.success = True
            result.message = "Recording stopped (%d nodes recorded)" % result.num_nodes
        else:
            goal_handle.succeed()
            result.success = True
            result.message = "Recording finished (%d nodes recorded)" % result.num_nodes

        self.get_logger().info(f"[RECORD] {result.message}")
        return result

    def _cancel_record_cb(self, _goal_handle):
        self.get_logger().info("[RECORD] Cancel requested")
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def _publish_record_feedback(self, goal_handle, status):
        feedback = RecordMap.Feedback()
        feedback.num_nodes = self.core.num_nodes()
        feedback.last_node = self.core.last_node_name()
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    # ==================================================================
    # Services
    # ==================================================================

    def _add_node_srv_cb(self, request, response):
        """Service: explicitly add the current robot pose as a new node."""
        if self._robot_pose is None:
            response.success = False
            response.message = "No robot pose received yet"
            return response

        name, created, message = self.core.add_node(
            self._robot_pose, node_distance=self.node_distance, force=True)
        self._publish_recorded_tmap()
        response.success = True
        response.message = f"{message} ({name})" if created else message
        return response

    def _delete_last_node_srv_cb(self, request, response):
        """Service: remove the most-recently added node."""
        success, message = self.core.delete_last_node()
        if success:
            self._publish_recorded_tmap()
        response.success = success
        response.message = message
        return response

    def _reset_map_srv_cb(self, request, response):
        """Service: clear the recorded map completely."""
        self.core.reset(source_tmap=self._source_tmap)
        self._publish_recorded_tmap()
        response.success = True
        response.message = "Recorded map cleared"
        return response

    def _save_map_srv_cb(self, request, response):
        """Service: save the recorded map to a YAML file."""
        filename = request.filename
        if not filename:
            filename = os.path.join(
                self.tmap_dir or '.', f"{self.core.pointset}.tmap2.yaml")
        elif self.tmap_dir and not os.path.isabs(filename):
            filename = os.path.join(self.tmap_dir, filename)

        try:
            save_tmap2_file(self.core.tmap, filename, no_alias=request.no_alias)
        except OSError as exc:
            response.success = False
            response.message = f"Failed to save map: {exc}"
            return response

        response.success = True
        response.message = f"Saved recorded map to {filename}"
        self.get_logger().info(response.message)
        return response

    def _load_map_srv_cb(self, request, response):
        """Service: load a (partial) map from a file to continue recording from it."""
        filename = request.filename
        if self.tmap_dir and not os.path.isabs(filename):
            filename = os.path.join(self.tmap_dir, filename)

        try:
            tmap = load_tmap2_file(filename, logger=self.get_logger())
        except (OSError, ValueError, TypeError) as exc:
            response.success = False
            response.message = f"Failed to load map: {exc}"
            response.num_nodes = 0
            return response

        self.core.load(tmap)
        self._publish_recorded_tmap()
        response.success = True
        response.message = f"Loaded map from {filename}"
        response.num_nodes = self.core.num_nodes()
        return response

    # ==================================================================
    # Publish helpers
    # ==================================================================

    def _publish_recorded_tmap(self):
        msg = String()
        msg.data = self.core.to_yaml()
        self.recorded_tmap_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapRecorder()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
