#!/usr/bin/env python3
# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""Web app for visualising the topological map, the robot and its route.

Serves a React front end (all assets bundled, works offline) on a single
port (default 8083).  The backend holds all state; browsers only render
what is pushed to them via Server-Sent Events, so every open client shows
the same synchronised view.

Features
--------
* Topological map drawn as a zoomable / pannable graph (roboflow-enabled
  nodes are coloured differently); hovering a node shows its properties.
* Live robot position (current/closest node and metric pose from TF) and
  speed (from odometry).
* Click a node to send the robot there (``GotoNode`` action), current
  destination / goal display with a cancel button.
* The planned route is highlighted while navigating.

Parameters
----------
port : int (default 8083)
    HTTP port serving both the UI and the API.
host : str (default ``0.0.0.0``)
    Interface to bind to.
odom_topic : str (default ``/odometry/global``)
    ``nav_msgs/Odometry`` topic used for the speed display.
base_frame : str (default ``base_link``)
    Robot frame looked up in TF for the metric position.
map_frame : str (default ``''``)
    Fixed frame; empty uses the map's ``topo_frame_id`` (or ``map``).
action_name : str (default ``/topological_navigation``)
    ``GotoNode`` action server.
pose_rate : float (default 5.0)
    Rate [Hz] of the TF pose lookup.
update_rate : float (default 10.0)
    Maximum rate [Hz] at which state updates are pushed to clients.
web_root : str (default ``''``)
    Directory with the front end; empty uses the bundled assets.
"""

import json

import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from topological_navigation_msgs.action import GotoNode
from topological_navigation_msgs.msg import TopologicalRoute

from topological_navigation_visual.web_server import (
    DEFAULT_PORT, DEFAULT_WEB_ROOT, WebAppServer,
)
from topological_navigation_visual.web_state import (
    WebAppState, yaw_from_quaternion,
)

_OUTCOMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'cancelled',
}


def parse_map_message(data: str):
    """Parse the ``/topological_map_2`` payload (JSON, YAML fallback)."""
    try:
        return json.loads(data)
    except ValueError:
        return yaml.safe_load(data)


class TopologicalWebApp(Node):
    """ROS 2 node feeding :class:`WebAppState` and serving the web UI."""

    def __init__(self):
        super().__init__('topological_web_app')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', '')
        self.declare_parameter('action_name', '/topological_navigation')
        self.declare_parameter('pose_rate', 5.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('web_root', '')

        port = int(self.get_parameter('port').value)
        host = str(self.get_parameter('host').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        action_name = str(self.get_parameter('action_name').value)
        pose_rate = max(float(self.get_parameter('pose_rate').value), 0.1)
        update_rate = max(float(self.get_parameter('update_rate').value), 0.1)
        web_root = str(self.get_parameter('web_root').value) or DEFAULT_WEB_ROOT

        self.state = WebAppState()
        self._cb_group = ReentrantCallbackGroup()
        self._tf_warned = False

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # -- Subscriptions ---------------------------------------------
        self.create_subscription(
            String, '/topological_map_2', self._map_cb, latched,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/current_node', self._current_node_cb, latched,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/closest_node', self._closest_node_cb, latched,
            callback_group=self._cb_group)
        self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data,
            callback_group=self._cb_group)
        self.create_subscription(
            TopologicalRoute, '/topological_navigation/Route',
            self._route_cb, best_effort, callback_group=self._cb_group)
        self.create_subscription(
            String, '/current_edge', self._current_edge_cb, latched,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/topological_navigation/move_action_status',
            self._move_status_cb, latched, callback_group=self._cb_group)
        self.create_subscription(
            String, '/robot_operation_current_status',
            self._nav_status_cb, latched, callback_group=self._cb_group)
        self.create_subscription(
            GoalStatusArray, action_name.rstrip('/') + '/_action/status',
            self._action_status_cb, latched, callback_group=self._cb_group)

        # -- Action client + cancel-all service ------------------------
        self._goto_client = ActionClient(
            self, GotoNode, action_name, callback_group=self._cb_group)
        self._cancel_client = self.create_client(
            CancelGoal, action_name.rstrip('/') + '/_action/cancel_goal',
            callback_group=self._cb_group)

        # -- TF pose lookup --------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(1.0 / pose_rate, self._pose_timer_cb,
                          callback_group=self._cb_group)

        # -- HTTP server -----------------------------------------------
        self.server = WebAppServer(
            self.state, on_goto=self.send_goal, on_cancel=self.cancel_goal,
            host=host, port=port, web_root=web_root,
            min_interval=1.0 / update_rate, logger=self.get_logger(),
        )
        self.server.start()
        self.get_logger().info(
            'Topological web app serving %s on http://%s:%d/'
            % (web_root, host, self.server.port))

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------
    def _map_cb(self, msg):
        try:
            tmap = parse_map_message(msg.data)
            if self.state.set_map(tmap):
                self.get_logger().info('Topological map updated')
        except Exception as exc:
            self.get_logger().error('Failed to parse topological map: %s' % exc)

    def _current_node_cb(self, msg):
        self.state.set_current_node(msg.data)

    def _closest_node_cb(self, msg):
        self.state.set_closest_node(msg.data)

    def _odom_cb(self, msg):
        twist = msg.twist.twist
        self.state.set_odometry(
            twist.linear.x, twist.linear.y, twist.angular.z)

    def _route_cb(self, msg):
        self.state.set_route(msg.nodes)

    def _current_edge_cb(self, msg):
        self.state.set_current_edge(msg.data)

    def _move_status_cb(self, msg):
        try:
            self.state.set_move_status(json.loads(msg.data))
        except ValueError:
            pass

    def _nav_status_cb(self, msg):
        self.state.set_nav_status(msg.data)

    def _action_status_cb(self, msg):
        self.state.set_action_statuses(s.status for s in msg.status_list)

    def _pose_timer_cb(self):
        frame = self._map_frame or self.state.map_frame or 'map'
        try:
            trans = self._tf_buffer.lookup_transform(
                frame, self._base_frame, rclpy.time.Time())
        except TransformException as exc:
            if not self._tf_warned:
                self.get_logger().warning(
                    'TF lookup %s -> %s failed: %s'
                    % (frame, self._base_frame, exc))
                self._tf_warned = True
            self.state.clear_pose()
            return
        self._tf_warned = False
        t = trans.transform.translation
        r = trans.transform.rotation
        self.state.set_pose(
            t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w), frame)

    # ------------------------------------------------------------------
    # Commands from the web UI (called from HTTP server threads)
    # ------------------------------------------------------------------
    def send_goal(self, target):
        """Send a ``GotoNode`` goal; returns ``(ok, message)``."""
        if not self._goto_client.wait_for_server(timeout_sec=1.0):
            return False, 'navigation action server not available'
        goal = GotoNode.Goal()
        goal.target = target
        goal.no_orientation = False
        self.get_logger().info('Web app: sending robot to %s' % target)
        self.state.goal_requested(target)
        future = self._goto_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._goal_response_cb(f, target))
        return True, 'goal to %s sent' % target

    def _goal_response_cb(self, future, target):
        try:
            handle = future.result()
        except Exception as exc:
            self.state.goal_rejected(target, 'failed: %s' % exc)
            return
        if handle is None or not handle.accepted:
            self.state.goal_rejected(target, 'rejected')
            return
        self.state.goal_accepted(target)
        handle.get_result_async().add_done_callback(
            lambda f: self._goal_result_cb(f, target))

    def _goal_result_cb(self, future, target):
        outcome = 'finished'
        try:
            result = future.result()
            outcome = _OUTCOMES.get(result.status, 'finished')
            if result.status == GoalStatus.STATUS_SUCCEEDED \
                    and not result.result.success:
                outcome = 'failed'
        except Exception as exc:
            outcome = 'failed: %s' % exc
        self.get_logger().info('Web app: goal to %s %s' % (target, outcome))
        self.state.goal_finished(target, outcome)

    def cancel_goal(self):
        """Cancel all goals on the navigation action server."""
        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            return False, 'navigation cancel service not available'
        # An empty request (zero goal id and stamp) cancels all goals, so
        # goals sent by other clients can be cancelled from the UI as well.
        self._cancel_client.call_async(CancelGoal.Request())
        self.get_logger().info('Web app: cancelling navigation')
        self.state.set_message('Cancel requested')
        return True, 'cancel requested'

    def destroy_node(self):
        self.server.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TopologicalWebApp()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
