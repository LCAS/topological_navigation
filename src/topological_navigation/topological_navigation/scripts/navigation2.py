#!/usr/bin/env python
"""Map-Driven Topological Navigation Server.

All navigation behaviour is defined by the topological map YAML file:

- **definitions**: inline BT XML blobs, written to temp files for Nav2.
- **actions**: maps edge action names to Nav2 action types, servers,
  goal templates, and composability flags.
- **edges**: reference action names in the ``actions`` section.

The map file is the single source of truth.  No BT file paths or
action-type mappings are hard-coded in this script.

Architecture
~~~~~~~~~~~~
1.  Map arrives on ``/topological_map_2`` -> parsed into a NetworkX
    ``DiGraph`` and map-level ``definitions`` / ``actions`` dicts.
2.  For every unique ``(action_type, action_server)`` pair an
    ``ActionClient`` is created.
3.  Route planning uses NetworkX shortest-path algorithms whose
    parameters (algorithm, weight attribute) are exposed as ROS 2
    parameters so they can be changed at launch time.
4.  Consecutive composable edges are merged into multi-waypoint
    segments; non-composable edges are dispatched individually.
5.  Boundary polygons are published for any segment whose edges
    carry ``boundary_left`` / ``boundary_right`` properties.

ROS 2 interfaces
    Action servers:
        /<node_name>                                     (GotoNode)
        /topological_navigation/execute_policy_mode      (ExecutePolicyMode)
    Subscriptions:
        /topological_map_2, closest_node, closest_edges, current_node
    Publishers:
        topological_navigation/Statistics, topological_navigation/Route,
        current_edge, /boundary_checker (PolygonStamped),
        /robot_operation_current_status (String),
        topological_navigation/move_action_status (String)
    Parameters:
        max_dist_to_closest_edge  (double)  -- origin heuristic
        default_boundary_left     (double)  -- row corridor left
        default_boundary_right    (double)  -- row corridor right
        route_algorithm           (string)  -- 'astar' | 'dijkstra'
        route_weight_attr         (string)  -- edge attribute for cost

Last Updated: 2026-02-25
"""

import importlib
import json
import math
import os
import tempfile
import time
import threading
from datetime import datetime

import yaml

import rclpy
import rclpy.node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy import Parameter
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from topological_navigation.navigation_graph import (
    ACTION_TO_STATE,
    NavState,
    NavStateMachine,
    compute_boundary_polygon,
    get_route_distance,
    get_route_edges,
    merge_action_segments,
    plan_route,
)
from topological_navigation.networkx_utils import build_graph_from_tmap
from topological_navigation.tmap_utils import (
    get_edge_from_id_tmap2,
    get_node_from_tmap2,
)
from topological_navigation_msgs.action import ExecutePolicyMode, GotoNode
from topological_navigation_msgs.msg import (
    ClosestEdges,
    ExecutePolicyModeFeedback,
    GotoNodeFeedback,
    NavStatistics,
    TopologicalRoute,
)
# =====================================================================
# GoalStatus helpers
# =====================================================================

_STATUS_MAP = {
    GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",
    GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",
    GoalStatus.STATUS_ABORTED: "STATUS_ABORTED",
}


def _status_str(code: int) -> str:
    return _STATUS_MAP.get(code, "STATUS_UNKNOWN(%d)" % code)


# =====================================================================
# YAML loader -- coerce pose ints to float
# =====================================================================

class _FloatSafeLoader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        m = super().construct_mapping(node, deep=deep)
        for k in ('x', 'y', 'z', 'w'):
            if k in m and isinstance(m[k], int):
                m[k] = float(m[k])
        return m


# =====================================================================
# Navigation statistics
# =====================================================================

class nav_stats:
    """Timing statistics for a single edge traversal."""

    def __init__(self, origin, target, topol_map, edge_id):
        self.status = "active"
        self.origin = origin
        self.target = target
        self.topological_map = topol_map
        self.edge_id = edge_id
        self.set_start()

    def set_start(self):
        """Record navigation start time."""
        self.date_started = datetime.now()
        self.date_at_node = self.date_started

    def set_ended(self, node):
        """Record navigation end time and compute durations."""
        self.final_node = node
        self.date_finished = datetime.now()
        self.get_operation_time()
        self.get_time_to_wp()

    def set_at_node(self):
        """Record time of arrival at intermediate node."""
        self.date_at_node = datetime.now()

    def get_operation_time(self):
        """Total seconds from start to finish."""
        delta = self.date_finished - self.date_started
        self.operation_time = delta.total_seconds()
        return self.operation_time

    def get_time_to_wp(self):
        """Seconds from last intermediate node to finish."""
        if self.date_at_node != self.date_started:
            delta = self.date_finished - self.date_at_node
            self.time_to_wp = delta.total_seconds()
        else:
            self.time_to_wp = 0
        return self.time_to_wp

    def get_start_time_str(self):
        """Human-readable start timestamp."""
        return self.date_started.strftime(
            '%A, %B %d %Y, at %H:%M:%S hours',
        )

    def get_finish_time_str(self):
        """Human-readable finish timestamp."""
        return self.date_finished.strftime(
            '%A, %B %d %Y, at %H:%M:%S hours',
        )


# =====================================================================
# Topological Navigation Server
# =====================================================================

class TopologicalNavServer(rclpy.node.Node):
    """Map-driven topological navigation server.

    All action clients, BT selection, and composability rules are
    derived from the ``definitions`` and ``actions`` sections of the
    topological map YAML. NetworkX path-planning parameters are
    exposed as ROS 2 parameters.
    """

    # -----------------------------------------------------------------
    # Construction
    # -----------------------------------------------------------------

    def __init__(self, name):
        super().__init__(name)
        rclpy.get_default_context().on_shutdown(self._on_shutdown)

        # -- State machine -------------------------------------------
        self._sm = NavStateMachine(logger=self.get_logger())
        self._sm.transition(NavState.WAITING_FOR_MAP)

        # -- Runtime flags -------------------------------------------
        self._cancelled = False
        self._preempted = False
        self._goal_reached = False
        self._navigation_activated = False
        self._no_orientation = False
        self._target = "none"
        self._current_target = "none"

        # -- Map data ------------------------------------------------
        self._tmap = None
        self._graph = None
        self._topol_map = ""

        # -- Deferred map update (thread-safe buffering) -------------
        self._pending_map_msg = None
        self._map_lock = threading.RLock()
        self._map_updated_during_nav = False

        # -- Map-driven config (populated by _load_map_config) -------
        self._map_definitions = {}
        self._map_actions = {}
        self._bt_files = {}
        self._action_clients = {}    # name -> {client, action_class, config}

        # -- Localisation --------------------------------------------
        self._current_node = "Unknown"
        self._closest_node = "Unknown"
        self._closest_edges = ClosestEdges()

        # -- Nav2 infra ----------------------------------------------
        self._nav2_cb_group = MutuallyExclusiveCallbackGroup()
        self._goal_handle = None
        self._action_status = GoalStatus.STATUS_UNKNOWN

        # -- Stats ---------------------------------------------------
        self._stat = None

        # -- Parameters ----------------------------------------------
        self._declare_parameters()
        self._load_parameters()

        # -- QoS -----------------------------------------------------
        self._latch = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        self._best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -- Publishers ----------------------------------------------
        self._stats_pub = self.create_publisher(
            NavStatistics, "topological_navigation/Statistics",
            qos_profile=self._latch,
        )
        self._route_pub = self.create_publisher(
            TopologicalRoute, "topological_navigation/Route",
            qos_profile=self._best,
        )
        self._route_timer = self.create_timer(2.0, self._route_pub_cb)
        self._stroute = None
        self._cur_edge_pub = self.create_publisher(
            String, "current_edge", qos_profile=self._latch,
        )
        self._move_status_pub = self.create_publisher(
            String, "topological_navigation/move_action_status",
            qos_profile=self._latch,
        )
        self._boundary_pub = self.create_publisher(
            PolygonStamped, "/boundary_checker",
            qos_profile=self._latch,
        )
        self._status_pub = self.create_publisher(
            String, "/robot_operation_current_status",
            qos_profile=self._latch,
        )

        # -- Map subscription (blocking wait) ------------------------
        self._map_received = False
        self._loc_received = False
        self._cb_map = ReentrantCallbackGroup()
        self.create_subscription(
            String, '/topological_map_2', self._map_cb,
            callback_group=self._cb_map, qos_profile=self._latch,
        )

        self.get_logger().info(
            "[INIT] Waiting for topological map on /topological_map_2 ...",
        )
        self._publish_status("WAITING_FOR_MAP")
        while rclpy.ok() and not self._map_received:
            rclpy.spin_once(self)

        # Parse map-driven config (definitions, actions, clients)
        self._load_map_config()

        self.get_logger().info(
            "[INIT] Map '%s' -- %d nodes, %d edges"
            % (
                self._topol_map,
                self._graph.number_of_nodes(),
                self._graph.number_of_edges(),
            ),
        )

        # -- Localisation subscription (blocking wait) ---------------
        self._sm.transition(NavState.WAITING_FOR_LOCALISATION)
        self._publish_status("WAITING_FOR_LOCALISATION")
        self.create_subscription(
            String, 'closest_node', self._closest_node_cb,
            qos_profile=self._latch,
        )
        self.get_logger().info("[INIT] Waiting for localisation ...")
        while rclpy.ok() and not self._loc_received:
            rclpy.spin_once(self)
        self.get_logger().info(
            "[INIT] Localisation OK. Closest: %s" % self._closest_node,
        )

        self.create_subscription(
            ClosestEdges, 'closest_edges', self._closest_edges_cb,
            qos_profile=self._latch,
        )
        self.create_subscription(
            String, 'current_node', self._current_node_cb,
            qos_profile=self._latch,
        )

        # -- Action servers ------------------------------------------
        self._sm.transition(NavState.READY)
        self._publish_status("READY")

        cb_goto = ReentrantCallbackGroup()
        cb_policy = ReentrantCallbackGroup()

        self._as_goto = ActionServer(
            self, GotoNode, "/" + name,
            execute_callback=self._exec_goto_cb,
            cancel_callback=self._cancel_goto_cb,
            callback_group=cb_goto,
        )
        self._goto_fb_pub = self.create_publisher(
            GotoNodeFeedback, "/" + name + "/feedback",
            qos_profile=self._latch,
        )

        self._as_policy = ActionServer(
            self, ExecutePolicyMode,
            "/topological_navigation/execute_policy_mode",
            execute_callback=self._exec_policy_cb,
            cancel_callback=self._cancel_policy_cb,
            callback_group=cb_policy,
        )
        self._policy_fb_pub = self.create_publisher(
            ExecutePolicyModeFeedback,
            "topological_navigation/execute_policy_mode/feedback",
            qos_profile=self._latch,
        )

        # -- Goal checker service client ---------------------------
        gc_node = self._goal_checker_node
        self._set_params_client = self.create_client(
            SetParameters,
            '/%s/set_parameters' % gc_node,
            callback_group=ReentrantCallbackGroup(),
        )
        self._last_xy_tol = None
        self._last_yaw_tol = None

        self.get_logger().info(
            "[INIT] Navigation server READY ('%s', algo=%s, weight=%s)"
            % (self._topol_map, self._route_algorithm, self._route_weight),
        )

    # =================================================================
    # Parameters
    # =================================================================

    def _declare_parameters(self):
        """Declare all ROS 2 parameters."""
        for name, ptype in [
            ('max_dist_to_closest_edge', Parameter.Type.DOUBLE),
            ('default_boundary_left', Parameter.Type.DOUBLE),
            ('default_boundary_right', Parameter.Type.DOUBLE),
            # NetworkX path optimisation
            ('route_algorithm', Parameter.Type.STRING),
            ('route_weight_attr', Parameter.Type.STRING),
            # Nav2 goal checker
            ('goal_checker_node', Parameter.Type.STRING),
            ('xy_tolerance_param', Parameter.Type.STRING),
            ('yaw_tolerance_param', Parameter.Type.STRING),
        ]:
            self.declare_parameter(name, ptype)

    def _load_parameters(self):
        """Read parameter values with sensible defaults."""

        def _p(name, ptype, default):
            return self.get_parameter_or(
                name, Parameter('_', ptype, default),
            ).value

        self._max_dist_to_closest_edge = _p(
            'max_dist_to_closest_edge', Parameter.Type.DOUBLE, 1.0,
        )
        self._default_boundary_left = _p(
            'default_boundary_left', Parameter.Type.DOUBLE, 0.5,
        )
        self._default_boundary_right = _p(
            'default_boundary_right', Parameter.Type.DOUBLE, 0.5,
        )
        # 'astar' (with Euclidean heuristic) or 'dijkstra'
        self._route_algorithm = _p(
            'route_algorithm', Parameter.Type.STRING, 'astar',
        )
        # The edge attribute used as the cost for path planning
        self._route_weight = _p(
            'route_weight_attr', Parameter.Type.STRING, 'weight',
        )
        # Nav2 goal checker node and parameter names
        self._goal_checker_node = _p(
            'goal_checker_node', Parameter.Type.STRING,
            'controller_server',
        )
        self._xy_tolerance_param = _p(
            'xy_tolerance_param', Parameter.Type.STRING,
            'goal_checker.xy_goal_tolerance',
        )
        self._yaw_tolerance_param = _p(
            'yaw_tolerance_param', Parameter.Type.STRING,
            'goal_checker.yaw_goal_tolerance',
        )

    # =================================================================
    # Map-driven configuration
    # =================================================================

    def _load_action_type(self, type_str):
        # Example: nav2_msgs.action.NavigateThroughPoses
        module_name, class_name = type_str.rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, class_name)
    

    def _load_map_config(self):
        """Extract ``definitions`` and ``actions`` from the topomap.

        1. Writes each BT definition to a temp file.
        2. Resolves ``${definitions.<name>}`` template refs to paths.
        3. Creates ``ActionClient`` instances for unique servers.
        """
        self._map_definitions = self._tmap.get('definitions', {})
        self._map_actions = self._tmap.get('actions', {})

        if not self._map_actions:
            self.get_logger().warning(
                "[MAP] No 'actions' section -- behaviour undefined",
            )
            return

        # -- Write BT definitions to temp files ----------------------
        bt_dir = os.path.join(tempfile.gettempdir(), 'topo_nav_bt')
        os.makedirs(bt_dir, exist_ok=True)

        self._bt_files = {}
        for name, content in self._map_definitions.items():
            path = os.path.join(bt_dir, '%s.xml' % name)
            with open(path, 'w') as fh:
                fh.write(content)
            self._bt_files[name] = path
            self.get_logger().info(
                "[MAP] BT '%s' -> %s" % (name, path),
            )

        # -- Resolve ${definitions.<key>} refs -----------------------
        for act_name, act_cfg in self._map_actions.items():
            tpl = act_cfg.get('action_goal_template', {})
            bt_ref = tpl.get('behavior_tree', '')
            if (
                isinstance(bt_ref, str)
                and bt_ref.startswith('${definitions.')
                and bt_ref.endswith('}')
            ):
                key = bt_ref[len('${definitions.'):-1]
                if key in self._bt_files:
                    tpl['behavior_tree'] = self._bt_files[key]
                else:
                    self.get_logger().warning(
                        "[MAP] '%s' ref not found for action '%s'"
                        % (key, act_name),
                    )

            self.get_logger().info(
                "[MAP] Action '%s': type=%s server=%s composable=%s"
                % (
                    act_name,
                    act_cfg.get('action_type', '?'),
                    act_cfg.get('action_server', '?'),
                    act_cfg.get('composable', True),
                ),
            )

        # -- Create action clients -----------------------------------
        self._create_action_clients()

    def _create_action_clients(self):
        """Create ``ActionClient`` instances from the map ``actions``.

        Clients are shared when multiple actions target the same
        ``action_server``.
        """
        created = {}   # action_server -> (client, action_class)
        self._action_clients = {}

        for act_name, act_cfg in self._map_actions.items():
            type_str = act_cfg.get('action_type', '')
            server = act_cfg.get('action_server', '')

            # Dynamically import the action type (e.g. nav2_msgs.action.NavigateToPose)
            try:
                action_class = self._load_action_type(type_str)
            except Exception as exc:
                self.get_logger().warning(
                    "[MAP] Cannot load action_type '%s' for '%s': %s"
                    % (type_str, act_name, exc),
                )
                continue

            if server in created:
                client, existing = created[server]
                if existing != action_class:
                    self.get_logger().error(
                        "[MAP] Conflicting types on '%s': %s vs %s"
                        % (server, existing.__name__, action_class.__name__),
                    )
                    continue
            else:
                client = ActionClient(
                    self, action_class, server,
                    callback_group=self._nav2_cb_group,
                )
                created[server] = (client, action_class)
                self.get_logger().info(
                    "[MAP] Client: %s -> %s"
                    % (action_class.__name__, server),
                )

            self._action_clients[act_name] = {
                'client': client,
                'action_class': action_class,
                'config': act_cfg,
            }

    # =================================================================
    # Lifecycle
    # =================================================================

    def _on_shutdown(self):
        self.get_logger().info("[SHUTDOWN] Tearing down")
        if self._navigation_activated:
            self._preempted = True
            self._cancel_nav2_goal(timeout_sec=2.0)

    # =================================================================
    # Topic callbacks
    # =================================================================

    def _map_cb(self, msg):
        """``/topological_map_2`` callback -- buffer map update.

        The first map is applied immediately (needed for init).  All
        subsequent maps received during active navigation are buffered
        and applied at safe points between route segments.
        """
        with self._map_lock:
            self._pending_map_msg = msg
            if not self._map_received:
                # First map: apply immediately so __init__ can proceed.
                self._apply_pending_map()
                return

        if self._navigation_activated:
            self.get_logger().info(
                "[MAP] Update buffered (navigation active)",
            )
        else:
            # Not navigating -- safe to apply immediately.
            self._apply_pending_map()

    def _apply_pending_map(self):
        """Apply a buffered map update.  Thread-safe.

        Returns ``True`` if a new map was applied, ``False`` if there
        was nothing pending or the update failed.
        """
        with self._map_lock:
            if self._pending_map_msg is None:
                return False
            msg = self._pending_map_msg
            self._pending_map_msg = None

        try:
            tmap = yaml.load(msg.data, Loader=_FloatSafeLoader)
            graph = build_graph_from_tmap(
                tmap, logger=self.get_logger(),
            )
            if graph is None:
                self.get_logger().error("[MAP] Graph build failed")
                return False

            self._tmap = tmap
            self._graph = graph
            self._topol_map = tmap.get("pointset", "unknown")
            self._map_received = True
            self._load_map_config()

            self.get_logger().info(
                "[MAP] Applied update '%s' -- %d nodes, %d edges"
                % (
                    self._topol_map,
                    graph.number_of_nodes(),
                    graph.number_of_edges(),
                ),
            )
            return True
        except Exception as exc:
            self.get_logger().error("[MAP] Apply error: %s" % exc)
            return False

    def _validate_remaining_route(self, route_nodes, start_idx):
        """Check that remaining nodes and edges still exist in the graph.

        Parameters
        ----------
        route_nodes : list[str]
            Full ordered list of node names in the route.
        start_idx : int
            Index into *route_nodes* of the segment about to execute
            (i.e. remaining = route_nodes[start_idx:]).

        Returns
        -------
        bool
            ``True`` if every remaining node exists in ``self._graph``
            and every consecutive pair is connected by an edge.
        """
        remaining = route_nodes[start_idx:]
        for node_name in remaining:
            if node_name not in self._graph:
                self.get_logger().warning(
                    "[MAP-CHECK] Node '%s' no longer in map"
                    % node_name,
                )
                return False

        for i in range(len(remaining) - 1):
            src, tgt = remaining[i], remaining[i + 1]
            if not self._graph.has_edge(src, tgt):
                self.get_logger().warning(
                    "[MAP-CHECK] Edge '%s' -> '%s' no longer in map"
                    % (src, tgt),
                )
                return False

        return True

    def _closest_node_cb(self, msg):
        self._loc_received = True
        self._closest_node = msg.data

    def _closest_edges_cb(self, msg):
        self._closest_edges = msg

    def _current_node_cb(self, msg):
        if self._current_node != msg.data:
            self._current_node = msg.data
            if msg.data != "none":
                self.get_logger().info(
                    "[LOC] Entered node: %s" % self._current_node,
                )
                if (
                    self._navigation_activated
                    and self._current_node == self._current_target
                ):
                    self.get_logger().info(
                        "[LOC] Reached target: %s" % self._current_target,
                    )
                    self._goal_reached = True

    def _route_pub_cb(self):
        if self._stroute and self._stroute.nodes:
            self._route_pub.publish(self._stroute)

    # =================================================================
    # Status publishers
    # =================================================================

    def _publish_status(self, state_str):
        msg = String()
        msg.data = state_str
        self._status_pub.publish(msg)

    def _publish_boundary(self, polygon_pts, frame_id="map"):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        for x, y in polygon_pts:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.polygon.points.append(pt)
        self._boundary_pub.publish(msg)
        self.get_logger().info(
            "[BOUNDARY] %d-point polygon (frame=%s)"
            % (len(polygon_pts), frame_id),
        )

    def _publish_empty_boundary(self, frame_id="map"):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        self._boundary_pub.publish(msg)

    def _publish_route(self, route_nodes):
        stroute = TopologicalRoute()
        for n in route_nodes:
            stroute.nodes.append(n)
        self._stroute = stroute
        self._route_pub.publish(stroute)

    def _publish_stats(self):
        if self._stat is None:
            return
        s = self._stat
        msg = NavStatistics()
        msg.edge_id = s.edge_id
        msg.status = s.status
        msg.origin = s.origin
        msg.target = s.target
        msg.topological_map = s.topological_map
        msg.time_to_waypoint = float(s.time_to_wp)
        msg.operation_time = s.operation_time
        msg.date_started = s.get_start_time_str()
        msg.date_at_node = s.date_at_node.strftime(
            "%A, %B %d %Y, at %H:%M:%S hours",
        )
        msg.date_finished = s.get_finish_time_str()
        self._stats_pub.publish(msg)

    def _publish_current_edge(self, edge_id):
        msg = String()
        msg.data = (
            "%s--%s" % (edge_id, self._topol_map)
            if edge_id != "none" else "none"
        )
        self._cur_edge_pub.publish(msg)

    def _publish_move_status(self, goal_node, action, status_str):
        d = {
            "goal": goal_node,
            "final_goal": self._target,
            "action": action.upper(),
            "status": status_str,
        }
        msg = String()
        msg.data = json.dumps(d)
        self._move_status_pub.publish(msg)

    # =================================================================
    # Pose construction
    # =================================================================

    def _nav_frame(self):
        """Default navigation frame from ``transformation.topo_frame_id``.

        Falls back to ``transformation.parent``, then ``'map'``.
        """
        tx = self._tmap.get('transformation', {})
        return tx.get('topo_frame_id', tx.get('parent', 'map'))

    def _node_nav_frame(self, node_name):
        """Resolve the navigation frame for a specific node.

        Lookup order:
            1. ``nav_frame`` attribute on the graph node (per-node override)
            2. Map-level default via :meth:`_nav_frame`
        """
        if node_name and node_name in self._graph:
            nf = self._graph.nodes[node_name].get('nav_frame', '')
            if nf:
                return nf
        return self._nav_frame()

    def _build_pose_stamped(self, node_dict, ignore_orientation=False):
        """Build ``PoseStamped`` from a raw topomap node dict."""
        nd = node_dict["node"]
        pose = nd["pose"]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        raw_frame = nd.get("nav_frame", '')
        if raw_frame and raw_frame.startswith('${'):
            raw_frame = self._resolve_tmap_ref(raw_frame)
        ps.header.frame_id = raw_frame or self._nav_frame()
        ps.pose.position.x = float(pose["position"]["x"])
        ps.pose.position.y = float(pose["position"]["y"])
        ps.pose.position.z = float(pose["position"]["z"])
        if ignore_orientation:
            ps.pose.orientation.w = 1.0
        else:
            ps.pose.orientation.x = float(pose["orientation"]["x"])
            ps.pose.orientation.y = float(pose["orientation"]["y"])
            ps.pose.orientation.z = float(pose["orientation"]["z"])
            ps.pose.orientation.w = float(pose["orientation"]["w"])
        return ps

    def _build_pose_from_graph(self, node_name, ignore_orientation=False):
        """Build ``PoseStamped`` from NetworkX graph attributes."""
        if node_name not in self._graph:
            return None
        attrs = self._graph.nodes[node_name]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self._node_nav_frame(node_name)
        ps.pose.position.x = float(attrs.get('x', 0.0))
        ps.pose.position.y = float(attrs.get('y', 0.0))
        ps.pose.position.z = float(attrs.get('z', 0.0))
        if ignore_orientation:
            ps.pose.orientation.w = 1.0
        else:
            ori = attrs.get(
                'orientation',
                {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            )
            ps.pose.orientation.x = float(ori.get('x', 0.0))
            ps.pose.orientation.y = float(ori.get('y', 0.0))
            ps.pose.orientation.z = float(ori.get('z', 0.0))
            ps.pose.orientation.w = float(ori.get('w', 1.0))
        return ps

    def _pose_or_fallback(self, node_name, ignore_orientation=False):
        """Build pose from graph, falling back to raw YAML."""
        ps = self._build_pose_from_graph(node_name, ignore_orientation)
        if ps is None:
            nd = get_node_from_tmap2(self._tmap, node_name)
            if nd:
                ps = self._build_pose_stamped(nd, ignore_orientation)
        return ps

    def _build_edge_oriented_pose(self, node_name, next_node_name):
        """Build ``PoseStamped`` at *node_name* oriented toward *next_node*.

        Used for intermediate waypoints in composable segments so the
        robot faces the direction of the next edge instead of using
        the stored node orientation.

        Args:
            node_name: Node whose position is used.
            next_node_name: Node toward which the orientation points.

        Returns:
            ``PoseStamped`` with yaw facing *next_node_name*,
            or ``None`` if either node is missing from the graph.
        """
        if (
            node_name not in self._graph
            or next_node_name not in self._graph
        ):
            return None

        attrs = self._graph.nodes[node_name]
        next_attrs = self._graph.nodes[next_node_name]

        dx = float(next_attrs.get('x', 0.0)) - float(attrs.get('x', 0.0))
        dy = float(next_attrs.get('y', 0.0)) - float(attrs.get('y', 0.0))
        yaw = math.atan2(dy, dx)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self._node_nav_frame(node_name)
        ps.pose.position.x = float(attrs.get('x', 0.0))
        ps.pose.position.y = float(attrs.get('y', 0.0))
        ps.pose.position.z = float(attrs.get('z', 0.0))
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    # =================================================================
    # Nav2 goal tolerances
    # =================================================================

    def _set_goal_tolerances(self, node_name):
        """Set Nav2 goal checker tolerances from node properties.

        Reads ``xy_goal_tolerance`` and ``yaw_goal_tolerance`` from
        the target node's properties and sets them on the Nav2
        controller_server via the ``SetParameters`` service.

        This is best-effort: if the service is unavailable the
        goal proceeds with the previously configured tolerances.
        """
        if node_name not in self._graph:
            return

        props = self._graph.nodes[node_name].get('properties', {})
        xy_tol = props.get('xy_goal_tolerance')
        yaw_tol = props.get('yaw_goal_tolerance')

        if xy_tol is None and yaw_tol is None:
            return

        # Skip if unchanged from last set
        if xy_tol == self._last_xy_tol and yaw_tol == self._last_yaw_tol:
            return

        params = []
        if xy_tol is not None:
            p = RclParameter()
            p.name = self._xy_tolerance_param
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(xy_tol),
            )
            params.append(p)

        if yaw_tol is not None:
            p = RclParameter()
            p.name = self._yaw_tolerance_param
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(yaw_tol),
            )
            params.append(p)

        if not self._set_params_client.service_is_ready():
            self.get_logger().debug(
                "[TOL] SetParameters service not available, skipping",
            )
            return

        req = SetParameters.Request()
        req.parameters = params
        future = self._set_params_client.call_async(req)
        future.add_done_callback(self._tolerance_set_cb)

        self._last_xy_tol = xy_tol
        self._last_yaw_tol = yaw_tol

        self.get_logger().info(
            "[TOL] Setting tolerances for '%s': xy=%.3f yaw=%.3f"
            % (
                node_name,
                float(xy_tol) if xy_tol is not None else -1,
                float(yaw_tol) if yaw_tol is not None else -1,
            ),
        )

    def _tolerance_set_cb(self, future):
        """Log result of tolerance parameter update."""
        try:
            result = future.result()
            failed = [
                r for r in result.results if not r.successful
            ]
            if failed:
                self.get_logger().warning(
                    "[TOL] Some parameters failed to set",
                )
        except Exception as exc:
            self.get_logger().debug(
                "[TOL] SetParameters call failed: %s" % exc,
            )

    # =================================================================
    # Goal construction (map-driven)
    # =================================================================

    def _resolve_tmap_ref(self, value):
        """Resolve ``${transformation.<key>}`` against the map."""
        if not isinstance(value, str):
            return value
        if not (
            value.startswith('${transformation.')
            and value.endswith('}')
        ):
            return value
        key = value[len('${transformation.'):-1]
        tx = self._tmap.get('transformation', {})
        return tx.get(key, value)

    def _resolve_node_ref(self, value, node_name):
        """Resolve ``${node.<attr>}`` and ``${transformation.<key>}``.

        Looks up *attr* on the NetworkX graph node for *node_name*,
        or resolves transformation-level references.
        Returns the resolved value, or the original string if the
        pattern does not match or the attribute is missing.
        """
        if not isinstance(value, str):
            return value
        if value.startswith('${transformation.'):
            return self._resolve_tmap_ref(value)
        if not (value.startswith('${node.') and value.endswith('}')):
            return value
        attr = value[len('${node.'):-1]
        if node_name and node_name in self._graph:
            resolved = self._graph.nodes[node_name].get(attr, '')
            if resolved:
                return resolved
        return value

    def _resolve_template_frame(self, template, node_name):
        """Return the resolved ``header.frame_id`` from *template*.

        Supports the nested format where ``header`` lives inside the
        goal field (``pose.header`` or ``poses[0].header``) as well as
        the legacy flat format (``header`` at template root).

        Returns ``None`` when the template does not declare a header.
        """
        header = None

        # New nested format: pose.header or poses[0].header
        pose_tpl = template.get('pose')
        if isinstance(pose_tpl, dict):
            header = pose_tpl.get('header', {})
        if not header:
            poses_tpl = template.get('poses')
            if isinstance(poses_tpl, list) and poses_tpl:
                first = poses_tpl[0]
                if isinstance(first, dict):
                    header = first.get('header', {})

        # Legacy flat format: header at template root
        if not header:
            header = template.get('header', {})

        if not header:
            return None
        raw = header.get('frame_id', '')
        if not raw:
            return None
        return self._resolve_node_ref(raw, node_name)

    def _build_segment_goal(self, segment, is_final_segment):
        """Build a Nav2 goal dynamically from map ``actions`` config.

        The action class is resolved at map-load time via
        ``_load_action_type`` and stored in ``_action_clients``.
        Single-pose vs multi-pose dispatch is detected by checking
        whether the goal object has a ``poses`` (list) or ``pose``
        (single) attribute -- no hardcoded action imports needed.

        The ``action_goal_template`` mirrors the ROS 2 goal structure::

            action_goal_template:
              pose:                          # PoseStamped wrapper
                header:
                  frame_id: '${node.nav_frame}'
                pose: '${node.pose}'
              behavior_tree: '${definitions.default_bt}'

        If the template contains a ``header.frame_id`` (nested under
        ``pose`` or ``poses[0]``), the resolved value overrides the
        frame on every built ``PoseStamped``.
        """
        action = segment.action_type
        info = self._action_clients.get(action)

        if not info:
            self.get_logger().error(
                "[GOAL] No action config for '%s'" % action,
            )
            return None

        action_class = info['action_class']
        tpl = info['config'].get('action_goal_template', {})
        bt = tpl.get('behavior_tree', '')
        if isinstance(bt, str) and bt.startswith('${'):
            bt = ''

        goal = action_class.Goal()

        # Multi-waypoint goal (e.g. NavigateThroughPoses)
        if hasattr(goal, 'poses'):
            n = segment.num_edges
            for i, ed in enumerate(segment.edge_data):
                is_last = (i == n - 1)
                tgt = ed['target']

                if not is_last:
                    # Intermediate waypoint: orient toward the next
                    # edge's target so the robot faces its travel
                    # direction instead of using the node's stored
                    # orientation.
                    next_tgt = segment.edge_data[i + 1]['target']
                    ps = self._build_edge_oriented_pose(tgt, next_tgt)
                    if ps is None:
                        ps = self._pose_or_fallback(
                            tgt, ignore_orientation=True,
                        )
                elif self._no_orientation and is_final_segment:
                    # Final waypoint with no-orientation flag.
                    ps = self._pose_or_fallback(
                        tgt, ignore_orientation=True,
                    )
                else:
                    # Final waypoint: keep the node's orientation.
                    ps = self._pose_or_fallback(tgt)

                if ps is not None:
                    frame = self._resolve_template_frame(
                        tpl, tgt,
                    )
                    if frame:
                        ps.header.frame_id = frame
                    goal.poses.append(ps)
            self.get_logger().info(
                "[GOAL] %s %d poses, BT=%s"
                % (action_class.__name__, len(goal.poses),
                   bt or 'default'),
            )
        # Single-pose goal (e.g. NavigateToPose)
        elif hasattr(goal, 'pose'):
            tgt = segment.last_target
            ignore = self._no_orientation and is_final_segment
            ps = self._pose_or_fallback(
                tgt, ignore_orientation=ignore,
            )
            if ps is not None:
                frame = self._resolve_template_frame(tpl, tgt)
                if frame:
                    ps.header.frame_id = frame
                goal.pose = ps
            self.get_logger().info(
                "[GOAL] %s -> '%s', BT=%s"
                % (action_class.__name__, tgt, bt or 'default'),
            )
        else:
            self.get_logger().warning(
                "[GOAL] %s goal has no 'pose'/'poses' attr"
                % action_class.__name__,
            )

        if bt and hasattr(goal, 'behavior_tree'):
            goal.behavior_tree = bt

        return goal

    # =================================================================
    # Nav2 dispatch
    # =================================================================

    def _send_nav2_goal(self, goal, action_client=None):
        """Send goal to Nav2.  Blocks until result.

        The main ``MultiThreadedExecutor`` processes action-client
        callbacks, so this method polls futures with ``time.sleep``
        instead of calling ``rclpy.spin_once`` (which would conflict
        with the executor that already owns the node).

        Returns ``GoalStatus`` integer.
        """
        if action_client is None:
            for info in self._action_clients.values():
                action_client = info['client']
                break
        if action_client is None:
            self.get_logger().error("[NAV2] No action client available")
            return GoalStatus.STATUS_ABORTED

        if not action_client.server_is_ready():
            self.get_logger().info("[NAV2] Waiting for server ...")
            if not action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("[NAV2] Server unavailable")
                return GoalStatus.STATUS_ABORTED

        future = action_client.send_goal_async(
            goal, feedback_callback=self._nav2_fb_cb,
        )

        # Wait for goal acceptance
        while rclpy.ok() and not future.done():
            if self._preempted or self._cancelled:
                return GoalStatus.STATUS_CANCELED
            time.sleep(0.1)

        if not future.done():
            self.get_logger().error("[NAV2] send_goal did not complete")
            return GoalStatus.STATUS_ABORTED

        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("[NAV2] Goal REJECTED")
            return GoalStatus.STATUS_ABORTED
        self.get_logger().info("[NAV2] Goal ACCEPTED")

        # Wait for result
        result_future = self._goal_handle.get_result_async()
        while rclpy.ok():
            if self._preempted or self._cancelled:
                self._cancel_nav2_goal()
                return GoalStatus.STATUS_CANCELED
            if result_future.done():
                res = result_future.result()
                self._action_status = res.status
                self.get_logger().info(
                    "[NAV2] Finished: %s" % _status_str(res.status),
                )
                return res.status
            time.sleep(0.1)
        return GoalStatus.STATUS_ABORTED

    def _nav2_fb_cb(self, _feedback_msg):
        self._action_status = GoalStatus.STATUS_EXECUTING

    def _cancel_nav2_goal(self, timeout_sec=None):
        """Cancel the active Nav2 goal (fire-and-forget)."""
        if self._goal_handle is None:
            return
        try:
            self._goal_handle.cancel_goal_async()
            self.get_logger().info("[NAV2] Cancel requested")
        except Exception as exc:
            self.get_logger().error("[NAV2] Cancel error: %s" % exc)
        finally:
            self._goal_handle = None

    # =================================================================
    # Action-server callbacks
    # =================================================================

    def _exec_goto_cb(self, goal_handle):
        """``GotoNode`` action callback."""
        target = goal_handle.request.target
        self.get_logger().info(
            "=" * 60 + "\n[GOTO] target='%s', no_ori=%s"
            % (target, goal_handle.request.no_orientation),
        )
        # Preempt any active navigation and wait for it to stop
        if self._navigation_activated:
            self.get_logger().info("[GOTO] Preempting active navigation")
            self._preempted = True
            self._cancel_nav2_goal()
            deadline = time.time() + 5.0
            while self._navigation_activated and time.time() < deadline:
                time.sleep(0.05)
            if self._navigation_activated:
                self.get_logger().warning(
                    "[GOTO] Timeout waiting for old navigation",
                )
                self._navigation_activated = False
            if self._sm.is_terminal():
                self._sm.reset()
            elif self._sm.state != NavState.READY:
                self._sm.transition(NavState.CANCELLED)
                self._sm.reset()

        self._navigation_activated = True
        self._cancelled = False
        self._preempted = False
        self._no_orientation = goal_handle.request.no_orientation

        fb = GotoNodeFeedback()
        fb.route = "Planning..."
        self._goto_fb_pub.publish(fb)

        success = self._navigate(target)
        self._navigation_activated = False

        result = GotoNode.Result()
        result.success = success
        if success:
            goal_handle.succeed()
            self.get_logger().info("[GOTO] SUCCEEDED -> '%s'" % target)
        else:
            goal_handle.abort()
            self.get_logger().warning(
                "[GOTO] %s -> '%s'"
                % ("CANCELLED" if self._preempted else "FAILED", target),
            )
        if self._sm.is_terminal():
            self._sm.reset()
        return result

    def _exec_policy_cb(self, goal_handle):
        """``ExecutePolicyMode`` action callback."""
        self.get_logger().info("=" * 60 + "\n[POLICY] Goal received")

        # Preempt any active navigation and wait for it to stop
        if self._navigation_activated:
            self.get_logger().info("[POLICY] Preempting active navigation")
            self._preempted = True
            self._cancel_nav2_goal()
            deadline = time.time() + 5.0
            while self._navigation_activated and time.time() < deadline:
                time.sleep(0.05)
            if self._navigation_activated:
                self.get_logger().warning(
                    "[POLICY] Timeout waiting for old navigation",
                )
                self._navigation_activated = False
            if self._sm.is_terminal():
                self._sm.reset()
            elif self._sm.state != NavState.READY:
                self._sm.transition(NavState.CANCELLED)
                self._sm.reset()

        self._navigation_activated = True
        self._cancelled = False
        self._preempted = False

        route = goal_handle.request.route
        if (
            len(route.source) < 1
            or len(route.source) != len(route.edge_id)
        ):
            self.get_logger().error("[POLICY] Invalid route data")
            self._navigation_activated = False
            goal_handle.succeed()
            return ExecutePolicyMode.Result(success=False)

        if (
            route.source[0] != self._current_node
            and route.source[0] != self._closest_node
        ):
            self.get_logger().error(
                "[POLICY] Route starts at '%s' but robot at '%s'"
                % (route.source[0], self._current_node),
            )
            self._navigation_activated = False
            goal_handle.succeed()
            return ExecutePolicyMode.Result(success=False)

        route_nodes = list(route.source)
        if route.edge_id:
            last_e = get_edge_from_id_tmap2(
                self._tmap, route.source[-1], route.edge_id[-1],
            )
            if last_e:
                ft = last_e["node"]
                if not route_nodes or route_nodes[-1] != ft:
                    route_nodes.append(ft)

        target = route_nodes[-1]
        self.get_logger().info(
            "[POLICY] Route: %s" % " -> ".join(route_nodes),
        )
        self._publish_route(route_nodes)
        success = self._execute_route(route_nodes, target)

        self._navigation_activated = False
        if self._sm.is_terminal():
            self._sm.reset()
        goal_handle.succeed()
        self.get_logger().info(
            "[POLICY] %s" % ("SUCCEEDED" if success else "FAILED"),
        )
        return ExecutePolicyMode.Result(success=success)

    def _cancel_goto_cb(self, _gh):
        self.get_logger().warning("[GOTO] Cancel requested")
        self._preempted = True
        self._cancel_nav2_goal()

    def _cancel_policy_cb(self, _gh):
        self.get_logger().warning("[POLICY] Cancel requested")
        self._preempted = True
        self._cancel_nav2_goal()

    # =================================================================
    # Core navigation
    # =================================================================

    def _navigate(self, target):
        """Plan route and execute.  Returns ``True`` on success."""
        if self._cancelled:
            return False
        self._sm.transition(NavState.PLANNING)
        self._publish_status("PLANNING")
        self._target = target

        if target not in self._graph:
            self.get_logger().error("[NAV] '%s' not in map" % target)
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")
            return False

        origin = self._determine_origin(target)
        if origin is None:
            self.get_logger().error("[NAV] Cannot determine origin")
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")
            return False

        self.get_logger().info(
            "[NAV] Planning '%s' -> '%s'" % (origin, target),
        )

        if origin == target:
            self.get_logger().info("[NAV] Already at target")
            self._sm.transition(NavState.SUCCEEDED)
            self._publish_status("SUCCEEDED")
            return True

        route_nodes = plan_route(
            self._graph, origin, target,
            logger=self.get_logger(),
            algorithm=self._route_algorithm,
            weight=self._route_weight,
        )
        if not route_nodes or len(route_nodes) < 2:
            self.get_logger().error(
                "[NAV] No route '%s' -> '%s'" % (origin, target),
            )
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")
            return False

        self.get_logger().info(
            "[NAV] Route: %s (%d nodes)"
            % (" -> ".join(route_nodes), len(route_nodes)),
        )
        self._publish_route(route_nodes)
        success = self._execute_route(route_nodes, target)

        # If the map was updated mid-execution, replan from scratch.
        if not success and self._map_updated_during_nav:
            self._map_updated_during_nav = False
            self.get_logger().info(
                "[NAV] Replanning after mid-navigation map update",
            )
            return self._navigate(target)

        if not success and not self._cancelled and not self._preempted:
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")
        return success

    def _determine_origin(self, target):
        """Best origin: current_node > closest-edge > closest_node."""
        if self._current_node not in ("none", "Unknown"):
            return self._current_node

        if (
            self._closest_edges.distances
            and self._closest_edges.distances[0]
            <= self._max_dist_to_closest_edge
        ):
            origin = self._origin_from_closest_edge(target)
            if origin:
                return origin

        if self._closest_node != "Unknown":
            return self._closest_node
        return None

    def _origin_from_closest_edge(self, target):
        eids = self._closest_edges.edge_ids

        def _src(eid):
            if not eid:
                return None
            for u, _v, d in self._graph.edges(data=True):
                if d.get('edge_id') == eid:
                    return u
            return None

        src1 = _src(eids[0] if eids else None)
        if src1 is None:
            return self._closest_node

        if len(eids) > 1 and len(self._closest_edges.distances) > 1:
            src2 = _src(eids[1])
            if (
                src2
                and self._closest_edges.distances[0]
                == self._closest_edges.distances[1]
            ):
                r1 = plan_route(
                    self._graph, src1, target,
                    algorithm=self._route_algorithm,
                    weight=self._route_weight,
                )
                r2 = plan_route(
                    self._graph, src2, target,
                    algorithm=self._route_algorithm,
                    weight=self._route_weight,
                )
                d1 = get_route_distance(self._graph, r1) if r1 else float('inf')
                d2 = get_route_distance(self._graph, r2) if r2 else float('inf')
                return src1 if d1 <= d2 else src2
        return src1

    # =================================================================
    # Route execution
    # =================================================================

    def _execute_route(self, route_nodes, target):
        """Execute a planned route segment by segment.

        Between segments the method checks for buffered map updates.
        If the map changed, the remaining route is validated; if any
        node or edge is now missing the method signals a replan by
        setting ``_map_updated_during_nav`` and returning ``False``.
        """
        self._navigation_activated = True
        self._map_updated_during_nav = False

        route_edges = get_route_edges(self._graph, route_nodes)
        if not route_edges:
            self.get_logger().error("[EXEC] No edges in route")
            return False

        segments = merge_action_segments(
            route_edges, map_actions=self._map_actions or None,
        )

        self.get_logger().info(
            "[EXEC] %d edges -> %d segment(s):"
            % (len(route_edges), len(segments)),
        )
        for i, seg in enumerate(segments):
            self.get_logger().info(
                "  [%d] %s x%d: %s -> %s" % (
                    i, seg.action_type, seg.num_edges,
                    seg.first_source, seg.last_target,
                ),
            )

        # Track which node index the current segment starts at so we
        # can validate the *remaining* route after a map update.
        node_idx = 0

        for si, seg in enumerate(segments):
            if self._cancelled or self._preempted:
                self._sm.transition(NavState.CANCELLED)
                self._publish_status("CANCELLED")
                return False

            # -- Safe point: apply any buffered map update -----------
            if self._apply_pending_map():
                self.get_logger().info(
                    "[EXEC] Map updated between segments",
                )
                if not self._validate_remaining_route(
                    route_nodes, node_idx,
                ):
                    self.get_logger().warning(
                        "[EXEC] Remaining route invalid after map "
                        "update -- requesting replan",
                    )
                    self._map_updated_during_nav = True
                    return False
                # Route still valid on new graph; re-derive segments
                # from the remaining nodes so edge lookups use the
                # updated ``self._tmap``.
                remaining_nodes = route_nodes[node_idx:]
                route_edges = get_route_edges(
                    self._graph, remaining_nodes,
                )
                if not route_edges:
                    self.get_logger().warning(
                        "[EXEC] No edges after re-derive -- replan",
                    )
                    self._map_updated_during_nav = True
                    return False
                segments = merge_action_segments(
                    route_edges,
                    map_actions=self._map_actions or None,
                )
                # Restart the segment loop with updated segments
                return self._execute_remaining_segments(
                    segments, route_nodes[node_idx:], target,
                )

            is_final = si == len(segments) - 1
            ok = self._execute_segment(seg, is_final, si, len(segments))
            if not ok:
                if self._cancelled or self._preempted:
                    self._sm.transition(NavState.CANCELLED)
                    self._publish_status("CANCELLED")
                return False

            # Advance node_idx past the nodes consumed by this segment
            node_idx += seg.num_edges

        self._sm.transition(NavState.SUCCEEDED)
        self._publish_status("SUCCEEDED")
        return True

    def _execute_remaining_segments(
        self, segments, route_nodes, target,
    ):
        """Continue segment execution after a mid-route map update.

        This is factored out of ``_execute_route`` to avoid resetting
        the segment loop counter.  The logic is identical.
        """
        node_idx = 0

        for si, seg in enumerate(segments):
            if self._cancelled or self._preempted:
                self._sm.transition(NavState.CANCELLED)
                self._publish_status("CANCELLED")
                return False

            # Check for *another* map update
            if self._apply_pending_map():
                self.get_logger().info(
                    "[EXEC] Map updated again between segments",
                )
                if not self._validate_remaining_route(
                    route_nodes, node_idx,
                ):
                    self._map_updated_during_nav = True
                    return False
                remaining_nodes = route_nodes[node_idx:]
                route_edges = get_route_edges(
                    self._graph, remaining_nodes,
                )
                if not route_edges:
                    self._map_updated_during_nav = True
                    return False
                segments = merge_action_segments(
                    route_edges,
                    map_actions=self._map_actions or None,
                )
                return self._execute_remaining_segments(
                    segments, route_nodes[node_idx:], target,
                )

            is_final = si == len(segments) - 1
            ok = self._execute_segment(seg, is_final, si, len(segments))
            if not ok:
                if self._cancelled or self._preempted:
                    self._sm.transition(NavState.CANCELLED)
                    self._publish_status("CANCELLED")
                return False

            node_idx += seg.num_edges

        self._sm.transition(NavState.SUCCEEDED)
        self._publish_status("SUCCEEDED")
        return True

    def _execute_segment(self, segment, is_final, seg_idx, total):
        """Execute one action segment."""
        action = segment.action_type
        exec_state = ACTION_TO_STATE.get(
            action, NavState.EXECUTING_NAVIGATE_TO_POSE,
        )
        self._sm.transition(exec_state)
        self._publish_status(exec_state.value)

        self.get_logger().info(
            "[SEG %d/%d] %s x%d: %s -> %s" % (
                seg_idx + 1, total, action, segment.num_edges,
                segment.first_source, segment.last_target,
            ),
        )

        # Publish boundary polygon if edges carry boundary props
        self._publish_segment_boundary(segment)

        # Pre-flight: validate all edges
        edge_dicts = []
        for edata in segment.edge_data:
            if self._cancelled or self._preempted:
                return False

            src = edata['source']
            tgt = edata['target']
            eid = edata.get('edge_id', '')

            ed = get_edge_from_id_tmap2(self._tmap, src, eid)
            if not ed:
                self.get_logger().error(
                    "  Edge '%s' (%s->%s): lookup failed"
                    % (eid, src, tgt),
                )
                self._stat = nav_stats(src, tgt, self._topol_map, eid)
                self._stat.set_ended(self._current_node)
                self._stat.status = "failed"
                self._publish_stats()
                return False

            edge_dicts.append(ed)

        # Build and send goal
        self._current_target = segment.last_target
        self._publish_current_edge(
            segment.edge_ids[0] if segment.edge_ids else "none",
        )

        # Set Nav2 goal checker tolerances from target node properties
        self._set_goal_tolerances(segment.last_target)

        self.get_logger().info(
            "  Sending %d-wp %s goal" % (segment.num_edges, action),
        )

        goal = self._build_segment_goal(segment, is_final)
        self._stat = nav_stats(
            segment.first_source or "?",
            segment.last_target or "?",
            self._topol_map,
            segment.edge_ids[0] if segment.edge_ids else "",
        )

        info = self._action_clients.get(action)
        client = info['client'] if info else None
        status = self._send_nav2_goal(goal, action_client=client)

        self._publish_move_status(
            segment.last_target or "?", action, _status_str(status),
        )

        # Evaluate
        self._stat.set_ended(self._current_node)
        if status == GoalStatus.STATUS_SUCCEEDED or self._goal_reached:
            self._stat.status = "success"
            self._publish_stats()
            self.get_logger().info(
                "  Segment OK: %s -> %s (%.1fs)" % (
                    segment.first_source, segment.last_target,
                    self._stat.operation_time,
                ),
            )
            self._goal_reached = False
        else:
            self._stat.status = "failed"
            self._publish_stats()
            if status == GoalStatus.STATUS_CANCELED:
                self._preempted = True
            self.get_logger().warning(
                "  Segment FAILED: %s -> %s (%s)" % (
                    segment.first_source, segment.last_target,
                    _status_str(status),
                ),
            )
            return False

        self._publish_current_edge("none")
        return True

    # =================================================================
    # Boundary publishing
    # =================================================================

    def _publish_segment_boundary(self, segment):
        """Publish a boundary polygon if edges have boundary properties.

        Checks whether any edge in the segment carries
        ``boundary_left`` or ``boundary_right`` properties.  If so,
        a corridor polygon is computed and published.  Otherwise an
        empty polygon is published to clear any previous boundary.
        """
        has_boundary = any(
            'boundary_left' in ed.get('properties', {})
            or 'boundary_right' in ed.get('properties', {})
            for ed in segment.edge_data
        )
        if not has_boundary:
            self._publish_empty_boundary()
            return

        frame_id = self._node_nav_frame(segment.first_source)
        poly = compute_boundary_polygon(
            self._graph, segment,
            default_left=self._default_boundary_left,
            default_right=self._default_boundary_right,
        )
        if poly:
            self._publish_boundary(poly, frame_id)
        else:
            self.get_logger().warning(
                "[BOUNDARY] Cannot compute boundary polygon",
            )
            self._publish_empty_boundary(frame_id)

    # =================================================================
    # Helpers
    # =================================================================

    def _cancel_current_nav(self):
        if self._navigation_activated:
            self.get_logger().info("[CANCEL] Stopping current nav")
            self._cancel_nav2_goal(timeout_sec=2.0)
            self._cancelled = True
            self._navigation_activated = False
            # Reset state machine so next goal can transition to PLANNING
            if not self._sm.is_terminal() and self._sm.state != NavState.READY:
                self._sm.transition(NavState.CANCELLED)
            if self._sm.is_terminal():
                self._sm.reset()


# =====================================================================
# Entry point
# =====================================================================

def main():
    rclpy.init(args=None)
    node = TopologicalNavServer('topological_navigation')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[SHUTDOWN] Keyboard interrupt")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
