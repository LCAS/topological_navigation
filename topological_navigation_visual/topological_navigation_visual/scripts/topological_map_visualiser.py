#!/usr/bin/env python3
"""Unified topological map visualiser with interactive editing.

Provides a single ROS 2 node that:
- Subscribes to ``/topological_map_2`` and renders nodes, edges, zones, and
  labels as RViz ``MarkerArray`` messages.
- Optionally loads a map directly from a YAML file for editing (``map_file``
  parameter).
- Creates interactive markers for every node, allowing the user to drag
  positions (X/Y) and rotate yaw in RViz.
- Saves the modified map back to YAML and re-publishes it on
  ``/topological_map_2`` so that downstream nodes pick up changes
  immediately.

Usage
-----
**Live visualisation only** (subscribes to map topic)::

    ros2 run topological_navigation_visual topological_map_visualiser.py

**Interactive editing from file**::

    ros2 run topological_navigation_visual topological_map_visualiser.py \\
        --ros-args -p map_file:=/path/to/map.tmap2.yaml

**Save** (from another terminal)::

    ros2 service call /topological_map_visualiser/save_map std_srvs/srv/Trigger

"""

import math
import os
import sys
import tempfile
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

import yaml
import tf_transformations

from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)
from rclpy.action import ActionClient
from interactive_markers import InteractiveMarkerServer, MenuHandler

from topological_navigation.tmap_utils import CustomSafeLoader
from topological_navigation.navigation_graph import plan_route
from topological_navigation.networkx_utils import build_graph_from_tmap
from topological_navigation_msgs.action import GotoNode

# ──────────────────────────────────────────────────────────────────
#  Colour palette
# ──────────────────────────────────────────────────────────────────
_PALETTE = [
    [0.2, 0.2, 0.7],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [1.0, 1.0, 0.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
    [1.0, 1.0, 1.0],
]


def _colour(index: int):
    """Return an RGB list from the palette, wrapping around."""
    return _PALETTE[index % len(_PALETTE)]


def _node2pose(pose_dict) -> Pose:
    """Convert a tmap2 pose dict to a ``geometry_msgs/Pose``."""
    p = Pose()
    p.position.x = float(pose_dict['position']['x'])
    p.position.y = float(pose_dict['position']['y'])
    p.position.z = float(pose_dict['position']['z'])
    p.orientation.w = float(pose_dict['orientation']['w'])
    p.orientation.x = float(pose_dict['orientation']['x'])
    p.orientation.y = float(pose_dict['orientation']['y'])
    p.orientation.z = float(pose_dict['orientation']['z'])
    return p


# ══════════════════════════════════════════════════════════════════
#  Main visualiser node
# ══════════════════════════════════════════════════════════════════
class TopologicalMapVisualiser(Node):
    """Unified map visualiser with optional interactive editing."""

    def __init__(self):
        super().__init__('topological_map_visualiser')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('map_file', '')
        self.declare_parameter('auto_save', False)
        self.declare_parameter('marker_scale', 0.5)
        self.declare_parameter('edit_mode', True)
        self.declare_parameter(
            'nav_action_name', '/topological_navigation',
        )

        self.map_file: str = self.get_parameter('map_file').value
        self.auto_save: bool = self.get_parameter('auto_save').value
        self.marker_scale: float = self.get_parameter('marker_scale').value
        self.edit_mode: bool = self.get_parameter('edit_mode').value
        nav_action: str = self.get_parameter('nav_action_name').value

        # ── State ────────────────────────────────────────────────
        self.tmap = None
        self._graph = None
        self._map_dirty = False
        self._node_entries_by_name: dict[str, dict] = {}
        self._incoming_edges_by_target: dict[str, list[tuple[dict, dict]]] = {}
        self._action_names: list[str] = []
        self._action_index: dict[str, int] = {}
        self._marker_ids = {
            'nodes': {},
            'names': {},
            'zones': {},
            'edges': {},
            'legend': {},
        }
        self._navigating_to: str | None = None
        self._current_node: str = 'none'
        self._closest_node: str = 'none'
        self._route_nodes: list = []  # ordered node names on active route
        self._pending_route_target: str | None = None
        self._route_retry_timer = None
        self._initial_vis_timer = None

        # ── Debounced republish after drag ────────────────────────
        self._republish_timer = None
        self._temp_map_dir = os.path.join(
            tempfile.gettempdir(), 'topological_maps',
        )
        os.makedirs(self._temp_map_dir, exist_ok=True)

        # ── QoS ──────────────────────────────────────────────────
        self._latching_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._cb_group = ReentrantCallbackGroup()

        # ── Publishers ───────────────────────────────────────────
        self.map_marker_pub = self.create_publisher(
            MarkerArray,
            'topological_map_visualisation',
            qos_profile=self._latching_qos,
        )
        self.map_topic_pub = self.create_publisher(
            String,
            '/topological_map_2',
            qos_profile=self._latching_qos,
        )
        self.route_marker_pub = self.create_publisher(
            MarkerArray,
            'topological_route_visualisation',
            qos_profile=self._latching_qos,
        )

        # ── Interactive-marker server (for dragging nodes) ──────
        self._im_server = InteractiveMarkerServer(
            self, 'topological_map_editor'
        )

        # ── GotoNode action client ──────────────────────────────
        self._goto_client = ActionClient(
            self,
            GotoNode,
            nav_action,
            callback_group=self._cb_group,
        )
        self._goto_goal_handle = None

        # ── Menu handler for right-click context menu ────────────
        self._menu_handler = MenuHandler()
        self._menu_nav_id = self._menu_handler.insert(
            'Navigate Here', callback=self._menu_navigate_cb,
        )
        self._menu_cancel_id = self._menu_handler.insert(
            'Cancel Navigation', callback=self._menu_cancel_cb,
        )

        # ── Subscribe to localisation topics ─────────────────────
        self.create_subscription(
            String,
            '/current_node',
            self._current_node_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String,
            '/closest_node',
            self._closest_node_cb,
            10,
            callback_group=self._cb_group,
        )

        # ── Load map from file or subscribe to topic ─────────────
        if self.map_file:
            self._load_map_from_file()
            self._publish_map_topic()
            self._rebuild_visualisation()
            # Delayed re-publish so late-subscribing RViz gets markers
            self._initial_vis_timer = self.create_timer(
                2.0, self._delayed_initial_vis,
                callback_group=self._cb_group,
            )
        else:
            self.get_logger().info(
                'No map_file parameter — subscribing to /topological_map_2'
            )

        # Always subscribe so we pick up external updates too
        self.create_subscription(
            String,
            '/topological_map_2',
            self._map_topic_cb,
            qos_profile=self._latching_qos,
            callback_group=self._cb_group,
        )

        # ── Save service ─────────────────────────────────────────
        self.create_service(
            Trigger,
            f'{self.get_name()}/save_map',
            self._save_map_service,
            callback_group=self._cb_group,
        )

        # ── Auto-save timer ──────────────────────────────────────
        if self.auto_save and self.map_file:
            self.create_timer(30.0, self._auto_save_cb)

        self.get_logger().info('Topological map visualiser started')
        if self.map_file:
            self.get_logger().info(f'  map_file : {self.map_file}')
            self.get_logger().info(f'  edit_mode: {self.edit_mode}')
            self.get_logger().info(f'  auto_save: {self.auto_save}')
        self.get_logger().info(
            'Call /<node>/save_map service to persist changes'
        )

    def _refresh_visual_cache(self):
        """Build lookup tables used by static-marker updates."""
        self._node_entries_by_name = {}
        self._incoming_edges_by_target = {}
        self._action_names = []
        self._action_index = {}

        if self.tmap is None:
            return

        seen_actions = set()
        nodes = self.tmap.get('nodes', [])
        for entry in nodes:
            node = entry['node']
            self._node_entries_by_name[node['name']] = node

        for entry in nodes:
            node = entry['node']
            for edge in node.get('edges', []):
                target = edge.get('node', '')
                if target:
                    self._incoming_edges_by_target.setdefault(
                        target, []
                    ).append((node, edge))

                action = edge.get('action', '')
                if action and action not in seen_actions:
                    seen_actions.add(action)
                    self._action_names.append(action)

        self._action_index = {
            action: idx for idx, action in enumerate(self._action_names)
        }

    @staticmethod
    def _edge_key(source_name: str, edge: dict) -> str:
        """Return a stable key for an edge marker."""
        edge_id = edge.get('edge_id', '')
        if edge_id:
            return edge_id
        return f'{source_name}->{edge.get("node", "")}'

    # ──────────────────────────────────────────────────────────────
    #  Map loading / saving
    # ──────────────────────────────────────────────────────────────
    def _load_map_from_file(self):
        """Load a tmap2 YAML file into ``self.tmap``."""
        try:
            with open(self.map_file, 'r') as fh:
                self.tmap = yaml.load(fh, Loader=CustomSafeLoader)
            self._graph = build_graph_from_tmap(
                self.tmap, logger=self.get_logger(),
            )
            self.get_logger().info(
                f'Loaded map with {len(self.tmap.get("nodes", []))} nodes '
                f'from {self.map_file}'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load map file: {exc}')
            sys.exit(1)

    def save_map(self):
        """Save current map to the YAML file (with backup)."""
        if not self.map_file:
            self.get_logger().warn('No map_file set — cannot save')
            return False
        try:
            backup = self.map_file + '.backup'
            if os.path.exists(self.map_file):
                if os.path.exists(backup):
                    os.remove(backup)
                os.rename(self.map_file, backup)

            with open(self.map_file, 'w') as fh:
                yaml.dump(
                    self.tmap,
                    fh,
                    default_flow_style=False,
                    sort_keys=False,
                )
            self.get_logger().info(f'Map saved to {self.map_file}')
            self._map_dirty = False
            self._publish_map_topic()
            return True

        except Exception as exc:
            self.get_logger().error(f'Save failed: {exc}')
            backup = self.map_file + '.backup'
            if os.path.exists(backup):
                os.rename(backup, self.map_file)
            return False

    # ── Callbacks ────────────────────────────────────────────────
    def _current_node_cb(self, msg: String):
        """Track the robot's current topological node."""
        self._current_node = msg.data

    def _closest_node_cb(self, msg: String):
        """Track the robot's closest topological node."""
        self._closest_node = msg.data

    def _map_topic_cb(self, msg: String):
        """Handle updates from the ``/topological_map_2`` topic."""
        incoming = yaml.load(msg.data, Loader=CustomSafeLoader)
        if incoming == self.tmap:
            return  # no change, avoid flicker
        self.tmap = incoming
        self._graph = build_graph_from_tmap(
            self.tmap, logger=self.get_logger(),
        )
        self.get_logger().info('Received updated map from topic')
        self._rebuild_visualisation()
        # Delayed re-publish so late-subscribing RViz gets markers
        if self._initial_vis_timer is not None:
            self.destroy_timer(self._initial_vis_timer)
        self._initial_vis_timer = self.create_timer(
            2.0, self._delayed_initial_vis,
            callback_group=self._cb_group,
        )

    def _save_map_service(self, request, response):
        ok = self.save_map()
        response.success = ok
        response.message = (
            f'Map saved to {self.map_file}' if ok else 'Save failed'
        )
        return response

    def _auto_save_cb(self):
        if self._map_dirty:
            self.get_logger().info('Auto-saving map…')
            self.save_map()

    def _delayed_initial_vis(self):
        """One-shot republish so late-subscribing RViz gets markers."""
        if self._initial_vis_timer is not None:
            self.destroy_timer(self._initial_vis_timer)
            self._initial_vis_timer = None
        if self.tmap is not None:
            self._rebuild_static_markers()
            self.get_logger().debug(
                'Deferred map visualisation republished'
            )

    # ──────────────────────────────────────────────────────────────
    #  GotoNode navigation helpers
    # ──────────────────────────────────────────────────────────────
    def _menu_navigate_cb(self, feedback: InteractiveMarkerFeedback):
        """Context-menu callback: send GotoNode goal for this node."""
        node_name = feedback.marker_name
        self.get_logger().info(
            f'Navigate request → {node_name}'
        )
        # Compute and highlight route before sending goal
        self._compute_and_highlight_route(node_name)
        self._send_goto_goal(node_name)

    def _menu_cancel_cb(self, feedback: InteractiveMarkerFeedback):
        """Context-menu callback: cancel the active GotoNode goal."""
        self._cancel_navigation()

    def _send_goto_goal(self, target: str):
        """Send a ``GotoNode`` goal to the topological navigation server."""
        if not self._goto_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'GotoNode action server not available — is navigation2 '
                'running?'
            )
            return
        goal = GotoNode.Goal()
        goal.target = target
        goal.no_orientation = False
        self.get_logger().info(f'Sending GotoNode goal: target={target}')

        future = self._goto_client.send_goal_async(
            goal, feedback_callback=self._goto_feedback_cb,
        )
        future.add_done_callback(self._goto_response_cb)
        self._navigating_to = target

    def _goto_response_cb(self, future):
        """Handle the goal acceptance / rejection response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('GotoNode goal was rejected')
            self._navigating_to = None
            return
        self.get_logger().info('GotoNode goal accepted')
        self._goto_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goto_result_cb)

    def _goto_result_cb(self, future):
        """Handle the final result of a GotoNode action."""
        result = future.result().result
        status = 'SUCCESS' if result.success else 'FAILURE'
        self.get_logger().info(
            f'GotoNode result: {status} (target={self._navigating_to})'
        )
        self._navigating_to = None
        self._goto_goal_handle = None
        self._clear_route_highlight()

    def _goto_feedback_cb(self, feedback_msg):
        """Log GotoNode action feedback."""
        fb = feedback_msg.feedback
        self.get_logger().debug(f'GotoNode feedback: route={fb.route}')

    def _cancel_navigation(self):
        """Cancel the currently active GotoNode goal, if any."""
        if self._goto_goal_handle is not None:
            self.get_logger().info('Cancelling active GotoNode goal')
            self._goto_goal_handle.cancel_goal_async()
            self._navigating_to = None
            self._goto_goal_handle = None
            self._clear_route_highlight()
        else:
            self.get_logger().info('No active navigation goal to cancel')

    # ──────────────────────────────────────────────────────────────
    #  Route highlighting
    # ──────────────────────────────────────────────────────────────
    def _get_source_node(self) -> str:
        """Return the best available source node for route search."""
        if self._current_node and self._current_node != 'none':
            return self._current_node
        if self._closest_node and self._closest_node != 'none':
            return self._closest_node
        return 'none'

    def _compute_and_highlight_route(self, target: str):
        """Compute route from current node to target and publish markers."""
        source = self._get_source_node()
        if source == 'none':
            self.get_logger().info(
                'Source node unknown — will retry route highlight'
            )
            self._pending_route_target = target
            if self._route_retry_timer is None:
                self._route_retry_timer = self.create_timer(
                    0.5, self._retry_route_highlight,
                    callback_group=self._cb_group,
                )
            return
        if self._graph is None:
            if self.tmap is not None:
                self._graph = build_graph_from_tmap(
                    self.tmap, logger=self.get_logger(),
                )
            else:
                self.get_logger().warn(
                    'Cannot highlight route — no map loaded'
                )
                return

        route_nodes = plan_route(
            self._graph, source, target,
            logger=self.get_logger(),
        )
        if not route_nodes or len(route_nodes) < 2:
            self.get_logger().warn(
                f'No route found from {source} to {target}'
            )
            self._route_nodes = []
            self._clear_route_highlight()
            return

        self._route_nodes = route_nodes
        self.get_logger().info(
            f'Route highlighted: {" → ".join(self._route_nodes)}'
        )
        self._publish_route_markers()

    def _retry_route_highlight(self):
        """Timer callback: retry route highlighting once source is known."""
        if self._pending_route_target is None or self._navigating_to is None:
            # Navigation finished or cancelled before source appeared
            self._pending_route_target = None
            if self._route_retry_timer is not None:
                self.destroy_timer(self._route_retry_timer)
                self._route_retry_timer = None
            return

        source = self._get_source_node()
        if source == 'none':
            return  # Still unknown — timer will fire again

        target = self._pending_route_target
        self._pending_route_target = None
        if self._route_retry_timer is not None:
            self.destroy_timer(self._route_retry_timer)
            self._route_retry_timer = None

        self._compute_and_highlight_route(target)

    def _publish_route_markers(self):
        """Create and publish route highlight markers."""
        if not self._route_nodes or self.tmap is None:
            return

        marker_array = MarkerArray()
        idn = 0
        scale = self.marker_scale

        # Highlight nodes on the route
        for node_name in self._route_nodes:
            node_data = self._node_entries_by_name.get(node_name)
            if node_data is None:
                continue
            m = Marker()
            m.id = idn
            m.header.frame_id = node_data.get('parent_frame', 'map')
            m.type = Marker.SPHERE
            m.scale.x = scale * 0.7
            m.scale.y = scale * 0.7
            m.scale.z = scale * 0.7
            m.color.a = 0.9
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.pose = _node2pose(node_data['pose'])
            m.pose.position.z += 0.15
            m.ns = '/route_nodes'
            marker_array.markers.append(m)
            idn += 1

        # Highlight edges along the route
        for i in range(len(self._route_nodes) - 1):
            src_name = self._route_nodes[i]
            dst_name = self._route_nodes[i + 1]
            src_data = self._node_entries_by_name.get(src_name)
            dst_data = self._node_entries_by_name.get(dst_name)
            if src_data is None or dst_data is None:
                continue

            m = Marker()
            m.id = idn
            m.header.frame_id = src_data.get('parent_frame', 'map')
            m.type = Marker.LINE_STRIP
            m.pose.orientation.w = 1.0
            m.scale.x = scale * 0.12
            m.color.a = 0.9
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0

            v1 = _node2pose(src_data['pose']).position
            v1.z += 0.15
            v2 = _node2pose(dst_data['pose']).position
            v2.z += 0.15
            m.points.append(v1)
            m.points.append(v2)
            m.ns = '/route_edges'
            marker_array.markers.append(m)
            idn += 1

        # Source node highlight (cyan)
        src_data = self._node_entries_by_name.get(self._route_nodes[0])
        if src_data is not None:
            m = Marker()
            m.id = idn
            m.header.frame_id = src_data.get('parent_frame', 'map')
            m.type = Marker.SPHERE
            m.scale.x = scale * 0.9
            m.scale.y = scale * 0.9
            m.scale.z = scale * 0.9
            m.color.a = 0.7
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.pose = _node2pose(src_data['pose'])
            m.pose.position.z += 0.2
            m.ns = '/route_endpoints'
            marker_array.markers.append(m)
            idn += 1

        # Target node highlight (yellow)
        tgt_data = self._node_entries_by_name.get(self._route_nodes[-1])
        if tgt_data is not None:
            m = Marker()
            m.id = idn
            m.header.frame_id = tgt_data.get('parent_frame', 'map')
            m.type = Marker.SPHERE
            m.scale.x = scale * 0.9
            m.scale.y = scale * 0.9
            m.scale.z = scale * 0.9
            m.color.a = 0.7
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.pose = _node2pose(tgt_data['pose'])
            m.pose.position.z += 0.2
            m.ns = '/route_endpoints'
            marker_array.markers.append(m)
            idn += 1

        # Route label
        if tgt_data is not None:
            m = Marker()
            m.id = idn
            m.header.frame_id = tgt_data.get('parent_frame', 'map')
            m.type = Marker.TEXT_VIEW_FACING
            m.text = f'Route: {self._route_nodes[0]} → {self._route_nodes[-1]}'
            m.pose = _node2pose(tgt_data['pose'])
            m.pose.position.z += 0.6
            m.scale.z = scale * 0.35
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.ns = '/route_label'
            marker_array.markers.append(m)
            idn += 1

        self.route_marker_pub.publish(marker_array)

    def _clear_route_highlight(self):
        """Remove route highlight markers from RViz."""
        self._route_nodes = []
        self._pending_route_target = None
        if self._route_retry_timer is not None:
            self.destroy_timer(self._route_retry_timer)
            self._route_retry_timer = None
        # Publish a DELETE_ALL marker to clear the route topic
        marker_array = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        marker_array.markers.append(m)
        self.route_marker_pub.publish(marker_array)

    # ──────────────────────────────────────────────────────────────
    #  Publishing helpers
    # ──────────────────────────────────────────────────────────────
    def _publish_map_topic(self):
        """Publish the in-memory map to ``/topological_map_2``."""
        if self.tmap is None:
            return
        msg = String()
        msg.data = yaml.dump(
            self.tmap, default_flow_style=False, sort_keys=False
        )
        self.map_topic_pub.publish(msg)
        self.get_logger().info('Published map to /topological_map_2')

    # ──────────────────────────────────────────────────────────────
    #  Build / rebuild all visualisation markers
    # ──────────────────────────────────────────────────────────────
    def _rebuild_visualisation(self):
        """Re-create marker array + interactive markers from ``self.tmap``."""
        if self.tmap is None:
            return

        self._refresh_visual_cache()
        nodes = self.tmap.get('nodes', [])
        marker_array = self._build_full_static_marker_array()

        self.map_marker_pub.publish(marker_array)

        # Interactive editor markers
        if self.edit_mode:
            self._rebuild_interactive_markers(nodes)

        self.get_logger().info(
            f'Visualisation published ({len(nodes)} nodes, '
            f'{len(marker_array.markers)} markers)'
        )

    # ──────────────────────────────────────────────────────────────
    #  Interactive marker layer
    # ──────────────────────────────────────────────────────────────
    def _rebuild_interactive_markers(self, nodes):
        """Create / update interactive markers for every node."""
        # Clear existing markers
        self._im_server.clear()

        for entry in nodes:
            node = entry['node']
            self._create_edit_marker(node)

        # Apply the right-click context menu to every marker, then
        # re-register type-specific callbacks.  MenuHandler.apply()
        # overwrites the default (catch-all) callback, so BUTTON_CLICK
        # and POSE_UPDATE would be silently dropped without this step.
        for entry in nodes:
            name = entry['node']['name']
            self._menu_handler.apply(self._im_server, name)
            self._im_server.setCallback(
                name, self._im_feedback,
                InteractiveMarkerFeedback.BUTTON_CLICK,
            )
            self._im_server.setCallback(
                name, self._im_feedback,
                InteractiveMarkerFeedback.POSE_UPDATE,
            )

        self._im_server.applyChanges()

    def _create_edit_marker(self, node: dict):
        """Insert one interactive marker for *node*."""
        scale = self.marker_scale

        im = InteractiveMarker()
        im.header.frame_id = node.get('parent_frame', 'map')
        im.name = node['name']
        im.description = ''

        pose = _node2pose(node['pose'])
        im.pose = pose

        # ── Visible sphere ──────────────────────────────────────
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = scale
        sphere.scale.y = scale
        sphere.scale.z = scale
        sphere.color.r = 0.0
        sphere.color.g = 0.5
        sphere.color.b = 1.0
        sphere.color.a = 0.8

        vis_ctrl = InteractiveMarkerControl()
        vis_ctrl.always_visible = True
        vis_ctrl.markers.append(sphere)
        vis_ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        vis_ctrl.orientation.w = 1.0
        vis_ctrl.orientation.y = 1.0  # XY plane
        im.controls.append(vis_ctrl)

        # ── Move X ──────────────────────────────────────────────
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.w = 1.0
        ctrl.orientation.x = 1.0
        ctrl.name = 'move_x'
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(ctrl)

        # ── Move Y ──────────────────────────────────────────────
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.w = 1.0
        ctrl.orientation.z = 1.0
        ctrl.name = 'move_y'
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(ctrl)

        # ── Rotate Z (yaw) ─────────────────────────────────────
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.w = 1.0
        ctrl.orientation.y = 1.0
        ctrl.name = 'rotate_z'
        ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(ctrl)

        # ── Arrow showing orientation ───────────────────────────
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = scale * 1.5
        arrow.scale.y = scale * 0.2
        arrow.scale.z = scale * 0.2
        arrow.color.r = 1.0
        arrow.color.a = 1.0

        arrow_ctrl = InteractiveMarkerControl()
        arrow_ctrl.always_visible = True
        arrow_ctrl.markers.append(arrow)
        im.controls.append(arrow_ctrl)

        # ── Text label ──────────────────────────────────────────
        txt = Marker()
        txt.type = Marker.TEXT_VIEW_FACING
        txt.text = node['name']
        txt.scale.z = scale * 0.5
        txt.color.r = 1.0
        txt.color.g = 1.0
        txt.color.b = 1.0
        txt.color.a = 1.0
        txt.pose.position.z = scale * 0.8

        txt_ctrl = InteractiveMarkerControl()
        txt_ctrl.always_visible = True
        txt_ctrl.markers.append(txt)
        im.controls.append(txt_ctrl)

        self._im_server.insert(im)
        self._im_server.setCallback(im.name, self._im_feedback)

    def _im_feedback(self, feedback: InteractiveMarkerFeedback):
        """Handle interactive-marker drag / rotate / click events."""
        # Let MenuHandler process right-click menu selections
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            return

        # Left-click on node sphere → navigate to that node
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            node_name = feedback.marker_name
            self.get_logger().info(
                f'Node clicked → navigating to {node_name}'
            )
            # Cancel any active navigation before sending new goal
            if self._goto_goal_handle is not None:
                self._cancel_navigation()
            self._compute_and_highlight_route(node_name)
            self._send_goto_goal(node_name)
            self._im_server.applyChanges()
            return

        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            self._im_server.applyChanges()
            return

        name = feedback.marker_name
        node = self._node_entries_by_name.get(name)
        if node is None:
            self._refresh_visual_cache()
            node = self._node_entries_by_name.get(name)

        if node is not None:
            node['pose']['position']['x'] = float(
                feedback.pose.position.x
            )
            node['pose']['position']['y'] = float(
                feedback.pose.position.y
            )
            node['pose']['position']['z'] = 0.0

            euler = tf_transformations.euler_from_quaternion([
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w,
            ])
            yaw = euler[2]
            quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
            node['pose']['orientation']['x'] = float(quat[0])
            node['pose']['orientation']['y'] = float(quat[1])
            node['pose']['orientation']['z'] = float(quat[2])
            node['pose']['orientation']['w'] = float(quat[3])

            self._map_dirty = True
            yaw_deg = math.degrees(yaw)
            self.get_logger().debug(
                f'{name}: pos=({feedback.pose.position.x:.2f}, '
                f'{feedback.pose.position.y:.2f}), yaw={yaw_deg:.1f}°'
            )

        # Refresh only the moved node and directly connected edges.
        self._publish_incremental_node_update(name)
        self._im_server.applyChanges()

        # Schedule a debounced republish so downstream nodes
        # (navigation, localisation) pick up the change.
        self._schedule_republish()

    def _schedule_republish(self):
        """Debounce map republish: wait 0.5 s after the last drag event."""
        if self._republish_timer is not None:
            self._republish_timer.cancel()
            self.destroy_timer(self._republish_timer)
        self._republish_timer = self.create_timer(
            0.5, self._deferred_republish,
            callback_group=self._cb_group,
        )

    def _deferred_republish(self):
        """Republish map, rebuild graph, and save to temp folder."""
        # One-shot: cancel the timer immediately
        if self._republish_timer is not None:
            self._republish_timer.cancel()
            self.destroy_timer(self._republish_timer)
            self._republish_timer = None

        # Rebuild the NetworkX graph so route planning uses new positions
        self._graph = build_graph_from_tmap(
            self.tmap, logger=self.get_logger(),
        )

        # Publish updated map to topic so all subscribers get it
        self._publish_map_topic()
        self.get_logger().info(
            'Map republished after node position update'
        )

        # Save to temp folder
        self._save_to_temp()

    def _save_to_temp(self):
        """Save the current map to a temp folder for recovery."""
        if self.tmap is None:
            return
        map_name = self.tmap.get('pointset', 'unknown_map')
        temp_path = os.path.join(
            self._temp_map_dir,
            f'{map_name}.tmap2.yaml',
        )
        try:
            with open(temp_path, 'w') as fh:
                yaml.dump(
                    self.tmap,
                    fh,
                    default_flow_style=False,
                    sort_keys=False,
                )
            self.get_logger().info(f'Map saved to {temp_path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to save temp map: {exc}')

    def _rebuild_static_markers(self):
        """Re-publish static MarkerArray only (no interactive markers)."""
        if self.tmap is None:
            return

        self._refresh_visual_cache()
        marker_array = self._build_full_static_marker_array()
        self.map_marker_pub.publish(marker_array)

    def _build_full_static_marker_array(self) -> MarkerArray:
        """Build the full static marker set and refresh marker IDs."""
        marker_array = MarkerArray()
        self._marker_ids = {
            'nodes': {},
            'names': {},
            'zones': {},
            'edges': {},
            'legend': {},
        }
        idn = 0

        for entry in self.tmap.get('nodes', []):
            node = entry['node']
            node_name = node['name']

            self._marker_ids['nodes'][node_name] = idn
            marker_array.markers.append(self._mk_node(node, idn))
            idn += 1

            self._marker_ids['names'][node_name] = idn
            marker_array.markers.append(self._mk_name(node, idn))
            idn += 1

            if node.get('verts'):
                self._marker_ids['zones'][node_name] = idn
                marker_array.markers.append(self._mk_zone(node, idn))
                idn += 1

            for edge in node.get('edges', []):
                m = self._mk_edge(node, edge)
                if m is None:
                    continue
                edge_key = self._edge_key(node_name, edge)
                self._marker_ids['edges'][edge_key] = idn
                m.id = idn
                marker_array.markers.append(m)
                idn += 1

        for row, action_name in enumerate(self._action_names):
            self._marker_ids['legend'][action_name] = idn
            marker_array.markers.append(
                self._mk_legend(action_name, row, idn)
            )
            idn += 1

        return marker_array

    def _publish_incremental_node_update(self, node_name: str):
        """Publish only the markers affected by a dragged node."""
        node = self._node_entries_by_name.get(node_name)
        if node is None:
            self._rebuild_static_markers()
            return

        marker_array = MarkerArray()
        marker_ids = self._marker_ids
        seen_edges = set()

        node_id = marker_ids['nodes'].get(node_name)
        name_id = marker_ids['names'].get(node_name)
        if node_id is None or name_id is None:
            self._rebuild_static_markers()
            return

        marker_array.markers.append(self._mk_node(node, node_id))
        marker_array.markers.append(self._mk_name(node, name_id))

        zone_id = marker_ids['zones'].get(node_name)
        if node.get('verts') and zone_id is not None:
            marker_array.markers.append(self._mk_zone(node, zone_id))

        for edge in node.get('edges', []):
            edge_key = self._edge_key(node_name, edge)
            edge_id = marker_ids['edges'].get(edge_key)
            if edge_id is None:
                continue
            marker = self._mk_edge(node, edge)
            if marker is None:
                continue
            marker.id = edge_id
            marker_array.markers.append(marker)
            seen_edges.add(edge_key)

        for source_node, edge in self._incoming_edges_by_target.get(node_name, []):
            source_name = source_node['name']
            edge_key = self._edge_key(source_name, edge)
            if edge_key in seen_edges:
                continue
            edge_id = marker_ids['edges'].get(edge_key)
            if edge_id is None:
                continue
            marker = self._mk_edge(source_node, edge)
            if marker is None:
                continue
            marker.id = edge_id
            marker_array.markers.append(marker)
            seen_edges.add(edge_key)

        self.map_marker_pub.publish(marker_array)

        if self._route_nodes and node_name in self._route_nodes:
            self._publish_route_markers()

    # ──────────────────────────────────────────────────────────────
    #  Marker factory helpers
    # ──────────────────────────────────────────────────────────────
    def _mk_node(self, node: dict, idn: int) -> Marker:
        m = Marker()
        m.id = idn
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.SPHERE
        s = self.marker_scale
        m.scale.x = s * 0.4
        m.scale.y = s * 0.4
        m.scale.z = s * 0.4
        m.color.a = 0.4
        m.color.r = 0.2
        m.color.g = 0.2
        m.color.b = 0.7
        m.pose = _node2pose(node['pose'])
        m.pose.position.z += 0.1
        m.ns = '/nodes'
        return m

    def _mk_name(self, node: dict, idn: int) -> Marker:
        m = Marker()
        m.id = idn
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.TEXT_VIEW_FACING
        m.text = node['name']
        m.pose = _node2pose(node['pose'])
        m.pose.position.z += 0.25
        m.scale.z = self.marker_scale * 0.24
        m.color.a = 0.9
        m.color.r = 0.3
        m.color.g = 0.3
        m.color.b = 0.3
        m.ns = '/names'
        return m

    def _mk_zone(self, node: dict, idn: int) -> Marker:
        m = Marker()
        m.id = idn
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.LINE_STRIP
        m.pose.orientation.w = 1.0
        m.scale.x = self.marker_scale * 0.06
        m.color.a = 0.8
        m.color.r = 0.7
        m.color.g = 0.1
        m.color.b = 0.2

        px = float(node['pose']['position']['x'])
        py = float(node['pose']['position']['y'])
        pz = float(node['pose']['position']['z'])

        for v in node['verts']:
            pt = Point()
            pt.x = px + float(v['x'])
            pt.y = py + float(v['y'])
            pt.z = pz
            m.points.append(pt)

        # Close the polygon
        first = node['verts'][0]
        pt = Point()
        pt.x = px + float(first['x'])
        pt.y = py + float(first['y'])
        pt.z = pz
        m.points.append(pt)
        m.ns = '/zones'
        return m

    def _mk_edge(
        self, node: dict, edge: dict
    ) -> Marker | None:
        to_node = self._node_entries_by_name.get(edge['node'])
        if to_node is None:
            self.get_logger().warn(
                f"Edge target node '{edge['node']}' not found"
            )
            return None

        action = edge.get('action', '')
        col_idx = self._action_index.get(action, 0)
        col = _colour(col_idx)

        m = Marker()
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.LINE_LIST

        v1 = _node2pose(node['pose']).position
        v1.z += 0.1
        v2 = _node2pose(to_node['pose']).position
        v2.z += 0.1

        m.pose.orientation.w = 1.0
        m.scale.x = self.marker_scale * 0.06
        m.color.a = 0.5
        m.color.r = float(col[0])
        m.color.g = float(col[1])
        m.color.b = float(col[2])
        m.points.append(v1)
        m.points.append(v2)
        m.ns = '/edges'
        return m

    def _mk_legend(
        self, action: str, row: int, idn: int
    ) -> Marker:
        col_idx = self._action_index.get(action, 0)
        col = _colour(col_idx)
        m = Marker()
        m.id = idn
        m.header.frame_id = 'map'
        m.type = Marker.TEXT_VIEW_FACING
        m.text = action
        m.pose.position.x = 1.0
        m.pose.position.y = 0.18 * row
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.z = self.marker_scale * 0.3
        m.color.a = 1.0
        m.color.r = float(col[0])
        m.color.g = float(col[1])
        m.color.b = float(col[2])
        m.ns = '/legend'
        return m


# ══════════════════════════════════════════════════════════════════
#  Entry-point
# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = TopologicalMapVisualiser()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down topological map visualiser')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
