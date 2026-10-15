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

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy

import yaml
import tf_transformations

from geometry_msgs.msg import Point, Pose
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
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

from topological_navigation_visual import viz_utils

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


def _node_xyz(node, z_offset: float = 0.0):
    """Return a node's ``(x, y, z)`` position from its tmap2 dict."""
    pos = node['pose']['position']
    return (
        float(pos['x']),
        float(pos['y']),
        float(pos.get('z', 0.0)) + z_offset,
    )


# ══════════════════════════════════════════════════════════════════
#  Main visualiser node
# ══════════════════════════════════════════════════════════════════
class TopologicalMapVisualiser(Node):
    """Unified map visualiser with optional interactive editing."""

    def __init__(self):
        super().__init__('topological_map_visualiser')

        # ── Parameters ──────────────────────────────────────────
        # ``map_file`` and ``nav_action_name`` are start-up only.
        self.declare_parameter(
            'map_file', '',
            ParameterDescriptor(
                description='Path to a .tmap2.yaml file. Empty = subscribe '
                            'to /topological_map_2 (start-up only).',
                read_only=True,
            ),
        )
        self.declare_parameter(
            'nav_action_name', '/topological_navigation',
            ParameterDescriptor(
                description='GotoNode action server for click-to-navigate '
                            '(start-up only).',
                read_only=True,
            ),
        )

        # The following parameters can all be changed at runtime via
        # ``ros2 param set`` (see ``_on_set_parameters``).
        self.declare_parameter(
            'auto_save', False,
            ParameterDescriptor(
                description='Periodically save the map to file every 30 s.',
            ),
        )
        self.declare_parameter(
            'marker_scale', 0.5,
            ParameterDescriptor(
                description='Base scale factor for RViz markers.',
                floating_point_range=[FloatingPointRange(
                    from_value=float(viz_utils.MIN_SCALE),
                    to_value=float(viz_utils.MAX_SCALE),
                    step=0.0,
                )],
            ),
        )
        self.declare_parameter(
            'edit_mode', True,
            ParameterDescriptor(
                description='Enable interactive drag-and-drop node editing.',
            ),
        )
        self.declare_parameter(
            'show_node_labels', True,
            ParameterDescriptor(
                description='Render per-node text labels. Disable for large '
                            'maps — text markers are the most expensive '
                            'RViz primitive.',
            ),
        )
        self.declare_parameter(
            'show_zones', True,
            ParameterDescriptor(
                description='Render node influence-zone polygons.',
            ),
        )
        self.declare_parameter(
            'show_edges', True,
            ParameterDescriptor(
                description='Render edges between nodes.',
            ),
        )
        self.declare_parameter(
            'auto_marker_scale', False,
            ParameterDescriptor(
                description='Derive marker_scale automatically from the '
                            'spatial spread of the map.',
            ),
        )
        self.declare_parameter(
            'interactive_marker_limit', 750,
            ParameterDescriptor(
                description='Maximum node count for which interactive '
                            '(editable) markers are created. Above this the '
                            'map is shown read-only to keep RViz responsive.',
                integer_range=[IntegerRange(
                    from_value=0, to_value=1_000_000, step=1,
                )],
            ),
        )

        self.map_file: str = self.get_parameter('map_file').value
        nav_action: str = self.get_parameter('nav_action_name').value
        self.auto_save: bool = self.get_parameter('auto_save').value
        self.edit_mode: bool = self.get_parameter('edit_mode').value
        self.show_node_labels: bool = self.get_parameter(
            'show_node_labels').value
        self.show_zones: bool = self.get_parameter('show_zones').value
        self.show_edges: bool = self.get_parameter('show_edges').value
        self.auto_marker_scale: bool = self.get_parameter(
            'auto_marker_scale').value
        self.interactive_marker_limit: int = self.get_parameter(
            'interactive_marker_limit').value

        # ``_base_marker_scale`` is the user-requested scale; ``marker_scale``
        # is the *effective* scale (possibly auto-computed) used by markers.
        self._base_marker_scale: float = self.get_parameter(
            'marker_scale').value
        self.marker_scale: float = self._base_marker_scale

        # Register the runtime parameter callback (ROS 2 Humble compatible).
        self._param_apply_timer = None
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ── State ────────────────────────────────────────────────
        self.tmap = None
        self._graph = None
        self._map_dirty = False
        self._node_entries_by_name: dict[str, dict] = {}
        self._action_names: list[str] = []
        self._action_index: dict[str, int] = {}
        # Names of source nodes for every edge that *terminates* at a node,
        # keyed by the target node name. Lets an incremental update redraw
        # only the edge markers whose endpoint moved.
        self._incoming_edges_by_target: dict[str, list] = {}
        # Stable RViz marker ids assigned per namespace, keyed by node name
        # (or action name for the legend). Reused by incremental updates so a
        # single dragged node republishes the same ids instead of the map.
        self._marker_ids: dict[str, dict] = {
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
        # Always create the timer when a file is available; the callback
        # gates on ``self.auto_save`` so it can be toggled at runtime.
        if self.map_file:
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
        self._action_names = []
        self._action_index = {}
        self._incoming_edges_by_target = {}

        if self.tmap is None:
            return

        seen_actions = set()
        nodes = self.tmap.get('nodes', [])
        for entry in nodes:
            node = entry['node']
            self._node_entries_by_name[node['name']] = node

        for entry in nodes:
            node = entry['node']
            src_name = node['name']
            for edge in node.get('edges', []):
                action = edge.get('action', '')
                if action and action not in seen_actions:
                    seen_actions.add(action)
                    self._action_names.append(action)
                target = edge.get('node')
                if target:
                    self._incoming_edges_by_target.setdefault(
                        target, []
                    ).append(src_name)

        self._action_index = {
            action: idx for idx, action in enumerate(self._action_names)
        }

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
        # Clear first: a new map may have fewer nodes/edges than before.
        self._rebuild_visualisation(clear_first=True)
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
        if self.auto_save and self.map_file and self._map_dirty:
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
    #  Runtime (dynamic) parameter handling — ROS 2 Humble compatible
    # ──────────────────────────────────────────────────────────────
    def _on_set_parameters(self, params):
        """Validate and apply parameter updates received at runtime.

        Registered via ``add_on_set_parameters_callback`` (the only
        runtime-parameter hook available in ROS 2 Humble).  The callback
        runs *before* the new values are committed, so we validate here
        and read the new values directly from the ``params`` list.  The
        (potentially expensive) marker rebuild is deferred to a one-shot
        timer so it never blocks the parameter-set transaction.
        """
        needs_rebuild = False

        for p in params:
            name = p.name

            if name == 'marker_scale':
                if p.type_ not in (
                    Parameter.Type.DOUBLE, Parameter.Type.INTEGER,
                ):
                    return SetParametersResult(
                        successful=False,
                        reason='marker_scale must be a number',
                    )
                value = float(p.value)
                if not (viz_utils.MIN_SCALE <= value <= viz_utils.MAX_SCALE):
                    return SetParametersResult(
                        successful=False,
                        reason='marker_scale out of range [%g, %g]'
                        % (viz_utils.MIN_SCALE, viz_utils.MAX_SCALE),
                    )
                self._base_marker_scale = value
                needs_rebuild = True

            elif name == 'auto_marker_scale':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='auto_marker_scale must be a boolean',
                    )
                self.auto_marker_scale = bool(p.value)
                needs_rebuild = True

            elif name == 'show_node_labels':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='show_node_labels must be a boolean',
                    )
                self.show_node_labels = bool(p.value)
                needs_rebuild = True

            elif name == 'show_zones':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='show_zones must be a boolean',
                    )
                self.show_zones = bool(p.value)
                needs_rebuild = True

            elif name == 'show_edges':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='show_edges must be a boolean',
                    )
                self.show_edges = bool(p.value)
                needs_rebuild = True

            elif name == 'edit_mode':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='edit_mode must be a boolean',
                    )
                self.edit_mode = bool(p.value)
                needs_rebuild = True

            elif name == 'interactive_marker_limit':
                if p.type_ != Parameter.Type.INTEGER:
                    return SetParametersResult(
                        successful=False,
                        reason='interactive_marker_limit must be an integer',
                    )
                if int(p.value) < 0:
                    return SetParametersResult(
                        successful=False,
                        reason='interactive_marker_limit must be >= 0',
                    )
                self.interactive_marker_limit = int(p.value)
                needs_rebuild = True

            elif name == 'auto_save':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='auto_save must be a boolean',
                    )
                self.auto_save = bool(p.value)

        if needs_rebuild:
            self._schedule_param_apply()

        return SetParametersResult(successful=True)

    def _schedule_param_apply(self):
        """Defer a full rebuild so it runs outside the param transaction."""
        if self._param_apply_timer is not None:
            self._param_apply_timer.cancel()
            self.destroy_timer(self._param_apply_timer)
        self._param_apply_timer = self.create_timer(
            0.05, self._apply_param_changes,
            callback_group=self._cb_group,
        )

    def _apply_param_changes(self):
        """Recompute the effective scale and rebuild the visualisation."""
        if self._param_apply_timer is not None:
            self._param_apply_timer.cancel()
            self.destroy_timer(self._param_apply_timer)
            self._param_apply_timer = None

        self._recompute_effective_scale()
        # Full rebuild with a clear so toggled-off layers are removed.
        self._rebuild_visualisation(clear_first=True)

    def _recompute_effective_scale(self):
        """Update ``self.marker_scale`` from the base value / auto mode."""
        if self.auto_marker_scale and self.tmap is not None:
            positions = viz_utils.collect_node_positions(
                self.tmap.get('nodes', [])
            )
            self.marker_scale = viz_utils.compute_auto_scale(
                positions, fallback=self._base_marker_scale,
            )
            self.get_logger().info(
                'Auto marker scale -> %.3f (%d nodes)'
                % (self.marker_scale, len(positions))
            )
        else:
            self.marker_scale = self._base_marker_scale

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
    def _rebuild_visualisation(self, clear_first: bool = False):
        """Re-create marker array + interactive markers from ``self.tmap``."""
        if self.tmap is None:
            return

        self._recompute_effective_scale()
        self._refresh_visual_cache()
        nodes = self.tmap.get('nodes', [])
        marker_array = self._build_full_static_marker_array(
            clear_first=clear_first,
        )

        self.map_marker_pub.publish(marker_array)

        # Interactive editor markers (or clear them when editing is off).
        if self.edit_mode:
            self._rebuild_interactive_markers(nodes)
        else:
            self._clear_interactive_markers()

        self.get_logger().info(
            f'Visualisation published ({len(nodes)} nodes, '
            f'{len(marker_array.markers)} markers, '
            f'scale={self.marker_scale:.3f})'
        )

    # ──────────────────────────────────────────────────────────────
    #  Interactive marker layer
    # ──────────────────────────────────────────────────────────────
    def _rebuild_interactive_markers(self, nodes):
        """Create / update interactive markers for every node.

        For very large maps creating an interactive marker per node makes
        RViz unresponsive, so above ``interactive_marker_limit`` the map is
        shown read-only (static markers only) and a warning is logged.
        """
        # Clear existing markers
        self._im_server.clear()

        if len(nodes) > self.interactive_marker_limit:
            self._im_server.applyChanges()
            self.get_logger().warning(
                'Map has %d nodes (> interactive_marker_limit=%d): '
                'showing read-only. Raise the limit or disable edit_mode '
                'to suppress this message.'
                % (len(nodes), self.interactive_marker_limit)
            )
            return

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

    def _clear_interactive_markers(self):
        """Remove all interactive markers (used when edit_mode is off)."""
        self._im_server.clear()
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

            # Republish only the markers touched by this move so the
            # visualisation stays responsive even on very large maps.
            self._publish_incremental_node_update(name)

            # Keep the route highlight aligned with the dragged node.
            if self._route_nodes:
                self._publish_route_markers()

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

    def _map_frame(self) -> str:
        """Resolve a single frame id for the batched markers.

        All nodes in a tmap share the same frame in practice, so we use the
        first node's ``parent_frame`` (default ``'map'``).
        """
        if self.tmap is not None:
            for entry in self.tmap.get('nodes', []):
                return entry.get('node', {}).get('parent_frame', 'map')
        return 'map'

    def _rebuild_static_markers(self, clear_first: bool = False):
        """Re-publish static MarkerArray only (no interactive markers)."""
        if self.tmap is None:
            return

        self._refresh_visual_cache()
        marker_array = self._build_full_static_marker_array(
            clear_first=clear_first,
        )
        self.map_marker_pub.publish(marker_array)

    def _build_full_static_marker_array(
        self, clear_first: bool = False,
    ) -> MarkerArray:
        """Build the static marker set for the whole map (per node).

        Each node owns its own markers so a single dragged node can be
        republished on its own (see
        :meth:`_publish_incremental_node_update`) instead of rebuilding the
        entire map. Per namespace a node contributes at most one marker,
        all sharing the node's index as a stable id:

        * node sphere -> ns ``/nodes``  (one per node)
        * label       -> ns ``/names``  (one per node, optional)
        * zone        -> ns ``/zones``  (one per node with a polygon)
        * edges       -> ns ``/edges``  (one ``LINE_LIST`` of the node's
          outgoing edges, optional)
        * legend      -> ns ``/legend`` (one ``TEXT`` per action)

        The assigned ids are cached in ``self._marker_ids`` so incremental
        updates reuse them. When *clear_first* is set a ``DELETEALL`` is
        prepended so stale markers (toggled-off layers or deleted nodes)
        are removed cleanly.
        """
        marker_array = MarkerArray()
        if clear_first:
            clear = Marker()
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)

        # Reset the id cache — full rebuilds reassign ids by node order.
        self._marker_ids = {
            'nodes': {},
            'names': {},
            'zones': {},
            'edges': {},
            'legend': {},
        }

        show_labels = getattr(self, 'show_node_labels', True)
        show_zones = getattr(self, 'show_zones', True)
        show_edges = getattr(self, 'show_edges', True)

        nodes = self.tmap.get('nodes', [])
        for i, entry in enumerate(nodes):
            node = entry['node']
            name = node['name']
            frame = node.get('parent_frame', 'map')

            marker_array.markers.append(
                self._mk_node_sphere(node, i, frame)
            )
            self._marker_ids['nodes'][name] = i

            if show_labels:
                marker_array.markers.append(self._mk_name(node, i))
                self._marker_ids['names'][name] = i

            if show_zones:
                zone_marker = self._mk_node_zone(node, i, frame)
                if zone_marker is not None:
                    marker_array.markers.append(zone_marker)
                    self._marker_ids['zones'][name] = i

            if show_edges:
                edge_marker = self._mk_node_edges(node, i, frame)
                if edge_marker is not None:
                    marker_array.markers.append(edge_marker)
                    self._marker_ids['edges'][name] = i

        # Legend -> one TEXT marker per action.
        for row, action_name in enumerate(self._action_names):
            marker_array.markers.append(
                self._mk_legend(action_name, row, row)
            )
            self._marker_ids['legend'][action_name] = row

        return marker_array

    def _publish_incremental_node_update(self, name: str):
        """Republish only the markers affected when *name* moves.

        Touches the node's own sphere, label, zone, and outgoing-edge
        markers, plus the edge markers of any node with an edge that
        terminates at *name* (so the moved endpoint follows). Relies solely
        on the cached lookups, so it never scans the whole node list. Falls
        back to a full static rebuild if the node has no cached marker id
        yet (e.g. a node added since the last full rebuild).
        """
        node = self._node_entries_by_name.get(name)
        if node is None:
            return

        node_id = self._marker_ids['nodes'].get(name)
        if node_id is None:
            self._rebuild_static_markers()
            return

        show_labels = getattr(self, 'show_node_labels', True)
        show_zones = getattr(self, 'show_zones', True)
        show_edges = getattr(self, 'show_edges', True)
        frame = node.get('parent_frame', 'map')

        marker_array = MarkerArray()
        marker_array.markers.append(
            self._mk_node_sphere(node, node_id, frame)
        )

        if show_labels:
            label_id = self._marker_ids['names'].get(name, node_id)
            marker_array.markers.append(self._mk_name(node, label_id))

        if show_zones:
            zone_id = self._marker_ids['zones'].get(name, node_id)
            zone_marker = self._mk_node_zone(node, zone_id, frame)
            if zone_marker is not None:
                marker_array.markers.append(zone_marker)

        if show_edges:
            edge_id = self._marker_ids['edges'].get(name, node_id)
            edge_marker = self._mk_node_edges(node, edge_id, frame)
            if edge_marker is not None:
                marker_array.markers.append(edge_marker)

            # Redraw the edge markers belonging to nodes that point *at*
            # this one so their endpoint tracks the move.
            for src_name in self._incoming_edges_by_target.get(name, []):
                if src_name == name:
                    continue
                src_node = self._node_entries_by_name.get(src_name)
                src_id = self._marker_ids['edges'].get(src_name)
                if src_node is None or src_id is None:
                    continue
                src_frame = src_node.get('parent_frame', 'map')
                src_marker = self._mk_node_edges(src_node, src_id, src_frame)
                if src_marker is not None:
                    marker_array.markers.append(src_marker)

        self.map_marker_pub.publish(marker_array)

    # ──────────────────────────────────────────────────────────────
    #  Marker factory helpers (per node)
    # ──────────────────────────────────────────────────────────────
    def _mk_node_sphere(self, node: dict, idn: int, frame: str) -> Marker:
        """Build a single ``SPHERE`` marker for one node."""
        m = Marker()
        m.id = idn
        m.ns = '/nodes'
        m.header.frame_id = frame
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = _node2pose(node['pose'])
        m.pose.position.z += 0.1
        s = self.marker_scale * 0.4
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s
        m.color.a = 0.6
        m.color.r = 0.2
        m.color.g = 0.2
        m.color.b = 0.7
        return m

    def _mk_node_zone(self, node: dict, idn: int, frame: str):
        """Build a ``LINE_LIST`` marker for one node's influence zone.

        Returns ``None`` when the node has no usable polygon.
        """
        verts = node.get('verts') or []
        if len(verts) < 2:
            return None
        bx, by, bz = _node_xyz(node)
        points = []
        for v in verts:
            try:
                points.append((bx + float(v['x']), by + float(v['y']), bz))
            except (KeyError, TypeError, ValueError):
                return None
        if len(points) < 2:
            return None
        m = Marker()
        m.id = idn
        m.ns = '/zones'
        m.header.frame_id = frame
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = max(self.marker_scale * 0.06, 0.02)
        m.color.a = 0.8
        m.color.r = 0.7
        m.color.g = 0.1
        m.color.b = 0.2
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            m.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
            m.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))
        return m

    def _mk_node_edges(self, node: dict, idn: int, frame: str):
        """Build a ``LINE_LIST`` of one node's outgoing edges.

        Edges whose target is missing from the map are skipped, and the
        marker colour follows the first valid edge's action. Returns
        ``None`` when the node has no drawable outgoing edge.
        """
        src = _node_xyz(node, z_offset=0.1)
        segments = []
        colour_action = ''
        for edge in node.get('edges', []) or []:
            target = edge.get('node')
            tnode = self._node_entries_by_name.get(target)
            if tnode is None:
                continue
            segments.append((src, _node_xyz(tnode, z_offset=0.1)))
            if not colour_action:
                colour_action = edge.get('action', '') or ''
        if not segments:
            return None
        col = _colour(self._action_index.get(colour_action, idn))
        m = Marker()
        m.id = idn
        m.ns = '/edges'
        m.header.frame_id = frame
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = max(self.marker_scale * 0.06, 0.02)
        m.color.a = 0.6
        m.color.r = float(col[0])
        m.color.g = float(col[1])
        m.color.b = float(col[2])
        for p1, p2 in segments:
            m.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
            m.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))
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
