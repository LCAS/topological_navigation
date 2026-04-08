"""
Navigation graph utilities using NetworkX.

Provides graph-based route planning, state machine, action segment merging,
and boundary polygon computation for topological navigation.

Key Features:
    - Navigation state machine with validated transitions
    - Route planning via NetworkX A* shortest path
    - Action segment merging for consecutive same-type edges
    - Boundary polygon computation for RowTraversal edges
    - Route distance calculation

Dependencies:
    - networkx (>=2.5): Graph data structures and algorithms
    - numpy (>=1.19): Numerical operations
"""

import math
import networkx as nx
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from threading import Lock
from typing import List, Dict, Optional, Tuple, Any


# ==============================================================================
# State Machine
# ==============================================================================

class NavState(Enum):
    """Navigation state machine states.

    Lifecycle:
        IDLE -> WAITING_FOR_MAP -> WAITING_FOR_LOCALISATION -> READY
        READY -> PLANNING -> EXECUTING_* -> SUCCEEDED / FAILED / CANCELLED
        SUCCEEDED / FAILED / CANCELLED -> READY
    """

    IDLE = "IDLE"
    WAITING_FOR_MAP = "WAITING_FOR_MAP"
    WAITING_FOR_LOCALISATION = "WAITING_FOR_LOCALISATION"
    READY = "READY"
    PLANNING = "PLANNING"
    EXECUTING_NAVIGATE_TO_POSE = "EXECUTING_NAVIGATE_TO_POSE"
    EXECUTING_GOAL_ALIGN = "EXECUTING_GOAL_ALIGN"
    EXECUTING_ROW_TRAVERSAL = "EXECUTING_ROW_TRAVERSAL"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


_EXECUTING_STATES = {
    NavState.EXECUTING_NAVIGATE_TO_POSE,
    NavState.EXECUTING_GOAL_ALIGN,
    NavState.EXECUTING_ROW_TRAVERSAL,
}

_TERMINAL_STATES = {
    NavState.SUCCEEDED,
    NavState.FAILED,
    NavState.CANCELLED,
}

_EXEC_AND_TERMINAL = _EXECUTING_STATES | _TERMINAL_STATES

VALID_TRANSITIONS: Dict[NavState, set] = {
    NavState.IDLE: {NavState.WAITING_FOR_MAP},
    NavState.WAITING_FOR_MAP: {NavState.WAITING_FOR_LOCALISATION},
    NavState.WAITING_FOR_LOCALISATION: {NavState.READY},
    NavState.READY: {NavState.PLANNING, NavState.CANCELLED},
    NavState.PLANNING: (
        _EXECUTING_STATES | {NavState.SUCCEEDED, NavState.FAILED,
                             NavState.CANCELLED}
    ),
    NavState.EXECUTING_NAVIGATE_TO_POSE: (
        _EXEC_AND_TERMINAL
    ),
    NavState.EXECUTING_GOAL_ALIGN: (
        _EXEC_AND_TERMINAL
    ),
    NavState.EXECUTING_ROW_TRAVERSAL: (
        _EXEC_AND_TERMINAL
    ),
    NavState.SUCCEEDED: {NavState.READY},
    NavState.FAILED: {NavState.READY},
    NavState.CANCELLED: {NavState.READY},
}


ACTION_TO_STATE: Dict[str, NavState] = {
    "NavigateToPose": NavState.EXECUTING_NAVIGATE_TO_POSE,
    "navigate_to_pose": NavState.EXECUTING_NAVIGATE_TO_POSE,
    "navigate_to_pose_goal_align": NavState.EXECUTING_GOAL_ALIGN,
    "goal_align": NavState.EXECUTING_GOAL_ALIGN,
    "GoalAlign": NavState.EXECUTING_GOAL_ALIGN,
    "row_traversal": NavState.EXECUTING_ROW_TRAVERSAL,
    "RowTraversal": NavState.EXECUTING_ROW_TRAVERSAL,
}


# ==============================================================================
# Action Name Normalisation (dual naming convention)
# ==============================================================================

CANONICAL_ACTIONS: Dict[str, str] = {
    # CamelCase forms
    'NavigateToPose': 'NavigateToPose',
    'RowTraversal': 'row_traversal',
    'GoalAlign': 'goal_align',
    # snake_case forms
    'navigate_to_pose': 'NavigateToPose',
    'row_traversal': 'row_traversal',
    'goal_align': 'goal_align',
}


def normalize_action_name(name: str) -> str:
    """Normalize edge action name to its canonical form.

    Both CamelCase and snake_case conventions are accepted::

        'NavigateToPose' / 'navigate_to_pose' -> 'NavigateToPose'
        'RowTraversal'   / 'row_traversal'    -> 'row_traversal'
        'GoalAlign'      / 'goal_align'       -> 'goal_align'

    Unknown action names are returned unchanged.

    Args:
        name: Edge action name in either convention.

    Returns:
        Canonical action name string.
    """
    return CANONICAL_ACTIONS.get(name, name)


class NavStateMachine:
    """Navigation state machine with validated transitions.

    Thread-safe via internal lock. All state reads/writes are atomic.
    Invalid transitions are rejected with a warning log.

    Example:
        >>> sm = NavStateMachine()
        >>> sm.transition(NavState.WAITING_FOR_MAP)
        True
        >>> sm.state
        <NavState.WAITING_FOR_MAP: 'WAITING_FOR_MAP'>
        >>> sm.transition(NavState.READY)  # invalid
        False
    """

    def __init__(self, logger=None):
        self._state = NavState.IDLE
        self._logger = logger
        self._lock = Lock()

    @property
    def state(self) -> NavState:
        """Current state (thread-safe read)."""
        with self._lock:
            return self._state

    def transition(self, new_state: NavState) -> bool:
        """Attempt state transition.

        Args:
            new_state: Target state.

        Returns:
            True if transition succeeded, False if invalid.
        """
        with self._lock:
            old = self._state
            valid = VALID_TRANSITIONS.get(old, set())

            if new_state not in valid:
                if self._logger:
                    self._logger.warning(
                        f"Invalid state transition: {old.value} -> "
                        f"{new_state.value}. Valid: "
                        f"{[s.value for s in sorted(valid, key=lambda s: s.value)]}"
                    )
                return False

            self._state = new_state
            if self._logger:
                self._logger.info(
                    f"[STATE] {old.value} -> {new_state.value}"
                )
            return True

    def is_executing(self) -> bool:
        """True if currently executing any action."""
        return self.state in _EXECUTING_STATES

    def is_terminal(self) -> bool:
        """True if in a terminal state (SUCCEEDED/FAILED/CANCELLED)."""
        return self.state in _TERMINAL_STATES

    def reset(self) -> bool:
        """Reset to READY from a terminal state."""
        return self.transition(NavState.READY)


# ==============================================================================
# Action Segments
# ==============================================================================

@dataclass
class ActionSegment:
    """A batch of consecutive edges sharing the same action type.

    When a route contains consecutive edges with the same action,
    they are merged into a single ActionSegment for efficient
    batch execution and unified boundary publishing.

    Attributes:
        action_type: Edge action name ('NavigateToPose', 'goal_align',
            'row_traversal').
        edge_ids: List of edge IDs in this segment.
        source_nodes: List of source node names.
        target_nodes: List of target node names.
        edge_data: List of full edge attribute dicts from the graph.
    """

    action_type: str
    edge_ids: List[str] = field(default_factory=list)
    source_nodes: List[str] = field(default_factory=list)
    target_nodes: List[str] = field(default_factory=list)
    edge_data: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def is_empty(self) -> bool:
        """True if segment contains no edges."""
        return len(self.edge_ids) == 0

    @property
    def first_source(self) -> Optional[str]:
        """Name of the first source node, or None."""
        return self.source_nodes[0] if self.source_nodes else None

    @property
    def last_target(self) -> Optional[str]:
        """Name of the last target node, or None."""
        return self.target_nodes[-1] if self.target_nodes else None

    @property
    def num_edges(self) -> int:
        """Number of edges in this segment."""
        return len(self.edge_ids)


# ==============================================================================
# Route Planning
# ==============================================================================

def _heuristic(u: str, v: str, graph: nx.DiGraph) -> float:
    """Euclidean distance heuristic for A*."""
    ux, uy = graph.nodes[u]['x'], graph.nodes[u]['y']
    vx, vy = graph.nodes[v]['x'], graph.nodes[v]['y']
    return math.hypot(vx - ux, vy - uy)


def plan_route(
    graph: nx.DiGraph,
    origin: str,
    target: str,
    avoid_edges: Optional[List[str]] = None,
    logger=None,
    algorithm: str = 'astar',
    weight: str = 'weight',
) -> Optional[List[str]]:
    """Plan shortest route using NetworkX algorithms.

    Supports multiple path-finding algorithms selectable at runtime
    via the ``algorithm`` parameter:

    - ``'astar'`` (default): A* with Euclidean distance heuristic.
      Optimal and efficient for spatial graphs.
    - ``'dijkstra'``: Dijkstra's algorithm.  No heuristic -- explores
      more nodes but handles non-spatial weights better.

    The ``weight`` parameter selects which edge attribute is used as
    the path cost.  By default this is ``'weight'`` (set to 1.0
    during graph construction but overridable via edge properties).

    Args:
        graph: NetworkX DiGraph from build_graph_from_tmap().
        origin: Name of the origin node.
        target: Name of the target node.
        avoid_edges: Optional list of edge_ids to exclude.
        logger: Optional ROS 2 logger.
        algorithm: ``'astar'`` or ``'dijkstra'``.
        weight: Edge attribute used as cost (default ``'weight'``).

    Returns:
        Ordered list of node names [origin, ..., target],
        or None if no route exists.
    """
    if origin == target:
        return [origin]

    if origin not in graph:
        if logger:
            logger.error(f"Origin node '{origin}' not in graph")
        return None

    if target not in graph:
        if logger:
            logger.error(f"Target node '{target}' not in graph")
        return None

    # Optionally filter out avoided edges
    if avoid_edges:
        avoid_set = set(avoid_edges)
        view = nx.subgraph_view(
            graph,
            filter_edge=lambda u, v: (
                graph[u][v].get('edge_id', '') not in avoid_set
            ),
        )
    else:
        view = graph

    try:
        if algorithm == 'dijkstra':
            path = nx.dijkstra_path(
                view, origin, target, weight=weight,
            )
        else:
            # Default: A* with Euclidean heuristic
            path = nx.astar_path(
                view, origin, target,
                heuristic=lambda u, v: _heuristic(u, v, graph),
                weight=weight,
            )
        return path
    except nx.NetworkXNoPath:
        if logger:
            logger.warning(f"No route from '{origin}' to '{target}'")
        return None
    except nx.NodeNotFound as exc:
        if logger:
            logger.error(f"Node not found during planning: {exc}")
        return None


def get_route_edges(
    graph: nx.DiGraph,
    route_nodes: List[str],
) -> List[Dict[str, Any]]:
    """Extract edge data for consecutive nodes along a route.

    Args:
        graph: NetworkX DiGraph with edge attributes.
        route_nodes: Ordered node names [origin, ..., target].

    Returns:
        List of edge data dicts. Each contains at minimum:
        ``edge_id``, ``source``, ``target``, ``action``,
        ``action_type``, ``properties``, ``weight``.
    """
    edges: List[Dict[str, Any]] = []
    for i in range(len(route_nodes) - 1):
        src = route_nodes[i]
        tgt = route_nodes[i + 1]

        if graph.has_edge(src, tgt):
            attrs = dict(graph[src][tgt])
            attrs['source'] = src
            attrs['target'] = tgt
            edges.append(attrs)

    return edges


def merge_action_segments(
    route_edges: List[Dict[str, Any]],
    map_actions: Optional[Dict[str, Any]] = None,
) -> List[ActionSegment]:
    """Merge consecutive same-action-type edges into segments.

    Given edges: [Nav, Nav, Row, Row, GoalAlign]
    Produces:    [Seg(Nav,2), Seg(Row,2), Seg(GoalAlign,1)]

    When ``map_actions`` is provided the ``composable`` flag from
    each action configuration controls merging:

    - ``composable: true`` -- consecutive same-action edges are
      merged into a single multi-waypoint segment.
    - ``composable: false`` -- each edge becomes its own segment
      even when the action name matches the previous edge.

    If ``map_actions`` is *None* (legacy maps without an ``actions``
    section), all edges are merged by action name as before.

    Args:
        route_edges: Edge data dicts from :func:`get_route_edges`.
        map_actions: Optional ``actions`` dict from the topological
            map YAML.  Keys are action names, values are config
            dicts with at least a ``composable`` boolean.

    Returns:
        List of ActionSegment instances with merged edges.
    """
    if not route_edges:
        return []

    segments: List[ActionSegment] = []
    current: Optional[ActionSegment] = None

    for edge in route_edges:
        action = edge.get('action', 'navigate_to_pose')
        # Legacy maps: normalise CamelCase -> canonical form
        if map_actions is None:
            action = normalize_action_name(action)

        # Determine if this action type is composable
        composable = True
        if map_actions and action in map_actions:
            composable = map_actions[action].get('composable', True)

        # Start a new segment when the action changes *or* when
        # the action is explicitly non-composable.
        if (
            current is None
            or current.action_type != action
            or not composable
        ):
            if current is not None:
                segments.append(current)
            current = ActionSegment(action_type=action)

        current.edge_ids.append(edge.get('edge_id', ''))
        current.source_nodes.append(edge['source'])
        current.target_nodes.append(edge['target'])
        current.edge_data.append(edge)

    if current is not None:
        segments.append(current)

    return segments


# ==============================================================================
# Boundary Polygon
# ==============================================================================

def compute_boundary_polygon(
    graph: nx.DiGraph,
    segment: ActionSegment,
    default_left: float = 0.5,
    default_right: float = 0.5,
) -> List[Tuple[float, float]]:
    """Compute boundary polygon for an action segment.

    Creates a corridor polygon around the path by offsetting each
    waypoint perpendicularly. The polygon goes left-side forward
    then right-side backward, forming a closed boundary.

    Boundary distances are read from the first edge's properties:
    ``boundary_left`` and ``boundary_right``. Falls back to defaults.

    This is action-type agnostic -- any segment whose edges carry
    ``boundary_left`` / ``boundary_right`` properties will produce
    a corridor polygon.

    Args:
        graph: NetworkX DiGraph with node position attributes.
        segment: ActionSegment whose edges may carry boundary props.
        default_left: Default left offset in meters.
        default_right: Default right offset in meters.

    Returns:
        List of ``(x, y)`` tuples forming a closed polygon.
        Empty list if segment has fewer than 2 waypoints.
    """
    if segment.is_empty:
        return []

    # Collect unique waypoints along the segment
    waypoints: List[Tuple[float, float]] = []
    seen: List[str] = []

    for edge in segment.edge_data:
        src = edge['source']
        if src not in seen:
            waypoints.append((
                float(graph.nodes[src]['x']),
                float(graph.nodes[src]['y']),
            ))
            seen.append(src)

        tgt = edge['target']
        if tgt not in seen:
            waypoints.append((
                float(graph.nodes[tgt]['x']),
                float(graph.nodes[tgt]['y']),
            ))
            seen.append(tgt)

    if len(waypoints) < 2:
        return []

    # Read boundary distances from the first edge properties
    props = segment.edge_data[0].get('properties', {})
    left_dist = float(props.get('boundary_left', default_left))
    right_dist = float(props.get('boundary_right', default_right))

    # Compute perpendicular offsets at each waypoint
    left_pts: List[Tuple[float, float]] = []
    right_pts: List[Tuple[float, float]] = []

    for i in range(len(waypoints)):
        # Direction vector: use forward difference, backward for last
        if i < len(waypoints) - 1:
            dx = waypoints[i + 1][0] - waypoints[i][0]
            dy = waypoints[i + 1][1] - waypoints[i][1]
        else:
            dx = waypoints[i][0] - waypoints[i - 1][0]
            dy = waypoints[i][1] - waypoints[i - 1][1]

        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue

        # Perpendicular unit vector (left = rotate 90deg CCW)
        perp_x = -dy / length
        perp_y = dx / length

        x, y = waypoints[i]
        left_pts.append((x + perp_x * left_dist, y + perp_y * left_dist))
        right_pts.append((x - perp_x * right_dist, y - perp_y * right_dist))

    # Closed polygon: left side forward + right side backward
    return left_pts + list(reversed(right_pts))


# ==============================================================================
# NavRoute conversion
# ==============================================================================

def plan_route_as_navroute(
    graph: nx.DiGraph,
    origin: str,
    target: str,
    avoid_edges: Optional[List[str]] = None,
    logger=None,
    algorithm: str = 'astar',
    weight: str = 'weight',
):
    """Plan a route and return a ``NavRoute`` message.

    Thin wrapper around :func:`plan_route` that converts the
    resulting node list into the ``NavRoute`` format expected by
    legacy services (``source[]`` and ``edge_id[]``).

    The caller must import ``NavRoute`` from
    ``topological_navigation_msgs.msg``.

    Args:
        graph: NetworkX DiGraph from :func:`build_graph_from_tmap`.
        origin: Name of the origin node.
        target: Name of the target node.
        avoid_edges: Optional list of edge_ids to exclude.
        logger: Optional ROS 2 logger.
        algorithm: ``'astar'`` or ``'dijkstra'``.
        weight: Edge attribute used as cost (default ``'weight'``).

    Returns:
        ``NavRoute`` message with ``source`` and ``edge_id`` fields
        populated when a route is found, or an empty ``NavRoute`` on
        failure.
    """
    from topological_navigation_msgs.msg import NavRoute

    route_msg = NavRoute()
    route_nodes = plan_route(
        graph, origin, target,
        avoid_edges=avoid_edges,
        logger=logger,
        algorithm=algorithm,
        weight=weight,
    )
    if not route_nodes or len(route_nodes) < 2:
        return route_msg

    route_edges = get_route_edges(graph, route_nodes)
    for edge in route_edges:
        route_msg.source.append(edge['source'])
        route_msg.edge_id.append(edge.get('edge_id', ''))

    return route_msg


# ==============================================================================
# Distance
# ==============================================================================

def get_route_distance(
    graph: nx.DiGraph,
    route_nodes: List[str],
) -> float:
    """Compute total Euclidean distance along a route.

    Args:
        graph: NetworkX DiGraph with node position attributes.
        route_nodes: Ordered list of node names.

    Returns:
        Total Euclidean distance in map units.
    """
    if not route_nodes or len(route_nodes) < 2:
        return 0.0

    total = 0.0
    for i in range(len(route_nodes) - 1):
        n1, n2 = route_nodes[i], route_nodes[i + 1]
        if n1 in graph and n2 in graph:
            dx = graph.nodes[n2]['x'] - graph.nodes[n1]['x']
            dy = graph.nodes[n2]['y'] - graph.nodes[n1]['y']
            total += math.hypot(dx, dy)

    return total
