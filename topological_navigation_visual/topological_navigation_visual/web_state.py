#!/usr/bin/env python3
# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""Backend state store for the topological navigation web app.

The web front end is stateless: every browser renders exactly what this
store holds.  The ROS node feeds the store from topics / action callbacks,
and the HTTP server streams snapshots of it to all connected clients, so
that every open view is always synchronised.

This module has no ROS dependencies so it can be unit-tested with plain
``pytest``.
"""

import math
import threading
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple


# action_msgs/msg/GoalStatus values that indicate a goal is still running.
_ACTIVE_GOAL_STATUSES = (1, 2, 3)  # ACCEPTED, EXECUTING, CANCELING

_TRUE_STRINGS = ('true', '1', 'yes', 'on', 'enabled')


def _truthy(value: Any) -> bool:
    """Interpret booleans, numbers and strings ("true", "yes", ...)."""
    if isinstance(value, str):
        return value.strip().lower() in _TRUE_STRINGS
    return bool(value)


def is_roboflow_enabled(properties: Optional[Dict[str, Any]]) -> bool:
    """Return True if a node's properties have roboflow enabled.

    Accepts the namespaced form ``{roboflow: {enabled: true}}`` as well as
    the shorthand ``{roboflow: true}``.
    """
    if not isinstance(properties, dict):
        return False
    roboflow = properties.get('roboflow')
    if isinstance(roboflow, dict):
        return _truthy(roboflow.get('enabled', False))
    if roboflow is None:
        return False
    return _truthy(roboflow)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the yaw (rotation about z) of a quaternion in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def tmap_to_graph(tmap: Dict[str, Any]) -> Dict[str, Any]:
    """Convert a tmap2 dictionary into the compact graph used by the UI.

    Returns a dict with ``name``, ``frame``, ``nodes`` and ``edges``.  Each
    node carries its metric position, yaw, properties and a ``roboflow``
    flag; each edge carries its id, source, target, action and properties.
    """
    if not isinstance(tmap, dict):
        raise ValueError('topological map must be a dictionary')

    transformation = tmap.get('transformation') or {}
    frame = 'map'
    if isinstance(transformation, dict):
        frame = transformation.get('topo_frame_id') or frame

    nodes: List[Dict[str, Any]] = []
    edges: List[Dict[str, Any]] = []
    for entry in tmap.get('nodes') or []:
        node = (entry or {}).get('node') or {}
        name = node.get('name')
        if not name:
            continue
        pose = node.get('pose') or {}
        pos = pose.get('position') or {}
        ori = pose.get('orientation') or {}
        properties = node.get('properties') or {}
        nodes.append({
            'name': name,
            'x': float(pos.get('x', 0.0) or 0.0),
            'y': float(pos.get('y', 0.0) or 0.0),
            'yaw': yaw_from_quaternion(
                float(ori.get('x', 0.0) or 0.0),
                float(ori.get('y', 0.0) or 0.0),
                float(ori.get('z', 0.0) or 0.0),
                float(ori.get('w', 1.0) if ori.get('w') is not None else 1.0),
            ),
            'properties': properties,
            'roboflow': is_roboflow_enabled(properties),
        })
        for edge in node.get('edges') or []:
            target = (edge or {}).get('node')
            if not target:
                continue
            edges.append({
                'id': edge.get('edge_id') or '%s_%s' % (name, target),
                'source': name,
                'target': target,
                'action': edge.get('action', ''),
                'properties': edge.get('properties') or {},
            })

    # Drop edges whose target is not part of the map (UI cannot draw them).
    names = {n['name'] for n in nodes}
    edges = [e for e in edges if e['target'] in names]

    return {
        'name': tmap.get('name') or tmap.get('pointset') or '',
        'frame': frame,
        'nodes': nodes,
        'edges': edges,
    }


def route_edge_ids(route: Iterable[str],
                   edges: Iterable[Dict[str, Any]]) -> List[str]:
    """Return the ids of map edges connecting consecutive route nodes."""
    by_pair: Dict[Tuple[str, str], List[str]] = {}
    for edge in edges:
        by_pair.setdefault((edge['source'], edge['target']), []).append(
            edge['id'])
    route = list(route)
    result: List[str] = []
    for src, tgt in zip(route, route[1:]):
        result.extend(by_pair.get((src, tgt), []))
    return result


def _clean_node_name(name: Optional[str]) -> Optional[str]:
    if name is None:
        return None
    name = str(name).strip()
    if not name or name.lower() in ('none', 'unknown'):
        return None
    return name


class WebAppState:
    """Thread-safe store of everything the web UI displays.

    Every mutation bumps ``version`` and wakes up waiters, which lets the
    HTTP server push changes to all clients (Server-Sent Events).
    """

    def __init__(self, clock=time.time):
        self._clock = clock
        self._cond = threading.Condition()
        self._version = 0
        self._map_version = 0
        self._map: Optional[Dict[str, Any]] = None
        self._node_names: set = set()

        self._current_node: Optional[str] = None
        self._closest_node: Optional[str] = None
        self._pose: Optional[Dict[str, Any]] = None
        self._speed: Optional[Dict[str, Any]] = None

        self._nav_status: Optional[str] = None
        self._route: List[str] = []
        self._current_edge: Optional[str] = None
        self._current_goal: Optional[str] = None
        self._destination: Optional[str] = None
        self._server_active = False
        self._web_goal_pending = False
        self._last_result: Optional[Dict[str, Any]] = None
        self._message: Optional[str] = None

    # ------------------------------------------------------------------
    # Change notification
    # ------------------------------------------------------------------
    def _bump(self, map_changed: bool = False) -> None:
        """Increment versions and notify waiters (lock must be held)."""
        self._version += 1
        if map_changed:
            self._map_version += 1
        self._cond.notify_all()

    @property
    def version(self) -> int:
        with self._cond:
            return self._version

    @property
    def map_version(self) -> int:
        with self._cond:
            return self._map_version

    def wait_for_change(self, version: int, timeout: float) -> int:
        """Block until ``version`` is outdated or ``timeout`` expires."""
        with self._cond:
            self._cond.wait_for(lambda: self._version != version, timeout)
            return self._version

    def notify_all(self) -> None:
        """Wake up all waiters without changing the state (e.g. shutdown)."""
        with self._cond:
            self._cond.notify_all()

    # ------------------------------------------------------------------
    # Map
    # ------------------------------------------------------------------
    def set_map(self, tmap: Dict[str, Any]) -> bool:
        """Store a new topological map; returns True if it changed."""
        graph = tmap_to_graph(tmap)
        with self._cond:
            if graph == self._map:
                return False
            self._map = graph
            self._node_names = {n['name'] for n in graph['nodes']}
            self._bump(map_changed=True)
            return True

    def has_node(self, name: str) -> bool:
        with self._cond:
            return name in self._node_names

    @property
    def map_frame(self) -> Optional[str]:
        with self._cond:
            return self._map['frame'] if self._map else None

    def map_snapshot(self) -> Dict[str, Any]:
        with self._cond:
            return {'map_version': self._map_version, 'map': self._map}

    # ------------------------------------------------------------------
    # Robot
    # ------------------------------------------------------------------
    def _set(self, attr: str, value: Any) -> None:
        with self._cond:
            if getattr(self, attr) != value:
                setattr(self, attr, value)
                self._bump()

    def set_current_node(self, name: Optional[str]) -> None:
        self._set('_current_node', _clean_node_name(name))

    def set_closest_node(self, name: Optional[str]) -> None:
        self._set('_closest_node', _clean_node_name(name))

    def set_pose(self, x: float, y: float, yaw: float,
                 frame: str = 'map') -> None:
        self._set('_pose', {
            'x': round(float(x), 3), 'y': round(float(y), 3),
            'yaw': round(float(yaw), 3), 'frame': frame,
        })

    def clear_pose(self) -> None:
        self._set('_pose', None)

    def set_odometry(self, vx: float, vy: float, wz: float) -> None:
        self._set('_speed', {
            'linear': round(math.hypot(float(vx), float(vy)), 3),
            'angular': round(float(wz), 3),
        })

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def set_nav_status(self, status: Optional[str]) -> None:
        self._set('_nav_status', status or None)

    def set_route(self, nodes: Iterable[str]) -> None:
        self._set('_route', [str(n) for n in nodes])

    def set_current_edge(self, edge: Optional[str]) -> None:
        """Store the current edge; strips the ``--<map>`` suffix."""
        if edge:
            edge = str(edge).split('--')[0]
        self._set('_current_edge', _clean_node_name(edge))

    def set_move_status(self, status: Dict[str, Any]) -> None:
        """Handle the JSON published on ``move_action_status``."""
        if not isinstance(status, dict):
            return
        with self._cond:
            goal = _clean_node_name(status.get('goal'))
            final_goal = _clean_node_name(status.get('final_goal'))
            changed = False
            if goal != self._current_goal:
                self._current_goal = goal
                changed = True
            if final_goal and final_goal != self._destination:
                self._destination = final_goal
                changed = True
            if changed:
                self._bump()

    def set_action_statuses(self, statuses: Iterable[int]) -> None:
        """Update from the action server's goal status array."""
        active = any(int(s) in _ACTIVE_GOAL_STATUSES for s in statuses)
        with self._cond:
            if active != self._server_active:
                self._server_active = active
                self._bump()

    def goal_requested(self, target: str) -> None:
        with self._cond:
            self._destination = target
            self._current_goal = None
            self._route = []
            self._web_goal_pending = True
            self._message = 'Goal to %s sent' % target
            self._bump()

    def goal_rejected(self, target: str, reason: str = 'rejected') -> None:
        with self._cond:
            if target == self._destination:
                self._web_goal_pending = False
            self._last_result = {
                'target': target, 'outcome': reason, 'time': self._clock(),
            }
            self._message = 'Goal to %s %s' % (target, reason)
            self._bump()

    def goal_accepted(self, target: str) -> None:
        with self._cond:
            self._web_goal_pending = False
            self._server_active = True
            self._destination = target
            self._message = 'Goal to %s accepted' % target
            self._bump()

    def goal_finished(self, target: str, outcome: str) -> None:
        with self._cond:
            # A goal that was pre-empted by a newer one must not clear the
            # newer goal's state; the status array corrects it otherwise.
            if target == self._destination:
                self._web_goal_pending = False
                self._server_active = False
            self._last_result = {
                'target': target, 'outcome': outcome, 'time': self._clock(),
            }
            self._message = 'Goal to %s %s' % (target, outcome)
            self._bump()

    def set_message(self, message: Optional[str]) -> None:
        self._set('_message', message)

    @property
    def navigation_active(self) -> bool:
        with self._cond:
            return self._server_active or self._web_goal_pending

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------
    def snapshot(self) -> Dict[str, Any]:
        """Return a JSON-serialisable view of the full UI state."""
        with self._cond:
            active = self._server_active or self._web_goal_pending
            route = list(self._route) if active else []
            edges = self._map['edges'] if self._map else []
            return {
                'version': self._version,
                'map_version': self._map_version,
                'robot': {
                    'current_node': self._current_node,
                    'closest_node': self._closest_node,
                    'pose': dict(self._pose) if self._pose else None,
                    'speed': dict(self._speed) if self._speed else None,
                },
                'navigation': {
                    'active': active,
                    'status': self._nav_status,
                    'destination': self._destination if active else None,
                    'current_goal': self._current_goal if active else None,
                    'current_edge': self._current_edge if active else None,
                    'route': route,
                    'route_edges': route_edge_ids(route, edges),
                    'last_result': (dict(self._last_result)
                                    if self._last_result else None),
                    'message': self._message,
                },
            }
