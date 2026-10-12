#!/usr/bin/env python3
"""Pure-Python core logic for recording topological maps.

This module implements the map-building logic used by the ``map_recorder``
ROS 2 node (see ``scripts/map_recorder.py``). It has no dependency on
``rclpy`` so that it can be unit tested without a running ROS 2 environment.

The recorder builds a tmap2-format dictionary incrementally:

- A new node is created whenever the robot has moved ``node_distance``
  metres away from the previously recorded node.
- Loop closure is supported: if the candidate pose falls inside the
  influence zone (``verts`` polygon) of an already recorded node, that
  existing node is reused/linked instead of creating a duplicate node.
- Nodes are connected successively: each new/linked node is connected to
  the previously recorded node with two edges (one in each direction).
- Recorded nodes are tagged with a ``map.source: recording`` property so
  that downstream tools can tell which nodes were produced by the
  recorder.
"""

import datetime
import math
from copy import deepcopy

import yaml

from topological_navigation.networkx_utils import build_graph_from_tmap, point_in_poly_nx

MAP_SOURCE_PROPERTY_VALUE = "recording"


def _now():
    """Return a timestamp string in the same format used elsewhere in the repo."""
    return datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


class _PositionOnly:
    """Minimal stand-in for a ``geometry_msgs/Pose`` with only ``position.x/y``."""

    class _Position:
        __slots__ = ("x", "y")

    def __init__(self, x, y):
        self.position = self._Position()
        self.position.x = x
        self.position.y = y


def init_topomap(pointset, site_name, template_action):
    """Create an empty tmap2 dict from *template_action* definitions/actions."""
    return {
        "meta": {"last_updated": _now()},
        "metric_map": site_name,
        "name": pointset,
        "pointset": pointset,
        "transformation": {
            "topo_frame_id": site_name or "map",
            "parent": "map",
            "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "definitions": deepcopy(template_action.get("definitions", {})),
        "actions": deepcopy(template_action.get("actions", {})),
        "nodes": [],
    }


def topomap_from_source(source_tmap, pointset, site_name, template_action):
    """Create an empty map, inheriting meta data from *source_tmap* when given."""
    tmap = init_topomap(pointset, site_name, template_action)
    if source_tmap:
        tmap["metric_map"] = source_tmap.get("metric_map", tmap["metric_map"])
        tmap["name"] = pointset or source_tmap.get("name", tmap["name"])
        tmap["pointset"] = pointset or source_tmap.get("pointset", tmap["pointset"])
        tmap["transformation"] = deepcopy(
            source_tmap.get("transformation", tmap["transformation"]))
        if source_tmap.get("definitions"):
            tmap["definitions"] = deepcopy(source_tmap["definitions"])
        if source_tmap.get("actions"):
            tmap["actions"] = deepcopy(source_tmap["actions"])
    return tmap


def distance_xy(pose_a, pose_b):
    """Euclidean distance between two ``{"x":, "y":}``-like dicts."""
    return math.hypot(pose_a["x"] - pose_b["x"], pose_a["y"] - pose_b["y"])


def make_node_dict(template_node, name, pose, pointset, site_name, extra_properties=None):
    """Build a tmap2 node dict for *name* at *pose* from *template_node*.

    ``pose`` is a dict with keys ``x``, ``y`` and optionally ``z``, ``qx``,
    ``qy``, ``qz``, ``qw`` (defaults: ``z=0``, identity orientation).
    """
    node_dict = deepcopy(template_node)
    node_dict["meta"]["map"] = site_name
    node_dict["meta"]["node"] = name
    node_dict["meta"]["pointset"] = pointset

    nd = node_dict["node"]
    nd["name"] = name
    nd["edges"] = []
    nd["pose"]["position"] = {
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "z": float(pose.get("z", 0.0)),
    }
    nd["pose"]["orientation"] = {
        "x": float(pose.get("qx", 0.0)),
        "y": float(pose.get("qy", 0.0)),
        "z": float(pose.get("qz", 0.0)),
        "w": float(pose.get("qw", 1.0)),
    }

    properties = deepcopy(nd.get("properties", {}))
    if extra_properties:
        properties.update(deepcopy(extra_properties))
    nd["properties"] = properties

    return node_dict


def make_bidirectional_edges(template_edge, node_a, node_b):
    """Return the ``(a->b, b->a)`` edge dicts connecting *node_a* and *node_b*."""
    fwd = deepcopy(template_edge)
    fwd["edge_id"] = f"{node_a}_{node_b}"
    fwd["node"] = node_b

    rev = deepcopy(template_edge)
    rev["edge_id"] = f"{node_b}_{node_a}"
    rev["node"] = node_a
    return fwd, rev


def find_loop_closure_node(tmap, pose, logger=None):
    """Return the name of an existing node whose influence zone contains *pose*.

    Returns ``None`` if no recorded node's polygon contains the pose (or the
    map has no nodes yet).
    """
    graph = build_graph_from_tmap(tmap, logger=logger)
    if graph is None or graph.number_of_nodes() == 0:
        return None

    ros_pose = _PositionOnly(pose["x"], pose["y"])
    for node_name in graph.nodes:
        if point_in_poly_nx(graph, node_name, ros_pose):
            return node_name
    return None


class MapRecorderCore:
    """Incrementally builds a tmap2 map while the robot is being driven around.

    This class contains no ROS dependencies; the ``map_recorder`` node wires
    it to ROS topics/services/actions.
    """

    def __init__(self, template_node, template_edge, template_action,
                 pointset="recorded_map", site_name="map"):
        self.template_node = template_node
        self.template_edge = template_edge
        self.template_action = template_action
        self.pointset = pointset
        self.site_name = site_name
        self.reset()

    # ------------------------------------------------------------------
    # Map (re)initialisation
    # ------------------------------------------------------------------

    def reset(self, source_tmap=None):
        """Clear the recorded map, optionally seeding it from *source_tmap*."""
        self.tmap = topomap_from_source(
            source_tmap, self.pointset, self.site_name, self.template_action)
        self._order = []       # node names in the order they were added
        self._node_index = {}  # node name -> index into tmap['nodes']
        self._history = []     # recording steps, for delete_last_node() to undo

        if source_tmap:
            for node_data in source_tmap.get("nodes", []):
                name = node_data["node"]["name"]
                self.tmap["nodes"].append(deepcopy(node_data))
                self._node_index[name] = len(self.tmap["nodes"]) - 1
                self._order.append(name)

        self._next_id = len(self._order)

    def load(self, tmap_dict):
        """Load *tmap_dict* as the (extendable) base of the recorded map."""
        self.reset(source_tmap=tmap_dict)

    # ------------------------------------------------------------------
    # Node helpers
    # ------------------------------------------------------------------

    def _get_node(self, name):
        return self.tmap["nodes"][self._node_index[name]]

    def _pose_of(self, name):
        position = self._get_node(name)["node"]["pose"]["position"]
        return {"x": position["x"], "y": position["y"], "z": position.get("z", 0.0)}

    def _link(self, node_a, node_b):
        """Create bidirectional edges between *node_a* and *node_b* (if needed).

        Returns ``True`` if at least one new edge was appended, ``False`` if
        both edges already existed (or *node_a* is ``None``/equal to *node_b*).
        """
        if node_a is None or node_a == node_b:
            return False
        fwd, rev = make_bidirectional_edges(self.template_edge, node_a, node_b)

        added = False
        a_edges = self._get_node(node_a)["node"]["edges"]
        if not any(e["edge_id"] == fwd["edge_id"] for e in a_edges):
            a_edges.append(fwd)
            added = True

        b_edges = self._get_node(node_b)["node"]["edges"]
        if not any(e["edge_id"] == rev["edge_id"] for e in b_edges):
            b_edges.append(rev)
            added = True

        return added

    def _unlink(self, node_a, node_b):
        """Remove the bidirectional edges between *node_a* and *node_b* (if present)."""
        if node_a is None or node_a == node_b:
            return
        fwd_id = f"{node_a}_{node_b}"
        rev_id = f"{node_b}_{node_a}"

        a_edges = self._get_node(node_a)["node"]["edges"]
        a_edges[:] = [e for e in a_edges if e["edge_id"] != fwd_id]

        b_edges = self._get_node(node_b)["node"]["edges"]
        b_edges[:] = [e for e in b_edges if e["edge_id"] != rev_id]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_node(self, pose, node_distance=0.0, force=False):
        """Try to record *pose* as a new node (or reuse an existing one).

        Args:
            pose: dict with keys ``x``, ``y`` and optionally ``z``, ``qx``,
                ``qy``, ``qz``, ``qw``.
            node_distance: minimum distance (m) from the last recorded node
                required before a new node is created. Ignored when *force*.
            force: when ``True``, bypass the ``node_distance`` check (used
                for the explicit "add node" service call).

        Returns:
            Tuple ``(node_name, created, message)`` where *created* is
            ``True`` only if a brand-new node was appended to the map.
        """
        last_name = self._order[-1] if self._order else None

        if last_name is not None and not force:
            if distance_xy(pose, self._pose_of(last_name)) < node_distance:
                return last_name, False, "too close to the last recorded node"

        loop_node = find_loop_closure_node(self.tmap, pose)
        if loop_node is not None:
            if loop_node == last_name:
                return last_name, False, "still within the last node's influence zone"
            link_added = self._link(last_name, loop_node)
            self._order.append(loop_node)
            self._history.append({
                "name": loop_node, "created": False,
                "linked_from": last_name, "link_added": link_added,
            })
            return loop_node, False, f"loop closure: linked to existing node '{loop_node}'"

        name = f"node{self._next_id}"
        self._next_id += 1

        node_dict = make_node_dict(
            self.template_node, name, pose, self.pointset, self.site_name,
            extra_properties={"map": {"source": MAP_SOURCE_PROPERTY_VALUE}},
        )
        self.tmap["nodes"].append(node_dict)
        self._node_index[name] = len(self.tmap["nodes"]) - 1
        self._order.append(name)

        link_added = self._link(last_name, name)
        self._history.append({
            "name": name, "created": True,
            "linked_from": last_name, "link_added": link_added,
        })

        self.tmap["meta"]["last_updated"] = _now()
        return name, True, f"node '{name}' added"

    def delete_last_node(self):
        """Undo the most recent recording step.

        If the last step created a brand-new node, that node (and the edge
        linking it to the previous one) is removed. If the last step was a
        loop closure (re-visiting an already recorded node), only the edge
        linking it to the previous node is removed -- the reused node itself
        is kept since it may still be referenced earlier in the recording.
        """
        if not self._history:
            return False, "no nodes to delete"

        entry = self._history.pop()
        name = self._order.pop()

        if entry["link_added"]:
            self._unlink(entry["linked_from"], name)

        if entry["created"]:
            idx = self._node_index.pop(name)
            self.tmap["nodes"].pop(idx)

            for other_name, other_idx in self._node_index.items():
                if other_idx > idx:
                    self._node_index[other_name] = other_idx - 1

            for node_data in self.tmap["nodes"]:
                node_data["node"]["edges"] = [
                    e for e in node_data["node"]["edges"] if e["node"] != name
                ]

        self.tmap["meta"]["last_updated"] = _now()
        kind = "node" if entry["created"] else "loop-closure link to node"
        return True, f"removed {kind} '{name}'"

    def num_nodes(self):
        """Return the number of unique nodes currently in the recorded map."""
        return len(self._node_index)

    def num_visits(self):
        """Return the number of recording steps, including loop-closure revisits."""
        return len(self._order)

    def last_node_name(self):
        return self._order[-1] if self._order else ""

    def to_yaml(self):
        return yaml.dump(self.tmap, default_flow_style=False)
