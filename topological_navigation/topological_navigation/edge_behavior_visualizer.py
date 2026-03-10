#!/usr/bin/env python3
"""Publish color-coded topological graph edges by applied behavior class."""

from __future__ import annotations

import json
import os
import random

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class EdgeBehaviorVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("edge_behavior_visualizer")

        self.declare_parameter("graph_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("behavior_map_file", "")
        self.declare_parameter("randomize_edge_behaviors", True)
        self.declare_parameter("edge_behavior_seed", 42)
        self.declare_parameter("default_controller_id", "FollowPath")
        self.declare_parameter("slow_controller_id", "SlowFollowPath")
        self.declare_parameter("reverse_controller_id", "ReverseFollowPath")
        self.declare_parameter("marker_topic", "/topological_edges_colored")
        self.declare_parameter("line_width", 0.12)
        self.declare_parameter("line_z", 0.12)
        self.declare_parameter("publish_period_sec", 2.0)

        self.graph_file = str(self.get_parameter("graph_file").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.line_width = float(self.get_parameter("line_width").value)
        self.line_z = float(self.get_parameter("line_z").value)

        behavior_map_file = str(self.get_parameter("behavior_map_file").value)
        default_bt, edge_bts = self._load_behavior_map(behavior_map_file)
        graph_edges, edge_pairs = self._load_graph_edges(self.graph_file)
        edge_ids = sorted(graph_edges.keys())
        edge_controller_overrides = self._assign_random_edge_behaviors(edge_ids, edge_pairs)

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            marker_qos,
        )

        self.marker_array = self._build_markers(
            graph_edges,
            default_bt,
            edge_bts,
            edge_controller_overrides,
        )
        self.publisher.publish(self.marker_array)

        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(period, self._republish)

        self.get_logger().info(
            f"Edge behavior visualizer ready with {len(graph_edges)} edges"
        )

    def _load_behavior_map(self, file_path: str) -> tuple[str, dict[int, str]]:
        edge_bts: dict[int, str] = {}
        default_bt = ""
        if not file_path:
            return default_bt, edge_bts

        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        default_bt = os.path.expandvars(str(data.get("default_behavior_tree", "")))
        for entry in data.get("edge_behaviors", []):
            edge_id = int(entry["edge_id"])
            bt_file = os.path.expandvars(str(entry["bt_file"]))
            edge_bts[edge_id] = bt_file
        return default_bt, edge_bts

    def _load_graph_edges(
        self, graph_file: str
    ) -> tuple[dict[int, list[tuple[float, float]]], dict[int, tuple[int, int]]]:
        if not graph_file:
            raise RuntimeError("graph_file parameter is required")

        with open(graph_file, "r", encoding="utf-8") as f:
            graph = json.load(f)

        edges: dict[int, list[tuple[float, float]]] = {}
        edge_pairs: dict[int, tuple[int, int]] = {}
        for feature in graph.get("features", []):
            props = feature.get("properties", {})
            geom = feature.get("geometry", {})
            if "id" not in props:
                continue
            edge_id = int(props["id"])

            if "startid" in props and "endid" in props:
                a = int(props["startid"])
                b = int(props["endid"])
                edge_pairs[edge_id] = (min(a, b), max(a, b))
            else:
                edge_pairs[edge_id] = (edge_id, edge_id)

            geom_type = geom.get("type")
            if geom_type == "LineString":
                coords = geom.get("coordinates", [])
                edges[edge_id] = [(float(c[0]), float(c[1])) for c in coords]
            elif geom_type == "MultiLineString":
                flattened: list[tuple[float, float]] = []
                for line in geom.get("coordinates", []):
                    flattened.extend((float(c[0]), float(c[1])) for c in line)
                if flattened:
                    edges[edge_id] = flattened
        return edges, edge_pairs

    def _assign_random_edge_behaviors(
        self,
        edge_ids: list[int],
        edge_pairs: dict[int, tuple[int, int]],
    ) -> dict[int, str]:
        overrides: dict[int, str] = {}
        if not bool(self.get_parameter("randomize_edge_behaviors").value):
            return overrides

        pair_to_edges: dict[tuple[int, int], list[int]] = {}
        for edge_id in edge_ids:
            pair = edge_pairs.get(edge_id, (edge_id, edge_id))
            pair_to_edges.setdefault(pair, []).append(edge_id)

        pair_keys = list(pair_to_edges.keys())
        if len(pair_keys) < 3:
            return overrides

        pair_keys = list(pair_keys)
        seed = int(self.get_parameter("edge_behavior_seed").value)
        rng = random.Random(seed)
        rng.shuffle(pair_keys)

        slow_controller = str(self.get_parameter("slow_controller_id").value)
        reverse_controller = str(self.get_parameter("reverse_controller_id").value)
        group_size = len(pair_keys) // 3

        for pair in pair_keys[:group_size]:
            for edge_id in pair_to_edges[pair]:
                overrides[edge_id] = slow_controller
        for pair in pair_keys[group_size : 2 * group_size]:
            for edge_id in pair_to_edges[pair]:
                overrides[edge_id] = reverse_controller
        return overrides

    def _edge_class(
        self,
        edge_id: int,
        default_bt: str,
        edge_bts: dict[int, str],
        edge_controller_overrides: dict[int, str],
    ) -> str:
        slow_controller = str(self.get_parameter("slow_controller_id").value)
        reverse_controller = str(self.get_parameter("reverse_controller_id").value)

        controller = edge_controller_overrides.get(
            edge_id,
            str(self.get_parameter("default_controller_id").value),
        )

        if controller == slow_controller:
            return "slow"
        if controller == reverse_controller:
            return "reverse"

        edge_bt = edge_bts.get(edge_id, default_bt)
        if edge_bt and default_bt and edge_bt != default_bt:
            return "custom_bt"
        return "default"

    def _make_marker(self, marker_id: int, ns: str, color: ColorRGBA) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.line_width
        marker.color = color
        return marker

    def _build_markers(
        self,
        graph_edges: dict[int, list[tuple[float, float]]],
        default_bt: str,
        edge_bts: dict[int, str],
        edge_controller_overrides: dict[int, str],
    ) -> MarkerArray:
        markers = {
            "default": self._make_marker(0, "default", ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.95)),
            "slow": self._make_marker(1, "slow", ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.98)),
            "reverse": self._make_marker(2, "reverse", ColorRGBA(r=0.1, g=0.55, b=1.0, a=0.98)),
            "custom_bt": self._make_marker(3, "custom_bt", ColorRGBA(r=0.25, g=0.95, b=0.45, a=0.98)),
        }

        for edge_id, coords in graph_edges.items():
            if len(coords) < 2:
                continue
            klass = self._edge_class(edge_id, default_bt, edge_bts, edge_controller_overrides)
            marker = markers[klass]

            for i in range(len(coords) - 1):
                p0 = Point()
                p0.x = coords[i][0]
                p0.y = coords[i][1]
                p0.z = self.line_z
                p1 = Point()
                p1.x = coords[i + 1][0]
                p1.y = coords[i + 1][1]
                p1.z = self.line_z
                marker.points.append(p0)
                marker.points.append(p1)

        out = MarkerArray()
        out.markers = [markers["default"], markers["slow"], markers["reverse"], markers["custom_bt"]]
        return out

    def _republish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.marker_array.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.marker_array)


def main() -> None:
    rclpy.init()
    node = EdgeBehaviorVisualizer()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
