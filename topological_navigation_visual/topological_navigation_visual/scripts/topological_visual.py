#!/usr/bin/env python3
"""Route and occupancy visualisation nodes.

Provides two ROS 2 nodes that run together in a single process:

- **RouteVisualiserNode** — subscribes to ``topological_navigation/Route``
  and publishes arrow markers between consecutive waypoints on
  ``topological_route_visualisation``.
- **OccupancyVisualiserNode** — subscribes to
  ``/topological_navigation/occupied_node`` and publishes sphere markers
  on ``/topological_navigation/visual/occupied_node``.

Both require a ``tmap`` parameter pointing to a ``.tmap2.yaml`` file.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

import yaml
import numpy as np
import tf_transformations

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from topological_navigation_msgs.msg import (
    TopologicalOccupiedNode,
    TopologicalRoute,
)


# ──────────────────────────────────────────────────────────────────
#  Utils
# ──────────────────────────────────────────────────────────────────
def load_waypoints_from_tmap(file_path):
    """Load and pre-transform waypoint vertices into the map frame."""
    try:
        with open(file_path, 'r') as fh:
            tmap_data = yaml.safe_load(fh)

        waypoints = {}
        for entry in tmap_data.get('nodes', []):
            try:
                node = entry['node']
                name = node['name']
                pos = node['pose']['position']
                ori = node['pose']['orientation']

                t_mat = tf_transformations.translation_matrix(
                    [pos['x'], pos['y'], pos['z']]
                )
                r_mat = tf_transformations.quaternion_matrix(
                    [ori['x'], ori['y'], ori['z'], ori['w']]
                )
                transform = np.dot(t_mat, r_mat)

                verts = []
                for v in node.get('verts', []):
                    local = np.array([v['x'], v['y'], 0.0, 1.0])
                    world = np.dot(transform, local)
                    verts.append((world[0], world[1]))

                waypoints[name] = {
                    'position': (pos['x'], pos['y'], pos['z']),
                    'orientation': (
                        ori['x'], ori['y'], ori['z'], ori['w']
                    ),
                    'verts': verts,
                }
            except KeyError:
                continue
        return waypoints

    except Exception:
        return {}


# ══════════════════════════════════════════════════════════════════
#  Route visualiser
# ══════════════════════════════════════════════════════════════════
class RouteVisualiserNode(Node):
    """Subscribes to a route topic and publishes arrow markers."""

    def __init__(self):
        super().__init__('route_visualiser')
        self.declare_parameter('tmap', rclpy.Parameter.Type.STRING)
        tmap_path = (
            self.get_parameter('tmap').get_parameter_value().string_value
        )
        self.tmap = load_waypoints_from_tmap(tmap_path)

        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(
            MarkerArray, 'topological_route_visualisation', qos_profile=qos
        )
        self.create_subscription(
            TopologicalRoute,
            'topological_navigation/Route',
            self._route_cb,
            qos_profile=qos,
        )
        self.get_logger().info('Route visualiser started')

    def _route_cb(self, msg: TopologicalRoute):
        self._clear()
        arr = MarkerArray()
        idn = 0
        if self.tmap:
            for i in range(1, len(msg.nodes)):
                arr.markers.append(
                    self._arrow(msg.nodes[i - 1], msg.nodes[i], idn)
                )
                idn += 1
        self.pub.publish(arr)

    def _clear(self):
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        self.pub.publish(arr)

    def _arrow(self, origin: str, end: str, idn: int) -> Marker:
        m = Marker()
        m.id = idn
        m.header.frame_id = 'map'
        m.type = Marker.ARROW
        o = self.tmap[origin]
        e = self.tmap[end]
        v1 = Point(
            x=o['position'][0], y=o['position'][1], z=o['position'][2] + 0.25
        )
        v2 = Point(
            x=e['position'][0], y=e['position'][1], z=e['position'][2] + 0.25
        )
        m.pose.orientation.w = 1.0
        m.scale.x = 0.08
        m.scale.y = 0.12
        m.scale.z = 0.2
        m.color.a = 1.0
        m.color.r = 0.33
        m.color.g = 0.99
        m.color.b = 0.55
        m.points.append(v1)
        m.points.append(v2)
        m.ns = '/route_path'
        return m


# ══════════════════════════════════════════════════════════════════
#  Occupancy visualiser
# ══════════════════════════════════════════════════════════════════
class OccupancyVisualiserNode(Node):
    """Subscribes to occupied-node topics and publishes sphere markers."""

    def __init__(self):
        super().__init__('occupancy_visualiser')
        self.declare_parameter('tmap', rclpy.Parameter.Type.STRING)
        tmap_path = (
            self.get_parameter('tmap').get_parameter_value().string_value
        )
        self.tmap = load_waypoints_from_tmap(tmap_path)
        self.last_count = 0

        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(
            MarkerArray,
            '/topological_navigation/visual/occupied_node',
            qos,
        )
        self.create_subscription(
            TopologicalOccupiedNode,
            '/topological_navigation/occupied_node',
            self._cb,
            10,
        )
        self.get_logger().info('Occupancy visualiser started')

    def _cb(self, msg: TopologicalOccupiedNode):
        names = msg.nodes if msg.nodes else []
        arr = MarkerArray()

        for i, wp in enumerate(names):
            data = self.tmap.get(wp)
            if not data:
                continue
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'occupied_waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = data['position'][0]
            m.pose.position.y = data['position'][1]
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 0.1
            m.color.r = 1.0
            m.color.a = 0.8
            arr.markers.append(m)

        # Delete stale markers
        for i in range(len(names), self.last_count):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'occupied_waypoints'
            m.id = i
            m.action = Marker.DELETE
            arr.markers.append(m)

        self.pub.publish(arr)
        self.last_count = len(names)


# ══════════════════════════════════════════════════════════════════
#  Entry-point
# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    route_node = RouteVisualiserNode()
    occupancy_node = OccupancyVisualiserNode()

    executor = MultiThreadedExecutor()
    executor.add_node(route_node)
    executor.add_node(occupancy_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        route_node.destroy_node()
        occupancy_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
