#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from tf_transformations import quaternion_from_euler

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from topological_navigation_msgs.msg import TopologicalMap, NavRoute


class PoliciesVis(Node):

    def __init__(self):
        super().__init__('topological_policy_markers')
        self.map = None
        self.policy = MarkerArray()
        self.route_nodes = NavRoute()
        self.marker_pub = self.create_publisher(MarkerArray, '~/vis', 10)
        self.marker_pub.publish(self.policy)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(TopologicalMap, '/topological_map', self.map_callback, qos)
        self.policy_sub = self.create_subscription(NavRoute, 'mdp_plan_exec/current_policy_mode', self.policies_callback, 10)
        self.get_logger().info('policy visualiser init complete')

    def map_callback(self, msg):
        self.get_logger().info('map callback triggered')
        self.map = msg

    def policies_callback(self, msg):
        self.get_logger().info('policies callback triggered')
        self.get_logger().info(str(msg))
        if not self.map: return

        self.added_sources = []
        self.route_nodes = msg
        print(self.route_nodes)

        self.map_edges.markers=[]

        counter=0
        total = len(self.route_nodes.source)

        for i in range(0, total):
            # Find source and edge
            source = self.route_nodes.source[i]
            edge_id = self.route_nodes.edge_id[i]

            # Get positions for start and end
            ori = tmap_utils.get_node(self.map, source)
            targ = tmap_utils.get_edge_from_id(self.map, ori.name, edge_id).node
            if not targ: continue

            # Edge
            target = self.get_node(self.map, targ)
            self.added_sources.append(source)
            colour = [0.1,0.1,0.9] if targ in self.added_sources else [0.9,0.1,0.1]
            self.map_edges.markers.append(self.create_edge(ori.pose.position, target.pose.position, colour))

        # Publish
        for i, marker in enumerate(self.policy.markers):
            marker.id = i
        self.policies_pub.publish(self.policy)

    def create_edge(self, point1, point2, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.ARROW

        pose = Pose()

        pose.position.x = point1.x
        pose.position.y = point1.y
        pose.position.z = point1.z
        angle = math.atan2((point2.y-point1.y),(point2.x-point1.x))

        qat = quaternion_from_euler(0, 0, angle)
        pose.orientation.w = qat[3]
        pose.orientation.x = qat[0]
        pose.orientation.y = qat[1]
        pose.orientation.z = qat[2]

        r = math.hypot((point2.y-point1.y),(point2.x-point1.x))#/3.0
        marker.scale.x = r
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 0.95
        marker.color.r = colour[0]
        marker.color.g = colour[1]
        marker.color.b = colour[2]
        marker.pose = pose
        self.policy.markers.append(marker)


def main(args=None):
    rclpy.init(args=args)

    PV = PoliciesVis()
    rclpy.spin(PV)

    PV.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
