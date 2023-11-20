#!/usr/bin/env python3

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

#from topological_navigation_msgs.msg import TopologicalMap
import topological_navigation.tmap_utils as tmap_utils


class TopologicalVis(Node):

    def __init__(self):
        super().__init__('topological_map_markers')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Marker Publisher
        self.map_markers = MarkerArray()
        self.topmap_pub = self.create_publisher(MarkerArray, '~/vis', qos_profile=qos)
        self.topmap_pub.publish(self.map_markers)

        # Rescaller Subscriber
        self.scale = 1
        self.rescale_sub = self.create_subscription(String, '~/rescale', self.rescale_callback, qos_profile=qos)

        # Map Subscriber
        self.map = None
        self.map_sub = self.create_subscription(String, '/topological_map_2', self.map_callback, qos_profile=qos)
        self.get_logger().info('map visualiser init complete')


    def rescale_callback(self, msg):
        self.get_logger().info('new scale recieved')
        self.get_logger().info(f'scale: {msg.data}')
        self.scale = float(msg.data)
        self.map_callback(self.map)

    def map_callback(self, msg):
        # Save Message for Republishing with scale
        self.map = msg

        # Load Yaml
        self.tmap = yaml.safe_load(msg.data)

        # Display Meta Information
        self.get_logger().info(' ')
        self.get_logger().info('-----')
        self.get_logger().info(' ')
        self.get_logger().info('map callback triggered')
        self.get_logger().info(f'name: {self.tmap["name"]}')
        self.get_logger().info(f'map: {self.tmap["metric_map"]}')
        self.get_logger().info(f'pointset: {self.tmap["pointset"]}')
        self.get_logger().info(f'last_updated: {self.tmap["meta"]["last_updated"]}')
        self.get_logger().info(f'total nodes: {len(self.tmap["nodes"])}')

        # Array
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]

        # Legend
        actions = list(set(sum([[edge['action'] for edge in node['node']['edges']] for node in self.tmap['nodes']],[])))
        for i, action in enumerate(actions):
            self.map_markers.markers.append(self.get_legend_marker(action, i))

        # Map
        for node in self.tmap['nodes']:

            # Nodes
            self.map_markers.markers.append(self.get_node_marker(node)) # Node
            self.map_markers.markers.append(self.get_name_marker(node)) # Name
            self.map_markers.markers.append(self.get_zone_marker(node)) # Zone

            # Edges
            for edge in node['node']['edges']:
                marker = self.get_edge_marker(node, edge, actions.index(edge['action']))
                if marker:
                    self.map_markers.markers.append(marker)

        # Publish
        for i, marker in enumerate(self.map_markers.markers):
            marker.id = i
        self.topmap_pub.publish(self.map_markers)
        self.get_logger().info('new map visual published')


    def get_legend_marker(self, action, col_id):
        col=self.get_colour(col_id)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text=action
        marker.pose.position.x= 1.0+(0.12*(self.scale*col_id))
        marker.pose.position.y= 0.0
        marker.pose.position.z= 0.2
        marker.pose.orientation.w= 1.0
        marker.scale.z = self.scale * 0.1
        marker.color.a = 0.5
        marker.color.r = float(col[0])
        marker.color.g = float(col[1])
        marker.color.b = float(col[2])
        marker.ns='/legend'
        return marker

    def get_name_marker(self, node):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text= node['node']['name']
        marker.pose.position.x = node['node']['pose']['position']['x']
        marker.pose.position.y = node['node']['pose']['position']['y']
        marker.pose.position.z = node['node']['pose']['position']['z']
        marker.scale.z = self.scale * 0.12
        marker.color.a = 0.9
        marker.color.r = 0.3
        marker.color.g = 0.3
        marker.color.b = 0.3
        marker.ns='/names'
        return marker

    def get_edge_marker(self, node, edge, col_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_LIST
        V1 = Point()
        V1.x = node['node']['pose']['position']['x']
        V1.y = node['node']['pose']['position']['y']
        V1.z = node['node']['pose']['position']['z']
        to_node = tmap_utils.get_node_from_tmap2(self.tmap, edge['node'])
        col = self.get_colour(col_id)
        if to_node:
            V2 = Point()
            V2.x = to_node['node']['pose']['position']['x']
            V2.y = to_node['node']['pose']['position']['y']
            V2.z = to_node['node']['pose']['position']['z']
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.scale * 0.1
            marker.color.a = 0.5
            marker.color.r = float(col[0])
            marker.color.g = float(col[1])
            marker.color.b = float(col[2])
            marker.points.append(V1)
            marker.points.append(V2)
            marker.ns='/edges'
            return marker
        else:
            self.get_logger().warn("No node %s found" %edge['node'])
            return None

    def get_node_marker(self, node):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.scale.x = self.scale * 0.2
        marker.scale.y = self.scale * 0.2
        marker.scale.z = self.scale * 0.2
        marker.color.a = 0.4
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.7
        marker.pose.position.x = node['node']['pose']['position']['x']
        marker.pose.position.y = node['node']['pose']['position']['y']
        marker.pose.position.z = node['node']['pose']['position']['z'] + 0.2
        marker.ns='/nodes'
        return marker


    def get_zone_marker(self, node):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.scale * 0.1
        marker.color.a = 0.8
        marker.color.r = 0.7
        marker.color.g = 0.1
        marker.color.b = 0.2

        for j in node['node']['verts']:
            vert = Point()
            vert.z = 0.05
            vert.x = node['node']['pose']['position']['x'] + j['x']
            vert.y = node['node']['pose']['position']['y'] + j['y']
            marker.points.append(vert)

        vert = Point()
        vert.z = 0.05
        vert.x = node['node']['pose']['position']['x'] + node['node']['verts'][0]['x']
        vert.y = node['node']['pose']['position']['y'] + node['node']['verts'][0]['y']
        marker.points.append(vert)
        marker.ns='/zones'
        return marker

    def get_colour(self, number):
        pallete=[[1,1,1],[0,0,0],[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1]]
        return pallete[number] if number < len(pallete) else pallete[0]


def main(args=None):
    rclpy.init(args=args)

    TV = TopologicalVis()
    rclpy.spin(TV)

    TV.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
