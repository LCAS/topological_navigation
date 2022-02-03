#!/usr/bin/env python

import sys
import rospy
import pymongo
import json
import sys
import math


from time import sleep

from threading import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import String

from visualization_msgs.msg import *


def get_node(nodes_list, name):
    for i in nodes_list:
        if i['node']['name'] == name:
            return i['node']


class TopoMap2Vis(object):
    _pallete=[[1,1,1],[0,0,0],[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1]]
    def __init__(self, name) :
        """
        """
        self.killall=False
        self.topological_map = None
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]

        rospy.on_shutdown(self._on_node_shutdown)

        self.topmap_pub = rospy.Publisher('topological_map2_visualisation', MarkerArray, queue_size = 1, latch=True)
        self.topo_map_sub = rospy.Subscriber("topological_map_2", String, self.topo_map_cb)

        rospy.loginfo("Waiting for topo_map")

        rospy.loginfo("All Done ...")
        rospy.spin()

    def topo_map_cb(self, msg):
        """
        """
        self.topological_map = json.loads(msg.data)
        rospy.loginfo("Received new topo_map")
        print(self.topological_map['name'])
        self.create_map_marker()


    def create_map_marker(self):
        """
        """
        del self.map_markers
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]
        self.actions=[]
        idn=0
        for i in self.topological_map['nodes']:
            self.map_markers.markers.append(self.get_node_marker(i['node'], idn))
            idn+=1
            self.map_markers.markers.append(self.get_name_marker(i['node'], idn))
            idn+=1
            self.map_markers.markers.append(self.get_zone_marker(i['node'], idn))
            idn+=1
            for j in i['node']['edges']:
                self.update_actions(j['action'])
                marker = self.get_edge_marker(i['node'], j, idn)
                if marker:
                    self.map_markers.markers.append(marker)
                    idn += 1
        
        legend =0
        for k in self.actions:
            marker = self.get_legend_marker(k, legend, idn)
            self.map_markers.markers.append(marker)
            idn += 1
            legend+=1
        self.topmap_pub.publish(self.map_markers)


    def get_colour(self, number):
        """
        """
        return self._pallete[number]

    def update_actions(self, action):
        """
        """
        if action not in self.actions:
            self.actions.append(action)


    def get_legend_marker(self, action, row, idn):
        """
        """
        col=self.get_colour(self.actions.index(action))
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text=action
        marker.pose.position.x= 1.0
        marker.pose.position.y= 0.0 + (0.18*row)
        marker.pose.position.z= 0.2
        marker.pose.orientation.w= 1.0
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = col[0]
        marker.color.g = col[1]
        marker.color.b = col[2]
        marker.ns='/legend'
        return marker


    def get_edge_marker(self, node, edge, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = node['parent_frame']
        marker.type = marker.LINE_LIST
        V1=Point()
        V2=Point()
        V1=self.node2pose(node['pose']).position
        V1.z += 0.1
        to_node=get_node(self.topological_map['nodes'], edge['node'])
        col=self.get_colour(self.actions.index(edge['action']))
        #print col
        if to_node:
            V2= self.node2pose(to_node['pose']).position
            V2.z += 0.1
            marker.pose.orientation.w= 1.0
            marker.scale.x = 0.1
            marker.color.a = 0.5
            marker.color.r = col[0]
            marker.color.g = col[1]
            marker.color.b = col[2]
            marker.points.append(V1)
            marker.points.append(V2)
            marker.ns='/edges'
            return marker
        else:
            rospy.logwarn("No node %s found" %edge.node)
            return None


    def get_node_marker(self, node, idn):
        """
        """
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = node['parent_frame']
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.6
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.7
        marker.pose = self.node2pose(node['pose'])
        marker.pose.position.z += 0.1
        marker.ns='/nodes'
        return marker


    def get_name_marker(self, node, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = node['parent_frame']
        marker.type = marker.TEXT_VIEW_FACING
        marker.text= node['name']
        marker.pose = self.node2pose(node['pose'])
        marker.pose.position.z += 0.2
        #marker.pose.position.y += 0.2
        marker.scale.z = 0.15
        marker.color.a = 0.99
        marker.color.r = 0.99
        marker.color.g = 0.99
        marker.color.b = 0.99
        marker.ns='/names'
        return marker


    def node2pose(self, pose):
        node_pose=Pose()
        node_pose.position.x = pose['position']['x']
        node_pose.position.y = pose['position']['y']
        node_pose.position.z = pose['position']['z']
        node_pose.orientation.w = pose['orientation']['w']
        node_pose.orientation.x = pose['orientation']['x']
        node_pose.orientation.y = pose['orientation']['y']
        node_pose.orientation.z = pose['orientation']['z']
        return node_pose


    def get_zone_marker(self, node, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.pose.orientation.w= 1.0
        marker.scale.x = 0.1
        marker.color.a = 0.8
        marker.color.r = 0.7
        marker.color.g = 0.1
        marker.color.b = 0.2

        for j in node['verts'] :
            Vert = Point()
            Vert.z = 0.05
            Vert.x = node['pose']['position']['x'] + j['x']
            Vert.y = node['pose']['position']['y'] + j['y']
            marker.points.append(Vert)

        Vert = Point()
        Vert.z = 0.05
        Vert.x = node['pose']['position']['x'] + node['verts'][0]['x']
        Vert.y = node['pose']['position']['y'] + node['verts'][0]['y']
        marker.points.append(Vert)
        marker.ns='/zones'
        return marker


    def _on_node_shutdown(self):
        """
        """
        self.killall=True
        rospy.loginfo("See you later")
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('topomap2_visualisation')
    server = TopoMap2Vis(rospy.get_name())
