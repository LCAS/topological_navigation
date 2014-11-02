#!/usr/bin/env python

import sys
import rospy
import math
import tf
import numpy

import matplotlib as mpl
import matplotlib.cm as cm

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from visualization_msgs.msg import *

from strands_navigation_msgs.msg import TopologicalNode
from topological_navigation.topological_map import *
from strands_navigation_msgs.msg import NavRoute


class policies_marker(object):
    
    def __init__(self, map_name) :
        self.map_name = map_name
        self.route_nodes = NavRoute()
        self.updating=True
        self.update_map(map_name)
        

    def update_map(self,map_name) :
        self.topo_map = topological_map(self.map_name)
        self.map_edges = MarkerArray()
                
        counter=0
        total = len(self.route_nodes.source)

        #print 'updating '+str(total)+' edges'        
        while counter < total :
            #print 'Creating edge '+str(counter) 
            inds = self.topo_map._get_node_index(self.route_nodes.source[counter])
            indt = self.topo_map._get_node_index(self.route_nodes.target[counter])
            point1=Point()
            point2=Point()
            point1= (self.topo_map.nodes[inds]._get_pose()).position
            point2= (self.topo_map.nodes[indt]._get_pose()).position
            #val = self.route_nodes.prob[counter]
            self.create_edge(point1, point2)
            counter+=1

        idn = 0
        for m in self.map_edges.markers:
            m.id = idn
            idn += 1
        self.updating=False
        
        
        print "All Done"


    def create_edge(self, point1, point2):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.ARROW
        pose = Pose()
        
        pose.position.x = point1.x
        pose.position.y = point1.y
        pose.position.z = point1.z
        angle = math.atan2((point2.y-point1.y),(point2.x-point1.x))
        
        qat = tf.transformations.quaternion_from_euler(0, 0, angle)
        pose.orientation.w = qat[3]
        pose.orientation.x = qat[0]
        pose.orientation.y = qat[1]
        pose.orientation.z = qat[2]                
        
        r = math.hypot((point2.y-point1.y),(point2.x-point1.x))#/3.0
        marker.scale.x = r
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 0.95
        marker.color.r = 0.1
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.pose = pose
        self.map_edges.markers.append(marker)


    def received_route(self, route):
        #print "Route Received"
        if not self.updating :
            #print "Updating Route"
            self.route_nodes = route
            self.clear()
            self.update_map(self.topo_map)
    
    def clear(self):
        if not self.updating :
            self.updating=True
            del self.map_edges
