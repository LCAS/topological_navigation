#!/usr/bin/env python

import sys
import rospy


from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from visualization_msgs.msg import *
from strands_navigation_msgs.msg import TopologicalNode
from topological_navigation.topological_map import *


class waypoints_markers(object):
    
    def __init__(self, nodes) :
        self.map_nodes = MarkerArray()
        for i in nodes.nodes :
            marker = Marker()
            marker.header.frame_id = "/map"
            #marker.header.stamp = rospy.now()
            #marker.type = marker.ARROW
            marker.type = Marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.2
            marker.color.g = 0.2
            marker.color.b = 0.7
            #marker.lifetime=2
            marker.pose = i._get_pose()
            marker.pose.position.z = marker.pose.position.z+0.1
            self.map_nodes.markers.append(marker)


        idn = 0
        for m in self.map_nodes.markers:
            m.id = idn
            idn += 1



class edges_marker(object):
    
    def __init__(self, nodes) :
        self.map_edges = MarkerArray()
        for node in nodes.nodes :
            for i in node.edges :
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.LINE_LIST
                ind = nodes._get_node_index(i['node'])
                V1=Point()
                V2=Point()
                V1= (node._get_pose()).position
                V2= (nodes.nodes[ind]._get_pose()).position
                marker.scale.x = 0.1
                marker.color.a = 0.6
                marker.color.r = 0.1
                marker.color.g = 0.1
                marker.color.b = 0.1
                marker.points.append(V1)
                marker.points.append(V2)
                self.map_edges.markers.append(marker)

        idn = 0
        for m in self.map_edges.markers:
            m.id = idn
            idn += 1            


class vertices_marker(object):

    def __init__(self, nodes) :
        self.node_zone = MarkerArray()
        for node in nodes.nodes :
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.LINE_STRIP
            marker.scale.x = 0.1
            marker.color.a = 0.8
            marker.color.r = 0.7
            marker.color.g = 0.1
            marker.color.b = 0.2
            node._get_coords()
            count=0
            for i in node.vertices :
                #print i[0], i[1]
                Vert = Point()
                Vert.z = 0.05
                Vert.x = node.px + i[0]
                Vert.y = node.py + i[1]
                marker.points.append(Vert)
                #vertname = "%s-%d" %(node.name, count)
                Pos = Pose()
                Pos.position = Vert
                # OJO: esto hay que hacerlo de alguna forma
                #self._vertex_marker(vertname, Pos, vertname)
                count+=1
            marker.points.append(marker.points[0])
            self.node_zone.markers.append(marker)
        
        idn = 0
        for m in self.node_zone.markers:
            m.id = idn
            idn += 1 


