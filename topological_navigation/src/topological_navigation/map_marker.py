#!/usr/bin/env python

import rospy

from visualization_msgs.msg import *

from topological_navigation_msgs.msg import TopologicalMap
from geometry_msgs.msg import Point

import topological_navigation.tmap_utils as tmap_utils


class TopologicalVis(object):
    _pallete=[[1,1,1],[0,0,0],[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1]]  
    def __init__(self) :
        self.actions=[]
        self.map_markers = MarkerArray()
        self.topmap_pub = rospy.Publisher('topological_map_visualisation', MarkerArray, queue_size = 1, latch=True)
        self._killall=False
        self.lnodes = None
        #Waiting for Topological Map        
        self.map_received=False
        rospy.Subscriber('topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while not self.map_received and not self._killall :
            rospy.sleep(rospy.Duration.from_sec(0.05))
        rospy.loginfo(" ...done")
        
        
        self.publish_markers()
        #rospy.loginfo("All Done ...")


    def _update_everything(self) :
        print "updating marker arrays ..."
        del self.map_markers
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]
        
        
        idn = 0        
        for i in self.lnodes.nodes :
            marker = self.get_node_marker(i,idn)
            self.map_markers.markers.append(marker)
            idn += 1

            marker = self.get_name_marker(i,idn)
            self.map_markers.markers.append(marker)
            idn += 1

            for j in i.edges : 
                marker = self.get_edge_marker(i, j, idn)
                if marker:
                    self.map_markers.markers.append(marker)
                    idn += 1

            legend =0
            for k in self.actions:
                marker = self.get_legend_marker(k, legend, idn)
                self.map_markers.markers.append(marker)
                idn += 1
                legend+=1
                
            marker = self.get_zone_marker(i, idn)
            self.map_markers.markers.append(marker)
            idn += 1
        
        self.publish_markers()

    def get_legend_marker(self, action, row, idn):
        col=self.get_colour(self.actions.index(action))
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "/map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text=action
        marker.pose.position.x= 1.0+(0.12*row)
        marker.pose.position.y= 0.0
        marker.pose.position.z= 0.2
        marker.scale.z = 0.1
        marker.color.a = 0.5
        marker.color.r = col[0]
        marker.color.g = col[1]
        marker.color.b = col[2]
        marker.ns='/legend'
        return marker


    def get_name_marker(self, node, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "/map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text= node.name
        marker.pose = node.pose
        marker.scale.z = 0.12
        marker.color.a = 0.9
        marker.color.r = 0.3
        marker.color.g = 0.3
        marker.color.b = 0.3
        marker.ns='/names'
        return marker
        
        

    def get_edge_marker(self, node, edge, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_LIST
        V1=Point()
        V2=Point()
        V1= node.pose.position
        to_node=tmap_utils.get_node(self.lnodes, edge.node)
        col=self.get_colour(self.actions.index(edge.action))
        #print col
        if to_node:
            V2= to_node.pose.position
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
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "/map"
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.4
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.7
        marker.pose = node.pose
        marker.pose.position.z = marker.pose.position.z+0.1
        marker.ns='/nodes'
        return marker
    
    
    def get_zone_marker(self, node, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.color.a = 0.8
        marker.color.r = 0.7
        marker.color.g = 0.1
        marker.color.b = 0.2

        for j in node.verts :
            Vert = Point()
            Vert.z = 0.05
            Vert.x = node.pose.position.x + j.x
            Vert.y = node.pose.position.y + j.y
            marker.points.append(Vert)

        Vert = Point()
        Vert.z = 0.05
        Vert.x = node.pose.position.x + node.verts[0].x
        Vert.y = node.pose.position.y + node.verts[0].y
        marker.points.append(Vert)
        marker.ns='/zones'
        return marker

    def publish_markers(self):
        self.topmap_pub.publish(MarkerArray())
        self.topmap_pub.publish(self.map_markers)
        

    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        if self.lnodes:
            del self.lnodes
        self.lnodes = msg

        for i in self.lnodes.nodes:
            for j in i.edges:
                if not j.action in self.actions:
                    self.actions.append(j.action)
        
        print self.actions

        self.map_received = True 
        self._update_everything()
        
    def get_colour(self, number):
        return self._pallete[number]


    def on_node_shutdown(self):
        self._killall=True
