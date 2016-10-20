#!/usr/bin/env python

import rospy

from visualization_msgs.msg import *

from strands_navigation_msgs.msg import TopologicalMap
from geometry_msgs.msg import Point

import topological_navigation.tmap_utils as tmap_utils


class TopologicalVis(object):

    def __init__(self) :       
        self.map_markers = MarkerArray()
        self.topmap_pub = rospy.Publisher('/topological_map_markers', MarkerArray, queue_size = 1, latch=True)
        self._killall=False
        self.lnodes = None
        #Waiting for Topological Map        
        self.map_received=False
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      
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
            #print i
            marker = self.get_node_marker(i,idn)
            self.map_markers.markers.append(marker)
            idn += 1

            for j in i.edges : 
                marker = self.get_edge_marker(i, j, idn)
                if marker:
                    self.map_markers.markers.append(marker)
                    idn += 1
                
            marker = self.get_zone_marker(i, idn)
            self.map_markers.markers.append(marker)
            idn += 1
        
        self.publish_markers()


    def get_edge_marker(self, node, edge, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_LIST
        V1=Point()
        V2=Point()
        V1= node.pose.position
        to_node=tmap_utils.get_node(self.lnodes, edge.node)
        if to_node:
            V2= to_node.pose.position
            marker.scale.x = 0.1
            marker.color.a = 0.6
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.points.append(V1)
            marker.points.append(V2)       
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
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.7
        marker.pose = node.pose
        marker.pose.position.z = marker.pose.position.z+0.1        
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
        return marker


    def publish_markers(self):
        self.topmap_pub.publish(self.map_markers)
        

    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        if self.lnodes:
            del self.lnodes
        self.lnodes = msg
        self.map_received = True 
        self._update_everything()
        

    def on_node_shutdown(self):
        self._killall=True
