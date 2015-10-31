#!/usr/bin/env python

import sys
import rospy
import math
import tf

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *

from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap
from topological_navigation.topological_map import *



class edge_controllers(object):

    def __init__(self) :
        self.in_feedback=False        
        #self.timer = Timer(1.0, self.timer_callback)
        map_name = rospy.get_param('/topological_map_name', 'top_map')
        self.map_update = rospy.Publisher('/update_map', std_msgs.msg.Time)
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)        
        self._edge_server = InteractiveMarkerServer(map_name+"_edges")



    def update_map(self, msg) :
        print "updating edge controllers..."
        
        self.topo_map = topological_map(msg.name, msg=msg)       
        self._edge_server.clear()
        self._edge_server.applyChanges()
        
        for node in self.topo_map.nodes :
            for i in node.edges :
                ind = self.topo_map._get_node_index(i['node'])
                V1=Point()
                V2=Point()
                V1= (node._get_pose()).position
                V2= (self.topo_map.nodes[ind]._get_pose()).position
                edge_name=node.name+"_"+self.topo_map.nodes[ind].name
                self._edge_marker(edge_name, V1, V2, edge_name)

    
    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.update_map(msg)



    def makeEmptyMarker(self, dummyBox=True ):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.scale = 1
        return int_marker



    def makeBox(self, msg ):
        marker = Marker()
    
        marker.type = Marker.ARROW
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.25
        marker.scale.z = msg.scale * 0.15
        marker.lifetime.secs = 3
        marker.color.r = 0.8
        marker.color.g = 0.1
        marker.color.b = 0.0
        marker.color.a = 1.0
    
        return marker



    def _edge_marker(self, marker_name, point1, point2, marker_description="edge marker") :
        marker = self.makeEmptyMarker()
        marker.name = marker_name
        #marker.description = marker_description

        control = InteractiveMarkerControl()
    
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        control.markers.append( self.makeBox( marker ) )
        marker.controls.append(control)
    
        self._edge_server.insert(marker, self.feedback_cb)
        self._edge_server.applyChanges()

        pose = Pose()
        
        pose.position.x = (point1.x+point2.x)/2
        pose.position.y = (point1.y+point2.y)/2
        pose.position.z = ((point1.z+point2.z)/2)+0.15
        angle = math.atan2((point2.y-point1.y),(point2.x-point1.x))
        qat = tf.transformations.quaternion_from_euler(0, 0, angle)
        pose.orientation.w = qat[3]
        pose.orientation.x = qat[0]
        pose.orientation.y = qat[1]
        pose.orientation.z = qat[2]

        if pose is not None:
            self._edge_server.setPose( marker.name, pose )
            self._edge_server.applyChanges()



    def feedback_cb(self, feedback):
        self.topo_map.remove_edge(feedback.marker_name)
        self._edge_server.erase(feedback.marker_name)
        self._edge_server.applyChanges()
        self.map_update.publish(rospy.Time.now())
        

    
    def clear():
        self._edge_server.clear()
        self._edge_server.applyChanges()


        
    def __del__(self):
        del self._edge_server
