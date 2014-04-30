#!/usr/bin/env python

import sys
import rospy
import pymongo
import json
import sys
import math


from threading import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from ros_datacentre.message_store import MessageStoreProxy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from strands_navigation_msgs.msg import TopologicalNode

from topological_navigation.topological_map import *


class TopologicalMapVis(object):

    _killall_timers=True
    
    def __init__(self, name, filename) :

        self._point_set=filename
        self._marker_server = InteractiveMarkerServer(self._point_set+"_markers")

        self.map_pub = rospy.Publisher('/topological_nodes_array', MarkerArray)
        self.map_nodes = MarkerArray()


        self.map_edge_pub = rospy.Publisher('/topological_edges_array', MarkerArray)
        self.map_edges = MarkerArray()
        
        print "loading file from map %s" %filename
        self.topo_map = topological_map(filename)
        print "Done"

        self._create_interactive_markers()
        self._create_marker_array()
        self._create_edges_array()

        self.map_pub.publish(self.map_nodes)
        t = Timer(1.0, self.timer_callback)
        t.start()       
        #self.run_analysis()

        rospy.loginfo("All Done ...")
        rospy.spin()


    def _create_marker_array(self):
             
        for i in self.topo_map.nodes :
            #print i
            marker = Marker()
            marker.header.frame_id = "/map"
            #marker.header.stamp = rospy.now()
            marker.type = marker.ARROW
            marker.scale.x = 0.3
            marker.scale.y = 0.15
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.2
            marker.color.g = 0.2
            marker.color.b = 0.7
            #marker.lifetime=0
            marker.pose = i._get_pose()
            marker.pose.position.z = marker.pose.position.z+0.1
            self.map_nodes.markers.append(marker)

        idn = 0
        for m in self.map_nodes.markers:
            m.id = idn
            idn += 1
    

    def _create_edges_array(self):
        #node=self.topo_map.nodes[0]
        for node in self.topo_map.nodes :
            for i in node.edges :
                marker = Marker()
                marker.header.frame_id = "/map"
                #marker.header.stamp = rospy.now()
                marker.type = marker.LINE_LIST
                #marker.lifetime=0
                ind = self.topo_map._get_node_index(i['node'])
                #print "%s -> %s" %(node.name, self.topo_map.nodes[ind].name)
                V1=Point()
                V2=Point()
                V1= (node._get_pose()).position
                V2= (self.topo_map.nodes[ind]._get_pose()).position
                marker.scale.x = 0.1
                #marker.scale.y = 0.15
                #marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.2
                marker.color.g = 0.8
                marker.color.b = 0.2
                marker.points.append(V1)
                marker.points.append(V2)
                self.map_edges.markers.append(marker)

        idn = 0
        for m in self.map_edges.markers:
            m.id = idn
            idn += 1            



    def _create_interactive_markers(self):
        for i in self.topo_map.nodes :
            self._create_marker(i.name, i._get_pose(), i.name)


    def _create_marker(self, marker_name, pose, marker_description="waypoint marker"):
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = "/map"
        marker.name = marker_name
        marker.description = marker_description

        # the marker in the middle
        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.25
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        marker.controls.append( box_control )

        # move x
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        #move y
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        #rotate z
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        self._marker_server.insert(marker, self._marker_feedback)
        self._marker_server.applyChanges()

        if pose is not None:
            pose.position.z=pose.position.z+0.15
            self._marker_server.setPose( marker.name, pose )
            self._marker_server.applyChanges()


    def _marker_feedback(self, feedback):
        #update={}
        #msg_store = MessageStoreProxy()
        
        #p = feedback.pose.position
        #q = feedback.pose.orientation
        #msg_store.update_named(feedback.marker_name, feedback.pose, upsert=True);
        
        rospy.loginfo(feedback)
        #print feedback.marker_name + " is now at x:" + str(p.x) + ", y:" + str(p.y) + ", z:" + str(p.z) + ", w:" + str(q.w)
      

    def timer_callback(self):
        self.map_pub.publish(self.map_nodes)
        self.map_edge_pub.publish(self.map_edges)
        if not self._killall_timers :
            t = Timer(1.0, self.timer_callback)
            t.start()

    def _on_node_shutdown(self):
        self._killall_timers=True


if __name__ == '__main__':
    mapname=str(sys.argv[1])
    rospy.init_node('topological_visualisation')
    server = TopologicalMapVis(rospy.get_name(),mapname)
