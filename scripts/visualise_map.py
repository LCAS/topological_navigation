#!/usr/bin/env python

import sys
import rospy
import pymongo
import json
import sys
import math


from threading import Timer
from geometry_msgs.msg import Pose

from ros_datacentre.message_store import MessageStoreProxy

from strands_navigation_msgs.msg import TopologicalNode

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


class TopologicalMapVis(object):

    _killall_timers=True
    
    def __init__(self, name, filename) :

        self._point_set=filename
        self._marker_server = InteractiveMarkerServer(self._point_set+"_markers")
        
        print "loading file from map %s" %filename
        self.lnodes = self.loadMap(filename)
        print "Done"

        self.map_pub = rospy.Publisher('/topological_nodes_array', MarkerArray)

        self.map_nodes = MarkerArray()
        
        for i in self.lnodes :
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
            marker.pose.position.x = i.position.x
            marker.pose.position.y = i.position.y
            marker.pose.position.z = i.position.z+0.05
            marker.pose.orientation = i.orientation
            self.map_nodes.markers.append(marker)

        idn = 0
        for m in self.map_nodes.markers:
            m.id = idn
            idn += 1
    
        self.map_pub.publish(self.map_nodes)
        t = Timer(1.0, self.timer_callback)
        t.start()
        #self.run_analysis()

        rospy.loginfo("All Done ...")
        rospy.spin()



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
            self._marker_server.setPose( marker.name, pose )
            self._marker_server.applyChanges()


    def _marker_feedback(self, feedback):
        #update={}
        p = feedback.pose.position
        q = feedback.pose.orientation
        #self.msg_store.update_named(feedback.marker_name, feedback.pose, upsert=True);
        
        rospy.loginfo(feedback)
       # print feedback.marker_name + " is now at x:" + str(p.x) + ", y:" + str(p.y) + ", z:" + str(p.z) + ", w:" + str(p.w)

        


    def timer_callback(self):
        self.map_pub.publish(self.map_nodes)
        if not self._killall_timers :
            t = Timer(1.0, self.timer_callback)
            t.start()


    def loadMap(self, point_set):

        point_set=str(sys.argv[1])
        #map_name=str(sys.argv[3])
    
        msg_store = MessageStoreProxy()
    
        query_meta = {}
        query_meta["pointset"] = point_set
        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0

        print available

        if available <= 0 :
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
    
        else :
            query_meta = {}
            query_meta["pointset"] = point_set
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
    
            points = []
            for i in message_list:
                c=i[0].pose
                points.append(c)
                self._create_marker(i[0].name, c, i[0].name)
                #print c
            return points

    def _on_node_shutdown(self):
        self._killall_timers=True


if __name__ == '__main__':
    mapname=str(sys.argv[1])
    rospy.init_node('topological_visualisation')
    server = TopologicalMapVis(rospy.get_name(),mapname)
