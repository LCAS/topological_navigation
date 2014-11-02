#!/usr/bin/env python

import sys
import rospy

import std_msgs.msg
from threading import Timer

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *


from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.msg import TopologicalNode
from topological_navigation.topological_map import *



class waypoint_controllers(object):

    def __init__(self, map_name) :
        self.timer = Timer(1.0, self.timer_callback)
        self._marker_server = InteractiveMarkerServer(map_name+"_markers")       
        self.map_update = rospy.Publisher('/update_map', std_msgs.msg.Time)


    
    def update_map(self, msg) :
        print "updating node controllers..."
        self.topo_map = topological_map(msg.name, msg=msg)
        self._marker_server.clear()
        self._marker_server.applyChanges()
        
        for i in self.topo_map.nodes :
            self._create_marker(i.name, i._get_pose(), i.name)

    def _create_marker(self, marker_name, pose, marker_description="waypoint marker") :
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
        #print '+'
        self.info = feedback
        self.timer.cancel()
        del self.timer
        self.timer = Timer(1.0, self.timer_callback)      
        self.timer.start()


    def timer_callback(self) :
        #print '*'
        self.topo_map.update_node_waypoint(self.info.marker_name, self.info.pose)
        self.map_update.publish(rospy.Time.now())
        

    def clear():
        self._marker_server.clear()
        self._marker_server.applyChanges()

    def __del__(self):
        del self._marker_server