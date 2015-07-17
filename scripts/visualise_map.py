#!/usr/bin/env python

import sys
import rospy
import sys
import math


from time import sleep

from threading import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import std_msgs.msg

from mongodb_store.message_store import MessageStoreProxy

from interactive_markers.interactive_marker_server import *
#from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from strands_navigation_msgs.msg import TopologicalNode
from topological_navigation.topological_map import *
from topological_navigation.marker_arrays import *
from topological_navigation.node_controller import *
from topological_navigation.edge_controller import *
from topological_navigation.vertex_controller import *
from topological_navigation.node_manager import *
from topological_navigation.edge_std import *



from topological_navigation.goto import *


from strands_navigation_msgs.msg import NavRoute
from strands_navigation_msgs.msg import TopologicalMap
import topological_navigation.policies
import topological_navigation.map_marker


class VisualiseMap(object):
    _killall_timers=False

    def __init__(self, name, filename, edit_mode) :
        rospy.on_shutdown(self._on_node_shutdown)

        self.update_needed=False
        self.in_feedback=False
        self._point_set=filename
        self._edit_mode = edit_mode
        
        rospy.loginfo("Edge Controllers ...")
        self.edge_cont = edge_controllers()
        rospy.loginfo("Vertex Controllers ...")
        self.vert_cont = VertexControllers()
        rospy.loginfo("Waypoint Controllers ...")
        self.node_cont = WaypointControllers()

        if not self._edit_mode :
            rospy.loginfo("Go To Controllers ...")
            self.goto_cont = go_to_controllers()

        rospy.loginfo("Node Manager Controllers ...")
        self.add_rm_node = node_manager()
        rospy.loginfo("Done ...")

        self.map_markers = topological_navigation.map_marker.TopologicalVis()

        rospy.loginfo("All Done ...")

       
    def _on_node_shutdown(self):
        print "bye"



if __name__ == '__main__':
    edit_mode=False
    mapname=str(sys.argv[1])
    argc = len(sys.argv)
    if argc > 2:
        if '-edit' in sys.argv or '-e' in sys.argv :
            edit_mode = True
    rospy.init_node('topological_visualisation')
    server = VisualiseMap(rospy.get_name(),mapname, edit_mode)
    rospy.spin()
