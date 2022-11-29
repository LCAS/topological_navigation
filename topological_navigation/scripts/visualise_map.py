#!/usr/bin/env python

import sys
import rospy

import math


from time import sleep

from threading import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import std_msgs.msg

#from mongodb_store.message_store import MessageStoreProxy

from interactive_markers.interactive_marker_server import *
#from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from topological_navigation_msgs.msg import TopologicalNode
from topological_navigation.topological_map import *
from topological_navigation.marker_arrays import *
from topological_navigation.node_controller import *
from topological_navigation.edge_controller import *
from topological_navigation.vertex_controller import *
from topological_navigation.node_manager import *
from topological_navigation.edge_std import *



from topological_navigation.goto import *


from topological_navigation_msgs.msg import NavRoute
from topological_navigation_msgs.msg import TopologicalMap
import topological_navigation.policies
import topological_navigation.map_marker


class VisualiseMap(object):
    _killall_timers=False

    def __init__(self, name, edit_mode, noedit_mode) :
        rospy.on_shutdown(self._on_node_shutdown)

        self.update_needed=False
        self.in_feedback=False
        #self._point_set=filename
        self._edit_mode = edit_mode
        self._noedit_mode = noedit_mode

        self.map_markers = topological_navigation.map_marker.TopologicalVis()
        self.pol_markers = topological_navigation.policies.PoliciesVis()
        
        if not noedit_mode:
            rospy.loginfo("Edge Controllers ...")
            self.edge_cont = edge_controllers()
            rospy.loginfo("Vertex Controllers ...")
            self.vert_cont = VertexControllers()
            rospy.loginfo("Waypoint Controllers ...")
            self.node_cont = WaypointControllers()
            rospy.loginfo("Node Manager Controllers ...")
            self.add_rm_node = node_manager()
        else:
            rospy.logwarn("No edit Visualisation mode ...")

        if not self._edit_mode :
            rospy.loginfo("Go To Controllers ...")
            self.goto_cont = go_to_controllers()
        else:
            rospy.logwarn("Edit only Visualisation mode ...")
            
        rospy.loginfo("Done ...")


        rospy.loginfo("All Done ...")

       
    def _on_node_shutdown(self):
        print("bye")



if __name__ == '__main__':
    edit_mode=False
    noedit_mode=False
    #mapname=str(sys.argv[1])
    argc = len(sys.argv)
    print(argc)
    if argc > 1:
        if '-edit' in sys.argv or '-e' in sys.argv :
            edit_mode = True
        if '-noedit' in sys.argv or '-n' in sys.argv :
            noedit_mode=True
    rospy.init_node('topological_visualisation')
    server = VisualiseMap(rospy.get_name(), edit_mode, noedit_mode)
    rospy.spin()
