#!/usr/bin/env python

import sys
import rospy
import pymongo
import json
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
from topological_navigation.policies import *
from topological_navigation.goto import *
from strands_navigation_msgs.msg import NavRoute

from strands_navigation_msgs.msg import TopologicalMap


class TopologicalMapVis(object):
    _killall_timers=False

    def __init__(self, name, filename) :

        rospy.on_shutdown(self._on_node_shutdown)

        self.update_needed=False
        self.in_feedback=False
        self._point_set=filename
        
        self.map_pub = rospy.Publisher('/topological_nodes_array', MarkerArray)
        self.map_zone_pub = rospy.Publisher('/topological_node_zones_array', MarkerArray)
        self.map_edge_pub = rospy.Publisher('/topological_edges_array', MarkerArray)
        self.map_edge_std_pub = rospy.Publisher('/topological_edges_deviation', MarkerArray)
        self.policies_pub = rospy.Publisher('/topological_edges_policies', MarkerArray)


        self.edge_cont = edge_controllers(self._point_set)
        self.vert_cont = vertex_controllers(self._point_set)
        self.node_cont = waypoint_controllers(self._point_set)
        self.goto_cont = go_to_controllers(self._point_set)
        self.add_rm_node = node_manager(self._point_set)

        
        self.subs = rospy.Subscriber("/top_nodes_std", NavRoute, self.route_callback)
        self.subs3 = rospy.Subscriber("/mdp_plan_exec/current_policy_mode", NavRoute, self.policies_callback)       
        rospy.Subscriber('/topological_map', TopologicalMap, self.map_callback)

        
        self._update_everything()
        
        self.subs = rospy.Subscriber("/top_nodes_std", NavRoute, self.route_callback)
        self.subs3 = rospy.Subscriber("/mdp_plan_exec/current_policy_mode", NavRoute, self.policies_callback)       
        rospy.Subscriber('/topological_map', TopologicalMap, self.map_callback)
      
        t = Timer(1.0, self.timer_callback)
        t.start()
        rospy.loginfo("All Done ...")


    def _update_everything(self) :
        print "updating ..."
        print "loading file from map %s" %self._point_set
        self.topo_map = topological_map(self._point_set)
        print "Done"
        self.wayp_marker = waypoints_markers(self.topo_map)
        self.map_edges = edges_marker(self.topo_map)
        self.node_zone = vertices_marker(self.topo_map)
        self.edge_std = edges_std_marker(self._point_set)
        self.policies = policies_marker(self._point_set)
        self.edge_std.update_map(self._point_set)
        self.policies.update_map(self._point_set) 
       


    def timer_callback(self):
        self.map_pub.publish(self.wayp_marker.map_nodes)
        self.map_edge_pub.publish(self.map_edges.map_edges)
        self.map_zone_pub.publish(self.node_zone.node_zone)
        if not self.edge_std.updating :
            self.map_edge_std_pub.publish(self.edge_std.map_edges)
        if not self.policies.updating :            
            self.policies_pub.publish(self.policies.map_edges)
        
        if not self._killall_timers :
            t = Timer(2.0, self.timer_callback)
            t.start()


    def route_callback(self, msg) :
        self.edge_std.received_route(msg)
        
    def policies_callback(self, msg) :
        self.policies.received_route(msg)


    def map_callback(self, msg) :
        self.goto_cont.update_map(msg)
        self.node_cont.update_map(msg)
        self.vert_cont.update_map(msg)
        self.edge_cont.update_map(msg)
        self.add_rm_node.update_map(msg)
        self.topo_map = topological_map(msg.name, msg=msg)

        self.wayp_marker = waypoints_markers(self.topo_map)
        self.map_edges = edges_marker(self.topo_map)
        self.node_zone = vertices_marker(self.topo_map)
        self.edge_std = edges_std_marker(self._point_set)
        self.policies = policies_marker(self._point_set)
        self.edge_std.update_map(self._point_set)
        self.policies.update_map(self._point_set)        
        

    def _on_node_shutdown(self):
        self._killall_timers=True
        #sleep(2)


if __name__ == '__main__':
    mapname=str(sys.argv[1])
    rospy.init_node('topological_visualisation')
    server = TopologicalMapVis(rospy.get_name(),mapname)
    rospy.spin()
