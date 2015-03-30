#!/usr/bin/env python

import sys
import rospy
#import actionlib
#import pymongo
#import json
import sys
import math


#from geometry_msgs.msg import Pose
from std_msgs.msg import String
#import scitos_apps_msgs.msg

#from strands_navigation_msgs.msg import TopologicalNode
#from mongodb_store.message_store import MessageStoreProxy

from strands_navigation_msgs.msg import TopologicalMap
from topological_navigation.tmap_utils import *
from topological_navigation.route_search import *
        

class SearchRoute(object):
       
    def __init__(self, goal) :     
        rospy.loginfo("Waiting for Topological map ...")

        try:
            msg = rospy.wait_for_message('/topological_map', TopologicalMap, timeout=10.0)
            self.top_map = msg
            self.lnodes = msg.nodes
        except rospy.ROSException :
            rospy.logwarn("Failed to get topological map")
            return

        rospy.loginfo("Waiting for Current Node ...")
              
        try:
            msg = rospy.wait_for_message('/closest_node', String, timeout=10.0)
            cnode = msg.data
        except rospy.ROSException :
            rospy.logwarn("Failed to get closest node")
            return

            
        rsearch = TopologicalRouteSearch(self.top_map)
        route = rsearch.search_route(cnode, goal)
        print route
        
        rospy.loginfo("All Done ...")
        #rospy.spin()


if __name__ == '__main__':
    rospy.init_node('route_search')
    goal =  str(sys.argv[1])
    searcher = SearchRoute(goal)