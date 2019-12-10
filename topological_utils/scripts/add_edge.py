#!/usr/bin/env python

import sys
import rospy
#import pymongo


#from strands_navigation_msgs.msg import TopologicalNode
#from ros_datacentre.message_store import MessageStoreProxy
#from topological_navigation.topological_node import *
from topological_navigation.topological_map import *

#import topological_navigation.msg



class topologicalEdgeAdd(object):

    def __init__(self, pointset, or_waypoint, de_waypoint, action) :

        self.topo_map = topological_map(pointset)
        self.topo_map.add_edge(or_waypoint, de_waypoint, action)           
        rospy.loginfo("All Done ...")


if __name__ == '__main__':
    pointset=str(sys.argv[1])
    or_waypoint=str(sys.argv[2])
    de_waypoint=str(sys.argv[3])
    action = str(sys.argv[4])
    rospy.init_node('add_edge')
    server = topologicalEdgeAdd(pointset,or_waypoint, de_waypoint, action)