#!/usr/bin/env python

import sys
import rospy
import pymongo


#from strands_navigation_msgs.msg import TopologicalNode
#from ros_datacentre.message_store import MessageStoreProxy
#from topological_navigation.topological_node import *
from topological_navigation.topological_map import *

#import topological_navigation.msg



class topologicalNodeRM(object):

    def __init__(self, pointset, waypoint) :

        self.topo_map = topological_map(pointset)
        self.topo_map.remove_node(waypoint)           
        rospy.loginfo("All Done ...")


if __name__ == '__main__':
    pointset=str(sys.argv[1])
    waypoint=str(sys.argv[2])
    rospy.init_node('node_rm')
    server = topologicalNodeRM(pointset,waypoint)