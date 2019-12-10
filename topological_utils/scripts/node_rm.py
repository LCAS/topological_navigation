#!/usr/bin/env python

import sys
import rospy
import pymongo

import std_msgs.msg
#from strands_navigation_msgs.msg import TopologicalNode
#from ros_datacentre.message_store import MessageStoreProxy
#from topological_navigation.topological_node import *
from topological_navigation.topological_map import *

#import topological_navigation.msg



class topologicalNodeRM(object):

    def __init__(self, pointset, waypoint) :

        self.topo_map = topological_map(pointset)
        self.topo_map.remove_node(waypoint)    
        
        
        map_update = rospy.Publisher('/update_map', std_msgs.msg.Time)        
        map_update.publish(rospy.Time.now())

        rospy.loginfo("All Done ...")


if __name__ == '__main__':
    pointset=str(sys.argv[1])
    waypoint=str(sys.argv[2])
    rospy.init_node('node_rm')
    server = topologicalNodeRM(pointset,waypoint)