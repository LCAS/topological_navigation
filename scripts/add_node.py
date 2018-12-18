#!/usr/bin/env python

import sys
import rospy
#import pymongo

import std_msgs.msg
from geometry_msgs.msg import Pose

#from strands_navigation_msgs.msg import TopologicalNode
#from ros_datacentre.message_store import MessageStoreProxy
#from topological_navigation.topological_node import *
from topological_navigation.topological_map import *


#import topological_navigation.msg



class topologicalNodeAdd(object):

    def __init__(self, pointset, name, dist) :

        try:
            pos = rospy.wait_for_message('robot_pose', Pose, timeout=10.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get robot pose")
            return

        self.topo_map = topological_map(pointset)     
        self.topo_map.add_node(name,dist, pos, 'move_base')

        map_update = rospy.Publisher('/update_map', std_msgs.msg.Time)        
        map_update.publish(rospy.Time.now())
        rospy.loginfo("All Done ...")


if __name__ == '__main__':
    pointset=str(sys.argv[1])
    name=str(sys.argv[2])
    dist=float(sys.argv[3])
    rospy.init_node('node_add')
    server = topologicalNodeAdd(pointset,name,dist)