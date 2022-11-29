#!/usr/bin/env python

import sys
import rospy

import std_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import Pose

#from strands_navigation_msgs.msg import TopologicalNode
#from ros_datacentre.message_store import MessageStoreProxy
#from topological_navigation.topological_node import *
from topological_navigation_msgs.msg import TopologicalMap
from topological_navigation.topological_map import *


#import topological_navigation.msg



class addWaypoint(object):

    def __init__(self, filename) :
        rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.JoyCallback)      
        self.filename = filename
        #fh = open(filename, "a")
        #fh.close
        rospy.loginfo("All Done ...")
        rospy.spin()


    def JoyCallback(self, msg) :
        if msg.buttons[1]:
            print "JOY"
            self.add_waypoint()


    def add_waypoint(self) :

        try:
            p = rospy.wait_for_message('robot_pose', Pose, timeout=10.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get robot pose")
            return

        fh = open(filename, "a")
        
        s_output = "%s,%s,%s,%s,%s,%s,%s\n" %(str(p.position.x),str(p.position.y),str(p.position.z),
                                              str(p.orientation.x),str(p.orientation.y),str(p.orientation.z),str(p.orientation.w))
        fh.write(s_output)
        fh.close
      



if __name__ == '__main__':
    filename=str(sys.argv[1])
#    name=str(sys.argv[2])
#    dist=float(sys.argv[3])
    name = 'WayPoint'
    dist = 10.0
    rospy.init_node('node_add')
    server = addWaypoint(filename)