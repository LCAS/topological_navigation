#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg


if __name__ == '__main__' :
    rospy.init_node("topological_map_update")
    rospy.loginfo("Publishing update request for topological map ...")
    map_update = rospy.Publisher('/update_map', std_msgs.msg.Time)
    map_update.publish(rospy.Time.now())
    rospy.loginfo("All Done ...")
    rospy.loginfo("Bye")
