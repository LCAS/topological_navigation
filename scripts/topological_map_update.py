#!/usr/bin/env python
import math
import rospy
import sys

from time import sleep
import std_msgs.msg


if __name__ == '__main__' :
    rospy.init_node("topological_map_update")
    rospy.loginfo("Publishing update request for topological map ...")
    map_update = rospy.Publisher('/update_map', std_msgs.msg.Time)

    rospy.sleep(rospy.Duration.from_sec(1))
    tmstp = std_msgs.msg.Time()
    tmstp.data = rospy.Time.now()
    map_update.publish(tmstp)
    rospy.loginfo("All Done ...")
    rospy.loginfo("Bye")
