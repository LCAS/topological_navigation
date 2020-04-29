#!/usr/bin/env python

import rospy
from topological_navigation.goto import *

if __name__ == '__main__':
    rospy.init_node("go_to_node")

    goto_cont = go_to_controllers()

    rospy.spin()
