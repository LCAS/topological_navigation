#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
from topological_navigation.topological_node import *
from topological_navigation_msgs.msg import TopologicalNode
from topological_navigation_msgs.msg import TopologicalMap

from mongodb_store.message_store import MessageStoreProxy

from topological_navigation.publisher import map_publisher


if __name__ == '__main__' :
    point_set = sys.argv[1]
    rospy.init_node("topological_map_publisher")
    ps = map_publisher(point_set)
    rospy.spin()