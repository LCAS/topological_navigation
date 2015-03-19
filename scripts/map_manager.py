#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap

from mongodb_store.message_store import MessageStoreProxy

from topological_navigation.manager import map_manager


if __name__ == '__main__' :
    point_set = sys.argv[1]
    rospy.init_node("topological_map_manager")
    ps = map_manager(point_set)
    rospy.spin()