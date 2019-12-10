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


def usage():
    print "\nPublishes Topological Maps:"
    print "\nFor loading a map from the mongodb:"
    print "\t rosrun topological_navigation map_manager.py map_name"
    print "\nFor creating a new map:"
    print "\t rosrun topological_navigation map_manager.py -n map_name"
    print "\n\n"



if __name__ == '__main__' :
    load=True
    if '-h' in sys.argv or '--help' in sys.argv or len(sys.argv) < 2 :
        usage()
        sys.exit(1)
    else:
        if '-n' in sys.argv:
            ind = sys.argv.index('-n')
            point_set=sys.argv[ind+1]
            print "Creating new Map (%s)" %point_set
            load=False
        else:
            point_set=sys.argv[1]
            
    rospy.init_node("topological_map_manager")
    ps = map_manager(point_set,load)
    rospy.spin()