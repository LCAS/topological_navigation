#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
from topological_navigation.topological_node import *
from topological_navigation_msgs.msg import TopologicalNode
from topological_navigation_msgs.msg import TopologicalMap

from mongodb_store.message_store import MessageStoreProxy

from topological_navigation.manager import map_manager


def usage():
    print("\nPublishes Topological Maps:")
    print("\nFor loading a map from the mongodb:")
    print("\t rosrun topological_navigation map_manager.py map_name")
    print("\nFor loading a map from a tmap file:")
    print("\t rosrun topological_navigation map_manager.py -f map_filename")
    print("\nFor creating a new map:")
    print("\t rosrun topological_navigation map_manager.py -n map_name")
    print("\n\n")
    
    
    
if __name__ == '__main__' :
    load=True
    load_from_file = False
    if '-h' in sys.argv or '--help' in sys.argv or len(sys.argv) < 2 :
        usage()
        sys.exit(1)
    else:
        if '-n' in sys.argv:
            ind = sys.argv.index('-n')
            _map=sys.argv[ind+1]
            print("Creating new Map (%s)" %_map)
            load=False
        elif '-f' in sys.argv:
            ind = sys.argv.index('-f')
            _map=sys.argv[ind+1]
            print("Loading map from tmap file (%s)" %_map)
            load_from_file=True
        else:
            _map=sys.argv[1]

    rospy.init_node("topological_map_manager")
    ps = map_manager()
    ps.init_map(_map,load,load_from_file)
    rospy.spin()
#########################################################################################################