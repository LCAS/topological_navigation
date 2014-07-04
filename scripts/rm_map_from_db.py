#!/usr/bin/env python

import sys
import rospy
import pymongo


from topological_navigation.topological_map import *



class topologicalNodeRM(object):

    def __init__(self, pointset) :

        self.topo_map = topological_map(pointset)
        self.topo_map.delete_map()           
        rospy.loginfo("All Done ...")


if __name__ == '__main__':
    pointset=str(sys.argv[1])
    rospy.init_node('map_rm')
    server = topologicalNodeRM(pointset)