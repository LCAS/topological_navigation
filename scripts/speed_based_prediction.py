#!/usr/bin/env python

import sys
import rospy
import actionlib
import pymongo
import json
import sys
import math
import time

from datetime import datetime

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from nav_msgs.srv import *


import strands_navigation_msgs.msg
from strands_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import NavStatistics
from strands_navigation_msgs.msg import TopologicalMap

from topological_navigation.tmap_utils import *

from strands_navigation_msgs.srv import *


def usage():
    print "For using all the available stats use:"
    print "\t rosrun topological_navigation topological_prediction.py"
    print "For all the stats in a range use:"
    print "\t rosrun topological_navigation topological_prediction.py from_epoch to_epoch"
    print "For all the stats from a date until now use:"
    print "\t rosrun topological_navigation topological_prediction.py from_epoch -1"
    print "For all the stats until one date:"
    print "\t rosrun topological_navigation topological_prediction.py 0 to_epoch"


class TopologicalSpeedPred(object):

    def __init__(self) :

        name= rospy.get_name()
        action_name = name+'/build_temporal_model'

        rospy.Subscriber('/topological_map', TopologicalMap, self.map_callback)
        
        rospy.loginfo("Waiting for Topological map ...")
        
        self.map_received = False
        self.edge_to_duration = {}

        while not self.map_received:
            rospy.sleep(rospy.Duration(0.1))
            rospy.loginfo("Waiting for Topological map ...")

        rospy.loginfo("... Got Topological map")


        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(action_name, strands_navigation_msgs.msg.BuildTopPredictionAction, execute_cb = self.build_callback, auto_start = False)
        
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")


        self.predict_srv=rospy.Service('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState, self.predict_edge_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()

 

    def predict_edge_cb(self, req):
        return self.edge_to_duration.keys(), [1] * len(self.edge_to_duration), self.edge_to_duration.values()

    """
     map_callback
     
     This function receives the topological map the computes duration estimates for all edges
    """
    def map_callback(self, tmap) :
        self.edge_to_duration = {}
        
        for i in tmap.nodes :
            for j in i.edges:
                if j.edge_id not in self.edge_to_duration:                                    
                    destination = get_node(tmap, j.node)
                    if j.top_vel >=  0.1:
                        self.edge_to_duration[j.edge_id] = rospy.Duration(get_distance_to_node(i, destination)/j.top_vel)
                    else :
                        self.edge_to_duration[j.edge_id] = rospy.Duration(get_distance_to_node(i, destination)/0.1)

        self.map_received = True

    """
     build_callback
     
     Does nothing for this version.
    """
    def build_callback(self, goal):    
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('topological_prediction')
    server = TopologicalSpeedPred()
