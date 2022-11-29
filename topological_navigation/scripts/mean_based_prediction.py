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
import numpy as np

import topological_navigation_msgs.msg
from topological_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy
from topological_navigation_msgs.msg import NavStatistics
from topological_navigation_msgs.msg import TopologicalMap

from topological_navigation.tmap_utils import *

from topological_navigation_msgs.srv import *


def usage():
    print("For using all the available stats use:")
    print("\t rosrun topological_navigation mean_based_prediction.py")
    print("For all the stats in a range use:")
    print("\t rosrun topological_navigation mean_based_prediction.py from_epoch to_epoch")
    print("For all the stats from a date until now use:")
    print("\t rosrun topological_navigation mean_based_prediction.py from_epoch -1")
    print("For all the stats until one date:")
    print("\t rosrun topological_navigation mean_based_prediction.py 0 to_epoch")


class TopologicalMeanPred(object):

    def __init__(self, epochs) :

        name= rospy.get_name()
        action_name = name+'/build_temporal_model'
        self.range = epochs
        self.edge_to_duration = {}
        self.edge_to_probability = {}
        self.map = None
        rospy.Subscriber('topological_map', TopologicalMap, self.map_callback)
        
        rospy.loginfo("Waiting for Topological map and building model ...")
        
        self.map_received = False


        while not self.map_received:
            rospy.sleep(rospy.Duration(1))
            rospy.loginfo("Waiting for Topological map and building model ...")

        rospy.loginfo("... Done")


        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(action_name, topological_navigation_msgs.msg.BuildTopPredictionAction, execute_cb = self.build_callback, auto_start = False)
        
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")


        self.predict_srv=rospy.Service('topological_prediction/predict_edges', topological_navigation_msgs.srv.PredictEdgeState, self.predict_edge_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()

 

    def predict_edge_cb(self, req):
        # build like this to ensure both lists are in the same order
        results = [[edge_id, self.edge_to_probability[edge_id], self.edge_to_duration[edge_id]] for edge_id in self.edge_to_duration.keys()]
        return zip(*results)

    """
     map_callback
     
     This function receives the topological map the computes duration estimates for all edges
    """
    def map_callback(self, tmap) :
        self.map = tmap    
        self.build_model()        
        self.map_received = True

    """
     build_callback
     
     Does nothing for this version.
    """
    def build_callback(self, goal):  
        # set epoch ranges based on goal
        if goal.start_range.secs > 0:
            self.range[0] = goal.start_range.secs
        if goal.end_range.secs > 0:
            self.range[1] = goal.end_range.secs

        rospy.loginfo('Building model for epoch range: %s' % self.range)

        self.build_model()


        self._as.set_succeeded()


    def build_model(self):
        """ Builds a model of mean duration for each edge using the defined nav stat range
        """


        msg_store = MessageStoreProxy(collection='nav_stats')

        # reset model
        self.edge_to_duration = {}
        self.edge_to_probability = {}

        query_meta={}                    
        if len(self.range) == 2:            
            if self.range[1] < 1:
                upperlim = rospy.Time.now().secs
            else:
                upperlim = self.range[1]
            query_meta["epoch"] = {"$gte": self.range[0], "$lt" : upperlim}    


        for i in self.map.nodes :
            for j in i.edges:
                if j.edge_id not in self.edge_to_duration:                                                 
                    
                    query = {"topological_map": self.map.name, "edge_id": j.edge_id}                
                    stats = msg_store.query(NavStatistics._type, query, query_meta)

                    rospy.loginfo('%s stats for %s' % (len(stats), j.edge_id))

                    # if no stats, use speed the same as speed-based predictor
                    if len(stats) == 0:
                        destination = get_node(self.map, j.node)
                        if j.top_vel >=  0.1:
                            self.edge_to_duration[j.edge_id] = rospy.Duration(get_distance_to_node(i, destination)/j.top_vel)
                        else :
                            self.edge_to_duration[j.edge_id] = rospy.Duration(get_distance_to_node(i, destination)/0.1)
                        self.edge_to_probability[j.edge_id] = 0.0
                    else:
                        durations = np.array([stat.operation_time for stat, meta in stats])
                        self.edge_to_duration[j.edge_id] = rospy.Duration(durations.mean())

                        successes = np.array([1 if stat.status == 'success' else 0 for stat, meta in stats])
                        self.edge_to_probability[j.edge_id] = successes.mean()


if __name__ == '__main__':
    rospy.init_node('topological_prediction')
    epochs=[]
    if len(sys.argv) < 2:
        print("gathering all the stats")
        epochs=[0, rospy.get_rostime().to_sec()]
    else:
        if len(sys.argv) == 3:
            epochs.append(int(sys.argv[1]))
            epochs.append(int(sys.argv[2]))
            print(epochs)
        else: 
            usage()
            sys.exit(1)
    server = TopologicalMeanPred(epochs)