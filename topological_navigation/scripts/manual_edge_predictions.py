#!/usr/bin/env python

import sys
import rospy
import yaml

import actionlib
from actionlib_msgs.msg import *

from topological_navigation_msgs.srv import *
from topological_navigation_msgs.msg import *


def usage():
    print("Run with a yaml file containing edge prediction information:")
    print("\t rosrun topological_navigation manual_edge_predictions.py <yaml file>")

# Examaple yaml format:
# - edge_id: WayPoint0_WayPoint1
#   duration: 1.0
#   success_probability: 1.0
# - edge_id: WayPoint0_WayPoint4
#   duration: 1.0
#   success_probability: 1.0
# - edge_id: WayPoint1_WayPoint2
#   duration: 1.0
#   success_probability: 1.0


class YamlConfiguredPredictor(object):

    def __init__(self, config) :

        self.response = PredictEdgeStateResponse()

        for edge in config:            
            self.response.edge_ids.append(edge['edge_id'])
            self.response.probs.append(edge['success_probability'])
            self.response.durations.append(rospy.Duration(edge['duration']))

        name= rospy.get_name()
        action_name = name+'/build_temporal_model'

        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(action_name, BuildTopPredictionAction, execute_cb = self.build_callback, auto_start = False)
        self._as.start()


        self.predict_srv=rospy.Service('topological_prediction/predict_edges', PredictEdgeState, self.predict_edge_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()


    def predict_edge_cb(self, req):
        return self.response


    """
     Build predictions from data. Or not.
     
     Does nothing for this version.
    """
    def build_callback(self, goal):    
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('topological_prediction')

    if len(sys.argv) != 2:
        usage()
        sys.exit(1)
    config_file = sys.argv[1]

    
    with open(config_file, "r") as f:
        config = yaml.load(f.read())

    server = YamlConfiguredPredictor(config)
