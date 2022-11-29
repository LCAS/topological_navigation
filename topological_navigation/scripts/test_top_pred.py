#!/usr/bin/env python

import sys
import rospy
from topological_navigation_msgs.srv import *


def predict_edges(seconds_from_now):
    rospy.wait_for_service('topological_prediction/predict_edges')
    try:
        get_prediction = rospy.ServiceProxy('topological_prediction/predict_edges', topological_navigation_msgs.srv.PredictEdgeState)
        now = rospy.Time.now()
        prediction_time = now + rospy.Duration(seconds_from_now) 
        print("Requesting prediction for %f"%prediction_time.secs)
        resp1 = get_prediction(prediction_time)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s seconds_from_now"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        seconds_from_now = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    rospy.init_node('topological_prediction_test')
    est = predict_edges(seconds_from_now)
    print("set of repeated edges:")
    print(set([x for x in est.edge_ids if est.edge_ids.count(x) > 1]))
    #print est.edge_id
    
    if len(est.edge_ids) == len(est.probs) and len(est.edge_ids) == len(est.durations):
        print("Good Answer!!! %d" %len(est.probs)
        for i in range(len(est.edge_ids)):
            print(est.edge_ids[i], est.probs[i], est.durations[i].secs))
    else:
        print("something failed")
        print(len(est.edge_ids), len(est.probs), len(est.durations))