#! /usr/bin/env python


import rospy
from topological_navigation_msgs.msg import TopologicalMap
from topological_navigation_msgs.srv import EstimateTravelTime
from std_msgs.msg import Duration
import math
import threading

class TravelTimeEstimator(object):
    """docstring for TravelTimeEstimator"""
    def __init__(self):
        super(TravelTimeEstimator, self).__init__()
        self.service_lock = threading.Lock()
        self.nodes = dict()
        rospy.Subscriber('topological_map', TopologicalMap, self.map_callback)
                
        while(not rospy.is_shutdown() and len(self.nodes) == 0):
            rospy.sleep(1)

        rospy.Service('topological_navigation/travel_time_estimator', EstimateTravelTime, self.estimate_travel_time)


    def map_callback(self, msg):
        self.nodes.clear()
        for node in msg.nodes:
            self.nodes[node.name] = node


    def estimate_travel_time(self, req):
        self.service_lock.acquire()

        print(req)

        if req.start not in self.nodes:
            raise Exception('Unknown start node: %s' % req.start)
        if  req.target not in self.nodes:
            raise Exception('Unknown target node: %s' % req.target)

        startNode = self.nodes[req.start]    
        endNode = self.nodes[req.target]    

        travel_distance = math.hypot((startNode.pose.position.x-endNode.pose.position.x),(startNode.pose.position.y-endNode.pose.position.y))

        # this is the default scitos speed
        travel_speed_ms = 0.5
        travel_time = rospy.Duration(travel_distance/travel_speed_ms)
        self.service_lock.release()
        return travel_time

if __name__ == '__main__':
    rospy.init_node('travel_time_estimator')
    tte = TravelTimeEstimator()
    rospy.spin()
    
