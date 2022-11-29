#!/usr/bin/env python

import sys
import rospy
import topological_navigation_msgs.srv
        

class GetDistancesBetweenPointsServer(object):
       
    def __init__(self) :     

        # services client
        rospy.loginfo("Waiting for service: get_simple_policy/get_route_between")
        rospy.wait_for_service('get_simple_policy/get_route_between')
        self.get_route_service = rospy.ServiceProxy('get_simple_policy/get_route_between', topological_navigation_msgs.srv.GetRouteBetween)
        rospy.loginfo("Service get_simple_policy/get_route_between active")

        #This service returns any given map
        rospy.loginfo("Setting get_distances_between_points service")
        self.get_distances_srv=rospy.Service('get_distances_between_points', topological_navigation_msgs.srv.GetDistancesBetweenPoints, self.get_distances_cb)
        rospy.loginfo("All Done! Ready to receive requests")
        rospy.spin()


    def get_distances_cb(self, req):
        print(len(req.points_list))
        # if len(req.points_list) == 1:
        #     rospy.loginfo("Error: you need to pass at least 2 points")
        #     return [],[]
        # try:
        #     resp = self.get_route_service(goal_node)
        #     path = resp.route.source
        # except:
        #     rospy.loginfo("Error: The goal is the current robot position or the node name does not exist")
        #     _result.message = "Error: The goal is the current robot position or the node name does not exist"
        #     _result.status = 2
        #     self.goto_as.set_succeeded(_result)
        #     return


        print req

        combinations = ['r8','r2']
        distances = [1,2]
        return combinations, distances





if __name__ == '__main__':
    rospy.init_node('get_distances_between_points')
    distance_server = GetDistancesBetweenPointsServer()
