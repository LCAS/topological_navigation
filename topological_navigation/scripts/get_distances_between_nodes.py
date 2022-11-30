#!/usr/bin/env python

import sys
import rospy
import topological_navigation_msgs.srv
from  topological_navigation_msgs.msg import DistanceBetweenNodes
import itertools

class GetDistancesBetweenNodesServer(object):
       
    def __init__(self) :

        # services client
        rospy.loginfo("Waiting for service: get_simple_policy/get_route_between")
        rospy.wait_for_service('get_simple_policy/get_route_between')
        self.get_route_service = rospy.ServiceProxy('get_simple_policy/get_route_between', topological_navigation_msgs.srv.GetRouteBetween)
        rospy.loginfo("Service get_simple_policy/get_route_between active")

        #This service returns any given map
        rospy.loginfo("Setting get_distances_between_nodes service")
        self.get_distances_srv=rospy.Service('get_distances_between_nodes', topological_navigation_msgs.srv.GetDistancesBetweenNodes, self.get_distances_cb)
        rospy.loginfo("All Done! Ready to receive requests")
        rospy.spin()


    def get_distances_cb(self, req):
        rospy.loginfo("Request received with the following nodes:")
        rospy.loginfo(req.nodes_list)
        all_distances_message = {'distance_between_nodes': []}

        # check we have at least 2 nodes 
        if len(req.nodes_list) == 1:
            rospy.logerr("Error: you need to pass at least 2 points")
            return {'distance_between_nodes': []}

        # go over all the combinations of pairs of nodes and call the service that tells us the travel distance
        all_combinations = itertools.permutations(req.nodes_list,2)     
        for c in all_combinations:
            origin_node = c[0]
            destination_node = c[1]

            try:
                resp = self.get_route_service(origin_node, destination_node)
                distance  = resp.route.distance
            except:
                rospy.logerr("Error calling the 'get_simple_policy/get_route_between' service. Check the nodes names.")
                return {'distance_between_nodes': []}

            distance_between_nodes_msg = DistanceBetweenNodes()
            distance_between_nodes_msg.orig_node = origin_node
            distance_between_nodes_msg.dest_node = destination_node
            distance_between_nodes_msg.distance = distance

            all_distances_message["distance_between_nodes"].append(distance_between_nodes_msg)

        return all_distances_message

if __name__ == '__main__':
    rospy.init_node('get_distances_between_nodes')
    distance_server = GetDistancesBetweenNodesServer()
