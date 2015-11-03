#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String

import strands_navigation_msgs.srv
from strands_navigation_msgs.msg import TopologicalMap
import topological_navigation.route_search
#from topological_navigation.route_search import *
        

class SearchPolicyServer(object):
       
    def __init__(self) :     

        #Waiting for Topological Map        
        self._map_received=False
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while not self._map_received :
            rospy.sleep(rospy.Duration.from_sec(0.05))
        rospy.loginfo(" ...done")

        self._top_loc=False
        rospy.loginfo("Waiting for Topological localisation ...")
        rospy.Subscriber('/closest_node', String, self.closestNodeCallback)
        while not self._top_loc :
            rospy.sleep(rospy.Duration.from_sec(0.05))
        rospy.loginfo(" ...done")


        #This service returns any given map
        self.get_map_srv=rospy.Service('/get_simple_policy/get_route_to', strands_navigation_msgs.srv.GetRouteTo, self.get_route_cb)
        self.get_map_srv=rospy.Service('/get_simple_policy/get_route_between', strands_navigation_msgs.srv.GetRouteBetween, self.get_routeb_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()


    def get_route_cb(self, req):
        rsearch = topological_navigation.route_search.TopologicalRouteSearch(self.top_map)
        route = rsearch.search_route(self.closest_node, req.goal)
        print route
        return route

    def get_routeb_cb(self, req):
        rsearch = topological_navigation.route_search.TopologicalRouteSearch(self.top_map)
        route = rsearch.search_route(req.origin, req.goal)
        print route
        return route        


    """
     Update Map CallBack
     
     This Function updates the Topological Map everytime it is called
    """
    def MapCallback(self, msg) :
        self.top_map = msg
        self._map_received=True


    """
     Closest Node CallBack
     
    """
    def closestNodeCallback(self, msg):
        self.closest_node=msg.data
        self._top_loc=True





if __name__ == '__main__':
    rospy.init_node('route_search')
    searcher = SearchPolicyServer()