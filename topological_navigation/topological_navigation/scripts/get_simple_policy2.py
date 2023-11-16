#!/usr/bin/env python

"""
Created on Tue Nov 17 22:02:24 2023
@author: Geesara Kulathunga (ggeesara@gmail.com)

"""

import sys
import rclpy, json 
from std_msgs.msg import String
from topological_navigation_msgs.srv import GetRouteTo, GetRouteBetween
from topological_navigation.route_search2 import RouteChecker, TopologicalRouteSearch2, get_route_distance
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

class SearchPolicyServer(rclpy.node.Node):
       
    def __init__(self, name):
        super().__init__(name)           
        self._map_received = False
        self.qos = QoSProfile(depth=1, 
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mapper_sub = self.create_subscription(String, '/topological_map_2', self.map_callback, qos_profile=self.qos)      
        self.get_logger().info("Waiting for Topological map ...")   
        while rclpy.ok():
            rclpy.spin_once(self)
            if self._map_received:
                self.get_logger().info("Navigation received the Topological Map")
                break 
        self._top_loc=False
        self.get_logger().info("Waiting for Topological localisation ...")
        self.closest_node_sub = self.create_subscription(String, 'closest_node', self.closest_node_callback, qos_profile=self.qos)      
        while rclpy.ok():
            rclpy.spin_once(self)
            if self._top_loc:
              self.get_logger().info("Localisation model is up...")
              break
             
        self.service_to_get_goal = self.create_service(GetRouteTo, 'get_simple_policy/get_route_to', self.get_route_cb)
        self.service_to_get_path = self.create_service(GetRouteBetween, 'get_simple_policy/get_route_between', self.get_routeb_cb)
        self.get_logger().info("All Done ...")


    def get_route_cb(self, req, res):
        route = self.rsearch.search_route(self.closest_node, req.goal)
        print(route)
        res.route = route 
        return res 

    def get_routeb_cb(self, req, res):
        route = self.rsearch.search_route(req.origin, req.goal)
        print(route)
        res.route =  route 
        return res         

    def map_callback(self, msg) :
        self.lnodes = json.loads(msg.data)
        self.topol_map = self.lnodes["pointset"]
        self.rsearch = TopologicalRouteSearch2(self.lnodes)
        self._map_received = True

    def closest_node_callback(self, msg):
        self.closest_node=msg.data
        self._top_loc=True

def main(args=None):
    rclpy.init(args=args)
    node = SearchPolicyServer('simple_policy_server')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('shutting down simple_policy_server node\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
    main()


