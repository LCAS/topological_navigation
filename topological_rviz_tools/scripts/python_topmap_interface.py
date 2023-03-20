#!/usr/bin/env python

import rospy
import math
import operator
from std_msgs.msg import Time
from topological_navigation_msgs.msg import TopologicalMap
from topological_navigation_msgs.srv import *
from geometry_msgs.msg import Pose

class TopmapInterface(object):
    """Creates some topics which can be used by c++ code in the rviz portion of this
    package to call into python functions to modify the topological map

    """

    def __init__(self):
        """
        name: the name of the map as recorded in the database. Use
        topological_util/list_maps to see which ones are available

        """
        rospy.init_node("topmap_interface")
        self.name = rospy.get_param('~map_name')
        self.topmap = None
        self.new_nodes = 0

        self.topmap_sub = rospy.Subscriber("/topological_map", TopologicalMap, self.topmap_cb)
        self.add_edge_srv = rospy.Service("~add_edge", topological_navigation_msgs.srv.AddEdgeRviz, self.add_edge)

        self.manager_add_edge = rospy.ServiceProxy("/topological_map_manager/add_edges_between_nodes", topological_navigation_msgs.srv.AddEdge)

        rospy.spin()
    
    def add_edge(self, req):
        def tuple_dist(pose_tuple):
            return math.sqrt(pow(pose_tuple[0].position.x - pose_tuple[1].position.x, 2)
                             + pow(pose_tuple[0].position.y - pose_tuple[1].position.y, 2))

        # Find the nodes closest to the start and end point of the line given in the request
        poses = [node.pose for node in self.topmap.nodes]

        from_dists = map(tuple_dist, zip([req.first]*len(poses), poses))
        to_dists = map(tuple_dist, zip([req.second]*len(poses), poses))
        closest_from_ind, from_dist = min(enumerate(from_dists), key=operator.itemgetter(1))
        closest_to_ind, to_dist = min(enumerate(to_dists), key=operator.itemgetter(1))

        if req.max_distance > 0 and (from_dist > req.max_distance or to_dist > req.max_distance):
            return AddEdgeResponse(True, "Click locations were not close enough to a node")

        from_name = self.topmap.nodes[closest_from_ind].name
        to_name = self.topmap.nodes[closest_to_ind].name

        if from_name == to_name:
            return AddEdgeResponse(True, "Can't have an edge from a node to itself.")

        message = "Added edge from {0} to {1}".format(from_name, to_name)

        self.manager_add_edge(origin=from_name, destination=to_name, action="move_base")
        if req.bidirectional:
            self.manager_add_edge(origin=to_name, destination=from_name, action="move_base")
            message += " (bidirectional)"

        return topological_navigation_msgs.srv.AddEdgeRvizResponse(True, message)

    def topmap_cb(self, msg):
        rospy.loginfo("Topological map was updated via callback.")
        self.topmap = msg

if __name__ == '__main__':
    TopmapInterface()
