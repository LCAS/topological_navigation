#!/usr/bin/env python

import sys
import unittest
import topological_navigation.testing
import rospy
import math 
from mongodb_store.message_store import MessageStoreProxy
from topological_navigation.publisher import map_publisher
from topological_navigation.publisher import map_publisher
from strands_navigation_msgs.srv import EstimateTravelTime


class TestTravelTimeEstimator(unittest.TestCase):
# class TestTravelTimeEstimator():
    def __init__(self, *args): 
        super(TestTravelTimeEstimator, self).__init__(*args)
        rospy.init_node('test_travel_time_estimator')

    def test_travel_time_estimator(self):
        
        # create a test topological map
        width = 5 
        height = 5 
        nodeSeparation = 10

        test_nodes = topological_navigation.testing.create_cross_map(width = width, height = height, nodeSeparation = nodeSeparation)
        self.assertEqual(len(test_nodes), width + height - 1)
        startIndex = -(width/2)
        endIndex = (width/2)
        startNode = test_nodes['h_%s' % startIndex]
        self.assertIsNotNone(startNode)
        endNode = test_nodes['h_%s' % endIndex]
        self.assertIsNotNone(endNode)
        
        # locally check distance 
        dist = math.hypot((startNode.pose.position.x-endNode.pose.position.x),(startNode.pose.position.y-endNode.pose.position.y))
        self.assertEqual(dist, (width - 1) * nodeSeparation )

        # now insert the map into the database
        msg_store = MessageStoreProxy(collection='topological_maps')

        meta = {}
        meta['map'] = 'test_travel_time_estimator_map'
        meta['pointset'] = 'test_travel_time_estimator_map'

        for (nodeName, node) in test_nodes.iteritems():
            meta["node"] = nodeName
            node.map = meta['map']
            node.pointset = meta['pointset']
            msg_store.insert(node,meta)
    
        # and publish the map
        ps = map_publisher('test_travel_time_estimator_map')

        # now wait for the distance service
        time_srv_name = 'topological_navigation/travel_time_estimator'
        rospy.wait_for_service(time_srv_name, timeout=10)
        time_srv = rospy.ServiceProxy(time_srv_name, EstimateTravelTime)
        time_estimate = time_srv(startNode.name, endNode.name)
        print time_estimate


if __name__ == '__main__':
    import rostest
    PKG = 'topological_navigation'
    rostest.rosrun(PKG, 'test_travel_time_estimator', TestTravelTimeEstimator)
 
