#!/usr/bin/env python

import unittest
import rospy
import yaml
from std_msgs.msg import String
from topological_navigation.route_search2 import TopologicalRouteSearch2
from topological_navigation_msgs.srv import RestrictMap, RestrictMapRequest, RestrictMapResponse

class TestRestrictions(unittest.TestCase):

    def restr_tmap_cb(self, msg):
        self.restr_tmap = yaml.safe_load(msg.data)

    def tmap_cb(self, msg):
        self.tmap = yaml.safe_load(msg.data)
    
    # only functions with 'test_'-prefix will be run!
    def setUp(self):
        rospy.init_node("restrictions_tester")

        rospy.Subscriber("/restricted_topological_map_2", String, self.restr_tmap_cb)

        rospy.Subscriber("/topological_map_2", String, self.tmap_cb)

        self.tmap = None
        self.restr_tmap = None

    def test_maps_received(self):

        rate = rospy.Rate(1) # check every second
        max_seconds = 10
        count = 0
        
        while not rospy.is_shutdown() and count < max_seconds and\
                (self.restr_tmap is None or self.tmap is None): 
            count += 1
            rate.sleep()

        self.assertTrue(self.restr_tmap is not None)
        self.assertTrue(self.tmap is not None)

    def test_restriction_tall_robot(self):

        # restrict the planning map for a tall robot
        srvpr = rospy.ServiceProxy("/restrictions_manager/restrict_planning_map", RestrictMap)

        req = RestrictMapRequest()
        req.state = '{"type": "tall"}'

        res = srvpr.call(req)
        self.assertTrue(res.success)

        if res.success:
            restricted_map = yaml.safe_load(res.restricted_tmap)
            rsearch = TopologicalRouteSearch2(restricted_map)

            _r = rsearch.search_route("WayPoint141", "WayPoint66")
            # this should contain the nodes ....
            should_contain = ["WayPoint73", "WayPoint74"]
            for node in should_contain:
                self.assertIn(node, _r.source)
            
            # this should return a viable path
            _r = rsearch.search_route("WayPoint141", "WayPoint67")

            self.assertGreater(len(_r.edge_id), 0)

    def test_restriction_short_robot(self):
        # restrict the planning map for a tall robot
        srvpr = rospy.ServiceProxy("/restrictions_manager/restrict_planning_map", RestrictMap)

        req = RestrictMapRequest()
        req.state = '{"type": "short"}'

        res = srvpr.call(req)
        self.assertTrue(res.success)

        if res.success:
            restricted_map = yaml.safe_load(res.restricted_tmap)
            rsearch = TopologicalRouteSearch2(restricted_map)

            _r = rsearch.search_route("WayPoint141", "WayPoint66")
            # this should contain the nodes ....
            should_contain = ["WayPoint140", "WayPoint142"]
            for node in should_contain:
                self.assertIn(node, _r.source)
            
            # this should not return a viable path
            _r = rsearch.search_route("WayPoint141", "WayPoint67")
            self.assertEqual(len(_r.edge_id), 0)

if __name__ == '__main__':
    PKG = 'topological_navigation'
    import rostest
    rostest.rosrun(PKG, 'TestRestrictions', TestRestrictions)