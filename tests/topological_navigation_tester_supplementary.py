#!/usr/bin/env python

import unittest
import rospy
from strands_navigation_msgs.srv import LoadTopoNavTestScenario, RunTopoNavTestScenario


PKG = 'topological_navigation'


class TestTopologicalNavigation(unittest.TestCase):
    _map_name = 'mb_test'

    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node('topological_navigation_tester')

    def _run(self, map_name):
        try:
            l = rospy.ServiceProxy("/scenario_server/load", LoadTopoNavTestScenario)
            l.wait_for_service(timeout=60.)
            l(map_name)
            s = rospy.ServiceProxy("/scenario_server/start", RunTopoNavTestScenario)
            s.wait_for_service(timeout=60.)
            res = s()
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logfatal(e)
            self.assertTrue(False)
        return res

    def test_static_facing_wall_10cm_distance_0_degrees_goal_behind(self):
        res = self._run(self._map_name+str(1))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_facing_wall_10cm_distance_minus_45_degrees_goal_behind(self):
        res = self._run(self._map_name+str(2))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_facing_wall_10cm_distance_plus_45_degrees_goal_behind(self):
        res = self._run(self._map_name+str(3))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_155cm_corridor_with_55cm_chairs_on_one_side(self):
        res = self._run(self._map_name+str(4))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_210cm_corridor_with_55cm_chairs_on_both_sides(self):
        res = self._run(self._map_name+str(5))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_80cm_wide_door(self):
        res = self._run(self._map_name+str(6))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_70cm_wide_door(self):
        res = self._run(self._map_name+str(7))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_trapped_in_corner(self):
        res = self._run(self._map_name+str(8))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_1m_striaght_corridor(self):
        res = self._run(self._map_name+str(9))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_1m_l_shaped_corridor(self):
        res = self._run(self._map_name+str(10))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_wheelchair_on_intermediate_point(self):
        res = self._run(self._map_name+str(11))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_wheelchair_on_end_point(self):
        res = self._run(self._map_name+str(12))
        rospy.loginfo(res)
        self.assertTrue(res.graceful_fail and not res.nav_timeout)  # Cannot reach final node and navigation should fail without timeout

    def test_static_human_on_intermediate_point(self):
        res = self._run(self._map_name+str(13))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_human_on_end_point(self):
        res = self._run(self._map_name+str(14))
        rospy.loginfo(res)
        self.assertTrue(res.graceful_fail and not res.nav_timeout)  # Cannot reach final node and navigation should fail without timeout

    def test_static_chairs_on_one_side_of_corridor(self):
        res = self._run(self._map_name+str(15))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_wheelchairs_on_one_side_of_corridor(self):
        res = self._run(self._map_name+str(16))
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test_static_corridor_blocked_by_wheelchairs(self):
        res = self._run(self._map_name+str(17))
        rospy.loginfo(res)
        self.assertTrue(res.graceful_fail and not res.nav_timeout) # Cannot reach final node and navigation should fail without timeout

    def test_static_corridor_blocked_by_humans(self):
        res = self._run(self._map_name+str(18))
        rospy.loginfo(res)
        self.assertTrue(res.graceful_fail and not res.nav_timeout) # Cannot reach final node and navigation should fail without timeout


if __name__ == '__main__':
    import rostest

    rostest.rosrun(PKG, 'topological_navigation_tester', TestTopologicalNavigation)

