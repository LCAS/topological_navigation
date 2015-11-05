#!/usr/bin/env python

import unittest
import rospy
from strands_navigation_msgs.srv import LoadTopoNavTestScenario, RunTopoNavTestScenario


PKG = 'topological_navigation'


class TestTopologicalNavigation(unittest.TestCase):
    _map_names = ['mb_test0', 'mb_test1', 'mb_test2', 'mb_test3', 'mb_test4', 'mb_test5', 'mb_test6', 'mb_test7', 'mb_test8', 'mb_test9', 'mb_test10']

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

    def test0(self):
        res = self._run(self._map_names[0])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test1(self):
        res = self._run(self._map_names[1])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test2(self):
        res = self._run(self._map_names[2])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test3(self):
        res = self._run(self._map_names[3])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test4(self):
        res = self._run(self._map_names[4])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test5(self):
        res = self._run(self._map_names[5])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test6(self):
        res = self._run(self._map_names[6])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test7(self):
        res = self._run(self._map_names[7])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test8(self):
        res = self._run(self._map_names[8])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test9(self):
        res = self._run(self._map_names[9])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)

    def test10(self):
        res = self._run(self._map_names[10])
        rospy.loginfo(res)
        self.assertTrue(res.nav_success)


if __name__ == '__main__':
    import rostest

    rostest.rosrun(PKG, 'topological_navigation_tester', TestTopologicalNavigation)

