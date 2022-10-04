#! /usr/bin/env python

import rospy
import actionlib
import topological_navigation_msgs.srv
from std_msgs.msg import String
from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal


class GoTo(object): 
    def __init__(self):

        rospy.wait_for_service('restrictions_manager/restrict_planning_map')
        self.apply_restriction_service = rospy.ServiceProxy('restrictions_manager/restrict_planning_map',topological_navigation_msgs.srv.RestrictMap)

        print "calling"
        self.apply_restriction_service('{"allowedside": "a"}',True)
        print "Im here"


if __name__ == '__main__':
    rospy.init_node('GoTo_node_test')
    goto = GoTo()
    rospy.spin()