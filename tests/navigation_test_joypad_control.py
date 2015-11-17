#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scitos_teleop.msg import action_buttons
from strands_navigation_msgs.srv import RunTopoNavTestScenario
from std_srvs.srv import Empty


class JoyPadControl(object):
    _start = "start"
    _reset = "reset"

    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.action = self._start
        rospy.Subscriber("/teleop_joystick/action_buttons", action_buttons, self.callback)
        rospy.loginfo("... done")

    def callback(self, msg):
        if msg.B:
            self.action = self._start if self.action == self._reset else self._reset
            rospy.loginfo("+++ Current action '%s'. Press 'A' to confirm or 'B' to toggle. +++")
        elif msg.A:
            if self.action == self._start:
                try:
                    s = rospy.ServiceProxy("/scenario_server/start", RunTopoNavTestScenario)
                    s.wait_for_service()
                    rospy.loginfo(s())
                except (rospy.ServiceException, rospy.ROSInterruptException) as e:
                    rospy.logwarn(e)
            elif self.action == self._reset:
                try:
                    s = rospy.ServiceProxy("/scenario_server/reset", Empty)
                    s.wait_for_service()
                    s()
                except (rospy.ServiceException, rospy.ROSInterruptException) as e:
                    rospy.logwarn(e)


if __name__ == "__main__":
    rospy.init_node("navigation_test_joypad_control")
    JoyPadControl(rospy.get_name())
    rospy.spin()
