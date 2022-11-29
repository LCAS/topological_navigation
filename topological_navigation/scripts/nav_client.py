#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import topological_navigation_msgs.msg


class topol_nav_client(object):
    
    def __init__(self, targ) :
        
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation_msgs.msg.GotoNodeAction)
        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")
    
        navgoal = topological_navigation_msgs.msg.GotoNodeGoal()
    
        print("Requesting Navigation to %s" %targ)
    
        navgoal.target = targ
        #navgoal.origin = orig
    
        # Sends the goal to the action server.
        self.client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        print(ps)

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    print('Argument List:',str(sys.argv))
    if len(sys.argv) < 2 :
	sys.exit(2)
    rospy.init_node('topol_nav_test')
    ps = topol_nav_client(sys.argv[1])
    
