#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import topological_navigation.msg


def topol_nav_client(targ):
    
    
    client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
    
    client.wait_for_server()
    rospy.loginfo(" ... Init done")

    navgoal = topological_navigation.msg.GotoNodeGoal()

    print "Requesting Navigation to %s" %targ

    navgoal.target = targ
    #navgoal.origin = orig

    # Sends the goal to the action server.
    client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    print 'Argument List:',str(sys.argv)
    if len(sys.argv) < 2 :
	sys.exit(2)
    rospy.init_node('topol_nav_test')
    ps = topol_nav_client(sys.argv[1])
    print ps
