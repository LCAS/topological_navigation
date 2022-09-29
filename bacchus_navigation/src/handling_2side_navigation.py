#! /usr/bin/env python

import rospy
import actionlib
import topological_navigation_msgs.srv
from std_msgs.msg import String
from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal
from bacchus_navigation.msg import GoToAction, GoToFeedback, GoToResult
import time

class GoTo(object): 

    def __init__(self):
        # variables
        self.goal_finished = True
        self.inside_corridor = False
        self.current_side = "none"

        # services client
        rospy.loginfo("Waiting for service: get_simple_policy/get_route_to")
        rospy.wait_for_service('get_simple_policy/get_route_to')
        self.get_route_service = rospy.ServiceProxy('get_simple_policy/get_route_to', topological_navigation_msgs.srv.GetRouteTo)
        rospy.loginfo("Service get_simple_policy/get_route_to active")

        rospy.loginfo("Waiting for service: restrictions_manager/restrict_planning_map")
        rospy.wait_for_service('restrictions_manager/restrict_planning_map')
        self.apply_restriction_service = rospy.ServiceProxy('restrictions_manager/restrict_planning_map',topological_navigation_msgs.srv.RestrictMap)
        rospy.loginfo("Service restrictions_manager/restrict_planning_map active")

        # action client
        self.toponav_goal_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)

        # subscribers
        rospy.Subscriber("GoTo", String, self.GoTo_callback)

        # publishers
        
        # action server
        self.goto_as = actionlib.SimpleActionServer('GoTo', GoToAction, execute_cb=self.execute_cb, auto_start = False)
        self.goto_as.start()

        rospy.loginfo("GoTo Action server ready!")

    def execute_cb(self,goal):
        _feedback = GoToFeedback()
        _result = GoToResult()

        goal_node = goal.node_name
        rospy.loginfo("-----------------------------")
        rospy.loginfo("GoTo request received - Goal: "+goal_node)

        # compute the path to the goal using the complete topomap
        try:
            resp = self.get_route_service(goal_node)
            path = resp.route.source
        except:
            rospy.loginfo("Error: The goal is the current robot position or the node name does not exist")
            _result.message = "Error: The goal is the current robot position or the node name does not exist"
            _result.status = 2
            self.goto_as.set_succeeded(_result)
            return

        # divide the path in subgoal to implement restrictions
        subgoals = []
        for node in path:
            if node[-2] == "s":
                subgoals.append(node)
        subgoals.append(goal_node)
        rospy.loginfo("Subgoals: "+str(subgoals))

        # iterate over all subgoals
        for subgoal_node in subgoals:
            subgoal_msg = GotoNodeGoal()
            subgoal_msg.target = subgoal_node

            # send the subgoal to the topological navigation
            self.toponav_goal_client.send_goal(subgoal_msg, done_cb = self.toponav_goal_done)
            self.subgoal_reached = False
            self.toponav_goal_error = False

            # wait until the robot has reached the goal or the goal is cancelled
            while self.subgoal_reached == False and self.toponav_goal_error==False: #while the goal has been reached or no toponav errors are reported
                if self.goto_as.is_preempt_requested():
                    self.toponav_goal_client.cancel_all_goals()
                    rospy.loginfo("The goal has been cancelled")
                    _result.message = "The goal has been cancelled"
                    _result.status = 3
                    self.goto_as.set_succeeded(_result)
                    return

                _feedback.message = "Navigating to " + str(subgoal_node)
                self.goto_as.publish_feedback(_feedback)
                time.sleep(1)

            if self.toponav_goal_error == True:
                rospy.loginfo("Error in the topological navigation")
                _result.message = "Error in the topological navigation"
                _result.status = 2
                self._as.set_succeeded(_result)
                return

            rospy.loginfo("Arrived to "+ subgoal_node)
            _feedback.message = "Arrived to " + str(subgoal_node)
            self.goto_as.publish_feedback(_feedback)

            # apply restrictions to block a certain side so the topo localisation is not confused
            try:
                if subgoal_node[-2:] == "sa":
                    self.apply_restriction_service('{"allowedside": "a"}',True)
                    rospy.loginfo("Restrictions applied")
                    _feedback.message = "Restrictions applied"
                    self.goto_as.publish_feedback(_feedback)
                if subgoal_node[-2:] == "sb":
                    self.apply_restriction_service('{"allowedside": "b"}',True)
                    rospy.loginfo("Restrictions applied")
                    _feedback.message = "Restrictions applied"
                    self.goto_as.publish_feedback(_feedback)
            except:
                rospy.loginfo("Error applying the node restrictions.")
                self.toponav_goal_client.cancel_all_goals()
                _result.message = "Error applying the node restrictions."
                _result.status = 2
                self.goto_as.set_succeeded(_result)
                return 

        rospy.loginfo("GoTo request completed!")
        rospy.loginfo("-----------------------------")
        _result.message = "Goal reached"
        _result.status = 1
        self.goto_as.set_succeeded(_result)


    def toponav_goal_done(self, state, result ):
        if result:
            self.subgoal_reached = True
        else:
            self.toponav_goal_error = True

    def GoTo_callback(self,msg):
        #if self.goal_finished: #send a new goal
        self.GoTo(msg.data)
        #else: #cancel goal and 

    def GoTo(self,goal_node_name):
        # compute the path to the goal
        try:
            resp = self.get_route_service(goal_node_name)
            path = resp.route.source
        except:
            rospy.loginfo("Error: The goal is the current robot position or the node name does not exist")
            return 0
        
        # divide the path to implement restrictions
        subgoals = []
        for node in path:
            if node[-2] == "s":
                subgoals.append(node)
        subgoals.append(goal_node_name)
        print("subgoals",subgoals)

        for subgoal_node in subgoals:
            subgoal_msg = GotoNodeGoal()
            subgoal_msg.target = subgoal_node
            self.toponav_goal_client.send_goal(subgoal_msg)
            self.toponav_goal_client.wait_for_result()

            if self.toponav_goal_client.get_result():
                print("arrived to " + subgoal_node)
                if subgoal_node[-2:] == "sa":
                    self.apply_restriction_service('{"allowedside": "a"}',True)
                    #self.allowed_side_pub.publish("a")
                if subgoal_node[-2:] == "sb":
                    self.apply_restriction_service('{"allowedside": "b"}',True)
                    #self.allowed_side_pub.publish("b")
            else:
                print("error in navigation")


if __name__ == '__main__':
    rospy.init_node('GoTo_node')
    goto = GoTo()
    rospy.spin()