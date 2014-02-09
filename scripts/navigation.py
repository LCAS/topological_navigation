#!/usr/bin/env python

import rospy
import actionlib
import pymongo
import json
import sys


from topological_navigation.topological_node import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from std_msgs.msg import String
import scitos_apps_msgs.msg
import strands_datacentre.util
import topological_navigation.msg



""" Class for Topological Navigation """

class TopologicalNavServer(object):
    _feedback = topological_navigation.msg.GotoNodeFeedback()
    _result   = topological_navigation.msg.GotoNodeResult()

    def __init__(self, name, filename) :
        self.cancelled = False
        self.current_node = "Unknown"
        self.actions_needed=[]
        
        self._action_name = name
        rospy.loginfo("Loading file from map %s", filename)
        self.lnodes = self.loadMap(filename)
        rospy.loginfo(" ...done")

       
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, topological_navigation.msg.GotoNodeAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        if 'move_base' in self.actions_needed:
            #print "move_base needed"
            rospy.loginfo("Creating base movement client.")
            self.baseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.baseClient.wait_for_server()
            rospy.loginfo(" ...done")

        if 'ramp_climb' in self.actions_needed:
            #print "ramp_climb needed"        
            rospy.loginfo("Creating Ramp Client")
            self.rampClient = actionlib.SimpleActionClient('rampClimbingServer', scitos_apps_msgs.msg.RampClimbingAction)
            self.rampClient.wait_for_server()
            rospy.loginfo(" ...done")
        
        rospy.loginfo("Subscribing to Topic")
        rospy.Subscriber('/closest_node', String, self.NodeCallback)
        rospy.loginfo(" ...done")
        

        rospy.loginfo("All Done ...")
        rospy.spin()

    def executeCallback(self, goal):
        self.cancelled = False
        self._feedback.route = 'Starting...'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('%s: Navigating From %s to %s', self._action_name, self.current_node, goal.target)
        Onode = get_node(self.current_node, self.lnodes)
        Gnode = get_node(goal.target, self.lnodes)
        if (Gnode is not None) and (Onode is not None) and (Gnode != Onode) :
            exp_index=0
            to_expand=[Onode]
            to_expand[exp_index]._set_Father('none')
            children=to_expand[exp_index]._get_Children()
            not_goal=True
            while not_goal :
                pos=findInList(goal.target, children)
                if pos>=0 :
                    print "Goal found in Pos %d" %pos
                    not_goal=False
                else :
                    print "Goal NOT found"
                    update_to_expand(to_expand, children, self.lnodes, to_expand[exp_index].name)
                    exp_index=exp_index+1
                    print "nodes to expand %d:" %len(to_expand)
                    for m in to_expand :
                        print m.name
                    print "expanding node %d: (%s)" %(exp_index,to_expand[exp_index].name)
                    if exp_index >= len(to_expand) :
                        not_goal=False
                    children=to_expand[exp_index]._get_Children()
                    print "nodes in list:"
                    print children
        
            print "fixing Father %s for goal %s" %(to_expand[exp_index].name,Gnode.name)
            Gnode._set_Father(to_expand[exp_index].name)
            print "Father for Gnode %s" %(Gnode.father)
            route=[Gnode]
            #del route[:]
            print "Current Route %d" %len(route)
            rindex=0
            print route[rindex].father
            while route[rindex].father is not 'none' :
                nwnode = get_node(route[rindex].father, to_expand)
                route.append(nwnode)
                rindex=rindex+1
            
            route.reverse()
            result=self.followRoute(route)

        else :
            if(Gnode == Onode) :
                rospy.loginfo("Target and Origin Nodes are the same")  
                result=self.move_base_to_waypoint(Gnode.waypoint)
                rospy.loginfo("going to waypoint in node resulted in")
                print result
            else:
                rospy.loginfo("Target or Origin Nodes were not found on Map")  
                result=False
            
        if not self.cancelled :
            self._result.success = result
            self._feedback.route = goal.target
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)

    
    def NodeCallback(self, msg):
        self.current_node=msg.data


    def followRoute(self, route):
        rospy.loginfo("%d Nodes on route" %len(route))
        #movegoal = MoveBaseGoal()
        rindex=0
        nav_ok=True
        while rindex < (len(route)-1) and not self.cancelled and nav_ok :
            a = route[rindex]._get_action(route[rindex+1].name)
            print "From %s do (%s) to %s" %(route[rindex].name, a, route[rindex+1].name)

            if a == 'move_base' :
                print "move_base to:"
                inf = route[rindex+1].waypoint
                print inf
                nav_ok=self.move_base_to_waypoint(inf)
            elif a == 'ramp_climbing' :
                print "ramp_climbing"
                rampgoal = scitos_apps_msgs.msg.RampClimbingGoal()
                rampgoal.timeOut = 1000
                print "sending goal"
                print rampgoal
                self.rampClient.send_goal(rampgoal)
                self.rampClient.wait_for_result()
                if self.rampClient.get_state() != GoalStatus.SUCCEEDED:
                    nav_ok=False
            rindex=rindex+1
        result=nav_ok
        return result

    def move_base_to_waypoint(self, inf):
        result = True
        movegoal = MoveBaseGoal()
        movegoal.target_pose.header.frame_id = "map"
        movegoal.target_pose.header.stamp = rospy.get_rostime()
        movegoal.target_pose.pose.position.x = float(inf[0])
        movegoal.target_pose.pose.position.y = float(inf[1])
        movegoal.target_pose.pose.orientation.x = 0
        movegoal.target_pose.pose.orientation.y = 0
        movegoal.target_pose.pose.orientation.z = float(inf[5])
        movegoal.target_pose.pose.orientation.w = float(inf[6])
        self.baseClient.cancel_all_goals()
        rospy.sleep(rospy.Duration.from_sec(1))
        print movegoal
        self.baseClient.send_goal(movegoal)
        self.baseClient.wait_for_result()
        if self.baseClient.get_state() != GoalStatus.SUCCEEDED:
            result = False
        rospy.sleep(rospy.Duration.from_sec(0.3))
        return result

    def preemptCallback(self):
        self.cancelled = True
        self._result.success = False
        self._as.set_preempted(self._result)
        
    def loadMap(self, pointset):

        pointset=str(sys.argv[1])
        host = rospy.get_param("datacentre_host")
        port = rospy.get_param("datacentre_port")
        print "Using datacentre  ",host,":", port
        client = pymongo.MongoClient(host, port)
        db=client.autonomous_patrolling
        points_db=db["waypoints"]
        available = points_db.find().distinct("meta.pointset")
        #print available
        
        if pointset not in available :
            rospy.logerr("Desired pointset '"+pointset+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
        
        points = []
        search =  {"meta.pointset": pointset}
        for point in points_db.find(search) :
            a= point["meta"]["name"]
            b = topological_node(a)
            b.edges = point["meta"]["edges"]
            b.waypoint = point["meta"]["waypoint"]
            points.append(b)

        #print "Actions Needed"
        for i in points:
            for k in i.edges :
                j = k['action']
                if j not in self.actions_needed:
                    self.actions_needed.append(j)
        return points


if __name__ == '__main__':
    filename=str(sys.argv[1])
    rospy.init_node('topological_navigation')
    server = TopologicalNavServer(rospy.get_name(),filename)