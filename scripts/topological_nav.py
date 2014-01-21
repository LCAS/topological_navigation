#!/usr/bin/env python

import rospy
import actionlib
import pymongo
import json
import sys

from topological_node import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
import scitos_apps_msgs.msg
import strands_datacentre.util
import topological_navigation.msg


class TopologicalNavServer(object):
    _feedback = topological_navigation.msg.GotoNodeFeedback()
    _result   = topological_navigation.msg.GotoNodeResult()

    def __init__(self, name, filename) :

        self.cancelled = False
        
        self._action_name = name
        print "loading file from map %s" %filename
        self.lnodes = self.loadMap(filename)
        print "Done"
        
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, topological_navigation.msg.GotoNodeAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.baseClient.wait_for_server()
        rospy.loginfo(" ...done")
        
        rospy.loginfo("Creating Ramp Client")
        self.rampClient = actionlib.SimpleActionClient('rampClimbingServer', scitos_apps_msgs.msg.RampClimbingAction)
        self.rampClient.wait_for_server()
        rospy.loginfo(" ...done")

        rospy.loginfo("All Done ...")
        rospy.spin()

    def executeCallback(self, goal):
        self.cancelled = False
        self._feedback.route = 'Starting...'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('%s: Navigating From %s to %s' %(self._action_name, goal.origin, goal.target))
        Onode = get_node(goal.origin, self.lnodes)
        Gnode = get_node(goal.target, self.lnodes)
        if (Gnode is not None) and (Onode is not None) :
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
                    print "nodos para expandir %d:" %len(to_expand)
                    for m in to_expand :
                        print m.name
                    print "expandiendo nodo %d: (%s)" %(exp_index,to_expand[exp_index].name)
                    if exp_index >= len(to_expand) :
                        not_goal=False
                    children=to_expand[exp_index]._get_Children()
                    print "nodos en la lista:"
                    print children
        
            print "fixing Father %s for goal %s" %(to_expand[exp_index].name,Gnode.name)
            Gnode._set_Father(to_expand[exp_index].name)
            print "Father for Gnode %s" %(Gnode.father)
            route=[Gnode]
            #del route[:]
            print "Ruta actual %d" %len(route)
            rindex=0
            print route[rindex].father
            while route[rindex].father is not 'none' :
                nwnode = get_node(route[rindex].father, to_expand)
                route.append(nwnode)
                rindex=rindex+1
            
            route.reverse()
            result=self.followRoute(route)

        else :
            rospy.loginfo("Target or Origin Nodes were not found on Map")  
            result=False #self._send_tweet(goal.text)
            
        if not self.cancelled :
            self._result.success = result
            self._feedback.route = goal.target
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        #del to_expand[:]
        

    def followRoute(self, route):
        rospy.loginfo("%d Nodes on route" %len(route))
        movegoal = MoveBaseGoal()
        rindex=0
        nav_ok=True
        while rindex < (len(route)-1) and not self.cancelled and nav_ok :
            a = route[rindex]._get_action(route[rindex+1].name)
            print "From %s do (%s) to %s" %(route[rindex].name, a, route[rindex+1].name)

            if a == 'move_base' :
                print "move_base to:" 
                #inf = route[rindex+1].waypoint.split(',',7)
                inf = route[rindex+1].waypoint
                print inf
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
                    nav_ok=False
                rospy.sleep(rospy.Duration.from_sec(0.3))
            elif a == 'ramp_climbing' :
                print "ramp_climbing"
                rampgoal = scitos_apps_msgs.msg.RampClimbingGoal()
                rampgoal.timeOut = 1000
                print "sending goal"
                print rampgoal
                self.rampClient.send_goal(rampgoal)
                self.rampClient.wait_for_result()
                #if self.baseClient.get_state() != GoalStatus.SUCCEEDED:
                #    nav_ok=False
            rindex=rindex+1
        result=nav_ok
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
        print available
        if pointset not in available :
            rospy.logerr("Desired pointset '"+pointset+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
        
        #points = self._get_points(waypoints_name) 
        points = []
        search =  {"meta.pointset": pointset}
        for point in points_db.find(search) :
            a= point["meta"]["name"]
            b = topological_node(a)
            b.edges = point["meta"]["edges"]
            b.waypoint = point["meta"]["waypoint"]
            print b.name
            #if point["meta"]["name"] != "charging_point":
            points.append(b)

        return points

if __name__ == '__main__':
    filename=str(sys.argv[1])
    rospy.init_node('topological_navigation')
    server = TopologicalNavServer(rospy.get_name(),filename)