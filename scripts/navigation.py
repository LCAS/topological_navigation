#!/usr/bin/env python

import rospy
import actionlib
import pymongo
import json
import sys

from time import sleep
from datetime import datetime
from topological_navigation.topological_node import *
from topological_navigation.navigation_stats import *
from strands_navigation_msgs.msg import MonitoredNavigationAction
from strands_navigation_msgs.msg import MonitoredNavigationGoal
from strands_navigation_msgs.msg import NavStatistics

from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from std_msgs.msg import String
import scitos_apps_msgs.msg
import ros_datacentre.util
import topological_navigation.msg
import dynamic_reconfigure.client


""" Class for Topological Navigation """

class TopologicalNavServer(object):
    _feedback = topological_navigation.msg.GotoNodeFeedback()
    _result   = topological_navigation.msg.GotoNodeResult()


    def __init__(self, name, filename, mode) :
        self.node_by_node = False
        self.cancelled = False
        self.current_node = "Unknown"
        self.closest_node = "Unknown"
        self.actions_needed=[]
        self.navigation_activated=False
        
        self._action_name = name
        rospy.loginfo("Loading file from map %s", filename)
        self.lnodes = self.loadMap(filename)
        self.topol_map = filename
        rospy.loginfo(" ...done")

        if mode == "Node_by_Node" :
            self.node_by_node = True

        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, topological_navigation.msg.GotoNodeAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Creating monitored navigation client.")
        self.monNavClient= actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        self.monNavClient.wait_for_server()
        rospy.loginfo(" ...done")
        
        
        rospy.loginfo("Subscribing to Topics")
        rospy.Subscriber('/closest_node', String, self.closestNodeCallback)
        rospy.Subscriber('/current_node', String, self.currentNodeCallback)
        rospy.loginfo(" ...done")
        
        rospy.loginfo("Creating Reconfigure Client")
        self.client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        config = self.client.get_configuration()
        self.dyt = config['yaw_goal_tolerance']

        self.stats_pub = rospy.Publisher('/TopologicalNavigation/Statistics', NavStatistics)

        rospy.loginfo("All Done ...")
        rospy.spin()

    def executeCallback(self, goal):
        self.cancelled = False
        self._feedback.route = 'Starting...'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('%s: Navigating From %s to %s', self._action_name, self.closest_node, goal.target)
        self.navigate(goal.target)
        
    def navigate(self, target):
        Onode = get_node(self.closest_node, self.lnodes)
        Gnode = get_node(target, self.lnodes)
        if (Gnode is not None) and (Onode is not None) and (Gnode != Onode) :
            exp_index=0
            to_expand=[Onode]
            to_expand[exp_index]._set_Father('none')
            children=to_expand[exp_index]._get_Children()
            not_goal=True
            while not_goal :
                pos=findInList(target, children)
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
            #del route[:]
            route=[Gnode]
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
                result= self.monitored_navigation(Onode.waypoint, 'move_base')
                rospy.loginfo("going to waypoint in node resulted in")
                print result
            else:
                rospy.loginfo("Target or Origin Nodes were not found on Map")  
                result=False
            
        if not self.cancelled :
            self._result.success = result
            self._feedback.route = target
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        else :
            self._result.success = result
            self._feedback.route = self.current_node
            self._as.publish_feedback(self._feedback)
            self._as.set_aborted(self._result)
    
    def closestNodeCallback(self, msg):
        self.closest_node=msg.data


    def currentNodeCallback(self, msg):
        if self.current_node != msg.data and msg.data != 'none' :
            self.current_node = msg.data
            print "new node reached %s" %self.current_node
            if self.navigation_activated :
                self.stat.set_at_node()
                if self.current_node != self.stat.target and self.node_by_node :
                    self.monNavClient.cancel_all_goals()
                    self.cancelled = True
                  

    def followRoute(self, route):
        nnodes=len(route)
        Orig = route[0].name
        Targ = route[nnodes-1].name
        self.stat=nav_stats(Orig, Targ, self.topol_map)
        dt_text=self.stat.get_start_time_str()
        rospy.loginfo("%d Nodes on route" %nnodes)
        rospy.loginfo("navigation started on %s" %dt_text)
        self.navigation_activated=True
        #movegoal = MoveBaseGoal()
        rindex=0
        nav_ok=True
        route_len = len(route)-2
        while rindex < (len(route)-1) and not self.cancelled and nav_ok :
            a = route[rindex]._get_action(route[rindex+1].name)
            print "From %s do (%s) to %s" %(route[rindex].name, a, route[rindex+1].name)

            if rindex < route_len:
                #self.dyt = config['yaw_goal_tolerance']
                params = { 'yaw_goal_tolerance' : 6.283 }
                config = self.client.update_configuration(params)
            print "move_base to:"
            inf = route[rindex+1].waypoint
            print inf
            nav_ok= self.monitored_navigation(inf, a)
            #self.dyt = config['yaw_goal_tolerance']
            params = { 'yaw_goal_tolerance' : self.dyt }
            config = self.client.update_configuration(params)
            
            rindex=rindex+1
            
        if self.cancelled :
            nav_ok=False
            nodewp = get_node(self.current_node, self.lnodes)
            #params = { 'yaw_goal_tolerance' : 6.283 }
            #config = self.client.update_configuration(params)
            #not_fatal=self.move_base_to_waypoint(nodewp.waypoint)
            
            not_fatal = self.monitored_navigation(nodewp.waypoint, 'move_base')
            #params = { 'yaw_goal_tolerance' : self.dyt }
            #config = self.client.update_configuration(params)
        else :
            not_fatal=nav_ok
        
        self.stat.set_ended(self.current_node)
        dt_text=self.stat.get_finish_time_str()
        operation_time = self.stat.operation_time
        time_to_wp = self.stat.time_to_wp
        result=nav_ok
      
        if nav_ok :
            self.stat.status= "success"
            rospy.loginfo("navigation finished on %s (%d/%d)" %(dt_text,operation_time,time_to_wp))
        else :
            if not_fatal :
                rospy.loginfo("navigation failed on %s (%d/%d)" %(dt_text,operation_time,time_to_wp))
                self.stat.status= "failed"
            else :
                rospy.loginfo("Fatal fail on %s (%d/%d)" %(dt_text,operation_time,time_to_wp))
                self.stat.status= "fatal"
        
        val=self.stat.__dict__
        #rospy.loginfo("%s" %val)
        #print val
        self._stats_collection.insert(val)
        self.navigation_activated=False
        
        pubst = NavStatistics()
        pubst.status = self.stat.status
        pubst.origin = self.stat.origin
        pubst.target = self.stat.target
        pubst.topological_map = self.stat.topological_map
        pubst.final_node = self.stat.final_node
        pubst.time_to_waypoint = self.stat.time_to_wp
        pubst.operation_time = self.stat.operation_time
        pubst.date_started = self.stat.get_start_time_str()
        pubst.date_at_node = self.stat.date_at_node.strftime('%A, %B %d, at %H:%M:%S hours')
        pubst.date_finished = self.stat.get_finish_time_str()
        self.stats_pub.publish(pubst)
        
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
        #print movegoal
        self.baseClient.send_goal(movegoal)
        self.baseClient.wait_for_result()
        if self.baseClient.get_state() != GoalStatus.SUCCEEDED:
            result = False
        rospy.sleep(rospy.Duration.from_sec(0.3))
        return result


    def monitored_navigation(self, inf, command):
        result = True
        goal=MonitoredNavigationGoal()
        goal.action_server=command
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = float(inf[0])
        goal.target_pose.pose.position.y = float(inf[1])
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = float(inf[5])
        goal.target_pose.pose.orientation.w = float(inf[6])
        
        self.monNavClient.send_goal(goal)
        self.monNavClient.wait_for_result()
        if self.monNavClient.get_state() != GoalStatus.SUCCEEDED:
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
        self.mongo_client = pymongo.MongoClient(host, port)
        db=self.mongo_client.autonomous_patrolling
        points_db=db["waypoints"]
        
        self._stats_collection = db.nav_stats
        
        #available = points_db.find().distinct("meta.pointset")
        available = points_db.find().distinct("_meta.pointset")
        #print available
        
        if pointset not in available :
            rospy.logerr("Desired pointset '"+pointset+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
        
        points = []
        search =  {"_meta.pointset": pointset}
        for point in points_db.find(search) :
            a= point["_meta"]["name"]
            b = topological_node(a)
            b.edges = point["_meta"]["edges"]
            b.waypoint = point["_meta"]["waypoint"]
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
    mode="normal"
    if len(sys.argv) > 2:
        print str(sys.argv[2])
        if str(sys.argv[2]) == "true":
            mode="Node_by_Node"
            print "Node_by_Node"
    rospy.init_node('topological_navigation')
    server = TopologicalNavServer(rospy.get_name(),filename,mode)