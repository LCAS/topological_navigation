#!/usr/bin/env python

import rospy
import actionlib
import pymongo
import json
import sys

import calendar
from time import sleep
from datetime import datetime

from strands_navigation_msgs.msg import MonitoredNavigationAction
from strands_navigation_msgs.msg import MonitoredNavigationGoal
from strands_navigation_msgs.msg import NavStatistics

from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from std_msgs.msg import String
import scitos_apps_msgs.msg

from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap
from ros_datacentre.message_store import MessageStoreProxy
from topological_navigation.topological_node import *
from topological_navigation.navigation_stats import *

import topological_navigation.msg
import dynamic_reconfigure.client



"""
 Class for Topological Navigation 

"""

class TopologicalNavServer(object):
    _feedback = topological_navigation.msg.GotoNodeFeedback()
    _result   = topological_navigation.msg.GotoNodeResult()

    """
     Initialization for Topological Navigation Class
    
    """
    def __init__(self, name, mode) :
        self.node_by_node = False
        self.cancelled = False
        self.preempted = False
        self._target = "None"
        self.current_node = "Unknown"
        self.closest_node = "Unknown"
        self.actions_needed=[]
        self.navigation_activated=False
        self._action_name = name
        self.stats_pub = rospy.Publisher('/topological_navigation/Statistics', NavStatistics)
        
        #Waiting for Topological Map        
        self.lnodes = []
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while len(self.lnodes) == 0:
            pass
        rospy.loginfo(" ...done")
        rospy.set_param('topological_map_name', self.topol_map)


        #Choosing operation mode 
        if mode == "Node_by_Node" :
            #self.node_by_node = True
            rospy.set_param('/topological_navigation/mode','Node_by_Node')
            self.nav_mode = "Node_by_Node"
        else:
            rospy.set_param('/topological_navigation/mode','Normal')
            self.nav_mode = "Normal"


        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, topological_navigation.msg.GotoNodeAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        #Creating monitored navigation client
        rospy.loginfo("Creating monitored navigation client.")
        self.monNavClient= actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        self.monNavClient.wait_for_server()
        rospy.loginfo(" ...done")
        
        
        #Subscribing to Localisation Topics
        rospy.loginfo("Subscribing to Localisation Topics")
        rospy.Subscriber('/closest_node', String, self.closestNodeCallback)
        rospy.Subscriber('/current_node', String, self.currentNodeCallback)
        rospy.loginfo(" ...done")

        
        #Creating Reconfigure Client
        rospy.loginfo("Creating Reconfigure Client")
        self.rcnfclient = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        config = self.rcnfclient.get_configuration()
        self.dyt = config['yaw_goal_tolerance']


        rospy.loginfo("All Done ...")
        rospy.spin()


    """
     Update Map CallBack
     
     This Function updates the Topological Map everytime it is called
    """
    def MapCallback(self, msg) :
        self.topol_map = msg.pointset
        points = []
        for i in msg.nodes : 
            b = topological_node(i.name)
            edges = []
            for j in i.edges :
                data = {}
                data["node"]=j.node
                data["action"]=j.action
                edges.append(data)
            b.edges = edges
            verts = []
            for j in i.verts :
                data = [j.x,j.y]
                verts.append(data)
            b._insert_vertices(verts)  
            c=i.pose
            waypoint=[str(c.position.x), str(c.position.y), str(c.position.z), str(c.orientation.x), str(c.orientation.y), str(c.orientation.z), str(c.orientation.w)]
            b.waypoint = waypoint
            b._get_coords()
            points.append(b)
        
        for i in points:
            for k in i.edges :
                j = k['action']
                if j not in self.actions_needed:
                    self.actions_needed.append(j)
        self.lnodes = points


    """
     Execute CallBack
     
     This Functions is called when the Action Server is called
    """
    def executeCallback(self, goal):
        self.cancelled = False
        self.preempted = False
        self._feedback.route = 'Starting...'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('%s: Navigating From %s to %s', self._action_name, self.closest_node, goal.target)
        self.navigate(goal.target)


    """
     Preempt CallBack
    """
    def preemptCallback(self):
        self.cancelled = True
        self.preempted = True
        self._result.success = False
        self.monNavClient.cancel_all_goals()
        #self._as.set_preempted(self._result)


    """
     Closest Node CallBack
     
    """
    def closestNodeCallback(self, msg):
        self.closest_node=msg.data



    """
     Current Node CallBack
     
    """
    def currentNodeCallback(self, msg):
        if self.current_node != msg.data and msg.data != 'none' :
            self.current_node = msg.data
            print "new node reached %s" %self.current_node
            if self.navigation_activated :
                self.stat.set_at_node()
                if self.nav_mode != 'Normal' :
                    if self.current_node != self._target or self.nav_mode == 'Node_to_IZ':
                        self.cancelled = True
                        if self.nav_mode != 'Node_to_IZ':
                            self.monNavClient.cancel_all_goals()


 
    """
     Navigate
     
     This function takes the taget node and plans the actions that are required
     to reach it
    """       
    def navigate(self, target):
        Onode = get_node(self.closest_node, self.lnodes)
        Gnode = get_node(target, self.lnodes)
        
        
        if (Gnode is not None) and (Onode is not None) and (Gnode != Onode) :
            route = self.search_route(Onode, target)
            result=self.followRoute(route)

        else :
            if(Gnode == Onode) :
                n_edges=len(Gnode.edges) 
                for i in range(0,n_edges):
                    action_server=Gnode.edges[i]["action"]
                    if  action_server == 'move_base' or  action_server == 'human_aware_navigation':
                        break
                    action_server=None
                        
                    
                rospy.loginfo("Target and Origin Nodes are the same")
                if action_server is None:
                    rospy.loginfo("Action not taken, outputing success")
                    result=True
                else:
                    rospy.loginfo("Getting to exact pose")
                    result= self.monitored_navigation(Onode.waypoint, action_server)
                    rospy.loginfo("going to waypoint in node resulted in")
                    print result
            else:
                rospy.loginfo("Target or Origin Nodes were not found on Map")
                self.cancelled = True
                result=False
        
        
        if (not self.cancelled) and (not self.preempted) :
            self._result.success = result
            self._feedback.route = target
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        else :
            if self.preempted == False :
                self._result.success = result
                self._feedback.route = self.current_node
                self._as.publish_feedback(self._feedback)
                #self._as.set_succeeded(self._result)
                self._as.set_aborted(self._result)
            else :
                if self.nav_mode == 'Node_to_IZ' :
                    self._result.success = result
                    self._feedback.route = self.current_node
                    self._as.publish_feedback(self._feedback)
                    self._as.set_succeeded(self._result)
                else :
                    self._result.success = False
                    self._as.set_preempted(self._result)


    """
     search_route
     
     This function takes the search the route to reach the goal
    """       
    def search_route(self, Origin, target):
        Gnode = get_node(target, self.lnodes)
        exp_index=0
        to_expand=[Origin]
        to_expand[exp_index]._set_Father('none')
        children=to_expand[exp_index]._get_Children()
        not_goal=True
        while not_goal :
            pos=findInList(target, children)
            if pos>=0 :
                not_goal=False
            else :
                #print "Goal NOT found"
                update_to_expand(to_expand, children, self.lnodes, to_expand[exp_index].name)
                exp_index=exp_index+1
                #print "nodes to expand %d:" %len(to_expand)
                #for m in to_expand :
                #    print m.name
                #print "expanding node %d: (%s)" %(exp_index,to_expand[exp_index].name)
                if exp_index >= len(to_expand) :
                    not_goal=False
                children=to_expand[exp_index]._get_Children()
                #print "nodes in list:"
                #print children
                
        Gnode._set_Father(to_expand[exp_index].name)
        #print "Father for Gnode %s" %(Gnode.father)
        #del route[:]
        route=[Gnode]
        #print "Current Route %d" %len(route)
        rindex=0
        #print route[rindex].father
        while route[rindex].father is not 'none' :
            nwnode = get_node(route[rindex].father, to_expand)
            route.append(nwnode)
            rindex=rindex+1
        
        route.reverse()
        return route
                  

    def followRoute(self, route):
        nnodes=len(route)
        self.nav_mode = rospy.get_param('/topological_navigation/mode')

        self.navigation_activated=True
        Orig = route[0].name
        Targ = route[nnodes-1].name
        self._target = Targ
        
        rospy.loginfo("%d Nodes on route" %nnodes)

        rindex=0
        nav_ok=True
        route_len = len(route)-2


        while rindex < (len(route)-1) and not self.cancelled and nav_ok :
            #current action
            a = route[rindex]._get_action(route[rindex+1].name)
            #next action
            if rindex < route_len :
                a1 = route[rindex+1]._get_action(route[rindex+2].name)
            else :
                a1 = 'none'
            
            rospy.loginfo("From %s do (%s) to %s" %(route[rindex].name, a, route[rindex+1].name))

            self._feedback.route = '%s to %s using %s' % (route[rindex].name, route[rindex+1].name, a)
            self._as.publish_feedback(self._feedback)

            self.stat=nav_stats(route[rindex].name, route[rindex+1].name, self.topol_map)
            dt_text=self.stat.get_start_time_str()

            # do not care for the orientation of the waypoint if is not the last waypoint AND 
            # the current and following action are move_base or human_aware_navigation
            if rindex < route_len and (a1 == 'move_base' or a1 == 'human_aware_navigation') and (a == 'move_base' or a == 'human_aware_navigation') :
                params = { 'yaw_goal_tolerance' : 6.283 }
                config = self.rcnfclient.update_configuration(params)

            #print "move_base to:"
            inf = route[rindex+1].waypoint
            #print inf
                        
            nav_ok= self.monitored_navigation(inf, a)
            params = { 'yaw_goal_tolerance' : self.dyt }
            config = self.rcnfclient.update_configuration(params)
            not_fatal=nav_ok
            if self.cancelled :
                if self.nav_mode == 'Node_by_Node' :
                    nav_ok=False
                    nodewp = get_node(self.current_node, self.lnodes)          
                    not_fatal = self.monitored_navigation(nodewp.waypoint, 'move_base')
                if self.nav_mode == 'Node_to_IZ' :
                    not_fatal = False
                    if self.current_node != self._target :
                        nav_ok=False
                    else :
                        nav_ok=True
            if self.preempted :
                not_fatal = False
                nav_ok = False

            self.stat.set_ended(self.current_node)
            dt_text=self.stat.get_finish_time_str()
            operation_time = self.stat.operation_time
            time_to_wp = self.stat.time_to_wp            

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

            self.publish_stats()
            rindex=rindex+1
        
        #val=self.stat.__dict__
        #rospy.loginfo("%s" %val)
        #print val
        #self._stats_collection.insert(val)
        self.navigation_activated=False
                
        result=nav_ok
        return result


    def publish_stats(self):
        pubst = NavStatistics()
        pubst.status = self.stat.status
        pubst.origin = self.stat.origin
        pubst.target = self.stat.target
        pubst.topological_map = self.stat.topological_map
        pubst.final_node = self.stat.final_node
        pubst.time_to_waypoint = self.stat.time_to_wp
        pubst.operation_time = self.stat.operation_time
        pubst.date_started = self.stat.get_start_time_str()
        pubst.date_at_node = self.stat.date_at_node.strftime('%A, %B %d %Y, at %H:%M:%S hours')
        pubst.date_finished = self.stat.get_finish_time_str()
        self.stats_pub.publish(pubst)

        meta = {}
        meta["type"] = "Topological Navigation Stat"
        meta["epoch"] = calendar.timegm(self.stat.date_at_node.timetuple())
        meta["date"] = self.stat.date_at_node.strftime('%A, %B %d %Y, at %H:%M:%S hours')
        meta["pointset"] = self.stat.topological_map
        
        msg_store = MessageStoreProxy()
        msg_store.insert(pubst,meta)        


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
                #        self.monNavClient.wait_for_result()
        status=self.monNavClient.get_state()
        while (status == GoalStatus.ACTIVE or status == GoalStatus.PENDING) and not self.cancelled :
            status=self.monNavClient.get_state()
        
        #rospy.loginfo(str(status))
        #print status
        if status != GoalStatus.SUCCEEDED:
            result = False
            if status is GoalStatus.PREEMPTED:
                self.preempted = True
        rospy.sleep(rospy.Duration.from_sec(0.3))
        return result



if __name__ == '__main__':
    #filename=str(sys.argv[1])
    mode="normal"
#    if len(sys.argv) > 2:
#        print str(sys.argv[2])
#        if str(sys.argv[2]) == "true":
#            mode="Node_by_Node"
#            print "Node_by_Node"
    rospy.init_node('topological_navigation')
    server = TopologicalNavServer(rospy.get_name(),mode)