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
from strands_navigation_msgs.msg import CurrentEdge

from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from std_msgs.msg import String

from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy
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
        self.current_action = 'none'
        self.next_action = 'none'
        self.n_tries = rospy.get_param('~retries', 3)
        

        self.current_node = "Unknown"
        self.closest_node = "Unknown"
        self.actions_needed=[]

        self.move_base_actions = ['move_base','human_aware_navigation']        

        self.navigation_activated=False
        self._action_name = name
        self.stats_pub = rospy.Publisher('/topological_navigation/Statistics', NavStatistics)
        self.edge_pub = rospy.Publisher('/topological_navigation/Edge', CurrentEdge)
        self.cur_edge = rospy.Publisher('/current_edge', String)        
        self.monit_nav_cli= False

        
        #Waiting for Topological Map        
        self.lnodes = []
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while len(self.lnodes) == 0:
            pass
        rospy.loginfo(" ...done")
        rospy.set_param('topological_map_name', self.topol_map)


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
        self.monit_nav_cli= True
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
            c=i.pose[0]
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
        if self.monit_nav_cli :
            self.cancelled = False
            self.preempted = False
            self._feedback.route = 'Starting...'
            self._as.publish_feedback(self._feedback)
            rospy.loginfo('%s: Navigating From %s to %s', self._action_name, self.closest_node, goal.target)
            self.navigate(goal.target)
        else:
            rospy.loginfo('Monitored Navigation client has not started!!!')

    """
     Preempt CallBack
    """
    def preemptCallback(self):
        self.cancelled = True
        self.preempted = True
        self._result.success = False
        self.navigation_activated = False
        self.monNavClient.cancel_all_goals()
        #self._as.set_preempted(self._result)


    """
     Closest Node CallBack
     
    """
    def closestNodeCallback(self, msg):
        self.closest_node=msg.data
        if not self.monit_nav_cli :
            rospy.loginfo('Monitored Navigation client has not started!!!')



    """
     Current Node CallBack
     
    """
    def currentNodeCallback(self, msg):
        if self.current_node != msg.data :  #is there any change on this topic?
            self.current_node = msg.data    #we are at this new node
            if msg.data != 'none' :         #are we at a node?
                rospy.loginfo('new node reached %s', self.current_node)
                #print "new node reached %s" %self.current_node
                if self.navigation_activated :  #is the action server active?
                    self.stat.set_at_node()
                    # if the robot reached and intermediate node and the next action is move base goal has been reached
                    if self.current_node == self.current_target and self._target != self.current_target and self.next_action in self.move_base_actions and self.current_action in self.move_base_actions :
                        rospy.loginfo('intermediate node reached %s', self.current_node)
                        self.goal_reached=True
                        


 
    """
     Navigate
     
     This function takes the target node and plans the actions that are required
     to reach it
    """       
    def navigate(self, target):
        tries=0
        result = False
        while tries <= self.n_tries and not result and not self.cancelled :
            Onode = get_node(self.closest_node, self.lnodes)
            Gnode = get_node(target, self.lnodes)
            
            # Everything is Awesome!!!
            # Target and Origin are Different and none of them is None
            if (Gnode is not None) and (Onode is not None) and (Gnode != Onode) :
                route = self.search_route(Onode, target)
                if route:
                    result, inc = self.followRoute(route)
                else:
                    rospy.logerr("There is no route to this node check your edges ...")
                    result = False
                    inc = 1
            else :
                # Target and Origin are the same
                if(Gnode == Onode) :
                    n_edges=len(Gnode.edges)
                    for i in range(0,n_edges):
                        action_server=Gnode.edges[i]["action"]
                        #if  action_server == 'move_base' or  action_server == 'human_aware_navigation':
                        # Check if there is a move_base action in the edages of this node
                        # if not is dangerous to move
                        if  action_server in self.move_base_actions :
                            break
                        action_server=None
    
    
                    rospy.loginfo("Target and Origin Nodes are the same")
                    if action_server is None:
                        rospy.loginfo("Action not taken, outputing success")
                        result=True
                        inc=0
                    else:
                        rospy.loginfo("Getting to exact pose")
                        result, inc = self.monitored_navigation(Onode.waypoint, action_server)
                        rospy.loginfo("going to waypoint in node resulted in")
                        print result
                else:
                    rospy.loginfo("Target or Origin Nodes were not found on Map")
                    self.cancelled = True
                    result=False
                    inc=1
            tries+=inc


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
                    not_goal=True
                    break                    
                children=to_expand[exp_index]._get_Children()
                #print "nodes in list:"
                #print children
        
        if not_goal:
            route=None
        else:
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

                
    """
     Follow Route
     
     This function takes the follows the chosen route to reach the goal
    """
    def followRoute(self, route):
        nnodes=len(route)

        self.navigation_activated=True
        Orig = route[0].name
        Targ = route[nnodes-1].name
        self._target = Targ

        # move base original config
        config = self.rcnfclient.get_configuration()
        self.dyt = config['yaw_goal_tolerance']

        rospy.loginfo("%d Nodes on route" %nnodes)

        rindex=0
        nav_ok=True
        route_len = len(route)-2
        

        a = route[rindex]._get_action(route[rindex+1].name)

        # If the robot is not on a node or the first action is not move base type
        # navigate to closest node waypoint (only when first action is not move base)
        if self.current_node == 'none' and a not in self.move_base_actions :
            if a not in self.move_base_actions:
                self.next_action = a
                print 'Do move_base to %s' %self.closest_node#(route.source[0])
                inf = route[0].waypoint
                params = { 'yaw_goal_tolerance' : 0.087266 }   #5 degrees tolerance
                self.reconf_movebase(params)
                #self.rcnfclient.update_configuration(params)
                nav_ok, inc= self.monitored_navigation(inf,'move_base')
        else:
            if a not in self.move_base_actions :
                n_edges=len(route[0].edges)
                for i in range(0,n_edges):
                    action_server=route[0].edges[i]["action"]
                    #if  action_server == 'move_base' or  action_server == 'human_aware_navigation':
                    # Check if there is a move_base action in the edages of this node
                    # if not is dangerous to move
                    if  action_server in self.move_base_actions :
                        break
                    action_server=None
                                       
                if action_server is None:
                    rospy.loginfo("Action not taken, outputing success")
                    nav_ok = True
                    inc = 0
                else:
                    rospy.loginfo("Getting to exact pose")
                    nav_ok, inc = self.monitored_navigation(route[0].waypoint, action_server)
                    rospy.loginfo("going to waypoint in node resulted in")
                    print nav_ok
                

        while rindex < (len(route)-1) and not self.cancelled and nav_ok :
            #current action
            a = route[rindex]._get_action(route[rindex+1].name)
            #next action
            if rindex < route_len :
                a1 = route[rindex+1]._get_action(route[rindex+2].name)
            else :
                a1 = 'none'

            self.current_action = a
            self.next_action = a1

            rospy.loginfo("From %s do (%s) to %s" %(route[rindex].name, a, route[rindex+1].name))

            current_edge = '%s_%s--%s' %(route[rindex].name, route[rindex+1].name, self.topol_map)
            self.cur_edge.publish(current_edge)

            pubedg = CurrentEdge()
            pubedg.header.stamp = rospy.Time.now()
            pubedg.origin = route[rindex].name
            pubedg.target = route[rindex+1].name
            pubedg.action = a
            pubedg.active = True
            pubedg.result = True
            self.edge_pub.publish(pubedg)
            
            
            self._feedback.route = '%s to %s using %s' % (route[rindex].name, route[rindex+1].name, a)
            self._as.publish_feedback(self._feedback)

            self.stat=nav_stats(route[rindex].name, route[rindex+1].name, self.topol_map)
            dt_text=self.stat.get_start_time_str()

            # do not care for the orientation of the waypoint if is not the last waypoint AND
            # the current and following action are move_base or human_aware_navigation
            if rindex < route_len and a1 in self.move_base_actions and a in self.move_base_actions :
                params = { 'yaw_goal_tolerance' : 6.283 }   # No orientation restrictions
                self.reconf_movebase(params)
                #self.rcnfclient.update_configuration(params)


            inf = route[rindex+1].waypoint
            self.current_target = route[rindex+1].name
            nav_ok, inc = self.monitored_navigation(inf, a)
            params = { 'yaw_goal_tolerance' : 0.087266 }   #5 degrees tolerance
            self.reconf_movebase(params)
            #self.rcnfclient.update_configuration(params)
            rospy.loginfo('setting parameters back')
            
            
            
            not_fatal=nav_ok
            if self.cancelled :
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

            pubedg.header.stamp = rospy.Time.now()
            pubedg.active = False
            pubedg.result = nav_ok
            self.edge_pub.publish(pubedg)
            
            current_edge = 'none'
            self.cur_edge.publish(current_edge)            
            
            self.current_action = 'none'
            self.next_action = 'none'
            rindex=rindex+1


        params = { 'yaw_goal_tolerance' : self.dyt }   #setting original config back
        self.reconf_movebase(params)
        #self.rcnfclient.update_configuration(params)


        self.navigation_activated=False

        result=nav_ok
        return result, inc


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
        inc = 0
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


        self.goal_reached=False
        self.monNavClient.send_goal(goal)
        #self.monNavClient.wait_for_result()
        status=self.monNavClient.get_state()
        while (status == GoalStatus.ACTIVE or status == GoalStatus.PENDING) and not self.cancelled and not self.goal_reached :
            status=self.monNavClient.get_state()
            rospy.sleep(rospy.Duration.from_sec(0.01))
        #rospy.loginfo(str(status))
        #print status
        res=self.monNavClient.get_result()
#        print "--------------RESULT------------"
#        print res
#        print "--------------RESULT------------"
        if status != GoalStatus.SUCCEEDED :
            if not self.goal_reached:
                result = False
                if status is GoalStatus.PREEMPTED:
                    self.preempted = True
            else:
                result = True

        if not res:
            if not result:
                inc = 1
            else :
                inc = 0
        else:
            if res.recovered is True and res.human_interaction is False :
                inc = 1
            else :
                inc = 0

            
        rospy.sleep(rospy.Duration.from_sec(0.3))
        return result, inc

    def reconf_movebase(self, params):
        try:
            self.rcnfclient.update_configuration(params)
        except rospy.ServiceException as exc:
            rospy.logwarn("I couldn't reconfigure move_base parameters. Caught service exception: %s. will continue with previous params", exc)



if __name__ == '__main__':
    mode="normal"
    rospy.init_node('topological_navigation')
    server = TopologicalNavServer(rospy.get_name(),mode)
