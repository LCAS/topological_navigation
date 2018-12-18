#!/usr/bin/env python

import math
import rospy
import actionlib
#import pymongo
#import json
#import sys
import rosservice

import calendar
#from time import sleep
from datetime import datetime


from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

from strands_navigation_msgs.msg import MonitoredNavigationAction
from strands_navigation_msgs.msg import MonitoredNavigationGoal
from strands_navigation_msgs.msg import NavStatistics


#from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy
#from topological_navigation.topological_node import *
from topological_navigation.navigation_stats import *


import topological_navigation.msg
import strands_navigation_msgs.msg
import dynamic_reconfigure.client


"""
    get_node
    
    Given a topological map and a node name it returns the node object
"""
def get_node(top_map, node_name):
    for i in top_map.nodes:
        if i.name == node_name:
            return i
    return None


# a list of parameters top nav is allowed to change 
# and their mapping from dwa speak 
# if not listed then the param is not sent, 
# e.g. TrajectoryPlannerROS doesn't have tolerances
DYNPARAM_MAPPING = {
        'DWAPlannerROS': {
            'yaw_goal_tolerance': 'yaw_goal_tolerance',
            'xy_goal_tolerance': 'xy_goal_tolerance',
            'max_vel_x': 'max_vel_x',
            'max_trans_vel' : 'max_trans_vel',
        },

        'TrajectoryPlannerROS': {
            'max_vel_x': 'max_vel_x',
            'max_trans_vel' : 'max_vel_x',
        },
    }



"""
 Class for Policy Execution

"""

class PolicyExecutionServer(object):
    _feedback = strands_navigation_msgs.msg.ExecutePolicyModeFeedback()
    _result   = strands_navigation_msgs.msg.ExecutePolicyModeResult()


    """
     Initialization for Policy Execution Class
    
    """
    def __init__(self) :
        
        self.cancelled = False
        self.preempted = False
        self.nav_preempted = False
        #self.aborted = False
        self.current_action = 'none'
        self.current_route = None
        self.nfails = 0
        self.n_tries = rospy.get_param('~retries', 3)
        self.move_base_reconf_service = rospy.get_param('~move_base_reconf_service', 'DWAPlannerROS')
        
        rospy.on_shutdown(self._on_node_shutdown)
        self.move_base_actions = ['move_base']
        self.needed_actions=[]
        
        
        self.navigation_activated=False
        self._action_name = 'topological_navigation/execute_policy_mode'
        self.stats_pub = rospy.Publisher('topological_navigation/Statistics', NavStatistics, queue_size=1)


        self.lnodes = []
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)       
        rospy.loginfo("Waiting for Topological map ...")        
        while len(self.lnodes) == 0 :
            rospy.sleep(rospy.Duration.from_sec(0.05))
        rospy.loginfo(" ...done")
        self.needed_move_base_actions = [x for x in self.needed_actions if x in self.move_base_actions]
        if 'move_base' not in self.needed_move_base_actions:
            self.needed_move_base_actions.append('move_base')
        

        #Creating monitored navigation client
        rospy.loginfo("Creating monitored navigation client.")
        self.monNavClient= actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        self.monNavClient.wait_for_server()
        rospy.loginfo(" ...done")


        #Subscribing to Localisation Topics
        rospy.loginfo("Waiting for Localisation Topics")
        self.current_node = rospy.wait_for_message('current_node', String)
        self.closest_node = rospy.wait_for_message('closest_node', String)  

        rospy.loginfo("Subscribing to Localisation Topics")
        rospy.Subscriber('closest_node', String, self.closestNodeCallback)
        rospy.Subscriber('current_node', String, self.currentNodeCallback)
        rospy.loginfo(" ...done")

        mb_service_created=False
        self.rcnfclient = {}
        self.init_dynparams = {}

        config = {}
        

        self.move_base_planner = rospy.get_param('~move_base_planner', 'DWAPlannerROS')
            
        #Creating Reconfigure Client
        for i in self.needed_move_base_actions:
           service_created = self.create_reconfigure_client(i)

            if service_created and i == 'move_base':
                mb_service_created = True
                
        if not mb_service_created:
            while not mb_service_created and not self.cancelled:
                rospy.sleep(1)
                rospy.logwarn("%s must be created! will keep trying until its there" %rcnfsrvrname)
                mb_service_created = self.create_reconfigure_client('move_base')
        
        if not self.cancelled:
    
            #Creating Action Server
            rospy.loginfo("Creating action server.")
            self._as = actionlib.SimpleActionServer(self._action_name, strands_navigation_msgs.msg.ExecutePolicyModeAction, execute_cb = self.executeCallback, auto_start = False)
            self._as.register_preempt_callback(self.preemptCallback)
            rospy.loginfo(" ...starting")
            self._as.start()
            rospy.loginfo(" ...done")
    
    
            rospy.loginfo("EPM All Done ...")
            rospy.spin()


    def create_reconfigure_client(self, mb_action):
        """
        Creates the dynamic reconfigure clients for the given actions
        """
        rcnfsrvrname= rospy.get_namespace() + mb_action +'/' + self.move_base_planner
        test_service = rcnfsrvrname+'/set_parameters'
        
        service_created = False
        service_created_tries = 50
        while service_created_tries > 0 and not self.cancelled:              
            service_names = rosservice.get_service_list()
            if test_service in service_names:
                rospy.loginfo("Creating Reconfigure Client %s" %rcnfsrvrname)
                client = dynamic_reconfigure.client.Client(rcnfsrvrname, timeout=10)
                self.rcnfclient[mb_action] = client
                self.init_dynparams[mb_action] = client.get_configuration()
                return True
            else:
                service_created_tries -= 1
                if service_created_tries > 0:
                    rospy.logwarn("I couldn't create reconfigure client %s. remaining tries %d" % (rcnfsrvrname,service_created_tries))
                    rospy.sleep(1)
                else:
                    rospy.logerr("I couldn't create reconfigure client %s. is %s running?" % (rcnfsrvrname, i))
        return False


    def store_initial_parameters(self):
        for mb_action, client in self.rcnfclient.iteritems():
            try:
                self.init_dynparams[mb_action] = client.get_configuration()
            except Exception as e:
                rospy.logwarn("I couldn't store initial move_base parameters. Caught exception: %s. will continue with previous params", exc)


    def reset_reconfigure_params(self, mb_action):
        if mb_action in self.init_dynparams:
            self._do_movebase_reconf(mb_action, self.init_dynparams[mb_action])
        else:
            rospy.logwarn('No initial parameters stored for %s' % mb_action)




    def reconfigure_movebase_params(self, mb_action, params):
        translated_params = {}
        translation = DYNPARAM_MAPPING[self.move_base_planner]
        for k, v in params.iteritems():
            if k in translation:
                translated_params[translation[k]] = v
            else:
                rospy.logwarn('%s has no dynparam translation for %s' % (self.move_base_planner, k))
        self._do_movebase_reconf(mb_action, translated_params)

    """
    Reconfigure Move Base
     
    """
    def _do_movebase_reconf(self, mb_action, params):
        if mb_action in self.rcnfclient:
            try:
                self.rcnfclient[mb_action].update_configuration(params)
            except  dynamic_reconfigure.DynamicReconfigureCallbackException as exc:
                rospy.logwarn("I couldn't reconfigure %s parameters. Caught service exception: %s. will continue with previous params" % (mb_action, exc))
        else:
            rospy.logwarn("No dynamic reconfigure for %s" % mb_action)



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
        for mb_action in self.move_base_actions:
            self.reset_reconfigure_params(mb_action)

    """
     Closest Node CallBack
     
    """
    def closestNodeCallback(self, msg):
        self.closest_node=msg.data



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
                    # If the current action is a move_base type action and there is a new node in the route connected 
                    # by another move_base type action the goal will be set as reached so a new goal can be called in
                    # execute_policy
                    if self.current_action in self.move_base_actions and self.current_node in self.current_route.source : 
                        nod_ind = self.current_route.source.index(self.current_node)
                        next_action, target = self.find_action(self.current_route.source[nod_ind], self.current_route.edge_id[nod_ind])
                        if next_action in self.move_base_actions :
                            self.goal_reached=True
                        else:
                            if self.current_node != self.current_target :
                                #This means that the robot will try to execute the policies again which means navigating to the waypoint
                                self.goal_failed=True                            


    def get_edge(self, orig, dest, a):
        found = False
        edge = None
        for i in self.curr_tmap.nodes:
            if i.name == orig:
                for j in i.edges:
                    if j.node == dest and j.action == a :
                        found = True
                        edge = j
                        break
            if found:
                break
        
        return edge


    """
     Execute CallBack
     
     This Functions is called when the Action Server is called
    """
    def executeCallback(self, goal):
        self.cancelled = False
        self.preempted = False

        self.store_initial_parameters()

        result = self.followRoute(goal.route)
    
        if not self.cancelled :     
            self._result.success = result
            #self._feedback.route_status = self.current_node
            #self._as.publish_feedback(self._feedback)
            if result:
                self._as.set_succeeded(self._result)
            else :
                self._as.set_aborted(self._result)
        else:
            if self.preempted :
                self._result.success = False
                self._as.set_preempted(self._result)
            else : 
                self._result.success = False
                self._as.set_aborted(self._result)
        #self._feedback.route_status = 'Starting...'
        #self._as.publish_feedback(self._feedback)
        #rospy.loginfo('%s: Navigating From %s to %s', self._action_name, self.closest_node, goal.target)
        #self.navigate(goal.target)


    
    """
     Follow Route
     
    """
    def followRoute(self, route):

#        for i in range(0, len(route.source)):
#            action = self.find_action(route.source[i], route.target[i])
#            print '%s -(%s)-> %s' %(route.source[i], action, route.target[i])


        # If the robot is not on a node navigate to closest node
#        if self.current_node == 'none' :
#            rospy.loginfo('Do move_base to %s' %self.closest_node)#(route.source[0])
#            result=self.navigate_to('move_base',self.closest_node)



        #if self.current_node in route.source:
                
        # execute policies
        result=self.execute_policy(route)
        
        
        #result=True
        return result


    """
     Execute Policy
     
    """
    def execute_policy(self, route):
        keep_executing=True  # Flag Variable to remain in loop until all conditions are met
        success = True
        
        self.current_route = route
        self.navigation_activated=True
        no_reset=False
        
#        print "---------------------------------------------------------"
#        print "HERE WE GO AGAIN"        
#        print "---------------------------------------------------------"        
        while keep_executing :
#            print "#####################################################"
            rospy.loginfo("Navigating from %s: %d tries", self.current_node,self.nfails)
#            print "#####################################################"
            if self.current_node in route.source and not self.cancelled :
                rospy.loginfo("case A")
                # If there is an action associated to the current node and action server not preempted or aborted
                if success :
                    #rospy.loginfo("case A.1")
                    if no_reset:        #if previous action was just navigate to waypoint before trying no move_base action do not reset fail counter
                        no_reset=False
                    else:
                        self.nfails=0
                    nod_ind = route.source.index(self.current_node)
#                    self.current_action = self.find_action(route.source[nod_ind], route.target[nod_ind])
                    self.current_action, target = self.find_action(route.source[nod_ind], route.edge_id[nod_ind])
                    
                    if self.current_action != 'none':
                        # There is an edge between these two nodes
                        rospy.loginfo('%s -(%s)-> %s' %(route.source[nod_ind], self.current_action, target))
                        success=self.navigate_to(self.current_action, target)
                    else:
                        # There is NO edge between these two nodes so abort the execution
                        success = False
                        keep_executing = False
                        rospy.loginfo("There is NO edge %s will ABORT policy execution", route.edge_id[nod_ind])
                        #rospy.loginfo("There is NO edge between %s and %s will ABORT policy execution",route.source[nod_ind], route.target[nod_ind])
                else :
                    #print "case A.2"
                    if self.nfails < self.n_tries :
                        nod_ind = route.source.index(self.current_node)
#                        action = self.find_action(route.source[nod_ind], route.target[nod_ind])
                        action, target = self.find_action(route.source[nod_ind], route.edge_id[nod_ind])
                        if action != 'none':
                            self.current_action = action
                            rospy.loginfo('%s -(%s)-> %s' %(route.source[nod_ind], self.current_action, target))
                            success=self.navigate_to(self.current_action,target)
                        else:
                            success = False
                            keep_executing = False
                    else:
                        success = False
                        keep_executing = False

            else :
                #rospy.loginfo("case B")
                if self.cancelled:
                    print "case B.1"
                    # Execution was preempted or aborted
                    success = False
                    keep_executing = False
                    break
                else :
                    #print "case B.2"
                    # No action associated with current node
                    #print "%s not in:" %self.current_node 
                    #print route.source
                    if self.current_node == 'none' :
                        #print "case B.2.1"
                        # if current_node is none then is a failure
                        if self.nfails < self.n_tries :
                            if self.closest_node in route.source :
                                # Retry using policy from closest node
                                nod_ind = route.source.index(self.closest_node)
                                #action = self.find_action(route.source[nod_ind], route.target[nod_ind])
                                action, target = self.find_action(route.source[nod_ind], route.edge_id[nod_ind])
                                if action != 'none':
                                    if action in self.move_base_actions :
                                        rospy.loginfo("case B.2")
                                        # If closest_node and its target are connected by move_base type action nvigate to target
                                        self.current_action = action
                                        rospy.loginfo('%s -(%s)-> %s' %(route.source[nod_ind], self.current_action, target))
                                        success=self.navigate_to(self.current_action,target)
                                    else:
                                        rospy.loginfo("case B.3")
                                        # If closest_node and its target are not connected by move_base type action navigate to closest_node
                                        rospy.loginfo('Do move_base to %s' %self.closest_node)#(route.source[0])
                                        self.current_action = 'move_base'
                                        success=self.navigate_to(self.current_action,self.closest_node)
                                        #if previous action was just navigate to waypoint before trying no move_base action do not reset fail counter
                                        no_reset=True 
                                else:
                                    rospy.loginfo("case C")
                                    # No edge between Closest Node and its target Abort execution
                                    success = False
                                    keep_executing = False
                                    rospy.loginfo("There is NO edge %s will ABORT policy execution", route.edge_id[nod_ind])
                                    break
                            else :
                                rospy.loginfo("case D")
                                # Closest node not in route navigate to it (if it suceeds policy execution will be successful)
                                rospy.loginfo('Do move_base to %s' %self.closest_node)
                                self.current_action = 'move_base'
                                success=self.navigate_to(self.current_action,self.closest_node)
                        else:
                            #Maximun number of failures exceeded
                            success = False
                            keep_executing = False
                    else :
                        rospy.loginfo("case D.1")
                        #print "case B.2.2"
                        # Current node not in route so policy execution was successful
                        cl_node = get_node(self.curr_tmap, self.closest_node)
                        if no_reset:  #if previous action was just navigate to waypoint before trying no move_base action do not reset fail counter
                            no_reset=False
                        else:
                            self.nfails=0

                        if not cl_node.localise_by_topic:
                            if self.nfails < self.n_tries :
                                rospy.loginfo('Do move_base to %s' %self.current_node)
                                self.current_action = 'move_base'
                                success=self.navigate_to(self.current_action,self.current_node)
                                if success :
                                    keep_executing = False
                            else:
                                keep_executing = False
                        else:
                            rospy.loginfo('Policy was successful %s' %self.current_node)
                            #self.current_action = 'move_base'
                            success=True #self.navigate_to(self.current_action,self.current_node)
                            if success :
                                keep_executing = False

            rospy.sleep(rospy.Duration.from_sec(0.1))
        self.navigation_activated = False
        self.current_route = None
        self.nfails=0
        return success



    """
     Find Action
         
    """
#    def find_action(self, source, target):
    def find_action(self, source, edge_id):
        #print 'Searching for action between: %s -> %s' %(source, target)
        found = False
        action = 'none'
        target = 'none'
        for i in self.lnodes:
            if i.name == source :
                for j in i.edges:
                    if j.edge_id == edge_id:
                        action = j.action
                        target = j.node
                found = True
        if not found:
            self.publish_feedback(GoalStatus.ABORTED)
            rospy.logwarn("source node not found")
        return action,target




    """
     Navigate to
     
    """
    def navigate_to(self, action, node):
        self.current_target=node
        node_in_route = False
        found = False
        tolerance=0.0
        ytolerance= 0.0
        for i in self.lnodes:
            if i.name == node :
                found = True
                target_pose = i.pose#[0]
                tolerance=i.xy_goal_tolerance
                ytolerance= i.yaw_goal_tolerance
                break
        
        #temporary safety measures (Until all maps are updated)
        if tolerance == 0.0:
            tolerance = 0.48
        if ytolerance == 0.0:
            ytolerance = 0.087266
        
        if self.current_route != None :
            if node in self.current_route.source:
                routeind = self.current_route.source.index(node)
                next_action, next_node = self.find_action(node, self.current_route.edge_id[routeind])
                node_in_route = True
                #print "Next goal (%s) is the %d node in route" %(node,routeind)
                #print "Next Edge %s, Next Action %s" %(self.current_route.edge_id[routeind],next_action)
            else :
                next_action = 'none'
                node_in_route = False
                #print "Next goal NOT on route"
        else:
            next_action = 'none'
            node_in_route = False
            #print "no route"
        
        if found:
            self.current_action = action
            
            #self.stat=nav_stats(route[rindex].name, route[rindex+1].name, self.topol_map, edg)
            # Creating Navigation Object
            edg= self.get_edge(self.current_node, node, action)
            if edg is None:
                edge_id = 'none'
                top_vel = 0.55
            else:
                edge_id = edg.edge_id
                if edg.top_vel >= 0:
                    top_vel = edg.top_vel
                else:
                    top_vel = 0.55
            
            self.stat=nav_stats(self.current_node, node, self.topol_map, edge_id)
            #dt_text=self.stat.get_start_time_str()

            if action in self.move_base_actions and node_in_route :
                rospy.set_param("move_base/NavfnROS/default_tolerance",tolerance/math.sqrt(2))

            if next_action in self.move_base_actions :
                params = { 'yaw_goal_tolerance' : 6.28318531, 'max_vel_x':top_vel, 'max_trans_vel':top_vel}   #360 degrees tolerance
            else:
                if next_action == 'none':                                                #Next node is the final destination
                    params = { 'yaw_goal_tolerance' : ytolerance, 'max_vel_x':top_vel, 'max_trans_vel':top_vel} #Node predetermined tolerance
                else:                                                                    # Next action not move_base type
                    params = { 'yaw_goal_tolerance' : 0.523598776, 'max_vel_x':top_vel, 'max_trans_vel':top_vel}   #30 degrees tolerance

            if action in self.move_base_actions:
                self.reconfigure_movebase_params(action, params)
                
            (succeeded, status) = self.monitored_navigation(target_pose, action)

            if action in self.move_base_actions:
                self.reset_reconfigure_params(action)
            
            rospy.set_param("move_base/NavfnROS/default_tolerance",0.0)

            self.stat.set_ended(self.current_node)


            if succeeded :
                rospy.loginfo("navigation finished successfully")
                self.stat.status= "success"
                self.publish_feedback(GoalStatus.SUCCEEDED)
            else :
                if self.cancelled:
                    rospy.loginfo("Fatal fail")
                    self.stat.status= "fatal"
                    self.publish_feedback(GoalStatus.PREEMPTED)
                else:
                    rospy.loginfo("navigation failed")
                    self.stat.status= "failed"
                    self.nfails+=1
                    if self.nfails >= self.n_tries:
                        self.publish_feedback(GoalStatus.ABORTED)
            self.publish_stats()
                   #Publish Feedback
        else :
            # That node is not on the map
            succeeded = False
        return succeeded

    def publish_feedback(self, nav_outcome):
        if self.current_node == 'none': #Happens due to lag in fetch system
            rospy.sleep(0.5)
            if self.current_node == 'none':
                self._feedback.current_wp = self.closest_node
            else:
                self._feedback.current_wp = self.current_node
        else:
            self._feedback.current_wp = self.current_node
        self._feedback.status = nav_outcome
        self._as.publish_feedback(self._feedback)

    """
     Monitored Navigation
     
    """
    def monitored_navigation(self, pose, command):
        succeeded = True
        goal=MonitoredNavigationGoal()
        goal.action_server=command
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose = pose

        self.goal_reached=False
        self.goal_failed=False
        self.monNavClient.send_goal(goal)
        status=self.monNavClient.get_state()
        
        while (status == GoalStatus.ACTIVE or status == GoalStatus.PENDING) and not self.cancelled and not self.goal_reached and not self.goal_failed :
            status=self.monNavClient.get_state()
            rospy.sleep(rospy.Duration.from_sec(0.05))



        if status == GoalStatus.SUCCEEDED or self.goal_reached:
            succeeded = True
        else:
            succeeded = False #preempted nav in order to send a new goal, i.e., from the topo nav pov the navigation succeeded
            
        return (succeeded, status)



    """
     Publish Stats
     
    """
    def publish_stats(self):
        pubst = NavStatistics()
        pubst.edge_id = self.stat.edge_id
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

        msg_store = MessageStoreProxy(collection='nav_stats')
        msg_store.insert(pubst,meta)


    """
     Map CallBack
     
    """
    def MapCallback(self, msg) :
        self.topol_map = msg.name
        self.lnodes = msg.nodes
        self.curr_tmap = msg
        for i in self.lnodes:
            for j in i.edges:
                if j.action not in self.needed_actions:
                    self.needed_actions.append(j.action)

    """
     Node Shutdown CallBack
     
    """
    def _on_node_shutdown(self):
        self.cancelled = True
        for mb_action in self.move_base_actions:
            self.reset_reconfigure_params(mb_action)

if __name__ == '__main__':
    mode="normal"
    rospy.init_node('execute_policy_mode')
    server = PolicyExecutionServer()
