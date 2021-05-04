#!/usr/bin/env python

import rospy
import actionlib
import json

import topological_navigation.msg
import strands_navigation_msgs.msg

import dynamic_reconfigure.client

from strands_navigation_msgs.msg import NavStatistics
from strands_navigation_msgs.msg import CurrentEdge
from topological_navigation_msgs.msg import ClosestEdges

from std_msgs.msg import String
from geometry_msgs.msg import Pose

from actionlib_msgs.msg import *
from move_base_msgs.msg import *

from topological_navigation.navigation_stats import *
from topological_navigation.tmap_utils import *
from topological_navigation.route_search2 import *

from topological_navigation.edge_action_manager import EdgeActionManager
from topological_navigation.edge_reconfigure_manager import EdgeReconfigureManager

from copy import deepcopy
from threading import Lock

# A list of parameters topo nav is allowed to change and their mapping from dwa speak.
# If not listed then the param is not sent, e.g. TrajectoryPlannerROS doesn't have tolerances.
DYNPARAM_MAPPING = {
    "DWAPlannerROS": {
        "yaw_goal_tolerance": "yaw_goal_tolerance",
        "xy_goal_tolerance": "xy_goal_tolerance",
    },
    "TebLocalPlannerROS": {
        "yaw_goal_tolerance": "yaw_goal_tolerance",
        "xy_goal_tolerance": "xy_goal_tolerance",
    },
}


class TopologicalNavServer(object):
    
    _feedback = topological_navigation.msg.GotoNodeFeedback()
    _result = topological_navigation.msg.GotoNodeResult()

    _feedback_exec_policy = strands_navigation_msgs.msg.ExecutePolicyModeFeedback()
    _result_exec_policy = strands_navigation_msgs.msg.ExecutePolicyModeResult()

    def __init__(self, name, mode):
        
        self.node_by_node = False
        self.cancelled = False
        self.preempted = False
        self.stat = None
        self.no_orientation = False
        self._target = "None"
        self.current_action = "none"
        self.next_action = "none"
        self.n_tries = rospy.get_param("~retries", 3)

        self.current_node = "Unknown"
        self.closest_node = "Unknown"
        self.closest_edges = ClosestEdges()
        self.nfails = 0
        
        self.navigation_activated = False
        self.navigation_lock = Lock()

        move_base_actions = [
            "move_base",
            "human_aware_navigation",
            "han_adapt_speed",
            "han_vc_corridor",
            "han_vc_junction",
        ]

        self.needed_actions = []
        self.move_base_actions = rospy.get_param("~move_base_actions", move_base_actions)

        # what service are we using as move_base?
        self.move_base_name = rospy.get_param("~move_base_name", "move_base")
        if not self.move_base_name in self.move_base_actions:
            self.move_base_actions.append(self.move_base_name)
        
        self.stats_pub = rospy.Publisher("topological_navigation/Statistics", NavStatistics)
        self.edge_pub = rospy.Publisher("topological_navigation/Edge", CurrentEdge)
        self.route_pub = rospy.Publisher("topological_navigation/Route", strands_navigation_msgs.msg.TopologicalRoute)
        self.cur_edge = rospy.Publisher("current_edge", String)

        self._map_received = False
        rospy.Subscriber("/topological_map_2", String, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")

        while not self._map_received:
            rospy.sleep(rospy.Duration.from_sec(0.05))
        rospy.loginfo(" ...done")
        
        self.edge_action_manager = EdgeActionManager()

        # Creating Action Server for navigation
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(name, topological_navigation.msg.GotoNodeAction,
                                                execute_cb=self.executeCallback, auto_start=False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        # Creating Action Server for execute policy
        rospy.loginfo("Creating execute action server.")
        self._as_exec_policy = actionlib.SimpleActionServer("topological_navigation/execute_policy_mode", strands_navigation_msgs.msg.ExecutePolicyModeAction, 
                                                            execute_cb=self.executeCallbackexecpolicy, auto_start=False)
        self._as_exec_policy.register_preempt_callback(self.preemptCallbackexecpolicy)
        rospy.loginfo(" ...starting")
        self._as_exec_policy.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("EPM All Done ...")

        rospy.loginfo("Subscribing to Localisation Topics.")
        rospy.Subscriber("closest_node", String, self.closestNodeCallback)
        rospy.Subscriber("closest_edges", ClosestEdges, self.closestEdgesCallback)
        rospy.Subscriber("current_node", String, self.currentNodeCallback)
        rospy.loginfo(" ...done")

        self.edge_reconfigure = rospy.get_param("~reconfigure_edges", False)
        if self.edge_reconfigure:
            self.edgeReconfigureManager = EdgeReconfigureManager()

        rospy.loginfo("All Done ...")
        rospy.spin()
        

    def init_reconfigure(self):
        
        self.move_base_planner = rospy.get_param("~move_base_planner", "move_base/DWAPlannerROS")
        rospy.loginfo("Creating Reconfigure Client")
        self.rcnfclient = dynamic_reconfigure.client.Client(self.move_base_planner)
        self.init_dynparams = self.rcnfclient.get_configuration()
        

    def reconf_movebase(self, cedg, cnode, intermediate):

        if cnode["node"]["properties"]["xy_goal_tolerance"] <= 0.1:
            cxygtol = 0.1
        else:
            cxygtol = cnode["node"]["properties"]["xy_goal_tolerance"]
        if not intermediate:
            if cnode["node"]["properties"]["yaw_goal_tolerance"] <= 0.087266:
                cytol = 0.087266
            else:
                cytol = cnode["node"]["properties"]["yaw_goal_tolerance"]
        else:
            cytol = 6.283

        params = {"yaw_goal_tolerance": cytol, "xy_goal_tolerance": cxygtol}
        print "reconfiguring %s with %s" % (self.move_base_name, params)
        print "Intermediate: {}".format(intermediate)
        self.reconfigure_movebase_params(params)
        

    def reconfigure_movebase_params(self, params):

        self.init_dynparams = self.rcnfclient.get_configuration()
        
        key = self.move_base_planner[self.move_base_planner.rfind("/") + 1 :]
        translation = DYNPARAM_MAPPING[key]
        
        translated_params = {}
        for k, v in params.iteritems():
            if k in translation:
                if rospy.has_param(self.move_base_planner + "/" + translation[k]):
                    translated_params[translation[k]] = v
                else:
                    rospy.logwarn("%s has no parameter %s" % (self.move_base_planner, translation[k]))
            else:
                rospy.logwarn("%s has no dynparam translation for %s" % (self.move_base_planner, k))
                
        self._do_movebase_reconf(translated_params)
        

    def _do_movebase_reconf(self, params):
        
        try:
            self.rcnfclient.update_configuration(params)
        except rospy.ServiceException as exc:
            rospy.logwarn("I couldn't reconfigure move_base parameters. Caught service exception: %s. Will continue with previous params", exc)
            

    def reset_reconf(self):
        self._do_movebase_reconf(self.init_dynparams)


    def MapCallback(self, msg):
        """
         This Function updates the Topological Map everytime it is called
        """
        self.lnodes = json.loads(msg.data)
        self.topol_map = self.lnodes["pointset"]
        self.curr_tmap = deepcopy(self.lnodes)
        self.route_checker = RouteChecker(self.lnodes)

        for node in self.lnodes["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["action"] not in self.needed_actions:
                    self.needed_actions.append(edge["action"])
        
        self._map_received = True


    def executeCallback(self, goal):
        """
        This Functions is called when the topo nav Action Server is called
        """
        can_start = False

        with self.navigation_lock:
            if self.cancel_current_action(timeout_secs=10):
                # we successfully stopped the previous action, claim the title to activate navigation
                self.navigation_activated = True
                can_start = True

        if can_start:

            self.cancelled = False
            self.preempted = False
            self.no_orientation = goal.no_orientation
            print "NO ORIENTATION (%s)" % self.no_orientation
            
            self._feedback.route = "Starting..."
            self._as.publish_feedback(self._feedback)
            rospy.loginfo("Navigating From %s to %s", self.closest_node, goal.target)
            self.navigate(goal.target)

        else:
            rospy.logwarn("Could not cancel current navigation action, TOPONAV goal action aborted!")
            self._as.set_aborted()

        self.navigation_activated = False


    def executeCallbackexecpolicy(self, goal):
        """
        This Function is called when the execute policy Action Server is called
        """
        can_start = False

        with self.navigation_lock:
            if self.cancel_current_action(timeout_secs=10):
                # we successfully stopped the previous action, claim the title to activate navigation
                self.navigation_activated = True
                can_start = True

        if can_start:

            self.cancelled = False
            self.preempted = False
            
            route = goal.route
            valid_route = self.route_checker.check_route(route)
            
            if valid_route:
                target = route.source[-1]
                self._target = target
                route = self.enforce_navigable_route(route, target)
                result = self.execute_policy(route, target)
            else:
                result = False
                self.cancelled = True
                rospy.logwarn("Empty or invalid route in exec policy goal!")

            if not self.cancelled and not self.preempted:
                self._result_exec_policy.success = result
                self._as_exec_policy.set_succeeded(self._result_exec_policy)
            else:
                if not self.preempted:
                    self._result_exec_policy.success = result
                    self._as_exec_policy.set_aborted(self._result_exec_policy)
                else:
                    self._result_exec_policy.success = False
                    self._as_exec_policy.set_preempted(self._result_exec_policy)

        else: 
            rospy.logwarn(
                "Could not cancel current navigation action, EXEC_POLICY goal action aborted!")
            self._as_exec_policy.set_aborted()

        self.navigation_activated = False


    def preemptCallback(self):
        rospy.logwarn("Preempting toponav")
        self.preempted = True
        self.cancel_current_action(timeout_secs=2)


    def preemptCallbackexecpolicy(self):
        rospy.logwarn("Preempting toponav exec_policy")
        self.preempted = True
        self.cancel_current_action(timeout_secs=2)


    def closestNodeCallback(self, msg):
        self.closest_node = msg.data


    def closestEdgesCallback(self, msg):
        self.closest_edges = msg


    def currentNodeCallback(self, msg):
        
        if self.current_node != msg.data:  # is there any change on this topic?
            self.current_node = msg.data  # we are at this new node
            if msg.data != "none":  # are we at a node?
                rospy.loginfo("new node reached: {}".format(self.current_node))
                if self.navigation_activated:  # is the action server active?
                    if self.stat:
                        self.stat.set_at_node()
                    # if the robot reached and intermediate node and the next action is move base goal has been reached
                    if (
                        self.current_node == self.current_target
                        and self._target != self.current_target
                        and self.next_action in self.move_base_actions
                        and self.current_action in self.move_base_actions
                    ):
                        rospy.loginfo("intermediate node reached %s", self.current_node)
                        self.goal_reached = True


    def navigate(self, target):
        """
        This function takes the target node and plans the actions that are required
        to reach it.
        """
        tries = 0
        result = False
        route_found = False

        while tries <= self.n_tries and not result and not self.cancelled:
            o_node = get_node_from_tmap2(self.lnodes, self.closest_node)
            g_node = get_node_from_tmap2(self.lnodes, target)

            rospy.loginfo("Navigating Take : %d", tries)
            # Everything is Awesome!!!
            # Target and Origin are not None
            if g_node is not None and o_node is not None:
                rsearch = TopologicalRouteSearch2(self.lnodes)
                route = rsearch.search_route(o_node["node"]["name"], target)
                route = self.enforce_navigable_route(route, target)

                if route.source:
                    route_found = True
                    rospy.loginfo("Navigating Case 1")
                    self.publish_route(route, target)
                    result, inc = self.followRoute(route, target, 0)
                    rospy.loginfo("Navigating Case 1 -> res: %d", inc)
                else:
                    rospy.logerr("There is no route to this node check your edges ...")
                    rospy.loginfo("Navigating Case 1b")
                    result = False
                    inc = 1
                    rospy.loginfo("Navigating Case 1b -> res: %d", inc)
            else:
                rospy.loginfo("Navigating Case 2")
                rospy.loginfo("Target or Origin Nodes were not found on Map")
                self.cancelled = True
                result = False
                inc = 1
                rospy.loginfo("Navigating Case 3a -> res: %d", inc)
                
            tries += inc
            rospy.loginfo("Navigating next try: %d", tries)
        
        if not route_found:
            self.cancelled = True

        if (not self.cancelled) and (not self.preempted):
            self._result.success = result
            self._feedback.route = target
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        else:
            if not self.preempted:
                self._result.success = result
                self._feedback.route = self.current_node
                self._as.publish_feedback(self._feedback)
                self._as.set_aborted(self._result)
            else:
                self._result.success = False
                self._as.set_preempted(self._result)


    def execute_policy(self, route, target):
        
        succeeded, inc = self.followRoute(route, target, 1)

        if succeeded:
            rospy.loginfo("Navigation Finished Successfully")
            self.publish_feedback_exec_policy(GoalStatus.SUCCEEDED)
        else:
            if self.cancelled and self.preempted:
                rospy.loginfo("Fatal Fail")
                self.publish_feedback_exec_policy(GoalStatus.PREEMPTED)
            elif self.cancelled:
                rospy.loginfo("Navigation Failed")
                self.publish_feedback_exec_policy(GoalStatus.ABORTED)

        return succeeded
    

    def publish_feedback_exec_policy(self, nav_outcome=None):
        
        if self.current_node == "none":  # Happens due to lag in fetch system
            rospy.sleep(0.5)
            if self.current_node == "none":
                self._feedback_exec_policy.current_wp = self.closest_node
            else:
                self._feedback_exec_policy.current_wp = self.current_node
        else:
            self._feedback_exec_policy.current_wp = self.current_node
        if nav_outcome is not None:
            self._feedback_exec_policy.status = nav_outcome
        self._as_exec_policy.publish_feedback(self._feedback_exec_policy)


    def enforce_navigable_route(self, route, target_node):
        """
        Enforces the route to always contain the initial edge that leads the robot to the first node in the given route.
        In other words, avoid that the route contains an initial edge that is too far from the robot pose. 
        """
        rospy.loginfo("Current route {} ".format(route))
        if not(self.closest_edges.edge_ids[0] in route.edge_id or self.closest_edges.edge_ids[1] in route.edge_id):
            first_node = route.source[0] if len(route.source) > 0 else target_node
            for edge_id in self.closest_edges.edge_ids:
                if edge_id.endswith(first_node):
                    route.source.insert(0, edge_id.split("_")[0])
                    route.edge_id.insert(0, edge_id)
                    break
                
        rospy.loginfo("Modified route {}".format(route))
        return route


    def followRoute(self, route, target, exec_policy):
        """
        This function follows the chosen route to reach the goal.
        """
        
        nnodes = len(route.source)
        Orig = route.source[0]
        Targ = target
        self._target = Targ

        self.init_reconfigure()

        rospy.loginfo("%d Nodes on route" % nnodes)

        inc = 1
        rindex = 0
        nav_ok = True
        route_len = len(route.edge_id)

        o_node = get_node_from_tmap2(self.lnodes, Orig)
        edge_from_id = get_edge_from_id_tmap2(self.lnodes, route.source[0], route.edge_id[0])
        if edge_from_id:
            a = edge_from_id["action"]
            rospy.loginfo("first action %s" % a)
        else:
            rospy.logerr("Failed to get edge from id!! Invalid route!!")
            return False, inc

        # If the robot is not on a node or the first action is not move base type
        # navigate to closest node waypoint (only when first action is not move base)
        if self.current_node == "none" and a not in self.move_base_actions:
            if a not in self.move_base_actions:
                self.next_action = a
                print "Do %s to %s" % (self.move_base_name, self.closest_node)

                # 5 degrees tolerance
                params = {"yaw_goal_tolerance": 0.087266}
                self.reconfigure_movebase_params(params)

                self.current_target = Orig
                nav_ok, inc = self.execute_action(edge_from_id, o_node)
        else:
            if a not in self.move_base_actions:
                move_base_act = False
                for i in o_node.edges:
                    # Check if there is a move_base action in the edages of this node
                    # if not is dangerous to move
                    if i.action in self.move_base_actions:
                        move_base_act = True

                if not move_base_act:
                    rospy.loginfo("Action not taken, outputing success")
                    nav_ok = True
                    inc = 0
                else:
                    rospy.loginfo("Getting to exact pose")
                    self.current_target = Orig
                    nav_ok, inc = self.execute_action(edge_from_id, o_node)
                    rospy.loginfo("going to waypoint in node resulted in")
                    print nav_ok

        while rindex < (len(route.edge_id)) and not self.cancelled and nav_ok:
            
            cedg = get_edge_from_id_tmap2(self.lnodes, route.source[rindex], route.edge_id[rindex])
            a = cedg["action"]
            
            if rindex < (route_len - 1):
                nedge = get_edge_from_id_tmap2(self.lnodes, route.source[rindex + 1], route.edge_id[rindex + 1])
                a1 = nedge["action"]
            else:
                a1 = "none"

            self.current_action = a
            self.next_action = a1

            rospy.loginfo("From %s do (%s) to %s" % (route.source[rindex], a, cedg["node"]))

            current_edge = "%s--%s" % (cedg["edge_id"], self.topol_map)
            rospy.loginfo("Current edge: %s" % current_edge)
            self.cur_edge.publish(current_edge)

            if not exec_policy:
                self._feedback.route = "%s to %s using %s" % (route.source[rindex], cedg["node"], a)
                self._as.publish_feedback(self._feedback)
            else:
                self.publish_feedback_exec_policy()

            cnode = get_node_from_tmap2(self.lnodes, cedg["node"])

            # do not care for the orientation of the waypoint if is not the last waypoint AND
            # the current and following action are move_base or human_aware_navigation
            if rindex < route_len - 1 and a1 in self.move_base_actions and a in self.move_base_actions:
                self.reconf_movebase(cedg, cnode, True)
            else:
                if self.no_orientation:
                    self.reconf_movebase(cedg, cnode, True)
                else:
                    self.reconf_movebase(cedg, cnode, False)

            self.current_target = cedg["node"]

            self.stat = nav_stats(route.source[rindex], cedg["node"], self.topol_map, cedg["edge_id"])
            dt_text = self.stat.get_start_time_str()

            if self.edge_reconfigure:
                self.edgeReconfigureManager.register_edge(cedg)
                if self.edgeReconfigureManager.active:
                    self.edgeReconfigureManager.initialise()
                    self.edgeReconfigureManager.reconfigure()

            nav_ok, inc = self.execute_action(cedg, cnode)

            if self.edge_reconfigure and self.edgeReconfigureManager.active:
                self.edgeReconfigureManager._reset()
                rospy.sleep(rospy.Duration.from_sec(0.3))

            params = {"yaw_goal_tolerance": 0.087266, "xy_goal_tolerance": 0.1}
            self.reconfigure_movebase_params(params)

            not_fatal = nav_ok
            if self.cancelled:
                nav_ok = True
            if self.preempted:
                not_fatal = False
                nav_ok = False

            self.stat.set_ended(self.current_node)
            dt_text=self.stat.get_finish_time_str()
            operation_time = self.stat.operation_time
            time_to_wp = self.stat.time_to_wp

            if nav_ok:
                self.stat.status = "success"
                rospy.loginfo("Navigation Finished on %s (%d/%d)" % (dt_text, operation_time, time_to_wp))
            else:
                if not_fatal:
                    rospy.loginfo("Navigation Failed on %s (%d/%d)" % (dt_text, operation_time, time_to_wp))
                    self.stat.status = "failed"
                else:
                    rospy.loginfo("Fatal Fail on %s (%d/%d)" % (dt_text, operation_time, time_to_wp))
                    self.stat.status = "fatal"

            self.publish_stats()

            current_edge = "none"
            self.cur_edge.publish(current_edge)

            self.current_action = "none"
            self.next_action = "none"
            rindex = rindex + 1

        self.reset_reconf()

        result = nav_ok
        return result, inc


    def cancel_current_action(self, timeout_secs=-1):
        """
        Cancels the action currently in execution. Returns True if the current goal is correctly ended.
        """
        rospy.loginfo("Cancelling current navigation goal, timeout_secs={}...".format(timeout_secs))
        
        self.edge_action_manager.preempt()
        self.cancelled = True

        if timeout_secs > 0:
            stime = rospy.get_rostime()
            timeout = rospy.Duration().from_sec(timeout_secs)
            while self.navigation_activated:
                if (rospy.get_rostime() - stime) > timeout:
                    rospy.loginfo("\t[timeout called]")
                    break
                rospy.sleep(0.2)

        rospy.loginfo("DONE " + str(self.navigation_activated))
        return not self.navigation_activated


    def publish_route(self, route, target):
        
        stroute = strands_navigation_msgs.msg.TopologicalRoute()
        for i in route.source:
            stroute.nodes.append(i)
        stroute.nodes.append(target)
        self.route_pub.publish(stroute)
        

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
        pubst.date_at_node = self.stat.date_at_node.strftime("%A, %B %d %Y, at %H:%M:%S hours")
        
        pubst.date_finished = self.stat.get_finish_time_str()
        self.stats_pub.publish(pubst)
        self.stat = None
        

    def execute_action(self, edge, destination_node):
        
        inc = 0
        result = True
        self.goal_reached = False
        
        self.edge_action_manager.initialise(edge, destination_node)
        self.edge_action_manager.execute()
        
        status = self.edge_action_manager.client.get_state()
        while (
            (status == GoalStatus.ACTIVE or status == GoalStatus.PENDING)
            and not self.cancelled
            and not self.goal_reached
        ):
            status = self.edge_action_manager.client.get_state()
            rospy.sleep(rospy.Duration.from_sec(0.01))

        res = self.edge_action_manager.client.get_result()

        if status != GoalStatus.SUCCEEDED:
            if not self.goal_reached:
                result = False
                if status is GoalStatus.PREEMPTED:
                    self.preempted = True
            else:
                result = True

        if not res:
            if not result:
                inc = 1
            else:
                inc = 0

        rospy.sleep(rospy.Duration.from_sec(0.3))
        
        return result, inc
###################################################################################################################
        

###################################################################################################################
if __name__ == "__main__":
    rospy.init_node("topological_navigation")
    mode = "normal"
    server = TopologicalNavServer(rospy.get_name(), mode)

    rospy.loginfo("Exiting.")
###################################################################################################################
