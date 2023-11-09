#!/usr/bin/env python

import rospy, actionlib, json
import dynamic_reconfigure.client

import topological_navigation_msgs.msg
from topological_navigation_msgs.msg import NavStatistics
from topological_navigation_msgs.msg import CurrentEdge
from topological_navigation_msgs.msg import ClosestEdges
from topological_navigation_msgs.srv import EvaluateEdge, EvaluateEdgeRequest, EvaluateNode, EvaluateNodeRequest

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus

from topological_navigation.route_search2 import RouteChecker, TopologicalRouteSearch2, get_route_distance
from topological_navigation.navigation_stats import nav_stats
from topological_navigation.tmap_utils import *

from topological_navigation.edge_action_manager import EdgeActionManager
from topological_navigation.edge_reconfigure_manager import EdgeReconfigureManager

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
    
status_mapping = {}
status_mapping[0] = "PENDING"
status_mapping[1] = "ACTIVE"
status_mapping[2] = "PREEMPTED"
status_mapping[3] = "SUCCEEDED"
status_mapping[4] = "ABORTED"
status_mapping[5] = "REJECTED"
status_mapping[6] = "PREEMPTING"
status_mapping[7] = "RECALLING"
status_mapping[8] = "RECALLED"
status_mapping[9] = "LOST"
###################################################################################################################


###################################################################################################################
class TopologicalNavServer(object):
    """
WE WANT AN ADDITIONAL FIELD "enforce_tollerance" WHICH IF SET, WILL ENFORCE THE TOLLERANCES DEFINED REGARDLESS OF IF THE NODE IS INTERMEDIATE

if goal.success == True:
    if cedge['enforce_goal_tolerance'] and is_intermediate_node:
        return
    self.nextgoal()
    return

"""
    
    _feedback = topological_navigation_msgs.msg.GotoNodeFeedback()
    _result = topological_navigation_msgs.msg.GotoNodeResult()

    _feedback_exec_policy = topological_navigation_msgs.msg.ExecutePolicyModeFeedback()
    _result_exec_policy = topological_navigation_msgs.msg.ExecutePolicyModeResult()

    def __init__(self, name, mode):
        
        rospy.on_shutdown(self._on_node_shutdown)
        
        self.node_by_node = False
        self.cancelled = False
        self.preempted = False
        self.stat = None
        self.no_orientation = False
        self._target = "None"
        self.current_target = "none"
        self.current_action = "none"
        self.next_action = "none"
        self.nav_from_closest_edge = False
        self.fluid_navigation = True
        self.final_goal = False

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

        # Save the robot pose information for use with ackerman steeering alignment
        self.robot_odometry = Odometry()
        rospy.Subscriber("/odometry/filtered/global", Odometry, self.robot_odometry_cb)

        self.move_base_actions = rospy.get_param("~move_base_actions", move_base_actions)

        # what service are we using as move_base?
        self.move_base_name = rospy.get_param("~move_base_name", "move_base")
        if not self.move_base_name in self.move_base_actions:
            self.move_base_actions.append(self.move_base_name)
        
        self.stats_pub = rospy.Publisher("topological_navigation/Statistics", NavStatistics, queue_size=10)
        self.edge_pub = rospy.Publisher("topological_navigation/Edge", CurrentEdge, queue_size=10)
        self.route_pub = rospy.Publisher("topological_navigation/Route", topological_navigation_msgs.msg.TopologicalRoute, queue_size=10)
        self.cur_edge = rospy.Publisher("current_edge", String, queue_size=10)
        self.move_act_pub = rospy.Publisher("topological_navigation/move_action_status", String, latch=True, queue_size=1)

        self._map_received = False
        rospy.Subscriber("/topological_map_2", String, self.MapCallback)
        rospy.loginfo("Navigation waiting for the Topological Map...")

        while not self._map_received:
            rospy.sleep(rospy.Duration.from_sec(0.05))
        rospy.loginfo("Navigation received the Topological Map")
        
        self.edge_action_manager = EdgeActionManager()

        self.edge_reconfigure = rospy.get_param("~reconfigure_edges", True)
        self.srv_edge_reconfigure = rospy.get_param("~reconfigure_edges_srv", False)
        if self.edge_reconfigure:
            self.edgeReconfigureManager = EdgeReconfigureManager()
        else:
            rospy.logwarn("Edge Reconfigure Unavailable")

        rospy.loginfo("Subscribing to Localisation Topics...")
        #rospy.wait_for_message('closest_edges', ClosestEdges, timeout=10)
        rospy.Subscriber("closest_node", String, self.closestNodeCallback)
        rospy.Subscriber("closest_edges", ClosestEdges, self.closestEdgesCallback)
        rospy.Subscriber("current_node", String, self.currentNodeCallback)
        rospy.loginfo("...done")
        
        try:
            rospy.loginfo("Waiting for restrictions...")
            rospy.wait_for_service('restrictions_manager/evaluate_edge', timeout=3.0)
            
            self.evaluate_edge_srv = rospy.ServiceProxy(
                'restrictions_manager/evaluate_edge', EvaluateEdge)
            self.evaluate_node_srv = rospy.ServiceProxy(
                'restrictions_manager/evaluate_node', EvaluateNode)
            
            rospy.loginfo("Restrictions Available")
            self.using_restrictions = True
        except:
            rospy.logwarn("Restrictions Unavailable")
            self.using_restrictions = False

        # this keeps the runtime state of the fail policies that are currently in execution 
        self.executing_fail_policy = {}
        
        # Creating Action Server for navigation
        rospy.loginfo("Creating GO-TO-NODE action server...")
        self._as = actionlib.SimpleActionServer(name, topological_navigation_msgs.msg.GotoNodeAction,
                                                execute_cb=self.executeCallback, auto_start=False)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("...done")

        # Creating Action Server for execute policy
        rospy.loginfo("Creating EXECUTE POLICY MODE action server...")
        self._as_exec_policy = actionlib.SimpleActionServer("topological_navigation/execute_policy_mode", topological_navigation_msgs.msg.ExecutePolicyModeAction, 
                                                            execute_cb=self.executeCallbackexecpolicy, auto_start=False)
        self._as_exec_policy.register_preempt_callback(self.preemptCallbackexecpolicy)
        self._as_exec_policy.start()
        rospy.loginfo("...done")

        rospy.loginfo("All Done.")
        
        
    def _on_node_shutdown(self):
        with self.navigation_lock:
            if self.navigation_activated:
                self.preempted = True
                self.cancel_current_action(timeout_secs=2)


    def robot_odometry_cb(self, msg):
        self.robot_odometry = msg

    def init_reconfigure(self):
        
        self.move_base_planner = rospy.get_param("~move_base_planner", "move_base/DWAPlannerROS")
        planner = self.move_base_planner.split("/")[-1]
        if not planner in DYNPARAM_MAPPING:
            DYNPARAM_MAPPING[planner] = {}
        
        rospy.loginfo("Creating reconfigure client for {}".format(self.move_base_planner))
        self.rcnfclient = dynamic_reconfigure.client.Client(self.move_base_planner)
        self.init_dynparams = self.rcnfclient.get_configuration()
        

    def reconf_movebase(self, cedg, cnode, intermediate):

        # Ensure current xy goal tollerance is not below 0.1
        #cxygtol = cnode["node"]["properties"]["xy_goal_tolerance"]
        #if cxygtol <= 0.1: cxygtol = 0.1

        # Ensure current yaw goal tollerance is not below 0.87266
        #cytol = cnode["node"]["properties"]["yaw_goal_tolerance"]
        #if cytol <= 0.087266: cytol = 0.087266

        ## If node is not final node (is intermidiate) set yaw tollerance to 100%
        #if intermediate: cytol = 6.283


        if cnode["node"]["properties"]["xy_goal_tolerance"] <= 0.1:
            cxygtol = 0.1
        else:
            cxygtol = cnode["node"]["properties"]["xy_goal_tolerance"]
        
        if not intermediate:
            if cnode["node"]["properties"]["yaw_goal_tolerance"] <= 0.087266:
                cytol = 0.087266
            else:
                cytol = cnode["node"]["properties"]["yaw_goal_tolerance"]
        else: # is intermediate
            # any number larger than 3.2 gets clipped
            cytol = 3.19

        print("NODE NODE NODE, Checking if node is entry 220")
        if cnode["node"]["name"].endswith("-ca"):
            print("Setting cytol to 0.1 as this is an entry node, navigation.py line 221")
            cytol = 0.1

        params = {"yaw_goal_tolerance": cytol, "xy_goal_tolerance": cxygtol}
        rospy.loginfo("Reconfiguring %s with %s" % (self.move_base_planner, params))
        print("Intermediate: {}".format(intermediate))
        self.reconfigure_movebase_params(params)
        

    def reconfigure_movebase_params(self, params):

        self.init_dynparams = self.rcnfclient.get_configuration()
        
        key = self.move_base_planner[self.move_base_planner.rfind("/") + 1 :]
        translation = DYNPARAM_MAPPING[key]
        
        translated_params = {}
        for k, v in params.items():
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
            rospy.logwarn("Could not reconfigure move_base parameters. Caught service exception: %s. Will continue with previous parameters", exc)
            

    def reset_reconf(self):
        self._do_movebase_reconf(self.init_dynparams)


    def MapCallback(self, msg):
        """
         This Function updates the Topological Map everytime it is called
        """
        self.lnodes = json.loads(msg.data)
        self.topol_map = self.lnodes["pointset"]
        self.rsearch = TopologicalRouteSearch2(self.lnodes)
        self.route_checker = RouteChecker(self.lnodes)
        self.make_move_base_edge()

        self._map_received = True


    def make_move_base_edge(self):

        self.move_base_edge = {}
        self.move_base_edge["action"] = self.move_base_name
        self.move_base_edge["edge_id"] = "move_base_edge"

        move_base_goal = rospy.get_param("~move_base_goal", {})

        if not move_base_goal:
            for node in self.lnodes["nodes"]:
                for edge in node["node"]["edges"]:
                    if edge["action"] == self.move_base_name:
                        move_base_goal["action_type"] = edge["action_type"]
                        move_base_goal["goal"] = edge["goal"]
                        break
                else:
                    continue
                break

        if not move_base_goal:
            move_base_goal["action_type"] = "move_base_msgs/MoveBaseGoal"
            move_base_goal["goal"] = {}
            move_base_goal["goal"]["target_pose"] = {}
            move_base_goal["goal"]["target_pose"]["pose"] = "$node.pose"
            move_base_goal["goal"]["target_pose"]["header"] = {}
            move_base_goal["goal"]["target_pose"]["header"]["frame_id"] = "$node.parent_frame"

        self.move_base_edge["action_type"] = move_base_goal["action_type"]
        self.move_base_edge["goal"] = move_base_goal["goal"]

        rospy.loginfo("Move Base Goal set to {}".format(move_base_goal["action_type"]))


    def executeCallback(self, goal):
        """
        This Functions is called when the topo nav Action Server is called
        """
        print("\n####################################################################################################")
        rospy.loginfo("Processing GO-TO-NODE goal (No Orientation = {})".format(goal.no_orientation))
        can_start = False

        with self.navigation_lock:
            if self.cancel_current_action(timeout_secs=10):
                # we successfully stopped the previous action, claim the title to activate navigation
                self.navigation_activated = True
                can_start = True

        if can_start:

            self.cancelled = False
            self.preempted = False
            self.final_goal = False
            self.no_orientation = goal.no_orientation
            self.executing_fail_policy = {}
            
            self._feedback.route = "Starting..."
            self._as.publish_feedback(self._feedback)
            self.navigate(goal.target)

        else:
            rospy.logwarn("Could not cancel current navigation action, GO-TO-NODE goal aborted")
            self._as.set_aborted()

        self.navigation_activated = False


    def executeCallbackexecpolicy(self, goal):
        """
        This Function is called when the execute policy Action Server is called
        """
        print("\n####################################################################################################")
        rospy.loginfo("Processing EXECUTE POLICY MODE goal")
        can_start = False

        with self.navigation_lock:
            if self.cancel_current_action(timeout_secs=10):
                # we successfully stopped the previous action, claim the title to activate navigation
                self.navigation_activated = True
                can_start = True

        if can_start:

            self.cancelled = False
            self.preempted = False
            self.final_goal = False
            
            self.max_dist_to_closest_edge = rospy.get_param("~max_dist_to_closest_edge", 1.0)
            
            if self.closest_edges.distances[0] > self.max_dist_to_closest_edge or self.current_node != "none":
                self.nav_from_closest_edge = False
            else:
                self.nav_from_closest_edge = True
            
            route = goal.route
            valid_route = self.route_checker.check_route(route)
            
            if valid_route and (route.source[0] == self.current_node or route.source[0] == self.closest_node):
                final_edge = get_edge_from_id_tmap2(self.lnodes, route.source[-1], route.edge_id[-1])
                target = final_edge["node"]
                route = self.enforce_navigable_route(route, target)
                result = self.execute_policy(route, target)
            else:
                result = False
                self.cancelled = True
                rospy.logerr("Invalid route in execute policy mode goal")

            if not self.cancelled and not self.preempted:
                self._result_exec_policy.success = result
                if result:
                    print("navigation.py line 385")
                    self._as_exec_policy.set_succeeded(self._result_exec_policy)
                else:
                    print("navigation.py line 388")
                    self._as_exec_policy.set_aborted(self._result_exec_policy)
            else:
                if not self.preempted:
                    print("navigation.py line 392")
                    self._result_exec_policy.success = result
                    self._as_exec_policy.set_aborted(self._result_exec_policy)
                else:
                    print("navigation.py line 396")
                    self._result_exec_policy.success = False
                    self._as_exec_policy.set_preempted(self._result_exec_policy)

        else: 
            rospy.logwarn("Could not cancel current navigation action, EXECUTE POLICY MODE goal aborted.")
            self._as_exec_policy.set_aborted()

        self.navigation_activated = False


    def preemptCallback(self):
        rospy.logwarn("Preempting GO-TO-NODE goal")
        self.preempted = True
        self.cancel_current_action(timeout_secs=2)


    def preemptCallbackexecpolicy(self):
        rospy.logwarn("Preempting EXECUTE POLICY MODE goal")
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
                rospy.loginfo("New node reached: {}".format(self.current_node))
                if self.navigation_activated:  # is the action server active?
                    if self.stat:
                        self.stat.set_at_node()
                    # if the robot reached and intermediate node and the next action is move base goal has been reached
                    if (
                        self.current_node == self.current_target
                        and self._target != self.current_target
                        and self.next_action in self.move_base_actions
                        and self.current_action in self.move_base_actions
                        and self.fluid_navigation
                    ):
                        rospy.loginfo("Intermediate node reached: %s", self.current_node)
                        self.goal_reached = True


    def navigate(self, target):
        """
        This function takes the target node and plans the actions that are required
        to reach it.
        """
        result = False
        if not self.cancelled:

            g_node = self.rsearch.get_node_from_tmap2(target)
            
            self.max_dist_to_closest_edge = rospy.get_param("~max_dist_to_closest_edge", 1.0)
            
            if self.closest_edges.distances and (self.closest_edges.distances[0] > self.max_dist_to_closest_edge or self.current_node != "none"):
                self.nav_from_closest_edge = False
                o_node = self.rsearch.get_node_from_tmap2(self.closest_node)
                rospy.loginfo("Planning from the closest NODE: {}".format(self.closest_node))
            else:
                self.nav_from_closest_edge = True
                o_node, the_edge = self.orig_node_from_closest_edge(g_node)
                rospy.loginfo("Planning from the closest EDGE: {}".format(the_edge["edge_id"]))
                
            rospy.loginfo("Navigating From Origin %s to Target %s", o_node["node"]["name"], target)
             
            # Everything is Awesome!!!
            # Target and Origin are not None
            if (g_node is not None) and (o_node is not None):
                if g_node["node"]["name"] != o_node["node"]["name"]:
                    route = self.rsearch.search_route(o_node["node"]["name"], target)
                    route = self.enforce_navigable_route(route, target)
                    if route.source:
                        rospy.loginfo("Navigating Case 1: Following route")
                        self.publish_route(route, target)
                        result, inc = self.followRoute(route, target, 0)
                        rospy.loginfo("Navigating Case 1 -> res: %d", inc)
                    else:
                        rospy.logwarn("Navigating Case 1a: There is no route from {} to {}. Check your edges.".format(o_node["node"]["name"], target))
                        self.cancelled = True
                        result = False
                        inc = 1
                        rospy.loginfo("Navigating Case 1a -> res: %d", inc)
                else:      
                    if self.nav_from_closest_edge:
                        result, inc = self.to_goal_node(g_node, the_edge)
                    else:
                        result, inc = self.to_goal_node(g_node)
            else:
                rospy.logwarn("Navigating Case 3: Target or Origin Nodes were not found on Map")
                self.cancelled = True
                result = False
                inc = 1
                rospy.loginfo("Navigating Case 3 -> res: %d", inc)
        
        if (not self.cancelled) and (not self.preempted):
            self._result.success = result
            if result:
                self._feedback.route = target
                self._as.publish_feedback(self._feedback)
                self._as.set_succeeded(self._result)
            else:
                self._feedback.route = self.current_node
                self._as.publish_feedback(self._feedback)
                self._as.set_aborted(self._result)
        else:
            if not self.preempted:
                self._feedback.route = self.current_node
                self._as.publish_feedback(self._feedback)
                self._result.success = result
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
                rospy.logwarn("Fatal Fail")
                self.publish_feedback_exec_policy(GoalStatus.PREEMPTED)
            elif self.cancelled:
                rospy.logwarn("Navigation Failed")
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
        
        
    def orig_node_from_closest_edge(self, g_node):
        
        name_1, _ = get_node_names_from_edge_id_2(self.lnodes, self.closest_edges.edge_ids[0])
        name_2, _ = get_node_names_from_edge_id_2(self.lnodes, self.closest_edges.edge_ids[1])
        
        # Navigate from the closest edge instead of the closest node. First get the closest edges.
        edge_1 = get_edge_from_id_tmap2(self.lnodes, name_1, self.closest_edges.edge_ids[0])
        edge_2 = get_edge_from_id_tmap2(self.lnodes, name_2, self.closest_edges.edge_ids[1])

        # Then get their destination nodes.
        o_node_1 = self.rsearch.get_node_from_tmap2(edge_1["node"])
        o_node_2 = self.rsearch.get_node_from_tmap2(edge_2["node"])

        # If the closest edges are of equal distance (usually a bidirectional edge) 
        # then use the destination node that results in a shorter route to the goal.
        if self.closest_edges.distances[0] == self.closest_edges.distances[1]:
            d1 = get_route_distance(self.lnodes, o_node_1, g_node)
            d2 = get_route_distance(self.lnodes, o_node_2, g_node)
        else: # Use the destination node of the closest edge.
            d1 = 0; d2 = 1
        
        if d1 <= d2:
            return o_node_1, edge_1
        else:
            return o_node_2, edge_2
        
        
    def to_goal_node(self, g_node, the_edge=None):
        
        rospy.loginfo("Target and Origin Nodes are the same")
        self.current_target = g_node["node"]["name"]
        
        if the_edge is None:
            # Check if there is a move_base action in the edges of this node and choose the earliest one in the 
            # list of move_base actions. If not is dangerous to move.
            act_ind = 100
            for i in g_node["node"]["edges"]:
                c_action_server = i["action"]
                if c_action_server in self.move_base_actions:
                    c_ind = self.move_base_actions.index(c_action_server)
                    if c_ind < act_ind:
                        act_ind = c_ind
                        the_edge = i

        if the_edge is None:
            rospy.logwarn("Navigating Case 2a: Could not find a move base action in the edges of target {}. Unsafe to move".format(g_node["node"]["name"]))
            rospy.loginfo("Action not taken, outputting success")
            result=True
            inc=0
            rospy.loginfo("Navigating Case 2a -> res: %d", inc)
        else:
            rospy.loginfo("Navigating Case 2: Getting to the exact pose of target {}".format(g_node["node"]["name"]))
            self.final_goal = True
            self.current_target = g_node["node"]["name"]
            origin_name,_ = get_node_names_from_edge_id_2(self.lnodes, the_edge["edge_id"])
            origin_node = self.rsearch.get_node_from_tmap2(origin_name)

            self.edge_reconf_start(the_edge)
            print("line 595")
            result, inc = self.execute_action(the_edge, g_node, origin_node)
            self.edge_reconf_end()

            if not result:
                rospy.logwarn("Navigation Failed")
                inc=1
            else:
                rospy.loginfo("Navigation Finished Successfully")
            rospy.loginfo("Navigating Case 2 -> res: %d", inc)
            
        return result, inc


    def enforce_navigable_route(self, route, target_node):
        """
        Enforces the route to always contain the initial edge that leads the robot to the first node in the given route.
        In other words, avoid that the route contains an initial edge that is too far from the robot pose. 
        """
        if self.nav_from_closest_edge and self.closest_edges.edge_ids and len(self.closest_edges.edge_ids) == 2:
            if not(self.closest_edges.edge_ids[0] in route.edge_id or self.closest_edges.edge_ids[1] in route.edge_id):
                first_node = route.source[0] if len(route.source) > 0 else target_node
                
                for edge_id in self.closest_edges.edge_ids:
                    origin, destination = get_node_names_from_edge_id_2(self.lnodes, edge_id)
                    
                    if destination == first_node and edge_id not in route.edge_id:
                        route.source.insert(0, origin)
                        route.edge_id.insert(0, edge_id)
                        break
        return route


    def followRoute(self, route, target, exec_policy):
        """
        This function follows the chosen route to reach the goal.
        """
        self.navigation_activated = True
        
        nnodes = len(route.source)
        Orig = route.source[0]
        Targ = target
        self._target = Targ

        self.init_reconfigure()

        rospy.loginfo("%d Nodes on route" % nnodes)

        inc = 1
        rindex = 0
        nav_ok = True
        recovering = False
        replanned = False
        self.fluid_navigation = True

        o_node = self.rsearch.get_node_from_tmap2(Orig)
        edge_from_id = get_edge_from_id_tmap2(self.lnodes, route.source[0], route.edge_id[0])
        if edge_from_id:
            a = edge_from_id["action"]
            rospy.loginfo("First action: %s" % a)
        else:
            rospy.logerr("Failed to get edge from id {}. Invalid route".format(route.edge_id[0]))
            return False, inc
        
        if not self.nav_from_closest_edge:        
            # If the robot is not on a node or the first action is not move base type
            # navigate to closest node waypoint (only when first action is not move base)
            if a not in self.move_base_actions:
                rospy.loginfo("The action of the first edge in the route is not a move base action")
                rospy.loginfo("Current node is {}".format(self.current_node))
                
            if self.current_node == "none" and a not in self.move_base_actions:
                self.next_action = a
                rospy.loginfo("Do %s to origin %s" % (self.move_base_name, o_node["node"]["name"]))
    
                # 5 degrees tolerance
                params = {"yaw_goal_tolerance": 0.087266}
                self.reconfigure_movebase_params(params)

                self.current_target = Orig
                print("line 675")
                nav_ok, inc = self.execute_action(self.move_base_edge, o_node)
                rospy.loginfo("Navigation Finished Successfully") if nav_ok else rospy.logwarn("Navigation Failed")
                
            elif a not in self.move_base_actions:
                move_base_act = False
                for i in o_node["node"]["edges"]:
                    # Check if there is a move_base action in the edages of this node
                    # if not is dangerous to move
                    if i["action"] in self.move_base_actions:
                        move_base_act = True

                if not move_base_act:
                    rospy.logwarn("Could not find a move base action in the edges of origin {}. Unsafe to move".format(o_node["node"]["name"]))
                    rospy.loginfo("Action not taken, outputing success")
                    nav_ok = True
                    inc = 0
                else:
                    rospy.loginfo("Getting to the exact pose of origin {}".format(o_node["node"]["name"]))
                    self.current_target = Orig
                    print("line 695")
                    nav_ok, inc = self.execute_action(self.move_base_edge, o_node)
                    rospy.loginfo("Navigation Finished Successfully") if nav_ok else rospy.logwarn("Navigation Failed")
                

        while rindex < (len(route.edge_id)) and not self.cancelled and (nav_ok or recovering):
            
            cedg = get_edge_from_id_tmap2(self.lnodes, route.source[rindex], route.edge_id[rindex])
            a = cedg["action"]
            
            if rindex < (len(route.edge_id) - 1):
                nedge = get_edge_from_id_tmap2(self.lnodes, route.source[rindex + 1], route.edge_id[rindex + 1])
                a1 = nedge["action"]
                self.fluid_navigation = nedge["fluid_navigation"]
            else:
                nedge = None
                a1 = "none"
                self.fluid_navigation = False
                self.final_goal = True

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

            cnode = self.rsearch.get_node_from_tmap2(cedg["node"])
            onode = self.rsearch.get_node_from_tmap2(route.source[rindex])

            # do not care for the orientation of the waypoint if is not the last waypoint AND
            # the current and following action are move_base or human_aware_navigation
            # and when the fuild_navigation is true
            if rindex < len(route.edge_id) - 1 and a1 in self.move_base_actions and a in self.move_base_actions and self.fluid_navigation:
                self.reconf_movebase(cedg, cnode, True)
            else:
                if self.no_orientation:
                    self.reconf_movebase(cedg, cnode, True)
                else:
                    self.reconf_movebase(cedg, cnode, False)

            self.current_target = cedg["node"]

            self.stat = nav_stats(route.source[rindex], cedg["node"], self.topol_map, cedg["edge_id"])
            dt_text = self.stat.get_start_time_str()

            self.edge_reconf_start(cedg)
            if exec_policy:
                print("line 752")
                is_last_edge_in_route = rindex >= len(route.edge_id) - 1
                if is_last_edge_in_route:
                    print('NODE IS LAST IN ROUTE')
                else:
                    print('NODE IS NOT NOT NOT NOT LAST IN ROUTE')
                nav_ok, inc = self.execute_action(cedg, cnode, onode, is_last_edge_in_route)
            else:                
                nav_ok, inc, recovering, route, replanned = self.execute_action_fail_recovery(cedg, cnode, route, rindex, onode, target)
            self.edge_reconf_end()

            # FIXME: Hardcoding?
            params = {"yaw_goal_tolerance": 0.087266, "xy_goal_tolerance": 0.1}
            self.reconfigure_movebase_params(params)
            print(str(params) + "navigation.py line 784")

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
                    rospy.logwarn("Navigation Failed on %s (%d/%d)" % (dt_text, operation_time, time_to_wp))
                    self.stat.status = "failed"
                else:
                    rospy.logwarn("Fatal Fail on %s (%d/%d)" % (dt_text, operation_time, time_to_wp))
                    self.stat.status = "fatal"
            print('\n\n\n')

            self.publish_stats()

            current_edge = "none"
            self.cur_edge.publish(current_edge)

            self.current_action = "none"
            self.next_action = "none"
            
            if not replanned:
                rindex = rindex + 1

        self.reset_reconf()

        self.navigation_activated = False

        result = nav_ok
        return result, inc


    def edge_reconf_start(self, edge):

        if self.edge_reconfigure:
            if not self.srv_edge_reconfigure:
                self.edgeReconfigureManager.register_edge(edge)
                self.edgeReconfigureManager.initialise()
                self.edgeReconfigureManager.reconfigure()
            else:
                self.edgeReconfigureManager.srv_reconfigure(edge["edge_id"])


    def edge_reconf_end(self):

        if self.edge_reconfigure and not self.srv_edge_reconfigure and self.edgeReconfigureManager.active:
            self.edgeReconfigureManager._reset()
            rospy.sleep(rospy.Duration.from_sec(0.3))


    def cancel_current_action(self, timeout_secs=-1):
        """
        Cancels the action currently in execution. Returns True if the current goal is correctly ended.
        """
        rospy.loginfo("Cancelling current navigation goal, timeout_secs = {}...".format(timeout_secs))
        
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

        rospy.loginfo("Navigation active: " + str(self.navigation_activated))
        return not self.navigation_activated


    def publish_route(self, route, target):
        stroute = topological_navigation_msgs.msg.TopologicalRoute()
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
        

    def get_fail_policy_state(self, edge):
        policy = None
        state = -1
        if len(self.executing_fail_policy) == 0:
            _policy = [action.strip().split("_") for action in edge["fail_policy"].split(",")]
            policy = []
            # repeat the actions that can be repeated more than once
            for action in _policy:
                if len(action) > 1 and isinstance(eval(action[-1]), int):
                    for _ in range(eval(action[-1])):
                        policy.append(action[:-1])
                else:
                    policy.append(action)
            state = 0
            self.executing_fail_policy = {
                "policy": policy,   # the policy we want to execute
                "state": state,     # at which point of the policy we are
                "edge": edge["edge_id"]
            }
            
        else:
            policy = self.executing_fail_policy["policy"]
            # increment the state because if we are here it means that the previous policy action failed
            self.executing_fail_policy["state"] += 1
            state = self.executing_fail_policy["state"]

        return policy, state


    def execute_action_fail_recovery(self, edge, destination_node, route, idx, origin_node, target):
        """
        This function wraps `execute_action` by executing the fail_policy in case of ABORTED action
        The fail policy sometimes modifies the current route by including the recovery action
        """
        print("line 904")

        is_last_edge_in_route = idx >= len(route.edge_id) - 1
        if is_last_edge_in_route:
            print('NODE IS LAST IN ROUTE')
            print('we wouldnt need this is yaw tollerance "actually" worked')
        else:
            print('NODE IS NOT NOT NOT NOT LAST IN ROUTE')

        nav_ok, inc = self.execute_action(edge, destination_node, origin_node, is_last_edge_in_route)

        new_route = route
        recovering = False
        replanned = False

        # this means the action is aborted -> execute the fail policy
        # make sure that if the goal is cancelled by the client we don't enter here
        if not nav_ok and not self.preempted:
            rospy.loginfo("\t>> route: {}".format(route))
            route_updated = False
            while not route_updated:
                policy, state = self.get_fail_policy_state(edge)
                if state < len(policy):
                    rec_action = policy[state]
                    rospy.loginfo(">> EXECUTING FAIL POLICY ACTION: {}.".format(rec_action))

                    if rec_action[0] == "retry":
                        new_route.source.insert(idx+1, route.source[idx]) 
                        new_route.edge_id.insert(idx+1, route.edge_id[idx])
                        route_updated = True
                        recovering = True
                        replanned = False
                    elif rec_action[0] == "fail":
                        route_updated = True
                        recovering = False
                        replanned = False
                    elif rec_action[0] == "wait":
                        secs = 1
                        if len(rec_action) > 1:
                            secs = int(rec_action[-1])
                        rospy.sleep(secs)
                        recovering = True
                        replanned = False
                    elif rec_action[0] == "replan":
                        _route = self.rsearch.search_route(origin_node["node"]["name"], target, avoid_edges=[edge["edge_id"]])
                        _route = self.enforce_navigable_route(_route, target)

                        # build the new route
                        new_route.source = route.source[:idx] + _route.source
                        new_route.edge_id = route.edge_id[:idx] + _route.edge_id
                        route_updated = True
                        recovering = True
                        replanned = True
                else:
                    # clean the current fail policy data
                    self.executing_fail_policy = {}
                    recovering = False
                    replanned = False

            rospy.loginfo("\t>> new route: {}".format(new_route))
        return nav_ok, inc, recovering, new_route, replanned
    

    def execute_action(self, edge, destination_node, origin_node=None, is_last_edge_in_route=False):
        inc = 0
        result = True
        self.goal_reached = False
        self.prev_status = None

        if self.using_restrictions and edge["edge_id"] != "move_base_edge":
            rospy.loginfo("Evaluating restrictions on edge {}".format(edge["edge_id"]))
            ev_edge_msg = EvaluateEdgeRequest()
            ev_edge_msg.edge = edge["edge_id"]
            ev_edge_msg.runtime = True
            resp = self.evaluate_edge_srv.call(ev_edge_msg)
            if resp.success and resp.evaluation:
                rospy.logwarn("The edge is restricted, stopping navigation")
                result = False
                inc = 1
                return result, inc

            rospy.loginfo("Evaluating restrictions on node {}".format(destination_node["node"]["name"]))
            ev_node_msg = EvaluateNodeRequest()
            ev_node_msg.node = destination_node["node"]["name"]
            ev_node_msg.runtime = True
            resp = self.evaluate_node_srv.call(ev_node_msg)
            if resp.success and resp.evaluation:
                rospy.logwarn("The node is restricted, stopping navigation")
                result = False
                inc = 1
                return result, inc

        # Default edge['enforce_goal_tolerance'] to False
        edge['enforce_goal_tolerance'] = False if 'enforce_goal_tolerance' not in edge else edge['enforce_goal_tolerance']
        if destination_node['node']['name'] == 'WayPoint141': edge['enforce_goal_tolerance'] = True
        if '-ca' in destination_node['node']['name'] and '-c' not in origin_node['node']['name']: edge['enforce_goal_tolerance'] = True


        self.edge_action_manager.initialise(edge, destination_node, origin_node, is_last_edge_in_route=is_last_edge_in_route, robot_odometry=self.robot_odometry)
        self.edge_action_manager.execute()

        status = self.edge_action_manager.client.get_state()
        self.pub_status(status)
        while (
            (status == GoalStatus.ACTIVE or status == GoalStatus.PENDING) # move_base action server live feedback
            and not self.cancelled    # tripped to True by (no route, or nodes not in map)
            and not (self.goal_reached and edge['enforce_goal_tolerance'] == False) # tripped to True by /current_node
        ):
            # we want on condition of edge['enforce_goal_tolerance'], we dont want self.goal_reached to be a valid trip

            status = self.edge_action_manager.client.get_state()
            self.pub_status(status)
            rospy.sleep(rospy.Duration.from_sec(0.01))

        print("status: " + str(status))
        print("self.cancelled: "+ str(self.cancelled))
        print("self.goal_reached: " + str(self.goal_reached))

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

        rospy.sleep(rospy.Duration.from_sec(0.5))
        status = self.edge_action_manager.client.get_state()
        self.pub_status(status)

        rospy.loginfo("move action status: {}, goal reached: {}, inc: {}".format(status_mapping[status], result, inc))
        return result, inc
    
    
    def pub_status(self, status):
        
        if status != self.prev_status:
            d = {}
            d["goal"] = self.edge_action_manager.destination_node["node"]["name"]
            d["final_goal"] = self.final_goal
            d["action"] = self.edge_action_manager.current_action.upper()
            d["status"] = status_mapping[status]
            
            self.move_act_pub.publish(String(json.dumps(d)))
        self.prev_status = status
###################################################################################################################
        

###################################################################################################################
if __name__ == "__main__":
    rospy.init_node("topological_navigation")
    mode = "normal"
    server = TopologicalNavServer(rospy.get_name(), mode)
    rospy.spin()

    rospy.loginfo("Exiting.")
###################################################################################################################
