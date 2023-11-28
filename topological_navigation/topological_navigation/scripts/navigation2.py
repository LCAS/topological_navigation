#!/usr/bin/env python
"""
Created on Tue Nov 5 22:02:24 2023
@author: Geesara Kulathunga (ggeesara@gmail.com)

"""

import rclpy, json
from topological_navigation_msgs.msg import NavStatistics, CurrentEdge, ClosestEdges, TopologicalRoute, GotoNodeFeedback, ExecutePolicyModeFeedback
from topological_navigation_msgs.srv import EvaluateEdge, EvaluateNode
from topological_navigation_msgs.action import GotoNode, ExecutePolicyMode
from topological_navigation_msgs.action import ExecutePolicyMode
from std_msgs.msg import String
import os 
from action_msgs.msg import GoalStatus
from topological_navigation.route_search2 import RouteChecker, TopologicalRouteSearch2, get_route_distance
from topological_navigation.navigation_stats import nav_stats
from topological_navigation.scripts.param_processing import ParameterUpdaterNode
from topological_navigation.tmap_utils import *
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, QoSDurabilityPolicy
from rclpy.action import ActionServer
from rclpy import Parameter 
from topological_navigation.edge_action_manager2 import EdgeActionManager
from topological_navigation.edge_reconfigure_manager2 import EdgeReconfigureManager
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup 
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor 
from threading import Lock

from ament_index_python.packages import get_package_share_directory
from topological_navigation.scripts.actions_bt import ActionsType 
# A list of parameters topo nav is allowed to change and their mapping from dwa speak.
# If not listed then the param is not sent, e.g. TrajectoryPlannerROS doesn't have tolerances.
DYNPARAM_MAPPING = {
    "dwb_core::DWBLocalPlanner": {
        "FollowPath.yaw_goal_tolerance": "FollowPath.yaw_goal_tolerance",
        "FollowPath.xy_goal_tolerance": "FollowPath.xy_goal_tolerance",
    },
    "TebLocalPlannerROS": {
        "FollowPath.yaw_goal_tolerance": "FollowPath.yaw_goal_tolerance",
        "FollowPath.xy_goal_tolerance": "FollowPath.xy_goal_tolerance",
    },
}
    

###################################################################################################################
        
###################################################################################################################
class TopologicalNavServer(rclpy.node.Node):
    
    _feedback = GotoNodeFeedback()
    _result = GotoNode.Result()

    _feedback_exec_policy = ExecutePolicyModeFeedback()
    _result_exec_policy = ExecutePolicyMode.Result()

    def __init__(self, name, mode):
        super().__init__(name)
        rclpy.get_default_context().on_shutdown(self._on_node_shutdown)
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
        self.ACTIONS = ActionsType()

        
        self.declare_parameter('navigation_action_name', Parameter.Type.STRING)
        self.declare_parameter('navigation_actions', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('navigation_action_goal', Parameter.Type.STRING_ARRAY)
        self.declare_parameter("max_dist_to_closest_edge", Parameter.Type.DOUBLE)
        self.declare_parameter('reconfigure_edges', Parameter.Type.BOOL)
        self.declare_parameter('reconfigure_edges_srv', Parameter.Type.BOOL)
        self.declare_parameter("nav_planner", Parameter.Type.STRING)

        self.declare_parameter('use_nav2_follow_route', Parameter.Type.BOOL)
        self.declare_parameter(self.ACTIONS.BT_DEFAULT, Parameter.Type.STRING)
        self.declare_parameter(self.ACTIONS.BT_IN_ROW, Parameter.Type.STRING)
        self.declare_parameter(self.ACTIONS.BT_GOAL_ALIGN, Parameter.Type.STRING)

        self.navigation_action_name = self.get_parameter_or("navigation_action_name", Parameter('str', Parameter.Type.STRING, self.ACTIONS.NAVIGATE_TO_POSE)).value
        self.navigation_actions = self.get_parameter_or("navigation_actions", Parameter('str', Parameter.Type.STRING_ARRAY, self.ACTIONS.navigation_actions)).value

        self.use_nav2_follow_route = self.get_parameter_or("use_nav2_follow_route", Parameter('bool', Parameter.Type.BOOL, False)).value
        
        bt_tree_default = os.path.join(get_package_share_directory('topological_navigation'), 'config', 'bt_tree_default.xml')
        bt_tree_in_row = os.path.join(get_package_share_directory('topological_navigation'), 'config', 'bt_tree_in_row.xml')
        bt_tree_goal_align = os.path.join(get_package_share_directory('topological_navigation'), 'config', 'bt_tree_goal_align.xml')
        self.bt_trees = {}
        self.bt_trees[self.ACTIONS.NAVIGATE_TO_POSE] =  self.get_parameter_or(self.ACTIONS.BT_DEFAULT, Parameter('str'
                                       , Parameter.Type.STRING, bt_tree_default)).value
        self.bt_trees[self.ACTIONS.ROW_TRAVERSAL] =   self.get_parameter_or(self.ACTIONS.BT_IN_ROW
                                        , Parameter('str', Parameter.Type.STRING, bt_tree_in_row)).value
        self.bt_trees[self.ACTIONS.GOAL_ALIGN] =   self.get_parameter_or(self.ACTIONS.BT_GOAL_ALIGN
                                        , Parameter('str', Parameter.Type.STRING, bt_tree_goal_align)).value

        if not self.navigation_action_name in self.navigation_actions:
            self.navigation_actions.append(self.navigation_action_name)
        
        self.latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.stats_pub = self.create_publisher(NavStatistics, "topological_navigation/Statistics", qos_profile=self.latching_qos)
        self.route_pub = self.create_publisher(TopologicalRoute, "topological_navigation/Route", qos_profile=self.latching_qos)
        self.cur_edge = self.create_publisher(String, "current_edge", qos_profile=self.latching_qos)
        self.move_act_pub =  self.create_publisher(String, "topological_navigation/move_action_status", qos_profile=self.latching_qos)
        self._map_received = False
        self._localization_received = False 

        self.callback_group_map = ReentrantCallbackGroup()
        self.subs_topmap = self.create_subscription(String, '/topological_map_2', self.MapCallback, callback_group=self.callback_group_map, qos_profile=self.latching_qos)
        self.get_logger().info("Navigation waiting for the Topological Map...")

        while rclpy.ok():
            rclpy.spin_once(self)
            if self._map_received:
                self.get_logger().info("Navigation received the Topological Map")
                break 
        
        self.edge_action_manager = EdgeActionManager()

        self.edge_reconfigure = self.get_parameter_or("reconfigure_edges", Parameter('bool', Parameter.Type.BOOL, True)).value
        self.srv_edge_reconfigure = self.get_parameter_or("reconfigure_edges_srv", Parameter('bool', Parameter.Type.BOOL, False)).value 
        if self.edge_reconfigure:
            self.edgeReconfigureManager = EdgeReconfigureManager()
        else:
            self.get_logger().warn("Edge Reconfigure Unavailable")

        self.get_logger().info("Subscribing to Localisation Topics...")
        self.subs_closest_node = self.create_subscription(String, 'closest_node', self.closestNodeCallback, qos_profile=self.latching_qos)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self._localization_received:
                self.get_logger().info("Navigation received the localisation info")
                break 

        self.subs_closest_edges = self.create_subscription(ClosestEdges, 'closest_edges', self.closestEdgesCallback, qos_profile=self.latching_qos)
        self.subs_current_node = self.create_subscription(String, 'current_node', self.currentNodeCallback, qos_profile=self.latching_qos)

        self.get_logger().info("...done")
        
        try:
            self.get_logger().info("Waiting for restrictions...")
            self.evaluate_edge_srv = self.create_client(EvaluateEdge, '/restrictions_manager/evaluate_edge')
            if not self.evaluate_edge_srv.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning('/restrictions_manager/evaluate_edge service not available')
                self.get_logger().warning("Restrictions Unavailable")
                self.using_restrictions = False
            else:
                self.evaluate_node_srv = self.create_client(EvaluateNode, '/restrictions_manager/evaluate_node')
                self.get_logger().info("Restrictions Available")
                self.using_restrictions = True
        except Exception as e:
            self.get_logger().error("Error while calling servie /restrictions_manager/evaluate_edge {}".format(e))
           

        # this keeps the runtime state of the fail policies that are currently in execution 
        self.executing_fail_policy = {}
        self.callback_group_gotonode = ReentrantCallbackGroup()
        self.callback_group_policy = ReentrantCallbackGroup()
        # Creating Action Server for navigation
        self.get_logger().info("Creating GO-TO-NODE action server...")  
        self._as  =  ActionServer(self, GotoNode, name, execute_callback=self.executeCallback
                                  , cancel_callback=self.preemptCallback, callback_group=self.callback_group_gotonode)
        self._as_action_feedback_pub = self.create_publisher(GotoNodeFeedback, name + '/feedback', qos_profile=self.latching_qos)
        self.get_logger().info("...done")
   
        # Creating Action Server for execute policy
        self.get_logger().info("Creating EXECUTE POLICY MODE action server...")
        self._as_exec_policy = ActionServer(self, ExecutePolicyMode, "topological_navigation/execute_policy_mode", 
                            execute_callback=self.executeCallbackexecpolicy, cancel_callback=self.preemptCallbackexecpolicy
                            , callback_group=self.callback_group_policy)
        self._as_exec_policy_action_feedback_pub = self.create_publisher(ExecutePolicyModeFeedback, 'topological_navigation/execute_policy_mode/feedback'
                                                                                    , qos_profile=self.latching_qos)
        
        self.update_params_planner = ParameterUpdaterNode("controller_server") #TODO change the name 

        self.get_logger().info("...done")
        self.get_logger().info("All Done.")
        
        
        
    def _on_node_shutdown(self):
        if self.navigation_activated:
            self.preempted = True
            self.cancel_current_action(timeout_secs=2)


    def init_reconfigure(self):
        self.nav_planner  = self.get_parameter_or("nav_planner", Parameter('str', Parameter.Type.STRING, "dwb_core::DWBLocalPlanner")).value
        planner = self.nav_planner.split("/")[-1]
        if not planner in DYNPARAM_MAPPING:
            DYNPARAM_MAPPING[planner] = {}
        self.get_logger().info("Creating reconfigure client for {}".format(self.nav_planner))
        self.init_dynparams = self.update_params_planner.get_params()
        

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

        params = {"FollowPath.yaw_goal_tolerance": cytol, "FollowPath.xy_goal_tolerance": cxygtol}
        self.get_logger().info("Reconfiguring %s with %s" % (self.nav_planner, params))
        print("Intermediate: {}".format(intermediate))
        self.update_params_planner.set_params(params)     

    def reset_reconf(self):
        self.update_params_planner.set_params(self.init_dynparams)

    def MapCallback(self, msg):
        """
         This Function updates the Topological Map everytime it is called
        """
        self.lnodes = json.loads(msg.data)
        self.topol_map = self.lnodes["pointset"]
        self.rsearch = TopologicalRouteSearch2(self.lnodes)
        self.route_checker = RouteChecker(self.lnodes)
        self.make_navigation_edge()
        self._map_received = True


    def make_navigation_edge(self):

        self.navigation_action_edge = {}
        self.navigation_action_edge["action"] = self.navigation_action_name
        self.navigation_action_edge["edge_id"] = "navigation_action_edge"
        navigation_action_goal = {}
        if not navigation_action_goal:
            for node in self.lnodes["nodes"]:
                for edge in node["node"]["edges"]:
                    if edge["action"] == self.navigation_action_name:
                        navigation_action_goal["action_type"] = edge["action_type"]
                        navigation_action_goal["goal"] = edge["goal"]
                        break
                else:
                    continue
                break
        if not navigation_action_goal:
            navigation_action_goal["action_type"] = "geometry_msgs/PoseStamped"
            navigation_action_goal["goal"] = {}
            navigation_action_goal["goal"]["target_pose"] = {}
            navigation_action_goal["goal"]["target_pose"]["pose"] = "$node.pose"
            navigation_action_goal["goal"]["target_pose"]["header"] = {}
            navigation_action_goal["goal"]["target_pose"]["header"]["frame_id"] = "$node.parent_frame"

        self.navigation_action_edge["action_type"] = navigation_action_goal["action_type"]
        self.navigation_action_edge["goal"] = navigation_action_goal["goal"]
        self.get_logger().info("Move Base Goal set to {}".format(navigation_action_goal["action_type"]))


    def executeCallback(self, goal):
        """
        This Functions is called when the topo nav Action Server is called
        """
        self.get_logger().info("\n####################################################################################################")
        self.get_logger().info("Processing GO-TO-NODE goal (No Orientation = {})".format(goal.request.no_orientation))
        can_start = False
        if self.cancel_current_action(timeout_secs=10):
            # we successfully stopped the previous action, claim the title to activate navigation
            self.navigation_activated = True
            can_start = True
        if can_start:
            self.cancelled = False
            self.preempted = False
            self.final_goal = False
            self.no_orientation = goal.request.no_orientation
            self.executing_fail_policy = {}
            self._feedback.route = "Starting..."
            self._as_action_feedback_pub.publish(self._feedback)
            self.navigate(goal.request.target)
        else:
            self.get_logger().warning("Could not cancel current navigation action, GO-TO-NODE goal aborted")

        self.navigation_activated = False
        nav_current_state = self.edge_action_manager.get_state()
        if (nav_current_state == GoalStatus.STATUS_SUCCEEDED):
            goal.succeed()
        else: 
            goal.abort()

        self.get_logger().warning("Done processing the nav action GO-TO-NODE....")
        result = GotoNode.Result()
        result.success = self._result.success 
        
        return result 

    def executeCallbackexecpolicy(self, goal):
        """
        This Function is called when the execute policy Action Server is called
        """
        print("\n####################################################################################################")
        self.get_logger().info("Processing EXECUTE POLICY MODE goal")
        can_start = False
        if self.cancel_current_action(timeout_secs=10):
            # we successfully stopped the previous action, claim the title to activate navigation
            self.navigation_activated = True
            can_start = True

        if can_start:

            self.cancelled = False
            self.preempted = False
            self.final_goal = False
            
            self.max_dist_to_closest_edge = self.get_parameter_or("max_dist_to_closest_edge",  Parameter('double', Parameter.Type.DOUBLE, 1.0)).value 
            if self.closest_edges.distances[0] > self.max_dist_to_closest_edge or self.current_node != "none":
                self.nav_from_closest_edge = False
            else:
                self.nav_from_closest_edge = True
            
            route = goal.request.route
            valid_route = self.route_checker.check_route(route)
            
            if valid_route and (route.source[0] == self.current_node or route.source[0] == self.closest_node):
                final_edge = get_edge_from_id_tmap2(self.lnodes, route.source[-1], route.edge_id[-1])
                target = final_edge["node"]
                route = self.enforce_navigable_route(route, target)
                result = self.execute_policy(route, target)
            else:
                result = False
                self.cancelled = True
                self.get_logger().error("Invalid route in execute policy mode goal")

            if not self.cancelled and not self.preempted:
                self._result_exec_policy.success = result
            else:
                if not self.preempted:
                    self._result_exec_policy.success = result
                else:
                    self._result_exec_policy.success = False
        else: 
            self.get_logger().warning("Could not cancel current navigation action, EXECUTE POLICY MODE goal aborted.")

        self.navigation_activated = False
        goal.succeed()
        self.get_logger().warning("Done processing the nav action EXECUTE POLICY MODE....")
        result = ExecutePolicyMode.Result()
        result.success = self._result_exec_policy.success 
        return result


    def preemptCallback(self, goal_handle):
        self.get_logger().warning("Preempting GO-TO-NODE goal")
        self.preempted = True
        self.cancel_current_action(timeout_secs=2)


    def preemptCallbackexecpolicy(self, goal_handle):
        self.get_logger().warning("Preempting EXECUTE POLICY MODE goal")
        self.preempted = True
        self.cancel_current_action(timeout_secs=2)


    def closestNodeCallback(self, msg):
        self._localization_received = True 
        self.closest_node = msg.data


    def closestEdgesCallback(self, msg):
        self.closest_edges = msg


    def currentNodeCallback(self, msg):
        if self.current_node != msg.data:  # is there any change on this topic?
            self.current_node = msg.data  # we are at this new node
            if msg.data != "none":  # are we at a node?
                self.get_logger().info("New node reached: {}".format(self.current_node))
                if self.navigation_activated:  # is the action server active?
                    if self.stat:
                        self.stat.set_at_node()
                    # if the robot reached and intermediate node and the next action is move base goal has been reached
                    if (
                        self.current_node == self.current_target
                        and self._target != self.current_target
                        and self.next_action in self.navigation_actions
                        and self.current_action in self.navigation_actions
                        and self.fluid_navigation
                    ):
                        self.get_logger().info("Intermediate node reached: {}".format(self.current_node))
                        self.goal_reached = True

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
        self.get_logger().info(" {} Nodes on route".format(nnodes))

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
            self.get_logger().info("First action: {}".format(a))
        else:
            self.get_logger().error("Failed to get edge from id {}. Invalid route".format(route.edge_id[0]))
            return False, inc
        
        if not self.nav_from_closest_edge:        
            # If the robot is not on a node or the first action is not move base type
            # navigate to closest node waypoint (only when first action is not move base)
            if a not in self.navigation_actions:
                self.get_logger().info("The action of the first edge in the route is not a move base action")
                self.get_logger().info("Current node is {}".format(self.current_node))
                
            if self.current_node == "none" and a not in self.navigation_actions:
                self.next_action = a
                self.get_logger().info("Do {} to origin {}".format(self.navigation_action_name, o_node["node"]["name"]))
    
                # 5 degrees tolerance
                params = {"yaw_goal_tolerance": 0.087266}
                self.update_params_planner.set_params(params)

                self.current_target = Orig
                nav_ok, inc = self.execute_action(self.navigation_action_edge, o_node)
                self.get_logger().info("Navigation Finished Successfully") if nav_ok else self.get_logger().warning("Navigation Failed")
                
            elif a not in self.navigation_actions:
                navigation_action_act = False
                for i in o_node["node"]["edges"]:
                    # Check if there is a navigation action in the edages of this node
                    # if not is dangerous to move
                    if i["action"] in self.navigation_actions:
                        navigation_action_act = True

                if not navigation_action_act:
                    self.get_logger().warning("Could not find a move base action in the edges of origin {}. Unsafe to move".format(o_node["node"]["name"]))
                    self.get_logger().info("Action not taken, outputing success")
                    nav_ok = True
                    inc = 0
                else:
                    self.get_logger().info("Getting to the exact pose of origin {}".format(o_node["node"]["name"]))
                    self.current_target = Orig
                    nav_ok, inc = self.execute_action(self.navigation_action_edge, o_node)
                    self.get_logger().info("Navigation Finished Successfully") if nav_ok else self.get_logger().warning("Navigation Failed")
                
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

            self.get_logger().info("From {} do ({}) to {}".format(route.source[rindex], a, cedg["node"]))

            current_edge = "%s--%s" % (cedg["edge_id"], self.topol_map)
            self.get_logger().info("Current edge: {}".format(current_edge))
            msg = String()
            msg.data = current_edge 
            self.cur_edge.publish(msg)

            if not exec_policy:
                self._feedback.route = "%s to %s using %s" % (route.source[rindex], cedg["node"], a)
                self._as_action_feedback_pub.publish(self._feedback)
            else:
                self.publish_feedback_exec_policy()

            cnode = self.rsearch.get_node_from_tmap2(cedg["node"])
            onode = self.rsearch.get_node_from_tmap2(route.source[rindex])

            # do not care for the orientation of the waypoint if is not the last waypoint AND
            # the current and following action are navigation or human_aware_navigation
            # and when the fuild_navigation is true
            if rindex < len(route.edge_id) - 1 and a1 in self.navigation_actions and a in self.navigation_actions and self.fluid_navigation:
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
                nav_ok, inc = self.execute_action(cedg, cnode, onode)
            else:
                nav_ok, inc, recovering, route, replanned = self.execute_action_fail_recovery(cedg, cnode, route, rindex, onode, target)
            self.edge_reconf_end()

            params = {"yaw_goal_tolerance": 0.087266, "xy_goal_tolerance": 0.1}
            self.update_params_planner.set_params(params)

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
                self.get_logger().info("Navigation Finished on {} ({}/{})".format(dt_text, operation_time, time_to_wp))
            else:
                if not_fatal:
                    self.get_logger().warning("Navigation Failed on {} ({}/{})".format(dt_text, operation_time, time_to_wp))
                    self.stat.status = "failed"
                else:
                    self.get_logger().warning("Fatal Fail on {} ({}/{})".format(dt_text, operation_time, time_to_wp))
                    self.stat.status = "fatal"

            self.publish_stats()

            current_edge = "none"
            msg = String()
            msg.data = current_edge
            self.cur_edge.publish(msg)

            self.current_action = "none"
            self.next_action = "none"
            
            if not replanned:
                rindex = rindex + 1

        self.reset_reconf()

        self.navigation_activated = False

        result = nav_ok
        return result, inc

    def navigate_to_poses(self, route, target, exec_policy):
        """
        This function follows the chosen route to reach the goal.
        """
        self.navigation_activated = True
        
        nnodes = len(route.source)
        Orig = route.source[0]
        Targ = target
        self._target = Targ

        self.init_reconfigure()
        self.get_logger().info(" {} Nodes on route".format(nnodes))

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
            self.get_logger().info("First action: {}".format(a))
        else:
            self.get_logger().error("Failed to get edge from id {}. Invalid route".format(route.edge_id[0]))
            return False, inc
        
        if not self.nav_from_closest_edge:        
            # If the robot is not on a node or the first action is not move base type
            # navigate to closest node waypoint (only when first action is not move base)
            if a not in self.navigation_actions:
                self.get_logger().info("The action of the first edge in the route is not a move base action")
                self.get_logger().info("Current node is {}".format(self.current_node))
                
            if self.current_node == "none" and a not in self.navigation_actions:
                self.next_action = a
                self.get_logger().info("Do {} to origin {}".format(self.navigation_action_name, o_node["node"]["name"]))
    
                # 5 degrees tolerance
                params = {"yaw_goal_tolerance": 0.087266}
                self.update_params_planner.set_params(params)

                self.current_target = Orig
                nav_ok, inc = self.execute_action(self.navigation_action_edge, o_node)
                self.get_logger().info("Navigation Finished Successfully") if nav_ok else self.get_logger().warning("Navigation Failed")
                
            elif a not in self.navigation_actions:
                navigation_action_act = False
                for i in o_node["node"]["edges"]:
                    # Check if there is a navigation action in the edages of this node
                    # if not is dangerous to move
                    if i["action"] in self.navigation_actions:
                        navigation_action_act = True

                if not navigation_action_act:
                    self.get_logger().warning("Could not find a move base action in the edges of origin {}. Unsafe to move".format(o_node["node"]["name"]))
                    self.get_logger().info("Action not taken, outputing success")
                    nav_ok = True
                    inc = 0
                else:
                    self.get_logger().info("Getting to the exact pose of origin {}".format(o_node["node"]["name"]))
                    self.current_target = Orig
                    nav_ok, inc = self.execute_action(self.navigation_action_edge, o_node)
                    self.get_logger().info("Navigation Finished Successfully") if nav_ok else self.get_logger().warning("Navigation Failed")

        
        route_edges = []
        route_dests = []
        route_origins = []
        route_actions_list = []

        while rindex < (len(route.edge_id)):
            cedg = get_edge_from_id_tmap2(self.lnodes, route.source[rindex], route.edge_id[rindex])
            a = cedg["action"]
            route_actions_list.append(a)
            if(rindex == 0):
                self.stat = nav_stats(route.source[rindex], cedg["node"], self.topol_map, cedg["edge_id"])
                dt_text = self.stat.get_start_time_str()

            if(rindex == (len(route.edge_id)-1)):
                self.stat.target = cedg["edge_id"]

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
            current_edge = "%s--%s" % (cedg["edge_id"], self.topol_map)
            self.get_logger().info("From {} do ({}) to {} from edge {}".format(route.source[rindex], a, cedg["node"], current_edge))
            # msg = String()
            # msg.data = current_edge 
            # self.cur_edge.publish(msg)
            # if not exec_policy:
            #     self._feedback.route = "%s to %s using %s" % (route.source[rindex], cedg["node"], a)
            #     self._as_action_feedback_pub.publish(self._feedback)
            # else:
            #     self.publish_feedback_exec_policy()
            cnode = self.rsearch.get_node_from_tmap2(cedg["node"])
            onode = self.rsearch.get_node_from_tmap2(route.source[rindex])
            # do not care for the orientation of the waypoint if is not the last waypoint AND
            # the current and following action are navigation or human_aware_navigation
            # and when the fuild_navigation is true
            # if rindex < len(route.edge_id) - 1 and a1 in self.navigation_actions and a in self.navigation_actions and self.fluid_navigation:
            #     self.reconf_movebase(cedg, cnode, True)
            # else:
            #     if self.no_orientation:
            #         self.reconf_movebase(cedg, cnode, True)
            #     else:
            #         self.reconf_movebase(cedg, cnode, False)
            self.current_target = cedg["node"]
            route_edges.append(cedg)
            route_dests.append(cnode)
            route_origins.append(onode)
            # params = {"yaw_goal_tolerance": 0.087266, "xy_goal_tolerance": 0.1}
            # self.update_params_planner.set_params(params)
            rindex = rindex + 1

        self.get_logger().info(" ========== Action list {} ".format(route_actions_list))
        nav_ok, inc, status  = self.execute_actions(route_edges, route_dests, route_origins
                            , action_name=self.ACTIONS.NAVIGATE_THROUGH_POSES)
        
        self.stat.set_ended(self.current_node)
        dt_text = self.stat.get_finish_time_str()
        operation_time = self.stat.operation_time
        time_to_wp = self.stat.time_to_wp
        not_fatal = nav_ok
        if self.cancelled:
            nav_ok = True
        if self.preempted:
            not_fatal = False
            nav_ok = False

        if nav_ok:
            self.stat.status = "success"
            self.get_logger().info("Navigation Finished on {} ({}/{})".format(dt_text, operation_time, time_to_wp))
        else:
            if not_fatal:
                self.get_logger().warning("Navigation Failed on {} ({}/{})".format(dt_text, operation_time, time_to_wp))
                self.stat.status = "failed"
            else:
                self.get_logger().warning("Fatal Fail on {} ({}/{})".format(dt_text, operation_time, time_to_wp))
                self.stat.status = "fatal"
        self.publish_stats()
        self.reset_reconf()
        self.navigation_activated = False
        result = nav_ok
        return result, inc, status 
    
    def navigate(self, target):
        """
        This function takes the target node and plans the actions that are required
        to reach it.
        """
        result = False
        if not self.cancelled:
            g_node = self.rsearch.get_node_from_tmap2(target)
            
            self.max_dist_to_closest_edge = self.get_parameter_or("max_dist_to_closest_edge",  Parameter('double', Parameter.Type.DOUBLE, 1.0)).value 
          
            if self.closest_edges.distances and (self.closest_edges.distances[0] > self.max_dist_to_closest_edge or self.current_node != "none"):
                self.nav_from_closest_edge = False
                o_node = self.rsearch.get_node_from_tmap2(self.closest_node)
                self.get_logger().info("Planning from the closest NODE: {}".format(self.closest_node))
            else:
                self.nav_from_closest_edge = True
                o_node, the_edge = self.orig_node_from_closest_edge(g_node)
                self.get_logger().info("Planning from the closest EDGE: {}".format(the_edge["edge_id"]))
                
            self.get_logger().info("Navigating From Origin {} to Target {} ".format(o_node["node"]["name"], target))
             
            # Everything is Awesome!!!
            # Target and Origin are not None
            if (g_node is not None) and (o_node is not None):
                if g_node["node"]["name"] != o_node["node"]["name"]:
                    route = self.rsearch.search_route(o_node["node"]["name"], target)
                    route = self.enforce_navigable_route(route, target)
                    if route.source:
                        self.get_logger().info("Navigating Case 1: Following route")
                        self.publish_route(route, target)
                        if(self.use_nav2_follow_route):
                            result, inc, status = self.navigate_to_poses(route, target, 0)
                        else:
                            result, inc = self.followRoute(route, target, 0)
                        self.get_logger().info("Navigating Case 1 -> res: {}".format(inc))
                    else:
                        self.get_logger().warning("Navigating Case 1a: There is no route from {} to {}. Check your edges.".format(o_node["node"]["name"], target))
                        self.cancelled = True
                        result = False
                        inc = 1
                        self.get_logger().info("Navigating Case 1a -> res: {}".format(inc))
                else:      
                    if self.nav_from_closest_edge:
                        result, inc = self.to_goal_node(g_node, the_edge)
                    else:
                        result, inc = self.to_goal_node(g_node)
            else:
                self.get_logger().warning("Navigating Case 3: Target or Origin Nodes were not found on Map")
                self.cancelled = True
                result = False
                inc = 1
                self.get_logger().info("Navigating Case 3 -> res: {}".format(inc))
        
        if (not self.cancelled) and (not self.preempted):
            self._result.success = result
            if result:
                self._feedback.route = target
                self._as_action_feedback_pub.publish(self._feedback)
                # self._as.set_succeeded(self._result)
            else:
                self._feedback.route = self.current_node
                self._as_action_feedback_pub.publish(self._feedback)
                # self._as.set_aborted(self._result)
        else:
            if not self.preempted:
                self._feedback.route = self.current_node
                self._as_action_feedback_pub.publish(self._feedback)
                self._result.success = result
                # self._as.set_aborted(self._result)
            else:
                self._result.success = False
                # self._as.set_preempted(self._result)
        
 

    def execute_policy(self, route, target):
        succeeded, inc = self.followRoute(route, target, 1)
        if succeeded:
            self.get_logger().info("Navigation Finished Successfully")
            self.publish_feedback_exec_policy(GoalStatus.STATUS_SUCCEEDED)
        else:
            if self.cancelled and self.preempted:
                self.get_logger().warning("Fatal Fail")
                self.publish_feedback_exec_policy(GoalStatus.STATUS_CANCELED)
            elif self.cancelled:
                self.get_logger().warning("Navigation Failed")
                self.publish_feedback_exec_policy(GoalStatus.STATUS_ABORTED)
        return succeeded
    

    def publish_feedback_exec_policy(self, nav_outcome=None):
        if self.current_node == "none":  # Happens due to lag in fetch system
            if self.current_node == "none":
                self._feedback_exec_policy.current_wp = self.closest_node
            else:
                self._feedback_exec_policy.current_wp = self.current_node
        else:
            self._feedback_exec_policy.current_wp = self.current_node
        if nav_outcome is not None:
            self._feedback_exec_policy.status = nav_outcome
        self._as_exec_policy_action_feedback_pub.publish(self._feedback_exec_policy)
        
        
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
        
        self.get_logger().info("Target and Origin Nodes are the same")
        self.current_target = g_node["node"]["name"]
        if the_edge is None:
            # Check if there is a navigation  action in the edges of this node and choose the earliest one in the 
            # list of navigation actions. If not is dangerous to move.
            act_ind = 100
            for i in g_node["node"]["edges"]:
                c_action_server = i["action"]
                if c_action_server in self.navigation_actions:
                    c_ind = self.navigation_actions.index(c_action_server)
                    if c_ind < act_ind:
                        act_ind = c_ind
                        the_edge = i
        if the_edge is None:
            self.get_logger().warning("Navigating Case 2a: Could not find a move base action in the edges of target {}. Unsafe to move".format(g_node["node"]["name"]))
            self.get_logger().info("Action not taken, outputting success")
            result=True
            inc=0
            self.get_logger().info("Navigating Case 2a -> res: {}".format(inc))
        else:
            self.get_logger().info("Navigating Case 2: Getting to the exact pose of target {}".format(g_node["node"]["name"]))
            self.final_goal = True
            self.current_target = g_node["node"]["name"]
            origin_name,_ = get_node_names_from_edge_id_2(self.lnodes, the_edge["edge_id"])
            origin_node = self.rsearch.get_node_from_tmap2(origin_name)

            if(self.edge_reconf_start(the_edge)):
                result, inc = self.execute_action(the_edge, g_node, origin_node)
                self.edge_reconf_end()
            else:
                result = False 
            if not result:
                self.get_logger().warning("Navigation Failed")
                inc=1
            else:
                self.get_logger().info("Navigation Finished Successfully")
            self.get_logger().info("Navigating Case 2 -> res: {} ".format(inc))
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

    def edge_reconf_start(self, edge):

        if self.edge_reconfigure:
            if not self.srv_edge_reconfigure:
                self.edgeReconfigureManager.register_edge(edge)
                if (not self.edgeReconfigureManager.initialise()):
                    return False 
                self.edgeReconfigureManager.reconfigure()
            else:
                self.edgeReconfigureManager.srv_reconfigure(edge["edge_id"])
        return True 


    def edge_reconf_end(self):
        if self.edge_reconfigure and not self.srv_edge_reconfigure and self.edgeReconfigureManager.active:
            self.edgeReconfigureManager._reset()


    def cancel_current_action(self, timeout_secs=-1):
        """
        Cancels the action currently in execution. Returns True if the current goal is correctly ended.
        """
        self.get_logger().info("Cancelling current navigation goal, timeout_secs = {}...".format(timeout_secs))
        self.edge_action_manager.preempt()
        self.cancelled = True
        self.navigation_activated = False 
        self.get_logger().info("Navigation active: " + str(self.navigation_activated))
        return not self.navigation_activated


    def publish_route(self, route, target):
        stroute = TopologicalRoute()
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
        self.get_logger().info(" {}".format(self.stat.time_to_wp))
        pubst.time_to_waypoint = float(self.stat.time_to_wp)
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
        nav_ok, inc = self.execute_action(edge, destination_node, origin_node)
        new_route = route
        recovering = False
        replanned = False
        # this means the action is aborted -> execute the fail policy
        # make sure that if the goal is cancelled by the client we don't enter here
        if not nav_ok and not self.preempted:
            self.get_logger().info("\t>> route: {}".format(route))
            route_updated = False
            while not route_updated:
                policy, state = self.get_fail_policy_state(edge)
                if state < len(policy):
                    rec_action = policy[state]
                    self.get_logger().info(">> EXECUTING FAIL POLICY ACTION: {}.".format(rec_action))
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
                        recovering = True
                        replanned = False
                    elif rec_action[0] == "replan":
                        _route = self.rsearch.search_route(origin_node["node"]["name"], target, avoid_edges=[edge["edge_id"]])
                        _route = self.enforce_navigable_route(_route, target)
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

            self.get_logger().info("\t>> new route: {}".format(new_route))
        return nav_ok, inc, recovering, new_route, replanned
    
    def execute_actions(self, edges, destination_nodes, origin_nodes=None,  action_name=None):

        inc = 0
        result = True
        self.goal_reached = False
        self.prev_status = None

        self.edge_action_manager.initialise(edges, destination_nodes, origin_nodes, action_name=action_name, bt_trees=self.bt_trees)
        self.edge_action_manager.execute()
        status = self.edge_action_manager.get_state()
        self.pub_status(status)
        
        if ((status == GoalStatus.STATUS_EXECUTING or status == GoalStatus.STATUS_UNKNOWN) and not self.cancelled and not self.goal_reached):
            try:
                self.get_logger().info(" Current status  {} ".format(self.edge_action_manager.get_status_msg(status)))
                self.pub_status(status)
            except Exception as e:
                pass
            
        res = self.edge_action_manager.get_result()
        if status != GoalStatus.STATUS_SUCCEEDED:
            if not self.goal_reached:
                result = False
                if status is GoalStatus.STATUS_CANCELED:
                    self.preempted = True
            else:
                result = True

        if not res:
            if not result:
                inc = 1
            else:
                inc = 0

        status = self.edge_action_manager.get_state()
        self.pub_status(status)

        self.get_logger().info("Navigation action status: {}, goal reached: {}, inc: {}".format(self.edge_action_manager.get_status_msg(status), result, inc))
        return result, inc, status 
    
    def execute_action(self, edge, destination_node, origin_node=None):

        inc = 0
        result = True
        self.goal_reached = False
        self.prev_status = None

        if self.using_restrictions and edge["edge_id"] != "navigation_action_edge":
            self.get_logger().info("Evaluating restrictions on edge {}".format(edge["edge_id"]))
            ev_edge_msg = EvaluateEdge()
            ev_edge_msg.edge = edge["edge_id"]
            ev_edge_msg.runtime = True
            future = self.evaluate_edge_srv.call_async(ev_edge_msg)
            while rclpy.ok():
                rclpy.spin_once(self,)
                if future.done():
                    resp = future.result()
                    break 
            if resp.success and resp.evaluation:
                self.get_logger().warning("The edge is restricted, stopping navigation")
                result = False
                inc = 1
                return result, inc

            self.get_logger().info("Evaluating restrictions on node {}".format(destination_node["node"]["name"]))
            ev_node_msg = EvaluateNode()
            ev_node_msg.node = destination_node["node"]["name"]
            ev_node_msg.runtime = True
            future = self.evaluate_node_srv.call_async(ev_node_msg)
            while rclpy.ok():
                rclpy.spin_once(self,)
                if future.done():
                    resp = future.result()
                    break 
            if resp.success and resp.evaluation:
                self.get_logger().warning("The node is restricted, stopping navigation")
                result = False
                inc = 1
                return result, inc

        self.edge_action_manager.initialise(edge, destination_node, origin_node)
        self.edge_action_manager.execute()
        status = self.edge_action_manager.get_state()
        self.pub_status(status)
        
        if ((status == GoalStatus.STATUS_EXECUTING or status == GoalStatus.STATUS_UNKNOWN) and not self.cancelled and not self.goal_reached):
            try:
                self.get_logger().info(" Current status  {} ".format(self.edge_action_manager.get_status_msg(status)))
                self.pub_status(status)
            except Exception as e:
                pass
            
        res = self.edge_action_manager.get_result()
        if status != GoalStatus.STATUS_SUCCEEDED:
            if not self.goal_reached:
                result = False
                if status is GoalStatus.STATUS_CANCELED:
                    self.preempted = True
            else:
                result = True

        if not res:
            if not result:
                inc = 1
            else:
                inc = 0

        status = self.edge_action_manager.get_state()
        self.pub_status(status)

        self.get_logger().info("Navigation action status: {}, goal reached: {}, inc: {}".format(self.edge_action_manager.get_status_msg(status), result, inc))
        return result, inc
    
    
    def pub_status(self, status):
        if status != self.prev_status:
            d = {}
            d["goal"] = self.edge_action_manager.destination_node["node"]["name"]
            d["final_goal"] = self.final_goal
            d["action"] = self.edge_action_manager.current_action.upper()
            d["status"] = self.edge_action_manager.get_status_msg(status)
            msg = String()
            msg.data = json.dumps(d)
            self.move_act_pub.publish(msg)
        self.prev_status = status
###################################################################################################################

def main():
    rclpy.init(args=None)
    wtags = True
    node = TopologicalNavServer('topological_navigation', wtags)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('shutting down localisation node\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()