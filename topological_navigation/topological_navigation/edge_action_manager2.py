#!/usr/bin/env python
"""
Created on Tue Nov 5 22:02:24 2023
@author: Geesara Kulathunga (ggeesara@gmail.com)

"""
#########################################################################################################
import json, yaml
import operator, collections, copy
import rclpy
import numpy as np  
from functools import reduce  # forward compatibility for Python 3
from rclpy.action import ActionClient 
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from topological_navigation.route_search2 import TopologicalRouteSearch2


# from rospy_message_converter import message_converter
# from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Header 
from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup 
from unique_identifier_msgs.msg import UUID
from rclpy.executors import SingleThreadedExecutor
from topological_navigation.scripts.actions_bt import ActionsType 
from topological_navigation.scripts.param_processing import ParameterUpdaterNode
from topological_navigation.scripts.in_row_operations import RowOperations 
from std_msgs.msg import String


try:
    from collections.abc import Mapping
except ImportError:
    from collections import Mapping

def _import(location, name):
    mod = __import__(location, fromlist=[name]) 
    return getattr(mod, name) 


class dict_tools(object):
                    
    def get_paths_from_nested_dict(self, nested):
        paths = list(self.nested_dict_iter(nested))
        return [{"keys": item[0], "value": item[1]} for item in paths]
    
    def nested_dict_iter(self, nested, prefix=""):
        """
        Recursively loops through a nested dictionary. 
        For each inner-most value generates the list of keys needed to access it.
        """
        for key, value in nested.items():
            path = "{},{}".format(prefix, key)
            if isinstance(value, Mapping):
                for inner_key, inner_value in self.nested_dict_iter(value, path):
                    yield inner_key, inner_value
            else:
                yield path[1:].split(","), value
    
    
    def getFromDict(self, dataDict, mapList):
        return reduce(operator.getitem, mapList, dataDict)


    def setInDict(self, dataDict, mapList, value):     
        self.getFromDict(dataDict, mapList[:-1])[mapList[-1]] = value
        return dataDict
    
class TopoNavEdgeActionMsg():

    def __init__(self,):
        pass 

    def setAction(self, action):
        self.action = action 

    def setNavGoal(self, nav_goal):
        self.nav_goal = nav_goal 

    def setSideEdges(self, side_edges, target_frame_id):
        self.side_edges = side_edges
        self.target_frame_id = target_frame_id

    def getBoundary(self, ):
        path = Path()
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame_id
        path.header = header
        if(len(self.side_edges)>0):
            for key, val in self.side_edges.items():
                for pose in val:
                    path.poses.append(pose)
        return path 
    
    def getTargetFrameId(self, ):
        return self.target_frame_id
    
    def getAction(self):
        return self.action

    def getNavGoal(self):
        return self.nav_goal 
        
    def setControlPluginParams(self, control_plugin_params):
        self.control_plugin_params = control_plugin_params


#########################################################################################################


        
#########################################################################################################
class EdgeActionManager(rclpy.node.Node):
    
    def __init__(self, name="edge_action_manager"):
        super().__init__(name)
        self.client = None        
        self.current_action = "none"
        self.dt = dict_tools()
        
    def init(self, ACTIONS, route_search, update_params_control_server, inrow_step_size=3.0, intermediate_dis=0.7):
        self.ACTIONS = ACTIONS
        self.route_search = route_search
        self.goal_handle = None 
        self.goal_resposne = None 
        self.internal_executor = SingleThreadedExecutor()
        self.latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.bt_trees =  {}
        self.in_row_operation = False
        self.control_server_configs = []
        self.inrow_step_size = inrow_step_size
        self.is_row_boundary_published = False
        self.intermediate_dis = intermediate_dis
        self.nav2_client_callback_group = MutuallyExclusiveCallbackGroup()

        self.update_params_control_server = update_params_control_server
        self.current_robot_pose = None 
        self.odom_sub = self.create_subscription(Odometry, '/odometry/global', self.odom_callback,
                                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_current_node_sub = self.create_subscription(String, 'closest_node', self.set_current_pose
                                , QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
       
        self.boundary_publisher = self.create_publisher(Path, '/boundary_checker', qos_profile=self.latching_qos)
        self.robot_current_status_pub = self.create_publisher(String, '/robot_operation_current_status', qos_profile=self.latching_qos)
       

        self.robot_current_behavior_pub = None
        self.current_node = None 
        self.action_status = 0
        self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_NAVIGATION_STATE 
        self.executor_nav_client = SingleThreadedExecutor()

    def set_current_pose(self, msg):
        self.current_node = msg.data

    def odom_callback(self, msg):
        self.current_robot_pose = msg.pose
    
    def get_nav_action_server_status(self, ):
        return self.ACTIONS.status_mapping 
    
    def get_status_msg(self, status_code):
        try:
            return self.ACTIONS.status_mapping[status_code]
        except Exception as e:
            self.get_logger().error("Status code is invalid {}".format(status_code))
            return self.ACTIONS.status_mapping[0]
        
    def get_goal_cancel_error_msg(self, status_code):
        try:
            return self.ACTIONS.goal_cancel_error_codes[status_code]
        except Exception as e:
            self.get_logger().error("Goal cancel code {}".format(status_code))
            return self.ACTIONS.goal_cancel_error_codes[0]
        
    def initialise(self, bt_trees, edge, destination_node, origin_node=None
                                , action_name=None, package="nav2_msgs.action", in_row_operation=False):
        
        self.bt_trees = bt_trees
        self.in_row_operation = in_row_operation
        if self.in_row_operation:
            try: 
                from robot_behavior_msg.msg import RobotBehavior
                self.robot_current_behavior_pub = self.create_publisher(RobotBehavior, '/robot_current_behavior', qos_profile=self.latching_qos)
            except  Exception as e:
                self.get_logger().error("Edge Action Manager: robot_behavior_msg.msg.RobotBehavior message type is not defined")
                
        if(action_name is not None):
            self.action_name = action_name
        else:
            self.edge = yaml.safe_load(json.dumps(edge)) # no unicode in edge
            self.action_name = self.edge["action"]

        if (self.action_name == self.ACTIONS.ROW_TRAVERSAL or self.action_name == self.ACTIONS.ROW_CHANGE):
            self.action_name = self.ACTIONS.NAVIGATE_TO_POSE #TODO change this to actual  

        if self.action_name != self.current_action:
            self.preempt()

        self.package = package
        self.set_nav_client()
        self.action_status = 0 

        if self.action_name == self.ACTIONS.NAVIGATE_TO_POSE:
            self.destination_node = destination_node
            self.origin_node = origin_node
            self.get_logger().info("Edge Action Manager: Processing edge {}".format(self.edge["edge_id"]))
            self.get_logger().info("Edge Action Manager: Constructing the goal")
            target_pose = self.construct_goal(copy.deepcopy(edge["goal"]), destination_node, origin_node)
            self.action_msgs = self.construct_navigate_to_pose_goal(target_pose)

        if self.action_name == self.ACTIONS.NAVIGATE_THROUGH_POSES:
            poses = {}
            actions = {}
            edge_ids = {}
            segment = 0
            previous_action = ""
            current_action = ""
            total_edges = len(edge)
            
            for index in range(0, total_edges):
                edge_i, dest_i, origin_i = edge[index], destination_node[index], origin_node[index]
                
                edge_i = yaml.safe_load(json.dumps(edge_i)) # no unicode in edge
                current_action = edge_i["action"]
                # self.get_logger().info("Edge Action Manager: -----> before processing edge {} action {}  goal {}".format(edge_i["edge_id"], current_action, edge_i["action"]))
                if(index < (total_edges-1)):
                    next_edge_id = edge[index+1]["edge_id"]
                    current_action = self.get_goal_align_if(edge_i["edge_id"], current_action, next_edge_id)
                else:
                    current_action = self.get_goal_align_if(edge_i["edge_id"], current_action)

                # current_action = self.get_goal_align_if(edge_i["edge_id"], current_action)
                self.get_logger().info("Edge Action Manager: Processing edge {} action {}  goal {}".format(edge_i["edge_id"], current_action, edge_i["action"]))
                intermediate_goal = self.construct_goal(copy.deepcopy(edge_i["goal"]), dest_i, origin_i)
                self.get_logger().info("Edge Action Manager: Constructing intermediate goal")
                if (previous_action != current_action):
                    segment = segment + 1

                if segment in poses:
                    poses[segment].append(intermediate_goal)
                    actions[segment].append(current_action)
                    edge_ids[segment].append(edge_i["edge_id"])
                else:
                    poses[segment] = []
                    actions[segment] = []
                    edge_ids[segment] = []
                    poses[segment].append(intermediate_goal)
                    actions[segment].append(current_action)
                    edge_ids[segment].append(edge_i["edge_id"])

                previous_action = current_action
            self.destination_node = destination_node[-1]
            self.action_msgs, self.control_server_configs = self.construct_navigate_through_poses_goal(poses, actions, edge_ids)

        return True 
    
    def get_goal_align_if(self, edge_id, current_action, next_edge_id=None):
        edges = edge_id.split("_")
        if next_edge_id is not None:
            next_edge_ids = next_edge_id.split("_")
            if len(next_edge_ids) == 2:
                next_goal_stage = next_edge_ids[1].split("-")
                if len(next_goal_stage) == 2:
                    if (next_goal_stage[1] in self.ACTIONS.GOAL_ALIGN_INDEX) or (next_goal_stage[1] not in self.ACTIONS.GOAL_ALIGN_GOAL):
                        return current_action
                elif len(next_goal_stage) == 1:
                    if(current_action == self.ACTIONS.ROW_TRAVERSAL):
                        return current_action
        if len(edges) == 2:
            goal = edges[1]
            goal_stage = goal.split("-")
            if len(goal_stage) == 2:
                if goal_stage[1] in self.ACTIONS.GOAL_ALIGN_INDEX:
                    return self.ACTIONS.GOAL_ALIGN 
        return current_action

    def set_nav_client(self):
        self.action_server_name = self.get_action_server_name(self.action_name)
        self.get_logger().info("Edge Action Manager: Importing {} from {}".format(self.action_name, self.package))
        try:
            action = _import(self.package, self.action_name)
        except Exception as e:
            self.get_logger().error("Edge Action Manager: Still does not support actin type: {}".format(self.action_name))
            return False 

        self.get_logger().info("Edge Action Manager: Creating a {} client".format(self.action_server_name))
        self.action = action 
        self.client = ActionClient(self, self.action, self.action_server_name, callback_group=self.nav2_client_callback_group)
    
    def feedback_callback(self, feedback_msg):
        self.nav_client_feedback = feedback_msg
        # self.get_logger().info("Distance to goal: {} ".format(self.nav_client_feedback.distance_remaining))
        return 
    
    def preempt_feedback_callback(self, feedback_msg):
        self.nav_client_preempt_feedback = feedback_msg
        self.get_logger().info("preempt: {} ".format(self.nav_client_preempt_feedback))
        return 

    def get_action_server_name(self, action_name):
        action_topic = ""
        for char in action_name:
            if char.isupper():
                if(len(action_topic)>1):
                    action_topic += "_"
                    action_topic += char.lower()
                else:
                    action_topic += char.lower()
            else:
                action_topic += char
        return action_topic  
    
    def get_state(self,):
        return self.action_status 
        
    def preempt(self, timeout_secs=2.0):
        if self.client is not None:
            if not self.client.server_is_ready():
                self.get_logger().info("Edge Action Manager: Waiting for the action server  {}...".format(self.action_server_name))
                self.client.wait_for_server(timeout_sec=2)
            if not self.client.server_is_ready():
                self.get_logger().info("Edge Action Manager: action server  {} not responding ... can not perform any action".format(self.action_server_name))
                return True
            if self.goal_handle is None:
                self.get_logger().info("Edge Action Manager: There is no goal to stop it is already cancelled with status {}".format(self.action_status))
                return True
            
            counter = 0
            
            try: 
                # rclpy.spin_until_future_complete(self, cancel_future, feedback=self.preempt_feedback_callback)
                cancel_future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                # self.internal_executor.spin_until_future_complete(cancel_future)
                self.get_logger().info("Edge Action Manager: Waiting till terminating the current preemption")
                self.action_status = 5
                self.get_logger().info("Edge Action Manager: The goal cancel error code {} ".format(self.get_goal_cancel_error_msg(cancel_future.result().return_code)))
                self.robot_current_status = self.ACTIONS.ROBOT_STATUS_NATURAL_STATE
                self.publish_robot_current_status_msg(self.ACTIONS.NAVIGATE_THROUGH_POSES, self.robot_current_status)
                return True 
            except Exception as e:
                # counter = counter +1
                # if(counter < 3):
                # self.goal_handle = None
                self.get_logger().error("Edge Action Manager: Something wrong with Nav2 Control server {} while preempting {}".format(e, self.action_server_name))
                return True 
        
    def construct_goal(self, goal_args, destination_node, origin_node):
        paths = self.dt.get_paths_from_nested_dict(goal_args)
        for item in paths:
            value = item["value"]
            if isinstance(value, str):
                if value.startswith("$"):
                    _property = self.dt.getFromDict(destination_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
                    
                elif value.startswith("+") and origin_node is not None:
                    _property = self.dt.getFromDict(origin_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
        goal = goal_args
        # self.get_logger().info("  goal {} ".format(goal))
        return goal 
    
    def construct_navigate_to_pose_goal(self, goal):
            action_msgs = self.get_navigate_to_pose_goal(goal["target_pose"]["header"]["frame_id"], goal["target_pose"]["pose"])
            return action_msgs
    
    def get_navigate_to_pose_goal(self, frame_id, goal):
        nav_goal = NavigateToPose.Goal()
        target_pose = self.create_pose_stamped_msg(frame_id, goal)
        nav_goal.pose = target_pose
        action_msgs = []
        action_msg = TopoNavEdgeActionMsg()
        self.get_logger().info("Edge Action Manager: Action: {}".format(self.ACTIONS.NAVIGATE_TO_POSE))
        if(self.ACTIONS.NAVIGATE_TO_POSE in self.bt_trees):
            nav_goal.behavior_tree = self.bt_trees[self.ACTIONS.NAVIGATE_TO_POSE]
            self.get_logger().info("Edge Action Manager: Bt_tree: {}".format(self.bt_trees[self.ACTIONS.NAVIGATE_TO_POSE]))

        action_msg.setAction(self.ACTIONS.NAVIGATE_TO_POSE)
        action_msg.setNavGoal(nav_goal)
        action_msgs.append(action_msg)
        return action_msgs
    
    def construct_navigate_through_poses_goal(self, goals, actions, edge_ids):
        action_msgs, control_server_configs = self.get_navigate_through_poses_goal(goals, actions, edge_ids)
        return action_msgs, control_server_configs 
    
    def check_edges_area_same(self, side_edges):
        edge_poses = []
        if(len(side_edges) >= 2):
            for key, val in side_edges.items():
                for pose in val:
                    edge_poses.append(np.array([pose.pose.position.x, pose.pose.position.y]))
                if(len(edge_poses) == 2):
                    if(np.linalg.norm(edge_poses[0]-edge_poses[1]) < 0.001):
                        return True 
        return False
    
    def check_target_is_same(self, node1, node2):
        target1 = np.array([node1["pose"]["position"]["x"], node1["pose"]["position"]["y"]])
        target2 = np.array([node2["pose"]["position"]["x"], node1["pose"]["position"]["y"]])
        if(np.linalg.norm(target1-target2) < 0.001):
            return True 
        return False

    def two_smallest_indices(self, lst):
        if len(lst) < 1:
            return []
        if len(lst) < 2:
            return [0]
        lst_copy = lst[:]
        min_index1 = lst_copy.index(min(lst_copy))
        lst_copy[min_index1] = float('inf')
        min_index2 = lst_copy.index(min(lst_copy))
        return [min_index1, min_index2]
    
    def extract_number(self, s):
        return float(s.split('-')[0][1:])
                
    def get_navigate_through_poses_goal(self, poses, actions, edge_ids):
        # goals = []
        # actions_execute = []
        control_server_configs = {}
        # edge_actions = []
        action_msgs = []
        self.is_row_boundary_published = False
        edge_action_is_valid = True
        for seg_i, nodes  in poses.items():
            nav_goal = NavigateThroughPoses.Goal()
            # print("===============nodes: {}".format(nodes))     
            for pose in nodes:
                target_pose = self.create_pose_stamped_msg(pose["target_pose"]["header"]["frame_id"], pose["target_pose"]["pose"])
                nav_goal.poses.append(target_pose)

            action = actions[seg_i][0]
            edge_id = edge_ids[seg_i][0]
            action_msg = TopoNavEdgeActionMsg()

            self.get_logger().info("Edge Action Manager: seg: {}, action: {}, edge id: {} ".format(seg_i, action, edge_id))
            if( (action == self.ACTIONS.ROW_TRAVERSAL) and (self.in_row_operation == False)):
                controller_plugin = self.ACTIONS.bt_tree_with_control_server_config[self.ACTIONS.ROW_TRAVERSAL]
                control_server_configs[self.ACTIONS.ROW_TRAVERSAL] = self.ACTIONS.planner_with_goal_checker_config[controller_plugin]
                if(self.ACTIONS.ROW_TRAVERSAL in self.bt_trees):
                    nav_goal.behavior_tree = self.bt_trees[self.ACTIONS.ROW_TRAVERSAL]
                edge_action_is_valid = True
                
            if((action == self.ACTIONS.NAVIGATE_TO_POSE) or (action == self.ACTIONS.ROW_CHANGE)):
                controller_plugin = self.ACTIONS.bt_tree_with_control_server_config[self.ACTIONS.NAVIGATE_TO_POSE]
                control_server_configs[self.ACTIONS.NAVIGATE_TO_POSE] = self.ACTIONS.planner_with_goal_checker_config[controller_plugin] 
                if(self.ACTIONS.NAVIGATE_TO_POSE in self.bt_trees):
                    nav_goal.behavior_tree = self.bt_trees[self.ACTIONS.NAVIGATE_TO_POSE]
                edge_action_is_valid = True
                    
            if(action == self.ACTIONS.GOAL_ALIGN):
                controller_plugin = self.ACTIONS.bt_tree_with_control_server_config[self.ACTIONS.GOAL_ALIGN]
                control_server_configs[self.ACTIONS.GOAL_ALIGN] = self.ACTIONS.planner_with_goal_checker_config[controller_plugin]
                if(self.ACTIONS.GOAL_ALIGN in self.bt_trees):
                    nav_goal.behavior_tree = self.bt_trees[self.ACTIONS.GOAL_ALIGN]
                edge_action_is_valid = True

            if(action == self.ACTIONS.ROW_TRAVERSAL and self.in_row_operation == True and (len(nodes) > 0)):
                # print("===========edge_id {} edge_ids {}".format(edge_id, edge_ids))
                self.target_row_edge_id = edge_id.split("_")[0]
                tag_id = self.target_row_edge_id.split("-")[1]
                self.target_row_edge_id = self.target_row_edge_id[:-1] + self.ACTIONS.ROW_START_INDEX
                tag_id = tag_id[:-1] + self.ACTIONS.ROW_START_INDEX

                self.get_logger().info("Edge Action Manager: Action in_row_operation ")
                self.get_logger().info("Edge Action Manager: Edge id and tag id  {} {}".format(self.target_row_edge_id, tag_id))
                cen =  self.route_search.get_node_from_tmap2(self.target_row_edge_id)
                children = self.route_search.get_connected_nodes_tmap2(cen)
                selected_row_edge_nodes = {}
                for next_edge in children:
                    if(next_edge.startswith(self.ACTIONS.OUTSIDE_EDGE_START_INDEX)):
                        upper_nodes =  self.route_search.get_node_from_tmap2(next_edge)["node"]["edges"]
                        for edges_all in upper_nodes:
                            if((self.ACTIONS.GOAL_ALIGN_INDEX[0] in edges_all["node"]) 
                                                and (self.target_row_edge_id not in edges_all["node"])):
                                
                                targte_pose = self.route_search.get_node_from_tmap2(edges_all["node"])["node"]["pose"]
                                targte_pose_x_y = np.array([targte_pose["position"]["x"], targte_pose["position"]["y"]])
                                selected_row_edge_nodes[edges_all["node"]] = (targte_pose, targte_pose_x_y)
                
                center_pose = self.route_search.get_node_from_tmap2(self.target_row_edge_id)["node"]["pose"]["position"]
                center_pose = np.array([center_pose["x"], center_pose["y"]])
                distance_vector = []
                distance_with_edge_ids = {}
                index_dis = 0
                for the_key, the_value in selected_row_edge_nodes.items():  
                    distance_vector.append(np.linalg.norm(center_pose- the_value[1]))
                    distance_with_edge_ids[index_dis] = the_key
                    index_dis += 1
                # print("index distance .....", distance_vector)    
                min_indices = self.two_smallest_indices(distance_vector)
                children = []
                for index in min_indices:
                    children.append(distance_with_edge_ids[index])
                    # print("----------Index.... {} ".format(distance_with_edge_ids[index]))
                
                # Sort the list based on the extracted number in descending order
                children = sorted(children, key=self.extract_number, reverse=True)

                # self.get_logger().info("Edge Action Manager: Next Egdges edges {}".format(selected_row_edge_nodes))  
                        
                self.get_logger().info("Edge Action Manager: Children edges {}".format(children)) 
                
                target_pose_frame_id = nodes[0]["target_pose"]["header"]["frame_id"]
                last_goal = nodes[-1]

                if(len(nodes) > 1):
                    if(self.check_target_is_same(cen["node"], last_goal["target_pose"])):
                        last_goal = nodes[0]

                selected_last_node = last_goal
                if(len(nodes) == 1):
                    edges = edge_id.split("_")
                    if(len(edges) == 2):
                        edge_0, edge_1 = edges[0], edges[1]
                        tag_0, tag_1 = edge_0.split("-")[1], edge_1.split("-")[1]
                        if(tag_1 == tag_id):
                                selected_last_node["target_pose"]["pose"] = self.route_search.get_node_from_tmap2(edge_0)["node"]["pose"]
                        elif(tag_0 == tag_id):
                                selected_last_node["target_pose"]["pose"] = self.route_search.get_node_from_tmap2(edge_1)["node"]["pose"]
                    elif(len(selected_last_node) == 0 and self.current_node is not None):
                        selected_last_node["target_pose"]["pose"] = self.route_search.get_node_from_tmap2(self.current_node)["node"]["pose"]
                    else:
                        self.get_logger().error("Cound not find bounday edge...") 
                        
                # print("===============nodes {}".format(nodes)) 
                self.selected_edges = {}    
                for child in children:
                    # self.get_logger().info("=============children h selected ===== {} {} ".format(child, tag_id))
                    if tag_id in child:
                        # self.get_logger().info("=============children h selected ===== {}".format(child))
                        child_node = self.route_search.get_node_from_tmap2(child)
                        # self.get_logger().info("=============after children===== {}".format(child))
                        # distance = self.route_search.get_distance_to_node_tmap2(cen, child_node)
                        side_intermediate_pose = self.get_intermediate_pose(cen, child_node, target_pose_frame_id)
                        side_last_intermediate_pose = self.get_last_intermediate_pose(side_intermediate_pose, cen, selected_last_node)
                        self.selected_edges[child] = [side_intermediate_pose, side_last_intermediate_pose]
                        # self.get_logger().info("=============children i selected ===== {} {} ".format(side_intermediate_pose, side_last_intermediate_pose))

                if(len(self.selected_edges) == 1):
                    self.selected_edges["side_wall"] = self.get_intermediate_poses_interpolated(self.selected_edges, cen, selected_last_node)

                if(self.check_edges_area_same(self.selected_edges)):
                    edge_action_is_valid = True
                    self.get_logger().error("Edge Action Manager: Bounday edges are same")
                else:
                    edge_action_is_valid = True   
                if edge_action_is_valid:
                    action_msg.setSideEdges(self.selected_edges, target_pose_frame_id) 
                    if self.is_row_boundary_published == False:
                        boundary_info = action_msg.getBoundary()
                        self.boundary_publisher.publish(boundary_info)
                        self.is_row_boundary_published = True  
                    action = self.ACTIONS.ROW_OPERATION

            self.get_logger().info("Edge Action Manager:  Action {}  Bt_tree : {}".format(action, nav_goal.behavior_tree))
            if edge_action_is_valid:
                action_msg.setAction(action)
                action_msg.setNavGoal(nav_goal)
                action_msgs.append(action_msg)
                
        return action_msgs, control_server_configs
    
    def get_intermediate_poses_interpolated(self, side_poses, center_pose, last_pose):
        
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.frame_id = last_pose["target_pose"]["header"]["frame_id"]

        side_edge_key = list(side_poses)[0]
        side_front =  side_poses[side_edge_key][0].pose.position
        side_back = side_poses[side_edge_key][1].pose.position
        center_pose = center_pose["node"]["pose"]
        last_pose = last_pose["target_pose"]['pose']

        side_front_opposite = PoseStamped()
        side_front_opposite.header = header 
        side_front_opposite.pose.position.x = center_pose["position"]["x"] + -1.0*(side_front.x-center_pose["position"]["x"])
        side_front_opposite.pose.position.y = center_pose["position"]["y"] + -1.0*(side_front.y-center_pose["position"]["y"])
        side_front_opposite.pose.position.z = center_pose["position"]["z"]
        side_front_opposite.pose.orientation.w = center_pose["orientation"]["w"]
        side_front_opposite.pose.orientation.x = center_pose["orientation"]["x"]
        side_front_opposite.pose.orientation.y = center_pose["orientation"]["y"]
        side_front_opposite.pose.orientation.z = center_pose["orientation"]["z"]

        side_back_opposite = PoseStamped()
        side_back_opposite.header = header 
        side_back_opposite.pose.position.x = last_pose["position"]["x"] + -1.0*(side_back.x-last_pose["position"]["x"])
        side_back_opposite.pose.position.y = last_pose["position"]["y"] + -1.0*(side_back.y-last_pose["position"]["y"])
        side_back_opposite.pose.position.z = last_pose["position"]["z"]
        side_back_opposite.pose.orientation.w = last_pose["orientation"]["w"]
        side_back_opposite.pose.orientation.x = last_pose["orientation"]["x"]
        side_back_opposite.pose.orientation.y = last_pose["orientation"]["y"]
        side_back_opposite.pose.orientation.z = last_pose["orientation"]["z"]

        return [side_front_opposite, side_back_opposite]

    def get_last_intermediate_pose(self, side_intermediate_pose, center_pose, last_pose):
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.frame_id = last_pose["target_pose"]["header"]["frame_id"]
        target_pose = PoseStamped()
        target_pose.header = header  
        intermediate_pose = side_intermediate_pose.pose.position
        center_pose = center_pose["node"]["pose"]
        last_pose = last_pose["target_pose"]['pose']
        target_pose.pose.position.x = last_pose["position"]["x"] + (intermediate_pose.x - center_pose["position"]["x"])
        target_pose.pose.position.y = last_pose["position"]["y"] + (intermediate_pose.y - center_pose["position"]["y"])
        target_pose.pose.position.z = center_pose["position"]["z"]
        target_pose.pose.orientation.w = center_pose["orientation"]["w"]
        target_pose.pose.orientation.x = center_pose["orientation"]["x"]
        target_pose.pose.orientation.y = center_pose["orientation"]["y"]
        target_pose.pose.orientation.z = center_pose["orientation"]["z"]
        return target_pose        


    def get_intermediate_pose(self, pose1, pose2, header_frame_id, dev_factor=1.0):
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.frame_id = header_frame_id
        target_pose = PoseStamped()
        target_pose.header = header  
        pose1 =  pose1["node"]["pose"]
        pose2 =  pose2["node"]["pose"]
        if(dev_factor > 1.0):
            target_pose.pose.position.x = (pose1["position"]["x"] + pose2["position"]["x"])/2.0
            target_pose.pose.position.y = (pose1["position"]["y"] + pose2["position"]["y"])/2.0 
        else:
            target_pose.pose.position.x = pose2["position"]["x"]
            target_pose.pose.position.y = pose2["position"]["y"]
            
        target_pose.pose.position.z = pose1["position"]["z"]
        target_pose.pose.orientation.w = pose1["orientation"]["w"]
        target_pose.pose.orientation.x = pose1["orientation"]["x"]
        target_pose.pose.orientation.y = pose1["orientation"]["y"]
        target_pose.pose.orientation.z = pose1["orientation"]["z"]
        return target_pose 

    def create_pose_stamped_msg(self, frame_id, goal):
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        target_pose = PoseStamped()
        target_pose.header = header 
        desired_target_pose = goal 
        target_pose.pose.position.x = desired_target_pose["position"]["x"]
        target_pose.pose.position.y = desired_target_pose["position"]["y"]
        target_pose.pose.position.z = desired_target_pose["position"]["z"]
        target_pose.pose.orientation.w = desired_target_pose["orientation"]["w"]
        target_pose.pose.orientation.x = desired_target_pose["orientation"]["x"]
        target_pose.pose.orientation.y = desired_target_pose["orientation"]["y"]
        target_pose.pose.orientation.z = desired_target_pose["orientation"]["z"]
        return target_pose 

    def crete_pose_stamped_msg_from_position(self, frame_id, goal):
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        target_pose = PoseStamped()
        target_pose.header = header 
        target_pose.pose = goal.pose
        return target_pose 

    def get_result(self, ):
        return self.goal_resposne

    def execute_row_operation_one_step(self, next_goal, target_pose_frame_id):
        target_goal = NavigateThroughPoses.Goal()
        if(self.ACTIONS.ROW_TRAVERSAL in self.bt_trees): 
            target_goal.behavior_tree = self.bt_trees[self.ACTIONS.ROW_TRAVERSAL]
            self.get_logger().info("Edge Action Manager: Row traversal BT path {}".format(target_goal.behavior_tree))
        else:
            self.get_logger().info("Edge Action Manager: Row traversal BT path not found")
        target_pose = self.crete_pose_stamped_msg_from_position(target_pose_frame_id, next_goal)
        target_goal.poses.append(target_pose)
        
        controller_plugin = self.ACTIONS.bt_tree_with_control_server_config[self.ACTIONS.ROW_TRAVERSAL]
        
        control_server_config = self.ACTIONS.planner_with_goal_checker_config[controller_plugin] 
        self.get_logger().info(" Edge Action Manager: Control params {}".format(control_server_config))
        self.update_params_control_server.set_params(control_server_config)
                
        send_goal_future = self.client.send_goal_async(target_goal,  feedback_callback=self.feedback_callback)
        goal_accepted = self.send_goal_request(send_goal_future, self.ACTIONS.ROW_OPERATION)
        if(goal_accepted == False):
            return False 
        processed_goal = self.processing_goal_request(self.ACTIONS.ROW_OPERATION)
        return processed_goal

    def send_goal_request(self, send_goal_future, msg):
        while rclpy.ok():
            try:
                # rclpy.spin_once(self)
                # print("send_goal_future ", send_goal_future)
                rclpy.spin_once(self, executor=self.executor_nav_client, timeout_sec=0.5)
                # rclpy.spin_until_future_complete(self, send_goal_future, executor=self.executor_nav_client, timeout_sec=2.0)
                if send_goal_future.done():
                    self.goal_handle = send_goal_future.result()
                    break
            except Exception as e:
                self.get_logger().error("Edge Action Manager: Nav2 server got some unexpected errors : {} while executing  send_goal_request {}".format(e, msg))
                return False 
                  
        if not self.goal_handle.accepted:
            self.get_logger().error('Edge Action Manager: The goal rejected: {}'.format(msg))
            return False
        self.get_logger().info('Edge Action Manager: The goal accepted for {}'.format(msg))
        return True 

    def processing_goal_request(self, target_action):
        if self.goal_handle is None:
            self.get_logger().error("Edge Action Manager: Something wrong with the goal request, there is no goal to process {}".format(self.action_status))
            return True
        self.goal_get_result_future = self.goal_handle.get_result_async()
        self.get_logger().info("Edge Action Manager: Waiting for {} action to complete".format(self.action_server_name))
        while rclpy.ok():
            try:
                # rclpy.spin_once(self)
                # print("goal_get_result_future ", self.goal_get_result_future)
                rclpy.spin_once(self, executor=self.executor_nav_client, timeout_sec=1.5)
                # rclpy.spin_until_future_complete(self, self.goal_get_result_future, executor=self.executor_nav_client, timeout_sec=2.0)
                if self.goal_get_result_future.done():
                    status = self.goal_get_result_future.result().status
                    self.action_status = status
                    self.get_logger().info("Edge Action Manager: Executing the action response with status {}".format(self.get_status_msg(self.action_status)))
                    self.current_action = self.action_name
                    self.goal_resposne = self.goal_get_result_future.result()
                    if((self.get_status_msg(self.action_status) == "STATUS_EXECUTING") or (self.get_status_msg(self.action_status) == "STATUS_SUCCEEDED")):
                        self.get_logger().info("Edge Action Manager: action is completed {}".format(target_action))
                        break  
                    if target_action in self.ACTIONS.ABORT_NOT_CONTINUE:
                            self.get_logger().info("Edge Action Manager: action is not completed {}".format(target_action))
                            return False 
                    else:
                        self.get_logger().info("Edge Action Manager: action is not completed {}. Executing next actions...".format(target_action))
                        break 
            except Exception as e:
                # self.goal_handle = Nosne 
                self.get_logger().error("Edge Action Manager: Nav2 server got some unexpected errors: {} while executing processing_goal_request {}".format(e, target_action))
                return False
        return True 

    def execute_row_operation(self,):
        target_goal = NavigateThroughPoses.Goal()
        if(self.ACTIONS.ROW_OPERATION in self.bt_trees): 
            target_goal.behavior_tree = self.bt_trees[self.ACTIONS.ROW_OPERATION]
            self.get_logger().info("Edge Action Manager: Row operation BT path {}".format(target_goal.behavior_tree))
        else:
            self.get_logger().error("Edge Action Manager: Row operation BT is not provided {}")
            return False
        send_goal_future = self.client.send_goal_async(target_goal)
        goal_accepted = self.send_goal_request(send_goal_future, self.ACTIONS.ROW_OPERATION)
        if(goal_accepted == False):
            return False 
        processed_goal = self.processing_goal_request(self.ACTIONS.ROW_OPERATION)
        return processed_goal
    
    def execute_row_recovery(self, intermediate_pose, target_pose_frame_id):
        target_goal = NavigateThroughPoses.Goal()
        if(self.ACTIONS.ROW_RECOVERY in self.bt_trees): 
            target_goal.behavior_tree = self.bt_trees[self.ACTIONS.ROW_RECOVERY]
            self.get_logger().info("Edge Action Manager: Row recovery BT path {}".format(target_goal.behavior_tree))
        else:
            self.get_logger().error("Edge Action Manager: Row recovery BT is not provided {}")
            return False
        target_pose = self.crete_pose_stamped_msg_from_position(target_pose_frame_id, intermediate_pose)
        target_goal.poses.append(target_pose)
        send_goal_future = self.client.send_goal_async(target_goal)
        goal_accepted = self.send_goal_request(send_goal_future, self.ACTIONS.ROW_RECOVERY)
        if(goal_accepted == False):
            return False 
        processed_goal = self.processing_goal_request(self.ACTIONS.ROW_RECOVERY)
        return processed_goal
    
    def publish_robot_current_status_msg(self, action, status):
        msg = String()
        msg.data = status
        self.robot_current_status_pub.publish(msg)
        if self.robot_current_behavior_pub is not None:
            from robot_behavior_msg.msg import RobotBehavior
            robot_behavir = RobotBehavior()
            robot_behavir.message = status
            robot_behavir.behavior_code = self.ACTIONS.getCodeForRobotCurrentStatus(status)
            self.robot_current_behavior_pub.publish(robot_behavir)

    def execute_row_operation_action(self, action_msg):
        nav_goal = action_msg.getNavGoal()
        boundary_info = action_msg.getBoundary()
        target_frame_id = action_msg.getTargetFrameId()
        self.boundary_publisher.publish(boundary_info)  
        self.is_row_boundary_published = False

        self.robot_current_status = self.ACTIONS.ROBOT_STATUS_NATURAL_STATE
        self.publish_robot_current_status_msg(self.ACTIONS.NAVIGATE_THROUGH_POSES, self.robot_current_status)
        self.in_row_inter_pose = nav_goal.poses

        if len(self.in_row_inter_pose) > 0:
            self.get_logger().info("Edge Action Manager: Executing action : {} ".format(self.ACTIONS.ROW_OPERATION))
            while rclpy.ok():
                try:
                    # rclpy.spin_once(self)
                    # print("send_goal_future self.current_robot_pose ", self.current_robot_pose)
                    rclpy.spin_once(self, executor=self.executor_nav_client, timeout_sec=0.5)
                    if(self.current_robot_pose is not None):
                        # self.get_logger().info("Robot current pose: {}".format(self.current_robot_pose))
                        break 
                except Exception as e:
                    self.allowed_row_operation = False
                    self.get_logger().error("Edge Action Manager: Could not set the current pose of the robot {}".format(e, self.ACTIONS.ROW_OPERATION))
                    return False
                
            if(len(self.in_row_inter_pose)==1):
                self.in_row_inter_pose = [self.current_robot_pose, self.in_row_inter_pose[0]]

            self.get_logger().info("Edge Action Manager:  ==========start executing in row operaitons============")
            inrow_opt = RowOperations(self.in_row_inter_pose, self.inrow_step_size, intermediate_dis=self.intermediate_dis) 
            robot_init_pose = self.current_robot_pose
            
            if inrow_opt.isPlanCalculated():
                while True:
                    robot_init_pose = self.current_robot_pose 
                    next_goal, intermediate_pose, get_to_goal = inrow_opt.getNextGoal(robot_init_pose)
                    self.robot_current_status = self.ACTIONS.ROBOT_STATUS_PREPARATION_STATE
                    if(self.current_node is not None):
                        node_id = self.current_node.split("-")
                        if (len(node_id) == 2):
                            node_id = node_id[1]
                            if(node_id.startswith(self.ACTIONS.ROW_COLUMN_START_INDEX)):
                                if (self.ACTIONS.ROW_START_INDEX not in node_id):
                                    self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_HARVESTING_STATE

                    self.publish_robot_current_status_msg(self.ACTIONS.ROW_OPERATION, self.robot_current_status)
                    done_operation = self.execute_row_operation()
                    self.get_logger().info("Edge Action Manager: done_operation {} ".format(done_operation))
                    # target_goal = NavigateThroughPoses.Goal()
                    self.get_logger().info("Edge Action Manager: Robot current pose: {},{}"
                                                .format(next_goal.pose.position.x, next_goal.pose.position.y))
                    if(self.intermediate_dis > 0.0):
                        self.robot_current_status = self.ACTIONS.ROBOT_STATUS_PREPARATION_STATE
                        self.publish_robot_current_status_msg(self.ACTIONS.ROW_RECOVERY, self.robot_current_status)
                        get_proper_alignment = self.execute_row_recovery(intermediate_pose, target_frame_id)
                        if(get_proper_alignment == False):
                            self.robot_current_status = self.ACTIONS.ROBOT_STATUS_DISABLE_STATE
                            self.publish_robot_current_status_msg(self.ACTIONS.ROW_RECOVERY, self.robot_current_status)
                            return False
                    self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_NAVIGATION_STATE
                    self.publish_robot_current_status_msg(self.ACTIONS.ROW_TRAVERSAL, self.robot_current_status)
                    step_moved = self.execute_row_operation_one_step(next_goal, target_frame_id)
                    if(step_moved == False):
                        if(self.intermediate_dis > 0.0):
                            intermediate_pose, _ = inrow_opt.getNextIntermediateGoal(self.current_robot_pose)
                            self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_RECOVERY_STATE
                            self.publish_robot_current_status_msg(self.ACTIONS.ROW_RECOVERY, self.robot_current_status)
                            get_proper_alignment = self.execute_row_recovery(intermediate_pose, target_frame_id)
                            if(get_proper_alignment == False):
                                self.robot_current_status = self.ACTIONS.ROBOT_STATUS_DISABLE_STATE
                                self.publish_robot_current_status_msg(self.ACTIONS.ROW_RECOVERY, self.robot_current_status)
                                return False
                            else:
                                self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_NAVIGATION_STATE
                                self.publish_robot_current_status_msg(self.ACTIONS.ROW_TRAVERSAL, self.robot_current_status)
                                step_moved = self.execute_row_operation_one_step(next_goal, target_frame_id)
                                if(step_moved == False):
                                    self.robot_current_status = self.ACTIONS.ROBOT_STATUS_DISABLE_STATE
                                    self.publish_robot_current_status_msg(self.ACTIONS.ROW_TRAVERSAL, self.robot_current_status)
                                    return False 
                        else:
                            self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_NAVIGATION_STATE
                            self.publish_robot_current_status_msg(self.ACTIONS.ROW_TRAVERSAL, self.robot_current_status)
                            step_moved = self.execute_row_operation_one_step(next_goal, target_frame_id)
                            if(step_moved == False):
                                self.robot_current_status = self.ACTIONS.ROBOT_STATUS_DISABLE_STATE
                                self.publish_robot_current_status_msg(self.ACTIONS.ROW_TRAVERSAL, self.robot_current_status)
                                return False
                    if(get_to_goal):
                        if step_moved:
                            self.robot_current_status = self.ACTIONS.ROBOT_STATUS_NATURAL_STATE
                            self.publish_robot_current_status_msg(self.ACTIONS.ROW_TRAVERSAL, self.robot_current_status)
                            self.get_logger().info("Edge Action Manager: Reach to the final goal {},{}"
                                        .format(next_goal.pose.position.x, next_goal.pose.position.y))
                            
                        else:
                            self.get_logger().error("Edge Action Manager: Can not reach to the final goal {},{}"
                                            .format(next_goal.pose.position.x, next_goal.pose.position.y))
                        break
        return True 

    def execute(self):     
        if not self.client.server_is_ready():
            self.get_logger().info("Edge Action Manager: Waiting for the action server  {}...".format(self.action_server_name))
            self.client.wait_for_server(timeout_sec=2)
        
        if not self.client.server_is_ready():
            self.get_logger().info("Edge Action Manager: action server  {} not responding ... can not perform any action".format(self.action_server_name))
            return 
        
        for action_msg in self.action_msgs:
            target_goal, target_action = action_msg.getNavGoal(), action_msg.getAction()
            self.get_logger().info("Edge Action Manager: Executing action : {} ".format(target_action))
            self.get_logger().info("===========================================Edge Action Manager: Executing action : {} ".format(self.control_server_configs))
            if target_action in self.control_server_configs:
                control_server_config = self.control_server_configs[target_action]
                self.update_params_control_server.set_params(control_server_config)
                        
            if (target_action == self.ACTIONS.ROW_OPERATION):
                row_operation_is_completed = self.execute_row_operation_action(action_msg)
                if(row_operation_is_completed == False):
                    return False
            else:
                self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_NAVIGATION_STATE
                self.publish_robot_current_status_msg(self.ACTIONS.NAVIGATE_THROUGH_POSES, self.robot_current_status)
                send_goal_future = self.client.send_goal_async(target_goal,  feedback_callback=self.feedback_callback)
                goal_accepted = self.send_goal_request(send_goal_future, target_action)
                if(goal_accepted == False):
                    self.robot_current_status = self.ACTIONS.ROBOT_STATUS_DISABLE_STATE
                    self.publish_robot_current_status_msg(self.ACTIONS.NAVIGATE_THROUGH_POSES, self.robot_current_status)
                    return False 
                processed_goal = self.processing_goal_request(target_action)
                if(processed_goal == False):
                    self.robot_current_status = self.ACTIONS.ROBOT_STATUS_DISABLE_STATE
                    self.publish_robot_current_status_msg(self.ACTIONS.NAVIGATE_THROUGH_POSES, self.robot_current_status)
                    return processed_goal
        self.robot_current_status = self.ACTIONS.ROBOT_STATUS_NATURAL_STATE
        self.publish_robot_current_status_msg(self.ACTIONS.NAVIGATE_THROUGH_POSES, self.robot_current_status)
        return True 
                            
