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
import math

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
        self.goal_response = None 
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
        self.is_inside_tunnel = False
        self.odom_sub = self.create_subscription(Odometry, '/odometry/global', self.odom_callback,
                                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_current_node_sub = self.create_subscription(String, 'closest_node', self.set_current_pose
                                , QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.robot_nav_area_sub = self.create_subscription(String, '/robot_navigation_area', self.nav_area_callback, 10)
       
        self.boundary_publisher = self.create_publisher(Path, '/boundary_checker', qos_profile=self.latching_qos)
        self.robot_current_status_pub = self.create_publisher(String, '/robot_operation_current_status', qos_profile=self.latching_qos)
        self.current_dest = self.create_publisher(String, '/topological_navigation/current_destination', qos_profile=self.latching_qos)
        self.target_edge_path_pub = self.create_publisher(Path, "/target_edge_path", qos_profile=self.latching_qos)
        self.center_node_pose_pub = self.create_publisher(PoseStamped, "/center_node/pose", qos_profile=self.latching_qos)

        self.robot_current_behavior_pub = None
        self.current_node = None 
        self.action_status = 0
        self.robot_current_status = self.ACTIONS.ROBOT_STATUS_AUTONOMOUS_NAVIGATION_STATE 
        self.executor_nav_client = SingleThreadedExecutor()


    def set_current_pose(self, msg):
        self.current_node = msg.data


    def odom_callback(self, msg):
        self.current_robot_pose = msg.pose


    def nav_area_callback(self, msg):
        self.nav_area = msg.data
        self.is_inside_tunnel = (self.nav_area == 'INSIDE_POLYTUNNEL')

    
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

    def _is_waypoint_name(self, name: str) -> bool:
        n = (name or "").lower()
        return ("waypoint" in n) or n.startswith("wp") or ("_wp" in n)

    def _is_row_node_name(self, name: str) -> bool:
        # Uses your existing constant that you already rely on elsewhere
        # (you used startswith(ROW_COLUMN_START_INDEX) in execute_row_operation_action)
        if not name:
            return False
        n = str(name)
        return n.split("-")[-1].startswith(self.ACTIONS.ROW_COLUMN_START_INDEX) and (not self._is_waypoint_name(n))

    def _yaw_from_quat(self, q) -> float:
        # q: dict with x,y,z,w
        x, y, z, w = q["x"], q["y"], q["z"], q["w"]
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def _set_pose_yaw(self, pose_stamped: PoseStamped, yaw: float):
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)

        
    def _adjust_orientations_for_next_wp(self, poses_dict):
        """
        Adjusts the orientation of each pose to face the next one in the sequence.
        The final pose's orientation is left unchanged.
        """
        # Check if there are enough poses to perform the calculation
        if len(poses_dict) < 2:
            self.get_logger().warn("[_adjust_orientations_for_next_wp] Not enough poses to adjust orientations. Returning original poses.")
            return poses_dict

        # 1. Create a flat list of the actual pose dictionaries.
        # Sorting by key ensures the poses are in the correct sequential order.
        flat_poses = [poses_dict[key][0] for key in sorted(poses_dict.keys())]

        # 2. Iterate up to the second-to-last pose to update its orientation
        for i in range(len(flat_poses) - 1):
            current_pos = flat_poses[i]['target_pose']['pose']['position']
            next_pos = flat_poses[i+1]['target_pose']['pose']['position']

            # Calculate the yaw angle to face the next pose
            delta_x = next_pos['x'] - current_pos['x']
            delta_y = next_pos['y'] - current_pos['y']
            yaw = math.atan2(delta_y, delta_x)

            # 3. Convert yaw to a quaternion and modify the orientation directly in the pose dictionary.
            orientation = flat_poses[i]['target_pose']['pose']['orientation']
            orientation['w'] = math.cos(yaw / 2.0)
            orientation['x'] = 0.0
            orientation['y'] = 0.0
            orientation['z'] = math.sin(yaw / 2.0)
            self.get_logger().info("[_adjust_orientations_for_next_wp] Adjusted orientation for pose {} to face next goal".format(i))

        return poses_dict
        
    def _process_and_segment_edges(self, edge, destination_node, origin_node, is_execpolicy):
        """
        Processes a list of edges, groups them into segments based on the policy.
        """
        self.get_logger().warn(f"[_process_and_segment_edges] - {'EXECUTE-POLICY' if is_execpolicy else 'GO-TO-NODE'}")
        
        poses, actions, edge_ids = {}, {}, {}
        segment = 0
        previous_action = ""
        total_edges = len(edge)

        for index in range(total_edges):
            edge_i, dest_i, origin_i = edge[index], destination_node[index], origin_node[index]
            edge_i = yaml.safe_load(json.dumps(edge_i))
            current_action_base = edge_i["action"]

            # Determine the effective current action (common logic)
            if index < (total_edges - 1):
                next_edge_id = edge[index + 1]["edge_id"]
                current_action = self.get_goal_align_if(edge_i["edge_id"], current_action_base, next_edge_id)
                self.get_logger().debug(f"[_process_and_segment_edges] - intermediate edge {edge_i['edge_id']} -> action {current_action}")
            else:
                current_action = self.get_goal_align_if(edge_i["edge_id"], current_action_base)
                self.get_logger().debug(f"[_process_and_segment_edges] - final edge {edge_i['edge_id']} -> action {current_action}")

            # Determine segment number based on the policy (the only differing logic)
            if not is_execpolicy:
                # Group by action: increment segment only when action changes
                if previous_action != current_action: segment += 1
            else:
                # One edge per segment
                segment = index

            # Construct and append data (common logic)
            self.get_logger().info(f"Processing edge {edge_i['edge_id']} for segment {segment}")
            intermediate_goal = self.construct_goal(copy.deepcopy(edge_i["goal"]), dest_i, origin_i)
            
            if segment not in poses:
                self.get_logger().debug(f"[_process_and_segment_edges] - Creating new segment {segment}")
                poses[segment], actions[segment], edge_ids[segment] = [], [], []
            
            poses[segment].append(intermediate_goal)
            actions[segment].append(current_action)
            edge_ids[segment].append(edge_i["edge_id"])

            previous_action = current_action
            
        return poses, actions, edge_ids
    
    
    def initialise(self, bt_trees, edge, destination_node, origin_node=None,
                   action_name=None, package="nav2_msgs.action", in_row_operation=False, is_execpolicy=False):

        self.bt_trees = bt_trees
        self.in_row_operation = in_row_operation
        if self.in_row_operation:
            try:
                from robot_behavior_msg.msg import RobotBehavior
                self.robot_current_behavior_pub = self.create_publisher(RobotBehavior, '/robot_current_behavior', qos_profile=self.latching_qos)
            except Exception as e:
                self.get_logger().error("robot_behavior_msg.msg.RobotBehavior message type is not defined")

        if action_name is not None:
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
            self.get_logger().info(f"Processing edge {self.edge['edge_id']}")
            self.get_logger().info("Constructing the goal")
            target_pose = self.construct_goal(copy.deepcopy(edge["goal"]), destination_node, origin_node)
            self.action_msgs = self.construct_navigate_to_pose_goal(target_pose)

        if self.action_name == self.ACTIONS.NAVIGATE_THROUGH_POSES:
            
            self.destination_node_str = {}
            for item in destination_node:
                name = item['node']['name']
                x_coord = item['node']['pose']['position']['x']
                y_coord = item['node']['pose']['position']['y']
                coordinates = (x_coord, y_coord)
                self.destination_node_str[coordinates] = name
            self.get_logger().warn(f"Destination nodes: {self.destination_node_str}")
                
            poses, actions, edge_ids = self._process_and_segment_edges(edge, destination_node, origin_node, is_execpolicy)
            if is_execpolicy: poses = self._adjust_orientations_for_next_wp(poses)
                        
            self.destination_node = destination_node[-1]
            self.action_msgs, self.control_server_configs = self.construct_navigate_through_poses_goal(poses, actions, edge_ids, is_execpolicy=is_execpolicy)

        return True
        
    
    def get_goal_align_if(self, edge_id, current_action, next_edge_id=None):
        edges = edge_id.split("_")
        if next_edge_id is not None:
            next_edge_ids = next_edge_id.split("_")
            if len(next_edge_ids) == 2:
                next_goal_stage = next_edge_ids[1].split("-")
                if len(next_goal_stage) == 2:
                    if (next_goal_stage[1] in self.ACTIONS.GOAL_ALIGN_INDEX) or \
                    (next_goal_stage[1] not in self.ACTIONS.GOAL_ALIGN_GOAL):
                        return current_action
                elif len(next_goal_stage) == 1:
                    if current_action == self.ACTIONS.ROW_TRAVERSAL:
                        return current_action
        if len(edges) == 2:
            goal = edges[1]
            goal_stage = goal.split("-")
            if len(goal_stage) == 2:
                if goal_stage[1] in self.ACTIONS.GOAL_ALIGN_INDEX and not self.is_inside_tunnel:
                    return self.ACTIONS.GOAL_ALIGN

        return current_action


    def set_nav_client(self):
        self.action_server_name = self.get_action_server_name(self.action_name)
        self.get_logger().info("Importing {} from {}".format(self.action_name, self.package))
        try:
            action = _import(self.package, self.action_name)
        except Exception as e:
            self.get_logger().error("Still does not support action type: {}".format(self.action_name))
            return False 

        self.get_logger().info("Creating a {} client".format(self.action_server_name))
        self.action = action 
        self.client = ActionClient(self, self.action, self.action_server_name, callback_group=self.nav2_client_callback_group)
    
    
    def feedback_callback(self, feedback_msg):
        self.nav_client_feedback = feedback_msg
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
                self.get_logger().info("Waiting for the action server  {}...".format(self.action_server_name))
                self.client.wait_for_server(timeout_sec=2)
            if not self.client.server_is_ready():
                self.get_logger().info("action server  {} not responding ... can not perform any action".format(self.action_server_name))
                return True
            if self.goal_handle is None:
                self.get_logger().info("There is no goal to stop it is already cancelled with status {}".format(self.action_status))
                return True
            
            counter = 0
            
            try: 
                cancel_future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                self.get_logger().info("Waiting till terminating the current preemption")
                self.action_status = 5
                self.get_logger().info("The goal cancel error code {} ".format(self.get_goal_cancel_error_msg(cancel_future.result().return_code)))
                self.robot_current_status = self.ACTIONS.ROBOT_STATUS_NATURAL_STATE
                self.publish_robot_current_status_msg(self.ACTIONS.NAVIGATE_THROUGH_POSES, self.robot_current_status)
                return True 
            except Exception as e:
                self.get_logger().error("Something wrong with Nav2 Control server {} while preempting {}".format(e, self.action_server_name))
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
        self.get_logger().info("Action: {}".format(self.ACTIONS.NAVIGATE_TO_POSE))
        if(self.ACTIONS.NAVIGATE_TO_POSE in self.bt_trees):
            nav_goal.behavior_tree = self.bt_trees[self.ACTIONS.NAVIGATE_TO_POSE]
            self.get_logger().info("Bt_tree: {}".format(self.bt_trees[self.ACTIONS.NAVIGATE_TO_POSE]))
        
        action_msg.setAction(self.ACTIONS.NAVIGATE_TO_POSE)
        action_msg.setNavGoal(nav_goal)
        action_msgs.append(action_msg)
        return action_msgs


    def construct_navigate_through_poses_goal(self, goals, actions, edge_ids, is_execpolicy=False):
        action_msgs, control_server_configs = self.get_navigate_through_poses_goal(goals, actions, edge_ids, is_execpolicy=is_execpolicy)
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
        target2 = np.array([node2["pose"]["position"]["x"], node2["pose"]["position"]["y"]])
        return np.linalg.norm(target1 - target2) < 0.001

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
                
    def publish_target_edges_as_path(self, selected_edges_dict):
        """
        Convert selected edges dictionary to a ROS Path and publish.
        Safely handles placeholder values like '$node.pose' by substituting
        with actual node pose from selected_edges_dict['node']['pose'].
        """
        path_msg = Path()
        node_info = selected_edges_dict.get("node", {})
        node_pose = node_info.get("pose", {})
        parent_frame = node_info.get("parent_frame", "map")  # fallback to 'map'

        path_msg.header.frame_id = parent_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if not node_pose:
            self.get_logger().warn("Node pose missing, cannot publish path")
            return

        valid_poses_count = 0

        for edge in node_info.get("edges", []):
            target_pose_data = edge.get("goal", {}).get("target_pose", {})

            # Check if pose is placeholder
            if target_pose_data.get("pose") == "$node.pose":
                pose_source = node_pose
            elif isinstance(target_pose_data.get("pose"), dict):
                pose_source = target_pose_data["pose"]
            else:
                self.get_logger().warn(
                    f"Skipping edge {edge.get('edge_id')} due to invalid target_pose"
                )
                continue

            # Create PoseStamped
            try:
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = parent_frame
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position.x = pose_source["position"]["x"]
                pose_msg.pose.position.y = pose_source["position"]["y"]
                pose_msg.pose.position.z = pose_source["position"]["z"]
                pose_msg.pose.orientation.x = pose_source["orientation"]["x"]
                pose_msg.pose.orientation.y = pose_source["orientation"]["y"]
                pose_msg.pose.orientation.z = pose_source["orientation"]["z"]
                pose_msg.pose.orientation.w = pose_source["orientation"]["w"]
            except KeyError as e:
                self.get_logger().warn(
                    f"Skipping edge {edge.get('edge_id')} due to missing key: {e}"
                )
                continue

            path_msg.poses.append(pose_msg)
            valid_poses_count += 1

        self.get_logger().info(f"Publishing Path with {valid_poses_count} poses")
        self.target_edge_path_pub.publish(path_msg)

    def _publish_empty_boundary(self, frame_id: str):
        """Publish an empty boundary path."""
        empty = Path()
        empty.header.frame_id = frame_id
        empty.header.stamp = self.get_clock().now().to_msg()
        self.boundary_publisher.publish(empty)

    def _get_row_center_node(self, edge_id: str):
        """
        Parse edge_id to find the row center node.
        Returns: (center_node, tag_id, target_row_edge_id) or (None, None, None) on failure.
        """
        try:
            target_row_edge_id_raw = edge_id.split("_")[0]
            tag_id = target_row_edge_id_raw.split("-")[1]
            # print trget_row_edge_id_raw, tag_id
            self.get_logger().info(f"[_get_row_center_node] Parsed edge_id='{edge_id}' to target_row_edge_id_raw='{target_row_edge_id_raw}', tag_id='{tag_id}'")
            # Force ROW_START_INDEX by replacing the last character
            target_row_edge_id = target_row_edge_id_raw[:-1] + self.ACTIONS.ROW_START_INDEX
            tag_id = tag_id[:-1] + self.ACTIONS.ROW_START_INDEX
            #print target_row_edge_id, tag_id
            self.get_logger().info(f"[_get_row_center_node] Adjusted to target_row_edge_id='{target_row_edge_id}', tag_id='{tag_id}'")
        except Exception as e:
            self.get_logger().error(f"[_get_row_center_node] Failed to parse edge_id='{edge_id}': {e}")
            return None, None, None

        cen = self.route_search.get_node_from_tmap2(target_row_edge_id)
        if not cen or "node" not in cen or "pose" not in cen["node"]:
            self.get_logger().error(f"[_get_row_center_node] Could not resolve '{target_row_edge_id}'")
            return None, None, None

        return cen, tag_id, target_row_edge_id

    def _collect_boundary_candidates(self, cen, target_row_edge_id: str):
        """
        Collect candidate boundary nodes from connected WayPoint nodes.
        Returns: dict of node_id -> (pose_dict, xy_np)
        """
        candidates = {}
        children = self.route_search.get_connected_nodes_tmap2(cen) or []

        for next_edge in children:
            if not next_edge.startswith(self.ACTIONS.OUTSIDE_EDGE_START_INDEX):
                continue

            next_edge_node = self.route_search.get_node_from_tmap2(next_edge)
            if not next_edge_node or "node" not in next_edge_node:
                continue

            for edge_info in next_edge_node["node"].get("edges", []):
                node_id = edge_info.get("node", "")
                if not node_id or not isinstance(node_id, str):
                    continue
                # Must contain GOAL_ALIGN_INDEX and not be the target row itself
                if self.ACTIONS.GOAL_ALIGN_INDEX[0] not in node_id:
                    continue
                if target_row_edge_id in node_id:
                    continue
                if self._is_waypoint_name(node_id):
                    continue

                node_obj = self.route_search.get_node_from_tmap2(node_id)
                if not node_obj or "node" not in node_obj or "pose" not in node_obj["node"]:
                    continue

                pose = node_obj["node"]["pose"]
                xy = np.array([pose["position"]["x"], pose["position"]["y"]], dtype=float)
                candidates[node_id] = (pose, xy)

        return candidates

    def _select_boundary_nodes(self, candidates, center_xy, row_dir, tag_id):
        """
        Select left and right boundary nodes relative to the row direction.
        Returns: list of selected node_ids (up to 2).
        """
        scored = []  # (side_sign, dist, node_id)

        for node_id, (_pose, txy) in candidates.items():
            if tag_id not in node_id:
                continue

            v = txy - center_xy
            dist = float(np.linalg.norm(v))
            if dist < 1e-6:
                continue

            # Reject nodes too aligned with row direction (want perpendicular/lateral nodes)
            cosang = abs(float(np.dot(row_dir, v) / dist))
            if cosang > 0.6:
                continue

            # Determine which side (left=positive, right=negative)
            crossz = float(row_dir[0] * v[1] - row_dir[1] * v[0])
            side = 1.0 if crossz > 0.0 else -1.0
            scored.append((side, dist, node_id))

        if not scored:
            return []

        # Pick closest on each side
        left = min((c for c in scored if c[0] > 0.0), default=None, key=lambda x: x[1])
        right = min((c for c in scored if c[0] < 0.0), default=None, key=lambda x: x[1])

        picked = []
        if left:
            picked.append(left[2])
        if right:
            picked.append(right[2])

        # Fallback: fill up to 2 from remaining candidates
        if len(picked) < 2:
            for _, _, nid in sorted(scored, key=lambda x: x[1]):
                if nid not in picked:
                    picked.append(nid)
                if len(picked) == 2:
                    break

        return picked

    def _select_last_row_goal(self, nodes):
        """Select the last goal that is a ROW node (not a waypoint)."""
        selected = None
        for g in nodes:
            try:
                p = g["target_pose"]["pose"]["position"]
                xy = (p["x"], p["y"])
                name = getattr(self, "destination_node_str", {}).get(xy)
                if name and self._is_row_node_name(name):
                    selected = g
            except Exception:
                continue
        return selected if selected else nodes[-1]

    def _handle_row_operation(self, nodes, edge_id, action_msg):
        """
        Build row boundaries and return (action, action_msg).
        Simplified logic for selecting boundary nodes.
        """
        # Validate inputs
        if not nodes:
            self.get_logger().error("[_handle_row_operation] nodes list is empty")
            self._publish_empty_boundary("map")
            action_msg.setSideEdges({}, "map")
            return self.ACTIONS.ROW_OPERATION, action_msg

        frame_id = nodes[0]["target_pose"]["header"].get("frame_id", "map")

        if not edge_id:
            self.get_logger().error("[_handle_row_operation] edge_id is empty")
            self._publish_empty_boundary(frame_id)
            action_msg.setSideEdges({}, frame_id)
            return self.ACTIONS.ROW_OPERATION, action_msg

        # Get row center node
        cen, tag_id, target_row_edge_id = self._get_row_center_node(edge_id)
        if not cen:
            self._publish_empty_boundary(frame_id)
            action_msg.setSideEdges({}, frame_id)
            return self.ACTIONS.ROW_OPERATION, action_msg

        self.get_logger().info(f"[_handle_row_operation] center={target_row_edge_id}, tag={tag_id}")
        
        # get target_row_edge_id pose and orientation
        target_row_edge_node = self.route_search.get_node_from_tmap2(target_row_edge_id)
        if target_row_edge_node and "node" in target_row_edge_node:
            target_row_edge_pose = target_row_edge_node["node"]["pose"]
            self.get_logger().info(f"Target Row Edge Pose: {target_row_edge_pose}")
        else:
            self.get_logger().warn(f"[_handle_row_operation] Could not find node for target_row_edge_id: {target_row_edge_id}")

        try:
            # print out cen info
            # self.get_logger().info(f"Center Node Info: {cen}")
            # get target edges from cen and publish as pose stamped message
            node_info = cen.get("node", {})
            node_pose = node_info.get("pose", {})
            parent_frame = node_info.get("parent_frame", "map")
            # get the node IDs from cen edges
            selected_edges_dict = {"node": {"parent_frame": parent_frame, "pose": node_pose, "edges": []}}
            # print node pose 
            self.get_logger().info(f"Center Node Pose: {node_pose}")

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = node_pose["position"]["x"]
            msg.pose.position.y = node_pose["position"]["y"]
            msg.pose.position.z = node_pose["position"]["z"]
            msg.pose.orientation.x = node_pose["orientation"]["x"]
            msg.pose.orientation.y = node_pose["orientation"]["y"]
            msg.pose.orientation.z = node_pose["orientation"]["z"]
            msg.pose.orientation.w = node_pose["orientation"]["w"]
            self.center_node_pose_pub.publish(msg)
            self.get_logger().info('Published latched Center Node Pose')

        except Exception as e:
            self.get_logger().error(f"[_handle_row_operation] Failed to extract center node info: {e}")
            
        # Collect boundary candidates
        candidates = self._collect_boundary_candidates(cen, target_row_edge_id)
        if not candidates:
            self.get_logger().error("[_handle_row_operation] No boundary candidates found")
            self._publish_empty_boundary(frame_id)
            action_msg.setSideEdges({}, frame_id)
            return self.ACTIONS.ROW_OPERATION, action_msg

        # Select last row goal and compute row direction
        selected_last_node = self._select_last_row_goal(nodes)
        cpos = cen["node"]["pose"]["position"]
        lpos = selected_last_node["target_pose"]["pose"]["position"]

        # Avoid using center as last goal
        if np.linalg.norm(np.array([lpos["x"] - cpos["x"], lpos["y"] - cpos["y"]])) < 1e-6:
            selected_last_node = nodes[0]
            lpos = selected_last_node["target_pose"]["pose"]["position"]

        dx, dy = lpos["x"] - cpos["x"], lpos["y"] - cpos["y"]
        row_yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else self._yaw_from_quat(cen["node"]["pose"]["orientation"])
        row_dir = np.array([math.cos(row_yaw), math.sin(row_yaw)], dtype=float)
        center_xy = np.array([cpos["x"], cpos["y"]], dtype=float)

        # Select boundary nodes
        picked = self._select_boundary_nodes(candidates, center_xy, row_dir, tag_id)
        if not picked:
            self.get_logger().error("[_handle_row_operation] No valid boundary nodes selected")
            self._publish_empty_boundary(frame_id)
            action_msg.setSideEdges({}, frame_id)
            return self.ACTIONS.ROW_OPERATION, action_msg

        # Build selected_edges with poses aligned to row direction
        self.selected_edges = {}
        for child in picked:
            child_node = self.route_search.get_node_from_tmap2(child)
            if not child_node or "node" not in child_node:
                continue

            side_pose = self.get_intermediate_pose(cen, child_node, frame_id)
            side_last_pose = self.get_last_intermediate_pose(side_pose, cen, selected_last_node)
            
            # self.get_logger().info(f"Side Pose: {side_pose}")
            self.get_logger().info(f"Side Last Pose: {side_last_pose}")
            self.get_logger().info(f"Row Yaw: {row_yaw}")
            self.get_logger().info(f"Child Node ID: {child}")
            # how to print the node orientation??
            self.get_logger().info(f"Child Node Orientation: {child_node['node']['pose']['orientation']}")
            # print node orientation as yaw
            child_yaw = self._yaw_from_quat(child_node["node"]["pose"]["orientation"])
            self.get_logger().info(f"Child Node Yaw: {child_yaw}")
            
            self._set_pose_yaw(side_pose, child_yaw)
            self._set_pose_yaw(side_last_pose, child_yaw)
            self.selected_edges[child] = [side_pose, side_last_pose]

        # Mirror if only one wall found
        if len(self.selected_edges) == 1:
            self.get_logger().info("[_handle_row_operation] Single wall found, creating mirror")
            self.selected_edges["side_wall"] = self.get_intermediate_poses_interpolated(
                self.selected_edges, cen, selected_last_node
            )
            for ps in self.selected_edges["side_wall"]:
                self._set_pose_yaw(ps, child_yaw)

        if not self.selected_edges:
            self.get_logger().error("[_handle_row_operation] No edges built")
            self._publish_empty_boundary(frame_id)
            action_msg.setSideEdges({}, frame_id)
            return self.ACTIONS.ROW_OPERATION, action_msg

        # Publish boundary
        action_msg.setSideEdges(self.selected_edges, frame_id)
        if not self.is_row_boundary_published:
            boundary_info = action_msg.getBoundary()
            boundary_info.header.stamp = self.get_clock().now().to_msg()
            self.boundary_publisher.publish(boundary_info)
            self.is_row_boundary_published = True

        return self.ACTIONS.ROW_OPERATION, action_msg

    def get_navigate_through_poses_goal(self, poses, actions, edge_ids, is_execpolicy=False):


        
        control_server_configs = {}
        action_msgs = []
        self.is_row_boundary_published = False
        edge_action_is_valid = True
        
        
        # ==================================================================
        #  IF in Execute-Policy Mode: relaxed yaw tolerance for all segments
        # ==================================================================
        if is_execpolicy:
            control_server_configs = []

            self.get_logger().debug("[get_navigate_through_poses_goal] Executing with Execute-policy logic.")

            for seg_i, nodes in poses.items():
                nav_goal = NavigateThroughPoses.Goal()
                for pose in nodes:
                    target_pose = self.create_pose_stamped_msg(pose["target_pose"]["header"]["frame_id"], pose["target_pose"]["pose"])
                    nav_goal.poses.append(target_pose)

                action = actions[seg_i][0]
                edge_id = edge_ids[seg_i][0]
                action_msg = TopoNavEdgeActionMsg()

                self.get_logger().info("seg: {}, action: {}, edge id: {} ".format(seg_i, action, edge_id))
                
                
                self.get_logger().info(f"Segment {seg_i} -- action {action}")                    
                current_config = None
                if action in self.ACTIONS.bt_tree_with_control_server_config:
                    controller_plugin = self.ACTIONS.bt_tree_with_control_server_config[action]
                    current_config = self.ACTIONS.planner_with_goal_checker_config[controller_plugin].copy()

                    if action == self.ACTIONS.NAVIGATE_TO_POSE and seg_i < len(poses) - 1:
                        self.get_logger().info(f"Segment {seg_i} is intermediate NAVIGATE_TO_POSE. Relaxing yaw tolerance.")
                        current_config['goal_checker.yaw_goal_tolerance'] = 2 * math.pi
                    
                control_server_configs.append(current_config)

                if current_config:
                    self.get_logger().warn(f"Segment {seg_i} | XY tol: {current_config['goal_checker.xy_goal_tolerance']} | Yaw tol: {current_config['goal_checker.yaw_goal_tolerance']}")


                if action in self.bt_trees:
                    nav_goal.behavior_tree = self.bt_trees[action]
                edge_action_is_valid = True
                
                if action == self.ACTIONS.ROW_TRAVERSAL and self.in_row_operation and nodes:
                    self.get_logger().warn(f"Segment {seg_i}: ROW_TRAVERSAL with in_row_operation")
                    action, action_msg = self._handle_row_operation(nodes, edge_id, action_msg)

                self.get_logger().info(" Action {}  Bt_tree : {}".format(action, nav_goal.behavior_tree))
                if edge_action_is_valid:
                    self.get_logger().warn(f"Segment {seg_i} edge_action_is_valid")
                    action_msg.setAction(action)
                    action_msg.setNavGoal(nav_goal)
                    action_msgs.append(action_msg)

        # ==================================================================
        #  ELSE (Go-To-Node Mode): Use the original logic
        # ==================================================================
        else:

            self.get_logger().debug("[get_navigate_through_poses_goal] Executing with Go-To-Node logic.")
        
            for seg_i, nodes  in poses.items():
                nav_goal = NavigateThroughPoses.Goal()
                for pose in nodes:
                    target_pose = self.create_pose_stamped_msg(pose["target_pose"]["header"]["frame_id"], pose["target_pose"]["pose"])
                    nav_goal.poses.append(target_pose)

                action = actions[seg_i][0]
                edge_id = edge_ids[seg_i][0]
                action_msg = TopoNavEdgeActionMsg()

                self.get_logger().info("seg: {}, action: {}, edge id: {} ".format(seg_i, action, edge_id))

                if action in self.ACTIONS.bt_tree_with_control_server_config:
                    controller_plugin = self.ACTIONS.bt_tree_with_control_server_config[action]
                    control_server_configs[action] = self.ACTIONS.planner_with_goal_checker_config[controller_plugin]
                    if action in self.bt_trees:
                        nav_goal.behavior_tree = self.bt_trees[action]
                    edge_action_is_valid = True

                if action == self.ACTIONS.ROW_TRAVERSAL and self.in_row_operation and nodes:
                    self.get_logger().warn(f"Segment {seg_i}: ROW_TRAVERSAL with in_row_operation")
                    action, action_msg = self._handle_row_operation(nodes, edge_id, action_msg)

                self.get_logger().info(" Action {}  Bt_tree : {}".format(action, nav_goal.behavior_tree))
                if edge_action_is_valid:
                    action_msg.setAction(action)
                    action_msg.setNavGoal(nav_goal)
                    action_msgs.append(action_msg)

        self.get_logger().debug("[get_navigate_through_poses_goal] - Control server configs: {}".format(control_server_configs))
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
        return self.goal_response

    def execute_row_operation_one_step(self, next_goal, target_pose_frame_id):
        target_goal = NavigateThroughPoses.Goal()
        if(self.ACTIONS.ROW_TRAVERSAL in self.bt_trees): 
            target_goal.behavior_tree = self.bt_trees[self.ACTIONS.ROW_TRAVERSAL]
            self.get_logger().info("Row traversal BT path {}".format(target_goal.behavior_tree))
        else:
            self.get_logger().info("Row traversal BT path not found")
        target_pose = self.crete_pose_stamped_msg_from_position(target_pose_frame_id, next_goal)
        target_goal.poses.append(target_pose)
        
        controller_plugin = self.ACTIONS.bt_tree_with_control_server_config[self.ACTIONS.ROW_TRAVERSAL]
        
        control_server_config = self.ACTIONS.planner_with_goal_checker_config[controller_plugin] 
        self.get_logger().info(" Control params {}".format(control_server_config))
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
                self.get_logger().error("Nav2 server got some unexpected errors : {} while executing  send_goal_request {}".format(e, msg))
                return False 
                  
        if not self.goal_handle.accepted:
            self.get_logger().error('The goal rejected: {}'.format(msg))
            return False
        self.get_logger().info('The goal accepted for {}'.format(msg))
        return True 

    def processing_goal_request(self, target_action):
        if self.goal_handle is None:
            self.get_logger().error("Something wrong with the goal request, there is no goal to process {}".format(self.action_status))
            return True
        self.goal_get_result_future = self.goal_handle.get_result_async()
        self.get_logger().info("Waiting for {} action to complete".format(self.action_server_name))
        while rclpy.ok():
            try:
                # rclpy.spin_once(self)
                # print("goal_get_result_future ", self.goal_get_result_future)
                rclpy.spin_once(self, executor=self.executor_nav_client, timeout_sec=1.5)
                # rclpy.spin_until_future_complete(self, self.goal_get_result_future, executor=self.executor_nav_client, timeout_sec=2.0)
                if self.goal_get_result_future.done():
                    status = self.goal_get_result_future.result().status
                    self.action_status = status
                    self.get_logger().info("Executing the action response with status {}".format(self.get_status_msg(self.action_status)))
                    self.current_action = self.action_name
                    self.goal_response = self.goal_get_result_future.result()
                    if((self.get_status_msg(self.action_status) == "STATUS_EXECUTING") or (self.get_status_msg(self.action_status) == "STATUS_SUCCEEDED")):
                        self.get_logger().info("action is completed {}".format(target_action))
                        break  
                    if target_action in self.ACTIONS.ABORT_NOT_CONTINUE:
                            self.get_logger().info("action is not completed {}".format(target_action))
                            return False 
                    else:
                        self.get_logger().info("action is not completed {}. Executing next actions...".format(target_action))
                        break 
            except Exception as e:
                # self.goal_handle = Nosne 
                self.get_logger().error("Nav2 server got some unexpected errors: {} while executing processing_goal_request {}".format(e, target_action))
                return False
        return True 

    def execute_row_operation(self,):
        target_goal = NavigateThroughPoses.Goal()
        if(self.ACTIONS.ROW_OPERATION in self.bt_trees): 
            target_goal.behavior_tree = self.bt_trees[self.ACTIONS.ROW_OPERATION]
            self.get_logger().info("Row operation BT path {}".format(target_goal.behavior_tree))
        else:
            self.get_logger().error("Row operation BT is not provided {}")
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
            self.get_logger().info("Row recovery BT path {}".format(target_goal.behavior_tree))
        else:
            self.get_logger().error("Row recovery BT is not provided {}")
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
            self.get_logger().info("Executing action : {} ".format(self.ACTIONS.ROW_OPERATION))
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
                    self.get_logger().error("Could not set the current pose of the robot {}".format(e, self.ACTIONS.ROW_OPERATION))
                    return False
                
            if(len(self.in_row_inter_pose)==1):
                self.in_row_inter_pose = [self.current_robot_pose, self.in_row_inter_pose[0]]

            self.get_logger().info(" ==========start executing in row operaitons============")
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
                    self.get_logger().info("done_operation {} ".format(done_operation))
                    # target_goal = NavigateThroughPoses.Goal()
                    self.get_logger().info("Robot current pose: {},{}"
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
                            self.get_logger().info("Reach to the final goal {},{}"
                                        .format(next_goal.pose.position.x, next_goal.pose.position.y))
                            
                        else:
                            self.get_logger().error("Can not reach to the final goal {},{}"
                                            .format(next_goal.pose.position.x, next_goal.pose.position.y))
                        break
        return True 
                            
                            
    def execute(self):     
        self.get_logger().warn("===========================================Executing action===========================================")
        if not self.client.server_is_ready():
            self.get_logger().info("Waiting for the action server  {}...".format(self.action_server_name))
            self.client.wait_for_server(timeout_sec=2)
        
        if not self.client.server_is_ready():
            self.get_logger().info("action server  {} not responding ... can not perform any action".format(self.action_server_name))
            return

        self.get_logger().info("Number of actions : {} ".format(len(self.action_msgs)))
        for index, action_msg in enumerate(self.action_msgs):
            target_goal, target_action = action_msg.getNavGoal(), action_msg.getAction()
            self.get_logger().warn("Executing action : {} ".format(target_action))

            # Publish segment destination for visualisation purposes
            current_destination = (target_goal.poses[-1].pose.position.x, target_goal.poses[-1].pose.position.y)
            self.get_logger().warn("Current destination : {} ".format(self.destination_node_str[current_destination]))
            self.current_dest.publish(String(data=self.destination_node_str[current_destination]))
            
            # Handles both lists and dictionaries
            control_server_config = None
            if isinstance(self.control_server_configs, list):
                # New exec_policy mode
                if index < len(self.control_server_configs):
                    control_server_config = self.control_server_configs[index]
            elif isinstance(self.control_server_configs, dict):
                # Original go-to-node mode
                if target_action in self.control_server_configs:
                    control_server_config = self.control_server_configs[target_action]
            
            # If a valid config was found, apply it.
            if control_server_config:
                self.get_logger().info(f"Applying parameters for segment {index}: {control_server_config}")
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
