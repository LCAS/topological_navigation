#!/usr/bin/env python
"""
Created on Tue Nov 5 22:02:24 2023
@author: Geesara Kulathunga (ggeesara@gmail.com)

"""
#########################################################################################################
import json, yaml
import operator, collections, copy
import rclpy 
from functools import reduce  # forward compatibility for Python 3
from rclpy.action import ActionClient 
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
# from rospy_message_converter import message_converter
# from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Header 
from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup 
from unique_identifier_msgs.msg import UUID
from rclpy.executors import SingleThreadedExecutor


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
#########################################################################################################


#########################################################################################################
class EdgeActionManager(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("edge_action_manager")
        self.client = None        
        self.current_action = "none"
        self.dt = dict_tools()
        self.status_mapping = {}
        self.status_mapping[0] = "STATUS_UNKNOWN"
        self.status_mapping[1] = "STATUS_ACCEPTED"
        self.status_mapping[2] = "STATUS_EXECUTING"
        self.status_mapping[3] = "STATUS_CANCELING"
        self.status_mapping[4] = "STATUS_SUCCEEDED"
        self.status_mapping[5] = "STATUS_CANCELED"
        self.status_mapping[6] = "STATUS_ABORTED"
        self.goal_cancle_error_codes = {} 
        self.goal_cancle_error_codes[0] = "ERROR_NONE"
        self.goal_cancle_error_codes[1] = "ERROR_REJECTED"
        self.goal_cancle_error_codes[2] = "ERROR_UNKNOWN_GOAL_ID"
        self.goal_cancle_error_codes[3] = "ERROR_GOAL_TERMINATED"
        self.goal_handle = None 
        self.internal_executor = SingleThreadedExecutor()

    
    def get_nav_action_server_status(self, ):
        return self.status_mapping 
    
    def get_status_msg(self, status_code):
        try:
            return self.status_mapping[status_code]
        except Exception as e:
            self.get_logger().error("Status code is invalid {}".format(status_code))
            return self.status_mapping[0]
        
    def get_goal_cancle_error_msg(self, status_code):
        try:
            return self.goal_cancle_error_codes[status_code]
        except Exception as e:
            self.get_logger().error("Goal cancle code {}".format(status_code))
            return self.goal_cancle_error_codes[0]
        
    def initialise(self, edge, destination_node, origin_node=None):
        
        self.edge = yaml.safe_load(json.dumps(edge)) # no unicode in edge
        self.destination_node = destination_node
        self.origin_node = origin_node
        self.get_logger().info("Edge Action Manager: Processing edge {}".format(self.edge["edge_id"]))
        
        self.action_name = self.edge["action"]
        if self.action_name == "row_traversal":
            self.action_name = "NavigateToPose"

        if self.action_name != self.current_action:
            self.preempt()

        action_type = self.edge["action_type"]
        package = "nav2_msgs.action" # TODO change this 

        action_spec = self.action_name
        self.action_server_name = self.count_action_server_name(self.action_name)
        self.get_logger().info("Edge Action Manager: Importing {} from {}".format(action_spec, package))
        action = _import(package, action_spec)

        self.latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.get_logger().info("Edge Action Manager: Creating a {} client".format(self.action_server_name))
        self.action = action 
        self.client = ActionClient(self, action, self.action_server_name)
        self.action_status = 0 

        self.get_logger().info("Edge Action Manager: Constructing the goal")
        self.construct_goal(action_type, copy.deepcopy(self.edge["goal"]))
    
    def feedback_callback(self, feedback_msg):
        self.nav_client_feedback = feedback_msg.feedback
        # self.get_logger().info("Distance to goal: {} ".format(self.nav_client_feedback.distance_remaining))
        return 
    
    def preempt_feedback_callback(self, feedback_msg):
        self.nav_client_feedback = feedback_msg.feedback
        return 

    def count_action_server_name(self, action_name):
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
        
    def preempt(self):
        if self.client is not None:
            if not self.client.server_is_ready():
                self.get_logger().info("Edge Action Manager: Waiting for the action server  {}...".format(self.action_server_name))
                self._action_client.wait_for_server(timeout_sec=2)
            
            if not self.client.server_is_ready():
                self.get_logger().info("Edge Action Manager: action server  {} not responding ... can not perform any action".format(self.action_server_name))
                return True
            
            if self.goal_handle is None:
                self.get_logger().info("There is no goal to stop it is already cancelled with status {}".format(self.action_status))
                return True 
             
            cancel_future = self.client._cancel_goal_async(self.goal_handle)
            self.get_logger().info("Waiting till terminating the current preemption")
            while rclpy.ok():
                try: 
                    rclpy.spin_once(self)
                    if cancel_future.done() and self.goal_get_result_future.done():
                        self.action_status = 5
                        self.get_logger().info("The goal cancel error code {} ".format(self.get_goal_cancle_error_msg(cancel_future.result().return_code)))
                        return True 
                except Exception as e:
                    pass 
        
    def construct_goal(self, action_type, goal_args):
        paths = self.dt.get_paths_from_nested_dict(goal_args)
        for item in paths:
            value = item["value"]
            if isinstance(value, str):
                if value.startswith("$"):
                    _property = self.dt.getFromDict(self.destination_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
                    
                elif value.startswith("+") and self.origin_node is not None:
                    _property = self.dt.getFromDict(self.origin_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
        self.goal = goal_args
        self.get_logger().info("  goal {} ".format(self.goal))

        #TODO need to add other types 
        if(self.action_name == "NavigateToPose"):
            self.nav_goal = NavigateToPose.Goal()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.goal["target_pose"]["header"]["frame_id"]
            target_pose = PoseStamped()
            target_pose.header = header 
            desired_target_pose = self.goal["target_pose"]["pose"]
            target_pose.pose.position.x = desired_target_pose["position"]["x"]
            target_pose.pose.position.y = desired_target_pose["position"]["y"]
            target_pose.pose.position.z = desired_target_pose["position"]["z"]
            target_pose.pose.orientation.w = desired_target_pose["orientation"]["w"]
            target_pose.pose.orientation.x = desired_target_pose["orientation"]["x"]
            target_pose.pose.orientation.y = desired_target_pose["orientation"]["y"]
            target_pose.pose.orientation.z = desired_target_pose["orientation"]["z"]
            self.nav_goal.pose = target_pose

        self.goal = self.nav_goal
        
    def get_result(self, ):
        return self.goal_resposne


    def execute(self):     
        
        if not self.client.server_is_ready():
            self.get_logger().info("Edge Action Manager: Waiting for the action server  {}...".format(self.action_server_name))
            self._action_client.wait_for_server(timeout_sec=2)
        
        if not self.client.server_is_ready():
            self.get_logger().info("Edge Action Manager: action server  {} not responding ... can not perform any action".format(self.action_server_name))
            return 
        
        self.get_logger().info("Edge Action Manager: Executing the action...")
        send_goal_future = self.client.send_goal_async(self.goal,  feedback_callback=self.feedback_callback)
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                if send_goal_future.done():
                    self.goal_handle = send_goal_future.result()
                    break
            except Exception as e:
                pass 

        if not self.goal_handle.accepted:
            self.get_logger().error('The goal rejected')
            return False

        self.get_logger().info('The goal accepted')
        self.goal_get_result_future = self.goal_handle.get_result_async()
        self.get_logger().info("Waiting for {} action to complete".format(self.action_server_name))
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                if self.goal_get_result_future.done():
                    status = self.goal_get_result_future.result().status
                    self.action_status = status
                    self.get_logger().info("Edge Action Manager: Executing the action response with status {}".format(self.get_status_msg(self.action_status)))
                    self.current_action = self.action_name
                    self.goal_resposne = self.goal_get_result_future.result() 
                    return True 
            except Exception as e:
                pass 
                            
#########################################################################################################