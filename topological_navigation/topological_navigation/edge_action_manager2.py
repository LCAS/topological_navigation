#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

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
# https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
from action_msgs.msg import GoalStatus

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
        
    def initialise(self, edge, destination_node, origin_node=None):
        
        self.edge = yaml.safe_load(json.dumps(edge)) # no unicode in edge
        self.destination_node = destination_node
        self.origin_node = origin_node
        self.get_logger().info("Edge Action Manager: Processing edge {}".format(self.edge["edge_id"]))
        
        self.action_name = self.edge["action"]
        if self.action_name != self.current_action:
            self.preempt()

        action_type = self.edge["action_type"]        
        # items = action_type.split("/")
        # package = items[0]
        package = "nav2_msgs.action"

        
        action_spec = self.action_name
        self.action_server_name = self.count_action_server_name(self.action_name)
        self.get_logger().info("Edge Action Manager: Importing {} from {}".format(action_spec, package))
        action = _import(package, action_spec)
        
        self.get_logger().info("Edge Action Manager: Creating a {} client".format(self.action_server_name))
        self.action = action 
        self.client = ActionClient(self, action, self.action_server_name)
        self.get_logger().info("Edge Action Manager: Constructing the goal")
        self.construct_goal(action_type, copy.deepcopy(self.edge["goal"]))
        
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
            goal_msg = self.action.Goal()
            future = self.client.send_goal_async(goal_msg)
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    try:
                        respose = future.result 
                        self.get_logger().info(" preempt repose {}".format(respose))
                        self.action_status = self.client.get_feedback()
                        if self.action_status == GoalStatus.PENDING or self.action_status == GoalStatus.ACTIVE:
                            self.client.cancel_all_goals()
                            return True 
                    except Exception as e:
                        self.get_logger().error("can not call the action server {}".format(self.action_name))
                        pass
                    return False
        
    def construct_goal(self, action_type, goal_args):
        paths = self.dt.get_paths_from_nested_dict(goal_args)
        # self.get_logger().info("   {}".format(paths))
        for item in paths:
            value = item["value"]
            # self.get_logger().info("  value {}".format(value))
            if isinstance(value, str):
                if value.startswith("$"):
                    # self.get_logger().info("  value {}".format(value))
                    _property = self.dt.getFromDict(self.destination_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
                    
                elif value.startswith("+") and self.origin_node is not None:
                    _property = self.dt.getFromDict(self.origin_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
        self.goal = goal_args #TODO need to convert to correct message type for other message types 
        self.get_logger().info("  goal {} ".format(self.goal))
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
        self.get_logger().info("Edge Action Manager: Executing the action...")
        self.future = self.client.send_goal_async(self.goal)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                    self.goal_resposne = response 
                    self.action_status = response.status 
                    self.get_logger().info("Edge Action Manager: Executing the action respose {} with status {}".format(response, self.action_status))
                    self.current_action = self.action_name
                    if (self.action_status != GoalStatus.STATUS_EXECUTING):
                        self.get_logger().info("Edge Action Manager: Done executing action respose {} with status {}".format(response, self.action_status))
                        return False   
                except Exception as e:
                    self.get_logger().error("Edge Action Manager: Executing the action error: ".format(e))
                    pass
                return False
        
        
#########################################################################################################