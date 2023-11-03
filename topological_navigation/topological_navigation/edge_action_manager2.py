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

# from rospy_message_converter import message_converter
# from actionlib_msgs.msg import GoalStatus


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
            if isinstance(value, collections.Mapping):
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
        items = action_type.split("/")
        package = items[0]
        action_spec = items[1][:-4] + "Action"
        
        self.get_logger().info("Edge Action Manager: Importing {} from {}.msg".format(action_spec, package))
        action = _import(package+".msg", action_spec)
        
        self.get_logger().info("Edge Action Manager: Creating a {} client".format(self.action_name.upper()))
        self.action = action 
        self.client = ActionClient(self, action, self.action_name)
        self.get_logger().info("Edge Action Manager: Constructing the goal")
        self.construct_goal(action_type, copy.deepcopy(self.edge["goal"]))
        
        
    def preempt(self):
        if self.client is not None:
            goal_msg = self.action.Goal()
            future = self.client.send_goal(goal_msg, self.status_callback)
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    try:
                        if self.action_status == GoalStatus.PENDING or self.action_status == GoalStatus.ACTIVE:
                            self.client.cancel_all_goals()
                            return True 
                    except Exception as e:
                        self.get_logger().error("can not call the action server {}".format(self.action_name))
                        pass
                    return False
        
    def status_callback(self, goal_handle):
        # get the current status of the action server
        self.action_status = goal_handle.get_status()
        
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
        self.goal = goal_args #TODO need to convert to correct message type 
        # self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)
        
 
    def execute(self):     
        self.get_logger().info("Edge Action Manager: Executing the action...")
        self.client.send_goal(self.goal)
        self.current_action = self.action_name
#########################################################################################################