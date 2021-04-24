#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib
import operator, collections, json

from functools import reduce  # forward compatibility for Python 3
from rospy_message_converter import message_converter

from actionlib_msgs.msg import GoalStatus


def _import(location, object_name):
    mod = __import__(location, fromlist=[object_name]) 
    return getattr(mod, object_name) 


class dict_tools(object):
                
                
    def get_paths_from_nested_dict(self, nested):
        paths = list(self.nested_dict_iter(nested))
        return [{"keys": item[0], "value": item[1]} for item in paths]
    
    
    def nested_dict_iter(self, nested, prefix=""):
        """
        Recursively loops through a nested dictionary. 
        For each inner-most value generates the list of keys needed to access it.
        """
        for key, value in nested.iteritems():
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
class EdgeActionManager(object):
    
    
    def __init__(self):
        
        self.client = None        
        self.current_action = None
        self.dt = dict_tools()
        
    
    def initialise(self, edge, destination_node):
        
        self.edge = eval(json.dumps(edge)) # remove unicode prefix notation u
        self.destination_node = eval(json.dumps(destination_node))
        
        rospy.loginfo("Edge Action Manager: Processing edge {} ...".format(self.edge["edge_id"]))
        
        action_type = self.edge["action_type"]
        self.action_name = self.edge["action"]
        
        if self.action_name != self.current_action and self.current_action is not None:
            self.preempt()
        
        items = action_type.split("/")
        package = items[0]
        action_spec = items[1][:-4] + "Action"
        
        rospy.loginfo("Edge Action Manager: Importing {} from {}.msg".format(action_spec, package))
        action = _import(package + ".msg", action_spec)
        
        rospy.loginfo("Edge Action Manager: Creating a {} client".format(self.action_name))
        self.client = actionlib.SimpleActionClient(self.action_name, action)        
        self.client.wait_for_server()
        
        rospy.loginfo("Edge Action Manager: Constructing the goal")
        self.construct_goal(action_type, self.edge["goal"])
        
        
    def preempt(self):
        
        if self.client is not None:
            status = self.client.get_state()
            if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:
                self.client.cancel_all_goals()
        
        
    def construct_goal(self, action_type, goal_args):
        
        paths = self.dt.get_paths_from_nested_dict(goal_args)
        
        for item in paths:
            value = item["value"]
            
            if isinstance(value, str) and value.startswith("$"):
                _property = self.dt.getFromDict(self.destination_node, value[1:].split("."))
                goal_args = self.dt.setInDict(goal_args, item["keys"], _property)

        self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)
        
 
    def execute(self):
        
        rospy.loginfo("Edge Action Manager: Executing the action")
        self.client.send_goal(self.goal)
        self.current_action = self.action_name
        rospy.loginfo("Edge Action Manager: Waiting for the result ...")
#########################################################################################################