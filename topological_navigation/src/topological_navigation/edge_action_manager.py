#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib
import operator, collections

from functools import reduce  # forward compatibility for Python 3
from rospy_message_converter import message_converter


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
    
    
    def __init__(self, edge, destination_node):
        
        self.client_created = False
        
        self.edge = edge # remove unicode prefix notation u
        self.destination_node = destination_node
        
        rospy.loginfo("Edge Action Manager: Processing edge {} ...".format(self.edge["edge_id"]))
        
        action_type = self.edge["action_type"]
        action_name = self.edge["action"]
        
        items = action_type.split("/")
        package = items[0]
        action_spec = items[1][:-4] + "Action"
        
        rospy.loginfo("Edge Action Manager: Importing {} from {}.msg".format(action_spec, package))
        action = _import(package + ".msg", action_spec)
        
        rospy.loginfo("Edge Action Manager: Creating a {} client".format(action_name))
        self.client = actionlib.SimpleActionClient(action_name, action)        
        self.client.wait_for_server()
        self.client_created = True
        
        rospy.loginfo("Edge Action Manager: Constructing the goal")
        self.construct_goal(action_type, self.edge["goal"])
        
        
    def construct_goal(self, action_type, goal_args):
        
        dt = dict_tools()
        paths = dt.get_paths_from_nested_dict(goal_args)
        
        for item in paths:
            keys = item["keys"]
            value = item["value"]
        
            if isinstance(value, str) and value.startswith("$"):
                _property = dt.getFromDict(self.destination_node, value[1:].split("."))
                goal_args = dt.setInDict(goal_args, keys, _property)

        self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)
        
        
    def execute(self):
        
        rospy.loginfo("Edge Action Manager: Executing the action")
        self.client.send_goal(self.goal)
        
        rospy.loginfo("Edge Action Manager: Waiting for the result ...")
#########################################################################################################