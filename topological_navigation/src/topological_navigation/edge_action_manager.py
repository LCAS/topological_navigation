#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib
from rospy_message_converter import message_converter
import operator
from copy import deepcopy
from functools import reduce  # forward compatibility for Python 3


class EdgeActionManager(object):
    
    
    def __init__(self, edge, destination_node):
        
        rospy.loginfo("Edge Action Manager: Processing edge {} ...".format(edge["edge_id"]))
        
        self.edge = edge
        self.destination_node = destination_node
        
        action_type = edge["action_type"]
        self.action_name = edge["action"]
        
        items = action_type.split("/")
        package = items[0]
        action_spec = items[1][:-4] + "Action"
        
        rospy.loginfo("Edge Action Manager: Importing {} from {}.msg".format(action_spec, package))
        action = self._import(package + ".msg", action_spec)
        
        rospy.loginfo("Edge Action Manager: Creating a {} client".format(self.action_name))
        self.client = actionlib.SimpleActionClient(self.action_name, action)        
        self.client.wait_for_server()
        
        rospy.loginfo("Edge Action Manager: Constructing the goal")
        self.construct_goal(action_type, edge["goal"])
        
        
    def _import(self, location, object_name):
        mod = __import__(location, fromlist=[object_name]) 
        return getattr(mod, object_name) 
        
        
    def construct_goal(self, action_type, goal_args):
        
        goal_args_cpy = deepcopy(goal_args)
        stack = list(goal_args_cpy.items()) 
        
        visited = set() 
        keys = []
        while stack: 
            k, v = stack.pop() 
            if isinstance(v, dict): 
                keys.append(k)
                if k not in visited: 
                    stack.extend(v.items()) 
            else:
                keys.append(k)
                if isinstance(v, str) and v.startswith("$"):
                    _property = self.getFromDict(self.destination_node, v[1:].split("."))
                    self.setInDict(goal_args, keys, _property)
                
                keys = [keys[0]]
            visited.add(k)
        
        self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)
        
        
    def getFromDict(self, dataDict, mapList):
        return reduce(operator.getitem, mapList, dataDict)


    def setInDict(self, dataDict, mapList, value):
        self.getFromDict(dataDict, mapList[:-1])[mapList[-1]] = value
        
        
    def execute(self):
        
        rospy.loginfo("Edge Action Manager: Executing the action")
        self.client.send_goal(self.goal, self.execute_callback)
        
        rospy.loginfo("Edge Action Manager: Waiting for the result ...")
        self.client.wait_for_result()
        
        return self.client.get_result()
    

    def execute_callback(self, status, result):
        
        if status == 3:
            rospy.loginfo("Edge Action Manager: Succeeded")
        elif status == 2 or status == 6:
            rospy.logwarn("Edge Action Manager: Preempted")
        elif status != 3:
            rospy.logerr("Edge Action Manager: Failed")    
#########################################################################################################