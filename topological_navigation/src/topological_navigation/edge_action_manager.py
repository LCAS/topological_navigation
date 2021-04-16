#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib
from rospy_message_converter import message_converter
from collections import namedtuple
from copy import deepcopy


class EdgeActionManager(object):
    
    
    def __init__(self, edge, destination_node, frame_id="map"):
        
        rospy.loginfo("Edge Action Manager: Processing edge {} ...".format(edge["edge_id"]))
        
        self.edge = edge
        self.destination_node = destination_node
        self.frame_id = frame_id
        
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
        
        
    def _import(self, _file, object_name):
        mod = __import__(_file, fromlist=[object_name]) 
        return getattr(mod, object_name) 
        
        
    def construct_goal(self, action_type, goal_args):
        
        node_dict = deepcopy(self.destination_node)
        
        # pose field of node modified to match a geometry_msgs/PoseStamped msg
        node_dict["pose"] = {"pose": node_dict["pose"]}
        node_dict["pose"]["header"] = {}
        node_dict["pose"]["header"]["frame_id"] = self.frame_id
        
        node = namedtuple("Node", node_dict.keys())(*node_dict.values())
        
        for arg in goal_args.keys():
            value = deepcopy(goal_args[arg])
            if type(value) is str and value.startswith("$node"):
                goal_args[arg] = eval(value[1:]) 
        
        self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)
        
        
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