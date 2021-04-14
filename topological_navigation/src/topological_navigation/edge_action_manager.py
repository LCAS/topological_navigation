#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib


class EdgeActionManager(object):
    
    
    def __init__(self, edge):
        
        self.client_created = False
        
        action_type = edge["action_type"]
        action_name = edge["action"]
        
        items = action_type.split("/")
        package = items[0]
        goal_spec = items[1]
        action_spec = items[1][:-4] + "Action"
        
        action = self._import(package, action_spec)
        goal = self._import(package, goal_spec)
        
        rospy.loginfo("Edge Action Manager: Creating {} client with {} imported from {}.msg ...".format(action_name, action_spec, package))
        
        action_client = actionlib.SimpleActionClient(action_name, action)        
        self.client_created = action_client.wait_for_server(rospy.Duration(5.0))
        
        if self.client_created:
            rospy.loginfo("Edge Action Manager: Client created")
        else:
            rospy.logerr("Edge Action Manager: Client not created")
            
    
    def _import(self, package, object_name):
        mod = __import__(package + ".msg", fromlist=[object_name]) 
        return getattr(mod, object_name) 
#########################################################################################################