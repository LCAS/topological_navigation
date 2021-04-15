#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib


class EdgeActionManager(object):
    
    
    def __init__(self, edge):
        
        action_type = edge["action_type"]
        action_name = edge["action"]
        
        items = action_type.split("/")
        package = items[0]
        goal_spec = items[1]
        action_spec = items[1][:-4] + "Action"
        
        rospy.loginfo("Edge Action Manager: Importing {} from {}.msg".format(action_spec, package))
        
        action = self._import(package, action_spec)
        goal = self._import(package, goal_spec)
        
        rospy.loginfo("Edge Action Manager: Creating {} client ...".format(action_name))
        
        action_client = actionlib.SimpleActionClient(action_name, action)        
        action_client.wait_for_server()
        
        rospy.loginfo("Edge Action Manager: Client created")
        print action_client.get_state()
        res = action_client.get_result()

            
    
    def _import(self, package, object_name):
        mod = __import__(package + ".msg", fromlist=[object_name]) 
        return getattr(mod, object_name) 
#########################################################################################################