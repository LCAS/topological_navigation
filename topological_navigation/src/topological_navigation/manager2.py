#!/usr/bin/env python
"""
Created on Tue Sep 29 16:06:36 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
default_verts = [{'x': 0.689,  'y': 0.287},  {'x': 0.287,  'y': 0.689},   {'x': -0.287, 'y': 0.689},
                 {'x': -0.689, 'y': 0.287},  {'x': -0.689, 'y': -0.287},  {'x': -0.287, 'y': -0.689},
                 {'x': 0.287,  'y': -0.689}, {'x': 0.689,  'y': -0.287}]



class map_manager_2(object):
    
    
    def __init__(self, name="new_map", metric_map="map_2d", pointset="new_map", transformation="default", tmap2=None):
        
        self.name = name
        self.metric_map = metric_map
        self.pointset = pointset
        
        if transformation == "default":
            self.transformation = {}
            self.transformation["rotation"] = {}
            self.transformation["rotation"]["w"] = 1.0
            self.transformation["rotation"]["x"] = 0.0
            self.transformation["rotation"]["y"] = 0.0
            self.transformation["rotation"]["z"] = 0.0
            self.transformation["translation"] = {}
            self.transformation["translation"]["x"] = 0.0
            self.transformation["translation"]["y"] = 0.0
            self.transformation["translation"]["z"] = 0.0
            self.transformation["child"] = self.name
            self.transformation["parent"] = self.metric_map
        else:
            self.transformation = transformation
            
        if tmap2 is None:
            self.tmap2 = {}
            self.tmap2["name"] = self.name
            self.tmap2["metric_map"] = self.metric_map
            self.tmap2["pointset"] = self.pointset
            self.tmap2["transformation"] = self.transformation
            self.tmap2["meta"] = {}
            self.tmap2["meta"]["last_updated"] = 0.0 #rospy.Time.now()
            self.tmap2["nodes"] = []
        else:
            self.name = tmap2["name"]
            self.metric_map = tmap2["metric_map"]
            self.pointset = tmap2["pointset"]
            self.transformation = tmap2["transformation"]
            self.tmap2 = tmap2
        
        
    def add_node(self, name, pose, localise_by_topic="", verts="default", properties="default", restrictions=None):
        
        if "orientation" not in pose:
            pose["orientation"] = {}
            pose["orientation"]["w"] = 1.0
            pose["orientation"]["x"] = 0.0
            pose["orientation"]["y"] = 0.0
            pose["orientation"]["z"] = 0.0
        
        node = {}
        node["meta"] = {}
        node["meta"]["map"] = self.metric_map
        node["meta"]["node"] = name
        node["meta"]["pointset"] = self.pointset
        
        node["node"] = {}
        node["node"]["edges"] = []
        node["node"]["localise_by_topic"] = localise_by_topic
        node["node"]["name"] = name        
        node["node"]["pose"] = pose
        
        if verts == "default":
            node["node"]["verts"] = default_verts
        else:
            node["node"]["verts"] = verts
            
        if properties == "default":
            node["node"]["properties"] = {}
            node["node"]["properties"]["xy_goal_tolerance"] = 0.3
            node["node"]["properties"]["yaw_goal_tolerance"] = 0.1
        else:
            node["node"]["properties"] = properties
            
        if restrictions is None:
            node["node"]["restrictions"] = {}
            node["node"]["restrictions"]["robot_type"] = ""
        else:
            node["node"]["restrictions"] = restrictions
            
        self.tmap2["nodes"].append(node)
        
        
    def add_edge_to_node(self, origin, destination, action="move_base", config="default", 
                         action_type="default", goal="default", 
                         fail_policy="fail", restrictions=None):
        
        edge = {}
        edge["action"] = action
        edge["node"] = destination
        
        if config == "default":
            edge["config"] = {}
            edge["config"]["inflation_radius"] = 0.0
            edge["config"]["top_vel"] = 0.55
            edge["config"]["recovery_behaviours_config"] = ""
        else:
            edge["config"] = config
        
        if action_type == "default":
            edge["action_type"] = self.set_action_type(action)
        else:
            edge["action_type"] = action_type
            
        if goal == "default":
            edge["goal"] = {}
            edge["goal"]["target_pose"] = "$node.pose"
        else:
            edge["goal"] = goal
            
        edge["fail_policy"] = fail_policy
        
        if restrictions is None:
            edge["restrictions"] = {}
            edge["restrictions"]["robot_type"] = ""
        else:
            edge["restrictions"] = restrictions
        
        for node in self.tmap2["nodes"]:
            if node["meta"]["node"] == origin and node["node"]["name"] == origin:
                node["node"]["edges"].append(edge)
                
                
    def set_action_type(self, action):
        
        package = action + "_msgs"
        items = [item[0].upper() + item[1:] for item in action.split("_")]
        goal_type = "".join(items) + "Goal"
        action_type = package + "/" + goal_type
            
        return action_type
#########################################################################################################