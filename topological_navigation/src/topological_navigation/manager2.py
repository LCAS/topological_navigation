#!/usr/bin/env python
"""
Created on Tue Sep 29 16:06:36 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import rospy, tf2_ros
import yaml, datetime, json

import strands_navigation_msgs.srv
import std_msgs.msg

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped

from rospy_message_converter import message_converter


default_verts = [{'x': 0.689,  'y': 0.287},  {'x': 0.287,  'y': 0.689},   {'x': -0.287, 'y': 0.689},
                 {'x': -0.689, 'y': 0.287},  {'x': -0.689, 'y': -0.287},  {'x': -0.287, 'y': -0.689},
                 {'x': 0.287,  'y': -0.689}, {'x': 0.689,  'y': -0.287}]


class map_manager_2(object):
    
    
    def __init__(self, name="new_map", metric_map="map_2d", pointset="new_map", transformation="default", tmap2_path=""):
        
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
            self.transformation["child"] = "topo_map"
            self.transformation["parent"] = "map"
        else:
            self.transformation = transformation
            
        if not tmap2_path:
            self.tmap2 = {}
            self.tmap2["name"] = self.name
            self.tmap2["metric_map"] = self.metric_map
            self.tmap2["pointset"] = self.pointset
            self.tmap2["transformation"] = self.transformation
            self.tmap2["meta"] = {}
            self.tmap2["meta"]["last_updated"] = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self.tmap2["nodes"] = []
        else:
            self.load_map(tmap2_path)
            
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcast_transform()
        
        self.map_pub = rospy.Publisher('/topological_map_2', std_msgs.msg.String, latch=True, queue_size=1)
        self.map_pub.publish(std_msgs.msg.String(json.dumps(self.tmap2)))
        
        self.get_map_srv=rospy.Service('/topological_map_manager2/get_topological_map', Trigger, self.get_topological_map_cb)
        self.get_tagged_srv=rospy.Service('/topological_map_manager2/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes, self.get_tagged_cb)       
        self.get_tag_srv=rospy.Service('/topological_map_manager2/get_tags', strands_navigation_msgs.srv.GetTags, self.get_tags_cb)
        self.get_node_tag_srv=rospy.Service('/topological_map_manager2/get_node_tags', strands_navigation_msgs.srv.GetNodeTags, self.get_node_tags_cb)

        
    def update(self):
        self.map_pub.publish(std_msgs.msg.String(json.dumps(self.tmap2)))
        
        
    def load_map(self, filename):
        
        with open(filename, "r") as f:
            try:
                self.tmap2 = yaml.safe_load(f)
            except Exception as e:
                rospy.logerr(e)
                exit()
                
        self.name = self.tmap2["name"]
        self.metric_map = self.tmap2["metric_map"]
        self.pointset = self.tmap2["pointset"]
        self.transformation = self.tmap2["transformation"]
        
        
    def broadcast_transform(self):
        
        trans = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Vector3", self.transformation["translation"])
        rot = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Quaternion", self.transformation["rotation"])
        
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.transformation["parent"]
        msg.child_frame_id = self.transformation["child"]
        msg.transform.translation = trans
        msg.transform.rotation = rot
        
        self.broadcaster.sendTransform(msg)
        
        
    def add_node(self, name, pose, localise_by_topic="", verts="default", properties="default", tags=[], 
                 restrictions=None, update=False):
        
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
        if tags:
            node["meta"]["tag"] = tags
        
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
        
        if update:
            self.update()
        
        
    def add_edge_to_node(self, origin, destination, action="move_base", edge_id="default", config="default", 
                         action_type="default", goal="default", fail_policy="fail", restrictions=None, update=False):
        
        edge = {}
        edge["action"] = action
        
        if edge_id == "default":
            edge["edge_id"] = origin + "_" + destination
        else:
            edge["edge_id"] = edge_id
        
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
                
        if update:
            self.update()
                
                
    def set_action_type(self, action):
        
        package = action + "_msgs"
        items = [item[0].upper() + item[1:] for item in action.split("_")]
        goal_type = "".join(items) + "Goal"
        action_type = package + "/" + goal_type
            
        return action_type
    
    
    def get_topological_map_cb(self, req):
        """
        Returns the topological map
        """
        ans = TriggerResponse()
        ans.success = True
        ans.message = json.dumps(self.tmap2)
        
        return ans
    
    
    def get_tagged_cb(self, req):
        """
        Returns a list of nodes that have a given tag
        """
        names=[]
        for node in self.tmap2["nodes"]:
            if "tag" in node["meta"]:
                if req.tag in node["meta"]["tag"]:
                    names.append(node["node"]["name"])
                    
        return [names]
    
    
    def get_tags_cb(self, req):
        """
        Returns a list of available tags in the map
        """
        tt = [tag for node in self.tmap2["nodes"] if "tag" in node["meta"] for tag in node["meta"]["tag"]]
        return [set(tt)]
    
    
    def get_node_tags_cb(self, req):
        """
        Returns a list of a node's tags
        """
        num_available = 0
        for node in self.tmap2["nodes"]:
            if node["meta"]["node"] == req.node_name and node["node"]["name"] == req.node_name:
                if "tag" in node["meta"]:
                    tags = node["meta"]["tag"]
                else:
                    tags = []
                    
                num_available+=1
                
        if num_available == 1:
            succeded = True
        else:
            succeded = False
            tags = []
            
        return succeded, tags
#########################################################################################################