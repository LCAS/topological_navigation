#!/usr/bin/env python
"""
Created on Tue Sep 29 16:06:36 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
from __future__ import division
import rospy, tf2_ros, math
import yaml, datetime, json
import re, uuid, copy

import strands_navigation_msgs.srv
import topological_navigation_msgs.srv
import std_msgs.msg

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped

from rospy_message_converter import message_converter


def pose_dist(pose1, pose2):
    dist = math.sqrt((pose1["position"]["x"] - pose2["position"]["x"])**2 + (pose1["position"]["y"] - pose2["position"]["y"])**2)
    return dist


class map_manager_2(object):
    
    
    def __init__(self, name="new_map", metric_map="map_2d", pointset="new_map", transformation="default", filename="", load=True):
        
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
            
        self.filename = filename
        self.load = load
            
        if self.filename and self.load:
            self.load_map(self.filename)
        else:
            self.tmap2 = {}
            self.tmap2["name"] = self.name
            self.tmap2["metric_map"] = self.metric_map
            self.tmap2["pointset"] = self.pointset
            self.tmap2["transformation"] = self.transformation
            self.tmap2["meta"] = {}
            self.tmap2["meta"]["last_updated"] = self.get_time()
            self.tmap2["nodes"] = []

        self.map_pub = rospy.Publisher('/topological_map_2', std_msgs.msg.String, latch=True, queue_size=1) 
        self.map_pub.publish(std_msgs.msg.String(json.dumps(self.tmap2)))
        self.names = self.create_list_of_nodes()
        
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcast_transform()
        
        # Services that retrieve information from the map
        self.get_map_srv=rospy.Service('/topological_map_manager2/get_topological_map', Trigger, self.get_topological_map_cb)
        self.switch_map_srv=rospy.Service('/topological_map_manager2/switch_topological_map', topological_navigation_msgs.srv.GetTopologicalMap, self.switch_topological_map_cb)
        self.get_tagged_srv=rospy.Service('/topological_map_manager2/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes, self.get_tagged_cb)       
        self.get_tag_srv=rospy.Service('/topological_map_manager2/get_tags', strands_navigation_msgs.srv.GetTags, self.get_tags_cb)
        self.get_node_tag_srv=rospy.Service('/topological_map_manager2/get_node_tags', strands_navigation_msgs.srv.GetNodeTags, self.get_node_tags_cb)
        self.get_node_edges_srv=rospy.Service('/topological_map_manager2/get_edges_between_nodes', strands_navigation_msgs.srv.GetEdgesBetweenNodes, self.get_edges_between_cb)

        # Services that modify the map
        self.write_map_srv=rospy.Service('/topological_map_manager2/write_topological_map', topological_navigation_msgs.srv.WriteTopologicalMap, self.write_topological_map_cb)
        self.add_node_srv=rospy.Service('/topological_map_manager2/add_topological_node', strands_navigation_msgs.srv.AddNode, self.add_topological_node_cb)
        self.remove_node_srv=rospy.Service('/topological_map_manager2/remove_topological_node', strands_navigation_msgs.srv.RmvNode, self.remove_node_cb)
        self.add_edges_srv=rospy.Service('/topological_map_manager2/add_edges_between_nodes', strands_navigation_msgs.srv.AddEdge, self.add_edge_cb)
        self.remove_edge_srv=rospy.Service('/topological_map_manager2/remove_edge', strands_navigation_msgs.srv.AddEdge, self.remove_edge_cb)
        self.add_content_to_node_srv=rospy.Service('/topological_map_manager2/add_content_to_node', strands_navigation_msgs.srv.AddContent, self.add_content_cb)
        self.update_node_name_srv = rospy.Service("/topological_map_manager2/update_node_name", strands_navigation_msgs.srv.UpdateNodeName, self.update_node_name_cb)
        self.update_node_waypoint_srv = rospy.Service("/topological_map_manager2/update_node_pose", strands_navigation_msgs.srv.AddNode, self.update_node_waypoint_cb)
        self.update_node_tolerance_srv = rospy.Service("/topological_map_manager2/update_node_tolerance", strands_navigation_msgs.srv.UpdateNodeTolerance, self.update_node_tolerance_cb)
        self.modify_tag_srv=rospy.Service('/topological_map_manager2/modify_node_tags', strands_navigation_msgs.srv.ModifyTag, self.modify_tag_cb)
        self.add_tag_srv=rospy.Service('/topological_map_manager2/add_tag_to_node', strands_navigation_msgs.srv.AddTag, self.add_tag_cb)
        
        
    def get_time(self):
        return datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


    def load_map(self, filename):
        
        with open(filename, "r") as f:
            try:
                self.tmap2 = yaml.safe_load(f)
            except Exception as e:
                rospy.logerr(e)
                self.tmap2 = {}
                return
        
        map_type = type(self.tmap2)
        if map_type is list:
            e = "Loaded map is {}. You may be attemting to load an old-format map using topological_navigation/manager2.py" \
                " Please use topological_navigation/manager.py instead.".format(map_type)
            rospy.logerr(e)
            self.tmap2 = {}
            return
                
        self.name = self.tmap2["name"]
        self.metric_map = self.tmap2["metric_map"]
        self.pointset = self.tmap2["pointset"]
        self.transformation = self.tmap2["transformation"]
        
        self.map_check()
        
        
    def update(self, update_time=True):
        
        if update_time:
            self.tmap2["meta"]["last_updated"] = self.get_time()
        
        self.map_pub.publish(std_msgs.msg.String(json.dumps(self.tmap2)))
        self.names = self.create_list_of_nodes()
        
        
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
        
        
    def create_list_of_nodes(self):
        
        names = []
        if "nodes" in self.tmap2:
            names = [node["node"]["name"] for node in self.tmap2["nodes"]]
            names.sort()
            
        return names
        
        
    def get_topological_map_cb(self, req):
        """
        Returns the topological map
        """
        ans = TriggerResponse()
        ans.success = True
        ans.message = json.dumps(self.tmap2)
        
        return ans
    
    
    def switch_topological_map_cb(self, req):
        """
        Changes the topological map
        """
        self.load_map(req.filename)        
        self.update(False)
        self.broadcast_transform()
        
        success = True
        if not self.tmap2:
            success = False
            
        return success, json.dumps(self.tmap2)
    
    
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
        tags = [tag for node in self.tmap2["nodes"] if "tag" in node["meta"] for tag in node["meta"]["tag"]]
        return [set(tags)]
    
    
    def get_node_tags_cb(self, req):
        """
        Returns a list of a node's tags
        """
        num_available = 0
        for node in self.tmap2["nodes"]:
            if node["node"]["name"] == req.node_name:
                if "tag" in node["meta"]:
                    tags = node["meta"]["tag"]
                else:
                    tags = []
                    
                num_available+=1
                
        succeded = True        
        if num_available != 1:
            succeded = False
            tags = []
            
        return succeded, tags
    
    
    def get_edges_between(self, nodea, nodeb):
        
         ab=[]; ba=[]
         for node in self.tmap2["nodes"]:
             if nodea == node["node"]["name"]:
                 for edge in node["node"]["edges"]:
                     if edge["node"] == nodeb:
                         ab.append(edge["edge_id"])
             if nodeb == node["node"]["name"]:
                 for edge in node["node"]["edges"]:
                     if edge["node"] == nodea:
                         ba.append(edge["edge_id"])
                         
         return ab, ba
     

    def get_edges_between_cb(self, req):
        """
        Returns a list of the ids of edges from nodea to nodeb and vice-versa
        """
        return self.get_edges_between(req.nodea, req.nodeb)
    
    
    def write_topological_map_cb(self, req):
        """
        Saves the topological map to a yaml file
        """
        filename = req.filename
        if not filename:
            filename = self.filename
        
        try:
            message = "Writing map to {}".format(filename)
            self.write_topological_map(filename)
            success = True
        except Exception as message:
            success = False
        
        return success, str(message)
        
    
    def write_topological_map(self, filename):
        
        rospy.loginfo("Writing map to {}".format(filename))
        
        nodes = copy.deepcopy(self.tmap2["nodes"])
        nodes.sort(key=lambda node: node["node"]["name"])
        self.tmap2["nodes"] = nodes
        
        yml = yaml.safe_dump(self.tmap2, default_flow_style=False)
        fh = open(filename, "w")
        fh.write(str(yml))
        fh.close
        
        
    def add_topological_node_cb(self, req):
        """
        Adds a node to the topological map
        """
        return self.add_topological_node(req.name, req.pose, req.add_close_nodes)
        
        
    def add_topological_node(self, node_name, node_pose, add_close_nodes, dist=8.0):
        
        if node_name:
            name = node_name
        else:
            name = self.get_new_name()

        rospy.loginfo("Creating Node {}: ".format(name))

        if name in self.names:
            rospy.logerr("Node already exists, try another name")
            return False
        
        pose = message_converter.convert_ros_message_to_dictionary(node_pose)
        close_nodes = []
        if add_close_nodes:
            for node in self.tmap2["nodes"]:
                ndist = pose_dist(pose, node["node"]["pose"])
                if ndist < dist :
                    if node["node"]["name"] != "ChargingPoint":
                        close_nodes.append(node["node"]["name"])
                        
        self.add_node(name, pose)
        
        for close_node in close_nodes:
            self.add_edge(name, close_node, "move_base", "", False)
            self.add_edge(close_node, name, "move_base", "", False)

        self.update()
        self.write_topological_map(self.filename)

        return True
    
    
    def get_new_name(self):
        
        namesnum=[]
        for i in self.names :
            if i.startswith('WayPoint') :
                nam = i.strip('WayPoint')
                namesnum.append(int(nam))
        namesnum.sort()
        if namesnum:
            nodname = 'WayPoint%d'%(int(namesnum[-1])+1)
        else :
            nodname = 'WayPoint1'
            
        return nodname
        
        
    def add_node(self, name, pose, localise_by_topic="", verts="default", properties="default", tags=[], restrictions=None):
        
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
            node["node"]["verts"] = self.generate_circle_vertices()
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
        
        
    def generate_circle_vertices(self, radius=0.75, number=8):
        
        separation_angle = 2 * math.pi / number
        start_angle = separation_angle / 2
        current_angle = start_angle
        points = []
        for i in range(0, number):
            points.append({"x": math.cos(current_angle) * radius, "y": math.sin(current_angle) * radius})
            current_angle += separation_angle

        return points
    
    
    def add_edge_cb(self, req):
        """
        Adds an edge to a topological node
        """
        return self.add_edge(req.origin, req.destination, req.action, req.edge_id)
    
    
    def add_edge(self, origin, destination, action, edge_id, update=True):
        
        rospy.loginfo("Adding Edge from {} to {} using {}".format(origin, destination, action))
        
        num_available, index = self.get_instances_of_node(origin)
        
        if num_available == 1 :
            eids = []
            for edge in self.tmap2["nodes"][index]["node"]["edges"]:
                eids.append(edge["edge_id"])

            if not edge_id or edge_id in eids:
                test=0
                eid = '%s_%s' %(origin, destination)
                while eid in eids:
                    eid = '%s_%s_%03d' %(origin, destination, test)
                    test += 1
            else:
                eid=edge_id
                
            self.add_edge_to_node(origin, destination, action, eid)
            
            if update:
                self.update()
                self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("Error adding edge to node {}. {} instances of node with name {} found".format(origin, num_available, origin))
            return False
        
        
    def add_edge_to_node(self, origin, destination, action="move_base", edge_id="default", config="default", 
                         action_type="default", goal="default", fail_policy="fail", restrictions=None):
        
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
            if node["node"]["name"] == origin:
                node["node"]["edges"].append(edge)
                
                
    def set_action_type(self, action):
        
        package = action + "_msgs"
        items = [item[0].upper() + item[1:] for item in action.split("_")]
        goal_type = "".join(items) + "Goal"
        action_type = package + "/" + goal_type
            
        return action_type
                
                
    def remove_node_cb(self, req):
        response = self.remove_node(req.name)
        return response
                
                
    def remove_node(self, node_name):
        
        rospy.loginfo("Removing Node: ".format(node_name))
        
        num_available, rm_id = self.get_instances_of_node(node_name)
        
        if num_available == 1:
            del self.tmap2["nodes"][rm_id]
            
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if edge["node"] == node_name:
                        print "removing edge {}".format(edge["edge_id"])
                        self.remove_edge(edge["edge_id"], False)
            
            self.update()
            self.write_topological_map(self.filename)            
            return True
        else:
            rospy.logerr("Error removing node {}. {} instances of node with name {} found".format(node_name, num_available, node_name))
            return False
        
        
    def remove_edge_cb(self, req):
        return self.remove_edge(req.edge_id)
      

    def remove_edge(self, edge_name, update=True):
        
        rospy.loginfo("Removing Edge: ".format(edge_name))
        
        num_available = 0
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["edge_id"] == edge_name:
                    num_available+=1

        if num_available >= 1:
            for node in self.tmap2["nodes"]:
                edges = copy.deepcopy(node["node"]["edges"])
                edges_new = list(filter(lambda edge: edge["edge_id"] != edge_name, edges))
                node["node"]["edges"] = edges_new
                
            if update:
                self.update()
                self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("No edges with id {} found".format(edge_name))
            return False
    
    
    def add_content_cb(self, req):
        
        data = json.loads(req.content)
        
        num_available, index = self.get_instances_of_node(req.node)        
        
        if num_available != 1:
             succeded = False
             meta_out = None
             print "there are no nodes or more than 1 with that name"
        else:
            succeded = True
            node_meta = self.tmap2["nodes"][index]["meta"]
            if "contains" in node_meta:
                if type(data) is list:
                    for j in data:
                        if "category" in j and "name" in j:
                            node_meta['contains'].append(j)
                elif type(data) is dict:
                    if "category" in data and "name" in data:
                        node_meta["contains"].append(data)
            else:
                a=[]
                if type(data) is list:
                    for j in data:
                        if "category" in j and "name" in j:
                            a.append(j)
                elif type(data) is dict:
                    if "category" in data and "name" in data:
                        a.append(data)
                node_meta["contains"] = a
            meta_out = str(node_meta)
            
            print "Updating %s--%s" %(self.tmap2["name"], req.node)
            self.update()
            self.write_topological_map(self.filename)

        return succeded, meta_out
    
    
    def update_node_name_cb(self, req):
        return self.update_node_name(req.node_name, req.new_name)
      

    def update_node_name(self, node_name, new_name):
        if new_name in self.names:
            return False, "node with name {0} already exists".format(new_name)

        num_available, index = self.get_instances_of_node(node_name)
                
        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            the_node["meta"]["node"] = new_name
            the_node["node"]["name"] = new_name
            self.tmap2["nodes"][index] = the_node
            
            # update all the edges which involve the renamed node
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if edge["node"] == node_name:
                        edge["node"] = new_name
                        edge["edge_id"] = edge["edge_id"].replace(node_name, new_name)
             
            self.update()
            self.write_topological_map(self.filename)
            return True, ""
        else:
            return False, "Multiple nodes with name {} found, or it does not exist".format(node_name)
        
        
    def update_node_waypoint_cb(self, req):
        return self.update_node_waypoint(req.name, req.pose)
      

    def update_node_waypoint(self, name, pose_msg):
        
        num_available, index = self.get_instances_of_node(name)
        
        if num_available == 1:
            pose = message_converter.convert_ros_message_to_dictionary(pose_msg)
            self.tmap2["nodes"][index]["node"]["pose"] = pose
        
            self.update()
            self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("Error updating the pose of node {}. {} instances of node with name {} found".format(name, num_available, name))
            return False
        
        
    def update_node_tolerance_cb(self, req):
        return self.update_node_tolerance(req.node_name, req.xy_tolerance, req.yaw_tolerance)
      

    def update_node_tolerance(self, name, new_xy, new_yaw):
        
        num_available, index = self.get_instances_of_node(name)
        
        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            
            if "properties" in the_node["node"]:
                the_node["node"]["properties"]["xy_goal_tolerance"] = new_xy
                the_node["node"]["properties"]["yaw_goal_tolerance"] = new_yaw
            else:
                properties = {}
                properties["xy_goal_tolerance"] = new_xy
                properties["yaw_goal_tolerance"] = new_yaw
                the_node["node"]["properties"] = properties
                
            self.tmap2["nodes"][index] = the_node  
            
            self.update()
            self.write_topological_map(self.filename)
            return True, ""
        else:
            rospy.logerr("Error updating the tolerance of node {}. {} instances of node with name {} found".format(name, num_available, name))
            return False, ""
        
        
    def modify_tag_cb(self, msg):
        
        succeded = True
        meta_out = None
        for node_name in msg.node:
            available = [i for i, node in enumerate(self.tmap2["nodes"]) if node["node"]["name"] == node_name]
            
            for i in available:
                meta = self.tmap2["nodes"][i]["meta"]
                if "tag" in meta:
                    if not msg.tag in meta["tag"]:
                        continue
                    else:
                        tag_ind = meta["tag"].index(msg.tag)
                        meta["tag"][tag_ind] = msg.new_tag
                        
                meta_out = str(meta)

            if len(available) == 0:
                 succeded = False
                 
        if succeded:
            self.update()
            self.write_topological_map(self.filename)

        return succeded, meta_out
    
    
    def add_tag_cb(self, msg):
        """
        Adds a tag to nodes in the map
        """
        succeded = False
        meta_out = None
        for j in msg.node:
            for node in self.tmap2["nodes"]:
                if j == node["node"]["name"]:
                    succeded = True
                    if "tag" in node["meta"]:
                        if msg.tag not in node["meta"]["tag"]:
                            node["meta"]["tag"].append(msg.tag)
                    else:
                        a = []
                        a.append(msg.tag)
                        node["meta"][ "tag"] = a
                    meta_out = str(node["meta"])
                    
        if succeded:
            self.update()
            self.write_topological_map(self.filename)
                    
        return succeded, meta_out
        
        
    def get_instances_of_node(self, node_name):
        
        num_available = 0
        index = None
        for i, node in enumerate(self.tmap2["nodes"]):
            if node["node"]["name"] == node_name:
                num_available+=1
                index = i
                
        return num_available, index
                
        
    def map_check(self):
        
        self.map_ok = True
        
        # check that all nodes have the same pointset
        pointsets = [node["meta"]["pointset"] for node in self.tmap2["nodes"]]
        if len(set(pointsets)) > 1:
            rospy.logwarn("multiple poinsets found in meta info: {}".format(set(pointsets)))
            self.map_ok = False
        
        # check for duplicate node names
        print "\n"
        names = self.create_list_of_nodes()
        for name in set(names):
            n = names.count(name)
            if n > 1:
                rospy.logwarn("{} instances of node with name '{}' found".format(n, name))
                self.map_ok = False
        
        sep = "_" + str(uuid.uuid4()) + "_"
        edge_ids = [node["node"]["name"] + sep + edge["node"] for node in self.tmap2["nodes"] for edge in node["node"]["edges"]]
        edge_ids.sort()

        # check for duplicate edges
        print "\n"
        for edge in set(edge_ids):
            edge_nodes = re.match("(.*)" + sep + "(.*)", edge).groups()
            origin = edge_nodes[0]
            destination = edge_nodes[1]
 
            n = edge_ids.count(edge)
            if n > 1:
                rospy.logwarn("{} instances of edge with origin '{}' and destination '{}' found".format(n, origin, destination))
                self.map_ok = False
        
        # check that an edge's destination node exists
        print "\n"         
        for edge in set(edge_ids):
            edge_nodes = re.match("(.*)" + sep + "(.*)", edge).groups()
            origin = edge_nodes[0]
            destination = edge_nodes[1]
 
            if destination not in names:
                rospy.logwarn("edge with origin '{}' has a destination '{}' that does not exist".format(origin, destination))
                self.map_ok = False
#########################################################################################################