 #!/usr/bin/env python3
"""
Created on Tue Sep 29 16:06:36 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import yaml, datetime, json
import re, uuid, copy, os
import multiprocessing, math
import importlib
import traceback

import rclpy
from rclpy.parameter import Parameter
import rosidl_runtime_py
import tf2_ros
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from topological_navigation_msgs.msg import *
import topological_navigation_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Pose

from std_srvs.srv import Trigger, Empty
import topological_navigation_msgs.srv as tn_srv

import topological_navigation
from topological_navigation.tmap_utils import get_node_names_from_edge_id_2


def pose_dist(pose1, pose2):
    return math.sqrt((pose1["position"]["x"] - pose2["position"]["x"])**2 + (pose1["position"]["y"] - pose2["position"]["y"])**2)

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True
#########################################################################################################


#########################################################################################################
class map_manager_2(rclpy.node.Node):

    def __init__(self, advertise_srvs=True):
        super().__init__('topological_map_manager_2')

        self.cache_maps = self.get_parameter_or("~cache_topological_maps", Parameter('bool', Parameter.Type.BOOL, False)).value
        self.auto_write = self.get_parameter_or("~auto_write_topological_maps", Parameter('bool', Parameter.Type.BOOL, False)).value
        
        package_path = get_package_share_directory('topological_navigation')
        nav_config = str(os.path.join(package_path, 'config', 'navigation_goal.yaml'))

        self.nav_config = str(self.get_parameter_or("nav_config", Parameter('str', Parameter.Type.STRING, nav_config)).value)
        self.get_logger().info("cache_topological_maps: {}".format(self.cache_maps))
        self.get_logger().info("auto_write_topological_maps: {}".format(self.auto_write))
        self.get_logger().info("nav config file: {}".format(self.nav_config))

        self.cache_dir = os.path.join(os.path.expanduser("~"), ".ros", "topological_maps")
        if not os.path.exists(self.cache_dir):
            os.mkdir(self.cache_dir)
	
        self.goal_mappings = {}
        
        with open(self.nav_config, "r") as f:
            self.move_base_goal = yaml.safe_load(f)["topological_navigation/navigation_goal"]

        if advertise_srvs:
            self.advertise()


    def advertise(self):

        # Services that retrieve information from the map
        self.get_map_srv = self.create_service(Trigger, '/topological_map_manager2/get_topological_map', self.get_topological_map_cb)

        self.get_tagged_srv = self.create_service(tn_srv.GetTaggedNodes, '/topological_map_manager2/get_tagged_nodes', self.get_tagged_cb)
        self.get_tag_srv = self.create_service(tn_srv.GetTags, '/topological_map_manager2/get_tags', self.get_tags_cb)
        self.get_node_tag_srv = self.create_service(tn_srv.GetNodeTags, '/topological_map_manager2/get_node_tags', self.get_node_tags_cb)
        self.get_node_edges_srv = self.create_service(tn_srv.GetEdgesBetweenNodes, '/topological_map_manager2/get_edges_between_nodes', self.get_edges_between_cb)

        # Services that modify the map
        self.write_map_srv = self.create_service(tn_srv.WriteTopologicalMap, '/topological_map_manager2/write_topological_map', self.write_topological_map_cb)
        self.switch_map_srv = self.create_service(tn_srv.WriteTopologicalMap, '/topological_map_manager2/switch_topological_map', self.switch_topological_map_cb)

        self.add_node_srv = self.create_service(tn_srv.AddNode, '/topological_map_manager2/add_topological_node', self.add_topological_node_cb)
        self.remove_node_srv = self.create_service(tn_srv.RmvNode, '/topological_map_manager2/remove_topological_node', self.remove_node_cb)

        self.add_edges_srv = self.create_service(tn_srv.AddEdge, '/topological_map_manager2/add_edges_between_nodes', self.add_edge_cb)
        self.remove_edge_srv = self.create_service(tn_srv.AddEdge, '/topological_map_manager2/remove_edge', self.remove_edge_cb)

        self.add_content_to_node_srv = self.create_service(tn_srv.AddContent, '/topological_map_manager2/add_content_to_node', self.add_content_cb)

        self.update_node_name_srv = self.create_service(tn_srv.UpdateNodeName, "/topological_map_manager2/update_node_name", self.update_node_name_cb)
        self.update_node_waypoint_srv = self.create_service(tn_srv.AddNode, "/topological_map_manager2/update_node_pose", self.update_node_waypoint_cb)
        self.update_node_tolerance_srv = self.create_service(tn_srv.UpdateNodeTolerance, "/topological_map_manager2/update_node_tolerance", self.update_node_tolerance_cb)

        self.modify_tag_srv = self.create_service(tn_srv.ModifyTag, '/topological_map_manager2/modify_node_tags', self.modify_tag_cb)
        self.add_tag_srv = self.create_service(tn_srv.AddTag, '/topological_map_manager2/add_tag_to_node', self.add_tag_cb)
        self.rm_tag_srv = self.create_service(tn_srv.AddTag, '/topological_map_manager2/rm_tag_from_node', self.rm_tag_cb)

        self.add_param_to_edge_config_srv = self.create_service(tn_srv.UpdateEdgeConfig, '/topological_map_manager2/add_param_to_edge_config', self.add_param_to_edge_config_cb)
        self.rm_param_from_edge_config_srv = self.create_service(tn_srv.UpdateEdgeConfig, '/topological_map_manager2/rm_param_from_edge_config', self.rm_param_from_edge_config_cb)
        self.rm_param_from_topological_map_srv = self.create_service(tn_srv.UpdateEdgeConfig, '/topological_map_manager2/rm_param_from_topological_map', self.rm_param_from_topological_map_cb)

        self.update_node_restrictions_srv = self.create_service(tn_srv.UpdateRestrictions, '/topological_map_manager2/update_node_restrictions', self.update_node_restrictions_cb)
        self.update_edge_restrictions_srv = self.create_service(tn_srv.UpdateRestrictions, '/topological_map_manager2/update_edge_restrictions', self.update_edge_restrictions_cb)

        self.update_edge_srv = self.create_service(tn_srv.UpdateEdge, '/topological_map_manager2/update_edge', self.update_edge_cb)
        self.update_action_srv = self.create_service(tn_srv.UpdateAction, '/topological_map_manager2/update_action', self.update_action_cb)

        self.add_datum_srv = self.create_service(tn_srv.AddDatum, '/topological_map_manager2/add_datum', self.add_datum_cb)

        self.update_fail_policy_srv = self.create_service(tn_srv.UpdateFailPolicy, '/topological_map_manager2/update_fail_policy', self.update_fail_policy_cb)
        self.set_influence_zone_srv = self.create_service(tn_srv.SetInfluenceZone, '/topological_map_manager2/set_node_influence_zone', self.set_influence_zone_cb)

        self.clear_nodes_srv = self.create_service(Empty, '/topological_map_manager2/clear_topological_nodes', self.clear_nodes_cb)

        # Services for modifying the map quickly
        self.add_nodes_srv = self.create_service(tn_srv.AddNodeArray, '/topological_map_manager2/add_topological_node_multi', self.add_topological_nodes_cb)
        self.add_edges_srv = self.create_service(tn_srv.AddEdgeArray, '/topological_map_manager2/add_edges_between_nodes_multi', self.add_edges_cb)
        self.add_params_to_edges_srv = self.create_service(tn_srv.UpdateEdgeConfigArray, '/topological_map_manager2/add_param_to_edge_config_multi', self.add_params_to_edges_cb)
        self.set_influence_zones_srv = self.create_service(tn_srv.SetInfluenceZoneArray, '/topological_map_manager2/set_node_influence_zone_multi', self.set_influence_zones_cb)


    def init_map(self, name="new_map", metric_map="map_2d", pointset="new_map", transformation="default", filename="", load=True):
    
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
        if not self.filename:
            self.filename = os.path.join(self.cache_dir, self.name + ".tmap2")

        self.loaded = False
        self.load = load
        if self.load:
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
            self.declare_parameter('topological_map2_name', self.pointset)
            self.declare_parameter('topological_map2_filename', os.path.split(self.filename)[1])
            self.declare_parameter('topological_map2_path', os.path.split(self.filename)[0])


        qos = QoSProfile(depth=10, 
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(std_msgs.msg.String, '/topological_map_2', qos)
        self.map_pub.publish(std_msgs.msg.String(data=json.dumps(self.tmap2)))
        self.names = self.create_list_of_nodes()

        self.broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
        self.broadcast_transform()

        self.convert_to_legacy = self.get_parameter_or("~convert_to_legacy", Parameter('bool', Parameter.Type.BOOL, True)).get_parameter_value()
        
        if self.tmap2 and self.convert_to_legacy:
            self.points_pub = self.create_publisher(topological_navigation_msgs.msg.TopologicalMap, '/topological_map', qos)
            self.tmap2_to_tmap()
            self.points_pub.publish(self.points)

        # self.create_timer(10.0, self.topnav_map_pub_callback)


    # def topnav_map_pub_callback(self, ):
    #     if self.tmap2: 
    #         self.map_pub.publish(std_msgs.msg.String(data=json.dumps(self.tmap2)))
    #     else:
    #         self.get_logger().warning('there is no topological map...', skip_first=True)

    def get_time(self):
        return datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


    def load_map(self, filename):

        def loader(filename, transporter):
            try:
                with open(filename, "r") as f:
                    transporter["tmap2"] = yaml.safe_load(f)
            except Exception as e:
                self.get_logger().error(e)
                transporter["tmap2"] = {}


        self.loaded = False
        self.get_logger().info("Loading Topological Map {} ...".format(filename))

        transporter = multiprocessing.Manager().dict()
        p = multiprocessing.Process(target=loader, args=(filename, transporter))
        p.start()
        p.join()

        self.tmap2 = transporter["tmap2"]
        if not self.tmap2:
            return

        e1 = "Loaded map is {} and should be {}."
        e2 = " You may be attemting to load a legacy map using topological_navigation/map_manager2.py." \
                " In that case please use topological_navigation/map_manager.py instead."

        map_type = type(self.tmap2)
        if map_type is list:
            self.get_logger().error((e1+e2).format(map_type, dict))
            self.tmap2 = {}
            return
        elif map_type is not dict:
            self.get_logger().error(e1.format(map_type, dict))
            self.tmap2 = {}
            return

        self.loaded = True

        self.name = self.tmap2["name"]
        self.metric_map = self.tmap2["metric_map"]
        self.pointset = self.tmap2["pointset"]
        self.transformation = self.tmap2["transformation"]

        self.names = self.create_list_of_nodes()

        self.declare_parameter('topological_map2_name', self.pointset)
        self.declare_parameter('topological_map2_filename', os.path.split(self.filename)[1])
        self.declare_parameter('topological_map2_path', os.path.split(self.filename)[0])

        self.get_logger().info("Done")

        self.map_check()

        if self.cache_maps:
            self.get_logger().info("Caching the map...")
            self.write_topological_map(os.path.join(self.cache_dir, os.path.basename(self.filename)))


    def write_topological_map(self, filename, no_alias=False):

        self.get_logger().info("Writing map to {} ...".format(filename))

        nodes = copy.deepcopy(self.tmap2["nodes"])
        nodes.sort(key=lambda node: node["node"]["name"])
        self.tmap2["nodes"] = nodes

        if no_alias:
            self.get_logger().info("Disabling anchors and aliases in topological map yaml ...")
            yml = yaml.dump(self.tmap2, default_flow_style=False, Dumper=NoAliasDumper)
        else:
            yml = yaml.safe_dump(self.tmap2, default_flow_style=False)

        fh = open(filename, "w")
        fh.write(str(yml))
        fh.close

        self.get_logger().info("Done")


    def update(self, update_time=True):

        if update_time:
            self.tmap2["meta"]["last_updated"] = self.get_time()

        self.map_pub.publish(std_msgs.msg.String(json.dumps(self.tmap2)))
        self.names = self.create_list_of_nodes()
        self.map_check()

        if self.tmap2 and self.convert_to_legacy:
            self.tmap2_to_tmap()
            self.points_pub.publish(self.points)


    def broadcast_transform(self):

        trans, rot = Vector3(), Quaternion()
        rosidl_runtime_py.set_message_fields(trans, self.transformation["translation"])
        rosidl_runtime_py.set_message_fields(rot, self.transformation["rotation"])

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        ans = Trigger.response()
        ans.success = True
        ans.message = json.dumps(self.tmap2)

        return ans


    def switch_topological_map_cb(self, req):
        """
        Changes the topological map
        """
        self.declare_parameter('topological_map2_filename', req.filename)
        path = self.get_parameter("topological_map2_path").get_parameter_value()
        self.filename = path + "/" + req.filename

        self.load_map(self.filename)
        self.update(False)
        self.broadcast_transform()

        return True, json.dumps(self.tmap2)


    def get_tagged_cb(self, req, res):
        """
        Returns a list of nodes that have a given tag
        """
        res.nodes=[]
        for node in self.tmap2["nodes"]:
            if "tag" in node["meta"]:
                if req.tag in node["meta"]["tag"]:
                    res.nodes.append(node["node"]["name"])
        return res


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


    def get_edges_between_cb(self, req):
        """
        Returns a list of the ids of edges from nodea to nodeb and vice-versa
        """
        return self.get_edges_between(req.nodea, req.nodeb)


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


    def write_topological_map_cb(self, req):
        """
        Saves the topological map to a yaml file
        """
        filename = req.filename
        if not filename:
            path = self.get_parameter("topological_map2_path").get_parameter_value()
            fname = self.get_parameter_or("topological_map2_filename").get_parameter_value()
            filename = path + "/" + fname

        try:
            message = "Writing map to {}".format(filename)
            self.write_topological_map(filename, req.no_alias)
            success = True
        except Exception as message:
            success = False

        return success, str(message)


    def add_topological_node_cb(self, req):
        """
        Adds a node to the topological map
        """
        return self.add_topological_node(req.name, req.pose, req.add_close_nodes)


    def add_topological_node(self, node_name, node_pose, add_close_nodes, dist=8.0, update=True, write_map=True):

        if node_name:
            name = node_name
        else:
            name = self.get_new_name()

        self.get_logger().info("Creating Node {}".format(name))

        if name in self.names:
            self.get_logger().error("Node {} already exists, try another name".format(name))
            return False

        pose = rosidl_runtime_py.message_to_ordereddict(node_pose)

        close_nodes = []
        if add_close_nodes:
            for node in self.tmap2["nodes"]:
                ndist = pose_dist(pose, node["node"]["pose"])
                if ndist < dist :
                    if node["node"]["name"] != "ChargingPoint":
                        close_nodes.append(node["node"]["name"])

        self.add_node(name, pose)

        for close_node in close_nodes:
            self.add_edge(name, close_node, "move_base", "", update=False, write_map=False)
            self.add_edge(close_node, name, "move_base", "", update=False, write_map=False)

        if update:
            self.update()
        if self.auto_write and write_map:
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


    def add_node(self, name, pose, localise_by_topic="", verts="default", properties="default", tags=[],
                 restrictions_planning="True", restrictions_runtime="True"):

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

        node["node"]["restrictions_planning"] = restrictions_planning
        node["node"]["restrictions_runtime"] = restrictions_runtime

        node["node"]["parent_frame"] = self.transformation["parent"]

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
        return self.add_edge(req.origin, req.destination, req.action, req.action_type, req.edge_id)


    def add_edge(self, origin, destination, action, action_type, edge_id, update=True, write_map=True):

        self.get_logger().info("Adding Edge from {} to {} using {}".format(origin, destination, action))

        num_available, index = self.get_instances_of_node(origin)

        if num_available == 1 :
            eids = []
            for edge in self.tmap2["nodes"][index]["node"]["edges"]:
                eids.append(edge["edge_id"])

                if edge_id == edge["edge_id"]:
                    self.get_logger().error("Error adding edge to node {}. Edge already exists.".format(origin))
                    return False

            if not edge_id or edge_id in eids:
                test=0
                eid = '%s_%s' %(origin, destination)
                while eid in eids:
                    eid = '%s_%s_%03d' %(origin, destination, test)
                    test += 1
            else:
                eid=edge_id

            self.add_edge_to_node(origin, destination, action, eid, action_type=action_type)

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            self.get_logger().error("Error adding edge to node {}. {} instances of node with name {} found".format(origin, num_available, origin))
            return False


    def add_edge_to_node(self, origin, destination, action="", edge_id="default", config=[],
                         recovery_behaviours_config="", action_type="", goal=None, fail_policy="fail",
                         restrictions_planning="True", restrictions_runtime="True", fluid_navigation=True):

        edge = {}
        edge["action"] = action


        if edge_id == "default":
            edge["edge_id"] = origin + "_" + destination
        else:
            edge["edge_id"] = edge_id

        edge["node"] = destination
        edge["config"] = config
        edge["recovery_behaviours_config"] = recovery_behaviours_config

        if not action_type:
            action_type = "move_base_msgs/MoveBaseGoal"

        the_action_type, the_goal = self.set_goal(action, action_type, goal)

        edge["action_type"] = the_action_type
        edge["goal"] = the_goal

        edge["fail_policy"] = fail_policy
        edge["restrictions_planning"] = restrictions_planning
        edge["restrictions_runtime"] = restrictions_runtime
        edge["fluid_navigation"] = fluid_navigation

        for node in self.tmap2["nodes"]:
            if node["node"]["name"] == origin:
                node["node"]["edges"].append(edge)


    def set_goal(self, action, action_type, _goal=None):

        if action in self.goal_mappings and action_type == self.goal_mappings[action]["action_type"]:
            goal = self.goal_mappings[action]["goal"]
        else:
            if _goal is not None:
                goal = _goal
            else:
                try:
                    package = action_type.split("/")[0]
                    goal_def = action_type.split("/")[1]

                    _file = self.get_parameter(f"~{action_type}", Parameter('str', Parameter.Type.STRING, "")).get_parameter_value()
                    if not _file:
                        package_object = importlib.import_module(package)
                        _file = os.path.join(package_object.__path__[0], '..', 'config', f"{goal_def}.yaml")
                    with open(_file, "r") as f:
                        goal = yaml.safe_load(f)
                except:
                    action_type = self.move_base_goal["action_type"]
                    goal = self.move_base_goal["goal"]

            self.goal_mappings[action] = {"action_type": action_type, "goal": goal}

        return action_type, goal


    def set_action_type(self, action):

        package = action + "_msgs"
        items = [item[0].upper() + item[1:] for item in action.split("_")]
        goal_type = "".join(items) + "Goal"
        action_type = package + "/" + goal_type

        return action_type


    def remove_node_cb(self, req):
        """
        Removes a node from the topological map
        """
        return self.remove_node(req.name)


    def remove_node(self, node_name, update=True, write_map=True):

        self.get_logger().info("Removing Node {}".format(node_name))

        num_available, rm_id = self.get_instances_of_node(node_name)

        if num_available == 1:
            del self.tmap2["nodes"][rm_id]

            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if edge["node"] == node_name:
                        self.remove_edge(edge["edge_id"], False)

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            self.get_logger().error("Error removing node {}. {} instances of node with name {} found".format(node_name, num_available, node_name))
            return False


    def remove_edge_cb(self, req):
        """
        Removes an edge from a topological node
        """
        return self.remove_edge(req.edge_id)


    def remove_edge(self, edge_name, update=True, write_map=True):

        self.get_logger().info("Removing Edge {}".format(edge_name))

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
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            self.get_logger().error("No edges with id {} found".format(edge_name))
            return False


    def add_content_cb(self, req):
        """
        Adds content to a node's meta information
        """
        data = json.loads(req.content)

        num_available, index = self.get_instances_of_node(req.node)

        if num_available != 1:
             succeded = False
             meta_out = None
             self.get_logger().error("There are no nodes or more than one with name {}".format(req.node))
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

            self.get_logger().info("Updating %s--%s" %(self.tmap2["name"], req.node))
            self.update()
            if self.auto_write:
                self.write_topological_map(self.filename)

        return succeded, meta_out


    def update_node_name_cb(self, req):
        """
        Changes a node's name and updates edges which involve the renamed node
        """
        return self.update_node_name(req.node_name, req.new_name)


    def update_node_name(self, node_name, new_name, update=True, write_map=True):
        if new_name in self.names:
            return False, "node with name {0} already exists".format(new_name)

        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            # update all the edges which involve the renamed node
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if node["node"]["name"] == node_name:
                        edge["edge_id"] = new_name + "_" + edge["node"]
                    if edge["node"] == node_name:
                        edge["node"] = new_name
                        edge["edge_id"] = node["node"]["name"] + "_" + new_name

            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            the_node["meta"]["node"] = new_name
            the_node["node"]["name"] = new_name
            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            return False, "Multiple nodes with name {} found, or it does not exist".format(node_name)


    def update_node_waypoint_cb(self, req):
        """
        Updates a node's pose
        """
        return self.update_node_waypoint(req.name, req.pose)


    def update_node_waypoint(self, name, pose_msg, update=True, write_map=True):

        num_available, index = self.get_instances_of_node(name)

        if num_available == 1:
            pose = rosidl_runtime_py.message_to_ordereddict(pose_msg)
            
            self.tmap2["nodes"][index]["node"]["pose"] = pose

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            self.get_logger().error("Error updating the pose of node {}. {} instances of node with name {} found".format(name, num_available, name))
            return False


    def update_node_tolerance_cb(self, req):
        """
        Update node tolerances
        """
        return self.update_node_tolerance(req.node_name, req.xy_tolerance, req.yaw_tolerance)


    def update_node_tolerance(self, name, new_xy, new_yaw, update=True, write_map=True):

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

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            self.get_logger().error("Error updating the tolerance of node {}. {} instances of node with name {} found".format(name, num_available, name))
            return False, ""


    def modify_tag_cb(self, msg):
        """
        Changes the tag belonging to a node or a list of nodes
        """
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
            if self.auto_write:
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
            if self.auto_write:
                self.write_topological_map(self.filename)

        return succeded, meta_out


    def rm_tag_cb(self, msg):
        """
        Remove a tag from nodes in the map
        """
        succeded = True
        for node_name in msg.node:
            available = [i for i, node in enumerate(self.tmap2["nodes"]) if node["node"]["name"] == node_name]

            succeded = False
            meta_out = None
            for i in available:
                meta = self.tmap2["nodes"][i]["meta"]
                if "tag" in meta:
                    if msg.tag in meta["tag"]:
                        print('removing tag')
                        meta["tag"].remove(msg.tag)
                        print('new list of tags')
                        print(meta["tag"])
                        succeded = True
                meta_out = str(meta)

        if succeded:
            self.update()
            if self.auto_write:
                self.write_topological_map(self.filename)

        return succeded, meta_out


    def add_param_to_edge_config_cb(self, req):
        """
        Update edge reconfigure parameters.
        """
        return self.add_param_to_edge_config(req.edge_id, req.namespace, req.name, req.value, req.value_is_string, req.not_reset)


    def add_param_to_edge_config(self, edge_id, namespace, name, value, value_is_string, not_reset, update=True, write_map=True):

        if not value:
            return False, "no value provided"

        if not value_is_string:
            value = eval(value)

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        param = {"namespace":namespace, "name":name, "value":value, "reset":not not_reset}

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            msg = ""
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:

                    config = copy.deepcopy(edge["config"])
                    config = [i for i in config if not (i["namespace"] == param["namespace"] and i["name"] == param["name"])]

                    self.get_logger().info("Adding param {} to edge {}".format(param, edge["edge_id"]))
                    config.append(param)
                    edge["config"] = config

                    msg = "edge action is {} and edge config is {}".format(edge["action"], edge["config"])

            self.tmap2["nodes"][index] = the_node
            if update:
                self.update()
            if write_map and self.auto_write:
                self.write_topological_map(self.filename)

            return True, msg
        else:
            self.get_logger().error("Cannot update edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False, "No edge found or multiple edges found"


    def rm_param_from_edge_config_cb(self, req):
        """
        Remove a param from an edge's config.
        """
        return self.rm_param_from_edge_config(req.edge_id, req.namespace, req.name)


    def rm_param_from_edge_config(self, edge_id, namespace, name, update=True, write_map=True):

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            msg = ""
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:

                    config = copy.deepcopy(edge["config"])
                    config = [i for i in config if not (i["namespace"] == namespace and i["name"] == name)]

                    edge["config"] = config
                    msg = "edge action is {} and edge config is {}".format(edge["action"], edge["config"])

            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)

            return True, msg
        else:
            self.get_logger().error("Cannot update edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False, "No edge found or multiple edges found"


    def rm_param_from_topological_map_cb(self, req):
        """
        Remove all instances of a param from the topological map.
        """
        return self.rm_param_from_topological_map(req.namespace, req.name)


    def rm_param_from_topological_map(self, namespace, name, update=True, write_map=True):

        success = False
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                config0 = copy.deepcopy(edge["config"])
                config0 = [i for i in config0 if (i["namespace"] == namespace and i["name"] == name)]
                if config0:
                    success = True

                config = copy.deepcopy(edge["config"])
                config = [i for i in config if not (i["namespace"] == namespace and i["name"] == name)]
                edge["config"] = config

        if success:
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            return False, "parameter not found in topological map"


    def update_node_restrictions_cb(self, req):
        """
        Update a node's restrictions
        """
        return self.update_node_restrictions(req.name, req.restrictions_planning, req.restrictions_runtime, req.update_edges)


    def update_node_restrictions(self, node_name, restrictions_planning, restrictions_runtime, update_edges, update=True, write_map=True):

        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            if restrictions_planning:
                self.tmap2["nodes"][index]["node"]["restrictions_planning"] = restrictions_planning
            if restrictions_runtime:
                self.tmap2["nodes"][index]["node"]["restrictions_runtime"] = restrictions_runtime

            edge_ids = []
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if node["node"]["name"] == node_name or edge["node"] == node_name:
                        edge_ids.append(edge["edge_id"])

            if restrictions_planning and update_edges:
                for edge_id in set(edge_ids):
                    self.update_edge_restrictions(edge_id, restrictions_planning, "", False)

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            self.get_logger().error("Error updating the restrictions of node {}. {} instances of node with name {} found".format(node_name, num_available, node_name))
            return False, ""


    def update_edge_restrictions_cb(self, req):
        """
        Update an edge's restrictions
        """
        return self.update_edge_restrictions(req.name, req.restrictions_planning, req.restrictions_runtime)


    def update_edge_restrictions(self, edge_id, restrictions_planning, restrictions_runtime, update=True, write_map=True):

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:
                    if restrictions_planning:
                        edge["restrictions_planning"] = restrictions_planning
                    if restrictions_runtime:
                        edge["restrictions_runtime"] = restrictions_runtime

            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            self.get_logger().error("Error updating the restrictions of edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False, ""


    def update_edge_cb(self, req):
        """
        Updates an edge's args (action, action type, goal etc)
        """
        return self.update_edge(req.edge_id, req.action_name, req.action_type, req.goal, req.fail_policy, req.not_fluid)


    def update_edge(self, edge_id, action_name, action_type, goal, fail_policy, not_fluid, update=True, write_map=True):

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:
                    if action_name:
                        edge["action"] = action_name
                    if action_type:
                        edge["action_type"] = action_type
                    if goal:
                        edge["goal"] = json.loads(goal)
                    elif action_type and not goal:
                        _action_type, _goal = self.set_goal(action_name, action_type)
                        edge["action_type"] = _action_type
                        edge["goal"] = _goal
                    if fail_policy:
                        edge["fail_policy"] = fail_policy
                    if not_fluid:
                        edge["fluid_navigation"] = False
                    else:
                        edge["fluid_navigation"] = True

            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            self.get_logger().error("Cannot update edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False


    def update_action_cb(self, req):
        """
        Updates the action type and goal for all action_name edges
        """
        return self.update_action(req.action_name, req.action_type, req.goal)


    def update_action(self, action_name, action_type, goal, update=True, write_map=True):

        success = False
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["action"] == action_name:
                    if action_type:
                        edge["action_type"] = action_type
                    if goal:
                        edge["goal"] = json.loads(goal)
                    elif action_type and not goal:
                        _action_type, _goal = self.set_goal(action_name, action_type)
                        edge["action_type"] = _action_type
                        edge["goal"] = _goal
                    success = True

        if success:
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)

        return success


    def add_datum_cb(self, req):
        """
        Adds GNSS latitude/longitude to the topological map's top-level meta info
        """
        return self.add_datum(req.latitude, req.longitude)


    def add_datum(self, latitude, longitude, update=True, write_map=True):

        try:
            self.tmap2["meta"]["datum_latitude"] = latitude
            self.tmap2["meta"]["datum_longitude"] = longitude
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True

        except Exception as e:
            self.get_logger().error(e)
            return False


    def update_fail_policy_cb(self, req):
        """
        Update he fail policy of all edges in the map
        """
        return self.update_fail_policy(req.fail_policy)


    def update_fail_policy(self, fail_policy, update=True, write_map=True):

        if not fail_policy:
            return False

        try:
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    edge["fail_policy"] = fail_policy

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True

        except Exception as e:
            self.get_logger().error(e)
            return False


    def set_influence_zone_cb(self, req):
        """
        Set the influence zone (vertices) of a node
        """
        return self.set_influence_zone(req.name, req.vertices_x, req.vertices_y)


    def set_influence_zone(self, node_name, vertices_x, vertices_y, update=True, write_map=True):

        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:

            if len(vertices_x) < 3 or len(vertices_y) < 3 or len(vertices_x) != len(vertices_y):
                self.get_logger().error("Invalid node vertices")
                return False
            else:
                verts = [{"x":x, "y":y} for x, y in zip(vertices_x, vertices_y)]

            self.tmap2["nodes"][index]["node"]["verts"] = verts
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            self.get_logger().error("Error updating the influence zone of node {}. {} instances of node with name {} found".format(node_name, num_available, node_name))
            return False


    def clear_nodes_cb(self, req):
        """
        Remove all nodes from the topological map
        """
        self.clear_nodes()

        ans = Empty.response()
        return ans


    def clear_nodes(self, update=True, write_map=True):

        self.tmap2["nodes"] = []

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)


    def add_topological_nodes_cb(self, req):
        """
        Add a list of nodes to the topological map
        """
        return self.add_topological_nodes(req.data)


    def add_topological_nodes(self, data, update=True, write_map=True):

        for item in data:
            success = self.add_topological_node(item.name, item.pose, add_close_nodes=False, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def add_edges_cb(self, req):
        """
        Add a list of edges to the topological map
        """
        return self.add_edges(req.data)


    def add_edges(self, data, update=True, write_map=True):

        for item in data:
            success = self.add_edge(item.origin, item.destination, item.action, item.action_type, item.edge_id, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def add_params_to_edges_cb(self, req):
        """
        Add parameters to a list of edges
        """
        return self.add_params_to_edges(req.data)


    def add_params_to_edges(self, data, update=True, write_map=True):

        for item in data:
            success,_ = self.add_param_to_edge_config(item.edge_id, item.namespace, item.name, item.value, item.value_is_string, item.not_reset, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def set_influence_zones_cb(self, req):
        """
        Set the influence zone of a list of edges
        """
        return self.set_influence_zones(req.data)


    def set_influence_zones(self, data, update=True, write_map=True):

        for item in data:
            success = self.set_influence_zone(item.name, item.vertices_x, item.vertices_y, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


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
            self.get_logger().warn("Multiple poinsets found in meta info: {}".format(set(pointsets)))
            self.map_ok = False

        # check for duplicate node names
        names = self.create_list_of_nodes()
        for name in set(names):
            n = names.count(name)
            if n > 1:
                self.get_logger().warn("{} instances of node with name '{}' found".format(n, name))
                self.map_ok = False

        sep = "_" + str(uuid.uuid4()) + "_"
        edge_ids = [node["node"]["name"] + sep + edge["node"] for node in self.tmap2["nodes"] for edge in node["node"]["edges"]]
        edge_ids.sort()

        # check for duplicate edges
        for edge in set(edge_ids):
            edge_nodes = re.match("(.*)" + sep + "(.*)", edge).groups()
            origin = edge_nodes[0]
            destination = edge_nodes[1]

            n = edge_ids.count(edge)
            if n > 1:
                self.get_logger().warn("{} instances of edge with origin '{}' and destination '{}' found".format(n, origin, destination))
                self.map_ok = False

        # check that an edge's destination node exists
        for edge in set(edge_ids):
            edge_nodes = re.match("(.*)" + sep + "(.*)", edge).groups()
            origin = edge_nodes[0]
            destination = edge_nodes[1]

            if destination not in names:
                self.get_logger().warn("Edge with origin '{}' has a destination '{}' that does not exist".format(origin, destination))
                self.map_ok = False

        # check if a node has an edge to itself
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                if node["node"]["name"] == edge["node"]:
                    self.get_logger().warn("Edge with id '{}' has a destination '{}' equal to its origin".format(edge["edge_id"], edge["node"]))
                    self.map_ok = False


    def tmap2_to_tmap(self):
        self.points = map_manager_2.convert_tmap2_to_tmap(self.tmap2, self.pointset, self.metric_map)


    @classmethod
    def convert_tmap2_to_tmap(cls, tmap2, pointset, metric_map):
        points = topological_navigation_msgs.msg.TopologicalMap()

        try:
            point_set = pointset
            points.name = point_set
            points.pointset = point_set
            points.map = metric_map

            for node in tmap2["nodes"]:
                msg = topological_navigation_msgs.msg.TopologicalNode()
                msg.name = node["node"]["name"]
                msg.map = metric_map
                msg.pointset = point_set

                msg.pose = Pose()
                rosidl_runtime_py.set_message_fields(msg.pose, node["node"]["pose"])

                msg.yaw_goal_tolerance = node["node"]["properties"]["yaw_goal_tolerance"]
                msg.xy_goal_tolerance = node["node"]["properties"]["xy_goal_tolerance"]

                msgs_verts = []
                for v in node["node"]["verts"]:
                    msg_v = topological_navigation_msgs.msg.Vertex()
                    msg_v.x = v["x"]
                    msg_v.y = v["y"]
                    msgs_verts.append(msg_v)
                msg.verts = msgs_verts

                msgs_edges = []
                for e in node["node"]["edges"]:
                    msg_e = topological_navigation_msgs.msg.Edge()
                    msg_e.edge_id = e["edge_id"]
                    msg_e.node = e["node"]
                    msg_e.action = e["action"]
                    msg_e.top_vel = 0.55
                    msg_e.map_2d = metric_map
                    msg_e.inflation_radius = 0.0
                    msg_e.recovery_behaviours_config = e["recovery_behaviours_config"]
                    msgs_edges.append(msg_e)
                msg.edges = msgs_edges

                msg.localise_by_topic = node["node"]["localise_by_topic"]
                points.nodes.append(msg)

        except Exception as e:
            print(traceback.format_exc())
            print("Cannot convert map to the legacy format. The conversion requires all fields of the legacy map to be set.")
            points = topological_navigation_msgs.msg.TopologicalMap()

        return points
#########################################################################################################
