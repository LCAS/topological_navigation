#! /usr/bin/env python3

"""
    This node is used to manually create a topomap live using the robot driven using
    a controller, buttons are pressed to add/remove nodes and to generate
    the topomap.
    Original Author: Sergi Molina <sergi.molina@sagarobotics.com> 2022 - ROS1
    Maintainer: Ibrahim Hroob <ihroob@lincoln.ac.uk> 2024 - ROS2 support 
"""

import os
import yaml
import datetime
import numpy as np

from math import sqrt
from copy import deepcopy

import rclpy
import rclpy.duration

from rclpy import Parameter 
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from builtin_interfaces.msg import Duration

from tf_transformations import euler_from_quaternion

from ament_index_python.packages import get_package_share_directory


class RobotTmapping(Node):

    def __init__(self):
        super().__init__('manual_tmapping_node')

        # Params
        self.declare_parameter( 'tmap'            , Parameter.Type.STRING  )
        self.declare_parameter( 'tmap_dir'        , Parameter.Type.STRING  )
        self.declare_parameter( 'site_name'       , Parameter.Type.STRING  )
        self.declare_parameter( "insert_map"      , Parameter.Type.BOOL    )
        self.declare_parameter( 'inorder_cluster' , Parameter.Type.BOOL    )  #use_inorder_clustering
        self.declare_parameter( 'dbscan_eps'      , Parameter.Type.DOUBLE  )
        self.declare_parameter( 'node_thresh'     , Parameter.Type.DOUBLE  )  #closest_node_threshold
        self.declare_parameter( 'min_num_rows'    , Parameter.Type.INTEGER )
        # Joystick buttons 
        self.declare_parameter( 'lock_btn'        , Parameter.Type.INTEGER )  #lock_button_index
        self.declare_parameter( 'add_btn'         , Parameter.Type.INTEGER )  #add_node_button_index
        self.declare_parameter( 'remove_btn'      , Parameter.Type.INTEGER )  #remove_node_button_index
        self.declare_parameter( 'gen_map_btn'     , Parameter.Type.INTEGER )  #generate_tmap_button_index 
        # Topics 
        self.declare_parameter( 'topic_joy'       , Parameter.Type.STRING  )  #remove_node_button_index
        self.declare_parameter( 'topic_pose'      , Parameter.Type.STRING  )  #generate_tmap_button_index 
       

        self.pointset        = self.get_parameter_or('tmap'            , Parameter('str'   , Parameter.Type.STRING, '')  ).value
        self.tmap_dir        = self.get_parameter_or('tmap_dir'        , Parameter('str'   , Parameter.Type.STRING, '')  ).value
        self.site_name       = self.get_parameter_or('site_name'       , Parameter('str'   , Parameter.Type.STRING, '')  ).value
        self.insert_map      = self.get_parameter_or('insert_map'      , Parameter('bool'  , Parameter.Type.BOOL, False) ).value
        self.inorder_cluster = self.get_parameter_or('inorder_cluster' , Parameter('bool'  , Parameter.Type.BOOL, False) ).value
        self.dbscan_eps      = self.get_parameter_or('dbscan_eps'      , Parameter('double', Parameter.Type.DOUBLE, 4.0) ).value
        self.node_thresh     = self.get_parameter_or('node_thresh'     , Parameter('double', Parameter.Type.DOUBLE, 0.5) ).value
        self.min_num_rows    = self.get_parameter_or('min_num_rows'    , Parameter('int'   , Parameter.Type.INTEGER, 2)  ).value
        self.lock_btn        = self.get_parameter_or('lock_btn'        , Parameter('int'   , Parameter.Type.INTEGER, 6)  ).value
        self.add_btn         = self.get_parameter_or('add_btn'         , Parameter('int'   , Parameter.Type.INTEGER, 1)  ).value
        self.remove_btn      = self.get_parameter_or('remove_btn'      , Parameter('int'   , Parameter.Type.INTEGER, 2)  ).value
        self.gen_map_btn     = self.get_parameter_or('gen_map_btn'     , Parameter('int'   , Parameter.Type.INTEGER, 3)  ).value
        self.topic_joy       = self.get_parameter_or('topic_joy'       , Parameter('str'   , Parameter.Type.STRING, '/joy')  ).value
        self.topic_pose      = self.get_parameter_or('topic_pose'      , Parameter('str'   , Parameter.Type.STRING, '/gps_base/odometry')  ).value



        # Disable until toponav 2 supports mongodb storage of new map.
        self.insert_map = False

        # Variables
        self.node_id = 0
        self.nodes = []  # A list of node Pose msgs that will be used to generate the tmap
        self.previous_button = None
        
        # Load templates
        toponav_dir = get_package_share_directory('topological_navigation')
        config_dir = os.path.join(toponav_dir, 'config')
        self.template_node = self.load_yaml(os.path.join(config_dir, 'template_node_2.yaml'))
        self.template_edge = self.load_yaml(os.path.join(config_dir, 'template_edge.yaml'))

        # Services
        self.create_service(Trigger, '/tmapping_robot/save_waypoints', self.save_waypoints_srv_cb)
        self.create_service(Trigger, '/tmapping_robot/save_map', self.generate_tmap_srv_cb)

        # Subscribers
        self.create_subscription(Joy     , self.topic_joy , self.joy_cb       , 10)
        self.create_subscription(Odometry, self.topic_pose, self.robot_pose_cb, 10)

        # Publishers
        self.node_vis_pub = self.create_publisher(MarkerArray, '/tmapping_nodes', 10)

        # Create save folder if is does not exist 
        if not os.path.exists(self.tmap_dir):
            self.get_logger().info("Creating tmap_dir: %s" % (self.tmap_dir))
            os.makedirs(self.tmap_dir)

        # Load existing nodes from tmap_dir if there are any
        self.get_tmap_nodes()


    def load_yaml(self, filename):
        with open(filename, 'r') as f:
            return yaml.safe_load(f)


    def save_yaml(self, filename, data, dfs=True):
        with open(filename, 'w') as f:
            return yaml.safe_dump(data, f, default_flow_style=dfs)


    def get_tmap_nodes(self):
        tmap = os.path.join(self.tmap_dir, self.pointset)
        self.get_logger().info("Getting current nodes from tmap file")
        self.get_logger().info(tmap)

        if os.path.exists(tmap):
            self.topomap = self.load_yaml(tmap)
            self.get_logger().info(f"Existing tmap: {tmap}")
            if self.topomap is not None:
                for node in self.topomap["nodes"]:
                    pose = Pose()
                    pose.position.x = node["node"]["pose"]["position"]["x"]
                    pose.position.y = node["node"]["pose"]["position"]["y"]
                    pose.orientation.x = node["node"]["pose"]["orientation"]["x"]
                    pose.orientation.y = node["node"]["pose"]["orientation"]["y"]
                    pose.orientation.z = node["node"]["pose"]["orientation"]["z"]
                    pose.orientation.w = node["node"]["pose"]["orientation"]["w"]
                    self.nodes.append([self.node_id, pose])
                    self.node_id += 1
                    self.update_node_markers()
            else:
                self.init_map()
        else:
            self.init_map()
        self.get_logger().info(f'There are {len(self.nodes)} nodes in the existing map.')


    def robot_pose_cb(self, msg):
        self.robot_pose_msg = msg.pose.pose


    def joy_cb(self, msg):
        buttons = msg.buttons

        if buttons[self.lock_btn]:
            if buttons[self.add_btn]:
                if self.previous_button is None or self.add_btn != self.previous_button:
                    self.add_node()
                    self.previous_button = self.add_btn
            elif buttons[self.remove_btn]:
                if self.remove_btn != self.previous_button:
                    self.remove_node()
                    self.previous_button = self.remove_btn
            elif buttons[self.gen_map_btn]:
                if self.gen_map_btn != self.previous_button:
                    self.generate_tmap()
                    self.previous_button = self.gen_map_btn
            else:
                self.previous_button = None


    def add_node(self):
        self.get_logger().info("Adding node")
        pose = self.robot_pose_msg
        dist, nearest_id, _ = self.get_nearest_node(pose)
        if not dist or dist > self.node_thresh:
            self.nodes.append([self.node_id, pose])
            self.get_logger().info(f"New node: {pose}")
            self.get_logger().info(f"Num nodes: {len(self.nodes)}")
            self.update_node_markers()
            self.node_id += 1
        else:
            self.get_logger().info("Too close to an existing node, won't add one!")


    def remove_node(self):
        self.get_logger().info("Removing nearest node")
        pose = self.robot_pose_msg
        dist, nearest_id, ind = self.get_nearest_node(pose)
        if not dist:
            self.get_logger().info("List is empty!")
        else:
            if dist > 5:
                self.get_logger().info("Not near any nodes so not removing any!")
            else:
                self.remove_marker(nearest_id)
                self.nodes.pop(ind)
                self.get_logger().info(f"Num nodes: {len(self.nodes)}")


    def update_node_markers(self):
        diameter = 0.7
        marker_array = MarkerArray()

        for node in self.nodes:
            node_vis = Marker()
            node_vis.type = Marker.SPHERE
            node_vis.header.frame_id = 'map'
            node_vis.id = node[0]
            node_vis.pose.position.x = node[1].position.x
            node_vis.pose.position.y = node[1].position.y
            node_vis.pose.position.z = node[1].position.z + diameter / 2
            node_vis.pose.orientation = node[1].orientation
            node_vis.scale.x = diameter
            node_vis.scale.y = diameter
            node_vis.scale.z = diameter
            node_vis.color.a = 1.0
            node_vis.color.r = 1.0
            node_vis.color.g = 0.6
            node_vis.color.b = 0.0

            node_vis.lifetime = Duration()  # Initialize the Duration message

            marker_array.markers.append(node_vis)

        self.node_vis_pub.publish(marker_array)


    def remove_marker(self, id):
        marker_array = MarkerArray()
        node_vis = Marker()
        node_vis.id = id
        node_vis.action = Marker.DELETE
        marker_array.markers.append(node_vis)
        self.node_vis_pub.publish(marker_array)


    def get_nearest_node(self, pose):
        dist = None
        id = None
        ind = None
        for i, node in enumerate(self.nodes):
            d = sqrt((pose.position.x - node[1].position.x) ** 2
                     + (pose.position.y - node[1].position.y) ** 2)
            if not dist or d < dist:
                dist = d
                id = node[0]
                ind = i
        self.get_logger().info(f"{dist}, {id}, {ind}")
        return dist, id, ind


    def rotate_influence_zone(self, node, quat):
        _, _, theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])

        verts = deepcopy(node["node"]["verts"])
        new_verts = []
        for vert in verts:
            new_vert = np.array([vert["x"], vert["y"]])
            new_vert = np.dot(R, new_vert)
            new_verts.append({"x": float(new_vert[0]), "y": float(new_vert[1])})

        node["node"]["verts"] = new_verts

        return node


    def init_map(self):
        self.topomap = {}
        self.topomap["name"] = self.pointset
        self.topomap["metric_map"] = self.site_name
        self.topomap["pointset"] = self.pointset

        self.topomap["transformation"] = {}
        self.topomap["transformation"]["rotation"] = {}
        self.topomap["transformation"]["rotation"]["w"] = 1.0
        self.topomap["transformation"]["rotation"]["x"] = 0.0
        self.topomap["transformation"]["rotation"]["y"] = 0.0
        self.topomap["transformation"]["rotation"]["z"] = 0.0
        self.topomap["transformation"]["translation"] = {}
        self.topomap["transformation"]["translation"]["x"] = 0.0
        self.topomap["transformation"]["translation"]["y"] = 0.0
        self.topomap["transformation"]["translation"]["z"] = 0.0
        self.topomap["transformation"]["child"] = "topo_map"
        self.topomap["transformation"]["parent"] = "map"

        self.topomap["meta"] = {}
        self.topomap["meta"]["last_updated"] = self.get_time()
        self.topomap["nodes"] = []


    def get_time(self):
        return datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


    def set_meta(self):
        for node in self.topomap["nodes"]:
            node["meta"]["map"] = self.site_name
            node["meta"]["pointset"] = self.pointset


    def generate_tmap(self):
        if not self.nodes:
            self.get_logger().warn("No nodes yet, nothing to save!")
            return

        self.get_logger().info("Generating tmap from node list")
        self.init_map()
        self.save_waypoints()
        self.set_meta()

        self.get_logger().info("Saving yaml")
        tmap_file = os.path.join(self.tmap_dir, self.pointset)
        self.save_yaml(tmap_file, self.topomap, False)
        self.get_logger().info("Saved yaml")


    def save_waypoints_srv_cb(self, request, response):
        self.get_logger().info("Saving waypoints to file")
        waypoints = []
        for node in self.nodes:
            waypoint = {
                "pose": {
                    "position": {
                        "x": node[1].position.x,
                        "y": node[1].position.y,
                        "z": node[1].position.z
                    },
                    "orientation": {
                        "x": node[1].orientation.x,
                        "y": node[1].orientation.y,
                        "z": node[1].orientation.z,
                        "w": node[1].orientation.w
                    }
                }
            }
            waypoints.append(waypoint)

        data = {
            "site": self.site_name,
            "nodes": waypoints
        }

        save_file = os.path.join(self.tmap_dir, f'{datetime.datetime.now().strftime("%Y%m%d%H%M%S")}.yml') 
        self.save_yaml(save_file, data)
        response.success = True
        response.message = "Waypoints saved successfully"
        return response


    def generate_tmap_srv_cb(self, request, response):
        self.get_logger().info("Generating topological map")
        self.get_logger().info(f'Generating {len(self.nodes)} point Tmap from current nodes')

        if self.insert_map:
            node_id = 0
            for node in self.nodes:
                node_to_insert = deepcopy(self.template_node)
                node_to_insert["node"]["name"] = f"{self.site_name}_node_{node_id}"
                node_to_insert["node"]["pose"]["position"]["x"] = node[1].position.x
                node_to_insert["node"]["pose"]["position"]["y"] = node[1].position.y
                node_to_insert["node"]["pose"]["position"]["z"] = node[1].position.z
                node_to_insert["node"]["pose"]["orientation"]["x"] = node[1].orientation.x
                node_to_insert["node"]["pose"]["orientation"]["y"] = node[1].orientation.y
                node_to_insert["node"]["pose"]["orientation"]["z"] = node[1].orientation.z
                node_to_insert["node"]["pose"]["orientation"]["w"] = node[1].orientation.w

                self.topomap["nodes"].append(node_to_insert)
                node_id += 1

            tmap_file = os.path.join(self.tmap_dir, self.pointset)
            self.save_yaml(tmap_file, self.topomap)

        response.success = True
        response.message = "Topological map generated successfully"
        return response


    def save_waypoints(self):
        self.get_logger().info("Saving tmap waypoints only")
        self.init_map()
        for i, node_pose in enumerate(self.nodes):
            node = deepcopy(self.template_node)
            node["meta"]["node"] = 'n' + str(i)
            node["node"]["name"] = 'n' + str(i)

            node["node"]["pose"]["position"]["x"] = float(node_pose[1].position.x)
            node["node"]["pose"]["position"]["y"] = float(node_pose[1].position.y)
            node["node"]["pose"]["orientation"]["x"] = float(node_pose[1].orientation.x)
            node["node"]["pose"]["orientation"]["y"] = float(node_pose[1].orientation.y)
            node["node"]["pose"]["orientation"]["z"] = float(node_pose[1].orientation.z)
            node["node"]["pose"]["orientation"]["w"] = float(node_pose[1].orientation.w)
            node["node"]["properties"]["xy_goal_tolerance"] = 0.3
            node["node"]["properties"]["yaw_goal_tolerance"] = 0.3

            node = self.rotate_influence_zone(node, node_pose[1].orientation)
            self.topomap["nodes"].append(node)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTmapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
