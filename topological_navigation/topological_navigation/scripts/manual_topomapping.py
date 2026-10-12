#! /usr/bin/env python3

"""Manual topological mapping node for ROS 2.

Drive a robot with a joystick and press buttons to add / remove nodes.
Edges are created automatically between each new node and its closest
existing neighbour (bidirectional).  Node orientations are set to face
along their outgoing edge direction.

The topological map is published on ``/topological_map_2`` every time a
node is added or removed so that downstream nodes (localisation,
navigation, visualiser) pick up the changes immediately.

The map follows the ``test_simple_tmap2.yaml`` format with a single
``navigate_to_pose`` action definition.

Original Author: Sergi Molina <sergi.molina@sagarobotics.com> 2022 - ROS1
Maintainer: Ibrahim Hroob <ihroob@lincoln.ac.uk> 2024 - ROS2 support
"""

import math
import os
import yaml
import datetime
import numpy as np

from copy import deepcopy

import rclpy
import rclpy.duration

from rclpy import Parameter
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import String

from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from builtin_interfaces.msg import Duration

from tf_transformations import quaternion_from_euler

from ament_index_python.packages import get_package_share_directory


class RobotTmapping(Node):
    """Joystick-driven topological mapping node.

    Nodes are added / removed via joystick buttons.  Edges between the
    two closest nodes are created automatically (bidirectional).  Node
    orientations are set to face the direction of their first outgoing
    edge.  The full map is published to ``/topological_map_2`` on every
    change.
    """

    def __init__(self):
        super().__init__('manual_tmapping_node')

        # -- ROS parameters ---------------------------------------------------
        self.declare_parameter('tmap',          Parameter.Type.STRING)
        self.declare_parameter('tmap_dir',      Parameter.Type.STRING)
        self.declare_parameter('site_name',     Parameter.Type.STRING)
        self.declare_parameter('node_thresh',   Parameter.Type.DOUBLE)
        # Joystick buttons
        self.declare_parameter('lock_btn',      Parameter.Type.INTEGER)
        self.declare_parameter('add_btn',       Parameter.Type.INTEGER)
        self.declare_parameter('remove_btn',    Parameter.Type.INTEGER)
        self.declare_parameter('gen_map_btn',   Parameter.Type.INTEGER)
        # Topics
        self.declare_parameter('topic_joy',     Parameter.Type.STRING)
        self.declare_parameter('topic_pose',    Parameter.Type.STRING)
        self.declare_parameter('topic_imu',     Parameter.Type.STRING)

        self.pointset   = self.get_parameter_or(
            'tmap', Parameter('str', Parameter.Type.STRING, '')).value
        self.tmap_dir   = self.get_parameter_or(
            'tmap_dir', Parameter('str', Parameter.Type.STRING, '')).value
        self.site_name  = self.get_parameter_or(
            'site_name', Parameter('str', Parameter.Type.STRING, '')).value
        self.node_thresh = self.get_parameter_or(
            'node_thresh', Parameter('double', Parameter.Type.DOUBLE, 0.5)).value
        self.lock_btn   = self.get_parameter_or(
            'lock_btn', Parameter('int', Parameter.Type.INTEGER, 6)).value
        self.add_btn    = self.get_parameter_or(
            'add_btn', Parameter('int', Parameter.Type.INTEGER, 1)).value
        self.remove_btn = self.get_parameter_or(
            'remove_btn', Parameter('int', Parameter.Type.INTEGER, 2)).value
        self.gen_map_btn = self.get_parameter_or(
            'gen_map_btn', Parameter('int', Parameter.Type.INTEGER, 3)).value
        self.topic_joy  = self.get_parameter_or(
            'topic_joy', Parameter('str', Parameter.Type.STRING, '/joy')).value
        self.topic_pose = self.get_parameter_or(
            'topic_pose', Parameter('str', Parameter.Type.STRING, '/gps_base/odometry')).value
        self.topic_imu  = self.get_parameter_or(
            'topic_imu', Parameter('str', Parameter.Type.STRING, '/gps_base/yaw')).value

        # -- Internal state ----------------------------------------------------
        self.node_id = 0
        self.nodes = []            # list of [id, Pose]
        self.previous_button = None
        self.robot_pose_msg = Pose()
        self.robot_imu_msg = None

        # -- Load templates from package config --------------------------------
        toponav_dir = get_package_share_directory('topological_navigation')
        config_dir = os.path.join(toponav_dir, 'config')
        self.template_node = self._load_yaml(
            os.path.join(config_dir, 'template_node.yaml'))
        self.template_edge = self._load_yaml(
            os.path.join(config_dir, 'template_edge.yaml'))
        self.template_action = self._load_yaml(
            os.path.join(config_dir, 'template_action.yaml'))

        # -- QoS (transient-local, same as map_manager2) -----------------------
        self._latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # -- Publishers --------------------------------------------------------
        self.node_vis_pub = self.create_publisher(
            MarkerArray, '/tmapping_nodes', 10)
        self.map_pub = self.create_publisher(
            String, '/topological_map_2', self._latched_qos)

        # -- Services ----------------------------------------------------------
        self.create_service(
            Trigger, '/tmapping_robot/save_waypoints',
            self.save_waypoints_srv_cb)
        self.create_service(
            Trigger, '/tmapping_robot/save_map',
            self.generate_tmap_srv_cb)

        # -- Subscribers -------------------------------------------------------
        self.create_subscription(Joy, self.topic_joy, self.joy_cb, 10)
        self.create_subscription(Imu, self.topic_imu, self.robot_imu_cb, 10)
        self.create_subscription(
            Odometry, self.topic_pose, self.robot_pose_cb, 10)

        # -- Create save directory ---------------------------------------------
        if self.tmap_dir and not os.path.exists(self.tmap_dir):
            self.get_logger().info(f"Creating tmap_dir: {self.tmap_dir}")
            os.makedirs(self.tmap_dir)

        # -- Initialise map and load existing nodes ----------------------------
        self._init_topomap()
        self._load_existing_nodes()
        self.get_logger().info(
            f"There are {len(self.nodes)} nodes in the map.")

    # ==================================================================
    # YAML helpers
    # ==================================================================

    @staticmethod
    def _load_yaml(filename):
        """Load a YAML file and return the parsed data."""
        with open(filename, 'r') as fh:
            return yaml.safe_load(fh)

    @staticmethod
    def _save_yaml(filename, data):
        """Dump *data* as YAML to *filename*."""
        with open(filename, 'w') as fh:
            yaml.safe_dump(data, fh, default_flow_style=False)

    # ==================================================================
    # Map initialisation (test_simple_tmap2.yaml structure)
    # ==================================================================

    def _init_topomap(self):
        """Create an empty topomap dict using loaded template files."""
        self.topomap = {
            "meta": {
                "last_updated": self._get_time(),
            },
            "metric_map": self.site_name,
            "name": self.pointset,
            "pointset": self.pointset,
            "transformation": {
                "topo_frame_id": self.site_name or "map",
                "parent": "map",
                "rotation":    {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "definitions": deepcopy(
                self.template_action.get("definitions", {})
            ),
            "actions": deepcopy(
                self.template_action.get("actions", {})
            ),
            "nodes": [],
        }

    # ==================================================================
    # Load existing nodes from disk
    # ==================================================================

    def _load_existing_nodes(self):
        """Reload node list from tmap file on disk (if it exists)."""
        if not self.tmap_dir or not self.pointset:
            return
        tmap_path = os.path.join(self.tmap_dir, self.pointset)
        if not os.path.exists(tmap_path):
            return

        existing = self._load_yaml(tmap_path)
        if existing is None or "nodes" not in existing:
            return

        self.get_logger().info(f"Loading existing tmap: {tmap_path}")
        for nd in existing["nodes"]:
            pose = Pose()
            pose.position.x = nd["node"]["pose"]["position"]["x"]
            pose.position.y = nd["node"]["pose"]["position"]["y"]
            pose.position.z = nd["node"]["pose"]["position"].get("z", 0.0)
            pose.orientation.x = nd["node"]["pose"]["orientation"]["x"]
            pose.orientation.y = nd["node"]["pose"]["orientation"]["y"]
            pose.orientation.z = nd["node"]["pose"]["orientation"]["z"]
            pose.orientation.w = nd["node"]["pose"]["orientation"]["w"]
            self.nodes.append([self.node_id, pose])
            self.node_id += 1

        # Rebuild the full topomap with edges and publish
        self._rebuild_topomap()
        self._update_node_markers()

    # ==================================================================
    # Subscriber callbacks
    # ==================================================================

    def robot_pose_cb(self, msg):
        """Store the latest robot pose from Odometry."""
        self.robot_pose_msg = msg.pose.pose
        if self.robot_imu_msg is not None:
            self.robot_pose_msg.orientation = self.robot_imu_msg

    def robot_imu_cb(self, msg):
        """Store the latest IMU orientation."""
        self.robot_imu_msg = msg.orientation

    def joy_cb(self, msg):
        """Handle joystick button presses."""
        buttons = msg.buttons
        if not buttons[self.lock_btn]:
            self.previous_button = None
            return

        if buttons[self.add_btn]:
            if self.previous_button != self.add_btn:
                self.add_node()
                self.previous_button = self.add_btn
        elif buttons[self.remove_btn]:
            if self.previous_button != self.remove_btn:
                self.remove_node()
                self.previous_button = self.remove_btn
        elif buttons[self.gen_map_btn]:
            if self.previous_button != self.gen_map_btn:
                self.generate_tmap()
                self.previous_button = self.gen_map_btn
        else:
            self.previous_button = None

    # ==================================================================
    # Add / remove nodes
    # ==================================================================

    def add_node(self):
        """Add a node at the robot's current pose if not too close to an existing one."""
        self.get_logger().info("Adding node")
        pose = deepcopy(self.robot_pose_msg)
        dist, _, _ = self._get_nearest_node(pose)
        if dist is not None and dist <= self.node_thresh:
            self.get_logger().info("Too close to an existing node, won't add one!")
            return

        self.nodes.append([self.node_id, pose])
        self.get_logger().info(
            f"New node{self.node_id}: x={pose.position.x:.2f}, "
            f"y={pose.position.y:.2f}  (total: {len(self.nodes)})")
        self.node_id += 1

        self._rebuild_topomap()
        self._update_node_markers()

    def remove_node(self):
        """Remove the nearest node to the robot (within 5 m)."""
        self.get_logger().info("Removing nearest node")
        pose = self.robot_pose_msg
        dist, nearest_id, ind = self._get_nearest_node(pose)
        if dist is None:
            self.get_logger().info("Node list is empty!")
            return
        if dist > 5.0:
            self.get_logger().info("Not near any nodes – not removing any!")
            return

        self._remove_marker(nearest_id)
        self.nodes.pop(ind)
        self.get_logger().info(f"Removed node (total: {len(self.nodes)})")

        self._rebuild_topomap()
        self._update_node_markers()

    # ==================================================================
    # Nearest-node helpers
    # ==================================================================

    def _get_nearest_node(self, pose):
        """Return (distance, id, index) of the closest node, or (None, None, None)."""
        best_dist = None
        best_id = None
        best_ind = None
        for i, (nid, npose) in enumerate(self.nodes):
            d = math.hypot(
                pose.position.x - npose.position.x,
                pose.position.y - npose.position.y,
            )
            if best_dist is None or d < best_dist:
                best_dist = d
                best_id = nid
                best_ind = i
        return best_dist, best_id, best_ind

    # ==================================================================
    # Edge creation – connect each node to its closest neighbour
    # ==================================================================

    @staticmethod
    def _yaw_between(src_pose, dst_pose):
        """Return yaw angle (radians) from *src_pose* to *dst_pose*."""
        dx = dst_pose.position.x - src_pose.position.x
        dy = dst_pose.position.y - src_pose.position.y
        return math.atan2(dy, dx)

    @staticmethod
    def _orientation_from_yaw(yaw):
        """Return a quaternion dict for a given yaw angle."""
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        return {
            "x": float(qx), "y": float(qy),
            "z": float(qz), "w": float(qw),
        }

    def _build_edges(self):
        """Build bidirectional edges between closest node pairs.

        For every node, an edge to the closest neighbour is created in
        both directions.  Duplicate edges are avoided.  Returns a dict
        mapping ``node_name -> [edge_dict, ...]`` and a dict mapping
        ``node_name -> yaw`` (orientation toward first edge target).
        """
        n = len(self.nodes)
        edges_map = {f"node{nd[0]}": [] for nd in self.nodes}
        node_yaws = {}

        if n < 2:
            return edges_map, node_yaws

        # Collect positions for vectorised distance calculation
        positions = np.array(
            [[nd[1].position.x, nd[1].position.y] for nd in self.nodes])

        for i in range(n):
            src_name = f"node{self.nodes[i][0]}"
            src_pose = self.nodes[i][1]

            # Distance to every other node
            diffs = positions - positions[i]
            dists = np.linalg.norm(diffs, axis=1)
            dists[i] = np.inf  # exclude self

            closest_idx = int(np.argmin(dists))
            dst_name = f"node{self.nodes[closest_idx][0]}"
            dst_pose = self.nodes[closest_idx][1]

            # Forward edge (src -> dst)
            edge_id_fwd = f"{src_name}_{dst_name}"
            existing_ids = {e["edge_id"] for e in edges_map[src_name]}
            if edge_id_fwd not in existing_ids:
                fwd = deepcopy(self.template_edge)
                fwd["edge_id"] = edge_id_fwd
                fwd["node"] = dst_name
                edges_map[src_name].append(fwd)

            # Reverse edge (dst -> src)
            edge_id_rev = f"{dst_name}_{src_name}"
            existing_ids_dst = {e["edge_id"] for e in edges_map[dst_name]}
            if edge_id_rev not in existing_ids_dst:
                rev = deepcopy(self.template_edge)
                rev["edge_id"] = edge_id_rev
                rev["node"] = src_name
                edges_map[dst_name].append(rev)

            # Yaw: point from src toward its closest neighbour
            if src_name not in node_yaws:
                node_yaws[src_name] = self._yaw_between(src_pose, dst_pose)

        return edges_map, node_yaws

    # ==================================================================
    # Topomap rebuild & publish
    # ==================================================================

    def _rebuild_topomap(self):
        """Rebuild the full topomap dict from self.nodes and publish it."""
        self._init_topomap()

        edges_map, node_yaws = self._build_edges()

        for nid, pose in self.nodes:
            name = f"node{nid}"

            # Orientation: face along the edge direction
            if name in node_yaws:
                orientation = self._orientation_from_yaw(node_yaws[name])
            else:
                orientation = {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                }

            # Build node dict from template
            node_dict = deepcopy(self.template_node)

            # Populate meta
            node_dict["meta"]["map"] = self.site_name
            node_dict["meta"]["node"] = name
            node_dict["meta"]["pointset"] = self.pointset

            # Populate node fields
            nd = node_dict["node"]
            nd["edges"] = edges_map.get(name, [])
            nd["name"] = name
            nd["pose"]["orientation"] = orientation
            nd["pose"]["position"] = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            }

            # Rotate template influence-zone vertices to match orientation
            template_verts = nd.get("verts", [])
            nd["verts"] = self._rotate_verts(
                template_verts, node_yaws.get(name, 0.0))

            self.topomap["nodes"].append(node_dict)

        self.topomap["meta"]["last_updated"] = self._get_time()

        # Publish to /topological_map_2
        self._publish_topomap()

    def _publish_topomap(self):
        """Serialise self.topomap to YAML and publish on /topological_map_2."""
        msg = String()
        msg.data = yaml.dump(self.topomap, default_flow_style=False)
        self.map_pub.publish(msg)
        self.get_logger().info(
            f"Published topological map ({len(self.topomap['nodes'])} nodes) "
            "to /topological_map_2")

    # ==================================================================
    # Influence-zone rotation
    # ==================================================================

    @staticmethod
    def _rotate_verts(verts, yaw):
        """Rotate influence-zone vertices by *yaw* radians."""
        cos_t = math.cos(yaw)
        sin_t = math.sin(yaw)
        rotated = []
        for v in verts:
            rx = v["x"] * cos_t - v["y"] * sin_t
            ry = v["x"] * sin_t + v["y"] * cos_t
            rotated.append({"x": round(rx, 6), "y": round(ry, 6)})
        return rotated

    # ==================================================================
    # Save / generate helpers
    # ==================================================================

    @staticmethod
    def _get_time():
        """Return a timestamp string."""
        return datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    def generate_tmap(self):
        """Rebuild, save, and publish the topological map."""
        if not self.nodes:
            self.get_logger().warn("No nodes yet, nothing to save!")
            return

        self.get_logger().info("Generating tmap from node list")
        self._rebuild_topomap()

        if self.tmap_dir and self.pointset:
            tmap_file = os.path.join(self.tmap_dir, self.pointset)
            self._save_yaml(tmap_file, self.topomap)
            self.get_logger().info(f"Saved tmap to {tmap_file}")

    def save_waypoints_srv_cb(self, request, response):
        """Service: save raw waypoint poses to a timestamped YAML file."""
        self.get_logger().info("Saving waypoints to file")
        waypoints = []
        for nid, pose in self.nodes:
            waypoints.append({
                "pose": {
                    "position": {
                        "x": float(pose.position.x),
                        "y": float(pose.position.y),
                        "z": float(pose.position.z),
                    },
                    "orientation": {
                        "x": float(pose.orientation.x),
                        "y": float(pose.orientation.y),
                        "z": float(pose.orientation.z),
                        "w": float(pose.orientation.w),
                    },
                },
            })

        data = {"site": self.site_name, "nodes": waypoints}
        ts = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        save_file = os.path.join(self.tmap_dir, f'{ts}.yml')
        self._save_yaml(save_file, data)
        response.success = True
        response.message = f"Waypoints saved to {save_file}"
        return response

    def generate_tmap_srv_cb(self, request, response):
        """Service: rebuild, save, and publish the topological map."""
        self.get_logger().info("Generating topological map via service")
        self.generate_tmap()
        response.success = True
        response.message = "Topological map generated and published"
        return response

    # ==================================================================
    # Marker visualisation
    # ==================================================================

    def _update_node_markers(self):
        """Publish RViz markers for all current nodes."""
        diameter = 0.7
        marker_array = MarkerArray()

        for nid, pose in self.nodes:
            m = Marker()
            m.type = Marker.SPHERE
            m.header.frame_id = 'map'
            m.id = nid
            m.pose.position.x = pose.position.x
            m.pose.position.y = pose.position.y
            m.pose.position.z = pose.position.z + diameter / 2
            m.pose.orientation = pose.orientation
            m.scale.x = diameter
            m.scale.y = diameter
            m.scale.z = diameter
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.6
            m.color.b = 0.0
            m.lifetime = Duration()
            marker_array.markers.append(m)

        self.node_vis_pub.publish(marker_array)

    def _remove_marker(self, marker_id):
        """Delete a single RViz marker by ID."""
        marker_array = MarkerArray()
        m = Marker()
        m.id = marker_id
        m.action = Marker.DELETE
        marker_array.markers.append(m)
        self.node_vis_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTmapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
