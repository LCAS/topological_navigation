#!/usr/bin/env python3
"""
Self-contained Topological Map Manager 2 for ROS 2.

This script manages topological maps: loading, saving, validating against a
YAML schema, and publishing via ROS 2 topics and services. It is fully
self-contained with no dependency on external tmap_model or map_types modules.

@author: Adam Binch (abinch@sagarobotics.com)
@maintainer: Ibrahim Hroob (ihroob@lincoln.ac.uk)
Refactored: 2026-02-11
"""

#########################################################################################################
import os
import sys
import json
import yaml
import datetime

import rclpy
import rclpy.node
import tf2_ros
import std_msgs.msg
import rosidl_runtime_py
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped

from std_srvs.srv import Trigger
import topological_navigation_msgs.srv as tn_srv

from topological_navigation.tmap_utils import load_tmap2_file, save_tmap2_file

try:
    import jsonschema
except ImportError:
    jsonschema = None


#########################################################################################################
# Exceptions
#########################################################################################################

class MapValidationError(Exception):
    pass

class NodeNotFoundError(Exception):
    pass

class EdgeNotFoundError(Exception):
    pass


#########################################################################################################
# map_manager_2 (self-contained)
#########################################################################################################

class map_manager_2(rclpy.node.Node):
    """
    Self-contained topological map manager node.

    All map data (tmap dict), YAML schema validation, loading, saving, and
    consistency checking are handled directly in this class -- no external
    model module is required.
    """

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------
    def __init__(self, advertise_srvs=True):
        super().__init__('topological_map_manager_2')

        package_path = get_package_share_directory('topological_navigation')

        # Schema path for validation
        self.schema_path = str(os.path.join(package_path, 'config', 'tmap-schema.yaml'))

        # Declare parameters
        self.declare_parameter('cache_topological_maps', False)
        self.declare_parameter('auto_write_topological_maps', False)
        self.declare_parameter('topological_map2_name', '')
        self.declare_parameter('topological_map2_path', '')

        self.add_on_set_parameters_callback(self._parameters_callback)

        self.cache_maps = self.get_parameter('cache_topological_maps').value
        self.auto_write = self.get_parameter('auto_write_topological_maps').value
        self.topomap2_name = self.get_parameter('topological_map2_name').value
        self.topomap2_path = self.get_parameter('topological_map2_path').value

        # Cache directory
        self.cache_dir = os.path.join(os.path.expanduser("~"), ".ros", "topological_maps")
        os.makedirs(self.cache_dir, exist_ok=True)

        # Load the schema once
        self.schema = self._load_schema(self.schema_path)

        # Initialise an empty tmap dict
        self.tmap = self._empty_tmap()
        self._tmap_io_layout = None

        # Latched QoS for map and schema publishers
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publisher: topological map (JSON string)
        self.map_pub = self.create_publisher(
            std_msgs.msg.String, '/topological_map_2', latched_qos
        )

        # Publisher: schema (YAML string, latched)
        self.schema_pub = self.create_publisher(
            std_msgs.msg.String, '/topological_map_schema', latched_qos
        )
        self._publish_schema()

        # Advertise services
        if advertise_srvs:
            self._advertise()

    # ------------------------------------------------------------------
    # Schema helpers
    # ------------------------------------------------------------------
    def _load_schema(self, schema_path):
        """Load the YAML schema file. Returns dict or None."""
        try:
            with open(schema_path, 'r') as f:
                schema = yaml.safe_load(f)
            self.get_logger().info(f"Loaded schema from {schema_path}")
            return schema
        except Exception as e:
            self.get_logger().error(f"Failed to load schema: {e}")
            return None

    def _publish_schema(self):
        """Publish the schema as a latched String message."""
        if self.schema is not None:
            msg = std_msgs.msg.String()
            msg.data = yaml.safe_dump(self.schema, default_flow_style=False)
            self.schema_pub.publish(msg)
            self.get_logger().info("Published schema on /topological_map_schema")

    # ------------------------------------------------------------------
    # Empty map template
    # ------------------------------------------------------------------
    def _empty_tmap(self):
        return {
            "meta": {"last_updated": self._get_time()},
            "metric_map": "map_2d",
            "name": "new_map",
            "pointset": "new_map",
            "transformation": {
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "child": "topo_map",
                "parent": "map",
            },
            "nodes": [],
        }

    @staticmethod
    def _get_time():
        return datetime.datetime.now().strftime('%d-%m-%Y_%H-%M-%S')

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------
    def validate(self):
        """Validate self.tmap against schema + logical consistency."""
        if self.schema is not None:
            if jsonschema is None:
                self.get_logger().warning(
                    "jsonschema not installed -- skipping schema validation"
                )
            else:
                try:
                    jsonschema.validate(instance=self.tmap, schema=self.schema)
                except jsonschema.exceptions.ValidationError as e:
                    raise MapValidationError(
                        f"Schema validation failed: {e.message}"
                    )

        self._check_consistency()

    def _check_consistency(self):
        """Logical checks: duplicate names, broken edges, etc."""
        if "nodes" not in self.tmap:
            raise MapValidationError("Map must contain 'nodes' key.")

        pointsets = {n["meta"]["pointset"] for n in self.tmap["nodes"]}
        if len(pointsets) > 1:
            raise MapValidationError(f"Multiple pointsets found: {pointsets}")

        names = [n["node"]["name"] for n in self.tmap["nodes"]]
        if len(names) != len(set(names)):
            seen, dupes = set(), []
            for x in names:
                if x in seen:
                    dupes.append(x)
                seen.add(x)
            raise MapValidationError(f"Duplicate node names: {dupes}")

        name_set = set(names)
        for node in self.tmap["nodes"]:
            origin = node["node"]["name"]
            for edge in node["node"]["edges"]:
                dest = edge["node"]
                if dest not in name_set:
                    raise MapValidationError(
                        f"Edge from {origin} points to non-existent node {dest}"
                    )

    # ------------------------------------------------------------------
    # Load / Save
    # ------------------------------------------------------------------
    def load_map(self, filename):
        """Load a topological map YAML file, validate, and sync state."""
        self.get_logger().info(f"Loading topological map: {filename}")
        try:
            loaded, self._tmap_io_layout = load_tmap2_file(
                filename,
                logger=self.get_logger(),
                return_layout=True,
            )
            self.tmap = loaded
            self.validate()
            self._sync_from_tmap()
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'topological_map2_name',
                    rclpy.Parameter.Type.STRING,
                    self.tmap.get("pointset", ""),
                )
            ])
            self.get_logger().info("Map loaded and validated successfully.")
            if self.cache_maps:
                cache_file = os.path.join(
                    self.cache_dir, os.path.basename(filename)
                )
                self._save_map(cache_file, no_alias=True)
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            raise

    def _save_map(self, filename, no_alias=False):
        """Write self.tmap to a YAML file."""
        self.get_logger().info(f"Saving map to {filename}")
        if "nodes" in self.tmap:
            self.tmap["nodes"].sort(key=lambda n: n["node"]["name"])
        save_tmap2_file(
            self.tmap,
            filename,
            no_alias=no_alias,
            layout=self._tmap_io_layout,
            logger=self.get_logger(),
        )
        self.get_logger().info("Map saved successfully.")

    def _sync_from_tmap(self):
        """Sync convenience attributes from self.tmap."""
        self.name = self.tmap.get("name", "new_map")
        self.metric_map = self.tmap.get("metric_map", "map_2d")
        self.pointset = self.tmap.get("pointset", "new_map")
        self.transformation = self.tmap.get("transformation", {})

    # ------------------------------------------------------------------
    # Map initialisation
    # ------------------------------------------------------------------
    def init_map(self, name="new_map", metric_map="map_2d", pointset="new_map",
                 transformation="default", filepath=None, load=True):

        if transformation == "default":
            self.transformation = {
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "child": "topo_map",
                "parent": "map",
            }
        else:
            self.transformation = transformation

        if load:
            self.load_map(filepath)
        else:
            self.tmap = self._empty_tmap()
            self.tmap["name"] = name
            self.tmap["metric_map"] = metric_map
            self.tmap["pointset"] = pointset
            self.tmap["transformation"] = self.transformation

        self.map_pub.publish(std_msgs.msg.String(data=json.dumps(self.tmap)))
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_transform()

    # ------------------------------------------------------------------
    # Update & broadcast
    # ------------------------------------------------------------------
    def update(self, update_time=True):
        if update_time:
            self.tmap["meta"]["last_updated"] = self._get_time()
        self.validate()
        self.map_pub.publish(std_msgs.msg.String(data=json.dumps(self.tmap)))

    def broadcast_transform(self):
        trans, rot = Vector3(), Quaternion()
        rosidl_runtime_py.set_message_fields(
            trans, self.transformation.get("translation", {})
        )
        rosidl_runtime_py.set_message_fields(
            rot, self.transformation.get("rotation", {})
        )

        # Use topo_frame_id as the child frame so localisation
        # can look up the transform to the frame the map is defined in.
        child_frame = self.transformation.get(
            "topo_frame_id",
            self.transformation.get("child", "topo_map"),
        )

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.transformation.get("parent", "map")
        msg.child_frame_id = child_frame
        msg.transform.translation = trans
        msg.transform.rotation = rot
        self.broadcaster.sendTransform(msg)
        self.get_logger().info(
            f'Broadcasting static TF: {msg.header.frame_id} -> {child_frame}'
        )

    # ------------------------------------------------------------------
    # Parameter callback
    # ------------------------------------------------------------------
    def _parameters_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'cache_topological_maps':
                self.cache_maps = param.value
            elif param.name == 'auto_write_topological_maps':
                self.auto_write = param.value
            elif param.name == 'topological_map2_name':
                self.tmap["name"] = param.value
            elif param.name == 'topological_map2_path':
                pass
            self.get_logger().info(f'Param {param.name} -> {param.value}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # ROS 2 services
    # ------------------------------------------------------------------
    def _advertise(self):
        self.get_logger().info("Advertising services...")
        self.get_map_srv = self.create_service(
            Trigger,
            '/topological_map_manager2/get_topological_map',
            self.get_topological_map_cb,
        )
        self.write_map_srv = self.create_service(
            tn_srv.WriteTopologicalMap,
            '/topological_map_manager2/write_topological_map',
            self.write_topological_map_cb,
        )
        self.switch_map_srv = self.create_service(
            tn_srv.WriteTopologicalMap,
            '/topological_map_manager2/switch_topological_map',
            self.switch_topological_map_cb,
        )

    def get_topological_map_cb(self, req, res):
        """Service: return the current map as JSON."""
        self.get_logger().info("[SRV] get_topological_map")
        res.success = True
        res.message = json.dumps(self.tmap)
        return res

    def switch_topological_map_cb(self, req, res):
        """Service: switch to a different map file."""
        path = self.get_parameter("topological_map2_path").value
        if os.path.isabs(req.filename) or path == "":
            filename = req.filename
        else:
            filename = os.path.join(path, req.filename)

        self.get_logger().info(f"[SRV] switch_topological_map -> {filename}")
        try:
            self.load_map(filename)
            self.update(False)
            self.broadcast_transform()
            res.success = True
            res.message = json.dumps(self.tmap)
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def write_topological_map_cb(self, req, res):
        """Service: save the current map to a YAML file."""
        filename = req.filename
        if not filename:
            path = self.get_parameter("topological_map2_path").value
            fname = self.get_parameter("topological_map2_name").value
            filename = os.path.join(path, fname)

        self.get_logger().info(f"[SRV] write_topological_map -> {filename}")
        try:
            self._save_map(filename, req.no_alias)
            res.success = True
            res.message = f"Map written to {filename}"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res


#########################################################################################################
# CLI
#########################################################################################################

def parse_arguments():
    """Parse CLI arguments."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Topological Map Manager 2 -- self-contained ROS 2 node',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s my_map.yaml              Load an existing map
  %(prog)s -n new_map.yaml          Create a new empty map
  %(prog)s --test                   Load the default test map
  %(prog)s -v my_map.yaml           Load map with verbose logging
""",
    )
    parser.add_argument('map_file', nargs='?', default=None,
                        help='Path to the topological map YAML file')
    parser.add_argument('-n', '--new', action='store_true',
                        help='Create a new empty map')
    parser.add_argument('-t', '--test', action='store_true',
                        help='Load the default test map')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Enable verbose logging')

    # Use parse_known_args to ignore ROS2 --ros-args passed by launch files
    args, _ = parser.parse_known_args()

    if args.test or (args.map_file is None and not args.new):
        try:
            pkg = get_package_share_directory('topological_navigation')
            map_file = os.path.join(pkg, 'config', 'test_simple_tmap2.yaml')
        except Exception as e:
            print(f"Error: Could not find default test map: {e}")
            sys.exit(1)
    elif args.new:
        if args.map_file is None:
            print("Error: --new requires a map filename")
            sys.exit(1)
        map_file = args.map_file
    else:
        map_file = args.map_file

    if not args.new and map_file and not os.path.exists(map_file):
        print(f"Error: Map file not found: {map_file}")
        sys.exit(1)

    return map_file, not args.new, args.verbose


def main(args=None):
    manager = None
    try:
        map_file, load, verbose = parse_arguments()
        rclpy.init(args=args)

        manager = map_manager_2(advertise_srvs=True)

        if verbose:
            manager.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        manager.get_logger().info("=" * 60)
        manager.get_logger().info("Topological Map Manager 2 -- Starting")
        manager.get_logger().info("=" * 60)

        manager.init_map(filepath=map_file, load=load)
        manager.get_logger().info(
            f"Map ready -- name={manager.tmap.get('name')}, "
            f"nodes={len(manager.tmap.get('nodes', []))}"
        )

        try:
            rclpy.spin(manager)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                if manager is not None:
                    manager.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

        return 0

    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
#########################################################################################################
