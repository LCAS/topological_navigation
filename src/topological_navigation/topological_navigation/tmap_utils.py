#!/usr/bin/env python3
"""Topological map utility functions.

Provides helper functions for YAML loading and node/edge access
in the tmap2 format.
"""
import yaml

# ===== YAML Loader =====


class CustomSafeLoader(yaml.SafeLoader):
    """Custom YAML loader that ensures poses and translations are float-type.

    ROS 2 messages (Vector3, Pose, etc.) have assertions for float-type
    [x, y, z, w] keys.
    """

    def construct_mapping(self, node, deep=False):
        """Construct mapping, converting int to float for pose keys."""
        mapping = super().construct_mapping(node, deep=deep)

        # Convert int to float for pose/vector keys
        for key in ['x', 'y', 'z', 'w', 'yaw_goal_tolerance', 'xy_goal_tolerance']:
            if key in mapping and isinstance(mapping[key], int):
                mapping[key] = float(mapping[key])

        return mapping


# ========================


class NoAliasDumper(yaml.SafeDumper):
    """YAML dumper that disables aliases/anchors for cleaner output."""

    def ignore_aliases(self, data):
        """Return True for all data to disable aliases."""
        return True


# ========================


def get_node_from_tmap2(top_map, node_name):
    """Given a topological map 2 and a node name return the node object."""
    for i in top_map["nodes"]:
        if i["node"]["name"] == node_name:
            return i
    return None


def get_edge_from_id_tmap2(top_map, node_name, edge_id):
    """Given a node and the edge_id return the edge object for topomap 2."""
    node = get_node_from_tmap2(top_map, node_name)
    if node:
        for i in node["node"]["edges"]:
            if i["edge_id"] == edge_id:
                return i
    return None
