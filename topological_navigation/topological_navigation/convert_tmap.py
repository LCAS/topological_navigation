#!/usr/bin/env python
"""Convert legacy topological maps to the tmap3 format.

The old format stores ``action_type`` on every edge and uses CamelCase
action names (e.g. ``NavigateToPose``, ``RowOperation``).  Nodes carry
``localise_by_topic`` and ``parent_frame`` directly.  The transformation
block uses ``child`` instead of ``topo_frame_id``.

The new tmap3 format adds ``nav_frame`` to each node, renames
``transformation.child`` to ``transformation.topo_frame_id``, drops
legacy node fields (``localise_by_topic``, ``parent_frame``,
``restrictions_planning``, ``restrictions_runtime``), maps action names
to snake_case, and removes ``tag`` from node meta.
No top-level ``definitions`` or ``actions`` sections are emitted.

Usage::

    ros2 run topological_navigation convert_tmap.py input.yaml -o output.tmap3.yaml
    ros2 run topological_navigation convert_tmap.py input.yaml  # writes to stdout
"""

import argparse
import os
import re
import sys

import yaml


# =====================================================================
# Action name mapping (old CamelCase -> new snake_case keys)
# =====================================================================

_ACTION_NAME_MAP = {
    'NavigateToPose': 'navigate_to_pose',
    'NavigateThroughPoses': 'navigate_through_poses',
    'FollowWaypoints': 'follow_waypoints',
    'RowOperation': 'row_traversal',
    'RowTraversal': 'row_traversal',
    'GoalAlign': 'goal_align',
}




# =====================================================================
# Conversion helpers
# =====================================================================


def _map_action_name(old_name):
    """Map old CamelCase action name to new snake_case name."""
    if old_name in _ACTION_NAME_MAP:
        return _ACTION_NAME_MAP[old_name]
    # Fallback: convert CamelCase to snake_case
    s1 = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', old_name)
    return re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s1).lower()





def _convert_node(node_entry):
    """Convert a single node entry from old to tmap3 format.

    - Removes ``action_type`` from edges.
    - Maps edge ``action`` names to snake_case.
    - Adds ``nav_frame`` referencing the transformation topo_frame_id.
    - Removes ``localise_by_topic``, ``parent_frame``,
      ``restrictions_planning``, and ``restrictions_runtime``.
    - Removes ``tag`` from meta.
    """
    new_entry = {}

    # -- meta --------------------------------------------------------
    old_meta = node_entry.get('meta', {})
    new_meta = {
        'map': old_meta.get('map', ''),
        'node': old_meta.get('node', ''),
        'pointset': old_meta.get('pointset', ''),
    }
    new_entry['meta'] = new_meta

    # -- node --------------------------------------------------------
    old_node = node_entry.get('node', {})
    new_node = {}

    # Edges: convert action names, drop action_type
    new_edges = []
    for edge in old_node.get('edges', []):
        new_edge = {
            'action': _map_action_name(edge.get('action', '')),
            'edge_id': edge.get('edge_id', ''),
            'node': edge.get('node', ''),
        }
        props = edge.get('properties')
        if props:
            new_edge['properties'] = props
        new_edges.append(new_edge)
    new_node['edges'] = new_edges

    # Core fields
    new_node['name'] = old_node.get('name', '')
    new_node['nav_frame'] = '${transformation.topo_frame_id}'
    new_node['pose'] = old_node.get('pose', {})

    # Properties (keep as-is)
    props = old_node.get('properties')
    if props:
        new_node['properties'] = props

    # Verts (influence zone)
    verts = old_node.get('verts')
    if verts:
        new_node['verts'] = verts

    new_entry['node'] = new_node
    return new_entry


def _convert_transformation(old_tf):
    """Convert transformation block.

    Renames ``child`` to ``topo_frame_id`` if present.
    """
    new_tf = {}
    for k, v in old_tf.items():
        if k == 'child':
            new_tf['topo_frame_id'] = v
        else:
            new_tf[k] = v
    # Ensure topo_frame_id exists
    if 'topo_frame_id' not in new_tf:
        new_tf['topo_frame_id'] = old_tf.get(
            'child', old_tf.get('topo_frame_id', ''),
        )
    return new_tf


def convert_tmap(old_data):
    """Convert an old-format topological map dict to the tmap3 format.

    Parameters
    ----------
    old_data : dict
        Parsed YAML of an old-format topological map.

    Returns
    -------
    dict
        New tmap3-format topological map.
    """
    new_data = {}

    # -- Top-level metadata ------------------------------------------
    new_data['meta'] = old_data.get('meta', {})

    for key in ('metric_map', 'name', 'pointset'):
        if key in old_data:
            new_data[key] = old_data[key]

    # -- Transformation ----------------------------------------------
    old_tf = old_data.get('transformation', {})
    new_data['transformation'] = _convert_transformation(old_tf)

    # -- Nodes -------------------------------------------------------
    old_nodes = old_data.get('nodes', [])
    new_data['nodes'] = [_convert_node(n) for n in old_nodes]

    return new_data


# =====================================================================
# CLI
# =====================================================================

def main(args=None):
    """Entry point for ``convert_tmap.py``."""
    parser = argparse.ArgumentParser(
        description=(
            'Convert a legacy topological map YAML to the tmap3 format. '
            'Recommended output extension: .tmap3.yaml'
        ),
    )
    parser.add_argument(
        'input',
        help='Path to the old-format topological map YAML file.',
    )
    parser.add_argument(
        '-o', '--output',
        default=None,
        help=(
            'Output file path.  If omitted, writes to stdout.  '
            'Recommended extension: .tmap3.yaml'
        ),
    )

    parsed = parser.parse_args(args)

    # -- Load input --------------------------------------------------
    input_path = parsed.input
    if not os.path.isfile(input_path):
        print("Error: file not found: %s" % input_path, file=sys.stderr)
        sys.exit(1)

    with open(input_path, 'r') as fh:
        old_data = yaml.safe_load(fh)

    if not isinstance(old_data, dict):
        print("Error: input is not a valid YAML mapping", file=sys.stderr)
        sys.exit(1)

    # -- Convert -----------------------------------------------------
    new_data = convert_tmap(old_data)

    # -- Output ------------------------------------------------------
    output_yaml = yaml.dump(
        new_data,
        default_flow_style=False,
        sort_keys=False,
        allow_unicode=True,
        width=120,
    )

    if parsed.output:
        out_dir = os.path.dirname(parsed.output)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(parsed.output, 'w') as fh:
            fh.write(output_yaml)
        print(
            "Converted: %s -> %s" % (input_path, parsed.output),
            file=sys.stderr,
        )
    else:
        sys.stdout.write(output_yaml)


if __name__ == '__main__':
    main()
