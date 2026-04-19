#!/usr/bin/env python
"""Convert legacy topological maps to the new map-driven format.

The old format stores ``action_type`` on every edge and uses CamelCase
action names (e.g. ``NavigateToPose``, ``RowOperation``).  Nodes carry
``localise_by_topic`` and ``parent_frame`` directly.  There is no
top-level ``definitions`` or ``actions`` section.

The new format centralises action configuration in top-level
``definitions`` (BT XML blobs) and ``actions`` (mapping action names
to Nav2 servers, types, composability, and goal templates).  Edges
reference action names only; ``action_type`` is removed.  Node-level
``localise_by_topic`` and ``parent_frame`` are dropped (handled
elsewhere in the new architecture).

Usage::

    ros2 run topological_navigation convert_tmap.py input.yaml -o output.tmap2.yaml
    ros2 run topological_navigation convert_tmap.py input.yaml  # writes to stdout

The script auto-discovers which action types appear in edges and
generates default ``actions`` entries.  Custom BT XML can be supplied
via ``--bt-dir`` pointing to a directory of ``.xml`` files whose
stems become definition names.
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

# Default action_type (ROS 2 dotted import) for each new action name.
_DEFAULT_ACTION_TYPES = {
    'navigate_to_pose': 'nav2_msgs.action.NavigateToPose',
    'navigate_through_poses': 'nav2_msgs.action.NavigateThroughPoses',
    'follow_waypoints': 'nav2_msgs.action.FollowWaypoints',
    'row_traversal': 'nav2_msgs.action.NavigateThroughPoses',
    'goal_align': 'nav2_msgs.action.NavigateToPose',
}

_DEFAULT_ACTION_SERVERS = {
    'navigate_to_pose': '/navigate_to_pose',
    'navigate_through_poses': '/navigate_through_poses',
    'follow_waypoints': '/follow_waypoints',
    'row_traversal': '/navigate_through_poses',
    'goal_align': '/navigate_to_pose',
}

# Multi-waypoint actions are composable; single-pose actions are not.
_DEFAULT_COMPOSABLE = {
    'navigate_to_pose': False,
    'navigate_through_poses': True,
    'follow_waypoints': True,
    'row_traversal': True,
    'goal_align': False,
}

# Goal template: single-pose vs multi-pose
_SINGLE_POSE_ACTIONS = {'navigate_to_pose', 'goal_align'}
_MULTI_POSE_ACTIONS = {
    'navigate_through_poses', 'follow_waypoints', 'row_traversal',
}


# =====================================================================
# Default BT XML templates
# =====================================================================

_DEFAULT_BT = """\
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" \
service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" \
service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" \
service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" \
service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.15" backup_speed="0.025"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""

_ROW_TRAVERSAL_BT = """\
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="0" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="0.333">
          <RecoveryNode number_of_retries="0" \
name="ComputePathThroughPoses">
            <ReactiveSequence>
              <RemovePassedGoals input_goals="{goals}" \
output_goals="{goals}" radius="1.5"/>
              <ComputePathThroughPoses goals="{goals}" path="{path}" \
planner_id="GridBased"/>
            </ReactiveSequence>
            <ReactiveFallback \
name="ComputePathThroughPosesRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" \
service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="0" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" \
service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" \
service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" \
service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.15" backup_speed="0.025"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""

# Which BT definition name to use for each action
_ACTION_BT_MAP = {
    'navigate_to_pose': 'default_bt',
    'navigate_through_poses': 'default_bt',
    'follow_waypoints': 'default_bt',
    'goal_align': 'goal_align_bt',
    'row_traversal': 'row_traversal_bt',
}

# Built-in BT definitions (used when --bt-dir is not supplied)
_BUILTIN_BT_DEFS = {
    'default_bt': _DEFAULT_BT,
    'goal_align_bt': _DEFAULT_BT,       # same XML, different name
    'row_traversal_bt': _ROW_TRAVERSAL_BT,
}


# =====================================================================
# Conversion helpers
# =====================================================================

def _normalise_action_type(old_type):
    """Convert ``nav2_msgs/action/X`` (slash) to ``nav2_msgs.action.X`` (dot)."""
    if '/' in old_type:
        return old_type.replace('/', '.')
    return old_type


def _map_action_name(old_name):
    """Map old CamelCase action name to new snake_case name."""
    if old_name in _ACTION_NAME_MAP:
        return _ACTION_NAME_MAP[old_name]
    # Fallback: convert CamelCase to snake_case
    s1 = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', old_name)
    return re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


def _discover_actions(nodes):
    """Scan all edges and return a dict of unique action configs.

    Returns ``{new_action_name: action_type_dotted}`` for every
    distinct ``(action, action_type)`` pair found in edges.

    If the new action name has a known default action_type (e.g.
    ``row_traversal`` -> ``NavigateThroughPoses``), that default
    takes priority over the old edge-level ``action_type``, which
    was often incorrect in legacy maps.
    """
    actions = {}  # new_name -> dotted action_type
    for node_entry in nodes:
        nd = node_entry.get('node', {})
        for edge in nd.get('edges', []):
            old_name = edge.get('action', '')
            old_type = edge.get('action_type', '')
            new_name = _map_action_name(old_name)

            if new_name in actions:
                continue

            # Prefer the known default for this action name; fall
            # back to the old edge-level type only for unknown actions.
            if new_name in _DEFAULT_ACTION_TYPES:
                actions[new_name] = _DEFAULT_ACTION_TYPES[new_name]
            elif old_type:
                actions[new_name] = _normalise_action_type(old_type)
            else:
                actions[new_name] = 'nav2_msgs.action.NavigateToPose'

    return actions


def _build_actions_section(discovered, bt_defs):
    """Build the top-level ``actions`` dict from discovered actions.

    Parameters
    ----------
    discovered : dict
        ``{action_name: dotted_action_type}`` from ``_discover_actions``.
    bt_defs : dict
        ``{bt_name: xml_string}`` of available BT definitions.
    """
    actions = {}
    for name, action_type in sorted(discovered.items()):
        bt_name = _ACTION_BT_MAP.get(name)
        # If no predefined BT mapping, pick a reasonable default
        if bt_name is None:
            bt_name = 'default_bt'

        # Determine goal template based on action type.
        # The template mirrors the ROS 2 goal structure: pose/poses
        # contain PoseStamped dicts with header and pose sub-fields.
        _pose_stamped_tpl = {
            'header': {'frame_id': '${node.nav_frame}'},
            'pose': '${node.pose}',
        }
        if name in _SINGLE_POSE_ACTIONS:
            goal_tpl = {
                'pose': dict(_pose_stamped_tpl),
            }
        elif name in _MULTI_POSE_ACTIONS:
            goal_tpl = {
                'poses': [dict(_pose_stamped_tpl)],
            }
        else:
            # Guess from action_type name
            type_tail = action_type.rsplit('.', 1)[-1]
            if 'Through' in type_tail or 'Waypoint' in type_tail:
                goal_tpl = {'poses': [dict(_pose_stamped_tpl)]}
            else:
                goal_tpl = {'pose': dict(_pose_stamped_tpl)}

        if bt_name in bt_defs:
            goal_tpl['behavior_tree'] = '${definitions.%s}' % bt_name

        actions[name] = {
            'composable': _DEFAULT_COMPOSABLE.get(name, False),
            'action_type': action_type,
            'action_server': _DEFAULT_ACTION_SERVERS.get(
                name, '/' + name,
            ),
            'action_goal_template': goal_tpl,
        }
    return actions


def _build_definitions(discovered, bt_dir, bt_defs):
    """Build the ``definitions`` dict.

    If ``bt_dir`` is given, load ``.xml`` files from it.
    Otherwise, use built-in defaults for referenced BT names.
    """
    needed = set()
    for name in discovered:
        bt_name = _ACTION_BT_MAP.get(name, 'default_bt')
        needed.add(bt_name)

    definitions = {}

    if bt_dir and os.path.isdir(bt_dir):
        for fname in sorted(os.listdir(bt_dir)):
            if fname.endswith('.xml'):
                stem = fname[:-4]
                path = os.path.join(bt_dir, fname)
                with open(path, 'r') as fh:
                    definitions[stem] = fh.read()
        # Also add any needed builtins not found on disk
        for bt_name in needed:
            if bt_name not in definitions and bt_name in bt_defs:
                definitions[bt_name] = bt_defs[bt_name]
    else:
        for bt_name in sorted(needed):
            if bt_name in bt_defs:
                definitions[bt_name] = bt_defs[bt_name]

    return definitions


def _convert_node(node_entry):
    """Convert a single node entry from old to new format.

    - Removes ``action_type`` from edges.
    - Maps edge ``action`` names to snake_case.
    - Removes ``localise_by_topic`` and ``parent_frame`` from the node.
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


def convert_tmap(old_data, bt_dir=None):
    """Convert an old-format topological map dict to the new format.

    Parameters
    ----------
    old_data : dict
        Parsed YAML of an old-format topological map.
    bt_dir : str or None
        Optional path to a directory of ``.xml`` BT files.

    Returns
    -------
    dict
        New-format topological map.
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

    # -- Discover actions from edges ---------------------------------
    old_nodes = old_data.get('nodes', [])
    discovered = _discover_actions(old_nodes)

    # -- Definitions (BT XML) ---------------------------------------
    definitions = _build_definitions(
        discovered, bt_dir, _BUILTIN_BT_DEFS,
    )
    new_data['definitions'] = definitions

    # -- Actions -----------------------------------------------------
    new_data['actions'] = _build_actions_section(
        discovered, definitions,
    )

    # -- Nodes -------------------------------------------------------
    new_data['nodes'] = [_convert_node(n) for n in old_nodes]

    return new_data


# =====================================================================
# CLI
# =====================================================================

def main(args=None):
    """Entry point for ``convert_tmap.py``."""
    parser = argparse.ArgumentParser(
        description=(
            'Convert a legacy topological map YAML to the new '
            'map-driven format with definitions and actions sections.'
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
            'Recommended extension: .tmap2.yaml'
        ),
    )
    parser.add_argument(
        '--bt-dir',
        default=None,
        help=(
            'Directory containing .xml BT files to embed as '
            'definitions.  File stems become definition names '
            '(e.g. default_bt.xml -> definitions.default_bt).'
        ),
    )
    parser.add_argument(
        '--no-definitions',
        action='store_true',
        help=(
            'Omit the definitions section (no inline BT XML).  '
            'Useful if BTs are managed externally.'
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

    # Quick check: if it already has 'actions', it may already be new format
    if 'actions' in old_data and 'definitions' in old_data:
        print(
            "Warning: input already has 'actions' and 'definitions' "
            "sections -- it may already be in the new format.",
            file=sys.stderr,
        )

    # -- Convert -----------------------------------------------------
    new_data = convert_tmap(old_data, bt_dir=parsed.bt_dir)

    if parsed.no_definitions:
        new_data.pop('definitions', None)
        # Remove BT refs from action templates
        for act_cfg in new_data.get('actions', {}).values():
            tpl = act_cfg.get('action_goal_template', {})
            tpl.pop('behavior_tree', None)

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
