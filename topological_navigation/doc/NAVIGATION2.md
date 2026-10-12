# navigation2.py: Map-Driven Topological Navigation Server

This document describes the ROS 2 node implemented in `topological_navigation/scripts/navigation2.py`, including how it interacts with the rest of the topological navigation stack.

## Purpose

`navigation2.py` is the execution engine for topological navigation:

- Accepts high-level goals (`GotoNode`, `ExecutePolicyMode`)
- Plans graph routes using NetworkX
- Converts route edges into executable action segments
- Dynamically builds Nav2 action goals from map-defined templates
- Executes each segment through ROS 2 `ActionClient`s
- Publishes route, status, edge, boundary, and timing telemetry

The script is **map-driven**: action behavior, action server bindings, and behavior tree templates come from the topological map YAML (`definitions` + `actions` sections).

---

## Runtime Context in the Stack

`navigation2.py` depends on two upstream nodes at startup:

1. **Map manager (`map_manager2.py`)**
   - Publishes `/topological_map_2` (latched / transient local)
   - Provides the full map content used to build route graph and action config

2. **Localisation (`localisation2.py`)**
   - Publishes `closest_node`, `current_node`, `closest_edges` (latched)
   - Supplies robot topological position and closest-edge heuristic input

Typical launch order (see `launch/topological_navigation.launch.py`): map manager -> localisation -> navigation server.

---

## ROS Interfaces Exposed by navigation2.py

## Subscriptions

- `/topological_map_2` (`std_msgs/String`, transient local)
- `closest_node` (`std_msgs/String`, transient local)
- `closest_edges` (`topological_navigation_msgs/ClosestEdges`, transient local)
- `current_node` (`std_msgs/String`, transient local)

## Publishers

- `topological_navigation/Statistics` (`topological_navigation_msgs/NavStatistics`, latched)
- `topological_navigation/Route` (`topological_navigation_msgs/TopologicalRoute`, best effort)
- `current_edge` (`std_msgs/String`, latched)
- `/boundary_checker` (`geometry_msgs/PolygonStamped`, latched)
- `/robot_operation_current_status` (`std_msgs/String`, latched)
- `topological_navigation/move_action_status` (`std_msgs/String`, latched)
- Action feedback topics for both action servers

## Action servers

- `/<node_name>` (`topological_navigation_msgs/action/GotoNode`)
- `/topological_navigation/execute_policy_mode` (`topological_navigation_msgs/action/ExecutePolicyMode`)

## Service client

- `/<goal_checker_node>/set_parameters` (`rcl_interfaces/srv/SetParameters`)
  - Used to push per-segment ROS 2 parameters (goal tolerances, action
    parameters) and restore previous values after segment completion
- `/<goal_checker_node>/get_parameters` (`rcl_interfaces/srv/GetParameters`)
  - Used to query current ROS 2 parameter values so they can be saved and
    restored around each segment

---

## Main Lifecycle

### 1) Construction and state initialization

`TopologicalNavServer` initializes:

- State machine (`NavStateMachine`) from `navigation_graph.py`
- Internal map/graph buffers (`_tmap`, `_graph`, pending map update buffer)
- Runtime flags (cancel/preempt/goal reached)
- QoS profiles (`_latch`, `_best`)
- Route/status/statistics publishers

### 2) Blocking wait for first map

The node subscribes to `/topological_map_2` and blocks until first map arrives.

On first map application:

- YAML parsed with `_FloatSafeLoader`
- Graph built via `build_graph_from_tmap()` (from `networkx_utils.py`)
- Map ID fields cached (`_topol_map`)
- Map-driven action configuration loaded (`_load_map_config`)

### 3) Build map-driven action clients

From map `actions` section:

- Dynamically imports each `action_type`
- Creates/reuses `ActionClient`s by `action_server`
- Resolves behavior tree template references (`${definitions.<name>}` -> temp XML file)

Result: `self._action_clients[action_name] = {client, action_class, config}`.

### 4) Blocking wait for localisation

Subscribes to `closest_node` and blocks until first localisation message.

Then subscribes to `closest_edges` and `current_node` for continuous updates.

### 5) Action servers become READY

Creates both action servers and transitions state machine to `READY`.

---

## Route Planning and Execution Flow

Before either action entry point can plan or execute a route, the robot pose
must be available in the coarse main-map frame and occupy a known-free (`0`)
cell in the published main occupancy grid. Unknown (`-1`), occupied (`100`),
and out-of-grid positions are outside. This check fails closed: the server
does not plan a route, generate `/topo_map_route_segment`, or send a Nav2 goal
when the pose cannot be proven to be inside the main map. The latched operation
status is set to `OUTSIDE_MAIN_MAP`, `ROBOT_POSE_UNAVAILABLE`, or
`MAIN_MAP_UNAVAILABLE`, as appropriate.

## Goal entry points

- `GotoNode`: plan from current estimated origin to target
- `ExecutePolicyMode`: execute caller-provided route (after sanity checks)

## Origin determination

`_determine_origin(target)` priority:

1. `current_node` if known
2. Closest-edge heuristic if edge distance <= `max_dist_to_closest_edge`
3. `closest_node`

For equal-distance dual edges, compares route distance from candidate sources.

## Route planning

Uses `plan_route()` from `navigation_graph.py` with parameters:

- `route_algorithm`: `astar` (default) or `dijkstra`
- `route_weight_attr`: edge attribute used as cost (default `weight`)

## Segment generation

From planned nodes:

1. `get_route_edges()` to expand node list into edge dicts
2. `merge_action_segments()` to group consecutive compatible edges
   - Controlled by map action config `composable`
   - Edges with empty or absent `properties` are *transparent*: they merge freely
     into any segment regardless of that segment's parameters
   - Two edges that each carry different non-empty `properties` force a split,
     ensuring each resulting segment has a consistent parameter set

## Segment execution

For each segment:

- Transition to action-specific execution state (`ACTION_TO_STATE`)
- Publish boundary polygon when boundary properties exist
- Apply ROS 2 parameters at segment entrance and restore them after exit (`_apply_segment_parameters` / `_restore_segment_parameters`), on the goal-checker node and on any node named by `set_ros_params`
- Build goal dynamically from map template (`_build_segment_goal`)
- Dispatch goal with `_send_nav2_goal`
- Publish move status + nav statistics
- Fail/cancel handling updates state machine and feedback
- Parameters are restored in a `try/finally` block, so they are always
  undone regardless of segment outcome (success, failure, cancellation)

---

## Mid-Navigation Map Updates

`navigation2.py` supports runtime map updates with deferred application:

- Incoming map messages are buffered (`_pending_map_msg`)
- If robot is navigating, update is postponed to safe points (between segments)
- At safe point, `_apply_pending_map()` rebuilds graph/config
- Remaining route validity checked by `_validate_remaining_route()`

If remaining nodes/edges no longer exist:

- Current execution returns `False`
- `_map_updated_during_nav` is set
- `_navigate()` triggers a full replan from current situation

This design avoids mutating route structures while a segment is in flight.

---

## Boundary Polygon Behavior

Boundary publishing is delegated to `compute_boundary_polygon()` in `navigation_graph.py`.

Trigger condition:

- Any edge in segment contains `properties.boundary_left` or `properties.boundary_right`

Behavior:

- Computes corridor polygon around segment waypoints
- Publishes to `/boundary_checker` in node nav frame
- Publishes empty polygon when no boundary applies (clears stale boundary)

Defaults controlled by parameters:

- `default_boundary_left`
- `default_boundary_right`

---

## Map Schema Expectations Relevant to navigation2.py

Although validation is performed upstream in map manager, `navigation2.py` expects:

- `nodes[*].node.name`
- `nodes[*].node.pose.{position,orientation}`
- `nodes[*].node.edges[*].{edge_id,node,action}`
- `actions` dictionary with per-action config
- Optional `definitions` dictionary for inline BT XML
- Optional `navigation_config_file` path pointing at a second YAML file
  that contains the top-level `definitions` and `actions` sections
- `transformation.topo_frame_id` (or fallback) for pose frame defaults

The recommended sidecar name is `topological_navigation_config.yaml`. It can
be referenced from the map with `navigation_config_file` or passed explicitly
to launch/CLI tools with `navigation_config_file` /
`--navigation-config-file`. Relative sidecar paths are resolved against the
main map file.

Action config shape (typical):

```yaml
actions:
  navigate_to_pose:
    action_type: nav2_msgs.action.NavigateToPose
    action_server: /navigate_to_pose
    composable: false
    action_goal_template:
      pose:
        header:
          frame_id: ${node.nav_frame}
        pose: ${node.pose}
      behavior_tree: ${definitions.default_bt}
```

The optional `ros_parameters` key in an action config maps **edge property
names** to **ROS 2 parameter names** on the goal-checker node.  When a segment
is entered the navigation server reads the current parameter values, applies the
values from the segment's edge properties, and restores the originals after the
segment exits — regardless of whether the segment succeeded, failed, or was
cancelled.

```yaml
actions:
  row_traversal:
    composable: true
    action_type: nav2_msgs.action.NavigateThroughPoses
    action_server: /navigate_through_poses
    ros_parameters:
      # edge property name → ROS 2 parameter path on goal_checker_node
      max_speed: FollowPath.max_robot_speed
    action_goal_template:
      poses:
        - header:
            frame_id: ${node.nav_frame}
          pose: ${node.pose}
      behavior_tree: ${definitions.row_traversal_bt}
```

With this config, an edge that carries `properties: {max_speed: 0.3}` will
cause the server to set `FollowPath.max_robot_speed = 0.3` before executing the
segment and restore the previous value afterwards.

Edge `properties` are optional on every edge.  Edges without properties (or
with empty properties `{}`) are *transparent* from a parameter perspective —
they inherit the effective parameters of the segment they are merged into.  This
means you only need to set parameters on the **first edge of an aisle** and they
will apply for the entire aisle segment:

```yaml
# Entry edge – carries the speed limit for the whole aisle
- edge_id: N3_N4
  action: row_traversal
  node: N4
  properties:
    max_speed: 0.3   # applied for the whole N3→N4→N5 segment

# Subsequent edge – no properties, inherits max_speed: 0.3
- edge_id: N4_N5
  action: row_traversal
  node: N5
  # properties: {}  (absent or empty — transparent)
```

---

## Dynamic ROS 2 Parameter Binding

`navigation2.py` applies two categories of ROS 2 parameters at segment
entrance and restores them after exit.

### Node-level goal tolerances

If the **target node** of a segment carries `xy_goal_tolerance` or
`yaw_goal_tolerance` in its `properties`, those values are pushed to the
goal-checker node before the segment executes:

```yaml
nodes:
  - node:
      name: TightDockingNode
      properties:
        xy_goal_tolerance: 0.02
        yaw_goal_tolerance: 0.01
```

The node-level parameters that receive these values are configurable:

- `xy_tolerance_param` (default `goal_checker.xy_goal_tolerance`)
- `yaw_tolerance_param` (default `goal_checker.yaw_goal_tolerance`)

### Edge-level action parameters

Any edge property listed in an action's `ros_parameters` map is pushed before
the segment runs.  The mapping is `{edge_property_name: ros2_param_name}`.
Only properties actually present in the segment's effective parameters (see
segment merging rules) are applied.

### Arbitrary node parameters — `set_ros_params`

`ros_parameters` can only target the goal-checker node and only forwards values
that come from an edge property.  `set_ros_params` lifts both restrictions: it
sets **constant values** on **any node**, for the duration of the segment.

Declare it on the action to apply it to every edge using that action:

```yaml
actions:
  row_traversal:
    composable: true
    action_type: nav2_msgs.action.NavigateThroughPoses
    action_server: /navigate_through_poses
    set_ros_params:
      collision_monitor:
        FootprintApproach.enabled: false
    action_goal_template: ...
```

…and/or on an individual edge to override the action-level value.  The merge is
per node **and** per parameter, so an edge can flip one parameter while keeping
the rest of the action's defaults:

```yaml
- edge_id: N3_N4
  action: row_traversal
  node: N4
  properties:
    set_ros_params:
      collision_monitor:
        FootprintApproach.enabled: true   # override just for this segment
```

Three authoring styles are accepted in both places:

| Style | Example |
|---|---|
| Node-keyed mapping | `{collision_monitor: {FootprintApproach.enabled: false}}` |
| Flat mapping (goal-checker node) | `{FollowPath.max_robot_speed: 0.3}` |
| List of entries | `[{node: collision_monitor, param: FootprintApproach.enabled, value: false}]` |

List entries accept either `param` + `value` or a `params` mapping, and the
`node` may be written with or without a leading `/`.  Values may be `bool`,
`int`, `float` or `str`.

Parameter clients for non-goal-checker nodes are created on first use and
cached.  As with the other sources, the previous values are read back before the
change and restored when the segment exits.  The precedence order when several
sources set the same parameter is:

1. node goal tolerances → 2. `ros_parameters` → 3. action `set_ros_params` →
4. edge `set_ros_params` (highest).

Worked examples: [SET_ROS_PARAMS.md](SET_ROS_PARAMS.md).

### Save / restore guarantee

Before any parameter is changed, its current value is queried via
`GetParameters`.  After the segment exits (in a `try/finally`), the saved
values are restored via `SetParameters`.  Both operations are best-effort:
if the parameter service is unavailable the segment still executes with the
currently configured values and no restoration is attempted.

---

## Module Interaction Details

## With `networkx_utils.py`

- `build_graph_from_tmap()` provides normalized graph attributes used by planner and goal builders:
  - Node coordinates, orientation, properties, `nav_frame`
  - Edge metadata (`edge_id`, `action`, `action_type`, `properties`, `weight`)

`navigation2.py` relies on these attributes for:

- shortest-path planning
- route edge extraction
- dynamic pose generation
- frame resolution
- property-based tolerances

## With `navigation_graph.py`

Directly uses:

- `NavState`, `NavStateMachine`, `ACTION_TO_STATE`
- `plan_route()`
- `get_route_edges()`
- `merge_action_segments()`
- `compute_boundary_polygon()`
- `get_route_distance()`

This module holds most pure planning/state logic; `navigation2.py` orchestrates ROS I/O and action execution.

## With `tmap_utils.py`

Uses:

- `get_node_from_tmap2()` as fallback when graph-based node lookup fails
- `get_edge_from_id_tmap2()` for policy-mode route edge resolution and segment preflight

## With `localisation2.py`

Consumes localisation outputs:

- `closest_node` for route start fallback
- `current_node` for strongest origin evidence
- `closest_edges` for edge-based origin heuristic

If localisation is unavailable, server intentionally blocks during startup waiting for first message.

## With `map_manager2.py`

Depends on `/topological_map_2` as source of truth.

QoS compatibility is important:

- map manager publishes with transient local durability
- navigation server subscribes with transient local durability
- late joiner still receives latest map

---

## Concurrency and Thread Safety

Key mechanisms:

- `MultiThreadedExecutor`
- Reentrant callback groups for map and action servers
- `_map_lock` (`RLock`) protecting pending/current map mutation

Design choice:

- Nav2 futures are polled with `time.sleep()` in `_send_nav2_goal`
- avoids `spin_once()` conflict with external executor ownership

---

## Parameters

Declared and loaded in `_declare_parameters()` / `_load_parameters()`:

- `max_dist_to_closest_edge` (double, default `1.0`)
- `default_boundary_left` (double, default `0.5`)
- `default_boundary_right` (double, default `0.5`)
- `route_algorithm` (string, `astar`|`dijkstra`, default `astar`)
- `route_weight_attr` (string, default `weight`)
- `goal_checker_node` (string, default `controller_server`)
- `xy_tolerance_param` (string, default `goal_checker.xy_goal_tolerance`)
- `yaw_tolerance_param` (string, default `goal_checker.yaw_goal_tolerance`)

---

## Failure/Recovery Semantics

Common failure points and behavior:

- Missing target node -> immediate `FAILED`
- No route found -> immediate `FAILED`
- Action type import failure in map config -> action skipped/unavailable
- Nav2 server unavailable -> segment `FAILED`
- Mid-route map invalidates remaining path -> force replan
- Cancel/preempt -> propagate cancel to Nav2 and transition `CANCELLED`

Statistics are still published on failed segments when sufficient context exists.

---

## Extension Guide

Safe extension points:

1. **New edge action types**
   - Add action definition to map `actions`
   - Ensure `action_type` import path is valid
   - Provide compatible `action_goal_template`

2. **Custom route costs**
   - Add edge property used as weight
   - Set `route_weight_attr` accordingly

3. **Boundary behavior tuning**
   - Use edge properties `boundary_left/right`
   - or change default boundary parameters

4. **Goal tolerance policies**
   - Provide per-node tolerances in node `properties`
   - map to correct Nav2 parameter names via `xy_tolerance_param` / `yaw_tolerance_param`

5. **Dynamic ROS 2 parameters per segment**
   - Add a `ros_parameters` key to the relevant action config
   - Map edge property names to ROS 2 parameter paths on the goal-checker node
   - Set the desired property on the entry edge of each aisle / segment
   - Edges without properties are transparent and inherit the segment's values

6. **Parameters on other nodes (e.g. disabling a collision-monitor polygon)**
   - Add `set_ros_params` to the action config for the global default
   - Override it in an edge's `properties` where a specific segment differs

---

## Backward Compatibility with Existing Topological Maps

This release is **fully backward compatible**.  Existing maps that do not use
edge `properties` or the `ros_parameters` action key will behave identically
to previous versions:

| Map feature | Behaviour |
|---|---|
| Edges with no `properties` key | Treated as empty (`{}`); transparent — always merge with adjacent same-action edges |
| Edges with `properties: {}` | Same as above |
| Edges with `properties: null` | Normalised to `{}`; same as above |
| `ros_parameters` absent from action config | No dynamic parameters are applied or restored |
| `set_ros_params` absent from action config and edge properties | No dynamic parameters are applied or restored |
| Node `properties` without `xy_goal_tolerance` / `yaw_goal_tolerance` | No tolerance change |

**No changes to map YAML files are required.**

The only behaviour change relative to the *prior* implementation is that the
old `_set_goal_tolerances` / `_tolerance_set_cb` pair (which only set, never
restored) has been replaced by `_apply_segment_parameters` /
`_restore_segment_parameters`, which additionally saves the previous values and
restores them after each segment.  For maps that relied on goal tolerances
being *permanent* (i.e. set once and left) this means tolerances now reset to
their pre-segment values after each segment.  If you want a tolerance to
persist across segments, either set it identically on every relevant node or
configure it as the static default in your Nav2 controller configuration.

---

## Operational Tips

- If node hangs at startup, confirm latched map/localisation topics are present.
- If planning fails unexpectedly, verify edge directionality in map graph.
- If frame mismatches occur, check map `transformation.topo_frame_id`, node `nav_frame`, and Nav2 frame expectations.
- For policy execution, ensure route starts at current/closest node.

---

## Related Files

- `topological_navigation/scripts/navigation2.py`
- `topological_navigation/navigation_graph.py`
- `topological_navigation/networkx_utils.py`
- `topological_navigation/scripts/localisation2.py`
- `topological_navigation/scripts/map_manager2.py`
- `topological_navigation/tmap_utils.py`
- `topological_navigation/launch/topological_navigation.launch.py`

---

Last Updated: 2026-09-02
