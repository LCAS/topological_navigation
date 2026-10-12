# Setting ROS 2 Parameters from Edge Actions (`set_ros_params`)

`set_ros_params` lets a topological edge action change ROS 2 parameters on **any
node** while a segment executes, and restores the previous values when the
segment exits (success, failure, or cancellation).

It can be declared in two places:

| Where | Scope |
|---|---|
| Action config (`actions:` in the tmap or in the sidecar config file) | Global — every edge that uses that action |
| Edge `properties` in the topomap | Override for that segment only |

The edge-level declaration is merged over the action-level one **per node and
per parameter**, so an edge can flip a single parameter and still inherit the
rest of the action defaults.

---

## 1. Global — per action

In [`config/topological_navigation_config.yaml`](../config/topological_navigation_config.yaml)
(or the inline `actions:` section of the tmap):

```yaml
actions:
  row_traversal:
    composable: true
    action_type: nav2_msgs.action.NavigateThroughPoses
    action_server: /navigate_through_poses
    set_ros_params:
      collision_monitor:
        FootprintApproach.enabled: false
    action_goal_template:
      poses:
        - header:
            frame_id: '${node.nav_frame}'
          pose: '${node.pose}'
      behavior_tree: '${definitions.row_traversal_bt}'
```

Every `row_traversal` segment now disables `FootprintApproach` on
`/collision_monitor` for the duration of the segment and re-enables it (restores
the pre-segment value) afterwards.

Several nodes and parameters can be set at once:

```yaml
    set_ros_params:
      collision_monitor:
        FootprintApproach.enabled: false
        PolygonSlow.action_type: none
      controller_server:
        FollowPath.max_robot_speed: 0.4
```

---

## 2. Per edge — in the topomap

The same key inside an edge's `properties`:

```yaml
nodes:
  - node:
      name: N3
      edges:
        # Re-enable it for this row only (action default was false)
        - edge_id: N3_N4
          action: row_traversal
          node: N4
          properties:
            set_ros_params:
              collision_monitor:
                FootprintApproach.enabled: true

        # No declaration -> inherits the action-level default (false)
        - edge_id: N3_N9
          action: row_traversal
          node: N9
```

> **Segment merging:** consecutive edges with the same action are merged into a
> single goal, and only the **first non-empty** `properties` dict in the segment
> is used.  Put `set_ros_params` on the **entry edge** of the row; subsequent
> edges without properties are transparent and inherit it.

---

## 3. Accepted syntaxes

All three styles work in both the action config and the edge properties.

**Node-keyed mapping** (recommended):

```yaml
set_ros_params:
  collision_monitor:
    FootprintApproach.enabled: false
```

**List of entries** — `param`/`value` or a `params` mapping; the node name may
be written with or without a leading `/`:

```yaml
set_ros_params:
  - node: /collision_monitor
    param: FootprintApproach.enabled
    value: false
  - node: controller_server
    params:
      FollowPath.max_robot_speed: 0.4
```

**Flat mapping** — targets the goal-checker node (`goal_checker_node`
parameter, default `controller_server`):

```yaml
set_ros_params:
  FollowPath.max_robot_speed: 0.4
```

Supported value types: `bool`, `int`, `float`, `str`.

---

## 4. Precedence

When more than one source sets the same parameter, the last one wins:

1. Node `properties` goal tolerances (`xy_goal_tolerance` / `yaw_goal_tolerance`)
2. Action `ros_parameters` (edge property → goal-checker parameter mapping)
3. Action `set_ros_params`
4. Edge `properties.set_ros_params` — highest priority

---

## 5. Verifying

Parameter clients for non-goal-checker nodes are created on first use and
cached.  Watch the navigation server log:

```
[PARAM] Parameter clients created for node 'collision_monitor'
[PARAM] Segment 'row_traversal': setting ['FootprintApproach.enabled'] on 'collision_monitor'
[PARAM] Restoring 1 parameter(s) on 1 node(s) to pre-segment values
```

Or query the target node directly while the robot is in the row:

```bash
ros2 param get /collision_monitor FootprintApproach.enabled
```

All parameter operations are best-effort: if the target node's parameter
services are unavailable the segment still executes with the currently
configured values.

---

## 6. `set_ros_params` vs `ros_parameters`

| | `ros_parameters` | `set_ros_params` |
|---|---|---|
| Target node | goal-checker node only | any node |
| Value source | an edge property | a constant in the config / edge |
| Declared in | action config only | action config and/or edge properties |

Use `ros_parameters` when the value varies per edge (e.g. `max_speed`), and
`set_ros_params` when a fixed setting must hold while an action runs.

See also [NAVIGATION2.md](NAVIGATION2.md) — *Dynamic ROS 2 Parameter Binding*.
