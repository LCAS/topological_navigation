# Minimal Topological Navigation Wrapper

This repository has been reduced to two minimal ROS 2 packages:

- `topological_navigation`: thin wrapper on top of `nav2_route`
- `topological_navigation_msgs`: one action definition for named-waypoint goals

## Scope

- Route planning is delegated to `nav2_route` (`ComputeRoute`)
- Edge-level BT selection is read from `config/edge_behaviors.yaml`
- Consecutive edges with the same BT are merged into one execution segment
- Action server accepts waypoint names and plans shortest routes for each consecutive pair

## Build and run (native)

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch topological_navigation minimal_route_wrapper.launch.py
```

## Build and run (Docker)

Dockerfile is pinned to ROS 2 Humble.

```bash
docker build -t topo-min .
docker run --rm -it topo-min
```

## Build and run (Docker Compose with TurtleBot sim)

This starts a full TurtleBot3 simulation with Nav2 bringup, `nav2_route`,
and the topological wrapper.

```bash
docker compose up --build
```

The compose stack launches:

- `nav2_bringup` TurtleBot simulation (`tb3_simulation_launch.py`)
- Nav2 core stack including `route_server` with the configured graph
- `route_bt_wrapper`
- `closest_node_publisher`

Execution mode is controlled per goal using the action field
`execute_navigation` (`false` = plan only, `true` = plan + execute).

## Action API

Action name:

- `/execute_named_waypoints`

Action type:

- `topological_navigation_msgs/action/ExecuteNamedWaypoints`

Example goal (plan only):

```bash
ros2 action send_goal /execute_named_waypoints \
	topological_navigation_msgs/action/ExecuteNamedWaypoints \
	"{waypoint_names: ['A', 'B', 'D'], execute_navigation: false}"
```

Example goal (plan and execute):

```bash
ros2 action send_goal /execute_named_waypoints \
	topological_navigation_msgs/action/ExecuteNamedWaypoints \
	"{waypoint_names: ['A', 'B', 'C', 'D'], execute_navigation: true}"
```

## Closest Node Publisher

`closest_node_publisher` runs in the same launch file and continuously subscribes to:

- `/robot_pose` (`geometry_msgs/PoseStamped`)
- `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)

It publishes:

- `/closest_node` (`topological_navigation_msgs/msg/ClosestNode`)

Where `ClosestNode` contains:

- `node_name`
- `distance`

Example:

```bash
ros2 topic echo /closest_node
```
