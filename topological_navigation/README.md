# topological_navigation (minimal)

This package is now a minimal wrapper around `nav2_route`.

## What it does

- Starts `nav2_route` and its lifecycle manager
- Computes a route between node IDs
- Groups consecutive edges by Behavior Tree (BT) file
- Executes grouped segments using:
  - `NavigateThroughPoses` when a segment has multiple edges
  - `NavigateToPose` when a segment has a single edge

## Run

```bash
ros2 launch topological_navigation minimal_route_wrapper.launch.py
```

Then send goals to `/execute_named_waypoints` using
`topological_navigation_msgs/action/ExecuteNamedWaypoints`.

## TurtleBot Simulation Run

```bash
ros2 launch topological_navigation turtlebot_route_compose.launch.py
```
