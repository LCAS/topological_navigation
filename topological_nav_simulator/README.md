# topological_nav_simulator

Fake Nav2 action servers with a virtual robot for testing topological navigation **without a real robot or the full Nav2 stack**.

## Motivation

When developing and testing topological navigation behaviour (route planning, edge actions, localisation), you need a Nav2 action server responding to `NavigateToPose` / `NavigateThroughPoses` goals. Spinning up the full Nav2 stack with Gazebo is heavy and slow. This package provides lightweight **drop-in replacements** that:

- Accept the same Nav2 action goals  
- Simulate robot movement at configurable speed  
- Publish TF (`map → odom → base_link`) so `localisation2` works  
- Publish `/odometry/global` so `edge_action_manager2` works  
- Show a **virtual robot marker** in RViz (cube + direction arrow + breadcrumb trail)  
- Support cancel / preemption  

## Architecture

```
┌──────────────────────────────────────────────┐
│           fake_nav2_server (node)             │
│                                               │
│  Action Servers:                              │
│    /navigate_to_pose      (NavigateToPose)    │
│    /navigate_through_poses(NavigateThroughPoses)│
│    /follow_waypoints      (FollowWaypoints)   │
│                                               │
│  Publishers:                                  │
│    TF: map → odom → base_link                 │
│    /odometry/global       (Odometry)          │
│    /virtual_robot/markers (MarkerArray)        │
│    /virtual_robot/pose    (PoseStamped)        │
│                                               │
│  Subscribers:                                 │
│    /initialpose  (teleport from RViz)         │
└──────────────────────────────────────────────┘
```

## Quick Start

```bash
# Build
cd /path/to/workspace
colcon build --packages-select topological_nav_simulator
source install/setup.bash

# Run
ros2 launch topological_nav_simulator fake_nav2.launch.py

# In another terminal — send a test goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_speed` | double | 0.8 | Linear speed (m/s) |
| `angular_speed` | double | 1.5 | Angular speed (rad/s) |
| `goal_tolerance` | double | 0.15 | Distance to snap to goal (m) |
| `update_rate` | double | 30.0 | TF/marker publish rate (Hz) |
| `initial_x` | double | 0.0 | Starting X position |
| `initial_y` | double | 0.0 | Starting Y position |
| `initial_yaw` | double | 0.0 | Starting yaw (rad) |
| `map_frame` | string | `map` | Map TF frame |
| `odom_frame` | string | `odom` | Odometry TF frame |
| `base_frame` | string | `base_link` | Robot base TF frame |

## RViz Visualization

The virtual robot appears as:
- **Blue cube** — robot body  
- **Red arrow** — heading direction  
- **Green ring** — footprint  
- **Blue trail** — breadcrumb path  
- **🤖 Sim Robot** — floating label  

Use the **2D Pose Estimate** tool in RViz to teleport the robot to any location.

An RViz config is provided:
```bash
rviz2 -d $(ros2 pkg prefix topological_nav_simulator)/share/topological_nav_simulator/config/fake_nav2.rviz
```

## Usage with Topological Navigation

Launch the full topological navigation stack, replacing the real Nav2 with this simulator:

```bash
# Terminal 1: Fake Nav2
ros2 launch topological_nav_simulator fake_nav2.launch.py \
    initial_x:=10.5 initial_y:=5.2

# Terminal 2: Map manager + localisation + navigation
ros2 run topological_navigation map_manager2.py --ros-args -p topological_map2_name:=my_map
ros2 run topological_navigation localisation2.py
ros2 run topological_navigation navigation2.py

# Terminal 3: Send topological nav goal
ros2 action send_goal /topological_navigation \
    topological_navigation_msgs/action/GotoNode \
    "{target: 'WayPoint1'}"
```

## Movement Simulation

When a goal is received, the robot:
1. **Rotates** to face the target position  
2. **Drives straight** toward it at `robot_speed`  
3. **Rotates** to the goal's final orientation  

For `NavigateThroughPoses`, it moves through each waypoint sequentially.

Feedback is published with `distance_remaining` so the topological navigation stack can monitor progress.
