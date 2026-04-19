# Topological Navigation

A ROS 2 framework for **graph-based topological navigation** of autonomous mobile robots. Unlike traditional metric navigation that operates in continuous coordinate space, topological navigation represents the environment as a graph of discrete **nodes** (waypoints) connected by **edges** (traversable paths), enabling high-level route planning, named-location navigation, and pluggable edge actions (e.g., Nav2 goals, row traversal, door opening).

Originally developed for the [STRANDS](https://strands-project.eu/) long-term autonomy project, this framework is actively used in agricultural robotics for autonomous navigation in vineyards, orchards, and crop rows.

> **ROS 2 Only** — version 4.0.0+ is ROS 2 exclusive (Humble / Iron). For ROS 1 support use version 3.x or earlier.

---

## Table of Contents

- [Topological Navigation](#topological-navigation)
  - [Table of Contents](#table-of-contents)
  - [Architecture Overview](#architecture-overview)
  - [Packages](#packages)
    - [topological\_navigation (Core)](#topological_navigation-core)
    - [topological\_navigation\_msgs](#topological_navigation_msgs)
    - [topological\_navigation\_visual](#topological_navigation_visual)
    - [topological\_nav\_simulator](#topological_nav_simulator)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Build from Source](#build-from-source)
    - [Python Dependencies](#python-dependencies)
  - [Launching the System](#launching-the-system)
    - [Full Stack (with Simulator)](#full-stack-with-simulator)
    - [Standalone Simulator](#standalone-simulator)
    - [Visualiser Only](#visualiser-only)
    - [On a Real Robot](#on-a-real-robot)
    - [Sending Navigation Goals](#sending-navigation-goals)
  - [ROS 2 Node Parameters](#ros-2-node-parameters)
    - [topological\_navigation (navigation2.py)](#topological_navigation-navigation2py)
    - [topological\_localisation (localisation2.py)](#topological_localisation-localisation2py)
    - [topological\_map\_manager\_2 (map\_manager2.py)](#topological_map_manager_2-map_manager2py)
    - [manual\_tmapping\_node (manual\_topomapping.py)](#manual_tmapping_node-manual_topomappingpy)
    - [topological\_map\_visualiser (topological\_map\_visualiser.py)](#topological_map_visualiser-topological_map_visualiserpy)
    - [route\_visualiser / occupancy\_visualiser (topological\_visual.py)](#route_visualiser--occupancy_visualiser-topological_visualpy)
    - [fake\_nav2\_server (fake\_nav2\_server.py — simulator)](#fake_nav2_server-fake_nav2_serverpy--simulator)
  - [ROS 2 Topics](#ros-2-topics)
    - [Published Topics](#published-topics)
    - [Subscribed Topics](#subscribed-topics)
    - [TF Broadcasts](#tf-broadcasts)
  - [ROS 2 Services](#ros-2-services)
  - [ROS 2 Actions](#ros-2-actions)
    - [Action Servers](#action-servers)
    - [Action Clients](#action-clients)
  - [TF Frames](#tf-frames)
  - [Custom Message, Service \& Action Definitions](#custom-message-service--action-definitions)
    - [Messages (.msg)](#messages-msg)
    - [Services (.srv)](#services-srv)
    - [Actions (.action)](#actions-action)
  - [Topological Map Format](#topological-map-format)
    - [Flexible Properties System](#flexible-properties-system)
    - [Map Validation](#map-validation)
  - [Command-Line Utilities](#command-line-utilities)
  - [Testing](#testing)
  - [Contributing](#contributing)
  - [License](#license)

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                        Topological Navigation Stack                  │
│                                                                      │
│  ┌────────────────┐   ┌──────────────────┐   ┌───────────────────┐  │
│  │  Map Manager   │──▶│  Localisation     │──▶│  Navigation       │  │
│  │  (map_manager2)│   │  (localisation2)  │   │  (navigation2)    │  │
│  │                │   │                   │   │                   │  │
│  │ Loads YAML map │   │ Determines        │   │ A* route planning │  │
│  │ Publishes to   │   │ current/closest   │   │ Edge action exec  │  │
│  │ /topological_  │   │ node via KD-tree  │   │ GotoNode action   │  │
│  │ map_2 topic    │   │ + influence zones │   │ server            │  │
│  └────────────────┘   └──────────────────┘   └────────┬──────────┘  │
│                                                        │             │
│                                                        ▼             │
│                                              ┌───────────────────┐  │
│                                              │  Nav2 Stack       │  │
│                                              │  (or Simulator)   │  │
│                                              │  NavigateToPose   │  │
│                                              │  NavigateThrough  │  │
│                                              │  FollowWaypoints  │  │
│                                              └───────────────────┘  │
│                                                                      │
│  ┌────────────────────┐   ┌──────────────────────────────────────┐  │
│  │  Map Visualiser    │   │  Fake Nav2 Simulator                 │  │
│  │  (visual package)  │   │  (simulator package)                 │  │
│  │  RViz markers +    │   │  Virtual robot + fake action servers │  │
│  │  interactive edit  │   │  for testing without real hardware   │  │
│  └────────────────────┘   └──────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

**Data Flow:**
1. **Map Manager** loads a `.tmap2.yaml` file and publishes it as a JSON string on `/topological_map_2`
2. **Localisation** subscribes to the map, builds a NetworkX graph with KD-tree spatial indexing, and continuously publishes which node the robot is at (`/current_node`, `/closest_node`)
3. **Navigation** receives `GotoNode` action goals, computes A\* routes, and executes edge actions by calling Nav2 action servers
4. **Visualiser** renders the topological map as interactive RViz markers for viewing and editing

---

## Packages

### topological_navigation (Core)

| | |
|---|---|
| **Type** | `ament_python` |
| **Version** | 5.0.0 |
| **Purpose** | Core navigation stack: map management, localisation, route planning, navigation execution |

**Key modules:**

| Module | Description |
|--------|-------------|
| `scripts/navigation2.py` | Main navigation action server — receives `GotoNode` goals, plans routes (A\*), and executes edge actions via Nav2 |
| `scripts/localisation2.py` | Topological localisation — determines current/closest node using KD-tree + influence zone checks |
| `scripts/map_manager2.py` | Loads `.tmap2.yaml` maps, validates them against JSON schema, publishes to ROS topics |
| `scripts/manual_topomapping.py` | Joystick-driven waypoint recording for field mapping |
| `networkx_utils.py` | NetworkX graph construction, KD-tree spatial indexing, point-in-polygon, edge distance calculations |
| `navigation_graph.py` | Navigation state machine, route planning, segment merging |
| `tmap_utils.py` | YAML map loading utilities |
| `validate_map.py` | CLI tool for topological map validation against schema |
| `convert_tmap.py` | Convert between topological map format versions |

### topological_navigation_msgs

| | |
|---|---|
| **Type** | `ament_cmake` |
| **Version** | 3.0.5 |
| **Purpose** | ROS 2 message, service, and action interface definitions |

Defines 16 message types, 9 service types, and 3 action types used across the navigation stack. See [Custom Message, Service & Action Definitions](#custom-message-service--action-definitions) below for full details.

### topological_navigation_visual

| | |
|---|---|
| **Type** | `ament_python` |
| **Version** | 1.0.0 |
| **Purpose** | RViz visualisation and interactive map editing |

**Key executables:**

| Executable | Description |
|-----------|-------------|
| `topological_map_visualiser.py` | Unified visualiser + interactive drag-and-drop editor for topological maps in RViz |
| `topological_visual.py` | Route and occupied-node visualisation |
| `policy_marker.py` | Policy arrow visualisation for MDP planning |

### topological_nav_simulator

| | |
|---|---|
| **Type** | `ament_python` |
| **Version** | 1.0.0 |
| **Purpose** | Fake Nav2 action servers with a virtual robot for testing without real hardware |

Provides `NavigateToPose`, `NavigateThroughPoses`, and `FollowWaypoints` action servers that simulate robot movement, publishing TF transforms, odometry, and RViz markers. The simulated robot smoothly interpolates toward goal poses at a configurable speed.

---

## Installation

### Prerequisites

- **ROS 2** Humble or Iron
- **Python 3.8+**
- **Nav2** (for real robot deployment; the simulator replaces it for testing)

### Build from Source

```bash
# Create workspace
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/LCAS/topological_navigation.git

# Install dependencies
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

### Python Dependencies

| Package | Min Version | Purpose |
|---------|------------|---------|
| `networkx` | 2.5 | Graph data structures and algorithms |
| `scipy` | 1.5 | KD-tree spatial indexing |
| `numpy` | 1.19 | Numerical operations |
| `jsonschema` | — | Map schema validation |

---

## Launching the System

### Full Stack (with Simulator)

Launch the complete topological navigation system with a virtual robot (no real hardware needed):

```bash
ros2 launch topological_navigation topological_navigation.launch.py
```

This starts (in order with delays for dependency readiness):

| Delay | Node | Description |
|-------|------|-------------|
| 0 s | `topological_map_manager_2` | Loads and publishes the topological map |
| 2 s | `topological_localisation` | Starts localisation |
| 2 s | `fake_nav2_server` | Virtual robot + fake Nav2 action servers |
| 3 s | `topological_navigation` | Navigation action server |
| 4 s | `topological_map_visualiser` | Interactive RViz map editor |
| 5 s | `rviz2` | RViz with pre-configured display layout |

**Launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `map_path` | `<share>/config/mixed_actions_map.yaml` | Absolute path to a `.tmap2.yaml` topological map |
| `rviz_config` | `<share>/rviz/topological_navigation.rviz` | Path to the RViz config file |
| `initial_x` | `0.0` | Initial robot X position (metres) |
| `initial_y` | `0.0` | Initial robot Y position (metres) |
| `initial_yaw` | `0.0` | Initial robot yaw (radians) |

**Example with a custom map:**

```bash
ros2 launch topological_navigation topological_navigation.launch.py \
    map_path:=/path/to/my_map.tmap2.yaml \
    initial_x:=5.0 initial_y:=3.0
```

### Standalone Simulator

Launch only the fake Nav2 server (useful when developing against a simulated robot):

```bash
ros2 launch topological_nav_simulator fake_nav2.launch.py
```

**Launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_speed` | `0.8` | Simulated linear speed (m/s) |
| `angular_speed` | `1.5` | Simulated angular speed (rad/s) |
| `goal_tolerance` | `0.15` | Goal position tolerance (m) |
| `update_rate` | `30.0` | TF/marker publish rate (Hz) |
| `initial_x` | `0.0` | Initial robot X position |
| `initial_y` | `0.0` | Initial robot Y position |
| `initial_yaw` | `0.0` | Initial robot yaw (rad) |
| `map_frame` | `map` | Map TF frame |
| `odom_frame` | `odom` | Odometry TF frame |
| `base_frame` | `base_link` | Robot base TF frame |

### Visualiser Only

Launch the topological map visualiser standalone:

```bash
ros2 launch topological_navigation_visual topological_map_visualiser.launch.py \
    map_file:=/path/to/map.tmap2.yaml edit_mode:=true
```

**Launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `map_file` | `''` | Path to `.tmap2.yaml` file (empty = subscribe to topic) |
| `auto_save` | `false` | Auto-save changes every 30 seconds |
| `marker_scale` | `0.5` | Scale factor for RViz markers |
| `edit_mode` | `true` | Enable interactive drag-and-drop editing |
| `nav_action_name` | `/topological_navigation` | GotoNode action name for click-to-navigate |

### On a Real Robot

When using a real robot with Nav2, launch the navigation stack without the simulator:

```bash
# 1. Start Nav2 (robot-specific)
ros2 launch <your_robot_pkg> navigation.launch.py

# 2. Start the map manager
ros2 run topological_navigation map_manager2.py /path/to/map.tmap2.yaml

# 3. Start localisation
ros2 run topological_navigation localisation2.py

# 4. Start topological navigation
ros2 run topological_navigation navigation2.py

# 5. (Optional) Start visualiser
ros2 run topological_navigation_visual topological_map_visualiser.py
```

### Sending Navigation Goals

Once the system is running, send a robot to a named node:

```bash
# Navigate to a node called "node3"
ros2 action send_goal /topological_navigation \
    topological_navigation_msgs/action/GotoNode \
    "{target: 'node3', no_orientation: false}"
```

---

## ROS 2 Node Parameters

### topological_navigation (navigation2.py)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_dist_to_closest_edge` | double | `1.0` | Maximum distance (m) to consider an edge as valid origin for navigation |
| `default_boundary_left` | double | `0.5` | Default left boundary offset for row corridor polygons |
| `default_boundary_right` | double | `0.5` | Default right boundary offset for row corridor polygons |
| `route_algorithm` | string | `"astar"` | Path planning algorithm: `"astar"` or `"dijkstra"` |
| `route_weight_attr` | string | `"weight"` | Edge attribute used as cost for path planning |

### topological_localisation (localisation2.py)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `LocalisationThrottle` | int | `1` | Process every Nth pose callback (throttle rate) |
| `OnlyLatched` | bool | `true` | Only publish when values actually change |
| `base_frame` | string | `"base_link"` | Robot base TF frame for localisation lookups |

### topological_map_manager_2 (map_manager2.py)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cache_topological_maps` | bool | `false` | Cache maps locally to `~/.ros/topological_maps` |
| `auto_write_topological_maps` | bool | `false` | Automatically save map on any modification |
| `topological_map2_name` | string | `""` | Current map name identifier |
| `topological_map2_path` | string | `""` | Filesystem path where maps are stored |

### manual_tmapping_node (manual_topomapping.py)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tmap` | string | `""` | Map YAML filename |
| `tmap_dir` | string | `""` | Directory to save map files |
| `site_name` | string | `""` | Site or farm name |
| `node_thresh` | double | `0.5` | Minimum distance (m) between nodes when recording |
| `lock_btn` | int | `6` | Joystick lock button index |
| `add_btn` | int | `1` | Joystick add-node button index |
| `remove_btn` | int | `2` | Joystick remove-node button index |
| `gen_map_btn` | int | `3` | Joystick generate-map button index |
| `topic_joy` | string | `"/joy"` | Joystick input topic |
| `topic_pose` | string | `"/gps_base/odometry"` | Robot pose input topic |
| `topic_imu` | string | `"/gps_base/yaw"` | IMU yaw input topic |

### topological_map_visualiser (topological_map_visualiser.py)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_file` | string | `""` | Path to `.tmap2.yaml` file for editing (empty = subscribe to topic) |
| `auto_save` | bool | `false` | Automatically save map every 30 seconds |
| `marker_scale` | double | `0.5` | Scale factor for RViz node/edge markers |
| `edit_mode` | bool | `true` | Enable interactive drag-and-drop node editing |
| `nav_action_name` | string | `"/topological_navigation"` | GotoNode action name for click-to-navigate context menu |

### route_visualiser / occupancy_visualiser (topological_visual.py)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tmap` | string | *(required)* | Path to a `.tmap2.yaml` topological map file |

### fake_nav2_server (fake_nav2_server.py — simulator)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_speed` | double | `0.8` | Simulated linear speed (m/s) |
| `angular_speed` | double | `1.5` | Simulated angular speed (rad/s) |
| `goal_tolerance` | double | `0.15` | Position tolerance for goal reached (m) |
| `update_rate` | double | `30.0` | TF and marker publish rate (Hz) |
| `initial_x` | double | `0.0` | Initial robot X position |
| `initial_y` | double | `0.0` | Initial robot Y position |
| `initial_yaw` | double | `0.0` | Initial robot yaw (radians) |
| `map_frame` | string | `"map"` | Map TF frame |
| `odom_frame` | string | `"odom"` | Odometry TF frame |
| `base_frame` | string | `"base_link"` | Robot base TF frame |

---

## ROS 2 Topics

### Published Topics

| Topic | Type | Publisher Node | Description |
|-------|------|---------------|-------------|
| `/topological_map_2` | `std_msgs/String` | Map Manager | Full topological map as a JSON string (latched) |
| `/topological_map_schema` | `std_msgs/String` | Map Manager | JSON schema for map validation (latched) |
| `/current_node` | `std_msgs/String` | Localisation | Name of the node the robot is currently inside (influence zone). `"none"` if outside all zones |
| `/closest_node` | `std_msgs/String` | Localisation | Name of the nearest node by Euclidean distance |
| `/closest_node_distance` | `std_msgs/Float32` | Localisation | Distance (m) to the closest node |
| `/closest_edges` | `topological_navigation_msgs/ClosestEdges` | Localisation | Sorted list of closest edge IDs with distances |
| `/current_node/tag` | `std_msgs/String` | Localisation | Semantic tag of the current node (from properties) |
| `/current_edge` | `std_msgs/String` | Navigation | Edge currently being traversed |
| `topological_navigation/Statistics` | `topological_navigation_msgs/NavStatistics` | Navigation | Navigation statistics for completed edges |
| `topological_navigation/Route` | `topological_navigation_msgs/TopologicalRoute` | Navigation | Currently planned route (list of node names) |
| `topological_navigation/move_action_status` | `std_msgs/String` | Navigation | Status of the current Nav2 move action |
| `/boundary_checker` | `geometry_msgs/PolygonStamped` | Navigation | Row corridor boundary polygon for agricultural operations |
| `/robot_operation_current_status` | `std_msgs/String` | Navigation | Current robot operation status |
| `/virtual_robot/markers` | `visualization_msgs/MarkerArray` | Fake Nav2 | Virtual robot body visualisation in RViz |
| `/odometry/global` | `nav_msgs/Odometry` | Fake Nav2 | Simulated global odometry |
| `/virtual_robot/pose` | `geometry_msgs/PoseStamped` | Fake Nav2 | Simulated robot pose |
| `topological_map_visualisation` | `visualization_msgs/MarkerArray` | Map Visualiser | Node and edge markers for RViz |
| `topological_route_visualisation` | `visualization_msgs/MarkerArray` | Route Visualiser | Active route highlight markers |
| `/topological_navigation/visual/occupied_node` | `visualization_msgs/MarkerArray` | Occupancy Visualiser | Occupied node indicators |
| `/tmapping_nodes` | `visualization_msgs/MarkerArray` | Manual Mapping | Recorded waypoint markers |

### Subscribed Topics

| Topic | Type | Subscriber Node | Description |
|-------|------|----------------|-------------|
| `/topological_map_2` | `std_msgs/String` | Localisation, Navigation, Visualiser | Topological map data |
| `/current_node` | `std_msgs/String` | Navigation, Visualiser | Current node for navigation state |
| `/closest_node` | `std_msgs/String` | Navigation, Visualiser | Closest node for route origin |
| `/closest_edges` | `topological_navigation_msgs/ClosestEdges` | Navigation | Edge proximity for navigation decisions |
| `/initialpose` | `geometry_msgs/PoseStamped` | Fake Nav2 | Set the virtual robot's pose (e.g., from RViz **2D Pose Estimate**) |
| `topological_navigation/Route` | `topological_navigation_msgs/TopologicalRoute` | Route Visualiser | Route to highlight |
| `/topological_navigation/occupied_node` | `topological_navigation_msgs/TopologicalOccupiedNode` | Occupancy Visualiser | Nodes occupied by other robots |
| `/joy` | `sensor_msgs/Joy` | Manual Mapping | Joystick input for waypoint recording |
| `mdp_plan_exec/current_policy_mode` | `topological_navigation_msgs/NavRoute` | Policy Marker | Policy route for arrow rendering |

### TF Broadcasts

| From | To | Publisher | Type |
|------|----|----------|------|
| `map` | `odom` | Fake Nav2 | Dynamic TF |
| `odom` | `base_link` | Fake Nav2 | Dynamic TF |
| `parent` | `topo_frame_id` | Map Manager | Static TF |

---

## ROS 2 Services

| Service | Type | Node | Description |
|---------|------|------|-------------|
| `/topological_map_manager2/get_topological_map` | `std_srvs/Trigger` | Map Manager | Returns the current map as a JSON string |
| `/topological_map_manager2/write_topological_map` | `WriteTopologicalMap` | Map Manager | Saves the current map to a YAML file |
| `/topological_map_manager2/switch_topological_map` | `WriteTopologicalMap` | Map Manager | Switches to a different map file |
| `/topological_localisation/localise_pose` | `LocalisePose` | Localisation | Localise an arbitrary pose (returns current + closest node) |
| `/tmapping_robot/save_waypoints` | `std_srvs/Trigger` | Manual Mapping | Save raw waypoints to file |
| `/tmapping_robot/save_map` | `std_srvs/Trigger` | Manual Mapping | Generate and save topological map from waypoints |
| `<visualiser>/save_map` | `std_srvs/Trigger` | Map Visualiser | Save the edited map to YAML |

---

## ROS 2 Actions

### Action Servers

| Action | Type | Node | Description |
|--------|------|------|-------------|
| `/topological_navigation` | `GotoNode` | Navigation | Navigate the robot to a target topological node |
| `/topological_navigation/execute_policy_mode` | `ExecutePolicyMode` | Navigation | Execute a pre-computed policy route |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Fake Nav2 | Simulated single-goal metric navigation |
| `/navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` | Fake Nav2 | Simulated multi-waypoint metric navigation |
| `/follow_waypoints` | `nav2_msgs/FollowWaypoints` | Fake Nav2 | Simulated waypoint-following navigation |

### Action Clients

| Action | Type | Node | Description |
|--------|------|------|-------------|
| *(dynamic from map actions section)* | *(e.g., `nav2_msgs/NavigateToPose`)* | Navigation | Created for each unique `(action_type, action_server)` pair defined in the map |
| `/topological_navigation` | `GotoNode` | Map Visualiser | Click-to-navigate from RViz context menu |

---

## TF Frames

```
map (global frame)
 ├── topo_frame_id (static TF, published by map_manager2)
 │    └── Topological node poses are defined in this frame
 │
 └── odom (dynamic TF, published by simulator or real robot)
      └── base_link (robot body frame)
```

The localisation node resolves the robot pose by looking up `topo_frame_id → base_frame` via `tf2_ros`.

---

## Custom Message, Service & Action Definitions

All definitions reside in the `topological_navigation_msgs` package.

### Messages (.msg)

| Message | Fields | Description |
|---------|--------|-------------|
| **NavRoute** | `string[] source`, `string[] edge_id` | Ordered route: source nodes and edge IDs to traverse |
| **TopologicalRoute** | `string[] nodes` | Ordered list of node names forming a route |
| **ClosestEdges** | `string[] edge_ids`, `float32[] distances` | Sorted edges by proximity with distances |
| **NavStatistics** | `string edge_id`, `string status`, `string origin`, `string target`, `string final_node`, `string date_started`, `string date_at_node`, `string date_finished`, `float64 time_to_waypoint`, `float64 operation_time`, `string topological_map` | Detailed statistics for one edge traversal |
| **CurrentEdge** | `Header header`, `string edge_id`, `bool active`, `bool result` | Currently active edge status |
| **GotoNodeFeedback** | `string route` | Navigation progress feedback (remaining route) |
| **ExecutePolicyModeFeedback** | `string current_wp`, `uint8 status` | Policy execution progress |
| **ExecutePolicyModeGoal** | `Header header`, `NavRoute route` | Policy execution goal with header |
| **TopologicalMap** | `string name`, `string map`, `string pointset`, `string last_updated`, `TopologicalNode[] nodes` | Full topological map structure |
| **TopologicalNode** | `string name`, `string map`, `string pointset`, `Pose pose`, `float64 yaw_goal_tolerance`, `float64 xy_goal_tolerance`, `Vertex[] verts`, `Edge[] edges`, `string localise_by_topic` | Single topological node with edges and influence zone |
| **Edge** | `string edge_id`, `string node`, `string action`, `float64 top_vel`, `string map_2d`, `float64 inflation_radius`, `string recovery_behaviours_config` | Edge connecting two nodes |
| **Vertex** | `float32 x`, `float32 y` | 2D vertex of an influence zone polygon |
| **AddNodeReq** | `string name`, `Pose pose`, `bool add_close_nodes` | Request to add a node to the map |
| **AddEdgeReq** | `string origin`, `string destination`, `string action`, `string action_type`, `string edge_id` | Request to add an edge between nodes |
| **SetInfluenceZoneReq** | `string name`, `float64[] vertices_x`, `float64[] vertices_y` | Request to set a node's influence zone polygon |
| **UpdateEdgeConfigReq** | `string edge_id`, `string namespace_name`, `string name`, `string value`, `bool value_is_string`, `bool not_reset` | Request to update edge configuration parameters |
| **TopologicalOccupiedNode** | `string[] nodes` | List of currently occupied node names |

### Services (.srv)

| Service | Request → Response | Description |
|---------|--------------------|-------------|
| **LocalisePose** | `Pose pose` → `string current_node, string closest_node` | Determine which node a pose belongs to |
| **GetRouteTo** | `string goal` → `NavRoute route` | Get route from current position to goal node |
| **GetRouteBetween** | `string origin, string goal` → `NavRoute route` | Get route between two named nodes |
| **WriteTopologicalMap** | `string filename, bool no_alias` → `bool success, string message` | Write / save a topological map to disk |
| **EvaluateEdge** | `string state, string edge, bool runtime` → `bool success, bool evaluation` | Evaluate restrictions on an edge |
| **EvaluateNode** | `string state, string node, bool runtime` → `bool success, bool evaluation` | Evaluate restrictions on a node |
| **ReconfAtEdges** | `string edge_id` → `bool success` | Reconfigure parameters at edge traversal |
| **LoadTopoNavTestScenario** | `string pointset` → *(empty)* | Load a test scenario |
| **RunTopoNavTestScenario** | *(empty)* → `bool nav_success, bool nav_timeout, bool graceful_fail, bool human_success, float64 min_distance_to_human, float64 mean_speed, float64 distance_travelled, float64 travel_time` | Run a test scenario and return metrics |

### Actions (.action)

| Action | Goal | Result | Feedback |
|--------|------|--------|----------|
| **GotoNode** | `string target`, `bool no_orientation` | `bool success` | `string route` |
| **ExecutePolicyMode** | `NavRoute route` | `bool success` | `string current_wp`, `uint8 status` |
| **BuildTopPrediction** | `Time start_range`, `Time end_range` | `bool success` | `string result` |

---

## Topological Map Format

Topological maps are YAML files with a `.tmap2.yaml` extension. Here is an abbreviated example:

```yaml
meta:
  last_updated: 08-03-2026_12-00-00
  origin:
    latitude: 53.268
    longitude: -0.524
    altitude: 0

metric_map: my_farm
name: my_farm
pointset: my_farm

transformation:
  topo_frame_id: my_farm      # Child frame for node poses
  parent: map                  # Parent TF frame
  rotation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  translation: {x: 0.0, y: 0.0, z: 0.0}

definitions:
  default_bt: |
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </PipelineSequence>
      </BehaviorTree>
    </root>

actions:
  navigate_to_pose:
    composable: false
    action_type: nav2_msgs.action.NavigateToPose
    action_server: /navigate_to_pose
    action_goal_template:
      pose:
        header:
          frame_id: '${node.nav_frame}'
        pose: '${node.pose}'
      behavior_tree: '${definitions.default_bt}'

nodes:
  - meta:
      map: my_farm
      node: RowEntry_A1
      pointset: my_farm
    node:
      name: RowEntry_A1
      nav_frame: '${transformation.topo_frame_id}'
      pose:
        position: {x: 10.5, y: 5.2, z: 0.0}
        orientation: {w: 0.707, x: 0.0, y: 0.0, z: 0.707}
      properties:                    # Flexible key-value metadata
        xy_goal_tolerance: 0.3
        yaw_goal_tolerance: 0.1
      verts:                         # Influence zone polygon
        - {x: -0.5, y: -0.5}
        - {x:  0.5, y: -0.5}
        - {x:  0.5, y:  0.5}
        - {x: -0.5, y:  0.5}
      edges:
        - edge_id: RowEntry_A1_RowEnd_A1
          node: RowEnd_A1            # Target node name
          action: navigate_to_pose   # Action key from 'actions' section
          properties:
            max_speed: 0.5
```

**Key sections:**
- **`meta`**: Map metadata (timestamps, origin coordinates)
- **`transformation`**: TF frame for all node poses (published as a static transform)
- **`definitions`**: Reusable templates (e.g., Nav2 behavior trees)
- **`actions`**: Named action configurations — each maps to a ROS 2 action server with a goal template supporting variable substitution (`${node.pose}`, `${definitions.default_bt}`, etc.)
- **`nodes`**: List of topological nodes, each with pose, properties, influence zone vertices, and outgoing edges

### Flexible Properties System

Both nodes and edges support an optional `properties` dictionary. Properties are freeform key-value pairs — no schema changes needed:

```yaml
properties:
  xy_goal_tolerance: 0.3          # Navigation tolerance
  semantics: "row_entry"          # Semantic label
  roboflow:                       # Namespaced properties
    enabled: true
    confidence: 0.7
```

Always access properties defensively in code:
```python
props = node["node"].get("properties", {})
tolerance = props.get("xy_goal_tolerance", 0.5)
```

See [doc/PROPERTIES.md](topological_navigation/doc/PROPERTIES.md) for full documentation.

### Map Validation

```bash
ros2 run topological_navigation validate_map.py /path/to/map.tmap2.yaml -v
```

---

## Command-Line Utilities

| Command | Description |
|---------|-------------|
| `ros2 run topological_navigation validate_map.py <map.yaml> -v` | Validate a map against the JSON schema |
| `ros2 run topological_navigation convert_tmap.py <args>` | Convert between topological map format versions |
| `ros2 action send_goal /topological_navigation topological_navigation_msgs/action/GotoNode "{target: 'node_name'}"` | Send a navigation goal |
| `ros2 topic echo /current_node` | Monitor which node the robot is at |
| `ros2 topic echo /closest_node` | Monitor the nearest node |
| `ros2 service call /topological_localisation/localise_pose topological_navigation_msgs/srv/LocalisePose "{pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}"` | Localise an arbitrary pose |

---

## Testing

```bash
# Build the workspace
colcon build

# Run all tests for the core package
colcon test --packages-select topological_navigation --event-handlers console_direct+

# Run specific tests
cd topological_navigation
pytest test/test_networkx_utils.py -v
pytest test/test_localisation2.py -v

# Python linting
colcon test --packages-select topological_navigation \
    --event-handlers console_direct+ --pytest-args -k "test_flake8 or test_pep257"
```

---

## Contributing

1. Follow **PEP 8** / **PEP 257** for Python code
2. Use `ament_flake8` and `ament_pep257` for validation
3. Add tests for new functionality
4. Validate maps with `validate_map.py`
5. Update documentation when changing behavior

---

## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.
