# AI Coding Agent Guide for topological_navigation

This document provides comprehensive information about the topological_navigation repository to help AI coding agents understand and work effectively with the codebase.

## Project Overview

**topological_navigation** is a ROS 2 framework for topological navigation of autonomous mobile robots. Unlike traditional metric navigation that operates in continuous coordinate space, topological navigation represents the environment as a graph of discrete nodes (waypoints) connected by edges (paths).

### Purpose
This system enables robots to:
- Navigate between named locations using high-level graph-based planning
- Represent complex environments efficiently as topological maps
- Execute domain-specific actions when traversing edges (e.g., opening doors, climbing ramps, row operations in agriculture)
- Leverage flexible metadata for application-specific customization
- Support agricultural robot operations including in-row navigation and boundary detection

### Originally Developed For
STRANDS (Spatio-Temporal Representations and Activities for Cognitive Control in Long-term Scenarios) - a long-term autonomy project for mobile robots. Currently actively used in agricultural robotics applications, particularly for autonomous navigation in vineyards, orchards, and crop rows.

### Recent Development Focus (aoc branch)
The `aoc` branch focuses on:
- Enhanced edge action management for agricultural operations
- Row operation handling with boundary node selection
- Improved closest node detection algorithms
- AI agent preparation with comprehensive documentation
- Flexible node/edge property system for domain-specific metadata

## Repository Structure

This is a multi-package ROS 2 workspace with four packages:

```
topological_navigation/
├── topological_navigation/        # Core package (Python)
│   ├── topological_navigation/    # Python modules
│   │   ├── scripts/              # ROS 2 executable scripts
│   │   │   ├── navigation2.py   # Main navigation action server
│   │   │   ├── map_manager.py   # Map loading and publishing
│   │   │   ├── localisation2.py # Topological localisation
│   │   │   ├── map_publisher.py # Map topic publisher
│   │   │   └── actions_bt.py    # Behaviour tree action types
│   │   ├── edge_action_manager2.py  # Edge action execution (67KB, complex)
│   │   ├── manager2.py          # Map management core (58KB)
│   │   ├── route_search2.py     # A* path planning
│   │   ├── topological_map.py   # Map data structures
│   │   ├── load_maps_from_yaml.py # YAML map loader
│   │   └── restrictions_impl.py # Navigation restrictions
│   ├── config/                   # YAML schemas and templates
│   │   └── schema2.json         # JSON schema for map validation
│   ├── doc/                      # Documentation
│   │   └── PROPERTIES.md        # Properties system guide
│   ├── launch/                   # ROS 2 launch files
│   └── test/, tests/            # Unit and integration tests
│
├── topological_navigation_msgs/  # Message definitions
│   ├── msg/                      # ROS 2 messages
│   ├── srv/                      # ROS 2 services
│   └── action/                   # ROS 2 actions
│
├── topological_rviz_tools/       # Interactive map editing (C++/Qt)
│   ├── src/                      # RViz plugins (tools and panels)
│   ├── include/                  # C++ headers
│   └── scripts/                  # Python interface
│
└── topological_utils/            # Utilities (Python)
    └── topological_utils/        # Map creation and management tools
```

## Technology Stack

- **Language**: Python 3 (core navigation), C++ (RViz tools)
- **Framework**: ROS 2 (Humble, Iron distributions)
- **Build System**: ament_python, ament_cmake
- **Testing**: pytest, launch_pytest, ament linters
- **GUI**: Qt (for RViz panels)
- **Data Format**: YAML for topological maps (`.tmap2.yaml` extension)
- **Validation**: JSON Schema (`config/schema2.json`)
- **Key Dependencies**:
  - `nav2_msgs` - Nav2 navigation actions and messages
  - `geometry_msgs`, `nav_msgs`, `sensor_msgs` - ROS 2 standard messages
  - `tf_transformations` - Coordinate frame transformations
  - `visualization_msgs` - RViz markers
  - `topological_navigation_msgs` - Custom message definitions

## Core Concepts

### 1. Topological Maps

Topological maps are the fundamental data structure in this system.

**Structure**:
- **Nodes**: Represent discrete locations (waypoints) with:
  - Unique name (string identifier)
  - Pose (position + orientation) in a coordinate frame
  - Parent frame (typically "map")
  - Influence zone (polygon defining the node's spatial extent)
  - Flexible properties dictionary (optional)
  - List of outgoing edges

- **Edges**: Represent navigable connections with:
  - Target node reference (node name)
  - Action name (e.g., "NavigateToPose", "OpenDoor", "RowOperation")
  - Action type (ROS 2 message/action type)
  - Edge ID (unique identifier)
  - Flexible properties dictionary (optional)

**File Format**: YAML with `.tmap2.yaml` extension

**Example Node**:
```yaml
nodes:
  - meta:
      map: vineyard_01
      node: RowEntry_A1
      pointset: vineyard_01
    node:
      name: RowEntry_A1
      parent_frame: map
      pose:
        position: {x: 10.5, y: 5.2, z: 0.0}
        orientation: {w: 0.707, x: 0.0, y: 0.0, z: 0.707}
      properties:
        xy_goal_tolerance: 0.3
        yaw_goal_tolerance: 0.1
        semantics: "row_entry"
        roboflow:
          enabled: true
          confidence: 0.7
      edges:
        - edge_id: RowEntry_A1_RowEnd_A1
          node: RowEnd_A1
          action: RowOperation
          action_type: nav2_msgs/action/NavigateToPose
          properties:
            max_speed: 0.5
            row_type: "vineyard"
```

### 2. Flexible Properties System

Both nodes and edges support an optional `properties` dictionary for application-specific metadata.

**Key Features**:
- Completely flexible schema (any key-value pairs)
- Optional - properties can be omitted entirely
- Supports namespacing for organization
- Enables domain-specific customization without schema changes
- Validated against JSON schema for structural correctness

**Common Property Patterns**:
- **Flat structure**: `{capacity: 2, zone: "A"}`
- **Domain namespaces**: `{restrictions: {capacity: 2}, semantics: {zone: "A"}}`
- **Package namespaces**: `{roboflow: {enabled: true, confidence: 0.7}}`
- **Navigation parameters**: `{xy_goal_tolerance: 0.3, max_speed: 0.8}`

**Important**: Always access properties defensively:
```python
# Safe property access pattern
props = node["node"].get("properties", {})
capacity = props.get("capacity", 1)  # Default to 1

# Nested property access
roboflow_config = props.get("roboflow", {})
enabled = roboflow_config.get("enabled", False)
```

See `topological_navigation/doc/PROPERTIES.md` for comprehensive documentation.

### 3. Navigation Architecture

**Key Components**:

1. **Map Manager** (`scripts/map_manager.py`, `manager2.py`)
   - Loads topological maps from YAML files
   - Publishes map structure to ROS 2 topics (`/topological_map_2`)
   - Handles map updates and modifications
   - Validates map structure against JSON schema
   - Manages node and edge metadata

2. **Localisation** (`scripts/localisation2.py`)
   - Determines robot's current topological node
   - Publishes `/current_node` and `/closest_node`
   - Supports pose-based and topic-based localization
   - Uses influence zones to determine node proximity
   - Critical for navigation start conditions

3. **Navigation** (`scripts/navigation2.py`)
   - Executes topological navigation actions
   - Coordinates with metric navigation (Nav2)
   - Handles edge action execution via EdgeActionManager
   - Implements navigation state machine
   - Provides `/topological_navigation` action server

4. **Route Search** (`route_search2.py`)
   - A* based path planning on topological graph
   - Finds optimal routes between nodes
   - Considers edge properties for path optimization
   - Handles node restrictions and blocked edges
   - Returns ordered list of nodes and edges to traverse

5. **Edge Action Manager** (`edge_action_manager2.py`)
   - Manages execution of edge-specific actions
   - Pluggable action system for custom behaviors
   - Handles Nav2 action clients (NavigateToPose, NavigateThroughPoses, etc.)
   - Implements row operations for agricultural navigation
   - Manages boundary detection and edge side edges
   - **Complex module**: 1,363 lines, extensive ROS 2 action handling

### 4. Key Classes and Their Responsibilities

#### EdgeActionManager2 (`edge_action_manager2.py`)
- **Purpose**: Execute navigation actions for edges, handle agricultural row operations
- **Key Methods**:
  - `execute_action()` - Main action execution entry point
  - `navigate_to_pose()` - Send Nav2 NavigateToPose actions
  - `navigate_through_poses()` - Send Nav2 NavigateThroughPoses actions
  - `handle_row_operations()` - Agricultural row navigation logic
  - `get_boundary_nodes()` - Determine entry/exit boundary nodes for rows
- **Important**: Integrates with Nav2 action server, handles goal status callbacks

#### TopologicalMap (`topological_map.py`)
- **Purpose**: In-memory representation of topological map
- **Key Methods**:
  - `load_from_yaml()` - Load map from YAML file
  - `get_node()` - Retrieve node by name
  - `get_edges_from_node()` - Get all edges from a node
  - `get_node_neighbours()` - Get neighbouring nodes

#### TopologicalRouteSearch2 (`route_search2.py`)
- **Purpose**: A* path planning on topological graph
- **Key Methods**:
  - `search_route()` - Find optimal path between nodes
  - `get_path_cost()` - Calculate path cost with property-based weighting
  - `is_node_blocked()` - Check if node is restricted/blocked

## Development Guidelines

### Code Style

**Python**:
- Follow PEP 8 style guide
- Use PEP 257 docstring conventions
- Type hints encouraged for clarity (especially in new code)
- Validated with `ament_flake8` and `ament_pep257`
- Line length: 120 characters (flexible, follow existing file style)

**C++**:
- Follow ROS 2 C++ style guidelines
- Use modern C++ features (C++14 minimum)
- Follow ROS 2 naming conventions (snake_case for functions/variables)

**ROS 2 Naming Conventions**:
- Node names: lowercase with underscores (e.g., `topological_navigation`)
- Topic names: lowercase with slashes (e.g., `/topological_map_2`)
- Service/action names: CamelCase (e.g., `NavigateToPose`)

### Testing

**Test Structure**:
- `test/` - Unit tests (pytest)
- `tests/` - Integration tests (launch_pytest)
- Test files: `test_*.py` or `*_tester.py`

**Running Tests**:
```bash
# Run all tests for a package
colcon test --packages-select topological_navigation

# Run with verbose output
colcon test --packages-select topological_navigation --event-handlers console_direct+

# Run specific test
colcon test --packages-select topological_navigation --pytest-args -k test_route_search
```

**Writing Tests**:
- Follow existing test patterns in `test/` and `tests/` directories
- Use pytest fixtures for common setup
- Integration tests should use launch_pytest for ROS 2 node lifecycle
- Mock external dependencies (Nav2 action servers, etc.)

### Building

```bash
# Build all packages in workspace
cd /path/to/workspace
colcon build

# Build specific package
colcon build --packages-select topological_navigation

# Build with symlink install (faster for Python changes)
colcon build --symlink-install

# Clean build
colcon build --cmake-clean-cache
```

**Important**: After building, source the workspace:
```bash
source install/setup.bash
```

### Linting

```bash
# Python linting (flake8)
colcon test --packages-select topological_navigation \
  --event-handlers console_direct+ \
  --pytest-args -k test_flake8

# PEP 257 docstring checks
colcon test --packages-select topological_navigation \
  --event-handlers console_direct+ \
  --pytest-args -k test_pep257

# Run all linters
colcon test --packages-select topological_navigation \
  --event-handlers console_direct+
```

## Common Tasks for AI Agents

### Task 1: Add New Property to Topological Maps

**Goal**: Add a new property to node/edge definitions

**Steps**:
1. **Update Map Files** (`.tmap2.yaml`):
   ```yaml
   properties:
     my_new_property: value
   ```

2. **Update Code to Use Property**:
   ```python
   props = node["node"].get("properties", {})
   my_value = props.get("my_new_property", default_value)
   ```

3. **Update Documentation**: Add property description to `doc/PROPERTIES.md`

4. **Consider Schema Update**: If property should be validated, update `config/schema2.json`

**Key Files**:
- Map YAML files (*.tmap2.yaml)
- `doc/PROPERTIES.md`
- `config/schema2.json` (optional)
- Code files using the property

### Task 2: Modify Edge Action Behavior

**Goal**: Change how edges are executed during navigation

**Key File**: `topological_navigation/edge_action_manager2.py`

**Common Modifications**:
- Add new action type support
- Modify Nav2 goal parameters
- Add pre/post-action behaviors
- Implement custom action handlers

**Example Pattern**:
```python
def execute_custom_action(self, edge_data):
    """Execute custom edge action"""
    # Extract properties
    props = edge_data.get("properties", {})
    
    # Prepare action goal
    goal = CustomAction.Goal()
    goal.parameter = props.get("custom_param", default)
    
    # Send action
    self._send_goal(goal)
```

**Testing**: Add integration test in `tests/` directory

### Task 3: Add New ROS 2 Node/Script

**Goal**: Create a new ROS 2 executable script

**Steps**:
1. **Create Script**: Add to `topological_navigation/topological_navigation/scripts/`
2. **Add to setup.py**:
   ```python
   entry_points={
       'console_scripts': [
           'my_new_node = topological_navigation.scripts.my_new_node:main',
       ],
   },
   ```
3. **Follow ROS 2 Node Pattern**:
   ```python
   import rclpy
   from rclpy.node import Node
   
   class MyNewNode(Node):
       def __init__(self):
           super().__init__('my_new_node')
           self.get_logger().info('Node started')
   
   def main(args=None):
       rclpy.init(args=args)
       node = MyNewNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```

4. **Add Launch File** (optional): `launch/my_new_node.launch.py`
5. **Add Test**: `test/test_my_new_node.py`

### Task 4: Fix Navigation Issues

**Common Issues**:

1. **Robot not localising correctly**:
   - Check `localisation2.py` - verify influence zone configuration
   - Check `/closest_node` topic - is closest node being published?
   - Verify robot pose is being published to correct topic

2. **Edge action not executing**:
   - Check `edge_action_manager2.py` - verify action client is connected
   - Check Nav2 action server is running
   - Verify edge action type matches available action servers
   - Check edge properties for correct action parameters

3. **Route planning fails**:
   - Check `route_search2.py` - verify graph connectivity
   - Check for blocked/restricted nodes
   - Verify source and target nodes exist in map

**Debugging Tools**:
- `/topological_map_2` topic - view current map
- `/current_node` topic - view current localisation
- `/topological_navigation/feedback` - view navigation progress
- RViz markers - visualise topological map

### Task 5: Update Map Schema

**Goal**: Modify the structure of topological maps

**Key File**: `config/schema2.json`

**Steps**:
1. Update JSON schema with new fields
2. Update map loader (`load_maps_from_yaml.py`)
3. Update map validation logic
4. Update example maps in `tests/` directory
5. Update documentation (`doc/PROPERTIES.md`)
6. Run validation tests to ensure backward compatibility

## Important Patterns

### Pattern 1: Safe Property Access

Always use defensive property access to avoid KeyErrors:

```python
# Node properties
node_data = topological_map.get_node("node_name")
props = node_data["node"].get("properties", {})
tolerance = props.get("xy_goal_tolerance", 0.5)  # Default 0.5

# Nested properties
roboflow = props.get("roboflow", {})
enabled = roboflow.get("enabled", False)

# Check existence before use
if "custom_behavior" in props:
    behavior = props["custom_behavior"]
else:
    behavior = default_behavior
```

### Pattern 2: ROS 2 Action Client Usage

Follow the pattern in `edge_action_manager2.py`:

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class MyActionHandler:
    def __init__(self, node):
        self._nav_client = ActionClient(
            node,
            NavigateToPose,
            '/navigate_to_pose'
        )
    
    def send_goal(self, pose):
        # Wait for action server
        self._nav_client.wait_for_server()
        
        # Prepare goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Send goal with feedback/result callbacks
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
```

### Pattern 3: Map Loading and Validation

```python
from topological_navigation.topological_map import TopologicalMap
from topological_navigation.load_maps_from_yaml import load_map_from_yaml

# Load map
map_data = load_map_from_yaml("path/to/map.tmap2.yaml")

# Create TopologicalMap object
topo_map = TopologicalMap()
topo_map.load_from_dict(map_data)

# Access nodes
node = topo_map.get_node("node_name")
edges = topo_map.get_edges_from_node("node_name")
```

### Pattern 4: ROS 2 Parameter Handling

```python
from rcl_interfaces.msg import ParameterDescriptor

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameters with defaults
        self.declare_parameter(
            'map_file',
            '',
            ParameterDescriptor(description='Path to topological map YAML')
        )
        
        # Get parameter value
        map_file = self.get_parameter('map_file').value
```

## Common Pitfalls

### Pitfall 1: Property Access Without Defaults
**Wrong**:
```python
tolerance = node["node"]["properties"]["xy_goal_tolerance"]  # KeyError if missing!
```

**Right**:
```python
props = node["node"].get("properties", {})
tolerance = props.get("xy_goal_tolerance", 0.5)
```

### Pitfall 2: Not Checking Action Server Availability
**Wrong**:
```python
self._nav_client.send_goal_async(goal)  # May fail silently
```

**Right**:
```python
if not self._nav_client.wait_for_server(timeout_sec=5.0):
    self.get_logger().error("Nav2 action server not available")
    return False
self._nav_client.send_goal_async(goal)
```

### Pitfall 3: Hardcoding Frame IDs
**Wrong**:
```python
pose.header.frame_id = "map"  # May not match robot's frame
```

**Right**:
```python
pose.header.frame_id = node_data["node"]["parent_frame"]
```

### Pitfall 4: Ignoring ROS 2 Callback Groups
For concurrent operations in ROS 2, use appropriate callback groups:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# For actions that shouldn't block each other
self.callback_group = ReentrantCallbackGroup()
self._nav_client = ActionClient(
    self,
    NavigateToPose,
    '/navigate_to_pose',
    callback_group=self.callback_group
)
```

## Key Files Reference

### Core Navigation Files
- `scripts/navigation2.py` - Main navigation action server (topological navigation entry point)
- `edge_action_manager2.py` - Edge action execution (complex, 1363 lines)
- `route_search2.py` - A* path planning algorithm
- `scripts/localisation2.py` - Topological localisation node
- `scripts/map_manager.py` - Map loading and publishing node

### Map Data Structures
- `topological_map.py` - In-memory map representation
- `load_maps_from_yaml.py` - YAML map loader
- `manager2.py` - Map management core logic

### Utility Classes
- `dict_tools` (in edge_action_manager2.py) - Nested dictionary operations
- `tmap_utils.py` - Map manipulation utilities
- `route_search.py` - Legacy route search (v1)

### Configuration and Validation
- `config/schema2.json` - JSON schema for map validation
- `doc/PROPERTIES.md` - Properties system documentation

### Testing
- `test/test_navigationcore.py` - Navigation core tests
- `tests/topological_navigation_tester_critical.py` - Critical integration tests
- `tests/map_manager_tester.py` - Map manager tests

## Agricultural Navigation Specifics

### Row Operations
The system includes specialized support for agricultural row navigation:

**Key Features**:
- Boundary node detection for row entry/exit
- Side edge support for parallel row paths
- Roboflow integration for vision-based row detection
- Configurable row operation parameters

**Usage Pattern**:
```yaml
edges:
  - edge_id: entry_to_exit
    node: row_exit
    action: RowOperation
    action_type: nav2_msgs/action/NavigateToPose
    properties:
      row_type: "vineyard"
      roboflow:
        enabled: true
        confidence: 0.7
      max_speed: 0.5
```

**Implementation**: See `edge_action_manager2.py` → `RowOperations` class integration

### Boundary Detection
- `get_boundary_nodes()` in EdgeActionManager2
- Uses side_edges property to identify parallel paths
- Determines optimal entry/exit points for row operations

## Dependencies and External Systems

### ROS 2 Navigation Stack (Nav2)
- **Actions Used**:
  - `NavigateToPose` - Navigate to single goal
  - `NavigateThroughPoses` - Navigate through waypoints
  - `FollowWaypoints` - Follow ordered waypoint list
  - `ComputePathToPose` - Path planning only
  - `ComputePathThroughPoses` - Multi-goal path planning

- **Integration Point**: `edge_action_manager2.py`
- **Assumption**: Nav2 action servers are running and accessible

### Coordinate Frame Requirements
- **Base Frame**: Robot's base link (typically `base_link`)
- **Map Frame**: Global reference frame (typically `map`)
- **TF Tree**: Must have valid transforms between frames
- **Node Poses**: Defined in parent_frame (usually `map`)

### Message Dependencies
- `geometry_msgs` - Pose, PoseStamped, Point, Quaternion
- `nav_msgs` - Odometry, Path
- `std_msgs` - Header, String
- `sensor_msgs` - Various sensor data
- `visualization_msgs` - Marker, MarkerArray (for RViz)

## Debugging and Troubleshooting

### Common ROS 2 Commands

```bash
# List all nodes
ros2 node list

# View node info
ros2 node info /topological_navigation

# List topics
ros2 topic list

# View topic data
ros2 topic echo /topological_map_2
ros2 topic echo /current_node

# Check action servers
ros2 action list

# Send test action goal
ros2 action send_goal /topological_navigation \
  topological_navigation_msgs/action/GotoNode \
  "{target: 'node_name'}"

# View logs
ros2 run rqt_console rqt_console
```

### Log Analysis
- **Info logs**: Normal operation, navigation progress
- **Warn logs**: Recoverable issues, fallback behaviors
- **Error logs**: Action failures, missing data, connectivity issues

### RViz Visualization
Launch RViz to visualize topological map:
```bash
ros2 launch topological_rviz_tools topological_rviz.launch.py
```

**Markers**:
- Red spheres: Nodes
- Green arrows: Edges
- Blue lines: Planned route
- Yellow highlight: Current node

## CI/CD and Quality Assurance

### GitHub Actions Workflows
- YAML schema validation
- Python linting (flake8, pep257)
- Unit and integration tests
- Build verification

### Pre-commit Checks
Before committing, run:
```bash
# Lint
colcon test --packages-select topological_navigation \
  --pytest-args -k "flake8 or pep257"

# Unit tests
colcon test --packages-select topological_navigation
```

## Additional Resources

### Documentation
- `README.md` - Package overview
- `doc/PROPERTIES.md` - Properties system detailed guide
- `.github/copilot-instructions.md` - GitHub Copilot specific guidance
- `REVIEW.md` - Code review guidelines

### Example Maps
- `tests/` directory contains example topological maps
- Look for `.tmap2.yaml` files

### External Links
- ROS 2 Documentation: https://docs.ros.org/
- Nav2 Documentation: https://navigation.ros.org/
- ROS 2 Action Documentation: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## Getting Help

### Internal Resources
1. Check existing code patterns in similar files
2. Read test files for usage examples
3. Review commit history for recent changes: `git log --oneline`

### External Resources
1. ROS 2 Discourse: https://discourse.ros.org/
2. Nav2 GitHub Issues: https://github.com/ros-planning/navigation2/issues
3. ROS Answers: https://answers.ros.org/

## Agent-Specific Guidelines

### When Analyzing This Codebase
1. **Start with map structure**: Understand topological maps first
2. **Follow data flow**: Map → Route Search → Navigation → Edge Actions
3. **Check properties**: Always verify what properties are available/required
4. **Test incrementally**: Small changes, test frequently
5. **Respect ROS 2 patterns**: Use established action/topic/service patterns

### When Making Changes
1. **Preserve backward compatibility**: Especially for map formats
2. **Add tests**: Every new feature needs tests
3. **Update documentation**: Keep AGENTS.md, PROPERTIES.md, README.md in sync
4. **Follow existing patterns**: Don't introduce new patterns without discussion
5. **Consider agricultural use cases**: Changes impact real-world robot operations

### When Debugging
1. **Start with logs**: Check ROS 2 logs for errors/warnings
2. **Verify map structure**: Ensure YAML maps are valid
3. **Check action servers**: Ensure Nav2 is running
4. **Use RViz**: Visualize to understand spatial relationships
5. **Isolate components**: Test map loading, routing, actions separately

---

**Last Updated**: 2026-01-27
**Branch**: agent_prep
**Maintainer**: AI Coding Agents
