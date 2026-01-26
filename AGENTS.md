# AI Coding Agent Guide for topological_navigation

This document provides comprehensive information about the topological_navigation repository to help AI coding agents understand and work effectively with the codebase.

## Project Overview

**topological_navigation** is a ROS 2 framework for topological navigation of autonomous mobile robots. Unlike traditional metric navigation that operates in continuous coordinate space, topological navigation represents the environment as a graph of discrete nodes (waypoints) connected by edges (paths).

### Purpose
This system enables robots to:
- Navigate between named locations using high-level graph-based planning
- Represent complex environments efficiently as topological maps
- Execute domain-specific actions when traversing edges (e.g., opening doors, climbing ramps)
- Leverage flexible metadata for application-specific customization

### Originally Developed For
STRANDS (Spatio-Temporal Representations and Activities for Cognitive Control in Long-term Scenarios) - a long-term autonomy project for mobile robots.

## Repository Structure

This is a multi-package ROS 2 workspace with four packages:

```
topological_navigation/
├── topological_navigation/        # Core package (Python)
│   ├── topological_navigation/   # Python modules
│   │   ├── scripts/             # ROS 2 executable scripts
│   │   ├── *.py                 # Core classes (route_search, map_marker, etc.)
│   ├── config/                   # YAML schemas and templates
│   ├── doc/                      # Documentation (especially PROPERTIES.md)
│   ├── launch/                   # ROS 2 launch files
│   └── test/, tests/            # Unit and integration tests
│
├── topological_navigation_msgs/  # Message definitions
│   ├── msg/, srv/, action/      # ROS 2 interface definitions
│
├── topological_rviz_tools/       # Interactive map editing (C++/Qt)
│   ├── src/                      # RViz plugins (tools and panels)
│   ├── include/                  # C++ headers
│
└── topological_utils/            # Utilities (Python)
    └── topological_utils/        # Map creation and management tools
```

## Technology Stack

- **Language**: Python 3 (core), C++ (RViz tools)
- **Framework**: ROS 2 (Humble, Iron distributions)
- **Build System**: ament_python, ament_cmake
- **Testing**: pytest, launch_pytest, ament linters
- **GUI**: Qt (for RViz panels)
- **Data Format**: YAML for topological maps
- **Validation**: JSON Schema for map structure

## Core Concepts

### 1. Topological Maps

Topological maps are the fundamental data structure in this system.

**Structure**:
- **Nodes**: Represent discrete locations (waypoints) with:
  - Unique name
  - Pose (position + orientation) in a coordinate frame
  - Influence zone (polygon defining the node's spatial extent)
  - Flexible properties dictionary
  - List of outgoing edges

- **Edges**: Represent navigable connections with:
  - Target node reference
  - Action name (e.g., "NavigateToPose", "OpenDoor")
  - Action type (ROS 2 message/action type)
  - Flexible properties dictionary

**File Format**: YAML with `.tmap2.yaml` extension

**Example**:
```yaml
nodes:
  - meta:
      map: my_environment
      node: ChargingStation
      pointset: my_environment
    node:
      name: ChargingStation
      parent_frame: map
      pose:
        position: {x: 10.0, y: 5.0, z: 0.0}
        orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      properties:
        xy_goal_tolerance: 0.3
        yaw_goal_tolerance: 0.1
        semantics: "charging_station"
      edges:
        - edge_id: ChargingStation_WayPoint1
          node: WayPoint1
          action: NavigateToPose
          action_type: geometry_msgs/PoseStamped
          properties:
            max_speed: 0.5
```

### 2. Flexible Properties System

Both nodes and edges support an optional `properties` dictionary for application-specific metadata.

**Key Features**:
- Completely flexible schema (any key-value pairs)
- Optional - properties can be omitted entirely
- Supports namespacing for organization
- Enables domain-specific customization without schema changes

**Common Property Patterns**:
- **Flat structure**: `{capacity: 2, zone: "A"}`
- **Domain namespaces**: `{restrictions: {capacity: 2}, semantics: {zone: "A"}}`
- **Package namespaces**: `{my_fleet_manager: {priority_zone: true}}`

**Important**: Always access properties defensively:
```python
props = node["node"].get("properties", {})
capacity = props.get("capacity", 1)  # Default to 1
```

See `topological_navigation/doc/PROPERTIES.md` for comprehensive documentation.

### 3. Navigation Architecture

**Key Components**:

1. **Map Manager** (`map_manager2.py`)
   - Loads topological maps from YAML files
   - Publishes map structure to ROS 2 topics
   - Handles map updates and modifications

2. **Localisation** (`localisation2.py`)
   - Determines robot's current topological node
   - Publishes `/current_node` and `/closest_node`
   - Supports pose-based and topic-based localization

3. **Navigation** (`navigation2.py`)
   - Executes topological navigation actions
   - Coordinates with metric navigation (Nav2)
   - Handles edge action execution

4. **Route Search** (`route_search2.py`)
   - A* based path planning on topological graph
   - Finds optimal routes between nodes
   - Considers edge properties for path optimization

5. **Edge Action Manager** (`edge_action_manager.py`)
   - Manages execution of edge-specific actions
   - Pluggable action system for custom behaviors

## Development Guidelines

### Code Style

**Python**:
- Follow PEP 8 style guide
- Use PEP 257 docstring conventions
- Validated with `ament_flake8` and `ament_pep257`
- Type hints encouraged for clarity

**C++**:
- Follow ROS 2 C++ style guidelines
- Use modern C++ features (C++14 minimum)

### Testing

**Test Structure**:
- `test/` or `tests/` directories in each package
- Python: pytest framework
- ROS 2 integration: launch_pytest
- Run: `colcon test --packages-select <package_name>`

**Important**: Maintain existing test infrastructure. Add tests for new features following established patterns.

### Building

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select topological_navigation

# Run tests
colcon test --packages-select topological_navigation
```

### Linting

```bash
# Python linting (flake8, pep257)
colcon test --packages-select topological_navigation \
  --event-handlers console_direct+ \
  --pytest-args -k "test_flake8 or test_pep257"
```

## Common Tasks for AI Agents

### Task: Add New Property to Topological Maps

1. **Document the property** in `topological_navigation/doc/PROPERTIES.md`
   - Add to appropriate table (node or edge properties)
   - Specify type, description, and examples
   - Include code examples for accessing the property

2. **Update code to use the property**:
   ```python
   # Access safely with default
   props = node["node"].get("properties", {})
   new_property = props.get("new_property", default_value)
   ```

3. **Consider namespacing** if the property is application-specific

4. **No schema changes needed** - properties are flexible by design

### Task: Add New ROS 2 Script

1. **Create script** in `{package}/scripts/my_script.py`
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node

   class MyNode(Node):
       def __init__(self):
           super().__init__('my_node')
           # Initialization
   
   def main(args=None):
       rclpy.init(args=args)
       node = MyNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   
   if __name__ == '__main__':
       main()
   ```

2. **Register in setup.py**:
   ```python
   entry_points={
       'console_scripts': [
           'my_script = package_name.scripts.my_script:main',
       ],
   }
   ```

3. **Make executable**: `chmod +x {package}/scripts/my_script.py`

4. **Test**: `ros2 run package_name my_script`

### Task: Modify Map Schema

1. **Update schema** in `topological_navigation/config/tmap-schema.yaml`

2. **Update templates**:
   - `config/template_node_2.yaml`
   - `config/template_edge.yaml`

3. **Update validation logic** if needed in `scripts/validate_map.py`

4. **Document changes** in `doc/PROPERTIES.md` or README

5. **Test with validator**: `ros2 run topological_navigation validate_map.py map.tmap2.yaml -v`

### Task: Fix Failing Tests

1. **Identify failing test**: Check CI logs or run `colcon test`

2. **Run test locally**:
   ```bash
   cd {package}
   pytest test/test_name.py -v
   ```

3. **Fix code or test** as appropriate

4. **Verify fix**: Re-run tests

5. **Run linters** to ensure code quality

### Task: Add C++ RViz Plugin

1. **Create class** inheriting from `rviz_common::Tool` or `rviz_common::Panel`

2. **Implement required methods**:
   - `onInitialize()` for setup
   - Event handlers for interaction

3. **Register plugin** in `plugin_description.xml`

4. **Update CMakeLists.txt** to compile new source files

5. **Test in RViz**: Load custom plugin from RViz GUI

## Important Patterns

### Pattern: Safe Property Access

Always access properties defensively to handle missing properties:

```python
# Node properties
node_props = node["node"].get("properties", {})
tolerance = node_props.get("xy_goal_tolerance", 0.3)

# Namespaced properties
restrictions = node_props.get("restrictions", {})
capacity = restrictions.get("capacity", 1)

# Check existence before use
semantics = node_props.get("semantics")
if semantics is not None:
    # Use semantics
    pass
```

### Pattern: ROS 2 Node Creation

Follow standard ROS 2 patterns:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node initialized')
        
        # Declare parameters
        self.declare_parameter('param_name', 'default_value')
        
        # Create publishers/subscribers
        self.publisher = self.create_publisher(MsgType, 'topic', 10)
        self.subscription = self.create_subscription(
            MsgType, 'topic', self.callback, 10)
    
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg}')
```

### Pattern: Map Validation

Always validate maps before deployment:

```bash
ros2 run topological_navigation validate_map.py my_map.tmap2.yaml -v
```

Validation checks:
- Schema compliance
- Duplicate node names
- Edges pointing to non-existent nodes
- Required fields present

## Key Files Reference

### Configuration
- `topological_navigation/config/tmap-schema.yaml` - JSON schema for map validation
- `topological_navigation/config/template_node_2.yaml` - Node template
- `topological_navigation/config/template_edge.yaml` - Edge template

### Documentation
- `README.md` - Repository and package documentation
- `topological_navigation/doc/PROPERTIES.md` - **Essential** properties system guide
- `CHANGELOG.rst` - Version history

### Core Python Modules
- `route_search2.py` - Path planning algorithm
- `map_marker.py` - Visualization markers
- `edge_action_manager.py` - Edge action execution
- `navigation_stats.py` - Navigation statistics tracking

### Core Scripts
- `localisation2.py` - Topological localization
- `navigation2.py` - Navigation execution
- `map_manager2.py` - Map data management
- `validate_map.py` - Map validation tool
- `visualise_map2.py` - Map visualization

### Testing
- `topological_navigation/test/test_navigationcore.py` - Core navigation tests

## CI/CD

**GitHub Actions** (`.github/workflows/`):
- `ci.yaml` - Builds and tests on multiple ROS 2 distributions (Humble, Iron)
- `validate-schema.yaml` - Validates topological map schema

**Requirements**:
- All tests must pass
- Code must pass linters (flake8, pep257)
- Compatible with ROS 2 Humble and Iron

## Dependencies

**ROS 2 Packages**:
- `nav2_msgs` - Navigation 2 messages
- `geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs` - Core ROS 2 messages
- `visualization_msgs` - RViz visualization
- `tf_transformations` - Coordinate transformations

**Build Dependencies**:
- `ament_cmake` - C++ build system
- `ament_python` - Python build system

**Test Dependencies**:
- `pytest` - Python testing
- `launch_pytest` - ROS 2 launch testing
- `ament_flake8`, `ament_pep257` - Code quality

## Best Practices for AI Agents

### When Modifying Code

1. **Understand the context**: Read related documentation (especially PROPERTIES.md)
2. **Maintain compatibility**: Don't break existing topological maps
3. **Make properties optional**: Provide sensible defaults
4. **Add tests**: Follow existing test patterns
5. **Run linters**: Ensure code quality before committing
6. **Update documentation**: Reflect changes in relevant docs

### When Working with Properties

1. **Access defensively**: Use `.get()` with defaults
2. **Consider namespaces**: Avoid property name conflicts
3. **Document custom properties**: In PROPERTIES.md or app-specific docs
4. **Validate maps**: Use `validate_map.py` after modifications

### When Adding Features

1. **Follow ROS 2 patterns**: Use standard node/topic/service patterns
2. **Minimize dependencies**: Prefer standard library and existing dependencies
3. **Consider multiple distributions**: Code should work on Humble and Iron
4. **Test thoroughly**: Unit tests and integration tests
5. **Update CI if needed**: Ensure new features are tested

### When Fixing Bugs

1. **Identify root cause**: Understand why the bug occurs
2. **Add regression test**: Prevent the bug from reoccurring
3. **Consider edge cases**: Empty maps, invalid data, missing properties
4. **Run full test suite**: Ensure fix doesn't break other functionality

## Common Pitfalls to Avoid

1. **Assuming properties exist**: Always use `.get()` with defaults
2. **Breaking backward compatibility**: Old maps should still work
3. **Hardcoding values**: Use parameters or properties instead
4. **Forgetting ROS 2 lifecycle**: Initialize rclpy, shutdown properly
5. **Ignoring validation**: Always validate maps before deployment
6. **Not updating docs**: Changes should be reflected in documentation
7. **Mixing ROS 1 and ROS 2 APIs**: This is a ROS 2 project only

## Getting Help

- **Documentation**: Start with README files and PROPERTIES.md
- **Code examples**: Look at existing scripts for patterns
- **Tests**: Show expected behavior and usage
- **Schema**: `tmap-schema.yaml` defines valid map structure
- **Templates**: Show minimal valid nodes and edges

## Quick Reference

### Build & Test
```bash
colcon build                                          # Build all packages
colcon test --packages-select topological_navigation  # Test package
```

### Validate Map
```bash
ros2 run topological_navigation validate_map.py map.tmap2.yaml -v
```

### Run Node
```bash
ros2 run topological_navigation script_name.py [args]
```

### Access Properties
```python
props = node["node"].get("properties", {})
value = props.get("key", default)
```

---

**Remember**: This is a topological (graph-based) navigation system, not metric (coordinate-based). Nodes are discrete locations, edges are connections. Properties are flexible and optional. Always code defensively.
