# GitHub Copilot Instructions for topological_navigation

This repository contains a ROS 2 topological navigation framework for autonomous mobile robots.

## Repository Overview

This is a multi-package ROS 2 workspace with the following packages:

- **topological_navigation**: Core navigation and mapping functionality (Python)
- **topological_navigation_msgs**: Message, service, and action definitions
- **topological_rviz_tools**: RViz-based interactive map construction tools (C++)
- **topological_utils**: Command-line utilities for map creation and management (Python)

## Code Style and Conventions

### Python Code
- **Style Guide**: Follow PEP 8 and PEP 257 for Python code
- **Linters**: Use `ament_flake8` and `ament_pep257` for validation
- **Testing**: Use `pytest` and `launch_pytest` for tests
- **ROS 2 Patterns**: Follow standard ROS 2 node patterns with lifecycle management
- **Type Hints**: Use type hints where appropriate for better code clarity

### C++ Code
- **Style Guide**: Follow ROS 2 C++ style guidelines
- **Build System**: Uses `ament_cmake` for C++ packages
- **RViz Plugins**: Follow RViz plugin development patterns for tool and panel creation

### File Organization
- Scripts go in `{package}/scripts/` directory
- Python modules in `{package}/{package}/` directory
- Launch files in `{package}/launch/` directory
- Configuration files (YAML schemas, templates) in `{package}/config/` directory
- Tests in `{package}/test/` or `{package}/tests/` directories

## Key Concepts

### Topological Maps
- Maps are stored as YAML files (`.tmap2.yaml` extension)
- Schema validation is available via `validate_map.py`
- Maps consist of **nodes** (waypoints) and **edges** (connections)
- Each node has a pose, properties, and edges to other nodes
- Each edge has an action (e.g., NavigateToPose), action type, and optional properties

### Flexible Properties System
- Both nodes and edges support a flexible `properties` dictionary
- Properties can be namespaced for organization (e.g., `restrictions`, `semantics`, `navigation`)
- Properties are optional and domain-specific
- See `topological_navigation/doc/PROPERTIES.md` for detailed documentation

### Core Components
- **Map Manager**: Manages topological map data and publishes map information
- **Localisation**: Determines robot's current topological node
- **Navigation**: Plans and executes routes through the topological map
- **Route Search**: A* based path planning on topological graphs
- **Edge Actions**: Pluggable actions for traversing edges (e.g., NavigateToPose)

## Building and Testing

### Build the Workspace
```bash
colcon build
```

### Run Linters
```bash
# Python linting
colcon test --packages-select topological_navigation --event-handlers console_direct+ --pytest-args -k "test_flake8 or test_pep257"

# Run all tests for a package
colcon test --packages-select topological_navigation
```

### Run Specific Tests
```bash
# Run pytest tests directly
cd topological_navigation
pytest test/test_navigationcore.py -v
```

### Validate Topological Maps
```bash
ros2 run topological_navigation validate_map.py path/to/map.tmap2.yaml -v
```

## Common Tasks

### Adding New Node Properties
1. Update documentation in `topological_navigation/doc/PROPERTIES.md` with new property descriptions
2. Access properties safely in code using `.get()` with defaults
3. Consider using namespaces to organize properties by domain or package

### Creating New Scripts
1. Add Python script to `{package}/scripts/`
2. Update `entry_points` in `setup.py` to expose as ROS 2 executable
3. Follow ROS 2 node patterns: use `rclpy.init()`, create node class, spin
4. Add appropriate command-line argument parsing

### Modifying Map Schema
1. Update schema in `topological_navigation/config/tmap-schema.yaml`
2. Update validation logic if needed
3. Update templates in `config/template_node_2.yaml` and `config/template_edge.yaml`
4. Update documentation in `doc/PROPERTIES.md`

### Working with RViz Tools (C++)
1. RViz tools inherit from `rviz_common::Tool`
2. RViz panels inherit from `rviz_common::Panel`
3. Use Qt for UI components in panels
4. Register plugins in `plugin_description.xml`

## Important Files

### Configuration
- `config/tmap-schema.yaml`: JSON schema for topological map validation
- `config/template_node_2.yaml`: Template for creating new nodes
- `config/template_edge.yaml`: Template for creating new edges

### Documentation
- `README.md`: Package-level documentation
- `doc/PROPERTIES.md`: Comprehensive guide to the properties system
- `CHANGELOG.rst`: Version history and changes

### Testing
- `test/`: Unit tests and integration tests
- Uses `pytest` for Python tests
- Uses `launch_pytest` for ROS 2 launch-based tests

## CI/CD

- GitHub Actions workflows in `.github/workflows/`
- `ci.yaml`: Builds and tests on ROS 2 Humble and Iron distributions
- `validate-schema.yaml`: Validates topological map schema
- All PRs should pass CI checks before merging

## Dependencies

### ROS 2 Dependencies
- `nav2_msgs`: Navigation 2 message definitions
- `geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`: Standard ROS 2 messages
- `tf_transformations`: Coordinate transformations
- `visualization_msgs`: RViz markers and visualization

### Python Dependencies
- Standard library modules are preferred
- Minimal external dependencies to maintain portability

## Best Practices

### When Adding Features
1. Maintain backward compatibility with existing topological maps
2. Make properties optional with sensible defaults
3. Document new properties or features in relevant documentation files
4. Add validation logic to catch misconfigurations early
5. Add tests for new functionality

### When Modifying Core Logic
1. Understand the impact on navigation behavior
2. Test with sample topological maps
3. Consider edge cases (empty maps, disconnected graphs, invalid actions)
4. Update documentation if behavior changes

### When Working with Maps
1. Always validate maps with `validate_map.py` before deployment
2. Use namespaced properties to avoid conflicts between applications
3. Access properties defensively with `.get()` and default values
4. Document custom properties in your application-specific docs

## Tips for AI Assistants

- This is a ROS 2 package (not ROS 1) - use ROS 2 APIs and patterns
- Topological navigation differs from metric navigation: it operates on graphs of nodes, not continuous space
- Properties are flexible but optional - don't assume they exist
- Many scripts are command-line tools - respect existing argument parsing patterns
- Tests should be added for new functionality, following existing test patterns
- The repository supports multiple ROS 2 distributions (Humble, Iron) - use compatible APIs
