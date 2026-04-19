# Flexible Node and Edge Properties System

This document describes the flexible properties system for topological maps, which allows application-specific metadata to be attached to both nodes and edges without requiring schema modifications.

## Overview

The topological map schema supports optional `properties` dictionaries for both nodes and edges. These properties enable domain-specific customisation while maintaining backwards compatibility with existing maps.

## Node Properties

Node properties are defined in `nodes[].node.properties` as a YAML dictionary. The schema allows any key-value pairs.

### Default Properties

The following properties are commonly used for navigation control:

| Property | Type | Description |
|----------|------|-------------|
| `xy_goal_tolerance` | float | XY position tolerance for goal reaching (metres) |
| `yaw_goal_tolerance` | float | Yaw orientation tolerance for goal reaching (radians) |

### Example Custom Properties

| Property | Type | Description |
|----------|------|-------------|
| `row` | integer | Row identifier (e.g., for agricultural polytunnel scenarios) |
| `semantics` | string | Semantic meaning of the node (e.g., "charging_station", "inspection_point") |
| `zone` | string | Operational zone designation |
| `access_level` | string | Permission level required for access |
| `capacity` | integer | Maximum number of robots that can occupy the node |

### Node Properties Example

```yaml
nodes:
- meta:
    map: riseholme
    node: ChargingStation1
    pointset: riseholme
  node:
    name: ChargingStation1
    parent_frame: map
    pose:
      position: {x: 10.0, y: 5.0, z: 0.0}
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    properties:
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.1
      semantics: "charging_station"
      row: 3
      zone: "A"
      capacity: 2
    edges: []
```

## Edge Properties

Edge properties are defined in `nodes[].node.edges[].properties` as a YAML dictionary. The schema allows any key-value pairs.

### Example Edge Properties

| Property | Type | Description |
|----------|------|-------------|
| `max_speed` | float | Maximum traversal speed (m/s) |
| `priority` | integer | Preference weight for path planning (higher = more preferred) |
| `width` | float | Physical width of the traversable path (metres) |
| `surface_type` | string | Terrain classification (e.g., "concrete", "grass", "gravel") |
| `bidirectional` | boolean | Whether the edge can be traversed in both directions |
| `weather_restrictions` | list | Conditions under which edge should not be used |

### Edge Properties Example

```yaml
edges:
- edge_id: ChargingStation1_WayPoint2
  node: WayPoint2
  action: NavigateToPose
  action_type: geometry_msgs/PoseStamped
  properties:
    max_speed: 0.5
    priority: 10
    surface_type: "concrete"
    bidirectional: true
    weather_restrictions: ["heavy_rain", "snow"]
  # ... other edge fields
```

## Backwards Compatibility

The `properties` field is optional for both nodes and edges. Existing topological maps without properties remain valid and will continue to work without modification.

When properties are not specified:
- Node properties default to standard navigation tolerances if needed by the navigation system
- Edge properties are simply absent (empty dictionary)

## Usage Guidelines

### Naming Conventions

- Use `snake_case` for property names
- Use descriptive, domain-appropriate names
- Avoid abbreviations unless they are widely understood

### Using Namespaces for Properties

To organise properties by functional area or application, use **namespaces**. A namespace is simply a nested dictionary that groups related properties together. This approach:

- Prevents naming conflicts between different systems
- Makes it clear which application or module owns each property
- Improves readability and maintainability
- Supports both domain-based and package-based organisation

#### Namespace Structure

```yaml
properties:
  namespace_name:
    property1: value1
    property2: value2
```

#### Namespace Organisation Approaches

You can organise namespaces in different ways depending on your needs:

1. **Domain-based namespaces** (by concept or functional area):
   ```yaml
   properties:
     restrictions:    # Physical and access constraints
       capacity: 2
       max_external_width: 0.8
     semantics:       # Semantic labels and classifications
       zone: "warehouse"
       type: "charging_station"
   ```

2. **Package-based namespaces** (by ROS package):
   ```yaml
   properties:
     my_fleet_manager:
       priority_zone: true
     my_safety_module:
       emergency_stop_point: false
   ```

3. **Flat structure** (no namespaces):
   ```yaml
   properties:
     capacity: 2
     zone: "warehouse"
   ```

All three approaches are valid. Choose the organisation that best fits your application's needs and team conventions.

#### Common Namespace Examples

**Domain-based namespaces:**

| Namespace | Purpose | Example Properties |
|-----------|---------|-------------------|
| `restrictions` | Physical and access constraints | `capacity`, `max_external_width`, `max_external_height`, `access_level` |
| `semantics` | Semantic labels and classifications | `zone`, `type`, `features`, `environmental_conditions` |
| `navigation` | Navigation-specific parameters | `xy_goal_tolerance`, `yaw_goal_tolerance` |
| `safety` | Safety-related constraints | `restricted_hours`, `emergency_access`, `hazard_zones` |

**Application/package-based namespaces:**

| Namespace | Purpose | Example Properties |
|-----------|---------|-------------------|
| `fleet_management` | Multi-robot coordination properties | `priority_zone`, `queue_enabled` |
| `logistics` | Warehouse/delivery application properties | `pickup_point`, `dropoff_point`, `storage_type` |
| `agriculture` | Agricultural robotics properties | `row_id`, `crop_type`, `irrigation_zone` |

#### Namespaced Node Properties Example

**Package-based organisation:**
```yaml
node:
  name: ChargingStation1
  properties:
    navigation:
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.1
    fleet_management:
      capacity: 2
      priority_zone: true
      queue_enabled: true
    semantics:
      type: "charging_station"
      zone: "A"
    logistics:
      pickup_point: false
      dropoff_point: false
```

**Domain-based organisation (alternative):**
```yaml
node:
  name: PolytunnelRow4_Col2
  properties:
    restrictions:
      capacity: 1
      max_external_width: 0.8  # metres
      max_external_height: 1.5  # metres
      min_internal_width: 1.2   # metres
      min_internal_height: 2.0  # metres
      access_level: "operator"
    semantics:
      zone:
        labels: ['polytunnel', 'growing_area']
        details: {tunnel_id: 4, row_id: 5, column_idx: 2}
      features:
        labels: ['irrigation_point', 'sensor_node']
        details: {sensor_types: ['humidity', 'temperature']}
      environmental:
        labels: ['soil', 'shade']
        details: {soil_type: 'loam', 'shade_percentage': 30}
```

#### Namespaced Edge Properties Example

**Package-based organisation:**
```yaml
edges:
- edge_id: ChargingStation1_WayPoint2
  node: WayPoint2
  properties:
    navigation:
      max_speed: 0.5
    safety:
      width: 1.2
      restricted_hours: ["22:00-06:00"]
    logistics:
      priority: 10
      surface_type: "concrete"
```

**Domain-based organisation (alternative):**
```yaml
edges:
- edge_id: Row4_Col2_Row4_Col3
  node: Row4_Col3
  properties:
    restrictions:
      max_external_width: 0.75  # metres - narrow passage
      max_external_height: 1.8  # metres - low overhead
      capacity: 1  # single robot at a time
    semantics:
      zone:
        labels: ['polytunnel_interior', 'growing_area']
        details: {tunnel_id: 4}
      environmental:
        labels: ['grass', 'uneven']
        details: {surface_quality: 'variable'}
```

#### Accessing Namespaced Properties in Code

```python
# Safe access to namespaced node properties
node_props = node["node"].get("properties", {})

# Access domain-based namespace
restrictions = node_props.get("restrictions", {})
capacity = restrictions.get("capacity", 1)
max_width = restrictions.get("max_external_width")

# Access package-based namespace
fleet_props = node_props.get("fleet_management", {})
priority_zone = fleet_props.get("priority_zone", False)

# Access nested semantic properties
semantics = node_props.get("semantics", {})
zone_info = semantics.get("zone", {})
zone_labels = zone_info.get("labels", [])
zone_details = zone_info.get("details", {})

# Access namespaced edge properties
edge_props = edge.get("properties", {})
restrictions = edge_props.get("restrictions", {})
max_width = restrictions.get("max_external_width")
```

#### Guidelines for Namespace Usage

1. **Choose the right organisation for your use case**:
   - Use **domain-based namespaces** (like `restrictions`, `semantics`) when organising by conceptual categories
   - Use **package-based namespaces** when multiple ROS packages share the same map and need to avoid conflicts
   - Use **flat structure** for simple cases with few properties

2. **Be consistent within your project**: Choose one organisational approach and stick with it across your maps.

3. **Document your namespace conventions**: If you introduce new namespaces, document their purpose and expected properties in your project documentation.

4. **Avoid deep nesting**: One level of namespace nesting is usually sufficient. Deeper nesting (like `properties.semantics.zone.labels`) should be reserved for truly hierarchical data.

5. **Backwards compatibility**: For core navigation properties like `xy_goal_tolerance`, either keep them at root level or consistently place them in a `navigation` namespace across all maps.

6. **Package name convention** (optional): ROS packages that define properties can use their package name as a namespace to avoid conflicts. E.g., a package called "topfleets_coordinator" could use this as the namespace.

### Type Flexibility

Properties support various data types:
- **Strings**: `"charging_station"`, `"concrete"`
- **Numbers**: `0.5`, `10`, `3.14`
- **Booleans**: `true`, `false`
- **Lists**: `["heavy_rain", "snow"]`
- **Nested Objects**: `{min: 0.1, max: 1.0}`

### Application-Specific Properties

Applications can define their own property schemas and document them appropriately. The topological navigation system will safely ignore properties it does not recognise, allowing different applications to coexist on the same map.

### Accessing Properties in Code

When accessing properties programmatically, always check for property existence before use:

```python
# Safe access to node properties
node_props = node["node"].get("properties", {})
xy_tolerance = node_props.get("xy_goal_tolerance", 0.3)  # Default to 0.3
semantics = node_props.get("semantics")  # Returns None if not present

# Safe access to edge properties
edge_props = edge.get("properties", {})
max_speed = edge_props.get("max_speed")  # Returns None if not present
if max_speed is not None:
    # Use max_speed for navigation control
    pass
```

## Schema Reference

The properties fields are defined in `config/tmap-schema.yaml`:

```yaml
# Node properties (at nodes[].node.properties)
properties:
  type: object
  additionalProperties: true
  description: Flexible dictionary of application-specific node properties

# Edge properties (at nodes[].node.edges[].properties)
properties:
  type: object
  additionalProperties: true
  description: Flexible dictionary of application-specific edge properties
```

## Validating Topological Maps

The `validate_map.py` script can be used to validate topological map YAML files against the schema:

```bash
# Basic validation
ros2 run topological_navigation validate_map.py my_map.tmap2.yaml

# With verbose output
ros2 run topological_navigation validate_map.py my_map.tmap2.yaml -v

# With custom schema file
ros2 run topological_navigation validate_map.py my_map.tmap2.yaml --schema custom_schema.yaml
```

The validator will:
- Check the map structure against the JSON schema
- Report any missing required fields
- Warn about duplicate node names
- Warn about edges pointing to non-existent nodes

## Related Resources

- [Topological Map Schema](../config/tmap-schema.yaml)
- [Node Template](../config/template_node_2.yaml)
- [Edge Template](../config/template_edge.yaml)
