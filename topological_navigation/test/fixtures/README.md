# Test Fixture YAML Files

This directory contains topological map YAML fixtures used by the localisation, graph, map analyser and map merger tests. The `*.yaml` files are also installed into the package's `config/` share directory (see `setup.py`).

## Files

### 1. simple_map.yaml
**Purpose:** Basic functionality testing with minimal complexity

**Characteristics:**
- **Nodes:** 2 (WP1, WP2)
- **Edges:** 1 (WP1 → WP2)
- **Polygon shapes:** Square influence zones
- **Features:**
  - Basic NavigateToPose action
  - Simple node properties (xy_goal_tolerance, yaw_goal_tolerance)
  - Edge properties (max_speed)

**Use cases:**
- Basic graph construction tests
- Simple KD-tree queries
- Basic point-in-polygon checks
- Edge distance calculations with minimal edges

### 2. complex_map.yaml
**Purpose:** Comprehensive testing with realistic map structure

**Characteristics:**
- **Nodes:** 10 (Entry, Junction1, Junction2, Row1Start, Row1End, Row2Start, Row2End, Exit, NoGoZone, TopicLocaliseNode)
- **Edges:** 10, forming a connected graph (Exit, NoGoZone and TopicLocaliseNode have no outgoing edges)
- **Polygon shapes:** Square influence zones (various sizes)
- **Features:**
  - Multiple edge actions (`navigate_to_pose`, `row_traversal`)
  - Agricultural navigation patterns (row traversal)
  - Node tags (entry_point, row_entry, row_exit, exit_point, no_go)
  - No-go node (NoGoZone with no_go: true property)
  - Topic-based localization (TopicLocaliseNode with localise_by_topic JSON config)
  - Roboflow integration properties
  - Various node properties and semantics

**Use cases:**
- Complex graph traversal
- Multiple node filtering (no-go, topic-based)
- Realistic localization scenarios
- Edge action testing
- Tag-based node queries
- Agricultural navigation testing

### 3. polygon_shapes_map.yaml
**Purpose:** Point-in-polygon algorithm testing with diverse polygon geometries

**Characteristics:**
- **Nodes:** 10 nodes with different polygon shapes
- **Edges:** None (focused on polygon testing)
- **Polygon shapes:**
  1. **TriangleNode** - Equilateral triangle (3 vertices)
  2. **PentagonNode** - Regular pentagon (5 vertices)
  3. **HexagonNode** - Regular hexagon (6 vertices)
  4. **LShapeNode** - Concave L-shaped polygon (6 vertices)
  5. **IrregularNode** - Asymmetric irregular polygon (5 vertices)
  6. **TinyNode** - Very small square (0.2m x 0.2m) for precision testing
  7. **LargeNode** - Very large square (20m x 20m) for scale testing
  8. **RectangleNode** - Non-square rectangle (6m x 2m)
  9. **EmptyVertsNode** - Empty verts list (edge case)
  10. **StarNode** - Complex concave star shape (10 vertices)

**Use cases:**
- Point-in-polygon algorithm validation
- Convex polygon testing (triangle, pentagon, hexagon, rectangle)
- Concave polygon testing (L-shape, star)
- Edge case testing (empty verts, tiny polygon, large polygon)
- Irregular polygon testing
- Ray-casting algorithm correctness verification

### 4. mixed_actions_map.yaml
**Purpose:** Route planning and action segment merging with several action types

**Characteristics:**
- **Nodes:** 6 (N1 to N6), square influence zones
- **Edges:** 10, using `navigate_to_pose`, `row_traversal` and `goal_align` actions

**Use cases:** `navigation_graph` tests (route planning, segment merging, boundary polygons)

### 5. mergeable_map_a.yaml / mergeable_map_b.yaml
**Purpose:** Map merging tests

**Characteristics:**
- **Nodes:** 2 each (WP1, WP2), no influence-zone vertices
- **Edges:** 1 each (WP1 to WP2, `navigate_to_pose`)

**Use cases:** `map_merger` / `map_analyser merge` tests (name deduplication, reprojection)

### 6. riseholme.tmap3.yaml
**Purpose:** Real-world map (92 nodes, 182 edges) of the Riseholme site, also the default map of `topological_navigation.launch.py`

**Use cases:** Large-map tests and the default demo map

## Testing Guidelines

### Unit Tests
When writing unit tests, use these fixtures as follows:

1. **simple_map.yaml** - For basic functionality tests where you need minimal complexity
2. **complex_map.yaml** - For integration tests and realistic scenarios
3. **polygon_shapes_map.yaml** - For point-in-polygon specific tests
4. **mixed_actions_map.yaml**, **mergeable_map_a/b.yaml**, **riseholme.tmap3.yaml** - For graph, merge and large-map tests (see above)

### Example Usage

```python
import yaml
from pathlib import Path

# Load fixture
fixture_path = Path(__file__).parent / 'fixtures' / 'simple_map.yaml'
with open(fixture_path, 'r') as f:
    map_data = yaml.safe_load(f)

# Use in tests
def test_graph_construction():
    graph = build_graph_from_tmap(map_data)
    assert graph.number_of_nodes() == 2
    assert graph.number_of_edges() == 1
```

### Point-in-Polygon Test Cases

The `polygon_shapes_map.yaml` fixture enables comprehensive testing:

```python
# Test point inside triangle
pose_inside = create_pose(0.0, 0.5)  # Inside TriangleNode
assert point_in_poly_nx(graph, 'TriangleNode', pose_inside) == True

# Test point outside triangle
pose_outside = create_pose(5.0, 5.0)  # Outside TriangleNode
assert point_in_poly_nx(graph, 'TriangleNode', pose_outside) == False

# Test concave polygon (L-shape)
pose_in_concave = create_pose(0.5, 11.5)  # Inside L-shape
assert point_in_poly_nx(graph, 'LShapeNode', pose_in_concave) == True

# Test empty verts edge case
pose_any = create_pose(20.0, -10.0)
assert point_in_poly_nx(graph, 'EmptyVertsNode', pose_any) == False
```

## Maintenance

When adding new fixtures:
1. Follow the existing YAML structure
2. Use descriptive node names
3. Document the purpose in this README
4. Ensure valid topological map format (compliant with `topological_navigation/config/tmap-schema.yaml`)
5. Add corresponding test cases

## Validation

All fixtures should validate against `topological_navigation/config/tmap-schema.yaml`.

To validate manually (requires `pyyaml` and `jsonschema`):
```bash
python3 topological_navigation/topological_navigation/validate_map.py \
    topological_navigation/test/fixtures/simple_map.yaml
```

## References

- Schema: `topological_navigation/config/tmap-schema.yaml`
- AGENTS.md: Project development guidelines
