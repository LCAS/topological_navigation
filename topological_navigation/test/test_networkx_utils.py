"""
Unit tests for networkx_utils module.

Tests cover:
- Graph construction from topological map data
- KD-tree spatial indexing
- Point-in-polygon checks
- Edge distance calculations
- Localization logic
"""

import pytest
import networkx as nx
import yaml
from pathlib import Path

# Import the module to test
from topological_navigation.networkx_utils import build_graph_from_tmap

# geometry_msgs is only available in a ROS 2 environment
try:
    import geometry_msgs  # noqa: F401
    _has_geometry_msgs = True
except ImportError:
    _has_geometry_msgs = False

_skip_no_geometry = pytest.mark.skipif(
    not _has_geometry_msgs, reason='geometry_msgs not available'
)


# ---------------------------------------------------------------------------
# Module-level fixtures (available to all test classes)
# ---------------------------------------------------------------------------

FIXTURE_DIR = Path(__file__).parent / 'fixtures'


@pytest.fixture
def simple_map_data():
    """Load simple map fixture."""
    with open(FIXTURE_DIR / 'simple_map.yaml', 'r') as f:
        return yaml.safe_load(f)


@pytest.fixture
def complex_map_data():
    """Load complex map fixture."""
    with open(FIXTURE_DIR / 'complex_map.yaml', 'r') as f:
        return yaml.safe_load(f)


class TestGraphConstruction:
    """Tests for build_graph_from_tmap function."""

    def test_simple_map_conversion(self, simple_map_data):
        """Test conversion of simple map (2 nodes)."""
        graph = build_graph_from_tmap(simple_map_data)
        
        assert graph is not None
        assert isinstance(graph, nx.DiGraph)
        assert graph.number_of_nodes() == 2
        assert graph.number_of_edges() == 1
        
        # Check node names
        assert 'WP1' in graph.nodes
        assert 'WP2' in graph.nodes

    def test_complex_map_conversion(self, complex_map_data):
        """Test conversion of complex map (10 nodes)."""
        graph = build_graph_from_tmap(complex_map_data)
        
        assert graph is not None
        assert isinstance(graph, nx.DiGraph)
        assert graph.number_of_nodes() == 10
        assert graph.number_of_edges() > 0
        
        # Check some node names
        assert 'Entry' in graph.nodes
        assert 'Exit' in graph.nodes
        assert 'Junction1' in graph.nodes

    def test_node_attribute_preservation(self, simple_map_data):
        """Test that node attributes are preserved correctly."""
        graph = build_graph_from_tmap(simple_map_data)
        
        # Check WP1 attributes
        wp1_attrs = graph.nodes['WP1']
        assert wp1_attrs['name'] == 'WP1'
        assert wp1_attrs['x'] == 0.0
        assert wp1_attrs['y'] == 0.0
        assert wp1_attrs['z'] == 0.0
        assert 'orientation' in wp1_attrs
        assert wp1_attrs['orientation']['w'] == 1.0
        assert 'verts' in wp1_attrs
        assert len(wp1_attrs['verts']) == 4
        assert wp1_attrs['parent_frame'] == 'map'
        
        # Check WP2 attributes
        wp2_attrs = graph.nodes['WP2']
        assert wp2_attrs['x'] == 5.0
        assert wp2_attrs['y'] == 0.0

    def test_edge_attribute_preservation(self, simple_map_data):
        """Test that edge attributes are preserved correctly."""
        graph = build_graph_from_tmap(simple_map_data)
        
        # Check edge from WP1 to WP2
        assert graph.has_edge('WP1', 'WP2')
        edge_attrs = graph.edges['WP1', 'WP2']
        
        assert edge_attrs['edge_id'] == 'WP1_WP2'
        assert edge_attrs['action'] == 'navigate_to_pose'
        assert 'properties' in edge_attrs
        assert 'weight' in edge_attrs

    def test_optional_properties_handling(self, complex_map_data):
        """Test handling of optional properties dictionaries."""
        graph = build_graph_from_tmap(complex_map_data)
        
        # Check node with properties
        entry_attrs = graph.nodes['Entry']
        assert 'properties' in entry_attrs
        assert entry_attrs['properties']['xy_goal_tolerance'] == 0.3
        assert entry_attrs['properties']['semantics'] == 'entry'
        
        # Check edge with properties
        edge_attrs = graph.edges['Row1Start', 'Row1End']
        assert 'properties' in edge_attrs
        assert edge_attrs['properties']['max_speed'] == 0.3
        assert edge_attrs['properties']['row_type'] == 'vineyard'

    def test_missing_field_handling(self):
        """Test handling of missing required fields."""
        # Map with missing 'nodes' field
        invalid_map1 = {'name': 'test_map'}
        assert build_graph_from_tmap(invalid_map1) is None
        
        # Map with invalid nodes structure
        invalid_map2 = {'nodes': 'not_a_list'}
        assert build_graph_from_tmap(invalid_map2) is None
        
        # Map with node missing 'name' -- only invalid node means 0 valid,
        # so build_graph_from_tmap returns None.
        invalid_map3 = {
            'nodes': [{
                'node': {
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    }
                }
            }]
        }
        graph = build_graph_from_tmap(invalid_map3)
        assert graph is None  # No valid nodes -> returns None

    def test_invalid_data_handling(self):
        """Test handling of completely invalid data."""
        # None input
        assert build_graph_from_tmap(None) is None
        
        # String input
        assert build_graph_from_tmap("invalid") is None
        
        # Empty dict
        assert build_graph_from_tmap({}) is None

    def test_empty_map(self):
        """Test handling of empty map (no nodes)."""
        empty_map = {'nodes': []}
        graph = build_graph_from_tmap(empty_map)
        
        # Empty nodes list -> 0 valid nodes -> returns None
        assert graph is None

    def test_node_with_no_edges(self, simple_map_data):
        """Test node with no outgoing edges."""
        graph = build_graph_from_tmap(simple_map_data)
        
        # WP2 has no edges
        assert graph.out_degree('WP2') == 0
        assert list(graph.successors('WP2')) == []

    def test_localise_by_topic_preservation(self, complex_map_data):
        """Test that localise_by_topic configuration is preserved."""
        graph = build_graph_from_tmap(complex_map_data)
        
        # Check node with localise_by_topic
        topic_node_attrs = graph.nodes['TopicLocaliseNode']
        assert 'localise_by_topic' in topic_node_attrs
        assert topic_node_attrs['localise_by_topic'] != ''
        assert '"topic"' in topic_node_attrs['localise_by_topic']
        
        # Check node without localise_by_topic
        entry_attrs = graph.nodes['Entry']
        assert entry_attrs['localise_by_topic'] == ''

    def test_meta_preservation(self, complex_map_data):
        """Test that meta information is preserved."""
        graph = build_graph_from_tmap(complex_map_data)
        
        # Check node with tags
        entry_attrs = graph.nodes['Entry']
        assert 'meta' in entry_attrs
        assert 'tag' in entry_attrs['meta']
        assert 'entry_point' in entry_attrs['meta']['tag']

    def test_edge_weight_default(self, simple_map_data):
        """Test that edge weight defaults to 1.0."""
        graph = build_graph_from_tmap(simple_map_data)
        
        edge_attrs = graph.edges['WP1', 'WP2']
        assert edge_attrs['weight'] == 1.0

    def test_graph_is_directed(self, simple_map_data):
        """Test that returned graph is a DiGraph."""
        graph = build_graph_from_tmap(simple_map_data)
        
        assert isinstance(graph, nx.DiGraph)
        assert graph.is_directed()

    def test_multiple_edges_from_node(self, complex_map_data):
        """Test node with multiple outgoing edges."""
        graph = build_graph_from_tmap(complex_map_data)
        
        # Entry node has 2 edges
        assert graph.out_degree('Entry') == 2
        successors = list(graph.successors('Entry'))
        assert 'Junction1' in successors
        assert 'Row1Start' in successors

    def test_self_loop_edge_handling(self):
        """Test handling of self-loop edges (should be added to graph)."""
        map_with_self_loop = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': [{
                        'edge_id': 'WP1_WP1',
                        'node': 'WP1',
                        'action': 'Wait'
                    }]
                }
            }]
        }
        
        graph = build_graph_from_tmap(map_with_self_loop)
        assert graph is not None
        assert graph.has_edge('WP1', 'WP1')  # Self-loop should be added


class TestErrorHandlingWithLogging:
    """Tests for error handling with logging."""

    class MockLogger:
        """Mock logger for testing."""
        def __init__(self):
            self.errors = []
            self.warnings = []
        
        def error(self, msg):
            self.errors.append(msg)
        
        def warning(self, msg):
            self.warnings.append(msg)

    def test_invalid_map_type_logs_error(self):
        """Test that invalid map type logs appropriate error."""
        logger = self.MockLogger()
        result = build_graph_from_tmap("not a dict", logger=logger)
        
        assert result is None
        assert len(logger.errors) == 1
        assert "Expected dictionary" in logger.errors[0]

    def test_missing_nodes_field_logs_error(self):
        """Test that missing 'nodes' field logs error."""
        logger = self.MockLogger()
        result = build_graph_from_tmap({'name': 'test'}, logger=logger)
        
        assert result is None
        assert len(logger.errors) == 1
        assert "Missing required 'nodes' field" in logger.errors[0]

    def test_invalid_nodes_type_logs_error(self):
        """Test that invalid 'nodes' type logs error."""
        logger = self.MockLogger()
        result = build_graph_from_tmap({'nodes': 'not a list'}, logger=logger)
        
        assert result is None
        assert len(logger.errors) == 1
        assert "'nodes' field must be a list" in logger.errors[0]

    def test_invalid_node_structure_logs_warning(self):
        """Test that invalid node structure logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [
                'not a dict',  # Invalid node
                {
                    'node': {
                        'name': 'WP1',
                        'pose': {
                            'position': {'x': 0, 'y': 0, 'z': 0},
                            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                        }
                    }
                }
            ]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_nodes() == 1  # Only valid node added
        assert len(logger.warnings) >= 1
        assert any("Expected dictionary" in w for w in logger.warnings)

    def test_missing_node_field_logs_warning(self):
        """Test that missing 'node' field logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [
                {'meta': {}},  # Missing 'node' field
                {
                    'node': {
                        'name': 'WP1',
                        'pose': {
                            'position': {'x': 0, 'y': 0, 'z': 0},
                            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                        }
                    }
                }
            ]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_nodes() == 1
        assert len(logger.warnings) >= 1
        assert any("Missing 'node' field" in w for w in logger.warnings)

    def test_missing_name_field_logs_warning(self):
        """Test that missing 'name' field logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    }
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        # Single invalid node -> 0 valid nodes -> returns None
        assert result is None
        assert len(logger.warnings) >= 1
        assert any("Missing required 'name' field" in w for w in logger.warnings)

    def test_missing_pose_field_logs_warning(self):
        """Test that missing 'pose' field logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1'
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is None  # No valid nodes -> returns None
        assert len(logger.warnings) >= 1
        assert any("Missing required 'pose' field" in w for w in logger.warnings)

    def test_missing_position_logs_warning(self):
        """Test that missing 'position' in pose logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    }
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is None  # No valid nodes -> returns None
        assert len(logger.warnings) >= 1
        assert any("Missing 'position' in pose" in w for w in logger.warnings)

    def test_missing_orientation_logs_warning(self):
        """Test that missing 'orientation' in pose logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0}
                    }
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is None  # No valid nodes -> returns None
        assert len(logger.warnings) >= 1
        assert any("Missing 'orientation' in pose" in w for w in logger.warnings)

    def test_missing_position_coordinates_logs_warning(self):
        """Test that missing position coordinates log warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0},  # Missing 'z'
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    }
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is None  # No valid nodes -> returns None
        assert len(logger.warnings) >= 1
        assert any("Missing position coordinates" in w for w in logger.warnings)

    def test_missing_orientation_components_logs_warning(self):
        """Test that missing orientation components log warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0}  # Missing 'w'
                    }
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is None  # No valid nodes -> returns None
        assert len(logger.warnings) >= 1
        assert any("Missing orientation components" in w for w in logger.warnings)

    def test_invalid_coordinate_type_logs_warning(self):
        """Test that invalid coordinate types log warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 'not a number', 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    }
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is None  # No valid nodes -> returns None
        assert len(logger.warnings) >= 1
        assert any("Error converting coordinates to float" in w for w in logger.warnings)

    def test_invalid_edges_type_logs_warning(self):
        """Test that invalid 'edges' type logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': 'not a list'
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_nodes() == 1
        assert len(logger.warnings) >= 1
        assert any("'edges' field must be a list" in w for w in logger.warnings)

    def test_invalid_edge_structure_logs_warning(self):
        """Test that invalid edge structure logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': ['not a dict']
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_nodes() == 1
        assert result.number_of_edges() == 0
        assert len(logger.warnings) >= 1
        assert any("Expected dictionary" in w for w in logger.warnings)

    def test_missing_edge_id_logs_warning(self):
        """Test that missing edge_id logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': [{
                        'node': 'WP2',
                        'action': 'NavigateToPose'
                    }]
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_edges() == 0
        assert len(logger.warnings) >= 1
        assert any("Missing 'edge_id'" in w for w in logger.warnings)

    def test_missing_edge_target_node_logs_warning(self):
        """Test that missing edge target node logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': [{
                        'edge_id': 'edge1',
                        'action': 'NavigateToPose'
                    }]
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_edges() == 0
        assert len(logger.warnings) >= 1
        assert any("Missing target 'node'" in w for w in logger.warnings)

    def test_missing_edge_action_logs_warning(self):
        """Test that missing edge action logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': [{
                        'edge_id': 'edge1',
                        'node': 'WP2'
                    }]
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_edges() == 0
        assert len(logger.warnings) >= 1
        assert any("Missing 'action'" in w for w in logger.warnings)

    def test_invalid_edge_properties_type_logs_warning(self):
        """Test that invalid edge properties type logs warning."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': [{
                        'edge_id': 'edge1',
                        'node': 'WP2',
                        'action': 'NavigateToPose',
                        'properties': 'not a dict'
                    }]
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_edges() == 1
        assert len(logger.warnings) >= 1
        assert any("'properties' must be a dictionary" in w for w in logger.warnings)

    def test_invalid_edge_weight_logs_warning(self):
        """Test that invalid edge weight logs warning and uses default."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    },
                    'edges': [{
                        'edge_id': 'edge1',
                        'node': 'WP2',
                        'action': 'NavigateToPose',
                        'properties': {'weight': 'not a number'}
                    }]
                }
            }]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is not None
        assert result.number_of_edges() == 1
        edge_attrs = result.edges['WP1', 'WP2']
        assert edge_attrs['weight'] == 1.0  # Default weight
        assert len(logger.warnings) >= 1
        assert any("Error converting weight to float" in w for w in logger.warnings)

    def test_no_valid_nodes_logs_error(self):
        """Test that map with no valid nodes logs error."""
        logger = self.MockLogger()
        invalid_map = {
            'nodes': [
                {'invalid': 'data'},
                {'node': {'invalid': 'structure'}}
            ]
        }
        result = build_graph_from_tmap(invalid_map, logger=logger)
        
        assert result is None
        assert len(logger.errors) >= 1
        assert any("No valid nodes found" in e for e in logger.errors)

    def test_unexpected_exception_logs_error(self):
        """Test that unexpected exceptions are caught and logged."""
        logger = self.MockLogger()
        
        # Create a map that will cause an unexpected error during processing
        # This is a bit tricky to trigger, but we can test the exception handler exists
        invalid_map = {
            'nodes': [{
                'node': {
                    'name': 'WP1',
                    'pose': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                    }
                }
            }]
        }
        
        # This should succeed normally
        result = build_graph_from_tmap(invalid_map, logger=logger)
        assert result is not None

    def test_partial_map_processing(self):
        """Test that valid nodes are processed even when some nodes are invalid."""
        logger = self.MockLogger()
        mixed_map = {
            'nodes': [
                {
                    'node': {
                        'name': 'ValidNode1',
                        'pose': {
                            'position': {'x': 0, 'y': 0, 'z': 0},
                            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                        }
                    }
                },
                {
                    'node': {
                        'name': 'InvalidNode',
                        'pose': {
                            'position': {'x': 'invalid', 'y': 0, 'z': 0},
                            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                        }
                    }
                },
                {
                    'node': {
                        'name': 'ValidNode2',
                        'pose': {
                            'position': {'x': 5, 'y': 5, 'z': 0},
                            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
                        }
                    }
                }
            ]
        }
        
        result = build_graph_from_tmap(mixed_map, logger=logger)
        
        assert result is not None
        assert result.number_of_nodes() == 2  # Only valid nodes
        assert 'ValidNode1' in result.nodes
        assert 'ValidNode2' in result.nodes
        assert 'InvalidNode' not in result.nodes
        assert len(logger.warnings) >= 1  # Warning for invalid node


if __name__ == '__main__':
    pytest.main([__file__, '-v'])


class TestKDTreeConstruction:
    """Tests for build_kdtree_from_graph function."""

    @pytest.fixture
    def simple_graph(self, simple_map_data):
        """Build graph from simple map (uses module-level fixture)."""
        return build_graph_from_tmap(simple_map_data)

    @pytest.fixture
    def complex_graph(self, complex_map_data):
        """Build graph from complex map (uses module-level fixture)."""
        return build_graph_from_tmap(complex_map_data)

    def test_kdtree_construction_simple(self, simple_graph):
        """Test KD-tree construction from simple graph."""
        from topological_navigation.networkx_utils import build_kdtree_from_graph
        
        kdtree, node_names = build_kdtree_from_graph(simple_graph)
        
        assert kdtree is not None
        assert len(node_names) == 2
        assert 'WP1' in node_names
        assert 'WP2' in node_names

    def test_kdtree_construction_complex(self, complex_graph):
        """Test KD-tree construction from complex graph."""
        from topological_navigation.networkx_utils import build_kdtree_from_graph
        
        kdtree, node_names = build_kdtree_from_graph(complex_graph)
        
        assert kdtree is not None
        assert len(node_names) == 10
        assert 'Entry' in node_names
        assert 'Exit' in node_names

    def test_kdtree_none_graph(self):
        """Test KD-tree construction with None graph."""
        from topological_navigation.networkx_utils import build_kdtree_from_graph
        
        kdtree, node_names = build_kdtree_from_graph(None)
        
        assert kdtree is None
        assert node_names == []

    def test_kdtree_empty_graph(self):
        """Test KD-tree construction with empty graph."""
        from topological_navigation.networkx_utils import build_kdtree_from_graph
        
        empty_graph = nx.DiGraph()
        kdtree, node_names = build_kdtree_from_graph(empty_graph)
        
        assert kdtree is None
        assert node_names == []


@_skip_no_geometry
class TestNearestNodeQuery:
    """Tests for query_nearest_nodes function."""

    @pytest.fixture
    def simple_kdtree_setup(self, simple_map_data):
        """Build graph and KD-tree from simple map."""
        from topological_navigation.networkx_utils import build_kdtree_from_graph
        graph = build_graph_from_tmap(simple_map_data)
        kdtree, node_names = build_kdtree_from_graph(graph)
        return kdtree, node_names

    def test_query_single_nearest(self, simple_kdtree_setup):
        """Test querying single nearest node."""
        from topological_navigation.networkx_utils import query_nearest_nodes
        from geometry_msgs.msg import Pose
        
        kdtree, node_names = simple_kdtree_setup
        
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 0.0
        
        results = query_nearest_nodes(kdtree, node_names, pose, k=1)
        
        assert len(results) == 1
        assert results[0]['node'] == 'WP1'
        assert results[0]['dist'] > 0

    def test_query_multiple_nearest(self, simple_kdtree_setup):
        """Test querying multiple nearest nodes."""
        from topological_navigation.networkx_utils import query_nearest_nodes
        from geometry_msgs.msg import Pose
        
        kdtree, node_names = simple_kdtree_setup
        
        pose = Pose()
        pose.position.x = 2.5
        pose.position.y = 0.0
        
        results = query_nearest_nodes(kdtree, node_names, pose, k=2)
        
        assert len(results) == 2
        assert all('node' in r and 'dist' in r for r in results)

    def test_query_none_kdtree(self):
        """Test query with None KD-tree."""
        from topological_navigation.networkx_utils import query_nearest_nodes
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        results = query_nearest_nodes(None, [], pose, k=1)
        
        assert results == []


@_skip_no_geometry
class TestPointInPolygon:
    """Tests for point_in_poly_nx function."""

    @pytest.fixture
    def polygon_graph(self):
        """Create graph with polygon influence zones."""
        fixture_path = Path(__file__).parent / 'fixtures' / 'polygon_shapes_map.yaml'
        with open(fixture_path, 'r') as f:
            map_data = yaml.safe_load(f)
        return build_graph_from_tmap(map_data)

    def test_point_inside_rectangle(self, polygon_graph):
        """Test point inside RectangleNode influence zone.

        RectangleNode is at (10, -10) with verts [(-3,-1),(3,-1),(3,1),(-3,1)].
        So absolute point (10.5, -9.5) is relative (0.5, 0.5) -> inside.
        """
        from topological_navigation.networkx_utils import point_in_poly_nx
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        pose.position.x = 10.5
        pose.position.y = -9.5
        
        result = point_in_poly_nx(polygon_graph, 'RectangleNode', pose)
        assert result is True

    def test_point_outside_rectangle(self, polygon_graph):
        """Test point outside RectangleNode influence zone."""
        from topological_navigation.networkx_utils import point_in_poly_nx
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        pose.position.x = 50.0
        pose.position.y = 50.0
        
        result = point_in_poly_nx(polygon_graph, 'RectangleNode', pose)
        assert result is False

    def test_point_in_poly_none_graph(self):
        """Test point-in-polygon with None graph."""
        from topological_navigation.networkx_utils import point_in_poly_nx
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        result = point_in_poly_nx(None, 'node', pose)
        
        assert result is False


@_skip_no_geometry
class TestEdgeDistances:
    """Tests for get_edge_distances_nx function."""

    @pytest.fixture
    def simple_graph(self, simple_map_data):
        """Build graph from simple map (uses module-level fixture)."""
        return build_graph_from_tmap(simple_map_data)

    def test_edge_distances_calculation(self, simple_graph):
        """Test edge distance calculation."""
        from topological_navigation.networkx_utils import get_edge_distances_nx
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        pose.position.x = 2.5
        pose.position.y = 1.0
        
        edge_ids, distances = get_edge_distances_nx(simple_graph, pose)
        
        assert len(edge_ids) > 0
        assert len(distances) > 0
        assert len(edge_ids) == len(distances)

    def test_edge_distances_none_graph(self):
        """Test edge distances with None graph."""
        from topological_navigation.networkx_utils import get_edge_distances_nx
        from geometry_msgs.msg import Pose
        import numpy as np
        
        pose = Pose()
        edge_ids, distances = get_edge_distances_nx(None, pose)
        
        assert edge_ids == []
        assert len(distances) == 0


@_skip_no_geometry
class TestCurrentNodeDetermination:
    """Tests for determine_current_node function."""

    @pytest.fixture
    def localization_setup(self, simple_map_data):
        """Build graph and KD-tree for localization."""
        from topological_navigation.networkx_utils import build_kdtree_from_graph
        graph = build_graph_from_tmap(simple_map_data)
        kdtree, node_names = build_kdtree_from_graph(graph)
        return graph, kdtree, node_names

    def test_determine_current_inside_zone(self, localization_setup):
        """Test determining current node when inside influence zone."""
        from topological_navigation.networkx_utils import determine_current_node
        from geometry_msgs.msg import Pose
        
        graph, kdtree, node_names = localization_setup
        
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.5
        
        current = determine_current_node(graph, kdtree, node_names, pose, [], [])
        
        assert current == 'WP1'

    def test_determine_current_outside_zones(self, localization_setup):
        """Test determining current node when outside all zones."""
        from topological_navigation.networkx_utils import determine_current_node
        from geometry_msgs.msg import Pose
        
        graph, kdtree, node_names = localization_setup
        
        pose = Pose()
        pose.position.x = 100.0
        pose.position.y = 100.0
        
        current = determine_current_node(graph, kdtree, node_names, pose, [], [])
        
        assert current == 'none'

    def test_determine_current_none_inputs(self):
        """Test determine_current_node with None inputs."""
        from topological_navigation.networkx_utils import determine_current_node
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        current = determine_current_node(None, None, [], pose, [], [])
        
        assert current == 'none'


@_skip_no_geometry
class TestClosestNodeDetermination:
    """Tests for determine_closest_node function."""

    @pytest.fixture
    def localization_setup(self, simple_map_data):
        """Build graph and KD-tree for localization."""
        from topological_navigation.networkx_utils import build_kdtree_from_graph
        graph = build_graph_from_tmap(simple_map_data)
        kdtree, node_names = build_kdtree_from_graph(graph)
        return graph, kdtree, node_names

    def test_determine_closest_basic(self, localization_setup):
        """Test determining closest node."""
        from topological_navigation.networkx_utils import determine_closest_node
        from geometry_msgs.msg import Pose
        
        graph, kdtree, node_names = localization_setup
        
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 0.0
        
        closest, dist = determine_closest_node(kdtree, node_names, graph, 'none', [], [], pose)
        
        assert closest == 'WP1'
        assert dist > 0

    def test_determine_closest_with_current(self, localization_setup):
        """Test closest node when current node is set."""
        from topological_navigation.networkx_utils import determine_closest_node
        from geometry_msgs.msg import Pose
        
        graph, kdtree, node_names = localization_setup
        
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.5
        
        closest, dist = determine_closest_node(kdtree, node_names, graph, 'WP1', [], [], pose)
        
        assert closest == 'WP1'

    def test_determine_closest_none_kdtree(self):
        """Test determine_closest_node with None KD-tree."""
        from topological_navigation.networkx_utils import determine_closest_node
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        closest, dist = determine_closest_node(None, [], None, 'none', [], [], pose)
        
        assert closest == 'none'
        assert dist == float('inf')


class TestTopicBasedLocalization:
    """Tests for update_loc_by_topic_nx function."""

    @pytest.fixture
    def complex_graph(self, complex_map_data):
        """Build graph from complex map."""
        return build_graph_from_tmap(complex_map_data)

    def test_update_loc_by_topic(self, complex_graph):
        """Test extracting topic-based localization config."""
        from topological_navigation.networkx_utils import update_loc_by_topic_nx
        
        nodes_by_topic, names_by_topic = update_loc_by_topic_nx(complex_graph)
        
        assert len(nodes_by_topic) > 0
        assert len(names_by_topic) > 0
        assert 'TopicLocaliseNode' in names_by_topic

    def test_update_loc_by_topic_none_graph(self):
        """Test update_loc_by_topic_nx with None graph."""
        from topological_navigation.networkx_utils import update_loc_by_topic_nx
        
        nodes, names = update_loc_by_topic_nx(None)
        
        assert nodes == []
        assert names == []


# ===========================================================================
# Graph algorithm wrappers
# ===========================================================================


class TestComputeShortestPath:
    """Tests for compute_shortest_path."""

    @pytest.fixture
    def weighted_graph(self):
        """A -> B (1) -> C (2), A -> C (5)."""
        from topological_navigation.networkx_utils import compute_shortest_path  # noqa: F401
        import networkx as nx
        G = nx.DiGraph()
        G.add_edge('A', 'B', weight=1.0)
        G.add_edge('B', 'C', weight=2.0)
        G.add_edge('A', 'C', weight=5.0)
        return G

    def test_shortest_via_intermediate(self, weighted_graph):
        from topological_navigation.networkx_utils import compute_shortest_path
        path = compute_shortest_path(weighted_graph, 'A', 'C')
        assert path == ['A', 'B', 'C']

    def test_same_node(self, weighted_graph):
        from topological_navigation.networkx_utils import compute_shortest_path
        assert compute_shortest_path(weighted_graph, 'A', 'A') == ['A']

    def test_no_path(self, weighted_graph):
        """C has no outgoing edges, so C->A should return []."""
        from topological_navigation.networkx_utils import compute_shortest_path
        assert compute_shortest_path(weighted_graph, 'C', 'A') == []

    def test_none_graph(self):
        from topological_navigation.networkx_utils import compute_shortest_path
        assert compute_shortest_path(None, 'A', 'B') == []

    def test_missing_source(self, weighted_graph):
        from topological_navigation.networkx_utils import compute_shortest_path
        assert compute_shortest_path(weighted_graph, 'X', 'A') == []

    def test_missing_target(self, weighted_graph):
        from topological_navigation.networkx_utils import compute_shortest_path
        assert compute_shortest_path(weighted_graph, 'A', 'X') == []


class TestComputePathLength:
    """Tests for compute_path_length."""

    @pytest.fixture
    def weighted_graph(self):
        import networkx as nx
        G = nx.DiGraph()
        G.add_edge('A', 'B', weight=1.5)
        G.add_edge('B', 'C', weight=2.5)
        G.add_edge('A', 'C', weight=5.0)
        return G

    def test_shortest_length(self, weighted_graph):
        from topological_navigation.networkx_utils import compute_path_length
        length = compute_path_length(weighted_graph, 'A', 'C')
        assert length == pytest.approx(4.0)

    def test_same_node_zero(self, weighted_graph):
        from topological_navigation.networkx_utils import compute_path_length
        assert compute_path_length(weighted_graph, 'A', 'A') == 0.0

    def test_no_path_inf(self, weighted_graph):
        from topological_navigation.networkx_utils import compute_path_length
        assert compute_path_length(weighted_graph, 'C', 'A') == float('inf')

    def test_none_graph_inf(self):
        from topological_navigation.networkx_utils import compute_path_length
        assert compute_path_length(None, 'A', 'B') == float('inf')


class TestCheckConnectivity:
    """Tests for check_connectivity."""

    @pytest.fixture
    def directed_graph(self):
        import networkx as nx
        G = nx.DiGraph()
        G.add_edge('A', 'B')
        G.add_edge('B', 'C')
        G.add_node('D')  # isolated
        return G

    def test_connected(self, directed_graph):
        from topological_navigation.networkx_utils import check_connectivity
        assert check_connectivity(directed_graph, 'A', 'C') is True

    def test_not_connected_reverse(self, directed_graph):
        from topological_navigation.networkx_utils import check_connectivity
        assert check_connectivity(directed_graph, 'C', 'A') is False

    def test_isolated_node(self, directed_graph):
        from topological_navigation.networkx_utils import check_connectivity
        assert check_connectivity(directed_graph, 'A', 'D') is False

    def test_same_node(self, directed_graph):
        from topological_navigation.networkx_utils import check_connectivity
        assert check_connectivity(directed_graph, 'A', 'A') is True

    def test_none_graph(self):
        from topological_navigation.networkx_utils import check_connectivity
        assert check_connectivity(None, 'A', 'B') is False

    def test_missing_node(self, directed_graph):
        from topological_navigation.networkx_utils import check_connectivity
        assert check_connectivity(directed_graph, 'X', 'A') is False


class TestGetNeighbors:
    """Tests for get_neighbors."""

    @pytest.fixture
    def g(self):
        import networkx as nx
        G = nx.DiGraph()
        G.add_edge('A', 'B')
        G.add_edge('A', 'C')
        G.add_edge('B', 'C')
        return G

    def test_neighbors_of_hub(self, g):
        from topological_navigation.networkx_utils import get_neighbors
        assert sorted(get_neighbors(g, 'A')) == ['B', 'C']

    def test_leaf_has_one_neighbor(self, g):
        from topological_navigation.networkx_utils import get_neighbors
        assert get_neighbors(g, 'B') == ['C']

    def test_no_outgoing(self, g):
        from topological_navigation.networkx_utils import get_neighbors
        assert get_neighbors(g, 'C') == []

    def test_none_graph(self):
        from topological_navigation.networkx_utils import get_neighbors
        assert get_neighbors(None, 'A') == []

    def test_missing_node(self, g):
        from topological_navigation.networkx_utils import get_neighbors
        assert get_neighbors(g, 'X') == []


# ===========================================================================
# normalize_action_name (navigation_graph)
# ===========================================================================


class TestNormalizeActionName:
    """Tests for navigation_graph.normalize_action_name."""

    def test_camel_to_canonical(self):
        from topological_navigation.navigation_graph import normalize_action_name
        assert normalize_action_name('NavigateToPose') == 'NavigateToPose'

    def test_snake_to_canonical(self):
        from topological_navigation.navigation_graph import normalize_action_name
        assert normalize_action_name('navigate_to_pose') == 'NavigateToPose'

    def test_row_traversal_cases(self):
        from topological_navigation.navigation_graph import normalize_action_name
        assert normalize_action_name('RowTraversal') == 'row_traversal'
        assert normalize_action_name('row_traversal') == 'row_traversal'

    def test_goal_align_cases(self):
        from topological_navigation.navigation_graph import normalize_action_name
        assert normalize_action_name('GoalAlign') == 'goal_align'
        assert normalize_action_name('goal_align') == 'goal_align'

    def test_unknown_returned_unchanged(self):
        from topological_navigation.navigation_graph import normalize_action_name
        assert normalize_action_name('SomethingNew') == 'SomethingNew'
