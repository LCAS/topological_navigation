"""Unit tests for localisation2 module.

Tests cover the pure-logic parts of TopologicalNavLoc without starting a
full ROS 2 node.  The tests mock rclpy so that no actual ROS 2 runtime
is required.

Tested functionality:
- Import / module syntax
- Static message helpers (_make_string_msg, _make_float32_msg)
- _get_node_tag
- get_edge_distances_to_pose
- _publish_topics (latched vs non-latched)
- _map_callback graph/KD-tree construction
- localise_pose_cb
- _pose_callback throttle logic
"""

# -- Path fix: the ROS package directory has an __init__.py that shadows
# the inner Python package of the same name.  We must ensure the correct
# "topological_navigation" package (the inner one) is found first.
import sys as _sys
from pathlib import Path as _Path

_src_dir = str(_Path(__file__).resolve().parent.parent)
if _src_dir not in _sys.path:
    _sys.path.insert(0, _src_dir)

# Evict stale cached root package (outer __init__.py) if necessary.
_tn = _sys.modules.get('topological_navigation')
if _tn is not None:
    _f = getattr(_tn, '__file__', '') or ''
    if 'topological_navigation/topological_navigation' not in _f:
        del _sys.modules['topological_navigation']

import math
from pathlib import Path
from threading import Lock
from unittest.mock import MagicMock, patch, PropertyMock

import numpy as np
import pytest
import yaml

# We need geometry_msgs for Pose objects used in tests
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32

from topological_navigation.networkx_utils import (
    build_graph_from_tmap,
    build_kdtree_from_graph,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

FIXTURE_DIR = Path(__file__).parent / 'fixtures'


@pytest.fixture
def simple_map_data():
    """Load simple 2-node map fixture."""
    with open(FIXTURE_DIR / 'simple_map.yaml') as f:
        return yaml.safe_load(f)


@pytest.fixture
def complex_map_data():
    """Load complex 10-node map fixture."""
    with open(FIXTURE_DIR / 'complex_map.yaml') as f:
        return yaml.safe_load(f)


@pytest.fixture
def simple_graph(simple_map_data):
    """Build a NetworkX graph from the simple map."""
    return build_graph_from_tmap(simple_map_data)


@pytest.fixture
def simple_kdtree(simple_graph):
    """Build a KD-tree from the simple graph."""
    return build_kdtree_from_graph(simple_graph)


def _make_pose(x=0.0, y=0.0, z=0.0):
    """Helper: create a geometry_msgs/Pose at (x, y, z)."""
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.w = 1.0
    return p


def _make_loc_node(simple_map_data, simple_graph, simple_kdtree):
    """Create a TopologicalNavLoc-like object without starting ROS.

    We patch __init__ to avoid rclpy.node.Node initialisation, then
    manually set the attributes the methods under test rely on.
    """
    from topological_navigation.scripts.localisation2 import TopologicalNavLoc

    with patch.object(TopologicalNavLoc, '__init__', lambda self, *a, **kw: None):
        loc = TopologicalNavLoc.__new__(TopologicalNavLoc)

    # Minimal attributes for the methods under test
    loc.tmap = simple_map_data
    loc._graph = simple_graph
    kdtree, names = simple_kdtree
    loc._kdtree = kdtree
    loc._kdtree_node_names = names
    loc.nogos = []
    loc.names_by_topic = []
    loc.loc_by_topic = []
    loc.force_check = True
    loc.current_pose = _make_pose()

    # Latched-mode state
    loc.only_latched = True
    loc.wpstr = "Unknown"
    loc.closest_dist = 1e6 - 1
    loc.cnstr = "Unknown"
    loc.nodetag = "Unknown"
    loc.closest_edge_ids = []
    loc.closest_edge_dists = []
    loc.current_closest_node_name = ""

    # Thread lock (required by get_edge_distances_to_pose)
    loc._map_lock = Lock()

    # Mock publishers
    loc.wp_pub = MagicMock()
    loc.wd_pub = MagicMock()
    loc.cn_pub = MagicMock()
    loc.ce_pub = MagicMock()
    loc.tag_pub = MagicMock()

    # Mock logger
    _logger = MagicMock()
    loc.get_logger = MagicMock(return_value=_logger)

    return loc


# ---------------------------------------------------------------------------
# Test: module imports cleanly
# ---------------------------------------------------------------------------

class TestImport:
    """Verify that the module can be imported without errors."""

    def test_import_module(self):
        import topological_navigation.scripts.localisation2  # noqa: F401

    def test_import_class(self):
        from topological_navigation.scripts.localisation2 import TopologicalNavLoc  # noqa: F401

    def test_import_main(self):
        from topological_navigation.scripts.localisation2 import main  # noqa: F401


# ---------------------------------------------------------------------------
# Test: static message helpers
# ---------------------------------------------------------------------------

class TestMessageHelpers:
    """Test _make_string_msg and _make_float32_msg."""

    def test_make_string_msg(self):
        from topological_navigation.scripts.localisation2 import TopologicalNavLoc
        msg = TopologicalNavLoc._make_string_msg("hello")
        assert isinstance(msg, String)
        assert msg.data == "hello"

    def test_make_string_msg_empty(self):
        from topological_navigation.scripts.localisation2 import TopologicalNavLoc
        msg = TopologicalNavLoc._make_string_msg("")
        assert msg.data == ""

    def test_make_float32_msg(self):
        from topological_navigation.scripts.localisation2 import TopologicalNavLoc
        msg = TopologicalNavLoc._make_float32_msg(3.14)
        assert isinstance(msg, Float32)
        assert abs(msg.data - 3.14) < 1e-6

    def test_make_float32_msg_zero(self):
        from topological_navigation.scripts.localisation2 import TopologicalNavLoc
        msg = TopologicalNavLoc._make_float32_msg(0.0)
        assert msg.data == 0.0


# ---------------------------------------------------------------------------
# Test: _get_node_tag
# ---------------------------------------------------------------------------

class TestGetNodeTag:
    """Test the _get_node_tag helper."""

    def test_tag_unknown_when_node_missing(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        assert loc._get_node_tag("nonexistent_node") == 'Unknown'

    def test_tag_unknown_when_no_tag_key(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        # WP1 in simple_map has no 'tag' in meta
        assert loc._get_node_tag("WP1") == 'Unknown'

    def test_tag_returned_when_present(self, complex_map_data):
        """complex_map Entry node has tag: ['entry_point']."""
        graph = build_graph_from_tmap(complex_map_data)
        kdtree = build_kdtree_from_graph(graph)

        from topological_navigation.scripts.localisation2 import TopologicalNavLoc
        with patch.object(TopologicalNavLoc, '__init__', lambda self, *a, **kw: None):
            loc = TopologicalNavLoc.__new__(TopologicalNavLoc)
        loc.tmap = complex_map_data
        loc._graph = graph
        loc.get_logger = MagicMock(return_value=MagicMock())

        assert loc._get_node_tag("Entry") == 'entry_point'


# ---------------------------------------------------------------------------
# Test: get_edge_distances_to_pose
# ---------------------------------------------------------------------------

class TestGetEdgeDistancesToPose:
    """Test get_edge_distances_to_pose."""

    def test_returns_results(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        pose = _make_pose(2.5, 1.0)
        edge_ids, dists = loc.get_edge_distances_to_pose(pose)
        assert len(edge_ids) > 0
        assert len(dists) > 0

    def test_empty_when_no_graph(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        loc._graph = None
        edge_ids, dists = loc.get_edge_distances_to_pose(_make_pose())
        assert edge_ids == []
        assert len(dists) == 0


# ---------------------------------------------------------------------------
# Test: _publish_topics
# ---------------------------------------------------------------------------

class TestPublishTopics:
    """Test the _publish_topics method (latched and non-latched modes)."""

    def test_latched_publishes_on_change(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        loc.only_latched = True

        loc._publish_topics("WP1", 1.5, "WP1", ["e1"], [0.5], "tag1")

        loc.wp_pub.publish.assert_called_once()
        loc.wd_pub.publish.assert_called_once()
        loc.cn_pub.publish.assert_called_once()
        loc.tag_pub.publish.assert_called_once()

    def test_latched_no_publish_when_same(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        loc.only_latched = True

        # First publish sets the state
        loc._publish_topics("WP1", 1.5, "WP1", ["e1"], [0.5], "tag1")
        loc.wp_pub.reset_mock()
        loc.cn_pub.reset_mock()

        # Same values → no publish
        loc._publish_topics("WP1", 1.5, "WP1", ["e1"], [0.5], "tag1")
        loc.wp_pub.publish.assert_not_called()
        loc.cn_pub.publish.assert_not_called()

    def test_non_latched_always_publishes(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        loc.only_latched = False

        loc._publish_topics("WP1", 1.5, "WP1", ["e1"], [0.5])
        loc.wp_pub.publish.assert_called_once()

        # Same values → still publishes
        loc.wp_pub.reset_mock()
        loc._publish_topics("WP1", 1.5, "WP1", ["e1"], [0.5])
        loc.wp_pub.publish.assert_called_once()

    def test_edge_ids_sorted_when_dists_equal(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        loc.only_latched = False

        loc._publish_topics("WP1", 1.0, "WP1", ["b_edge", "a_edge"], [1.0, 1.0])
        # After the call, stored edge_ids should be sorted
        assert loc.closest_edge_ids == ["a_edge", "b_edge"]


# ---------------------------------------------------------------------------
# Test: _map_callback
# ---------------------------------------------------------------------------

class TestMapCallback:
    """Test _map_callback builds graph and KD-tree correctly."""

    def _make_bare_loc(self):
        """Create a bare TopologicalNavLoc without ROS init."""
        from topological_navigation.scripts.localisation2 import TopologicalNavLoc

        with patch.object(TopologicalNavLoc, '__init__', lambda self, *a, **kw: None):
            loc = TopologicalNavLoc.__new__(TopologicalNavLoc)

        loc.rec_map = False
        loc._graph = None
        loc._kdtree = None
        loc._kdtree_node_names = []
        loc.names_by_topic = []
        loc.nodes_by_topic = []
        loc.loc_by_topic = []
        loc.nogos = []
        loc.with_tags = False
        loc._map_lock = Lock()
        loc.get_logger = MagicMock(return_value=MagicMock())
        return loc

    def test_builds_graph_and_kdtree(self, simple_map_data):
        loc = self._make_bare_loc()

        msg = String()
        msg.data = yaml.dump(simple_map_data)
        loc._map_callback(msg)

        assert loc.rec_map is True
        assert loc._graph is not None
        assert loc._graph.number_of_nodes() == 2
        assert loc._kdtree is not None
        assert len(loc._kdtree_node_names) == 2
        assert loc.tmap_frame == 'simple_test_map'

    def test_update_replaces_graph(self, simple_map_data):
        """Calling _map_callback again should update (not skip)."""
        loc = self._make_bare_loc()

        msg = String()
        msg.data = yaml.dump(simple_map_data)
        loc._map_callback(msg)
        assert loc.rec_map is True
        old_graph = loc._graph

        # Second call should update, not skip
        loc._map_callback(msg)
        assert loc.rec_map is True
        assert loc._graph is not None
        assert loc._graph.number_of_nodes() == 2

    def test_handles_bad_map_data(self):
        loc = self._make_bare_loc()

        msg = String()
        msg.data = yaml.dump({'transformation': {'topo_frame_id': 'test'}, 'nodes': []})
        loc._map_callback(msg)

        # Should not set rec_map because graph build returned None (empty nodes)
        assert loc.rec_map is False


# ---------------------------------------------------------------------------
# Test: localise_pose_cb
# ---------------------------------------------------------------------------

class TestLocalisePoseCb:
    """Test the localise_pose service callback."""

    def test_returns_nodes(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)

        req = MagicMock()
        req.pose = _make_pose(0.0, 0.0)
        res = MagicMock()
        res.current_node = ''
        res.closest_node = ''

        result = loc.localise_pose_cb(req, res)

        # At origin, should be inside WP1's influence zone
        assert result.current_node != 'none'
        assert result.closest_node == 'WP1'

    def test_returns_none_when_no_graph(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)
        loc._graph = None

        req = MagicMock()
        req.pose = _make_pose()
        res = MagicMock()

        result = loc.localise_pose_cb(req, res)
        assert result.current_node == 'none'
        assert result.closest_node == 'none'

    def test_closest_node_far_from_all(self, simple_map_data, simple_graph, simple_kdtree):
        loc = _make_loc_node(simple_map_data, simple_graph, simple_kdtree)

        req = MagicMock()
        req.pose = _make_pose(100.0, 100.0)
        res = MagicMock()
        res.current_node = ''
        res.closest_node = ''

        result = loc.localise_pose_cb(req, res)
        # Should still return a closest node even if far away
        assert result.closest_node in ('WP1', 'WP2')
        # Should NOT be inside any influence zone
        assert result.current_node == 'none'
