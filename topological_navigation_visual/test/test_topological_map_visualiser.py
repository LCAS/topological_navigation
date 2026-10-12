"""Tests for fast marker updates in ``topological_map_visualiser.py``."""

from types import SimpleNamespace

from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarkerFeedback

from topological_navigation_visual.scripts.topological_map_visualiser import (
    TopologicalMapVisualiser,
)


class _DummyLogger:
    """Small logger stub for unit tests."""

    def debug(self, _msg):
        pass

    def info(self, _msg):
        pass

    def warn(self, _msg):
        pass

    def warning(self, _msg):
        pass

    def error(self, _msg):
        pass


class _Publisher:
    """Publisher stub that records the last message."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _InteractiveMarkerServer:
    """Minimal server stub for exercising feedback handlers."""

    def applyChanges(self):
        pass


class _NoIterList(list):
    """List-like object that raises if someone iterates it."""

    def __iter__(self):
        raise AssertionError('unexpected full node-list scan')


def _make_node(name, x, y, edges=None):
    """Build a minimal tmap node entry."""
    return {
        'meta': {'map': 'test_map', 'node': name, 'pointset': 'test_map'},
        'node': {
            'name': name,
            'parent_frame': 'map',
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0,
                },
            },
            'edges': edges or [],
        },
    }


def _make_visualiser(tmap):
    """Create a partially constructed visualiser for unit tests."""
    vis = TopologicalMapVisualiser.__new__(TopologicalMapVisualiser)
    vis.get_logger = lambda: _DummyLogger()
    vis.tmap = tmap
    vis.marker_scale = 0.5
    vis.map_marker_pub = _Publisher()
    vis.route_marker_pub = _Publisher()
    vis._im_server = _InteractiveMarkerServer()
    vis._route_nodes = []
    vis._node_entries_by_name = {}
    vis._incoming_edges_by_target = {}
    vis._action_names = []
    vis._action_index = {}
    vis._marker_ids = {
        'nodes': {},
        'names': {},
        'zones': {},
        'edges': {},
        'legend': {},
    }
    return vis


def test_refresh_visual_cache_indexes_incoming_edges_and_actions():
    """The visual cache should index nodes, incoming edges, and action order."""
    tmap = {
        'nodes': [
            _make_node('A', 0.0, 0.0, [
                {'edge_id': 'A_B', 'node': 'B', 'action': 'drive'},
            ]),
            _make_node('B', 1.0, 0.0),
            _make_node('C', -1.0, 0.0, [
                {'edge_id': 'C_B', 'node': 'B', 'action': 'inspect'},
            ]),
        ],
    }
    vis = _make_visualiser(tmap)

    vis._refresh_visual_cache()

    assert set(vis._node_entries_by_name) == {'A', 'B', 'C'}
    assert len(vis._incoming_edges_by_target['B']) == 2
    assert vis._action_names == ['drive', 'inspect']
    assert vis._action_index == {'drive': 0, 'inspect': 1}


def test_incremental_node_update_publishes_only_local_markers_for_large_map():
    """Dragging one node should only republish the node and connected edges."""
    nodes = []
    total_nodes = 100
    for idx in range(total_nodes):
        edges = []
        if idx < total_nodes - 1:
            edges.append({
                'edge_id': f'N{idx}_N{idx + 1}',
                'node': f'N{idx + 1}',
                'action': 'navigate_to_pose',
            })
        nodes.append(_make_node(f'N{idx}', float(idx), 0.0, edges))

    vis = _make_visualiser({'nodes': nodes})
    vis._refresh_visual_cache()
    full_array = vis._build_full_static_marker_array()

    assert len(full_array.markers) == 300

    vis._publish_incremental_node_update('N50')
    incremental = vis.map_marker_pub.messages[-1]

    assert len(incremental.markers) == 4
    assert {marker.ns for marker in incremental.markers} == {
        '/nodes',
        '/names',
        '/edges',
    }


def test_pose_update_uses_cached_node_lookup():
    """Pose updates should mutate the cached node without scanning all nodes."""
    tmap = {
        'nodes': [
            _make_node('A', 0.0, 0.0, [
                {'edge_id': 'A_B', 'node': 'B', 'action': 'drive'},
            ]),
            _make_node('B', 1.0, 0.0),
        ],
    }
    vis = _make_visualiser(tmap)
    vis._refresh_visual_cache()
    vis._build_full_static_marker_array()
    vis._schedule_republish = lambda: None
    vis._goto_goal_handle = None
    vis._cancel_navigation = lambda: None
    vis._compute_and_highlight_route = lambda _name: None
    vis._send_goto_goal = lambda _name: None
    vis._map_dirty = False

    cached_node = vis._node_entries_by_name['A']
    vis.tmap['nodes'] = _NoIterList(vis.tmap['nodes'])

    pose = Pose()
    pose.position.x = 5.0
    pose.position.y = -2.0
    pose.orientation.w = 1.0

    feedback = SimpleNamespace(
        event_type=InteractiveMarkerFeedback.POSE_UPDATE,
        marker_name='A',
        pose=pose,
    )

    vis._im_feedback(feedback)

    assert vis._map_dirty is True
    assert cached_node['pose']['position']['x'] == 5.0
    assert cached_node['pose']['position']['y'] == -2.0
