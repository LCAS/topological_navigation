#!/usr/bin/env python3
"""Unit tests for :mod:`topological_navigation_visual.viz_utils`.

These tests are pure-Python (no ROS dependencies) so they run both in the
lightweight CI job and under ``colcon test``.
"""

import math
import os
import sys

import pytest

# Make the package importable without an installed/sourced workspace.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from topological_navigation_visual import viz_utils  # noqa: E402


def _node(name, x, y, z=0.0, edges=None, verts=None):
    """Build a minimal tmap2 node entry."""
    return {
        'node': {
            'name': name,
            'pose': {'position': {'x': x, 'y': y, 'z': z}},
            'edges': edges or [],
            'verts': verts or [],
        }
    }


# ----------------------------------------------------------------------
# collect_node_positions / build_position_lookup
# ----------------------------------------------------------------------
def test_collect_node_positions_basic():
    nodes = [_node('a', 1.0, 2.0, 3.0), _node('b', 4.0, 5.0)]
    positions = viz_utils.collect_node_positions(nodes)
    assert positions == [(1.0, 2.0, 3.0), (4.0, 5.0, 0.0)]


def test_collect_node_positions_skips_invalid():
    nodes = [_node('a', 0.0, 0.0), {'node': {'name': 'b'}}, {}]
    positions = viz_utils.collect_node_positions(nodes)
    assert positions == [(0.0, 0.0, 0.0)]


def test_build_position_lookup():
    nodes = [_node('a', 1.0, 2.0), _node('b', 3.0, 4.0)]
    lookup = viz_utils.build_position_lookup(nodes)
    assert lookup == {'a': (1.0, 2.0, 0.0), 'b': (3.0, 4.0, 0.0)}


# ----------------------------------------------------------------------
# group_edge_segments
# ----------------------------------------------------------------------
def test_group_edge_segments_by_action():
    nodes = [
        _node('a', 0.0, 0.0, edges=[
            {'node': 'b', 'action': 'navigate_to_pose'},
            {'node': 'c', 'action': 'row_operation'},
        ]),
        _node('b', 1.0, 0.0, edges=[
            {'node': 'c', 'action': 'navigate_to_pose'},
        ]),
        _node('c', 2.0, 0.0),
    ]
    groups = viz_utils.group_edge_segments(nodes)
    assert set(groups.keys()) == {'navigate_to_pose', 'row_operation'}
    assert len(groups['navigate_to_pose']) == 2
    assert len(groups['row_operation']) == 1
    # Endpoints resolve to the correct coordinates.
    assert groups['row_operation'][0] == ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))


def test_group_edge_segments_skips_missing_target():
    nodes = [
        _node('a', 0.0, 0.0, edges=[{'node': 'ghost', 'action': 'x'}]),
    ]
    groups = viz_utils.group_edge_segments(nodes)
    assert groups == {}


def test_group_edge_segments_z_offset_applied():
    nodes = [
        _node('a', 0.0, 0.0, edges=[{'node': 'b', 'action': 'x'}]),
        _node('b', 1.0, 1.0),
    ]
    groups = viz_utils.group_edge_segments(nodes, z_offset=0.1)
    seg = groups['x'][0]
    assert seg[0][2] == pytest.approx(0.1)
    assert seg[1][2] == pytest.approx(0.1)


# ----------------------------------------------------------------------
# collect_zone_segments
# ----------------------------------------------------------------------
def test_collect_zone_segments_closes_polygon():
    verts = [
        {'x': -1.0, 'y': -1.0},
        {'x': 1.0, 'y': -1.0},
        {'x': 1.0, 'y': 1.0},
        {'x': -1.0, 'y': 1.0},
    ]
    nodes = [_node('a', 10.0, 20.0, verts=verts)]
    segments = viz_utils.collect_zone_segments(nodes)
    # 4 vertices -> 4 segments (polygon is closed).
    assert len(segments) == 4
    # Vertices are translated by the node position.
    assert segments[0][0] == (9.0, 19.0, 0.0)
    # Last segment closes back to the first vertex.
    assert segments[-1][1] == (9.0, 19.0, 0.0)


def test_collect_zone_segments_skips_degenerate():
    nodes = [
        _node('a', 0.0, 0.0, verts=[{'x': 0.0, 'y': 0.0}]),  # 1 vertex
        _node('b', 0.0, 0.0),  # no verts
    ]
    assert viz_utils.collect_zone_segments(nodes) == []


# ----------------------------------------------------------------------
# compute_auto_scale
# ----------------------------------------------------------------------
def test_compute_auto_scale_fallback_for_few_nodes():
    assert viz_utils.compute_auto_scale([], fallback=0.5) == 0.5
    assert viz_utils.compute_auto_scale([(0.0, 0.0, 0.0)], fallback=0.7) == 0.7


def test_compute_auto_scale_coincident_nodes_fallback():
    pts = [(1.0, 1.0, 0.0), (1.0, 1.0, 0.0)]
    assert viz_utils.compute_auto_scale(pts, fallback=0.4) == 0.4


def test_compute_auto_scale_scales_with_spread():
    small = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    large = [(0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)]
    s_small = viz_utils.compute_auto_scale(small)
    s_large = viz_utils.compute_auto_scale(large)
    assert s_large > s_small


def test_compute_auto_scale_is_clamped():
    huge = [(0.0, 0.0, 0.0), (1e9, 1e9, 0.0)]
    assert viz_utils.compute_auto_scale(huge) <= viz_utils.MAX_SCALE
    tiny = [(0.0, 0.0, 0.0), (1e-9, 0.0, 0.0)]
    assert viz_utils.compute_auto_scale(tiny) >= viz_utils.MIN_SCALE


def test_compute_auto_scale_expected_value():
    # 4 nodes on a 3x3 box: diagonal = sqrt(18); spacing = diag/2; scale = spacing*0.5
    pts = [(0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (3.0, 3.0, 0.0), (0.0, 3.0, 0.0)]
    diag = math.hypot(3.0, 3.0)
    expected = (diag / math.sqrt(4)) * 0.5
    assert viz_utils.compute_auto_scale(pts) == pytest.approx(expected)


# ----------------------------------------------------------------------
# ordered_action_names
# ----------------------------------------------------------------------
def test_ordered_action_names_first_seen_order():
    nodes = [
        _node('a', 0.0, 0.0, edges=[
            {'node': 'b', 'action': 'beta'},
            {'node': 'c', 'action': 'alpha'},
        ]),
        _node('b', 1.0, 0.0, edges=[
            {'node': 'c', 'action': 'beta'},  # duplicate
        ]),
        _node('c', 2.0, 0.0),
    ]
    assert viz_utils.ordered_action_names(nodes) == ['beta', 'alpha']
