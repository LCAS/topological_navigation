"""Tests for ``navigation_graph.py``.

Covers the pure-Python navigation-graph module:
- NavStateMachine transitions (valid and invalid)
- plan_route (A* via NetworkX)
- get_route_edges
- merge_action_segments
- compute_boundary_polygon
- get_route_distance
- ActionSegment dataclass
"""

import math
import os

import networkx as nx
import pytest
import yaml

from topological_navigation.navigation_graph import (
    ACTION_TO_STATE,
    ActionSegment,
    NavState,
    NavStateMachine,
    VALID_TRANSITIONS,
    compute_boundary_polygon,
    get_route_distance,
    get_route_edges,
    merge_action_segments,
    plan_route,
)
from topological_navigation.networkx_utils import build_graph_from_tmap


# =====================================================================
# Fixtures
# =====================================================================


def _make_node(name, x, y, z=0.0, verts=None, props=None):
    """Tiny helper to build a graph node attribute dict."""
    return dict(
        x=float(x), y=float(y), z=float(z),
        name=name, parent_frame='map',
        orientation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        verts=verts or [], properties=props or {},
    )


@pytest.fixture
def simple_graph():
    """Linear graph: A -> B -> C -> D.

    A(0,0) --[Nav]--> B(2,0) --[Row]--> C(4,0) --[Row]--> D(6,0)
    """
    G = nx.DiGraph()
    G.add_node('A', **_make_node('A', 0, 0))
    G.add_node('B', **_make_node('B', 2, 0))
    G.add_node('C', **_make_node('C', 4, 0))
    G.add_node('D', **_make_node('D', 6, 0))

    G.add_edge('A', 'B', edge_id='A_B',
               action='NavigateToPose',
               action_type='nav2_msgs/action/NavigateToPose',
               properties={}, weight=1.0)
    G.add_edge('B', 'C', edge_id='B_C',
               action='row_traversal',
               action_type='nav2_msgs/action/NavigateToPose',
               properties={'boundary_left': 0.3, 'boundary_right': 0.4},
               weight=1.0)
    G.add_edge('C', 'D', edge_id='C_D',
               action='row_traversal',
               action_type='nav2_msgs/action/NavigateToPose',
               properties={}, weight=1.0)
    return G


@pytest.fixture
def mixed_graph():
    """Graph with all three action types.

    N1 -[Nav]-> N2 -[Nav]-> N3 -[Row]-> N4 -[Row]-> N5 -[Align]-> N6
    """
    G = nx.DiGraph()
    positions = {
        'N1': (0, 0), 'N2': (2, 0), 'N3': (4, 0),
        'N4': (6, 0), 'N5': (8, 0), 'N6': (10, 0),
    }
    for name, (x, y) in positions.items():
        G.add_node(name, **_make_node(name, x, y))

    edges = [
        ('N1', 'N2', 'NavigateToPose'),
        ('N2', 'N3', 'NavigateToPose'),
        ('N3', 'N4', 'row_traversal'),
        ('N4', 'N5', 'row_traversal'),
        ('N5', 'N6', 'goal_align'),
    ]
    for src, tgt, action in edges:
        G.add_edge(src, tgt,
                   edge_id='%s_%s' % (src, tgt),
                   action=action,
                   action_type='nav2_msgs/action/NavigateToPose',
                   properties={}, weight=1.0)
    return G


@pytest.fixture
def mixed_actions_tmap():
    """Load the mixed_actions_map.yaml fixture as a dict."""
    here = os.path.dirname(__file__)
    path = os.path.join(here, 'fixtures', 'mixed_actions_map.yaml')
    with open(path, 'r') as f:
        return yaml.safe_load(f)


# =====================================================================
# NavStateMachine tests
# =====================================================================


class TestNavStateMachine:
    """Validate state machine transitions."""

    def test_initial_state_is_idle(self):
        sm = NavStateMachine()
        assert sm.state == NavState.IDLE

    def test_valid_startup_sequence(self):
        sm = NavStateMachine()
        assert sm.transition(NavState.WAITING_FOR_MAP)
        assert sm.transition(NavState.WAITING_FOR_LOCALISATION)
        assert sm.transition(NavState.READY)
        assert sm.state == NavState.READY

    def test_planning_to_executing(self):
        sm = NavStateMachine()
        sm.transition(NavState.WAITING_FOR_MAP)
        sm.transition(NavState.WAITING_FOR_LOCALISATION)
        sm.transition(NavState.READY)
        assert sm.transition(NavState.PLANNING)
        assert sm.transition(NavState.EXECUTING_NAVIGATE_TO_POSE)
        assert sm.is_executing()
        assert not sm.is_terminal()

    def test_execute_to_succeed_to_ready(self):
        sm = NavStateMachine()
        sm.transition(NavState.WAITING_FOR_MAP)
        sm.transition(NavState.WAITING_FOR_LOCALISATION)
        sm.transition(NavState.READY)
        sm.transition(NavState.PLANNING)
        sm.transition(NavState.EXECUTING_NAVIGATE_TO_POSE)
        assert sm.transition(NavState.SUCCEEDED)
        assert sm.is_terminal()
        assert sm.reset()
        assert sm.state == NavState.READY

    def test_execute_to_failed(self):
        sm = NavStateMachine()
        sm.transition(NavState.WAITING_FOR_MAP)
        sm.transition(NavState.WAITING_FOR_LOCALISATION)
        sm.transition(NavState.READY)
        sm.transition(NavState.PLANNING)
        sm.transition(NavState.EXECUTING_ROW_TRAVERSAL)
        assert sm.transition(NavState.FAILED)
        assert sm.is_terminal()

    def test_execute_to_cancelled(self):
        sm = NavStateMachine()
        sm.transition(NavState.WAITING_FOR_MAP)
        sm.transition(NavState.WAITING_FOR_LOCALISATION)
        sm.transition(NavState.READY)
        sm.transition(NavState.PLANNING)
        sm.transition(NavState.EXECUTING_GOAL_ALIGN)
        assert sm.transition(NavState.CANCELLED)
        assert sm.is_terminal()

    def test_execute_to_different_execute(self):
        """Can transition directly between executing states (recovery/segment switch)."""
        sm = NavStateMachine()
        sm.transition(NavState.WAITING_FOR_MAP)
        sm.transition(NavState.WAITING_FOR_LOCALISATION)
        sm.transition(NavState.READY)
        sm.transition(NavState.PLANNING)
        sm.transition(NavState.EXECUTING_ROW_TRAVERSAL)
        assert sm.transition(NavState.EXECUTING_NAVIGATE_TO_POSE)

    def test_invalid_transition_returns_false(self):
        sm = NavStateMachine()
        # IDLE -> READY is invalid (must go via WAITING_FOR_*)
        assert not sm.transition(NavState.READY)
        assert sm.state == NavState.IDLE

    def test_invalid_skip_waiting(self):
        sm = NavStateMachine()
        sm.transition(NavState.WAITING_FOR_MAP)
        # Cannot skip WAITING_FOR_LOCALISATION
        assert not sm.transition(NavState.READY)

    def test_switch_between_executing_states(self):
        """Can move directly between executing states (segment switch)."""
        sm = NavStateMachine()
        sm.transition(NavState.WAITING_FOR_MAP)
        sm.transition(NavState.WAITING_FOR_LOCALISATION)
        sm.transition(NavState.READY)
        sm.transition(NavState.PLANNING)
        sm.transition(NavState.EXECUTING_NAVIGATE_TO_POSE)
        assert sm.transition(NavState.EXECUTING_ROW_TRAVERSAL)
        assert sm.transition(NavState.EXECUTING_GOAL_ALIGN)
        assert sm.transition(NavState.EXECUTING_NAVIGATE_TO_POSE)

    def test_all_action_to_state_mappings_exist(self):
        for action, state in ACTION_TO_STATE.items():
            assert isinstance(state, NavState)
            assert 'EXECUTING' in state.value

    def test_all_states_have_transitions_defined(self):
        for state in NavState:
            assert state in VALID_TRANSITIONS


# =====================================================================
# plan_route tests
# =====================================================================


class TestPlanRoute:
    """Route planning via NetworkX A*."""

    def test_direct_route(self, simple_graph):
        route = plan_route(simple_graph, 'A', 'D')
        assert route == ['A', 'B', 'C', 'D']

    def test_partial_route(self, simple_graph):
        route = plan_route(simple_graph, 'B', 'D')
        assert route == ['B', 'C', 'D']

    def test_same_origin_target(self, simple_graph):
        route = plan_route(simple_graph, 'A', 'A')
        assert route == ['A']

    def test_no_route_reverse(self, simple_graph):
        """No backward edges -> no route D to A."""
        route = plan_route(simple_graph, 'D', 'A')
        assert route is None

    def test_unknown_origin(self, simple_graph):
        route = plan_route(simple_graph, 'X', 'D')
        assert route is None

    def test_unknown_target(self, simple_graph):
        route = plan_route(simple_graph, 'A', 'X')
        assert route is None

    def test_avoid_edges(self, simple_graph):
        """With A->B avoided, add alternative A->C to enable a bypass."""
        simple_graph.add_edge('A', 'C', edge_id='A_C',
                              action='NavigateToPose', properties={},
                              weight=10.0)
        route = plan_route(simple_graph, 'A', 'D', avoid_edges=['A_B'])
        assert route is not None
        assert route[0] == 'A'
        # Must bypass B via the new A->C edge
        assert 'C' in route

    def test_avoid_all_paths(self, simple_graph):
        """If the only path's edges are all avoided, return None."""
        route = plan_route(
            simple_graph, 'A', 'D',
            avoid_edges=['A_B'],
        )
        assert route is None

    def test_weighted_preference(self):
        """A* should prefer the cheaper route."""
        G = nx.DiGraph()
        G.add_node('S', **_make_node('S', 0, 0))
        G.add_node('M', **_make_node('M', 5, 0))
        G.add_node('T', **_make_node('T', 10, 0))
        G.add_node('D', **_make_node('D', 5, 5))
        G.add_edge('S', 'M', edge_id='SM', action='NavigateToPose',
                   properties={}, weight=1.0)
        G.add_edge('M', 'T', edge_id='MT', action='NavigateToPose',
                   properties={}, weight=1.0)
        G.add_edge('S', 'D', edge_id='SD', action='NavigateToPose',
                   properties={}, weight=100.0)
        G.add_edge('D', 'T', edge_id='DT', action='NavigateToPose',
                   properties={}, weight=100.0)
        route = plan_route(G, 'S', 'T')
        assert route == ['S', 'M', 'T']


# =====================================================================
# get_route_edges tests
# =====================================================================


class TestGetRouteEdges:
    """Extract edge data from route node list."""

    def test_full_route(self, simple_graph):
        edges = get_route_edges(simple_graph, ['A', 'B', 'C', 'D'])
        assert len(edges) == 3
        assert edges[0]['edge_id'] == 'A_B'
        assert edges[0]['source'] == 'A'
        assert edges[0]['target'] == 'B'
        assert edges[2]['edge_id'] == 'C_D'

    def test_single_edge(self, simple_graph):
        edges = get_route_edges(simple_graph, ['A', 'B'])
        assert len(edges) == 1
        assert edges[0]['action'] == 'NavigateToPose'

    def test_empty_route(self, simple_graph):
        assert get_route_edges(simple_graph, []) == []

    def test_single_node(self, simple_graph):
        assert get_route_edges(simple_graph, ['A']) == []

    def test_actions_preserved(self, simple_graph):
        edges = get_route_edges(simple_graph, ['A', 'B', 'C'])
        assert edges[0]['action'] == 'NavigateToPose'
        assert edges[1]['action'] == 'row_traversal'


# =====================================================================
# merge_action_segments tests
# =====================================================================


class TestMergeActionSegments:
    """Merge consecutive same-action edges into segments."""

    def test_mixed_actions(self, mixed_graph):
        route = ['N1', 'N2', 'N3', 'N4', 'N5', 'N6']
        edges = get_route_edges(mixed_graph, route)
        segments = merge_action_segments(edges)

        assert len(segments) == 3

        # NavigateToPose x2
        assert segments[0].action_type == 'NavigateToPose'
        assert segments[0].num_edges == 2
        assert segments[0].first_source == 'N1'
        assert segments[0].last_target == 'N3'

        # row_traversal x2
        assert segments[1].action_type == 'row_traversal'
        assert segments[1].num_edges == 2
        assert segments[1].first_source == 'N3'
        assert segments[1].last_target == 'N5'

        # goal_align x1
        assert segments[2].action_type == 'goal_align'
        assert segments[2].num_edges == 1
        assert segments[2].first_source == 'N5'
        assert segments[2].last_target == 'N6'

    def test_single_action_type(self, simple_graph):
        edges = get_route_edges(simple_graph, ['B', 'C', 'D'])
        segments = merge_action_segments(edges)
        assert len(segments) == 1
        assert segments[0].action_type == 'row_traversal'
        assert segments[0].num_edges == 2

    def test_empty_edges(self):
        assert merge_action_segments([]) == []

    def test_alternating_never_merges(self):
        edges = [
            {'edge_id': 'e1', 'source': 'A', 'target': 'B',
             'action': 'NavigateToPose', 'properties': {}},
            {'edge_id': 'e2', 'source': 'B', 'target': 'C',
             'action': 'row_traversal', 'properties': {}},
            {'edge_id': 'e3', 'source': 'C', 'target': 'D',
             'action': 'NavigateToPose', 'properties': {}},
        ]
        segments = merge_action_segments(edges)
        assert len(segments) == 3
        assert all(s.num_edges == 1 for s in segments)

    def test_all_same_type_single_segment(self):
        edges = [
            {'edge_id': 'e%d' % i, 'source': 'n%d' % i,
             'target': 'n%d' % (i + 1), 'action': 'goal_align',
             'properties': {}}
            for i in range(5)
        ]
        segments = merge_action_segments(edges)
        assert len(segments) == 1
        assert segments[0].num_edges == 5

    def test_edge_ids_preserved(self, mixed_graph):
        edges = get_route_edges(mixed_graph, ['N1', 'N2', 'N3'])
        segments = merge_action_segments(edges)
        assert segments[0].edge_ids == ['N1_N2', 'N2_N3']


# =====================================================================
# compute_boundary_polygon tests
# =====================================================================


class TestComputeRowBoundary:
    """Compute corridor polygons for RowTraversal segments."""

    def test_single_edge_horizontal(self, simple_graph):
        """B(2,0) -> C(4,0): horizontal edge, polygon should be a
        rectangle centred along the x-axis."""
        edges = get_route_edges(simple_graph, ['B', 'C'])
        segments = merge_action_segments(edges)
        assert len(segments) == 1

        poly = compute_boundary_polygon(
            simple_graph, segments[0],
            default_left=0.5, default_right=0.5,
        )
        # Rectangle: 2 left + 2 right = 4 points
        assert len(poly) == 4

        # Left points (first half) should be above y=0
        left_y = [p[1] for p in poly[:2]]
        assert all(y > 0 for y in left_y)

        # Right points (second half) should be below y=0
        right_y = [p[1] for p in poly[2:]]
        assert all(y < 0 for y in right_y)

    def test_custom_distances_from_properties(self, simple_graph):
        """B->C edge has boundary_left=0.3, boundary_right=0.4."""
        edges = get_route_edges(simple_graph, ['B', 'C'])
        segments = merge_action_segments(edges)

        poly = compute_boundary_polygon(simple_graph, segments[0])

        # Left offset should be 0.3 (y = +0.3 for horizontal edge)
        assert abs(poly[0][1] - 0.3) < 0.01
        # Right offset should be -0.4
        assert abs(poly[3][1] + 0.4) < 0.01

    def test_merged_edges_more_points(self, simple_graph):
        """B->C->D: two edges merged -> 3 waypoints -> 6 points."""
        edges = get_route_edges(simple_graph, ['B', 'C', 'D'])
        segments = merge_action_segments(edges)

        poly = compute_boundary_polygon(
            simple_graph, segments[0],
            default_left=0.5, default_right=0.5,
        )
        assert len(poly) == 6

    def test_empty_segment_returns_empty(self, simple_graph):
        segment = ActionSegment(action_type='row_traversal')
        poly = compute_boundary_polygon(simple_graph, segment)
        assert poly == []

    def test_polygon_is_closed_corridor(self, simple_graph):
        """All left points should be on one side, right on the other."""
        edges = get_route_edges(simple_graph, ['B', 'C', 'D'])
        segments = merge_action_segments(edges)
        poly = compute_boundary_polygon(
            simple_graph, segments[0],
            default_left=1.0, default_right=1.0,
        )
        # For horizontal edges, left = positive y, right = negative y
        n_wp = 3  # three waypoints
        left_pts = poly[:n_wp]
        right_pts = poly[n_wp:]
        assert all(p[1] > 0 for p in left_pts)
        assert all(p[1] < 0 for p in right_pts)

    def test_diagonal_edge(self):
        """45-degree edge: verify perpendicular offset direction."""
        G = nx.DiGraph()
        G.add_node('P', **_make_node('P', 0, 0))
        G.add_node('Q', **_make_node('Q', 1, 1))
        G.add_edge('P', 'Q', edge_id='PQ', action='row_traversal',
                   properties={}, weight=1.0)

        edges = get_route_edges(G, ['P', 'Q'])
        segments = merge_action_segments(edges)
        poly = compute_boundary_polygon(
            G, segments[0], default_left=1.0, default_right=1.0,
        )
        assert len(poly) == 4

        # Verify the width: distance from left to right at each
        # waypoint should be left_dist + right_dist = 2.0
        for i in range(2):
            lx, ly = poly[i]
            rx, ry = poly[3 - i]
            dist = math.hypot(rx - lx, ry - ly)
            assert abs(dist - 2.0) < 0.01


# =====================================================================
# get_route_distance tests
# =====================================================================


class TestGetRouteDistance:
    """Total Euclidean distance along a route."""

    def test_full_route(self, simple_graph):
        dist = get_route_distance(simple_graph, ['A', 'B', 'C', 'D'])
        assert abs(dist - 6.0) < 0.01  # 2 + 2 + 2

    def test_single_edge(self, simple_graph):
        dist = get_route_distance(simple_graph, ['A', 'B'])
        assert abs(dist - 2.0) < 0.01

    def test_single_node(self, simple_graph):
        assert get_route_distance(simple_graph, ['A']) == 0.0

    def test_empty_route(self, simple_graph):
        assert get_route_distance(simple_graph, []) == 0.0


# =====================================================================
# ActionSegment dataclass tests
# =====================================================================


class TestActionSegment:
    """Properties and edge case handling."""

    def test_empty_segment(self):
        seg = ActionSegment(action_type='NavigateToPose')
        assert seg.is_empty
        assert seg.first_source is None
        assert seg.last_target is None
        assert seg.num_edges == 0

    def test_populated_segment(self):
        seg = ActionSegment(
            action_type='row_traversal',
            edge_ids=['e1', 'e2'],
            source_nodes=['A', 'B'],
            target_nodes=['B', 'C'],
            edge_data=[{'edge_id': 'e1'}, {'edge_id': 'e2'}],
        )
        assert not seg.is_empty
        assert seg.first_source == 'A'
        assert seg.last_target == 'C'
        assert seg.num_edges == 2


# =====================================================================
# Integration: build_graph_from_tmap + plan_route
# =====================================================================


class TestIntegrationWithBuildGraph:
    """End-to-end: YAML fixture -> NetworkX graph -> route planning."""

    def test_build_and_plan_route(self, mixed_actions_tmap):
        graph = build_graph_from_tmap(mixed_actions_tmap)
        assert graph is not None
        assert graph.number_of_nodes() == 6

        route = plan_route(graph, 'N1', 'N6')
        assert route is not None
        assert route[0] == 'N1'
        assert route[-1] == 'N6'

    def test_merge_from_yaml(self, mixed_actions_tmap):
        graph = build_graph_from_tmap(mixed_actions_tmap)
        route = plan_route(graph, 'N1', 'N6')
        edges = get_route_edges(graph, route)
        segments = merge_action_segments(edges)

        # Expect 3 segments: Nav(2), Row(2), Align(1)
        assert len(segments) == 3
        types = [s.action_type for s in segments]
        assert types == ['NavigateToPose', 'row_traversal', 'goal_align']

    def test_boundary_from_yaml_props(self, mixed_actions_tmap):
        """The YAML defines boundary_left=0.4 and boundary_right=0.6
        on the first row_traversal edge (N3->N4)."""
        graph = build_graph_from_tmap(mixed_actions_tmap)
        route = plan_route(graph, 'N1', 'N6')
        edges = get_route_edges(graph, route)
        segments = merge_action_segments(edges)

        row_seg = [s for s in segments
                   if s.action_type == 'row_traversal'][0]
        poly = compute_boundary_polygon(graph, row_seg)

        # 3 waypoints (N3, N4, N5) -> 6 polygon points
        assert len(poly) == 6

        # Verify asymmetric boundary (left=0.4, right=0.6)
        # For horizontal edges: left_pts have positive y, right negative y
        left_y = poly[0][1]
        right_y = poly[5][1]
        assert abs(left_y - 0.4) < 0.01
        assert abs(right_y + 0.6) < 0.01
