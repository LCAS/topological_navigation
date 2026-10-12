"""Unit tests for the map_analyser module.

Covers geometry helpers (polygon overlap / point-in-polygon), graph
analysis (orphaned nodes, disconnected components, statistics),
grid-angle-deviation node filtering/detection/alignment, SVG
generation, and the top-level ``analyse_map`` / CLI ``check``
behaviour.
"""

import math
import os

import networkx as nx
import pytest

import yaml

from topological_navigation.map_analyser import (
    DEFAULT_ANCHORS_KEY,
    DEFAULT_GRID_ANGLE_THRESHOLD_DEG,
    AnalysisResult,
    GridAlignResult,
    MinifyResult,
    analyse_map,
    build_arg_parser,
    compute_edge_bearing,
    compute_grid_alignment_adjustments,
    compute_statistics,
    find_disconnected_components,
    find_grid_angle_deviations,
    find_orphaned_nodes,
    find_overlapping_influence_zones,
    generate_svg,
    get_node_polygon,
    grid_align_map,
    grid_angle_deviation,
    is_bidirectional_edge,
    main,
    minify_map,
    node_matches_filter,
    node_matches_filter_expression,
    parse_node_filter,
    parse_node_filter_expression,
    point_in_polygon,
    polygons_overlap,
    select_nodes_by_filters,
)
from topological_navigation.tmap_utils import CustomSafeLoader, load_tmap2_file


FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")
SIMPLE_MAP = os.path.join(FIXTURES, "simple_map.yaml")
COMPLEX_MAP = os.path.join(FIXTURES, "complex_map.yaml")
MIXED_ACTIONS_MAP = os.path.join(FIXTURES, "mixed_actions_map.yaml")
POLYGON_SHAPES_MAP = os.path.join(FIXTURES, "polygon_shapes_map.yaml")


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

class TestPointInPolygon:
    def test_point_inside_square(self):
        square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert point_in_polygon((0, 0), square) is True

    def test_point_outside_square(self):
        square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert point_in_polygon((5, 5), square) is False

    def test_degenerate_polygon_returns_false(self):
        assert point_in_polygon((0, 0), [(0, 0), (1, 1)]) is False


class TestPolygonsOverlap:
    def test_overlapping_squares(self):
        a = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        b = [(0, 0), (2, 0), (2, 2), (0, 2)]
        assert polygons_overlap(a, b) is True

    def test_disjoint_squares(self):
        a = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        b = [(10, 10), (12, 10), (12, 12), (10, 12)]
        assert polygons_overlap(a, b) is False

    def test_fully_contained_polygon(self):
        outer = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
        inner = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert polygons_overlap(outer, inner) is True

    def test_empty_polygon_never_overlaps(self):
        square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        assert polygons_overlap(square, []) is False


class TestGetNodePolygon:
    def test_translates_verts_by_node_position(self):
        graph = nx.DiGraph()
        graph.add_node(
            "N1", x=10.0, y=5.0,
            verts=[{"x": -1.0, "y": -1.0}, {"x": 1.0, "y": -1.0},
                   {"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.0}],
        )
        poly = get_node_polygon(graph, "N1")
        assert poly == [(9.0, 4.0), (11.0, 4.0), (11.0, 6.0), (9.0, 6.0)]

    def test_no_verts_returns_empty_list(self):
        graph = nx.DiGraph()
        graph.add_node("N1", x=0.0, y=0.0, verts=[])
        assert get_node_polygon(graph, "N1") == []


# ---------------------------------------------------------------------------
# Graph analysis
# ---------------------------------------------------------------------------

class TestFindOrphanedNodes:
    def test_simple_map_has_one_orphan(self):
        result = analyse_map(SIMPLE_MAP)
        assert result.orphaned_nodes == ["WP1"]

    def test_mixed_actions_map_has_no_orphans(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        assert result.orphaned_nodes == []

    def test_isolated_node_is_orphaned(self):
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0, verts=[])
        assert find_orphaned_nodes(graph) == ["A"]


class TestFindDisconnectedComponents:
    def test_fully_connected_map_is_single_component(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        assert len(result.disconnected_components) == 1

    def test_disconnected_graph_has_multiple_components(self):
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0, verts=[])
        graph.add_node("B", x=1.0, y=0.0, verts=[])
        components = find_disconnected_components(graph)
        assert len(components) == 2


class TestIsBidirectionalEdge:
    def test_bidirectional_pair(self):
        graph = nx.DiGraph()
        graph.add_edge("A", "B")
        graph.add_edge("B", "A")
        assert is_bidirectional_edge(graph, "A", "B") is True

    def test_unidirectional_pair(self):
        graph = nx.DiGraph()
        graph.add_edge("A", "B")
        assert is_bidirectional_edge(graph, "A", "B") is False


class TestComputeStatistics:
    def test_mixed_actions_map_statistics(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        stats = result.statistics
        assert stats["num_nodes"] == 6
        assert stats["num_edges"] == 10
        assert stats["bidirectional_edge_count"] == 5
        assert stats["unidirectional_edge_count"] == 0
        assert stats["action_counts"] == {
            "navigate_to_pose": 4, "row_traversal": 4, "goal_align": 2,
        }
        assert stats["total_edge_length"] == pytest.approx(30.0)


class TestFindOverlappingInfluenceZones:
    def test_polygon_shapes_map_has_overlaps(self):
        result = analyse_map(POLYGON_SHAPES_MAP)
        pairs = {frozenset((o["node_a"], o["node_b"])) for o in result.overlaps}
        assert frozenset({"TriangleNode", "LargeNode"}) in pairs
        assert frozenset({"PentagonNode", "LargeNode"}) in pairs
        assert frozenset({"LargeNode", "RectangleNode"}) in pairs

    def test_simple_map_has_no_overlaps(self):
        result = analyse_map(SIMPLE_MAP)
        assert result.overlaps == []


# ---------------------------------------------------------------------------
# Node filtering (name/property, used by grid-angle-deviation / grid-align)
# ---------------------------------------------------------------------------

def _make_filter_graph() -> "nx.DiGraph":
    graph = nx.DiGraph()
    graph.add_node(
        "RowA1", x=0.0, y=0.0,
        properties={"semantics": "row_entry", "roboflow": {"enabled": True}},
    )
    graph.add_node("RowA2", x=1.0, y=0.0, properties={"semantics": "row_exit"})
    graph.add_node("Junction1", x=2.0, y=0.0, properties={})
    return graph


def _make_field_filter_graph() -> "nx.DiGraph":
    """Graph with 'field'/'tunnel'/'roboflow.enabled' properties as raised in
    issue feedback, for testing boolean filter expressions."""
    graph = nx.DiGraph()
    graph.add_node(
        "A", x=0.0, y=0.0,
        properties={"roboflow": {"enabled": True}, "field": 1, "tunnel": 17},
    )
    graph.add_node(
        "B", x=1.0, y=0.0,
        properties={"roboflow": {"enabled": True}, "field": 2, "tunnel": 18},
    )
    graph.add_node(
        "C", x=2.0, y=0.0,
        properties={"roboflow": {"enabled": False}, "field": 1, "tunnel": 17},
    )
    graph.add_node(
        "D", x=3.0, y=0.0,
        properties={"roboflow": {"enabled": True}, "field": 3, "tunnel": 19},
    )
    return graph


class TestParseNodeFilter:
    def test_parses_name_filter(self):
        node_filter = parse_node_filter("name:Row*")
        assert node_filter.kind == "name"
        assert node_filter.pattern == "Row*"

    def test_parses_property_presence_filter(self):
        node_filter = parse_node_filter("property:roboflow.enabled")
        assert node_filter.kind == "property"
        assert node_filter.key == "roboflow.enabled"
        assert node_filter.value is None

    def test_parses_property_value_filter(self):
        node_filter = parse_node_filter("property:semantics=row_entry")
        assert node_filter.kind == "property"
        assert node_filter.key == "semantics"
        assert node_filter.value == "row_entry"

    def test_missing_colon_raises(self):
        with pytest.raises(ValueError):
            parse_node_filter("bogus-spec")

    def test_unknown_filter_type_raises(self):
        with pytest.raises(ValueError):
            parse_node_filter("foo:bar")

    def test_empty_name_pattern_raises(self):
        with pytest.raises(ValueError):
            parse_node_filter("name:")

    def test_empty_property_key_raises(self):
        with pytest.raises(ValueError):
            parse_node_filter("property:")


class TestNodeMatchesFilter:
    def test_name_glob_matches(self):
        graph = _make_filter_graph()
        assert node_matches_filter(graph, "RowA1", parse_node_filter("name:Row*")) is True
        assert node_matches_filter(graph, "Junction1", parse_node_filter("name:Row*")) is False

    def test_property_presence_matches(self):
        graph = _make_filter_graph()
        node_filter = parse_node_filter("property:roboflow.enabled")
        assert node_matches_filter(graph, "RowA1", node_filter) is True
        assert node_matches_filter(graph, "RowA2", node_filter) is False

    def test_property_value_matches_case_insensitively(self):
        graph = _make_filter_graph()
        node_filter = parse_node_filter("property:semantics=ROW_EXIT")
        assert node_matches_filter(graph, "RowA2", node_filter) is True
        assert node_matches_filter(graph, "RowA1", node_filter) is False

    def test_missing_property_key_does_not_match(self):
        graph = _make_filter_graph()
        node_filter = parse_node_filter("property:nosuchkey")
        assert node_matches_filter(graph, "RowA1", node_filter) is False


class TestSelectNodesByFilters:
    def test_no_filters_selects_all_nodes(self):
        graph = _make_filter_graph()
        assert select_nodes_by_filters(graph) == set(graph.nodes())

    def test_single_name_filter(self):
        graph = _make_filter_graph()
        assert select_nodes_by_filters(graph, filters=["name:Row*"]) == {"RowA1", "RowA2"}

    def test_disjunction_of_multiple_filters(self):
        graph = _make_filter_graph()
        selected = select_nodes_by_filters(graph, filters=["name:RowA1", "name:Junction1"])
        assert selected == {"RowA1", "Junction1"}

    def test_exclude_filter_removes_matches(self):
        graph = _make_filter_graph()
        selected = select_nodes_by_filters(
            graph, filters=["name:Row*"], exclude_filters=["name:RowA2"]
        )
        assert selected == {"RowA1"}

    def test_exclude_without_include_applies_to_all_nodes(self):
        graph = _make_filter_graph()
        selected = select_nodes_by_filters(graph, exclude_filters=["name:Junction1"])
        assert selected == {"RowA1", "RowA2"}


class TestParseNodeFilterExpression:
    def test_bare_term_behaves_like_single_filter(self):
        expr = parse_node_filter_expression("name:Row*")
        assert expr == ("leaf", parse_node_filter("name:Row*"))

    def test_and_combination(self):
        graph = _make_field_filter_graph()
        expr = parse_node_filter_expression("property:roboflow.enabled and property:field=1")
        assert node_matches_filter_expression(graph, "A", expr) is True
        assert node_matches_filter_expression(graph, "C", expr) is False  # roboflow disabled
        assert node_matches_filter_expression(graph, "B", expr) is False  # field=2

    def test_or_combination(self):
        graph = _make_field_filter_graph()
        expr = parse_node_filter_expression("property:field=1 or property:field=2")
        assert node_matches_filter_expression(graph, "A", expr) is True
        assert node_matches_filter_expression(graph, "B", expr) is True
        assert node_matches_filter_expression(graph, "C", expr) is True
        assert node_matches_filter_expression(graph, "D", expr) is False

    def test_not_negates(self):
        graph = _make_field_filter_graph()
        expr = parse_node_filter_expression("not property:roboflow.enabled")
        assert node_matches_filter_expression(graph, "C", expr) is True
        assert node_matches_filter_expression(graph, "A", expr) is False

    def test_and_or_precedence_with_parentheses(self):
        graph = _make_field_filter_graph()
        expr = parse_node_filter_expression(
            "property:roboflow.enabled and (property:field=1 or property:field=2)"
        )
        assert node_matches_filter_expression(graph, "A", expr) is True
        assert node_matches_filter_expression(graph, "B", expr) is True
        assert node_matches_filter_expression(graph, "C", expr) is False  # roboflow disabled
        assert node_matches_filter_expression(graph, "D", expr) is False  # field=3

    def test_and_binds_tighter_than_or_without_parentheses(self):
        # "a or b and c" == "a or (b and c)"
        graph = _make_field_filter_graph()
        expr = parse_node_filter_expression(
            "property:tunnel=19 or property:roboflow.enabled and property:field=1"
        )
        assert node_matches_filter_expression(graph, "A", expr) is True  # roboflow & field=1
        assert node_matches_filter_expression(graph, "D", expr) is True  # tunnel=19
        assert node_matches_filter_expression(graph, "B", expr) is False

    def test_case_insensitive_operators(self):
        graph = _make_field_filter_graph()
        expr = parse_node_filter_expression("property:field=1 AND property:roboflow.enabled")
        assert node_matches_filter_expression(graph, "A", expr) is True

    def test_unbalanced_parenthesis_raises(self):
        with pytest.raises(ValueError):
            parse_node_filter_expression("(property:field=1")

    def test_trailing_operator_raises(self):
        with pytest.raises(ValueError):
            parse_node_filter_expression("property:field=1 and")

    def test_empty_expression_raises(self):
        with pytest.raises(ValueError):
            parse_node_filter_expression("")


class TestSelectNodesByFiltersWithExpressions:
    def test_select_nodes_by_combined_expression(self):
        graph = _make_field_filter_graph()
        selected = select_nodes_by_filters(
            graph,
            filters=["property:roboflow.enabled and (property:field=1 or property:field=2)"],
        )
        assert selected == {"A", "B"}

    def test_exclude_with_combined_expression(self):
        graph = _make_field_filter_graph()
        selected = select_nodes_by_filters(
            graph,
            filters=["property:roboflow.enabled"],
            exclude_filters=["property:field=3"],
        )
        assert selected == {"A", "B"}


# ---------------------------------------------------------------------------
# Grid angle deviation
# ---------------------------------------------------------------------------
#
# The grid-angle-deviation check is about the angle *between* a node's own
# edges, not about the edges' absolute orientation relative to the map's
# global x/y axes: a node whose edges are mutually at right angles must
# never be flagged, no matter how the whole map (or just that node's local
# neighbourhood) happens to be rotated.

def _axis_aligned_graph() -> "nx.DiGraph":
    graph = nx.DiGraph()
    graph.add_node("A", x=0.0, y=0.0)
    graph.add_node("B", x=10.0, y=0.0)  # due east of A
    graph.add_node("C", x=0.0, y=10.0)  # due north of A
    graph.add_edge(
        "A", "B", edge_id="A_B", action="navigate_to_pose", action_type="", properties={}, weight=1.0
    )
    graph.add_edge(
        "A", "C", edge_id="A_C", action="navigate_to_pose", action_type="", properties={}, weight=1.0
    )
    return graph


def _bearing_graph(center: str, center_pos, edges) -> "nx.DiGraph":
    """Build a star graph: *center* at *center_pos*, with one outgoing edge
    to a node placed at each ``(name, bearing_deg)`` pair in *edges*
    (10 units away from the centre)."""
    graph = nx.DiGraph()
    graph.add_node(center, x=center_pos[0], y=center_pos[1])
    for name, bearing in edges:
        x = center_pos[0] + 10.0 * math.cos(math.radians(bearing))
        y = center_pos[1] + 10.0 * math.sin(math.radians(bearing))
        graph.add_node(name, x=x, y=y)
        graph.add_edge(
            center, name, edge_id=f"{center}_{name}", action="navigate_to_pose",
            action_type="", properties={}, weight=1.0,
        )
    return graph


class TestGridAngleDeviationHelpers:
    def test_grid_angle_deviation_zero_on_cardinal_directions(self):
        for bearing in (0.0, 90.0, 180.0, 270.0, 360.0):
            assert grid_angle_deviation(bearing) == pytest.approx(0.0, abs=1e-9)

    def test_grid_angle_deviation_max_at_45_degrees(self):
        assert grid_angle_deviation(45.0) == pytest.approx(45.0)
        assert grid_angle_deviation(135.0) == pytest.approx(45.0)

    def test_grid_angle_deviation_uses_reference_phase(self):
        """A bearing that is a 90-degree multiple away from a non-zero
        reference phase (i.e. a locally-rotated grid) has zero deviation,
        not a deviation relative to the global 0/90/180/270 axes."""
        assert grid_angle_deviation(120.0, reference_phase_deg=30.0) == pytest.approx(0.0, abs=1e-9)
        assert grid_angle_deviation(30.0, reference_phase_deg=30.0) == pytest.approx(0.0, abs=1e-9)
        assert grid_angle_deviation(75.0, reference_phase_deg=30.0) == pytest.approx(45.0)

    def test_compute_edge_bearing(self):
        graph = _axis_aligned_graph()
        assert compute_edge_bearing(graph, "A", "B") == pytest.approx(0.0)
        assert compute_edge_bearing(graph, "A", "C") == pytest.approx(90.0)
        assert compute_edge_bearing(graph, "B", "A") == pytest.approx(180.0)


class TestFindGridAngleDeviations:
    def test_axis_aligned_grid_has_no_deviations(self):
        graph = _axis_aligned_graph()
        assert find_grid_angle_deviations(graph) == []

    def test_rotated_but_mutually_perpendicular_edges_have_no_deviations(self):
        """Edges at 30/120/210/300 degrees are all still 90-degree
        multiples *of one another*, even though none of them lines up
        with the global 0/90/180/270 directions."""
        graph = _bearing_graph("A", (0.0, 0.0), [
            ("B", 30.0), ("C", 120.0), ("D", 210.0), ("E", 300.0),
        ])
        assert find_grid_angle_deviations(graph) == []

    def test_single_edge_node_is_never_flagged(self):
        """A lone diagonal edge has no other edge at the same node to
        measure an angle against, so it must not be flagged even though
        its absolute bearing is 45 degrees away from the global grid."""
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0)
        graph.add_node("B", x=10.0, y=10.0)  # 45 degrees
        graph.add_edge(
            "A", "B", edge_id="A_B", action="navigate_to_pose", action_type="", properties={}, weight=1.0
        )
        assert find_grid_angle_deviations(graph) == []

    def test_odd_edge_out_is_flagged_among_consistent_edges(self):
        """Two edges at right angles to each other plus one diagonal edge:
        only the diagonal edge is inconsistent with the other two, so only
        it should be flagged."""
        graph = _bearing_graph("A", (0.0, 0.0), [
            ("East", 0.0), ("North", 90.0), ("Diag", 45.0),
        ])
        findings = find_grid_angle_deviations(graph)
        assert {f["node"] for f in findings} == {"A"}
        assert {f["neighbour"] for f in findings} == {"Diag"}
        assert findings[0]["deviation_deg"] == pytest.approx(45.0)
        assert findings[0]["edge_ids"] == ["A_Diag"]

    def test_incoming_edge_is_considered(self):
        """Incoming edges must contribute to the local reference phase and
        be checked, exactly like outgoing edges: here East (outgoing) and
        North (incoming) reinforce each other at a right angle, so the
        diagonal incoming edge is the only one flagged."""
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0)
        graph.add_node("East", x=10.0, y=0.0)
        graph.add_node("North", x=0.0, y=10.0)
        graph.add_node("Diag", x=10.0, y=10.0)
        graph.add_edge(
            "A", "East", edge_id="A_East", action="navigate_to_pose", action_type="", properties={}, weight=1.0
        )
        graph.add_edge(
            "North", "A", edge_id="North_A", action="navigate_to_pose", action_type="", properties={}, weight=1.0
        )
        graph.add_edge(
            "Diag", "A", edge_id="Diag_A", action="navigate_to_pose", action_type="", properties={}, weight=1.0
        )

        findings = find_grid_angle_deviations(graph, nodes=["A"])
        assert len(findings) == 1
        assert findings[0]["node"] == "A"
        assert findings[0]["neighbour"] == "Diag"

    def test_threshold_suppresses_small_deviations(self):
        """East and North reinforce a right angle; C drifts 10 degrees off
        North, so it is the only edge whose deviation grows with the
        threshold."""
        graph = _bearing_graph("A", (0.0, 0.0), [
            ("East", 0.0), ("North", 90.0), ("C", 100.0),
        ])

        assert find_grid_angle_deviations(graph, threshold_deg=10.0) == []
        findings = find_grid_angle_deviations(graph, threshold_deg=5.0)
        assert len(findings) == 1
        assert findings[0]["neighbour"] == "C"

    def test_nodes_argument_restricts_scope(self):
        graph = _bearing_graph("A", (0.0, 0.0), [("East", 0.0), ("Diag", 45.0)])
        graph.add_node("Z", x=100.0, y=100.0)
        graph.add_node("Z2", x=110.0, y=145.0)
        graph.add_edge(
            "Z", "Z2", edge_id="Z_Z2", action="navigate_to_pose", action_type="", properties={}, weight=1.0
        )

        findings = find_grid_angle_deviations(graph, nodes=["A"])
        assert {f["node"] for f in findings} == {"A"}


class TestComputeGridAlignmentAdjustments:
    def test_already_aligned_node_is_left_untouched(self):
        graph = _axis_aligned_graph()
        adjustments, unresolved = compute_grid_alignment_adjustments(graph, ["A"])
        assert adjustments == []
        assert unresolved == []

    def test_already_aligned_but_rotated_node_is_left_untouched(self):
        graph = _bearing_graph("A", (0.0, 0.0), [("B", 30.0), ("C", 120.0)])
        adjustments, unresolved = compute_grid_alignment_adjustments(graph, ["A"])
        assert adjustments == []
        assert unresolved == []

    def test_node_without_edges_is_skipped(self):
        graph = nx.DiGraph()
        graph.add_node("Isolated", x=0.0, y=0.0)
        adjustments, unresolved = compute_grid_alignment_adjustments(graph, ["Isolated"])
        assert adjustments == []
        assert unresolved == []

    def test_single_edge_node_is_skipped(self):
        """With only one edge there is no angle between edges to correct,
        so the node must be left untouched even if its lone edge is
        diagonal."""
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0)
        graph.add_node("B", x=10.0, y=10.0)
        graph.add_edge(
            "A", "B", edge_id="A_B", action="navigate_to_pose", action_type="", properties={}, weight=1.0
        )
        adjustments, unresolved = compute_grid_alignment_adjustments(graph, ["A"])
        assert adjustments == []
        assert unresolved == []

    def test_corner_node_with_two_perpendicular_neighbours(self):
        graph = nx.DiGraph()
        graph.add_node("Corner", x=1.0, y=1.0)
        graph.add_node("East", x=10.0, y=0.0)
        graph.add_node("North", x=0.0, y=10.0)
        graph.add_edge(
            "Corner", "East", edge_id="c_e", action="navigate_to_pose",
            action_type="", properties={}, weight=1.0,
        )
        graph.add_edge(
            "Corner", "North", edge_id="c_n", action="navigate_to_pose",
            action_type="", properties={}, weight=1.0,
        )

        adjustments, unresolved = compute_grid_alignment_adjustments(graph, ["Corner"])
        assert unresolved == []
        assert len(adjustments) == 1
        new_x, new_y = adjustments[0].new_position
        assert new_y == pytest.approx(0.0)  # aligned with East
        assert new_x == pytest.approx(0.0)  # aligned with North

    def test_corner_node_resolved_relative_to_a_rotated_local_grid(self):
        """The two neighbours are already 90 degrees apart, just not
        aligned to the global x/y axes; the node should be moved onto
        that (rotated) right angle rather than the global one."""
        graph = nx.DiGraph()
        graph.add_node("Corner", x=2.0, y=1.0)
        for name, bearing in (("N1", 30.0), ("N2", 120.0)):
            x = 10.0 * math.cos(math.radians(bearing))
            y = 10.0 * math.sin(math.radians(bearing))
            graph.add_node(name, x=x, y=y)
            graph.add_edge(
                "Corner", name, edge_id=f"c_{name}", action="navigate_to_pose",
                action_type="", properties={}, weight=1.0,
            )

        adjustments, unresolved = compute_grid_alignment_adjustments(graph, ["Corner"])
        assert unresolved == []
        assert len(adjustments) == 1
        new_x, new_y = adjustments[0].new_position
        b1 = compute_edge_bearing(graph, "Corner", "N1")
        # compute_edge_bearing reads from the (unmodified) graph, so
        # recompute the post-move bearings from the new position directly.
        bearing_n1 = math.degrees(math.atan2(
            graph.nodes["N1"]["y"] - new_y, graph.nodes["N1"]["x"] - new_x
        )) % 360.0
        bearing_n2 = math.degrees(math.atan2(
            graph.nodes["N2"]["y"] - new_y, graph.nodes["N2"]["x"] - new_x
        )) % 360.0
        assert (bearing_n2 - bearing_n1) % 360.0 == pytest.approx(90.0)
        assert adjustments[0].max_deviation_after == pytest.approx(0.0, abs=1e-6)

    def test_conflicting_neighbours_are_left_unresolved(self):
        """Three fixed neighbours where one is genuinely inconsistent with
        the other two cannot be reconciled by moving a single node."""
        graph = _bearing_graph("A", (0.0, 0.0), [
            ("East", 0.0), ("North", 90.0), ("Diag", 45.0),
        ])

        adjustments, unresolved = compute_grid_alignment_adjustments(
            graph, ["A"], threshold_deg=DEFAULT_GRID_ANGLE_THRESHOLD_DEG
        )
        assert adjustments == []
        assert unresolved == ["A"]
        # Neighbours must never be modified, even when unresolved.
        assert graph.nodes["East"]["x"] == pytest.approx(10.0)
        assert graph.nodes["East"]["y"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# SVG generation
# ---------------------------------------------------------------------------

class TestGenerateSvg:
    def test_generates_valid_svg_file(self, tmp_path):
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(MIXED_ACTIONS_MAP))

        out = tmp_path / "map.svg"
        svg = generate_svg(graph, str(out), title="test")

        assert out.is_file()
        assert svg.startswith("<svg")
        assert svg.strip().endswith("</svg>")
        # Bidirectional edges must not reference an arrow marker.
        assert 'marker-end="url(#arrow-2ca02c)"' not in svg.split("<defs>")[0]

    def test_directional_edges_get_arrow_marker(self, tmp_path):
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(SIMPLE_MAP))

        out = tmp_path / "simple.svg"
        svg = generate_svg(graph, str(out))
        assert 'marker-end="url(#arrow-' in svg

    def test_raises_on_empty_graph(self, tmp_path):
        graph = nx.DiGraph()
        with pytest.raises(ValueError):
            generate_svg(graph, str(tmp_path / "empty.svg"))

    def test_output_is_well_formed_xml(self, tmp_path):
        """The generated document must always be parseable XML."""
        import xml.etree.ElementTree as ET
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(COMPLEX_MAP))

        out = tmp_path / "complex.svg"
        svg = generate_svg(graph, str(out), title="A & B <weird> \"title\"")
        ET.fromstring(svg)  # raises ParseError if malformed

    def test_special_characters_in_names_are_escaped(self, tmp_path):
        """Node/action/edge names with XML metacharacters must not break the SVG."""
        import xml.etree.ElementTree as ET

        graph = nx.DiGraph()
        graph.add_node("N<1>", x=0.0, y=0.0, verts=[])
        graph.add_node('N&"2', x=5.0, y=0.0, verts=[])
        graph.add_edge(
            "N<1>", 'N&"2',
            edge_id="e<1>&'2\"", action="go & <stop>",
            action_type="", properties={}, weight=1.0,
        )

        out = tmp_path / "special.svg"
        svg = generate_svg(graph, str(out), title="Map <1> & \"2\"")

        # Must parse as valid XML despite the special characters.
        ET.fromstring(svg)
        # Raw metacharacters must not appear unescaped in text content.
        assert "<1>" not in svg
        assert "go & <stop>" not in svg
        assert "&lt;1&gt;" in svg

    def test_non_finite_node_coordinates_do_not_break_svg(self, tmp_path):
        """NaN/Inf coordinates must be sanitised, not leak into the markup."""
        import re
        import xml.etree.ElementTree as ET

        graph = nx.DiGraph()
        graph.add_node("A", x=float("nan"), y=float("inf"), verts=[])
        graph.add_node("B", x=1.0, y=1.0, verts=[])
        graph.add_edge(
            "A", "B", edge_id="A_B", action="navigate_to_pose",
            action_type="", properties={}, weight=1.0,
        )

        out = tmp_path / "nonfinite.svg"
        svg = generate_svg(graph, str(out))

        ET.fromstring(svg)
        # Every numeric-looking attribute value must be a finite number;
        # a loose whole-document substring search would false-positive on
        # unrelated text (e.g. font-family names), so inspect attribute
        # values directly instead.
        for value in re.findall(r'"(-?\d[\d.eE+-]*)"', svg):
            assert value.lower() not in ("nan", "inf", "-inf", "infinity", "-infinity")
            assert math.isfinite(float(value))

    def test_degenerate_width_height_do_not_break_svg(self, tmp_path):
        """A width/height smaller than 2x margin must not produce an invalid document."""
        import xml.etree.ElementTree as ET
        from topological_navigation.networkx_utils import build_graph_from_tmap
        from topological_navigation.tmap_utils import load_tmap2_file
        graph = build_graph_from_tmap(load_tmap2_file(SIMPLE_MAP))

        out = tmp_path / "tiny.svg"
        svg = generate_svg(graph, str(out), width=10, height=10, margin=40.0)

        ET.fromstring(svg)

    def test_all_nodes_at_same_position_do_not_break_svg(self, tmp_path):
        """A single point (zero span) map must still render valid SVG."""
        import xml.etree.ElementTree as ET

        graph = nx.DiGraph()
        graph.add_node("A", x=3.0, y=3.0, verts=[])
        graph.add_node("B", x=3.0, y=3.0, verts=[])

        out = tmp_path / "single_point.svg"
        svg = generate_svg(graph, str(out))

        ET.fromstring(svg)


# ---------------------------------------------------------------------------
# analyse_map / AnalysisResult
# ---------------------------------------------------------------------------

class TestAnalyseMap:
    def test_valid_map_reports_valid(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        assert isinstance(result, AnalysisResult)
        assert result.is_valid is True

    def test_map_with_orphan_is_invalid(self):
        result = analyse_map(SIMPLE_MAP)
        assert result.is_valid is False

    def test_map_with_overlap_is_invalid(self):
        result = analyse_map(POLYGON_SHAPES_MAP)
        assert result.is_valid is False

    def test_disconnected_submaps_alone_do_not_invalidate(self):
        # complex_map has disconnected submaps AND orphans (NoGoZone /
        # TopicLocaliseNode are isolated nodes), so build a graph by hand
        # to isolate the "disconnected-only" scenario.
        graph = nx.DiGraph()
        graph.add_node("A", x=0.0, y=0.0, verts=[])
        graph.add_node("B", x=0.0, y=0.0, verts=[])
        graph.add_edge("A", "B", action="navigate_to_pose", action_type="", properties={}, weight=1.0)
        graph.add_node("C", x=100.0, y=100.0, verts=[])
        graph.add_node("D", x=100.0, y=101.0, verts=[])
        graph.add_edge("C", "D", action="navigate_to_pose", action_type="", properties={}, weight=1.0)

        components = find_disconnected_components(graph)
        orphans = find_orphaned_nodes(graph)
        overlaps = find_overlapping_influence_zones(graph)

        assert len(components) == 2
        assert orphans == ["A", "C"]  # sources of each sub-map have no incoming edges... but that's expected
        # Sanity: components alone shouldn't be conflated with validity;
        # validity is computed from schema + orphans + overlaps only.
        result = AnalysisResult(
            map_file="dummy",
            schema_valid=True,
            schema_message="ok",
            orphaned_nodes=[],
            disconnected_components=components,
            statistics=compute_statistics(graph),
            overlaps=overlaps,
        )
        assert result.is_valid is True

    def test_generates_svg_when_path_given(self, tmp_path):
        out = tmp_path / "out.svg"
        result = analyse_map(MIXED_ACTIONS_MAP, svg_path=str(out))
        assert result.svg_path == str(out)
        assert out.is_file()

    def test_format_report_contains_all_sections(self):
        result = analyse_map(MIXED_ACTIONS_MAP)
        report = result.format_report()
        for section in (
            "[Schema validation]", "[Orphaned nodes]",
            "[Disconnected sub-maps]", "[Overlapping influence zones]",
            "[Grid angle deviation]", "[Statistics]",
        ):
            assert section in report


class TestAnalyseMapGridAngleDeviation:
    """COMPLEX_MAP contains genuinely diagonal (45 deg) edges around
    Entry/Row1Start/Row1End/Row2End/Junction2 (see test/fixtures/README.md);
    these are used here to exercise the (disabled-by-default) check."""

    def test_disabled_by_default(self):
        result = analyse_map(COMPLEX_MAP)
        assert result.grid_angle_deviations == []
        assert result.check_severities["grid-angle-deviation"] is None

    def test_enabled_finds_deviations(self):
        result = analyse_map(COMPLEX_MAP, check_severities={"grid-angle-deviation": "warning"})
        assert len(result.grid_angle_deviations) > 0
        assert "Row1Start" in {d["node"] for d in result.grid_angle_deviations}

    def test_error_severity_makes_map_invalid(self):
        result = analyse_map(
            COMPLEX_MAP,
            check_severities={
                "schema": None, "orphaned-node": None,
                "sub-map-separation": None, "influence-zone-overlap": None,
                "grid-angle-deviation": "error",
            },
        )
        assert result.is_valid is False

    def test_warning_severity_does_not_invalidate_map(self):
        result = analyse_map(
            COMPLEX_MAP,
            check_severities={
                "schema": None, "orphaned-node": None,
                "sub-map-separation": None, "influence-zone-overlap": None,
                "grid-angle-deviation": "warning",
            },
        )
        assert len(result.grid_angle_deviations) > 0
        assert result.is_valid is True

    def test_filter_restricts_selected_nodes(self):
        result = analyse_map(
            COMPLEX_MAP,
            check_severities={"grid-angle-deviation": "warning"},
            grid_angle_filters=["name:Row1*"],
        )
        nodes = {d["node"] for d in result.grid_angle_deviations}
        assert nodes and nodes <= {"Row1Start", "Row1End"}

    def test_exclude_filter_removes_matches(self):
        result = analyse_map(
            COMPLEX_MAP,
            check_severities={"grid-angle-deviation": "warning"},
            grid_angle_exclude_filters=["name:Row1Start"],
        )
        assert "Row1Start" not in {d["node"] for d in result.grid_angle_deviations}

    def test_looser_threshold_clears_deviations(self):
        result = analyse_map(
            COMPLEX_MAP,
            check_severities={"grid-angle-deviation": "warning"},
            grid_angle_threshold_deg=60.0,
        )
        assert result.grid_angle_deviations == []

    def test_format_report_shows_skipped_when_disabled(self):
        result = analyse_map(COMPLEX_MAP)
        report = result.format_report()
        section = report.split("[Grid angle deviation]")[1].split("[Statistics]")[0]
        assert "SKIPPED" in section


# ---------------------------------------------------------------------------
# Minification
# ---------------------------------------------------------------------------

class TestMinifyMap:
    def test_default_anchors_produce_smaller_valid_roundtrip(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(COMPLEX_MAP, output_file=str(out))

        assert isinstance(result, MinifyResult)
        assert out.is_file()
        assert result.minified_size <= result.original_size
        assert result.schema_valid is True

        with open(out, encoding="utf-8") as fh:
            reparsed = yaml.load(fh, Loader=CustomSafeLoader)
        reparsed_body = {k: v for k, v in reparsed.items() if k != DEFAULT_ANCHORS_KEY}
        assert reparsed_body == load_tmap2_file(COMPLEX_MAP)

    def test_low_thresholds_create_anchors(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(
            COMPLEX_MAP, output_file=str(out), min_size=1, min_occurrences=2
        )
        assert result.anchors_created > 0
        assert result.occurrences_collapsed >= result.anchors_created * 2

        with open(out, encoding="utf-8") as fh:
            text = fh.read()
        assert f"{DEFAULT_ANCHORS_KEY}:" in text
        assert "&" in text  # at least one anchor definition
        assert "*" in text  # at least one alias reference

    def test_no_anchors_disables_anchor_section(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(
            COMPLEX_MAP, output_file=str(out), anchors=False, min_size=1, min_occurrences=2
        )
        assert result.anchors_created == 0
        assert result.occurrences_collapsed == 0

        with open(out, encoding="utf-8") as fh:
            text = fh.read()
        assert f"{DEFAULT_ANCHORS_KEY}:" not in text

    def test_strip_comments_removes_leading_banner(self, tmp_path):
        out_kept = tmp_path / "kept.min.yaml"
        out_stripped = tmp_path / "stripped.min.yaml"
        minify_map(COMPLEX_MAP, output_file=str(out_kept), strip_comments=False)
        minify_map(COMPLEX_MAP, output_file=str(out_stripped), strip_comments=True)

        with open(COMPLEX_MAP, encoding="utf-8") as fh:
            original_first_line = fh.readline()

        if original_first_line.lstrip().startswith("#"):
            assert out_kept.read_text().startswith("#")
            assert not out_stripped.read_text().startswith("#")

    def test_strip_unreachable_removes_isolated_nodes_and_stays_valid(self, tmp_path):
        graph_before = load_tmap2_file(COMPLEX_MAP)
        node_names = [n["node"]["name"] for n in graph_before["nodes"]]
        start = node_names[0]

        out = tmp_path / "out.min.yaml"
        result = minify_map(COMPLEX_MAP, output_file=str(out), strip_unreachable_from=start)

        assert result.stripped_nodes >= 0
        analysis = analyse_map(str(out))
        assert analysis.orphaned_nodes == [] or analysis.orphaned_nodes == [start]
        assert len(analysis.disconnected_components) <= 1

    def test_strip_unreachable_raises_for_unknown_node(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        with pytest.raises(ValueError):
            minify_map(COMPLEX_MAP, output_file=str(out), strip_unreachable_from="NoSuchNode")

    def test_derives_output_path_when_not_given(self, tmp_path):
        src = tmp_path / "my_map.tmap2.yaml"
        src.write_text(open(COMPLEX_MAP, encoding="utf-8").read())
        result = minify_map(str(src))
        expected = tmp_path / "my_map.min.tmap2.yaml"
        assert result.output_file == str(expected)
        assert expected.is_file()

    def test_format_report_contains_key_figures(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(COMPLEX_MAP, output_file=str(out), min_size=1, min_occurrences=2)
        report = result.format_report()
        assert "Original size" in report
        assert "Minified size" in report
        assert "Reduction" in report
        assert "Anchors created" in report

    def test_minifying_an_already_minified_file_is_idempotent(self, tmp_path):
        pass1 = tmp_path / "pass1.min.yaml"
        pass2 = tmp_path / "pass2.min.yaml"
        minify_map(COMPLEX_MAP, output_file=str(pass1), min_size=1, min_occurrences=2)
        assert f"{DEFAULT_ANCHORS_KEY}:" in pass1.read_text()

        result2 = minify_map(str(pass1), output_file=str(pass2), min_size=1, min_occurrences=2)
        assert result2.schema_valid is True
        assert pass2.read_text() == pass1.read_text()

    def test_custom_anchors_key_is_used_instead_of_default(self, tmp_path):
        out = tmp_path / "out.min.yaml"
        result = minify_map(
            COMPLEX_MAP, output_file=str(out), anchors_key="custom_key", min_size=1, min_occurrences=2
        )
        assert result.anchors_created > 0

        text = out.read_text()
        assert "custom_key:" in text
        assert f"{DEFAULT_ANCHORS_KEY}:" not in text

        with open(out, encoding="utf-8") as fh:
            reparsed = yaml.load(fh, Loader=CustomSafeLoader)
        reparsed_body = {k: v for k, v in reparsed.items() if k != "custom_key"}
        assert reparsed_body == load_tmap2_file(COMPLEX_MAP)


# ---------------------------------------------------------------------------
# Grid alignment
# ---------------------------------------------------------------------------

class TestGridAlignMap:
    def test_writes_output_and_reports_schema_validity(self, tmp_path):
        out = tmp_path / "aligned.yaml"
        result = grid_align_map(COMPLEX_MAP, output_file=str(out), filters=["name:Row1*"])

        assert isinstance(result, GridAlignResult)
        assert out.is_file()
        assert result.schema_valid is True

    def test_derives_output_path_when_not_given(self, tmp_path):
        src = tmp_path / "my_map.tmap2.yaml"
        src.write_text(open(COMPLEX_MAP, encoding="utf-8").read())
        result = grid_align_map(str(src))
        expected = tmp_path / "my_map.gridalign.tmap2.yaml"
        assert result.output_file == str(expected)
        assert expected.is_file()

    def test_unresolved_nodes_are_listed_and_left_unmoved(self, tmp_path):
        out = tmp_path / "aligned.yaml"
        original = load_tmap2_file(COMPLEX_MAP)
        original_position = next(
            n["node"]["pose"]["position"] for n in original["nodes"] if n["node"]["name"] == "Row1Start"
        )

        result = grid_align_map(COMPLEX_MAP, output_file=str(out), filters=["name:Row1Start"])

        assert "Row1Start" in result.unresolved_nodes
        assert result.adjustments == []

        aligned = load_tmap2_file(str(out))
        aligned_position = next(
            n["node"]["pose"]["position"] for n in aligned["nodes"] if n["node"]["name"] == "Row1Start"
        )
        assert aligned_position == original_position

    def test_format_report_contains_key_sections(self, tmp_path):
        out = tmp_path / "aligned.yaml"
        result = grid_align_map(COMPLEX_MAP, output_file=str(out), filters=["name:Row1*"])
        report = result.format_report()
        assert "Angle threshold" in report
        assert "Nodes adjusted" in report
        assert "Unresolved nodes" in report

    def test_raises_for_map_with_no_nodes_field(self, tmp_path):
        out = tmp_path / "aligned.yaml"
        empty = tmp_path / "empty.yaml"
        empty.write_text("meta: {}\n")
        with pytest.raises(ValueError):
            grid_align_map(str(empty), output_file=str(out))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

class TestCli:
    def test_check_exits_zero_for_valid_map(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["check", MIXED_ACTIONS_MAP])
        assert exc_info.value.code == 0

    def test_check_exits_one_for_invalid_map(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["check", SIMPLE_MAP])
        assert exc_info.value.code == 1

    def test_check_exits_two_for_missing_file(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["check", "/no/such/file.yaml"])
        assert exc_info.value.code == 2

    def test_svg_command_writes_file(self, tmp_path, capsys):
        out = tmp_path / "cli_out.svg"
        main(["svg", MIXED_ACTIONS_MAP, "-o", str(out)])
        assert out.is_file()

    def test_analyse_command_prints_report(self, capsys):
        main(["analyse", MIXED_ACTIONS_MAP])
        captured = capsys.readouterr()
        assert "Map analysis report" in captured.out

    def test_build_arg_parser_requires_subcommand(self):
        parser = build_arg_parser()
        with pytest.raises(SystemExit):
            parser.parse_args([])

    def test_minify_command_writes_file_and_prints_report(self, tmp_path, capsys):
        out = tmp_path / "cli_out.min.yaml"
        main(["minify", COMPLEX_MAP, "-o", str(out)])
        assert out.is_file()
        captured = capsys.readouterr()
        assert "Minify report" in captured.out

    def test_minify_command_exits_two_for_missing_file(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["minify", "/no/such/file.yaml"])
        assert exc_info.value.code == 2

    def test_minify_command_exits_two_for_bad_strip_unreachable(self, tmp_path, capsys):
        out = tmp_path / "cli_out.min.yaml"
        with pytest.raises(SystemExit) as exc_info:
            main(["minify", COMPLEX_MAP, "-o", str(out), "--strip-unreachable", "NoSuchNode"])
        assert exc_info.value.code == 2

    def test_check_grid_angle_deviation_disabled_by_default(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main([
                "check", COMPLEX_MAP,
                "--orphaned-node=false", "--sub-map-separation=false",
                "--influence-zone-overlap=false",
            ])
        assert exc_info.value.code == 0
        captured = capsys.readouterr()
        assert "SKIPPED" in captured.out

    def test_check_grid_angle_deviation_error_severity_fails(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main([
                "check", COMPLEX_MAP, "--grid-angle-deviation=error",
                "--orphaned-node=false", "--sub-map-separation=false",
                "--influence-zone-overlap=false",
            ])
        assert exc_info.value.code == 1
        captured = capsys.readouterr()
        assert "Grid angle deviation" in captured.out

    def test_check_grid_angle_deviation_warning_passes(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main([
                "check", COMPLEX_MAP, "--grid-angle-deviation=warning",
                "--orphaned-node=false", "--sub-map-separation=false",
                "--influence-zone-overlap=false",
            ])
        assert exc_info.value.code == 0

    def test_check_grid_angle_filter_restricts_failures(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main([
                "check", COMPLEX_MAP, "--grid-angle-deviation=error",
                "--orphaned-node=false", "--sub-map-separation=false",
                "--influence-zone-overlap=false",
                "--grid-angle-filter", "name:NoSuchNode*",
            ])
        assert exc_info.value.code == 0

    def test_grid_align_command_writes_file_and_prints_report(self, tmp_path, capsys):
        out = tmp_path / "aligned.yaml"
        main([
            "grid-align", COMPLEX_MAP, "-o", str(out),
            "--grid-angle-filter", "name:Exit",
        ])
        assert out.is_file()
        captured = capsys.readouterr()
        assert "Grid-align report" in captured.out

    def test_grid_align_command_exits_one_when_unresolved(self, tmp_path, capsys):
        out = tmp_path / "aligned.yaml"
        with pytest.raises(SystemExit) as exc_info:
            main([
                "grid-align", COMPLEX_MAP, "-o", str(out),
                "--grid-angle-filter", "name:Row1Start",
            ])
        assert exc_info.value.code == 1
        captured = capsys.readouterr()
        assert "Row1Start" in captured.out

    def test_grid_align_command_exits_two_for_missing_file(self, capsys):
        with pytest.raises(SystemExit) as exc_info:
            main(["grid-align", "/no/such/file.yaml"])
        assert exc_info.value.code == 2
