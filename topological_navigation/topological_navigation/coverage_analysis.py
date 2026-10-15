"""Route sign-off coverage analysis for topological maps.

This module combines a (possibly merged, multi-map) topological graph with
recorded traversal statistics from :class:`~topological_navigation.nav_stats_db.NavStatsDB`
to compute *coverage*: how many of the selected edges have been
successfully traversed at least once, and how many times.

This supports a quality-control / sign-off workflow: a site (a dedicated
map, or a set of maps considered together) can be marked as adequately
tested once a sufficient fraction of its edges have recorded successful
traversals.

Node selection (which nodes -- and therefore which incident edges -- are
considered for the report) uses the same filter DSL as
:mod:`topological_navigation.map_analyser`'s grid-angle-deviation check:
``name:<glob>``, ``tag:<glob>`` or ``property:<key>[=<value>]``.

No ROS 2 dependency is required; only ``networkx`` (via
:mod:`topological_navigation.networkx_utils`) and the filter helpers from
:mod:`topological_navigation.map_analyser`.
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
from xml.sax.saxutils import escape as _xml_escape

try:
    import networkx as nx
except ImportError:  # pragma: no cover - exercised only when nx missing
    nx = None

from topological_navigation.map_analyser import select_nodes_by_filters
from topological_navigation.networkx_utils import build_graph_from_tmap
from topological_navigation.nav_stats_db import NavStatsDB


# ---------------------------------------------------------------------------
# Graph merging
# ---------------------------------------------------------------------------

def merge_graphs(graphs: List["nx.DiGraph"]) -> "nx.DiGraph":
    """Merge multiple topological map graphs into one.

    Nodes and edges are matched by name/``edge_id``: when the same node (or
    edge) appears in more than one graph, later graphs' attributes are
    merged on top of (without discarding) earlier ones, and the node/edge
    is only represented once in the merged graph.
    """
    merged = nx.DiGraph()
    for graph in graphs:
        for node, data in graph.nodes(data=True):
            if node in merged:
                merged.nodes[node].update(data)
            else:
                merged.add_node(node, **data)
        for u, v, data in graph.edges(data=True):
            if merged.has_edge(u, v):
                merged.edges[u, v].update(data)
            else:
                merged.add_edge(u, v, **data)
    return merged


def build_merged_graph_for_maps(db: NavStatsDB, map_ids: List[str]) -> "nx.DiGraph":
    """Load and merge the graphs for the given map identifiers (name or hash)."""
    graphs = []
    for map_id in map_ids:
        m = db.get_map(map_id)
        if m is None:
            raise ValueError("Map '%s' not found in database." % map_id)
        import yaml
        tmap = yaml.safe_load(m["map_data"]) or {}
        graph = build_graph_from_tmap(tmap)
        if graph is None:
            raise ValueError("Map '%s' could not be parsed into a graph." % map_id)
        graphs.append(graph)
    if not graphs:
        raise ValueError("No maps selected for coverage analysis.")
    return merge_graphs(graphs)


def resolve_map_hashes(db: NavStatsDB, map_ids: List[str]) -> List[str]:
    """Return the list of ``map_hash`` values corresponding to *map_ids*."""
    hashes = []
    for map_id in map_ids:
        m = db.get_map(map_id)
        if m is None:
            raise ValueError("Map '%s' not found in database." % map_id)
        hashes.append(m["map_hash"])
    return hashes


# ---------------------------------------------------------------------------
# Coverage computation
# ---------------------------------------------------------------------------

#: Default minimum number of recorded successful traversals an edge needs
#: before it is considered "covered" (signed off). Configurable per-report
#: via ``compute_coverage(..., min_success=...)`` / the CLI's
#: ``--min-success`` option.
DEFAULT_MIN_SUCCESS_TRAVERSALS = 2


@dataclass
class EdgeCoverage:
    """Coverage information for a single edge."""

    edge_id: str
    origin: str
    target: str
    action: str
    selected: bool
    total: int = 0
    success: int = 0
    failed: int = 0
    aborted: int = 0
    min_success: int = DEFAULT_MIN_SUCCESS_TRAVERSALS

    @property
    def covered(self) -> bool:
        """True if the edge has at least :attr:`min_success` recorded
        successful traversals (the configurable sign-off threshold)."""
        return self.success >= self.min_success


@dataclass
class CoverageReport:
    """Full coverage report for a set of maps/nodes."""

    edges: List[EdgeCoverage] = field(default_factory=list)
    tags: Dict[str, List[str]] = field(default_factory=dict)  # tag -> selected edge_ids
    min_success: int = DEFAULT_MIN_SUCCESS_TRAVERSALS

    @property
    def selected_edges(self) -> List[EdgeCoverage]:
        """Return only the edges selected by the node filters."""
        return [e for e in self.edges if e.selected]

    def summary(self) -> Dict[str, Any]:
        """Return overall coverage counters for the selected edges."""
        selected = self.selected_edges
        total_edges = len(selected)
        covered_edges = sum(1 for e in selected if e.covered)
        total_traversals = sum(e.total for e in selected)
        total_success = sum(e.success for e in selected)
        total_failed = sum(e.failed for e in selected)
        total_aborted = sum(e.aborted for e in selected)
        coverage_pct = round(100.0 * covered_edges / total_edges, 2) if total_edges else 0.0
        return {
            "total_edges": total_edges,
            "covered_edges": covered_edges,
            "uncovered_edges": total_edges - covered_edges,
            "coverage_pct": coverage_pct,
            "total_traversals": total_traversals,
            "total_success": total_success,
            "total_failed": total_failed,
            "total_aborted": total_aborted,
        }

    def tag_summary(self) -> Dict[str, Dict[str, Any]]:
        """Return per-tag coverage counters, keyed by tag value."""
        by_edge_id = {e.edge_id: e for e in self.edges}
        result = {}
        for tag, edge_ids in sorted(self.tags.items()):
            edges = [by_edge_id[eid] for eid in edge_ids if eid in by_edge_id]
            total_edges = len(edges)
            covered_edges = sum(1 for e in edges if e.covered)
            coverage_pct = (
                round(100.0 * covered_edges / total_edges, 2) if total_edges else 0.0
            )
            result[tag] = {
                "total_edges": total_edges,
                "covered_edges": covered_edges,
                "uncovered_edges": total_edges - covered_edges,
                "coverage_pct": coverage_pct,
                "total_traversals": sum(e.total for e in edges),
            }
        return result


def _node_tags(graph: "nx.DiGraph", node: str) -> List[str]:
    return list((graph.nodes[node].get("meta") or {}).get("tag") or [])


def compute_coverage(
    graph: "nx.DiGraph",
    db: NavStatsDB,
    map_hashes: List[str],
    filters: Optional[List[str]] = None,
    exclude_filters: Optional[List[str]] = None,
    where: Optional[str] = None,
    min_success: int = DEFAULT_MIN_SUCCESS_TRAVERSALS,
) -> CoverageReport:
    """Compute a :class:`CoverageReport` for *graph*.

    Parameters
    ----------
    graph:
        The (possibly merged) topological map graph.
    db:
        Database to read traversal statistics from.
    map_hashes:
        ``map_hash`` values to consider when aggregating traversal stats.
    filters / exclude_filters:
        Node filter DSL specs (``name:<glob>``, ``tag:<glob>`` or
        ``property:<key>[=<value>]``) used to select which nodes (and
        their incident edges) are counted for the report.  All edges are
        still returned (for SVG rendering) but marked ``selected=False``
        when neither endpoint is selected.
    where:
        Additional raw SQL WHERE expression applied to the traversals
        table (e.g. a time-window filter).
    min_success:
        Minimum number of recorded successful traversals an edge needs
        before it is considered "covered"/signed off (default
        :data:`DEFAULT_MIN_SUCCESS_TRAVERSALS`). Must be a positive
        integer.
    """
    if min_success < 1:
        raise ValueError("min_success must be a positive integer (got %r)" % (min_success,))

    selected_nodes = select_nodes_by_filters(graph, filters, exclude_filters)

    if not map_hashes:
        hash_clause = "1=0"
    else:
        placeholders = ", ".join("'%s'" % h.replace("'", "''") for h in map_hashes)
        hash_clause = "map_hash IN (%s)" % placeholders

    sql_where = hash_clause
    if where:
        sql_where += " AND (" + where + ")"

    stats_rows = db.query(
        """
        SELECT
            edge_id,
            COUNT(*)                                            AS total,
            SUM(CASE WHEN status='success' THEN 1 ELSE 0 END)  AS success,
            SUM(CASE WHEN status='failed'  THEN 1 ELSE 0 END)  AS failed,
            SUM(CASE WHEN status='aborted' THEN 1 ELSE 0 END)  AS aborted
        FROM traversals
        WHERE {where}
        GROUP BY edge_id
        """.format(where=sql_where)
    )
    stats_by_edge = {r["edge_id"]: r for r in stats_rows}

    report = CoverageReport(min_success=min_success)
    tag_edges: Dict[str, Set[str]] = {}

    for u, v, data in graph.edges(data=True):
        edge_id = data.get("edge_id") or ("%s_%s" % (u, v))
        selected = (u in selected_nodes) or (v in selected_nodes)
        s = stats_by_edge.get(edge_id, {})
        ec = EdgeCoverage(
            edge_id=edge_id,
            origin=u,
            target=v,
            action=data.get("action", "unknown"),
            selected=selected,
            total=int(s.get("total") or 0),
            success=int(s.get("success") or 0),
            failed=int(s.get("failed") or 0),
            aborted=int(s.get("aborted") or 0),
            min_success=min_success,
        )
        report.edges.append(ec)

        if selected:
            for node in (u, v):
                if node not in selected_nodes:
                    continue
                for tag in _node_tags(graph, node):
                    tag_edges.setdefault(tag, set()).add(edge_id)

    report.tags = {tag: sorted(eids) for tag, eids in tag_edges.items()}
    return report


# ---------------------------------------------------------------------------
# Markdown report
# ---------------------------------------------------------------------------

def render_markdown_summary(report: CoverageReport, title: str = "Coverage Summary") -> str:
    """Render a brief Markdown summary of *report*."""
    s = report.summary()
    lines = [
        "# %s" % title,
        "",
        "Sign-off threshold: an edge is considered covered once it has "
        "at least %d recorded successful traversal(s)." % report.min_success,
        "",
        "| Metric | Value |",
        "| --- | --- |",
        "| Selected edges | %d |" % s["total_edges"],
        "| Covered edges | %d |" % s["covered_edges"],
        "| Uncovered edges | %d |" % s["uncovered_edges"],
        "| Coverage | %.2f%% |" % s["coverage_pct"],
        "| Total traversals | %d |" % s["total_traversals"],
        "| Successful | %d |" % s["total_success"],
        "| Failed | %d |" % s["total_failed"],
        "| Aborted | %d |" % s["total_aborted"],
    ]
    tag_summary = report.tag_summary()
    if tag_summary:
        lines += ["", "## Coverage by tag", "", "| Tag | Edges | Covered | Coverage |",
                  "| --- | --- | --- | --- |"]
        for tag, t in tag_summary.items():
            lines.append(
                "| %s | %d | %d | %.2f%% |"
                % (tag, t["total_edges"], t["covered_edges"], t["coverage_pct"])
            )
    return "\n".join(lines) + "\n"


def render_markdown_report(
    report: CoverageReport,
    title: str = "Route Coverage Sign-off Report",
    map_names: Optional[List[str]] = None,
) -> str:
    """Render a comprehensive Markdown report suitable as sign-off evidence."""
    s = report.summary()
    lines = ["# %s" % title, ""]
    if map_names:
        lines += ["**Maps considered:** %s" % ", ".join(map_names), ""]
    lines += [
        "**Sign-off threshold:** an edge is considered covered once it has "
        "at least %d recorded successful traversal(s)." % report.min_success,
        "",
    ]

    lines += [
        "## Overall Coverage",
        "",
        "| Metric | Value |",
        "| --- | --- |",
        "| Selected edges | %d |" % s["total_edges"],
        "| Covered edges (>=%d success) | %d |" % (report.min_success, s["covered_edges"]),
        "| Uncovered edges | %d |" % s["uncovered_edges"],
        "| Coverage | %.2f%% |" % s["coverage_pct"],
        "| Total traversals | %d |" % s["total_traversals"],
        "| Successful traversals | %d |" % s["total_success"],
        "| Failed traversals | %d |" % s["total_failed"],
        "| Aborted traversals | %d |" % s["total_aborted"],
        "",
    ]

    tag_summary = report.tag_summary()
    if tag_summary:
        lines += [
            "## Coverage by Tag", "",
            "| Tag | Edges | Covered | Uncovered | Coverage | Traversals |",
            "| --- | --- | --- | --- | --- | --- |",
        ]
        for tag, t in tag_summary.items():
            lines.append(
                "| %s | %d | %d | %d | %.2f%% | %d |"
                % (tag, t["total_edges"], t["covered_edges"], t["uncovered_edges"],
                   t["coverage_pct"], t["total_traversals"])
            )
        lines.append("")

    lines += [
        "## Per-Edge Detail", "",
        "| Edge ID | Origin | Target | Action | Traversals | Success | Failed | Aborted | Covered |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for e in sorted(report.selected_edges, key=lambda e: e.edge_id):
        lines.append(
            "| %s | %s | %s | %s | %d | %d | %d | %d | %s |"
            % (e.edge_id, e.origin, e.target, e.action, e.total, e.success,
               e.failed, e.aborted, "yes" if e.covered else "no")
        )
    lines.append("")

    uncovered = [e for e in report.selected_edges if not e.covered]
    if uncovered:
        lines += ["## Uncovered Edges (require testing before sign-off)", ""]
        for e in sorted(uncovered, key=lambda e: e.edge_id):
            lines.append("- `%s` (%s -> %s, action: %s)" % (e.edge_id, e.origin, e.target, e.action))
        lines.append("")

    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# SVG rendering
# ---------------------------------------------------------------------------

_UNSELECTED_COLOUR = "#cccccc"
_UNCOVERED_COLOUR = "#d62728"   # red - selected, never successfully traversed
_LOW_COVERAGE_COLOUR = "#ff9900"  # amber - selected, some but not enough successes yet
_COVERED_COLOUR = "#2ca02c"    # green - selected, signed off (>= min_success successes)


def _svg_text(value: Any) -> str:
    return _xml_escape("" if value is None else str(value))


def _finite_or(value: Any, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(number) or math.isinf(number):
        return default
    return number


def _edge_colour(ec: EdgeCoverage) -> str:
    if not ec.selected:
        return _UNSELECTED_COLOUR
    if ec.success == 0:
        return _UNCOVERED_COLOUR
    if ec.success < ec.min_success:
        return _LOW_COVERAGE_COLOUR
    return _COVERED_COLOUR


def generate_coverage_svg(
    graph: "nx.DiGraph",
    report: CoverageReport,
    output_path: str,
    title: Optional[str] = None,
    width: int = 1200,
    height: int = 900,
    margin: float = 40.0,
) -> str:
    """Render *graph* as an SVG, colour-coded by coverage.

    Selected edges are coloured red (never successfully traversed), amber
    (fewer than *report*'s ``min_success`` recorded successes) or green
    (signed off: at least ``min_success`` successes). Unselected
    edges/nodes are drawn in faint grey.
    """
    node_names = list(graph.nodes())
    if not node_names:
        raise ValueError("Cannot render SVG for an empty map")

    selected_nodes = {e.origin for e in report.edges if e.selected} | {
        e.target for e in report.edges if e.selected
    }

    xs = [_finite_or(graph.nodes[n].get("x"), 0.0) for n in node_names]
    ys = [_finite_or(graph.nodes[n].get("y"), 0.0) for n in node_names]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    width = max(int(_finite_or(width, 1200)), 1)
    height = max(int(_finite_or(height, 900)), 1)
    margin = max(_finite_or(margin, 40.0), 0.0)
    margin = min(margin, (width - 1) / 2.0, (height - 1) / 2.0)
    margin = max(margin, 0.0)

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    drawable_w = width - 2 * margin
    drawable_h = height - 2 * margin
    scale = min(drawable_w / span_x, drawable_h / span_y)

    positions = dict(zip(node_names, zip(xs, ys)))

    def to_px(point):
        px = margin + (point[0] - min_x) * scale
        py = margin + (max_y - point[1]) * scale
        return px, py

    edge_px_lengths = [
        math.hypot(x2 - x1, y2 - y1)
        for u, v in graph.edges()
        for (x1, y1), (x2, y2) in [(to_px(positions[u]), to_px(positions[v]))]
        if u != v
    ]
    if edge_px_lengths:
        char_length = sum(edge_px_lengths) / len(edge_px_lengths)
    else:
        char_length = min(drawable_w, drawable_h) / max(math.sqrt(len(node_names)), 1.0)

    node_radius = max(1.0, min(char_length * 0.06, 6.0))
    edge_stroke_width = max(0.5, min(char_length * 0.04, 5.0))
    font_size = max(3.0, min(char_length * 0.09, 14.0))

    svg_parts: List[str] = []
    svg_parts.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">'
    )
    svg_parts.append('<rect width="100%" height="100%" fill="white"/>')

    if title:
        svg_parts.append(
            f'<text x="{width / 2:.1f}" y="20" text-anchor="middle" '
            f'font-size="16" font-family="sans-serif" font-weight="bold">'
            f'{_svg_text(title)}</text>'
        )

    edge_by_uv: Dict[Tuple[str, str], EdgeCoverage] = {(e.origin, e.target): e for e in report.edges}

    drawn: Set[Tuple[str, str]] = set()
    for u, v in graph.edges():
        if (v, u) in drawn:
            continue
        drawn.add((u, v))
        ec = edge_by_uv.get((u, v))
        colour = _edge_colour(ec) if ec is not None else _UNSELECTED_COLOUR

        x1, y1 = to_px(positions[u])
        x2, y2 = to_px(positions[v])
        edge_id = ec.edge_id if ec is not None else "%s_%s" % (u, v)
        traversal_info = ""
        if ec is not None and ec.selected:
            traversal_info = " - %d success / %d total" % (ec.success, ec.total)

        svg_parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{colour}" stroke-width="{edge_stroke_width:.2f}">'
            f'<title>{_svg_text(edge_id)}{_svg_text(traversal_info)}</title>'
            f'</line>'
        )

    for node_name in node_names:
        px, py = to_px(positions[node_name])
        selected = node_name in selected_nodes
        colour = "steelblue" if selected else _UNSELECTED_COLOUR
        svg_parts.append(
            f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{node_radius:.2f}" fill="{colour}" '
            f'stroke="black" stroke-width="0"><title>{_svg_text(node_name)}</title></circle>'
        )
        svg_parts.append(
            f'<text x="{px + node_radius + 1:.2f}" y="{py + font_size * 0.35:.2f}" '
            f'font-size="{font_size:.2f}" font-family="sans-serif" '
            f'fill="{"black" if selected else "#999999"}">{_svg_text(node_name)}</text>'
        )

    # --- legend --------------------------------------------------------
    legend = [
        (_COVERED_COLOUR, "Covered (>= %d successes)" % report.min_success),
        (_LOW_COVERAGE_COLOUR, "Low coverage (< %d successes)" % report.min_success),
        (_UNCOVERED_COLOUR, "Uncovered (0 successes)"),
        (_UNSELECTED_COLOUR, "Not selected"),
    ]
    legend_y = height - 20 * len(legend) - 10
    for i, (colour, label) in enumerate(legend):
        ly = legend_y + i * 20
        svg_parts.append(
            f'<line x1="{width - 260}" y1="{ly}" x2="{width - 230}" y2="{ly}" '
            f'stroke="{colour}" stroke-width="4"/>'
        )
        svg_parts.append(
            f'<text x="{width - 225}" y="{ly + 4:.1f}" font-size="11" '
            f'font-family="sans-serif">{_svg_text(label)}</text>'
        )

    svg_parts.append('</svg>')
    svg_doc = "\n".join(svg_parts)

    with open(output_path, "w", encoding="utf-8") as fh:
        fh.write(svg_doc)

    return svg_doc
