#!/usr/bin/env python3
"""Pure-Python helpers for the topological map visualiser.

These functions contain **no ROS dependencies** so they can be unit-tested
without a ROS 2 installation and reused by the visualiser node.  They
encapsulate the logic that makes the visualiser scale to large maps:

- :func:`collect_node_positions` / :func:`build_position_lookup` extract
  node coordinates once so markers can be batched.
- :func:`group_edge_segments` collapses every edge into a small number of
  line-segment batches keyed by action, so a map with thousands of edges
  produces only a handful of RViz markers.
- :func:`collect_zone_segments` batches all influence-zone polygons into a
  single line-segment list.
- :func:`compute_auto_scale` estimates a sensible marker scale from the
  spatial spread of the nodes, which keeps markers legible on both small
  and very large maps.

A *point* throughout this module is a plain ``(x, y, z)`` tuple of floats and
a *segment* is a ``(point_from, point_to)`` tuple.  The visualiser converts
these to ``geometry_msgs/Point`` instances when it builds markers.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

Point3 = Tuple[float, float, float]
Segment = Tuple[Point3, Point3]

#: Hard limits applied to any computed/!user marker scale so the
#: visualisation never collapses to nothing or explodes off-screen.
MIN_SCALE = 0.05
MAX_SCALE = 25.0


def _clamp(value: float, low: float, high: float) -> float:
    """Clamp *value* into the inclusive ``[low, high]`` range."""
    return max(low, min(high, value))


def _node_position(node: dict) -> Optional[Point3]:
    """Return the ``(x, y, z)`` position of a tmap2 *node* dict.

    Returns ``None`` if the node has no usable position.
    """
    try:
        pos = node['pose']['position']
        return (float(pos['x']), float(pos['y']), float(pos.get('z', 0.0)))
    except (KeyError, TypeError, ValueError):
        return None


def collect_node_positions(nodes: Sequence[dict]) -> List[Point3]:
    """Extract ``(x, y, z)`` positions from a list of tmap2 node entries.

    Parameters
    ----------
    nodes:
        The ``tmap['nodes']`` list, where each entry is a dict with a
        ``'node'`` key.

    Returns
    -------
    list of (float, float, float)
        Positions of every node with a valid pose, in map order.
    """
    positions: List[Point3] = []
    for entry in nodes:
        node = entry.get('node', {})
        pos = _node_position(node)
        if pos is not None:
            positions.append(pos)
    return positions


def build_position_lookup(nodes: Sequence[dict]) -> Dict[str, Point3]:
    """Build a ``{node_name: (x, y, z)}`` lookup from tmap2 node entries."""
    lookup: Dict[str, Point3] = {}
    for entry in nodes:
        node = entry.get('node', {})
        name = node.get('name')
        if name is None:
            continue
        pos = _node_position(node)
        if pos is not None:
            lookup[name] = pos
    return lookup


def group_edge_segments(
    nodes: Sequence[dict],
    z_offset: float = 0.0,
) -> Dict[str, List[Segment]]:
    """Group every edge into line segments keyed by action name.

    Batching edges by action lets the visualiser emit a single
    ``LINE_LIST`` marker per action (and thus per colour) instead of one
    marker per edge.  For a map with *E* edges and *A* distinct actions
    this reduces the edge marker count from *E* to *A*.

    Parameters
    ----------
    nodes:
        The ``tmap['nodes']`` list.
    z_offset:
        Value added to the ``z`` coordinate of every endpoint so edges
        render slightly above the ground plane.

    Returns
    -------
    dict[str, list[Segment]]
        Mapping of action name to a list of ``(from, to)`` segments.
        Edges whose target node is missing from the map are skipped.
    """
    lookup = build_position_lookup(nodes)
    groups: Dict[str, List[Segment]] = {}

    for entry in nodes:
        node = entry.get('node', {})
        name = node.get('name')
        if name not in lookup:
            continue
        src = lookup[name]
        src = (src[0], src[1], src[2] + z_offset)
        for edge in node.get('edges', []) or []:
            target = edge.get('node')
            if target not in lookup:
                continue
            dst = lookup[target]
            dst = (dst[0], dst[1], dst[2] + z_offset)
            action = edge.get('action', '') or ''
            groups.setdefault(action, []).append((src, dst))

    return groups


def collect_zone_segments(
    nodes: Sequence[dict],
    z_offset: float = 0.0,
) -> List[Segment]:
    """Batch every influence-zone polygon into a flat list of segments.

    The returned segments are suitable for a single ``LINE_LIST`` marker
    that draws all node influence zones at once.  Vertices are stored in
    node-local coordinates in the map, so they are translated by the
    node position here.

    Parameters
    ----------
    nodes:
        The ``tmap['nodes']`` list.
    z_offset:
        Value added to the ``z`` coordinate of every vertex.

    Returns
    -------
    list[Segment]
        One ``(from, to)`` segment per polygon edge (the polygon is
        closed automatically).  Polygons with fewer than two vertices are
        skipped.
    """
    segments: List[Segment] = []

    for entry in nodes:
        node = entry.get('node', {})
        verts = node.get('verts') or []
        if len(verts) < 2:
            continue
        base = _node_position(node)
        if base is None:
            continue
        bx, by, bz = base
        bz += z_offset

        points: List[Point3] = []
        valid = True
        for v in verts:
            try:
                points.append((bx + float(v['x']), by + float(v['y']), bz))
            except (KeyError, TypeError, ValueError):
                valid = False
                break
        if not valid or len(points) < 2:
            continue

        for i in range(len(points)):
            segments.append((points[i], points[(i + 1) % len(points)]))

    return segments


def compute_auto_scale(
    positions: Sequence[Point3],
    fallback: float = 0.5,
    min_scale: float = MIN_SCALE,
    max_scale: float = MAX_SCALE,
) -> float:
    """Estimate a marker scale from the spatial spread of nodes.

    The heuristic uses the bounding-box diagonal divided by ``sqrt(n)`` as
    a proxy for the average spacing between nodes, then takes a fraction of
    that spacing so markers stay visible without overlapping on dense maps
    or vanishing on sparse ones.

    Parameters
    ----------
    positions:
        Node positions as ``(x, y, z)`` tuples (z is ignored).
    fallback:
        Scale returned when there are not enough nodes to estimate spread
        (fewer than two, or all coincident).
    min_scale, max_scale:
        Inclusive clamp applied to the result.

    Returns
    -------
    float
        A marker scale clamped to ``[min_scale, max_scale]``.
    """
    pts = list(positions)
    n = len(pts)
    if n < 2:
        return _clamp(fallback, min_scale, max_scale)

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    diagonal = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    if diagonal <= 0.0:
        return _clamp(fallback, min_scale, max_scale)

    spacing = diagonal / math.sqrt(n)
    scale = spacing * 0.5
    return _clamp(scale, min_scale, max_scale)


def ordered_action_names(nodes: Sequence[dict]) -> List[str]:
    """Return distinct edge action names in first-seen order.

    The ordering is stable so colour assignment and the legend remain
    consistent between rebuilds.
    """
    seen: List[str] = []
    seen_set = set()
    for entry in nodes:
        node = entry.get('node', {})
        for edge in node.get('edges', []) or []:
            action = edge.get('action', '') or ''
            if action and action not in seen_set:
                seen_set.add(action)
                seen.append(action)
    return seen
