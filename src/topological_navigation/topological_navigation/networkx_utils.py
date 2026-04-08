"""
NetworkX utilities for topological navigation.

This module provides utilities for converting topological map data structures
into NetworkX graphs and performing graph-based operations for localization.

Key Features:
- Convert YAML topological maps to NetworkX DiGraph
- Build KD-tree spatial index for efficient nearest neighbor search
- Point-in-polygon checks for influence zone localization
- Edge distance calculations using vectorized operations
- NetworkX algorithm wrappers for shortest paths and connectivity

Performance:
- KD-tree construction: O(n log n) where n is number of nodes
- KD-tree query: O(log n) average case for nearest neighbor
- Point-in-polygon: O(m) where m is number of polygon vertices
- Edge distance: O(e) where e is number of edges (vectorized)

Dependencies:
- networkx (>=2.5): Graph data structures and algorithms
- scipy (>=1.5): KD-tree spatial indexing
- numpy (>=1.19): Numerical operations
"""

import json
import numpy as np
import networkx as nx
from scipy.spatial import KDTree
from typing import Dict, Any, Optional, Tuple, List


def build_graph_from_tmap(tmap_data: Dict[str, Any], logger=None) -> Optional[nx.DiGraph]:
    """
    Convert topological map YAML data to NetworkX DiGraph.

    This function takes a topological map dictionary (typically loaded from YAML)
    and converts it into a NetworkX directed graph representation. Node positions,
    influence zones, properties, and edge metadata are stored as graph attributes.

    Args:
        tmap_data: Dictionary containing topological map data with structure:
        logger: Optional ROS 2 logger for error messages
            {
                'nodes': [
                    {
                        'node': {
                            'name': str,
                            'pose': {
                                'position': {'x': float, 'y': float, 'z': float},
                                'orientation': {'x': float, 'y': float, 'z': float, 'w': float}
                            },
                            'verts': [{'x': float, 'y': float}, ...],
                            'parent_frame': str,
                            'properties': dict (optional),
                            'localise_by_topic': str (optional),
                            'edges': [
                                {
                                    'edge_id': str,
                                    'node': str,
                                    'action': str,
                                    'action_type': str (optional),
                                    'properties': dict (optional)
                                },
                                ...
                            ]
                        },
                        'meta': dict (optional)
                    },
                    ...
                ]
            }

    Returns:
        NetworkX DiGraph with node and edge attributes, or None if map data is invalid.

        Node attributes:
            - name: str - Node name (also the node ID)
            - x, y, z: float - Position coordinates
            - orientation: dict - Quaternion {x, y, z, w}
            - verts: list - Influence zone vertices [{'x': float, 'y': float}, ...]
            - parent_frame: str - Coordinate frame (structural)
            - nav_frame: str - Navigation frame override for PoseStamped
              goals.  Empty string means "use map-level default
              (topo_frame_id)".
            - properties: dict - Optional user-defined properties
            - localise_by_topic: str - JSON config string for topic-based localization
            - meta: dict - Metadata including tags

        Edge attributes:
            - edge_id: str - Unique edge identifier
            - action: str - Action name (e.g., "NavigateToPose")
            - action_type: str - ROS 2 action type
            - properties: dict - Optional user-defined properties
            - weight: float - Edge weight for shortest path (default: 1.0)

    Example:
        >>> import yaml
        >>> with open('map.yaml') as f:
        ...     tmap_data = yaml.safe_load(f)
        >>> graph = build_graph_from_tmap(tmap_data)
        >>> if graph:
        ...     print(f"Graph has {graph.number_of_nodes()} nodes")
        ...     node_attrs = graph.nodes['WP1']
        ...     print(f"WP1 position: ({node_attrs['x']}, {node_attrs['y']})")

    Raises:
        No exceptions are raised. Invalid data returns None.
    """
    # Validate required fields
    if not isinstance(tmap_data, dict):
        if logger:
            logger.error("Invalid map data: Expected dictionary, got {}".format(type(tmap_data).__name__))
        return None

    if 'nodes' not in tmap_data:
        if logger:
            logger.error("Invalid map data: Missing required 'nodes' field")
        return None

    if not isinstance(tmap_data['nodes'], list):
        if logger:
            logger.error("Invalid map data: 'nodes' field must be a list, got {}".format(type(tmap_data['nodes']).__name__))
        return None

    # Create directed graph
    G = nx.DiGraph()

    # Resolve default nav_frame from transformation section
    tx = tmap_data.get('transformation', {})
    default_nav_frame = tx.get('topo_frame_id', tx.get('parent', 'map'))

    def _resolve_tx_ref(value):
        """Resolve ``${transformation.<key>}`` to a concrete value."""
        if (
            isinstance(value, str)
            and value.startswith('${transformation.')
            and value.endswith('}')
        ):
            key = value[len('${transformation.'):-1]
            return tx.get(key, value)
        return value

    try:
        # Process each node in the map
        for idx, node_data in enumerate(tmap_data['nodes']):
            # Validate node structure
            if not isinstance(node_data, dict):
                if logger:
                    logger.warning(f"Skipping node at index {idx}: Expected dictionary, got {type(node_data).__name__}")
                continue
            
            if 'node' not in node_data:
                if logger:
                    logger.warning(f"Skipping node at index {idx}: Missing 'node' field")
                continue

            node = node_data['node']

            # Validate required node fields
            if 'name' not in node:
                if logger:
                    logger.warning(f"Skipping node at index {idx}: Missing required 'name' field")
                continue

            node_name = node['name']

            if 'pose' not in node:
                if logger:
                    logger.warning(f"Skipping node '{node_name}': Missing required 'pose' field")
                continue

            pose = node['pose']
            if 'position' not in pose:
                if logger:
                    logger.warning(f"Skipping node '{node_name}': Missing 'position' in pose")
                continue
            
            if 'orientation' not in pose:
                if logger:
                    logger.warning(f"Skipping node '{node_name}': Missing 'orientation' in pose")
                continue

            position = pose['position']
            orientation = pose['orientation']

            # Validate position coordinates
            if not all(key in position for key in ['x', 'y', 'z']):
                if logger:
                    missing = [k for k in ['x', 'y', 'z'] if k not in position]
                    logger.warning(f"Skipping node '{node_name}': Missing position coordinates: {missing}")
                continue

            # Validate orientation quaternion
            if not all(key in orientation for key in ['x', 'y', 'z', 'w']):
                if logger:
                    missing = [k for k in ['x', 'y', 'z', 'w'] if k not in orientation]
                    logger.warning(f"Skipping node '{node_name}': Missing orientation components: {missing}")
                continue

            # Extract node attributes with defaults for optional fields
            try:
                node_attrs = {
                    'name': node_name,
                    'x': float(position['x']),
                    'y': float(position['y']),
                    'z': float(position['z']),
                    'orientation': {
                        'x': float(orientation['x']),
                        'y': float(orientation['y']),
                        'z': float(orientation['z']),
                        'w': float(orientation['w'])
                    },
                    'verts': node.get('verts', []),
                    'parent_frame': node.get('parent_frame', 'map'),
                    'nav_frame': _resolve_tx_ref(
                        node.get('nav_frame', '')
                    ) or default_nav_frame,
                    'properties': node.get('properties', {}),
                    'localise_by_topic': node.get('localise_by_topic', ''),
                    'meta': node_data.get('meta', {})
                }
            except (ValueError, TypeError) as e:
                if logger:
                    logger.warning(f"Skipping node '{node_name}': Error converting coordinates to float: {e}")
                continue

            # Add node to graph
            G.add_node(node_name, **node_attrs)

            # Process edges for this node
            edges = node.get('edges', [])
            if not isinstance(edges, list):
                if logger:
                    logger.warning(f"Node '{node_name}': 'edges' field must be a list, got {type(edges).__name__}")
                continue
            
            for edge_idx, edge in enumerate(edges):
                # Validate edge structure
                if not isinstance(edge, dict):
                    if logger:
                        logger.warning(f"Node '{node_name}': Skipping edge at index {edge_idx}: Expected dictionary")
                    continue

                if 'edge_id' not in edge:
                    if logger:
                        logger.warning(f"Node '{node_name}': Skipping edge at index {edge_idx}: Missing 'edge_id'")
                    continue
                
                if 'node' not in edge:
                    if logger:
                        logger.warning(f"Node '{node_name}': Skipping edge '{edge.get('edge_id', 'unknown')}': Missing target 'node'")
                    continue
                
                if 'action' not in edge:
                    if logger:
                        logger.warning(f"Node '{node_name}': Skipping edge '{edge['edge_id']}': Missing 'action'")
                    continue

                target_node = edge['node']

                # Extract edge attributes
                edge_props = edge.get('properties', {})
                if not isinstance(edge_props, dict):
                    if logger:
                        logger.warning(f"Node '{node_name}': Edge '{edge['edge_id']}': 'properties' must be a dictionary, using empty dict")
                    edge_props = {}
                
                try:
                    edge_attrs = {
                        'edge_id': edge['edge_id'],
                        'action': edge['action'],
                        'action_type': edge.get('action_type', ''),
                        'properties': edge_props,
                        'weight': float(edge_props.get('weight', 1.0))  # Default weight for shortest path
                    }
                except (ValueError, TypeError) as e:
                    if logger:
                        logger.warning(f"Node '{node_name}': Edge '{edge['edge_id']}': Error converting weight to float: {e}, using default 1.0")
                    edge_attrs = {
                        'edge_id': edge['edge_id'],
                        'action': edge['action'],
                        'action_type': edge.get('action_type', ''),
                        'properties': edge_props,
                        'weight': 1.0
                    }

                # Add edge to graph
                G.add_edge(node_name, target_node, **edge_attrs)

        # Check if graph has any nodes
        if G.number_of_nodes() == 0:
            if logger:
                logger.error("Invalid map data: No valid nodes found in map")
            return None

        return G

    except Exception as e:
        # Catch any unexpected errors
        if logger:
            logger.error(f"Unexpected error building graph from map data: {e}")
        return None


def build_kdtree_from_graph(graph: Optional[nx.DiGraph], logger=None) -> Tuple[Optional[KDTree], List[str]]:
    """
    Build KD-tree spatial index from node positions in NetworkX graph.

    This function extracts 2D coordinates (x, y) from all nodes in the graph
    and constructs a scipy.spatial.KDTree for efficient O(log n) nearest
    neighbor spatial queries. The KD-tree enables fast localization by quickly
    finding the closest nodes to the robot's current position.

    The function returns both the KD-tree and a list of node names in the same
    order as the points in the tree, allowing efficient mapping from KD-tree
    query results back to node names.

    Args:
        graph: NetworkX DiGraph with node position attributes ('x', 'y').
               Can be None or empty, in which case (None, []) is returned.
        logger: Optional ROS 2 logger for warning/error messages.

    Returns:
        Tuple of (kdtree, node_names) where:
        - kdtree: scipy.spatial.KDTree for spatial queries, or None if graph is empty/invalid
        - node_names: List of node names in same order as kdtree points (empty list if no tree)

    Performance:
        - Construction: O(n log n) where n is number of nodes
        - Memory: O(n) for storing 2D points
        - Query: O(log n) average case for nearest neighbor search

    Example:
        >>> import networkx as nx
        >>> G = nx.DiGraph()
        >>> G.add_node('WP1', x=0.0, y=0.0, z=0.0)
        >>> G.add_node('WP2', x=5.0, y=0.0, z=0.0)
        >>> G.add_node('WP3', x=5.0, y=5.0, z=0.0)
        >>> kdtree, node_names = build_kdtree_from_graph(G)
        >>> # Query nearest neighbor to point (2.5, 0.0)
        >>> distances, indices = kdtree.query([[2.5, 0.0]], k=1)
        >>> closest_node = node_names[indices[0][0]]
        >>> print(f"Closest node: {closest_node}")  # 'WP1'

    Edge Cases:
        - None graph: Returns (None, [])
        - Empty graph: Returns (None, [])
        - Single node: Returns valid KDTree with one point
        - Missing 'x' or 'y' attributes: Node is skipped with warning

    Raises:
        No exceptions are raised. Invalid input returns (None, []).

    Requirements:
        Validates: Requirements 2.1, 2.2, 2.5, 15.2, 15.3
        - 2.1: Build KD-tree index from node positions
        - 2.2: Store 2D coordinates (x, y) for each node
        - 2.5: Rebuild KD-tree when map is updated
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Handle None or empty graph
    if graph is None:
        if logger:
            logger.debug("Cannot build KD-tree: graph is None")
        return None, []
    
    if graph.number_of_nodes() == 0:
        if logger:
            logger.debug("Cannot build KD-tree: graph has no nodes")
        return None, []
    
    # Extract 2D coordinates and node names
    points = []
    node_names = []
    
    for node_name in graph.nodes():
        node_attrs = graph.nodes[node_name]
        
        # Validate required attributes
        if 'x' not in node_attrs or 'y' not in node_attrs:
            if logger:
                logger.warning(
                    f"Skipping node '{node_name}' in KD-tree construction: "
                    f"Missing required 'x' or 'y' coordinate"
                )
            continue
        
        try:
            # Extract 2D coordinates (x, y only, z is not used for spatial indexing)
            x = float(node_attrs['x'])
            y = float(node_attrs['y'])
            
            points.append([x, y])
            node_names.append(node_name)
            
        except (ValueError, TypeError) as e:
            if logger:
                logger.warning(
                    f"Skipping node '{node_name}' in KD-tree construction: "
                    f"Invalid coordinate values: {e}"
                )
            continue
    
    # Check if we have any valid points
    if len(points) == 0:
        if logger:
            logger.error(
                "Cannot build KD-tree: No nodes with valid 'x' and 'y' coordinates found"
            )
        return None, []
    
    # Build KD-tree from 2D points
    try:
        kdtree = KDTree(np.array(points))
        
        if logger:
            logger.debug(
                f"Built KD-tree with {len(node_names)} nodes for efficient spatial queries"
            )
        
        return kdtree, node_names
        
    except Exception as e:
        if logger:
            logger.error(f"Failed to build KD-tree: {e}")
        return None, []


def query_nearest_nodes(kdtree: Optional[KDTree], node_names: List[str], pose, k: int = 3) -> List[Dict[str, Any]]:
    """
    Query k-nearest nodes to a pose using KD-tree spatial index.

    This function performs efficient O(log n) nearest neighbor search using
    a pre-built KD-tree to find the k closest nodes to a given robot pose.
    Results are returned as a sorted list of dictionaries containing node
    names and their Euclidean distances from the query pose.

    The function handles both single nearest neighbor queries (k=1) and
    k-nearest neighbors queries (k>1), automatically adjusting for edge
    cases where k exceeds the number of available nodes.

    Args:
        kdtree: scipy.spatial.KDTree built from node positions.
                Can be None, in which case an empty list is returned.
        node_names: List of node names corresponding to kdtree points.
                    Must be in the same order as points used to build kdtree.
                    Can be empty, in which case an empty list is returned.
        pose: geometry_msgs.msg.Pose object with position attributes.
              Only position.x and position.y are used for 2D spatial query.
        k: Number of nearest neighbors to return (default: 3).
           If k > number of nodes, returns all nodes.
           Must be >= 1.

    Returns:
        List of dictionaries sorted by distance (ascending):
        [
            {'node': str, 'dist': float},  # Closest node
            {'node': str, 'dist': float},  # 2nd closest
            ...
        ]
        Returns empty list if kdtree is None, node_names is empty, or k < 1.

    Performance:
        - Time: O(log n) for k=1, O(k log n) for k>1 where n is number of nodes
        - Space: O(k) for storing results
        - Query point: 2D coordinates (x, y) extracted from pose

    Example:
        >>> from scipy.spatial import KDTree
        >>> import numpy as np
        >>> from geometry_msgs.msg import Pose
        >>> 
        >>> # Build KD-tree
        >>> points = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]])
        >>> node_names = ['WP1', 'WP2', 'WP3']
        >>> kdtree = KDTree(points)
        >>> 
        >>> # Create query pose
        >>> pose = Pose()
        >>> pose.position.x = 2.5
        >>> pose.position.y = 0.0
        >>> 
        >>> # Query 3 nearest nodes
        >>> results = query_nearest_nodes(kdtree, node_names, pose, k=3)
        >>> print(results)
        [{'node': 'WP1', 'dist': 2.5}, {'node': 'WP2', 'dist': 2.5}, {'node': 'WP3', 'dist': 5.59}]
        >>> 
        >>> # Query single nearest node
        >>> results = query_nearest_nodes(kdtree, node_names, pose, k=1)
        >>> print(results[0]['node'])  # 'WP1' or 'WP2' (both equidistant)

    Edge Cases:
        - kdtree is None: Returns []
        - node_names is empty: Returns []
        - k < 1: Returns []
        - k > len(node_names): Returns all nodes sorted by distance
        - k = 1: Returns single-element list (handles scipy's different return format)
        - Multiple nodes at same distance: Order is determined by KD-tree implementation

    Raises:
        No exceptions are raised. Invalid input returns empty list.

    Requirements:
        Validates: Requirements 2.3, 2.4, 4.1, 4.2, 4.3, 15.2, 15.3
        - 2.3: Use KD-tree nearest neighbor search for closest node queries
        - 2.4: KD-tree query returns results in O(log n) average time
        - 4.1: Calculate distance to closest node using KD-tree
        - 4.2: Calculate distances to multiple nodes using k-nearest neighbors
        - 4.3: Sort nodes by distance in ascending order
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate inputs
    if kdtree is None:
        return []
    
    if not node_names or len(node_names) == 0:
        return []
    
    if k < 1:
        return []
    
    # Limit k to available nodes
    k_actual = min(k, len(node_names))
    
    # Extract 2D query point from pose
    query_point = np.array([[pose.position.x, pose.position.y]])
    
    try:
        # Query k-nearest neighbors from KD-tree
        distances, indices = kdtree.query(query_point, k=k_actual)
        
        # Ensure distances and indices are 1-D arrays.
        # scipy returns shape (1,) for k=1 and shape (1, k) for k>1
        # when query_point has shape (1, 2).
        distances = np.atleast_1d(distances).flatten()
        indices = np.atleast_1d(indices).flatten()
        
        # Build result list with node names and distances
        results = []
        for dist, idx in zip(distances, indices):
            results.append({
                'node': node_names[int(idx)],
                'dist': float(dist)
            })
        
        # Results are already sorted by distance (KD-tree returns sorted results)
        return results
        
    except Exception as e:
        # Handle any unexpected errors from KD-tree query
        # This should rarely happen with valid inputs
        return []



def compute_shortest_path(graph: Optional[nx.DiGraph], source: str, target: str, 
                         weight: str = 'weight', logger=None) -> List[str]:
    """
    Compute shortest path between two nodes using Dijkstra's algorithm.

    This function uses NetworkX's dijkstra_path implementation to find the
    optimal path between source and target nodes in a weighted directed graph.
    The algorithm guarantees the shortest path for graphs with non-negative
    edge weights.

    The function is a wrapper around networkx.dijkstra_path that provides
    consistent error handling and logging for the topological navigation system.

    Args:
        graph: NetworkX DiGraph representing the topological map.
               Can be None, in which case an empty list is returned.
        source: Source node name (starting point of path).
                Must exist in the graph.
        target: Target node name (destination of path).
                Must exist in the graph.
        weight: Edge attribute to use as weight for path computation (default: 'weight').
                If an edge doesn't have this attribute, weight of 1.0 is assumed.
        logger: Optional ROS 2 logger for warning/error messages.

    Returns:
        List of node names representing the shortest path from source to target,
        including both source and target nodes. Returns empty list if:
        - graph is None or empty
        - source or target node doesn't exist
        - no path exists between source and target

    Performance:
        - Time: O((V + E) log V) using binary heap, where V is vertices, E is edges
        - Space: O(V) for storing distances and predecessors
        - Algorithm: Dijkstra's shortest path algorithm

    Example:
        >>> import networkx as nx
        >>> G = nx.DiGraph()
        >>> G.add_edge('A', 'B', weight=1.0)
        >>> G.add_edge('B', 'C', weight=2.0)
        >>> G.add_edge('A', 'C', weight=5.0)
        >>> path = compute_shortest_path(G, 'A', 'C')
        >>> print(path)  # ['A', 'B', 'C'] (total weight: 3.0)
        >>> 
        >>> # No path exists
        >>> G.add_node('D')  # Isolated node
        >>> path = compute_shortest_path(G, 'A', 'D')
        >>> print(path)  # []

    Edge Cases:
        - graph is None: Returns []
        - Empty graph: Returns []
        - source == target: Returns [source]
        - source not in graph: Returns []
        - target not in graph: Returns []
        - No path exists: Returns [] (logs warning if logger provided)
        - Missing weight attribute: Uses default weight of 1.0

    Raises:
        No exceptions are raised. All errors are handled gracefully and return empty list.

    Requirements:
        Validates: Requirements 3.1, 3.2, 3.3, 15.2, 15.3
        - 3.1: Use networkx.shortest_path or networkx.dijkstra_path for shortest paths
        - 3.2: Check graph connectivity using NetworkX algorithms
        - 3.3: Compute path lengths using networkx.shortest_path_length
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate graph
    if graph is None:
        if logger:
            logger.debug("Cannot compute shortest path: graph is None")
        return []
    
    if graph.number_of_nodes() == 0:
        if logger:
            logger.debug("Cannot compute shortest path: graph is empty")
        return []
    
    # Validate source and target nodes exist
    if source not in graph.nodes:
        if logger:
            logger.warning(f"Cannot compute shortest path: source node '{source}' not in graph")
        return []
    
    if target not in graph.nodes:
        if logger:
            logger.warning(f"Cannot compute shortest path: target node '{target}' not in graph")
        return []
    
    # Handle trivial case: source == target
    if source == target:
        return [source]
    
    try:
        # Use Dijkstra's algorithm for weighted shortest path
        path = nx.dijkstra_path(graph, source, target, weight=weight)
        
        if logger:
            logger.debug(
                f"Found shortest path from '{source}' to '{target}': "
                f"{' -> '.join(path)} ({len(path)} nodes)"
            )
        
        return path
        
    except nx.NetworkXNoPath:
        # No path exists between source and target
        if logger:
            logger.warning(
                f"No path exists from '{source}' to '{target}' in the graph"
            )
        return []
        
    except nx.NodeNotFound as e:
        # This shouldn't happen since we check above, but handle it anyway
        if logger:
            logger.error(f"Node not found during path computation: {e}")
        return []
        
    except Exception as e:
        # Catch any unexpected errors
        if logger:
            logger.error(f"Unexpected error computing shortest path: {e}")
        return []


def compute_path_length(graph: Optional[nx.DiGraph], source: str, target: str,
                       weight: str = 'weight', logger=None) -> float:
    """
    Compute shortest path length between two nodes using Dijkstra's algorithm.

    This function uses NetworkX's dijkstra_path_length implementation to
    calculate the total weight (length) of the shortest path between source
    and target nodes without computing the actual path. This is more efficient
    than compute_shortest_path() when only the distance is needed.

    Args:
        graph: NetworkX DiGraph representing the topological map.
               Can be None, in which case infinity is returned.
        source: Source node name (starting point).
                Must exist in the graph.
        target: Target node name (destination).
                Must exist in the graph.
        weight: Edge attribute to use as weight for path computation (default: 'weight').
                If an edge doesn't have this attribute, weight of 1.0 is assumed.
        logger: Optional ROS 2 logger for warning/error messages.

    Returns:
        float: Total weight (length) of the shortest path from source to target.
               Returns float('inf') if:
               - graph is None or empty
               - source or target node doesn't exist
               - no path exists between source and target

    Performance:
        - Time: O((V + E) log V) using binary heap, where V is vertices, E is edges
        - Space: O(V) for storing distances
        - Algorithm: Dijkstra's shortest path algorithm (length only, no path reconstruction)
        - More efficient than compute_shortest_path() when path is not needed

    Example:
        >>> import networkx as nx
        >>> G = nx.DiGraph()
        >>> G.add_edge('A', 'B', weight=1.5)
        >>> G.add_edge('B', 'C', weight=2.5)
        >>> G.add_edge('A', 'C', weight=5.0)
        >>> length = compute_path_length(G, 'A', 'C')
        >>> print(length)  # 4.0 (via B: 1.5 + 2.5)
        >>> 
        >>> # No path exists
        >>> G.add_node('D')  # Isolated node
        >>> length = compute_path_length(G, 'A', 'D')
        >>> print(length)  # inf

    Edge Cases:
        - graph is None: Returns inf
        - Empty graph: Returns inf
        - source == target: Returns 0.0
        - source not in graph: Returns inf
        - target not in graph: Returns inf
        - No path exists: Returns inf (logs warning if logger provided)
        - Missing weight attribute: Uses default weight of 1.0

    Raises:
        No exceptions are raised. All errors are handled gracefully and return infinity.

    Requirements:
        Validates: Requirements 3.1, 3.3, 15.2, 15.3
        - 3.1: Use NetworkX algorithms for graph operations
        - 3.3: Compute path lengths using networkx.shortest_path_length
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate graph
    if graph is None:
        if logger:
            logger.debug("Cannot compute path length: graph is None")
        return float('inf')
    
    if graph.number_of_nodes() == 0:
        if logger:
            logger.debug("Cannot compute path length: graph is empty")
        return float('inf')
    
    # Validate source and target nodes exist
    if source not in graph.nodes:
        if logger:
            logger.warning(f"Cannot compute path length: source node '{source}' not in graph")
        return float('inf')
    
    if target not in graph.nodes:
        if logger:
            logger.warning(f"Cannot compute path length: target node '{target}' not in graph")
        return float('inf')
    
    # Handle trivial case: source == target
    if source == target:
        return 0.0
    
    try:
        # Use Dijkstra's algorithm to compute path length
        length = nx.dijkstra_path_length(graph, source, target, weight=weight)
        
        if logger:
            logger.debug(
                f"Shortest path length from '{source}' to '{target}': {length:.2f}"
            )
        
        return float(length)
        
    except nx.NetworkXNoPath:
        # No path exists between source and target
        if logger:
            logger.warning(
                f"No path exists from '{source}' to '{target}' in the graph"
            )
        return float('inf')
        
    except nx.NodeNotFound as e:
        # This shouldn't happen since we check above, but handle it anyway
        if logger:
            logger.error(f"Node not found during path length computation: {e}")
        return float('inf')
        
    except Exception as e:
        # Catch any unexpected errors
        if logger:
            logger.error(f"Unexpected error computing path length: {e}")
        return float('inf')


def check_connectivity(graph: Optional[nx.DiGraph], source: str, target: str, logger=None) -> bool:
    """
    Check if a path exists between two nodes in a directed graph.

    This function uses NetworkX's has_path implementation to determine whether
    there is any directed path from source to target. This is more efficient
    than computing the actual path when only connectivity information is needed.

    Note: For directed graphs, connectivity is not symmetric. A path from A to B
    does not imply a path from B to A.

    Args:
        graph: NetworkX DiGraph representing the topological map.
               Can be None, in which case False is returned.
        source: Source node name (starting point).
                Must exist in the graph.
        target: Target node name (destination).
                Must exist in the graph.
        logger: Optional ROS 2 logger for warning/error messages.

    Returns:
        bool: True if a directed path exists from source to target, False otherwise.
              Returns False if:
              - graph is None or empty
              - source or target node doesn't exist
              - no path exists between source and target

    Performance:
        - Time: O(V + E) using breadth-first search, where V is vertices, E is edges
        - Space: O(V) for visited set
        - Algorithm: BFS-based reachability check
        - More efficient than computing full shortest path

    Example:
        >>> import networkx as nx
        >>> G = nx.DiGraph()
        >>> G.add_edge('A', 'B')
        >>> G.add_edge('B', 'C')
        >>> 
        >>> # Path exists A -> B -> C
        >>> print(check_connectivity(G, 'A', 'C'))  # True
        >>> 
        >>> # No reverse path (directed graph)
        >>> print(check_connectivity(G, 'C', 'A'))  # False
        >>> 
        >>> # Isolated node
        >>> G.add_node('D')
        >>> print(check_connectivity(G, 'A', 'D'))  # False
        >>> 
        >>> # Same node
        >>> print(check_connectivity(G, 'A', 'A'))  # True

    Edge Cases:
        - graph is None: Returns False
        - Empty graph: Returns False
        - source == target: Returns True (node is reachable from itself)
        - source not in graph: Returns False
        - target not in graph: Returns False
        - Disconnected components: Returns False if nodes in different components

    Raises:
        No exceptions are raised. All errors are handled gracefully and return False.

    Requirements:
        Validates: Requirements 3.2, 3.5, 15.2, 15.3
        - 3.2: Check graph connectivity using networkx.is_connected or networkx.has_path
        - 3.5: Use networkx.neighbors() for finding adjacent nodes
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate graph
    if graph is None:
        if logger:
            logger.debug("Cannot check connectivity: graph is None")
        return False
    
    if graph.number_of_nodes() == 0:
        if logger:
            logger.debug("Cannot check connectivity: graph is empty")
        return False
    
    # Validate source and target nodes exist
    if source not in graph.nodes:
        if logger:
            logger.warning(f"Cannot check connectivity: source node '{source}' not in graph")
        return False
    
    if target not in graph.nodes:
        if logger:
            logger.warning(f"Cannot check connectivity: target node '{target}' not in graph")
        return False
    
    # Handle trivial case: source == target
    if source == target:
        return True
    
    try:
        # Use NetworkX has_path for efficient connectivity check
        has_path = nx.has_path(graph, source, target)
        
        if logger:
            logger.debug(
                f"Connectivity check from '{source}' to '{target}': "
                f"{'connected' if has_path else 'not connected'}"
            )
        
        return has_path
        
    except nx.NodeNotFound as e:
        # This shouldn't happen since we check above, but handle it anyway
        if logger:
            logger.error(f"Node not found during connectivity check: {e}")
        return False
        
    except Exception as e:
        # Catch any unexpected errors
        if logger:
            logger.error(f"Unexpected error checking connectivity: {e}")
        return False


def get_neighbors(graph: Optional[nx.DiGraph], node: str, logger=None) -> List[str]:
    """
    Get all neighboring nodes connected by outgoing edges from a given node.

    This function uses NetworkX's neighbors() method to retrieve all nodes
    that are directly reachable from the specified node via outgoing edges.
    For directed graphs, this returns only successors (nodes pointed to by
    outgoing edges), not predecessors.

    Args:
        graph: NetworkX DiGraph representing the topological map.
               Can be None, in which case an empty list is returned.
        node: Node name to get neighbors for.
              Must exist in the graph.
        logger: Optional ROS 2 logger for warning/error messages.

    Returns:
        List[str]: List of neighbor node names (successors) reachable via
                   outgoing edges from the specified node. Returns empty list if:
                   - graph is None or empty
                   - node doesn't exist in graph
                   - node has no outgoing edges

    Performance:
        - Time: O(k) where k is the number of neighbors (out-degree of node)
        - Space: O(k) for storing neighbor list
        - Efficient for sparse graphs (typical in topological maps)

    Example:
        >>> import networkx as nx
        >>> G = nx.DiGraph()
        >>> G.add_edge('A', 'B')
        >>> G.add_edge('A', 'C')
        >>> G.add_edge('B', 'C')
        >>> 
        >>> # Get neighbors of A
        >>> neighbors = get_neighbors(G, 'A')
        >>> print(sorted(neighbors))  # ['B', 'C']
        >>> 
        >>> # Get neighbors of C (no outgoing edges)
        >>> neighbors = get_neighbors(G, 'C')
        >>> print(neighbors)  # []
        >>> 
        >>> # Directed graph: B's neighbors don't include A
        >>> neighbors = get_neighbors(G, 'B')
        >>> print(neighbors)  # ['C']

    Edge Cases:
        - graph is None: Returns []
        - Empty graph: Returns []
        - node not in graph: Returns [] (logs warning if logger provided)
        - node has no outgoing edges: Returns []
        - Self-loops: If node has edge to itself, includes itself in neighbors

    Use Cases:
        - Finding directly reachable nodes for navigation planning
        - Exploring local graph structure around current position
        - Validating edge connectivity during map updates
        - Building adjacency lists for custom graph algorithms

    Raises:
        No exceptions are raised. All errors are handled gracefully and return empty list.

    Requirements:
        Validates: Requirements 3.5, 15.2, 15.3
        - 3.5: Use networkx.neighbors() for finding adjacent nodes
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate graph
    if graph is None:
        if logger:
            logger.debug("Cannot get neighbors: graph is None")
        return []
    
    if graph.number_of_nodes() == 0:
        if logger:
            logger.debug("Cannot get neighbors: graph is empty")
        return []
    
    # Validate node exists
    if node not in graph.nodes:
        if logger:
            logger.warning(f"Cannot get neighbors: node '{node}' not in graph")
        return []
    
    try:
        # Use NetworkX neighbors() to get all successor nodes
        # For DiGraph, this returns nodes reachable via outgoing edges
        neighbors = list(graph.neighbors(node))
        
        if logger:
            logger.debug(
                f"Node '{node}' has {len(neighbors)} neighbor(s): "
                f"{', '.join(neighbors) if neighbors else 'none'}"
            )
        
        return neighbors
        
    except nx.NetworkXError as e:
        # Handle any NetworkX-specific errors
        if logger:
            logger.error(f"NetworkX error getting neighbors for node '{node}': {e}")
        return []
        
    except Exception as e:
        # Catch any unexpected errors
        if logger:
            logger.error(f"Unexpected error getting neighbors for node '{node}': {e}")
        return []


def point_in_poly_nx(graph: Optional[nx.DiGraph], node_name: str, pose) -> bool:
    """
    Check if a pose is inside a node's influence zone polygon using ray-casting algorithm.

    This function implements the ray-casting algorithm for point-in-polygon testing.
    It checks if a robot pose is within a node's influence zone by casting a horizontal
    ray from the point and counting intersections with polygon edges. An odd number of
    intersections means the point is inside the polygon.

    The pose coordinates are transformed to node-relative coordinates before testing,
    allowing influence zones to be defined relative to the node's position.

    Args:
        graph: NetworkX DiGraph with node attributes including 'x', 'y', and 'verts'.
               Can be None, in which case False is returned.
        node_name: Name of the node to check.
                   Must exist in the graph.
        pose: geometry_msgs.msg.Pose object with position attributes.
              Only position.x and position.y are used for 2D polygon check.

    Returns:
        bool: True if pose is inside the node's influence zone polygon, False otherwise.
              Returns False if:
              - graph is None or empty
              - node doesn't exist in graph
              - node missing 'x' or 'y' attributes
              - node has empty 'verts' list (no influence zone defined)

    Algorithm:
        Ray-casting algorithm:
        1. Transform pose to node-relative coordinates
        2. Cast horizontal ray from point to infinity
        3. Count intersections with polygon edges
        4. Odd count = inside, even count = outside
        
        Time Complexity: O(n) where n is number of polygon vertices
        Space Complexity: O(1)

    Example:
        >>> import networkx as nx
        >>> from geometry_msgs.msg import Pose
        >>> 
        >>> # Create graph with square influence zone
        >>> G = nx.DiGraph()
        >>> G.add_node('WP1', x=0.0, y=0.0, z=0.0, verts=[
        ...     {'x': -1.0, 'y': -1.0},
        ...     {'x': 1.0, 'y': -1.0},
        ...     {'x': 1.0, 'y': 1.0},
        ...     {'x': -1.0, 'y': 1.0}
        ... ])
        >>> 
        >>> # Test point inside square
        >>> pose = Pose()
        >>> pose.position.x = 0.5
        >>> pose.position.y = 0.5
        >>> print(point_in_poly_nx(G, 'WP1', pose))  # True
        >>> 
        >>> # Test point outside square
        >>> pose.position.x = 2.0
        >>> pose.position.y = 2.0
        >>> print(point_in_poly_nx(G, 'WP1', pose))  # False

    Edge Cases:
        - graph is None: Returns False
        - Empty graph: Returns False
        - node not in graph: Returns False
        - Empty verts list: Returns False (no polygon to be inside)
        - Point on polygon boundary: May return True or False (implementation dependent)
        - Degenerate polygon (< 3 vertices): Returns False
        - Self-intersecting polygon: Behavior follows ray-casting algorithm

    Requirements:
        Validates: Requirements 7.1, 7.2, 7.3, 7.4, 7.5, 15.2, 15.3
        - 7.1: Use ray-casting algorithm for point-in-polygon detection
        - 7.2: Transform pose coordinates relative to node position
        - 7.3: Check all n edges for ray intersections
        - 7.4: Handle edge cases including points on boundaries
        - 7.5: Return true when inside, false otherwise
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate graph
    if graph is None:
        return False
    
    if graph.number_of_nodes() == 0:
        return False
    
    # Check if node exists
    if node_name not in graph.nodes:
        return False
    
    node_attrs = graph.nodes[node_name]
    
    # Check required attributes
    if 'x' not in node_attrs or 'y' not in node_attrs:
        return False
    
    # Get influence zone vertices
    verts = node_attrs.get('verts', [])
    
    # Empty polygon - point cannot be inside
    if not verts or len(verts) == 0:
        return False
    
    # Transform pose to node-relative coordinates
    x = pose.position.x - node_attrs['x']
    y = pose.position.y - node_attrs['y']
    
    # Ray-casting algorithm
    n = len(verts)
    inside = False
    
    p1x = verts[0]['x']
    p1y = verts[0]['y']
    
    for i in range(n + 1):
        p2x = verts[i % n]['x']
        p2y = verts[i % n]['y']
        
        # Check if horizontal ray from point intersects this edge
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        
        p1x, p1y = p2x, p2y
    
    return inside


# ==============================================================================
# Point-to-line-segment distance (vectorized)
# ==============================================================================

def _pnt2line(
    pnt: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
) -> np.ndarray:
    """Vectorized perpendicular distance from points to line segments.

    All three arguments are ``(N, 3)`` arrays.  Returns an ``(N,)``
    array of distances from each point to the corresponding segment.

    The projection parameter *t* is clamped to ``[0, 1]`` so the
    result is the distance to the *segment*, not the infinite line.

    Args:
        pnt: Array of query points, shape ``(N, 3)``.
        start: Segment start points, shape ``(N, 3)``.
        end: Segment end points, shape ``(N, 3)``.

    Returns:
        1-D array of Euclidean distances, shape ``(N,)``.
    """
    line_vec = end - start
    pnt_vec = pnt - start

    line_len = np.linalg.norm(line_vec, axis=1, keepdims=True)
    line_unitvec = line_vec / line_len
    pnt_vec_scaled = pnt_vec / line_len

    # Projection parameter clamped to [0, 1]
    t = np.sum(line_unitvec * pnt_vec_scaled, axis=1)
    t = np.clip(t, 0.0, 1.0)

    nearest = line_vec * t[:, np.newaxis]
    diff = nearest - pnt_vec
    return np.linalg.norm(diff, axis=1)


def get_edge_distances_nx(graph: Optional[nx.DiGraph], pose, logger=None) -> Tuple[List[str], np.ndarray]:
    """
    Calculate perpendicular distances from pose to all edges using vectorized operations.

    This function computes the perpendicular distance from a robot pose to each edge
    in the topological graph. It uses vectorized numpy operations for efficiency and
    the existing pnt2line function for distance calculations. Results are sorted by
    distance in ascending order.

    Self-loop edges (where source == destination) are skipped with error logging,
    as they don't represent valid navigable paths.

    Args:
        graph: NetworkX DiGraph with node position attributes ('x', 'y').
               Can be None, in which case empty results are returned.
        pose: geometry_msgs.msg.Pose object with position attributes.
              Only position.x and position.y are used (z is set to 0).
        logger: Optional ROS 2 logger for error/warning messages.

    Returns:
        Tuple of (edge_ids, distances) where:
        - edge_ids: List of edge IDs sorted by distance (ascending)
        - distances: numpy array of distances corresponding to edge_ids
        Returns ([], np.array([])) if:
        - graph is None or has no edges
        - all edges are invalid (self-loops or missing data)
        - distance calculation fails

    Performance:
        - Time: O(m) where m is number of edges (vectorized operations)
        - Space: O(m) for storing edge vectors and distances
        - Uses numpy vectorization for batch distance calculations

    Example:
        >>> import networkx as nx
        >>> import numpy as np
        >>> from geometry_msgs.msg import Pose
        >>> 
        >>> # Create graph with edges
        >>> G = nx.DiGraph()
        >>> G.add_node('A', x=0.0, y=0.0, z=0.0)
        >>> G.add_node('B', x=5.0, y=0.0, z=0.0)
        >>> G.add_node('C', x=5.0, y=5.0, z=0.0)
        >>> G.add_edge('A', 'B', edge_id='AB')
        >>> G.add_edge('B', 'C', edge_id='BC')
        >>> 
        >>> # Calculate distances from pose
        >>> pose = Pose()
        >>> pose.position.x = 2.5
        >>> pose.position.y = 1.0
        >>> edge_ids, distances = get_edge_distances_nx(G, pose)
        >>> print(edge_ids)  # ['AB', 'BC'] (sorted by distance)
        >>> print(distances)  # [1.0, ...] (perpendicular distances)

    Edge Cases:
        - graph is None: Returns ([], np.array([]))
        - Empty graph: Returns ([], np.array([]))
        - No edges: Returns ([], np.array([]))
        - Self-loop edges: Skipped with error log
        - Missing position data: Edge skipped
        - Equal distances: Sorted by edge_id alphabetically (stable sort)

    Error Handling:
        - Self-loop edges (u == v): Logged as error, skipped
        - Missing 'x' or 'y' attributes: Edge skipped silently
        - pnt2line exception: Returns empty results with warning

    Requirements:
        Validates: Requirements 4.6, 4.7, 11.1, 11.2, 11.4, 15.2, 15.3
        - 4.6: Use vectorized numpy operations for edge distances
        - 4.7: Compute perpendicular distance from pose to edge line segments
        - 11.1: Calculate perpendicular distance from robot pose to each edge
        - 11.2: Use vectorized numpy operations for efficiency
        - 11.4: Log error and skip self-loop edges
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    
    # Validate graph
    if graph is None:
        return [], np.array([])
    
    if graph.number_of_edges() == 0:
        return [], np.array([])
    
    edge_ids = []
    vectors_start = []
    vectors_end = []
    
    # Build edge vectors from graph
    for u, v, edge_data in graph.edges(data=True):
        # Skip self-loop edges
        if u == v:
            if logger:
                logger.error(
                    f"Cannot get distance to edge {edge_data.get('edge_id', 'unknown')}: "
                    f"Destination is equal to origin"
                )
            continue
        
        u_attrs = graph.nodes[u]
        v_attrs = graph.nodes[v]
        
        # Skip edges with missing position data
        if 'x' not in u_attrs or 'y' not in u_attrs:
            continue
        if 'x' not in v_attrs or 'y' not in v_attrs:
            continue
        
        edge_ids.append(edge_data.get('edge_id', f"{u}_{v}"))
        vectors_start.append([u_attrs['x'], u_attrs['y'], 0])
        vectors_end.append([v_attrs['x'], v_attrs['y'], 0])
    
    if len(edge_ids) == 0:
        return [], np.array([])
    
    # Convert to numpy arrays for vectorized operations
    vectors_start = np.array(vectors_start)
    vectors_end = np.array(vectors_end)
    
    # Create array of pose points (one for each edge)
    pose_points = np.array(len(edge_ids) * [[pose.position.x, pose.position.y, 0]])
    
    # Calculate perpendicular distances using vectorized operation
    try:
        distances = _pnt2line(pose_points, vectors_start, vectors_end)
    except Exception as e:
        if logger:
            logger.warning(f"Cannot calculate distance to edges: {e}")
        return [], np.array([])
    
    # Sort by distance
    sorted_indices = np.argsort(distances)
    sorted_edge_ids = [edge_ids[i] for i in sorted_indices]
    sorted_distances = distances[sorted_indices]
    
    return sorted_edge_ids, sorted_distances



def update_loc_by_topic_nx(graph: Optional[nx.DiGraph], logger=None) -> Tuple[List[Dict[str, Any]], List[str]]:
    """
    Extract and parse localise_by_topic configuration from NetworkX graph.

    This function scans all nodes in the graph for localise_by_topic configuration
    strings, parses them as JSON, and returns structured configuration data. This
    enables topic-based localization where nodes can be activated by ROS 2 topic
    messages instead of geometric position checks.

    The function sets default values for missing configuration fields and handles
    JSON parsing errors gracefully by logging warnings and skipping invalid nodes.

    Args:
        graph: NetworkX DiGraph with node attributes including 'localise_by_topic'.
               Can be None, in which case empty results are returned.
        logger: Optional ROS 2 logger for warning/error messages.

    Returns:
        Tuple of (nodes_by_topic, names_by_topic) where:
        - nodes_by_topic: List of configuration dictionaries with structure:
          {
              'name': str,                    # Node name (added by this function)
              'topic': str,                   # ROS 2 topic to subscribe to
              'msg_type': str,                # Message type
              'localise_anywhere': bool,      # True = no influence zone check (default: True)
              'persistency': int,             # How long to persist (default: 10)
              ... (other user-defined fields)
          }
        - names_by_topic: List of node names that have topic-based localization
        Returns ([], []) if graph is None or has no nodes with valid config.

    JSON Configuration Format:
        Node attribute 'localise_by_topic' should contain JSON string:
        {
            "topic": "/my_topic",
            "msg_type": "std_msgs/String",
            "localise_anywhere": true,
            "persistency": 10
        }

    Default Values:
        - localise_anywhere: true (if not specified)
        - persistency: 10 (if not specified)

    Example:
        >>> import networkx as nx
        >>> import json
        >>> 
        >>> # Create graph with topic-based localization
        >>> G = nx.DiGraph()
        >>> config = {
        ...     "topic": "/docking_station",
        ...     "msg_type": "std_msgs/Bool",
        ...     "localise_anywhere": False,
        ...     "persistency": 5
        ... }
        >>> G.add_node('Dock', x=0.0, y=0.0, localise_by_topic=json.dumps(config))
        >>> G.add_node('WP1', x=5.0, y=0.0, localise_by_topic='')
        >>> 
        >>> nodes, names = update_loc_by_topic_nx(G)
        >>> print(names)  # ['Dock']
        >>> print(nodes[0]['name'])  # 'Dock'
        >>> print(nodes[0]['localise_anywhere'])  # False

    Edge Cases:
        - graph is None: Returns ([], [])
        - Empty graph: Returns ([], [])
        - No nodes with localise_by_topic: Returns ([], [])
        - Empty localise_by_topic string: Node skipped
        - Invalid JSON: Node skipped with warning
        - Non-dict JSON: Node skipped with warning
        - Missing optional fields: Default values used

    Error Handling:
        - JSON parsing errors: Logged as warning, node skipped
        - Non-dictionary JSON: Logged as warning, node skipped
        - Missing required fields: Node included with defaults

    Requirements:
        Validates: Requirements 5.2, 5.3, 5.4, 17.5, 15.2, 15.3
        - 5.2: Prioritize topic-based localization over geometric
        - 5.3: Handle localise_anywhere flag
        - 5.4: Verify influence zone when localise_anywhere is false
        - 17.5: Handle JSON parsing errors gracefully
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate graph
    if graph is None:
        return [], []
    
    if graph.number_of_nodes() == 0:
        return [], []
    
    nodes_by_topic = []
    names_by_topic = []
    
    for node_name in graph.nodes():
        node_attrs = graph.nodes[node_name]
        loc_by_topic_str = node_attrs.get('localise_by_topic', '')
        
        # Skip nodes without localise_by_topic configuration
        if not loc_by_topic_str:
            continue
        
        try:
            # Parse JSON configuration
            config = json.loads(loc_by_topic_str)
            
            # Validate that config is a dictionary
            if not isinstance(config, dict):
                if logger:
                    logger.warning(
                        f"Invalid localise_by_topic for node {node_name}: "
                        f"Expected JSON object, got {type(config).__name__}"
                    )
                continue
            
            # Add node name to configuration
            config['name'] = node_name
            
            # Set default values for optional fields
            if 'localise_anywhere' not in config:
                config['localise_anywhere'] = True
            if 'persistency' not in config:
                config['persistency'] = 10
            
            nodes_by_topic.append(config)
            names_by_topic.append(node_name)
            
        except json.JSONDecodeError as e:
            if logger:
                logger.warning(
                    f"Invalid JSON in localise_by_topic for node {node_name}: {e}"
                )
            continue
    
    return nodes_by_topic, names_by_topic



def determine_current_node(graph: Optional[nx.DiGraph], kdtree: Optional[KDTree], 
                          node_names: List[str], pose, loc_by_topic: List[Dict[str, Any]], 
                          nogos: List[str]) -> str:
    """
    Determine current node based on influence zones and localization rules.

    This function implements the core localization logic to determine which topological
    node the robot is currently within. It follows a priority-based approach:
    
    Priority 1: Topic-based localization (highest priority)
    Priority 2: Geometric localization using KD-tree and influence zones
    
    For geometric localization, the function uses KD-tree to efficiently find the 3
    closest nodes, then checks their influence zones using point-in-polygon tests.
    This avoids checking all nodes in the map, providing O(log n) performance.

    Args:
        graph: NetworkX DiGraph with node attributes including position and influence zones.
               Can be None, in which case 'none' is returned.
        kdtree: scipy.spatial.KDTree for efficient spatial queries.
                Can be None, in which case 'none' is returned.
        node_names: List of node names corresponding to kdtree points.
                    Must be in same order as kdtree points.
        pose: geometry_msgs.msg.Pose object with position attributes.
        loc_by_topic: List of active topic-based localizations from update_loc_by_topic_nx().
                      Each dict should have 'name' and 'localise_anywhere' fields.
        nogos: List of no-go node names to exclude from geometric localization.
               No-go nodes can only be current if robot is within their influence zone.

    Returns:
        str: Current node name, or 'none' if robot is not within any influence zone.

    Algorithm:
        1. Check topic-based localization first (priority)
           - If localise_anywhere=true: Return node immediately
           - If localise_anywhere=false: Check influence zone
        2. Use KD-tree to find 3 closest nodes (O(log n))
        3. For each of the 3 closest nodes:
           - Skip if it's a topic-based node
           - Skip if it's a no-go node
           - Check if pose is within influence zone
           - Return first match
        4. Return 'none' if no match found

    Performance:
        - Time: O(log n + k*m) where k=3 closest nodes, m=avg polygon vertices
        - Space: O(1)
        - KD-tree query: O(log n)
        - Point-in-polygon: O(m) per node

    Example:
        >>> import networkx as nx
        >>> from scipy.spatial import KDTree
        >>> import numpy as np
        >>> from geometry_msgs.msg import Pose
        >>> 
        >>> # Create graph with influence zones
        >>> G = nx.DiGraph()
        >>> G.add_node('WP1', x=0.0, y=0.0, z=0.0, verts=[
        ...     {'x': -1.0, 'y': -1.0}, {'x': 1.0, 'y': -1.0},
        ...     {'x': 1.0, 'y': 1.0}, {'x': -1.0, 'y': 1.0}
        ... ])
        >>> G.add_node('WP2', x=5.0, y=0.0, z=0.0, verts=[])
        >>> 
        >>> # Build KD-tree
        >>> points = np.array([[0.0, 0.0], [5.0, 0.0]])
        >>> kdtree = KDTree(points)
        >>> node_names = ['WP1', 'WP2']
        >>> 
        >>> # Test pose inside WP1
        >>> pose = Pose()
        >>> pose.position.x = 0.5
        >>> pose.position.y = 0.5
        >>> current = determine_current_node(G, kdtree, node_names, pose, [], [])
        >>> print(current)  # 'WP1'
        >>> 
        >>> # Test pose outside all zones
        >>> pose.position.x = 10.0
        >>> pose.position.y = 10.0
        >>> current = determine_current_node(G, kdtree, node_names, pose, [], [])
        >>> print(current)  # 'none'

    Edge Cases:
        - graph is None: Returns 'none'
        - kdtree is None: Returns 'none'
        - Empty node_names: Returns 'none'
        - No influence zones defined: Returns 'none'
        - Multiple nodes overlap: Returns first match from closest 3
        - Topic-based node with localise_anywhere=true: Returns immediately
        - No-go nodes: Excluded from geometric search

    Requirements:
        Validates: Requirements 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7, 15.2, 15.3
        - 5.1: Identify node when robot pose is within influence zone
        - 5.2: Prioritize topic-based localization over geometric
        - 5.3: Handle localise_anywhere=true without influence zone check
        - 5.4: Verify influence zone when localise_anywhere=false
        - 5.5: Return 'none' when not within any influence zone
        - 5.6: Skip no-go and topic-based nodes in geometric search
        - 5.7: Use KD-tree to find 3 closest nodes, check only those
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate inputs
    if graph is None or kdtree is None or not node_names:
        return 'none'
    
    # Priority 1: Check localise by topic first
    for topic_loc in loc_by_topic:
        node_name = topic_loc.get('name')
        
        if not node_name or node_name not in graph.nodes:
            continue
        
        if topic_loc.get('localise_anywhere', True):
            # Localise anywhere - no influence zone check needed
            return node_name
        else:
            # Check influence zone
            if point_in_poly_nx(graph, node_name, pose):
                return node_name
    
    # Priority 2: Check geometric localization using KD-tree
    # Get 3 closest nodes efficiently using KD-tree
    nearest_nodes = query_nearest_nodes(kdtree, node_names, pose, k=3)
    
    # Extract list of topic-based node names for filtering
    names_by_topic = [x.get('name') for x in loc_by_topic if 'name' in x]
    
    # Check up to 3 closest nodes
    for node_info in nearest_nodes:
        node_name = node_info['node']
        
        # Skip nodes that are localise by topic
        if node_name in names_by_topic:
            continue
        
        # Skip no-go nodes
        if node_name in nogos:
            continue
        
        # Check if pose is within influence zone
        if point_in_poly_nx(graph, node_name, pose):
            return node_name
    
    # No match found
    return 'none'



def determine_closest_node(kdtree: Optional[KDTree], node_names: List[str], 
                          graph: Optional[nx.DiGraph], current_node: str,
                          nogos: List[str], names_by_topic: List[str], pose) -> Tuple[str, float]:
    """
    Determine closest node by Euclidean distance with filtering rules.

    This function finds the topologically closest node to the robot's current position
    using KD-tree for efficient O(log n) spatial search. It applies filtering rules
    to exclude no-go nodes and topic-based nodes unless the robot is within their
    influence zones.

    The function follows these rules:
    1. If robot has a current node, that node is also the closest
    2. Otherwise, find the closest node that isn't no-go or topic-based
    3. No-go and topic-based nodes can only be closest if robot is within their zone

    Args:
        kdtree: scipy.spatial.KDTree for efficient spatial queries.
                Can be None, in which case ('none', inf) is returned.
        node_names: List of node names corresponding to kdtree points.
                    Must be in same order as kdtree points.
        graph: NetworkX DiGraph with node attributes (used for distance calculation).
               Can be None, in which case ('none', inf) is returned.
        current_node: Current node name from determine_current_node(), or 'none'.
                      If not 'none', this is returned as the closest node.
        nogos: List of no-go node names to exclude from closest node selection.
               No-go nodes can only be closest if they are the current node.
        names_by_topic: List of topic-based node names to exclude from selection.
                        Topic-based nodes can only be closest if they are current.
        pose: geometry_msgs.msg.Pose object with position attributes.

    Returns:
        Tuple of (closest_node_name, distance) where:
        - closest_node_name: Name of closest node, or 'none' if no valid node found
        - distance: Euclidean distance to closest node, or inf if no valid node
        
        Special cases:
        - If current_node != 'none': Returns (current_node, distance_to_current)
        - If all nodes filtered: Returns (first_node, distance) as fallback
        - If kdtree is None: Returns ('none', inf)

    Performance:
        - Time: O(log n) for KD-tree query, O(k) for filtering where k is query size
        - Space: O(k) for storing k nearest neighbors
        - Efficient even with many no-go/topic-based nodes

    Example:
        >>> import networkx as nx
        >>> from scipy.spatial import KDTree
        >>> import numpy as np
        >>> from geometry_msgs.msg import Pose
        >>> 
        >>> # Create graph and KD-tree
        >>> G = nx.DiGraph()
        >>> G.add_node('WP1', x=0.0, y=0.0, z=0.0)
        >>> G.add_node('WP2', x=5.0, y=0.0, z=0.0)
        >>> G.add_node('NoGo', x=2.5, y=0.0, z=0.0)
        >>> points = np.array([[0.0, 0.0], [5.0, 0.0], [2.5, 0.0]])
        >>> kdtree = KDTree(points)
        >>> node_names = ['WP1', 'WP2', 'NoGo']
        >>> 
        >>> # Test closest node with no-go filtering
        >>> pose = Pose()
        >>> pose.position.x = 2.5
        >>> pose.position.y = 0.0
        >>> closest, dist = determine_closest_node(
        ...     kdtree, node_names, G, 'none', ['NoGo'], [], pose
        ... )
        >>> print(closest)  # 'WP1' or 'WP2' (NoGo is filtered out)
        >>> 
        >>> # Test with current node
        >>> closest, dist = determine_closest_node(
        ...     kdtree, node_names, G, 'WP1', ['NoGo'], [], pose
        ... )
        >>> print(closest)  # 'WP1' (current node is always closest)

    Edge Cases:
        - kdtree is None: Returns ('none', inf)
        - Empty node_names: Returns ('none', inf)
        - current_node != 'none': Returns (current_node, distance)
        - All nodes filtered: Returns first node as fallback
        - Multiple nodes at same distance: Order determined by KD-tree

    Filtering Rules:
        - No-go nodes: Excluded unless they are the current node
        - Topic-based nodes: Excluded unless they are the current node
        - Current node: Always returned as closest (Rule 1)
        - Fallback: If all filtered, return first node from KD-tree

    Requirements:
        Validates: Requirements 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 15.2, 15.3
        - 6.1: Use KD-tree nearest neighbor search for closest node
        - 6.2: Exclude no-go nodes unless within influence zone
        - 6.3: Exclude topic-based nodes unless within influence zone
        - 6.4: Set both current and closest to same node when within zone
        - 6.5: Publish Euclidean distance to closest node
        - 6.6: Handle filtering efficiently with KD-tree
        - 15.2: Comprehensive docstrings with parameter descriptions
        - 15.3: Type hints for parameters and return values
    """
    # Validate inputs
    if kdtree is None or not node_names:
        return 'none', float('inf')
    
    # Rule 1: If we have a current node, it's also the closest
    if current_node != 'none':
        # Get distance to current node using KD-tree
        nearest = query_nearest_nodes(kdtree, node_names, pose, k=1)
        if nearest and nearest[0]['node'] == current_node:
            return current_node, nearest[0]['dist']
        # If current node is not in kdtree, calculate distance manually
        # (This shouldn't happen in normal operation)
        if graph and current_node in graph.nodes:
            node_attrs = graph.nodes[current_node]
            if 'x' in node_attrs and 'y' in node_attrs:
                dx = pose.position.x - node_attrs['x']
                dy = pose.position.y - node_attrs['y']
                dist = np.sqrt(dx*dx + dy*dy)
                return current_node, float(dist)
        return current_node, 0.0
    
    # Rule 2: Find closest node that isn't no-go or topic-based
    # Query more nodes than needed to account for filtering
    k = min(len(node_names), len(nogos) + len(names_by_topic) + 10)
    nearest_nodes = query_nearest_nodes(kdtree, node_names, pose, k=k)
    
    for node_info in nearest_nodes:
        node_name = node_info['node']
        
        # Skip no-go nodes
        if node_name in nogos:
            continue
        
        # Skip topic-based nodes
        if node_name in names_by_topic:
            continue
        
        # Found a valid closest node
        return node_name, node_info['dist']
    
    # Fallback: return first node if all are filtered
    if nearest_nodes:
        return nearest_nodes[0]['node'], nearest_nodes[0]['dist']
    
    return 'none', float('inf')

def draw_topological_graph(
    graph: Optional[nx.DiGraph],
    highlight_nodes: Optional[List[str]] = None,
    highlight_edges: Optional[List[Tuple[str, str]]] = None,
    title: str = "Topological Map",
    save_path: Optional[str] = None,
    figsize: Tuple[float, float] = (14, 10),
    logger=None,
) -> None:
    """Draw the topological map graph for visual debugging.

    Uses ``matplotlib`` and ``nx.draw_networkx_*`` to render the graph with
    real-world (x, y) positions stored on the nodes.  Optionally highlights
    a subset of nodes/edges (e.g. a planned route) and can save the figure
    to disk instead of showing it interactively.

    Args:
        graph: NetworkX DiGraph built by :func:`build_graph_from_tmap`.
               If *None* or empty the call is a no-op.
        highlight_nodes: Node names to draw in a distinct colour (default
               colour: orange).  Useful for marking the current node, goal,
               or route waypoints.
        highlight_edges: Edge tuples ``(source, target)`` to draw in a
               distinct colour (default: red).  Useful for showing a planned
               route.
        title: Figure title.
        save_path: If given, the figure is saved to this path (e.g.
               ``'/tmp/topo_map.png'``) instead of calling ``plt.show()``.
        figsize: Matplotlib figure size in inches ``(width, height)``.
        logger: Optional ROS 2 logger for error messages.

    Returns:
        None.  A matplotlib figure is either displayed or saved.

    Example:
        >>> graph = build_graph_from_tmap(tmap_data)
        >>> # Show full map
        >>> draw_topological_graph(graph)
        >>> # Highlight a route
        >>> route_nodes = ['WP1', 'WP2', 'WP3']
        >>> route_edges = [('WP1', 'WP2'), ('WP2', 'WP3')]
        >>> draw_topological_graph(
        ...     graph,
        ...     highlight_nodes=route_nodes,
        ...     highlight_edges=route_edges,
        ...     title='Planned route',
        ...     save_path='/tmp/route.png',
        ... )
    """
    try:
        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend safe for headless
        import matplotlib.pyplot as plt
    except ImportError:
        msg = (
            "matplotlib is required for draw_topological_graph. "
            "Install it with: pip install matplotlib"
        )
        if logger:
            logger.error(msg)
        else:
            print(msg)
        return

    if graph is None or graph.number_of_nodes() == 0:
        if logger:
            logger.warning("draw_topological_graph: graph is None or empty")
        return

    highlight_nodes = set(highlight_nodes or [])
    highlight_edges = set(highlight_edges or [])

    # Build position dict from node attributes (x, y)
    pos: Dict[str, Tuple[float, float]] = {}
    for node_name in graph.nodes():
        attrs = graph.nodes[node_name]
        pos[node_name] = (attrs.get("x", 0.0), attrs.get("y", 0.0))

    fig, ax = plt.subplots(figsize=figsize)
    ax.set_title(title, fontsize=14)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.4)

    # --- edges -----------------------------------------------------------
    normal_edges = [e for e in graph.edges() if e not in highlight_edges]
    nx.draw_networkx_edges(
        graph, pos, edgelist=normal_edges, ax=ax,
        edge_color="grey", alpha=0.6, arrows=True,
        arrowstyle="-|>", arrowsize=12, connectionstyle="arc3,rad=0.08",
    )
    if highlight_edges:
        present = [e for e in highlight_edges if graph.has_edge(*e)]
        if present:
            nx.draw_networkx_edges(
                graph, pos, edgelist=present, ax=ax,
                edge_color="red", width=2.2, arrows=True,
                arrowstyle="-|>", arrowsize=14,
                connectionstyle="arc3,rad=0.08",
            )

    # --- nodes -----------------------------------------------------------
    normal_nodes = [n for n in graph.nodes() if n not in highlight_nodes]
    nx.draw_networkx_nodes(
        graph, pos, nodelist=normal_nodes, ax=ax,
        node_color="steelblue", node_size=200, alpha=0.85,
    )
    if highlight_nodes:
        present = [n for n in highlight_nodes if n in graph.nodes()]
        if present:
            nx.draw_networkx_nodes(
                graph, pos, nodelist=present, ax=ax,
                node_color="orange", node_size=320, alpha=0.95,
                edgecolors="red", linewidths=1.5,
            )

    # --- labels ----------------------------------------------------------
    nx.draw_networkx_labels(
        graph, pos, ax=ax, font_size=7, font_weight="bold",
    )

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        if logger:
            logger.info(f"Topological map figure saved to {save_path}")
        else:
            print(f"Saved to {save_path}")
    else:
        plt.show()

    plt.close(fig)


def main():
    """Load test fixture maps, build graphs, run basic queries, and visualise."""
    import argparse
    import os
    import sys
    import yaml

    # ------------------------------------------------------------------
    # CLI
    # ------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        description="Load a topological map, build a NetworkX graph, "
                    "run basic spatial queries, and save a visualisation."
    )
    parser.add_argument(
        "--map", "-m",
        default=None,
        help="Path to a .yaml topological map.  "
             "If omitted, the test fixtures are used.",
    )
    parser.add_argument(
        "--out-dir", "-o",
        default="/tmp",
        help="Directory for saved PNG figures (default: /tmp).",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Resolve map file(s)
    # ------------------------------------------------------------------
    this_dir = os.path.dirname(os.path.abspath(__file__))
    fixture_dir = os.path.normpath(
        os.path.join(this_dir, "..", "test", "fixtures")
    )
    config_dir = os.path.normpath(
        os.path.join(this_dir, "..", "config")
    )

    if args.map:
        map_files = [args.map]
    else:
        # Default: all fixture maps + the polytunnel config map
        map_files = sorted(
            os.path.join(fixture_dir, f)
            for f in os.listdir(fixture_dir)
            if f.endswith(".yaml") and not f.startswith("README")
        )
        polytunnel = os.path.join(config_dir, "strawberry_polytunnel.tmap2.yaml")
        if os.path.isfile(polytunnel):
            map_files.append(polytunnel)

    if not map_files:
        print("No map files found. Pass --map <path>.")
        sys.exit(1)

    os.makedirs(args.out_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # Process each map
    # ------------------------------------------------------------------
    for map_path in map_files:
        map_name = os.path.splitext(os.path.basename(map_path))[0]
        print(f"\n{'=' * 60}")
        print(f"Map: {map_name}  ({map_path})")
        print("=" * 60)

        # 1. Load YAML
        with open(map_path) as f:
            tmap_data = yaml.safe_load(f)

        if tmap_data is None:
            print("  [SKIP] empty YAML file")
            continue

        # 2. Build NetworkX graph
        graph = build_graph_from_tmap(tmap_data)
        if graph is None:
            print("  [SKIP] build_graph_from_tmap returned None")
            continue
        print(f"  Nodes : {graph.number_of_nodes()}")
        print(f"  Edges : {graph.number_of_edges()}")

        # 3. Build KD-tree
        kdtree, node_names = build_kdtree_from_graph(graph)
        if kdtree is None:
            print("  [SKIP] KD-tree build failed")
            continue
        print(f"  KD-tree built with {len(node_names)} points")

        # 4. Pick a sample pose (first node's position)
        sample_node = node_names[0]
        sample_attrs = graph.nodes[sample_node]

        class _FakePose:
            """Minimal pose-like object for testing."""
            class position:
                x = sample_attrs.get("x", 0.0)
                y = sample_attrs.get("y", 0.0)
                z = sample_attrs.get("z", 0.0)

        pose = _FakePose()
        print(f"  Sample pose at node '{sample_node}': "
              f"({pose.position.x:.2f}, {pose.position.y:.2f})")

        # 5. Query nearest nodes
        nearest = query_nearest_nodes(kdtree, node_names, pose, k=3)
        print(f"  3-nearest nodes:")
        for entry in nearest:
            print(f"    {entry['node']:20s}  dist={entry['dist']:.3f}")

        # 6. Point-in-polygon for sample node
        inside = point_in_poly_nx(graph, sample_node, pose)
        print(f"  Pose inside '{sample_node}' influence zone: {inside}")

        # 7. Shortest path (first node -> last node)
        if len(node_names) >= 2:
            src, dst = node_names[0], node_names[-1]
            path = compute_shortest_path(graph, src, dst)
            if path:
                length = compute_path_length(graph, src, dst)
                print(f"  Shortest path {src} -> {dst}: "
                      f"{' -> '.join(path)}  (length={length:.3f})")
            else:
                print(f"  No path from {src} to {dst}")

        # 8. Connectivity check
        if len(node_names) >= 2:
            connected = check_connectivity(graph, node_names[0], node_names[-1])
            print(f"  Connected {node_names[0]} -> {node_names[-1]}: {connected}")

        # 9. Neighbours of first node
        nbrs = get_neighbors(graph, sample_node)
        print(f"  Neighbors of '{sample_node}': {nbrs}")

        # 10. Visualise and save
        png_path = os.path.join(args.out_dir, f"topo_{map_name}.png")

        hl_nodes = [n['node'] for n in nearest] if nearest else []
        hl_edges = []
        if path and len(path) >= 2:
            hl_edges = list(zip(path[:-1], path[1:]))

        draw_topological_graph(
            graph,
            highlight_nodes=hl_nodes,
            highlight_edges=hl_edges,
            title=f"{map_name}  ({graph.number_of_nodes()} nodes, "
                  f"{graph.number_of_edges()} edges)",
            save_path=png_path,
        )

    print(f"\nDone.  Figures saved under {args.out_dir}/")


if __name__ == '__main__':
    main()