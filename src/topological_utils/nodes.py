"""
Provides routines for working with topological map nodes
"""

from strands_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy
import queries
import rospy
import copy

def rename_node(name, new_name, top_map_name):
    """
    Renames a topological map node. All edges will be updated to reflect the change.

    @param name: the current name of the node to be changec
    @param new_name: the new name to give the node
    @param top_map_name: the name of the topological map to work with (pointset)

    @return (number of nodes changed, number of edges changed)
    """

    maps = queries.get_maps()
    if top_map_name not in maps:
        raise Exception("Unknown topological map.")

    msg_store = MessageStoreProxy(collection='topological_maps')
    
    nodes = msg_store.query(TopologicalNode._type, {}, {'pointset':top_map_name})
    node_renames = 0
    edge_changes = 0
    node_changes = 0

    if name not in nodes:
        raise Exception("No such node.")
    if new_name in nodes:
        raise Exception("New node name already in use.")

    old_metas = []
    for node, meta in nodes:
        old_metas.append(copy.deepcopy(meta))
        if meta["node"] == name:
            meta["node"] = new_name
        if node.name == name:
            node.name = new_name
            node_renames += 1
            if node_renames > 1:
                raise Exception("More than one node has the same name!")
        for edge in node.edges:
            if edge.node == name:
                edge.node = new_name
            if edge.edge_id.find(name) > 0:
                edge.edge_id = edge.edge_id.replace(name, new_name)
                edge_changes += 1
            
    # Save the changed nodes
    for ((node, meta), old_meta) in zip(nodes, old_metas):
        changed = msg_store.update(node, meta, {'name':old_meta['node'],
                                                'pointset':meta['pointset']})
        if changed.success:
            node_changes += 1

    return (node_changes, edge_changes)
