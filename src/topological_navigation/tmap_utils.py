#!/usr/bin/env python
import math
import rospy


"""
    get_node
    
    Given a topological map and a node name it returns the node object
"""
def get_node(top_map, node_name):
    for i in top_map.nodes:
        if i.name == node_name:
            return i
    return None

 
"""
    get_distance_to_node
    
    Given two nodes it returns the straight line distance between them
""" 
def get_distance_to_node(nodea, nodeb):
    dist=math.hypot((nodeb.pose.position.x-nodea.pose.position.x),(nodeb.pose.position.y-nodea.pose.position.y))
    return dist


"""
    get_conected_nodes
    
    Given a node it returns the nodes connected to it by one single edge
"""
def get_conected_nodes(node):
    childs=[]
    for i in node.edges :
        childs.append(i.node)
    return childs


"""
    get_edges_between
    
    Given a node a it returns the connecting edges to node b
"""
def get_edges_between(top_map, nodea, nodeb):
     ab=[]
     noda = get_node(top_map, nodea)
     for j in noda.edges:
         if j.node == nodeb:
             ab.append(j)
     return ab


"""
    get_edge_from_id
    
    Given a node and the edge_id it returns the edges object
"""
def get_edge_from_id(top_map, node_name, edge_id):
    node = get_node(top_map, node_name)
    for i in node.edges:
        if i.edge_id == edge_id:
            return i
    return None