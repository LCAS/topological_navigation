#!/usr/bin/env python
import math

# import rospy


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
    get_node_from_tmap2
    
    Given a topological map 2 and a node name it returns the node object
"""


def get_node_from_tmap2(top_map, node_name):
    for i in top_map["nodes"]:
        if i["node"]["name"] == node_name:
            return i
    return None


"""
    get_distance
    
    Returns the straight line distance between two poses
"""


def get_distance(pose1, pose2):
    # dist=math.hypot((pose.position.x-node.pose[0].position.x),(pose.position.y-node.pose[0].position.y))
    dist = math.hypot(
        (pose1.position.x - pose2.position.x), (pose1.position.y - pose2.position.y)
    )
    return dist


"""
    get_distance_node_pose
    
    Returns the straight line distance between a pose and a node
"""


def get_distance_node_pose(node, pose):
    # dist=math.hypot((pose.position.x-node.pose[0].position.x),(pose.position.y-node.pose[0].position.y))
    dist = math.hypot(
        (pose.position.x - node.pose.position.x),
        (pose.position.y - node.pose.position.y),
    )
    return dist


def get_distance_node_pose_from_tmap2(node, pose):
    dist = math.hypot(
        (pose.position.x - node["node"]["pose"]["position"]["x"]),
        (pose.position.y - node["node"]["pose"]["position"]["y"]),
    )
    return dist


"""
    get_distance_to_node
    
    Given two nodes it returns the straight line distance between them
"""


def get_distance_to_node(nodea, nodeb):
    dist = math.hypot(
        (nodeb.pose.position.x - nodea.pose.position.x),
        (nodeb.pose.position.y - nodea.pose.position.y),
    )
    return dist


"""
    get_distance_to_node_tmap2
    
    Given two nodes it returns the straight line distance between them for tmap2
"""


def get_distance_to_node_tmap2(nodea, nodeb):
    dist = math.hypot(
        (nodeb["node"]["pose"]["position"]["x"] - nodea["node"]["pose"]["position"]["x"]),
        (nodeb["node"]["pose"]["position"]["y"] - nodea["node"]["pose"]["position"]["y"]),
    )
    return dist


"""
    get_conected_nodes
    
    Given a node it returns the nodes connected to it by one single edge
"""


def get_conected_nodes(node):
    childs = []
    for i in node.edges:
        childs.append(i.node)
    return childs


"""
    get_conected_nodes_tmap2
    
    Given a node it returns the nodes connected to it by one single edge
"""


def get_conected_nodes_tmap2(node):
    childs = []
    for i in node["node"]["edges"]:
        childs.append(i["node"])
    return childs


"""
    get_edges_between
    
    Given a node a it returns the connecting edges to node b
"""


def get_edges_between(top_map, nodea, nodeb):
    ab = []
    noda = get_node(top_map, nodea)
    for j in noda.edges:
        if j.node == nodeb:
            ab.append(j)
    return ab


"""
    get_edges_between_tmap2
    
    Given a node a it returns the connecting edges to node b
"""


def get_edges_between_tmap2(top_map, nodea, nodeb):
    ab = []
    noda = get_node_from_tmap2(top_map, nodea)
    for j in noda["node"]["edges"]:
        if j["node"] == nodeb:
            ab.append(j)
    return ab


"""
    get_edge_from_id
    
    Given a node and the edge_id it returns the edges object
"""


def get_edge_from_id(top_map, node_name, edge_id):
    node = get_node(top_map, node_name)
    if node:
        for i in node.edges:
            if i.edge_id == edge_id:
                return i
    return None


"""
    get_edge_from_id_tmap2
    
    Given a node and the edge_id it returns the edges object for topomap 2
"""


def get_edge_from_id_tmap2(top_map, node_name, edge_id):
    node = get_node_from_tmap2(top_map, node_name)
    if node:
        for i in node["node"]["edges"]:
            if i["edge_id"] == edge_id:
                return i
    return None