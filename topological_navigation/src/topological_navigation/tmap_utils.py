#!/usr/bin/env python
import math


def get_node(top_map, node_name):
    """
    Given a topological map and a node name it returns the node object
    """
    for i in top_map.nodes:
        if i.name == node_name:
            return i
    return None


def get_node_from_tmap2(top_map, node_name):
    """
    Given a topological map 2 and a node name it returns the node object
    """
    for i in top_map["nodes"]:
        if i["node"]["name"] == node_name:
            return i
    return None


def get_distance(pose1, pose2):
    """
    Returns the straight line distance between two poses
    """
    return math.hypot((pose1.position.x - pose2.position.x), (pose1.position.y - pose2.position.y))


def get_distance_node_pose(node, pose):
    """
    Returns the straight line distance between a pose and a node
    """
    return math.hypot((pose.position.x - node.pose.position.x), (pose.position.y - node.pose.position.y))


def get_distance_node_pose_from_tmap2(node, pose):
    dist = math.hypot((pose.position.x - node["node"]["pose"]["position"]["x"]),
                      (pose.position.y - node["node"]["pose"]["position"]["y"]))
    return dist


def get_distance_to_node(nodea, nodeb):
    """
    Given two nodes it returns the straight line distance between them
    """
    dist = math.hypot((nodeb.pose.position.x - nodea.pose.position.x),
                      (nodeb.pose.position.y - nodea.pose.position.y))
    return dist


def get_distance_to_node_tmap2(nodea, nodeb):
    """
    Given two nodes it returns the straight line distance between them for tmap2
    """
    dist = math.hypot((nodeb["node"]["pose"]["position"]["x"] - nodea["node"]["pose"]["position"]["x"]),
                      (nodeb["node"]["pose"]["position"]["y"] - nodea["node"]["pose"]["position"]["y"]))
    return dist


def get_conected_nodes(node):
    """
    Given a node it returns the nodes connected to it by one single edge
    """
    childs = []
    for i in node.edges:
        childs.append(i.node)
    return childs


def get_conected_nodes_tmap2(node):
    """
    Given a node it returns the nodes connected to it by one single edge
    """
    childs = []
    for i in node["node"]["edges"]:
        childs.append(i["node"])
    return childs


def get_edges_between(top_map, nodea, nodeb):
    """
    Given a node a it returns the connecting edges to node b
    """
    ab = []
    noda = get_node(top_map, nodea)
    for j in noda.edges:
        if j.node == nodeb:
            ab.append(j)
    return ab


def get_edges_between_tmap2(top_map, nodea, nodeb):
    """
    Given a node a it returns the connecting edges to node b
    """
    ab = []
    noda = get_node_from_tmap2(top_map, nodea)
    for j in noda["node"]["edges"]:
        if j["node"] == nodeb:
            ab.append(j)
    return ab


def get_edge_from_id(top_map, node_name, edge_id):
    """
    Given a node and the edge_id it returns the edges object
    """
    node = get_node(top_map, node_name)
    if node:
        for i in node.edges:
            if i.edge_id == edge_id:
                return i
    return None


def get_edge_from_id_tmap2(top_map, node_name, edge_id):
    """
    Given a node and the edge_id it returns the edges object for topomap 2
    """
    node = get_node_from_tmap2(top_map, node_name)
    if node:
        for i in node["node"]["edges"]:
            if i["edge_id"] == edge_id:
                return i
    return None


def get_node_names_from_edge_id(top_map, edge_id):
    """
    Given a tmap and edge id it returns the names of the edge's origin and destination nodes
    """
    for node in top_map.nodes:
        origin = node.name
        for edge in node.edges:
            destination = edge.node
            if edge.edge_id == edge_id:
                return origin, destination
    return None,None


def get_node_names_from_edge_id_2(top_map, edge_id):
    """
    Given a tmap2 and edge id it returns the names of the edge's origin and destination nodes
    """
    for node in top_map["nodes"]:
        origin = node["node"]["name"]
        for edge in node["node"]["edges"]:
            destination = edge["node"]
            if edge["edge_id"] == edge_id:
                return origin, destination
    return None,None
#########################################################################################################