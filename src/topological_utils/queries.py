"""
Provides routines for querying the database for map information.
"""

from strands_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy
import rospy

def get_maps():
    """
    Queries the database and returns details of the available topological maps.
    :return: A dictionary where each key is the name of a topological map and each
    item is a dictionary of details. Details are:
       "number_nodes" ; integer
        "edge_actions" : list of action server names used for traversal
        "last_modified" : datetime.datetime object for the last time a node was inserted
    """
    maps = dict()
    msg_store = MessageStoreProxy(collection='topological_maps')

    nodes = msg_store.query(TopologicalNode._type)

    for node in nodes:
        pointset = node[1]["pointset"]
        if not maps.has_key(pointset):
            maps[pointset] = {"number_nodes": 0, "edge_actions": set(), "last_modified": ""}
        maps[pointset]["number_nodes"] += 1
        if (maps[pointset]["last_modified"] == "" or
                    node[1]["inserted_at"] > maps[pointset]["last_modified"]):
            maps[pointset]["last_modified"] = node[1]["inserted_at"]
        for edge in node[0].edges:
            maps[pointset]["edge_actions"].add(edge.action)

    return maps


def get_nodes(point_set):

    msg_store = MessageStoreProxy(collection="topological_maps")

    query_meta = {}
    query_meta["pointset"] = {}
    query_meta["inserted_by"] = {}
    query_meta["pointset"]['$regex'] = point_set
    #query_meta["inserted_by"] = "/unnamed"
    #query_meta["inserted_by"] = "{'$regex': 'unnamed'}"
    query_meta["inserted_by"]['$regex'] = 'unnamed'
    print query_meta

    #query_meta["stored_class"] = "strands_navigation_msgs/TopologicalNode"
    nodes = msg_store.query(TopologicalNode._type, {}, query_meta);
    available = len(nodes) > 0

    if available <= 0 :
        print "Desired pointset '"+point_set+"' not in datacentre"
        print "Available pointsets: "+str(available)
        raise Exception("Can't find waypoints.")

    return nodes
    
if __name__ == "__main__":
    get_maps()
