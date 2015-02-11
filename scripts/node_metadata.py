#!/usr/bin/env python

import json
import sys
import rospy


from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy

from topological_utils.queries import get_nodes
from topological_utils.srv import *

from random import randint

import uuid

def my_random_string(string_length=10):
    """Returns a random string of length string_length."""
    random = str(uuid.uuid4()) # Convert UUID format to a Python string.
    random = random.upper() # Make all characters uppercase.
    random = random.replace("-","") # Remove the UUID '-'.
    return random[0:string_length]

def handle_query_nodes(req):
    print "Queried for %s %s %s"%(req.map_name, req.pointset, req.meta_category)
    
    resp = NodeMetadataResponse()
    resp.name=[]
    resp.description = []
    resp.goto_node=[]
    resp.node_type = []
    
    nodes = get_nodes(req.map_name, req.pointset, req.meta_category)
    
    for node in nodes:
        print node[1]["node"]
        for gui_node in node[1]["contains"]: 
            if (gui_node["category"] == req.meta_category) :
                resp.name.append(gui_node["name"])
                resp.node_type.append(gui_node["category"])
                resp.goto_node.append(node[1]["node"])
                if gui_node.has_key("description"):
                    resp.description.append(gui_node["description"])
                else:
                    resp.description.append("")

    return resp;

def query_nodes_server():
    rospy.init_node('query_nodes_server')
    s = rospy.Service('query_nodes', NodeMetadata, handle_query_nodes)
    print "Ready to query the database for topological nodes."
    rospy.spin()

if __name__ == '__main__':
    #point_set=str(sys.argv[1])
    
    #get_nodes(point_set)
    query_nodes_server()


