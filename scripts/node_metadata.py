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
    print "Queried for  %s %s"%(req.pointset, req.meta_category)
    nodes_list = []
    #nodes = get_nodes(req.pointset)
    resp = NodeMetadataResponse()
    resp.name=[]
    resp.description = []
    resp.goto_node=[]
    resp.node_type = []
    #for node in nodes :
    #    nodes_list.append(node[0].name)
    #resp.nodes = nodes_list
    results = randint(1,5);
    for i in range(0, results) : 
        resp.name.append(my_random_string(8));
        resp.description.append("This is some really meaningful text");
        resp.goto_node.append(my_random_string(6));
        resp.node_type.append(req.meta_category);
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


