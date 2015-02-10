#!/usr/bin/env python

import json
import sys
import rospy


from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy

from topological_utils.queries import get_nodes
from topological_utils.srv import *

def handle_query_nodes(req):
    print "Queried for  %s "%(req.pointset)
    nodes_list = []
    nodes = get_nodes(req.pointset)
    resp = NodeMetadataResponse("")
    for node in nodes :
        nodes_list.append(node[0].name)
    resp.nodes = nodes_list
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


