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
    print "Queried for %s %s %s"%(req.map_name, req.pointset, req.meta_category)
    
    resp = NodeMetadataResponse()
    resp.name=[]
    resp.description = []
    resp.goto_node=[]
    resp.node_type = []
    
    nodes = get_nodes(req.map_name, req.pointset, req.meta_category)
    
    for node in nodes:
        for gui_node in node[1]["contains"]: 
            if (gui_node["category"] == req.meta_category) :
                resp.name.append(gui_node["name"])
                resp.node_type.append(gui_node["category"])
                resp.goto_node.append(node[1]["node"])
                if gui_node.has_key("description"):
                    resp.description.append(gui_node["description"])
                else:
                    resp.description.append("")
                
                if gui_node.has_key("at_node"):
                    resp.at_node.append(gui_node["at_node"])
                else:
                    resp.at_node.append(False)

    print "Returning ",len(resp.name)," nodes for the meta_query ",req.meta_category
    return resp;

def query_nodes_server():
    rospy.init_node('query_nodes_server')
    s = rospy.Service('query_node_metadata', NodeMetadata, handle_query_nodes)
    print "Ready to query the database for topological nodes."
    rospy.spin()

if __name__ == '__main__':
    query_nodes_server()


