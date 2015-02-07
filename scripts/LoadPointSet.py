#!/usr/bin/env python

import json
import sys
import rospy


from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.msg import TopMap
from strands_navigation_msgs.msg import TopNode
from strands_navigation_msgs.msg import TopEdge
#from topological_utils.msg import vertex
#from topological_utils.msg import edge
#from geometry_msgs.msg import Pose
from mongodb_store.message_store import MessageStoreProxy



def loadMap(point_set) :
    msg_store = MessageStoreProxy(collection='topological_maps')

    query_meta = {}
    query_meta["pointset"] = point_set

    # waiting for the map to be in the mongodb_store
    ntries=1
    map_found=False
    
    while not map_found :
        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0
        #print available
        if available <= 0 :
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre, try :"+str(ntries))
            #rospy.logerr("Available pointsets: "+str(available))
            if ntries <=10 :
                ntries+=1
                rospy.sleep(rospy.Duration.from_sec(6))
            else :
                raise Exception("Can't find waypoints.")
        else:
            map_found=True
 
 
    query_meta = {}
    query_meta["pointset"] = point_set
          
    message_list = msg_store.query(TopologicalNode._type, {}, query_meta)

    points = TopologicalMap()
    points.name = point_set
    points.map = point_set
    points.pointset = point_set
    #string last_updated
    for i in message_list:
        b = i[0]
        points.nodes.append(b)
        
    return points



if __name__ == '__main__':
    #filename=str(sys.argv[1])
    point_set=str(sys.argv[1])
    #map_name=str(sys.argv[3])
    points = loadMap(point_set)
    print points
    tpoint = TopMap()
    tnode = TopNode()
    tpoint.name = points.name
    tpoint.map = points.map
    tpoint.pointset = points.pointset
    print "--------"   
    print points.nodes[0]
    tnode.name = points.nodes[0].name
    tnode.map = points.nodes[0].map
    tnode.pointset = points.nodes[0].pointset
    tnode.pose = points.nodes[0].pose
    tnode.verts = points.nodes[0].verts
    tnode.yaw_goal_tolerance = 0.1
    tnode.xy_goal_tolerance = 0.3

    
    tedges = []
    for i in points.nodes[0].edges :
        ted = TopEdge()
        ted.edge_id = "0001"
        ted.node = i.node
        ted.action = i.action
        ted.top_vel = 20
        ted.map_2d = "any"
        tedges.append(ted)
    
    tnode.edges = tedges
    tpoint.nodes.append(tnode)    
    
    print "--------"    
    print tpoint
    print "--------"

