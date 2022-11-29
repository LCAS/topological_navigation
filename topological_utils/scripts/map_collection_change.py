#!/usr/bin/env python

import json
import sys
import rospy
from geometry_msgs.msg import Pose
from topological_navigation_msgs.msg import TopologicalNode, Vertex, Edge

import pymongo
#import mongodb_store.util
from mongodb_store.message_store import MessageStoreProxy





if __name__ == '__main__':
    #if len(sys.argv) < 4 :
    #    print "usage: insert_map input_file.txt dataset_name map_name"
    #	sys.exit(2)
    
    #filename=str(sys.argv[1])
    #dataset_name=str(sys.argv[2])
    #map_name=str(sys.argv[3])
    
    msg_store = MessageStoreProxy()
    msg_store_maps = MessageStoreProxy(collection='topological_maps')
    
    
    query_meta = {}
    query_meta["stored_type"] = "topological_navigation_msgs/TopologicalNode"
    
    available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0
    
    #print available
    
    if available <= 0 :
        #rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
        #rospy.logerr("Available pointsets: "+str(available))
        raise Exception("Can't find waypoints.")

    else :
        message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
        for i in message_list:
            #print i
            meta = {}
            meta["node"] = i[0].name
            meta["map"] = i[0].map
            meta["pointset"] = i[0].pointset
            available = len(msg_store_maps.query(TopologicalNode._type, {}, meta))
            if available == 0 :
                msg_store_maps.insert(i[0],meta)
            else : 
                rospy.logerr("this point is already in datacentre:")
                print meta