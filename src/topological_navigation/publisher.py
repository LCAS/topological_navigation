#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.srv import GetTopologicalMap

from mongodb_store.message_store import MessageStoreProxy


class map_publisher(object):

    def __init__(self, name) :
        self.name = name
        self.nodes = self.loadMap(name)
        self.map_pub = rospy.Publisher('/topological_map', TopologicalMap, latch=True, queue_size=1)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)
        rospy.Subscriber('/update_map', std_msgs.msg.Time, self.updateCallback)
        self.get_map_srv=rospy.Service('/topological_map_publisher/get_topological_map', GetTopologicalMap, self.get_topological_map_cb)


     
    def updateCallback(self, msg) :
#        if msg.data > self.last_updated :
        self.nodes = self.loadMap(self.name)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)        


    def get_topological_map_cb(self, req):
        nodes = self.loadMap(req.pointset)
        print "Returning Map %s"%req.pointset
        return nodes


    def loadMap(self, point_set) :
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