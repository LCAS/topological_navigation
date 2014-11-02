#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap

from mongodb_store.message_store import MessageStoreProxy

class map_publisher(object):


    def __init__(self, name) :
        self.name = name
        self.nodes = self.loadMap(name)
        self.map_pub = rospy.Publisher('/topological_map', TopologicalMap, latch=True, queue_size=1)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)
        rospy.Subscriber('/update_map', std_msgs.msg.Time, self.updateCallback)


     
    def updateCallback(self, msg) :
#        if msg.data > self.last_updated :
        self.nodes = self.loadMap(self.name)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)        

    def loadMap(self, point_set) :
        msg_store = MessageStoreProxy(collection='topological_maps')
    
        query_meta = {}
        query_meta["pointset"] = point_set

        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0

        #print available

        if available <= 0 :
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")

        else :
            query_meta = {}
            query_meta["pointset"] = point_set
            
            
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
    
            points = TopologicalMap()
            points.name = self.name
            points.map = self.name
            points.pointset = point_set
            #string last_updated
            for i in message_list:
                b = i[0]
                points.nodes.append(b)
                
            return points