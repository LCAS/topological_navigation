#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
#from geometry_msgs.msg import Pose

#from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import *
from strands_navigation_msgs.srv import *


from mongodb_store.message_store import MessageStoreProxy


def node_dist(node1,node2):
    dist = math.sqrt((node1.pose.position.x - node2.pose.position.x)**2 + (node1.pose.position.y - node2.pose.position.y)**2 )
    return dist

class map_manager(object):

    def __init__(self, name) :
        self.name = name
        self.nodes = self.loadMap(name)
        self.names = self.create_list_of_nodes()
        
        self.map_pub = rospy.Publisher('/topological_map', strands_navigation_msgs.msg.TopologicalMap, latch=True, queue_size=1)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)
               
        rospy.Subscriber('/update_map', std_msgs.msg.Time, self.updateCallback)
        #This service returns any given map
        self.get_map_srv=rospy.Service('/topological_map_publisher/get_topological_map', strands_navigation_msgs.srv.GetTopologicalMap, self.get_topological_map_cb)
        #This service adds a node 
        self.get_map_srv=rospy.Service('/topological_map_publisher/add_topological_node', strands_navigation_msgs.srv.AddNode, self.add_topological_node_cb)

     
    def updateCallback(self, msg) :
#        if msg.data > self.last_updated :
        self.nodes = self.loadMap(self.name)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)        


    def get_topological_map_cb(self, req):
        nodes = self.loadMap(req.pointset)
        print "Returning Map %s"%req.pointset
        return nodes
        

    def get_new_name(self):        
        namesnum=[]
        for i in self.names :
            if i.startswith('WayPoint') :
                nam = i.strip('WayPoint')
                namesnum.append(int(nam))
        namesnum.sort()
        if namesnum:
            nodname = 'WayPoint%d'%(int(namesnum[-1])+1)
        else :
            nodname = 'WayPoint1'
        return nodname


    def add_edge(self, or_waypoint, de_waypoint, action) :

        rospy.loginfo('Adding Edge from '+or_waypoint+' to '+de_waypoint+' using '+action)
        node_name = or_waypoint

        #nodeindx = self._get_node_index(edged[0])
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : node_name, "pointset": self.nodes.name}
        query_meta = {}
        query_meta["pointset"] = self.nodes.name
        query_meta["map"] = self.nodes.map

        print query, query_meta
        available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
        print len(available)
        if len(available) == 1 :
            eids = []
            for i in available[0][0].edges :
                eids.append(i.edge_id)

            test=0
            eid = '%s_%s' %(or_waypoint, de_waypoint)
            while eid in eids:
                eid = '%s_%s_%3d' %(or_waypoint, de_waypoint, test)
                test += 1

            edge = strands_navigation_msgs.msg.Edge()
            edge.node = de_waypoint
            edge.action = action
            edge.top_vel = 0.55
            edge.edge_id = eid
            edge.map_2d = available[0][0].map

            available[0][0].edges.append(edge)
                     
            print available[0][0]
            msg_store.update(available[0][0], query_meta, query, upsert=True)
            return True
        else :
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))
            return False


    def add_topological_node_cb(self, req):
        dist = 8.0
        #Get New Node Name
        if req.name:
            name = req.name
        else:
            name = self.get_new_name()


        rospy.loginfo('Creating Node: '+name)        

        if name in self.names:
            rospy.logerr("Node already exists, try another name")
            return False
        
        #Create Message store
        msg_store = MessageStoreProxy(collection='topological_maps')

        meta = {}
        meta["map"] = self.nodes.map
        meta["pointset"] = self.nodes.name
        meta["node"] = name

        node = strands_navigation_msgs.msg.TopologicalNode()
        node.name = name
        node.map = self.nodes.map
        node.pointset = self.name
        node.pose = req.pose
        node.yaw_goal_tolerance = 0.1
        node.xy_goal_tolerance = 0.3
        node.localise_by_topic = ''
        vertices=[(0.69, 0.287), (0.287, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.287, -0.69), (0.69, -0.287)]
        for j in vertices :
            v = strands_navigation_msgs.msg.Vertex()
            v.x = float(j[0])
            v.y = float(j[1])
            node.verts.append(v)

        close_nodes = []
        for i in self.nodes.nodes :
            ndist = node_dist(node, i)
            if ndist < dist :
                if i.name != 'ChargingPoint' :
                    close_nodes.append(i.name)

        for i in close_nodes:
            e = strands_navigation_msgs.msg.Edge()
            e.node = i
            e.action = 'move_base'
            eid = '%s_%s' %(node.name, i)#get_edge_id(i, e.node, eids)
            #eids.append(eid)
            e.edge_id = eid
            e.top_vel =0.55
            e.map_2d = node.map
            node.edges.append(e)

#        print "=====   NEW NODE   ====="
#        print node
#
#        print "===   UPDATED NODES   ==="
        for i in close_nodes:
            self.add_edge(i, node.name, 'move_base')
#            print "+++++++++++++++++++++++"

        #Here I save the node        
        msg_store.insert(node,meta)

        return True


    def loadMap(self, point_set) :
        msg_store = MessageStoreProxy(collection='topological_maps')
    
        query_meta = {}
        query_meta["pointset"] = point_set

        # waiting for the map to be in the mongodb_store
        ntries=1
        map_found=False
        
        while not map_found :
            available = len(msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, {}, query_meta)) > 0
            #print available
            if available <= 0 :
                rospy.logerr("Desired pointset '"+point_set+"' not in datacentre, try :"+str(ntries))
                #rospy.logerr("Available pointsets: "+str(available))
                if ntries <=10 :
                    ntries+=1
                    rospy.sleep(rospy.Duration.from_sec(6))
                else :
                    raise Exception("Can't find waypoints.")
                    sys.exit(2)
            else:
                map_found=True
 
 
        query_meta = {}
        query_meta["pointset"] = point_set
              
        message_list = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, {}, query_meta)

        points = strands_navigation_msgs.msg.TopologicalMap()
        points.name = point_set
        #points.map = point_set
        points.pointset = point_set
        #string last_updated
        for i in message_list:
            b = i[0]
            points.nodes.append(b)
        
        points.map = points.nodes[0].map
        return points
    
    def create_list_of_nodes(self):
        names=[]
        for i in self.nodes.nodes :
            names.append(i.name)
        names.sort()
        return names