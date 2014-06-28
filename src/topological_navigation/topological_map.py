#!/usr/bin/env python
import math
import rospy

from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode


from ros_datacentre.message_store import MessageStoreProxy
#import topological_navigation.msg


class topological_map(object):

    def __init__(self, name):
        self.name = name
        self.nodes = self.loadMap(name)


    def _get_node_index(self, node_name):
        ind = -1
        counter = 0
       
        for i in self.nodes :
            if i.name == node_name :
                ind = counter
            counter+=1
        return ind


    def update_node_waypoint(self, node_name, new_pose) :
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1 :
            positionZ=available[0][0].pose.position.z
            available[0][0].pose = new_pose
            available[0][0].pose.position.z = positionZ
            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else :
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))


    def update_node_vertex(self, node_name, vertex_index, vertex_pose) :
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1 :
            #vertex_to_edit=available[0][0].verts[vertex_index]
            new_x_pos = -(available[0][0].pose.position.x - vertex_pose.position.x)
            new_y_pos = -(available[0][0].pose.position.y - vertex_pose.position.y)
            available[0][0].verts[vertex_index].x = new_x_pos
            available[0][0].verts[vertex_index].y = new_y_pos
            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else :
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))


    def remove_edge(self, edge_name) :
        print 'removing edge: '+edge_name
        edged = edge_name.split('_')
        print edged
        node_name = edged[0]
        #nodeindx = self._get_node_index(edged[0])
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1 :
            for i in available[0][0].edges :
                print i.node
                if i.node == edged[1] :
                    available[0][0].edges.remove(i)
            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else :
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))        
        
        

    def loadMap(self, point_set):
        msg_store = MessageStoreProxy(collection='topological_maps')
    
        query_meta = {}
        query_meta["pointset"] = point_set

        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0

        print available

        if available <= 0 :
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
    
        else :
            query_meta = {}
            query_meta["pointset"] = point_set
            
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
    
            points = []
            for i in message_list:
                self.map = i[0].map
                b = topological_node(i[0].name)
                edges = []
                for j in i[0].edges :
                    data = {}
                    data["node"]=j.node
                    data["action"]=j.action
                    edges.append(data)
                b.edges = edges
                
                verts = []
                for j in i[0].verts :
                    data = [j.x,j.y]
                    verts.append(data)
                b._insert_vertices(verts)
    
                c=i[0].pose
                waypoint=[str(c.position.x), str(c.position.y), str(c.position.z), str(c.orientation.x), str(c.orientation.y), str(c.orientation.z), str(c.orientation.w)]
                b.waypoint = waypoint
                b._get_coords()
    
                points.append(b)
            
            return points
