#!/usr/bin/env python
import math
import rospy
import warnings

from topological_navigation.topological_node import *
from topological_navigation_msgs.msg import TopologicalNode, Edge, Vertex


from mongodb_store.message_store import MessageStoreProxy
#import topological_navigation.msg


# from http://code.activestate.com/recipes/391367-deprecated/
def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""
    def newFunc(*args, **kwargs):
        warnings.warn("Call to deprecated function %s of topological_map. You should use functions from topological_navigation/manager.py instead." % func.__name__,
                      category=DeprecationWarning,
                      stacklevel=2)
        rospy.logwarn("Call to deprecated function {0} of topological_map. You should use functions from topological_navigation/manager.py instead.".format(func.__name__))
        return func(*args, **kwargs)
    newFunc.__name__ = func.__name__
    newFunc.__doc__ = func.__doc__
    newFunc.__dict__.update(func.__dict__)
    return newFunc

class topological_map(object):

    def __init__(self, name, msg = None):
        if msg is None:
            self.name = name
            self.nodes = self.loadMap(name)
        else:
            self.name = msg.pointset
            self.nodes = self.map_from_msg(msg.nodes)


    def _get_node_index(self, node_name):
        ind = -1
        counter = 0

        for i in self.nodes:
            if i.name == node_name:
                ind = counter
            counter+=1
        return ind


    @deprecated
    def update_node_waypoint(self, node_name, new_pose):
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name": node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            positionZ=available[0][0].pose.position.z
            available[0][0].pose = new_pose
            available[0][0].pose.position.z = positionZ
            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))

    @deprecated
    def update_node_vertex(self, node_name, vertex_index, vertex_pose):
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name": node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            #vertex_to_edit=available[0][0].verts[vertex_index]
            new_x_pos = -(available[0][0].pose.position.x - vertex_pose.position.x)
            new_y_pos = -(available[0][0].pose.position.y - vertex_pose.position.y)
            available[0][0].verts[vertex_index].x = new_x_pos
            available[0][0].verts[vertex_index].y = new_y_pos
            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))

    @deprecated
    def remove_edge(self, edge_name):
        #print 'removing edge: '+edge_name
        rospy.loginfo('Removing Edge: '+edge_name)
        edged = edge_name.split('_')
        #print edged
        node_name = edged[0]
        #nodeindx = self._get_node_index(edged[0])
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name": node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            for i in available[0][0].edges:
                #print i.node
                if i.node == edged[1]:
                    available[0][0].edges.remove(i)
            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))

    @deprecated
    def update_edge(self, node_name, edge_id, new_action=None, new_top_vel=None):
        msg_store = MessageStoreProxy(collection='topological_maps')
        # The query retrieves the node name with the given name from the given pointset.
        query = {"name": node_name, "pointset": self.name}
        # The meta-information is some additional information about the specific
        # map that we are interested in (?)
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        # This returns a tuple containing the object, if it exists, and some
        # information about how it's stored in the database.
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            for edge in available[0][0].edges:
                if edge.edge_id == edge_id:
                    edge.action = new_action or edge.action
                    edge.top_vel = new_top_vel or edge.top_vel
                
            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))

    @deprecated
    def update_node_name(self, node_name, new_name):
        msg_store = MessageStoreProxy(collection='topological_maps')
        # The query retrieves the node name with the given name from the given pointset.
        query = {"name": node_name, "pointset": self.name}
        # The meta-information is some additional information about the specific
        # map that we are interested in (?)
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        # This returns a tuple containing the object, if it exists, and some
        # information about how it's stored in the database.
        available = msg_store.query(TopologicalNode._type, query, query_meta)

        if len(available) == 1:
            available[0][0].name = new_name
            # Also need to update all edges which involve the renamed node
            allnodes_query = {"pointset": self.name}
            allnodes_query_meta = {}
            allnodes_query_meta["pointset"] = self.name
            allnodes_query_meta["map"] = self.map
            # this produces a list of tuples, each with [0] as the node, [1] as database info
            allnodes_available = msg_store.query(TopologicalNode._type, {}, allnodes_query_meta)
            
            # Check the edges of each node for a reference to the node to be
            # renamed, and change the edge id if there is one. Enumerate the
            # values so that we can edit the objects in place to send them back
            # to the database.
            for node_ind, node_tpl in enumerate(allnodes_available):
                for edge_ind, edge in enumerate(node_tpl[0].edges):
                    # change names of the edges in other nodes, and update their values in the database
                    if node_tpl[0].name != node_name and node_name in edge.edge_id:
                        allnodes_available[node_ind][0].edges[edge_ind].edge_id = edge.edge_id.replace(node_name, new_name)
                        # must also update the name of the node this edge goes to
                        allnodes_available[node_ind][0].edges[edge_ind].node = new_name
                        curnode_query = {"name": node_tpl[0].name, "pointset": self.name}
                        msg_store.update(allnodes_available[node_ind][0], allnodes_query_meta, curnode_query, upsert=True)

            # update all edge ids for this node
            for edge_ind, edge in enumerate(available[0][0].edges):
                available[0][0].edges[edge_ind].edge_id = edge.edge_id.replace(node_name, new_name)

            msg_store.update(available[0][0], query_meta, query, upsert=True)
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))

    @deprecated
    def add_edge(self, or_waypoint, de_waypoint, action):
        #print 'removing edge: '+edge_name
        rospy.loginfo('Adding Edge from '+or_waypoint+' to '+de_waypoint+' using '+action)
        node_name = or_waypoint
        #nodeindx = self._get_node_index(edged[0])
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name": node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            found =False
            for i in available[0][0].edges:
                #print i.node
                if i.node == de_waypoint:
                    found=True
                    break
            if not found:
                edge = Edge()
                edge.edge_id = "{0}_{1}".format(or_waypoint, de_waypoint)
                edge.node = de_waypoint
                edge.action = action
                edge.top_vel = 0.55
                edge.map_2d = available[0][0].map
                available[0][0].edges.append(edge)
                msg_store.update(available[0][0], query_meta, query, upsert=True)
            else:
                rospy.logerr("Edge already exist: Try deleting it first")
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))

    @deprecated
    def remove_node(self, node_name):
        rospy.loginfo('Removing Node: '+node_name)
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name": node_name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.map

        available = msg_store.query(TopologicalNode._type, query, query_meta)

        node_found = False
        if len(available) == 1:
            node_found = True
            rm_id = str(available[0][1]['_id'])
            print(rm_id)
        else:
            rospy.logerr("Node not found "+str(len(available))+" waypoints found after query")
            #rospy.logerr("Available data: "+str(available))


        if node_found:
            query_meta = {}
            query_meta["pointset"] = self.name
            edges_to_rm = []
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
            for i in message_list:
                for j in i[0].edges:
                    if j.node == node_name:
                        edge_rm = i[0].name+'_'+node_name
                        edges_to_rm.append(edge_rm)

            for k in edges_to_rm:
                print('remove: '+k)
                self.remove_edge(k)
            msg_store.delete(rm_id)

    @deprecated
    def add_node(self, name, dist, pos, std_action):
        rospy.loginfo('Creating Node: '+name)
        msg_store = MessageStoreProxy(collection='topological_maps')

        found = False
        for i in self.nodes:
            if i.name == name:
                found = True

        if found:
            rospy.logerr("Node already exists, try another name")
        else:
            rospy.loginfo('Adding node: '+name)

            meta = {}
            meta["map"] = self.map
            meta["pointset"] = self.name
            meta["node"] = name

            node = TopologicalNode()
            node.name = name
            node.map = self.map
            node.pointset = self.name
            node.pose = pos
            vertices=[(0.69, 0.287), (0.287, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.287, -0.69), (0.69, -0.287)]
            for j in vertices:
                v = Vertex()
                v.x = float(j[0])
                v.y = float(j[1])
                node.verts.append(v)


            cx = node.pose.position.x
            cy = node.pose.position.y
            close_nodes = []
            for i in self.nodes:
                ndist = i._get_distance(cx, cy)
                if ndist < dist:
                    if i.name != 'ChargingPoint':
                        close_nodes.append(i.name)

            for i in close_nodes:
                edge = Edge()
                edge.node = i
                edge.action = std_action
                node.edges.append(edge)

            msg_store.insert(node,meta)

            for i in close_nodes:
                self.add_edge(i, name, std_action)

        # need to reload the map when a node is added, for consistency
        self.loadMap(self.name)

    @deprecated
    def delete_map(self):

        rospy.loginfo('Deleting map: '+self.name)
        msg_store = MessageStoreProxy(collection='topological_maps')

        query_meta = {}
        query_meta["pointset"] = self.name

        message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
        for i in message_list:
            rm_id = str(i[1]['_id'])
            msg_store.delete(rm_id)

    @deprecated
    def map_from_msg(self, nodes):
        #self.topol_map = msg.pointset
        points = []
        for i in nodes:
            self.map = i.map
            b = topological_node(i.name)
            edges = []
            for j in i.edges:
                data = {}
                data["node"]=j.node
                data["action"]=j.action
                edges.append(data)
            b.edges = edges
            verts = []
            for j in i.verts:
                data = [j.x,j.y]
                verts.append(data)
            b._insert_vertices(verts)
            c=i.pose
            waypoint=[str(c.position.x), str(c.position.y), str(c.position.z), str(c.orientation.x), str(c.orientation.y), str(c.orientation.z), str(c.orientation.w)]
            b.waypoint = waypoint
            b._get_coords()
            points.append(b)

        return points

    @deprecated
    def loadMap(self, point_set):
        msg_store = MessageStoreProxy(collection='topological_maps')

        query_meta = {}
        query_meta["pointset"] = point_set

        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0

        if available <= 0:
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")

        else:
            query_meta = {}
            query_meta["pointset"] = point_set

            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)

            points = []
            for i in message_list:
                self.map = i[0].map
                b = topological_node(i[0].name)
                edges = []
                for j in i[0].edges:
                    data = {}
                    data["node"]=j.node
                    data["action"]=j.action
                    edges.append(data)
                b.edges = edges

                verts = []
                for j in i[0].verts:
                    data = [j.x,j.y]
                    verts.append(data)
                b._insert_vertices(verts)

                c=i[0].pose
                waypoint=[str(c.position.x), str(c.position.y), str(c.position.z), str(c.orientation.x), str(c.orientation.y), str(c.orientation.z), str(c.orientation.w)]
                b.waypoint = waypoint
                b._get_coords()

                points.append(b)

            return points
