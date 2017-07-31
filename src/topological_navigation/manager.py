#!/usr/bin/env python
import math
import rospy
import sys
import pymongo
import json

import std_msgs.msg

from strands_navigation_msgs.msg import *
from strands_navigation_msgs.srv import *
from mongodb_store.message_store import MessageStoreProxy



def node_dist(node1,node2):
    dist = math.sqrt((node1.pose.position.x - node2.pose.position.x)**2 + (node1.pose.position.y - node2.pose.position.y)**2 )
    return dist

class map_manager(object):

    def __init__(self, name, load=True) :
        self.name = name

        if load:
            self.nodes = self.loadMap(name)
            self.names = self.create_list_of_nodes()

            rospy.set_param('topological_map_name', self.nodes.pointset)
        else:
            self.nodes = strands_navigation_msgs.msg.TopologicalMap()
            self.nodes.name = name
            self.nodes.pointset = name
            self.names=[]
            rospy.set_param('topological_map_name', self.nodes.pointset)


        self.map_pub = rospy.Publisher('/topological_map', strands_navigation_msgs.msg.TopologicalMap, latch=True, queue_size=1)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)

        rospy.Subscriber('/update_map', std_msgs.msg.Time, self.updateCallback)
        #This service returns any given map
        self.get_map_srv=rospy.Service('/topological_map_publisher/get_topological_map', strands_navigation_msgs.srv.GetTopologicalMap, self.get_topological_map_cb)
        #This service switches topological map
        self.switch_map_srv=rospy.Service('/topological_map_manager/switch_topological_map', strands_navigation_msgs.srv.GetTopologicalMap, self.switch_topological_map_cb)
        #This service adds a node 
        self.add_node_srv=rospy.Service('/topological_map_manager/add_topological_node', strands_navigation_msgs.srv.AddNode, self.add_topological_node_cb)
        #This service deletes a node 
        self.remove_node_srv=rospy.Service('/topological_map_manager/remove_topological_node', strands_navigation_msgs.srv.RmvNode, self.remove_node_cb)
        #This service adds content to a node
        self.add_content_to_node_srv=rospy.Service('/topological_map_manager/add_content_to_node', strands_navigation_msgs.srv.AddContent, self.add_content_cb)
        self.update_node_name_srv = rospy.Service("/topological_map_manager/update_node_name", strands_navigation_msgs.srv.UpdateNodeName, self.update_node_name_cb)
        self.update_node_waypoint_srv = rospy.Service("/topological_map_manager/update_node_pose", strands_navigation_msgs.srv.AddNode, self.update_node_waypoint_cb)
        self.update_node_tolerance_srv = rospy.Service("/topological_map_manager/update_node_tolerance", strands_navigation_msgs.srv.UpdateNodeTolerance, self.update_node_tolerance_cb)
        #This service adds a tag to the meta information of a list of nodes
        self.get_tag_srv=rospy.Service('/topological_map_manager/get_tags', strands_navigation_msgs.srv.GetTags, self.get_tags_cb)
        #This service adds gets all tags from the meta information of a node
        self.get_node_tag_srv=rospy.Service('/topological_map_manager/get_node_tags', strands_navigation_msgs.srv.GetNodeTags, self.get_node_tags_cb)
        #This service adds gets all tags from the meta information of a node
        self.modify_tag_srv=rospy.Service('/topological_map_manager/modify_node_tags', strands_navigation_msgs.srv.ModifyTag, self.modify_tag_cb)
        #This service adds a tag to the meta information of a list of nodes
        self.add_tag_srv=rospy.Service('/topological_map_manager/add_tag_to_node', strands_navigation_msgs.srv.AddTag, self.add_tag_cb)
        #This service removes a tag from the meta information of a list of nodes
        self.rm_tag_srv=rospy.Service('/topological_map_manager/rm_tag_from_node', strands_navigation_msgs.srv.AddTag, self.rm_tag_cb)        
        #This service returns a list of nodes that have a given tag
        self.get_tagged_srv=rospy.Service('/topological_map_manager/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes, self.get_tagged_cb)       
        #This service returns a list of edges_ids between two nodes
        self.get_node_edges_srv=rospy.Service('/topological_map_manager/get_edges_between_nodes', strands_navigation_msgs.srv.GetEdgesBetweenNodes, self.get_edges_between_cb)
        #adds edge between two nodes
        self.add_edges_srv=rospy.Service('/topological_map_manager/add_edges_between_nodes', strands_navigation_msgs.srv.AddEdge, self.add_edge_cb)
        self.update_edge_srv=rospy.Service('/topological_map_manager/update_edge', strands_navigation_msgs.srv.UpdateEdge, self.update_edge_cb)
        self.remove_edge_srv=rospy.Service('/topological_map_manager/remove_edge', strands_navigation_msgs.srv.AddEdge, self.remove_edge_cb)

    def updateCallback(self, msg) :
#        if msg.data > self.last_updated :
        self.nodes = self.loadMap(self.name)
        self.last_updated = rospy.Time.now()
        self.map_pub.publish(self.nodes)
        self.names = self.create_list_of_nodes()


    def get_tags_cb(self, req):
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        client = pymongo.MongoClient(host, port)

        db=client.message_store
        collection=db["topological_maps"]
        available = collection.find({"pointset": self.nodes.name}).distinct("_meta.tag")
        tt=[]
        #for i in available:
        tt.append(available)
        return tt

    def get_node_tags_cb(self, req):
        #rospy.loginfo('Adding Tag '+msg.tag+' to '+str(msg.node))
        succeded = True
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : req.node_name, "pointset": self.nodes.name}
        query_meta = {}
        query_meta["pointset"] = self.nodes.name
        query_meta["map"] = self.nodes.map

        #print query, query_meta
        available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
        #print len(available)
        if len(available) == 1:
            # The meta information for a node is in the second part of the tuple
            # returned by the message store query
            if 'tag' in available[0][1]:
                tags = available[0][1]['tag']
            else:
                tags = []
        else:
             succeded = False
             tags = []

        return succeded, tags

    def get_tagged_nodes(self, tag):
        mm=[]
        a=[]

        #db.topological_maps.find({ "_meta.tag":"AAA" })

        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"_meta.tag": tag, "pointset": self.nodes.name}
        query_meta = {}
        query_meta["pointset"] = self.nodes.name
        query_meta["map"] = self.nodes.map

        #print query, query_meta
        available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
        #print len(available)
        for i in available:
            nname= i[1]['node']
            a.append(nname)

        mm.append(a)

        return mm


    def get_tagged_cb(self, msg):
        return self.get_tagged_nodes(msg.tag)


    def add_content_cb(self, req):
        #print req
        data = json.loads(req.content)
        #print data

        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : req.node, "pointset": self.nodes.name}
        query_meta = {}
        query_meta["pointset"] = self.nodes.name
        query_meta["map"] = self.nodes.map

        #print query, query_meta
        available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
        #print len(available)
        if len(available) != 1:
             succeded = False
             meta_out = None
             print 'there are no nodes or more than 1 with that name'
        else:
            succeded = True
            for i in available:
                msgid= i[1]['_id']
                if 'contains' in i[1]:
                    if type(data) is list :
                        for j in data:
                            if 'category' in j and 'name' in j :
                                i[1]['contains'].append(j)
                    elif type(data) is dict :
                        if 'category' in data and 'name' in data :
                            i[1]['contains'].append(data)
                else:
                    a=[]
                    if type(data) is list :
                        for j in data:
                            if 'category' in j and 'name' in j :
                                a.append(j)
                    elif type(data) is dict :
                        if 'category' in data and 'name' in data :
                            a.append(data)
                    i[1]['contains']=a
                meta_out = str(i[1])
                print "Updating %s--%s" %(i[0].pointset, i[0].name)
                msg_store.update_id(msgid, i[0], i[1], upsert = False)

        return succeded, meta_out


    def add_tag_cb(self, msg):
        #rospy.loginfo('Adding Tag '+msg.tag+' to '+str(msg.node))
        succeded = True
        meta_out = None
        for j in msg.node:

            msg_store = MessageStoreProxy(collection='topological_maps')
            query = {"name" : j, "pointset": self.nodes.name}
            query_meta = {}
            query_meta["pointset"] = self.nodes.name
            query_meta["map"] = self.nodes.map

            #print query, query_meta
            available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
            #print len(available)
            for i in available:
                msgid= i[1]['_id']
                if 'tag' in i[1]:
                    if not msg.tag in i[1]['tag']:
                        i[1]['tag'].append(msg.tag)
                else:
                    a=[]
                    a.append(msg.tag)
                    i[1]['tag']=a
                meta_out = str(i[1])

                msg_store.update_id(msgid, i[0], i[1], upsert = False)
                #print trstr
            if len(available) == 0:
                 succeded = False

        return succeded, meta_out


    def rm_tag_cb(self, msg):
        #rospy.loginfo('Adding Tag '+msg.tag+' to '+str(msg.node))
        succeded = True
        for j in msg.node:

            msg_store = MessageStoreProxy(collection='topological_maps')
            query = {"name" : j, "pointset": self.nodes.name}
            query_meta = {}
            query_meta["pointset"] = self.nodes.name
            query_meta["map"] = self.nodes.map

            #print query, query_meta
            available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
            #print len(available)
            succeded = False
            meta_out = None
            for i in available:
                msgid= i[1]['_id']
                if 'tag' in i[1]:
                    if msg.tag in i[1]['tag']:
                        print 'removing tag'
                        i[1]['tag'].remove(msg.tag)
                        print 'new list of tags'
                        print i[1]['tag']
                        msg_store.update_id(msgid, i[0], i[1], upsert = False)
                        succeded = True
                meta_out = str(i[1])

        return succeded, meta_out

    def modify_tag_cb(self, msg):
        succeded = True
        meta_out = None
        for node in msg.node:
            msg_store = MessageStoreProxy(collection='topological_maps')
            query = {"name" : node, "pointset": self.nodes.name}
            query_meta = {}
            query_meta["pointset"] = self.nodes.name
            query_meta["map"] = self.nodes.map

            #print query, query_meta
            available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
            #print len(available)
            for node_plus_meta in available:
                msgid= node_plus_meta[1]['_id']
                if 'tag' in node_plus_meta[1]:
                    if not msg.tag in node_plus_meta[1]['tag']:
                        continue
                    else:
                        tag_ind = node_plus_meta[1]['tag'].index(msg.tag)
                        node_plus_meta[1]['tag'][tag_ind] = msg.new_tag
                meta_out = str(node_plus_meta[1])

                msg_store.update_id(msgid, node_plus_meta[0], node_plus_meta[1], upsert = True)
            if len(available) == 0:
                 succeded = False

        return succeded, meta_out

    def get_topological_map_cb(self, req):
        nodes = self.loadMap(req.pointset)
        print "Returning Map %s"%req.pointset
        nodes.nodes.sort(key=lambda node: node.name)
        return nodes


    def switch_topological_map_cb(self, req):
        self.nodes=[]
        self.name = req.pointset
        self.nodes = self.loadMap(req.pointset)
        print "Returning Map %s"%req.pointset
        #nodes.nodes.sort(key=lambda node: node.name)
        self.names = self.create_list_of_nodes()
        self.map_pub.publish(self.nodes)
        return self.nodes


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


    def add_edge_cb(self, req):
        return self.add_edge(req.origin, req.destination, req.action, req.edge_id)

    def add_edge(self, or_waypoint, de_waypoint, action, edge_id) :

        rospy.loginfo('Adding Edge from '+or_waypoint+' to '+de_waypoint+' using '+action)
        node_name = or_waypoint

        #nodeindx = self._get_node_index(edged[0])
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : node_name, "pointset": self.nodes.name}
        query_meta = {}
        query_meta["pointset"] = self.nodes.name
        query_meta["map"] = self.nodes.map

        #print query, query_meta
        available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
        #print len(available)
        if len(available) == 1 :
            eids = []
            for i in available[0][0].edges :
                eids.append(i.edge_id)

            if not edge_id or edge_id in eids:
                test=0
                eid = '%s_%s' %(or_waypoint, de_waypoint)
                while eid in eids:
                    eid = '%s_%s_%03d' %(or_waypoint, de_waypoint, test)
                    test += 1
            else:
                eid=edge_id

            edge = strands_navigation_msgs.msg.Edge()
            edge.node = de_waypoint
            edge.action = action
            edge.top_vel = 0.55
            edge.edge_id = eid
            edge.map_2d = available[0][0].map

            available[0][0].edges.append(edge)

            #print available[0][0]
            msg_store.update(available[0][0], query_meta, query, upsert=True)
            return True
        else :
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))
            return False



    def add_topological_node_cb(self, req):
        res=self.add_topological_node(req.name, req.pose)
        return res

    def add_topological_node(self, node_name, node_pose):
        dist = 8.0
        #Get New Node Name
        if node_name:
            name = node_name
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
        node.pose = node_pose
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
            eid = '%s_%s' %(node.name, i)
            e.edge_id = eid
            e.top_vel =0.55
            e.map_2d = node.map
            node.edges.append(e)

        for i in close_nodes:
            self.add_edge(i, node.name, 'move_base', '')

        msg_store.insert(node,meta)
        return True

    def update_node_name_cb(self, req):
        return self.update_node_name(req.node_name, req.new_name)

    def update_node_name(self, node_name, new_name):
        if new_name in self.names:
            return False, "node with name {0} already exists".format(req.new_name)

        msg_store = MessageStoreProxy(collection='topological_maps')
        # The query retrieves the node name with the given name from the given pointset.
        query = {"name": node_name, "pointset": self.name}
        # The meta-information is some additional information about the specific
        # map that we are interested in (?)
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.nodes.map
        # This returns a tuple containing the object, if it exists, and some
        # information about how it's stored in the database.
        available = msg_store.query(TopologicalNode._type, query, query_meta)

        if len(available) == 1:
            available[0][0].name = new_name
            # Also need to update all edges which involve the renamed node
            allnodes_query = {"pointset": self.name}
            allnodes_query_meta = {}
            allnodes_query_meta["pointset"] = self.name
            allnodes_query_meta["map"] = self.nodes.map
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
            return True, ""
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))
            return False, "multiple nodes with the name existed, or node not found"

    def update_node_waypoint_cb(self, req):
        return self.update_node_waypoint(req.name, req.pose)

    def update_node_waypoint(self, name, pose):
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name": name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.nodes.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            positionZ=available[0][0].pose.position.z
            available[0][0].pose = pose
            available[0][0].pose.position.z = positionZ
            msg_store.update(available[0][0], query_meta, query, upsert=True)
            return True
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))
            return False

    def update_node_tolerance_cb(self, req):
        return self.update_node_tolerance(req.node_name, req.xy_tolerance, req.yaw_tolerance)

    def update_node_tolerance(self, name, new_xy, new_yaw):
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name": name, "pointset": self.name}
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.nodes.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            available[0][0].xy_goal_tolerance = new_xy
            available[0][0].yaw_goal_tolerance = new_yaw

            msg_store.update(available[0][0], query_meta, query, upsert=True)
            return True, ""
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))
            return False, ""


    def remove_node_cb(self, req):
        res = self.remove_node(req.name)
        return res

    def remove_node(self, node_name) :
        rospy.loginfo('Removing Node: '+node_name)
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"name" : node_name, "pointset": self.nodes.name}
        query_meta = {}
        query_meta["pointset"] = self.nodes.name
        query_meta["map"] = self.nodes.map

        available = msg_store.query(TopologicalNode._type, query, query_meta)

        node_found = False
        if len(available) == 1 :
            node_found = True
            rm_id = str(available[0][1]['_id'])
            print rm_id
        else :
            rospy.logerr("Node not found "+str(len(available))+" waypoints found after query")
            #rospy.logerr("Available data: "+str(available))

        if node_found :
            query_meta = {}
            query_meta["pointset"] = self.nodes.name
            edges_to_rm = []
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
            for i in message_list:
                for j in i[0].edges :
                    if j.node == node_name :
                        edge_rm = j.edge_id
                        edges_to_rm.append(edge_rm)

            for k in edges_to_rm :
                print 'remove: '+k
                self.remove_edge(k)

            msg_store.delete(rm_id)
            return True
        else:
            return False


    def remove_edge_cb(self, req):
        return self.remove_edge(req.edge_id)

    def remove_edge(self, edge_name) :
        #print 'removing edge: '+edge_name
        rospy.loginfo('Removing Edge: '+edge_name)
        msg_store = MessageStoreProxy(collection='topological_maps')
        query = {"edges.edge_id" : edge_name, "pointset": self.nodes.name}
        query_meta = {}
        query_meta["pointset"] = self.nodes.name
        query_meta["map"] = self.nodes.map
        available = msg_store.query(TopologicalNode._type, query, query_meta)

        if len(available) >= 1 :
            for i in available :
                print i[0]
                for j in i[0].edges:
                    if j.edge_id == edge_name :
                        i[0].edges.remove(j)
                        msg_store.update(i[0], query_meta, query, upsert=True)
            return True
        else :
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available)) 
            return False

    def update_edge_cb(self, req):
        return self.update_edge(req.edge_id, req.action, req.top_vel)

    def update_edge(self, edge_id, action, top_vel):
        msg_store = MessageStoreProxy(collection='topological_maps')
        # The query retrieves the node name with the given name from the given pointset.
        query = {"name": edge_id.split('_')[0], "pointset": self.name}
        # The meta-information is some additional information about the specific
        # map that we are interested in (?)
        query_meta = {}
        query_meta["pointset"] = self.name
        query_meta["map"] = self.nodes.map
        # This returns a tuple containing the object, if it exists, and some
        # information about how it's stored in the database.
        available = msg_store.query(TopologicalNode._type, query, query_meta)
        if len(available) == 1:
            for edge in available[0][0].edges:
                if edge.edge_id == edge_id:
                    edge.action = action or edge.action
                    edge.top_vel = top_vel or edge.top_vel

            msg_store.update(available[0][0], query_meta, query, upsert=True)
            return True, ""
        else:
            rospy.logerr("Impossible to store in DB "+str(len(available))+" waypoints found after query")
            rospy.logerr("Available data: "+str(available))
            return False, "no edge found or multiple edges found"


    def get_edges_between(self, nodea, nodeb):
         ab=[]
         ba=[]
         for i in self.nodes.nodes:
             if nodea == i.name:
                 for j in i.edges:
                     if j.node == nodeb:
                         ab.append(j.edge_id)
             if nodeb == i.name:
                 for j in i.edges:
                     if j.node == nodea:
                         ba.append(j.edge_id)
         return ab, ba

    def get_edges_between_cb(self, req):
         return self.get_edges_between(req.nodea, req.nodeb)



    def loadMap(self, point_set) :
        msg_store = MessageStoreProxy(collection='topological_maps')
        points = strands_navigation_msgs.msg.TopologicalMap()
        points.name = point_set
        points.pointset = point_set

        query_meta = {}
        query_meta["pointset"] = point_set

        ntries=1
        map_found=False

        #Tries to load the map for a minute if not it quits
        while not map_found :
            available = len(msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, {}, query_meta))
            if available <= 0 :
                rospy.logerr("Desired pointset '"+point_set+"' not in datacentre, try :"+str(ntries))
                if ntries <=10 :
                    ntries+=1
                    rospy.sleep(rospy.Duration.from_sec(6))
                else :
                    raise Exception("Can't find waypoints.")
                    return points
                    #We just want to raise the exception not quit
                    #sys.exit(2)
            else:
                map_found=True

        query_meta = {}
        query_meta["pointset"] = point_set

        message_list = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, {}, query_meta)


        points.name = point_set
        points.pointset = point_set
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
