#!/usr/bin/env python

import json
import sys
import rospy
from geometry_msgs.msg import Pose
from strands_navigation_msgs.msg import TopologicalNode, Edge, Vertex

import pymongo
#import mongodb_store.util
from mongodb_store.message_store import MessageStoreProxy


class topological_node(object):
    def __init__(self,node_name, dataset_name, map_name):
        self.name=node_name
        self.pointset=dataset_name
        self.map=map_name

    def _insert_waypoint(self, swaypoint):
        self.waypoint=swaypoint.split(',')
        
    def _insert_edges(self, edges):
        self.edges=edges

    def _insert_vertices(self, vertices):
        self.vertices=vertices

def get_edge_id(source, target, eids):
    test=0
    eid = '%s_%s' %(source, target)
    while eid in eids:
        eid = '%s_%s_%3d' %(source, target, test)
        test += 1
    return eid


def loadMap(inputfile, dataset_name, map_name) :

    print "openning %s" %inputfile 
    fin = open(inputfile, 'r')
    print "Done"

    line = fin.readline()
    anode=topological_node("Empty", dataset_name, map_name)
    lnodes=[anode]
    while line:
        #node line
        if line.startswith('node:'):
            #Saving Name of the Node
            line = fin.readline()
            name = line.strip('\t')
            name = name.strip('\n')
            anode=topological_node(name, dataset_name, map_name)

            #Saving WayPoint            
            line = fin.readline()
            if line.startswith('\t') :
                if line.startswith('\twaypoint:') :
                    #Reading Line with WayPoint
                    line = fin.readline()
                    ways = line.strip('\t')
                    ways = ways.strip('\n')
                    anode._insert_waypoint(ways)
                    
            #Saving edges
            line = fin.readline()
            if line.startswith('\t') :
                if line.startswith('\tedges:') :
                    aedge = {'node':"empty", 'action':"move_base"}
                    edges=[aedge]
                    line = fin.readline()
                    while line and not(line.startswith('\tvertices:')) :
                        info= line.strip('\t')
                        inf = info.split(',',2)
                        aedge = {'node':inf[0].strip(), 'action':inf[1].strip()}
                        edges.append(aedge)
                        line = fin.readline()
                    edges.pop(0)
                    anode._insert_edges(edges)

            #Saving vertices
            #line = fin.readline()                    
            if line.startswith('\t') :
                if line.startswith('\tvertices:') :
                    vertices=[]
                    line = fin.readline()
                    while line and not(line.startswith('node:')) :
                        info= line.strip('\t')
                        inf = info.split(',',2)
                        vertex = (float(inf[0].strip()), float(inf[1].strip()))
                        vertices.append(vertex)
                        line = fin.readline()
                    anode._insert_vertices(vertices)
            lnodes.append(anode)
    fin.close()
    lnodes.pop(0)

    return lnodes         


if __name__ == '__main__':
    if len(sys.argv) < 4 :
        print "usage: insert_map input_file.txt dataset_name map_name"
	sys.exit(2)

    filename=str(sys.argv[1])
    dataset_name=str(sys.argv[2])
    map_name=str(sys.argv[3])

    msg_store = MessageStoreProxy(collection='topological_maps')

    eids=[] #list of known edge id's
    lnodes=loadMap(filename, dataset_name, map_name)

    meta = {}
    meta["map"] = map_name
    meta["pointset"] = dataset_name
        
    for i in lnodes:
        #val=i.__dict__#json.loads(vala)        print val #+ '\n'
        n = TopologicalNode()
        n.name = i.name
        meta["node"] = i.name
        n.map = i.map
        n.pointset = i.pointset
        p = Pose()
        p.position.x=float(i.waypoint[0])
        p.position.y=float(i.waypoint[1])
        p.position.z=float(i.waypoint[2])
        p.orientation.x=float(i.waypoint[3])
        p.orientation.y=float(i.waypoint[4])
        p.orientation.z=float(i.waypoint[5])
        p.orientation.w=float(i.waypoint[6])
        n.pose = p
        for j in i.vertices :
            v = Vertex()
            v.x = float(j[0])
            v.y = float(j[1])
            n.verts.append(v)
        for k in i.edges :
            e = Edge()
            e.node = k['node']
            e.action = k['action']
            eid = get_edge_id(i.name, e.node, eids)
            eids.append(eid)
            e.edge_id = eid
            e.top_vel =0.55
            e.map_2d = map_name        
            n.edges.append(e)
            
        print n
        msg_store.insert(n,meta)
        #mongodb_store.util.store_message(points_db,p,val)
