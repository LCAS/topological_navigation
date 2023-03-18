#!/usr/bin/env python

#==============================================================================
# This Script takes a topological map file (.tmap) and creates a Yaml file with
# the topological map definition
#
# Usage:
#
# `rosrun topological_utils tmap_to_yaml.py input_file.tmap outfile dataset map_name`
#
#==============================================================================


#import json
import sys
#import rospy
import yaml

from geometry_msgs.msg import Pose
from topological_navigation_msgs.msg import TopologicalNode, Edge, Vertex
import mongodb_store.util as dc_util

#import pymongo
#import mongodb_store.util
#from mongodb_store.message_store import MessageStoreProxy


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
    if len(sys.argv) < 5 :
        print "usage: tmap_to_yaml.py input_file.tmap outfile dataset map_name"
        sys.exit(2)

    filename=str(sys.argv[1])
    outfile=str(sys.argv[2])
    dataset_name=str(sys.argv[3])
    map_name=str(sys.argv[4])

    eids=[] #list of known edge id's
    lnodes=loadMap(filename, dataset_name, map_name)
    nnodes=[]

    meta = {}
    meta["map"] = map_name
    meta["pointset"] = dataset_name

    for i in lnodes:
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
#            e.use_default_recovery = True
#            e.use_default_nav_recovery = True
#            e.use_default_helpers = True
            n.edges.append(e)
        nnodes.append(n)


    onodes = []
    for i in nnodes:
        r = {}
        q = dc_util.msg_to_document(i)
        m = {}
        m["map"] = map_name
        m["pointset"] = dataset_name
        m["node"] = n.name
        r["meta"] = m
        r["node"] = q
        onodes.append(r)


    onodes.sort(key=lambda x: x["node"]['name'])
    yml = yaml.safe_dump(onodes, default_flow_style=False)
    #print yml
    #print s_output

    fh = open(outfile, "w")
    #s_output = str(bson.json_util.dumps(nodeinf, indent=1))
    s_output = str(yml)
    print s_output
    fh.write(s_output)
    fh.close

#    for i in onodes:
#        print i