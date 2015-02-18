#!/usr/bin/env python

import sys
import math
import json
import bson
import pymongo
import yaml
import mongodb_store.util as dc_util

from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import Vertex
from strands_navigation_msgs.msg import Edge
from geometry_msgs.msg import Pose


def pose_from_waypoint(waypoint):
    v=waypoint.split(',')
    p = Pose()
    p.position.x = float(v[0])
    p.position.y = float(v[1])
    p.position.z = float(v[2])
    p.orientation.x = float(v[3])
    p.orientation.y = float(v[4])
    p.orientation.z = float(v[5])
    p.orientation.w = float(v[6])
    return p

def get_vertex_list(vertices):
    v = Vertex()
    verts = []
    for i in vertices:
        v.x = i[0]
        v.y = i[1]
        verts.append(v)
    return verts

def node_dist(node1,node2):
    dist = math.sqrt((node1.pose[0].position.x - node2.pose[0].position.x)**2 + (node1.pose[0].position.y - node2.pose[0].position.y)**2 )
    return dist

def get_empty_edge(mapname,standard_action):
    e = Edge()
    e.action = standard_action
    e.top_vel =0.55
    e.map_2d = mapname
    e.use_default_recovery = True
    e.use_default_nav_recovery = True
    e.use_default_helpers = True
    return e

if __name__ == '__main__':
    if len(sys.argv) < 4 :
        print "usage: tmap_from_waypoint input_file pointset map [max_dist_connect]"
        sys.exit(2)
        
    filename=str(sys.argv[1])
    pointset=str(sys.argv[2])
    mapname=str(sys.argv[3])

    vertices=[(0.69, 0.287), (0.287, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.287, -0.69), (0.69, -0.287)]

    if len(sys.argv) == 5:
        max_dist_connect=float(sys.argv[4])
    else:
        max_dist_connect=100
    
    standard_action = 'move_base'
    

    fin = open(filename, 'r')
    lnodes=[]
    line = fin.readline()
    nnodes=0
    #Inserting waypoints
    while line:
        nnodes=nnodes+1
        wname= "WayPoint%d" %nnodes
        o = []
        n = TopologicalNode()
        n.name = wname
        n.map = mapname
        n.pointset = pointset
        n.pose.append(pose_from_waypoint(line))
        n.yaw_goal_tolerance = 0.1
        n.xy_goal_tolerance = 0.3
        n.verts = get_vertex_list(vertices)
        m = {}
        m["map"] = mapname
        m["pointset"] = pointset
        m["node"] = n.name
        o.append(n)
        o.append(m)
        lnodes.append(o)
        line = fin.readline()
    fin.close()

#    for i in lnodes:        print i["node"]

    edge_names = []
    for i in lnodes:
        for j in lnodes:
            if node_dist(i[0],j[0]) < max_dist_connect and i[0].name != j[0].name :
                print "connecting %s to %s"%(i[0].name,j[0].name)
                e = get_empty_edge(mapname, standard_action)
                e.edge_id = "%s_%s"%(i[0].name,j[0].name)
                e.node = j[0].name
                i[0].edges.append(e)


    for i in lnodes:                   
        yml = yaml.dump(i[1])
        print yml
    #print s_output






    #max speed 0.55



#    #Inserting charging point
#    nnodes=0
#    #line = fin.readline()
#    line = "0,0,0,0,0,0,0\n"
#    cnode=Topo_node("ChargingPoint",line)
#    #lnodes=[node]
#    
#    lnodes=[]
#    line = fin.readline()
#    #Inserting waypoints
#    while line:
#        nnodes=nnodes+1
#        wname= "WayPoint%d" %nnodes
#        node=Topo_node(wname,line)
#        lnodes.append(node)
#        line = fin.readline()
#    fin.close()
#    
#    #inserting edges
#    cedges=[]
#    cedge = {'node':lnodes[0].node_name, 'action':"undocking"}
#    cedges.append(cedge)
#    cnode._insert_edges(cedges)
#    
#    nnodes=len(lnodes)
#    eind=0
#    for i in lnodes :
#        edge = {'node':"empty", 'action':"move_base"}
#        edges=[edge]
#        if eind == 0:
#            edge = {'node':cnode.node_name, 'action':"docking"}
#            edges.append(edge)
#        for j in lnodes :
#            if i.node_name is not j.node_name and node_dist(i.waypoint,j.waypoint)<max_dist_connect:
#                node_dist(i.waypoint,j.waypoint)
#                edge = {'node':j.node_name, 'action':"human_aware_navigation"}
#                edges.append(edge)
#                i._insert_edges(edges)
#        eind+=1
#        i.edges.pop(0)
#
#
#    #inserting corners
#    cvertices=[(0.25, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.25, -0.69)]
#    cnode._insert_vertices(cvertices)
#
#    for i in lnodes :
#        vertices=[(0.69, 0.287), (0.287, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.287, -0.69), (0.69, -0.287)]
#        i._insert_vertices(vertices)
#
#    lnodes.insert(0,cnode)
#    
#    #Clean the file in case it existed
#    fh = open(outfile, "w")
#    fh.close
#
#    #Write File
#    for i in lnodes :
#        fh = open(outfile, "a")
#        print "node: \n\t%s" %i.node_name
#        s_output = "node: \n\t%s\n" %i.node_name
#        fh.write(s_output)
#        print "\twaypoint:\n\t%s" %i.waypoint
#        s_output = "\twaypoint:\n\t\t%s" %i.waypoint
#        fh.write(s_output)
#        print "\tedges:"
#        s_output = "\tedges:\n"
#        fh.write(s_output)
#        for k in i.edges :
#            print "\t\t %s, %s" %(k['node'],k['action'])
#            s_output = "\t\t %s, %s\n" %(k['node'],k['action'])
#            fh.write(s_output)
#        print "\tvertices:"
#        s_output = "\tvertices:\n"
#        fh.write(s_output)
#        for k in i.vertices :
#            print "\t\t%f,%f" %(k[0],k[1])
#            s_output = "\t\t%f,%f\n" %(k[0],k[1])
#            fh.write(s_output)
#        fh.close        