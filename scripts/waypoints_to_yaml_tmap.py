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
    if len(sys.argv) < 5 :
        print "usage: tmap_from_waypoint input_file output_file pointset map [max_dist_connect]"
        sys.exit(2)
        
    filename=str(sys.argv[1])
    outfile=str(sys.argv[2])
    pointset=str(sys.argv[3])
    mapname=str(sys.argv[4])

    vertices=[(0.69, 0.287), (0.287, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.287, -0.69), (0.69, -0.287)]

    if len(sys.argv) == 6:
        max_dist_connect=float(sys.argv[5])
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
        #n = dc_util.msg_to_document(n1)
        #print n
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
    print "calculate edges"
    edge_names = []
    for i in lnodes:
        #im = dc_util.dictionary_to_message(i[0], TopologicalNode)
        for j in lnodes:
            #jm = dc_util.dictionary_to_message(j[0], TopologicalNode)
            if node_dist(i[0],j[0]) < max_dist_connect and i[0].name != j[0].name :
                print "connecting %s to %s"%(i[0].name,j[0].name)
                e = get_empty_edge(mapname, standard_action)
                e.edge_id = "%s_%s"%(i[0].name,j[0].name)
                e.node = j[0].name
                i[0].edges.append(e)

    onodes = []
    for i in lnodes:
        r = []
        q = dc_util.msg_to_document(i[0])
        r.append(q)
        r.append(i[1])
        onodes.append(r)


    yml = yaml.safe_dump(onodes, default_flow_style=False)
    print yml
    #print s_output

    fh = open(outfile, "w")
    #s_output = str(bson.json_util.dumps(nodeinf, indent=1))
    s_output = str(yml)
    print s_output
    fh.write(s_output)
    fh.close
       