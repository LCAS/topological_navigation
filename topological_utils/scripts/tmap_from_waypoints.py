#!/usr/bin/env python

import sys
import math

class Topo_node(object):
    def __init__(self,node_name,waypoint):
        self.node_name=node_name
        self.waypoint=waypoint

    def _insert_edges(self, edges):
        self.edges=edges

    def _insert_vertices(self, vertices):
        self.vertices=vertices
        

        
def node_dist(waypoint1,waypoint2):
    vector1=waypoint1.split(',')
    vector2=waypoint2.split(',')
    print vector1[1]
    dist = math.sqrt( (float(vector1[0]) - float(vector2[0]))**2 + (float(vector1[1]) - float(vector2[1]))**2 )
    return dist


if __name__ == '__main__':
    if len(sys.argv) < 3 :
        print "usage: tmap_from_waypoint input_file output_file [max_dist_connect]"
        sys.exit(2)
    
    
    
    filename=str(sys.argv[1])
    outfile=str(sys.argv[2])

    
    if len(sys.argv) == 4:
        max_dist_connect=float(sys.argv[3])
    else:
        max_dist_connect=1000
    
    
    fin = open(filename, 'r')

    #Inserting charging point
    nnodes=0
    #line = fin.readline()
    line = "0,0,0,0,0,0,0\n"
    cnode=Topo_node("ChargingPoint",line)
    #lnodes=[node]
    
    lnodes=[]
    line = fin.readline()
    #Inserting waypoints
    while line:
        nnodes=nnodes+1
        wname= "WayPoint%d" %nnodes
        node=Topo_node(wname,line)
        lnodes.append(node)
        line = fin.readline()
    fin.close()
    
    #inserting edges
    cedges=[]
    cedge = {'node':lnodes[0].node_name, 'action':"undocking"}
    cedges.append(cedge)
    cnode._insert_edges(cedges)
    
    nnodes=len(lnodes)
    eind=0
    for i in lnodes :
        edge = {'node':"empty", 'action':"move_base"}
        edges=[edge]
        if eind == 0:
            edge = {'node':cnode.node_name, 'action':"docking"}
            edges.append(edge)
        for j in lnodes :
            if i.node_name is not j.node_name and node_dist(i.waypoint,j.waypoint)<max_dist_connect:
                node_dist(i.waypoint,j.waypoint)
                edge = {'node':j.node_name, 'action':"human_aware_navigation"}
                edges.append(edge)
                i._insert_edges(edges)
        eind+=1
        i.edges.pop(0)


    #inserting corners
    cvertices=[(0.25, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.25, -0.69)]
    cnode._insert_vertices(cvertices)

    for i in lnodes :
        vertices=[(0.69, 0.287), (0.287, 0.69), (-0.287, 0.69), (-0.69, 0.287), (-0.69, -0.287), (-0.287, -0.69), (0.287, -0.69), (0.69, -0.287)]
        i._insert_vertices(vertices)

    lnodes.insert(0,cnode)
    
    #Clean the file in case it existed
    fh = open(outfile, "w")
    fh.close

    #Write File
    for i in lnodes :
        fh = open(outfile, "a")
        print "node: \n\t%s" %i.node_name
        s_output = "node: \n\t%s\n" %i.node_name
        fh.write(s_output)
        print "\twaypoint:\n\t%s" %i.waypoint
        s_output = "\twaypoint:\n\t\t%s" %i.waypoint
        fh.write(s_output)
        print "\tedges:"
        s_output = "\tedges:\n"
        fh.write(s_output)
        for k in i.edges :
            print "\t\t %s, %s" %(k['node'],k['action'])
            s_output = "\t\t %s, %s\n" %(k['node'],k['action'])
            fh.write(s_output)
        print "\tvertices:"
        s_output = "\tvertices:\n"
        fh.write(s_output)
        for k in i.vertices :
            print "\t\t%f,%f" %(k[0],k[1])
            s_output = "\t\t%f,%f\n" %(k[0],k[1])
            fh.write(s_output)
        fh.close        