#!/usr/bin/env python

import sys

class Topo_node(object):
    def __init__(self,node_name,waypoint):
        self.node_name=node_name
        self.waypoint=waypoint

    def _insert_edges(self, edges):
        self.edges=edges

if __name__ == '__main__':
    if len(sys.argv) < 3 :
        print "usage: tmap_from_waypoint input_file output_file"
	sys.exit(2)
    filename=str(sys.argv[1])
    outfile=str(sys.argv[2])
    fin = open(filename, 'r')
    nnodes=0
    line = fin.readline()
    wname=("WayPoint%d", nnodes)
    node=Topo_node("Charging Point",line)
    lnodes=[node]
    line = fin.readline()
    while line:
        nnodes=nnodes+1
        wname= "WayPoint%d" %nnodes
        node=Topo_node(wname,line)
        lnodes.append(node)
        line = fin.readline()
    fin.close()
    nnodes=len(lnodes)
    for i in lnodes :
        edge = {'node':"empty", 'action':"move_base"}
        edges=[edge]
        for j in lnodes :
            if i.node_name is not j.node_name :
                edge = {'node':j.node_name, 'action':"move_base"}
                edges.append(edge)
                i._insert_edges(edges)
        i.edges.pop(0)
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
        fh.close        