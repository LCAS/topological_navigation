#!/usr/bin/env python

import json
import sys
import rospy
from geometry_msgs.msg import Pose

import pymongo
import strands_datacentre.util

class topological_node(object):
    def __init__(self,node_name, dataset_name, map_name):
        self.name=node_name
        self.pointset=dataset_name
        self.map=map_name
        #self.expanded=False
        #self.father='none'

    def _insert_waypoint(self, swaypoint):
        self.waypoint=swaypoint.split(',')
        #self.waypoint = Pose()
        #self.waypoint.position.x=lwaypoint[0]
        #self.waypoint.position.y=lwaypoint[1]
        #self.waypoint.position.z=lwaypoint[2]
        #self.waypoint.orientation.x=lwaypoint[3]
        #self.waypoint.orientation.y=lwaypoint[4]
        #self.waypoint.orientation.z=lwaypoint[5]
        #self.waypoint.orientation.w=lwaypoint[6]
        
    def _insert_edges(self, edges):
        self.edges=edges

def loadMap(inputfile, dataset_name, map_name) :
    print "openning %s" %inputfile 
    fin = open(inputfile, 'r')
    print "Done"
    line = fin.readline()
    node=topological_node("Empty", dataset_name, map_name)
    lnodes=[node]
    while line:
        if line.startswith('node:') :
            line = fin.readline()
            name = line.strip('\t')
            name = name.strip('\n')
            node=topological_node(name, dataset_name, map_name)
            line = fin.readline()
            if line.startswith('\t') :
                if line.startswith('\twaypoint:') :
                    line = fin.readline()
                    ways = line.strip('\t')
                    ways = ways.strip('\n')
                    node._insert_waypoint(ways)
            line = fin.readline()
            if line.startswith('\t') :
                if line.startswith('\tedges:') :
                    edge = {'node':"empty", 'action':"move_base"}
                    edges=[edge]
                    line = fin.readline()
                    while line and not(line.startswith('node:')) :
                        info= line.strip('\t')
                        inf = info.split(',',2)
                        edge = {'node':inf[0].strip(), 'action':inf[1].strip()}
                        edges.append(edge)
                        line = fin.readline()
                    edges.pop(0)
                    node._insert_edges(edges)
            lnodes.append(node)
    fin.close()
    lnodes.pop(0)

    return lnodes        

if __name__ == '__main__':
    filename=str(sys.argv[1])
    dataset_name=str(sys.argv[2])
    map_name=str(sys.argv[3])
    host = rospy.get_param("datacentre_host")
    port = rospy.get_param("datacentre_port")
    print "Using datacentre  ",host,":", port
    client = pymongo.MongoClient(host, port)
    db=client.autonomous_patrolling
    points_db=db["waypoints"]
    lnodes=loadMap(filename, dataset_name, map_name)
    
    for i in lnodes:
        #vala = json.dumps(i.__dict__)
        val=i.__dict__#json.loads(vala)
        print val #+ '\n'
        p = Pose()
        p.position.x=i.waypoint[0]
        p.position.y=i.waypoint[1]
        p.position.z=i.waypoint[2]
        p.orientation.x=i.waypoint[3]
        p.orientation.y=i.waypoint[4]
        p.orientation.z=i.waypoint[5]
        p.orientation.w=i.waypoint[6]

        strands_datacentre.util.store_message(points_db,p,val)
    