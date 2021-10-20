#!/usr/bin/env python

import sys
import rospy
import actionlib
import pymongo
import json
import sys
import math



from topological_navigation.topological_node import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from topological_navigation_msgs.msg import TopologicalNode

from mongodb_store.message_store import MessageStoreProxy
import topological_navigation.msg



class TopologicalNavLoc(object):

    def __init__(self, dataset_name, filename) :
        #print "loading file from map %s" %filename
        self.lnodes = self.loadMap(dataset_name)
        self.export_map(filename)


    def export_map(self, filename):
        #Clean the file in case it existed
        fh = open(filename, "w")
        fh.close

        #Write File
        for i in self.lnodes :
            fh = open(filename, "a")
            print "node: \n\t%s" %i.name
            s_output = "node: \n\t%s\n" %i.name
            fh.write(s_output)
            print "\twaypoint:\n\t%s" %i.waypoint
            s_output = "\twaypoint:\n\t\t%f,%f,%f,%f,%f,%f,%f\n" %(float(i.waypoint[0]),float(i.waypoint[1]),float(i.waypoint[2]),float(i.waypoint[3]),float(i.waypoint[4]),float(i.waypoint[5]),float(i.waypoint[6]))
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


    def loadMap(self, point_set):

        point_set=str(sys.argv[1])
        #map_name=str(sys.argv[3])

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
                #print i[0].name
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


if __name__ == '__main__':
    if len(sys.argv) < 3 :
        print "usage: map_export dataset_name output_file.tplg"
        sys.exit(2)

    dataset_name=str(sys.argv[1])
    filename=str(sys.argv[2])
    rospy.init_node('topological_map_exporter')
    server = TopologicalNavLoc(dataset_name, filename)
