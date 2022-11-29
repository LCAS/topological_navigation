#!/usr/bin/env python

import sys
import rospy
import yaml
import math
import time
import json

#import strands_navigation_msgs.msg
from topological_navigation_msgs.msg import TopologicalMap
from topological_navigation.tmap_utils import *
import mongodb_store.util as dc_util
from topological_navigation.tmap_utils import *
from geometry_msgs.msg import Point


"""
    get_distance
    
    Returns the straight line distance between two poses
""" 
def point_distance(point1, point2):
    #dist=math.hypot((pose.position.x-node.pose[0].position.x),(pose.position.y-node.pose[0].position.y))
    dist=math.hypot((point1.x-point2.x),(point1.y-point2.y))
    return dist


def get_radius(node):
    rad=0
    ori=Point()
    ori.x=0.0
    ori.y=0.0
    ori.z=0.0
    for k in node.verts:
        point = Point()
        point.x = k.x
        point.y = k.y
        point.z = 0.0
        #print point, node.pose.position 
        vertrad = point_distance(ori, point)
        #print vertrad, rad
        rad+=vertrad    
    #print rad, len(node.verts)
    rad=rad/len(node.verts)
    return rad

class EdgeAnalysis(object):

    def __init__(self, name) :
        self.lnodes = []
        self.edgid=[]
        self.eids = []
#        self.unknowns = []

        self.map_received =False
        rospy.Subscriber('topological_map', TopologicalMap, self.MapCallback)       
        rospy.loginfo("Waiting for Topological map ...")
        
        while not self.map_received:
            pass
        
        self.get_list_of_edges()

        for i in self.eids:
            print i

        yml = yaml.safe_dump(self.eids, default_flow_style=False)
        
        fh = open("edges.yaml", "w")
        s_output = str(yml)
#        print s_output
        fh.write(s_output)
        fh.close            


    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.lnodes = msg
        self.map_received = True
        #print self.lnodes


    def get_list_of_edges(self):
        self.eids = []
        
        rospy.loginfo("Querying for list of edges")
        for i in self.lnodes.nodes :
            for j in i.edges:
                if j.edge_id not in self.edgid :
                    self.edgid.append(j.edge_id)
                    val={}
                    val["edge_id"]=j.edge_id
                    #val["model_id"]=self.lnodes.name+'__'+j.edge_id
                    ori={}
                    ori['pos']= dc_util.msg_to_document(i.pose.position)
                    ori['name']=i.name
                    rad = get_radius(i)
                    ori['radius']=rad
                    val["ori"]=ori

                    ddn=get_node(self.lnodes, j.node)                   
                    dest={}
                    dest['name']=j.node    
                    dest['pos']= dc_util.msg_to_document(ddn.pose.position)
                    rad = get_radius(ddn)
                    dest['radius']=rad
                    val["dest"]=dest
                    val["dist"]= get_distance_to_node(i,ddn)

                    self.eids.append(val)
        fdbmsg = 'Done. %d edges found' %len(self.edgid)
        rospy.loginfo(fdbmsg)



if __name__ == '__main__':
    rospy.init_node('edge_analisys')
    server = EdgeAnalysis(rospy.get_name())