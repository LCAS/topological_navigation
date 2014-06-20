#!/usr/bin/env python

import sys
import rospy
import actionlib
import pymongo
import json
import sys
import math



from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import scitos_apps_msgs.msg

from strands_navigation_msgs.msg import TopologicalNode
from ros_datacentre.message_store import MessageStoreProxy
from topological_navigation.topological_node import *

import topological_navigation.msg


def get_node(name, clist):
    for i in clist:
        if i.name == name:
            return i


class TopologicalNavLoc(object):
    _feedback = topological_navigation.msg.GotoNodeFeedback()
    _result   = topological_navigation.msg.GotoNodeResult()
    
    def __init__(self, name, filename) :
        self.throttle=5
        self.cancelled = False
        self.node="Unknown"
        
        self._action_name = name
        #print "loading file from map %s" %filename
        self.lnodes = self.loadMap(filename)
        #print "Done"

        self.wp_pub = rospy.Publisher('/closest_node', String)
        self.cn_pub = rospy.Publisher('/current_node', String)
            
        rospy.Subscriber("/robot_pose", Pose, self.PoseCallback)

        #self.run_analysis()

        rospy.loginfo("All Done ...")
        rospy.spin()

    def PoseCallback(self, msg):
        if(self.throttle%5==0):
            #print "robot pose (%f, %f)" %(msg.position.x,msg.position.y)
            a = float('1000.0')
            x=msg.position.x
            y=msg.position.y
            for i in self.lnodes:
                d=i._get_distance(x, y)
                if d < a:
                    b=i
                    a=d
            self.wp_pub.publish(String(b.name))
            if a < b.influence_radius :
                if self.point_in_poly(x,y,b) :
                    self.cn_pub.publish(String(b.name))
            else :
                self.cn_pub.publish('none')
            self.throttle=1
        else:
            self.throttle +=1


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


    def run_analysis(self):
        for i in self.lnodes:
            print ("%s:") %i.name
            print ("%s:") %i.influence_radius

    def point_in_poly(self,x,y,node):
        x=x-node.px
        y=y-node.py
   
        n = len(node.vertices)
        inside = False
    
        p1x,p1y = node.vertices[0]
        for i in range(n+1):
            p2x,p2y = node.vertices[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y
            
        return inside


if __name__ == '__main__':
    filename=str(sys.argv[1])
    rospy.init_node('topological_localisation')
    server = TopologicalNavLoc(rospy.get_name(),filename)