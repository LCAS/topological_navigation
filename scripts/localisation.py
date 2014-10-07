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
from mongodb_store.message_store import MessageStoreProxy

from topological_navigation.topological_node import *

import topological_navigation.msg


def get_node(name, clist):
    for i in clist:
        if i.name == name:
            return i


def get_distance_to_node(node, pose):
    dist=math.hypot((pose.position.x-node.pose.position.x),(pose.position.y-node.pose.position.y))
    return dist


class TopologicalNavLoc(object):
       
    
    def __init__(self, name) :
        self.throttle_val = rospy.get_param("~LocalisationThrottle", 5)
        self.throttle = self.throttle_val
        self.node="Unknown"
        
        #self._action_name = name
        self.wp_pub = rospy.Publisher('/closest_node', String)
        self.cn_pub = rospy.Publisher('/current_node', String)
        
        self.lnodes = []

        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        
        rospy.loginfo("Waiting for Topological map ...")
        
        while len(self.lnodes) == 0:
            pass
            
        rospy.Subscriber("/robot_pose", Pose, self.PoseCallback)
        #self.run_analysis()

        rospy.loginfo("All Done ...")
        rospy.spin()


    def PoseCallback(self, msg):
        if(self.throttle%self.throttle_val==0):
            a = float('1000.0')
            for i in self.lnodes:
                d=get_distance_to_node(i, msg)
                if d < a:
                    b=i
                    a=d
            self.wp_pub.publish(String(b.name))
            if self.point_in_poly(b, msg) :
                self.cn_pub.publish(String(b.name))
            else :
                self.cn_pub.publish('none')
            self.throttle=1
        else:
            self.throttle +=1


    def MapCallback(self, msg) :
        self.lnodes = msg.nodes
        for i in self.lnodes : 
            print i


    def point_in_poly(self,node,pose):
        x=pose.position.x-node.pose.position.x
        y=pose.position.y-node.pose.position.y
        
        n = len(node.verts)
        inside = False

    
        p1x = node.verts[0].x
        p1y = node.verts[0].y
        for i in range(n+1):
            p2x = node.verts[i % n].x
            p2y = node.verts[i % n].y
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
    rospy.init_node('topological_localisation')
    server = TopologicalNavLoc(rospy.get_name())
