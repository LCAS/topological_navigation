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
import scitos_apps_msgs.msg
import strands_datacentre.util
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
        print "loading file from map %s" %filename
        self.lnodes = self.loadMap(filename)
        print "Done"

        self.wp_pub = rospy.Publisher('/current_node', String)

        rospy.Subscriber("/robot_pose", Pose, self.PoseCallback)


        self.run_analysis()

        rospy.loginfo("All Done ...")
        rospy.spin()

    def PoseCallback(self, msg):
        if(self.throttle%5==0):
            #print "robot pose (%f, %f)" %(msg.position.x,msg.position.y)
            a = float('1000.0')
            for i in self.lnodes:
                d=i._get_distance(msg.position.x, msg.position.y)
                if d < a:
                    b=i
                    a=d
            self.wp_pub.publish(String(b.name))
            self.throttle=1
        else:
            self.throttle +=1


    def loadMap(self, pointset):
        pointset=str(sys.argv[1])
        host = rospy.get_param("datacentre_host")
        port = rospy.get_param("datacentre_port")
        print "Using datacentre  ",host,":", port
        client = pymongo.MongoClient(host, port)
        db=client.autonomous_patrolling
        points_db=db["waypoints"]
        available = points_db.find().distinct("meta.pointset")
        print available
        if pointset not in available :
            rospy.logerr("Desired pointset '"+pointset+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
        
        #points = self._get_points(waypoints_name) 
        points = []
        search =  {"meta.pointset": pointset}
        for point in points_db.find(search) :
            a= point["meta"]["name"]
            b = topological_node(a)
            b.edges = point["meta"]["edges"]
            b.waypoint = point["meta"]["waypoint"]
            b._get_coords()
            print b.name
            #if point["meta"]["name"] != "charging_point":
            points.append(b)

        return points


    def run_analysis(self):
        for i in self.lnodes:
            print ("%s:") %i.name
            ox=float(i.waypoint[0])
            oy=float(i.waypoint[1])
            print ("%.2f,%.2f") %(ox,oy)
            for j in i.edges:
                b = get_node(j["node"],self.lnodes)
                if b :
                    dist=b._get_distance(ox, oy)
                    print ("%s: %f") %(b.name,dist)

if __name__ == '__main__':
    filename=str(sys.argv[1])
    rospy.init_node('topological_localisation')
    server = TopologicalNavLoc(rospy.get_name(),filename)