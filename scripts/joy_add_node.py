#!/usr/bin/env python

import sys
import rospy

import std_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import Pose

#from strands_navigation_msgs.msg import TopologicalNode
#from ros_datacentre.message_store import MessageStoreProxy
#from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalMap
from topological_navigation.topological_map import *


#import topological_navigation.msg



class topologicalNodeAdd(object):

    def __init__(self, base_name, dist) :
        self.map_received = False
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.JoyCallback)      
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      

        self.button = rospy.get_param('~button', 1)

        rospy.loginfo("Waiting for Topological map ...")        
        while not self.map_received :
            pass

        rospy.loginfo("All Done ...")
        rospy.spin()


    def JoyCallback(self, msg) :
        if msg.buttons[self.button]:
            #print "JOY"
            self.add_node()

    def MapCallback(self, msg) :
        self.topo_map = topological_map(msg.name, msg=msg)
        self.map_received =True


    def add_node(self) :

        try:
            current = rospy.wait_for_message('/current_node', std_msgs.msg.String, timeout=10.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get current node")
            return
        
        print current.data
        if current.data == 'none':
            try:
                pos = rospy.wait_for_message('/robot_pose', Pose, timeout=10.0)
            except rospy.ROSException :
                rospy.logwarn("Failed to get robot pose")
                return
            
            lnames=[]
            for i in self.topo_map.nodes :
                 if i.name.startswith('WayPoint') :
                     nam = i.name.strip('WayPoint')
                     lnames.append(int(nam))
            
            lnames.sort()
            print "used names:"
            for j in lnames :
                print j
            
            print "Chosen name!!!!!!"
            if lnames:
                nodname = 'WayPoint%d'%(int(lnames[-1])+1)
            else :
                nodname = 'WayPoint1'
            print nodname
            #self.topo_map = topological_map(pointset)     
            self.topo_map.add_node(nodname,8.0, pos, 'move_base')
        
            map_update = rospy.Publisher('/update_map', std_msgs.msg.Time)        
            map_update.publish(rospy.Time.now())            
        
        else:
            rospy.loginfo("I will NOT add a node within the influence area of another! Solve this and try again!")



if __name__ == '__main__':
#    pointset=str(sys.argv[1])
#    name=str(sys.argv[2])
#    dist=float(sys.argv[3])
    name = 'WayPoint'
    dist = 10.0
    rospy.init_node('node_add')
    server = topologicalNodeAdd(name,dist)