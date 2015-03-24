#!/usr/bin/env python

import sys
import rospy
#import actionlib
#import pymongo
#import json
import sys
import math


#from geometry_msgs.msg import Pose
from std_msgs.msg import String
#import scitos_apps_msgs.msg

#from strands_navigation_msgs.msg import TopologicalNode
#from mongodb_store.message_store import MessageStoreProxy

from strands_navigation_msgs.msg import TopologicalMap
#from topological_navigation.topological_node import *

#import topological_navigation.msg


class NodeToExpand(object):
    def __init__(self, name, father, current_distance, dist_to_target):
        self.name = name
        self.expanded=False
        self.father=father
        self.current_distance = current_distance
        self.dist_to_target = dist_to_target
        self.cost = self.current_distance + self.dist_to_target

    def __repr__(self):
        return "-------\n\t Node: \n\t name:%s \n\t Father:%s \n\t current_distance:%f \n\t distance to target: %f \n\t cost %f \n" %(self.name, self.father, self.current_distance, self.dist_to_target, self.cost)
        

class TopologicalRouteSearch(object):
       
    def __init__(self, goal) :     
        rospy.loginfo("Waiting for Topological map ...")

        try:
            msg = rospy.wait_for_message('/topological_map', TopologicalMap, timeout=10.0)
            self.lnodes = msg.nodes
        except rospy.ROSException :
            rospy.logwarn("Failed to get topological map")
            return

        rospy.loginfo("Waiting for Current Node ...")
              
        try:
            msg = rospy.wait_for_message('/closest_node', String, timeout=10.0)
            cnode = msg.data
        except rospy.ROSException :
            rospy.logwarn("Failed to get closest node")
            return
            

        self.search_route(cnode, goal)
        rospy.loginfo("All Done ...")
        #rospy.spin()


    def get_node(self, target):
        for i in self.lnodes:
            if i.name == target:
                return i
        return None


    def get_children(self, node):
        childs=[]
        for i in node.edges :
            childs.append(i.node)
        return childs


    def get_distance_to_node(self, nodea, nodeb):
        dist=math.hypot((nodeb.pose.position.x-nodea.pose.position.x),(nodeb.pose.position.y-nodea.pose.position.y))
        return dist

    """
     search_route
     
     This function takes the search the route to reach the goal
    """
    def search_route(self, origin, target):
        goal = self.get_node(target)
        orig = self.get_node(origin)
        to_expand=[]
        children=[]
        expanded=[]
        
        print 'searching route from %s to %s' %(orig.name, goal.name)
        
        self.get_distance_to_node(goal, orig)
        nte = NodeToExpand(orig.name, 'none', 0.0, self.get_distance_to_node(goal, orig))  #Node to Expand
        expanded.append(nte)
        #to_expand.append(nte)
        
#        exp_index=0
        cen = orig      #currently expanded node 
        
        children = self.get_children(cen) #nodes current node is connected to
        #print children
        not_goal=True
        while not_goal :
            if target in children:
                not_goal=False
                cdist = self.get_distance_to_node(cen, goal)
                cnte = NodeToExpand(goal.name, nte.name, nte.current_distance+cdist, 0.0)  #Node to Expand
                expanded.append(cnte)
                print "goal found"
            else :
                print "Goal NOT found"
                for i in children:
                    been_expanded = False
                    for j in expanded:
                        if i == j.name:
                            been_expanded = True
                    for j in to_expand:
                        if i == j.name:
                            been_expanded = True
                            
                    if not been_expanded:
                        nnn = self.get_node(i)
                        tdist = self.get_distance_to_node(goal, nnn)
                        cdist = self.get_distance_to_node(cen, nnn)
                        cnte = NodeToExpand(nnn.name, nte.name, nte.current_distance+cdist, tdist)  #Node to Expand
                        to_expand.append(cnte)
                        to_expand = sorted(to_expand, key=lambda node: node.cost)

                if len(to_expand)>0:
                    nte = to_expand.pop(0)
                    print 'expanding node %s (%d)' %(nte.name,len(to_expand))
                    cen =  self.get_node(nte.name)
                    expanded.append(nte)
                    children = self.get_children(cen)
                else:
                    not_goal=False

        for i in to_expand:
            print i
            
        print "===== RESULT ====="
        route=[]
        val = len(expanded)-1
        route.append(expanded[val])
        next_node = expanded[val].father
        #print next_node
        while next_node != 'none':
            for i in expanded:
                if i.name == next_node :
                    route.append(i)
                    next_node = i.father
                    break
        
        route.reverse()
        for i in route:
            print i.name
        


if __name__ == '__main__':
    rospy.init_node('route_search')
    goal =  str(sys.argv[1])
    searcher = TopologicalRouteSearch(goal)