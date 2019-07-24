#!/usr/bin/env python

import rospy
from topological_navigation.tmap_utils import *
from strands_navigation_msgs.msg import NavRoute


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

    def __init__(self, top_map) :
#        rospy.loginfo("Waiting for Topological map ...")
        self.top_map = top_map

    """
     search_route

     This function searches the route to reach the goal
    """
    def search_route(self, origin, target):
        goal = get_node(self.top_map, target)
        orig = get_node(self.top_map, origin)
        to_expand=[]
        children=[]
        expanded=[]

        #print 'searching route from %s to %s' %(orig.name, goal.name)

        #self.get_distance_to_node(goal, orig)
        nte = NodeToExpand(orig.name, 'none', 0.0, get_distance_to_node(goal, orig))  #Node to Expand
        expanded.append(nte)
        #to_expand.append(nte)

#        exp_index=0
        cen = orig      #currently expanded node

        children = get_conected_nodes(cen) #nodes current node is connected to
        #print children
        not_goal=True
        route_found=False
        while not_goal :
            if target in children:
                not_goal=False
                route_found=True
                cdist = get_distance_to_node(cen, goal)
                cnte = NodeToExpand(goal.name, nte.name, nte.current_distance+cdist, 0.0)  #Node to Expand
                expanded.append(cnte)
                #print "goal found"
            else :
                #print "Goal NOT found"
                for i in children:
                    been_expanded = False
                    for j in expanded:
                        if i == j.name:
                            been_expanded = True
                    for j in to_expand:
                        if i == j.name:
                            been_expanded = True

                    if not been_expanded:
                        nnn = get_node(self.top_map, i)
                        tdist = get_distance_to_node(goal, nnn)
                        cdist = get_distance_to_node(cen, nnn)
                        cnte = NodeToExpand(nnn.name, nte.name, nte.current_distance+cdist, tdist)  #Node to Expand
                        to_expand.append(cnte)
                        to_expand = sorted(to_expand, key=lambda node: node.cost)

                if len(to_expand)>0:
                    nte = to_expand.pop(0)
                    #print 'expanding node %s (%d)' %(nte.name,len(to_expand))
                    cen =  get_node(self.top_map, nte.name)
                    expanded.append(nte)
                    children = get_conected_nodes(cen)
                else:
                    not_goal=False
                    route_found=False

        route = NavRoute()
#        print "===== RESULT ====="
        if route_found:
            steps=[]
            val = len(expanded)-1
            steps.append(expanded[val])
            next_node = expanded[val].father
            #print next_node
            while next_node != 'none':
                for i in expanded:
                    if i.name == next_node :
                        steps.append(i)
                        next_node = i.father
                        break

            steps.reverse()
            val = len(steps)
            for i in range(1, val):
                edg=get_edges_between(self.top_map, steps[i].father, steps[i].name)
                route.source.append(steps[i].father)
                route.edge_id.append(edg[0].edge_id)
                #route.append(r)

            return route
        else:
            return None