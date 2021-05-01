#!/usr/bin/env python
#########################################################################################################
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
#########################################################################################################


#########################################################################################################
class TopologicalRouteSearch2(object):
    

    def __init__(self, top_map) :

        self.top_map = top_map


    def search_route(self, origin, target):
        """
        This function searches the route to reach the goal
        """
        route = NavRoute()

        if origin == "none" or target == "none" or origin == target:
            return route

        goal = get_node_from_tmap2(self.top_map, target)
        orig = get_node_from_tmap2(self.top_map, origin)
        to_expand=[]
        children=[]
        expanded=[]

        nte = NodeToExpand(orig["node"]["name"], 'none', 0.0, get_distance_to_node_tmap2(goal, orig)) # Node to Expand
        expanded.append(nte)

        cen = orig # currently expanded node
        children = get_conected_nodes_tmap2(cen) # nodes current node is connected to

        not_goal=True
        route_found=False
        while not_goal :
            if target in children:
                not_goal=False
                route_found=True
                cdist = get_distance_to_node_tmap2(cen, goal)
                cnte = NodeToExpand(goal["node"]["name"], nte.name, nte.current_distance+cdist, 0.0) # Node to Expand
                expanded.append(cnte)
            else :
                for i in children:
                    been_expanded = False
                    to_be_expanded = False
                    for j in expanded: # search in expanded
                        if i == j.name:
                            been_expanded = True
                            old_expand_node = j # found it. skip remaining
                            break
                    if not been_expanded:
                        # not in expanded, search in to_expand. can't be in both
                        for j in to_expand:
                            if i == j.name:
                                been_expanded = True
                                to_be_expanded = True
                                old_expand_node = j
                                # found it. skip remaining
                                break

                    if not been_expanded:
                        nnn = get_node_from_tmap2(self.top_map, i)
                        tdist = get_distance_to_node_tmap2(goal, nnn)
                        cdist = get_distance_to_node_tmap2(cen, nnn)
                        cnte = NodeToExpand(nnn["node"]["name"], nte.name, nte.current_distance+cdist, tdist) # Node to Expand
                        to_expand.append(cnte)
                        to_expand = sorted(to_expand, key=lambda node: node.cost)
                    else:
                        nnn = get_node_from_tmap2(self.top_map, i)
                        tdist = get_distance_to_node_tmap2(goal, nnn)
                        cdist = get_distance_to_node_tmap2(cen, nnn)
                        # update existing NTE with new data if a shorter route to it is found
                        if nte.current_distance+cdist < old_expand_node.current_distance:
                            old_expand_node.father = nte.name
                            old_expand_node.current_distance = nte.current_distance+cdist
                            old_expand_node.dist_to_target = tdist
                            old_expand_node.cost = old_expand_node.current_distance + old_expand_node.dist_to_target
                            if to_be_expanded: # re-sort to_expand with new costs
                                to_expand = sorted(to_expand, key=lambda node: node.cost)

                if len(to_expand)>0:
                    nte = to_expand.pop(0)
                    cen =  get_node_from_tmap2(self.top_map, nte.name)
                    expanded.append(nte)
                    children = get_conected_nodes_tmap2(cen)
                else:
                    not_goal=False
                    route_found=False

#        print "===== RESULT ====="
        if route_found:
            steps=[]
            val = len(expanded)-1
            steps.append(expanded[val])
            next_node = expanded[val].father

            while next_node != 'none':
                for i in expanded:
                    if i.name == next_node :
                        steps.append(i)
                        next_node = i.father
                        break

            steps.reverse()
            val = len(steps)
            for i in range(1, val):
                edg=get_edges_between_tmap2(self.top_map, steps[i].father, steps[i].name)
                route.source.append(steps[i].father)
                route.edge_id.append(edg[0]["edge_id"])
                
        return route
#########################################################################################################


#########################################################################################################
class RouteChecker(object):
    
    
    def __init__(self, tmap):
        
        self.edge_dict = {}
        for node in tmap["nodes"]:
            self.edge_dict[node["node"]["name"]] = []
            
            for edge in node["node"]["edges"]:
                item = {"node": edge["node"], "edge_id": edge["edge_id"]}
                self.edge_dict[node["node"]["name"]].append(item)
                
                
    def check_route(self, route):
        
        rospy.loginfo("Checking Nav Route ...")

        N = len(route.source)
        if N < 1 or N != len(route.edge_id):
            rospy.logerr("Invalid Route: Either the route is empty or the number of nodes do not equal the number of edge ids")
            return False
        
        if "" in route.source or "" in route.edge_id:
            rospy.logerr("Invalid Route: Empty string found in nodes or edge ids or both")
            return False
        
        for i in range(N-1):
            
            node = route.source[i]
            edge_id = route.edge_id[i]
            
            if node not in self.edge_dict:
                rospy.logerr("Invalid Route: Node {} does not exist".format(node))
                return False
                
            n = 0
            next_node = route.source[i+1]  
            for edge in self.edge_dict[node]:
                if edge["edge_id"] == edge_id and edge["node"] == next_node:
                    n += 1
                    
            if n != 1:
                rospy.logerr("Invalid Route: No edge from {} to {} with id {} found or multiple edges found".format(node, next_node, edge_id))
                return False
        
        final_node = route.source[-1]    
        final_edge_id = route.edge_id[-1]
        
        if final_node not in self.edge_dict:
            rospy.logerr("Invalid Route: Node {} does not exist".format(final_node))
            return False
        
        n = 0
        for edge in self.edge_dict[final_node]:
            if edge["edge_id"] == final_edge_id and edge["node"] in self.edge_dict:
                    n += 1
                    
        if n != 1:
            rospy.logerr("Invalid Route: No edge from {} with id {} found or its destination node does not exist or multiple edges found".format(final_node, final_edge_id))
            return False
        
        rospy.loginfo("Route is Valid")
        return True
#########################################################################################################