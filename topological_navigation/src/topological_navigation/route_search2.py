#!/usr/bin/env python
#########################################################################################################
import rospy
from topological_navigation.tmap_utils import *
from topological_navigation_msgs.msg import NavRoute


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
        self.nodes = {}
        for node in self.top_map["nodes"]:
            name = node["node"]["name"]
            self.nodes[name] = node
            
        self.children = {}
        for node in self.top_map["nodes"]:
            name = node["node"]["name"]
            self.children[name] = []
            for edge in node["node"]["edges"]:
                dist = get_distance_to_node_tmap2(self.nodes[name], self.nodes[edge["node"]])
                item = {"name": edge["node"], "distance": dist}
                self.children[name].append(item)


    def search_route(self, origin, target, avoid_edges=[]):
        """
        This function searches the route to reach the goal
        """
        route = NavRoute()

        if origin == "none" or target == "none" or origin == target:
            return route

        goal = self.get_node_from_tmap2(target)
        orig = self.get_node_from_tmap2(origin)
        to_expand=[]
        children=[]
        expanded=[]

        nte = NodeToExpand(orig["node"]["name"], 'none', 0.0, get_distance_to_node_tmap2(goal, orig)) # Node to Expand
        expanded.append(nte)

        cen = orig # currently expanded node
        children = self.get_connected_nodes_tmap2(cen) # nodes current node is connected to

        # get the pair of node (orig-dest) of the edges we need to avoid
        avoid_pairs = []
        for _edge in avoid_edges:
            avoid_pairs.append(
                get_node_names_from_edge_id_2(self.top_map, _edge)
            )

        # remove the nodes we want to avoid
        if origin in [p[0] for p in avoid_pairs]:
            for (_, _dest) in avoid_pairs:
                if _dest in children:
                    children.remove(_dest)

        not_goal=True
        route_found=False
        while not_goal:
            if target in children:
                not_goal=False
                route_found=True
                cdist = self.get_distance_to_node_tmap2(cen, goal)
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
                        nnn = self.get_node_from_tmap2(i)
                        tdist = get_distance_to_node_tmap2(goal, nnn)
                        cdist = self.get_distance_to_node_tmap2(cen, nnn)
                        cnte = NodeToExpand(nnn["node"]["name"], nte.name, nte.current_distance+cdist, tdist) # Node to Expand
                        to_expand.append(cnte)
                        to_expand = sorted(to_expand, key=lambda node: node.cost)
                    else:
                        nnn = self.get_node_from_tmap2(i)
                        tdist = get_distance_to_node_tmap2(goal, nnn)
                        cdist = self.get_distance_to_node_tmap2(cen, nnn)
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
                    cen =  self.get_node_from_tmap2(nte.name)
                    expanded.append(nte)
                    children = self.get_connected_nodes_tmap2(cen)
                    # remove the nodes we want to avoid
                    if nte.name in [p[0] for p in avoid_pairs]:
                        for (_, _dest) in avoid_pairs:
                            if _dest in children:
                                children.remove(_dest)
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
                edg=self.get_edges_between_tmap2(steps[i].father, steps[i].name)
                route.source.append(steps[i].father)
                route.edge_id.append(edg[0]["edge_id"])
                
        return route
    
    
    def get_node_from_tmap2(self, node_name):
        
        try:
            node = self.nodes[node_name]
        except Exception:
            node = None
        return node
    
    
    def get_edges_between_tmap2(self, nodea, nodeb):
        
        ab = []
        noda = self.get_node_from_tmap2(nodea)
        for j in noda["node"]["edges"]:
            if j["node"] == nodeb:
                ab.append(j)
        return ab
    
    
    def get_connected_nodes_tmap2(self, node):
        
        items = self.children[node["node"]["name"]]
        return [item["name"] for item in items]
    
    
    def get_distance_to_node_tmap2(self, nodea, nodeb):
        
        items = self.children[nodea["node"]["name"]]
        for item in items:
            if item["name"] == nodeb["node"]["name"]:
                return item["distance"]
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
        
        rospy.loginfo("Checking Route...")

        N = len(route.source)
        if N < 1 or N != len(route.edge_id):
            rospy.logerr("Invalid Route: Either the route is empty or the number of nodes do not equal the number of edge ids")
            return False
        
        if "" in route.source or "" in route.edge_id:
            rospy.logerr("Invalid Route: Empty string found in nodes or edge ids or both")
            return False
        
        for i in range(N-1):
            
            node = route.source[i]            
            if node not in self.edge_dict:
                rospy.logerr("Invalid Route: Node {} does not exist".format(node))
                return False
                
            n = 0
            edge_id = route.edge_id[i]
            next_node = route.source[i+1]  
            for edge in self.edge_dict[node]:
                if edge["edge_id"] == edge_id and edge["node"] == next_node:
                    n += 1
                    
            if n != 1:
                rospy.logerr("Invalid Route: No edge from {} to {} with id {} found or multiple edges found".format(node, next_node, edge_id))
                return False
        
        final_node = route.source[-1]            
        if final_node not in self.edge_dict:
            rospy.logerr("Invalid Route: Node {} does not exist".format(final_node))
            return False
        
        n = 0
        final_edge_id = route.edge_id[-1]
        for edge in self.edge_dict[final_node]:
            if edge["edge_id"] == final_edge_id and edge["node"] in self.edge_dict:
                n += 1
                    
        if n != 1:
            rospy.logerr("Invalid Route: No edge from {} with id {} found or its destination node does not exist or multiple edges found".format(final_node, final_edge_id))
            return False
        
        rospy.loginfo("Route is Valid")
        return True
#########################################################################################################

        
#########################################################################################################    
def get_route_distance(tmap, node_a, node_b):
    
    if node_a is None or node_b is None:
        return 10e5-1
    
    if node_a["node"]["name"] == node_b["node"]["name"]:
        return 0.0
    
    rsearch = TopologicalRouteSearch2(tmap)
    route = rsearch.search_route(node_a["node"]["name"], node_b["node"]["name"])
    
    if not route.source:
        return 10e5-1
    
    dist = 0
    for i in range(len(route.source)-1):
        node_1 = rsearch.get_node_from_tmap2(route.source[i])
        node_2 = rsearch.get_node_from_tmap2(route.source[i+1])
        dist += get_distance_to_node_tmap2(node_1, node_2)
    
    dist += get_distance_to_node_tmap2(rsearch.get_node_from_tmap2(route.source[-1]), node_b)
    return dist
#########################################################################################################