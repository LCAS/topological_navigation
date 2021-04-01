#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info:
# ----------------------------------

import numpy
import random
import copy
# import rospy
import topological_navigation.tmap_utils
import topological_navigation.route_search
import strands_navigation_msgs.msg
import topological_simpy.config_utils


class TopologicalForkGraph(object):
    """TopologicalForkGraph: A class to store and retreive information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one head lane and different rows.
    """
    def __init__(self, n_polytunnels, n_farm_rows, n_topo_nav_rows, second_head_lane, verbose):
        """TopologicalForkGraph: A class to store and retreive information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one/two head lane and different rows.

        Keyword arguments:

        n_polytunnels -- number of polytunnels, int
        n_farm_rows -- list of number of farm beds for each polytunnel, list of ints
        n_topo_nav_rows -- number of navigation rows, int
        second_head_lane -- uses a secondary head lane, bool
        verbose -- to control rosinfo, bool
        """
        ns = rospy.get_namespace()
        self.row_ids = ["row-%02d" %(i) for i in range(n_topo_nav_rows)]
        self.n_polytunnels = n_polytunnels
        self.n_farm_rows = n_farm_rows
        self.n_topo_nav_rows = n_topo_nav_rows
        self.second_head_lane = second_head_lane

        # half_rows: rows requiring picking in one direction
        self.half_rows = set()
        if self.n_polytunnels == 1:
            self.half_rows.add(self.row_ids[0])
            self.half_rows.add(self.row_ids[-1])
        else:
            row_num = 0
            for i in range(self.n_polytunnels):
                row_id = "row-%02d" %(row_num)
                self.half_rows.add(row_id)
                row_num += self.n_farm_rows[i]
                row_id = "row-%02d" %(row_num)
                self.half_rows.add(row_id)
                row_num += 1

        self.head_nodes = {}        # {row_id:[pri_head_node, sec_head_node]}
        self.row_nodes = {}         # {row_id:[row_nodes]}
        self._row_nodes = []        # list of all row nodes, for internal use
        # row_info {row_id:[pri_head_node, start_node, end_node, local_storage_node, sec_head_node]}
        self.row_info = {}
        # yield_at_node {node_id:yield_at_node}
        self.yield_at_node = {}
        # local storage nodes
        self.local_storages = {}
        self.local_storage_nodes = {row_id:None for row_id in self.row_ids}
        self.cold_storage = None
        self.cold_storage_node = None

        self.use_local_storage = True # if False, store at cold storage

        self.verbose = verbose

        self.topo_map = None
        self.node_index = {} # index of node in topo_map
        for i in range(10):
            try:
                self.topo_map = rospy.wait_for_message(ns + "topological_map", strands_navigation_msgs.msg.TopologicalMap, timeout=10)
            except:
                rospy.logerr(ns + "topological_map topic is not published?")
                rospy.sleep(0.1)
            else:
                self.loginfo("TopologicalForkGraph object ready")
                break

        if not self.topo_map:
            raise Exception(ns + "topological_map topic not received")

        if len(self.topo_map.nodes) == 0:
            raise Exception("No nodes in topo_map. Try relaunching topological_navigation nodes.")

        self.update_node_index()

        self.agent_nodes = {} # agent_id:agent.curr_node - should be updated by the agent

        self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)

    def update_node_index(self):
        """once topo_map is received, get the indices of nodes for easy access to node object"""
        for i in range(len(self.topo_map.nodes)):
            self.node_index[self.topo_map.nodes[i].name] = i

    def set_node_yields(self, node_yields):
        """set_node_yields: Set the yields at each node from the node yields
        given for each row / all rows

        Keyword arguments:

        node_yields -- yields per node distance for each row,
                            list of size n_topo_nav_rows or 1
        """
        node_yield_in_row = topological_simpy.config_utils.param_list_to_dict("node_yields", node_yields, self.row_ids)

        for row_id in self.row_ids:
            n_row_nodes = len(self.row_nodes[row_id])

            for j in range(n_row_nodes):
                node_id = self.row_nodes[row_id][j]
                # yield from each row node to next row node
                if j != n_row_nodes - 1:
#                    self.yield_at_node[node_id] = node_yield_in_row[row_id]
                    # adding Gaussian white noise to yield with std of 2% of node_yield
                    self.yield_at_node[node_id] = node_yield_in_row[row_id] + random.gauss(0, 0.02 * node_yield_in_row[row_id])
                else:
                    # between the last two nodes, the distance could be smaller than node_dist
                    row_node_dist = self.get_distance_between_adjacent_nodes(self.row_nodes[row_id][0],
                                                                             self.row_nodes[row_id][1])
                    last_node_dist = self.get_distance_between_adjacent_nodes(self.row_nodes[row_id][-2],
                                                                              self.row_nodes[row_id][-1])
#                    self.yield_at_node[node_id] = (node_yield_in_row[row_id] * last_node_dist) / row_node_dist
                    last_node_yield = (node_yield_in_row[row_id] * last_node_dist) / row_node_dist
                    # adding Gaussian white noise to yield with std of 2% of node_yield
                    self.yield_at_node[node_id] = last_node_yield + random.gauss(0, 0.02 * last_node_yield)

    def set_local_storages(self, local_storages):
        """set local_storage_nodes and local_storages

        Keyword arguments:

        local_storages -- simpy.Resource object list
        """
        # reset
        self.local_storage_nodes = {row_id:None for row_id in self.row_ids}
        self.local_storages = {}

        n_local_storages = len(local_storages)
        # set local storage nodes
        storage_row_groups = numpy.array_split(numpy.arange(self.n_topo_nav_rows), n_local_storages)

        for i in range(n_local_storages):
            start_row = storage_row_groups[i][0]
            end_row = storage_row_groups[i][-1]
            storage_row = "pri_hn-%02d" %(start_row + int((end_row - start_row) / 2))

            for row in storage_row_groups[i]:
                self.local_storage_nodes["row-%02d" %(row)] = storage_row
            self.local_storages[storage_row] = local_storages[i]

        for row_id in self.row_ids:
            self.row_info[row_id][3] = self.local_storage_nodes[row_id]

    def set_cold_storage(self, cold_storage):
        """set cold_storage_node and cold_storage

        Keyword arguments:

        cold_storage -- simpy.Resource object
        """
        self.cold_storage_node = "cold_storage"
        self.cold_storage = cold_storage
        self.use_local_storage = False

    def set_row_info(self):
        """set_row_info: Set information about each row
        {row_id: [pri_head_node, start_node, end_node, local_storage_node, sec_head_node]}

        Also sets
          head_nodes {row_id:[pri_head_node, sec_head_node]}
          row_nodes {row_id:[row_nodes]}
        """
        # TODO: meta information is not queried from the db now.
        # The row and head node names are hard coded now
        # An ugly way to sort the nodes is implemented
        # get_nodes in topological_utils.queries might be useful to get nodes with same tag
        self.head_nodes = {"row-%02d" %(i):[] for i in range(self.n_topo_nav_rows)}
        for i in range(self.n_topo_nav_rows):
            self.head_nodes["row-%02d" %(i)].append("pri_hn-%02d" %(i))
            if self.second_head_lane:
                self.head_nodes["row-%02d" %(i)].append("sec_hn-%02d" %(i))

        self.row_nodes = {"row-%02d" %(i):[] for i in range(self.n_topo_nav_rows)}

        for node in self.topo_map.nodes:
            for i in range(self.n_topo_nav_rows):
                if "rn-%02d" %(i) in node.name:
                    self.row_nodes["row-%02d" %(i)].append(node.name)
                    self._row_nodes.append(node.name)

        for row_id in self.row_ids:
            # assuming the node names can be directly sorted
            self.row_nodes[row_id].sort()
            # local_storage_nodes should be modified by calling set_local_storages
            self.row_info[row_id] = [self.head_nodes[row_id][0],
                                     self.row_nodes[row_id][0],
                                     self.row_nodes[row_id][-1],
                                     self.local_storage_nodes[row_id]]
            if self.second_head_lane:
                self.row_info[row_id].append(self.head_nodes[row_id][1])

    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:
        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_nodes = []
        route_edges = []
        route_distances = []
        if start_node == goal_node:
            return ([start_node], route_edges, route_distances)

        route = self.route_search.search_route(start_node, goal_node)
        if route is not None:
            route_nodes = route.source
            route_edges = route.edge_id

            # append goal_node to route_nodes
            route_nodes.append(goal_node)

            for i in range(len(route_nodes) - 1):
                route_distances.append(self.get_distance_between_adjacent_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distances)

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        return self.topo_map.nodes[self.node_index[node]]

    def get_distance_between_adjacent_nodes(self, from_node, to_node):
        """get_distance_between_adjacent_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return topological_navigation.tmap_utils.get_distance_to_node(from_node_obj, to_node_obj)

    def get_edges_between_nodes(self, from_node, to_node):
        """get_edges_between_nodes: Given names of two nodes, return the direct edges
        between their node objects. A wrapper for the get_edges_between function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        edge_ids = []
        edges = topological_navigation.tmap_utils.get_edges_between(self.topo_map, from_node, to_node)
        for edge in edges:
            edge_ids.append(edge.edge_id)
        return edge_ids

    def get_topo_map_no_agent_rownodes(self):
        """get the current topological map without the edges connecting to row nodes with agents.
        this map should be used for route_search for all route searches here."""
        dyn_map = copy.deepcopy(self.topo_map)         # copy of map to be modified
        agent_nodes = copy.deepcopy(self.agent_nodes)   # copy to avoid data inconsistency

        # get all nodes with an agent (picker/robot)
        row_nodes_with_agents = []
        for agent in agent_nodes:
            node = self.agent_nodes[agent]
            if (node in self._row_nodes) and (node not in row_nodes_with_agents):
                row_nodes_with_agents.append(node)

        for node in row_nodes_with_agents:
            # find neighbouring nodes and edges from the agent node
            nbr_nodes = []
            for edge in dyn_map.nodes[self.node_index[node]].edges:
                nbr_nodes.append(edge.node)
            # remove all edges
            dyn_map.nodes[self.node_index[node]].edges = []

            # for each neighbour, find and remove the edge back to the agent node
            for nbr_node in nbr_nodes:
                edge_indices = []
                for edge in dyn_map.nodes[self.node_index[nbr_node]].edges:
                    if edge.node == node:
                        edge_indices.append(dyn_map.nodes[self.node_index[nbr_node]].edges.index(edge))
                edge_indices.sort(reverse=True)
                for edge_index in edge_indices:
                    dyn_map.nodes[self.node_index[nbr_node]].edges.pop(edge_index)

        return dyn_map

    def get_path_details_no_agent_rownodes(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node without
        touching any row nodes with agents. the goal_node should not be a row node occupied by
        an agent.

        Keyword arguments:
        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distances = []
        route_nodes = []
        route_edges = []

        # route_search using modified topological map
        dyn_map = self.get_topo_map_no_agent_rownodes()
        router = topological_navigation.route_search.TopologicalRouteSearch(dyn_map)

        # find a path to the goal_node
        route = router.search_route(start_node, goal_node)
        if route is not None:
            route_nodes = route.source
            route_edges = route.edge_id

            # append goal_node to route_nodes
            route_nodes.append(goal_node)

            # sum the path distance
            for i in range(len(route_nodes) - 1):
                route_distances.append(self.get_distance_between_adjacent_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distances)

    def get_row_id_of_row_node(self, node):
        """given a row_node, return the row_id"""
        for row_id in self.row_ids:
            if node in self.row_nodes[row_id]:
                return row_id
        return None

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            rospy.loginfo(msg)
