#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info:
# ----------------------------------


import numpy
# import rospy
import topological_simpy.topo


class TopologicalForkGraphMimic(topological_simpy.topo.TopologicalForkGraph):
    """TopologicalForkGraphMimic: A derived class from class TopologicalForkGraph to mimic the
        operation in an actual / gazebo simulated environment. Some methods are overloaded
    """

    def __init__(self, n_polytunnels, n_farm_rows, n_topo_nav_rows, second_head_lane,
                 base_stations, wait_nodes, env, topo_map2_file, verbose=False):
        """TopologicalForkGraphMimic: A child class from TopologicalForkGraph class to store and retreive
        information of topological map, stored in the mongodb, necessary for the longer discrete event simulations
        mimicking longer picking operations with ros interfaces. Assumes a fork map with
        one/two head lane and different rows.

        Keyword arguments:

        n_polytunnels -- number of polytunnels, int
        n_farm_rows -- list of number of farm beds for each polytunnel, list of ints
        n_topo_nav_rows -- number of navigation rows, int
        second_head_lane -- uses a secondary head lane, bool
        verbose -- to control rosinfo, bool
        """
        super(TopologicalForkGraphMimic, self).__init__(n_polytunnels, n_farm_rows, n_topo_nav_rows, second_head_lane,
                                                        base_stations, wait_nodes, env,
                                                        topo_map2_file, verbose)

    def set_row_info(self, pri_head_nodes, row_nodes, sec_head_nodes=None):
        """set_row_info: Set information about each row
        {row_id: [pri_head_node, start_node, end_node, local_storage_node, sec_head_node]}

        Also sets
          head_nodes {row_id:[pri_head_node, sec_head_node]}
          row_nodes {row_id:[row_nodes]}

        Keyword arguments:

        pri_head_nodes -- , list
        row_nodes -- , list
        sec_head_nodes -- , list/None
        """
        # TODO: meta information is not queried from the db now.
        # The row and head node names are hard coded now
        # An ugly way to sort the nodes is implemented
        # get_nodes in topological_utils.queries might be useful to get nodes with same tag
        self.head_nodes = {"row-%02d" % (i): [] for i in range(self.n_topo_nav_rows)}
        for i in range(self.n_topo_nav_rows):
            self.head_nodes["row-%02d" % (i)].append(pri_head_nodes[i])
            if self.second_head_lane:
                assert sec_head_nodes is not None
                self.head_nodes["row-%02d" % (i)].append(sec_head_nodes[i])

        self.row_nodes = {"row-%02d" % (i): [] for i in range(self.n_topo_nav_rows)}

        # rospy.sleep(1) TODO: this verification process could be ignored in the SimPy model, as all nodes are retrieved directly
        for i in range(self.n_topo_nav_rows):
            #            rospy.sleep(0.3)
            for node_name in row_nodes[i]:
                node_found = False
                for node in self.tmap2['nodes']:
                    if node_name == node['node']['name']:
                        self.row_nodes["row-%02d" % (i)].append(node['node']['name'])
                        node_found = True
                        break
                if not node_found:
                    msg = "node - %s not found in topological_map topic" % node_name
                    # rospy.logerr(msg)
                    raise Exception(msg)

        for row_id in self.row_ids:
            # local_storage_nodes should be modified by calling set_local_storages
            self.row_info[row_id] = [self.head_nodes[row_id][0],
                                     self.row_nodes[row_id][0],
                                     self.row_nodes[row_id][-1],
                                     self.local_storage_nodes[row_id]]
            if self.second_head_lane:
                self.row_info[row_id].append(self.head_nodes[row_id][1])

    def set_local_storages(self, local_storages, local_storage_nodes):
        """set local_storage_nodes and local_storages

        Keyword arguments:

        local_storages -- simpy.Resource object list
        """
        assert len(local_storages) == len(local_storage_nodes)
        # reset
        self.local_storage_nodes = {row_id: None for row_id in self.row_ids}
        self.local_storages = {}

        # set local storage nodes
        # for start_node of each row, find the distance to all local storages
        # assign the nearest local storage node to the row
        storage_nodes = {}
        count = 0
        for storage in local_storage_nodes:
            storage_nodes[storage] = self.get_node(storage)
            self.local_storages[storage] = local_storages[count]
            count += 1

        for row_id in self.row_ids:
            dist = float("inf")
            head_node = self.get_node(self.head_nodes[row_id][0])
            for storage in local_storage_nodes:
                dist_to_storage = numpy.hypot(
                    (head_node['node']["pose"]["position"]["x"] - storage_nodes[storage]['node']["pose"]["position"]["x"]),
                    (head_node['node']["pose"]["position"]["y"] - storage_nodes[storage]['node']["pose"]["position"]["y"]))
                if dist_to_storage < dist:
                    dist = dist_to_storage
                    self.local_storage_nodes[row_id] = storage + ""

            self.row_info[row_id][3] = self.local_storage_nodes[row_id]

    def set_cold_storage(self, cold_storage, cold_storage_node):
        """set cold_storage_node and cold_storage

        Keyword arguments:

        cold_storage -- simpy.Resource object
        """
        self.cold_storage_node = cold_storage_node
        self.cold_storage = cold_storage
        self.use_local_storage = False
        
    def get_row_head_node_of_row_node(self, node):
        """get the head node of the current row of the node

        Keyword arguments:

        node -- node from any row
        """        
        row_id = self.get_row_id_of_row_node(node)
        if row_id is not None:
            return self.head_nodes[row_id][0]
        else:
            return None
