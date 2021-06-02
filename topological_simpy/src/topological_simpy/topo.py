#!/usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide, ZuyuanZhu
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info: 01-Apr-2021, updates @ZuyuanZhu:
#        1. model topological map node as SimPy.container
#        2. add node occupying time monitor for better route planning
#        3. update TopologicalRouteSearch to TopologicalRouteSearch2
#
# ----------------------------------

import numpy
import random
import copy
from topological_navigation.tmap_utils import *
import topological_simpy.config_utils
from yaml import safe_load
from topological_navigation.route_search2 import TopologicalRouteSearch2
from math import hypot
from functools import partial, wraps
import simpy


def patch_resource(resource, pre=None, post=None):
    """Patch *resource* so that it calls the callable *pre* before each
     put/get/request/release operation and the callable *post* after each
     operation.  The only argument to these functions is the resource
     instance.

    """

    def get_wrapper(func):
        # Generate a wrapper for put/get/request/release
        @wraps(func)
        def wrapper(*args, **kwargs):
            # This is the actual wrapper
            # Call "pre" callback
            if pre:
                pre(resource)

            # Perform actual operation
            ret = func(*args, **kwargs)

            # Call "post" callback
            if post:
                post(resource)

            return ret

        return wrapper

    # Replace the original operations with our wrapper
    for name in ['put', 'get', 'request', 'release']:
        if hasattr(resource, name):
            setattr(resource, name, get_wrapper(getattr(resource, name)))


def patch_res_put(resource, pre=None, post=None):
    """Patch *resource* so that it calls the callable *pre* before each
     put/get/request/release operation and the callable *post* after each
     operation.  The only argument to these functions is the resource
     instance.

    """

    def get_wrapper(func):
        # Generate a wrapper for put
        @wraps(func)
        def wrapper(*args, **kwargs):
            # This is the actual wrapper
            # Call "pre" callback
            if pre:
                pre(resource)

            # Perform actual operation
            ret = func(*args, **kwargs)

            # Call "post" callback
            if post:
                post(resource)

            return ret

        return wrapper

    # Replace the original operations with our wrapper
    name = 'put'
    if hasattr(resource, name):
        setattr(resource, name, get_wrapper(getattr(resource, name)))


def patch_res_get(resource, pre=None, post=None):
    """Patch *resource* so that it calls the callable *pre* before each
     put/get/request/release operation and the callable *post* after each
     operation.  The only argument to these functions is the resource
     instance.

    """

    def get_wrapper(func):
        # Generate a wrapper for get
        @wraps(func)
        def wrapper(*args, **kwargs):
            # This is the actual wrapper
            # Call "pre" callback
            if pre:
                pre(resource)

            # Perform actual operation
            ret = func(*args, **kwargs)

            # Call "post" callback
            if post:
                post(resource)

            return ret

        return wrapper

    # Replace the original operations with our wrapper
    name = 'get'
    if hasattr(resource, name):
        setattr(resource, name, get_wrapper(getattr(resource, name)))


class TopologicalForkGraph(object):
    """TopologicalForkGraph: A class to store and retrieve information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one head lane and different rows.
    """

    def __init__(self, n_polytunnels, n_farm_rows, n_topo_nav_rows, second_head_lane,
                 base_stations, wait_nodes, env, topo_map2_file, verbose):
        """TopologicalForkGraph: A class to store and retrieve information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one/two head lane and different rows.

        Keyword arguments:

        n_polytunnels -- number of polytunnels, int
        n_farm_rows -- list of number of farm beds for each polytunnel, list of ints
        n_topo_nav_rows -- number of navigation rows, int
        second_head_lane -- uses a secondary head lane, bool
        verbose -- to control rosinfo, bool
        """
        # ns = rospy.get_namespace()
        with open(topo_map2_file, "r") as f:
            self.tmap2 = safe_load(f)
        self._route_planner = TopologicalRouteSearch2(self.tmap2)

        self.env = env

        self._nodes = sorted([n['node']['name'] for n in self.tmap2['nodes']])
        self._nodes_dict = {n['node']['name']: n['node'] for n in self.tmap2['nodes']}
        self._node_res = {}

        self._node_log = {}
        self._hold = {}
        self.active_nodes = {}   # keep all the occupied nodes for checking that any two robots requesting each other's nodes

        self.req_ret = {}  # integer, request return: mode of the node requested
        self.deadlocks = {'deadlock_robot_ids': [],
                          'robot_occupied_nodes': {}}  # robots and their nodes who are in deadlock
        self.couple_robots = []  # robots who are in deadlock
        self.com_nodes = []  # the nodes that hold completed robots

        self.base_stations = base_stations
        self.wait_nodes = wait_nodes
        self.cold_storage_usage_queue = []   # robots that need to use cold storage for unloading will join the queue

        if self.env:
            for n in self._nodes:
                self._node_res[n] = simpy.Container(self.env, capacity=1, init=0)  # each node can hold one robot max
                self._node_log[n] = []
                self.req_ret[n] = int()
                self._hold[n] = {'now': [],
                                 'hold_time': [],
                                 'queue_time': [],
                                 'start_moment': [],
                                 'wait_time': []}
                patch_resource(self._node_res[n], post=partial(self.log_resource, n))
                patch_res_put(self._node_res[n], post=partial(self.put_monitor, n))
                patch_res_get(self._node_res[n], post=partial(self.get_monitor, n))

        """ inherited from previous topo map"""
        self.row_ids = ["row-%02d" % i for i in range(n_topo_nav_rows)]
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
                row_id = "row-%02d" % row_num
                self.half_rows.add(row_id)
                row_num += self.n_farm_rows[i]
                row_id = "row-%02d" % row_num
                self.half_rows.add(row_id)
                row_num += 1

        self.head_nodes = {}  # {row_id:[pri_head_node, sec_head_node]}
        self.row_nodes = {}  # {row_id:[row_nodes]}
        self._row_nodes = []  # list of all row nodes, for internal use
        # row_info {row_id:[pri_head_node, start_node, end_node, local_storage_node, sec_head_node]}
        self.row_info = {}
        # yield_at_node {node_id:yield_at_node}
        self.yield_at_node = {}
        # local storage nodes
        self.local_storages = {}
        self.local_storage_nodes = {row_id: None for row_id in self.row_ids}
        self.cold_storage = None
        self.cold_storage_node = None

        self.use_local_storage = True  # if False, store at cold storage

        self.verbose = verbose

        self.node_index = {}  # index of node in topo_map

        self.update_node_index()

        self.agent_nodes = {}  # agent_id:agent.curr_node - should be updated by the agent

        """
        The bellow methods are used for model topological nodes as containers, each container can only hold one robot.
        Pickers don't use this container feature but use the same topological map. 
        """
    def add_cold_storage_usage_queue(self, robot_id):
        """
        robots that need to use cold storage for unloading will join the queue
        """
        self.cold_storage_usage_queue.append(robot_id)

    def remove_cold_storage_usage_queue(self, robot_id):
        """
        robots that have used cold storage for unloading will be removed from the queue
        """
        if robot_id in self.cold_storage_usage_queue:
            self.cold_storage_usage_queue.remove(robot_id)
        else:
            msg = "%s not in cold_storage_usage_queue: %s" % (robot_id, self.cold_storage_usage_queue)
            raise Exception(msg)

    def get_cold_storage_usage_queue_head(self):
        """
        get the first element of the queue
        """
        if len(self.cold_storage_usage_queue) > 0:
            return self.cold_storage_usage_queue[0]
        else:
            return None

    def get_node_res_level(self, node):
        """
        get the level of the node resource
        """
        return self._node_res[node].level

    def get_node_res_capacity(self, node):
        """
        get the capacity of the node resource
        """
        return self._node_res[node].capacity

    def log_resource(self, node, resource):
        """
        This is our monitoring callback. Log all put and get operations with container resource.
        :param node: string, topological map node
        :param resource: SimPy resource Container
        """
        item = (
            round(self.env.now, 1),
            resource.level
        )
        self._node_log[node].append(item)

    def put_monitor(self, node, resource):
        """
        Callback when request a node
        :param node: string, topological map node
        :param resource: SimPy resource Container
        """
        self.put_hold_info(node, self._hold[node]['hold_time'][-1], self.req_ret[node])

    def get_monitor(self, node, resource):
        """
        Callback when release a node
        :param node: string, topological map node
        :param resource: SimPy resource Container
        """
        self.adj_hold_info(node, self._hold[node]['hold_time'][-1], self.req_ret[node])

    def set_hold_time(self, n, hold_t):
        """
        Set the time that the robot plan to occupy the node
        :param n: string, topological node
        :param hold_t: integer, the time that robot plan to occupy the node
        return: integer list, the hold time that has been set
        """
        return self._hold[n]['hold_time'].append(hold_t)

    def cancel_hold_time(self, n, hold_t):
        """
        Cancel the hold_time that just be set
        :param n: string, topological node
        :param hold_t: integer, the time that robot plan to occupy the node
        return: True, cancel succeed
        """
        if self._hold[n]['hold_time'][-1] == hold_t:
            self._hold[n]['hold_time'].pop()
            return True

    def get_wait_time(self, node):
        """
        Get the time that the robot should wait before occupy the node
        :param node: string, topological node name
        return: integer, wait time that the robot should wait before occupying the node
        """
        if self._hold[node]['wait_time'][-1] is not None:   # if self.req_ret[n] is 1: wait_time = 0
            now = self.env.now
            wait_time = 0  # self.req_ret[n] == 1
            queue_time = 0
            start_moment = self._hold[node]['start_moment'][-1]
            if self.req_ret[node] is 2:
                now = self.env.now
                queue_time = 0
                wait_time = queue_time + self._hold[node]['hold_time'][-2] - (now - start_moment)
            elif self.req_ret[node] > 2:
                num = self.req_ret[node] - 2
                for i in range(num):
                    queue_time = queue_time + self._hold[node]['hold_time'][-2-i]
                wait_time = self._hold[node]['hold_time'][-2-num] + queue_time - (now - start_moment)
            if wait_time < 0:
                print('! %5.1f: %s has held longer than expected! Extend the wait time as before' % (
                    self.env.now, node))
                wait_time = self._hold[node]['hold_time'][-2]  # Reset the wait time
                # TODO: better estimation of wait_time needed:
                #  e.g., 1. when the agent reaches base station, what the wait_time should be?
                #        2. when the agent is in deadlocking or waiting for another agent who is in deadlocking, what
                #           the wait_time should be?

            return wait_time

    def adj_hold_info(self, node_name, hold_time, req_ret):
        """
        Adjust holt information, whenever release a node, clear the corresponding occupy info
        :param node_name: string, topological map node
        :param hold_time: integer, the time that robot plan to hold the node, to be used later
        :param req_ret: integer, the return of requesting a node,
                        indicating the level and queue info of the node container
        return: dictionary, the current holding info of the node
        """
        if req_ret is 1:
            for r in self._hold[node_name]:
                self._hold[node_name][r].pop(0)
            self.req_ret[node_name] = self.req_ret[node_name] - 1
            return self._hold

        elif req_ret > 1:
            for r in self._hold[node_name]:
                self._hold[node_name][r].pop(0)
            if len(self._hold[node_name]['start_moment']):
                self._hold[node_name]['start_moment'][-1] = self._node_log[node_name][-1][0]
                self._hold[node_name]['now'][0] = self.env.now
            self.req_ret[node_name] = self.req_ret[node_name] - 1
            return self._hold

        # elif req_ret is 3:
        #     for r in self._hold[node_name]:
        #         self._hold[node_name][r].pop(0)
        #     if len(self._hold[node_name]['start_moment']):
        #         self._hold[node_name]['start_moment'][-1] = self._node_log[node_name][-1][0]
        #         self._hold[node_name]['now'][0] = self.env.now
        #     self.req_ret[node_name] = self.req_ret[node_name] - 1
        #     return self._hold

    def put_hold_info(self, node_name, hold_time, req_ret):
        """
        Log the waiting time for estimating the time cost when plan a new route
        :param node_name: string, name of the node to be hold
        :param hold_time: integer, total time that the node will be hold by the robot
        :param req_ret: integer, [1,2,3,4], level and queue info of requested node
        return: time info about the node, especially wait_time - how long a robot should wait
                before the node being released
        queue_time: the time that robot needs to wait before the current queue gone, robot cannot acquire the node now
        wait_time: the time that robot needs to wait before occupying the node
        hold_time: the time that the robot will hold the node if the robot occupies the node, not used at present
        start_moment: the moment that the current occupied robot started occupying the node
        """
        queue_time = 0
        wait_time = 0
        if req_ret is 1:  # the robot himself occupies the node
            now = self.env.now
            start_moment = now

        # elif req_ret is 2:
        #     now = self.env.now
        #     queue_time = 0
        #     start_moment = self._hold[node_name]['start_moment'][-1]
        #     wait_time = self._hold[node_name]['hold_time'][-1] - (now - start_moment)

        # elif req_ret is 3:
        #     now = self.env.now
        #     queue_time = self._hold[node_name]['hold_time'][-1]  # the last robot's hold_time
        #     start_moment = self._hold[node_name]['start_moment'][-1]
        #     wait_time = self._hold[node_name]['hold_time'][-2] + queue_time - (now - start_moment)

        elif req_ret > 1:
            n = req_ret - 2
            now = self.env.now
            for i in range(n):
                queue_time = queue_time + self._hold[node_name]['hold_time'][-i-1]
            start_moment = self._hold[node_name]['start_moment'][-1]
            wait_time = self._hold[node_name]['hold_time'][-n-1] + queue_time - (now - start_moment)

        if wait_time < 0:
            print('! %5.1f: %s has held longer than expected! Extend the wait time as before' % (
                self.env.now, node_name))
            wait_time = self._hold[node_name]['hold_time'][-2]  # Reset the wait time

        self._hold[node_name]['now'].append(self.env.now)
        self._hold[node_name]['queue_time'].append(queue_time)
        self._hold[node_name]['start_moment'].append(start_moment)
        self._hold[node_name]['wait_time'].append(wait_time)

        return self._hold

    def get_nodes(self):
        """
        get the topological map nodes
        return: string list, topological map node
        """
        return self._nodes

    def get_active_nodes(self, robot_ids_to_remove=[]):
        """
        get the nodes occupied by robots who are not in the list of robot_ids_to_remove
        """
        active_nodes = []
        active_nodes_to_remove = []
        for robot_id in self.active_nodes:
            if robot_id not in robot_ids_to_remove:
                active_nodes += self.active_nodes[robot_id]
        active_nodes = list(dict.fromkeys(active_nodes))
        for robot_id in robot_ids_to_remove:
            active_nodes_to_remove += self.active_nodes[robot_id]
        active_nodes_to_remove = list(dict.fromkeys(active_nodes_to_remove))
        for node in active_nodes_to_remove:
            if node in active_nodes:
                active_nodes.remove(node)
        return active_nodes

    def distance(self, nodeA, nodeB):
        """
        get the distance between node A and node B
        :param nodeA: string, topological map node name
        :param nodeB: string, topological map node name
        return: float, euclidean distance
        """
        a = self._nodes_dict[nodeA]
        b = self._nodes_dict[nodeB]
        return hypot(
            a['pose']['position']['x'] - b['pose']['position']['x'],
            a['pose']['position']['y'] - b['pose']['position']['y']
        )

    def route_dist_cost(self, route_nodes):
        """
        get the total distance cost of the route from node to node
        :param route_nodes: string list, the route nodes
        return: float, distance cost of the route
        """
        cost = 0.0
        if len(route_nodes) is 0:
            msg = "route_nodes: %s, please provide at least 1 node" % route_nodes
            raise Exception(msg)
        if len(route_nodes) is 1:
            return cost
        else:
            for idx, n in enumerate(route_nodes[:-1]):
                cost = cost + self.distance(n, route_nodes[idx+1])
            return cost

    def time_cost_to_dist(self, time_cost, speed_m_s):
        """
        Convert waiting-time cost to distance cost for searching a new route
        :param time_cost: int/float, the waiting time that the robot will be spending when waiting
                          for the requested node to be released
        :param speed_m_s: float, robot forward speed m/s
        return: float, distance cost
        """
        if self.env:
            return time_cost * speed_m_s

    def get_avoiding_route(self, origin, target, avoid_nodes):
        """
        get the topological route from origin to target but avoiding avoid_node
        :param origin: string, origin topological node
        :param target: string, target topological node
        :param avoid_nodes: string, topological node that should be avoided when planning the route
        return: string list, route node names, from current node to target node
        """
        dynamic_map = self.get_topo_map_no_occupied_rownodes(avoid_nodes)
        route_planner = TopologicalRouteSearch2(dynamic_map)
        return route_planner.search_route(origin, target)

    def get_com_nodes(self):
        """
        Get nodes that robot completed and stops finally
        """
        return self.com_nodes

    def add_com_node(self, node):
        """
        Add node to the com_nodes list
        """
        self.com_nodes.append(node)

    def get_node_state(self, node):
        """
        Get the status of node level and put_queue
        :param node: string, topological map node
        return: integer, the level and queue info of the node container
        """
        state = None
        if self._node_res[node]:
            if self._node_res[node].capacity - self._node_res[node].level > 0 and self._node_res[node].put_queue == []:
                state = 1
            elif self._node_res[node].capacity - self._node_res[node].level == 0 and self._node_res[node].put_queue == []:
                state = 2
            elif self._node_res[node].capacity - self._node_res[node].level == 0 and self._node_res[node].put_queue != []:
                state = 2 + len(self._node_res[node].put_queue)
            self.req_ret[node] = state
            return state
        else:
            return self.env.timeout(0)

    def request_node(self, robot_name, node):
        """
        Put one robot into the _node_res, i.e., the node resource is occupied by the robot.
        Note: the robot starts requesting the next route node when the robot reaches the current node
        :param node: string, the node to be requested
        :param robot_name: string, robot name
        return: SimPy put event, put the robot into the node container
        """
        self.add_act_node(robot_name, node)
        if self.is_in_deadlock(robot_name):
            self.loginfo('> %5.1f: %s is in deadlock, prepare to change route' % (self.env.now, robot_name))
            self.pop_free_node(robot_name, node, deadlock=True)  # pop the node planned to request
            self.add_robot_to_deadlock(robot_name)
            return self.env.timeout(0)
        if self.env:
            return self._node_res[node].put(1)
        else:
            return self.env.timeout(0)

    def release_node(self, robot_name, node):
        """
        Get the robot out of the _node_res, i.e., the node resource is released, the robot leaves this node
        :param node: string, the node to be requested
        :param robot_name: string, robot name
        return: SimPy get event, get the robot out of the node container
        """
        self.pop_free_node(robot_name, node)
        if self.env:
            if self._node_res[node].level > 0:
                return self._node_res[node].get(1)
            else:
                return self.env.timeout(0)

    def get_route(self, origin, target):
        """
        Get the topological map route from origin to target
        :param origin: string, topological map node
        :param target: string, topological map node
        return: string list, topological map route
        """
        return self._route_planner.search_route(origin, target)

    def monitor(self, freq=2):
        """
        Get the topological map nodes that occupied by the robot currently
        :param freq: to be used
        return: string list, the current occupied nodes by the robot
        """
        occupied_nodes = []
        for n in self._node_res:
            if self._node_res[n].level > 0:
                occupied_nodes.append(n)
        print("  %5.1f:    nodes     %s      are occupied" % (self.env.now, sorted(occupied_nodes)))
        return occupied_nodes

    def add_robot_to_deadlock(self, robot_name):
        """
        Add robots who are in deadlock to the deadlocks, along with the robots' occupied nodes
        :param robot_name: string, robot name
        """
        if len(self.couple_robots) > 0:
            for couple in self.couple_robots:
                if robot_name in couple:
                    self.deadlocks['deadlock_robot_ids'] = couple
                    for robot_id in couple:
                        self.deadlocks['robot_occupied_nodes'][robot_id] = self.active_nodes[robot_id]
        else:
            msg = "Error: robot %s not in deadlock, can not be added to deadlocks" % robot_name
            raise Exception(msg)

    def remove_robot_from_deadlock(self, robot_name):
        """
        Remove robot from deadlock list, and empty the deadlocks
        :param robot_name: string, robot name
        """
        if robot_name in self.deadlocks['deadlock_robot_ids']:
            self.deadlocks = {'deadlock_robot_ids': [],
                              'robot_occupied_nodes': {}}
        else:
            msg = "Error: robot %s not in deadlock, can not be removed from deadlocks" % robot_name
            raise Exception(msg)

    def add_act_node(self, robot_name, node):
        """
        Add active nodes to the shared list active_nodes
        :param robot_name: string, robot name
        :param node: string, topological node
        return: string list, the list of nodes that currently occupied or requested by robots
        """
        self.active_nodes[robot_name].append(node)
        return self.active_nodes

    def pop_free_node(self, robot_name, cur_node, deadlock=False):
        """
        Pop out the node from the active node list when a robot release the node
        :param robot_name: string, robot name
        :param cur_node: string, topological node that the robot release
        :param deadlock: bool, True - the robot is in the deadlock
                          False - the robot is not in the deadlock
        return: bool
        """
        if self.active_nodes[robot_name]:
            if deadlock and cur_node == self.active_nodes[robot_name].pop(-1):
                return True
            elif cur_node == self.active_nodes[robot_name].pop(0):
                return True
        else:
            print("Robot %s is not hold by any node" % robot_name)
            return False

    def get_couples(self, active_nodes):
        """
        Find two robots that are requesting each other's node, return all the couple nodes
        :param active_nodes: string list, the list of nodes that currently occupied or requested by robots
        return: string list of list, the couples of robots that are requesting each others' nodes
        """
        couple_robots = []
        self.couple_robots = []
        for a in active_nodes:
            if len(active_nodes[a]) == 2:
                for b in active_nodes:
                    if len(active_nodes[b]) == 2:
                        if a != b:
                            if active_nodes[a][0] == active_nodes[b][1] and active_nodes[a][1] == active_nodes[b][0]:
                                if ([a, b] not in couple_robots) and ([b, a] not in couple_robots):
                                    couple_robots.append([a, b])

        if couple_robots:
            self.couple_robots = couple_robots
        return self.couple_robots

    def is_in_deadlock(self, robot_name):
        """
        Check if the robot is in deadlock, waiting for each other
        :param robot_name: string, robot name
        return: bool, True - the robot is in deadlock
        """
        deadlock = self.get_couples(self.active_nodes)
        if deadlock:
            for c in deadlock:
                for r in c:
                    if robot_name == r:
                        return True
        else:
            return False

    def get_deadlocks(self, active_nodes):
        """
        Get the deadlock loop:
        Two robots in deadlock: A->B->A
        Three robots in deadlock: A->B->C->A
        Four robots in deadlock: A->B->C->D->A
        ...

        :param active_nodes: dict of list, {robot_id: [curr_node, requested_node]}
        return:  string list, robot_ids in deadlock
        """
        # get all the agents that have two occupied nodes: one for current, one for requested or prepare to request
        # act_nodes = {a: active_nodes[a] for a in active_nodes if len(active_nodes[a]) == 2}
        # deadlock_agents = []
        # agents = []
        # for agent in active_nodes:
        #     agents.append(agent)
        # while len(agents):
        #     TODO

    @property
    def node_log(self):
        return self._node_log

    """ inherited from previous topo class"""
    def update_node_index(self):
        """once topo_map is received, get the indices of nodes for easy access to node object"""
        for i in range(len(self.tmap2['nodes'])):
            self.node_index[self.tmap2['nodes'][i]['node']['name']] = i  # TODO: to be tested

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
                    self.yield_at_node[node_id] = node_yield_in_row[row_id] + random.gauss(0, 0.02 * node_yield_in_row[
                        row_id])
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
        self.local_storage_nodes = {row_id: None for row_id in self.row_ids}
        self.local_storages = {}

        n_local_storages = len(local_storages)
        # set local storage nodes
        storage_row_groups = numpy.array_split(numpy.arange(self.n_topo_nav_rows), n_local_storages)

        for i in range(n_local_storages):
            start_row = storage_row_groups[i][0]
            end_row = storage_row_groups[i][-1]
            storage_row = "pri_hn-%02d" % (start_row + int((end_row - start_row) / 2))

            for row in storage_row_groups[i]:
                self.local_storage_nodes["row-%02d" % (row)] = storage_row
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
        self.head_nodes = {"row-%02d" % (i): [] for i in range(self.n_topo_nav_rows)}
        for i in range(self.n_topo_nav_rows):
            self.head_nodes["row-%02d" % (i)].append("pri_hn-%02d" % (i))
            if self.second_head_lane:
                self.head_nodes["row-%02d" % (i)].append("sec_hn-%02d" % (i))

        self.row_nodes = {"row-%02d" % (i): [] for i in range(self.n_topo_nav_rows)}

        for node in self.tmap2.nodes:
            for i in range(self.n_topo_nav_rows):
                if "rn-%02d" % (i) in node.name:
                    self.row_nodes["row-%02d" % (i)].append(node.name)
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
            return [start_node], route_edges, route_distances

        route = self._route_planner.search_route(start_node, goal_node)
        if route is not None:
            route_nodes = route.source
            route_edges = route.edge_id

            # append goal_node to route_nodes
            route_nodes.append(goal_node)

            for i in range(len(route_nodes) - 1):
                route_distances.append(self.get_distance_between_adjacent_nodes(route_nodes[i], route_nodes[i + 1]))

        return route_nodes, route_edges, route_distances

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        # return self.tmap2.nodes[self.node_index[node]]  # old tmap
        return self.tmap2['nodes'][self.node_index[node]]  # new tmap2

    def get_node_edges(self, node_id):
        """
        Given node_id, return all connected edges
        """
        node = self.get_node(node_id)
        edges = get_conected_nodes_tmap2(node)
        return edges

    def get_distance_between_adjacent_nodes(self, from_node, to_node):
        """get_distance_between_adjacent_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return get_distance_to_node_tmap2(from_node_obj, to_node_obj)

    def get_route_between_adjacent_nodes(self, from_node, to_node, avoid_nodes=None):
        """
        get the route between two adjacent nodes, the route excludes the direct one, i.e., from_node -> to_node
        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name
        """

        if avoid_nodes is not None:
            dyn_map = self.get_topo_map_no_occupied_rownodes(avoid_nodes)
        else:
            dyn_map = copy.deepcopy(self.tmap2)

        from_node_idx = self.node_index[from_node]
        to_node_idx = self.node_index[to_node]

        for edge_idx, edge in enumerate(dyn_map['nodes'][from_node_idx]['node']['edges']):
            if edge['node'] == to_node:
                dyn_map['nodes'][from_node_idx]['node']['edges'].remove(
                    dyn_map['nodes'][from_node_idx]['node']['edges'][edge_idx])
                break
        for edge_idx, edge in enumerate(dyn_map['nodes'][to_node_idx]['node']['edges']):
            if edge['node'] == from_node:
                dyn_map['nodes'][to_node_idx]['node']['edges'].remove(
                    dyn_map['nodes'][to_node_idx]['node']['edges'][edge_idx])
                break

        route_planner = TopologicalRouteSearch2(dyn_map)
        route = route_planner.search_route(from_node, to_node)
        if route is not None:
            route_nodes = route.source
            if from_node in route_nodes:
                route_nodes.remove(from_node)
            # append goal_node to route_nodes
            route_nodes.append(to_node)
            return route_nodes
        else:
            return None

    def get_edges_between_nodes(self, from_node, to_node):
        """get_edges_between_nodes: Given names of two nodes, return the direct edges
        between their node objects. A wrapper for the get_edges_between function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        edge_ids = []
        edges = get_edges_between_tmap2(self.tmap2, from_node, to_node)
        for edge in edges:
            edge_ids.append(edge.edge_id)
        return edge_ids

    # deprecated, only works with tmap1, use get_topo_map_no_occupied_rownodes instead
    def get_topo_map_no_agent_rownodes(self):
        """get the current topological map without the edges connecting to row nodes with agents.
        this map should be used for route_search for all route searches here."""
        dyn_map = copy.deepcopy(self.tmap2)  # copy of map to be modified
        agent_nodes = copy.deepcopy(self.agent_nodes)  # copy to avoid data inconsistency

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

    def get_topo_map_no_occupied_rownodes(self, avoid_nodes):
        """get the current topological map without the edges connecting to row nodes occupied by robots.
        this map should be used for route_search for all route searches here."""
        dyn_map = copy.deepcopy(self.tmap2)  # copy of map to be modified

        for node in avoid_nodes:
            # find neighbouring nodes and edges from the agent node
            nbr_nodes = []
            for edge in dyn_map['nodes'][self.node_index[node]]['node']['edges']:
                nbr_nodes.append(edge['node'])
            # remove all edges
            dyn_map['nodes'][self.node_index[node]]['node']['edges'] = []

            # for each neighbour, find and remove the edge back to the agent node
            for nbr_node in nbr_nodes:
                edge_indices = []
                for edge in dyn_map['nodes'][self.node_index[nbr_node]]['node']['edges']:
                    if edge['node'] == node:
                        edge_indices.append(dyn_map['nodes'][self.node_index[nbr_node]]['node']['edges'].index(edge))
                edge_indices.sort(reverse=True)
                for edge_index in edge_indices:
                    dyn_map['nodes'][self.node_index[nbr_node]]['node']['edges'].pop(edge_index)

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
        # router = topological_navigation.route_search.TopologicalRouteSearch(dyn_map)
        router = TopologicalRouteSearch2(dyn_map)

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

        return route_nodes, route_edges, route_distances

    def get_row_id_of_row_node(self, node):
        """given a row_node, return the row_id"""
        for row_id in self.row_ids:
            if node in self.row_nodes[row_id]:
                return row_id
        return None

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            # rospy.loginfo(msg)
            print(msg)