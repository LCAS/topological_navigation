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
import collections


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
        self.dead_locks = []  # robots who are in deadlock
        self.com_nodes = []  # the nodes that hold completed robots
        self.cold_storage_queue = []  # robots that in the front of queue

        self.base_stations = base_stations
        self.wait_nodes = wait_nodes
        self.cold_storage_usage_queue = []   # robots that need to use cold storage for unloading will join the queue
        self.cold_storage_queue_wait_time = []

        self.targets = {}
        self.agent_nodes = {}  # agent_id:agent.curr_node - should be updated by the agent
        self.curr_node = {}

        self.time_spent_requesting = .0  # time that robot spent on waiting when requesting a node

        # agent specific parameters, initialised when initialising agent instance
        self.transportation_rate = .0  # float, transportation speed
        self.transportation_rate_std = .0  # float, transportation speed standard deviation
        self.unloading_time = .0
        self.assigned_picker_n_trays = .0  # float,

        self.dodge = {}  # agent dodge flags
        self.other_robot_dodged = False
        self.interrupted = {}

        self.request_process = None
        self.graph_goto_process = None
        self._goto_process = None

        self.node_put_queue = {}

        self.process_timeout = 0.10
        self.loop_timeout = 0.10

        self.t_mode = {}

        self._cost = {'robot_id': [],
                      'time_now': [],
                      'time_wait': [],
                      'dist_cost': [],
                      'route': [],
                      'dist_cost_total': [],
                      'time_wait_total': [],
                      'time_cost_total': [],
                      'cost_total': []}

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

        # self.agent_nodes = {}  # agent_id:agent.curr_node - should be updated by the agent

        self.dist_cost_to_target = {}

        """
        The bellow methods are used for model topological nodes as containers, each container can only hold one robot.
        Pickers don't use this container feature but use the same topological map. 
        """
    def init_agent_nodes(self, robot_id, transportation_rate,
                         transportation_rate_std, unloading_time, assigned_picker_n_trays):
        """
        Initialise agent current nodes when initialising agent instance
        """
        self.transportation_rate = transportation_rate
        self.transportation_rate_std = transportation_rate_std
        self.unloading_time = unloading_time
        self.assigned_picker_n_trays = assigned_picker_n_trays

        self.curr_node[robot_id] = self.base_stations[robot_id]  # robot starts from base station
        self.agent_nodes[robot_id] = self.curr_node[robot_id]

        # used for deadlocks
        self.dodge[robot_id] = {}
        self.dodge[robot_id]['to_dodge'] = False
        self.dodge[robot_id]['dodged'] = False

    def log_cost(self, robot_id, time_wait, dist_cost, node_name):
        """
        Update the time and distance cost
        :param robot_id: string, robot name
        :param time_wait: integer, waiting time before occupy the requested node
        :param dist_cost: float, the distance from current node to next node
        :param node_name: string, the node will be traversed
        return: dictionary, cost info
        """
        dist_cost_total = 0
        time_wait_total = 0
        dist_cost = float('{:.2f}'.format(dist_cost))
        self._cost['robot_id'].append(robot_id)
        self._cost['time_now'].append(round(self.env.now, 1))
        self._cost['time_wait'].append(round(time_wait, 1))
        self._cost['dist_cost'].append(dist_cost)
        self._cost['route'].append(node_name)
        for n in self._cost['dist_cost']:
            dist_cost_total = dist_cost_total + n
        dist_cost_total = float('{:.2f}'.format(dist_cost_total))
        self._cost['dist_cost_total'].append(dist_cost_total)
        for n in self._cost['time_wait']:
            time_wait_total = round(time_wait_total + n, 1)
        time_cost_total = float('{:.2f}'.format(self.time_to_dist_cost(time_wait_total)))
        self._cost['time_wait_total'].append(time_wait_total)
        self._cost['time_cost_total'].append(time_cost_total)
        cost_total = float('{:.2f}'.format(time_cost_total + dist_cost_total))
        self._cost['cost_total'].append(cost_total)
        return self._cost

    def time_to_dist_cost(self, waiting_time):
        """
        :param waiting_time: the time robot will be waiting
        return: float, the distance cost
        """
        return (self.transportation_rate + random.gauss(0, self.transportation_rate_std)) * waiting_time

    def get_route_nodes(self, cur_node, target, avoid_nodes=None):
        """
        get the topological route nodes from current node to target node, if request_node failed,
        then get the route nodes from current node to target node and avoid the failed node.
        :param cur_node: string, the current node traversed
        :param target: string, the target node that the robot will go to
        :param avoid_nodes: string list, the nodes that won't be considered when planning route
        return: string list, route node names, from current node to target node
        """
        avo_nodes = copy.deepcopy(avoid_nodes)
        if self.env:
            if target == cur_node:
                print('  %5.1f: robot is already at target %s' % (self.env.now, target))
                return []
            else:
                if avo_nodes is None:
                    route = self.get_route(cur_node, target)
                elif target in avo_nodes:
                    # target is occupied at present, but maybe available later, so get the route anyway
                    avo_nodes.remove(target)
                    route = self.get_avoiding_route(cur_node, target, avo_nodes)
                    # It will lead to deadlocking if the target is still occupied when the agent is next to
                    # the target
                    if len(route.source) == 1:  # in deadlock
                        return None
                else:  # target not in avo_nodes
                    route = self.get_avoiding_route(cur_node, target, avo_nodes)

                if len(route.source) == 0:
                    return None
                else:
                    r = route.source[1:]
                    r.append(target)
                    # print('  %5.1f: %s going from node %s to node %s via route %s' % (
                    #     self.graph.env.now, robot_id, cur_node, target, r))
                    return r

    def deadlock_dodge(self, robot_id, n, target, new_route):
        """
        When in deadlock, find a new route.
        If no new route available, then go to an edge node, then find a new route to the target

        TODO: dodging rules:
                1. if robot['target'] is storage, robot['priority'] = 100, all other robots dodge.
                2. if robot['target'] is base, i.e., in the queue of using storage, this robot should always first to
                    dodge to a rarely used edge node.
                        2.1 Do not dodge to base station or other highly used nodes! --> which would cause
                            further congestion
                        2.2 Do not dodge to other robots' next node (and current route?)
                3.

        :param n: string, topological node, the next node that the robot should go
        :param new_route: string, topological node list, a new topological route from current node to the target
        :param target: string, topological node, the target node that the robot should go
        :param robot_id: string, robot name

        return: topological node list, a new route to avoid deadlock
        """
        if new_route is not None:  # TODO: is it possible?
            route_nodes = new_route
        else:
            avoid_nodes = self.get_active_nodes([robot_id])
            curr_node_edges = self.get_node_edges(self.curr_node[robot_id])
            # get all the edge nodes that could be used for dodging
            for node in avoid_nodes:
                if node in curr_node_edges:
                    curr_node_edges.remove(node)
            # find the node to dodge:  node_to_dodge
            if n in avoid_nodes:
                avoid_nodes.remove(n)
                route_nodes_to_dodge = self.get_route_between_adjacent_nodes(
                    self.curr_node[robot_id], n, avoid_nodes)
                if route_nodes_to_dodge[0] != n and len(route_nodes_to_dodge) != 0:
                    node_to_dodge = route_nodes_to_dodge[0]
                    # Without considering avoid_nodes, route_nodes_to_dodge is ensured
                    # with a valid route_nodes
                    # TODO: is it better to get route_nodes with avoiding avoid_nodes?
                    route_nodes_to_dodge = self.get_route_nodes(node_to_dodge, target)
                    route_nodes = [node_to_dodge] + route_nodes_to_dodge
                elif len(curr_node_edges) != 0:
                    node_to_dodge = curr_node_edges[0]  # TODO: any better selecting criteria?
                    route_nodes_to_dodge = self.get_route_nodes(node_to_dodge, target)
                    route_nodes = [node_to_dodge] + route_nodes_to_dodge
                else:
                    # no edges for dodging, wait for other agents to move
                    # TODO: Inform another deadlocked robot to dodge.
                    #       note: another robot has locked two nodes
                    #       Double check if another deadlocked robot releases the node and
                    #       the node then be occupied by a third robot: what the
                    #       current robot should do?
                    msg = "%s: @%s No edge to dodge, curr_node_edges: %s" % (
                        robot_id, self.curr_node[robot_id], curr_node_edges)
                    raise Exception(msg)
                    # route_nodes = None
                    # yield self.env.timeout(self.loop_timeout)
            else:
                # todo: impossible, remove the if-else later
                msg = "%s: %s not in avoid_nodes!" % (robot_id, n)
                raise Exception(msg)

        return route_nodes

    def deadlock_rank(self, dead_locks=None):
        """
        Get all robots' information and considering them as a unity, find out the breakpoint:
            which robot should make way

        return: ranked robot list from high to low, based on robot's priority

        Robot's information:
            current node --> next node --> target node
            requested node, requested moment, wait time(this should be the key point that needs to be re-estimated,
            usually situations change after )

        Priority rank:
            Rank robots' priority, the higher priority (bigger value) robot should move first, the lower priority
            robot should move later
            100: lowest priority value, should move at last. This is the initial priority value, other robot's
                priorities increase based on this.
            110:
            120:
            130:
            The robot who occupies other robot's target node has higher priority to move

        TODO: dodging rules:
            1. if robot['target'] is storage, robot['priority'] = 0, all other robots dodge.
            2. if robot['target'] is base, i.e., in the queue of using storage, this robot should always first to
                dodge to a rarely used edge node.
                    2.1 Do not dodge to base station or other highly used nodes! --> which would cause
                        further congestion
                    2.2 Do not dodge to other robots' next node (and current route?)
            3.
        """

        # get all active nodes into a list:
        active_node_list = []
        for robot_id in self.active_nodes.keys():
            active_node_list = active_node_list + self.active_nodes[robot_id]

        all_robots = list(self.targets.keys())
        to_be_ranked_robots = copy.deepcopy(all_robots)

        for robot_id in all_robots:
            # there should be only one robot targets at storage at a moment, or back from storage
            if self.targets[robot_id]['target'] == self.cold_storage_node or robot_id == self.cold_storage_usage_queue[0]['robot_id']:
                self.targets[robot_id]['priority'] = 0  # highest priority, all other robots doge
                to_be_ranked_robots.remove(robot_id)

                # the robot who blocks highest priority robot should dodge first
                for robot_dodge in to_be_ranked_robots:
                    if self.targets[robot_dodge]['curr_node'] == self.targets[robot_id]['next_node']:
                        self.targets[robot_dodge]['priority'] = 1000
                        to_be_ranked_robots.remove(robot_dodge)
                        # todo: this robot should dodge to a rarely used edge node
                        break
            # robot initial priority = 100, if target is base, then increase by 1000. Note: the robot's priority could
            #   be 1000 by now if known as robot_dodge in last step
            elif self.targets[robot_id]['target'] == self.base_stations[robot_id]:
                self.targets[robot_id]['priority'] += 1000  # TODO[today]: if robot['target'] is back from storage, priority should still be 0!

            # TODO[future]: optimise priority value. At present, take the target as pickers current node, so the robot
            #   has priority to get to the picker.
            elif self.targets[robot_id]['target'] not in active_node_list:
                self.targets[robot_id]['priority'] -= 10

                # if len(self.active_nodes[robot_id]) == 2:
                #     # remove robot_id from the put_queue of node self.graph.active_nodes[robot_id][1].
                #     # After removing, what about other robots in the waiting list? their actual waiting time will
                #     # shortened, no negative impact.
                #     # TODO[today]: how to wake up robot from the put_queue? put_queue.pop(indx) could clear the queue,
                #     #               but robot_01 is not waking up! process.interrupt()
                #     pass

        # rank the robots based on priority
        ranked_robots_dict = {}
        for robot_id in all_robots:
            ranked_robots_dict[robot_id] = self.targets[robot_id]['priority']
        ranked_robots = collections.OrderedDict(sorted(ranked_robots_dict.items(), key=lambda x: x[1], reverse=True))

        return ranked_robots

    def deadlock_coordinator(self, dead_locks):
        """
        Coordinate the (deadlocking) robots to get free routes

        TODO:   1. Just let the highest priority deadlocking robot to dodge
                2. Create a for loop, if the previous robot can not solve the deadlocking( deadlock again after dodging),
                    the second highest priority deadlocking robot to dodge, if still not solved, then the third, etc.

        """
        try:
            ranked_robots_list = self.deadlock_rank()
        except Exception:
            raise Exception(" deadlock_rank() is not working")

        for robot_id in ranked_robots_list:
            if robot_id in self.dead_locks:
                self.dodge[robot_id]['to_dodge'] = True
                self.dodge[robot_id]['dodged'] = False
                break

    def wait_to_proceed(self, robot_id):
        """
        Based on the priority of dodging, deadlocking robots dodge one by one until all robots are free to go
        """

        # wait for other robot to dodge, todo: complete the deadlock_coordinator function first
        while True:
            yield self.env.timeout(self.loop_timeout)
            for robot_id_ in self.deadlocks['deadlock_robot_ids']:
                if self.dodge[robot_id_]['dodged']:
                    self.other_robot_dodged = True
                    break
            if self.other_robot_dodged:
                break

    def wait_for_dodging(self, robot_id):
        """
        Wait for other robot who has highest priority to dodge. Or the robot himself dodge if highest priority
        """
        while True:
            if self.dodge[robot_id]['to_dodge'] is True:
                break
            elif self.other_robot_dodged:
                break
            else:
                yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.loop_timeout)

    def inform_dodged(self, robot_id):
        """
        Inform robot dodge status after dodging
        """
        self.dodge[robot_id]['dodged'] = True
        self.dodge[robot_id]['to_dodge'] = False

    def inform_interrupted(self, robot_id):
        """
        Inform others after finishing interruption
        """
        self.interrupted[robot_id]['interrupted'] = True
        self.interrupted[robot_id]['to_interrupted'] = False

    def inform_to_interrupt(self, robot_id):
        """
        Inform robot to interrupt
        """
        self.interrupted[robot_id]['interrupted'] = False
        self.interrupted[robot_id]['to_interrupted'] = True

    def reset_interrupted(self, robot_id):
        """
        Reset interrupt flag
        """
        self.interrupted[robot_id]['interrupted'] = False
        self.interrupted[robot_id]['to_interrupted'] = False

    def reset_dodge(self, robot_id):
        """
        Reset dodge flag if dodged before
        """
        if self.dodge[robot_id]['dodged'] is True:
            self.dodge[robot_id]['dodged'] = False
        if self.dodge[robot_id]['to_dodge'] is True:
            self.dodge[robot_id]['to_dodge'] = False

    def mark_node_put_queue_idx(self, robot_id, req_node):
        """
        Robot join the queue and waiting for occupying the requested node.
        Record the position of the robot in the waiting queue, i.e., the index of put_queue, for later getting robot
        out of the put_queue if needed

        Usually use this method after requesting a node
        :param robot_id: string, robot name
        :param req_node: string, topological node name, the node that requested to occupy
        """
        # node resource is full, the next robot will join the put_queue. We have to cache the robot's position before
        # calling self._node_res[node].put(1) as once the robot joined the put_queue, the process will stay there and
        # will not continue. So we can't get the robot's position in the put_queue immediately.
        if self._node_res[req_node].capacity == self._node_res[req_node].level:
            curr_queue_len = len(self._node_res[req_node].put_queue)
            self.node_put_queue[robot_id] = {'req_node': req_node,
                                             'idx': curr_queue_len}

    # deprecated
    def add_node_put_queue(self, robot_id, req_node):
        """
        Record the put_queue index for later getting robot out of the put_queue if needed

        Only use this method when req_node has been occupied by other agents
        :param robot_id: string, robot name
        :param req_node: string, topological node name, the node that requested to occupy
        """
        put_queue_idx = self.get_node_state(req_node) - 3

        # try:
        #     assert put_queue_idx > -1
        # except AssertionError:
        #     raise Exception("%s requested %s, but the node is free, robot won't join the put_queue" % (
        #         robot_id, req_node))

        self.node_put_queue[robot_id] = {'req_node': req_node,
                                         'idx': put_queue_idx}

    def release_node_from_put_queue(self, robot_id, req_node):
        """
        Remove robot from the put_queue of node

        Only use this method when robot is in the put_queue of the node
        :param robot_id: string, robot name
        :param req_node: string, topological node name, the node that requested to occupy
        """
        self.pop_free_node(robot_id, req_node, deadlock=True)
        if len(self._node_res[req_node].put_queue) > 0 and req_node == self.node_put_queue[robot_id]['req_node']:
            yield self._node_res[req_node].put_queue.pop(self.node_put_queue[robot_id]['idx'])
            yield self.env.timeout(0.1)
        else:
            yield self.env.timeout(0)

    def release_node_from_put_queue_return(self, robot_id, req_node):
        """
        Remove robot from the put_queue of node

        Only use this method when robot is in the put_queue of the node
        :param robot_id: string, robot name
        :param req_node: string, topological node name, the node that requested to occupy
        """
        self.pop_free_node(robot_id, req_node, deadlock=True)
        if len(self._node_res[req_node].put_queue) > 0 and req_node == self.node_put_queue[robot_id]['req_node']:
            return self._node_res[req_node].put_queue.pop(self.node_put_queue[robot_id]['idx'])
        else:
            return self.env.timeout(0)

    def release_node_from_put_queue_return_remove(self, robot_id, req_node):
        """
        Remove robot from the put_queue of node

        Only use this method when robot is in the put_queue of the node
        :param robot_id: string, robot name
        :param req_node: string, topological node name, the node that requested to occupy
        """
        self.pop_free_node(robot_id, req_node, deadlock=True)
        if len(self._node_res[req_node].put_queue) > 0 and req_node == self.node_put_queue[robot_id]['req_node']:
            for i in range(len(self._node_res[req_node].put_queue)):
                if i == self.node_put_queue[robot_id]['idx']:
                    obj = self._node_res[req_node].put_queue[i]
                    self._node_res[req_node].put_queue.remove(obj)
                    return self.env.timeout(0)
                    # TODO[today, now]:
                    #  target = <ContainerPut() object at 0x7f267fe1e190>
                    #  callbacks = [<bound method Container._trigger_get of <simpy.resources.container.Container object at 0x7f267fe74990>>]
                    #  Even though robot removed from the put_queue, the process will died here. As there is no target to proceed? so can't back
                    #  to callbacks?
                    # return self._node_res[req_node].put_queue.pop(self.node_put_queue[robot_id]['idx'])
        else:
            return self.env.timeout(0)

    def go_to_storage_operation(self, robot_id, target):
        """
        Robot joining the queue for using storage station(node). If the queue is empty then go to storage directly.
        If not, join in the queue, go to base station and predict the waiting time for using the storage. Keep waiting
        until the storage is available.
        Split the process (self.mode == 3) that from picker loading on robot to robot arrives at (cold) storage
            node into 4 sub modes(t_mode: transportation mode):
            t_mode 30: robot going to (cold) storage directly from picker;
                        (mode 3 ->) t_mode 30 -> t_mode 4 (-> mode 4 -> mode 6)
            t_mode 31: robot going to base station from picker;
                        (mode 3 ->) t_mode 31 -> t_mode 32
            t_mode 32: robot parking at base station, waiting in the self.cold_storage_usage_queue;
                        t_mode 31 -> t_mode 32 -> t_mode 33
            t_mode 33: robot going to (cold) storage from base station;
                        t_mode 32 -> t_mode 33 -> t_mode 4 (-> mode 4 -> mode 6)
        :param target: string, target topological node
        :param robot_id: string, robot name
        Note: the function could be extend to use for any other target node that in a single track route
        """
        # TODO [future]: to be done...
        #       1. self.time_spent_waiting_storage needs to be recorded

        # if target is the cold_storage_node, but no robot planning to use cold_storage_node now, then go to use
        if len(self.cold_storage_usage_queue) == 0:
            self.t_mode[robot_id] = 30
        else:
            self.t_mode[robot_id] = 31

        while self.t_mode[robot_id] != 4:
            if self.t_mode[robot_id] == 30:
                self.add_cold_storage_usage_queue(robot_id, self.env.now)
                self.init_wait_queue(robot_id, target)
                self._goto_process = self.env.process(self._goto_container(target, robot_id))
                yield self._goto_process
                self.t_mode[robot_id] = 4

            elif self.t_mode[robot_id] == 31:
                self.add_cold_storage_usage_queue(robot_id, self.env.now)
                if self.curr_node[robot_id] != self.base_stations[robot_id]:
                    self._goto_process = self.env.process(
                        self._goto_container(self.base_stations[robot_id],
                                             robot_id))
                    yield self._goto_process
                self.t_mode[robot_id] = 32

            elif self.t_mode[robot_id] == 32:
                try:
                    self.start_parking_at_base(robot_id, self.env.now)
                    wait_info = self.get_queue_wait_time(robot_id)
                    # if wait_info['wait_time'] < 0:  # TODO: never reaching, to be removed
                    #     print("wait_info['wait_time']: %f" % wait_info['wait_time'])
                    self.extend_hold_time(self.curr_node[robot_id],
                                          wait_info['wait_time'])  # todo: inform other robots who are waiting for this node
                    # yield self.env.timeout(wait_info['wait_time'])
                    while robot_id not in self.get_cold_storage_usage_queue_head()['robot_id']:
                        yield self.env.timeout(self.loop_timeout)
                    self.t_mode[robot_id] = 33
                except Exception:
                    print('parking bug')

            elif self.t_mode[robot_id] == 33:
                # correct the predict wait_time according to real situation
                for idx, queue_robot in enumerate(self.cold_storage_queue_wait_time):
                    if robot_id == queue_robot['robot_id']:
                        if 'start_parking_time' in queue_robot.keys():
                            real_wait_time = self.env.now - queue_robot['start_parking_time']
                        else:
                            real_wait_time = .0
                        self.cold_storage_queue_wait_time[idx]['wait_time'] = real_wait_time
                        self.cold_storage_queue_wait_time[idx]['start_using_storage_time'] = self.env.now
                        break
                self._goto_process = self.env.process(self._goto_container(target, robot_id))
                yield self._goto_process
                self. t_mode[robot_id] = 4

    def goto_scheduler_mode(self, target, robot_id):
        """
        Scheduler of assigning target node to the robot based on whether the target is the cold_storage_node and
        whether the queue of waiting for using cold storage node is free (len(cold_storage_usage_queue) == 0). if
        the target is not cold_storage_node or the queue is free, then go to the target directly. Or, join the queue
        and keep waiting until the previous robots have finished using the cold_storage_node before going to the
        target(cold_storage_node)

        :param target: string, topological node that to be reached
        :param robot_id: string, robot name
        """

        # before go to the target, if the target node is not cold_storage_node, go to the target directly
        if target != self.cold_storage_node:
            yield self.env.process(self._goto_container(target,
                                                        robot_id))
            # if self.interrupted:
            #     yield self.graph.env.process(self._goto(target))
        else:
            # try:
            #     assert self.mode == 3  # robot go to local_storage_node or cold_storage_node from picker_node
            # except AssertionError:
            #     raise Exception("goto_scheduler is trying to assign %s going to %s, but robot is in %d" %
            #                     (self.robot_id, target, self.mode))

            # go to storage, robot enters different sub modes based on the storage usage queue
            yield self.env.process(self.go_to_storage_operation(robot_id, target))

    def _goto_container(self, target, robot_id):
        """
        robot goes from current node to target node.
        The robot always chooses minimum-distance-cost route(original_route) and travels from current node to target.
        If robot finds the next node is occupied during travelling, the robot will try to find a new_route avoiding the
        occupied node. But at the same time, the robot also evaluates the waiting time cost if keeps
        using the original_route. The robot always chooses low cost route between new_route and original_route.

        :param target: string, target node
        :param robot_id: string, robot name
        """
        # self.curr_node[robot_id] = curr_node
        route_nodes = self.get_route_nodes(self.curr_node[robot_id], target)
        interrupted = None
        idx = 0
        change_route = False

        if len(route_nodes) == 0:
            print("len(route_nodes)=0, no route from %s to %s" % (self.curr_node[robot_id], target))
            interrupted = 'no_route'

        while idx < len(route_nodes):

            self.other_robot_dodged = False  # todo: test after deadlock change route

            n = route_nodes[idx]

            # get the current node, next node and target node, used for resolving deadlocks
            self.targets_info(robot_id, self.curr_node[robot_id], n, target)

            route_dist_cost = self.route_dist_cost([self.curr_node[robot_id]] + route_nodes[idx:])
            self.update_dist_cost_to_target(robot_id, self.curr_node[robot_id], target, route_dist_cost)

            d_cur = self.distance(self.curr_node[robot_id], n)
            time_to_travel = round(d_cur / self.transportation_rate + random.gauss(0, self.transportation_rate_std), 1)
            if idx + 1 < len(route_nodes):
                # estimate next distance cost and travelling time
                d_next = self.distance(n, route_nodes[idx + 1])
                time_to_travel_next = round(d_next / (2 * (self.transportation_rate +
                                                           random.gauss(0, self.transportation_rate_std))), 1)
            else:
                time_to_travel_next = 0

            # the time that node will hold the robot
            # if the node is the cold_storage_node, the hold time needs to consider robot unloading trays(self.unloading_time)
            # TODO: the agent's rotating time should be considered
            if n == self.cold_storage_node:
                hold_time = time_to_travel + time_to_travel_next + self.unloading_time * self.assigned_picker_n_trays
            else:
                hold_time = time_to_travel + time_to_travel_next

            # self.loginfo('  %5.1f:  %s traversing route from node %s to node %s '
            #              '(distance: %f, travel time: %5.1f, plan hold time: %5.1f)' %
            #              (self.graph.env.now, robot_id, self.curr_node[robot_id], n, d_cur, time_to_travel, hold_time))

            try:
                # The node to be requested may be occupied by other robots, mark the time when requesting
                start_wait = self.env.now
                self.set_hold_time(n, hold_time)
                node_state = self.get_node_state(n)
                if node_state is 1:
                    yield self.request_node(robot_id, n)
                    # self.loginfo('  %5.1f: active nodes connected to robots: %s ' %
                    #              (self.env.now, self.graph.active_nodes))
                else:
                    # self.loginfo('  %5.1f: %s: %s is occupied, node state: %d' %
                    #              (self.env.now, robot_id, n, node_state))
                    avoid_nodes = [n] + self.get_active_nodes([robot_id])
                    avoid_nodes = list(dict.fromkeys(avoid_nodes))
                    new_route = self.get_route_nodes(self.curr_node[robot_id], target, avoid_nodes)  # avoid node n
                    if new_route is None:
                        new_route_dc = float("inf")
                    elif new_route is []:  # robot is at target now
                        new_route_dc = 0
                    else:
                        new_route_dc = self.route_dist_cost([self.curr_node[robot_id]] + new_route)
                    wait_time = round(self.get_wait_time(n), 1)
                    time_cost = self.time_to_dist_cost(wait_time)
                    old_route_cost = route_dist_cost + time_cost
                    # self.loginfo('$ %5.1f: old_route_cost = %d, new_route_dc = %d' % (
                    #     self.env.now, old_route_cost, new_route_dc))
                    if old_route_cost > new_route_dc:
                        route_nodes = new_route
                        self.cancel_hold_time(n, hold_time)  # found cheap route, cancel the hold_time just be set
                        idx = 0
                        self.loginfo('~ %5.1f: %s go NEW route: %s' % (self.env.now, robot_id, route_nodes))
                        continue
                    else:
                        route_nodes_to_go = [self.curr_node[robot_id]] + route_nodes[idx:]
                        self.loginfo('* %5.1f: %s wait %5.1f, use old route %s' % (
                            self.env.now, robot_id, wait_time, route_nodes_to_go))
                        yield self.request_node(robot_id, n)

                        if robot_id in self.deadlocks['deadlock_robot_ids']:

                            # self.deadlock_coordinator(self.deadlocks['deadlock_robot_ids'])  # todo[next]: use coordinator after splitting out container map

                            self.dodge[robot_id]['to_dodge'] = True

                            if self.dodge[robot_id]['to_dodge'] is True:
                                self.loginfo('| %5.1f: %s is in deadlock, change route now' %
                                             (self.env.now, robot_id))
                                route_nodes = self.deadlock_dodge(robot_id, n, target, new_route)

                                # prepare to change to the new route_nodes
                                self.cancel_hold_time(n, hold_time)  # cancel the hold_time just set
                                idx = 0
                                change_route = True
                                self.loginfo('^ %5.1f: %s go NEW route: %s' %
                                             (self.env.now, robot_id, route_nodes))
                                # reset dodge flags todo: duplicate with the inform_dodged, keep later
                                self.dodge[robot_id]['to_dodge'] = False
                                self.dodge[robot_id]['dodged'] = True
                            else:
                                # back to farm, inform other robots to dodge.
                                yield self.env.timeout(self.loop_timeout)

                                # then other robots will be interrupted in yield self.request_node(robot_id, n), and go
                                # to exception. Then [todo[now]: dodge here! in the exception before] process _goto_container again.
                                # todo: checking the wait_to_proceed, how to do if it's not your turn to dodge
                                try:
                                    yield self.env.process(self.wait_to_proceed(robot_id))
                                except Exception:
                                    self.loginfo('$ %5.1f:  %s wait ERROR' % (self.env.now, robot_id))

                        if change_route:
                            self.log_cost(robot_id, 0, 0, 'CHANGE ROUTE')  # for monitor
                            self.remove_robot_from_deadlock(robot_id)

                            self.inform_dodged(robot_id)  # TODO[today], then reset all dodge? Or reset when robot reaching next node?

                            self.other_robot_dodged = True   #  TODO [to be removed] other robots could read from self.dodge that some robot has dodged

                            # TODO: How to ensure that the deadlock will be resolved? --> the dodge route is promised to
                            #       be working(unless curr_node_edges is None, i.e., deadlock in single track),
                            #       so the deadlock must be resolved at present.
                            continue  # change to new route
                        else:
                            pass  # travel to the requested node
                if self.env.now - start_wait > 0:  # The time that the robot has waited since requesting
                    self.time_spent_requesting += self.env.now - start_wait
                    self.loginfo('$ %5.1f:  %s has lock on %s after %5.1f' % (
                        self.env.now, robot_id, n,
                        self.env.now - start_wait))

            except simpy.Interrupt:
                if robot_id in self.dead_locks:
                    # self.reset_dodge(robot_id)  # reset 'to_dodge'
                    if self.dodge[robot_id]['to_dodge'] is True:
                        self.dodge[robot_id]['to_dodge'] = False
                    if self.dodge[robot_id]['dodged'] is False:
                        self.dodge[robot_id]['dodged'] = True

                    interrupted = 'deadlock'    # interrupted by farm.scheduler_monitor: robot._goto_process.interrupt()
                    self.loginfo('  %5.1f: @@@ %s INTERRUPTED when in deadlock, quiting from put_queue of %s' %
                                 (self.env.now, robot_id, n))
                    # yield self.env.process(self.release_node_from_put_queue(robot_id, n))
                    # yield self.release_node_from_put_queue_return(robot_id, n)  # TODO[now]: where is the robot after remove from the put_queue???
                    yield self.release_node_from_put_queue_return_remove(robot_id, n)
                break

            except TypeError:
                self.loginfo('  %5.1f: %s INTERRUPTED:TypeError, from node %s going to node %s'
                             % (self.env.now, robot_id, self.curr_node[robot_id], n))

            except ValueError:
                self.loginfo('  %5.1f: %s INTERRUPTED:ValueError, from node %s going to node %s'
                             % (self.env.now, robot_id, self.curr_node[robot_id], n))

            except Exception:
                # Not triggered when requesting an occupied node!
                # Not triggered when the robot has a goal running and be assigned a new goal
                self.loginfo('  %5.1f: %s INTERRUPTED while waiting to gain access to go from node %s going to node %s'
                             % (self.env.now, robot_id, self.curr_node[robot_id], n))
                self.log_cost(robot_id, 0, 0, 'INTERRUPTED')  # for monitor
                self.loginfo('  %5.1f: @@@ %s release previously acquired target node %s' %
                             (self.env.now, robot_id, n))

                interrupted = 'waiting'
                yield self.release_node(robot_id, n)
                # TODO [next]: reset robot status. send robot back to a safe place? Use a new mode to deal with the
                #  exception.
                break

            try:
                time_to_travel_before_release = round(d_cur / (2 * (self.transportation_rate +
                                                                    random.gauss(0, self.transportation_rate_std))), 1)
                yield self.env.timeout(time_to_travel_before_release)

                # robot reaching at half way, if dodged before, reset dodge status todo [testing]: reset earlier?
                # self.reset_dodge(robot_id)

                yield self.release_node(robot_id, self.curr_node[robot_id])
                # The robot is reaching at the half way between the current node and next node, release current node
                # self.loginfo('  %5.1f:  %s ---> %s reaching half way ---> %s, releasing %s' % (
                #     self.graph.env.now, self.curr_node[robot_id], robot_id, n, self.curr_node[robot_id]))
                self.curr_node[robot_id] = n
                self.agent_nodes[robot_id] = self.curr_node[robot_id]

                remain_time_to_travel = time_to_travel - time_to_travel_before_release
                yield self.env.timeout(remain_time_to_travel)
                self.loginfo('@ %5.1f:  %s reached node %s' % (self.env.now, robot_id, n))
                self.log_cost(robot_id, self.env.now - start_wait, d_cur, n)  # for monitor
                yield self.env.timeout(0)
            except simpy.Interrupt:  # When the robot has a running goal but being assigned a new goal
                self.loginfo('  %5.1f: %s INTERRUPTED while travelling from node %s going to node %s' % (
                    self.env.now, robot_id,
                    self.curr_node[robot_id], n
                ))
                self.log_cost(robot_id, 0, 0, 'INTERRUPTED')  # for monitor
                self.loginfo('  %5.1f: @@@ %s release previously acquired target node %s' %
                             (self.env.now, robot_id, n))
                yield self.release_node(robot_id, n)
                interrupted = 'travelling'
                break
            idx = idx + 1  # go to next while loop

        if interrupted is not None:
            # When the robot has a goal running and be assigned a new goal node
            self.loginfo('  %5.1f: %s INTERRUPTED at %s BY %s' % (self.env.now, robot_id, self.curr_node[robot_id], interrupted))
            self.loginfo('  %5.1f: %s ABORTED at %s' % (self.env.now, robot_id, self.curr_node[robot_id]))
            self.log_cost(robot_id, 0, 0, 'ABORTED')  # for monitor
            # self.interrupted = True

            if interrupted == 'deadlock':
                self.loginfo('C %5.1f: %s continue going to target: %s' % (self.env.now, robot_id, target))
                # TODO: [future] interrupted reason is still unclear (one reason is the wait_time < 0, but why it causes
                #  interrupt?)
                self.graph_goto_process = self.env.process(self._goto_container(target, robot_id))
                yield self.graph_goto_process
                # self.interrupted = False
            elif interrupted == 'travelling':
                pass    # todo
            elif interrupted == 'no_route':
                pass    # todo
            elif interrupted == 'waiting':
                pass    # todo

        else:
            self.update_dist_cost_to_target(robot_id, self.curr_node[robot_id], target, 0)
            # TODO [NEXT]: get the node usage frequency, when robot dodging in deadlocks, dodge to node from low
            #               frequency to high frequency
            self.loginfo('. %5.1f: %s COMPLETED at %s' % (self.env.now, robot_id, self.curr_node[robot_id]))
            self.log_cost(robot_id, 0, 0, 'COMPLETED')  # for monitor
            self.add_com_node(self.curr_node[robot_id])

    def targets_info(self, robot_id, curr_node, next_node, target):
        """
        Store robot's target node, used for solving deadlocks.
        :param robot_id: string, robot name
        :param curr_node: string, topological current node
        :param next_node: string, topological next node
        :param target: string, topological target node
        """
        self.targets[robot_id] = {'curr_node': curr_node,
                                  'next_node': next_node,
                                  'target': target,
                                  'priority': 100}

    def update_dist_cost_to_target(self, robot_id, curr_node, target, route_dist_cost):
        """
        Update the distance cost from current node to the target node
        :param robot_id: string, robot name
        :param curr_node: string, current node
        :param target: string, target node
        :param route_dist_cost: float, total distance cost from current node to target
        """
        self.dist_cost_to_target[robot_id] = {'curr_node': curr_node,
                                              'target_node': target,
                                              'dist_cost': route_dist_cost}
        return self.dist_cost_to_target
    
    def add_cold_storage_usage_queue(self, robot_id, join_queue_time):
        """
        robots that need to use cold storage for unloading will join the queue
        """
        # self.cold_storage_usage_queue.append([robot_id, join_queue_time])
        self.cold_storage_usage_queue.append({'robot_id': robot_id,
                                              'join_queue_time': join_queue_time})

    def start_parking_at_base(self, robot_id, start_parking_time):
        """
        Robot starts parking at the base station, record the start parking time.
        Note: the robot must have been in the cold_storage_usage_queue
        return: bool, True: robot start parking at base
                      False: robot is not in cold_storage_usage_queue, i.e., not waiting for using the cold storage node
        """
        for idx, robot_info in enumerate(self.cold_storage_usage_queue):
            if robot_id == robot_info['robot_id']:
                # if robot has been added in the queue before, remove it
                if len(robot_info) == 3:
                    self.cold_storage_usage_queue.remove(robot_info)

                self.cold_storage_usage_queue[idx]['start_parking_time'] = start_parking_time
                print("  %.1f: %s start parking" % (start_parking_time, robot_id))
                # return True
        # return False

    def remove_cold_storage_usage_queue(self, robot_id):
        """
        robots that have used cold storage for unloading will be removed from the queue
        """
        remove_1 = False
        remove_2 = False
        for robot_id_and_time in self.cold_storage_usage_queue:
            if robot_id == robot_id_and_time['robot_id']:
                self.cold_storage_usage_queue.remove(robot_id_and_time)
                remove_1 = True

        for robot_id_and_wait_time in self.cold_storage_queue_wait_time:
            if robot_id == robot_id_and_wait_time['robot_id']:
                self.cold_storage_queue_wait_time.remove(robot_id_and_wait_time)
                remove_2 = True
        if (not remove_1) or (not remove_2):
            msg = "%s not in cold_storage_usage_queue: %s\n or cold_storage_queue_wait_time: %s" % (
                robot_id, self.cold_storage_usage_queue, self.cold_storage_queue_wait_time)
            raise Exception(msg)

    def get_cold_storage_usage_queue_head(self):
        """
        get the first element of the queue: [robot_id, join_queue_time]
        """
        if len(self.cold_storage_usage_queue) > 0:
            return self.cold_storage_usage_queue[0]
        else:
            return None

    def get_queue_wait_time(self, robot_name):
        """
        Robot in the queue for using (cold) storage station. Get the waiting time before using.
        :param robot_name: string, robot name
        :return: dictionary, {'robot_id': str,
                              'start_parking_time: None/float,
                              'wait_time': float,
                              'use_time': float}
        Note: in the comment bellow, teh robots in the cold_storage_usage_queue will be referred as:
                robot[i] -- current robot in the cold_storage_usage_queue
                robot[i-1] -- previous robot  in the cold_storage_usage_queue
        """
        wait_time = []
        use_time = []
        base_dist_to_cold = .0
        curr_dist_to_base = []
        curr_dist_to_base_time = []
        start_parking_time = .0
        stop_using_storage_time = .0
        unloading_time = self.unloading_time * self.assigned_picker_n_trays  # assume all robots use same unloading_time

        # the previous robot finished using cold storage, but the first robot of the usage queue haven't reached his base
        if len(self.cold_storage_queue_wait_time) == 0:

            # robot arriving at base and in the head of the usage queue, go to use directly
            robot_id = self.cold_storage_usage_queue[0]['robot_id']
            curr_dist_to_base.append(self.get_total_route_distances(self.agent_nodes[robot_id],
                                                                    self.base_stations[robot_id]))
            curr_dist_to_base_time.append(curr_dist_to_base[0]/self.transportation_rate)
            base_dist_to_cold = self.get_total_route_distances(self.cold_storage_node,
                                                               self.base_stations[robot_name])
            use_time.append((2*base_dist_to_cold + unloading_time)/self.transportation_rate)
            wait_time.append(curr_dist_to_base_time[0])
            self.cold_storage_queue_wait_time.append({'robot_id': robot_name,
                                                      'wait_time': wait_time[0],
                                                      'use_time': use_time[0]})

            # robot arriving base and not in the head of the usage queue, wait for previous robots
            for i in range(1, len(self.cold_storage_usage_queue)):
                robot_id = self.cold_storage_usage_queue[i]['robot_id']
                curr_dist_to_base.append(self.get_total_route_distances(self.agent_nodes[robot_id],
                                                                        self.base_stations[robot_id]))
                curr_dist_to_base_time.append(curr_dist_to_base[i]/self.transportation_rate)
                base_dist_to_cold = self.get_total_route_distances(self.cold_storage_node,  # todo:[next] initialise in _init_
                                                                   self.base_stations[robot_id])
                use_time.append((2*base_dist_to_cold + unloading_time)/self.transportation_rate)
                # if i == 1:
                #     wait_time.append(use_time[0] + max(wait_time[0], curr_dist_to_base_time[0]))
                # if i == 2:
                #     wait_time.append(use_time[1] + max(wait_time[1], curr_dist_to_base_time[1]))
                # if i > 0:
                wait_time.append(use_time[i-1] + max(wait_time[i-1], curr_dist_to_base_time[i-1]))

                # get the time of start parking at base:
                # if robot started parking, record parking time
                if len(self.cold_storage_usage_queue[i]) == 3:
                    start_parking_time = self.cold_storage_usage_queue[i]['start_parking_time']
                    self.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                              'start_parking_time': start_parking_time,
                                                              'wait_time': wait_time[i],
                                                              'use_time': use_time[i]})
                else:
                    # robot has not parked, ignore
                    self.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                              'wait_time': wait_time[i],
                                                              'use_time': use_time[i]})

        else:
            wait_time.append(self.cold_storage_queue_wait_time[0]['wait_time'])
            use_time.append(self.cold_storage_queue_wait_time[0]['use_time'])
            start_using_storage_time = self.cold_storage_queue_wait_time[0]['start_using_storage_time']
            robot_id = self.cold_storage_usage_queue[0]['robot_id']

            # note: robot transportation_rate and unloading time are simply use the same value for all robots as the wait
            #       time prediction won't be so precise and it's not worthy to be precise at such level. Because the robot
            #       may change route during traveling.

            for i in range(1, len(self.cold_storage_usage_queue)):
                # i = 1
                # robot_id = self.graph.cold_storage_usage_queue[i]['robot_id']
                # curr_dist_to_base[1] = self.graph.get_total_route_distances(self.graph.agent_nodes[robot_id],
                #                                                             self.graph.base_stations[robot_id])
                # curr_dist_to_base_time[1] = curr_dist_to_base[1]/self.transportation_rate
                # base_dist_to_cold[1] = self.graph.get_total_route_distances(self.cold_storage_node,
                #                                                             self.graph.base_stations[robot_id])
                # wait_time[1] = use_time[0] + self.graph.cold_storage_usage_queue[0]['join_queue_time'] - self.env.now
                #
                # use_time[1] = (2*base_dist_to_cold + unloading_time)/self.transportation_rate
                #
                # self.graph.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                #                                                 'start_parking_time': self.env.now+curr_dist_to_base_time[1],
                #                                                 'wait_time': wait_time[1],
                #                                                 'use_time': use_time[1]})
                #
                # i = 2
                # robot_id = self.graph.cold_storage_usage_queue[i]['robot_id']
                # curr_dist_to_base[2] = self.graph.get_total_route_distances(self.graph.agent_nodes[robot_id],
                #                                                             self.graph.base_stations[robot_id])
                # curr_dist_to_base_time[2] = curr_dist_to_base[2]/self.transportation_rate
                # base_dist_to_cold[2] = self.graph.get_total_route_distances(self.cold_storage_node,
                #                                                             self.graph.base_stations[robot_id])
                # use_time[2] = (2*base_dist_to_cold + unloading_time)/self.transportation_rate
                # wait_time[2] = max(curr_dist_to_base_time[1], wait_time[1]) + use_time[1]
                # self.graph.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                #                                                 'start_parking_time': self.env.now+curr_dist_to_base_time[2],
                #                                                 'wait_time': wait_time[2],
                #                                                 'use_time': use_time[2]})

                # i = 3
                robot_id = self.cold_storage_usage_queue[i]['robot_id']
                curr_dist_to_base.append(self.get_total_route_distances(self.agent_nodes[robot_id],
                                                                        self.base_stations[robot_id]))
                curr_dist_to_base_time.append(curr_dist_to_base[i-1]/self.transportation_rate)
                base_dist_to_cold = self.get_total_route_distances(self.cold_storage_node,
                                                                   self.base_stations[robot_id])

                use_time.append((2*base_dist_to_cold + unloading_time)/self.transportation_rate)

                # get the time that robot[i] will be waiting
                if i == 1:
                    # get the time of robot[i-1] start using storage
                    # if the robot in the queue head never parked before, start time is the join queue time
                    if len(self.cold_storage_usage_queue[0]) == 2:
                        start_using_storage_time = self.cold_storage_usage_queue[0]['join_queue_time']
                    # if the robot in the queue head has parked in the base before, start time is available in queue wait time
                    elif len(self.cold_storage_usage_queue[0]) == 3:
                        start_using_storage_time = self.cold_storage_queue_wait_time[0]['start_using_storage_time']
                    wait_time.append(abs(use_time[0] + start_using_storage_time - self.env.now))  #  todo: remove abs?
                else:
                    # wait_time.append(max(curr_dist_to_base_time[i-1], wait_time[i-1]) + use_time[i-1])
                    # if robot[i-1] won't start using storage before robot[i] arriving at base
                    if wait_time[i-1] > curr_dist_to_base_time[i-1]:
                        wait_time.append(wait_time[i-1] + use_time[i-1])
                    # if robot[i-1] start using storage, but robot[i] is still on the way to the base
                    else:
                        previous_robot_id = self.cold_storage_usage_queue[i-1]['robot_id']
                        previous_robot_wait_time = .0
                        previous_robot_use_time = .0
                        for robot_info in self.cold_storage_queue_wait_time:
                            if previous_robot_id == robot_info['robot_id']:
                                previous_robot_wait_time = robot_info['wait_time']
                                previous_robot_use_time = robot_info['use_time']
                        # robot[i] takes longer time to get to base, or robot[i-1] takes longer time to finish using storage
                        wait_time.append(max(curr_dist_to_base_time[i-1], previous_robot_wait_time+previous_robot_use_time))

                # remove old wait time before append latest wait time
                for queue_robot in self.cold_storage_queue_wait_time:
                    if robot_id == queue_robot['robot_id']:
                        self.cold_storage_queue_wait_time.remove(queue_robot)

                # get the time of start parking at base:
                # if robot started parking
                if len(self.cold_storage_usage_queue[i]) == 3:
                    start_parking_time = self.cold_storage_usage_queue[i]['start_parking_time']
                    self.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                              'start_parking_time': start_parking_time,
                                                              'wait_time': wait_time[i],
                                                              'use_time': use_time[i]})
                else:
                    # robot has not parked, predict the parking time
                    # start_parking_time = self.env.now + curr_dist_to_base_time[i-1]
                    # robot has not parked, ignore
                    self.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                              'wait_time': wait_time[i],
                                                              'use_time': use_time[i]})
        # return the queried wait time info
        for queue_robot in self.cold_storage_queue_wait_time:
            if robot_name == queue_robot['robot_id']:
                return queue_robot

    def init_wait_queue(self, robot_id, target):
        """
        Initialise the wait queue for using (cold) storage station. There is only one robot in the queue and the robot
        is using the storage.
        """
        curr_dist_to_storage = self.get_total_route_distances(self.curr_node[robot_id],
                                                              self.cold_storage_node)
        self.update_dist_cost_to_target(robot_id, self.curr_node[robot_id],
                                        target, curr_dist_to_storage)
        storage_dist_to_base = self.get_total_route_distances(self.cold_storage_node,
                                                              self.base_stations[robot_id])
        unloading_time = self.unloading_time * self.assigned_picker_n_trays
        use_t = (curr_dist_to_storage + storage_dist_to_base) / self.transportation_rate + unloading_time
        # stop_using_storage_time = self.env.now + use_t
        self.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                  'start_parking_time': None,
                                                  'start_using_storage_time': self.env.now,
                                                  # 'stop_using_storage_time': stop_using_storage_time,
                                                  'wait_time': .0,
                                                  'use_time': use_t})

    def get_cold_storage_queue_wait_time(self, robot_name, robot_curr_node, target, robot_transportation_rate, 
                                         unloading_time, is_head):
        """
        Get the estimated wait time for the robot in the queue before being able to use the cold storage
        Note: this estimated wait time is the minimum waiting time, as the robot could face delay when going to/ return
            from the storage node, especially when the system is complicate. So the robot should not strictly stick to 
            the wait_time when planning a path.
        distance_cost_to_storage: the head of cold_storage_usage_queue
        unloading_time: time of robot unloading at the storage station
        is_head: bool, True: the robot is in the head of self.cold_storage_usage_queue; False: not the head
        """
        wait_time = []
        use_time = []

        storage_node = target
        # todo: local_storage is not tested
        if self.use_local_storage:
            row_id = self.get_row_id_of_row_node(robot_curr_node)
            storage_node = self.local_storage_nodes[row_id]
            
        if is_head:
            try:
                assert self.cold_storage_usage_queue[0]['robot_id'] == robot_name
            except AssertionError:
                raise Exception("%s is not at the head of cold_storage_usage_queue: %s" % (
                    robot_name, self.cold_storage_usage_queue))

            wait_time.append(0)
            curr_distances_to_storage = self.get_total_route_distances(robot_curr_node, storage_node)
            self.update_dist_cost_to_target(robot_name, robot_curr_node, target, curr_distances_to_storage)

            storage_distances_to_base = self.get_total_route_distances(storage_node, self.base_stations[robot_name])
            use_t = (curr_distances_to_storage + storage_distances_to_base
                     ) / robot_transportation_rate + unloading_time
            use_time.append(round(use_t, 1))
            self.cold_storage_queue_wait_time.append({'robot_id': robot_name,
                                                      'join_queue_time': self.cold_storage_usage_queue[0]['join_queue_time'],
                                                      'wait_time': wait_time[0],
                                                      'use_time': use_time[0]})
            return self.cold_storage_queue_wait_time[0]
        
        # wait_time[1] = use_time[0]
        # robot_id = self.cold_storage_usage_queue[1][0]
        # storage_distances_to_base = self.get_total_route_distances(storage_node, self.base_stations[robot_id])
        # use_time[1] = storage_distances_to_base * 2 / robot_transportation_rate
        #
        # wait_time[2] = wait_time[1] + use_time[1]
        # robot_id = self.cold_storage_usage_queue[2][0]
        # storage_distances_to_base = self.get_total_route_distances(storage_node, self.base_stations[robot_id])
        # use_time[2] = storage_distances_to_base * 2 / robot_transportation_rate
        else:
            wait_time.append(self.cold_storage_queue_wait_time[0]['wait_time'])
            use_time.append(self.cold_storage_queue_wait_time[0]['use_time'])
            for i in range(1, len(self.cold_storage_usage_queue)):
                curr_distances_to_base = self.get_total_route_distances(robot_curr_node, self.base_stations[robot_name])
                
                wait_time.append(wait_time[i-1] + use_time[i-1] - (
                        self.env.now - self.cold_storage_queue_wait_time[i-1]['join_queue_time']))
                if wait_time[i] < 0:
                    wait_time[i] = wait_time[i-1] + use_time[i-1]  # todo: better solution to be done
                robot_id = self.cold_storage_usage_queue[i]['robot_id']   # todo: use robot_name directly?
                storage_distances_to_base = self.get_total_route_distances(storage_node, self.base_stations[robot_id])
                use_time.append(storage_distances_to_base * 2 / robot_transportation_rate + unloading_time)
                self.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                          'join_queue_time': self.cold_storage_usage_queue[i]['start_parking_time'],
                                                          'wait_time': wait_time[i],
                                                          'use_time': use_time[i]})

                return self.cold_storage_queue_wait_time[i]

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
        :param hold_t: float, the time that robot plan to occupy the node
        return: True, cancel succeed
        """
        if self._hold[n]['hold_time'][-1] == hold_t:
            self._hold[n]['hold_time'].pop()
            return True

    def extend_hold_time(self, node, hold_t):
        """
        Extend the hold time of the node
        :param node: string, topological node
        :param hold_t: float, the time that robot plan to occupy the node
        """
        if self._hold[node]['hold_time'][-1] is not None:
            self._hold[node]['hold_time'][-1] += hold_t
            return True
        else:
            return False

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

    def init_request_node(self, robot_id, node):
        """
        Requesting of a node at initialisation stage, e.g., spawn robot at his base station
        """
        self.add_act_node(robot_id, node)
        if self.env:
            return self._node_res[node].put(1)

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
        elif self.env:
            self.mark_node_put_queue_idx(robot_name, node)
            return self._node_res[node].put(1)
            # print(len(self._node_res[node].put_queue))
            # if self._node_res[node].put_queue:
            #     yield self._node_res[node].put_queue[-1]
            #     print(len(self._node_res[node].put_queue))
            #     self.node_put_queue[node] = {'req_node': node,
            #                                  'idx': idx}
            #     yield self.env.timeout(0)
        # else:
        #     yield self.env.timeout(0)

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
        if len(self.dead_locks) > 0:
            if robot_name in self.dead_locks:
                self.deadlocks['deadlock_robot_ids'] = copy.deepcopy(self.dead_locks)
                for robot_id in self.dead_locks:
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
            if robot_name in self.dead_locks:
                self.dead_locks.remove(robot_name)
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
            if deadlock and cur_node == self.active_nodes[robot_name][1]:
                self.active_nodes[robot_name].remove(cur_node)
                return True
            elif (not deadlock) and cur_node in self.active_nodes[robot_name]:
                self.active_nodes[robot_name].remove(cur_node)
                return True
            elif cur_node not in self.active_nodes[robot_name]:
                msg = "%s not in active_nodes: %s" % (cur_node, self.active_nodes[robot_name])
                raise Exception(msg)
        else:
            print("Robot %s is not hold by any node" % robot_name)
            return False

    def get_deadlocks_with_queue(self, active_nodes_):
        """
        Find all robots that are requesting each other's node, return all the couple nodes. Considering the storage
        queue as another form of 'request': when robot_01 is in front of robot_01, it's like robot_01 is requesting
        robot_00's node.

        :param active_nodes_: string list, the list of nodes that currently occupied or requested by robots
        return: string list of list, the robots that are waiting for each others' nodes
        """
        active_nodes = copy.deepcopy(active_nodes_)
        # if the robot started parking at base, get the node that the robot (in the storage queue) is waiting for
        cold_storage_queue = {}
        if len(self.cold_storage_usage_queue) > 1:
            for i in range(1, len(self.cold_storage_usage_queue)):
                # if the robot started parking, find out which node the robot is waiting for
                if len(self.cold_storage_usage_queue[i]) > 2:
                    wait_for_id = self.cold_storage_usage_queue[i - 1]['robot_id']
                    wait_for_node = self.agent_nodes[wait_for_id]
                    cold_storage_queue[self.cold_storage_usage_queue[i]['robot_id']] = wait_for_node

            self.cold_storage_queue = cold_storage_queue

            # extend the active_nodes: active_nodes = active_nodes + cold_storage_queue
            # if there are at least two robots in the queue for storage, adding the waiting node to active_nodes
            if len(cold_storage_queue) > 0:
                for robot_id in active_nodes.keys():
                    node = cold_storage_queue.get(robot_id)
                    # if the parking robot is waiting for the robot in the front of cold_storage_usage_queue
                    # update the node(occupied by the robot who is using storage) that the parking robot is waiting for
                    if node is not None:
                        if len(active_nodes[robot_id]) < 2:
                            active_nodes[robot_id].append(node)
                        elif len(active_nodes[robot_id]) == 2:
                            active_nodes[robot_id][1] = node

        # find deadlocks in the extended active_nodes
        deadlocks = self.get_deadlocks(active_nodes)
        self.dead_locks = deadlocks

        return deadlocks

    def is_in_deadlock(self, robot_name):
        """
        Check if the robot is in deadlock, waiting for each other
        :param robot_name: string, robot name
        return: bool, True - the robot is in deadlock
        """
        deadlock = self.get_deadlocks_with_queue(self.active_nodes)
        if deadlock is not None:
            for robot_id in deadlock:
                if robot_name == robot_id:
                    return True
        else:
            return False

    @staticmethod
    def get_deadlocks(active_nodes):
        """
        Get the deadlock loop:
        Two robots in deadlock: A->B->A
        Three robots in deadlock: A->B->C->A
        Four robots in deadlock: A->B->C->D->A
        ...
        Example:
             {'robot_02': ['WayPoint142', 'WayPoint66'],
              'robot_01': ['WayPoint66', 'WayPoint56'],
              'robot_00': ['WayPoint56', 'WayPoint142']}

        :param active_nodes: dict of list, {robot_id: [curr_node, requested_node]}
        return:  string list, robot_ids in deadlock

        Note: We don't consider the situation that two separate deadlocks happening at the same moment
        """

        # get all the agents that have two occupied nodes: one for current, one for requested or prepare to request
        dead_locks = []
        dead_lock_found = False
        act_nodes = {a: active_nodes[a] for a in active_nodes if len(active_nodes[a]) == 2}
        robot_ids = list(act_nodes)
        for i in range(len(robot_ids)):
            dead_locks.append(robot_ids[i])
            pointer_origin = act_nodes[robot_ids[i]][0]
            pointer_i = act_nodes[robot_ids[i]][1]
            j = i + 1
            while j < len(robot_ids):
                pointer_j = act_nodes[robot_ids[j]][0]
                if pointer_j == pointer_i:
                    dead_locks.append(robot_ids[j])
                    pointer_i = act_nodes[robot_ids[j]][1]

                    if pointer_i == pointer_origin:
                        dead_lock_found = True
                        break
                    else:
                        j = i + 1  # reset j
                else:
                    j += 1
                # if all robot_ids have been iterated, break and increase i
                if len(set(dead_locks)) == len(robot_ids):
                    dead_locks = []
                    break

            if dead_lock_found:
                break
            else:
                if robot_ids[i] in dead_locks:
                    dead_locks.remove(robot_ids[i])

        if dead_lock_found:
            return dead_locks
        else:
            return None

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

    def get_total_route_distances(self, start_node, goal_node):
        """
        Get the total route distances from tart node to goal node
        """
        if start_node == goal_node:
            return .0
        else:
            route_distances_sum = 0.0
            route_nodes, route_edges, route_distances = self.get_path_details(start_node, goal_node)
            for route_dist in route_distances:
                route_distances_sum += route_dist
            return route_distances_sum

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
