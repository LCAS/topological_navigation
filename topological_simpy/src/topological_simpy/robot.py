#!/usr/bin/env python

import simpy
from yaml import safe_load
from topological_navigation.route_search2 import TopologicalRouteSearch2
from topological_navigation.tmap_utils import *
from math import hypot, ceil
from functools import partial, wraps
from copy import deepcopy


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
    for name in ['put']:
        if hasattr(resource, name):
            setattr(resource, name, get_wrapper(getattr(resource, name)))


def patch_res_get(resource, pre=None, post=None):
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
    for name in ['get']:
        if hasattr(resource, name):
            setattr(resource, name, get_wrapper(getattr(resource, name)))


class TopoMap():
    def __init__(self, topo_map_file, env=None):
        with open(topo_map_file, "r") as f:
            self.tmap2 = safe_load(f)
        self._route_planner = TopologicalRouteSearch2(self.tmap2)
        self.t_map = deepcopy(self.tmap2)  # used for map node managing
        self.t_map2 = deepcopy(self.tmap2)  # used for map node managing
        self.route_planner = TopologicalRouteSearch2(self.t_map2)  #
        self.env = env

        self._nodes = sorted([n['node']['name'] for n in self.tmap2['nodes']])
        self._nodes_dict = {n['node']['name']: n['node'] for n in self.tmap2['nodes']}
        self._node_res = {}

        self._node_log = {}
        self._hold = {}
        self.active_nodes = {}   # keep all the occupied nodes for checking that any two robots requesting each other's nodes

        self.req_ret = {}  # bool, request return: mode of the node requested
        self.jam = []  # the robot that in traffic jam
        self.com_nodes = []  # the nodes that hold completed robots
        if self.env:
            for n in self._nodes:
                self._node_res[n] = simpy.Container(self.env, capacity=1, init=0)  # each node can hold one robot max
                self._node_log[n] = []
                self.req_ret[n] = None
                self._hold[n] = {'now': [],
                                 'hold_time': [],
                                 'queue_time': [],
                                 'start_moment': [],
                                 'wait_time': []}
                patch_resource(self._node_res[n], post=partial(self.log_resource, n))
                patch_res_put(self._node_res[n], post=partial(self.put_monitor, n))
                patch_res_get(self._node_res[n], post=partial(self.get_monitor, n))

    def log_resource(self, node, resource):
        """
        This is our monitoring callback. Log all put and get operations with container resource.
        :param node: string, topological map node
        :param resource: SimPy resource Container
        """
        item = (
            resource._env.now,
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
                print('!%4d: %s has held longer than expected! Extend the wait time as before' % (
                    self.env.now, node))
                wait_time = self._hold[node]['hold_time'][-2]  # Reset the wait time

            return wait_time

    def adj_hold_info(self, node_name, hold_time, req_ret):
        """
        Whenever release a node, clear the corresponding occupy info
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
            print('!%4d: %s has held longer than expected! Extend the wait time as before' % (
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
        if len(route_nodes) < 2:
            print('One node left before arriving at goal.')
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

    def get_avoiding_route(self, origin, target, avoid_node):
        """
        get the topological route from origin to target but avoiding avoid_node
        :param origin: string, origin topological node
        :param target: string, target topological node
        :param avoid_node: string, topological node that should be avoided when planning the route
        return: string list, route node names, from current node to target node
        """
        self.t_map2 = self.delete_tmap_node(avoid_node)
        self.route_planner = TopologicalRouteSearch2(self.t_map2)
        return self.route_planner.search_route(origin, target)

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

    def delete_tmap_node(self, node_names):
        """
        delete the node from the topological map, including the node and the edges pointed to the node
        :param node_names: string list, node names
        return: topological map with deleted nodes
        """
        idx = 0
        for node_name in node_names:
            while idx < len(self.t_map['nodes']):
                n = self.t_map['nodes'][idx]
                if n['node']['name'] == node_name:
                    children = get_conected_nodes_tmap2(n['node'])  # get all children names, type: string list
                    for child in children:
                        node_idx = get_node_and_idx_from_tmap2(self.t_map, child)
                        for e_idx, n_edge in enumerate(node_idx['node']['edges']):
                            if n_edge['node'] == node_name:
                                self.t_map['nodes'][node_idx['idx']]['node']['edges'].pop(e_idx)
                    self.t_map['nodes'].pop(idx)
                    idx = 0
                    break
                idx = idx + 1
        if self.t_map == self.tmap2:
            print('%s not found in tmap' % node_names)
        else:
            self.t_map2 = deepcopy(self.t_map)
            self.t_map = deepcopy(self.tmap2)
        return self.t_map2

    def get_node_state(self, node):
        """
        Get the status of node level and put_queue
        :param node: string, topological map node
        return: integer, the level and queue info of the node container
        """
        state = None
        if self._node_res[node]:
            if self._node_res[node].level == 0 and self._node_res[node].put_queue == []:
                state = 1
            elif self._node_res[node].level == 1 and self._node_res[node].put_queue == []:
                state = 2
            elif self._node_res[node].level == 1 and self._node_res[node].put_queue != []:
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
        if self.is_in_tra_jam(robot_name):
            print('>% 4d: %s is in traffic jam, prepare to change route' % (self.env.now, robot_name))
            self.pop_free_node(robot_name, node, jam=True)  # pop the node planned to request
            self.add_robot_to_jam(robot_name, node)
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
        print("% 5d:    nodes     %s      are occupied" % (self.env.now, sorted(occupied_nodes)))
        return occupied_nodes

    def add_robot_to_jam(self, robot_name, node):
        """
        Add active nodes to the shared list active_nodes
        :param robot_name: string, robot name
        :param node: string, topological node
        return: string list, the list of robots that currently in traffic jam
        """
        self.jam.append([robot_name, node])
        return self.jam

    def add_act_node(self, robot_name, node):
        """
        Add active nodes to the shared list active_nodes
        :param robot_name: string, robot name
        :param node: string, topological node
        return: string list, the list of nodes that currently occupied or requested by robots
        """
        self.active_nodes[robot_name].append(node)
        return self.active_nodes

    def pop_free_node(self, robot_name, cur_node, jam=False):
        """
        Pop out the node from the active node list when a robot release the node
        :param robot_name: string, robot name
        :param cur_node: string, topological node that the robot release
        :param jam: bool, True - the robot is in the traffic jam
                          False - the robot is not in the traffic jam
        return: bool
        """
        if self.active_nodes[robot_name]:
            if jam and cur_node == self.active_nodes[robot_name].pop(-1):
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
        couple_nodes = []
        for a in active_nodes:
            if len(active_nodes[a]) == 2:
                for b in active_nodes:
                    if len(active_nodes[b]) == 2:
                        if a != b:
                            if active_nodes[a][0] == active_nodes[b][1] and active_nodes[a][1] == active_nodes[b][0]:
                                couple_nodes.append([a, b])
            # elif len(active_nodes[a]) > 2:
            #     print('Robot %s occupies more than 2 nodes!' % a)
            #     return False
            # elif len(active_nodes[a]) < 2:
            #     print('Robot %s occupies only %d node!' % (a, len(active_nodes[a])))
            #     return False
        if couple_nodes:
            return couple_nodes
        else:
            return False

    def is_in_tra_jam(self, robot_name):
        """
        Check if the robot is in the traffic jam, waiting for each other
        :param robot_name: string, robot name
        return: bool, True - the robot is in the traffic jam
        """
        tra_jam = self.get_couples(self.active_nodes)
        if tra_jam:
            for c in tra_jam:
                for r in c:
                    if robot_name == r:
                        return True
        else:
            return False

    @property
    def node_log(self):
        return self._node_log


# noinspection PyBroadException
class Robot():
    def __init__(self, name, tmap, initial_node='A'):
        self._current_node = initial_node
        self._tmap = tmap
        self._env = tmap.env
        self._name = name
        self._speed_m_s = .3
        self._active_process = None
        self.init_hold_t = 5  # initial hold time, for first node
        self._cost = {'robot_name': [],
                      'time_now': [],
                      'time_wait': [],
                      'dist_cost': [],
                      'route': [],
                      'dist_cost_total': [],
                      'time_wait_total': [],
                      'time_cost_total': [],
                      'cost_total': []}

        if self._tmap.env:
            self._tmap.req_ret[initial_node] = 1
            self._tmap.set_hold_time(initial_node, self.init_hold_t)
            self._tmap.active_nodes[self._name] = []
            self._tmap.request_node(self._name, initial_node)
            self.log_cost(self._name, 0, 0, initial_node)

    def _goto(self, target):
        route_nodes = self.get_route_nodes(self._current_node, target)
        interrupted = False
        idx = 0
        while idx < len(route_nodes):
            n = route_nodes[idx]
            route_dist_cost = self._tmap.route_dist_cost([self._current_node] + route_nodes[idx:])
            d_cur = self._tmap.distance(self._current_node, n)
            time_to_travel = int(ceil(d_cur / self._speed_m_s))
            if idx+1 < len(route_nodes):
                # estimate next distance cost and travelling time
                d_next = self._tmap.distance(n, route_nodes[idx+1])
                time_to_travel_next = ceil(d_next / (2 * self._speed_m_s))
            else:
                time_to_travel_next = 0
            hold_time = time_to_travel + time_to_travel_next  # the time node will hold the robot
            print('% 5d:  %s traversing route from node %10s to node %10s (distance: %f, travel time: %d, hold time: %d)'
                  % (self._tmap.env.now, self._name, self._current_node, n, d_cur, time_to_travel, hold_time))

            try:
                # The node to be requested may be occupied by other robots, mark the time when requesting
                start_wait = self._env.now
                self._tmap.set_hold_time(n, hold_time)
                node_state = self._tmap.get_node_state(n)
                if node_state is 1:
                    yield self._tmap.request_node(self._name, n)
                    print('% 5d: active nodes connected to robots: %s ' % (self._env.now, self._tmap.active_nodes))
                else:
                    print('% 5d: %s: %s is occupied, node state: %d' % (self._env.now, self._name, n, node_state))
                    completed_nodes = self._tmap.get_com_nodes()
                    avoid_nodes = [n] + completed_nodes
                    avoid_nodes = list(set(avoid_nodes))  # ignore the duplicated nodes
                    new_route = self.get_route_nodes(self._current_node, target, avoid_nodes)  # avoid node n

                    # there is no new route available and all the other robots stop at their completed nodes
                    if (new_route is None) and (len(completed_nodes) == len(self._tmap.active_nodes) - 1):
                        print('?% 4d: %s: route %s blocked by other completed nodes - %s' % (
                            self._env.now, self._name, route_nodes, completed_nodes
                        ))
                        interrupted = True
                        break
                    elif new_route:
                        new_route_dc = self._tmap.route_dist_cost([self._current_node] + new_route)  # dc: distance cost
                    elif not new_route:
                        new_route_dc = 10000   # no new route, return a big distance cost
                    wait_time = self._tmap.get_wait_time(n)
                    time_cost = self.time_to_dist_cost(wait_time)
                    old_route_cost = route_dist_cost + time_cost
                    print('$% 4d: old_route_cost = %d, new_route_dc = %d' % (
                        self._env.now, old_route_cost, new_route_dc))
                    if old_route_cost > new_route_dc:
                        route_nodes = new_route
                        self._tmap.cancel_hold_time(n, hold_time)  # found cheap route, cancel the hold_time just be set
                        idx = 0
                        print('~%4d: %s go NEW route: %s' % (self._env.now, self._name, route_nodes))
                        continue
                    else:
                        print('*% 4d: %s wait %d, use old route %s' % (
                            self._env.now, self._name, wait_time, [self._current_node] + route_nodes[idx:]))
                        yield self._tmap.request_node(self._name, n)
                        for j in self._tmap.jam:
                            for r in j:
                                if r == self._name:
                                    print('|% 4d: %s is in traffic jam, change route now' % (self._env.now, self._name))
                                    route_nodes = new_route
                                    self._tmap.cancel_hold_time(n, hold_time)   # cancel the hold_time just set
                                    idx = 0
                                    print('^% 4d: %s go NEW route: %s' % (self._env.now, self._name, route_nodes))
                                    break
                        if idx == 0:
                            self.log_cost(self._name, 0, 0, 'CHANGE ROUTE')  # for monitor
                            continue  # change to new route
                        else:
                            pass  # travel to the requested node
                if self._env.now - start_wait > 0:  # The time that the robot has waited since requesting
                    print('$% 4d:  %s has lock on %s after %d' % (
                        self._env.now, self._name, n,
                        self._env.now - start_wait))
            except Exception:
                # Not triggered when requesting an occupied node!
                # Not triggered when the robot has a goal running and be assigned a new goal
                print('% 5d: %s INTERRUPTED while waiting to gain access to go from node %s going to node %s' % (
                    self._tmap.env.now, self._name,
                    self._current_node, n
                ))
                interrupted = True
                break

            try:
                time_to_travel = ceil(d_cur / (2 * self._speed_m_s))
                yield self._tmap.env.timeout(time_to_travel)
                yield self._tmap.release_node(self._name, self._current_node)
                # The robot is reaching at the half way between the current node and next node
                print('% 5d:  %s ---> %s reaching half way ---> %s' % (
                    self._tmap.env.now, self._current_node, self._name, n))
                self._current_node = n
                remain_time_to_travel = ceil(d_cur / self._speed_m_s) - time_to_travel
                yield self._tmap.env.timeout(remain_time_to_travel)
                print('@% 4d:  %s reached node %s' % (self._tmap.env.now, self._name, n))
                self.log_cost(self._name, self._env.now - start_wait, d_cur, n)  # for monitor
                yield self._tmap.env.timeout(0)
            except simpy.Interrupt:  # When the robot has a running goal but being assigned a new goal
                print('% 5d: %s INTERRUPTED while travelling from node %s going to node %s' % (
                    self._tmap.env.now, self._name,
                    self._current_node, n
                ))
                self.log_cost(self._name, 0, 0, 'INTERRUPTED')  # for monitor
                print('% 5d: @@@ %s release previously acquired target node %s' % (self._env.now, self._name, n))
                yield self._tmap.release_node(self._name, n)
                interrupted = True
                break
            idx = idx + 1  # go to next while loop

        if interrupted:
            # When the robot has a goal running and be assigned a new goal node
            print('% 5d: %s ABORTED at %s' % (self._tmap.env.now, self._name, self._current_node))
            self.log_cost(self._name, 0, 0, 'ABORTED')  # for monitor
        else:
            print('.% 4d: %s COMPLETED at %s' % (self._tmap.env.now, self._name, self._current_node))
            self.log_cost(self._name, 0, 0, 'COMPLETED')  # for monitor
            self._tmap.add_com_node(self._current_node)

    def goto(self, target):
        """
        Start the _goto(target) process
        :param target: string, topological node that to be reached
        """
        if self._tmap.env:
            if self._active_process:
                if self._active_process.is_alive:  # The robot has an alive goal running
                    self._active_process.interrupt()
                    self._active_process = None  # Clear the old goal
            self._active_process = self._tmap.env.process(self._goto(target))
        else:
            self._current_node = target

    def get_route_nodes(self, cur_node, target, avoid_nodes=None):
        """
        get the topological route nodes from current node to target node, if request_node failed,
        then get the route nodes from current node to target node and avoid the failed node.
        :param cur_node: string, the current node traversed
        :param target: string, the target node that the robot will go to
        :param avoid_nodes: string list, the nodes that won't be considered when planning route
        return: string list, route node names, from current node to target node
        """
        if self._tmap.env:
            if target == cur_node:
                print('% 5d: %s is already at target %s' % (self._tmap.env.now, self._name, target))
                return [target]
            else:
                if avoid_nodes is None:
                    route = self._tmap.get_route(cur_node, target)
                else:
                    route = self._tmap.get_avoiding_route(cur_node, target, avoid_nodes)

                if route is None:
                    return None
                else:
                    r = route.source[1:]
                    r.append(target)
                    print('% 5d: %s going from node %s to node %s via route %s' % (
                        self._tmap.env.now, self._name, cur_node, target, r))
                    return r

    def time_to_dist_cost(self, waiting_time):
        """
        :param waiting_time: the time robot will be waiting
        return: float, the distance cost
        """
        return self._speed_m_s*waiting_time

    def log_cost(self, robot_name, time_wait, dist_cost, node_name):
        """
        Update the time and distance cost
        :param robot_name: string, robot name
        :param time_wait: integer, waiting time before occupy the requested node
        :param dist_cost: float, the distance from current node to next node
        :param node_name: string, the node will be traversed
        return: dictionary, cost info
        """
        dist_cost_total = 0
        time_wait_total = 0
        dist_cost = float('{:.2f}'.format(dist_cost))
        self._cost['robot_name'].append(robot_name)
        self._cost['time_now'].append(self._env.now)
        self._cost['time_wait'].append(time_wait)
        self._cost['dist_cost'].append(dist_cost)
        self._cost['route'].append(node_name)
        for n in self._cost['dist_cost']:
            dist_cost_total = dist_cost_total + n
        dist_cost_total = float('{:.2f}'.format(dist_cost_total))
        self._cost['dist_cost_total'].append(dist_cost_total)
        for n in self._cost['time_wait']:
            time_wait_total = time_wait_total + n
        time_cost_total = float('{:.2f}'.format(self.time_to_dist_cost(time_wait_total)))
        self._cost['time_wait_total'].append(time_wait_total)
        self._cost['time_cost_total'].append(time_cost_total)
        cost_total = float('{:.2f}'.format(time_cost_total + dist_cost_total))
        self._cost['cost_total'].append(cost_total)
        return self._cost

    @property
    def cost(self):
        return self._cost
