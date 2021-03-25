#!/usr/bin/env python

import itertools
import random
import simpy
from yaml import safe_load
from topological_navigation.route_search2 import TopologicalRouteSearch2
from topological_navigation.tmap_utils import *
from math import hypot, ceil
from functools import partial, wraps
from copy import deepcopy

RANDOM_SEED = 42
GAS_STATION_SIZE = 200  # liters
THRESHOLD = 20  # Threshold for calling the tank truck (in %)
FUEL_TANK_SIZE = 40  # liters
FUEL_TANK_LEVEL = [5, 25]  # Min/max levels of fuel tanks (in liters)
REFUELING_SPEED = 2  # liters / second
TANK_TRUCK_TIME = 30  # Seconds it takes the tank truck to arrive
T_INTER = [1, 30]  # Create a car every [min, max] seconds
SIM_TIME = 100  # Simulation time in seconds


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
        self._hold_time_log = {}
        self.queue_time = 10

        self._req_suc = None  # bool, request node succeed
        self._has_queue_ = None
        self.req_ret = None  # bool, request return: mode of the node requested
        if self.env:
            for n in self._nodes:
                self._node_res[n] = simpy.Container(self.env, capacity=1, init=0)  # each node can hold one robot max
                self._node_log[n] = []
                self._hold_time_log[n] = {'now': [],
                                          'robot_name': [],
                                          'hold_time': [],
                                          'q_len': [],
                                          'queue_time': [],
                                          'start_moment': [],
                                          'wait_time': [],
                                          'release_moment': []}
                patch_resource(self._node_res[n], post=partial(self.log_resource, n))

    def log_resource(self, node, resource):
        """This is our monitoring callback."""
        item = (
            resource._env.now,  # The current simulation time
            resource.level
        )
        self._node_log[node].append(item)

    def query_wait(self, robot_name, node_name, hold_time, req_ret):
        """
        Log the waiting time for estimating the time cost when plan a new route
        :param robot_name: string, robot name
        :param node_name: string, name of the node to be hold
        :param hold_time: integer, total time that the node will be hold by the robot
        :param req_ret: integer, [1,2,3,4], level and queue info of requested node
        return: time info about the node, especially wait_time - how long a robot should wait
                before the node being released

        # queue_time: the time that robot needs to wait before the current queue gone, robot cannot acquire the node now
        """
        if req_ret is 1:
            now = self.env.now
            robot_name = robot_name
            hold_time = hold_time
            start_moment = now
            queue_time = 0
            q_len = 0
            wait_time = hold_time
            release_moment = start_moment + hold_time

        if req_ret is 2:
            now = self.env.now
            robot_name = robot_name
            queue_time = 0
            q_len = 0
            start_moment = self._node_log[node_name][-1][0]
            hold_time = self._hold_time_log[node_name]['hold_time'][-1]
            wait_time = hold_time - (now - start_moment)
            release_moment = start_moment + hold_time

        if req_ret is 3:
            now = self.env.now
            robot_name = robot_name
            q_len = 1
            queue_time = hold_time  # TODO double check
            hold_time = self._hold_time_log[node_name]['hold_time'][-1]  # todo: double check
            start_moment = self._node_log[node_name][-1][0]
            wait_time = hold_time + queue_time - (now - start_moment)
            release_moment = start_moment + hold_time + queue_time

        if req_ret is 4:  # todo: never happen? same as 2?
            pass

        self._hold_time_log[node_name]['now'].append(self.env.now)
        self._hold_time_log[node_name]['robot_name'].append(robot_name)
        self._hold_time_log[node_name]['hold_time'].append(hold_time)
        self._hold_time_log[node_name]['q_len'].append(q_len)
        self._hold_time_log[node_name]['queue_time'].append(queue_time)
        self._hold_time_log[node_name]['start_moment'].append(start_moment)
        self._hold_time_log[node_name]['wait_time'].append(wait_time)
        self._hold_time_log[node_name]['release_moment'].append(release_moment)

        return self._hold_time_log
        # self._hold_time_log[node_name]['now'].append(self.env.now)
        # self._hold_time_log[node_name]['robot_name'].append(robot_name)
        # self._hold_time_log[node_name]['request'].append(request_suc)
        # if request_suc:
        #     queue_time = 0
        #     self._hold_time_log[node_name]['hold_time'].append(hold_time)
        #     if self._node_res[node_name].level:  # the node is occupied TODO: redundant?
        #         start_time = self._node_log[node_name][-1][0]  # the start time when the node was occupied
        #         self._hold_time_log[node_name]['start_time'].append(start_time)
        #
        # else:  # request node failed
        #     if join_queue:
        #         # queue_time: robot join the queue, it's the time that the robot will be hold
        #         queue_time = hold_time
        #     else:
        #         queue_time = 0
        #     hold_time = self._hold_time_log[node_name]['hold_time'][-1]
        #     self._hold_time_log[node_name]['hold_time'].append(hold_time)
        #     self._hold_time_log[node_name]['start_time'].append(
        #         self._hold_time_log[node_name]['start_time'][-1])
        #
        # # queue_len: the number of robot in the queue
        # q_len = len(self._node_res[node_name].put_queue)
        # self._hold_time_log[node_name]['queue_len'].append(q_len)
        # self._hold_time_log[node_name]['queue_time'].append(queue_time)
        # # wait_time: from now on --> the end of hold,
        # #            it's the time that another robot should wait before occupying
        # wait_time = queue_time + hold_time - (self._hold_time_log[node_name]['now'][-1]
        #                                       - self._hold_time_log[node_name]['start_time'][-1])
        # self._hold_time_log[node_name]['wait_time'].append(wait_time)
        # self._hold_time_log[node_name]['release_time'].append(
        #     self._hold_time_log[node_name]['start_time'][-1] + self._hold_time_log[node_name]['hold_time'][-1])
        # return self._hold_time_log

    def get_nodes(self):
        return self._nodes

    def distance(self, nodeA, nodeB):
        a = self._nodes_dict[nodeA]
        b = self._nodes_dict[nodeB]
        return hypot(
            a['pose']['position']['x'] - b['pose']['position']['x'],
            a['pose']['position']['y'] - b['pose']['position']['y']
        )

    def route_dist_cost(self, route_nodes):
        cost = 0.0
        if len(route_nodes) < 2:  # todo: forgot cost from current node to the first node of the route?
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
        """
        if self.env:
            return time_cost * speed_m_s

    def get_avoiding_route(self, origin, target, avoid_node):
        """
        get the topological route from origin to target but avoiding avoid_node
        :param origin: string, origin topological node
        :param target: string, target topological node
        :param avoid_node: string, topological node that should be avoided when planning the route
        """
        self.t_map2 = self.delete_tmap_node(avoid_node)
        self.route_planner = TopologicalRouteSearch2(self.t_map2)
        return self.route_planner.search_route(origin, target)

    def delete_tmap_node(self, node_name):
        """
        delete the node from the topological map, including the node and the edges pointed to the node
        :param node_name: string, node name
        """
        for idx, n in enumerate(self.t_map['nodes']):
            if n['node']['name'] == node_name:
                children = get_conected_nodes_tmap2(n['node'])  # get all children names, type: string list
                for child in children:
                    node_idx = get_node_and_idx_from_tmap2(self.t_map, child)
                    for e_idx, n_edge in enumerate(node_idx['node']['edges']):
                        if n_edge['node'] == node_name:
                            self.t_map['nodes'][node_idx['idx']]['node']['edges'].pop(e_idx)
                pop_node = self.t_map['nodes'].pop(idx)
                self.t_map2 = deepcopy(self.t_map)
                #self.t_map['nodes'].append(pop_node)
                self.t_map = deepcopy(self.tmap2)
                break
            elif idx == len(self.t_map['nodes']):
                print('node %s not found' % node)
                return self.env.timeout(0)
        return self.t_map2

    def request_node(self, node, force_req=False):
        """
        Put one robot into the _node_res, i.e., the node resource is occupied by the robot.
        Note: the robot starts requesting the next route node when the robot reaches the current node
        :param node: string, the node to be requested
        :param force_req: bool, - True: request anyway no matter the node is occupied or not
                                 - False: do not request the node if the node is occupied
        """
        self.req_ret = None   # request return
        if self.env:
            if self._node_res[node].level == 0 and self._node_res[node].put_queue == []:
                self.req_ret = 1
                return self._node_res[node].put(1)
            else:
                if self._node_res[node].level == 1 and self._node_res[node].put_queue == []:
                    self.req_ret = 2
                elif self._node_res[node].level == 1 and self._node_res[node].put_queue != []:
                    self.req_ret = 3
                elif self._node_res[node].level == 0 and self._node_res[node].put_queue != []:
                    self.req_ret = 4
                if force_req:
                    return self._node_res[node].put(1)
                else:
                    return self.env.timeout(0)  # give up requesting

        # self._req_suc = None
        # self._has_queue_ = None
        # if self.env:
        #     if force_req:
        #         if not self._node_res[node].put_queue:
        #             self._has_queue_ = False
        #             print('%s put_queue: %s empty' % (node, self._node_res[node].put_queue))
        #         else:
        #             self._has_queue_ = True
        #             print('%s put_queue has  %s robot waiting' % (node, len(self._node_res[node].put_queue)))
        #         return self._node_res[node].put(1)  # join_queue = True ?
        #     else:
        #         if self._node_res[node].level == 0 and self._node_res[node].put_queue == []:
        #             self._req_suc = True
        #             return self._node_res[node].put(1)
        #         else:
        #             self._req_suc = False
        #             return self.env.timeout(0)  # return the len(self._node_res[node].put_queue)? TODO

    def release_node(self, node):
        """
        Get the robot out of the _node_res, i.e., the node resource is released, the robot leaves this node
        """
        if self.env:
            if self._node_res[node].level > 0:
                return self._node_res[node].get(1)
            else:
                return self.env.timeout(0)

    def get_route(self, origin, target):
        return self._route_planner.search_route(origin, target)

    def monitor(self, freq=2):
        """
        Get the topological map nodes that occupied by the robot currently
        """
        occupied_nodes = []
        for n in self._node_res:
            if self._node_res[n].level > 0:
                occupied_nodes.append(n)
        print("*% 4d:    nodes     %s      are occupied" % (self.env.now, sorted(occupied_nodes)))


class Robot():
    def __init__(self, name, tmap, initial_node='A'):
        self._current_node = initial_node
        self._tmap = tmap
        self._env = tmap.env
        self._name = name
        self._speed_m_s = .3
        self._active_process = None
        self.route_nodes = None
        self.init_hold_t = 5  # initial hold time, for first node
        self._cost = {'robot_name': [],
                      'time_now': [],
                      'cost': [],
                      'time_cost': [],
                      'dist_cost': [],
                      'route': []}

        if self._tmap.env:
            self._tmap.request_node(initial_node)
            self._tmap.query_wait(self._name, initial_node, self.init_hold_t, 1)
            # self._tmap._node_res[initial_node].count = 1
            # print(self._current_node)
            # self._tmap.env.process(self._goto(initial_node))

    def _goto(self, target):
        self.route_nodes = self.get_route_nodes(self._current_node, target)
        interrupted = False
        idx = 0
        while idx < len(self.route_nodes):
            n = self.route_nodes[idx]
            route_dist_cost = self._tmap.route_dist_cost([self._current_node] + self.route_nodes)
            #  get the distance between current and next node:
            d_cur = self._tmap.distance(self._current_node, n)
            time_to_travel = int(ceil(d_cur / self._speed_m_s))
            if idx+1 < len(self.route_nodes):
                # estimate next distance cost and travelling time
                d_next = self._tmap.distance(n, self.route_nodes[idx+1])
                time_to_travel_next = ceil(d_next / (2 * self._speed_m_s))
            else:
                time_to_travel_next = 0
            hold_time = time_to_travel + time_to_travel_next  # the time node will hold the robot

            print('% 5d:  %s traversing route from node %10s to node %10s (distance: %f, travel time: %d, hold time: %d)'
                  % (self._tmap.env.now, self._name, self._current_node, n, d_cur, time_to_travel, hold_time))
            
            try:
                # The node to be requested may be occupied by other robots, mark the time when requesting
                start_wait = self._env.now  
                yield self._tmap.request_node(n, force_req=False)

                if self._tmap.req_ret is 1:
                    self._tmap.query_wait(self._name, n, hold_time, self._tmap.req_ret)  # TODO: rewrite query_wait
                else:  # the node is occupied, find a new route, then compare cost
                    new_route = self.get_route_nodes(self._current_node, target, n, False)  # avoid node n TODO: rewrite get_route_nodes
                    new_route_dc = self._tmap.route_dist_cost([self._current_node] + new_route)  # dc: distance cost

                    if self._tmap.req_ret is 2:
                        query_wait = self._tmap.query_wait(self._name, n, hold_time, self._tmap.req_ret)
                        wait_time = query_wait[n]['wait_time'][-1]
                        time_cost = self.time_to_dist_cost(wait_time)
                        old_route_cost = route_dist_cost + time_cost
                        if old_route_cost > new_route_dc:
                            self.route_nodes = new_route
                            idx = 0
                            continue
                        else:
                            print('Use old route, wait node being released')
                            yield self._tmap.request_node(n, force_req=True)

                    if self._tmap.req_ret is 3:
                        query_wait = self._tmap.query_wait(self._name, n, hold_time, self._tmap.req_ret)
                        wait_time = query_wait[n]['wait_time'][-1]
                        time_cost = self.time_to_dist_cost(wait_time)
                        old_route_cost = route_dist_cost + time_cost
                        if old_route_cost > new_route_dc:
                            self.route_nodes = new_route
                            idx = 0
                            continue
                        else:
                            print('Use old route, wait node being released')
                            yield self._tmap.request_node(n, force_req=True)

                    if self._tmap.req_ret is 4:
                        query_wait = self._tmap.query_wait(self._name, n, hold_time, self._tmap.req_ret)
                        wait_time = query_wait[n]['wait_time'][-1]
                        time_cost = self.time_to_dist_cost(wait_time)
                        old_route_cost = route_dist_cost + time_cost
                        if old_route_cost > new_route_dc:
                            self.route_nodes = new_route
                            idx = 0
                            continue
                        else:
                            print('Use old route, wait node being released')
                            yield self._tmap.request_node(n, force_req=True)


                # hold_times = self._tmap.query_wait(self._name, n, hold_time, self._tmap._req_suc, not self._tmap._req_suc)
                # wait_time = hold_times[n]['wait_time'][-1]
                # time_cost = self.time_to_dist_cost(wait_time)
                # cost = route_dist_cost + time_cost
                #
                # if self._tmap._req_suc is False:
                #     print('+++ %8s use a new route avoiding node %10s +++' % (self._name, n))
                #     new_route_nodes = self.get_route_nodes(self._current_node, target, n, self._tmap._req_suc)
                #     new_route_dist_cost = self._tmap.route_dist_cost([self._current_node] + new_route_nodes)
                #     #
                #     # hold_times = self._tmap.query_wait(self._name, n, hold_time, self._tmap._req_suc, not self._tmap._req_suc)
                #     # wait_time = hold_times[n]['wait_time'][-1]
                #     # time_cost = self.time_to_dist_cost(wait_time)
                #     # cost = route_dist_cost + time_cost
                #
                #     if new_route_nodes[0] == self._current_node:
                #         # cannot find an alternative route to avoid the occupied node n
                #         print('=== %8s no new route found, queueing for requesting node %10s ===' % (self._name, n))
                #         yield self._tmap.request_node(n, False)  # join the requesting queue
                #         self._tmap.query_wait(self._name, n, hold_time, self._tmap._req_suc, True)
                #     else:
                #         print('### %4s finds new route %s' % (self._name, new_route_nodes))
                #         # request failed, use the latest hold_time
                #         time_cost = 0  # no waiting time
                #         self.update_cost(time_cost, new_route_nodes, n)
                #         if new_route_dist_cost < cost:
                #             print('### Use new route %s' % new_route_nodes)
                #             self.route_nodes = new_route_nodes
                #             idx = 0
                #             continue
                #         else:
                #             print('### Use current route, waiting for %4d sec before node %8s released' % (wait_time, n))
                #             yield self._tmap.request_node(n, False)  # join the requesting queue
                #             self._tmap.query_wait(self._name, n, hold_time, self._tmap._req_suc, True)
                # else:
                #     #self._tmap.query_wait(self._name, n, hold_time, self._tmap._req_suc)
                #     #self.update_cost(time_cost, self.route_nodes[idx:], n)
                #     print('')

                if self._env.now - start_wait > 0:
                    # The time that the robot has waited since requesting
                    print('$$$$ % 5d:  %s has lock on %s after %d' % (
                        self._env.now, self._name, n,
                        self._env.now - start_wait))
            except:
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
                yield self._tmap.release_node(self._current_node)  # TODO request = False?
                # The robot is reaching at the half way between the current node and next node
                print('% 5d:  %s ---> %s reaching half way ---> %s' % (
                    self._tmap.env.now, self._current_node, self._name, n))
                self._current_node = n
                remain_time_to_travel = ceil(d_cur / self._speed_m_s) - time_to_travel
                yield self._tmap.env.timeout(remain_time_to_travel)
                print('% 5d:  %s reached node %s' % (self._tmap.env.now, self._name, n))
                yield self._tmap.env.timeout(0)  # Go back to the next env.step()
            except simpy.Interrupt:  # When the robot has a running goal but being assigned a new goal
                print('% 5d: %s INTERRUPTED while travelling from node %s going to node %s' % (
                    self._tmap.env.now, self._name,
                    self._current_node, n
                ))
                print('% 5d: @@@ %s release previously acquired target node %s' % (self._env.now, self._name, n))
                yield self._tmap.release_node(n)
                interrupted = True
                break
            idx = idx + 1  # go to next while loop

        if interrupted:
            # When the robot has a goal running and be assigned a new goal node
            print('% 5d: %s ABORTED at %s' % (self._tmap.env.now, self._name, self._current_node))
        else:
            print('% 5d: %s COMPLETED at %s' % (self._tmap.env.now, self._name, self._current_node))

    def goto(self, target):
        if self._tmap.env:
            if self._active_process:
                if self._active_process.is_alive:  # The robot has an alive goal running
                    self._active_process.interrupt()
                    self._active_process = None  # Clear the old goal
            self._active_process = self._tmap.env.process(self._goto(target))
        else:
            self._current_node = target

    def get_route_nodes(self, cur_node, target, avoid_node=None, request_suc=True):
        """
        get the topological route nodes from current node to target node, if request_node failed,
        then get the route nodes from current node to target node and avoid the failed node.
        :param cur_node: string, the current node traversed
        :param target: string, the target node that the robot will go to
        :param avoid_node: string, the failed node when requesting
        :param request_suc: bool, if requesting node failed, then request_suc = False
        return: string list, route node names, from current node to target node
        """
        if self._tmap.env:
            if target == cur_node:
                print('% 5d: %s is already at target %s' % (self._tmap.env.now, self._name, target))
                return [target]
            else:
                if request_suc:
                    route = self._tmap.get_route(cur_node, target)
                else:
                    route = self._tmap.get_avoiding_route(cur_node, target, avoid_node)

                if route is None:
                    return [cur_node]
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

    def update_cost(self, time_cost, route_nodes, node_name):
        """
        Update the time and distance cost
        :param time_cost: float, waiting time cost
        :param route_nodes: string list, the route robot will be traversing
        :param node_name: string, the node traversed
        return: dictionary
        """
        dist_cost = self._tmap.route_dist_cost(route_nodes)
        cost = dist_cost + time_cost
        self._cost['robot_name'].append(self._name)
        self._cost['time_now'].append(self._env.now)
        self._cost['time_cost'].append(time_cost)
        self._cost['dist_cost'].append(dist_cost)
        self._cost['cost'].append(cost)
        self._cost['route'].append(node_name)

        return self._cost


def car(name, env, gas_station, fuel_pump):
    """A car arrives at the gas station for refueling.

    It requests one of the gas station's fuel pumps and tries to get the
    desired amount of gas from it. If the stations reservoir is
    depleted, the car has to wait for the tank truck to arrive.

    """
    fuel_tank_level = random.randint(*FUEL_TANK_LEVEL)
    print('%s queuing at gas station at %.1f' % (name, env.now))
    with gas_station.request() as req:
        start = env.now
        # Request one of the gas pumps
        yield req
        print('%s serving at gas station at %.1f' % (name, env.now))

        # Get the required amount of fuel
        liters_required = FUEL_TANK_SIZE - fuel_tank_level
        yield fuel_pump.get(liters_required)

        # The "actual" refueling process takes some time
        yield env.timeout(liters_required / REFUELING_SPEED)

        print('%s finished refueling in %.1f seconds.' % (name,
                                                          env.now - start))


def gas_station_control(env, fuel_pump):
    """Periodically check the level of the *fuel_pump* and call the tank
    truck if the level falls below a threshold."""
    while True:
        print('  fuel level at %f' % fuel_pump.level)
        if fuel_pump.level / fuel_pump.capacity * 100 < THRESHOLD:
            # We need to call the tank truck now!
            print('Calling tank truck at %d' % env.now)
            # Wait for the tank truck to arrive and refuel the station
            yield env.process(tank_truck(env, fuel_pump))

        yield env.timeout(10)  # Check every 10 seconds


def tank_truck(env, fuel_pump):
    """Arrives at the gas station after a certain delay and refuels it."""
    yield env.timeout(TANK_TRUCK_TIME)
    print('Tank truck arriving at time %d' % env.now)
    amount = fuel_pump.capacity - fuel_pump.level
    print('Tank truck refuelling %.1f liters.' % amount)
    yield fuel_pump.put(amount)


def car_generator(env, gas_station, fuel_pump):
    """Generate new cars that arrive at the gas station."""
    for i in itertools.count():
        yield env.timeout(random.randint(*T_INTER))
        env.process(car('Car %d' % i, env, gas_station, fuel_pump))


if __name__ == '__main__':
    # Setup and start the simulation
    print('Gas Station refuelling')
    random.seed(RANDOM_SEED)

    # Create environment and start processes
    env = simpy.Environment()
    gas_station = simpy.Resource(env, 2)
    fuel_pump = simpy.Container(env, GAS_STATION_SIZE, init=GAS_STATION_SIZE)
    env.process(gas_station_control(env, fuel_pump))
    env.process(car_generator(env, gas_station, fuel_pump))

    # Execute!
    env.run(until=SIM_TIME)
