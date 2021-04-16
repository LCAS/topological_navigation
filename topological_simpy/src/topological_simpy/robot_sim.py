#!/usr/bin/env python

import simpy
from math import ceil
from topological_simpy.robot import Robot
from numpy import isclose


# noinspection PyBroadException
class RobotSim(Robot):
    def __init__(self, robot_id, transportation_rate, max_n_trays, unloading_time, env, topo_graph, verbose):

        super(RobotSim, self).__init__(robot_id, transportation_rate, max_n_trays, unloading_time, env, topo_graph, verbose)

        self._active_process = None
        self.init_hold_t = 5  # initial hold time, for first node TODO: assign a dynamic time--> reset when picker calls
        self._cost = {'robot_id': [],
                      'time_now': [],
                      'time_wait': [],
                      'dist_cost': [],
                      'route': [],
                      'dist_cost_total': [],
                      'time_wait_total': [],
                      'time_cost_total': [],
                      'cost_total': []}

        self.robot_state = "INIT"  # "INIT", "ACCEPTED", "TRAVELLING", "ARRIVED", "LOADED_ON", "LOADED_OFF"
        # self.robot_resource = simpy.Container(self.env, capacity=1, init=0)  # robot can only be occupied by 1 picker

        if self.graph.env:
            self.graph.req_ret[self.curr_node] = 1
            self.graph.set_hold_time(self.curr_node, self.init_hold_t)
            self.graph.active_nodes[self.robot_id] = []
            self.graph.request_node(self.robot_id, self.curr_node)
            self.log_cost(self.robot_id, 0, 0, self.curr_node)

        self.action = self.env.process(self.normal_operation())

    def call_robot(self, picker_id, robot_id):
        self.loginfo("? %5.1f: %s call robot %s" % (self.env.now, picker_id, robot_id))
        return self.robot_resource.put(1)
        # self.set_robot_state("ACCEPTED")
        # self.log_cost(". %5.1f: call robot: ACCEPTED" % self.env.now)
        # yield self.env.timeout(0)

    def release_robot(self):
        if self.robot_resource.level > 0:
            self.loginfo(". %5.1f: release robot %s" % (self.env.now, self.robot_id))
            return self.robot_resource.get(1)
            # self.set_robot_state("INIT")
            # self.loginfo(". %5.1f: release robot: INIT" % self.env.now)

    def set_robot_state(self, state):
        """
        robot state:
                    "INIT",  initial state
                    "ACCEPTED", robot accepted picker's call, i.e., robot had been assigned to a picker
                    "TRAVELLING", robot is travelling to the goal node
                    "LOADED_ON", picker loaded trays on robot
                    "LOADED_OFF", robot loaded trays off in a storage node
                    "ARRIVED", robot arrived at the goal node
        """
        self.robot_state = state

    def trays_loaded(self):
        """picker calls this to indicate the trays are loaded"""
        self.n_empty_trays -= self.assigned_picker_n_trays
        self.n_full_trays += self.assigned_picker_n_trays
        self.loaded = True
        self.set_robot_state("LOADED")  # "LOADED_ON" necessary ?

    def assign_robot_to_picker(self, picker_id, picker_node, n_trays, local_storage_node):
        """assign a picker to the robot, if it is idle - called by scheduler"""
        try:
            assert self.mode == 0
        except AssertionError:
            raise Exception(
                "Scheduler is trying to assign %s to %s, but robot is in %d" % (self.robot_id, picker_id, self.mode))

        self.assigned_picker_id = picker_id
        self.assigned_picker_node = picker_node
        self.assigned_picker_n_trays = n_trays
        self.assigned_local_storage_node = local_storage_node
        # self.call_robot(picker_id, self.robot_id)
        self.set_robot_state('ACCEPTED')

    def normal_operation(self):
        """
        normal operation sequences of the robot in different modes  TODO: check picker calling info before taking action!
        """
        idle_start_time = self.env.now
        transportation_start_time = 0.
        loading_start_time = 0.
        unloading_start_time = 0.
        charging_start_time = 0.

        while True:
            if self.picking_finished and (self.mode == 0 or self.mode == 5):
                self.loginfo("X %5.1f: all rows picked. %s exiting" % (self.env.now, self.robot_id))
                self.env.exit("all rows picked and idle")
                break

            if self.mode == 0:
                # check for assignments
                if self.assigned_picker_id is not None:
                    self.time_spent_idle += self.env.now - idle_start_time
                    # change mode to transport to picker
                    self.mode = 1
                    transportation_start_time = self.env.now
                    self.loginfo("  %5.1f: %s is assigned to %s" %
                                 (self.env.now, self.robot_id, self.assigned_picker_id))

                # TODO: idle state battery charge changes
                if self.battery_charge < 40.0 or isclose(self.battery_charge, 40.0):
                    self.loginfo("  %5.1f: battery low on %s, going to charging mode" % (self.env.now, self.robot_id))
                    self.time_spent_idle += self.env.now - idle_start_time
                    # change mode to charging
                    self.mode = 5
                    charging_start_time = self.env.now

            elif self.mode == 1:
                # farm assigns the robot to a picker by calling assign_robot_to_picker
                # go to picker_node from curr_node
                self.loginfo("  %5.1f: %s going to %s" % (self.env.now, self.robot_id, self.assigned_picker_node))
                yield self.env.process(self.goto(self.assigned_picker_node))
                self.time_spent_transportation += self.env.now - transportation_start_time
                self.loginfo("@ %5.1f: %s reached %s" % (self.env.now, self.robot_id, self.assigned_picker_node))
                self.set_robot_state("ARRIVED")
                # change mode to waiting_for_loading
                self.continue_transporting = False
                self.mode = 2
                loading_start_time = self.env.now

            elif self.mode == 2:
                self.loginfo("  %5.1f: %s is waiting for the trays to be loaded" % (self.env.now, self.robot_id))
                yield self.env.process(self.wait_for_loading())  # this reset mode to 3
                self.time_spent_loading += self.env.now - loading_start_time
                self.loginfo("L %5.1f: trays are loaded on %s" % (self.env.now, self.robot_id))
                self.set_robot_state("LOADED")  # TODO: LOADED_ON
                while True:
                    if self.continue_transporting:
                        # change mode to transporting to storage
                        self.mode = 3
                        transportation_start_time = self.env.now
                        break

                    yield self.env.timeout(self.loop_timeout)

            elif self.mode == 3:
                # robot go to local_storage_node or cold_storage_node from picker_node
                if self.use_local_storage:
                    # go to local_storage_node from picker_node
                    self.loginfo("  %5.1f: %s going to %s" %
                                 (self.env.now, self.robot_id, self.assigned_local_storage_node))
                    yield self.env.process(self.goto(self.assigned_local_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("@ %5.1f: %s reached %s" %
                                 (self.env.now, self.robot_id, self.assigned_local_storage_node))

                else:
                    # go to cold_storage_node from picker_node
                    self.loginfo("  %5.1f: %s going to cold_storage_node: %s" % (self.env.now, self.robot_id, self.cold_storage_node))
                    yield self.env.process(self.goto(self.cold_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("@ %5.1f: %s reached cold_storage_node: %s" % (self.env.now, self.robot_id, self.cold_storage_node))

                # change mode to unloading
                self.mode = 4
                unloading_start_time = self.env.now

            elif self.mode == 4:
                # wait for unloading
                self.loginfo("  %5.1f: %s is waiting for the trays to be unloaded" % (self.env.now, self.robot_id))
                local_storage = self.assigned_local_storage_node  # backup needed for mode 6 if going that way
                yield self.env.process(self.wait_for_unloading())  # this reset mode to 0
                self.time_spent_unloading += self.env.now - unloading_start_time
                self.loginfo("  %5.1f: trays are unloaded from %s" % (self.env.now, self.robot_id))
                # self.release_robot()  # release the robot, so the next picker could use this robot
                self.set_robot_state("INIT")
                if self.use_local_storage:
                    # change mode to idle
                    local_storage = None  # no need of local storage, reset
                    self.mode = 0
                    idle_start_time = self.env.now
                else:
                    # picking_finished is not considered here to make sure no picker is stranded
                    # change to transportation back to local storage of the picker's assigned row
                    self.mode = 6
                    transportation_start_time = self.env.now

            elif self.mode == 5:
                yield self.env.process(self.charging_process())
                self.time_spent_charging += self.env.now - charging_start_time
                # charging complete - now change mode to 0
                self.mode = 0
                idle_start_time = self.env.now

            elif self.mode == 6:
                # go to local_storage_node of the picker's assigned row
                self.loginfo("  %5.1f: %s going to local_storage: %s" % (self.env.now, self.robot_id, local_storage))
                yield self.env.process(self.goto(local_storage))
                self.time_spent_transportation += self.env.now - transportation_start_time
                self.loginfo("@ %5.1f: %s reached local_storage: %s" % (self.env.now, self.robot_id, local_storage))
                local_storage = None

                # change mode to idle
                self.mode = 0
                idle_start_time = self.env.now

            yield self.env.timeout(self.loop_timeout)
        yield self.env.timeout(self.process_timeout)

    def wait_for_loading(self):
        """wait until picker loads trays and confirms it"""
        while True:
            # wait until picker calls loading_complete
            if self.loaded:
                break
            else:
                # TODO: battery decay
                yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)

    def is_in_single_track_route(self, target):
        """ check if the target node is in the single_track_route"""
        if target in self.graph.single_track_route:
            for node in self.graph.single_track_route:
                if self.graph._node_res[node].level is not 0:
                    return True
        else:
            return False

    def _goto(self, target):
        route_nodes = self.get_route_nodes(self.curr_node, target)
        interrupted = False
        idx = 0
        while idx < len(route_nodes):
            n = route_nodes[idx]
            route_dist_cost = self.graph.route_dist_cost([self.curr_node] + route_nodes[idx:])
            d_cur = self.graph.distance(self.curr_node, n)
            time_to_travel = round(d_cur / self.transportation_rate, 1)
            if idx + 1 < len(route_nodes):
                # estimate next distance cost and travelling time
                d_next = self.graph.distance(n, route_nodes[idx + 1])
                time_to_travel_next = round(d_next / (2 * self.transportation_rate), 1)
            else:
                time_to_travel_next = 0

            # the time that node will hold the robot
            # if the node is the cold_storage_node, the hold time needs to consider robot unloading trays(unloading_time)
            if n == self.cold_storage_node:
                hold_time = time_to_travel + time_to_travel_next + self.unloading_time * self.assigned_picker_n_trays
            else:
                hold_time = time_to_travel + time_to_travel_next

            print('  %5.1f:  %s traversing route from node %10s to node %10s '
                  '(distance: %f, travel time: %d, plan hold time: %d)' %
                  (self.graph.env.now, self.robot_id, self.curr_node, n, d_cur, time_to_travel, hold_time))

            try:
                # The node to be requested may be occupied by other robots, mark the time when requesting
                start_wait = self.env.now
                self.graph.set_hold_time(n, hold_time)
                node_state = self.graph.get_node_state(n)
                if node_state is 1:
                    yield self.graph.request_node(self.robot_id, n)
                    print('  %5.1f: active nodes connected to robots: %s ' % (self.env.now, self.graph.active_nodes))
                else:
                    print('  %5.1f: %s: %s is occupied, node state: %d' % (self.env.now, self.robot_id, n, node_state))
                    completed_nodes = self.graph.get_com_nodes()
                    avoid_nodes = [n] + completed_nodes
                    avoid_nodes = list(set(avoid_nodes))  # ignore the duplicated nodes
                    new_route = self.get_route_nodes(self.curr_node, target, avoid_nodes)  # avoid node n

                    # there is no new route available and all the other robots stop at their completed nodes
                    if (new_route is None) and (len(completed_nodes) == len(self.graph.active_nodes) - 1):
                        print('? %5.1f: %s: route %s blocked by other completed nodes - %s' % (
                            self.env.now, self.robot_id, route_nodes, completed_nodes
                        ))
                        interrupted = True
                        break
                    elif new_route:
                        new_route_dc = self.graph.route_dist_cost([self.curr_node] + new_route)  # dc: distance cost
                    elif not new_route:
                        new_route_dc = 10000  # no new route, return a big distance cost
                    wait_time = self.graph.get_wait_time(n)
                    time_cost = self.time_to_dist_cost(wait_time)
                    old_route_cost = route_dist_cost + time_cost
                    print('$ %5.1f: old_route_cost = %d, new_route_dc = %d' % (
                        self.env.now, old_route_cost, new_route_dc))
                    if old_route_cost > new_route_dc:
                        route_nodes = new_route
                        self.graph.cancel_hold_time(n, hold_time)  # found cheap route, cancel the hold_time just be set
                        idx = 0
                        print('~ %5.1f: %s go NEW route: %s' % (self.env.now, self.robot_id, route_nodes))
                        continue
                    else:
                        print('* %5.1f: %s wait %d, use old route %s' % (
                            self.env.now, self.robot_id, wait_time, [self.curr_node] + route_nodes[idx:]))
                        yield self.graph.request_node(self.robot_id, n)
                        for j in self.graph.jam:
                            for r in j:
                                if r == self.robot_id:
                                    print('| %5.1f: %s is in traffic jam, change route now' % (self.env.now, self.robot_id))
                                    route_nodes = new_route
                                    self.graph.cancel_hold_time(n, hold_time)  # cancel the hold_time just set
                                    idx = 0
                                    print('^ %5.1f: %s go NEW route: %s' % (self.env.now, self.robot_id, route_nodes))
                                    break
                        if idx == 0:
                            self.log_cost(self.robot_id, 0, 0, 'CHANGE ROUTE')  # for monitor
                            continue  # change to new route
                        else:
                            pass  # travel to the requested node
                if self.env.now - start_wait > 0:  # The time that the robot has waited since requesting
                    print('$ %5.1f:  %s has lock on %s after %d' % (
                        self.env.now, self.robot_id, n,
                        self.env.now - start_wait))
            except Exception:
                # Not triggered when requesting an occupied node!
                # Not triggered when the robot has a goal running and be assigned a new goal
                print('  %5.1f: %s INTERRUPTED while waiting to gain access to go from node %s going to node %s' % (
                    self.graph.env.now, self.robot_id,
                    self.curr_node, n
                ))
                interrupted = True
                break

            try:
                time_to_travel = round(d_cur / (2 * self.transportation_rate), 1)
                yield self.graph.env.timeout(time_to_travel)
                yield self.graph.release_node(self.robot_id, self.curr_node)
                # The robot is reaching at the half way between the current node and next node
                print('  %5.1f:  %s ---> %s reaching half way ---> %s' % (
                    self.graph.env.now, self.curr_node, self.robot_id, n))
                self.curr_node = n
                remain_time_to_travel = round(d_cur / self.transportation_rate, 1) - time_to_travel
                yield self.graph.env.timeout(remain_time_to_travel)

                self.graph.agent_nodes[self.robot_id] = self.curr_node
                print('@ %5.1f:  %s reached node %s' % (self.graph.env.now, self.robot_id, n))
                self.log_cost(self.robot_id, self.env.now - start_wait, d_cur, n)  # for monitor
                yield self.graph.env.timeout(0)
            except simpy.Interrupt:  # When the robot has a running goal but being assigned a new goal
                print('  %5.1f: %s INTERRUPTED while travelling from node %s going to node %s' % (
                    self.graph.env.now, self.robot_id,
                    self.curr_node, n
                ))
                self.log_cost(self.robot_id, 0, 0, 'INTERRUPTED')  # for monitor
                print('  %5.1f: @@@ %s release previously acquired target node %s' % (self.env.now, self.robot_id, n))
                yield self.graph.release_node(self.robot_id, n)
                interrupted = True
                break
            idx = idx + 1  # go to next while loop

        if interrupted:
            # When the robot has a goal running and be assigned a new goal node
            print('  %5.1f: %s ABORTED at %s' % (self.graph.env.now, self.robot_id, self.curr_node))
            self.log_cost(self.robot_id, 0, 0, 'ABORTED')  # for monitor
        else:
            print('. %5.1f: %s COMPLETED at %s' % (self.graph.env.now, self.robot_id, self.curr_node))
            self.log_cost(self.robot_id, 0, 0, 'COMPLETED')  # for monitor
            self.graph.add_com_node(self.curr_node)

    def goto(self, target):
        """
        Start the _goto(target) process
        :param target: string, topological node that to be reached
        """
        # before go to the target, if the target node is in the single_track_route, and the single_track_route is
        # occupied by a robot, then go back to home node and keep waiting until the single_track_route is free to use
        while self.is_in_single_track_route(target):
            for home_node in self.graph.local_storages.keys():
                # if the home node(local_storages) is not full, go to home node, or stay at current node
                if self.graph._node_res[home_node].level < self.graph._node_res[home_node].capacity:
                    self._active_process = self.graph.env.process(self._goto(home_node))
                    yield self._active_process
                else:
                    continue
                    # self._active_process = self.graph.env.process(self._goto(self.curr_node))
            yield self.env.timeout(self.loop_timeout)

        # the single_track_route is free or the target is not in single_track_route, then go to the target
        self._active_process = self.graph.env.process(self._goto(target))
        yield self._active_process

        # if self.graph.env:
        #     if self._active_process:
        #         if self._active_process.is_alive:  # The robot has an alive goal running
        #             self._active_process.interrupt()
        #             self._active_process = None  # Clear the old goal
        #     self._active_process = self.graph.env.process(self._goto(target))
        # else:
        #     self.curr_node = target

    def get_route_nodes(self, cur_node, target, avoid_nodes=None):
        """
        get the topological route nodes from current node to target node, if request_node failed,
        then get the route nodes from current node to target node and avoid the failed node.
        :param cur_node: string, the current node traversed
        :param target: string, the target node that the robot will go to
        :param avoid_nodes: string list, the nodes that won't be considered when planning route
        return: string list, route node names, from current node to target node
        """
        if self.graph.env:
            if target == cur_node:
                print('  %5.1f: %s is already at target %s' % (self.graph.env.now, self.robot_id, target))
                return [target]
            else:
                if avoid_nodes is None:
                    route = self.graph.get_route(cur_node, target)
                else:
                    route = self.graph.get_avoiding_route(cur_node, target, avoid_nodes)

                if route is None:
                    return None
                else:
                    r = route.source[1:]
                    r.append(target)
                    print('  %5.1f: %s going from node %s to node %s via route %s' % (
                        self.graph.env.now, self.robot_id, cur_node, target, r))
                    return r

    def time_to_dist_cost(self, waiting_time):
        """
        :param waiting_time: the time robot will be waiting
        return: float, the distance cost
        """
        return self.transportation_rate * waiting_time

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
        self._cost['time_now'].append(self.env.now)
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

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            print(msg)

    @property
    def cost(self):
        return self._cost
