#!/usr/bin/env python

import simpy
from math import ceil
from topological_simpy.robot import Robot
from numpy import isclose


# noinspection PyBroadException
class RobotSim(Robot):
    def __init__(self, name, transportation_rate, max_n_trays, unloading_time, env, topo_graph, verbose):

        super(RobotSim, self).__init__(name, transportation_rate, max_n_trays, unloading_time, env, topo_graph, verbose)

        self._current_node = self.curr_node  # TODO: merge the two names
        self._tmap = topo_graph
        self._env = topo_graph.env  # TODO: to be fixed
        self._name = name
        self._speed_m_s = .3
        self._active_process = None
        self.init_hold_t = 5  # initial hold time, for first node TODO: assign a dynamic time--> reset when picker calls
        self._cost = {'robot_name': [],
                      'time_now': [],
                      'time_wait': [],
                      'dist_cost': [],
                      'route': [],
                      'dist_cost_total': [],
                      'time_wait_total': [],
                      'time_cost_total': [],
                      'cost_total': []}

        self.robot_state = "INIT"  # robot state, "INIT", "ACCEPTED", "TRAVELLING", "ARRIVED", "LOADED_ON", "LOADED_OFF"

        if self._tmap.env:
            self._tmap.req_ret[self._current_node] = 1
            self._tmap.set_hold_time(self._current_node, self.init_hold_t)
            self._tmap.active_nodes[self._name] = []
            self._tmap.request_node(self._name, self._current_node)
            self.log_cost(self._name, 0, 0, self._current_node)

        self.action = self.env.process(self.normal_operation())

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
                self.loginfo("X %5.1f: all rows picked. %s exiting" % (self._env.now, self.robot_id))
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
                                 (self._env.now, self.robot_id, self.assigned_picker_id))

                # TODO: idle state battery charge changes
                if self.battery_charge < 40.0 or isclose(self.battery_charge, 40.0):
                    self.loginfo("  %5.1f: battery low on %s, going to charging mode" % (self._env.now, self.robot_id))
                    self.time_spent_idle += self.env.now - idle_start_time
                    # change mode to charging
                    self.mode = 5
                    charging_start_time = self.env.now

            elif self.mode == 1:
                # farm assigns the robot to a picker by calling assign_robot_to_picker
                # go to picker_node from curr_node
                self.loginfo("  %5.1f: %s going to %s" % (self._env.now, self.robot_id, self.assigned_picker_node))
                yield self.env.process(self.go_to_node(self.assigned_picker_node))
                self.time_spent_transportation += self.env.now - transportation_start_time
                self.loginfo("@ %5.1f: %s reached %s" % (self._env.now, self.robot_id, self.assigned_picker_node))
                self.set_robot_state("ARRIVED")
                # change mode to waiting_for_loading
                self.continue_transporting = False
                self.mode = 2
                loading_start_time = self.env.now

            elif self.mode == 2:
                self.loginfo("  %5.1f: %s is waiting for the trays to be loaded" % (self._env.now, self.robot_id))
                yield self.env.process(self.wait_for_loading())  # this reset mode to 3
                self.time_spent_loading += self.env.now - loading_start_time
                self.loginfo("L %5.1f: trays are loaded on %s" % (self._env.now, self.robot_id))
                self.set_robot_state("LOADED")  # TODO: LOADED_ON
                while True:
                    if self.continue_transporting:
                        # change mode to transporting to storage
                        self.mode = 3
                        transportation_start_time = self.env.now
                        break

                    yield self.env.timeout(self.loop_timeout)

            elif self.mode == 3:
                if self.use_local_storage:
                    # go to local_storage_node from picker_node
                    self.loginfo("  %5.1f: %s going to %s" %
                                 (self._env.now, self.robot_id, self.assigned_local_storage_node))
                    yield self.env.process(self.go_to_node(self.assigned_local_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("@ %5.1f: %s reached %s" %
                                 (self._env.now, self.robot_id, self.assigned_local_storage_node))

                else:
                    # go to cold_storage_node from picker_node
                    self.loginfo("  %5.1f: %s going to %s" % (self._env.now, self.robot_id, self.cold_storage_node))
                    yield self.env.process(self.go_to_node(self.cold_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("@ %5.1f: %s reached %s" % (self._env.now, self.robot_id, self.cold_storage_node))

                # change mode to unloading
                self.mode = 4
                unloading_start_time = self.env.now

            elif self.mode == 4:
                # wait for unloading
                self.loginfo("  %5.1f: %s is waiting for the trays to be unloaded" % (self._env.now, self.robot_id))
                local_storage = self.assigned_local_storage_node  # backup needed for mode 6 if going that way
                yield self.env.process(self.wait_for_unloading())  # this reset mode to 0
                self.time_spent_unloading += self.env.now - unloading_start_time
                self.loginfo("  %5.1f: trays are unloaded from %s" % (self._env.now, self.robot_id))
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
                self.loginfo("  %5.1f: %s going to %s" % (self._env.now, self.robot_id, local_storage))
                yield self.env.process(self.go_to_node(local_storage))
                self.time_spent_transportation += self.env.now - transportation_start_time
                self.loginfo("@ %5.1f: %s reached %s" % (self._env.now, self.robot_id, local_storage))
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

    def _goto(self, target):
        route_nodes = self.get_route_nodes(self._current_node, target)
        interrupted = False
        idx = 0
        while idx < len(route_nodes):
            n = route_nodes[idx]
            route_dist_cost = self._tmap.route_dist_cost([self._current_node] + route_nodes[idx:])
            d_cur = self._tmap.distance(self._current_node, n)
            time_to_travel = int(ceil(d_cur / self._speed_m_s))
            if idx + 1 < len(route_nodes):
                # estimate next distance cost and travelling time
                d_next = self._tmap.distance(n, route_nodes[idx + 1])
                time_to_travel_next = ceil(d_next / (2 * self._speed_m_s))
            else:
                time_to_travel_next = 0
            hold_time = time_to_travel + time_to_travel_next  # the time node will hold the robot
            print(
                    '% 5d:  %s traversing route from node %10s to node %10s (distance: %f, travel time: %d, hold time: %d)'
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
                        new_route_dc = 10000  # no new route, return a big distance cost
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
                                    self._tmap.cancel_hold_time(n, hold_time)  # cancel the hold_time just set
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
        return self._speed_m_s * waiting_time

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

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            print(msg)

    @property
    def cost(self):
        return self._cost
