#!/usr/bin/env python

import simpy
from math import ceil
from topological_simpy.robot import Robot
from numpy import isclose
import random
from copy import deepcopy


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

        self.robot_state = "INIT"  # "INIT", "ACCEPTED", "ARRIVED", "LOADED"
        # self.robot_resource = simpy.Container(self.env, capacity=1, init=0)  # robot can only be occupied by 1 picker

        self.time_spent_requesting = .0
        self.time_spent_base = .0

        if self.graph.env:
            self.graph.req_ret[self.curr_node] = 1
            self.graph.set_hold_time(self.curr_node, self.init_hold_t)
            self.graph.active_nodes[self.robot_id] = []
            self.graph.request_node(self.robot_id, self.curr_node)
            self.log_cost(self.robot_id, 0, 0, self.curr_node)

        self.action = self.env.process(self.normal_operation())

    def set_robot_state(self, state):
        """
        robot state:
                    "INIT",  initial state
                    "ACCEPTED", robot accepted picker's call, i.e., robot had been assigned to a picker
                    "LOADED", picker loaded trays on robot
                    "ARRIVED", robot arrived at the goal node
        """
        self.robot_state = state

    def trays_loaded(self):
        """picker calls this to indicate the trays are loaded"""
        self.n_empty_trays -= self.assigned_picker_n_trays
        self.n_full_trays += self.assigned_picker_n_trays
        self.loaded = True
        self.set_robot_state("LOADED")

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
                    self.loginfo("A %5.1f: %s is assigned to %s" %
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
                yield self.env.process(self.goto_scheduler(self.assigned_picker_node))
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
                    yield self.env.process(self.goto_scheduler(self.assigned_local_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("@ %5.1f: %s reached %s" %
                                 (self.env.now, self.robot_id, self.assigned_local_storage_node))

                else:
                    # go to cold_storage_node from picker_node
                    self.loginfo("  %5.1f: %s going to cold_storage_node: %s" % (self.env.now, self.robot_id, self.cold_storage_node))
                    yield self.env.process(self.goto_scheduler(self.cold_storage_node))
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
                # # go to local_storage_node of the picker's assigned row
                # self.loginfo("  %5.1f: %s going to local_storage: %s" % (self.env.now, self.robot_id, local_storage))
                # yield self.env.process(self.goto(local_storage))
                # self.time_spent_transportation += self.env.now - transportation_start_time
                # self.loginfo("@ %5.1f: %s reached local_storage: %s" % (self.env.now, self.robot_id, local_storage))
                # local_storage = None

                # go to base_storage node of the robot, in this case, abandon local_storage prepared in previous modes
                self.loginfo("  %5.1f: %s going to base_storage: %s" %
                             (self.env.now, self.robot_id, self.graph.base_stations[self.robot_id]))
                yield self.env.process(self.goto_scheduler(self.graph.base_stations[self.robot_id]))
                self.time_spent_transportation += self.env.now - transportation_start_time
                self.loginfo("@ %5.1f: %s reached base station: %s" %
                             (self.env.now, self.robot_id, self.graph.base_stations[self.robot_id]))

                # change mode to idle
                self.mode = 0
                idle_start_time = self.env.now

                # remove the robot from the usage queue, so other robots know cold_storage_node is free to use
                self.graph.remove_cold_storage_usage_queue(self.robot_id)

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
        """
        robot goes from current node to target node.
        The robot always chooses minimum-distance-cost route(original_route) and travels from current node to target.
        If robot finds the next node is occupied during travelling, the robot will try to find a new_route avoiding the
        occupied node. But at the same time, the robot also evaluates the waiting time cost if keeps
        using the original_route. The robot always chooses low cost route between new_route and original_route.

        :param target: string, target node
        """
        route_nodes = self.get_route_nodes(self.curr_node, target)
        interrupted = False
        idx = 0
        change_route = False

        if len(route_nodes) == 0:
            print("len(route_nodes)=0, no route from %s to %s" % (self.curr_node, target))
            interrupted = True

        while idx < len(route_nodes):
            n = route_nodes[idx]
            route_dist_cost = self.graph.route_dist_cost([self.curr_node] + route_nodes[idx:])
            d_cur = self.graph.distance(self.curr_node, n)
            time_to_travel = round(d_cur / self.transportation_rate + random.gauss(0, self.transportation_rate_std), 1)
            if idx + 1 < len(route_nodes):
                # estimate next distance cost and travelling time
                d_next = self.graph.distance(n, route_nodes[idx + 1])
                time_to_travel_next = round(d_next / (2 * (self.transportation_rate +
                                                           random.gauss(0, self.transportation_rate_std))), 1)
            else:
                time_to_travel_next = 0

            # the time that node will hold the robot
            # if the node is the cold_storage_node, the hold time needs to consider robot unloading trays(unloading_time)
            # TODO: the agent's rotating time should be considered
            if n == self.cold_storage_node:
                hold_time = time_to_travel + time_to_travel_next + self.unloading_time * self.assigned_picker_n_trays
            else:
                hold_time = time_to_travel + time_to_travel_next

            # self.loginfo('  %5.1f:  %s traversing route from node %s to node %s '
            #              '(distance: %f, travel time: %5.1f, plan hold time: %5.1f)' %
            #              (self.graph.env.now, self.robot_id, self.curr_node, n, d_cur, time_to_travel, hold_time))

            try:
                # The node to be requested may be occupied by other robots, mark the time when requesting
                start_wait = self.env.now
                self.graph.set_hold_time(n, hold_time)
                node_state = self.graph.get_node_state(n)
                if node_state is 1:
                    yield self.graph.request_node(self.robot_id, n)
                    # self.loginfo('  %5.1f: active nodes connected to robots: %s ' %
                    #              (self.env.now, self.graph.active_nodes))
                else:
                    # self.loginfo('  %5.1f: %s: %s is occupied, node state: %d' %
                    #              (self.env.now, self.robot_id, n, node_state))
                    avoid_nodes = [n] + self.graph.get_active_nodes([self.robot_id])
                    avoid_nodes = list(dict.fromkeys(avoid_nodes))
                    new_route = self.get_route_nodes(self.curr_node, target, avoid_nodes)  # avoid node n
                    if new_route is None:
                        new_route_dc = float("inf")
                    elif new_route is []:    # robot is at target now
                        new_route_dc = 0
                    else:
                        new_route_dc = self.graph.route_dist_cost([self.curr_node] + new_route)
                    wait_time = round(self.graph.get_wait_time(n), 1)
                    time_cost = self.time_to_dist_cost(wait_time)
                    old_route_cost = route_dist_cost + time_cost
                    # self.loginfo('$ %5.1f: old_route_cost = %d, new_route_dc = %d' % (
                    #     self.env.now, old_route_cost, new_route_dc))
                    if old_route_cost > new_route_dc:
                        route_nodes = new_route
                        self.graph.cancel_hold_time(n, hold_time)  # found cheap route, cancel the hold_time just be set
                        idx = 0
                        self.loginfo('~ %5.1f: %s go NEW route: %s' % (self.env.now, self.robot_id, route_nodes))
                        continue
                    else:
                        route_nodes_to_go = [self.curr_node] + route_nodes[idx:]
                        self.loginfo('* %5.1f: %s wait %5.1f, use old route %s' % (
                            self.env.now, self.robot_id, wait_time, route_nodes_to_go))
                        yield self.graph.request_node(self.robot_id, n)
                        for robot_id in self.graph.deadlocks['deadlock_robot_ids']:
                            if robot_id == self.robot_id:
                                self.loginfo('| %5.1f: %s is in deadlock, change route now' %
                                             (self.env.now, self.robot_id))
                                # TODO: 1. find avoid route: new_route = self.get_route_nodes(
                                #                                           self.curr_node, target, avoid_nodes),
                                #          if new_route is not None, go for new_route; if new_route is None, then?
                                if new_route is not None:     # TODO: is it possible?
                                    route_nodes = new_route
                                else:
                                    avoid_nodes = self.graph.get_active_nodes([self.robot_id])
                                    curr_node_edges = self.graph.get_node_edges(self.curr_node)
                                    # get all the edge nodes that could be used for dodging
                                    for node in avoid_nodes:
                                        if node in curr_node_edges:
                                            curr_node_edges.remove(node)
                                    # find the node to dodge:  node_to_dodge
                                    if n in avoid_nodes:
                                        avoid_nodes.remove(n)
                                        route_nodes_to_dodge = self.graph.get_route_between_adjacent_nodes(
                                            self.curr_node, n, avoid_nodes)
                                        if len(route_nodes_to_dodge) != 0:
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
                                                self.robot_id, self.curr_node, curr_node_edges)
                                            raise Exception(msg)
                                            # route_nodes = None
                                            # yield self.env.timeout(self.loop_timeout)
                                    else:
                                        # todo: impossible, remove the if-else later
                                        msg = "%s: %s not in avoid_nodes!" % (self.robot_id, n)
                                        raise Exception(msg)

                                # prepare to change to the new route_nodes
                                self.graph.cancel_hold_time(n, hold_time)  # cancel the hold_time just set
                                idx = 0
                                change_route = True
                                self.loginfo('^ %5.1f: %s go NEW route: %s' %
                                             (self.env.now, self.robot_id, route_nodes))
                                break
                        if change_route:
                            self.log_cost(self.robot_id, 0, 0, 'CHANGE ROUTE')  # for monitor
                            self.graph.remove_robot_from_deadlock(self.robot_id)
                            # TODO: How to ensure that the deadlock will be resolved? --> the dodge route is promised to
                            #       be working(unless curr_node_edges is None, i.e., deadlock in single track),
                            #       so the deadlock must be resolved at present.
                            continue  # change to new route
                        else:
                            pass  # travel to the requested node
                if self.env.now - start_wait > 0:  # The time that the robot has waited since requesting
                    self.time_spent_requesting += self.env.now - start_wait
                    self.loginfo('$ %5.1f:  %s has lock on %s after %5.1f' % (
                        self.env.now, self.robot_id, n,
                        self.env.now - start_wait))
            except Exception:
                # Not triggered when requesting an occupied node!
                # Not triggered when the robot has a goal running and be assigned a new goal
                self.loginfo('  %5.1f: %s INTERRUPTED while waiting to gain access to go from node %s going to node %s'
                             % (self.graph.env.now, self.robot_id, self.curr_node, n))
                interrupted = True
                break

            try:
                time_to_travel_before_release = round(d_cur / (2 * (self.transportation_rate +
                                                                    random.gauss(0, self.transportation_rate_std))), 1)
                yield self.graph.env.timeout(time_to_travel_before_release)
                yield self.graph.release_node(self.robot_id, self.curr_node)
                # The robot is reaching at the half way between the current node and next node, release current node
                # self.loginfo('  %5.1f:  %s ---> %s reaching half way ---> %s, releasing %s' % (
                #     self.graph.env.now, self.curr_node, self.robot_id, n, self.curr_node))
                self.curr_node = n
                self.graph.agent_nodes[self.robot_id] = self.curr_node

                remain_time_to_travel = time_to_travel - time_to_travel_before_release
                yield self.graph.env.timeout(remain_time_to_travel)
                self.loginfo('@ %5.1f:  %s reached node %s' % (self.graph.env.now, self.robot_id, n))
                self.log_cost(self.robot_id, self.env.now - start_wait, d_cur, n)  # for monitor
                yield self.graph.env.timeout(0)
            except simpy.Interrupt:  # When the robot has a running goal but being assigned a new goal
                self.loginfo('  %5.1f: %s INTERRUPTED while travelling from node %s going to node %s' % (
                    self.graph.env.now, self.robot_id,
                    self.curr_node, n
                ))
                self.log_cost(self.robot_id, 0, 0, 'INTERRUPTED')  # for monitor
                self.loginfo('  %5.1f: @@@ %s release previously acquired target node %s' %
                             (self.env.now, self.robot_id, n))
                yield self.graph.release_node(self.robot_id, n)
                interrupted = True
                break
            idx = idx + 1  # go to next while loop

        if interrupted:
            # When the robot has a goal running and be assigned a new goal node
            self.loginfo('  %5.1f: %s ABORTED at %s' % (self.graph.env.now, self.robot_id, self.curr_node))
            self.log_cost(self.robot_id, 0, 0, 'ABORTED')  # for monitor
        else:
            self.loginfo('. %5.1f: %s COMPLETED at %s' % (self.graph.env.now, self.robot_id, self.curr_node))
            self.log_cost(self.robot_id, 0, 0, 'COMPLETED')  # for monitor
            self.graph.add_com_node(self.curr_node)

    def goto_scheduler(self, target):
        """
        Scheduler of assigning target node to the robot based on whether the target is the cold_storage_node and
        whether the queue of waiting for using cold storage node is free (len(cold_storage_usage_queue) == 0). if
        the target is not cold_storage_node or the queue is free, then go to the target directly. Or, join the queue
        and keep waiting until the previous robots have finished using the cold_storage_node before going to the
        target(cold_storage_node)

        :param target: string, topological node that to be reached
        """

        # before go to the target, if the target node is not cold_storage_node, go to the target directly
        if target != self.cold_storage_node:
            yield self.graph.env.process(self._goto(target))
        else:
            try:
                assert self.mode == 3  # robot go to local_storage_node or cold_storage_node from picker_node
            except AssertionError:
                raise Exception("goto_scheduler is trying to assign %s going to %s, but robot is in %d" %
                                (self.robot_id, target, self.mode))

            # if target is the cold_storage_node, but no robot planning to use cold_storage_node now, then go to use
            if len(self.graph.cold_storage_usage_queue) == 0:
                self.graph.add_cold_storage_usage_queue(self.robot_id)
                yield self.env.process(self._goto(target))
            # if any robot is using cold_storage_node, go to base station and wait until the queue is free
            else:
                # robot join waiting queue for using cold storage node
                self.graph.add_cold_storage_usage_queue(self.robot_id)
                # if robot is not at base station, then go to the base station
                if self.curr_node != self.graph.base_stations[self.robot_id]:
                    yield self.env.process(self._goto(self.graph.base_stations[self.robot_id]))

                # keep waiting at the base station until previous robots have finished usage
                start_time = self.env.now
                while self.graph.get_cold_storage_usage_queue_head() != self.robot_id:
                    yield self.env.timeout(self.loop_timeout)
                self.time_spent_base += self.env.now - start_time
                wait_time = self.env.now - start_time
                self.loginfo("F %5.1f: cold_storage_node free, %s going to target node: %s, wait time: %5.1f" %
                             (self.env.now, self.robot_id, target, wait_time))
                yield self.env.process(self._goto(target))

    def get_route_nodes(self, cur_node, target, avoid_nodes=None):
        """
        get the topological route nodes from current node to target node, if request_node failed,
        then get the route nodes from current node to target node and avoid the failed node.
        :param cur_node: string, the current node traversed
        :param target: string, the target node that the robot will go to
        :param avoid_nodes: string list, the nodes that won't be considered when planning route
        return: string list, route node names, from current node to target node
        """
        avo_nodes = deepcopy(avoid_nodes)
        if self.graph.env:
            if target == cur_node:
                print('  %5.1f: %s is already at target %s' % (self.graph.env.now, self.robot_id, target))
                return []
            else:
                if avo_nodes is None:
                    route = self.graph.get_route(cur_node, target)
                elif target in avo_nodes:
                    # target is occupied at present, but maybe available later, so get the route anyway
                    avo_nodes.remove(target)
                    route = self.graph.get_avoiding_route(cur_node, target, avo_nodes)
                    # It will lead to deadlocking if the target is still occupied when the agent is next to
                    # the target
                    if len(route.source) == 1:  # in deadlock
                        return None
                else:  # target not in avo_nodes
                    route = self.graph.get_avoiding_route(cur_node, target, avo_nodes)

                if len(route.source) == 0:
                    return None
                else:
                    r = route.source[1:]
                    r.append(target)
                    # print('  %5.1f: %s going from node %s to node %s via route %s' % (
                    #     self.graph.env.now, self.robot_id, cur_node, target, r))
                    return r

    def time_to_dist_cost(self, waiting_time):
        """
        :param waiting_time: the time robot will be waiting
        return: float, the distance cost
        """
        return (self.transportation_rate + random.gauss(0, self.transportation_rate_std)) * waiting_time

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

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            print(msg)

    @property
    def cost(self):
        return self._cost
