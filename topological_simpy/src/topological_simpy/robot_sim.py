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

        super(RobotSim, self).__init__(robot_id, transportation_rate, max_n_trays, unloading_time, env, topo_graph,
                                       verbose)

        self._active_process = None
        self.init_hold_t = float("inf")  # 5, initial hold time, for first node TODO: assign a dynamic time--> reset when picker calls
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
        self.t_mode = None  # robot transportation mode

        self.interrupted = False
        self._goto_process = None
        
        self.time_spent_requesting = .0
        self.time_spent_base = .0

        # self.dist_cost_to_target = {self.robot_id: {'curr_node': None,
        #                                             'target_node': None,
        #                                             'dist_cost': .0}}

        # Initialise robot starting position
        self.graph.init_agent_nodes(self.robot_id,
                                    self.transportation_rate,
                                    self.transportation_rate_std,
                                    self.unloading_time,
                                    self.assigned_picker_n_trays)

        # Robot requesting starting node
        if self.graph.env:
            self.graph.req_ret[self.graph.curr_node[self.robot_id]] = 1
            self.graph.set_hold_time(self.graph.curr_node[self.robot_id], self.init_hold_t)
            self.graph.active_nodes[self.robot_id] = []
            self.graph.init_request_node(self.robot_id, self.graph.curr_node[self.robot_id])
            self.log_cost(self.robot_id, 0, 0, self.graph.curr_node[self.robot_id])

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
                yield self.env.process(self.goto_scheduler_mode(self.assigned_picker_node))
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
                    yield self.env.process(self.goto_scheduler_mode(self.assigned_local_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("@ %5.1f: %s reached %s" %
                                 (self.env.now, self.robot_id, self.assigned_local_storage_node))

                else:
                    # go to cold_storage_node from picker_node
                    self.loginfo("  %5.1f: %s going to cold_storage_node: %s" % (
                        self.env.now, self.robot_id, self.cold_storage_node))
                    yield self.env.process(self.goto_scheduler_mode(self.cold_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("@ %5.1f: %s reached cold_storage_node: %s" % (
                        self.env.now, self.robot_id, self.cold_storage_node))

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
                # TODO: [future] optimise the congestion nearby base_stations by parking robot in any node among the
                #       base stations instead of paired node currently used
                self.loginfo("  %5.1f: %s going to base_storage: %s" %
                             (self.env.now, self.robot_id, self.graph.base_stations[self.robot_id]))
                yield self.env.process(self.goto_scheduler_mode(self.graph.base_stations[self.robot_id]))
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

    # deprecated
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

            # get the current node, next node and target node, used for resolving deadlocks
            self.graph.targets_info(self.robot_id, self.curr_node, n, target)

            route_dist_cost = self.graph.route_dist_cost([self.curr_node] + route_nodes[idx:])
            self.graph.update_dist_cost_to_target(self.robot_id, self.curr_node, target, route_dist_cost)

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
                    elif new_route is []:  # robot is at target now
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
                                self.deadlock_dodge(n, new_route, target)
                                # TODO: 1. find avoid route: new_route = self.get_route_nodes(
                                #                                           self.curr_node, target, avoid_nodes),
                                #          if new_route is not None, go for new_route; if new_route is None, then?
                                # if new_route is not None:  # TODO: is it possible?
                                #     route_nodes = new_route
                                # else:
                                #     avoid_nodes = self.graph.get_active_nodes([self.robot_id])
                                #     curr_node_edges = self.graph.get_node_edges(self.curr_node)
                                #     # get all the edge nodes that could be used for dodging
                                #     for node in avoid_nodes:
                                #         if node in curr_node_edges:
                                #             curr_node_edges.remove(node)
                                #     # find the node to dodge:  node_to_dodge
                                #     if n in avoid_nodes:
                                #         avoid_nodes.remove(n)
                                #         route_nodes_to_dodge = self.graph.get_route_between_adjacent_nodes(
                                #             self.curr_node, n, avoid_nodes)
                                #         if route_nodes_to_dodge[0] != n and len(route_nodes_to_dodge) != 0:
                                #             node_to_dodge = route_nodes_to_dodge[0]
                                #             # Without considering avoid_nodes, route_nodes_to_dodge is ensured
                                #             # with a valid route_nodes
                                #             # TODO: is it better to get route_nodes with avoiding avoid_nodes?
                                #             route_nodes_to_dodge = self.get_route_nodes(node_to_dodge, target)
                                #             route_nodes = [node_to_dodge] + route_nodes_to_dodge
                                #         elif len(curr_node_edges) != 0:
                                #             node_to_dodge = curr_node_edges[0]  # TODO: any better selecting criteria?
                                #             route_nodes_to_dodge = self.get_route_nodes(node_to_dodge, target)
                                #             route_nodes = [node_to_dodge] + route_nodes_to_dodge
                                #         else:
                                #             # no edges for dodging, wait for other agents to move
                                #             # TODO: Inform another deadlocked robot to dodge.
                                #             #       note: another robot has locked two nodes
                                #             #       Double check if another deadlocked robot releases the node and
                                #             #       the node then be occupied by a third robot: what the
                                #             #       current robot should do?
                                #             msg = "%s: @%s No edge to dodge, curr_node_edges: %s" % (
                                #                 self.robot_id, self.curr_node, curr_node_edges)
                                #             raise Exception(msg)
                                #             # route_nodes = None
                                #             # yield self.env.timeout(self.loop_timeout)
                                #     else:
                                #         # todo: impossible, remove the if-else later
                                #         msg = "%s: %s not in avoid_nodes!" % (self.robot_id, n)
                                #         raise Exception(msg)

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
                self.log_cost(self.robot_id, 0, 0, 'INTERRUPTED')  # for monitor
                self.loginfo('  %5.1f: @@@ %s release previously acquired target node %s' %
                             (self.env.now, self.robot_id, n))
                yield self.graph.release_node(self.robot_id, n)
                interrupted = True
                # TODO [next]: reset robot status. send robot back to a safe place? Use a new mode to deal with the
                #  exception.
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
            self.interrupted = True
            self.loginfo('C %5.1f: %s continue going to target: %s' % (self.graph.env.now, self.robot_id, target))
            # TODO: [future] interrupted reason is still unclear (one reason is the wait_time < 0, but why it causes
            #  interrupt?)
            self._goto_process = self.env.process(self._goto(target))
            yield self._goto_process
            self.interrupted = False

        else:
            self.graph.update_dist_cost_to_target(self.robot_id, self.curr_node, target, 0)
            self.loginfo('. %5.1f: %s COMPLETED at %s' % (self.graph.env.now, self.robot_id, self.curr_node))
            self.log_cost(self.robot_id, 0, 0, 'COMPLETED')  # for monitor
            self.graph.add_com_node(self.curr_node)

    # deprecated
    def deadlock_dodge(self, n, new_route, target):
        """
        When in deadlock, find a new route.
        If no new route available, then go to an edge node, then find a new route to the target

        :param n: string, topological node, the next node that the robot should go
        :param new_route: string, topological node list, a new topological route from current node to the target
        :param target: string, topological node, the target node that the robot should go

        return: topological node list, a new route to avoid deadlock
        """
        if new_route is not None:  # TODO: is it possible?
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
                        self.robot_id, self.curr_node, curr_node_edges)
                    raise Exception(msg)
                    # route_nodes = None
                    # yield self.env.timeout(self.loop_timeout)
            else:
                # todo: impossible, remove the if-else later
                msg = "%s: %s not in avoid_nodes!" % (self.robot_id, n)
                raise Exception(msg)

        return route_nodes

    # deprecated
    def deadlock_coordinate(self, dead_locks):
        """
        Get all robots' information and considering them as a unity, find out the breakpoint:
            which robot should make way
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
        """

        # get all active nodes into a list:
        active_node_list = []
        for robot_id in self.graph.active_nodes.keys():
            active_node_list = active_node_list + self.graph.active_nodes[robot_id]

        for robot_id in self.graph.targets.keys():
            if self.graph.targets[robot_id]['target'] not in active_node_list:
                self.graph.targets[robot_id]['priority'] = 1000
                if len(self.graph.active_nodes[robot_id]) == 2:
                    # remove robot_id from the put_queue of node self.graph.active_nodes[robot_id][1].
                    # After removing, what about other robots in the waiting list? their actual waiting time will
                    # shortened, no negative impact.
                    # TODO[today]: how to wake up robot from the put_queue? put_queue.pop(indx) could clear the queue,
                    #               but robot_01 is not waking up!
                    pass

    # deprecated
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
                self.graph.add_cold_storage_usage_queue(self.robot_id, self.env.now)
                unloading_time = self.unloading_time * self.assigned_picker_n_trays
                self.graph.get_cold_storage_queue_wait_time(self.robot_id,
                                                            self.curr_node,
                                                            target,
                                                            self.transportation_rate,
                                                            unloading_time,
                                                            True)
                self._goto_process = self.env.process(self._goto(target))
                yield self._goto_process
            # if any robot is using cold_storage_node, go to base station and wait until the queue is free
            else:
                # robot join waiting queue for using cold storage node
                self.graph.add_cold_storage_usage_queue(self.robot_id, self.env.now)
                # if robot is not at base station, then go to the base station
                if self.curr_node != self.graph.base_stations[self.robot_id]:
                    self._goto_process = self.env.process(self._goto(self.graph.base_stations[self.robot_id]))
                    yield self._goto_process
                    self.graph.start_parking_at_base(self.robot_id, self.env.now)

                # keep waiting at the base station until previous robots have finished usage
                # TODO: update hold time here! so the other robots who want to use the occupied base_station knows how 
                #       long they should wait.
                #       1. In cold_storage_usage_queue, estimate each robot's waiting time from the queue head to the 
                #           queue rear. 
                #           For the queue head robot: 
                #               wait_time[0] = 0 (in use)
                #               use_time[0] = from the current moment to the moment when the current using robot return 
                #                             to his base_station
                #           For the 2nd queue head robot: 
                #               wait_time[1] = wait_time[0] + use_time[0]
                #               use_time[1] = round trip travel time to cold_storage + unloading time
                #           For the 3rd queue head robot:
                #               wait_time[2] = wait_time[1] + use_time[1]
                #               use_time[2] = round trip travel time to cold_storage + unloading time
                #           For the n_th queue head robot:
                #               wait_time[n-1] = wait_time[n-2] + use_time[n-2]
                #               use_time[n-1] = round trip travel time to cold_storage + unloading time

                start_time = self.env.now
                # TODO:[future] in the config file, the base(parking) station should be set nearby storage station,
                #       so the queue for using storage station is still useful
                unloading_time = self.unloading_time * self.assigned_picker_n_trays
                robot_wait_info = self.graph.get_cold_storage_queue_wait_time(self.robot_id,
                                                                              self.curr_node,
                                                                              target,
                                                                              self.transportation_rate,
                                                                              unloading_time,
                                                                              False)
                hold_time = robot_wait_info['wait_time']
                self.graph.extend_hold_time(self.curr_node, hold_time)

                # wait the cold_storage to be free after hold_time, if still not free, wait in the while loop
                yield self.env.timeout(hold_time)
                while self.robot_id != self.graph.get_cold_storage_usage_queue_head()['robot_id']:
                    yield self.env.timeout(self.loop_timeout)
                self.time_spent_base += self.env.now - start_time
                wait_time = self.env.now - start_time
                self.loginfo("F %5.1f: cold_storage_node free, %s going to target node: %s, wait time: %5.1f" %
                             (self.env.now, self.robot_id, target, wait_time))
                self._goto_process = self.env.process(self._goto(target))
                yield self._goto_process

    def goto_scheduler_mode(self, target):
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
            yield self.env.process(self.graph._goto_container(target,
                                                              self.robot_id))
            # if self.interrupted:
            #     yield self.graph.env.process(self._goto(target))
        else:
            try:
                assert self.mode == 3  # robot go to local_storage_node or cold_storage_node from picker_node
            except AssertionError:
                raise Exception("goto_scheduler is trying to assign %s going to %s, but robot is in %d" %
                                (self.robot_id, target, self.mode))

            # go to storage, robot enters different sub modes based on the storage usage queue
            yield self.env.process(self.go_to_storage_operation(target))

    # deprecated
    def goto_scheduler_mode_park(self, target):
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
            yield self.env.process(self.go_to_storage_operation(target))
            
    def go_to_storage_operation(self, target):
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
        Note: the function could be extend to use for any other target node that in a single track route
        """
        # TODO [future]: to be done...
        #       1. self.time_spent_waiting_storage needs to be recorded

        # if target is the cold_storage_node, but no robot planning to use cold_storage_node now, then go to use
        if len(self.graph.cold_storage_usage_queue) == 0:
            self.t_mode = 30
        else:
            self.t_mode = 31

        while self.t_mode != 4:
            if self.t_mode == 30:
                self.graph.add_cold_storage_usage_queue(self.robot_id, self.env.now)
                # todo: initialise wait queue
                self.init_wait_queue(target)
                self._goto_process = self.env.process(self.graph._goto_container(target,
                                                                                 self.robot_id))
                yield self._goto_process
                self.t_mode = 4

            elif self.t_mode == 31:
                self.graph.add_cold_storage_usage_queue(self.robot_id, self.env.now)
                if self.graph.curr_node[self.robot_id] != self.graph.base_stations[self.robot_id]:
                    self._goto_process = self.env.process(
                        self.graph._goto_container(self.graph.base_stations[self.robot_id],
                                                   self.robot_id))
                    yield self._goto_process
                self.t_mode = 32

            elif self.t_mode == 32:
                try:
                    self.graph.start_parking_at_base(self.robot_id, self.env.now)
                    wait_info = self.get_queue_wait_time(self.robot_id)
                    # if wait_info['wait_time'] < 0:  # TODO: never reaching, to be removed
                    #     print("wait_info['wait_time']: %f" % wait_info['wait_time'])
                    self.graph.extend_hold_time(self.graph.curr_node[self.robot_id],
                                                wait_info['wait_time'])  # todo: inform other robots who are waiting for this node
                    # yield self.env.timeout(wait_info['wait_time'])
                    while self.robot_id not in self.graph.get_cold_storage_usage_queue_head()['robot_id']:
                        yield self.env.timeout(self.loop_timeout)
                    self.t_mode = 33
                except Exception:
                    print('parking bug')

            elif self.t_mode == 33:
                # correct the predict wait_time according to real situation
                for idx, queue_robot in enumerate(self.graph.cold_storage_queue_wait_time):
                    if self.robot_id == queue_robot['robot_id']:
                        if 'start_parking_time' in queue_robot.keys():
                            real_wait_time = self.env.now - queue_robot['start_parking_time']
                        else:
                            real_wait_time = .0
                        self.graph.cold_storage_queue_wait_time[idx]['wait_time'] = real_wait_time
                        self.graph.cold_storage_queue_wait_time[idx]['start_using_storage_time'] = self.env.now
                        break
                self._goto_process = self.env.process(self.graph._goto_container(target,
                                                                                 self.robot_id))
                yield self._goto_process
                self. t_mode = 4

    def init_wait_queue(self, target):
        """
        Initialise the wait queue for using (cold) storage station. There is only one robot in the queue and the robot
        is using the storage.
        """
        curr_dist_to_storage = self.graph.get_total_route_distances(self.graph.curr_node[self.robot_id],
                                                                    self.cold_storage_node)
        self.graph.update_dist_cost_to_target(self.robot_id, self.graph.curr_node[self.robot_id],
                                              target, curr_dist_to_storage)
        storage_dist_to_base = self.graph.get_total_route_distances(self.cold_storage_node,
                                                                    self.graph.base_stations[self.robot_id])
        unloading_time = self.unloading_time * self.assigned_picker_n_trays
        use_t = (curr_dist_to_storage + storage_dist_to_base) / self.transportation_rate + unloading_time
        # stop_using_storage_time = self.env.now + use_t
        self.graph.cold_storage_queue_wait_time.append({'robot_id': self.robot_id,
                                                        'start_parking_time': None,
                                                        'start_using_storage_time': self.env.now,
                                                        # 'stop_using_storage_time': stop_using_storage_time,
                                                        'wait_time': .0,
                                                        'use_time': use_t})

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
        if len(self.graph.cold_storage_queue_wait_time) == 0:

            # robot arriving at base and in the head of the usage queue, go to use directly
            robot_id = self.graph.cold_storage_usage_queue[0]['robot_id']
            curr_dist_to_base.append(self.graph.get_total_route_distances(self.graph.agent_nodes[robot_id],
                                                                          self.graph.base_stations[robot_id]))
            curr_dist_to_base_time.append(curr_dist_to_base[0]/self.transportation_rate)
            base_dist_to_cold = self.graph.get_total_route_distances(self.cold_storage_node,
                                                                     self.graph.base_stations[robot_name])
            use_time.append((2*base_dist_to_cold + unloading_time)/self.transportation_rate)
            wait_time.append(curr_dist_to_base_time[0])
            self.graph.cold_storage_queue_wait_time.append({'robot_id': robot_name,
                                                            'wait_time': wait_time[0],
                                                            'use_time': use_time[0]})

            # robot arriving base and not in the head of the usage queue, wait for previous robots
            for i in range(1, len(self.graph.cold_storage_usage_queue)):
                robot_id = self.graph.cold_storage_usage_queue[i]['robot_id']
                curr_dist_to_base.append(self.graph.get_total_route_distances(self.graph.agent_nodes[robot_id],
                                                                              self.graph.base_stations[robot_id]))
                curr_dist_to_base_time.append(curr_dist_to_base[i]/self.transportation_rate)
                base_dist_to_cold = self.graph.get_total_route_distances(self.cold_storage_node,  # todo:[next] initialise in _init_
                                                                         self.graph.base_stations[robot_id])
                use_time.append((2*base_dist_to_cold + unloading_time)/self.transportation_rate)
                # if i == 1:
                #     wait_time.append(use_time[0] + max(wait_time[0], curr_dist_to_base_time[0]))
                # if i == 2:
                #     wait_time.append(use_time[1] + max(wait_time[1], curr_dist_to_base_time[1]))
                # if i > 0:
                wait_time.append(use_time[i-1] + max(wait_time[i-1], curr_dist_to_base_time[i-1]))

                # get the time of start parking at base:
                # if robot started parking, record parking time
                if len(self.graph.cold_storage_usage_queue[i]) == 3:
                    start_parking_time = self.graph.cold_storage_usage_queue[i]['start_parking_time']
                    self.graph.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                                    'start_parking_time': start_parking_time,
                                                                    'wait_time': wait_time[i],
                                                                    'use_time': use_time[i]})
                else:
                    # robot has not parked, ignore
                    self.graph.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                                    'wait_time': wait_time[i],
                                                                    'use_time': use_time[i]})

        else:
            wait_time.append(self.graph.cold_storage_queue_wait_time[0]['wait_time'])
            use_time.append(self.graph.cold_storage_queue_wait_time[0]['use_time'])
            start_using_storage_time = self.graph.cold_storage_queue_wait_time[0]['start_using_storage_time']
            robot_id = self.graph.cold_storage_usage_queue[0]['robot_id']

            # note: robot transportation_rate and unloading time are simply use the same value for all robots as the wait
            #       time prediction won't be so precise and it's not worthy to be precise at such level. Because the robot
            #       may change route during traveling.

            for i in range(1, len(self.graph.cold_storage_usage_queue)):
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
                robot_id = self.graph.cold_storage_usage_queue[i]['robot_id']
                curr_dist_to_base.append(self.graph.get_total_route_distances(self.graph.agent_nodes[robot_id],
                                                                              self.graph.base_stations[robot_id]))
                curr_dist_to_base_time.append(curr_dist_to_base[i-1]/self.transportation_rate)
                base_dist_to_cold = self.graph.get_total_route_distances(self.cold_storage_node,
                                                                         self.graph.base_stations[robot_id])

                use_time.append((2*base_dist_to_cold + unloading_time)/self.transportation_rate)

                # get the time that robot[i] will be waiting
                if i == 1:
                    # get the time of robot[i-1] start using storage
                    # if the robot in the queue head never parked before, start time is the join queue time
                    if len(self.graph.cold_storage_usage_queue[0]) == 2:
                        start_using_storage_time = self.graph.cold_storage_usage_queue[0]['join_queue_time']
                    # if the robot in the queue head has parked in the base before, start time is available in queue wait time
                    elif len(self.graph.cold_storage_usage_queue[0]) == 3:
                        start_using_storage_time = self.graph.cold_storage_queue_wait_time[0]['start_using_storage_time']
                    wait_time.append(abs(use_time[0] + start_using_storage_time - self.env.now))  #  todo: remove abs?
                else:
                    # wait_time.append(max(curr_dist_to_base_time[i-1], wait_time[i-1]) + use_time[i-1])
                    # if robot[i-1] won't start using storage before robot[i] arriving at base
                    if wait_time[i-1] > curr_dist_to_base_time[i-1]:
                        wait_time.append(wait_time[i-1] + use_time[i-1])
                    # if robot[i-1] start using storage, but robot[i] is still on the way to the base
                    else:
                        previous_robot_id = self.graph.cold_storage_usage_queue[i-1]['robot_id']
                        previous_robot_wait_time = .0
                        previous_robot_use_time = .0
                        for robot_info in self.graph.cold_storage_queue_wait_time:
                            if previous_robot_id == robot_info['robot_id']:
                                previous_robot_wait_time = robot_info['wait_time']
                                previous_robot_use_time = robot_info['use_time']
                        # robot[i] takes longer time to get to base, or robot[i-1] takes longer time to finish using storage
                        wait_time.append(max(curr_dist_to_base_time[i-1], previous_robot_wait_time+previous_robot_use_time))

                # remove old wait time before append latest wait time
                for queue_robot in self.graph.cold_storage_queue_wait_time:
                    if robot_id == queue_robot['robot_id']:
                        self.graph.cold_storage_queue_wait_time.remove(queue_robot)

                # get the time of start parking at base:
                # if robot started parking
                if len(self.graph.cold_storage_usage_queue[i]) == 3:
                    start_parking_time = self.graph.cold_storage_usage_queue[i]['start_parking_time']
                    self.graph.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                                    'start_parking_time': start_parking_time,
                                                                    'wait_time': wait_time[i],
                                                                    'use_time': use_time[i]})
                else:
                    # robot has not parked, predict the parking time
                    # start_parking_time = self.env.now + curr_dist_to_base_time[i-1]
                    # robot has not parked, ignore
                    self.graph.cold_storage_queue_wait_time.append({'robot_id': robot_id,
                                                                    'wait_time': wait_time[i],
                                                                    'use_time': use_time[i]})
        # return the queried wait time info
        for queue_robot in self.graph.cold_storage_queue_wait_time:
            if robot_name == queue_robot['robot_id']:
                return queue_robot

    # moved to topo.py
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

    # def update_dist_cost_to_target(self, robot_id, curr_node, target, route_dist_cost):
    #     """
    #     Update the distance cost from current node to the target node
    #     :param robot_id: string, robot name
    #     :param curr_node: string, current node
    #     :param target: string, target node
    #     :param route_dist_cost: float, total distance cost from current node to target
    #     """
    #     self.dist_cost_to_target[robot_id]['curr_node'] = curr_node
    #     self.dist_cost_to_target[robot_id]['target_node'] = target
    #     self.dist_cost_to_target[robot_id]['dist_cost'] = route_dist_cost
    #     return self.dist_cost_to_target

    # moved to topo.py
    def time_to_dist_cost(self, waiting_time):
        """
        :param waiting_time: the time robot will be waiting
        return: float, the distance cost
        """
        return (self.transportation_rate + random.gauss(0, self.transportation_rate_std)) * waiting_time

    # moved to topo.py
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
