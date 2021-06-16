#!/usr/bin/env python
# ----------------------------------
# @author: ZuyuanZhu
# @email: zuyuanzhu@gmail.com
# @date: 11-06-2021
# @info: A class to resolve deadlocks
# ----------------------------------


import topological_simpy.robot_sim


class DeadLock(object):
    """
    A class to resolve deadlocks
    """
    def __init__(self, graph, robot):
        self.graph = graph

    # TODO: when split out the simpy.container tmap, use this class for deadlocks related functions
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

    # back up deadlock solution
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
                            # for robot_id_dl in self.deadlocks['deadlock_robot_ids']:
                            self.deadlock_coordinator(self.deadlocks['deadlock_robot_ids'])
                            yield self.env.timeout(self.loop_timeout)

                            yield self.env.process(self.wait_to_proceed(robot_id))

                            if self.dodge_status[robot_id]['to_dodge'] is True:
                                self.loginfo('| %5.1f: %s is in deadlock, change route now' %
                                             (self.env.now, robot_id))
                                route_nodes = self.deadlock_dodge(robot_id, n, target, new_route)
                                # TODO: 1. find avoid route: new_route = self.get_route_nodes(
                                #                                           self.curr_node[robot_id], target, avoid_nodes),
                                #          if new_route is not None, go for new_route; if new_route is None, then?
                                # if new_route is not None:  # TODO: is it possible?
                                #     route_nodes = new_route
                                # else:
                                #     avoid_nodes = self.graph.get_active_nodes([robot_id])
                                #     curr_node_edges = self.graph.get_node_edges(self.curr_node[robot_id])
                                #     # get all the edge nodes that could be used for dodging
                                #     for node in avoid_nodes:
                                #         if node in curr_node_edges:
                                #             curr_node_edges.remove(node)
                                #     # find the node to dodge:  node_to_dodge
                                #     if n in avoid_nodes:
                                #         avoid_nodes.remove(n)
                                #         route_nodes_to_dodge = self.graph.get_route_between_adjacent_nodes(
                                #             self.curr_node[robot_id], n, avoid_nodes)
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
                                #                 robot_id, self.curr_node[robot_id], curr_node_edges)
                                #             raise Exception(msg)
                                #             # route_nodes = None
                                #             # yield self.env.timeout(self.loop_timeout)
                                #     else:
                                #         # todo: impossible, remove the if-else later
                                #         msg = "%s: %s not in avoid_nodes!" % (robot_id, n)
                                #         raise Exception(msg)

                                # prepare to change to the new route_nodes
                                self.cancel_hold_time(n, hold_time)  # cancel the hold_time just set
                                idx = 0
                                change_route = True
                                self.loginfo('^ %5.1f: %s go NEW route: %s' %
                                             (self.env.now, robot_id, route_nodes))
                                # break

                        if change_route:
                            self.log_cost(robot_id, 0, 0, 'CHANGE ROUTE')  # for monitor
                            self.remove_robot_from_deadlock(robot_id)

                            self.inform_dodged(robot_id)  # TODO[today], then reset all dodge? Or reset when robot reaching next node?

                            self.other_robot_dodged = True   #  TODO testing

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
            except Exception:
                # Not triggered when requesting an occupied node!
                # Not triggered when the robot has a goal running and be assigned a new goal
                self.loginfo('  %5.1f: %s INTERRUPTED while waiting to gain access to go from node %s going to node %s'
                             % (self.env.now, robot_id, self.curr_node[robot_id], n))
                self.log_cost(robot_id, 0, 0, 'INTERRUPTED')  # for monitor
                self.loginfo('  %5.1f: @@@ %s release previously acquired target node %s' %
                             (self.env.now, robot_id, n))
                yield self.release_node(robot_id, n)
                if robot_id in self.dead_locks:
                    interrupted = 'deadlock'    # interrupted by farm.scheduler_monitor: robot._goto_process.interrupt()
                else:
                    interrupted = 'waiting'
                # TODO [next]: reset robot status. send robot back to a safe place? Use a new mode to deal with the
                #  exception.
                break

            try:
                time_to_travel_before_release = round(d_cur / (2 * (self.transportation_rate +
                                                                    random.gauss(0, self.transportation_rate_std))), 1)
                yield self.env.timeout(time_to_travel_before_release)
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
                self._goto_process = self.env.process(self._goto_container(target))
                yield self._goto_process
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
