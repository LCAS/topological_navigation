#! /usr/bin/env python

import math
import numpy
import random
from geometry_msgs_sim import Pose


class PickerSim(Pose):
    def __init__(self, picker_id, tray_capacity, max_n_trays, picking_rate, transportation_rate, unloading_time, env,
                 topo_graph, robots, with_robots, verbose):
        """
        Keyword arguments:
        picker_id -- name/id of the picker
        try_capacity -- capacity of the tray picker is carrying
        max_n_trays -- number of trays with the picker
        picking_rate -- rate at which the picker moves while picking
        transportation_rate -- rate at which the picker moves while transporting
        unloading_time -- time the picker will spend at the local storage for unloading
        env -- simpy.Environment
        topo_graph -- TopologicalForkGraph object
        robots -- list of robot agents
        """

        Pose.__init__(self)   # TODO: check Pose is working?
        self.with_robots = with_robots
        self.closest_node = []
        self.current_node = []
        self.pose = Pose()

        self.picker_state = "INIT"   # picker state, "INIT", "CALLED", "WAIT", "LOADED" TODO: more states needed?
        self.curr_iteration = None

        self.picker_id = picker_id
        self.env = env

        self.robots = {}
        self.robot_ids = []
        self.n_robots = len(robots)
        for robot in robots:
            robot_id = robot.robot_id
            self.robot_ids.append(robot_id)
            self.robots[robot_id] = robot

        self.verbose = verbose

        self.n_rows = 0
        self.n_trays = 0     # current number of trays with the picker
        self.tot_trays = 0   # total number of trays by the picker
        self.tray_capacity = tray_capacity
        self.picking_rate = picking_rate
        self.picking_rate_std = 0.02 * self.picking_rate
        self.transportation_rate = transportation_rate
        self.transportation_rate_std = 0.02 * self.transportation_rate
        self.max_n_trays = max_n_trays
        self.graph = topo_graph

        # time/tray same for loading on robot or unloading at localStorage
        self.unloading_time = unloading_time

        # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
        # 4: waiting for unload at storage, 5: waiting for loading on robot
        # 6: transporting to local storage from cold storage
        self.mode = 0

        self.picking_finished = False
        self.allocation_finished = False

        self.curr_node = None
        self.local_storage_node = None
        self.picking_dir = None     # "forward" or "reverse"

        self.curr_row = None
        self.prev_row = None

        # [head_node, start_node, end_node, local_storage_node]
        self.curr_row_info = []
        self.prev_row_info = []

        self.row_finish_time = 0.  # time at which current row is completed

        self.row_path = []
        self.goal_node = None

        self.picking_progress = 0.  # percentage of tray_capacity

        # parameters to check utilisation
        self.time_spent_idle = 0.
        self.time_spent_transportation = 0.
        self.time_spent_picking = 0.
        self.time_spent_unloading = 0.
        self.time_spent_waiting = 0.
        self.time_spent_loading = 0.
        self.time_spent_working = lambda: self.time_spent_picking + self.time_spent_transportation + self.time_spent_unloading

        self.process_timeout = 0.10
        self.loop_timeout = 0.10

        self.assigned_robot_id = None
        self.continue_picking = True

        # local storage node of the first row is assumed to be the starting loc?
        # After reaching another local storage, the robot can wait there
        self.curr_node = self.graph.local_storage_nodes[self.graph.row_ids[0]]
        # update agent_nodes in topo_graph
        self.graph.agent_nodes[self.picker_id] = self.curr_node

        self.use_local_storage = self.graph.use_local_storage  # if False, store at cold storage

        self.cold_storage_node = self.graph.cold_storage_node

        self.action = self.env.process(self.normal_operation())

    def get_robot_state(self):
        if self.assigned_robot_id:
            return self.robots[self.assigned_robot_id].robot_state

    def set_picker_state(self, state):
        # msg = std_msgs.msg.String()
        # msg.data = '{\"user\":\"%s\", \"state\": \"%s\"}' %(self.picker_id, state)
        self.picker_state = state

    def loginfo(self, msg):
        """
        print msg based on the flag - verbose
        """
        if self.verbose:
            print(msg)

    def allocate_row_to_picker(self, row_id):
        """
        scheduler calls this to assign a row to the picker
        """
        try:
            assert self.mode == 0
        except AssertionError:
            raise Exception('Scheduler is trying to allocate %s to %s, but it is in %d' %
                            (self.picker_id, row_id, self.mode))

        self.curr_row = row_id
        self.curr_row_info = self.graph.row_info[row_id]
        self.local_storage_node = self.curr_row_info[3]
        self.picking_dir = "forward"
        self.row_path, _, _ = self.graph.get_path_details(self.curr_row_info[1], self.curr_row_info[2])
        self.goal_node = "" + self.curr_row_info[1]

    def assign_robot_to_picker(self, robot_id):
        """
        scheduler calls this to assign a robot to the picker
        """
        try:
            assert self.mode == 5
        except AssertionError:
            raise Exception("Scheduler is trying to assign %s to %s, but picker is in %d" %
                            (robot_id, self.picker_id, self.mode))

        self.assigned_robot_id = robot_id

    def proceed_with_picking(self):
        """
        scheduler lets the picker continue picking after loading trays on robot.
        this is just to make sure the scheduler is aware that the robot is loaded
        """
        self.loginfo("  %5.1f: %s can continue picking" % (self.env.now, self.picker_id))
        self.continue_picking = True

    def picking_node_to_node(self):
        """
        mimic picking from one node to another
        """
        try:
            assert self.picking_dir == "forward" or "reverse"
        except AssertionError:
            raise Exception("Allowed picking dir: forward, reverse, but picker is using %s" % self.picking_dir)

        curr_node_index = self.row_path.index(self.curr_node)
        if self.picking_dir == "forward":
            next_node = self.row_path[curr_node_index + 1]
        elif self.picking_dir == "reverse":
            next_node = self.row_path[curr_node_index - 1]

        # move to the next node
        # adding Gaussian white noise to introduce variations
        yield self.env.process(self.go_to_node(next_node, self.picking_rate + random.gauss(0, self.picking_rate_std)))

        # update picking_process and n_tray
        self.picking_progress += self.graph.yield_at_node[self.curr_node]
        if self.picking_progress > self.tray_capacity or numpy.isclose(self.picking_progress, self.tray_capacity):
            self.n_trays += 1
            self.picking_progress -= self.tray_capacity

        yield self.env.timeout(self.process_timeout)

    def update_trays_unload(self):
        """
        update tray counts after unloading
        """
        self.tot_trays += self.n_trays
        self.n_trays = 0

        if self.picking_finished or self.allocation_finished:
            self.tot_trays += self.picking_progress / self.tray_capacity
            self.picking_progress = 0.

    def dist_to_robot(self):
        """
        return Euclidean distance between robot's pose and picker's pose
        """
        robot_node = self.robots[self.assigned_robot_id].curr_node

        if robot_node == self.curr_node:
            return 0
        else:
            robot_node_obj = self.graph.get_node(robot_node)
            robot_x = robot_node_obj.pose.position.x  # TODO: to be matched with the fake ROS msg
            robot_y = robot_node_obj.pose.position.y

            curr_node_obj = self.graph.get_node(robot_node)
            curr_x = curr_node_obj.pose.position.x
            curr_y = curr_node_obj.pose.position.y

            return math.hypot((robot_x - curr_x), (robot_y - curr_y))

    def update_trays_loaded(self):
        """
        update tray_counts after loading on a robot
        """
        self.tot_trays += self.n_trays
        self.n_trays = 0

        # inform the robot that the tray has been loaded, so the robot can stop waiting for the picker
        self.robots[self.assigned_robot_id].trays_loaded()

    def reset_robot_assignment(self):
        """
        reset the assigned robot info after loading
        To ensure the picker won't have a robot assigned, the next time the picker's trays are full
        and the current robot has not unloaded yet
        """
        self.assigned_robot_id = None

    def go_to_node(self, goal_node, nav_speed):
        """
        Simpy process to Mimic moving to the goal_node by publishing new position based on speed
        :param goal_node: string, topological node to reach from current node
        :param nav_speed: float, navigation speed (picking / transportation rate)
        """
        self.loginfo('  %5.1f: %s going to %s from %s' % (self.env.now, self.picker_id, goal_node, self.curr_node))
        route_nodes, _, route_distance = self.graph.get_path_details(self.curr_node, goal_node)  # TODO: avoid dead node?
        for i in range(len(route_nodes) - 1):
            # move through each node
            edge_distance = route_distance[i]
            travel_time = edge_distance / nav_speed
            yield self.env.timeout(travel_time)

            self.curr_node = route_nodes[i + 1]

            # update agent_nodes in the topo_graph
            self.graph.agent_nodes[self.picker_id] = self.curr_node

        self.loginfo('& %5.1f: %s reached %s' % (self.env.now, self.picker_id, goal_node))
        yield self.env.timeout(self.process_timeout)

    def finished_row_routine(self):
        """
        Common things to do when picking along the allocated row is finished - reset and update
        """
        self.picking_dir = None
        self.prev_row = "" + self.curr_row
        self.prev_row_info = [] + self.curr_row_info
        self.curr_row = None
        self.row_path = []
        self.row_finish_time = self.env.now
        self.n_rows += 1

    def inform_picking_finished(self):
        """
        called by farm - scheduler to indicate all rows are now picked
        """
        self.picking_finished = True

    def inform_allocation_finished(self):
        """
        called by farm - scheduler to indicate all rows are now allocated
        """
        self.allocation_finished = True

    def normal_operation(self):
        """ Picker's picking process when there are robots to carry full trays
        """
        idle_start_time = self.env.now
        transportation_start_time = 0.
        picking_start_time = 0.
        unloading_start_time = 0.
        waiting_start_time = 0.
        loading_start_time = 0.

        while True:
            if self.mode == 0:
                # picker is idle. wait for a row allocation or picking_finished status
                if self.picking_finished:
                    # go to local storage and unload all trays
                    if self.curr_node == self.local_storage_node:
                        # if already at local storage - do nothing
                        self.loginfo("  %5.1f: %s is idle and at local storage" % (self.env.now, self.picker_id))
                        self.mode = 4
                        unloading_start_time = self.env.now
                    else:
                        # go to local storage node
                        self.loginfo("  %5.1f: %s is idle and going to local storage" % (self.env.now, self.picker_id))
                        self.goal_node = "" + self.local_storage_node
                        self.mode = 3
                        transportation_start_time = self.env.now

                    # mode changed from idle
                    self.time_spent_idle += self.env.now - idle_start_time

                elif self.curr_row is not None:
                    # idle picker allocated to a new row
                    self.loginfo("  %5.1f: %s is allocated to %s" %(self.env.now, self.picker_id, self.curr_row))
                    self.time_spent_idle += self.env.now - idle_start_time
                    self.mode = 1 # transporting to a row_node from curr_node
                    transportation_start_time = self.env.now

                elif self.allocation_finished:
                    # all rows allocated and no assignment for the picker (if there
                    # was any, previous condition would have been satisfied.)
                    # go to local storage and unload all trays
                    if self.curr_node == self.local_storage_node:
                        # if already at local storage - do nothing
                        self.loginfo("%s is idle and at local storage" % (self.env.now, self.picker_id))
                        self.mode = 4
                        unloading_start_time = self.env.now
                    else:
                        # go to local storage node
                        self.loginfo("%s is idle and going to local storage" % self.picker_id)
                        self.goal_node = "" + self.local_storage_node
                        self.mode = 3
                        transportation_start_time = self.env.now

                    # mode changed from idle
                    self.time_spent_idle += self.env.now - idle_start_time

                # mode remains idle - not updating self.time_spent_idle now

            elif self.mode == 1:
                # from curr_node go to a row_node (self.goal_node) and continue picking
                # goal_node is set in allocate_row
                self.loginfo("  %5.1f: %s going to %s from %s" %
                             (self.env.now, self.picker_id, self.goal_node, self.curr_node))
#                yield self.env.process(self.go_to_node(self.goal_node, self.transportation_rate))
                # adding Gaussian white noise to introduce variations
                yield self.env.process(self.go_to_node(self.goal_node, self.transportation_rate + random.gauss(0, self.transportation_rate_std)))
                self.time_spent_transportation += self.env.now - transportation_start_time

                self.loginfo("P %5.1f: %s will start picking now" % (self.env.now, self.picker_id))
                self.mode = 2 # picking
                picking_start_time = self.env.now

            elif self.mode == 2:
                # at curr_node, pick through a node_dist to next_node
                yield self.env.process(self.picking_node_to_node())
                self.time_spent_picking += self.env.now - picking_start_time

                if self.curr_row in self.graph.half_rows:
                    # row_end and dir_change nodes are the same
                    row_end_node = self.curr_row_info[2]
                    dir_change_node = self.curr_row_info[2]
                else:
                    row_end_node = self.curr_row_info[1]
                    dir_change_node = self.curr_row_info[2]

                # decide what is the next mode of action
                if self.n_trays >= self.max_n_trays:
                    self.loginfo("T %5.1f: %s has trays full" % (self.env.now, self.picker_id))
                    if not self.with_robots:
                        # picker should go to local storage to unload
                        # but, should s/he return?
                        if self.curr_node == row_end_node:
                            # inform row complete and go to local storage node and don't return
                            self.loginfo("  %5.1f: %s finished %s" % (self.env.now, self.picker_id, self.curr_row))
                            self.finished_row_routine()
                            self.goal_node = "" + self.local_storage_node
                        elif self.curr_node == dir_change_node:
                            # forward -> reverse
                            self.loginfo("  %5.1f: %s changing direction to reverse" % (self.env.now, self.picker_id))
                            self.picking_dir = "reverse"
                            self.goal_node = "" + self.curr_node
                        else:
                            # row not finished, so come back to curr_node and continue in same dir
                            self.loginfo("  %5.1f: %s going to %s and will return to %s" %
                                         (self.env.now, self.picker_id, self.local_storage_node, self.curr_node))
                            self.goal_node = "" + self.curr_node

                        self.mode = 3
                        transportation_start_time = self.env.now
                    else:
                        if self.curr_node == row_end_node:
                            # inform row complete
                            self.loginfo("  %5.1f: %s finished %s" % (self.env.now, self.picker_id, self.curr_row))
                            self.finished_row_routine()
                        elif self.curr_node == dir_change_node:
                            self.loginfo("  %5.1f: %s changing direction to reverse" % (self.env.now, self.picker_id))
                            self.picking_dir = "reverse"

                        # go to wait_and_load_on_robot mode (5)
                        self.loginfo("  %5.1f: %s waiting for a robot to collect the trays" %
                                     (self.env.now, self.picker_id))
                        self.continue_picking = False
                        self.mode = 5
                        self.set_picker_state("CALLED")

                        # wait for the event to register
                        while True:
                            robot_state = self.get_robot_state()
                            # if self.state == "CALLED" or self.state == "ACCEPT" or self.state == "ARRIVED":
                            if robot_state == "CALLED" or robot_state == "ACCEPT" or robot_state == "ARRIVED":
                                break
                            else:
                                yield self.env.timeout(self.loop_timeout)

                        waiting_start_time = self.env.now

                else:
                    # trays not full but has the row finished? if finished, wait for next allocation
                    if self.curr_node == row_end_node:
                        self.loginfo("  %5.1f: %s finished %s" % (self.env.now, self.picker_id, self.curr_row))
                        self.finished_row_routine()
                        self.mode = 0
                        idle_start_time = self.env.now

                    elif self.curr_node == dir_change_node:
                        # forward -> reverse, continue picking
                        self.loginfo("  %5.1f: %s changing direction to reverse" % (self.env.now, self.picker_id))
                        self.picking_dir = "reverse"
                        self.mode = 2
                        picking_start_time = self.env.now

                    else:
                        # in-between node continue picking
                        self.mode = 2
                        picking_start_time = self.env.now

            elif self.mode == 3:

                storage_node = self.local_storage_node if self.use_local_storage else self.cold_storage_node

                # go to local storage node and change mode to 4
                yield self.env.process(self.go_to_node(storage_node,
                                                       self.transportation_rate))
                self.time_spent_transportation += self.env.now - transportation_start_time

                self.mode = 4   # unload at local or cold storage
                unloading_start_time = self.env.now

            elif self.mode == 4:
                # wait and unload at local / cold storage
                # a picker can be in this mode in both cases - with and without robots
                # with robots it could happen only at the end
                # without robots, it could be at the end or when trays are full

                if self.n_trays > 0 or self.picking_progress > 0:
                    if self.use_local_storage:
                        storage = self.graph.local_storages[self.local_storage_node]
                        storage_node = self.local_storage_node
                    else:
                        storage = self.graph.cold_storage
                        storage_node = self.cold_storage_node

                    with storage.request() as req:
                        yield req
                        if not self.with_robots: # without robots
                            if self.picking_finished:
                                self.loginfo("  %5.1f: %s unloading all trays at %s" %
                                             (self.env.now, self.picker_id, storage_node))
                                wait_time = self.unloading_time * (
                                    self.n_trays if self.picking_progress == 0 else self.n_trays + 1)
                            else:
                                self.loginfo("  %5.1f: %s unloading full trays at %s" %
                                             (self.env.now, self.picker_id, storage_node))
                                wait_time = self.unloading_time * self.n_trays
                            yield self.env.timeout(wait_time)
                            self.time_spent_unloading += self.env.now - unloading_start_time
                            self.update_trays_unloaded()

                            if self.curr_row is None:
                                # current row is finished. what next?
                                if self.use_local_storage:
                                    # local storage
                                    # if there are no current allocation, stay here
                                    self.mode = 0
                                    idle_start_time = self.env.now
                                else:
                                    # cold storage
                                    if self.allocation_finished:
                                        # all rows allocated stay here
                                        self.mode = 0
                                        idle_start_time = self.env.now
                                    else:
                                        # still unallocated rows, go to local storage of previous
                                        self.mode = 6
                                        transportation_start_time = self.env.now
                            else:
                                # go back to previous node
                                self.mode = 1
                                transportation_start_time = self.env.now

                        else: # with robots
                            # this unloading will happen if s/he finished picking the last allocated row
                            # and tray is not full
                            self.loginfo("  %5.1f: %s unloading all trays at %s" %
                                         (self.env.now, self.picker_id, storage_node))
                            wait_time = self.unloading_time * (self.n_trays if self.picking_progress == 0 else self.n_trays + 1)
                            yield self.env.timeout(wait_time)
                            self.time_spent_unloading += self.env.now - unloading_start_time
                            self.update_trays_unloaded()

                            self.mode = 0
                            idle_start_time = self.env.now

                if self.picking_finished or (self.allocation_finished and self.mode == 0):
                    self.loginfo("  %5.1f: all rows picked. %s exiting" % (self.env.now, self.picker_id))
                    self.env.exit("all rows picked and idle")
                    break

            elif self.mode == 5:
                # wait for the robot to arrive
                # the car state would change from CALLED -> ACCEPT -> ARRIVED
                # after loading set state to LOADED
                robot_state = self.get_robot_state()
                # if self.state == "CALLED" or self.state == "ACCEPT":
                if robot_state == "CALLED" or robot_state == "ACCEPT":
                    # robot not yet assigned. wait
                    pass
                # elif self.state == "ARRIVED":
                elif robot_state == "ARRIVED":
                    # robot is here. load the full trays and set state as LOADED
                    self.loginfo("  %5.1f: %s reached %s" % (self.env.now, self.assigned_robot_id, self.picker_id))
                    self.time_spent_waiting += self.env.now - waiting_start_time

                    # wait for loading on the assigned robot
                    loading_start_time = self.env.now
                    wait_time = self.unloading_time * self.n_trays
                    yield self.env.timeout(wait_time)
                    self.loginfo("  %5.1f: %s loaded full trays on %s" %
                                 (self.env.now, self.picker_id, self.assigned_robot_id))
                    self.time_spent_loading += self.env.now - loading_start_time

                    self.update_trays_loaded()
                    # set state to LOADED
                    # scheduler should know from the CAR status that the tray is loaded
                    self.set_picker_state("LOADED")
                # elif self.state == "INIT":
                elif robot_state == "INIT":
                    # scheduler knew the robot is loaded and set the state of picker to INIT
                    # the picker is now free to continue picking
                    if self.curr_row is not None:
                        self.loginfo("  %5.1f: %s will continue picking %s" %
                                     (self.env.now, self.picker_id, self.curr_row))
                        self.mode = 2
                        picking_start_time = self.env.now
                    else:
                        self.loginfo("  %5.1f: %s does not have any rows allocated" % (self.env.now, self.picker_id))
                        self.mode = 0
                        idle_start_time = self.env.now

            elif self.mode == 6:
                # go back to previous row's local storage node
                # from curr_node go to local_storage_node and stay idle
                waiting_node = self.prev_row_info[3]
                self.loginfo("  %5.1f: %s going to %s from %s" %
                             (self.env.now, self.picker_id, self.goal_node, waiting_node))
                yield self.env.process(self.go_to_node(waiting_node, self.transportation_rate))
                self.time_spent_transportation += self.env.now - transportation_start_time

                self.loginfo("  %5.1f: %s is idle now" % (self.env.now, self.picker_id))
                self.mode = 0  # picking
                idle_start_time = self.env.now

                if self.picking_finished or (self.allocation_finished and self.mode == 0):
                    self.loginfo("  %5.1f: all rows picked. %s exiting" % (self.env.now, self.picker_id))
                    self.env.exit("all rows picked and idle")
                    break

            yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)
