#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import random
import rospy
import numpy


class Robot(object):
    """Robot class definition"""
    def __init__(self, robot_id, transportation_rate, max_n_trays, unloading_time,
                 env, topo_graph, verbose):
        self.robot_id = robot_id
        self.env = env
        self.graph = topo_graph
        self.transportation_rate = transportation_rate
        self.transportation_rate_std = 0.02 * self.transportation_rate  # list(map(lambda x: x*0.02, self.transportation_rate))
        self.max_n_trays = max_n_trays
        self.n_empty_trays = self.max_n_trays
        self.n_full_trays = 0
        self.tot_trays = 0
        self.unloading_time = unloading_time

        self.verbose = verbose

        # 0 - idle, 1 - transporting_to_picker, 2 - waiting for loading,
        # 3 - waiting for unloading, 4 - transporting to storage, 5- charging
        # 6 - return to local_storage from cold_storage
        self.mode = 0

        # parameters to check utilisation
        self.time_spent_picking = 0.
        self.time_spent_transportation = 0.
        self.time_spent_idle = 0.
        self.time_spent_loading = 0.
        self.time_spent_unloading = 0.
        self.time_spent_charging = 0.
        self.time_spent_working = lambda: self.time_spent_loading + self.time_spent_transportation + self.time_spent_unloading

        self.loaded = False
        self.picking_finished = False # this will be modified by farm
        self.allocation_finished = False
        self.continue_transporting = True

        # TODO: local storage node of the first row is assumed to be the starting loc
        # After reaching another local storage, the robot can wait there
        self.curr_node = self.graph.local_storage_nodes[self.graph.row_ids[0]]
        # update agent_nodes in topo_graph, robot's initial node
        self.graph.agent_nodes[self.robot_id] = self.curr_node

        self.process_timeout = 0.10
        self.loop_timeout = 0.10

        self.battery_charge = 100.0

        # parameters related to a picking operation
        self.assigned_picker_id = None
        self.assigned_picker_node = None
        self.assigned_picker_n_trays = 0
        self.assigned_local_storage_node = None

        self.use_local_storage = self.graph.use_local_storage  # if False, store at cold storage
        self.cold_storage_node = self.graph.cold_storage_node

        self.action = self.env.process(self.normal_operation())

    def normal_operation(self):
        """normal operation sequences of the robot in different modes"""
        idle_start_time = self.env.now
        transportation_start_time = 0.
        loading_start_time = 0.
        unloading_start_time = 0.
        charging_start_time = 0.

        while True:
            if rospy.is_shutdown():
                break

            if self.picking_finished and (self.mode == 0 or self.mode == 5):
                self.loginfo("all rows picked. %s exiting" %(self.robot_id))
                self.env.exit("all rows picked and idle")
                break

            if self.mode == 0:
                # check for assignments
                if self.assigned_picker_id is not None:
                    self.time_spent_idle += self.env.now - idle_start_time
                    # change mode to transport to picker
                    self.mode = 1
                    transportation_start_time = self.env.now
                    self.loginfo("%s is assigned to %s" %(self.robot_id, self.assigned_picker_id))

                # TODO: idle state battery charge changes
                if self.battery_charge < 40.0 or numpy.isclose(self.battery_charge, 40.0):
                    self.loginfo("battery low on %s, going to charging mode" %(self.robot_id))
                    self.time_spent_idle += self.env.now - idle_start_time
                    # change mode to charging
                    self.mode = 5
                    charging_start_time = self.env.now

            elif self.mode == 1:
                # farm assigns the robot to a picker by calling assign_robot_to_picker
                # go to picker_node from curr_node
                self.loginfo("%s going to %s" %(self.robot_id, self.assigned_picker_node))
                yield self.env.process(self.go_to_node(self.assigned_picker_node))
                self.time_spent_transportation += self.env.now - transportation_start_time
                self.loginfo("%s reached %s" %(self.robot_id, self.assigned_picker_node))
                # change mode to waiting_for_loading
                self.continue_transporting = False
                self.mode = 2
                loading_start_time = self.env.now

            elif self.mode == 2:
                self.loginfo("%s is waiting for the trays to be loaded" %(self.robot_id))
                yield self.env.process(self.wait_for_loading()) # this reset mode to 3
                self.time_spent_loading += self.env.now - loading_start_time
                self.loginfo("trays are loaded on %s" %(self.robot_id))
                while True:
                    if rospy.is_shutdown():
                        break

                    if self.continue_transporting:
                        # change mode to transporting to storage
                        self.mode = 3
                        transportation_start_time = self.env.now
                        break

                    yield self.env.timeout(self.loop_timeout)

            elif self.mode == 3:
                if self.use_local_storage:
                    # go to local_storage_node from picker_node
                    self.loginfo("%s going to %s" %(self.robot_id, self.assigned_local_storage_node))
                    yield self.env.process(self.go_to_node(self.assigned_local_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("%s reached %s" %(self.robot_id, self.assigned_local_storage_node))

                else:
                    # go to cold_storage_node from picker_node
                    self.loginfo("%s going to %s" %(self.robot_id, self.cold_storage_node))
                    yield self.env.process(self.go_to_node(self.cold_storage_node))
                    self.time_spent_transportation += self.env.now - transportation_start_time
                    self.loginfo("%s reached %s" %(self.robot_id, self.cold_storage_node))

                # change mode to unloading
                self.mode = 4
                unloading_start_time = self.env.now

            elif self.mode == 4:
                # wait for unloading
                self.loginfo("%s is waiting for the trays to be unloaded" %(self.robot_id))
                local_storage = self.assigned_local_storage_node # backup needed for mode 6 if going that way
                yield self.env.process(self.wait_for_unloading()) # this reset mode to 0
                self.time_spent_unloading += self.env.now - unloading_start_time
                self.loginfo("trays are unloaded from %s" %(self.robot_id))
                if self.use_local_storage:
                    # change mode to idle
                    local_storage = None # no need of local storage, reset
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
                self.loginfo("%s going to %s" %(self.robot_id, local_storage))
                yield self.env.process(self.go_to_node(local_storage))
                self.time_spent_transportation += self.env.now - transportation_start_time
                self.loginfo("%s reached %s" %(self.robot_id, local_storage))
                local_storage = None

                # change mode to idle
                self.mode = 0
                idle_start_time = self.env.now

            yield self.env.timeout(self.loop_timeout)
        yield self.env.timeout(self.process_timeout)

    def assign_robot_to_picker(self, picker_id, picker_node, n_trays, local_storage_node):
        """assign a picker to the robot, if it is idle - called by scheduler"""
        try:
            assert self.mode == 0
        except AssertionError:
            raise Exception("Scheduler is trying to assign %s to %s, but robot is in %d" %(self.robot_id, picker_id, self.mode))

        self.assigned_picker_id = picker_id
        self.assigned_picker_node = picker_node
        self.assigned_picker_n_trays = n_trays
        self.assigned_local_storage_node = local_storage_node

    def proceed_with_transporting(self):
        """scheduler confirms the robot can go to local storage and unload.
        this is to make sure the scheduler is aware that the picker has loaded trays on the robot.
        """
        self.loginfo("%s can continue transporting to local storage" %(self.robot_id))
        self.continue_transporting = True

    def go_to_node(self, goal_node):
        """Simpy process to Mimic moving to the goal_node

        Keyword arguments:
        goal_node -- node to reach from current node
        """
        self.loginfo("%s going to %s from %s" %(self.robot_id, goal_node, self.curr_node))
        route_nodes, _, route_distance = self.graph.get_path_details(self.curr_node, goal_node)
        for i in range(len(route_nodes) - 1):
            # move through each edge
            if rospy.is_shutdown():
                break

            edge_distance = route_distance[i]
#            travel_time = edge_distance / self.transportation_rate
            # adding Gaussian white noise to introduce variations
            travel_time = edge_distance / (self.transportation_rate + random.gauss(0, self.transportation_rate_std))

            # travel the node distance
            yield self.env.timeout(travel_time)

            self.curr_node = route_nodes[i + 1]

            # update agent_nodes in the topo_graph
            self.graph.agent_nodes[self.robot_id] = self.curr_node

        self.loginfo("%s reached %s" %(self.robot_id, goal_node))
        yield self.env.timeout(self.process_timeout)

    def wait_for_loading(self):
        """wait until picker loads trays and confirms it"""
        while True:
            # wait until picker calls loading_complete
            if rospy.is_shutdown():
                break
            if self.loaded:
                break
            else:
                # TODO: battery decay
                yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)

    def trays_loaded(self):
        """picker calls this to indicate the trays are loaded"""
        self.n_empty_trays -= self.assigned_picker_n_trays
        self.n_full_trays += self.assigned_picker_n_trays
        self.loaded = True

    def wait_for_unloading(self):
        """wait for unloading the trays at the local storage"""
        storage = self.graph.local_storages[self.assigned_local_storage_node] if self.use_local_storage else self.graph.cold_storage
        with storage.request() as req:
            # request to access local storage
            yield req
            # access to local storage is granted
            unloading_time = self.unloading_time * self.assigned_picker_n_trays
            yield self.env.timeout(unloading_time)
        self.trays_unloaded()
        yield self.env.timeout(self.process_timeout)

    def trays_unloaded(self):
        """update tray counts and assignments"""
        self.tot_trays += self.assigned_picker_n_trays
        self.n_empty_trays += self.assigned_picker_n_trays
        self.n_full_trays -= self.assigned_picker_n_trays
        self.assigned_picker_id = None
        self.assigned_picker_node = None
        self.assigned_picker_n_trays = 0
        self.assigned_local_storage_node = None
        self.loaded = False

    def charging_process(self):
        """charging process"""
        while True:
            if rospy.is_shutdown():
                break
            yield self.env.timeout(1)
            self.battery_charge += 1

            if self.battery_charge == 100.:
                break

        yield self.env.timeout(self.process_timeout)

    def inform_picking_finished(self):
        """called by farm - scheduler to indicate all rows are now picked"""
        self.picking_finished = True

    def inform_allocation_finished(self):
        """called by farm - scheduler to indicate all rows are now allocated"""
        self.allocation_finished = True

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            rospy.loginfo(msg)
