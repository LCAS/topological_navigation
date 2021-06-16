#!/usr/bin/env python
# ----------------------------------
# @author: gpdas, ZuyuanZhu
# @maintainer: ZuyuanZhu
# @email: zuyuanzhu@gmail.com
# @date: 22 Apr 2021
# ----------------------------------


import topological_simpy.farm


class FarmSim(topological_simpy.farm.Farm):
    """FarmSim class definition
    FarmSim is the extension of Farm class object, managing the processes of robots and pickers
    """

    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, n_iteration, verbose):
        """inherit from Farm object
        """
        super(FarmSim, self).__init__(name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose)

        self.n_iteration = n_iteration

        self.active_iterations = []

        self.finished_rows = {}
        self.n_finished_rows = {}
        self.row_finish_time = {}

        # related to allocation
        self.unallocated_rows = {}
        self.allocations = {}
        self.allocation_time = {}

        self.picker_allocations = {}
        self.curr_picker_allocations = {picker_id: (None, None) for picker_id in self.picker_ids}
        self.allocated_pickers = {}

        self.finished_iterations = False
        self.finished_picking = lambda iteration: True if self.n_finished_rows[
                                                              iteration] == self.n_topo_nav_rows else False
        #        self.finished_allocating = lambda iteration: False if self.unallocated_rows[iteration] else True
        self.finished_allocating = lambda iteration: True if len(self.unallocated_rows[iteration]) == 0 else False

        self.action = self.env.process(self.scheduler_monitor())

    @staticmethod
    def monitor(pickers, robots):
        """
        monitor the position and mode of pickers and robots, if their statuses change,
        then update the visualising figure in the main script
        """
        agent_status = {}
        for picker in pickers:
            agent_status[picker.picker_id] = [picker.picker_id, picker.mode, picker.curr_node]
        for robot in robots:
            agent_status[robot.robot_id] = [robot.robot_id, robot.mode, robot.graph.curr_node[robot.robot_id]]

        return agent_status

    def scheduler_monitor(self):
        """A process to allocate rows to the pickers.
        the picker should request for a row or
        when a picker becomes free, it should be allocated automatically.

        A simple implementation:
            1. do a periodic checking of completion status of rows
            2. allocate free pickers to one of the unallocated rows
        """
        predicted_from = {picker_id: None for picker_id in self.picker_ids}
        inform_allocation_finished = False
        inform_picking_finished = False
        predictions = {}

        time_now = self.env.now
        # time_now, event
        self.events.append([time_now, "starting the process"])

        while True:
            # increment curr_iteration of all idle pickers by one, if allocation for that iter is finished
            for picker_id in self.picker_ids:
                if self.pickers[picker_id].curr_iteration is None:  # Never iterated
                    self.pickers[picker_id].curr_iteration = 0
                elif picker_id in self.idle_pickers and self.pickers[picker_id].curr_iteration < self.active_iterations[
                    -1]:
                    # if allocation is finished for the iteration of the picker, increment it
                    if self.finished_allocating(self.pickers[picker_id].curr_iteration):
                        self.pickers[picker_id].curr_iteration += 1

            # try starting the next iteration
            if len(self.active_iterations) == 0:
                self.next_iteration()
            else:
                inc_iteration = True
                for iteration in self.active_iterations:
                    # Allocation is not done, stay at current active_iterations
                    if not self.finished_allocating(iteration):
                        inc_iteration = False
                        break
                if inc_iteration:
                    self.next_iteration()

            # picking finished in all rows
            if len(self.active_iterations) == 0 and self.finished_picking(
                    self.n_iteration - 1) and not inform_picking_finished:
                # time_now, event
                self.events.append([time_now, "finished picking"])
                inform_picking_finished = True
                self.loginfo("* %5.1f: all rows are picked" % self.env.now)
                for robot_id in self.robot_ids:
                    self.robots[robot_id].inform_picking_finished()
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_picking_finished()
                self.loginfo("* %5.1f: all rows picked. scheduler exiting" % self.env.now)
                self.env.exit("all rows are picked")
                break

            # allocation of all rows is finished
            if (self.n_iteration - 1) in self.active_iterations and self.finished_allocating(
                    self.n_iteration - 1) and not inform_allocation_finished:
                # time_now, event
                self.events.append([time_now, "finished row allocations"])
                self.loginfo("all rows are allocated")
                inform_allocation_finished = True  # do it only once
                for robot_id in self.robot_ids:
                    self.robots[robot_id].inform_allocation_finished()
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_allocation_finished()
                # but some picking may be going on. so don't break

            time_now = self.env.now

            for iteration in self.active_iterations:
                # ==============================================================================
                #             # update modes of pickers already assigned to a row
                # ==============================================================================
                to_remove_pickers = []

                for picker_id in self.allocated_pickers[iteration]:
                    picker = self.pickers[picker_id]
                    # picker modes
                    # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
                    # 4: waiting for unload at storage, 5: waiting for loading on robot
                    # 6: transporting to local storage from cold storage
                    if picker.mode == 0:
                        # finished the assigned row and are idle now
                        # if previously assigned any row, update its status
                        row_id = self.curr_picker_allocations[picker_id]
                        self.finished_rows[iteration].append(row_id)
                        self.n_finished_rows[iteration] += 1
                        self.row_finish_time[iteration][row_id[1]] = self.pickers[picker_id].row_finish_time
                        to_remove_pickers.append(picker_id)

                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                    elif picker.mode == 1:
                        # moving to a row_node possibly from the previous node
                        # this can happen either after a trip to a storage or after a new row allocation
                        # picker will be in picking mode (2) soon
                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                    elif picker.mode == 2:
                        # picking now
                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                        # to track predictions
                        if self.tray_full[picker_id]:
                            self.tray_full[picker_id] = False
                            self.tray_counts[picker_id] += 1
                            self.predictions[picker_id][self.tray_counts[picker_id]] = []

                    elif picker.mode == 3 or picker.mode == 4 or picker.mode == 6:
                        # picker transporting to storage (3) or unloading at storage (4)
                        # or transporting to local storage from cold storage (6)
                        # if the current row is finished, the picker's mode will be changed
                        # to idle (0) soon, which will be taken care of in next loop
                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                        # for prediction checking
                        if picker.mode == 3 and not self.tray_full[picker_id]:
                            if picker.curr_row is None:
                                prediction_actual = "%s, %s, %s, %0.1f, actual" % (
                                    picker.prev_row, picker.curr_node, picker.picking_dir, time_now)
                            else:
                                prediction_actual = "%s, %s, %s, %0.1f, actual" % (
                                    picker.curr_row, picker.curr_node, picker.picking_dir, time_now)

                            self.predictions[picker_id][self.tray_counts[picker_id]].append(prediction_actual)
                            self.tray_full[picker_id] = True

                    elif picker.mode == 5:
                        # waiting for a robot to arrive
                        # if a robot is not assigned, assign one
                        if picker_id in self.waiting_for_robot_pickers:
                            # this is an existing request and robot has been assigned or
                            # has not reached yet
                            if self.assigned_picker_robot[picker_id] is not None:
                                # a robot have been already assigned to this picker
                                # and that robot must be on its way
                                # check whether trays are loaded on the robot
                                # if loaded, remove picker from waiting_for_robot_pickers
                                robot_id = self.assigned_picker_robot[picker_id]
                                if self.robots[robot_id].loaded:
                                    self.robots[robot_id].proceed_with_transporting()
                                    self.pickers[picker_id].proceed_with_picking()
                                    self.waiting_for_robot_pickers.remove(picker_id)
                                else:
                                    # robot has not reached the picker yet
                                    pass
                            else:
                                # no robot has been assigned. scheduler
                                # will try to assign one in this round
                                pass
                        else:
                            if self.assigned_picker_robot[picker_id] is None:
                                # this is a new request for a robot, which should be assigned to this picker
                                self.waiting_for_robot_pickers.append(picker_id)

                        # for prediction checking
                        if not self.tray_full[picker_id]:
                            if picker.curr_row is None:
                                prediction_actual = "%s, %s, %s, %0.1f, actual" % (
                                    picker.prev_row, picker.curr_node, picker.picking_dir, time_now)
                            else:
                                prediction_actual = "%s, %s, %s, %0.1f, actual" % (
                                    picker.curr_row, picker.curr_node, picker.picking_dir, time_now)

                            self.predictions[picker_id][self.tray_counts[picker_id]].append(prediction_actual)
                            self.tray_full[picker_id] = True

                for picker_id in to_remove_pickers:
                    self.allocated_pickers[iteration].remove(picker_id)
                    if self.pickers[picker_id].mode == 0:
                        self.idle_pickers.append(picker_id)

                # ==============================================================================
                #             # update picker mode in picker predictors
                # ==============================================================================
                # TODO: Check whether any mode changes could be missed?
                for picker_id in self.picker_ids:
                    goal_node = None
                    picker = self.pickers[picker_id]
                    # update mode and pose in predictor
                    if picker.mode == 1:
                        goal_node = picker.goal_node
                    elif picker.mode == 3:
                        # if local storage, it will be set according to picker.curr_row
                        goal_node = picker.local_storage_node if self.graph.use_local_storage else picker.cold_storage_node
                    elif picker.mode == 6:
                        goal_node = picker.local_storage_node

                    self.predictor.update_mode_and_pose(picker_id, picker.mode, picker.curr_node,
                                                        picker.picking_dir, goal_node)
                    # make predictions - only once from a node and only when in picking mode
                    # if ((picker.mode == 2) and
                    #         ((predicted_from[picker_id] is None) or (predicted_from[picker_id] != picker.curr_node))):
                    #     # predict individual picker's tray_full events -> no global view and next row may be wrong
                    #     pred_row, pred_node, pred_dir, pred_time = self.predictor.predictors[
                    #         picker_id].predict_current_tray_full()
                    #     new_prediction = ("%s, %s, %s, %0.1f, from %s %s %0.1f" % (
                    #         pred_row, pred_node, pred_dir, pred_time, picker.curr_node, picker.picking_dir, time_now))
                    #     self.predictions[picker_id][self.tray_counts[picker_id]].append(new_prediction)
                    #     predicted_from[picker_id] = picker.curr_node

                # predict tray_full events with a global view
                # has any picker, not in previous predictions, started a new tray?
                new_tray_started = False
                for picker_id in self.picker_ids:
                    if picker_id not in predictions and self.predictor.predictors[picker_id].has_started_a_tray():
                        new_tray_started = True
                        break
                # has any picker in previous predictions changed his node?
                node_changed = False
                for picker_id in predictions:
                    if predicted_from[picker_id] != self.pickers[picker_id].curr_node:
                        node_changed = True
                        break
                # predict only if (to reduce redundant predictions)
                #   any picker, not in previous predictions, started a new tray, or
                #   any picker in previous predictions changed the node
                if new_tray_started or node_changed:
                    # predictions are made only for the pickers who has started picking and did not fill the current tray
                    predictions = self.predictor.predict_tray_full()

                    for picker_id in predictions:
                        pred_row, pred_node, pred_dir, pred_time = predictions[picker_id][1:5]
                        picker = self.pickers[picker_id]
                        new_prediction = ("%s, %s, %s, %0.1f, from %s %s %0.1f" % (
                            pred_row, pred_node, pred_dir, pred_time, picker.curr_node, picker.picking_dir, time_now))
                        self.predictions[picker_id][self.tray_counts[picker_id]].append(new_prediction)
                        predicted_from[picker_id] = picker.curr_node

                # ==============================================================================
                #             # update modes of all assigned robots
                # ==============================================================================
                to_remove_robots = []
                for robot_id in self.assigned_robots:
                    robot = self.robots[robot_id]
                    # robot modes
                    # 0 - idle, 1 - transporting_to_picker, 2 - waiting for loading,
                    # 3 - waiting for unloading, 4 - transporting to storage, 5- charging

                    # #############Test interrupt#######todo to be removed #################
                    if robot.graph.dodge[robot_id]['to_dodge'] is True:
                        print('%.1f: %s is to interrupt' % (self.env.now, robot_id))
                        robot._goto_process.interrupt()

                    # #######################################################

                    if robot.mode == 0:
                        # robot completed the unloading at storage, idle now
                        # remove current assignments and add to idle_robots
                        picker_id = self.assigned_robot_picker[robot_id]
                        self.assigned_robot_picker[robot_id] = None
                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                        to_remove_robots.append(robot_id)

                    elif robot.mode == 1:
                        # transporting to the picker node
                        pass
                    elif robot.mode == 2:
                        # waiting for the picker to load
                        picker_id = self.assigned_robot_picker[robot_id]
                        if robot.loaded and picker_id in self.waiting_for_robot_pickers:
                            # trays loaded and scheduler not yet acknowledged
                            picker_id = self.assigned_robot_picker[robot_id]
                            self.robots[robot_id].proceed_with_transporting()
                            self.pickers[picker_id].proceed_with_picking()
                            self.waiting_for_robot_pickers.remove(picker_id)
                        else:
                            # not loaded yet
                            pass
                    elif robot.mode == 3:
                        # transporting to local storage
                        picker_id = self.assigned_robot_picker[robot_id]
                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                    elif robot.mode == 4:
                        # unloading
                        picker_id = self.assigned_robot_picker[robot_id]
                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                    elif robot.mode == 5:
                        # charging - after transporting and becoming idle
                        # scheduler missed the idle mode
                        picker_id = self.assigned_robot_picker[robot_id]
                        self.assigned_robot_picker[robot_id] = None
                        if self.assigned_picker_robot[picker_id] == robot_id:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                        to_remove_robots.append(robot_id)

                    elif robot.mode == 6:
                        # transporting back to local storage after unloading at cold storage
                        picker_id = self.assigned_robot_picker[robot_id]
                        if self.assigned_picker_robot[picker_id] is not None:
                            # if there is a robot assigned to the picker, loading has been completed
                            # remove it from self.assigned_picker_robot[picker_id]
                            self.assigned_picker_robot[picker_id] = None

                # modify assigned_robots list
                for robot_id in to_remove_robots:
                    self.assigned_robots.remove(robot_id)
                    if self.robots[robot_id].mode == 0:
                        self.idle_robots.append(robot_id)
                    elif self.robots[robot_id].mode == 5:
                        self.charging_robots.append(robot_id)

                # charging robots to idle robots
                for robot_id in self.charging_robots:
                    robot = self.robots[robot_id]
                    if robot.mode == 0:
                        self.charging_robots.remove(robot_id)
                        self.idle_robots.append(robot_id)

            # row allocation to idle_pickers
            self.allocate_rows_to_pickers()

            # assign robots
            self.assign_robots_to_pickers()

            yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)

    def next_iteration(self):
        """checks whether incrementing the iteration number for all rows is possible
        """
        # check if any picker still in the oldest iterations in active_iterations
        to_remove_iterations = []
        for iteration in self.active_iterations:
            to_remove = True
            for picker_id in self.picker_ids:
                if self.pickers[picker_id].curr_iteration == iteration:
                    # if any picker is in this iteration don't remove it
                    to_remove = False
                    break
            if to_remove:
                to_remove_iterations.append(iteration)
            else:
                # ignore next iterations as well
                break
        for iteration in to_remove_iterations:
            self.active_iterations.remove(iteration)

        # pending allocations in active iterations
        pending_allocations = False
        for picker_id in self.idle_pickers:
            if len(self.active_iterations) == 0:
                break
            elif not self.finished_allocating(self.pickers[picker_id].curr_iteration):
                pending_allocations = True
                break

        # start next iteration if all iterations are not done
        # this will add an iteration although the previous one is not finished_allocating
        if not pending_allocations:
            if len(self.active_iterations) == 0 or self.active_iterations[-1] < self.n_iteration - 1:
                if len(self.active_iterations) == 0:
                    iteration = 0
                else:
                    iteration = self.active_iterations[-1] + 1
                self.active_iterations.append(iteration)

                self.finished_rows[iteration] = []
                self.n_finished_rows[iteration] = 0
                self.row_finish_time[iteration] = {row_id: None for row_id in self.graph.row_ids}

                self.unallocated_rows[iteration] = [] + self.graph.row_ids
                self.allocations[iteration] = {row_id: None for row_id in self.graph.row_ids}
                self.allocation_time[iteration] = {row_id: None for row_id in self.graph.row_ids}

                self.picker_allocations[iteration] = {picker_id: [] for picker_id in self.picker_ids}
                self.allocated_pickers[iteration] = []
            elif len(self.active_iterations) == 1 or self.active_iterations[-1] == self.n_iteration - 1:
                if self.finished_picking(self.n_iteration - 1):
                    self.active_iterations.remove(self.n_iteration - 1)

    def allocate_rows_to_pickers(self):
        """allocate unallocated_rows to idle_pickers based on scheduler_policy"""
        n_idle_pickers = len(self.idle_pickers)
        if n_idle_pickers > 0:
            # find active rows
            active_rows = []
            for iteration in self.active_iterations:
                for picker_id in self.picker_ids:
                    if picker_id not in self.idle_pickers and self.pickers[picker_id].curr_iteration == iteration:
                        active_rows.append(self.curr_picker_allocations[picker_id][1])

            for iteration in self.active_iterations:
                allocated_rows = []
                # row allocation is assumed to be "lexicographical"
                self.idle_pickers.sort()
                i = 0
                for row_id in self.unallocated_rows[iteration]:
                    # someone else is picking in this row from another iteration
                    if row_id in active_rows:
                        continue
                    # row can be allocated if there is a picker available
                    selected_picker_id = None
                    for picker_id in self.idle_pickers:
                        # if the current rows have not finished harvesting, allocate picking task to idle picker
                        if self.pickers[picker_id].curr_iteration == iteration:
                            selected_picker_id = picker_id
                            break
                    if selected_picker_id is not None:
                        self.allocate(iteration, row_id, picker_id)
                        allocated_rows.append(row_id)
                        if i == n_idle_pickers - 1:
                            break
                        else:
                            i += 1

                for row_id in allocated_rows:
                    self.unallocated_rows[iteration].remove(row_id)

    def allocate(self, iteration, row_id, picker_id):
        """allocate a picker to a row

        Keyword arguments:
        iteration - ? # the index of allocating-field time, one iteration means finishing the picking of all rows
        row_id - row to be allocated to the picker
        picker_id - id of the picker to be allocated to the row
        """
        # allocate row_id to the picker
        self.allocations[iteration][row_id] = picker_id
        self.picker_allocations[iteration][picker_id].append(row_id)
        self.curr_picker_allocations[picker_id] = (iteration, row_id)

        self.idle_pickers.remove(picker_id)
        self.allocated_pickers[iteration].append(picker_id)

        self.pickers[picker_id].allocate_row_to_picker(row_id)

        self.allocation_time[iteration][row_id] = self.env.now

        # update unallocated rows in predictor
        # TODO: predictor needs to take care of the iterations?
        # self.predictor.update_unallocated(row_id)

        self.loginfo("%s is allocated to %s in iteration %d at %0.3f" % (picker_id,
                                                                         row_id,
                                                                         iteration,
                                                                         self.allocation_time[iteration][row_id]))
