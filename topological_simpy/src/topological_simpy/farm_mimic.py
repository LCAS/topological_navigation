#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import topological_simpy.farm


class FarmMimic(topological_simpy.farm.Farm):
    """Farm class definition"""
    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, with_robots, n_iteration=0, verbose=False):
        """Create a Farm object

        Keyword arguments:
        name -- name of farm/poly-tunnel
        env -- simpy.Environment
        n_topo_nav_rows -- navigational rows in topological map
        topo_graph -- TopologicalForkMap object
        robots - robot agent objects
        pickers -- picker agent objects
        policy -- "lexicographical", "shortest_distance", "uniform_utilisation"

        TODO
        with_robots -- with robots in the field when picking
        n_iteration -- ?
        verbose -- ?
        """
        super(FarmMimic, self).__init__(name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose)

        self.with_robots = with_robots

        self.n_iteration = n_iteration

        self.active_iterations = []     # TODO why list?
        # track the curr_iter of each picker in the picker itself

        self.finished_rows = {}
        self.n_finished_rows = {}
        self.row_finish_time = {}

        # related to allocation
        self.unallocated_rows = {}
        self.allocations = {}
        self.allocation_time = {}

        self.picker_allocations = {}
        self.curr_picker_allocations = {picker_id:(None, None) for picker_id in self.picker_ids}
        self.allocated_pickers = {}

        self.finished_iterations = False

        self.finished_picking = lambda iteration: True if self.n_finished_rows[iteration] == self.n_topo_nav_rows else False
#        self.finished_allocating = lambda iteration: False if self.unallocated_rows[iteration] else True
        self.finished_allocating = lambda iteration: True if len(self.unallocated_rows[iteration]) == 0 else False

        self.action = self.env.process(self.scheduler_monitor())

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
            if (len(self.active_iterations) == 0 or self.active_iterations[-1] < self.n_iteration - 1):
                if len(self.active_iterations) == 0:
                    iteration = 0
                else:
                    iteration = self.active_iterations[-1] + 1
                self.active_iterations.append(iteration)

                self.finished_rows[iteration] = []
                self.n_finished_rows[iteration] = 0
                self.row_finish_time[iteration] = {row_id:None for row_id in self.graph.row_ids}

                self.unallocated_rows[iteration] = [] + self.graph.row_ids
                self.allocations[iteration] = {row_id:None for row_id in self.graph.row_ids}
                self.allocation_time[iteration] = {row_id:None for row_id in self.graph.row_ids}

                self.picker_allocations[iteration] = {picker_id:[] for picker_id in self.picker_ids}
                self.allocated_pickers[iteration] = []
            elif (len(self.active_iterations) == 1 or self.active_iterations[-1] == self.n_iteration - 1):
                if self.finished_picking(self.n_iteration - 1):
                    self.active_iterations.remove(self.n_iteration - 1)

    def scheduler_monitor(self):
        """A process to allocate rows to the pickers.
        the picker should request for a row or
        when a picker becomes free, it should be allocated automatically.

        A simple implementation:
            1. do a periodic checking of completion status of rows
            2. allocate free pickers to one of the unallocated rows
        """
        inform_allocation_finished = False
        inform_picking_finished = False
        while True:
            if rospy.is_shutdown():   # TODO: comment out
                break

            # increment curr_iteration of all idle pickers by one, if allocation for that iter is finished
            for picker_id in self.picker_ids:
                if self.pickers[picker_id].curr_iteration is None:   # Never iterated
                    self.pickers[picker_id].curr_iteration = 0
                elif (picker_id in self.idle_pickers and
                    self.pickers[picker_id].curr_iteration < self.active_iterations[-1]):
                        # if allocation is finished for the iteration of the picker, increment it
                        if self.finished_allocating(self.pickers[picker_id].curr_iteration):
                            self.pickers[picker_id].curr_iteration += 1

            # try starting the next iteration
            if len(self.active_iterations) == 0:  # TODO active_iterations means a whole picking activity, i.e., harvesting all the rows available?
                self.next_iteration()
            else:
                inc_iteration = True
                for iteration in self.active_iterations:
                    if not self.finished_allocating(iteration):  # Allocation is not done, stay at current active_iterations
                        inc_iteration = False
                        break
                if inc_iteration:
                    self.next_iteration()

            # all rows of all iterations are picked
            if len(self.active_iterations) == 0 and self.finished_picking(self.n_iteration-1) and not inform_picking_finished:
                inform_picking_finished = True
                self.loginfo("all rows are picked")
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_picking_finished()
                self.loginfo("all rows picked. scheduler exiting")
                self.env.exit("all rows are picked")
                break

            # all rows of all iterations are allocated, may not be finished
            if (self.n_iteration - 1) in self.active_iterations and self.finished_allocating(self.n_iteration-1) and not inform_allocation_finished:
                self.loginfo("all rows are allocated")
                inform_allocation_finished = True # do it only once
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_allocation_finished()

            for iteration in self.active_iterations:
                to_remove_pickers = []
                # update modes of pickers already assigned to a row
                for picker_id in self.allocated_pickers[iteration]:
                    picker = self.pickers[picker_id]
                    if picker.mode == 0:
                        # finished the assigned row and are idle now
                        # if previously assigned any row, update its status
                        row_id = self.curr_picker_allocations[picker_id][1]
                        self.finished_rows[iteration].append(row_id)    # TODO: just assigned to picker, finished assigning?
                        self.n_finished_rows[iteration] += 1
                        self.row_finish_time[iteration][row_id] = self.pickers[picker_id].row_finish_time
                        to_remove_pickers.append(picker_id)

                    elif picker.mode == 1:
                        # moving to a row_node possibly from the previous node
                        # this can happen either after a trip to a storage or after a new row allocation
                        # picker will be in picking mode (2) soon
                        pass

                    elif picker.mode == 2:
                        # picking now
                        pass

                    elif picker.mode == 3 or picker.mode == 4 or picker.mode == 6:
                        # picker transporting to storage or unloading at storage
                        # or transporting to local storage from cold storage
                        # if the current row is finished, the picker's mode will be changed
                        # to idle (0) soon, which will be taken care of in next loop
                        pass

                    elif picker.mode == 5:
                        # waiting for a robot to arrive
                        # if a robot is not assigned, coordinator will assign one
                        pass

                for picker_id in to_remove_pickers:
                    self.allocated_pickers[iteration].remove(picker_id)
                    if self.pickers[picker_id].mode == 0:
                        self.idle_pickers.append(picker_id)

            # row allocation to idle_pickers
            self.allocate_rows_to_pickers()

            yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)

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
                        if self.pickers[picker_id].curr_iteration == iteration:    # curr_iteration == iteration means the current rows have not finished harvesting, allocate picking task to idle picker
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
#        self.predictor.update_unallocated(row_id)

        self.loginfo("%s is allocated to %s in iteration %d at %0.3f" %(picker_id,
                                                                        row_id,
                                                                        iteration,
                                                                        self.allocation_time[iteration][row_id]))
