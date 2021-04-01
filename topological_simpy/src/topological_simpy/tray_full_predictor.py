#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import copy
import rospy
import numpy

import rasberry_des.picker_predictor


class TrayFullPredictor(object):
    """
    A class that can predict when pickers are going to finish their current tray
    """
    def __init__(self, picker_ids, env, topo_graph, n_robots, mean_idle_times, mean_tray_pick_dists,
                 mean_pick_rates, mean_trans_rates, mean_unload_times, mean_load_times, verbose=False):
        """
        initialise the TrayFullPredictor object

        Keyword arguments:

        picker_ids -- list of picker_ids
        env -- simpy.Environment object
        topo_graph -- rasberry_des.TopoGraph object
        n_robots -- number of robots
        """
        self.picker_ids = picker_ids
        self.env = env
        self.graph = topo_graph
        self.n_pickers = len(self.picker_ids)
        self.n_robots = n_robots
        self.verbose = verbose

        self._allocated_rows = []
        self._unallocated_rows = copy.deepcopy(self.graph.row_ids)
        self._free_rows = copy.deepcopy(self._unallocated_rows)

        # PickerPredictor is used as logging object for each picker
        self.predictors = {picker_id:rasberry_des.picker_predictor.PickerPredictor(picker_id, self.env,
                                                                                   self.graph, self.n_pickers, self.n_robots,
                                                                                   mean_idle_times[picker_id], mean_tray_pick_dists[picker_id],
                                                                                   mean_pick_rates[picker_id], mean_trans_rates[picker_id],
                                                                                   mean_unload_times[picker_id], mean_load_times[picker_id],
                                                                                   self.verbose) for picker_id in self.picker_ids}

        # prediction related
        self._picker_modes = None
        self._mode_start_times = None
        self._mode_finish_modes = None
        self._mode_finish_details = None
        self._curr_nodes = None
        self._curr_rows = None
        self._goal_nodes = None
        self._dists_picked = None
        self._update_needed = None

    def set_initial_mode_and_pose(self, picker_id, mode, node, direction=None):
        """call the set_initial_mode_and_pose method of the picker with picker_id
        """
        self.predictors[picker_id].set_initial_mode_and_pose(mode, node, direction)

    def update_mode_and_pose(self, picker_id, mode, node, direction=None, goal_node=None):
        """call the update_mode_and_pose method of the picker with picker_id
        """
        self.predictors[picker_id].update_mode_and_pose(mode, node, direction, goal_node)

    def update_unallocated(self, row_id):
        """remove a row_id from unallocated when a row is actually allocated to a picker
        """
        self._unallocated_rows.remove(row_id)

    def predict_tray_full(self):
        """predict when and where the pickers (who are in the picking mode now) will have their
        trays full.

        TODO: current implementation assumes clear sequeces when modes are changed

        assuming mode changes happen only when the current mode is finished normally,
        e.g. picking - tray_full or row_finish; idle - new_alloc.
        when needed based on the MDP probability to switch to another mode from current,
        predict the next mode.
        """

        time_org = time_now = self.env.now

        # start fresh
        self._picker_modes = {picker_id:self.predictors[picker_id].mode[-1] for picker_id in self.picker_ids}
        self._mode_start_times = {picker_id:self.predictors[picker_id].mode_start_times[-1] for picker_id in self.picker_ids}
        self._mode_finish_modes = {picker_id:None for picker_id in self.picker_ids}
        self._mode_finish_details = {picker_id:None for picker_id in self.picker_ids}
        self._curr_nodes = {picker_id:self.predictors[picker_id].curr_node for picker_id in self.picker_ids}
        self._curr_dirs = {picker_id:self.predictors[picker_id].curr_dir for picker_id in self.picker_ids}
        self._curr_rows = {picker_id:self.predictors[picker_id].curr_row for picker_id in self.picker_ids}
        self._goal_nodes = {picker_id:self.predictors[picker_id].goal_node for picker_id in self.picker_ids}
        self._dists_picked = {picker_id:None for picker_id in self.picker_ids}
        self._update_needed = {picker_id:True for picker_id in self.picker_ids}
        self._tray_started = {picker_id:False for picker_id in self.picker_ids}

        self._free_rows = copy.deepcopy(self._unallocated_rows)

        picking_pickers = [] # once tray_full predictions of all these pickers are finished, break

        for picker_id in self.picker_ids:
            # not all pickers who started a new tray is in [2] mode (e.g. could be idle/go_to_row_node etc.)
            if self.predictors[picker_id].has_started_a_tray():
                # tray_start index must be > tray_stop index for the picker to be filling up a tray
                picking_pickers.append(picker_id)
                self._dists_picked[picker_id] = self.predictors[picker_id].get_dist_picked_so_far()
            else:
                self._dists_picked[picker_id] = 0.0

        predictions = {}
        # in a loop virtually populate MDP
        while True:
            if rospy.is_shutdown():
                predictions = {}
                break

            # update mode finish details
            self._update_mode_finishes(time_now)

            # get minimum event time and all pickers changing modes at that time
            min_picker_ids, min_event_time = self._get_min_event_time(time_now)

            if len(picking_pickers) == 0:
                # predicted for all picking_pickers
                break
            elif min_event_time == float("inf"):
                # all pickers at storage and all rows are picked
                break

            time_now = min_event_time

            try:
                assert time_now > time_org or numpy.isclose(time_now, time_org)
            except:
                msg = "%0.2f, %0.2f" %(time_now, time_org)
                raise Exception(msg)

            mode_0_pickers = []
            mode_1_pickers = []
            mode_2_pickers = []
            mode_3_pickers = []
            mode_4_pickers = []
            mode_5_pickers = []
            mode_6_pickers = []
            for picker_id in min_picker_ids:
                # depending on the future mode of the min_pickers, group them
                if self._mode_finish_modes[picker_id] == 0:
                    mode_0_pickers.append(picker_id)
                elif self._mode_finish_modes[picker_id] == 1:
                    mode_1_pickers.append(picker_id)
                elif self._mode_finish_modes[picker_id] == 2:
                    mode_2_pickers.append(picker_id)
                elif self._mode_finish_modes[picker_id] == 3:
                    mode_3_pickers.append(picker_id)
                elif self._mode_finish_modes[picker_id] == 4:
                    mode_4_pickers.append(picker_id)
                elif self._mode_finish_modes[picker_id] == 5:
                    mode_5_pickers.append(picker_id)
                elif self._mode_finish_modes[picker_id] == 6:
                    mode_6_pickers.append(picker_id)

            # set the min_pickers' mode to their future modes
            for picker_id in mode_0_pickers:
                self._start_idling(picker_id, time_now)

            self._row_allocation(mode_1_pickers, time_now)

            for picker_id in mode_2_pickers:
                self._start_picking(picker_id, time_now)

            for picker_id in mode_3_pickers:
                # go to storage - (tray_full and no robots) or (tray_full/partial and last row)
                self._go_to_storage(picker_id, time_now)
                # store prediction and remove from picking_pickers
                if picker_id in picking_pickers:
                    predictions[picker_id] = ["go_to_storage",
                                              self._curr_rows[picker_id],
                                              self._curr_nodes[picker_id],
                                              self._curr_dirs[picker_id],
                                              time_now]

                    picking_pickers.remove(picker_id)
                    msg = "removing %s from picking_pickers: [3]" %(picker_id)
                    self.loginfo(msg)

            for picker_id in mode_4_pickers:
                self._start_unloading(picker_id, time_now)

            for picker_id in mode_5_pickers:
                # waiting for loading - tray_full with robots
                self._start_loading(picker_id, time_now)
                # store prediction and remove from picking_pickers
                if picker_id in picking_pickers:
                    predictions[picker_id] = ["wait_loading",
                                              self._curr_rows[picker_id],
                                              self._curr_nodes[picker_id],
                                              self._curr_dirs[picker_id],
                                              time_now]
                    picking_pickers.remove(picker_id)
                    msg = "removing %s from picking_pickers: [5]" %(picker_id)
                    self.loginfo(msg)

            for picker_id in mode_6_pickers:
                self._go_to_local_from_cold(picker_id, time_now)
        return predictions

    def _update_mode_finishes(self, time_now):
        """virtually updating mode finish modes and details for all pickers based on their
        current state
        """
        # picker modes
        # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
        # 4: waiting for unload at storage, 5: waiting for loading on robot
        # 6: transporting to local storage from cold storage
        # if a picker is in the mode
        #   [0] idle, he will switch to go_to_row_node [1] after being allocated to a new row.
        #   [1] go_to_row_node, he will be starting/resuming picking [2] soon - need goal pose (node and dir)
        #   [2] picking, he will either have his tray full or pick the full row. possible next states - go_to_storage [3] (without robots) or wait_for_robot [5]
        #   [3] go_to_storage, he will be reaching storage for unloading [4]
        #   [4] wait_unloading, he will be switching to go_to_row_node [1] after unloading trays
        #   [5] wait_loading, he is waiting for robot and for loading trays on it. next states idle [0] (if curr_node = row_finish_node) or picking [2]
        #   [6] go_to_local_from_cold, he will be reaching local storage and switch to idle [0]
        for picker_id in self.picker_ids:
            msg = "%s, %s, %d" %(picker_id, self._update_needed[picker_id], self._picker_modes[picker_id])
            self.loginfo(msg)

            if not self._update_needed[picker_id]:
                # update mode only if needed
                continue

            if self._picker_modes[picker_id] == 0:
                # idle will finish in idle mode if no more rows are unallocated
                self._mode_finish_details[picker_id] = [self._curr_nodes[picker_id], None,
                                                        self._mode_start_times[picker_id] + self.predictors[picker_id].mean_idle_time(),
                                                        0.0]
                # if the picker had started a tray and became idle, we should send him to storage to unload partial tray
                if len(self._free_rows) == 0:
                    # if there are no unallocated rows, and not at storage, go to storage
                    storage = self.graph.local_storage_nodes[self._curr_rows[picker_id]] if self.graph.use_local_storage else self.graph.cold_storage_node
                    if self._curr_nodes[picker_id] == storage:
                        # at storage, no free rows and already idle, setting mode_finish_time
                        # as inf so that this picker will be ignored in min_event_time
                        self._mode_finish_modes[picker_id] = 0
                        self._mode_finish_details[picker_id][2] = float("inf")
                    else:
                        self._mode_finish_modes[picker_id] = 3
                else:
                    self._mode_finish_modes[picker_id] = 1
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 1:
                # started this mode earlier. will be picking after this
                # could be to start node of a row or to resume picking from an
                # intermediate row node
                self._mode_finish_details[picker_id] = [self._goal_nodes[picker_id], "forward",
                                                        time_now + self.predictors[picker_id].get_trans_time_to_node(self._curr_nodes[picker_id], self._goal_nodes[picker_id]),
                                                        0.0]
                self._mode_finish_modes[picker_id] = 2
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 2:
                # two possible events - tray_full -> [3/5] and row_finish -> [0/3]
                next_event, event_details = self.predictors[picker_id].predict_picking_finish_event(self._curr_nodes[picker_id],
                                                                                                    self._curr_dirs[picker_id],
                                                                                                    self._dists_picked[picker_id],
                                                                                                    time_now)

                self._mode_finish_details[picker_id] = event_details # full details
                if next_event == "tray_full":
                    self._mode_finish_modes[picker_id] = 3 if self.n_robots==0 else 5
                elif next_event == "row_finish":
                    self._mode_finish_modes[picker_id] = 0
                else:
                    raise Exception("event must be tray_full or row_finish")

                self._dists_picked[picker_id] = self._mode_finish_details[picker_id][3]
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 3:
                # next mode - unload_at_storage [4]
                # dist from curr_node to storage / trans_rate
                # storage node is already set as goal_node
                self._mode_finish_details[picker_id] = [self._goal_nodes[picker_id], None,
                                                        time_now + self.predictors[picker_id].get_trans_time_to_node(self._curr_nodes[picker_id], self._goal_nodes[picker_id]),
                                                        0.0]
                self._mode_finish_modes[picker_id] = 4
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 4:
                # next mode - idle [0], go_to_row_node [1] or go_to_local_from_cold [6]
                self._mode_finish_details[picker_id] = [self._curr_nodes[picker_id], None,
                                                        time_now + self.predictors[picker_id].mean_unload_time(),
                                                        0.0]
                self._dists_picked[picker_id] = 0.0

                if len(self._free_rows) == 0:
                    self._mode_finish_modes[picker_id] = 0
                elif self.graph.use_local_storage:
                    # unloading at local -> idle
                    self._mode_finish_modes[picker_id] = 0
                else:
                    # unloading at cold -> go_to_local_from_cold
                    self._mode_finish_modes[picker_id] = 6
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 5:
                # next mode - idle [0] or picking [2]
                # if row is finished and no rows left for allocation -> idle [0]
                # else continue picking
                if self._curr_rows[picker_id] in self.graph.half_rows:
                    row_finish_node = self.graph.row_info[self._curr_rows[picker_id]][2]
                else:
                    row_finish_node = self.graph.row_info[self._curr_rows[picker_id]][1]
                self._mode_finish_details[picker_id] = [self._curr_nodes[picker_id], self._curr_dirs[picker_id],
                                                        time_now + self.predictors[picker_id].mean_unload_time(),
                                                        0.0]
                self._dists_picked[picker_id] = 0.0

                if len(self._free_rows) == 0 and self._curr_nodes[picker_id] == row_finish_node:
                    self._mode_finish_modes[picker_id] = 0
                else:
                    self._mode_finish_modes[picker_id] = 2
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 6:
                # next mode -- idle[0] at local storage
                self._mode_finish_details[picker_id] = [self.graph.local_storage_nodes[self._curr_rows[picker_id]], None,
                                                        time_now + self.predictors[picker_id].get_trans_time_to_node(self._curr_nodes[picker_id], self.graph.local_storage_nodes[self._curr_rows[picker_id]]),
                                                        0.0]
                self._mode_finish_modes[picker_id] = 0
                self._update_needed[picker_id] = False

    def _get_min_event_time(self, time_now):
        """_get_min_event_time: return the picker_id and the minimum next_event_time among all pickers

        Keyword arguments:

        time_now -- current time sample
        """
        min_event_time = float("inf")
        for picker_id in self.picker_ids:
            # ignore pickers with empty mode_finish_details
            if len(self._mode_finish_details[picker_id]) == 0:
                continue

            # This is a fake correction on predicted time by moving prediction to time_now
            # This happens when an event predicted to happen in the past has not
            # happened yet. As this can affect the MDP progression, a fix to the
            # predicted_time is done.
            # This could happen for predictions in any mode as the predictions are based on
            # time at which the mode started and mean time for that mode, which could be less
            # than what the picker is taking in the current execution.
            if self._mode_finish_details[picker_id][2] < time_now:
                self._mode_finish_details[picker_id][2] += time_now - self._mode_finish_details[picker_id][2]

            if self._mode_finish_details[picker_id][2] < min_event_time:
                min_event_time = self._mode_finish_details[picker_id][2]

        min_picker_ids = []
        for picker_id in self.picker_ids:
            # ignore pickers with empty mode_finish_details
            if len(self._mode_finish_details[picker_id]) == 0:
                continue
            if self._mode_finish_details[picker_id][2] == min_event_time:
                min_picker_ids.append(picker_id)
        return (min_picker_ids, min_event_time)

    def _start_idling(self, picker_id, time_now):
        """virtual idling - set picker's mode to [0]
        """
        self._picker_modes[picker_id] = 0
        self._curr_nodes[picker_id] = self._mode_finish_details[picker_id][0]
        self._curr_dirs[picker_id] = self._mode_finish_details[picker_id][1]
        self._mode_start_times[picker_id] = time_now
        self._update_needed[picker_id] = True

    def _row_allocation(self, picker_ids, time_now):
        """virtual row allocation to pickers - set picker's mode to [1]
        """
        # row allocation is using lexicographical order
        picker_ids.sort()
        self._free_rows.sort()
        unallocated_pickers = copy.deepcopy(picker_ids)
        for picker_id in picker_ids:
            self.loginfo(self._free_rows)
            if len(self._free_rows) > 0:
                row_id = self._free_rows[0]
                msg = "allocating %s to %s" %(row_id, picker_id)
                self.loginfo(msg)
                self._go_to_row_node(row_id, picker_id, time_now)
                unallocated_pickers.remove(picker_id)

        for picker_id in unallocated_pickers:
            # no unallocated rows for these pickers.
            # last tray for the picker. send him to storage
            self._mode_finish_modes[picker_id] = 3
            self._dists_picked[picker_id] = 0.
            self._update_needed[picker_id] = True

    def _go_to_row_node(self, row_id, picker_id, time_now):
        """virtually start go_to_row_node of a picker to the start of new row after allocation
        """
        self._allocated_rows.append(row_id)
        self._free_rows.remove(row_id)

        self._picker_modes[picker_id] = 1
        self._curr_nodes[picker_id] = self._mode_finish_details[picker_id][0]
        self._curr_dirs[picker_id] = self._mode_finish_details[picker_id][1]
        self._goal_nodes[picker_id] = self.graph.row_info[row_id][1]
        self._mode_start_times[picker_id] = time_now
        self._update_needed[picker_id] = True

    def _start_picking(self, picker_id, time_now):
        """virtual start of picking to a picker- set picker's mode to [2]
        """
        self._picker_modes[picker_id] = 2
        self._curr_nodes[picker_id] = self._mode_finish_details[picker_id][0]
        self._curr_dirs[picker_id] = self._mode_finish_details[picker_id][1]
        self._mode_start_times[picker_id] = time_now
        self._update_needed[picker_id] = True

    def _go_to_storage(self, picker_id, time_now):
        """virtual go to storage - set picker's mode to [3]
        """
        self._picker_modes[picker_id] = 3
        self._curr_nodes[picker_id] = self._mode_finish_details[picker_id][0]
        self._curr_dirs[picker_id] = self._mode_finish_details[picker_id][1]
        if self.graph.use_local_storage:
            self._goal_nodes[picker_id] = self.graph.local_storage_nodes[self._curr_rows[picker_id]]
        else:
            self._goal_nodes[picker_id] = self.graph.cold_storage_node
        self._mode_start_times[picker_id] = time_now
        self._update_needed[picker_id] = True

    def _start_unloading(self, picker_id, time_now):
        """virtual unloading at storage - set picker's mode to [4]
        """
        self._picker_modes[picker_id] = 4
        self._curr_nodes[picker_id] = self._mode_finish_details[picker_id][0]
        self._curr_dirs[picker_id] = self._mode_finish_details[picker_id][1]
        self._mode_start_times[picker_id] = time_now
        self._update_needed[picker_id] = True

    def _start_loading(self, picker_id, time_now):
        """virtual loading on robot - set picker's mode to [5]
        """
        self._picker_modes[picker_id] = 5
        self._curr_nodes[picker_id] = self._mode_finish_details[picker_id][0]
        self._curr_dirs[picker_id] = self._mode_finish_details[picker_id][1]
        self._mode_start_times[picker_id] = time_now
        self._update_needed[picker_id] = True

    def _go_to_local_from_cold(self, picker_id, time_now):
        """virtual go to local storage from cold storage - set picker's mode to [6]
        """
        self._picker_modes[picker_id] = 6
        self._curr_nodes[picker_id] = self._mode_finish_details[picker_id][0]
        self._curr_dirs[picker_id] = self._mode_finish_details[picker_id][1]
        self._goal_nodes[picker_id] = self.graph.local_storage_nodes[self._curr_rows[picker_id]]
        self._mode_start_times[picker_id] = time_now
        self._update_needed[picker_id] = True

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            rospy.loginfo(msg)
