#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import numpy
import rospy


class PickerPredictor(object):
    """A class to predict a picker's actions and positions"""
    def __init__(self, picker_id, env, topo_graph, n_pickers, n_robots, mean_idle_time, mean_tray_pick_dist,
                 mean_pick_rate, mean_trans_rate, mean_unload_time, mean_load_time, verbose=False):
        """Initialiser for PickerPredictor

        Keyword arguments:

        picker_id -- id of the picker being tracked
        env -- simpy environment object
        topo_graph -- topo_graph object
        n_picker -- number of pickers
        n_robots -- number of robots
        mean_idle_time -- average time the picker spends in idle mode (to be used only before getting any logged data)
        mean_tray_pick_dist -- mean distance to be picked for filling a tray (to be used only before getting any logged data)
        mean_pick_rate -- picker's mean rate of travel while picking (to be used only before getting any logged data)
        mean_trans_rate -- picker's transportation rate when not picking (to be used only before getting any logged data)
        mean_unload_time -- time a picker may spend for unloading a tray at storage (to be used only before getting any logged data)
        mean_load_time -- time a picker may spend for loading a tray on a robot (to be used only before getting any logged data)
        verbose -- to print detailed logs or not
        """
        self.picker_id = picker_id

        self.env = env
        self.graph = topo_graph
        self.n_pickers = n_pickers
        self.n_robots = n_robots
        self.verbose = verbose

        self.curr_row = None
        self.curr_node = None
        self.curr_dir = None
        self.goal_node = None

        # picker modes
        # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
        # 4: waiting for unload at storage, 5: waiting for loading on robot
        # 6: transporting to local storage from cold storage
        # NOTE: stores modes of the picker - mainly for monitoring mode changes
        # cannot store all continuous repetitive modes, as it will make it difficult to track
        # mode changes for detecting tray start or tray full events
        self.mode = []                  # list of modes
        self.mode_start_times = []       # time at which this mode is started
        self.mode_stop_times = []        # time at which this mode is finished
        self.mode_start_poses = []       # (node_id, direction) at which the mode started
        self.mode_stop_poses = []        # (node_id, direction) at which the mode stopped

        self.mode_index_tray_start = [] # index of the mode (in self.modes) at which a tray picking is started
        self.mode_index_tray_stop = []  # index of the mode (in self.modes) at which a tray picking is finished
        self.picking_dists_per_tray = [] # distance travelled for filling a tray (only picking mode)
        self.picking_times_per_tray = [] # time taken to fill a tray (only picking mode)

        self.picker_modes = [0, 1, 2, 3, 4, 5, 6]
        self.picker_modes_str = {0:"idle", 1:"go_to_row_node", 2:"picking",
                                 3:"go_to_storage", 4:"wait_unloading",
                                 5:"wait_loading", 6:"go_to_local_from_cold"}
        self.n_picker_modes = len(self.picker_modes)
        # to calculate MDP probability of transition from one mode to another
        self.mode_transition = numpy.zeros((self.n_picker_modes, self.n_picker_modes))
        self.time_spent_in_mode = {mode:[] for mode in self.picker_modes}

        # detailed monitoring.
        self.modes_nodes_dirs_times = [] # [(mode, node, dir, time), ...]

        self.picking_dists = [] # picking distances
        self.picking_times = [] # picking times
        self.trans_dists = [] # transportation distances
        self.trans_times = [] # transportation times

        # all lambda functions below to get mean of an array would return 0. if the array is empty

        self.mean_idle_time = lambda: numpy.mean(self.time_spent_in_mode[0]) if numpy.size(self.time_spent_in_mode[0]) != 0. else mean_idle_time
        self.mean_tray_pick_dist = lambda: numpy.mean(self.picking_dists_per_tray) if numpy.size(self.picking_dists_per_tray) != 0. else mean_tray_pick_dist

        # mean picking rate (speed while picking)  - per tray or for individual row edges?
#        # per tray - problem is update frequency is low
#        self.mean_pick_rate = lambda: numpy.mean(numpy.array(self.picking_dists_per_tray) / numpy.array(self.picking_times_per_tray)) if numpy.size(self.picking_dists_per_tray) != 0. else mean_pick_rate
        # per individual row edges
        # - considering picking rate along an edge as a distribution of RV
        self.mean_pick_rate = lambda: numpy.mean(numpy.array(self.picking_dists) / numpy.array(self.picking_times)) if numpy.size(self.picking_dists) != 0. else mean_pick_rate
#        # - from total distance and time
#        self.mean_pick_rate = lambda: sum(self.picking_dists) / sum(self.picking_times) if numpy.size(self.picking_dists) != 0. else mean_pick_rate

        # mean_trans_rate
        # there will be at least one transportation mode before first picking
        # - cosnidering trans_rate as taken from a distribution of RV
        self.mean_trans_rate = lambda: numpy.mean(numpy.array(self.trans_dists) / numpy.array(self.trans_times)) if numpy.size(self.trans_dists) != 0. else mean_trans_rate
#        # - from total distance and time
#        self.mean_trans_rate = lambda: sum(self.trans_dists) / sum(self.trans_times) if numpy.size(self.trans_dists) != 0. else mean_trans_rate

        # unloading time at local storage (mode 4)
        self.unload_times = []
        self.mean_unload_time = lambda: numpy.mean(numpy.array(self.unload_times)) if numpy.size(self.unload_times) != 0. else mean_unload_time

        # loading time on robots (mode 5) - this also includes the time for the robot to arrive
        self.load_times = []
        self.mean_load_time = lambda: numpy.mean(numpy.array(self.load_times)) if numpy.size(self.load_times) != 0. else mean_load_time

        # TODO: Pickers are assumed to start at the local storage of first row in idle mode
        self.set_initial_mode_and_pose(0, self.graph.local_storage_nodes[self.graph.row_ids[0]])

    def set_initial_mode_and_pose(self, mode, node, direction=None):
        """set the initial mode and pose of the picker

        Keyword arguments:

        mode - current mode of the picker
        node - current node of the picker
        direction - picking direction of the picker
        """
        self.mode.append(mode)
        time_now = self.env.now
        self.mode_start_times.append(time_now)
        self.mode_start_poses.append((node, direction))
        self.curr_node = node
        self.modes_nodes_dirs_times.append((mode, node, direction, time_now))

    def update_mode_and_pose(self, mode, node, direction=None, goal_node=None):
        """update the current mode of the picker

        Keyword arguments:

        mode - current mode of the picker
        node - current node of the picker
        direction - picking direction of the picker
        """
        if (not (self.modes_nodes_dirs_times[-1][0] == mode) or
            not (self.modes_nodes_dirs_times[-1][1] == node) or
            not (self.modes_nodes_dirs_times[-1][2] == direction)):
            # picker state (node or mode or dir) is changed
            # update time
            time_now = self.env.now
            # update modes_nodes_dirs_times
            self.modes_nodes_dirs_times.append((mode, node, direction, time_now))
#==============================================================================
#             # update current pose
#==============================================================================
            if self.curr_node != node:
                self.curr_node = node
            if self.curr_dir != direction:
                self.curr_dir = direction

#==============================================================================
#             # if there is no mode change - detailed picking monitoring for better estimates
#==============================================================================
            if self.modes_nodes_dirs_times[-2][0] == self.modes_nodes_dirs_times[-1][0]:
                # current and previous modes are the same
                if self.modes_nodes_dirs_times[-2][1] != self.modes_nodes_dirs_times[-1][1]:
                    # current and previous nodes are different
                    _, _, route_dists = self.graph.get_path_details(self.modes_nodes_dirs_times[-2][1], self.modes_nodes_dirs_times[-1][1])
                    if self.mode[-1] == 2:
                        # picking
                        self.picking_dists.append(sum(route_dists))
                        self.picking_times.append(self.modes_nodes_dirs_times[-1][3] - self.modes_nodes_dirs_times[-2][3])

                    elif self.mode[-1] == 1 or self.mode[-1] == 3 or self.mode[-1] == 6:
                        # transportation
                        self.trans_dists.append(sum(route_dists))
                        self.trans_times.append(self.modes_nodes_dirs_times[-1][3] - self.modes_nodes_dirs_times[-2][3])

#==============================================================================
#             # if there a mode change?
#==============================================================================
            elif self.modes_nodes_dirs_times[-2][0] != self.modes_nodes_dirs_times[-1][0]:
                # mode is changed
                # close out previous mode related things
                self.mode_stop_times.append(time_now)
                self.mode_stop_poses.append((node, direction))

                # new mode related things
                self.mode.append(mode)
                self.mode_start_times.append(time_now)
                self.mode_start_poses.append((node, direction))

                if self.modes_nodes_dirs_times[-2][1] != self.modes_nodes_dirs_times[-1][1]:
                    # if the node has been changed
                    _, _, route_dists = self.graph.get_path_details(self.modes_nodes_dirs_times[-2][1], self.modes_nodes_dirs_times[-1][1])
                    # picking related
                    if self.mode[-2] == 2:
                        # previously in picking mode, but now in another mode
                        self.picking_dists.append(sum(route_dists))
                        self.picking_times.append(self.modes_nodes_dirs_times[-1][3] - self.modes_nodes_dirs_times[-2][3])

                    # transportation related
                    if self.mode[-1] == 1 or self.mode[-1] == 3 or self.mode[-1] == 6:
                        # starting transportation
                        self.trans_dists.append(sum(route_dists))
                        self.trans_times.append(self.modes_nodes_dirs_times[-1][3] - self.modes_nodes_dirs_times[-2][3])
                    elif ((self.mode[-2] == 1 and self.mode[-1] == 2) or
                          (self.mode[-2] == 3 and self.mode[-1] == 4) or
                          (self.mode[-2] == 6 and self.mode[-1] == 0)):
                        # finishing transportation
                        self.trans_dists.append(sum(route_dists))
                        self.trans_times.append(self.modes_nodes_dirs_times[-1][3] - self.modes_nodes_dirs_times[-2][3])

                    # unloading related
                    if self.mode[-2] == 4 and (self.mode[-1] == 0 or self.mode[-1] == 1 or self.mode[-1] == 6):
                        # after unloadig modes -> idle, go_to_row_node or go_to_local_from_cold
                        self.unload_times.append(self.modes_nodes_dirs_times[-1][3] - self.modes_nodes_dirs_times[-2][3])

                    # loading related
                    if self.mode[-2] == 5 and (self.mode[-1] == 0 or self.mode[-1] == 2):
                        # after unloadig modes -> idle (if at row finish node) or picking
                        self.load_times.append(self.modes_nodes_dirs_times[-1][3] - self.modes_nodes_dirs_times[-2][3])

                # goal_pose in [1] go_to_row_node, [3] go_to_storage or [6] go_to_local_from_cold
                if self.mode[-1] == 1 or self.mode[-1] == 3 or self.mode[-1] == 6:
                    self.goal_node = goal_node

#==============================================================================
#                 # is this a new tray event?
#==============================================================================
                # TODO: assuming picker has only one tray
                # common: curr_mode = picking
                if self.mode[-1] == 2:
                    # first 0 -> 1 -> 2 is a tray start event
                    if len(self.mode) == 3:
                        self.mode_index_tray_start.append(len(self.mode) - 1)

                    # second tray onwards
                    elif len(self.mode) > 3:
                        # without robots
                        if self.n_robots == 0:
                            if ((self.mode[-4] == 3) and (self.mode[-3] == 4) and
                                (self.mode[-2] == 1)):
                                # tray full happened in between a row
                                # resuming picking after trays are unloaded at storage
                                self.mode_index_tray_start.append(len(self.mode) - 1)
                            elif ((self.mode[-6] == 3) and (self.mode[-5] == 4) and
                                  (self.mode[-4] == 6) and (self.mode[-3] == 0) and
                                  (self.mode[-2] == 1)):
                                # tray full happened at the end of previous row
                                # trays unloaded at cold -> go_to_local_from_cold -> idle -> go_to_row_node -> picking
                                self.mode_index_tray_start.append(len(self.mode) - 1)
                            elif ((self.mode[-5] == 3) and (self.mode[-4] == 4) and
                                  (self.mode[-3] == 0) and (self.mode[-2] == 1)):
                                # tray full happened at the end of previous row
                                # trays unloaded at local -> idle -> go_to_row_node -> picking
                                self.mode_index_tray_start.append(len(self.mode) - 1)

                        # with robots
                        else:
                            if self.mode[-2] == 5:
                                # tray full happened in between a row
                                # resuming picking after trays are loaded on robot
                                self.mode_index_tray_start.append(len(self.mode) - 1)
                            elif ((self.mode[-4] == 5) and (self.mode[-3] == 0) and
                                  (self.mode[-2] == 1)):
                                # tray full happened at the end of previous row
                                # trays loaded on robot -> idle -> go_to_row_node -> picking
                                self.mode_index_tray_start.append(len(self.mode) - 1)

#==============================================================================
#                 # is this a tray full event?
#==============================================================================
                # with robots: wait_load_robot
                # without robots: go_to_storage
                if self.mode[-1] == 3 or self.mode[-1] == 5:
                    # tray_full happened at the end of previous mode
                    # the mode end time is the time at which next mode starts
                    self.mode_index_tray_stop.append(len(self.mode) - 2)

                    # update distance and time for this tray
                    picking_time = picking_dist = 0.0
                    # 1. no intermediate mode changes (idx = self.mode_index_tray_start[-1])
                    # 2. with intermediate mode changes (e.g. new row)
                    # loop through the modes & add distance and time, while in picking mode
                    for idx in range(self.mode_index_tray_start[-1], self.mode_index_tray_stop[-1] + 1):
                        if self.mode[idx] == 2:
                            # only if this is picking mode
                            picking_time += self.mode_stop_times[idx] - self.mode_start_times[idx]
                            picking_dist += self.get_picking_distance(self.mode_start_poses[idx],
                                                                      self.mode_stop_poses[idx])

                    self.picking_times_per_tray.append(picking_time)
                    self.picking_dists_per_tray.append(picking_dist)

#==============================================================================
#                  # is this a new allocation?
#==============================================================================
                if (((self.mode[-2] == 1) and (self.mode[-1] == 2)) or
                    ((self.mode[-2] == 5) and (self.mode[-1] == 2))):
                    # reached a row_node and started picking
                    # get_row_id_of_row_node() could return None, but not here as the curr_node is a row_node
                    row_id = self.graph.get_row_id_of_row_node(self.curr_node)
                    if (self.curr_row is None) or (self.curr_row != row_id):
                        self.curr_row = row_id

#==============================================================================
#                 # mode transitions and time spent in each mode
#==============================================================================
                self.mode_transition[self.mode[-2]][self.mode[-1]] += 1
                self.time_spent_in_mode[self.mode[-2]].append(self.mode_stop_times[-1] - self.mode_start_times[-2])

    def get_picking_distance(self, start_pose, stop_pose):
        """given poses at both start and stop, return the distance spent in the picking mode.
        assumes the mode is picking mode (i.e. start and stop poses
        of another mode is not supposed to be used with this method)

        Keyword arguments:

        start_pose - pose at the start of mode (node, dir)
        stop_pose - pose at the finish of mode (node, dir)
        """
        row_id = self.graph.get_row_id_of_row_node(start_pose[0])

        try:
            assert row_id == self.graph.get_row_id_of_row_node(stop_pose[0])
        except:
            raise Exception("row_id == self.graph.get_row_id_of_row_node(stop_pose[0])")

        picking_dist = 0.0

        # row_id != None as the start/stop node will be a row_node
        if row_id in self.graph.half_rows:
            # row_end and dir_change nodes are the same
            row_end_node = self.graph.row_info[row_id][2]
            dir_change_node = self.graph.row_info[row_id][2]
        else:
            row_end_node = self.graph.row_info[row_id][1]
            dir_change_node = self.graph.row_info[row_id][2]

        # distance
        # is there a direction change?
        if stop_pose[1] is None:
            # stop_pose (may be with tray_full) at end_of_row
            # could be the start_row_node or last_row_node (depending on half/full row)
            if start_pose[1] == "reverse":
                # row_end_node is row_start_node
                picking_dist += sum(self.graph.get_path_details(start_pose[0], row_end_node)[2])
            else:
                # start_pose[1] = "forward"
                dist_forward = dist_reverse = 0.0
                if row_id in self.graph.half_rows:
                    dist_forward = sum(self.graph.get_path_details(start_pose[0], row_end_node)[2])
                else:
                    dist_forward = sum(self.graph.get_path_details(start_pose[0], dir_change_node)[2])
                    dist_reverse = sum(self.graph.get_path_details(dir_change_node, row_end_node)[2])
                picking_dist += dist_forward + dist_reverse

        elif stop_pose[1] == start_pose[1]:
            # tray_full not at end_of_row, but happened in the same direction
            # both start and finish directions are either forward or reverse
            picking_dist += sum(self.graph.get_path_details(start_pose[0], stop_pose[0])[2])

        elif stop_pose[1] == "reverse":
            # tray_full not at end_of_row, but happened in different directions
            # only possibility is start_direction is forward and stop_direction is reverse
            # so this row can't be a half_row
            dist_forward = sum(self.graph.get_path_details(start_pose[0], dir_change_node)[2])
            dist_reverse = sum(self.graph.get_path_details(dir_change_node, stop_pose[0])[2])
            picking_dist += dist_forward + dist_reverse

        return picking_dist

    def predict_current_tray_full(self):
        """predict where and when the current tray could be full.
        it assumes the next row (if the tray won't be full in the current row)
        is curr_row + n_pickers, as the predictions are based only on individual picker's info
        """
        # tray full -> mode 3 (without robots) or mode 5 (with robots)
        # predict only if a new tray has been initialised, else return None
        time_now = self.env.now
        # place holders for prediction
        finish_row = None
        finish_node = None
        finish_dir = None
        finish_time = 0.

        if not self.has_started_a_tray():
            # if a tray is not started, nothing to calculate
            return (finish_row, finish_node, finish_dir, finish_time)

        # TODO: Predictions enabled without prior data, as we initialise mean values
#        # No prediction without prior data
#        if len(self.picking_times_per_tray) == 0:
#            return (finish_row, finish_node, finish_dir, finish_time)

        # for how long (distance) the picking (of the current tray) has been going on?
        dist_so_far = self.get_dist_picked_so_far()

        # prediction is going to be in a loop
        ## picking mode
        #     will the picker finish in the current row?
        #     if he will not finish,
        #         which would be his next row?
        #         go to next node for picking (no in picking mode)
        ## not in picking mode
        #     which would be his next row?
        #     can he reach there?
        #     how long would it take to reach there?
        #     after reaching start picking (picking mode)

        # calculate rem_time and estimate where the picker would be at that time
        remain_tray_pick_dist = self.mean_tray_pick_dist() - dist_so_far if (self.mean_tray_pick_dist() - dist_so_far) > 0. else 0.

        if remain_tray_pick_dist == 0:
            finish_row = self.curr_row
            finish_node = self.curr_node
            finish_dir = self.curr_dir
            finish_time = time_now
            return (finish_row, finish_node, finish_dir, finish_time)

        remain_tray_pick_time = remain_tray_pick_dist / self.mean_pick_rate()

        # remaining picking time to fill tray is already calculated
        # what would be the extra time needed for any other modes from now to tray_full
        extra_time = 0.

        curr_mode = self.mode[-1]
        curr_row = self.curr_row
        curr_node = self.curr_node
        curr_dir = self.curr_dir
        next_row = None

        if self.verbose:
            print "id:%s, row:%s, node:%s, dir:%s, mode:%s" %(self.picker_id, curr_row, curr_node, curr_dir, curr_mode)
            print "\t mean_tray_pick_dist = %0.2f" %(self.mean_tray_pick_dist())
            print "\t mean_pick_rate = %0.2f" %(self.mean_pick_rate())
            print "\t dist so far: %0.2f" %(dist_so_far)
            print "\t remaining_pick_dist: %0.2f" %(remain_tray_pick_dist)
            print "\t mean_idle_time: %0.2f" %(self.mean_idle_time())
            print "\t mean_trans_rate: %0.2f" %(self.mean_trans_rate())

        curr_row_idx = self.graph.row_ids.index(self.curr_row)
        while True:
            if rospy.is_shutdown():
                break

            # only possible modes here are idle and go_to_row_node (after row_finish) and picking
            if curr_mode == 0:
                # estimate the next row
                # accurately estimating the next row for individual picker is difficult as
                # the picking rate or states of the other pickers are unknown
                # curr_row + n_pickers is assumed to be the next row (with pickers having
                # equal picking rate)
                curr_row_idx = self.graph.row_ids.index(curr_row)
                next_row_idx = curr_row_idx + self.n_pickers
                # check whether getting to next_row possible
                if len(self.graph.row_ids) < next_row_idx + 1:
                    # can't estimate next row, resetting
                    extra_time = 0.
                    if self.verbose:
                        print "\t can't estimate - breaking 1"
                        print "\t next_row_idx: ", next_row_idx
                        print "\t len(row_ids): ", len(self.graph.row_ids)
                    break

                # estimated next_row exists
                next_row = self.graph.row_ids[next_row_idx]
                extra_time += self.mean_idle_time()
                curr_mode = 1
                if self.verbose:
                    print "\t idling now, extra_time + %0.2f" %(self.mean_idle_time())

            elif curr_mode == 1:
                if next_row is None:
                    # reaches here when the picker started in mode [1]
                    if self.verbose:
                        print "\t next_row is None"
                    # if the prediction method is called in this mode
                    curr_row_idx = self.graph.row_ids.index(curr_row)
                    next_row_idx = curr_row_idx + self.n_pickers
                    # check whether getting to next_row possible
                    if len(self.graph.row_ids) < next_row_idx + 1:
                        # can't estimate next row, resetting
                        extra_time = 0.
                        if self.verbose:
                            print "\t can't estimate - breaking 2"
                            print "\t next_row_idx: ", next_row_idx
                            print "\t len(row_ids): ", len(self.graph.row_ids)
                        break
                    # estimated next_row exists
                    next_row = self.graph.row_ids[next_row_idx]

                # calculate time to reach next row
                next_row_start_node = self.graph.row_info[next_row][1]
                extra_time += self.get_trans_time_to_node(curr_node, next_row_start_node)
                curr_row = next_row
                curr_node = next_row_start_node
                curr_mode = 2
                curr_dir = "forward"
                if self.verbose:
                    print "\t trans, extra_time +%0.2f" %(self.get_trans_time_to_node(curr_node, next_row_start_node))

            elif curr_mode == 2:
                # closest node in the current row
                cn_nodes, cn_dists, cn_dirs = self.get_closest_event_nodes_while_picking(curr_node, curr_dir, remain_tray_pick_dist)

                if self.verbose:
                    print "\t cn_nodes:", cn_nodes
                    print "\t rem_dists:", cn_dists
                    print "\t cn_dirs:", cn_dirs

#                if max(cn_dists) > remain_tray_pick_dist or numpy.isclose(max(cn_dists), remain_tray_pick_dist):
                if len(cn_dists) == 2:
                    # will finish in this row - two nodes are returned
                    if abs(cn_dists[0]) < abs(cn_dists[1]):
                        finish_node = cn_nodes[0]
                        finish_dir = cn_dirs[0]
                        if self.verbose:
                            print "\t picking_to_finish, tot_dist %0.2f" %(dist_so_far + remain_tray_pick_dist + cn_dists[0])
                    else:
                        finish_node = cn_nodes[1]
                        finish_dir = cn_dirs[1]
                        if self.verbose:
                            print "\t picking_to_finish, tot_dist %0.2f" %(dist_so_far + remain_tray_pick_dist + cn_dists[1])
                    finish_row = curr_row
                    break

                else:
                    # won't finish in this row - one node and distance returned
                    dist_so_far += remain_tray_pick_dist - cn_dists[0]
                    remain_tray_pick_dist = cn_dists[0]
                    curr_node = cn_nodes[0]
                    curr_dir = cn_dirs[0]
                    curr_mode = 0
                    if self.verbose:
                        print "\t picking_not_to_finish, tot_dist %0.2f" %(dist_so_far)

        # find finish_time
        finish_time = time_now + remain_tray_pick_time + extra_time
        if self.verbose:
            print "\t now:%0.2f, remain_pick:%0.2f, extra:%0.2f" %(time_now, remain_tray_pick_time, extra_time)

        return (finish_row, finish_node, finish_dir, finish_time)

    def get_dist_picked_so_far(self):
        """get the distance travelled so far while picking the current tray
        """
        if not self.has_started_a_tray():
            # if a tray is not started, nothing to calculate
            return 0.0

        # TODO: Predictions enabled without prior data, as we initialise mean values
#        # No prediction without prior data
#        if len(self.picking_times_per_tray) == 0:
#            return None

        # for how long (distance) the picking (of the current tray) has been going on?
        dist_so_far = 0.
        if self.mode_index_tray_start[-1] == len(self.mode) - 1:
            # picking started and no mode changes so far
            dist_so_far = self.get_picking_distance(self.mode_start_poses[-1], (self.curr_node, self.curr_dir))
        else:
            # picking started earlier, and there has been some mode changes bcz of row_finish
            # a single row picking may not fill up a tray - so there could be more than one
            # picking modes. curr_mode is not finished yet
            for idx in range(self.mode_index_tray_start[-1], len(self.mode) - 1):
                if self.mode[idx] == 2:
                    dist_so_far += self.get_picking_distance(self.mode_start_poses[idx], self.mode_stop_poses[idx])
            # if curr_mode == 2
            if self.mode[-1] == 2:
                dist_so_far += self.get_picking_distance(self.mode_start_poses[-1], (self.curr_node, self.curr_dir))
        return dist_so_far

    def get_trans_time_to_node(self, curr_node=None, goal_node=None):
        """If a picker is transporting to a row node, when will it reach there
        from the curr_node? Either both arguments must be given or both must
        not be given.

        Keyword arguments:

        curr_node - starting node, default: None (use self.curr_node)
        goal_node - finish node, default: None (use self.goal_node)
        """
        if ((curr_node is None and goal_node is not None) or
            (curr_node is not None and goal_node is None)):
            return None

        elif curr_node is None:
            _, _, dists = self.graph.get_path_details(self.curr_node, self.goal_node)
            return sum(dists) / self.mean_trans_rate()

        else:
            _, _, dists = self.graph.get_path_details(curr_node, goal_node)
            return sum(dists) / self.mean_trans_rate()

    def predict_picking_finish_event(self, curr_node, curr_dir, dist_so_far, time_now):
        """predict which of tray_full and row_finish will be the first event
        if tray_full, event: `tray_full`, details: [node, dir, time, dist_so_far]
        if row_finish, event: `row_finish`, details: [node, dir, time, dist_so_far]

        this method works only when the picker is in the picking mode

        Keyword arguments:
        curr_node -- current node
        curr_dir -- current direction
        dist_so_far -- if already started picking a tray, how much is picked so far
        time_now -- current time
        """
        next_event = None
        event_details = []

        # remaining_distance
        remain_tray_pick_dist = self.mean_tray_pick_dist() - dist_so_far

        try:
            assert remain_tray_pick_dist > 0. and not numpy.isclose(remain_tray_pick_dist, 0.)
        except:
            # remaining dist is less than zero
            next_event = "tray_full"
            event_details.append(curr_node)
            event_details.append(curr_dir)
            event_details.append(time_now)
            event_details.append(dist_so_far)
            return (next_event, event_details)

        # will it be finished in the current row?
        # if picking would not be finishing in this row, only node returnd is the row_end_node
        # if picking could be finishing this row, there will be two nodes (just before and after the
        # distance travelled equals the target distance) returned
        cn_nodes, cn_dists, cn_dirs = self.get_closest_event_nodes_while_picking(curr_node, curr_dir, remain_tray_pick_dist)

        if len(cn_dists) == 2:
            # will finish in this row - two nodes in cn_nodes
            # now return the closer one
            # TODO: better to return both with some probability?
            next_event = "tray_full"
            if abs(remain_tray_pick_dist - cn_dists[0]) < abs(remain_tray_pick_dist - cn_dists[1]):
                # picking_to_finish, remain_dist is cn_dists[0]
                event_details.append(cn_nodes[0])
                event_details.append(cn_dirs[0])
                tot_dist = dist_so_far + cn_dists[0]
                event_details.append(time_now + (cn_dists[0] / self.mean_pick_rate())) # picking time
                event_details.append(tot_dist) # picked distance
            else:
                # picking_to_finish, remain_dist is cn_dists[1]
                event_details.append(cn_nodes[1])
                event_details.append(cn_dirs[1])
                tot_dist = dist_so_far + cn_dists[1]
                event_details.append(time_now + (cn_dists[1] / self.mean_pick_rate())) # picking time
                event_details.append(tot_dist) # picked distance

        else:
            # won't finish in this row - one node and distance returned
            next_event = "row_finish"
            event_details.append(cn_nodes[0])
            event_details.append(cn_dirs[0])
            tot_dist = dist_so_far + cn_dists[0]
            event_details.append(time_now + (cn_dists[0] / self.mean_pick_rate())) # picking time
            event_details.append(tot_dist) # picked distance

        return (next_event, event_details)

    def get_closest_event_nodes_while_picking(self, curr_node, curr_dir, remain_tray_pick_dist):
        """given a row_id and a distance, return the closest nodes while picking
        along with the remaining distances and directions at these nodes.

        if picking would not be finishing in this row, only node returnd is the row_end_node
        if picking could be finishing this row, there will be two nodes (just before and after the
        distance travelled equals the target distance) returned

        keyword arguments:

        curr_node -- current node
        curr_dir -- current picking direction
        remain_tray_pick_dist -- remaining picking distance to fill the current tray
        """
        row_id = self.graph.get_row_id_of_row_node(curr_node)

        try:
            assert remain_tray_pick_dist > 0. and not numpy.isclose(remain_tray_pick_dist, 0.)
        except:
            msg = "remain_tray_pick_dist: %0.1f" %(remain_tray_pick_dist)
            raise Exception(msg)

        # find details of the route from curr_pose to row_end_node
        if curr_dir == "reverse":
            # full-row / bi-directional picking
            route_nodes, _, route_dists = self.graph.get_path_details(curr_node, self.graph.row_info[row_id][1])
            route_dir = ["reverse" for i in range(len(route_nodes))]
        else:
            # could be a half/full row
            route_nodes = []
            route_dists = []
            if row_id in self.graph.half_rows:
                # half-row, only forward picking
                route_nodes, _, route_dists = self.graph.get_path_details(curr_node, self.graph.row_info[row_id][2])
                route_dir = ["forward" for i in range(len(route_nodes))]
            else:
                # full-row, forward and reverse picking
                _route_nodes, _, _route_dists = self.graph.get_path_details(curr_node, self.graph.row_info[row_id][2])
                route_nodes.extend(_route_nodes)
                route_dists.extend(_route_dists)
                route_dir = ["forward" for i in range(len(_route_nodes))]
                _route_nodes, _, _route_dists = self.graph.get_path_details(self.graph.row_info[row_id][2], self.graph.row_info[row_id][1])
                route_nodes.extend(_route_nodes[1:])
                route_dists.extend(_route_dists)
                route_dir.extend(["reverse" for i in range(len(_route_nodes) - 1)])

        closest_nodes = []
        finish_dists = []
        finish_dirs = []
        remain_row_dist = sum(route_dists)

        if remain_tray_pick_dist < remain_row_dist or numpy.isclose(remain_tray_pick_dist, remain_row_dist):
            # given remain_tray_pick_dist <= the row_length (picking could finish in this row)
            _dist = 0.
            for i in range(len(route_dists)):
                _dist += route_dists[i]
                if (_dist > remain_tray_pick_dist or numpy.isclose(_dist, remain_tray_pick_dist)):
                    # len(route_dists) = len(route_nodes) - 1
                    # return the nodes just before and after _dist == remain_tray_pick_dist
                    closest_nodes.append(route_nodes[i])
                    finish_dists.append(_dist - route_dists[i])
                    finish_dirs.append(route_dir[i])
                    closest_nodes.append(route_nodes[i+1])
                    finish_dists.append(_dist)
                    finish_dirs.append(route_dir[i+1])
                    break

        else:
            # given remain_tray_pick_dist > remaining row_length
            if (len(route_dists) == 0):
                # start and end nodes are the same -> empty route_dists
                closest_nodes.append(curr_node)
                finish_dists.append(0.0)
                finish_dirs.append(curr_dir)
            else:
                # won't be finishing in this row
                closest_nodes.append(route_nodes[-1])
                finish_dists.append(remain_row_dist)
                finish_dirs.append(route_dir[-1])

        return (closest_nodes, finish_dists, finish_dirs)

    def has_started_a_tray(self):
        """return true if a tray has been started and it is not full yet"""
        return True if len(self.mode_index_tray_start) > len(self.mode_index_tray_stop) else False
