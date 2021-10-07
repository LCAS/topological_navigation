#!/usr/bin/env python
# ----------------------------------
# @author: ZuyuanZhu
# @email: zuyuanzhu@gmail.com
# @date: 07 Apr 2021
# ----------------------------------

import topological_simpy.picker_sim
import topological_simpy.config_utils_sim
import topological_simpy.topo_mimic
import topological_simpy.farm_sim
import topological_simpy.config_utils
import topological_simpy.robot_sim
import topological_simpy.visualise_sim
import numpy
import simpy
import csv
import os
import time
import multiprocessing as mp

from random import seed
from datetime import datetime

RANDOM_SEED = 19
SIM_RT_FACTOR = 0.00001  # simulation speed, 1: real time; 0.01: 100 times faster
VERBOSE = True
SHOW_VIS = True  # show visualisation
SAVE_RANDOM = True  # save random figures
SAVE_FINAL = True  # save final figures
SAVE_STATS = True

seed(RANDOM_SEED)
numpy.random.seed(RANDOM_SEED)


def run_sim(num_pickers, num_robots, scheduling_policy, ith_trial, tmap_configure_file):
    configure_params = topological_simpy.config_utils_sim.get_mimic_des_params(config_file, n_pickers=num_pickers,
                                                                               n_robots=num_robots)
    picker_ids = ["picker_%02d" % i for i in range(num_pickers)]
    robot_ids = ["robot_%02d" % i for i in range(num_robots)]

    _wait_nodes = configure_params["wait_nodes"]  # list of waiting nodes
    wait_nodes = {}
    if _wait_nodes is None:
        wait_nodes = {robot_id: "none" for robot_id in robot_ids}
    elif _wait_nodes.__class__ == str:
        if num_robots > 1:
            raise Exception("Not enough wait nodes (1) for %d robots!!!" % num_robots)
        wait_nodes = {robot_id: _wait_nodes for robot_id in robot_ids}
    elif _wait_nodes.__class__ == list:
        if len(_wait_nodes) < num_robots:
            raise Exception("Not enough base stations (%d) for %d robots!!!" % (len(_wait_nodes), num_robots))
        wait_nodes = {robot_ids[i]: _wait_nodes[i] for i in range(num_robots)}

    _base_stations = configure_params["base_station_nodes"]  # list of base station nodes
    base_stations = {}
    if _base_stations.__class__ == str:
        if num_robots > 1:
            raise Exception("Not enough base stations (1) for %d robots!!!" % num_robots)
        base_stations = {robot_id: _base_stations for robot_id in robot_ids}
    elif _base_stations.__class__ == list:
        if len(_base_stations) < num_robots:
            raise Exception("Not enough base stations (%d) for %d robots!!!" % (len(_base_stations), num_robots))
        base_stations = {robot_ids[i]: _base_stations[i] for i in range(num_robots)}

    topo_graph = topological_simpy.topo_mimic.TopologicalForkGraphMimic(configure_params["n_polytunnels"],
                                                                        configure_params["n_farm_rows"],
                                                                        configure_params["n_topo_nav_rows"],
                                                                        configure_params["second_head_lane"],
                                                                        base_stations,
                                                                        wait_nodes,
                                                                        env,
                                                                        tmap_configure_file,
                                                                        VERBOSE)

    # set row_info, local_storage_nodes should be set separately
    if configure_params["second_head_lane"]:
        topo_graph.set_row_info(configure_params["pri_head_nodes"],
                                configure_params["row_nodes"],
                                configure_params["sec_head_nodes"])
    else:
        topo_graph.set_row_info(configure_params["pri_head_nodes"],
                                configure_params["row_nodes"])

    # set node_yields
    topo_graph.set_node_yields(configure_params["yield_per_node"])

    # set local storage and cold storage
    # TODO:[done?] pickers use local_storage_nodes(capacity = num_pickers), robot use WayPoint66(capacity = 1).
    #       local_storage_nodes used for initialise picker's positions.
    # TODO: When the pickers also use the same simpy tmap as the robots, how to solve the initialisation position of pickers?
    local_storage_capacity = num_pickers  # local storage node keeps the rule: capacity = 1 for every node
    # TODO:1.expand local storage node capacity(topo.py) to n_pickers
    #       so the node could hold multiple pickers at the beginning
    local_storages = [simpy.Resource(env, capacity=local_storage_capacity) for i in
                      range(len(configure_params["local_storage_nodes"]))]
    topo_graph.set_local_storages(local_storages, configure_params["local_storage_nodes"])
    # TODO: [next] set cold_storage_capacity = 1 after split out the SimPy topo.py; use cold_storage for storing, use WayPoint69 for navigation
    cold_storage_capacity = num_pickers
    if configure_params["use_cold_storage"]:
        cold_storage = simpy.Resource(env, capacity=cold_storage_capacity)
        topo_graph.set_cold_storage(cold_storage, configure_params["cold_storage_node"])
    
    # set picker picking information    
    picker_picking_rate = topological_simpy.config_utils.param_list_to_dict(
        "picker_picker_rate", configure_params["picker_picking_rate"], picker_ids)
    picker_transportation_rate = topological_simpy.config_utils.param_list_to_dict(
        "picker_transportation_rate", configure_params["picker_transportation_rate"], picker_ids)
    picker_max_n_trays = topological_simpy.config_utils.param_list_to_dict(
        "picker_max_n_trays", configure_params["picker_max_n_trays"], picker_ids)
    picker_unloading_time = topological_simpy.config_utils.param_list_to_dict(
        "picker_unloading_time", configure_params["picker_unloading_time"], picker_ids)

    # set robot transporting information
    robot_transportation_rate = topological_simpy.config_utils.param_list_to_dict(
        "robot_transportation_rate", configure_params["robot_transportation_rate"], robot_ids)
    robot_max_n_trays = topological_simpy.config_utils.param_list_to_dict(
        "robot_max_n_trays", configure_params["robot_max_n_trays"], robot_ids)
    robot_unloading_time = topological_simpy.config_utils.param_list_to_dict(
        "robot_unloading_time", configure_params["robot_unloading_time"], robot_ids)

    robots = []
    for robot_id in robot_ids:
        robots.append(topological_simpy.robot_sim.RobotSim(robot_id,
                                                           robot_transportation_rate[robot_id],
                                                           robot_max_n_trays[robot_id],
                                                           robot_unloading_time[robot_id],
                                                           env,
                                                           topo_graph,
                                                           verbose=True))

    pickers = []
    for picker_id in picker_ids:
        pickers.append(topological_simpy.picker_sim.PickerSim(picker_id,
                                                              configure_params["tray_capacity"],
                                                              picker_max_n_trays[picker_id],
                                                              picker_picking_rate[picker_id],
                                                              picker_transportation_rate[picker_id],
                                                              picker_unloading_time[picker_id],
                                                              env,
                                                              topo_graph,
                                                              robots,
                                                              configure_params["with_robots"],
                                                              VERBOSE))

    farm = topological_simpy.farm_sim.FarmSim(configure_params["map_name"],
                                              env,
                                              configure_params["n_topo_nav_rows"],
                                              topo_graph,
                                              robots,
                                              pickers,
                                              scheduling_policy,
                                              configure_params["n_iteration"],
                                              VERBOSE)

    # create log directory if does not exist already
    if SAVE_STATS:
        asc_time = time.asctime().split(" ")
        if asc_time[2] == "":
            log_dir = os.path.join(os.path.expanduser("~"), "des_simpy_logs", "%s_%s_0%s_%s" % (
                asc_time[5], asc_time[1], asc_time[3], asc_time[4].replace(":", "_")))
        else:
            log_dir = os.path.join(os.path.expanduser("~"), "des_simpy_logs", "%s_%s_%s_%s" % (
                asc_time[4], asc_time[1], asc_time[2], asc_time[3].replace(":", "_")))
        if os.path.exists(log_dir):
            pass
        else:
            os.makedirs(log_dir)

    n_deadlock = topo_graph.n_deadlock
    if SHOW_VIS:
        vis = topological_simpy.visualise_sim.VisualiseAgentsSim(log_dir, topo_graph, robots,
                                                                 pickers, scheduling_policy,
                                                                 n_deadlock,
                                                                 tmap_name=tmap_configure_file,
                                                                 show_cs=True,
                                                                 save_random=SAVE_RANDOM,
                                                                 save_final=SAVE_FINAL,
                                                                 trial=ith_trial)

    # instead of env.run() we should env.step() to have any control (Ctrl+c)
    # If multiple events are scheduled for the same simpy.time, there would be
    # at least a ms delay (in ros/realtime clock) between the events
    # This seems to be unavoidable at this stage
    # until = 3600
    # env_peek = 0.0
    monitor = farm.monitor(pickers, robots)

    start_time_simpy = env.now
    start_time_real = time.time()
    while True:
        # env_peek = env.peek()
        try:
            env.step()
            if SHOW_VIS and (monitor != farm.monitor(pickers, robots)):
                vis.update_plot()
                monitor = farm.monitor(pickers, robots)
        except simpy.core.EmptySchedule:
            if SHOW_VIS:
                vis.close_plot()
            break
        else:
            pass

    finish_time_simpy = env.now
    finish_time_real = time.time()

    # if until - env_peek < 0.1:
    #     print("Stop at simulation time: %.1f, allowed maximum-simulation-time: %.1f " % (env_peek, until))
    #     print("Considering increase allowed maximum-simulation-time: 'until', and run again!")

    # =============================================
    # save simulation result data
    # =============================================

    # print("Saving simulation results:\n "
    #       "num_pickers:%d, num_robots:%d, scheduling_policy:%s, ith_trial:%d, running time(clock):%.3f"
    #       % (num_pickers, num_robots, scheduling_policy, ith_trial, finish_time_real - start_time_real))

    if SAVE_STATS:
        time_now = time.time() * 1000000

        # event logs
        map_name = configure_params["map_name"]
        tray_capacity = configure_params["tray_capacity"]
        # n_iteration = configure_params["n_iteration"]

    # for n_iter in range(n_iteration):
        f_handle = open(log_dir + "/M%s_P%d_R%d_S%s_I%d_%d_events.yaml" % (
            map_name, num_pickers, num_robots, scheduling_policy, ith_trial, time_now), "w")

        # sim details
        print >> f_handle, "# Environment details"
        print >> f_handle, "env_details:"
        print >> f_handle, "  map_name: %s" % map_name
        print >> f_handle, "  n_polytunnels: %d" % topo_graph.n_polytunnels
        if topo_graph.n_polytunnels > 1:
            print >> f_handle, "  n_farm_rows:"
            for i in range(topo_graph.n_polytunnels):
                print >> f_handle, "    tunnel-%d: %d" % (i, topo_graph.n_farm_rows[i])
        print >> f_handle, "  n_topo_nav_rows: %d" % topo_graph.n_topo_nav_rows
        print >> f_handle, "  n_local_storages: %d" % len(topo_graph.local_storages)
        print >> f_handle, "  use_local_storage: %s" % topo_graph.use_local_storage
        tot_yield = 0.
        if len(topo_graph.row_ids) > 1:
            print >> f_handle, "  row_details:"
        for row_id in topo_graph.row_ids:
            row_start_node = topo_graph.row_info[row_id][1]
            row_end_node = topo_graph.row_info[row_id][2]
            row_start_x = topo_graph.get_node(row_start_node)['node']['pose']['position']['x']
            row_start_y = topo_graph.get_node(row_start_node)['node']['pose']['position']['y']
            row_end_x = topo_graph.get_node(row_end_node)['node']['pose']['position']['x']
            row_end_y = topo_graph.get_node(row_end_node)['node']['pose']['position']['y']
            row_length = numpy.hypot((row_end_x - row_start_x), (row_end_y - row_start_y))
            node_dist = topo_graph.get_distance_between_adjacent_nodes(topo_graph.row_nodes[row_id][0],
                                                                       topo_graph.row_nodes[row_id][1])
            storage_node = topo_graph.local_storage_nodes[row_id]
            print >> f_handle, "  -  row_id: %s" % row_id
            print >> f_handle, "     storage_node:"
            print >> f_handle, "       node_id: %s" % (topo_graph.local_storage_nodes[row_id])
            print >> f_handle, "       x: %0.3f" % topo_graph.get_node(storage_node)['node']['pose']['position']['x']
            print >> f_handle, "       y: %0.3f" % topo_graph.get_node(storage_node)['node']['pose']['position']['y']
            print >> f_handle, "     start_node:"
            print >> f_handle, "       node_id: %s" % (topo_graph.row_info[row_id][1])
            print >> f_handle, "       x: %0.3f" % topo_graph.get_node(row_start_node)['node']['pose']['position']['x']
            print >> f_handle, "       y: %0.3f" % topo_graph.get_node(row_start_node)['node']['pose']['position']['y']
            print >> f_handle, "     end_node:"
            print >> f_handle, "       node_id: %s" % (topo_graph.row_info[row_id][2])
            print >> f_handle, "       x: %0.3f" % topo_graph.get_node(row_end_node)['node']['pose']['position']['x']
            print >> f_handle, "       y: %0.3f" % topo_graph.get_node(row_end_node)['node']['pose']['position']['y']
            print >> f_handle, "     row_length: %0.3f" % row_length
            print >> f_handle, "     node_dist: %0.3f" % node_dist
            row_yield = 0.
            n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
            if row_id in topo_graph.half_rows:
                print >> f_handle, "     half_row: True"
                print >> f_handle, "     head_nodes:"
                for head_node in topo_graph.head_nodes[row_id]:
                    print >> f_handle, "     -  node_id: %s" % head_node
                    print >> f_handle, "        x: %0.3f" % topo_graph.get_node(head_node)['node']['pose']['position']['x']
                    print >> f_handle, "        y: %0.3f" % topo_graph.get_node(head_node)['node']['pose']['position']['y']
                print >> f_handle, "     row_nodes:"
                for i in range(n_row_nodes):
                    node_name = topo_graph.row_nodes[row_id][i]
                    print >> f_handle, "     -  node_id: %s" % node_name
                    print >> f_handle, "        x: %0.3f" % (topo_graph.get_node(node_name)['node']['pose']['position']['x'],)
                    print >> f_handle, "        y: %0.3f" % topo_graph.get_node(node_name)['node']['pose']['position']['y']
                    print >> f_handle, "        yield: %0.3f" % (topo_graph.yield_at_node[node_name])
                    row_yield += topo_graph.yield_at_node[node_name]
            else:
                print >> f_handle, "     half_row: False"
                print >> f_handle, "     head_nodes:"
                for head_node in topo_graph.head_nodes[row_id]:
                    print >> f_handle, "     -  node_id: %s" % head_node
                    print >> f_handle, "        x: %0.3f" % topo_graph.get_node(head_node)['node']['pose']['position']['x']
                    print >> f_handle, "        y: %0.3f" % topo_graph.get_node(head_node)['node']['pose']['position']['y']
                print >> f_handle, "     row_nodes:"
                for i in range(n_row_nodes):
                    node_name = topo_graph.row_nodes[row_id][i]
                    print >> f_handle, "     -  node_id: %s" % node_name
                    print >> f_handle, "        x: %0.3f" % (topo_graph.get_node(node_name)['node']['pose']['position']['x'],)
                    print >> f_handle, "        y: %0.3f" % topo_graph.get_node(node_name)['node']['pose']['position']['y']
                    print >> f_handle, "        yield: %0.3f" % (topo_graph.yield_at_node[node_name])
                    if (i == 0) or (i == n_row_nodes - 1):
                        row_yield += topo_graph.yield_at_node[node_name]
                    else:
                        row_yield += 2 * topo_graph.yield_at_node[node_name]
            print >> f_handle, "     row_yield: %0.3f" % row_yield
            tot_yield += row_yield
        print >> f_handle, "  tot_yield: %0.3f" % tot_yield
        print >> f_handle, "  tot_yield_trays: %0.3f" % (tot_yield / tray_capacity)

        print >> f_handle, "# Simulation Details"
        print >> f_handle, "sim_details:"
        print >> f_handle, "  map_name: %s" % map_name
        print >> f_handle, "  n_pickers: %d" % num_pickers
        print >> f_handle, "  n_robots: %d" % num_robots
        print >> f_handle, "  sim_finish_time_simpy: %0.3f" % finish_time_simpy
        print >> f_handle, "  sim_finish_time_clock: %0.3f" % (finish_time_real - start_time_real)
        for item in farm.events:
            if item[1] == "starting the process":
                print >> f_handle, "  start_time_simpy: %0.3f" % (float(item[0]))
            if item[1] == "finished row allocations":
                print >> f_handle, "  finish_allocation_time_simpy: %0.3f" % (float(item[0]))
            if item[1] == "finished picking":
                print >> f_handle, "  finish_picking_time_simpy: %0.3f" % (float(item[0]))
        print >> f_handle, "# picker_modes:"
        print >> f_handle, "# 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,"
        print >> f_handle, "# 4: waiting for unload at storage, 5: waiting for loading on robot"
        print >> f_handle, "# 6: transporting to local storage from cold storage"
        print >> f_handle, "  picker_states:"
        for i in range(num_pickers):
            picker_id = picker_ids[i]
            print >> f_handle, "    -  picker_id: %s" % pickers[i].picker_id
            print >> f_handle, "       picking_rate: %0.3f" % pickers[i].picking_rate
            print >> f_handle, "       transportation_rate: %0.3f" % pickers[i].transportation_rate
            print >> f_handle, "       tray_capacity: %d" % pickers[i].tray_capacity
            print >> f_handle, "       max_n_trays: %d" % pickers[i].max_n_trays
            print >> f_handle, "       tray_loading_time: %0.3f" % pickers[i].unloading_time
            print >> f_handle, "       tray_unloading_time: %0.3f" % pickers[i].unloading_time
            print >> f_handle, "       allocated_rows:"
            for row_id in farm.picker_allocations[ith_trial][pickers[i].picker_id]:
                print >> f_handle, "       -  row_id: %s" % row_id
                alloc_time = farm.allocation_time[ith_trial][row_id]
                finish_time = farm.row_finish_time[ith_trial][row_id]
                print >> f_handle, "          allocation_time: %0.3f" % (
                    alloc_time if alloc_time is not None else float("inf"))
                print >> f_handle, "          completion_time: %0.3f" % (
                    finish_time if finish_time is not None else float("inf"))
            print >> f_handle, "       tot_trays: %0.3f" % pickers[i].tot_trays
            print >> f_handle, "       tot_tray_capacity: %0.3f" % (pickers[i].tot_trays * pickers[i].tray_capacity)
            print >> f_handle, "       picking_time: %0.3f" % pickers[i].time_spent_picking
            print >> f_handle, "       transportation_time: %0.3f" % pickers[i].time_spent_transportation
            print >> f_handle, "       idle_time: %0.3f" % pickers[i].time_spent_idle
            print >> f_handle, "       wait_for_robot_time: %0.3f" % pickers[i].time_spent_waiting
            print >> f_handle, "       loading_on_robot_time: %0.3f" % pickers[i].time_spent_loading
            print >> f_handle, "       unloading_at_storage_time: %0.3f" % pickers[i].time_spent_unloading
            print >> f_handle, "       total_working_time: %0.3f" % (pickers[i].time_spent_working())
            print >> f_handle, "       state_changes:"
            print >> f_handle, "# state_changes: mode, node, direction, time_now"
            for item in farm.predictor.predictors[picker_id].modes_nodes_dirs_times:
                print >> f_handle, "       -  mode: %d" % (item[0])
                print >> f_handle, "          node: %s" % (item[1])
                print >> f_handle, "          direction: %s" % (item[2])
                print >> f_handle, "          time: %0.3f" % (item[3])
        f_handle.close()

        # predictions log
        f_handle = open(log_dir + "/M%s_P%d_R%d_S%s_%d_predictions.dat" % (
            map_name, num_pickers, num_robots, scheduling_policy, time_now), "w")
        print >> f_handle, "picker.pred_row, picker.pred_node, picker.pred_dir, picker.pred_time, picker.curr_node, picker.picking_dir, time_now"
        print >> f_handle, "picker.prev_row, picker.curr_node, picker.picking_dir, time_now, actual\n"
        for picker_id in picker_ids:
            print >> f_handle, picker_id
            predictions = farm.predictions[picker_id]
            for tray in range(1, farm.tray_counts[picker_id] + 1):
                #                                f_handle.write(predictions[tray])
                print >> f_handle, "\t", tray, ":"
                for item in predictions[tray]:
                    print >> f_handle, "\t\t", item
        f_handle.close()

        # des logs
        f_handle = open(
            log_dir + "/M%s_P%d_R%d_S%s_%d_farm.dat" % (map_name, num_pickers, num_robots, scheduling_policy, time_now), "w")
        # farm details
        print >> f_handle, "-----------------\n----%s----\n-----------------" % farm.name

        print >> f_handle, "simulation_finish_time(sim): %0.3f" % finish_time_simpy
        print >> f_handle, "simulation_finish_time(clock): %0.3f" % (finish_time_real - start_time_real)

        print >> f_handle, "n_pickers: %d" % num_pickers
        print >> f_handle, "n_robots: %d" % num_robots

        print >> f_handle, "n_polytunnels: %d" % topo_graph.n_polytunnels
        for i in range(topo_graph.n_polytunnels):
            print >> f_handle, "n_farm_rows[tunnel-%d]: %d" % (i, topo_graph.n_farm_rows[i])
        print >> f_handle, "n_topo_nav_rows: %d" % topo_graph.n_topo_nav_rows

        tot_yield = 0.
        for row_id in topo_graph.row_ids:
            print >> f_handle, "  --%s--" % row_id
            row_start_node = topo_graph.row_info[row_id][1]
            row_end_node = topo_graph.row_info[row_id][2]
            row_start_x = topo_graph.get_node(row_start_node)['node']['pose']['position']['x']
            row_start_y = topo_graph.get_node(row_start_node)['node']['pose']['position']['y']
            row_end_x = topo_graph.get_node(row_end_node)['node']['pose']['position']['x']
            row_end_y = topo_graph.get_node(row_end_node)['node']['pose']['position']['y']
            row_length = numpy.hypot((row_end_x - row_start_x), (row_end_y - row_start_y))
            node_dist = topo_graph.get_distance_between_adjacent_nodes(topo_graph.row_nodes[row_id][0],
                                                                       topo_graph.row_nodes[row_id][1])
            print >> f_handle, "  row_length: %0.3f m" % row_length
            print >> f_handle, "  node_dist: %0.3f m" % node_dist
            row_yield = 0.
            n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
            if row_id in topo_graph.half_rows:
                for i in range(1, n_row_nodes):
                    row_yield += topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
            else:
                for i in range(n_row_nodes):
                    if (i == 0) or (i == n_row_nodes - 1):
                        row_yield += topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
                    else:
                        row_yield += 2 * topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
            print >> f_handle, "  row_yield: %0.3f g" % row_yield
            tot_yield += row_yield
        print >> f_handle, "tot_yield: %0.3f trays (%0.3f g)" % (tot_yield / tray_capacity, tot_yield)
        print >> f_handle, "\n"

        # picker details
        for i in range(num_pickers):
            print >> f_handle, "----%s----\n-----------------" % pickers[i].picker_id
            print >> f_handle, "picker_picking_rate: %0.3f m/s" % pickers[i].picking_rate
            print >> f_handle, "picker_transportation_rate: %0.3f m/s" % pickers[i].transportation_rate
            print >> f_handle, "tray_capacity: %d g" % pickers[i].tray_capacity
            print >> f_handle, "picker_max_n_trays: %d" % pickers[i].max_n_trays
            print >> f_handle, "picker_unloading_time(per tray): %d" % pickers[i].unloading_time
            print >> f_handle, "rows allocated: ", farm.picker_allocations[ith_trial][pickers[i].picker_id]
            for row_id in farm.picker_allocations[ith_trial][pickers[i].picker_id]:
                alloc_time = farm.allocation_time[ith_trial][row_id]
                finish_time = farm.row_finish_time[ith_trial][row_id]
                print >> f_handle, "  %s allocation time: %0.3f" % (row_id,
                                                                    alloc_time if alloc_time is not None
                                                                    else float("inf"))
                print >> f_handle, "  %s completion time: %0.3f" % (row_id,
                                                                    finish_time if finish_time is not None
                                                                    else float("inf"))
            print >> f_handle, "tot_trays: %0.3f (%0.3f g)" % (pickers[i].tot_trays,
                                                               pickers[i].tot_trays * pickers[i].tray_capacity)
            print >> f_handle, "picking_time: %0.3f" % pickers[i].time_spent_picking
            print >> f_handle, "transportation_time: %0.3f" % pickers[i].time_spent_transportation
            print >> f_handle, "idle_time: %0.3f" % pickers[i].time_spent_idle
            print >> f_handle, "waiting_for_robot_time: %0.3f" % pickers[i].time_spent_waiting
            print >> f_handle, "loading_on_robot_time: %0.3f" % pickers[i].time_spent_loading
            print >> f_handle, "unloading_time: %0.3f" % pickers[i].time_spent_unloading
            print >> f_handle, "total_working_time: %0.3f" % (pickers[i].time_spent_working())
            print >> f_handle, "-----------------\n"

        # robot details
        for i in range(num_robots):
            print >> f_handle, "----%s----\n-----------------" % robots[i].robot_id
            print >> f_handle, "robot_transportation_rate: %0.3f m/s" % robots[i].transportation_rate
            print >> f_handle, "robot_max_n_trays: %d" % robots[i].max_n_trays
            print >> f_handle, "robot_unloading_time(per tray): %d" % robots[i].unloading_time
            print >> f_handle, "tot_trays: %0.3f" % robots[i].tot_trays
            print >> f_handle, "picking_time: %0.3f" % robots[i].time_spent_picking
            print >> f_handle, "transportation_time: %0.3f" % robots[i].time_spent_transportation
            print >> f_handle, "idle_time: %0.3f" % robots[i].time_spent_idle
            print >> f_handle, "loading_time: %0.3f" % robots[i].time_spent_loading
            print >> f_handle, "unloading_time: %0.3f" % robots[i].time_spent_unloading
            print >> f_handle, "charging_time: %0.3f" % robots[i].time_spent_charging
            print >> f_handle, "requesting_time: %0.3f" % robots[i].time_spent_requesting
            print >> f_handle, "base_time: %0.3f" % robots[i].time_spent_base
            print >> f_handle, "total_working_time: %0.3f" % (robots[i].time_spent_working())
            print >> f_handle, "-----------------\n"

        f_handle.close()

        # write the node_log to file
        now = datetime.now()
        now_iso = now.isoformat()
        now_iso = now_iso.replace(":", "_")
        file_name = 'node_log_' + now_iso + '.csv'

        with open(log_dir + file_name, 'w') as csv_file:
            writer = csv.writer(csv_file)
            for key, value in topo_graph.node_log.items():
                writer.writerow([key, value])
            writer.writerow('')
            for r in robots:
                for key, value in r.cost.items():
                    writer.writerow([key, value])
                writer.writerow('')


if __name__ == "__main__":
    # config_file = 'topological_simpy/config/picking_sim_combined.yaml'
    # config_file = '../config/picking_sim_riseholme_simple.yaml'
    # config_file = '../config/picking_sim_riseholme.yaml'
    config_file = '../config/picking_sim_riseholme_4_bases_2_parellel.yaml'
    # config_file = '../config/picking_sim_riseholme_4_bases.yaml'
    # get the config params
    # config_params = topological_simpy.config_utils_sim.get_mimic_des_params(config_file)

    env = simpy.RealtimeEnvironment(factor=SIM_RT_FACTOR, strict=False)
    # env = simpy.Environment()
    # tmap_config_file = 'topological_simpy/maps/tmap.yaml'
    # tmap_config_file = '../maps/riseholme_4_bases_large.tmap2'   # default
    # tmap_config_file = '../maps/edited_riseholme_3.tmap2'  # default
    # tmap_config_file = '../maps/riseholme.tmap2'  # default
    # tmap_config_file = '../maps/clockhouse.tmap'
    tmap_config_file = '../maps/riseholme_4_bases_2_parallel_4.tmap2'   # default

    # parameters for running the simulation
    n_trials = 1
    min_n_pickers = 4
    max_n_pickers = 5
    min_n_robots = 4
    max_n_robots = max_n_pickers
    policies = ["uniform_utilisation"]   # ["lexicographical", "shortest_distance", "uniform_utilisation"]

    # run the simulation as many as needed
    start_time_real = time.time()
    for n_picker in range(min_n_pickers, max_n_pickers):
        for n_robot in range(min_n_robots, max_n_robots):
            for schedule_policy in policies:
                pool = mp.Pool(mp.cpu_count())
                # pool = mp.Pool(1)
                single_start_time_real = time.time()
                for trial in range(n_trials):
                    result = pool.apply_async(run_sim, (n_picker, n_robot, schedule_policy, trial, tmap_config_file, ))
                    # run(n_picker, n_robot, schedule_policy, trial)  #  for testing serial running
                print("Waiting for all subprocesses done...")
                pool.close()  # use single core, wait one finish and add another
                pool.join()
                single_end_time_real = time.time()
                print("n_picker: %d, n_robot: %d, schedule_policy: %s, n_trials: %d, running time: %.3f" % (
                    n_picker, n_robot, schedule_policy, n_trials, single_end_time_real - single_start_time_real))
    end_time_real = time.time()
    print("Program total running time: %.3f" % (end_time_real - start_time_real))











