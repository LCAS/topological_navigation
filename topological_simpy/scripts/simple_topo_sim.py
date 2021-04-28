#!/usr/bin/env python
# ----------------------------------
# @author: ZuyuanZhu
# @email: zuyuanzhu@gmail.com
# @date: 07 Apr 2021
# ----------------------------------

import numpy
import topological_simpy.picker_sim
import topological_simpy.config_utils_sim
import topological_simpy.topo_mimic
import topological_simpy.farm_sim
import topological_simpy.config_utils
import topological_simpy.robot_sim
import simpy
import csv
import os
import topological_simpy.visualise_sim

from random import seed
from datetime import datetime

RANDOM_SEED = 10
SIM_RT_FACTOR = 0.2   # simulation speed, 1: real time; 0.01: 100 times faster
VERBOSE = True
SHOW_VIS = True        # show visualisation
trial = 0

seed(RANDOM_SEED)
numpy.random.seed(RANDOM_SEED)

if __name__ == "__main__":
    config_file = '../config/picking_sim_combined.yaml'
    # get the config params
    config_params = topological_simpy.config_utils_sim.get_mimic_des_params(config_file)

    env = simpy.RealtimeEnvironment(factor=SIM_RT_FACTOR, strict=False)
    # env = simpy.Environment()
    tmap_config_file = '../maps/tmap.yaml'

    picker_ids = config_params["picker_ids"]
    robot_ids = config_params["robot_ids"]
    n_pickers = len(picker_ids)
    n_robots = len(robot_ids)

    _wait_nodes = config_params["wait_nodes"]  # list of waiting nodes
    wait_nodes = {}
    if _wait_nodes is None:
        wait_nodes = {robot_id: "none" for robot_id in robot_ids}
    elif _wait_nodes.__class__ == str:
        if len(robot_ids) > 1:
            raise Exception("Not enough wait nodes (1) for %d robots!!!" % (len(robot_ids)))
        wait_nodes = {robot_id: _wait_nodes for robot_id in robot_ids}
    elif _wait_nodes.__class__ == list:
        if len(_wait_nodes) != len(robot_ids):
            raise Exception("Not enough base stations (%d) for %d robots!!!" % (len(_wait_nodes), len(robot_ids)))
        wait_nodes = {robot_ids[i]: _wait_nodes[i] for i in range(len(robot_ids))}

    _base_stations = config_params["base_station_nodes"]  # list of base station nodes
    base_stations = {}
    if _base_stations.__class__ == str:
        if len(robot_ids) > 1:
            raise Exception("Not enough base stations (1) for %d robots!!!" % (len(robot_ids)))
        base_stations = {robot_id: _base_stations for robot_id in robot_ids}
    elif _base_stations.__class__ == list:
        if len(_base_stations) != len(robot_ids):
            raise Exception("Not enough base stations (%d) for %d robots!!!" % (len(_base_stations), len(robot_ids)))
        base_stations = {robot_ids[i]: _base_stations[i] for i in range(len(robot_ids))}

    topo_graph = topological_simpy.topo_mimic.TopologicalForkGraphMimic(config_params["n_polytunnels"],
                                                                        config_params["n_farm_rows"],
                                                                        config_params["n_topo_nav_rows"],
                                                                        config_params["second_head_lane"],
                                                                        base_stations,
                                                                        wait_nodes,
                                                                        env,
                                                                        tmap_config_file,
                                                                        VERBOSE)

    # set row_info, local_storage_nodes should be set separately
    if config_params["second_head_lane"]:
        topo_graph.set_row_info(config_params["pri_head_nodes"],
                                config_params["row_nodes"],
                                config_params["sec_head_nodes"])
    else:
        topo_graph.set_row_info(config_params["pri_head_nodes"],
                                config_params["row_nodes"])
    # set node_yields
    topo_graph.set_node_yields(config_params["yield_per_node"])

    picker_picking_rate = topological_simpy.config_utils.param_list_to_dict(
        "picker_picker_rate", config_params["picker_picking_rate"], picker_ids)
    picker_transportation_rate = topological_simpy.config_utils.param_list_to_dict(
        "picker_transportation_rate", config_params["picker_transportation_rate"], picker_ids)
    picker_max_n_trays = topological_simpy.config_utils.param_list_to_dict(
        "picker_max_n_trays", config_params["picker_max_n_trays"], picker_ids)
    picker_unloading_time = topological_simpy.config_utils.param_list_to_dict(
        "picker_unloading_time", config_params["picker_unloading_time"], picker_ids)

    local_storage_capacity = n_robots + n_pickers
    # expand local storage node capacity(topo.py) to n_pickers + n_robots,
    # so the node could hold multiple robots at the beginning
    local_storages = [simpy.Resource(env, capacity=local_storage_capacity) for i in
                      range(len(config_params["local_storage_nodes"]))]
    topo_graph.set_local_storages(local_storages, config_params["local_storage_nodes"])

    if config_params["use_cold_storage"]:
        cold_storage = simpy.Resource(env, capacity=n_pickers)
        topo_graph.set_cold_storage(cold_storage, config_params["cold_storage_node"])

    robot_transportation_rate = topological_simpy.config_utils.param_list_to_dict(
        "robot_transportation_rate", config_params["robot_transportation_rate"], robot_ids)
    robot_max_n_trays = topological_simpy.config_utils.param_list_to_dict(
        "robot_max_n_trays", config_params["robot_max_n_trays"], robot_ids)
    robot_unloading_time = topological_simpy.config_utils.param_list_to_dict(
        "robot_unloading_time", config_params["robot_unloading_time"], robot_ids)

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
                                                              config_params["tray_capacity"],
                                                              picker_max_n_trays[picker_id],
                                                              picker_picking_rate[picker_id],
                                                              picker_transportation_rate[picker_id],
                                                              picker_unloading_time[picker_id],
                                                              env,
                                                              topo_graph,
                                                              robots,
                                                              config_params["with_robots"],
                                                              VERBOSE))

    scheduling_policy = "lexicographical"  # ["lexicographical", "shortest_distance", "uniform_utilisation"]

    farm = topological_simpy.farm_sim.FarmSim(config_params["map_name"],
                                              env,
                                              config_params["n_topo_nav_rows"],
                                              topo_graph,
                                              robots,
                                              pickers,
                                              scheduling_policy,
                                              VERBOSE)

    if SHOW_VIS:
        vis = topological_simpy.visualise_sim.VisualiseAgentsSim(topo_graph, robots,
                                                                 pickers, scheduling_policy,
                                                                 show_cs=True,
                                                                 save_random=False,
                                                                 trial=trial)

    # instead of env.run() we should env.step() to have any control (Ctrl+c)
    # If multiple events are scheduled for the same simpy.time, there would be
    # at least a ms delay (in ros/realtime clock) between the events
    # This seems to be unavoidable at this stage
    until = 600
    monitor = farm.monitor(pickers, robots)
    while env.peek() < until:
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

    # write the node_log to file
    now = datetime.now()

    file_name = 'node_log_' + now.isoformat() + '.csv'
    try:
        os.mkdir("../data")
    except OSError as e:
        print("Directory exists")

    with open('../data/' + file_name, 'w') as csv_file:
        writer = csv.writer(csv_file)
        for key, value in topo_graph.node_log.items():
            writer.writerow([key, value])
        writer.writerow('')
        for r in robots:
            for key, value in r.cost.items():
                writer.writerow([key, value])
            writer.writerow('')
