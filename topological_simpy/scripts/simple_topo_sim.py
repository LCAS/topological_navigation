#!/usr/bin/env python


import numpy
import topological_simpy.picker_sim
import topological_simpy.config_utils_mimic
import topological_simpy.topo_mimic
import topological_simpy.farm
import topological_simpy.config_utils
import topological_simpy.robot_sim
from topological_simpy.topo import TopologicalForkGraph
import simpy
import itertools
import csv
import os

from random import randint, choice, seed
from datetime import datetime

RANDOM_SEED = 10
SIM_RT_FACTOR = 1.0
VERBOSE = True

seed(RANDOM_SEED)
numpy.random.seed(RANDOM_SEED)

if __name__ == "__main__":
    # config parameters, specified in rasberry-multisim.yaml, for VPICKER_CONFIG
    # config_file = '/home/zuyuan/rasberry_ws/src/RASberry/rasberry_bringup/config/site_files/riseholme/polytunnel/transportation/picking_des.yaml'
    # config_file_des = '/home/zuyuan/rasberry_ws/src/RASberry/rasberry_des/config/des_config.yaml'
    config_file = '/home/zuyuan/catkin_ws/src/topological_navigation/topological_simpy/config/picking_sim.yaml'
    # get the config params
    config_params = topological_simpy.config_utils_mimic.get_mimic_des_params(config_file)
    # config_params = topological_simpy.config_utils.get_des_config_parameters(config_file_des)

    # env = simpy.RealtimeEnvironment(factor=SIM_RT_FACTOR, strict=False) # todo: test later
    env = simpy.Environment()
    tmap_config_file = '/home/zuyuan/catkin_ws/src/topological_navigation/topological_navigation/maps/riseholme.tmap2'
    # tmap = TopologicalForkGraph(env, tmap_config_file, verbose=True) #TODO: tmap --> topo_graph
    topo_graph = topological_simpy.topo_mimic.TopologicalForkGraphMimic(config_params["n_polytunnels"],
                                                                        config_params["n_farm_rows"],
                                                                        config_params["n_topo_nav_rows"],
                                                                        config_params["second_head_lane"],
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

    picker_ids = config_params["picker_ids"]
    n_pickers = len(picker_ids)
    picker_picking_rate = topological_simpy.config_utils.param_list_to_dict(
        "picker_picker_rate", config_params["picker_picking_rate"], picker_ids)
    picker_transportation_rate = topological_simpy.config_utils.param_list_to_dict(
        "picker_transportation_rate", config_params["picker_transportation_rate"], picker_ids)
    picker_max_n_trays = topological_simpy.config_utils.param_list_to_dict(
        "picker_max_n_trays", config_params["picker_max_n_trays"], picker_ids)
    picker_unloading_time = topological_simpy.config_utils.param_list_to_dict(
        "picker_unloading_time", config_params["picker_unloading_time"], picker_ids)

    # TODO: expand local storage node capacity(topo.py) to n_pickers + n_robots, so the node could hold multiple robots at the beginning
    local_storages = [simpy.Resource(env, capacity=n_pickers) for i in range(len(config_params["local_storage_nodes"]))]
    topo_graph.set_local_storages(local_storages, config_params["local_storage_nodes"])

    if config_params["use_cold_storage"]:
        cold_storage = simpy.Resource(env, capacity=n_pickers)
        topo_graph.set_cold_storage(cold_storage, config_params["cold_storage_node"])

    # robot_homes = ['dock_0', 'dock_1', 'dock_2']
    robot_homes = ['WayPoint131', 'WayPoint111', 'WayPoint66', 'WayPoint94']
    target_nodes = ['WayPoint66', 'WayPoint142', 'WayPoint102', 'WayPoint78']
    # robot_ids = ['Hurga', 'Foo', 'Foo2']
    robot_ids = ['Hurga']
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

    farm = topological_simpy.farm.Farm(config_params["map_name"],
                                       env,
                                       config_params["n_topo_nav_rows"],
                                       topo_graph,
                                       robots,
                                       pickers,
                                       scheduling_policy,
                                       # config_params["with_robots"],
                                       # config_params["n_iteration"],
                                       VERBOSE)

    # nodes = topo_graph.get_nodes()

    # def goal_generator(_env, _robots, _nodes, max_interval=50):
    #     for idx in itertools.count():
    #         delay_time = randint(_env.now + 1, _env.now + max_interval)  # TODO decrease delay_time
    #         # delay_time = randint(10, max_interval)  # TODO decrease delay_time
    #         print('.% 4d:    Generating goal after %6d seconds at %6d s' % (env.now, delay_time, env.now + delay_time))
    #         yield _env.timeout(delay_time)
    #         rob = choice(_robots)
    #         n = choice(_nodes)
    #         print('+% 4d:    new goal for robot %10s: %s' % (_env.now, rob._name, n))
    #         rob.goto(n)
    #
    #
    # def goal_generator2(_env, _robots, _target_nodes):
    #     for idx in range(len(_robots)):
    #         yield _env.timeout(5)
    #         rob = _robots[idx]
    #         print('+% 4d:    new goal for robot %10s: %s' % (_env.now, rob._name, _target_nodes[idx]))
    #         rob.goto(_target_nodes[idx])

    # env.process(goal_generator(env, robots, nodes))
    # env.process(goal_generator2(env, robots, target_nodes))

    until = 3000
    while env.peek() < until:
        # topo_graph.monitor()
        # for r in robots:
        #     print("R% 3d:     STATUS: % 10s: %s (%s)" % (
        #         env.now,
        #         r._name,
        #         r._current_node,
        #         'ACTIVE' if r._active_process and r._active_process.is_alive else 'IDLE'
        #     ))
        env.step()
    # print(tmap._node_log)

    # write the node_log to file
    now = datetime.now()

    file_name = 'node_log_' + now.isoformat() + '.csv'
    try:
        os.mkdir("./data")
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
    # env.run(until=3600)
