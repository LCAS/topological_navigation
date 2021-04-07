#! /usr/bin/env python


import random
import simpy
import numpy
import topological_simpy.picker_sim
import topological_simpy.config_utils_mimic
import topological_simpy.topo_mimic
import topological_simpy.farm_mimic
import topological_simpy.config_utils
from topological_simpy.robot import Robot, TopoMap

RANDOM_SEED = 10
SIM_RT_FACTOR = 1.0
VERBOSE = True

random.seed(RANDOM_SEED)
numpy.random.seed(RANDOM_SEED)


if __name__ == "__main__":
    # config parameters, specified in rasberry-multisim.yaml, for VPICKER_CONFIG
    config_file = '/home/zuyuan/rasberry_ws/src/RASberry/rasberry_bringup/config/site_files/riseholme/polytunnel/transportation/picking_des.yaml'
    # get the config params
    config_params = topological_simpy.config_utils_mimic.get_mimic_des_params(config_file)

    # env = simpy.RealtimeEnvironment(factor=SIM_RT_FACTOR, strict=False) # todo: test later
    env = simpy.Environment()
    topo_map2_file = '/home/zuyuan/catkin_ws/src/topological_navigation/topological_navigation/maps/riseholme.tmap2'
    # tmap = TopoMap(topo_map2_file, env)   # TODO: fuse with old topo_graph
    # create the topo_graph
    topo_graph = topological_simpy.topo_mimic.TopologicalForkGraphMimic(config_params["n_polytunnels"],
                                                                        config_params["n_farm_rows"],
                                                                        config_params["n_topo_nav_rows"],
                                                                        config_params["second_head_lane"],
                                                                        env,
                                                                        topo_map2_file,
                                                                        VERBOSE)

    # set row_info, local_storage_nodes should be set separately
    if config_params["second_head_lane"]:
        topo_graph.set_row_info(config_params["pri_head_nodes"], config_params["row_nodes"], config_params["sec_head_nodes"])
    else:
        topo_graph.set_row_info(config_params["pri_head_nodes"], config_params["row_nodes"])
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

    robots = []   # todo: for what? import our robot generator, simpy version?

    local_storages = [simpy.Resource(env, capacity=n_pickers) for i in range(len(config_params["local_storage_nodes"]))]
    topo_graph.set_local_storages(local_storages, config_params["local_storage_nodes"])

    if config_params["use_cold_storage"]:
        cold_storage = simpy.Resource(env, capacity=n_pickers)
        topo_graph.set_cold_storage(cold_storage, config_params["cold_storage_node"])

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

        farm = topological_simpy.farm_mimic.FarmMimic(config_params["map_name"],
                                                      env,
                                                      config_params["n_topo_nav_rows"],
                                                      topo_graph,
                                                      robots,
                                                      pickers,
                                                      scheduling_policy,
                                                      config_params["with_robots"],
                                                      config_params["n_iteration"],
                                                      VERBOSE)

    until = 3000
    while env.peek() < until:
        try:
            env.step()
        except simpy.core.EmptySchedule:
            break
        else:
            pass

    finish_time_simpy = env.now
