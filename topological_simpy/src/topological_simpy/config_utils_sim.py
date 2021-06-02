#!/usr/bin/env python
"""
----------------------------------
@author: gpdas, ZuyuanZhu
@date: 05-Feb-2018
@info: 26-Apr-2021, updates @ZuyuanZhu:
       1. add robot parameters
       2. remove rospy
----------------------------------
"""

import topological_simpy.config_utils as config


get_config_data = config.get_config_data


def check_mimic_des_params(config_data):
    """check_mimic_des_params - Check whether all parameters needed to run the long picking des
    """
    req_params = ["map_name", "n_polytunnels", "n_farm_rows",
                  "second_head_lane", "pri_head_nodes", "sec_head_nodes", "row_nodes", "yield_per_node",
                  "local_storage_nodes", "use_cold_storage", "cold_storage_node",
                  "base_station_nodes", "wait_nodes",
                  "with_robots", "n_iteration",
                  "picker_picking_rate", "picker_transportation_rate",
                  "picker_max_n_trays", "picker_unloading_time", "tray_capacity",
                  "robot_transportation_rate", "robot_max_n_trays", "robot_unloading_time"]
    missing_params = []
    for param in req_params:
        if param not in config_data:
            missing_params.append(param)
    redundant_params = []

    for key in config_data.keys():
        if key not in req_params:
            redundant_params.append(key)
    if len(redundant_params) is not 0:
        print("Not all parameters are used: %s" % redundant_params)

    if len(missing_params) != 0:
        msg = "Not all required parameters are not in the config_file"
        print(msg)
        print(missing_params)
        raise Exception(msg)
    return missing_params


def get_head_nodes(node_dict_str, node_dict, n_polytunnels, n_farm_rows):
    """
    """
    nodes = []
    # check all required info is available
    for poly_num in range(n_polytunnels):
        poly = "polytunnel_%02d" % (poly_num + 1)
        if poly not in node_dict:
            msg = "%s info missing from config_data['%s']" % (poly, node_dict_str)
            raise Exception(msg)
        elif len(node_dict[poly]) != n_farm_rows[poly_num] + 1:
            msg = "Not all %s row info available for %s" % (node_dict_str, poly)
            raise Exception(msg)
        else:
            for item in node_dict[poly]:
                nodes.append(item)
    return nodes


def get_row_nodes(row_nodes, n_polytunnels, n_farm_rows):
    """
    """
    nodes = []
    # check all required info is available
    for poly_num in range(n_polytunnels):
        poly = "polytunnel_%02d" % (poly_num + 1)
        if poly not in row_nodes:
            msg = "%s info missing from config_data['row_nodes']" % (poly)
            raise Exception(msg)
        else:
            if len(row_nodes[poly].keys()) != n_farm_rows[poly_num] + 1:
                msg = "Not all row_nodes are available for %s" % (poly)
                raise Exception(msg)
            else:
                # there are (n_farm_rows[poly_num] + 1) topo_nav_rows
                for row_num in range(n_farm_rows[poly_num] + 1):
                    nodes.append(row_nodes[poly]["row_%02d" % (row_num + 1)])
    return nodes


def get_mimic_des_params(config_file, n_pickers=None, n_robots=None):
    """get_des_config_parameters: Get the parameters for configuring the discrete event simulation
    from a config_file
    """
    # get config data
    config_data = get_config_data(config_file)
    # check for all params
    check_mimic_des_params(config_data)
    # place holder for all params
    config_params = {"map_name": config_data["map_name"], "n_polytunnels": config_data["n_polytunnels"]}
    # extract params
    # map-related
    assert config_params["n_polytunnels"] > 0
    config_params["n_farm_rows"] = config.graph_param_to_poly_list(
        "n_farm_rows", config_data["n_farm_rows"]["value"],
        config_params["n_polytunnels"], config_data["n_farm_rows"]["func"])

    # n_rows+1 picking rows are needed, all except first and last (in a polytunnel)
    # rows are forward and reverse. first and last are forward/reverse only
    if config_params["n_polytunnels"] == 0:
        config_params["n_topo_nav_rows"] = config_params["n_farm_rows"][0] + 1
    else:
        config_params["n_topo_nav_rows"] = 0
        for i in range(config_params["n_polytunnels"]):
            config_params["n_topo_nav_rows"] += config_params["n_farm_rows"][i] + 1
    # head nodes
    config_params["pri_head_nodes"] = get_head_nodes("pri_head_nodes",
                                                     config_data["pri_head_nodes"],
                                                     config_params["n_polytunnels"],
                                                     config_params["n_farm_rows"])
    config_params["second_head_lane"] = config_data["second_head_lane"]
    if config_params["second_head_lane"]:
        config_params["sec_head_nodes"] = get_head_nodes("sec_head_nodes",
                                                         config_data["sec_head_nodes"],
                                                         config_params["n_polytunnels"],
                                                         config_params["n_farm_rows"])
    # row_nodes
    config_params["row_nodes"] = get_row_nodes(config_data["row_nodes"],
                                               config_params["n_polytunnels"],
                                               config_params["n_farm_rows"])
    # yield per node
    config_params["yield_per_node"] = config.graph_param_list_check("yield_per_node",
                                                                    config_data["yield_per_node"][
                                                                        "value"],
                                                                    config_params["n_polytunnels"],
                                                                    config_params["n_farm_rows"],
                                                                    config_params["n_topo_nav_rows"],
                                                                    config_data["yield_per_node"][
                                                                        "func"])
    # storages
    config_params["local_storage_nodes"] = config_data["local_storage_nodes"]
    config_params["use_cold_storage"] = config_data["use_cold_storage"]
    if config_params["use_cold_storage"]:
        config_params["cold_storage_node"] = config_data["cold_storage_node"]
    else:
        config_params["cold_storage_node"] = None

    # des info
    config_params["with_robots"] = config_data["with_robots"]
    config_params["n_iteration"] = config_data["n_iteration"]

    # picker info
    config_params["picker_picking_rate"] = config.des_param_list_check("picker_picking_rate",
                                                                       config_data[
                                                                           "picker_picking_rate"][
                                                                           "value"],
                                                                       n_pickers,
                                                                       config_data[
                                                                           "picker_picking_rate"][
                                                                           "func"])

    config_params["picker_transportation_rate"] = config.des_param_list_check(
        "picker_transportation_rate",
        config_data["picker_transportation_rate"]["value"],
        n_pickers,
        config_data["picker_transportation_rate"]["func"])

    config_params["picker_max_n_trays"] = config.des_param_list_check("picker_max_n_trays",
                                                                      config_data[
                                                                          "picker_max_n_trays"][
                                                                          "value"],
                                                                      n_pickers,
                                                                      config_data[
                                                                          "picker_max_n_trays"][
                                                                          "func"])

    config_params["picker_unloading_time"] = config.des_param_list_check("picker_unloading_time",
                                                                         config_data[
                                                                             "picker_unloading_time"][
                                                                             "value"],
                                                                         n_pickers,
                                                                         config_data[
                                                                             "picker_unloading_time"][
                                                                             "func"])

    config_params["tray_capacity"] = config_data["tray_capacity"]

    # robot parameters - des parameters
    config_params['robot_transportation_rate'] = config.des_param_list_check("robot_transportation_rate",
                                                                             config_data["robot_transportation_rate"],
                                                                             n_robots)

    config_params['robot_max_n_trays'] = config.des_param_list_check("robot_max_n_trays",
                                                                     config_data["robot_max_n_trays"]["value"],
                                                                     n_robots,
                                                                     config_data["robot_max_n_trays"]["func"])

    config_params['robot_unloading_time'] = config.des_param_list_check("robot_unloading_time",
                                                                        config_data["robot_unloading_time"], n_robots)

    config_params['wait_nodes'] = config_data['wait_nodes']
    config_params['base_station_nodes'] = config_data['base_station_nodes']

    return config_params
