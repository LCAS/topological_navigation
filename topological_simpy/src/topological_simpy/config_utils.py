#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import random
import yaml


def get_config_data(config_file):
    """read data from the config yaml file"""
    f_handle = open(config_file, "r")
    config_data = yaml.load(f_handle, Loader=yaml.FullLoader)
    f_handle.close()
    return config_data


def get_config_keys(config_file):
    """read data from config yaml file and extract the keys"""
    config_data = get_config_data(config_file)
    keys = []
    for item in config_data:
        keys.append(item)
    return keys


def check_fork_map_config(config_file):
    """check whether all config params required for fork_map generation are
    available"""
    # try to get the configuration parameters required for fork_map generation
    # and if any required parameter is not available throw an error
    config_keys = get_config_keys(config_file)
    req_params = ["map_name", "n_polytunnels", "n_farm_rows", "head_row_node_dist", "x_offset",
                  "y_offset", "row_node_dist", "row_length", "row_spacing", "second_head_lane"]

    for key in config_keys:
        if key in req_params:
            req_params.remove(key)
    return req_params


def check_des_config(config_file):
    """"Check whether all parameters required for the DES are available"""
    config_keys = get_config_keys(config_file)
    req_params = ["des_env", "n_pickers", "picker_picking_rate", "picker_transportation_rate",
                  "picker_max_n_trays", "picker_unloading_time", "tray_capacity", "yield_per_node",
                  "n_local_storages", "n_robots", "robot_transportation_rate",
                  "robot_max_n_trays", "robot_unloading_time"]
    for key in config_keys:
        if key in req_params:
            req_params.remove(key)
    return req_params


def param_list_to_dict(list_name, list_object, keys):
    """convert a list of values (list_object) to a dict with keys (keys)

    Keyword arguments:

    list_name -- string name of the list object for exception purposes
    list_object -- list of values
    keys -- list of keys to be used in dict (len same as that of list_object)
    """
    ideal_len = len(keys)
    if list_object.__class__ != list:
        msg = "%s must be a list of size %d" %(list_name, ideal_len)
        raise Exception(msg)

    if len(list_object) != ideal_len:
        msg = "%s must be a list of size %d" %(list_name, ideal_len)
        raise Exception(msg)
    dict_object = {keys[i]:list_object[i] for i in range(ideal_len)}
    return dict_object


def des_param_list_check(param, list_object, ideal_list_len, func="copy"):
    """check the size of a list of values (list_object) of parameter (param) and extend to a
    list of len ideal_list_len. use func to determine how this can be done.

    Keyword arguments:

    param --string name of the parameter for exception purposes
    list_object -- list of values
    ideal_list_len -- ideal length of the list
    func -- func to be used to extend list_object to len ideal_list_len (none, copy or gauss)
    """
    if list_object.__class__ != list:
        msg = "%s must be a list" %(param)
        raise Exception(msg)

    if func == "none":
        # there must be required number of values
        if len(list_object) != ideal_list_len:
            msg = "%s must be a list of size %d for func='none'" %(param, ideal_list_len)
            raise Exception(msg)
        new_list = list_object

    elif func == "copy":
        # copy single value for ideal_list_len
        if len(list_object) != 1:
            msg = "%s must be a list of size 1 for func='copy'" %(param)
            raise Exception(msg)
        new_list = [list_object[0] for i in range(ideal_list_len)]

    elif func == "gauss":
        if len(list_object) != 2:
            msg = "%s must be a list of size 2 for func='gauss'" %(param)
            raise Exception(msg)
        new_list = [random.gauss(list_object[0], list_object[1]) for i in range(ideal_list_len)]

    else:
        msg = "%s must be a list of (size %d & func='none'), (size 1 & func='copy') or (size 2 & func='gauss')" %(param, ideal_list_len)
        raise Exception(msg)
    return new_list


def graph_param_list_check(param, list_object, n_polytunnels, n_farm_rows,
                           n_topo_nav_rows, func):
    """function to extend graph parameters to a list of size n_topo_nav_rows from
    one or n_polytunnels values
    """
    # head_row_dose_dist, head_node_x, row_node_dist, row_length, row_spacing, yield_per_node
    if list_object.__class__ != list:
        msg = "%s must be a list" %(param)
        raise Exception(msg)

    if func == "none":
        if len(list_object) != n_topo_nav_rows:
            msg = "%s must be a list of size %d for func='none'" %(param, n_topo_nav_rows)
            raise Exception(msg)
        return des_param_list_check(param, list_object, n_topo_nav_rows, func="none")

    elif func == "copy":
        if len(list_object) == 1:
            return des_param_list_check(param, list_object, n_topo_nav_rows, func)
        elif len(list_object) == n_polytunnels:
            # same value for all rows of one polytunnel
            # get the list for each polytunnel and append them
            new_list = []

            for i in range(n_polytunnels):
                n_rows = n_farm_rows[i] + 1
                new_list += des_param_list_check(param, [list_object[i]], n_rows, func)
            return new_list

    elif func == "gauss":
        if len(list_object) != 2:
            msg = "%s must be a list of size 2 for func='gauss'" %(param)
            raise Exception(msg)
        return des_param_list_check(param, list_object, n_topo_nav_rows, func)

    else:
        raise Exception("func must be none, copy or gauss")


def graph_param_to_poly_list(param, list_object, n_polytunnels, func):
    """convert a given list of values to a list of size n_polytunnels

    Keyword arguments:

    param -- string name of parameter
    list_object -- original list
    n_polytunnels -- number of polytunnels
    func -- function to be used for extending to n_polytunnels values
    """
    if list_object.__class__ != list:
        msg = "%s must either be a list" %(param)
        raise Exception(msg)

    if n_polytunnels == 1:
        # func can be ignored as only one value is necessary
        if len(list_object) != 1:
            msg = "%s must be a list of size 1 for n_polytunnels = %d" %(param, n_polytunnels)
            raise Exception(msg)
        return list_object

    else:
        if func == "none":
            if len(list_object) != n_polytunnels:
                msg = "%s must be a list of size %d for func 'none'" %(param, n_polytunnels)
                raise Exception(msg)
            return list_object

        elif func == "copy":
            if len(list_object) != 1:
                msg = "%s must be a list of size 1 for func 'copy'" %(param)
                raise Exception(msg)
            return [list_object[0] for i in range(n_polytunnels)]


def get_fork_map_config_parameters(config_file):
    """get_fork_map_config_parameters: get the configuration parameters requried for
    making a fork map from a confg_file
    """
    missing_params = check_fork_map_config(config_file)
    if len(missing_params) != 0:
        msg = "Some required configuration parameters are missing: %s" %(missing_params)
        raise Exception(msg)

    config_data = get_config_data(config_file)

    map_name = config_data["map_name"]
    n_polytunnels = config_data["n_polytunnels"]
    assert n_polytunnels > 0

    n_farm_rows = graph_param_to_poly_list("n_farm_rows",
                                           config_data["n_farm_rows"]["value"],
                                           n_polytunnels,
                                           config_data["n_farm_rows"]["func"])

    # n_rows+1 picking rows are needed, all except first and last (in a polytunnel)
    # rows are forward and reverse. first and last are forward/reverse only
    n_topo_nav_rows = 0
    for i in range(n_polytunnels):
        n_topo_nav_rows += n_farm_rows[i] + 1

    head_row_node_dist = graph_param_list_check("head_row_node_dist",
                                                config_data["head_row_node_dist"]["value"],
                                                n_polytunnels, n_farm_rows, n_topo_nav_rows,
                                                config_data["head_row_node_dist"]["func"])

    x_offset = graph_param_list_check("x_offset",
                                      config_data["x_offset"]["value"], n_polytunnels,
                                      n_farm_rows, n_topo_nav_rows,
                                      config_data["x_offset"]["func"])

    y_offset = config_data["y_offset"]

    row_node_dist = graph_param_list_check("row_node_dist",
                                           config_data["row_node_dist"]["value"], n_polytunnels,
                                           n_farm_rows, n_topo_nav_rows,
                                           config_data["row_node_dist"]["func"])

    row_length = graph_param_list_check("row_length",
                                        config_data["row_length"]["value"], n_polytunnels,
                                        n_farm_rows, n_topo_nav_rows,
                                        config_data["row_length"]["func"])

    # row_spacing is assumed to be the spacing between the
    # farm rows or between the edge and a row, or between two rows
    # navigation rows should be at the middle of this spacing.
    row_spacing = graph_param_list_check("row_spacing",
                                         config_data["row_spacing"]["value"], n_polytunnels,
                                         n_farm_rows, n_topo_nav_rows,
                                         config_data["row_spacing"]["func"])

    second_head_lane = config_data["second_head_lane"]

    if "dist_to_cold_storage" in config_data:
        dist_to_cold_storage = config_data["dist_to_cold_storage"]

    config_params = {}
    config_params["map_name"] = map_name
    config_params["n_polytunnels"] = n_polytunnels
    config_params["n_farm_rows"] = n_farm_rows
    config_params["n_topo_nav_rows"] = n_topo_nav_rows
    config_params["head_row_node_dist"] = head_row_node_dist
    config_params["x_offset"] = x_offset
    config_params["y_offset"] = y_offset
    config_params["row_node_dist"] = row_node_dist
    config_params["row_length"] = row_length
    config_params["row_spacing"] = row_spacing
    config_params["second_head_lane"] = second_head_lane
    if "dist_to_cold_storage" in config_data:
        config_params["dist_to_cold_storage"] = dist_to_cold_storage

    return config_params


def get_des_config_parameters(config_file, n_pickers=None, n_robots=None):
    """get_des_config_parameters: Get the parameters for configuring the discrete event simulation
    from a config_file
    """
    # map related parameters read first
    config_params = get_fork_map_config_parameters(config_file)

    missing_params = check_des_config(config_file)
    if len(missing_params) != 0:
        msg = "Some required configuration parameters are missing: %s" %(missing_params)
        raise Exception(msg)

    config_data = get_config_data(config_file)

    n_polytunnels = config_params["n_polytunnels"]
    n_farm_rows = config_params["n_farm_rows"]
    n_topo_nav_rows = config_params["n_topo_nav_rows"]

    # remaining parameters
    des_env = config_data["des_env"]

    if n_pickers is None:
        n_pickers = config_data["n_pickers"]

    # picker parameters - des parameters
    picker_picking_rate = des_param_list_check("picker_picking_rate",
                                               config_data["picker_picking_rate"]["value"],
                                               n_pickers,
                                               config_data["picker_picking_rate"]["func"])

    picker_transportation_rate = des_param_list_check("picker_transportation_rate",
                                                      config_data["picker_transportation_rate"]["value"],
                                                      n_pickers,
                                                      config_data["picker_transportation_rate"]["func"])

    picker_max_n_trays = des_param_list_check("picker_max_n_trays",
                                              config_data["picker_max_n_trays"]["value"],
                                              n_pickers,
                                              config_data["picker_max_n_trays"]["func"])

    picker_unloading_time = des_param_list_check("picker_unloading_time",
                                                 config_data["picker_unloading_time"]["value"],
                                                 n_pickers,
                                                 config_data["picker_unloading_time"]["func"])

    tray_capacity = config_data["tray_capacity"]

    # yield - graph parameter
    yield_per_node = graph_param_list_check("yield_per_node",
                                            config_data["yield_per_node"]["value"], n_polytunnels,
                                            n_farm_rows, n_topo_nav_rows,
                                            config_data["yield_per_node"]["func"])

    n_local_storages = config_data["n_local_storages"]

    if n_robots is None:
        n_robots = config_data["n_robots"]

    # robot parameters - des parameters
    robot_transportation_rate = des_param_list_check("robot_transportation_rate",
                                                     config_data["robot_transportation_rate"],
                                                     n_robots)

    robot_max_n_trays = des_param_list_check("robot_max_n_trays",
                                             config_data["robot_max_n_trays"]["value"], n_robots,
                                             config_data["robot_max_n_trays"]["func"])

    robot_unloading_time = des_param_list_check("robot_unloading_time",
                                                config_data["robot_unloading_time"], n_robots)

    config_params["des_env"] = des_env
    config_params["n_pickers"] = n_pickers
    config_params["picker_picking_rate"] = picker_picking_rate
    config_params["picker_transportation_rate"] = picker_transportation_rate
    config_params["picker_max_n_trays"] = picker_max_n_trays
    config_params["picker_unloading_time"] = picker_unloading_time
    config_params["tray_capacity"] = tray_capacity
    config_params["yield_per_node"] = yield_per_node
    config_params["n_local_storages"] = n_local_storages
    config_params["n_robots"] = n_robots
    config_params["robot_transportation_rate"] = robot_transportation_rate
    config_params["robot_max_n_trays"] = robot_max_n_trays
    config_params["robot_unloading_time"] = robot_unloading_time

    return config_params


def isclose(a, b, rel_tol=1e-06, abs_tol=0.0):
    """to check two floats a and b are close (nearly equal)
    """
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


if __name__ == "__main__":
    import rospkg
    rospack = rospkg.RosPack()
    config_file = "../../config/des_config.yaml"

    config_params = get_des_config_parameters(config_file)
