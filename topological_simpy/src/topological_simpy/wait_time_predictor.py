#!/usr/bin/env python
# ----------------------------------
# @author: ZuyuanZhu
# @email: zuyuanzhu@gmail.com
# @date: 03-06-2021
# @info: A class to predict how long a robot should wait before the (propose to) requested node being released.
# ----------------------------------


import topological_simpy.robot_sim


class WaitTimePredictor(object):
    """
    A class to predict how long a robot should wait before the (propose to) requested node being released.
    """
    def __init__(self):
    # avoid using new class to save data copy time
        pass
