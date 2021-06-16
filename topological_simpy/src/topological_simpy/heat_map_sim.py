#!/usr/bin/env python
"""
----------------------------------
@author: ZuyuanZhu
@email: zuyuanzhu@gmail.com
@date: 11-06-2021
@info: A class to analyse the topological node usage frequency. The frequency is used to create a heat map and used as
        weight when computing the distance cost of route. If the nodes of a route are frequently used by now, then this
        route would be relatively expensive. The principle is that we encourage to use less frequently used route nodes.
----------------------------------
"""

import topological_simpy.robot_sim


class HeatMap(object):
    """
    A class to analyse the topological node usage frequency.
    """
    def __init__(self):
        pass
