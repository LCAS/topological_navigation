#!/usr/bin/env python


import topological_simpy.farm


class FarmSim(topological_simpy.farm.Farm):
    """FarmSim class definition
    FarmSim is the extension of Farm class object, managing the processes of robots and pickers
    """

    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose):
        """inherit from Farm object
        """
        super(FarmSim, self).__init__(name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose)

        self.action = self.env.process(self.scheduler_monitor())
