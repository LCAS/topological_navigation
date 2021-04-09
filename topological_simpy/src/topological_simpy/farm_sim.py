#!/usr/bin/env python


import topological_simpy.farm


class FarmSim(topological_simpy.farm.Farm):
    """Farm class definition"""

    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose):
        """Create a Farm object
        """
        super(FarmSim, self).__init__(name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose)

        self.action = self.env.process(self.scheduler_monitor())
