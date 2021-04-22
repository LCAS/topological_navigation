#!/usr/bin/env python
# ----------------------------------
# @author: ZuyuanZhu
# @email: zuyuanzhu@gmail.com
# @date: 22 Apr 2021
# ----------------------------------


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

    def monitor(self, pickers, robots):
        """
        monitor the position and mode of pickers and robots, if their statuses change,
        then update the visualising figure in the main script
        """
        agent_status = {}
        for picker in pickers:
            agent_status[picker.picker_id] = [picker.picker_id, picker.mode, picker.curr_node]
        for robot in robots:
            agent_status[robot.robot_id] = [robot.robot_id, robot.mode, robot.curr_node]

        return agent_status
