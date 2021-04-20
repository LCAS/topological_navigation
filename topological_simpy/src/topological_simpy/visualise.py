#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import os
import random
import matplotlib.pyplot
import rospy


class VisualiseAgents(object):
    """A class to animate agent locations in matplotlib"""
    def __init__(self, topo_graph, robots, pickers, policy, show_cs=False,
                 save_random=False, trial=0):
        """initialise the Visualise_Agents class

        Keyword arguments:

        farm -- rasberry_des.farm.Farm object
        pickers == list of rasberry_des.picker.Picker objects
        """
        self.pickers = pickers
        self.n_pickers = len(self.pickers)
        self.robots = robots
        self.n_robots = len(self.robots)
        self.graph = topo_graph
        self.policy = policy

        self.show_cold_storage = show_cs    # should the cold storage be shown

        self.save_fig = save_random     # images are saved in the home directory

        self.trial = trial

        if self.save_fig:
            self.fig_name_base = os.path.expanduser("~")+"/P%d_R%d_S%s_T%d_" %(self.n_pickers,
                                                                               self.n_robots,
                                                                               self.policy,
                                                                               self.trial)

        self.fig = matplotlib.pyplot.figure(figsize=(9.6, 8), dpi=100)

        self.ax = self.fig.add_subplot(111, frameon=True)

        self.font = {'family': 'serif', 'color':  'red', 'weight': 'bold', 'size': 9,}

        self.static_lines = []
        self.picker_position_lines = []
        self.picker_status_texts = []
        self.robot_position_lines = []
        self.robot_status_texts = []

        self.init_plot()

        # show the plot
        matplotlib.pyplot.show(block=False)

    def close_plot(self):
        """close plot"""
        matplotlib.pyplot.close(self.fig)

    def init_plot(self):
        """Initialise the plot frame"""
        farm_rows_x, farm_rows_y = [], []
        nav_rows_x, nav_rows_y = [], []
        nav_row_nodes_x, nav_row_nodes_y = [], []
        pri_head_lane_x, pri_head_lane_y = [], []
        pri_head_nodes_x, pri_head_nodes_y = [], []
        if self.graph.second_head_lane:
            sec_head_lane_x, sec_head_lane_y = [], []
            sec_head_nodes_x, sec_head_nodes_y = [], []
        local_storage_x, local_storage_y = [], []
        local_storage_nodes = []
        cold_storage_node = None

        for i in range(self.graph.n_topo_nav_rows):
            row_id = self.graph.row_ids[i]
            pri_head_node = self.graph.get_node(self.graph.head_nodes[row_id][0])
            pri_head_nodes_x.append(pri_head_node.pose.position.x)
            pri_head_nodes_y.append(pri_head_node.pose.position.y)
            if self.graph.second_head_lane:
                sec_head_node = self.graph.get_node(self.graph.head_nodes[row_id][1])
                sec_head_nodes_x.append(sec_head_node.pose.position.x)
                sec_head_nodes_y.append(sec_head_node.pose.position.y)
            for j in range(len(self.graph.row_nodes[row_id])):
                curr_node = self.graph.get_node(self.graph.row_nodes[row_id][j])
                if j == 0:
                    start_node = curr_node
                elif j == len(self.graph.row_nodes[row_id]) - 1:
                    last_node = curr_node
                nav_row_nodes_x.append(curr_node.pose.position.x)
                nav_row_nodes_y.append(curr_node.pose.position.y)

            if self.graph.second_head_lane:
                # from head_node to last_row_node of the row
                nav_rows_x.append((pri_head_node.pose.position.x, sec_head_node.pose.position.x))
                nav_rows_y.append((pri_head_node.pose.position.y, sec_head_node.pose.position.y))
            else:
                # from head_node to last_row_node of the row
                nav_rows_x.append((pri_head_node.pose.position.x, last_node.pose.position.x))
                nav_rows_y.append((pri_head_node.pose.position.y, last_node.pose.position.y))

            # primary head lane
#            if (i == 0) or (i == self.graph.n_topo_nav_rows - 1):
            pri_head_lane_x.append(pri_head_node.pose.position.x)
            pri_head_lane_y.append(pri_head_node.pose.position.y)

            # secondary head lane
            if self.graph.second_head_lane:
#                if (i == 0) or (i == self.graph.n_topo_nav_rows - 1):
                sec_head_lane_x.append(sec_head_node.pose.position.x)
                sec_head_lane_y.append(sec_head_node.pose.position.y)

            # farm rows
            if i < self.graph.n_topo_nav_rows - 1:
                curr_row_id = self.graph.row_ids[i]
                next_row_id = self.graph.row_ids[i + 1]
                if not (curr_row_id in self.graph.half_rows and next_row_id in self.graph.half_rows):
                    curr_row_start_node = self.graph.get_node(self.graph.row_nodes[curr_row_id][0])
                    curr_row_last_node = self.graph.get_node(self.graph.row_nodes[curr_row_id][-1])
                    next_row_start_node = self.graph.get_node(self.graph.row_nodes[next_row_id][0])
                    next_row_last_node = self.graph.get_node(self.graph.row_nodes[next_row_id][-1])
                    start_node_x = curr_row_start_node.pose.position.x + 0.5 * (next_row_start_node.pose.position.x - curr_row_start_node.pose.position.x)
                    start_node_y = curr_row_start_node.pose.position.y + 0.5 * (next_row_start_node.pose.position.y - curr_row_start_node.pose.position.y)
                    last_node_x = curr_row_last_node.pose.position.x + 0.5 * (next_row_last_node.pose.position.x - curr_row_last_node.pose.position.x)
                    last_node_y = curr_row_last_node.pose.position.y + 0.5 * (next_row_last_node.pose.position.y - curr_row_last_node.pose.position.y)

                    farm_rows_x.append((start_node_x, last_node_x))
                    farm_rows_y.append((start_node_y, last_node_y))

            if self.graph.local_storage_nodes[row_id] not in local_storage_nodes:
                local_storage_nodes.append(self.graph.local_storage_nodes[row_id])
                node_obj = self.graph.get_node(local_storage_nodes[-1])
                local_storage_x.append(node_obj.pose.position.x)
                local_storage_y.append(node_obj.pose.position.y)

            if self.graph.cold_storage_node is not None:
                cold_storage_node = self.graph.cold_storage_node
                node_obj = self.graph.get_node(cold_storage_node)
                cold_storage_x = node_obj.pose.position.x
                cold_storage_y = node_obj.pose.position.y

        if not self.show_cold_storage:
            min_x = min(min(nav_rows_x[0]), min(farm_rows_x[0]))
            max_x = max(max(nav_rows_x[-1]), max(farm_rows_x[-1]))
            min_y = min(min(nav_rows_y[0]), min(farm_rows_y[0]))
            max_y = max(max(nav_rows_y[-1]), max(farm_rows_y[-1]))

            # limits of the axes
            self.ax.set_xlim(min_x - 1, max_x + 1)
            self.ax.set_ylim(min_y - 1, max_y + 1)
        else:
            min_x = min(min(nav_rows_x[0]), min(farm_rows_x[0]), cold_storage_x)
            max_x = max(max(nav_rows_x[-1]), max(farm_rows_x[-1]), cold_storage_x)
            min_y = min(min(nav_rows_y[0]), min(farm_rows_y[0]), cold_storage_y)
            max_y = max(max(nav_rows_y[-1]), max(farm_rows_y[-1]), cold_storage_y)

            # limits of the axes
            self.ax.set_xlim(min_x - 5, max_x + 5)
            self.ax.set_ylim(min_y - 5, max_y + 5)

#        self.fig.set_figheight((max_y - min_y + 2)*2)
#        self.fig.set_figwidth((max_x - min_x + 2)*2)

        # static objects - nodes
        # nav_rows
        for i, item in enumerate(zip(nav_rows_x, nav_rows_y)):
            self.static_lines.append(self.ax.plot(item[0], item[1],
                                                  color="black", linewidth=4)[0])
        # farm_rows
        for i, item in enumerate(zip(farm_rows_x, farm_rows_y)):
            self.static_lines.append(self.ax.plot(item[0], item[1],
                                                  color="green", linewidth=4)[0])
        # primary head lane
        self.static_lines.append(self.ax.plot(pri_head_lane_x, pri_head_lane_y,
                                              color="black", linewidth=4)[0])
        # secondary head lane
        if self.graph.second_head_lane:
            self.static_lines.append(self.ax.plot(sec_head_lane_x, sec_head_lane_y,
                                                  color="black", linewidth=4)[0])
        # nav_row_nodes
        self.static_lines.append(self.ax.plot(nav_row_nodes_x, nav_row_nodes_y,
                                              color="black", marker="o", markersize=6,
                                              linestyle="none")[0])
        # pri_head_lane_nodes
        self.static_lines.append(self.ax.plot(pri_head_nodes_x, pri_head_nodes_y,
                                              color="black", marker="o", markersize=6,
                                              linestyle="none")[0])
        # sec_head_lane_nodes
        if self.graph.second_head_lane:
            self.static_lines.append(self.ax.plot(sec_head_nodes_x, sec_head_nodes_y,
                                                  color="black", marker="o", markersize=6,
                                                  linestyle="none")[0])
        # local storages
        self.static_lines.append(self.ax.plot(local_storage_x, local_storage_y,
                                              color="black", marker="s", markersize=12,
                                              markeredgecolor="r", linestyle="none")[0])
        # cold_storage
        if self.show_cold_storage and cold_storage_node is not None:
            self.static_lines.append(self.ax.plot(cold_storage_x, cold_storage_y,
                                                  color="black", marker="8", markersize=12,
                                                  markeredgecolor="r", linestyle="none")[0])
            self.static_lines.append(self.ax.plot([pri_head_nodes_x[0], cold_storage_x],
                                                  [pri_head_nodes_y[0], cold_storage_y],
                                                  color="black", linewidth=4)[0])

        # dynamic objects - pickers and robots
        # pickers
        for i in range(self.n_pickers):
            picker = self.pickers[i]
            picker_id = picker.picker_id
            if picker.curr_node is not None:
                curr_node_obj = self.graph.get_node(picker.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.

            self.picker_position_lines.append(self.ax.plot(x, y,
                                                           color="b", marker="o",
                                                           markersize=24,
                                                           markeredgecolor="b",
                                                           linestyle="none")[0])
            self.picker_status_texts.append(self.ax.text(x - 2.9, y + 0.5,
                                                         "P_%s:%d" %(picker_id[-2:], picker.mode),
                                                         fontdict=self.font))
        # robots
        for i in range(self.n_robots):
            robot = self.robots[i]
            robot_id = robot.robot_id
            if robot.curr_node is not None:
                curr_node_obj = self.graph.get_node(robot.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.

            self.robot_position_lines.append(self.ax.plot(x, y,
                                                          color="#8b12b3", marker="^",
                                                          markersize=24,
                                                          markeredgecolor="#8b12b3",
                                                          linestyle="none")[0])
            self.robot_status_texts.append(self.ax.text(x + 0.6, y + 0.5,
                                                        "R_%s:%d" %(robot_id[-2:], robot.mode),
                                                        fontdict=self.font))
        self.fig.canvas.draw()
        if self.save_fig:
            self.fig.savefig(self.fig_name_base+"%d.svg" %(rospy.get_time()*10**6))

        return (self.static_lines + self.picker_position_lines +
                self.picker_status_texts + self.robot_position_lines + self.robot_status_texts)

    def update_plot(self):
        """update the positions of the dynamic objects"""
        for i in range(self.n_pickers):
            picker = self.pickers[i]
            if picker.curr_node is not None:
                curr_node_obj = self.graph.get_node(picker.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.

            self.picker_position_lines[i].set_data(x, y)
            self.picker_status_texts[i].set_text("P_%s:%d" %(picker.picker_id[-2:], picker.mode))
            self.picker_status_texts[i].set_position((x - 2.9, y + 0.5))

        for i in range(self.n_robots):
            robot = self.robots[i]
            if robot.curr_node is not None:
                curr_node_obj = self.graph.get_node(robot.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.
            self.robot_position_lines[i].set_data(x, y)
            self.robot_status_texts[i].set_text("R_%s:%d" %(robot.robot_id[-2:], robot.mode))
            self.robot_status_texts[i].set_position((x + 1.0, y + 0.5))

        self.fig.canvas.draw()

        if self.save_fig:
            if random.random() < 0.001:
                self.fig.savefig(self.fig_name_base + "%d.svg" %(rospy.get_time()*10**6))

        return (self.static_lines + self.picker_position_lines + self.picker_status_texts +
                self.robot_position_lines + self.robot_status_texts)
