#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import os
import sys
import matplotlib.pyplot
import yaml


class TopoMapPlotter(object):
    """TopoMapPlotter class definition
    """
    def __init__(self, yaml_file, fig_file, fig_width, fig_height):
        """initialise TopoMapPlotter object

        Keyword arguments:

        yaml_file -- topomap in yaml format
        fig_file -- name of the map image file to be saved with extension
        fig_width -- width of map image file
        fig_height -- height of map image file
        """

        self.fig_file = fig_file
        self.width = fig_width
        self.height = fig_height

        f_handle = open(yaml_file, "r")
        self.topo_map = yaml.load(f_handle)
        f_handle.close()

        self.nodes = {}
        self.node_names = []
        self._extract_topo_map()


    def _extract_topo_map(self):
        """extract node infor from topomap
        """
        for node in self.topo_map:
            self.nodes[node["node"]["name"]] = node["node"]
            self.node_names.append(node["node"]["name"])

    def plot_map(self, strip_str=""):
        """plot into the topo map image file
        """
        fig = matplotlib.pyplot.figure(figsize=(self.width, self.height))
        min_x = None
        max_x = None
        min_y = None
        max_y = None
        ax = fig.add_subplot(111)
        # node markers
        for node_name in self.node_names:
            x = self.nodes[node_name]["pose"]["position"]["x"]
            y = self.nodes[node_name]["pose"]["position"]["y"]
            ax.plot(x, y, color="black", marker="o", markersize=6, markeredgecolor="r")

            if min_x is None:
                min_x = x
                max_x = x
                min_y = y
                max_y = y
            else:
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y

        x_delta = (max_x - min_x) * 0.1
        y_delta = (max_y - min_y) * 0.1
        # node names
        for node_name in self.node_names:
            x = self.nodes[node_name]["pose"]["position"]["x"]
            y = self.nodes[node_name]["pose"]["position"]["y"]
            _node_name = node_name
            if strip_str in node_name:
                _node_name = node_name.strip(strip_str)
            ax.text(x-x_delta/4.0, y + y_delta/4.0, _node_name, fontsize=10)

        for node_name in self.node_names:
            from_x = self.nodes[node_name]["pose"]["position"]["x"]
            from_y = self.nodes[node_name]["pose"]["position"]["y"]
            for edge in self.nodes[node_name]["edges"]:
                to_node = edge["node"]
                to_x = self.nodes[to_node]["pose"]["position"]["x"]
                to_y = self.nodes[to_node]["pose"]["position"]["y"]
#                ax.plot([from_x, to_x], [from_y, to_y], color="black", linewidth=2)
                ax.arrow(from_x, from_y, to_x-from_x, to_y-from_y, color="black",
                         head_width=0.2 * min(x_delta, y_delta),
                         head_length=0.2 * min(x_delta, y_delta), length_includes_head=True,
                         visible=True)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_xlim(min_x - x_delta, max_x + x_delta)
        ax.set_ylim(min_y - y_delta, max_y + y_delta)
        fig.savefig(self.fig_file)

if __name__ == "__main__":

    if len(sys.argv) < 5:
        rospy.loginfo("Usage is plot_yaml <path_to_yaml_topomap> <path_to_plot_file> <fig_width_inches> <fig_height_inches>")
    else:
        yaml_map = os.path.abspath(sys.argv[1])
        fig_file = os.path.abspath(sys.argv[2])

        try:
            map_plotter = TopoMapPlotter(yaml_map, fig_file, float(sys.argv[3]), float(sys.argv[4]))
        except:
            rospy.signal_shutdown("Couldn't create TopoMapPlotter object")
        else:
            map_plotter.plot_map(strip_str="WayPoint")

