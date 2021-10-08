#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import os
import sys
import rospy
import matplotlib.pyplot

import topological_navigation_msgs.msg


class TopoMapPlotter(object):
    """
    """
    def __init__(self, fig_file, fig_width, fig_height):
        """
        """
        self.topo_map = None
        self.rec_map = False
        self.nodes = {}
        self.node_names = []
        rospy.loginfo("Waiting for topo_map")
        while not self.rec_map:
            self.topo_map_sub = rospy.Subscriber("/topological_map", topological_navigation_msgs.msg.TopologicalMap, self.topo_map_cb)
            rospy.sleep(0.5)
        rospy.loginfo("Received topo_map")

        self.fig_file = fig_file
        self.width = fig_width
        self.height = fig_height

    def topo_map_cb(self, msg):
        """
        """
        self.topo_map = msg
        for node in self.topo_map.nodes:
            self.nodes[node.name] = node
            self.node_names.append(node.name)
        self.rec_map = True

    def plot_map(self, strip_str=""):
        fig = matplotlib.pyplot.figure(figsize=(self.width, self.height))
        min_x = None
        max_x = None
        min_y = None
        max_y = None
        ax = fig.add_subplot(111)
        # node markers
        for node_name in self.node_names:
            x = self.nodes[node_name].pose.position.x
            y = self.nodes[node_name].pose.position.y
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
            x = self.nodes[node_name].pose.position.x
            y = self.nodes[node_name].pose.position.y
            _node_name = node_name
            if strip_str in node_name:
                _node_name = node_name.strip(strip_str)
            ax.text(x-x_delta/4.0, y + y_delta/4.0, _node_name, fontsize=10)

        for node_name in self.node_names:
            from_x = self.nodes[node_name].pose.position.x
            from_y = self.nodes[node_name].pose.position.y
            for edge in self.nodes[node_name].edges:
                to_node = edge.node
                to_x = self.nodes[to_node].pose.position.x
                to_y = self.nodes[to_node].pose.position.y
#                ax.plot([from_x, to_x], [from_y, to_y], color="black", linewidth=2)
                ax.arrow(from_x, from_y, to_x-from_x, to_y-from_y, color="black",
                         head_width=0.2 * min(x_delta, y_delta),
                         head_length=0.2 * min(x_delta, y_delta), length_includes_head=True,
                         visible=True)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_xlim(min_x - x_delta, max_x + x_delta)
        ax.set_ylim(min_y - y_delta, max_y + y_delta)
        matplotlib.pyplot.show()
        fig.savefig(self.fig_file)

if __name__ == "__main__":
    rospy.init_node("plot_topo_map")

    if len(sys.argv) < 4:
        rospy.loginfo("Usage is plot_topo_map path_to_plot_file fig_width_inches fig_height_inches")
    else:
        fig_file = os.path.abspath(sys.argv[1])

        try:
            map_plotter = TopoMapPlotter(fig_file, float(sys.argv[2]), float(sys.argv[3]))
        except:
            rospy.signal_shutdown("Couldn't create TopoMapPlotter object")
        else:
            map_plotter.plot_map(strip_str="WayPoint")

#    rospy.spin()
