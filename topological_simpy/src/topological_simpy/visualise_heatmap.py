#!/usr/bin/env python
# ----------------------------------
# @author: ZuyuanZhu
# @email: zuyuanzhu@gmail.com
# @date: 5 Oct 2021
# ----------------------------------


import random
import os
from datetime import datetime
import matplotlib.pyplot
import numpy
import pandas
import seaborn
# import rospy
import yaml
from scandir import scandir


def list_directories(path):
    dir_list = []
    for entry in scandir(path):
        if entry.is_dir() and not entry.is_symlink():
            dir_list.append(entry.path)
            dir_list.extend(list_directories(entry.path))
    return dir_list


def get_data(config_file):
    """read data from the config yaml file"""
    f_handle = open(config_file, "r")
    config_data = yaml.full_load(f_handle)
    f_handle.close()
    return config_data


class VisualiseHeatmap(object):
    def __init__(self, data_path_, file_type_):
        self.data_path = data_path_
        self.entries_list = []
        self.file_type = file_type_
        self.heatmap_data = None
        self.heatmap_index = None
        self.heatmap_columns = None

        self.heatmap_values = []
        self.heatmap_node_pose_x_list = []
        self.heatmap_node_pose_y_list = []
        self.map_name = ''
        self.trial = None
        self.n_deadlock = None
        self.simulation_time = 0.0

        self.show_cbar = True
        self.fig, self.ax = matplotlib.pyplot.subplots(2, 1, figsize=(16, 10), sharex=False, sharey=False)

        self.init_plot()

    def close_plot(self):
        """close plot"""
        self.fig.savefig(self.data_path + datetime.now().isoformat().replace(":", "_") + "redo.eps")

        # Save just the portion _inside_ the second axis's boundaries
        extent = self.ax.get_window_extent().transformed(self.fig.dpi_scale_trans.inverted())
        self.fig.savefig(self.data_path + datetime.now().isoformat().replace(":", "_") + "redo_heatmap.eps",
                         bbox_inches=extent.expanded(1.22, 1.24))

        matplotlib.pyplot.close(self.fig)

    def get_folder_files(self, data_path_):
        entries_list = []
        with os.scandir(data_path) as entries:
            for entry in entries:
                if entry.is_file():
                    entries_list.append(entry.name)
        entries_list.sort()
        self.entries_list = entries_list

    def load_data(self, data_path_, entries_list, file_type_):
        heatmap_data = []
        for file_name in entries_list:
            if file_type_ in file_name:
                heatmap_data.append(get_data(data_path_ + "/" + file_name))
                print(file_name)

        # heatmap_data = sorted(heatmap_data, key=lambda data: data["trial"])
        # self.heatmap_data = heatmap_data
        self.heatmap_values = heatmap_data[0]["heatmap_values"]
        self.heatmap_node_pose_x_list = heatmap_data[0]["heatmap_node_pose_x_list"]
        self.heatmap_node_pose_y_list = heatmap_data[0]["heatmap_node_pose_y_list"]
        self.map_name = heatmap_data[0]["map_name"]
        self.trial = heatmap_data[0]["trial"]
        self.n_deadlock = heatmap_data[0]["n_deadlock"]
        self.simulation_time = heatmap_data[0]["simulation_time"]

    def init_plot(self):
        self.get_folder_files(self.data_path)
        self.load_data(self.data_path, self.entries_list, self.file_type)

        node_pose_x = numpy.array(self.heatmap_node_pose_x_list)
        node_pose_y = numpy.array(self.heatmap_node_pose_y_list)

        max_x = max(self.heatmap_node_pose_x_list)
        min_x = min(self.heatmap_node_pose_x_list)
        max_y = max(self.heatmap_node_pose_y_list)
        min_y = min(self.heatmap_node_pose_y_list)
        data = numpy.zeros((max_x - min_x,
                            max_y - min_y))

        for i, x in enumerate(node_pose_x):
            for j, y in enumerate(node_pose_y):
                if i == j:
                    data[x - min_x - 1, y - min_y - 1] = self.heatmap_values[i]
                    break

        df = pandas.DataFrame(data,
                              index=numpy.linspace(min_x, max_x - 1, max_x - min_x, dtype='int'),
                              columns=numpy.linspace(min_y, max_y - 1, max_y - min_y, dtype='int'))

        seaborn.set(font_scale=1.6)

        # only initialise color bar once, then don't update it anymore
        if self.show_cbar:
            # get sharp grid back by removing rasterized=True, and save fig as svg format
            self.ax = seaborn.heatmap(df, cbar=True, rasterized=True)
            self.show_cbar = False
        else:
            # get sharp grid back by removing rasterized=True, and save fig as svg format
            self.ax = seaborn.heatmap(df, cbar=False, rasterized=True)
        # matplotlib.rcParams.update({'font.size': 22})
        # self.ax[1].set(xlabel='Node pose y', ylabel='Node pose x')
        self.ax.set_xlabel('Node pose y', fontsize=20)
        self.ax.set_ylabel('Node pose x', fontsize=20)

        self.fig.canvas.draw()

        self.close_plot()


if __name__ == "__main__":

    folder = "2021_Oct_06_11_45_50"
    data_folder_path = "/home/zuyuan/des_simpy_logs/"
    data_path = data_folder_path + folder
    file_type = 'heatmap.yaml'
    vis = VisualiseHeatmap(data_path, file_type)

