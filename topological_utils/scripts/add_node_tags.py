#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 12 20:21:38 2019

@author: gpdas

"""

import yaml
import sys
import os


def load_data_from_yaml(file_path):
    assert os.path.exists(file_path)

    f_handle = open(file_path, "r")
    f_data = yaml.load(f_handle)
    f_handle.close()
    return f_data


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print ("Usage: add_node_tags.py <in_file> <out_file> <config_file>")
    else:
        in_file = sys.argv[1]
        out_file = sys.argv[2]
        cfg_file = sys.argv[3]

        topo_map = load_data_from_yaml(in_file)

        tags = load_data_from_yaml(cfg_file)
        node_tag = {}
        for tag in tags:
            for node in tags[tag]:
                node_tag[node] = tag

        for node in topo_map:
            if node["meta"]["node"] in node_tag:
                if "tag" in node["meta"]:
                    if node_tag[node["meta"]["node"]] not in node["meta"]["tag"]:
                        node["meta"]["tag"].append(node_tag[node["meta"]["node"]])
                else:
                    node["meta"]["tag"] = [node_tag[node["meta"]["node"]]]

        with open(out_file, 'w') as f_out:
            yaml.dump(topo_map, f_out, default_flow_style=False)
        print("Successfully created the new file with node tags")
