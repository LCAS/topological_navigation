# -*- coding: utf-8 -*-
"""
Created on Wed Nov  4 10:56:07 2015

@author: cdondrup
"""

import rospy
import yaml
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from topological_navigation_msgs.msg import TopologicalNode
from topological_navigation.topological_map import topological_map
import os


class YamlMapLoader(object):
    def __init__(self):
        self.msg_store = MessageStoreProxy(collection='topological_maps')

    def get_maps(self):
        """
        Queries the database and returns details of the available topological maps.
        :return: A dictionary where each key is the name of a topological map and each
        item is a dictionary of details. Details are:
           "number_nodes" ; integer
            "edge_actions" : list of action server names used for traversal
            "last_modified" : datetime.datetime object for the last time a node was inserted
        """
        maps = dict()

        nodes = self.msg_store.query(TopologicalNode._type)

        for node in nodes:
            pointset = node[1]["pointset"]
            if not maps.has_key(pointset):
                maps[pointset] = {"number_nodes": 0, "edge_actions": set(), "last_modified": ""}
            maps[pointset]["number_nodes"] += 1
            if (maps[pointset]["last_modified"] == "" or
                        node[1]["inserted_at"] > maps[pointset]["last_modified"]):
                maps[pointset]["last_modified"] = node[1]["inserted_at"]
            for edge in node[0].edges:
                maps[pointset]["edge_actions"].add(edge.action)

        return maps

    def _load_yaml(self, filename):
        rospy.loginfo("loading %s"%filename)
        with open(filename, 'r') as f:
            return yaml.load(f)

    def read_maps(self, p):
        data = []
        if os.path.isdir(p):
            for f in os.listdir(p):
                if f.endswith(".yaml"):
                    data.append(self._load_yaml(p+'/'+f))
        else:
            data.append(self._load_yaml(p))
        return data

    def insert_maps(self, data, new_pointset=None, force=False):
        current_maps = self.get_maps()
        for idx, tmap in enumerate(data):
            pointset = None
            if new_pointset != None: # If there are more than one map, it takes the custom pointset and appends an index
                pointset = new_pointset+str(idx+1) if len(data) > 1 else new_pointset
            first_node = True
            for i in tmap:
                try:
                    meta = i['meta']
                    meta['pointset'] = pointset if pointset != None else meta['pointset']
                    if meta['pointset'] in current_maps and first_node:
                        first_node = False
                        if not force:
                            rospy.logwarn("Map '%s' already in datacentre, skipping! Use -f to force override or change pointset name with --pointset" % meta['pointset'])
                            break
                        else:
                            topo_map = topological_map(meta['pointset'])
                            topo_map.delete_map()
                    elif first_node:
                        first_node = False
                        rospy.loginfo("Inserting map: %s" % meta['pointset'])
                    msgv = dc_util.dictionary_to_message(i['node'], TopologicalNode)
                    self.msg_store.insert(msgv,meta)
                except TypeError:
                    pass # Not a topo map