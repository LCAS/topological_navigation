#!/usr/bin/env python
import sys
import rospy
import yaml
from topological_navigation.load_maps_from_yaml import YamlMapLoader
from mongodb_store.message_store import MessageStoreProxy
from topological_navigation_msgs.msg import TopologicalNode
from topological_navigation_msgs.srv import AddNode
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('insert_empty_map')
    
    if len(sys.argv) != 2:
        rospy.logwarn("usage: insert_empty_map.py pointset_name")
        sys.exit(1)

    pointset = sys.argv[1]
    map_loader = YamlMapLoader()
    if pointset in map_loader.get_maps():
        rospy.logwarn("Map with name {0} already exists. Try another one.".format(sys.argv[1]))
        sys.exit(1)

    msg_store = MessageStoreProxy(collection='topological_maps')

    empty_node = TopologicalNode()
    empty_node.name = "temp_node"
    empty_node.map = "unused"
    empty_node.pointset = pointset

    meta = {}
    meta['pointset'] = pointset
    meta['map'] = "unused"
    meta['node'] = "temp_node"

    msg_store.insert(empty_node, meta)
