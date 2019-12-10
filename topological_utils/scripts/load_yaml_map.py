#!/usr/bin/env python

import rospy
import yaml
import argparse

from topological_navigation.load_maps_from_yaml import YamlMapLoader
from std_srvs.srv import Empty


def load_yaml(filename):
    rospy.loginfo("loading %s"%filename)
    with open(filename, 'r') as f:
        return yaml.load(f)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("yaml_map", help="The yaml file or directory containing yaml files to insert into the datacentre", type=str)
    parser.add_argument("--pointset", help="Override the pointset name defined in the yaml before inserting it into datacentre, i.e. rename map. If a directory is given the pointset will be extended by an index to keep the maps separate.", type=str)
    parser.add_argument("-f", "--force", help="Override maps with the same name in the datacentre", action="store_true")
    parser.add_argument("--keep-alive", help="Keeps the node alive and advertises the service '/load_yaml_map/loaded' as soon as all the maps are stored in the datacentre. Can be used for testing in a launch file to trigger components that rely on maps being inserted in the datacentre first.", action="store_true")
    args, unknown = parser.parse_known_args() # Necessary due to roslaunch injecting rubbish arguments

    rospy.init_node('load_yaml_map')
    y = YamlMapLoader()

    data = y.read_maps(args.yaml_map)
    y.insert_maps(data=data, new_pointset=args.pointset, force=args.force)

    if args.keep_alive:
        rospy.Service("~loaded", Empty, None)
        rospy.spin()
