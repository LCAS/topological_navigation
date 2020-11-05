#!/usr/bin/env python
"""
Created on Thu Nov  5 10:41:24 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import rospy, argparse
from topological_navigation.manager2 import map_manager_2
    
    
if __name__ == '__main__' :
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str, default="new_map", help="name for new map")
    parser.add_argument("--metric_map", type=str, default="map_2d", help="metric map")       
    parser.add_argument("--pointset", type=str, default="new_map", help="pointset of the new map")       
    parser.add_argument("--transformation", type=str, default="default", help="metric map to topomap tf transform")
    parser.add_argument("--filename", type=str, default="", help="map filename")
    parser.add_argument("--load", type=lambda x: (str(x).lower() == 'true'), default=True, help="load existing map")
    
    
    rospy.init_node("topological_map_manager_2")
    args = parser.parse_args()
    mm2 = map_manager_2(args.name, args.metric_map, args.pointset, args.transformation, args.filename, args.load)
    rospy.spin()
#########################################################################################################