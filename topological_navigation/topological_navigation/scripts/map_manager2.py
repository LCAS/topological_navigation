#!/usr/bin/env python3
"""
Created on Thu Nov  5 10:41:24 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""

import sys
import rclpy

from topological_navigation.manager2 import map_manager_2


def usage():
    print("\nPublishes Topological Maps:")
    print("\nFor loading a map:")
    print("\t rosrun topological_navigation map_manager2.py map_filename")
    print("\nFor creating a new map:")
    print("\t rosrun topological_navigation map_manager2.py -n map_filename")
    print("\n\n")


def main(args=None):
    
    load=True
    if '-h' in sys.argv or '--help' in sys.argv or len(sys.argv) < 2:
        usage()
        sys.exit(1)
    else:
        if '-n' in sys.argv:
            ind = sys.argv.index('-n')
            _map=sys.argv[ind+1]
            print("Creating new Map (%s)" %_map)
            load=False
        else:
            _map=sys.argv[1]


    rclpy.init(args=args)
    node = map_manager_2()
    
    node.init_map(filename=_map, load=load)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__' :
    main()
