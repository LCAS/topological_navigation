#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from topological_navigation.policy_marker import PoliciesVis
from topological_navigation.topomap_marker import TopologicalVis


def main(args=None):
    rclpy.init(args=args)

    TV = TopologicalVis()
    PV = PoliciesVis()

    executor = MultiThreadedExecutor()
    executor.add_node(TV)
    executor.add_node(PV)
    executor.spin()

    TV.destroy_node()
    PV.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


