# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date: 16 november 2023
# ----------------------------------

# Builtin imports
import yaml, json
from pprint import pprint

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy

# Msg imports
from std_msgs.msg import String
from topological_navigation_msgs.msg import TopologicalMap as TMap



class Restrictor(Node):

    def __init__(self):
        super().__init__('restriction_handler')

        # Declare Parameters
        self.declare_parameter('initial_restriction', "")
        self.declare_parameter('enable_eval_sub', True)
        ir = Parameter('str', Parameter.Type.STRING, "")
        es = Parameter('bool', Parameter.Type.BOOL, True)

        # Set initial eval restriction
        initial_restriction = self.get_parameter_or("initial_restriction", ir).value
        self.eval = str(initial_restriction) if initial_restriction else None
        self.get_logger().info(f"restrictor launched with initial restriction as: {self.eval}")

        # Setup publisher for map
        self.res_tmap2_pub = self.create_publisher(String, '~/restricted_topological_map_2', 2)

        # Check if eval subscriber is needed to enable changes to eval restriction condition
        enable_eval_sub = self.get_parameter_or("~/enable_eval_sub", es).value
        if enable_eval_sub:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.res_eval_sub = self.create_subscription(String, '~/restriction_evalator', self.py_eval, qos)

        # Subscribe to map
        self.tmap = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tmap2_sub = self.create_subscription(String, '/topological_map_2', self.tmap_cb, qos)

    def tmap_cb(self, msg):
        #Recieve and decode map, and send through filter is evaluation paramaters are already set
        self.tmap = yaml.safe_load(msg.data)
        self.get_logger().info(f"map recieved with {len(self.tmap['nodes'])} nodes")
        if self.eval:
            self.py_eval(String(data=self.eval))


    def py_eval(self, msg):
        """ This eval is compared with the restriction condition for the topological map, only those which match the evaluation remain.
        e.g. | node.restriction = "['short', 'tall']"
             | msg.data = "'short' in $"
             | eval( "'short' in $".replace('$', node.restriction) )
             | eval( "'short' in ['short', 'tall']" ) = True
        """
        self.get_logger().info(f"py_eval: {msg.data}")

        # Early exit for if tmap not available
        if not self.tmap:
            self.eval = msg
            self.get_logger().info("tmap not recieved, will apply eval once map arrives")
            return
        tmap = self.tmap
        nodes_to_keep = []

        # Quick pass to reduce evals for multilples of the same restriction condition
        condition = msg.data
        self.condition_results = {True:[], False:[]}

        # Filter nodes
        self.get_logger().info('\n\n\n')

        total_nodes = len(tmap['nodes'])
        tmap['nodes'] = [n for n in tmap['nodes'] if self.check_outcome(condition, n['node']['restrictions_planning'])]
        remaining_nodes = len(tmap['nodes'])
        self.get_logger().info(f"filter removed {total_nodes - remaining_nodes} nodes")

        # Filter edges
        kept_nodes = [n['node']['name'] for n in tmap['nodes']]
        total_edges = sum([len(n['node']['edges']) for n in tmap['nodes']])

        for node in tmap['nodes']:
            if 'edges' in node['node']:
                # Only allow edges which go to a kept node
                node['node']['edges'] = [e for e in node['node']['edges'] if e['node'] in kept_nodes]
                # Only allow edges which match the condition
                node['node']['edges'] = [e for e in node['node']['edges'] if self.check_outcome(condition, e['restrictions_planning'])]

        remaining_edges = sum([len(n['node']['edges']) for n in tmap['nodes']])
        self.get_logger().info(f"filter removed {total_edges-remaining_edges} edges")

        # Output results of each restriction evaluation
        self.get_logger().info("Restriction evaluations:")
        for c in self.condition_results[True]: self.get_logger().info(f"     True: {c}")
        for c in self.condition_results[False]: self.get_logger().info(f"    False: {c}")

        # Save and publish new map
        self.tmap = tmap
        s = json.dumps(self.tmap)
        self.res_tmap2_pub.publish(String(data=s))


    def check_outcome(self, query, restriction):
        # Format query and restriction together for eval
        check = query.replace('$', str(restriction))

        # Early exit for if eval already processed for previous component
        if check in self.condition_results[True]:  return True
        if check in self.condition_results[False]: return False

        # Process check condition
        result = eval(check)
        self.get_logger().info(f"checking: {check} = {result}")

        # Save result for early exit
        self.condition_results[result] += [check]

        return result



def main(args=None):
    rclpy.init(args=args)

    R = Restrictor()
    rclpy.spin(R)

    R.destroy_node()
    rclpy.shutdown()


def example(args=None):
    print('Example 1:')
    node_restriction = "['short', 'tall']"
    msg_data = "'short' in $"
    query = msg_data.replace('$', node_restriction)
    print('restriction', node_restriction)
    print('condition', msg_data)
    print('full query', query)
    print('query result', eval(query))
    print('\n')

    print('Example 2:')
    node_restriction = "robot_short & robot_tall"
    msg_data = "'robot_short' in '$'"
    query = msg_data.replace('$', node_restriction)
    print('restriction', node_restriction)
    print('condition', msg_data)
    print('full query', query)
    print('query result', eval(query))
    print('\n')

    print('Example 3:')
    node_restriction = "'True'"
    msg_data = "'robot_short' in '$' or '$' == 'True'"
    query = msg_data.replace('$', node_restriction)
    print('restriction', node_restriction)
    print('condition', msg_data)
    print('full query', query)
    print('query result', eval(query))
    print('\n')

if __name__ == '__main__':
    main()
