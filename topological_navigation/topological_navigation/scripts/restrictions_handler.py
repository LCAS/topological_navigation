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

# Msg imports
from std_msgs.msg import String
from topological_navigation_msgs.msg import TopologicalMap as TMap



class Restrictor(Node):

    def __init__(self):
        super().__init__('restriction_handler')

        # Set initial eval restriction
        ir = Parameter('str', Parameter.Type.STRING, "")
        initial_restriction = self.get_parameter_or("~initial_restriction", ir).value
        self.eval = Str(initial_restriction) if initial_restriction else None
        self.logger.info(f"restrictor launched with initial restriction as: {initial_restriction}")

        # Setup publisher for map
        self.res_tmap2_pub = self.create_publisher(String, '~restricted_topological_map_2', 2)

        # Check if eval subscriber is needed to enable changes to eval restriction condition
        es = Parameter('bool', Parameter.Type.BOOL, True)
        enable_eval_sub = self.get_parameter_or("~enable_eval_sub", es).value
        if enable_eval_sub:
            self.res_eval_sub = self.create_subscription(String, '~restriction_evalator', self.py_eval)

        # Subscribe to map
        self.tmap = None
        self.tmap2_sub = self.create_subscription(String, '~topological_map_2', self.tmap_cb)

    def tmap_cb(self, msg):
        #Recieve and decode map, and send through filter is evaluation paramaters are already set
        self.tmap = yaml.safe_load(msg.data)
        self.logger.info(f"map recieved with {len(self.tmap['nodes'])} nodes")
        if self.eval:
            self.py_eval(self.eval)


    def py_eval(self, msg):
        """ This eval is compared with the restriction condition for the topological map, only those which match the evaluation remain.
        e.g. | node.restriction = "['short', 'tall']"
             | msg.data = "'short' in $"
             | eval( "'short' in $".replace('$', node.restriction) )
             | eval( "'short' in ['short', 'tall']" ) = True
        """
        self.logger.info(f"py_eval: {msg.data}")

        # Early exit for if tmap not available
        if not self.tmap:
            self.eval = msg
            self.logger.info("tmap not recieved, will apply eval once map arrives")
            return
        tmap = self.tmap
        nodes_to_keep = []

        # Quick pass to reduce evals for multilples of the same restriction condition
        condition = msg.data
        self.condition_results = {True:[], False:[]}

        # Filter nodes
        tmap['nodes'] = [n for n in tmap['nodes'] if self.check_outcome(condition, n['node']['restrictions_planning'])]
        self.logger.info(f"filter removed {len(self.tmap['nodes'])-len(tmap['nodes'])} nodes")

        # Filter edges
        kept_nodes = [n['node']['name'] for n in tmap['nodes']]
        for node in tmap['nodes']:
            if 'edges' in node['node']:
                node['node']['edges'] = [e for e in node['node']['edges'] if e['node'] in kept_nodes]
                node['node']['edges'] = [e for e in node['node']['edges'] if check_outcome(condition, e['restrictions_planning'])]

        # Save and publish new map
        self.tmap = tmap
        s = json.dumps(self.tmap)
        self.res_tmap2_pub.publish(Str(s))


    def check_outcome(self, query, restriction):
        # Format query and restriction together for eval
        check = query.replace('$', str(restriction))

        # Early exit for if eval already processed for previous component
        if check in self.condition_results[True]:  return True
        if check in self.condition_results[False]: return False

        # Process check condition
        result = eval(check)

        # Save result for early exit
        self.condition_results[result] += [check]

        return result



def main(args=None):
    rclpy.init(args=args)
    r = Restrictor(initial_restriction=ir, enable_eval_sub=es):

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
    msg_data = "'robot_short' in $ or $ == 'True'"
    query = msg_data.replace('$', node_restriction)
    print('restriction', node_restriction)
    print('condition', msg_data)
    print('full query', query)
    print('query result', eval(query))
    print('\n')

if __name__ == '__main__':
    main()
