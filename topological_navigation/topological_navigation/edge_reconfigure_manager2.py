#!/usr/bin/env python
"""
Created on Tue Nov 5 22:02:24 2023
@author: Geesara Kulathunga (ggeesara@gmail.com)

"""
#############################################################################################################
import rclpy
from topological_navigation_msgs.srv import ReconfAtEdges
from topological_navigation.scripts.param_processing import ParameterUpdaterNode
from rclpy import Parameter 
from rclpy.callback_groups import ReentrantCallbackGroup 

class EdgeReconfigureManager(rclpy.node.Node):
    def __init__(self):
        super().__init__("edge_conguration_manager")
        self.active = False
        self.edge = {}
        self.initial_config = {}                    
        self.edge_config = {}
        self.namespaces = []
        self.current_edge_group = "none"
        self.declare_parameter("/edge_nav_reconfig_groups", Parameter.Type.STRING_ARRAY)
        self.edge_groups = self.get_parameter_or("/edge_nav_reconfig_groups", Parameter('str', Parameter.Type.STRING_ARRAY, [])).value
        self.callback_group = ReentrantCallbackGroup()
        self.reconf_at_edges = self.create_client(ReconfAtEdges, '/reconf_at_edges')

        
    def register_edge(self, edge):
        self.active = False
        self.edge = edge
        namespaces = []
        if "config" in edge:
            namespaces = [param["namespace"] for param in edge["config"]]
        self.namespaces = list(set(namespaces))
        if self.namespaces:
            self.active = True
            self.get_logger().info("Edge Reconfigure Manager: Processing edge {}".format(edge["edge_id"]))
        
        
    def initialise(self):
        
        self.initial_config = {}                    
        self.edge_config = {}
        for namespace in self.namespaces:
            self.get_logger().info("Edge Reconfigure Manager: Getting initial configuration for {}".format(namespace))
            client = ParameterUpdaterNode(namespace)
            try:
                config = client.get_params()
            except Exception as e:
                self.get_logger().warning("Edge Reconfigure Manager: Caught service exception: {}".format(e))
                continue
                
            self.initial_config[namespace] = {}
            self.edge_config[namespace] = {}

            for param in self.edge["config"]:
                if param["namespace"] == namespace:
                    self.edge_config[namespace][param["name"]] = param["value"]

                    reset = True if "reset" not in param else param["reset"]
                    if reset:
                        self.initial_config[namespace][param["name"]] = config[param["name"]]


    def reconfigure(self):
        """
        If using the new map then edge reconfigure is done using settings in the map.
        """
        for namespace in self.edge_config:
            self.get_logger().info("Edge Reconfigure Manager: Setting {} = {}".format(namespace, self.edge_config[namespace]))            
            self.update(namespace, self.edge_config[namespace])
                    
                    
    def _reset(self):
        """
        Used to reset edge params to their default values when the action has completed (only if using the new map)
        """
        for namespace in self.initial_config:
            if self.initial_config[namespace]:
                self.get_logger().info("Edge Reconfigure Manager: Resetting {} = {}".format(namespace, self.initial_config[namespace]))
                self.update(namespace, self.initial_config[namespace])
                
                
    def update(self, namespace, params):
        client = ParameterUpdaterNode(namespace)
        try:
            client.get_params()
            client.set_params(params)
        except Exception as e:
            self.get_logger().warning("Edge Reconfigure Manager: Caught service exception: {}".format(e))
                    
                    
    def srv_reconfigure(self, edge_id):
        """
        If using the old map then edge reconfigure is done via a service.
        """
        edge_group = "none"
        for group in self.edge_groups:
            self.get_logger().info("Check Edge: ", edge_id, "in ", group)
            if edge_id in self.edge_groups[group]["edges"]:
                edge_group = group
                break

        self.get_logger().info("current group: ", self.current_edge_group)
        self.get_logger().info("edge group: ", edge_group)

        if edge_group is not self.current_edge_group:
            self.get_logger().info("RECONFIGURING EDGE: ", edge_id)
            self.get_logger().info("TO ", edge_group)
            self.req_get = ReconfAtEdges.Request()
            self.req_get.edge_id = edge_id 
            self.future = self.reconf_at_edges.call_async(self.req_get)
            while rclpy.ok():
                rclpy.spin_once(self)
                if self.future.done():
                    try:
                        response = self.future.result()
                        if response.success:
                            self.current_edge_group = edge_group 
                            break 
                    except Exception as e:
                        self.get_logger().error("Error while reconfiguring the edge {}".format(e))
                        pass
        
############################################################################################################# 