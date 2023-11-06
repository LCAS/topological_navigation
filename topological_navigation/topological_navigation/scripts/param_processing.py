from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import ParameterDescriptor, ParameterValue, ParameterType
from rclpy import Parameter 
import rclpy 
from rclpy.node import Node

from rclpy.callback_groups import ReentrantCallbackGroup 

class ParameterUpdaterNode(Node):     
    def __init__(self, server_name):
        super().__init__('param_update_node')
        self.callback_group = ReentrantCallbackGroup()
        self.cli_set_param = self.create_client(SetParameters, '/' + server_name + '/set_parameters', callback_group=self.callback_group)
        self.cli_list_params = self.create_client(ListParameters, '/' + server_name + '/list_parameters' , callback_group=self.callback_group)
        self.cli_get_params = self.create_client(GetParameters, '/' + server_name + '/get_parameters', callback_group=self.callback_group)
        while not self.cli_set_param.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again... {}'.format('/' + server_name + '/set_parameters'))
        self.get_logger().info('service /{} is available'.format(server_name))
        
        self.get_params()


    def set_params(self, params):
        self.req = SetParameters.Request()
        self.req.parameters = []
        for key, value in params.items():
            param_name, param_value = key, value 
            self.req.parameters.append(Parameter(name=param_name, value=param_value).to_parameter_msg())
        self.future = self.cli_set_param.call_async(self.req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result().results
                    if response[0].successful:
                        return True
                except Exception as e:
                    self.get_logger().error(" error while setting params {} ".format(e))
                    pass
                return False
            
    def list_params(self, ):
        self.req_list = ListParameters.Request()
        self.future = self.cli_list_params.call_async(self.req_list)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result().result 
                    if response:
                        return response.names
                except Exception as e:
                    self.get_logger().error("Error while getting paramer list {}".format(e))
                    pass
                return []
            
    def get_params(self, ):
        param_names = self.list_params()
        param_values = {}
        params = {}
        self.req_get = GetParameters.Request()
        self.req_get.names = param_names 
        self.future = self.cli_get_params.call_async(self.req_get)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                    if response:
                        param_values = response.values
                        break 
                except Exception as e:
                    self.get_logger().error("Error while getting paramer list {}".format(e))
                    pass
        if param_values:
            for key, value in zip(param_names, param_values):
                params[key] = value  
        return params 


if __name__ == '__main__':
    rclpy.init(args=None)
    try_params = ParameterUpdaterNode("controller_server")
    # params = try_params.get_params()
    # try_params.get_logger().info(" {}".format(params))

    params = {'FollowPath.xy_goal_tolerance': 0.3}
    try_params.set_params(params)